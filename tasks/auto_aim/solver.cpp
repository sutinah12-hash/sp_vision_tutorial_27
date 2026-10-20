#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;       // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;      // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;    // m
constexpr double SPECIAL_ARMOR_WIDTH = 129e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SPECIAL_ARMOR_POINTS{
  {0, SPECIAL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SPECIAL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SPECIAL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SPECIAL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

namespace
{
struct CameraPose
{
  cv::Vec3d rotation{}, translation{};
  double error = std::numeric_limits<double>::infinity();
};

bool finite_vector(const cv::Vec3d & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

CameraPose camera_pose(const std::vector<cv::Point3f> & object_points,
                       const std::vector<cv::Point2f> & image_points,
                       const cv::Mat & camera, const cv::Mat & distortion,
                       bool free_armor)
{
  CameraPose best;
  const auto consider = [&](const cv::Vec3d & rotation, const cv::Vec3d & translation) {
    if (!finite_vector(rotation) || !finite_vector(translation) ||
        translation[2] <= .05 || cv::norm(translation) > 30) return;
    cv::Matx33d matrix;
    cv::Rodrigues(rotation, matrix);
    if (free_armor) for (const auto & point : object_points) {
      const auto in_camera = matrix * cv::Vec3d(point.x, point.y, point.z) + translation;
      if (!finite_vector(in_camera) || in_camera[2] <= .05) return;
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rotation, translation, camera, distortion, projected);
    double error = 0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
      error += cv::norm(projected[i] - image_points[i]) / image_points.size();
    if (std::isfinite(error) && error < best.error) best = {rotation, translation, error};
  };
  // Keep the established vehicle-model input unchanged for tasks 2/3. The
  // free-pose refinement below is a task-1 correction, not an EKF retuning.
  if (!free_armor) {
    try {
      cv::Vec3d rotation, translation;
      if (cv::solvePnP(object_points, image_points, camera, distortion,
                       rotation, translation, false, cv::SOLVEPNP_IPPE))
        consider(rotation, translation);
    } catch (const cv::Exception &) {}
    return best;
  }
  const auto refine = [&](cv::Vec3d rotation, cv::Vec3d translation) {
    if (!finite_vector(rotation) || !finite_vector(translation)) return;
    // Keep the original solution if refinement fails or makes the residual worse.
    consider(rotation, translation);
    try {
      cv::solvePnPRefineLM(object_points, image_points, camera, distortion,
                          rotation, translation);
      consider(rotation, translation);
    } catch (const cv::Exception &) {}
  };
  try {
    std::vector<cv::Mat> rotations, translations;
    cv::solvePnPGeneric(object_points, image_points, camera, distortion,
                        rotations, translations, false, cv::SOLVEPNP_IPPE);
    for (std::size_t i = 0; i < rotations.size(); ++i) {
      const auto & r = rotations[i];
      const auto & t = translations[i];
      refine(cv::Vec3d(r.at<double>(0), r.at<double>(1), r.at<double>(2)),
             cv::Vec3d(t.at<double>(0), t.at<double>(1), t.at<double>(2)));
    }
  } catch (const cv::Exception &) {}
  // Some exactly frontal, symmetric poses are singular in older IPPE builds.
  // Iterative planar PnP is a fallback, subject to the same physical/error gates.
  if (!std::isfinite(best.error) || best.error >= 5.0) {
    try {
      cv::Vec3d rotation, translation;
      if (cv::solvePnP(object_points, image_points, camera, distortion,
                       rotation, translation, false, cv::SOLVEPNP_ITERATIVE))
        refine(rotation, translation);
    } catch (const cv::Exception &) {}
  }
  return best;
}
}

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = tools::load(config_path);

  auto R_gimbal2imubody_data = tools::read<std::vector<double>>(yaml, "R_gimbal2imubody");
  auto R_camera2gimbal_data = tools::read<std::vector<double>>(yaml, "R_camera2gimbal");
  auto t_camera2gimbal_data = tools::read<std::vector<double>>(yaml, "t_camera2gimbal");
  const auto validate_array = [](const std::vector<double> & values, std::size_t size,
                                 const char * name) {
    if (values.size() != size || !std::all_of(values.begin(), values.end(),
                                             [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument(std::string("Invalid calibration array: ") + name);
  };
  validate_array(R_gimbal2imubody_data, 9, "R_gimbal2imubody");
  validate_array(R_camera2gimbal_data, 9, "R_camera2gimbal");
  validate_array(t_camera2gimbal_data, 3, "t_camera2gimbal");
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());
  for (const auto & rotation : {R_gimbal2imubody_, R_camera2gimbal_}) {
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
        std::abs(rotation.determinant() - 1.0) > 1e-3)
      throw std::invalid_argument("Calibration rotation must belong to SO(3)");
  }

  auto camera_matrix_data = tools::read<std::vector<double>>(yaml, "camera_matrix");
  auto distort_coeffs_data = tools::read<std::vector<double>>(yaml, "distort_coeffs");
  validate_array(camera_matrix_data, 9, "camera_matrix");
  validate_array(distort_coeffs_data, 5, "distort_coeffs");
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  if (camera_matrix(0, 0) <= 0 || camera_matrix(1, 1) <= 0 ||
      std::abs(camera_matrix(2, 0)) > 1e-10 || std::abs(camera_matrix(2, 1)) > 1e-10 ||
      std::abs(camera_matrix(2, 2) - 1.0) > 1e-10)
    throw std::invalid_argument("Invalid pinhole camera matrix");
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  t_yaw2pitch_ = Eigen::Vector3d::Zero();
  if (yaml["t_yaw2pitch"]) {
    auto t_yaw2pitch_data = yaml["t_yaw2pitch"].as<std::vector<double>>();
    validate_array(t_yaw2pitch_data, 3, "t_yaw2pitch");
    t_yaw2pitch_ = Eigen::Vector3d(t_yaw2pitch_data.data());
  }
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  if (!q.coeffs().allFinite() || q.norm() < 1e-10 || !std::isfinite(q.norm()))
    throw std::invalid_argument("Invalid IMU quaternion");
  Eigen::Matrix3d R_imubody2imuabs = q.normalized().toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）
bool Solver::solve(Armor & armor, PoseMode mode) const
{
  armor.pose_valid = false;
  armor.raw_pose_valid = false;
  armor.reprojection_error = armor.pnp_reprojection_error =
    armor.model_reprojection_error = std::numeric_limits<double>::infinity();
  const auto reject = [&](const char * reason) {
    armor.pose_status = reason;
    return false;
  };
  if (armor.points.size() != 4) return reject("corner_count");
  for (const auto & point : armor.points)
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return reject("nonfinite_corner");
  if (!cv::isContourConvex(armor.points) || std::abs(cv::contourArea(armor.points)) < 4)
    return reject("invalid_quad");
  const auto & object_points = (armor.type == ArmorType::big)       ? BIG_ARMOR_POINTS
                               : (armor.type == ArmorType::special) ? SPECIAL_ARMOR_POINTS
                                                                    : SMALL_ARMOR_POINTS;

  const auto pose = camera_pose(object_points, armor.points, camera_matrix_, distort_coeffs_,
                                mode == PoseMode::FreeArmor);
  if (!std::isfinite(pose.error)) return reject("pnp_no_physical_solution");
  armor.reprojection_error = armor.pnp_reprojection_error = pose.error;
  if (mode == PoseMode::FreeArmor && pose.error >= 5.0) return reject("pnp_reprojection");
  const auto & rvec = pose.rotation;
  const auto & tvec = pose.translation;

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  double yaw = tools::eulers(R_gimbal2world_, 2, 1, 0)[0];
  Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal + R_yaw * t_yaw2pitch_;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);
  armor.yaw_raw = armor.ypr_in_world[0];
  armor.raw_pitch = armor.ypr_in_world[1];
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  if (!armor.xyz_in_world.allFinite() || !armor.ypr_in_world.allFinite() ||
      !armor.ypd_in_world.allFinite()) return reject("nonfinite_world_pose");
  armor.raw_pose_valid = true;
  if (mode == PoseMode::FreeArmor) {
    // A genuine PnP pose does not become invalid merely because a handheld
    // plate is not tilted by the vehicle model's fixed +/-15 degrees.
    armor.pose_valid = true;
    armor.pose_status = "accepted_free";
    return true;
  }
  // Only the three -15 degree middle panels belong to the outpost model.
  if (armor.name == ArmorName::outpost &&
      std::abs(tools::limit_rad(armor.raw_pitch + 15 * CV_PI / 180)) > 0.45)
    return reject("outpost_panel_tilt");

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) {
    if (pose.error >= 5.0) return reject("pnp_reprojection");
    armor.pose_valid = true;
    armor.pose_status = "accepted_balance";
    return true;
  }

  optimize_yaw(armor);
  armor.reprojection_error = armor_reprojection_error(armor, armor.ypr_in_world[0], 0) / 4.0;
  armor.model_reprojection_error = armor.reprojection_error;
  armor.pose_valid = armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() &&
                     std::isfinite(armor.reprojection_error) && armor.reprojection_error < 5.0;
  armor.pose_status = armor.pose_valid ? "accepted_model" : "vehicle_model_reprojection";
  return armor.pose_valid;
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;

  // 云台yaw
  double gimbal_yaw = tools::eulers(R_gimbal2world_, 2, 1, 0)[0];
  Eigen::Matrix3d R_gimbal_yaw =
    Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d t_armor2gimbal = t_armor2world - R_gimbal_yaw * t_yaw2pitch_;

  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() *
    (R_gimbal2world_.transpose() * t_armor2gimbal - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big)       ? BIG_ARMOR_POINTS
                               : (type == ArmorType::special) ? SPECIAL_ARMOR_POINTS
                                                              : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & /* inclined */) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  // Use exactly the same yaw-to-pitch translation as the single-point overload.
  std::vector<cv::Point2f> pixelPoints;
  for (const auto & world_point : worldPoints) {
    const auto pixel = world2pixel(Eigen::Vector3d(world_point.x, world_point.y, world_point.z));
    if (pixel) pixelPoints.push_back(*pixel);
  }
  return pixelPoints;
}

std::optional<cv::Point2f> Solver::world2pixel(const Eigen::Vector3d & world_point) const
{
  if (!world_point.allFinite()) return std::nullopt;
  double current_yaw = tools::eulers(R_gimbal2world_, 2, 1, 0)[0];
  Eigen::Matrix3d R_yaw =
    Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d gimbal_origin_offset = R_yaw * t_yaw2pitch_;

  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;
  Eigen::Vector3d camera_point =
    R_world2camera * (world_point - gimbal_origin_offset) + t_world2camera;

  if (!camera_point.allFinite() || camera_point.z() <= 1e-8) {
    return std::nullopt;
  }

  std::vector<cv::Point3f> camera_points{
    cv::Point3f(camera_point.x(), camera_point.y(), camera_point.z())};
  std::vector<cv::Point2f> pixel_points;
  cv::projectPoints(
    camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix_, distort_coeffs_,
    pixel_points);
  if (pixel_points.empty() || !std::isfinite(pixel_points[0].x) || !std::isfinite(pixel_points[0].y)) {
    return std::nullopt;
  }
  return pixel_points.front();
}

}  // namespace auto_aim
