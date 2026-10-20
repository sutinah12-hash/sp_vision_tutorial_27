#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "tasks/auto_aim/controller.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

namespace
{
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

// Independently project a freely held plate; do not generate the observations
// with Solver::reproject_armor, which assumes the vehicle's fixed plate tilt.
auto_aim::Armor project_plate(const Eigen::Vector3d & center,
                             const Eigen::Matrix3d & plate_to_world,
                             const Eigen::Matrix3d & gimbal_to_world)
{
  const Eigen::Matrix3d camera_to_gimbal =
    (Eigen::Matrix3d() << 0,0,1, -1,0,0, 0,-1,0).finished();
  const Eigen::Matrix3d world_to_camera =
    camera_to_gimbal.transpose() * gimbal_to_world.transpose();
  const Eigen::Matrix3d rotation = world_to_camera * plate_to_world;
  const Eigen::Vector3d translation = world_to_camera * center;
  cv::Mat rotation_cv;
  cv::eigen2cv(rotation, rotation_cv);
  cv::Vec3d rvec;
  cv::Rodrigues(rotation_cv, rvec);
  const cv::Mat camera = (cv::Mat_<double>(3,3) <<
    1200,0,720, 0,1200,540, 0,0,1);
  const std::vector<cv::Point3f> corners{
    {0,.0675f,.028f}, {0,-.0675f,.028f},
    {0,-.0675f,-.028f}, {0,.0675f,-.028f}};
  auto_aim::Armor armor;
  armor.color = auto_aim::blue;
  armor.name = auto_aim::four;
  armor.type = auto_aim::small;
  armor.confidence = .99;
  cv::projectPoints(corners, rvec,
    cv::Vec3d(translation.x(), translation.y(), translation.z()),
    camera, cv::Mat::zeros(1,5,CV_64F), armor.points);
  return armor;
}
}

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "Pass sim/camera.yaml");
    auto_aim::Solver solver(argv[1]);
    int cases = 0, rejected = 0;
    double worst_position_error = 0.0;
    // Distances and attitudes are test samples, not operating requirements.
    for (double distance : {.8, 1.5, 3.0, 6.0}) {
      for (double heading : {-3.15, -.4, 0.0, .4, 3.15}) {
        const Eigen::Matrix3d attitude =
          tools::rotation_matrix(Eigen::Vector3d(heading, .12, -.04));
        solver.set_R_gimbal2world(Eigen::Quaterniond(attitude));
        const Eigen::Vector3d center = attitude * Eigen::Vector3d(distance, .04, -.02);
        for (double relative_yaw : {-.5, 0.0, .5}) {
          for (double pitch_deg : {-30.0, -10.0, 0.0, 15.0, 30.0}) {
            for (double roll_deg : {-30.0, -15.0, 0.0, 15.0, 30.0}) {
              const Eigen::Matrix3d plate = tools::rotation_matrix(Eigen::Vector3d(
                heading + relative_yaw, pitch_deg * CV_PI / 180, roll_deg * CV_PI / 180));
              auto armor = project_plate(center, plate, attitude);
              ++cases;
              if (!solver.solve(armor, auto_aim::PoseMode::FreeArmor)) { ++rejected; continue; }
              require(armor.raw_pose_valid && armor.pose_valid, "raw/free pose flags");
              require(armor.pnp_reprojection_error < .05, "exact corners must reproject accurately");
              worst_position_error = std::max(worst_position_error,
                (armor.xyz_in_world - center).norm());
            }
          }
        }
      }
    }
    std::cout << "Free-plate poses: cases=" << cases << " rejected=" << rejected
              << " worst_position_error_m=" << worst_position_error << '\n';
    require(rejected == 0, "valid freely tilted plates must remain usable for task 1");
    require(worst_position_error < .002, "free-plate position round trip");

    // Solve -> world coordinates -> controller: both directions, including
    // headings across the +/-pi seam. These checks never touch hardware.
    int direction_cases = 0;
    for (double heading : {-3.15, 0.0, 3.15}) {
      const Eigen::Matrix3d attitude = tools::rotation_matrix({heading, 0, 0});
      solver.set_R_gimbal2world(Eigen::Quaterniond(attitude));
      for (double distance : {.8, 3.0, 6.0}) {
        for (double side : {-.2, .2}) {
          auto armor = project_plate(attitude * Eigen::Vector3d(distance, side, 0),
            tools::rotation_matrix({heading, -.3, .25}), attitude);
          require(solver.solve(armor, auto_aim::PoseMode::FreeArmor), "direction test pose");
          auto_aim::ControllerConfig cfg;
          cfg.task = 1;
          cfg.enemy = auto_aim::blue;
          cfg.allow_fire = true;  // Task 1 must ignore even an armed caller.
          auto_aim::Controller controller(cfg);
          auto_aim::Feedback feedback;
          feedback.yaw = tools::limit_rad(heading);
          feedback.fire_enabled = true;
          const auto time = auto_aim::AimTime{} + std::chrono::seconds(1);
          const auto command = controller.step({armor}, time, time, feedback);
          require(command.control && !command.fire, "task 1 controls but never fires");
          const double delta = tools::limit_rad(command.yaw - feedback.yaw);
          require(delta * side > 0, "left/right sign survives projection, IMU and yaw wrapping");
          require(std::abs(delta) <= cfg.yaw_rate_limit * .01 + 1e-9,
            "existing yaw slew limit is retained");
          ++direction_cases;
        }
      }
    }
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    auto tilted = project_plate({1.0,0,0}, tools::rotation_matrix({0,-.4,.4}), identity);
    require(solver.solve(tilted, auto_aim::PoseMode::FreeArmor), "arbitrary tilt is valid for task 1");
    require(!solver.solve(tilted) && tilted.raw_pose_valid && !tilted.pose_valid &&
      tilted.pose_status == "vehicle_model_reprojection", "task 2/3 still reject nonvehicle geometry");
    auto frontal = project_plate({2.0,0,0}, identity, identity);
    require(solver.solve(frontal, auto_aim::PoseMode::FreeArmor), "exactly frontal planar pose");
    frontal.points = {{0,0},{10,10},{0,10},{10,0}};
    require(!solver.solve(frontal, auto_aim::PoseMode::FreeArmor) && !frontal.raw_pose_valid &&
      !frontal.pose_valid && frontal.pose_status == "invalid_quad", "reused pose is invalidated");
    auto too_far = project_plate({40.0,0,0}, identity, identity);
    require(!solver.solve(too_far, auto_aim::PoseMode::FreeArmor), "30 m physical sanity gate retained");
    std::cout << "PASS " << direction_cases << " bidirectional/no-fire cases and pose rejection gates\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
