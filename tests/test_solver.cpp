#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

namespace {
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
struct Fixture {
  std::filesystem::path path;
  Fixture() {
    char pattern[] = "/tmp/aim-solver-XXXXXX";
    const int fd = mkstemp(pattern);
    if (fd < 0) throw std::runtime_error("mkstemp");
    close(fd);
    path = pattern;
  }
  ~Fixture() { std::error_code ec; std::filesystem::remove(path, ec); }
  void save(const YAML::Node & node) const { std::ofstream(path) << node; }
};
}

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) throw std::invalid_argument("Pass sim/camera.yaml");
    const auto base = YAML::LoadFile(argv[1]);
    Fixture fixture;
    auto expect_bad_config = [&](const char * key, const std::vector<double> & values) {
      auto yaml = YAML::Clone(base);
      yaml[key] = values;
      fixture.save(yaml);
      bool rejected = false;
      try { auto_aim::Solver bad(fixture.path.string()); }
      catch (const std::exception &) { rejected = true; }
      check(rejected, "malformed calibration must fail before Eigen memory mapping");
    };
    expect_bad_config("camera_matrix", {1, 2});
    expect_bad_config("distort_coeffs", {0});
    expect_bad_config("t_yaw2pitch", {0, 1});
    expect_bad_config("R_camera2gimbal", {1,0,0,0,1,0,0,0,-1});
    expect_bad_config("t_camera2gimbal", {0,0,std::numeric_limits<double>::quiet_NaN()});
    expect_bad_config("camera_matrix", {0,0,720,0,1200,540,0,0,1});
    auto yaml = YAML::Clone(base);
    yaml["t_camera2gimbal"] = std::vector<double>{.2,.03,.05};
    yaml["t_yaw2pitch"] = std::vector<double>{.12,.01,.02};
    fixture.save(yaml);
    auto_aim::Solver solver(fixture.path.string());
    const Eigen::Quaterniond q(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond scaled_q = q;
    scaled_q.coeffs() *= 2;
    solver.set_R_gimbal2world(scaled_q);
    check((solver.R_gimbal2world() - q.toRotationMatrix()).norm() < 1e-12,
          "IMU quaternion normalization");
    bool rejected_q = false;
    try { solver.set_R_gimbal2world(Eigen::Quaterniond(0,0,0,0)); }
    catch (const std::invalid_argument &) { rejected_q = true; }
    check(rejected_q, "zero quaternion rejected");
    const Eigen::Vector3d camera_point(.2,-.1,3);
    const Eigen::Matrix3d camera_to_gimbal = (Eigen::Matrix3d() <<
      0,0,1,-1,0,0,0,-1,0).finished();
    const Eigen::Vector3d world = q * (camera_to_gimbal * camera_point +
      Eigen::Vector3d(.2,.03,.05) + Eigen::Vector3d(.12,.01,.02));
    const auto pixel = solver.world2pixel(world);
    check(pixel && std::abs(pixel->x - 800) < 1e-3 &&
          std::abs(pixel->y - 500) < 1e-3, "analytic projection with nonzero axis offsets");
    const auto batch = solver.world2pixel(std::vector<cv::Point3f>{
      {float(world.x()),float(world.y()),float(world.z())}});
    check(batch.size() == 1 && cv::norm(batch[0] - *pixel) < 1e-3,
          "single and batch projection agree with nonzero yaw/pitch offset");
    check(!solver.world2pixel(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
          "nonfinite point cannot be rendered");
    for (const auto type : {auto_aim::small, auto_aim::special, auto_aim::big}) {
      auto_aim::Armor armor;
      armor.type = type;
      armor.name = type == auto_aim::special ? auto_aim::outpost : auto_aim::three;
      armor.points = solver.reproject_armor(world, .5, armor.type, armor.name);
      check(solver.solve(armor) && armor.pose_valid, "PnP valid armor");
      check((armor.xyz_in_world - world).norm() < 1e-3, "PnP roundtrip translation");
      check(armor.reprojection_error < .05, "all armor types enforce reprojection");
    }
    auto_aim::Armor bad;
    bad.points = {{0,0},{10,10},{0,10},{10,0}};
    check(!solver.solve(bad), "crossed corner order rejected");
    bad.points = {{0,0},{1,0},{1,1},{0,1}};
    check(!solver.solve(bad), "tiny contour rejected");
    check(std::isnan(tools::limit_rad(std::numeric_limits<double>::infinity())),
          "infinite angle returns NaN without hanging");
    check(std::abs(tools::limit_rad(1e100)) <= CV_PI, "huge finite angle bounded");
    check(tools::limit_rad(-CV_PI) == CV_PI, "preserve angular endpoint convention");
    check(tools::eulers(Eigen::Quaterniond::Identity(),2,1,0).allFinite(),
          "identity Euler finite");
    std::cout << "PASS calibration validation, IMU, PnP, projection offsets and angle edges\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
