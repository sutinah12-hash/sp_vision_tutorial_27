#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include "tasks/auto_aim/target.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/math_tools.hpp"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

auto_aim::Armor observation(const Eigen::Vector3d & xyz, double yaw, bool outpost = false)
{
  auto_aim::Armor result;
  result.name = outpost ? auto_aim::outpost : auto_aim::four;
  result.type = outpost ? auto_aim::special : auto_aim::small;
  result.color = auto_aim::blue;
  result.xyz_in_world = xyz;
  result.ypr_in_world = {std::remainder(yaw, 2.0 * CV_PI), 0.0, 0.0};
  result.ypd_in_world = tools::xyz2ypd(xyz);
  return result;
}

void test_filter()
{
  for (int dimension : {1, 3, 4, 5}) {
    tools::ExtendedKalmanFilter filter(
      Eigen::VectorXd::Zero(dimension), Eigen::MatrixXd::Identity(dimension, dimension));
    const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(dimension, dimension);
    filter.nis_gate = 20.0;
    filter.update(Eigen::VectorXd::Ones(dimension), H, H);
    require(filter.last_update_accepted, "valid EKF observation rejected");
    require(std::abs(filter.last_nis - 0.5 * dimension) < 1e-10, "NIS must use prior covariance");
    require((filter.x.array() - 0.5).abs().maxCoeff() < 1e-10, "Kalman gain scalar check");
    require(filter.data.count("nees") == 0, "NEES must not be fabricated without truth");
    const Eigen::VectorXd before = filter.x;
    const Eigen::MatrixXd covariance_before = filter.P;
    filter.update(Eigen::VectorXd::Constant(dimension, 1000), H, H);
    require(!filter.last_update_accepted && filter.x.isApprox(before) &&
      filter.P.isApprox(covariance_before), "rejected innovation changed state");
    filter.update(before, H, H);
    require(filter.data.at("nis_fail") == 0.0, "NIS failure flag did not reset");
    require(filter.P.isApprox(filter.P.transpose(), 1e-12), "Joseph covariance asymmetry");
    require(filter.P.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff() >= -1e-12,
      "Joseph covariance not PSD");
    auto invalid = before;
    invalid[0] = std::numeric_limits<double>::quiet_NaN();
    filter.update(invalid, H, H);
    require(!filter.last_update_accepted && filter.x.allFinite(), "NaN observation contaminated state");
  }
}

void test_boundaries()
{
  const auto t0 = std::chrono::steady_clock::time_point{};
  auto armor = observation({3.0, 0.0, 0.3}, 0.0);
  auto_aim::Target target(armor, t0, 0.2, 2);
  require(!target.is_converged(), "initial radius alone must not imply convergence");
  require(target.armor_xyza_list().size() == 2, "two-plate geometry size");
  for (int i = 0; i < 100; ++i) {
    target.predict(0.01);
    require(target.update(armor), "two-plate update rejected");
  }
  const auto prior = target.ekf_x();
  bool rejected_time = false;
  try { target.predict(-0.01); } catch (const std::invalid_argument &) { rejected_time = true; }
  require(rejected_time && target.ekf_x().isApprox(prior), "negative dt changed state");
  rejected_time = false;
  try { target.predict(std::numeric_limits<double>::infinity()); }
  catch (const std::invalid_argument &) { rejected_time = true; }
  require(rejected_time, "infinite dt accepted");
  auto invalid = armor;
  invalid.xyz_in_world[0] = std::numeric_limits<double>::quiet_NaN();
  require(!target.update(invalid) && target.ekf_x().allFinite(), "invalid armor poisoned state");
  invalid = armor;
  invalid.xyz_in_world[0] = 30.0;
  require(!target.update(invalid), "large spatial outlier accepted");
  auto_aim::Target uninitialized;
  require(uninitialized.diverged() && uninitialized.armor_xyza_list().empty(), "default target safety");
}

void test_jacobians()
{
  const auto t0 = std::chrono::steady_clock::time_point{};
  for (bool outpost : {false, true}) {
    auto_aim::Target target(observation({3.0, 0.2, 0.4}, 0.3, outpost), t0);
    for (int id = 0; id < target.armor_count(); ++id) {
      const auto state = target.ekf_x();
      const auto analytic = target.observation_jacobian(id);
      const auto model = [&](const Eigen::VectorXd & x) {
        const double yaw = x[6] + id * 2.0 * CV_PI / target.armor_count();
        const double r = x[8] + (!outpost && id % 2 ? x[9] : 0.0);
        const double dh = outpost ? (id == 1 ? x[9] : id == 2 ? x[10] : 0.0)
                                  : (id % 2 ? x[10] : 0.0);
        const Eigen::Vector3d p(x[0] - r * std::cos(yaw), x[2] - r * std::sin(yaw), x[4] + dh);
        const auto ypd = tools::xyz2ypd(p);
        return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], yaw);
      };
      for (int column = 0; column < 11; ++column) {
        auto plus = state, minus = state;
        plus[column] += 1e-6;
        minus[column] -= 1e-6;
        const Eigen::Vector4d numerical = (model(plus) - model(minus)) / 2e-6;
        require((numerical - analytic.col(column)).norm() < 1e-5,
          "observation Jacobian finite-difference mismatch");
      }
    }
  }
}

void test_rotation(double omega, bool outpost, int initial_layer)
{
  constexpr double dt = 1.0 / 60.0;
  constexpr int frames = 720;
  const int count = outpost ? 3 : 4;
  const double phase0 = -initial_layer * 2.0 * CV_PI / count;
  std::mt19937 random(734 + initial_layer);
  std::normal_distribution<double> position_noise(0.0, 0.003);
  std::normal_distribution<double> angle_noise(0.0, 0.01);
  const auto center = [&](double t) {
    return Eigen::Vector3d(3.5 + (outpost ? 0.0 : 0.06 * t),
      0.15 + (outpost ? 0.0 : 0.04 * t), outpost ? 1.116 : 0.28);
  };
  const auto plate = [&](int id, double t) {
    const double phase = phase0 + omega * t + id * 2.0 * CV_PI / count;
    const double r = outpost ? 0.2765 : (id % 2 ? 0.30 : 0.24);
    Eigen::Vector3d xyz = center(t) - Eigen::Vector3d(r * std::cos(phase), r * std::sin(phase), 0.0);
    xyz[2] += outpost ? 0.1 * id : (id % 2 ? 0.08 : 0.0);
    return observation(xyz, phase, outpost);
  };
  const auto visible = [&](double t) {
    int selected = 0;
    double score = 10.0;
    for (int id = 0; id < count; ++id) {
      const auto truth = plate(id, t);
      const double error = std::abs(std::remainder(truth.ypr_in_world[0] -
        std::atan2(truth.xyz_in_world[1], truth.xyz_in_world[0]), 2.0 * CV_PI));
      if (error < score) { score = error; selected = id; }
    }
    return selected;
  };
  const int first_id = visible(0.0);
  const auto t0 = std::chrono::steady_clock::time_point{};
  auto_aim::Target target(plate(first_id, 0.0), t0);  // No truth omega or plate id enters Target.
  for (int frame = 1; frame <= frames; ++frame) {
    const double t = frame * dt;
    auto measured = plate(visible(t), t);
    for (int axis = 0; axis < 3; ++axis) measured.xyz_in_world[axis] += position_noise(random);
    measured.ypr_in_world[0] += angle_noise(random);
    target.predict(t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(t)));
    if (frame % 79 == 0) continue;  // Occasional missing observation is not a truth update.
    const bool accepted = target.update(measured);
    if (!accepted && frame < 30)
      std::cout << "early reject frame=" << frame << " nis=" << target.ekf().last_nis
        << " state=" << target.ekf_x().transpose() << '\n';
    require(!target.diverged(), "rotation scenario diverged");
  }
  const double t = frames * dt;
  const auto x = target.ekf_x();
  const double center_error = Eigen::Vector2d(x[0] - center(t)[0], x[2] - center(t)[1]).norm();
  const auto armors = target.armor_xyza_list();
  double plate_error = 0.0;
  for (int id = 0; id < count; ++id) {
    plate_error = std::max(plate_error, (armors[id].head<3>() - plate((first_id + id) % count, t).xyz_in_world).norm());
  }
  std::cout << "rotation omega=" << omega << " outpost=" << outpost << " first_layer="
    << initial_layer << " accepted=" << target.accepted_updates() << " rejected="
    << target.rejected_updates() << " center_error=" << center_error << " plate_error="
    << plate_error << " omega_error=" << std::abs(x[7] - omega) << " converged="
    << target.is_converged() << '\n';
  require(target.accepted_updates() > frames * 0.90, "too many legitimate observations rejected");
  require(std::abs(x[7] - omega) < 0.25, "angular speed did not converge from zero");
  require(center_error < 0.07 && plate_error < 0.08, "geometry estimate outside metric tolerance");
  require(target.is_converged(), "stable observed rotation never declared converged");
  auto predicted = target;
  predicted.predict(0.2);
  const auto future = predicted.armor_xyza_list();
  double future_error = 0.0;
  for (int id = 0; id < count; ++id)
    future_error = std::max(future_error,
      (future[id].head<3>() - plate((first_id + id) % count, t + 0.2).xyz_in_world).norm());
  require(future_error < 0.10, "200ms predictive geometry outside tolerance");
  require(!predicted.is_converged(), "stale observation incorrectly remained fire-ready");
}

void test_outliers_and_gaps(double omega)
{
  constexpr double dt = 1.0 / 60.0;
  const auto t0 = std::chrono::steady_clock::time_point{};
  std::mt19937 random(104);
  std::normal_distribution<double> position_noise(0.0, 0.008);
  std::normal_distribution<double> angle_noise(0.0, 0.025);
  const auto measure = [&](double t) {
    int selected = 0;
    double best = 10.0;
    for (int id = 0; id < 4; ++id) {
      const double yaw = omega * t + id * CV_PI / 2.0;
      const double error = std::abs(std::remainder(yaw, 2.0 * CV_PI));
      if (error < best) { best = error; selected = id; }
    }
    const double angle = omega * t + selected * CV_PI / 2.0;
    const double radius = selected % 2 ? 0.30 : 0.24;
    return observation({3.5 - radius * std::cos(angle), -radius * std::sin(angle),
      0.3 + (selected % 2 ? 0.08 : 0.0)}, angle);
  };
  auto_aim::Target target(measure(0.0), t0);
  int injected = 0, correctly_rejected = 0;
  bool stale_checked = false;
  for (int frame = 1; frame <= 840; ++frame) {
    const double t = frame * dt;
    target.predict(t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(t)));
    // Two 0.3 s outages remain below the controller's separate 0.45 s reset policy.
    if ((frame >= 180 && frame <= 198) || (frame >= 420 && frame <= 438)) {
      if (frame == 198 || frame == 438) {
        require(!target.is_converged(), "observation timeout did not withdraw readiness");
        stale_checked = true;
      }
      continue;
    }
    auto measured = measure(t);
    for (int axis = 0; axis < 3; ++axis) measured.xyz_in_world[axis] += position_noise(random);
    measured.ypr_in_world[0] += angle_noise(random);
    const bool bad_position = frame > 120 && frame % 37 == 0;
    const bool bad_orientation = frame > 120 && frame % 61 == 0;
    if (bad_position) measured.xyz_in_world[0] += 10.0;
    else if (bad_orientation) measured.ypr_in_world[0] += CV_PI / 2;
    const auto before = target.ekf_x();
    const auto covariance = target.ekf().P;
    const bool accepted = target.update(measured);
    if (bad_position || bad_orientation) {
      ++injected;
      if (!accepted) ++correctly_rejected;
      require(!accepted, "injected position/orientation outlier accepted");
      require(target.ekf_x().isApprox(before) && target.ekf().P.isApprox(covariance),
        "outlier changed the estimator state");
    }
    require(!target.diverged(), "noise/dropout stress scenario diverged");
  }
  const auto state = target.ekf_x();
  const double center_error = Eigen::Vector2d(state[0] - 3.5, state[2]).norm();
  std::cout << "stress omega=" << omega << " outliers=" << injected << " rejected="
    << correctly_rejected << " center_error=" << center_error << " omega_error="
    << std::abs(state[7] - omega) << '\n';
  require(stale_checked, "outage scenario did not exercise freshness");
  if (omega != 0.0)
    require(target.is_converged(), "track failed to recover after a brief outage");
  else
    require(!target.is_converged(), "unobserved geometry must not be declared fully converged");
  require(center_error < 0.07 && std::abs(state[7] - omega) < 0.25,
    "noise/dropout stress recovery outside tolerance");
}
}  // namespace

int main()
{
  try {
    test_filter();
    test_boundaries();
    test_jacobians();
    for (double speed : {-9.0, -6.0, -3.0, 3.0, 6.0, 9.0}) test_rotation(speed, false, 0);
    for (double speed : {-2.5, 2.5})
      for (int layer = 0; layer < 3; ++layer) test_rotation(speed, true, layer);
    for (double speed : {-9.0, -3.0, 0.0, 3.0, 9.0}) test_outliers_and_gaps(speed);
    std::cout << "All Target/EKF checks passed.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
