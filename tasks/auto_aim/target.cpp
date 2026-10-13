#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr double kOutpostRadius = 0.2765;
constexpr double kInnovationGate = 18.46682695;  // chi-square(4), upper 99.9% quantile.
double wrap(double value) { return std::remainder(value, 2.0 * CV_PI); }

bool valid_armor(const Armor & armor)
{
  return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() &&
         armor.xyz_in_world.head<2>().norm() > 0.1 && armor.xyz_in_world.norm() < 100.0;
}

Eigen::VectorXd residual(const Eigen::VectorXd & observed, const Eigen::VectorXd & expected)
{
  Eigen::VectorXd result = observed - expected;
  result[0] = wrap(result[0]);
  result[1] = wrap(result[1]);
  result[3] = wrap(result[3]);
  return result;
}
}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num)
: name(armor.name), armor_type(armor.type), priority(armor.priority),
  armor_num_(armor.name == outpost ? 3 : armor_num), t_(t), initial_t_(t), last_observation_t_(t)
{
  if (!valid_armor(armor) || (armor_num_ != 2 && armor_num_ != 3 && armor_num_ != 4))
    throw std::invalid_argument("Target requires a valid pose and 2, 3 or 4 armor plates");
  const double r = is_layered_outpost() ? kOutpostRadius : radius;
  if (!std::isfinite(r) || r <= 0.05 || r >= 0.5)
    throw std::invalid_argument("Target radius must lie in (0.05, 0.5) metres");
  const double yaw = wrap(armor.ypr_in_world[0]);
  const auto & xyz = armor.xyz_in_world;
  Eigen::VectorXd x0(11);
  // [xc,vx,yc,vy,z0,vz,theta,omega,r,dr,dh] for vehicles;
  // [xc,vx,yc,vy,z0,vz,theta,omega,r,z1-z0,z2-z0] for the layered outpost.
  // The first observed plate defines phase zero. Its physical layer is unknown.
  x0 << xyz[0] + r * std::cos(yaw), 0.0, xyz[1] + r * std::sin(yaw), 0.0,
    xyz[2], 0.0, yaw, 0.0, r, 0.0, 0.0;
  Eigen::VectorXd variances(11);
  // Conservative starting priors (SI units), validated on the included noisy scenarios.
  // These are not a substitute for covariance calibration on the assessment robot.
  variances << 0.0625, 1.0, 0.0625, 1.0, 0.01, 0.04, 0.09, 64.0, 0.0064, 0.0064, 0.0225;
  if (is_layered_outpost()) {
    // A stationary centre and equal known radii are structural priors, not rotation truth.
    variances << 0.0225, 0.0, 0.0225, 0.0, 0.01, 0.0, 0.09, 64.0, 0.0, 0.04, 0.04;
    geometry_enabled_ = true;  // The unknown layer offsets must learn from the first switch.
  } else if (armor_num_ != 4) {
    variances[9] = variances[10] = 0.0;
  }
  const auto add = [](const Eigen::VectorXd & state, const Eigen::VectorXd & increment) {
    Eigen::VectorXd result = state + increment;
    result[6] = wrap(result[6]);
    return result;
  };
  ekf_ = tools::ExtendedKalmanFilter(x0, variances.asDiagonal(), add);
  ekf_.nis_gate = kInnovationGate;
  seen_ids_.assign(armor_num_, 0);
  seen_ids_[0] = 1;
  isinit = true;
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  const double dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  if (!isinit) throw std::logic_error("Cannot predict an uninitialized Target");
  if (!std::isfinite(dt) || dt < 0.0 || dt > 5.0)
    throw std::invalid_argument("Target prediction requires finite dt in [0, 5] seconds");
  if (dt == 0.0) return;
  // Substeps keep covariance propagation well-conditioned across short observation gaps.
  const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.05)));
  const double h = dt / steps;
  for (int step = 0; step < steps; ++step) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    const auto add_acceleration = [&](int position, double variance) {
      F(position, position + 1) = h;
      Eigen::Vector2d G(0.5 * h * h, h);
      Q.block<2, 2>(position, position) = variance * G * G.transpose();
    };
    if (!is_layered_outpost()) {
      add_acceleration(0, 9.0);
      add_acceleration(2, 9.0);
      add_acceleration(4, 0.04);
    } else {
      Q(0, 0) = Q(2, 2) = Q(4, 4) = 1e-8 * h;
    }
    add_acceleration(6, is_layered_outpost() ? 0.25 : 16.0);
    // Small geometry random walks allow adaptation without forcing a known test radius/layer.
    if (geometry_enabled_ && !is_layered_outpost()) Q(8, 8) = 1e-7 * h;
    if (armor_num_ == 4 || is_layered_outpost()) {
      Q(9, 9) = Q(10, 10) = geometry_enabled_ ? 1e-7 * h : 0.0;
    }
    ekf_.predict(F, Q, [&](const Eigen::VectorXd & state) {
      Eigen::VectorXd result = F * state;
      result[6] = wrap(result[6]);
      return result;
    });
  }
  t_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
  update_convergency();
}

Eigen::Matrix4d Target::observation_noise(const Armor & armor) const
{
  // Baseline PnP noise model; tune from held-out real-camera residuals, not simulation truth.
  const double d = armor.xyz_in_world.norm();
  const double incidence = std::abs(wrap(
    armor.ypr_in_world[0] - std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0])));
  const double distance_sigma = 0.025 + 0.002 * d * d;
  const double orientation_sigma = 0.035 + 0.007 * d + 0.025 * incidence;
  return Eigen::Vector4d(
    9e-6, 9e-6, distance_sigma * distance_sigma,
    orientation_sigma * orientation_sigma).asDiagonal();
}

bool Target::update(const Armor & armor)
{
  const auto reject = [&]() {
    ++rejected_count_;
    ++consecutive_rejections_;
    update_convergency();
    return false;
  };
  if (!isinit || !valid_armor(armor) || armor.name != name || armor.type != armor_type || diverged())
    return reject();
  const Eigen::Vector3d ypd = tools::xyz2ypd(armor.xyz_in_world);
  const Eigen::Vector4d z(ypd[0], ypd[1], ypd[2], wrap(armor.ypr_in_world[0]));
  const Eigen::Matrix4d R = observation_noise(armor);
  int selected = -1;
  double best_nis = std::numeric_limits<double>::infinity();
  // Enumerate the actual count: two-plate targets must never read a third candidate.
  for (int id = 0; id < armor_num_; ++id) {
    const Eigen::Vector4d expected = h_observation(ekf_.x, id);
    const Eigen::VectorXd innovation = residual(z, expected);
    if (std::abs(innovation[3]) > std::min(1.3, 1.5 * CV_PI / armor_num_)) continue;
    if ((armor.xyz_in_world - h_armor_xyz(ekf_.x, id)).norm() > 1.5) continue;
    const Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
    const Eigen::Matrix4d S = H * ekf_.P * H.transpose() + R;
    const Eigen::LDLT<Eigen::Matrix4d> factor(S);
    if (factor.info() != Eigen::Success || factor.vectorD().minCoeff() <= 1e-12) continue;
    const double nis = innovation.dot(factor.solve(innovation));
    if (std::isfinite(nis) && nis < best_nis) {
      best_nis = nis;
      selected = id;
    }
  }
  if (selected < 0) return reject();
  const bool geometry_was_enabled = geometry_enabled_;
  // The first plate may disappear within only a few frames at high spin rates.
  // Waiting for many accepted frames here would reject the new height forever.
  if (!geometry_enabled_ && update_count_ >= 2 && selected != 0) geometry_enabled_ = true;
  if (!update_ypda(armor, selected)) {
    geometry_enabled_ = geometry_was_enabled;
    return reject();
  }
  if (selected != last_id) ++switch_count_;
  jumped = jumped || selected != 0;
  last_id = selected;
  ++seen_ids_[selected];
  ++update_count_;
  consecutive_rejections_ = 0;
  last_observation_t_ = t_;
  if (!geometry_enabled_ && update_count_ >= 12 && switch_count_ > 0) geometry_enabled_ = true;
  update_convergency();
  return true;
}

bool Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // With only the first short arc, radius and centre translation are poorly observable.
  // No cross-covariance is created while these columns are disabled, so geometry stays at its prior.
  if (!geometry_enabled_) H.block(0, 8, 4, 3).setZero();
  const Eigen::Vector3d ypd = tools::xyz2ypd(armor.xyz_in_world);
  const Eigen::Vector4d z(ypd[0], ypd[1], ypd[2], wrap(armor.ypr_in_world[0]));
  const auto before = ekf_;
  ekf_.update(z, H, observation_noise(armor),
    [&](const Eigen::VectorXd & state) { return h_observation(state, id); }, residual);
  if (!ekf_.last_update_accepted) return false;
  if (diverged()) {
    ekf_ = before;  // Reject physically impossible geometry without corrupting the track.
    ekf_.last_update_accepted = false;
    return false;
  }
  return true;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }
const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> result;
  if (!isinit) return result;
  for (int id = 0; id < armor_num_; ++id) {
    const auto p = h_armor_xyz(ekf_.x, id);
    result.emplace_back(p[0], p[1], p[2], wrap(ekf_.x[6] + id * 2.0 * CV_PI / armor_num_));
  }
  return result;
}

Eigen::Vector4d Target::predicted_observation(int id) const
{
  if (!isinit || id < 0 || id >= armor_num_) throw std::out_of_range("Target armor index");
  return h_observation(ekf_.x, id);
}
Eigen::MatrixXd Target::observation_jacobian(int id) const
{
  if (!isinit || id < 0 || id >= armor_num_) throw std::out_of_range("Target armor index");
  return h_jacobian(ekf_.x, id);
}

bool Target::diverged() const
{
  if (!isinit || ekf_.x.size() != 11 || !ekf_.x.allFinite() || !ekf_.P.allFinite()) return true;
  const auto & x = ekf_.x;
  if ((ekf_.P.diagonal().array() < -1e-9).any() || std::abs(x[0]) > 100.0 ||
      std::abs(x[2]) > 100.0 || std::abs(x[4]) > 20.0 || std::abs(x[7]) > 40.0 ||
      x[8] <= 0.05 || x[8] >= 0.5) return true;
  if (is_layered_outpost()) {
    return std::abs(x[8] - kOutpostRadius) > 1e-6 || std::abs(x[9]) > 0.30 ||
           std::abs(x[10]) > 0.30 || std::abs(x[9] - x[10]) > 0.30;
  }
  return (armor_num_ == 4 && (x[8] + x[9] <= 0.05 || x[8] + x[9] >= 0.5 ||
    std::abs(x[10]) > 0.30)) || std::abs(x[1]) > 20.0 || std::abs(x[3]) > 20.0;
}

void Target::update_convergency()
{
  is_converged_ = false;
  if (diverged() || update_count_ < 30 || consecutive_rejections_ >= 3 ||
      tools::delta_time(t_, initial_t_) < 0.4 || tools::delta_time(t_, last_observation_t_) > 0.15)
    return;
  const auto & P = ekf_.P;
  if (P(0, 0) > 0.04 || P(2, 2) > 0.04 || P(4, 4) > 0.0025 ||
      P(6, 6) > 0.025 || P(7, 7) > 0.36) return;
  if (is_layered_outpost()) {
    if (std::any_of(seen_ids_.begin(), seen_ids_.end(), [](int n) { return n < 3; }) ||
        P(9, 9) > 0.0016 || P(10, 10) > 0.0016) return;
  } else if (armor_num_ == 4) {
    if (seen_ids_[0] + seen_ids_[2] < 3 || seen_ids_[1] + seen_ids_[3] < 3 ||
        P(8, 8) > 0.0036 || P(9, 9) > 0.0064 || P(10, 10) > 0.0025) return;
  }
  is_converged_ = true;
}

bool Target::is_converged() const { return is_converged_; }
bool Target::convergened() { update_convergency(); return is_converged(); }
bool Target::checkinit() { return isinit; }
bool Target::is_layered_outpost() const { return name == outpost && armor_num_ == 3; }

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  const double angle = wrap(x[6] + id * 2.0 * CV_PI / armor_num_);
  const bool alternate = armor_num_ == 4 && id % 2 == 1;
  const double radius = x[8] + (alternate ? x[9] : 0.0);
  double height = alternate ? x[10] : 0.0;
  if (is_layered_outpost()) height = id == 1 ? x[9] : id == 2 ? x[10] : 0.0;
  return {x[0] - radius * std::cos(angle), x[2] - radius * std::sin(angle), x[4] + height};
}

Eigen::Vector4d Target::h_observation(const Eigen::VectorXd & x, int id) const
{
  const Eigen::Vector3d ypd = tools::xyz2ypd(h_armor_xyz(x, id));
  return {ypd[0], ypd[1], ypd[2], wrap(x[6] + id * 2.0 * CV_PI / armor_num_)};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  const double angle = wrap(x[6] + id * 2.0 * CV_PI / armor_num_);
  const bool alternate = armor_num_ == 4 && id % 2 == 1;
  const double radius = x[8] + (alternate ? x[9] : 0.0);
  Eigen::MatrixXd geometry = Eigen::MatrixXd::Zero(4, 11);
  geometry(0, 0) = geometry(1, 2) = geometry(2, 4) = geometry(3, 6) = 1.0;
  geometry(0, 6) = radius * std::sin(angle);
  geometry(1, 6) = -radius * std::cos(angle);
  geometry(0, 8) = -std::cos(angle);
  geometry(1, 8) = -std::sin(angle);
  if (alternate) {
    geometry(0, 9) = -std::cos(angle);
    geometry(1, 9) = -std::sin(angle);
    geometry(2, 10) = 1.0;
  }
  if (is_layered_outpost() && id != 0) geometry(2, 8 + id) = 1.0;
  Eigen::Matrix4d spherical = Eigen::Matrix4d::Identity();
  spherical.block<3, 3>(0, 0) = tools::xyz2ypd_jacobian(h_armor_xyz(x, id));
  return spherical * geometry;
}
}  // namespace auto_aim
