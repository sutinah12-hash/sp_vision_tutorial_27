#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
namespace
{
double seconds(AimTime a, AimTime b) { return std::chrono::duration<double>(a - b).count(); }
bool finite_observation(const Armor & a)
{
  return a.xyz_in_world.allFinite() && a.ypd_in_world.allFinite() &&
         a.ypr_in_world.allFinite() && a.xyz_in_world.norm() > 0.15 &&
         a.xyz_in_world.norm() < 30.0 && a.ypd_in_world.z() > 0.15 &&
         std::isfinite(a.confidence);
}
struct Intercept
{
  bool valid = false;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double yaw = 0, pitch = 0, impact_yaw = 0, impact_pitch = 0, flight = 0, facing = 0;
  int id = -1;
};

// Predict a copy, keeping a fixed physical armor identity during flight-time
// iteration. No simulation truth or future measurement enters this function.
Intercept intercept(const Target & estimated, int id, double delay, double speed,
                    const ControllerConfig & config)
{
  Intercept result;
  double flight = 0.0;
  bool converged = false;
  // At medium spin speed a finite-bandwidth gimbal cannot chase the rapidly
  // switching lateral reference without persistent lag. Wait at the estimated
  // front phase instead, but ONLY fire when the independently propagated armor
  // actually crosses this ray at projectile arrival. No simulator truth enters.
  const bool wait_front = config.task == 2 && std::abs(estimated.ekf_x()[7]) >= 5.5;
  for (int iteration = 0; iteration < 12; ++iteration) {
    if (delay + flight > config.max_prediction) return {};
    Target future = estimated;
    future.predict(delay + flight);
    const auto list = future.armor_xyza_list();
    if (id < 0 || id >= static_cast<int>(list.size())) return {};
    const auto & armor = list[id];
    if (!armor.allFinite()) return {};
    Eigen::Vector3d reference = armor.head<3>();
    if (wait_front) {
      const auto state = future.ekf_x();
      const Eigen::Vector2d center(state[0], state[2]);
      const double center_distance = center.norm();
      if (center_distance < 0.2) return {};
      const double radius = (armor.head<2>() - center).norm();
      reference.head<2>() = center - radius * center / center_distance;
    }
    tools::Trajectory ballistic(speed, reference.head<2>().norm(), reference.z());
    if (ballistic.unsolvable) return {};
    tools::Trajectory moving_armor_ballistic(speed, armor.head<2>().norm(), armor.z());
    if (moving_armor_ballistic.unsolvable) return {};
    result.point = reference;
    result.yaw = tools::limit_rad(std::atan2(reference.y(), reference.x()) + config.yaw_offset);
    result.pitch = -ballistic.pitch - config.pitch_offset;
    result.impact_yaw = tools::limit_rad(std::atan2(armor.y(), armor.x()) + config.yaw_offset);
    result.impact_pitch = -moving_armor_ballistic.pitch - config.pitch_offset;
    result.flight = ballistic.fly_time;
    result.facing = tools::limit_rad(armor[3] - std::atan2(armor.y(), armor.x()));
    result.id = id;
    if (std::abs(ballistic.fly_time - flight) < 1e-5) { converged = true; break; }
    flight = ballistic.fly_time;
  }
  result.valid = converged && std::isfinite(result.pitch) &&
                 std::abs(result.pitch) <= config.pitch_limit &&
                 (wait_front || std::abs(result.facing) <= config.max_facing_angle);
  return result;
}
}  // namespace

Controller::Controller(ControllerConfig config) : config_(config)
{
  const double values[] = {config.yaw_offset, config.pitch_offset, config.pitch_limit,
    config.yaw_rate_limit, config.pitch_rate_limit, config.measurement_timeout,
    config.reset_timeout, config.execution_delay, config.max_prediction,
    config.fire_angle_tolerance, config.max_facing_angle, config.settle_time,
    config.stage_change_threshold, config.acknowledgement_timeout};
  if (!std::all_of(std::begin(values), std::end(values), [](double v) { return std::isfinite(v); }) ||
      config.task < 1 || config.task > 3 || (config.enemy != red && config.enemy != blue) ||
      std::abs(config.yaw_offset) > CV_PI || std::abs(config.pitch_offset) > CV_PI / 2 ||
      config.pitch_limit <= 0 || config.pitch_limit > 1.4 || config.yaw_rate_limit <= 0 ||
      config.yaw_rate_limit > 100 || config.pitch_rate_limit <= 0 || config.pitch_rate_limit > 100 ||
      config.measurement_timeout <= 0 || config.measurement_timeout > 5.0 ||
      config.reset_timeout < config.measurement_timeout || config.execution_delay < 0 ||
      config.reset_timeout > 5.0 || config.execution_delay > 2.0 ||
      config.max_prediction <= config.execution_delay || config.max_prediction > 5.0 ||
      config.fire_angle_tolerance <= 0 || config.fire_angle_tolerance > 0.2 ||
      config.max_facing_angle <= 0 || config.max_facing_angle >= CV_PI / 2 ||
      config.settle_time <= 0 || config.settle_time > 30 ||
      config.stage_change_threshold <= 0 || config.stage_change_threshold > 20 ||
      config.acknowledgement_timeout <= 0 || config.acknowledgement_timeout > 10.0)
    throw std::invalid_argument("Invalid controller configuration");
}

void Controller::reset_tracking()
{
  target_.reset();
  followed_.reset();
  last_seen_.reset();
  last_frame_.reset();
  acquired_.reset();
  stable_since_.reset();
  stage_change_since_.reset();
  // Losing the target never replenishes ammunition allowances.
}

AimCommand Controller::step(const std::vector<Armor> & observations, AimTime frame_time,
                            AimTime now, const Feedback & feedback)
{
  AimCommand command;
  command.yaw = std::isfinite(feedback.yaw) ? tools::limit_rad(feedback.yaw) : 0.0;
  command.pitch = std::isfinite(feedback.pitch) ? feedback.pitch : 0.0;
  // Invalid time must not advance ANY bookkeeping (including acknowledgement
  // or the monotonic-time baseline), otherwise a second older sample can rearm.
  if (frame_time > now || (last_step_ && now < *last_step_)) {
    command.reason = "invalid_timestamp";
    command.shots_commanded = shots_commanded_;
    command.shots_confirmed = shots_confirmed_;
    command.stage = stage_;
    return command;
  }
  auto finish = [&](const std::string & reason) {
    command.reason = reason;
    command.shots_commanded = shots_commanded_;
    command.shots_confirmed = shots_confirmed_;
    command.stage = stage_;
    previous_ = command;
    last_step_ = now;
    return command;
  };

  // Check the deadline before processing a just-arrived acknowledgement: a
  // delayed next step must not clear an overdue outstanding pulse first.
  if (outstanding_since_ && seconds(now, *outstanding_since_) > config_.acknowledgement_timeout)
    acknowledgement_fault_ = true;
  if (!counter_initialized_) {
    last_bullet_count_ = feedback.bullet_count;
    counter_initialized_ = true;
  } else {
    const unsigned delta = static_cast<std::uint16_t>(feedback.bullet_count - last_bullet_count_);
    last_bullet_count_ = feedback.bullet_count;
    if (delta > 0 && delta <= 20 && outstanding_since_) {
      shots_confirmed_ += static_cast<int>(delta);
      outstanding_since_.reset();
      acknowledgement_fault_ = acknowledgement_fault_ || delta != 1;
    } else if (delta > 0) { acknowledgement_fault_ = true; }
  }
  if (!feedback.aim_enabled || !feedback.transport_ok || !std::isfinite(feedback.yaw) ||
      !std::isfinite(feedback.pitch)) {
    reset_tracking();
    previous_mode_ = false;
    return finish("disabled_or_transport_stale");
  }
  if (!previous_mode_) reset_tracking();
  previous_mode_ = true;
  if (last_seen_ && seconds(now, *last_seen_) > config_.reset_timeout) reset_tracking();

  bool accepted = false;
  const bool new_frame = !last_frame_ || frame_time > *last_frame_;
  if (new_frame && seconds(now, frame_time) <= config_.measurement_timeout) {
    last_frame_ = frame_time;
    std::vector<const Armor *> candidates;
    for (const auto & armor : observations) {
      if (armor.color != config_.enemy || armor.name == ArmorName::not_armor ||
          !finite_observation(armor)) continue;
      if (config_.task == 3 && armor.name != ArmorName::outpost) continue;
      if (config_.task == 2 && (armor.name == ArmorName::outpost || armor.name == ArmorName::base)) continue;
      candidates.push_back(&armor);
    }
    std::sort(candidates.begin(), candidates.end(), [&](const Armor * a, const Armor * b) {
      const double da = std::abs(tools::limit_rad(a->ypd_in_world[0] - feedback.yaw));
      const double db = std::abs(tools::limit_rad(b->ypd_in_world[0] - feedback.yaw));
      return da < db;
    });
    if (config_.task == 1) {
      if (!candidates.empty()) { followed_ = *candidates.front(); accepted = true; }
    } else {
      if (target_ && target_->diverged()) reset_tracking();
      if (!target_ && !candidates.empty()) {
        const auto & armor = *candidates.front();
        const bool balance = armor.type == ArmorType::big &&
          (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);
        target_.emplace(armor, frame_time, config_.task == 3 ? 0.2765 : 0.2,
                        config_.task == 3 ? 3 : (balance ? 2 : 4));
        acquired_ = frame_time;
        accepted = true;
      } else if (target_) {
        target_->predict(frame_time);
        for (const Armor * armor : candidates) {
          if (armor->name != target_->name || armor->type != target_->armor_type) continue;
          accepted = target_->update(*armor) || accepted;
        }
        if (target_->diverged()) { reset_tracking(); accepted = false; }
      }
    }
    if (accepted) last_seen_ = frame_time;
  }
  if (!last_seen_) return finish("no_valid_observation");
  command.observation_age = seconds(now, *last_seen_);
  if (command.observation_age > config_.measurement_timeout) return finish("observation_timeout");

  const double step_dt = last_step_ ? std::clamp(seconds(now, *last_step_), 0.0, 0.05) : 0.01;
  const double previous_yaw = previous_.control ? previous_.yaw : feedback.yaw;
  const double previous_pitch = previous_.control ? previous_.pitch : feedback.pitch;
  double desired_yaw = 0.0, desired_pitch = 0.0;
  double impact_yaw = 0.0, impact_pitch = 0.0;
  bool converged = false;
  if (config_.task == 1) {
    const auto xyz = followed_->xyz_in_world;
    desired_yaw = tools::limit_rad(std::atan2(xyz.y(), xyz.x()) + config_.yaw_offset);
    desired_pitch = -std::atan2(xyz.z(), xyz.head<2>().norm()) - config_.pitch_offset;
    command.aim_point = xyz;
    command.state = accepted ? "following" : "temporary_loss";
  } else {
    if (!target_) return finish("target_lost");
    command.omega = target_->ekf_x()[7];
    command.omega_variance = target_->ekf().P(7, 7);
    converged = target_->is_converged() && acquired_ && seconds(now, *acquired_) >= config_.settle_time;
    command.state = converged ? (accepted ? "tracking" : "temporary_loss") : "acquiring";
    if (!std::isfinite(feedback.bullet_speed) || feedback.bullet_speed < 10 || feedback.bullet_speed > 35)
      return finish("invalid_bullet_speed");
    const double age = seconds(now, target_->state_time());
    Intercept selected;
    double best_score = std::numeric_limits<double>::infinity();
    const auto estimated_armors = target_->armor_xyza_list();
    const auto count = estimated_armors.size();
    double lowest_estimated_height = std::numeric_limits<double>::infinity();
    for (const auto & armor : estimated_armors)
      lowest_estimated_height = std::min(lowest_estimated_height, armor.z());
    const bool wait_front = config_.task == 2 && std::abs(command.omega) >= 5.5;
    for (int id = 0; id < static_cast<int>(count); ++id) {
      // Hold one estimated height group at medium speed. Switching the pitch
      // reference between staggered plates every quarter-turn defeats a
      // finite-bandwidth gimbal even when yaw already waits at the front ray.
      // This is a selection from the fitted model, never a ground-truth label.
      if (wait_front && estimated_armors[id].z() > lowest_estimated_height + 0.03) continue;
      auto candidate = intercept(*target_, id, age + config_.execution_delay, feedback.bullet_speed, config_);
      if (!candidate.valid) continue;
      const double score = std::abs(candidate.facing) +
        0.15 * std::abs(tools::limit_rad(candidate.yaw - previous_yaw)) +
        (previous_.selected_armor == id ? -0.04 : 0.0);
      if (score < best_score) { selected = candidate; best_score = score; }
    }
    if (!selected.valid) return finish("no_visible_reachable_intercept");
    desired_yaw = selected.yaw;
    desired_pitch = selected.pitch;
    impact_yaw = selected.impact_yaw;
    impact_pitch = selected.impact_pitch;
    command.aim_point = selected.point;
    command.flight_time = selected.flight;
    command.facing_angle = selected.facing;
    command.selected_armor = selected.id;
  }
  if (!std::isfinite(desired_yaw) || !std::isfinite(desired_pitch)) return finish("nonfinite_aim");
  const bool within_pitch = std::abs(desired_pitch) <= config_.pitch_limit;
  desired_pitch = std::clamp(desired_pitch, -config_.pitch_limit, config_.pitch_limit);
  const double yaw_delta = tools::limit_rad(desired_yaw - previous_yaw);
  command.yaw = tools::limit_rad(previous_yaw +
    std::clamp(yaw_delta, -config_.yaw_rate_limit * step_dt, config_.yaw_rate_limit * step_dt));
  command.pitch = previous_pitch + std::clamp(desired_pitch - previous_pitch,
    -config_.pitch_rate_limit * step_dt, config_.pitch_rate_limit * step_dt);
  command.pitch = std::clamp(command.pitch, -config_.pitch_limit, config_.pitch_limit);
  command.distance = command.aim_point.norm();
  command.control = true;
  if (config_.task == 1) return finish(within_pitch ? "follow_only" : "pitch_limited");

  const double current_speed = std::abs(command.omega);
  if (!stable_since_ || std::abs(current_speed - stable_speed_) > 0.3) {
    stable_since_ = now;
    stable_speed_ = current_speed;
  }
  // No wheel-enable bit exists in the supplied serial protocol. A lost target
  // never rearms task 2; only a distinct stable speed may start its second stage.
  if (config_.task == 2 && stage_ == 1 && shots_confirmed_ >= 10 &&
      std::abs(current_speed - stage_speed_) >= config_.stage_change_threshold && converged) {
    if (!stage_change_since_) stage_change_since_ = now;
    if (seconds(now, *stage_change_since_) >= config_.settle_time && stable_since_ &&
        seconds(now, *stable_since_) >= config_.settle_time) {
      stage_ = 2;
      shots_commanded_ = shots_confirmed_ = 0;
      stage_speed_ = current_speed;
      stage_change_since_.reset();
    }
  } else stage_change_since_.reset();

  if (!config_.allow_fire || !feedback.fire_enabled) return finish("fire_not_armed");
  if (!within_pitch) return finish("pitch_limited");
  if (!accepted) return finish("no_fresh_observation_for_fire");
  if (!converged) return finish("estimator_not_converged");
  if (acknowledgement_fault_) return finish("shot_acknowledgement_fault");
  if (outstanding_since_) return finish("waiting_shot_acknowledgement");
  if (config_.task == 2 && shots_commanded_ >= 10) return finish("stage_shot_budget_exhausted");
  if (config_.task == 3 && engagement_started_ && seconds(now, *engagement_started_) >= 30.0)
    return finish("outpost_time_budget_exhausted");
  if (std::abs(command.facing_angle) > config_.max_facing_angle)
    return finish("waiting_for_front_face");
  if (std::abs(tools::limit_rad(impact_yaw - desired_yaw)) > config_.fire_angle_tolerance ||
      std::abs(impact_pitch - desired_pitch) > config_.fire_angle_tolerance)
    return finish("waiting_for_predicted_front_crossing");
  if (std::abs(tools::limit_rad(feedback.yaw - desired_yaw)) > config_.fire_angle_tolerance ||
      std::abs(feedback.pitch - desired_pitch) > config_.fire_angle_tolerance ||
      std::abs(tools::limit_rad(feedback.yaw - impact_yaw)) > config_.fire_angle_tolerance ||
      std::abs(feedback.pitch - impact_pitch) > config_.fire_angle_tolerance ||
      std::abs(tools::limit_rad(command.yaw - desired_yaw)) > config_.fire_angle_tolerance ||
      std::abs(command.pitch - desired_pitch) > config_.fire_angle_tolerance)
    return finish("gimbal_not_aligned");
  const double interval = config_.task == 2 ? 0.2 : 1.0 / 12.0;
  if (last_fire_ && seconds(now, *last_fire_) + 1e-9 < interval) return finish("rate_limited");
  command.fire = true;
  ++shots_commanded_;
  if (shots_commanded_ == 1) stage_speed_ = current_speed;
  last_fire_ = now;
  outstanding_since_ = now;
  if (!engagement_started_) engagement_started_ = now;
  return finish("single_shot_pulse");
}
}  // namespace auto_aim
