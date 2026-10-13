#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{
using AimClock = std::chrono::steady_clock;
using AimTime = AimClock::time_point;

// Shared by the real task executables and the closed-loop simulator. No truth
// position, armor index, or angular speed is part of the input interface.
struct ControllerConfig
{
  int task = 1;
  Color enemy = Color::red;
  double yaw_offset = 0.0;          // radians
  double pitch_offset = 0.0;        // radians, protocol pitch is positive down
  double pitch_limit = 0.3490658503988659;
  double yaw_rate_limit = 6.0;      // rad/s
  double pitch_rate_limit = 2.0;
  double measurement_timeout = 0.12;
  double reset_timeout = 0.45;
  double execution_delay = 0.025;
  double max_prediction = 0.65;
  double fire_angle_tolerance = 0.008;
  double max_facing_angle = 0.65;
  double settle_time = 1.0;
  double stage_change_threshold = 0.8;
  double acknowledgement_timeout = 0.5;
  bool allow_fire = false;
};

struct Feedback
{
  bool aim_enabled = true;
  bool fire_enabled = false;
  bool transport_ok = true;
  double yaw = 0.0;
  double pitch = 0.0;
  double bullet_speed = 22.0;
  std::uint16_t bullet_count = 0;
};

struct AimCommand
{
  bool control = false;
  bool fire = false;               // a single packet pulse, never a latched level
  double yaw = 0.0;
  double pitch = 0.0;
  double omega = 0.0;
  double omega_variance = 0.0;
  double observation_age = 0.0;
  double flight_time = 0.0;
  double facing_angle = 0.0;
  double distance = 0.0;
  Eigen::Vector3d aim_point = Eigen::Vector3d::Zero();
  int selected_armor = -1;
  int shots_commanded = 0;
  int shots_confirmed = 0;
  int stage = 1;
  std::string state = "lost";
  std::string reason = "no_target";
};

class Controller
{
public:
  explicit Controller(ControllerConfig config);
  AimCommand step(
    const std::vector<Armor> & observations, AimTime frame_time, AimTime now,
    const Feedback & feedback);
  void reset_tracking();
  const std::optional<Target> & target() const { return target_; }
  const ControllerConfig & config() const { return config_; }

private:
  ControllerConfig config_;
  std::optional<Target> target_;
  std::optional<Armor> followed_;
  std::optional<AimTime> last_frame_, last_seen_, acquired_, last_step_, last_fire_;
  std::optional<AimTime> stable_since_, stage_change_since_, outstanding_since_;
  std::optional<AimTime> engagement_started_;
  AimCommand previous_;
  double stage_speed_ = 0.0;
  double stable_speed_ = 0.0;
  int stage_ = 1;
  int shots_commanded_ = 0;
  int shots_confirmed_ = 0;
  std::uint16_t last_bullet_count_ = 0;
  bool counter_initialized_ = false;
  bool acknowledgement_fault_ = false;
  bool previous_mode_ = false;
};
}  // namespace auto_aim
