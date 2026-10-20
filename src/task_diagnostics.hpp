#pragma once

#include <algorithm>
#include <cmath>
#include <list>
#include <stdexcept>
#include <string>

#include "tasks/auto_aim/controller.hpp"

namespace task_ui
{
struct WindowSize { int width = 960; int height = 600; };

inline cv::Mat add_status_panel(const cv::Mat & image)
{
  // Extra canvas, never a crop: all original camera pixels remain below the HUD.
  cv::Mat output;
  cv::copyMakeBorder(image, output, 166, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(20, 20, 20));
  return output;
}

inline WindowSize parse_window_size(const std::string & value)
{
  const auto separator = value.find('x');
  const auto digits = [](const std::string & text) {
    return !text.empty() && text.size() <= 4 &&
      std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
  };
  if (separator == std::string::npos || !digits(value.substr(0, separator)) ||
      !digits(value.substr(separator + 1)))
    throw std::invalid_argument("--window-size must be WIDTHxHEIGHT, e.g. 960x600");
  WindowSize result{std::stoi(value.substr(0, separator)), std::stoi(value.substr(separator + 1))};
  if (result.width < 320 || result.width > 3840 || result.height < 240 || result.height > 2160)
    throw std::invalid_argument("--window-size bounds: width 320..3840, height 240..2160");
  return result;
}

// Diagnostics only: these counts never authorize control or bypass Controller gates.
struct FrameDiagnostics
{
  int red = 0, blue = 0, enemy_detected = 0, enemy_solved = 0, task_poses = 0;
  double age_ms = 0.0, limit_ms = 0.0;
  bool imu_ok = false;
  bool requested_control = false;
  std::string enemy, hint;
};

inline FrameDiagnostics inspect_frame(
  const std::list<auto_aim::Armor> & detections, const auto_aim::ControllerConfig & config,
  const auto_aim::Feedback & feedback, const auto_aim::AimCommand & command,
  double age_ms, bool imu_ok)
{
  FrameDiagnostics d;
  d.age_ms = age_ms;
  d.limit_ms = config.measurement_timeout * 1000.0;
  d.imu_ok = imu_ok;
  d.requested_control = command.control;
  d.enemy = config.enemy == auto_aim::red ? "red" : "blue";
  for (const auto & a : detections) {
    if (a.color == auto_aim::red) ++d.red;
    if (a.color == auto_aim::blue) ++d.blue;
    if (a.color != config.enemy) continue;
    ++d.enemy_detected;
    if (!a.pose_valid) continue;
    ++d.enemy_solved;
    if (a.name == auto_aim::not_armor) continue;
    if (config.task == 3 && a.name != auto_aim::outpost) continue;
    if (config.task == 2 && (a.name == auto_aim::outpost || a.name == auto_aim::base)) continue;
    ++d.task_poses;
  }
  if (!imu_ok) d.hint = "imu_missing";
  else if (!feedback.transport_ok) d.hint = "telemetry_stale";
  else if (!feedback.aim_enabled) d.hint = "mode_not_auto";
  else if (!std::isfinite(age_ms) || age_ms < 0) d.hint = "invalid_frame_time";
  else if (age_ms > d.limit_ms) d.hint = "frame_too_old";
  else if (detections.empty()) d.hint = "no_detections";
  else if (d.enemy_detected == 0) d.hint = "enemy_color_mismatch";
  else if (d.enemy_solved == 0) d.hint = "enemy_pnp_rejected";
  else if (d.task_poses == 0) d.hint = "wrong_armor_type";
  else d.hint = command.reason;
  return d;
}

}  // namespace task_ui
