#include "src/task_diagnostics.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
}

int main()
{
  try {
    auto size = task_ui::parse_window_size("960x600");
    require(size.width == 960 && size.height == 600, "window dimensions");
    cv::Mat original(1080, 1440, CV_8UC3);
    cv::randu(original, cv::Scalar::all(0), cv::Scalar::all(255));
    const cv::Mat untouched = original.clone();
    const cv::Mat canvas = task_ui::add_status_panel(original);
    require(canvas.cols == original.cols && canvas.rows == original.rows + 166, "HUD adds canvas, not crop");
    require(cv::norm(original, untouched, cv::NORM_INF) == 0, "input pixels unchanged");
    require(cv::norm(canvas(cv::Rect(0, 166, original.cols, original.rows)), original, cv::NORM_INF) == 0,
      "all camera pixels preserved, including corners and edges");
    for (const char * invalid : {"0x600", "960x0", "-1x600", "960x600junk", "960x600x1",
                                "999999999999x600", "96x60", "960X600", "960x", "x600"}) {
      bool rejected = false;
      try { task_ui::parse_window_size(invalid); } catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, invalid);
    }
    auto_aim::ControllerConfig config;
    auto_aim::Feedback feedback;
    auto_aim::AimCommand command;
    command.reason = "follow_only";
    std::list<auto_aim::Armor> detections;
    auto inspect = [&]() { return task_ui::inspect_frame(detections, config, feedback, command, 10, true); };
    require(inspect().hint == "no_detections", "empty detections");
    auto_aim::Armor armor;
    armor.color = auto_aim::blue;
    armor.name = auto_aim::four;
    armor.pose_valid = true;
    detections.push_back(armor);
    require(inspect().hint == "enemy_color_mismatch", "blue is not red");
    armor.color = auto_aim::red;
    armor.pose_valid = false;
    detections.push_back(armor);
    require(inspect().hint == "enemy_pnp_rejected", "blue PnP cannot hide red PnP failure");
    detections.back().raw_pose_valid = true;
    detections.back().pose_status = "vehicle_model_reprojection";
    require(inspect().hint == "enemy_model_rejected", "model mismatch is not PnP failure");
    require(inspect().pose_rejections.at("vehicle_model_reprojection") == 1,
      "preserve the exact pose rejection reason");
    detections.back().pose_valid = true;
    require(inspect().enemy_solved == 1 && inspect().hint == "follow_only", "matching valid pose");
    require(inspect().red == 1 && inspect().blue == 1, "color counts");
    config.task = 3;
    require(inspect().hint == "wrong_armor_type", "task 3 requires outpost");
    detections.back().name = auto_aim::outpost;
    require(inspect().task_poses == 1, "outpost eligible for task 3");
    config.task = 2;
    require(inspect().hint == "wrong_armor_type", "task 2 excludes outpost");
    detections.back().name = auto_aim::four;
    require(inspect().task_poses == 1, "red four eligible for task 2");
    feedback.aim_enabled = false;
    require(inspect().hint == "mode_not_auto", "auto mode diagnosis");
    feedback.transport_ok = false;
    require(inspect().hint == "telemetry_stale", "transport diagnosis");
    require(task_ui::inspect_frame(detections, config, feedback, command, 10, false).hint == "imu_missing", "IMU diagnosis");
    feedback.aim_enabled = feedback.transport_ok = true;
    require(task_ui::inspect_frame(detections, config, feedback, command, 121, true).hint == "frame_too_old", "120 ms limit unchanged");
    require(task_ui::inspect_frame(detections, config, feedback, command, -1, true).hint == "invalid_frame_time", "future frame");
    require(task_ui::inspect_frame(detections, config, feedback, command,
      std::numeric_limits<double>::quiet_NaN(), true).hint == "invalid_frame_time", "NaN age");
    require(command.reason == "follow_only", "diagnostics preserve controller reason");
    std::cout << "Window, full-view pixel preservation and red-target diagnostics passed.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
