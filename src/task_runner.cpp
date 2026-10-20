#include "task_runner.hpp"
#include "task_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/controller.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr double kRadiansPerDegree = 0.017453292519943295;
volatile std::sig_atomic_t stop_requested = 0;

class SignalScope
{
public:
  SignalScope()
  {
    stop_requested = 0;
    old_int_ = std::signal(SIGINT, [](int) { stop_requested = 1; });
    old_term_ = std::signal(SIGTERM, [](int) { stop_requested = 1; });
  }
  ~SignalScope()
  {
    if (old_int_ != SIG_ERR) std::signal(SIGINT, old_int_);
    if (old_term_ != SIG_ERR) std::signal(SIGTERM, old_term_);
  }
private:
  using Handler = void (*)(int);
  Handler old_int_, old_term_;
};

struct Options
{
  std::string config;
  std::string video;
  std::filesystem::path output = "artifacts";
  int frames = 0;
  bool live = false;
  bool allow_fire = false;
  bool show = false;
  task_ui::WindowSize window;
  bool help = false;
};

void usage(int task)
{
  std::cout << "Task " << task << " - safe default: no camera or serial access\n"
            << "Usage: task_" << task << " CONFIG (--video FILE | --live) [options]\n"
            << "  --video FILE   Decode a local recording, YOLO/PnP with STATIC IDENTITY IMU.\n"
            << "                 Not closed-loop, pose-time, ballistic, or hit validation.\n"
            << "  --live         Explicitly open the configured camera and gimbal serial port.\n"
            << "  --allow-fire   Separately arm task 2/3; ONLY under supervised field procedure.\n"
            << "                 Task 1 can never fire. This flag is invalid with --video.\n"
            << "  --frames N     Stop after N decoded/captured frames (default: until EOF/exit).\n"
            << "  --output DIR   Parent directory for a unique log and PNG run directory.\n"
            << "  --show         Show annotated frames; Esc or q exits.\n"
            << "  --window-size WIDTHxHEIGHT  Resizable preview, default 960x600; does not resize detection input.\n"
            << "  --help         Show this help. Run from the project root for relative assets.\n";
}

Options parse(int task, int argc, char * argv[])
{
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::invalid_argument("Missing value for " + arg);
      return argv[i];
    };
    if (arg == "--help" || arg == "-h") result.help = true;
    else if (arg == "--live") result.live = true;
    else if (arg == "--allow-fire") result.allow_fire = true;
    else if (arg == "--show") result.show = true;
    else if (arg == "--window-size") result.window = task_ui::parse_window_size(value());
    else if (arg == "--video") result.video = value();
    else if (arg == "--output") result.output = value();
    else if (arg == "--frames") {
      const auto text = value();
      std::size_t consumed = 0;
      result.frames = std::stoi(text, &consumed);
      if (consumed != text.size() || result.frames <= 0)
        throw std::invalid_argument("--frames must be a positive integer");
    } else if (!arg.empty() && arg[0] == '-') {
      throw std::invalid_argument("Unknown option: " + arg);
    } else if (result.config.empty()) result.config = arg;
    else throw std::invalid_argument("Unexpected argument: " + arg);
  }
  if (result.help || argc == 1) { result.help = true; return result; }
  if (result.config.empty()) throw std::invalid_argument("A YAML configuration path is required");
  if (result.live == !result.video.empty())
    throw std::invalid_argument("Choose exactly one of --live or --video FILE");
  if (result.allow_fire && (!result.live || task == 1))
    throw std::invalid_argument("--allow-fire is permitted only for live task 2 or task 3");
  if (!result.video.empty() && !std::filesystem::is_regular_file(result.video))
    throw std::invalid_argument("--video requires an existing local file, not a device or URL");
  return result;
}

double finite_value(const YAML::Node & node, const char * key, double fallback)
{
  const double value = node && node[key] ? node[key].as<double>() : fallback;
  if (!std::isfinite(value)) throw std::invalid_argument(std::string("Non-finite YAML value: ") + key);
  return value;
}

auto_aim::ControllerConfig controller_config(int task, const Options & options, const YAML::Node & yaml)
{
  auto_aim::ControllerConfig config;
  config.task = task;
  const auto color = yaml["enemy_color"].as<std::string>();
  if (color != "red" && color != "blue") throw std::invalid_argument("enemy_color must be red or blue");
  config.enemy = color == "red" ? auto_aim::red : auto_aim::blue;
  config.yaw_offset = finite_value(yaml, "yaw_offset", 0.0) * kRadiansPerDegree;
  config.pitch_offset = finite_value(yaml, "pitch_offset", 0.0) * kRadiansPerDegree;
  const auto section = yaml["controller"];
  config.pitch_limit = finite_value(section, "pitch_limit_deg", 20.0) * kRadiansPerDegree;
  config.yaw_rate_limit = finite_value(section, "yaw_rate_limit", config.yaw_rate_limit);
  config.pitch_rate_limit = finite_value(section, "pitch_rate_limit", config.pitch_rate_limit);
  config.measurement_timeout = finite_value(section, "measurement_timeout", config.measurement_timeout);
  config.reset_timeout = finite_value(section, "reset_timeout", config.reset_timeout);
  config.execution_delay = finite_value(section, "execution_delay", config.execution_delay);
  config.max_prediction = finite_value(section, "max_prediction", config.max_prediction);
  config.fire_angle_tolerance = finite_value(section, "fire_angle_tolerance", config.fire_angle_tolerance);
  config.max_facing_angle = finite_value(section, "max_facing_angle", config.max_facing_angle);
  config.settle_time = finite_value(section, "settle_time", config.settle_time);
  config.stage_change_threshold = finite_value(section, "stage_change_threshold", config.stage_change_threshold);
  config.acknowledgement_timeout = finite_value(section, "acknowledgement_timeout", config.acknowledgement_timeout);
  if (config.max_facing_angle <= 0 || config.max_facing_angle >= 1.5707963267948966 ||
      config.stage_change_threshold <= 0 || std::abs(config.yaw_offset) > 3.141592653589793 ||
      std::abs(config.pitch_offset) > 1.5707963267948966)
    throw std::invalid_argument("Controller angular parameters are outside safe bounds");
  config.allow_fire = options.live && options.allow_fire && task != 1;
  return config;
}

void validate_hardware_config(const YAML::Node & yaml)
{
  // Validate before opening anything: legacy helper functions terminate on missing fields.
  for (const char * key : {"com_port", "camera_name", "exposure_ms", "rotation_angle", "gain", "vid_pid"}) {
    if (!yaml[key]) throw std::invalid_argument(std::string("Missing live configuration key: ") + key);
  }
  if (yaml["camera_name"].as<std::string>() == "hikrobot_sn" && !yaml["serial_number"])
    throw std::invalid_argument("hikrobot_sn requires serial_number");
}

struct StopGimbal
{
  io::Gimbal * gimbal = nullptr;
  ~StopGimbal()
  {
    if (!gimbal) return;
    try { gimbal->send(false, false, 0.0f, 0.0f); } catch (...) {}
  }
};

auto_aim::Feedback hardware_feedback(io::Gimbal & gimbal, bool allow_fire, bool pose_ok)
{
  const auto state = gimbal.state();
  auto_aim::Feedback feedback;
  feedback.aim_enabled = gimbal.mode() == io::GimbalMode::AUTO_AIM;
  feedback.fire_enabled = allow_fire;
  feedback.transport_ok = pose_ok && gimbal.fresh(200ms);
  feedback.yaw = state.yaw;
  feedback.pitch = state.pitch;
  feedback.bullet_speed = state.bullet_speed;
  feedback.bullet_count = state.bullet_count;
  return feedback;
}

std::string armor_label(const auto_aim::Armor & armor)
{
  const auto color = static_cast<std::size_t>(armor.color);
  const auto name = static_cast<std::size_t>(armor.name);
  const std::string c = color < auto_aim::COLORS.size() ? auto_aim::COLORS[color] : "unknown";
  const std::string n = name < auto_aim::ARMOR_NAMES.size() ? auto_aim::ARMOR_NAMES[name] : "unknown";
  std::ostringstream text;
  text << c << n << ' ' << std::fixed << std::setprecision(2) << armor.confidence;
  if (!armor.pose_valid) text << " PNP_REJECTED";
  return text.str();
}

cv::Mat annotate(const cv::Mat & image, const std::list<auto_aim::Armor> & detections,
                 const auto_aim::AimCommand & command, const auto_aim::Solver & solver,
                 int task, bool live, int frame,
                 const task_ui::FrameDiagnostics & diagnostics, const auto_aim::Feedback & feedback)
{
  auto output = image.clone();
  const cv::Scalar green(0, 255, 0);
  for (const auto & armor : detections) {
    if (armor.points.size() != 4) continue;
    bool valid = true;
    std::vector<cv::Point> polygon;
    for (const auto & point : armor.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          std::abs(point.x) > 1e6 || std::abs(point.y) > 1e6) { valid = false; break; }
      polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    if (!valid) continue;
    cv::polylines(output, polygon, true, green, 2, cv::LINE_AA);
    const auto label_position = cv::Point(std::clamp(polygon[0].x, 0, std::max(0, image.cols - 100)),
                                          std::clamp(polygon[0].y - 8, 18, std::max(18, image.rows - 5)));
    cv::putText(output, armor_label(armor), label_position, cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2);
  }
  if (command.control) {
    const auto aim_pixel = solver.world2pixel(command.aim_point);
    if (aim_pixel && aim_pixel->x >= 0 && aim_pixel->y >= 0 && aim_pixel->x < image.cols && aim_pixel->y < image.rows)
      cv::drawMarker(output, *aim_pixel, cv::Scalar(0, 180, 255), cv::MARKER_CROSS, 22, 2);
  }
  output = task_ui::add_status_panel(output);
  std::ostringstream first, second, third, fourth, fifth;
  first << "TASK " << task << " | " << (live ? "LIVE" : "OFFLINE STATIC IMU / NO HARDWARE VALIDATION") << " | frame " << frame;
  second << std::fixed << std::setprecision(3) << "yaw=" << command.yaw << " pitch=" << command.pitch
         << " omega=" << command.omega << " control=" << command.control << " fire=" << command.fire;
  third << command.state << " / " << command.reason << " | shots=" << command.shots_confirmed;
  fourth << "enemy=" << diagnostics.enemy << " red=" << diagnostics.red << " blue=" << diagnostics.blue
         << " enemy_pose=" << diagnostics.enemy_solved << " task_pose=" << diagnostics.task_poses
         << " auto=" << feedback.aim_enabled << " imu=" << diagnostics.imu_ok << " link=" << feedback.transport_ok;
  fifth << std::fixed << std::setprecision(1) << "frame_age=" << diagnostics.age_ms << "/" << diagnostics.limit_ms
        << "ms | " << diagnostics.hint;
  cv::putText(output, first.str(), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
  cv::putText(output, second.str(), {10, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.55, green, 1, cv::LINE_AA);
  cv::putText(output, third.str(), {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
  cv::putText(output, fourth.str(), {10, 115}, cv::FONT_HERSHEY_SIMPLEX, 0.55, green, 1, cv::LINE_AA);
  cv::putText(output, fifth.str(), {10, 145}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
  return output;
}

struct RunLog
{
  std::filesystem::path directory;
  std::ofstream stream;
  tools::Plotter plotter;
  int saved_images = 0;

  RunLog(const Options & options, int task)
  {
    const auto identifier = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    directory = options.output / ("task_" + std::to_string(task) + "_" + std::to_string(identifier));
    if (!std::filesystem::create_directories(directory))
      throw std::runtime_error("Output run directory already exists: " + directory.string());
    stream.open(directory / "telemetry.jsonl");
    if (!stream) throw std::runtime_error("Cannot open telemetry.jsonl");
    stream.exceptions(std::ios::badbit | std::ios::failbit);
  }

  void record(int frame, double time, const auto_aim::AimCommand & command,
              const auto_aim::Feedback & feedback, bool live, int detected, int solved,
              double processing_ms, const std::string & event = "frame",
              const task_ui::FrameDiagnostics * diagnostics = nullptr)
  {
    nlohmann::json numeric = {
      {"time", time}, {"frame", frame}, {"cmd_yaw", command.yaw}, {"cmd_pitch", command.pitch},
      {"control", command.control ? 1 : 0}, {"fire", command.fire ? 1 : 0},
      {"estimated_omega", command.omega}, {"omega_variance", command.omega_variance},
      {"age", command.observation_age}, {"shotcount", command.shots_confirmed},
      {"shots_commanded", command.shots_commanded}, {"stage", command.stage},
      {"feedback_yaw", feedback.yaw}, {"feedback_pitch", feedback.pitch},
      {"transport_fresh", feedback.transport_ok ? 1 : 0}, {"bullet_count", feedback.bullet_count},
      {"detections", detected}, {"valid_poses", solved}, {"processing_ms", processing_ms},
      {"flight_time", command.flight_time}, {"distance", command.distance}};
    numeric["auto_mode"] = feedback.aim_enabled ? 1 : 0;
    if (diagnostics) {
      numeric["red_detections"] = diagnostics->red;
      numeric["blue_detections"] = diagnostics->blue;
      numeric["enemy_detections"] = diagnostics->enemy_detected;
      numeric["enemy_valid_poses"] = diagnostics->enemy_solved;
      numeric["task_type_poses"] = diagnostics->task_poses;
      numeric["frame_age_ms"] = diagnostics->age_ms;
      numeric["frame_limit_ms"] = diagnostics->limit_ms;
      numeric["imu_ok"] = diagnostics->imu_ok ? 1 : 0;
      numeric["requested_control"] = diagnostics->requested_control ? 1 : 0;
    }
    plotter.plot(numeric);
    if (diagnostics) {
      numeric["diagnostic"] = diagnostics->hint;
      numeric["enemy_color"] = diagnostics->enemy;
    }
    numeric["event"] = event;
    numeric["state"] = command.state;
    numeric["reason"] = command.reason;
    numeric["input_mode"] = live ? "live" : "offline_static_imu";
    numeric["hardware_validation"] = false;
    stream << numeric.dump() << '\n';
    stream.flush();
  }

  void snapshot(const cv::Mat & image, int frame, const std::string & suffix = "")
  {
    if (image.empty() || (saved_images >= 8 && suffix.empty())) return;
    std::ostringstream name;
    name << "frame_" << std::setw(6) << std::setfill('0') << frame << suffix << ".png";
    if (!cv::imwrite((directory / name.str()).string(), image))
      throw std::runtime_error("Failed to save annotated PNG");
    ++saved_images;
  }
};
}  // namespace

int run_task(int task, int argc, char * argv[])
{
  try {
    const auto options = parse(task, argc, argv);
    if (options.help) { usage(task); return 0; }
    SignalScope signals;
    tools::logger();
    const auto yaml = YAML::LoadFile(options.config);
    auto_aim::Controller controller(controller_config(task, options, yaml));
    // All model/config failures happen before opening real hardware.
    auto_aim::Solver solver(options.config);
    auto_aim::YOLO detector(options.config, false);
    RunLog log(options, task);
    log.stream << nlohmann::json({{"event", "metadata"}, {"task", task},
      {"config", std::filesystem::absolute(options.config).string()}, {"live", options.live},
      {"fire_armed", controller.config().allow_fire}, {"video", options.video},
      {"enemy_color", yaml["enemy_color"].as<std::string>()},
      {"window_width", options.window.width}, {"window_height", options.window.height},
      {"measurement_timeout_ms", controller.config().measurement_timeout * 1000.0},
      {"notice", options.live ? "Field validation and wheel-enable confirmation required" :
        "Fixed identity IMU, file time axis; no real robot, closed-loop, or hit validation"}}).dump() << '\n';
    std::unique_ptr<io::Gimbal> gimbal;
    std::unique_ptr<io::Camera> camera;
    cv::VideoCapture video;
    // Declared after Camera so unwinding disables the gimbal before camera teardown.
    StopGimbal stop_guard;
    if (options.live) {
      validate_hardware_config(yaml);
      gimbal = std::make_unique<io::Gimbal>(options.config);
      stop_guard.gimbal = gimbal.get();
    }
    double fps = 30.0;
    if (options.live) {
      camera = std::make_unique<io::Camera>(options.config);
      std::cout << "LIVE: confirm calibration, physical limits, protocol pulse semantics and wheel enable with the supervisor.\n";
    } else {
      if (!video.open(options.video)) throw std::runtime_error("Cannot decode video: " + options.video);
      const double reported_fps = video.get(cv::CAP_PROP_FPS);
      if (std::isfinite(reported_fps) && reported_fps >= 1.0 && reported_fps <= 240.0) fps = reported_fps;
      std::cout << "OFFLINE: static identity IMU; perception/PnP diagnostics only, no hardware or hit validation. FPS=" << fps << '\n';
    }
    std::cout << "Artifacts: " << std::filesystem::absolute(log.directory) << '\n';
    std::cout << "Enemy=" << yaml["enemy_color"].as<std::string>() << " | config=" << options.config
              << " | fire_armed=" << controller.config().allow_fire << '\n';
    if (options.show) {
      cv::namedWindow("Final Project Aim", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
      cv::resizeWindow("Final Project Aim", options.window.width, options.window.height);
    }
    const auto epoch = Clock::now();
    int frame = 0, camera_timeouts = 0, detection_frames = 0;
    cv::Mat last_overlay;
    while (!stop_requested && (options.frames == 0 || frame < options.frames)) {
      cv::Mat image;
      auto_aim::AimTime frame_time;
      if (options.live) {
        if (!camera->try_read_for(image, frame_time, 100ms) || image.empty()) {
          const auto now = Clock::now();
          auto feedback = hardware_feedback(*gimbal, options.allow_fire, false);
          auto command = controller.step({}, now, now, feedback);
          command.control = command.fire = false;
          command.reason = "camera_timeout";
          gimbal->send(false, false, 0.0f, 0.0f);
          log.record(frame, std::chrono::duration<double>(now - epoch).count(), command,
                     feedback, true, 0, 0, 0.0, "camera_timeout");
          if (++camera_timeouts >= 30) throw std::runtime_error("No camera frame after 30 bounded attempts");
          continue;
        }
        camera_timeouts = 0;
      } else {
        if (!video.read(image) || image.empty()) break;
        frame_time = epoch + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(frame / fps));
      }
      const auto work_start = Clock::now();
      auto attitude = options.live ? gimbal->try_q(frame_time, 50ms) :
        std::optional<Eigen::Quaterniond>{Eigen::Quaterniond::Identity()};
      std::list<auto_aim::Armor> detections;
      std::vector<auto_aim::Armor> observations;
      if (attitude) {
        solver.set_R_gimbal2world(*attitude);
        detections = detector.detect(image, frame);
        for (auto & armor : detections) if (solver.solve(armor)) observations.push_back(armor);
      }
      const auto now = options.live ? Clock::now() : frame_time;
      auto_aim::Feedback feedback;
      if (options.live) feedback = hardware_feedback(*gimbal, options.allow_fire, attitude.has_value());
      else {
        feedback.aim_enabled = true;
        feedback.fire_enabled = false;
        feedback.transport_ok = true;  // Only synthetic/offline controller input, not physical health.
        feedback.bullet_speed = finite_value(yaml["controller"], "offline_bullet_speed", 22.0);
      }
      auto command = controller.step(observations, frame_time, now, feedback);
      const bool requested_control = command.control;
      if (options.live) {
        if (!attitude || !gimbal->fresh(200ms)) {
          command.control = command.fire = false;
          command.reason = "imu_unavailable_or_stale";
          feedback.transport_ok = false;
        } else if (gimbal->mode() != io::GimbalMode::AUTO_AIM) {
          // Recheck after perception/decision. The MCU must also reject commands
          // outside AUTO_AIM: a host-side check cannot eliminate a final wire-time race.
          command.control = command.fire = false;
          command.reason = "mode_exited_before_send";
          feedback.aim_enabled = false;
        }
        gimbal->send(command.control, task != 1 && options.allow_fire && command.fire,
                     static_cast<float>(command.yaw), static_cast<float>(command.pitch));
      } else command.fire = false;
      auto diagnostics = task_ui::inspect_frame(detections, controller.config(), feedback, command,
        std::chrono::duration<double, std::milli>(now - frame_time).count(), attitude.has_value());
      diagnostics.requested_control = requested_control;
      const double processing_ms = std::chrono::duration<double, std::milli>(Clock::now() - work_start).count();
      log.record(frame, std::chrono::duration<double>(now - epoch).count(), command, feedback,
                 options.live, static_cast<int>(detections.size()), static_cast<int>(observations.size()), processing_ms,
                 "frame", &diagnostics);
      last_overlay = annotate(image, detections, command, solver, task, options.live, frame,
                              diagnostics, feedback);
      if (!detections.empty()) ++detection_frames;
      if (frame == 0 || frame % 30 == 0 || (!detections.empty() && detection_frames == 1)) log.snapshot(last_overlay, frame);
      if (options.show) {
        cv::imshow("Final Project Aim", last_overlay);
        const int key = cv::waitKey(1) & 0xff;
        // GTK3 in Ubuntu 22.04 returns -1 for unsupported WND_PROP_VISIBLE.
        // Do not mistake that value for a closed window and stop after one frame.
        if (key == 27 || key == 'q') {
          ++frame;
          break;
        }
      }
      ++frame;
    }
    if (options.live) gimbal->send(false, false, 0.0f, 0.0f);
    if (frame == 0 && !stop_requested) throw std::runtime_error("No frame was decoded or captured");
    log.snapshot(last_overlay, std::max(0, frame - 1), "_final");
    if (options.show) cv::destroyWindow("Final Project Aim");
    std::cout << "Processed " << frame << " frames; detections in " << detection_frames
              << ". Logs and PNGs: " << std::filesystem::absolute(log.directory) << '\n';
    if (!options.live) std::cout << "This result does not certify real-world aiming or hit performance.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Task " << task << " failed safely: " << error.what() << '\n';
    return 1;
  }
}
