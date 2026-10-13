// Reproducible closed-loop synthetic-input test of the SAME C++ Solver/Controller
// used by the task programs. This is NOT a detector benchmark or hardware result.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/controller.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/plotter.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
constexpr double PI = 3.14159265358979323846;
constexpr double GRAVITY = 9.7833;
constexpr double DT = 0.01;
constexpr double SENSOR_DELAY = 0.02;
constexpr double ACTUATION_DELAY = 0.025;
constexpr int IMAGE_WIDTH = 1440, IMAGE_HEIGHT = 1080;

double wrap(double x) { return std::remainder(x, 2.0 * PI); }
double square(double x) { return x * x; }
std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}
auto_aim::AimTime sim_time(double seconds)
{
  return auto_aim::AimTime{} + std::chrono::seconds(100) +
         std::chrono::duration_cast<auto_aim::AimClock::duration>(
           std::chrono::duration<double>(seconds));
}

struct Options
{
  std::string scenario = "low", config = "sim/camera.yaml", output = "artifacts/sim";
  double speed = 3.0, duration = 15.0, noise = 0.25, dropout = 0.02;
  unsigned seed = 1;
  bool all = false, stress = false, plot = false, assertions = true, self_test = false;
};

// Ground truth exists only in this synthetic scene and the independent scorer.
// Neither IDs nor these coordinates are passed directly to Controller::step.
struct Plate
{
  Eigen::Vector3d position;
  double yaw;
  int truth_index;
  auto_aim::ArmorType type;
  auto_aim::ArmorName name;
};

bool follows(const Options & options)
{
  return options.scenario == "follow" || options.scenario == "follow_turn";
}

double truth_speed(const Options & options, double t)
{
  if (follows(options)) return 0.0;
  if (options.scenario == "low_to_medium")
    return (options.speed < 0.0 ? -1.0 : 1.0) * (t < options.duration / 2.0 ? 4.0 : 7.0);
  return options.speed;
}

double truth_phase(const Options & options, double t)
{
  if (options.scenario != "low_to_medium") return options.speed * t;
  const double split = options.duration / 2.0;
  return (options.speed < 0.0 ? -1.0 : 1.0) *
         (4.0 * std::min(t, split) + 7.0 * std::max(0.0, t - split));
}

std::vector<Plate> scene(const Options & options, double t)
{
  if (follows(options)) {
    const double bearing = options.scenario == "follow_turn"
      ? PI + 2.0 * PI * t / 15.0 : PI + 0.48 * std::sin(0.55 * t);
    const double elevation = 18.0 * PI / 180.0 * std::sin(0.72 * t);
    const Eigen::Vector3d position(
      3.0 * std::cos(elevation) * std::cos(bearing),
      3.0 * std::cos(elevation) * std::sin(bearing), 3.0 * std::sin(elevation));
    return {{position, bearing, 0, auto_aim::small, auto_aim::four}};
  }
  const bool outpost = options.scenario == "outpost";
  const int count = outpost ? 3 : 4;
  const double radius = outpost ? 0.2765 : 0.25;
  std::vector<Plate> result;
  for (int i = 0; i < count; ++i) {
    const double phase = truth_phase(options, t) + i * 2.0 * PI / count;
    const double r = radius + ((!outpost && i % 2) ? 0.02 : 0.0);
    const double height = outpost ? 0.10 * i : 0.10 * (i % 2);
    result.push_back({Eigen::Vector3d(3.0 - r * std::cos(phase), -r * std::sin(phase),
                                    0.10 + height),
                      wrap(phase), i, outpost ? auto_aim::special : auto_aim::small,
                      outpost ? auto_aim::outpost : auto_aim::four});
  }
  return result;
}

Eigen::Matrix3d plate_rotation(const Plate & plate)
{
  const double tilt = (plate.name == auto_aim::outpost ? -15.0 : 15.0) * PI / 180.0;
  return (Eigen::AngleAxisd(plate.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY())).toRotationMatrix();
}

struct Gimbal
{
  double yaw = 0.0, pitch = 0.0, yaw_velocity = 0.0, pitch_velocity = 0.0;
  void advance(const auto_aim::AimCommand & command)
  {
    const double desired_yaw_velocity = command.control
      ? std::clamp(wrap(command.yaw - yaw) / 0.035, -6.0, 6.0) : 0.0;
    const double desired_pitch_velocity = command.control
      ? std::clamp((command.pitch - pitch) / 0.035, -2.0, 2.0) : 0.0;
    yaw_velocity += std::clamp(desired_yaw_velocity - yaw_velocity, -40.0 * DT, 40.0 * DT);
    pitch_velocity += std::clamp(desired_pitch_velocity - pitch_velocity, -30.0 * DT, 30.0 * DT);
    yaw = wrap(yaw + yaw_velocity * DT);
    pitch = std::clamp(pitch + pitch_velocity * DT, -20.0 * PI / 180.0, 20.0 * PI / 180.0);
  }
  Eigen::Quaterniond attitude() const
  {
    // Positive protocol pitch points DOWN: Ry(+pitch) sends +x toward -z.
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  }
};

struct Frame
{
  double capture_time = 0.0;
  std::vector<auto_aim::Armor> observations;
  std::vector<std::vector<cv::Point2f>> truth_polygons;
  int corrupt_observations = 0;
};

Frame capture(
  const Options & options, double t, const Gimbal & gimbal, auto_aim::Solver & solver,
  std::mt19937 & random)
{
  Frame frame;
  frame.capture_time = t;
  solver.set_R_gimbal2world(gimbal.attitude());
  std::normal_distribution<double> noise(0.0, options.noise);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const bool blackout = options.stress && std::fmod(t, 5.0) > 3.0 && std::fmod(t, 5.0) < 3.28;
  const bool lost = blackout || uniform(random) < options.dropout;
  for (const auto & plate : scene(options, t)) {
    const double view = std::atan2(plate.position.y(), plate.position.x());
    if (std::abs(wrap(plate.yaw - view)) > 1.10) continue;
    if (!solver.world2pixel(plate.position)) continue;
    auto pixels = solver.reproject_armor(plate.position, plate.yaw, plate.type, plate.name);
    bool visible = pixels.size() == 4;
    for (const auto & pixel : pixels) {
      visible = visible && std::isfinite(pixel.x) && std::isfinite(pixel.y) &&
                pixel.x >= 4 && pixel.x < IMAGE_WIDTH - 4 && pixel.y >= 4 && pixel.y < IMAGE_HEIGHT - 4;
    }
    if (!visible) continue;
    frame.truth_polygons.push_back(pixels);
    if (lost) continue;
    for (auto & pixel : pixels) {
      pixel.x += static_cast<float>(noise(random));
      pixel.y += static_cast<float>(noise(random));
    }
    const bool corrupted = options.stress && uniform(random) < 0.03;
    if (corrupted) {
      // Preserve the shape but inject a geometrically plausible bad detection.
      const float shift = uniform(random) < 0.5 ? -95.0f : 95.0f;
      for (auto & pixel : pixels) pixel.x += shift;
      ++frame.corrupt_observations;
    }
    auto_aim::Armor observed;
    observed.color = auto_aim::red;
    observed.type = plate.type;
    observed.name = plate.name;
    observed.confidence = 0.95;
    observed.points = pixels;
    observed.center = cv::Point2f(0.0f, 0.0f);
    for (const auto & pixel : pixels) observed.center += pixel * 0.25f;
    observed.box = cv::boundingRect(pixels);
    try {
      if (solver.solve(observed) && observed.xyz_in_world.allFinite() && observed.ypr_in_world.allFinite())
        frame.observations.push_back(std::move(observed));
    } catch (const cv::Exception &) {
      // A rejected PnP sample is a missing observation, never a truth replacement.
    }
  }
  return frame;
}

struct Impact
{
  bool hit = false;
  int truth_plate = -1;
  double flight_time = 0.0;
  double miss_distance = 999.0;
};

Impact score_shot(const Options & options, double launch_time, const Gimbal & gimbal, double speed,
                  double h = 0.001)
{
  // 17 mm projectile; test collision against the finite armor face enlarged by
  // projectile radius. No estimated target or selected_armor is used here.
  constexpr double projectile_radius = 0.0085;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity(speed * std::cos(gimbal.pitch) * std::cos(gimbal.yaw),
                           speed * std::cos(gimbal.pitch) * std::sin(gimbal.yaw),
                           -speed * std::sin(gimbal.pitch));
  const Eigen::Vector3d acceleration(0.0, 0.0, -GRAVITY);
  Impact result;
  for (double elapsed = 0.0; elapsed < 0.8; elapsed += h) {
    const Eigen::Vector3d next_position = position + h * velocity + 0.5 * h * h * acceleration;
    const auto previous_plates = scene(options, launch_time + elapsed);
    const auto next_plates = scene(options, launch_time + elapsed + h);
    for (std::size_t i = 0; i < next_plates.size(); ++i) {
      const Eigen::Vector3d old_local = plate_rotation(previous_plates[i]).transpose() *
                                       (position - previous_plates[i].position);
      const Eigen::Vector3d new_local = plate_rotation(next_plates[i]).transpose() *
                                       (next_position - next_plates[i].position);
      // Only entry through the outward/front face counts; do not award a hit
      // on an occluded rear plate after travelling through the robot body.
      if (!(old_local.x() <= 0.0 && new_local.x() >= 0.0)) continue;
      const double denominator = old_local.x() - new_local.x();
      if (std::abs(denominator) < 1e-12) continue;
      const double fraction = std::clamp(old_local.x() / denominator, 0.0, 1.0);
      const double event_time = elapsed + fraction * h;
      const auto event_plate = scene(options, launch_time + event_time)[i];
      const Eigen::Vector3d event_position = position + fraction * (next_position - position);
      const Eigen::Vector3d local = plate_rotation(event_plate).transpose() *
                                    (event_position - event_plate.position);
      const double width = event_plate.type == auto_aim::special ? 0.129 : 0.135;
      const double outside_y = std::max(0.0, std::abs(local.y()) - width * 0.5);
      const double outside_z = std::max(0.0, std::abs(local.z()) - 0.028);
      const double outside_distance = std::hypot(outside_y, outside_z);
      result.miss_distance = std::min(result.miss_distance, outside_distance);
      if (outside_distance <= projectile_radius) {
        result.hit = true;
        result.truth_plate = static_cast<int>(i);
        result.flight_time = event_time;
        return result;
      }
    }
    position = next_position;
    velocity += acceleration * h;
  }
  return result;
}

bool self_test(const std::string & camera_config)
{
  Options stationary;
  stationary.scenario = "low";
  stationary.speed = 0.0;
  stationary.noise = 0.0;
  stationary.dropout = 0.0;
  Gimbal gimbal;
  const auto position = scene(stationary, 0.0).front().position;
  const double distance = position.head<2>().norm(), speed = 22.0;
  const double v2 = speed * speed;
  const double discriminant = v2 * v2 - GRAVITY * (GRAVITY * distance * distance + 2.0 * position.z() * v2);
  gimbal.pitch = -std::atan((v2 - std::sqrt(discriminant)) / (GRAVITY * distance));
  const auto hit = score_shot(stationary, 0.0, gimbal, speed);
  const auto refined = score_shot(stationary, 0.0, gimbal, speed, 0.0005);
  const auto repeated = score_shot(stationary, 0.0, gimbal, speed);
  Gimbal offset = gimbal;
  offset.yaw += 0.20;
  const auto miss = score_shot(stationary, 0.0, offset, speed);
  auto_aim::Solver solver(camera_config);
  std::mt19937 random(1234);
  const auto frame = capture(stationary, 0.0, Gimbal{}, solver, random);
  const bool pnp_round_trip = frame.observations.size() == 1 &&
    (frame.observations.front().xyz_in_world - position).norm() < 1e-4;
  // Deliberately label a horizontal top panel as "outpost" to verify the
  // middle-panel geometry gate, rather than relying on perfect class labels.
  auto_aim::Armor top_panel;
  top_panel.name = auto_aim::outpost;
  top_panel.type = auto_aim::special;
  top_panel.color = auto_aim::red;
  top_panel.confidence = 0.99;
  const Eigen::Matrix3d top_rotation = Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const std::array<Eigen::Vector3d, 4> top_local{{
    {0.0, 0.129 / 2.0, 0.056 / 2.0}, {0.0, -0.129 / 2.0, 0.056 / 2.0},
    {0.0, -0.129 / 2.0, -0.056 / 2.0}, {0.0, 0.129 / 2.0, -0.056 / 2.0}}};
  for (const auto & local : top_local) {
    const auto pixel = solver.world2pixel(Eigen::Vector3d(3.0, 0.0, 0.5) + top_rotation * local);
    if (pixel) top_panel.points.push_back(*pixel);
  }
  const bool top_panel_rejected = top_panel.points.size() == 4 && !solver.solve(top_panel);
  const json checks = {
    {"stationary_front_plate_hit", hit.hit && hit.truth_plate == 0},
    {"known_yaw_offset_misses", !miss.hit},
    {"integration_step_refinement", refined.hit && std::abs(refined.flight_time - hit.flight_time) < 0.002},
    {"collision_deterministic", repeated.hit == hit.hit && repeated.flight_time == hit.flight_time},
    {"exact_project_solve_round_trip", pnp_round_trip},
    {"outpost_top_panel_rejected", top_panel_rejected}};
  bool passed = true;
  for (const auto & item : checks.items()) passed = passed && item.value().get<bool>();
  std::cout << json({{"evidence", "SYNTHETIC SCORER SELF-TEST / NOT HARDWARE"},
                     {"passed", passed}, {"checks", checks}}).dump(2) << '\n';
  return passed;
}

void text(cv::Mat & image, const std::string & value, int x, int y, cv::Scalar color, double size = 0.55)
{
  cv::putText(image, value, {x, y}, cv::FONT_HERSHEY_SIMPLEX, size, color, 1, cv::LINE_AA);
}

void chart(cv::Mat & image, cv::Rect rectangle, const std::vector<double> & estimated,
           const std::vector<double> & truth, const std::string & title)
{
  cv::rectangle(image, rectangle, {65, 71, 83}, 1);
  text(image, title, rectangle.x + 8, rectangle.y + 19, {225, 225, 230}, 0.45);
  if (estimated.empty()) return;
  double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
  for (const auto * values : {&estimated, &truth})
    for (double value : *values)
      if (std::isfinite(value)) { minimum = std::min(minimum, value); maximum = std::max(maximum, value); }
  if (!std::isfinite(minimum)) return;
  const double range = std::max(0.2, maximum - minimum);
  minimum -= range * 0.1;
  maximum += range * 0.1;
  auto draw = [&](const std::vector<double> & values, cv::Scalar color) {
    std::vector<cv::Point> points;
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!std::isfinite(values[i])) continue;
      points.emplace_back(rectangle.x + 8 + static_cast<int>((rectangle.width - 16) *
                            double(i) / std::max<std::size_t>(1, values.size() - 1)),
        rectangle.y + 32 + static_cast<int>((rectangle.height - 43) *
                            (maximum - values[i]) / (maximum - minimum)));
    }
    if (points.size() > 1) cv::polylines(image, points, false, color, 1, cv::LINE_AA);
  };
  draw(truth, {240, 190, 60});
  draw(estimated, {90, 230, 120});
}

void screenshot(const fs::path & path, const Options & options, double now, const Frame & frame,
                const auto_aim::AimCommand & command, const Gimbal & gimbal, int shots, int hits,
                const std::vector<double> & omega, const std::vector<double> & truth_omega,
                const std::vector<double> & yaw, const std::vector<double> & actual_yaw,
                const std::vector<double> & pitch, const std::vector<double> & actual_pitch)
{
  cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH + 420, CV_8UC3, cv::Scalar(20, 23, 29));
  for (int x = 0; x < IMAGE_WIDTH; x += 80) cv::line(image, {x, 0}, {x, IMAGE_HEIGHT}, {34, 38, 44});
  for (int y = 0; y < IMAGE_HEIGHT; y += 80) cv::line(image, {0, y}, {IMAGE_WIDTH, y}, {34, 38, 44});
  for (const auto & polygon : frame.truth_polygons) {
    std::vector<cv::Point> points;
    for (const auto & point : polygon) points.emplace_back(cvRound(point.x), cvRound(point.y));
    cv::fillConvexPoly(image, points, {35, 40, 75});
    cv::polylines(image, points, true, {240, 190, 60}, 2, cv::LINE_AA);
  }
  for (const auto & observed : frame.observations) {
    std::vector<cv::Point> points;
    for (const auto & point : observed.points) points.emplace_back(cvRound(point.x), cvRound(point.y));
    cv::polylines(image, points, true, {90, 230, 120}, 2, cv::LINE_AA);
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      cv::circle(image, points[i], 3, {90, 230, 120}, -1);
      text(image, std::to_string(i), points[i].x + 5, points[i].y - 5, {230, 230, 230}, 0.42);
    }
    text(image, options.scenario == "outpost" ? "red outpost" : "red four",
         cvRound(observed.center.x) - 45, cvRound(observed.center.y) - 35, {90, 230, 120});
  }
  // A magnified copy of the ACTUAL sampled pixels, not a separately generated
  // attractive mock-up. This makes the small 3 m target legible in the evidence.
  if (!frame.observations.empty()) {
    const auto center = frame.observations.front().center;
    const int left = std::clamp(cvRound(center.x) - 110, 0, IMAGE_WIDTH - 220);
    const int top = std::clamp(cvRound(center.y) - 82, 104, IMAGE_HEIGHT - 164);
    cv::Mat enlarged;
    cv::resize(image(cv::Rect(left, top, 220, 164)).clone(), enlarged, cv::Size(440, 328), 0, 0, cv::INTER_NEAREST);
    enlarged.copyTo(image(cv::Rect(28, 154, 440, 328)));
    cv::rectangle(image, cv::Rect(28, 154, 440, 328), {90, 230, 120}, 1);
    text(image, "2x crop of actual sampled synthetic pixels", 28, 138, {180, 190, 200}, 0.5);
  }
  cv::drawMarker(image, {720, 540}, {220, 220, 220}, cv::MARKER_CROSS, 24, 1);
  cv::rectangle(image, {0, 0, IMAGE_WIDTH, 104}, {15, 18, 24}, cv::FILLED);
  text(image, "SYNTHETIC INPUT / NOT HARDWARE RESULT", 28, 35, {65, 205, 255}, 0.8);
  text(image, "Actual C++ Solver -> Controller -> delayed finite-bandwidth gimbal -> ballistic collision",
       28, 70, {210, 215, 225}, 0.62);
  const int x = IMAGE_WIDTH + 18;
  text(image, options.scenario + " | seed " + std::to_string(options.seed), x, 38, {230, 230, 240}, 0.7);
  text(image, "simulated time: " + number(now, 2) + " s", x, 76, {210, 215, 225});
  text(image, "state: " + command.state, x, 111, {90, 230, 120});
  text(image, "gate: " + command.reason, x, 141, {210, 215, 225}, 0.46);
  text(image, "command yaw: " + number(command.yaw) + " rad", x, 180, {210, 215, 225});
  text(image, "command pitch: " + number(command.pitch) + " rad", x, 212, {210, 215, 225});
  text(image, "actual yaw: " + number(gimbal.yaw), x, 244, {240, 190, 60});
  text(image, "estimated omega: " + number(command.omega), x, 280, {90, 230, 120});
  text(image, "TRUTH omega: " + number(truth_speed(options, now)),
       x, 312, {240, 190, 60});
  text(image, "shots / synthetic hits: " + std::to_string(shots) + " / " + std::to_string(hits),
       x, 350, {210, 215, 225});
  text(image, "pixel sigma: " + number(options.noise) + " px", x, 385, {210, 215, 225});
  text(image, "dropout: " + number(options.dropout * 100, 1) + "%", x, 416, {210, 215, 225});
  text(image, "cyan: independent truth / feedback", x, 451, {240, 190, 60}, 0.46);
  text(image, "green: actual controller / observations", x, 479, {90, 230, 120}, 0.46);
  chart(image, {x, 505, 380, 160}, omega, truth_omega, "Estimated omega vs independent truth (rad/s)");
  chart(image, {x, 682, 380, 160}, yaw, actual_yaw, "Command yaw vs simulated feedback (rad)");
  chart(image, {x, 859, 380, 160}, pitch, actual_pitch, "Command pitch vs simulated feedback (rad)");
  text(image, "Projected + noisy keypoints, not a YOLO accuracy test. No real camera or robot used.",
       28, IMAGE_HEIGHT - 30, {180, 185, 195}, 0.64);
  if (!cv::imwrite(path.string(), image)) throw std::runtime_error("Cannot save " + path.string());
}

double rms(const std::vector<double> & values)
{
  if (values.empty()) return 0.0;
  return std::sqrt(std::inner_product(values.begin(), values.end(), values.begin(), 0.0) / values.size());
}
double percentile(std::vector<double> values, double quantile)
{
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(quantile * (values.size() - 1))];
}

json run(const Options & options)
{
  fs::create_directories(options.output);
  auto_aim::Solver solver(options.config);
  auto_aim::ControllerConfig config;
  config.task = follows(options) ? 1 : options.scenario == "outpost" ? 3 : 2;
  config.allow_fire = config.task != 1;
  config.execution_delay = ACTUATION_DELAY;
  auto_aim::Controller controller(config);
  Gimbal gimbal;
  if (config.task == 1) gimbal.yaw = PI;
  auto_aim::Feedback feedback;
  feedback.fire_enabled = config.allow_fire;
  auto_aim::AimCommand active_command, command;
  active_command.yaw = gimbal.yaw;
  std::deque<Frame> delayed_frames;
  std::deque<std::pair<double, auto_aim::AimCommand>> delayed_commands;
  std::vector<double> omega_errors, settled_errors, yaw_errors, pitch_errors, fire_times;
  std::vector<double> pending_hit_arrivals;
  std::optional<double> first_fire_time;
  std::vector<double> history_omega, history_truth, history_yaw, history_actual_yaw;
  std::vector<double> history_pitch, history_actual_pitch;
  int shots = 0, hits = 0, geometric_hits = 0, late_impacts = 0;
  int commands = 0, stale_fires = 0, finite_faults = 0, pitch_faults = 0;
  int samples = 0, estimated_samples = 0, controlled_samples = 0;
  int captured_frames = 0, empty_frames = 0, injected_outliers = 0;
  double maximum_yaw_step = 0.0, minimum_pitch = 0.0, maximum_pitch = 0.0;
  double actual_unwrapped_yaw = gimbal.yaw;
  double minimum_unwrapped_yaw = gimbal.yaw, maximum_unwrapped_yaw = gimbal.yaw;
  int yaw_wrap_crossings = 0;
  std::mt19937 random(options.seed);
  Frame displayed;
  double last_capture = -1.0;
  std::ofstream telemetry(fs::path(options.output) / "telemetry.csv");
  std::ofstream events(fs::path(options.output) / "shots.jsonl");
  telemetry << "time,cmd_yaw,cmd_pitch,actual_yaw,actual_pitch,estimated_omega,truth_omega,"
               "observation_age,control,fire,shots_confirmed,synthetic_hits,state,gate\n";
  telemetry << std::setprecision(12);
  std::unique_ptr<tools::Plotter> plotter;
  if (options.plot) plotter = std::make_unique<tools::Plotter>();
  const int ticks = static_cast<int>(std::ceil(options.duration / DT));
  for (int tick = 0; tick <= ticks; ++tick) {
    const double t = tick * DT;
    while (!delayed_commands.empty() && delayed_commands.front().first <= t + 1e-9) {
      active_command = delayed_commands.front().second;
      delayed_commands.pop_front();
      if (active_command.fire) {
        ++shots;
        ++feedback.bullet_count;
        const auto impact = score_shot(options, t, gimbal, feedback.bullet_speed);
        const double arrival = t + impact.flight_time;
        const double deadline = config.task == 3 && first_fire_time
          ? *first_fire_time + 30.0 : std::numeric_limits<double>::infinity();
        const bool within_window = arrival <= deadline + 1e-9;
        geometric_hits += impact.hit;
        late_impacts += impact.hit && !within_window;
        // Queue the collision event for its physical arrival. In particular,
        // launching before a deadline does not award an impact after it.
        if (impact.hit && within_window) pending_hit_arrivals.push_back(arrival);
        events << json({{"launch_time", t}, {"actual_yaw", gimbal.yaw},
                        {"actual_pitch", gimbal.pitch}, {"impact_time", arrival},
                        {"geometric_hit", impact.hit},
                        {"hit", impact.hit && within_window && arrival <= options.duration + 1e-9},
                        {"within_scoring_window", within_window},
                        {"truth_plate", impact.truth_plate}, {"flight_time", impact.flight_time},
                        {"miss_distance_m", impact.miss_distance},
                        {"evidence", "SYNTHETIC COLLISION, NOT HARDWARE"}}).dump() << '\n';
      }
      active_command.fire = false;
    }
    for (auto it = pending_hit_arrivals.begin(); it != pending_hit_arrivals.end();) {
      if (*it <= t + 1e-9) { ++hits; it = pending_hit_arrivals.erase(it); }
      else ++it;
    }
    const double previous_actual_yaw = gimbal.yaw;
    gimbal.advance(active_command);
    const double actual_yaw_increment = wrap(gimbal.yaw - previous_actual_yaw);
    actual_unwrapped_yaw += actual_yaw_increment;
    minimum_unwrapped_yaw = std::min(minimum_unwrapped_yaw, actual_unwrapped_yaw);
    maximum_unwrapped_yaw = std::max(maximum_unwrapped_yaw, actual_unwrapped_yaw);
    maximum_yaw_step = std::max(maximum_yaw_step, std::abs(actual_yaw_increment));
    if (std::abs(gimbal.yaw - previous_actual_yaw) > PI) ++yaw_wrap_crossings;
    minimum_pitch = std::min(minimum_pitch, gimbal.pitch);
    maximum_pitch = std::max(maximum_pitch, gimbal.pitch);
    if (tick % 2 == 0) {
      auto frame = capture(options, t, gimbal, solver, random);
      ++captured_frames;
      empty_frames += frame.observations.empty();
      injected_outliers += frame.corrupt_observations;
      delayed_frames.push_back(std::move(frame));
    }
    std::vector<auto_aim::Armor> observations;
    while (!delayed_frames.empty() && delayed_frames.front().capture_time + SENSOR_DELAY <= t + 1e-9) {
      displayed = std::move(delayed_frames.front());
      delayed_frames.pop_front();
      last_capture = displayed.capture_time;
      observations = displayed.observations;
    }
    feedback.yaw = gimbal.yaw;
    feedback.pitch = gimbal.pitch;
    command = controller.step(observations, sim_time(last_capture), sim_time(t), feedback);
    delayed_commands.emplace_back(t + ACTUATION_DELAY, command);
    if (command.fire) {
      if (!first_fire_time) first_fire_time = t;
      ++commands;
      fire_times.push_back(t);
      if (command.observation_age > config.measurement_timeout + 1e-9) ++stale_fires;
    }
    if (!std::isfinite(command.yaw) || !std::isfinite(command.pitch) || !std::isfinite(command.omega))
      ++finite_faults;
    if (std::abs(command.pitch) > config.pitch_limit + 1e-9) ++pitch_faults;
    if (t > 2.0) {
      ++samples;
      controlled_samples += command.control;
      const bool estimate_present = command.state == "tracking" || command.state == "acquiring" ||
                                    command.state == "temporary_loss" || command.state == "following";
      if (estimate_present) {
        ++estimated_samples;
        if (config.task != 1) {
          const double error = std::abs(command.omega - truth_speed(options, t));
          omega_errors.push_back(error);
          const bool recovering = options.stress && std::fmod(t, 5.0) > 3.0 && std::fmod(t, 5.0) < 4.28;
          const bool changing_stage = options.scenario == "low_to_medium" &&
            t >= options.duration / 2.0 && t < options.duration / 2.0 + 2.0;
          if (!recovering && !changing_stage) settled_errors.push_back(error);
        }
        else {
          const auto target = scene(options, t).front().position;
          yaw_errors.push_back(std::abs(wrap(gimbal.yaw - std::atan2(target.y(), target.x()))));
          pitch_errors.push_back(std::abs(gimbal.pitch + std::atan2(target.z(), target.head<2>().norm())));
        }
      }
    }
    const double ground_truth_speed = truth_speed(options, t);
    history_omega.push_back(command.omega);
    history_truth.push_back(ground_truth_speed);
    history_yaw.push_back(command.yaw);
    history_actual_yaw.push_back(gimbal.yaw);
    history_pitch.push_back(command.pitch);
    history_actual_pitch.push_back(gimbal.pitch);
    telemetry << t << ',' << command.yaw << ',' << command.pitch << ',' << gimbal.yaw << ','
      << gimbal.pitch << ',' << command.omega << ',' << ground_truth_speed << ',' << command.observation_age
      << ',' << command.control << ',' << command.fire << ',' << shots << ',' << hits << ','
      << command.state << ',' << command.reason << '\n';
    if (plotter) plotter->plot({{"time", t}, {"cmd_yaw", command.yaw}, {"cmd_pitch", command.pitch},
      {"estimated_omega", command.omega}, {"truth_omega", ground_truth_speed},
      {"simulated_feedback_yaw", gimbal.yaw}, {"simulated_feedback_pitch", gimbal.pitch}});
    if (tick == ticks / 2 || tick == ticks) {
      screenshot(fs::path(options.output) / (tick == ticks ? "final.png" : "tracking.png"),
        options, t, displayed, command, gimbal, shots, hits, history_omega, history_truth,
        history_yaw, history_actual_yaw, history_pitch, history_actual_pitch);
    }
  }
  double minimum_interval = options.duration;
  for (std::size_t i = 1; i < fire_times.size(); ++i)
    minimum_interval = std::min(minimum_interval, fire_times[i] - fire_times[i - 1]);
  const double tolerance = options.scenario == "low" ? 0.6 :
    (options.scenario == "medium" || options.scenario == "low_to_medium") ? 1.0 : 0.4;
  const double coverage = samples ? double(estimated_samples) / samples : 0.0;
  const double requested_interval = config.task == 3 ? 1.0 / 12.0 : 1.0 / 5.0;
  json checks = {
    {"finite_commands", finite_faults == 0}, {"pitch_within_20_degrees", pitch_faults == 0},
    {"no_stale_fire", stale_fires == 0}, {"tracking_coverage", coverage >= (options.stress ? 0.70 : 0.90)},
    {"fire_rate_limit", fire_times.size() < 2 || minimum_interval + 1e-8 >= requested_interval},
    {"shot_budget", config.task != 2 || commands <= (options.scenario == "low_to_medium" ? 20 : 10)},
    {"follow_never_fires", config.task != 1 || commands == 0},
    {"follow_engineering_rmse", config.task != 1 || (!yaw_errors.empty() && rms(yaw_errors) <= 0.04 && rms(pitch_errors) <= 0.04)},
    {"gimbal_yaw_step_bound", maximum_yaw_step <= 6.0 * DT + 1e-9},
    {"follow_angular_range_exercised", options.scenario != "follow" || options.duration < 12.0 ||
      (yaw_wrap_crossings >= 2 && minimum_pitch < -17.0 * PI / 180.0 && maximum_pitch > 17.0 * PI / 180.0)},
    {"full_360_degree_follow_exercised", options.scenario != "follow_turn" ||
      (maximum_unwrapped_yaw - minimum_unwrapped_yaw >= 2.0 * PI &&
       minimum_pitch < -17.0 * PI / 180.0 && maximum_pitch > 17.0 * PI / 180.0)},
    {"speed_settled_rmse", config.task == 1 || (!settled_errors.empty() && rms(settled_errors) <= tolerance)},
    {"speed_settled_p95", config.task == 1 || (!settled_errors.empty() && percentile(settled_errors, 0.95) <= tolerance)},
    {"speed_settled_max", config.task == 1 || (!settled_errors.empty() && percentile(settled_errors, 1.0) <= tolerance)},
    {"engagement_exercised", config.task == 1 || shots > 0},
    {"nonzero_synthetic_hits", config.task == 1 || hits > 0},
    {"stage_transition", options.scenario != "low_to_medium" || command.stage == 2}};
  checks["outpost_30s_fire_budget"] = config.task != 3 || fire_times.size() < 2 ||
    fire_times.back() - fire_times.front() < 30.0 + 1e-8;
  checks["outpost_budget_window_exercised"] = config.task != 3 || options.duration < 35.0 ||
    (first_fire_time && options.duration >= *first_fire_time + 30.0 + 0.2);
  bool passed = true;
  for (const auto & check : checks.items()) passed = passed && check.value().get<bool>();
  const json result = {
    {"schema", "aim-simulator-v1"}, {"evidence", "SYNTHETIC INPUT / NOT HARDWARE RESULT"},
    {"not_tested", {"YOLO detection accuracy", "USB camera", "serial hardware", "actual robot hit rate"}},
    {"scenario", options.scenario}, {"seed", options.seed}, {"duration_s", options.duration},
    {"truth_speed_rad_s", truth_speed(options, options.duration)}, {"pixel_noise_sigma", options.noise},
    {"dropout_probability", options.dropout}, {"stress", options.stress},
    {"captured_frames", captured_frames}, {"empty_frames", empty_frames},
    {"injected_outlier_observations", injected_outliers}, {"tracking_coverage", coverage},
    {"control_coverage", samples ? double(controlled_samples) / samples : 0.0},
    {"speed_error_rmse_rad_s", rms(omega_errors)}, {"speed_error_p95_rad_s", percentile(omega_errors, 0.95)},
    {"speed_error_max_rad_s", percentile(omega_errors, 1.0)},
    {"settled_speed_rmse_rad_s", rms(settled_errors)},
    {"settled_speed_p95_rad_s", percentile(settled_errors, 0.95)},
    {"settled_speed_max_rad_s", percentile(settled_errors, 1.0)},
    {"settled_metric_window", "after initial 2s; excluding 2s after stage change and blackout+1s recovery; all-frame metrics retained"},
    {"final_stage", command.stage},
    {"speed_rmse_limit_rad_s", config.task == 1 ? 0.0 : tolerance},
    {"follow_yaw_rmse_rad", rms(yaw_errors)}, {"follow_pitch_rmse_rad", rms(pitch_errors)},
    {"follow_sanity_limit_rad", 0.04}, {"follow_limit_source", "self-defined engineering sanity bound, not assessment rule"},
    {"maximum_actual_yaw_step_rad", maximum_yaw_step}, {"yaw_wrap_crossings", yaw_wrap_crossings},
    {"actual_unwrapped_yaw_span_rad", maximum_unwrapped_yaw - minimum_unwrapped_yaw},
    {"minimum_actual_pitch_rad", minimum_pitch}, {"maximum_actual_pitch_rad", maximum_pitch},
    {"fire_pulses", commands}, {"shots_confirmed", shots}, {"synthetic_hits", hits},
    {"geometric_hits_including_uncredited", geometric_hits}, {"impacts_after_deadline", late_impacts},
    {"impacts_still_in_flight_at_run_end", pending_hit_arrivals.size()},
    {"first_fire_time_s", first_fire_time ? json(*first_fire_time) : json(nullptr)},
    {"outpost_scoring_deadline_s", config.task == 3 && first_fire_time ? json(*first_fire_time + 30.0) : json(nullptr)},
    {"outpost_damage_in_window_hp", config.task == 3 ? std::min(1500, hits * 20) : 0},
    {"scoring_window_contract", "outpost: first controller fire pulse +30s; collision ARRIVAL must be in window and before run end"},
    {"synthetic_hit_rate", shots ? double(hits) / shots : 0.0},
    {"minimum_fire_interval_s", fire_times.size() < 2 ? json(nullptr) : json(minimum_interval)},
    {"collision_model", "1 ms gravity integration; moving finite armor planes; 17 mm projectile"},
    {"gimbal_model", "35 ms first-order velocity demand; velocity and acceleration saturation; 25 ms actuation delay"},
    {"sensor_delay_s", SENSOR_DELAY}, {"checks", checks}, {"passed", passed}};
  std::ofstream metrics(fs::path(options.output) / "metrics.json");
  metrics << result.dump(2) << '\n';
  std::cout << result.dump(2) << '\n';
  return result;
}

Options parse(int argc, char ** argv)
{
  Options options;
  bool duration_supplied = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::invalid_argument("Missing value after " + arg);
      return argv[i];
    };
    if (arg == "--scenario") options.scenario = value();
    else if (arg == "--speed") options.speed = std::stod(value());
    else if (arg == "--duration") { options.duration = std::stod(value()); duration_supplied = true; }
    else if (arg == "--noise") options.noise = std::stod(value());
    else if (arg == "--dropout") options.dropout = std::stod(value());
    else if (arg == "--seed") options.seed = static_cast<unsigned>(std::stoul(value()));
    else if (arg == "--config") options.config = value();
    else if (arg == "--output") options.output = value();
    else if (arg == "--all") options.all = true;
    else if (arg == "--self-test") options.self_test = true;
    else if (arg == "--stress") options.stress = true;
    else if (arg == "--plot") options.plot = true;
    else if (arg == "--no-assert") options.assertions = false;
    else if (arg == "--help") {
      std::cout << "aim_simulator [--scenario follow|follow_turn|low|medium|outpost|low_to_medium] [--speed RAD/S]\n"
                   "  [--seed N] [--duration SECONDS] [--noise PIXELS] [--dropout PROBABILITY]\n"
                   "  [--stress] [--all] [--self-test] [--plot] [--config YAML] [--output DIR] [--no-assert]\n"
                   "All output is SYNTHETIC and must not be called a hardware result.\n";
      std::exit(0);
    } else throw std::invalid_argument("Unknown option " + arg);
  }
  if (options.scenario == "follow_turn" && !duration_supplied) options.duration = 16.0;
  if (!follows(options) && options.scenario != "low" && options.scenario != "medium" &&
      options.scenario != "outpost" && options.scenario != "low_to_medium")
    throw std::invalid_argument("Unknown scenario");
  if (!std::isfinite(options.duration) || !std::isfinite(options.noise) ||
      !std::isfinite(options.dropout) || options.duration <= 2.0 || options.duration > 120.0 || options.noise < 0.0 ||
      options.dropout < 0.0 || options.dropout >= 1.0 || !std::isfinite(options.speed))
    throw std::invalid_argument("Invalid duration/noise/dropout/speed");
  if (options.scenario == "follow_turn" && options.duration < 15.5)
    throw std::invalid_argument("follow_turn requires at least 15.5 s to exercise a complete physical turn");
  return options;
}

int main(int argc, char ** argv)
{
  try {
    const auto options = parse(argc, argv);
    if (options.self_test) return self_test(options.config) ? 0 : 2;
    if (!options.all) return run(options)["passed"].get<bool>() || !options.assertions ? 0 : 2;
    json results = json::array();
    bool passed = true;
    std::vector<std::pair<std::string, double>> cases{{"follow", 0.0}};
    for (double sign : {1.0, -1.0}) {
      for (double speed : {3.0, 4.0, 5.0}) cases.emplace_back("low", sign * speed);
      for (double speed : {5.0, 7.0, 9.0}) cases.emplace_back("medium", sign * speed);
      cases.emplace_back("outpost", sign * 2.5);
      cases.emplace_back("low_to_medium", sign * 4.0);
    }
    for (const auto & [scenario, speed] : cases) {
      Options current = options;
      current.scenario = scenario;
      current.speed = speed;
      if (scenario == "outpost") current.duration = 35.0;
      if (scenario == "low_to_medium") current.duration = 32.0;
      current.output = (fs::path(options.output) / (scenario + "_" + number(speed, 1))).string();
      const auto result = run(current);
      passed = passed && result["passed"].get<bool>();
      results.push_back(result);
    }
    std::ofstream summary(fs::path(options.output) / "suite.json");
    summary << json({{"passed", passed}, {"runs", results},
                     {"evidence", "SYNTHETIC INPUT / NOT HARDWARE RESULT"}}).dump(2) << '\n';
    return passed || !options.assertions ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "Simulator error: " << error.what() << '\n';
    return 1;
  }
}
