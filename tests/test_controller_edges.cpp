// Pure in-memory boundary tests: this executable never opens a camera or serial port.
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "tasks/auto_aim/controller.hpp"
#include "tools/math_tools.hpp"

namespace
{
void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}
auto_aim::AimTime at(double seconds)
{
  return auto_aim::AimTime{} + std::chrono::seconds(100) +
    std::chrono::duration_cast<auto_aim::AimClock::duration>(std::chrono::duration<double>(seconds));
}
auto_aim::Armor observed(double phase, int count = 4)
{
  int selected = 0;
  double best = 10.0;
  for (int id = 0; id < count; ++id) {
    const double error = std::abs(std::remainder(phase + id * 2.0 * CV_PI / count, 2.0 * CV_PI));
    if (error < best) { selected = id; best = error; }
  }
  const double angle = std::remainder(phase + selected * 2.0 * CV_PI / count, 2.0 * CV_PI);
  const double radius = count == 3 ? 0.2765 : (selected % 2 ? 0.27 : 0.25);
  auto_aim::Armor armor;
  armor.color = auto_aim::red;
  armor.name = count == 3 ? auto_aim::outpost : auto_aim::four;
  armor.type = count == 3 ? auto_aim::special : auto_aim::small;
  armor.confidence = 0.99;
  armor.xyz_in_world = {3.0 - radius * std::cos(angle), -radius * std::sin(angle),
    0.1 + (count == 3 ? 0.1 * selected : 0.1 * (selected % 2))};
  armor.ypr_in_world = {angle, 0.0, 0.0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  return armor;
}

auto_aim::ControllerConfig configuration(int task)
{
  auto_aim::ControllerConfig config;
  config.task = task;
  config.allow_fire = task != 1;
  return config;
}

struct Replay
{
  explicit Replay(int task) : controller(configuration(task)), count(task == 3 ? 3 : 4),
    omega(task == 3 ? 2.5 : 3.0)
  {
    feedback.fire_enabled = task != 1;
  }
  auto_aim::Controller controller;
  auto_aim::Feedback feedback;
  auto_aim::AimCommand command;
  int count;
  double omega;
  double time = -0.01, phase = 0.0;
  int pulses = 0;
  bool pending = false;
  double first_fire = -1.0, last_fire = -1.0;

  void acknowledge(unsigned delta = 1)
  {
    feedback.bullet_count = static_cast<std::uint16_t>(feedback.bullet_count + delta);
    pending = false;
  }

  void tick(bool provide_observation = true, bool automatic_ack = true)
  {
    if (pending && automatic_ack) acknowledge();
    time += 0.01;
    if (time > 0.0) phase += omega * 0.01;
    feedback.yaw = command.yaw;
    feedback.pitch = command.pitch;
    std::vector<auto_aim::Armor> observations;
    if (provide_observation) observations.push_back(observed(phase, count));
    command = controller.step(observations, at(time), at(time), feedback);
    require(std::isfinite(command.yaw) && std::isfinite(command.pitch), "non-finite command");
    if (command.fire) {
      require(!pending, "more than one unacknowledged pulse");
      pending = true;
      ++pulses;
      if (first_fire < 0.0) first_fire = time;
      last_fire = time;
    }
  }

  void until_pulse()
  {
    for (int i = 0; i < 1200 && pulses == 0; ++i) tick(true, false);
    require(pulses == 1, "test did not reach an initial eligible pulse");
  }
};

void nonfinite_configuration()
{
  using Field = double auto_aim::ControllerConfig::*;
  const std::vector<std::pair<const char *, Field>> fields = {
    {"yaw_offset", &auto_aim::ControllerConfig::yaw_offset},
    {"pitch_offset", &auto_aim::ControllerConfig::pitch_offset},
    {"pitch_limit", &auto_aim::ControllerConfig::pitch_limit},
    {"yaw_rate_limit", &auto_aim::ControllerConfig::yaw_rate_limit},
    {"pitch_rate_limit", &auto_aim::ControllerConfig::pitch_rate_limit},
    {"measurement_timeout", &auto_aim::ControllerConfig::measurement_timeout},
    {"reset_timeout", &auto_aim::ControllerConfig::reset_timeout},
    {"execution_delay", &auto_aim::ControllerConfig::execution_delay},
    {"max_prediction", &auto_aim::ControllerConfig::max_prediction},
    {"fire_angle_tolerance", &auto_aim::ControllerConfig::fire_angle_tolerance},
    {"max_facing_angle", &auto_aim::ControllerConfig::max_facing_angle},
    {"settle_time", &auto_aim::ControllerConfig::settle_time},
    {"stage_change_threshold", &auto_aim::ControllerConfig::stage_change_threshold},
    {"acknowledgement_timeout", &auto_aim::ControllerConfig::acknowledgement_timeout}};
  std::string missed;
  for (const auto & [name, field] : fields) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
      auto config = configuration(2);
      config.*field = value;
      bool rejected = false;
      try { auto_aim::Controller invalid(config); }
      catch (const std::invalid_argument &) { rejected = true; }
      if (!rejected) missed += std::string(name) + (std::isnan(value) ? "=NaN " : "=Inf ");
    }
  }
  require(missed.empty(), "non-finite configurations accepted: " + missed);
}

void timestamp_and_duplicate()
{
  auto_aim::Controller controller(configuration(2));
  auto_aim::Feedback feedback;
  controller.step({observed(0.0)}, at(1.0), at(1.0), feedback);
  const int count = controller.target()->accepted_updates();
  for (int i = 1; i <= 8; ++i) {
    const auto command = controller.step({observed(0.0)}, at(1.0), at(1.0 + 0.01 * i), feedback);
    require(!command.fire, "duplicate observation was eligible for firing");
    require(controller.target()->accepted_updates() == count, "duplicate frame updated target");
  }
  require(!controller.step({}, at(0.9), at(0.9), feedback).control,
    "backward now timestamp did not disable control");
  const auto command = controller.step({}, at(0.95), at(0.95), feedback);
  require(!command.control && !command.fire, "backward time sequence regained control");
}

void prediction_horizon_contract()
{
  auto config = configuration(2);
  config.max_prediction = 6.0;  // Target::predict explicitly supports at most 5 s.
  bool rejected = false;
  try { auto_aim::Controller invalid(config); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "controller horizon exceeds estimator prediction contract");

  config = configuration(2);
  config.reset_timeout = 10.0;
  std::optional<auto_aim::Controller> controller;
  try { controller.emplace(config); }
  catch (const std::invalid_argument &) { return; }  // Constraining reset timeout is also safe.
  auto_aim::Feedback feedback;
  controller->step({observed(0.0)}, at(0.0), at(0.0), feedback);
  const auto result = controller->step({observed(0.2)}, at(6.0), at(6.0), feedback);
  require(!result.fire, "long-gap reinitialization cannot immediately fire");
}

void acknowledgement_timeout_is_latched()
{
  Replay replay(2);
  replay.until_pulse();
  for (int i = 0; i < 80; ++i) replay.tick(true, false);
  require(replay.pulses == 1, "unacknowledged pulse was repeated");
  replay.acknowledge();  // Late acknowledgement must not silently rearm a timed-out run.
  for (int i = 0; i < 300; ++i) replay.tick();
  require(replay.pulses == 1, "late acknowledgement automatically cleared timeout fault");
}

void late_acknowledgement_without_intermediate_poll()
{
  Replay replay(2);
  replay.until_pulse();
  replay.acknowledge();
  // No step() calls occur while the acknowledgement is delayed. The same call
  // that observes the increment must first notice that its deadline expired.
  replay.time += 0.79;
  replay.phase += 0.79 * replay.omega;
  replay.tick();
  for (int i = 0; i < 400; ++i) replay.tick();
  require(replay.pulses == 1, "same-call late acknowledgement escaped timeout latching");
}

void counter_wrap_and_multiple_ack()
{
  Replay replay(2);
  replay.feedback.bullet_count = 65535;
  replay.until_pulse();
  replay.tick();
  require(replay.feedback.bullet_count == 0 && replay.command.shots_confirmed == 1,
    "uint16 counter wrap failed");
  for (int i = 0; i < 1000 && replay.pulses < 2; ++i) replay.tick();
  require(replay.pulses == 2, "test did not reach second pulse after valid wrap");
  replay.acknowledge(2);
  for (int i = 0; i < 300; ++i) replay.tick();
  require(replay.pulses == 2, "multiple-shot acknowledgement failed to latch a fault");
}

void ammunition_stage_budget()
{
  Replay replay(2);
  for (int i = 0; i < 1800 && replay.pulses < 10; ++i) replay.tick();
  require(replay.pulses == 10, "first stage failed to exercise its ten-shot allowance");
  for (int i = 0; i < 300; ++i) replay.tick();
  require(replay.pulses == 10 && replay.command.stage == 1, "first-stage allowance exceeded");
  for (int i = 0; i < 70; ++i) replay.tick(false);
  for (int i = 0; i < 300; ++i) replay.tick();
  require(replay.pulses == 10, "tracking loss replenished first-stage ammunition");
  replay.feedback.aim_enabled = false;
  replay.tick();
  replay.feedback.aim_enabled = true;
  for (int i = 0; i < 300; ++i) replay.tick();
  require(replay.pulses == 10, "mode toggle replenished first-stage ammunition");
  replay.omega = 7.0;
  for (int i = 0; i < 1800 && replay.pulses < 20; ++i) replay.tick();
  require(replay.pulses == 20 && replay.command.stage == 2, "stable distinct speed did not allow second stage");
  replay.omega = 5.0;
  for (int i = 0; i < 500; ++i) replay.tick();
  require(replay.pulses == 20, "third speed replenished completed two-stage allowance");
}

void outpost_time_budget()
{
  Replay replay(3);
  bool budget_gate_exercised = false;
  for (int i = 0; i < 4000; ++i) {
    replay.tick();
    budget_gate_exercised = budget_gate_exercised || replay.command.reason == "outpost_time_budget_exhausted";
  }
  require(replay.first_fire > 0.0 && replay.pulses > 10, "outpost test never exercised firing");
  // This tests the currently documented first-pulse-relative budget. An official
  // assessment start signal is a separate timing contract and must not be inferred.
  require(replay.last_fire - replay.first_fire < 30.0 + 1e-8, "outpost continued beyond time budget");
  require(budget_gate_exercised, "outpost deadline branch was never exercised");
  const int shots = replay.pulses;
  replay.feedback.aim_enabled = false;
  replay.tick();
  replay.feedback.aim_enabled = true;
  for (int i = 0; i < 500; ++i) replay.tick();
  require(replay.pulses == shots, "outpost mode reset replenished time budget");
}
}  // namespace

int main()
{
  int failures = 0;
  const std::vector<std::pair<const char *, std::function<void()>>> tests = {
    {"all floating-point configuration is finite", nonfinite_configuration},
    {"timestamps and duplicate observation identity", timestamp_and_duplicate},
    {"prediction horizon and long-gap contract", prediction_horizon_contract},
    {"acknowledgement timeout remains latched", acknowledgement_timeout_is_latched},
    {"late acknowledgement without an intermediate poll", late_acknowledgement_without_intermediate_poll},
    {"counter wrap and multiple acknowledgement", counter_wrap_and_multiple_ack},
    {"two-stage ammunition budget", ammunition_stage_budget},
    {"outpost 30-second first-pulse budget", outpost_time_budget}};
  for (const auto & [name, test] : tests) {
    try { test(); std::cout << "PASS " << name << '\n'; }
    catch (const std::exception & error) {
      ++failures;
      std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    }
  }
  return failures == 0 ? 0 : 1;
}
