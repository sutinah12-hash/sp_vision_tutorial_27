#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "tasks/auto_aim/controller.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace
{
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
auto_aim::AimTime at(double t)
{
  return auto_aim::AimTime{} +
    std::chrono::duration_cast<auto_aim::AimClock::duration>(std::chrono::duration<double>(t + 1));
}
auto_aim::Armor observation(double bearing, double elevation = 0)
{
  auto_aim::Armor a;
  a.color = auto_aim::Color::red;
  a.name = auto_aim::ArmorName::three;
  a.type = auto_aim::ArmorType::small;
  a.confidence = 0.99;
  a.xyz_in_world = {3 * std::cos(bearing), 3 * std::sin(bearing), 3 * std::tan(elevation)};
  a.ypr_in_world = {bearing, 0, 0};
  a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
  return a;
}
}
int main()
{
  try {
    auto_aim::ControllerConfig cfg;
    cfg.allow_fire = true;
    auto_aim::Controller follow(cfg);
    auto_aim::Feedback fb;
    fb.fire_enabled = true;
    fb.yaw = 3.13;
    auto cmd = follow.step({observation(-3.13)}, at(0), at(0), fb);
    check(cmd.control && !cmd.fire, "task1 is tracking only");
    check(std::abs(tools::limit_rad(cmd.yaw - fb.yaw)) <= cfg.yaw_rate_limit * .01 + 1e-9,
          "short-path yaw rate limit");
    fb.yaw = cmd.yaw;
    cmd = follow.step({observation(-3.12, .8)}, at(.01), at(.01), fb);
    check(std::abs(cmd.pitch) <= cfg.pitch_limit && !cmd.fire, "pitch clamp");
    cmd = follow.step({}, at(.3), at(.3), fb);
    check(!cmd.control && !cmd.fire, "stale observation fail closed");
    auto wrong = observation(0); wrong.color = auto_aim::Color::blue;
    auto_aim::Controller colors(cfg);
    check(!colors.step({wrong}, at(0), at(0), fb).control, "enemy filtering");
    auto bad = observation(0); bad.xyz_in_world.x() = std::numeric_limits<double>::quiet_NaN();
    check(!colors.step({bad}, at(.01), at(.01), fb).control, "NaN observation");
    check(!colors.step({observation(0)}, at(1), at(.02), fb).control, "future timestamp");
    fb.aim_enabled = false;
    check(!follow.step({observation(0)}, at(.4), at(.4), fb).control, "mode disable");
    fb.aim_enabled = true; fb.transport_ok = false;
    check(!follow.step({observation(0)}, at(.5), at(.5), fb).control, "transport watchdog");
    fb.transport_ok = true;
    cfg.task = 2; cfg.allow_fire = false;
    auto_aim::Controller disarmed(cfg);
    for (int i = 0; i < 50; ++i) {
      auto c = disarmed.step({observation(0)}, at(i * .01), at(i * .01), fb);
      check(!c.fire, "fire explicit opt-in");
    }
    tools::Trajectory zero(0, 3, 0), near(22, 0, 0), unreachable(1, 100, 100);
    check(zero.unsolvable && near.unsolvable && unreachable.unsolvable, "invalid ballistics");
    tools::Trajectory valid(22, 3, .4);
    check(!valid.unsolvable && valid.fly_time > 0, "ballistic solution");
    const double z = 22 * std::sin(valid.pitch) * valid.fly_time -
                     0.5 * 9.794 * valid.fly_time * valid.fly_time;
    check(std::abs(z - .4) < 1e-9, "independent ballistic residual");
    bool threw = false;
    try { cfg.pitch_limit = -1; auto_aim::Controller invalid(cfg); }
    catch (const std::invalid_argument &) { threw = true; }
    check(threw, "invalid config rejected");
    std::cout << "PASS controller safety, follow, timestamps, mode and ballistics\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
