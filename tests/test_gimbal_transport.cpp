// Linux PTY-only tests: every TX byte (including simulated mode 2) stays inside
// an anonymous test pseudo-terminal. No camera or physical serial device is opened.
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

class FakeGimbal
{
public:
  enum class Feed { VALID, ZERO_QUATERNION, NAN_QUATERNION, BAD_CRC, PAUSED };

  explicit FakeGimbal(Feed initial = Feed::VALID) : feed(initial)
  {
    char slave_name[128]{};
    require(::openpty(&master_, &slave_, slave_name, nullptr, nullptr) == 0, "openpty failed");
    try {
      termios tty{};
      require(::tcgetattr(slave_, &tty) == 0, "tcgetattr failed");
      ::cfmakeraw(&tty);
      require(::tcsetattr(slave_, TCSANOW, &tty) == 0, "tcsetattr failed");
      require(::fcntl(master_, F_SETFL, O_NONBLOCK) == 0, "fcntl failed");
      char config_name[] = "/tmp/aim-gimbal-test-XXXXXX";
      const int fd = ::mkstemp(config_name);
      require(fd >= 0, "mkstemp failed");
      config_path_ = config_name;
      const std::string yaml = std::string("com_port: '") + slave_name + "'\nbaudrate: 115200\n";
      const auto written = ::write(fd, yaml.data(), yaml.size());
      ::close(fd);
      require(written == static_cast<ssize_t>(yaml.size()), "temporary YAML write failed");
      thread_ = std::thread([this] { publish(); });
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~FakeGimbal()
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    cleanup();
  }

  const std::string & config() const { return config_path_; }
  std::atomic<Feed> feed;

  io::VisionToGimbal read_command(std::chrono::milliseconds timeout = 300ms)
  {
    static_assert(sizeof(io::VisionToGimbal) == 30, "Recheck the packed 30-byte TX protocol");
    std::array<uint8_t, sizeof(io::VisionToGimbal) + 1> bytes{};
    std::size_t received = 0;
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline && received < sizeof(io::VisionToGimbal)) {
      pollfd descriptor{master_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 10);
      if (ready < 0 && errno == EINTR) continue;
      require(ready >= 0, "PTY command poll failed");
      if (ready == 0) continue;
      const auto count = ::read(master_, bytes.data() + received, bytes.size() - received);
      if (count < 0 && (errno == EAGAIN || errno == EINTR)) continue;
      require(count > 0, "PTY command read failed");
      received += static_cast<std::size_t>(count);
    }
    require(received == sizeof(io::VisionToGimbal), "TX packet byte length was not exactly 30");
    // Only the test calls send, so extra bytes here indicate an unintended extra/oversized packet.
    pollfd descriptor{master_, POLLIN, 0};
    require(::poll(&descriptor, 1, 5) == 0, "Unexpected bytes after one TX command");
    io::VisionToGimbal packet{};
    std::memcpy(&packet, bytes.data(), sizeof(packet));
    require(packet.head[0] == 'S' && packet.head[1] == 'P', "TX packet header is incorrect");
    require(tools::check_crc16(bytes.data(), sizeof(packet)), "TX packet CRC16 is invalid");
    require(std::isfinite(packet.yaw) && std::isfinite(packet.pitch), "TX packet contains non-finite angles");
    require(packet.yaw_vel == 0.0f && packet.yaw_acc == 0.0f &&
            packet.pitch_vel == 0.0f && packet.pitch_acc == 0.0f,
            "TX reserved velocity/acceleration fields are not zero");
    return packet;
  }

private:
  int master_ = -1;
  int slave_ = -1;
  std::string config_path_;
  std::atomic<bool> stop_{false};
  std::thread thread_;

  void cleanup()
  {
    if (master_ >= 0) ::close(master_);
    if (slave_ >= 0) ::close(slave_);
    master_ = slave_ = -1;
    if (!config_path_.empty()) ::unlink(config_path_.c_str());
  }

  void publish()
  {
    while (!stop_) {
      const auto behavior = feed.load();
      if (behavior != Feed::PAUSED) {
        io::GimbalToVision packet{};
        packet.mode = 1;
        packet.q[0] = static_cast<float>(std::cos(0.1));
        packet.q[3] = static_cast<float>(std::sin(0.1));
        packet.yaw = 0.2f;
        packet.pitch = 0.05f;
        packet.bullet_speed = 22.0f;
        if (behavior == Feed::ZERO_QUATERNION) packet.q[0] = packet.q[3] = 0.0f;
        if (behavior == Feed::NAN_QUATERNION) {
          packet.q[1] = std::numeric_limits<float>::quiet_NaN();
        }
        packet.crc16 = tools::get_crc16(
          reinterpret_cast<const uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc16));
        if (behavior == Feed::BAD_CRC) packet.crc16 ^= 0x0101;
        // At most one small packet per iteration; a full PTY buffer just drops this sample.
        const auto ignored = ::write(master_, &packet, sizeof(packet));
        (void)ignored;
      }
      std::this_thread::sleep_for(3ms);
    }
  }
};

void test_freshness_and_interpolation()
{
  FakeGimbal simulator;
  auto gimbal = std::make_unique<io::Gimbal>(simulator.config());
  require(gimbal->fresh(), "fresh valid telemetry was not accepted");
  require(gimbal->mode() == io::GimbalMode::AUTO_AIM, "valid mode did not propagate");
  require(std::abs(gimbal->state().bullet_speed - 22.0f) < 1e-6f, "state parse failed");
  require(gimbal->state().game_progress == 0, "unused telemetry state is not initialized");

  std::this_thread::sleep_for(60ms);
  const auto attitude = gimbal->try_q(Clock::now() - 15ms, 100ms);
  require(attitude.has_value(), "valid image timestamp could not be bracketed");
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  require(attitude->angularDistance(expected) < 1e-5, "interpolated attitude is incorrect");
  require(std::abs(attitude->norm() - 1.0) < 1e-10, "attitude was not normalized");
  require(!gimbal->try_q(Clock::now() - 1s, 10ms), "stale image was extrapolated");
  require(!gimbal->try_q(Clock::now() + 1s, 10ms), "far future image was extrapolated");

  const FakeGimbal::Feed invalid_feeds[] = {
    FakeGimbal::Feed::ZERO_QUATERNION, FakeGimbal::Feed::NAN_QUATERNION,
    FakeGimbal::Feed::BAD_CRC, FakeGimbal::Feed::PAUSED};
  for (const auto invalid_feed : invalid_feeds) {
    simulator.feed = invalid_feed;
    require(wait_until([&] { return !gimbal->fresh(); }, 700ms),
            "invalid or missing telemetry refreshed the watchdog");
    require(gimbal->mode() == io::GimbalMode::IDLE, "stale mode remained active");
    require(gimbal->state().bullet_speed == 0.0f, "stale state was exposed as valid");
    const auto start = Clock::now();
    require(!gimbal->try_q(Clock::now(), 30ms), "stale attitude was returned");
    require(Clock::now() - start < 150ms, "try_q exceeded its bounded wait");
    simulator.feed = FakeGimbal::Feed::VALID;
    require(wait_until([&] { return gimbal->fresh(); }, 700ms), "valid telemetry did not recover");
    std::this_thread::sleep_for(40ms);
    const auto recovered = gimbal->try_q(Clock::now() - 10ms, 100ms);
    require(recovered.has_value() && recovered->angularDistance(expected) < 1e-5,
            "pose history did not recover after invalidation");
  }

  simulator.feed = FakeGimbal::Feed::PAUSED;
  const auto shutdown_start = Clock::now();
  gimbal.reset();
  require(Clock::now() - shutdown_start < 500ms, "reader shutdown blocked after telemetry loss");
  std::cout << "PASS valid telemetry, interpolation, stale/future rejection, zero/NaN/CRC rejection, recovery, shutdown\n";
}

void test_startup_timeout()
{
  FakeGimbal simulator(FakeGimbal::Feed::PAUSED);
  const auto start = Clock::now();
  bool expected_failure = false;
  try {
    io::Gimbal gimbal(simulator.config());
  } catch (const std::runtime_error & error) {
    expected_failure = std::string(error.what()).find("within 3 seconds") != std::string::npos;
  }
  require(expected_failure, "no-packet constructor did not fail with the expected timeout");
  const auto duration = Clock::now() - start;
  require(duration >= 2800ms && duration < 4200ms, "constructor timeout was not bounded near 3s");
  std::cout << "PASS startup timeout and constructor thread cleanup\n";
}

void test_pseudo_terminal_commands()
{
  FakeGimbal simulator;
  io::Gimbal gimbal(simulator.config());
  auto exchange = [&](bool control, bool fire, float yaw, float pitch, int expected_mode) {
    const auto start = Clock::now();
    gimbal.send(control, fire, yaw, pitch);
    const double send_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    std::cout << "PTY TX expected_mode=" << expected_mode << " send_ms=" << send_ms << std::endl;
    require(send_ms < 200.0, "TX was starved by RX instead of completing within the test latency budget");
    const auto packet = simulator.read_command();
    require(packet.mode == expected_mode, "TX control/fire mode encoding is incorrect");
    if (std::isfinite(yaw) && std::isfinite(pitch)) {
      require(std::abs(packet.yaw - yaw) < 1e-6f && std::abs(packet.pitch - pitch) < 1e-6f,
              "TX finite angle payload changed unexpectedly");
    } else {
      require(packet.yaw == 0.0f && packet.pitch == 0.0f,
              "Non-finite command did not become finite disabled angles");
    }
  };
  exchange(false, false, 0.12f, -0.05f, 0);
  exchange(false, true, 0.12f, -0.05f, 0);  // fire cannot override disabled control
  exchange(true, false, 1.23f, -0.2f, 1);
  exchange(true, true, 1.0f, 0.1f, 2);     // virtual packet only: PTY has no actuator
  exchange(true, false, 1.0f, 0.1f, 1);    // next no-fire packet must not latch mode 2
  exchange(false, false, 0.0f, 0.0f, 0);
  exchange(true, true, std::numeric_limits<float>::quiet_NaN(), 0.1f, 0);
  exchange(true, true, 0.1f, std::numeric_limits<float>::infinity(), 0);
  simulator.feed = FakeGimbal::Feed::PAUSED;
  require(wait_until([&] { return !gimbal.fresh(); }, 700ms), "PTY telemetry did not become stale");
  exchange(true, true, 0.15f, -0.1f, 0);   // stale link must always encode disabled control
  std::cout << "PASS PTY-only 30-byte TX header/CRC, mode 0/1/2, no fire latch, finite and stale gates\n";
}
}  // namespace

int main()
{
  try {
    // Initialize logging before the simulator/reader threads are created.
    tools::logger();
    test_freshness_and_interpolation();
    test_pseudo_terminal_commands();
    test_startup_timeout();
    std::cout << "All PTY transport tests passed; all command bytes stayed inside test pseudo-terminals.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL gimbal transport: " << error.what() << '\n';
    return 1;
  }
}
