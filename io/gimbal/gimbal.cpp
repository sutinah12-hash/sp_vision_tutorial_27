#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr auto kFreshness = std::chrono::milliseconds{200};
constexpr auto kReadWait = std::chrono::milliseconds{10};

bool valid_packet(const GimbalToVision & packet)
{
  const float values[] = {
    packet.q[0], packet.q[1], packet.q[2], packet.q[3], packet.yaw, packet.yaw_vel,
    packet.pitch, packet.pitch_vel, packet.bullet_speed, packet.supercap_power_in,
    packet.supercap_power_out, packet.supercap_voltage};
  for (float value : values) {
    if (!std::isfinite(value)) return false;
  }
  Eigen::Quaterniond attitude(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
  const double norm = attitude.norm();
  return packet.mode <= 4 && norm >= 0.5 && norm <= 1.5 &&
         packet.bullet_speed >= 0.0f && packet.bullet_speed <= 100.0f &&
         packet.supercap_voltage >= 0.0f && packet.supercap_voltage <= 100.0f;
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto com_port = tools::read<std::string>(yaml, "com_port");
  const int baudrate = tools::read_or<int>(yaml, "baudrate", 921600);
  if (baudrate <= 0) throw std::invalid_argument("Gimbal baudrate must be positive");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    // Both reads and writes have finite timeouts. No sleeping while holding the port lock.
    auto timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(kReadWait.count()));
    serial_.setTimeout(timeout);
    serial_.open();
    is_connected_ = true;
    thread_ = std::thread(&Gimbal::read_thread, this);
    TimedQuaternion first;
    if (!queue_.try_front_for(first, std::chrono::seconds{3})) {
      throw std::runtime_error("Gimbal: no valid telemetry packet within 3 seconds");
    }
    tools::logger()->info("[Gimbal] First validated telemetry packet received.");
  } catch (...) {
    // A throwing constructor must stop its thread itself: the destructor will not run.
    quit_ = true;
    if (thread_.joinable()) thread_.join();
    is_connected_ = false;
    try {
      std::unique_lock<std::shared_mutex> lock(serial_mutex_);
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
    }
    throw;
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  invalidate();
  is_connected_ = false;
  try {
    std::unique_lock<std::shared_mutex> lock(serial_mutex_);
    if (serial_.isOpen()) serial_.close();
  } catch (...) {
    // Destruction must not throw on an unplugged port.
  }
}

bool Gimbal::fresh(std::chrono::milliseconds max_age) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !quit_ && is_connected_ && has_valid_packet_ && max_age.count() >= 0 &&
         Clock::now() - last_valid_packet_ <= max_age;
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_valid_packet_ || Clock::now() - last_valid_packet_ > kFreshness) {
    return GimbalMode::IDLE;
  }
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_valid_packet_ || Clock::now() - last_valid_packet_ > kFreshness) return {};
  return state_;
}

void Gimbal::send(bool control, bool fire, float yaw, float pitch)
{
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    tools::logger()->warn("[Gimbal] Non-finite command downgraded to disabled control.");
    control = false;
    fire = false;
    yaw = pitch = 0.0f;
  }
  std::lock_guard<std::mutex> tx_lock(tx_mutex_);
  std::shared_lock<std::shared_mutex> port_lock(serial_mutex_);
  if (quit_ || !is_connected_ || !serial_.isOpen()) return;
  // Never allow a stale link to turn a previous tracking decision into an active command.
  if (!fresh()) {
    control = false;
    fire = false;
  }
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.pitch = pitch;
  tx_data_.yaw_vel = tx_data_.yaw_acc = tx_data_.pitch_vel = tx_data_.pitch_acc = 0.0f;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  try {
    const auto sent = serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    if (sent != sizeof(tx_data_)) {
      is_connected_ = false;
      invalidate();
      tools::logger()->warn("[Gimbal] Partial serial write; telemetry invalidated.");
    }
  } catch (const std::exception & e) {
    is_connected_ = false;
    invalidate();
    tools::logger()->warn("[Gimbal] Serial write failed: {}", e.what());
  }
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    case GimbalMode::HANGING: return "HANGING";
    default: return "INVALID";
  }
}

std::optional<Eigen::Quaterniond> Gimbal::try_q(
  Clock::time_point t, std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + std::max(timeout, std::chrono::milliseconds{0});
  std::unique_lock<std::timed_mutex> lock(q_mutex_, std::defer_lock);
  if (!lock.try_lock_until(deadline)) return std::nullopt;
  if (!fresh() || Clock::now() - t > kFreshness || t > deadline) return std::nullopt;
  const auto generation = pose_generation_.load();
  if (q_generation_ != generation) {
    q_left_.reset();
    q_generation_ = generation;
  }
  while (!quit_ && fresh() && pose_generation_.load() == generation) {
    const auto now = Clock::now();
    if (now >= deadline) return std::nullopt;
    const auto wait = std::min(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now), kReadWait);
    if (!q_left_) {
      TimedQuaternion first;
      if (!queue_.try_pop_for(first, wait)) continue;
      q_left_ = first;
    }
    const auto & [q_a, t_a] = *q_left_;
    if (t < t_a) return std::nullopt;
    if (t == t_a) {
      return fresh() && pose_generation_.load() == generation
               ? std::optional<Eigen::Quaterniond>{q_a} : std::nullopt;
    }
    TimedQuaternion next;
    if (!queue_.try_front_for(next, wait)) continue;
    const auto & [q_b, t_b] = next;
    if (t_b <= t_a || t > t_b) {
      if (queue_.try_pop_for(next, std::chrono::milliseconds{0})) q_left_ = next;
      continue;
    }
    const double interval = std::chrono::duration<double>(t_b - t_a).count();
    const double fraction = std::chrono::duration<double>(t - t_a).count() / interval;
    auto interpolated = q_a.slerp(fraction, q_b).normalized();
    if (!fresh() || pose_generation_.load() != generation ||
        !interpolated.coeffs().allFinite()) {
      return std::nullopt;
    }
    return interpolated;
  }
  return std::nullopt;
}

Eigen::Quaterniond Gimbal::q(Clock::time_point t)
{
  auto attitude = try_q(t);
  if (!attitude) throw std::runtime_error("Gimbal: no fresh attitude bracketing image time");
  return *attitude;
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  // A timeout while waiting for input must not prevent a concurrent stop packet.
  std::shared_lock<std::shared_mutex> port_lock(serial_mutex_);
  if (quit_ || !is_connected_ || !serial_.isOpen()) return false;
  try {
    return serial_.read(buffer, size) == size;
  } catch (...) {
    is_connected_ = false;
    invalidate();
    return false;
  }
}

void Gimbal::invalidate()
{
  // State and history invalidation share a lock with publishing validated packets.
  std::lock_guard<std::mutex> lock(mutex_);
  has_valid_packet_ = false;
  last_valid_packet_ = {};
  state_ = {};
  mode_ = GimbalMode::IDLE;
  queue_.clear();
  ++pose_generation_;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] Telemetry reader started.");
  auto last_packet = Clock::now();
  bool stale_cleared = false;
  while (!quit_) {
    const auto now = Clock::now();
    if (now - last_packet > kFreshness && !stale_cleared) {
      invalidate();
      stale_cleared = true;
    }
    if (!is_connected_ || now - last_packet > std::chrono::seconds{1}) {
      reconnect();
      last_packet = Clock::now();
      stale_cleared = true;
      continue;
    }
    uint8_t head_byte = 0;
    if (!read(&head_byte, 1) || head_byte != 'S') continue;
    if (!read(&head_byte, 1) || head_byte != 'P') continue;
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + 2, sizeof(rx_data_) - 2)) continue;
    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_)) ||
        !valid_packet(rx_data_)) {
      continue;
    }
    const auto t = Clock::now();
    Eigen::Quaterniond attitude(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    attitude.normalize();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!is_connected_) continue;
      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;
      state_.supercap_power_in = rx_data_.supercap_power_in;
      state_.supercap_power_out = rx_data_.supercap_power_out;
      state_.supercap_voltage = rx_data_.supercap_voltage;
      state_.supercap_temputer = rx_data_.supercap_temputer;
      state_.supercap_status = rx_data_.supercap_status;
      state_.supercap_cap_energy = 2.5f * state_.supercap_voltage * state_.supercap_voltage;
      mode_ = static_cast<GimbalMode>(rx_data_.mode);
      last_valid_packet_ = t;
      has_valid_packet_ = true;
      queue_.push({attitude, t});
    }
    last_packet = t;
    stale_cleared = false;
  }
  tools::logger()->info("[Gimbal] Telemetry reader stopped.");
}

void Gimbal::reconnect()
{
  is_connected_ = false;
  invalidate();
  try {
    {
      std::unique_lock<std::shared_mutex> lock(serial_mutex_);
      if (serial_.isOpen()) serial_.close();
    }
    // Short, interruptible backoff outside the port lock; destruction does not wait one second.
    for (int i = 0; i < 5 && !quit_; ++i) std::this_thread::sleep_for(kReadWait);
    if (quit_) return;
    std::unique_lock<std::shared_mutex> lock(serial_mutex_);
    serial_.open();
    is_connected_ = true;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
  }
}
}  // namespace io
