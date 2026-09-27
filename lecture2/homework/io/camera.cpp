#include "camera.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "hikrobot/hikrobot.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  const auto config = YAML::LoadFile(config_path);
  const auto camera_name = config["camera_name"].as<std::string>();

  if (camera_name != "hikrobot") {
    throw std::runtime_error("Unsupported camera: " + camera_name);
  }

  const auto exposure_ms = config["exposure_ms"].as<double>();
  const auto gain = config["gain"].as<double>();
  const auto vid_pid = config["vid_pid"].as<std::string>();
  const auto rotation_angle = config["rotation_angle"] ? config["rotation_angle"].as<int>() : 0;

  if (config["serial_number"]) {
    camera_ = std::make_unique<HikRobot>(
      exposure_ms, gain, vid_pid, config["serial_number"].as<std::string>(), rotation_angle);
  } else {
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, rotation_angle);
  }
}

Camera::~Camera() = default;

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
