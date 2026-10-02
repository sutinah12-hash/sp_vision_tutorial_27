#pragma once
#include "tracking_math.hpp"
#include "sp_controller_server/controller_plugin.hpp"

namespace nav_tracking {
class TrackingController : public sp_controller_server::ControllerPlugin {
 public:
  explicit TrackingController(bool use_mpc) : use_mpc_(use_mpc) {}
  void configure(const rclcpp::Node::SharedPtr &, const std::string &,
                 const std::shared_ptr<tf2_ros::Buffer> &) override;
  void setPlan(const nav_msgs::msg::Path &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &) override;
  void setSpeedLimit(double limit) override;
 private:
  bool use_mpc_, new_goal_=true;
  rclcpp::Node::SharedPtr node_;
  std::string name_, base_frame_, map_frame_="map";
  ReferencePath path_;
  VelocityMpc mpc_;
  Vec2 previous_=Vec2::Zero(), integral_=Vec2::Zero();
  double cruise_=1.25, acceleration_=1.7, braking_=0.8, lateral_=0.8;
  double kp_=1.8, ki_=0.05, kd_=0.2, last_time_=0.0, plan_time_=0.0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  std::size_t calls_=0;
};
class MpcController : public TrackingController {
 public:
  MpcController() : TrackingController(true) {}
};
}  // namespace nav_tracking
