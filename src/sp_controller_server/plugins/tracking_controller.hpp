#pragma once
#include "tracking_math.hpp"
#include "sampling_predictive.hpp"
#include "sp_controller_server/controller_plugin.hpp"
#include <std_msgs/msg/float64.hpp>
#include <deque>

namespace nav_tracking {
class TrackingController : public sp_controller_server::ControllerPlugin {
 public:
  explicit TrackingController(bool use_mpc,bool use_sampling=false)
    : use_mpc_(use_mpc),use_sampling_(use_sampling) {}
  void configure(const rclcpp::Node::SharedPtr &, const std::string &,
                 const std::shared_ptr<tf2_ros::Buffer> &) override;
  void setPlan(const nav_msgs::msg::Path &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &) override;
  void setSpeedLimit(double limit) override;
 private:
  bool use_mpc_, use_sampling_, new_goal_=true;
  rclcpp::Node::SharedPtr node_;
  std::string name_, base_frame_, map_frame_="map";
  ReferencePath path_;
  VelocityMpc mpc_;
  SamplingPredictive sampling_;
  Vec2 previous_=Vec2::Zero(), integral_=Vec2::Zero();
  double cruise_=1.25, acceleration_=1.7, braking_=0.8, lateral_=0.8;
  double kp_=1.8, ki_=0.05, kd_=0.2, last_time_=0.0, plan_time_=0.0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
  std::size_t calls_=0;
  int delay_steps_=3;
  std::deque<std::pair<double,Vec2>> command_history_;
};
class MpcController : public TrackingController {
 public:
  MpcController() : TrackingController(true) {}
};
class SamplingController : public TrackingController {
 public:
  SamplingController() : TrackingController(true,true) {}
};
}  // namespace nav_tracking
