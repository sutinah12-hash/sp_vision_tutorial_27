#include "tracking_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav_tracking {
namespace {
template <class T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & value) {
  return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() :
         node->declare_parameter<T>(name,value);
}
}
void TrackingController::configure(const rclcpp::Node::SharedPtr & node,
    const std::string & name, const std::shared_ptr<tf2_ros::Buffer> & tf) {
  (void)tf;
  if (!node) throw std::invalid_argument("Controller requires a node");
  node_=node; name_=name;
  const auto key=[&](const std::string & suffix){return name+"."+suffix;};
  base_frame_=parameter<std::string>(node,key("base_frame_id"),"base_link");
  map_frame_=parameter<std::string>(node,key("map_frame_id"),"map");
  cruise_=parameter(node,key("max_speed"),1.25);
  acceleration_=parameter(node,key("max_acceleration"),1.7);
  braking_=parameter(node,key("braking_acceleration"),0.8);
  lateral_=parameter(node,key("lateral_acceleration"),0.8);
  kp_=parameter(node,key("kp"),1.8);
  ki_=parameter(node,key("ki"),0.05);
  kd_=parameter(node,key("kd"),0.2);
  mpc_.horizon=parameter(node,key("horizon"),28);
  mpc_.dt=parameter(node,key("prediction_dt"),0.07);
  mpc_.tau=parameter(node,key("velocity_time_constant"),0.315);
  delay_steps_=parameter(node,key("delay_steps"),3);
  mpc_.iterations=parameter(node,key("solver_iterations"),100);
  mpc_.position_weight=parameter(node,key("position_weight"),24.0);
  mpc_.velocity_weight=parameter(node,key("velocity_weight"),1.5);
  mpc_.change_weight=parameter(node,key("command_change_weight"),0.7);
  if (!std::isfinite(cruise_) || cruise_<=0 || cruise_>2.0 ||
      acceleration_<=0 || braking_<=0 || lateral_<=0 || delay_steps_<0 || delay_steps_>=mpc_.horizon)
    throw std::invalid_argument("Invalid controller speed or acceleration limits");
  mpc_.speed_limit=cruise_;
  mpc_.initialize();
  prediction_pub_=node_->create_publisher<nav_msgs::msg::Path>("/mpc_prediction",10);
  RCLCPP_INFO(node_->get_logger(),"%s configured: max_speed=%.2f m/s horizon=%d dt=%.3f s",
              use_mpc_ ? "MPC" : "PID",cruise_,mpc_.horizon,mpc_.dt);
}

void TrackingController::setPlan(const nav_msgs::msg::Path & plan) {
  if (!node_) throw std::runtime_error("configure must precede setPlan");
  std::vector<Vec2> points;
  if (plan.header.frame_id==map_frame_) {
    for (const auto & p:plan.poses) points.emplace_back(p.pose.position.x,p.pose.position.y);
  }
  if (!points.empty() && (path_.points.empty() || (points.back()-path_.points.back()).norm()>0.15))
    new_goal_=true;
  path_.set(points,cruise_,braking_,lateral_);
  plan_time_=node_->now().seconds();
}

void TrackingController::setSpeedLimit(double limit) {
  if (!std::isfinite(limit) || limit<0) throw std::invalid_argument("Invalid speed limit");
  cruise_=std::min(limit,2.0); mpc_.speed_limit=cruise_;
  const auto copy=path_.points;
  path_.set(copy,cruise_,braking_,lateral_);
}

geometry_msgs::msg::TwistStamped TrackingController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity) {
  if (!node_) throw std::runtime_error("Controller is not configured");
  geometry_msgs::msg::TwistStamped result;
  result.header.stamp=node_->now(); result.header.frame_id=base_frame_;
  const double now=node_->now().seconds();
  const double dt=last_time_>0 ? std::clamp(now-last_time_,0.005,0.1) : 0.02;
  last_time_=now;
  const Vec2 p(pose.pose.position.x,pose.pose.position.y);
  const Vec2 v(velocity.linear.x,velocity.linear.y); // Server explicitly supplies map-frame velocity.
  if (path_.points.empty() || pose.header.frame_id!=map_frame_ ||
      !p.allFinite() || !v.allFinite() || now-plan_time_>2.0 || cruise_<=0) {
    previous_.setZero(); integral_.setZero(); command_history_.clear(); mpc_.reset(); return result;
  }
  if (new_goal_) {
    integral_.setZero(); previous_=bounded(v,cruise_); command_history_.clear(); mpc_.reset(); new_goal_=false;
  }
  Vec2 command;
  const double distance=(path_.points.back()-p).norm();
  if (distance<0.018 && v.norm()<0.04) {
    command.setZero(); integral_.setZero(); mpc_.reset();
  } else if (use_mpc_) {
    Eigen::MatrixXd ref_p,ref_v;
    path_.preview(p,v,mpc_.horizon,mpc_.dt,ref_p,ref_v);
    Eigen::MatrixXd pending=Eigen::MatrixXd::Zero(delay_steps_,2);
    for (int i=0;i<delay_steps_;++i) {
      const double time=now-(delay_steps_-i)*mpc_.dt;
      for (const auto & sample:command_history_) {
        if (sample.first>time) break;
        pending.row(i)=sample.second.transpose();
      }
    }
    command=mpc_.solve(p,v,previous_,ref_p,ref_v,pending);
    if (++calls_%5==0) {
      nav_msgs::msg::Path predicted;
      predicted.header.stamp=result.header.stamp; predicted.header.frame_id=map_frame_;
      for (int i=0;i<mpc_.prediction.rows();++i) {
        geometry_msgs::msg::PoseStamped point;
        point.header=predicted.header; point.pose.orientation.w=1.0;
        point.pose.position.x=mpc_.prediction(i,0);
        point.pose.position.y=mpc_.prediction(i,1);
        predicted.poses.push_back(point);
      }
      prediction_pub_->publish(predicted);
    }
  } else if (distance<0.70) {
    // The path's square-root braking profile is too aggressive near the endpoint
    // for a delayed PID plant. Use a damped, zero-feedforward terminal regulator.
    integral_.setZero();
    command=1.0*(path_.points.back()-p)-0.60*v;
  } else {
    const double s=path_.nearest(p);
    // Feedback corrects cross-track error without an artificial forward position
    // bias. A short tangent preview anticipates bends in the delayed plant.
    const Vec2 error=path_.at(s)-p;
    const double preview=std::min(0.20,path_.length()-s);
    const Vec2 feedforward=path_.tangent(s+preview)*path_.speed(s);
    integral_=bounded(integral_+error*dt,0.25);
    command=feedforward+kp_*error+ki_*integral_+kd_*(feedforward-v);
  }
  // Limit the command in world coordinates; base_link rotates during the simulation.
  command=bounded(command,cruise_);
  command=previous_+bounded(command-previous_,acceleration_*dt);
  previous_=command;
  command_history_.emplace_back(now,command);
  while (!command_history_.empty() && command_history_.front().first<now-2.0)
    command_history_.pop_front();
  const auto & q=pose.pose.orientation;
  const double yaw=std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
  const Vec2 body=worldToBody(command,yaw);
  result.twist.linear.x=body.x(); result.twist.linear.y=body.y();
  return result;
}
}  // namespace nav_tracking
PLUGINLIB_EXPORT_CLASS(nav_tracking::MpcController,sp_controller_server::ControllerPlugin)
