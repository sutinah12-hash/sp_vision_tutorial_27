#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/robot_kinematics_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using visualization_msgs::msg::Marker;

std::array<float, 3> color_for_robot(const uint32_t id)
{
  switch (id) {
    case 1:
      return {0.10F, 0.80F, 1.00F};
    case 2:
      return {0.20F, 0.90F, 0.25F};
    case 3:
      return {1.00F, 0.25F, 0.15F};
    default:
      return {0.95F, 0.75F, 0.15F};
  }
}

void set_color(Marker & marker, const std::array<float, 3> & color, const float alpha)
{
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = alpha;
}
}  // namespace

class RobotMarkerPublisher : public rclcpp::Node
{
public:
  RobotMarkerPublisher()
  : Node("robot_marker_publisher")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/robots");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/dynamic_obstacles/markers");

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);
    robot_sub_ = create_subscription<robot_msg::msg::RobotKinematicsArray>(
      input_topic, rclcpp::QoS(10).reliable(),
      std::bind(&RobotMarkerPublisher::on_robots, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Visualizing robot states from %s on %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void on_robots(const robot_msg::msg::RobotKinematicsArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray output;
    output.markers.reserve(msg->robots.size() * 3U);

    for (const auto & robot : msg->robots) {
      const auto color = color_for_robot(robot.id);
      const auto base_id = static_cast<int32_t>(robot.id * 10U);
      auto header = msg->header;
      if (header.frame_id.empty()) {
        header.frame_id = "map";
      }

      Marker body;
      body.header = header;
      body.ns = "robot_bodies";
      body.id = base_id;
      body.type = Marker::CUBE;
      body.action = Marker::ADD;
      body.pose = robot.pose.pose;
      body.pose.position.z = 0.10;
      body.scale.x = 0.50;
      body.scale.y = 0.50;
      body.scale.z = 0.20;
      set_color(body, color, 0.92F);
      body.lifetime = rclcpp::Duration::from_seconds(0.30);
      output.markers.push_back(body);

      Marker heading;
      heading.header = header;
      heading.ns = "robot_headings";
      heading.id = base_id + 1;
      heading.type = Marker::ARROW;
      heading.action = Marker::ADD;
      heading.pose = robot.pose.pose;
      heading.pose.position.z = 0.24;
      heading.scale.x = 0.46;
      heading.scale.y = 0.09;
      heading.scale.z = 0.12;
      set_color(heading, color, 1.00F);
      heading.lifetime = rclcpp::Duration::from_seconds(0.30);
      output.markers.push_back(heading);

      Marker label;
      label.header = header;
      label.ns = "robot_labels";
      label.id = base_id + 2;
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose = robot.pose.pose;
      label.pose.position.z = 0.55;
      label.pose.orientation.x = 0.0;
      label.pose.orientation.y = 0.0;
      label.pose.orientation.z = 0.0;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.32;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = "R" + std::to_string(robot.id);
      label.lifetime = rclcpp::Duration::from_seconds(0.30);
      output.markers.push_back(label);
    }

    marker_pub_->publish(output);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<robot_msg::msg::RobotKinematicsArray>::SharedPtr robot_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
