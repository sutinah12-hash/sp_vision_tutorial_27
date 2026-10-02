#include <atomic>
#include <cmath>
#include <chrono>
#include <action_msgs/msg/goal_status_array.hpp>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace sp_nav_tools {
class FixedGoalPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit FixedGoalPanel(QWidget * parent=nullptr) : Panel(parent) {
    auto layout=new QVBoxLayout(this);
    label_=new QLabel("Waiting for robot at the fixed start...",this);
    label_->setWordWrap(true);
    button_=new QPushButton("Navigate once: (14.100, 14.100)",this);
    button_->setEnabled(false); button_->setMinimumHeight(44);
    layout->addWidget(label_); layout->addWidget(button_);
    connect(button_,&QPushButton::clicked,this,[this] {
      if (sent_ || !ready_.load() || !publisher_) return;
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id="map"; goal.header.stamp=node_->now();
      goal.pose.position.x=14.1; goal.pose.position.y=14.1; goal.pose.orientation.w=1.0;
      goal_stamp_.store(node_->now().nanoseconds());
      started_=std::chrono::steady_clock::now();
      publisher_->publish(goal); sent_=true; button_->setEnabled(false);
      label_->setText("One goal sent. Follow the green path. Restart the launch for a new trial.");
    });
    auto timer=new QTimer(this);
    connect(timer,&QTimer::timeout,this,[this] {
      if (!sent_ && publisher_) {
        const bool available=ready_.load() && publisher_->get_subscription_count()>=2;
        button_->setEnabled(available);
        if (available) label_->setText("Start: (0.900, 0.900). Click below once to run both behavior trees.");
      } else if (sent_) {
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started_).count();
        const int status=status_.load();
        if (status==4 && arrived_time_<0) arrived_time_=elapsed;
        const QString state=status==4 ? "Action SUCCEEDED" : (status==6 ? "Action ABORTED" : "Navigating");
        label_->setText(QString("%1 | ONE goal\nTime: %2 s\nGoal error: %3 m | Speed: %4 m/s")
          .arg(state).arg(arrived_time_>=0 ? arrived_time_ : elapsed,0,'f',2)
          .arg(goal_error_.load(),0,'f',3).arg(speed_.load(),0,'f',3));
      }
    });
    timer->start(250);
  }
  void onInitialize() override {
    node_=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_=node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);
    odom_=node_->create_subscription<nav_msgs::msg::Odometry>("/Odometry",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
        goal_error_.store(std::hypot(odom->pose.pose.position.x+0.9-14.1,
                                    odom->pose.pose.position.y+0.9-14.1));
        speed_.store(std::hypot(odom->twist.twist.linear.x,odom->twist.twist.linear.y));
        ready_.store(std::hypot(odom->pose.pose.position.x,odom->pose.pose.position.y)<0.03 &&
                     std::hypot(odom->twist.twist.linear.x,odom->twist.twist.linear.y)<0.02);
      });
    status_sub_=node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status",rclcpp::QoS(10).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        const auto start=goal_stamp_.load();
        if (start==0) return;
        for (const auto & item:message->status_list) {
          const auto stamp=rclcpp::Time(item.goal_info.stamp).nanoseconds();
          if (stamp+100000000>=start) status_.store(item.status);
        }
      });
  }
 private:
  QLabel * label_; QPushButton * button_; bool sent_=false;
  std::atomic<bool> ready_{false};
  std::atomic<int> status_{0};
  std::atomic<int64_t> goal_stamp_{0};
  std::atomic<double> goal_error_{0},speed_{0};
  std::chrono::steady_clock::time_point started_;
  double arrived_time_=-1;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
};
}
PLUGINLIB_EXPORT_CLASS(sp_nav_tools::FixedGoalPanel,rviz_common::Panel)
#include "fixed_goal_panel.moc"
