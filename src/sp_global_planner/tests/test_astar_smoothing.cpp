#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "astar_planner.hpp"

class Planning : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::unique_ptr<sp_global_planner::AStarPlanner> planner(bool smooth=true) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("AStar.lethal_cost",95),
      rclcpp::Parameter("AStar.cost_weight",6.0),
      rclcpp::Parameter("AStar.smoothing_enabled",smooth)});
    nodes.push_back(std::make_shared<rclcpp::Node>("planner_test_"+std::to_string(nodes.size()),options));
    auto result=std::make_unique<sp_global_planner::AStarPlanner>();
    result->configure(nodes.back(),"AStar"); return result;
  }
  nav_msgs::msg::OccupancyGrid map(int width=40,int height=40) {
    nav_msgs::msg::OccupancyGrid m; m.header.frame_id="map";
    m.info.resolution=.05; m.info.width=width; m.info.height=height;
    m.info.origin.orientation.w=1; m.data.assign(width*height,0); return m;
  }
  geometry_msgs::msg::PoseStamped pose(double x,double y) {
    geometry_msgs::msg::PoseStamped p; p.header.frame_id="map";
    p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1; return p;
  }
  double bending(const nav_msgs::msg::Path & path) {
    double sum=0;
    for (std::size_t i=1;i+1<path.poses.size();++i) {
      const auto & a=path.poses[i-1].pose.position;
      const auto & b=path.poses[i].pose.position;
      const auto & c=path.poses[i+1].pose.position;
      sum+=std::pow(a.x+c.x-2*b.x,2)+std::pow(a.y+c.y-2*b.y,2);
    }
    return sum;
  }
};

TEST_F(Planning, MissingMapAndBlockedEndpointsReturnNoPlan) {
  auto p=planner(); EXPECT_TRUE(p->createPlan(pose(.2,.2),pose(1.7,1.7)).poses.empty());
  auto m=map(); m.data[4*40+4]=100; p->setMap(m);
  EXPECT_TRUE(p->createPlan(pose(.225,.225),pose(1.7,1.7)).poses.empty());
  EXPECT_TRUE(p->createPlan(pose(-1,-1),pose(1.7,1.7)).poses.empty());
}
TEST_F(Planning, NoDiagonalCornerCutting) {
  auto p=planner(); auto m=map(2,2); m.data={0,100,100,0}; p->setMap(m);
  EXPECT_TRUE(p->createPlan(pose(.025,.025),pose(.075,.075)).poses.empty());
}
TEST_F(Planning, RequestedCoordinatesArePreserved) {
  auto p=planner(); p->setMap(map());
  const auto path=p->createPlan(pose(.213,.217),pose(1.773,1.741));
  ASSERT_GT(path.poses.size(),1u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x,.213);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y,.217);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x,1.773);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y,1.741);
}
TEST_F(Planning, SmoothingReducesBendingWithoutCrossingWall) {
  auto m=map(); for (int y=0;y<29;++y) m.data[y*40+20]=100;
  auto raw_planner=planner(false), smooth_planner=planner(true);
  raw_planner->setMap(m); smooth_planner->setMap(m);
  const auto raw=raw_planner->createPlan(pose(.3,.3),pose(1.7,.3));
  const auto smooth=smooth_planner->createPlan(pose(.3,.3),pose(1.7,.3));
  ASSERT_GT(raw.poses.size(),2u); ASSERT_EQ(raw.poses.size(),smooth.poses.size());
  EXPECT_LT(bending(smooth),bending(raw));
  for (std::size_t i=1;i<smooth.poses.size();++i) {
    const auto & a=smooth.poses[i-1].pose.position;
    const auto & b=smooth.poses[i].pose.position;
    for (int k=0;k<=20;++k) {
      const double t=k/20.0;
      const int x=static_cast<int>(std::floor((a.x+t*(b.x-a.x))/m.info.resolution));
      const int y=static_cast<int>(std::floor((a.y+t*(b.y-a.y))/m.info.resolution));
      ASSERT_GE(x,0); ASSERT_LT(x,40); ASSERT_GE(y,0); ASSERT_LT(y,40);
      EXPECT_LT(m.data[y*40+x],95);
    }
  }
}
TEST_F(Planning, SameCellStillUsesExactGoal) {
  auto p=planner(); p->setMap(map());
  const auto path=p->createPlan(pose(.21,.21),pose(.23,.23));
  ASSERT_EQ(path.poses.size(),1u); EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x,.23);
}
