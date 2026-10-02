#pragma once
#include "sp_global_planner/global_planner_plugin.hpp"
#include "sp_global_planner/grid_utils.hpp"
#include <optional>
#include <vector>

namespace sp_global_planner {

class AStarPlanner : public GlobalPlannerPlugin
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name) override;
  void setMap(const nav_msgs::msg::OccupancyGrid& costmap) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) override;

private:
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};

  int lethal_cost_{100};
  double cost_weight_{2.0};
  bool smoothing_enabled_{true};
  int smoothing_iterations_{120};
  int smoothing_max_cost_{85};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;

  bool safeSegment(const geometry_msgs::msg::Point & a,
                   const geometry_msgs::msg::Point & b) const;
  nav_msgs::msg::Path smoothPlan(const nav_msgs::msg::Path & raw) const;

  bool isBlocked(int8_t c) const;
  double cellCostFactor(int8_t c) const;
};

}
