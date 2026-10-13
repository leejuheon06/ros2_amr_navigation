#ifndef AMR_GLOBAL_PLANNERS__ASTAR_PLANNER_HPP_
#define AMR_GLOBAL_PLANNERS__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"

namespace amr_global_planners
{

/**
 * @brief Custom A* global planner plugin for ROS2 Humble Nav2.
 *
 * This class is only a Nav2 adapter around the shared grid-search engine.
 * The actual search logic is kept in grid_search.cpp so it can be compared
 * fairly with Dijkstra.
 */
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;

  bool allow_unknown_{false};
  double cost_penalty_{2.0};

  // data = [planning_time_ms, expanded_nodes, path_length_m, path_pose_count]
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
};

}  // namespace amr_global_planners

#endif  // AMR_GLOBAL_PLANNERS__ASTAR_PLANNER_HPP_
