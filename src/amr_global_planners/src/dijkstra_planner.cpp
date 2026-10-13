#include "amr_global_planners/dijkstra_planner.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "amr_global_planners/grid_search.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_global_planners
{
namespace
{

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    length += std::hypot(dx, dy);
  }
  return length;
}

}  // namespace

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_penalty", rclcpp::ParameterValue(2.0));

  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".cost_penalty", cost_penalty_);

  metrics_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/planner_metrics/dijkstra", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured custom Dijkstra planner | allow_unknown=%s cost_penalty=%.3f",
    allow_unknown_ ? "true" : "false",
    cost_penalty_);
}

void DijkstraPlanner::cleanup()
{
  metrics_pub_.reset();
  costmap_ = nullptr;
  tf_.reset();
  node_.reset();
}

void DijkstraPlanner::activate()
{
  if (metrics_pub_) {
    metrics_pub_->on_activate();
  }
}

void DijkstraPlanner::deactivate()
{
  if (metrics_pub_) {
    metrics_pub_->on_deactivate();
  }
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (costmap_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Dijkstra: costmap is null");
    return path;
  }

  // The Nav2 costmap can be updated by another thread. Lock it for the
  // complete search / path conversion so the planner sees one consistent map.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(costmap_->getMutex()));

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    RCLCPP_WARN(node_->get_logger(), "Dijkstra: start or goal lies outside the global costmap");
    return path;
  }

  const auto begin = std::chrono::steady_clock::now();

  const auto result = runGridSearch(
    *costmap_,
    start_x,
    start_y,
    goal_x,
    goal_y,
    false,  // Dijkstra = heuristic disabled
    allow_unknown_,
    cost_penalty_);

  const auto end = std::chrono::steady_clock::now();
  const double planning_ms =
    std::chrono::duration<double, std::milli>(end - begin).count();

  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "Dijkstra: no path found");
    return path;
  }

  path.poses.reserve(result.cells.size());

  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(result.cells[i].first, result.cells[i].second, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    if (i + 1 < result.cells.size()) {
      double next_x = 0.0;
      double next_y = 0.0;
      costmap_->mapToWorld(
        result.cells[i + 1].first,
        result.cells[i + 1].second,
        next_x,
        next_y);
      const double yaw = std::atan2(next_y - wy, next_x - wx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }

    path.poses.push_back(pose);
  }

  const double length_m = pathLength(path);

  if (metrics_pub_ && metrics_pub_->is_activated()) {
    std_msgs::msg::Float64MultiArray metrics;
    metrics.data = {
      planning_ms,
      static_cast<double>(result.expanded_nodes),
      length_m,
      static_cast<double>(path.poses.size())
    };
    metrics_pub_->publish(metrics);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Dijkstra | %.3f ms | expanded=%zu | length=%.3f m | poses=%zu",
    planning_ms,
    result.expanded_nodes,
    length_m,
    path.poses.size());

  return path;
}

}  // namespace amr_global_planners

PLUGINLIB_EXPORT_CLASS(amr_global_planners::DijkstraPlanner, nav2_core::GlobalPlanner)
