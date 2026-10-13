#!/usr/bin/env python3
"""Automated Dijkstra vs A* planner benchmark through Nav2.

The benchmark talks to Nav2's /compute_path_to_pose action rather than calling
our C++ classes directly. Therefore the measured path is the path returned by
Nav2 Planner Server using the actual global costmap and plugin configuration.

CSV columns include both:
- Nav2 action planning time
- plugin-internal planning time / expanded nodes / path length
"""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Float64MultiArray


class PlanningBenchmark(Node):
    def __init__(self) -> None:
        super().__init__('planning_benchmark')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('repetitions', 10)
        self.declare_parameter('output_csv', '/tmp/planner_benchmark.csv')

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.repetitions = int(self.get_parameter('repetitions').value)
        self.output_csv = Path(str(self.get_parameter('output_csv').value)).expanduser()

        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self.latest_metrics: Dict[str, Optional[List[float]]] = {
            'AStar': None,
            'Dijkstra': None,
        }

        self.create_subscription(
            Float64MultiArray,
            '/planner_metrics/astar',
            lambda msg: self._metrics_callback('AStar', msg),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/planner_metrics/dijkstra',
            lambda msg: self._metrics_callback('Dijkstra', msg),
            10,
        )

        # Publish the most recently returned path from each planner so RViz2 can
        # overlay both paths on the maze map.
        self.astar_path_pub = self.create_publisher(PathMsg, '/benchmark/astar_path', 10)
        self.dijkstra_path_pub = self.create_publisher(PathMsg, '/benchmark/dijkstra_path', 10)

    def _metrics_callback(self, planner: str, msg: Float64MultiArray) -> None:
        self.latest_metrics[planner] = list(msg.data)

    @staticmethod
    def _path_length(path: PathMsg) -> float:
        length = 0.0
        for i in range(1, len(path.poses)):
            dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x
            dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y
            length += math.hypot(dx, dy)
        return length

    def _build_goal(self, planner_id: str) -> ComputePathToPose.Goal:
        goal = ComputePathToPose.Goal()
        now = self.get_clock().now().to_msg()

        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = now
        goal.start.pose.position.x = self.start_x
        goal.start.pose.position.y = self.start_y
        goal.start.pose.orientation.w = 1.0

        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = now
        goal.goal.pose.position.x = self.goal_x
        goal.goal.pose.position.y = self.goal_y
        goal.goal.pose.orientation.w = 1.0

        goal.use_start = True
        goal.planner_id = planner_id
        return goal

    def run_once(self, planner_id: str, iteration: int) -> dict:
        self.latest_metrics[planner_id] = None

        send_future = self.client.send_goal_async(self._build_goal(planner_id))
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{planner_id}: goal rejected')
            return {
                'iteration': iteration,
                'planner': planner_id,
                'success': 0,
            }

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()

        if wrapped is None:
            self.get_logger().error(f'{planner_id}: no action result')
            return {
                'iteration': iteration,
                'planner': planner_id,
                'success': 0,
            }

        result = wrapped.result
        path = result.path

        # Give the metrics subscription one extra spin in case the plugin's
        # metrics message and action result arrive in the same cycle.
        rclpy.spin_once(self, timeout_sec=0.05)
        metrics = self.latest_metrics[planner_id]

        action_planning_ms = (
            float(result.planning_time.sec) * 1000.0
            + float(result.planning_time.nanosec) / 1.0e6
        )
        result_path_length = self._path_length(path)

        plugin_planning_ms = float('nan')
        expanded_nodes = float('nan')
        plugin_path_length = float('nan')
        plugin_pose_count = float('nan')
        if metrics is not None and len(metrics) >= 4:
            plugin_planning_ms = metrics[0]
            expanded_nodes = metrics[1]
            plugin_path_length = metrics[2]
            plugin_pose_count = metrics[3]

        if planner_id == 'AStar':
            self.astar_path_pub.publish(path)
        else:
            self.dijkstra_path_pub.publish(path)

        self.get_logger().info(
            f'{planner_id} #{iteration}: '
            f'action={action_planning_ms:.3f} ms, '
            f'plugin={plugin_planning_ms:.3f} ms, '
            f'expanded={expanded_nodes:.0f}, '
            f'length={result_path_length:.3f} m'
        )

        return {
            'iteration': iteration,
            'planner': planner_id,
            'success': int(len(path.poses) > 0),
            'action_planning_ms': action_planning_ms,
            'plugin_planning_ms': plugin_planning_ms,
            'expanded_nodes': expanded_nodes,
            'result_path_length_m': result_path_length,
            'plugin_path_length_m': plugin_path_length,
            'result_pose_count': len(path.poses),
            'plugin_pose_count': plugin_pose_count,
        }

    def run(self) -> None:
        self.get_logger().info('Waiting for /compute_path_to_pose ...')
        if not self.client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('Nav2 /compute_path_to_pose action server not available')

        rows: List[dict] = []

        # Interleave planners so both are tested under nearly identical runtime
        # conditions instead of completing all runs for one planner first.
        for iteration in range(1, self.repetitions + 1):
            rows.append(self.run_once('Dijkstra', iteration))
            rows.append(self.run_once('AStar', iteration))

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            'iteration',
            'planner',
            'success',
            'action_planning_ms',
            'plugin_planning_ms',
            'expanded_nodes',
            'result_path_length_m',
            'plugin_path_length_m',
            'result_pose_count',
            'plugin_pose_count',
        ]

        with self.output_csv.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

        self.get_logger().info(f'CSV saved: {self.output_csv}')

        for planner in ('Dijkstra', 'AStar'):
            valid = [r for r in rows if r.get('planner') == planner and r.get('success') == 1]
            if not valid:
                continue

            times = [float(r['plugin_planning_ms']) for r in valid]
            nodes = [float(r['expanded_nodes']) for r in valid]
            lengths = [float(r['result_path_length_m']) for r in valid]

            self.get_logger().info(
                f'{planner} summary | '
                f'planning={statistics.mean(times):.3f} ms | '
                f'expanded={statistics.mean(nodes):.1f} | '
                f'length={statistics.mean(lengths):.3f} m | '
                f'success={len(valid)}/{self.repetitions}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningBenchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
