#!/usr/bin/env python3
"""Send and monitor one Nav2 NavigateToPose goal from the command line."""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("snowplow_navigation_goal")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def run(self, x: float, y: float, yaw: float, timeout: float) -> bool:
        if not self.client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("/navigate_to_pose action server is unavailable")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f"Goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Goal was rejected")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        succeeded = result is not None and result.status == 4
        self.get_logger().info("Goal succeeded" if succeeded else f"Goal ended with status {result.status if result else 'unknown'}")
        return succeeded

    def _feedback(self, message) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f"remaining={feedback.distance_remaining:.2f} m, "
            f"recoveries={feedback.number_of_recoveries}",
            throttle_duration_sec=1.0,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a snowplow Nav2 map-frame goal.")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float, nargs="?", default=0.0)
    parser.add_argument("--server-timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = NavigationGoalClient()
    try:
        success = node.run(args.x, args.y, args.yaw, args.server_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
