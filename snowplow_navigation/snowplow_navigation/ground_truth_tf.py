#!/usr/bin/env python3
"""Publish Nav2 TF from Gazebo ground-truth odometry."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class GroundTruthTf(Node):
    def __init__(self) -> None:
        super().__init__("snowplow_ground_truth_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            30,
        )
        self.static_timer = self.create_timer(1.0, self._publish_map_to_odom)
        self._publish_map_to_odom()
        self.get_logger().info("Publishing map->odom and odom->base_link from /odom")

    def _publish_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def _on_odom(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
