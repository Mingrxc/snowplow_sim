#!/usr/bin/env python3
"""Republish velocity commands and stop the robot after an input timeout.

Use this compatibility node between teleop_twist_keyboard and Gazebo:

    python3 cmd_vel_watchdog.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
        -r cmd_vel:=/cmd_vel_raw

The Gazebo diff-drive plugin subscribes to /cmd_vel. The watchdog prevents a
stale non-zero command from remaining active after the keyboard stops sending
messages, and clamps the command to the diff-drive axes linear.x/angular.z.

For daily interactive driving, operator_console.py is the preferred interface.
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self, input_topic: str, output_topic: str,
                 timeout: float, publish_rate: float,
                 max_linear_speed: float, max_angular_speed: float):
        super().__init__("cmd_vel_watchdog")
        self.timeout = timeout
        self.max_linear_speed = abs(max_linear_speed)
        self.max_angular_speed = abs(max_angular_speed)
        self.last_received = None
        self.last_command = Twist()
        self.zero_command = Twist()

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self._on_command, 10
        )
        self.timer = self.create_timer(
            1.0 / publish_rate, self._publish_safe_command
        )

        self.get_logger().info(
            f"watchdog: {input_topic} -> {output_topic}, "
            f"timeout={timeout:.2f}s, rate={publish_rate:.1f}Hz, "
            f"linear_limit={self.max_linear_speed:.3f}m/s, "
            f"angular_limit={self.max_angular_speed:.3f}rad/s"
        )

    def _on_command(self, message: Twist) -> None:
        self.last_command = self._limit_command(message)
        self.last_received = time.monotonic()

    def _limit_command(self, message: Twist) -> Twist:
        command = Twist()
        command.linear.x = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, float(message.linear.x)),
        )
        command.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, float(message.angular.z)),
        )
        return command

    def _publish_safe_command(self) -> None:
        if self.last_received is None:
            self.publisher.publish(self.zero_command)
            return

        elapsed = time.monotonic() - self.last_received
        if elapsed > self.timeout:
            self.publisher.publish(self.zero_command)
        else:
            self.publisher.publish(self.last_command)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Forward /cmd_vel with an automatic zero-velocity timeout."
    )
    parser.add_argument("--input-topic", default="/cmd_vel_raw")
    parser.add_argument("--output-topic", default="/cmd_vel")
    parser.add_argument("--timeout", type=float, default=0.8,
                        help="seconds without input before publishing zero")
    parser.add_argument("--publish-rate", type=float, default=20.0,
                        help="output publish rate in Hz")
    parser.add_argument("--max-linear-speed", type=float, default=0.30,
                        help="absolute linear.x limit in m/s")
    parser.add_argument("--max-angular-speed", type=float, default=1.00,
                        help="absolute angular.z limit in rad/s")
    args = parser.parse_args()

    if (
        args.timeout <= 0 or args.publish_rate <= 0
        or args.max_linear_speed <= 0 or args.max_angular_speed <= 0
    ):
        parser.error(
            "--timeout, --publish-rate, --max-linear-speed and "
            "--max-angular-speed must be positive"
        )

    rclpy.init()
    node = CmdVelWatchdog(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        timeout=args.timeout,
        publish_rate=args.publish_rate,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(node.zero_command)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
