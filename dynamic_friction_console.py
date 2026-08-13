#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive terminal for changing dynamic friction and teleop speed."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DynamicFrictionConsole(Node):
    def __init__(
        self,
        friction_command_topic: str,
        state_topic: str,
        drive_command_topic: str,
        default_mu: float,
        default_mu2: float,
        default_linear_speed: float,
        default_angular_speed: float,
    ) -> None:
        super().__init__("dynamic_friction_console")
        self.friction_command_topic = friction_command_topic
        self.state_topic = state_topic
        self.drive_command_topic = drive_command_topic
        self.friction_publisher = self.create_publisher(
            Float64MultiArray, friction_command_topic, 10
        )
        self.drive_publisher = self.create_publisher(
            Float64MultiArray, drive_command_topic, 10
        )
        self.subscription = self.create_subscription(
            Float64MultiArray, state_topic, self._on_state, 10
        )
        self.current_mu = default_mu
        self.current_mu2 = default_mu2
        self.current_linear_speed = default_linear_speed
        self.current_angular_speed = default_angular_speed
        self.state_seen = False

    def _on_state(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 2:
            self.current_mu = float(message.data[0])
            self.current_mu2 = float(message.data[1])
            self.state_seen = True

    def publish_friction(self, mu: float, mu2: float) -> None:
        message = Float64MultiArray()
        message.data = [float(mu), float(mu2)]
        self.friction_publisher.publish(message)

    def publish_drive(self, linear_speed: float, angular_speed: float) -> None:
        message = Float64MultiArray()
        message.data = [float(linear_speed), float(angular_speed)]
        self.drive_publisher.publish(message)
        self.current_linear_speed = float(linear_speed)
        self.current_angular_speed = float(angular_speed)


def parse_nonnegative_numbers(items: list[str]) -> Optional[list[float]]:
    try:
        values = [float(item) for item in items]
    except ValueError:
        return None
    if any(value < 0.0 for value in values):
        return None
    return values


def print_help() -> None:
    print("")
    print("输入格式：")
    print("  mu mu2                 只更新动态地块摩擦，例如：0.30 0.15")
    print("  mu mu2 vx wz           同时更新摩擦和按键速度，例如：0.30 0.15 0.70 1.80")
    print("  speed vx [wz]          只更新终端 2 的按键速度，例如：speed 0.80 2.20")
    print("  show                   显示当前已知状态")
    print("  q                      退出")


def parse_command(text: str) -> Optional[tuple[str, list[float]]]:
    parts = text.strip().split()
    if not parts:
        return None

    keyword = parts[0].lower()
    if keyword in {"h", "help", "?"}:
        return ("help", [])
    if keyword == "show":
        return ("show", [])
    if keyword == "speed":
        values = parse_nonnegative_numbers(parts[1:])
        if values is None or len(values) not in (1, 2):
            return None
        return ("speed", values)

    values = parse_nonnegative_numbers(parts)
    if values is None or len(values) not in (2, 4):
        return None
    return ("friction", values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Update the Gazebo dynamic friction pad and teleop speed."
    )
    parser.add_argument("--command-topic", default="/dynamic_friction/set")
    parser.add_argument("--state-topic", default="/dynamic_friction/state")
    parser.add_argument("--drive-command-topic", default="/dynamic_drive/set")
    parser.add_argument("--default-mu", type=float, default=0.80)
    parser.add_argument("--default-mu2", type=float, default=0.45)
    parser.add_argument("--default-linear-speed", type=float, default=0.50)
    parser.add_argument("--default-angular-speed", type=float, default=1.30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (
        args.default_mu < 0.0
        or args.default_mu2 < 0.0
        or args.default_linear_speed < 0.0
        or args.default_angular_speed < 0.0
    ):
        raise SystemExit("默认 mu、mu2 和速度必须为非负数。")

    rclpy.init()
    node = DynamicFrictionConsole(
        friction_command_topic=args.command_topic,
        state_topic=args.state_topic,
        drive_command_topic=args.drive_command_topic,
        default_mu=args.default_mu,
        default_mu2=args.default_mu2,
        default_linear_speed=args.default_linear_speed,
        default_angular_speed=args.default_angular_speed,
    )

    try:
        print("DYNAMIC FRICTION PAD")
        print("====================")
        print("动态地块摩擦：发布到 /dynamic_friction/set")
        print("按键速度参数：发布到 /dynamic_drive/set，由终端 2 接收")
        print_help()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            source = "gazebo" if node.state_seen else "default"
            print(
                f"\ncurrent ({source}): mu={node.current_mu:.3f} "
                f"mu2={node.current_mu2:.3f} | "
                f"speed vx={node.current_linear_speed:.3f} m/s "
                f"wz={node.current_angular_speed:.3f} rad/s"
            )
            try:
                text = input("input > ")
            except EOFError:
                break
            if text.strip().lower() in {"q", "quit", "exit"}:
                break
            command = parse_command(text)
            if command is None:
                print("格式错误。输入 help 查看示例。")
                continue
            kind, values = command
            if kind == "help":
                print_help()
                continue
            if kind == "show":
                continue
            if kind == "speed":
                linear_speed = values[0]
                angular_speed = values[1] if len(values) >= 2 else node.current_angular_speed
                node.publish_drive(linear_speed, angular_speed)
                rclpy.spin_once(node, timeout_sec=0.1)
                print(f"sent speed: vx={linear_speed:.3f} wz={angular_speed:.3f}")
                continue

            mu, mu2 = values[0], values[1]
            node.publish_friction(mu, mu2)
            if len(values) == 4:
                node.publish_drive(values[2], values[3])
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(values) == 4:
                print(
                    f"sent: mu={mu:.3f} mu2={mu2:.3f} "
                    f"vx={values[2]:.3f} wz={values[3]:.3f}"
                )
            else:
                print(f"sent friction: mu={mu:.3f} mu2={mu2:.3f}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write("\n")


if __name__ == "__main__":
    main()
