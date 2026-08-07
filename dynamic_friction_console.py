#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive terminal for changing the dynamic friction pad at runtime."""

from __future__ import annotations

import argparse
import sys
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DynamicFrictionConsole(Node):
    def __init__(
        self,
        command_topic: str,
        state_topic: str,
        default_mu: float,
        default_mu2: float,
    ) -> None:
        super().__init__("dynamic_friction_console")
        self.command_topic = command_topic
        self.state_topic = state_topic
        self.publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, state_topic, self._on_state, 10
        )
        self.current_mu = default_mu
        self.current_mu2 = default_mu2
        self.state_seen = False

    def _on_state(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 2:
            self.current_mu = float(message.data[0])
            self.current_mu2 = float(message.data[1])
            self.state_seen = True

    def publish(self, mu: float, mu2: float) -> None:
        message = Float64MultiArray()
        message.data = [float(mu), float(mu2)]
        self.publisher.publish(message)


def parse_pair(text: str) -> Optional[Tuple[float, float]]:
    parts = text.strip().split()
    if len(parts) != 2:
        return None
    try:
        mu = float(parts[0])
        mu2 = float(parts[1])
    except ValueError:
        return None
    if mu < 0.0 or mu2 < 0.0:
        return None
    return mu, mu2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Update the Gazebo dynamic friction pad using mu and mu2."
    )
    parser.add_argument("--command-topic", default="/dynamic_friction/set")
    parser.add_argument("--state-topic", default="/dynamic_friction/state")
    parser.add_argument("--default-mu", type=float, default=0.80)
    parser.add_argument("--default-mu2", type=float, default=0.50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.default_mu < 0.0 or args.default_mu2 < 0.0:
        raise SystemExit("默认 mu 和 mu2 必须为非负数。")

    rclpy.init()
    node = DynamicFrictionConsole(
        command_topic=args.command_topic,
        state_topic=args.state_topic,
        default_mu=args.default_mu,
        default_mu2=args.default_mu2,
    )

    try:
        print("DYNAMIC FRICTION PAD")
        print("====================")
        print("输入两个非负数：mu mu2。例如：0.30 0.15")
        print("输入 q 退出。")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            source = "gazebo" if node.state_seen else "default"
            print(
                f"\ncurrent ({source}): mu={node.current_mu:.3f} "
                f"mu2={node.current_mu2:.3f}"
            )
            try:
                text = input("input mu mu2 > ")
            except EOFError:
                break
            if text.strip().lower() in {"q", "quit", "exit"}:
                break
            pair = parse_pair(text)
            if pair is None:
                print("格式错误。请只输入两个非负数，例如：0.30 0.15")
                continue
            node.publish(*pair)
            rclpy.spin_once(node, timeout_sec=0.2)
            print(f"sent: mu={pair[0]:.3f} mu2={pair[1]:.3f}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write("\n")


if __name__ == "__main__":
    main()
