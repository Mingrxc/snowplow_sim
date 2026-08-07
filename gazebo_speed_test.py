#!/usr/bin/env python3
"""Run a fixed-duration straight-line Gazebo speed test.

The robot must already be spawned on the desired terrain. The node publishes
the same forward command for every run, collects /odom during the drive phase,
and publishes zero velocity before exiting.
"""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GazeboSpeedTest(Node):
    def __init__(self, label: str, command_speed: float, settle: float,
                 duration: float, stop_time: float, output: Path | None):
        super().__init__("gazebo_speed_test")
        self.label = label
        self.command_speed = command_speed
        self.settle = settle
        self.duration = duration
        self.stop_time = stop_time
        self.output = output
        self.started_at = time.monotonic()
        self.samples = []
        self.done = False
        self.success = False

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self._on_odom, 20
        )
        self.timer = self.create_timer(0.05, self._tick)

    def _on_odom(self, message: Odometry) -> None:
        elapsed = time.monotonic() - self.started_at
        if self.settle <= elapsed < self.settle + self.duration:
            self.samples.append(
                (elapsed, float(message.twist.twist.linear.x))
            )

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started_at
        command = Twist()

        if elapsed < self.settle:
            pass
        elif elapsed < self.settle + self.duration:
            command.linear.x = self.command_speed
        elif elapsed < self.settle + self.duration + self.stop_time:
            pass
        else:
            self._finish()
            return

        self.publisher.publish(command)

    def _finish(self) -> None:
        self.publisher.publish(Twist())
        self.timer.cancel()
        self.done = True

        if not self.samples:
            self.get_logger().error(
                "No /odom samples were received during the drive phase."
            )
            return

        values = [value for _, value in self.samples]
        mean_speed = sum(values) / len(values)
        max_speed = max(values)
        min_speed = min(values)
        variance = sum((value - mean_speed) ** 2 for value in values) / len(values)
        std_speed = variance ** 0.5

        print("\n=== Gazebo speed test ===")
        print(f"terrain:       {self.label}")
        print(f"command:       {self.command_speed:.4f} m/s")
        print(f"samples:       {len(values)}")
        print(f"mean odom:     {mean_speed:.4f} m/s")
        print(f"min odom:      {min_speed:.4f} m/s")
        print(f"max odom:      {max_speed:.4f} m/s")
        print(f"std odom:      {std_speed:.4f} m/s")
        self.success = True

        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            exists = self.output.exists()
            with self.output.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "terrain", "command_speed", "duration", "samples",
                        "mean_odom_speed", "min_odom_speed",
                        "max_odom_speed", "std_odom_speed",
                    ],
                )
                if not exists:
                    writer.writeheader()
                writer.writerow({
                    "terrain": self.label,
                    "command_speed": f"{self.command_speed:.6f}",
                    "duration": f"{self.duration:.3f}",
                    "samples": len(values),
                    "mean_odom_speed": f"{mean_speed:.6f}",
                    "min_odom_speed": f"{min_speed:.6f}",
                    "max_odom_speed": f"{max_speed:.6f}",
                    "std_odom_speed": f"{std_speed:.6f}",
                })
            print(f"csv:           {self.output}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Measure fixed-duration straight-line speed in Gazebo."
    )
    parser.add_argument("--label", required=True,
                        help="terrain name, for example snow or ice")
    parser.add_argument("--speed", type=float, default=0.20,
                        help="forward command in m/s")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds to publish zero before the test")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="seconds of forward motion")
    parser.add_argument("--stop-time", type=float, default=1.0,
                        help="seconds to publish zero before exit")
    parser.add_argument("--output", type=Path,
                        help="optional CSV path for the summary row")
    args = parser.parse_args()

    if args.speed < 0 or args.settle < 0 or args.duration <= 0 or args.stop_time < 0:
        parser.error("speed must be >= 0; duration must be > 0")

    rclpy.init()
    node = GazeboSpeedTest(
        label=args.label,
        command_speed=args.speed,
        settle=args.settle,
        duration=args.duration,
        stop_time=args.stop_time,
        output=args.output,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if node.success else 1)


if __name__ == "__main__":
    main()
