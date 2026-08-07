#!/usr/bin/env python3
"""Run a fixed-duration straight-line Gazebo terrain test.

The robot must already be spawned on the desired terrain. The node publishes
the same forward command for every run, collects /odom during the drive phase,
computes speed and straight-line tracking metrics, and publishes zero velocity
before exiting.
"""

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    """Return planar yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboSpeedTest(Node):
    def __init__(self, label: str, command_speed: float, settle: float,
                 duration: float, stop_time: float, output: Path | None,
                 raw_output: Path | None, manifest_output: Path | None):
        super().__init__("gazebo_speed_test")
        self.label = label
        self.command_speed = command_speed
        self.settle = settle
        self.duration = duration
        self.stop_time = stop_time
        self.output = output
        self.raw_output = raw_output
        self.manifest_output = manifest_output
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
            pose = message.pose.pose
            twist = message.twist.twist
            self.samples.append(
                {
                    "time": elapsed - self.settle,
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                    "yaw": yaw_from_quaternion(pose.orientation),
                    "vx": float(twist.linear.x),
                    "vy": float(twist.linear.y),
                    "wz": float(twist.angular.z),
                }
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

        metrics = self._compute_metrics()
        values = [sample["vx"] for sample in self.samples]
        mean_speed = metrics["mean_odom_speed"]
        max_speed = max(values)
        min_speed = min(values)
        std_speed = metrics["std_odom_speed"]

        print("\n=== Gazebo terrain test ===")
        print(f"terrain:       {self.label}")
        print(f"command:       {self.command_speed:.4f} m/s")
        print(f"samples:       {len(values)}")
        print(f"mean odom:     {mean_speed:.4f} m/s")
        print(f"min odom:      {min_speed:.4f} m/s")
        print(f"max odom:      {max_speed:.4f} m/s")
        print(f"std odom:      {std_speed:.4f} m/s")
        print(f"speed error:   {metrics['speed_relative_error'] * 100:.2f} %")
        print(f"lateral max:   {metrics['max_lateral_deviation']:.4f} m")
        print(f"heading drift: {metrics['heading_change']:.4f} rad")
        self.success = True

        if self.output is not None:
            self._write_summary(metrics)
            print(f"csv:           {self.output}")
        if self.raw_output is not None:
            self._write_raw_samples()
            print(f"raw csv:       {self.raw_output}")
        if self.manifest_output is not None:
            self._write_manifest(metrics)
            print(f"manifest:      {self.manifest_output}")

    def _compute_metrics(self) -> dict:
        values = [sample["vx"] for sample in self.samples]
        mean_speed = sum(values) / len(values)
        variance = sum((value - mean_speed) ** 2 for value in values) / len(values)

        first = self.samples[0]
        prev = first
        yaw0 = first["yaw"]
        lateral_values = []
        path_length = 0.0
        forward_displacement = 0.0

        for sample in self.samples:
            dx0 = sample["x"] - first["x"]
            dy0 = sample["y"] - first["y"]
            lateral_values.append(-math.sin(yaw0) * dx0 + math.cos(yaw0) * dy0)
            forward_displacement = math.cos(yaw0) * dx0 + math.sin(yaw0) * dy0
            path_length += math.hypot(sample["x"] - prev["x"], sample["y"] - prev["y"])
            prev = sample

        abs_command = abs(self.command_speed)
        speed_error = abs(mean_speed - self.command_speed)
        speed_relative_error = (
            speed_error / abs_command if abs_command > 1.0e-9 else 0.0
        )
        lateral_rms = math.sqrt(
            sum(value * value for value in lateral_values) / len(lateral_values)
        )

        return {
            "terrain": self.label,
            "command_speed": self.command_speed,
            "duration": self.duration,
            "samples": len(self.samples),
            "mean_odom_speed": mean_speed,
            "min_odom_speed": min(values),
            "max_odom_speed": max(values),
            "std_odom_speed": math.sqrt(variance),
            "speed_abs_error": speed_error,
            "speed_relative_error": speed_relative_error,
            "start_x": first["x"],
            "start_y": first["y"],
            "start_yaw": first["yaw"],
            "end_x": self.samples[-1]["x"],
            "end_y": self.samples[-1]["y"],
            "end_yaw": self.samples[-1]["yaw"],
            "forward_displacement": forward_displacement,
            "path_length": path_length,
            "final_lateral_deviation": lateral_values[-1],
            "max_lateral_deviation": max(abs(value) for value in lateral_values),
            "rms_lateral_deviation": lateral_rms,
            "heading_change": wrap_angle(self.samples[-1]["yaw"] - first["yaw"]),
        }

    def _write_summary(self, metrics: dict) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = list(metrics.keys())
        exists = self.output.exists()
        if exists:
            with self.output.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.reader(handle)
                existing_header = next(reader, [])
            if existing_header and existing_header != fieldnames:
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.output = self.output.with_name(
                    f"{self.output.stem}_{stamp}{self.output.suffix}"
                )
                exists = False
        with self.output.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            if not exists:
                writer.writeheader()
            writer.writerow({
                key: f"{value:.6f}" if isinstance(value, float) else value
                for key, value in metrics.items()
            })

    def _write_raw_samples(self) -> None:
        self.raw_output.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["time", "x", "y", "z", "yaw", "vx", "vy", "wz"]
        with self.raw_output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow({
                    key: f"{sample[key]:.9f}" for key in fieldnames
                })

    def _write_manifest(self, metrics: dict) -> None:
        self.manifest_output.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "script": Path(__file__).name,
            "result_type": "gazebo_odom_measurement",
            "note": "/odom is Gazebo ground-truth odometry in the current v1 route, not encoder/IMU/RTK fused odometry.",
            "config": {
                "terrain": self.label,
                "command_speed": self.command_speed,
                "settle": self.settle,
                "duration": self.duration,
                "stop_time": self.stop_time,
                "cmd_topic": "/cmd_vel",
                "odom_topic": "/odom",
            },
            "outputs": {
                "summary_csv": str(self.output) if self.output else None,
                "raw_csv": str(self.raw_output) if self.raw_output else None,
                "manifest_json": str(self.manifest_output),
            },
            "metrics": metrics,
        }
        with self.manifest_output.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)


def default_run_dir(label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_label = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label)
    return Path("results") / "gazebo_terrain" / f"{stamp}_{safe_label}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Measure fixed-duration straight-line terrain response in Gazebo."
    )
    parser.add_argument("--label", required=True,
                        help="terrain name, for example concrete, sand, or snow")
    parser.add_argument("--speed", type=float, default=0.20,
                        help="forward command in m/s")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds to publish zero before the test")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="seconds of forward motion")
    parser.add_argument("--stop-time", type=float, default=1.0,
                        help="seconds to publish zero before exit")
    parser.add_argument("--output", type=Path,
                        help="optional summary CSV path")
    parser.add_argument("--run-dir", type=Path,
                        help="result directory; default: results/gazebo_terrain/<time>_<label>")
    parser.add_argument("--no-save", action="store_true",
                        help="print metrics only; do not save CSV or manifest")
    args = parser.parse_args()

    if args.speed < 0 or args.settle < 0 or args.duration <= 0 or args.stop_time < 0:
        parser.error("speed must be >= 0; duration must be > 0")

    output = None
    raw_output = None
    manifest_output = None
    if not args.no_save:
        run_dir = args.run_dir or default_run_dir(args.label)
        output = args.output or run_dir / "summary.csv"
        raw_output = run_dir / "odom_samples.csv"
        manifest_output = run_dir / "manifest.json"

    rclpy.init()
    node = GazeboSpeedTest(
        label=args.label,
        command_speed=args.speed,
        settle=args.settle,
        duration=args.duration,
        stop_time=args.stop_time,
        output=output,
        raw_output=raw_output,
        manifest_output=manifest_output,
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
