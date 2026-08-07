#!/usr/bin/env python3
"""Interactive keyboard control and live telemetry for the snowplow.

This node is intentionally kept as a standalone script so it can run directly
from the project directory on Ubuntu without a colcon install step.

The terminal controller is a "hold to move" interface:

* a direction key refreshes the command for a short safety window;
* when no direction input arrives before the timeout, zero velocity is sent;
* the node publishes directly to /cmd_vel and must not be run together with
  another publisher that is also driving the same topic.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from world_generator import TERRAIN_PRESETS, default_zones, load_config


SCRIPT_DIR = Path(__file__).resolve().parent

# (display name, linear multiplier, angular multiplier)
KeyCommand = Tuple[str, float, float]

KEY_COMMANDS: Dict[str, KeyCommand] = {
    "i": ("FORWARD", 1.0, 0.0),
    "w": ("FORWARD", 1.0, 0.0),
    "k": ("BACKWARD", -1.0, 0.0),
    "s": ("BACKWARD", -1.0, 0.0),
    "a": ("SPIN_LEFT", 0.0, 1.6),
    "d": ("SPIN_RIGHT", 0.0, -1.6),
    "j": ("SPIN_LEFT", 0.0, 1.0),
    "l": ("SPIN_RIGHT", 0.0, -1.0),
    "\x1b[A": ("FORWARD", 1.0, 0.0),
    "\x1b[B": ("BACKWARD", -1.0, 0.0),
    "\x1b[D": ("SPIN_LEFT", 0.0, 1.6),
    "\x1b[C": ("SPIN_RIGHT", 0.0, -1.6),
}


class RawTerminal:
    """Temporarily enable character-at-a-time input without echo."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self._old_settings = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise RuntimeError(
                "operator_console.py 需要在可交互的 Ubuntu 终端中运行。"
            )
        self._old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._old_settings is not None:
            termios.tcsetattr(
                self.fd, termios.TCSADRAIN, self._old_settings
            )
        sys.stdout.write("\n")
        sys.stdout.flush()


def read_key(fd: int, escape_timeout: float = 0.03) -> Optional[str]:
    """Read one key or one ANSI arrow sequence when available."""

    try:
        first = os.read(fd, 1)
    except OSError:
        return None

    if not first:
        return None
    if first == b"\x03":
        return "q"
    if first != b"\x1b":
        try:
            return first.decode("utf-8", errors="ignore")
        except UnicodeDecodeError:
            return None

    sequence = bytearray(first)
    deadline = time.monotonic() + escape_timeout
    while len(sequence) < 3 and time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select([fd], [], [], remaining)
        if not ready:
            break
        sequence.extend(os.read(fd, 1))
    return bytes(sequence).decode("ascii", errors="ignore")


class TerrainLookup:
    """Resolve the configured terrain under the robot's odometry position."""

    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.zones = []
        self.slope = {}
        self.error = ""

        try:
            cfg = load_config(str(config_path))
            self.zones = list(cfg.get("terrain_zones", []))
            if not self.zones:
                self.zones = default_zones(float(cfg["ground_size"]))
            self.slope = dict(cfg.get("slope", {}))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.error = str(exc)

    @staticmethod
    def _contains(zone: dict, x: float, y: float) -> bool:
        half_width = float(zone.get("width", 0.0)) / 2.0
        half_depth = float(zone.get("depth", 0.0)) / 2.0
        return (
            abs(x - float(zone.get("x", 0.0))) <= half_width
            and abs(y - float(zone.get("y", 0.0))) <= half_depth
        )

    def _slope_zone(self, x: float, y: float) -> Optional[dict]:
        if not self.slope.get("enabled", False):
            return None
        if not self._contains(self.slope, x, y):
            return None
        return {
            "type": "slope",
            "terrain_type": self.slope.get("terrain_type", "asphalt"),
            "x": self.slope.get("x", 0.0),
            "y": self.slope.get("y", 0.0),
            "width": self.slope.get("width", 0.0),
            "depth": self.slope.get("depth", 0.0),
        }

    def lookup(self, x: float, y: float) -> Tuple[str, Optional[dict]]:
        slope_zone = self._slope_zone(x, y)
        if slope_zone is not None:
            return "slope", slope_zone

        # Later SDF models are placed on top of earlier overlapping models.
        # Reverse the list so the telemetry follows that visible/physical order.
        for zone in reversed(self.zones):
            if self._contains(zone, x, y):
                return str(zone.get("type", "unknown")), zone

        return "outside", None

    def preset_for(self, terrain_name: str) -> dict:
        if terrain_name == "slope":
            return TERRAIN_PRESETS.get(
                str(self.slope.get("terrain_type", "asphalt")),
                TERRAIN_PRESETS["asphalt"],
            )
        return TERRAIN_PRESETS.get(terrain_name, {})


class OperatorConsole(Node):
    def __init__(
        self,
        cmd_topic: str,
        odom_topic: str,
        terrain_lookup: TerrainLookup,
        linear_speed: float,
        angular_speed: float,
        max_linear_speed: float,
        max_angular_speed: float,
        input_timeout: float,
        publish_rate: float,
        display_rate: float,
    ):
        super().__init__("snowplow_operator_console")
        self.cmd_topic = cmd_topic
        self.odom_topic = odom_topic
        self.terrain_lookup = terrain_lookup
        self.linear_speed = min(abs(linear_speed), max_linear_speed)
        self.angular_speed = min(abs(angular_speed), max_angular_speed)
        self.max_linear_speed = abs(max_linear_speed)
        self.max_angular_speed = abs(max_angular_speed)
        self.input_timeout = input_timeout
        self.publish_rate = publish_rate
        self.display_rate = display_rate

        self.publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 20
        )

        self.last_input_at: Optional[float] = None
        self.last_key = "-"
        self.command_name = "WAITING"
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.published_linear = 0.0
        self.published_angular = 0.0
        self.manual_stop = False
        self.quit_requested = False
        self.last_publish_at = 0.0
        self.last_display_at = 0.0

        self.odom_received = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose.position
        twist = message.twist.twist
        self.odom_x = float(pose.x)
        self.odom_y = float(pose.y)
        self.odom_z = float(pose.z)
        self.odom_vx = float(twist.linear.x)
        self.odom_vy = float(twist.linear.y)
        self.odom_wz = float(twist.angular.z)
        self.odom_received = True

    def handle_key(self, key: str) -> None:
        if key in ("q", "Q"):
            self.quit_requested = True
            return

        if key in (" ", "\x00"):
            self.last_key = key
            self.last_input_at = time.monotonic()
            self.command_name = "STOPPED"
            self.command_linear = 0.0
            self.command_angular = 0.0
            self.manual_stop = True
            return

        command = KEY_COMMANDS.get(key)
        if command is None:
            return

        now = time.monotonic()
        self.last_key = key
        self.last_input_at = now
        self.command_name, linear_sign, angular_sign = command
        self.command_linear = linear_sign * self.linear_speed
        self.command_angular = angular_sign * self.angular_speed
        self.manual_stop = False

    def _command_is_alive(self, now: float) -> bool:
        return (
            not self.manual_stop
            and self.last_input_at is not None
            and now - self.last_input_at <= self.input_timeout
        )

    def _safe_command(self, now: float) -> Twist:
        command = Twist()
        if self._command_is_alive(now):
            command.linear.x = max(
                -self.max_linear_speed,
                min(self.max_linear_speed, self.command_linear),
            )
            command.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.command_angular),
            )
        return command

    def publish_command(self, now: float) -> str:
        command = self._safe_command(now)
        self.publisher.publish(command)
        self.published_linear = float(command.linear.x)
        self.published_angular = float(command.angular.z)
        self.last_publish_at = now
        if command.linear.x == 0.0 and command.angular.z == 0.0:
            if self.manual_stop:
                return "STOPPED"
            if self.last_input_at is None:
                return "WAITING"
            return "TIMEOUT"
        return "ACTIVE"

    def _format_terrain(self) -> Tuple[str, dict]:
        terrain_name, _ = self.terrain_lookup.lookup(self.odom_x, self.odom_y)
        preset = self.terrain_lookup.preset_for(terrain_name)
        return terrain_name, preset

    def render(self, now: float, control_state: str) -> None:
        terrain_name, preset = self._format_terrain()
        planar_speed = math.hypot(self.odom_vx, self.odom_vy)
        input_age = (
            float("inf")
            if self.last_input_at is None
            else max(0.0, now - self.last_input_at)
        )
        odom_state = "OK" if self.odom_received else "WAITING"
        config_state = "OK" if not self.terrain_lookup.error else "ERROR"

        def value(key: str, default: str = "-") -> str:
            item = preset.get(key)
            return default if item is None else f"{float(item):.3f}"

        lines = [
            "\x1b[2J\x1b[H",
            "SNOWPLOW OPERATOR CONSOLE",
            "=========================",
            "Press or hold a direction key to move; Space stops immediately.",
            "i/w or Up: forward    k/s or Down: backward",
            "a/d or Left/Right: in-place spin for heading change",
            "j/l: slower in-place spin left/right",
            "Space: stop immediately    q or Ctrl+C: quit and stop",
            "",
            f"CONTROL  : {control_state:<8} command={self.command_name:<10} "
            f"last_key={self.last_key!r}",
            f"TARGET   : vx={self.command_linear:+.3f} m/s  "
            f"wz={self.command_angular:+.3f} rad/s",
            f"PUBLISHED: vx={self.published_linear:+.3f} m/s  "
            f"wz={self.published_angular:+.3f} rad/s",
            f"INPUT AGE: {input_age:.3f} s"
            if math.isfinite(input_age)
            else "INPUT AGE: -",
            "",
            f"ODOM     : {odom_state}  x={self.odom_x:+.3f}  "
            f"y={self.odom_y:+.3f}  z={self.odom_z:+.3f} m",
            f"VELOCITY : vx={self.odom_vx:+.3f}  vy={self.odom_vy:+.3f}  "
            f"planar={planar_speed:.3f} m/s  wz={self.odom_wz:+.3f} rad/s",
            "",
            f"TERRAIN  : {terrain_name:<12} config={config_state}",
            f"FRICTION : mu={value('mu')}  mu2={value('mu2')}",
            f"CONTACT  : kp={value('kp')}  kd={value('kd')}",
            f"CONTACT  : min_depth={value('min_depth')}  "
            f"max_vel={value('max_vel')}",
            "",
            f"TOPICS   : cmd={self.cmd_topic}  odom={self.odom_topic}",
            f"LIMITS   : linear={self.max_linear_speed:.3f} m/s  "
            f"angular={self.max_angular_speed:.3f} rad/s",
        ]
        if self.terrain_lookup.error:
            lines.append(f"CONFIG ERROR: {self.terrain_lookup.error}")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        self.last_display_at = now

    def stop(self) -> None:
        self.publisher.publish(Twist())


def resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = SCRIPT_DIR / path
    return path.resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Hold-to-move keyboard control with live odom/terrain telemetry."
    )
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--terrain-config",
        default="examples/terrain_multi.yaml",
        help="World YAML used to resolve the configured terrain under the robot.",
    )
    parser.add_argument("--linear-speed", type=float, default=0.50)
    parser.add_argument("--angular-speed", type=float, default=1.30)
    parser.add_argument("--max-linear-speed", type=float, default=0.80)
    parser.add_argument("--max-angular-speed", type=float, default=2.60)
    parser.add_argument(
        "--input-timeout",
        type=float,
        default=1.00,
        help="Seconds without a repeated key before publishing zero.",
    )
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--display-rate", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (
        args.linear_speed < 0
        or args.angular_speed < 0
        or args.max_linear_speed <= 0
        or args.max_angular_speed <= 0
        or args.input_timeout <= 0
        or args.publish_rate <= 0
        or args.display_rate <= 0
    ):
        raise SystemExit("速度、限幅、超时和频率参数必须为正数。")

    terrain_lookup = TerrainLookup(resolve_path(args.terrain_config))
    rclpy.init()
    node = OperatorConsole(
        cmd_topic=args.cmd_topic,
        odom_topic=args.odom_topic,
        terrain_lookup=terrain_lookup,
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        input_timeout=args.input_timeout,
        publish_rate=args.publish_rate,
        display_rate=args.display_rate,
    )

    try:
        with RawTerminal() as terminal:
            print("\x1b[2J\x1b[H", end="")
            while rclpy.ok() and not node.quit_requested:
                loop_started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.0)

                ready, _, _ = select.select(
                    [terminal.fd], [], [], 1.0 / node.publish_rate
                )
                if ready:
                    key = read_key(terminal.fd)
                    if key is not None:
                        node.handle_key(key)

                now = time.monotonic()
                state = node.publish_command(now)
                if (
                    now - node.last_display_at
                    >= 1.0 / node.display_rate
                ):
                    node.render(now, state)

                elapsed = time.monotonic() - loop_started
                remaining = 1.0 / node.publish_rate - elapsed
                if remaining > 0:
                    time.sleep(min(remaining, 0.01))
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
