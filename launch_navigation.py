#!/usr/bin/env python3
"""Generate navigation artifacts and launch Nav2 plus RViz."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from navigation_map_generator import generate_map


SCRIPT_DIR = Path(__file__).resolve().parent


def resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (SCRIPT_DIR / path).resolve()


def require_command(command: str) -> None:
    if shutil.which(command) is None:
        raise RuntimeError(f"required command not found: {command}")


def require_ros_package(package: str) -> None:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"ROS package is missing or not sourced: {package}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the map and launch snowplow Nav2 click-to-go navigation."
    )
    parser.add_argument(
        "--terrain-config", default="examples/terrain_navigation.yaml"
    )
    parser.add_argument("--map-dir", default="output/maps")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument(
        "--params", default="snowplow_navigation/config/nav2_params.yaml"
    )
    parser.add_argument("--no-rviz", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        require_command("ros2")
        for package in (
            "snowplow_navigation",
            "nav2_map_server",
            "nav2_controller",
            "nav2_planner",
            "nav2_bt_navigator",
            "nav2_behaviors",
            "nav2_lifecycle_manager",
            "nav2_mppi_controller",
            "nav2_navfn_planner",
            "nav2_smoother",
            "nav2_velocity_smoother",
        ):
            require_ros_package(package)
        if not args.no_rviz:
            require_ros_package("rviz2")
            require_ros_package("nav2_rviz_plugins")
    except RuntimeError as error:
        raise SystemExit(f"[launch_navigation] {error}") from error

    config_path = resolve_path(args.terrain_config)
    map_dir = resolve_path(args.map_dir)
    params_path = resolve_path(args.params)
    if not config_path.exists():
        raise SystemExit(f"[launch_navigation] terrain config not found: {config_path}")
    if not params_path.exists():
        raise SystemExit(f"[launch_navigation] Nav2 params not found: {params_path}")

    _, map_yaml = generate_map(config_path, map_dir, args.resolution)
    command = [
        "ros2",
        "launch",
        "snowplow_navigation",
        "navigation.launch.py",
        f"map:={map_yaml}",
        f"params_file:={params_path}",
        "use_sim_time:=true",
        "autostart:=true",
    ]
    print("[launch_navigation] Starting Nav2 ...")
    print(f"[launch_navigation] map: {map_yaml}")
    print("[launch_navigation] Do not run operator_console.py at the same time.")
    nav2_process = subprocess.Popen(command)
    rviz_process = None
    if not args.no_rviz:
        rviz_config = (
            Path(
                subprocess.run(
                    ["ros2", "pkg", "prefix", "snowplow_navigation"],
                    capture_output=True,
                    text=True,
                    check=True,
                ).stdout.strip()
            )
            / "share"
            / "snowplow_navigation"
            / "rviz"
            / "navigation.rviz"
        )
        rviz_process = subprocess.Popen(["rviz2", "-d", str(rviz_config)])

    try:
        return_code = nav2_process.wait()
    except KeyboardInterrupt:
        nav2_process.terminate()
        return_code = nav2_process.wait(timeout=10)
    finally:
        if rviz_process is not None and rviz_process.poll() is None:
            rviz_process.terminate()
            try:
                rviz_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                rviz_process.kill()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
