#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch Gazebo, spawn the snowplow, and start ros2_control controllers."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).parent.resolve()


def check_dependencies(start_controllers: bool) -> bool:
    required = ["xacro", "ros2", "gazebo"]
    issues = [tool for tool in required if shutil.which(tool) is None]

    result = subprocess.run(
        ["ros2", "pkg", "prefix", "robot_state_publisher"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        issues.append("ROS package missing: robot_state_publisher")

    if start_controllers:
        for package in (
            "controller_manager",
            "joint_state_broadcaster",
            "gazebo_ros2_control",
        ):
            result = subprocess.run(
                ["ros2", "pkg", "prefix", package],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                issues.append(f"ROS package missing: {package}")

        result = subprocess.run(
            ["ros2", "pkg", "prefix", "snowplow_control"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            issues.append("snowplow_control is not built/sourced")

    if issues:
        print("[launch_sim] Missing dependencies:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    return True


def resolve_project_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    return path if path.is_absolute() else SCRIPT_DIR / path


def compile_xacro(xacro_path: Path) -> str:
    result = subprocess.run(
        ["xacro", str(xacro_path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip())
    return result.stdout


def write_urdf(urdf_text: str, output_path: str | None) -> Path:
    if output_path:
        path = resolve_project_path(output_path)
    else:
        path = Path(tempfile.gettempdir()) / "snowplow_robot.urdf"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(urdf_text, encoding="utf-8")
    return path


def wait_for_service(service_name: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if service_name in result.stdout.splitlines():
            return True
        time.sleep(1.0)
    return False


def spawn_robot(urdf_path: Path, entity_name: str, x: float, y: float, z: float) -> None:
    result = subprocess.run(
        [
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-file",
            str(urdf_path),
            "-entity",
            entity_name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())


def entity_name_from_xacro(xacro_path: Path) -> str:
    name = xacro_path.name
    for suffix in (".urdf.xacro", ".xacro", ".urdf"):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return xacro_path.stem


def start_controller(controller_name: str, timeout: float) -> None:
    result = subprocess.run(
        [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            controller_name,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            str(int(timeout)),
        ],
        capture_output=True,
        text=True,
        timeout=timeout + 10.0,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())


def start_gazebo(world_path: Path, verbose: bool) -> subprocess.Popen:
    command = [
        "gazebo",
        "-s",
        "libgazebo_ros_init.so",
        "-s",
        "libgazebo_ros_factory.so",
    ]
    if verbose:
        command.append("--verbose")
    command.append(str(world_path))

    if verbose:
        return subprocess.Popen(command)
    return subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def start_robot_state_publisher(urdf_path: Path, verbose: bool) -> subprocess.Popen:
    command = [
        "ros2",
        "run",
        "robot_state_publisher",
        "robot_state_publisher",
        str(urdf_path),
        "--ros-args",
        "-p",
        "use_sim_time:=true",
    ]
    if verbose:
        return subprocess.Popen(command)
    return subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Launch the snowplow Gazebo simulation with ros2_control."
    )
    parser.add_argument("--xacro", required=True, help="Generated .urdf.xacro path.")
    parser.add_argument("--world", required=True, help="Generated Gazebo .world path.")
    parser.add_argument("--auto", action="store_true", help="Start Gazebo and spawn robot.")
    parser.add_argument("--verbose", action="store_true", help="Show Gazebo logs.")
    parser.add_argument("--rviz", action="store_true", help="Start RViz2 as a helper window.")
    parser.add_argument("--no-controllers", action="store_true", help="Spawn without starting controllers.")
    parser.add_argument("--controller-timeout", type=float, default=60.0)
    parser.add_argument("--urdf-out", default=None, help="Expanded URDF output path.")
    parser.add_argument("--entity", default=None, help="Gazebo entity name.")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.35)
    args = parser.parse_args()

    xacro_path = resolve_project_path(args.xacro)
    world_path = resolve_project_path(args.world)
    start_controllers = not args.no_controllers

    if not xacro_path.exists():
        print(f"[launch_sim] Xacro not found: {xacro_path}")
        sys.exit(1)
    if not world_path.exists():
        print(f"[launch_sim] World not found: {world_path}")
        sys.exit(1)
    if not check_dependencies(start_controllers):
        sys.exit(1)

    print(f"[launch_sim] Compiling Xacro: {xacro_path}")
    try:
        urdf_text = compile_xacro(xacro_path)
        urdf_path = write_urdf(urdf_text, args.urdf_out)
    except RuntimeError as error:
        print(f"[launch_sim] Xacro failed:\n{error}")
        sys.exit(1)
    print(f"[launch_sim] URDF written: {urdf_path}")

    entity_name = args.entity or entity_name_from_xacro(xacro_path)
    if not args.auto:
        print("\nManual run:")
        print(f"  gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so {world_path}")
        print(
            "  ros2 run robot_state_publisher robot_state_publisher "
            f"{urdf_path} --ros-args -p use_sim_time:=true"
        )
        print(
            "  ros2 run gazebo_ros spawn_entity.py "
            f"-file {urdf_path} -entity {entity_name} -x {args.x} -y {args.y} -z {args.z}"
        )
        print("  ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager")
        print("  ros2 run controller_manager spawner pmsm_controller -c /controller_manager")
        return

    print("[launch_sim] Starting robot_state_publisher ...")
    start_robot_state_publisher(urdf_path, args.verbose)
    print("[launch_sim] Waiting for /robot_state_publisher/get_parameters ...")
    if not wait_for_service("/robot_state_publisher/get_parameters", 15.0):
        print("[launch_sim] Timed out waiting for robot_state_publisher.")
        sys.exit(1)

    print(f"[launch_sim] Starting Gazebo: {world_path}")
    start_gazebo(world_path, args.verbose)

    if args.rviz:
        subprocess.Popen(["rviz2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[launch_sim] Waiting for /spawn_entity ...")
    if not wait_for_service("/spawn_entity", 45.0):
        print("[launch_sim] Timed out waiting for /spawn_entity.")
        sys.exit(1)

    print(f"[launch_sim] Spawning {entity_name} ...")
    try:
        spawn_robot(urdf_path, entity_name, args.x, args.y, args.z)
    except RuntimeError as error:
        print(f"[launch_sim] Spawn failed:\n{error}")
        sys.exit(1)
    print("[launch_sim] Robot spawned.")

    if not start_controllers:
        return

    print("[launch_sim] Waiting for /controller_manager/list_controllers ...")
    if not wait_for_service("/controller_manager/list_controllers", args.controller_timeout):
        print("[launch_sim] Timed out waiting for controller_manager.")
        sys.exit(1)

    for controller_name in ("joint_state_broadcaster", "pmsm_controller"):
        print(f"[launch_sim] Starting controller: {controller_name}")
        try:
            start_controller(controller_name, args.controller_timeout)
        except RuntimeError as error:
            print(f"[launch_sim] Controller '{controller_name}' failed:\n{error}")
            sys.exit(1)

    print("[launch_sim] Simulation is ready. Publish /cmd_vel or run operator_console.py.")


if __name__ == "__main__":
    main()
