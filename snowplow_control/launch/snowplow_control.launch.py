#!/usr/bin/env python3
"""Spawn controllers for the Gazebo-hosted controller_manager."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    pmsm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pmsm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        TimerAction(period=3.0, actions=[joint_state_spawner, pmsm_spawner]),
    ])
