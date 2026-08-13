from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("snowplow_navigation"))
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]
    common = {
        "output": "screen",
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
    }
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config" / "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            Node(
                package="snowplow_navigation",
                executable="ground_truth_tf",
                name="ground_truth_tf",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "yaml_filename": map_file},
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                remappings=[("cmd_vel", "cmd_vel_nav")],
                **common,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                **common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                **common,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                **common,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                **common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")],
                **common,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                        "bond_timeout": 4.0,
                    }
                ],
            ),
        ]
    )
