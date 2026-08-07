# Reproduce Guide

This repository now has one main simulation route:

- `ros2_control + effort interface`
- Gazebo provides the environment and ground-truth odometry
- `snowplow_control` provides the drive controller

## Use this order

1. Source ROS 2 Humble
2. Build `snowplow_control`
3. Generate `output/urdf` and `output/worlds`
4. Start Gazebo with `launch_sim.py --auto`
5. Run `operator_console.py`
6. Use `terrain_spawn.py` + `gazebo_speed_test.py` for terrain tests

## Clean rules

- Do not commit `output/`
- Do not commit `results/`
- Do not mix old benchmark figures with new runs
- Keep the control route unified

## Main files

- `urdf_generator.py`
- `world_generator.py`
- `launch_sim.py`
- `snowplow_control/`
- `operator_console.py`
- `gazebo_speed_test.py`

