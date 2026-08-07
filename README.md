# snowplow_sim

履带式扫雪车 Gazebo 仿真项目。当前唯一主线是：

```text
ros2_control + effort interface
```

## 目录结构

- `urdf_generator.py`：生成机器人 Xacro 和 `ros2_control.yaml`
- `world_generator.py`：生成 Gazebo 世界文件
- `launch_sim.py`：启动 Gazebo、spawn 机器人、启动控制器
- `snowplow_control/`：C++ `ros2_control` 控制器包
- `operator_console.py`：交互式操作台
- `terrain_spawn.py`：把机器人放到指定地形
- `gazebo_speed_test.py`：固定时长测速
- `terrain_experiment.py`：Python 地形对比实验
- `benchmark_runner.py`：控制参数扫描

## 重要约定

- 不再把 `diff_drive` 作为主控路线
- `/odom` 来自 Gazebo 真值里程计插件
- 左右驱动轮由 `snowplow_control` 通过 effort 接口控制
- `output/`、`results/`、旧图表和缓存不提交到 Git

## 第一次使用

先编译控制器：

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
ln -sfn /mnt/e/zheng_snowplow_project/snowplow_sim/snowplow_control ~/ros2_ws/src/snowplow_control

cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash
ros2 pkg prefix snowplow_control
```

最后一行应输出 `~/ros2_ws/install/snowplow_control`。

## 快速启动

每个新终端都先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

生成模型和世界：

```bash
cd /mnt/e/zheng_snowplow_project/snowplow_sim
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
python3 world_generator.py examples/terrain_multi.yaml -o output/worlds
```

启动仿真：

```bash
python3 launch_sim.py \
  --xacro output/urdf/snowplow_robot.urdf.xacro \
  --world output/worlds/snowplow_multi_terrain.world \
  --auto
```

第二个终端运行操作台：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd /mnt/e/zheng_snowplow_project/snowplow_sim
python3 operator_console.py --terrain-config examples/terrain_multi.yaml
```

## 常见启动错误

如果看到：

```text
snowplow_control is not built/sourced
```

说明当前终端没有加载 `~/ros2_ws/install/setup.bash`，执行：

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix snowplow_control
```

如果看到：

```text
unrecognized arguments: --autopython3 launch_sim.py
```

说明命令粘贴到一起了。重新复制“快速启动”里的启动命令，单独执行一次。

## 优化代码前的原则

- 保持 `ros2_control + effort` 主线不变
- 优先清理冗余和重复逻辑
- 每次优化都跑 Python 语法检查、Xacro 生成检查和 C++ 编译检查
- 实验输出统一放 `results/`，不要和源码混放

