# snowplow_sim

履带式智能扫雪车 Gazebo 仿真代码仓库。

本目录是源码仓库根目录，主要说明代码结构、模块职责和维护边界。具体启动和实验步骤放在：

```text
E:\zheng_snowplow_project\guidance\README.md
```

## 当前主线

底层控制主线固定为：

```text
ros2_control + effort interface
```

也就是说：

- Gazebo 负责物理环境、传感器和真值里程计；
- `snowplow_control` 负责左右驱动轮 effort 控制；
- 不再把 `libgazebo_ros_diff_drive.so` 作为主控路线；
- 默认世界对齐项目书三地形：水泥/硬化路面、沙土/泥沙路面、冰雪路面；
- 生成物和实验结果不作为源码提交。

## 代码结构

```text
snowplow_sim/
├── README.md
├── REPRODUCE_GUIDE.md
├── MODIFY_LOG.md
├── CODEX_MEMORY.md
├── requirements.txt
├── urdf_generator.py
├── world_generator.py
├── launch_sim.py
├── operator_console.py
├── terrain_spawn.py
├── gazebo_speed_test.py
├── control_prototype.py
├── terrain_experiment.py
├── benchmark_runner.py
├── cmd_vel_watchdog.py
├── examples/
├── docs/
├── snowplow_control/
├── meshes/
├── launch/
├── output/
├── results/
└── figures/
```

## 核心模块

### `urdf_generator.py`

根据 `examples/robot_*.yaml` 生成机器人模型。

输出内容：

- `output/urdf/*.urdf.xacro`
- `output/urdf/ros2_control.yaml`

关键职责：

- 生成车体、履带、扫雪铲和传感器连杆；
- 生成 `<ros2_control>` 硬件接口；
- 加载 `gazebo_ros2_control/GazeboSystem`；
- 配置左右驱动轮 effort command interface；
- 保留 `/odom` 真值里程计插件。

### `world_generator.py`

根据 `examples/terrain_*.yaml` 生成 Gazebo Classic world。

关键职责：

- 生成多地形区域；
- 默认生成水泥/硬化、沙土/泥沙、冰雪三条测试带；
- 保留沥青、冰面、草地作为扩展对照预设；
- 生成障碍物和坡道；
- 避免把 Gazebo factory 插件硬写进 world。

### `launch_sim.py`

仿真启动辅助脚本。

关键职责：

- 编译 Xacro 为临时 URDF；
- 启动 Gazebo；
- spawn 机器人；
- 等待 `/controller_manager`；
- 启动 `joint_state_broadcaster` 和 `pmsm_controller`。

启动命令不在本 README 展开，详见 `guidance/README.md`。

### `snowplow_control/`

ROS 2 C++ 控制器包。

核心文件：

- `src/pmsm_cascade_controller.cpp`
- `include/snowplow_control/pmsm_cascade_controller.hpp`
- `config/snowplow_control.yaml`
- `snowplow_control_plugin.xml`

控制器名称：

```text
snowplow_control/PMSMController
```

控制接口：

- command interface: `left_drive_wheel_joint/effort`
- command interface: `right_drive_wheel_joint/effort`
- state interface: position / velocity / effort

Gazebo 适配口径：

- `/cmd_vel` 转为左右轮角速度目标；
- 速度 PI 和前馈项直接输出 Gazebo wheel effort；
- `current_p/current_i` 保留为 PMSM 原型参数，不在 Gazebo effort 适配层反推电流反馈。

输入话题：

```text
/cmd_vel
```

### `operator_console.py`

终端交互控制台。

关键职责：

- 按住移动、松手停车；
- 发布 `/cmd_vel`；
- 订阅 `/odom`；
- 显示当前速度、位姿、地形信息和实际发布速度。

### `terrain_spawn.py`

地形切换辅助脚本。

关键职责：

- 删除旧机器人；
- 按地形名称重新 spawn；
- 用于多地形测速前的定位。

默认支持的项目书场景：

- `concrete`
- `sand`
- `snow`
- `slope`

### `gazebo_speed_test.py`

固定速度测试脚本。

关键职责：

- 发布固定 `/cmd_vel`；
- 采集 `/odom`；
- 计算速度误差、横向偏差、航向变化和路径长度；
- 默认输出 summary CSV、原始 odom CSV 和 manifest 到 `results/gazebo_terrain/<time>_<label>/`。

### `control_prototype.py`

纯 Python 控制原型。

用途：

- PMSM 控制算法快速验证；
- 不等同于 Gazebo 真实物理实验。

### `terrain_experiment.py`

纯 Python 地形代理实验。

注意：

- 使用代理负载参数；
- 默认只运行 `concrete`、`sand`、`snow` 三地形；
- `--include-extended` 才运行沥青、草地、冰面扩展对照；
- 不直接读取 Gazebo 接触状态；
- 不能作为最终 Gazebo 实验结论。

### `benchmark_runner.py`

控制参数扫描脚本。

注意：

- 自定义 score 属于工程辅助指标；
- 不应直接写成项目书或论文的正式评价指标。

## 配置目录

### `examples/`

示例配置：

- `robot_default.yaml`
- `robot_small.yaml`
- `terrain_multi.yaml`
- `terrain_obstacles.yaml`

### `docs/`

调研、接口和历史设计说明。

这些文档用于理解背景，不一定代表当前代码已经完全实现所有目标。

## 生成目录

以下目录不应提交到 Git：

- `output/`
- `results/`
- `figures/`
- `benchmark_output/`
- `terrain_output/`（legacy 历史输出）
- `__pycache__/`

这些目录由脚本运行时生成或保存实验输出。

## 维护规则

1. 控制主线保持 `ros2_control + effort interface`。
2. 启动和实验步骤只维护在 `guidance/README.md`。
3. 本 README 只写代码结构、模块职责和维护边界。
4. 新增实验结果必须进入 `results/`，不要和源码混放。
5. Python 代理实验和 Gazebo 实测结果必须分开描述。
6. 删除文件前先确认它是生成物、缓存、旧报告脚本或已被替代的路径。
7. 每轮优化后至少运行语法检查；涉及控制器时还要编译 `snowplow_control`。
8. 默认实验口径只使用 `concrete`、`sand`、`snow`；扩展地形必须显式标注。
