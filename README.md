# snowplow_sim

履带式智能扫雪车 Gazebo 仿真代码仓库。

本目录是完整实验项目和 Git 仓库根目录，主要说明代码结构、模块职责和维护边界。具体启动和实验步骤放在：

```text
guidance/README.md
```

## 当前主线

底层控制主线固定为：

```text
ros2_control + effort interface
```

也就是说：

- Gazebo 负责物理环境、传感器和真值里程计；
- `snowplow_control` 负责左右履带代理轮 effort 控制；
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
├── dynamic_friction_console.py
├── navigation_map_generator.py
├── launch_navigation.py
├── terrain_spawn.py
├── gazebo_speed_test.py
├── control_prototype.py
├── terrain_experiment.py
├── benchmark_runner.py
├── cmd_vel_watchdog.py
├── examples/
├── guidance/
├── docs/
├── snowplow_control/
├── snowplow_navigation/
├── meshes/
├── output/                 # 运行时生成，不提交
└── results/                # 实验结果，不提交
```

## 核心模块

### `urdf_generator.py`

根据 `examples/robot_*.yaml` 生成机器人模型。

输出内容：

- `output/urdf/*.urdf.xacro`
- `output/urdf/ros2_control.yaml`

关键职责：

- 生成车体、履带、扫雪铲和传感器连杆；
- 当前默认外观为粉色车身、黑色可视化履带带面、传感器支架，便于 Gazebo 中观察；
- 生成 `<ros2_control>` 硬件接口；
- 加载 `gazebo_ros2_control/GazeboSystem`；
- 配置左右前后履带代理轮 effort command interface；
- 配置履带轮 Gazebo 接触参数，默认采用纵向抓地、横向可滑的近似模型；
- 保留 `/odom` 真值里程计插件。

### `world_generator.py`

根据 `examples/terrain_*.yaml` 生成 Gazebo Classic world。

关键职责：

- 生成多地形区域；
- 默认生成水泥/硬化、沙土/泥沙、冰雪三条测试带；
- 可额外生成 `dynamic_pad` 动态摩擦扩展实验地块；
- 保留沥青、冰面、草地作为扩展对照预设；
- 生成障碍物；
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
- `src/dynamic_friction_plugin.cpp`
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
- command interface: `left_idler_wheel_joint/effort`
- command interface: `right_idler_wheel_joint/effort`
- state interface: position / velocity / effort

Gazebo 适配口径：

- `/cmd_vel` 转为左右轮角速度目标；
- 速度 PI 和前馈项计算每侧履带代理总 effort；
- 默认四轮代理配置下，每侧 effort 平均分配给前后两个接触轮，避免只有前部驱动轮“拧车”；
- `current_p/current_i` 保留为 PMSM 原型参数，不在 Gazebo effort 适配层反推电流反馈。

输入话题：

```text
/cmd_vel
```

动态摩擦插件：

- 插件库：`libsnowplow_dynamic_friction_plugin.so`
- 控制话题：`/dynamic_friction/set`
- 状态话题：`/dynamic_friction/state`
- 消息类型：`std_msgs/Float64MultiArray`
- 数据格式：`[mu, mu2]`

该插件只作用于 `dynamic_friction_pad` 模型，不改变默认三地形本身的固定摩擦参数。

### `operator_console.py`

终端交互控制台。

关键职责：

- 按键刷新运动命令，超时自动停车；
- 发布 `/cmd_vel`；
- 订阅 `/odom`；
- 订阅 `/dynamic_drive/set`，允许运行时调整 `w/s` 线速度和 `a/d` 角速度；
- 显示当前速度、位姿、地形信息和实际发布速度。

默认遥控手感：

- `w/i` 前进，`s/k` 后退；
- `a/d` 为原地左/右转向，用于快速换方向；
- `j/l` 保留为原地左/右旋转备用键；
- 默认线速度 `0.50 m/s`，默认角速度 `1.30 rad/s`。

注意：当前模型用圆柱轮、四轮履带代理 effort 和接触参数近似履带。真实履带车通过左右履带差速和地面横向滑移转向，因此 `examples/robot_default.yaml` 中的 `tracks.contact_mu2` 和 `tracks.contact_slip2` 会明显影响 `a/d` 转向手感。当前履带自身 `contact_mu1/contact_mu2` 设得较高，目的是让地块 `mu/mu2` 更直接地限制抓地能力。该参数属于 Gazebo 工程近似，项目书未指定具体数值。

### `dynamic_friction_console.py`

动态调参控制台。

关键职责：

- 作为终端 3 运行；
- 输入新的 `mu mu2`，更新动态摩擦地块；
- 输入 `speed vx [wz]` 或 `mu mu2 vx wz`，更新终端 2 的按键速度；
- 发布到 `/dynamic_friction/set`；
- 发布到 `/dynamic_drive/set`；
- 订阅 `/dynamic_friction/state`，显示 Gazebo 插件当前生效配置。

该脚本只用于扩展实验地块 `dynamic_pad` 和人工调试手感，不参与默认三地形测速结论。

### `navigation_map_generator.py`

根据导航场景 YAML 中的世界尺寸和静态障碍物生成 Nav2 占据栅格地图。

关键边界：

- Gazebo world 和 Nav2 map 共用 `examples/terrain_navigation.yaml`，避免手工维护两套障碍物位置；
- 输出为 `output/maps/*.pgm` 和 `output/maps/*.yaml`，属于可再生成文件，不提交到 Git；
- 地形摩擦不会写入占据栅格。Nav2 只把地图用于通行与避障，摩擦差异仍由 Gazebo 物理引擎处理。

### `launch_navigation.py`

导航演示入口。运行时生成地图并启动 `snowplow_navigation` 的 Nav2 节点，可选启动 RViz。

它不负责启动 Gazebo；完整启动步骤见 `guidance/README.md`。

### `snowplow_navigation/`

ROS 2 Python 导航包，提供类似“点击地点后自动寻路”的工程演示：

- `launch/navigation.launch.py`：编排 NavFn、MPPI、代价地图、行为树和速度平滑器；
- `config/nav2_params.yaml`：履带式差速模型、车体 footprint、速度和避障参数；
- `ground_truth_tf.py`：根据 Gazebo `/odom` 发布 `map -> odom -> base_link`；
- `navigation_goal.py`：从命令行发送并监控一个 `NavigateToPose` 目标；
- `rviz/navigation.rviz`：地图、激光、代价地图、路径及 Nav2 Goal 工具配置。

数据链为：

```text
RViz Nav2 Goal / navigation_goal
  -> NavFn global planner
  -> MPPI local controller
  -> velocity_smoother
  -> /cmd_vel
  -> snowplow_control
  -> Gazebo
```

当前终点检查只要求位置误差不超过 `0.20 m`，不强迫履带车在终点原地对齐点击箭头朝向，避免到点后持续摆动。

重要限制：

- 当前定位使用 Gazebo 真值 `/odom`，只用于离线仿真验证，不代表实车定位方案；
- `operator_console.py` 与 Nav2 都会发布 `/cmd_vel`，不得同时运行；
- 导航演示是项目书“自主导航、智能避障、Gazebo 离线验证”的工程实现，不改变默认三地形测速实验语义；
- 导航演示数据与 `gazebo_speed_test.py` 的正式地形测速结果必须分开保存和解释。

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

- `dynamic_pad`，扩展动态摩擦测试地块

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
- `terrain_navigation.yaml`（导航与避障演示专用）

### `docs/`

调研、接口和历史设计说明。

这些文档用于理解背景，不一定代表当前代码已经完全实现所有目标。

### `guidance/`

随仓库维护的实验操作文档：

- `README.md`：从环境加载、编译、生成模型到键盘控制、动态摩擦和 Nav2 导航的完整步骤；
- `TERMINAL_DISPLAY_FIELDS.md`：操作终端中速度、摩擦、接触状态等字段的含义。

该目录属于项目交付内容，应与源码一同提交。文档使用仓库相对路径，克隆后无需依赖外层 `E:\zheng_snowplow_project` 目录。

本机外层 `E:\zheng_snowplow_project\guidance/` 另保留一份方便访问的同步副本。仓库内本目录是 Git 主版本；修改指南后必须将其同步到外层副本。

### `meshes/`

SolidWorks 真实模型导出后的 Gazebo 网格入口。当前先提交 `meshes/README.md` 说明交付结构；后续视觉网格放入 `meshes/visual/`，简化碰撞网格放入 `meshes/collision/`。

SolidWorks 原始装配体、COM 桥接脚本和本机 DLL 不属于仿真运行依赖，不随本仓库上传。具体接口规范见 `docs/SOLIDWORKS_MODEL_INTERFACE.md`。

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
9. 手动键盘控制与 Nav2 导航互斥，任何时刻只允许一个 `/cmd_vel` 命令源。
10. 真值定位导航演示不得表述为已经完成实车定位或 SLAM。

## 仓库边界

GitHub 上传范围就是本 `snowplow_sim/` 目录。仓库应包含：

- Python 仿真、控制、实验和导航入口；
- `snowplow_control/` 与 `snowplow_navigation/` ROS 2 包；
- `examples/` 配置、`docs/` 设计文档、`guidance/` 实验步骤；
- `meshes/` 的接口说明及后续导出的 Gazebo 网格；
- README、许可证、修改记录和项目记忆。

仓库不包含：

- 外层项目计划书、其他研究资料和其他项目代码；
- 本机 SolidWorks COM 桥接环境、DLL、探针输出；
- `output/` 可再生成文件、Python 缓存、ROS 构建目录和运行日志；
- `results/` 正式实验输出，除非以后明确选择发布某一组冻结结果。
