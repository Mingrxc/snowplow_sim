# Codex 工作记忆

更新时间：2026-08-07
工作目录：`E:\zheng_snowplow_project\snowplow_sim`
主动模式：`reproduce/modify`

## 项目定位

本仓库服务于履带式智能扫雪车项目中的“小车仿真：不同地形仿真数据”板块。代码必须优先对齐项目计划书，而不是追求花哨功能。

## 当前主线

- 底层控制路线：`ros2_control + effort interface`
- 控制器包：`snowplow_control`
- 控制器类型：`snowplow_control/PMSMController`
- 输入话题：`/cmd_vel`
- 当前 `/odom`：Gazebo 真值里程计，不是编码器/IMU/RTK 融合里程计
- 远程仓库：`https://github.com/Mingrxc/snowplow_sim.git`

## 项目书对齐口径

默认 Gazebo 世界只按三类项目书场景组织：

- `concrete`：水泥/硬化路面
- `sand`：沙土/泥沙路面
- `snow`：冰雪/积雪路面

`asphalt`、`ice`、`grass` 仍可作为扩展预设保留在 `world_generator.py`，但不属于默认正式实验口径。

## 本轮已改

- `world_generator.py`
  - 默认分区由五地形旧布局改为三条项目书测试带
  - 阶梯坡道已从活动代码中彻底删除
  - `snow` 文案改为冰雪/积雪低附着路面
- `examples/terrain_multi.yaml`
  - 注释和示例布局同步为三地形
- `terrain_spawn.py`
  - spawn 坐标同步为 `concrete/sand/snow/dynamic_pad`
  - 去掉默认世界中不可靠的 `asphalt/ice/grass` 快捷入口
- `gazebo_speed_test.py`
  - 从单一速度均值扩展为速度误差、横向偏差、航向变化、路径长度等指标
  - 默认保存到 `results/gazebo_terrain/<time>_<label>/`
  - 每轮写 `summary.csv`、`odom_samples.csv`、`manifest.json`
- `terrain_experiment.py`
  - 明确为 Python 代理/灵敏度实验，不再伪装成 Gazebo 自动实验
  - 默认只跑 `concrete/sand/snow`
  - `--include-extended` 才跑扩展地形
  - 摩擦参数从 `world_generator.TERRAIN_PRESETS` 读取，减少重复漂移
- `snowplow_control`
  - 修复键盘控制无响应问题
  - Gazebo effort 接口现在由速度 PI + 前馈扭矩直接输出 wheel effort
  - 不再把 Gazebo joint effort state 反推为电流反馈，避免扭矩高频正负抵消
- `operator_console.py`
  - 默认输入超时从 0.30s 放宽到 1.00s
  - 面板增加 `PUBLISHED`，用于确认实际发布到 `/cmd_vel` 的速度
- `README.md`、`REPRODUCE_GUIDE.md`、`guidance/README.md`
  - 同步三地形口径、结果目录规则和 Gazebo/代理实验边界

## 仍需诚实标注的缺口

- 还没有真正的编码器/IMU/RTK 融合里程计节点。
- 还没有真实滑移检测 ROS 节点和 `<=200 ms` 响应测试。
- `snowplow_control` 有 PMSM 级联控制和前馈框架，但尚未实现完整“左右摩擦不对称漂移 -> 平衡环补偿”的正式验证链路。
- `terrain_experiment.py` 的 `load_factor` 是工程代理假设，不能写成 Gazebo 实测结论。

## 最小验证命令

Windows 语法检查：

```powershell
python -m py_compile urdf_generator.py launch_sim.py world_generator.py operator_console.py terrain_spawn.py gazebo_speed_test.py benchmark_runner.py control_prototype.py terrain_experiment.py
```

WSL 生成检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd /mnt/e/zheng_snowplow_project/snowplow_sim
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
python3 world_generator.py examples/terrain_multi.yaml -o output/worlds
xacro output/urdf/snowplow_robot.urdf.xacro > /tmp/snowplow_robot.urdf
```

WSL 控制器编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash
```

## 2026-08-07 履带转向问题记忆

- 现象：`w/s` 直走和倒退已经可用，但 `a/d` 几乎不转。
- 判断：键盘和 `/cmd_vel` 链路已基本排除；优先检查 Gazebo 履带轮接触模型。
- 原因：简化模型用圆柱轮近似履带。真实履带车需要横向擦滑才能差速转向；如果 `mu1/mu2` 都很高且接近，Gazebo 会让车像被地面锁住。
- 本轮修复：
  - `urdf_generator.py` 新增 `tracks.contact_*` 接触参数；
  - `examples/robot_default.yaml` 当前默认 `contact_mu1=20.0`, `contact_mu2=0.35`, `contact_slip2=0.35`；
  - `operator_console.py` 将 `a/d` 改成原地转向，默认 `wz≈±2.08 rad/s`；
  - `guidance/README.md` 写入重新生成 URDF、重启 Gazebo和调参步骤。
- 注意：接触参数只在重新生成 Xacro 并重新 spawn 机器人后生效。正式 Gazebo 地形实验需要基于同一套接触参数重新跑，不能和旧结果混合。

## 2026-08-07 四轮履带代理记忆

- 现象：`a/d` 成功转向后，视觉上像以前部为转轴。
- 判断：简化模型只有前部 `drive_wheel` 接收 effort，后部 `idler` 被动拖拽，驱动力分布偏前。
- 本轮修复：
  - `urdf_generator.py` 和 `snowplow_control/config/snowplow_control.yaml` 默认把四个轮关节都纳入控制器；
  - `pmsm_cascade_controller.cpp` 兼容 2/4 关节；
  - 四轮配置下每侧 effort 平均分给前后两个接触轮，避免动力翻倍。
- 注意：这改变 C++ 控制器接口，必须重新 `colcon build --packages-select snowplow_control`，再重新生成 URDF 并重启 Gazebo。

## 2026-08-07 动态摩擦地块记忆

- 用户明确不要多块固定摩擦地块，要同一块地运行中动态改 `mu/mu2`。
- 当前设计：保留 `concrete/sand/snow`，额外新增 `dynamic_pad`，默认在 `x=0, y=-18`。
- C++ 插件：`snowplow_control/src/dynamic_friction_plugin.cpp`。
- 插件库：`libsnowplow_dynamic_friction_plugin.so`。
- 终端 3：`dynamic_friction_console.py`。
- 控制话题：`/dynamic_friction/set`，状态话题：`/dynamic_friction/state`，消息类型 `std_msgs/Float64MultiArray`，数据 `[mu, mu2]`。
- 终端 2：`operator_console.py` 订阅 state，小车位于 `dynamic_pad` 时显示最新配置摩擦系数。
- 注意：这是工程扩展，不改变项目书默认三地形固定摩擦结论；动态地块实验要单独解释和保存。

## 2026-08-07 摩擦表现与起步稳定记忆

- 现象：地块/动态摩擦改到 20 后速度差异仍不明显；按 `w` 偶发极慢甚至倒走。
- 判断：
  - Gazebo 的 `mu/mu2` 表示最大抓地能力，不是越大越慢的线性阻力。
  - 旧履带 `contact_mu1/contact_mu2` 较低，会封顶地块高摩擦效果。
  - C++ 控制器旧负载观测前馈在 Gazebo effort 适配层可能产生反向扭矩。
- 本轮修复：
  - 履带纵向接触上限提高到 `contact_mu1=20.0`，横向保持 `contact_mu2=0.35`, `contact_slip2=0.35` 以兼顾转向；
  - 三地形摩擦拉开：水泥更高，沙地中等，雪地更低；
  - 控制器移除 Gazebo effort 适配层的负载观测前馈，改为参考速度方向前馈；
  - 默认 `max_torque=45.0`, `velocity_p=8.0`, `velocity_i=4.0`。
- 调动态地块时，高 `mu` 表示抓地强；要看速度下降，应测试低值如 `0.10 0.08`。

## 2026-08-07 默认坡道关闭记忆

- 用户认为默认世界里的阶梯坡道不必要。
- 用户进一步要求彻底删除坡道，而不是只关闭。
- 已删除 `world_generator.py` 中的坡道默认配置、`sdf_slope()` 和生成调用。
- 已删除 `examples/terrain_multi.yaml`、`examples/terrain_obstacles.yaml` 中的 `slope` 配置块。
- 已删除 `operator_console.py` 的 slope 识别逻辑和 `terrain_spawn.py` 的 slope 快捷入口。
- `terrain_spawn.py` 当前可直接 spawn 到 `dynamic_pad`。
- 修改后需要重新生成 `output/worlds/snowplow_multi_terrain.world` 并重启 Gazebo。

## 2026-08-07 外观、雪地转向和动态速度记忆

- 用户要求车身改为粉色、轮侧增加可见履带、传感器不要像悬浮在空中。
- `urdf_generator.py` 已改为生成粉色车体、浅粉上盖、黑色履带带面/履齿，以及传感器支架视觉件。
- 用户反馈雪地里无法旋转；本轮没有提高全局履带横向摩擦，避免再次破坏 `a/d`，而是把 `world_generator.py` 中雪地纵向抓地校准为 `mu=0.30`、横向保持低值 `mu2=0.16`。
- 终端 3 `dynamic_friction_console.py` 已从“只调摩擦”扩展为“摩擦 + 按键速度”调参台：
  - `0.30 0.15`：只改动态地块摩擦；
  - `0.30 0.15 0.70 1.80`：同时改摩擦和终端 2 按键速度；
  - `speed 0.80` 或 `speed 0.80 2.20`：只改终端 2 按键速度。
- 终端 2 `operator_console.py` 新增订阅 `/dynamic_drive/set`，收到 `[linear_speed, angular_speed]` 后动态更新 `w/s` 和 `a/d` 的速度步长。
- 该动态速度链路是人工调试/展示扩展，不属于项目书三地形正式测速实验语义。

## 2026-08-13 点击目标自动寻路记忆

- 用户要求增加类似 MOBA 游戏“点击地点后自动寻路”的功能，并授权自主完成。
- 项目书包含自主导航、智能避障、多场景导航、Gazebo 离线验证和 MPPI 控制器方向，因此导航演示与总体需求一致；具体软件编排属于工程实现，项目书未规定这些节点和参数。
- 新增 `snowplow_navigation` ROS 2 Python 包：NavFn 全局规划、MPPI 差速局部控制、速度平滑、行为树、激光代价地图及 RViz Nav2 Goal。
- 新增 `examples/terrain_navigation.yaml`，保留沙地/水泥/雪地并布置静态障碍物；它与正式测速的 `terrain_multi.yaml` 分离。
- 新增 `navigation_map_generator.py`，从同一场景 YAML 生成 Gazebo 障碍物对应的 Nav2 占据地图；输出进入被 Git 忽略的 `output/maps/`。
- 新增 `launch_navigation.py`，负责地图生成和 Nav2/RViz 启动；Gazebo 仍由 `launch_sim.py` 单独启动。
- 导航控制链最终仍发布 `/cmd_vel` 给现有 `snowplow_control` effort 控制器，没有建立第二套底盘控制。
- 当前定位由 `ground_truth_tf.py` 使用 Gazebo `/odom` 发布 `map -> odom -> base_link`，只代表仿真真值定位，不能写成实车 SLAM/定位已经完成。
- `operator_console.py` 和 Nav2 不能同时运行，两者都是 `/cmd_vel` 命令源。
- 终点使用 `nav2_controller::PositionGoalChecker`，位置容差 `0.20 m`，忽略终点 yaw，修复履带车到点后反复原地调头并超时的问题。
- 已实测目标接收、规划、激光避障、MPPI、速度平滑、履带控制和 Gazebo 运动链打通；跨障碍目标到达约 `0.18 m` 位置误差，近目标回归输出 `Goal succeeded`。
- 导航演示结果必须与正式三地形测速结果分开，不得混写。

## 2026-08-14 仓库边界整合记忆

- `E:\zheng_snowplow_project\snowplow_sim` 是唯一需要上传的实验项目本体和 Git 根目录。
- 原本放在仓库外的 `E:\zheng_snowplow_project\guidance` 已整体移入 `snowplow_sim/guidance/`，实验指南现在会随 Git 上传。
- 根 README 和指南内部引用已改成仓库相对路径，克隆到其他机器后不依赖固定盘符。
- 外层 `papers/` 是项目计划书和历史资料，`tools/`、`.solidworks_com/`、DLL 与探针文件是独立 SolidWorks 自动化环境，不属于当前仿真仓库运行依赖，未混入 `snowplow_sim`。
- `output/`、`results/`、`figures/`、`benchmark_output/`、`terrain_output/` 仍按生成物或结果目录处理，不把中间产物提交到 Git。

## 2026-08-14 guidance 双位置维护记忆

- 用户要求在外层工作区 `E:\zheng_snowplow_project\guidance/` 也保留一份实验指南，方便直接打开。
- 仓库正式版本位于 `E:\zheng_snowplow_project\snowplow_sim\guidance/`，该版本随 Git 上传，是版本管理的主版本。
- 外层 `E:\zheng_snowplow_project\guidance/` 是方便访问的同步副本，不属于 `snowplow_sim` Git 仓库。
- 今后修改 `guidance/README.md` 或 `guidance/TERMINAL_DISPLAY_FIELDS.md` 时，必须先更新仓库内主版本，再同步覆盖外层副本，保持两处内容一致。
- 不应只修改外层副本，否则 GitHub 中的指南不会得到更新。
