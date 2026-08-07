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
  - 坡道默认地形由 `asphalt` 改为 `concrete`
  - `snow` 文案改为冰雪/积雪低附着路面
- `examples/terrain_multi.yaml`
  - 注释和示例布局同步为三地形
- `terrain_spawn.py`
  - spawn 坐标同步为 `concrete/sand/snow/slope`
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
  - `examples/robot_default.yaml` 默认 `contact_mu1=1.2`, `contact_mu2=0.25`, `contact_slip2=0.2`；
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
