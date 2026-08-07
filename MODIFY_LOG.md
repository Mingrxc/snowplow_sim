# reproduce/modify 修改日志

> 状态说明（2026-08-07）：本文记录的是逐轮修改历史，其中较早条目可能反映旧实现口径。当前默认实验口径已收束为项目书三地形 `concrete/sand/snow`，`/odom` 以 Gazebo 真值里程计为准，Python 代理实验仅作为辅助分析。

主动模式：`reproduce/modify`。

本轮目标是整理项目、清理明显中间文件、补齐复现路径，并做不改变论文方法语义的复现友好优化。

## 1. 清理结果

已删除：

| 路径 | 原因 |
| --- | --- |
| `__pycache__/` | Python 字节码缓存，可由 Python 自动再生成 |
| `gen_guide.py.bak` | 备份文件，已有正式 `gen_guide.py` |
| `gen_guide.py.bak2` | 重复备份文件 |
| `output2/` | 重复生成目录，主生成目录为 `output/` |

已保留：

| 路径 | 原因 |
| --- | --- |
| `output/` | 默认 URDF/World 示例生成产物 |
| `figures/` | 控制算法图表 |
| `terrain_output/` | 地形实验旧结果（历史保留，不作为新结果目录） |
| `benchmark_output/` | benchmark 旧结果 |
| `papers/` 和根目录 `.docx` | 论文、报告、指南、调研材料 |

## 2. 论文-代码对齐矩阵

| 论文/报告依据 | 代码位置 | 当前结论 |
| --- | --- | --- |
| `扫雪机器人论文_初稿.docx` 第 2 章，段落 52-67：URDF、Link/Joint、驱动与传动参数 | `urdf_generator.py` | 基本对齐。代码可由 YAML 生成车体、履带、扫雪铲、传感器和 Gazebo 插件 |
| `扫雪机器人论文_初稿.docx` 第 3 章，段落 68-83：Gazebo 多地形、摩擦、接触刚度/阻尼 | `world_generator.py` | 基本对齐。默认场景已切到 concrete/sand/snow，保留 asphalt/ice/grass 作为扩展预设 |
| `扫雪机器人论文_初稿.docx` 第 3.3 节，段落 84-91：LiDAR、IMU、里程计仿真 | `urdf_generator.py` | 部分对齐。LiDAR/IMU 已实现；当前 `/odom` 按 Gazebo 真值里程计口径记录 |
| `扫雪机器人论文_初稿.docx` 第 4.2-4.3 节，段落 98-106：TAVCC 地形自适应速度修正节点 | 当前缺少对应 ROS 节点 | 未完整对齐。不能声称已复现第 4 章 TAVCC |
| `扫雪机器人论文_初稿.docx` 第 5 章，段落 107-124：PMSM、速度 PI、电流 PI、三通道前馈、ros_control | `control_prototype.py`, `snowplow_control/` | Python 原型基本对齐；C++ 控制器包待 Ubuntu/ROS2 实机环境编译审查 |
| `调研现状.docx` 段落 6-7：仿真与真机话题名称一致 | `urdf_generator.py`, `docs/传感器接入调研报告.md` | 基本对齐，核心话题为 `/cmd_vel`, `/odom`, `/scan`, `/imu/data` |

## 3. 审查问题与选择

| 问题 | 级别 | 选择 | 处理 |
| --- | --- | --- | --- |
| `urdf_generator.py` 缺少实际生成调用和脚本入口，命令退出 0 但不生成文件 | A：复现阻断修复 | 补上 `generate_urdf()`、`generate_control_config()` 和 `main()` 入口 | 已改 |
| `launch_sim.py` 自动模式会把空字符串作为 Gazebo 参数传入 | B：保守修复 | 修复参数列表构造 | 已改 |
| `world_generator.py` 把 `libgazebo_ros_factory.so` 写入 `.world`，Gazebo 报 incorrect plugin type | A：Gazebo 启动阻断修复 | 从 world 中移除 factory system plugin，改为通过 `ros2 launch gazebo_ros gazebo.launch.py` 或 `gazebo -s` 加载 | 已改 |
| `world_generator.py` 把 gravity 放在 physics 参数里触发 bad any_cast 警告 | B：兼容性修复 | 将 `<gravity>` 移到 `<world>` 下 | 已改 |
| `launch_sim.py` entity 名从 `snowplow_robot.urdf.xacro` 变成 `snowplow_robot.urdf` | B：保守修复 | 明确剥离 `.urdf.xacro` 后缀 | 已改 |
| 生成器空 YAML 报错不友好，默认配置嵌套字典可能被重复调用污染 | B：保守修复 | `deepcopy` + 顶层类型检查 | 已改 |
| `control_prototype.py` 图表把角速度转线速度写成除以半径 | A：结果显示修复 | 改为乘以半径 | 已改 |
| `terrain_experiment.py` 图表中文标签在默认 matplotlib 字体中缺字 | B：显示修复 | 图表使用英文地形名，CSV/终端仍保留中文标签 | 已改 |
| 实验脚本默认输出到固定目录，容易覆盖旧结果 | B：保守增强 | 增加 `--output-dir`，默认兼容旧目录 | 已改 |
| README 原环境为 Ubuntu22/Humble，用户实际为 Ubuntu20 | B：文档澄清 | 复现指南区分 Foxy/Humble | 已写 |
| 第 4 章 TAVCC 节点缺失 | 语义缺口 | 不擅自实现 | 标为未完整复现 |

## 4. 修改文件

| 文件 | 修改原因 |
| --- | --- |
| `urdf_generator.py` | 配置合并稳定性修复 |
| `world_generator.py` | 配置合并稳定性修复 |
| `launch_sim.py` | 启动参数、依赖检查、entity 命名、临时 URDF 编码修复 |
| `control_prototype.py` | 图表单位换算修复，增加 `--output-dir` |
| `terrain_experiment.py` | 增加 `--output-dir` 和 `--no-save` |
| `benchmark_runner.py` | 增加 `--output-dir`，修正文档中的实验组数描述 |
| `cmd_vel_watchdog.py` | 新增安全速度桥接；输入超时后自动发布零速度 |
| `terrain_spawn.py` | 新增按地形名称删除并重新 spawn 的工具，坐标与默认 world 布局一致 |
| `gazebo_speed_test.py` | 新增固定时长直行测速；采集 `/odom` 并追加 CSV 汇总 |
| `requirements.txt` | 明确 Python 依赖 |
| `REPRODUCE_GUIDE.md` | 傻瓜式复现流程 |
| `MODIFY_LOG.md` | 本日志 |

## 5. 性能与内存影响

- 启动脚本修复不改变仿真语义，只减少启动失败概率。
- 图表修复不改变实验计算，只让 PNG 图中的速度单位正确。
- `--output-dir` 不改变默认结果；复现时推荐写入 `results/<run_id>/...`。
- 生成器 `deepcopy` 的额外耗时可忽略，通常小于 0.1 秒。
- `cmd_vel_watchdog.py` 和 `terrain_spawn.py` 是论文未指定的工程辅助工具，不改变地形参数、速度指标或控制算法。
- `gazebo_speed_test.py` 同样是论文未指定的测量辅助工具；它固定实验时长和输入速度，便于获得真实 Gazebo 数据，不改变仿真物理或指标定义。
- 由于 Gazebo diff-drive 插件可能保持最后一次非零 `/cmd_vel`，指南已改为有限时长发布、明确零速停车和安全键盘桥接。

## 6. 结果与缓存规则

当前项目没有传统模型 cache。建议把结果分成：

| 类型 | 推荐路径 |
| --- | --- |
| 自动生成模型/场景 | `output/urdf`, `output/worlds` |
| 一次性实验结果 | `results/<run_id>/control`, `results/<run_id>/terrain`, `results/<run_id>/benchmark` |
| 历史示例图表 | `figures`, `terrain_output`, `benchmark_output` |

不要把调试日志、临时 bag 文件、不同日期的 CSV 全堆在同一个目录。

## 7. 剩余风险

- Ubuntu20/ROS2 环境未在本机真实运行，Gazebo 和 `snowplow_control` 编译状态需要你那边审查。
- `snowplow_control` 可能需要 Foxy/Humble API 兼容调整。
- 第 4 章 TAVCC 没有实现入口，属于论文复现缺口。
- 地形参数缺少实测标定，属于论文和工程共同的开放问题。

## 8. 本地验证

已在 Windows + Python 3.13 环境完成无 ROS 验证：

```bash
python -B urdf_generator.py examples/robot_default.yaml -o results/smoke_urdf
python -B world_generator.py examples/terrain_multi.yaml -o results/smoke_worlds
python -B launch_sim.py --help
python -B control_prototype.py --exp all --save --output-dir results/smoke_control
python -B terrain_experiment.py --output-dir results/smoke_terrain
python -B benchmark_runner.py --quick --output-dir results/smoke_benchmark
```

验证产物已在检查后删除，避免项目目录残留 smoke 中间文件。

## 9. 本轮视觉整改

本轮目标是让铲雪车模型和地形看起来更完整，但不改控制语义和地形物理参数。

### 9.1 已完成

- `urdf_generator.py`：为车体、履带轮组和扫雪铲增加多层视觉细节，默认开启 `visual_detail`
- `world_generator.py`：为多地形世界增加视觉纹理分块、边界条、雪堆/沙纹/冰裂/草丛/路面标线
- `examples/robot_default.yaml`、`examples/robot_small.yaml`、`examples/terrain_multi.yaml`、`examples/terrain_obstacles.yaml`：补充 `visual_detail` 示例配置
- `README.md`、`REPRODUCE_GUIDE.md`、`guidance/README.md`：同步说明默认视觉增强和临时编译文件的处理方式
- `docs/MODEL_TERRAIN_REDESIGN_PLAN.md`：新增整改计划，作为后续工作记录

### 9.2 清理结果

- 删除了 `output/urdf/snowplow_robot.urdf`
- 删除了语法检查产生的 `__pycache__/`
- 删除了空目录 `templates/`

### 9.3 重新生成结果

- 重新生成 `output/urdf/snowplow_robot.urdf.xacro`
- 重新生成 `output/urdf/snowplow_robot_small.urdf.xacro`
- 重新生成 `output/worlds/snowplow_multi_terrain.world`
- 重新生成 `output/worlds/snowplow_snow_obstacles.world`

### 9.4 影响判断

- collision 和关节命名未改
- `/cmd_vel`、`/odom`、`/scan` 等话题语义未改
- 视觉层更丰富，但 Gazebo 仿真行为原则上不应变化

## 10. 本轮交互控制与实时观测整改

本轮目标是解决键盘控制不直观、`/odom` 无限刷屏、左右转残留指令和终端数量过多的问题。该整改属于论文未指定的工程辅助，不作为固定地形测速实验的数据来源。

### 10.1 已完成

- 新增 `operator_console.py`
  - 直接发布 `/cmd_vel`
  - 订阅 `/odom`
  - 读取 world YAML，对照当前位姿显示地形配置参数
  - 支持按住才走、松手超时停车
  - 支持空格立即停车、`q` 退出并停车
  - 内置线速度和角速度限幅
- 更新 `cmd_vel_watchdog.py`
  - 保留为 `teleop_twist_keyboard` 旧链路的兼容桥接
  - 新增 `linear.x` 和 `angular.z` 限幅
  - 丢弃差速车不使用的 Twist 分量
- 更新 `launch_sim.py`
  - 手动提示改为两终端路线：仿真启动 + 交互控制台
  - 默认把展开后的 URDF 写入 `/tmp/robot.urdf`，便于 `terrain_spawn.py` 直接复用
- 更新 `README.md`、`REPRODUCE_GUIDE.md`、`guidance/README.md`
  - 默认推荐 `operator_console.py`
  - 旧键盘链路降级为兼容方案
  - 不再建议用无限 `ros2 topic echo` 观察速度

### 10.2 影响判断

- 未修改地形摩擦系数、接触刚度、接触阻尼或 World 物理参数。
- 未修改 `left_drive_wheel_joint`、`right_drive_wheel_joint`、`base_link`。
- 未修改 `/cmd_vel` 和 `/odom` 的话题语义。
- 人工键盘控制只用于调试和展示，正式多地形速度对比仍使用 `gazebo_speed_test.py`。
- 实时面板显示的是当前地形的配置参数，不是真实接触求解后的有效摩擦力。

### 10.3 预计资源

| 阶段 | 时间估计 | 峰值内存估计 |
| --- | ---: | ---: |
| `operator_console.py` 启动 | 1-3 秒 | 50-150 MB |
| Gazebo + Spawn | 10-60 秒 | 1-3 GB |
| 单地形固定测速 | 10-30 秒 | 取决于 Gazebo |

### 10.4 Ubuntu 验证重点

- `operator_console.py` 启动后能看到 `/odom` 从 `WAITING` 变成 `OK`。
- 按住 `i/w` 前进，松手约 0.3 秒内停车。
- 按住 `a/左键` 平滑左转，轨迹应更像前进弧线。
- 按住 `d/右键` 平滑右转，轨迹应更像前进弧线。
- 按住 `j` 原地左转，按住 `l` 原地右转。
- 空格能立即发布零速度。
- 当前地形显示能随 `terrain_spawn.py concrete/sand/snow/slope` 改变。

## 11. 键盘控制无响应修复

问题现象：运行 `operator_console.py` 后按任意方向键，小车不产生可见运动。

根因判断：

- Gazebo `effort` command interface 接收的是轮端扭矩，不是电机电流。
- 旧 C++ 控制器把 Gazebo joint `effort` state 反推成电流反馈，再进入 current PI；在仿真中容易产生正负扭矩高频抵消，表现为 `/cmd_vel` 有命令但车不动。
- `operator_console.py` 默认 `input_timeout=0.30s` 太短，在 WSL/VSCode 终端中按键重复延迟可能超过这个窗口，导致刚按下就回零。

已处理：

- `snowplow_control/src/pmsm_cascade_controller.cpp`
  - Gazebo 适配层改为速度 PI + 前馈扭矩直接输出 effort。
  - 保留 PMSM 前馈负载估计框架。
  - 不再把 Gazebo effort state 当作电流反馈。
- `snowplow_control/include/snowplow_control/pmsm_cascade_controller.hpp`
  - 同步控制函数签名。
- `urdf_generator.py`、`examples/robot_default.yaml`、`snowplow_control/config/snowplow_control.yaml`
  - 将 Gazebo effort 适配下的速度 PI 参数调到扭矩量级：`velocity_p=6.0`, `velocity_i=3.0`。
- `operator_console.py`
  - 默认输入超时改为 `1.00s`。
  - 面板新增 `PUBLISHED` 行，确认实际发布到 `/cmd_vel` 的命令。
- `guidance/README.md`
  - 新增“按键后小车完全不动”的排查步骤。

影响判断：

- 不改变地形参数、世界布局、测速指标或 `/cmd_vel` 话题语义。
- Gazebo 中的 C++ 控制器需要重新 `colcon build --packages-select snowplow_control`。
- `output/urdf/ros2_control.yaml` 需要重新由 `urdf_generator.py` 生成。

## 12. 手动遥控速度与转向手感优化

问题现象：

- 直走、倒退速度偏慢。
- 只按 `a` 左转时不够顺畅。

已处理：

- `operator_console.py`
  - 默认 `linear_speed` 从 `0.20 m/s` 提高到 `0.35 m/s`。
  - 默认 `max_linear_speed` 从 `0.30 m/s` 提高到 `0.60 m/s`。
  - 默认 `angular_speed` 从 `0.50 rad/s` 提高到 `1.00 rad/s`。
  - 默认 `max_angular_speed` 从 `1.00 rad/s` 提高到 `1.80 rad/s`。
  - `a/d` 改为带一点前进速度的平滑弧线转向。
  - `j/l` 保留为原地左/右旋转。
- `guidance/README.md`
  - 更新按键说明和临时调参命令。
- `README.md`
  - 更新 `operator_console.py` 模块说明。

影响判断：

- 只影响手动遥控体验。
- 不改变 `gazebo_speed_test.py` 的正式固定速度实验。
- 不改变地形参数、结果目录或项目书三地形口径。

## 13. 手动遥控二次提速与弧线转向修正

问题现象：

- `w/s` 速度仍偏慢。
- `a/d` 虽然发布了速度，但线速度分量过小，换算到左右履带后接近一侧反转、一侧正转，Gazebo 中仍像原地硬转，容易卡住。

已处理：

- `operator_console.py`
  - 默认 `linear_speed` 从 `0.35 m/s` 提高到 `0.50 m/s`。
  - 默认 `max_linear_speed` 从 `0.60 m/s` 提高到 `0.80 m/s`。
  - 默认 `angular_speed` 从 `1.00 rad/s` 提高到 `1.10 rad/s`。
  - 默认 `max_angular_speed` 从 `1.80 rad/s` 提高到 `2.00 rad/s`。
  - `a/d` 弧线转向改为 `0.85 * linear_speed` 和 `0.75 * angular_speed`，使左右履带都保持向前。
- `guidance/README.md`
  - 更新默认遥控参数和激进调试命令。
- `README.md`
  - 更新默认遥控手感说明。

影响判断：

- 只影响手动操作台默认 `/cmd_vel` 输出。
- 不影响正式固定速度地形测试。

## 14. `a/d` 强弧线转向修正

问题现象：

- `w/s` 直行体验已经良好。
- `a/d` 按住 5 秒约只改变 10 度左右，不能有效换方向。

已处理：

- `operator_console.py`
  - `a/d` 从温和弧线转向改为强弧线转向。
  - 默认 `a/d` 输出约为 `vx=0.275 m/s`, `wz=±2.200 rad/s`。
  - 保持 `w/s` 默认直行速度不变。
- `guidance/README.md`
  - 更新 `a/d` 预期输出和调试说明。
- `README.md`
  - 更新 `operator_console.py` 键位语义。

影响判断：

- 只影响手动遥控换方向能力。
- 不影响 `gazebo_speed_test.py` 正式地形测速。
- 如果仍然转向不足，下一层需要审查履带横向摩擦或控制器转向扭矩增益。

## 15. 履带横向滑移与原地转向修复

问题现象：

- `w/s` 直走、倒退速度已经可接受。
- 按住 `a/d` 时 `/cmd_vel` 已有角速度输出，但车头几乎不转，约 5 秒只变化十度左右。

根因判断：

- 这不再是键盘输入链路问题。
- 履带车转向依赖左右履带差速和地面横向滑移；当前模型用圆柱轮近似履带，如果轮/地接触使用 `mu1=1.0, mu2=1.0` 的各向同性高摩擦，Gazebo 会把横向擦地转向“锁住”。
- 项目书未指定履带接触求解的具体 `mu1/mu2/slip` 数值，因此本轮属于 Gazebo 工程近似修复，不写成项目书方法。

已处理：

- `urdf_generator.py`
  - 在 `tracks` 配置中新增履带轮接触参数：`contact_mu1`, `contact_mu2`, `contact_slip1`, `contact_slip2`, `contact_kp`, `contact_kd`, `contact_min_depth`, `contact_max_vel`。
  - 生成 Xacro 时默认写入 `mu1=1.2`, `mu2=0.25`, `slip2=0.2`，让纵向保持抓地、横向允许擦滑。
- `examples/robot_default.yaml`
  - 写出上述默认接触参数，便于后续手动调参。
- `operator_console.py`
  - `a/d` 从强弧线转向改成原地转向，默认输出约 `vx=0.000 m/s`, `wz=±2.080 rad/s`。
  - 默认 `angular_speed` 调整为 `1.30 rad/s`，`max_angular_speed` 调整为 `2.60 rad/s`。
- `README.md`
  - 说明履带转向依赖接触滑移，`contact_mu2/contact_slip2` 属于工程近似参数。
- `guidance/README.md`
  - 新增 `w/s` 能走但 `a/d` 几乎不转的排查与调参步骤。

影响判断：

- 会改变 Gazebo 中机器人与地面接触行为，因此需要重新生成 `output/urdf/snowplow_robot.urdf.xacro` 并重启 Gazebo。
- 不改变 `/cmd_vel`、`/odom` 话题语义。
- 不改变 `gazebo_speed_test.py` 的指标定义、结果目录或三地形口径。
- 正式地形实验结果需要在本改动后重新跑，不能和旧接触参数下的结果混写。

## 16. 四轮履带代理 effort 修复

问题现象：

- `a/d` 已经可以转向，但视觉上更像以前部驱动轮为转轴，而不是绕车体中部附近偏航。

根因判断：

- 当前简化模型只有 `left_drive_wheel_joint` 和 `right_drive_wheel_joint` 接收 effort。
- 后部 `idler` 轮虽然接触地面，但不主动代表同侧履带运动，导致驱动力集中在前部接触轮。
- 项目书未指定真实履带链节/接触求解方式，因此本轮仍属于 Gazebo 工程近似修复。

已处理：

- `urdf_generator.py`
  - `<ros2_control>` 默认纳入 `left_idler_wheel_joint` 和 `right_idler_wheel_joint`。
  - 生成的 `ros2_control.yaml` 默认包含四个履带代理轮关节。
- `snowplow_control/src/pmsm_cascade_controller.cpp`
  - 控制器兼容 2 轮旧配置和 4 轮履带代理配置。
  - 四轮代理配置下，每侧计算出的总 effort 平均分给前后两个接触轮，避免总扭矩翻倍。
- `snowplow_control/config/snowplow_control.yaml`
  - 同步四轮默认关节列表。
- `README.md`, `guidance/README.md`, `CODEX_MEMORY.md`
  - 同步说明四轮履带代理边界和重新编译/重新生成要求。

影响判断：

- C++ 控制器接口数量改变，必须重新 `colcon build --packages-select snowplow_control`。
- URDF/ros2_control 生成物改变，必须重新运行 `urdf_generator.py` 并重启 Gazebo。
- 该修复改变 Gazebo 接触驱动力分布，正式 Gazebo 地形实验需要重新跑。
- 不改变 `/cmd_vel`、`/odom` 话题语义，也不改变 `gazebo_speed_test.py` 指标定义。
