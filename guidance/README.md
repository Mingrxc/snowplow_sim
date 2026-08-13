# 实验步骤指南

本指南只对应当前主线：`ros2_control + effort interface`。  
每次打开新的 Ubuntu 终端，都先执行环境加载命令，这是最容易漏的一步。

终端 2 中 `TARGET`、`PUBLISHED`、`FRICTION`、`CONTACT` 等字段的含义，见：

```text
guidance/TERMINAL_DISPLAY_FIELDS.md
```

如果你刚更新过本项目代码，尤其是 `snowplow_control/` 里的 C++ 控制器代码，请从第 2 节开始重新执行。只重新运行 `launch_sim.py` 不会自动重新编译控制器。

如果你刚刚启动失败过，建议先清理旧进程：

```bash
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher
```

## 0. 每个新终端都先做

```bash
source /opt/ros/humble/setup.bash

# 仓库换位置时只修改这一行；后续命令不依赖固定盘符。
export SNOWPLOW_SIM_ROOT=/mnt/e/zheng_snowplow_project/snowplow_sim
```

如果你已经编译过 `snowplow_control`，还要继续执行：

```bash
source ~/ros2_ws/install/setup.bash
```

检查是否生效：

```bash
ros2 pkg prefix snowplow_control
```

正常时应输出类似：

```text
/home/rurubaby/ros2_ws/install/snowplow_control
```

如果这一行报错，就先回到第 2 节重新编译控制器。

## 1. 进入项目

```bash
cd "$SNOWPLOW_SIM_ROOT"
```

## 2. 第一次使用或代码更新后：先编译控制器

第一次使用必须做。以后只要 `snowplow_control/` 里的 C++ 代码改过，也必须重做这一节。

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
ln -sfn "$SNOWPLOW_SIM_ROOT/snowplow_control" ~/ros2_ws/src/snowplow_control

cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash

ros2 pkg prefix snowplow_control
```

看到 `~/ros2_ws/install/snowplow_control` 才继续。

本次键盘控制修复后，还建议确认控制器源码已经是新版本：

```bash
grep -n "velocity_torque" "$SNOWPLOW_SIM_ROOT/snowplow_control/src/pmsm_cascade_controller.cpp"
grep -n "track proxy" "$SNOWPLOW_SIM_ROOT/snowplow_control/src/pmsm_cascade_controller.cpp"
```

能看到 `velocity_torque` 和 `track proxy` 才说明源码是当前修复后的版本。

## 3. 生成机器人和世界

控制参数会写进 `output/urdf/ros2_control.yaml`，所以控制参数改过后一定要重新生成机器人文件。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
python3 world_generator.py examples/terrain_multi.yaml -o output/worlds
```

应得到：

- `output/urdf/snowplow_robot.urdf.xacro`
- `output/urdf/ros2_control.yaml`
- `output/worlds/snowplow_multi_terrain.world`

确认新控制参数已经写入生成文件：

```bash
grep -n "velocity_p" output/urdf/ros2_control.yaml
grep -n "velocity_i" output/urdf/ros2_control.yaml
grep -n "left_idler_wheel_joint" output/urdf/ros2_control.yaml
grep -n "right_idler_wheel_joint" output/urdf/ros2_control.yaml
```

当前修复版应看到：

```text
velocity_p: 8.0
velocity_i: 4.0
left_idler_wheel_joint
right_idler_wheel_joint
```

本次履带转向修复还会把轮/地接触参数写进 Xacro。继续确认：

```bash
grep -n "slip2" output/urdf/snowplow_robot.urdf.xacro
grep -n "mu1" output/urdf/snowplow_robot.urdf.xacro
grep -n "mu2" output/urdf/snowplow_robot.urdf.xacro
```

当前默认履带轮接触应包含：

```text
mu1: 20.0
mu2: 0.35
slip2: 0.35
```

这些参数只在机器人重新 spawn 时生效，所以改完后必须重新生成 URDF 并重启 Gazebo。履带轮自身 `mu1` 保持高值，用于保证直行纵向抓地；`mu2/slip2` 保持可滑移，用于保证履带差速转向。

当前控制器还使用四轮履带代理：每侧控制器算出的总 effort 会平均分配给前后两个接触轮，使转向中心更接近车体中部。因为 C++ 控制器接口数量变了，本轮更新后必须重新执行第 2 节 `colcon build`。

默认 world 还会额外生成一块扩展实验地块：

```text
dynamic_pad: x=0, y=-18, width=8, depth=6
```

这块地不属于项目书三地形结论，只用于运行中手动调整地块摩擦系数。

默认世界是项目书三地形布局：

- `concrete`：水泥/硬化路面
- `sand`：沙土/泥沙路面
- `snow`：冰雪/积雪路面

## 4. 启动仿真：终端 1

不要把两段命令粘在同一行。尤其不要出现这种错误：

```text
--autopython3 launch_sim.py
```

正确命令如下：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 launch_sim.py \
  --xacro output/urdf/snowplow_robot.urdf.xacro \
  --world output/worlds/snowplow_multi_terrain.world \
  --auto
```

这一步会自动：

- 启动 `robot_state_publisher`
- 启动 Gazebo
- spawn 机器人
- 启动 `joint_state_broadcaster`
- 启动 `pmsm_controller`

## 5. 打开操作台：终端 2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 operator_console.py --terrain-config examples/terrain_multi.yaml
```

常用按键：

- `i` / `w` 前进
- `k` / `s` 后退
- `a` / 左方向键 原地左转，用于快速换方向
- `d` / 右方向键 原地右转，用于快速换方向
- `j` 原地左转，备用键
- `l` 原地右转，备用键
- 空格 立刻停车
- `q` 退出并停车

操作台里重点看两行：

- `PUBLISHED` 不是 0：说明键盘台已经在发布 `/cmd_vel`
- `ODOM` 是 `OK`：说明已经收到 Gazebo 里程计

如果你按一下键车只动一下就停，可以临时把保持时间调长：

```bash
python3 operator_console.py --terrain-config examples/terrain_multi.yaml --input-timeout 2.0
```

如果还想临时试更激进的手感，可以这样启动，不用改源码：

```bash
python3 operator_console.py --terrain-config examples/terrain_multi.yaml \
  --linear-speed 0.60 \
  --max-linear-speed 1.20 \
  --angular-speed 1.50 \
  --max-angular-speed 3.20 \
  --input-timeout 1.2
```

默认手感已经比旧版更快：

```text
linear-speed: 0.50 m/s
max-linear-speed: 1.20 m/s
angular-speed: 1.30 rad/s
max-angular-speed: 3.20 rad/s
```

终端 2 还会订阅 `/dynamic_drive/set`。启动后不用重启仿真，你可以在终端 3 动态调整 `w/s` 的线速度和 `a/d` 的角速度。

按 `a/d` 时，操作台里大约会显示：

```text
PUBLISHED: vx=+0.000 m/s  wz=+/-2.080 rad/s
```

这个组合是“换方向优先”的原地转向。履带车转向依赖地面横向滑移；如果 `PUBLISHED` 已经非零但车头仍几乎不转，优先检查第 3 节是否重新生成了含 `mu2/slip2` 的 Xacro，并确认第 4 节已经重启 Gazebo。

## 6. 做地形测速

先把机器人放到指定地形，再测速：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 terrain_spawn.py concrete
python3 gazebo_speed_test.py --label concrete --speed 0.20 --settle 2 --duration 5
```

换地形只改 `terrain_spawn.py` 后面的名字：

- `concrete`
- `sand`
- `snow`

扩展动态摩擦地块也可以用于手动定位：

- `dynamic_pad`

`gazebo_speed_test.py` 默认会把本次结果写到：

```text
results/gazebo_terrain/<时间>_<地形>/
```

目录里会有：

- `summary.csv`：本次摘要指标
- `odom_samples.csv`：原始 `/odom` 采样
- `manifest.json`：命令、参数、指标和结果类型说明

如果只想看终端输出，不保存结果：

```bash
python3 gazebo_speed_test.py --label snow --speed 0.20 --settle 2 --duration 5 --no-save
```

## 7. 动态摩擦与速度调参：终端 3

动态摩擦地块是额外扩展实验区，默认位置：

```text
x=0, y=-18
```

启动 Gazebo 和操作台后，再开第三个 Ubuntu 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 dynamic_friction_console.py
```

看到提示后可以输入摩擦系数，也可以同时输入速度：

```text
input > 0.30 0.15
input > 0.30 0.15 0.70 1.80
input > speed 0.80
input > speed 0.80 2.20
```

含义：

- 第一个数是 `mu`
- 第二个数是 `mu2`
- 第三个数如果存在，是终端 2 按 `w/s` 时使用的线速度 `vx`
- 第四个数如果存在，是终端 2 按 `a/d` 时使用的角速度 `wz`

三种常用输入：

```text
0.30 0.15
```

只改动态地块摩擦。

```text
0.30 0.15 0.70 1.80
```

同时改动态地块摩擦和按键速度。

```text
speed 0.80 2.20
```

不改摩擦，只改按键速度。

如果只输入 `speed 0.80`，只会改 `w/s` 的线速度，`a/d` 的角速度保持上一组值。

终端 3 会发布到：

```text
/dynamic_friction/set
/dynamic_drive/set
```

Gazebo 插件会把当前值发布到：

```text
/dynamic_friction/state
```

终端 2 的操作台进入 `dynamic_pad` 后，`FRICTION` 会显示动态地块当前配置值。注意这仍然是配置摩擦系数，不是 Gazebo 每一帧真实接触力。
终端 2 收到 `/dynamic_drive/set` 后，`DRIVE` 会显示 `source=dynamic`，并展示当前按键线速度和角速度。

想明显观察差异时，建议这样试：

```text
高附着：2.00 1.00
低附着：0.10 0.08
极低附着：0.05 0.04
```

注意：Gazebo 的 `mu/mu2` 表示最大抓地能力，不是线性行驶阻力。把 `mu` 改到 `20` 通常代表“非常抓地、不容易打滑”，不代表“摩擦阻力大所以一定变慢”。如果要看速度明显下降，应测试较低的 `mu/mu2`。

如果终端 3 一直只显示 `current (default)`，通常说明 Gazebo 插件没有加载。检查：

```bash
ros2 topic list | grep dynamic_friction
```

应能看到：

```text
/dynamic_friction/set
/dynamic_friction/state
```

如果看不到，重新编译并重启：

```bash
cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 world_generator.py examples/terrain_multi.yaml -o output/worlds
```

然后重新执行第 4 节启动 Gazebo。

## 8. 常见报错

### `snowplow_control is not built/sourced`

原因：当前终端没有加载自定义控制器包。

处理：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix snowplow_control
```

如果仍然找不到，就重新编译：

```bash
cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash
```

### `unrecognized arguments: --autopython3 launch_sim.py`

原因：把两次启动命令粘在一起了。

处理：重新复制第 4 节的完整命令，单独执行一次。

### `/controller_manager` 不见了

如果 Gazebo 终端反复出现：

```text
[gazebo_ros2_control]: robot_state_publisher service not available, waiting again...
```

说明 `gazebo_ros2_control` 没有拿到机器人描述，`/controller_manager` 就不会出现。当前 `launch_sim.py` 已经会自动启动 `robot_state_publisher`；如果仍然出现，请先确认你用的是更新后的启动脚本，并清理旧进程后重启：

```bash
cd "$SNOWPLOW_SIM_ROOT"
grep -n "Starting robot_state_publisher" launch_sim.py

pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher
```

然后重新执行第 4 节启动命令。

先确认：

```bash
ros2 service list | grep controller_manager
```

如果没有输出，通常是 `robot_state_publisher` 没有启动、机器人没有成功 spawn，或 `gazebo_ros2_control` 没有加载。

### `xacro` 报错

重新生成模型：

```bash
cd "$SNOWPLOW_SIM_ROOT"
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
xacro output/urdf/snowplow_robot.urdf.xacro > /tmp/snowplow_robot.urdf
```

### `/odom` 没有数据

检查顺序：

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 control list_controllers
```

`joint_state_broadcaster` 和 `pmsm_controller` 都应为 `active`。

### 按键后小车完全不动

先确认你已经重新编译控制器并重新生成 `output/urdf/ros2_control.yaml`。然后不要反复重启，按下面顺序查：

```bash
grep -n "velocity_p" "$SNOWPLOW_SIM_ROOT/output/urdf/ros2_control.yaml"
```

应看到：

```text
velocity_p: 8.0
```

再查控制器状态：

```bash
ros2 control list_controllers
```

应看到：

```text
joint_state_broadcaster active
pmsm_controller active
```

再开一个终端，看键盘台是否真的发出了速度：

```bash
ros2 topic echo /cmd_vel --once
```

此时在操作台按 `i` 或 `w`。如果 `linear.x` 仍然是 0，问题在键盘台；如果 `linear.x` 非 0，但车仍不动，问题在控制器或 Gazebo effort 接口。

控制器 C++ 改过后，必须重新编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash
```

重新编译后还要重新启动 Gazebo。已经打开的 Gazebo 不会自动加载新编译的控制器库。

### `w` 起步很慢，甚至短暂倒走

当前控制器已经移除了 Gazebo effort 适配层里容易反向补偿的负载观测前馈。遇到这个现象时，优先确认你已经重新编译并重新生成：

```bash
cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash

cd "$SNOWPLOW_SIM_ROOT"
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
grep -n "max_torque" output/urdf/ros2_control.yaml
grep -n "velocity_p" output/urdf/ros2_control.yaml
```

当前修复版应看到：

```text
max_torque: 45.0
velocity_p: 8.0
```

然后必须重启 Gazebo。旧 Gazebo 进程不会自动加载新控制器库。

### `w/s` 能走，但 `a/d` 几乎不转

这通常不是键盘问题。履带车靠左右履带差速和地面横向滑移转向；如果仿真里轮/地横向摩擦太强，小车会像被地面锁住，表现为按住 `a/d` 很久只转一点点。

本项目当前默认通过 `examples/robot_default.yaml` 配置履带轮接触：

```yaml
tracks:
  contact_mu1: 20.0
  contact_mu2: 0.35
  contact_slip2: 0.35
```

修改这些参数后必须重新生成并重启：

```bash
cd "$SNOWPLOW_SIM_ROOT"
python3 urdf_generator.py examples/robot_default.yaml -o output/urdf

pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher

python3 launch_sim.py \
  --xacro output/urdf/snowplow_robot.urdf.xacro \
  --world output/worlds/snowplow_multi_terrain.world \
  --auto
```

调试顺序建议：

1. 先在水泥地 `concrete` 测试，别先去雪地。
2. 按 `a` 看 `PUBLISHED` 是否约为 `vx=0.000, wz=+2.080`。
3. 如果仍然太慢，把 `contact_mu2` 再降到 `0.25`，重新生成并重启。
4. 如果开始横向漂得过多，把 `contact_mu2` 回调到 `0.45` 或把 `contact_slip2` 降到 `0.20`。

### `a/d` 能转，但像以前部为转轴

这通常说明前部驱动轮承担了过多接触驱动力。当前修复版已经把 `left_idler_wheel_joint` 和 `right_idler_wheel_joint` 也纳入 `ros2_control`，用四轮履带代理把每侧 effort 平均分给前后两个接触轮。

确认你已经生成并编译到新版本：

```bash
grep -n "left_idler_wheel_joint" output/urdf/ros2_control.yaml
grep -n "right_idler_wheel_joint" output/urdf/ros2_control.yaml

cd ~/ros2_ws
colcon build --packages-select snowplow_control
source install/setup.bash
```

然后必须重启 Gazebo。已经打开的 Gazebo 不会自动加载新的控制器接口。

## 9. 即将优化代码前的规则

后续优化代码时，优先级如下：

1. 不改变项目书语义
2. 不破坏 `ros2_control + effort` 主线
3. 先删冗余和重复，再做性能优化
4. 每次优化后都跑最小验证
5. 旧结果不和新结果混在一起

## 10. 点击地点自动寻路与避障演示

这一节独立于前面的键盘控制和三地形正式测速。演示场景仍包含沙地、水泥和雪地，并额外布置静态障碍物。Gazebo 世界和 Nav2 地图均从同一份 `examples/terrain_navigation.yaml` 生成。

### 10.1 首次使用或导航代码更新后编译

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
ln -sfn "$SNOWPLOW_SIM_ROOT/snowplow_control" ~/ros2_ws/src/snowplow_control
ln -sfn "$SNOWPLOW_SIM_ROOT/snowplow_navigation" ~/ros2_ws/src/snowplow_navigation

cd ~/ros2_ws
colcon build --packages-select snowplow_control snowplow_navigation
source install/setup.bash

ros2 pkg prefix snowplow_navigation
```

最后一条应显示：

```text
/home/rurubaby/ros2_ws/install/snowplow_navigation
```

### 10.2 生成机器人和导航世界

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd "$SNOWPLOW_SIM_ROOT"

python3 urdf_generator.py examples/robot_default.yaml -o output/urdf
python3 world_generator.py examples/terrain_navigation.yaml -o output/worlds
python3 navigation_map_generator.py examples/terrain_navigation.yaml -o output/maps
```

应生成：

```text
output/worlds/snowplow_navigation_demo.world
output/maps/snowplow_navigation_demo.pgm
output/maps/snowplow_navigation_demo.yaml
```

### 10.3 终端 1：启动 Gazebo 和小车

建议从地图下方 `x=0, y=-8` 出发：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd "$SNOWPLOW_SIM_ROOT"

python3 launch_sim.py \
  --xacro output/urdf/snowplow_robot.urdf.xacro \
  --world output/worlds/snowplow_navigation_demo.world \
  --auto \
  --x 0 \
  --y -8 \
  --z 0.35
```

只做无界面自动测试时，可在最后增加 `--headless`。

### 10.4 终端 2：启动 Nav2 和 RViz

不要启动 `operator_console.py`。键盘控制台和 Nav2 都会向 `/cmd_vel` 发命令，同时运行会互相抢控制权。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd "$SNOWPLOW_SIM_ROOT"

python3 launch_navigation.py
```

看到 Nav2 终端输出 `Managed nodes are active` 后再点击目标。

### 10.5 在 RViz 点击目标

1. 在 RViz 顶部工具栏选择 `Nav2 Goal`。
2. 在地图可通行区域按下鼠标左键并拖出一个箭头。
3. 松开鼠标，小车会规划路径、绕开障碍并驶向点击位置。
4. 到达判定只看位置，允许终点误差 `0.20 m`；箭头朝向不会强迫履带车到点后原地对齐。

不要点击障碍物内部、地图外部或离边界太近的位置。

### 10.6 不开 RViz的命令行验证

终端 2：

```bash
python3 launch_navigation.py --no-rviz
```

终端 3：发送地图坐标目标，例如 `(0, 5)`：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run snowplow_navigation navigation_goal 0 5
```

成功时最后显示：

```text
Goal succeeded
```

第三个可选参数是目标 yaw（弧度），但当前位置型终点检查器不会要求最终朝向严格一致：

```bash
ros2 run snowplow_navigation navigation_goal 3 6 1.57
```

### 10.7 快速排错

检查控制器、传感器和导航生命周期：

```bash
ros2 control list_controllers
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 lifecycle get /bt_navigator
ros2 param get /controller_server general_goal_checker.plugin
```

正常状态应包含：

```text
joint_state_broadcaster active
pmsm_controller active
active [3]
nav2_controller::PositionGoalChecker
```

若终端提示包不存在，回到 10.1 重新编译并在每个新终端执行 `source ~/ros2_ws/install/setup.bash`。若 Nav2 启动时提示节点名重复，先结束旧的导航终端，或执行：

```bash
killall -q controller_server smoother_server planner_server \
  behavior_server bt_navigator velocity_smoother map_server \
  lifecycle_manager ground_truth_tf
```

### 10.8 实验边界

- 当前 `map -> odom -> base_link` 来自 Gazebo 真值 `/odom`，用于验证规划、避障和底盘控制，不等同于完成 SLAM、AMCL 或实车定位。
- Nav2 代价地图使用 `/scan` 感知障碍物，静态地图来自场景 YAML。
- 导航场景是项目书自主导航与智能避障目标的工程扩展。
- 正式三地形测速仍使用 `examples/terrain_multi.yaml` 和 `gazebo_speed_test.py`。
- 导航演示的行程、耗时和速度不能直接写入正式三地形测速结果。
