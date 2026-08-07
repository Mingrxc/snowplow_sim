# SolidWorks 建模与 Gazebo/ROS2 对接规范

本文档是当前项目的 CAD 建模接口。建模时应优先遵守本文件，而不是自行猜测 ROS2 或 Gazebo 的坐标和尺寸。

当前项目路径：

```text
E:\zheng_snowplow_project\snowplow_sim
```

Ubuntu 中对应的访问路径：

```text
/mnt/e/zheng_snowplow_project/snowplow_sim
```

## 1. 文件放置位置

### 1.1 SolidWorks 原始文件

建议放在项目外观模型的源文件目录：

```text
E:\zheng_snowplow_project\cad_source\solidworks\
```

建议至少保留：

```text
snowplow_robot.SLDASM
snowplow_robot.SLDPRT
snowplow_robot.step
```

如果装配体包含多个零件，应保留完整的 `.SLDASM` 和所有被引用的 `.SLDPRT`。

### 1.2 Gazebo 视觉模型

当前项目已有目录：

```text
E:\zheng_snowplow_project\snowplow_sim\meshes\
```

建议整理为：

```text
meshes/
├─ visual/
│  ├─ chassis_visual.dae
│  ├─ left_track_visual.dae
│  ├─ right_track_visual.dae
│  ├─ left_drive_wheel_visual.dae
│  ├─ right_drive_wheel_visual.dae
│  ├─ left_idler_wheel_visual.dae
│  ├─ right_idler_wheel_visual.dae
│  ├─ blade_base_visual.dae
│  └─ blade_visual.dae
├─ collision/
│  ├─ chassis_collision.stl
│  ├─ left_track_collision.stl
│  ├─ right_track_collision.stl
│  ├─ blade_base_collision.stl
│  └─ blade_collision.stl
└─ textures/
   ├─ chassis/
   ├─ track/
   └─ blade/
```

### 1.3 重要说明

目前 `urdf_generator.py` 中的 `chassis.mesh` 只提供了车体视觉网格的初步入口。完整 CAD 装配体需要后续扩展 Xacro，使车体、左右履带、驱动轮、从动轮和铲刀分别绑定到对应的 link。

因此：

- 不要现在把整个 SolidWorks 装配体直接当作一个 `base_link` 网格；
- 不要把一个包含铲刀和履带的整车 STL 作为唯一模型；
- 等待按本规范导出分部件文件后，再进行项目侧导入改造。

## 2. 必须保持的 ROS/Gazebo 接口

以下名称属于项目接口，不能因为 SolidWorks 的零件名称而修改：

```text
base_link
left_drive_wheel_joint
right_drive_wheel_joint
left_idler_wheel_joint
right_idler_wheel_joint
left_track_beam_joint
right_track_beam_joint
blade_pivot
blade_fixed
/cmd_vel
/odom
```

其中最重要的是：

| 项目接口 | 当前作用 |
| --- | --- |
| `base_link` | 机器人根坐标和车体主 link |
| `left_drive_wheel_joint` | 左侧驱动关节 |
| `right_drive_wheel_joint` | 右侧驱动关节 |
| `blade_pivot` | 扫雪铲俯仰关节 |
| `/cmd_vel` | 速度控制输入 |
| `/odom` | 里程计和速度输出 |

SolidWorks 中可以使用中文或自定义零件名称，但导出清单中必须明确对应到以上接口。

## 3. 坐标系规范

### 3.1 项目当前坐标约定

`base_link` 位于车体主结构的几何中心附近，当前全尺寸模型的车体视觉盒子以它为中心：

```text
车体长度方向：X
车体横向：Y
竖直方向：Z
```

当前项目中的前方是：

```text
X 正方向
```

扫雪铲位于：

```text
X 正方向
```

### 3.2 左右侧的现有项目约定

当前代码中左右履带位置是：

```text
左履带：Y = -0.325 m
右履带：Y = +0.325 m
```

这与部分 ROS 教程中的“Y 正方向为左”习惯不同。为了避免镜像、左右轮反向和转向异常，SolidWorks 导出时必须遵守当前项目的左右位置，不能自行交换左右。

### 3.3 当前关键关节坐标

全尺寸默认模型：

```text
左驱动轮中心：X=+0.45, Y=-0.325, Z=-0.15 m
右驱动轮中心：X=+0.45, Y=+0.325, Z=-0.15 m
左从动轮中心：X=-0.45, Y=-0.325, Z=-0.15 m
右从动轮中心：X=-0.45, Y=+0.325, Z=-0.15 m
```

驱动轮和从动轮旋转轴：

```text
axis = (0, 1, 0)
```

铲刀旋转轴：

```text
axis = (0, 1, 0)
```

铲刀根部关节位置：

```text
blade_pivot: X=+0.50, Y=0, Z=+0.20 m
```

### 3.4 原点要求

SolidWorks 装配体建议建立一个名为 `Gazebo_Origin` 的坐标系，并将其作为导出基准：

- 原点对应 `base_link`；
- X 正方向指向车头和扫雪铲；
- Y 正方向对应当前项目右侧；
- Z 正方向向上；
- 不要将原点放在地面；
- 不要将原点放在某个单独的驱动轮中心。

如果 SolidWorks 导出工具无法按上述方向导出，必须在交付文件中写明实际坐标方向和旋转变换，不能由导入方盲目试错。

## 4. 全尺寸默认模型参数

这些是当前 `examples/robot_default.yaml` 中的基准值。

### 4.1 车体

```text
长度：1.20 m
宽度：0.80 m
高度：0.40 m
车体质量参数：80.0 kg
```

当前基准惯量：

```text
ixx = 8.0 kg·m²
iyy = 12.0 kg·m²
izz = 10.0 kg·m²
```

SolidWorks 可以计算更准确的质量、质心和惯量，但第一次导入时建议先保持项目中的质量和惯量不变，只替换视觉模型。否则模型外观、质量和驱动力会同时变化，无法判断速度差异来自哪里。

### 4.2 履带和车轮

```text
单侧履带宽度：0.15 m
驱动轮半径：0.15 m
驱动轮直径：0.30 m
前后轮中心距：0.90 m
左右履带中心距：0.65 m
单个车轮质量参数：2.0 kg
单侧履带组件质量参数：5.0 kg
```

对应的轮心位置为：

```text
前轮 X = +0.45 m
后轮 X = -0.45 m
左/右轮 Y = -0.325 / +0.325 m
轮心 Z = -0.15 m
```

特别注意：

- 驱动轮半径会直接影响 `/odom` 速度和差速驱动；
- 左右履带中心距会直接影响原地转向角速度；
- 左右驱动轮直径不一致会造成持续偏航；
- 左右履带不要在 SolidWorks 中镜像后产生不同尺度。

### 4.3 扫雪铲

```text
铲刀宽度：1.00 m
铲刀高度：0.35 m
铲刀厚度：0.05 m
铲刀质量参数：15.0 kg
铲刀根部 X 偏移：+0.50 m
铲刀根部 Z 偏移：+0.20 m
```

关节限制：

```text
最小角度：-0.30 rad
最大角度：+0.50 rad
旋转轴：(0, 1, 0)
```

建议拆成：

```text
blade_base
blade
```

如果以后要模拟液压杆，再额外拆分：

```text
left_lift_arm
right_lift_arm
left_hydraulic_rod
right_hydraulic_rod
```

但新增部件不能擅自改变现有 `blade_pivot` 语义。

### 4.4 传感器安装基准

当前默认传感器位置相对于 `base_link`：

```text
LiDAR：X=0.30, Y=0, Z=0.55 m
IMU：X=0, Y=0, Z=0.30 m
```

SolidWorks 可以增加真实外壳和支架，但不要遮挡 LiDAR 的扫描方向，也不要让视觉外壳改变传感器 link 的坐标。

## 5. 单位要求

SolidWorks 建模可以使用毫米，但 Gazebo/URDF 使用米。

推荐做法：

1. SolidWorks 内部可以使用 `MMGS`；
2. 导出 STEP 作为尺寸基准；
3. 导出网格时明确记录单位；
4. Gazebo 使用的最终网格必须确认是米制；
5. 如果网格实际是毫米，必须在 Xacro 中使用 `scale="0.001 0.001 0.001"`，不能凭肉眼猜比例。

交付时必须在清单中写明：

```text
mesh_unit: m
```

或：

```text
mesh_unit: mm
required_scale: 0.001
```

## 6. 视觉模型与碰撞模型必须分离

### 6.1 视觉模型

视觉模型可以高精度，包含：

- 车身曲面；
- 螺栓；
- 加强筋；
- 灯具；
- 防护罩；
- 履带花纹；
- 铲刀加强结构；
- 液压杆外观；
- 传感器支架。

建议格式：

```text
DAE 优先
STL 可用但通常没有材质和纹理
```

### 6.2 碰撞模型

碰撞模型必须简化，建议只保留：

- 车体主碰撞包络；
- 左右驱动轮；
- 左右从动轮；
- 左右履带接地包络；
- 铲刀主板；
- 必要的液压杆碰撞体。

碰撞模型不要包含：

- 螺栓；
- 细小加强筋；
- 灯罩细节；
- 复杂履带齿；
- 内部不可接触零件；
- 纹理和装饰件。

否则 Gazebo 可能出现卡顿、穿透、弹跳或左右转不稳定。

## 7. 推荐交付文件

SolidWorks 建模完成后，建议一次性交付：

```text
cad_source/solidworks/
├─ snowplow_robot.SLDASM
├─ 所有被引用的 .SLDPRT
└─ snowplow_robot.step

snowplow_sim/meshes/visual/
├─ chassis_visual.dae
├─ left_track_visual.dae
├─ right_track_visual.dae
├─ left_drive_wheel_visual.dae
├─ right_drive_wheel_visual.dae
├─ left_idler_wheel_visual.dae
├─ right_idler_wheel_visual.dae
├─ blade_base_visual.dae
└─ blade_visual.dae

snowplow_sim/meshes/collision/
├─ chassis_collision.stl
├─ left_track_collision.stl
├─ right_track_collision.stl
├─ blade_base_collision.stl
└─ blade_collision.stl
```

另外提供一个清单文件：

```text
snowplow_sim/meshes/cad_manifest.yaml
```

清单至少包含：

```yaml
model_name: snowplow_robot
mesh_unit: m
coordinate_frame: Gazebo_Origin
forward_axis: +X
up_axis: +Z
project_left_side_y: -1
project_right_side_y: +1

links:
  base_link:
    visual: visual/chassis_visual.dae
    collision: collision/chassis_collision.stl
  left_track:
    visual: visual/left_track_visual.dae
    collision: collision/left_track_collision.stl
  right_track:
    visual: visual/right_track_visual.dae
    collision: collision/right_track_collision.stl
  blade:
    visual: visual/blade_visual.dae
    collision: collision/blade_collision.stl
```

## 8. 项目侧导入计划

项目侧不会直接修改论文实验参数。推荐分三阶段：

### 阶段一：只替换视觉

- 保留当前 collision；
- 保留当前质量和惯量；
- 保留当前差速驱动参数；
- 只导入 SolidWorks 视觉网格；
- 验证外观、坐标和左右方向。

### 阶段二：替换碰撞包络

- 导入简化 collision 网格；
- 重新检查 Spawn 高度；
- 检查地面接触；
- 检查原地左转和右转；
- 检查不同地形速度；
- 与阶段一结果对比。

### 阶段三：更新物理参数

- 使用 SolidWorks 质量属性更新质量、质心和惯量；
- 记录旧值和新值；
- 重新生成 URDF；
- 重新进行全部地形测速；
- 将结果标记为新的实验版本。

阶段三会改变实验动力学，必须单独记录，不能覆盖历史结果。

## 9. 建模时禁止事项

以下做法会导致导入或控制问题：

1. 将单位混用为毫米和米而不记录缩放；
2. 左右履带位置互换；
3. 把整车导出为一个不可拆分网格；
4. 改变 `base_link` 的位置；
5. 把驱动轮旋转轴改为 X 或 Z；
6. 让左右驱动轮直径不同；
7. 使用高精度视觉网格作为碰撞网格；
8. 在 SolidWorks 中加入项目未要求的自由度；
9. 直接根据外观估算质量和惯量并覆盖现有参数；
10. 修改 `/cmd_vel`、`/odom` 或现有关节名称。

## 10. 交付前检查表

交给项目导入前，请确认：

- [ ] 有完整 `.SLDASM` 和所有 `.SLDPRT`；
- [ ] 有 STEP 备份；
- [ ] 有独立视觉网格；
- [ ] 有独立碰撞网格；
- [ ] 所有尺寸和网格单位已写明；
- [ ] `base_link` 对应 `Gazebo_Origin`；
- [ ] 车头指向 X 正方向；
- [ ] 当前项目左履带位于 Y 负方向；
- [ ] 左右驱动轮中心距为 0.65 m；
- [ ] 驱动轮半径为 0.15 m；
- [ ] 前后轮中心距为 0.90 m；
- [ ] 铲刀根部位置为 `(0.50, 0, 0.20) m`；
- [ ] 驱动轮和铲刀旋转轴为 `(0, 1, 0)`；
- [ ] 没有把高精度装饰件放入碰撞网格；
- [ ] 提供了每个导出文件对应的 link 名称。

## 11. 当前项目基准文件

建模时需要对照的项目文件：

```text
E:\zheng_snowplow_project\snowplow_sim\examples\robot_default.yaml
E:\zheng_snowplow_project\snowplow_sim\urdf_generator.py
E:\zheng_snowplow_project\snowplow_sim\output\urdf\snowplow_robot.urdf.xacro
E:\zheng_snowplow_project\snowplow_sim\docs\MODEL_TERRAIN_REDESIGN_PLAN.md
```

其中：

- `robot_default.yaml` 是全尺寸默认参数；
- `urdf_generator.py` 是模型生成逻辑；
- `snowplow_robot.urdf.xacro` 是当前生成结果；
- `MODEL_TERRAIN_REDESIGN_PLAN.md` 说明视觉增强和物理语义保护原则。

