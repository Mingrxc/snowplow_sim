#!/usr/bin/env python3
"""
world_generator.py — 参数化 Gazebo 多地形仿真场景生成器

用法:
    python3 world_generator.py <config.yaml> [--output-dir ./output/worlds]

示例:
    python3 world_generator.py examples/terrain_multi.yaml
    python3 world_generator.py examples/terrain_obstacles.yaml -o my_worlds/

输入: YAML 配置文件，描述地形区域和障碍物布局
输出: 完整的 Gazebo Classic SDF .world 文件，可直接用 gazebo 命令加载
"""

import os
import sys
import argparse
import copy
import yaml
from datetime import datetime
from typing import Dict, Any, List, Tuple, Optional


# ============================================================
# 地面物理参数库
#
# 项目计划书要求的默认三类场景为：
# 1) 水泥/硬化路面；2) 泥沙/沙土路面；3) 冰雪路面。
# 其余 asphalt/ice/grass 保留为扩展对照预设，不进入默认项目书场景。
# ============================================================

TERRAIN_PRESETS = {
    "asphalt": {
        "name": "标准沥青路面",
        "description": "硬质高摩擦路面，对照组基准",
        "color": (0.15, 0.15, 0.15, 1.0),   # dark grey
        "mu": 0.75,
        "mu2": 1.5,
        "kp": 1_000_000.0,
        "kd": 5000.0,
        "min_depth": 0.001,
        "max_vel": 0.1,
    },
    "snow": {
        "name": "冰雪路面",
        "description": "冰雪/积雪低附着路面，保留低附着同时保证履带可差速转向",
        "color": (0.95, 0.95, 0.95, 1.0),   # white
        "mu": 0.30,
        "mu2": 0.16,
        "kp": 80_000.0,
        "kd": 1000.0,
        "min_depth": 0.003,
        "max_vel": 0.05,
    },
    "sand": {
        "name": "沙地区",
        "description": "颗粒松软路面，行驶阻力大",
        "color": (0.85, 0.75, 0.55, 1.0),   # tan/sand
        "mu": 0.38,
        "mu2": 0.32,
        "kp": 120_000.0,
        "kd": 1500.0,
        "min_depth": 0.003,
        "max_vel": 0.05,
    },
    "ice": {
        "name": "冰面区",
        "description": "极端低附着力场景，控制算法鲁棒性测试",
        "color": (0.75, 0.90, 0.95, 1.0),   # light blue
        "mu": 0.08,
        "mu2": 0.16,
        "kp": 1_000_000.0,
        "kd": 5000.0,
        "min_depth": 0.001,
        "max_vel": 0.02,
    },
    "grass": {
        "name": "草地",
        "description": "中等摩擦软地",
        "color": (0.25, 0.60, 0.25, 1.0),   # green
        "mu": 0.60,
        "mu2": 1.20,
        "kp": 500_000.0,
        "kd": 3000.0,
        "min_depth": 0.002,
        "max_vel": 0.05,
    },
    "concrete": {
        "name": "水泥路面",
        "description": "标准硬质路面",
        "color": (0.55, 0.55, 0.55, 1.0),
        "mu": 1.20,
        "mu2": 0.70,
        "kp": 1_000_000.0,
        "kd": 5000.0,
        "min_depth": 0.001,
        "max_vel": 0.1,
    },
    "dynamic_pad": {
        "name": "动态摩擦地块",
        "description": "运行中由终端控制 mu/mu2 的扩展实验地块",
        "color": (0.35, 0.42, 0.70, 1.0),
        "mu": 0.80,
        "mu2": 0.45,
        "kp": 1_000_000.0,
        "kd": 5000.0,
        "min_depth": 0.001,
        "max_vel": 0.1,
    },
}


# ============================================================
# 障碍物预设
# ============================================================

OBSTACLE_PRESETS = {
    "cone": {
        "name": "交通锥",
        "mesh": None,  # 用 cylinder 近似
        "geometry": '<cylinder><radius>0.15</radius><length>0.5</length></cylinder>',
        "footprint": ("circle", 0.15, 0.15),
        "mass": 2.0,
        "color": (1.0, 0.4, 0.0, 1.0),
        "ground_z": 0.25,
    },
    "barrier": {
        "name": "路障",
        "mesh": None,
        "geometry": '<box><size>1.5 0.3 0.6</size></box>',
        "footprint": ("box", 1.50, 0.30),
        "mass": 15.0,
        "color": (0.9, 0.9, 0.0, 1.0),
        "ground_z": 0.30,
    },
    "wall": {
        "name": "墙体",
        "mesh": None,
        "geometry": '<box><size>2.0 0.2 1.0</size></box>',
        "footprint": ("box", 2.00, 0.20),
        "mass": 200.0,
        "color": (0.5, 0.4, 0.35, 1.0),
        "ground_z": 0.50,
    },
    "box": {
        "name": "纸箱",
        "mesh": None,
        "geometry": '<box><size>0.4 0.4 0.4</size></box>',
        "footprint": ("box", 0.40, 0.40),
        "mass": 3.0,
        "color": (0.7, 0.5, 0.3, 1.0),
        "ground_z": 0.20,
    },
    "tree": {
        "name": "树木",
        "mesh": None,
        "geometry": '<cylinder><radius>0.1</radius><length>2.0</length></cylinder>',
        "footprint": ("circle", 0.10, 0.10),
        "mass": 50.0,
        "color": (0.3, 0.2, 0.1, 1.0),
        "ground_z": 1.00,
    },
}


# ============================================================
# 默认配置
# ============================================================

DEFAULT_WORLD_CONFIG = {
    "world_name": "snowplow_world",
    "ground_size": 30.0,           # 总体场地大小 (m)，正方形
    "physics": {
        "engine": "ode",           # Gazebo Classic 默认 ODE 引擎
        "max_step_size": 0.001,
        "real_time_update_rate": 1000,
        "gravity": [0, 0, -9.81],
    },
    "scene": {
        "sky_color": [0.5, 0.7, 1.0],
        "ambient": [0.5, 0.5, 0.5, 1.0],
        "shadows": True,
    },
    "terrain_zones": [
        # 每个 zone: {type, x, y, width, depth, ...}
        # 如果不指定 zones，则生成单一默认地面
    ],
    "obstacles": [
        # 每个 obstacle: {type, x, y, z, rpy}
        # type 必须匹配 OBSTACLE_PRESETS 的 key
    ],
    "lighting": {
        "sun_pos": [10, 10, 30],
        "sun_diffuse": [0.8, 0.8, 0.8, 1.0],
    },
    "visual_detail": {
        "enabled": True,       # 仅增强视觉，不改变地形 collision / 摩擦参数
        "tiles": 5,            # 每个地形区域的视觉纹理分块数
        "edge_band": 0.18,     # 地形边界视觉带宽 (m)
    },
    "dynamic_friction_pad": {
        "enabled": False,
        "x": 0.0,
        "y": -18.0,
        "width": 8.0,
        "depth": 6.0,
        "thickness": 0.02,
        "mu": 0.80,
        "mu2": 0.45,
        "kp": 1_000_000.0,
        "kd": 5000.0,
        "min_depth": 0.001,
        "max_vel": 0.1,
        "command_topic": "/dynamic_friction/set",
        "state_topic": "/dynamic_friction/state",
    },
}


# ============================================================
# SDF 生成
# ============================================================

def visual_detail_enabled(cfg: dict) -> bool:
    return cfg.get("visual_detail", {}).get("enabled", True)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def rgba_text(color: Tuple[float, float, float, float]) -> str:
    return " ".join(f"{float(v):.3f}" for v in color)


def shifted_color(color: Tuple[float, float, float, float],
                  delta: float) -> Tuple[float, float, float, float]:
    r, g, b, a = color
    return (clamp01(r + delta), clamp01(g + delta),
            clamp01(b + delta), a)


def sdf_visual_box(name: str, xyz: str, size: str,
                   color: Tuple[float, float, float, float],
                   rpy: str = "0 0 0") -> str:
    rgba = rgba_text(color)
    return f"""
        <visual name="{name}">
          <pose>{xyz} {rpy}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>
        </visual>
"""


def sdf_terrain_visual_details(zone: dict, preset: dict, zone_index: int,
                               visual_cfg: dict) -> str:
    """生成地形视觉细节，不生成 collision。"""
    if not visual_detail_enabled({"visual_detail": visual_cfg}):
        return ""

    terrain_type = zone["type"]
    width = float(zone.get("width", 5.0))
    depth = float(zone.get("depth", 5.0))
    thickness = float(zone.get("thickness", 0.01))
    base_color = preset["color"]
    tiles = max(2, int(visual_cfg.get("tiles", 5)))
    edge_band = min(float(visual_cfg.get("edge_band", 0.18)),
                    max(width, depth) * 0.04)
    patch_h = max(0.002, thickness * 0.10)
    patch_z = thickness / 2 + patch_h / 2 + 0.001

    parts = []
    tile_w = width / tiles
    tile_d = depth / tiles

    for ix in range(tiles):
        for iy in range(tiles):
            seed = zone_index * 17 + ix * 7 + iy * 11
            shade = ((seed % 7) - 3) * 0.018
            if terrain_type == "snow":
                shade *= 0.55
            elif terrain_type == "ice":
                shade *= 0.75
            color = shifted_color(base_color, shade)
            px = -width / 2 + tile_w * (ix + 0.5)
            py = -depth / 2 + tile_d * (iy + 0.5)
            bump = ((seed % 5) - 2) * 0.0007
            parts.append(sdf_visual_box(
                f"terrain_patch_{zone_index}_{ix}_{iy}",
                f"{px:.3f} {py:.3f} {patch_z + bump:.4f}",
                f"{tile_w * 0.96:.3f} {tile_d * 0.96:.3f} {patch_h:.4f}",
                color,
            ))

    edge_color = shifted_color(base_color, -0.08)
    edge_z = thickness / 2 + patch_h + 0.002
    parts.extend([
        sdf_visual_box(
            f"terrain_edge_n_{zone_index}",
            f"0 {depth / 2 - edge_band / 2:.3f} {edge_z:.4f}",
            f"{width:.3f} {edge_band:.3f} {patch_h:.4f}",
            edge_color,
        ),
        sdf_visual_box(
            f"terrain_edge_s_{zone_index}",
            f"0 {-depth / 2 + edge_band / 2:.3f} {edge_z:.4f}",
            f"{width:.3f} {edge_band:.3f} {patch_h:.4f}",
            edge_color,
        ),
        sdf_visual_box(
            f"terrain_edge_e_{zone_index}",
            f"{width / 2 - edge_band / 2:.3f} 0 {edge_z:.4f}",
            f"{edge_band:.3f} {depth:.3f} {patch_h:.4f}",
            edge_color,
        ),
        sdf_visual_box(
            f"terrain_edge_w_{zone_index}",
            f"{-width / 2 + edge_band / 2:.3f} 0 {edge_z:.4f}",
            f"{edge_band:.3f} {depth:.3f} {patch_h:.4f}",
            edge_color,
        ),
    ])

    feature_z = thickness / 2 + patch_h * 1.8 + 0.002
    if terrain_type in ("asphalt", "concrete"):
        parts.append(sdf_visual_box(
            f"{terrain_type}_center_line_{zone_index}",
            f"0 0 {feature_z:.4f}",
            f"{width * 0.70:.3f} 0.070 0.003",
            (0.95, 0.78, 0.12, 1.0),
        ))
        parts.append(sdf_visual_box(
            f"{terrain_type}_stop_mark_{zone_index}",
            f"{-width * 0.22:.3f} {depth * 0.18:.3f} {feature_z + 0.001:.4f}",
            f"{width * 0.18:.3f} 0.050 0.003",
            (0.95, 0.95, 0.88, 1.0),
            rpy="0 0 0.65",
        ))
    elif terrain_type == "snow":
        bank_color = (0.98, 0.99, 1.0, 1.0)
        parts.extend([
            sdf_visual_box(
                f"snow_bank_n_{zone_index}",
                f"0 {depth / 2 - 0.25:.3f} {feature_z + 0.012:.4f}",
                f"{width * 0.88:.3f} 0.36 0.045",
                bank_color,
            ),
            sdf_visual_box(
                f"snow_bank_s_{zone_index}",
                f"0 {-depth / 2 + 0.25:.3f} {feature_z + 0.006:.4f}",
                f"{width * 0.82:.3f} 0.30 0.035",
                bank_color,
            ),
            sdf_visual_box(
                f"snow_track_marks_{zone_index}",
                f"0 0 {feature_z:.4f}",
                f"{width * 0.58:.3f} 0.070 0.004",
                (0.78, 0.83, 0.86, 1.0),
            ),
        ])
    elif terrain_type == "sand":
        for k, offset in enumerate([-0.28, -0.12, 0.05, 0.22]):
            parts.append(sdf_visual_box(
                f"sand_ripple_{zone_index}_{k}",
                f"0 {depth * offset:.3f} {feature_z + k * 0.0007:.4f}",
                f"{width * 0.82:.3f} 0.075 0.004",
                (0.72, 0.61, 0.42, 1.0),
                rpy=f"0 0 {0.08 * ((k % 2) * 2 - 1):.3f}",
            ))
    elif terrain_type == "ice":
        crack_color = (0.38, 0.63, 0.78, 1.0)
        parts.extend([
            sdf_visual_box(
                f"ice_crack_a_{zone_index}",
                f"0 0 {feature_z:.4f}",
                f"{width * 0.74:.3f} 0.035 0.004",
                crack_color,
                rpy="0 0 0.42",
            ),
            sdf_visual_box(
                f"ice_crack_b_{zone_index}",
                f"{width * 0.12:.3f} {-depth * 0.16:.3f} {feature_z + 0.001:.4f}",
                f"{width * 0.42:.3f} 0.025 0.004",
                (0.85, 0.95, 1.0, 1.0),
                rpy="0 0 -0.38",
            ),
        ])
    elif terrain_type == "grass":
        for k, (px, py) in enumerate([(-0.22, 0.18), (0.12, -0.18), (0.28, 0.02)]):
            parts.append(sdf_visual_box(
                f"grass_tuft_{zone_index}_{k}",
                f"{width * px:.3f} {depth * py:.3f} {feature_z:.4f}",
                f"{width * 0.18:.3f} {depth * 0.10:.3f} 0.006",
                (0.18, 0.42, 0.16, 1.0),
                rpy=f"0 0 {0.35 * (k - 1):.3f}",
            ))

    return "".join(parts)

def sdf_header(cfg: dict) -> str:
    """生成 World 文件头部"""
    world_name = cfg["world_name"]
    phys = cfg.get("physics", {})
    scene = cfg.get("scene", {})
    gravity = " ".join(str(g) for g in phys.get("gravity", [0, 0, -9.81]))
    return f"""<?xml version="1.0" ?>
<!--
  Gazebo 多地形仿真场景
  由 world_generator.py 自动生成
  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
  场景名称: {world_name}
-->
<sdf version="1.6">
  <world name="{world_name}">

    <gravity>{gravity}</gravity>

    <!-- ===== 物理引擎配置 ===== -->
    <physics type="{phys.get('engine', 'ode')}">
      <max_step_size>{phys.get('max_step_size', 0.001)}</max_step_size>
      <real_time_update_rate>{phys.get('real_time_update_rate', 1000)}</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>


    <!-- ===== 场景渲染 ===== -->
    <scene>
      <ambient>{' '.join(str(v) for v in scene.get('ambient', [0.5, 0.5, 0.5, 1.0]))}</ambient>
      <background>{' '.join(str(v) for v in scene.get('sky_color', [0.5, 0.7, 1.0]))} 1.0</background>
      <shadows>{str(scene.get('shadows', True)).lower()}</shadows>
    </scene>

    <!-- ===== 光照 ===== -->
    <include>
      <uri>model://sun</uri>
    </include>

    <light name="main_sun" type="directional">
      <pose>{' '.join(str(v) for v in cfg.get('lighting', {}).get('sun_pos', [10, 10, 30]))} 0 -0.5 -0.87</pose>
      <diffuse>{' '.join(str(v) for v in cfg.get('lighting', {}).get('sun_diffuse', [0.8, 0.8, 0.8, 1.0]))}</diffuse>
      <specular>0.2 0.2 0.2 1.0</specular>
      <cast_shadows>true</cast_shadows>
    </light>

"""


def sdf_ground_plane(cfg: dict) -> str:
    """生成基础地面"""
    size = cfg["ground_size"]
    return f"""
    <!-- ===== 基础地面 ===== -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>{size*2} {size*2}</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.75</mu>
                <mu2>1.5</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1000000.0</kp>
                <kd>5000.0</kd>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>{size*2} {size*2}</size></plane>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1.0</ambient>
            <diffuse>0.15 0.15 0.15 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def sdf_terrain_zone(zone: dict, zone_index: int,
                     visual_cfg: Optional[dict] = None) -> str:
    """生成单个地形区域"""
    terrain_type = zone["type"]
    preset = TERRAIN_PRESETS.get(terrain_type, TERRAIN_PRESETS["asphalt"])

    x = zone.get("x", 0)
    y = zone.get("y", 0)
    width = zone.get("width", 5.0)
    depth = zone.get("depth", 5.0)
    thickness = zone.get("thickness", 0.01)
    z = thickness / 2  # 放在地面上方

    r, g, b, a = preset["color"]
    visual_details = sdf_terrain_visual_details(
        zone, preset, zone_index, visual_cfg or {}
    )

    # 检查是否有过渡带
    has_transition = zone.get("transition", False)
    transition_width = zone.get("transition_width", 0.5) if has_transition else 0

    base = f"""
    <!-- ===== 地形区 [{zone_index}]: {preset['name']} ({terrain_type}) ===== -->
    <model name="terrain_{terrain_type}_{zone_index}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="terrain_link_{zone_index}">
        <collision name="terrain_collision_{zone_index}">
          <geometry>
            <box><size>{width} {depth} {thickness}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{preset['mu']}</mu>
                <mu2>{preset['mu2']}</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>{preset['kp']}</kp>
                <kd>{preset['kd']}</kd>
                <max_vel>{preset['max_vel']}</max_vel>
                <min_depth>{preset['min_depth']}</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="terrain_visual_{zone_index}">
          <geometry>
            <box><size>{width} {depth} {thickness}</size></box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
{visual_details}
      </link>
    </model>
"""
    return base


def sdf_dynamic_friction_pad(cfg: dict) -> str:
    pad = cfg.get("dynamic_friction_pad", {})
    if not pad.get("enabled", False):
        return "\n    <!-- 动态摩擦地块: 已禁用 -->"

    preset = TERRAIN_PRESETS["dynamic_pad"]
    x = float(pad.get("x", 0.0))
    y = float(pad.get("y", -18.0))
    width = float(pad.get("width", 8.0))
    depth = float(pad.get("depth", 6.0))
    thickness = float(pad.get("thickness", 0.02))
    z = thickness / 2.0
    mu = float(pad.get("mu", preset["mu"]))
    mu2 = float(pad.get("mu2", preset["mu2"]))
    kp = float(pad.get("kp", preset["kp"]))
    kd = float(pad.get("kd", preset["kd"]))
    min_depth = float(pad.get("min_depth", preset["min_depth"]))
    max_vel = float(pad.get("max_vel", preset["max_vel"]))
    command_topic = pad.get("command_topic", "/dynamic_friction/set")
    state_topic = pad.get("state_topic", "/dynamic_friction/state")
    r, g, b, a = preset["color"]

    return f"""
    <!-- ===== 扩展实验区: 动态摩擦地块 ===== -->
    <model name="dynamic_friction_pad">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="dynamic_friction_link">
        <collision name="dynamic_friction_collision">
          <geometry>
            <box><size>{width} {depth} {thickness}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{mu}</mu>
                <mu2>{mu2}</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>{kp}</kp>
                <kd>{kd}</kd>
                <max_vel>{max_vel}</max_vel>
                <min_depth>{min_depth}</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="dynamic_friction_visual">
          <geometry>
            <box><size>{width} {depth} {thickness}</size></box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
        <visual name="dynamic_friction_center_mark">
          <pose>0 0 {thickness / 2.0 + 0.004} 0 0 0</pose>
          <geometry>
            <box><size>{width * 0.80:.3f} 0.080 0.004</size></box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.20 1.0</ambient>
            <diffuse>0.95 0.95 0.20 1.0</diffuse>
          </material>
        </visual>
      </link>
      <plugin name="dynamic_friction_plugin" filename="libsnowplow_dynamic_friction_plugin.so">
        <link_name>dynamic_friction_link</link_name>
        <collision_name>dynamic_friction_collision</collision_name>
        <initial_mu>{mu}</initial_mu>
        <initial_mu2>{mu2}</initial_mu2>
        <command_topic>{command_topic}</command_topic>
        <state_topic>{state_topic}</state_topic>
        <publish_rate>2.0</publish_rate>
      </plugin>
    </model>
"""


def sdf_obstacle(obs: dict, obs_index: int) -> str:
    """生成单个障碍物"""
    obs_type = obs.get("type", "cone")
    preset = OBSTACLE_PRESETS.get(obs_type, OBSTACLE_PRESETS["cone"])

    x = obs.get("x", 0)
    y = obs.get("y", 0)
    z = float(obs.get("z", 0.0)) + float(preset.get("ground_z", 0.0))
    r, p, yaw = obs.get("rpy", [0, 0, 0])
    r_, g_, b_, a_ = preset["color"]

    return f"""
    <!-- ===== 障碍物 [{obs_index}]: {preset['name']} ===== -->
    <model name="obstacle_{obs_type}_{obs_index}">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {yaw}</pose>
      <link name="obs_link_{obs_index}">
        <collision name="obs_collision_{obs_index}">
          <geometry>
            {preset['geometry']}
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1000000.0</kp>
                <kd>5000.0</kd>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="obs_visual_{obs_index}">
          <geometry>
            {preset['geometry']}
          </geometry>
          <material>
            <ambient>{r_} {g_} {b_} {a_}</ambient>
            <diffuse>{r_} {g_} {b_} {a_}</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>{preset['mass']}</mass>
          <inertia>
            <ixx>1.0</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1.0</iyy><iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
"""


def sdf_footer() -> str:
    return """
  </world>
</sdf>
"""


# ============================================================
# 默认地形布局（论文中的分区式场景）
# ============================================================

def default_zones(size: float) -> List[Dict]:
    """生成项目书默认三地形布局：水泥、沙土、冰雪三条测试带。"""
    lane_width = size / 3
    lane_depth = size * 0.8
    return [
        {"type": "sand", "x": -lane_width, "y": 0, "width": lane_width, "depth": lane_depth, "thickness": 0.02},
        {"type": "concrete", "x": 0, "y": 0, "width": lane_width, "depth": lane_depth, "thickness": 0.015},
        {"type": "snow", "x": lane_width, "y": 0, "width": lane_width, "depth": lane_depth, "thickness": 0.02},
    ]


# ============================================================
# 核心生成逻辑
# ============================================================

def deep_merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    if override is None:
        return result
    if not isinstance(override, dict):
        raise ValueError("YAML 配置文件顶层必须是字典/map")
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def load_config(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        user_config = yaml.safe_load(f)
    return deep_merge(DEFAULT_WORLD_CONFIG, user_config)


def generate_world(cfg: dict, output_dir: str) -> str:
    world_name = cfg["world_name"]
    os.makedirs(output_dir, exist_ok=True)

    # 如果用户没指定地形区，用默认分区布局
    zones = cfg.get("terrain_zones", [])
    if not zones:
        zones = default_zones(cfg["ground_size"])

    # 拼接 SDF
    parts = [sdf_header(cfg), sdf_ground_plane(cfg)]

    visual_cfg = cfg.get("visual_detail", {})
    for i, zone in enumerate(zones):
        parts.append(sdf_terrain_zone(zone, i, visual_cfg))

    parts.append(sdf_dynamic_friction_pad(cfg))

    obstacles = cfg.get("obstacles", [])
    for i, obs in enumerate(obstacles):
        parts.append(sdf_obstacle(obs, i))

    parts.append(sdf_footer())

    world_path = os.path.join(output_dir, f"{world_name}.world")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write("".join(parts))

    print(f"[world_generator] World 文件已生成: {world_path}")
    print(f"[world_generator] 地形区数量: {len(zones)}")
    for z in zones:
        print(f"  - {TERRAIN_PRESETS.get(z['type'], {}).get('name', z['type'])} "
              f"({z['width']}x{z['depth']} m @ [{z['x']}, {z['y']}])")
    print(f"[world_generator] 障碍物数量: {len(obstacles)}")
    dynamic_pad = cfg.get("dynamic_friction_pad", {})
    print(f"[world_generator] 动态摩擦地块: {'启用' if dynamic_pad.get('enabled') else '禁用'}")

    return world_path



def main():
    parser = argparse.ArgumentParser(
        description="参数化 Gazebo 多地形仿真场景生成器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
内置地形类型:
  concrete 水泥/硬化路面 (μ=0.70) [项目书默认]
  sand     沙土/泥沙路面 (μ=0.50) [项目书默认]
  snow     冰雪/积雪路面 (mu=0.30, mu2=0.16) [项目书默认]
  asphalt  标准沥青路面 (μ=0.75) [扩展对照]
  ice      冰面区 (μ=0.08) [扩展对照]
  grass    草地 (μ=0.60) [扩展对照]

示例:
  python3 world_generator.py examples/terrain_multi.yaml
  python3 world_generator.py examples/terrain_obstacles.yaml -o output/worlds
        """,
    )
    parser.add_argument("config", help="YAML 配置文件路径")
    parser.add_argument("-o", "--output-dir", default="./output/worlds",
                        help="输出目录 (默认: ./output/worlds)")
    args = parser.parse_args()

    cfg = load_config(args.config)

    world_name = cfg["world_name"]
    ground_size = cfg["ground_size"]
    phys_engine = cfg.get("physics", {}).get("engine", "ode")
    print(f"[world_generator] 场景: {world_name}")
    print(f"[world_generator] 场地大小: {ground_size}x{ground_size} m")
    print(f"[world_generator] 物理引擎: {phys_engine}")

    world_path = generate_world(cfg, args.output_dir)

    print("\n[OK] 生成完成。")
    wpath_abs = os.path.abspath(world_path)
    print(f"   启动命令: ros2 launch gazebo_ros gazebo.launch.py world:={wpath_abs} verbose:=true")


if __name__ == "__main__":
    main()
