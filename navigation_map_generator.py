#!/usr/bin/env python3
"""Generate a Nav2 occupancy map from a snowplow world YAML file."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import yaml

from world_generator import OBSTACLE_PRESETS, load_config


def world_to_pixel(
    x: float,
    y: float,
    origin: float,
    resolution: float,
    height: int,
) -> tuple[int, int]:
    col = int(math.floor((x - origin) / resolution))
    row_from_bottom = int(math.floor((y - origin) / resolution))
    return col, height - 1 - row_from_bottom


def mark_disc(
    pixels: bytearray,
    width: int,
    height: int,
    center_x: float,
    center_y: float,
    radius: float,
    origin: float,
    resolution: float,
) -> None:
    col_min, row_max = world_to_pixel(
        center_x - radius, center_y - radius, origin, resolution, height
    )
    col_max, row_min = world_to_pixel(
        center_x + radius, center_y + radius, origin, resolution, height
    )
    radius_sq = radius * radius
    for row in range(max(0, row_min), min(height - 1, row_max) + 1):
        world_y = origin + (height - row - 0.5) * resolution
        for col in range(max(0, col_min), min(width - 1, col_max) + 1):
            world_x = origin + (col + 0.5) * resolution
            if (world_x - center_x) ** 2 + (world_y - center_y) ** 2 <= radius_sq:
                pixels[row * width + col] = 0


def mark_rotated_box(
    pixels: bytearray,
    width: int,
    height: int,
    center_x: float,
    center_y: float,
    box_width: float,
    box_depth: float,
    yaw: float,
    origin: float,
    resolution: float,
) -> None:
    radius = math.hypot(box_width, box_depth) / 2.0
    col_min, row_max = world_to_pixel(
        center_x - radius, center_y - radius, origin, resolution, height
    )
    col_max, row_min = world_to_pixel(
        center_x + radius, center_y + radius, origin, resolution, height
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    half_width = box_width / 2.0
    half_depth = box_depth / 2.0
    for row in range(max(0, row_min), min(height - 1, row_max) + 1):
        world_y = origin + (height - row - 0.5) * resolution
        for col in range(max(0, col_min), min(width - 1, col_max) + 1):
            world_x = origin + (col + 0.5) * resolution
            dx = world_x - center_x
            dy = world_y - center_y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if abs(local_x) <= half_width and abs(local_y) <= half_depth:
                pixels[row * width + col] = 0


def generate_map(config_path: Path, output_dir: Path, resolution: float) -> tuple[Path, Path]:
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    cfg: dict[str, Any] = load_config(str(config_path))
    size = float(cfg["ground_size"])
    width = max(1, int(math.ceil(size / resolution)))
    height = width
    origin = -size / 2.0
    pixels = bytearray([254]) * (width * height)

    for obstacle in cfg.get("obstacles", []):
        obstacle_type = str(obstacle.get("type", "cone"))
        if obstacle_type not in OBSTACLE_PRESETS:
            raise ValueError(f"unknown obstacle type: {obstacle_type}")
        shape, footprint_x, footprint_y = OBSTACLE_PRESETS[obstacle_type][
            "footprint"
        ]
        x = float(obstacle.get("x", 0.0))
        y = float(obstacle.get("y", 0.0))
        yaw = float(obstacle.get("rpy", [0.0, 0.0, 0.0])[2])
        if shape == "circle":
            mark_disc(
                pixels, width, height, x, y, footprint_x, origin, resolution
            )
        else:
            mark_rotated_box(
                pixels,
                width,
                height,
                x,
                y,
                footprint_x,
                footprint_y,
                yaw,
                origin,
                resolution,
            )

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = str(cfg.get("world_name", config_path.stem))
    pgm_path = output_dir / f"{stem}.pgm"
    yaml_path = output_dir / f"{stem}.yaml"
    with pgm_path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(pixels)
    metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": resolution,
        "origin": [origin, origin, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    print(f"[navigation_map_generator] map image: {pgm_path}")
    print(f"[navigation_map_generator] map yaml : {yaml_path}")
    print(
        f"[navigation_map_generator] size={width}x{height}, "
        f"resolution={resolution:.3f} m, obstacles={len(cfg.get('obstacles', []))}"
    )
    return pgm_path, yaml_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Nav2 map from the same YAML used by world_generator.py."
    )
    parser.add_argument("config", type=Path)
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("output/maps"))
    parser.add_argument("--resolution", type=float, default=0.05)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    generate_map(args.config, args.output_dir, args.resolution)


if __name__ == "__main__":
    main()
