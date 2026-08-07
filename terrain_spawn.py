#!/usr/bin/env python3
"""Delete and respawn the robot at a named terrain zone.

The coordinates match world_generator.py's default 30 m three-terrain layout.
This is a positioning helper only; it does not change terrain physics or the
speed metric.
"""

import argparse
from pathlib import Path

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node


TERRAIN_POSES = {
    "sand": (-10.0, 0.0),
    "concrete": (0.0, 0.0),
    "snow": (10.0, 0.0),
    "slope": (0.0, 10.0),
}


class TerrainSpawner(Node):
    def __init__(self, entity: str, urdf_path: Path, terrain: str,
                 z: float, delete_existing: bool):
        super().__init__("terrain_spawn")
        self.entity = entity
        self.urdf_path = urdf_path
        self.terrain = terrain
        self.z = z
        self.delete_existing = delete_existing
        self.delete_client = self.create_client(
            DeleteEntity, "/delete_entity"
        )
        self.spawn_client = self.create_client(
            SpawnEntity, "/spawn_entity"
        )

    def run(self) -> bool:
        if not self.urdf_path.is_file():
            self.get_logger().error(f"URDF file not found: {self.urdf_path}")
            return False

        x, y = TERRAIN_POSES[self.terrain]
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "Gazebo /spawn_entity is unavailable. Is Gazebo running?"
            )
            return False

        if self.delete_existing:
            if self.delete_client.wait_for_service(timeout_sec=5.0):
                request = DeleteEntity.Request()
                request.name = self.entity
                future = self.delete_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.exception() is not None:
                    self.get_logger().warning(
                        f"Could not delete existing entity: {future.exception()}"
                    )
                elif not future.result().success:
                    self.get_logger().warning(
                        f"Delete returned: {future.result().status_message}"
                    )
            else:
                self.get_logger().warning(
                    "Gazebo /delete_entity is unavailable; trying spawn anyway."
                )

        request = SpawnEntity.Request()
        request.name = self.entity
        request.xml = self.urdf_path.read_text(encoding="utf-8")
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = self.z

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            self.get_logger().error(f"Spawn failed: {future.exception()}")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(f"Spawn failed: {response.status_message}")
            return False

        self.get_logger().info(
            f"Spawned {self.entity} on {self.terrain} at "
            f"({x:.1f}, {y:.1f}, {self.z:.1f})"
        )
        return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Respawn snowplow at a named multi-terrain zone."
    )
    parser.add_argument("terrain", choices=sorted(TERRAIN_POSES))
    parser.add_argument("--urdf", default="/tmp/robot.urdf")
    parser.add_argument("--entity", default="snowplow")
    parser.add_argument("--z", type=float, default=0.3)
    parser.add_argument(
        "--keep-existing", action="store_true",
        help="do not call /delete_entity before spawning"
    )
    args = parser.parse_args()

    rclpy.init()
    node = TerrainSpawner(
        entity=args.entity,
        urdf_path=Path(args.urdf).expanduser(),
        terrain=args.terrain,
        z=args.z,
        delete_existing=not args.keep_existing,
    )
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
