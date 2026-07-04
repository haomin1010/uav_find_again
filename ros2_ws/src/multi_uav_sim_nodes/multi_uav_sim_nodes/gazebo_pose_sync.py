from __future__ import annotations

import math
import shutil
import subprocess
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from multi_uav_sim_msgs.msg import UavPose2D

from .scenario import default_uav_ids


@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


class GazeboPoseSync(Node):
    """Sync lightweight ROS2 mock poses into Gazebo Fortress models.

    This uses the `ign service /world/<world>/set_pose` command so the first
    Gazebo integration does not require a custom Gazebo plugin.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_pose_sync")
        self.declare_parameter("world_name", "multi_uav_reacquire")
        self.declare_parameter("sync_period", 0.2)
        self.declare_parameter("uav_altitude", 8.0)
        self.declare_parameter("timeout_ms", 100)

        self.world_name = str(self.get_parameter("world_name").value)
        self.uav_altitude = float(self.get_parameter("uav_altitude").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.service_name = f"/world/{self.world_name}/set_pose"
        self.ign_path = shutil.which("ign")
        self.warned_missing_ign = False

        self.model_poses: Dict[str, Pose2D] = {}
        self.create_subscription(PointStamped, "/target/ground_truth", self.on_target, 10)
        for uav_id in default_uav_ids():
            self.create_subscription(UavPose2D, f"/{uav_id}/pose2d", self.on_uav_pose, 10)

        self.create_timer(float(self.get_parameter("sync_period").value), self.on_timer)

    def on_target(self, msg: PointStamped) -> None:
        self.model_poses["target"] = Pose2D(msg.point.x, msg.point.y, 0.0, 0.0)

    def on_uav_pose(self, msg: UavPose2D) -> None:
        self.model_poses[msg.uav_id] = Pose2D(msg.position.x, msg.position.y, self.uav_altitude, msg.yaw)

    def on_timer(self) -> None:
        if self.ign_path is None:
            if not self.warned_missing_ign:
                self.get_logger().warning("ign command not found; Gazebo pose sync is disabled")
                self.warned_missing_ign = True
            return

        for model_name, pose in list(self.model_poses.items()):
            self.set_model_pose(model_name, pose)

    def set_model_pose(self, model_name: str, pose: Pose2D) -> None:
        half_yaw = pose.yaw * 0.5
        qw = math.cos(half_yaw)
        qz = math.sin(half_yaw)
        request = (
            f'name: "{model_name}" '
            f'position {{ x: {pose.x:.4f} y: {pose.y:.4f} z: {pose.z:.4f} }} '
            f'orientation {{ w: {qw:.6f} x: 0.0 y: 0.0 z: {qz:.6f} }}'
        )
        cmd = [
            self.ign_path,
            "service",
            "-s",
            self.service_name,
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(self.timeout_ms),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(cmd, check=False, capture_output=True, text=True, timeout=1.0)
        except subprocess.TimeoutExpired:
            self.get_logger().debug(f"Gazebo pose sync timeout for {model_name}")
            return
        except OSError as exc:
            self.get_logger().warning(f"Gazebo pose sync failed for {model_name}: {exc}")
            return

        if result.returncode != 0:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            detail = stderr or stdout or f"return code {result.returncode}"
            self.get_logger().debug(f"Gazebo pose sync rejected {model_name}: {detail}")


def main() -> None:
    rclpy.init()
    node = GazeboPoseSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

