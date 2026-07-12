from __future__ import annotations

import math
import os
import shutil
import subprocess
from dataclasses import dataclass
from typing import Dict

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


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_pose(current: Pose2D, target: Pose2D, alpha: float) -> Pose2D:
    yaw_delta = wrap_angle(target.yaw - current.yaw)
    return Pose2D(
        x=current.x + (target.x - current.x) * alpha,
        y=current.y + (target.y - current.y) * alpha,
        z=current.z + (target.z - current.z) * alpha,
        yaw=wrap_angle(current.yaw + yaw_delta * alpha),
    )


class GazeboPoseSync(Node):
    """Sync ROS2 mock poses into Ignition Gazebo models."""

    def __init__(self) -> None:
        super().__init__("gazebo_pose_sync")
        self.declare_parameter("world_name", "multi_uav_reacquire")
        self.declare_parameter("ign_partition", "multi_uav_reacquire")
        self.declare_parameter("ign_ip", "127.0.0.1")
        self.declare_parameter("sync_period", 0.05)
        self.declare_parameter("smoothing_time", 0.15)
        self.declare_parameter("uav_altitude", 8.0)
        self.declare_parameter("timeout_ms", 500)

        self.world_name = str(self.get_parameter("world_name").value)
        self.ign_partition = str(self.get_parameter("ign_partition").value)
        self.ign_ip = str(self.get_parameter("ign_ip").value)
        self.sync_period = float(self.get_parameter("sync_period").value)
        self.smoothing_time = float(self.get_parameter("smoothing_time").value)
        self.uav_altitude = float(self.get_parameter("uav_altitude").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.vector_service_name = f"/world/{self.world_name}/set_pose_vector"
        self.single_service_name = f"/world/{self.world_name}/set_pose"
        self.ign_path = shutil.which("ign")
        self.use_vector_service = True

        self.target_poses: Dict[str, Pose2D] = {}
        self.display_poses: Dict[str, Pose2D] = {}
        self.logged_models = set()
        self.warned_missing_ign = False
        self.warned_sync_failure = False
        self.logged_connected = False
        self.success_count = 0
        self.failure_count = 0
        self.timeout_count = 0
        self.last_status_log = 0.0
        self.status_log_period = 5.0
        self.previous_target_pose: Pose2D | None = None
        self.target_yaw = 0.0

        self.create_subscription(PointStamped, "/target/ground_truth", self.on_target, 10)
        for uav_id in default_uav_ids():
            self.create_subscription(UavPose2D, f"/{uav_id}/pose2d", self.on_uav_pose, 10)

        self.create_timer(self.sync_period, self.on_timer)
        self.get_logger().info(
            "Gazebo pose sync starting: "
            f"service={self.vector_service_name}, partition={self.ign_partition}, "
            f"period={self.sync_period:.3f}s, smoothing_time={self.smoothing_time:.3f}s, "
            f"altitude={self.uav_altitude:.2f}, timeout_ms={self.timeout_ms}"
        )

    def ign_env(self) -> dict:
        env = os.environ.copy()
        env["IGN_PARTITION"] = self.ign_partition
        env["IGN_IP"] = self.ign_ip
        return env

    def on_target(self, msg: PointStamped) -> None:
        pose = Pose2D(msg.point.x, msg.point.y, msg.point.z, self.target_yaw)
        if self.previous_target_pose is not None:
            dx = pose.x - self.previous_target_pose.x
            dy = pose.y - self.previous_target_pose.y
            if math.hypot(dx, dy) > 0.02:
                self.target_yaw = math.atan2(dy, dx)
                pose.yaw = self.target_yaw
        self.previous_target_pose = pose
        self.update_target_pose("target", pose)

    def on_uav_pose(self, msg: UavPose2D) -> None:
        self.update_target_pose(
            msg.uav_id,
            Pose2D(msg.position.x, msg.position.y, self.uav_altitude, msg.yaw),
        )

    def update_target_pose(self, model_name: str, pose: Pose2D) -> None:
        self.target_poses[model_name] = pose
        self.display_poses.setdefault(model_name, pose)
        if model_name not in self.logged_models:
            self.logged_models.add(model_name)
            self.get_logger().info(
                f"received first ROS pose for {model_name}: "
                f"x={pose.x:.2f}, y={pose.y:.2f}, z={pose.z:.2f}, yaw={pose.yaw:.2f}"
            )

    def on_timer(self) -> None:
        if self.ign_path is None:
            if not self.warned_missing_ign:
                self.get_logger().warning("ign command was not found; Gazebo pose sync is disabled")
                self.warned_missing_ign = True
            return
        if not self.target_poses:
            return

        self.advance_display_poses()
        self.set_model_poses(self.display_poses)
        self.log_status()

    def advance_display_poses(self) -> None:
        if self.smoothing_time <= 0.0:
            self.display_poses = dict(self.target_poses)
            return
        alpha = min(1.0, self.sync_period / self.smoothing_time)
        for model_name, target in self.target_poses.items():
            current = self.display_poses.get(model_name, target)
            self.display_poses[model_name] = interpolate_pose(current, target, alpha)

    def set_model_poses(self, poses: Dict[str, Pose2D]) -> None:
        if not self.use_vector_service:
            self.set_model_poses_individually(poses)
            return

        request = " ".join(
            "pose { " + self.pose_request(model_name, pose) + " }"
            for model_name, pose in poses.items()
        )
        cmd = [
            self.ign_path,
            "service",
            "-s",
            self.vector_service_name,
            "--reqtype",
            "ignition.msgs.Pose_V",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(self.timeout_ms),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(1.0, self.timeout_ms / 1000.0 + 0.5),
                env=self.ign_env(),
            )
        except subprocess.TimeoutExpired:
            self.timeout_count += 1
            return
        except OSError as exc:
            self.failure_count += 1
            self.get_logger().warning(f"Gazebo pose sync failed: {exc}")
            return

        if result.returncode != 0:
            self.failure_count += 1
            self.warn_sync_failure(
                "pose vector service rejected request; falling back to per-model set_pose. "
                + (result.stderr.strip() or result.stdout.strip() or f"return code {result.returncode}")
            )
            self.use_vector_service = False
            self.set_model_poses_individually(poses)
            return

        stdout = result.stdout.strip().lower()
        if "data: false" in stdout or stdout == "false":
            self.failure_count += 1
            self.warn_sync_failure("pose vector service returned false; falling back to per-model set_pose")
            self.use_vector_service = False
            self.set_model_poses_individually(poses)
            return

        if not self.logged_connected:
            self.get_logger().info(f"Gazebo pose sync connected to {self.vector_service_name}")
            self.logged_connected = True
        self.success_count += 1

    def set_model_poses_individually(self, poses: Dict[str, Pose2D]) -> None:
        for model_name, pose in poses.items():
            self.set_model_pose(model_name, pose)

    def set_model_pose(self, model_name: str, pose: Pose2D) -> None:
        request = self.pose_request(model_name, pose)
        cmd = [
            self.ign_path,
            "service",
            "-s",
            self.single_service_name,
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
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(1.0, self.timeout_ms / 1000.0 + 0.5),
                env=self.ign_env(),
            )
        except subprocess.TimeoutExpired:
            self.timeout_count += 1
            return
        except OSError as exc:
            self.failure_count += 1
            self.get_logger().warning(f"Gazebo pose sync failed for {model_name}: {exc}")
            return

        if result.returncode != 0:
            self.failure_count += 1
            self.warn_sync_failure(
                result.stderr.strip() or result.stdout.strip() or f"return code {result.returncode}"
            )
            return

        stdout = result.stdout.strip().lower()
        if "data: false" in stdout or stdout == "false":
            self.failure_count += 1
            self.warn_sync_failure(f"set_pose returned false for {model_name}; request was: {request}")
            return

        if not self.logged_connected:
            self.get_logger().info(f"Gazebo pose sync connected to {self.single_service_name}")
            self.logged_connected = True
        self.success_count += 1

    def warn_sync_failure(self, detail: str) -> None:
        if not self.warned_sync_failure:
            self.get_logger().warning(f"Gazebo pose sync rejected request: {detail}")
            self.warned_sync_failure = True
        else:
            self.get_logger().debug(f"Gazebo pose sync rejected request: {detail}")

    def pose_request(self, model_name: str, pose: Pose2D) -> str:
        half_yaw = pose.yaw * 0.5
        qw = math.cos(half_yaw)
        qz = math.sin(half_yaw)
        return (
            f'name: "{model_name}" '
            f'position {{ x: {pose.x:.4f} y: {pose.y:.4f} z: {pose.z:.4f} }} '
            f'orientation {{ w: {qw:.6f} x: 0.0 y: 0.0 z: {qz:.6f} }}'
        )

    def log_status(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_status_log < self.status_log_period:
            return
        self.last_status_log = now
        uav_1 = self.display_poses.get("uav_1")
        uav_1_text = ""
        if uav_1 is not None:
            uav_1_text = f", uav_1=({uav_1.x:.2f}, {uav_1.y:.2f}, {uav_1.z:.2f})"
        self.get_logger().info(
            "Gazebo pose sync status: "
            f"service={self.current_service_name()}, models={sorted(self.display_poses)}, "
            f"success={self.success_count}, failures={self.failure_count}, timeouts={self.timeout_count}"
            f"{uav_1_text}"
        )

    def current_service_name(self) -> str:
        if self.use_vector_service:
            return self.vector_service_name
        return self.single_service_name


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
