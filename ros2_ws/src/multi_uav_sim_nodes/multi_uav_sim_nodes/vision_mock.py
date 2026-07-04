from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from multi_uav_sim_msgs.msg import UavPose2D, VisionObservation

from .geometry import is_occluded, norm, wrap_angle
from .ros_utils import point_from_xy, xy_from_point
from .scenario import default_obstacles, default_uav_ids


class VisionMock(Node):
    def __init__(self) -> None:
        super().__init__("vision_mock")
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("camera_range", 34.0)
        self.declare_parameter("camera_fov_deg", 78.0)
        self.declare_parameter("observation_noise_std", 0.45)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("seed", 7)

        self.dt = float(self.get_parameter("dt").value)
        self.camera_range = float(self.get_parameter("camera_range").value)
        self.camera_fov = math.radians(float(self.get_parameter("camera_fov_deg").value))
        self.noise_std = float(self.get_parameter("observation_noise_std").value)
        self.dropout_probability = float(self.get_parameter("dropout_probability").value)
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))

        self.obstacles = default_obstacles()
        self.target: Optional[np.ndarray] = None
        self.poses: Dict[str, UavPose2D] = {}
        self.publishers = {
            uav_id: self.create_publisher(VisionObservation, f"/{uav_id}/vision_observation", 10)
            for uav_id in default_uav_ids()
        }

        self.create_subscription(PointStamped, "/target/ground_truth", self.on_target, 10)
        for uav_id in default_uav_ids():
            self.create_subscription(UavPose2D, f"/{uav_id}/pose2d", self.on_pose, 10)
        self.start_time = self.get_clock().now()
        self.create_timer(self.dt, self.on_timer)

    def on_target(self, msg: PointStamped) -> None:
        self.target = xy_from_point(msg.point)

    def on_pose(self, msg: UavPose2D) -> None:
        self.poses[msg.uav_id] = msg

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def on_timer(self) -> None:
        if self.target is None:
            return

        for uav_id, pose_msg in self.poses.items():
            if uav_id not in self.publishers:
                continue
            observer = xy_from_point(pose_msg.position)
            rel = self.target - observer
            distance = norm(rel)
            bearing_world = math.atan2(rel[1], rel[0])
            bearing = wrap_angle(bearing_world - pose_msg.yaw)

            in_range = distance <= self.camera_range
            in_fov = abs(bearing) <= self.camera_fov / 2.0
            occluded = is_occluded(observer, self.target, self.obstacles)
            forced_loss = uav_id == "uav_1" and 18.0 <= self.elapsed_seconds() <= 28.0
            random_dropout = self.rng.random() < self.dropout_probability
            detected = in_range and in_fov and not occluded and not forced_loss and not random_dropout

            loss_reason = "VISIBLE"
            if not in_range:
                loss_reason = "TOO_FAR"
            elif not in_fov:
                loss_reason = "OUT_OF_FOV"
            elif occluded or forced_loss:
                loss_reason = "OCCLUDED"
            elif random_dropout:
                loss_reason = "RANDOM_DROPOUT"

            msg = VisionObservation()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.observer_id = uav_id
            msg.detected = bool(detected)
            msg.loss_reason = loss_reason
            msg.observer_position = pose_msg.position
            msg.observer_yaw = float(pose_msg.yaw)
            msg.bearing = float(bearing)
            msg.elevation = 0.0
            msg.range_estimate = float(distance)
            msg.target_position_covariance = [0.0] * 9

            if detected:
                range_score = max(0.0, 1.0 - distance / (self.camera_range * 1.2))
                center_score = max(0.0, 1.0 - abs(bearing) / (self.camera_fov / 2.0))
                msg.confidence = float(np.clip(0.55 + 0.25 * range_score + 0.2 * center_score, 0.0, 0.99))
                estimate = self.target + self.rng.normal(0.0, self.noise_std, size=2)
                msg.target_position_estimate = point_from_xy(estimate)
                msg.target_position_covariance = [
                    self.noise_std**2,
                    0.0,
                    0.0,
                    0.0,
                    self.noise_std**2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]
                msg.bbox_cx = float(0.5 + 0.5 * bearing / (self.camera_fov / 2.0))
                msg.bbox_cy = 0.5
                msg.bbox_w = float(np.clip(0.5 / max(distance, 1.0), 0.02, 0.25))
                msg.bbox_h = msg.bbox_w * 1.4
            else:
                msg.confidence = 0.0
                msg.target_position_estimate = point_from_xy(np.array([0.0, 0.0]))

            self.publishers[uav_id].publish(msg)


def main() -> None:
    rclpy.init()
    node = VisionMock()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

