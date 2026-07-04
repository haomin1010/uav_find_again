from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from multi_uav_sim_msgs.msg import TrackerState, UavPose2D

from .geometry import norm, unit, wrap_angle
from .ros_utils import point_from_xy, xy_from_point
from .scenario import default_target_waypoints, default_uavs


@dataclass
class UavRuntime:
    uav_id: str
    pos: np.ndarray
    yaw: float
    slot_angle: float
    status: str = "INIT"
    target: Optional[np.ndarray] = None


class UavControl(Node):
    def __init__(self) -> None:
        super().__init__("uav_control")
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("uav_speed", 3.0)
        self.declare_parameter("follow_radius", 9.5)
        self.declare_parameter("world_half_size", 30.0)

        self.dt = float(self.get_parameter("dt").value)
        self.uav_speed = float(self.get_parameter("uav_speed").value)
        self.follow_radius = float(self.get_parameter("follow_radius").value)
        self.world_half_size = float(self.get_parameter("world_half_size").value)
        first_target = default_target_waypoints()[0]

        self.uavs: Dict[str, UavRuntime] = {}
        self.pose_publishers = {}
        for uav_id, init in default_uavs().items():
            pos = np.array([init.x, init.y], dtype=float)
            rel = first_target - pos
            self.uavs[uav_id] = UavRuntime(
                uav_id=uav_id,
                pos=pos,
                yaw=math.atan2(rel[1], rel[0]),
                slot_angle=init.slot_angle,
            )
            self.pose_publishers[uav_id] = self.create_publisher(UavPose2D, f"/{uav_id}/pose2d", 10)
            self.create_subscription(
                TrackerState,
                f"/{uav_id}/tracker_state",
                lambda msg, uid=uav_id: self.on_tracker_state(uid, msg),
                10,
            )

        self.create_timer(self.dt, self.on_timer)

    def on_tracker_state(self, uav_id: str, msg: TrackerState) -> None:
        uav = self.uavs[uav_id]
        uav.status = msg.state
        if msg.state in ("TRACKING", "REACQUIRING", "RECOVERED"):
            uav.target = xy_from_point(msg.current_target_estimate)

    def on_timer(self) -> None:
        for uav in self.uavs.values():
            if uav.target is not None and uav.status != "FAILED":
                radius = self.follow_radius * (0.72 if uav.status == "REACQUIRING" else 1.0)
                offset = np.array([math.cos(uav.slot_angle), math.sin(uav.slot_angle)]) * radius
                desired = uav.target + offset
                delta = desired - uav.pos
                max_step = self.uav_speed * self.dt
                uav.pos += unit(delta) * min(max_step, norm(delta))
                look_delta = uav.target - uav.pos
                if norm(look_delta) > 1e-6:
                    uav.yaw = math.atan2(look_delta[1], look_delta[0])
            elif uav.status == "LOST":
                uav.yaw = wrap_angle(uav.yaw + math.radians(35.0) * self.dt)

            uav.pos[0] = float(np.clip(uav.pos[0], -self.world_half_size, self.world_half_size))
            uav.pos[1] = float(np.clip(uav.pos[1], -self.world_half_size, self.world_half_size))
            self.publish_pose(uav)

    def publish_pose(self, uav: UavRuntime) -> None:
        msg = UavPose2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.uav_id = uav.uav_id
        msg.position = point_from_xy(uav.pos)
        msg.yaw = float(uav.yaw)
        msg.status = uav.status
        self.pose_publishers[uav.uav_id].publish(msg)


def main() -> None:
    rclpy.init()
    node = UavControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

