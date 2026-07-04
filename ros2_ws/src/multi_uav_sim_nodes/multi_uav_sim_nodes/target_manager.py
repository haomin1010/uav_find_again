from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from .geometry import norm, unit
from .ros_utils import point_from_xy
from .scenario import default_target_waypoints


class TargetManager(Node):
    def __init__(self) -> None:
        super().__init__("target_manager")
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("target_speed", 1.45)

        self.dt = float(self.get_parameter("dt").value)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.waypoints = default_target_waypoints()
        self.position = self.waypoints[0].copy()
        self.waypoint_index = 1
        self.publisher = self.create_publisher(PointStamped, "/target/ground_truth", 10)
        self.create_timer(self.dt, self.on_timer)

    def on_timer(self) -> None:
        waypoint = self.waypoints[self.waypoint_index]
        delta = waypoint - self.position
        step = self.target_speed * self.dt
        if norm(delta) <= step:
            self.position = waypoint.copy()
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
        else:
            self.position += unit(delta) * step

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.point = point_from_xy(self.position)
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = TargetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

