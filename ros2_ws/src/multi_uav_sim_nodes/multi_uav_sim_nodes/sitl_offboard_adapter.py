from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from multi_uav_sim_msgs.msg import UavPose2D

from .geometry import norm, unit, wrap_angle
from .ros_utils import point_from_xyz
from .scenario import default_target_waypoints, default_uavs

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleAttitude,
        VehicleCommand,
        VehicleLocalPosition,
    )

    PX4_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - exercised only without px4_msgs.
    OffboardControlMode = None  # type: ignore[assignment]
    TrajectorySetpoint = None  # type: ignore[assignment]
    VehicleAttitude = None  # type: ignore[assignment]
    VehicleCommand = None  # type: ignore[assignment]
    VehicleLocalPosition = None  # type: ignore[assignment]
    PX4_IMPORT_ERROR = exc


@dataclass
class EntityRuntime:
    entity_id: str
    namespace: str
    target_system: int
    origin_enu: np.ndarray
    desired_enu: np.ndarray
    command_enu: np.ndarray
    desired_yaw_enu: float
    command_yaw_enu: float
    current_enu: Optional[np.ndarray] = None
    current_yaw_enu: Optional[float] = None
    previous_desired_enu: Optional[np.ndarray] = None
    status: str = "SITL"


def enu_to_ned_position(position: np.ndarray) -> list[float]:
    return [float(position[1]), float(position[0]), float(-position[2])]


def ned_to_enu_position(x: float, y: float, z: float) -> np.ndarray:
    return np.array([float(y), float(x), float(-z)], dtype=float)


def enu_yaw_to_ned_yaw(yaw: float) -> float:
    return wrap_angle(math.pi / 2.0 - yaw)


def ned_yaw_to_enu_yaw(yaw: float) -> float:
    return wrap_angle(math.pi / 2.0 - yaw)


def yaw_from_px4_quaternion(q: list[float]) -> float:
    if len(q) < 4:
        return 0.0
    w, x, y, z = q[:4]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def px4_topic(namespace: str, suffix: str) -> str:
    return f"/{namespace.strip('/')}/fmu/{suffix}"


class SitlOffboardAdapter(Node):
    """Bridge project-level desired poses to PX4 ROS2 Offboard setpoints."""

    def __init__(self) -> None:
        super().__init__("sitl_offboard_adapter")
        if PX4_IMPORT_ERROR is not None:
            raise RuntimeError(
                "px4_msgs is required for sitl_offboard_adapter. "
                "Build/source PX4 px4_msgs in this ROS2 workspace first."
            ) from PX4_IMPORT_ERROR

        self.declare_parameter("uav_ids", ["uav_1", "uav_2", "uav_3"])
        self.declare_parameter("px4_namespaces", ["px4_1", "px4_2", "px4_3"])
        self.declare_parameter("target_system_ids", [1, 2, 3])
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("hold_altitude_m", 8.0)
        self.declare_parameter("target_hold_altitude_m", 7.0)
        self.declare_parameter("max_xy_speed_mps", 2.5)
        self.declare_parameter("target_max_xy_speed_mps", 1.8)
        self.declare_parameter("max_z_speed_mps", 1.0)
        self.declare_parameter("max_yaw_rate_dps", 60.0)
        self.declare_parameter("warmup_sec", 1.0)
        self.declare_parameter("command_retry_period_sec", 2.0)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("target_enabled", True)
        self.declare_parameter("target_px4_namespace", "px4_4")
        self.declare_parameter("target_system_id", 4)
        self.declare_parameter("desired_pose_suffix", "desired_pose2d")
        self.declare_parameter("feedback_pose_suffix", "pose2d")
        self.declare_parameter("target_desired_topic", "/target/desired_position")
        self.declare_parameter("target_feedback_topic", "/target/ground_truth")

        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.command_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.hold_altitude_m = float(self.get_parameter("hold_altitude_m").value)
        self.target_hold_altitude_m = float(self.get_parameter("target_hold_altitude_m").value)
        self.max_xy_speed_mps = float(self.get_parameter("max_xy_speed_mps").value)
        self.target_max_xy_speed_mps = float(self.get_parameter("target_max_xy_speed_mps").value)
        self.max_z_speed_mps = float(self.get_parameter("max_z_speed_mps").value)
        self.max_yaw_rate = math.radians(float(self.get_parameter("max_yaw_rate_dps").value))
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.command_retry_period_sec = float(self.get_parameter("command_retry_period_sec").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.auto_offboard = bool(self.get_parameter("auto_offboard").value)
        self.target_enabled = bool(self.get_parameter("target_enabled").value)
        self.desired_pose_suffix = str(self.get_parameter("desired_pose_suffix").value).strip("/")
        self.feedback_pose_suffix = str(self.get_parameter("feedback_pose_suffix").value).strip("/")
        self.target_desired_topic = str(self.get_parameter("target_desired_topic").value)
        self.target_feedback_topic = str(self.get_parameter("target_feedback_topic").value)

        self.entities: Dict[str, EntityRuntime] = {}
        self.offboard_publishers = {}
        self.setpoint_publishers = {}
        self.command_publishers = {}
        self.uav_pose_publishers = {}
        self.target_pose_publisher = self.create_publisher(PointStamped, self.target_feedback_topic, 10)

        self.start_time = self.get_clock().now()
        self.last_vehicle_command_time = -self.command_retry_period_sec

        self.configure_uavs()
        if self.target_enabled:
            self.configure_target()

        self.create_timer(self.command_period, self.on_timer)
        self.get_logger().info(
            "SITL offboard adapter started: "
            f"entities={list(self.entities)}, rate={self.command_rate_hz:.1f}Hz, "
            f"uav_altitude={self.hold_altitude_m:.1f}m, target_altitude={self.target_hold_altitude_m:.1f}m"
        )

    def configure_uavs(self) -> None:
        uav_ids = list(self.get_parameter("uav_ids").value)
        namespaces = list(self.get_parameter("px4_namespaces").value)
        target_system_ids = [int(v) for v in self.get_parameter("target_system_ids").value]
        default_init = default_uavs()

        for index, uav_id in enumerate(uav_ids):
            namespace = namespaces[index]
            target_system = target_system_ids[index] if index < len(target_system_ids) else index + 1
            init = default_init[uav_id]
            position = np.array([init.x, init.y, self.hold_altitude_m], dtype=float)
            entity = EntityRuntime(
                entity_id=uav_id,
                namespace=namespace,
                target_system=target_system,
                origin_enu=np.array([init.x, init.y, 0.0], dtype=float),
                desired_enu=position.copy(),
                command_enu=position.copy(),
                desired_yaw_enu=float(init.yaw),
                command_yaw_enu=float(init.yaw),
            )
            self.add_px4_io(entity)
            self.entities[uav_id] = entity
            self.create_subscription(
                UavPose2D,
                f"/{uav_id}/{self.desired_pose_suffix}",
                lambda msg, eid=uav_id: self.on_uav_desired(eid, msg),
                10,
            )
            self.uav_pose_publishers[uav_id] = self.create_publisher(
                UavPose2D, f"/{uav_id}/{self.feedback_pose_suffix}", 10
            )

    def configure_target(self) -> None:
        target_start = default_target_waypoints()[0]
        position = np.array(
            [target_start[0], target_start[1], self.target_hold_altitude_m],
            dtype=float,
        )
        entity = EntityRuntime(
            entity_id="target",
            namespace=str(self.get_parameter("target_px4_namespace").value),
            target_system=int(self.get_parameter("target_system_id").value),
            origin_enu=np.array([target_start[0], target_start[1], 0.0], dtype=float),
            desired_enu=position.copy(),
            command_enu=position.copy(),
            desired_yaw_enu=0.0,
            command_yaw_enu=0.0,
        )
        self.add_px4_io(entity)
        self.entities["target"] = entity
        self.create_subscription(PointStamped, self.target_desired_topic, self.on_target_desired, 10)

    def add_px4_io(self, entity: EntityRuntime) -> None:
        self.offboard_publishers[entity.entity_id] = self.create_publisher(
            OffboardControlMode, px4_topic(entity.namespace, "in/offboard_control_mode"), 10
        )
        self.setpoint_publishers[entity.entity_id] = self.create_publisher(
            TrajectorySetpoint, px4_topic(entity.namespace, "in/trajectory_setpoint"), 10
        )
        self.command_publishers[entity.entity_id] = self.create_publisher(
            VehicleCommand, px4_topic(entity.namespace, "in/vehicle_command"), 10
        )
        self.create_subscription(
            VehicleLocalPosition,
            px4_topic(entity.namespace, "out/vehicle_local_position"),
            lambda msg, eid=entity.entity_id: self.on_local_position(eid, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleAttitude,
            px4_topic(entity.namespace, "out/vehicle_attitude"),
            lambda msg, eid=entity.entity_id: self.on_attitude(eid, msg),
            qos_profile_sensor_data,
        )

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_uav_desired(self, entity_id: str, msg: UavPose2D) -> None:
        entity = self.entities[entity_id]
        entity.desired_enu = np.array([msg.position.x, msg.position.y, self.hold_altitude_m], dtype=float)
        entity.desired_yaw_enu = float(msg.yaw)
        entity.status = msg.status or "SITL"

    def on_target_desired(self, msg: PointStamped) -> None:
        entity = self.entities["target"]
        desired = np.array([msg.point.x, msg.point.y, self.target_hold_altitude_m], dtype=float)
        if entity.previous_desired_enu is not None:
            delta = desired[:2] - entity.previous_desired_enu[:2]
            if norm(delta) > 0.03:
                entity.desired_yaw_enu = math.atan2(delta[1], delta[0])
        entity.previous_desired_enu = desired.copy()
        entity.desired_enu = desired

    def on_local_position(self, entity_id: str, msg: VehicleLocalPosition) -> None:
        entity = self.entities[entity_id]
        entity.current_enu = entity.origin_enu + ned_to_enu_position(msg.x, msg.y, msg.z)
        if hasattr(msg, "heading") and math.isfinite(float(msg.heading)):
            entity.current_yaw_enu = ned_yaw_to_enu_yaw(float(msg.heading))
        self.publish_feedback(entity)

    def on_attitude(self, entity_id: str, msg: VehicleAttitude) -> None:
        entity = self.entities[entity_id]
        yaw_ned = yaw_from_px4_quaternion(list(msg.q))
        entity.current_yaw_enu = ned_yaw_to_enu_yaw(yaw_ned)

    def on_timer(self) -> None:
        now = self.elapsed_seconds()
        for entity in self.entities.values():
            self.advance_command(entity)
            self.publish_offboard_control_mode(entity)
            self.publish_trajectory_setpoint(entity)

        if now >= self.warmup_sec and now - self.last_vehicle_command_time >= self.command_retry_period_sec:
            for entity in self.entities.values():
                if self.auto_offboard:
                    self.publish_offboard_command(entity)
                if self.auto_arm:
                    self.publish_arm_command(entity)
            self.last_vehicle_command_time = now

    def advance_command(self, entity: EntityRuntime) -> None:
        max_xy_speed = self.target_max_xy_speed_mps if entity.entity_id == "target" else self.max_xy_speed_mps
        delta_xy = entity.desired_enu[:2] - entity.command_enu[:2]
        max_xy_step = max_xy_speed * self.command_period
        if norm(delta_xy) > max_xy_step:
            entity.command_enu[:2] += unit(delta_xy) * max_xy_step
        else:
            entity.command_enu[:2] = entity.desired_enu[:2]

        delta_z = entity.desired_enu[2] - entity.command_enu[2]
        max_z_step = self.max_z_speed_mps * self.command_period
        entity.command_enu[2] += float(np.clip(delta_z, -max_z_step, max_z_step))

        yaw_delta = wrap_angle(entity.desired_yaw_enu - entity.command_yaw_enu)
        max_yaw_step = self.max_yaw_rate * self.command_period
        entity.command_yaw_enu = wrap_angle(
            entity.command_yaw_enu + float(np.clip(yaw_delta, -max_yaw_step, max_yaw_step))
        )

    def publish_offboard_control_mode(self, entity: EntityRuntime) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False
        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False
        self.offboard_publishers[entity.entity_id].publish(msg)

    def publish_trajectory_setpoint(self, entity: EntityRuntime) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = enu_to_ned_position(entity.command_enu - entity.origin_enu)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(enu_yaw_to_ned_yaw(entity.command_yaw_enu))
        msg.yawspeed = math.nan
        self.setpoint_publishers[entity.entity_id].publish(msg)

    def publish_offboard_command(self, entity: EntityRuntime) -> None:
        self.publish_vehicle_command(
            entity,
            getattr(VehicleCommand, "VEHICLE_CMD_DO_SET_MODE", 176),
            param1=1.0,
            param2=6.0,
        )

    def publish_arm_command(self, entity: EntityRuntime) -> None:
        self.publish_vehicle_command(
            entity,
            getattr(VehicleCommand, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400),
            param1=1.0,
        )

    def publish_vehicle_command(
        self,
        entity: EntityRuntime,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = int(entity.target_system)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_publishers[entity.entity_id].publish(msg)

    def publish_feedback(self, entity: EntityRuntime) -> None:
        if entity.current_enu is None:
            return
        yaw = entity.current_yaw_enu if entity.current_yaw_enu is not None else entity.command_yaw_enu
        stamp = self.get_clock().now().to_msg()
        if entity.entity_id == "target":
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "world"
            msg.point = point_from_xyz(entity.current_enu)
            self.target_pose_publisher.publish(msg)
            return

        msg = UavPose2D()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.uav_id = entity.entity_id
        msg.position = point_from_xyz(entity.current_enu)
        msg.yaw = float(yaw)
        msg.status = entity.status
        self.uav_pose_publishers[entity.entity_id].publish(msg)


def main() -> None:
    rclpy.init()
    node = SitlOffboardAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
