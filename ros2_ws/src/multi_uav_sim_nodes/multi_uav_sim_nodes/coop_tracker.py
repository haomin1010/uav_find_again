from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from multi_uav_sim_msgs.msg import SimEvent, SwarmTargetEstimate, TrackerState, VisionObservation

from .ros_utils import point_from_xy, xy_from_point
from .scenario import default_uav_ids


@dataclass
class LocalTrack:
    uav_id: str
    state: str = "INIT"
    lost_frames: int = 0
    recovered_frames: int = 0
    reacquire_started_at: Optional[float] = None
    last_estimate: Optional[np.ndarray] = None
    target_for_control: Optional[np.ndarray] = None
    reason: str = ""
    local_detected: bool = False
    local_confidence: float = 0.0


class CoopTracker(Node):
    def __init__(self) -> None:
        super().__init__("coop_tracker")
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("lost_frame_threshold", 8)
        self.declare_parameter("recovered_frame_threshold", 6)
        self.declare_parameter("observation_timeout_sec", 0.7)
        self.declare_parameter("max_recovery_time_sec", 18.0)

        self.dt = float(self.get_parameter("dt").value)
        self.lost_frame_threshold = int(self.get_parameter("lost_frame_threshold").value)
        self.recovered_frame_threshold = int(self.get_parameter("recovered_frame_threshold").value)
        self.observation_timeout_sec = float(self.get_parameter("observation_timeout_sec").value)
        self.max_recovery_time_sec = float(self.get_parameter("max_recovery_time_sec").value)

        self.tracks = {uav_id: LocalTrack(uav_id) for uav_id in default_uav_ids()}
        self.latest_observations: Dict[str, Tuple[float, VisionObservation]] = {}
        self.current_observations: Dict[str, VisionObservation] = {}

        self.state_publishers = {
            uav_id: self.create_publisher(TrackerState, f"/{uav_id}/tracker_state", 10)
            for uav_id in default_uav_ids()
        }
        self.swarm_pub = self.create_publisher(SwarmTargetEstimate, "/swarm/target_estimate", 10)
        self.event_pub = self.create_publisher(SimEvent, "/swarm/events", 10)

        for uav_id in default_uav_ids():
            self.create_subscription(
                VisionObservation,
                f"/{uav_id}/vision_observation",
                lambda msg, uid=uav_id: self.on_observation(uid, msg),
                10,
            )
        self.start_time = self.get_clock().now()
        self.create_timer(self.dt, self.on_timer)

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def on_observation(self, uav_id: str, msg: VisionObservation) -> None:
        self.current_observations[uav_id] = msg
        if msg.detected:
            self.latest_observations[uav_id] = (self.elapsed_seconds(), msg)

    def fuse_swarm_estimate(self) -> Tuple[Optional[np.ndarray], List[str], float]:
        now = self.elapsed_seconds()
        weighted = []
        weights = []
        contributors = []
        for uav_id, (stamp, obs) in self.latest_observations.items():
            if now - stamp > self.observation_timeout_sec:
                continue
            weight = max(0.01, float(obs.confidence)) / (1.0 + 0.4 * (now - stamp))
            estimate = xy_from_point(obs.target_position_estimate)
            weighted.append(estimate * weight)
            weights.append(weight)
            contributors.append(uav_id)

        if not weights:
            return None, [], 0.0
        estimate = np.sum(weighted, axis=0) / float(np.sum(weights))
        confidence = float(np.clip(np.mean(weights), 0.0, 1.0))
        return estimate, contributors, confidence

    def on_timer(self) -> None:
        swarm_estimate, contributors, swarm_confidence = self.fuse_swarm_estimate()
        self.publish_swarm_estimate(swarm_estimate, contributors, swarm_confidence)

        for uav_id, track in self.tracks.items():
            obs = self.current_observations.get(uav_id)
            if obs is None:
                continue
            self.update_track(track, obs, swarm_estimate)
            self.publish_tracker_state(track, swarm_estimate is not None)

    def update_track(
        self,
        track: LocalTrack,
        obs: VisionObservation,
        swarm_estimate: Optional[np.ndarray],
    ) -> None:
        now = self.elapsed_seconds()
        track.local_detected = bool(obs.detected)
        track.local_confidence = float(obs.confidence)

        if obs.detected:
            estimate = xy_from_point(obs.target_position_estimate)
            track.last_estimate = estimate.copy()
            track.target_for_control = estimate.copy()
            track.lost_frames = 0
            track.reason = "VISIBLE"

            if track.state in ("INIT", "TRACKING"):
                track.state = "TRACKING"
            elif track.state in ("LOST", "REACQUIRING"):
                track.state = "RECOVERED"
                track.recovered_frames = 1
                self.publish_event("TARGET_RECOVERED", track.uav_id, "local vision reacquired target")
            elif track.state == "RECOVERED":
                track.recovered_frames += 1
                if track.recovered_frames >= self.recovered_frame_threshold:
                    track.state = "TRACKING"
                    track.recovered_frames = 0
            return

        track.lost_frames += 1
        track.reason = obs.loss_reason
        if track.state in ("INIT", "TRACKING", "RECOVERED") and track.lost_frames >= self.lost_frame_threshold:
            track.state = "LOST"
            track.reacquire_started_at = None
            self.publish_event("TARGET_LOST", track.uav_id, obs.loss_reason)

        if track.state == "LOST":
            if swarm_estimate is not None:
                track.state = "REACQUIRING"
                track.reacquire_started_at = now
                track.target_for_control = swarm_estimate.copy()
                self.publish_event("COOP_REACQUIRE_START", track.uav_id, "using swarm target estimate")
            elif track.last_estimate is not None:
                track.target_for_control = track.last_estimate.copy()
        elif track.state == "REACQUIRING":
            if swarm_estimate is not None:
                track.target_for_control = swarm_estimate.copy()
            if track.reacquire_started_at is not None and now - track.reacquire_started_at > self.max_recovery_time_sec:
                track.state = "FAILED"
                self.publish_event("RECOVERY_FAILED", track.uav_id, "recovery timeout")

    def publish_swarm_estimate(
        self,
        estimate: Optional[np.ndarray],
        contributors: List[str],
        confidence: float,
    ) -> None:
        msg = SwarmTargetEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.valid = estimate is not None
        msg.contributing_uav_ids = contributors
        msg.confidence = float(confidence)
        msg.covariance = [0.0] * 9
        if estimate is not None:
            msg.position = point_from_xy(estimate)
        self.swarm_pub.publish(msg)

    def publish_tracker_state(self, track: LocalTrack, coop_available: bool) -> None:
        msg = TrackerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.uav_id = track.uav_id
        msg.state = track.state
        msg.local_detected = track.local_detected
        msg.coop_available = bool(coop_available)
        msg.local_confidence = float(track.local_confidence)
        msg.uncertainty_radius = 2.0 if track.state != "REACQUIRING" else 5.0
        msg.reason = track.reason
        if track.target_for_control is not None:
            msg.current_target_estimate = point_from_xy(track.target_for_control)
        self.state_publishers[track.uav_id].publish(msg)

    def publish_event(self, event_type: str, uav_id: str, description: str) -> None:
        msg = SimEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.event_type = event_type
        msg.uav_id = uav_id
        msg.description = description
        self.event_pub.publish(msg)
        self.get_logger().info(f"{event_type} {uav_id}: {description}")


def main() -> None:
    rclpy.init()
    node = CoopTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

