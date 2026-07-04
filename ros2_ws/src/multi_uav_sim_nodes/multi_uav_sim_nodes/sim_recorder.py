from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from multi_uav_sim_msgs.msg import SimEvent, SwarmTargetEstimate, TrackerState, UavPose2D

from .scenario import default_uav_ids


class SimRecorder(Node):
    def __init__(self) -> None:
        super().__init__("sim_recorder")
        self.declare_parameter("output_dir", "runs_ros2")
        self.declare_parameter("dt", 0.1)

        root = Path(str(self.get_parameter("output_dir").value))
        run_name = datetime.now().strftime("%Y%m%d_%H%M%S_ros2_reacquire")
        self.run_dir = root / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.events_file = (self.run_dir / "events.jsonl").open("w", encoding="utf-8")
        self.tracks_file = (self.run_dir / "tracks.csv").open("w", encoding="utf-8", newline="")
        self.track_writer = csv.writer(self.tracks_file)
        self.track_writer.writerow(
            [
                "time",
                "target_x",
                "target_y",
                "uav_id",
                "uav_x",
                "uav_y",
                "yaw",
                "status",
                "tracker_state",
                "swarm_valid",
                "swarm_x",
                "swarm_y",
            ]
        )

        self.target: Optional[PointStamped] = None
        self.poses: Dict[str, UavPose2D] = {}
        self.states: Dict[str, TrackerState] = {}
        self.swarm: Optional[SwarmTargetEstimate] = None
        self.event_counts: Dict[str, int] = {}
        self.start_time = self.get_clock().now()

        self.create_subscription(PointStamped, "/target/ground_truth", self.on_target, 10)
        self.create_subscription(SwarmTargetEstimate, "/swarm/target_estimate", self.on_swarm, 10)
        self.create_subscription(SimEvent, "/swarm/events", self.on_event, 10)
        for uav_id in default_uav_ids():
            self.create_subscription(UavPose2D, f"/{uav_id}/pose2d", self.on_pose, 10)
            self.create_subscription(TrackerState, f"/{uav_id}/tracker_state", self.on_state, 10)

        self.create_timer(float(self.get_parameter("dt").value), self.on_timer)
        self.get_logger().info(f"recording ROS2 demo to {self.run_dir}")

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def on_target(self, msg: PointStamped) -> None:
        self.target = msg

    def on_swarm(self, msg: SwarmTargetEstimate) -> None:
        self.swarm = msg

    def on_pose(self, msg: UavPose2D) -> None:
        self.poses[msg.uav_id] = msg

    def on_state(self, msg: TrackerState) -> None:
        self.states[msg.uav_id] = msg

    def on_event(self, msg: SimEvent) -> None:
        event = {
            "time": round(self.elapsed_seconds(), 3),
            "event_type": msg.event_type,
            "uav_id": msg.uav_id,
            "description": msg.description,
        }
        self.event_counts[msg.event_type] = self.event_counts.get(msg.event_type, 0) + 1
        self.events_file.write(json.dumps(event, ensure_ascii=False) + "\n")
        self.events_file.flush()

    def on_timer(self) -> None:
        if self.target is None:
            return
        swarm_valid = self.swarm.valid if self.swarm is not None else False
        swarm_x = self.swarm.position.x if self.swarm is not None and self.swarm.valid else ""
        swarm_y = self.swarm.position.y if self.swarm is not None and self.swarm.valid else ""

        for uav_id, pose in self.poses.items():
            state = self.states.get(uav_id)
            self.track_writer.writerow(
                [
                    f"{self.elapsed_seconds():.3f}",
                    f"{self.target.point.x:.4f}",
                    f"{self.target.point.y:.4f}",
                    uav_id,
                    f"{pose.position.x:.4f}",
                    f"{pose.position.y:.4f}",
                    f"{pose.yaw:.4f}",
                    pose.status,
                    state.state if state is not None else "",
                    int(swarm_valid),
                    swarm_x,
                    swarm_y,
                ]
            )
        self.tracks_file.flush()

    def destroy_node(self) -> bool:
        metrics = {
            "event_counts": self.event_counts,
            "run_dir": str(self.run_dir),
        }
        (self.run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        self.events_file.close()
        self.tracks_file.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

