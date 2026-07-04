from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon, Rectangle


Vec2 = np.ndarray


STATUS_COLORS = {
    "INIT": "#6b7280",
    "TRACKING": "#16a34a",
    "LOST": "#dc2626",
    "REACQUIRING": "#f59e0b",
    "RECOVERED": "#2563eb",
    "FAILED": "#7f1d1d",
}


@dataclass
class RectObstacle:
    name: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def corners(self) -> List[Vec2]:
        return [
            np.array([self.x_min, self.y_min]),
            np.array([self.x_max, self.y_min]),
            np.array([self.x_max, self.y_max]),
            np.array([self.x_min, self.y_max]),
        ]

    def edges(self) -> Iterable[Tuple[Vec2, Vec2]]:
        c = self.corners()
        for i in range(4):
            yield c[i], c[(i + 1) % 4]


@dataclass
class VisionObservation:
    observer_id: str
    detected: bool
    confidence: float
    loss_reason: str
    target_estimate: Optional[Vec2]
    bearing: float
    range_estimate: float


@dataclass
class UavState:
    uav_id: str
    pos: Vec2
    yaw: float
    slot_angle: float
    status: str = "INIT"
    last_local_estimate: Optional[Vec2] = None
    lost_frames: int = 0
    reacquire_started_at: Optional[float] = None
    recovered_frames: int = 0
    local_detected: bool = False
    target_for_control: Optional[Vec2] = None
    status_reason: str = ""


@dataclass
class SimConfig:
    duration: float = 36.0
    dt: float = 0.1
    render_fps: int = 20
    world_half_size: float = 30.0
    camera_range: float = 34.0
    camera_fov_deg: float = 78.0
    target_speed: float = 1.45
    uav_speed: float = 3.0
    follow_radius: float = 9.5
    lost_frame_threshold: int = 8
    recovered_frame_threshold: int = 6
    observation_noise_std: float = 0.45
    random_dropout_probability: float = 0.0
    observation_timeout: float = 0.7
    max_recovery_time: float = 18.0
    seed: int = 7


@dataclass
class FrameRecord:
    t: float
    target: Vec2
    uavs: Dict[str, Tuple[Vec2, float, str, bool]]
    observations: Dict[str, VisionObservation]
    swarm_estimate: Optional[Vec2]


@dataclass
class Metrics:
    lost_events: int = 0
    recovery_successes: int = 0
    recovery_failures: int = 0
    recovery_times: List[float] = field(default_factory=list)


def norm(vec: Vec2) -> float:
    return float(np.linalg.norm(vec))


def unit(vec: Vec2) -> Vec2:
    n = norm(vec)
    if n < 1e-9:
        return np.zeros(2)
    return vec / n


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def segment_intersects(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool:
    def orient(p: Vec2, q: Vec2, r: Vec2) -> float:
        return float((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))

    def on_segment(p: Vec2, q: Vec2, r: Vec2) -> bool:
        return (
            min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
            and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
        )

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    eps = 1e-9
    return (
        abs(o1) < eps
        and on_segment(a, c, b)
        or abs(o2) < eps
        and on_segment(a, d, b)
        or abs(o3) < eps
        and on_segment(c, a, d)
        or abs(o4) < eps
        and on_segment(c, b, d)
    )


def is_occluded(observer: Vec2, target: Vec2, obstacles: List[RectObstacle]) -> bool:
    for obs in obstacles:
        for edge_a, edge_b in obs.edges():
            if segment_intersects(observer, target, edge_a, edge_b):
                return True
    return False


class MockReacquisitionSim:
    def __init__(self, config: SimConfig):
        self.cfg = config
        self.rng = np.random.default_rng(config.seed)
        self.events: List[dict] = []
        self.metrics = Metrics()
        self.latest_observations: Dict[str, Tuple[float, VisionObservation]] = {}

        self.obstacles = [
            RectObstacle("scripted_occluder", -4.0, 20.0, 4.0, 26.0),
            RectObstacle("pillar_a", -25.0, 15.0, -20.0, 22.0),
            RectObstacle("pillar_b", 20.0, -24.0, 25.0, -17.0),
        ]
        self.target_waypoints = [
            np.array([-14.0, -7.0]),
            np.array([-5.0, -5.0]),
            np.array([8.0, 2.0]),
            np.array([20.0, 8.0]),
            np.array([7.0, 18.0]),
            np.array([-16.0, 13.0]),
        ]
        self.target = self.target_waypoints[0].copy()
        self.target_wp_idx = 1
        self.uavs: Dict[str, UavState] = {
            "uav_1": UavState("uav_1", np.array([-22.0, -11.0]), 0.0, math.radians(210)),
            "uav_2": UavState("uav_2", np.array([-14.0, -18.0]), 0.0, math.radians(270)),
            "uav_3": UavState("uav_3", np.array([-5.0, -9.0]), 0.0, math.radians(330)),
        }
        for uav in self.uavs.values():
            rel = self.target - uav.pos
            uav.yaw = math.atan2(rel[1], rel[0])

    def run(self) -> List[FrameRecord]:
        frames: List[FrameRecord] = []
        steps = int(self.cfg.duration / self.cfg.dt)
        for step in range(steps):
            t = step * self.cfg.dt
            self._update_target()
            observations = self._observe(t)
            swarm_estimate = self._fuse_swarm_estimate(t)
            self._update_trackers(t, observations, swarm_estimate)
            self._update_controls(t)
            frames.append(
                FrameRecord(
                    t=t,
                    target=self.target.copy(),
                    uavs={u.uav_id: (u.pos.copy(), u.yaw, u.status, u.local_detected) for u in self.uavs.values()},
                    observations=observations,
                    swarm_estimate=swarm_estimate.copy() if swarm_estimate is not None else None,
                )
            )
        return frames

    def _update_target(self) -> None:
        waypoint = self.target_waypoints[self.target_wp_idx]
        delta = waypoint - self.target
        step = self.cfg.target_speed * self.cfg.dt
        if norm(delta) <= step:
            self.target = waypoint.copy()
            self.target_wp_idx = (self.target_wp_idx + 1) % len(self.target_waypoints)
        else:
            self.target += unit(delta) * step

    def _observe(self, t: float) -> Dict[str, VisionObservation]:
        observations = {}
        for uav in self.uavs.values():
            rel = self.target - uav.pos
            distance = norm(rel)
            bearing_world = math.atan2(rel[1], rel[0])
            bearing = wrap_angle(bearing_world - uav.yaw)
            in_range = distance <= self.cfg.camera_range
            in_fov = abs(bearing) <= math.radians(self.cfg.camera_fov_deg) / 2.0
            occluded = is_occluded(uav.pos, self.target, self.obstacles)
            random_dropout = self.rng.random() < self.cfg.random_dropout_probability

            forced_loss = uav.uav_id == "uav_1" and 18.0 <= t <= 28.0
            detected = in_range and in_fov and not occluded and not random_dropout and not forced_loss
            loss_reason = "VISIBLE"
            if not in_range:
                loss_reason = "TOO_FAR"
            elif not in_fov:
                loss_reason = "OUT_OF_FOV"
            elif occluded or forced_loss:
                loss_reason = "OCCLUDED"
            elif random_dropout:
                loss_reason = "RANDOM_DROPOUT"

            confidence = 0.0
            target_estimate = None
            if detected:
                range_score = max(0.0, 1.0 - distance / (self.cfg.camera_range * 1.2))
                center_score = max(0.0, 1.0 - abs(bearing) / (math.radians(self.cfg.camera_fov_deg) / 2.0))
                confidence = float(np.clip(0.55 + 0.25 * range_score + 0.2 * center_score, 0.0, 0.99))
                noise = self.rng.normal(0.0, self.cfg.observation_noise_std, size=2)
                target_estimate = self.target + noise
                self.latest_observations[uav.uav_id] = (
                    t,
                    VisionObservation(
                        observer_id=uav.uav_id,
                        detected=True,
                        confidence=confidence,
                        loss_reason=loss_reason,
                        target_estimate=target_estimate.copy(),
                        bearing=bearing,
                        range_estimate=distance + float(self.rng.normal(0.0, 0.25)),
                    ),
                )

            obs = VisionObservation(
                observer_id=uav.uav_id,
                detected=detected,
                confidence=confidence,
                loss_reason=loss_reason,
                target_estimate=target_estimate.copy() if target_estimate is not None else None,
                bearing=bearing,
                range_estimate=distance,
            )
            observations[uav.uav_id] = obs
        return observations

    def _fuse_swarm_estimate(self, t: float) -> Optional[Vec2]:
        weighted = []
        weights = []
        for _, (stamp, obs) in self.latest_observations.items():
            if t - stamp > self.cfg.observation_timeout or obs.target_estimate is None:
                continue
            weight = max(0.01, obs.confidence) / (1.0 + 0.4 * (t - stamp))
            weighted.append(obs.target_estimate * weight)
            weights.append(weight)
        if not weighted:
            return None
        return np.sum(weighted, axis=0) / float(np.sum(weights))

    def _update_trackers(
        self,
        t: float,
        observations: Dict[str, VisionObservation],
        swarm_estimate: Optional[Vec2],
    ) -> None:
        for uav in self.uavs.values():
            obs = observations[uav.uav_id]
            uav.local_detected = obs.detected

            if obs.detected and obs.target_estimate is not None:
                uav.last_local_estimate = obs.target_estimate.copy()
                uav.target_for_control = obs.target_estimate.copy()
                uav.lost_frames = 0
                if uav.status in ("INIT", "TRACKING"):
                    uav.status = "TRACKING"
                elif uav.status in ("LOST", "REACQUIRING"):
                    if uav.reacquire_started_at is not None:
                        self.metrics.recovery_successes += 1
                        self.metrics.recovery_times.append(t - uav.reacquire_started_at)
                    uav.status = "RECOVERED"
                    uav.recovered_frames = 1
                    self._event(t, "TARGET_RECOVERED", uav.uav_id, "local vision reacquired target")
                elif uav.status == "RECOVERED":
                    uav.recovered_frames += 1
                    if uav.recovered_frames >= self.cfg.recovered_frame_threshold:
                        uav.status = "TRACKING"
                        uav.recovered_frames = 0
                continue

            uav.lost_frames += 1
            if uav.status in ("INIT", "TRACKING", "RECOVERED") and uav.lost_frames >= self.cfg.lost_frame_threshold:
                uav.status = "LOST"
                uav.reacquire_started_at = None
                uav.status_reason = obs.loss_reason
                self.metrics.lost_events += 1
                self._event(t, "TARGET_LOST", uav.uav_id, obs.loss_reason)

            if uav.status == "LOST":
                if swarm_estimate is not None:
                    uav.status = "REACQUIRING"
                    uav.reacquire_started_at = t
                    uav.target_for_control = swarm_estimate.copy()
                    self._event(t, "COOP_REACQUIRE_START", uav.uav_id, "using swarm target estimate")
                elif uav.last_local_estimate is not None:
                    uav.target_for_control = uav.last_local_estimate.copy()

            elif uav.status == "REACQUIRING":
                if swarm_estimate is not None:
                    uav.target_for_control = swarm_estimate.copy()
                if uav.reacquire_started_at is not None and t - uav.reacquire_started_at > self.cfg.max_recovery_time:
                    uav.status = "FAILED"
                    self.metrics.recovery_failures += 1
                    self._event(t, "RECOVERY_FAILED", uav.uav_id, "recovery timeout")

    def _update_controls(self, t: float) -> None:
        for uav in self.uavs.values():
            desired = None
            if uav.target_for_control is not None:
                radius = self.cfg.follow_radius
                if uav.status == "REACQUIRING":
                    radius = self.cfg.follow_radius * 0.72
                offset = np.array([math.cos(uav.slot_angle), math.sin(uav.slot_angle)]) * radius
                desired = uav.target_for_control + offset
            elif uav.last_local_estimate is not None:
                desired = uav.last_local_estimate.copy()

            if desired is not None and uav.status != "FAILED":
                delta = desired - uav.pos
                max_step = self.cfg.uav_speed * self.cfg.dt
                uav.pos += unit(delta) * min(max_step, norm(delta))
                look_at = uav.target_for_control if uav.target_for_control is not None else desired
                look_delta = look_at - uav.pos
                if norm(look_delta) > 1e-6:
                    uav.yaw = math.atan2(look_delta[1], look_delta[0])
            elif uav.status == "LOST":
                uav.yaw = wrap_angle(uav.yaw + math.radians(35.0) * self.cfg.dt)

            uav.pos[0] = float(np.clip(uav.pos[0], -self.cfg.world_half_size, self.cfg.world_half_size))
            uav.pos[1] = float(np.clip(uav.pos[1], -self.cfg.world_half_size, self.cfg.world_half_size))

    def _event(self, t: float, event_type: str, uav_id: str, description: str) -> None:
        self.events.append(
            {
                "time": round(t, 3),
                "event_type": event_type,
                "uav_id": uav_id,
                "description": description,
            }
        )


def fov_polygon(pos: Vec2, yaw: float, fov_deg: float, length: float) -> np.ndarray:
    half = math.radians(fov_deg) / 2.0
    left = yaw + half
    right = yaw - half
    return np.array(
        [
            pos,
            pos + np.array([math.cos(left), math.sin(left)]) * length,
            pos + np.array([math.cos(right), math.sin(right)]) * length,
        ]
    )


def render_video(frames: List[FrameRecord], sim: MockReacquisitionSim, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    writer = FFMpegWriter(fps=sim.cfg.render_fps, metadata={"title": "multi-uav reacquisition demo"})

    with writer.saving(fig, str(output_path), dpi=120):
        for idx, frame in enumerate(frames):
            if idx % max(1, int((1.0 / sim.cfg.dt) / sim.cfg.render_fps)) != 0:
                continue
            ax.clear()
            ax.set_xlim(-sim.cfg.world_half_size, sim.cfg.world_half_size)
            ax.set_ylim(-sim.cfg.world_half_size, sim.cfg.world_half_size)
            ax.set_aspect("equal", adjustable="box")
            ax.set_title(f"Multi-UAV cooperative reacquisition   t={frame.t:05.1f}s")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.grid(True, color="#e5e7eb", linewidth=0.6)

            for obs in sim.obstacles:
                ax.add_patch(
                    Rectangle(
                        (obs.x_min, obs.y_min),
                        obs.x_max - obs.x_min,
                        obs.y_max - obs.y_min,
                        facecolor="#6b7280",
                        edgecolor="#374151",
                        alpha=0.55,
                    )
                )
                ax.text((obs.x_min + obs.x_max) / 2, obs.y_max + 0.6, obs.name, ha="center", fontsize=7)

            if frame.swarm_estimate is not None:
                ax.scatter([frame.swarm_estimate[0]], [frame.swarm_estimate[1]], marker="x", s=80, c="#7c3aed")
                ax.text(frame.swarm_estimate[0] + 0.5, frame.swarm_estimate[1] + 0.5, "swarm estimate", fontsize=8)

            ax.scatter([frame.target[0]], [frame.target[1]], marker="*", s=170, c="#111827", label="target")
            ax.text(frame.target[0] + 0.5, frame.target[1] - 1.2, "target", fontsize=8)

            for uav_id, (pos, yaw, status, detected) in frame.uavs.items():
                color = STATUS_COLORS.get(status, "#111827")
                poly = fov_polygon(pos, yaw, sim.cfg.camera_fov_deg, sim.cfg.camera_range * 0.45)
                ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor=color, alpha=0.12))
                ax.scatter([pos[0]], [pos[1]], marker="^", s=110, c=color, edgecolors="#111827")
                heading = pos + np.array([math.cos(yaw), math.sin(yaw)]) * 2.4
                ax.plot([pos[0], heading[0]], [pos[1], heading[1]], color=color, linewidth=1.8)
                label = f"{uav_id}: {status}"
                if not detected:
                    reason = frame.observations[uav_id].loss_reason
                    label += f" ({reason})"
                ax.text(pos[0] + 0.6, pos[1] + 0.6, label, fontsize=8, color=color)

            legend_lines = [
                "green=tracking",
                "red=lost",
                "orange=reacquiring",
                "blue=recovered",
            ]
            ax.text(
                -sim.cfg.world_half_size + 1,
                sim.cfg.world_half_size - 2,
                "\n".join(legend_lines),
                fontsize=8,
                va="top",
                bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "#d1d5db"},
            )
            writer.grab_frame()
    plt.close(fig)


def save_outputs(frames: List[FrameRecord], sim: MockReacquisitionSim, run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)

    with (run_dir / "events.jsonl").open("w", encoding="utf-8") as f:
        for event in sim.events:
            f.write(json.dumps(event, ensure_ascii=False) + "\n")

    recovery_times = sim.metrics.recovery_times
    metrics = {
        "lost_events": sim.metrics.lost_events,
        "recovery_successes": sim.metrics.recovery_successes,
        "recovery_failures": sim.metrics.recovery_failures,
        "mean_recovery_time": float(np.mean(recovery_times)) if recovery_times else None,
        "max_recovery_time": float(np.max(recovery_times)) if recovery_times else None,
        "duration": sim.cfg.duration,
        "dt": sim.cfg.dt,
        "seed": sim.cfg.seed,
    }
    (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    with (run_dir / "tracks.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "target_x",
                "target_y",
                "uav_id",
                "uav_x",
                "uav_y",
                "yaw",
                "status",
                "local_detected",
                "loss_reason",
                "swarm_estimate_x",
                "swarm_estimate_y",
            ]
        )
        for frame in frames:
            for uav_id, (pos, yaw, status, detected) in frame.uavs.items():
                obs = frame.observations[uav_id]
                if frame.swarm_estimate is None:
                    sx, sy = "", ""
                else:
                    sx, sy = f"{frame.swarm_estimate[0]:.4f}", f"{frame.swarm_estimate[1]:.4f}"
                writer.writerow(
                    [
                        f"{frame.t:.3f}",
                        f"{frame.target[0]:.4f}",
                        f"{frame.target[1]:.4f}",
                        uav_id,
                        f"{pos[0]:.4f}",
                        f"{pos[1]:.4f}",
                        f"{yaw:.4f}",
                        status,
                        int(detected),
                        obs.loss_reason,
                        sx,
                        sy,
                    ]
                )

    config = {
        "duration": sim.cfg.duration,
        "dt": sim.cfg.dt,
        "render_fps": sim.cfg.render_fps,
        "camera_range": sim.cfg.camera_range,
        "camera_fov_deg": sim.cfg.camera_fov_deg,
        "target_speed": sim.cfg.target_speed,
        "uav_speed": sim.cfg.uav_speed,
        "lost_frame_threshold": sim.cfg.lost_frame_threshold,
        "observation_noise_std": sim.cfg.observation_noise_std,
        "random_dropout_probability": sim.cfg.random_dropout_probability,
    }
    (run_dir / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a lightweight multi-UAV reacquisition mock demo.")
    parser.add_argument("--duration", type=float, default=36.0, help="Simulation duration in seconds.")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation step in seconds.")
    parser.add_argument("--fps", type=int, default=20, help="Output video FPS.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed.")
    parser.add_argument("--output-dir", type=Path, default=Path("runs"), help="Directory for run outputs.")
    parser.add_argument("--no-video", action="store_true", help="Skip topdown.mp4 rendering.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = SimConfig(duration=args.duration, dt=args.dt, render_fps=args.fps, seed=args.seed)
    sim = MockReacquisitionSim(cfg)
    frames = sim.run()

    run_name = datetime.now().strftime("%Y%m%d_%H%M%S_mock_reacquire")
    run_dir = args.output_dir / run_name
    save_outputs(frames, sim, run_dir)
    if not args.no_video:
        render_video(frames, sim, run_dir / "topdown.mp4")

    print(f"Run directory: {run_dir}")
    print(f"Events: {len(sim.events)}")
    print(f"Lost events: {sim.metrics.lost_events}")
    print(f"Recovery successes: {sim.metrics.recovery_successes}")
    print(f"Recovery failures: {sim.metrics.recovery_failures}")
    if sim.metrics.recovery_times:
        print(f"Mean recovery time: {np.mean(sim.metrics.recovery_times):.2f}s")


if __name__ == "__main__":
    main()
