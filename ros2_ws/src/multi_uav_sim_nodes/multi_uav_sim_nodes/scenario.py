from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from .geometry import RectObstacle


@dataclass(frozen=True)
class UavInit:
    uav_id: str
    x: float
    y: float
    slot_angle: float


def default_uav_ids() -> List[str]:
    return ["uav_1", "uav_2", "uav_3"]


def default_target_waypoints() -> List[np.ndarray]:
    return [
        np.array([-16.0, -10.0, 7.0], dtype=float),
        np.array([-6.0, -6.0, 7.5], dtype=float),
        np.array([4.0, 0.0, 8.2], dtype=float),
        np.array([12.0, 7.0, 7.2], dtype=float),
        np.array([22.0, 10.0, 8.8], dtype=float),
        np.array([12.0, 20.0, 7.5], dtype=float),
        np.array([-10.0, 16.0, 8.0], dtype=float),
    ]


def default_obstacles() -> List[RectObstacle]:
    return [
        RectObstacle("central_wall", 4.0, 2.0, 15.0, 8.5),
        RectObstacle("pipe_gate_left", 8.0, 8.0, 10.0, 13.0),
        RectObstacle("pipe_gate_right", 15.0, 7.0, 17.0, 13.0),
        RectObstacle("tower_cluster", 1.0, 10.0, 5.0, 15.0),
        RectObstacle("factory_block", -2.0, 2.0, 2.0, 7.0),
    ]


def default_uavs() -> Dict[str, UavInit]:
    return {
        "uav_1": UavInit("uav_1", -24.0, -12.0, math.radians(210.0)),
        "uav_2": UavInit("uav_2", -16.0, -18.0, math.radians(270.0)),
        "uav_3": UavInit("uav_3", -7.0, -11.0, math.radians(330.0)),
    }
