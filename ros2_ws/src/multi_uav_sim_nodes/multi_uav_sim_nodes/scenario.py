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
        np.array([-14.0, -7.0], dtype=float),
        np.array([-5.0, -5.0], dtype=float),
        np.array([8.0, 2.0], dtype=float),
        np.array([20.0, 8.0], dtype=float),
        np.array([7.0, 18.0], dtype=float),
        np.array([-16.0, 13.0], dtype=float),
    ]


def default_obstacles() -> List[RectObstacle]:
    return [
        RectObstacle("scripted_occluder", -4.0, 20.0, 4.0, 26.0),
        RectObstacle("pillar_a", -25.0, 15.0, -20.0, 22.0),
        RectObstacle("pillar_b", 20.0, -24.0, 25.0, -17.0),
    ]


def default_uavs() -> Dict[str, UavInit]:
    return {
        "uav_1": UavInit("uav_1", -22.0, -11.0, math.radians(210.0)),
        "uav_2": UavInit("uav_2", -14.0, -18.0, math.radians(270.0)),
        "uav_3": UavInit("uav_3", -5.0, -9.0, math.radians(330.0)),
    }

