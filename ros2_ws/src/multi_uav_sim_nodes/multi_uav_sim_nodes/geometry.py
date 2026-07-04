from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import numpy as np


Vec2 = np.ndarray


@dataclass(frozen=True)
class RectObstacle:
    name: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def corners(self) -> List[Vec2]:
        return [
            np.array([self.x_min, self.y_min], dtype=float),
            np.array([self.x_max, self.y_min], dtype=float),
            np.array([self.x_max, self.y_max], dtype=float),
            np.array([self.x_min, self.y_max], dtype=float),
        ]

    def edges(self) -> Iterable[Tuple[Vec2, Vec2]]:
        corners = self.corners()
        for index in range(4):
            yield corners[index], corners[(index + 1) % 4]


def norm(vec: Vec2) -> float:
    return float(np.linalg.norm(vec))


def unit(vec: Vec2) -> Vec2:
    length = norm(vec)
    if length < 1e-9:
        return np.zeros(2, dtype=float)
    return vec / length


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
    for obstacle in obstacles:
        for edge_a, edge_b in obstacle.edges():
            if segment_intersects(observer, target, edge_a, edge_b):
                return True
    return False

