from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point


def point_from_xy(xy: np.ndarray) -> Point:
    msg = Point()
    msg.x = float(xy[0])
    msg.y = float(xy[1])
    msg.z = 0.0
    return msg


def xy_from_point(point: Point) -> np.ndarray:
    return np.array([point.x, point.y], dtype=float)

