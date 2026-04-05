from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple


def normalize_angle(angle_rad: float) -> float:
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    return angle_rad


def compute_tracking_errors(
    robot_x: float,
    robot_y: float,
    robot_yaw_rad: float,
    path_points: Sequence[Tuple[float, float]],
) -> Optional[Tuple[float, float, int]]:
    if len(path_points) < 2:
        return None

    best_distance = None
    best_cross_track = 0.0
    best_heading_error = 0.0
    best_segment_index = -1

    for index, ((x1, y1), (x2, y2)) in enumerate(zip(path_points[:-1], path_points[1:])):
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1.0e-12:
            continue

        t = ((robot_x - x1) * dx + (robot_y - y1) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        proj_x = x1 + (t * dx)
        proj_y = y1 + (t * dy)

        err_x = robot_x - proj_x
        err_y = robot_y - proj_y
        distance = math.hypot(err_x, err_y)
        if best_distance is not None and distance >= best_distance:
            continue

        cross = (dx * (robot_y - y1)) - (dy * (robot_x - x1))
        signed_cross_track = 0.0
        if distance > 1.0e-12:
            signed_cross_track = math.copysign(distance, cross)

        path_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(path_heading - robot_yaw_rad)

        best_distance = distance
        best_cross_track = signed_cross_track
        best_heading_error = heading_error
        best_segment_index = index

    if best_distance is None:
        return None
    return best_cross_track, best_heading_error, best_segment_index
