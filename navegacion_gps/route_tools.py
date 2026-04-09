"""Shared helpers for manual route recording and replay."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import yaml


class RouteFileError(ValueError):
    """Raised when a recorded route YAML file is invalid."""


@dataclass(frozen=True)
class RoutePoint:
    """Simple planar waypoint used by teach-and-repeat."""

    x: float
    y: float
    yaw: float


def normalize_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    value = float(angle_rad)
    while value <= -math.pi:
        value += 2.0 * math.pi
    while value > math.pi:
        value -= 2.0 * math.pi
    return value


def angular_distance_rad(a_rad: float, b_rad: float) -> float:
    """Return the shortest absolute angular distance between two yaws."""

    return abs(normalize_angle_rad(float(a_rad) - float(b_rad)))


def yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a quaternion."""

    return math.atan2(
        2.0 * ((float(w) * float(z)) + (float(x) * float(y))),
        1.0 - 2.0 * ((float(y) * float(y)) + (float(z) * float(z))),
    )


def quaternion_xyzw_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """Build a planar quaternion from yaw."""

    half = 0.5 * float(yaw_rad)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def planar_distance_m(a: RoutePoint, b: RoutePoint) -> float:
    """Return planar distance between two recorded points."""

    return math.hypot(float(b.x) - float(a.x), float(b.y) - float(a.y))


def should_record_point(
    last_saved: RoutePoint,
    current: RoutePoint,
    min_distance_m: float,
    min_yaw_rad: float,
) -> bool:
    """Return True when the current point is worth saving."""

    if planar_distance_m(last_saved, current) >= max(0.0, float(min_distance_m)):
        return True
    return angular_distance_rad(current.yaw, last_saved.yaw) >= max(
        0.0, float(min_yaw_rad)
    )


def route_point_almost_equal(a: RoutePoint, b: RoutePoint, tol: float = 1.0e-6) -> bool:
    """Return True when two points are effectively the same."""

    return (
        planar_distance_m(a, b) <= float(tol)
        and angular_distance_rad(a.yaw, b.yaw) <= float(tol)
    )


def route_to_yaml_data(frame_id: str, points: Sequence[RoutePoint]) -> dict[str, Any]:
    """Convert a route into the YAML structure stored on disk."""

    return {
        "frame_id": str(frame_id),
        "points": [
            {
                "x": float(point.x),
                "y": float(point.y),
                "yaw": float(point.yaw),
            }
            for point in points
        ],
    }


def save_route_yaml(path: Path, frame_id: str, points: Sequence[RoutePoint]) -> None:
    """Write a route YAML file to disk."""

    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(route_to_yaml_data(frame_id, points), handle, sort_keys=False)


def load_route_yaml(path: Path) -> tuple[str, list[RoutePoint]]:
    """Load and validate a recorded route YAML file."""

    input_path = Path(path).expanduser()
    try:
        with input_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError as exc:
        raise RouteFileError(f"route file not found: {input_path}") from exc
    except OSError as exc:
        raise RouteFileError(f"failed reading route file {input_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise RouteFileError(f"invalid YAML in route file {input_path}: {exc}") from exc

    if not isinstance(data, dict):
        raise RouteFileError("route YAML must contain a dictionary at the root")

    frame_id = str(data.get("frame_id", "")).strip()
    if not frame_id:
        raise RouteFileError("route YAML is missing a non-empty frame_id")

    raw_points = data.get("points")
    if not isinstance(raw_points, list) or len(raw_points) == 0:
        raise RouteFileError("route YAML must contain a non-empty points list")

    parsed_points: list[RoutePoint] = []
    for index, raw_point in enumerate(raw_points, start=1):
        if not isinstance(raw_point, dict):
            raise RouteFileError(f"route point {index} must be a dictionary")
        try:
            point = RoutePoint(
                x=float(raw_point["x"]),
                y=float(raw_point["y"]),
                yaw=float(raw_point["yaw"]),
            )
        except KeyError as exc:
            raise RouteFileError(
                f"route point {index} is missing key {exc.args[0]!r}"
            ) from exc
        except (TypeError, ValueError) as exc:
            raise RouteFileError(
                f"route point {index} must contain numeric x/y/yaw values"
            ) from exc
        parsed_points.append(point)

    return frame_id, parsed_points
