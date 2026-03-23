from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time

from navegacion_gps.sim_global_motion_benchmark import MotionBenchmarkProbe
from navegacion_gps.sim_global_motion_benchmark import _cleanup_process
from navegacion_gps.sim_global_motion_benchmark import _launch_simulation
from navegacion_gps.sim_global_motion_benchmark import _resolve_profile
from navegacion_gps.sim_global_motion_benchmark import _resolve_config_path


DEFAULT_GOAL = (12.0, 8.0)
DEFAULT_BOOTSTRAP_TIMEOUT_S = 45.0
DEFAULT_GOAL_TIMEOUT_S = 60.0
DEFAULT_POST_BOOTSTRAP_SETTLE_S = 3.0
DEFAULT_POST_GOAL_DURATION_S = 12.0
DEFAULT_SAMPLE_INTERVAL_S = 0.25
DEFAULT_USE_RVIZ = True


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _angle_delta(a: float, b: float) -> float:
    return abs(_normalize_angle(float(a) - float(b)))


def _distance_xy(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def build_postgoal_report(samples: list[dict[str, Any]], *, goal_status: str) -> dict[str, Any]:
    if not samples:
        raise ValueError("post-goal report requires at least one sample")

    first = samples[0]

    def _max_distance(field: str) -> float:
        values = [
            _distance_xy(first.get(field), sample.get(field))
            for sample in samples
        ]
        valid = [value for value in values if value is not None]
        return max(valid) if valid else 0.0

    def _max_angle(field: str) -> float:
        values = [
            _angle_delta(float(first.get(field, 0.0)), float(sample.get(field, 0.0)))
            for sample in samples
        ]
        return max(values) if values else 0.0

    map_odom_drift_m = _max_distance("map_odom_xy")
    local_motion_m = _max_distance("local_xy")
    global_motion_m = _max_distance("global_xy")
    odom_origin_orbit_m = _max_distance("odom_origin_in_base_xy")
    map_odom_yaw_drift_rad = _max_angle("map_odom_yaw_rad")

    orbit_detected = (
        map_odom_drift_m > 0.5
        and local_motion_m < 0.75
        and odom_origin_orbit_m > 0.5
    ) or (
        map_odom_yaw_drift_rad > 0.25
        and local_motion_m < 0.75
    )

    return {
        "goal_status": goal_status,
        "sample_count": len(samples),
        "post_goal_duration_s": float(samples[-1]["elapsed_s"] - samples[0]["elapsed_s"]),
        "max_map_odom_translation_drift_m": map_odom_drift_m,
        "max_map_odom_yaw_drift_rad": map_odom_yaw_drift_rad,
        "max_local_motion_m": local_motion_m,
        "max_global_motion_m": global_motion_m,
        "max_odom_origin_orbit_m": odom_origin_orbit_m,
        "orbit_detected": orbit_detected,
        "likely_issue": "map_odom_orbit_after_goal" if orbit_detected else "none_detected",
    }


class PostGoalOrbitProbe(MotionBenchmarkProbe):
    def _lookup_transform(self, target_frame: str, source_frame: str) -> Any:
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )

    def _capture_sample(self, start_time: float) -> dict[str, Any]:
        map_odom = self._lookup_transform("map", "odom")
        local_xy = self._pose_xy(self.odom_local)
        global_xy = self._pose_xy(self.odom_global)

        odom_origin = PointStamped()
        odom_origin.header.frame_id = "odom"
        odom_origin.header.stamp = Time().to_msg()
        odom_origin.point.x = 0.0
        odom_origin.point.y = 0.0
        odom_origin.point.z = 0.0
        odom_origin_in_base = self.tf_buffer.transform(
            odom_origin,
            "base_footprint",
            timeout=Duration(seconds=1.0),
        )

        rotation = map_odom.transform.rotation
        return {
            "elapsed_s": float(time.time() - start_time),
            "map_odom_xy": [
                float(map_odom.transform.translation.x),
                float(map_odom.transform.translation.y),
            ],
            "map_odom_yaw_rad": _yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
            "local_xy": list(local_xy) if local_xy is not None else None,
            "global_xy": list(global_xy) if global_xy is not None else None,
            "odom_origin_in_base_xy": [
                float(odom_origin_in_base.point.x),
                float(odom_origin_in_base.point.y),
            ],
        }

    def observe_post_goal(self, *, duration_s: float, sample_interval_s: float) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        start_time = time.time()
        end_time = start_time + float(duration_s)
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            samples.append(self._capture_sample(start_time))
            time.sleep(sample_interval_s)
        return samples


def _parse_goal(text: str) -> tuple[float, float]:
    x_text, y_text = [part.strip() for part in text.split(",", 1)]
    return float(x_text), float(y_text)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze post-goal map->odom orbit behavior in sim_global_v2.",
    )
    parser.add_argument(
        "--profile",
        default="strict_split_lag_compensated",
        help="Internal benchmark profile to launch.",
    )
    parser.add_argument("--goal", default=f"{DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]}")
    parser.add_argument("--bootstrap-timeout-s", type=float, default=DEFAULT_BOOTSTRAP_TIMEOUT_S)
    parser.add_argument("--goal-timeout-s", type=float, default=DEFAULT_GOAL_TIMEOUT_S)
    parser.add_argument("--post-bootstrap-settle-s", type=float, default=DEFAULT_POST_BOOTSTRAP_SETTLE_S)
    parser.add_argument("--post-goal-duration-s", type=float, default=DEFAULT_POST_GOAL_DURATION_S)
    parser.add_argument("--sample-interval-s", type=float, default=DEFAULT_SAMPLE_INTERVAL_S)
    parser.add_argument(
        "--use-rviz",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_USE_RVIZ,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    definition = _resolve_profile(args.profile)
    config_path = _resolve_config_path(args.profile)
    goal_x, goal_y = _parse_goal(args.goal)

    launch_proc = None
    rclpy.init()
    probe = PostGoalOrbitProbe()
    try:
        launch_proc, launch_log_path = _launch_simulation(
            config_path=config_path,
            gps_profile=definition.gps_profile,
            gps_reference_mode=definition.gps_reference_mode,
            use_rviz=bool(args.use_rviz),
        )
        if not probe.wait_for_bootstrap(args.bootstrap_timeout_s):
            raise RuntimeError("simulation bootstrap failed")
        probe._spin_for_duration(float(args.post_bootstrap_settle_s))

        goal_result = probe.send_goal(
            x=goal_x,
            y=goal_y,
            timeout_s=args.goal_timeout_s,
            post_result_grace_s=0.0,
        )
        post_goal_samples = probe.observe_post_goal(
            duration_s=args.post_goal_duration_s,
            sample_interval_s=args.sample_interval_s,
        )
        post_goal_report = build_postgoal_report(
            post_goal_samples,
            goal_status=str(goal_result["goal_status"]),
        )
        payload = {
            "profile": args.profile,
            "config_path": config_path,
            "gps_profile": definition.gps_profile,
            "gps_reference_mode": definition.gps_reference_mode,
            "goal_xy": [goal_x, goal_y],
            "goal_result": goal_result,
            "post_goal_report": post_goal_report,
            "post_goal_samples": post_goal_samples,
            "latest_diag_map": probe.latest_diag_map,
            "events_tail": probe.events[-12:],
            "launch_log_tail": Path(launch_log_path).read_text(encoding="utf-8", errors="replace").splitlines()[-80:],
        }
        text = json.dumps(payload, indent=2, sort_keys=True)
        print(text)
        if args.output:
            Path(args.output).write_text(text + "\n", encoding="utf-8")
        return 0 if not post_goal_report["orbit_detected"] else 2
    finally:
        _cleanup_process(launch_proc, timeout_s=20.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
