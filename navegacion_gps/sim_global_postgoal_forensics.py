from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from navegacion_gps.sim_global_consistency_check import GlobalConsistencyProbe
from navegacion_gps.sim_global_consistency_check import _normalize_angle
from navegacion_gps.sim_global_consistency_check import _quaternion_to_yaw
from navegacion_gps.sim_global_motion_benchmark import _cleanup_process
from navegacion_gps.sim_global_motion_benchmark import _launch_simulation
from navegacion_gps.sim_global_motion_benchmark import _resolve_config_path
from navegacion_gps.sim_global_motion_benchmark import _resolve_profile


DEFAULT_PROFILE = "strict_split_lag_compensated"
DEFAULT_GOAL = (12.0, 8.0)
DEFAULT_BOOTSTRAP_TIMEOUT_S = 45.0
DEFAULT_GOAL_TIMEOUT_S = 60.0
DEFAULT_POST_BOOTSTRAP_SETTLE_S = 3.0
DEFAULT_POST_GOAL_DURATION_S = 12.0
DEFAULT_SAMPLE_INTERVAL_S = 0.25
DEFAULT_USE_RVIZ = True
DEFAULT_JUMP_TRANSLATION_M = 2.0
DEFAULT_JUMP_YAW_RAD = 0.25


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _angle_delta(a: float, b: float) -> float:
    return abs(_normalize_angle(float(a) - float(b)))


class PostGoalForensicsProbe(GlobalConsistencyProbe):
    def __init__(self) -> None:
        super().__init__()
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def navigation_snapshot(self) -> dict[str, Any]:
        local_xy = self._odom_pose_xy(self.odom_local) if self.odom_local is not None else None
        global_xy = self._odom_pose_xy(self.odom_global) if self.odom_global is not None else None
        global_xy_in_odom = None
        if global_xy is not None:
            global_xy_in_odom = self._frame_xy_to_map_xy(global_xy, source_frame="map")
        return {
            "local_pose_xy": list(local_xy) if local_xy is not None else None,
            "global_pose_xy": list(global_xy) if global_xy is not None else None,
            "global_pose_xy_in_odom": list(global_xy_in_odom) if global_xy_in_odom is not None else None,
        }

    def _spin_for_duration(self, duration_s: float) -> None:
        if duration_s <= 0.0:
            return
        end = time.time() + float(duration_s)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_goal(
        self, *, x: float, y: float, timeout_s: float, post_result_grace_s: float
    ) -> dict[str, Any]:
        self.events.clear()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        send_future = self.navigate_client.send_goal_async(goal)
        end = time.time() + min(10.0, timeout_s)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if send_future.done():
                break
        if not send_future.done():
            raise RuntimeError("timeout waiting for navigate_to_pose goal acceptance")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            snapshot = self.navigation_snapshot()
            return {
                "accepted": False,
                "goal_status": "REJECTED",
                "nav_snapshot": snapshot,
                "nav_snapshot_late": snapshot,
                "terminal_event": None,
            }

        result_future = goal_handle.get_result_async()
        finished = False
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                finished = True
                break
        if not finished:
            goal_handle.cancel_goal_async()
            snapshot = self.navigation_snapshot()
            self._spin_for_duration(post_result_grace_s)
            return {
                "accepted": True,
                "goal_status": "TIMEOUT",
                "nav_snapshot": snapshot,
                "nav_snapshot_late": self.navigation_snapshot(),
                "terminal_event": None,
            }

        result = result_future.result()
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        snapshot = self.navigation_snapshot()
        self._spin_for_duration(post_result_grace_s)
        return {
            "accepted": True,
            "goal_status": status_map.get(result.status, str(result.status)),
            "nav_snapshot": snapshot,
            "nav_snapshot_late": self.navigation_snapshot(),
            "terminal_event": None,
        }

    def _lookup_map_odom(self) -> TransformStamped:
        return self.tf_buffer.lookup_transform(
            "map",
            "odom",
            Time(),
            timeout=Duration(seconds=1.0),
        )

    def capture_forensics_sample(self, *, started_at: float) -> dict[str, Any]:
        sample = self.capture_sample()
        map_odom = self._lookup_map_odom()
        sample["elapsed_s"] = float(time.time() - started_at)
        sample["map_odom_xy"] = [
            float(map_odom.transform.translation.x),
            float(map_odom.transform.translation.y),
        ]
        sample["map_odom_yaw_rad"] = _quaternion_to_yaw(
            float(map_odom.transform.rotation.x),
            float(map_odom.transform.rotation.y),
            float(map_odom.transform.rotation.z),
            float(map_odom.transform.rotation.w),
        )
        return sample

    def observe_post_goal(self, *, duration_s: float, sample_interval_s: float) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        started_at = time.time()
        deadline = started_at + float(duration_s)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                samples.append(self.capture_forensics_sample(started_at=started_at))
            except Exception:
                time.sleep(0.1)
                continue
            time.sleep(sample_interval_s)
        return samples


def _parse_goal(text: str) -> tuple[float, float]:
    x_text, y_text = [part.strip() for part in text.split(",", 1)]
    return float(x_text), float(y_text)


def _jump_metrics(prev_sample: dict[str, Any], next_sample: dict[str, Any]) -> dict[str, float]:
    prev_positions = prev_sample["positions_map_xy"]
    next_positions = next_sample["positions_map_xy"]
    return {
        "map_odom_translation_delta_m": _distance_xy(
            tuple(prev_sample["map_odom_xy"]),
            tuple(next_sample["map_odom_xy"]),
        ),
        "map_odom_yaw_delta_rad": _angle_delta(
            float(prev_sample["map_odom_yaw_rad"]),
            float(next_sample["map_odom_yaw_rad"]),
        ),
        "global_pose_delta_m": _distance_xy(
            tuple(prev_positions["global"]),
            tuple(next_positions["global"]),
        ),
        "local_pose_delta_m": _distance_xy(
            tuple(prev_positions["local"]),
            tuple(next_positions["local"]),
        ),
        "gps_odom_delta_m": _distance_xy(
            tuple(prev_positions["gps_odom"]),
            tuple(next_positions["gps_odom"]),
        ),
        "gps_fix_delta_m": _distance_xy(
            tuple(prev_positions["gps"]),
            tuple(next_positions["gps"]),
        ),
        "base_delta_m": _distance_xy(
            tuple(prev_positions["base"]),
            tuple(next_positions["base"]),
        ),
        "gps_odom_stamp_delta_s": abs(
            float(next_sample["source_stamps_s"]["gps_odom"])
            - float(prev_sample["source_stamps_s"]["gps_odom"])
        ),
        "global_stamp_delta_s": abs(
            float(next_sample["source_stamps_s"]["global"])
            - float(prev_sample["source_stamps_s"]["global"])
        ),
        "local_stamp_delta_s": abs(
            float(next_sample["source_stamps_s"]["local"])
            - float(prev_sample["source_stamps_s"]["local"])
        ),
    }


def analyze_postgoal_forensics(
    samples: list[dict[str, Any]],
    *,
    jump_translation_threshold_m: float,
    jump_yaw_threshold_rad: float,
) -> dict[str, Any]:
    if len(samples) < 2:
        raise ValueError("need at least 2 samples")

    jump_events: list[dict[str, Any]] = []
    for index in range(1, len(samples)):
        prev_sample = samples[index - 1]
        next_sample = samples[index]
        metrics = _jump_metrics(prev_sample, next_sample)
        if (
            metrics["map_odom_translation_delta_m"] < jump_translation_threshold_m
            and metrics["map_odom_yaw_delta_rad"] < jump_yaw_threshold_rad
        ):
            continue

        if (
            metrics["map_odom_translation_delta_m"] >= jump_translation_threshold_m
            and metrics["local_pose_delta_m"] < 0.25 * jump_translation_threshold_m
            and metrics["gps_odom_delta_m"] < 0.25 * jump_translation_threshold_m
        ):
            likely_source = "ekf_global_map_to_odom_jump"
        elif (
            metrics["global_pose_delta_m"] >= jump_translation_threshold_m
            and metrics["local_pose_delta_m"] < 0.5
            and metrics["gps_odom_delta_m"] < 0.5
        ):
            likely_source = "ekf_global_internal_jump"
        elif metrics["gps_odom_delta_m"] >= jump_translation_threshold_m:
            likely_source = "navsat_or_gps_odom_jump"
        elif metrics["local_pose_delta_m"] >= jump_translation_threshold_m:
            likely_source = "local_odometry_jump"
        else:
            likely_source = "map_odom_jump_unclassified"

        jump_events.append(
            {
                "sample_index": index,
                "elapsed_s": float(next_sample["elapsed_s"]),
                "likely_source": likely_source,
                "metrics": metrics,
                "before": {
                    "map_odom_xy": prev_sample["map_odom_xy"],
                    "map_odom_yaw_rad": prev_sample["map_odom_yaw_rad"],
                    "positions_map_xy": prev_sample["positions_map_xy"],
                    "source_stamps_s": prev_sample["source_stamps_s"],
                    "source_ages_s": prev_sample["source_ages_s"],
                },
                "after": {
                    "map_odom_xy": next_sample["map_odom_xy"],
                    "map_odom_yaw_rad": next_sample["map_odom_yaw_rad"],
                    "positions_map_xy": next_sample["positions_map_xy"],
                    "source_stamps_s": next_sample["source_stamps_s"],
                    "source_ages_s": next_sample["source_ages_s"],
                },
            }
        )

    counts: dict[str, int] = {}
    for event in jump_events:
        counts[event["likely_source"]] = counts.get(event["likely_source"], 0) + 1
    dominant_source = max(counts.items(), key=lambda item: item[1])[0] if counts else "none_detected"

    return {
        "jump_count": len(jump_events),
        "dominant_source": dominant_source,
        "jump_events": jump_events,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forensic analyzer for post-goal map->odom jumps in sim_global_v2.",
    )
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--goal", default=f"{DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]}")
    parser.add_argument("--bootstrap-timeout-s", type=float, default=DEFAULT_BOOTSTRAP_TIMEOUT_S)
    parser.add_argument("--goal-timeout-s", type=float, default=DEFAULT_GOAL_TIMEOUT_S)
    parser.add_argument("--post-bootstrap-settle-s", type=float, default=DEFAULT_POST_BOOTSTRAP_SETTLE_S)
    parser.add_argument("--post-goal-duration-s", type=float, default=DEFAULT_POST_GOAL_DURATION_S)
    parser.add_argument("--sample-interval-s", type=float, default=DEFAULT_SAMPLE_INTERVAL_S)
    parser.add_argument("--jump-translation-threshold-m", type=float, default=DEFAULT_JUMP_TRANSLATION_M)
    parser.add_argument("--jump-yaw-threshold-rad", type=float, default=DEFAULT_JUMP_YAW_RAD)
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
    probe = PostGoalForensicsProbe()
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
        analysis = analyze_postgoal_forensics(
            post_goal_samples,
            jump_translation_threshold_m=float(args.jump_translation_threshold_m),
            jump_yaw_threshold_rad=float(args.jump_yaw_threshold_rad),
        )

        payload = {
            "profile": args.profile,
            "config_path": config_path,
            "gps_profile": definition.gps_profile,
            "gps_reference_mode": definition.gps_reference_mode,
            "goal_xy": [goal_x, goal_y],
            "goal_result": goal_result,
            "analysis": analysis,
            "post_goal_samples": post_goal_samples,
            "latest_diag_map": probe.latest_diag_map,
            "events_tail": probe.events[-12:],
            "launch_log_tail": Path(launch_log_path).read_text(encoding="utf-8", errors="replace").splitlines()[-80:],
        }
        text = json.dumps(payload, indent=2, sort_keys=True)
        print(text)
        if args.output:
            Path(args.output).write_text(text + "\n", encoding="utf-8")
        return 0 if analysis["jump_count"] == 0 else 2
    finally:
        _cleanup_process(launch_proc, timeout_s=20.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
