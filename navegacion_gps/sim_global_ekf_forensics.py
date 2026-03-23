from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Twist
from interfaces.msg import CmdVelFinal
from interfaces.msg import DriveTelemetry
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Imu

from navegacion_gps.goal_pose_to_follow_path_v2 import closest_path_pose
from navegacion_gps.goal_pose_to_follow_path_v2 import minimum_distance_to_path_xy
from navegacion_gps.goal_pose_to_follow_path_v2 import normalize_angle as path_normalize_angle
from navegacion_gps.goal_pose_to_follow_path_v2 import yaw_from_quaternion
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
DEFAULT_POST_BOOTSTRAP_SETTLE_S = 5.0
DEFAULT_POST_GOAL_DURATION_S = 10.0
DEFAULT_SAMPLE_INTERVAL_S = 0.05
DEFAULT_USE_RVIZ = True
DEFAULT_JUMP_TRANSLATION_M = 2.0
DEFAULT_JUMP_YAW_RAD = 0.25
DEFAULT_OBSERVE_PHASE = "full_run"
DEFAULT_WINDOW_RADIUS = 6
OBSERVE_PHASES = ("during_goal", "post_goal", "full_run")


@dataclass(frozen=True)
class ForensicsRunConfig:
    profile: str
    goal_xy: tuple[float, float]
    bootstrap_timeout_s: float
    goal_timeout_s: float
    post_bootstrap_settle_s: float
    observe_phase: str
    post_goal_duration_s: float
    sample_interval_s: float
    jump_translation_threshold_m: float
    jump_yaw_threshold_rad: float
    window_radius: int
    use_rviz: bool
    output: str = ""


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _angle_delta(a: float, b: float) -> float:
    return abs(_normalize_angle(float(a) - float(b)))


def _covariance_diag3(covariance: list[float] | tuple[float, ...]) -> dict[str, float]:
    if len(covariance) < 36:
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}
    return {
        "x": float(covariance[0]),
        "y": float(covariance[7]),
        "yaw": float(covariance[35]),
    }


def _clock_now_s(node: GlobalConsistencyProbe) -> float:
    return float(node.get_clock().now().nanoseconds) / 1e9


def _safe_rate(delta: float, dt: float) -> float:
    if dt <= 1e-6:
        return 0.0
    return float(delta) / float(dt)


def _dominant_label(counts: dict[str, int], fallback: str) -> str:
    if not counts:
        return fallback
    return max(counts.items(), key=lambda item: item[1])[0]


def _twist_to_dict(msg: Twist | None) -> dict[str, float] | None:
    if msg is None:
        return None
    return {
        "linear_x": float(msg.linear.x),
        "angular_z": float(msg.angular.z),
    }


def _cmd_vel_final_to_dict(msg: CmdVelFinal | None) -> dict[str, float | int] | None:
    if msg is None:
        return None
    return {
        "linear_x": float(msg.twist.linear.x),
        "angular_z": float(msg.twist.angular.z),
        "brake_pct": int(msg.brake_pct),
    }


def _drive_telemetry_to_dict(msg: DriveTelemetry | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        "ready": bool(msg.ready),
        "fresh": bool(msg.fresh),
        "drive_enabled": bool(msg.drive_enabled),
        "estop": bool(msg.estop),
        "reverse_requested": bool(msg.reverse_requested),
        "speed_valid": bool(msg.speed_valid),
        "steer_valid": bool(msg.steer_valid),
        "control_source": str(msg.control_source),
        "speed_mps_measured": float(msg.speed_mps_measured),
        "steer_deg_measured": float(msg.steer_deg_measured),
        "brake_applied_pct": int(msg.brake_applied_pct),
    }


def _imu_to_dict(msg: Imu | None) -> dict[str, float] | None:
    if msg is None:
        return None
    return {
        "yaw_rad": _quaternion_to_yaw(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        ),
        "angular_velocity_z": float(msg.angular_velocity.z),
    }


def _path_pose_xy(path: NavPath, index: int) -> tuple[float, float]:
    pose = path.poses[index].pose.position
    return float(pose.x), float(pose.y)


def _plan_summary(path: NavPath | None, *, sample_points: int = 8) -> dict[str, Any] | None:
    if path is None or len(path.poses) < 2:
        return None
    pose_count = len(path.poses)
    total_length = 0.0
    segment_lengths: list[float] = []
    headings: list[float] = []
    limit = min(pose_count, sample_points)
    for index in range(limit - 1):
        a = _path_pose_xy(path, index)
        b = _path_pose_xy(path, index + 1)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)
        segment_lengths.append(length)
        total_length += length
        headings.append(math.atan2(dy, dx) if length > 1e-6 else 0.0)
    curvatures: list[float] = []
    for index in range(1, len(headings)):
        heading_delta = abs(_normalize_angle(headings[index] - headings[index - 1]))
        arc = max(segment_lengths[index - 1], 1e-6)
        curvatures.append(heading_delta / arc)
    return {
        "pose_count": pose_count,
        "sampled_length_m": total_length,
        "mean_curvature": float(sum(curvatures) / len(curvatures)) if curvatures else 0.0,
        "max_curvature": float(max(curvatures)) if curvatures else 0.0,
    }


def _pose_from_xy_yaw(x_m: float, y_m: float, yaw_rad: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x_m)
    pose.position.y = float(y_m)
    pose.position.z = 0.0
    half_yaw = 0.5 * float(yaw_rad)
    pose.orientation.w = math.cos(half_yaw)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half_yaw)
    return pose


def _path_tracking_snapshot(path: NavPath | None, sample: dict[str, Any]) -> dict[str, Any] | None:
    if path is None or not path.poses:
        return None
    headings = sample["headings_map_rad"]
    positions = sample["positions_map_xy"]
    tracking: dict[str, Any] = {}
    for key in ("global", "local", "base"):
        pose = _pose_from_xy_yaw(
            float(positions[key][0]),
            float(positions[key][1]),
            float(headings[key]),
        )
        distance_to_path_m = float(minimum_distance_to_path_xy(path, pose))
        closest = closest_path_pose(path, pose)
        if closest is None:
            heading_error_rad = 0.0
            closest_xy = [0.0, 0.0]
            closest_yaw_rad = 0.0
        else:
            closest_yaw_rad = float(yaw_from_quaternion(closest.pose.orientation))
            heading_error_rad = abs(path_normalize_angle(closest_yaw_rad - float(headings[key])))
            closest_xy = [
                float(closest.pose.position.x),
                float(closest.pose.position.y),
            ]
        tracking[key] = {
            "distance_to_path_m": distance_to_path_m,
            "heading_error_rad": float(heading_error_rad),
            "closest_path_xy": closest_xy,
            "closest_path_yaw_rad": closest_yaw_rad,
        }
    return tracking


class EkfForensicsProbe(GlobalConsistencyProbe):
    def __init__(self) -> None:
        super().__init__()
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_vel_nav: Twist | None = None
        self.cmd_vel_final: CmdVelFinal | None = None
        self.drive_telemetry: DriveTelemetry | None = None
        self.global_plan: NavPath | None = None
        self.imu_data: Imu | None = None
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel_nav, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self._on_cmd_vel_final, 10)
        self.create_subscription(DriveTelemetry, "/controller/drive_telemetry", self._on_drive_telemetry, 10)
        self.create_subscription(NavPath, "/plan", self._on_global_plan, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu_data, 20)

    def _on_cmd_vel_nav(self, msg: Twist) -> None:
        self.cmd_vel_nav = msg

    def _on_cmd_vel_final(self, msg: CmdVelFinal) -> None:
        self.cmd_vel_final = msg

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        self.drive_telemetry = msg

    def _on_global_plan(self, msg: NavPath) -> None:
        self.global_plan = msg

    def _on_imu_data(self, msg: Imu) -> None:
        self.imu_data = msg

    def _spin_for_duration(self, duration_s: float) -> None:
        if duration_s <= 0.0:
            return
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _lookup_map_odom(self):
        return self.tf_buffer.lookup_transform(
            "map",
            "odom",
            Time(),
            timeout=Duration(seconds=1.0),
        )

    def _lookup_odom_xy(self, target_frame: str, source_frame: str) -> list[float]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        return [
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        ]

    def capture_sample(self, *, started_at: float, phase: str) -> dict[str, Any]:
        sample = super().capture_sample()
        map_odom = self._lookup_map_odom()
        sample["phase"] = str(phase)
        sample["elapsed_s"] = float(time.monotonic() - started_at)
        sample["wall_time_monotonic_s"] = float(time.monotonic())
        sample["ros_now_s"] = _clock_now_s(self)
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
        sample["filtered_covariance_diag"] = _covariance_diag3(self.odom_global.pose.covariance)
        sample["local_covariance_diag"] = _covariance_diag3(self.odom_local.pose.covariance)
        sample["gps_odom_covariance_diag"] = _covariance_diag3(self.odom_gps.pose.covariance)
        sample["odom_origin_in_base_xy"] = self._lookup_odom_xy("base_footprint", "odom")
        sample["cmd_vel_nav"] = _twist_to_dict(self.cmd_vel_nav)
        sample["cmd_vel_final"] = _cmd_vel_final_to_dict(self.cmd_vel_final)
        sample["drive_telemetry"] = _drive_telemetry_to_dict(self.drive_telemetry)
        sample["plan_summary"] = _plan_summary(self.global_plan)
        sample["gps_odom_yaw_rad"] = _quaternion_to_yaw(
            float(self.odom_gps.pose.pose.orientation.x),
            float(self.odom_gps.pose.pose.orientation.y),
            float(self.odom_gps.pose.pose.orientation.z),
            float(self.odom_gps.pose.pose.orientation.w),
        )
        sample["imu_data"] = _imu_to_dict(self.imu_data)
        sample["path_tracking"] = _path_tracking_snapshot(self.global_plan, sample)
        return sample

    def _sample_loop(
        self,
        *,
        started_at: float,
        deadline_monotonic: float,
        sample_interval_s: float,
        phase: str,
        stop_future: Any | None = None,
    ) -> tuple[list[dict[str, Any]], bool]:
        samples: list[dict[str, Any]] = []
        next_sample_at = time.monotonic()
        result_done = False
        while time.monotonic() < deadline_monotonic:
            now = time.monotonic()
            timeout_sec = min(0.02, max(0.0, next_sample_at - now))
            rclpy.spin_once(self, timeout_sec=timeout_sec)
            now = time.monotonic()
            while now >= next_sample_at:
                try:
                    samples.append(self.capture_sample(started_at=started_at, phase=phase))
                except Exception:
                    pass
                next_sample_at += float(sample_interval_s)
            if stop_future is not None and stop_future.done():
                result_done = True
                break
        return samples, result_done

    def send_goal_and_observe(
        self,
        *,
        x: float,
        y: float,
        timeout_s: float,
        sample_interval_s: float,
        observe_phase: str,
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        self.events.clear()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        started_at = time.monotonic()
        samples: list[dict[str, Any]] = []
        send_future = self.navigate_client.send_goal_async(goal)
        acceptance_deadline = time.monotonic() + min(10.0, timeout_s)
        while time.monotonic() < acceptance_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if send_future.done():
                break
        if not send_future.done():
            raise RuntimeError("timeout waiting for navigate_to_pose goal acceptance")

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return ({
                "accepted": False,
                "goal_status": "REJECTED",
                "terminal_event": None,
            }, samples)

        result_future = goal_handle.get_result_async()
        if observe_phase in {"during_goal", "full_run"}:
            deadline = time.monotonic() + float(timeout_s)
            samples, finished = self._sample_loop(
                started_at=started_at,
                deadline_monotonic=deadline,
                sample_interval_s=sample_interval_s,
                phase="during_goal",
                stop_future=result_future,
            )
        else:
            finished = False
            deadline = time.monotonic() + float(timeout_s)
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if result_future.done():
                    finished = True
                    break

        if not result_future.done():
            goal_handle.cancel_goal_async()
            return ({
                "accepted": True,
                "goal_status": "TIMEOUT",
                "terminal_event": None,
            }, samples)

        result = result_future.result()
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        return ({
            "accepted": True,
            "goal_status": status_map.get(result.status, str(result.status)),
            "terminal_event": None,
        }, samples)

    def observe_phase(
        self,
        *,
        duration_s: float,
        sample_interval_s: float,
        started_at: float,
        phase: str,
    ) -> list[dict[str, Any]]:
        samples, _ = self._sample_loop(
            started_at=started_at,
            deadline_monotonic=time.monotonic() + float(duration_s),
            sample_interval_s=sample_interval_s,
            phase=phase,
        )
        return samples


def _parse_goal(text: str) -> tuple[float, float]:
    x_text, y_text = [part.strip() for part in text.split(",", 1)]
    return float(x_text), float(y_text)


def _tf_filtered_disagreement(sample: dict[str, Any]) -> tuple[float, float]:
    return (
        float(sample["position_gaps_m"]["global_vs_local"]),
        float(sample["heading_gaps_rad"]["global_vs_local"]),
    )


def _jump_metrics(prev_sample: dict[str, Any], next_sample: dict[str, Any]) -> dict[str, float]:
    prev_positions = prev_sample["positions_map_xy"]
    next_positions = next_sample["positions_map_xy"]
    prev_tf_disagreement_m, prev_tf_disagreement_yaw = _tf_filtered_disagreement(prev_sample)
    next_tf_disagreement_m, next_tf_disagreement_yaw = _tf_filtered_disagreement(next_sample)
    global_stamp_delta_signed_s = float(next_sample["source_stamps_s"]["global"]) - float(
        prev_sample["source_stamps_s"]["global"]
    )
    gps_odom_stamp_delta_signed_s = float(next_sample["source_stamps_s"]["gps_odom"]) - float(
        prev_sample["source_stamps_s"]["gps_odom"]
    )
    local_stamp_delta_signed_s = float(next_sample["source_stamps_s"]["local"]) - float(
        prev_sample["source_stamps_s"]["local"]
    )
    gps_stamp_delta_signed_s = float(next_sample["source_stamps_s"]["gps"]) - float(
        prev_sample["source_stamps_s"]["gps"]
    )
    base_stamp_delta_signed_s = float(next_sample["source_stamps_s"]["base"]) - float(
        prev_sample["source_stamps_s"]["base"]
    )
    wall_dt_s = max(
        1e-6,
        float(next_sample["wall_time_monotonic_s"]) - float(prev_sample["wall_time_monotonic_s"]),
    )
    map_jump_m = _distance_xy(
        tuple(prev_sample["map_odom_xy"]),
        tuple(next_sample["map_odom_xy"]),
    )
    map_yaw_jump = _angle_delta(
        float(prev_sample["map_odom_yaw_rad"]),
        float(next_sample["map_odom_yaw_rad"]),
    )
    filtered_jump_m = _distance_xy(
        tuple(prev_positions["global"]),
        tuple(next_positions["global"]),
    )
    filtered_yaw_jump = _angle_delta(
        float(prev_sample["headings_map_rad"]["global"]),
        float(next_sample["headings_map_rad"]["global"]),
    )
    return {
        "map_odom_translation_delta_m": map_jump_m,
        "map_odom_yaw_delta_rad": map_yaw_jump,
        "filtered_delta_m": filtered_jump_m,
        "filtered_yaw_delta_rad": filtered_yaw_jump,
        "local_delta_m": _distance_xy(
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
        "global_stamp_delta_s": abs(global_stamp_delta_signed_s),
        "global_stamp_delta_signed_s": global_stamp_delta_signed_s,
        "local_stamp_delta_s": abs(local_stamp_delta_signed_s),
        "local_stamp_delta_signed_s": local_stamp_delta_signed_s,
        "gps_odom_stamp_delta_s": abs(gps_odom_stamp_delta_signed_s),
        "gps_odom_stamp_delta_signed_s": gps_odom_stamp_delta_signed_s,
        "gps_stamp_delta_s": abs(gps_stamp_delta_signed_s),
        "gps_stamp_delta_signed_s": gps_stamp_delta_signed_s,
        "base_stamp_delta_s": abs(base_stamp_delta_signed_s),
        "base_stamp_delta_signed_s": base_stamp_delta_signed_s,
        "largest_source_age_s": max(
            float(next_sample["source_ages_s"]["global"]),
            float(next_sample["source_ages_s"]["local"]),
            float(next_sample["source_ages_s"]["gps_odom"]),
            float(next_sample["source_ages_s"]["base"]),
            float(next_sample["source_ages_s"]["gps"]),
        ),
        "largest_global_age_s": float(next_sample["source_ages_s"]["global"]),
        "largest_gps_odom_age_s": float(next_sample["source_ages_s"]["gps_odom"]),
        "largest_local_age_s": float(next_sample["source_ages_s"]["local"]),
        "largest_gps_age_s": float(next_sample["source_ages_s"]["gps"]),
        "largest_base_age_s": float(next_sample["source_ages_s"]["base"]),
        "tf_vs_filtered_disagreement_before_m": prev_tf_disagreement_m,
        "tf_vs_filtered_disagreement_after_m": next_tf_disagreement_m,
        "tf_vs_filtered_disagreement_delta_m": abs(next_tf_disagreement_m - prev_tf_disagreement_m),
        "tf_vs_filtered_disagreement_before_yaw_rad": prev_tf_disagreement_yaw,
        "tf_vs_filtered_disagreement_after_yaw_rad": next_tf_disagreement_yaw,
        "tf_vs_filtered_disagreement_delta_yaw_rad": abs(
            next_tf_disagreement_yaw - prev_tf_disagreement_yaw
        ),
        "tf_jump_m": map_jump_m,
        "filtered_jump_m": filtered_jump_m,
        "tf_vs_filtered_delta_m": abs(map_jump_m - filtered_jump_m),
        "tf_vs_filtered_delta_yaw_rad": abs(map_yaw_jump - filtered_yaw_jump),
        "map_odom_speed_mps": _safe_rate(map_jump_m, wall_dt_s),
        "filtered_speed_mps": _safe_rate(filtered_jump_m, wall_dt_s),
        "map_odom_yaw_rate_rps": _safe_rate(map_yaw_jump, wall_dt_s),
        "filtered_yaw_rate_rps": _safe_rate(filtered_yaw_jump, wall_dt_s),
        "wall_dt_s": wall_dt_s,
        "source_age_vector": {
            key: float(value) for key, value in next_sample["source_ages_s"].items()
        },
        "source_stamp_vector": {
            key: float(value) for key, value in next_sample["source_stamps_s"].items()
        },
        "has_out_of_order_source": any(
            delta < -1e-6
            for delta in (
                global_stamp_delta_signed_s,
                local_stamp_delta_signed_s,
                gps_odom_stamp_delta_signed_s,
                gps_stamp_delta_signed_s,
                base_stamp_delta_signed_s,
            )
        ),
    }


def _classify_error_mode(
    metrics: dict[str, float],
    *,
    jump_translation_threshold_m: float,
    jump_yaw_threshold_rad: float,
) -> str:
    map_jump = metrics["map_odom_translation_delta_m"]
    filtered_jump = metrics["filtered_delta_m"]
    map_yaw_jump = metrics["map_odom_yaw_delta_rad"]
    filtered_yaw_jump = metrics["filtered_yaw_delta_rad"]
    if (
        map_jump >= jump_translation_threshold_m
        and filtered_jump < jump_translation_threshold_m * 0.5
        and metrics["tf_vs_filtered_disagreement_after_m"]
        > metrics["tf_vs_filtered_disagreement_before_m"] + 0.5
    ) or (
        map_yaw_jump >= jump_yaw_threshold_rad
        and filtered_yaw_jump < jump_yaw_threshold_rad * 0.5
        and metrics["tf_vs_filtered_disagreement_after_yaw_rad"]
        > metrics["tf_vs_filtered_disagreement_before_yaw_rad"] + 0.1
    ):
        return "map_to_odom_tf_jump"
    if (
        filtered_jump >= jump_translation_threshold_m
        and map_jump < jump_translation_threshold_m * 0.5
    ) or (
        filtered_yaw_jump >= jump_yaw_threshold_rad
        and map_yaw_jump < jump_yaw_threshold_rad * 0.5
    ):
        return "filtered_state_drift"
    if (
        map_jump >= jump_translation_threshold_m and filtered_jump >= jump_translation_threshold_m * 0.5
    ) or (
        map_yaw_jump >= jump_yaw_threshold_rad and filtered_yaw_jump >= jump_yaw_threshold_rad * 0.5
    ):
        return "combined_global_correction"
    return "mixed_or_unclassified"


def classify_jump_event(
    metrics: dict[str, float],
    *,
    jump_translation_threshold_m: float,
    jump_yaw_threshold_rad: float,
) -> dict[str, str]:
    lag_suspected = (
        metrics["largest_source_age_s"] >= 0.25
        or metrics["gps_odom_stamp_delta_s"] >= 0.25
        or metrics["global_stamp_delta_s"] >= 0.25
        or metrics["has_out_of_order_source"]
    )
    if lag_suspected and metrics["map_odom_translation_delta_m"] >= jump_translation_threshold_m:
        return {
            "classification": "lag_or_reorder_suspected",
            "confidence": "high" if metrics["largest_source_age_s"] >= 0.5 else "medium",
            "dominant_source": "lag_or_reorder",
            "dominant_error_mode": _classify_error_mode(
                metrics,
                jump_translation_threshold_m=jump_translation_threshold_m,
                jump_yaw_threshold_rad=jump_yaw_threshold_rad,
            ),
        }

    error_mode = _classify_error_mode(
        metrics,
        jump_translation_threshold_m=jump_translation_threshold_m,
        jump_yaw_threshold_rad=jump_yaw_threshold_rad,
    )
    if metrics["gps_odom_delta_m"] >= jump_translation_threshold_m:
        return {
            "classification": "gps_odom_driven_jump",
            "confidence": "high" if metrics["gps_odom_delta_m"] >= jump_translation_threshold_m * 1.5 else "medium",
            "dominant_source": "gps_odom",
            "dominant_error_mode": error_mode,
        }
    if metrics["local_delta_m"] >= jump_translation_threshold_m or metrics["base_delta_m"] >= jump_translation_threshold_m:
        return {
            "classification": "local_odom_driven_jump",
            "confidence": "high" if max(metrics["local_delta_m"], metrics["base_delta_m"]) >= jump_translation_threshold_m * 1.5 else "medium",
            "dominant_source": "local_odom",
            "dominant_error_mode": error_mode,
        }
    if error_mode == "map_to_odom_tf_jump":
        return {
            "classification": "map_to_odom_tf_jump",
            "confidence": "high",
            "dominant_source": "global_tf",
            "dominant_error_mode": error_mode,
        }
    if error_mode == "filtered_state_drift":
        return {
            "classification": "filtered_state_drift",
            "confidence": "medium",
            "dominant_source": "filtered_state",
            "dominant_error_mode": error_mode,
        }
    if error_mode == "combined_global_correction":
        return {
            "classification": "combined_global_correction",
            "confidence": "medium",
            "dominant_source": "global_chain",
            "dominant_error_mode": error_mode,
        }
    if (
        metrics["map_odom_translation_delta_m"] >= jump_translation_threshold_m
        or metrics["map_odom_yaw_delta_rad"] >= jump_yaw_threshold_rad
    ):
        return {
            "classification": "mixed_or_unclassified",
            "confidence": "low",
            "dominant_source": "mixed",
            "dominant_error_mode": error_mode,
        }
    return {
        "classification": "none",
        "confidence": "low",
        "dominant_source": "none",
        "dominant_error_mode": "none",
    }


def _compact_sample(sample: dict[str, Any]) -> dict[str, Any]:
    compact = {
        "elapsed_s": float(sample["elapsed_s"]),
        "phase": str(sample["phase"]),
        "wall_time_monotonic_s": float(sample["wall_time_monotonic_s"]),
        "ros_now_s": float(sample["ros_now_s"]),
        "map_odom_xy": sample["map_odom_xy"],
        "map_odom_yaw_rad": float(sample["map_odom_yaw_rad"]),
        "positions_map_xy": sample["positions_map_xy"],
        "headings_map_rad": sample["headings_map_rad"],
        "position_gaps_m": sample["position_gaps_m"],
        "heading_gaps_rad": sample["heading_gaps_rad"],
        "source_stamps_s": sample["source_stamps_s"],
        "source_ages_s": sample["source_ages_s"],
        "filtered_covariance_diag": sample["filtered_covariance_diag"],
        "local_covariance_diag": sample["local_covariance_diag"],
        "gps_odom_covariance_diag": sample["gps_odom_covariance_diag"],
        "gps_odom_yaw_rad": float(sample.get("gps_odom_yaw_rad", 0.0)),
        "cmd_vel_nav": sample.get("cmd_vel_nav"),
        "cmd_vel_final": sample.get("cmd_vel_final"),
        "drive_telemetry": sample.get("drive_telemetry"),
        "plan_summary": sample.get("plan_summary"),
        "imu_data": sample.get("imu_data"),
    }
    if "delta_from_prev" in sample:
        compact["delta_from_prev"] = sample["delta_from_prev"]
    if "rates" in sample:
        compact["rates"] = sample["rates"]
    return compact


def _enrich_samples(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    enriched: list[dict[str, Any]] = []
    prev_sample: dict[str, Any] | None = None
    for sample in samples:
        current = dict(sample)
        if prev_sample is None:
            current["delta_from_prev"] = {}
            current["rates"] = {}
        else:
            metrics = _jump_metrics(prev_sample, current)
            current["delta_from_prev"] = {
                "tf_jump_m": metrics["tf_jump_m"],
                "filtered_jump_m": metrics["filtered_jump_m"],
                "local_jump_m": metrics["local_delta_m"],
                "gps_odom_jump_m": metrics["gps_odom_delta_m"],
                "gps_fix_jump_m": metrics["gps_fix_delta_m"],
                "base_jump_m": metrics["base_delta_m"],
                "tf_jump_yaw_rad": metrics["map_odom_yaw_delta_rad"],
                "filtered_jump_yaw_rad": metrics["filtered_yaw_delta_rad"],
            }
            current["rates"] = {
                "map_odom_speed_mps": metrics["map_odom_speed_mps"],
                "filtered_speed_mps": metrics["filtered_speed_mps"],
                "map_odom_yaw_rate_rps": metrics["map_odom_yaw_rate_rps"],
                "filtered_yaw_rate_rps": metrics["filtered_yaw_rate_rps"],
            }
        enriched.append(current)
        prev_sample = current
    return enriched


def _steering_saturation_info(sample: dict[str, Any]) -> dict[str, Any]:
    cmd = sample.get("cmd_vel_final") or sample.get("cmd_vel_nav") or {}
    telemetry = sample.get("drive_telemetry") or {}
    angular_z = float(cmd.get("angular_z", 0.0))
    linear_x = float(cmd.get("linear_x", 0.0))
    steer_deg = abs(float(telemetry.get("steer_deg_measured", 0.0)))
    curvature = 0.0
    if abs(linear_x) > 1e-3:
        curvature = abs(angular_z / linear_x)
    return {
        "cmd_linear_x": linear_x,
        "cmd_angular_z": angular_z,
        "command_curvature": curvature,
        "measured_steer_deg_abs": steer_deg,
        "measured_steer_saturated": steer_deg >= 29.0,
        "plan_mean_curvature": float((sample.get("plan_summary") or {}).get("mean_curvature", 0.0)),
        "plan_max_curvature": float((sample.get("plan_summary") or {}).get("max_curvature", 0.0)),
    }


def _heading_source_context(sample: dict[str, Any]) -> dict[str, float]:
    local_yaw = float((sample.get("headings_map_rad") or {}).get("local", 0.0))
    gps_odom_yaw = float(sample.get("gps_odom_yaw_rad", 0.0))
    imu_yaw = float((sample.get("imu_data") or {}).get("yaw_rad", 0.0))
    return {
        "local_yaw_rad": local_yaw,
        "gps_odom_yaw_rad": gps_odom_yaw,
        "imu_yaw_rad": imu_yaw,
        "local_vs_gps_odom_yaw_gap_rad": _angle_delta(local_yaw, gps_odom_yaw),
        "local_vs_imu_yaw_gap_rad": _angle_delta(local_yaw, imu_yaw),
        "gps_odom_vs_imu_yaw_gap_rad": _angle_delta(gps_odom_yaw, imu_yaw),
        "imu_angular_velocity_z": float((sample.get("imu_data") or {}).get("angular_velocity_z", 0.0)),
        "gps_odom_vs_local_position_gap_m": _distance_xy(
            tuple(sample["positions_map_xy"]["gps_odom"]),
            tuple(sample["positions_map_xy"]["local"]),
        ),
    }


def _phase_samples(samples: list[dict[str, Any]], phase: str) -> list[dict[str, Any]]:
    return [sample for sample in samples if str(sample.get("phase", "")) == phase]


def _max_excursion_xy(samples: list[dict[str, Any]], key_fn) -> float:
    if len(samples) < 2:
        return 0.0
    origin = tuple(key_fn(samples[0]))
    return max(_distance_xy(origin, tuple(key_fn(sample))) for sample in samples)


def summarize_post_goal_drift(samples: list[dict[str, Any]]) -> dict[str, Any]:
    post_goal_samples = _phase_samples(samples, "post_goal")
    if len(post_goal_samples) < 2:
        return {
            "available": False,
            "sample_count": len(post_goal_samples),
            "duration_s": 0.0,
            "classification": "insufficient_post_goal_samples",
            "map_odom_translation_drift_m": 0.0,
            "map_odom_yaw_drift_rad": 0.0,
            "global_translation_drift_m": 0.0,
            "global_yaw_drift_rad": 0.0,
            "local_translation_drift_m": 0.0,
            "local_yaw_drift_rad": 0.0,
            "gps_odom_translation_drift_m": 0.0,
            "base_translation_drift_m": 0.0,
            "max_map_odom_excursion_m": 0.0,
            "max_global_excursion_m": 0.0,
            "max_local_excursion_m": 0.0,
            "max_gps_odom_excursion_m": 0.0,
            "max_base_excursion_m": 0.0,
            "global_vs_local_gap_growth_m": 0.0,
            "global_vs_local_gap_growth_yaw_rad": 0.0,
        }

    first = post_goal_samples[0]
    last = post_goal_samples[-1]
    duration_s = max(0.0, float(last["elapsed_s"]) - float(first["elapsed_s"]))
    map_odom_translation_drift_m = _distance_xy(tuple(first["map_odom_xy"]), tuple(last["map_odom_xy"]))
    map_odom_yaw_drift_rad = _angle_delta(
        float(first["map_odom_yaw_rad"]),
        float(last["map_odom_yaw_rad"]),
    )
    global_translation_drift_m = _distance_xy(
        tuple(first["positions_map_xy"]["global"]),
        tuple(last["positions_map_xy"]["global"]),
    )
    global_yaw_drift_rad = _angle_delta(
        float(first["headings_map_rad"]["global"]),
        float(last["headings_map_rad"]["global"]),
    )
    local_translation_drift_m = _distance_xy(
        tuple(first["positions_map_xy"]["local"]),
        tuple(last["positions_map_xy"]["local"]),
    )
    local_yaw_drift_rad = _angle_delta(
        float(first["headings_map_rad"]["local"]),
        float(last["headings_map_rad"]["local"]),
    )
    gps_odom_translation_drift_m = _distance_xy(
        tuple(first["positions_map_xy"]["gps_odom"]),
        tuple(last["positions_map_xy"]["gps_odom"]),
    )
    base_translation_drift_m = _distance_xy(
        tuple(first["positions_map_xy"]["base"]),
        tuple(last["positions_map_xy"]["base"]),
    )
    global_vs_local_gap_growth_m = float(last["position_gaps_m"].get("global_vs_local", 0.0)) - float(
        first["position_gaps_m"].get("global_vs_local", 0.0)
    )
    global_vs_local_gap_growth_yaw_rad = float(last["heading_gaps_rad"].get("global_vs_local", 0.0)) - float(
        first["heading_gaps_rad"].get("global_vs_local", 0.0)
    )
    max_map_odom_excursion_m = _max_excursion_xy(post_goal_samples, lambda sample: sample["map_odom_xy"])
    max_global_excursion_m = _max_excursion_xy(post_goal_samples, lambda sample: sample["positions_map_xy"]["global"])
    max_local_excursion_m = _max_excursion_xy(post_goal_samples, lambda sample: sample["positions_map_xy"]["local"])
    max_gps_odom_excursion_m = _max_excursion_xy(post_goal_samples, lambda sample: sample["positions_map_xy"]["gps_odom"])
    max_base_excursion_m = _max_excursion_xy(post_goal_samples, lambda sample: sample["positions_map_xy"]["base"])

    if (
        max(max_map_odom_excursion_m, max_global_excursion_m) >= 0.5
        and max(max_local_excursion_m, max_base_excursion_m) <= 0.25
    ):
        classification = "global_reference_drift"
    elif max(max_local_excursion_m, max_base_excursion_m) >= 0.5:
        classification = "real_robot_motion_after_goal"
    elif max(max_map_odom_excursion_m, max_global_excursion_m, max_local_excursion_m, max_base_excursion_m) <= 0.1:
        classification = "stable_after_goal"
    else:
        classification = "mixed_small_post_goal_motion"

    return {
        "available": True,
        "sample_count": len(post_goal_samples),
        "duration_s": duration_s,
        "classification": classification,
        "map_odom_translation_drift_m": map_odom_translation_drift_m,
        "map_odom_yaw_drift_rad": map_odom_yaw_drift_rad,
        "global_translation_drift_m": global_translation_drift_m,
        "global_yaw_drift_rad": global_yaw_drift_rad,
        "local_translation_drift_m": local_translation_drift_m,
        "local_yaw_drift_rad": local_yaw_drift_rad,
        "gps_odom_translation_drift_m": gps_odom_translation_drift_m,
        "base_translation_drift_m": base_translation_drift_m,
        "max_map_odom_excursion_m": max_map_odom_excursion_m,
        "max_global_excursion_m": max_global_excursion_m,
        "max_local_excursion_m": max_local_excursion_m,
        "max_gps_odom_excursion_m": max_gps_odom_excursion_m,
        "max_base_excursion_m": max_base_excursion_m,
        "global_vs_local_gap_growth_m": global_vs_local_gap_growth_m,
        "global_vs_local_gap_growth_yaw_rad": global_vs_local_gap_growth_yaw_rad,
    }


def summarize_goal_proximity(
    samples: list[dict[str, Any]],
    *,
    goal_xy: tuple[float, float],
    xy_goal_tolerance_m: float = 0.8,
) -> dict[str, Any]:
    if not samples:
        return {
            "available": False,
            "xy_goal_tolerance_m": float(xy_goal_tolerance_m),
        }

    tracked_keys = ("global", "local", "base", "gps_odom")
    min_distance_by_key: dict[str, float] = {key: float("inf") for key in tracked_keys}
    final_distance_by_key: dict[str, float] = {}
    first_within_tolerance_elapsed_s: dict[str, float | None] = {key: None for key in tracked_keys}

    for sample in samples:
        positions = sample["positions_map_xy"]
        for key in tracked_keys:
            distance = _distance_xy(tuple(positions[key]), goal_xy)
            if distance < min_distance_by_key[key]:
                min_distance_by_key[key] = distance
            if distance <= xy_goal_tolerance_m and first_within_tolerance_elapsed_s[key] is None:
                first_within_tolerance_elapsed_s[key] = float(sample["elapsed_s"])

    last_sample = samples[-1]
    for key in tracked_keys:
        final_distance_by_key[key] = _distance_xy(tuple(last_sample["positions_map_xy"][key]), goal_xy)

    if (
        min_distance_by_key["local"] <= xy_goal_tolerance_m
        or min_distance_by_key["base"] <= xy_goal_tolerance_m
    ) and min_distance_by_key["global"] > xy_goal_tolerance_m:
        classification = "reached_in_local_but_not_global"
    elif all(min_distance_by_key[key] > xy_goal_tolerance_m for key in ("global", "local", "base")):
        classification = "never_reached_goal_tolerance"
    elif (
        final_distance_by_key["global"] > xy_goal_tolerance_m
        and min_distance_by_key["global"] <= xy_goal_tolerance_m
    ):
        classification = "entered_goal_then_moved_away"
    else:
        classification = "goal_proximity_consistent"

    return {
        "available": True,
        "xy_goal_tolerance_m": float(xy_goal_tolerance_m),
        "classification": classification,
        "min_distance_by_frame_m": {
            key: float(value if math.isfinite(value) else 0.0) for key, value in min_distance_by_key.items()
        },
        "final_distance_by_frame_m": {
            key: float(value) for key, value in final_distance_by_key.items()
        },
        "first_within_tolerance_elapsed_s": first_within_tolerance_elapsed_s,
    }


def summarize_path_tracking(samples: list[dict[str, Any]]) -> dict[str, Any]:
    tracking_samples = [sample for sample in samples if sample.get("path_tracking")]
    if not tracking_samples:
        return {"available": False}

    def _max_for(source: str, field: str) -> float:
        return max(
            float(sample["path_tracking"][source][field])
            for sample in tracking_samples
        )

    def _min_for(source: str, field: str) -> float:
        return min(
            float(sample["path_tracking"][source][field])
            for sample in tracking_samples
        )

    return {
        "available": True,
        "max_distance_to_path_m": {
            source: _max_for(source, "distance_to_path_m")
            for source in ("global", "local", "base")
        },
        "min_distance_to_path_m": {
            source: _min_for(source, "distance_to_path_m")
            for source in ("global", "local", "base")
        },
        "max_heading_error_rad": {
            source: _max_for(source, "heading_error_rad")
            for source in ("global", "local", "base")
        },
    }


def summarize_odom_alignment(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {
            "available": False,
            "max_position_gap_m": {},
            "min_position_gap_m": {},
            "max_heading_gap_rad": {},
        }
    position_pairs = ("base_vs_local", "base_vs_gps_odom", "gps_odom_vs_local")
    heading_pairs = ("base_vs_local", "base_vs_gps_odom", "gps_odom_vs_local")
    return {
        "available": True,
        "max_position_gap_m": {
            pair: max(float(sample.get("odom_position_gaps_m", {}).get(pair, 0.0)) for sample in samples)
            for pair in position_pairs
        },
        "min_position_gap_m": {
            pair: min(float(sample.get("odom_position_gaps_m", {}).get(pair, 0.0)) for sample in samples)
            for pair in position_pairs
        },
        "max_heading_gap_rad": {
            pair: max(float(sample.get("odom_heading_gaps_rad", {}).get(pair, 0.0)) for sample in samples)
            for pair in heading_pairs
        },
    }


def analyze_ekf_forensics(
    samples: list[dict[str, Any]],
    *,
    goal_status: str,
    goal_xy: tuple[float, float],
    jump_translation_threshold_m: float,
    jump_yaw_threshold_rad: float,
    window_radius: int = DEFAULT_WINDOW_RADIUS,
) -> dict[str, Any]:
    if len(samples) < 2:
        raise ValueError("need at least 2 samples")

    enriched_samples = _enrich_samples(samples)
    jump_events: list[dict[str, Any]] = []
    source_counts: dict[str, int] = {}
    mode_counts: dict[str, int] = {}
    for index in range(1, len(enriched_samples)):
        prev_sample = enriched_samples[index - 1]
        next_sample = enriched_samples[index]
        metrics = _jump_metrics(prev_sample, next_sample)
        if (
            metrics["map_odom_translation_delta_m"] < jump_translation_threshold_m
            and metrics["map_odom_yaw_delta_rad"] < jump_yaw_threshold_rad
            and metrics["filtered_delta_m"] < jump_translation_threshold_m
            and metrics["filtered_yaw_delta_rad"] < jump_yaw_threshold_rad
        ):
            continue
        classification = classify_jump_event(
            metrics,
            jump_translation_threshold_m=jump_translation_threshold_m,
            jump_yaw_threshold_rad=jump_yaw_threshold_rad,
        )
        steering = _steering_saturation_info(next_sample)
        heading_sources = _heading_source_context(next_sample)
        source_counts[classification["dominant_source"]] = (
            source_counts.get(classification["dominant_source"], 0) + 1
        )
        mode_counts[classification["dominant_error_mode"]] = (
            mode_counts.get(classification["dominant_error_mode"], 0) + 1
        )
        pre_window = [
            _compact_sample(sample)
            for sample in enriched_samples[max(0, index - window_radius):index]
        ]
        post_window = [
            _compact_sample(sample)
            for sample in enriched_samples[index:min(len(enriched_samples), index + window_radius)]
        ]
        jump_events.append(
            {
                "sample_index": index,
                "phase": str(next_sample.get("phase", "")),
                "elapsed_s": float(next_sample["elapsed_s"]),
                "classification": classification["classification"],
                "confidence": classification["confidence"],
                "dominant_source": classification["dominant_source"],
                "dominant_error_mode": classification["dominant_error_mode"],
                "metrics": metrics,
                "control_context": steering,
                "heading_source_context": heading_sources,
                "before": _compact_sample(prev_sample),
                "after": _compact_sample(next_sample),
                "pre_window": pre_window,
                "post_window": post_window,
            }
        )

    def _largest(field: str) -> float:
        if not jump_events:
            return 0.0
        return max(float(event["metrics"][field]) for event in jump_events)

    post_goal_drift = summarize_post_goal_drift(enriched_samples)
    goal_proximity = summarize_goal_proximity(
        enriched_samples,
        goal_xy=goal_xy,
        xy_goal_tolerance_m=0.8,
    )
    path_tracking = summarize_path_tracking(enriched_samples)
    odom_alignment = summarize_odom_alignment(enriched_samples)

    summary = {
        "goal_status": str(goal_status),
        "jump_count": len(jump_events),
        "dominant_source": _dominant_label(source_counts, "none_detected"),
        "dominant_error_mode": _dominant_label(mode_counts, "none_detected"),
        "first_jump_elapsed_s": float(jump_events[0]["elapsed_s"]) if jump_events else 0.0,
        "largest_map_odom_jump_m": _largest("map_odom_translation_delta_m"),
        "largest_map_odom_yaw_jump_rad": _largest("map_odom_yaw_delta_rad"),
        "largest_gps_odom_jump_m": _largest("gps_odom_delta_m"),
        "largest_local_jump_m": max(_largest("local_delta_m"), _largest("base_delta_m")),
        "largest_filtered_jump_m": _largest("filtered_delta_m"),
        "largest_stamp_gap_s": max(
            _largest("global_stamp_delta_s"),
            _largest("local_stamp_delta_s"),
            _largest("gps_odom_stamp_delta_s"),
            _largest("gps_stamp_delta_s"),
            _largest("base_stamp_delta_s"),
            _largest("largest_source_age_s"),
        ),
        "largest_global_age_s": _largest("largest_global_age_s"),
        "largest_gps_odom_age_s": _largest("largest_gps_odom_age_s"),
        "largest_local_age_s": _largest("largest_local_age_s"),
        "largest_tf_filtered_disagreement_m": max(
            _largest("tf_vs_filtered_disagreement_before_m"),
            _largest("tf_vs_filtered_disagreement_after_m"),
        ),
        "largest_tf_filtered_disagreement_yaw_rad": max(
            _largest("tf_vs_filtered_disagreement_before_yaw_rad"),
            _largest("tf_vs_filtered_disagreement_after_yaw_rad"),
        ),
        "jump_events_with_steering_saturation": sum(
            1 for event in jump_events if event["control_context"]["measured_steer_saturated"]
        ),
        "jump_events_with_high_angular_command": sum(
            1 for event in jump_events if abs(float(event["control_context"]["cmd_angular_z"])) >= 0.35
        ),
        "jump_events_with_high_command_curvature": sum(
            1 for event in jump_events if float(event["control_context"]["command_curvature"]) >= 0.25
        ),
        "largest_command_curvature": max(
            (float(event["control_context"]["command_curvature"]) for event in jump_events),
            default=0.0,
        ),
        "largest_plan_max_curvature": max(
            (float(event["control_context"]["plan_max_curvature"]) for event in jump_events),
            default=0.0,
        ),
        "largest_local_vs_gps_odom_yaw_gap_rad": max(
            (float(event["heading_source_context"]["local_vs_gps_odom_yaw_gap_rad"]) for event in jump_events),
            default=0.0,
        ),
        "largest_local_vs_imu_yaw_gap_rad": max(
            (float(event["heading_source_context"]["local_vs_imu_yaw_gap_rad"]) for event in jump_events),
            default=0.0,
        ),
        "largest_gps_odom_vs_imu_yaw_gap_rad": max(
            (float(event["heading_source_context"]["gps_odom_vs_imu_yaw_gap_rad"]) for event in jump_events),
            default=0.0,
        ),
        "jump_events_with_high_local_vs_gps_odom_yaw_gap": sum(
            1
            for event in jump_events
            if float(event["heading_source_context"]["local_vs_gps_odom_yaw_gap_rad"]) >= 0.25
        ),
        "jump_events_with_high_local_vs_imu_yaw_gap": sum(
            1
            for event in jump_events
            if float(event["heading_source_context"]["local_vs_imu_yaw_gap_rad"]) >= 0.25
        ),
        "post_goal_drift_classification": str(post_goal_drift["classification"]),
        "post_goal_map_odom_drift_m": float(post_goal_drift["map_odom_translation_drift_m"]),
        "post_goal_global_drift_m": float(post_goal_drift["global_translation_drift_m"]),
        "post_goal_local_drift_m": float(post_goal_drift["local_translation_drift_m"]),
        "post_goal_base_drift_m": float(post_goal_drift["base_translation_drift_m"]),
        "post_goal_max_map_odom_excursion_m": float(post_goal_drift["max_map_odom_excursion_m"]),
        "post_goal_max_global_excursion_m": float(post_goal_drift["max_global_excursion_m"]),
        "post_goal_max_local_excursion_m": float(post_goal_drift["max_local_excursion_m"]),
        "post_goal_max_base_excursion_m": float(post_goal_drift["max_base_excursion_m"]),
        "goal_proximity_classification": str(goal_proximity["classification"]),
        "goal_min_global_distance_m": float(goal_proximity["min_distance_by_frame_m"]["global"]),
        "goal_min_local_distance_m": float(goal_proximity["min_distance_by_frame_m"]["local"]),
        "goal_min_base_distance_m": float(goal_proximity["min_distance_by_frame_m"]["base"]),
        "goal_final_global_distance_m": float(goal_proximity["final_distance_by_frame_m"]["global"]),
        "goal_final_local_distance_m": float(goal_proximity["final_distance_by_frame_m"]["local"]),
        "goal_final_base_distance_m": float(goal_proximity["final_distance_by_frame_m"]["base"]),
        "path_tracking_available": bool(path_tracking["available"]),
        "path_max_global_distance_m": float(path_tracking.get("max_distance_to_path_m", {}).get("global", 0.0)),
        "path_max_local_distance_m": float(path_tracking.get("max_distance_to_path_m", {}).get("local", 0.0)),
        "path_max_base_distance_m": float(path_tracking.get("max_distance_to_path_m", {}).get("base", 0.0)),
        "path_max_global_heading_error_rad": float(path_tracking.get("max_heading_error_rad", {}).get("global", 0.0)),
        "path_max_local_heading_error_rad": float(path_tracking.get("max_heading_error_rad", {}).get("local", 0.0)),
        "path_max_base_heading_error_rad": float(path_tracking.get("max_heading_error_rad", {}).get("base", 0.0)),
        "odom_alignment_available": bool(odom_alignment["available"]),
        "odom_max_base_vs_local_gap_m": float(odom_alignment.get("max_position_gap_m", {}).get("base_vs_local", 0.0)),
        "odom_max_base_vs_gps_odom_gap_m": float(odom_alignment.get("max_position_gap_m", {}).get("base_vs_gps_odom", 0.0)),
        "odom_max_gps_odom_vs_local_gap_m": float(odom_alignment.get("max_position_gap_m", {}).get("gps_odom_vs_local", 0.0)),
        "odom_max_base_vs_local_yaw_gap_rad": float(odom_alignment.get("max_heading_gap_rad", {}).get("base_vs_local", 0.0)),
        "odom_max_base_vs_gps_odom_yaw_gap_rad": float(odom_alignment.get("max_heading_gap_rad", {}).get("base_vs_gps_odom", 0.0)),
        "odom_max_gps_odom_vs_local_yaw_gap_rad": float(odom_alignment.get("max_heading_gap_rad", {}).get("gps_odom_vs_local", 0.0)),
    }
    return {
        "summary": summary,
        "jump_events": jump_events,
        "sample_count": len(enriched_samples),
        "post_goal_drift": post_goal_drift,
        "goal_proximity": goal_proximity,
        "path_tracking": path_tracking,
        "odom_alignment": odom_alignment,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="High-precision forensic analyzer of EKF-global map/filtered failures in sim_global_v2.",
    )
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--goal", default=f"{DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]}")
    parser.add_argument("--bootstrap-timeout-s", type=float, default=DEFAULT_BOOTSTRAP_TIMEOUT_S)
    parser.add_argument("--goal-timeout-s", type=float, default=DEFAULT_GOAL_TIMEOUT_S)
    parser.add_argument("--post-bootstrap-settle-s", type=float, default=DEFAULT_POST_BOOTSTRAP_SETTLE_S)
    parser.add_argument("--observe-phase", choices=OBSERVE_PHASES, default=DEFAULT_OBSERVE_PHASE)
    parser.add_argument("--post-goal-duration-s", type=float, default=DEFAULT_POST_GOAL_DURATION_S)
    parser.add_argument("--sample-interval-s", type=float, default=DEFAULT_SAMPLE_INTERVAL_S)
    parser.add_argument("--jump-translation-threshold-m", type=float, default=DEFAULT_JUMP_TRANSLATION_M)
    parser.add_argument("--jump-yaw-threshold-rad", type=float, default=DEFAULT_JUMP_YAW_RAD)
    parser.add_argument("--window-radius", type=int, default=DEFAULT_WINDOW_RADIUS)
    parser.add_argument(
        "--use-rviz",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_USE_RVIZ,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def run_forensics_session(config: ForensicsRunConfig) -> tuple[dict[str, Any], int]:
    definition = _resolve_profile(config.profile)
    config_path = _resolve_config_path(config.profile)
    goal_x, goal_y = config.goal_xy

    launch_proc = None
    probe: EkfForensicsProbe | None = None
    try:
        launch_proc, launch_log_path = _launch_simulation(
            config_path=config_path,
            gps_profile=definition.gps_profile,
            gps_reference_mode=definition.gps_reference_mode,
            gps_covariance_horizontal_stddev_override_m=(
                definition.gps_covariance_horizontal_stddev_override_m
            ),
            gps_covariance_vertical_stddev_override_m=(
                definition.gps_covariance_vertical_stddev_override_m
            ),
            use_rviz=bool(config.use_rviz),
        )
        rclpy.init()
        probe = EkfForensicsProbe()
        if not probe.wait_for_bootstrap(config.bootstrap_timeout_s):
            raise RuntimeError("simulation bootstrap failed")
        probe._spin_for_duration(float(config.post_bootstrap_settle_s))

        started_at = time.monotonic()
        goal_result, during_goal_samples = probe.send_goal_and_observe(
            x=goal_x,
            y=goal_y,
            timeout_s=config.goal_timeout_s,
            sample_interval_s=config.sample_interval_s,
            observe_phase=str(config.observe_phase),
        )

        post_goal_samples: list[dict[str, Any]] = []
        if str(config.observe_phase) in {"post_goal", "full_run"}:
            post_goal_samples = probe.observe_phase(
                duration_s=config.post_goal_duration_s,
                sample_interval_s=config.sample_interval_s,
                started_at=started_at,
                phase="post_goal",
            )

        samples: list[dict[str, Any]] = []
        if str(config.observe_phase) in {"during_goal", "full_run"}:
            samples.extend(during_goal_samples)
        if str(config.observe_phase) in {"post_goal", "full_run"}:
            samples.extend(post_goal_samples)

        analysis = analyze_ekf_forensics(
            samples,
            goal_status=str(goal_result["goal_status"]),
            goal_xy=(goal_x, goal_y),
            jump_translation_threshold_m=float(config.jump_translation_threshold_m),
            jump_yaw_threshold_rad=float(config.jump_yaw_threshold_rad),
            window_radius=int(config.window_radius),
        )
        payload = {
            "profile": config.profile,
            "config_path": config_path,
            "gps_profile": definition.gps_profile,
            "gps_reference_mode": definition.gps_reference_mode,
            "goal_xy": [goal_x, goal_y],
            "goal_result": goal_result,
            "summary": analysis["summary"],
            "jump_events": analysis["jump_events"],
            "sample_count": analysis["sample_count"],
            "post_goal_drift": analysis["post_goal_drift"],
            "goal_proximity": analysis["goal_proximity"],
            "path_tracking": analysis["path_tracking"],
            "latest_diag_map": probe.latest_diag_map,
            "events_tail": probe.events[-12:],
            "launch_log_tail": Path(launch_log_path).read_text(encoding="utf-8", errors="replace").splitlines()[-80:],
        }
        text = json.dumps(payload, indent=2, sort_keys=True)
        if config.output:
            Path(config.output).write_text(text + "\n", encoding="utf-8")
        return payload, 0 if analysis["summary"]["jump_count"] == 0 else 2
    finally:
        _cleanup_process(launch_proc, timeout_s=20.0)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    args = _parse_args()
    payload, return_code = run_forensics_session(
        ForensicsRunConfig(
            profile=str(args.profile),
            goal_xy=_parse_goal(args.goal),
            bootstrap_timeout_s=float(args.bootstrap_timeout_s),
            goal_timeout_s=float(args.goal_timeout_s),
            post_bootstrap_settle_s=float(args.post_bootstrap_settle_s),
            observe_phase=str(args.observe_phase),
            post_goal_duration_s=float(args.post_goal_duration_s),
            sample_interval_s=float(args.sample_interval_s),
            jump_translation_threshold_m=float(args.jump_translation_threshold_m),
            jump_yaw_threshold_rad=float(args.jump_yaw_threshold_rad),
            window_radius=int(args.window_radius),
            use_rviz=bool(args.use_rviz),
            output=str(args.output),
        )
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
