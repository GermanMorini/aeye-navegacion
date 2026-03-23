from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from interfaces.msg import NavEvent
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class ExperimentProfile:
    config_filename: str
    gps_reference_mode: str = "ideal_from_local_odom"
    gps_profile: str = "ideal"
    gps_covariance_horizontal_stddev_override_m: float = 0.0
    gps_covariance_vertical_stddev_override_m: float = 0.0


PROFILE_TO_CONFIG = {
    "operational_default": ExperimentProfile("global_localization_v2.yaml"),
    "baseline_current": ExperimentProfile("global_localization_v2_baseline_current.yaml"),
    "motion_candidate_navsat_decoupled": ExperimentProfile(
        "global_localization_v2_motion_candidate_navsat_decoupled.yaml"
    ),
    "motion_candidate_local_vyaw_fallback": ExperimentProfile(
        "global_localization_v2_motion_candidate_local_vyaw_fallback.yaml"
    ),
    "motion_candidate_heading_only": ExperimentProfile(
        "global_localization_v2_motion_candidate_heading_only.yaml"
    ),
    "motion_candidate_no_lateral_velocity": ExperimentProfile(
        "global_localization_v2_motion_candidate_no_lateral_velocity.yaml"
    ),
    "motion_candidate_local_yaw_anchor": ExperimentProfile(
        "global_localization_v2_motion_candidate_local_yaw_anchor.yaml"
    ),
    "motion_candidate_yaw_anchor_gps_position": ExperimentProfile(
        "global_localization_v2_motion_candidate_yaw_anchor_gps_position.yaml"
    ),
    "motion_candidate_gps_position_hard_lock": ExperimentProfile(
        "global_localization_v2_motion_candidate_gps_position_hard_lock.yaml"
    ),
    "motion_candidate_lag_compensated": ExperimentProfile(
        "global_localization_v2_motion_candidate_lag_compensated.yaml"
    ),
    "rejected_reference_linear_only": ExperimentProfile(
        "global_localization_v2_rejected_reference_linear_only.yaml"
    ),
    "strict_split_twist_global": ExperimentProfile("global_localization_v2.yaml"),
    "strict_split_heading_only_global": ExperimentProfile(
        "global_localization_v2_motion_candidate_heading_only.yaml"
    ),
    "strict_split_lag_compensated": ExperimentProfile(
        "global_localization_v2_motion_candidate_lag_compensated.yaml"
    ),
    "strict_split_lag_compensated_odom_yaw": ExperimentProfile(
        "global_localization_v2_motion_candidate_lag_compensated_odom_yaw.yaml"
    ),
    "strict_split_lag_compensated_odom_yaw_no_delay": ExperimentProfile(
        "global_localization_v2_motion_candidate_lag_compensated_odom_yaw_no_delay.yaml"
    ),
    "strict_split_lag_compensated_odom_yaw_no_delay_soft_gps": ExperimentProfile(
        "global_localization_v2_motion_candidate_lag_compensated_odom_yaw_no_delay.yaml",
        gps_covariance_horizontal_stddev_override_m=0.15,
        gps_covariance_vertical_stddev_override_m=0.25,
    ),
    "strict_split_rate_matched": ExperimentProfile(
        "global_localization_v2_motion_candidate_rate_matched.yaml"
    ),
    "heading_from_local_odom": ExperimentProfile(
        "global_localization_v2_baseline_current.yaml"
    ),
    "heading_from_imu": ExperimentProfile(
        "global_localization_v2_motion_candidate_navsat_decoupled.yaml"
    ),
    "heading_decoupled_minimal": ExperimentProfile("global_localization_v2.yaml"),
    "ideal_from_local_odom": ExperimentProfile(
        "global_localization_v2.yaml", gps_reference_mode="ideal_from_local_odom"
    ),
    "ideal_from_raw_odom": ExperimentProfile(
        "global_localization_v2.yaml", gps_reference_mode="ideal_from_raw_odom"
    ),
    "ideal_from_ground_truth": ExperimentProfile(
        "global_localization_v2.yaml", gps_reference_mode="ideal_from_ground_truth"
    ),
}
DEFAULT_GOALS = [(12.0, 8.0), (15.0, 10.0), (18.0, 12.0)]
DEFAULT_BOOTSTRAP_TIMEOUT_S = 45.0
DEFAULT_GOAL_TIMEOUT_S = 60.0
DEFAULT_CHECKER_DURATION_S = 45.0
DEFAULT_CHECKER_INTERVAL_S = 0.5
DEFAULT_POST_RESULT_GRACE_S = 2.0
DEFAULT_USE_RVIZ = True
MAX_PROFILE_ATTEMPTS = 2


def _run_command(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=False)


def _tail_file(path: Path, line_count: int = 80) -> str:
    if not path.exists():
        return ""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-line_count:])


def _cleanup_process(proc: subprocess.Popen[Any] | None, *, timeout_s: float = 20.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    end = time.time() + timeout_s
    while time.time() < end:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _distance_xy(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    return float(((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2) ** 0.5)


def _snapshot_delta(
    first: dict[str, Any] | None, second: dict[str, Any] | None
) -> dict[str, float | None]:
    first = first or {}
    second = second or {}
    return {
        "global_pose_map_jump_m": _distance_xy(
            first.get("global_pose_xy"), second.get("global_pose_xy")
        ),
        "global_pose_odom_jump_m": _distance_xy(
            first.get("global_pose_xy_in_odom"), second.get("global_pose_xy_in_odom")
        ),
        "local_pose_odom_jump_m": _distance_xy(
            first.get("local_pose_xy"), second.get("local_pose_xy")
        ),
    }


class MotionBenchmarkProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_global_motion_benchmark")
        self.gps_fix: NavSatFix | None = None
        self.odom_local: Odometry | None = None
        self.odom_global: Odometry | None = None
        self.global_costmap: OccupancyGrid | None = None
        self.local_costmap: OccupancyGrid | None = None
        self.global_plan: NavPath | None = None
        self.latest_diag_map: dict[str, Any] | None = None
        self.events: list[dict[str, Any]] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps_fix, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_odom_local, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom_global, 10)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_global_costmap, 10
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap, 10
        )
        self.create_subscription(NavPath, "/plan", self._on_global_plan, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_subscription(NavEvent, "/nav_command_server/events", self._on_nav_event, 100)
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        self.gps_fix = msg

    def _on_odom_local(self, msg: Odometry) -> None:
        self.odom_local = msg

    def _on_odom_global(self, msg: Odometry) -> None:
        self.odom_global = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self.global_costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self.local_costmap = msg

    def _on_global_plan(self, msg: NavPath) -> None:
        self.global_plan = msg

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == "ekf_filter_node_map: Filter diagnostic updater":
                level = status.level
                if isinstance(level, (bytes, bytearray)):
                    level = int(level[0])
                self.latest_diag_map = {
                    "level": int(level),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }
                return

    def _on_nav_event(self, msg: NavEvent) -> None:
        severity = msg.severity
        if isinstance(severity, (bytes, bytearray)):
            severity = int(severity[0])
        self.events.append(
            {
                "time_sec": float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1_000_000_000.0,
                "severity": int(severity),
                "component": msg.component,
                "code": msg.code,
                "message": msg.message,
            }
        )

    def spin_until(self, predicate, timeout_s: float) -> bool:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_for_bootstrap(self, timeout_s: float) -> bool:
        if not self.navigate_client.wait_for_server(timeout_sec=timeout_s):
            return False
        return self.spin_until(
            lambda: self.gps_fix is not None
            and self.odom_local is not None
            and self.odom_global is not None
            and self.global_costmap is not None
            and self.local_costmap is not None
            and self.latest_diag_map is not None,
            timeout_s=timeout_s,
        )

    def _pose_xy(self, msg: Odometry | None) -> tuple[float, float] | None:
        if msg is None:
            return None
        return (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _point_to_frame_xy(
        self, xy: tuple[float, float] | None, *, source_frame: str, target_frame: str
    ) -> tuple[float, float] | None:
        if xy is None:
            return None
        if source_frame == target_frame:
            return (float(xy[0]), float(xy[1]))
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = Time().to_msg()
        point.point.x = float(xy[0])
        point.point.y = float(xy[1])
        point.point.z = 0.0
        transformed = self.tf_buffer.transform(
            point,
            target_frame,
            timeout=Duration(seconds=1.0),
        )
        return (float(transformed.point.x), float(transformed.point.y))

    def _costmap_bounds(self, costmap: OccupancyGrid | None) -> dict[str, float] | None:
        if costmap is None:
            return None
        info = costmap.info
        min_x = float(info.origin.position.x)
        min_y = float(info.origin.position.y)
        max_x = min_x + float(info.width) * float(info.resolution)
        max_y = min_y + float(info.height) * float(info.resolution)
        return {
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "width_m": float(info.width) * float(info.resolution),
            "height_m": float(info.height) * float(info.resolution),
            "resolution": float(info.resolution),
        }

    def _xy_inside_costmap(
        self, xy: tuple[float, float] | None, bounds: dict[str, float] | None
    ) -> bool | None:
        if xy is None or bounds is None:
            return None
        return (
            bounds["min_x"] <= xy[0] <= bounds["max_x"]
            and bounds["min_y"] <= xy[1] <= bounds["max_y"]
        )

    def navigation_snapshot(self) -> dict[str, Any]:
        global_costmap_bounds = self._costmap_bounds(self.global_costmap)
        local_costmap_bounds = self._costmap_bounds(self.local_costmap)
        local_xy = self._pose_xy(self.odom_local)
        global_xy = self._pose_xy(self.odom_global)
        global_xy_in_odom = self._point_to_frame_xy(
            global_xy, source_frame="map", target_frame="odom"
        )
        return {
            "local_pose_xy": list(local_xy) if local_xy is not None else None,
            "global_pose_xy": list(global_xy) if global_xy is not None else None,
            "global_pose_xy_in_odom": list(global_xy_in_odom)
            if global_xy_in_odom is not None
            else None,
            "global_costmap_bounds": global_costmap_bounds,
            "local_costmap_bounds": local_costmap_bounds,
            "global_pose_inside_global_costmap": self._xy_inside_costmap(
                global_xy, global_costmap_bounds
            ),
            "local_pose_inside_global_costmap": self._xy_inside_costmap(
                local_xy, global_costmap_bounds
            ),
            "global_pose_inside_local_costmap": self._xy_inside_costmap(
                global_xy_in_odom, local_costmap_bounds
            ),
            "local_pose_inside_local_costmap": self._xy_inside_costmap(
                local_xy, local_costmap_bounds
            ),
            "plan_pose_count": int(len(self.global_plan.poses)) if self.global_plan is not None else 0,
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
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        send_future = self.navigate_client.send_goal_async(goal)
        if not self.spin_until(lambda: send_future.done(), timeout_s=min(10.0, timeout_s)):
            raise RuntimeError("timeout waiting for navigate_to_pose goal acceptance")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            immediate_snapshot = self.navigation_snapshot()
            return {
                "accepted": False,
                "goal_status": "REJECTED",
                "terminal_event": self._terminal_event(),
                "events_tail": self.events[-12:],
                "nav_snapshot": immediate_snapshot,
                "nav_snapshot_late": immediate_snapshot,
                "nav_snapshot_delta": _snapshot_delta(immediate_snapshot, immediate_snapshot),
            }

        result_future = goal_handle.get_result_async()
        finished = self.spin_until(lambda: result_future.done(), timeout_s=timeout_s)
        if not finished:
            goal_handle.cancel_goal_async()
            immediate_snapshot = self.navigation_snapshot()
            self._spin_for_duration(post_result_grace_s)
            late_snapshot = self.navigation_snapshot()
            return {
                "accepted": True,
                "goal_status": "TIMEOUT",
                "terminal_event": self._terminal_event(),
                "events_tail": self.events[-12:],
                "nav_snapshot": immediate_snapshot,
                "nav_snapshot_late": late_snapshot,
                "nav_snapshot_delta": _snapshot_delta(immediate_snapshot, late_snapshot),
            }

        result = result_future.result()
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        immediate_snapshot = self.navigation_snapshot()
        self._spin_for_duration(post_result_grace_s)
        late_snapshot = self.navigation_snapshot()
        return {
            "accepted": True,
            "goal_status": status_map.get(result.status, str(result.status)),
            "terminal_event": self._terminal_event(),
            "events_tail": self.events[-12:],
            "nav_snapshot": immediate_snapshot,
            "nav_snapshot_late": late_snapshot,
            "nav_snapshot_delta": _snapshot_delta(immediate_snapshot, late_snapshot),
        }

    def _terminal_event(self) -> dict[str, Any] | None:
        for event in reversed(self.events):
            if event["code"] in {"GOAL_FAILED", "GOAL_COMPLETED", "GOAL_REJECTED"}:
                return event
        return None


def _parse_goal(text: str) -> tuple[float, float]:
    x_text, y_text = [part.strip() for part in text.split(",", 1)]
    return (float(x_text), float(y_text))


def _resolve_profile(profile: str) -> ExperimentProfile:
    if profile not in PROFILE_TO_CONFIG:
        valid = ", ".join(sorted(PROFILE_TO_CONFIG))
        raise ValueError(f"unsupported profile '{profile}'. Valid values: {valid}")
    return PROFILE_TO_CONFIG[profile]

def _resolve_config_path(profile: str) -> str:
    package_root = Path(__file__).resolve().parents[1]
    definition = _resolve_profile(profile)
    return str((package_root / "config" / definition.config_filename).resolve())


def _launch_simulation(
    *,
    config_path: str,
    gps_profile: str,
    gps_reference_mode: str,
    gps_covariance_horizontal_stddev_override_m: float = 0.0,
    gps_covariance_vertical_stddev_override_m: float = 0.0,
    use_rviz: bool,
) -> tuple[subprocess.Popen[Any], Path]:
    log_fd, log_path_str = tempfile.mkstemp(prefix="sim_global_motion_", suffix=".log")
    os.close(log_fd)
    log_path = Path(log_path_str)
    log_file = open(log_path, "w", encoding="utf-8")
    launch_cmd = [
        "ros2",
        "launch",
        "navegacion_gps",
        "sim_global_v2.launch.py",
        f"use_rviz:={'true' if use_rviz else 'false'}",
        f"gps_profile:={gps_profile}",
        f"gps_reference_mode:={gps_reference_mode}",
        f"gps_covariance_horizontal_stddev_override_m:={gps_covariance_horizontal_stddev_override_m}",
        f"gps_covariance_vertical_stddev_override_m:={gps_covariance_vertical_stddev_override_m}",
        f"global_localization_params_file:={config_path}",
    ]
    process = subprocess.Popen(
        launch_cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, log_path


def _run_checker(output_path: Path) -> subprocess.Popen[Any]:
    checker_cmd = [
        "ros2",
        "run",
        "navegacion_gps",
        "sim_global_consistency_check",
        "--duration-s",
        str(DEFAULT_CHECKER_DURATION_S),
        "--sample-interval-s",
        str(DEFAULT_CHECKER_INTERVAL_S),
        "--output",
        str(output_path),
    ]
    return subprocess.Popen(
        checker_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def _wait_for_checker(proc: subprocess.Popen[Any], timeout_s: float = 5.0) -> tuple[int | None, str]:
    try:
        stdout, _ = proc.communicate(timeout=timeout_s)
        return proc.returncode, stdout
    except subprocess.TimeoutExpired:
        return None, ""


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark motion stability of sim_global_v2 global localization variants.",
    )
    parser.add_argument(
        "--profile",
        action="append",
        choices=sorted(PROFILE_TO_CONFIG),
        help="Global localization profile to run. Repeat to benchmark multiple profiles.",
    )
    parser.add_argument(
        "--goal",
        action="append",
        help="Goal in map frame as x,y. Repeat for multiple goals.",
    )
    parser.add_argument("--bootstrap-timeout-s", type=float, default=DEFAULT_BOOTSTRAP_TIMEOUT_S)
    parser.add_argument("--goal-timeout-s", type=float, default=DEFAULT_GOAL_TIMEOUT_S)
    parser.add_argument("--post-result-grace-s", type=float, default=DEFAULT_POST_RESULT_GRACE_S)
    parser.add_argument(
        "--use-rviz",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_USE_RVIZ,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def _run_profile_once(profile: str, goals: list[tuple[float, float]], args: argparse.Namespace) -> dict[str, Any]:
    launch_proc = None
    checker_proc = None
    checker_output_path = Path(tempfile.mktemp(prefix=f"{profile}_", suffix="_consistency.json"))
    launch_log_path: Path | None = None
    rclpy.init()
    probe = MotionBenchmarkProbe()
    try:
        definition = _resolve_profile(profile)
        config_path = _resolve_config_path(profile)
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
            use_rviz=bool(args.use_rviz),
        )
        if not probe.wait_for_bootstrap(args.bootstrap_timeout_s):
            raise RuntimeError("simulation bootstrap failed")
        settle_end = time.time() + 3.0
        while time.time() < settle_end:
            rclpy.spin_once(probe, timeout_sec=0.1)
        checker_proc = _run_checker(checker_output_path)
        goal_results = []
        for x, y in goals:
            goal_result = probe.send_goal(
                x=x,
                y=y,
                timeout_s=args.goal_timeout_s,
                post_result_grace_s=args.post_result_grace_s,
            )
            goal_result["goal_xy"] = [float(x), float(y)]
            goal_results.append(goal_result)
            if goal_result["goal_status"] != "SUCCEEDED":
                break
            if checker_proc.poll() is not None:
                break

        checker_returncode, checker_stdout = _wait_for_checker(checker_proc, timeout_s=5.0)
        if checker_returncode is None:
            _cleanup_process(checker_proc, timeout_s=5.0)
            checker_returncode, checker_stdout = _wait_for_checker(checker_proc, timeout_s=1.0)

        checker_report: dict[str, Any] | None = None
        if checker_output_path.exists():
            checker_report = json.loads(checker_output_path.read_text(encoding="utf-8"))

        return {
            "profile": profile,
            "gps_profile": definition.gps_profile,
            "gps_reference_mode": definition.gps_reference_mode,
            "config_path": config_path,
            "checker_returncode": checker_returncode,
            "checker_stdout_tail": checker_stdout.splitlines()[-20:],
            "checker_report": checker_report,
            "goal_results": goal_results,
            "launch_log_tail": _tail_file(launch_log_path) if launch_log_path else "",
            "latest_diag_map": probe.latest_diag_map,
            "events_tail": probe.events[-12:],
        }
    finally:
        _cleanup_process(checker_proc, timeout_s=5.0)
        _cleanup_process(launch_proc, timeout_s=20.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_profile(profile: str, goals: list[tuple[float, float]], args: argparse.Namespace) -> dict[str, Any]:
    last_error: RuntimeError | None = None
    for attempt in range(1, MAX_PROFILE_ATTEMPTS + 1):
        try:
            result = _run_profile_once(profile, goals, args)
            result["attempt"] = attempt
            return result
        except RuntimeError as exc:
            last_error = exc
            if attempt >= MAX_PROFILE_ATTEMPTS:
                raise
            print(f"[benchmark] retrying profile={profile} after error: {exc}")
            time.sleep(2.0)
    raise last_error if last_error is not None else RuntimeError("benchmark failed")


def _choose_winner(results: list[dict[str, Any]]) -> str:
    status_rank = {
        "SUCCEEDED": 0,
        "TIMEOUT": 1,
        "CANCELED": 2,
        "ABORTED": 3,
        "REJECTED": 4,
    }

    def _score(item: dict[str, Any]) -> tuple[int, int, float, float, float]:
        statuses = [result["goal_status"] for result in item["goal_results"]]
        best_status = min((status_rank.get(status, 99) for status in statuses), default=99)
        succeeded = sum(1 for status in statuses if status == "SUCCEEDED")
        worst_status = max((status_rank.get(status, 99) for status in statuses), default=99)
        report = item.get("checker_report") or {}
        summary = report.get("summary") or {}
        core_pos = float(summary.get("core_position_gap_m", summary.get("max_position_gap_m", 999.0)))
        core_heading = float(summary.get("core_heading_gap_rad", summary.get("max_heading_gap_rad", 999.0)))
        max_map_jump = max(
            float((result.get("nav_snapshot_delta") or {}).get("global_pose_map_jump_m") or 999.0)
            for result in item["goal_results"]
        )
        return (best_status, -succeeded, worst_status, max_map_jump, core_pos, core_heading)

    return min(results, key=_score)["profile"]


def main() -> int:
    args = _parse_args()
    profiles = args.profile or [
        "strict_split_twist_global",
        "strict_split_heading_only_global",
        "strict_split_lag_compensated",
        "heading_from_local_odom",
        "heading_from_imu",
        "heading_decoupled_minimal",
        "ideal_from_local_odom",
        "ideal_from_raw_odom",
        "ideal_from_ground_truth",
    ]
    goals = [_parse_goal(text) for text in (args.goal or [])] or list(DEFAULT_GOALS)

    results = []
    for profile in profiles:
        print(f"[benchmark] running profile={profile}")
        results.append(_run_profile(profile, goals, args))

    payload = {
        "profiles": profiles,
        "goals": [[float(x), float(y)] for x, y in goals],
        "winner": _choose_winner(results),
        "results": results,
    }
    text = json.dumps(payload, indent=2, sort_keys=True)
    print(text)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
