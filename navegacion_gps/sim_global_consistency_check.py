from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any, Optional

from diagnostic_msgs.msg import DiagnosticArray
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from interfaces.msg import NavEvent
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener

DEFAULT_DURATION_S = 60.0
DEFAULT_SAMPLE_INTERVAL_S = 0.5
DEFAULT_BOOTSTRAP_TIMEOUT_S = 30.0
DEFAULT_WARN_POSITION_GAP_M = 1.0
DEFAULT_FAIL_POSITION_GAP_M = 3.0
DEFAULT_WARN_HEADING_GAP_RAD = 0.35
DEFAULT_FAIL_HEADING_GAP_RAD = 0.8


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _mean(values: list[float]) -> float:
    return sum(values) / float(len(values)) if values else 0.0


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_gap_rad(a: float, b: float) -> float:
    return abs(_normalize_angle(a - b))


def align_base_pose_to_odom(
    *,
    raw_xy: tuple[float, float],
    raw_yaw: float,
    raw_origin_xy: tuple[float, float],
    raw_origin_yaw: float,
    odom_origin_xy: tuple[float, float],
    odom_origin_yaw: float,
) -> tuple[tuple[float, float], float]:
    return (
        (
            float(odom_origin_xy[0] + raw_xy[0] - raw_origin_xy[0]),
            float(odom_origin_xy[1] + raw_xy[1] - raw_origin_xy[1]),
        ),
        _normalize_angle(float(odom_origin_yaw + raw_yaw - raw_origin_yaw)),
    )


def _pair_key(a: str, b: str) -> str:
    left, right = sorted((str(a), str(b)))
    return f"{left}_vs_{right}"


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _pairwise_scalar_gaps(values: dict[str, float]) -> dict[str, float]:
    keys = sorted(values.keys())
    gaps: dict[str, float] = {}
    for index, left in enumerate(keys):
        for right in keys[index + 1 :]:
            gaps[_pair_key(left, right)] = abs(float(values[left]) - float(values[right]))
    return gaps


def pairwise_position_gaps(
    positions: dict[str, tuple[float, float]]
) -> dict[str, float]:
    keys = sorted(positions.keys())
    gaps: dict[str, float] = {}
    for index, left in enumerate(keys):
        for right in keys[index + 1 :]:
            gaps[_pair_key(left, right)] = _distance_xy(positions[left], positions[right])
    return gaps


def pairwise_heading_gaps(headings: dict[str, float]) -> dict[str, float]:
    keys = sorted(headings.keys())
    gaps: dict[str, float] = {}
    for index, left in enumerate(keys):
        for right in keys[index + 1 :]:
            gaps[_pair_key(left, right)] = _angle_gap_rad(headings[left], headings[right])
    return gaps


def classify_consistency_level(
    *,
    max_position_gap_m: float,
    max_heading_gap_rad: float,
    warn_position_gap_m: float,
    fail_position_gap_m: float,
    warn_heading_gap_rad: float,
    fail_heading_gap_rad: float,
) -> str:
    if fail_position_gap_m < warn_position_gap_m:
        raise ValueError("fail_position_gap_m must be >= warn_position_gap_m")
    if fail_heading_gap_rad < warn_heading_gap_rad:
        raise ValueError("fail_heading_gap_rad must be >= warn_heading_gap_rad")
    if (
        max_position_gap_m >= fail_position_gap_m
        or max_heading_gap_rad >= fail_heading_gap_rad
    ):
        return "fail"
    if (
        max_position_gap_m >= warn_position_gap_m
        or max_heading_gap_rad >= warn_heading_gap_rad
    ):
        return "warn"
    return "ok"


def infer_likely_divergence_origin(
    *,
    position_gaps_m: dict[str, float],
    heading_gaps_rad: dict[str, float],
    warn_position_gap_m: float,
    warn_heading_gap_rad: float,
) -> str:
    scores = {
        "local_pose": 0.0,
        "global_pose": 0.0,
        "gps_fix_pose": 0.0,
        "gps_odom_pose": 0.0,
        "base_odometry": 0.0,
    }
    source_aliases = {
        "local": "local_pose",
        "global": "global_pose",
        "gps": "gps_fix_pose",
        "gps_odom": "gps_odom_pose",
        "base": "base_odometry",
    }

    for pair_key, value in position_gaps_m.items():
        if value < warn_position_gap_m:
            continue
        left, _, right = pair_key.partition("_vs_")
        scores[source_aliases[left]] += float(value)
        scores[source_aliases[right]] += float(value)

    for pair_key, value in heading_gaps_rad.items():
        if value < warn_heading_gap_rad:
            continue
        left, _, right = pair_key.partition("_vs_")
        # Heading only exists on local/global/base, so it adds weaker evidence.
        weighted = float(value) * 0.5
        scores[source_aliases[left]] += weighted
        scores[source_aliases[right]] += weighted

    ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    top_name, top_score = ranked[0]
    second_score = ranked[1][1]
    if top_score <= 0.0:
        return "aligned"
    if second_score <= 0.0 or top_score >= second_score * 1.25:
        return top_name
    return "mixed_or_inconclusive"


def _max_pair_value(values: dict[str, float], allowed_pairs: tuple[str, ...]) -> tuple[str, float]:
    filtered = {pair: float(values.get(pair, 0.0)) for pair in allowed_pairs}
    if not filtered:
        return ("", 0.0)
    pair, value = max(filtered.items(), key=lambda item: item[1])
    return (str(pair), float(value))


def build_report(
    *,
    result: dict[str, Any],
    duration_s: float,
    warn_position_gap_m: float,
    fail_position_gap_m: float,
    warn_heading_gap_rad: float,
    fail_heading_gap_rad: float,
) -> dict[str, Any]:
    max_position_gap_m = float(result["max_position_gap_m"])
    max_heading_gap_rad = float(result["max_heading_gap_rad"])
    level = classify_consistency_level(
        max_position_gap_m=max_position_gap_m,
        max_heading_gap_rad=max_heading_gap_rad,
        warn_position_gap_m=warn_position_gap_m,
        fail_position_gap_m=fail_position_gap_m,
        warn_heading_gap_rad=warn_heading_gap_rad,
        fail_heading_gap_rad=fail_heading_gap_rad,
    )
    likely_origin = infer_likely_divergence_origin(
        position_gaps_m=result["max_position_gaps_m"],
        heading_gaps_rad=result["max_heading_gaps_rad"],
        warn_position_gap_m=warn_position_gap_m,
        warn_heading_gap_rad=warn_heading_gap_rad,
    )
    breach = result.get("threshold_breach") or {}
    core_position_pair, core_position_gap = _max_pair_value(
        result.get("max_position_gaps_m", {}),
        ("global_vs_local", "global_vs_gps_odom", "gps_odom_vs_local"),
    )
    core_heading_pair, core_heading_gap = _max_pair_value(
        result.get("max_heading_gaps_rad", {}),
        ("global_vs_local",),
    )
    return {
        "ok": level != "fail",
        "level": level,
        "stopped_early": bool(result.get("stopped_early", False)),
        "duration_s": float(duration_s),
        "elapsed_s": float(result["elapsed_s"]),
        "thresholds": {
            "warn_position_gap_m": float(warn_position_gap_m),
            "fail_position_gap_m": float(fail_position_gap_m),
            "warn_heading_gap_rad": float(warn_heading_gap_rad),
            "fail_heading_gap_rad": float(fail_heading_gap_rad),
        },
        "summary": {
            "sample_count": int(result["sample_count"]),
            "max_position_gap_m": max_position_gap_m,
            "max_position_pair": str(result["max_position_pair"]),
            "max_heading_gap_rad": max_heading_gap_rad,
            "max_heading_pair": str(result["max_heading_pair"]),
            "max_source_age_s": float(result.get("max_source_age_s", 0.0)),
            "max_stamp_delta_s": float(result.get("max_stamp_delta_s", 0.0)),
            "max_stamp_delta_pair": str(result.get("max_stamp_delta_pair", "")),
            "core_position_gap_m": core_position_gap,
            "core_position_pair": core_position_pair,
            "core_heading_gap_rad": core_heading_gap,
            "core_heading_pair": core_heading_pair,
            "likely_origin": likely_origin,
            "threshold_breach_pair": str(breach.get("pair", "")),
            "threshold_breach_type": str(breach.get("type", "")),
            "threshold_breach_value": float(breach.get("value", 0.0)),
        },
        "details": result,
    }


class GlobalConsistencyProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_global_consistency_check")
        self.gps_fix: Optional[NavSatFix] = None
        self.odom_gps: Optional[Odometry] = None
        self.odom_local: Optional[Odometry] = None
        self.odom_global: Optional[Odometry] = None
        self.odom_base: Optional[Odometry] = None
        self.latest_diag_map: Optional[dict[str, Any]] = None
        self.events: list[dict[str, Any]] = []
        self._base_alignment_odom: Optional[dict[str, Any]] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps_fix, 10)
        self.create_subscription(Odometry, "/odometry/gps", self._on_odom_gps, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_odom_local, 10)
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom_global, 10
        )
        self.create_subscription(Odometry, "/odom_raw", self._on_odom_base, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_subscription(NavEvent, "/nav_command_server/events", self._on_nav_event, 100)
        self._fromll_clients = [
            ("/fromLL", self.create_client(FromLL, "/fromLL")),
            (
                "/navsat_transform/fromLL",
                self.create_client(FromLL, "/navsat_transform/fromLL"),
            ),
        ]
        self._active_fromll_client = None
        self._active_fromll_service_name = ""

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        self.gps_fix = msg

    def _on_odom_local(self, msg: Odometry) -> None:
        self.odom_local = msg

    def _on_odom_gps(self, msg: Odometry) -> None:
        self.odom_gps = msg

    def _on_odom_global(self, msg: Odometry) -> None:
        self.odom_global = msg

    def _on_odom_base(self, msg: Odometry) -> None:
        self.odom_base = msg

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

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _select_fromll_client(self, timeout_s: float) -> bool:
        if self._active_fromll_client is not None:
            return True
        end = time.time() + timeout_s
        while time.time() < end:
            for service_name, client in self._fromll_clients:
                if client.wait_for_service(timeout_sec=0.2):
                    self._active_fromll_client = client
                    self._active_fromll_service_name = service_name
                    self.get_logger().info(f"Using fromLL service: {service_name}")
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_bootstrap(self, timeout_s: float) -> bool:
        if not self._select_fromll_client(timeout_s):
            return False

        def _ready() -> bool:
            if (
                self.gps_fix is None
                or self.odom_gps is None
                or self.odom_local is None
                or self.odom_global is None
                or self.odom_base is None
                or self.latest_diag_map is None
            ):
                return False
            try:
                self.lookup_xy("map", "odom", timeout_s=0.2)
            except Exception:
                return False
            return True

        return self._spin_until(_ready, timeout_s=timeout_s)

    def lookup_xy(
        self, target_frame: str, source_frame: str, timeout_s: float = 1.0
    ) -> tuple[float, float]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def fromll_to_map_xy(self, lat: float, lon: float, timeout_s: float = 5.0) -> tuple[float, float]:
        if self._active_fromll_client is None:
            raise RuntimeError("fromLL service is not ready")
        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        future = self._active_fromll_client.call_async(request)
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                stamped_point = PointStamped()
                stamped_point.header.frame_id = "odom"
                stamped_point.header.stamp = Time().to_msg()
                stamped_point.point.x = float(response.map_point.x)
                stamped_point.point.y = float(response.map_point.y)
                stamped_point.point.z = 0.0
                transformed = self.tf_buffer.transform(
                    stamped_point,
                    "map",
                    timeout=Duration(seconds=1.0),
                )
                return (float(transformed.point.x), float(transformed.point.y))
        raise RuntimeError("timeout waiting for fromLL response")

    def _frame_or_default(self, frame_id: str, default_frame: str) -> str:
        normalized = str(frame_id).strip()
        return normalized if normalized else default_frame

    def _odom_pose_to_map_xy(
        self, msg: Odometry, *, default_frame: str, timeout_s: float = 1.0
    ) -> tuple[float, float]:
        source_frame = self._frame_or_default(msg.header.frame_id, default_frame)
        return self._frame_xy_to_map_xy(
            (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            ),
            source_frame=source_frame,
            timeout_s=timeout_s,
        )

    def _odom_pose_to_map_yaw(
        self, msg: Odometry, *, default_frame: str, timeout_s: float = 1.0
    ) -> float:
        source_frame = self._frame_or_default(msg.header.frame_id, default_frame)
        pose_yaw = _quaternion_to_yaw(
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        return self._frame_yaw_to_map_yaw(
            pose_yaw,
            source_frame=source_frame,
            timeout_s=timeout_s,
        )

    def _frame_xy_to_map_xy(
        self,
        xy: tuple[float, float],
        *,
        source_frame: str,
        timeout_s: float = 1.0,
    ) -> tuple[float, float]:
        if source_frame == "map":
            return (float(xy[0]), float(xy[1]))
        stamped_point = PointStamped()
        stamped_point.header.frame_id = source_frame
        stamped_point.header.stamp = Time().to_msg()
        stamped_point.point.x = float(xy[0])
        stamped_point.point.y = float(xy[1])
        stamped_point.point.z = 0.0
        transformed = self.tf_buffer.transform(
            stamped_point,
            "map",
            timeout=Duration(seconds=timeout_s),
        )
        return (float(transformed.point.x), float(transformed.point.y))

    def _frame_yaw_to_map_yaw(
        self,
        yaw_rad: float,
        *,
        source_frame: str,
        timeout_s: float = 1.0,
    ) -> float:
        if source_frame == "map":
            return float(yaw_rad)
        transform = self.tf_buffer.lookup_transform(
            "map",
            source_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        transform_yaw = _quaternion_to_yaw(
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )
        return _normalize_angle(transform_yaw + float(yaw_rad))

    def _odom_pose_xy(self, msg: Odometry) -> tuple[float, float]:
        return (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _odom_pose_yaw(self, msg: Odometry) -> float:
        return _quaternion_to_yaw(
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )

    def _base_odom_to_map_xy(
        self,
        msg: Odometry,
        *,
        reference_odom_xy: tuple[float, float],
        reference_odom_yaw: float,
    ) -> tuple[float, float]:
        raw_xy = self._odom_pose_xy(msg)
        raw_yaw = self._odom_pose_yaw(msg)
        if self._base_alignment_odom is None:
            # /odom_raw is published in Gazebo's own namespaced odom frame, not
            # in Nav2's "odom". Align it once in odom coordinates, then apply
            # the live map->odom transform so the checker doesn't blame
            # /odom_raw when the global stack itself moves map->odom.
            self._base_alignment_odom = {
                "raw_origin_xy": raw_xy,
                "raw_origin_yaw": raw_yaw,
                "odom_origin_xy": reference_odom_xy,
                "odom_origin_yaw": reference_odom_yaw,
            }
        aligned_odom_xy, _ = align_base_pose_to_odom(
            raw_xy=raw_xy,
            raw_yaw=raw_yaw,
            raw_origin_xy=self._base_alignment_odom["raw_origin_xy"],
            raw_origin_yaw=self._base_alignment_odom["raw_origin_yaw"],
            odom_origin_xy=self._base_alignment_odom["odom_origin_xy"],
            odom_origin_yaw=self._base_alignment_odom["odom_origin_yaw"],
        )
        return self._frame_xy_to_map_xy(aligned_odom_xy, source_frame="odom")

    def _base_odom_to_map_yaw(
        self,
        msg: Odometry,
        *,
        reference_odom_xy: tuple[float, float],
        reference_odom_yaw: float,
    ) -> float:
        raw_xy = self._odom_pose_xy(msg)
        raw_yaw = self._odom_pose_yaw(msg)
        if self._base_alignment_odom is None:
            self._base_alignment_odom = {
                "raw_origin_xy": raw_xy,
                "raw_origin_yaw": raw_yaw,
                "odom_origin_xy": reference_odom_xy,
                "odom_origin_yaw": reference_odom_yaw,
            }
        _, aligned_odom_yaw = align_base_pose_to_odom(
            raw_xy=raw_xy,
            raw_yaw=raw_yaw,
            raw_origin_xy=self._base_alignment_odom["raw_origin_xy"],
            raw_origin_yaw=self._base_alignment_odom["raw_origin_yaw"],
            odom_origin_xy=self._base_alignment_odom["odom_origin_xy"],
            odom_origin_yaw=self._base_alignment_odom["odom_origin_yaw"],
        )
        return self._frame_yaw_to_map_yaw(aligned_odom_yaw, source_frame="odom")

    def capture_sample(self) -> dict[str, Any]:
        if (
            self.gps_fix is None
            or self.odom_gps is None
            or self.odom_local is None
            or self.odom_global is None
            or self.odom_base is None
        ):
            raise RuntimeError("topics not ready")

        local_odom_xy = self._odom_pose_xy(self.odom_local)
        local_odom_yaw = self._odom_pose_yaw(self.odom_local)
        gps_odom_xy = self._odom_pose_xy(self.odom_gps)
        gps_odom_yaw = self._odom_pose_yaw(self.odom_gps)
        raw_base_xy = self._odom_pose_xy(self.odom_base)
        raw_base_yaw = self._odom_pose_yaw(self.odom_base)
        if self._base_alignment_odom is None:
            self._base_alignment_odom = {
                "raw_origin_xy": raw_base_xy,
                "raw_origin_yaw": raw_base_yaw,
                "odom_origin_xy": local_odom_xy,
                "odom_origin_yaw": local_odom_yaw,
            }
        base_odom_xy, base_odom_yaw = align_base_pose_to_odom(
            raw_xy=raw_base_xy,
            raw_yaw=raw_base_yaw,
            raw_origin_xy=self._base_alignment_odom["raw_origin_xy"],
            raw_origin_yaw=self._base_alignment_odom["raw_origin_yaw"],
            odom_origin_xy=self._base_alignment_odom["odom_origin_xy"],
            odom_origin_yaw=self._base_alignment_odom["odom_origin_yaw"],
        )
        local_map_xy = self._odom_pose_to_map_xy(self.odom_local, default_frame="odom")
        local_map_yaw = self._odom_pose_to_map_yaw(self.odom_local, default_frame="odom")
        gps_odom_map_xy = self._odom_pose_to_map_xy(self.odom_gps, default_frame="odom")
        global_map_xy = self._odom_pose_to_map_xy(self.odom_global, default_frame="map")
        global_map_yaw = self._odom_pose_to_map_yaw(self.odom_global, default_frame="map")
        positions = {
            "local": local_map_xy,
            "global": global_map_xy,
            "gps": self.fromll_to_map_xy(self.gps_fix.latitude, self.gps_fix.longitude),
            "gps_odom": gps_odom_map_xy,
            "base": self._base_odom_to_map_xy(
                self.odom_base,
                reference_odom_xy=local_odom_xy,
                reference_odom_yaw=local_odom_yaw,
            ),
        }
        odom_positions = {
            "local": local_odom_xy,
            "gps_odom": gps_odom_xy,
            "base": base_odom_xy,
        }
        headings = {
            "local": local_map_yaw,
            "global": global_map_yaw,
            "base": self._base_odom_to_map_yaw(
                self.odom_base,
                reference_odom_xy=local_odom_xy,
                reference_odom_yaw=local_odom_yaw,
            ),
        }
        odom_headings = {
            "local": local_odom_yaw,
            "gps_odom": gps_odom_yaw,
            "base": base_odom_yaw,
        }
        source_stamps_s = {
            "gps": _stamp_to_seconds(self.gps_fix.header.stamp),
            "gps_odom": _stamp_to_seconds(self.odom_gps.header.stamp),
            "local": _stamp_to_seconds(self.odom_local.header.stamp),
            "global": _stamp_to_seconds(self.odom_global.header.stamp),
            "base": _stamp_to_seconds(self.odom_base.header.stamp),
        }
        freshest_stamp_s = max(source_stamps_s.values(), default=0.0)
        source_ages_s = {
            key: max(0.0, freshest_stamp_s - stamp_s) for key, stamp_s in source_stamps_s.items()
        }
        stamp_gaps_s = _pairwise_scalar_gaps(source_stamps_s)
        position_gaps = pairwise_position_gaps(positions)
        heading_gaps = pairwise_heading_gaps(headings)
        odom_position_gaps = pairwise_position_gaps(odom_positions)
        odom_heading_gaps = pairwise_heading_gaps(odom_headings)
        return {
            "positions_map_xy": {
                key: [float(value[0]), float(value[1])] for key, value in positions.items()
            },
            "headings_map_rad": {key: float(value) for key, value in headings.items()},
            "positions_odom_xy": {
                key: [float(value[0]), float(value[1])] for key, value in odom_positions.items()
            },
            "headings_odom_rad": {key: float(value) for key, value in odom_headings.items()},
            "source_stamps_s": {key: float(value) for key, value in source_stamps_s.items()},
            "source_ages_s": {key: float(value) for key, value in source_ages_s.items()},
            "stamp_gaps_s": {key: float(value) for key, value in stamp_gaps_s.items()},
            "position_gaps_m": {key: float(value) for key, value in position_gaps.items()},
            "heading_gaps_rad": {key: float(value) for key, value in heading_gaps.items()},
            "odom_position_gaps_m": {key: float(value) for key, value in odom_position_gaps.items()},
            "odom_heading_gaps_rad": {key: float(value) for key, value in odom_heading_gaps.items()},
            "latest_diag_map": self.latest_diag_map,
            "latest_event": self.events[-1] if self.events else None,
        }

    def sample_consistency(
        self,
        *,
        duration_s: float,
        sample_interval_s: float,
        warn_position_gap_m: float,
        fail_position_gap_m: float,
        warn_heading_gap_rad: float,
        fail_heading_gap_rad: float,
    ) -> dict[str, Any]:
        samples: list[dict[str, Any]] = []
        max_position_gaps: dict[str, float] = {}
        max_heading_gaps: dict[str, float] = {}
        max_source_ages: dict[str, float] = {}
        max_stamp_gaps: dict[str, float] = {}
        threshold_breach: Optional[dict[str, Any]] = None
        stopped_early = False
        started_at = time.time()
        deadline = started_at + duration_s

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                sample = self.capture_sample()
            except Exception:
                time.sleep(0.1)
                continue

            elapsed_s = max(0.0, time.time() - started_at)
            sample["elapsed_s"] = float(elapsed_s)
            samples.append(sample)

            for pair_key, value in sample["position_gaps_m"].items():
                max_position_gaps[pair_key] = max(
                    float(value), float(max_position_gaps.get(pair_key, 0.0))
                )
                if threshold_breach is None and value >= fail_position_gap_m:
                    threshold_breach = {
                        "type": "position_gap_m",
                        "pair": pair_key,
                        "value": float(value),
                        "elapsed_s": float(elapsed_s),
                    }
                    stopped_early = True
                    break
            if threshold_breach is not None:
                break

            for pair_key, value in sample["heading_gaps_rad"].items():
                max_heading_gaps[pair_key] = max(
                    float(value), float(max_heading_gaps.get(pair_key, 0.0))
                )
                if threshold_breach is None and value >= fail_heading_gap_rad:
                    threshold_breach = {
                        "type": "heading_gap_rad",
                        "pair": pair_key,
                        "value": float(value),
                        "elapsed_s": float(elapsed_s),
                    }
                    stopped_early = True
                    break
            if threshold_breach is not None:
                break

            for source_key, value in sample["source_ages_s"].items():
                max_source_ages[source_key] = max(
                    float(value), float(max_source_ages.get(source_key, 0.0))
                )
            for pair_key, value in sample["stamp_gaps_s"].items():
                max_stamp_gaps[pair_key] = max(
                    float(value), float(max_stamp_gaps.get(pair_key, 0.0))
                )

            time.sleep(sample_interval_s)

        if not samples:
            raise RuntimeError("insufficient consistency samples")

        last_sample = samples[-1]
        max_position_pair = max(
            max_position_gaps.items(),
            key=lambda item: item[1],
            default=("", 0.0),
        )
        max_heading_pair = max(
            max_heading_gaps.items(),
            key=lambda item: item[1],
            default=("", 0.0),
        )
        max_stamp_pair = max(
            max_stamp_gaps.items(),
            key=lambda item: item[1],
            default=("", 0.0),
        )
        final_level = classify_consistency_level(
            max_position_gap_m=float(max_position_pair[1]),
            max_heading_gap_rad=float(max_heading_pair[1]),
            warn_position_gap_m=warn_position_gap_m,
            fail_position_gap_m=fail_position_gap_m,
            warn_heading_gap_rad=warn_heading_gap_rad,
            fail_heading_gap_rad=fail_heading_gap_rad,
        )
        return {
            "sample_count": len(samples),
            "elapsed_s": float(last_sample["elapsed_s"]),
            "fromll_service": self._active_fromll_service_name,
            "first_sample": samples[0],
            "last_sample": last_sample,
            "latest_diag_map": self.latest_diag_map,
            "terminal_event": next(
                (
                    event
                    for event in reversed(self.events)
                    if event["code"] in {"GOAL_FAILED", "GOAL_COMPLETED", "GOAL_REJECTED"}
                ),
                None,
            ),
            "last_events": self.events[-12:],
            "max_position_gaps_m": max_position_gaps,
            "max_heading_gaps_rad": max_heading_gaps,
            "mean_position_gaps_m": {
                pair_key: _mean(
                    [float(sample["position_gaps_m"].get(pair_key, 0.0)) for sample in samples]
                )
                for pair_key in sorted(max_position_gaps.keys())
            },
            "mean_heading_gaps_rad": {
                pair_key: _mean(
                    [float(sample["heading_gaps_rad"].get(pair_key, 0.0)) for sample in samples]
                )
                for pair_key in sorted(max_heading_gaps.keys())
            },
            "max_position_gap_m": float(max_position_pair[1]),
            "max_position_pair": str(max_position_pair[0]),
            "max_heading_gap_rad": float(max_heading_pair[1]),
            "max_heading_pair": str(max_heading_pair[0]),
            "max_source_ages_s": max_source_ages,
            "max_source_age_s": max(max_source_ages.values(), default=0.0),
            "max_stamp_gaps_s": max_stamp_gaps,
            "max_stamp_delta_s": float(max_stamp_pair[1]),
            "max_stamp_delta_pair": str(max_stamp_pair[0]),
            "threshold_breach": threshold_breach,
            "stopped_early": stopped_early,
            "level": final_level,
        }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure active consistency between local pose, global pose, GPS, "
            "and base odometry during an already-running sim_global_v2 session."
        ),
    )
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument(
        "--sample-interval-s",
        type=float,
        default=DEFAULT_SAMPLE_INTERVAL_S,
    )
    parser.add_argument(
        "--bootstrap-timeout-s",
        type=float,
        default=DEFAULT_BOOTSTRAP_TIMEOUT_S,
    )
    parser.add_argument(
        "--warn-position-gap-m",
        type=float,
        default=DEFAULT_WARN_POSITION_GAP_M,
    )
    parser.add_argument(
        "--fail-position-gap-m",
        type=float,
        default=DEFAULT_FAIL_POSITION_GAP_M,
    )
    parser.add_argument(
        "--warn-heading-gap-rad",
        type=float,
        default=DEFAULT_WARN_HEADING_GAP_RAD,
    )
    parser.add_argument(
        "--fail-heading-gap-rad",
        type=float,
        default=DEFAULT_FAIL_HEADING_GAP_RAD,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = GlobalConsistencyProbe()
    try:
        if not node.wait_for_bootstrap(args.bootstrap_timeout_s):
            raise RuntimeError(
                "timed out waiting for sim_global_v2 topics, TF, and fromLL"
            )
        result = node.sample_consistency(
            duration_s=args.duration_s,
            sample_interval_s=args.sample_interval_s,
            warn_position_gap_m=args.warn_position_gap_m,
            fail_position_gap_m=args.fail_position_gap_m,
            warn_heading_gap_rad=args.warn_heading_gap_rad,
            fail_heading_gap_rad=args.fail_heading_gap_rad,
        )
        report = build_report(
            result=result,
            duration_s=args.duration_s,
            warn_position_gap_m=args.warn_position_gap_m,
            fail_position_gap_m=args.fail_position_gap_m,
            warn_heading_gap_rad=args.warn_heading_gap_rad,
            fail_heading_gap_rad=args.fail_heading_gap_rad,
        )
        summary = report["summary"]
        print(
            "sim_global_v2 consistency "
            f"[{report['level'].upper()}] "
            f"max_pos_gap={summary['max_position_gap_m']:.3f} m "
            f"({summary['max_position_pair']}), "
            f"max_heading_gap={summary['max_heading_gap_rad']:.3f} rad "
            f"({summary['max_heading_pair']}), "
            f"likely_origin={summary['likely_origin']}, "
            f"stopped_early={int(report['stopped_early'])}"
        )
        payload = json.dumps(report, indent=2, sort_keys=True)
        print(payload)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as handle:
                handle.write(payload)
                handle.write("\n")
        return 2 if report["level"] == "fail" else 0
    except Exception as exc:
        print(f"sim_global_v2 consistency check failed: {exc}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
