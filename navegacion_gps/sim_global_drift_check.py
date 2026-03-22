from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any, Optional

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener

DEFAULT_DURATION_S = 20.0
DEFAULT_SAMPLE_INTERVAL_S = 2.0
DEFAULT_BOOTSTRAP_TIMEOUT_S = 30.0
DEFAULT_WARN_THRESHOLD_M = 0.5
DEFAULT_FAIL_THRESHOLD_M = 1.5


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _mean(values: list[float]) -> float:
    return sum(values) / float(len(values)) if values else 0.0


def _pose_covariance_summary(msg: Odometry) -> dict[str, float]:
    covariance = msg.pose.covariance
    return {
        "x": float(covariance[0]),
        "y": float(covariance[7]),
        "yaw": float(covariance[35]),
    }


def infer_likely_origin(
    *, navsat_transform_drift_m: float, global_fusion_drift_m: float
) -> str:
    if global_fusion_drift_m > max(1.5, navsat_transform_drift_m * 2.0):
        return "fusion_with_odometry_local"
    if navsat_transform_drift_m > 1.0:
        return "navsat_transform_or_gps"
    return "mixed_or_inconclusive"


def classify_drift_level(
    *, map_odom_drift_m: float, warn_threshold_m: float, fail_threshold_m: float
) -> str:
    if fail_threshold_m < warn_threshold_m:
        raise ValueError("fail_threshold_m must be >= warn_threshold_m")
    if map_odom_drift_m >= fail_threshold_m:
        return "fail"
    if map_odom_drift_m >= warn_threshold_m:
        return "warn"
    return "ok"


def build_report(
    *,
    result: dict[str, Any],
    duration_s: float,
    warn_threshold_m: float,
    fail_threshold_m: float,
) -> dict[str, Any]:
    map_odom_drift_m = float(result["drift_m"]["map_odom"])
    level = classify_drift_level(
        map_odom_drift_m=map_odom_drift_m,
        warn_threshold_m=warn_threshold_m,
        fail_threshold_m=fail_threshold_m,
    )
    likely_origin = infer_likely_origin(
        navsat_transform_drift_m=float(
            result["drift_attribution"]["navsat_transform_drift_m"]
        ),
        global_fusion_drift_m=float(
            result["drift_attribution"]["global_fusion_drift_m"]
        ),
    )
    return {
        "ok": level != "fail",
        "level": level,
        "duration_s": float(duration_s),
        "thresholds_m": {
            "warn_map_odom": float(warn_threshold_m),
            "fail_map_odom": float(fail_threshold_m),
        },
        "summary": {
            "map_odom_drift_m": map_odom_drift_m,
            "map_base_drift_m": float(result["drift_m"]["map_base"]),
            "odom_base_drift_m": float(result["drift_m"]["odom_base"]),
            "odom_gps_drift_m": float(result["drift_m"]["odom_gps"]),
            "fromll_map_drift_m": float(result["drift_m"]["fromll_map"]),
            "fromll_odom_drift_m": float(result["drift_m"]["fromll_odom"]),
            "likely_origin": likely_origin,
        },
        "details": result,
    }


class GlobalDriftProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_global_drift_check")
        self.gps_fix: Optional[NavSatFix] = None
        self.odom_local: Optional[Odometry] = None
        self.odom_gps: Optional[Odometry] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps_fix, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_odom_local, 10)
        self.create_subscription(Odometry, "/odometry/gps", self._on_odom_gps, 10)
        self._fromll_clients = [
            ("/fromLL", self.create_client(FromLL, "/fromLL")),
            (
                "/navsat_transform/fromLL",
                self.create_client(FromLL, "/navsat_transform/fromLL"),
            ),
        ]
        self._active_fromll_client: Optional[FromLL.Client] = None
        self._active_fromll_service_name = ""

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        self.gps_fix = msg

    def _on_odom_local(self, msg: Odometry) -> None:
        self.odom_local = msg

    def _on_odom_gps(self, msg: Odometry) -> None:
        self.odom_gps = msg

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
            if self.gps_fix is None or self.odom_local is None or self.odom_gps is None:
                return False
            try:
                self.lookup_xy("map", "odom", timeout_s=0.2)
                self.lookup_xy("odom", "base_footprint", timeout_s=0.2)
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

    def fromll_to_map(self, lat: float, lon: float, timeout_s: float = 5.0) -> dict[str, Any]:
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
                return {
                    "raw_odom_xy": [
                        float(response.map_point.x),
                        float(response.map_point.y),
                    ],
                    "map_xy": [
                        float(transformed.point.x),
                        float(transformed.point.y),
                    ],
                }
        raise RuntimeError("timeout waiting for fromLL response")

    def sample_idle_drift(
        self, *, duration_s: float, sample_interval_s: float
    ) -> dict[str, Any]:
        samples: list[dict[str, Any]] = []
        target_samples = max(2, int(math.ceil(duration_s / sample_interval_s)))
        end = time.time() + duration_s + 5.0

        while time.time() < end and len(samples) < target_samples:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gps_fix is None or self.odom_local is None or self.odom_gps is None:
                continue
            try:
                gps_projection = self.fromll_to_map(
                    self.gps_fix.latitude,
                    self.gps_fix.longitude,
                )
                map_base = self.lookup_xy("map", "base_footprint", timeout_s=0.5)
                odom_base = self.lookup_xy("odom", "base_footprint", timeout_s=0.5)
                map_odom = self.lookup_xy("map", "odom", timeout_s=0.5)
            except Exception:
                time.sleep(0.2)
                continue

            samples.append(
                {
                    "gps_lat": float(self.gps_fix.latitude),
                    "gps_lon": float(self.gps_fix.longitude),
                    "fromll_odom_xy": gps_projection["raw_odom_xy"],
                    "fromll_map_xy": gps_projection["map_xy"],
                    "navsat_vs_fromll_odom_delta_m": _distance_xy(
                        tuple(gps_projection["raw_odom_xy"]),
                        (
                            float(self.odom_gps.pose.pose.position.x),
                            float(self.odom_gps.pose.pose.position.y),
                        ),
                    ),
                    "map_base_xy": [float(map_base[0]), float(map_base[1])],
                    "odom_base_xy": [float(odom_base[0]), float(odom_base[1])],
                    "map_odom_xy": [float(map_odom[0]), float(map_odom[1])],
                    "odom_local_xy": [
                        float(self.odom_local.pose.pose.position.x),
                        float(self.odom_local.pose.pose.position.y),
                    ],
                    "odom_gps_xy": [
                        float(self.odom_gps.pose.pose.position.x),
                        float(self.odom_gps.pose.pose.position.y),
                    ],
                    "odom_local_covariance": _pose_covariance_summary(self.odom_local),
                    "odom_gps_covariance": _pose_covariance_summary(self.odom_gps),
                }
            )
            time.sleep(sample_interval_s)

        if len(samples) < 2:
            raise RuntimeError("insufficient idle samples")

        first = samples[0]
        last = samples[-1]
        navsat_transform_drift = max(
            _distance_xy(tuple(first["fromll_odom_xy"]), tuple(last["fromll_odom_xy"])),
            _distance_xy(tuple(first["odom_gps_xy"]), tuple(last["odom_gps_xy"])),
        )
        global_fusion_drift = _distance_xy(
            tuple(first["map_odom_xy"]), tuple(last["map_odom_xy"])
        )
        fusion_added_drift = max(0.0, global_fusion_drift - navsat_transform_drift)

        return {
            "sample_count": len(samples),
            "fromll_service": self._active_fromll_service_name,
            "first_sample": first,
            "last_sample": last,
            "drift_m": {
                "map_odom": global_fusion_drift,
                "map_base": _distance_xy(
                    tuple(first["map_base_xy"]), tuple(last["map_base_xy"])
                ),
                "odom_base": _distance_xy(
                    tuple(first["odom_base_xy"]), tuple(last["odom_base_xy"])
                ),
                "fromll_map": _distance_xy(
                    tuple(first["fromll_map_xy"]), tuple(last["fromll_map_xy"])
                ),
                "fromll_odom": _distance_xy(
                    tuple(first["fromll_odom_xy"]), tuple(last["fromll_odom_xy"])
                ),
                "odom_gps": _distance_xy(
                    tuple(first["odom_gps_xy"]), tuple(last["odom_gps_xy"])
                ),
            },
            "odometry_gps_covariance": {
                "first": first["odom_gps_covariance"],
                "last": last["odom_gps_covariance"],
                "mean": {
                    axis: _mean([float(sample["odom_gps_covariance"][axis]) for sample in samples])
                    for axis in ("x", "y", "yaw")
                },
            },
            "odometry_local_covariance": {
                "first": first["odom_local_covariance"],
                "last": last["odom_local_covariance"],
                "mean": {
                    axis: _mean(
                        [float(sample["odom_local_covariance"][axis]) for sample in samples]
                    )
                    for axis in ("x", "y", "yaw")
                },
            },
            "navsat_consistency": {
                "delta_fromll_to_odom_gps_first_m": float(first["navsat_vs_fromll_odom_delta_m"]),
                "delta_fromll_to_odom_gps_last_m": float(last["navsat_vs_fromll_odom_delta_m"]),
                "delta_fromll_to_odom_gps_mean_m": _mean(
                    [float(sample["navsat_vs_fromll_odom_delta_m"]) for sample in samples]
                ),
            },
            "drift_attribution": {
                "navsat_transform_drift_m": navsat_transform_drift,
                "global_fusion_drift_m": global_fusion_drift,
                "fusion_added_drift_m": fusion_added_drift,
                "likely_origin": infer_likely_origin(
                    navsat_transform_drift_m=navsat_transform_drift,
                    global_fusion_drift_m=global_fusion_drift,
                ),
            },
        }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure idle drift for an already-running sim_global_v2 session.",
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
        "--warn-threshold-m",
        type=float,
        default=DEFAULT_WARN_THRESHOLD_M,
    )
    parser.add_argument(
        "--fail-threshold-m",
        type=float,
        default=DEFAULT_FAIL_THRESHOLD_M,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = GlobalDriftProbe()
    try:
        if not node.wait_for_bootstrap(args.bootstrap_timeout_s):
            raise RuntimeError("timed out waiting for sim_global_v2 topics, TF, and fromLL")
        result = node.sample_idle_drift(
            duration_s=args.duration_s,
            sample_interval_s=args.sample_interval_s,
        )
        report = build_report(
            result=result,
            duration_s=args.duration_s,
            warn_threshold_m=args.warn_threshold_m,
            fail_threshold_m=args.fail_threshold_m,
        )
        summary = report["summary"]
        print(
            "sim_global_v2 drift "
            f"[{report['level'].upper()}] "
            f"map->odom={summary['map_odom_drift_m']:.3f} m, "
            f"map->base={summary['map_base_drift_m']:.3f} m, "
            f"odom->base={summary['odom_base_drift_m']:.3f} m, "
            f"odom_gps={summary['odom_gps_drift_m']:.3f} m, "
            f"fromll_map={summary['fromll_map_drift_m']:.3f} m, "
            f"likely_origin={summary['likely_origin']}"
        )
        payload = json.dumps(report, indent=2, sort_keys=True)
        print(payload)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as handle:
                handle.write(payload)
                handle.write("\n")
        return 2 if report["level"] == "fail" else 0
    except Exception as exc:
        print(f"sim_global_v2 drift check failed: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
