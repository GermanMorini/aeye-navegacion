from __future__ import annotations

from dataclasses import replace
import json
import math
from typing import Optional

import rclpy
from interfaces.msg import DriveTelemetry
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from navegacion_gps.gps_course_heading_core import CourseHeadingEstimate
from navegacion_gps.gps_course_heading_core import GpsCourseHeadingEstimator
from navegacion_gps.gps_course_heading_core import compensate_heading_for_sensor_offset


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _quaternion_from_yaw_deg(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw_rad = math.radians(float(yaw_deg))
    half_yaw = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class GpsCourseHeadingNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_course_heading")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("drive_telemetry_topic", "/controller/drive_telemetry")
        self.declare_parameter("output_topic", "/gps/course_heading")
        self.declare_parameter("debug_topic", "/gps/course_heading/debug")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("gps_frame", "gps_link")
        self.declare_parameter("transform_timeout_s", 0.2)
        self.declare_parameter("min_distance_m", 2.5)
        self.declare_parameter("min_speed_mps", 0.8)
        self.declare_parameter("max_abs_steer_deg", 6.0)
        self.declare_parameter("max_abs_yaw_rate_rps", 0.12)
        self.declare_parameter("max_fix_age_s", 0.5)
        self.declare_parameter("sample_dt_min_s", 0.05)
        self.declare_parameter("sample_dt_max_s", 4.0)
        self.declare_parameter("max_pair_distance_base_m", 0.10)
        self.declare_parameter("max_pair_distance_speed_gain", 1.5)
        self.declare_parameter("max_pair_speed_error_mps", 0.75)
        self.declare_parameter("heading_change_base_deg", 3.0)
        self.declare_parameter("heading_change_yaw_rate_gain", 1.0)
        self.declare_parameter("candidates", 5)
        self.declare_parameter("max_heading_dispersion_deg", 4.0)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("yaw_variance_rad2", 0.20)

        gps_topic = str(self.get_parameter("gps_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        drive_telemetry_topic = str(self.get_parameter("drive_telemetry_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._gps_frame = str(self.get_parameter("gps_frame").value)
        self._transform_timeout_s = max(
            0.0, float(self.get_parameter("transform_timeout_s").value)
        )
        self._publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._yaw_variance_rad2 = max(
            1.0e-6, float(self.get_parameter("yaw_variance_rad2").value)
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._estimator = GpsCourseHeadingEstimator(
            min_distance_m=float(self.get_parameter("min_distance_m").value),
            min_speed_mps=float(self.get_parameter("min_speed_mps").value),
            max_abs_steer_deg=float(self.get_parameter("max_abs_steer_deg").value),
            max_abs_yaw_rate_rps=float(
                self.get_parameter("max_abs_yaw_rate_rps").value
            ),
            max_fix_age_s=float(self.get_parameter("max_fix_age_s").value),
            sample_dt_min_s=float(self.get_parameter("sample_dt_min_s").value),
            sample_dt_max_s=float(self.get_parameter("sample_dt_max_s").value),
            max_pair_distance_base_m=float(
                self.get_parameter("max_pair_distance_base_m").value
            ),
            max_pair_distance_speed_gain=float(
                self.get_parameter("max_pair_distance_speed_gain").value
            ),
            max_pair_speed_error_mps=float(
                self.get_parameter("max_pair_speed_error_mps").value
            ),
            heading_change_base_deg=float(
                self.get_parameter("heading_change_base_deg").value
            ),
            heading_change_yaw_rate_gain=float(
                self.get_parameter("heading_change_yaw_rate_gain").value
            ),
            candidates=int(self.get_parameter("candidates").value),
            max_heading_dispersion_deg=float(
                self.get_parameter("max_heading_dispersion_deg").value
            ),
        )

        self._last_fix_stamp_s: Optional[float] = None
        self._last_local_speed_mps: float = 0.0
        self._last_local_yaw_rate_rps: float = 0.0
        self._last_steer_deg: Optional[float] = None
        self._last_steer_valid = False
        self._last_drive_fresh = False
        self._last_drive_speed_mps = 0.0
        self._gps_offset_x_m: Optional[float] = None
        self._gps_offset_y_m: Optional[float] = None
        self._last_transform_error: Optional[str] = None

        self._imu_pub = self.create_publisher(Imu, output_topic, 10)
        self._debug_pub = self.create_publisher(String, debug_topic, 10)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)
        self.create_subscription(
            DriveTelemetry,
            drive_telemetry_topic,
            self._on_drive_telemetry,
            10,
        )
        self.create_timer(1.0 / self._publish_hz, self._on_publish_timer)
        self.get_logger().info(
            "gps_course_heading ready "
            f"(gps={gps_topic}, odom={odom_topic}, drive={drive_telemetry_topic}, "
            f"output={output_topic}, debug={debug_topic})"
        )

    def _refresh_gps_offset(self) -> bool:
        if self._gps_offset_x_m is not None and self._gps_offset_y_m is not None:
            return True
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._gps_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=self._transform_timeout_s),
            )
        except TransformException as exc:
            self._last_transform_error = str(exc)
            return False
        self._gps_offset_x_m = float(transform.transform.translation.x)
        self._gps_offset_y_m = float(transform.transform.translation.y)
        self._last_transform_error = None
        self.get_logger().info(
            "gps_course_heading using GPS offset "
            f"{self._gps_frame}->{self._base_frame}: "
            f"x={self._gps_offset_x_m:.3f} m, y={self._gps_offset_y_m:.3f} m"
        )
        return True

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not math.isfinite(float(msg.latitude)) or not math.isfinite(float(msg.longitude)):
            return
        stamp_s = _stamp_to_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds / 1.0e9
        self._last_fix_stamp_s = float(stamp_s)
        self._estimator.add_fix(
            lat=float(msg.latitude),
            lon=float(msg.longitude),
            stamp_s=float(stamp_s),
        )

    def _on_odometry(self, msg: Odometry) -> None:
        self._last_local_speed_mps = float(msg.twist.twist.linear.x)
        self._last_local_yaw_rate_rps = float(msg.twist.twist.angular.z)

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        self._last_steer_valid = bool(msg.steer_valid)
        self._last_drive_fresh = bool(msg.fresh)
        self._last_drive_speed_mps = float(msg.speed_mps_measured)
        if self._last_steer_valid and math.isfinite(float(msg.steer_deg_measured)):
            self._last_steer_deg = float(msg.steer_deg_measured)
        else:
            self._last_steer_deg = None

    def _on_publish_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1.0e9
        speed_mps = float(self._last_local_speed_mps)
        if abs(speed_mps) < 1.0e-6:
            speed_mps = float(self._last_drive_speed_mps)
        estimate = self._estimator.estimate(
            now_s=float(now_s),
            speed_mps=float(speed_mps),
            steer_deg=self._last_steer_deg,
            steer_valid=bool(self._last_steer_valid and self._last_drive_fresh),
            yaw_rate_rps=float(self._last_local_yaw_rate_rps),
        )
        raw_gps_course_yaw_deg = estimate.yaw_deg
        corrected_base_yaw_deg: Optional[float] = None
        heading_correction_deg: Optional[float] = None
        offset_compensated = False

        if estimate.valid and estimate.yaw_deg is not None:
            if self._refresh_gps_offset():
                compensation = compensate_heading_for_sensor_offset(
                    antenna_yaw_deg=float(estimate.yaw_deg),
                    speed_mps=float(speed_mps),
                    yaw_rate_rps=float(self._last_local_yaw_rate_rps),
                    offset_x_m=float(self._gps_offset_x_m),
                    offset_y_m=float(self._gps_offset_y_m),
                )
                corrected_base_yaw_deg = float(compensation.yaw_deg)
                heading_correction_deg = float(compensation.correction_deg)
                estimate = replace(estimate, yaw_deg=corrected_base_yaw_deg)
                offset_compensated = True
            else:
                estimate = replace(
                    estimate,
                    valid=False,
                    reason="gps_offset_tf_unavailable",
                    yaw_deg=None,
                )

        self._publish_debug(
            estimate=estimate,
            raw_gps_course_yaw_deg=raw_gps_course_yaw_deg,
            corrected_base_yaw_deg=corrected_base_yaw_deg,
            heading_correction_deg=heading_correction_deg,
            offset_compensated=offset_compensated,
        )
        if estimate.valid and estimate.yaw_deg is not None:
            self._publish_imu(estimate)

    def _publish_imu(self, estimate: CourseHeadingEstimate) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        qx, qy, qz, qw = _quaternion_from_yaw_deg(float(estimate.yaw_deg))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [
            1.0e6,
            0.0,
            0.0,
            0.0,
            1.0e6,
            0.0,
            0.0,
            0.0,
            float(self._yaw_variance_rad2),
        ]
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.linear_acceleration_covariance = [
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self._imu_pub.publish(msg)

    def _publish_debug(
        self,
        *,
        estimate: CourseHeadingEstimate,
        raw_gps_course_yaw_deg: Optional[float],
        corrected_base_yaw_deg: Optional[float],
        heading_correction_deg: Optional[float],
        offset_compensated: bool,
    ) -> None:
        payload = {
            "valid": bool(estimate.valid),
            "reason": estimate.reason,
            "yaw_deg": estimate.yaw_deg,
            "distance_m": estimate.distance_m,
            "speed_mps": estimate.speed_mps,
            "steer_deg": estimate.steer_deg,
            "yaw_rate_rps": estimate.yaw_rate_rps,
            "latest_fix_age_s": estimate.latest_fix_age_s,
            "sample_dt_s": estimate.sample_dt_s,
            "candidate_count": estimate.candidate_count,
            "heading_dispersion_deg": estimate.heading_dispersion_deg,
            "mean_yaw_deg": estimate.mean_yaw_deg,
            "base_frame": self._base_frame,
            "gps_frame": self._gps_frame,
            "offset_compensated": bool(offset_compensated),
            "gps_offset_x_m": self._gps_offset_x_m,
            "gps_offset_y_m": self._gps_offset_y_m,
            "heading_correction_deg": heading_correction_deg,
            "raw_gps_course_yaw_deg": raw_gps_course_yaw_deg,
            "corrected_base_yaw_deg": corrected_base_yaw_deg,
            "transform_error": self._last_transform_error,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._debug_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsCourseHeadingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
