from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2


DEFAULT_IMU_ORIENTATION_VARIANCE = 0.01
DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE = 0.01
DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE = 0.1
DEFAULT_GPS_HORIZONTAL_VARIANCE = 2.5
DEFAULT_GPS_VERTICAL_VARIANCE = 4.0


def _covariance_is_zero(values) -> bool:
    return all(abs(float(value)) <= 1.0e-12 for value in values)


def _normalize_angle(angle_rad: float) -> float:
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    return angle_rad


def _normalize_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[Tuple[float, float, float, float]]:
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and math.isfinite(w)):
        return None
    norm = math.sqrt(
        float(x) * float(x)
        + float(y) * float(y)
        + float(z) * float(z)
        + float(w) * float(w)
    )
    if norm <= 1.0e-12:
        return None
    return (
        float(x) / norm,
        float(y) / norm,
        float(z) / norm,
        float(w) / norm,
    )


def _yaw_from_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[float]:
    normalized = _normalize_quaternion_xyzw(x, y, z, w)
    if normalized is None:
        return None
    qx, qy, qz, qw = normalized
    siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
    cosy_cosp = 1.0 - 2.0 * ((qy * qy) + (qz * qz))
    return _normalize_angle(math.atan2(siny_cosp, cosy_cosp))


def _quaternion_from_yaw_xyzw(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(yaw_rad)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _quaternion_multiply_xyzw(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
    )


def _apply_yaw_offset_to_quaternion(
    quat: Quaternion,
    yaw_offset_rad: float,
) -> Optional[Tuple[float, float, float, float]]:
    normalized_input = _normalize_quaternion_xyzw(quat.x, quat.y, quat.z, quat.w)
    if normalized_input is None:
        return None
    yaw_quat = _quaternion_from_yaw_xyzw(yaw_offset_rad)
    corrected = _quaternion_multiply_xyzw(yaw_quat, normalized_input)
    return _normalize_quaternion_xyzw(*corrected)


def _planar_speed_from_odometry(msg: Odometry) -> Optional[float]:
    vx = float(msg.twist.twist.linear.x)
    vy = float(msg.twist.twist.linear.y)
    if not (math.isfinite(vx) and math.isfinite(vy)):
        return None
    return math.sqrt((vx * vx) + (vy * vy))


def _update_auto_yaw_calibration(
    *,
    auto_enabled: bool,
    already_calibrated: bool,
    now_s: float,
    calibration_started_s: float,
    calibration_timeout_s: float,
    imu_yaw_rad: Optional[float],
    odom_yaw_rad: Optional[float],
    odom_speed_mps: Optional[float],
    speed_threshold_mps: float,
    current_auto_offset_rad: float,
) -> Tuple[bool, float, bool]:
    if not auto_enabled:
        return True, float(current_auto_offset_rad), False
    if already_calibrated:
        return True, float(current_auto_offset_rad), False
    if (float(now_s) - float(calibration_started_s)) >= float(calibration_timeout_s):
        return True, float(current_auto_offset_rad), True
    if imu_yaw_rad is None or odom_yaw_rad is None or odom_speed_mps is None:
        return False, float(current_auto_offset_rad), False
    if abs(float(odom_speed_mps)) > float(speed_threshold_mps):
        return False, float(current_auto_offset_rad), False
    return True, _normalize_angle(float(odom_yaw_rad) - float(imu_yaw_rad)), False


class SimSensorNormalizerV2Node(Node):
    def __init__(self) -> None:
        super().__init__("sim_sensor_normalizer_v2")

        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("imu_out_topic", "/imu/data")
        self.declare_parameter("gps_in_topic", "/gps/fix_raw")
        self.declare_parameter("gps_out_topic", "/gps/fix")
        self.declare_parameter(
            "gps_horizontal_variance", DEFAULT_GPS_HORIZONTAL_VARIANCE
        )
        self.declare_parameter("gps_vertical_variance", DEFAULT_GPS_VERTICAL_VARIANCE)
        self.declare_parameter("lidar_in_topic", "/scan_3d_raw")
        self.declare_parameter("lidar_out_topic", "/scan_3d")
        self.declare_parameter("odom_in_topic", "/odom_raw")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("lidar_frame_id", "lidar_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_link_frame_id", "base_footprint")
        self.declare_parameter("imu_auto_calibrate_yaw_from_odom", True)
        self.declare_parameter("imu_yaw_offset_rad", 0.0)
        self.declare_parameter("imu_yaw_calib_odom_topic", "/odom_raw")
        self.declare_parameter("imu_yaw_calib_speed_threshold_mps", 0.05)
        self.declare_parameter("imu_yaw_calib_timeout_s", 3.0)
        # Optional second GPS antenna (dual-antenna heading simulation).
        # Leave gps2_in_topic empty (default) to disable.
        self.declare_parameter("gps2_in_topic", "")
        self.declare_parameter("gps2_out_topic", "/gps2/fix")
        self.declare_parameter("gps2_frame_id", "gps_link2")

        imu_in_topic = str(self.get_parameter("imu_in_topic").value)
        imu_out_topic = str(self.get_parameter("imu_out_topic").value)
        gps_in_topic = str(self.get_parameter("gps_in_topic").value)
        gps_out_topic = str(self.get_parameter("gps_out_topic").value)
        self._gps_horizontal_variance = max(
            1.0e-6, float(self.get_parameter("gps_horizontal_variance").value)
        )
        self._gps_vertical_variance = max(
            1.0e-6, float(self.get_parameter("gps_vertical_variance").value)
        )
        lidar_in_topic = str(self.get_parameter("lidar_in_topic").value)
        lidar_out_topic = str(self.get_parameter("lidar_out_topic").value)
        odom_in_topic = str(self.get_parameter("odom_in_topic").value)
        odom_out_topic = str(self.get_parameter("odom_out_topic").value)
        self._imu_auto_calibrate_yaw_from_odom = bool(
            self.get_parameter("imu_auto_calibrate_yaw_from_odom").value
        )
        self._imu_yaw_offset_rad = float(self.get_parameter("imu_yaw_offset_rad").value)
        self._imu_yaw_calib_odom_topic = str(
            self.get_parameter("imu_yaw_calib_odom_topic").value
        )
        self._imu_yaw_calib_speed_threshold_mps = max(
            0.0, float(self.get_parameter("imu_yaw_calib_speed_threshold_mps").value)
        )
        self._imu_yaw_calib_timeout_s = max(
            0.0, float(self.get_parameter("imu_yaw_calib_timeout_s").value)
        )

        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self._gps_frame_id = str(self.get_parameter("gps_frame_id").value)
        self._lidar_frame_id = str(self.get_parameter("lidar_frame_id").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_link_frame_id = str(self.get_parameter("base_link_frame_id").value)
        self._gps2_frame_id = str(self.get_parameter("gps2_frame_id").value)

        self._imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self._gps_pub = self.create_publisher(NavSatFix, gps_out_topic, 10)
        self._lidar_pub = self.create_publisher(PointCloud2, lidar_out_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)

        self.create_subscription(Imu, imu_in_topic, self._on_imu, 10)
        self.create_subscription(NavSatFix, gps_in_topic, self._on_gps, 10)
        self.create_subscription(PointCloud2, lidar_in_topic, self._on_lidar, 10)
        self.create_subscription(Odometry, odom_in_topic, self._on_odom, 10)

        gps2_in_topic = str(self.get_parameter("gps2_in_topic").value)
        if gps2_in_topic:
            gps2_out_topic = str(self.get_parameter("gps2_out_topic").value)
            self._gps2_pub = self.create_publisher(NavSatFix, gps2_out_topic, 10)
            self.create_subscription(NavSatFix, gps2_in_topic, self._on_gps2, 10)
            self.get_logger().info(f"Second GPS antenna: {gps2_in_topic} → {gps2_out_topic}")
        else:
            self._gps2_pub = None
        self._odom_in_topic = odom_in_topic
        self._use_main_odom_for_calibration = (
            self._imu_yaw_calib_odom_topic == self._odom_in_topic
        )
        if not self._use_main_odom_for_calibration:
            self.create_subscription(
                Odometry,
                self._imu_yaw_calib_odom_topic,
                self._on_calib_odom,
                10,
            )

        self._latest_odom_yaw_rad: Optional[float] = None
        self._latest_odom_speed_mps: Optional[float] = None
        self._yaw_auto_calibrated = not self._imu_auto_calibrate_yaw_from_odom
        self._yaw_auto_offset_rad = 0.0
        self._yaw_calibration_started_s = self._clock_now_s()
        self._yaw_calibration_timeout_logged = False

        self.get_logger().info(
            "sim_sensor_normalizer_v2 ready "
            f"({imu_in_topic},{gps_in_topic},{lidar_in_topic},{odom_in_topic}, "
            f"auto_calib={self._imu_auto_calibrate_yaw_from_odom}, "
            f"calib_odom={self._imu_yaw_calib_odom_topic}, "
            f"gps_var_xy={self._gps_horizontal_variance:.3f}, "
            f"gps_var_z={self._gps_vertical_variance:.3f})"
        )

    def _clock_now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _capture_calibration_odom(self, msg: Odometry) -> None:
        yaw = _yaw_from_quaternion_xyzw(
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        speed = _planar_speed_from_odometry(msg)
        if yaw is None or speed is None:
            return
        self._latest_odom_yaw_rad = yaw
        self._latest_odom_speed_mps = speed

    def _on_calib_odom(self, msg: Odometry) -> None:
        self._capture_calibration_odom(msg)

    def _maybe_update_auto_yaw_calibration(self, msg: Imu) -> None:
        imu_yaw = _yaw_from_quaternion_xyzw(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        calibrated_before = bool(self._yaw_auto_calibrated)
        timed_out_before = bool(self._yaw_calibration_timeout_logged)
        self._yaw_auto_calibrated, self._yaw_auto_offset_rad, timed_out = (
            _update_auto_yaw_calibration(
                auto_enabled=self._imu_auto_calibrate_yaw_from_odom,
                already_calibrated=self._yaw_auto_calibrated,
                now_s=self._clock_now_s(),
                calibration_started_s=self._yaw_calibration_started_s,
                calibration_timeout_s=self._imu_yaw_calib_timeout_s,
                imu_yaw_rad=imu_yaw,
                odom_yaw_rad=self._latest_odom_yaw_rad,
                odom_speed_mps=self._latest_odom_speed_mps,
                speed_threshold_mps=self._imu_yaw_calib_speed_threshold_mps,
                current_auto_offset_rad=self._yaw_auto_offset_rad,
            )
        )
        if timed_out and not timed_out_before:
            self._yaw_calibration_timeout_logged = True
            self.get_logger().warn(
                "IMU yaw auto-calibration timed out; using manual offset only."
            )
        if (not calibrated_before) and self._yaw_auto_calibrated and (not timed_out):
            self.get_logger().info(
                f"IMU yaw auto-calibration completed (offset={self._yaw_auto_offset_rad:.6f} rad)."
            )

    def _on_imu(self, msg: Imu) -> None:
        out = msg
        out.header.frame_id = self._imu_frame_id
        self._maybe_update_auto_yaw_calibration(out)
        total_yaw_offset = _normalize_angle(
            float(self._imu_yaw_offset_rad) + float(self._yaw_auto_offset_rad)
        )
        corrected_quat = _apply_yaw_offset_to_quaternion(out.orientation, total_yaw_offset)
        if corrected_quat is not None:
            out.orientation.x = corrected_quat[0]
            out.orientation.y = corrected_quat[1]
            out.orientation.z = corrected_quat[2]
            out.orientation.w = corrected_quat[3]
        if _covariance_is_zero(out.orientation_covariance):
            out.orientation_covariance = [
                DEFAULT_IMU_ORIENTATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ORIENTATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ORIENTATION_VARIANCE,
            ]
        if _covariance_is_zero(out.angular_velocity_covariance):
            out.angular_velocity_covariance = [
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
            ]
        if _covariance_is_zero(out.linear_acceleration_covariance):
            out.linear_acceleration_covariance = [
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
            ]
        self._imu_pub.publish(out)

    def _on_gps(self, msg: NavSatFix) -> None:
        out = msg
        out.header.frame_id = self._gps_frame_id
        if _covariance_is_zero(out.position_covariance):
            out.position_covariance = [
                self._gps_horizontal_variance,
                0.0,
                0.0,
                0.0,
                self._gps_horizontal_variance,
                0.0,
                0.0,
                0.0,
                self._gps_vertical_variance,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(out)

    def _on_gps2(self, msg: NavSatFix) -> None:
        out = msg
        out.header.frame_id = self._gps2_frame_id
        if _covariance_is_zero(out.position_covariance):
            out.position_covariance = [
                self._gps_horizontal_variance,
                0.0,
                0.0,
                0.0,
                self._gps_horizontal_variance,
                0.0,
                0.0,
                0.0,
                self._gps_vertical_variance,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        if self._gps2_pub is not None:
            self._gps2_pub.publish(out)

    def _on_lidar(self, msg: PointCloud2) -> None:
        out = msg
        out.header.frame_id = self._lidar_frame_id
        self._lidar_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        if self._use_main_odom_for_calibration:
            self._capture_calibration_odom(msg)
        out = msg
        out.header.frame_id = self._odom_frame_id
        out.child_frame_id = self._base_link_frame_id
        self._odom_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSensorNormalizerV2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
