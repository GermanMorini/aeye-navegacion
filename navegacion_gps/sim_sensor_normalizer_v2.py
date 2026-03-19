from __future__ import annotations

from copy import deepcopy

import rclpy
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


class SimSensorNormalizerV2Node(Node):
    def __init__(self) -> None:
        super().__init__("sim_sensor_normalizer_v2")

        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("imu_out_topic", "/imu/data")
        self.declare_parameter("gps_in_topic", "/gps/fix_raw")
        self.declare_parameter("gps_out_topic", "/gps/fix")
        self.declare_parameter("lidar_in_topic", "/scan_3d_raw")
        self.declare_parameter("lidar_out_topic", "/scan_3d")
        self.declare_parameter("odom_in_topic", "/odom_raw")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("lidar_frame_id", "lidar_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_link_frame_id", "base_footprint")

        imu_in_topic = str(self.get_parameter("imu_in_topic").value)
        imu_out_topic = str(self.get_parameter("imu_out_topic").value)
        gps_in_topic = str(self.get_parameter("gps_in_topic").value)
        gps_out_topic = str(self.get_parameter("gps_out_topic").value)
        lidar_in_topic = str(self.get_parameter("lidar_in_topic").value)
        lidar_out_topic = str(self.get_parameter("lidar_out_topic").value)
        odom_in_topic = str(self.get_parameter("odom_in_topic").value)
        odom_out_topic = str(self.get_parameter("odom_out_topic").value)

        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self._gps_frame_id = str(self.get_parameter("gps_frame_id").value)
        self._lidar_frame_id = str(self.get_parameter("lidar_frame_id").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_link_frame_id = str(self.get_parameter("base_link_frame_id").value)

        self._imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self._gps_pub = self.create_publisher(NavSatFix, gps_out_topic, 10)
        self._lidar_pub = self.create_publisher(PointCloud2, lidar_out_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)

        self.create_subscription(Imu, imu_in_topic, self._on_imu, 10)
        self.create_subscription(NavSatFix, gps_in_topic, self._on_gps, 10)
        self.create_subscription(PointCloud2, lidar_in_topic, self._on_lidar, 10)
        self.create_subscription(Odometry, odom_in_topic, self._on_odom, 10)

        self.get_logger().info(
            "sim_sensor_normalizer_v2 ready "
            f"({imu_in_topic},{gps_in_topic},{lidar_in_topic},{odom_in_topic})"
        )

    def _on_imu(self, msg: Imu) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._imu_frame_id
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
        out = deepcopy(msg)
        out.header.frame_id = self._gps_frame_id
        if _covariance_is_zero(out.position_covariance):
            out.position_covariance = [
                DEFAULT_GPS_HORIZONTAL_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_GPS_HORIZONTAL_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_GPS_VERTICAL_VARIANCE,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(out)

    def _on_lidar(self, msg: PointCloud2) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._lidar_frame_id
        self._lidar_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        out = deepcopy(msg)
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
