from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from rclpy.executors import ExternalShutdownException

from .tracking_path_geometry import compute_tracking_errors, normalize_angle


def _yaw_from_quaternion(quat: Quaternion) -> float:
    siny_cosp = 2.0 * ((quat.w * quat.z) + (quat.x * quat.y))
    cosy_cosp = 1.0 - 2.0 * ((quat.y * quat.y) + (quat.z * quat.z))
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


class TrackingPathDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking_path_debug")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("cross_track_topic", "/tracking_debug/cross_track_error")
        self.declare_parameter("heading_error_topic", "/tracking_debug/heading_error")
        self.declare_parameter("publish_hz", 20.0)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self._path_topic = str(self.get_parameter("path_topic").value)
        self._cross_track_topic = str(self.get_parameter("cross_track_topic").value)
        self._heading_error_topic = str(self.get_parameter("heading_error_topic").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self._latest_odom: Optional[Odometry] = None
        self._latest_path: Optional[Path] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cross_track_pub = self.create_publisher(Float64, self._cross_track_topic, 10)
        self._heading_error_pub = self.create_publisher(Float64, self._heading_error_topic, 10)

        self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(Path, self._path_topic, self._on_path, 10)
        self.create_timer(1.0 / publish_hz, self._publish_errors)

        self.get_logger().info(
            "tracking_path_debug ready "
            f"(odom={self._odom_topic}, base={self._robot_base_frame}, path={self._path_topic}, "
            f"cross_track={self._cross_track_topic}, heading_error={self._heading_error_topic})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_path(self, msg: Path) -> None:
        self._latest_path = msg

    def _lookup_robot_pose_in_path_frame(self, path_frame: str) -> Optional[Tuple[float, float, float]]:
        if self._latest_odom is None:
            return None

        odom = self._latest_odom
        msg_time = Time.from_msg(odom.header.stamp)
        if path_frame:
            transform = None
            try:
                transform = self._tf_buffer.lookup_transform(
                    path_frame, self._robot_base_frame, msg_time
                )
            except Exception:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        path_frame, self._robot_base_frame, Time()
                    )
                except Exception:
                    transform = None

            if transform is not None:
                pose_translation = transform.transform.translation
                pose_rotation = transform.transform.rotation
                return (
                    float(pose_translation.x),
                    float(pose_translation.y),
                    _yaw_from_quaternion(pose_rotation),
                )

        odom_frame = odom.header.frame_id or "odom"
        robot_x = float(odom.pose.pose.position.x)
        robot_y = float(odom.pose.pose.position.y)
        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)

        if not path_frame or path_frame == odom_frame:
            return robot_x, robot_y, robot_yaw

        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(path_frame, odom_frame, msg_time)
        except Exception:
            try:
                transform = self._tf_buffer.lookup_transform(path_frame, odom_frame, Time())
            except Exception:
                return None

        tf_yaw = _yaw_from_quaternion(transform.transform.rotation)
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)
        transformed_x = tx + (cos_yaw * robot_x) - (sin_yaw * robot_y)
        transformed_y = ty + (sin_yaw * robot_x) + (cos_yaw * robot_y)
        transformed_yaw = normalize_angle(tf_yaw + robot_yaw)
        return transformed_x, transformed_y, transformed_yaw

    def _publish_errors(self) -> None:
        if self._latest_path is None or len(self._latest_path.poses) < 2:
            return

        path_frame = self._latest_path.header.frame_id or ""
        robot_pose = self._lookup_robot_pose_in_path_frame(path_frame)
        if robot_pose is None:
            return

        path_points: List[Tuple[float, float]] = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in self._latest_path.poses
        ]
        errors = compute_tracking_errors(
            robot_x=robot_pose[0],
            robot_y=robot_pose[1],
            robot_yaw_rad=robot_pose[2],
            path_points=path_points,
        )
        if errors is None:
            return

        cross_track_error, heading_error, _segment_index = errors
        self._cross_track_pub.publish(Float64(data=float(cross_track_error)))
        self._heading_error_pub.publish(Float64(data=float(heading_error)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingPathDebugNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
