from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        float(quaternion.w) * float(quaternion.z)
        + float(quaternion.x) * float(quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(quaternion.y) * float(quaternion.y)
        + float(quaternion.z) * float(quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    half_yaw = 0.5 * float(yaw_rad)
    quaternion = Quaternion()
    quaternion.w = math.cos(half_yaw)
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(half_yaw)
    return quaternion


def build_ackermann_path(
    start_pose: Pose,
    goal_pose: Pose,
    frame_id: str,
    step_distance_m: float,
    min_intermediate_poses: int,
    use_goal_orientation: bool = True,
) -> Path:
    path = Path()
    path.header.frame_id = str(frame_id)

    dx_m = float(goal_pose.position.x) - float(start_pose.position.x)
    dy_m = float(goal_pose.position.y) - float(start_pose.position.y)
    distance_m = math.hypot(dx_m, dy_m)
    heading_rad = (
        math.atan2(dy_m, dx_m)
        if distance_m > 1.0e-6
        else yaw_from_quaternion(goal_pose.orientation)
    )
    start_yaw_rad = yaw_from_quaternion(start_pose.orientation)
    goal_yaw_rad = (
        yaw_from_quaternion(goal_pose.orientation)
        if use_goal_orientation
        else heading_rad
    )

    if distance_m <= 1.0e-6:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = str(frame_id)
        pose_stamped.pose = goal_pose
        path.poses.append(pose_stamped)
        return path

    safe_step = max(0.05, float(step_distance_m))
    segment_count = max(
        int(math.ceil(distance_m / safe_step)),
        int(min_intermediate_poses),
    )
    control_distance_m = max(0.35, min(distance_m * 0.5, 2.0))

    p0_x = float(start_pose.position.x)
    p0_y = float(start_pose.position.y)
    p1_x = p0_x + control_distance_m * math.cos(start_yaw_rad)
    p1_y = p0_y + control_distance_m * math.sin(start_yaw_rad)
    p3_x = float(goal_pose.position.x)
    p3_y = float(goal_pose.position.y)
    p2_x = p3_x - control_distance_m * math.cos(goal_yaw_rad)
    p2_y = p3_y - control_distance_m * math.sin(goal_yaw_rad)

    for index in range(segment_count + 1):
        ratio = float(index) / float(segment_count)
        one_minus_ratio = 1.0 - ratio
        x_value = (
            (one_minus_ratio**3) * p0_x
            + 3.0 * (one_minus_ratio**2) * ratio * p1_x
            + 3.0 * one_minus_ratio * (ratio**2) * p2_x
            + (ratio**3) * p3_x
        )
        y_value = (
            (one_minus_ratio**3) * p0_y
            + 3.0 * (one_minus_ratio**2) * ratio * p1_y
            + 3.0 * one_minus_ratio * (ratio**2) * p2_y
            + (ratio**3) * p3_y
        )
        tangent_x = (
            3.0 * (one_minus_ratio**2) * (p1_x - p0_x)
            + 6.0 * one_minus_ratio * ratio * (p2_x - p1_x)
            + 3.0 * (ratio**2) * (p3_x - p2_x)
        )
        tangent_y = (
            3.0 * (one_minus_ratio**2) * (p1_y - p0_y)
            + 6.0 * one_minus_ratio * ratio * (p2_y - p1_y)
            + 3.0 * (ratio**2) * (p3_y - p2_y)
        )
        tangent_norm = math.hypot(tangent_x, tangent_y)
        tangent_yaw_rad = (
            math.atan2(tangent_y, tangent_x)
            if tangent_norm > 1.0e-6
            else heading_rad
        )
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = str(frame_id)
        pose_stamped.pose.position.x = x_value
        pose_stamped.pose.position.y = y_value
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = quaternion_from_yaw(tangent_yaw_rad)
        path.poses.append(pose_stamped)

    if use_goal_orientation and path.poses:
        path.poses[-1].pose.orientation = goal_pose.orientation

    return path


class GoalPoseToFollowPathV2(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_to_follow_path_v2")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("path_topic", "/goal_pose_path")
        self.declare_parameter("step_distance_m", 0.10)
        self.declare_parameter("min_intermediate_poses", 6)
        self.declare_parameter("transform_timeout_s", 0.2)
        self.declare_parameter("use_goal_orientation", True)
        self.declare_parameter("stop_hold_topic", "/local_nav_v2/stop_hold")

        goal_topic = str(self.get_parameter("goal_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        action_name = str(self.get_parameter("action_name").value)
        path_topic = str(self.get_parameter("path_topic").value)
        stop_hold_topic = str(self.get_parameter("stop_hold_topic").value)

        self._path_frame = str(self.get_parameter("path_frame").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._step_distance_m = float(self.get_parameter("step_distance_m").value)
        self._min_intermediate_poses = int(
            self.get_parameter("min_intermediate_poses").value
        )
        self._transform_timeout_s = float(
            self.get_parameter("transform_timeout_s").value
        )
        self._use_goal_orientation = bool(
            self.get_parameter("use_goal_orientation").value
        )

        self._latest_pose: Optional[PoseStamped] = None
        self._latest_goal_handle = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._follow_path_client = ActionClient(self, FollowPath, action_name)

        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._stop_hold_pub = self.create_publisher(Empty, stop_hold_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal_pose, 10)

        self.get_logger().info(
            "goal_pose_to_follow_path_v2 ready "
            f"({goal_topic} -> {action_name}, frame={self._path_frame})"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self._latest_pose = pose_stamped

    def _transform_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        if pose.header.frame_id == self._path_frame:
            return pose

        try:
            transform = self._tf_buffer.lookup_transform(
                self._path_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._transform_timeout_s),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"No pude transformar {pose.header.frame_id} -> {self._path_frame}: {exc}"
            )
            return None

        return do_transform_pose_stamped(pose, transform)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self._latest_pose is None:
            self.get_logger().warning(
                "Ignorando /goal_pose: todavia no hay /odometry/local disponible"
            )
            return

        current_pose = self._transform_pose(self._latest_pose)
        goal_pose = self._transform_pose(msg)
        if current_pose is None or goal_pose is None:
            return

        path = build_ackermann_path(
            start_pose=current_pose.pose,
            goal_pose=goal_pose.pose,
            frame_id=self._path_frame,
            step_distance_m=self._step_distance_m,
            min_intermediate_poses=self._min_intermediate_poses,
            use_goal_orientation=self._use_goal_orientation,
        )
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
        self._path_pub.publish(path)

        if not self._follow_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("El action server /follow_path no esta disponible")
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self.get_logger().info(
            "Enviando goal desde RViz a FollowPath "
            f"(poses={len(path.poses)}, x={goal_pose.pose.position.x:.2f}, "
            f"y={goal_pose.pose.position.y:.2f})"
        )

        send_goal_future = self._follow_path_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("FollowPath rechazo el goal generado desde RViz")
            return

        self._latest_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"Error esperando resultado de FollowPath: {exc}")
            return
        self._stop_hold_pub.publish(Empty())
        self.get_logger().info(f"Resultado FollowPath desde RViz: status={result.status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseToFollowPathV2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
