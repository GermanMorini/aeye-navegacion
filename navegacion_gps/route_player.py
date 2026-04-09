"""Replay a recorded route using Nav2 Navigate Through Poses."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node

from navegacion_gps.route_tools import (
    RouteFileError,
    RoutePoint,
    load_route_yaml,
    quaternion_xyzw_from_yaw,
)

STATUS_LABELS = {
    int(GoalStatus.STATUS_UNKNOWN): "UNKNOWN",
    int(GoalStatus.STATUS_ACCEPTED): "ACCEPTED",
    int(GoalStatus.STATUS_EXECUTING): "EXECUTING",
    int(GoalStatus.STATUS_CANCELING): "CANCELING",
    int(GoalStatus.STATUS_SUCCEEDED): "SUCCEEDED",
    int(GoalStatus.STATUS_CANCELED): "CANCELED",
    int(GoalStatus.STATUS_ABORTED): "ABORTED",
}


class RoutePlayerNode(Node):
    """Load a recorded route and hand it to Nav2."""

    def __init__(self) -> None:
        super().__init__("route_player")

        self.declare_parameter("input_yaml_path", "/tmp/navegacion_gps_route.yaml")
        self.declare_parameter("action_name", "navigate_through_poses")
        self.declare_parameter("server_wait_timeout_s", 10.0)

        self.input_yaml_path = Path(
            str(self.get_parameter("input_yaml_path").value)
        ).expanduser()
        self.action_name = str(self.get_parameter("action_name").value)
        self.server_wait_timeout_s = max(
            0.1, float(self.get_parameter("server_wait_timeout_s").value)
        )

        self._navigate_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.action_name,
        )

        self.get_logger().info(
            "route_player ready "
            f"(input={self.input_yaml_path}, action={self.action_name}, "
            f"wait_timeout={self.server_wait_timeout_s:.1f}s)"
        )

    @staticmethod
    def _build_pose(frame_id: str, point: RoutePoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(frame_id)
        pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_xyzw_from_yaw(point.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _load_poses(self) -> tuple[str, list[PoseStamped]]:
        frame_id, points = load_route_yaml(self.input_yaml_path)
        poses = [self._build_pose(frame_id, point) for point in points]
        if not poses:
            raise RouteFileError("route YAML does not contain any valid points")
        return frame_id, poses

    def run(self) -> int:
        """Execute the one-shot replay flow."""

        try:
            frame_id, poses = self._load_poses()
        except RouteFileError as exc:
            self.get_logger().error(str(exc))
            return 1

        self.get_logger().info(
            f"Loaded {len(poses)} route points from {self.input_yaml_path} "
            f"(frame_id={frame_id})"
        )

        if not self._navigate_client.wait_for_server(
            timeout_sec=self.server_wait_timeout_s
        ):
            self.get_logger().error(
                f"NavigateThroughPoses action server {self.action_name!r} is not available"
            )
            return 1

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info(
            f"Starting route replay with {len(poses)} poses on {self.action_name}"
        )
        send_future = self._navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("Failed to send NavigateThroughPoses goal")
            return 1
        if not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal was rejected")
            return 1

        self.get_logger().info("NavigateThroughPoses goal accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("Did not receive a result from NavigateThroughPoses")
            return 1

        status = int(wrapped_result.status)
        status_label = STATUS_LABELS.get(status, f"STATUS_{status}")
        if status == int(GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().info(
                f"Route replay finished successfully (poses={len(poses)}, status={status_label})"
            )
            return 0
        if status == int(GoalStatus.STATUS_CANCELED):
            self.get_logger().warning(
                f"Route replay was canceled (poses={len(poses)}, status={status_label})"
            )
            return 1

        self.get_logger().error(
            f"Route replay failed (poses={len(poses)}, status={status_label})"
        )
        return 1


def main(args: Optional[list[str]] = None) -> None:
    """Run the route player node and exit with its result code."""

    rclpy.init(args=args)
    node = RoutePlayerNode()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
