import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL


class YamlWaypointsFromLL(Node):
    def __init__(self, waypoints_file: str, service_name: str, map_frame: str):
        super().__init__("yaml_waypoints_from_ll")
        self._waypoints_file = waypoints_file
        self._service_name = service_name
        self._map_frame = map_frame
        self._client = self.create_client(FromLL, self._service_name)
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

    def _load_waypoints(self):
        with open(self._waypoints_file, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return data.get("waypoints", [])

    def _yaw_to_quaternion(self, yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _call_from_ll(self, latitude: float, longitude: float, altitude: float):
        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def build_pose_list(self):
        if not self._client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service '{self._service_name}' is not available")

        poses = []
        for wp in self._load_waypoints():
            latitude = float(wp.get("latitude", 0.0))
            longitude = float(wp.get("longitude", 0.0))
            altitude = float(wp.get("altitude", 0.0))
            yaw = float(wp.get("yaw", 0.0))

            response = self._call_from_ll(latitude, longitude, altitude)
            if response is None:
                raise RuntimeError("fromLL service call failed")

            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = response.map_point.x
            pose.pose.position.y = response.map_point.y
            pose.pose.position.z = response.map_point.z
            # Ignore waypoint yaw; keep neutral orientation.
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        return poses

    def send_waypoints(self, poses):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("follow_waypoints action server is not available")

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("follow_waypoints goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def _resolve_waypoints_file():
    default_yaml_file_path = os.path.join(
        get_package_share_directory("navegacion_gps"), "config", "default-waypoints.yaml"
    )
    if len(sys.argv) > 1:
        return sys.argv[1]
    return default_yaml_file_path


def main():
    rclpy.init()

    node = YamlWaypointsFromLL(
        waypoints_file=_resolve_waypoints_file(),
        service_name="/fromLL",
        map_frame="map",
    )

    try:
        poses = node.build_pose_list()
        result = node.send_waypoints(poses)
    finally:
        node.destroy_node()

    if result is not None and result.result is not None:
        print(f"Waypoints completed, missed: {result.result.missed_waypoints}")
    else:
        print("Waypoints completed")


if __name__ == "__main__":
    main()
