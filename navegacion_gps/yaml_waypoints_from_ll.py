import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL


class YamlWaypointsFromLL(Node):
    def __init__(self, waypoints_file: str, service_name: str, map_frame: str):
        super().__init__("yaml_waypoints_from_ll")
        for name, default in (
            ("costmap_service", "/global_costmap/get_costmap"),
            ("reject_outside_global_costmap", True),
            ("clamp_outside_global_costmap", False),
            ("costmap_timeout", 2.0),
        ):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self._waypoints_file = waypoints_file
        self._service_name = service_name
        self._map_frame = map_frame
        self._client = self.create_client(FromLL, self._service_name)
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._costmap_service = self.get_parameter("costmap_service").value
        self._reject_outside_global_costmap = self.get_parameter(
            "reject_outside_global_costmap"
        ).value
        self._clamp_outside_global_costmap = self.get_parameter(
            "clamp_outside_global_costmap"
        ).value
        self._costmap_timeout = float(self.get_parameter("costmap_timeout").value)
        self._costmap_client = self.create_client(GetCostmap, self._costmap_service)
        self._costmap = None

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

    def _get_costmap(self):
        if self._costmap is not None:
            return self._costmap
        if not self._costmap_client.wait_for_service(timeout_sec=self._costmap_timeout):
            self.get_logger().warning(
                "Global costmap service not available; skipping bounds check"
            )
            return None
        future = self._costmap_client.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._costmap_timeout)
        response = future.result()
        if response is None:
            self.get_logger().warning("Failed to get global costmap")
            return None
        self._costmap = response.map
        return self._costmap

    def _costmap_bounds(self):
        if self._costmap is None:
            return None
        info = self._costmap.metadata
        if info.resolution <= 0.0 or info.size_x == 0 or info.size_y == 0:
            return None
        min_x = info.origin.position.x
        min_y = info.origin.position.y
        max_x = min_x + (info.size_x * info.resolution)
        max_y = min_y + (info.size_y * info.resolution)
        return (min_x, min_y, max_x, max_y, info.resolution)

    def _filter_pose(self, pose: PoseStamped):
        if not self._reject_outside_global_costmap:
            return pose
        self._get_costmap()
        bounds = self._costmap_bounds()
        if bounds is None:
            self.get_logger().warning(
                "Global costmap not available; skipping waypoint"
            )
            return None

        min_x, min_y, max_x, max_y, resolution = bounds
        x = pose.pose.position.x
        y = pose.pose.position.y
        inside = (min_x <= x < max_x) and (min_y <= y < max_y)
        if inside:
            return pose

        if self._clamp_outside_global_costmap:
            eps = resolution * 0.5
            pose.pose.position.x = min(max(x, min_x + eps), max_x - eps)
            pose.pose.position.y = min(max(y, min_y + eps), max_y - eps)
            self.get_logger().warning(
                "Waypoint outside global costmap, clamping to nearest edge"
            )
            return pose

        self.get_logger().warning("Waypoint outside global costmap, skipping")
        return None

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
            filtered = self._filter_pose(pose)
            if filtered is not None:
                poses.append(filtered)
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
        if not poses:
            print("No valid waypoints to send (outside global costmap)")
            return
        result = node.send_waypoints(poses)
    finally:
        node.destroy_node()

    if result is not None and result.result is not None:
        print(f"Waypoints completed, missed: {result.result.missed_waypoints}")
    else:
        print("Waypoints completed")


if __name__ == "__main__":
    main()
