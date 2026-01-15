import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import GetCostmap
from robot_localization.srv import FromLL


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        for name, default in (
            ("costmap_service", "/global_costmap/get_costmap"),
            ("reject_outside_global_costmap", True),
            ("clamp_outside_global_costmap", False),
            ("costmap_update_period", 2.0),
        ):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self._costmap_service = self.get_parameter("costmap_service").value
        self._reject_outside_global_costmap = self.get_parameter(
            "reject_outside_global_costmap"
        ).value
        self._clamp_outside_global_costmap = self.get_parameter(
            "clamp_outside_global_costmap"
        ).value
        self._costmap_update_period = float(
            self.get_parameter("costmap_update_period").value
        )
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._from_ll = self.create_client(FromLL, "/fromLL")
        self._costmap_client = self.create_client(GetCostmap, self._costmap_service)
        self._costmap = None
        self._costmap_inflight = False
        self._pending_poses = []
        self._goal_in_flight = False

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        if self._reject_outside_global_costmap and self._costmap_update_period > 0.0:
            self._costmap_timer = self.create_timer(
                self._costmap_update_period, self._update_costmap
            )
            self._update_costmap()

    def mapviz_wp_cb(self, msg: PointStamped):
        """Clicked point callback, converts to map and queues a Pose for Nav2."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        if msg.header.frame_id == "map":
            pose.header.frame_id = "map"
            pose.pose.position = msg.point
            pose.pose.orientation.w = 1.0
            filtered = self._filter_pose(pose)
            if filtered is not None:
                self._enqueue_pose(filtered)
        elif msg.header.frame_id == "wgs84":
            if not self._from_ll.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning("Service /fromLL not available, cannot convert GPS to map")
                return
            request = FromLL.Request()
            request.ll_point.latitude = msg.point.y
            request.ll_point.longitude = msg.point.x
            request.ll_point.altitude = msg.point.z
            future = self._from_ll.call_async(request)
            future.add_done_callback(self._from_ll_done)
        else:
            self.get_logger().warning(
                f"Received point in unsupported frame '{msg.header.frame_id}'"
            )
            return

    def _from_ll_done(self, future):
        response = future.result()
        if response is None:
            self.get_logger().warning("Failed to convert GPS point to map")
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = response.map_point.x
        pose.pose.position.y = response.map_point.y
        pose.pose.position.z = response.map_point.z
        pose.pose.orientation.w = 1.0
        filtered = self._filter_pose(pose)
        if filtered is not None:
            self._enqueue_pose(filtered)

    def _update_costmap(self):
        if self._costmap_inflight:
            return
        if not self._costmap_client.service_is_ready():
            return
        self._costmap_inflight = True
        future = self._costmap_client.call_async(GetCostmap.Request())
        future.add_done_callback(self._costmap_done)

    def _costmap_done(self, future):
        self._costmap_inflight = False
        response = future.result()
        if response is None:
            self.get_logger().warning("Failed to get global costmap")
            return
        self._costmap = response.map

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

    def _enqueue_pose(self, pose: PoseStamped):
        self._pending_poses.append(pose)
        self._maybe_send_goal()

    def _maybe_send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("follow_waypoints action server not available")
            return

        if self._goal_in_flight or not self._pending_poses:
            return

        goal = FollowWaypoints.Goal()
        goal.poses = list(self._pending_poses)
        self._pending_poses.clear()
        self._goal_in_flight = True

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_sent)

    def _goal_sent(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("Waypoint goal was rejected")
            self._goal_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_done)

    def _goal_done(self, future):
        self._goal_in_flight = False
        self._maybe_send_goal()


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    rclpy.spin(gps_wpf)


if __name__ == "__main__":
    main()
