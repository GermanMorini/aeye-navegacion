import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from robot_localization.srv import FromLL


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._from_ll = self.create_client(FromLL, "/fromLL")
        self._pending_poses = []
        self._goal_in_flight = False

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

    def mapviz_wp_cb(self, msg: PointStamped):
        """Clicked point callback, converts to map and queues a Pose for Nav2."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        if msg.header.frame_id == "map":
            pose.header.frame_id = "map"
            pose.pose.position = msg.point
            pose.pose.orientation.w = 1.0
            self._enqueue_pose(pose)
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
        self._enqueue_pose(pose)

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
