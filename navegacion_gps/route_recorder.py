"""Record a manually driven route using the filtered robot pose."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from navegacion_gps.route_tools import (
    RoutePoint,
    angular_distance_rad,
    planar_distance_m,
    route_point_almost_equal,
    save_route_yaml,
    should_record_point,
    yaw_from_quaternion_xyzw,
)

FRAME_WARNING_PERIOD_NS = 5_000_000_000


class RouteRecorderNode(Node):
    """Record sparse waypoints from a filtered pose topic."""

    def __init__(self) -> None:
        super().__init__("route_recorder")

        self.declare_parameter("pose_topic", "/odometry/global")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("min_distance_m", 1.0)
        self.declare_parameter("min_yaw_deg", 10.0)
        self.declare_parameter("output_yaml_path", "/tmp/navegacion_gps_route.yaml")
        self.declare_parameter("autostart_recording", False)
        self.declare_parameter("start_service", "/route_recorder/start")
        self.declare_parameter("stop_service", "/route_recorder/stop")

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.expected_frame_id = str(self.get_parameter("expected_frame_id").value).strip()
        self.min_distance_m = max(0.0, float(self.get_parameter("min_distance_m").value))
        self.min_yaw_deg = max(0.0, float(self.get_parameter("min_yaw_deg").value))
        self.min_yaw_rad = math.radians(self.min_yaw_deg)
        self.output_yaml_path = Path(
            str(self.get_parameter("output_yaml_path").value)
        ).expanduser()
        self.autostart_recording = bool(
            self.get_parameter("autostart_recording").value
        )
        self.start_service_name = str(self.get_parameter("start_service").value)
        self.stop_service_name = str(self.get_parameter("stop_service").value)

        self._recording = False
        self._points: list[RoutePoint] = []
        self._latest_point: Optional[RoutePoint] = None
        self._latest_frame_id = ""
        self._last_frame_warning_ns = 0

        self.create_subscription(Odometry, self.pose_topic, self._on_odometry, 20)
        self.create_service(Trigger, self.start_service_name, self._on_start)
        self.create_service(Trigger, self.stop_service_name, self._on_stop)

        self.get_logger().info(
            "route_recorder ready "
            f"(pose_topic={self.pose_topic}, expected_frame={self.expected_frame_id or '<any>'}, "
            f"min_distance={self.min_distance_m:.2f}m, min_yaw={self.min_yaw_deg:.1f}deg, "
            f"output={self.output_yaml_path})"
        )

        if self.autostart_recording:
            self._start_recording("autostart")

    def is_recording(self) -> bool:
        """Return True when a route is currently being captured."""

        return bool(self._recording)

    def _throttled_frame_warning(self, frame_id: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_frame_warning_ns) < FRAME_WARNING_PERIOD_NS:
            return
        self._last_frame_warning_ns = now_ns
        self.get_logger().warning(
            f"Ignoring pose from frame {frame_id!r}; expected {self.expected_frame_id!r}"
        )

    def _extract_point(self, msg: Odometry) -> Optional[RoutePoint]:
        frame_id = str(msg.header.frame_id).strip()
        if self.expected_frame_id and frame_id != self.expected_frame_id:
            self._throttled_frame_warning(frame_id)
            return None

        orientation = msg.pose.pose.orientation
        point = RoutePoint(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quaternion_xyzw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        self._latest_point = point
        self._latest_frame_id = frame_id
        return point

    def _append_point(self, point: RoutePoint, reason: str) -> None:
        self._points.append(point)
        self.get_logger().info(
            f"Saved waypoint {len(self._points)} "
            f"(x={point.x:.2f}, y={point.y:.2f}, yaw={math.degrees(point.yaw):.1f}deg, "
            f"reason={reason})"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        current = self._extract_point(msg)
        if current is None or (not self._recording):
            return

        if not self._points:
            self._append_point(current, "first")
            return

        last_saved = self._points[-1]
        if should_record_point(
            last_saved=last_saved,
            current=current,
            min_distance_m=self.min_distance_m,
            min_yaw_rad=self.min_yaw_rad,
        ):
            distance_m = planar_distance_m(last_saved, current)
            yaw_delta_deg = math.degrees(
                angular_distance_rad(current.yaw, last_saved.yaw)
            )
            self._append_point(
                current,
                f"distance={distance_m:.2f}m yaw_delta={yaw_delta_deg:.1f}deg",
            )

    def _route_frame_id(self) -> str:
        if self.expected_frame_id:
            return self.expected_frame_id
        if self._latest_frame_id:
            return self._latest_frame_id
        return "map"

    def _start_recording(self, reason: str) -> tuple[bool, str]:
        if self._recording:
            return False, "recording already active"

        self._points = []
        self._recording = True
        self.get_logger().info(f"Route recording started (reason={reason})")

        if self._latest_point is not None:
            self._append_point(self._latest_point, "first")
        else:
            self.get_logger().info("Waiting for first valid pose sample before saving points")
        return True, "recording started"

    def _stop_recording(self, reason: str) -> tuple[bool, str]:
        if not self._recording:
            return False, "recording is not active"

        self._recording = False
        if self._latest_point is not None:
            if not self._points:
                self._append_point(self._latest_point, "last")
            elif not route_point_almost_equal(self._points[-1], self._latest_point):
                self._append_point(self._latest_point, "last")

        if not self._points:
            return False, "no valid pose samples were recorded"

        try:
            save_route_yaml(
                path=self.output_yaml_path,
                frame_id=self._route_frame_id(),
                points=self._points,
            )
        except OSError as exc:
            self.get_logger().error(f"Failed saving route YAML: {exc}")
            return False, f"failed saving route: {exc}"

        message = (
            f"saved {len(self._points)} points to {self.output_yaml_path} "
            f"(frame_id={self._route_frame_id()}, reason={reason})"
        )
        self.get_logger().info(f"Route recording stopped: {message}")
        return True, message

    def _on_start(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        success, message = self._start_recording("service_start")
        response.success = bool(success)
        response.message = str(message)
        return response

    def _on_stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        success, message = self._stop_recording("service_stop")
        response.success = bool(success)
        response.message = str(message)
        return response


def main(args: Optional[list[str]] = None) -> None:
    """Run the route recorder node."""

    rclpy.init(args=args)
    node = RouteRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_recording():
            ok, message = node._stop_recording("shutdown")
            if ok:
                node.get_logger().info(f"Route saved on shutdown: {message}")
            else:
                node.get_logger().warning(f"Route was not saved on shutdown: {message}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
