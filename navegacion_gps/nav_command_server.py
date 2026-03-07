import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix

from navegacion_gps_interfaces.msg import NavTelemetry
from navegacion_gps_interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    GetNavState,
    SetManualCmd,
    SetManualMode,
    SetNavGoalLL,
)


class NavCommandServerNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_command_server")

        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("fromll_service_fallback", "/navsat_transform/fromLL")
        self.declare_parameter("fromll_wait_timeout_s", 2.0)
        self.declare_parameter("fromll_call_retries", 4)
        self.declare_parameter("fromll_retry_delay_s", 0.15)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("brake_topic", "/cmd_vel_safe")
        self.declare_parameter("manual_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("brake_publish_count", 5)
        self.declare_parameter("brake_publish_interval_s", 0.1)
        self.declare_parameter("manual_cmd_timeout_s", 0.4)
        self.declare_parameter("manual_watchdog_hz", 10.0)
        self.declare_parameter("nav_telemetry_hz", 5.0)
        self.declare_parameter("telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("brake_service", "/nav_command_server/brake")
        self.declare_parameter("set_manual_mode_service", "/nav_command_server/set_manual_mode")
        self.declare_parameter("set_manual_cmd_service", "/nav_command_server/set_manual_cmd")
        self.declare_parameter("get_state_service", "/nav_command_server/get_state")

        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.fromll_service_fallback = str(
            self.get_parameter("fromll_service_fallback").value
        )
        self.fromll_wait_timeout_s = max(
            0.1, float(self.get_parameter("fromll_wait_timeout_s").value)
        )
        self.fromll_call_retries = max(
            1, int(self.get_parameter("fromll_call_retries").value)
        )
        self.fromll_retry_delay_s = max(
            0.0, float(self.get_parameter("fromll_retry_delay_s").value)
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)
        self.brake_topic = str(self.get_parameter("brake_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.brake_publish_count = max(1, int(self.get_parameter("brake_publish_count").value))
        self.brake_publish_interval_s = max(
            0.0, float(self.get_parameter("brake_publish_interval_s").value)
        )
        self.manual_cmd_timeout_s = max(
            0.1, float(self.get_parameter("manual_cmd_timeout_s").value)
        )
        self.manual_watchdog_hz = max(
            1.0, float(self.get_parameter("manual_watchdog_hz").value)
        )
        self.nav_telemetry_hz = max(1.0, float(self.get_parameter("nav_telemetry_hz").value))
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.set_goal_service = str(self.get_parameter("set_goal_service").value)
        self.cancel_goal_service = str(self.get_parameter("cancel_goal_service").value)
        self.brake_service = str(self.get_parameter("brake_service").value)
        self.set_manual_mode_service = str(
            self.get_parameter("set_manual_mode_service").value
        )
        self.set_manual_cmd_service = str(
            self.get_parameter("set_manual_cmd_service").value
        )
        self.get_state_service = str(self.get_parameter("get_state_service").value)

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._manual_enabled = False
        self._last_manual_cmd = Twist()
        self._last_manual_cmd_time: Optional[float] = None
        self._manual_watchdog_stop_sent = False
        self._last_cmd_vel_safe: Optional[Twist] = None
        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_telemetry_sent: Optional[float] = None

        # Service callbacks are mutually exclusive; clients/actions are reentrant to avoid
        # deadlocks when a service callback waits for a client future.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._fromll_client = self.create_client(
            FromLL, self.fromll_service, callback_group=self._client_group
        )
        self._fromll_fallback_client = None
        if self.fromll_service_fallback and (
            self.fromll_service_fallback != self.fromll_service
        ):
            self._fromll_fallback_client = self.create_client(
                FromLL,
                self.fromll_service_fallback,
                callback_group=self._client_group,
            )
        self._active_fromll_name: Optional[str] = None
        self._active_fromll_client: Optional[Any] = None
        self._last_fromll_error: Optional[str] = None
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._client_group,
        )

        self._telemetry_pub = self.create_publisher(NavTelemetry, self.telemetry_topic, 10)

        self._set_goal_srv = self.create_service(
            SetNavGoalLL,
            self.set_goal_service,
            self._on_set_goal,
            callback_group=self._service_group,
        )
        self._cancel_goal_srv = self.create_service(
            CancelNavGoal,
            self.cancel_goal_service,
            self._on_cancel_goal,
            callback_group=self._service_group,
        )
        self._brake_srv = self.create_service(
            BrakeNav,
            self.brake_service,
            self._on_brake,
            callback_group=self._service_group,
        )
        self._set_manual_mode_srv = self.create_service(
            SetManualMode,
            self.set_manual_mode_service,
            self._on_set_manual_mode,
            callback_group=self._service_group,
        )
        self._set_manual_cmd_srv = self.create_service(
            SetManualCmd,
            self.set_manual_cmd_service,
            self._on_set_manual_cmd,
            callback_group=self._service_group,
        )
        self._get_state_srv = self.create_service(
            GetNavState,
            self.get_state_service,
            self._on_get_state,
            callback_group=self._service_group,
        )

        self._brake_pub = self.create_publisher(Twist, self.brake_topic, 10)
        self._manual_cmd_pub = self.create_publisher(Twist, self.manual_cmd_topic, 10)
        self._gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self._on_gps_fix, 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_safe_topic, self._on_cmd_vel_safe, 10
        )

        self._manual_watchdog_timer = self.create_timer(
            1.0 / float(self.manual_watchdog_hz), self._manual_watchdog_tick
        )
        self.get_logger().info(
            "Nav command server ready "
            f"(set_goal={self.set_goal_service}, cancel={self.cancel_goal_service}, "
            f"brake={self.brake_service}, telemetry={self.telemetry_topic})"
        )
        self.get_logger().info(
            "Callback groups configured (services=MutuallyExclusive, clients=Reentrant)"
        )

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_sec:
                return None
            time.sleep(0.01)
        return None

    def _call_from_ll(self, lat: float, lon: float) -> Optional[Tuple[float, float, float]]:
        for attempt in range(self.fromll_call_retries):
            fromll_client = self._resolve_fromll_client()
            if fromll_client is None:
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            req = FromLL.Request()
            req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            future = fromll_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=2.5)
            except Exception as exc:
                self._last_fromll_error = str(exc)
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue
            if res is None:
                self._last_fromll_error = "timeout waiting fromLL response"
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            self._last_fromll_error = None
            return (float(res.map_point.x), float(res.map_point.y), float(res.map_point.z))

        self.get_logger().warning(
            "fromLL conversion failed "
            f"(lat={lat:.8f}, lon={lon:.8f}, reason={self._last_fromll_error or 'unknown'})"
        )
        return None

    def _resolve_fromll_client(self) -> Optional[Any]:
        candidates: list[tuple[Any, str, float]] = []
        if self._active_fromll_client is not None and self._active_fromll_name is not None:
            candidates.append((self._active_fromll_client, self._active_fromll_name, 0.05))

        candidates.append((self._fromll_client, self.fromll_service, self.fromll_wait_timeout_s))
        fallback = self._fromll_fallback_client
        if fallback is not None:
            candidates.append((fallback, self.fromll_service_fallback, self.fromll_wait_timeout_s))

        seen = set()
        for client, service_name, wait_s in candidates:
            key = (id(client), service_name)
            if key in seen:
                continue
            seen.add(key)
            if client.wait_for_service(timeout_sec=wait_s):
                self._active_fromll_client = client
                self._maybe_log_active_fromll(service_name)
                return client

        self.get_logger().warning(
            "fromLL service unavailable "
            f"(tried '{self.fromll_service}'"
            + (
                f" and '{self.fromll_service_fallback}'"
                if self._fromll_fallback_client is not None
                else ""
            )
            + ")"
        )
        self._last_fromll_error = "fromLL service unavailable"
        return None

    def _maybe_log_active_fromll(self, service_name: str) -> None:
        if self._active_fromll_name == service_name:
            return
        self._active_fromll_name = service_name
        self.get_logger().info(f"Using fromLL service: {service_name}")

    def _yaw_to_quaternion(self, yaw_deg: float) -> Quaternion:
        yaw_rad = math.radians(yaw_deg)
        half_yaw = yaw_rad / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def _cmd_vel_safe_payload_locked(self) -> Dict[str, Any]:
        if self._last_cmd_vel_safe is None:
            return {"available": False, "linear_x": 0.0, "angular_z": 0.0}
        msg = self._last_cmd_vel_safe
        return {
            "available": True,
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }

    def _manual_control_payload_locked(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self._manual_enabled),
            "linear_x_cmd": float(self._last_manual_cmd.linear.x),
            "angular_z_cmd": float(self._last_manual_cmd.angular.z),
        }

    def _fill_get_state_response(self, response: GetNavState.Response) -> None:
        with self._lock:
            goal_active = self._current_goal_handle is not None
            cmd_vel_safe = self._cmd_vel_safe_payload_locked()
            manual_control = self._manual_control_payload_locked()
            robot_pose = self._last_robot_pose

        response.ok = True
        response.error = ""
        response.goal_active = bool(goal_active)
        response.manual_enabled = bool(manual_control["enabled"])
        response.manual_linear_x_cmd = float(manual_control["linear_x_cmd"])
        response.manual_angular_z_cmd = float(manual_control["angular_z_cmd"])
        response.cmd_vel_available = bool(cmd_vel_safe["available"])
        response.cmd_vel_linear_x = float(cmd_vel_safe["linear_x"])
        response.cmd_vel_angular_z = float(cmd_vel_safe["angular_z"])

        if robot_pose is None:
            response.robot_lat = float("nan")
            response.robot_lon = float("nan")
        else:
            response.robot_lat = float(robot_pose["lat"])
            response.robot_lon = float(robot_pose["lon"])

    def _publish_telemetry(self, force: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            last_sent = self._last_telemetry_sent
            min_interval = 1.0 / float(self.nav_telemetry_hz)
            if (not force) and last_sent is not None and (now - last_sent) < min_interval:
                return
            self._last_telemetry_sent = now

            goal_active = self._current_goal_handle is not None
            manual_control = self._manual_control_payload_locked()
            cmd_vel_safe = self._cmd_vel_safe_payload_locked()
            robot_pose = self._last_robot_pose

        msg = NavTelemetry()
        msg.goal_active = bool(goal_active)
        msg.manual_enabled = bool(manual_control["enabled"])
        msg.manual_linear_x_cmd = float(manual_control["linear_x_cmd"])
        msg.manual_angular_z_cmd = float(manual_control["angular_z_cmd"])
        msg.cmd_vel_available = bool(cmd_vel_safe["available"])
        msg.cmd_vel_linear_x = float(cmd_vel_safe["linear_x"])
        msg.cmd_vel_angular_z = float(cmd_vel_safe["angular_z"])
        if robot_pose is None:
            msg.robot_lat = float("nan")
            msg.robot_lon = float("nan")
        else:
            msg.robot_lat = float(robot_pose["lat"])
            msg.robot_lon = float(robot_pose["lon"])
        self._telemetry_pub.publish(msg)

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return
        pose = {"lat": float(msg.latitude), "lon": float(msg.longitude)}
        with self._lock:
            self._last_robot_pose = pose
        self._publish_telemetry(force=False)

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        with self._lock:
            self._last_cmd_vel_safe = msg
        self._publish_telemetry(force=False)

    def _publish_manual_twist(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self._manual_cmd_pub.publish(cmd)

    def _publish_manual_stop(self) -> None:
        self._publish_manual_twist(0.0, 0.0)

    def send_nav2_goal(self, lat: float, lon: float, yaw_deg: float = 0.0) -> Tuple[bool, str]:
        self.get_logger().info(
            f"SetNavGoalLL request (lat={lat:.8f}, lon={lon:.8f}, yaw_deg={yaw_deg:.2f})"
        )
        converted = self._call_from_ll(lat, lon)
        if converted is None:
            detail = self._last_fromll_error or "unknown"
            return False, f"fromLL conversion failed: {detail}"
        x, y, _ = converted

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(yaw_deg)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("SetNavGoalLL failed: NavigateToPose action server not available")
            return False, "NavigateToPose action server not available"

        future = self._nav2_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future, timeout_sec=5.0)
        if goal_handle is None:
            self.get_logger().error("SetNavGoalLL failed: timeout sending goal to Nav2")
            return False, "failed to send goal"

        if not goal_handle.accepted:
            self.get_logger().warning("SetNavGoalLL rejected by Nav2")
            return False, "goal rejected by Nav2"

        with self._lock:
            self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result_done)
        self._publish_telemetry(force=True)
        self.get_logger().info(f"SetNavGoalLL accepted by Nav2 (x={x:.2f}, y={y:.2f})")
        return True, "goal accepted"

    def _on_goal_result_done(self, future: Any) -> None:
        status_text = "unknown"
        try:
            result_msg = future.result()
            status_text = str(getattr(result_msg, "status", "unknown"))
        except Exception as exc:
            status_text = f"exception:{exc}"
        self.get_logger().info(f"NavigateToPose result received (status={status_text})")
        with self._lock:
            self._current_goal_handle = None
        self._publish_telemetry(force=True)

    def cancel_current_goal(self) -> Tuple[bool, str]:
        with self._lock:
            handle = self._current_goal_handle
        if handle is None:
            return False, "no active goal"

        future = handle.cancel_goal_async()
        result = self._wait_for_future(future, timeout_sec=2.0)
        if result is None:
            return False, "timeout cancelling goal"

        with self._lock:
            self._current_goal_handle = None
        self._publish_telemetry(force=True)
        return True, "cancelled"

    def apply_brake(self) -> Tuple[bool, str]:
        cancel_ok = True
        cancel_msg = "no active goal"
        with self._lock:
            has_goal = self._current_goal_handle is not None
        if has_goal:
            cancel_ok, cancel_msg = self.cancel_current_goal()

        stop_cmd = Twist()
        for index in range(self.brake_publish_count):
            self._brake_pub.publish(stop_cmd)
            if index + 1 < self.brake_publish_count and self.brake_publish_interval_s > 0.0:
                time.sleep(self.brake_publish_interval_s)

        self._publish_telemetry(force=True)
        if cancel_ok:
            return True, "brake applied"
        return False, f"brake applied, but goal cancel failed: {cancel_msg}"

    def set_manual_mode(self, enabled: bool) -> Tuple[bool, str, bool]:
        if enabled:
            with self._lock:
                has_goal = self._current_goal_handle is not None
            if has_goal:
                cancel_ok, cancel_msg = self.cancel_current_goal()
                if not cancel_ok:
                    self.get_logger().warning(
                        f"Manual mode: failed to cancel goal ({cancel_msg}), continuing"
                    )

            with self._lock:
                self._manual_enabled = True
                self._last_manual_cmd = Twist()
                self._last_manual_cmd_time = None
                self._manual_watchdog_stop_sent = False
            self._publish_manual_stop()
            self._publish_telemetry(force=True)
            return True, "manual control enabled", True

        with self._lock:
            self._manual_enabled = False
            self._last_manual_cmd = Twist()
            self._last_manual_cmd_time = None
            self._manual_watchdog_stop_sent = False
        self._publish_manual_stop()
        self._publish_telemetry(force=True)
        return True, "manual control disabled", False

    def set_manual_cmd(self, linear_x: float, angular_z: float) -> Tuple[bool, str]:
        if not np.isfinite(linear_x) or not np.isfinite(angular_z):
            return False, "invalid manual command values"
        now = time.monotonic()
        with self._lock:
            if not self._manual_enabled:
                return False, "manual control is disabled"
            self._last_manual_cmd.linear.x = float(linear_x)
            self._last_manual_cmd.angular.z = float(angular_z)
            self._last_manual_cmd_time = now
            self._manual_watchdog_stop_sent = False
        self._publish_manual_twist(linear_x, angular_z)
        self._publish_telemetry(force=False)
        return True, "manual command published"

    def _manual_watchdog_tick(self) -> None:
        with self._lock:
            enabled = bool(self._manual_enabled)
            last_cmd_time = self._last_manual_cmd_time
            stop_sent = bool(self._manual_watchdog_stop_sent)

        if not enabled:
            return

        now = time.monotonic()
        stale = (
            (last_cmd_time is None)
            or ((now - last_cmd_time) > float(self.manual_cmd_timeout_s))
        )
        if stale and (not stop_sent):
            self._publish_manual_stop()
            with self._lock:
                self._last_manual_cmd = Twist()
                self._manual_watchdog_stop_sent = True
            self._publish_telemetry(force=True)

    def _on_set_goal(
        self,
        request: SetNavGoalLL.Request,
        response: SetNavGoalLL.Response,
    ) -> SetNavGoalLL.Response:
        with self._lock:
            manual_enabled = self._manual_enabled
        if manual_enabled:
            response.ok = False
            response.error = "manual control enabled; disable manual mode to send goals"
            return response

        ok, err = self.send_nav2_goal(
            float(request.lat), float(request.lon), float(request.yaw_deg)
        )
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        if not response.ok:
            self.get_logger().warning(f"SetNavGoalLL response failed: {response.error}")
        return response

    def _on_cancel_goal(
        self,
        _request: CancelNavGoal.Request,
        response: CancelNavGoal.Response,
    ) -> CancelNavGoal.Response:
        ok, err = self.cancel_current_goal()
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        self.get_logger().info(
            f"CancelNavGoal response (ok={response.ok}, error='{response.error}')"
        )
        return response

    def _on_brake(
        self,
        _request: BrakeNav.Request,
        response: BrakeNav.Response,
    ) -> BrakeNav.Response:
        ok, err = self.apply_brake()
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        self.get_logger().info(
            f"BrakeNav response (ok={response.ok}, error='{response.error}')"
        )
        return response

    def _on_set_manual_mode(
        self,
        request: SetManualMode.Request,
        response: SetManualMode.Response,
    ) -> SetManualMode.Response:
        ok, err, enabled_after = self.set_manual_mode(bool(request.enabled))
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        response.enabled_after = bool(enabled_after)
        self.get_logger().info(
            f"SetManualMode response (requested={bool(request.enabled)}, "
            f"enabled_after={response.enabled_after}, ok={response.ok}, error='{response.error}')"
        )
        return response

    def _on_set_manual_cmd(
        self,
        request: SetManualCmd.Request,
        response: SetManualCmd.Response,
    ) -> SetManualCmd.Response:
        ok, err = self.set_manual_cmd(float(request.linear_x), float(request.angular_z))
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        if not response.ok:
            self.get_logger().warning(
                "SetManualCmd rejected "
                f"(linear_x={float(request.linear_x):.3f}, angular_z={float(request.angular_z):.3f}, "
                f"error='{response.error}')"
            )
        return response

    def _on_get_state(
        self,
        _request: GetNavState.Request,
        response: GetNavState.Response,
    ) -> GetNavState.Response:
        self._fill_get_state_response(response)
        return response


def main() -> None:
    rclpy.init()
    node = NavCommandServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
