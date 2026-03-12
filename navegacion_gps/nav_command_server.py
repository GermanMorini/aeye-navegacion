import math
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix

from interfaces.msg import NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    GetNavState,
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
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")
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
        self.declare_parameter("get_state_service", "/nav_command_server/get_state")
        self.declare_parameter("follow_waypoints_action", "follow_waypoints")

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
        self.teleop_cmd_topic = str(self.get_parameter("teleop_cmd_topic").value)
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
        self.get_state_service = str(self.get_parameter("get_state_service").value)
        self.follow_waypoints_action = str(
            self.get_parameter("follow_waypoints_action").value
        )

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._manual_enabled = False
        self._last_manual_cmd = Twist()
        self._last_manual_cmd_time: Optional[float] = None
        self._manual_watchdog_stop_sent = False
        self._last_cmd_vel_safe: Optional[Twist] = None
        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_telemetry_sent: Optional[float] = None
        self._loop_waypoint_poses: List[PoseStamped] = []
        self._loop_enabled = False
        self._loop_restart_pending = False

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
        self._follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            self.follow_waypoints_action,
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
        self._teleop_cmd_sub = self.create_subscription(
            Twist, self.teleop_cmd_topic, self._on_teleop_cmd, 10
        )

        self._manual_watchdog_timer = self.create_timer(
            1.0 / float(self.manual_watchdog_hz), self._manual_watchdog_tick
        )
        self._waypoint_loop_timer = self.create_timer(0.25, self._waypoint_loop_tick)
        self.get_logger().info(
            "Nav command server ready "
            f"(set_goal={self.set_goal_service}, cancel={self.cancel_goal_service}, "
            f"brake={self.brake_service}, telemetry={self.telemetry_topic}, "
            f"teleop_topic={self.teleop_cmd_topic}, "
            f"follow_waypoints_action={self.follow_waypoints_action})"
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

    def _on_teleop_cmd(self, msg: Twist) -> None:
        ok, err = self.set_manual_cmd(float(msg.linear.x), float(msg.angular.z))
        if (not ok) and (err != "manual control is disabled"):
            self.get_logger().warning(
                "Teleop cmd rejected "
                f"(linear_x={float(msg.linear.x):.3f}, angular_z={float(msg.angular.z):.3f}, "
                f"error='{err}')"
            )

    def _publish_manual_twist(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self._manual_cmd_pub.publish(cmd)

    def _publish_manual_stop(self) -> None:
        self._publish_manual_twist(0.0, 0.0)

    def _clear_loop_config_locked(self) -> None:
        self._loop_waypoint_poses = []
        self._loop_enabled = False
        self._loop_restart_pending = False

    def _build_pose_from_ll(self, lat: float, lon: float, yaw_deg: float) -> Optional[PoseStamped]:
        converted = self._call_from_ll(lat, lon)
        if converted is None:
            return None
        x, y, _ = converted

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(yaw_deg)
        return pose

    def _convert_waypoints_to_poses(
        self, waypoints: Sequence[Tuple[float, float, float]]
    ) -> Tuple[Optional[List[PoseStamped]], str]:
        poses: List[PoseStamped] = []
        for idx, waypoint in enumerate(waypoints):
            lat, lon, yaw_deg = waypoint
            pose = self._build_pose_from_ll(lat, lon, yaw_deg)
            if pose is None:
                detail = self._last_fromll_error or "unknown"
                return None, f"fromLL conversion failed at waypoint {idx + 1}: {detail}"
            poses.append(pose)
        return poses, ""

    def _send_follow_waypoints_goal(
        self, poses: Sequence[PoseStamped], loop_enabled: bool, reason: str
    ) -> Tuple[bool, str]:
        if not poses:
            return False, "no waypoint poses to send"

        if not self._follow_waypoints_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "SetNavGoalLL failed: FollowWaypoints action server not available"
            )
            return False, "FollowWaypoints action server not available"

        goal = FollowWaypoints.Goal()
        goal.poses = list(poses)

        future = self._follow_waypoints_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future, timeout_sec=5.0)
        if goal_handle is None:
            self.get_logger().error("SetNavGoalLL failed: timeout sending FollowWaypoints goal")
            return False, "failed to send FollowWaypoints goal"
        if not goal_handle.accepted:
            self.get_logger().warning("SetNavGoalLL rejected by FollowWaypoints")
            return False, "goal rejected by FollowWaypoints"

        with self._lock:
            self._current_goal_handle = goal_handle
            self._loop_waypoint_poses = list(poses)
            self._loop_enabled = bool(loop_enabled and (len(poses) > 1))
            self._loop_restart_pending = False

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_follow_waypoints_result_done)

        self.get_logger().info(
            "FollowWaypoints goal accepted "
            f"(waypoints={len(poses)}, loop={bool(loop_enabled and (len(poses) > 1))}, "
            f"reason={reason})"
        )
        self._publish_telemetry(force=True)
        return True, "goal accepted"

    def send_nav2_goals(
        self, waypoints: Sequence[Tuple[float, float, float]], loop_enabled: bool
    ) -> Tuple[bool, str]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required"

        with self._lock:
            has_goal = self._current_goal_handle is not None
        if has_goal:
            cancel_ok, cancel_msg = self.cancel_current_goal(clear_loop_config=True)
            if not cancel_ok:
                return False, f"failed to cancel previous goal: {cancel_msg}"

        with self._lock:
            self._clear_loop_config_locked()

        self.get_logger().info(
            f"SetNavGoalLL request (waypoints={len(waypoints)}, loop={bool(loop_enabled)})"
        )
        poses, err = self._convert_waypoints_to_poses(waypoints)
        if poses is None:
            return False, err
        return self._send_follow_waypoints_goal(
            poses=poses,
            loop_enabled=loop_enabled,
            reason="set_goal_service",
        )

    def _on_follow_waypoints_result_done(self, future: Any) -> None:
        status = None
        missed_waypoints: List[int] = []
        try:
            result_msg = future.result()
            status = int(getattr(result_msg, "status", -1))
            result = getattr(result_msg, "result", None)
            if result is not None:
                missed_waypoints = [int(v) for v in getattr(result, "missed_waypoints", [])]
        except Exception as exc:
            self.get_logger().warning(f"FollowWaypoints result callback failed: {exc}")

        should_restart = False
        with self._lock:
            self._current_goal_handle = None
            if (
                status == GoalStatus.STATUS_SUCCEEDED
                and self._loop_enabled
                and len(self._loop_waypoint_poses) > 1
                and (not self._manual_enabled)
            ):
                self._loop_restart_pending = True
                should_restart = True

        self.get_logger().info(
            "FollowWaypoints result received "
            f"(status={status}, missed_waypoints={missed_waypoints}, loop_restart={should_restart})"
        )
        self._publish_telemetry(force=True)

    def _waypoint_loop_tick(self) -> None:
        with self._lock:
            if (
                (not self._loop_restart_pending)
                or (not self._loop_enabled)
                or self._manual_enabled
                or (self._current_goal_handle is not None)
                or (len(self._loop_waypoint_poses) <= 1)
            ):
                return
            poses = list(self._loop_waypoint_poses)
            self._loop_restart_pending = False

        ok, err = self._send_follow_waypoints_goal(
            poses=poses,
            loop_enabled=True,
            reason="loop_restart",
        )
        if not ok:
            self.get_logger().warning(f"FollowWaypoints loop restart failed: {err}")
            with self._lock:
                if self._loop_enabled and (not self._manual_enabled):
                    self._loop_restart_pending = True

    def cancel_current_goal(self, clear_loop_config: bool = True) -> Tuple[bool, str]:
        with self._lock:
            handle = self._current_goal_handle
            if clear_loop_config:
                self._clear_loop_config_locked()
        if handle is None:
            return False, "no active goal"

        future = handle.cancel_goal_async()
        result = self._wait_for_future(future, timeout_sec=2.0)
        if result is None:
            return False, "timeout cancelling goal"

        with self._lock:
            self._current_goal_handle = None
            if clear_loop_config:
                self._clear_loop_config_locked()
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

    def _parse_set_goal_request(
        self, request: SetNavGoalLL.Request
    ) -> Tuple[Optional[List[Tuple[float, float, float]]], bool, str]:
        lats = [float(v) for v in request.lats]
        lons = [float(v) for v in request.lons]
        yaws = [float(v) for v in request.yaws_deg]

        has_array_payload = bool(lats) or bool(lons) or bool(yaws)
        if has_array_payload:
            if len(lats) != len(lons):
                return None, False, "lats and lons must have the same length"
            if len(lats) == 0:
                return None, False, "at least one waypoint is required"
            if len(yaws) not in (0, len(lats)):
                return None, False, "yaws_deg must be empty or match lats length"

            waypoints: List[Tuple[float, float, float]] = []
            for idx in range(len(lats)):
                yaw_deg = yaws[idx] if len(yaws) == len(lats) else 0.0
                lat = float(lats[idx])
                lon = float(lons[idx])
                if (not np.isfinite(lat)) or (not np.isfinite(lon)) or (not np.isfinite(yaw_deg)):
                    return None, False, f"invalid waypoint values at index {idx}"
                waypoints.append((lat, lon, float(yaw_deg)))
            loop_enabled = bool(request.loop) or (len(waypoints) > 1)
            return waypoints, loop_enabled, ""

        lat = float(request.lat)
        lon = float(request.lon)
        yaw_deg = float(request.yaw_deg)
        if (not np.isfinite(lat)) or (not np.isfinite(lon)) or (not np.isfinite(yaw_deg)):
            return None, False, "invalid waypoint values"
        return [(lat, lon, yaw_deg)], bool(request.loop), ""

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

        waypoints, loop_enabled, parse_err = self._parse_set_goal_request(request)
        if waypoints is None:
            response.ok = False
            response.error = parse_err
            return response

        ok, err = self.send_nav2_goals(
            waypoints=waypoints,
            loop_enabled=loop_enabled,
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
