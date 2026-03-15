import json
import math
import threading
import time
from functools import partial
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Twist
from interfaces.msg import CmdVelFinal, NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    GetNavState,
    SetManualMode,
    SetNavGoalLL,
)
from nav2_msgs.action import ComputeAndTrackRoute, FollowPath
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import SetRouteGraph
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix


MODE_MANUAL = "manual"
MODE_AUTO_LOOP = "auto_loop"
MODE_AUTO_P2P = "auto_p2p"


class NavCommandServerNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_command_server")

        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("fromll_service_fallback", "/navsat_transform/fromLL")
        self.declare_parameter("fromll_wait_timeout_s", 2.0)
        self.declare_parameter("fromll_call_timeout_s", 2.5)
        self.declare_parameter("fromll_call_retries", 2)
        self.declare_parameter("fromll_retry_delay_s", 0.15)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")
        self.declare_parameter("cmd_vel_final_topic", "/cmd_vel_final")
        self.declare_parameter("collision_monitor_state_topic", "/collision_monitor_state")

        self.declare_parameter("brake_topic", "/cmd_vel_safe")
        self.declare_parameter("manual_cmd_topic", "/cmd_vel_safe")

        self.declare_parameter("brake_publish_count", 5)
        self.declare_parameter("brake_publish_interval_s", 0.1)
        self.declare_parameter("brake_pct_on_stop", 100)

        self.declare_parameter("manual_cmd_timeout_s", 0.4)
        self.declare_parameter("manual_watchdog_hz", 10.0)

        self.declare_parameter("nav_telemetry_hz", 5.0)
        self.declare_parameter("telemetry_topic", "/nav_command_server/telemetry")

        self.declare_parameter("set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("brake_service", "/nav_command_server/brake")
        self.declare_parameter(
            "set_manual_mode_service", "/nav_command_server/set_manual_mode"
        )
        self.declare_parameter("get_state_service", "/nav_command_server/get_state")

        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter(
            "compute_and_track_route_action", "compute_and_track_route"
        )
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("set_route_graph_service", "/route_server/set_route_graph")

        self.declare_parameter("route_first_path_timeout_s", 2.0)
        self.declare_parameter(
            "route_graph_temp_filepath", "/tmp/nav_command_server_route_graph.geojson"
        )
        self.declare_parameter("route_nav_to_pose_bt_xml", "")
        self.declare_parameter("legacy_nav_to_pose_bt_xml", "")
        self.declare_parameter("follow_path_controller_id", "")
        self.declare_parameter("follow_path_goal_checker_id", "")

        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.fromll_service_fallback = str(
            self.get_parameter("fromll_service_fallback").value
        )
        self.fromll_wait_timeout_s = max(
            0.05, float(self.get_parameter("fromll_wait_timeout_s").value)
        )
        self.fromll_call_timeout_s = max(
            0.1, float(self.get_parameter("fromll_call_timeout_s").value)
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
        self.teleop_cmd_topic = str(self.get_parameter("teleop_cmd_topic").value)
        self.cmd_vel_final_topic = str(self.get_parameter("cmd_vel_final_topic").value)
        self.collision_monitor_state_topic = str(
            self.get_parameter("collision_monitor_state_topic").value
        )

        self.brake_publish_count = max(
            1, int(self.get_parameter("brake_publish_count").value)
        )
        self.brake_publish_interval_s = max(
            0.01, float(self.get_parameter("brake_publish_interval_s").value)
        )
        self.brake_pct_on_stop = max(
            0, min(100, int(self.get_parameter("brake_pct_on_stop").value))
        )

        self.manual_cmd_timeout_s = max(
            0.05, float(self.get_parameter("manual_cmd_timeout_s").value)
        )
        self.manual_watchdog_hz = max(
            1.0, float(self.get_parameter("manual_watchdog_hz").value)
        )

        self.nav_telemetry_hz = max(
            1.0, float(self.get_parameter("nav_telemetry_hz").value)
        )
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)

        self.set_goal_service = str(self.get_parameter("set_goal_service").value)
        self.cancel_goal_service = str(self.get_parameter("cancel_goal_service").value)
        self.brake_service = str(self.get_parameter("brake_service").value)
        self.set_manual_mode_service = str(
            self.get_parameter("set_manual_mode_service").value
        )
        self.get_state_service = str(self.get_parameter("get_state_service").value)

        self.compute_and_track_route_action = str(
            self.get_parameter("compute_and_track_route_action").value
        )
        self.follow_path_action = str(self.get_parameter("follow_path_action").value)
        self.set_route_graph_service = str(
            self.get_parameter("set_route_graph_service").value
        )
        self.route_first_path_timeout_s = max(
            0.1, float(self.get_parameter("route_first_path_timeout_s").value)
        )
        self.route_graph_temp_filepath = Path(
            str(self.get_parameter("route_graph_temp_filepath").value)
        )
        self.follow_path_controller_id = str(
            self.get_parameter("follow_path_controller_id").value
        )
        self.follow_path_goal_checker_id = str(
            self.get_parameter("follow_path_goal_checker_id").value
        )

        self._lock = threading.Lock()

        self._mode = MODE_AUTO_P2P
        self._goal_active = False
        self._loop_enabled = False
        self._manual_enabled = False
        self._manual_linear_x_cmd = 0.0
        self._manual_angular_z_cmd = 0.0
        self._manual_cmd_last_monotonic: Optional[float] = None
        self._manual_watchdog_last_brake: Optional[float] = None

        self._cmd_vel_available = False
        self._cmd_vel_linear_x = 0.0
        self._cmd_vel_angular_z = 0.0

        self._robot_lat = float("nan")
        self._robot_lon = float("nan")

        self._collision_stop_active = False
        self._last_collision_brake_monotonic: Optional[float] = None

        self._mission_token = 0
        self._initial_segments: List[Tuple[int, int]] = []
        self._loop_segments: List[Tuple[int, int]] = []
        self._initial_cursor = 0
        self._loop_cursor = 0
        self._segment_started_monotonic: Optional[float] = None
        self._follow_path_started = False
        self._follow_goal_seq = 0
        self._active_leg_seq = 0
        self._active_leg_closed = False
        self._active_segment: Optional[Tuple[int, int]] = None
        self._last_path_signature: Optional[Tuple[int, int, int, int, int]] = None

        self._compute_goal_handle = None
        self._follow_goal_handle = None
        self._last_fromll_error: Optional[str] = None

        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_action_group = ReentrantCallbackGroup()

        self._fromll_client = self.create_client(
            FromLL,
            self.fromll_service,
            callback_group=self._client_action_group,
        )
        self._fromll_fallback_client = None
        if self.fromll_service_fallback and (
            self.fromll_service_fallback != self.fromll_service
        ):
            self._fromll_fallback_client = self.create_client(
                FromLL,
                self.fromll_service_fallback,
                callback_group=self._client_action_group,
            )
        self._active_fromll_client: Optional[Any] = None
        self._active_fromll_service_name: Optional[str] = None

        self._set_route_graph_client = self.create_client(
            SetRouteGraph,
            self.set_route_graph_service,
            callback_group=self._client_action_group,
        )

        self._compute_route_client = ActionClient(
            self,
            ComputeAndTrackRoute,
            self.compute_and_track_route_action,
            callback_group=self._client_action_group,
        )
        self._follow_path_client = ActionClient(
            self,
            FollowPath,
            self.follow_path_action,
            callback_group=self._client_action_group,
        )

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, 10
        )
        self._cmd_vel_safe_sub = self.create_subscription(
            Twist,
            self.cmd_vel_safe_topic,
            self._on_cmd_vel_safe,
            10,
        )
        self._teleop_sub = self.create_subscription(
            CmdVelFinal,
            self.teleop_cmd_topic,
            self._on_teleop_cmd,
            10,
        )
        self._collision_sub = self.create_subscription(
            CollisionMonitorState,
            self.collision_monitor_state_topic,
            self._on_collision_state,
            10,
        )

        self._cmd_vel_final_pub = self.create_publisher(
            CmdVelFinal,
            self.cmd_vel_final_topic,
            10,
        )
        self._telemetry_pub = self.create_publisher(NavTelemetry, self.telemetry_topic, 10)

        self._set_goal_srv = self.create_service(
            SetNavGoalLL,
            self.set_goal_service,
            self._on_set_goal_ll,
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
        self._set_manual_srv = self.create_service(
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

        self._manual_watchdog_timer = self.create_timer(
            1.0 / self.manual_watchdog_hz, self._manual_watchdog_tick
        )
        self._telemetry_timer = self.create_timer(
            1.0 / self.nav_telemetry_hz, self._publish_telemetry
        )

        self.get_logger().info(
            "nav_command_server ready "
            f"(goal_srv={self.set_goal_service}, cancel_srv={self.cancel_goal_service}, "
            f"manual_srv={self.set_manual_mode_service}, telemetry={self.telemetry_topic}, "
            f"compute_route_action={self.compute_and_track_route_action}, "
            f"follow_path_action={self.follow_path_action}, route_graph_srv={self.set_route_graph_service}, "
            f"fromll_retries={self.fromll_call_retries}, fromll_timeout_s={self.fromll_call_timeout_s:.2f})"
        )

    def _wait_for_future(self, future: Any, timeout_s: float) -> Optional[Any]:
        started = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - started) >= timeout_s:
                return None
            time.sleep(0.01)
        return None

    def _invalidate_active_fromll(self) -> None:
        self._active_fromll_client = None
        self._active_fromll_service_name = None

    def _resolve_fromll_client(
        self,
        *,
        prefer_fallback: bool = False,
    ) -> Optional[Tuple[Any, str]]:
        candidates: List[Tuple[Any, str, float]] = []

        if (
            (not prefer_fallback)
            and self._active_fromll_client is not None
            and self._active_fromll_service_name
        ):
            candidates.append(
                (self._active_fromll_client, self._active_fromll_service_name, 0.05)
            )

        primary_timeout = min(0.5, self.fromll_wait_timeout_s)
        primary_client = self._fromll_client
        primary_name = self.fromll_service
        fallback_timeout = min(0.5, self.fromll_wait_timeout_s)
        fallback_client = self._fromll_fallback_client
        fallback_name = self.fromll_service_fallback

        if prefer_fallback and fallback_client is not None:
            candidates.append((fallback_client, fallback_name, fallback_timeout))
            candidates.append((primary_client, primary_name, primary_timeout))
        else:
            candidates.append((primary_client, primary_name, primary_timeout))
            if fallback_client is not None:
                candidates.append((fallback_client, fallback_name, fallback_timeout))

        # Last chance: active endpoint even if fallback is preferred.
        if (
            prefer_fallback
            and self._active_fromll_client is not None
            and self._active_fromll_service_name
        ):
            candidates.append(
                (self._active_fromll_client, self._active_fromll_service_name, 0.05)
            )

        seen = set()
        for client, service_name, timeout_s in candidates:
            key = (id(client), service_name)
            if key in seen:
                continue
            seen.add(key)
            if client.wait_for_service(timeout_sec=timeout_s):
                if self._active_fromll_service_name != service_name:
                    self.get_logger().info(f"Using fromLL service: {service_name}")
                self._active_fromll_client = client
                self._active_fromll_service_name = service_name
                return client, service_name

        self._invalidate_active_fromll()
        return None

    def _from_ll_to_map(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        last_error = "fromLL conversion failed"

        for attempt in range(self.fromll_call_retries):
            prefer_fallback = (
                (attempt % 2 == 1) and (self._fromll_fallback_client is not None)
            )
            resolved = self._resolve_fromll_client(prefer_fallback=prefer_fallback)
            if resolved is None:
                last_error = (
                    "fromLL service unavailable "
                    f"(tried '{self.fromll_service}'"
                    + (
                        f" and '{self.fromll_service_fallback}'"
                        if self._fromll_fallback_client is not None
                        else ""
                    )
                    + ")"
                )
            else:
                client, service_name = resolved
                req = FromLL.Request()
                req.ll_point = GeoPoint(
                    latitude=float(lat),
                    longitude=float(lon),
                    altitude=0.0,
                )
                future = client.call_async(req)
                try:
                    response = self._wait_for_future(future, self.fromll_call_timeout_s)
                except Exception as exc:
                    last_error = (
                        f"fromLL request failed on '{service_name}' "
                        f"(attempt {attempt + 1}/{self.fromll_call_retries}): {exc}"
                    )
                    self._invalidate_active_fromll()
                else:
                    if response is None:
                        last_error = (
                            f"timeout waiting fromLL response on '{service_name}' "
                            f"(attempt {attempt + 1}/{self.fromll_call_retries}, "
                            f"timeout_s={self.fromll_call_timeout_s:.2f})"
                        )
                        self._invalidate_active_fromll()
                    else:
                        if attempt > 0:
                            self.get_logger().info(
                                "fromLL conversion recovered "
                                f"after {attempt + 1} attempts using '{service_name}'"
                            )
                        self._last_fromll_error = None
                        return float(response.map_point.x), float(response.map_point.y)

            if (attempt + 1) < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                time.sleep(self.fromll_retry_delay_s)

        self._last_fromll_error = last_error
        self.get_logger().warning(
            "fromLL conversion failed "
            f"(lat={lat:.8f}, lon={lon:.8f}, attempts={self.fromll_call_retries}): "
            f"{last_error}"
        )
        return None

    def _publish_cmd_final(self, linear_x: float, angular_z: float, brake_pct: int) -> None:
        msg = CmdVelFinal()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        msg.brake_pct = max(0, min(100, int(brake_pct)))
        self._cmd_vel_final_pub.publish(msg)

    def _publish_brake_sequence(self, reason: str) -> None:
        self.get_logger().warning(f"Applying brake sequence ({reason})")

        def _run() -> None:
            for idx in range(self.brake_publish_count):
                self._publish_cmd_final(0.0, 0.0, self.brake_pct_on_stop)
                if idx + 1 < self.brake_publish_count:
                    time.sleep(self.brake_publish_interval_s)

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def _cancel_goal_handle(self, handle: Any, label: str) -> None:
        if handle is None:
            return
        try:
            future = handle.cancel_goal_async()
            self._wait_for_future(future, 0.6)
        except Exception as exc:
            self.get_logger().warning(f"cancel {label} failed: {exc}")

    def _cancel_navigation_actions(self, reason: str) -> None:
        with self._lock:
            compute_handle = self._compute_goal_handle
            follow_handle = self._follow_goal_handle
            self._compute_goal_handle = None
            self._follow_goal_handle = None
            self._follow_path_started = False
            self._segment_started_monotonic = None
            self._active_leg_seq = 0
            self._active_leg_closed = False
            self._active_segment = None
            self._last_path_signature = None
        if compute_handle is not None:
            self._cancel_goal_handle(compute_handle, "compute_and_track_route")
        if follow_handle is not None:
            self._cancel_goal_handle(follow_handle, "follow_path")
        if self.context.ok():
            self.get_logger().info(f"Navigation actions canceled ({reason})")

    def _cancel_active_mission(self, reason: str, brake_if_auto: bool) -> None:
        with self._lock:
            was_goal_active = self._goal_active
            self._goal_active = False
            self._loop_enabled = False
            self._initial_segments = []
            self._loop_segments = []
            self._initial_cursor = 0
            self._loop_cursor = 0
            self._active_leg_seq = 0
            self._active_leg_closed = False
            self._active_segment = None
            self._mission_token += 1
            manual_enabled = self._manual_enabled
            if not manual_enabled:
                self._mode = MODE_AUTO_P2P

        self._cancel_navigation_actions(reason)

        if brake_if_auto and not manual_enabled:
            self._publish_brake_sequence(reason)

        if was_goal_active and self.context.ok():
            self.get_logger().info(f"Mission canceled ({reason})")

    def _set_manual_enabled(self, enabled: bool, reason: str) -> None:
        if enabled:
            with self._lock:
                already_manual = self._manual_enabled
                self._manual_enabled = True
                self._mode = MODE_MANUAL
            if not already_manual:
                self.get_logger().info(f"Manual mode enabled ({reason})")
        else:
            with self._lock:
                was_manual = self._manual_enabled
                self._manual_enabled = False
                self._manual_cmd_last_monotonic = None
                self._manual_watchdog_last_brake = None
                if self._goal_active and self._loop_enabled:
                    self._mode = MODE_AUTO_LOOP
                else:
                    self._mode = MODE_AUTO_P2P
            if was_manual:
                self.get_logger().info(f"Manual mode disabled ({reason})")

    def _activate_manual_takeover(self, reason: str) -> None:
        self._set_manual_enabled(True, reason)
        self._cancel_active_mission(reason=f"manual takeover: {reason}", brake_if_auto=False)

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        with self._lock:
            self._robot_lat = float(msg.latitude)
            self._robot_lon = float(msg.longitude)

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_vel_available = True
            self._cmd_vel_linear_x = float(msg.linear.x)
            self._cmd_vel_angular_z = float(msg.angular.z)
            manual_enabled = self._manual_enabled
            collision_stop = self._collision_stop_active

        if manual_enabled:
            return

        if collision_stop:
            now = time.monotonic()
            do_brake = False
            with self._lock:
                if (
                    self._last_collision_brake_monotonic is None
                    or (now - self._last_collision_brake_monotonic) > 0.2
                ):
                    self._last_collision_brake_monotonic = now
                    do_brake = True
            if do_brake:
                self._publish_cmd_final(0.0, 0.0, self.brake_pct_on_stop)
            return

        self._publish_cmd_final(float(msg.linear.x), float(msg.angular.z), 0)

    def _on_teleop_cmd(self, msg: CmdVelFinal) -> None:
        with self._lock:
            manual_enabled = self._manual_enabled

        if not manual_enabled:
            self._activate_manual_takeover("teleop topic")

        now = time.monotonic()
        linear_x = float(msg.twist.linear.x)
        angular_z = float(msg.twist.angular.z)
        brake_pct = max(0, min(100, int(msg.brake_pct)))

        with self._lock:
            self._manual_linear_x_cmd = linear_x
            self._manual_angular_z_cmd = angular_z
            self._manual_cmd_last_monotonic = now
            self._manual_watchdog_last_brake = None

        self._publish_cmd_final(linear_x, angular_z, brake_pct)

    def _on_collision_state(self, msg: CollisionMonitorState) -> None:
        is_stop = int(msg.action_type) == int(CollisionMonitorState.STOP)

        with self._lock:
            prev_stop = self._collision_stop_active
            self._collision_stop_active = bool(is_stop)
            manual_enabled = self._manual_enabled
            mode = self._mode

        if is_stop and (not prev_stop) and (not manual_enabled):
            self.get_logger().warning(
                f"Collision monitor STOP ({msg.polygon_name}) while mode={mode}; braking"
            )
            self._publish_brake_sequence("collision monitor STOP")
        elif (not is_stop) and prev_stop:
            self.get_logger().info("Collision monitor cleared STOP")

    def _extract_waypoints(
        self, request: SetNavGoalLL.Request
    ) -> Tuple[Optional[List[Tuple[float, float, float]]], str]:
        lats = [float(v) for v in request.lats]
        lons = [float(v) for v in request.lons]
        yaws = [float(v) for v in request.yaws_deg]

        use_array = len(lats) > 0 or len(lons) > 0 or len(yaws) > 0
        if use_array:
            if len(lats) == 0 or len(lons) == 0:
                return None, "lats and lons must be provided together"
            if len(lats) != len(lons):
                return None, "lats and lons size mismatch"
            if len(yaws) not in (0, len(lats)):
                return None, "yaws_deg must be empty or match lats/lons size"
            if len(yaws) == 0:
                yaws = [0.0] * len(lats)
            waypoints = list(zip(lats, lons, yaws))
        else:
            waypoints = [
                (
                    float(request.lat),
                    float(request.lon),
                    float(request.yaw_deg),
                )
            ]

        for lat, lon, yaw_deg in waypoints:
            if (not math.isfinite(lat)) or (not math.isfinite(lon)):
                return None, "invalid waypoint coordinates"
            if not math.isfinite(yaw_deg):
                return None, "invalid yaw value"

        return waypoints, ""

    def _get_current_anchor_point(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            lat = float(self._robot_lat)
            lon = float(self._robot_lon)

        if (not math.isfinite(lat)) or (not math.isfinite(lon)):
            return None

        return self._from_ll_to_map(lat, lon)

    def _build_graph_plan(
        self,
        waypoint_xy: Sequence[Tuple[float, float]],
        loop: bool,
    ) -> Tuple[Optional[Dict[str, Any]], str]:
        points: List[Tuple[float, float]] = []
        anchor = self._get_current_anchor_point()
        anchor_added = False
        if anchor is not None:
            points.append(anchor)
            anchor_added = True

        points.extend(waypoint_xy)

        if len(points) < 2:
            return (
                None,
                "unable to build route graph: need GPS fix for single-waypoint mission",
            )

        first_wp_id = 2 if anchor_added else 1
        last_wp_id = len(points)

        graph_edges: List[Tuple[int, int]] = []

        for node_id in range(1, len(points)):
            edge = (node_id, node_id + 1)
            graph_edges.append(edge)

        loop_effective = bool(loop and len(waypoint_xy) > 1)
        if loop_effective:
            closing_edge = (last_wp_id, first_wp_id)
            graph_edges.append(closing_edge)

        features: List[Dict[str, Any]] = []
        for idx, (x, y) in enumerate(points, start=1):
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [float(x), float(y)]},
                    "properties": {"id": int(idx)},
                }
            )

        edge_id = 1
        for start_id, end_id in graph_edges:
            start_x, start_y = points[start_id - 1]
            end_x, end_y = points[end_id - 1]
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [float(start_x), float(start_y)],
                            [float(end_x), float(end_y)],
                        ],
                    },
                    "properties": {
                        "id": int(edge_id),
                        "startid": int(start_id),
                        "endid": int(end_id),
                    },
                }
            )
            edge_id += 1

        return (
            {
                "graph": {"type": "FeatureCollection", "features": features},
                "loop_effective": loop_effective,
                "start_node_id": 1 if anchor_added else first_wp_id,
                "first_wp_id": first_wp_id,
                "last_wp_id": last_wp_id,
                "anchor_added": anchor_added,
            },
            "",
        )

    def _build_mission_legs(
        self, plan: Dict[str, Any]
    ) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        start_node_id = int(plan["start_node_id"])
        first_wp_id = int(plan["first_wp_id"])
        last_wp_id = int(plan["last_wp_id"])
        loop_effective = bool(plan["loop_effective"])

        if loop_effective:
            loop_cycle_legs = [
                (first_wp_id, last_wp_id),
                (last_wp_id, first_wp_id),
            ]
            initial_legs: List[Tuple[int, int]] = []
            if start_node_id != first_wp_id:
                initial_legs.append((start_node_id, first_wp_id))
            initial_legs.extend(loop_cycle_legs)
            return initial_legs, loop_cycle_legs

        initial_legs = [(start_node_id, last_wp_id)]
        return initial_legs, []

    def _install_route_graph(self, graph_payload: Dict[str, Any]) -> Tuple[bool, str]:
        try:
            self.route_graph_temp_filepath.parent.mkdir(parents=True, exist_ok=True)
            self.route_graph_temp_filepath.write_text(
                json.dumps(graph_payload, ensure_ascii=True),
                encoding="utf-8",
            )
        except Exception as exc:
            return False, f"failed writing route graph file: {exc}"

        if not self._set_route_graph_client.wait_for_service(timeout_sec=2.0):
            return False, "set_route_graph service unavailable"

        req = SetRouteGraph.Request()
        req.graph_filepath = str(self.route_graph_temp_filepath)
        future = self._set_route_graph_client.call_async(req)
        try:
            response = self._wait_for_future(future, 5.0)
        except Exception as exc:
            return False, f"set_route_graph call failed: {exc}"

        if response is None:
            return False, "set_route_graph timeout"
        if not bool(response.success):
            return False, "set_route_graph failed"
        return True, ""

    def _consume_next_segment_locked(self) -> Optional[Tuple[int, int]]:
        if self._initial_cursor < len(self._initial_segments):
            segment = self._initial_segments[self._initial_cursor]
            self._initial_cursor += 1
            return segment

        if self._loop_enabled and len(self._loop_segments) > 0:
            segment = self._loop_segments[self._loop_cursor % len(self._loop_segments)]
            self._loop_cursor += 1
            return segment

        return None

    def _dispatch_next_segment(self, mission_token: int) -> Tuple[bool, str]:
        with self._lock:
            if mission_token != self._mission_token:
                return False, "stale mission token"
            if (not self._goal_active) or self._manual_enabled:
                return False, "mission inactive"
            segment = self._consume_next_segment_locked()
            if segment is None:
                self._goal_active = False
                self._segment_started_monotonic = None
                self._follow_path_started = False
                self._active_leg_seq = 0
                self._active_leg_closed = False
                self._active_segment = None
                self._last_path_signature = None
                if self._loop_enabled:
                    self._mode = MODE_AUTO_LOOP
                else:
                    self._mode = MODE_AUTO_P2P
                self.get_logger().info("Mission completed")
                return True, ""
            self._active_leg_seq += 1
            leg_seq = int(self._active_leg_seq)
            self._active_leg_closed = False
            self._active_segment = segment
            self._segment_started_monotonic = time.monotonic()
            self._follow_path_started = False
            self._last_path_signature = None

        if not self._compute_route_client.wait_for_server(timeout_sec=2.0):
            return False, "compute_and_track_route action unavailable"

        goal = ComputeAndTrackRoute.Goal()
        goal.start_id = int(segment[0])
        goal.goal_id = int(segment[1])
        goal.use_start = False
        goal.use_poses = False

        send_future = self._compute_route_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._on_compute_feedback,
                mission_token=mission_token,
                leg_seq=leg_seq,
            ),
        )
        send_future.add_done_callback(
            partial(
                self._on_compute_goal_response,
                mission_token=mission_token,
                leg_seq=leg_seq,
                segment=segment,
            )
        )

        return True, ""

    def _on_compute_goal_response(
        self,
        future: Any,
        mission_token: int,
        leg_seq: int,
        segment: Tuple[int, int],
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_active_mission(
                mission_token,
                f"compute_and_track_route send failed: {exc}",
                brake=True,
            )
            return

        with self._lock:
            stale_goal = (
                mission_token != self._mission_token
                or (not self._goal_active)
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
            )
        if stale_goal:
            if goal_handle.accepted:
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, 0.4)
                except Exception:
                    pass
            return

        if not goal_handle.accepted:
            self._fail_active_mission(
                mission_token,
                "compute_and_track_route goal rejected",
                brake=True,
            )
            return

        with self._lock:
            if (
                mission_token != self._mission_token
                or (not self._goal_active)
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
            ):
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, 0.4)
                except Exception:
                    pass
                return
            self._compute_goal_handle = goal_handle

        self.get_logger().info(
            f"compute_and_track_route accepted segment {segment[0]}->{segment[1]}"
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._on_compute_result,
                mission_token=mission_token,
                leg_seq=leg_seq,
                segment=segment,
            )
        )

    def _on_compute_feedback(self, feedback_msg: Any, mission_token: int, leg_seq: int) -> None:
        fb = feedback_msg.feedback
        path = fb.path
        if len(path.poses) == 0:
            return

        try:
            first_pose = path.poses[0].pose.position
            last_pose = path.poses[-1].pose.position
            signature = (
                int(len(path.poses)),
                int(round(first_pose.x * 100.0)),
                int(round(first_pose.y * 100.0)),
                int(round(last_pose.x * 100.0)),
                int(round(last_pose.y * 100.0)),
            )
        except Exception:
            signature = (len(path.poses), 0, 0, 0, 0)

        send_follow = False
        with self._lock:
            if (
                mission_token != self._mission_token
                or (not self._goal_active)
                or self._manual_enabled
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
            ):
                return

            rerouted = bool(getattr(fb, "rerouted", False))
            if (
                (not self._follow_path_started)
                or rerouted
                or (signature != self._last_path_signature)
            ):
                self._follow_path_started = True
                self._last_path_signature = signature
                send_follow = True

        if send_follow:
            self._send_follow_path_goal(path, mission_token, leg_seq)

    def _send_follow_path_goal(self, path: Any, mission_token: int, leg_seq: int) -> None:
        if not self._follow_path_client.wait_for_server(timeout_sec=1.5):
            self._fail_active_mission(
                mission_token,
                "follow_path action unavailable",
                brake=True,
            )
            return

        with self._lock:
            if (
                mission_token != self._mission_token
                or (not self._goal_active)
                or self._manual_enabled
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
            ):
                return
            old_handle = self._follow_goal_handle
            self._follow_goal_handle = None
            self._follow_goal_seq += 1
            follow_goal_seq = int(self._follow_goal_seq)

        if old_handle is not None:
            self._cancel_goal_handle(old_handle, "follow_path")

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.follow_path_controller_id
        goal.goal_checker_id = self.follow_path_goal_checker_id

        send_future = self._follow_path_client.send_goal_async(goal)
        send_future.add_done_callback(
            partial(
                self._on_follow_goal_response,
                mission_token=mission_token,
                leg_seq=leg_seq,
                follow_goal_seq=follow_goal_seq,
            )
        )

    def _on_follow_goal_response(
        self,
        future: Any,
        mission_token: int,
        leg_seq: int,
        follow_goal_seq: int,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_active_mission(
                mission_token,
                f"follow_path send failed: {exc}",
                brake=True,
            )
            return

        with self._lock:
            if (
                mission_token != self._mission_token
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
                or follow_goal_seq != self._follow_goal_seq
            ):
                if goal_handle.accepted:
                    try:
                        cancel_future = goal_handle.cancel_goal_async()
                        self._wait_for_future(cancel_future, 0.4)
                    except Exception:
                        pass
                return

        if not goal_handle.accepted:
            self._fail_active_mission(
                mission_token,
                "follow_path goal rejected",
                brake=True,
            )
            return

        with self._lock:
            if (
                mission_token != self._mission_token
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
                or follow_goal_seq != self._follow_goal_seq
            ):
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, 0.4)
                except Exception:
                    pass
                return
            self._follow_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._on_follow_result,
                mission_token=mission_token,
                leg_seq=leg_seq,
                follow_goal_seq=follow_goal_seq,
            )
        )

    def _on_follow_result(
        self,
        future: Any,
        mission_token: int,
        leg_seq: int,
        follow_goal_seq: int,
    ) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._fail_active_mission(
                mission_token,
                f"follow_path result failed: {exc}",
                brake=True,
            )
            return

        status = int(wrapped_result.status)

        compute_handle = None
        segment: Optional[Tuple[int, int]] = None
        with self._lock:
            if (
                mission_token != self._mission_token
                or leg_seq != self._active_leg_seq
                or self._active_leg_closed
                or follow_goal_seq != self._follow_goal_seq
            ):
                return
            manual_enabled = self._manual_enabled
            goal_active = self._goal_active
            if status == GoalStatus.STATUS_SUCCEEDED and (not manual_enabled) and goal_active:
                self._active_leg_closed = True
                segment = self._active_segment
                compute_handle = self._compute_goal_handle
                self._compute_goal_handle = None
                self._follow_goal_handle = None
                self._follow_path_started = False
                self._segment_started_monotonic = None
                self._last_path_signature = None

        if manual_enabled or (not goal_active):
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if compute_handle is not None:
                self._cancel_goal_handle(compute_handle, "compute_and_track_route")
            if segment is not None:
                self.get_logger().info(
                    f"Segment completed {segment[0]}->{segment[1]}"
                )
            ok, err = self._dispatch_next_segment(mission_token)
            if not ok and err not in ("mission inactive", "stale mission token"):
                self._fail_active_mission(
                    mission_token,
                    f"failed dispatching next segment: {err}",
                    brake=True,
                )
            return

        if status == GoalStatus.STATUS_CANCELED:
            return

        self._fail_active_mission(
            mission_token,
            f"follow_path ended with status={status}",
            brake=True,
        )

    def _on_compute_result(
        self,
        future: Any,
        mission_token: int,
        leg_seq: int,
        segment: Tuple[int, int],
    ) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._fail_active_mission(
                mission_token,
                f"compute_and_track_route result failed: {exc}",
                brake=True,
            )
            return

        status = int(wrapped_result.status)

        with self._lock:
            if mission_token != self._mission_token:
                return
            if leg_seq != self._active_leg_seq or self._active_leg_closed:
                return
            self._compute_goal_handle = None
            manual_enabled = self._manual_enabled
            goal_active = self._goal_active

        if manual_enabled or (not goal_active):
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_active_mission(
                mission_token,
                f"compute_and_track_route status={status} for segment {segment[0]}->{segment[1]}",
                brake=True,
            )
            return

    def _fail_active_mission(
        self,
        mission_token: int,
        error: str,
        brake: bool,
    ) -> None:
        with self._lock:
            if mission_token != self._mission_token:
                return

        self.get_logger().warning(f"Mission failure: {error}")
        self._cancel_active_mission(reason=error, brake_if_auto=brake)

    def _on_set_goal_ll(
        self,
        request: SetNavGoalLL.Request,
        response: SetNavGoalLL.Response,
    ) -> SetNavGoalLL.Response:
        waypoints_ll, err = self._extract_waypoints(request)
        if waypoints_ll is None:
            response.ok = False
            response.error = err
            return response

        self._set_manual_enabled(False, "set_goal_ll")
        self._cancel_active_mission(reason="new set_goal_ll request", brake_if_auto=False)

        waypoint_xy: List[Tuple[float, float]] = []
        for lat, lon, _ in waypoints_ll:
            xy = self._from_ll_to_map(lat, lon)
            if xy is None:
                detail = self._last_fromll_error or "fromLL conversion failed"
                response.ok = False
                response.error = (
                    "failed converting waypoint "
                    f"lat={lat:.8f}, lon={lon:.8f} with fromLL: {detail}"
                )
                return response
            waypoint_xy.append(xy)

        plan, err = self._build_graph_plan(waypoint_xy=waypoint_xy, loop=bool(request.loop))
        if plan is None:
            response.ok = False
            response.error = err
            return response

        ok_graph, err_graph = self._install_route_graph(plan["graph"])
        if not ok_graph:
            response.ok = False
            response.error = err_graph
            return response

        initial_segments, loop_segments = self._build_mission_legs(plan)
        if len(initial_segments) == 0:
            response.ok = False
            response.error = "failed to build mission legs"
            return response

        with self._lock:
            self._mission_token += 1
            mission_token = self._mission_token
            self._goal_active = True
            self._loop_enabled = bool(plan["loop_effective"])
            self._initial_segments = list(initial_segments)
            self._loop_segments = list(loop_segments)
            self._initial_cursor = 0
            self._loop_cursor = 0
            self._segment_started_monotonic = None
            self._follow_path_started = False
            self._active_leg_seq = 0
            self._active_leg_closed = False
            self._active_segment = None
            self._last_path_signature = None
            self._mode = MODE_AUTO_LOOP if self._loop_enabled else MODE_AUTO_P2P

        ok_dispatch, err_dispatch = self._dispatch_next_segment(mission_token)
        if not ok_dispatch:
            self._cancel_active_mission(
                reason=f"failed to start mission: {err_dispatch}",
                brake_if_auto=True,
            )
            response.ok = False
            response.error = err_dispatch
            return response

        response.ok = True
        response.error = ""
        self.get_logger().info(
            "set_goal_ll accepted "
            f"(waypoints={len(waypoints_ll)}, loop={bool(plan['loop_effective'])}, "
            f"start_node_id={int(plan['start_node_id'])}, first_wp_id={int(plan['first_wp_id'])}, "
            f"last_wp_id={int(plan['last_wp_id'])}, mission_legs={len(initial_segments)}, "
            f"loop_cycle_legs={len(loop_segments)})"
        )
        return response

    def _on_cancel_goal(
        self,
        _request: CancelNavGoal.Request,
        response: CancelNavGoal.Response,
    ) -> CancelNavGoal.Response:
        with self._lock:
            manual_enabled = self._manual_enabled

        self._cancel_active_mission(reason="cancel_goal service", brake_if_auto=(not manual_enabled))
        response.ok = True
        response.error = ""
        return response

    def _on_brake(
        self,
        _request: BrakeNav.Request,
        response: BrakeNav.Response,
    ) -> BrakeNav.Response:
        self._cancel_active_mission(reason="brake service", brake_if_auto=False)
        self._publish_brake_sequence("brake service")
        response.ok = True
        response.error = ""
        return response

    def _on_set_manual_mode(
        self,
        request: SetManualMode.Request,
        response: SetManualMode.Response,
    ) -> SetManualMode.Response:
        enabled = bool(request.enabled)
        if enabled:
            self._set_manual_enabled(True, "set_manual_mode service")
            self._cancel_active_mission(
                reason="manual mode service takeover",
                brake_if_auto=False,
            )
            self._publish_brake_sequence("manual mode enable")
        else:
            self._set_manual_enabled(False, "set_manual_mode service")

        with self._lock:
            enabled_after = self._manual_enabled

        response.ok = True
        response.error = ""
        response.enabled_after = bool(enabled_after)
        return response

    def _on_get_state(
        self,
        _request: GetNavState.Request,
        response: GetNavState.Response,
    ) -> GetNavState.Response:
        with self._lock:
            response.ok = True
            response.error = ""
            response.goal_active = bool(self._goal_active)
            response.manual_enabled = bool(self._manual_enabled)
            response.manual_linear_x_cmd = float(self._manual_linear_x_cmd)
            response.manual_angular_z_cmd = float(self._manual_angular_z_cmd)
            response.cmd_vel_available = bool(self._cmd_vel_available)
            response.cmd_vel_linear_x = float(self._cmd_vel_linear_x)
            response.cmd_vel_angular_z = float(self._cmd_vel_angular_z)
            response.robot_lat = float(self._robot_lat)
            response.robot_lon = float(self._robot_lon)
        return response

    def _manual_watchdog_tick(self) -> None:
        now = time.monotonic()

        should_brake = False
        mission_token = 0
        route_elapsed = 0.0
        route_timeout_should_fail = False
        with self._lock:
            manual_enabled = self._manual_enabled
            last_cmd = self._manual_cmd_last_monotonic
            if manual_enabled and (last_cmd is not None):
                age = now - last_cmd
                if age > self.manual_cmd_timeout_s and (
                    self._manual_watchdog_last_brake is None
                    or (now - self._manual_watchdog_last_brake)
                    >= self.manual_cmd_timeout_s
                ):
                    self._manual_watchdog_last_brake = now
                    should_brake = True
                    self._manual_linear_x_cmd = 0.0
                    self._manual_angular_z_cmd = 0.0

            if (
                self._goal_active
                and (not self._manual_enabled)
                and self._segment_started_monotonic is not None
                and (not self._follow_path_started)
            ):
                route_elapsed = now - self._segment_started_monotonic
                mission_token = self._mission_token
                route_timeout_should_fail = route_elapsed > self.route_first_path_timeout_s

        if should_brake:
            self.get_logger().warning(
                f"Manual watchdog timeout ({self.manual_cmd_timeout_s:.2f}s), braking"
            )
            self._publish_cmd_final(0.0, 0.0, self.brake_pct_on_stop)

        if route_timeout_should_fail:
            self._fail_active_mission(
                mission_token,
                (
                    "timeout waiting first path feedback from "
                    f"compute_and_track_route ({route_elapsed:.2f}s > "
                    f"{self.route_first_path_timeout_s:.2f}s)"
                ),
                brake=True,
            )

    def _publish_telemetry(self) -> None:
        msg = NavTelemetry()
        with self._lock:
            msg.goal_active = bool(self._goal_active)
            msg.manual_enabled = bool(self._manual_enabled)
            msg.manual_linear_x_cmd = float(self._manual_linear_x_cmd)
            msg.manual_angular_z_cmd = float(self._manual_angular_z_cmd)
            msg.cmd_vel_available = bool(self._cmd_vel_available)
            msg.cmd_vel_linear_x = float(self._cmd_vel_linear_x)
            msg.cmd_vel_angular_z = float(self._cmd_vel_angular_z)
            msg.robot_lat = float(self._robot_lat)
            msg.robot_lon = float(self._robot_lon)
        self._telemetry_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._cancel_active_mission(reason="node shutdown", brake_if_auto=False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCommandServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
