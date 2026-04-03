import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.msg import CollisionMonitorState

from interfaces.msg import CmdVelFinal
from interfaces.srv import SetNavGoalLL
from navegacion_gps.nav_command_server import NavCommandServerNode


class _FakeArbNode:
    _build_cmd_vel_final = staticmethod(NavCommandServerNode._build_cmd_vel_final)
    _clear_final_stop_latch_locked = NavCommandServerNode._clear_final_stop_latch_locked

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._manual_enabled = False
        self._is_navigating = False
        self._auto_mode = "idle"
        self._current_goal_handle = None
        self._last_cmd_vel_safe = None
        self._collision_stop_active = False
        self.forward_cmd_vel_safe_without_goal = False
        self._last_manual_cmd = CmdVelFinal()
        self._last_manual_cmd_time = None
        self._manual_watchdog_stop_sent = False
        self.manual_cmd_timeout_s = 0.1
        self.brake_publish_count = 1
        self.brake_publish_interval_s = 0.0
        self.auto_stop_hold_s = 0.75
        self._auto_stop_hold_until_s = 0.0
        self._final_stop_latched = False
        self._final_stop_brake_pct = 100

        self.published = []
        self.telemetry_forced = []
        self.cancel_calls = 0

    def _publish_cmd_vel_final(self, msg: CmdVelFinal) -> None:
        self.published.append(
            (
                float(msg.twist.linear.x),
                float(msg.twist.angular.z),
                int(msg.brake_pct),
            )
        )

    def _publish_stop(self, brake_pct: int) -> None:
        self._publish_cmd_vel_final(
            NavCommandServerNode._build_cmd_vel_final(0.0, 0.0, int(brake_pct))
        )

    def _publish_manual_cmd(self, linear_x: float, angular_z: float, brake_pct: int) -> None:
        self._publish_cmd_vel_final(
            NavCommandServerNode._build_cmd_vel_final(linear_x, angular_z, brake_pct)
        )

    def _publish_manual_stop(self) -> None:
        self._publish_manual_cmd(0.0, 0.0, 0)

    def _publish_telemetry(self, force: bool = False) -> None:
        self.telemetry_forced.append(bool(force))

    def cancel_current_goal(self):
        self.cancel_calls += 1
        return False, "timeout cancelling goal"

    def _cancel_goal_for_manual_takeover_async(self) -> None:
        self.cancel_current_goal()

    def get_logger(self):
        class _Logger:
            def warning(self, _msg: str) -> None:
                pass

        return _Logger()


def test_on_cmd_vel_safe_ignores_auto_while_manual() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._is_navigating = True

    msg = Twist()
    msg.linear.x = 1.2
    msg.angular.z = 0.3
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == []


def test_on_cmd_vel_safe_publishes_auto_when_navigating() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._collision_stop_active = False

    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = -0.2
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.8, -0.2, 0)]


def test_on_cmd_vel_safe_ignores_auto_during_stop_hold() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._auto_stop_hold_until_s = time.monotonic() + 1.0

    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = -0.2
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == []


def test_on_cmd_vel_safe_keeps_braking_while_final_stop_latched() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = False
    node.forward_cmd_vel_safe_without_goal = True
    node._final_stop_latched = True
    node._final_stop_brake_pct = 100

    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = -0.2
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.0, 0.0, 100)]


def test_on_collision_monitor_state_stop_ignored_in_manual() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._is_navigating = True

    msg = CollisionMonitorState()
    msg.action_type = CollisionMonitorState.STOP
    NavCommandServerNode._on_collision_monitor_state(node, msg)

    assert node.published == []


def test_manual_watchdog_sends_single_stop() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._last_manual_cmd_time = time.monotonic() - 1.0
    node._manual_watchdog_stop_sent = False

    NavCommandServerNode._manual_watchdog_tick(node)
    NavCommandServerNode._manual_watchdog_tick(node)

    assert node.published == [(0.0, 0.0, 0)]


def test_auto_final_stop_latch_republishes_brake_on_watchdog() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._final_stop_latched = True
    node._final_stop_brake_pct = 100

    NavCommandServerNode._manual_watchdog_tick(node)
    NavCommandServerNode._manual_watchdog_tick(node)

    assert node.published == [(0.0, 0.0, 100), (0.0, 0.0, 100)]


def test_set_manual_mode_enables_even_if_cancel_fails() -> None:
    node = _FakeArbNode()
    node._current_goal_handle = object()
    ok, _err, enabled_after = NavCommandServerNode.set_manual_mode(node, True)

    assert ok is True
    assert enabled_after is True
    assert node._manual_enabled is True
    assert node._is_navigating is False
    assert node.cancel_calls == 1


class _FakeSetGoalNode(_FakeArbNode):
    _parse_set_goal_request = NavCommandServerNode._parse_set_goal_request

    def __init__(self) -> None:
        super().__init__()
        self.logged = []
        self.sent = []

    def set_manual_mode(self, enabled: bool):
        self._manual_enabled = bool(enabled)
        return True, "ok", bool(enabled)

    def send_nav2_goals(self, waypoints, loop_enabled: bool):
        self.sent.append((list(waypoints), bool(loop_enabled)))
        return True, "goal accepted"

    def get_logger(self):
        class _Logger:
            def __init__(self, sink):
                self._sink = sink

            def info(self, msg: str) -> None:
                self._sink.append(("info", msg))

            def warning(self, msg: str) -> None:
                self._sink.append(("warning", msg))

        return _Logger(self.logged)


def test_set_goal_auto_disables_manual_before_forwarding_goal() -> None:
    node = _FakeSetGoalNode()
    node._manual_enabled = True
    request = SetNavGoalLL.Request()
    request.lats = [-31.0, -31.1]
    request.lons = [-64.0, -64.1]
    request.yaws_deg = [0.0, 30.0]
    request.loop = True
    response = SimpleNamespace(ok=False, error="")

    out = NavCommandServerNode._on_set_goal(node, request, response)

    assert out.ok is True
    assert out.error == ""
    assert node._manual_enabled is False
    assert node.sent == [([(-31.0, -64.0, 0.0), (-31.1, -64.1, 30.0)], True)]


class _FakeGoalPoseNode(_FakeArbNode):
    _on_goal_pose = NavCommandServerNode._on_goal_pose

    def __init__(self) -> None:
        super().__init__()
        self.map_frame = "map"
        self.logged = []
        self.pose_calls = []

    def set_manual_mode(self, enabled: bool):
        self._manual_enabled = bool(enabled)
        return True, "ok", bool(enabled)

    def _send_goal_from_pose(self, pose, reason: str):
        self.pose_calls.append((pose, str(reason)))
        return True, "goal accepted"

    def get_logger(self):
        class _Logger:
            def __init__(self, sink):
                self._sink = sink

            def info(self, msg: str) -> None:
                self._sink.append(("info", msg))

            def warning(self, msg: str) -> None:
                self._sink.append(("warning", msg))

        return _Logger(self.logged)


def test_goal_pose_auto_disables_manual_before_forwarding_goal() -> None:
    node = _FakeGoalPoseNode()
    node._manual_enabled = True
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.0
    pose.pose.position.y = -1.0

    NavCommandServerNode._on_goal_pose(node, pose)

    assert node._manual_enabled is False
    assert len(node.pose_calls) == 1
    assert node.pose_calls[0][1] == "goal_pose_topic"


class _FakeArrivalNode:
    _activate_auto_stop_hold = NavCommandServerNode._activate_auto_stop_hold
    _detach_goal_handle_locked = NavCommandServerNode._detach_goal_handle_locked

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._manual_enabled = False
        self._is_navigating = True
        self._auto_mode = "point_to_point"
        self._current_goal_handle = object()
        self._active_goal_target_ll = (-31.0000000, -64.0000000)
        self._pending_nav_result_override = None
        self._last_robot_pose = None
        self._last_nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
        self._last_nav_result_text = ""
        self._nav_result_event_id = 0
        self.auto_stop_hold_s = 0.75
        self._auto_stop_hold_until_s = 0.0
        self._final_stop_latched = False
        self._final_stop_brake_pct = 100
        self.goal_arrival_radius_m = 0.8
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_restart_poses = []
        self._loop_enabled = False

        self.stop_calls = []
        self.brake_async_calls = []
        self.cancel_async_calls = []
        self.telemetry_forced = []
        self.logged = []

    def _clear_loop_config_locked(self) -> None:
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_restart_poses = []
        self._loop_enabled = False

    def _publish_stop(self, brake_pct: int) -> None:
        self.stop_calls.append(int(brake_pct))

    def _publish_brake_sequence_async(self, brake_pct: int, skip_first: bool = False) -> None:
        self.brake_async_calls.append((int(brake_pct), bool(skip_first)))

    def _cancel_goal_async(
        self,
        handle,
        reason: str,
        update_nav_result: bool = True,
    ) -> None:
        self.cancel_async_calls.append((handle, str(reason), bool(update_nav_result)))

    def _publish_telemetry(self, force: bool = False) -> None:
        self.telemetry_forced.append(bool(force))

    def get_logger(self):
        class _Logger:
            def __init__(self, sink):
                self._sink = sink

            def info(self, msg: str) -> None:
                self._sink.append(("info", msg))

        return _Logger(self.logged)


def test_on_gps_fix_completes_point_goal_within_arrival_radius() -> None:
    node = _FakeArrivalNode()
    msg = SimpleNamespace(latitude=-31.0000000, longitude=-64.0000000)

    NavCommandServerNode._on_gps_fix(node, msg)

    assert node._is_navigating is False
    assert node._current_goal_handle is None
    assert node._active_goal_target_ll is None
    assert node._last_nav_result_status == int(GoalStatus.STATUS_SUCCEEDED)
    assert "goal reached within" in node._last_nav_result_text
    assert node._pending_nav_result_override == (
        int(GoalStatus.STATUS_SUCCEEDED),
        node._last_nav_result_text,
    )
    assert node.stop_calls == [100]
    assert node.brake_async_calls == [(100, True)]
    assert len(node.cancel_async_calls) == 1
    assert node.cancel_async_calls[0][1] == "goal arrival radius reached"
    assert node.cancel_async_calls[0][2] is False
    assert node._final_stop_latched is True
    assert node._final_stop_brake_pct == 100
    assert node._auto_stop_hold_until_s > time.monotonic()


class _FakeArrivalResultNode:
    _on_nav_action_result_done = NavCommandServerNode._on_nav_action_result_done
    _arm_final_stop_latch = NavCommandServerNode._arm_final_stop_latch
    _activate_auto_stop_hold = NavCommandServerNode._activate_auto_stop_hold

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._loop_enabled = False
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_restart_poses = []
        self._manual_enabled = False
        self._is_navigating = False
        self._auto_mode = "idle"
        self._pending_nav_result_override = (
            int(GoalStatus.STATUS_SUCCEEDED),
            "goal reached within 0.20 m of target",
        )
        self._ignored_goal_result_ids = set()
        self.auto_stop_hold_s = 0.75
        self._auto_stop_hold_until_s = 0.0
        self._final_stop_latched = False
        self._final_stop_brake_pct = 100
        self._last_nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
        self._last_nav_result_text = ""
        self._nav_result_event_id = 0
        self.telemetry_forced = []
        self.brake_calls = []
        self.logged = []

    def _clear_loop_config_locked(self) -> None:
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_restart_poses = []
        self._loop_enabled = False

    def _publish_telemetry(self, force: bool = False) -> None:
        self.telemetry_forced.append(bool(force))

    def _publish_brake_sequence(self, brake_pct: int) -> None:
        self.brake_calls.append(int(brake_pct))

    def get_logger(self):
        class _Logger:
            def __init__(self, sink):
                self._sink = sink

            def info(self, msg: str) -> None:
                self._sink.append(("info", msg))

            def warning(self, msg: str) -> None:
                self._sink.append(("warning", msg))

        return _Logger(self.logged)


def test_nav_result_done_keeps_arrival_success_when_cancel_returns() -> None:
    node = _FakeArrivalResultNode()
    handle = object()
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            status=int(GoalStatus.STATUS_CANCELED),
            result=SimpleNamespace(missed_waypoints=[]),
        )
    )

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "FollowWaypoints",
        handle,
        future,
    )

    assert node._pending_nav_result_override is None
    assert node._last_nav_result_status == int(GoalStatus.STATUS_SUCCEEDED)
    assert node._last_nav_result_text == "goal reached within 0.20 m of target"
    assert node.brake_calls == []


def test_nav_result_done_latches_final_stop_on_success() -> None:
    node = _FakeArrivalResultNode()
    node._auto_mode = "point_to_point"
    node._is_navigating = True
    node._pending_nav_result_override = None
    handle = object()
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            status=int(GoalStatus.STATUS_SUCCEEDED),
            result=SimpleNamespace(missed_waypoints=[]),
        )
    )

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "FollowWaypoints",
        handle,
        future,
    )

    assert node._last_nav_result_status == int(GoalStatus.STATUS_SUCCEEDED)
    assert node._final_stop_latched is True
    assert node._final_stop_brake_pct == 100
    assert node.brake_calls == [100]
