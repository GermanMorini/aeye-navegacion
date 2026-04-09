import threading
import time

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState

from interfaces.msg import CmdVelFinal, NavTelemetry
from navegacion_gps.nav_command_server import NavCommandServerNode


class _FakeArbNode:
    _build_cmd_vel_final = staticmethod(NavCommandServerNode._build_cmd_vel_final)

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


def test_set_manual_mode_enables_even_if_cancel_fails() -> None:
    node = _FakeArbNode()
    node._current_goal_handle = object()
    ok, _err, enabled_after = NavCommandServerNode.set_manual_mode(node, True)

    assert ok is True
    assert enabled_after is True
    assert node._manual_enabled is True
    assert node._is_navigating is False
    assert node.cancel_calls == 1


class _FakeTelemetryPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg: NavTelemetry) -> None:
        self.messages.append(msg)


class _FakeTelemetryNode:
    _manual_control_payload_locked = NavCommandServerNode._manual_control_payload_locked
    _cmd_vel_safe_payload_locked = NavCommandServerNode._cmd_vel_safe_payload_locked
    _nav_result_payload_locked = NavCommandServerNode._nav_result_payload_locked
    _gps_pose_payload_locked = NavCommandServerNode._gps_pose_payload_locked
    _publish_telemetry = NavCommandServerNode._publish_telemetry

    def __init__(self, *, gps_age_s: float, gps_timeout_s: float) -> None:
        self._lock = threading.Lock()
        self.nav_telemetry_hz = 100.0
        self.gps_fix_timeout_s = float(gps_timeout_s)
        self._last_telemetry_sent = None
        self._is_navigating = False
        self._manual_enabled = False
        self._last_manual_cmd = CmdVelFinal()
        self._last_cmd_vel_safe = None
        self._last_robot_pose = {"lat": -31.0, "lon": -64.0}
        self._last_gps_fix_monotonic = time.monotonic() - float(gps_age_s)
        self._auto_mode = "idle"
        self._collision_stop_active = False
        self._last_nav_result_status = 0
        self._last_nav_result_text = "idle"
        self._nav_result_event_id = 0
        self._telemetry_pub = _FakeTelemetryPublisher()


def test_publish_telemetry_marks_robot_pose_available_for_fresh_gps_fix() -> None:
    node = _FakeTelemetryNode(gps_age_s=0.2, gps_timeout_s=2.0)

    node._publish_telemetry(force=True)

    assert len(node._telemetry_pub.messages) == 1
    msg = node._telemetry_pub.messages[0]
    assert msg.robot_pose_available is True
    assert msg.gps_fix_available is True
    assert msg.robot_lat == -31.0
    assert msg.robot_lon == -64.0


def test_publish_telemetry_clears_robot_pose_for_stale_gps_fix() -> None:
    node = _FakeTelemetryNode(gps_age_s=5.0, gps_timeout_s=2.0)

    node._publish_telemetry(force=True)

    assert len(node._telemetry_pub.messages) == 1
    msg = node._telemetry_pub.messages[0]
    assert msg.robot_pose_available is False
    assert msg.gps_fix_available is False
    assert msg.gps_age_s >= 5.0
    assert msg.robot_lat != msg.robot_lat
    assert msg.robot_lon != msg.robot_lon
