import threading
import time

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState

from interfaces.msg import CmdVelFinal
from navegacion_gps.nav_command_server import NavCommandServerNode


class _FakeArbNode:
    _build_cmd_vel_final = staticmethod(NavCommandServerNode._build_cmd_vel_final)

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._control_locked = False
        self._control_lock_reason = ""
        self.control_lock_heartbeat_required = False
        self._last_control_heartbeat_monotonic = time.monotonic()
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
        self.brake_async_calls = []
        self.failure_updates = []
        self.events = []

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

    def _publish_brake_sequence_async(self, brake_pct: int, skip_first: bool = False) -> None:
        self.brake_async_calls.append((int(brake_pct), bool(skip_first)))

    def _publish_telemetry(self, force: bool = False) -> None:
        self.telemetry_forced.append(bool(force))

    def _set_failure_locked(self, code: str = "", component: str = "") -> None:
        self.failure_updates.append((str(code), str(component)))

    def _publish_event(self, severity, component: str, code: str, message: str, *, details=None):
        self.events.append((int(severity), str(component), str(code), str(message), details))

    def cancel_current_goal(self):
        self.cancel_calls += 1
        return False, "timeout cancelling goal"

    def _cancel_goal_for_manual_takeover_async(self) -> None:
        self.cancel_current_goal()

    def set_manual_mode(self, enabled: bool):
        return NavCommandServerNode.set_manual_mode(self, enabled)

    def set_manual_cmd(self, linear_x: float, angular_z: float, brake_pct: int):
        return NavCommandServerNode.set_manual_cmd(self, linear_x, angular_z, brake_pct)

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


def test_on_cmd_vel_safe_publishes_auto_when_forward_without_goal_enabled() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = False
    node._collision_stop_active = False
    node.forward_cmd_vel_safe_without_goal = True

    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.1
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.5, 0.1, 0)]


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

    assert node.published == [(0.0, 0.0, 100)]
    assert node.brake_async_calls == [(100, True)]


def test_set_manual_mode_enables_even_if_cancel_fails() -> None:
    node = _FakeArbNode()
    node._current_goal_handle = object()
    ok, _err, enabled_after = NavCommandServerNode.set_manual_mode(node, True)

    assert ok is True
    assert enabled_after is True
    assert node._manual_enabled is True
    assert node._is_navigating is False
    assert node.cancel_calls == 1


def test_set_manual_mode_rejects_enable_when_control_locked() -> None:
    node = _FakeArbNode()
    node._control_locked = True
    node._control_lock_reason = "STARTUP_LOCKED"

    ok, err, enabled_after = NavCommandServerNode.set_manual_mode(node, True)

    assert ok is False
    assert enabled_after is False
    assert err == "control locked: STARTUP_LOCKED"


def test_on_teleop_cmd_does_not_activate_manual_for_zero_command() -> None:
    node = _FakeArbNode()
    msg = CmdVelFinal()

    NavCommandServerNode._on_teleop_cmd(node, msg)

    assert node._manual_enabled is False
    assert node.published == []


def test_on_teleop_cmd_activates_manual_for_nonzero_command() -> None:
    node = _FakeArbNode()
    msg = CmdVelFinal()
    msg.twist.linear.x = 1.2

    NavCommandServerNode._on_teleop_cmd(node, msg)

    assert node._manual_enabled is True
    assert node.published == [(0.0, 0.0, 0), (1.2, 0.0, 0)]
