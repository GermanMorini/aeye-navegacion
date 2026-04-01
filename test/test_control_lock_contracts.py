from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def test_control_lock_service_interfaces_are_declared() -> None:
    set_control_lock = (
        WORKSPACE_ROOT / "src" / "interfaces" / "srv" / "SetControlLock.srv"
    ).read_text(encoding="utf-8")
    touch_heartbeat = (
        WORKSPACE_ROOT / "src" / "interfaces" / "srv" / "TouchControlHeartbeat.srv"
    ).read_text(encoding="utf-8")

    assert "bool locked" in set_control_lock
    assert "bool ok" in set_control_lock
    assert "bool locked_after" in set_control_lock

    assert "---" in touch_heartbeat
    assert "bool ok" in touch_heartbeat
    assert "bool locked" in touch_heartbeat


def test_control_lock_state_fields_are_part_of_public_interfaces() -> None:
    get_nav_state = (
        WORKSPACE_ROOT / "src" / "interfaces" / "srv" / "GetNavState.srv"
    ).read_text(encoding="utf-8")
    nav_telemetry = (
        WORKSPACE_ROOT / "src" / "interfaces" / "msg" / "NavTelemetry.msg"
    ).read_text(encoding="utf-8")

    assert "bool control_locked" in get_nav_state
    assert "string control_lock_reason" in get_nav_state
    assert "bool control_locked" in nav_telemetry
    assert "string control_lock_reason" in nav_telemetry


def test_nav_command_server_declares_control_lock_contract() -> None:
    source = (
        WORKSPACE_ROOT
        / "src"
        / "navegacion_gps"
        / "navegacion_gps"
        / "nav_command_server.py"
    ).read_text(encoding="utf-8")

    assert 'self._control_locked = True' in source
    assert 'self._control_lock_reason = "STARTUP_LOCKED"' in source
    assert 'self.declare_parameter("control_lock_heartbeat_timeout_s", 10.0)' in source
    assert 'self.declare_parameter("control_lock_heartbeat_required", True)' in source
    assert "self._on_set_control_lock" in source
    assert "self._on_touch_control_heartbeat" in source
    assert 'event_code="UI_HEARTBEAT_TIMEOUT"' in source
