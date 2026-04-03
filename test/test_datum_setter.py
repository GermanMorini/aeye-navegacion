import math
import threading
import time

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from interfaces.srv import SetDatum
from navegacion_gps.datum_setter import DatumSetterNode


class _FakeLogger:
    def __init__(self) -> None:
        self.infos = []
        self.warnings = []

    def info(self, msg: str) -> None:
        self.infos.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warnings.append(str(msg))


class _FakeAutoNode:
    _combined_rtk_locked = DatumSetterNode._combined_rtk_locked
    _get_fresh_imu_yaw = DatumSetterNode._get_fresh_imu_yaw

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_gps_fix = None
        self._last_navsat_rtk = False
        self._last_rtk_status_is_rtk = False
        self._last_rtk_status_text = ""
        self._last_imu_yaw = None
        self._last_imu_yaw_monotonic = None
        self.imu_yaw_max_age_s = 1.0
        self._rtk_current = False
        self._pending_auto_set = False
        self._pending_first_fix_auto_set = False
        self.auto_set_on_first_fix = False
        self.already_set = False
        self.logger = _FakeLogger()
        self.auto_calls = []

    def _auto_set_datum_from_coords(
        self, lat: float, lon: float, gps_is_rtk: bool, reason: str
    ) -> None:
        self.auto_calls.append((float(lat), float(lon), bool(gps_is_rtk), str(reason)))

    def get_logger(self):
        return self.logger


class _FakeSetNode:
    _combined_rtk_locked = DatumSetterNode._combined_rtk_locked

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_gps_fix = None
        self._last_navsat_rtk = False
        self._last_rtk_status_is_rtk = False
        self._rtk_current = False
        self.already_set = False
        self.calls = []

    def _apply_datum(
        self,
        lat: float,
        lon: float,
        source: str,
        gps_is_rtk: bool,
        used_current_gps: bool,
    ):
        self.calls.append(
            (
                float(lat),
                float(lon),
                str(source),
                bool(gps_is_rtk),
                bool(used_current_gps),
            )
        )
        status = DatumSetterNode._build_status_message(
            already_set_before=bool(self.already_set),
            gps_is_rtk=bool(gps_is_rtk),
            used_current_gps=bool(used_current_gps),
            source=str(source),
        )
        return True, "", status, bool(self.already_set)


class _FakeClockNow:
    def to_msg(self) -> Time:
        return Time()


class _FakeClock:
    def now(self) -> _FakeClockNow:
        return _FakeClockNow()


class _FakeSetWithImuNode:
    _combined_rtk_locked = DatumSetterNode._combined_rtk_locked
    _get_fresh_imu_yaw = DatumSetterNode._get_fresh_imu_yaw
    _apply_datum = DatumSetterNode._apply_datum
    _on_set_datum = DatumSetterNode._on_set_datum

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._set_operation_lock = threading.Lock()
        self._last_gps_fix = (-31.42, -64.18)
        self._last_navsat_rtk = False
        self._last_rtk_status_is_rtk = False
        self._last_imu_yaw = None
        self._last_imu_yaw_monotonic = None
        self.imu_yaw_max_age_s = 1.0
        self.already_set = False
        self._datum_lat = None
        self._datum_lon = None
        self._last_set_stamp = None
        self._last_set_source = ""
        self._last_set_with_rtk = False
        self.calls = []

    def _call_set_datum(self, lat: float, lon: float, yaw: float):
        self.calls.append((float(lat), float(lon), float(yaw)))
        return True, ""

    def get_clock(self) -> _FakeClock:
        return _FakeClock()


class _ImmediateFuture:
    def __init__(self, result) -> None:
        self._result = result

    def done(self) -> bool:
        return True

    def result(self):
        return self._result


class _RecordingDatumClient:
    def __init__(self) -> None:
        self.request = None

    def call_async(self, req):
        self.request = req
        return _ImmediateFuture(object())


class _FakeCallDatumNode:
    _call_set_datum = DatumSetterNode._call_set_datum

    def __init__(self) -> None:
        self.datum_call_retries = 1
        self.datum_call_timeout_s = 1.0
        self.datum_retry_delay_s = 0.0
        self._last_datum_error = None
        self.client = _RecordingDatumClient()

    def _resolve_datum_client(self):
        return self.client

    def _wait_for_future(self, future, timeout_sec: float):
        return future.result()


def _gps_msg(lat: float, lon: float, status: int) -> NavSatFix:
    msg = NavSatFix()
    msg.latitude = float(lat)
    msg.longitude = float(lon)
    msg.status.status = int(status)
    return msg


def _imu_msg(yaw_rad: float) -> Imu:
    msg = Imu()
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = math.sin(0.5 * float(yaw_rad))
    msg.orientation.w = math.cos(0.5 * float(yaw_rad))
    return msg


def test_parse_coords_accepts_empty_and_pair() -> None:
    ok, err, use_current, lat, lon = DatumSetterNode._parse_coords([])
    assert ok is True
    assert err == ""
    assert use_current is True
    assert math.isnan(lat)
    assert math.isnan(lon)

    ok, err, use_current, lat, lon = DatumSetterNode._parse_coords([-31.5, -64.2])
    assert ok is True
    assert err == ""
    assert use_current is False
    assert lat == -31.5
    assert lon == -64.2


def test_parse_coords_rejects_invalid_shapes_and_ranges() -> None:
    ok, _err, _use_current, _lat, _lon = DatumSetterNode._parse_coords([-31.0])
    assert ok is False

    ok, _err, _use_current, _lat, _lon = DatumSetterNode._parse_coords([-95.0, -64.0])
    assert ok is False


def test_status_text_detection_covers_float_and_fixed() -> None:
    assert DatumSetterNode._status_text_is_rtk("RTK_FLOAT") is True
    assert DatumSetterNode._status_text_is_rtk("RTK_FIXED (22 sats)") is True
    assert DatumSetterNode._status_text_is_rtk("rtk_fix") is True
    assert DatumSetterNode._status_text_is_rtk("gps_only") is False


def test_auto_set_happens_once_on_rtk_edge() -> None:
    node = _FakeAutoNode()
    DatumSetterNode._on_imu(node, _imu_msg(0.2))

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.0, -64.0, NavSatStatus.STATUS_FIX),
    )
    assert node.auto_calls == []

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.0, -64.0, NavSatStatus.STATUS_GBAS_FIX),
    )
    assert len(node.auto_calls) == 1
    assert node.auto_calls[0][3] == "rtk_edge_gps"

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.1, -64.1, NavSatStatus.STATUS_GBAS_FIX),
    )
    assert len(node.auto_calls) == 1


def test_auto_set_pending_when_rtk_arrives_before_gps() -> None:
    node = _FakeAutoNode()
    DatumSetterNode._on_imu(node, _imu_msg(0.3))

    status_msg = String()
    status_msg.data = "RTK_FLOAT"
    DatumSetterNode._on_rtk_status(node, status_msg)
    assert node.auto_calls == []

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.25, -64.25, NavSatStatus.STATUS_FIX),
    )
    assert len(node.auto_calls) == 1
    assert node.auto_calls[0][3] == "rtk_edge_pending_gps"


def test_auto_set_is_deferred_until_imu_yaw_becomes_available() -> None:
    node = _FakeAutoNode()

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.5, -64.5, NavSatStatus.STATUS_GBAS_FIX),
    )
    assert node.auto_calls == []
    assert node._pending_auto_set is True

    DatumSetterNode._on_imu(node, _imu_msg(0.6))
    assert len(node.auto_calls) == 1
    assert node.auto_calls[0][3] == "rtk_edge_pending_imu"


def test_auto_set_can_run_on_first_fix_without_rtk() -> None:
    node = _FakeAutoNode()
    node.auto_set_on_first_fix = True
    DatumSetterNode._on_imu(node, _imu_msg(0.1))

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.7, -64.7, NavSatStatus.STATUS_FIX),
    )
    assert len(node.auto_calls) == 1
    assert node.auto_calls[0][2] is False
    assert node.auto_calls[0][3] == "first_fix_gps"


def test_auto_set_first_fix_waits_for_imu_when_needed() -> None:
    node = _FakeAutoNode()
    node.auto_set_on_first_fix = True

    DatumSetterNode._on_gps_fix(
        node,
        _gps_msg(-31.8, -64.8, NavSatStatus.STATUS_FIX),
    )
    assert node.auto_calls == []
    assert node._pending_first_fix_auto_set is True

    DatumSetterNode._on_imu(node, _imu_msg(0.4))
    assert len(node.auto_calls) == 1
    assert node.auto_calls[0][3] == "first_fix_pending_imu"


def test_set_datum_fails_without_current_gps_for_empty_coords() -> None:
    node = _FakeSetNode()
    req = SetDatum.Request()
    req.coords = []
    res = SetDatum.Response()

    out = DatumSetterNode._on_set_datum(node, req, res)
    assert out.ok is False
    assert "no current GPS sample available" in out.error


def test_set_datum_reapplies_when_already_set() -> None:
    node = _FakeSetNode()
    node.already_set = True
    req = SetDatum.Request()
    req.coords = [-31.5, -64.3]
    res = SetDatum.Response()

    out = DatumSetterNode._on_set_datum(node, req, res)
    assert out.ok is True
    assert out.already_set_before is True
    assert len(node.calls) == 1
    assert node.calls[0][2] == "service_manual_coords"


def test_set_datum_without_rtk_sets_and_reports_warning() -> None:
    node = _FakeSetNode()
    node._last_navsat_rtk = False
    node._last_rtk_status_is_rtk = False
    req = SetDatum.Request()
    req.coords = [-31.4, -64.4]
    res = SetDatum.Response()

    out = DatumSetterNode._on_set_datum(node, req, res)
    assert out.ok is True
    assert out.gps_is_rtk is False
    assert "without RTK quality" in out.status_message
    assert len(node.calls) == 1
    assert node.calls[0][3] is False


def test_extract_yaw_from_quaternion_handles_wrap() -> None:
    ok, yaw = DatumSetterNode._extract_yaw_from_quaternion(
        0.0,
        0.0,
        math.sin(-3.0 / 2.0),
        math.cos(-3.0 / 2.0),
    )
    assert ok is True
    assert math.isclose(yaw, -3.0, rel_tol=1e-6, abs_tol=1e-6)

    ok, yaw = DatumSetterNode._extract_yaw_from_quaternion(
        0.0,
        0.0,
        math.sin((3.1) / 2.0),
        math.cos((3.1) / 2.0),
    )
    assert ok is True
    assert math.isclose(yaw, 3.1, rel_tol=1e-6, abs_tol=1e-6)


def test_set_datum_manual_fails_when_no_fresh_imu_yaw() -> None:
    node = _FakeSetWithImuNode()
    req = SetDatum.Request()
    req.coords = [-31.5, -64.3]
    res = SetDatum.Response()

    out = DatumSetterNode._on_set_datum(node, req, res)
    assert out.ok is False
    assert "IMU yaw" in out.error
    assert node.calls == []


def test_set_datum_manual_uses_imu_yaw_in_setdatum_call() -> None:
    node = _FakeSetWithImuNode()
    node._last_imu_yaw = 1.234
    node._last_imu_yaw_monotonic = time.monotonic()
    req = SetDatum.Request()
    req.coords = [-31.5, -64.3]
    res = SetDatum.Response()

    out = DatumSetterNode._on_set_datum(node, req, res)
    assert out.ok is True
    assert len(node.calls) == 1
    assert math.isclose(node.calls[0][2], 1.234, rel_tol=1e-6, abs_tol=1e-6)


def test_call_set_datum_builds_quaternion_from_yaw() -> None:
    node = _FakeCallDatumNode()

    ok, err = node._call_set_datum(-31.5, -64.3, 1.234)
    assert ok is True
    assert err == ""
    assert node.client.request is not None

    geo_pose = node.client.request.geo_pose
    assert math.isclose(
        geo_pose.orientation.z, math.sin(1.234 / 2.0), rel_tol=1e-6, abs_tol=1e-6
    )
    assert math.isclose(
        geo_pose.orientation.w, math.cos(1.234 / 2.0), rel_tol=1e-6, abs_tol=1e-6
    )
