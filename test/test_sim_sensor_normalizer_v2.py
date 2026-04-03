import math
from pathlib import Path

from geometry_msgs.msg import Quaternion
from navegacion_gps.sim_sensor_normalizer_v2 import (
    _apply_yaw_offset_to_quaternion,
    _normalize_angle,
    _quaternion_from_yaw_xyzw,
    _update_auto_yaw_calibration,
    _yaw_from_quaternion_xyzw,
)


def test_yaw_helpers_roundtrip_and_identity_offset() -> None:
    yaw = math.radians(35.0)
    qx, qy, qz, qw = _quaternion_from_yaw_xyzw(yaw)
    recovered = _yaw_from_quaternion_xyzw(qx, qy, qz, qw)
    assert recovered is not None
    assert math.isclose(recovered, yaw, rel_tol=0.0, abs_tol=1.0e-9)

    quat = Quaternion(x=qx, y=qy, z=qz, w=qw)
    corrected = _apply_yaw_offset_to_quaternion(quat, 0.0)
    assert corrected is not None
    assert math.isclose(corrected[0], qx, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(corrected[1], qy, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(corrected[2], qz, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(corrected[3], qw, rel_tol=0.0, abs_tol=1.0e-9)


def test_apply_yaw_offset_handles_wrap_near_pi() -> None:
    yaw = math.radians(179.0)
    qx, qy, qz, qw = _quaternion_from_yaw_xyzw(yaw)
    quat = Quaternion(x=qx, y=qy, z=qz, w=qw)
    corrected = _apply_yaw_offset_to_quaternion(quat, math.radians(10.0))
    assert corrected is not None
    corrected_yaw = _yaw_from_quaternion_xyzw(*corrected)
    assert corrected_yaw is not None
    expected = _normalize_angle(math.radians(189.0))
    assert math.isclose(corrected_yaw, expected, rel_tol=0.0, abs_tol=1.0e-9)


def test_update_auto_yaw_calibration_calibrates_when_stopped() -> None:
    calibrated, offset, timed_out = _update_auto_yaw_calibration(
        auto_enabled=True,
        already_calibrated=False,
        now_s=2.0,
        calibration_started_s=0.0,
        calibration_timeout_s=5.0,
        imu_yaw_rad=math.radians(-50.0),
        odom_yaw_rad=math.radians(140.0),
        odom_speed_mps=0.01,
        speed_threshold_mps=0.05,
        current_auto_offset_rad=0.0,
    )
    assert calibrated is True
    assert timed_out is False
    expected = _normalize_angle(math.radians(190.0))
    assert math.isclose(offset, expected, rel_tol=0.0, abs_tol=1.0e-9)


def test_update_auto_yaw_calibration_waits_while_robot_is_moving() -> None:
    calibrated, offset, timed_out = _update_auto_yaw_calibration(
        auto_enabled=True,
        already_calibrated=False,
        now_s=1.0,
        calibration_started_s=0.0,
        calibration_timeout_s=5.0,
        imu_yaw_rad=0.1,
        odom_yaw_rad=0.3,
        odom_speed_mps=0.3,
        speed_threshold_mps=0.05,
        current_auto_offset_rad=0.0,
    )
    assert calibrated is False
    assert timed_out is False
    assert math.isclose(offset, 0.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_update_auto_yaw_calibration_timeout_keeps_manual_only() -> None:
    calibrated, offset, timed_out = _update_auto_yaw_calibration(
        auto_enabled=True,
        already_calibrated=False,
        now_s=4.0,
        calibration_started_s=0.0,
        calibration_timeout_s=3.0,
        imu_yaw_rad=None,
        odom_yaw_rad=None,
        odom_speed_mps=None,
        speed_threshold_mps=0.05,
        current_auto_offset_rad=0.25,
    )
    assert calibrated is True
    assert timed_out is True
    assert math.isclose(offset, 0.25, rel_tol=0.0, abs_tol=1.0e-9)


def test_sim_sensor_normalizer_source_declares_yaw_auto_calibration_params() -> None:
    source_path = (
        Path(__file__).resolve().parents[1]
        / "navegacion_gps"
        / "sim_sensor_normalizer_v2.py"
    )
    source_contents = source_path.read_text(encoding="utf-8")

    assert (
        'self.declare_parameter("imu_auto_calibrate_yaw_from_odom", True)'
        in source_contents
    )
    assert 'self.declare_parameter("imu_yaw_offset_rad", 0.0)' in source_contents
    assert (
        'self.declare_parameter("imu_yaw_calib_odom_topic", "/odom_raw")'
        in source_contents
    )
    assert (
        'self.declare_parameter("imu_yaw_calib_speed_threshold_mps", 0.05)'
        in source_contents
    )
    assert 'self.declare_parameter("imu_yaw_calib_timeout_s", 3.0)' in source_contents
    assert (
        'self.declare_parameter(\n'
        '            "gps_horizontal_variance", DEFAULT_GPS_HORIZONTAL_VARIANCE\n'
        "        )" in source_contents
    )
    assert (
        'self.declare_parameter("gps_vertical_variance", DEFAULT_GPS_VERTICAL_VARIANCE)'
        in source_contents
    )
