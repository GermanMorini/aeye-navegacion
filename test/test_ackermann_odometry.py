import math
from pathlib import Path

from builtin_interfaces.msg import Time

from navegacion_gps.ackermann_odometry import (
    apply_measured_steer_sign,
    build_odom_transform,
    compute_yaw_rate,
    integrate_planar,
    normalize_angle,
)


def test_compute_yaw_rate_returns_zero_for_invalid_wheelbase() -> None:
    assert compute_yaw_rate(1.0, 0.2, 0.0) == 0.0


def test_compute_yaw_rate_matches_ackermann_model() -> None:
    yaw_rate = compute_yaw_rate(2.0, math.radians(10.0), 0.94)
    assert yaw_rate == math.tan(math.radians(10.0)) * 2.0 / 0.94


def test_apply_measured_steer_sign_can_invert_measurement() -> None:
    assert apply_measured_steer_sign(-12.5, invert_sign=False) == -12.5
    assert apply_measured_steer_sign(-12.5, invert_sign=True) == 12.5


def test_inverted_measured_steer_flips_yaw_rate_sign() -> None:
    steer_rad = math.radians(apply_measured_steer_sign(-10.0, invert_sign=True))
    yaw_rate = compute_yaw_rate(0.8, steer_rad, 0.94)
    assert yaw_rate > 0.0


def test_integrate_planar_uses_midpoint_heading() -> None:
    x_m, y_m, yaw_rad = integrate_planar(0.0, 0.0, 0.0, 1.0, 0.5, 0.2)
    assert x_m > 0.0
    assert y_m > 0.0
    assert yaw_rad > 0.0


def test_normalize_angle_wraps_to_pi_interval() -> None:
    wrapped = normalize_angle(4.0 * math.pi + 0.3)
    assert math.isclose(wrapped, 0.3, rel_tol=0.0, abs_tol=1.0e-9)


def test_build_odom_transform_uses_expected_frames_and_stamp() -> None:
    stamp = Time(sec=12, nanosec=345_000_000)
    tf_msg = build_odom_transform(
        stamp=stamp,
        odom_frame="odom",
        base_frame="base_footprint",
        x_m=1.25,
        y_m=-0.5,
        yaw_rad=math.radians(30.0),
    )

    assert tf_msg.header.stamp.sec == stamp.sec
    assert tf_msg.header.stamp.nanosec == stamp.nanosec
    assert tf_msg.header.frame_id == "odom"
    assert tf_msg.child_frame_id == "base_footprint"
    assert math.isclose(tf_msg.transform.translation.x, 1.25, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(tf_msg.transform.translation.y, -0.5, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(tf_msg.transform.translation.z, 0.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(
        tf_msg.transform.rotation.w,
        math.cos(math.radians(15.0)),
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        tf_msg.transform.rotation.z,
        math.sin(math.radians(15.0)),
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )


def test_ackermann_odometry_source_exposes_publish_odom_tf_toggle() -> None:
    source_path = Path(__file__).resolve().parents[1] / "navegacion_gps" / "ackermann_odometry.py"
    source_contents = source_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("publish_odom_tf", False)' in source_contents
    assert "if self._publish_odom_tf and self._tf_broadcaster is not None:" in source_contents
    assert "self._tf_broadcaster.sendTransform(transform)" in source_contents
    assert "stamp=msg.stamp" in source_contents
