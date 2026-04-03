import math
from pathlib import Path

from navegacion_gps.dual_gps_heading_sim import (
    _ll_to_enu_delta,
    _quat_to_yaw as _sim_quat_to_yaw,
    _yaw_to_quaternion,
)
from navegacion_gps.dual_gps_heading_real import _quat_to_yaw


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_BASELINE_M = 0.5     # lateral antenna separation used in cuatri_2gps.urdf
_EARTH_R = 6_378_137.0
_ANGLE_TOL = 1e-4     # rad (~0.006 deg) — accounts for spherical-earth linearisation


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _enu_to_ll(lat_center_deg: float, lon_center_deg: float,
               dx_east_m: float, dy_north_m: float):
    """Approximate ENU offset → lat/lon (flat-earth, good for <100 m)."""
    lat = lat_center_deg + math.degrees(dy_north_m / _EARTH_R)
    lon = lon_center_deg + math.degrees(
        dx_east_m / (_EARTH_R * math.cos(math.radians(lat_center_deg)))
    )
    return lat, lon


def _lateral_gps_positions(vehicle_heading_rad: float,
                            lat_center: float, lon_center: float,
                            baseline_m: float):
    """
    Return (right_lat, right_lon, left_lat, left_lon) for a robot with
    lateral antennas separated by *baseline_m*.

    Geometry (standard ROS/ENU):
        robot +x = forward  →  heading_rad from East, CCW
        robot -y = right    →  right antenna at (sin h, -cos h) * d/2
        robot +y = left     →  left  antenna at (-sin h,  cos h) * d/2
    """
    h = vehicle_heading_rad
    d = baseline_m / 2.0
    right_lat, right_lon = _enu_to_ll(lat_center, lon_center,
                                       d * math.sin(h), -d * math.cos(h))
    left_lat, left_lon   = _enu_to_ll(lat_center, lon_center,
                                       -d * math.sin(h), d * math.cos(h))
    return right_lat, right_lon, left_lat, left_lon


def _sim_pipeline_heading(vehicle_heading_rad: float,
                           lat_c: float = -31.485,
                           lon_c: float = -64.241) -> float:
    """
    Run the full sim heading pipeline for a robot at *vehicle_heading_rad*:
      1. Compute GPS positions for right/left antennas.
      2. Apply dual_gps_heading_sim math (right→left ENU vector).
      3. Apply dual_gps_heading_real relay with yaw_offset = -π/2.
    Returns the resulting heading in (-π, π].
    """
    rl, ro, ll, lo = _lateral_gps_positions(
        vehicle_heading_rad, lat_c, lon_c, _BASELINE_M
    )
    dx_e, dy_n = _ll_to_enu_delta(rl, ro, ll, lo)   # right→left baseline
    raw_heading = math.atan2(dy_n, dx_e)

    relay_offset = -math.pi / 2.0                    # correct value after fix
    corrected = raw_heading + relay_offset
    return (corrected + math.pi) % (2 * math.pi) - math.pi


def _angle_close(a: float, b: float, tol: float = _ANGLE_TOL) -> bool:
    diff = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) <= tol


# ---------------------------------------------------------------------------
# _ll_to_enu_delta — unit tests
# ---------------------------------------------------------------------------

def test_ll_to_enu_delta_pure_north():
    """Point 1 km north of reference → dy ≈ 1000 m, dx ≈ 0."""
    lat_ref, lon_ref = -31.0, -64.0
    lat_target = lat_ref + math.degrees(1000.0 / _EARTH_R)
    dx_e, dy_n = _ll_to_enu_delta(lat_ref, lon_ref, lat_target, lon_ref)
    assert abs(dx_e) < 0.01
    assert abs(dy_n - 1000.0) < 0.5


def test_ll_to_enu_delta_pure_east():
    """Point 1 km east of reference → dx ≈ 1000 m, dy ≈ 0."""
    lat_ref, lon_ref = -31.0, -64.0
    lon_target = lon_ref + math.degrees(
        1000.0 / (_EARTH_R * math.cos(math.radians(lat_ref)))
    )
    dx_e, dy_n = _ll_to_enu_delta(lat_ref, lon_ref, lat_ref, lon_target)
    assert abs(dy_n) < 0.01
    assert abs(dx_e - 1000.0) < 0.5


def test_ll_to_enu_delta_is_antisymmetric():
    """Swapping reference and target negates both components.

    The function uses a flat-earth approximation with cos(lat_ref) for the
    longitude scale factor.  Swapping ref/target changes lat_ref slightly, so
    the EW component isn't exactly antisymmetric — the residual is O(Δlat)
    and is < 0.05% of the distance for separations < 1 km.
    """
    lat_a, lon_a = -31.0, -64.0
    lat_b, lon_b = -31.002, -64.003
    dx1, dy1 = _ll_to_enu_delta(lat_a, lon_a, lat_b, lon_b)
    dx2, dy2 = _ll_to_enu_delta(lat_b, lon_b, lat_a, lon_a)
    distance = math.hypot(dx1, dy1)
    # Residual must be < 0.1% of the distance (flat-earth approximation error).
    assert abs(dx1 + dx2) < distance * 0.001
    assert abs(dy1 + dy2) < 1e-6   # latitude scale is constant → exact


# ---------------------------------------------------------------------------
# _quat_to_yaw / _yaw_to_quaternion — roundtrip
# ---------------------------------------------------------------------------

def test_yaw_quaternion_roundtrip_zero():
    qx, qy, qz, qw = _yaw_to_quaternion(0.0)
    assert abs(_quat_to_yaw(qx, qy, qz, qw)) < 1e-9


def test_yaw_quaternion_roundtrip_east():
    yaw_in = 0.0
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_in)
    assert abs(_quat_to_yaw(qx, qy, qz, qw) - yaw_in) < 1e-9


def test_yaw_quaternion_roundtrip_north():
    yaw_in = math.pi / 2.0
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_in)
    assert abs(_quat_to_yaw(qx, qy, qz, qw) - yaw_in) < 1e-9


def test_yaw_quaternion_roundtrip_west():
    yaw_in = math.pi
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_in)
    result = _quat_to_yaw(qx, qy, qz, qw)
    assert _angle_close(result, yaw_in)


def test_yaw_quaternion_roundtrip_south():
    yaw_in = -math.pi / 2.0
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_in)
    assert abs(_quat_to_yaw(qx, qy, qz, qw) - yaw_in) < 1e-9


def test_sim_quat_to_yaw_roundtrip_matches_real_helper():
    yaw_in = math.radians(37.0)
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_in)
    assert abs(_sim_quat_to_yaw(qx, qy, qz, qw) - yaw_in) < 1e-9
    assert abs(_quat_to_yaw(qx, qy, qz, qw) - yaw_in) < 1e-9


# ---------------------------------------------------------------------------
# Full pipeline: sim heading for cardinal robot orientations
# The critical property: output must equal the actual vehicle heading.
# A wrong relay offset (+π/2 instead of -π/2) yields output = heading + π.
# ---------------------------------------------------------------------------

def test_heading_pipeline_robot_facing_east():
    """spawn_yaw=0.0: right→left vector points North (+π/2), relay must give 0."""
    result = _sim_pipeline_heading(0.0)
    assert _angle_close(result, 0.0), (
        f"Expected heading≈0 (East) but got {math.degrees(result):.1f}°. "
        "Check yaw_offset_rad sign in dual_gps_heading_real_sim."
    )


def test_heading_pipeline_robot_facing_north():
    result = _sim_pipeline_heading(math.pi / 2.0)
    assert _angle_close(result, math.pi / 2.0), (
        f"Expected heading≈90° (North) but got {math.degrees(result):.1f}°"
    )


def test_heading_pipeline_robot_facing_west():
    result = _sim_pipeline_heading(math.pi)
    assert _angle_close(result, math.pi), (
        f"Expected heading≈180° (West) but got {math.degrees(result):.1f}°"
    )


def test_heading_pipeline_robot_facing_south():
    result = _sim_pipeline_heading(-math.pi / 2.0)
    assert _angle_close(result, -math.pi / 2.0), (
        f"Expected heading≈-90° (South) but got {math.degrees(result):.1f}°"
    )


def test_heading_pipeline_robot_facing_northwest():
    """Default vacio spawn angle 3π/4 (~135°)."""
    yaw = 3 * math.pi / 4.0
    result = _sim_pipeline_heading(yaw)
    assert _angle_close(result, yaw), (
        f"Expected heading≈135° (NW) but got {math.degrees(result):.1f}°"
    )


def test_heading_pipeline_offset_wrong_sign_would_fail():
    """Regression guard: +π/2 relay offset produces heading+π (180° off)."""
    rl, ro, ll, lo = _lateral_gps_positions(0.0, -31.485, -64.241, _BASELINE_M)
    dx_e, dy_n = _ll_to_enu_delta(rl, ro, ll, lo)
    raw = math.atan2(dy_n, dx_e)
    wrong_corrected = (raw + math.pi / 2.0 + math.pi) % (2 * math.pi) - math.pi
    # With +π/2 offset and East-facing robot, result ≈ π (West) — clearly wrong.
    assert not _angle_close(wrong_corrected, 0.0), (
        "Sanity check failed: wrong offset +π/2 should NOT produce correct East heading"
    )
    assert _angle_close(wrong_corrected, math.pi)


# ---------------------------------------------------------------------------
# Raw heading vector direction (no relay) — documents expected raw values
# ---------------------------------------------------------------------------

def test_raw_heading_for_east_facing_robot_is_north():
    """right→left baseline points North (π/2) when robot faces East."""
    rl, ro, ll, lo = _lateral_gps_positions(0.0, -31.485, -64.241, _BASELINE_M)
    dx_e, dy_n = _ll_to_enu_delta(rl, ro, ll, lo)
    raw = math.atan2(dy_n, dx_e)
    assert _angle_close(raw, math.pi / 2.0), (
        f"Expected raw baseline angle ≈ π/2 (North) but got {math.degrees(raw):.1f}°"
    )


def test_raw_heading_for_north_facing_robot_is_west():
    """right→left baseline points West (π) when robot faces North."""
    rl, ro, ll, lo = _lateral_gps_positions(math.pi / 2.0, -31.485, -64.241, _BASELINE_M)
    dx_e, dy_n = _ll_to_enu_delta(rl, ro, ll, lo)
    raw = math.atan2(dy_n, dx_e)
    assert _angle_close(raw, math.pi), (
        f"Expected raw baseline angle ≈ π (West) but got {math.degrees(raw):.1f}°"
    )


# ---------------------------------------------------------------------------
# EKF overlay contract: absolute yaw, correct topic
# ---------------------------------------------------------------------------

def test_dual_gps_heading_ekf_overlay_uses_absolute_imu1_yaw():
    overlay_path = _PACKAGE_ROOT / "config" / "dual_gps_heading_ekf_overlay.yaml"
    contents = overlay_path.read_text(encoding="utf-8")

    assert "imu1: /dual_gps/heading" in contents
    assert "imu1_differential: false" in contents
    assert "imu1_relative: false" in contents


def test_dual_gps_heading_ekf_overlay_disables_imu0_absolute_yaw():
    """imu0 (Pixhawk) must NOT contribute absolute yaw when dual GPS is active."""
    overlay_path = _PACKAGE_ROOT / "config" / "dual_gps_heading_ekf_overlay.yaml"
    contents = overlay_path.read_text(encoding="utf-8")

    # imu0_config row 1 (orientation): indices [3,4,5] = roll, pitch, yaw.
    # Yaw at index 5 must be false (0).
    # The overlay sets: [true, true, false] for imu0 orientation row.
    assert "true,  true,  false" in contents


# ---------------------------------------------------------------------------
# Launch contract: relay offset must be -π/2 in sim_local_v2
# ---------------------------------------------------------------------------

def test_sim_local_v2_dual_gps_relay_uses_negative_half_pi_offset():
    """
    The relay node (dual_gps_heading_real_sim) must use yaw_offset_rad = -π/2.
    A positive offset (+π/2) produces a 180° heading error in simulation.
    """
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    contents = launch_path.read_text(encoding="utf-8")

    assert '{"yaw_offset_rad": -math.pi / 2.0}' in contents, (
        "dual_gps_heading_real_sim must use yaw_offset_rad = -π/2. "
        "A positive offset causes 180° heading error and robot oscillation."
    )
    assert '{"yaw_offset_rad": math.pi / 2.0}' not in contents


def test_sim_local_v2_dual_gps_sim_disables_corrected_output():
    """heading_imu_topic must be empty so the corrected output goes through the relay."""
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    contents = launch_path.read_text(encoding="utf-8")

    assert '{"heading_imu_topic": ""}' in contents


def test_sim_local_v2_dual_gps_sim_publishes_raw_to_relay_topic():
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    contents = launch_path.read_text(encoding="utf-8")

    assert '{"raw_heading_imu_topic": "/ublox_rover/navheading"}' in contents
    assert '{"odom_heading_topic": "/odom_raw"}' in contents
    assert '{"input_topic": "/ublox_rover/navheading"}' in contents
    assert '{"output_topic": "/dual_gps/heading"}' in contents


def test_sim_local_v2_dual_gps_relay_writes_to_imu1_topic():
    """The relay output topic must match the EKF overlay imu1 topic."""
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    overlay_path = _PACKAGE_ROOT / "config" / "dual_gps_heading_ekf_overlay.yaml"

    launch_contents = launch_path.read_text(encoding="utf-8")
    overlay_contents = overlay_path.read_text(encoding="utf-8")

    assert '{"output_topic": "/dual_gps/heading"}' in launch_contents
    assert "imu1: /dual_gps/heading" in overlay_contents


def test_sim_local_v2_exposes_pose_topics_for_rviz_heading_visualization():
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    rviz_path = _PACKAGE_ROOT / "config" / "rviz_local_v2.rviz"

    launch_contents = launch_path.read_text(encoding="utf-8")
    rviz_contents = rviz_path.read_text(encoding="utf-8")

    assert '"output_topic": "/ublox_rover/navheading_pose"' in launch_contents
    assert '"output_topic": "/dual_gps/heading_pose"' in launch_contents
    assert launch_contents.count('{"odom_topic": "/odometry/local"}') == 2
    assert "Value: /ublox_rover/navheading_pose" in rviz_contents
    assert "Value: /dual_gps/heading_pose" in rviz_contents


# ---------------------------------------------------------------------------
# URDF antenna geometry contract
# ---------------------------------------------------------------------------

def test_sim_local_v2_gps_topics_match_urdf_physical_layout():
    """
    In cuatri_2gps.urdf:
      gps_link  (→ /gps/fix)  is at y=+0.25 m → LEFT  side of robot (+y body)
      gps_link2 (→ /gps2/fix) is at y=-0.25 m → RIGHT side of robot (-y body)

    dual_gps_heading_sim expects:
      gps_right_topic → physically RIGHT antenna (-y body)
      gps_left_topic  → physically LEFT  antenna (+y body)

    Therefore the correct mapping is:
      gps_right_topic = /gps2/fix  (gps_link2, y=-0.25, RIGHT)
      gps_left_topic  = /gps/fix   (gps_link,  y=+0.25, LEFT)

    Swapping these reverses the baseline vector sign and produces a 180° heading
    error regardless of the relay offset.
    """
    launch_path = _PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    contents = launch_path.read_text(encoding="utf-8")

    assert '{"gps_right_topic": "/gps2/fix"}' in contents, (
        "gps_right_topic must be /gps2/fix (gps_link2 at y=-0.25 = physically RIGHT). "
        "Using /gps/fix here swaps the baseline and causes 180° heading error."
    )
    assert '{"gps_left_topic": "/gps/fix"}' in contents, (
        "gps_left_topic must be /gps/fix (gps_link at y=+0.25 = physically LEFT)."
    )
    # Guard against accidental revert to the wrong mapping
    assert '{"gps_right_topic": "/gps/fix"}' not in contents
