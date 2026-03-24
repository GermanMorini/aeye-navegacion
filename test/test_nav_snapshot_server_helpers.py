import math
from pathlib import Path

from navegacion_gps.nav_snapshot_server import (
    compute_north_yaw_in_target_frame,
    decimate_pose_history_by_distance,
    is_allowed_frame_overlay,
    transform_pose_2d,
)


def test_decimate_pose_history_by_distance_keeps_spacing_and_tail() -> None:
    poses = [
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.0),
        (1.6, 0.0, 0.0),
        (2.9, 0.0, 0.0),
    ]
    out = decimate_pose_history_by_distance(poses, spacing_m=1.5)
    assert out[0] == poses[0]
    assert out[-1] == poses[-1]
    assert len(out) == 3


def test_transform_pose_2d_rotates_yaw_and_translation() -> None:
    x_out, y_out, yaw_out = transform_pose_2d(
        x_m=1.0,
        y_m=0.0,
        yaw_rad=0.2,
        tf_x_m=2.0,
        tf_y_m=-1.0,
        tf_yaw_rad=math.pi / 2.0,
    )
    assert math.isclose(x_out, 2.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(y_out, 0.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(yaw_out, 0.2 + (math.pi / 2.0), rel_tol=0.0, abs_tol=1.0e-9)


def test_compute_north_yaw_from_imu_and_base_frame() -> None:
    # If IMU yaw indicates heading east (0 rad), north in target frame is +pi/2.
    north_yaw = compute_north_yaw_in_target_frame(
        imu_yaw_enu_rad=0.0,
        base_yaw_in_target_rad=0.0,
    )
    assert math.isclose(north_yaw, math.pi / 2.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_allowed_frame_overlay_filters_strict_frame_set() -> None:
    assert is_allowed_frame_overlay("map") is True
    assert is_allowed_frame_overlay("odom") is True
    assert is_allowed_frame_overlay("base_footprint") is True
    assert is_allowed_frame_overlay("base_link") is False
    assert is_allowed_frame_overlay("imu_link") is False


def test_nav_snapshot_server_source_declares_new_overlay_topics_and_styles() -> None:
    source_path = Path(__file__).resolve().parents[1] / "navegacion_gps" / "nav_snapshot_server.py"
    source_contents = source_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("raw_odom_topic", "/wheel/odometry")' in source_contents
    assert 'self.declare_parameter("local_odom_topic", "/odometry/local")' in source_contents
    assert 'self.declare_parameter("global_odom_topic", "/odometry/global")' in source_contents
    assert 'self.declare_parameter("imu_topic", "/imu/data")' in source_contents
    assert 'self.declare_parameter("odom_arrow_spacing_m", 1.5)' in source_contents
    assert 'self.declare_parameter("overlay_thin_thickness", 1)' in source_contents
    assert 'self._draw_odometry_history(' in source_contents
    assert "color_bgr=(0, 165, 255)" in source_contents
    assert "color_bgr=(255, 255, 0)" in source_contents
    assert "color_bgr=(255, 0, 170)" in source_contents
    assert "NO_GO_ZONE_COLOR_BGR: Tuple[int, int, int] = (80, 0, 80)" in source_contents
    assert 'in {"map", "odom", "base_footprint"}' in source_contents
