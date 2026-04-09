from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from navegacion_gps.route_tools import (
    RouteFileError,
    RoutePoint,
    load_route_yaml,
    normalize_angle_rad,
    route_to_yaml_data,
    save_route_yaml,
    should_record_point,
)


def test_should_record_point_when_distance_threshold_is_met() -> None:
    last_saved = RoutePoint(x=0.0, y=0.0, yaw=0.0)
    current = RoutePoint(x=1.1, y=0.0, yaw=0.0)

    assert should_record_point(last_saved, current, min_distance_m=1.0, min_yaw_rad=0.5)


def test_should_record_point_when_yaw_threshold_is_met() -> None:
    last_saved = RoutePoint(x=0.0, y=0.0, yaw=0.0)
    current = RoutePoint(x=0.1, y=0.0, yaw=0.3)

    assert should_record_point(last_saved, current, min_distance_m=1.0, min_yaw_rad=0.2)


def test_should_not_record_point_when_thresholds_are_not_met() -> None:
    last_saved = RoutePoint(x=0.0, y=0.0, yaw=0.0)
    current = RoutePoint(x=0.3, y=0.0, yaw=0.05)

    assert not should_record_point(
        last_saved,
        current,
        min_distance_m=1.0,
        min_yaw_rad=0.2,
    )


def test_route_yaml_round_trip_preserves_frame_and_points(tmp_path: Path) -> None:
    route_path = tmp_path / "route.yaml"
    original_points = [
        RoutePoint(x=1.23, y=4.56, yaw=normalize_angle_rad(0.78)),
        RoutePoint(x=2.10, y=5.20, yaw=normalize_angle_rad(-2.90)),
    ]

    save_route_yaml(route_path, "map", original_points)
    frame_id, loaded_points = load_route_yaml(route_path)

    assert frame_id == "map"
    assert loaded_points == original_points


def test_route_to_yaml_data_uses_simple_legible_schema() -> None:
    data = route_to_yaml_data("map", [RoutePoint(x=1.0, y=2.0, yaw=3.0)])

    assert data == {
        "frame_id": "map",
        "points": [{"x": 1.0, "y": 2.0, "yaw": 3.0}],
    }


def test_load_route_yaml_rejects_empty_points(tmp_path: Path) -> None:
    route_path = tmp_path / "invalid_route.yaml"
    route_path.write_text("frame_id: map\npoints: []\n", encoding="utf-8")

    with pytest.raises(RouteFileError, match="non-empty points list"):
        load_route_yaml(route_path)


def test_setup_exposes_route_recorder_and_route_player_entrypoints() -> None:
    setup_contents = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "'route_recorder = navegacion_gps.route_recorder:main'" in setup_contents
    assert "'route_player = navegacion_gps.route_player:main'" in setup_contents


def test_real_launch_remaps_global_filter_output_to_odometry_global() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "real.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'remappings=[("odometry/filtered", "/odometry/global")]' in launch_contents


def test_route_recorder_defaults_to_filtered_global_pose_in_map_frame() -> None:
    recorder_contents = (
        PACKAGE_ROOT / "navegacion_gps" / "route_recorder.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("pose_topic", "/odometry/global")' in recorder_contents
    assert 'self.declare_parameter("expected_frame_id", "map")' in recorder_contents
    assert 'self.declare_parameter("min_distance_m", 1.0)' in recorder_contents
    assert 'self.declare_parameter("min_yaw_deg", 10.0)' in recorder_contents
    assert 'self.create_service(Trigger, self.start_service_name, self._on_start)' in recorder_contents
    assert 'self.create_service(Trigger, self.stop_service_name, self._on_stop)' in recorder_contents


def test_route_player_uses_navigate_through_poses_action() -> None:
    player_contents = (
        PACKAGE_ROOT / "navegacion_gps" / "route_player.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("action_name", "navigate_through_poses")' in player_contents
    assert "NavigateThroughPoses.Goal()" in player_contents
    assert "self._navigate_client.wait_for_server" in player_contents
