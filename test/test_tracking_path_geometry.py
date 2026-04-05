from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "navegacion_gps"
    / "tracking_path_geometry.py"
)
_SPEC = spec_from_file_location("tracking_path_geometry", _MODULE_PATH)
_MODULE = module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)


def test_compute_tracking_errors_uses_closest_segment_not_only_points():
    errors = _MODULE.compute_tracking_errors(
        robot_x=1.5,
        robot_y=0.5,
        robot_yaw_rad=0.0,
        path_points=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
    )
    assert errors is not None
    cross_track, heading_error, segment_index = errors
    assert segment_index == 0
    assert cross_track == 0.5
    assert heading_error == 0.0


def test_compute_tracking_errors_sign_is_negative_on_right_of_segment():
    errors = _MODULE.compute_tracking_errors(
        robot_x=1.0,
        robot_y=-0.25,
        robot_yaw_rad=0.0,
        path_points=[(0.0, 0.0), (2.0, 0.0)],
    )
    assert errors is not None
    cross_track, _, segment_index = errors
    assert segment_index == 0
    assert cross_track == -0.25


def test_compute_tracking_errors_normalizes_heading_error():
    errors = _MODULE.compute_tracking_errors(
        robot_x=0.5,
        robot_y=0.0,
        robot_yaw_rad=3.0,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
    )
    assert errors is not None
    _, heading_error, _ = errors
    assert -3.141592653589793 <= heading_error <= 3.141592653589793
