from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


_SCRIPT_PATH = Path(__file__).resolve().parents[3] / "tools" / "diagnose_nav_drift.py"
_SPEC = spec_from_file_location("diagnose_nav_drift", _SCRIPT_PATH)
_MODULE = module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)


def test_point_to_segment_distance_on_projection():
    d = _MODULE._point_to_segment_distance(1.0, 1.0, 0.0, 0.0, 2.0, 0.0)
    assert d == 1.0


def test_point_to_segment_distance_clamps_to_endpoint():
    d = _MODULE._point_to_segment_distance(3.0, 4.0, 0.0, 0.0, 2.0, 0.0)
    assert d == (17.0 ** 0.5)


def test_point_to_polyline_distance_uses_nearest_segment():
    points = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    d = _MODULE._point_to_polyline_distance(1.5, 0.5, points)
    assert d == 0.5


def test_point_to_polyline_distance_single_point():
    d = _MODULE._point_to_polyline_distance(4.0, 6.0, [(1.0, 2.0)])
    assert d == 5.0
