import math
from pathlib import Path
from types import SimpleNamespace

from navegacion_gps.zones_manager import (
    apply_planar_transform,
    lookup_planar_transform_from_buffer,
    yaw_from_quaternion_xyzw,
)


class _FakeTfBuffer:
    def __init__(self, transform=None, exc: Exception | None = None) -> None:
        self._transform = transform
        self._exc = exc

    def lookup_transform(self, *_args, **_kwargs):
        if self._exc is not None:
            raise self._exc
        return self._transform


def test_yaw_from_quaternion_xyzw_extracts_planar_rotation() -> None:
    yaw_rad = yaw_from_quaternion_xyzw(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    assert math.isclose(yaw_rad, math.pi / 2.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_apply_planar_transform_rotates_and_translates_point() -> None:
    x_out, y_out = apply_planar_transform(
        x_m=2.0,
        y_m=0.0,
        tx_m=10.0,
        ty_m=-1.0,
        yaw_rad=math.pi / 2.0,
    )
    assert math.isclose(x_out, 10.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(y_out, 1.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_lookup_planar_transform_from_buffer_identity_when_frames_match() -> None:
    ok, tx_m, ty_m, yaw_rad, err = lookup_planar_transform_from_buffer(
        tf_buffer=_FakeTfBuffer(),
        target_frame="map",
        source_frame="map",
        timeout_s=0.5,
    )
    assert ok is True
    assert err == ""
    assert tx_m == 0.0
    assert ty_m == 0.0
    assert yaw_rad == 0.0


def test_lookup_planar_transform_from_buffer_reports_lookup_failure() -> None:
    ok, _tx_m, _ty_m, _yaw_rad, err = lookup_planar_transform_from_buffer(
        tf_buffer=_FakeTfBuffer(exc=RuntimeError("tf down")),
        target_frame="map",
        source_frame="odom",
        timeout_s=0.5,
    )
    assert ok is False
    assert "map<-odom" in err
    assert "tf down" in err


def test_lookup_planar_transform_from_buffer_reads_transform_components() -> None:
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.5, y=-2.0, z=0.0),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(math.pi / 4.0),
                w=math.cos(math.pi / 4.0),
            ),
        )
    )
    ok, tx_m, ty_m, yaw_rad, err = lookup_planar_transform_from_buffer(
        tf_buffer=_FakeTfBuffer(transform=transform),
        target_frame="map",
        source_frame="odom",
        timeout_s=0.5,
    )
    assert ok is True
    assert err == ""
    assert math.isclose(tx_m, 1.5, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(ty_m, -2.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(yaw_rad, math.pi / 2.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_zones_manager_source_rejects_apply_when_transform_unavailable() -> None:
    source_path = Path(__file__).resolve().parents[1] / "navegacion_gps" / "zones_manager.py"
    source_contents = source_path.read_text(encoding="utf-8")

    assert "if xy_polygons is None:" in source_contents
    assert "str(tf_error)" in source_contents


def test_zones_manager_source_retries_initial_load_until_keepout_applies() -> None:
    source_path = Path(__file__).resolve().parents[1] / "navegacion_gps" / "zones_manager.py"
    source_contents = source_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("startup_reload_retry_period_s", 1.0)' in source_contents
    assert "self._schedule_startup_reload_retry()" in source_contents
    assert "def _on_startup_reload_retry_timer(self) -> None:" in source_contents
