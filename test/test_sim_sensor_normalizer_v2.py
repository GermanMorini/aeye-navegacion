import math
from pathlib import Path
import tempfile

from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import NavSatFix

from navegacion_gps.sim_sensor_normalizer_v2 import build_synthetic_navsat_fix_from_odom
from navegacion_gps.sim_sensor_normalizer_v2 import GpsNoiseModel
from navegacion_gps.sim_sensor_normalizer_v2 import load_navsat_datum_from_file
from navegacion_gps.sim_sensor_normalizer_v2 import resolve_gps_reference_mode
from navegacion_gps.sim_sensor_normalizer_v2 import resolve_gps_profile


def _build_fix() -> NavSatFix:
    msg = NavSatFix()
    msg.header.stamp.sec = 10
    msg.latitude = -31.4858037
    msg.longitude = -64.2410570
    msg.altitude = 500.0
    return msg


def _build_odom(*, x: float, y: float, z: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    return msg


def test_resolve_gps_profile_accepts_supported_values() -> None:
    assert resolve_gps_profile("ideal")["name"] == "ideal"
    assert resolve_gps_profile("m8n")["name"] == "m8n"
    assert resolve_gps_profile("f9p_rtk")["name"] == "f9p_rtk"


def test_resolve_gps_profile_rejects_unknown_values() -> None:
    with pytest.raises(ValueError):
        resolve_gps_profile("unknown")


def test_resolve_gps_reference_mode_accepts_supported_values() -> None:
    assert resolve_gps_reference_mode("ideal_from_local_odom") == "ideal_from_local_odom"
    assert resolve_gps_reference_mode("ideal_from_raw_odom") == "ideal_from_raw_odom"
    assert resolve_gps_reference_mode("ideal_from_ground_truth") == "ideal_from_ground_truth"


def test_resolve_gps_reference_mode_rejects_unknown_values() -> None:
    with pytest.raises(ValueError):
        resolve_gps_reference_mode("mystery_mode")


def test_ideal_profile_is_passthrough() -> None:
    model = GpsNoiseModel("ideal", random_seed=7)
    fix = _build_fix()

    out = model.transform(fix, reference_time_ns=10_000_000_000)

    assert out is not None
    assert math.isclose(out.latitude, fix.latitude)
    assert math.isclose(out.longitude, fix.longitude)
    assert math.isclose(out.altitude, fix.altitude)
    assert model.apply_profile_covariance(out) is True
    assert math.isclose(out.position_covariance[0], 0.02**2)
    assert math.isclose(out.position_covariance[8], 0.05**2)


def test_load_navsat_datum_from_file_reads_expected_schema() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "global_localization_v2.yaml"
        path.write_text(
            "navsat_transform:\n"
            "  ros__parameters:\n"
            "    datum: [-31.4858037, -64.2410570, 12.5]\n",
            encoding="utf-8",
        )

        datum = load_navsat_datum_from_file(str(path))

    assert datum == (-31.4858037, -64.2410570, 12.5)


def test_load_navsat_datum_from_file_rejects_missing_datum() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "global_localization_v2.yaml"
        path.write_text("navsat_transform:\n  ros__parameters: {}\n", encoding="utf-8")

        with pytest.raises(RuntimeError):
            load_navsat_datum_from_file(str(path))


def test_build_synthetic_navsat_fix_from_odom_returns_datum_at_origin() -> None:
    odom = _build_odom(x=0.0, y=0.0, z=0.0)

    fix = build_synthetic_navsat_fix_from_odom(
        odom,
        (-31.4858037, -64.2410570, 100.0),
        frame_id="gps_link",
    )

    assert math.isclose(fix.latitude, -31.4858037)
    assert math.isclose(fix.longitude, -64.2410570)
    assert math.isclose(fix.altitude, 100.0)
    assert fix.header.frame_id == "gps_link"


def test_build_synthetic_navsat_fix_from_odom_moves_lat_lon_in_expected_directions() -> None:
    odom = _build_odom(x=10.0, y=20.0, z=1.5)

    fix = build_synthetic_navsat_fix_from_odom(
        odom,
        (-31.4858037, -64.2410570, 100.0),
        frame_id="gps_link",
    )

    assert fix.latitude > -31.4858037
    assert fix.longitude > -64.2410570
    assert math.isclose(fix.altitude, 101.5)


def test_m8n_profile_applies_noise_and_publish_rate() -> None:
    model = GpsNoiseModel("m8n", random_seed=13)
    fix = _build_fix()

    first = model.transform(fix, reference_time_ns=10_000_000_000)
    second = model.transform(fix, reference_time_ns=10_010_000_000)

    assert first is not None
    assert second is None
    assert not math.isclose(first.latitude, fix.latitude)
    assert not math.isclose(first.longitude, fix.longitude)
    assert math.isclose(first.position_covariance[0], 1.8**2)
    assert math.isclose(first.position_covariance[8], 3.5**2)


def test_f9p_rtk_profile_is_tighter_than_m8n() -> None:
    m8n = GpsNoiseModel("m8n", random_seed=21)
    f9p_rtk = GpsNoiseModel("f9p_rtk", random_seed=21)
    fix = _build_fix()

    m8n_fix = m8n.transform(fix, reference_time_ns=10_000_000_000)
    f9p_fix = f9p_rtk.transform(fix, reference_time_ns=10_000_000_000)

    assert m8n_fix is not None
    assert f9p_fix is not None
    assert f9p_fix.position_covariance[0] < m8n_fix.position_covariance[0]
    assert f9p_fix.position_covariance[8] < m8n_fix.position_covariance[8]


def test_profile_covariance_is_explicit_even_without_noise() -> None:
    ideal = GpsNoiseModel("ideal", random_seed=1)
    fix = _build_fix()

    out = ideal.transform(fix, reference_time_ns=10_000_000_000)

    assert out is not None
    assert ideal.apply_profile_covariance(out) is True
    assert math.isclose(out.position_covariance[0], 0.02**2)
    assert math.isclose(out.position_covariance[4], 0.02**2)
    assert math.isclose(out.position_covariance[8], 0.05**2)


def test_ideal_profile_ignores_raw_fix_content_and_follows_synthetic_base() -> None:
    ideal = GpsNoiseModel("ideal", random_seed=1)
    odom = _build_odom(x=5.0, y=7.0, z=0.0)
    synthetic_fix = build_synthetic_navsat_fix_from_odom(
        odom,
        (-31.4858037, -64.2410570, 0.0),
        frame_id="gps_link",
    )
    raw_fix = _build_fix()
    raw_fix.latitude = 10.0
    raw_fix.longitude = 20.0
    raw_fix.altitude = 30.0

    out = ideal.transform(synthetic_fix, reference_time_ns=10_000_000_000)

    assert out is not None
    assert math.isclose(out.latitude, synthetic_fix.latitude)
    assert math.isclose(out.longitude, synthetic_fix.longitude)
    assert math.isclose(out.altitude, synthetic_fix.altitude)
    assert not math.isclose(out.latitude, raw_fix.latitude)
    assert not math.isclose(out.longitude, raw_fix.longitude)
