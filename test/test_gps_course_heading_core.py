import math

from navegacion_gps.gps_course_heading_core import GpsCourseHeadingEstimator


def _build_estimator(**overrides):
    params = {
        "min_distance_m": 0.10,
        "min_speed_mps": 0.4,
        "max_abs_steer_deg": 3.0,
        "max_abs_yaw_rate_rps": 0.06,
        "max_fix_age_s": 0.5,
        "sample_dt_min_s": 0.05,
        "sample_dt_max_s": 4.0,
        "max_pair_distance_base_m": 0.10,
        "max_pair_distance_speed_gain": 1.5,
        "max_pair_speed_error_mps": 0.75,
        "heading_change_base_deg": 3.0,
        "heading_change_yaw_rate_gain": 1.0,
        "candidates": 5,
        "max_heading_dispersion_deg": 4.0,
    }
    params.update(overrides)
    return GpsCourseHeadingEstimator(**params)


def _east_offset_deg(distance_m: float) -> float:
    return float(distance_m) / 111320.0


def _lat_lon_from_distance_heading(distance_m: float, yaw_deg: float) -> tuple[float, float]:
    yaw_rad = math.radians(float(yaw_deg))
    north_m = float(distance_m) * math.sin(yaw_rad)
    east_m = float(distance_m) * math.cos(yaw_rad)
    return (-north_m / 111320.0, -east_m / 111320.0)


def test_estimator_accepts_rtk_like_candidates_with_consistent_angles() -> None:
    estimator = _build_estimator()
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.6), stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.4), stamp_s=1.1)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.2), stamp_s=1.2)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.3)

    estimate = estimator.estimate(
        now_s=1.32,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is True
    assert estimate.reason == "ok"
    assert estimate.yaw_deg is not None
    assert math.isclose(estimate.yaw_deg, 0.0, abs_tol=1.0e-3)
    assert estimate.candidate_count == 3
    assert estimate.heading_dispersion_deg is not None
    assert estimate.heading_dispersion_deg <= 1.0e-3


def test_estimator_rejects_pairs_outside_sample_dt_window() -> None:
    estimator = _build_estimator(sample_dt_max_s=0.09)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.6), stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.4), stamp_s=1.1)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.2), stamp_s=1.2)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.3)

    estimate = estimator.estimate(
        now_s=1.32,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "sample_dt_out_of_range"


def test_estimator_rejects_pair_distance_too_high_for_expected_motion() -> None:
    estimator = _build_estimator(min_distance_m=0.10)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(1.5), stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(1.0), stamp_s=1.05)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.5), stamp_s=1.10)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.15)

    estimate = estimator.estimate(
        now_s=1.17,
        speed_mps=0.5,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "pair_distance_too_high"


def test_estimator_rejects_pair_with_speed_inconsistency() -> None:
    estimator = _build_estimator(
        min_distance_m=0.10,
        max_pair_distance_base_m=1.0,
        max_pair_distance_speed_gain=3.0,
    )
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(2.0), stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(1.4), stamp_s=1.5)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.8), stamp_s=2.0)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=2.5)

    estimate = estimator.estimate(
        now_s=2.52,
        speed_mps=0.5,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "pair_speed_inconsistent"


def test_estimator_rejects_when_heading_change_exceeds_integrated_yaw_limit() -> None:
    estimator = _build_estimator(
        heading_change_base_deg=3.0,
        max_heading_dispersion_deg=20.0,
    )
    estimator.add_fix(*_lat_lon_from_distance_heading(0.6, 30.0), stamp_s=1.0)
    estimator.add_fix(*_lat_lon_from_distance_heading(0.4, 0.0), stamp_s=1.1)
    estimator.add_fix(*_lat_lon_from_distance_heading(0.2, 0.0), stamp_s=1.2)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.3)

    estimate = estimator.estimate(
        now_s=1.32,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "heading_change_too_high"


def test_estimator_rejects_when_insufficient_candidates_remain() -> None:
    estimator = _build_estimator()
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.4), stamp_s=1.1)
    estimator.add_fix(lat=0.0, lon=-_east_offset_deg(0.2), stamp_s=1.2)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.3)

    estimate = estimator.estimate(
        now_s=1.32,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "insufficient_heading_candidates"


def test_estimator_rejects_when_heading_dispersion_is_too_high() -> None:
    estimator = _build_estimator(
        max_heading_dispersion_deg=4.0,
        heading_change_base_deg=20.0,
    )
    estimator.add_fix(*_lat_lon_from_distance_heading(0.6, -8.0), stamp_s=1.0)
    estimator.add_fix(*_lat_lon_from_distance_heading(0.4, 8.0), stamp_s=1.1)
    estimator.add_fix(*_lat_lon_from_distance_heading(0.2, 0.0), stamp_s=1.2)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.3)

    estimate = estimator.estimate(
        now_s=1.32,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "heading_dispersion_too_high"


def test_estimator_clamps_candidates_to_minimum_three() -> None:
    estimator = _build_estimator(candidates=1)

    assert estimator.candidates == 3
