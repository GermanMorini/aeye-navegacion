import math

from navegacion_gps.gps_course_heading_core import GpsCourseHeadingEstimator


def _build_estimator(**overrides):
    params = {
        "min_distance_m": 0.30,
        "min_speed_mps": 0.4,
        "max_abs_steer_deg": 3.0,
        "max_abs_yaw_rate_rps": 0.06,
        "max_fix_age_s": 0.5,
        "sample_dt_min_s": 0.05,
        "sample_dt_max_s": 4.0,
        "max_pair_distance_base_m": 0.10,
        "max_pair_distance_speed_gain": 1.5,
        "max_pair_speed_error_mps": 0.75,
    }
    params.update(overrides)
    return GpsCourseHeadingEstimator(**params)


def test_estimator_accepts_rtk_like_pair_with_consistent_speed() -> None:
    estimator = _build_estimator()
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=0.36 / 111320.0, stamp_s=1.2)

    estimate = estimator.estimate(
        now_s=1.22,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is True
    assert estimate.reason == "ok"
    assert estimate.yaw_deg is not None
    assert math.isclose(estimate.yaw_deg, 0.0, abs_tol=1.0e-3)
    assert math.isclose(estimate.sample_dt_s or 0.0, 0.2, abs_tol=1.0e-9)


def test_estimator_rejects_pairs_outside_sample_dt_window() -> None:
    estimator = _build_estimator(sample_dt_max_s=0.15)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=0.36 / 111320.0, stamp_s=1.2)

    estimate = estimator.estimate(
        now_s=1.22,
        speed_mps=2.0,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "sample_dt_out_of_range"


def test_estimator_rejects_pair_distance_too_high_for_expected_motion() -> None:
    estimator = _build_estimator(min_distance_m=0.10)
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=1.5 / 111320.0, stamp_s=1.1)

    estimate = estimator.estimate(
        now_s=1.12,
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
    estimator.add_fix(lat=0.0, lon=0.0, stamp_s=1.0)
    estimator.add_fix(lat=0.0, lon=1.5 / 111320.0, stamp_s=1.5)

    estimate = estimator.estimate(
        now_s=1.52,
        speed_mps=0.5,
        steer_deg=0.0,
        steer_valid=True,
        yaw_rate_rps=0.0,
    )

    assert estimate.valid is False
    assert estimate.reason == "pair_speed_inconsistent"
