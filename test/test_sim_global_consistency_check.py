from navegacion_gps.sim_global_consistency_check import build_report
from navegacion_gps.sim_global_consistency_check import align_base_pose_to_odom
from navegacion_gps.sim_global_consistency_check import classify_consistency_level
from navegacion_gps.sim_global_consistency_check import infer_likely_divergence_origin
from navegacion_gps.sim_global_consistency_check import pairwise_heading_gaps
from navegacion_gps.sim_global_consistency_check import pairwise_position_gaps


def _build_result(
    *,
    max_position_gap_m: float,
    max_position_pair: str,
    max_heading_gap_rad: float,
    max_heading_pair: str,
    stopped_early: bool = False,
) -> dict:
    return {
        "sample_count": 4,
        "elapsed_s": 2.0,
        "max_position_gap_m": max_position_gap_m,
        "max_position_pair": max_position_pair,
        "max_heading_gap_rad": max_heading_gap_rad,
        "max_heading_pair": max_heading_pair,
        "max_source_ages_s": {"global": 0.12, "local": 0.08},
        "max_source_age_s": 0.12,
        "max_stamp_gaps_s": {"global_vs_local": 0.04},
        "max_stamp_delta_s": 0.04,
        "max_stamp_delta_pair": "global_vs_local",
        "max_position_gaps_m": {
            "global_vs_local": max_position_gap_m,
            "base_vs_local": 0.2,
            "base_vs_gps": 0.3,
            "base_vs_gps_odom": 0.2,
            "global_vs_base": max_position_gap_m,
            "global_vs_gps": max_position_gap_m,
            "global_vs_gps_odom": max_position_gap_m,
            "gps_vs_local": 0.4,
            "gps_odom_vs_local": 0.3,
            "gps_odom_vs_gps": 0.2,
        },
        "max_heading_gaps_rad": {
            "base_vs_local": 0.1,
            "base_vs_global": max_heading_gap_rad,
            "global_vs_local": max_heading_gap_rad,
        },
        "mean_position_gaps_m": {},
        "mean_heading_gaps_rad": {},
        "first_sample": {},
        "last_sample": {},
        "threshold_breach": {
            "type": "position_gap_m",
            "pair": max_position_pair,
            "value": max_position_gap_m,
            "elapsed_s": 2.0,
        }
        if stopped_early
        else None,
        "stopped_early": stopped_early,
        "level": "fail" if max_position_gap_m >= 3.0 else "ok",
    }


def test_pairwise_position_gaps_computes_expected_pairs() -> None:
    gaps = pairwise_position_gaps(
        {
            "local": (0.0, 0.0),
            "global": (3.0, 4.0),
            "gps": (0.0, 4.0),
        }
    )

    assert gaps["global_vs_local"] == 5.0
    assert gaps["global_vs_gps"] == 3.0
    assert gaps["gps_vs_local"] == 4.0


def test_pairwise_heading_gaps_normalizes_wraparound() -> None:
    gaps = pairwise_heading_gaps(
        {
            "local": 3.13,
            "global": -3.13,
        }
    )

    assert gaps["global_vs_local"] < 0.05


def test_align_base_pose_to_odom_keeps_relative_motion_in_odom_frame() -> None:
    aligned_xy, aligned_yaw = align_base_pose_to_odom(
        raw_xy=(12.0, 7.0),
        raw_yaw=0.9,
        raw_origin_xy=(10.0, 5.0),
        raw_origin_yaw=0.4,
        odom_origin_xy=(1.0, 2.0),
        odom_origin_yaw=0.1,
    )

    assert aligned_xy == (3.0, 4.0)
    assert aligned_yaw == 0.6


def test_classify_consistency_level_uses_position_and_heading_thresholds() -> None:
    assert (
        classify_consistency_level(
            max_position_gap_m=0.4,
            max_heading_gap_rad=0.1,
            warn_position_gap_m=1.0,
            fail_position_gap_m=3.0,
            warn_heading_gap_rad=0.35,
            fail_heading_gap_rad=0.8,
        )
        == "ok"
    )
    assert (
        classify_consistency_level(
            max_position_gap_m=0.4,
            max_heading_gap_rad=0.5,
            warn_position_gap_m=1.0,
            fail_position_gap_m=3.0,
            warn_heading_gap_rad=0.35,
            fail_heading_gap_rad=0.8,
        )
        == "warn"
    )
    assert (
        classify_consistency_level(
            max_position_gap_m=3.2,
            max_heading_gap_rad=0.2,
            warn_position_gap_m=1.0,
            fail_position_gap_m=3.0,
            warn_heading_gap_rad=0.35,
            fail_heading_gap_rad=0.8,
        )
        == "fail"
    )


def test_infer_likely_divergence_origin_marks_global_when_it_separates_from_all() -> None:
    assert (
        infer_likely_divergence_origin(
            position_gaps_m={
                "global_vs_local": 6.0,
                "global_vs_gps": 5.5,
                "global_vs_gps_odom": 5.6,
                "base_vs_global": 5.0,
                "base_vs_local": 0.2,
                "base_vs_gps": 0.3,
                "base_vs_gps_odom": 0.2,
                "gps_vs_local": 0.4,
                "gps_odom_vs_local": 0.3,
                "gps_odom_vs_gps": 0.2,
            },
            heading_gaps_rad={
                "base_vs_local": 0.1,
                "base_vs_global": 1.2,
                "global_vs_local": 1.1,
            },
            warn_position_gap_m=1.0,
            warn_heading_gap_rad=0.35,
        )
        == "global_pose"
    )


def test_build_report_exposes_breach_summary() -> None:
    report = build_report(
        result=_build_result(
            max_position_gap_m=4.2,
            max_position_pair="global_vs_local",
            max_heading_gap_rad=0.9,
            max_heading_pair="global_vs_local",
            stopped_early=True,
        ),
        duration_s=20.0,
        warn_position_gap_m=1.0,
        fail_position_gap_m=3.0,
        warn_heading_gap_rad=0.35,
        fail_heading_gap_rad=0.8,
    )

    assert report["ok"] is False
    assert report["level"] == "fail"
    assert report["stopped_early"] is True
    assert report["summary"]["max_position_pair"] == "global_vs_local"
    assert report["summary"]["max_source_age_s"] == 0.12
    assert report["summary"]["max_stamp_delta_pair"] == "global_vs_local"
    assert report["summary"]["core_position_pair"] == "global_vs_local"
    assert report["summary"]["core_heading_pair"] == "global_vs_local"
    assert report["summary"]["threshold_breach_pair"] == "global_vs_local"
