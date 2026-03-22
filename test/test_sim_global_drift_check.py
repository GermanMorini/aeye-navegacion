from navegacion_gps.sim_global_drift_check import build_report
from navegacion_gps.sim_global_drift_check import classify_drift_level
from navegacion_gps.sim_global_drift_check import infer_likely_origin


def _build_result(*, map_odom: float, navsat: float) -> dict:
    return {
        "drift_m": {
            "map_odom": map_odom,
            "map_base": 0.0,
            "odom_base": 0.0,
            "fromll_map": 0.0,
            "fromll_odom": 0.0,
            "odom_gps": navsat,
        },
        "drift_attribution": {
            "navsat_transform_drift_m": navsat,
            "global_fusion_drift_m": map_odom,
            "fusion_added_drift_m": max(0.0, map_odom - navsat),
            "likely_origin": infer_likely_origin(
                navsat_transform_drift_m=navsat,
                global_fusion_drift_m=map_odom,
            ),
        },
    }


def test_classify_drift_level_uses_warn_and_fail_thresholds() -> None:
    assert (
        classify_drift_level(
            map_odom_drift_m=0.2,
            warn_threshold_m=0.5,
            fail_threshold_m=1.5,
        )
        == "ok"
    )
    assert (
        classify_drift_level(
            map_odom_drift_m=0.8,
            warn_threshold_m=0.5,
            fail_threshold_m=1.5,
        )
        == "warn"
    )
    assert (
        classify_drift_level(
            map_odom_drift_m=1.8,
            warn_threshold_m=0.5,
            fail_threshold_m=1.5,
        )
        == "fail"
    )


def test_infer_likely_origin_marks_global_fusion_when_it_dominates() -> None:
    assert (
        infer_likely_origin(navsat_transform_drift_m=0.3, global_fusion_drift_m=2.1)
        == "fusion_with_odometry_local"
    )


def test_build_report_exposes_summary_and_level() -> None:
    report = build_report(
        result=_build_result(map_odom=0.9, navsat=0.2),
        duration_s=20.0,
        warn_threshold_m=0.5,
        fail_threshold_m=1.5,
    )

    assert report["ok"] is True
    assert report["level"] == "warn"
    assert report["summary"]["map_odom_drift_m"] == 0.9
    assert report["summary"]["likely_origin"] == "mixed_or_inconclusive"
