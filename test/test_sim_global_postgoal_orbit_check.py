import sys

from navegacion_gps import sim_global_postgoal_orbit_check


def test_build_postgoal_report_detects_orbit() -> None:
    samples = [
        {
            "elapsed_s": 0.0,
            "map_odom_xy": [0.0, 0.0],
            "map_odom_yaw_rad": 0.0,
            "local_xy": [1.0, 1.0],
            "global_xy": [1.0, 1.0],
            "odom_origin_in_base_xy": [0.0, 0.0],
        },
        {
            "elapsed_s": 5.0,
            "map_odom_xy": [1.2, 0.0],
            "map_odom_yaw_rad": 0.4,
            "local_xy": [1.1, 1.0],
            "global_xy": [1.0, 1.0],
            "odom_origin_in_base_xy": [1.0, 0.0],
        },
    ]

    report = sim_global_postgoal_orbit_check.build_postgoal_report(
        samples,
        goal_status="SUCCEEDED",
    )

    assert report["orbit_detected"] is True
    assert report["likely_issue"] == "map_odom_orbit_after_goal"
    assert report["max_map_odom_translation_drift_m"] > 1.0


def test_build_postgoal_report_allows_stable_post_goal() -> None:
    samples = [
        {
            "elapsed_s": 0.0,
            "map_odom_xy": [0.0, 0.0],
            "map_odom_yaw_rad": 0.0,
            "local_xy": [1.0, 1.0],
            "global_xy": [1.0, 1.0],
            "odom_origin_in_base_xy": [0.0, 0.0],
        },
        {
            "elapsed_s": 5.0,
            "map_odom_xy": [0.1, 0.0],
            "map_odom_yaw_rad": 0.05,
            "local_xy": [1.2, 1.0],
            "global_xy": [1.2, 1.0],
            "odom_origin_in_base_xy": [0.1, 0.0],
        },
    ]

    report = sim_global_postgoal_orbit_check.build_postgoal_report(
        samples,
        goal_status="SUCCEEDED",
    )

    assert report["orbit_detected"] is False
    assert report["likely_issue"] == "none_detected"


def test_parse_args_defaults_to_best_profile(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", ["sim_global_postgoal_orbit_check"])
    args = sim_global_postgoal_orbit_check._parse_args()

    assert args.profile == "strict_split_lag_compensated"
    assert args.use_rviz is True
