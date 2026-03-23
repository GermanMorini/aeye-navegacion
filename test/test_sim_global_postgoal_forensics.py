import sys

from navegacion_gps import sim_global_postgoal_forensics


def test_analyze_postgoal_forensics_detects_global_internal_jump() -> None:
    samples = [
        {
            "elapsed_s": 0.0,
            "map_odom_xy": [0.0, 0.0],
            "map_odom_yaw_rad": 0.0,
            "positions_map_xy": {
                "global": [10.0, 10.0],
                "local": [1.0, 1.0],
                "gps_odom": [1.1, 1.0],
                "gps": [1.1, 1.0],
                "base": [1.0, 1.0],
            },
            "source_stamps_s": {"global": 1.0, "local": 1.0, "gps_odom": 1.0},
            "source_ages_s": {"global": 0.0, "local": 0.0, "gps_odom": 0.0},
        },
        {
            "elapsed_s": 0.25,
            "map_odom_xy": [3.0, 0.0],
            "map_odom_yaw_rad": 0.4,
            "positions_map_xy": {
                "global": [13.5, 10.0],
                "local": [1.1, 1.0],
                "gps_odom": [1.2, 1.0],
                "gps": [1.2, 1.0],
                "base": [1.1, 1.0],
            },
            "source_stamps_s": {"global": 1.25, "local": 1.25, "gps_odom": 1.02},
            "source_ages_s": {"global": 0.0, "local": 0.0, "gps_odom": 0.23},
        },
    ]

    analysis = sim_global_postgoal_forensics.analyze_postgoal_forensics(
        samples,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )

    assert analysis["jump_count"] == 1
    assert analysis["dominant_source"] == "ekf_global_map_to_odom_jump"


def test_analyze_postgoal_forensics_detects_gps_odom_jump() -> None:
    samples = [
        {
            "elapsed_s": 0.0,
            "map_odom_xy": [0.0, 0.0],
            "map_odom_yaw_rad": 0.0,
            "positions_map_xy": {
                "global": [10.0, 10.0],
                "local": [1.0, 1.0],
                "gps_odom": [1.0, 1.0],
                "gps": [1.0, 1.0],
                "base": [1.0, 1.0],
            },
            "source_stamps_s": {"global": 1.0, "local": 1.0, "gps_odom": 1.0},
            "source_ages_s": {"global": 0.0, "local": 0.0, "gps_odom": 0.0},
        },
        {
            "elapsed_s": 0.25,
            "map_odom_xy": [3.0, 0.0],
            "map_odom_yaw_rad": 0.0,
            "positions_map_xy": {
                "global": [13.0, 10.0],
                "local": [1.1, 1.0],
                "gps_odom": [4.5, 1.0],
                "gps": [4.5, 1.0],
                "base": [1.1, 1.0],
            },
            "source_stamps_s": {"global": 1.25, "local": 1.25, "gps_odom": 1.24},
            "source_ages_s": {"global": 0.0, "local": 0.0, "gps_odom": 0.01},
        },
    ]

    analysis = sim_global_postgoal_forensics.analyze_postgoal_forensics(
        samples,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )

    assert analysis["jump_count"] == 1
    assert analysis["dominant_source"] == "navsat_or_gps_odom_jump"


def test_parse_args_defaults() -> None:
    argv = ["sim_global_postgoal_forensics"]
    old = sys.argv
    try:
        sys.argv = argv
        args = sim_global_postgoal_forensics._parse_args()
    finally:
        sys.argv = old

    assert args.profile == "strict_split_lag_compensated"
    assert args.use_rviz is True
