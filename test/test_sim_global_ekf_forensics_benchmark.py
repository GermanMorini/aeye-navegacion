import sys

from navegacion_gps import sim_global_ekf_forensics_benchmark


def test_benchmark_defaults_include_three_profiles() -> None:
    old = sys.argv
    try:
        sys.argv = ["sim_global_ekf_forensics_benchmark"]
        args = sim_global_ekf_forensics_benchmark._parse_args()
    finally:
        sys.argv = old

    assert args.profiles == []
    assert args.use_rviz is True


def test_score_prefers_status_then_disagreement() -> None:
    good = {
        "goal_result": {"goal_status": "SUCCEEDED"},
        "summary": {
            "largest_tf_filtered_disagreement_m": 1.0,
            "largest_map_odom_jump_m": 2.0,
            "largest_stamp_gap_s": 0.1,
        },
    }
    bad = {
        "goal_result": {"goal_status": "TIMEOUT"},
        "summary": {
            "largest_tf_filtered_disagreement_m": 0.5,
            "largest_map_odom_jump_m": 1.0,
            "largest_stamp_gap_s": 0.05,
        },
    }
    assert sim_global_ekf_forensics_benchmark._score_result(good) < sim_global_ekf_forensics_benchmark._score_result(bad)
