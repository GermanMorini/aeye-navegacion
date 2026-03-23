import sys
from pathlib import Path

from navegacion_gps import sim_global_motion_benchmark
from navegacion_gps.sim_global_motion_benchmark import PROFILE_TO_CONFIG


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_benchmark_profiles_cover_expected_variants() -> None:
    assert set(PROFILE_TO_CONFIG) == {
        "operational_default",
        "baseline_current",
        "motion_candidate_navsat_decoupled",
        "motion_candidate_local_vyaw_fallback",
        "motion_candidate_heading_only",
        "motion_candidate_no_lateral_velocity",
        "motion_candidate_local_yaw_anchor",
        "motion_candidate_yaw_anchor_gps_position",
        "motion_candidate_gps_position_hard_lock",
        "motion_candidate_lag_compensated",
        "rejected_reference_linear_only",
        "strict_split_twist_global",
        "strict_split_heading_only_global",
        "strict_split_lag_compensated",
        "strict_split_lag_compensated_odom_yaw",
        "strict_split_lag_compensated_odom_yaw_no_delay",
        "strict_split_lag_compensated_odom_yaw_no_delay_soft_gps",
        "strict_split_rate_matched",
        "heading_from_local_odom",
        "heading_from_imu",
        "heading_decoupled_minimal",
        "ideal_from_local_odom",
        "ideal_from_raw_odom",
        "ideal_from_ground_truth",
    }


def test_benchmark_profile_files_exist() -> None:
    for profile in PROFILE_TO_CONFIG.values():
        assert (PACKAGE_ROOT / "config" / profile.config_filename).is_file()


def test_benchmark_parser_exposes_post_result_grace(monkeypatch) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["sim_global_motion_benchmark", "--post-result-grace-s", "3.5"],
    )
    args = sim_global_motion_benchmark._parse_args()
    assert args.post_result_grace_s == 3.5


def test_benchmark_parser_enables_rviz_by_default(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", ["sim_global_motion_benchmark"])
    args = sim_global_motion_benchmark._parse_args()
    assert args.use_rviz is True


def test_snapshot_delta_separates_map_jump_from_odom_motion() -> None:
    delta = sim_global_motion_benchmark._snapshot_delta(
        {
            "global_pose_xy": [2.0, 1.0],
            "global_pose_xy_in_odom": [4.0, 5.0],
            "local_pose_xy": [4.0, 5.0],
        },
        {
            "global_pose_xy": [-4.0, 2.0],
            "global_pose_xy_in_odom": [4.1, 5.1],
            "local_pose_xy": [4.1, 5.1],
        },
    )
    assert delta["global_pose_map_jump_m"] > 6.0
    assert delta["global_pose_odom_jump_m"] < 0.2
    assert delta["local_pose_odom_jump_m"] < 0.2
