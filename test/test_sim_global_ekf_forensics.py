import sys

from navegacion_gps import sim_global_ekf_forensics


def _sample(
    *,
    elapsed_s: float,
    map_odom_xy,
    map_odom_yaw_rad: float,
    global_xy,
    local_xy,
    gps_odom_xy,
    gps_xy,
    base_xy,
    global_heading: float = 0.0,
    local_heading: float = 0.0,
    gps_odom_heading: float = 0.0,
    base_heading: float = 0.0,
    imu_yaw: float = 0.0,
    imu_angular_velocity_z: float = 0.0,
    global_stamp: float = 1.0,
    local_stamp: float = 1.0,
    gps_odom_stamp: float = 1.0,
    gps_stamp: float = 1.0,
    base_stamp: float = 1.0,
    source_age: float = 0.0,
    wall_time: float | None = None,
):
    global_vs_local_m = sim_global_ekf_forensics._distance_xy(tuple(global_xy), tuple(local_xy))
    global_vs_local_yaw = sim_global_ekf_forensics._angle_delta(global_heading, local_heading)
    return {
        "elapsed_s": elapsed_s,
        "phase": "post_goal",
        "wall_time_monotonic_s": elapsed_s if wall_time is None else wall_time,
        "ros_now_s": elapsed_s,
        "map_odom_xy": list(map_odom_xy),
        "map_odom_yaw_rad": map_odom_yaw_rad,
        "positions_map_xy": {
            "global": list(global_xy),
            "local": list(local_xy),
            "gps_odom": list(gps_odom_xy),
            "gps": list(gps_xy),
            "base": list(base_xy),
        },
        "headings_map_rad": {
            "global": global_heading,
            "local": local_heading,
            "base": base_heading,
        },
        "position_gaps_m": {
            "global_vs_local": global_vs_local_m,
        },
        "heading_gaps_rad": {
            "global_vs_local": global_vs_local_yaw,
        },
        "source_stamps_s": {
            "global": global_stamp,
            "local": local_stamp,
            "gps_odom": gps_odom_stamp,
            "gps": gps_stamp,
            "base": base_stamp,
        },
        "source_ages_s": {
            "global": source_age,
            "local": source_age,
            "gps_odom": source_age,
            "gps": source_age,
            "base": 0.0,
        },
        "filtered_covariance_diag": {"x": 0.1, "y": 0.1, "yaw": 0.1},
        "local_covariance_diag": {"x": 0.1, "y": 0.1, "yaw": 0.1},
        "gps_odom_covariance_diag": {"x": 0.1, "y": 0.1, "yaw": 0.1},
        "gps_odom_yaw_rad": gps_odom_heading,
        "cmd_vel_nav": {"linear_x": 1.0, "angular_z": 0.2},
        "cmd_vel_final": {"linear_x": 1.0, "angular_z": 0.2, "brake_pct": 0},
        "drive_telemetry": {
            "ready": True,
            "fresh": True,
            "drive_enabled": True,
            "estop": False,
            "reverse_requested": False,
            "speed_valid": True,
            "steer_valid": True,
            "control_source": "PI",
            "speed_mps_measured": 1.0,
            "steer_deg_measured": 31.0,
            "brake_applied_pct": 0,
        },
        "plan_summary": {"pose_count": 10, "sampled_length_m": 5.0, "mean_curvature": 0.2, "max_curvature": 0.5},
        "imu_data": {"yaw_rad": imu_yaw, "angular_velocity_z": imu_angular_velocity_z},
    }


def test_classify_jump_event_detects_map_to_odom_tf_jump() -> None:
    metrics = {
        "map_odom_translation_delta_m": 4.0,
        "map_odom_yaw_delta_rad": 0.3,
        "filtered_delta_m": 0.4,
        "filtered_yaw_delta_rad": 0.05,
        "local_delta_m": 0.1,
        "gps_odom_delta_m": 0.1,
        "gps_fix_delta_m": 0.1,
        "base_delta_m": 0.1,
        "global_stamp_delta_s": 0.1,
        "global_stamp_delta_signed_s": 0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.1,
        "largest_global_age_s": 0.1,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.2,
        "tf_vs_filtered_disagreement_after_m": 1.0,
        "tf_vs_filtered_disagreement_delta_m": 0.8,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.3,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.25,
        "tf_jump_m": 4.0,
        "filtered_jump_m": 0.4,
        "tf_vs_filtered_delta_m": 3.6,
        "tf_vs_filtered_delta_yaw_rad": 0.25,
        "map_odom_speed_mps": 20.0,
        "filtered_speed_mps": 2.0,
        "map_odom_yaw_rate_rps": 1.5,
        "filtered_yaw_rate_rps": 0.2,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": False,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "map_to_odom_tf_jump"
    assert result["dominant_error_mode"] == "map_to_odom_tf_jump"


def test_classify_jump_event_detects_filtered_state_drift() -> None:
    metrics = {
        "map_odom_translation_delta_m": 0.5,
        "map_odom_yaw_delta_rad": 0.05,
        "filtered_delta_m": 2.5,
        "filtered_yaw_delta_rad": 0.3,
        "local_delta_m": 0.1,
        "gps_odom_delta_m": 0.1,
        "gps_fix_delta_m": 0.1,
        "base_delta_m": 0.1,
        "global_stamp_delta_s": 0.1,
        "global_stamp_delta_signed_s": 0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.1,
        "largest_global_age_s": 0.1,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.2,
        "tf_vs_filtered_disagreement_after_m": 0.3,
        "tf_vs_filtered_disagreement_delta_m": 0.1,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.06,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.01,
        "tf_jump_m": 0.5,
        "filtered_jump_m": 2.5,
        "tf_vs_filtered_delta_m": 2.0,
        "tf_vs_filtered_delta_yaw_rad": 0.25,
        "map_odom_speed_mps": 2.5,
        "filtered_speed_mps": 12.5,
        "map_odom_yaw_rate_rps": 0.2,
        "filtered_yaw_rate_rps": 1.5,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": False,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "filtered_state_drift"
    assert result["dominant_source"] == "filtered_state"


def test_classify_jump_event_detects_combined_global_correction() -> None:
    metrics = {
        "map_odom_translation_delta_m": 3.5,
        "map_odom_yaw_delta_rad": 0.4,
        "filtered_delta_m": 2.0,
        "filtered_yaw_delta_rad": 0.3,
        "local_delta_m": 0.2,
        "gps_odom_delta_m": 0.2,
        "gps_fix_delta_m": 0.2,
        "base_delta_m": 0.2,
        "global_stamp_delta_s": 0.1,
        "global_stamp_delta_signed_s": 0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.1,
        "largest_global_age_s": 0.1,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.3,
        "tf_vs_filtered_disagreement_after_m": 0.8,
        "tf_vs_filtered_disagreement_delta_m": 0.5,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.2,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.15,
        "tf_jump_m": 3.5,
        "filtered_jump_m": 2.0,
        "tf_vs_filtered_delta_m": 1.5,
        "tf_vs_filtered_delta_yaw_rad": 0.1,
        "map_odom_speed_mps": 17.5,
        "filtered_speed_mps": 10.0,
        "map_odom_yaw_rate_rps": 2.0,
        "filtered_yaw_rate_rps": 1.5,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": False,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "combined_global_correction"


def test_summarize_post_goal_drift_detects_global_reference_drift() -> None:
    samples = [
        _sample(
            elapsed_s=10.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(12.0, 8.0),
            local_xy=(12.0, 8.0),
            gps_odom_xy=(12.0, 8.0),
            gps_xy=(12.0, 8.0),
            base_xy=(12.0, 8.0),
        ),
        _sample(
            elapsed_s=15.0,
            map_odom_xy=(0.9, 0.1),
            map_odom_yaw_rad=0.2,
            global_xy=(12.8, 8.1),
            local_xy=(12.05, 8.02),
            gps_odom_xy=(12.04, 8.01),
            gps_xy=(12.04, 8.01),
            base_xy=(12.03, 8.01),
            global_heading=0.2,
            local_heading=0.02,
            gps_odom_heading=0.0,
            base_heading=0.02,
        ),
    ]

    summary = sim_global_ekf_forensics.summarize_post_goal_drift(samples)

    assert summary["available"] is True
    assert summary["classification"] == "global_reference_drift"
    assert summary["map_odom_translation_drift_m"] > 0.5
    assert summary["max_local_excursion_m"] < 0.25


def test_summarize_post_goal_drift_detects_real_robot_motion() -> None:
    samples = [
        _sample(
            elapsed_s=10.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(12.0, 8.0),
            local_xy=(12.0, 8.0),
            gps_odom_xy=(12.0, 8.0),
            gps_xy=(12.0, 8.0),
            base_xy=(12.0, 8.0),
        ),
        _sample(
            elapsed_s=15.0,
            map_odom_xy=(0.05, 0.0),
            map_odom_yaw_rad=0.01,
            global_xy=(12.55, 8.0),
            local_xy=(12.6, 8.0),
            gps_odom_xy=(12.58, 8.0),
            gps_xy=(12.58, 8.0),
            base_xy=(12.65, 8.0),
            global_heading=0.03,
            local_heading=0.03,
            gps_odom_heading=0.0,
            base_heading=0.03,
        ),
    ]

    summary = sim_global_ekf_forensics.summarize_post_goal_drift(samples)

    assert summary["classification"] == "real_robot_motion_after_goal"
    assert summary["max_base_excursion_m"] > 0.5


def test_summarize_goal_proximity_detects_reached_in_local_but_not_global() -> None:
    goal = (12.0, 8.0)
    samples = [
        _sample(
            elapsed_s=1.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(13.5, 8.0),
            local_xy=(12.7, 8.0),
            gps_odom_xy=(12.8, 8.0),
            gps_xy=(12.8, 8.0),
            base_xy=(12.6, 8.0),
        ),
        _sample(
            elapsed_s=2.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(12.95, 8.0),
            local_xy=(12.2, 8.0),
            gps_odom_xy=(12.25, 8.0),
            gps_xy=(12.25, 8.0),
            base_xy=(12.1, 8.0),
        ),
    ]
    summary = sim_global_ekf_forensics.summarize_goal_proximity(samples, goal_xy=goal, xy_goal_tolerance_m=0.8)
    assert summary["classification"] == "reached_in_local_but_not_global"
    assert summary["min_distance_by_frame_m"]["local"] <= 0.8
    assert summary["min_distance_by_frame_m"]["global"] > 0.8


def test_summarize_goal_proximity_detects_entered_goal_then_moved_away() -> None:
    goal = (12.0, 8.0)
    samples = [
        _sample(
            elapsed_s=1.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(12.2, 8.0),
            local_xy=(12.2, 8.0),
            gps_odom_xy=(12.2, 8.0),
            gps_xy=(12.2, 8.0),
            base_xy=(12.2, 8.0),
        ),
        _sample(
            elapsed_s=2.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(13.4, 8.0),
            local_xy=(13.4, 8.0),
            gps_odom_xy=(13.4, 8.0),
            gps_xy=(13.4, 8.0),
            base_xy=(13.4, 8.0),
        ),
    ]
    summary = sim_global_ekf_forensics.summarize_goal_proximity(samples, goal_xy=goal, xy_goal_tolerance_m=0.8)
    assert summary["classification"] == "entered_goal_then_moved_away"
    assert summary["min_distance_by_frame_m"]["global"] <= 0.8
    assert summary["final_distance_by_frame_m"]["global"] > 0.8


def test_summarize_path_tracking_reports_maxima() -> None:
    samples = [
        {
            "path_tracking": {
                "global": {"distance_to_path_m": 0.5, "heading_error_rad": 0.1},
                "local": {"distance_to_path_m": 0.4, "heading_error_rad": 0.2},
                "base": {"distance_to_path_m": 0.6, "heading_error_rad": 0.15},
            }
        },
        {
            "path_tracking": {
                "global": {"distance_to_path_m": 1.2, "heading_error_rad": 0.3},
                "local": {"distance_to_path_m": 1.5, "heading_error_rad": 0.4},
                "base": {"distance_to_path_m": 1.8, "heading_error_rad": 0.35},
            }
        },
    ]
    summary = sim_global_ekf_forensics.summarize_path_tracking(samples)
    assert summary["available"] is True
    assert summary["max_distance_to_path_m"]["global"] == 1.2
    assert summary["max_distance_to_path_m"]["local"] == 1.5
    assert summary["max_distance_to_path_m"]["base"] == 1.8
    assert summary["max_heading_error_rad"]["local"] == 0.4


def test_classify_jump_event_detects_gps_driven_jump() -> None:
    metrics = {
        "map_odom_translation_delta_m": 4.0,
        "map_odom_yaw_delta_rad": 0.1,
        "filtered_delta_m": 1.0,
        "filtered_yaw_delta_rad": 0.1,
        "local_delta_m": 0.1,
        "gps_odom_delta_m": 2.5,
        "gps_fix_delta_m": 2.5,
        "base_delta_m": 0.1,
        "global_stamp_delta_s": 0.1,
        "global_stamp_delta_signed_s": 0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.1,
        "largest_global_age_s": 0.1,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.2,
        "tf_vs_filtered_disagreement_after_m": 0.8,
        "tf_vs_filtered_disagreement_delta_m": 0.6,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.1,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.05,
        "tf_jump_m": 4.0,
        "filtered_jump_m": 1.0,
        "tf_vs_filtered_delta_m": 3.0,
        "tf_vs_filtered_delta_yaw_rad": 0.0,
        "map_odom_speed_mps": 20.0,
        "filtered_speed_mps": 5.0,
        "map_odom_yaw_rate_rps": 0.5,
        "filtered_yaw_rate_rps": 0.5,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": False,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "gps_odom_driven_jump"


def test_classify_jump_event_detects_local_driven_jump() -> None:
    metrics = {
        "map_odom_translation_delta_m": 4.0,
        "map_odom_yaw_delta_rad": 0.1,
        "filtered_delta_m": 1.0,
        "filtered_yaw_delta_rad": 0.1,
        "local_delta_m": 2.3,
        "gps_odom_delta_m": 0.1,
        "gps_fix_delta_m": 0.1,
        "base_delta_m": 2.4,
        "global_stamp_delta_s": 0.1,
        "global_stamp_delta_signed_s": 0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.1,
        "largest_global_age_s": 0.1,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.2,
        "tf_vs_filtered_disagreement_after_m": 0.8,
        "tf_vs_filtered_disagreement_delta_m": 0.6,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.1,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.05,
        "tf_jump_m": 4.0,
        "filtered_jump_m": 1.0,
        "tf_vs_filtered_delta_m": 3.0,
        "tf_vs_filtered_delta_yaw_rad": 0.0,
        "map_odom_speed_mps": 20.0,
        "filtered_speed_mps": 5.0,
        "map_odom_yaw_rate_rps": 0.5,
        "filtered_yaw_rate_rps": 0.5,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": False,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "local_odom_driven_jump"


def test_classify_jump_event_detects_lag() -> None:
    metrics = {
        "map_odom_translation_delta_m": 4.0,
        "map_odom_yaw_delta_rad": 0.1,
        "filtered_delta_m": 1.0,
        "filtered_yaw_delta_rad": 0.1,
        "local_delta_m": 0.1,
        "gps_odom_delta_m": 0.1,
        "gps_fix_delta_m": 0.1,
        "base_delta_m": 0.1,
        "global_stamp_delta_s": 0.3,
        "global_stamp_delta_signed_s": -0.1,
        "local_stamp_delta_s": 0.1,
        "local_stamp_delta_signed_s": 0.1,
        "gps_odom_stamp_delta_s": 0.1,
        "gps_odom_stamp_delta_signed_s": 0.1,
        "gps_stamp_delta_s": 0.1,
        "gps_stamp_delta_signed_s": 0.1,
        "base_stamp_delta_s": 0.1,
        "base_stamp_delta_signed_s": 0.1,
        "largest_source_age_s": 0.3,
        "largest_global_age_s": 0.3,
        "largest_gps_odom_age_s": 0.1,
        "largest_local_age_s": 0.1,
        "largest_gps_age_s": 0.1,
        "largest_base_age_s": 0.0,
        "tf_vs_filtered_disagreement_before_m": 0.2,
        "tf_vs_filtered_disagreement_after_m": 0.8,
        "tf_vs_filtered_disagreement_delta_m": 0.6,
        "tf_vs_filtered_disagreement_before_yaw_rad": 0.05,
        "tf_vs_filtered_disagreement_after_yaw_rad": 0.1,
        "tf_vs_filtered_disagreement_delta_yaw_rad": 0.05,
        "tf_jump_m": 4.0,
        "filtered_jump_m": 1.0,
        "tf_vs_filtered_delta_m": 3.0,
        "tf_vs_filtered_delta_yaw_rad": 0.0,
        "map_odom_speed_mps": 20.0,
        "filtered_speed_mps": 5.0,
        "map_odom_yaw_rate_rps": 0.5,
        "filtered_yaw_rate_rps": 0.5,
        "wall_dt_s": 0.2,
        "source_age_vector": {},
        "source_stamp_vector": {},
        "has_out_of_order_source": True,
    }
    result = sim_global_ekf_forensics.classify_jump_event(
        metrics,
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
    )
    assert result["classification"] == "lag_or_reorder_suspected"


def test_analyze_ekf_forensics_builds_forensic_summary_and_windows() -> None:
    samples = [
        _sample(
            elapsed_s=0.0,
            map_odom_xy=(0.0, 0.0),
            map_odom_yaw_rad=0.0,
            global_xy=(1.0, 1.0),
            local_xy=(1.0, 1.0),
            gps_odom_xy=(1.05, 1.0),
            gps_xy=(1.05, 1.0),
            base_xy=(1.0, 1.0),
            gps_odom_heading=0.0,
            imu_yaw=0.0,
        ),
        _sample(
            elapsed_s=0.05,
            map_odom_xy=(3.0, 0.0),
            map_odom_yaw_rad=0.4,
            global_xy=(1.2, 1.0),
            local_xy=(1.8, 1.0),
            gps_odom_xy=(1.1, 1.0),
            gps_xy=(1.1, 1.0),
            base_xy=(1.1, 1.0),
            gps_odom_heading=0.6,
            imu_yaw=0.55,
            imu_angular_velocity_z=0.3,
            global_stamp=1.05,
            local_stamp=1.05,
            gps_odom_stamp=1.05,
        ),
    ]
    analysis = sim_global_ekf_forensics.analyze_ekf_forensics(
        samples,
        goal_status="TIMEOUT",
        goal_xy=(12.0, 8.0),
        jump_translation_threshold_m=2.0,
        jump_yaw_threshold_rad=0.25,
        window_radius=3,
    )
    assert analysis["summary"]["goal_status"] == "TIMEOUT"
    assert analysis["summary"]["jump_count"] == 1
    assert analysis["summary"]["dominant_error_mode"] == "map_to_odom_tf_jump"
    assert "largest_tf_filtered_disagreement_m" in analysis["summary"]
    assert "largest_local_vs_gps_odom_yaw_gap_rad" in analysis["summary"]
    assert analysis["summary"]["jump_events_with_steering_saturation"] == 1
    event = analysis["jump_events"][0]
    assert event["classification"] == "map_to_odom_tf_jump"
    assert event["confidence"] == "high"
    assert event["control_context"]["measured_steer_saturated"] is True
    assert event["heading_source_context"]["local_vs_gps_odom_yaw_gap_rad"] > 0.5
    assert "pre_window" in event
    assert "post_window" in event


def test_parse_args_defaults() -> None:
    old = sys.argv
    try:
        sys.argv = ["sim_global_ekf_forensics"]
        args = sim_global_ekf_forensics._parse_args()
    finally:
        sys.argv = old

    assert args.profile == "strict_split_lag_compensated"
    assert args.observe_phase == "full_run"
    assert args.use_rviz is True
    assert args.sample_interval_s == 0.05
