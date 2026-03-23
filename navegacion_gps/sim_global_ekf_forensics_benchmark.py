from __future__ import annotations

import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

from navegacion_gps.sim_global_ekf_forensics import DEFAULT_GOAL
from navegacion_gps.sim_global_ekf_forensics import ForensicsRunConfig
from navegacion_gps.sim_global_ekf_forensics import run_forensics_session


DEFAULT_PROFILES = (
    "strict_split_lag_compensated",
    "strict_split_twist_global",
    "strict_split_rate_matched",
)
DEFAULT_FOLLOWUP_GOAL = (15.0, 10.0)


def _parse_goal(text: str) -> tuple[float, float]:
    x_text, y_text = [part.strip() for part in text.split(",", 1)]
    return float(x_text), float(y_text)


def _score_result(result: dict[str, Any]) -> tuple[int, float, float, float]:
    goal_status = str(result["goal_result"]["goal_status"])
    summary = result["summary"]
    status_rank = {
        "SUCCEEDED": 0,
        "CANCELED": 1,
        "TIMEOUT": 2,
        "ABORTED": 3,
        "REJECTED": 4,
    }.get(goal_status, 5)
    return (
        status_rank,
        float(summary["largest_tf_filtered_disagreement_m"]),
        float(summary["largest_map_odom_jump_m"]),
        float(summary["largest_stamp_gap_s"]),
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Automatic forensic benchmark for sim_global_v2 using the high-precision EKF tool.",
    )
    parser.add_argument(
        "--profile",
        action="append",
        dest="profiles",
        default=[],
        help="Repeatable profile selector. Defaults to the three comparison profiles.",
    )
    parser.add_argument("--goal", default=f"{DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]}")
    parser.add_argument(
        "--followup-goal",
        default=f"{DEFAULT_FOLLOWUP_GOAL[0]},{DEFAULT_FOLLOWUP_GOAL[1]}",
    )
    parser.add_argument("--bootstrap-timeout-s", type=float, default=45.0)
    parser.add_argument("--goal-timeout-s", type=float, default=35.0)
    parser.add_argument("--post-bootstrap-settle-s", type=float, default=5.0)
    parser.add_argument("--post-goal-duration-s", type=float, default=8.0)
    parser.add_argument("--sample-interval-s", type=float, default=0.05)
    parser.add_argument("--jump-translation-threshold-m", type=float, default=2.0)
    parser.add_argument("--jump-yaw-threshold-rad", type=float, default=0.25)
    parser.add_argument("--window-radius", type=int, default=6)
    parser.add_argument(
        "--use-rviz",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    profiles = tuple(args.profiles) if args.profiles else DEFAULT_PROFILES
    primary_goal = _parse_goal(args.goal)
    followup_goal = _parse_goal(args.followup_goal)

    results: list[dict[str, Any]] = []
    with TemporaryDirectory(prefix="sim_global_ekf_forensics_") as tmp_dir:
        temp_root = Path(tmp_dir)
        for profile in profiles:
            primary_output = temp_root / f"{profile}_primary.json"
            primary_result, _ = run_forensics_session(
                ForensicsRunConfig(
                    profile=profile,
                    goal_xy=primary_goal,
                    bootstrap_timeout_s=float(args.bootstrap_timeout_s),
                    goal_timeout_s=float(args.goal_timeout_s),
                    post_bootstrap_settle_s=float(args.post_bootstrap_settle_s),
                    observe_phase="full_run",
                    post_goal_duration_s=float(args.post_goal_duration_s),
                        sample_interval_s=float(args.sample_interval_s),
                        jump_translation_threshold_m=float(args.jump_translation_threshold_m),
                        jump_yaw_threshold_rad=float(args.jump_yaw_threshold_rad),
                        window_radius=int(args.window_radius),
                        use_rviz=bool(args.use_rviz),
                        output=str(primary_output),
                    )
            )
            entry: dict[str, Any] = {
                "profile": profile,
                "primary_goal": list(primary_goal),
                "primary_result": primary_result,
            }
            primary_status = str(primary_result["goal_result"]["goal_status"])
            if primary_status not in {"ABORTED", "REJECTED"}:
                followup_output = temp_root / f"{profile}_followup.json"
                followup_result, _ = run_forensics_session(
                    ForensicsRunConfig(
                        profile=profile,
                        goal_xy=followup_goal,
                        bootstrap_timeout_s=float(args.bootstrap_timeout_s),
                        goal_timeout_s=float(args.goal_timeout_s),
                        post_bootstrap_settle_s=float(args.post_bootstrap_settle_s),
                        observe_phase="full_run",
                        post_goal_duration_s=float(args.post_goal_duration_s),
                        sample_interval_s=float(args.sample_interval_s),
                        jump_translation_threshold_m=float(args.jump_translation_threshold_m),
                        jump_yaw_threshold_rad=float(args.jump_yaw_threshold_rad),
                        window_radius=int(args.window_radius),
                        use_rviz=bool(args.use_rviz),
                        output=str(followup_output),
                    )
                )
                entry["followup_goal"] = list(followup_goal)
                entry["followup_result"] = followup_result
            results.append(entry)

    comparison = []
    for entry in results:
        primary_summary = entry["primary_result"]["summary"]
        item = {
            "profile": entry["profile"],
            "goal_status": entry["primary_result"]["goal_result"]["goal_status"],
            "dominant_source": primary_summary["dominant_source"],
            "dominant_error_mode": primary_summary["dominant_error_mode"],
            "largest_map_odom_jump_m": primary_summary["largest_map_odom_jump_m"],
            "largest_filtered_jump_m": primary_summary["largest_filtered_jump_m"],
            "largest_tf_filtered_disagreement_m": primary_summary["largest_tf_filtered_disagreement_m"],
            "largest_stamp_gap_s": primary_summary["largest_stamp_gap_s"],
        }
        if "followup_result" in entry:
            item["followup_goal_status"] = entry["followup_result"]["goal_result"]["goal_status"]
        comparison.append(item)

    winner = min(results, key=lambda item: _score_result(item["primary_result"]))["profile"] if results else ""
    payload = {
        "profiles": list(profiles),
        "primary_goal": list(primary_goal),
        "followup_goal": list(followup_goal),
        "results": results,
        "comparison": comparison,
        "winner": winner,
    }
    text = json.dumps(payload, indent=2, sort_keys=True)
    print(text)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
