#!/usr/bin/env python3
# Task 3 health-check tool.
# Run with: python tools/check_run_health.py results/student_run_YYYYMMDD_HHMMSS.csv --profile student --require-complete
"""Fail-fast health check for a Webots baseline CSV log.

The check looks for continued progress, not only initial motion.  This catches
the release-2 failure in which the robot moved for about thirty seconds and
then remained in alternating emergency turns for the rest of the run.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def as_float(row: dict, key: str) -> float | None:
    # Student logs use `time`; baseline Supervisor logs use `time_s`.
    value = row.get(key, "")
    if value in ("", None) and key == "time_s":
        value = row.get("time", "")
    if value in ("", None, "None"):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def as_int(row: dict, key: str) -> int | None:
    value = as_float(row, key)
    return None if value is None else int(value)


def row_at_or_after(rows: list[dict], target_time: float) -> dict:
    for row in rows:
        if (as_float(row, "time_s") or 0.0) >= target_time:
            return row
    return rows[-1]


def evaluate(
    rows: list[dict],
    minimum_duration_s: float = 45.0,
    require_complete: bool = False,
    expected_waypoints: int = 23,
    expected_goals: int = 4,
    home: tuple[float, float] = (-3.2, -2.2),
    home_tolerance_m: float = 0.20,
    profile: str = "baseline",
) -> tuple[bool, list[str], dict]:
    # The student profile keeps mission checks but removes the 23-waypoint rule.
    if profile not in {"baseline", "student"}:
        raise ValueError("profile must be 'baseline' or 'student'")
    if not rows:
        return False, ["the CSV contains no data rows"], {}

    start_time = as_float(rows[0], "time_s") or 0.0
    end_time = as_float(rows[-1], "time_s") or start_time
    duration = max(0.0, end_time - start_time)
    path_length = as_float(rows[-1], "ground_truth_path_length_m") or 0.0
    contact_values = [int(row.get("contact_now", 0) or 0) for row in rows]
    contact_fraction = sum(contact_values) / len(contact_values)

    lidar_rows = 0
    for row in rows:
        point_count = as_float(row, "lidar_point_count")
        minimum_range = as_float(row, "minimum_lidar_range_m")
        if (point_count is not None and point_count > 0) or minimum_range is not None:
            lidar_rows += 1
    lidar_fraction = lidar_rows / len(rows)

    command_rows = 0
    moving_command_rows = 0
    for row in rows:
        left = as_float(row, "left_wheel_speed_rad_s")
        right = as_float(row, "right_wheel_speed_rad_s")
        if left is not None and right is not None:
            command_rows += 1
            if max(abs(left), abs(right)) > 0.1:
                moving_command_rows += 1
    moving_command_fraction = moving_command_rows / command_rows if command_rows else None

    window_duration = min(20.0, max(5.0, 0.40 * duration))
    window_start_time = end_time - window_duration
    window_start_row = row_at_or_after(rows, window_start_time)
    window_start_path = as_float(window_start_row, "ground_truth_path_length_m") or 0.0
    final_window_path_gain = max(0.0, path_length - window_start_path)
    final_window_rows = [
        row for row in rows if (as_float(row, "time_s") or 0.0) >= window_start_time
    ]
    emergency_rows = 0
    for row in final_window_rows:
        safety_state = str(row.get("safety_state", "")).strip().lower()
        linear = as_float(row, "commanded_linear_speed_m_s")
        angular = as_float(row, "commanded_angular_speed_rad_s")
        inferred_emergency = (
            not safety_state
            and linear is not None
            and angular is not None
            and abs(linear) < 0.005
            and abs(angular) >= 0.60
        )
        if safety_state == "emergency_turn" or inferred_emergency:
            emergency_rows += 1
    final_window_emergency_fraction = (
        emergency_rows / len(final_window_rows) if final_window_rows else 0.0
    )

    waypoint_values = [
        value for row in rows if (value := as_int(row, "active_waypoint")) is not None
    ]
    maximum_active_waypoint = max(waypoint_values) if waypoint_values else None
    # Accept either baseline Supervisor names or student logger names.
    mission_complete = bool(
        as_int(rows[-1], "mission_complete_reported")
        or as_int(rows[-1], "mission_complete")
        or 0
    )
    goals_reached = as_int(rows[-1], "goals_reached_ground_truth")
    if goals_reached is None:
        goals_reached = as_int(rows[-1], "goals_reached")
    collision_episodes = as_int(rows[-1], "collision_episodes")
    blocked_rows = sum(
        1
        for row in rows
        if str(row.get("avoidance_state", row.get("safety_state", ""))).lower() == "blocked"
    )
    final_x = as_float(rows[-1], "gt_x_m")
    final_y = as_float(rows[-1], "gt_y_m")
    final_home_error = (
        math.hypot(final_x - home[0], final_y - home[1])
        if final_x is not None and final_y is not None
        else None
    )
    if final_home_error is None:
        final_home_error = as_float(rows[-1], "home_error_m")
    position_errors = [
        value
        for row in rows
        if (value := as_float(row, "position_error_m")) is not None
    ]
    position_rmse = (
        math.sqrt(sum(value * value for value in position_errors) / len(position_errors))
        if position_errors
        else None
    )
    maximum_position_error = max(position_errors) if position_errors else None

    issues: list[str] = []
    if duration < minimum_duration_s:
        issues.append(f"run is only {duration:.1f} s; record at least {minimum_duration_s:.1f} s")
    # These motion, LiDAR, and wheel checks apply only to baseline logs.
    if profile == "baseline" and duration >= minimum_duration_s and path_length < 1.0:
        issues.append(f"ground-truth path is only {path_length:.3f} m; the robot is effectively stuck")
    if profile == "baseline" and duration > 0.0 and path_length / duration < 0.025 and not mission_complete:
        issues.append(
            f"average ground-truth speed is {path_length / duration:.4f} m/s; expected visible progress"
        )
    if profile == "baseline" and (
        duration >= minimum_duration_s
        and not mission_complete
        and final_window_path_gain < 0.15
    ):
        issues.append(
            f"ground-truth path increased only {final_window_path_gain:.3f} m "
            f"during the final {window_duration:.1f} s; the controller has stalled"
        )
    if profile == "baseline" and duration >= minimum_duration_s and final_window_emergency_fraction > 0.80:
        issues.append(
            "emergency turns occupy "
            f"{100.0 * final_window_emergency_fraction:.1f}% of the final "
            f"{window_duration:.1f} s"
        )
    if profile == "baseline" and (
        duration >= minimum_duration_s
        and maximum_active_waypoint is not None
        and maximum_active_waypoint < 2
    ):
        issues.append(
            f"active_waypoint never advanced beyond {maximum_active_waypoint}; expected progress past waypoint 1"
        )
    if profile == "baseline" and contact_fraction > 0.35:
        issues.append(f"contact is active in {100.0 * contact_fraction:.1f}% of samples")
    if profile == "baseline" and lidar_fraction < 0.80:
        issues.append(f"usable LiDAR telemetry appears in only {100.0 * lidar_fraction:.1f}% of samples")
    if profile == "baseline" and command_rows and moving_command_fraction is not None and moving_command_fraction < 0.50:
        issues.append(
            f"nonzero wheel commands appear in only {100.0 * moving_command_fraction:.1f}% of command samples"
        )
    # Apply completion checks using the selected profile.
    if require_complete:
        if not mission_complete:
            issues.append("the controller never reported mission completion")
        if profile == "baseline" and (
            maximum_active_waypoint is None or maximum_active_waypoint < expected_waypoints
        ):
            issues.append(
                "the route did not finish: maximum active_waypoint is "
                f"{maximum_active_waypoint}; expected at least {expected_waypoints}"
            )
        if goals_reached is None or goals_reached < expected_goals:
            issues.append(
                f"ground truth recorded {goals_reached or 0}/{expected_goals} ordered goals"
            )
        if collision_episodes is not None and collision_episodes > 0:
            issues.append(f"ground truth recorded {collision_episodes} collision episode(s)")
        if final_home_error is None:
            issues.append("final ground-truth position is unavailable")
        elif final_home_error > home_tolerance_m:
            issues.append(
                f"final HOME error is {final_home_error:.3f} m; "
                f"limit is {home_tolerance_m:.3f} m"
            )
        if blocked_rows:
            issues.append(f"the controller entered a blocked state for {blocked_rows} sample(s)")

    # Task 3 health metrics summarize duration, completion, safety, and blocked-state behavior.
    metrics = {
        "duration_s": duration,
        "ground_truth_path_length_m": path_length,
        "average_ground_truth_speed_m_s": path_length / duration if duration > 0.0 else 0.0,
        "contact_sample_fraction": contact_fraction,
        "lidar_telemetry_fraction": lidar_fraction,
        "moving_command_fraction": moving_command_fraction,
        "final_window_duration_s": window_duration,
        "final_window_path_gain_m": final_window_path_gain,
        "final_window_emergency_fraction": final_window_emergency_fraction,
        "maximum_active_waypoint": maximum_active_waypoint,
        "mission_complete_reported": mission_complete,
        "goals_reached_ground_truth": goals_reached,
        "collision_episodes": collision_episodes,
        "final_home_error_m": final_home_error,
        "position_rmse_m": position_rmse,
        "position_max_error_m": maximum_position_error,
        "full_mission_required": require_complete,
        "profile": profile,
        "blocked_samples": blocked_rows,
    }
    return not issues, issues, metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--minimum-duration-s", type=float, default=45.0)
    # Let the command line select baseline or student completion semantics.
    parser.add_argument(
        "--profile",
        choices=("baseline", "student"),
        default="baseline",
        help="Use baseline waypoint rules or student mission rules.",
    )
    parser.add_argument(
        "--require-complete",
        action="store_true",
        help="Require all 23 route waypoints, A-B-C-HOME by ground truth, no collisions, and final HOME tolerance.",
    )
    args = parser.parse_args()

    with args.csv_file.expanduser().open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    passed, issues, metrics = evaluate(
        rows,
        args.minimum_duration_s,
        require_complete=args.require_complete,
        profile=args.profile,
    )

    print("PASS" if passed else "FAIL")
    for key, value in metrics.items():
        print(f"{key}: {'unavailable' if value is None else value}")
    for issue in issues:
        print(f"- {issue}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
