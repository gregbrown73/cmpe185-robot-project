#!/usr/bin/env python3
# Task 3 analysis tool.
# Run with: python tools/analyze_run.py results/student_run_YYYYMMDD_HHMMSS.csv
"""Summarize one evaluation CSV and optionally create diagnostic plots."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def choose_log(project_root: Path, requested: str | None) -> Path:
    if requested:
        return Path(requested).expanduser().resolve()
    # Search both Supervisor baseline logs and controller-side student logs.
    candidates = sorted(
        list((project_root / "results").glob("run_*.csv"))
        + list((project_root / "results").glob("student_run_*.csv"))
    )
    if not candidates:
        raise FileNotFoundError("No run_*.csv file exists under results/")
    return candidates[-1]


def read_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    # Normalize student time fields so the existing analysis can reuse them.
    for row in rows:
        if not row.get("time_s") and row.get("time"):
            row["time_s"] = row["time"]
    return rows


def as_float(row: dict, key: str) -> float | None:
    value = row.get(key, "")
    if value in ("", None) and key == "time_s":
        value = row.get("time", "")
    if value in ("", None, "None"):
        return None
    return float(value)


def as_optional_int(row: dict, key: str) -> int | None:
    # Preserve unavailable values as None instead of treating them as zero.
    value = row.get(key, "")
    if value in ("", None, "None"):
        return None
    return int(float(value))


def rms(values: list[float]) -> float | None:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def summarize(rows: list[dict], path: Path) -> dict:
    # Collect both baseline metrics and student route/event diagnostics.
    def last_value(key: str):
        # Event-only rows may leave telemetry columns blank, so use the latest value.
        for row in reversed(rows):
            value = row.get(key, "")
            if value not in ("", None, "None"):
                return value
        return ""

    position_errors = [value for row in rows if (value := as_float(row, "position_error_m")) is not None]
    heading_errors = [value for row in rows if (value := as_float(row, "heading_error_rad")) is not None]
    final = rows[-1]
    start_time = as_float(rows[0], "time_s") or 0.0
    end_time = as_float(final, "time_s") or start_time
    duration = end_time - start_time
    # Use ground-truth path length when available, otherwise use the student estimate.
    path_length = as_float({"ground_truth_path_length_m": last_value("ground_truth_path_length_m")}, "ground_truth_path_length_m")
    path_length_source = "ground_truth" if path_length is not None else "unavailable"
    if path_length is None:
        path_length = as_float({"estimated_path_length_m": last_value("estimated_path_length_m")}, "estimated_path_length_m")
        if path_length is not None:
            path_length_source = "student_estimate"
    contacts = [int(row.get("contact_now", 0) or 0) for row in rows]
    lidar_samples = [as_float(row, "lidar_point_count") for row in rows]
    minimum_ranges = [as_float(row, "minimum_lidar_range_m") for row in rows]
    lidar_rows = sum(
        1
        for count, minimum_range in zip(lidar_samples, minimum_ranges)
        if (count is not None and count > 0) or minimum_range is not None
    )
    final_window_duration = min(20.0, max(5.0, 0.40 * duration))
    final_window_start = end_time - final_window_duration
    final_window_rows = [
        row for row in rows if (as_float(row, "time_s") or 0.0) >= final_window_start
    ]
    # Use the same path source for the final-window progress calculation.
    window_start_path = None
    if final_window_rows:
        window_start_path = as_float(final_window_rows[0], "ground_truth_path_length_m")
        if window_start_path is None:
            window_start_path = as_float(final_window_rows[0], "estimated_path_length_m")
    if window_start_path is None:
        window_start_path = path_length
    final_window_path_gain = (
        max(0.0, path_length - window_start_path)
        if path_length is not None and window_start_path is not None
        else None
    )
    safety_state_counts: dict[str, int] = {}
    controller_state_counts: dict[str, int] = {}
    for row in rows:
        # Accept the baseline safety name and the student avoidance name.
        safety_state = str(row.get("safety_state") or row.get("avoidance_state", "")).strip()
        controller_state = str(row.get("controller_state", "")).strip()
        if safety_state:
            safety_state_counts[safety_state] = safety_state_counts.get(safety_state, 0) + 1
        if controller_state:
            controller_state_counts[controller_state] = controller_state_counts.get(controller_state, 0) + 1
    route_index_values = [
        int(value)
        for row in rows
        if (value := as_float(row, "active_waypoint")) is not None
    ]
    route_index_values.extend(
        int(value)
        for row in rows
        if (value := as_float(row, "route_index")) is not None
    )
    route_count_values = [
        int(value)
        for row in rows
        if (value := as_float(row, "route_count")) is not None
    ]
    # Count named events for quick failure diagnosis.
    # Preserve event order and timestamps for goal order and recovery analysis.
    event_log: list[dict] = []
    event_counts: dict[str, int] = {}
    replan_reasons: dict[str, int] = {}
    for row in rows:
        event = str(row.get("event", "")).strip()
        if event:
            event_counts[event] = event_counts.get(event, 0) + 1
            event_log.append(
                {
                    "time_s": as_float(row, "time_s"),
                    "event": event,
                    "details": str(row.get("event_details", "")),
                }
            )
        reason = str(row.get("replan_reason", "")).strip()
        if reason:
            replan_reasons[reason] = replan_reasons.get(reason, 0) + 1
    # Extract the ordered goals and mission completion timestamp from events.
    goal_completion_order = [
        event["details"]
        for event in event_log
        if event["event"] == "GOAL_REACHED" and event["details"]
    ]
    completion_times = [
        event["time_s"]
        for event in event_log
        if event["event"] == "MISSION_COMPLETE" and event["time_s"] is not None
    ]
    mission_completion_time_s = (
        completion_times[0] - start_time if completion_times else None
    )

    # Pair localization-loss and recovery events to estimate recovery time.
    recovery_times: list[float] = []
    lost_time = None
    for event in event_log:
        if event["event"] == "LOCALIZATION_LOST":
            lost_time = event["time_s"]
        elif (
            event["event"] == "LOCALIZATION_RECOVERED"
            and lost_time is not None
            and event["time_s"] is not None
        ):
            recovery_times.append(event["time_s"] - lost_time)
            lost_time = None

    final_x = as_float(final, "gt_x_m")
    final_y = as_float(final, "gt_y_m")
    final_home_error = (
        math.hypot(final_x + 3.2, final_y + 2.2)
        if final_x is not None and final_y is not None
        else None
    )
    # Task 3 summary: report experiment identity, mission progress, events, and subsystem state.
    final_home_error_from_student = as_float({"home_error_m": last_value("home_error_m")}, "home_error_m")
    if final_home_error is None:
        final_home_error = final_home_error_from_student
    collision_episodes = as_optional_int(
        {"collision_episodes": last_value("collision_episodes")}, "collision_episodes"
    )
    goals_reached = as_optional_int(
        {"goals_reached_ground_truth": last_value("goals_reached_ground_truth")},
        "goals_reached_ground_truth",
    )
    if goals_reached is None:
        goals_reached = as_optional_int({"goals_reached": last_value("goals_reached")}, "goals_reached")
    mission_complete = bool(
        int(float(last_value("mission_complete_reported") or last_value("mission_complete") or 0))
    )
    # Student success is based on mission completion, ordered goals, and HOME tolerance.
    success = (
        mission_complete
        and goals_reached is not None
        and goals_reached >= 4
        and (final_home_error is None or final_home_error <= 0.20)
        and (collision_episodes is None or collision_episodes == 0)
    )
    # Task 3 failure reasons explain why a run did not satisfy the mission gate.
    failure_reasons = []
    if not mission_complete:
        failure_reasons.append("MISSION_COMPLETE was not reported")
    if goals_reached is not None and goals_reached < 4:
        failure_reasons.append(f"only {goals_reached}/4 goals were reached")
    if final_home_error is not None and final_home_error > 0.20:
        failure_reasons.append(f"HOME error was {final_home_error:.3f} m")
    if collision_episodes is not None and collision_episodes > 0:
        failure_reasons.append(f"{collision_episodes} collision episode(s)")

    # Task 3 summary: report experiment identity, mission results, events, and diagnostics.
    return {
        "source": str(path),
        "experiment_method": last_value("experiment_method") or last_value("mode"),
        "scenario_name": last_value("scenario_name"),
        "trial_number": last_value("trial_number"),
        "random_seed": last_value("random_seed"),
        "duration_s": duration,
        "success": success,
        "failure_reasons": failure_reasons,
        "goal_completion_order": goal_completion_order,
        "mission_completion_time_s": mission_completion_time_s,
        "kidnapped_recovery_times_s": recovery_times,
        "kidnapped_recovery_time_s": recovery_times[0] if recovery_times else None,
        "position_rmse_m": rms(position_errors),
        "position_max_error_m": max(position_errors) if position_errors else None,
        "heading_rmse_rad": rms(heading_errors),
        "heading_max_error_rad": max(heading_errors) if heading_errors else None,
        "ground_truth_path_length_m": path_length,
        "path_length_source": path_length_source,
        "average_ground_truth_speed_m_s": (
            path_length / duration if path_length is not None and duration > 0.0 else None
        ),
        "contact_sample_fraction": sum(contacts) / len(contacts) if contacts else None,
        "lidar_telemetry_fraction": lidar_rows / len(rows),
        "collision_episodes": collision_episodes,
        "goals_reached": goals_reached,
        "mission_complete_reported": mission_complete,
        "final_home_error_m": final_home_error,
        "samples_with_estimate": len(position_errors),
        "maximum_active_waypoint": max(route_index_values) if route_index_values else None,
        "maximum_route_count": max(route_count_values) if route_count_values else None,
        "maximum_temporary_obstacle_count": max(
            (int(value) for row in rows if (value := as_float(row, "temporary_obstacle_count")) is not None),
            default=None,
        ),
        # Counted events show what happened during the run.
        "event_counts": event_counts,
        "event_log": event_log,
        "replan_count": as_float(final, "replan_count"),
        "route_version": as_float(final, "route_version"),
        "replan_reason": final.get("replan_reason", ""),
        "replan_reason_counts": replan_reasons,
        "configuration_snapshot": last_value("configuration_snapshot"),
        # Final state fields show where the controller ended.
        "final_active_goal": last_value("active_goal"),
        "final_avoidance_state": last_value("avoidance_state") or last_value("safety_state"),
        "final_mcl_state": last_value("mcl_state"),
        "final_mcl_confidence": as_float({"mcl_confidence": last_value("mcl_confidence")}, "mcl_confidence"),
        "final_mcl_ess": as_float({"mcl_ess": last_value("mcl_ess")}, "mcl_ess"),
        "final_particle_position_spread_m": as_float(
            {"particle_position_spread_m": last_value("particle_position_spread_m")},
            "particle_position_spread_m",
        ),
        "final_particle_heading_spread_rad": as_float(
            {"particle_heading_spread_rad": last_value("particle_heading_spread_rad")},
            "particle_heading_spread_rad",
        ),
        "final_window_duration_s": final_window_duration,
        "final_window_path_gain_m": final_window_path_gain,
        "controller_state_sample_counts": controller_state_counts,
        "safety_state_sample_counts": safety_state_counts,
    }


def make_plots(rows: list[dict], output_path: Path, project_root: Path) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Matplotlib is not installed; summary was created without plots.")
        return False

    # Support both Supervisor ground-truth plots and student estimated-pose plots.
    has_ground_truth = {"time_s", "gt_x_m", "gt_y_m", "est_x_m", "est_y_m"}.issubset(rows[0])
    has_student_estimate = {"time_s", "estimated_x_m", "estimated_y_m"}.issubset(rows[0])
    if not has_ground_truth and not has_student_estimate:
        print("Ground-truth/estimate columns are unavailable; summary was created without plots.")
        return False

    if has_student_estimate and not has_ground_truth:
        # Student logs can plot the estimated trajectory even without Supervisor ground truth.
        times = [as_float(row, "time_s") or 0.0 for row in rows]
        estimate_x = [as_float(row, "estimated_x_m") for row in rows]
        estimate_y = [as_float(row, "estimated_y_m") for row in rows]
        waypoint_data = json.loads(
            (project_root / "maps" / "waypoints.json").read_text(encoding="utf-8")
        )
        route_x = [point[0] for point in waypoint_data["baseline_waypoints"]]
        route_y = [point[1] for point in waypoint_data["baseline_waypoints"]]
        figure, axis = plt.subplots(figsize=(7, 6))
        axis.plot(route_x, route_y, "--", color="#94a3b8", label="provided route", linewidth=1.2)
        axis.plot(estimate_x, estimate_y, label="student estimate", linewidth=1.6)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title("Student estimated trajectory")
        axis.legend()
        axis.grid(True, alpha=0.25)
        figure.tight_layout()
        figure.savefig(output_path, dpi=160)
        plt.close(figure)
        return True

    times = [as_float(row, "time_s") or 0.0 for row in rows]
    gt_x = [as_float(row, "gt_x_m") for row in rows]
    gt_y = [as_float(row, "gt_y_m") for row in rows]
    est_x = [as_float(row, "est_x_m") for row in rows]
    est_y = [as_float(row, "est_y_m") for row in rows]
    pos_error = [as_float(row, "position_error_m") for row in rows]
    heading_error = [as_float(row, "heading_error_rad") for row in rows]
    waypoint_data = json.loads(
        (project_root / "maps" / "waypoints.json").read_text(encoding="utf-8")
    )
    route_x = [point[0] for point in waypoint_data["baseline_waypoints"]]
    route_y = [point[1] for point in waypoint_data["baseline_waypoints"]]

    figure, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    axes[0].plot(route_x, route_y, "--", color="#94a3b8", label="provided route", linewidth=1.2)
    axes[0].plot(gt_x, gt_y, label="ground truth", linewidth=2)
    axes[0].plot(est_x, est_y, label="estimate", linewidth=1.4)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title("Trajectory")
    axes[0].legend()
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, pos_error, color="#2563eb")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].set_title("Localization position error")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, heading_error, color="#16a34a")
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("absolute heading error [rad]")
    axes[2].set_title("Localization heading error")
    axes[2].grid(True, alpha=0.25)

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", nargs="?", help="Evaluation CSV; defaults to newest file")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[1]
    log_path = choose_log(project_root, args.csv_file)
    rows = read_rows(log_path)
    if not rows:
        raise ValueError(f"No data rows in {log_path}")
    summary = summarize(rows, log_path)
    summary_path = log_path.with_name(f"{log_path.stem}_summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Summary: {summary_path}")
    if not args.no_plot:
        plot_path = log_path.with_name(f"{log_path.stem}_plots.png")
        if make_plots(rows, plot_path, project_root):
            print(f"Plots: {plot_path}")


if __name__ == "__main__":
    main()
