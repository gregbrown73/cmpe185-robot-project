#!/usr/bin/env python3
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
    candidates = sorted((project_root / "results").glob("run_*.csv"))
    if not candidates:
        raise FileNotFoundError("No run_*.csv file exists under results/")
    return candidates[-1]


def read_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(row: dict, key: str) -> float | None:
    value = row.get(key, "")
    if value in ("", None, "None"):
        return None
    return float(value)


def rms(values: list[float]) -> float | None:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def summarize(rows: list[dict], path: Path) -> dict:
    position_errors = [value for row in rows if (value := as_float(row, "position_error_m")) is not None]
    heading_errors = [value for row in rows if (value := as_float(row, "heading_error_rad")) is not None]
    final = rows[-1]
    start_time = as_float(rows[0], "time_s") or 0.0
    end_time = as_float(final, "time_s") or start_time
    duration = end_time - start_time
    path_length = as_float(final, "ground_truth_path_length_m")
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
    window_start_path = (
        as_float(final_window_rows[0], "ground_truth_path_length_m")
        if final_window_rows
        else path_length
    )
    final_window_path_gain = (
        max(0.0, path_length - window_start_path)
        if path_length is not None and window_start_path is not None
        else None
    )
    safety_state_counts: dict[str, int] = {}
    controller_state_counts: dict[str, int] = {}
    for row in rows:
        safety_state = str(row.get("safety_state", "")).strip()
        controller_state = str(row.get("controller_state", "")).strip()
        if safety_state:
            safety_state_counts[safety_state] = safety_state_counts.get(safety_state, 0) + 1
        if controller_state:
            controller_state_counts[controller_state] = controller_state_counts.get(controller_state, 0) + 1
    waypoint_values = [
        int(value)
        for row in rows
        if (value := as_float(row, "active_waypoint")) is not None
    ]
    final_x = as_float(final, "gt_x_m")
    final_y = as_float(final, "gt_y_m")
    final_home_error = (
        math.hypot(final_x + 3.2, final_y + 2.2)
        if final_x is not None and final_y is not None
        else None
    )
    return {
        "source": str(path),
        "duration_s": duration,
        "position_rmse_m": rms(position_errors),
        "position_max_error_m": max(position_errors) if position_errors else None,
        "heading_rmse_rad": rms(heading_errors),
        "heading_max_error_rad": max(heading_errors) if heading_errors else None,
        "ground_truth_path_length_m": path_length,
        "average_ground_truth_speed_m_s": (
            path_length / duration if path_length is not None and duration > 0.0 else None
        ),
        "contact_sample_fraction": sum(contacts) / len(contacts) if contacts else None,
        "lidar_telemetry_fraction": lidar_rows / len(rows),
        "collision_episodes": int(final.get("collision_episodes", 0) or 0),
        "goals_reached": int(final.get("goals_reached_ground_truth", 0) or 0),
        "mission_complete_reported": bool(int(final.get("mission_complete_reported", 0) or 0)),
        "final_home_error_m": final_home_error,
        "samples_with_estimate": len(position_errors),
        "maximum_active_waypoint": max(waypoint_values) if waypoint_values else None,
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
