#!/usr/bin/env python3
"""Estimate wheel radius and effective track width from a Release 4 CSV log.

The calculation uses ground truth only as an instructor-side calibration tool.
Student controllers must never read the evaluation log during a run.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def value(row: dict, key: str) -> float | None:
    raw = row.get(key, "")
    if raw in ("", None, "None"):
        return None
    try:
        number = float(raw)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def estimate_drive_parameters(
    rows: list[dict],
    nominal_wheel_radius_m: float = 0.0200,
) -> dict:
    radius_samples: list[float] = []
    turn_pairs: list[tuple[float, float]] = []

    for previous, current in zip(rows, rows[1:]):
        left_previous = value(previous, "left_encoder_rad")
        left_current = value(current, "left_encoder_rad")
        right_previous = value(previous, "right_encoder_rad")
        right_current = value(current, "right_encoder_rad")
        x_previous = value(previous, "gt_x_m")
        x_current = value(current, "gt_x_m")
        y_previous = value(previous, "gt_y_m")
        y_current = value(current, "gt_y_m")
        theta_previous = value(previous, "gt_theta_rad")
        theta_current = value(current, "gt_theta_rad")
        linear = value(current, "commanded_linear_speed_m_s")
        angular = value(current, "commanded_angular_speed_rad_s")
        contact = int(value(current, "contact_now") or 0)
        required = (
            left_previous,
            left_current,
            right_previous,
            right_current,
            x_previous,
            x_current,
            y_previous,
            y_current,
            theta_previous,
            theta_current,
            linear,
            angular,
        )
        if contact or any(item is None for item in required):
            continue

        delta_left = left_current - left_previous
        delta_right = right_current - right_previous
        delta_mean = 0.5 * (delta_left + delta_right)
        distance = math.hypot(x_current - x_previous, y_current - y_previous)
        delta_heading = wrap_angle(theta_current - theta_previous)

        if (
            abs(linear) >= 0.03
            and abs(angular) <= 0.08
            and delta_left * delta_right > 0.0
            and abs(delta_mean) > 1e-5
        ):
            candidate = distance / abs(delta_mean)
            if 0.015 <= candidate <= 0.025:
                radius_samples.append(candidate)

        if (
            abs(linear) <= 0.01
            and abs(angular) >= 0.20
            and abs(delta_heading) > 1e-5
            and abs(delta_right - delta_left) > 1e-5
        ):
            turn_pairs.append((delta_right - delta_left, delta_heading))

    radius = statistics.median(radius_samples) if radius_samples else nominal_wheel_radius_m
    track_samples = [
        abs(radius * wheel_difference / heading_change)
        for wheel_difference, heading_change in turn_pairs
    ]
    track_samples = [sample for sample in track_samples if 0.040 <= sample <= 0.080]
    track = statistics.median(track_samples) if track_samples else None
    return {
        "estimated_wheel_radius_m": radius if radius_samples else None,
        "wheel_radius_sample_count": len(radius_samples),
        "wheel_radius_used_for_track_m": radius,
        "estimated_effective_track_width_m": track,
        "track_width_sample_count": len(track_samples),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()
    with args.csv_file.expanduser().open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    result = estimate_drive_parameters(rows)
    print(json.dumps(result, indent=2))
    if not result["wheel_radius_sample_count"] or not result["track_width_sample_count"]:
        raise SystemExit(
            "Not enough usable straight/rotate samples. Use a complete Release 4 log with encoder fields."
        )


if __name__ == "__main__":
    main()
