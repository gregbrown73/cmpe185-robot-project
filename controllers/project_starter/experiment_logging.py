# Task 3: controller-side diagnostics for student-method experiments.

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path


# Task 3 fields: identify the current goal, route, replanning, avoidance, and localization state.
COMMON_FIELDS = [
    "time",
    "experiment_method",
    "active_goal",
    "route_index",
    "route_count",
    "route_version",
    "replan_count",
    "replan_reason",
    "avoidance_state",
    "temporary_obstacle_count",
    "mcl_state",
    "mcl_confidence",
    "mcl_ess",
    "particle_position_spread_m",
    "particle_heading_spread_rad",
]

# Task 3 events: mark important mission and failure transitions consistently.
EVENT_NAMES = {
    "GOAL_REACHED",
    "REPLAN_REQUEST",
    "LOCALIZATION_LOST",
    "LOCALIZATION_RECOVERED",
    "NO_PATH",
    "MISSION_COMPLETE",
}

# Task 3 output fields: support health checks, event details, and repeatable trials.
FIELDNAMES = COMMON_FIELDS + [
    # Pose/path fields support completion-time and path-length analysis.
    "estimated_x_m",
    "estimated_y_m",
    "estimated_heading_rad",
    "estimated_path_length_m",
    # Mission and reproducibility fields support the Task 3 report.
    "mission_complete",
    "goals_reached",
    "home_error_m",
    "collision_episodes",
    "event",
    "event_details",
    "scenario_name",
    "trial_number",
    "random_seed",
    "configuration_snapshot",
]


class StudentRunLogger:
    # Write one controller-side CSV row per diagnostic or event update.

    def __init__(
        self,
        output_path: Path | None = None,
        *,
        experiment_method: str = "student",
        scenario_name: str = "default",
        trial_number: int = 1,
        random_seed: int | None = None,
        configuration: dict | None = None,
    ) -> None:
        # Choose a timestamped results file when the controller did not supply one.
        if output_path is None:
            results_dir = Path(__file__).resolve().parents[2] / "results"
            results_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = results_dir / f"student_run_{stamp}.csv"
        # Create the output directory and initialize the CSV header.
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.manifest_path = output_path.with_suffix(".manifest.json")
        self.handle = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=FIELDNAMES)
        self.writer.writeheader()
        self.handle.flush()
        # Store experiment identity on every row for easy filtering later.
        self.metadata = {
            "experiment_method": experiment_method,
            "scenario_name": scenario_name,
            "trial_number": trial_number,
            "random_seed": random_seed,
            "configuration_snapshot": json.dumps(configuration or {}, sort_keys=True),
        }
        # Save the complete configuration in a sidecar trial manifest.
        self.manifest_path.write_text(
            json.dumps(
                {
                    "log_file": str(output_path),
                    "experiment_method": experiment_method,
                    "scenario_name": scenario_name,
                    "trial_number": trial_number,
                    "random_seed": random_seed,
                    "configuration": configuration or {},
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )

    def write(
        self,
        time_s: float,
        diagnostics: dict | None = None,
        *,
        event: str = "",
        event_details: str = "",
    ) -> None:
        # Reject custom event names so analysis remains consistent.
        if event and event not in EVENT_NAMES:
            raise ValueError(f"unsupported experiment event: {event}")
        # Start with a complete row, then fill in available diagnostics.
        row = {field: "" for field in FIELDNAMES}
        row.update(self.metadata)
        row["time"] = time_s
        row["event"] = event
        row["event_details"] = event_details
        if diagnostics:
            for field in FIELDNAMES:
                if field in {"event", "event_details", *self.metadata}:
                    continue
                if field in diagnostics:
                    row[field] = diagnostics[field]
        # Flush each row so a stopped simulation keeps its latest diagnostics.
        self.writer.writerow(row)
        self.handle.flush()

    def log_event(
        self,
        time_s: float,
        event: str,
        diagnostics: dict | None = None,
        details: str = "",
    ) -> None:
        # Events use the same CSV schema as ordinary diagnostic rows.
        self.write(time_s, diagnostics, event=event, event_details=details)

    def close(self) -> None:
        # Close the file safely when the controller exits.
        if not self.handle.closed:
            self.handle.close()

    def __enter__(self) -> "StudentRunLogger":
        # Support `with StudentRunLogger(...)` in tests and tools.
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        # Always close the file at the end of a context block.
        self.close()
