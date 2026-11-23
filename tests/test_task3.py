# Tests for Person 2 Task 3 diagnostics and experiment tooling.
# Run with: python -m unittest tests.test_task3 -v

from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from analyze_run import read_rows, summarize  # noqa: E402
from check_run_health import evaluate  # noqa: E402
from controllers.project_starter.experiment_logging import (  # noqa: E402
    COMMON_FIELDS,
    EVENT_NAMES,
    StudentRunLogger,
)


class Task3Tests(unittest.TestCase):
    def test_student_health_profile_does_not_require_23_waypoints(self) -> None:
        # Student routes may contain any number of generated waypoints.
        rows = [
            {
                "time": str(time_s),
                "mission_complete": "1",
                "goals_reached": "4",
                "home_error_m": "0.05",
                "avoidance_state": "clear",
            }
            for time_s in (0, 20, 40, 60)
        ]
        passed, issues, metrics = evaluate(
            rows,
            minimum_duration_s=45.0,
            require_complete=True,
            profile="student",
        )
        self.assertTrue(passed, issues)
        self.assertEqual(metrics["profile"], "student")

    def test_logger_writes_required_fields_and_events(self) -> None:
        # Verify the CSV schema, event row, and reproducibility manifest.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "student.csv"
            with StudentRunLogger(
                path,
                experiment_method="astar",
                scenario_name="nominal",
                trial_number=2,
                random_seed=185,
                configuration={"mode": "student"},
            ) as logger:
                logger.write(1.0, {"active_goal": "A", "route_count": 5})
                logger.log_event(2.0, "GOAL_REACHED", details="A")

            manifest = json.loads(path.with_suffix(".manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["scenario_name"], "nominal")
            self.assertEqual(manifest["trial_number"], 2)

            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(set(COMMON_FIELDS).issubset(rows[0]))
            self.assertEqual(rows[1]["event"], "GOAL_REACHED")
            self.assertEqual(rows[1]["event_details"], "A")
            self.assertEqual(rows[0]["random_seed"], "185")

    def test_logger_rejects_unknown_event_names(self) -> None:
        # Rejecting unknown events keeps experiment analysis consistent.
        with tempfile.TemporaryDirectory() as directory:
            with StudentRunLogger(Path(directory) / "student.csv") as logger:
                with self.assertRaises(ValueError):
                    logger.log_event(0.0, "CUSTOM_EVENT")

    def test_analyzer_reads_student_time_and_event_fields(self) -> None:
        # Student logs use `time` and named events instead of ground truth fields.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "student.csv"
            with StudentRunLogger(path) as logger:
                logger.write(
                    0.0,
                    {
                        "route_count": 3,
                        "active_goal": "A",
                        "estimated_path_length_m": 2.0,
                        "mission_complete": 1,
                        "goals_reached": 4,
                        "home_error_m": 0.05,
                    },
                )
                logger.log_event(5.0, "GOAL_REACHED", details="A")
                logger.log_event(10.0, "MISSION_COMPLETE")
            rows = read_rows(path)
            summary = summarize(rows, path)
            self.assertEqual(summary["duration_s"], 10.0)
            self.assertEqual(summary["maximum_route_count"], 3)
            self.assertEqual(summary["path_length_source"], "student_estimate")
            self.assertEqual(summary["goal_completion_order"], ["A"])
            self.assertEqual(summary["mission_completion_time_s"], 10.0)
            self.assertEqual(summary["event_counts"]["MISSION_COMPLETE"], 1)

    def test_required_event_names_are_fixed(self) -> None:
        # The event vocabulary is part of the Task 3 interface.
        self.assertEqual(
            EVENT_NAMES,
            {
                "GOAL_REACHED",
                "REPLAN_REQUEST",
                "LOCALIZATION_LOST",
                "LOCALIZATION_RECOVERED",
                "NO_PATH",
                "MISSION_COMPLETE",
            },
        )


if __name__ == "__main__":
    unittest.main()
