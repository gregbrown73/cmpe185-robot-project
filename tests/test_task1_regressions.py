from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from localization import RecoveryHypothesis, StudentLocalizer  # noqa: E402
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import Pose2D  # noqa: E402
import run_task1_scenarios as task1_runner  # noqa: E402
from run_task1_scenarios import (  # noqa: E402
    DEFAULT_KIDNAP_TIMES,
    DEFAULT_UNCERTAIN_START_POSE,
    analyze_scenario,
    installed_task1_test_supervisor,
    final_belief_contains_ground_truth,
    parse_kidnap_times,
)


class RecoveryRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    def test_pruning_keeps_plausible_weaker_mode(self) -> None:
        """Regression: a plausible physical mode must not be pruned by leader ratio."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_observation_count = (
            localizer.RECOVERY_WEAK_MODE_MIN_OBSERVATIONS
        )
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(Pose2D(-1.0, 0.0, 0.0), math.log(0.55), 0.95),
            RecoveryHypothesis(Pose2D(0.0, 0.0, 0.0), math.log(0.30), 0.95),
            RecoveryHypothesis(Pose2D(1.0, 0.0, 0.0), math.log(0.149), 0.95),
            RecoveryHypothesis(Pose2D(2.0, 0.0, 0.0), math.log(0.001), 0.70),
        ]

        probabilities = localizer._recovery_probabilities()
        probabilities = localizer._prune_negligible_recovery_modes(probabilities)

        self.assertEqual(len(localizer._recovery_hypotheses), 3)
        self.assertTrue(any(abs(h.pose.x - 1.0) < 1e-9 for h in localizer._recovery_hypotheses))
        self.assertFalse(any(abs(h.pose.x - 2.0) < 1e-9 for h in localizer._recovery_hypotheses))
        self.assertAlmostEqual(sum(probabilities), 1.0)

    def test_recovery_view_evidence_is_tempered(self) -> None:
        """Regression: one imperfect scan must not make the posterior arbitrarily sharp."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(Pose2D(0.0, 0.0, 0.0), 0.0, 0.0),
            RecoveryHypothesis(Pose2D(1.0, 0.0, 0.0), 0.0, 0.0),
        ]

        def fake_measurement(particle, _scan, _grid):
            return (0.9, -10.0 if particle.x < 0.5 else -20.0)

        localizer._particle_measurement = fake_measurement
        probabilities = localizer._score_recovery_view([(1.0, 0.0)] * 40, None)

        self.assertAlmostEqual(
            localizer._recovery_hypotheses[0].log_weight
            - localizer._recovery_hypotheses[1].log_weight,
            2.0,
            places=9,
        )
        self.assertLess(probabilities[0], 0.90)
        self.assertGreater(probabilities[1], 0.10)


    def test_recovery_does_not_resolve_at_77_23_split(self) -> None:
        """Regression: a 77/23 alias split is still ambiguous, not uniquely localized."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_translation_m = 0.60
        localizer._recovery_observation_count = 17
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(Pose2D(-1.5, 1.4, -0.9), math.log(0.773), 0.95),
            RecoveryHypothesis(Pose2D(2.4, 1.4, -0.9), math.log(0.227), 0.95),
        ]
        self.assertNotEqual(localizer._classify_recovery([0.773, 0.227]), "resolved")

    def test_recovery_resolves_at_81_19_split(self) -> None:
        """A clearly dominant mode can still finish recovery after enough evidence."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_translation_m = 0.60
        localizer._recovery_observation_count = 17
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(Pose2D(2.4, 1.4, -0.9), math.log(0.81), 0.95),
            RecoveryHypothesis(Pose2D(-1.5, 1.4, -0.9), math.log(0.19), 0.80),
        ]
        self.assertEqual(localizer._classify_recovery([0.81, 0.19]), "resolved")


    def test_high_probability_alias_does_not_resolve_when_scan_fit_is_tied(self) -> None:
        """Regression: cumulative tiny biases must not turn a symmetric alias into TRACKING."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_translation_m = 0.61
        localizer._recovery_observation_count = 17
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(
                Pose2D(-1.46, 1.39, -0.98), math.log(0.84), 0.90,
                score_sum=16.20, score_count=18,
            ),
            RecoveryHypothesis(
                Pose2D(2.44, 1.39, -0.98), math.log(0.16), 0.89,
                score_sum=16.02, score_count=18,
            ),
        ]
        self.assertEqual(localizer._classify_recovery([0.84, 0.16]), "ambiguous")

    def test_low_probability_scan_consistent_mode_is_not_pruned(self) -> None:
        """A map-consistent mode survives even after cumulative posterior bias gets tiny."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_observation_count = 20
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(
                Pose2D(-1.5, 1.4, -0.9), math.log(0.995), 0.91,
                score_sum=19.0, score_count=21,
            ),
            RecoveryHypothesis(
                Pose2D(2.4, 1.4, -0.9), math.log(0.004), 0.90,
                score_sum=18.9, score_count=21,
            ),
            RecoveryHypothesis(
                Pose2D(0.0, -2.0, 1.0), math.log(0.001), 0.50,
                score_sum=10.5, score_count=21,
            ),
        ]
        probabilities = localizer._recovery_probabilities()
        localizer._prune_negligible_recovery_modes(probabilities)
        self.assertEqual(len(localizer._recovery_hypotheses), 2)
        self.assertTrue(any(h.pose.x > 2.0 for h in localizer._recovery_hypotheses))

    def test_recovery_translation_is_counted_once(self) -> None:
        """Regression: refactoring must not double-count translated recovery distance."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(Pose2D(0.0, 0.0, 0.0), 0.0, 0.9),
            RecoveryHypothesis(Pose2D(1.0, 0.0, 0.0), 0.0, 0.9),
        ]
        localizer._update_recovery_hypotheses((0.02, 0.0), [], self.grid)
        self.assertAlmostEqual(localizer._recovery_translation_m, 0.02)
        self.assertAlmostEqual(localizer._recovery_translation_since_evidence_m, 0.02)

    def test_recovery_particles_follow_hypothesis_posterior(self) -> None:
        """Regression: weak hypotheses should receive fewer particles, not equal clouds."""
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=100)
        first = Pose2D(-3.2, -2.2, 0.0)
        second = Pose2D(3.2, 2.2, math.pi)
        localizer._recovery_hypotheses = [
            RecoveryHypothesis(first, math.log(0.75), 0.95),
            RecoveryHypothesis(second, math.log(0.25), 0.95),
        ]

        localizer._rebuild_recovery_particles(self.grid)
        first_count = sum(
            math.hypot(p.x - first.x, p.y - first.y)
            < math.hypot(p.x - second.x, p.y - second.y)
            for p in localizer.particles
        )
        self.assertEqual(first_count, 75)
        self.assertEqual(len(localizer.particles) - first_count, 25)


class Task1RunnerRegressionTests(unittest.TestCase):

    def test_default_uncertain_start_pose_has_clearance(self) -> None:
        x, y, _ = DEFAULT_UNCERTAIN_START_POSE
        grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")
        self.assertFalse(grid.world_occupied(x, y))
        self.assertFalse(grid.inflated(0.30).world_occupied(x, y))

    def test_production_supervisor_has_no_task1_automation_hooks(self) -> None:
        production = (
            PROJECT_ROOT
            / "controllers"
            / "evaluation_supervisor"
            / "evaluation_supervisor.py"
        ).read_text(encoding="utf-8")
        fixture = (
            PROJECT_ROOT / "tests" / "webots" / "evaluation_supervisor_task1.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("CMPE185_AUTO_SCENARIO", production)
        self.assertIn("CMPE185_AUTO_SCENARIO", fixture)

    def test_test_supervisor_swap_restores_production_file(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            production = root / "evaluation_supervisor.py"
            fixture = root / "evaluation_supervisor_task1.py"
            production.write_text("production\n", encoding="utf-8")
            fixture.write_text("test fixture\n", encoding="utf-8")

            old_production = task1_runner.PRODUCTION_SUPERVISOR
            old_fixture = task1_runner.TASK1_SUPERVISOR_FIXTURE
            try:
                task1_runner.PRODUCTION_SUPERVISOR = production
                task1_runner.TASK1_SUPERVISOR_FIXTURE = fixture
                with installed_task1_test_supervisor():
                    self.assertEqual(production.read_text(encoding="utf-8"), "test fixture\n")
                self.assertEqual(production.read_text(encoding="utf-8"), "production\n")
            finally:
                task1_runner.PRODUCTION_SUPERVISOR = old_production
                task1_runner.TASK1_SUPERVISOR_FIXTURE = old_fixture

    def test_default_summary_stays_in_normal_results_folder(self) -> None:
        with TemporaryDirectory() as directory:
            old_root = task1_runner.PROJECT_ROOT
            try:
                task1_runner.PROJECT_ROOT = Path(directory)
                path = task1_runner.write_summary(
                    None,
                    [{"check": "unit_tests", "pass": "true", "notes": "ok"}],
                    "20261117_120000",
                )
                self.assertEqual(path.parent, Path(directory) / "results")
                self.assertFalse((Path(directory) / "task1_results").exists())
            finally:
                task1_runner.PROJECT_ROOT = old_root

    def test_optional_bundle_writes_summary_only_when_requested(self) -> None:
        with TemporaryDirectory() as directory:
            bundle = Path(directory) / "task1_results" / "20261117_120000"
            bundle.mkdir(parents=True)
            path = task1_runner.write_summary(
                bundle,
                [{"check": "unit_tests", "pass": "true", "notes": "ok"}],
                "20261117_120000",
            )
            self.assertEqual(path, bundle / "task1_summary.csv")

    def test_default_full_run_checks_three_kidnap_times(self) -> None:
        self.assertEqual(DEFAULT_KIDNAP_TIMES, (10.0, 30.0, 50.0))
        self.assertEqual(parse_kidnap_times("10, 30,50"), [10.0, 30.0, 50.0])

    def test_offline_truth_check_accepts_correct_nonleading_mode(self) -> None:
        mcl_row = {
            "top_hypothesis_x_m": "-0.9",
            "top_hypothesis_y_m": "2.1",
            "top_hypothesis_theta_rad": "0.0",
            "second_hypothesis_x_m": "0.92",
            "second_hypothesis_y_m": "-2.09",
            "second_hypothesis_theta_rad": "3.13",
            "third_hypothesis_x_m": "",
            "third_hypothesis_y_m": "",
            "third_hypothesis_theta_rad": "",
        }
        evaluation_rows = [
            {"gt_x_m": "0.9", "gt_y_m": "-2.1", "gt_theta_rad": str(math.pi)}
        ]
        present, error = final_belief_contains_ground_truth(mcl_row, evaluation_rows)
        self.assertTrue(present)
        self.assertLess(error, 0.03)

    def test_offline_truth_check_rejects_missing_correct_mode(self) -> None:
        mcl_row = {
            "top_hypothesis_x_m": "-0.9",
            "top_hypothesis_y_m": "2.1",
            "top_hypothesis_theta_rad": "0.0",
            "second_hypothesis_x_m": "-2.0",
            "second_hypothesis_y_m": "-1.0",
            "second_hypothesis_theta_rad": "1.0",
            "third_hypothesis_x_m": "2.0",
            "third_hypothesis_y_m": "1.0",
            "third_hypothesis_theta_rad": "-1.0",
        }
        evaluation_rows = [
            {"gt_x_m": "0.9", "gt_y_m": "-2.1", "gt_theta_rad": str(math.pi)}
        ]
        present, _ = final_belief_contains_ground_truth(mcl_row, evaluation_rows)
        self.assertFalse(present)


    def test_uncertain_start_rejects_wrong_tracking_pose(self) -> None:
        mcl_rows = [{
            "state": "TRACKING",
            "recovery_ambiguous": "0",
            "recovery_hypothesis_count": "0",
            "recovery_translation_m": "0.61",
            "recovery_observation_count": "17",
            "top_hypothesis_probability": "0",
            "second_hypothesis_probability": "0",
            "third_hypothesis_probability": "0",
            "measurement_score": "0.98",
        }]
        evaluation_rows = [{
            "position_error_m": "4.02",
            "heading_error_rad": "0.004",
            "gt_x_m": "2.27",
            "gt_y_m": "0.96",
            "gt_theta_rad": "-2.0",
        }]
        passed, note = analyze_scenario("uncertain_start", mcl_rows, evaluation_rows)
        self.assertFalse(passed)
        self.assertIn("truth_error=4.020m", note)

    def test_uncertain_start_accepts_correct_tracking_pose(self) -> None:
        mcl_rows = [{
            "state": "TRACKING",
            "recovery_ambiguous": "0",
            "recovery_hypothesis_count": "0",
            "recovery_translation_m": "0.61",
            "recovery_observation_count": "17",
            "top_hypothesis_probability": "0",
            "second_hypothesis_probability": "0",
            "third_hypothesis_probability": "0",
            "measurement_score": "0.98",
        }]
        evaluation_rows = [{
            "position_error_m": "0.05",
            "heading_error_rad": "0.03",
            "gt_x_m": "2.27",
            "gt_y_m": "0.96",
            "gt_theta_rad": "-2.0",
        }]
        passed, _ = analyze_scenario("uncertain_start", mcl_rows, evaluation_rows)
        self.assertTrue(passed)

    @staticmethod
    def _kidnap_rows(correct_mode: bool) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
        final = {
            "state": "LOCALIZING",
            "recovery_ambiguous": "1",
            "recovery_hypothesis_count": "3",
            "recovery_translation_m": "0.85",
            "recovery_observation_count": "23",
            "top_hypothesis_probability": "0.36",
            "second_hypothesis_probability": "0.33",
            "third_hypothesis_probability": "0.31",
            "measurement_score": "0.99",
            "top_hypothesis_x_m": "-0.9",
            "top_hypothesis_y_m": "2.1",
            "top_hypothesis_theta_rad": "0.0",
            "second_hypothesis_x_m": "0.91" if correct_mode else "-2.0",
            "second_hypothesis_y_m": "-2.09" if correct_mode else "-1.0",
            "second_hypothesis_theta_rad": "3.13" if correct_mode else "1.0",
            "third_hypothesis_x_m": "2.0",
            "third_hypothesis_y_m": "1.0",
            "third_hypothesis_theta_rad": "-1.0",
        }
        mcl_rows = [
            {"state": "TRACKING"},
            {"state": "LOST"},
            final,
        ]
        evaluation_rows = [
            {
                "gt_x_m": "0.9",
                "gt_y_m": "-2.1",
                "gt_theta_rad": str(math.pi),
                "commanded_linear_speed_m_s": "0.0",
                "commanded_angular_speed_rad_s": "0.0",
                "controller_state": "localization_localizing",
            }
        ]
        return mcl_rows, evaluation_rows

    def test_kidnap_analyzer_accepts_settled_multimodal_belief_with_truth_mode(self) -> None:
        mcl_rows, evaluation_rows = self._kidnap_rows(correct_mode=True)
        passed, _ = analyze_scenario("kidnap", mcl_rows, evaluation_rows)
        self.assertTrue(passed)

    def test_kidnap_analyzer_rejects_multimodal_belief_without_truth_mode(self) -> None:
        mcl_rows, evaluation_rows = self._kidnap_rows(correct_mode=False)
        passed, _ = analyze_scenario("kidnap", mcl_rows, evaluation_rows)
        self.assertFalse(passed)


if __name__ == "__main__":
    unittest.main()
