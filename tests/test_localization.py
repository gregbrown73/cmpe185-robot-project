from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))

from localization import Particle, StudentLocalizer, select_localization_pose  # noqa: E402
from navigation import LocalizationRecoveryMotion  # noqa: E402
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import Pose2D, wrap_angle  # noqa: E402


class LocalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    def make_localizer(self, **kwargs) -> StudentLocalizer:
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), **kwargs)
        localizer.initialize(self.grid)
        return localizer

    def synthetic_scan(
        self,
        pose,
        ray_count: int = 180,
        grid: OccupancyGrid | None = None,
    ) -> list[tuple[float, float]]:
        grid = grid or self.grid
        x, y, theta = pose
        points = []
        step = max(0.02, 0.5 * grid.resolution)
        for index in range(ray_count):
            relative_angle = -math.pi + 2.0 * math.pi * index / ray_count
            world_angle = theta + relative_angle
            distance = 0.05
            while distance < 4.0:
                world_x = x + distance * math.cos(world_angle)
                world_y = y + distance * math.sin(world_angle)
                if grid.world_occupied(world_x, world_y):
                    break
                distance += step
            if distance < 3.96:
                points.append(
                    (
                        distance * math.cos(relative_angle),
                        distance * math.sin(relative_angle),
                    )
                )
        return points

    @staticmethod
    def recovery_test_grid(asymmetric: bool) -> OccupancyGrid:
        """Small map that is exactly 180-degree symmetric unless requested otherwise."""
        width = height = 50
        cells = [[0 for _ in range(width)] for _ in range(height)]
        for row in range(height):
            cells[row][0] = 100
            cells[row][-1] = 100
        for col in range(width):
            cells[0][col] = 100
            cells[-1][col] = 100

        for row in range(10, 24):
            col = 16
            cells[row][col] = 100
            cells[height - 1 - row][width - 1 - col] = 100

        if asymmetric:
            for row in range(7, 20):
                cells[row][35] = 100
            for col in range(30, 40):
                cells[15][col] = 100

        return OccupancyGrid(cells, 0.1, -2.5, -2.5, 100)

    def test_nominal_particles_are_valid_and_normalized(self) -> None:
        localizer = self.make_localizer()
        inflated = self.grid.inflated(0.12)
        self.assertEqual(len(localizer.particles), 300)
        self.assertTrue(
            all(not inflated.world_occupied(p.x, p.y) for p in localizer.particles)
        )
        self.assertAlmostEqual(sum(p.weight for p in localizer.particles), 1.0)

    def test_motion_update_uses_midpoint_heading_and_wraps(self) -> None:
        localizer = self.make_localizer(
            motion_distance_noise_fraction=0.0,
            motion_heading_noise_fraction=0.0,
        )
        particle = Particle(0.0, 0.0, math.pi - 0.05, 1.0)
        moved = localizer._propagate_particle(particle, (1.0, 0.2))
        midpoint = particle.theta + 0.1
        self.assertAlmostEqual(moved.x, math.cos(midpoint))
        self.assertAlmostEqual(moved.y, math.sin(midpoint))
        self.assertGreaterEqual(moved.theta, -math.pi)
        self.assertLess(moved.theta, math.pi)

    def test_global_initialization_is_reproducible(self) -> None:
        first = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=25)
        second = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=25)
        first.initialize(self.grid, initialization_mode="global_free_space")
        second.initialize(self.grid, initialization_mode="global_free_space")
        first_poses = [(p.x, p.y, p.theta) for p in first.particles]
        second_poses = [(p.x, p.y, p.theta) for p in second.particles]
        self.assertEqual(first_poses, second_poses)

    def test_weight_normalization_and_resampling_are_safe(self) -> None:
        localizer = self.make_localizer(particle_count=4)
        localizer.particles = [
            Particle(0.5, 0.5, 0.0, 0.0),
            Particle(1.5, 0.5, 0.0, 0.0),
            Particle(2.5, 0.5, 0.0, 0.0),
            Particle(3.5, 0.5, 0.0, 0.0),
        ]
        localizer._normalize_weights()
        self.assertTrue(all(abs(p.weight - 0.25) < 1.0e-12 for p in localizer.particles))

        localizer.particles[0].weight = 1.0
        for particle in localizer.particles[1:]:
            particle.weight = 0.0
        self.assertAlmostEqual(localizer._effective_sample_size(), 1.0)
        localizer._systematic_resample()
        self.assertTrue(all(abs(p.weight - 0.25) < 1.0e-12 for p in localizer.particles))
        self.assertTrue(all(abs(p.x - 0.5) < 1.0e-12 for p in localizer.particles))

    def test_lidar_likelihood_prefers_correct_pose_and_rejects_outside_map(self) -> None:
        localizer = self.make_localizer()
        localizer._ensure_distance_field(self.grid)
        pose = (-3.2, -2.2, 0.0)
        scan = localizer._select_scan_points(self.synthetic_scan(pose))
        correct = localizer._particle_measurement_score(
            Particle(*pose, 1.0), scan, self.grid
        )
        wrong = localizer._particle_measurement_score(
            Particle(0.0, 0.0, 0.0, 1.0), scan, self.grid
        )
        outside = localizer._particle_measurement_score(
            Particle(20.0, 20.0, 0.0, 1.0), scan, self.grid
        )
        self.assertGreater(correct, wrong)
        self.assertLessEqual(outside, localizer.measurement_outlier_weight + 1.0e-9)

    def test_pose_estimate_uses_circular_heading_mean(self) -> None:
        localizer = self.make_localizer(particle_count=2)
        localizer.particles = [
            Particle(1.0, 2.0, math.pi - 0.05, 0.5),
            Particle(1.0, 2.0, -math.pi + 0.05, 0.5),
        ]
        estimate = localizer._weighted_pose(localizer.particles)
        self.assertLess(abs(abs(estimate.theta) - math.pi), 0.06)

    def test_required_diagnostics_api_is_preserved(self) -> None:
        localizer = self.make_localizer()
        diagnostics = localizer.get_diagnostics()
        self.assertEqual(
            set(diagnostics),
            {
                "ess",
                "position_spread_m",
                "heading_spread_rad",
                "measurement_score",
                "just_recovered",
            },
        )

    def test_stationary_symmetric_recovery_does_not_claim_tracking(self) -> None:
        localizer = self.make_localizer()
        odometry_pose = Pose2D(-3.2, -2.2, 0.0)
        start_scan = self.synthetic_scan((-3.2, -2.2, 0.0))
        for _ in range(40):
            localizer.update(odometry_pose, (0.0, 0.0), start_scan, self.grid)
        self.assertEqual(localizer.get_state(), "TRACKING")

        kidnapped = (0.9, -2.1, math.pi)
        kidnapped_scan = self.synthetic_scan(kidnapped)
        for _ in range(3):
            localizer.update(odometry_pose, (0.0, 0.0), kidnapped_scan, self.grid)
        self.assertEqual(localizer.get_state(), "LOST")
        self.assertGreaterEqual(len(localizer.get_recovery_hypotheses()), 2)

        for _ in range(20):
            localizer.update(odometry_pose, (0.0, 0.0), kidnapped_scan, self.grid)
        debug = localizer.get_debug_diagnostics()
        self.assertTrue(debug["recovery_active"])
        self.assertEqual(debug["recovery_observation_count"], 0)
        self.assertEqual(localizer.get_state(), "LOST")

    def test_symmetric_kidnap_preserves_multiple_modes_after_motion(self) -> None:
        grid = self.recovery_test_grid(asymmetric=False)
        start = (-1.7, -1.5, 0.2)
        localizer = StudentLocalizer(
            start,
            particle_count=200,
            initial_position_std_m=0.08,
            initial_heading_std_rad=0.08,
            motion_distance_noise_fraction=0.0,
            motion_heading_noise_fraction=0.0,
            recovery_measurement_threshold=0.65,
            lost_measurement_updates=3,
            recovery_reinjection_fraction=0.10,
        )
        localizer.initialize(grid)
        odometry_pose = Pose2D(*start)
        for _ in range(15):
            localizer.update(
                odometry_pose,
                (0.0, 0.0),
                self.synthetic_scan(start, grid=grid),
                grid,
            )
        self.assertEqual(localizer.get_state(), "TRACKING")

        true_pose = [1.5, -1.4, 2.4]
        for _ in range(3):
            localizer.update(
                odometry_pose,
                (0.0, 0.0),
                self.synthetic_scan(true_pose, grid=grid),
                grid,
            )
        self.assertEqual(localizer.get_state(), "LOST")

        for _ in range(80):
            distance = 0.01
            true_pose[0] += distance * math.cos(true_pose[2])
            true_pose[1] += distance * math.sin(true_pose[2])
            localizer.update(
                odometry_pose,
                (distance, 0.0),
                self.synthetic_scan(true_pose, grid=grid),
                grid,
            )
            if localizer.get_debug_diagnostics()["recovery_ambiguous"]:
                break

        hypotheses = localizer.get_recovery_hypotheses()
        self.assertEqual(localizer.get_state(), "LOCALIZING")
        self.assertTrue(localizer.get_debug_diagnostics()["recovery_ambiguous"])
        self.assertGreaterEqual(len(hypotheses), 2)
        self.assertGreater(hypotheses[0][0], 0.40)
        self.assertGreater(hypotheses[1][0], 0.40)

    def test_asymmetric_map_recovery_collapses_to_one_hypothesis(self) -> None:
        grid = self.recovery_test_grid(asymmetric=True)
        start = (-1.7, -1.5, 0.2)
        localizer = StudentLocalizer(
            start,
            particle_count=200,
            initial_position_std_m=0.08,
            initial_heading_std_rad=0.08,
            motion_distance_noise_fraction=0.0,
            motion_heading_noise_fraction=0.0,
            recovery_measurement_threshold=0.65,
            lost_measurement_updates=3,
            recovery_reinjection_fraction=0.10,
        )
        localizer.initialize(grid)
        odometry_pose = Pose2D(*start)
        for _ in range(15):
            localizer.update(
                odometry_pose,
                (0.0, 0.0),
                self.synthetic_scan(start, grid=grid),
                grid,
            )

        true_pose = [1.5, -1.4, 2.4]
        for _ in range(3):
            localizer.update(
                odometry_pose,
                (0.0, 0.0),
                self.synthetic_scan(true_pose, grid=grid),
                grid,
            )
        self.assertEqual(localizer.get_state(), "LOST")

        for _ in range(100):
            distance = 0.01
            true_pose[0] += distance * math.cos(true_pose[2])
            true_pose[1] += distance * math.sin(true_pose[2])
            localizer.update(
                odometry_pose,
                (distance, 0.0),
                self.synthetic_scan(true_pose, grid=grid),
                grid,
            )
            if not localizer.get_debug_diagnostics()["recovery_active"]:
                break
        self.assertFalse(localizer.get_debug_diagnostics()["recovery_active"])

        for _ in range(5):
            estimate = localizer.update(
                odometry_pose,
                (0.0, 0.0),
                self.synthetic_scan(true_pose, grid=grid),
                grid,
            )
        self.assertEqual(localizer.get_state(), "TRACKING")
        self.assertFalse(localizer.get_debug_diagnostics()["recovery_ambiguous"])
        self.assertLess(
            math.hypot(estimate.x - true_pose[0], estimate.y - true_pose[1]),
            0.15,
        )
        self.assertLess(abs(wrap_angle(estimate.theta - true_pose[2])), 0.20)

    def test_global_start_uses_map_wide_hypotheses_without_false_tracking(self) -> None:
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), particle_count=150)
        localizer.initialize(self.grid, initialization_mode="global_free_space")
        scan = self.synthetic_scan((0.9, -2.1, math.pi))
        localizer.update(Pose2D(-3.2, -2.2, 0.0), (0.0, 0.0), scan, self.grid)
        self.assertEqual(localizer.get_state(), "LOCALIZING")
        self.assertGreaterEqual(len(localizer.get_recovery_hypotheses()), 2)

    def test_pose_selector_preserves_baseline_mode(self) -> None:
        localizer = self.make_localizer(particle_count=1)
        localizer._particle_estimate = Pose2D(1.0, 2.0, 0.3)
        odometry = Pose2D(-1.0, -2.0, -0.3)
        self.assertEqual(select_localization_pose(odometry, localizer, "mcl"), localizer.get_estimate())
        self.assertIs(select_localization_pose(odometry, localizer, "odometry_baseline"), odometry)

    def test_recovery_motion_stops_when_localized(self) -> None:
        motion = LocalizationRecoveryMotion(
            turn_speed=0.35,
            forward_speed=0.045,
            translation_steps=10,
            translation_bursts=2,
            sensing_steps=5,
            avoidance_distance=0.23,
            maximum_cycles=2,
        )
        motion.start()
        linear, _, _, state = motion.command([(1.0, 0.0)], localized=False)
        self.assertGreater(linear, 0.0)
        self.assertEqual(state, "localization_active_translate")
        motion.command([(1.0, 0.0)], localized=True)
        self.assertFalse(motion.active)


if __name__ == "__main__":
    unittest.main()
