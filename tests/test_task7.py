"""Tests for Task 7 local avoidance and temporary obstacle handling."""

# Run with: python3 -m unittest tests.test_task7 -v
# TASK 7: Obstacle avoidance and temporary-obstacle unit tests.

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))

from navigation import SafetyLayer  # noqa: E402
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import Pose2D  # noqa: E402
from temporary_obstacles import TemporaryObstacleLayer  # noqa: E402


class TemporaryObstacleTests(unittest.TestCase):
    @staticmethod
    def grid() -> OccupancyGrid:
        return OccupancyGrid(
            [[0 for _ in range(8)] for _ in range(8)],
            resolution=0.5,
            origin_x=0.0,
            origin_y=0.0,
            occupied_value=100,
        )

    def test_requires_three_neighboring_observations(self) -> None:
        layer = TemporaryObstacleLayer(confirmation_count=3, ttl_s=5.0)
        grid = self.grid()
        pose = Pose2D(1.0, 1.0, 0.0)
        scan = [(0.75, 0.0)]

        self.assertFalse(layer.observe(scan, pose, grid, 0.0))
        self.assertEqual(layer.active_count(0.0), 0)
        self.assertFalse(layer.observe(scan, pose, grid, 0.1))
        self.assertTrue(layer.observe(scan, pose, grid, 0.2))
        self.assertEqual(layer.active_count(0.2), 1)
        self.assertEqual(layer.version, 1)

    def test_static_wall_observations_are_ignored(self) -> None:
        grid = self.grid()
        grid.cells[3][3] = 100
        layer = TemporaryObstacleLayer(confirmation_count=1, ttl_s=5.0)
        pose = Pose2D(1.0, 1.0, 0.0)
        static_center = grid.cell_to_world(3, 3)
        scan = [(static_center[0] - pose.x, static_center[1] - pose.y)]
        self.assertFalse(layer.observe(scan, pose, grid, 0.0))
        self.assertEqual(layer.active_count(0.0), 0)

    def test_observations_transform_into_world_cells(self) -> None:
        layer = TemporaryObstacleLayer(confirmation_count=1, ttl_s=5.0)
        grid = self.grid()
        pose = Pose2D(1.0, 1.0, math.pi / 2.0)
        self.assertTrue(layer.observe([(0.75, 0.0)], pose, grid, 0.0))
        planning_grid = layer.build_planning_grid(grid, inflation_radius_m=0.0, time_s=0.0)
        self.assertTrue(planning_grid.occupied(*grid.world_to_cell(1.0, 1.75)))

    def test_temporary_cells_expire_and_static_grid_is_unchanged(self) -> None:
        grid = self.grid()
        original = [row[:] for row in grid.cells]
        layer = TemporaryObstacleLayer(confirmation_count=1, ttl_s=5.0)
        pose = Pose2D(1.0, 1.0, 0.0)
        self.assertTrue(layer.observe([(0.75, 0.0)], pose, grid, 0.0))
        self.assertEqual(layer.active_count(5.0), 0)
        self.assertEqual(layer.version, 2)
        self.assertEqual(grid.cells, original)

    def test_planning_grid_inflates_combined_occupancy_once(self) -> None:
        layer = TemporaryObstacleLayer(confirmation_count=1, ttl_s=5.0)
        grid = self.grid()
        pose = Pose2D(1.0, 1.0, 0.0)
        self.assertTrue(layer.observe([(0.75, 0.0)], pose, grid, 0.0))
        inflated = layer.build_planning_grid(grid, inflation_radius_m=0.5, time_s=0.0)
        dynamic_cell = grid.world_to_cell(1.75, 1.0)
        self.assertTrue(inflated.occupied(*dynamic_cell))
        self.assertGreaterEqual(inflated.inflation_radius_m, 0.0)

    def test_dynamic_inflation_preserves_free_runtime_start_cell(self) -> None:
        layer = TemporaryObstacleLayer(confirmation_count=1, ttl_s=5.0)
        grid = self.grid()
        pose = Pose2D(1.0, 1.0, 0.0)
        self.assertTrue(layer.observe([(0.10, 0.0)], pose, grid, 0.0))
        planning_grid = layer.build_planning_grid(
            grid,
            inflation_radius_m=0.5,
            time_s=0.0,
            start_xy=(pose.x, pose.y),
        )
        self.assertFalse(planning_grid.occupied(*grid.world_to_cell(pose.x, pose.y)))


class SafetyLayerTask7Tests(unittest.TestCase):
    @staticmethod
    def safety() -> SafetyLayer:
        return SafetyLayer(
            emergency_distance=0.10,
            emergency_release_distance=0.16,
            avoidance_distance=0.15,
            emergency_turn_speed=0.75,
            minimum_turn_steps=3,
            clear_steps_required=2,
            escape_forward_speed=0.045,
            escape_forward_steps=2,
            maximum_recoveries_per_waypoint=2,
            control_period_s=0.1,
        )

    def test_local_avoidance_slows_and_steers(self) -> None:
        safety = self.safety()
        linear, angular, _ = safety.filter_command(
            0.10,
            0.0,
            [(0.14, 0.0), (0.30, 0.20), (0.30, -0.05)],
        )
        self.assertEqual(safety.state, "avoidance")
        self.assertLess(linear, 0.10)
        self.assertNotEqual(angular, 0.0)

    def test_persistent_blockage_requires_tracking_and_confirmation(self) -> None:
        safety = self.safety()
        scan = [(0.14, 0.0)]
        for _ in range(12):
            safety.filter_command(0.10, 0.0, scan)
        self.assertFalse(safety.replan_requested)
        self.assertFalse(safety.consider_blocked_replan("LOCALIZING", True))
        self.assertFalse(safety.replan_requested)
        self.assertTrue(safety.consider_blocked_replan("TRACKING", True))
        self.assertTrue(safety.replan_requested)
        self.assertEqual(safety.replan_reason, "BLOCKED")

    def test_replan_signal_can_be_cleared_and_progress_timeout_detected(self) -> None:
        safety = self.safety()
        self.assertFalse(safety.update_progress(Pose2D(0.0, 0.0, 0.0), (1.0, 0.0), 0.0))
        self.assertTrue(safety.update_progress(Pose2D(0.0, 0.0, 0.0), (1.0, 0.0), 3.1))
        self.assertTrue(safety.consider_blocked_replan("TRACKING", True, 3.1))
        safety.clear_replan_request()
        self.assertFalse(safety.replan_requested)
        self.assertEqual(safety.replan_reason, "NONE")

    def test_in_place_rotation_does_not_trigger_progress_block(self) -> None:
        safety = self.safety()
        clear_scan = [(1.0, 0.0)]
        for step in range(40):
            safety.filter_command(
                0.0,
                0.55,
                clear_scan,
                pose=Pose2D(0.0, 0.0, 0.0),
                target=(1.0, 0.0),
                time_s=step * 0.1,
            )
        self.assertFalse(safety.blocked)
        self.assertFalse(safety.replan_requested)

    def test_local_avoidance_does_not_cancel_in_place_turn(self) -> None:
        safety = self.safety()
        linear, angular, _ = safety.filter_command(
            0.0,
            0.20,
            [(0.14, 0.0)],
        )
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.20)


if __name__ == "__main__":
    unittest.main()
