from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from coordinate_frames import (  # noqa: E402
    is_non_floor_contact,
    lidar_point_to_robot_xy,
    project_pose_from_webots,
    webots_pose_from_project,
)
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import DifferentialDriveOdometry, Pose2D, wrap_angle  # noqa: E402
from navigation import SafetyLayer, WaypointFollower, unicycle_to_wheels  # noqa: E402
from check_run_health import evaluate as evaluate_run_health  # noqa: E402
from simulate_baseline import run_simulation  # noqa: E402


class OdometryTests(unittest.TestCase):
    def test_straight_motion(self) -> None:
        odometry = DifferentialDriveOdometry(0.02, 0.05, (0.0, 0.0, 0.0))
        odometry.update(0.0, 0.0)
        pose, motion = odometry.update(1.0, 1.0)
        self.assertAlmostEqual(pose.x, 0.02)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 0.0)
        self.assertEqual(motion, (0.02, 0.0))

    def test_in_place_rotation(self) -> None:
        odometry = DifferentialDriveOdometry(0.02, 0.05, (0.0, 0.0, 0.0))
        odometry.update(0.0, 0.0)
        pose, _ = odometry.update(-1.0, 1.0)
        self.assertAlmostEqual(pose.x, 0.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 0.8)

    def test_wrap_angle(self) -> None:
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)


class MapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    def test_dimensions(self) -> None:
        self.assertEqual((self.grid.width, self.grid.height), (80, 60))

    def test_coordinate_round_trip(self) -> None:
        for point in [(-3.2, -2.2), (0.7, 0.34), (3.2, 2.2)]:
            cell = self.grid.world_to_cell(*point)
            center = self.grid.cell_to_world(*cell)
            self.assertLess(abs(center[0] - point[0]), self.grid.resolution)
            self.assertLess(abs(center[1] - point[1]), self.grid.resolution)

    def test_mission_points_are_free(self) -> None:
        data = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
        points = [data["home"]["position"]]
        points.extend(goal["position"] for goal in data["delivery_goals"])
        points.extend(data["baseline_waypoints"])
        self.assertTrue(all(not self.grid.world_occupied(*point) for point in points))

    def test_baseline_segments_do_not_cross_static_walls(self) -> None:
        data = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
        route = data["baseline_waypoints"]
        for start, end in zip(route, route[1:]):
            samples = max(1, math.ceil(math.dist(start, end) / 0.02))
            for index in range(samples + 1):
                fraction = index / samples
                x = start[0] + fraction * (end[0] - start[0])
                y = start[1] + fraction * (end[1] - start[1])
                self.assertFalse(self.grid.world_occupied(x, y), (start, end, x, y))


class CoordinateFrameTests(unittest.TestCase):
    def test_lidar_flu_coordinates_are_not_swapped_or_negated(self) -> None:
        self.assertEqual(lidar_point_to_robot_xy(2.0, 0.5, 0.01), (2.0, 0.5))
        self.assertEqual(lidar_point_to_robot_xy(1.0, -0.3, -0.02), (1.0, -0.3))

    def test_lidar_rejects_nonfinite_planar_coordinates(self) -> None:
        self.assertIsNone(lidar_point_to_robot_xy(math.inf, 0.0, 0.0))
        self.assertIsNone(lidar_point_to_robot_xy(0.0, math.nan, 0.0))

    def test_webots_pose_identity_heading(self) -> None:
        pose = project_pose_from_webots(
            [-3.2, -2.2, 0.0],
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(pose[:2], (-3.2, -2.2))
        self.assertAlmostEqual(pose[2], 0.0)

    def test_webots_pose_quarter_turn_heading(self) -> None:
        pose = project_pose_from_webots(
            [0.9, -2.1, 0.0],
            [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertAlmostEqual(pose[2], 0.5 * math.pi)

    def test_project_pose_maps_to_xy_and_z_axis_rotation(self) -> None:
        translation, rotation = webots_pose_from_project(0.9, -2.1, math.pi)
        self.assertEqual(translation, [0.9, -2.1, 0.0])
        self.assertEqual(rotation[:3], [0.0, 0.0, 1.0])
        self.assertAlmostEqual(rotation[3], math.pi)

    def test_floor_contacts_are_excluded_by_id_or_height(self) -> None:
        self.assertFalse(is_non_floor_contact(10, 10, 0.02, 0.005))
        self.assertFalse(is_non_floor_contact(11, 10, 0.001, 0.005))
        self.assertTrue(is_non_floor_contact(11, 10, 0.03, 0.005))


class NavigationTests(unittest.TestCase):
    @staticmethod
    def follower() -> WaypointFollower:
        return WaypointFollower(
            cruise_speed=0.10,
            slowdown_distance=0.35,
            minimum_drive_speed=0.025,
            rotate_enter_heading_error=0.30,
            rotate_exit_heading_error=0.08,
            gain_heading=1.60,
            maximum_angular_speed=0.75,
        )

    def test_large_corner_rotates_before_driving(self) -> None:
        follower = self.follower()
        linear, angular = follower.command(
            pose=Pose2D(0.0, 0.0, 0.0),
            target=(0.0, 1.0),
        )
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.75)
        self.assertEqual(follower.state, "rotate")

    def test_waypoint_route_completes_in_kinematic_smoke_test(self) -> None:
        data = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
        route = [tuple(point) for point in data["baseline_waypoints"]]
        pose = Pose2D(-3.2, -2.2, 0.0)
        follower = self.follower()
        active = 0
        dt = 0.032
        for _ in range(18_000):
            while active < len(route) and follower.distance_to(pose, route[active]) <= 0.13:
                active += 1
            if active >= len(route):
                break
            linear, angular = follower.command(pose, route[active])
            midpoint = pose.theta + 0.5 * angular * dt
            pose.x += linear * dt * math.cos(midpoint)
            pose.y += linear * dt * math.sin(midpoint)
            pose.theta = wrap_angle(pose.theta + angular * dt)
        self.assertEqual(active, len(route))

    def test_emergency_turn_direction_is_latched_until_clear(self) -> None:
        safety = SafetyLayer(
            emergency_distance=0.10,
            emergency_release_distance=0.16,
            avoidance_distance=0.23,
            emergency_turn_speed=0.75,
            minimum_turn_steps=3,
            clear_steps_required=2,
            escape_forward_speed=0.045,
            escape_forward_steps=2,
            maximum_recoveries_per_waypoint=2,
        )
        first_scan = [(0.08, 0.01), (0.20, 0.18), (0.45, -0.10)]
        _, first_turn, _ = safety.filter_command(0.10, 0.0, first_scan)
        self.assertEqual(safety.state, "emergency_turn")
        first_direction = safety.turn_direction

        # Opposite lateral evidence on the next scan must not reverse the turn.
        opposite_scan = [(0.08, -0.01), (0.45, 0.10), (0.20, -0.18)]
        _, second_turn, _ = safety.filter_command(0.10, 0.0, opposite_scan)
        self.assertEqual(safety.turn_direction, first_direction)
        self.assertEqual(first_turn, second_turn)

        clear_scan = [(0.50, 0.0)]
        safety.filter_command(0.10, 0.0, clear_scan)
        linear, _, _ = safety.filter_command(0.10, 0.0, clear_scan)
        self.assertFalse(safety.emergency_active)
        self.assertEqual(safety.state, "escape_forward")
        self.assertAlmostEqual(linear, 0.045)

        linear, _, _ = safety.filter_command(0.10, 0.0, clear_scan)
        self.assertEqual(safety.state, "escape_forward")
        self.assertAlmostEqual(linear, 0.045)

        linear, _, _ = safety.filter_command(0.10, 0.0, clear_scan)
        self.assertEqual(safety.state, "clear")
        self.assertAlmostEqual(linear, 0.10)

    def test_wheel_speed_conversion_respects_limit(self) -> None:
        left, right = unicycle_to_wheels(0.10, 0.55, 0.0200, 0.0574, 6.28)
        self.assertLessEqual(max(abs(left), abs(right)), 6.28)

    def test_nominal_reference_simulation_returns_home(self) -> None:
        summary = run_simulation(lidar_ray_count=36)
        self.assertTrue(summary["passed"], summary)
        self.assertEqual(summary["route_waypoints_completed"], 23)
        self.assertEqual(summary["ordered_goals_reached"], 4)
        self.assertGreater(summary["minimum_body_clearance_m"], 0.06)


class RunHealthTests(unittest.TestCase):
    @staticmethod
    def row(time_s: float, path_m: float, waypoint: int, safety: str = "clear") -> dict:
        emergency = safety == "emergency_turn"
        return {
            "time_s": str(time_s),
            "ground_truth_path_length_m": str(path_m),
            "contact_now": "0",
            "lidar_point_count": "350",
            "minimum_lidar_range_m": "0.5",
            "left_wheel_speed_rad_s": "1.0",
            "right_wheel_speed_rad_s": "1.0",
            "commanded_linear_speed_m_s": "0.0" if emergency else "0.10",
            "commanded_angular_speed_rad_s": "0.75" if emergency else "0.0",
            "safety_state": safety,
            "active_waypoint": str(waypoint),
            "mission_complete_reported": "0",
            "goals_reached_ground_truth": "0",
            "collision_episodes": "0",
            "gt_x_m": "0.0",
            "gt_y_m": "0.0",
            "position_error_m": "0.0",
        }

    def test_detects_motion_followed_by_emergency_deadlock(self) -> None:
        rows = []
        for second in range(61):
            if second <= 30:
                rows.append(self.row(second, 0.09 * second, 2))
            else:
                rows.append(self.row(second, 2.70, 2, "emergency_turn"))
        passed, issues, _ = evaluate_run_health(rows)
        self.assertFalse(passed)
        self.assertTrue(any("stalled" in issue for issue in issues))
        self.assertTrue(any("emergency turns" in issue for issue in issues))

    def test_accepts_continuing_baseline_progress(self) -> None:
        rows = [self.row(second, 0.075 * second, 1 + second // 15) for second in range(61)]
        passed, issues, _ = evaluate_run_health(rows)
        self.assertTrue(passed, issues)

    def test_full_gate_requires_ground_truth_completion_at_home(self) -> None:
        rows = [self.row(second, 0.075 * second, min(23, 1 + second // 2)) for second in range(61)]
        rows[-1].update(
            {
                "active_waypoint": "23",
                "mission_complete_reported": "1",
                "goals_reached_ground_truth": "4",
                "gt_x_m": "-3.15",
                "gt_y_m": "-2.18",
            }
        )
        passed, issues, metrics = evaluate_run_health(rows, require_complete=True)
        self.assertTrue(passed, issues)
        self.assertLess(metrics["final_home_error_m"], 0.20)

    def test_full_gate_rejects_early_stop(self) -> None:
        rows = [self.row(second, 0.075 * second, 1 + second // 15) for second in range(61)]
        passed, issues, _ = evaluate_run_health(rows, require_complete=True)
        self.assertFalse(passed)
        self.assertTrue(any("never reported mission completion" in issue for issue in issues))


class WorldStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = (PROJECT_ROOT / "worlds" / "cmpe185_indoor_delivery.wbt").read_text(encoding="utf-8")

    def test_enu_floor_is_xy_with_z_thickness(self) -> None:
        self.assertIn('coordinateSystem "ENU"', self.world)
        self.assertIn("orientation -0.57735 0.57735 0.57735 2.0944", self.world)
        self.assertIn("position 0 0 10.5", self.world)
        self.assertIn("translation 0 0 -0.025", self.world)
        self.assertIn("size 8.2 6.2 0.05", self.world)
        self.assertNotIn("size 8.2 0.05 6.2", self.world)

    def test_robot_and_lidar_use_z_up_layout(self) -> None:
        self.assertIn("translation -3.2 -2.2 0", self.world)
        self.assertIn("rotation 0 0 1 0", self.world)
        self.assertIn("translation 0 0 0.055", self.world)
        self.assertNotIn("translation -3.2 0 2.2", self.world)

    def test_outer_walls_are_vertical_in_z(self) -> None:
        self.assertIn("size 0.10 6.0 0.35", self.world)
        self.assertIn("size 8.0 0.10 0.35", self.world)


if __name__ == "__main__":
    unittest.main()
