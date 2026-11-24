# Run with: python3 -m unittest tests.test_task6 -v
# TASK 6: Route-manager unit tests.

"""Tests for the Task 6 dynamic A* route manager."""

from __future__ import annotations

import unittest

from controllers.project_starter.route_manager import (
    BLOCKED,
    COMPLETE,
    FOLLOW,
    GOAL_CHANGED,
    NO_PATH,
    NONE,
    PATH_INVALID,
    PLAN,
    REPLAN,
    RouteManager,
)


class RouteManagerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.goals = [
            {"name": "A", "position": (1.0, 0.0)},
            {"name": "B", "position": (2.0, 0.0)},
            {"name": "C", "position": (3.0, 0.0)},
            {"name": "HOME", "position": (0.0, 0.0)},
        ]
        self.manager = RouteManager(self.goals)

    def test_initial_pose_enters_plan_and_route_is_followed(self) -> None:
        self.assertEqual(self.manager.state, "LOCALIZING")
        self.assertFalse(self.manager.needs_plan())
        self.manager.advance_route((0.0, 0.0))
        self.assertEqual(self.manager.state, PLAN)
        self.assertTrue(self.manager.needs_plan())

        self.manager.install_route([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
        self.assertEqual(self.manager.state, FOLLOW)
        self.assertEqual(self.manager.route, [(0.5, 0.0), (1.0, 0.0)])
        self.assertEqual(self.manager.current_target(), (0.5, 0.0))
        self.assertEqual(self.manager.route_version, 1)

    def test_single_final_goal_point_is_never_dropped(self) -> None:
        self.manager.advance_route((0.0, 0.0))
        self.manager.install_route([(0.05, 0.0)])
        self.assertEqual(self.manager.route, [(0.05, 0.0)])

    def test_goal_completion_uses_actual_goal_not_route_index(self) -> None:
        self.manager.advance_route((0.0, 0.0))
        self.manager.install_route([(0.0, 0.0), (0.79, 0.0)])
        self.manager.advance_route((0.79, 0.0))
        self.assertEqual(self.manager.route_index, 1)
        self.assertFalse(self.manager.advance_goal((0.79, 0.0)))
        self.assertEqual(self.manager.active_goal_index, 0)

        self.assertTrue(self.manager.advance_goal((1.0, 0.0)))
        self.assertEqual(self.manager.active_goal_index, 1)
        self.assertEqual(self.manager.state, PLAN)
        self.assertEqual(self.manager.replan_reason, GOAL_CHANGED)

    def test_replan_reasons_and_no_path_are_recoverable(self) -> None:
        self.manager.advance_route((0.0, 0.0))
        self.manager.install_route([(0.0, 0.0), (1.0, 0.0)])
        self.manager.request_replan(BLOCKED)
        self.assertEqual(self.manager.state, REPLAN)
        self.assertEqual(self.manager.replan_reason, BLOCKED)
        self.assertIsNone(self.manager.current_target())
        self.manager.mark_no_path()
        self.assertEqual(self.manager.state, NO_PATH)
        self.manager.request_replan(PATH_INVALID)
        self.assertTrue(self.manager.needs_plan())

    def test_mission_completes_only_after_home(self) -> None:
        self.manager.advance_route((0.0, 0.0))
        for point in ((1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (0.0, 0.0)):
            self.assertTrue(self.manager.advance_goal(point))
        self.assertEqual(self.manager.state, COMPLETE)
        self.assertIsNone(self.manager.current_goal())
        self.assertEqual(self.manager.replan_reason, NONE)


if __name__ == "__main__":
    unittest.main()
