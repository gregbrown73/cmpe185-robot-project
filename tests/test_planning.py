# Tests for the Person 2 A* global planner
# Run with: python -m unittest tests.test_planning -v

from __future__ import annotations

import math
import unittest

from controllers.project_starter.occupancy_grid import OccupancyGrid
from controllers.project_starter.planning import (
    INVALID_GOAL,
    INVALID_START,
    NO_PATH,
    SUCCESS,
    astar,
    plan_world_path,
)


class AStarTests(unittest.TestCase):
    def test_start_equals_goal(self) -> None:
        # Tests when the robot is already at its destination.
        # The planner should return the starting location as the path.
        grid = [[0, 0], [0, 0]]
        self.assertEqual(astar(grid, (0, 0), (0, 0)), [(0, 0)])

    def test_occupied_start_and_goal_return_no_path(self) -> None:
        # Tests when the robot's start or goal is on an obstacle.
        # The planner should return no path.
        grid = [[1, 0], [0, 1]]
        self.assertEqual(astar(grid, (0, 0), (1, 1)), [])

    def test_outside_map_start_and_goal_return_no_path(self) -> None:
        # Tests when the start or goal coordinates are outside the map.
        # The planner should reject them and return no path.
        grid = [[0, 0], [0, 0]]
        self.assertEqual(astar(grid, (-1, 0), (1, 1)), [])
        self.assertEqual(astar(grid, (0, 0), (2, 1)), [])

    def test_no_path_returns_empty_list(self) -> None:
        # Places a wall between the robot and its destination.
        # A* should realize that there is no possible route.
        grid = [
            [0, 1, 0],
            [0, 1, 0],
            [0, 1, 0],
        ]
        self.assertEqual(astar(grid, (1, 0), (1, 2)), [])

    def test_open_grid_uses_known_optimal_diagonal_path(self) -> None:
        # Gives A* an empty map.
        # It should choose the shortest diagonal path to the goal.
        grid = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        path = astar(grid, (0, 0), (2, 2))
        self.assertEqual(path, [(0, 0), (1, 1), (2, 2)])

    def test_diagonal_corner_cutting_is_rejected(self) -> None:
        # Puts obstacles on both sides of a diagonal opening.
        # The robot should not squeeze diagonally between the obstacles.
        grid = [
            [0, 1],
            [1, 0],
        ]
        self.assertEqual(astar(grid, (0, 0), (1, 1)), [])


class WorldPlannerTests(unittest.TestCase):
    @staticmethod
    def grid(cells: list[list[int]]) -> OccupancyGrid:
        # Use 0/1 values in these fixtures, with 1 meaning occupied.
        return OccupancyGrid(
            cells,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            occupied_value=1,
        )

    def test_world_and_grid_coordinate_conversion(self) -> None:
        # Checks conversion between real-world positions and grid cells.
        # Real-world coordinates, such as meters, must convert correctly to
        # grid cells and back.
        grid = self.grid([[0, 0], [0, 0]])
        self.assertEqual(grid.world_to_cell(0.2, 0.2), (1, 0))
        self.assertEqual(grid.cell_to_world(1, 0), (0.5, 0.5))

    def test_invalid_world_start_and_goal_have_distinct_statuses(self) -> None:
        # Gives the planner invalid real-world coordinates.
        # It should distinguish between an invalid start and an invalid goal.
        grid = self.grid([[0, 0], [0, 0]])
        self.assertEqual(plan_world_path(grid, (-1.0, 0.5), (1.5, 1.5))[1], INVALID_START)
        self.assertEqual(plan_world_path(grid, (0.5, 0.5), (2.0, 1.5))[1], INVALID_GOAL)

    def test_world_planner_appends_exact_goal(self) -> None:
        # Checks that the final path ends at the exact real-world goal.
        # The route should end at the requested coordinate, rather than only
        # at the center of the goal's grid cell.
        grid = self.grid([[0, 0], [0, 0]])
        goal = (1.25, 1.25)
        route, status = plan_world_path(grid, (0.25, 0.25), goal)
        self.assertEqual(status, SUCCESS)
        self.assertEqual(route[-1], goal)

    def test_inflation_removes_narrow_passage(self) -> None:
        # Enlarges obstacles to account for the robot's physical size.
        # A gap that is too narrow for the robot should be treated as blocked.
        # Two nearby obstacles leave a one-cell gap before inflation.
        grid = self.grid(
            [
                [0, 0, 0, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 0, 0, 0, 0],
            ]
        )
        inflated = grid.inflated(1.0)
        self.assertEqual(astar(inflated, (1, 0), (1, 4)), [])

    def test_simplified_route_segments_remain_collision_free(self) -> None:
        # Checks the simplified route after unnecessary points are removed.
        # The new straight-line segments must not pass through obstacles.
        grid = self.grid(
            [
                [0, 0, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        )
        route, status = plan_world_path(grid, (0.5, 0.5), (3.5, 3.5))
        self.assertEqual(status, SUCCESS)
        for start, end in zip(route, route[1:]):
            distance = math.dist(start, end)
            samples = max(1, math.ceil(distance / 0.1))
            for index in range(samples + 1):
                fraction = index / samples
                point = (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
                self.assertFalse(grid.world_occupied(*point), (start, end, point))


if __name__ == "__main__":
    unittest.main()
