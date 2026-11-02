"""Grid planning interfaces for the CMPE 185 project."""

from __future__ import annotations

from math import hypot


def reconstruct_path(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(
    occupancy_grid,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    """Plan a collision-free grid path.

    STUDENT TODO: implement A* using the pseudocode in
    `docs/astar_pseudocode.md`. Use the inflated map, reject diagonal corner
    cutting, and return an empty list when no path exists.
    """
    _ = occupancy_grid, start, goal
    raise NotImplementedError("Students implement A* in planning.py")


def simplify_world_path(path: list[tuple[float, float]], minimum_spacing: float = 0.15) -> list[tuple[float, float]]:
    """Remove waypoints that are much closer than the controller needs."""
    if not path:
        return []
    simplified = [path[0]]
    for point in path[1:-1]:
        if hypot(point[0] - simplified[-1][0], point[1] - simplified[-1][1]) >= minimum_spacing:
            simplified.append(point)
    if len(path) > 1:
        simplified.append(path[-1])
    return simplified

