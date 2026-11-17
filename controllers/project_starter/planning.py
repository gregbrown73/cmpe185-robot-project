"""Grid planning interfaces for the CMPE 185 project."""

from __future__ import annotations

import heapq
from math import hypot, sqrt


SUCCESS = "SUCCESS"
NO_PATH = "NO_PATH"
INVALID_START = "INVALID_START"
INVALID_GOAL = "INVALID_GOAL"


def reconstruct_path(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    # Follow parent links from the goal back to the start.
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

    STUDENT TODO guidance: implement A* using the pseudocode in
    `docs/astar_pseudocode.md`. Use the inflated map, reject diagonal corner
    cutting, and return an empty list when no path exists.

    The grid is expected to be the planning grid supplied by the caller.  In
    particular, callers should inflate the combined static/dynamic map once
    before invoking this function.
    """
    # Reject invalid or blocked endpoints before searching.
    if not _valid_cell(occupancy_grid, start) or _occupied(occupancy_grid, start):
        return []
    if not _valid_cell(occupancy_grid, goal) or _occupied(occupancy_grid, goal):
        return []
    if start == goal:
        return [start]

    # Estimate remaining cost with the octile-distance heuristic.
    def heuristic(cell: tuple[int, int]) -> float:
        row_delta = abs(cell[0] - goal[0])
        col_delta = abs(cell[1] - goal[1])
        diagonal = min(row_delta, col_delta)
        straight = max(row_delta, col_delta) - diagonal
        return diagonal * sqrt(2.0) + straight

    # Allow four cardinal moves and four diagonal moves.
    neighbors = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, sqrt(2.0)), (-1, 1, sqrt(2.0)),
        (1, -1, sqrt(2.0)), (1, 1, sqrt(2.0)),
    )
    # The priority queue stores cells ordered by estimated total cost.
    open_queue = [(heuristic(start), 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    while open_queue:
        _, current_g, current = heapq.heappop(open_queue)
        # Ignore stale queue entries left behind by a better path update.
        if current_g > g_score.get(current, float("inf")):
            continue
        if current == goal:
            return reconstruct_path(came_from, current)

        # Examine each possible next cell.
        for row_delta, col_delta, move_cost in neighbors:
            neighbor = (current[0] + row_delta, current[1] + col_delta)
            if not _valid_cell(occupancy_grid, neighbor) or _occupied(occupancy_grid, neighbor):
                continue
            if row_delta and col_delta:
                # Prevent diagonal movement through blocked corners.
                side_a = (current[0] + row_delta, current[1])
                side_b = (current[0], current[1] + col_delta)
                if _occupied(occupancy_grid, side_a) or _occupied(occupancy_grid, side_b):
                    continue

            tentative_g = current_g + move_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                # Save the better route and add the neighbor to the queue.
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(
                    open_queue,
                    (tentative_g + heuristic(neighbor), tentative_g, neighbor),
                )
    return []


def _valid_cell(grid, cell: tuple[int, int]) -> bool:
    # Support both OccupancyGrid objects and simple nested test lists.
    row, col = cell
    if hasattr(grid, "in_bounds"):
        return bool(grid.in_bounds(row, col))
    return bool(grid) and 0 <= row < len(grid) and 0 <= col < len(grid[row])


def _occupied(grid, cell: tuple[int, int]) -> bool:
    # Treat outside-map cells as blocked for safety.
    row, col = cell
    if hasattr(grid, "occupied"):
        return bool(grid.occupied(row, col))
    if not _valid_cell(grid, cell):
        return True
    return bool(grid[row][col])


def _line_is_clear(grid, start_xy: tuple[float, float], end_xy: tuple[float, float]) -> bool:
    # Sample a world-space segment to check for collisions.
    resolution = float(getattr(grid, "resolution", 1.0))
    distance = hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    samples = max(1, int(distance / max(resolution * 0.25, 1e-9)))
    for index in range(samples + 1):
        fraction = index / samples
        x = start_xy[0] + fraction * (end_xy[0] - start_xy[0])
        y = start_xy[1] + fraction * (end_xy[1] - start_xy[1])
        if hasattr(grid, "world_occupied"):
            occupied = grid.world_occupied(x, y)
        else:
            cell = grid.world_to_cell(x, y)
            occupied = _occupied(grid, cell)
        if occupied:
            return False
    return True


def _simplify_grid_path(grid, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    # Remove cells only when a direct shortcut is collision-free.
    if len(cells) < 3:
        return cells[:]
    points = [grid.cell_to_world(*cell) for cell in cells]
    simplified = [points[0]]
    anchor = 0
    for candidate in range(2, len(points)):
        if not _line_is_clear(grid, points[anchor], points[candidate]):
            simplified.append(points[candidate - 1])
            anchor = candidate - 1
    simplified.append(points[-1])
    return [grid.world_to_cell(*point) for point in simplified]


def plan_world_path(planning_grid, start_xy, goal_xy):
    """Return a collision-free metric route and a diagnostic status.

    ``planning_grid`` must contain the combined occupancy already inflated by
    the project's canonical 0.12 m radius.
    """
    # Convert the metric start position into a grid cell.
    try:
        start = planning_grid.world_to_cell(*start_xy)
    except (TypeError, ValueError, IndexError):
        return [], INVALID_START
    if not _valid_cell(planning_grid, start) or _occupied(planning_grid, start):
        return [], INVALID_START

    # Convert and validate the metric mission goal.
    try:
        goal = planning_grid.world_to_cell(*goal_xy)
    except (TypeError, ValueError, IndexError):
        return [], INVALID_GOAL
    if not _valid_cell(planning_grid, goal) or _occupied(planning_grid, goal):
        return [], INVALID_GOAL

    # Run A*, simplify the safe path, and convert it back to world points.
    cell_path = astar(planning_grid, start, goal)
    if not cell_path:
        return [], NO_PATH
    cell_path = _simplify_grid_path(planning_grid, cell_path)
    route = [planning_grid.cell_to_world(*cell) for cell in cell_path]
    # Append the exact requested goal when the final segment is safe.
    if not _line_is_clear(planning_grid, route[-1], goal_xy):
        return [], NO_PATH
    if hypot(route[-1][0] - goal_xy[0], route[-1][1] - goal_xy[1]) > 1e-12:
        route.append((float(goal_xy[0]), float(goal_xy[1])))
    return route, SUCCESS


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

