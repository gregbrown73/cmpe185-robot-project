# A* Planning Guidance

Students implement A* in `controllers/project_starter/planning.py`. Plan on an
inflated copy of the occupancy grid so the path accounts for the robot radius
and a safety margin.

## State and costs

- State: `(row, column)` of a free grid cell
- Four-neighbor move cost: `1`
- Diagonal move cost: `sqrt(2)`
- Heuristic: Euclidean or octile distance to the goal
- Priority: `f(n) = g(n) + h(n)`

## Pseudocode

```text
function A_STAR(grid, start, goal):
    if start or goal is occupied:
        return failure

    open_queue = priority queue containing start with priority 0
    came_from = empty map
    g_score = map with default infinity
    g_score[start] = 0

    while open_queue is not empty:
        current = pop state with the smallest priority

        if current equals goal:
            return RECONSTRUCT_PATH(came_from, current)

        for each valid neighbor of current:
            if neighbor is occupied:
                continue

            if move is diagonal and either adjacent cardinal cell is occupied:
                continue

            tentative_g = g_score[current] + move_cost(current, neighbor)

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                priority = tentative_g + heuristic(neighbor, goal)
                insert or update neighbor in open_queue with priority

    return failure
```

## Integration checklist

1. Convert the estimated metric pose and metric goal into map cells.
2. Plan on an inflated map.
3. Convert the cell path back into metric cell centers.
4. Remove redundant collinear cells or use line-of-sight simplification.
5. Send the simplified waypoints to the provided waypoint follower.
6. Replan when the robot changes goals, the localization estimate changes
   substantially, or the current path stays blocked.
7. Preserve a safe behavior when no path exists.

## Required tests

- start equals goal
- occupied start or goal
- no available path
- narrow passage removed by inflation
- diagonal corner-cutting case
- path length on a small grid with a known optimum

