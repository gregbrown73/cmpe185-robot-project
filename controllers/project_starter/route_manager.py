"""Mission and generated-route state for the dynamic A* controller.

The route manager deliberately does not know about localization, planning-grid
construction, or wheel control.  A caller supplies the current estimated pose,
asks whether a plan is needed, and installs the metric route returned by the
planner.
"""

# TASK 6: Dynamic A* mission route manager.

from __future__ import annotations

from math import hypot
from typing import Iterable, Sequence


LOCALIZING = "LOCALIZING"
PLAN = "PLAN"
FOLLOW = "FOLLOW"
REPLAN = "REPLAN"
NO_PATH = "NO_PATH"
RECOVERING = "RECOVERING"
COMPLETE = "COMPLETE"

NONE = "NONE"
GOAL_CHANGED = "GOAL_CHANGED"
BLOCKED = "BLOCKED"
LOCALIZATION_RECOVERED = "LOCALIZATION_RECOVERED"
PATH_INVALID = "PATH_INVALID"

# These are the only route-manager states allowed by the Task 6 contract.
_STATES = frozenset(
    {LOCALIZING, PLAN, FOLLOW, REPLAN, NO_PATH, RECOVERING, COMPLETE}
)
# These are the only reasons that may trigger a route replacement.
_REPLAN_REASONS = frozenset(
    {NONE, GOAL_CHANGED, BLOCKED, LOCALIZATION_RECOVERED, PATH_INVALID}
)


def _xy(value: object) -> tuple[float, float]:
    """Extract a metric x/y point from a pose or point-like value."""
    # Pose2D objects expose named coordinates; planner routes commonly use
    # two-element tuples or lists, so support both representations.
    if hasattr(value, "x") and hasattr(value, "y"):
        return float(value.x), float(value.y)
    try:
        x, y = value[0], value[1]  # type: ignore[index]
    except (IndexError, KeyError, TypeError) as exc:
        raise TypeError("expected an object or sequence containing x and y") from exc
    return float(x), float(y)


def _goal_point(goal: object) -> tuple[float, float]:
    """Accept the project's named goal dictionaries and plain x/y points."""
    # Mission goals are normally dictionaries loaded from waypoints.json, but
    # accepting plain points keeps the manager easy to unit-test and reuse.
    if isinstance(goal, dict):
        if "position" not in goal:
            raise ValueError("goal dictionaries must contain a 'position' field")
        goal = goal["position"]
    elif hasattr(goal, "position"):
        goal = goal.position
    return _xy(goal)


class RouteManager:
    """Manage one generated route at a time across the A->B->C->HOME mission."""

    def __init__(
        self,
        goals: Iterable[object],
        route_point_tolerance: float = 0.075,
        goal_tolerance: float = 0.20,
    ) -> None:
        # Validate tolerances before storing any mission state.
        if route_point_tolerance < 0.0:
            raise ValueError("route_point_tolerance must be non-negative")
        if goal_tolerance < 0.0:
            raise ValueError("goal_tolerance must be non-negative")

        # Normalize all mission goals to metric x/y coordinates once.
        self._goals = [_goal_point(goal) for goal in goals]
        self.route_point_tolerance = float(route_point_tolerance)
        self.goal_tolerance = float(goal_tolerance)

        # Start in LOCALIZING until the first estimated pose is supplied.
        self.active_goal_index = 0
        self.route: list[tuple[float, float]] = []
        self.route_index = 0
        self.route_version = 0
        self.state = LOCALIZING if self._goals else COMPLETE
        self.replan_reason = NONE
        self._last_pose: tuple[float, float] | None = None

    @property
    def route_count(self) -> int:
        """Number of points in the currently installed route."""
        return len(self.route)

    @property
    def active_goal(self) -> tuple[float, float] | None:
        """Read-only alias useful to telemetry code."""
        return self.current_goal()

    def current_goal(self) -> tuple[float, float] | None:
        """Return the actual mission goal currently being pursued."""
        # Once HOME has been completed, there is no active mission goal.
        if self.active_goal_index >= len(self._goals):
            return None
        return self._goals[self.active_goal_index]

    def current_target(self) -> tuple[float, float] | None:
        """Return the next route point, or ``None`` when no route is available."""
        # Returning None during planning/replanning prevents stale targets from
        # being sent to the waypoint follower.
        if self.route_index >= len(self.route):
            return None
        return self.route[self.route_index]

    def install_route(self, route: Iterable[Sequence[float]]) -> None:
        """Install a planner route and move the manager into ``FOLLOW``.

        The planner normally returns the current start cell as the first metric
        point.  If the most recently observed pose is already close to that
        point, it is removed so the follower does not turn toward its own
        starting position.  A one-point route is retained because that point
        may be the final mission goal and must never be discarded.
        """
        # A completed mission must not be restarted by a late planner result.
        if self.state == COMPLETE:
            raise RuntimeError("cannot install a route after mission completion")

        # Normalize the planner output and discard only a start point that is
        # already reached; keep a one-point final-goal route intact.
        points = [_xy(point) for point in route]
        if points and self._last_pose is not None and len(points) > 1:
            if (
                hypot(
                    points[0][0] - self._last_pose[0],
                    points[0][1] - self._last_pose[1],
                )
                <= self.route_point_tolerance
            ):
                points.pop(0)

        # Installing a route starts a new route version and resets progress.
        self.route = points
        self.route_index = 0
        self.route_version += 1
        self.replan_reason = NONE
        self.state = FOLLOW if self.route else PLAN

    def advance_route(self, pose: object) -> bool:
        """Consume reached route points and return whether progress occurred."""
        # Save the newest pose so a later route installation can identify its
        # already-reached starting point.
        current_pose = _xy(pose)
        self._last_pose = current_pose

        # The first pose moves the state machine into its planning phase.
        if self.state == LOCALIZING:
            self.state = PLAN
        if self.state not in {FOLLOW, PLAN, REPLAN}:
            return False

        # Consume every consecutive route point inside the route tolerance.
        progressed = False
        while self.route_index < len(self.route):
            target = self.route[self.route_index]
            if hypot(target[0] - current_pose[0], target[1] - current_pose[1]) > self.route_point_tolerance:
                break
            self.route_index += 1
            progressed = True

        # Exhausting a route is not the same as reaching the mission goal. If
        # the actual goal is still outside tolerance, request a safe replan.
        if self.route_index >= len(self.route) and self.route:
            # Reaching a route endpoint is not automatically mission
            # completion.  The caller must verify the actual goal coordinate.
            goal = self.current_goal()
            if goal is not None and hypot(goal[0] - current_pose[0], goal[1] - current_pose[1]) > self.goal_tolerance:
                self.state = REPLAN
                self.replan_reason = PATH_INVALID
        return progressed

    def advance_goal(self, pose: object) -> bool:
        """Advance the mission if the actual goal coordinate is within tolerance."""
        # Goal completion is measured against the true mission coordinate, not
        # against a route index or the number of generated waypoints.
        current_pose = _xy(pose)
        self._last_pose = current_pose
        goal = self.current_goal()
        if goal is None:
            self.state = COMPLETE
            return False
        if hypot(goal[0] - current_pose[0], goal[1] - current_pose[1]) > self.goal_tolerance:
            return False

        # Clear the completed leg before selecting the next goal.
        self.active_goal_index += 1
        self.route = []
        self.route_index = 0
        # HOME is the final goal; only then may the manager enter COMPLETE.
        if self.active_goal_index >= len(self._goals):
            self.state = COMPLETE
            self.replan_reason = NONE
        else:
            self.state = PLAN
            self.replan_reason = GOAL_CHANGED
        return True

    def request_replan(self, reason: str) -> None:
        """Request one replan using one of the task's fixed reason strings."""
        # Reject ad-hoc reason strings so diagnostics remain consistent across
        # localization, avoidance, and planning components.
        if reason not in _REPLAN_REASONS or reason == NONE:
            raise ValueError(
                "reason must be GOAL_CHANGED, BLOCKED, "
                "LOCALIZATION_RECOVERED, or PATH_INVALID"
            )
        if self.state == COMPLETE:
            return
        # A route that triggered replanning must not remain available as a
        # drive target while the caller is waiting for a replacement route.
        self.route = []
        self.route_index = 0
        self.replan_reason = reason
        self.state = REPLAN

    def needs_plan(self) -> bool:
        """Return true only during an explicit planning transition."""
        # FOLLOW and NO_PATH must not cause a new A* search every control step.
        return self.state in {PLAN, REPLAN}

    def mark_no_path(self) -> None:
        """Stop route following after a failed plan while remaining recoverable."""
        # Clear all drive targets so a failed plan always stops safely.
        if self.state == COMPLETE:
            return
        self.route = []
        self.route_index = 0
        self.state = NO_PATH


__all__ = [
    "RouteManager",
    "LOCALIZING",
    "PLAN",
    "FOLLOW",
    "REPLAN",
    "NO_PATH",
    "RECOVERING",
    "COMPLETE",
    "NONE",
    "GOAL_CHANGED",
    "BLOCKED",
    "LOCALIZATION_RECOVERED",
    "PATH_INVALID",
]
