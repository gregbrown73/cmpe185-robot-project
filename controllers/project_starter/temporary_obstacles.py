"""Temporary planning obstacles built from confirmed LiDAR observations."""

# TASK 7: Temporary obstacle confirmation, expiry, and planning-map support.

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, hypot, isfinite, sin, sqrt

from occupancy_grid import OccupancyGrid


@dataclass
class _PendingCell:
    """Confirmation bookkeeping for one dynamic-obstacle cell."""

    count: int
    last_seen_s: float


def _pose_xytheta(pose: object) -> tuple[float, float, float]:
    """Read a project Pose2D or a three-value pose sequence."""
    if hasattr(pose, "x") and hasattr(pose, "y") and hasattr(pose, "theta"):
        return float(pose.x), float(pose.y), float(pose.theta)
    try:
        return float(pose[0]), float(pose[1]), float(pose[2])  # type: ignore[index]
    except (IndexError, KeyError, TypeError) as exc:
        raise TypeError("pose must provide x, y, and theta") from exc


class TemporaryObstacleLayer:
    """Track confirmed dynamic cells without modifying the static map."""

    def __init__(self, confirmation_count: int = 3, ttl_s: float = 5.0) -> None:
        # Confirmation and expiry are deliberately conservative so one noisy
        # LiDAR return cannot permanently alter the planning map.
        if confirmation_count < 1:
            raise ValueError("confirmation_count must be positive")
        if ttl_s <= 0.0:
            raise ValueError("ttl_s must be positive")
        self.confirmation_count = int(confirmation_count)
        self.ttl_s = float(ttl_s)
        self._pending: dict[tuple[int, int], _PendingCell] = {}
        self._active: dict[tuple[int, int], float] = {}
        self.version = 0

    @staticmethod
    def _neighboring(a: tuple[int, int], b: tuple[int, int]) -> bool:
        """Return true when cells are the same or directly neighboring."""
        return max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= 1

    @staticmethod
    def _find_neighbor(
        cells: dict[tuple[int, int], object],
        cell: tuple[int, int],
    ) -> tuple[int, int] | None:
        """Find a same/neighboring cell in a pending or active collection."""
        return next((candidate for candidate in cells if TemporaryObstacleLayer._neighboring(candidate, cell)), None)

    @staticmethod
    def _world_point(
        pose: tuple[float, float, float],
        point_robot: tuple[float, float],
    ) -> tuple[float, float]:
        """Transform robot FLU coordinates into the project world frame."""
        x, y, theta = pose
        forward, left = point_robot
        c = cos(theta)
        s = sin(theta)
        return x + c * forward - s * left, y + s * forward + c * left

    @staticmethod
    def _near_static_occupied(
        static_grid: OccupancyGrid,
        world_x: float,
        world_y: float,
        radius_m: float = 0.15,
    ) -> bool:
        """Ignore observations on or immediately beside known static walls."""
        cell = static_grid.world_to_cell(world_x, world_y)
        if not static_grid.in_bounds(*cell):
            return True
        radius_cells = ceil(radius_m / static_grid.resolution) + 1
        for row in range(cell[0] - radius_cells, cell[0] + radius_cells + 1):
            for col in range(cell[1] - radius_cells, cell[1] + radius_cells + 1):
                if not static_grid.in_bounds(row, col) or not static_grid.occupied(row, col):
                    continue
                center_x, center_y = static_grid.cell_to_world(row, col)
                if hypot(world_x - center_x, world_y - center_y) <= radius_m + sqrt(2.0) * static_grid.resolution / 2.0:
                    return True
        return False

    def _expire(self, time_s: float) -> bool:
        """Remove expired confirmed cells and advance the planning version."""
        expired = [cell for cell, expiry in self._active.items() if time_s >= expiry]
        for cell in expired:
            del self._active[cell]
        if expired:
            self.version += 1
        return bool(expired)

    def observe(
        self,
        scan_points_robot: list[tuple[float, float]],
        pose: object,
        static_grid: OccupancyGrid,
        time_s: float,
        localization_state: str = "TRACKING",
    ) -> bool:
        """Add confirmed free-space hits to the temporary planning layer.

        The optional localization argument preserves the four-argument Task 7
        interface while allowing integration to suppress observations unless
        MCL is in the required TRACKING state.
        """
        now = float(time_s)
        changed = self._expire(now)

        # Stale unconfirmed evidence does not accumulate indefinitely.
        self._pending = {
            cell: pending
            for cell, pending in self._pending.items()
            if now - pending.last_seen_s <= self.ttl_s
        }
        if localization_state != "TRACKING":
            return changed

        robot_pose = _pose_xytheta(pose)
        # Collapse multiple returns from one scan into one observation per
        # neighboring-cell cluster, preventing a single scan from confirming a
        # blocker three times.
        candidates: list[tuple[int, int]] = []
        for point in scan_points_robot:
            try:
                forward, left = float(point[0]), float(point[1])
            except (IndexError, TypeError, ValueError):
                continue
            if not isfinite(forward) or not isfinite(left) or hypot(forward, left) < 0.05:
                continue
            world_x, world_y = self._world_point(robot_pose, (forward, left))
            cell = static_grid.world_to_cell(world_x, world_y)
            if not static_grid.in_bounds(*cell) or self._near_static_occupied(static_grid, world_x, world_y):
                continue
            if not any(self._neighboring(existing, cell) for existing in candidates):
                candidates.append(cell)

        for cell in candidates:
            active_cell = self._find_neighbor(self._active, cell)
            if active_cell is not None:
                # Reobserving an active blocker refreshes its TTL but does not
                # create a new planning-grid version.
                self._active[active_cell] = now + self.ttl_s
                continue

            pending_cell = self._find_neighbor(self._pending, cell)
            if pending_cell is None:
                pending_cell = cell
                self._pending[pending_cell] = _PendingCell(0, now)
            pending = self._pending[pending_cell]
            pending.count += 1
            pending.last_seen_s = now
            if pending.count >= self.confirmation_count:
                self._active[pending_cell] = now + self.ttl_s
                del self._pending[pending_cell]
                self.version += 1
                changed = True
        return changed

    def build_planning_grid(
        self,
        static_grid: OccupancyGrid,
        inflation_radius_m: float = 0.20,
        time_s: float = 0.0,
    ) -> OccupancyGrid:
        """Combine raw static and temporary occupancy, then inflate exactly once."""
        self._expire(float(time_s))
        combined_cells = [row[:] for row in static_grid.cells]
        for row, col in self._active:
            if static_grid.in_bounds(row, col):
                combined_cells[row][col] = static_grid.occupied_value
        combined = OccupancyGrid(
            combined_cells,
            static_grid.resolution,
            static_grid.origin_x,
            static_grid.origin_y,
            static_grid.occupied_value,
            inflation_radius_m=0.0,
        )
        return combined.inflated(float(inflation_radius_m))

    def active_count(self, time_s: float) -> int:
        """Return the number of active confirmed temporary cells."""
        self._expire(float(time_s))
        return len(self._active)


__all__ = ["TemporaryObstacleLayer"]
