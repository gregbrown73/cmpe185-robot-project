"""Optional occupancy-grid mapping scaffold for the SLAM extension."""

from __future__ import annotations


class StudentMapper:
    def __init__(self, occupancy_grid) -> None:
        self.grid = occupancy_grid

    def update(self, pose, scan_points_robot) -> None:
        """Update occupancy beliefs from one pose and LiDAR scan.

        STUDENT TODO for teams selecting the mapping or SLAM extension:
        transform scan points into the map frame, trace free cells along each
        ray, update occupied endpoints, and clamp log-odds values.
        """
        _ = pose, scan_points_robot

