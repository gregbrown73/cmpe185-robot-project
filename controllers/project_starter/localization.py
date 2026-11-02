"""Localization interface and student implementation scaffold."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random

from odometry import Pose2D


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float


class StudentLocalizer:
    """Replace the odometry pass-through with Monte Carlo Localization.

    The runnable baseline returns the odometry pose. The class already defines
    the data flow that the main controller expects, so teams can implement the
    particle-filter stages without rewriting the Webots device code.
    """

    def __init__(self, initial_pose: tuple[float, float, float], particle_count: int = 300) -> None:
        self.initial_pose = initial_pose
        self.particle_count = particle_count
        self.random = Random(185)
        self.particles: list[Particle] = []

    def initialize(self, occupancy_grid) -> None:
        """Initialize particles over a region or the free space.

        STUDENT TODO:
        1. Select an initial belief for each required test scenario.
        2. Reject particles that fall in occupied cells.
        3. Give valid particles normalized weights.
        """
        self.particles = []

    def update(
        self,
        odometry_pose: Pose2D,
        delta_motion: tuple[float, float],
        scan_points_robot: list[tuple[float, float]],
        occupancy_grid,
    ) -> Pose2D:
        """Return the current best pose estimate.

        STUDENT TODO:
        - Sample the motion model using `delta_motion`.
        - Evaluate a LiDAR measurement likelihood on the supplied map.
        - Normalize weights and monitor effective sample size.
        - Resample when appropriate.
        - Estimate the pose, treating heading as a circular quantity.
        - Add a recovery mechanism for the kidnapped-robot test.

        The pass-through below keeps the starter project runnable.
        """
        _ = delta_motion, scan_points_robot, occupancy_grid
        return Pose2D(odometry_pose.x, odometry_pose.y, odometry_pose.theta)

