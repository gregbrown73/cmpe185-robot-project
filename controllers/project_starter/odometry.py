"""Differential-drive odometry in the CMPE 185 project coordinate frame."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta


class DifferentialDriveOdometry:
    """Midpoint integration from cumulative wheel encoder angles."""

    def __init__(
        self,
        wheel_radius: float,
        axle_length: float,
        initial_pose: tuple[float, float, float],
    ) -> None:
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.pose = Pose2D(*initial_pose)
        self.previous_left: float | None = None
        self.previous_right: float | None = None

    def reset(
        self,
        pose: tuple[float, float, float],
        left_encoder: float | None = None,
        right_encoder: float | None = None,
    ) -> None:
        self.pose = Pose2D(*pose)
        self.previous_left = left_encoder
        self.previous_right = right_encoder

    def update(self, left_encoder: float, right_encoder: float) -> tuple[Pose2D, tuple[float, float]]:
        """Update pose and return `(pose, (delta_distance, delta_heading))`."""
        if self.previous_left is None or self.previous_right is None:
            self.previous_left = left_encoder
            self.previous_right = right_encoder
            return self.pose, (0.0, 0.0)

        delta_left_angle = left_encoder - self.previous_left
        delta_right_angle = right_encoder - self.previous_right
        self.previous_left = left_encoder
        self.previous_right = right_encoder

        delta_left = self.wheel_radius * delta_left_angle
        delta_right = self.wheel_radius * delta_right_angle
        delta_distance = 0.5 * (delta_left + delta_right)
        delta_heading = (delta_right - delta_left) / self.axle_length

        midpoint_heading = self.pose.theta + 0.5 * delta_heading
        self.pose.x += delta_distance * cos(midpoint_heading)
        self.pose.y += delta_distance * sin(midpoint_heading)
        self.pose.theta = wrap_angle(self.pose.theta + delta_heading)
        return self.pose, (delta_distance, delta_heading)

