"""Waypoint following and the deliberately limited starter safety layer.

The nominal baseline rotates before large heading changes, reduces speed near
each waypoint, and uses a calibrated e-puck drive model.  The safety layer can
turn away and move clear of a dangerously close surface, but it deliberately
does not plan around a blocked route.  Students replace it with the full local
avoidance and replanning method taught later in the course.
"""

from __future__ import annotations

from math import atan2, cos, hypot, inf

from odometry import Pose2D, wrap_angle


class WaypointFollower:
    """A conservative waypoint controller in unicycle form.

    A small hysteresis band separates ``rotate`` and ``drive`` modes.  The
    robot therefore finishes a corner before translating through a doorway.
    """

    def __init__(
        self,
        cruise_speed: float,
        slowdown_distance: float,
        minimum_drive_speed: float,
        rotate_enter_heading_error: float,
        rotate_exit_heading_error: float,
        gain_heading: float,
        maximum_angular_speed: float,
    ) -> None:
        self.cruise_speed = cruise_speed
        self.slowdown_distance = max(0.01, slowdown_distance)
        self.minimum_drive_speed = max(0.0, min(minimum_drive_speed, cruise_speed))
        self.rotate_enter_heading_error = rotate_enter_heading_error
        self.rotate_exit_heading_error = rotate_exit_heading_error
        self.gain_heading = gain_heading
        self.maximum_angular_speed = maximum_angular_speed
        self.rotating = False
        self.state = "drive"
        self.last_distance = 0.0
        self.last_heading_error = 0.0

    @staticmethod
    def distance_to(pose: Pose2D, target: tuple[float, float]) -> float:
        return hypot(target[0] - pose.x, target[1] - pose.y)

    def command(self, pose: Pose2D, target: tuple[float, float]) -> tuple[float, float]:
        distance = self.distance_to(pose, target)
        desired_heading = atan2(target[1] - pose.y, target[0] - pose.x)
        heading_error = wrap_angle(desired_heading - pose.theta)
        self.last_distance = distance
        self.last_heading_error = heading_error

        if self.rotating:
            if abs(heading_error) <= self.rotate_exit_heading_error:
                self.rotating = False
        elif abs(heading_error) >= self.rotate_enter_heading_error:
            self.rotating = True

        angular_speed = max(
            -self.maximum_angular_speed,
            min(self.maximum_angular_speed, self.gain_heading * heading_error),
        )
        if self.rotating:
            self.state = "rotate"
            return 0.0, angular_speed

        self.state = "drive"
        alignment = max(0.0, cos(heading_error))
        distance_scale = min(1.0, distance / self.slowdown_distance)
        approach_speed = max(
            self.minimum_drive_speed,
            self.cruise_speed * distance_scale,
        )
        linear_speed = approach_speed * alignment
        return linear_speed, angular_speed


class LocalizationRecoveryMotion:
    """Run bounded sensing motions while MCL has not recovered.

    A kidnapped robot must keep moving long enough to collect a new viewpoint;
    otherwise the particle estimate can change while the physical robot stays
    at the kidnapping location.  Each cycle is deliberately short and checks
    front clearance before translating.  If the belief is still ambiguous at
    the end of a cycle, another bounded cycle is armed instead of silently
    falling back to zero velocity.  A finite cycle limit keeps the behavior
    safe if the map or sensor data cannot support recovery.
    """

    def __init__(
        self,
        turn_speed: float,
        forward_speed: float,
        translation_steps: int,
        translation_bursts: int,
        sensing_steps: int,
        avoidance_distance: float,
        maximum_cycles: int = 12,
    ) -> None:
        self.turn_speed = abs(float(turn_speed))
        self.forward_speed = max(0.0, float(forward_speed))
        self.translation_steps = max(1, int(translation_steps))
        self.translation_bursts = max(1, int(translation_bursts))
        self.sensing_steps = max(1, int(sensing_steps))
        self.avoidance_distance = max(0.0, float(avoidance_distance))
        self.maximum_cycles = max(1, int(maximum_cycles))

        self.active = False
        self.attempted = False
        self.cycle_count = 0
        self.translation_steps_remaining = 0
        self.turn_steps_remaining = 0
        self.bursts_remaining = 0
        self.turn_direction = 1.0

    def _arm_cycle(self) -> bool:
        if self.cycle_count >= self.maximum_cycles:
            return False

        self.cycle_count += 1
        self.bursts_remaining = self.translation_bursts
        self.translation_steps_remaining = self.translation_steps
        self.turn_steps_remaining = 0
        return True

    def start(self) -> None:
        """Start a fresh recovery maneuver after a tracking-to-loss transition."""
        self.attempted = True
        self.cycle_count = 0
        self.turn_direction = 1.0
        self.active = self._arm_cycle()

    def stop(self) -> None:
        """Stop sensing motion once tracking is recovered or the limit is hit."""
        self.active = False
        self.translation_steps_remaining = 0
        self.turn_steps_remaining = 0
        self.bursts_remaining = 0

    def _finish_cycle(self, localized: bool) -> None:
        if localized or not self._arm_cycle():
            self.stop()

    def _turn_command(self) -> tuple[float, bool]:
        self.turn_steps_remaining = max(1, self.turn_steps_remaining)
        angular_speed = self.turn_direction * self.turn_speed
        self.turn_steps_remaining -= 1
        cycle_finished = False
        if self.turn_steps_remaining <= 0:
            self.bursts_remaining -= 1
            self.turn_direction *= -1.0
            if self.bursts_remaining > 0:
                self.translation_steps_remaining = self.translation_steps
            else:
                cycle_finished = True
        return angular_speed, cycle_finished

    def command(
        self,
        scan_points_robot: list[tuple[float, float]],
        localized: bool,
    ) -> tuple[float, float, float, str]:
        """Return a safe sensing command and the minimum front clearance."""
        minimum_front = min(
            (
                hypot(x, y)
                for x, y in scan_points_robot
                if x > 0.0 and abs(y) <= 0.20
            ),
            default=inf,
        )
        if not self.active:
            return 0.0, 0.0, minimum_front, "localization_lost"

        # The filter may become trustworthy partway through a sensing cycle.
        # Stop immediately so the next controller step can hand the estimate
        # back to the normal waypoint follower instead of driving on a pose
        # that has already been declared recovered.
        if localized:
            self.stop()
            return 0.0, 0.0, minimum_front, "localization_tracking"

        linear_speed = 0.0
        angular_speed = 0.0
        controller_state = "localization_active_turn"
        cycle_finished = False

        if self.translation_steps_remaining > 0:
            if minimum_front >= self.avoidance_distance:
                linear_speed = self.forward_speed
                self.translation_steps_remaining -= 1
                controller_state = "localization_active_translate"
                cycle_finished = (
                    self.translation_steps_remaining == 0
                    and self.bursts_remaining == 1
                )
            else:
                # Turn in place to obtain a different view instead of driving
                # into a wall.  This also lets the next cycle choose a new
                # translation direction after a blocked final burst.
                self.translation_steps_remaining = 0
                self.turn_steps_remaining = self.sensing_steps
                angular_speed, cycle_finished = self._turn_command()
        elif self.turn_steps_remaining > 0:
            angular_speed, cycle_finished = self._turn_command()
        elif self.bursts_remaining > 1:
            self.turn_steps_remaining = self.sensing_steps
            angular_speed, cycle_finished = self._turn_command()
        else:
            cycle_finished = True

        if cycle_finished:
            self._finish_cycle(localized)

        return linear_speed, angular_speed, minimum_front, controller_state


class SafetyLayer:
    """Stateful emergency behavior only; students add full avoidance.

    The selected turn direction is latched while emergency mode is active.
    Release requires both a minimum turn duration and several consecutive clear
    scans, preventing noisy LiDAR samples from reversing the turn every step.
    """

    def __init__(
        self,
        emergency_distance: float,
        emergency_release_distance: float,
        avoidance_distance: float,
        emergency_turn_speed: float,
        minimum_turn_steps: int,
        clear_steps_required: int,
        escape_forward_speed: float,
        escape_forward_steps: int,
        maximum_recoveries_per_waypoint: int,
    ) -> None:
        self.emergency_distance = emergency_distance
        self.emergency_release_distance = emergency_release_distance
        self.avoidance_distance = avoidance_distance
        self.emergency_turn_speed = emergency_turn_speed
        self.minimum_turn_steps = max(1, int(minimum_turn_steps))
        self.clear_steps_required = max(1, int(clear_steps_required))
        self.escape_forward_speed = max(0.0, float(escape_forward_speed))
        self.escape_forward_steps = max(0, int(escape_forward_steps))
        self.maximum_recoveries_per_waypoint = max(
            1, int(maximum_recoveries_per_waypoint)
        )
        self.emergency_active = False
        self.turn_direction = 0.0
        self.turn_steps = 0
        self.clear_steps = 0
        self.escape_steps_remaining = 0
        self.recovery_count = 0
        self.blocked = False
        self.state = "clear"

    def reset_for_waypoint(self) -> None:
        """Reset recovery bookkeeping after genuine route progress."""
        self.emergency_active = False
        self.turn_direction = 0.0
        self.turn_steps = 0
        self.clear_steps = 0
        self.escape_steps_remaining = 0
        self.recovery_count = 0
        self.blocked = False
        self.state = "clear"

    @staticmethod
    def _minimum_range(points: list[tuple[float, float]]) -> float:
        return min((hypot(x, y) for x, y in points), default=inf)

    def _choose_turn_direction(self, scan_points_robot: list[tuple[float, float]]) -> float:
        side_points = [
            (x_forward, y_left)
            for x_forward, y_left in scan_points_robot
            if x_forward > -0.05 and abs(atan2(y_left, x_forward)) < 1.20
        ]
        left_clearance = self._minimum_range([(x, y) for x, y in side_points if y > 0.0])
        right_clearance = self._minimum_range([(x, y) for x, y in side_points if y < 0.0])
        if left_clearance > right_clearance + 0.02:
            return 1.0
        if right_clearance > left_clearance + 0.02:
            return -1.0

        # When both sides look similar, turn away from the front-sector
        # centroid.  A centered obstacle uses a deterministic left turn.
        lateral_sum = sum(y for _, y in side_points)
        return -1.0 if lateral_sum > 0.0 else 1.0

    def filter_command(
        self,
        linear_speed: float,
        angular_speed: float,
        scan_points_robot: list[tuple[float, float]],
    ) -> tuple[float, float, float]:
        front_points = [
            (x_forward, y_left)
            for x_forward, y_left in scan_points_robot
            if x_forward > 0.0 and abs(atan2(y_left, x_forward)) < 0.55
        ]
        minimum_front = self._minimum_range(front_points)

        if self.blocked:
            self.state = "blocked"
            return 0.0, 0.0, minimum_front

        if (
            not self.emergency_active
            and self.escape_steps_remaining == 0
            and minimum_front < self.emergency_distance
        ):
            self.recovery_count += 1
            if self.recovery_count > self.maximum_recoveries_per_waypoint:
                self.blocked = True
                self.turn_direction = 0.0
                self.state = "blocked"
                return 0.0, 0.0, minimum_front
            self.emergency_active = True
            self.turn_direction = self._choose_turn_direction(scan_points_robot)
            self.turn_steps = 0
            self.clear_steps = 0

        if self.emergency_active:
            self.turn_steps += 1
            if minimum_front >= self.emergency_release_distance:
                self.clear_steps += 1
            else:
                self.clear_steps = 0

            can_release = (
                self.turn_steps >= self.minimum_turn_steps
                and self.clear_steps >= self.clear_steps_required
            )
            if not can_release:
                self.state = "emergency_turn"
                return 0.0, self.emergency_turn_speed * self.turn_direction, minimum_front

            self.emergency_active = False
            self.turn_steps = 0
            self.clear_steps = 0
            self.escape_steps_remaining = self.escape_forward_steps

        if self.escape_steps_remaining > 0:
            if minimum_front < self.emergency_distance:
                self.recovery_count += 1
                if self.recovery_count > self.maximum_recoveries_per_waypoint:
                    self.blocked = True
                    self.turn_direction = 0.0
                    self.escape_steps_remaining = 0
                    self.state = "blocked"
                    return 0.0, 0.0, minimum_front
                self.emergency_active = True
                self.turn_direction = self._choose_turn_direction(scan_points_robot)
                self.turn_steps = 0
                self.clear_steps = 0
                self.escape_steps_remaining = 0
                self.state = "emergency_turn"
                return 0.0, self.emergency_turn_speed * self.turn_direction, minimum_front

            self.escape_steps_remaining -= 1
            self.state = "escape_forward"
            return self.escape_forward_speed, 0.0, minimum_front

        self.turn_direction = 0.0

        if minimum_front < self.avoidance_distance:
            self.state = "slowdown"
            linear_speed *= 0.30
        else:
            self.state = "clear"

        # STUDENT TODO: replace this bounded safety recovery with the obstacle
        # avoidance method taught in class, and request replanning when the
        # current global path remains blocked.  The starter intentionally
        # enters ``blocked`` instead of pretending that repeated local turns
        # have solved a global path obstruction.
        return linear_speed, angular_speed, minimum_front


def unicycle_to_wheels(
    linear_speed: float,
    angular_speed: float,
    wheel_radius: float,
    axle_length: float,
    max_wheel_speed: float,
) -> tuple[float, float]:
    left = (linear_speed - 0.5 * axle_length * angular_speed) / wheel_radius
    right = (linear_speed + 0.5 * axle_length * angular_speed) / wheel_radius
    scale = max(1.0, abs(left) / max_wheel_speed, abs(right) / max_wheel_speed)
    return left / scale, right / scale
