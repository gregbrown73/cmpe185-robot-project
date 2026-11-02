#!/usr/bin/env python3
"""Deterministic controller-in-the-loop check for the nominal starter route.

This is not a replacement for a Webots run.  It exercises the same odometry,
waypoint follower, wheel conversion, motor acceleration limit, safety state
machine, wall geometry, movable blocker, and ordered goals without requiring a
Webots installation.  The release tests use it to catch inconsistent geometry,
kinematics, waypoints, or controller parameters before the world is opened.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "controllers" / "project_starter"
sys.path.insert(0, str(CONTROLLER_DIR))

from navigation import SafetyLayer, WaypointFollower, unicycle_to_wheels  # noqa: E402
from odometry import DifferentialDriveOdometry, Pose2D, wrap_angle  # noqa: E402


@dataclass(frozen=True)
class Rectangle:
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


STATIC_RECTANGLES = [
    Rectangle("west wall", -4.05, -3.95, -3.00, 3.00),
    Rectangle("east wall", 3.95, 4.05, -3.00, 3.00),
    Rectangle("south wall", -4.00, 4.00, -3.05, -2.95),
    Rectangle("north wall", -4.00, 4.00, 2.95, 3.05),
    Rectangle("corridor lower west", -4.00, -2.60, -0.65, -0.55),
    Rectangle("corridor lower center", -1.40, 1.40, -0.65, -0.55),
    Rectangle("corridor lower east", 2.60, 4.00, -0.65, -0.55),
    Rectangle("corridor upper west", -4.00, -2.60, 0.55, 0.65),
    Rectangle("corridor upper center", -1.40, 1.40, 0.55, 0.65),
    Rectangle("corridor upper east", 2.60, 4.00, 0.55, 0.65),
    Rectangle("south divider", -0.05, 0.05, -3.00, -0.60),
    Rectangle("north divider", -0.05, 0.05, 0.60, 3.00),
]


def blocker_rectangle(position: tuple[float, float] = (0.0, 0.0)) -> Rectangle:
    return Rectangle(
        "default movable blocker",
        position[0] - 0.225,
        position[0] + 0.225,
        position[1] - 0.225,
        position[1] + 0.225,
    )


def ray_rectangle_distance(
    origin_x: float,
    origin_y: float,
    direction_x: float,
    direction_y: float,
    rectangle: Rectangle,
) -> float | None:
    """Return the first nonnegative ray distance to an axis-aligned rectangle."""
    low = -math.inf
    high = math.inf
    for origin, direction, minimum, maximum in (
        (origin_x, direction_x, rectangle.xmin, rectangle.xmax),
        (origin_y, direction_y, rectangle.ymin, rectangle.ymax),
    ):
        if abs(direction) < 1e-12:
            if origin < minimum or origin > maximum:
                return None
            continue
        first = (minimum - origin) / direction
        second = (maximum - origin) / direction
        if first > second:
            first, second = second, first
        low = max(low, first)
        high = min(high, second)
        if low > high:
            return None
    if high < 0.0:
        return None
    return max(0.0, low)


def lidar_scan(
    pose: Pose2D,
    rectangles: list[Rectangle],
    ray_count: int = 90,
    maximum_range: float = 4.0,
    minimum_range: float = 0.05,
) -> list[tuple[float, float]]:
    """Create ideal planar FLU returns matching the world LiDAR convention."""
    # The official e-puck turret slot is 9.5 mm behind the robot origin.
    sensor_x = pose.x - 0.0095 * math.cos(pose.theta)
    sensor_y = pose.y - 0.0095 * math.sin(pose.theta)
    points: list[tuple[float, float]] = []
    for index in range(ray_count):
        relative_angle = -math.pi + index * (2.0 * math.pi / ray_count)
        world_angle = pose.theta + relative_angle
        direction_x = math.cos(world_angle)
        direction_y = math.sin(world_angle)
        distances = [
            distance
            for rectangle in rectangles
            if (
                distance := ray_rectangle_distance(
                    sensor_x,
                    sensor_y,
                    direction_x,
                    direction_y,
                    rectangle,
                )
            )
            is not None
            and minimum_range <= distance <= maximum_range
        ]
        if distances:
            distance = min(distances)
            points.append(
                (distance * math.cos(relative_angle), distance * math.sin(relative_angle))
            )
    return points


def point_rectangle_distance(x: float, y: float, rectangle: Rectangle) -> float:
    dx = max(rectangle.xmin - x, 0.0, x - rectangle.xmax)
    dy = max(rectangle.ymin - y, 0.0, y - rectangle.ymax)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return -min(
        x - rectangle.xmin,
        rectangle.xmax - x,
        y - rectangle.ymin,
        rectangle.ymax - y,
    )


def move_toward(current: float, target: float, maximum_change: float) -> float:
    return current + max(-maximum_change, min(maximum_change, target - current))


def run_simulation(
    output_path: Path | None = None,
    maximum_time_s: float = 720.0,
    plant_wheel_radius_m: float | None = None,
    plant_track_width_m: float | None = None,
    lidar_ray_count: int = 90,
    trace_stride: int = 1,
) -> dict:
    if trace_stride < 1:
        raise ValueError("trace_stride must be at least 1")
    config = json.loads((PROJECT_ROOT / "config" / "project_config.json").read_text())
    waypoint_data = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text())
    robot_config = config["robot"]
    controller_config = config["controller"]
    evaluation_config = config["evaluation"]

    route = [tuple(point) for point in waypoint_data["baseline_waypoints"]]
    goal_lookup = {
        goal["name"]: tuple(goal["position"])
        for goal in waypoint_data["delivery_goals"]
    }
    goal_lookup["HOME"] = tuple(waypoint_data["home"]["position"])
    goals = [goal_lookup[name] for name in waypoint_data["required_goal_order"]]

    wheel_radius = float(robot_config["wheel_radius_m"])
    effective_track = float(robot_config["effective_track_width_m"])
    plant_wheel_radius = (
        wheel_radius if plant_wheel_radius_m is None else float(plant_wheel_radius_m)
    )
    plant_track_width = (
        effective_track if plant_track_width_m is None else float(plant_track_width_m)
    )

    follower = WaypointFollower(
        robot_config["cruise_speed_m_s"],
        controller_config["waypoint_slowdown_distance_m"],
        controller_config["minimum_drive_speed_m_s"],
        controller_config["rotate_enter_heading_error_rad"],
        controller_config["rotate_exit_heading_error_rad"],
        controller_config["heading_gain"],
        controller_config["maximum_angular_speed_rad_s"],
    )
    safety = SafetyLayer(
        controller_config["emergency_stop_distance_m"],
        controller_config["emergency_release_distance_m"],
        controller_config["avoidance_distance_m"],
        controller_config["emergency_turn_speed_rad_s"],
        controller_config["emergency_minimum_turn_steps"],
        controller_config["emergency_clear_steps"],
        controller_config["escape_forward_speed_m_s"],
        controller_config["escape_forward_steps"],
        controller_config["maximum_recoveries_per_waypoint"],
    )

    initial_pose = tuple(config["coordinate_frame"]["initial_pose"])
    actual = Pose2D(*initial_pose)
    odometry = DifferentialDriveOdometry(wheel_radius, effective_track, initial_pose)
    left_encoder = 0.0
    right_encoder = 0.0
    odometry.update(left_encoder, right_encoder)
    current_left_speed = 0.0
    current_right_speed = 0.0
    active_waypoint = 0
    goals_reached = 0
    collision = False
    minimum_clearance = math.inf
    path_length = 0.0
    dt = 0.032
    time_s = 0.0
    simulation_step = 0
    rows: list[dict] = []
    rectangles = STATIC_RECTANGLES + [blocker_rectangle()]

    while time_s <= maximum_time_s:
        estimate = odometry.pose
        previous_active = active_waypoint
        while (
            active_waypoint < len(route)
            and follower.distance_to(estimate, route[active_waypoint])
            <= controller_config["waypoint_tolerance_m"]
        ):
            active_waypoint += 1
        if active_waypoint != previous_active:
            safety.reset_for_waypoint()

        if goals_reached < len(goals):
            if math.dist(actual.as_tuple()[:2], goals[goals_reached]) <= evaluation_config[
                "position_goal_tolerance_m"
            ]:
                goals_reached += 1

        scan = lidar_scan(actual, rectangles, ray_count=lidar_ray_count)
        if active_waypoint >= len(route):
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min((math.hypot(x, y) for x, y in scan), default=math.inf)
            follower.state = "complete"
            safety.state = "inactive"
        else:
            linear_speed, angular_speed = follower.command(
                estimate, route[active_waypoint]
            )
            linear_speed, angular_speed, minimum_front = safety.filter_command(
                linear_speed, angular_speed, scan
            )

        target_left_speed, target_right_speed = unicycle_to_wheels(
            linear_speed,
            angular_speed,
            wheel_radius,
            effective_track,
            robot_config["max_wheel_speed_rad_s"],
        )
        maximum_speed_change = robot_config["motor_acceleration_rad_s2"] * dt
        current_left_speed = move_toward(
            current_left_speed, target_left_speed, maximum_speed_change
        )
        current_right_speed = move_toward(
            current_right_speed, target_right_speed, maximum_speed_change
        )

        delta_left_angle = current_left_speed * dt
        delta_right_angle = current_right_speed * dt
        left_encoder += delta_left_angle
        right_encoder += delta_right_angle

        delta_left = plant_wheel_radius * delta_left_angle
        delta_right = plant_wheel_radius * delta_right_angle
        delta_distance = 0.5 * (delta_left + delta_right)
        delta_heading = (delta_right - delta_left) / plant_track_width
        midpoint_heading = actual.theta + 0.5 * delta_heading
        old_x, old_y = actual.x, actual.y
        actual.x += delta_distance * math.cos(midpoint_heading)
        actual.y += delta_distance * math.sin(midpoint_heading)
        actual.theta = wrap_angle(actual.theta + delta_heading)
        path_length += math.hypot(actual.x - old_x, actual.y - old_y)
        estimate, _ = odometry.update(left_encoder, right_encoder)

        clearance = min(
            point_rectangle_distance(actual.x, actual.y, rectangle)
            - robot_config["body_radius_m"]
            for rectangle in rectangles
        )
        minimum_clearance = min(minimum_clearance, clearance)
        collision = collision or clearance < -1e-6

        position_error = math.hypot(estimate.x - actual.x, estimate.y - actual.y)
        heading_error = abs(wrap_angle(estimate.theta - actual.theta))
        row = {
                "time_s": f"{time_s:.3f}",
                "gt_x_m": f"{actual.x:.6f}",
                "gt_y_m": f"{actual.y:.6f}",
                "gt_theta_rad": f"{actual.theta:.6f}",
                "est_x_m": f"{estimate.x:.6f}",
                "est_y_m": f"{estimate.y:.6f}",
                "est_theta_rad": f"{estimate.theta:.6f}",
                "position_error_m": f"{position_error:.6f}",
                "heading_error_rad": f"{heading_error:.6f}",
                "goals_reached_ground_truth": goals_reached,
                "active_waypoint": active_waypoint,
                "minimum_lidar_range_m": (
                    "" if not math.isfinite(minimum_front) else f"{minimum_front:.6f}"
                ),
                "controller_state": follower.state,
                "safety_state": safety.state,
                "commanded_linear_speed_m_s": f"{linear_speed:.6f}",
                "commanded_angular_speed_rad_s": f"{angular_speed:.6f}",
                "left_encoder_rad": f"{left_encoder:.6f}",
                "right_encoder_rad": f"{right_encoder:.6f}",
                "left_wheel_speed_rad_s": f"{current_left_speed:.6f}",
                "right_wheel_speed_rad_s": f"{current_right_speed:.6f}",
                "contact_now": int(clearance < -1e-6),
                "collision_episodes": int(collision),
                "ground_truth_path_length_m": f"{path_length:.6f}",
                "mission_complete_reported": int(active_waypoint >= len(route)),
                "mode": "deterministic_reference_simulation",
            }
        if simulation_step % trace_stride == 0 or active_waypoint >= len(route):
            rows.append(row)

        if active_waypoint >= len(route):
            break
        if safety.blocked or collision:
            break
        time_s += dt
        simulation_step += 1

    home = tuple(waypoint_data["home"]["position"])
    summary = {
        "passed": (
            active_waypoint == len(route)
            and goals_reached == len(goals)
            and not collision
            and math.dist(actual.as_tuple()[:2], home)
            <= evaluation_config["position_goal_tolerance_m"]
        ),
        "duration_s": time_s,
        "route_waypoints_completed": active_waypoint,
        "route_waypoints_total": len(route),
        "ordered_goals_reached": goals_reached,
        "ordered_goals_total": len(goals),
        "final_home_error_m": math.dist(actual.as_tuple()[:2], home),
        "ground_truth_path_length_m": path_length,
        "minimum_body_clearance_m": minimum_clearance,
        "collision": collision,
        "blocked": safety.blocked,
        "plant_wheel_radius_m": plant_wheel_radius,
        "plant_track_width_m": plant_track_width,
    }

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary_path = output_path.with_name(f"{output_path.stem}_summary.json")
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--maximum-time-s", type=float, default=720.0)
    parser.add_argument("--plant-wheel-radius-m", type=float)
    parser.add_argument("--plant-track-width-m", type=float)
    parser.add_argument(
        "--trace-stride",
        type=int,
        default=1,
        help="Write one CSV row per N simulation steps (the final row is always written).",
    )
    args = parser.parse_args()
    summary = run_simulation(
        output_path=args.output,
        maximum_time_s=args.maximum_time_s,
        plant_wheel_radius_m=args.plant_wheel_radius_m,
        plant_track_width_m=args.plant_track_width_m,
        trace_stride=args.trace_stride,
    )
    print(json.dumps(summary, indent=2))
    raise SystemExit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
