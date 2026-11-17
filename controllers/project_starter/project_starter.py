"""Runnable CMPE 185 starter controller for the autonomous-delivery project.

The baseline reads encoders and LiDAR, integrates odometry, follows a supplied
waypoint route, draws a diagnostic display, and sends estimates to the grading
Supervisor. It does not access Webots ground truth.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from controller import Robot

from coordinate_frames import lidar_point_to_robot_xy
from experiment_logging import StudentRunLogger
from localization import StudentLocalizer
from navigation import SafetyLayer, WaypointFollower, unicycle_to_wheels
from occupancy_grid import OccupancyGrid
from odometry import DifferentialDriveOdometry
from visualization import DisplayVisualizer


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def lidar_points_in_robot_frame(lidar) -> list[tuple[float, float]]:
    """Return LiDAR point-cloud samples as FLU ``(forward, left)`` pairs."""
    converted: list[tuple[float, float]] = []
    for point in lidar.getPointCloud():
        planar_point = lidar_point_to_robot_xy(point.x, point.y, point.z)
        if planar_point is not None:
            converted.append(planar_point)
    return converted


def main() -> None:
    config = load_json(PROJECT_ROOT / "config" / "project_config.json")
    waypoints_data = load_json(PROJECT_ROOT / "maps" / "waypoints.json")
    grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_encoder = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    lidar = robot.getDevice("lidar")
    display = robot.getDevice("student display")
    emitter = robot.getDevice("emitter")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    motor_acceleration = float(config["robot"].get("motor_acceleration_rad_s2", -1.0))
    if motor_acceleration > 0.0:
        left_motor.setAcceleration(motor_acceleration)
        right_motor.setAcceleration(motor_acceleration)
    left_encoder.enable(time_step)
    right_encoder.enable(time_step)
    lidar.enable(time_step)
    lidar.enablePointCloud()

    robot_config = config["robot"]
    controller_config = config["controller"]
    initial_pose = tuple(config["coordinate_frame"]["initial_pose"])
    odometry = DifferentialDriveOdometry(
        robot_config["wheel_radius_m"],
        robot_config["effective_track_width_m"],
        initial_pose,
    )
    localizer = StudentLocalizer(
        initial_pose,
        config["localization"]["particle_count_recommendation"],
    )
    localizer.initialize(grid)
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
    visualizer = DisplayVisualizer(display, grid)

    # Task 3: create a separate controller-side log; leave the evaluation Supervisor unchanged.
    logger = StudentRunLogger(
        experiment_method=controller_config.get("planning_mode", "student"),
        scenario_name=config.get("experiment", {}).get("scenario_name", "default"),
        trial_number=int(config.get("experiment", {}).get("trial_number", 1)),
        random_seed=config.get("experiment", {}).get("random_seed"),
        configuration=config,
    )

    route = [tuple(point) for point in waypoints_data["baseline_waypoints"]]
    goals = list(waypoints_data["delivery_goals"])
    goals.append({"name": "HOME", "position": waypoints_data["home"]["position"]})
    active_waypoint = 0
    mission_goal_index = 0
    mission_complete = False
    step_count = 0
    # Track route and state changes for experiment diagnostics.
    route_version = 0
    replan_count = 0
    previous_localization_state = getattr(localizer, "state", "TRACKING")
    previous_safety_state = safety.state
    mission_complete_logged = False
    # Accumulate the controller's estimated path for student-side analysis.
    estimated_path_length_m = 0.0
    previous_estimate = None

    print("CMPE 185 autonomous indoor delivery starter, release 4.0.0")
    print("Baseline: calibrated encoder odometry + speed-shaped waypoint following")
    print(
        "Drive parameters: "
        f"wheel radius={robot_config['wheel_radius_m']:.4f} m, "
        f"effective track={robot_config['effective_track_width_m']:.4f} m"
    )
    print("Frames: Webots ENU world (x-y floor, z up); robot/LiDAR FLU (+x forward, +y left)")
    print("Ground truth is recorded only by the evaluation Supervisor.")

    while robot.step(time_step) != -1:
        step_count += 1
        left_encoder_value = left_encoder.getValue()
        right_encoder_value = right_encoder.getValue()
        odometry_pose, delta_motion = odometry.update(
            left_encoder_value,
            right_encoder_value,
        )
        scan_points = lidar_points_in_robot_frame(lidar)
        estimate = localizer.update(odometry_pose, delta_motion, scan_points, grid)
        if previous_estimate is not None:
            estimated_path_length_m += math.hypot(
                estimate.x - previous_estimate[0],
                estimate.y - previous_estimate[1],
            )
        previous_estimate = (estimate.x, estimate.y)

        previous_active_waypoint = active_waypoint
        while active_waypoint < len(route) and follower.distance_to(estimate, route[active_waypoint]) <= controller_config["waypoint_tolerance_m"]:
            active_waypoint += 1
        if active_waypoint != previous_active_waypoint:
            safety.reset_for_waypoint()

        # Record each ordered delivery goal as soon as it is reached.
        if mission_goal_index < len(goals):
            current_goal = tuple(goals[mission_goal_index]["position"])
            if follower.distance_to(estimate, current_goal) <= config["evaluation"]["position_goal_tolerance_m"]:
                mission_goal_index += 1
                logger.log_event(
                    robot.getTime(),
                    "GOAL_REACHED",
                    details=str(goals[mission_goal_index - 1].get("name", mission_goal_index)),
                )

        if active_waypoint >= len(route):
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min((math.hypot(x, y) for x, y in scan_points), default=float("inf"))
            mission_complete = True
            controller_state = "complete"
            safety.state = "inactive"
            safety.turn_direction = 0.0
            follower.last_distance = 0.0
            follower.last_heading_error = 0.0
        else:
            linear_speed, angular_speed = follower.command(estimate, route[active_waypoint])
            linear_speed, angular_speed, minimum_front = safety.filter_command(
                linear_speed,
                angular_speed,
                scan_points,
            )
            controller_state = follower.state

        # Convert localization and safety state changes into named events.
        localization_state = getattr(localizer, "state", "TRACKING")
        if localization_state != previous_localization_state:
            if localization_state == "LOST":
                logger.log_event(robot.getTime(), "LOCALIZATION_LOST")
            elif previous_localization_state == "LOST":
                logger.log_event(robot.getTime(), "LOCALIZATION_RECOVERED")
            previous_localization_state = localization_state

        if safety.state == "blocked" and previous_safety_state != "blocked":
            replan_count += 1
            logger.log_event(robot.getTime(), "REPLAN_REQUEST", details="blocked")
        previous_safety_state = safety.state

        left_speed, right_speed = unicycle_to_wheels(
            linear_speed,
            angular_speed,
            robot_config["wheel_radius_m"],
            robot_config["effective_track_width_m"],
            robot_config["max_wheel_speed_rad_s"],
        )
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        if step_count % controller_config["display_update_steps"] == 0:
            visualizer.draw(
                estimate,
                scan_points,
                route,
                goals,
                min(active_waypoint, max(0, len(route) - 1)),
                config["localization"]["mode"],
                controller_state,
                safety.state,
            )

        # Write the required diagnostic fields at the configured telemetry rate.
        if step_count % controller_config["telemetry_period_steps"] == 0:
            # Task 3 diagnostics: describe the current route, avoidance, localization, and mission status.
            diagnostics = {
                "active_goal": goals[mission_goal_index]["name"] if mission_goal_index < len(goals) else "HOME",
                "route_index": active_waypoint,
                "route_count": len(route),
                "route_version": route_version,
                "replan_count": replan_count,
                "replan_reason": "",
                "avoidance_state": safety.state,
                "temporary_obstacle_count": 0,
                "mcl_state": localization_state,
                "mcl_confidence": getattr(localizer, "confidence", ""),
                "mcl_ess": getattr(localizer, "ess", ""),
                "particle_position_spread_m": getattr(localizer, "position_spread_m", ""),
                "particle_heading_spread_rad": getattr(localizer, "heading_spread_rad", ""),
                "estimated_x_m": estimate.x,
                "estimated_y_m": estimate.y,
                "estimated_heading_rad": estimate.theta,
                "estimated_path_length_m": estimated_path_length_m,
                "mission_complete": int(mission_complete),
                "goals_reached": mission_goal_index,
                "home_error_m": follower.distance_to(
                    estimate, tuple(waypoints_data["home"]["position"])
                ),
                "collision_episodes": "",
            }
            logger.write(robot.getTime(), diagnostics)
            telemetry = {
                "time": robot.getTime(),
                "estimate": list(estimate.as_tuple()),
                "active_waypoint": active_waypoint,
                "mission_goal_index": mission_goal_index,
                "minimum_lidar_range": minimum_front if math.isfinite(minimum_front) else None,
                "mission_complete": mission_complete,
                "mode": config["localization"]["mode"],
                "commanded_linear_speed": linear_speed,
                "commanded_angular_speed": angular_speed,
                "left_wheel_speed": left_speed,
                "right_wheel_speed": right_speed,
                "left_encoder_rad": left_encoder_value,
                "right_encoder_rad": right_encoder_value,
                "lidar_point_count": len(scan_points),
                "controller_state": controller_state,
                "safety_state": safety.state,
                "escape_turn_direction": safety.turn_direction,
                "distance_to_waypoint": follower.last_distance,
                "heading_error_to_waypoint": follower.last_heading_error,
            }
            emitter.send(json.dumps(telemetry).encode("utf-8"))

        # Log mission completion once instead of repeating the event every step.
        if mission_complete and not mission_complete_logged:
            logger.log_event(robot.getTime(), "MISSION_COMPLETE")
            mission_complete_logged = True

    logger.close()


if __name__ == "__main__":
    main()
