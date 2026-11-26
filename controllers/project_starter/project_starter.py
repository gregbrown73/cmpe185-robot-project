"""Runnable CMPE 185 controller with MCL, A* planning, and diagnostics.

The controller keeps the original baseline waypoint route in
``maps/waypoints.json`` for comparison, while the default
``full_student_method`` mode uses MCL, A*, the Task 6 route manager, and Task 7
obstacle handling. Odometry remains the motion input to MCL; it is not used as
the planner's pose when MCL is selected.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from controller import Robot

from coordinate_frames import lidar_point_to_robot_xy
from experiment_logging import StudentRunLogger
from localization import StudentLocalizer, select_localization_pose
from mcl_debug import MCLDebugLogger
from navigation import (
    LocalizationRecoveryMotion,
    SafetyLayer,
    WaypointFollower,
    unicycle_to_wheels,
)
from occupancy_grid import OccupancyGrid
from odometry import DifferentialDriveOdometry
from planning import (
    SUCCESS,
    plan_world_path,
    simplify_world_path,
)
from route_manager import RouteManager
from temporary_obstacles import TemporaryObstacleLayer
from visualization import DisplayVisualizer, MCLDebugVisualizer


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PLANNING_MODES = {
    "provided_waypoints",
    "astar",
}
EXPERIMENT_METHODS = {
    "odometry_baseline",
    "mcl_only",
    "full_student_method",
}


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


def plan_route_from_pose(
    planning_grid,
    pose,
    goal_xy: tuple[float, float],
    minimum_spacing_m: float,
) -> tuple[list[tuple[float, float]], str]:
    """Plan one metric route from the supplied localization pose to a goal."""
    route, status = plan_world_path(
        planning_grid,
        (float(pose.x), float(pose.y)),
        goal_xy,
    )
    if status == SUCCESS:
        route = simplify_world_path(route, minimum_spacing=minimum_spacing_m)
    return route, status


def select_experiment_modes(config: dict) -> tuple[str, str, str, str]:
    """Resolve the fixed Task 8 experiment, localization, planning, and avoidance modes."""
    # TASK 8: Keep one explicit mode table so baseline and student runs cannot
    # accidentally mix localization, planning, and avoidance implementations.
    experiment_method = config.get("experiment", {}).get("method")
    if experiment_method is None:
        # Accept the pre-T8 configuration names while users transition configs.
        legacy_planning = config.get("controller", {}).get(
            "planning_mode", "provided_waypoints"
        )
        legacy_localization = config.get("localization", {}).get("mode", "mcl")
        if legacy_planning == "provided_waypoints":
            experiment_method = (
                "odometry_baseline"
                if legacy_localization in {"odometry", "odometry_baseline"}
                else "mcl_only"
            )
        else:
            experiment_method = "full_student_method"
    if experiment_method not in EXPERIMENT_METHODS:
        raise ValueError(
            "experiment.method must be one of "
            f"{sorted(EXPERIMENT_METHODS)}"
        )
    selection = {
        "odometry_baseline": ("odometry", "provided_waypoints", "starter_safety"),
        "mcl_only": ("mcl", "provided_waypoints", "starter_safety"),
        "full_student_method": ("mcl", "astar", "student_avoidance"),
    }
    localization_mode, planning_mode, avoidance_mode = selection[experiment_method]
    mapping_mode = config.get("mapping", {}).get("mode", "off")
    if mapping_mode not in {"off", "occupancy_mapping"}:
        raise ValueError("mapping.mode must be 'off' or 'occupancy_mapping'")
    return experiment_method, localization_mode, planning_mode, avoidance_mode


def main() -> None:
    config = load_json(PROJECT_ROOT / "config" / "project_config.json")
    waypoints_data = load_json(PROJECT_ROOT / "maps" / "waypoints.json")
    grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")
    (
        experiment_method,
        localization_mode,
        planning_mode,
        avoidance_mode,
    ) = select_experiment_modes(config)

    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_encoder = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    lidar = robot.getDevice("lidar")
    display = robot.getDevice("student display")
    mcl_display = robot.getDevice("mcl display")
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
    if planning_mode not in PLANNING_MODES:
        raise ValueError(
            "controller.planning_mode must be one of "
            f"{sorted(PLANNING_MODES)}"
        )
    astar_enabled = planning_mode == "astar"
    inflation_radius_m = float(grid.inflation_radius_m)
    # TASK 8: Keep the static map raw; temporary obstacles are combined first,
    # then the complete planning map is inflated exactly once before A*.
    planning_grid = grid
    route_spacing_m = float(controller_config.get("planning_waypoint_spacing_m", 0.15))

    initial_pose = tuple(config["coordinate_frame"]["initial_pose"])
    odometry = DifferentialDriveOdometry(
        robot_config["wheel_radius_m"],
        robot_config["effective_track_width_m"],
        initial_pose,
    )
    localizer = StudentLocalizer(
        initial_pose=initial_pose,
        particle_count=config["localization"]["particle_count_recommendation"],
        initial_position_std_m=config["localization"]["initial_position_std_m"],
        initial_heading_std_rad=config["localization"]["initial_heading_std_rad"],
        inflation_radius_m=inflation_radius_m,
        motion_distance_noise_fraction=config["localization"]["motion_distance_noise_fraction"],
        motion_heading_noise_fraction=config["localization"]["motion_heading_noise_fraction"],
        ess_resample_fraction=config["localization"]["ess_resample_fraction"],
        lost_measurement_threshold=config["localization"]["lost_measurement_threshold"],
        recovery_measurement_threshold=config["localization"]["recovery_measurement_threshold"],
        lost_measurement_updates=config["localization"]["lost_measurement_updates"],
        recovery_reinjection_fraction=config["localization"]["recovery_reinjection_fraction"],
        maximum_lidar_beams=config["localization"]["maximum_lidar_beams"],
        measurement_sigma_m=config["localization"]["measurement_sigma_m"],
        measurement_outlier_weight=config["localization"]["measurement_outlier_weight"],
        lidar_min_range_m=config["localization"]["lidar_min_range_m"],
        lidar_max_range_m=config["localization"]["lidar_max_range_m"],
    )
    initialization_mode = os.environ.get(
        "CMPE185_MCL_INITIALIZATION_MODE",
        config["localization"].get("initialization_mode", "nominal"),
    )
    localizer.initialize(grid, initialization_mode=initialization_mode)
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
        student_avoidance=avoidance_mode == "student_avoidance",
    )
    visualizer = DisplayVisualizer(display, grid)
    mcl_visualizer = MCLDebugVisualizer(mcl_display, grid)
    debug_logger = MCLDebugLogger(
        PROJECT_ROOT,
        enabled=bool(config["localization"].get("debug_log_enabled", True)),
    )

    # Task 3 logging is additive; the production evaluation Supervisor remains
    # unchanged and the original MCL debug log remains available.
    logger = StudentRunLogger(
        experiment_method=experiment_method,
        scenario_name=config.get("experiment", {}).get("scenario_name", "default"),
        trial_number=int(config.get("experiment", {}).get("trial_number", 1)),
        random_seed=config.get("experiment", {}).get("random_seed"),
        configuration=config,
    )

    # Keep the supplied route permanently available for baseline comparison.
    baseline_route = [tuple(point) for point in waypoints_data["baseline_waypoints"]]
    goals = list(waypoints_data["delivery_goals"])
    goals.append({"name": "HOME", "position": waypoints_data["home"]["position"]})
    # TASK 8: Task 6 owns generated mission-route state; Task 7 owns dynamic
    # planning evidence. Neither subsystem edits the static map in place.
    route_manager = RouteManager(
        goals,
        route_point_tolerance=float(controller_config["waypoint_tolerance_m"]),
        goal_tolerance=float(config["evaluation"]["position_goal_tolerance_m"]),
    )
    temporary_obstacles = TemporaryObstacleLayer(
        confirmation_count=int(controller_config.get("temporary_obstacle_confirmation_count", 3)),
        ttl_s=float(controller_config.get("temporary_obstacle_ttl_s", 5.0)),
    )
    route = [] if astar_enabled else baseline_route[:]
    active_waypoint = 0
    mission_goal_index = 0
    mission_complete = False
    step_count = 0
    route_version = 0
    replan_count = 0
    last_replan_reason = ""
    last_plan_status = "PENDING" if astar_enabled else "BASELINE"
    next_plan_step = 0
    previous_localization_state = "LOCALIZING"
    localization_loss_pending_recovery = False
    previous_safety_state = safety.state
    mission_complete_logged = False
    estimated_path_length_m = 0.0
    previous_estimate = None
    recovery_motion = LocalizationRecoveryMotion(
        turn_speed=controller_config.get(
            "localization_recovery_turn_speed_rad_s", 0.35
        ),
        forward_speed=controller_config.get(
            "localization_recovery_forward_speed_m_s", 0.045
        ),
        translation_steps=controller_config.get(
            "localization_recovery_translation_steps", 120
        ),
        translation_bursts=controller_config.get(
            "localization_recovery_translation_bursts", 2
        ),
        sensing_steps=controller_config.get("recovery_sensing_steps", 90),
        avoidance_distance=controller_config.get("avoidance_distance_m", 0.23),
        maximum_cycles=controller_config.get(
            "localization_recovery_max_cycles", 12
        ),
    )

    print("CMPE 185 autonomous indoor delivery starter, release 4.0.0")
    if astar_enabled:
        print(
            "Planner: A* from the MCL pose estimate; "
            "baseline_waypoints retained for comparison"
        )
    else:
        print("Planner: provided baseline waypoints")
    print(
        "Drive parameters: "
        f"wheel radius={robot_config['wheel_radius_m']:.4f} m, "
        f"effective track={robot_config['effective_track_width_m']:.4f} m"
    )
    print("Frames: Webots ENU world (x-y floor, z up); robot/LiDAR FLU (+x forward, +y left)")
    print("Ground truth is recorded only by the evaluation Supervisor.")
    if debug_logger.path is not None:
        print(f"MCL debug log: {debug_logger.path}")

    while robot.step(time_step) != -1:
        step_count += 1
        left_encoder_value = left_encoder.getValue()
        right_encoder_value = right_encoder.getValue()
        odometry_pose, delta_motion = odometry.update(
            left_encoder_value,
            right_encoder_value,
        )
        scan_points = lidar_points_in_robot_frame(lidar)
        localizer.update(odometry_pose, delta_motion, scan_points, grid)
        mcl_pose = localizer.get_estimate()
        localization_state = (
            localizer.get_state() if localization_mode == "mcl" else "TRACKING"
        )
        mcl_diagnostics = localizer.get_debug_diagnostics()
        recovery_search_active = bool(mcl_diagnostics.get("recovery_active", False))
        recovery_ambiguous = bool(mcl_diagnostics.get("recovery_ambiguous", False))
        estimate = select_localization_pose(
            odometry_pose,
            localizer,
            localization_mode,
        )
        if previous_estimate is not None:
            estimated_path_length_m += math.hypot(
                estimate.x - previous_estimate[0],
                estimate.y - previous_estimate[1],
            )
        previous_estimate = (estimate.x, estimate.y)

        # Start bounded exploratory motion in two cases:
        #   1. a TRACKING belief was lost after kidnapping; or
        #   2. global/uncertain-start localization has several live hypotheses.
        # Odometry supplies motion to MCL, while the recovery motion changes the
        # physical viewpoint so LiDAR can distinguish those hypotheses.
        start_lost_recovery = (
            localization_state == "LOST"
            and previous_localization_state != "LOST"
        )
        start_global_recovery = (
            initialization_mode == "global_free_space"
            and localization_state == "LOCALIZING"
            and recovery_search_active
            and not recovery_ambiguous
            and not recovery_motion.attempted
        )
        if (start_lost_recovery or start_global_recovery) and not recovery_motion.active:
            recovery_motion.start()

        if recovery_motion.active and (
            localization_state == "TRACKING" or not recovery_search_active
        ):
            recovery_motion.stop()

        # Task 3 state transitions use the MCL API rather than guessed public
        # attributes, which keeps logging correct after the T1/T2-3 merge.
        if localization_state != previous_localization_state:
            if localization_state == "LOST":
                logger.log_event(robot.getTime(), "LOCALIZATION_LOST")
                localization_loss_pending_recovery = True
            elif localization_state == "TRACKING" and localization_loss_pending_recovery:
                logger.log_event(robot.getTime(), "LOCALIZATION_RECOVERED")
                if astar_enabled:
                    route_manager.request_replan("LOCALIZATION_RECOVERED")
                localization_loss_pending_recovery = False
            previous_localization_state = localization_state

        localization_ready = (
            localization_state == "TRACKING" and not recovery_motion.active
        )

        if localization_ready:
            recovery_motion.stop()

            if astar_enabled:
                # TASK 8: Task 7 observes dynamic obstacles only for the full
                # student method; baseline and MCL-only modes never add them.
                now_s = robot.getTime()
                if avoidance_mode == "student_avoidance":
                    temporary_obstacles.observe(
                        scan_points,
                        estimate,
                        grid,
                        now_s,
                        localization_state=localization_state,
                    )

                # TASK 8: RouteManager owns route progress and mission-goal
                # transitions; A* only supplies one route per active goal.
                route_manager.advance_route(estimate)
                goal_index_before = route_manager.active_goal_index
                if route_manager.advance_goal(estimate):
                    reached_goal = goals[goal_index_before]
                    logger.log_event(
                        now_s,
                        "GOAL_REACHED",
                        details=str(reached_goal.get("name", goal_index_before)),
                    )

                if route_manager.state == "COMPLETE":
                    mission_complete = True
                elif route_manager.state == "NO_PATH" and step_count >= next_plan_step:
                    route_manager.request_replan("PATH_INVALID")

                if not mission_complete and route_manager.needs_plan():
                    current_goal = route_manager.current_goal()
                    if current_goal is not None:
                        reason = route_manager.replan_reason or "initial_plan"
                        logger.log_event(
                            now_s,
                            "REPLAN_REQUEST",
                            details=reason,
                        )
                        if avoidance_mode == "student_avoidance":
                            planning_grid = temporary_obstacles.build_planning_grid(
                                grid,
                                inflation_radius_m=inflation_radius_m,
                                time_s=now_s,
                                start_xy=(estimate.x, estimate.y),
                            )
                        else:
                            planning_grid = grid.inflated(inflation_radius_m)
                        planned_route, last_plan_status = plan_route_from_pose(
                            planning_grid,
                            estimate,
                            current_goal,
                            route_spacing_m,
                        )
                        replan_count += 1
                        last_replan_reason = reason
                        if last_plan_status == SUCCESS:
                            route_manager.install_route(planned_route)
                            safety.reset_for_waypoint()
                            next_plan_step = step_count
                        else:
                            route_manager.mark_no_path()
                            next_plan_step = step_count + int(
                                controller_config.get("planner_retry_steps", 30)
                            )
                            logger.log_event(
                                now_s,
                                "NO_PATH",
                                details=f"{last_plan_status}:{goals[route_manager.active_goal_index]['name']}",
                            )

                route = list(route_manager.route)
                active_waypoint = route_manager.route_index
                mission_goal_index = route_manager.active_goal_index
            else:
                # The original route is deliberately preserved as a selectable
                # baseline for the final comparison experiments.
                previous_active_waypoint = active_waypoint
                while (
                    active_waypoint < len(route)
                    and follower.distance_to(
                        estimate, route[active_waypoint]
                    )
                    <= controller_config["waypoint_tolerance_m"]
                ):
                    active_waypoint += 1
                if active_waypoint != previous_active_waypoint:
                    safety.reset_for_waypoint()

                if mission_goal_index < len(goals):
                    current_goal = tuple(goals[mission_goal_index]["position"])
                    if (
                        follower.distance_to(estimate, current_goal)
                        <= config["evaluation"]["position_goal_tolerance_m"]
                    ):
                        reached_goal = goals[mission_goal_index]
                        mission_goal_index += 1
                        logger.log_event(
                            robot.getTime(),
                            "GOAL_REACHED",
                            details=str(reached_goal.get("name", mission_goal_index)),
                        )

        if not localization_ready:
            # Never follow a route on a stale or ambiguous pose.  During global
            # start or kidnapped recovery, the bounded motion gathers new views.
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min(
                (
                    math.hypot(x, y)
                    for x, y in scan_points
                    if x > 0.0 and abs(y) <= 0.20
                ),
                default=float("inf"),
            )
            if recovery_motion.active:
                (
                    linear_speed,
                    angular_speed,
                    minimum_front,
                    controller_state,
                ) = recovery_motion.command(
                    scan_points,
                    localized=localization_state == "TRACKING",
                )
            else:
                controller_state = f"localization_{localization_state.lower()}"
                if not recovery_motion.attempted:
                    turn_speed = float(
                        controller_config.get(
                            "localization_recovery_turn_speed_rad_s", 0.35
                        )
                    )
                    turn_phase = (step_count // 90) % 2
                    angular_speed = turn_speed if turn_phase == 0 else -turn_speed
        elif mission_complete or (
            not astar_enabled and active_waypoint >= len(route)
        ):
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min(
                (math.hypot(x, y) for x, y in scan_points),
                default=float("inf"),
            )
            mission_complete = True
            controller_state = "complete"
            safety.state = "inactive"
            safety.turn_direction = 0.0
            follower.last_distance = 0.0
            follower.last_heading_error = 0.0
        elif astar_enabled and not route:
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min(
                (math.hypot(x, y) for x, y in scan_points),
                default=float("inf"),
            )
            controller_state = f"planner_{last_plan_status.lower()}"
        elif active_waypoint >= len(route):
            linear_speed = 0.0
            angular_speed = 0.0
            minimum_front = min(
                (math.hypot(x, y) for x, y in scan_points),
                default=float("inf"),
            )
            controller_state = "route_waiting"
        else:
            linear_speed, angular_speed = follower.command(
                estimate,
                route[active_waypoint],
            )
            if avoidance_mode == "student_avoidance":
                linear_speed, angular_speed, minimum_front = safety.filter_command(
                    linear_speed,
                    angular_speed,
                    scan_points,
                    pose=estimate,
                    target=route[active_waypoint],
                    time_s=robot.getTime(),
                )
            else:
                linear_speed, angular_speed, minimum_front = safety.filter_command(
                    linear_speed,
                    angular_speed,
                    scan_points,
                )
            controller_state = follower.state

        # TASK 8: A BLOCKED signal is accepted only when Task 7 has a
        # confirmed temporary obstacle and localization is TRACKING.
        if avoidance_mode == "student_avoidance" and astar_enabled:
            confirmed_obstacle = temporary_obstacles.active_count(robot.getTime()) > 0
            if safety.consider_blocked_replan(
                localization_state,
                confirmed_obstacle,
                time_s=robot.getTime(),
            ):
                route_manager.request_replan("BLOCKED")
                safety.clear_replan_request()
        elif safety.state == "blocked" and previous_safety_state != "blocked":
            logger.log_event(
                robot.getTime(),
                "REPLAN_REQUEST",
                details="blocked",
            )
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
            # TASK 8: RouteManager removes an already-reached start point from
            # generated routes; prepend the current pose so the display still
            # shows the first visible leg into the next hallway/waypoint.
            display_route = route
            display_active_waypoint = active_waypoint
            if astar_enabled and route:
                display_route = [
                    (float(estimate.x), float(estimate.y)),
                    *route[active_waypoint:],
                ]
                display_active_waypoint = 0
            visualizer.draw(
                estimate,
                scan_points,
                display_route,
                goals,
                min(display_active_waypoint, max(0, len(display_route) - 1)),
                localization_mode,
                controller_state,
                safety.state,
            )
            mcl_visualizer.draw(
                localizer.get_estimate(),
                odometry_pose,
                localizer.particles,
                localizer.get_state(),
                localizer.get_confidence(),
                mcl_diagnostics,
                localizer.get_recovery_hypotheses(),
            )

        if step_count % controller_config["telemetry_period_steps"] == 0:
            telemetry_goal_index = (
                route_manager.active_goal_index if astar_enabled else mission_goal_index
            )
            telemetry_route_version = (
                route_manager.route_version if astar_enabled else route_version
            )
            telemetry_replan_reason = (
                route_manager.replan_reason
                if astar_enabled and route_manager.replan_reason != "NONE"
                else last_replan_reason
            )
            temporary_count = (
                temporary_obstacles.active_count(robot.getTime())
                if avoidance_mode == "student_avoidance"
                else 0
            )
            diagnostics = {
                "active_goal": (
                    goals[telemetry_goal_index]["name"]
                    if telemetry_goal_index < len(goals)
                    else "HOME"
                ),
                "route_index": active_waypoint,
                "route_count": len(route),
                "route_version": telemetry_route_version,
                "replan_count": replan_count,
                "replan_reason": telemetry_replan_reason,
                "avoidance_state": safety.state,
                "temporary_obstacle_count": temporary_count,
                "mcl_state": localization_state,
                "mcl_confidence": localizer.get_confidence(),
                "mcl_ess": mcl_diagnostics.get("ess", ""),
                "particle_position_spread_m": mcl_diagnostics.get(
                    "position_spread_m", ""
                ),
                "particle_heading_spread_rad": mcl_diagnostics.get(
                    "heading_spread_rad", ""
                ),
                "estimated_x_m": estimate.x,
                "estimated_y_m": estimate.y,
                "estimated_heading_rad": estimate.theta,
                "estimated_path_length_m": estimated_path_length_m,
                "mission_complete": int(mission_complete),
                "goals_reached": mission_goal_index,
                "home_error_m": follower.distance_to(
                    estimate,
                    tuple(waypoints_data["home"]["position"]),
                ),
                "collision_episodes": "",
            }
            logger.write(robot.getTime(), diagnostics)
            telemetry = {
                "time": robot.getTime(),
                "estimate": list(estimate.as_tuple()),
                "active_waypoint": active_waypoint,
                "mission_goal_index": telemetry_goal_index,
                "minimum_lidar_range": (
                    minimum_front if math.isfinite(minimum_front) else None
                ),
                "mission_complete": mission_complete,
                "mode": experiment_method,
                "planning_mode": planning_mode,
                "planner_status": last_plan_status,
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

        hypotheses = localizer.get_recovery_hypotheses()
        debug_logger.write(
            robot.getTime(),
            mcl_pose,
            localizer.get_state(),
            localizer.get_confidence(),
            mcl_diagnostics,
            hypotheses,
        )
        if step_count % 20 == 0:
            debug_logger.flush()

        if mission_complete and not mission_complete_logged:
            logger.log_event(robot.getTime(), "MISSION_COMPLETE")
            mission_complete_logged = True

    logger.close()


if __name__ == "__main__":
    main()
