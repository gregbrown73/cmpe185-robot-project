#!/usr/bin/env python3
"""Static consistency checks that do not require a Webots installation."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "controllers" / "project_starter"
sys.path.insert(0, str(CONTROLLER_DIR))

from occupancy_grid import OccupancyGrid  # noqa: E402
from generate_map import build_grid  # noqa: E402
from simulate_baseline import (  # noqa: E402
    STATIC_RECTANGLES,
    blocker_rectangle,
    point_rectangle_distance,
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    release = (PROJECT_ROOT / "VERSION").read_text(encoding="utf-8").strip()
    require(release == "4.0.0", f"Expected release 4.0.0, found {release!r}")
    world_path = PROJECT_ROOT / "worlds" / "cmpe185_indoor_delivery.wbt"
    world = world_path.read_text(encoding="utf-8")
    for required_text in [
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"',
        "DEF STUDENT_ROBOT E-puck",
        'name "lidar"',
        'name "student display"',
        'controller "project_starter"',
        'controller "evaluation_supervisor"',
        'version "2"',
        "DEF DYNAMIC_BLOCK Solid",
        "DEF FLOOR Solid",
        'coordinateSystem "ENU"',
        "orientation -0.57735 0.57735 0.57735 2.0944",
        "position 0 0 10.5",
        "fieldOfView 0.75",
        "translation 0 0 -0.025",
        "size 8.2 6.2 0.05",
        "translation -3.2 -2.2 0",
        "rotation 0 0 1 0",
        "translation 0 0 0.055",
        "translation 0 0 0.18",
        "horizontalResolution 360",
        "fieldOfView 6.283185",
        "minRange 0.05",
        "maxRange 4.0",
    ]:
        require(required_text in world, f"World is missing: {required_text}")

    for forbidden_text in [
        "size 8.2 0.05 6.2",
        "translation -3.2 0 2.2",
        "rotation 0 1 0 -1.5708",
        "translation 0 0.055 0",
        "project y = -Webots z",
    ]:
        require(forbidden_text not in world, f"World retains obsolete Y-up geometry: {forbidden_text}")

    supervisor_source = (
        PROJECT_ROOT / "controllers" / "evaluation_supervisor" / "evaluation_supervisor.py"
    ).read_text(encoding="utf-8")
    require("self.receiver.getString()" in supervisor_source, "Supervisor must use Receiver.getString")
    require("getData().decode" not in supervisor_source, "Deprecated Receiver.getData().decode remains")
    require("project_pose_from_webots" in supervisor_source, "Supervisor must use the shared pose conversion")
    require("webots_pose_from_project" in supervisor_source, "Kidnapping must use the shared pose conversion")

    starter_source = (CONTROLLER_DIR / "project_starter.py").read_text(encoding="utf-8")
    require("lidar_point_to_robot_xy" in starter_source, "Starter must use the shared LiDAR conversion")
    require("-point.z" not in starter_source, "Obsolete LiDAR -z forward conversion remains")
    require("-point.x" not in starter_source, "Obsolete LiDAR x sign flip remains")
    require("effective_track_width_m" in starter_source, "Starter must use the calibrated track width")
    require("left_encoder_rad" in starter_source, "Starter telemetry must expose encoder angles")
    for forbidden_ground_truth_api in [
        "from controller import Supervisor",
        "Supervisor()",
        ".getSelf(",
        ".getFromDef(",
        ".getPosition(",
        ".getOrientation(",
        ".getPose(",
    ]:
        require(
            forbidden_ground_truth_api not in starter_source,
            f"Student controller uses a forbidden ground-truth API: {forbidden_ground_truth_api}",
        )
    require('getDevice("gps")' not in starter_source, "Starter controller must not use GPS")

    frames_source = (CONTROLLER_DIR / "coordinate_frames.py").read_text(encoding="utf-8")
    require("local +x is forward" in frames_source, "FLU convention is not documented in code")

    navigation_source = (CONTROLLER_DIR / "navigation.py").read_text(encoding="utf-8")
    require("self.rotating" in navigation_source, "Waypoint follower must keep rotate/drive state")
    require("self.emergency_active" in navigation_source, "Safety layer must latch emergency state")
    require("emergency_release_distance" in navigation_source, "Safety layer must use release hysteresis")
    require("escape_forward" in navigation_source, "Safety layer must include a forward escape state")
    require("self.blocked" in navigation_source, "Safety layer must bound repeated recoveries")

    grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")
    require(grid.width == 80 and grid.height == 60, "Expected an 80 x 60 occupancy grid")
    require(any(value >= 100 for row in grid.cells for value in row), "Map has no occupied cells")
    require(any(value == 0 for row in grid.cells for value in row), "Map has no free cells")
    require(grid.cells == build_grid(), "Occupancy CSV does not match the declared world wall geometry")

    waypoints = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
    named_points = [(waypoints["home"]["name"], waypoints["home"]["position"])]
    named_points += [(goal["name"], goal["position"]) for goal in waypoints["delivery_goals"]]
    named_points += [(f"baseline {index}", point) for index, point in enumerate(waypoints["baseline_waypoints"])]
    for name, point in named_points:
        require(not grid.world_occupied(*point), f"{name} lies in an occupied map cell: {point}")

    body_radius = 0.037
    nominal_rectangles = STATIC_RECTANGLES + [blocker_rectangle()]
    route = waypoints["baseline_waypoints"]
    require(route[8] == [-0.7, 0.43] and route[9] == [0.7, 0.43], "Upper center detour is obsolete")
    require(route[18] == [0.7, -0.43] and route[19] == [-0.7, -0.43], "Lower center detour is obsolete")
    require(len(route) == 23, "Nominal route must contain 23 waypoints")
    minimum_route_clearance = math.inf
    for start, end in zip(route, route[1:]):
        samples = max(1, math.ceil(math.dist(start, end) / 0.01))
        for index in range(samples + 1):
            fraction = index / samples
            x = start[0] + fraction * (end[0] - start[0])
            y = start[1] + fraction * (end[1] - start[1])
            clearance = min(
                point_rectangle_distance(x, y, rectangle) - body_radius
                for rectangle in nominal_rectangles
            )
            minimum_route_clearance = min(minimum_route_clearance, clearance)
    require(
        minimum_route_clearance >= 0.075,
        f"Nominal route body clearance is only {minimum_route_clearance:.3f} m",
    )

    config = json.loads((PROJECT_ROOT / "config" / "project_config.json").read_text(encoding="utf-8"))
    require(config["evaluation"]["receiver_channel"] == 7, "World and config expect receiver channel 7")
    frame_description = config["coordinate_frame"]["description"]
    require("x-y floor plane" in frame_description and "FLU" in frame_description, "Config must declare ENU/FLU")
    require("-Webots z" not in frame_description, "Config retains obsolete coordinate conversion")
    controller = config["controller"]
    robot = config["robot"]
    require(abs(robot["wheel_radius_m"] - 0.0200) < 1e-9, "R2025a e-puck wheel radius is 0.0200 m")
    require(
        abs(robot["geometric_axle_length_m"] - 0.0520) < 1e-9,
        "R2025a e-puck wheel-center spacing is 0.0520 m",
    )
    require(
        0.056 <= robot["effective_track_width_m"] <= 0.059,
        "Effective track width must retain the validated dynamic-model calibration",
    )
    require(
        robot["effective_track_width_m"] > robot["geometric_axle_length_m"],
        "Effective track width should account for tire scrubbing in dynamic turns",
    )
    require(
        abs(robot["body_radius_m"] - body_radius) < 1e-9,
        "Configured body radius must match the R2025a e-puck bounding cylinder",
    )
    require(robot["max_wheel_speed_rad_s"] <= 7.536, "Command limit exceeds e-puck2 motor limit")
    require(
        controller["maximum_angular_speed_rad_s"] <= 0.80,
        "Baseline angular speed must remain conservative enough for repeatable turns",
    )
    require(
        controller["emergency_release_distance_m"] > controller["emergency_stop_distance_m"],
        "Emergency release distance must exceed the entry distance",
    )
    require(
        controller["waypoint_tolerance_m"] < minimum_route_clearance,
        "Waypoint tolerance is too large for the narrowest nominal route clearance",
    )

    documentation = "\n".join(
        path.read_text(encoding="utf-8")
        for path in [PROJECT_ROOT / "README.md", PROJECT_ROOT / "docs" / "assignment_spec.md"]
    )
    require("project y = -Webots z" not in documentation, "Student documentation retains obsolete axes")

    require(world.count("{") == world.count("}"), "World braces are unbalanced")

    reference_summary_path = PROJECT_ROOT / "reference" / "expected_baseline_trace_summary.json"
    reference_trace_path = PROJECT_ROOT / "reference" / "expected_baseline_trace.csv"
    reference_route_plot = PROJECT_ROOT / "reference" / "expected_baseline_route.png"
    require(reference_summary_path.is_file(), "Reference summary is missing")
    require(reference_trace_path.is_file(), "Reference trace is missing")
    require(reference_route_plot.is_file(), "Reference route plot is missing")
    reference = json.loads(reference_summary_path.read_text(encoding="utf-8"))
    require(reference["mission_complete_reported"], "Reference simulation does not complete")
    require(reference["maximum_active_waypoint"] == 23, "Reference route is incomplete")
    require(reference["goals_reached"] == 4, "Reference goals are incomplete")
    require(reference["collision_episodes"] == 0, "Reference route collides")
    require(reference["final_home_error_m"] <= 0.20, "Reference does not return HOME")
    print("Static project validation passed.")


if __name__ == "__main__":
    main()
