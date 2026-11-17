"""Instructor-side ground-truth logger and controlled test perturbations."""

from __future__ import annotations

import csv
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path

from controller import Keyboard, Supervisor


PROJECT_ROOT = Path(__file__).resolve().parents[2]
STARTER_DIR = PROJECT_ROOT / "controllers" / "project_starter"
sys.path.insert(0, str(STARTER_DIR))

from coordinate_frames import (  # noqa: E402
    is_non_floor_contact,
    project_pose_from_webots,
    webots_pose_from_project,
)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class EvaluationSupervisor:
    FIELDNAMES = [
        "time_s",
        "gt_x_m",
        "gt_y_m",
        "gt_theta_rad",
        "est_x_m",
        "est_y_m",
        "est_theta_rad",
        "position_error_m",
        "heading_error_rad",
        "mission_goal_index",
        "goals_reached_ground_truth",
        "active_waypoint",
        "minimum_lidar_range_m",
        "lidar_point_count",
        "controller_state",
        "safety_state",
        "escape_turn_direction",
        "distance_to_waypoint_m",
        "heading_error_to_waypoint_rad",
        "commanded_linear_speed_m_s",
        "commanded_angular_speed_rad_s",
        "left_encoder_rad",
        "right_encoder_rad",
        "left_wheel_speed_rad_s",
        "right_wheel_speed_rad_s",
        "contact_now",
        "collision_episodes",
        "ground_truth_path_length_m",
        "mission_complete_reported",
        "mode",
    ]

    def __init__(self) -> None:
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.robot_node = self.supervisor.getFromDef("STUDENT_ROBOT")
        self.floor_node = self.supervisor.getFromDef("FLOOR")
        self.block_node = self.supervisor.getFromDef("DYNAMIC_BLOCK")
        if self.robot_node is None or self.floor_node is None or self.block_node is None:
            raise RuntimeError("Required DEF nodes are missing from the world")

        self.robot_node.enablePoseTracking(self.time_step)
        self.robot_node.enableContactPointsTracking(self.time_step, True)
        self.floor_id = self.floor_node.getId()
        self.receiver = self.supervisor.getDevice("evaluation receiver")
        self.receiver.enable(self.time_step)
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(self.time_step)

        self.config = json.loads((PROJECT_ROOT / "config" / "project_config.json").read_text(encoding="utf-8"))
        self.waypoints = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
        goal_lookup = {goal["name"]: goal["position"] for goal in self.waypoints["delivery_goals"]}
        goal_lookup["HOME"] = self.waypoints["home"]["position"]
        self.required_goals = [goal_lookup[name] for name in self.waypoints["required_goal_order"]]

        self.latest_telemetry: dict = {}
        self.collision_episodes = 0
        self.contact_previous = False
        self.last_collision_time = -float("inf")
        self.previous_position: tuple[float, float] | None = None
        self.path_length = 0.0
        self.goals_reached = 0
        self.step_count = 0
        self.completed_message_printed = False
        self.log_handle = None
        self.log_writer = None
        self.auto_scenario = os.environ.get("CMPE185_AUTO_SCENARIO", "").strip().lower()
        self.auto_duration_s = float(os.environ.get("CMPE185_AUTO_DURATION_S", "0") or 0.0)
        self.auto_kidnap_time_s = float(
            os.environ.get("CMPE185_AUTO_KIDNAP_TIME_S", "20.0") or 20.0
        )
        self.auto_action_done = False
        self.start_new_log()
        self.print_controls()
        self.configure_automated_scenario()

    def print_controls(self) -> None:
        print("Evaluation controls: 1/2/3 move blocker, K kidnaps robot, N starts new log, H prints help")

    def set_robot_pose(self, project_pose: tuple[float, float, float]) -> None:
        translation, rotation = webots_pose_from_project(*project_pose)
        self.robot_node.getField("translation").setSFVec3f(translation)
        self.robot_node.getField("rotation").setSFRotation(rotation)
        self.robot_node.resetPhysics()

    def configure_automated_scenario(self) -> None:
        """Optional development harness; never sends truth to the student controller."""
        if not self.auto_scenario:
            return
        if self.auto_scenario not in {"nominal", "uncertain_start", "kidnap"}:
            raise ValueError(
                "CMPE185_AUTO_SCENARIO must be nominal, uncertain_start, or kidnap"
            )
        if self.auto_duration_s <= 0.0:
            self.auto_duration_s = {
                "nominal": 20.0,
                "uncertain_start": 15.0,
                "kidnap": 70.0,
            }[self.auto_scenario]
        if self.auto_scenario == "uncertain_start":
            # Development-only hidden start. The student controller receives
            # only a global prior and never this pose. Keep the default well
            # inside open free space so the automated test does not become a
            # collision/physics-reset test. Override with
            # CMPE185_AUTO_UNCERTAIN_POSE="x,y,theta" when desired.
            pose_text = os.environ.get(
                "CMPE185_AUTO_UNCERTAIN_POSE", "2.0,1.65,-0.9"
            )
            try:
                pose = tuple(float(value.strip()) for value in pose_text.split(","))
            except ValueError as exc:
                raise ValueError(
                    "CMPE185_AUTO_UNCERTAIN_POSE must be x,y,theta"
                ) from exc
            if len(pose) != 3:
                raise ValueError(
                    "CMPE185_AUTO_UNCERTAIN_POSE must contain exactly x,y,theta"
                )
            self.set_robot_pose(pose)
            self.auto_action_done = True
        print(
            f"[TASK1-AUTO] scenario={self.auto_scenario} "
            f"duration={self.auto_duration_s:.1f}s"
        )

    def process_automated_scenario(self) -> bool:
        """Apply scheduled perturbations and stop a batch run on time."""
        if not self.auto_scenario:
            return False
        now = self.supervisor.getTime()
        if (
            self.auto_scenario == "kidnap"
            and not self.auto_action_done
            and now >= self.auto_kidnap_time_s
        ):
            self.kidnap_robot()
            self.auto_action_done = True
            print(f"[TASK1-AUTO] official K perturbation at t={now:.3f}s")
        if now >= self.auto_duration_s:
            print(f"[TASK1-AUTO] completed scenario at t={now:.3f}s")
            if self.log_handle is not None:
                self.log_handle.flush()
            self.supervisor.simulationQuit(0)
            return True
        return False

    def start_new_log(self) -> None:
        if self.log_handle is not None:
            self.log_handle.close()
        results_dir = PROJECT_ROOT / "results"
        results_dir.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = results_dir / f"run_{stamp}.csv"
        self.log_handle = path.open("w", newline="", encoding="utf-8")
        self.log_writer = csv.DictWriter(self.log_handle, fieldnames=self.FIELDNAMES)
        self.log_writer.writeheader()
        self.log_handle.flush()
        self.collision_episodes = 0
        self.contact_previous = False
        self.last_collision_time = -float("inf")
        self.previous_position = None
        self.path_length = 0.0
        self.goals_reached = 0
        self.completed_message_printed = False
        print(f"Evaluation log: {path}")

    def read_telemetry(self) -> None:
        while self.receiver.getQueueLength() > 0:
            try:
                # R2025a returns text from getString().  The fallback keeps the
                # project usable with older Webots Python controller bindings.
                if hasattr(self.receiver, "getString"):
                    message = self.receiver.getString()
                else:
                    packet = self.receiver.getData()
                    message = packet.decode("utf-8") if isinstance(packet, (bytes, bytearray)) else str(packet)
                self.latest_telemetry = json.loads(message)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as error:
                print(f"Ignored malformed telemetry: {error}")
            self.receiver.nextPacket()

    def ground_truth_pose(self) -> tuple[float, float, float]:
        return project_pose_from_webots(
            self.robot_node.getPosition(),
            self.robot_node.getOrientation(),
        )

    def detect_non_floor_contact(self) -> bool:
        floor_tolerance = self.config["evaluation"]["floor_contact_height_tolerance_m"]
        for contact in self.robot_node.getContactPoints(True):
            node_id = contact.node_id if hasattr(contact, "node_id") else contact.getNodeId()
            coordinates = contact.point if hasattr(contact, "point") else contact.getPoint()
            # Some Webots/PROTO combinations associate a descendant node with
            # a wheel-floor contact. The contact height provides a second,
            # geometry-based way to reject the horizontal floor plane.
            if is_non_floor_contact(node_id, self.floor_id, coordinates[2], floor_tolerance):
                return True
        return False

    def update_metrics(self, gt_pose: tuple[float, float, float], contact_now: bool) -> None:
        x, y, _ = gt_pose
        if self.previous_position is not None:
            self.path_length += math.hypot(x - self.previous_position[0], y - self.previous_position[1])
        self.previous_position = (x, y)

        now = self.supervisor.getTime()
        debounce = self.config["evaluation"]["collision_debounce_s"]
        if contact_now and not self.contact_previous and now - self.last_collision_time >= debounce:
            self.collision_episodes += 1
            self.last_collision_time = now
        self.contact_previous = contact_now

        if self.goals_reached < len(self.required_goals):
            goal = self.required_goals[self.goals_reached]
            if math.hypot(x - goal[0], y - goal[1]) <= self.config["evaluation"]["position_goal_tolerance_m"]:
                self.goals_reached += 1
                print(f"Ground-truth goal progress: {self.goals_reached}/{len(self.required_goals)}")

    def write_log_row(self, gt_pose: tuple[float, float, float], contact_now: bool) -> None:
        estimate = self.latest_telemetry.get("estimate")
        if isinstance(estimate, list) and len(estimate) == 3:
            est_x, est_y, est_theta = map(float, estimate)
            position_error = math.hypot(est_x - gt_pose[0], est_y - gt_pose[1])
            heading_error = abs(wrap_angle(est_theta - gt_pose[2]))
        else:
            est_x = est_y = est_theta = position_error = heading_error = ""

        row = {
            "time_s": f"{self.supervisor.getTime():.3f}",
            "gt_x_m": f"{gt_pose[0]:.6f}",
            "gt_y_m": f"{gt_pose[1]:.6f}",
            "gt_theta_rad": f"{gt_pose[2]:.6f}",
            "est_x_m": "" if est_x == "" else f"{est_x:.6f}",
            "est_y_m": "" if est_y == "" else f"{est_y:.6f}",
            "est_theta_rad": "" if est_theta == "" else f"{est_theta:.6f}",
            "position_error_m": "" if position_error == "" else f"{position_error:.6f}",
            "heading_error_rad": "" if heading_error == "" else f"{heading_error:.6f}",
            "mission_goal_index": self.latest_telemetry.get("mission_goal_index", ""),
            "goals_reached_ground_truth": self.goals_reached,
            "active_waypoint": self.latest_telemetry.get("active_waypoint", ""),
            "minimum_lidar_range_m": self.latest_telemetry.get("minimum_lidar_range", ""),
            "lidar_point_count": self.latest_telemetry.get("lidar_point_count", ""),
            "controller_state": self.latest_telemetry.get("controller_state", ""),
            "safety_state": self.latest_telemetry.get("safety_state", ""),
            "escape_turn_direction": self.latest_telemetry.get("escape_turn_direction", ""),
            "distance_to_waypoint_m": self.latest_telemetry.get("distance_to_waypoint", ""),
            "heading_error_to_waypoint_rad": self.latest_telemetry.get("heading_error_to_waypoint", ""),
            "commanded_linear_speed_m_s": self.latest_telemetry.get("commanded_linear_speed", ""),
            "commanded_angular_speed_rad_s": self.latest_telemetry.get("commanded_angular_speed", ""),
            "left_encoder_rad": self.latest_telemetry.get("left_encoder_rad", ""),
            "right_encoder_rad": self.latest_telemetry.get("right_encoder_rad", ""),
            "left_wheel_speed_rad_s": self.latest_telemetry.get("left_wheel_speed", ""),
            "right_wheel_speed_rad_s": self.latest_telemetry.get("right_wheel_speed", ""),
            "contact_now": int(contact_now),
            "collision_episodes": self.collision_episodes,
            "ground_truth_path_length_m": f"{self.path_length:.6f}",
            "mission_complete_reported": int(bool(self.latest_telemetry.get("mission_complete", False))),
            "mode": self.latest_telemetry.get("mode", ""),
        }
        self.log_writer.writerow(row)
        if self.step_count % 20 == 0:
            self.log_handle.flush()

    def move_blocker(self, index: int) -> None:
        positions = [
            [0.0, 0.0, 0.18],
            [-2.0, 1.3, 0.18],
            [2.0, -1.3, 0.18],
        ]
        self.block_node.getField("translation").setSFVec3f(positions[index])
        print(f"Moved unexpected obstacle to test position {index + 1}")

    def kidnap_robot(self) -> None:
        project_pose = (0.9, -2.1, math.pi)
        self.set_robot_pose(project_pose)
        print("Kidnapped-robot perturbation applied")

    def process_keys(self) -> None:
        key = self.keyboard.getKey()
        while key != -1:
            character = chr(key & 0xFF).upper() if (key & 0xFF) < 128 else ""
            if character in {"1", "2", "3"}:
                self.move_blocker(int(character) - 1)
            elif character == "K":
                self.kidnap_robot()
            elif character == "N":
                self.start_new_log()
            elif character == "H":
                self.print_controls()
            key = self.keyboard.getKey()

    def run(self) -> None:
        try:
            while self.supervisor.step(self.time_step) != -1:
                self.step_count += 1
                if self.process_automated_scenario():
                    break
                self.process_keys()
                self.read_telemetry()
                gt_pose = self.ground_truth_pose()
                contact_now = self.detect_non_floor_contact()
                self.update_metrics(gt_pose, contact_now)
                if self.step_count % self.config["evaluation"]["log_every_steps"] == 0:
                    self.write_log_row(gt_pose, contact_now)
                if self.goals_reached == len(self.required_goals) and not self.completed_message_printed:
                    print(
                        "Mission completed by ground truth: "
                        f"path={self.path_length:.2f} m, collisions={self.collision_episodes}"
                    )
                    self.completed_message_printed = True
        finally:
            if self.log_handle is not None:
                self.log_handle.flush()
                self.log_handle.close()


if __name__ == "__main__":
    EvaluationSupervisor().run()
