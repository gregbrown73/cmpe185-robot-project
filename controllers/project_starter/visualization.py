"""Webots Display visualization for map, path, scan, and pose."""

from __future__ import annotations

from math import cos, sin


class DisplayVisualizer:
    def __init__(self, display, occupancy_grid) -> None:
        self.display = display
        self.grid = occupancy_grid
        self.width = display.getWidth()
        self.height = display.getHeight()
        self.scale = min(self.width / self.grid.width, self.height / self.grid.height)
        self.map_width = int(self.grid.width * self.scale)
        self.map_height = int(self.grid.height * self.scale)
        self.offset_x = (self.width - self.map_width) // 2
        self.offset_y = (self.height - self.map_height) // 2

    def world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        row, col = self.grid.world_to_cell(x, y)
        px = int(self.offset_x + (col + 0.5) * self.scale)
        py = int(self.offset_y + (row + 0.5) * self.scale)
        return px, py

    def draw(
        self,
        pose,
        scan_points_robot: list[tuple[float, float]],
        path: list[tuple[float, float]],
        goals: list[dict],
        active_waypoint: int,
        mode: str,
        controller_state: str,
        safety_state: str,
    ) -> None:
        self.display.setColor(0xF8FAFC)
        self.display.fillRectangle(0, 0, self.width, self.height)
        cell_size = max(1, int(self.scale + 0.5))
        for row, values in enumerate(self.grid.cells):
            for col, value in enumerate(values):
                if value >= self.grid.occupied_value:
                    self.display.setColor(0x334155)
                    self.display.fillRectangle(
                        int(self.offset_x + col * self.scale),
                        int(self.offset_y + row * self.scale),
                        cell_size,
                        cell_size,
                    )

        if len(path) > 1:
            self.display.setColor(0x2563EB)
            for first, second in zip(path, path[1:]):
                x1, y1 = self.world_to_pixel(*first)
                x2, y2 = self.world_to_pixel(*second)
                self.display.drawLine(x1, y1, x2, y2)

        goal_colors = {"A": 0x1D4ED8, "B": 0x16A34A, "C": 0xEA580C, "HOME": 0x64748B}
        for goal in goals:
            px, py = self.world_to_pixel(*goal["position"])
            self.display.setColor(goal_colors.get(goal["name"], 0x7C3AED))
            self.display.fillOval(px - 5, py - 5, 10, 10)

        # Transform a decimated LiDAR scan from robot to map coordinates.
        self.display.setColor(0x06B6D4)
        c = cos(pose.theta)
        s = sin(pose.theta)
        for x_forward, y_left in scan_points_robot[::3]:
            world_x = pose.x + c * x_forward - s * y_left
            world_y = pose.y + s * x_forward + c * y_left
            px, py = self.world_to_pixel(world_x, world_y)
            if 0 <= px < self.width and 0 <= py < self.height:
                self.display.fillOval(px - 1, py - 1, 2, 2)

        px, py = self.world_to_pixel(pose.x, pose.y)
        self.display.setColor(0xDC2626)
        self.display.fillOval(px - 5, py - 5, 10, 10)
        head_x = px + int(14 * cos(pose.theta))
        head_y = py - int(14 * sin(pose.theta))
        self.display.drawLine(px, py, head_x, head_y)

        self.display.setColor(0x0F172A)
        self.display.setFont("Arial", 14, True)
        self.display.drawText(f"mode: {mode}", 8, 8)
        self.display.drawText(f"waypoint: {active_waypoint + 1}/{len(path)}", 8, 26)
        self.display.drawText(f"control: {controller_state}", 8, 44)
        self.display.drawText(f"safety: {safety_state}", 8, 62)


class MCLDebugVisualizer(DisplayVisualizer):
    """Separate Webots display for localization diagnostics.

    The display accepts optional particle/state data and is intentionally
    separate from the student route display.  It has no effect on robot
    control.
    """

    def __init__(self, display, occupancy_grid) -> None:
        super().__init__(display, occupancy_grid)
        self.header_height = 130
        self.scale = min(
            self.width / self.grid.width,
            (self.height - self.header_height) / self.grid.height,
        )
        self.map_width = int(self.grid.width * self.scale)
        self.map_height = int(self.grid.height * self.scale)
        self.offset_x = (self.width - self.map_width) // 2
        self.offset_y = self.header_height + (self.height - self.header_height - self.map_height) // 2

    def _draw_map(self) -> None:
        cell_size = max(1, int(self.scale + 0.5))
        for row, values in enumerate(self.grid.cells):
            for col, value in enumerate(values):
                if value >= self.grid.occupied_value:
                    self.display.setColor(0x334155)
                    self.display.fillRectangle(
                        int(self.offset_x + col * self.scale),
                        int(self.offset_y + row * self.scale),
                        cell_size,
                        cell_size,
                    )

    def draw(
        self,
        pose,
        odometry_pose=None,
        particles=None,
        state: str = "MCL_PENDING",
        confidence: float | None = None,
        diagnostics: dict | None = None,
        hypotheses=None,
    ) -> None:
        """Draw localization state without exposing it to navigation."""
        particles = particles or []
        diagnostics = diagnostics or {}
        hypotheses = hypotheses or []
        # Show the full particle cloud.  The filter uses 300 particles by
        # default, which is small enough for this diagnostic display and makes
        # multimodal recovery easier to interpret visually.
        display_particles = particles

        self.display.setColor(0xF8FAFC)
        self.display.fillRectangle(0, 0, self.width, self.height)
        self._draw_map()

        if odometry_pose is not None:
            odom_x, odom_y = self.world_to_pixel(odometry_pose.x, odometry_pose.y)
            self.display.setColor(0x7C3AED)
            self.display.fillOval(odom_x - 5, odom_y - 5, 10, 10)

        pose_x, pose_y = self.world_to_pixel(pose.x, pose.y)
        # A single red MCL pose is meaningful only after the filter has reached
        # TRACKING.  During LOST/LOCALIZING recovery, the amber hypotheses and
        # particle clusters are the belief; drawing one arbitrary best mode in
        # red made multimodal recovery look falsely certain.
        if state == "TRACKING":
            self.display.setColor(0xDC2626)
            self.display.fillOval(pose_x - 5, pose_y - 5, 10, 10)

        # Draw particles over the markers so a concentrated cloud remains
        # visible. Higher weight particles are brighter.
        maximum_weight = max(
            (particle.weight for particle in display_particles),
            default=0.0,
        )
        for particle in display_particles:
            relative_weight = (
                particle.weight / maximum_weight if maximum_weight > 0.0 else 0.0
            )
            red = int(80 + 175 * relative_weight)
            green = int(35 + 100 * relative_weight)
            self.display.setColor((red << 16) | (green << 8))
            px, py = self.world_to_pixel(particle.x, particle.y)
            if 0 <= px < self.width and self.header_height <= py < self.height:
                self.display.fillOval(px - 1, py - 1, 3, 3)

        # Draw scan-match alternatives after the particles so they remain
        # visible even when a particle cloud is concentrated on one mode.
        # Very weak alternatives remain in the localizer/debug log but are
        # omitted from this compact view once their posterior probability is
        # below 3 percent.
        visible_hypotheses = [
            item for item in hypotheses if item[0] >= 0.03
        ]
        for rank, (score, hypothesis) in enumerate(visible_hypotheses):
            hypothesis_x, hypothesis_y = self.world_to_pixel(
                hypothesis.x,
                hypothesis.y,
            )
            color = 0xF59E0B if rank == 0 else 0xFB923C
            size = 8 if rank == 0 else (6 if score >= 0.10 else 4)
            half_size = size // 2
            self.display.setColor(color)
            self.display.fillOval(
                hypothesis_x - half_size,
                hypothesis_y - half_size,
                size,
                size,
            )
            self.display.drawLine(
                hypothesis_x - half_size - 2,
                hypothesis_y,
                hypothesis_x + half_size + 2,
                hypothesis_y,
            )
            self.display.drawLine(
                hypothesis_x,
                hypothesis_y - half_size - 2,
                hypothesis_x,
                hypothesis_y + half_size + 2,
            )

        # Crosshairs keep both estimates identifiable even when particles
        # overlap them.
        if odometry_pose is not None:
            self.display.setColor(0x7C3AED)
            self.display.drawLine(odom_x - 6, odom_y, odom_x + 6, odom_y)
            self.display.drawLine(odom_x, odom_y - 6, odom_x, odom_y + 6)
        if state == "TRACKING":
            self.display.setColor(0xDC2626)
            self.display.drawLine(pose_x - 6, pose_y, pose_x + 6, pose_y)
            self.display.drawLine(pose_x, pose_y - 6, pose_x, pose_y + 6)
            head_x = pose_x + int(14 * cos(pose.theta))
            head_y = pose_y - int(14 * sin(pose.theta))
            self.display.drawLine(pose_x, pose_y, head_x, head_y)

        self.display.setColor(0x0F172A)
        self.display.setFont("Arial", 14, True)
        confidence_text = "pending" if confidence is None else f"{confidence:.2f}"
        ess_text = diagnostics.get("ess", "pending")
        spread_text = diagnostics.get("position_spread_m", "pending")
        score_text = diagnostics.get("measurement_score", "pending")
        self.display.drawText(
            "MCL diagnostics (red=TRACKING pose, purple=odom, amber=alts)",
            8,
            8,
        )
        self.display.drawText(f"state: {state}", 8, 26)
        self.display.drawText(f"confidence: {confidence_text}", 8, 44)
        particle_text = f"particles: {len(particles)}"
        self.display.drawText(
            f"{particle_text}  ESS: {ess_text}",
            8,
            62,
        )
        self.display.drawText(
            f"spread: {spread_text}  score: {score_text}",
            8,
            80,
        )
        recovery_active = "yes" if diagnostics.get("recovery_active", False) else "no"
        recovery_count = diagnostics.get("recovery_count", 0)
        self.display.drawText(
            f"recovery active: {recovery_active}  count: {recovery_count}",
            8,
            98,
        )
        translation = diagnostics.get("recovery_translation_m", 0.0)
        observations = diagnostics.get("recovery_observation_count", 0)
        ambiguous = "yes" if diagnostics.get("recovery_ambiguous", False) else "no"
        top_probability = hypotheses[0][0] if hypotheses else 0.0
        second_probability = hypotheses[1][0] if len(hypotheses) > 1 else 0.0
        strong_modes = sum(probability >= 0.10 for probability, _ in hypotheses)
        self.display.drawText(
            f"alts: {len(hypotheses)} strong: {strong_modes}  "
            f"p1: {top_probability:.2f} p2: {second_probability:.2f}  "
            f"amb: {ambiguous} move: {translation:.2f}m/{observations}",
            8,
            116,
        )
