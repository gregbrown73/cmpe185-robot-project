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
