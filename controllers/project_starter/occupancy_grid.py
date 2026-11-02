"""Occupancy-grid loading, coordinate conversion, and collision inflation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from math import ceil, hypot
from pathlib import Path


def _parse_scalar(value: str):
    value = value.strip().strip('"').strip("'")
    if value.startswith("[") and value.endswith("]"):
        return [float(item.strip()) for item in value[1:-1].split(",")]
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def load_simple_yaml(path: Path) -> dict:
    """Read the flat metadata format used by this project without PyYAML."""
    data: dict = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = _parse_scalar(value)
    return data


@dataclass
class OccupancyGrid:
    cells: list[list[int]]
    resolution: float
    origin_x: float
    origin_y: float
    occupied_value: int = 100

    @property
    def height(self) -> int:
        return len(self.cells)

    @property
    def width(self) -> int:
        return len(self.cells[0]) if self.cells else 0

    @classmethod
    def from_files(cls, metadata_path: Path, grid_path: Path | None = None) -> "OccupancyGrid":
        metadata = load_simple_yaml(metadata_path)
        if grid_path is None:
            grid_path = metadata_path.parent / str(metadata["image"])
        with grid_path.open(newline="", encoding="utf-8") as handle:
            cells = [[int(value) for value in row] for row in csv.reader(handle)]
        origin = metadata["origin"]
        grid = cls(
            cells=cells,
            resolution=float(metadata["resolution"]),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            occupied_value=int(metadata.get("occupied_value", 100)),
        )
        if grid.width != int(metadata["width"]) or grid.height != int(metadata["height"]):
            raise ValueError("Map dimensions do not match building_map.yaml")
        return grid

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) // self.resolution)
        row_from_bottom = int((y - self.origin_y) // self.resolution)
        row = self.height - 1 - row_from_bottom
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        row_from_bottom = self.height - 1 - row
        y = self.origin_y + (row_from_bottom + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def occupied(self, row: int, col: int) -> bool:
        return not self.in_bounds(row, col) or self.cells[row][col] >= self.occupied_value

    def world_occupied(self, x: float, y: float) -> bool:
        return self.occupied(*self.world_to_cell(x, y))

    def inflated(self, radius_m: float) -> "OccupancyGrid":
        radius_cells = ceil(radius_m / self.resolution)
        occupied_cells = [
            (row, col)
            for row in range(self.height)
            for col in range(self.width)
            if self.occupied(row, col)
        ]
        new_cells = [row[:] for row in self.cells]
        for source_row, source_col in occupied_cells:
            for row in range(source_row - radius_cells, source_row + radius_cells + 1):
                for col in range(source_col - radius_cells, source_col + radius_cells + 1):
                    if not self.in_bounds(row, col):
                        continue
                    if hypot(row - source_row, col - source_col) * self.resolution <= radius_m:
                        new_cells[row][col] = self.occupied_value
        return OccupancyGrid(
            new_cells,
            self.resolution,
            self.origin_x,
            self.origin_y,
            self.occupied_value,
        )

