#!/usr/bin/env python3
"""Generate the occupancy CSV used by the Webots project.

The geometry list mirrors the wall boxes in
`worlds/cmpe185_indoor_delivery.wbt`. Coordinates use the student project
frame, where x points right and y points up on the map.
"""

from __future__ import annotations

import csv
from pathlib import Path


RESOLUTION = 0.1
ORIGIN_X = -4.0
ORIGIN_Y = -3.0
WIDTH = 80
HEIGHT = 60

# Rectangles are (xmin, xmax, ymin, ymax), inclusive by physical overlap.
WALL_RECTANGLES = [
    (-4.0, -3.9, -3.0, 3.0),
    (3.9, 4.0, -3.0, 3.0),
    (-4.0, 4.0, -3.0, -2.9),
    (-4.0, 4.0, 2.9, 3.0),
    (-4.0, -2.6, -0.65, -0.55),
    (-1.4, 1.4, -0.65, -0.55),
    (2.6, 4.0, -0.65, -0.55),
    (-4.0, -2.6, 0.55, 0.65),
    (-1.4, 1.4, 0.55, 0.65),
    (2.6, 4.0, 0.55, 0.65),
    (-0.05, 0.05, -3.0, -0.6),
    (-0.05, 0.05, 0.6, 3.0),
]


def cell_overlaps_rectangle(row: int, col: int, rectangle: tuple[float, ...]) -> bool:
    """Return True when a grid cell has nonzero overlap with a wall."""
    xmin, xmax, ymin, ymax = rectangle
    cell_xmin = ORIGIN_X + col * RESOLUTION
    cell_xmax = cell_xmin + RESOLUTION
    # CSV rows are top to bottom.
    cell_ymax = ORIGIN_Y + (HEIGHT - row) * RESOLUTION
    cell_ymin = cell_ymax - RESOLUTION
    return cell_xmax > xmin and cell_xmin < xmax and cell_ymax > ymin and cell_ymin < ymax


def build_grid() -> list[list[int]]:
    grid = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for row in range(HEIGHT):
        for col in range(WIDTH):
            if any(cell_overlaps_rectangle(row, col, wall) for wall in WALL_RECTANGLES):
                grid[row][col] = 100
    return grid


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    output_path = project_root / "maps" / "building_map.csv"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerows(build_grid())
    print(f"Wrote {HEIGHT} x {WIDTH} occupancy grid to {output_path}")


if __name__ == "__main__":
    main()

