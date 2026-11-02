#!/usr/bin/env python3
"""Plot the declared route and the deterministic Release 4 reference trace."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from simulate_baseline import STATIC_RECTANGLES, blocker_rectangle  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trace",
        type=Path,
        default=PROJECT_ROOT / "reference" / "expected_baseline_trace.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "reference" / "expected_baseline_route.png",
    )
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
    except ImportError as error:
        raise SystemExit("Matplotlib is required; install requirements-analysis.txt") from error

    waypoints = json.loads(
        (PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8")
    )
    with args.trace.open(newline="", encoding="utf-8") as handle:
        trace = list(csv.DictReader(handle))

    route = waypoints["baseline_waypoints"]
    trajectory_x = [float(row["gt_x_m"]) for row in trace]
    trajectory_y = [float(row["gt_y_m"]) for row in trace]

    figure, axis = plt.subplots(figsize=(10.2, 7.4))
    for wall in STATIC_RECTANGLES:
        axis.add_patch(
            Rectangle(
                (wall.xmin, wall.ymin),
                wall.xmax - wall.xmin,
                wall.ymax - wall.ymin,
                facecolor="#334155",
                edgecolor="#0f172a",
                linewidth=0.7,
                zorder=1,
            )
        )
    blocker = blocker_rectangle()
    axis.add_patch(
        Rectangle(
            (blocker.xmin, blocker.ymin),
            blocker.xmax - blocker.xmin,
            blocker.ymax - blocker.ymin,
            facecolor="#ef4444",
            edgecolor="#991b1b",
            linewidth=1.0,
            label="default blocker",
            zorder=3,
        )
    )

    axis.plot(
        [point[0] for point in route],
        [point[1] for point in route],
        linestyle="--",
        color="#64748b",
        linewidth=1.5,
        label="provided waypoint route",
        zorder=4,
    )
    axis.plot(
        trajectory_x,
        trajectory_y,
        color="#0ea5e9",
        linewidth=2.3,
        label="deterministic reference trajectory",
        zorder=5,
    )

    goal_lookup = {
        goal["name"]: tuple(goal["position"]) for goal in waypoints["delivery_goals"]
    }
    goal_lookup["HOME"] = tuple(waypoints["home"]["position"])
    colors = {"HOME": "#7c3aed", "A": "#2563eb", "B": "#16a34a", "C": "#f97316"}
    for name in ["HOME", "A", "B", "C"]:
        x, y = goal_lookup[name]
        axis.add_patch(
            Circle((x, y), 0.20, facecolor=colors[name], edgecolor="white", linewidth=1.2, alpha=0.28, zorder=2)
        )
        axis.scatter([x], [y], s=65, color=colors[name], edgecolor="white", linewidth=0.8, zorder=6)
        axis.annotate(name, (x, y), xytext=(7, 8), textcoords="offset points", weight="bold", color=colors[name], zorder=7)

    axis.set_xlim(-4.25, 4.25)
    axis.set_ylim(-3.25, 3.25)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("project x [m]")
    axis.set_ylabel("project y [m]")
    axis.set_title("Release 4 nominal baseline: HOME → A → B → C → HOME")
    axis.grid(True, color="#cbd5e1", alpha=0.55, linewidth=0.6)
    axis.legend(loc="lower center", ncol=3, frameon=True)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, facecolor="white")
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
