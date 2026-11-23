"""Temporary Task 1 MCL CSV logger.

This file is intentionally isolated from localization/control so it can be
removed after kidnapped-robot behavior is verified in Webots.
"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path


class MCLDebugLogger:
    FIELDNAMES = [
        "time_s",
        "mcl_x_m",
        "mcl_y_m",
        "mcl_theta_rad",
        "state",
        "confidence",
        "ess",
        "position_spread_m",
        "heading_spread_rad",
        "measurement_score",
        "bad_measurement_count",
        "recovery_active",
        "recovery_ambiguous",
        "recovery_count",
        "recovery_hypothesis_count",
        "recovery_translation_m",
        "recovery_observation_count",
        "top_hypothesis_probability",
        "second_hypothesis_probability",
        "third_hypothesis_probability",
        "top_hypothesis_x_m",
        "top_hypothesis_y_m",
        "top_hypothesis_theta_rad",
        "second_hypothesis_x_m",
        "second_hypothesis_y_m",
        "second_hypothesis_theta_rad",
        "third_hypothesis_x_m",
        "third_hypothesis_y_m",
        "third_hypothesis_theta_rad",
        "just_recovered",
    ]

    def __init__(self, project_root: Path, enabled: bool = True) -> None:
        self.enabled = enabled
        self.handle = None
        self.writer = None
        self.path = None
        if not enabled:
            return
        directory = project_root / "runlogs"
        directory.mkdir(exist_ok=True)
        self.path = directory / f"mcl_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv"
        self.handle = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.handle.flush()

    def write(self, time_s: float, pose, state: str, confidence: float, diagnostics: dict, hypotheses) -> None:
        if self.writer is None:
            return
        top = hypotheses[0][0] if hypotheses else 0.0
        second = hypotheses[1][0] if len(hypotheses) > 1 else 0.0
        third = hypotheses[2][0] if len(hypotheses) > 2 else 0.0
        top_pose = hypotheses[0][1] if hypotheses else None
        second_pose = hypotheses[1][1] if len(hypotheses) > 1 else None
        third_pose = hypotheses[2][1] if len(hypotheses) > 2 else None
        self.writer.writerow(
            {
                "time_s": f"{time_s:.3f}",
                "mcl_x_m": f"{pose.x:.6f}",
                "mcl_y_m": f"{pose.y:.6f}",
                "mcl_theta_rad": f"{pose.theta:.6f}",
                "state": state,
                "confidence": f"{confidence:.6f}",
                "ess": diagnostics.get("ess", ""),
                "position_spread_m": diagnostics.get("position_spread_m", ""),
                "heading_spread_rad": diagnostics.get("heading_spread_rad", ""),
                "measurement_score": diagnostics.get("measurement_score", ""),
                "bad_measurement_count": diagnostics.get("bad_measurement_count", ""),
                "recovery_active": int(bool(diagnostics.get("recovery_active", False))),
                "recovery_ambiguous": int(bool(diagnostics.get("recovery_ambiguous", False))),
                "recovery_count": diagnostics.get("recovery_count", ""),
                "recovery_hypothesis_count": diagnostics.get(
                    "recovery_hypothesis_count", ""
                ),
                "recovery_translation_m": diagnostics.get("recovery_translation_m", ""),
                "recovery_observation_count": diagnostics.get("recovery_observation_count", ""),
                "top_hypothesis_probability": top,
                "second_hypothesis_probability": second,
                "third_hypothesis_probability": third,
                "top_hypothesis_x_m": "" if top_pose is None else f"{top_pose.x:.6f}",
                "top_hypothesis_y_m": "" if top_pose is None else f"{top_pose.y:.6f}",
                "top_hypothesis_theta_rad": "" if top_pose is None else f"{top_pose.theta:.6f}",
                "second_hypothesis_x_m": "" if second_pose is None else f"{second_pose.x:.6f}",
                "second_hypothesis_y_m": "" if second_pose is None else f"{second_pose.y:.6f}",
                "second_hypothesis_theta_rad": "" if second_pose is None else f"{second_pose.theta:.6f}",
                "third_hypothesis_x_m": "" if third_pose is None else f"{third_pose.x:.6f}",
                "third_hypothesis_y_m": "" if third_pose is None else f"{third_pose.y:.6f}",
                "third_hypothesis_theta_rad": "" if third_pose is None else f"{third_pose.theta:.6f}",
                "just_recovered": int(bool(diagnostics.get("just_recovered", False))),
            }
        )

    def flush(self) -> None:
        if self.handle is not None:
            self.handle.flush()
