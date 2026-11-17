"""Run Task 1 unit tests plus repeatable Webots localization scenarios.

The Webots scenarios are driven only by the evaluation Supervisor.  The student
controller still sees only encoders, LiDAR, the static map, and its configured
initialization mode.
"""

from __future__ import annotations

import argparse
import csv
from contextlib import contextmanager
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORLD = PROJECT_ROOT / "worlds" / "cmpe185_indoor_delivery.wbt"
DEFAULT_KIDNAP_TIMES = (10.0, 30.0, 50.0)
DEFAULT_UNCERTAIN_DURATION_S = 30.0
DEFAULT_POST_KIDNAP_DURATION_S = 120.0
DEFAULT_UNCERTAIN_START_POSE = (2.0, 1.65, -0.9)
PRODUCTION_SUPERVISOR = PROJECT_ROOT / "controllers" / "evaluation_supervisor" / "evaluation_supervisor.py"
TASK1_SUPERVISOR_FIXTURE = PROJECT_ROOT / "tests" / "webots" / "evaluation_supervisor_task1.py"



@contextmanager
def installed_task1_test_supervisor():
    """Temporarily install the test-only automated Supervisor for a Webots run.

    The committed production evaluation Supervisor remains the original instructor
    version.  The runner swaps in the fixture only for the duration of the subprocess
    and restores the prior bytes in ``finally`` so failed/interrupted tests do not
    leave production code modified.
    """
    if not TASK1_SUPERVISOR_FIXTURE.exists():
        raise FileNotFoundError(f"Task 1 Supervisor fixture not found: {TASK1_SUPERVISOR_FIXTURE}")

    existed = PRODUCTION_SUPERVISOR.exists()
    original = PRODUCTION_SUPERVISOR.read_bytes() if existed else None
    PRODUCTION_SUPERVISOR.parent.mkdir(parents=True, exist_ok=True)
    PRODUCTION_SUPERVISOR.write_bytes(TASK1_SUPERVISOR_FIXTURE.read_bytes())
    try:
        yield
    finally:
        if existed:
            PRODUCTION_SUPERVISOR.write_bytes(original)
        elif PRODUCTION_SUPERVISOR.exists():
            PRODUCTION_SUPERVISOR.unlink()

def find_webots(explicit: str | None) -> Path | None:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))

    for name in ("webots", "webots.exe"):
        found = shutil.which(name)
        if found:
            candidates.append(Path(found))

    webots_home = os.environ.get("WEBOTS_HOME")
    if webots_home:
        home = Path(webots_home)
        candidates.extend(
            [
                home / "msys64" / "mingw64" / "bin" / "webots.exe",
                home / "webots.exe",
                home / "webots",
            ]
        )

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return None


def newest_new_file(directory: Path, pattern: str, before: set[Path]) -> Path | None:
    current = {path.resolve() for path in directory.glob(pattern)} if directory.exists() else set()
    new_files = current - before
    if not new_files:
        return None
    return max(new_files, key=lambda path: path.stat().st_mtime)


def read_rows(path: Path | None) -> list[dict[str, str]]:
    if path is None or not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def wrap_angle(value: float) -> float:
    while value >= 3.141592653589793:
        value -= 2.0 * 3.141592653589793
    while value < -3.141592653589793:
        value += 2.0 * 3.141592653589793
    return value


def final_belief_contains_ground_truth(
    mcl_row: dict[str, str],
    evaluation_rows: list[dict[str, str]] | None,
    position_tolerance_m: float = 0.35,
    heading_tolerance_rad: float = 0.50,
) -> tuple[bool, float]:
    """Offline evaluation only: check whether a logged leading mode matches truth.

    This function is used only by the development runner after a Webots trial.
    Ground truth never enters the student controller or localizer.
    """
    if not evaluation_rows:
        return True, float("nan")

    truth = evaluation_rows[-1]
    gt_x = as_float(truth, "gt_x_m", float("nan"))
    gt_y = as_float(truth, "gt_y_m", float("nan"))
    gt_theta = as_float(truth, "gt_theta_rad", float("nan"))
    if not all(value == value for value in (gt_x, gt_y, gt_theta)):
        return True, float("nan")

    best_position_error = float("inf")
    for prefix in ("top", "second", "third"):
        x = as_float(mcl_row, f"{prefix}_hypothesis_x_m", float("nan"))
        y = as_float(mcl_row, f"{prefix}_hypothesis_y_m", float("nan"))
        theta = as_float(mcl_row, f"{prefix}_hypothesis_theta_rad", float("nan"))
        if not all(value == value for value in (x, y, theta)):
            continue
        position_error = ((x - gt_x) ** 2 + (y - gt_y) ** 2) ** 0.5
        heading_error = abs(wrap_angle(theta - gt_theta))
        best_position_error = min(best_position_error, position_error)
        if position_error <= position_tolerance_m and heading_error <= heading_tolerance_rad:
            return True, position_error

    return False, best_position_error


def analyze_scenario(
    name: str,
    mcl_rows: list[dict[str, str]],
    evaluation_rows: list[dict[str, str]] | None = None,
) -> tuple[bool, str]:
    if not mcl_rows:
        return False, "no MCL debug rows were produced"

    states = [row.get("state", "") for row in mcl_rows]
    final = mcl_rows[-1]
    final_state = final.get("state", "")
    top = as_float(final, "top_hypothesis_probability")
    second = as_float(final, "second_hypothesis_probability")
    third = as_float(final, "third_hypothesis_probability")
    score = as_float(final, "measurement_score")
    ambiguous = final.get("recovery_ambiguous", "0") == "1"

    if name == "nominal":
        passed = "TRACKING" in states and final_state == "TRACKING"
        note = f"final={final_state}, score={score:.3f}"
        return passed, note

    if name == "uncertain_start":
        # A correct global belief may remain multi-modal at locally ambiguous
        # poses, but the offline evaluator must confirm that the physical pose
        # is represented. Ground truth is used only here in the test harness,
        # never by the student controller/localizer.
        hypothesis_count = int(as_float(final, "recovery_hypothesis_count"))
        translation = as_float(final, "recovery_translation_m")
        observations = int(as_float(final, "recovery_observation_count"))
        explored = translation >= 0.35 and observations >= 10

        truth_ok = False
        truth_note = "truth=unavailable"
        if evaluation_rows:
            final_eval = evaluation_rows[-1]
            if final_state == "TRACKING":
                position_error = as_float(final_eval, "position_error_m", float("inf"))
                heading_error = as_float(final_eval, "heading_error_rad", float("inf"))
                truth_ok = position_error <= 0.35 and heading_error <= 0.50
                truth_note = f"truth_error={position_error:.3f}m/{heading_error:.3f}rad"
            else:
                truth_ok, truth_error = final_belief_contains_ground_truth(
                    final, evaluation_rows
                )
                truth_note = (
                    "truth_mode=yes"
                    if truth_ok
                    else f"truth_mode=no(best={truth_error:.3f}m)"
                )

        uniquely_localized = final_state == "TRACKING" and truth_ok
        valid_multimodal = (
            final_state == "LOCALIZING"
            and ambiguous
            and hypothesis_count >= 2
            and explored
            and truth_ok
        )
        passed = uniquely_localized or valid_multimodal
        note = (
            f"final={final_state}, ambiguous={int(ambiguous)}, "
            f"hypotheses={hypothesis_count}, translation={translation:.3f}m, "
            f"observations={observations}, p1={top:.3f}, p2={second:.3f}, "
            f"p3={third:.3f}, score={score:.3f}, {truth_note}"
        )
        return passed, note

    if name == "kidnap":
        saw_tracking_before_loss = False
        saw_loss = False
        for state in states:
            if state == "TRACKING" and not saw_loss:
                saw_tracking_before_loss = True
            if state == "LOST":
                saw_loss = True
        translation = as_float(final, "recovery_translation_m")
        observations = int(as_float(final, "recovery_observation_count"))
        hypothesis_count = int(as_float(final, "recovery_hypothesis_count"))
        valid_ambiguous = (
            final_state == "LOCALIZING"
            and ambiguous
            and hypothesis_count >= 2
            and translation >= 0.35
            and observations >= 10
            and score >= 0.70
        )
        recovered_or_ambiguous = final_state == "TRACKING" or valid_ambiguous
        final_motion_stopped = True
        final_controller_state = ""
        if evaluation_rows:
            final_eval = evaluation_rows[-1]
            final_controller_state = final_eval.get("controller_state", "")
            final_linear = abs(as_float(final_eval, "commanded_linear_speed_m_s"))
            final_angular = abs(as_float(final_eval, "commanded_angular_speed_rad_s"))
            final_motion_stopped = final_linear < 1e-4 and final_angular < 1e-4

        # A uniquely recovered robot may already be driving the route again.
        # A multimodal result should be evaluated only after bounded exploratory
        # motion has ended, proving that extra safe motion did not break the tie.
        settled = final_state == "TRACKING" or final_motion_stopped
        truth_mode_present, truth_mode_error = final_belief_contains_ground_truth(
            final, evaluation_rows
        )
        passed = (
            saw_tracking_before_loss
            and saw_loss
            and recovered_or_ambiguous
            and settled
            and truth_mode_present
        )
        truth_note = (
            "truth_mode=yes"
            if truth_mode_present
            else f"truth_mode=no(best={truth_mode_error:.3f}m)"
        )
        note = (
            f"final={final_state}, ambiguous={int(ambiguous)}, "
            f"translation={translation:.3f}m, observations={observations}, "
            f"p1={top:.3f}, p2={second:.3f}, p3={third:.3f}, score={score:.3f}, "
            f"motion={'stopped' if final_motion_stopped else 'active'}, {truth_note}"
            + (f", controller={final_controller_state}" if final_controller_state else "")
        )
        return passed, note

    return False, f"unknown scenario {name}"


def run_unit_tests() -> bool:
    command = [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"]
    print("\n[TASK1] Unit/regression tests")
    result = subprocess.run(command, cwd=PROJECT_ROOT)
    return result.returncode == 0


def run_webots_scenario(
    webots: Path,
    scenario: str,
    output_dir: Path | None,
    rendering: bool,
    durations: dict[str, float],
    kidnap_time_s: float,
    output_label: str | None = None,
    run_stamp: str = "",
) -> tuple[bool, str]:
    results_dir = PROJECT_ROOT / "results"
    runlogs_dir = PROJECT_ROOT / "runlogs"
    before_results = {path.resolve() for path in results_dir.glob("run_*.csv")} if results_dir.exists() else set()
    before_mcl = {path.resolve() for path in runlogs_dir.glob("mcl_*.csv")} if runlogs_dir.exists() else set()

    env = os.environ.copy()
    env["CMPE185_AUTO_SCENARIO"] = scenario
    env["CMPE185_MCL_INITIALIZATION_MODE"] = (
        "global_free_space" if scenario == "uncertain_start" else "nominal"
    )
    env["CMPE185_AUTO_DURATION_S"] = str(durations[scenario])
    if scenario == "uncertain_start":
        env.setdefault(
            "CMPE185_AUTO_UNCERTAIN_POSE",
            ",".join(f"{value:g}" for value in DEFAULT_UNCERTAIN_START_POSE),
        )
    if scenario == "kidnap":
        env["CMPE185_AUTO_KIDNAP_TIME_S"] = str(kidnap_time_s)

    command = [
        str(webots),
        "--stdout",
        "--stderr",
        "--batch",
        "--mode=fast",
    ]
    if not rendering:
        command.append("--no-rendering")
    command.append(str(WORLD))

    label = output_label or scenario
    print(f"\n[TASK1] Webots scenario: {label}")
    with installed_task1_test_supervisor():
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(180.0, durations[scenario] + 90.0),
        )

    if output_dir is not None:
        console_path = output_dir / f"{label}_console.txt"
    else:
        runlogs_dir.mkdir(parents=True, exist_ok=True)
        suffix = f"_{run_stamp}" if run_stamp else ""
        console_path = runlogs_dir / f"task1_{label}{suffix}_console.txt"
    console_path.write_text(completed.stdout or "", encoding="utf-8")

    result_file = newest_new_file(results_dir, "run_*.csv", before_results)
    mcl_file = newest_new_file(runlogs_dir, "mcl_*.csv", before_mcl)

    if result_file is not None:
        print(f"[TASK1] evaluation log: {result_file}")
        if output_dir is not None:
            shutil.copy2(result_file, output_dir / f"{label}_evaluation.csv")
    if mcl_file is not None:
        print(f"[TASK1] MCL log: {mcl_file}")
        if output_dir is not None:
            shutil.copy2(mcl_file, output_dir / f"{label}_mcl.csv")

    if completed.returncode != 0:
        return False, f"Webots exited with code {completed.returncode}; see {console_path.name}"

    passed, note = analyze_scenario(
        scenario,
        read_rows(mcl_file),
        read_rows(result_file),
    )
    return passed, note


def parse_kidnap_times(value: str) -> list[float]:
    try:
        times = [
            max(0.0, float(item.strip()))
            for item in value.split(",")
            if item.strip()
        ]
    except ValueError as exc:
        raise ValueError("kidnap times must be comma-separated numbers") from exc
    if not times:
        raise ValueError("at least one kidnap time is required")
    return times


def record_result(
    summary: list[dict[str, str]], label: str, passed: bool, note: str
) -> None:
    print(f"[TASK1] {label}: {'PASS' if passed else 'FAIL'} - {note}")
    summary.append({"check": label, "pass": str(passed).lower(), "notes": note})


def run_scenario_checked(
    webots: Path,
    scenario: str,
    output_dir: Path | None,
    *,
    rendering: bool,
    durations: dict[str, float],
    kidnap_time_s: float,
    output_label: str | None = None,
    run_stamp: str = "",
) -> tuple[bool, str]:
    try:
        return run_webots_scenario(
            webots,
            scenario,
            output_dir,
            rendering=rendering,
            durations=durations,
            kidnap_time_s=kidnap_time_s,
            output_label=output_label,
            run_stamp=run_stamp,
        )
    except subprocess.TimeoutExpired:
        return False, "Webots process exceeded its scenario timeout"


def run_kidnap_sweep(
    webots: Path,
    output_dir: Path | None,
    summary: list[dict[str, str]],
    *,
    rendering: bool,
    base_durations: dict[str, float],
    kidnap_times: list[float],
    post_kidnap_duration_s: float,
    run_stamp: str = "",
) -> None:
    for kidnap_time in kidnap_times:
        label = f"kidnap_t{kidnap_time:g}"
        durations = dict(base_durations)
        durations["kidnap"] = kidnap_time + max(1.0, post_kidnap_duration_s)
        passed, note = run_scenario_checked(
            webots,
            "kidnap",
            output_dir,
            rendering=rendering,
            durations=durations,
            kidnap_time_s=kidnap_time,
            output_label=label,
            run_stamp=run_stamp,
        )
        record_result(summary, label, passed, note)


def write_summary(
    output_dir: Path | None, rows: list[dict[str, str]], stamp: str
) -> Path:
    if output_dir is not None:
        path = output_dir / "task1_summary.csv"
    else:
        results_dir = PROJECT_ROOT / "results"
        results_dir.mkdir(parents=True, exist_ok=True)
        path = results_dir / f"task1_summary_{stamp}.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["check", "pass", "notes"])
        writer.writeheader()
        writer.writerows(rows)
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", help="Path to webots/webots.exe if it is not on PATH")
    parser.add_argument(
        "--skip-webots",
        action="store_true",
        help="Run only Python unit/regression tests",
    )
    parser.add_argument(
        "--rendering",
        action="store_true",
        help="Keep Webots rendering enabled during automated scenarios",
    )
    parser.add_argument(
        "--scenario",
        choices=("all", "nominal", "uncertain_start", "kidnap", "kidnap_sweep"),
        default="all",
        help=(
            "Run all core scenarios, one scenario, or a kidnapping-time sweep "
            "(default: all)"
        ),
    )
    parser.add_argument(
        "--nominal-duration",
        type=float,
        default=20.0,
        help="Nominal Webots scenario duration in seconds (default: 20)",
    )
    parser.add_argument(
        "--uncertain-duration",
        type=float,
        default=DEFAULT_UNCERTAIN_DURATION_S,
        help="Uncertain/global-start scenario duration in seconds (default: 30)",
    )
    parser.add_argument(
        "--kidnap-duration",
        type=float,
        default=130.0,
        help="Kidnapped scenario total duration in seconds (default: 130)",
    )
    parser.add_argument(
        "--kidnap-time",
        type=float,
        default=20.0,
        help="Time of the automatic K kidnapping in seconds (default: 20)",
    )
    parser.add_argument(
        "--kidnap-times",
        default=",".join(f"{value:g}" for value in DEFAULT_KIDNAP_TIMES),
        help=(
            "Comma-separated K times used by --scenario kidnap_sweep "
            "(default: 10,30,50)"
        ),
    )
    parser.add_argument(
        "--bundle-results",
        action="store_true",
        help=(
            "Also copy scenario logs into task1_results/<timestamp>. "
            "By default outputs stay only in the normal results/ and runlogs/ folders."
        ),
    )
    parser.add_argument(
        "--post-kidnap-duration",
        type=float,
        default=DEFAULT_POST_KIDNAP_DURATION_S,
        help=(
            "Seconds to continue each kidnap_sweep run after K so all bounded "
            "recovery cycles can finish (default: 120)"
        ),
    )
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = None
    if args.bundle_results:
        output_dir = PROJECT_ROOT / "task1_results" / stamp
        output_dir.mkdir(parents=True, exist_ok=True)

    summary: list[dict[str, str]] = []
    unit_ok = run_unit_tests()
    summary.append(
        {"check": "unit_tests", "pass": str(unit_ok).lower(), "notes": "unittest discover"}
    )

    if not args.skip_webots:
        webots = find_webots(args.webots)
        if webots is None:
            note = "Webots executable not found; use --webots PATH or set WEBOTS_HOME"
            print(f"[TASK1] {note}")
            summary.append({"check": "webots", "pass": "false", "notes": note})
        else:
            print(f"[TASK1] Webots: {webots}")
            durations = {
                "nominal": max(1.0, args.nominal_duration),
                "uncertain_start": max(1.0, args.uncertain_duration),
                "kidnap": max(args.kidnap_time + 1.0, args.kidnap_duration),
            }
            try:
                kidnap_times = parse_kidnap_times(args.kidnap_times)
            except ValueError as exc:
                parser.error(f"--kidnap-times {exc}")

            if args.scenario == "kidnap_sweep":
                run_kidnap_sweep(
                    webots,
                    output_dir,
                    summary,
                    rendering=args.rendering,
                    base_durations=durations,
                    kidnap_times=kidnap_times,
                    post_kidnap_duration_s=args.post_kidnap_duration,
                    run_stamp=stamp,
                )
            elif args.scenario == "all":
                for scenario in ("nominal", "uncertain_start"):
                    passed, note = run_scenario_checked(
                        webots,
                        scenario,
                        output_dir,
                        rendering=args.rendering,
                        durations=durations,
                        kidnap_time_s=max(0.0, args.kidnap_time),
                        run_stamp=stamp,
                    )
                    record_result(summary, scenario, passed, note)
                run_kidnap_sweep(
                    webots,
                    output_dir,
                    summary,
                    rendering=args.rendering,
                    base_durations=durations,
                    kidnap_times=kidnap_times,
                    post_kidnap_duration_s=args.post_kidnap_duration,
                    run_stamp=stamp,
                )
            else:
                scenario = args.scenario
                passed, note = run_scenario_checked(
                    webots,
                    scenario,
                    output_dir,
                    rendering=args.rendering,
                    durations=durations,
                    kidnap_time_s=max(0.0, args.kidnap_time),
                    run_stamp=stamp,
                )
                record_result(summary, scenario, passed, note)

    summary_path = write_summary(output_dir, summary, stamp)
    print("\n[TASK1] Normal outputs: results/ and runlogs/")
    if output_dir is not None:
        print(f"[TASK1] Optional bundle: {output_dir}")
    print(f"[TASK1] Summary: {summary_path}")
    return 0 if all(row["pass"] == "true" for row in summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
