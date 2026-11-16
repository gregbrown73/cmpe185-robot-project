# Evaluation results

The evaluation Supervisor writes `run_YYYYMMDD_HHMMSS.csv` files here. Teams
should commit selected final CSV files or include them in the submitted ZIP,
along with the random seed and scenario configuration used for each run.

## Baseline registry

- `baseline_original_0574/baseline_webots_original_0574_20261115_175013.csv` — pre-width-tuning Webots baseline using
  `effective_track_width_m = 0.0574`. It completed the route but recorded only
  2/4 ground-truth goals; its plot and summary are beside the CSV as the
  original comparison/failure record.
- `baseline_original_0574/deterministic_baseline_original_0574_20261115_174946.csv` — deterministic
  simulator result for the original `0.0574` configuration. It passed 4/4
  goals; its plot and summary are beside the CSV.
- `baseline_calibrated_05691/baseline_webots_calibrated_05691_20261115_180520.csv` — passing calibrated Webots baseline using
  `effective_track_width_m = 0.05691`. It reached 4/4 goals, completed 23/23
  waypoints, had zero collisions, and finished within the HOME tolerance.
- `baseline_calibrated_05691/deterministic_baseline_width_05691_20261115_180455.csv` — deterministic
  simulator result for the same `0.05691` configuration. It passed 4/4 goals;
  its plot and summary are beside the CSV.
- Calibrated configuration-change metadata:
  `../runlogs/baseline_calibrated_05691/00_configuration_change.txt`.
- Original baseline health and analysis logs:
  `../runlogs/baseline_original_0574/`.
- Calibrated baseline health, calibration, and analysis logs:
  `../runlogs/baseline_calibrated_05691/`.

## Which baseline to compare against

Use the **calibrated live Webots baseline** as the primary comparison for live
world experiments:

`baseline_calibrated_05691/baseline_webots_calibrated_05691_20261115_180520.csv`

It uses the current `effective_track_width_m = 0.05691` and measures the
encoder-based estimate against Webots ground truth, so it is the meaningful
baseline for judging MCL, planning, avoidance, and SLAM in the live world.

For tests run with `python tools/simulate_baseline.py`, compare against the
**calibrated deterministic baseline** instead:

`baseline_calibrated_05691/deterministic_baseline_width_05691_20261115_180455.csv`

The original `0.0574` files are historical records only. Keep them to show the
pre-calibration behavior and the failed live run, but do not use them as the
main benchmark for new data. The deterministic original and calibrated plots
both show zero position/heading error because that simulator uses an idealized
plant and the same motion parameters as its odometry reference by default; it
is a software/regression check, not evidence that live odometry has no drift.

Always compare like with like: same execution environment, track-width value,
world/map, scenario, goal order, and metric definitions.

The CSVs and generated plots are locally ignored by Git, so copy selected final
artifacts into the submitted ZIP when required.

