# CMPE 185 Autonomous Indoor Delivery Project

## Software

- Webots R2025a
- Python 3 using the interpreter configured by Webots
- No third-party package is needed to run the controllers
- Matplotlib is optional and used only by the offline plotting tools

## First run

1. Extract the complete ZIP; 
2. From the project root, run the three offline checks:

   ```bash
   python3 tools/validate_project.py
   python3 -m unittest discover -s tests -v
   python3 tools/simulate_baseline.py
   ```

3. Open `worlds/cmpe185_indoor_delivery.wbt` in Webots R2025a. On first load,
   allow Webots network access so it can download the version-pinned official
   e-puck PROTO and its assets.
4. Reset the simulation, leave the blocker at position 1, and run without
   pressing `1`, `2`, `3`, `K`, or `N`.
5. The robot should follow HOME → A → B → C → HOME and stop. The evaluation
   Supervisor writes a CSV under `results/`.
6. Check the completed run:

   ```bash
   python3 tools/check_run_health.py results/run_YYYYMMDD_HHMMSS.csv --require-complete
   python3 tools/analyze_run.py results/run_YYYYMMDD_HHMMSS.csv
   ```

The deterministic controller-in-loop reference completes 23/23 waypoints in
531.9 s, travels 33.37 m, reaches all four ordered ground-truth goals, returns
within 0.074 m of HOME, and keeps at least 0.084 m body clearance. A Webots run
may differ slightly because of contact dynamics, but it must complete the same
route without a collision or permanent emergency turn. See
`docs/baseline_validation.md` and `reference/expected_baseline_route.png`.

The starter controller uses encoder odometry and the provided waypoints. It
does not use GPS, Supervisor pose, or the evaluation log. Its initial pose is
configured only to make the starter demonstration reproducible; teams replace
this assumption for uncertain-start and kidnapped-robot tests.

## Correct drive model

The supplied Webots R2025a e-puck2 model uses:

- wheel radius: `0.0200 m`
- geometric wheel-center spacing: `0.0520 m`
- calibrated effective track width for odometry/control: `0.05691 m`
- collision-body radius: `0.037 m`
- command limit: `6.28 rad/s` (below the model's motor limit)
- baseline cruise speed: `0.09 m/s`

Both encoder odometry and inverse kinematics use the same effective track
width. The calibrated effective value accounts for the Webots contact model during
turns; substituting the geometric axle length causes heading drift. The supplied
Release 4 reference value was `0.0574 m`; the team calibration now uses
`0.05691 m`. Details and
equations are in `docs/drive_model_and_expected_behavior.md`.

## Coordinate convention

The project and Webots world share the ENU `x-y` floor plane:

- `x` points right on the supplied map.
- `y` points upward on the supplied map.
- `z` is vertical.
- `theta = 0` points along `+x`; positive heading is counterclockwise about `+z`.
- Robot and LiDAR data use FLU: local `+x` forward, `+y` left, `+z` up.

LiDAR point-cloud `(x, y)` is already `(x_forward, y_left)`. Do not swap or
negate these components.

## Package contents

- `worlds/`: corrected Webots world
- `controllers/project_starter/`: sensing, odometry, visualization, conservative
  waypoint baseline, recovery state machine, and student implementation hooks
- `controllers/evaluation_supervisor/`: public ground-truth logger
- `maps/`: occupancy grid, metadata, mission goals, and nominal waypoints
- `config/`: robot, controller, and evaluation parameters
- `reference/`: canonical expected trace, summary, and trajectory figures
- `docs/`: assignment, baseline validation, drive model, and project guidance
- `tools/`: validation, simulation, health checking, plotting, and drive calibration
- `tests/`: frame, map, odometry, controller, safety, and full-route checks

Search the starter controller for `STUDENT TODO`. Required student work is
concentrated in `localization.py`, `planning.py`, and `navigation.py`, with
`mapping.py` available for a mapping/SLAM extension. Do not modify the public
Supervisor for evaluated runs.

## Evaluation controls

With the Webots 3D window focused:

- `1`, `2`, `3`: move the unexpected obstacle
- `K`: apply the kidnapped-robot pose change
- `N`: begin a new CSV log
- `H`: print the controls

The supplied starter is expected to complete only the untouched nominal setup
(blocker position 1). Positions 2 and 3 and `K` are deliberate student tests.


# cmpe185-robot-project
