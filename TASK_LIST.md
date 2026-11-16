# CMPE 185 Autonomous Indoor Delivery - Task List

This plan starts from the calibrated odometry baseline and keeps the starter
project intact until each implementation task actually needs a change.

The work is split between two people so that most development can happen in
parallel without both people editing the same files at the same time.

## Team split

### Person 1 - Localization / state estimation lead

Primary ownership:

- Monte Carlo Localization (MCL)
- localization confidence and recovery
- kidnapped-robot recovery
- pose-related diagnostics
- mapping extension core

Likely primary files:

```text
controllers/project_starter/localization.py
controllers/project_starter/mapping.py
localization-related sections of project_starter.py
localization tests
```

### Person 2 - Planning / navigation / tooling lead

Primary ownership:

- A* global planner
- route manager
- obstacle avoidance
- temporary obstacle layer / replanning
- experiment tooling and health checks

Likely primary files:

```text
controllers/project_starter/planning.py
controllers/project_starter/navigation.py
planning/navigation sections of project_starter.py
tools/check_run_health.py
tools/analyze_run.py
planning/navigation tests
```

### Both

Work together on:

- shared interface verification and integration
- integration into `project_starter.py`
- scenario testing
- experiment definitions and final data collection
- debugging interactions between localization and planning
- report, presentation, demo, and final submission

### Merge-conflict rule

Try to avoid both people editing `project_starter.py` at the same time.
For this plan, the rule is now fixed:

1. independent stages must be implemented in separate subsystem files/branches;
2. neither person edits `project_starter.py` during an independent stage unless that stage explicitly says so;
3. after both people finish, merge both branches into the integration branch;
4. only then wire the merged interfaces into `project_starter.py`;
5. the integration branch is the only branch allowed to contain the final combined controller wiring.

## Starting point

- Current calibrated `effective_track_width_m`: `0.05691`.
- Calibrated live baseline: 23/23 supplied waypoints, 4/4 goals, no collision,
  and HOME within tolerance.
- Preserve the existing baseline results as comparison evidence.
- Keep the baseline runnable while building the student method.
- Do not modify the public evaluation Supervisor for evaluated runs.

## Final-system rules

- Mission order: **A -> B -> C -> HOME**.
- Robot code may use encoders, LiDAR, the supplied map, and mission goals.
- Do not use GPS, Supervisor pose/APIs, ground truth, or evaluation logs for
  control/localization.
- Final global planning uses an inflated occupancy grid.
- Canonical total inflation radius: `0.12 m` from `maps/building_map.yaml`.
- The red blocker is absent from the static map.
- Persistent unexpected obstacles must be added to a temporary planning layer
  before replanning.
- `23 waypoints` is a baseline-only completion rule. A* routes may contain any
  number of waypoints.
- Required experiments compare the provided odometry baseline against the full
  student system in all four required scenarios, at least five trials each.
- Finish and freeze required results before the optional mapping/SLAM extension.

---

## Branching flow, stages, and ownership

The project now advances by **stage gates**. Independent work does not carry into the next
stage until both branches have been merged and the gate passes.

```text
STAGE 0 - BASELINE / PREFLIGHT
                    T0 [BOTH]
                       |
                       v
STAGE 1 - CORE ALGORITHMS, INDEPENDENT
       +---------------+----------------+
       |                                |
       v                                v
 T1 MCL core [P1]              T2 A* + T3 tools [P2]
       |                                |
       +---------------+----------------+
                       |
                 MERGE + GATE 1
                       |
                       v
STAGE 2 - RUNTIME SUBSYSTEMS, INDEPENDENT
       +---------------+----------------+
       |                                |
       v                                v
 T5 MCL runtime [P1]          T6 route manager + T7 [P2]
                              avoidance/temp obstacles
       |                                |
       +---------------+----------------+
                       |
                 MERGE + GATE 2
                       |
                       v
STAGE 3 - SHARED CONTROLLER INTEGRATION
                 T4 verify contracts
                       |
                 T8 [BOTH]
                       |
                 MERGE + GATE 3
                       |
                       v
STAGE 4 - REQUIRED SCENARIOS
                 T9 [BOTH]
        P1: uncertain/kidnapped
        P2: nominal/blocked-route
                       |
                 MERGE + GATE 4
                       |
                       v
STAGE 5 - REQUIRED EXPERIMENTS
                 T10 [BOTH]
        P1: uncertain/kidnapped trials
        P2: nominal/blocked trials
                       |
                 FREEZE RESULTS
                       |
                       v
STAGE 6 - OPTIONAL EXTENSION
      P1 mapping core | P2 viz/evaluation support
                 T11 -> MERGE
                       |
                       v
STAGE 7 - FINAL DELIVERABLES
                 T12 [BOTH]
```

### Stage gate rule

At every `MERGE + GATE`:

- both assigned task branches are committed and individually passing their tests;
- both branches are merged into `integration/current`;
- integration tests are run from the merged branch, not from either person's branch;
- any interface mismatch is fixed on the integration branch using the fixed decisions written inside the affected task;
- both people sync from the passing integration branch before beginning the next stage.

This is the default workflow for the rest of this file.

---

<details>
<summary><strong>T0 - Preflight and documentation [BOTH]</strong></summary>

### Goal

Start from a known-good baseline and make sure the repository documentation and
validation tools agree before either person changes controller behavior.

### Fixed decisions for T0

Use these conventions for the entire project; do not create alternate conventions later.

```text
world: x right, y up
theta: counterclockwise, wrapped to [-pi, pi)
robot/LiDAR: +x forward, +y left
Pose2D: (x_m, y_m, theta_rad)
delta_motion: (delta_distance_m, delta_heading_rad)
grid cell: (row, col), rows stored top-to-bottom
```

Use these working branches/worktrees:

```text
feature/person1-localization
feature/person2-planning
integration/current
```

At the end of every independent stage:

```text
1. each person finishes assigned code + tests and commits;
2. stop independent development for that stage;
3. merge Person 1 into integration/current;
4. merge Person 2 into integration/current;
5. resolve subsystem conflicts by the owner below;
6. run the stage gate from the merged integration branch;
7. fix integration-only issues on integration/current;
8. only after the gate passes, sync/branch for the next stage.
```

Conflict ownership:

```text
localization.py / mapping.py                 -> Person 1 decides
planning.py / navigation.py / route files   -> Person 2 decides
tests for one subsystem                     -> subsystem owner decides
project_starter.py / project_config.json     -> resolve together on integration/current
```

Keep this module split unless a later task explicitly says otherwise:

```text
controllers/project_starter/localization.py          Person 1
controllers/project_starter/mapping.py               Person 1
controllers/project_starter/planning.py              Person 2
controllers/project_starter/navigation.py            Person 2
controllers/project_starter/route_manager.py         Person 2 - add in T6
controllers/project_starter/temporary_obstacles.py   Person 2 - add in T7
controllers/project_starter/project_starter.py       integration only
```

Baseline preservation is mandatory throughout the project:

```text
odometry localization
provided 23-waypoint route
starter safety behavior
4/4 mission goals
23/23 supplied waypoints
```

Do not modify the reference artifacts under `reference/`.

### Person 1

- Verify the calibrated `effective_track_width_m = 0.05691` is documented.
- Check localization-related assumptions in the docs:
  - coordinate frame convention;
  - heading sign convention;
  - LiDAR angle convention;
  - odometry delta format.

### Person 2

- Check planning/navigation-related assumptions:
  - map resolution and origin;
  - occupancy values;
  - inflation radius;
  - mission goal order;
  - baseline waypoint behavior.

### Both

- Confirm no controller/config behavior changes are needed yet.
- Save or commit a known-good baseline before implementation starts.
- Create/sync the three branches/worktrees defined above in this task.

### Verify

```bash
python3 tools/validate_project.py
python3 -m unittest discover -s tests -v
python3 tools/simulate_baseline.py
```

Expected deterministic baseline:

```text
23/23 supplied waypoints
4/4 goals
no collision
no blocked state
```

### Handoff

Before splitting up, verify both people are using the fixed T0 conventions for:

- map/world coordinate convention;
- robot x/y/heading convention;
- LiDAR scan angle convention;
- how `delta_motion` is represented;
- where the mission goals come from;
- where runtime configuration is loaded.

**Done when:** all three checks pass and the baseline behavior is unchanged.

</details>

---

<details>
<summary><strong>T1 - Monte Carlo Localization [PERSON 1]</strong></summary>

Primary file:

```text
controllers/project_starter/localization.py
```

### Goal

Replace the placeholder/pass-through localization behavior with a real MCL
estimate based on odometry motion and LiDAR/map agreement.

### Fixed decisions for T1

Use the project coordinate/motion convention from T0. The public localization interface is fixed:

```python
localizer.initialize(occupancy_grid, initialization_mode="nominal")
estimate = localizer.update(odometry_pose, delta_motion, scan_points_robot, occupancy_grid)
localizer.get_state()       # "LOCALIZING", "TRACKING", or "LOST"
localizer.get_confidence()  # float in [0.0, 1.0]
localizer.get_diagnostics() # dict for logging only
```

`StudentLocalizer.update(...)` must continue returning `Pose2D`. Do not replace the return type
with a larger object. Diagnostics use these exact keys:

```text
ess
position_spread_m
heading_spread_rad
measurement_score
just_recovered
```

Starting MCL values:

```text
particle_count = 300
ESS resample threshold = 0.50 * particle_count
maximum LiDAR beams per update = 60
measurement distance sigma = 0.12 m
outlier/random measurement weight = 0.05
TRACKING position spread threshold = 0.20 m
TRACKING heading spread threshold = 0.30 rad
LOST position spread threshold = 0.75 m
recovery particle reinjection fraction = 0.25
```

Use this first confidence definition so T1/T3/T5 agree on what is logged:

```text
measurement_score = mean per-beam likelihood, normalized to [0, 1]
position_quality = clamp(1 - position_spread_m / 0.75, 0, 1)
heading_quality = clamp(1 - heading_spread_rad / 0.80, 0, 1)
confidence = 0.40*position_quality + 0.30*heading_quality + 0.30*measurement_score
```

Initialization mode names are exact:

```text
nominal           -> Gaussian around configured initial pose
global_free_space -> particles sampled across valid free cells
```

These are starting values; tune numeric values only if testing proves they need adjustment. Keep
the API, key names, and state names unchanged.

### 1. Particle representation

Each particle should contain at least:

```text
x
 y
 heading
 weight
```

Use a configurable particle count rather than hard-coding it throughout the
implementation.

### 2. Initialization

Implement two useful initialization styles:

- **nominal:** particles clustered around the expected start area;
- **uncertain start:** particles spread more broadly across valid free space.

Reject samples that land in occupied/inflated map cells.

Important checks:

- headings wrap correctly to `[-pi, pi)` or the project's chosen convention;
- all weights start normalized;
- initialization is reproducible with a fixed seed.

### 3. Motion update

Use odometry motion input similar to:

```text
delta_motion = (distance, heading_change)
```

For each particle:

1. perturb translation with noise;
2. perturb rotation with noise;
3. update x/y using particle heading;
4. update heading;
5. wrap heading.

Keep the noise parameters configurable.

### 4. LiDAR measurement likelihood

Compare predicted map agreement against the actual LiDAR scan.

A practical method is a likelihood-field approach:

1. precompute or query distance to the nearest occupied cell;
2. transform selected LiDAR endpoints from robot frame into world frame for
   each particle;
3. score endpoints based on distance to mapped obstacles;
4. combine endpoint scores into the particle likelihood.

Subsample the scan so this runs fast enough in Webots.

### 5. Unexpected-obstacle robustness

The red blocker is intentionally missing from the static map.

Do **not** make one unexpected LiDAR return destroy an otherwise-correct
particle.

Use one or more of:

- likelihood floor;
- random/outlier measurement component;
- capped per-beam penalty;
- ignored invalid/max-range returns.

### 6. Normalize weights and compute ESS

After measurement update:

```text
normalize weights
compute ESS = 1 / sum(w_i^2)
```

Handle degenerate cases safely. If all weights collapse numerically, recover
instead of dividing by zero or producing NaN.

### 7. Resampling

Use systematic or low-variance resampling when ESS falls below a threshold.

Do not resample every step unless justified.

### 8. Pose estimate

Estimate:

- x = weighted mean;
- y = weighted mean;
- heading = weighted circular mean.

Never average angles with a normal arithmetic mean near `-pi/+pi`.

### 9. Confidence estimate

Expose confidence metrics for integration. Good candidates:

- x/y particle spread;
- circular heading spread;
- measurement likelihood;
- ESS.

ESS alone is not enough because a wrong belief can still have evenly weighted
particles.

Provide a simple output such as:

```text
localization_state = LOCALIZING / TRACKING / LOST
confidence = numeric or boolean threshold result
```

### 10. Kidnapped recovery

When LiDAR agreement becomes persistently terrible or the belief collapses:

- mark localization as lost;
- inject/reseed a fraction of particles more broadly;
- temporarily broaden noise if useful;
- allow the belief to reconverge;
- do not use ground truth for detection/recovery.

### Tests - Person 1 owns

Add tests for:

- valid free-space initialization;
- normalized weights;
- ESS calculation;
- conditional resampling;
- deterministic motion update with fixed seed;
- heading wrapping;
- circular mean near `-pi/+pi`;
- correct-pose likelihood preference;
- tolerance to several unexpected scan returns;
- zero/degenerate-weight recovery;
- recovery/reinitialization behavior.

### Deliverable to Person 2 / integration

Implement the exact localization API defined below in this task:

```python
localizer.initialize(occupancy_grid, initialization_mode="nominal")
estimate = localizer.update(odometry_pose, delta_motion, scan_points_robot, occupancy_grid)
localizer.get_state()
localizer.get_confidence()
localizer.get_diagnostics()
```

Do not rename or replace this interface. Planning/navigation should depend only on
the returned pose, state/confidence, and documented diagnostics - never on the
particle-filter internals.

**Done when:** LiDAR meaningfully changes particle weights, pose estimates are
stable in Webots, the fixed API below is implemented exactly, and the filter can
recover after the kidnapped-robot event.

</details>

---

<details>
<summary><strong>T2 - A* global planner [PERSON 2]</strong></summary>

Primary file:

```text
controllers/project_starter/planning.py
```

### Goal

Generate safe map-based routes from an arbitrary estimated start pose to the
current mission goal.

### Fixed decisions for T2

Keep the existing low-level `astar(...)` function and expose this exact world-level helper:

```python
route, status = plan_world_path(planning_grid, start_xy, goal_xy)
```

`route` is a list of metric `(x, y)` world points. `status` is exactly one of:

```text
SUCCESS
NO_PATH
INVALID_START
INVALID_GOAL
```

Planning rules are fixed:

```text
8-connected A*
cardinal cost = 1
diagonal cost = sqrt(2)
octile heuristic
no diagonal corner cutting
combined occupancy inflated once by 0.12 m total
line-of-sight simplification with collision checking
append the exact mission-goal coordinate as the last route point when safe
```

The planner owns world/grid conversion. Do not use spacing-only simplification in the final
student planner.

### 1. Planning grid

Use the occupancy grid plus the canonical `0.12 m` total inflation radius.

The planner should accept a planning grid rather than assuming the static map is
always the only source. This becomes important in T4/T7 when temporary obstacles
are added.

### 2. Start/goal conversion

Implement and test:

```text
world x/y -> grid row/column
grid cell -> metric world center
```

Handle map origin and resolution consistently.

Reject or safely handle start/goal cells outside the map.

### 3. A* search

Use 8-connected motion:

```text
cardinal move cost = 1
diagonal move cost = sqrt(2)
```

Use Euclidean or octile heuristic.

Maintain standard A* state:

```text
open set
g-score
parent/came-from
closed/visited state
```

### 4. Safety rules

- reject occupied start/goal cells;
- never step into occupied/inflated cells;
- reject diagonal corner cutting;
- return an empty/no-path result when unreachable;
- do not crash if start equals goal.

### 5. Path reconstruction

Reconstruct the cell path from goal back to start, reverse it, then convert to
metric world coordinates.

### 6. Safe path simplification

Replace spacing-only simplification.

Use either:

- remove only collinear intermediate cells; or
- line-of-sight simplification where every proposed shortcut is checked against
  the inflated planning grid.

Every final segment must remain collision-free.

### 7. Planner outputs

Return enough information for the route manager to distinguish:

```text
SUCCESS
NO_PATH
INVALID_START
INVALID_GOAL
```

This makes T6 easier to debug than returning a generic empty list for every
failure.

### Tests - Person 2 owns

Add tests for:

- start equals goal;
- occupied start;
- occupied goal;
- outside-map start/goal;
- no-path case;
- known optimal path on a small grid;
- diagonal corner-cut rejection;
- narrow passage removed by inflation;
- world/grid coordinate conversions;
- simplified path remains collision-free.

### Deliverable to Person 1 / integration

Implement the exact planner interface defined below in this task:

```python
route, status = plan_world_path(planning_grid, start_xy, goal_xy)
```

Do not rename or replace this interface. Localization only supplies the estimated
metric pose; the planner owns world/grid conversion and returns metric route points.

**Done when:** A* returns safe metric routes to arbitrary goals through the
fixed API and fails safely when no route exists.

</details>

---

<details>
<summary><strong>T3 - Tests, logging, and experiment tooling [PERSON 2 lead]</strong></summary>

Person 1 should provide the localization diagnostic fields they want logged,
but Person 2 owns most tooling changes.

### Goal

Make failures diagnosable before full integration and avoid hard-coding baseline
assumptions into student-method evaluation.

### Fixed decisions for T3

Do not modify the public evaluation Supervisor to add student-only diagnostics. Write a separate
controller-side CSV or JSONL keyed by simulation time.

Use these exact event names:

```text
GOAL_REACHED
REPLAN_REQUEST
LOCALIZATION_LOST
LOCALIZATION_RECOVERED
NO_PATH
MISSION_COMPLETE
```

Use these exact common diagnostic field names when the corresponding subsystem exists:

```text
time
experiment_method
active_goal
route_index
route_count
route_version
replan_count
replan_reason
avoidance_state
temporary_obstacle_count
mcl_state
mcl_confidence
mcl_ess
particle_position_spread_m
particle_heading_spread_rad
```

The baseline health profile may keep the 23-waypoint requirement. The student mission profile
must not require 23 generated waypoints. Ground truth is allowed only for offline evaluation,
never robot control.

### 1. Health-check semantics

Preserve the current baseline behavior for the supplied 23-waypoint route.

Add a separate student/mission health-check profile that checks:

- mission complete;
- all four ordered goals reached;
- HOME within tolerance;
- no collision episodes;
- no permanent stall/blocked state.

Do **not** require exactly 23 active/generated waypoints in student mode.

### 2. Student-side diagnostics

Do not modify the public evaluation Supervisor simply to add custom columns.

Create a separate controller-side CSV/JSONL keyed by simulation time.

Use the exact common diagnostic field names defined below in this task:

```text
time
experiment_method
active_goal
route_index
route_count
route_version
replan_count
replan_reason
avoidance_state
temporary_obstacle_count
mcl_state
mcl_confidence
mcl_ess
particle_position_spread_m
particle_heading_spread_rad
```

### 3. Event logging

Use the exact event names defined below in this task:

```text
GOAL_REACHED
REPLAN_REQUEST
LOCALIZATION_LOST
LOCALIZATION_RECOVERED
NO_PATH
MISSION_COMPLETE
```

Put details such as the goal name or replan reason in separate fields rather than
creating new event names.

This makes trial failures much easier to inspect than only looking at continuous
numeric telemetry.

### 4. Reproducibility support

Before final experiments, support:

- fixed random seeds;
- scenario name;
- method name;
- trial number;
- configuration snapshot;
- trial manifest.

### 5. Analysis tooling

Update/create tooling to compute useful metrics without assuming a 23-waypoint
student route.

Useful outputs:

- success/failure;
- goal completion order;
- completion time;
- path length;
- collision episodes;
- localization RMSE when evaluation ground truth is available offline;
- kidnapped recovery time.

Ground truth may be used for **offline evaluation**, never robot control.

### Coordination with Person 1

Use the exact localization state names and diagnostic keys documented in T1/T5 and repeated
above in this task. Person 1 implements/exposes them; Person 2 must not rename them in tooling.

**Done when:** a failed run can be traced to localization, planning, avoidance,
or mission logic without modifying the evaluation Supervisor.

</details>

---

<details>
<summary><strong>T4 - Verify subsystem interfaces before controller integration [BOTH]</strong></summary>

### Goal

Do not design new interfaces here. Verify that the independently implemented T1/T2/T3/T5/T6/T7
code matches the fixed interfaces written in T1/T2/T3/T5/T6/T7 before wiring it into the controller.

### Fixed decisions for T4

T4 is a verification/merge task, not a redesign task. The interfaces to preserve are:

```text
localizer.update(...) -> Pose2D
localizer states -> LOCALIZING / TRACKING / LOST
plan_world_path(...) -> (route, status)
planner status -> SUCCESS / NO_PATH / INVALID_START / INVALID_GOAL
RouteManager owns mission/route/replan state
TemporaryObstacleLayer owns dynamic planning occupancy/TTL
SafetyLayer owns command filtering and replan request signaling
WaypointFollower interface and wheel conversion remain the existing project interfaces
```

If an independent branch deviates from these names or ownership boundaries, change that branch
to match the task-local contract rather than inventing a new integration API.

### Person 1 checks

- `StudentLocalizer.update(...)` still returns `Pose2D`.
- `initialize(..., initialization_mode=...)` accepts `nominal` and `global_free_space`.
- states are exactly `LOCALIZING`, `TRACKING`, `LOST`.
- confidence is clamped to `[0, 1]`.
- diagnostics use the agreed keys.
- no localization code imports route-manager or navigation internals.

### Person 2 checks

- planner status strings match the T2 list exactly.
- `plan_world_path(...)` returns world-coordinate route points.
- route manager owns goal/route/replan state.
- temporary obstacle layer owns dynamic occupancy/TTL.
- navigation owns wheel-command filtering only.
- no planning/navigation code reaches into MCL particle internals.

### Together

Run a small interface smoke test:

```text
Pose2D from localizer
  -> world start x/y
  -> plan_world_path
  -> route manager installs route
  -> current route target
  -> WaypointFollower command
  -> SafetyLayer filter
  -> wheel conversion
```

Also verify the dynamic-obstacle planning pipeline is exactly:

```text
static raw grid + temporary raw hit cells
-> combined raw occupancy
-> inflate once by 0.12 m
-> A*
```

Do not double-inflate the static map.

**Done when:** the interfaces match without redesign. Any mismatch is fixed to conform to the
task-local contracts, then Stage 3 integration begins.

</details>

---

<details>
<summary><strong>T5 - MCL runtime interface and recovery behavior [PERSON 1]</strong></summary>

Primary file:

```text
controllers/project_starter/localization.py
```

Do **not** edit `project_starter.py` during this independent stage. Final controller wiring is T8
after both branches merge.

### Goal

Turn the T1 particle filter into a controller-ready subsystem that follows the fixed localization
API in T1/T5 and can safely report low confidence/loss/recovery.

### Fixed decisions for T5

The runtime state rule is fixed initially:

```text
TRACKING:
    confidence >= 0.65
    AND position_spread_m <= 0.20 m
    AND heading_spread_rad <= 0.30 rad
    for exactly 3 consecutive updates

LOST:
    confidence < 0.25 for 10 consecutive updates
    OR position_spread_m > 0.75 m

otherwise:
    LOCALIZING
```

On the first update where recovery changes LOST/LOCALIZING back to TRACKING, set
`just_recovered = True` for that update only. The localizer reports state/confidence; it does not
decide wheel commands or mission progress.

### 1. Preserve baseline-compatible return type

`update(...)` continues to return `Pose2D`. Implement the agreed getters/properties for state,
confidence, and diagnostics.

### 2. State behavior

Implement exactly:

```text
LOCALIZING -> initial/broad belief, do not assume route is safe yet
TRACKING   -> belief meets tracking thresholds
LOST       -> belief is inconsistent/collapsed and needs recovery
```

Require exactly **3 consecutive good updates** before changing from LOCALIZING/LOST to TRACKING,
using the fixed T5 state rule below. This prevents state chatter while keeping the implementation
deterministic.

### 3. Recovery event

When LOST recovery converges back to TRACKING:

- set `just_recovered = True` for one update;
- emit/allow the controller to log `LOCALIZATION_RECOVERED`;
- T8 will convert this into `REPLAN_REQUEST = LOCALIZATION_RECOVERED`.

### 4. Low-confidence contract

Person 1 does not decide wheel behavior. The localizer only reports state/confidence. In T8 the
shared controller will stop route following whenever the state is not TRACKING.

### 5. Tests

Add tests for:

- state transitions;
- confidence clamping;
- 3-update tracking debounce;
- LOST recovery and one-shot `just_recovered`;
- diagnostics dictionary keys/types;
- both initialization modes.

### Stage-2 deliverable

Person 1 commits a localization subsystem that can be imported and exercised without Webots
ground truth and without any dependency on route-manager/navigation internals.

**Done when:** the agreed MCL API is stable and its tests pass.

</details>

---

<details>
<summary><strong>T6 - Dynamic A* route manager [PERSON 2]</strong></summary>

Primary new file:

```text
controllers/project_starter/route_manager.py
```

Do **not** edit `project_starter.py` during this independent stage. Final wiring is T8.

### Goal

Replace the fixed student route with per-goal A* routes while keeping the
provided baseline route intact for baseline experiments.

### Fixed decisions for T6

`route_manager.py` owns mission and generated-route state only. It does not own localization or
wheel control. Use only these states:

```text
LOCALIZING
PLAN
FOLLOW
REPLAN
NO_PATH
RECOVERING
COMPLETE
```

Use these replan reasons exactly:

```text
NONE
GOAL_CHANGED
BLOCKED
LOCALIZATION_RECOVERED
PATH_INVALID
```

Tolerances:

```text
generated route-point tolerance = controller.waypoint_tolerance_m = 0.075 m
mission goal tolerance = evaluation.position_goal_tolerance_m = 0.20 m
```

Required public data:

```text
active_goal_index
route
route_index
route_version
state
replan_reason
```

Required interface:

```python
manager = RouteManager(goals, route_point_tolerance=0.075, goal_tolerance=0.20)
manager.current_goal()
manager.current_target()
manager.install_route(route)
manager.advance_route(pose)
manager.advance_goal(pose)
manager.request_replan(reason)
manager.needs_plan()
manager.mark_no_path()
```

`install_route()` increments `route_version` and drops the first route point only when it is
already within `0.075 m`. It must never drop the final mission goal. Mission completion is always
A -> B -> C -> HOME based on distance to the actual goal coordinates, not a route index.

### Mission logic

For each goal:

1. read current estimated pose;
2. plan to active mission goal;
3. store the returned route;
4. follow route waypoints in order;
5. detect goal arrival within tolerance;
6. advance A -> B -> C -> HOME;
7. stop only after HOME.

### Route-manager state machine

Use only these fixed route-manager states:

```text
LOCALIZING
    |
    v
  PLAN -------> NO_PATH
    |
    v
 FOLLOW ------> REPLAN
    |             |
    |             v
    +----------> PLAN
    |
    | actual mission goal reached
    v
advance_goal()
    |
    +--> next goal -> PLAN
    |
    +--> after HOME -> COMPLETE

RECOVERING -> REPLAN
```

`GOAL_REACHED` is an **event**, not a route-manager state. Log the event, advance
the active mission goal, then transition to `PLAN` or `COMPLETE`.

### Replan triggers

Replan only when necessary:

- mission goal changes;
- temporary blocker invalidates route;
- localization recovers after a major correction;
- route becomes invalid;
- planner explicitly receives a new planning grid version.

Avoid replanning every control step.

### Route progress

Track at least:

```text
route_version
route_index
route_count
active_goal
```

When a new route is installed:

- reset route index safely;
- do not accidentally mark skipped route points as mission goals;
- verify the first useful route point is not behind the robot due to
  simplification/start-cell handling.

### Goal completion

Goal completion should be based on the actual mission goal coordinate and
configured tolerance, not on reaching a magic final route index.

### NO_PATH handling

If A* reports no path:

- stop safely;
- remain in a recoverable state;
- allow a later map/temporary-obstacle change to trigger another attempt;
- log the failure clearly.

Do not spin or continue driving without a valid route.

**Done when:** nominal A -> B -> C -> HOME completes using generated A* routes
with no dependency on 23 supplied waypoints.

</details>

---

<details>
<summary><strong>T7 - Obstacle avoidance and blocked-route replanning [PERSON 2]</strong></summary>

Primary files:

```text
controllers/project_starter/navigation.py
controllers/project_starter/temporary_obstacles.py   # new
```

Do **not** edit `project_starter.py` during this independent stage. Final wiring is T8.

### Goal

Handle nearby obstacles safely and make persistent unmapped blockers trigger a
new global route instead of a permanent stop.

### Fixed decisions for T7

Keep the existing `WaypointFollower` and wheel-conversion path. Extend the existing `SafetyLayer`
inside `navigation.py`; do not create a second wheel-command stack. Keep this signature:

```python
safety.filter_command(...) -> (linear_speed, angular_speed, minimum_front)
```

Add these read-only outputs and reset call:

```text
safety.replan_requested
safety.replan_reason   # NONE or BLOCKED
safety.state
safety.clear_replan_request()
```

Starting avoidance/progress thresholds:

```text
front range < 0.10 m -> emergency safety behavior
front range < 0.23 m -> local slowdown/avoidance zone
persistent forward blockage ~1.0 s -> candidate BLOCKED
no meaningful route progress for 3.0 s -> candidate BLOCKED
meaningful progress = at least 0.05 m reduction in distance-to-target
replan cooldown = 1.0 s
```

The temporary-obstacle layer is planning evidence only; it never edits the static map. Use this
flow exactly:

```text
raw static grid + confirmed temporary hit cells
-> combined raw occupancy
-> inflate combined grid once by 0.12 m
-> A*
```

Temporary-obstacle starting rules:

```text
only add world-frame observations while localization state == TRACKING
ignore a hit within 0.15 m of a known static occupied cell
require 3 confirming observations in the same/neighboring grid cell
temporary obstacle TTL = 5.0 s
never clear or overwrite a static occupied cell
```

Required interface:

```python
layer = TemporaryObstacleLayer(confirmation_count=3, ttl_s=5.0)
changed = layer.observe(scan_points_robot, pose, static_grid, time_s)
planning_grid = layer.build_planning_grid(static_grid, inflation_radius_m=0.12, time_s=time_s)
count = layer.active_count(time_s)
version = layer.version
```

Increment `version` only when confirmed active obstacle cells are added or expire. Request a
`BLOCKED` replan only when localization is TRACKING and the temporary obstacle is confirmed.

### 1. Preserve emergency safety

Keep the existing emergency stop/recovery behavior as a last-resort safety
fallback if useful.

The new behavior should sit above that rather than removing all safeguards.

### 2. Local obstacle behavior

Use LiDAR sectors or equivalent local geometry to:

- slow near obstacles;
- steer away when there is room;
- keep wheel commands bounded;
- avoid rapid left/right oscillation;
- maintain forward progress when safe.

### 3. Progress monitoring

Detect when the robot is not making meaningful progress toward the active route
point.

Useful signals:

- distance-to-waypoint not decreasing;
- repeated emergency stops;
- obstacle continuously occupying forward corridor;
- wheel motion but little positional progress.

### 4. Distinguish transient vs persistent obstacle

Do not replan for every short LiDAR observation.

Possible logic:

```text
short observation -> local avoidance only
persistent confirmed blocker -> update temporary obstacle layer + replan
```

### 5. Persistent blockage flow

1. stop/slow safely;
2. confirm blocker persistence;
3. require adequate localization confidence;
4. mark temporary obstacle cells;
5. increment planning-grid version;
6. issue `REPLAN_REQUEST = BLOCKED`;
7. run A* on combined static + temporary occupancy;
8. install new route;
9. continue toward the **same mission goal**.

### 6. Avoid false permanent obstacles

Temporary observations should expire or be cleared under a justified rule.

Example:

- TTL in simulation time;
- repeated free-space evidence;
- route has moved well beyond the obstacle;
- explicit scenario reset.

### 7. Required test

Move the red blocker onto the originally planned path after the route has been
created.

Success means:

```text
robot detects blocker
-> safely stops/avoids
-> planning grid changes
-> A* produces different route
-> robot continues mission
```

Simply stopping in front of the blocker is a failure.

**Done when:** the blocked-route scenario completes reliably using a genuine
alternate global route.

</details>

---

<details>
<summary><strong>T8 - Full integration [BOTH]</strong></summary>

### Goal

Start from the passing Stage-2 merge and wire the already-agreed subsystem APIs into
`project_starter.py` and `project_config.json`. This is the first stage where the final combined
controller loop is edited.

### Fixed decisions for T8

Add/use these exact configuration names during integration:

```text
experiment.method: odometry_baseline | mcl_only | full_student_method
localization.mode: odometry | mcl
controller.planning_mode: provided_waypoints | astar
controller.avoidance_mode: starter_safety | student_avoidance
mapping.mode: off | occupancy_mapping
```

Mode selection is fixed to:

| `experiment.method` | localization | planning | avoidance | mapping |
|---|---|---|---|---|
| `odometry_baseline` | `odometry` | `provided_waypoints` | `starter_safety` | `off` |
| `mcl_only` | `mcl` | `provided_waypoints` | `starter_safety` | `off` |
| `full_student_method` | `mcl` | `astar` | `student_avoidance` | `off` |

The public Supervisor telemetry field `mode` should contain `experiment.method`.

`odometry_baseline` must remain the original comparison behavior: odometry localization, the
provided 23-waypoint route, starter safety, and no student A*/temporary-obstacle path.
`mcl_only` is a debugging/ablation mode, not a replacement for the required comparison.

### Fixed integration-loop order

Use this exact structure unless a concrete Webots/device constraint requires otherwise. The
baseline and student planning paths must remain separate so the baseline stays reproducible.

```text
1. read encoders + LiDAR
2. update odometry and obtain delta_motion

3. localization
   if localization.mode == "mcl":
       update MCL
       estimate = MCL estimate
       localization_state = MCL state
   else:
       estimate = odometry pose
       localization_state = TRACKING

4. planning / route selection
   if controller.planning_mode == "provided_waypoints":
       use the original supplied 23-waypoint progression
       do NOT run RouteManager
       do NOT run A*
       do NOT add temporary-obstacle planning occupancy

   elif controller.planning_mode == "astar":
       if localization_state == TRACKING:
           update temporary-obstacle evidence from LiDAR
           update RouteManager state
           if state is PLAN or REPLAN:
               build combined + inflated planning grid
               run A*
           obtain current generated route target
       else:
           hold route progress and command stop/recovery behavior

5. WaypointFollower computes the nominal unicycle command for the selected target

6. avoidance
   if controller.avoidance_mode == "starter_safety":
       use the original SafetyLayer behavior
   else:
       use student avoidance / persistent-blockage replan behavior

7. convert to wheel speeds and command motors
8. update display
9. emit public Supervisor telemetry
10. write student diagnostic log
```

When MCL is `LOCALIZING` or `LOST`, do not advance mission goals. Default to zero
forward motion; controlled sensing turns may be added only if scenario testing shows
they are needed.

### Integration ownership

During each gate, choose one person to drive/edit `project_starter.py` while the
other observes logs and diagnoses their subsystem.

Suggested rotation:

- localization-heavy gate -> Person 1 edits;
- planning/navigation-heavy gate -> Person 2 edits;
- shared state-machine changes -> pair together.

### Gate 1 - MCL + supplied route

Already started in T5.

Verify:

- MCL estimate is stable;
- baseline route follower works with MCL pose;
- odometry baseline still works.

### Gate 2 - MCL + A* nominal

Enable A* but no special blocker behavior yet.

Verify:

- start pose converts correctly to grid;
- each mission goal plans successfully;
- route waypoints are reasonable;
- A -> B -> C -> HOME completes.

### Gate 3 - Dynamic route manager

Verify:

- route changes on new mission goal;
- route index resets correctly;
- no 23-waypoint dependency;
- HOME ends the mission cleanly.

### Gate 4 - Local avoidance

Verify:

- nearby obstacles alter wheel commands;
- commands stay finite and bounded;
- no severe left/right oscillation;
- normal route following resumes after obstacle clears.

### Gate 5 - Temporary-obstacle replanning

Verify:

- unexpected blocker is converted into world/grid cells only with adequate
  localization confidence;
- static map is not modified destructively;
- combined planning grid changes;
- A* route changes;
- mission continues.

### Gate 6 - Uncertain start

Verify:

- broad initial belief is used;
- robot does not drive aggressively before localization is trustworthy;
- MCL converges;
- plan begins from corrected pose;
- mission completes.

### Gate 7 - Kidnapped recovery

Press `K` during navigation.

Verify:

- localization mismatch is detected;
- old route is not followed blindly;
- MCL broadens/reseeds and recovers;
- route is replanned from recovered pose;
- mission resumes.

### At every gate

Run relevant unit tests and save at least one diagnostic log.

When something fails, classify it first:

```text
localization
coordinate transform
planning
route progress
local avoidance
temporary obstacle logic
mission state
```

Avoid changing multiple subsystems at once unless the failure clearly crosses
an interface.

**Done when:** every gate works independently and the complete required system
is repeatable.

</details>

---

<details>
<summary><strong>T9 - Validate the four required scenarios [BOTH]</strong></summary>

### Goal

Turn development success into repeatable scenario success before collecting the
final experiment set.

### Fixed decisions for T9

Validate both required experiment methods against the same scenario definitions. For development
checks, prioritize `full_student_method`; before T10, confirm the scenario setup is also usable with
`odometry_baseline`. Do not change goal coordinates, map geometry, or success tolerances between
methods. Mission order remains A -> B -> C -> HOME.

For uncertain-start tests, use the `global_free_space` MCL initialization mode. For kidnapped
tests, use the scenario's `K` event and never use its ground-truth pose inside controller logic.
For blocked-route tests, the red blocker remains absent from the static map and must be handled
through the T7 temporary-obstacle layer.

### Scenario 1 - Nominal

Configuration:

- normal start belief;
- default blocker position;
- full student method.

Expected:

```text
localize
plan
A -> B -> C -> HOME
no collision
```

### Scenario 2 - Uncertain start

Use a broader MCL prior.

Expected:

- remain safe while uncertain;
- converge sufficiently;
- generate route from estimated pose;
- complete mission.

Person 1 leads diagnosing localization convergence.

### Scenario 3 - Blocked route

Move the red blocker so it blocks the original planned route.

Expected:

- detect unexpected obstacle;
- confirm persistence;
- add temporary occupancy;
- replan;
- follow alternate path;
- complete mission.

Person 2 leads diagnosing route/avoidance behavior.

### Scenario 4 - Kidnapped robot

Press `K` during navigation.

Expected:

- localization becomes inconsistent;
- MCL declares/reaches recovery state;
- stale route is abandoned;
- pose reconverges;
- A* replans;
- mission resumes.

Person 1 leads localization diagnosis; Person 2 verifies route reset/replan.

### Repeatability check

Before official data collection, obtain more than one successful run for each
scenario if time allows.

Track known failure modes such as:

- MCL convergence too slow;
- false obstacle insertion;
- route simplification cutting corners;
- replan oscillation;
- no-path state never retrying;
- goal reached too early/late;
- kidnapped recovery keeps old route.

**Done when:** every required scenario has at least one repeatable success and
known failure modes are understood.

</details>

---

<details>
<summary><strong>T10 - Required quantitative experiments [BOTH]</strong></summary>

### Goal

Collect the required comparison dataset **before** starting the optional mapping
extension.

### Fixed decisions for T10

The required comparison is exactly:

```text
odometry_baseline vs full_student_method
4 scenarios x 2 methods x 5 trials = 40 minimum trials
```

Keep failed trials. `mcl_only` may be used for debugging/ablation but does not replace either
required method. Each trial must preserve both the public evaluation CSV and the student-side
diagnostics/event log from T3, together with method/scenario/trial/seed/config/commit metadata.
Do not redefine metrics after data collection starts.

### Methods

#### Method 1 - `odometry_baseline`

- encoder odometry;
- provided waypoint route;
- starter navigation/safety.

#### Method 2 - `full_student_method`

- MCL;
- A*;
- dynamic route manager;
- student avoidance;
- temporary-obstacle replanning.

`mcl_only` can be used for debugging/ablation but does not replace the required
full student-method comparison.

### Minimum trial matrix

```text
4 scenarios x 2 methods x 5 trials = 40 minimum trials
```

Keep failed trials.

### Fixed metric definitions

Use these definitions for the entire final dataset. Do not redefine them between methods or
scenarios.

```text
mission success:
    goals_reached_ground_truth == 4 AND mission_complete_reported == 1

goal tolerance:
    0.20 m, matching evaluation.position_goal_tolerance_m

completion time:
    first Supervisor time_s where goals_reached_ground_truth == 4
    minus the first logged time_s of that trial

position RMSE:
    sqrt(mean(position_error_m^2)) across all valid logged samples in the trial

heading RMSE:
    sqrt(mean(heading_error_rad^2)) across all valid logged samples; heading_error_rad is
    already wrapped by the Supervisor

ground-truth path length:
    final ground_truth_path_length_m value from the Supervisor log

collision episodes:
    final collision_episodes value from the Supervisor log

kidnap event time:
    simulation time recorded in the student/scenario log when K is applied

kidnapped recovery time:
    first LOCALIZATION_RECOVERED transition after the kidnap event minus kidnap event time;
    the localizer's 3-good-update TRACKING debounce is already part of this definition

failed/non-completing trial:
    keep the trial; success = 0; completion time is reported as NA rather than silently
    deleting the run
```

Collision count is reported separately from mission success so a run that reaches all four goals
but makes contact remains visible as a completed run with collision(s), not discarded data.

### Trial responsibilities

Recommended split:

**Person 1**

- uncertain-start trials;
- kidnapped trials;
- localization RMSE/recovery review.

**Person 2**

- nominal trials;
- blocked-route trials;
- route/collision/completion review.

Both should cross-check a sample of the other person's trials.

### For every trial save

```text
method
scenario
trial number
seed
scenario settings
config snapshot/commit
evaluation CSV
student diagnostics
success/failure
notes if failed
```

### Metrics

At minimum compute:

- position RMSE;
- heading RMSE;
- mission success rate;
- ordered goals reached;
- completion time;
- ground-truth path length;
- collision episodes;
- kidnapped recovery time.

### Aggregate results

Generate per-method/per-scenario:

- mean;
- standard deviation;
- success rate;
- counts of failure modes.

Do not report only successful trials.

### Freeze required results

After the required dataset is complete:

- preserve all final CSVs in a tracked submission location;
- intentionally adjust `.gitignore` only if needed;
- generate final aggregate tables/plots;
- record exact config and commit hash;
- avoid changing the required system unless fixing a demonstrated bug.

If a bug fix changes behavior after data collection, decide whether affected
trials need to be rerun.

**Done when:** the required comparison dataset is complete, reproducible, and
frozen before extension work starts.

</details>

---

<details>
<summary><strong>T11 - Occupancy-grid mapping extension [PERSON 1 lead, PERSON 2 support]</strong></summary>

### Scope decision

The proposal selected full SLAM, but the assignment accepts occupancy-grid
mapping **or** full SLAM.

### Fixed decisions for T11

Implement the lower-risk extension as `mapping.mode = occupancy_mapping` using the MCL pose.
`mapping.mode = off` must continue running the frozen required system unchanged. Static-map/MCL
localization and learned-map updates remain separate unless you intentionally implement a true
SLAM feedback loop. Never use Supervisor ground truth to update the learned map.

Recommended lower-risk extension:

```text
occupancy-grid mapping using the MCL pose
```

If the MCL estimate still depends on the supplied static map and the learned map
does not feed back into localization, describe it as **occupancy-grid mapping**,
not full SLAM.

### Person 1 - mapping core

Primary file:

```text
controllers/project_starter/mapping.py
```

Implement:

- occupancy/log-odds representation;
- world/grid conversions;
- ray tracing from robot to LiDAR endpoint;
- free-space evidence along the ray;
- occupied evidence at valid hit endpoint;
- probability/log-odds clamping;
- invalid/max-range handling;
- map reset/save if useful.

Never use Supervisor ground truth to update the learned map.

### Person 2 - support / visualization / evaluation

Help with:

- displaying learned occupancy;
- saving map snapshots;
- comparing learned map to expected geometry offline;
- experiment/plot scripts;
- making sure mapping does not interfere with required navigation mode.

### Integration

Keep mapping selectable:

```text
mapping = off
mapping = occupancy_mapping
```

The required frozen system should still run without the extension.

### Optional evaluation

Possible extension metrics:

- occupancy accuracy against known map offline;
- qualitative learned-map figure;
- runtime cost;
- navigation impact if intentionally tested.

**Done when:** the extension is technically correct, clearly labeled, and does
not break or overwrite the frozen required system.

</details>

---

<details>
<summary><strong>T12 - Final analysis, report, demo, and submission [BOTH]</strong></summary>

### Fixed decisions for T12

Do not silently alter the frozen required method after T10. If a behavior-changing bug fix is
necessary, record the commit and rerun any affected trials. Final submission must still reproduce
`odometry_baseline` and `full_student_method` from documented configuration, and all reported
plots/tables must be regenerable from included data/scripts.

Keep the public evaluation Supervisor and `reference/` baseline artifacts unchanged.

### 1. Final analysis

Split initial analysis work:

**Person 1**

- localization RMSE plots;
- heading RMSE;
- uncertain-start analysis;
- kidnapped recovery-time analysis;
- mapping-extension results.

**Person 2**

- success rate;
- completion time;
- path length;
- collision episodes;
- blocked-route/replanning analysis;
- representative route figures.

Then review all plots/tables together.

### 2. Report structure

IEEE-style report should include:

1. abstract;
2. introduction/problem definition;
3. related work with at least three archival sources outside class readings;
4. system architecture and coordinate conventions;
5. localization method and equations;
6. planning/navigation/avoidance methods;
7. experimental design and metric definitions;
8. quantitative results;
9. failure analysis;
10. extension;
11. conclusion;
12. individual contributions;
13. IEEE references.

### Suggested writing split

**Person 1 drafts:**

- localization/MCL section;
- uncertain-start and kidnapped results;
- mapping extension;
- localization-related failure analysis.

**Person 2 drafts:**

- A* and route manager;
- avoidance/replanning;
- experiment tooling/methodology;
- blocked-route results;
- planning/navigation failures.

**Both write/review:**

- abstract;
- introduction;
- architecture;
- overall results discussion;
- conclusion;
- contributions.

One person should do the final formatting pass for consistency.

### 3. Presentation/demo

- Presentation: December 1 or 3, 2026.
- 12 minutes total including about 3 minutes for questions.
- Final submission also includes a two-minute demo video or unlisted link.

Suggested presentation split:

**Person 1**

- problem/localization;
- uncertain start;
- kidnapped recovery;
- mapping extension.

**Person 2**

- planning/navigation;
- blocked-route behavior;
- experiment comparison;
- demo flow.

Both should be able to explain the full architecture during questions.

### 4. Final reproducibility check

Run:

```bash
python3 tools/validate_project.py
python3 -m unittest discover -s tests -v
python3 tools/simulate_baseline.py
```

Also manually verify:

- `odometry_baseline` still runs;
- `full_student_method` still runs;
- each required scenario can be reproduced from documented instructions;
- final configs/seeds are included;
- all experiment CSVs are included;
- analysis scripts regenerate report plots/tables;
- no report result depends on a missing local file.

### 5. Final package

Include:

- source code;
- configuration;
- run instructions;
- assignment/readme docs;
- all required experiment CSVs;
- student diagnostics used in analysis;
- seeds/scenario settings;
- analysis scripts;
- report;
- demo/video link;
- contribution statement.

**Done when:** another person can clone/open the submission, follow the
instructions, run both methods, reproduce the required scenarios, and regenerate
the reported quantitative results.

</details>

---

## File ownership during implementation

This repeats the ownership decisions from T0 for quick reference and merge-conflict resolution.

| File / area | Default owner |
|---|---|
| `localization.py` | Person 1 |
| `mapping.py` | Person 1 |
| MCL/localization tests | Person 1 |
| `planning.py` | Person 2 |
| `navigation.py` | Person 2 |
| `route_manager.py` | Person 2 |
| `temporary_obstacles.py` | Person 2 |
| planner/navigation tests | Person 2 |
| `tools/check_run_health.py` | Person 2 |
| `tools/analyze_run.py` | Person 2 |
| `project_starter.py` | Integration stage only; both review, one editor at a time |
| `project_config.json` | Integration stage only; use the exact T8 mode keys/table |
| experiment definitions | Both |
| final plots/report | Both |

---

## Leave these unchanged until needed

- `config/project_config.json` - update only when MCL/mode integration requires
  new settings.
- `controllers/project_starter/*.py` - modify only in the corresponding task.
- `tests/test_scaffold.py` - expand alongside implementation tasks.
- `tools/check_run_health.py` and `tools/analyze_run.py` - update in T3.
- `docs/astar_pseudocode.md` - update after T2 implementation to match the exact T2 A* rules.
- `results/README.md` and `.gitignore` - update when final experiment storage is
  chosen in T10.
- `reference/` baseline artifacts - preserve unchanged.
- `controllers/evaluation_supervisor/` - do not modify for evaluated runs.

---

## Immediate start instructions

No additional design meeting is required. Start with Stage 0, then split immediately.

### Stage 0 - both

```text
run validator + unit tests + deterministic baseline
verify the fixed T0 coordinate/config assumptions
create/sync the three working branches
commit clean starting point
```

### Stage 1 - split

**Person 1:** T1 MCL core.

Implementation order:

```text
particle initialization
-> motion update
-> likelihood-field measurement update
-> normalization + ESS
-> resampling
-> weighted/circular pose estimate
-> confidence metrics
-> kidnapped recovery core
-> localization tests
```

**Person 2:** T2 A* plus T3 tooling.

Implementation order:

```text
world/grid conversion tests
-> A*
-> corner-cut/inflation safety
-> line-of-sight simplification
-> world-level planner helper/status
-> planning tests
-> mission-vs-baseline health-check split
-> diagnostics/logging scaffolding
```

### Gate 1

Merge only when:

```text
P1: MCL unit tests pass and a Pose2D estimate is produced
P2: A* unit tests pass and safe world routes are produced
P2: tooling no longer assumes 23 waypoints for student mode
full existing test suite still passes after merge
```

### Stage 2 - split again from the passing Gate-1 merge

**Person 1:** T5 MCL runtime states/recovery API.

**Person 2:** T6 route manager + T7 student avoidance/temporary obstacles.

Neither person edits `project_starter.py` in Stage 2.

### Gate 2

Merge only when all subsystem tests pass and T4's interface smoke test passes.

### Stage 3 and later

T8 is the first full shared integration. After that, follow the stage diagram at the top: test the
required scenarios, merge/fix, run the required experiments, freeze them, then do the optional
extension and final deliverables.
