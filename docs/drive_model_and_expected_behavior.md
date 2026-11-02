# Drive Model and Expected Starter Behavior

## Release 4 parameter set

| Quantity | Symbol | Value | Use |
|---|---:|---:|---|
| Wheel radius | `r` | `0.0200 m` | Encoder distance and inverse kinematics |
| Geometric wheel-center spacing | `b_geom` | `0.0520 m` | Physical model documentation |
| Effective track width | `b_eff` | `0.0574 m` | Encoder heading and wheel commands |
| Collision-body radius | `R` | `0.037 m` | Clearance checks |
| Motor command limit |  | `6.28 rad/s` | Conservative controller saturation |
| Cruise speed |  | `0.09 m/s` | Nominal straight motion |
| Motor acceleration |  | `3.0 rad/s²` | Smooth command changes |

The official geometry and the effective track width are intentionally separate.
The physical wheel centers are 0.0520 m apart, while the larger effective value
matches the yaw produced by the dynamic wheel/contact model. Using `b_geom` in
the encoder equations makes the odometry turn too far and was a major source of
the earlier upper-wall failure.

## Differential-drive equations

For encoder changes `ΔφL` and `ΔφR`:

```text
ΔsL = r ΔφL
ΔsR = r ΔφR
Δs  = (ΔsR + ΔsL) / 2
Δθ  = (ΔsR - ΔsL) / b_eff
```

The odometry update uses the midpoint heading:

```text
x ← x + Δs cos(θ + Δθ/2)
y ← y + Δs sin(θ + Δθ/2)
θ ← wrap(θ + Δθ)
```

For a desired body command `(v, ω)`, inverse kinematics use the same `r` and
`b_eff`:

```text
ωL = (v - ω b_eff/2) / r
ωR = (v + ω b_eff/2) / r
```

This symmetry matters: changing the track width in odometry without changing
wheel-command conversion creates a second inconsistency.

## Sensor and frame assumptions

- The world is ENU with the floor in global `x-y` and `z` up.
- Project coordinates equal Webots global `x-y` coordinates.
- Robot and LiDAR use FLU: `+x` forward, `+y` left, `+z` up.
- LiDAR point-cloud `x` and `y` are used directly; there is no axis swap or sign
  change.
- The starter reads only wheel encoders and LiDAR for navigation. Ground truth
  is confined to the evaluation Supervisor.

## Waypoint and safety behavior

The route has 23 waypoints. A waypoint is accepted only when the encoder pose
is within 0.075 m. The follower slows inside 0.35 m and rotates in place when
the heading error is large. The central detours at `y = +0.43` and `y = -0.43`
give the 0.037 m body at least 0.084 m nominal clearance from walls/blocker in
the deterministic check.

The safety state machine is a last-resort layer:

1. enter a latched turn when a forward return is closer than 0.10 m;
2. keep one turn direction for at least 20 controller steps;
3. require four clear scans above 0.18 m;
4. move forward briefly to exit the local geometry;
5. reset recovery bookkeeping when a waypoint advances; and
6. stop as `blocked` after two recoveries at one waypoint instead of oscillating
   forever.

Students must replace or augment this with a real obstacle-avoidance/replanning
method. The starter is guaranteed only for the untouched default blocker.

## Verification evidence included in the package

- `tools/validate_project.py`: world/device/map/parameter consistency and route
  body-clearance sampling
- `tests/test_scaffold.py`: odometry, frames, map, follower, safety, wheel
  saturation, health checks, and full nominal mission regression
- `tools/simulate_baseline.py`: deterministic controller-in-loop mission
- `reference/expected_baseline_trace.csv`: downsampled canonical trace
- `reference/expected_baseline_trace_summary.json`: expected totals
- `reference/expected_baseline_route.png`: route and expected trajectory
- `tools/calibrate_drive.py`: instructor-side estimate of wheel radius and
  effective track width from raw encoders and logged ground truth

The simulator is a preflight check. Final release acceptance still requires one
complete Webots R2025a run and a `PASS` from the full-mission health checker.
