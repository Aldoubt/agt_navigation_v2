# V25-11E Safety Response and Simulation Validation Record

Date: 2026-08-11

Status: **FINAL PASS**

Validation mode: **SOFTWARE_ONLY Gazebo simulation**

Final branch: `feat/v25-11e-safety-abort-latency`

## 1. Purpose

This document freezes the final V25-11E simulation-validation result and the V25-11E-2 active-fault baseline collected before the V25-11E-3 route-abort optimization

The completed validation scope covers the software-only navigation execution chain, canonical localization guard, sensor-health supervision, tracked safety output, deterministic fault injection, route termination, observability, machine-readable acceptance reports, and response-latency gating

The result proves that the V25-11C through V25-11E validation workflow is closed in Gazebo

It does **not** claim that the complete real-robot system has been validated

## 2. Final validation conclusion

The following gates passed in the final clean runtime sequence

| Stage | Purpose | Final status |
|---|---|---|
| V25-11C | Static map + Route + Nav2 planner/controller + Gazebo physical-motion happy path | PASS |
| V25-11D | Deterministic six-case failure matrix and fail-closed behavior | PASS |
| V25-11E-1 | Unified observability baseline and runtime evidence | PASS |
| V25-11E-2 | Active-fault response measurement and comparison harness | PASS |
| V25-11E-3 | Safety-aware fast Route abort and tightened response gate | PASS |

The final V25-11E matrix uses schema:

```text
agt_v25_11e_comparison_matrix/v2
```

and stage:

```text
safety_response_latency_gate
```

The final runtime matrix reported `PASS`, including all of the following checks:

```text
all_fault_reports_present          true
all_fault_reports_passed           true
all_safety_zero_bounded            true
all_route_terminal_bounded         true
all_post_fault_distance_bounded    true
```

## 3. Validated end-to-end simulation chain

The final software-only runtime path is:

```text
Gazebo BUNKER physics
        ↓
Gazebo sensor / odometry adapters
        ↓
Canonical localization evidence
        ↓
Static validation map + Route
        ↓
Nav2 Planner
        ↓
Nav2 Controller
        ↓
/agt/navigation/cmd_vel
        ↓
agt_safety tracked controller
        ↓
/agt/safety/cmd_vel
        ↓
Gazebo BUNKER physical motion
```

The health and failure path is:

```text
LiDAR / IMU simulation stream
        ↓
agt_sensor_monitor
        ↓
/diagnostics
        ↓
agt_safety
        ↓
sensor_input_unhealthy
        ├──> /agt/safety/cmd_vel = 0
        └──> RouteRunner Safety guard
                    ↓
              cancel FollowPath
                    ↓
                Route FAILED
```

The localization failure path is:

```text
Canonical localization LOST
        ↓
RouteRunner continuous localization guard
        ↓
cancel active Nav2 goal
        ↓
Route FAILED
```

Therefore the simulation no longer validates only whether Nav2 can produce a path

It validates whether the navigation system can execute motion, detect critical failures, stop physical output, terminate the active route, and generate reproducible evidence for the complete event sequence

## 4. V25-11E-2 pre-optimization baseline

The first passing active-fault comparison matrix was collected before the V25-11E-3 RouteRunner Safety guard optimization

| Fault case | Fault to Safety zero | Fault to Route terminal | Fault speed | Max post-fault speed | Max post-fault distance | Route completion |
|---|---:|---:|---:|---:|---:|---:|
| `localization_lost` | 46 ms | 3 ms | 0.4011 m/s | 0.4078 m/s | 0.2537 m | 0.20 |
| `lidar_dropout` | 398 ms | 15307 ms | 0.4128 m/s | 0.4151 m/s | 0.4083 m | 0.20 |
| `imu_dropout` | 199 ms | 15187 ms | 0.2280 m/s | 0.2777 m/s | 0.2230 m | 0.20 |

This baseline demonstrated that physical fail-closed behavior was already working before V25-11E-3

The remaining issue was the approximately 15 s delay between sensor-health failure and the Route terminal state

## 5. Baseline interpretation

`localization_lost` already propagated through the continuous localization guard and terminated the Route almost immediately

`lidar_dropout` and `imu_dropout` already stopped the physical command through `agt_safety` within hundreds of milliseconds, but the Route runner did not consume the Safety fault state while waiting for `FollowPath`

The approximately 15 s Route delay was therefore a control-state convergence problem rather than a vehicle-stop problem

Changing the Nav2 progress-checker timeout was intentionally rejected as the primary fix because it would alter normal navigation behavior and slow or temporarily obstructed execution cases

## 6. V25-11E-3 design

The Route runner subscribes to:

```text
/agt/safety/status
```

Only the explicit tracked-safety reason:

```text
sensor_input_unhealthy
```

is promoted to a Route execution guard

Startup and transient reasons such as `input_timeout` and `motion_disabled` do not fail the Route

When `sensor_input_unhealthy` is observed while a controller goal is active, the Route runner requests cancellation of the current `FollowPath` goal and terminates the Route with:

```text
CONTROLLER_SAFETY_GUARD_FAILED: sensor_input_unhealthy
```

This preserves the accepted V25-11D `CONTROLLER_*` failure-class contract for sensor-dropout cases

The final event propagation is therefore:

```text
Sensor dropout
    ↓
SensorMonitor detects unhealthy stream
    ↓
Safety enters sensor_input_unhealthy
    ↓
Safety output becomes zero
    ↓
RouteRunner observes Safety authority
    ↓
FollowPath cancellation requested
    ↓
Route reaches FAILED terminal state
```

## 7. Final tightened response gate

The first tightened response gate intentionally keeps margin around the single-run baseline rather than overfitting exact milliseconds

| Metric | Final V25-11E-3 gate |
|---|---:|
| Fault to Safety zero | <= 750 ms |
| Fault to Route terminal | <= 1000 ms |
| Max post-fault distance | <= 0.75 m |

The final runtime matrix passed all three response bounds for all active-fault cases

Thresholds must not be loosened merely to obtain PASS

Future repeated experiments may tighten these thresholds after a statistically meaningful sample set is collected

## 8. Failure scenarios validated

V25-11D validated the complete deterministic failure matrix:

1. `map_identity_mismatch`
2. `localization_lost`
3. `planner_invalid`
4. `controller_invalid`
5. `lidar_dropout`
6. `imu_dropout`

V25-11E performs quantitative active-fault comparison for:

1. `localization_lost`
2. `lidar_dropout`
3. `imu_dropout`

These active cases have measurable fault timestamps, Safety response timestamps, Route terminal timestamps, post-fault command behavior, ground-truth motion, post-fault displacement, and Route completion ratio

## 9. Observability evidence

V25-11E-1 provides a unified runtime evidence layer over:

```text
/agt/localization/status
/agt/simulation/navigation/route_state
/diagnostics
/agt/safety/status
/agt/safety/cmd_vel
/simulation/bunker/ground_truth
```

Visual outputs include:

```text
/agt/validation/status_markers
/agt/validation/ground_truth_path
```

Machine-readable artifacts include:

```text
/tmp/agt_v25_11e_timeline.jsonl
/tmp/agt_v25_11e_summary.json
/tmp/agt_v25_11e_observability_result.json

/tmp/agt_v25_11e_<fault_case>_timeline.jsonl
/tmp/agt_v25_11e_<fault_case>_summary.json
/tmp/agt_v25_11e_<fault_case>_fault_metrics.json
/tmp/agt_v25_11e_compare_<fault_case>.json

/tmp/agt_v25_11e_comparison_matrix.json
```

The generated JSON files are the authoritative raw runtime records for exact per-run values

This Markdown file is the frozen engineering conclusion and retains the pre-optimization numerical baseline

## 10. Validation-harness reliability improvements

The completed V25-11 validation workflow also includes the following infrastructure fixes discovered during runtime testing:

- delayed launch arguments are snapshotted before nested includes mutate public LaunchConfigurations
- localization correction spacing follows the actual Gazebo ground-truth source timestamp
- active fault injection uses ROS simulation time and detects time rollback
- ROS `byte` fields from Humble Python bindings are normalized before integer conversion
- V25-11E status visualization uses individual `Marker` messages rather than `MarkerArray`
- live JSONL comparison reading tolerates only an incomplete append-in-progress tail record
- stale `/tmp` fault reports are removed before each comparison run
- top-level V25-11E validation launches terminate the entire test stack when acceptance exits, preventing old Gazebo or Nav2 processes from contaminating later runs

These fixes are part of the validation result because they are required for repeatable experiments rather than merely convenient tooling

## 11. What has been proven in simulation

The current Gazebo result supports the following claims:

- the validation BUNKER model can execute the commanded Nav2 route and produce physical displacement
- the static validation map and Route contract are compatible with the canonical localization identity
- Nav2 planner and controller can complete the software-only happy path
- navigation velocity commands pass through the tracked Safety authority before reaching the simulated chassis
- canonical localization failure causes fail-fast Route termination
- LiDAR and IMU dropout are detected by SensorHealth and force Safety zero output
- sensor-health failure now propagates to Route termination without waiting for the Nav2 progress-checker timeout
- all active failures produce machine-readable timing and physical-response metrics
- the normal path and failure paths are covered by automated acceptance Gates
- the validation stack can run repeatedly without intentionally retaining old validation processes

## 12. What has not yet been proven

The final V25-11E PASS must not be interpreted as validation of the complete field robot

The following remain outside the current SOFTWARE_ONLY scope:

- real BUNKER chassis driver and actuator dynamics
- real MID360 packet timing, packet loss, disconnect behavior, and driver recovery
- real IMU timing, calibration, vibration, drift, and disconnect behavior
- FAST-LIVO2 as the production odometry / localization source
- GNSS / RTK fusion and global correction behavior on the real platform
- real map construction and long-term map consistency
- real agricultural vegetation, dynamic obstacles, wheel slip, terrain, and lighting effects
- real network, USB, Ethernet, power, and process-failure behavior
- real emergency-stop hardware chain
- repeated-trial statistics and confidence intervals for response latency and stopping distance

These items belong to the next real-robot validation stage rather than V25-11E

## 13. Final engineering judgment

**V25-11C through V25-11E are closed for the current SOFTWARE_ONLY Gazebo validation scope**

The project now has a reproducible simulation validation platform rather than only a collection of navigation nodes

The platform can exercise a complete normal navigation loop and deterministic critical-failure loops, and it can judge the result through explicit machine-readable acceptance criteria

The next development stage should reuse these same interfaces, fault semantics, timelines, metrics, and Gates while replacing simulated authorities with real robot data sources

Recommended next stage:

```text
V25-11F REAL-ROBOT VALIDATION PREPARATION
```

The main objective of V25-11F should be to preserve the already validated software contracts while progressively replacing Gazebo adapters and synthetic localization evidence with the real BUNKER, MID360, IMU, FAST-LIVO2, and later GNSS data paths
