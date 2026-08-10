# V25-11 Gazebo System Validation

## Objective

V25-11 is the software-system validation layer between V25-10 localization / Route runtime acceptance and physical BUNKER testing.

The objective is not photorealistic simulation and not vehicle-dynamics research. The objective is to expose software integration problems before field deployment:

```text
Gazebo world / simulated vehicle
        ↓
ROS hardware-facing contracts
        ↓
Localization / correction
        ↓
Navigation capability
        ↓
MAP or ROUTE backend
        ↓
Nav2 controller
        ↓
Collision monitor / safety
        ↓
simulated vehicle
```

The simulation must be fail-closed and must never be cited as localization or vehicle accuracy evidence.

## Simulator baseline

Use:

- ROS 2 Humble
- Gazebo Fortress
- `ros_gz_sim`
- `ros_gz_bridge`

Do not introduce new Gazebo Classic dependencies.

Simulation assets live in the isolated `agt_simulation` package. Navigation and localization packages must not gain Gazebo-specific imports.

## V25-11A — Simulation foundation

Deliverables:

- `agt_simulation` package
- SOFTWARE_ONLY BUNKER proxy
- simple row / headland / fixed-obstacle world
- ROS <-> Gazebo command, odometry, lidar, IMU bridge
- simulation odometry adapter owning `odom -> base_footprint`
- sensor-frame normalization
- RViz profile with placeholders for Nav2 global plan and AGT RuntimePath

Acceptance:

```text
/clock                         one simulation authority
/agt/safety/cmd_vel            moves the simulated vehicle
/agt/mapping/odometry          stable and finite
odom -> base_footprint         available
/agt/sensors/lidar/scan        >= 5 Hz
/agt/sensors/imu/data          >= 50 Hz
base -> sensor TF              available
map -> odom                    MUST NOT be published by agt_simulation
```

## V25-11B — Localization and fault injection

Do not create a competing localization stack.

Add a simulation truth / evidence adapter that feeds the existing V25-10 relocalization / GlobalCorrectionManager contract. `global_correction_manager` remains the only `map -> odom` authority.

Fault-injection cases:

- translation correction: 0.2 m / 0.5 m / 1.0 m
- yaw correction: 5 deg / 10 deg / 20 deg
- temporary evidence loss: 1 s / 5 s
- stale evidence
- rejected correction
- RECOVERING -> LOST escalation
- accepted reanchor

Acceptance must verify canonical `correction_generation` and the V25-10 next-segment-only Route behavior.

## V25-11C — Full navigation system

Run the real software chain in Gazebo:

```text
ExecuteWaypointTask
    ├── MAP backend
    │     └── Nav2 global planner
    └── ROUTE backend
          └── fixed READY Route
                  ↓
              FollowPath
                  ↓
          controller_server
                  ↓
          collision_monitor
                  ↓
               safety
                  ↓
               Gazebo
```

Required cases:

- start / cancel / success
- forward segment
- reverse segment
- segment boundary
- localization loss during active segment
- safety stop
- fixed obstacle
- route binding invalid -> fail closed
- MAP task without Route binding retains legacy behavior

## V25-11D — Automated failure matrix

Create a deterministic acceptance harness that records:

- stage result
- ROS graph / process manifest
- TF ownership
- command velocity
- odometry
- localization status and correction generation
- Nav2 Action results
- collision / safety state
- final displacement

At minimum automate:

```text
baseline
correction_during_segment
localization_lost
estop
obstacle_stop
controller_failure
invalid_route_binding
```

Every run must produce machine-readable JSON plus a concise Markdown report.

## V25-11E — Planning visualization and comparison

RViz must show, at the same time where applicable:

- occupancy map
- robot pose / TF
- Nav2 global plan `/plan`
- READY Route reference
- AGT RuntimePath `/agt/navigation/runtime_path`
- local / global costmap
- footprint
- obstacle scan / cloud
- controller trajectory when available

This stage establishes the visualization and experiment interface before implementing agricultural planning research.

Planner comparison should eventually separate:

```text
MAP mode:
Nav2 Global Planner -> FollowPath

ROUTE mode:
READY Route -> RuntimePath -> FollowPath

Future agricultural planner:
row / headland planner -> formal Route asset -> RuntimePath -> FollowPath
```

The future agricultural planner must not bypass the Route asset identity / vehicle-binding contract.

## Post V25-11 — BUNKER + G70 field validation

After Gazebo software acceptance:

1. deploy BUNKER + LiDAR / IMU + G70 RTK GNSS in an open field
2. keep G70 as independent truth first
3. evaluate estimated trajectory against RTK ENU truth
4. report ATE / RPE, relocalization success, correction magnitude and LOST events
5. only after the independent-truth baseline consider GNSS fusion

Path-planning development can proceed in parallel after V25-11C once planner inputs, outputs and RViz visualization are stable.

## V25-11 branch gate

V25-11A is complete only after host execution proves the simulator starts and all A-stage topic / TF gates pass. Static file contracts alone are not runtime acceptance.
