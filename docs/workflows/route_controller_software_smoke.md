# Route Controller Software Smoke

## Purpose

This workflow validates the V25-09B controller plumbing before any bag or vehicle acceptance:

```text
synthetic odom Path
  -> Nav2 controller_server / FollowPath
  -> collision_monitor
  -> agt_safety
  -> differential-drive simulator
  -> /agt/mapping/odometry + odom->base_footprint
```

It deliberately does not start:

- map_server
- planner_server
- bt_navigator
- waypoint_follower
- Mission execution
- real chassis control

Therefore a PASS proves that an `odom`-frame RuntimePath can be tracked by the real Nav2 Controller Server through the existing collision and safety command chain. It does not prove map localization, offline Route Asset correctness, MK-mini Ackermann acceptance, BUNKER vehicle acceptance or field navigation accuracy.

## Prerequisites

Build the relevant packages:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to agt_navigation agt_safety
source install/setup.bash
```

## Terminal A - controller stack

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_navigation route_controller_smoke.launch.py
```

Expected active software chain:

```text
controller_server
collision_monitor
agt_route_controller_safety
agt_route_controller_simulator
lifecycle_manager_route_controller_smoke
```

The smoke launch forces `require_localization_valid=false` only inside this isolated controller test. Localization-loss behavior is tested independently by the Navigation Capability Action tests. Production navigation must continue to use the normal localization gate.

## Terminal B - send the odom path

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agt_navigation route_controller_smoke.py --ros-args \
  -p distance_m:=1.0 \
  -p minimum_motion_m:=0.50
```

The client waits for `/agt/mapping/odometry`, builds a straight path from the simulator's current odom pose, sends it directly to `FollowPath`, waits for the terminal result, and measures simulator displacement.

A successful result has the form:

```json
{
  "success": true,
  "follow_path_status": 4,
  "frame_id": "odom",
  "global_planner_requests": 0,
  "requested_distance_m": 1.0,
  "measured_displacement_m": 0.5
}
```

The measured displacement will normally exceed the configured minimum and approach the requested distance. Exact controller tracking metrics are not acceptance criteria for this software-only smoke.

## Failure interpretation

### FollowPath unavailable

Check that `controller_server` reached the active lifecycle state:

```bash
ros2 lifecycle get /controller_server
```

### Robot does not move

Inspect the velocity chain:

```bash
ros2 topic echo /agt/navigation/cmd_vel_raw
ros2 topic echo /agt/navigation/cmd_vel
ros2 topic echo /agt/safety/cmd_vel
```

Expected ownership is:

```text
controller_server
  -> /agt/navigation/cmd_vel_raw
collision_monitor
  -> /agt/navigation/cmd_vel
agt_safety
  -> /agt/safety/cmd_vel
simulator
  -> odom / TF
```

### Local costmap or TF error

Inspect:

```bash
ros2 topic hz /agt/mapping/odometry
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /agt/perception/obstacle_cloud
```

The simulator owns only this synthetic `odom -> base_footprint` transform for the smoke environment. It is not a production TF authority.

## Acceptance

Mark this gate PASS only when:

- `FollowPath` is accepted by the real `controller_server`
- the returned action status is `SUCCEEDED`
- the submitted path frame is `odom`
- measured simulator displacement exceeds `minimum_motion_m`
- `global_planner_requests` remains zero
- no planner_server is started by the smoke launch

After this gate, the next integration target is the full formal path:

```text
READY Route Asset
  -> ExecuteWaypointTask
  -> Navigation Capability ROUTE backend
  -> map->odom segment projection
  -> real controller_server / FollowPath
  -> collision_monitor
  -> safety
  -> simulator or vehicle
```

The later vehicle gate must use the correct execution-vehicle profile and real vehicle kinematic parameters. This smoke must never be cited as MK-mini or BUNKER vehicle validation.
