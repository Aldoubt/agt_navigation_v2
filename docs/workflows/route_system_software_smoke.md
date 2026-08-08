# Route System Software Smoke

## Purpose

This is the final software-only V25-09B gate before real dataset and vehicle acceptance.

It validates the complete formal ROUTE chain:

```text
synthetic READY Map / TaskGroup / Route Asset
  -> ExecuteWaypointTask
  -> Navigation Capability
  -> exact TaskGroup -> Route binding
  -> RouteSegment(map)
  -> live map->odom snapshot
  -> RuntimePath(odom)
  -> real Nav2 controller_server / FollowPath
  -> collision_monitor
  -> agt_safety
  -> differential-drive simulator
  -> odom / TF
```

The fixture writes only below `/tmp/agt_route_system_smoke` by default. It does not modify real `runtime/maps` assets.

This test deliberately does not start a Nav2 global planner, map server or BT navigator. A PASS therefore proves the ROUTE execution path can reach the real Controller Server without a Global Planner request.

It is not MK-mini, BUNKER, localization, map-production or field-accuracy acceptance.

## Build

```bash
cd ~/agt_navigation_v2

git checkout feat/v25-09b-route-navigation-core
git pull --ff-only origin feat/v25-09b-route-navigation-core

rm -rf build/agt_navigation install/agt_navigation
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to agt_navigation agt_safety
source install/setup.bash
```

Confirm both smoke executables exist:

```bash
ros2 pkg executables agt_navigation | grep route_system_smoke
```

Expected:

```text
agt_navigation route_system_smoke.py
agt_navigation route_system_smoke_fixture.py
```

## Terminal A - full software stack

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_navigation route_system_smoke.launch.py
```

The launch starts:

```text
agt_route_controller_simulator
agt_route_controller_safety
controller_server
collision_monitor
lifecycle_manager_route_controller_smoke
agt_route_system_smoke_fixture
agt_route_system_navigation_capability
```

Wait until the lifecycle manager reports:

```text
Managed nodes are active
```

and the fixture reports that the synthetic READY route fixture was prepared.

No additional waiting interval is required after those conditions are true.

## Terminal B - run the formal ROUTE task

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agt_navigation route_system_smoke.py
```

The client waits for:

- `/agt/navigation/execute_waypoint_task`
- synthetic READY active-map metadata
- simulator odometry
- safety motion-enabled state
- generated formal TaskGroup JSON

It then loads the exact TaskGroup content hash and submits the normal public `ExecuteWaypointTask` Action.

A successful result contains:

```json
{
  "success": true,
  "execute_waypoint_task_status": 4,
  "result_success": true,
  "blocker_code": "",
  "measured_displacement_m": 1.0,
  "global_planner_requests": 0
}
```

Exact displacement is not the acceptance target. It must only exceed the configured `minimum_motion_m` and complete successfully through the full formal route chain.

## Optional Terminal C - observe motion

```bash
ros2 topic echo /agt/mapping/odometry --field pose.pose.position
```

The default synthetic route is 2 m so that motion is easier to observe than the earlier 1 m controller-only smoke.

If `x/y` values become constant after several updates, this normally means the short route has completed and the simulator is continuing to publish its final pose. It is not rosbag playback and it does not by itself indicate a broken chain. Re-running Terminal B from the current pose should produce another visible motion segment if the stack remains healthy.

You can also inspect the command chain:

```bash
ros2 topic echo /agt/navigation/cmd_vel_raw
ros2 topic echo /agt/navigation/cmd_vel
ros2 topic echo /agt/safety/cmd_vel
```

## Generated synthetic assets

Default location:

```text
/tmp/agt_route_system_smoke/maps/
└── route_smoke_site/
    └── versions/
        └── route_smoke_v1/
            ├── manifest.yaml
            ├── tasks/
            │   ├── route_smoke_task.json
            │   └── route_smoke_task.route.yaml
            └── routes/
                └── route_smoke_main/
                    └── 1/
                        ├── route.yaml
                        └── route.csv
```

The route is bound to `config/route_smoke_vehicle.yaml`, whose acceptance status is explicitly `SOFTWARE_ONLY`. Do not reuse this profile for a real vehicle.

## Acceptance

PASS requires all of the following:

- formal `ExecuteWaypointTask` succeeds
- the TaskGroup content hash matches the generated TaskGroup
- the TaskGroup -> Route execution binding is READY and exact
- the Route manifest and `route.csv` hashes validate
- active Map identity and content binding validate
- `map -> odom` is available
- real Nav2 `FollowPath` accepts and completes the odom RuntimePath
- safety remains ready
- simulator displacement exceeds the configured threshold
- terminal ROUTE status reports `global_planner_requests == 0`

After PASS, V25-09B software integration can be considered closed. Remaining gates are external evidence:

```text
real bag -> canonical READY Map -> semantic/READY Route
MK-mini -> Ackermann vehicle profile + route tracking acceptance
BUNKER -> RTK truth localization experiment
```
