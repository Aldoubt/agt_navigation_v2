# P0 Reliable Operator Control Acceptance

This checklist validates the robot-side Task Registry, ID-driven waypoint
execution, Navigation Session recovery, and fail-closed map/task checks. Run on a
READY map version produced by `agt_map_manager`; keep `start_chassis:=false` for
dry runs and enable chassis only after CAN, estop, and watchdog checks pass.

## Common Setup

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

VERSION_ROOT=runtime/maps/<map_id>/versions/<map_version_id>
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  runtime_dir:=runtime \
  map:="$VERSION_ROOT/navigation/map.yaml" \
  global_map_pcd:="$VERSION_ROOT/pointcloud/localization_map.pcd" \
  global_map_processing_record:="$VERSION_ROOT/pointcloud/localization_map.processing.yaml" \
  map_id:=<map_id> \
  map_version_id:=<map_version_id> \
  start_gui:=true \
  start_chassis:=false
```

Confirm active identity and session topic:

```bash
ros2 topic echo /agt/maps/active --once
ros2 topic echo /agt/navigation/session_status --once
ros2 service call /agt/navigation/session/get agt_interfaces/srv/GetNavigationSession "{session_id: '', client_request_id: ''}"
```

## Scenario A: Different Qt And Robot Paths

Robot workspace may be `/home/robot/agt_navigation_v2`; the operator laptop may be
`/home/user/agt_navigation_v2`. Save or synchronize the task into the robot's
Task Registry under the selected READY version. Start execution from Qt.

Expected result: Qt sends only map/task ID, revision, hash, and request ID. The
robot loads the task from its own `runtime/maps` tree. No absolute laptop path is
visible in the Action goal or logs.

## Scenario B: Qt Disconnect

Start a three-point task, then close Qt or interrupt the network. Do not stop the
navigation launch.

Expected result: Nav2 continues subject to localization, TaskReadiness, and
`agt_safety`. Reopen Qt and verify the current or terminal session through
`/agt/navigation/session_status` or `/agt/navigation/session/get`.

## Scenario C: Duplicate Submit

Submit the same saved task twice with the same `client_request_id`, using an
Action client or captured Qt request.

Expected result: Nav2 receives one `FollowWaypoints` child goal. The second
request returns `duplicate_request=true` or the current session status and does
not start a second child.

## Scenario D: Wrong Map Version

Save a task bound to version `v1`, activate or launch version `v2`, and execute
the `v1` task identity.

Expected result: the Action fails closed before Nav2 dispatch with
`MAP_VERSION_MISMATCH`. The operator message is equivalent to
`任务属于其他地图版本，请切换地图或重新绑定任务。`

## Scenario E: Revision Conflict

Keep a client request at revision `2`, then update the robot-side task to
revision `3`. Submit revision `2`.

Expected result: the Action fails closed with `TASK_REVISION_CONFLICT`. No Nav2
child goal is created.

## Scenario F: Network Jitter

Temporarily make the status topic unavailable from the frontend side only, then
restore connectivity.

Expected result: Qt must not infer that the task stopped. After reconnect it
uses the transient-local session topic or `GetNavigationSession` service to show
the authoritative state.

## Deprecated Compatibility Check

`task_file` execution is disabled by default. A same-machine debug run may enable
`allow_legacy_local_task_file:=true`, but the file must resolve below the
configured `runtime/maps` root. Qt/Web navigation profiles must not use this path.
