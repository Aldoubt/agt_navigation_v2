# AGT UI Contracts

## Ownership

Qt5 is a frontend. Authoritative state belongs to:

| Domain | Owner | Qt boundary |
| --- | --- | --- |
| Runtime mode | `agt_system_manager` | `ChangeSystemMode` |
| Unified state | RobotState aggregator | `/agt/system/robot_state` |
| Mission | `agt_mission_manager` | `ExecuteMission`, `SetMissionRunState`, MissionStatus |
| Managed mapping | mapping session manager | `ManageMappingSession` |
| Active map and versions | `agt_map_manager` | map list/manage services and active summary |
| Bag and experiment | `agt_experiment_manager` | bag list/manage services |
| Relocalization | `agt_localization` | project `Relocalize` Action |
| Waypoint capability | `agt_navigation` | `ExecuteWaypointTask`, compatibility only in the new shell |
| Safety and chassis | `agt_safety`, `agt_chassis` | read state; never publish final velocity |

The formal Qt ROS endpoints are:

```text
/agt/system/change_mode
/agt/system/robot_state
/agt/missions/execute
/agt/missions/set_run_state
/agt/missions/status
/agt/mapping/manage_session
/agt/localization/relocalize
/agt/maps/list
/agt/maps/manage
/agt/data/bags/list
/agt/data/bags/manage
```

Do not call `/follow_waypoints`, `/navigate_to_pose`, launch files, or bag
processes from the new business pages. `/goal_pose` is deprecated debugging
only and must default hidden/disabled.

## Capability Matrix

Profile JSON owns permissions. Missing keys mean false.

| Profile | Mission | Mapping session | Relocalize | Map manager | Bag manager | Manual | Base raster edit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| mapping | no | yes | no | no | no | allowed by current baseline | no |
| candidate | no | no | no | no | no | no | yes, selected candidate only |
| navigation | yes | no | yes | yes | no | allowed through safety input | no |
| offline | no | no | no | no | no | no | no |
| teach | no | no | no | no | yes | no | no |

`UiThemeId`, `UiLayoutId`, `UiDensity`, and
`ShowAdvancedDiagnostics` never alter this matrix. Page visibility also does
not grant an operation; the ROS channel repeats every critical capability
check.

## Map And Task Rules

- READY PGM/YAML assets are immutable in navigation, offline, and teach.
- Candidate editing may write only the selected managed candidate in place.
- Candidate cannot open another map or save as another path.
- Task Library persists metric `map` poses under the selected immutable map
  version. Do not reimplement its repository or scene-coordinate conversion.
- Navigation mode arguments must use the active `MapVersionSummary` supplied by
  map manager through RobotState. Do not join `runtime/maps/...` paths in Qt.
- Theme or shell replacement must not change task hashes, map binding, planner
  preview semantics, or Action result handling.

## Velocity Boundary

```text
Qt manual input -> /agt/cmd_vel_manual -> agt_safety
Nav2 -> /agt/navigation/cmd_vel_raw -> collision monitor -> agt_safety
agt_safety -> /agt/safety/cmd_vel -> chassis guard -> /agt/chassis/cmd_vel
```

Qt may publish only the manual input. It must never publish either downstream
topic.

## Fork Synchronization

1. Change `/home/.../Ros_Qt5_Gui_App` on `agt-navigation-v2` first.
2. Build and commit the fork while retaining GPL-2.0 attribution.
3. Push the fork commit.
4. Copy the exact changed files into `third_party/ros_qt5_gui_app`.
5. Record the full 40-character commit in `.agt-fork-commit` and
   `third_party/README.md`.
6. Compare every file changed since the previous pin byte-for-byte.

The vendored directory is a fixed snapshot, not an independent source branch.
