# Web 实验与运维控制台架构

状态：P0-P6、P8 离线基线（2026-07-22）；P7 保持未修改维护版 Qt fork。本架构扩展现有模块化导航平台，不替换
FAST-LIVO2、定位、Nav2、`agt_safety`、BUNKER driver 或维护版 Qt。

## Ownership

| Package | Inputs | Outputs | Non-goals |
| --- | --- | --- | --- |
| `agt_system_manager` | ROS graph, typed topic observations, TF, lifecycle, active-map pointer | `SystemHealth`, `TaskReadiness`, `ChangeSystemMode`, bounded localization mode service | no velocity, TF, mapping, localization algorithm |
| `agt_map_manager` | version manifest and assets | SQLite index, validation result, atomic active pointer, retention decisions | no map algorithm or online map publisher |
| `agt_experiment_manager` | operator metadata, health snapshots, structured localization events, bag profile | immutable session files, JSONL event/result streams, summary/report | no arbitrary recorder command and no success fabrication |
| `agt_web_console` | ROS bridge data, optional offline simulator, and project file managers | REST/WebSocket/UI | no direct ROS velocity, TF, or human-readable status parsing |

The maintained Qt fork remains a frontend. It is not patched for this feature and
must continue to call project Actions. `map -> odom` ownership remains in
`agt_localization`.

## Runtime states

Main mode is one of `IDLE`, `SENSOR_ONLY`, `MAPPING`, `LOCALIZATION_DEBUG`,
`NAVIGATION`, or `ERROR`. Experiment recording is orthogonal and is represented
inside the experiment manifest as `RUNNING`; it is not a main mode.

Mode manager profiles are loaded from `mode_profiles.yaml`. Commands are argv
arrays, executable names are allowlisted, and only declared launch argument keys
are accepted. Each child starts in its own process group. Shutdown sends signals
only to process groups recorded by that manager; it never discovers and kills
unowned ROS nodes.

## Health and task flow

`health_contracts.yaml` evaluates message counts/timestamps, configured rate and
age bounds, expected type, TF pairs, node presence, lifecycle state, conditions,
and disk space. `UNKNOWN` means no evidence; graph discovery alone is not `OK`.

`TaskReadiness` is computed fail-closed from active mode, active map identity,
navigation/PCD asset validity, accepted fresh `LocalizationStatus`, emergency
stop, chassis, safety, Nav2 lifecycle, TF, and task validation. The waypoint
Action server checks it at goal acceptance, before child dispatch, and while a
child is running. A stale or blocked message cancels the child.

## Data flow

```text
ROS topics/TF/lifecycle -> agt_system_manager -> SystemHealth/TaskReadiness
                                      |                 |
                                      v                 v
                              ChangeSystemMode      task Actions
Web REST/WebSocket -> ROS bridge -----------------> existing Nav2/safety chain
Web map API -> agt_map_manager -> active_map.yaml -> health/readiness identity
Web experiment API -> agt_experiment_manager -> manifest/events/report
```

The Web listener defaults to loopback. Non-loopback configuration requires a
token. Runtime directories are configured, not hardcoded to a user workspace.

The Web runtime has two explicit backends. `ros` delegates to the real project
Actions/SRV and configured system-manager profiles. `offline` is a deterministic
UI simulator for machines without MID360, CAN, chassis, Nav2, or localization
inputs. The offline backend can show module transitions and a simulated bounded
relocalization result, but it never starts ROS launch processes, records bags,
publishes TF/velocity, or makes task execution ready. Switching backends requires
stopping all managed modules first.
