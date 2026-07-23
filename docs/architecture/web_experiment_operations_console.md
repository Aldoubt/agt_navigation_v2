# Web 实验与运维控制台架构

状态：P0-P6、P8 离线基线（2026-07-22）；P7 保持未修改维护版 Qt fork。本架构扩展现有模块化导航平台，不替换
FAST-LIVO2、定位、Nav2、`agt_safety`、BUNKER driver 或维护版 Qt。

## Ownership

| Package | Inputs | Outputs | Non-goals |
| --- | --- | --- | --- |
| `agt_system_manager` | ROS graph, typed topic observations, TF, lifecycle, active-map pointer | `SystemHealth`, `TaskReadiness`, `ChangeSystemMode`, bounded localization mode service | no velocity, TF, mapping, localization algorithm |
| `agt_map_manager` | version manifest and assets | SQLite index, validation result, atomic active pointer, retention decisions | no map algorithm or online map publisher |
| `agt_experiment_manager` | operator metadata, health snapshots, structured localization events, bag profile | immutable session files, JSONL event/result streams, summary/report | no arbitrary recorder command and no success fabrication |
| `agt_chassis` | configured CAN interface and BUNKER driver status | normalized chassis status, connection, odometry and battery topics; safety-protected command output in control mode | no Web-owned CAN provisioning, no direct Web command path |
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

The MID360 `sensor_only` profile is a separate reusable process group. A mapping
or navigation transition stops only the previous main chain and reuses that
sensor group. An explicit transition to `SENSOR_ONLY` stops the mapping and
navigation groups. This avoids repeated sensor initialization while keeping
normal manager-owned shutdown semantics.

The manager atomically writes its owned process snapshot below the configured
runtime log root. The Web bridge may read that snapshot after reconnecting, but
it never scans or terminates arbitrary processes; topic health remains the
evidence for whether a module is actually working.

## Health and task flow

`health_contracts.yaml` evaluates message counts/timestamps, configured rate and
age bounds, expected type, TF pairs, node presence, lifecycle state, conditions,
and disk space. `UNKNOWN` means no evidence; graph discovery alone is not `OK`.
The mapping OccupancyGrid is explicitly persistent: it is delivered through a
transient-local snapshot and is not treated as a three-second telemetry stream.

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
One Web process is allowed per configured runtime directory. The entrypoint
acquires a runtime lock before creating the ROS bridge, so a second ROS or
offline instance cannot register another `agt_web_console_ros_bridge` node.

The Web runtime has two explicit backends. `ros` delegates to the real project
Actions/SRV and configured system-manager profiles. `offline` is a deterministic
UI simulator for machines without MID360, CAN, chassis, Nav2, or localization
inputs. The offline backend can show module transitions and a simulated bounded
relocalization result, but it never starts ROS launch processes, records bags,
publishes TF/velocity, or makes task execution ready. Switching backends requires
stopping all managed modules first.

The BUNKER integration has two explicit launch roles. `control` is the real
navigation chain and is enabled only by an explicit `start_chassis:=true`; its
command path remains Nav2 -> `agt_safety` -> chassis guard -> BUNKER driver.
`monitor` is intended for mapping and disconnected-vehicle tests: it may consume
CAN status and battery/odometry evidence, but does not start safety or a command
guard and remaps the driver input to the unowned `/agt/chassis/monitor_cmd_vel`.
The Web process may display `/sys/class/net/<iface>/operstate`, but CAN setup
requiring root privileges is provisioned by the host administrator.

The Web ROS backend lists and replays only complete rosbag bundles below the
configured runtime `rosbag` root. It uses a fixed `ros2 bag play --clock` argv and
does not accept shell fragments or arbitrary recorder flags. While `MAPPING` is
active, the manager forces the `mapping_inputs` profile containing only
`/clock`, `/tf_static`, `/agt/sensors/lidar/custom`, and `/agt/sensors/imu/data`;
recorded FAST-LIVO2 outputs and recorded `/tf` are excluded. `NAVIGATION` refuses
bag playback because replayed control, costmap, or TF output could conflict with
the live chain. Playback is an input to the normal ROS graph; it does not make readiness executable by itself. The
offline backend disables recording and real playback because it is a UI simulator,
not a bag player. It may validate a selected complete bundle and expose a clearly
marked simulated playback state, but it never reads ROS messages, starts
`ros2 bag play`, publishes topics, or changes task readiness.

For UI workflow checks, simulated playback may unlock one bounded occupancy and
point-cloud preview while the simulated mapping profile is active. The preview is
deterministic and explicitly marked as simulated; it is not derived from bag
messages. Offline retain/delete has one in-memory map slot and cannot export real
PGM/YAML/PCD. Historical bag map generation for semantic authoring, real navigation,
and relocalization stays on the ROS backend.

Sensor startup and mapping-algorithm startup are separate decisions. A mapping
launch may set `start_sensor:=false`; FAST-LIVO2 and map processing remain active
and wait for their configured input topics, so a ROS backend bag replay can supply
 historical MID360/IMU data. The recommended mapping input bundle is
`mapping_20260719_172810`: it has roughly 10 Hz CustomMsg and 200 Hz IMU for the
longest mapping-focused recording. The longer `navigation_20260719_164431` and
shorter `bunker_mapping_20260719_163246` also contain usable raw inputs, but all
three contain recorded algorithm outputs and therefore require the filtered
profile. The Web mapping preview is stricter than raw topic
discovery: it accepts occupancy and registered-cloud data only while the managed
mode is `MAPPING`, and clears both products when mapping is stopped. Canvas pan,
zoom, and robot-centering are display-only operations.

Mapping persistence is owned by the Web console only as orchestration. A mapping
request receives a unique temporary root under the configured runtime
`mapping_sessions` directory and passes its PCD output directory to `agt_bringup`.
The launch owns the map-saver service and FAST-LIVO2 shutdown; the Web service
observes the resulting PGM/YAML and ready PCD processing record, then delegates
immutable version registration to `agt_map_manager`. Delete removes only that
temporary root. Navigation startup must supply a selected active READY version;
both the browser and service validate that the requested assets match its manifest.
