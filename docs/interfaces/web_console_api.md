# Web Console API

The runtime adapter is FastAPI plus WebSocket. It defaults to `127.0.0.1`; when
`token` is configured, REST uses `X-AGT-Token`. WebSocket uses that header for
native clients and accepts the same token as the `token` query parameter for
browser clients, because the browser WebSocket API cannot set arbitrary headers.
No endpoint accepts a shell command.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/api/v1/overview` | RobotState、MissionStatus、health、readiness、mode、maps、experiments、localization |
| GET | `/api/v1/health` | current `SystemHealth` projection |
| GET | `/api/v1/task-readiness` | current shared gate |
| GET | `/api/v1/system/status` | managed process/mode projection |
| GET | `/api/v1/mapping/map` | mode-gated bounded occupancy preview and current mapping robot pose; includes `active_mode` |
| GET | `/api/v1/mapping/pointcloud` | mode-gated bounded registered-point preview and current mapping robot pose; includes `active_mode` |
| GET | `/api/v1/mapping/session` | current managed mapping session and PGM/PCD asset evidence |
| POST | `/api/v1/mapping/session/prepare` | call ManageMappingSession START with a fixed map ID and allowed mapping arguments |
| POST | `/api/v1/mapping/finish` | `retain` finalizes capture to an editable candidate, `commit` registers a new version, `delete` discards recoverably |
| POST | `/api/v1/system/mode` | start a configured profile with declared key/value launch args |
| POST | `/api/v1/system/stop` | stop manager-owned mode process groups |
| GET | `/api/v1/maps` | list/filter registered versions |
| POST | `/api/v1/maps/{version_id}/validate` | validate bundle and hashes |
| POST | `/api/v1/maps/{version_id}/activate` | activate only a valid READY version |
| POST | `/api/v1/maps/{version_id}/{pin,unpin,archive,delete,purge}` | guarded version lifecycle action; purge is explicit |
| POST | `/api/v1/maps/import` | explicitly package legacy PGM/YAML, PCD and processing record |
| GET/POST | `/api/v1/experiments` | list/create sessions |
| POST | `/api/v1/experiments/{id}/{start,event,start_bag,stop_bag,finalize,invalid}` | controlled session and explicit bag-profile transitions |
| GET | `/api/v1/bags` | list complete configured rosbag bundles and playback state |
| POST | `/api/v1/bags/{play,stop}` | simulate selected-bag state in offline mode or start/stop real `ros2 bag play --clock` in ROS mode |
| GET | `/api/v1/missions/status` | current manager-owned MissionStatus projection |
| POST | `/api/v1/missions/execute` | send one versioned `ExecuteMission` project Action goal |
| POST | `/api/v1/missions/{id}/{pause,resume,cancel}` | call run-state service or ROS Action cancel |
| POST | `/api/v1/localization/mode` | call `SetLocalizationMode` |
| POST | `/api/v1/localization/relocalize` | send one bounded structured `Relocalize` Action goal |
| GET | `/api/v1/runtime` | current `ros` or `offline` backend |
| POST | `/api/v1/runtime/backend` | switch backend after managed modules are stopped |
| GET | `/api/v1/logs?component=...` | list manager-owned log files only |
| WebSocket | `/ws` | initial overview plus RobotState/MissionStatus-primary live status and audit events |

The static page defaults to `zh-CN` and provides a Chinese operations dashboard
with an ordered startup workflow, separate sensor/mapping/navigation controls,
map lifecycle actions, task-readiness display, experiment recording, and a
bounded relocalization Action form. It does not expose a velocity or TF control.

Real mapping is delegated entirely to `/agt/mapping/manage_session`; the Web
service does not create artifact directories, call SaveMap, wait for PCD, or
register a version. `retain` confirms acquisition and runs FINALIZE_CAPTURE,
which preserves the online preview, normally closes PCD/bag writers, rebuilds
the offline ray-traced + `ground_temporal` map, and yields `CANDIDATE_READY` only
after its production report passes. A retryable offline-build failure must not be
presented as an editable candidate. `commit` separately validates the possibly edited
candidate and creates an immutable version. Navigation accepts only an active
READY version and uses the three navigation asset paths returned by `agt_map_manager`.

The real backend does not instantiate map or experiment storage classes. Every map,
experiment, Bag, mode, mapping-session, localization, and Mission REST operation delegates
through `RosConsoleBridge` to a generated ROS topic/service/action. The bridge does not read
manager manifests or process snapshots. Existing REST paths remain compatibility adapters;
they are not a second business API owner.

The configured `offline` backend is a deterministic Web-only simulator. It may
simulate profile state, one bounded relocalization result, and the playback state
of a selected complete bag bundle for UI checks, but it never reads ROS messages,
launches ROS processes, runs `ros2 bag play`, records bags, publishes TF or
velocity, or opens task readiness. The ROS backend accepts only a relative bag
identifier discovered below the configured runtime rosbag root, validates its
metadata bundle, and launches it in a manager-owned process group with
`--clock`; nodes under test must use simulated time when replaying historical data.

When the offline simulator is in simulated `MAPPING` and a bag has been selected
for simulated playback, the two mapping preview endpoints expose bounded,
deterministic occupancy and point-cloud examples marked `simulated: true`. These
examples are for UI interaction checks only, are never decoded from bag contents,
and are cleared when mapping stops. Offline retain/delete uses one in-memory
simulated map slot and never exports PGM/YAML/PCD. Real assets for semantic
authoring, navigation, and relocalization require the ROS backend with
`start_sensor:=false`, historical bag playback, and the normal ready-PCD map
registration flow.

The ROS mapping preview endpoints are evidence views, not topic mirrors. They
return an empty preview unless the managed mode is `MAPPING`; a bag that contains
mapping topics cannot populate the preview while only bag playback is running.
The mapping profile may start FAST-LIVO2 with `start_sensor:=false`, allowing a
historical bag to provide the input topics. The returned `robot_pose` is only for
view centering and does not publish or modify TF. `active_mode` distinguishes a
stopped mapping chain from a running chain that is still waiting for its first
registered cloud. Entering `MAPPING` does not clear a preview that arrived during
the startup/readiness transition; leaving `MAPPING` clears it. The Web point-cloud
canvas is a read-only view with X-Y top, X-Z side, and Y-Z side projections, pan,
zoom, a bounded rotation slider, and frame/grid/axis overlays; it does not alter
the ROS cloud or navigation geometry.
