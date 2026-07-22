# Web Console API

The runtime adapter is FastAPI plus WebSocket. It defaults to `127.0.0.1`; when
`token` is configured, REST uses `X-AGT-Token` and WebSocket uses the same header.
No endpoint accepts a shell command.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/api/v1/overview` | health, readiness, mode, active maps, experiments, localization |
| GET | `/api/v1/health` | current `SystemHealth` projection |
| GET | `/api/v1/task-readiness` | current shared gate |
| GET | `/api/v1/system/status` | managed process/mode projection |
| POST | `/api/v1/system/mode` | start a configured profile with declared key/value launch args |
| POST | `/api/v1/system/stop` | stop manager-owned mode process groups |
| GET | `/api/v1/maps` | list/filter registered versions |
| POST | `/api/v1/maps/{version_id}/validate` | validate bundle and hashes |
| POST | `/api/v1/maps/{version_id}/activate` | activate only a valid READY version |
| POST | `/api/v1/maps/{version_id}/{pin,unpin,archive,delete,purge}` | guarded version lifecycle action; purge is explicit |
| POST | `/api/v1/maps/import` | explicitly package legacy PGM/YAML, PCD and processing record |
| GET/POST | `/api/v1/experiments` | list/create sessions |
| POST | `/api/v1/experiments/{id}/{start,event,start_bag,stop_bag,finalize,invalid}` | controlled session and explicit bag-profile transitions |
| POST | `/api/v1/localization/mode` | call `SetLocalizationMode` |
| POST | `/api/v1/localization/relocalize` | send one bounded structured `Relocalize` Action goal |
| GET | `/api/v1/runtime` | current `ros` or `offline` backend |
| POST | `/api/v1/runtime/backend` | switch backend after managed modules are stopped |
| GET | `/api/v1/logs?component=...` | list manager-owned log files only |
| WebSocket | `/ws` | initial overview plus live structured health/readiness/localization, audit, and mode events |

The static page defaults to `zh-CN` and provides a Chinese operations dashboard
with an ordered startup workflow, profile controls for sensor/mapping/navigation,
map lifecycle actions, task-readiness display, experiment recording, and a
bounded relocalization Action form. It does not expose a velocity or TF control.

The configured `offline` backend is a deterministic Web-only simulator. It may
simulate profile state and one bounded relocalization result for UI checks, but
it never launches ROS processes, records bags, publishes TF or velocity, or
opens task readiness.
