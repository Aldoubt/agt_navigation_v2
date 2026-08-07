# agt_web_console

The Web console is a local operations frontend. Its real backend contains only
`RosConsoleBridge`: mode, Mission, map, experiment, Bag and mapping operations use
generated project ROS interfaces. It does not instantiate manager storage classes,
read business manifests or process snapshots, publish `cmd_vel`/TF, run algorithms,
or accept arbitrary shell commands.

In `MAPPING`, its ROS bag playback path is input-only: the manager replays the
raw MID360/IMU topic allowlist and excludes recorded FAST-LIVO2 outputs and
recorded `/tf`. This keeps one publisher for each mapping output and lets the
live FAST-LIVO2 chain own those outputs.

FastAPI, Starlette, and Uvicorn are optional runtime dependencies because the
base ROS Humble image does not provide them. The service layer and offline tests
remain usable without those packages. The default listener is `127.0.0.1`; a
non-loopback listener requires a configured token.

`/agt/system/robot_state` and `/agt/missions/status` are the primary WebSocket read
models; health/readiness/localization projections remain for REST compatibility.
Existing REST URLs are retained as HTTP adapters over ROS.

The static frontend is intentionally dependency-light and defaults to a Chinese
operations dashboard (`zh-CN`). It presents the ordered startup flow, profile
controls, map lifecycle, task gate, experiment session, logs, and bounded
relocalization Action controls. Runtime writes are audited under the configured
runtime directory, and log browsing is restricted to manager-owned roots.

For hardware-free UI checks, start with `--backend offline`. This backend only
simulates configured profile state and a bounded relocalization response. It
does not launch ROS processes, record bags, publish TF or velocity, create real
map/experiment assets, or permit Mission/task execution. The page can switch
between `ros` and `offline` only after
managed modules are stopped.
