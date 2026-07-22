# agt_web_console

The Web console is a local operations frontend. It consumes structured health,
task-readiness, localization, map-registry, experiment, and process-manager
services. It does not publish `cmd_vel`, publish TF, run localization or mapping
algorithms, or accept arbitrary shell commands.

FastAPI, Starlette, and Uvicorn are optional runtime dependencies because the
base ROS Humble image does not provide them. The service layer and offline tests
remain usable without those packages. The default listener is `127.0.0.1`; a
non-loopback listener requires a configured token.

The static frontend is intentionally dependency-light and defaults to a Chinese
operations dashboard (`zh-CN`). It presents the ordered startup flow, profile
controls, map lifecycle, task gate, experiment session, logs, and bounded
relocalization Action controls. Runtime writes are audited under the configured
runtime directory, and log browsing is restricted to manager-owned roots.

For hardware-free UI checks, start with `--backend offline`. This backend only
simulates configured profile state and a bounded relocalization response. It
does not launch ROS processes, record bags, publish TF or velocity, or permit
task execution. The page can switch between `ros` and `offline` only after
managed modules are stopped.
