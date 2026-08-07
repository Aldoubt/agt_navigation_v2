# agt_navigation_v2

## Project Positioning

AGT Navigation V2.5 is a modular ROS 2 navigation platform for agricultural
robots. Its current production-style target is one finite BehaviorTree mission
that composes existing project Actions and readiness contracts. The execution
boundary remains `Nav2 -> agt_safety -> chassis`; Qt, Web, and future mission
clients do not publish velocity or TF.

The development line is:

```text
AGT Navigation V2.5
        ↓
P0 First BT Mission
        ↓
P1 Robust Localization / Local Mapping
        ↓
P2 Long-term Agricultural Navigation Research
```

## Current Architecture

The authoritative system architecture is [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md).
It separates sensor/mapping/localization, navigation and safety capabilities,
ROS 2 business managers, and replaceable Qt/Web clients.

## Current Status

Status labels describe implementation or evidence, not automatic vehicle
acceptance. Each capability must distinguish `implemented`, `validated offline`,
`validated on bag`, and `validated on vehicle`.

### DONE

- FAST-LIVO2 adapter, mapping odometry, canonical registered cloud, and sparse
  PCD persistence.
- ICP/NDT relocalization with unique `map -> odom` ownership.
- Nav2 waypoint integration through the project `ExecuteWaypointTask` Action.
- `agt_safety` and BUNKER chassis command/watchdog boundaries.
- Semantic map data contract, keepout pipeline, and offline coverage products;
  coverage output remains execution-gated.
- System health, TaskReadiness, map/experiment manager foundations, and the
  existing Livox self-filter baseline.

### P0

- URDF self-filter geometry upgrade while preserving the current CustomMsg
  boundary.
- Sensor synchronization and health monitoring.
- Semantic waypoint mode, `agt_mission`, and the first finite BT mission.

### P1

- Rolling local map with raycast/log-odds timeout behavior.
- Ground plane and ground factor, GNSS and wheel odometry factors.
- GTSAM/iSAM2 localization backend.

### P2

- STD/Scan Context loop closure, long-term multi-layer maps, cross-growth
  relocalization, and seasonal/structural agricultural navigation research.

### OPTIONAL

- Non-essential operator, visualization, and experiment tooling that does not
  weaken the project Action, safety, TF, or chassis boundaries.

## Core Data Flow

`MID360 -> profile self-filter -> FAST-LIVO2 -> odometry/registered_points ->
localization -> Nav2 -> agt_safety -> chassis`.

The raw input is `/agt/sensors/lidar/custom`; the normal baseline consumes
`/agt/sensors/lidar/custom_filtered`. The public registered cloud is
`/agt/mapping/registered_points` (`sensor_msgs/msg/PointCloud2`, frame `odom`).
The complete runtime sequence is [`docs/architecture/runtime_dataflow.md`](docs/architecture/runtime_dataflow.md).

## Core TF Contract

- `agt_localization` is the only owner of `map -> odom`.
- `agt_mapping_fast_livo2_adapter` is the only owner of `odom -> base_footprint`.
- `robot_state_publisher` owns the robot and sensor description chain.

## Core Capability Interfaces

- Topic source of truth: [`docs/interfaces/topic_contract.md`](docs/interfaces/topic_contract.md)
- Health and readiness: [`docs/interfaces/system_health.md`](docs/interfaces/system_health.md)
- Waypoint Action: [`docs/interfaces/waypoint_task_action.md`](docs/interfaces/waypoint_task_action.md)
- Semantic map contract: [`docs/interfaces/semantic_map_schema.md`](docs/interfaces/semantic_map_schema.md)
- Navigation task orchestration: [`docs/architecture/navigation_task_orchestration.md`](docs/architecture/navigation_task_orchestration.md)

## Repository Layout

```text
docs/architecture/   current system and runtime design
docs/interfaces/     current ROS and data contracts
docs/roadmap/        V2.5 priorities and component status
docs/archive/        historical migration, experiments, and plans
profiles/            canonical platform geometry and limits
src/                 ROS 2 packages
tests/               package and contract tests
```

## Quick Start

Use a repository-local workspace variable; do not hardcode a user or workspace
path:

```bash
export AGT_WS=/path/to/agt_navigation_v2
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start a profile through `agt_bringup` or the system manager. Navigation commands
must use the project Action and pass localization, readiness, Nav2, safety, and
chassis gates. Semantic and coverage launch switches remain disabled by default.

## Documentation Index

- Architecture: [`docs/architecture/system_architecture.md`](docs/architecture/system_architecture.md)
- Runtime flow: [`docs/architecture/runtime_dataflow.md`](docs/architecture/runtime_dataflow.md)
- Interfaces: [`docs/interfaces/topic_contract.md`](docs/interfaces/topic_contract.md)
- System health: [`docs/interfaces/system_health.md`](docs/interfaces/system_health.md)
- Semantic schema: [`docs/interfaces/semantic_map_schema.md`](docs/interfaces/semantic_map_schema.md)
- Roadmap: [`docs/roadmap/v2_5.md`](docs/roadmap/v2_5.md)
- Component status: [`docs/roadmap/component_status.md`](docs/roadmap/component_status.md)
- Historical records: [`docs/archive/README.md`](docs/archive/README.md)
