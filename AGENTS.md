# AGENTS.md

## Scope

- Current stage: `AGT Navigation V2.5 Mission Architecture Integration`.
- Primary target: complete the first production-style BehaviorTree mission by composing existing
  project Actions and readiness contracts without bypassing localization, Nav2, safety, TF ownership,
  chassis boundaries, or business-manager ownership.
- Delivery line: `V2.5 -> P0 First BT Mission -> P1 Robust Localization / Local Mapping -> P2 Long-term Agricultural Navigation Research`.
- Historical migration/TASK records under `docs/archive/` are evidence only and are not current design instructions.

## Authoritative Documents

Architecture/interface work must stay consistent with:

- `docs/architecture/system_architecture.md` — current system architecture and priority/status graph
- `docs/architecture/runtime_dataflow.md` — implemented runtime chains
- `docs/interfaces/topic_contract.md` — canonical cross-module topic names/types/owners
- `docs/interfaces/system_health.md` — health/readiness semantics
- `docs/interfaces/semantic_map_schema.md` — current semantic-map schema
- `docs/roadmap/v2_5.md` — active V2.5 delivery line
- directly affected package/interface documentation

Package-local debug or visualization topics may be documented by the owning package; they do not create a second canonical cross-module interface.

## Core Development Rules

- Change one module or one data chain at a time; prefer small reviewable commits/PRs.
- Do not hardcode usernames, workspace paths, device paths, or runtime map paths.
- Do not silently change validated datasets or experimental parameters.
- Do not create multiple publishers for the same authoritative TF edge.
- Do not bypass `agt_safety`, Collision Monitor, project Actions, or chassis watchdogs.
- Qt5 and Web are replaceable clients. They do not own system mode, Mission state, active-map state,
  localization truth, bag processes, safety state, or chassis state.
- A process being alive or a topic being discoverable is not readiness. Motion remains fail-closed on
  structured health/readiness/localization/safety evidence.
- `third_party/` provenance, pins, licenses, and patches must remain explicit; do not hide dependencies in an old sourced workspace.

## Canonical Topic and TF Contract

- `/agt/mapping/registered_points` is the only canonical registered-cloud topic.
- `registered_cloud`, `/agt/mapping/registered_points_lidar`, and `/agt/mapping/registered_cloud` are historical names and must not be reintroduced into runtime code/config.
- `agt_localization` owns authoritative `map -> odom` correction.
- `agt_mapping_fast_livo2_adapter` owns `odom -> base_footprint`.
- `robot_state_publisher` owns the robot/sensor description chain below `base_footprint`.
- FAST-LIVO2 backend TF output and chassis odom TF must remain disabled when those edges are already owned by project adapters.

## MID360 and URDF Self-Filter Contract

- Preserve raw MID360 `/agt/sensors/lidar/custom` for replay/audit.
- Normal FAST-LIVO2 input is `/agt/sensors/lidar/custom_filtered`.
- V2.5 default self-filter body geometry comes from `robot_description` URDF collision geometry.
- Self-filter may transform each point temporarily into `base_link`/collision-link coordinates for
  inside-geometry testing, but kept `CustomPoint`s must be copied from the original Livox message.
  Do not rewrite their XYZ, `offset_time`, `line`, `tag`, `reflectivity`, message frame, or ordering.
- Current realtime URDF self-filter supports explicit box/sphere/cylinder collision primitives. Mesh
  collision must fail closed until an explicit primitive proxy is provided; do not silently approximate it.
- `profiles/platforms/<platform>.yaml` remains the canonical platform physical/navigation contract and
  self-filter policy source. In URDF mode it supplies enable/padding and explicit supplemental boxes;
  its generated chassis body must not be applied in parallel with the URDF chassis collision.
- `geometry_source:=profile` is retained only as the legacy geometry A/B path. Full raw bypass remains an
  explicit baseline through `start_lidar_self_filter:=false`.
- Missing/invalid `robot_description` or required TF is fail-closed by default. Debug fail-open must be explicit.
- The pre-LIO self-filter does not replace the post-registration `agt_perception/local_obstacle_filter`.
- Platform-specific URDF/Xacro body dimensions must be contract-tested against the verified platform profile.

## Vehicle Geometry Contract

- `profiles/platforms/<platform>.yaml` is the canonical source for verified physical dimensions,
  navigation footprint, kinematic limits, and platform policy.
- Nav2, local obstacle filtering, coverage validation, and description configuration must be
  contract-tested against the selected profile instead of copying independent values without checks.
- Do not reuse the inflated navigation footprint as the self-filter chassis geometry or silently add a
  second safety margin.
- URDF collision is the runtime geometric representation used by the V2.5 self-filter; it must match the
  canonical physical dimensions rather than becoming a separate vehicle-dimension truth source.

## Business Ownership

- `agt_map_manager` is the runtime owner of map-registry mutation and active-map publication.
- `agt_experiment_manager` is the runtime owner of rosbag record/playback processes.
- `agt_system_manager` owns structured health/readiness and system-mode management boundaries.
- `agt_mission_manager` is the single project Mission Action/state owner. It may sequence only project
  Actions and finite waits and must not publish velocity/TF, start arbitrary launch files, or infer success from distance/time.
- The P0 BehaviorTree.CPP/Groot2 work is an execution engine behind `agt_mission_manager`, not a second
  parallel Mission manager or state owner.
- `/agt/system/robot_state` is a read model; it does not become a business-state owner.

## Behavior Tree Contract

- BT nodes do not publish chassis/navigation velocity or TF.
- BT nodes do not implement mapping, localization, perception, planning, or control algorithms.
- BT Action nodes call project-owned Actions/Services, not Nav2 native Actions directly.
- BT Conditions consume structured machine-readable state such as `TaskReadiness`, `SystemHealth`,
  localization status, and safety status; they do not inspect raw sensor streams.
- Continuously running sensor/mapping/localization/perception/safety/chassis modules stay outside the tree.
- Parent cancellation must propagate to active child Actions and wait for child cancellation semantics before finalizing.

## Mapping and Localization Contract

- Mapping preserves raw bag evidence and produces bounded, versioned map products; READY map versions are immutable.
- A localization PCD is accepted only with its ready processing record and matching content identity/hash where required.
- Relocalization is exposed through the project `/agt/localization/relocalize` Action boundary; manual/automatic request paths must converge on the same internal quality checks.
- Backend convergence alone does not mean localization acceptance; project quality gates remain authoritative.
- Tracking validation may report quality degradation but must not introduce a second `map -> odom` publisher.
- Navigation Actions must reject stale or invalid localization according to the configured structured status contract.

## Navigation, Safety, and Chassis Contract

- Formal waypoint execution uses the project `ExecuteWaypointTask` Action and robot-side task identity.
- Project navigation may dispatch Nav2 goals, but frontends and Mission logic do not publish velocity.
- Motion output remains `Nav2 -> collision/safety chain -> agt_safety -> agt_chassis`.
- Parent cancellation, safety loss, localization loss, map mismatch, missed waypoints, or Nav2 abort are terminal failures unless a bounded explicit recovery contract says otherwise.
- `start_chassis` defaults off for disconnected/offline testing. Monitor-only CAN mode must not create a command path.
- Host CAN provisioning remains an administrator boundary; ROS/Web code does not run privileged network setup commands.

## Semantic and Coverage Contract

- Semantic geometry stays separate from the base PGM/OccupancyGrid and uses versioned GeoJSON/sidecar data.
- Keepout cost is a reversible Nav2 filter product; never bake it back into the source PGM.
- Semantic-map/coverage launch switches remain disabled by default until explicitly requested.
- Raw coverage candidates are never executable. Only paths that pass the current semantic, full-footprint,
  unknown-space, kinematic, map-binding, and safety contracts may reach an execution boundary.
- Offline previews, metrics, comparisons, and repair candidates do not imply execution approval.
- Current schema behavior must not be changed merely because a future V2.5 waypoint extension is planned;
  schema changes require explicit versioned interface work and tests.

## Frontend Contract

- Qt/Web call generated project interfaces and display machine-readable feedback/results.
- Frontends do not inspect manager internals, reconstruct runtime asset paths, or duplicate Mission/map/experiment ownership.
- READY map assets are read-only in navigation/offline profiles; edits create a new version rather than mutating an active version.
- Planner previews are advisory and must not start control, safety enablement, localization ownership, TF publishers, or chassis commands.

## Testing and Acceptance

Every architecture/interface change must include the smallest relevant automated contract tests. For the
URDF self-filter specifically, keep tests for:

- URDF primitive parsing and mesh rejection
- profile supplemental-box separation
- platform-profile vs description body-size consistency
- preservation of Livox per-point fields/order
- canonical topic/TF contracts

Before declaring the URDF self-filter DONE, compare the same raw bag in three modes:

1. raw / self-filter disabled
2. `geometry_source:=profile`
3. `geometry_source:=urdf`

Record removal ratio, debug geometry/removed points, CPU/filter latency, FAST-LIVO2 trajectory, final PCD,
self-return residue, false removals near the vehicle, and relocalization impact. Vehicle validation remains separate from code/build success.

## Documentation and Archive Rule

- Current architecture/interface statements belong under `docs/architecture`, `docs/interfaces`, and `docs/roadmap`.
- Historical Phase/TASK/migration/experiment evidence belongs under `docs/archive` or dedicated experiment/calibration/testing records.
- Archive text may preserve historical names and decisions; it must not be treated as current runtime truth.
