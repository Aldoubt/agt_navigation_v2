# AGENTS.md

## Scope

- Current stage: `AGT Navigation V2.5 Architecture & Semantics Baseline (V25-08)`.
- P0 first BehaviorTree Mission software integration is complete; vehicle acceptance remains a separate gate.
- Primary target: freeze Navigation Capability, MAP/ROUTE/LOCAL semantics, map-product roles, TF/state-estimation ownership, route/task/path semantics, and readiness boundaries before V25-09 runtime work.
- Delivery line: `V25-08 Architecture & Semantics -> V25-09 Robust Route Navigation -> V25-10 Sparse Global Correction -> V25-11 Local Environment Mapping -> V25-12 Optional ESDF`, with GNSS/Wheel/GTSAM as the P1 state-estimation track and STD/Scan Context/seasonal maps as P2 research.
- Historical migration/TASK records under `docs/archive/` are evidence only and are not current design instructions.

## Authoritative Documents

Architecture/interface work must stay consistent with:

- `docs/architecture/navigation_semantics.md` — V25-08 canonical navigation concepts and ownership semantics
- `docs/architecture/system_architecture.md` — current system architecture and priority/status graph
- `docs/architecture/runtime_dataflow.md` — implemented runtime chains; future target semantics must be labeled as not implemented
- `docs/interfaces/topic_contract.md` — canonical cross-module topic names/types/frames/owners and TF boundary
- `docs/interfaces/system_health.md` — health/readiness semantics
- `docs/interfaces/semantic_map_schema.md` — current semantic-map schema
- `docs/interfaces/semantic_waypoints.md` — named semantic anchor contract
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
- Architecture-only stages must not silently introduce ROS runtime interfaces or implementation code.

## Canonical Topic and TF Contract

- `/agt/mapping/registered_points` is the only canonical registered-cloud topic and is in `odom`.
- `registered_cloud`, `/agt/mapping/registered_points_lidar`, and `/agt/mapping/registered_cloud` are historical names and must not be reintroduced into runtime code/config.
- `/agt/map/global_occupancy` is persistent global navigation geometry in `map`; dynamic/semantic layers must not be baked back into it.
- `/agt/map/local_occupancy` is reserved for a transient rolling local-environment `OccupancyGrid` in `odom`; it is not versioned global-map truth and currently has no formal runtime publisher.
- `/agt/map/waypoints` is a persistent SemanticWaypoint anchor library, not an execution sequence, Route, or Runtime Path.
- `agt_mapping_fast_livo2_adapter` uniquely owns `odom -> base_footprint`.
- authoritative `map -> odom` belongs to the localization subsystem and must have exactly one selected runtime publisher.
- Current baseline: `agt_localization` package / `agt_relocalization` node publishes `map -> odom` with `publish_tf=true`.
- Future fusion ownership is replacement, not addition: disable the baseline TF publisher before allowing `agt_localization_fusion` or another approved localization owner to publish the edge.
- NDT/ICP, GTSAM/iSAM2, GNSS factors, loop closure and place recognition may provide correction evidence/factors/candidates but must not become parallel `map -> odom` publishers.
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
- BehaviorTree.CPP is an execution backend behind `agt_mission_manager`, not a parallel Mission manager or state owner.
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
- Global Navigation Map, Localization Prior, Semantic Map and Local Environment Map are different products; do not use the generic word “map” to hide incompatible ownership or lifetime semantics.
- A localization PCD is accepted only with its ready processing record and matching content identity/hash where required.
- Relocalization is exposed through the project `/agt/localization/relocalize` Action boundary; manual/automatic request paths must converge on the same internal quality checks.
- Backend convergence alone does not mean localization acceptance; project quality gates remain authoritative.
- Tracking validation may report quality degradation but must not introduce a second `map -> odom` publisher.
- Future sparse correction/GTSAM/GNSS/loop modules produce evidence for the selected localization authority; they do not directly compete for TF ownership.

## Navigation Capability Contract

- Navigation is a project capability, not a synonym for Nav2.
- Current public waypoint execution boundary is `/agt/navigation/execute_waypoint_task` (`ExecuteWaypointTask`).
- Current MAP-oriented backend may internally dispatch Nav2 goals; Mission/BT/Qt/Web must not depend directly on `NavigateToPose`, `NavigateThroughPoses`, `FollowPath`, `/follow_waypoints`, or velocity topics.
- V25-08 defines three target modes: `MAP`, `ROUTE`, and `LOCAL`. ROUTE/LOCAL are not implemented merely because they appear in architecture docs.
- Preserve `SemanticWaypoint != WaypointTask != Route != Runtime Path`.
- `WaypointTask/TaskGroup` is ordered business/navigation intent; Route is a resolved internal navigation representation; Runtime Path is controller-consumable geometry.
- ROUTE should normally track the active segment in `odom` using robust odometry and request sparse global correction only on explicit anchor/confidence/recovery conditions.
- V25-08 does not add `ExecuteRouteTask`, `ExecuteNavigationTask`, `navigation_mode` Mission fields, or new speed topics. Any such change requires separate versioned interface work.

## Navigation, Safety, and Chassis Contract

- Current MAP runtime motion chain remains `Nav2 controller -> /agt/navigation/cmd_vel_raw -> collision/safety -> /agt/navigation/cmd_vel -> /agt/safety/cmd_vel -> /agt/chassis/cmd_vel`.
- Future ROUTE/LOCAL backends may reuse or replace internal planner/path-follower/controller components, but they must enter the same project safety/chassis boundary and must not create a second final command path.
- Parent cancellation, safety loss, mode-specific readiness loss, map/task identity mismatch, or backend abort are terminal failures unless a bounded explicit recovery contract says otherwise.
- `start_chassis` defaults off for disconnected/offline testing. Monitor-only CAN mode must not create a command path.
- Host CAN provisioning remains an administrator boundary; ROS/Web code does not run privileged network setup commands.

## Readiness Contract

- Current `EvaluateTaskReadiness.srv` remains unchanged in V25-08; current `TASK_EXECUTION` and `RELOCALIZATION` profiles mainly serve the MAP-oriented baseline.
- Future mode-aware concepts are `MAP_START_READY`, `MAP_CONTINUE_READY`, `ROUTE_START_READY`, `ROUTE_CONTINUE_READY`, `GLOBAL_CORRECTION_READY`, and `LOCAL_READY`.
- `ROUTE_CONTINUE_READY` must prioritize odometry, local control, safety and required local perception; it must not require a recent global correction on every control cycle.
- `GLOBAL_CORRECTION_READY` independently gates relocalization/sparse global correction.
- `LOCAL_READY` must not require a Global Navigation Map.
- Do not add these names to `.srv`/`.msg` until a separately reviewed versioned interface change is implemented and tested.

## Semantic and Coverage Contract

- Semantic geometry stays separate from the base PGM/OccupancyGrid and uses versioned GeoJSON/sidecar data.
- Keepout cost is a reversible Nav2 filter product; never bake it back into the source PGM.
- Semantic-map/coverage launch switches remain disabled by default until explicitly requested.
- Raw coverage candidates are never executable. Only paths that pass the current semantic, full-footprint,
  unknown-space, kinematic, map-binding, and safety contracts may reach an execution boundary.
- Offline previews, metrics, comparisons, and repair candidates do not imply execution approval.
- Current schema behavior must not be changed merely because a future waypoint/route extension is planned;
  schema changes require explicit versioned interface work and tests.

## Frontend Contract

- Qt/Web call generated project interfaces and display machine-readable feedback/results.
- Frontends do not inspect manager internals, reconstruct runtime asset paths, or duplicate Mission/map/experiment ownership.
- READY map assets are read-only in navigation/offline profiles; edits create a new version rather than mutating an active version.
- Planner previews are advisory and must not start control, safety enablement, localization ownership, TF publishers, or chassis commands.

## Testing and Acceptance

Every architecture/interface change must include the smallest relevant automated contract tests. V25-08 architecture tests must at minimum protect:

- `MAP`, `ROUTE`, `LOCAL` mode names and their implemented/reserved boundary
- `SemanticWaypoint != WaypointTask != Route != Runtime Path`
- `/agt/map/local_occupancy` as an `odom`-frame transient rolling product
- ESDF as optional/derived, not a default prerequisite
- one selected authoritative `map -> odom` publisher at a time
- unique `odom -> base_footprint` ownership
- project Navigation Capability above Nav2 native interfaces

For the URDF self-filter specifically, keep tests for:

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
