# AGENTS.md

## Scope

- Current stage: `AGT Navigation V2.5 Offline Asset & Route Preparation (V25-09A)`.
- P0 first BehaviorTree Mission software integration is complete; vehicle acceptance remains a separate gate.
- V25-08 architecture semantics are frozen. V25-09A now freezes and then implements the reproducible offline lineage from managed bag/calibration through map alignment, map products, semantic route derivation and vehicle-feasibility evidence before V25-09B Route runtime work.
- Delivery line: `V25-09A Offline Asset & Route Preparation -> V25-09B Robust Route Navigation -> V25-10 Sparse Global Correction -> V25-11 Local Environment Mapping -> V25-12 Optional ESDF`, with GNSS/Wheel/GTSAM as the P1 state-estimation track and STD/Scan Context/seasonal maps as P2 research.
- Historical migration/TASK records under `docs/archive/` are evidence only and are not current design instructions.

## Authoritative Documents

Architecture/interface work must stay consistent with:

- `docs/architecture/navigation_semantics.md` — canonical navigation concepts and ownership semantics
- `docs/architecture/system_architecture.md` — current runtime architecture and priority/status graph
- `docs/architecture/offline_asset_pipeline.md` — canonical offline asset/evaluation lineage
- `docs/architecture/runtime_dataflow.md` — implemented runtime chains; future target semantics must be labeled as not implemented
- `docs/interfaces/topic_contract.md` — canonical cross-module topic names/types/frames/owners and TF boundary
- `docs/interfaces/system_health.md` — health/readiness semantics
- `docs/interfaces/calibration_dataset_contract.md` — calibration and immutable Dataset/Bag provenance
- `docs/interfaces/map_derivation_contract.md` — reproducible map derivation, site alignment and quality evidence
- `docs/interfaces/map_manifest.md` — versioned map bundle and lineage binding
- `docs/interfaces/semantic_map_schema.md` — current semantic-map schema
- `docs/interfaces/semantic_waypoints.md` — named semantic anchor contract
- `docs/interfaces/route_asset_contract.md` — route rule, asset, tuning and footprint-feasibility contract
- `docs/interfaces/vehicle_tracker_adapter.md` — route/runtime-path to vehicle-controller adapter boundary
- `docs/workflows/bag_to_route_asset.md` — canonical bag-to-map-to-route workflow
- `docs/roadmap/v2_5.md` — active V2.5 delivery line
- directly affected package/interface documentation

Package-local debug or visualization topics may be documented by the owning package; they do not create a second canonical cross-module interface.

## Core Development Rules

- Change one module or one data chain at a time; prefer small reviewable commits/PRs.
- Do not hardcode usernames, workspace paths, device paths, or runtime map paths.
- Do not silently change validated datasets, calibration, map assets or experimental parameters.
- A formal offline derivation must bind immutable Dataset/Bag identity, Calibration Set, Platform Profile, Derivation Recipe and repository/dependency snapshot.
- Manual map cleaning, control-point alignment or route tuning that changes an accepted asset must be recorded as a reproducible patch/parameter artifact; unrecorded GUI edits cannot produce READY assets.
- READY map versions and READY route revisions are immutable. Re-alignment, cleaning, semantic edits or route tuning create a new version/revision.
- Do not create multiple publishers for the same authoritative TF edge.
- Do not bypass `agt_safety`, Collision Monitor, project Actions, or chassis watchdogs.
- Qt5 and Web are replaceable clients. They do not own system mode, Mission state, active-map state, localization truth, bag processes, safety state, route truth, or chassis state.
- A process being alive or a topic being discoverable is not readiness. Motion remains fail-closed on structured health/readiness/localization/safety evidence.
- `third_party/` provenance, pins, licenses, and patches must remain explicit; do not hide dependencies in an old sourced workspace.
- Architecture/contract stages must not silently introduce ROS runtime interfaces or implementation code.

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

## Calibration, Dataset and Experiment Contract

- `profiles/platforms/<platform>.yaml` remains the canonical source for vehicle geometry and kinematic limits; Calibration Set must not duplicate those values.
- Calibration Set owns sensor extrinsics, timing/lever-arm calibration and related measurement-model evidence. Re-calibration creates a new `calibration_id` and hash.
- Formal Dataset/Bag bindings must record `dataset_id`, `site_id`, `epoch_id`, purpose, bag hash, platform-profile hash and calibration identity/hash.
- `EVALUATION` datasets may use RTK/GNSS as independent truth. If GNSS is truth for an A/B estimator comparison, the same GNSS samples must not also update the estimator being evaluated.
- `OPERATIONAL` datasets produce facility/agricultural map and route assets; they may have no GNSS.
- Same-site different-season data keep the same `site_id` and stable site-frame definition while receiving new `epoch_id`, Dataset identity and map version.
- Different sites reuse the recipe/schema/tooling but do not copy site-specific control points or frame transforms.

## MID360 and URDF Self-Filter Contract

- Preserve raw MID360 `/agt/sensors/lidar/custom` for replay/audit.
- Normal FAST-LIVO2 input is `/agt/sensors/lidar/custom_filtered`.
- V2.5 default self-filter body geometry comes from `robot_description` URDF collision geometry.
- Self-filter may transform each point temporarily into `base_link`/collision-link coordinates for inside-geometry testing, but kept `CustomPoint`s must be copied from the original Livox message. Do not rewrite their XYZ, `offset_time`, `line`, `tag`, `reflectivity`, message frame, or ordering.
- Current realtime URDF self-filter supports explicit box/sphere/cylinder collision primitives. Mesh collision must fail closed until an explicit primitive proxy is provided; do not silently approximate it.
- `profiles/platforms/<platform>.yaml` remains the canonical platform physical/navigation contract and self-filter policy source. In URDF mode it supplies enable/padding and explicit supplemental boxes; its generated chassis body must not be applied in parallel with the URDF chassis collision.
- `geometry_source:=profile` is retained only as the legacy geometry A/B path. Full raw bypass remains an explicit baseline through `start_lidar_self_filter:=false`.
- Missing/invalid `robot_description` or required TF is fail-closed by default. Debug fail-open must be explicit.
- The pre-LIO self-filter does not replace the post-registration `agt_perception/local_obstacle_filter`.
- Platform-specific URDF/Xacro body dimensions must be contract-tested against the verified platform profile.

## Vehicle Geometry Contract

- `profiles/platforms/<platform>.yaml` is the canonical source for verified physical dimensions, navigation footprint, kinematic limits, and platform policy.
- Nav2, local obstacle filtering, coverage validation, offline route feasibility and description configuration must be contract-tested against the selected profile instead of copying independent values without checks.
- Route Policy and Tracker tuning must not create a second footprint, wheelbase, track-width or minimum-turning-radius truth source.
- Do not reuse the inflated navigation footprint as the self-filter chassis geometry or silently add a second safety margin.
- URDF collision is the runtime geometric representation used by the V2.5 self-filter; it must match the canonical physical dimensions rather than becoming a separate vehicle-dimension truth source.

## Offline Map Derivation Contract

- Mapping preserves raw bag evidence and produces bounded, versioned map products; READY map versions are immutable.
- Every formal map derivation must save an explicit `derivation/recipe.yaml` or hash-bound equivalent containing all parameters that affect output.
- All products in one map version must share one canonical site `map` frame derived through the same recorded alignment transform chain.
- Do not independently hand-shift PCD, OccupancyGrid, Semantic Map and Route assets while still claiming they are one version.
- `EVALUATION` sites may use recorded ENU georeference; operational facilities should use stable site control points/reference structure when GNSS is unavailable.
- Cross-season alignment should prioritize stable structures/control points rather than seasonal leaves, soft branches or temporary objects.
- Raw PCD must not be overwritten by cleaning. Voxel/crop/outlier/ground/stable-structure processing and manual cleanup patches must be replayable.
- Global Navigation Map, Localization Prior, Semantic Map and Local Environment Map are different products; do not use the generic word “map” to hide incompatible ownership or lifetime semantics.
- A localization PCD is accepted only with its ready processing record and matching content identity/hash where required.
- A formal derivation should emit `alignment_report.json` and `map_quality_report.json`; file/hash validity remains necessary but is not the only quality evidence.
- Relocalization is exposed through the project `/agt/localization/relocalize` Action boundary; manual/automatic request paths must converge on the same internal quality checks.
- Backend convergence alone does not mean localization acceptance; project quality gates remain authoritative.
- Tracking validation may report quality degradation but must not introduce a second `map -> odom` publisher.
- Future sparse correction/GTSAM/GNSS/loop modules produce evidence for the selected localization authority; they do not directly compete for TF ownership.

## Route Asset and Feasibility Contract

- A Route Asset is a versioned map-bound navigation asset, not a Mission, SemanticWaypoint Library, or Runtime Path.
- Route Asset must bind exact map-manifest, Semantic Map, Vehicle Platform Profile, Route Policy, route CSV and feasibility-report identities/hashes.
- Semantic derivation may use `row_centerline`, `access_lane`, `headland_zone`, keepout/exclusion, entry pose and SemanticWaypoint anchors according to an explicit Route Policy; source GeoJSON remains read-only.
- Route Policy may define clearance, reverse, unknown-space, direction-change and cost rules but must read vehicle geometry/kinematics from the selected platform profile.
- A centerline-only collision test is insufficient. READY Route Asset requires full navigation-footprint sweep and kinematic validation against occupancy + semantic restrictions.
- Preview must use the same canonical footprint used by validation and may show route centerline, F/R segments, sampled footprints, clearance hot spots, invalid poses and event anchors.
- Route tuning is non-destructive. Geometry changes create a new route revision/hash and must rerun sampling, footprint sweep, feasibility and preview generation.
- Route `event_ref` only references a business/semantic stop. Route Executor does not directly execute harvest, spray, capture or other task capabilities.

## Business Ownership

- `agt_map_manager` is the runtime owner of map-registry mutation and active-map publication.
- `agt_experiment_manager` is the runtime owner of rosbag record/playback processes.
- `agt_system_manager` owns structured health/readiness and system-mode management boundaries.
- `agt_mission_manager` is the single project Mission Action/state owner. It may sequence only project Actions and finite waits and must not publish velocity/TF, start arbitrary launch files, or infer success from distance/time.
- BehaviorTree.CPP is an execution backend behind `agt_mission_manager`, not a parallel Mission manager or state owner.
- `/agt/system/robot_state` is a read model; it does not become a business-state owner.

## Behavior Tree Contract

- BT nodes do not publish chassis/navigation velocity or TF.
- BT nodes do not implement mapping, localization, perception, planning, route derivation or control algorithms.
- BT Action nodes call project-owned Actions/Services, not Nav2 native Actions directly.
- BT Conditions consume structured machine-readable state such as `TaskReadiness`, `SystemHealth`, localization status, and safety status; they do not inspect raw sensor streams.
- Continuously running sensor/mapping/localization/perception/safety/chassis modules stay outside the tree.
- Parent cancellation must propagate to active child Actions and wait for child cancellation semantics before finalizing.

## Navigation Capability Contract

- Navigation is a project capability, not a synonym for Nav2.
- Current public waypoint execution boundary is `/agt/navigation/execute_waypoint_task` (`ExecuteWaypointTask`).
- Current MAP-oriented backend may internally dispatch Nav2 goals; Mission/BT/Qt/Web must not depend directly on `NavigateToPose`, `NavigateThroughPoses`, `FollowPath`, `/follow_waypoints`, or velocity topics.
- Target modes are `MAP`, `ROUTE`, and `LOCAL`. ROUTE/LOCAL are not implemented merely because they appear in architecture docs.
- Preserve `SemanticWaypoint != WaypointTask != Route != Runtime Path`.
- `WaypointTask/TaskGroup` is ordered business/navigation intent; Route is a resolved navigation representation; Runtime Path is controller-consumable geometry.
- V25-09B ROUTE runtime must consume only validated/compatible Route Asset revisions and normally track the active segment in `odom` using robust odometry.
- V25-09A/V25-09B do not add `ExecuteRouteTask`, `ExecuteNavigationTask`, `navigation_mode` Mission fields, or new speed topics unless a separately reviewed versioned interface change is approved.

## Vehicle Tracker Adapter Contract

- Runtime Path is adapted to a concrete vehicle/controller through the Vehicle Tracker Adapter boundary.
- Adapter inputs are Runtime Path, current odometry/pose, canonical Vehicle Profile, tracker tuning and local obstacle/cost evidence.
- Tracker tuning may contain controller parameters but must not duplicate vehicle geometry truth.
- Direction changes occur at Route Segment boundaries; policies requiring stop-before-reverse must be enforced explicitly.
- Tracker feedback should normalize active segment/path index, direction, cross-track error, heading error, remaining distance and failure state rather than leaking one controller plugin's private API upward.
- Offline route feasibility proves static geometric/kinematic validity; it does not guarantee runtime tracking success.

## Navigation, Safety, and Chassis Contract

- Current MAP runtime motion chain remains `Nav2 controller -> /agt/navigation/cmd_vel_raw -> collision/safety -> /agt/navigation/cmd_vel -> /agt/safety/cmd_vel -> /agt/chassis/cmd_vel`.
- Future ROUTE/LOCAL backends may reuse or replace internal planner/path-follower/controller components, but they must enter the same project safety/chassis boundary and must not create a second final command path.
- Parent cancellation, safety loss, mode-specific readiness loss, map/task identity mismatch, route/profile mismatch, or backend abort are terminal failures unless a bounded explicit recovery contract says otherwise.
- `start_chassis` defaults off for disconnected/offline testing. Monitor-only CAN mode must not create a command path.
- Host CAN provisioning remains an administrator boundary; ROS/Web code does not run privileged network setup commands.

## Readiness Contract

- Current `EvaluateTaskReadiness.srv` remains unchanged in the architecture baseline; current `TASK_EXECUTION` and `RELOCALIZATION` profiles mainly serve the MAP-oriented baseline.
- Future mode-aware concepts are `MAP_START_READY`, `MAP_CONTINUE_READY`, `ROUTE_START_READY`, `ROUTE_CONTINUE_READY`, `GLOBAL_CORRECTION_READY`, and `LOCAL_READY`.
- `ROUTE_CONTINUE_READY` must prioritize odometry, local control, safety and required local perception; it must not require a recent global correction on every control cycle.
- `GLOBAL_CORRECTION_READY` independently gates relocalization/sparse global correction.
- `LOCAL_READY` must not require a Global Navigation Map.
- Do not add these names to `.srv`/`.msg` until a separately reviewed versioned interface change is implemented and tested.

## Semantic and Coverage Contract

- Semantic geometry stays separate from the base PGM/OccupancyGrid and uses versioned GeoJSON/sidecar data.
- Keepout cost is a reversible Nav2 filter product; never bake it back into the source PGM.
- Semantic-map/coverage launch switches remain disabled by default until explicitly requested.
- Raw coverage/route candidates are never executable. Only paths that pass the current semantic, full-footprint, unknown-space, kinematic, map-binding and safety contracts may reach an execution boundary.
- `row_centerline`, `access_lane`, `headland_zone`, keepout/exclusion and SemanticWaypoint may be consumed by an offline Route Policy, but the derivation must not rewrite the source semantic GeoJSON.
- Offline previews, metrics, comparisons, repair/tuning candidates do not imply execution approval.
- Current schema behavior must not be changed merely because a future waypoint/route extension is planned; schema changes require explicit versioned interface work and tests.

## Frontend Contract

- Qt/Web call generated project interfaces and display machine-readable feedback/results.
- Frontends do not inspect manager internals, reconstruct runtime asset paths, or duplicate Mission/map/experiment ownership.
- READY map assets and READY route revisions are read-only; edits create a new version/revision rather than mutating accepted evidence.
- Offline route preview/tuning may visualize semantic layers, real vehicle footprint, clearance and invalid poses, but it does not start control or become a business-state owner.
- Planner previews are advisory and must not start control, safety enablement, localization ownership, TF publishers, or chassis commands.

## Testing and Acceptance

Every architecture/interface change must include the smallest relevant automated contract tests. V25-09A contract tests must at minimum protect:

- immutable Dataset/Bag + Calibration + Platform + Recipe lineage
- `EVALUATION` versus `OPERATIONAL` purpose and RTK truth isolation
- `site_id` / `epoch_id` semantics for cross-season reproducibility
- one canonical site frame per map version and non-destructive map cleaning
- map/semantic/vehicle/policy hash binding for Route Asset
- full-footprint and kinematic feasibility before Route READY
- non-destructive route tuning with new revision/revalidation
- Vehicle Tracker Adapter as an adapter, not a planning/TF/Mission owner
- absence of new public `ExecuteRouteTask`/`ExecuteNavigationTask` interfaces unless separately reviewed

V25-08 architecture tests continue to protect:

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

Record removal ratio, debug geometry/removed points, CPU/filter latency, FAST-LIVO2 trajectory, final PCD, self-return residue, false removals near the vehicle, and relocalization impact. Vehicle validation remains separate from code/build success.

## Documentation and Archive Rule

- Current architecture/interface statements belong under `docs/architecture`, `docs/interfaces`, `docs/roadmap`, and canonical `docs/workflows`.
- Historical Phase/TASK/migration/experiment evidence belongs under `docs/archive` or dedicated experiment/calibration/testing records.
- Archive text may preserve historical names and decisions; it must not be treated as current runtime truth.
