# AGENTS.md

## Scope
- Current stage: `Bunker Qt5 FAST-LIO Navigation Baseline Integration`.
- Integrate one Bunker baseline data chain at a time: MID360/FAST-LIVO2 mapping,
  2D map production, Qt5 operation, map persistence, PCD relocalization, Nav2,
  safety, and chassis output.
- Semantic-map, Keepout, Fields2Cover, and coverage-planning code stays present but
  is not developed or refactored in this stage.

## Hard Rules
- Migrate one module or one data chain at a time.
- Do not hardcode usernames, workspace paths, device paths, or map paths.
- Do not allow multiple nodes to publish the same TF edge.
- `third_party/ros_qt5_gui_app` is a fixed vendored snapshot of the
  `Aldoubt/Ros_Qt5_Gui_App:agt-navigation-v2` integration branch. Project
  changes are allowed there, must preserve GPL-2.0 attribution, must be pushed
  to that fork, and must update the pinned provenance in `third_party/README.md`.
- Do not bypass `agt_safety` or publish GUI/navigation velocity directly to the
  Bunker driver.
- Semantic-map and coverage-planning launch arguments must default to `false`.
- Do not modify validated parameters or datasets from the legacy repository without explicit approval.
- ROS 2 managers are the only real business backend. Qt5 and Web are clients and must not own
  system mode, mission, active-map, experiment, bag-process, safety, or chassis state.
- `agt_map_manager` is the only runtime owner of `MapRegistry` mutations and active-map
  publication. Managed mapping imports, activates, and cleans failed versions through
  `/agt/maps/manage`; consumers must use the returned asset paths instead of reconstructing them.
- `agt_experiment_manager` is the only runtime owner of rosbag record/playback processes.
  Managed mapping creates an experiment and starts the explicit `mapping` profile through
  `/agt/data/bags/manage`; the legacy launch recorder remains compatibility-only.
- `agt_mission_manager` may sequence only project Actions and finite waits. It must not call Nav2
  native Actions, start launch files, publish velocity/TF, or infer completion from distance or time.
- `/agt/system/robot_state` is a read model, not a new business owner. Unknown or stale evidence
  remains UNKNOWN, and active map identity comes only from `agt_map_manager`.
- Every change that affects architecture or interfaces must update `docs/`, this file, and `docs/migration/migration_matrix.md`.
- The raw MID360 `/agt/sensors/lidar/custom` topic is preserved. In the normal baseline FAST-LIVO2
  consumes only the profile-driven `/agt/sensors/lidar/custom_filtered` output from
  `agt_livox_self_filter`; an explicit A/B baseline may disable the filter and fall back to raw input.
  Zero placeholder points are rejected before TF geometry checks. The filter does not publish TF or
  replace the post-registration `agt_perception/local_obstacle_filter`.

## Vehicle Geometry Contract
- `profiles/platforms/<platform>.yaml` is the canonical source for vehicle geometry.
- Nav2, perception, and future coverage validation must consume or be contract-tested against the selected platform profile.
- Do not maintain a separate coverage-planning footprint or silently add a second safety margin.
- Changes to verified vehicle dimensions require explicit approval and corresponding contract-test updates.
- `greenhouse_ackermann` uses user-provided outer dimensions and a 1.5 m rear-axle-center turning radius; its geometric-center base reference and motion limits remain provisional until measured on the vehicle.

## Semantic Map Contract
- Semantic geometry uses `frame_id: map`, metric coordinates, and ROS right-handed axes.
- The versioned contract is `docs/interfaces/semantic_map_schema.md` plus `src/agt_ui_bridge/config/semantic_schema.yaml`.
- Keep semantic GeoJSON separate from the base OccupancyGrid; never write semantic zones into the source PGM.
- CloudCompare raster packaging for Qt5 annotation may create a provisional `runtime/maps/<map_id>/` Nav2 map bundle, but it must keep any missing Rasterize origin metadata explicit in `processing_record.yaml` and must not be presented as closed-loop ready until that metadata is recorded.
- The project-owned semantic editor may edit only the existing Nav2 image referenced by the selected YAML through explicit trinary freehand/line tools; it must not merge semantic objects back into the base raster.
- Semantic polygon authoring must reject self-intersecting completion without discarding the repairable draft, and completed geometry plus base-map strokes must remain undoable.
- Semantic tasks with repairable feature-geometry errors must open editable while save and preview remain fail-closed; document identity, frame, schema, missing-map, and base-map hash failures remain read-only.
- Semantic-editor route previews may derive temporary aisle centerlines between adjacent crop-row centerlines, but must not rewrite source GeoJSON, publish executable paths, or enable motion; the chosen row interpretation must be explicit in the UI.
- Optional `access_lane` LineStrings represent separate open, simple, non-backtracking drivable centerlines, remain separate from `work_direction`, and may be combined with derived crop-row aisles only in annotated-row planning; disabled lanes and source GeoJSON must remain unchanged. Temporary GML must equalize row point counts with at least three points for the locked OpenNav `ROWSARESWATHS` width defect. The adapter may transactionally restore SWATH endpoints only from the exact validated annotated-row request to compensate the locked PathComponents N-1 conversion; it must not infer SWATH geometry from heading.
- Semantic-editor offline preview processes and subscriptions must share a dedicated per-editor ROS domain so stale preview nodes cannot contaminate the live robot graph or a later preview result.
- Production annotated-row adaptation must honor `coverage.yaml:row_interpretation`; `crop_centerlines` derives adjacent aisle midlines transactionally, while missing/`direct_swaths` preserves the version-1 legacy behavior.
- Versioned examples live under `docs/interfaces/examples/`; runtime semantic files live under `runtime/maps/<map_id>/semantic/` and are not committed.
- Schema contract completion does not imply that coverage path repair or coverage execution is implemented.
- Project-owned semantic data classes and file I/O under `agt_ui_bridge` are data-contract code, not third-party semantic algorithm migration.
- Keep map transforms and semantic file logic free of Qt and ROS dependencies so they remain independently testable.
- The semantic editor treats the base PGM/YAML as read-only and writes only versioned GeoJSON plus `coverage.yaml`.
- Keep the project-owned semantic editor separate from `third_party/ros_qt5_gui_app`; do not patch vendor UI code for semantic authoring.
- Semantic topology and containment checks must use Shapely/GEOS; do not replace them with project-owned polygon Boolean algorithms.
- Footprint feasibility consumes `navigation_footprint` from the selected platform profile. Any extra boundary clearance must be explicit and defaults to zero.
- The semantic map server uses standard ROS messages/services and transactional candidate loading; a failed load must not replace or clear the last valid products.
- TASK-06 keepout masks rasterize enabled exclusion/keepout zones and configurable field exterior without modifying the base map.
- Keepout masks must preserve the base OccupancyGrid metadata exactly.
- TASK-07 connects only `/agt/map/keepout_mask` to the global Nav2 costmap through a type-0 FilterInfo server.
- Keep global costmap ordering as `StaticLayer -> KeepoutFilter -> InflationLayer`; do not add the semantic mask to the local obstacle chain.
- Keepout costs are reversible filter state. Never write them into `/agt/map/global_occupancy` or the source PGM.
- Humble KeepoutFilter is fail-open before FilterInfo/mask arrives; motion procedures must verify semantic status `LOADED` instead of treating node liveness as readiness.

## Coverage Dependency Contract
- ROS 2 Humble coverage dependencies are locked by full commit SHA in `nav_dependencies.repos`.
- `opennav_coverage` uses the `humble-v2` line and Fields2Cover uses `v2.0.0`; do not mix the ordinary Humble/F2C 1.2.1 line with this contract.
- Keep coverage algorithm sources in an external vcs workspace. Do not vendor them into `agt_coverage_planning`.
- Build Fields2Cover with `USE_ORTOOLS_VENDOR=ON` and the pinned `FETCHCONTENT_SOURCE_DIR_*` inputs documented in `docs/development/coverage_dependencies.md`; no build-time network fallback or hidden old-workspace overlay is allowed.
- TASK-09 depends on Coverage Server and `ComputeCoveragePath`, but not on Coverage Navigator, BT plugins, or demos.
- Coverage requests must pass complete semantic validation and canonical platform-profile snapshot checks before changing server parameters or sending a goal.
- Coverage Server launch must inject canonical `robot_width` and `min_turning_radius` before lifecycle configure; the locked Humble dynamic callback does not update the internal robot turning radius.
- Humble annotated-row requests use process-private generated GML because Row Coverage Server has no in-message row input; never write generated GML into semantic source files.
- `/agt/coverage/path_raw` is never executable; only TASK-10 may publish a non-empty `/agt/coverage/path_validated` after all current checks pass.
- Offline coverage preview may additionally compose a Nav2 planner_server with a static global costmap for preview-only Hybrid-A*/State Lattice CONNECTION repair. It must use explicit request start poses without publishing TF, force execution disabled, and must not start localization, BT navigation, Nav2 control, safety, or chassis nodes.
- Entry approach remains a separate preview-only path from the enabled `entry_pose` to the first repaired coverage pose. It must use the profile repair planner and the same costmap/keepout/full-footprint validation, and must not be inserted into authoritative SWATH/CONNECTION semantics or executable outputs.
- `/agt/coverage/path_preview` may expose a basically valid Coverage Server path for visualization when component semantics fail. It must be cleared before each request and must never feed validation, repair, execution, Nav2, or chassis control.
- An offline preview auditor may consume `path_preview`, the static base OccupancyGrid, semantic keepout mask, and canonical footprint only to publish advisory JSON and collision visualization. It must always report `eligible_for_execution=false`, must not publish any Path, and is not TASK-10 validation or execution evidence.
- Offline time simulation may consume preview or semantic coverage paths and canonical platform motion limits, but it is metrics-only: it must not publish TF, velocity, Nav2 goals, safety commands, or chassis commands. Missing/mismatched path semantics must be reported as geometric fallback, never fabricated as work/non-work metrics.
- Offline variant comparison may call one Coverage Server sequentially for multiple route/path/angle candidates, but it may publish only visualization markers, diagnostics, and metrics reports. It must not publish any candidate as `nav_msgs/Path` or feed validation, repair, execution, Nav2, safety, or chassis control.
- Geometric candidate ranking is never execution approval. Coverage and overlap metrics may be computed only after the authoritative PathComponents pass the complete TASK-11 semantic reconstruction contract; otherwise those fields must remain null and every candidate must remain `eligible_for_execution=false`.

## Relocalization Runtime Contract
- `ndt_num_threads` must be a positive integer. The validated Bunker baseline is `4`; zero and negative values must be rejected at the ROS parameter boundary and clamped defensively before reaching NDT-OMP.
- NDT-OMP must never size per-thread work buffers from an unchecked thread count.
- A successful field trial and low fitness score do not replace parameter-boundary regression tests or validation with the map PCD used for navigation.
- The generated `agt_interfaces/msg/LocalizationStatus` and `agt_interfaces/action/Relocalize` are the machine-readable localization contract. The legacy string status topic remains human-facing compatibility only.
- `has_converged` is backend convergence; `localization_accepted`/`pose_valid` require project quality validation and may not be inferred from fitness alone.
- `/agt/localization/relocalize` is the only project relocalization Action boundary; `/initialpose` must be adapted into the same internal request path.
- Baseline TF ownership remains unique: `agt_localization` publishes `map -> odom`, FAST-LIVO2 adapter publishes `odom -> base_footprint`, and description publishes robot/sensor frames.
- The localization supervisor owns structured `TRACKING/DEGRADED/RECOVERING/LOST` transitions. Low-frequency tracking validation may publish status but must not rewrite `map -> odom`; a lost state waits for an explicit recovery request.
- Startup automatic relocalization, when explicitly enabled, may send only one bounded project Action request. It must not publish velocity, enable motion, bypass Nav2/safety, or retry without a new explicit recovery request.
- `agt_navigation` must reject waypoint Actions without a fresh accepted `LocalizationStatus`; `agt_safety` must independently fail-closed for navigation input while preserving manual priority.
- The baseline `LocalizationStatus` tracking-validation period is 5 s with a 3 s validation timeout. Safety, the Nav2 localization gate, the waypoint Action, and the health contract must use a freshness window of at least 10 s; `TaskReadiness` retains its own short snapshot timeout. `agt_safety` diagnostics are authoritative for `emergency_stop`/`estop_latched` and `navigation_ready`; a missing optional `/agt/safety/emergency_stop` publisher must not fabricate an active stop after the controller reports a clear latch.
- `agt_localization` uses the fixed registered-cloud timestamp for dynamic TF, rejects invalid/stale/future
  clouds with explicit status errors, and tracking validation seeds registration from
  `map -> odom * odom -> tracking_frame` without rewriting `map -> odom` or the last accepted pose.
- Tracking validation skips a duplicate registered-cloud stamp only while that cached cloud is still fresh.
  A stale, future, or invalid duplicate is rejected and counts as one validation failure; a fresh ROS-time
  rollback baseline is skipped and becomes the new sequence baseline without changing supervisor counters.
- `runCandidates()` must not publish authoritative intermediate status while tracking validation is active.
  The outer tracking worker applies each non-skipped result to the supervisor exactly once and publishes one
  final status; skipped results publish no status and do not change success or failure counters.
- `tracking_confirmations_required` currently supports only `1`. Non-`1` values must fail at node startup;
  multi-frame bootstrap confirmation requires a separate future contract and must not be inferred from the
  existing supervisor capability.

## Realtime Traversability Resource Contract
- `bunker_realtime_traversability_provisional.yaml` is a non-running design candidate until a bounded runtime node, diagnostics, persistence, and bag regression exist; it must remain disabled by default.
- Temporal static evidence must never delay or replace the immediate Nav2 local obstacle layer.
- Active cells, active tiles, and process memory must have positive finite limits. A tile may be evicted only after successful persistence; persistence failure is fail-closed and diagnostic.
- Ground residual thresholds do not automatically follow URDF extrinsic changes. Recalibration requires threshold revalidation on an independent bag.

## Static Navigation Map Contract
- Offline obstacle completion must preserve a ray-traced free/unknown baseline and add only repeatable registered-cloud obstacle evidence; a raw point projection must not silently turn unknown space into free space.
- Managed mapping sessions treat the saved online OctoMap PGM/YAML as an
  `online_preview` input, never as the final editable candidate. After normal
  PCD/bag shutdown, the offline producer must rebuild bounded 2D free-space rays
  from timestamp-matched registered clouds and the recorded sensor origin, then
  overlay the conservative `ground_temporal` evidence and complete polygon sweep.
  Added canvas area remains unknown until crossed by a reconstructed ray.
- Managed static-map production records ray range, sampling interval, evidence
  range, canvas padding, source/target bounds, rejected points and edge margins.
  It must reject pose mismatches, ground-fit failures, evidence/sweep clipping,
  report/raster count mismatches and known cells entering the protected edge.
  `eligible_for_candidate` is only permission to edit/commit a map candidate; it
  is never localization, planning, safety, or execution approval.
- Registered-cloud height filtering is relative to a timestamp-matched/interpolated recorded base pose, and must reject non-finite points and canonical polygon-footprint self returns before grid accumulation. Callback-time latest pose is forbidden for offline evidence.
- Evidence thresholds and any grid padding must be explicit and recorded. Nav2 inflation and the robot footprint remain runtime costmap products and must not be baked into the source PGM.
- Trinary PGM/YAML saves must use `free_thresh: 0.196` and `occupied_thresh: 0.65` so the canonical `205` unknown pixel round-trips as unknown when Nav2 reloads the map.
- Static-map self-return cleanup may mark only the complete recorded canonical polygon-footprint sweep plus explicit clearance as free. It must consume every bag odometry pose directly, report the count, avoid DDS replay gaps, and never use a circular corridor to erase nearby obstacles.
- Ground/temporal/height-layer variants are offline comparison products. Ground fitting must report failures and model statistics; temporal rejection must require both distinct observations and elapsed span. A height layer may be treated as traversable only when the complete vehicle-plus-sensor clearance is physically verified; provisional layered maps must report `eligible_for_execution=false`.
- Offline planner visualization may display the static map, inflated global costmap, and canonical platform polygon together, but it remains non-executable and must not start the motion chain.

## Qt Waypoint Task Contract
- The maintained Qt fork is a frontend only. Its native Start/Stop controls must call the project Action and display Action feedback/result; pose-distance polling is forbidden as an execution-success test.
- Qt-compatible task JSON is operator input only. Portable frontends may submit `map`-frame PoseStamped arrays instead. Project-owned execution must validate finite coordinates, point count, repeated append patterns, and every waypoint against the currently published OccupancyGrid before sending motion goals.
- `/agt/navigation/execute_waypoint_task` is the project-owned `ExecuteWaypointTask` action. It may dispatch only Nav2 `FollowWaypoints`; it must not publish velocity or enable `agt_safety`.
- A task succeeds only when the Nav2 child Action succeeds with no missed waypoints. Rejection, abort, missed waypoints, stale safety state, map mismatch, cancellation, and unexpected exceptions are terminal failures.
- Looping must be explicit and finite. Zero-count or unbounded loops are forbidden.
- Parent cancellation and loss of recent `agt_safety` motion readiness must cancel the active Nav2 child before the project task finishes.
- New frontends and future autostart/lifecycle managers must call the project Action rather than reimplement waypoint distance polling.
- Switching the selected Nav2 map must clear stale topology before loading a matching sidecar; malformed map YAML and out-of-map topology points must fail without crashing or retaining actionable stale data.
- Mapping mode uses RViz as the default 3D frontend. Its optional Qt profile must default off and reject navigation task execution in both UI and channel; navigation mode owns the task-capable Qt profile.
- Qt navigation point authoring uses a two-click contract: the first click fixes
  the `map` position and the second fixes heading; canceling or changing tools
  must discard incomplete placement. The heading stem must begin outside the
  numbered marker and the marker label must render above every heading so its
  task order remains legible. Large-map Qt profiles keep full costmap rendering
  disabled by default because planning truth remains in Nav2/RViz.
- Offline Qt map inspection must allow view-level pan/zoom without moving map
  geometry and must cancel robot-follow mode on manual navigation. Task point
  clicks target the explicitly selected task row and topology mutations must
  refresh all task selectors. The Task Library topology selector snapshots the
  selected topology point name, metric map pose, and heading into a task
  waypoint; later topology edits must not silently rewrite saved task geometry.
  The default operator language is `zh_CN`;
  `en_US` is a persisted, restart-applied frontend preference.
- Navigation and offline Qt profiles expose the versioned Task Library as the
  only Task Center authoring surface; the legacy topology-task tab is mapping-only
  compatibility. Hiding the Task Center must cancel task-waypoint map editing so
  a hidden editor cannot consume map clicks.
- READY map-version PGM/YAML assets are immutable in navigation, offline, and
  teach Qt profiles. Base-raster editing and map save/save-as controls are
  mapping-only; edited rasters must be registered as a new version before they
  can be selected for navigation.
- Offline waypoint preview is planner-only: its Qt profile must disable task
  execution, and its launch may start only the map server, planner server,
  preview adapter, lifecycle managers, and GUI. It publishes advisory `/plan`
  from explicit task points and must not start control, BT navigation, safety
  enablement, velocity publishers, localization, or chassis nodes.
- Offline Qt must fail immediately when the preview adapter is absent instead
  of reporting a request as submitted. The adapter publishes each current/total
  segment and a terminal success/failure on its advisory status topic; every
  ComputePathToPose segment has a positive finite timeout and failure clears
  `/plan`.
- Offline multi-segment preview must start each later ComputePathToPose request
  from the previous segment's actual returned endpoint. A planner tolerance may
  adjust an intermediate task point; restarting from the original point may
  create a disconnected preview or a false lethal-start failure. This continuity
  rule must not mutate saved task points or convert preview into execution approval.
- Consecutive waypoint poses are ordered Nav2 goals, not a required straight-line
  path. Save-time validation checks each enabled endpoint against the base raster
  but must not reject a task because the visual chord crosses occupied or unknown
  cells; route reachability belongs to planner preview and runtime Nav2 Actions.
- Teach-route annotation semantics are project-owned data products. The backend
  records deterministic direction spacing and turn/U-turn window thresholds in the
  route asset and publishes `/agt/teach/route_annotations`; Qt may only render
  that read-only MarkerArray. The `teach` Qt profile must keep task authoring,
  planning preview, manual velocity publication, and execution disabled, and
  the overlay is never execution approval.
- Versioned waypoint task groups live only under
  `runtime/maps/<map_id>/versions/<map_version_id>/tasks/`; `task_index.json`
  is a rebuildable index and task files store map-version-relative asset paths.
- Schema-v1 task coordinates are metric `map` poses with stable waypoint IDs
  and finite bounded loops. Qt scene/image coordinates must never be persisted
  as execution coordinates.
- Task-group writes must use atomic replacement, retained backups, and a
  matching content hash. Save-as, copy, import, and delete must not silently
  overwrite another task or discard unsaved edits.
- `CONTENT_CHANGED` requires full offline revalidation and an explicit rebind
  plus save before execution. `GEOMETRY_MISMATCH` is read-only and may only be
  copied for explicit manual migration; automatic coordinate transforms are
  forbidden.
- Offline task validation checks enabled waypoint endpoints against the base
  raster only. It is not route reachability, footprint feasibility, Nav2
  planning, localization, safety, or execution approval.
- The Action server accepts legacy Qt `points/theta` JSON and schema-v1 task
  groups. Schema-v1 execution must fail closed unless the active map ID,
  version, YAML/image hashes, and localization PCD hash can be checked.

## Navigation Task Orchestration Contract
- Navigation is an Action capability, not a frontend-owned workflow. Qt, Web, autostart, and future mission managers must consume project Actions and their explicit result/cancel semantics.
- A future mission orchestrator may sequence navigation and manipulator Actions, but must not publish actuator velocities, bypass domain safety chains, or infer completion from pose/time alone.
- Navigation and manipulator children retain separate safety/watchdog ownership. Cross-domain execution requires explicit stationary-base, localization, TF, cancellation, and restart/idempotency policies.
- Initial workflows must be finite, sequential, versioned, and auditable. Arbitrary scripts, implicit retries, and unbounded loops are forbidden.
- The reserved architecture and staged rollout are documented in `docs/architecture/navigation_task_orchestration.md`; it does not imply that a mission or manipulator server exists.

## Future Semantic Perception Contract
- Manual, persistent semantic geometry remains GeoJSON/`coverage.yaml`; dynamic camera/lidar detections remain separate, timestamped observations and must never be painted into the source PGM.
- Sensor/backend adapters may change, but downstream integration must use normalized project topics and valid TF rather than depend directly on a detector vendor message.
- Semantic perception may contribute bounded obstacle observations, operator overlays, diagnostics, and future behavior decisions. It must never publish velocity, enable motion, or bypass Nav2, `agt_safety`, or the chassis watchdog.
- Human detections are advisory until dataset accuracy, latency, stale-data clearing, false-negative behavior, and fail-safe field tests are explicitly accepted.
- Reserved future interfaces and rollout gates are documented in `docs/architecture/future_semantic_perception_interfaces.md`; a reservation is not an implemented or safety-certified interface.

## PCD Persistence Runtime Contract
- LIO-only Bunker mapping must build the navigation PCD incrementally with sparse signed 64-bit voxel keys; it must not retain the complete raw accumulated cloud merely to downsample at shutdown.
- Non-finite points and finite points outside the configured absolute coordinate bound must be rejected before voxel insertion, and the saved processing record must report both rejection counts and observed bounds.
- A navigation PCD is ready only when `localization_map.processing.yaml` reports `state: ready`; legacy raw/downsampled files that are byte-identical or produced after PCL grid-index overflow are not valid localization inputs.
- Localization computes the active PCD `map_hash` as `sha256:<64 lowercase hexadecimal characters>` before loading candidates or persisting last pose. A processing record's optional `pcd_sha256` (or compatibility `map_hash`) must match the file when present; missing hashes are legacy metadata and must remain visible as unverified.
- x86 FAST-LIVO builds that exchange Eigen-aligned point storage with distribution PCL binaries must preserve the distribution's 16-byte alignment ABI; native AVX alignment flags are forbidden unless PCL is rebuilt with the same ABI.

## Coverage Path Validation Contract
- TASK-10 consumes `/agt/coverage/path_raw`, `/global_costmap/costmap`, and `/global_costmap/published_footprint`, all in `map` frame.
- Collision checking must use the complete canonical `navigation_footprint` polygon against costmap cells; center-only and corner-only checks are forbidden.
- Distance and angular interpolation must depend on costmap resolution and footprint radius so sparse translations and rotation sweeps are checked.
- OccupancyGrid values remain `-1/0..100`; unknown-space and outside-costmap policies must be explicit, with collision as the safe default.
- The canonical platform footprint performs collision checks. The published Nav2 footprint is a runtime shape-consistency check and must not become a second geometry source.
- Any invalid or incomplete validation result must publish an empty validated path so stale valid output cannot remain actionable.
- TASK-10 reports validation only. It must not repair paths, classify swaths/connections, publish TF, or command the chassis.

## Coverage Path Semantics Contract
- TASK-11 treats the locked OpenNav `PathComponents.swaths` and `PathComponents.turns` as the authoritative semantic source; do not infer swaths from path heading alone.
- The first semantic contract supports exactly `SWATH` and `CONNECTION`. Do not guess `APPROACH` or `EXIT` until their source semantics are defined.
- Every interval in `/agt/coverage/path_raw` and `/agt/coverage/path_reconstructed` must have exactly one component type and component ID.
- Stable `swath_NNNN` IDs are assigned from canonical endpoint geometry, independently of route order and travel direction; `order_index` separately records execution order.
- Reconstructing the flat path from components must preserve geometric length within explicit absolute/relative tolerances or reject the planning result transactionally.
- `/agt/coverage/path_semantics` must carry an exact raw-path fingerprint. TASK-10 must reject stale or mismatched semantics and expose invalid component/swath IDs in its report.
- TASK-11 classifies and reconstructs only. It must not repair connections, alter swath geometry, publish TF, or command the chassis.
- TASK-12 may replace invalid `CONNECTION` components only; all `SWATH` coordinates and IDs must remain unchanged.

## Coverage Path Repair Contract
- TASK-12 may call Nav2 `ComputePathToPose` only for component IDs that the matching TASK-10 report marks invalid and TASK-11 marks `CONNECTION`.
- A validation report must match the semantic raw-path fingerprint, and semantics must match the exact reconstructed-path fingerprint before repair starts.
- Repair requires semantic status `LOADED` and validates candidates directly against both the global costmap and `/agt/map/keepout_mask`; runtime KeepoutFilter state is not trusted as the only allowed-area guard.
- Candidate endpoints may differ only within an explicit tolerance and are replaced with the exact original connection endpoints before splicing.
- Every candidate and the final joined path must pass the same full-footprint, interpolation, unknown-space and curvature validator used by TASK-10.
- Repair is transactional. Any stale input, invalid swath, planner failure, collision, incomplete replacement or final validation failure clears the repaired output and leaves all source products unchanged.
- Platform profiles must explicitly select and enable a repair planner. Differential/tracked platforms allow in-place rotation; Ackermann platforms require a positive turning radius and Hybrid-A* or State Lattice family.
- The provisional MK-mini profile remains repair-disabled until its differential/Ackermann contradiction and minimum turning radius are resolved; never fall back to BUNKER planner parameters.
- TASK-12 must not alter SWATH coordinates or IDs, semantic geometry, user route order, TF, controller topics, or chassis commands.

## Coverage Task Interface Contract
- `agt_interfaces/action/ExecuteCoverageTask.action` must be generated through `rosidl_generate_interfaces`; installing an ungenerated text file is not an interface implementation.
- Python and C++ generated types are both contract-tested. Goal, Result, or Feedback field changes require interface documentation, serialization tests, and migration-matrix updates.
- Downstream packages must depend on `agt_interfaces` and import generated types; do not duplicate the Action declaration or create project-owned lookalike messages.
- TASK-14 exposes only `/agt/coverage/execute` as the project-owned coverage execution action. It must load matching semantic products, plan, validate, optionally repair, and reach `READY` before motion dispatch.
- The requested `field_id` and `planning_mode` must match the loaded semantic task exactly; stale semantic, validation, repair, or mask products must not be reused for a new goal.
- Coverage execution may send only a standard Nav2 `FollowPath` goal. It must never publish velocity commands, call the motion-enable service, bypass Nav2, or command the chassis directly.
- Execution is fail-closed by default. It requires explicit `execution_enabled`, recent `agt_safety` diagnostics with motion enabled and emergency stop clear, semantic state `LOADED`, and a ready Nav2 server.
- Parent cancellation during execution must be accepted by the active Nav2 child before the parent reports `CANCELED`; a safety readiness loss must cancel the child and fail the task.
- SWATH progress comes from TASK-11 path semantics and actual cumulative path distance. CONNECTION distance must not be counted as a second work row.
- `PAUSED` is reserved by the interface but is not emitted until a pause/resume contract is implemented. TASK-14 does not estimate coverage or overlap metrics; these remain zero until TASK-16.

## Coverage Bringup Contract
- TASK-15 composes semantic map, keepout filtering, coverage planning and the coverage task server only through `agt_bringup` navigation mode. It must not create a second Nav2, TF, description, safety or chassis owner.
- `start_semantic_map_server` and `start_coverage_planning` default to false so the pre-coverage navigation node set remains unchanged.
- Coverage planning requires the semantic server. Semantic operation requires an existing GeoJSON, its sibling `coverage.yaml`, and an existing canonical platform profile; reject invalid combinations before child launches start.
- Enabling the semantic server must also enable the existing Nav2 global Keepout Filter Info Server. Never add the semantic mask to the local obstacle chain.
- `annotation_mode` selects the project-owned semantic editor instead of the vendor operator GUI and must keep coverage execution disabled.
- Process startup order is not readiness. Motion procedures must verify map/localization, semantic `LOADED`, mask, global costmap, coverage server and `agt_safety` readiness before execution.
- Coverage components remain in the same launch process tree as Nav2 and safety. Normal shutdown must terminate their Action Servers and rely on the safety/chassis watchdog chain to zero commands; never document `kill -9` as a supported shutdown.

## Acceptance Mindset
- Prefer small, reviewable changes.
- Keep placeholders explicit so future migration tasks know what is still missing.
- For each module, document inputs, outputs, TF responsibility, and non-goals.

## Teach-Repeat Runtime Contract
- `agt_teach_repeat` is an optional navigation client and offline asset pipeline;
  it is not a SLAM, localization, TF, controller, safety, map-edit, or chassis owner.
- Raw teach poses come only from `/agt/mapping/odometry`. Missing mapping odometry
  must fail with an actionable error and must never fall back to velocity commands
  or chassis odometry.
- Each executable reference path is explicitly transformed from its recorded odom
  frame into `map` by the manifest's planar `map_from_teach_odom`; identity is an
  explicit configured value, not an implicit frame assumption.
- Teach assets bind the reference path, map YAML, localization PCD, and ready PCD
  processing record by SHA-256. Preview may remain available with a warning after
  a binding mismatch, but execution is fail-closed.
- Path and corridor checks consume `navigation_footprint` from the selected platform
  profile and reuse the coverage path-validation core. They must not contain a
  second footprint or modify the OccupancyGrid, PGM, PCD, or semantic products.
- `/agt/teach/path_validated` is empty after any invalid or incomplete validation.
  Corridor outputs are advisory and `eligible_for_automatic_map_edit` remains false.
- Teach execution may send only Nav2 `FollowPath` and a conservative Nav2 speed
  limit. It must not publish velocity, enable motion, publish TF/odometry, bypass
  Collision Monitor or `agt_safety`, or start a second navigation stack.
- Execution requires current accepted `TRACKING` localization with matching map
  identity, operator-enabled safety with clear emergency stop, ready
  `/agt/system/task_readiness`, a matching validated path, and an available Nav2
  server. Loss of any runtime gate cancels the active child goal.
- Repeatability errors use the onboard map-frame localization estimate and therefore
  measure system-internal repeatability, not independent absolute-position truth.
  Mapping and chassis odometry remain comparison evidence only.
- Runtime assets live under `runtime/teach_repeat/<demo_id>/` and are not committed.
  Writes are atomic, finite-only, schema checked, and do not overwrite an existing
  demo unless overwrite is explicitly requested.

## Web Experiment and Operations Console Contract
- Real MID360 health checks consume the adapter's `/agt/sensors/lidar/custom`
  `livox_ros_driver2/msg/CustomMsg` input. A running driver process is not
  considered healthy until this topic and `/agt/sensors/imu/data` are fresh.
- `agt_system_manager`, `agt_map_manager`, `agt_experiment_manager`, and
  `agt_web_console` remain separate ownership boundaries; do not move their
  health, process, map, or experiment logic into `agt_ui_bridge`.
- Web control may call only generated project ROS interfaces and configured
  map/experiment services. It must never accept or execute arbitrary shell,
  publish final `/cmd_vel`, publish TF, or bypass `agt_safety`.
- The real Web backend is only an HTTP/WebSocket adapter over `RosConsoleBridge`.
  It must not instantiate `MapRegistry` or `ExperimentManager`, inspect manager
  manifests/process snapshots, or own Mission, map, experiment, or Bag state.
  RobotState and MissionStatus are its primary live WebSocket read models.
- `ChangeSystemMode` profiles are argv allowlists. Profile arguments are
  validated against declared keys; browser input is never a command string.
  Managed processes use their own process group and only processes created by
  the manager may be stopped.
- `SystemHealth` is evidence-based: topic discovery alone is not health.
  Required message freshness, frequency, TF, nodes, lifecycle state, boolean
  conditions, and disk space must be evaluated from the versioned contract.
- `TaskReadiness` is the shared fail-closed gate. Waypoint and future task
  servers must check it at goal acceptance and again during execution; a GUI
  button is never an authorization boundary.
- `agt_mission_manager` sequences only finite `WAYPOINT_TASK`, `WAIT_DURATION`,
  and `WAIT_EVENT` steps. It calls only the project waypoint Action, never Nav2
  native Actions, launch files, TF, safety enablement, or velocity topics. Parent
  pause/cancel waits for child cancel confirmation; resume revalidates active-map,
  localization, and TaskReadiness evidence.
- `/agt/system/robot_state` is a freshness-aware read model for clients, not a
  business owner. Active-map identity comes only from `/agt/maps/active`; missing
  or stale lifecycle, safety, chassis, localization, readiness, and mission
  evidence remains UNKNOWN or blocked.
- Runtime map versions are immutable bundles under
  `runtime/maps/<map_id>/versions/<map_version_id>/`. `manifest.yaml` is the
  portable source of truth; SQLite is rebuildable index data. Activation
  requires a READY manifest, verified navigation assets, and a ready PCD
  processing record with matching content identity.
- Experiment state transitions and event/summary/report writes are atomic.
  An interrupted RUNNING session becomes `INTERRUPTED`, never `COMPLETED`.
  Rosbag profiles contain explicit topic lists and must not use `record -a`.
- Relocalization modes are `MANUAL_ONLY`, `AUTO_ON_START`, and
  `AUTO_RECOVERY`; every automatic request is bounded by attempts, cooldown,
  candidate count, and total timeout. Localization remains the only
  `map -> odom` owner.
- The Web default listener is `127.0.0.1`; a non-loopback listener requires a
  token. Log browsing is limited to manager-owned runtime roots.
- A configured Web `runtime_dir` admits only one Web process; duplicate startup
  must fail before creating another `agt_web_console_ros_bridge` node.
- Web `offline` mode is an explicit UI-test backend only. It may simulate
  configured profile state and bounded relocalization feedback, but must never
  start ROS processes, record bags, publish TF/velocity, or mark task readiness
  executable. It may show a bounded, clearly marked simulated occupancy and
  point-cloud preview after the simulated bag workflow enters mapping, but it
  must never read bag messages or export PGM/YAML/PCD. The simulator has at most
  one in-memory retained map slot. Backend switching requires all managed
  modules to be stopped.
- The Web console's mapping state is evidence-based: `MAPPING` requires fresh
  `/agt/mapping/odometry` and `/agt/mapping/registered_points_lidar`; the
  `/agt/map/mapping_occupancy` snapshot uses matching `RELIABLE +
  TRANSIENT_LOCAL` QoS and is persistent rather than a three-second periodic
  observation. Its occupancy preview is bounded,
  read-only UI data and cannot feed navigation.
- Full-map OctoMap projection consumes an explicit bounded-rate, voxel-capped
  copy of the registered cloud. Its republisher keeps only the newest unprocessed
  lidar-frame cloud on a steady-time timer and preserves the original stamp and
  frame so OctoMap resolves the matching dynamic sensor origin. It may release
  only one cloud until the corresponding OccupancyGrid publication acknowledges
  projection, with a bounded recovery timeout; it must not feed delayed queued
  clouds or relabel them into the fixed frame. The immediate local obstacle chain
  must continue consuming the unthrottled registered cloud.
  Mapping-only PCD replay may disable the full-map projection explicitly.
- Humble OctoMap Server 2.3.1 must use its declared `point_cloud_*` parameter
  names and complete 2D projection; its incremental projection path publishes
  an all-unknown grid in this baseline. OctoMap publishes a volatile internal
  `/agt/map/mapping_occupancy_raw`; the project throttle relays completed grids
  as transient-local `/agt/map/mapping_occupancy`. Qt, SaveMap, bags, and other
  consumers must use only the project output topic.
- Web mapping bag playback is forced to the configured raw-input allowlist
  (`/clock`, `/tf_static`, MID360 CustomMsg, and IMU), excluding recorded
  FAST-LIVO2 outputs and `/tf`; navigation mode refuses bag playback.
- MID360 sensor startup is owned by a reusable manager `sensor_only` process
  group. Mapping/navigation transitions reuse that group and stop only the
  previous main chain; switching to `SENSOR_ONLY` stops mapping/navigation.
  Mapping and navigation chassis startup is explicit and defaults off for
  disconnected-vehicle tests.
- The Web sensor-start control is evidence-locked: an active managed sensor
  process, healthy MID360 evidence, or an active mapping/navigation chain
  disables repeat startup. Mapping and navigation have separate primary
  controls; a running primary chain disables the other.
- The ROS Web backend requires a live `/agt/system/change_mode` Action server
  owned by `agt_system_mode_manager`; Web must report the missing server and
  its diagnostic command instead of treating an unavailable manager as a
  mapping launch failure.
- Real Web mapping control consumes `/agt/mapping/manage_session`; Web must not
  create mapping directories, call SaveMap directly, wait for PCD, or register
  map versions. The map ID is fixed at START. FINALIZE_CAPTURE produces an
  editable candidate, COMMIT creates a new immutable version, and DISCARD moves
  the session plus any failed version to recoverable trash. Offline
  retain/delete remains simulation-only and writes no real assets.
- Web navigation startup requires an active `READY` map version and derives
  `map`, localization PCD, and processing-record arguments from the map manager
  response.
  Browser-supplied asset paths must match the selected version or the service
  rejects the request.
- `agt_chassis` currently exposes the `bunker_can` backend. `operation_mode:=monitor`
  starts only the BUNKER CAN/status path, remaps the driver's command input to the
  deliberately unowned `/agt/chassis/monitor_cmd_vel`, and must not start
  `agt_safety` or the command guard. Mapping may use this mode for telemetry only;
  navigation control requires explicit `start_chassis:=true` and remains
  `Nav2 -> agt_safety -> agt_chassis`.
- CAN interface provisioning is a host privilege boundary. Web may read the
  configured interface's `/sys/class/net/<iface>/operstate` and show diagnostics,
  but it must never run `sudo`, `ip link`, `modprobe`, or an arbitrary CAN setup
  command. Provision CAN once through an administrator-owned system service or
  equivalent host configuration, then let the ROS node consume the ready interface.
- The Web point-cloud preview is a bounded, downsampled UI cache of registered
  mapping points. It is read-only, must not be treated as the persisted PCD or
  OccupancyGrid, and must not feed localization, planning, validation, or control.
- ROS Web bag replay is limited to complete bundles under the configured
  `runtime/rosbag` root and uses the fixed `ros2 bag play --clock` command. The
  offline backend may simulate the selected bundle state for UI testing, but it
  never reads ROS messages, starts `ros2 bag play`, records bags, publishes
  topics, or changes task readiness; real replayed nodes must explicitly use
  simulated time and normal readiness/safety gates still apply.
- Mapping algorithm startup is independent of sensor startup. A mapping profile
  may explicitly use `start_sensor:=false` for historical bag input; FAST-LIVO2
  and map processing then wait for their configured input topics. The Web
  mapping input selector must force `use_sim_time` for historical replay.
- Web mapping previews are mode-gated: `/agt/mapping/registered_points` and
  `/agt/map/mapping_occupancy` may populate the UI only while the managed main
  mode is `MAPPING`; stopping or never starting that mode clears both previews.
  Preview pan/robot-centering is view-only and cannot alter map coordinates or
  feed navigation.

## Mapping Session Workflow Contract
- `/agt/mapping/manage_session` is the only project boundary for a real mapping
  session. Qt, Web, CLI, and future frontends must use its finite
  START/FINALIZE_CAPTURE/COMMIT/DISCARD/STATUS operations and must not reproduce
  the artifact sequence.
- START owns `runtime_dir`, `map_name`, `mapping_output_dir`, `record_bag`, and
  `bag_profile`. Every real mapping session records the explicit `mapping` bag
  profile; a frontend cannot disable or redirect that evidence.
- FINALIZE_CAPTURE must save trinary PGM/YAML while the mapping OccupancyGrid is
  still live and verify that the raster contains both free and occupied evidence
  before stopping the managed mapping process normally so FAST-LIVO2 and rosbag
  can flush. It then requires a non-empty PCD, `state: ready` processing record
  with matching SHA-256, and bag `metadata.yaml`. The saved online raster is then
  preserved under `online_preview/`; FINALIZE runs the managed offline static-map
  producer and reaches `CANDIDATE_READY` only after its report and promoted PGM
  pass the static-navigation-map quality contract. A grid-save or grid-content
  failure leaves mapping running; incomplete post-stop assets and offline-build
  failures fail closed.
- `BUILDING_STATIC_MAP` and `CANDIDATE_BUILD_FAILED` are non-motion states. A
  failed/restarted offline build may retry FINALIZE from the preserved online
  preview without saving the grid again or stopping mapping a second time. Qt
  candidate authoring must not open before `CANDIDATE_READY`.
- The managed CLI may translate one Ctrl+C into FINALIZE_CAPTURE. A second
  signal must not force-kill asset writers. Normal workflow documentation must
  not use SIGKILL as a mapping completion path.
- `CANDIDATE_READY` assets below `runtime/mapping_sessions/` are the only base
  raster files that may be edited in place. COMMIT revalidates the current
  candidate metadata and free/occupied raster evidence, rejects all-unknown
  candidates, and copies valid contents into a new immutable
  `runtime/maps/<map_id>/versions/<map_version_id>/` bundle. Existing READY
  PGM/YAML files are never edited even when map geometry is unchanged.
- The maintained Qt candidate saver omits the optional Nav2 `mode` key while
  retaining trinary thresholds. COMMIT may atomically restore `mode: trinary`
  only when the key is absent and must record that repair; an explicit null,
  `scale`, `raw`, or other mode remains invalid and must fail closed.
- Qt `mapping` is monitor-only. Qt `candidate` may edit only its explicitly
  selected candidate in place and must disable map-open, Save As, task library,
  planning preview, execution, and manual control. Qt `offline`/`navigation`
  uses the committed READY map read-only and saves tasks only under that
  version's `tasks/` directory.
- Task authoring begins only after COMMIT supplies a real map version identity.
  Candidate geometry cannot be used to create a schema-v1 task binding. Planner
  preview remains advisory; activation and full navigation readiness are still
  required before execution.

## Teach Mapping MVP Contract
- Teach-mapping sessions bind one immutable bootstrap map, teach bag, platform
  profile, extracted route, rescan bag, and candidate output by SHA-256. Large
  source assets are referenced rather than copied, and session updates are
  atomic.
- The rescan launch uses only the session bootstrap map for navigation. It
  defaults chassis and execution off, fixes teach auto-start off, does not
  enable motion, and limits first-pass speed to `0.10 m/s` by default.
- Rescan evidence must retain raw MID360 CustomMsg, IMU, mapping odometry,
  localization, executed path, safety, and chassis status. Empty or wrong-type
  required topics fail registration.
- Candidate maps are generated only by bounded offline replay of raw MID360,
  IMU, and clock through the existing mapping chain. They remain separate from
  bootstrap and active maps, are never automatically published, and require a
  ready PGM/YAML/PCD/processing-record bundle.
- Candidate-process cleanup uses process-group `SIGINT` and may escalate to
  `SIGTERM`, but never `SIGKILL`, so FAST-LIVO2 can finalize its PCD. Failures
  preserve assets and the last successful session stage.
- Map comparison is advisory and deterministic. It uses canonical full-footprint
  geometry, supports differing map geometry and origin yaw, and must never
  choose or activate a winner.
