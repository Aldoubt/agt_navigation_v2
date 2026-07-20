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
- Every change that affects architecture or interfaces must update `docs/`, this file, and `docs/migration/migration_matrix.md`.

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
  must discard incomplete placement. Large-map Qt profiles keep full costmap
  rendering disabled by default because planning truth remains in Nav2/RViz.
- Offline Qt map inspection must allow view-level pan/zoom without moving map
  geometry and must cancel robot-follow mode on manual navigation. Task point
  clicks target the explicitly selected task row and topology mutations must
  refresh all task selectors. The default operator language is `zh_CN`;
  `en_US` is a persisted, restart-applied frontend preference.
- Offline waypoint preview is planner-only: its Qt profile must disable task
  execution, and its launch may start only the map server, planner server,
  preview adapter, lifecycle managers, and GUI. It publishes advisory `/plan`
  from explicit task points and must not start control, BT navigation, safety
  enablement, velocity publishers, localization, or chassis nodes.

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
