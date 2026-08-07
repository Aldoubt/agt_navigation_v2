# agt_system_manager

System-level health, readiness, process profile and bounded relocalization
coordination. `health.py`, `readiness.py`, `process_manager.py`, and
`localization_mode.py` are ROS-independent and unit tested with deterministic
observations. ROS adapters publish generated interfaces and call existing
launch/Action boundaries.

The node does not own any TF edge, velocity topic, map algorithm, or safety
enable service. Start it with `system_manager.launch.py`; configure `runtime_dir`
and the profile/health contract paths explicitly for deployment.

`robot_state_aggregator.py` publishes reliable transient-local
`/agt/system/robot_state` at 2 Hz and immediately after authoritative inputs
change. `/agt/system/get_robot_state` returns the same read model. It consumes
`/agt/maps/active` instead of reading map pointers or manifests, and preserves
UNKNOWN/blocker state when health, readiness, localization, Mission, lifecycle,
safety, chassis, odometry, or Bag evidence is missing or stale. It does not own
any field it aggregates. `system_manager.launch.py` also starts the separate
`agt_mission_manager`, `agt_map_manager`, and `agt_experiment_manager` business owners.

`mapping_session_manager.py` owns the finite real-mapping artifact sequence at
`/agt/mapping/manage_session`. `mapping_session_workflow.py run` turns one
operator Ctrl+C into save-grid, normal process stop, ready-PCD/hash and bag
metadata checks. Before normal shutdown it parses the saved online trinary
YAML/P5 and requires both free and occupied evidence. After shutdown it preserves
that grid as `online_preview`, rebuilds bounded ray-traced free space from the
same bag, overlays `ground_temporal` evidence and the complete canonical sweep,
and checks pose/ground-fit/clipping/report/edge quality. Offline failure is
retryable without a second grid save or mapping stop. COMMIT repeats the content,
production-geometry and protected-edge checks after editing. It atomically restores
an omitted `mode: trinary` from the Qt candidate-save path, records that repair,
and still rejects explicit non-trinary modes. It produces an editable candidate
only after those gates; only a later COMMIT copies that candidate into a new
immutable map version and creates its `tasks/` directory. See
[`docs/workflows/mapping_task_navigation_workflow.md`](../../docs/workflows/mapping_task_navigation_workflow.md).

The mapping Action does not own `MapRegistry` or a rosbag process. START creates an experiment and
starts the `mapping` bag profile through `/agt/data/bags/manage`; FINALIZE stops and completes that
capture; COMMIT imports and optionally activates the candidate through `/agt/maps/manage`. The
mapping launch receives `record_bag:=false`, so only one recorder and one map owner exist.

`teach_mapping_workflow.py` adds a bounded, atomic session workflow around the
existing teach-repeat and mapping modules. It references large source assets by
absolute path and SHA-256, composes low-speed rescan through the allowlisted
`teach_rescan` profile, and builds candidate maps only in a short-lived offline
process tree. See
[`docs/testing/teach_mapping_mvp_field_test.md`](../../docs/testing/teach_mapping_mvp_field_test.md).
