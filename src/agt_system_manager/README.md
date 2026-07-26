# agt_system_manager

System-level health, readiness, process profile and bounded relocalization
coordination. `health.py`, `readiness.py`, `process_manager.py`, and
`localization_mode.py` are ROS-independent and unit tested with deterministic
observations. ROS adapters publish generated interfaces and call existing
launch/Action boundaries.

The node does not own any TF edge, velocity topic, map algorithm, or safety
enable service. Start it with `system_manager.launch.py`; configure `runtime_dir`
and the profile/health contract paths explicitly for deployment.

`teach_mapping_workflow.py` adds a bounded, atomic session workflow around the
existing teach-repeat and mapping modules. It references large source assets by
absolute path and SHA-256, composes low-speed rescan through the allowlisted
`teach_rescan` profile, and builds candidate maps only in a short-lived offline
process tree. See
[`docs/testing/teach_mapping_mvp_field_test.md`](../../docs/testing/teach_mapping_mvp_field_test.md).
