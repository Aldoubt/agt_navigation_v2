# agt_system_manager

System-level health, readiness, process profile and bounded relocalization
coordination. `health.py`, `readiness.py`, `process_manager.py`, and
`localization_mode.py` are ROS-independent and unit tested with deterministic
observations. ROS adapters publish generated interfaces and call existing
launch/Action boundaries.

The node does not own any TF edge, velocity topic, map algorithm, or safety
enable service. Start it with `system_manager.launch.py`; configure `runtime_dir`
and the profile/health contract paths explicitly for deployment.
