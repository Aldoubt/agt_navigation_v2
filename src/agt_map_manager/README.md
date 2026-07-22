# agt_map_manager

`MapRegistry` stores portable immutable map bundles and a rebuildable SQLite
index. It validates declared asset hashes, PGM/YAML metadata, and ready PCD
processing records before activation. Active switching writes an atomic
`active_map.yaml` pointer; source maps and active PCD files are never modified.

The `OptimizeMap` Action is an explicit fail-closed reservation. No BA, pose
graph, factor graph, or visual backend is implemented.
