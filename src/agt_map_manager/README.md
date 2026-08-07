# agt_map_manager

`MapRegistry` stores portable immutable map bundles and a rebuildable SQLite
index. It validates declared asset hashes, PGM/YAML metadata, and ready PCD
processing records before activation. Active switching writes an atomic
`active_map.yaml` pointer; source maps and active PCD files are never modified.

The `OptimizeMap` Action is an explicit fail-closed reservation. No BA, pose
graph, factor graph, or visual backend is implemented.

`map_manager_node.py` is the sole runtime facade for registry mutations:

- `/agt/maps/list` filters by map ID/state and excludes deleted versions by default.
- `/agt/maps/manage` validates, activates, pins, archives, soft-deletes, purges, or imports a
  mapping candidate. Destructive operations require confirmation and reject dependencies or
  experiment references.
- `/agt/maps/active` publishes the manager-resolved YAML, PCD, processing-record, tasks path and
  hashes with reliable transient-local depth 1.

Clients must consume those returned paths. `SystemHealth`, Web, Qt, and mapping-session code do not
read `active_map.yaml` to create a second active-map owner.
