# Map Manifest and Version Registry

Canonical layout:

```text
runtime/maps/<map_id>/versions/<map_version_id>/
  manifest.yaml
  source/
  pointcloud/
  navigation/
  semantic/
  preview/
  reports/
```

`map_version_id` follows `map_YYYYMMDD_HHMMSS_<8 hex characters>`. A manifest
contains schema/version identity, parent, state (`DRAFT`, `PROCESSING`, `READY`,
`INVALID`, `ARCHIVED`), timestamps, platform/frame, asset relative paths and
SHA-256 values, navigation metadata, processing backend, active/pinned flags,
tags and notes. `manifest.yaml` is portable truth; `map_registry.sqlite3` is a
rebuildable query index.

Activation runs all checks transactionally: files and declared hashes, YAML/PGM
resolution/origin/size, ready PCD processing record, PCD content hash and map
identity. Only a valid `READY` version can be active. The selected identity is
written to `runtime/maps/active_map.yaml` atomically and is consumed by the
system health adapter.

`pin`, active state, processing state, parent dependencies and experiment
references protect versions from retention. Soft delete moves a version to
`.trash` with a restore record; permanent purge is separate. Existing legacy
`runtime/maps/<name>/` data is not silently rewritten or selected. It must be
explicitly packaged into a version manifest and registered.
