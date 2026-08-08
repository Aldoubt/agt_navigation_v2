# Map Manifest and Version Registry

Canonical layout:

```text
runtime/maps/<map_id>/versions/<map_version_id>/
  manifest.yaml
  source/
    dataset_binding.yaml
  derivation/
    recipe.yaml
  alignment/
    site_frame.yaml
    alignment.yaml
    alignment_report.json
  pointcloud/
  navigation/
  semantic/
  routes/
  preview/
  reports/
```

`map_version_id` follows `map_YYYYMMDD_HHMMSS_<8 hex characters>`. `manifest.yaml` is portable truth; `map_registry.sqlite3` is a rebuildable query index.

## Identity and purpose

A map version must have stable `map_id` / `map_version_id`. New reproducible derivations should additionally record:

```yaml
schema_version: 1
map_id: greenhouse_a
map_version_id: map_20260810_120000_1234abcd
site_id: greenhouse_a
epoch_id: 2026-08-10-am
purpose: OPERATIONAL
frame_id: map
```

`purpose` distinguishes at least:

- `EVALUATION`: open-field/RTK truth and algorithm A/B assets
- `OPERATIONAL`: facility/agricultural navigation, semantics and route assets

Existing schema-v1 manifests that predate these optional lineage fields remain readable; new offline derivation workflows should populate them and their companion binding files.

## Source provenance

A derived map must be traceable to immutable inputs:

```yaml
source:
  dataset_binding: source/dataset_binding.yaml
  dataset_binding_sha256: sha256:<64 lowercase hex>
calibration:
  calibration_id: cal_bunker_mid360_20260810
  calibration_sha256: sha256:<64 lowercase hex>
derivation:
  recipe: derivation/recipe.yaml
  recipe_sha256: sha256:<64 lowercase hex>
platform_profile: bunker
platform_profile_sha256: sha256:<64 lowercase hex>
```

The detailed contracts are defined in `calibration_dataset_contract.md` and `map_derivation_contract.md`.

## Alignment identity

All assets in one map version must share the same canonical `map` frame and alignment lineage. The manifest should bind:

```yaml
alignment:
  site_frame: alignment/site_frame.yaml
  site_frame_sha256: sha256:<64 lowercase hex>
  record: alignment/alignment.yaml
  record_sha256: sha256:<64 lowercase hex>
  report: alignment/alignment_report.json
  report_sha256: sha256:<64 lowercase hex>
```

For the same physical site at different epochs, keep the same `site_id`/site-frame definition and create a new `map_version_id`; do not overwrite the previous READY version. A later epoch may reference an accepted version for alignment, but stable/control-point evidence should dominate seasonal vegetation.

## Asset groups

A manifest contains schema/version identity, parent/reference, state (`DRAFT`, `PROCESSING`, `READY`, `INVALID`, `ARCHIVED`), timestamps, platform/frame, asset relative paths and SHA-256 values, navigation metadata, processing backend, active/pinned flags, tags and notes.

Typical asset groups:

```text
pointcloud/
  raw_map.pcd
  cleaned_map.pcd
  localization_map.pcd
  localization_map.processing.yaml

navigation/
  map.pgm
  map.yaml

semantic/
  semantic_map.geojson
  coverage.yaml
  validation_report.json

routes/<route_id>/<revision>/
  route.yaml
  route.csv
  policy.yaml
  feasibility_report.json
  preview.geojson
  tuning.yaml            # optional
```

Global Navigation Map, Localization Prior, Semantic Map and Route Asset are distinct products even when derived from the same source bag.

## READY quality gate

Activation keeps all current transactional checks: files and declared hashes, YAML/PGM resolution/origin/size, ready PCD processing record, PCD content hash and map identity. New derivation workflows must also produce `reports/map_quality_report.json` and keep alignment/recipe provenance intact before declaring the asset bundle operationally READY.

Only a valid `READY` version can be active. The selected identity is written to `runtime/maps/active_map.yaml` atomically and is consumed by the system health adapter.

Route readiness is separate from map readiness: each Route Asset binds the exact map/semantic/vehicle/policy hashes and has its own feasibility report. A READY map does not make every route READY.

## Immutability and retention

`pin`, active state, processing state, parent dependencies and experiment references protect versions from retention. Soft delete moves a version to `.trash` with a restore record; permanent purge is separate.

Existing legacy `runtime/maps/<name>/` data is not silently rewritten or selected. It must be explicitly packaged into a version manifest and registered. READY assets are immutable; cleaning, semantic edits, re-alignment or route tuning create a new map version or route revision instead of mutating accepted evidence in place.
