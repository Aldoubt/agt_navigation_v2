# Map Manifest and Version Registry

Canonical layout:

```text
runtime/maps/<map_id>/versions/<map_version_id>/
  manifest.yaml
  source/
    dataset_binding.yaml
    calibration.yaml
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

A derived map must be traceable to immutable inputs. Formal V25-09A derivation copies the exact Calibration Set artifact into the map bundle instead of recording only an external ID:

```yaml
source:
  dataset_binding: source/dataset_binding.yaml
  dataset_binding_sha256: sha256:<64 lowercase hex>
calibration:
  calibration_id: cal_bunker_mid360_20260810
  path: source/calibration.yaml
  sha256: sha256:<64 lowercase hex>
derivation:
  recipe: derivation/recipe.yaml
  recipe_sha256: sha256:<64 lowercase hex>
platform_profile: bunker
platform_profile_sha256: sha256:<64 lowercase hex>
```

The Dataset binding itself continues to use the field name `calibration_sha256`; the Map Manifest uses the generic `path` + `sha256` artifact form shown above. Both hashes must identify the same Calibration Set bytes.

The detailed contracts are defined in `calibration_dataset_contract.md` and `map_derivation_contract.md`.

## Alignment identity

All assets in one map version must share the same canonical `map` frame and alignment lineage. The manifest should bind:

```yaml
alignment:
  site_frame: alignment/site_frame.yaml
  site_frame_sha256: sha256:<64 lowercase hex>
  record: alignment/alignment.yaml
  record_sha256: sha256:<64 lowercase hex>
```

`alignment/alignment_report.json` is a canonical hashed asset in `assets` once generated. It must report `PASS` before a derived map is promoted to `READY`.

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

`refresh-map` freezes generated canonical products into the manifest `assets` table. When present, `semantic_map.geojson` and `coverage.yaml` are frozen separately as `semantic_map` and `semantic_coverage`; Route derivation must match both hashes exactly.

Global Navigation Map, Localization Prior, Semantic Map and Route Asset are distinct products even when derived from the same source bag.

## READY quality gate

A formal derived map remains `PROCESSING` while mapping, cleaning, alignment and semantic editing are still changing assets. It is promoted only after all products intended for that version are complete.

The V25-09A offline gate requires the immutable Dataset/Calibration/Recipe/Alignment lineage, navigation map, ready localization PCD processing record, `alignment_report.json` and `map_quality_report.json`. All declared assets are hashed before the one-way promotion to `READY`.

A `READY` map version is immutable. `agt_offline_assets refresh-map` refuses to refresh hashes or change state after promotion. Read-only auditing uses `validate-map`; activation/registry checks remain separate from asset generation.

Only a valid `READY` version can be active. The selected identity is written to `runtime/maps/active_map.yaml` atomically and is consumed by the system health adapter.

Route readiness is separate from map readiness: each Route Asset binds the exact map/semantic/coverage/vehicle/policy hashes and has its own feasibility report plus preview evidence. A READY map does not make every route READY.

## Immutability and retention

`pin`, active state, processing state, parent dependencies and experiment references protect versions from retention. Soft delete moves a version to `.trash` with a restore record; permanent purge is separate.

Existing legacy `runtime/maps/<name>/` data is not silently rewritten or selected. It must be explicitly packaged into a version manifest and registered. READY map content is immutable; re-alignment, cleaning or semantic edits create a new map version. Route revisions are separately versioned children under the bound READY map and may be added without changing the frozen map manifest; route tuning creates a new route revision rather than mutating an accepted Route Asset.
