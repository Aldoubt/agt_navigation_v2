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

A reproducible map version records:

```yaml
schema_version: 1
map_id: greenhouse_a
map_version_id: map_20260810_120000_1234abcd
site_id: greenhouse_a
epoch_id: 2026-08-10-am
purpose: OPERATIONAL
frame_id: map
map_content_sha256: sha256:<64 lowercase hex>
```

`purpose` distinguishes at least:

- `EVALUATION`: open-field/RTK truth and algorithm A/B assets
- `OPERATIONAL`: facility/agricultural navigation, semantics and route assets

Existing schema-v1 manifests that predate the V25-09A lineage fields remain readable by legacy registry code; new offline derivations must populate the complete lineage contract.

## Stable `map_content_sha256`

Route compatibility must not use the raw SHA256 of `manifest.yaml`, because `agt_map_manager` legitimately edits lifecycle fields such as `state`, `active` and `pinned` during activation, pinning and archival. The map's capture rig is provenance only; the Route Asset's `vehicle_binding` is the execution vehicle and may be MK-mini even when the map was captured handheld.

V25-09A therefore defines `map_content_sha256` as canonical JSON SHA256 over only stable map content and provenance:

```text
schema_version
map_id / map_version_id / parent_version_id
site_id / epoch_id / purpose / frame_id
source Dataset binding
Calibration binding
Derivation Recipe binding
Alignment binding
Platform binding
processing_backend
navigation metadata
frozen assets table
```

The following registry metadata is intentionally excluded:

```text
state
active
pinned
tags
notes
created_at
```

Therefore:

- activating/deactivating or pinning a map does not invalidate a Route Asset
- changing navigation geometry, localization prior, semantic/coverage content, calibration, recipe or alignment changes the stable content identity or a separately bound asset hash and must fail compatibility checks
- `validate-map` recomputes and checks `map_content_sha256` read-only

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

The Dataset binding uses the field name `calibration_sha256`; the Map Manifest uses the generic `path` + `sha256` artifact form. Both hashes identify the same Calibration Set bytes.

Detailed contracts are in `calibration_dataset_contract.md` and `map_derivation_contract.md`.

## Alignment identity

All assets in one map version share the same canonical `map` frame and alignment lineage:

```yaml
alignment:
  site_frame: alignment/site_frame.yaml
  site_frame_sha256: sha256:<64 lowercase hex>
  record: alignment/alignment.yaml
  record_sha256: sha256:<64 lowercase hex>
```

`alignment/alignment_report.json` becomes a canonical hashed asset once generated and must report `PASS` before promotion to `READY`.

For the same physical site at different epochs, preserve `site_id` and site-frame definition while creating a new `map_version_id`. A later epoch may reference an accepted version for alignment, but stable structures/control points should dominate seasonal vegetation.

## Asset groups

Typical products:

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

Global Navigation Map, Localization Prior, Semantic Map and Route Asset remain distinct products even when they originate from one bag.

## READY quality gate

A formal derived map remains `PROCESSING` while mapping, cleaning, alignment and semantic editing are still changing assets.

The V25-09A gate requires:

- immutable Dataset/Bag bundle, persisted Calibration artifact, Platform Profile, Recipe and Alignment lineage
- navigation `map.yaml/map.pgm`
- ready Localization Prior PCD processing record with exact content hash
- `alignment_report.json` with `PASS`
- `map_quality_report.json` with `PASS`
- frozen canonical asset hashes
- valid `map_content_sha256`

`agt_offline_assets refresh-map --state READY` performs the one-way asset-content promotion. After promotion it refuses to refresh content hashes or rewrite map products. `validate-map` is the read-only compliance audit.

`agt_map_manager` may still update Registry lifecycle metadata such as active/pinned/state. Those administrative changes are not map-content mutations and do not change `map_content_sha256`.

Only a valid READY version can be active. Active identity remains published through `runtime/maps/active_map.yaml` and existing system-health boundaries.

## Route binding

Route readiness is separate from map readiness. A Route Asset binds:

```text
map_id
map_version_id
map_content_sha256
semantic_map sha256
coverage sha256
vehicle profile sha256
route policy sha256
```

It then has its own feasibility and preview evidence. A READY map does not make every route READY.

## Immutability and retention

`pin`, active state, processing state, parent dependencies and experiment references protect versions from retention. Soft delete moves a version to `.trash`; permanent purge is separate.

Legacy `runtime/maps/<name>/` data is not silently rewritten. It must be explicitly packaged and registered.

READY map **content** is immutable: re-alignment, cleaning, calibration change, navigation-map regeneration or semantic edit creates a new map version. Registry lifecycle metadata may change without changing the stable content identity. Route revisions remain separately versioned children and route tuning creates a new revision rather than mutating an accepted Route Asset.
