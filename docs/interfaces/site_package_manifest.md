# Site Package Manifest Contract

V25-12A defines the Site Package as the deployment identity root above existing V25-09A Map Version and Route Asset contracts

A Site Package is not a second map registry and does not duplicate map-version internals

The canonical relationship is

```text
Dataset / Calibration / Recipe
        ↓
READY Map Version
        ↓
READY Route revision(s)
        ↓
Site Package root binding
        ↓
future Vehicle Package export / LoadSitePackage capability
```

## 1. Layout

V25-12A stores only the root manifest

```text
runtime/sites/<site_id>/packages/<site_package_id>/
└── manifest.yaml
```

The actual map and Route assets remain in their existing canonical registry

```text
runtime/maps/<map_id>/versions/<map_version_id>/
```

V25-12G may later export a self-contained vehicle package, but V25-12A must not copy full map/route bundles merely to create a second source of truth

## 2. Identity

`site_package_id` format

```text
sitepkg_YYYYMMDD_HHMMSS_<8 hex>
```

Example

```text
sitepkg_20260811_120000_1234abcd
```

A Site Package uses

```yaml
schema_version: 1
site_package_schema: agt_site_package/v1
site_id: greenhouse_a
site_package_id: sitepkg_20260811_120000_1234abcd
state: DRAFT
frame_id: map
```

`state` is lifecycle metadata and is not part of stable package content identity

## 3. Map binding

The Site Package selects an existing compliant READY Map Version by stable identity

```yaml
map_binding:
  map_id: greenhouse_a
  map_version_id: map_20260810_120000_1234abcd
  map_content_sha256: sha256:<64 lowercase hex>
```

No raw map-manifest path is required in the Site Package contract

Offline validation resolves the selected version through a controlled `maps_root`

Runtime implementations should resolve the same identity through the project map/site registry rather than reconstructing arbitrary filesystem paths in BT or GUI code

The selected Map Version must

- be `READY`
- pass existing `validate-map` compliance
- have matching `site_id`
- use `frame_id=map`
- match exact `map_content_sha256`

## 4. Calibration binding

Calibration is inherited from the selected READY Map Version rather than copied into a second calibration schema

```yaml
calibration_binding:
  calibration_id: cal_bunker_mid360_20260810
  sha256: sha256:<64 lowercase hex>
```

Validation requires exact agreement with the selected map manifest

## 5. Localization binding

V25-12A binds the existing frozen localization PCD and processing record

```yaml
localization_binding:
  map_asset_id: localization_pcd
  map_sha256: sha256:<64 lowercase hex>
  processing_asset_id: processing_record
  processing_sha256: sha256:<64 lowercase hex>
```

This is not yet the V25-12D Localization Prior schema

Future `localization_prior` metadata may be added as a separately versioned optional binding without changing the meaning of the localization-map bytes

## 6. Navigation binding

```yaml
navigation_binding:
  yaml_asset_id: navigation_yaml
  yaml_sha256: sha256:<64 lowercase hex>
  pgm_asset_id: navigation_pgm
  pgm_sha256: sha256:<64 lowercase hex>
```

Both hashes must match the selected READY Map Version assets table

## 7. Semantic binding

When a READY map contains a frozen Semantic Map, the Site Package binds it explicitly

```yaml
semantic_binding:
  map_asset_id: semantic_map
  map_sha256: sha256:<64 lowercase hex>
  coverage_asset_id: semantic_coverage
  coverage_sha256: sha256:<64 lowercase hex>
```

A map with no semantic product remains valid for the first MAP-baseline Site Package, but validation reports the absence as a warning

V25-12E will make semantic/task/route authoring a first-class workflow rather than changing V25-12A map compatibility rules

## 8. Execution vehicle binding

The execution vehicle is independent of the map capture rig

```yaml
vehicle_binding:
  platform_id: bunker
  platform_profile_sha256: sha256:<64 lowercase hex>
```

This distinction is required because a map may be captured by

- the target vehicle
- another vehicle
- a handheld MID360 rig
- a future mobile scanner

The Map Version preserves capture provenance while the Site Package selects the vehicle that will execute the task

## 9. Route bindings

Routes are optional in the first MAP-baseline package

When selected, every route must already be a canonical READY Route revision under the selected Map Version

```yaml
routes:
  - route_id: inspection_main
    revision: 1
    route_yaml_sha256: sha256:<64 lowercase hex>
```

Validation requires

- canonical route location under the selected Map Version
- `status=READY`
- exact map ID/version/content-hash binding
- exact selected execution-vehicle profile/hash binding
- valid hashes for route CSV, Route Policy, feasibility report and preview
- `feasibility_report.status=PASS`

The Site Package therefore does not re-run or redefine Route feasibility semantics; it binds accepted Route evidence

## 10. Benchmark bindings

V25-12A reserves

```yaml
benchmark_bindings: []
```

The list must remain structurally valid but non-empty benchmark semantics are deferred to V25-12F

This prevents V25-12A from inventing a premature benchmark schema while preserving a stable extension point

## 11. Stable content identity

`site_package_content_sha256` is canonical JSON SHA256 over stable binding content

Included

```text
schema_version
site_package_schema
site_id
site_package_id
frame_id
map_binding
calibration_binding
localization_binding
navigation_binding
semantic_binding when present
vehicle_binding
routes
localization_prior when present in a future version-compatible extension
benchmark_bindings
```

Excluded lifecycle/annotation metadata

```text
state
created_at
notes
```

Therefore promoting DRAFT to READY does not change the stable package content identity

Any map/vehicle/route binding change does change the content identity

## 12. READY immutability

A Site Package is created as `DRAFT`

Promotion sequence

```text
create-site-package
        ↓
validate all bound READY assets
        ↓
refresh-site-package --state READY
        ↓
READY Site Package
```

After READY promotion

- `refresh-site-package` refuses mutation
- content changes require a new `site_package_id`
- `validate-site-package` remains read-only

Registry/display lifecycle metadata may be designed later, but it must not silently change package content identity

## 13. CLI

Create

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py create-site-package \
  --sites-root runtime/sites \
  --map-manifest runtime/maps/greenhouse_a/versions/<map_version>/manifest.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --route-dir runtime/maps/greenhouse_a/versions/<map_version>/routes/inspection_main/1
```

Promote

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py refresh-site-package \
  --manifest runtime/sites/greenhouse_a/packages/<site_package_id>/manifest.yaml \
  --maps-root runtime/maps \
  --platform-profile profiles/platforms/bunker.yaml \
  --state READY
```

Audit

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-site-package \
  --manifest runtime/sites/greenhouse_a/packages/<site_package_id>/manifest.yaml \
  --maps-root runtime/maps \
  --platform-profile profiles/platforms/bunker.yaml
```

## 14. Runtime / BT boundary

V25-12A adds no runtime ROS interface

A future `LoadSitePackage` capability must consume project-level Site Package identity and validation evidence

BT must not

- open arbitrary filesystem map paths
- select the newest map directory
- parse Map Manifest/Route internals itself
- mutate READY assets

The future runtime capability resolves and validates the package, then publishes/activates state through the owning project managers
