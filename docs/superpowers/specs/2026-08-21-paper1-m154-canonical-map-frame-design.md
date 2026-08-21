# Paper I M1.5-4 Canonical Map Frame and Grid Freeze Design

Date: 2026-08-21
Branch: `feat/paper1-m154-canonical-map-frame`
Base: `feat/paper1-m1533-terrain-local-row-tracks`

## Goal

Make every Paper I raster layer and route evaluation share the same canonical `map` frame and the same Nav2 grid geometry as the accepted V25 site map. The stage fixes frame/grid lineage before any further row-detector or planner tuning.

## Root cause addressed

The Paper map pipeline currently reads a source PCD and directly calls `derive_ground_relative_navigation_map()` without binding the V25 session-to-site alignment. The derivation can accept a source-to-map transform, but `agt_map_pipeline.prepare_project()` does not pass it. It also derives its own origin and bounds from the PCD, so two pipelines may use different grid geometries even after sharing a physical frame.

The formal Paper `ours` adapter intentionally preserves V25 Route Asset geometry. Therefore route coordinates must remain canonical and must never be patched with renderer offsets.

## Canonical ownership

V25 remains the authority for:

- canonical site frame identity (`map`),
- accepted alignment transform,
- accepted Nav2 map grid geometry,
- platform profile,
- canonical Route Asset coordinates.

Paper I owns derived evidence layers and benchmark artifacts, but consumes the V25 frame/grid contract.

## Interfaces

### Alignment contract

`agt_map_pipeline.canonical_frame.AlignmentSpec` loads a V25 alignment artifact with:

- source frame,
- target/canonical frame,
- method,
- status,
- yaw,
- XYZ translation,
- source file SHA256.

Only `status: PASS` and target frame `map` are accepted for canonical preparation.

### Grid contract

`agt_map_pipeline.canonical_frame.NavigationGridSpec` contains:

- `frame_id`,
- `resolution_m`,
- `origin_x_m`, `origin_y_m`,
- `width`, `height`.

`load_nav2_grid_spec()` derives it from the accepted V25 map YAML and PGM image.

### Canonical preparation boundary

M1.5-4 deliberately does **not** change the mathematical implementation in `agt_offline_assets`.

When canonical inputs are supplied, `agt_map_pipeline` performs:

1. rigid transform of the PCD from source/session coordinates to canonical `map`;
2. conservative crop to canonical map bounds;
3. the existing `derive_ground_relative_navigation_map()` unchanged;
4. deterministic regridding of the resulting arrays onto the canonical Nav2 grid.

Regridding is allowed only when source and target resolutions match and the source-grid origin offset is an integer number of cells within tolerance. Cells not covered by the derived source grid remain conservative defaults: `UNKNOWN` for occupancy, `NaN` for floating terrain fields, and zero for count/support fields.

This boundary keeps the alignment fix auditable and prevents M1.5-4 from silently changing ground classification, row extraction, or obstacle semantics.

## Map pipeline behavior

`agt-map prepare` gains:

- `--alignment <alignment.{yaml,json}>`
- `--canonical-map-yaml <map.yaml>`

They are an atomic pair: supplying only one is an error.

When supplied, `prepare_project()`:

1. verifies the alignment artifact;
2. loads the canonical grid;
3. transforms/crops the PCD into canonical `map` coordinates;
4. derives the existing navigation evidence with unchanged V25/Paper math;
5. regrids every resulting raster layer to the exact canonical grid;
6. records transform/grid identities in `project.yaml`;
7. changes frame verification from `UNVERIFIED` to `VERIFIED`.

This stage does not make candidate rows/aisles accepted truth.

## Frame verification report

Add a deterministic frame verification utility and CLI command:

```text
agt-map verify-frame <project> [--route-csv <route.csv>] [--semantic-map <semantic.geojson>]
```

It writes `evidence/frame_alignment_report.json` and checks:

- project frame is `VERIFIED`;
- canonical frame is `map`;
- canonical grid metadata exists;
- all supplied route XY samples are inside canonical map bounds;
- all supplied semantic GeoJSON coordinates are inside canonical map bounds.

The command fails closed when any required check fails. It does not modify route or semantic coordinates.

## Safety and research invariants

1. Never apply route X/Y offsets to make figures look aligned.
2. Never rotate only the renderer; alignment is applied at the source PCD boundary.
3. Never infer a canonical frame from PCD bounds in formal preparation.
4. Do not modify V25 accepted map files.
5. Do not modify source PCD files.
6. Do not tune M1.5-3.3 row-track parameters in this stage.
7. Do not run planner optimization as part of map preparation.
8. Existing unaligned/development `agt-map prepare` remains available and remains `UNVERIFIED`.
9. A formal Paper site snapshot must later bind the alignment artifact and canonical map hashes before `paper1-v0.1` freeze.

## Acceptance gate

M1.5-4 is ready for target-machine real-PCD acceptance only when:

- canonical prepare produces the exact V25 grid origin/resolution/width/height;
- route-in-bounds ratio is 1.0 for the selected V25 Route Asset;
- semantic-in-bounds ratio is 1.0 for the selected semantic map;
- `frame_alignment_report.json` records PASS;
- an overlay generated by the existing Paper renderer shows V25 map and unmodified V25 route in the same physical location.

Pixel-level equality between the V25 occupancy and Paper evidence map is explicitly not required; their semantics may differ. Spatial frame/grid equality is required.
