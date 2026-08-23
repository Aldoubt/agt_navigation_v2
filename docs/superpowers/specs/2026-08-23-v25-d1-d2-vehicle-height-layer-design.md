# V25 D1 + D2 Vehicle Feasibility and Height-Layer Ablation Design

## Status

`EXPERIMENTAL_REVIEW_EVIDENCE` only. This design does not promote a new canonical Navigation Map policy and does not change Accepted Map authority.

## Motivation

A3 removed raster footprint padding, required trusted Ground for terrain HARD classification, and replaced the window-range Step metric with a local-linear discontinuity metric. Real greenhouse review then showed two remaining issues:

1. pixel-level aisle connectivity is not equivalent to vehicle feasibility;
2. the 2D direct-obstacle raster collapses all returns between the active obstacle min/max heights into one occupancy decision and therefore mixes low physical obstacles with higher crop/canopy returns.

D1 and D2 separate these concerns without modifying the formal authority chain.

## D1 — Vehicle-feasible connectivity

### Contract

Formal PGM remains an environment product. D1 is a downstream diagnostic only.

Clearance is constrained by:

- accepted non-FREE environment evidence;
- local lateral aisle boundaries.

Clearance is **not** constrained by virtual longitudinal `u_min/u_max` end caps. Start and end sets are interior terminal bands inset from the geometric aisle extrema.

The default review parameters are:

- required center clearance radius: `0.30 m`;
- terminal inset: `0.50 m`.

These are review proxies, not a final rectangular vehicle-footprint contract.

### Outputs

Each accepted ROW_ROW aisle reports:

- `raster_grid_connectivity`;
- `vehicle_feasible_connectivity`;
- `maximum_clearance_m`;
- `maximum_end_to_end_clearance_radius_m`;
- `minimum_end_to_end_free_width_m`;
- `start_has_free` / `end_has_free`;
- `start_has_feasible` / `end_has_feasible`;
- requested and effective terminal inset;
- `clearance_contract=ENVIRONMENT_PLUS_LATERAL_AISLE_BOUNDARY`.

The report schema is `agt_vehicle_feasible_aisle_audit/v2`.

## D2 — Height-layer obstacle ablation

### Fixed experimental controls

D2 is derived from A3. The following remain frozen:

- source PCD;
- A3 ground surface;
- trusted-ground terrain gating;
- A3 local-linear Step;
- Row hypothesis;
- Corridor hypothesis;
- geometric aisles;
- Site Boundary;
- formal materialization and override logic.

Only vertical direct-obstacle evidence changes.

### Height layers

Using local ground-relative height `h`:

- `LOW`: `[obstacle_min_height, 0.30 m)`;
- `MID`: `[0.30 m, 0.60 m)`;
- `HIGH`: `[0.60 m, obstacle_max_height]`.

The default A3 obstacle range remains `0.12–1.00 m`, so the default concrete layers are LOW `0.12–0.30`, MID `0.30–0.60`, HIGH `0.60–1.00 m`.

The PCD is scanned once and produces three per-cell obstacle-count grids.

### Profiles

- `D2-FULL = LOW + MID + HIGH`;
- `D2-LM = LOW + MID`;
- `D2-L = LOW`.

`D2-FULL` is a control profile and must reproduce A3 `obstacle_count` exactly. The runner fails closed if any grid cell differs, because a non-identical D2-FULL result would invalidate the controlled ablation.

Each D2 revision writes `validation/height_layer_ablation.json` with layer boundaries, interval semantics, total layer point counts, selected layers and selected obstacle point count.

## Scientific interpretation gate

D2 is not judged by how white the PGM becomes. The primary comparison is whether physically different real cases separate correctly:

- canopy/side-crop-like blockers such as reviewed aisle 001/014 should weaken when HIGH evidence is removed if the low collision envelope remains open;
- low/mid physical blockers such as reviewed aisle 005 should remain blocking under D2-L or D2-LM if low structure is physically persistent;
- terrain-only cases such as aisle 017 must remain governed by terrain policy and must not be "fixed" by D2.

Only after this discrimination succeeds may a production obstacle model be proposed.

## Authority boundary

D1 and D2 do not change the authority chain:

`PCD → environment evidence → Generated PGM → human override → Accepted PGM → vehicle feasibility`.

No D1 clearance is baked back into Formal PGM. No D2 profile becomes Accepted authority automatically.

## Real-PCD run

Recommended controlled comparison:

```bash
python3 scripts/run_navigation_ablation.py \
  --pcd "runtime/maps/agt_workbench_0822 night/processed.pcd" \
  --site-boundary "runtime/maps/agt_workbench_0822 night/site_boundary.yaml" \
  --output "runtime/maps/d1_d2_height_layers_a3" \
  --profiles A3 D2-FULL D2-LM D2-L \
  --resolution 0.10 \
  --ground-quantile 0.10 \
  --ground-fill-distance 0.35 \
  --ground-smoothing-radius 2 \
  --obstacle-min-height 0.12 \
  --obstacle-max-height 1.00 \
  --minimum-obstacle-points 2 \
  --maximum-slope-deg 10.0 \
  --maximum-step-m 0.12 \
  --obstacle-padding-m 0.05 \
  --soft-obstacle-max-count 4 \
  --soft-obstacle-max-ratio 0.05 \
  --soft-recovery-max-gap-m 0.60 \
  --vehicle-clearance-radius-m 0.30 \
  --vehicle-terminal-inset-m 0.50 \
  --d2-low-max-height-m 0.30 \
  --d2-mid-max-height-m 0.60 \
  --write-overlays
```

Do not enable D0 3D rescanning for this multi-profile run; D0 evidence has already served its diagnostic purpose and the runner deliberately keeps D0 single-profile.
