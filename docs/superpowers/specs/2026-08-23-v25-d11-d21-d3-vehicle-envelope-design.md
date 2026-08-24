# V25 D1.1 / D2.1 / D3 Vehicle-Envelope Design

## Goal

Separate environment evidence from vehicle-specific traversability without changing Accepted PGM authority. Preserve LOW/MID/HIGH ground-relative obstacle evidence as review sidecars, make the two connectivity scopes explicit, and replace the circular 0.30 m review proxy with a vehicle collision-envelope evaluator.

## Design constraints

- `V25 Map Workbench accepted revision` remains the only formal Paper navigation-map authority.
- A3 remains the current production candidate for Ground/terrain semantics; D2 profiles remain `EXPERIMENTAL_REVIEW_EVIDENCE`.
- D2-LM is the leading experimental vertical-evidence candidate, but no fixed `obstacle_max_height_m=0.60` is promoted into formal derivation by this phase.
- LOW/MID/HIGH evidence is preserved rather than deleting upper layers.
- Vehicle feasibility is a downstream diagnostic/derived product and never inflates or mutates the Formal PGM.
- Full-extent Formal QA connectivity and inset-terminal vehicle-review connectivity are distinct metrics and must be named distinctly.
- Existing Map Frame authority mismatch is out of scope for this phase.

## D1.1 — connectivity semantic cleanup

The existing D1 evaluator keeps the corrected lateral-boundary clearance and inset start/end terminal bands. Rename/report its raster metric as `interior_terminal_raster_connectivity` and add `connectivity_scope=INTERIOR_TERMINAL_BANDS`. Keep a compatibility alias `raster_grid_connectivity` for one transition period so old JSON consumers do not break.

Formal/QA `connected_interior_aisles` continues to mean full geometric extent. No attempt is made to force both values to match.

## D2.1 — persistent vertical evidence sidecar

`HeightLayerObstacleEvidence` becomes exportable as a deterministic sidecar bundle under each controlled revision:

```text
validation/vertical_evidence/
├── metadata.json
├── low_count.npy
├── mid_count.npy
└── high_count.npy
```

`metadata.json` records schema, grid frame/resolution/origin/shape, LOW/MID/HIGH interval contract, point counts and SHA256 for each array. Arrays are integer count rasters aligned 1:1 with the Navigation Grid. The sidecar is evidence only and is not map authority.

The runner writes the bundle once from the shared D2 evidence scan and records the relative path in `ablation_summary.json`. `D2-FULL`, `D2-LM`, and `D2-L` continue to consume the same in-memory evidence object.

## D3 — vehicle collision-envelope evaluator

Introduce a vehicle model independent of map generation:

```python
VehicleCollisionEnvelope(
    half_width_m: float,
    lateral_safety_margin_m: float = 0.0,
    collision_z_min_m: float = 0.0,
    collision_z_max_m: float = 0.60,
)
```

For the current aisle-aligned review, the effective lateral clearance radius is `half_width_m + lateral_safety_margin_m`. The evaluator selects vertical evidence layers whose height intervals overlap `[collision_z_min_m, collision_z_max_m]`, derives a vehicle-specific obstacle-count raster, reuses A3 trusted-ground terrain occupancy, materializes a derived environment occupancy for review, and calls D1.1 clearance/connectivity on that derived occupancy.

This phase intentionally does not model yaw-dependent rectangular swept area or Ackermann turning envelopes. Those are a later extension after the static aisle-aligned envelope is validated. D3 therefore replaces the arbitrary circular-radius proxy with an explicit physical vehicle-width + vertical-collision contract, while keeping YAGNI on orientation/swept-volume planning.

## D3 outputs

Each D3 run writes:

```text
validation/vehicle_collision_envelope.json
```

with envelope parameters, selected vertical layers, effective lateral radius, interior-terminal connectivity count, vehicle-feasible count, and per-aisle D1.1 reports.

The runner CLI accepts:

```text
--vehicle-half-width-m
--vehicle-lateral-safety-margin-m
--vehicle-collision-z-min-m
--vehicle-collision-z-max-m
```

If these are omitted, the legacy `--vehicle-clearance-radius-m` proxy remains available for compatibility and the D3 report is not produced.

## Verification

TDD gates:

1. D1.1 reports both explicit scope and compatibility alias with identical values.
2. D2.1 round-trips LOW/MID/HIGH arrays and validates SHA256/shape metadata.
3. D3 chooses overlapping vertical layers deterministically and computes `half_width + margin` as the required lateral clearance.
4. A HIGH-only obstacle is ignored by an envelope ending below 0.60 m but retained by an envelope reaching 1.00 m.
5. Existing A3/D2 Formal-map behavior and authority files remain byte/field compatible except for additive validation metadata.

## Non-goals

- No Accepted PGM promotion of D2-LM.
- No direct PGM pixel mutation.
- No FORCE_FREE use to improve connectivity.
- No route-benchmark executable-bit cleanup.
- No Map Frame contract repair.
- No semantic-GUI migration in this phase.
