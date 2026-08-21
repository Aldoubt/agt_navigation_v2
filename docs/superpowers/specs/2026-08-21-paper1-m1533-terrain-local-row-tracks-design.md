# Paper I M1.5-3.3 Terrain Morphology + Local Row Tracks Design

## Goal

Replace the current single global cross-row peak model with a local, auditable agricultural-structure model that:

1. preserves the verified agricultural frame,
2. reuses the V25 terrain-derivation philosophy (Ground Confidence, robust local slope, local relief),
3. exposes terrain morphology evidence for paper figures,
4. reconstructs crop rows from local observations linked along the agricultural longitudinal axis, and
5. derives aisle candidates only from adjacent row tracks with positive longitudinal overlap and physical width.

This stage remains diagnostic. It does not create an accepted navigation map, run a planner, or set any human-review acceptance flag.

## Research framing

The paper should avoid presenting this as "vegetation removal". The stronger formulation is:

> A binary occupancy projection collapses distinct 3D causes into one 2D obstacle class. Facility-agriculture navigation benefits from separating terrain morphology, vertical occupancy evidence, crop-row structure, and vehicle/task constraints before planning.

The terrain branch explicitly separates:

- **ground confidence**: whether a local ground estimate is supported and locally planar,
- **robust slope**: local plane inclination,
- **signed local relief**: ground height relative to a slowly varying terrain background,
- **ridge evidence**: positive relief associated with raised beds/ridges,
- **depression evidence**: negative relief associated with ruts/depressed ground,
- **step evidence**: abrupt local height change,
- **raw obstacle evidence**: non-ground point support, including posts/walls/vegetation.

These are geometric evidence layers, not semantic truth. In particular, slope or relief alone must not claim that a cell is a pillar, step, crop row, or depression object class.

## V25 reference behavior to preserve

V25-12C already provides useful principles:

- `NavigationStructureConfig` computes Ground Confidence from support and local-plane residual.
- A robust local plane over roughly 0.5 m produces `robust_slope_deg` and residual.
- Raised crop-ridge relief is extracted after removing a slowly varying Gaussian terrain background.
- Row evidence mixes obstacle support and terrain-ridge evidence.
- Longitudinal short-gap repair and minimum-span gates regularize row support.
- Corridor refinement keeps structural row width separate from the vegetation envelope.
- Interior aisle geometry exists only between adjacent accepted rows.
- Ground confidence, slope and obstacle clearance are later safety evidence, not row identity.

M1.5-3.3 reuses these principles but removes the remaining global-profile assumption.

## Architecture

### 1. Terrain morphology evidence

Add a focused public module:

`src/agt_offline_assets/agt_offline_assets/terrain_morphology.py`

It consumes `NavigationMapResult` and optionally existing robust slope / ground confidence arrays. It produces:

- `signed_relief_m`
- `ridge_evidence`
- `depression_evidence`
- `step_evidence`
- `valid_mask`

The background terrain is a confidence-gated Gaussian surface. Positive and negative relief are represented independently. Step evidence is a local gradient / neighbour-height-change proxy over the filled valid ground surface and is confidence-gated.

No semantic label is inferred from these layers.

### 2. NavigationStructureResult exposure

`NavigationStructureResult` keeps existing fields and gains optional diagnostic fields with backward-compatible defaults:

- `hybrid_row_evidence`
- `terrain_ridge_evidence`
- `terrain_depression_evidence`
- `terrain_step_evidence`

`derive_navigation_structure()` populates them. Existing manual test fixtures that construct `NavigationStructureResult` remain valid.

### 3. Local row observations

Add:

`src/agt_offline_assets/agt_offline_assets/navigation_row_tracks.py`

The verified row direction comes from the caller; this module does not auto-author a frame.

Transform map cells into agricultural coordinates `(u, v)`:

- `u`: along-row longitudinal axis,
- `v`: cross-row axis.

Split the observed interior into overlapping longitudinal windows. In each window:

1. build a normalized local cross-row profile from raw `hybrid_row_evidence`,
2. normalize by valid observed support per cross-row bin,
3. lightly smooth in `v`,
4. reject zero / insufficient-support bins,
5. detect local peaks with physical spacing and prominence gates,
6. emit `LocalRowObservation` records.

This prevents one vegetation-dense part of the greenhouse from dominating the complete global profile.

### 4. Row-track association

Associate observations across adjacent longitudinal windows using a deterministic nearest-neighbour gate in cross-row distance.

A track may bridge a bounded number of missing windows. Accepted tracks must satisfy:

- minimum number of observations,
- minimum longitudinal span,
- bounded cross-row drift.

The result is a `RowTrack`, not an infinitely long global line. The track carries:

- stable track id,
- observations,
- `u_min_m`, `u_max_m`,
- representative `v_m`,
- support score,
- longitudinal span.

The raster centreline / structural band is generated only over the active longitudinal range of each accepted track.

### 5. Pair-anchored aisle segments

Aisles are derived from **adjacent accepted row tracks**, never from leftover free space.

For adjacent tracks `R_i`, `R_j`, an aisle exists only where their longitudinal intervals overlap by at least `minimum_pair_overlap_m`.

At each overlapping longitudinal position, the available cross-row interval is:

`left_row_edge + side_clearance` to `right_row_edge - side_clearance`.

If the physical gap is non-positive or narrower than `aisle_minimum_width_m`, the pair is rejected.

Every accepted aisle carries provenance:

- left row track id,
- right row track id,
- longitudinal overlap,
- minimum / median available width,
- status.

No aisle cell may exist without a row-pair provenance object.

### 6. Terrain evidence and aisle review

This stage does **not** silently remove aisle geometry based on terrain morphology. Instead each aisle pair receives review diagnostics derived from:

- minimum Ground Confidence,
- robust slope,
- step evidence,
- depression evidence.

The geometric aisle remains structurally defined by row tracks. Terrain evidence can mark it `REVIEW_TERRAIN` but does not invent or delete row topology.

This separation is important for the paper:

`Agricultural topology` and `terrain executability evidence` are separate constraints.

## Configuration

Add diagnostic defaults only; all remain `formal_ready: false` in the Paper I pipeline until field acceptance.

Suggested core parameters:

- terrain relief background sigma: 0.45 m
- ridge relief scale: 0.08 m
- depression relief scale: 0.08 m
- step evidence scale: 0.10 m
- local row window length: 2.0 m
- local row window stride: 0.75 m
- local profile bin: 0.10 m
- local profile smoothing: 0.15 m
- local minimum peak spacing: 0.55 m
- observation association distance: 0.45 m
- maximum missed windows: 1
- minimum row-track span: 3.0 m
- fixed row structural half width: 0.20 m
- aisle side clearance: 0.12 m
- minimum aisle width: 0.45 m
- minimum row-pair longitudinal overlap: 1.5 m

These are diagnostic defaults, not paper-final physical truths.

## Safety invariants

1. Source PCD and raw `navigation_map.pgm` remain unchanged.
2. This stage never writes `accepted=true`.
3. No planner is run.
4. No row count is predeclared.
5. Periodicity may score row consistency but may not create missing rows.
6. Aisle cells require explicit adjacent row-track provenance.
7. Terrain morphology evidence is geometric evidence only, not semantic object classification.
8. Figures 26/27 remain stale until this stage passes local real-PCD review.

## Paper figure intent

The implementation should support a future progression figure:

1. raw occupancy,
2. Ground Confidence / robust slope,
3. signed relief with ridge/depression/step evidence,
4. local row observations,
5. reconstructed row tracks,
6. pair-anchored aisle segments,
7. later vehicle-aware traversability.

This directly supports the paper argument that collision-free 2D space, agricultural structure, terrain executability and vehicle/task feasibility are distinct layers.
