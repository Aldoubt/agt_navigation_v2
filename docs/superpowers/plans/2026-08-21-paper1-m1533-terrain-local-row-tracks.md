# Paper I M1.5-3.3 Terrain Morphology + Local Row Tracks Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add terrain morphology evidence, local row observations/tracks, and pair-anchored aisle segments while preserving the verified agricultural frame and raw navigation assets.

**Architecture:** Keep V25's terrain and hybrid-row principles, but replace the single global row-profile assumption with overlapping agricultural-coordinate windows. Reconstruct crop rows as longitudinal tracks from local observations, then derive aisle segments only from adjacent accepted tracks with physical overlap and width.

**Tech Stack:** Python 3.10, NumPy, SciPy, pytest, ROS 2 Humble `ament_cmake_pytest`.

**Spec:** `docs/superpowers/specs/2026-08-21-paper1-m1533-terrain-local-row-tracks-design.md`

## Global Constraints

- Do not mutate source PCD or raw Navigation Map artifacts.
- Do not run planners, coverage, or freeze logic.
- Do not set human acceptance flags.
- Terrain morphology is geometric evidence, not semantic truth.
- Aisles require explicit adjacent row-track provenance.
- Preserve backward compatibility of existing V25 structure/corridor APIs.

---

### Task 1: Terrain morphology evidence

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/terrain_morphology.py`
- Create: `src/agt_offline_assets/test/test_terrain_morphology.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_structure.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Produces `TerrainMorphologyConfig`, `TerrainMorphologyResult`, `derive_terrain_morphology()`.
- `derive_navigation_structure()` exposes optional raw hybrid/terrain diagnostic arrays.

- [ ] **Step 1: Write failing terrain tests**

Cover a flat plane, positive ridge, negative depression, and abrupt step. Assert that ridge/depression signs do not leak into each other and that invalid/unobserved cells remain zero evidence.

- [ ] **Step 2: Run targeted test and confirm RED**

Run:

```bash
PYTHONPATH=src/agt_offline_assets python3 -m pytest -q \
  src/agt_offline_assets/test/test_terrain_morphology.py
```

Expected: import/contract failure because the new public module does not exist.

- [ ] **Step 3: Implement terrain morphology**

Use confidence-gated filled ground height. Compute a Gaussian background, signed relief, positive ridge evidence, negative depression evidence, and local step-gradient evidence. Keep values deterministic and clipped to `[0, 1]` where appropriate.

- [ ] **Step 4: Expose diagnostics from `NavigationStructureResult`**

Append optional fields with defaults so existing manual fixture construction remains source-compatible:

```python
hybrid_row_evidence: np.ndarray | None = None
terrain_ridge_evidence: np.ndarray | None = None
terrain_depression_evidence: np.ndarray | None = None
terrain_step_evidence: np.ndarray | None = None
```

Populate them in `derive_navigation_structure()`.

- [ ] **Step 5: Run tests and commit**

Expected terrain tests PASS and existing `test_navigation_structure.py` still passes.

Commit:

```bash
git commit -m "feat(offline-assets): expose terrain morphology evidence"
```

---

### Task 2: Local row observations and tracks

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/navigation_row_tracks.py`
- Create: `src/agt_offline_assets/test/test_navigation_row_tracks.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes `NavigationMapResult`, `NavigationStructureResult`, and verified `row_direction_xy`.
- Produces `LocalRowTrackingConfig`, `LocalRowObservation`, `RowTrack`, `LocalRowTrackResult`, `derive_local_row_tracks()`.

- [ ] **Step 1: Write failing local-window tests**

Synthetic cases must prove:

1. rows visible only in different longitudinal halves are both recovered,
2. one dense region cannot suppress rows in a sparse clean region,
3. zero-valid-support bins cannot create observations,
4. a strong outer boundary is excluded from crop-row tracks,
5. short isolated observations fail the minimum-span gate,
6. a bounded one-window dropout can be bridged,
7. row-track raster extent stops at the observed longitudinal support instead of becoming an infinite line.

- [ ] **Step 2: Run targeted RED test**

Expected import failure because `navigation_row_tracks` does not exist.

- [ ] **Step 3: Implement local profile extraction**

Transform grid cells to `(u, v)` from the verified row direction. Slide overlapping windows in `u`. In each window, form normalized profile from `structure.hybrid_row_evidence` divided by valid observed support per `v` bin, smooth lightly, and detect support-gated peaks.

- [ ] **Step 4: Implement deterministic track association**

Associate adjacent-window observations by nearest cross-row distance under `association_distance_m`, with bounded missed windows. Reject tracks below minimum observations/span. Do not use global row count or periodic completion.

- [ ] **Step 5: Rasterize row-track centerlines and fixed structural bands**

Interpolate cross-row track position only over each track's active `u` range.

- [ ] **Step 6: Run tests and commit**

Commit:

```bash
git commit -m "feat(offline-assets): reconstruct local agricultural row tracks"
```

---

### Task 3: Pair-anchored aisle segments

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_row_tracks.py`
- Modify: `src/agt_offline_assets/test/test_navigation_row_tracks.py`

**Interfaces:**
- Adds `AisleSegmentDiagnostic` and aisle candidate/centerline outputs to `LocalRowTrackResult` or a focused companion result.

- [ ] **Step 1: Write failing aisle provenance tests**

Cover:

1. no aisle without two adjacent accepted row tracks,
2. no aisle when row-track longitudinal overlap is too short,
3. no aisle when physical gap after structural width/side clearance is non-positive,
4. accepted aisle carries left/right row ids and width diagnostics,
5. aisle raster exists only over the pair's longitudinal overlap,
6. three row tracks create exactly two interior row-pair candidates, never one large leftover-free-space rectangle.

- [ ] **Step 2: Run RED test**

Expected failure because pair-anchored aisle APIs are absent.

- [ ] **Step 3: Implement adjacent-track pairing**

Sort accepted tracks by representative cross-row coordinate. Pair only neighbours. Compute overlap interval and varying/interpolated row centres. Build available gap from fixed row structural half width plus side clearance.

- [ ] **Step 4: Add terrain review diagnostics**

For each accepted geometric aisle, summarize Ground Confidence, robust slope, terrain step evidence and depression evidence. Terrain may mark `REVIEW_TERRAIN` but must not author row topology or silently delete geometry.

- [ ] **Step 5: Run tests and commit**

Commit:

```bash
git commit -m "feat(offline-assets): derive pair-anchored aisle segments"
```

---

### Task 4: Regression and CI gate

**Files:**
- Modify: `.github/workflows/paper1-route-benchmark.yml`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Registers new tests for direct pytest and `colcon test`.

- [ ] **Step 1: Add targeted tests to Paper-I CI**

Run terrain morphology and local row-track suites before broad benchmark tests.

- [ ] **Step 2: Run complete cloud verification**

Required gates:

```bash
PYTHONPATH=src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_terrain_morphology.py \
  src/agt_offline_assets/test/test_navigation_structure.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_navigation_row_tracks.py

python3 -m compileall -q src/agt_offline_assets/agt_offline_assets
```

Then the existing Paper-I workflow must pass.

- [ ] **Step 3: Keep PR draft and document local acceptance**

The cloud branch does not contain the user's newer local M1.5 pipeline. The PR stays draft. Local Codex must semantically integrate the shared algorithms, preserve the manual agricultural frame, generate new diagnostic figures, and keep downstream traversability stale until human review.

---

## Self-review

- Spec coverage: terrain morphology, local row observations, row tracks, pair-anchored aisles, terrain review diagnostics, safety invariants and cloud/local split are each assigned to a task.
- No planner/freeze/human-acceptance path is introduced.
- Existing V25 APIs remain backward compatible through optional fields/default-zero configuration.
- No row count or periodic completion rule is introduced.
