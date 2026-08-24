# V25 D1.1 / D2.1 / D3 Vehicle-Envelope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make connectivity scope explicit, persist LOW/MID/HIGH vertical evidence as deterministic sidecars, and add a vehicle collision-envelope evaluator without changing Accepted PGM authority.

**Architecture:** D1.1 is an additive semantic cleanup in `vehicle_feasibility.py`. D2.1 extends `HeightLayerObstacleEvidence` with deterministic export/load helpers. D3 is a new downstream evaluator that maps a physical vehicle width + vertical collision range onto the persisted height evidence, derives vehicle-specific review occupancy, and reuses D1.1 for aisle feasibility.

**Tech Stack:** Python 3.10, NumPy, SciPy ndimage, pytest, JSON/NPY sidecars.

**Spec:** `docs/superpowers/specs/2026-08-23-v25-d11-d21-d3-vehicle-envelope-design.md`

## Global Constraints

- Accepted PGM authority is unchanged.
- D2 remains `EXPERIMENTAL_REVIEW_EVIDENCE`.
- D2-LM is not promoted into formal derivation in this plan.
- Vehicle feasibility remains downstream and never mutates Formal PGM.
- Map Frame repair, semantic GUI migration and route-benchmark executable permissions are out of scope.

---

### Task 1: D1.1 connectivity-scope contract

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasibility.py`
- Test: `src/agt_offline_assets/test/test_vehicle_feasibility.py`

**Interfaces:**
- Produces per-aisle keys `connectivity_scope`, `interior_terminal_raster_connectivity`, and compatibility alias `raster_grid_connectivity`.

- [ ] **Step 1: Write failing tests** asserting the explicit scope and alias equality.
- [ ] **Step 2: Run focused pytest** and verify the new assertions fail before implementation.
- [ ] **Step 3: Add additive report fields** without changing widest-path or clearance behavior.
- [ ] **Step 4: Run focused pytest** and verify pass.
- [ ] **Step 5: Commit** `refactor(map): clarify interior-terminal connectivity scope`.

### Task 2: D2.1 persistent vertical-evidence sidecar

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/height_layer_ablation.py`
- Test: `src/agt_offline_assets/test/test_navigation_ablation.py`

**Interfaces:**
- Produces `write_height_layer_evidence_bundle(evidence, navigation, output_dir) -> Path`.
- Produces `load_height_layer_evidence_bundle(path) -> HeightLayerObstacleEvidence`.

- [ ] **Step 1: Write failing round-trip test** using a temporary directory and exact array equality.
- [ ] **Step 2: Run focused pytest** and verify helper import/function failure.
- [ ] **Step 3: Implement deterministic NPY + metadata JSON export** with SHA256, shape, origin, resolution and interval metadata.
- [ ] **Step 4: Implement load-time schema, SHA256 and shape validation**.
- [ ] **Step 5: Run focused pytest** and verify pass.
- [ ] **Step 6: Commit** `feat(map): persist vertical obstacle evidence sidecar`.

### Task 3: D3 vehicle collision-envelope core

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_collision_envelope.py`
- Test: `src/agt_offline_assets/test/test_vehicle_collision_envelope.py`

**Interfaces:**
- Produces `VehicleCollisionEnvelope` dataclass.
- Produces `select_overlapping_height_layers(evidence, envelope) -> tuple[str, ...]`.
- Produces `derive_vehicle_envelope_navigation(a3_navigation, evidence, envelope) -> NavigationMapResult`.
- Produces `build_vehicle_collision_envelope_audit(...) -> dict[str, object]`.

- [ ] **Step 1: Write failing tests** for layer-overlap selection, effective radius, HIGH-only obstacle filtering, and retained LOW obstacle.
- [ ] **Step 2: Run focused pytest** and verify module-not-found/functions-not-found RED.
- [ ] **Step 3: Implement dataclass validation and layer-overlap selection** using interval overlap with `[collision_z_min_m, collision_z_max_m]`.
- [ ] **Step 4: Derive vehicle-specific obstacle-count raster** from selected evidence and preserve A3 terrain semantics using existing D2 reclassification behavior.
- [ ] **Step 5: Reuse D1.1 report builder** with effective radius `half_width + lateral_safety_margin`.
- [ ] **Step 6: Run focused pytest** and verify pass.
- [ ] **Step 7: Commit** `feat(map): add vehicle collision-envelope evaluator`.

### Task 4: Runner integration and additive validation outputs

**Files:**
- Modify: `scripts/run_navigation_ablation.py`
- Test: `src/agt_offline_assets/test/test_navigation_ablation.py`

**Interfaces:**
- Adds CLI `--vehicle-half-width-m`, `--vehicle-lateral-safety-margin-m`, `--vehicle-collision-z-min-m`, `--vehicle-collision-z-max-m`.
- Writes shared `vertical_evidence/` sidecar when D2 evidence is derived.
- Writes `validation/vehicle_collision_envelope.json` for profiles when an envelope is configured.

- [ ] **Step 1: Add failing contract test** for parser/help-visible D3 flags and sidecar/report writer behavior.
- [ ] **Step 2: Run focused pytest** and verify RED.
- [ ] **Step 3: Integrate sidecar export once per run** and record relative bundle path in summary control metadata.
- [ ] **Step 4: Integrate D3 audit per selected profile** without mutating exported Formal revision.
- [ ] **Step 5: Keep legacy `--vehicle-clearance-radius-m` path** when D3 flags are absent.
- [ ] **Step 6: Run focused pytest + compileall** and verify pass.
- [ ] **Step 7: Commit** `feat(map): wire vertical evidence and vehicle envelope into ablation runner`.

### Task 5: CI registration and regression verification

**Files:**
- Modify: `src/agt_offline_assets/CMakeLists.txt` if the package uses explicit pytest registration for the new file.
- Modify: `.github/workflows/paper1-route-benchmark.yml` only to add the new focused test file to the existing offline-assets contract list; do not touch route-benchmark permissions.

**Interfaces:** None; verification only.

- [ ] **Step 1: Register the new D3 test** in existing package/CI test lists.
- [ ] **Step 2: Run/observe offline-assets contracts**, agricultural regressions, Workbench contracts, map-pipeline contracts and compileall.
- [ ] **Step 3: Confirm the only remaining full-workflow failure, if present, is the pre-existing route benchmark executable-bit test.**
- [ ] **Step 4: Commit** `test(map): register D1.1 D2.1 D3 regression coverage`.

## Self-review

- Spec coverage: D1.1, D2.1, D3 core, runner integration and CI are each mapped to one independently reviewable task.
- Placeholder scan: no implementation placeholders or unspecified acceptance criteria remain.
- Type consistency: D2 helpers consume/return `HeightLayerObstacleEvidence`; D3 consumes the same object and returns `NavigationMapResult` plus JSON-serializable audit dictionaries.
