# Paper I Greenhouse Real-Map Freeze V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the real `greenhouse_01` Workbench output into an auditable, planner-independent, hash-frozen formal map revision that can safely feed the Paper I 23-cell benchmark.

**Architecture:** Extend the existing V25-12 `agt_offline_assets` navigation-map derivation lineage instead of creating a second override/raster system. The Workbench must export both the generated pre-override map and the accepted post-override map from the same `NavigationMapResult`/override sequence; `agt_route_benchmark` then independently replays and audits that revision, renders QA evidence, binds a curation manifest, and finally freezes a site snapshot only after map, semantics, and platform geometry are explicitly accepted.

**Tech Stack:** Python 3.10, NumPy, PyYAML, Shapely, Matplotlib, ROS 2 Humble, ament/colcon, pytest.

**Spec:** `docs/superpowers/specs/2026-08-17-paper1-route-benchmark-synthetic-real-map-design.md`

## Global Constraints

- Reuse `agt_offline_assets.apply_navigation_overrides()` as the only authoritative override rasterization rule.
- Do not introduce a second incompatible map/site registry; extend V25-12 derivation and Site Package lineage.
- Formal Paper I manual map edits are restricted to `FORCE_FREE` and `FORCE_OCCUPIED`.
- Manual edits must be independent of planner outputs and must preserve id, reason, evidence category, polygon geometry, and immutable hashes.
- The generated pre-override map and accepted post-override map must share identical resolution, origin, size, thresholds, and frame.
- Every changed accepted-map cell must be reproducible from the recorded override sequence; unexplained changed cells are a hard failure.
- READY/formal assets are immutable within one benchmark revision.
- Formal planner execution must not begin until map reliability, semantic correctness, and real MKmini geometry acceptance are all true.
- Real-map QA figures are evidence of map curation only; they do not contain planner paths and must not be conditioned on planner results.

---

## File Structure

**Modify canonical offline derivation**
- `src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py`
- `src/agt_offline_assets/agt_offline_assets/__init__.py`
- `src/agt_offline_assets/test/test_navigation_map_freeze_export.py`

**Modify Workbench authoring/export**
- `src/agt_map_workbench/agt_map_workbench/app.py`
- `src/agt_map_workbench/agt_map_workbench/model.py` only if a pure metadata helper is needed
- `src/agt_map_workbench/test/test_navigation_override_metadata.py`
- `src/agt_map_workbench/CMakeLists.txt`

**Create/extend Paper I audit**
- `src/agt_route_benchmark/agt_route_benchmark/map_quality.py`
- `src/agt_route_benchmark/agt_route_benchmark/map_curation.py`
- `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`
- `src/agt_route_benchmark/test/test_map_quality.py`
- `src/agt_route_benchmark/test/test_map_curation.py`

**Harden formal snapshot**
- `src/agt_route_benchmark/agt_route_benchmark/site_snapshot.py`
- `src/agt_route_benchmark/scripts/route_benchmark_accept_site.py`
- `src/agt_route_benchmark/config/site_acceptance_template.yaml`
- `src/agt_route_benchmark/test/test_site_snapshot.py`

**Integration**
- `src/agt_route_benchmark/CMakeLists.txt`
- `src/agt_route_benchmark/README.md`
- `.github/workflows/paper1-route-benchmark.yml`

---

### Task 1: Export One Replayable Generated/Accepted Map Revision

**Files:**
- Create: `src/agt_offline_assets/test/test_navigation_map_freeze_export.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `.github/workflows/paper1-route-benchmark.yml`

**Interfaces:**
- Reuses: `apply_navigation_overrides(result, overrides) -> np.ndarray`.
- Produces: `write_navigation_map_freeze_bundle(base_result, output_dir, *, source_asset, frame_id, overrides) -> Path`.
- Bundle layout:
  - `generated/navigation_map.pgm`
  - `generated/navigation_map.yaml`
  - `accepted/navigation_map.pgm`
  - `accepted/navigation_map.yaml`
  - `derivation.yaml`

- [ ] **Step 1: Write the failing export contract test**

Construct a tiny `NavigationMapResult` directly so the test does not require SciPy. Use a `FORCE_OCCUPIED` polygon covering exactly one cell-center and assert:

```python
output = write_navigation_map_freeze_bundle(
    base,
    tmp_path / "freeze",
    source_asset="processed.pcd",
    frame_id="map",
    overrides=[{
        "id": "ovr_001",
        "mode": "force_occupied",
        "polygon_xy": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        "reason": "measured support post",
        "evidence_category": "measured_structure",
    }],
)
assert (output / "generated/navigation_map.pgm").is_file()
assert (output / "accepted/navigation_map.pgm").is_file()
record = yaml.safe_load((output / "derivation.yaml").read_text())
assert record["outputs"]["generated"]["pgm_sha256"] != record["outputs"]["accepted"]["pgm_sha256"]
assert record["overrides"][0]["id"] == "ovr_001"
```

Add a no-override case requiring generated and accepted image SHA256 to be identical. Add a repeated-run case requiring byte-identical files for the same input.

- [ ] **Step 2: Make CI execute the new RED test**

Extend Paper I CI with:

```bash
PYTHONPATH=src/agt_offline_assets \
python -m pytest -q src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

Run/observe CI. Expected RED: import/function-not-found for `write_navigation_map_freeze_bundle`.

- [ ] **Step 3: Extract one deterministic Nav2-map writer helper**

Refactor the existing PGM/YAML write block into a private helper that writes the same `P5` PGM, `mode: trinary`, resolution, origin, thresholds, and hashes. Existing `write_navigation_map_derivation()` behavior must remain compatible.

- [ ] **Step 4: Implement the freeze bundle**

Compute the accepted occupancy only through:

```python
accepted_occupancy = apply_navigation_overrides(base_result, overrides)
accepted_result = NavigationMapResult(**{
    **base_result.__dict__,
    "occupancy": accepted_occupancy,
})
```

Write generated and accepted subdirectories with the same geometry metadata. `derivation.yaml` must contain the exact ordered override list and both asset hashes.

- [ ] **Step 5: Verify GREEN**

```bash
PYTHONPATH=src/agt_offline_assets \
python -m pytest -q src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

Then run the existing navigation derivation regression file:

```bash
PYTHONPATH=src/agt_offline_assets \
python -m pytest -q src/agt_offline_assets/test/test_navigation_map_derivation.py
```

- [ ] **Step 6: Commit**

Commit message:

```text
feat(offline-assets): export replayable navigation map revisions
```

---

### Task 2: Make Workbench Overrides Formal-Evidence Ready

**Files:**
- Create: `src/agt_map_workbench/test/test_navigation_override_metadata.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/app.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Workbench keeps calling the existing offline-assets override implementation.
- Each authored Paper I-compatible override mapping must contain:
  - `id`
  - `mode` = `force_free` or `force_occupied`
  - `polygon_xy`
  - `reason`
  - `evidence_category`
- Allowed evidence categories mirror benchmark curation: `pcd_inspection`, `site_photo`, `measured_structure`, `known_permanent_obstacle`, `field_note`, `other_documented`.

- [ ] **Step 1: Write failing pure metadata tests**

Test a small helper that validates/builds one mapping:

```python
record = build_navigation_override_record(
    override_id="ovr_free_001",
    mode="force_free",
    polygon_xy=[[0,0],[1,0],[1,1],[0,1]],
    reason="Sparse-return hole contradicted by PCD review",
    evidence_category="pcd_inspection",
)
assert record["id"] == "ovr_free_001"
```

Require rejection of blank reason, duplicate/blank id, unsupported evidence category, and formal modes `unknown`/`no_go`.

- [ ] **Step 2: Verify RED**

```bash
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets \
python -m pytest -q src/agt_map_workbench/test/test_navigation_override_metadata.py
```

- [ ] **Step 3: Implement the helper without Qt dependencies**

Keep validation in a pure module/helper so tests do not instantiate the GUI. The GUI may still display `unknown/no_go` for non-formal V25 workflows, but Paper I formal export must reject them rather than silently convert them.

- [ ] **Step 4: Wire the Workbench authoring UI**

When finishing an override, obtain a non-empty human reason and selected evidence category, create a unique stable id such as `ovr_0001`, and store that metadata with the world-coordinate polygon. Do not use planner names/results in the record.

- [ ] **Step 5: Change Workbench export to use the freeze bundle**

Call `write_navigation_map_freeze_bundle(self._navigation_base_result, ...)` rather than exporting only `self._navigation_result`. Preserve the exact ordered overrides.

- [ ] **Step 6: Verify GREEN and regressions**

Run the new metadata tests plus existing Map Workbench tests that do not require display interaction. Build the package locally at the later target-machine gate.

- [ ] **Step 7: Commit**

```text
feat(map-workbench): record auditable navigation overrides
```

---

### Task 3: Independently Replay and Audit the Curation Revision

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/map_quality.py`
- Create: `src/agt_route_benchmark/test/test_map_quality.py`
- Modify: `src/agt_route_benchmark/agt_route_benchmark/map_curation.py`
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Consumes generated map, accepted map, and derivation/override evidence.
- Replays recorded polygons with the canonical `agt_offline_assets.apply_navigation_overrides` semantics or an equivalent call through the same package, never a second rasterizer.
- Produces `map_qa_report.json` and `map_curation_qa.svg/.pdf/.png`.

- [ ] **Step 1: Write failing compatibility/diff tests**

Require hard failure for mismatched resolution/origin/size/thresholds. Require exact changed-cell accounting and:

```python
assert report["unexplained_changed_cell_count"] == 0
assert report["accepted_matches_replay"] is True
```

Corrupt one accepted cell outside all overrides and require fail-closed with `unexplained_changed_cell_count == 1`.

- [ ] **Step 2: Verify RED**

```bash
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python -m pytest -q src/agt_route_benchmark/test/test_map_quality.py
```

- [ ] **Step 3: Implement audit metrics**

Report at minimum:

```text
free/occupied/unknown before and after
changed_cell_count
changed_area_m2
changed_fraction
force_free_changed_cell_count
force_occupied_changed_cell_count
unexplained_changed_cell_count
accepted_matches_replay
```

No metric may depend on planner paths.

- [ ] **Step 4: Render planner-independent QA**

Use three panels: generated occupancy, accepted occupancy, and changed-cell/override overlay. Export SVG/PDF/PNG. The figure is for human map acceptance, not a planner result figure.

- [ ] **Step 5: Extend `build_map_curation_manifest()`**

Bind hashes for the derivation record and QA report/figure in addition to source PCD, generated map, accepted map, override evidence, semantic map, and platform profile.

- [ ] **Step 6: Verify GREEN**

Run `test_map_quality.py`, `test_map_curation.py`, `test_map_io.py`, then the Paper I pure-Python suite.

- [ ] **Step 7: Commit**

```text
feat(route-benchmark): audit real map curation revisions
```

---

### Task 4: Harden Site Snapshot Against Uncurated or Unverified Assets

**Files:**
- Modify: `src/agt_route_benchmark/agt_route_benchmark/site_snapshot.py`
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_accept_site.py`
- Modify: `src/agt_route_benchmark/config/site_acceptance_template.yaml`
- Modify: `src/agt_route_benchmark/test/test_site_snapshot.py`

**Interfaces:**
- `create_site_snapshot(...)` gains required `curation_manifest_path` for formal Paper I freeze.
- Acceptance requires all three:
  - `map_reliability_accepted: true`
  - `semantic_correctness_accepted: true`
  - `platform_geometry_accepted: true`

- [ ] **Step 1: Write failing snapshot tests**

Require rejection when platform geometry is false, when the curation manifest accepted-map hash differs from the selected map, when semantic/profile/PCD hashes differ, or when curation QA reports unexplained changes.

- [ ] **Step 2: Verify RED**

```bash
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python -m pytest -q src/agt_route_benchmark/test/test_site_snapshot.py
```

- [ ] **Step 3: Implement curation-manifest cross-binding**

Verify the selected PCD, accepted map YAML/PGM, semantic map, and platform profile exactly match the curation manifest hashes. Bind the curation manifest itself into `assets` and into `snapshot_sha256` identity.

- [ ] **Step 4: Add platform acceptance gate**

Extend the acceptance template and snapshot record with `platform_geometry_accepted`. Do not change `profiles/platforms/mk_mini.yaml` values automatically; the human field-measurement decision remains external evidence.

- [ ] **Step 5: Verify GREEN and formal-plan regressions**

Run `test_site_snapshot.py`, `test_matrix_plan.py`, `test_result_selection.py`, and the full benchmark tests.

- [ ] **Step 6: Commit**

```text
feat(route-benchmark): bind curation evidence into site snapshot
```

---

### Task 5: CLI, Documentation, and Target-Machine Freeze Gate

**Files:**
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_accept_site.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`
- Modify: `src/agt_route_benchmark/README.md`
- Modify: `.github/workflows/paper1-route-benchmark.yml`

**Interfaces:**
- Produces one documented real-map workflow rooted at `runtime/maps/greenhouse_01/`.

- [ ] **Step 1: Document the canonical directory layout**

```text
runtime/maps/greenhouse_01/
├── source/processed.pcd
├── derivation/
│   ├── generated/navigation_map.pgm|yaml
│   ├── accepted/navigation_map.pgm|yaml
│   └── derivation.yaml
├── semantic/semantic_map.geojson
├── semantic/coverage.yaml
└── benchmark/
    ├── map_qa_report.json
    ├── map_curation_qa.svg|pdf|png
    ├── map_curation_manifest.json
    ├── acceptance.yaml
    └── site_snapshot.json
```

- [ ] **Step 2: Provide exact CLI commands**

Document Workbench export, curation audit, human QA review, acceptance edit, and site snapshot creation. State explicitly that no formal planner run may precede `site_snapshot.json`.

- [ ] **Step 3: Run complete cloud-verifiable tests**

```bash
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python -m pytest -q src/agt_route_benchmark/test

PYTHONPATH=src/agt_offline_assets \
python -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py \
  src/agt_offline_assets/test/test_navigation_map_derivation.py
```

Run `compileall` for changed Python packages.

- [ ] **Step 4: Target-machine gate**

On `~/agt_navigation_v2_paper1`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to agt_route_benchmark agt_map_workbench
source install/setup.bash
```

Run isolated package tests. Then export the actual `greenhouse_01` Workbench revision, execute map curation QA, and stop for human acceptance of the QA figure and real MKmini geometry. Do not start formal planner cells yet.

- [ ] **Step 5: Completion condition**

This plan is complete only when a real revision has:

```text
accepted_matches_replay = true
unexplained_changed_cell_count = 0
map_reliability_accepted = true
semantic_correctness_accepted = true
platform_geometry_accepted = true
site_snapshot.json with verified asset hashes
```

Then, and only then, create the next plan for real S01-S06 scenario freezing, State Lattice control-set generation, and the formal 23-cell matrix.
