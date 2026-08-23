# Paper I M1.5-4B V25 Map Authority Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the immutable V25 Map Workbench accepted revision the only formal navigation-map authority consumed by Paper I while retaining Paper alignment/evidence derivation as non-authoritative analysis.

**Architecture:** Add a small `map_authority` binding module that validates a complete V25 revision and records exact hashes/grid metadata in `project.yaml`. Formal `agt-map prepare` consumes that binding, writes Paper-derived occupancy only as candidate evidence, and never registers a READY Paper Nav2 map. Status, frame verification, and Paper site-snapshot gates fail closed when the binding is absent or mismatched.

**Tech Stack:** Python 3.10, pytest, PyYAML, NumPy, existing `agt_offline_assets`, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-08-22-paper1-m154b-v25-map-authority-recovery-design.md`

## Global Constraints

- Do not modify V25 ground-estimation, row/corridor mathematics, V25-12F safety semantics, planner algorithms, or Route Asset geometry.
- Do not rewrite history of `feat/paper1-m154-canonical-map-frame`.
- Formal Paper map authority is exactly `V25_MAP_WORKBENCH / BOUND_VERIFIED`.
- M1.5-4A alignment/regrid utilities remain available as evidence utilities.
- Development prepare without a V25 revision may remain, but must be explicitly non-formal and cannot satisfy Paper freeze/snapshot gates.
- No Paper-derived occupancy PGM/YAML may be registered as the formal navigation map in formal mode.

---

### Task 1: V25 revision binding contract

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/map_authority.py`
- Create: `src/agt_map_pipeline/test/test_map_authority.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/project.py`
- Test: `src/agt_map_pipeline/test/test_project_contract.py`

**Interfaces:**
- Consumes: V25 revision directory containing `generated/navigation_map.yaml`, `accepted/navigation_map.yaml`, both referenced PGM files, and `derivation.yaml`.
- Produces: `bind_v25_map_revision(revision_dir) -> dict` with schema `agt_v25_map_authority_binding/v1`, authority `V25_MAP_WORKBENCH`, status `BOUND_VERIFIED`, exact asset hashes, and accepted grid geometry.
- Produces: `verify_bound_map_authority(binding) -> dict` which re-hashes bound assets and fails on drift.

- [ ] **Step 1: Write failing map-authority tests**

```python
def test_bind_v25_revision_records_exact_hashes_and_grid(tmp_path):
    revision = make_v25_revision(tmp_path)
    binding = bind_v25_map_revision(revision)
    assert binding["authority"] == "V25_MAP_WORKBENCH"
    assert binding["status"] == "BOUND_VERIFIED"
    assert binding["frame_id"] == "map"
    assert binding["grid"]["width"] == 4
    assert len(binding["accepted_map_yaml_sha256"]) == 64


def test_bound_authority_rejects_asset_drift(tmp_path):
    revision = make_v25_revision(tmp_path)
    binding = bind_v25_map_revision(revision)
    Path(binding["accepted_map_yaml_path"]).write_text("changed")
    with pytest.raises(MapAuthorityError, match="hash mismatch"):
        verify_bound_map_authority(binding)
```

- [ ] **Step 2: Run CI/pytest and verify RED**

Run: `python -m pytest -q src/agt_map_pipeline/test/test_map_authority.py src/agt_map_pipeline/test/test_project_contract.py`
Expected: FAIL because `map_authority` module/binding contract does not exist.

- [ ] **Step 3: Implement minimal binding/validation**

Implement strict revision structure checks, accepted YAML parsing, PGM dimension parsing, frame/resolution/origin/grid extraction, SHA256 recording, and drift verification. Add `map_authority: None` to new project manifests and validate a present binding's schema/authority/status.

- [ ] **Step 4: Run targeted tests and verify GREEN**

Run: same pytest command.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/agt_map_pipeline/agt_map_pipeline/map_authority.py src/agt_map_pipeline/agt_map_pipeline/project.py src/agt_map_pipeline/test/test_map_authority.py src/agt_map_pipeline/test/test_project_contract.py
git commit -m "fix(map-pipeline): bind V25 map authority"
```

### Task 2: Formal prepare consumes authority and demotes Paper occupancy

**Files:**
- Modify: `src/agt_map_pipeline/agt_map_pipeline/prepare.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/layer_io.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/cli.py`
- Modify: `src/agt_map_pipeline/test/test_prepare.py`
- Modify: `src/agt_map_pipeline/test/test_prepare_e2e.py`

**Interfaces:**
- `prepare_project(..., v25_map_revision=None, alignment_path=None, canonical_map_yaml=None)` remains backward compatible for development mode.
- Formal mode is selected by `v25_map_revision` and records `map_authority` from Task 1.
- In formal mode, frame/grid metadata come from the accepted map in the bound revision; `canonical_map_yaml` is not needed.
- Paper-derived occupancy is written under `layers/evidence/` and registered as `evidence.navigation_occupancy` with status `CANDIDATE_EVIDENCE`.

- [ ] **Step 1: Write failing formal-prepare tests**

```python
def test_formal_prepare_binds_v25_authority_and_has_no_ready_paper_nav_map(...):
    result = prepare_project(..., v25_map_revision=revision, alignment_path=alignment)
    project = load_project(result.project_dir)
    assert project["map_authority"]["status"] == "BOUND_VERIFIED"
    assert "navigation.nav2_yaml" not in project["layers"]
    assert "navigation.nav2_pgm" not in project["layers"]
    assert project["layers"]["evidence.navigation_occupancy"]["status"] == "CANDIDATE_EVIDENCE"


def test_formal_prepare_uses_bound_accepted_grid(...):
    ...
    assert project["frame"]["grid"] == project["map_authority"]["grid"]
```

- [ ] **Step 2: Run targeted tests and verify RED**

Run: `python -m pytest -q src/agt_map_pipeline/test/test_prepare.py src/agt_map_pipeline/test/test_prepare_e2e.py`
Expected: FAIL because `v25_map_revision` formal mode and `CANDIDATE_EVIDENCE` do not exist.

- [ ] **Step 3: Implement minimal formal-mode behavior**

Add `--v25-map-revision`; bind authority before derivation; use bound accepted grid for transform/crop/regrid; change formal-mode layer writer to avoid Nav2 map export and store occupancy preview/evidence only; extend layer status enum with `CANDIDATE_EVIDENCE`. Preserve existing development behavior without the flag.

- [ ] **Step 4: Run targeted tests and verify GREEN**

Run: same pytest command.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/agt_map_pipeline/agt_map_pipeline/prepare.py src/agt_map_pipeline/agt_map_pipeline/layer_io.py src/agt_map_pipeline/agt_map_pipeline/cli.py src/agt_map_pipeline/test/test_prepare.py src/agt_map_pipeline/test/test_prepare_e2e.py
git commit -m "fix(map-pipeline): make formal prepare consume V25 map"
```

### Task 3: Fail-closed status and frame verification

**Files:**
- Modify: `src/agt_map_pipeline/agt_map_pipeline/status.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/frame_verification.py`
- Modify: `src/agt_map_pipeline/test/test_status_cli.py`
- Modify: `src/agt_map_pipeline/test/test_frame_verification.py`

**Interfaces:**
- Formal project status exposes `map_authority` summary and blocks on authority hash drift.
- `build_frame_alignment_report()` uses `project.map_authority.grid` as the formal bounds source and confirms it agrees with mirrored `frame.grid`.

- [ ] **Step 1: Write failing status/frame tests**

```python
def test_status_blocks_when_bound_v25_asset_hash_changes(...):
    ...
    assert status["project_state"] == "BLOCKED"
    assert status["block_code"] == "BLOCKED_V25_MAP_HASH_MISMATCH"


def test_frame_report_uses_bound_map_authority_grid(...):
    report = build_frame_alignment_report(project)
    assert report["map_authority"] == "V25_MAP_WORKBENCH"
    assert report["grid"]["map_yaml_sha256"] == project_doc["map_authority"]["accepted_map_yaml_sha256"]
```

- [ ] **Step 2: Verify RED in CI/pytest**

Run: `python -m pytest -q src/agt_map_pipeline/test/test_status_cli.py src/agt_map_pipeline/test/test_frame_verification.py`
Expected: FAIL on missing authority checks/report fields.

- [ ] **Step 3: Implement minimal fail-closed behavior**

Verify bound hashes before normal layer checks; return exact map-authority block codes; read grid from authority in formal projects; keep old verified-frame path for development projects.

- [ ] **Step 4: Verify GREEN**

Run: same pytest command.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/agt_map_pipeline/agt_map_pipeline/status.py src/agt_map_pipeline/agt_map_pipeline/frame_verification.py src/agt_map_pipeline/test/test_status_cli.py src/agt_map_pipeline/test/test_frame_verification.py
git commit -m "fix(map-pipeline): fail closed on map authority drift"
```

### Task 4: Paper site snapshot rejects unbound/mismatched map authority

**Files:**
- Modify: `src/agt_route_benchmark/agt_route_benchmark/site_snapshot.py`
- Modify: `src/agt_route_benchmark/test/test_site_snapshot.py`
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_accept_site.py` only if CLI plumbing is required.

**Interfaces:**
- `create_site_snapshot(..., map_project_path=None, ...)` accepts the formal map project used to bind authority.
- When `map_project_path` is supplied, snapshot creation requires `V25_MAP_WORKBENCH / BOUND_VERIFIED` and the selected `map_yaml_path` SHA must equal `accepted_map_yaml_sha256`.
- Existing synthetic/development snapshot callers without a map project remain backward compatible and are explicitly non-authority-gated.

- [ ] **Step 1: Write failing snapshot tests**

```python
def test_formal_snapshot_rejects_unbound_map_project(...):
    with pytest.raises(ValueError, match="map authority"):
        create_site_snapshot(..., map_project_path=project)


def test_formal_snapshot_rejects_map_different_from_bound_v25_asset(...):
    with pytest.raises(ValueError, match="accepted map"):
        create_site_snapshot(..., map_project_path=project, map_yaml_path=other_map)
```

- [ ] **Step 2: Verify RED**

Run: `python -m pytest -q src/agt_route_benchmark/test/test_site_snapshot.py`
Expected: FAIL because snapshot has no map-project authority gate.

- [ ] **Step 3: Implement the minimal gate**

Load `project.yaml`, validate the authority binding, compare the selected map YAML hash, and record the map-authority identity in the snapshot so its checksum covers the authority identity.

- [ ] **Step 4: Verify GREEN plus package regression**

Run:

```bash
python -m pytest -q src/agt_map_pipeline/test
python -m pytest -q src/agt_route_benchmark/test
python -m compileall -q src/agt_map_pipeline/agt_map_pipeline src/agt_route_benchmark/agt_route_benchmark src/agt_route_benchmark/scripts
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/agt_route_benchmark/agt_route_benchmark/site_snapshot.py src/agt_route_benchmark/test/test_site_snapshot.py src/agt_route_benchmark/scripts/route_benchmark_accept_site.py
git commit -m "fix(paper1): gate site snapshot on V25 map authority"
```

### Task 5: CI and handoff acceptance

**Files:**
- Create: `docs/superpowers/handoffs/2026-08-22-paper1-m154b-v25-map-authority-recovery.md`

**Interfaces:**
- CI must execute the existing Paper I workflow without workflow relaxation.
- Handoff records exact commit, test results, remaining real-map human gate, and explicitly keeps M1.5-5 paused.

- [ ] **Step 1: Open/update draft PR to trigger existing Paper I CI**

Target the historical Paper branch only as a review/CI comparison base; do not merge automatically.

- [ ] **Step 2: Verify workflow jobs**

Expected: `pure-python-contracts` PASS, including all map-pipeline tests, route-benchmark tests, and compileall.

- [ ] **Step 3: Write handoff**

Record that code recovery is complete only at SOFTWARE/CI level; real map acceptance still requires a Workbench-exported immutable revision, map-only QA, human review, and a real Paper project bound to that revision.

- [ ] **Step 4: Keep Paper downstream development paused**

Do not start M1.5-5 until the real accepted V25 revision is bound and visual/QA acceptance passes.
