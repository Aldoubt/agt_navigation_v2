# AGT Map Pipeline Plan A — Core Project + Prepare/Status Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `agt_map_pipeline` so a user or Codex can provide one PCD and deterministically obtain a self-describing greenhouse map project with terrain/navigation/structure candidate layers, then query a stable `WAITING_HUMAN_REVIEW` status without running any planner.

**Architecture:** Add a small orchestration-only ROS2/Python package. It owns project/preset/layer/status contracts and calls the existing `agt_offline_assets` algorithms as the single mathematical authority. `prepare` creates candidate evidence only; official bounded traversability remains blocked until a human-authored Site Boundary exists.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, Python 3.10, ament_cmake + ament_cmake_python, NumPy, SciPy, PyYAML, Shapely, existing `agt_offline_assets`.

**Spec:** `docs/superpowers/specs/2026-08-19-agt-map-pcd-layer-pipeline-design.md`

## Global Constraints

- Work only in `~/agt_navigation_v2_paper1` on branch `feat/paper1-agri-route-benchmark`; do not modify/build/clean `~/agt_navigation_v2`.
- Strict RED -> GREEN for every task; preserve the failing-test evidence in the implementation report.
- Do not run or import A*, Theta*, Hybrid A*, State Lattice, Fields2Cover, Paper I proposed planners, or planner-result code from this subsystem.
- `agt_offline_assets` remains the algorithm authority. Do not duplicate ground estimation, row detection, corridor extraction, bounded traversability recovery, or aisle-graph math.
- The source PCD is referenced by absolute path and SHA256; never copy a multi-GB PCD into the project by default.
- The declared frame defaults to `map`, but `frame_verification` starts as `UNVERIFIED`; Plan A never promotes it automatically.
- Automatic `prepare` never creates or accepts a final Site Boundary, headland, no-go polygon, coverage task, or final semantic truth.
- Before a READY Site Boundary exists, traversability output is only `CANDIDATE_UNBOUNDED` visualization/evidence and performs no UNKNOWN recovery.
- Built-in preset values are resolved and frozen into `config/resolved_preset.yaml`; future code-default changes must not silently alter an existing project.
- Automatic parameters must never be selected after looking at planner output.
- Stage outputs use relative project paths and SHA256 records; no stage becomes READY merely because a file exists.
- A later-stage failure must preserve valid earlier generated artifacts.
- A destination containing an existing project is rejected unless `--resume` is supplied. `--resume` may only resume the same project/source/preset identity and may not overwrite a frozen project.
- Plan A stops at `WAITING_HUMAN_REVIEW`. `agt-map review` and `agt-map freeze` are Plan B and must not be implemented here.
- Exit codes: `0` command completed with no pending human/prerequisite action; `2` valid result requires human action; `3` deterministic prerequisite/input block; `4` project contract/validation failure; `5` runtime/implementation failure. `agt-map status --json` always exits `0` for a valid project regardless of project state.

---

## File Structure Locked for Plan A

Create one new focused package:

```text
src/agt_map_pipeline/
├── CMakeLists.txt
├── package.xml
├── README.md
├── agt_map_pipeline/
│   ├── __init__.py
│   ├── hashing.py              # canonical JSON/file identity helpers
│   ├── project.py              # agt_map_project/v1 load/write/validate contract
│   ├── presets.py              # MapPreset + greenhouse v1 resolution
│   ├── layer_io.py             # layer serialization/registry only
│   ├── traversability_preview.py
│   ├── prepare.py              # deterministic orchestration/state updates
│   ├── status.py               # read-only effective project status
│   └── cli.py                  # parser and exit-code mapping
├── scripts/
│   └── agt_map_launcher.py
└── test/
    ├── test_project_contract.py
    ├── test_presets.py
    ├── test_layer_io.py
    ├── test_prepare.py
    ├── test_status_cli.py
    └── test_prepare_e2e.py
```

Modify only the following existing files unless a RED test proves another change is required:

```text
src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py
src/agt_offline_assets/agt_offline_assets/__init__.py
src/agt_offline_assets/test/test_navigation_map_derivation.py
.github/workflows/paper1-route-benchmark.yml
```

The only intended `agt_offline_assets` change is exposing its existing Nav2 map serialization as a public writer; no classification math changes.

---

### Task 1: Project Contract, Canonical Hashing, and State Vocabulary

**Files:**
- Create: `src/agt_map_pipeline/package.xml`
- Create: `src/agt_map_pipeline/CMakeLists.txt`
- Create: `src/agt_map_pipeline/agt_map_pipeline/__init__.py`
- Create: `src/agt_map_pipeline/agt_map_pipeline/hashing.py`
- Create: `src/agt_map_pipeline/agt_map_pipeline/project.py`
- Create: `src/agt_map_pipeline/test/test_project_contract.py`

**Interfaces:**
- Produces: `PROJECT_SCHEMA = "agt_map_project/v1"`
- Produces: `PROJECT_STATES = {"NEW", "PREPARING", "WAITING_HUMAN_REVIEW", "REVIEW_IN_PROGRESS", "READY_TO_FREEZE", "FROZEN", "BLOCKED", "FAILED"}`
- Produces: `canonical_sha256(value: object) -> str`
- Produces: `create_project(project_dir: Path, *, source: dict, preset: dict, declared_frame_id: str, site_id: str | None) -> dict`
- Produces: `load_project(project_dir: Path | str) -> dict`
- Produces: `write_project(project_dir: Path | str, document: Mapping[str, object]) -> Path`
- Produces: `validate_project_document(document: Mapping[str, object]) -> None`
- Produces: `register_stage(document: dict, name: str, *, status: str, inputs_sha256: str, outputs: list[dict], message: str) -> None`
- Produces: `register_layer(document: dict, layer_id: str, *, status: str, path: str, sha256: str, stage: str, parents: list[str]) -> None`

- [ ] **Step 1: Write RED project-contract tests**

Add tests that pin the minimum schema and reject file-existence-only status inference:

```python
from pathlib import Path
import pytest
import yaml

from agt_map_pipeline.project import (
    PROJECT_SCHEMA,
    create_project,
    load_project,
    register_layer,
    register_stage,
    write_project,
)


def test_create_project_records_unverified_frame_and_empty_review_requirements(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(
        root,
        source={
            "absolute_path": "/data/site.pcd",
            "sha256": "a" * 64,
            "size_bytes": 123,
            "summary": {"point_count": 10},
        },
        preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64},
        declared_frame_id="map",
        site_id="greenhouse_01",
    )
    assert doc["schema"] == PROJECT_SCHEMA
    assert doc["project_state"] == "NEW"
    assert doc["frame"]["declared_frame_id"] == "map"
    assert doc["frame"]["verification"] == "UNVERIFIED"
    assert doc["accepted_revision"] is None
    assert (root / "project.yaml").is_file()


def test_stage_and_layer_status_are_manifest_records_not_file_presence(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(
        root,
        source={"absolute_path": "/x.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}},
        preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64},
        declared_frame_id="map",
        site_id=None,
    )
    (root / "layers").mkdir()
    fake = root / "layers" / "fake.npy"
    fake.write_bytes(b"exists")
    write_project(root, doc)
    loaded = load_project(root)
    assert "fake" not in loaded["layers"]


def test_register_stage_rejects_unknown_status(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(
        root,
        source={"absolute_path": "/x.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}},
        preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64},
        declared_frame_id="map",
        site_id=None,
    )
    with pytest.raises(ValueError, match="status"):
        register_stage(doc, "pcd_profile", status="DONE", inputs_sha256="c" * 64, outputs=[], message="")
```

- [ ] **Step 2: Run the RED test**

```bash
cd ~/agt_navigation_v2_paper1
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_project_contract.py
```

Expected: FAIL during import because `agt_map_pipeline.project` does not exist.

- [ ] **Step 3: Implement the minimal project contract**

`hashing.py` must canonicalize JSON-compatible values using sorted keys and compact separators:

```python
import hashlib
import json


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()
```

`project.py` must write `project.yaml` atomically through a sibling temporary file, use project-relative artifact paths, and create these directories on initial project creation:

```text
source/
config/
layers/terrain/
layers/obstacle/
layers/navigation/
layers/structure/
layers/traversability/
evidence/
review/
revisions/
```

Initial manifest shape:

```yaml
schema: agt_map_project/v1
project_id: <uuid4>
site_id: greenhouse_01 | null
project_state: NEW
source: {...}
preset: {...}
frame:
  declared_frame_id: map
  verification: UNVERIFIED
stages: {}
layers: {}
human_review:
  required:
    - frame
    - navigation_overrides
    - site_boundary
    - row_aisle_candidates
    - semantic_features
accepted_revision: null
```

`register_layer()` must require `path` to be relative and reject `..` traversal.

- [ ] **Step 4: Run Task 1 tests GREEN**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_project_contract.py
```

Expected: PASS.

- [ ] **Step 5: Commit Task 1**

```bash
git add src/agt_map_pipeline
git commit -m "feat(map-pipeline): add project manifest contract"
```

---

### Task 2: Declarative `greenhouse` Preset Resolution

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/presets.py`
- Create: `src/agt_map_pipeline/test/test_presets.py`

**Interfaces:**
- Consumes: existing `GroundRelativeNavigationConfig`, `NavigationStructureConfig`, `CorridorRefinementConfig`, `AisleGraphConfig`, `TraversabilityConfig`
- Produces:

```python
@dataclass(frozen=True)
class MapPreset:
    name: str
    version: str
    navigation_config: GroundRelativeNavigationConfig
    structure_config: NavigationStructureConfig | None
    corridor_config: CorridorRefinementConfig | None
    aisle_graph_config: AisleGraphConfig | None
    traversability_config: TraversabilityConfig | None
    required_human_layers: tuple[str, ...]
```

- Produces: `resolve_preset(name: str, *, profile_path: Path | str | None = None) -> MapPreset`
- Produces: `write_resolved_preset(preset: MapPreset, output_path: Path | str, *, profile_reference: dict | None = None) -> dict`

- [ ] **Step 1: Write RED preset tests**

```python
from dataclasses import asdict
from pathlib import Path
import pytest
import yaml

from agt_map_pipeline.presets import resolve_preset, write_resolved_preset


def test_greenhouse_preset_is_declarative_and_requires_human_semantics(tmp_path: Path):
    preset = resolve_preset("greenhouse")
    assert preset.name == "greenhouse"
    assert preset.version == "1"
    assert preset.structure_config is not None
    assert preset.corridor_config is not None
    assert preset.aisle_graph_config is not None
    assert "site_boundary" in preset.required_human_layers
    assert "semantic_features" in preset.required_human_layers
    assert preset.corridor_config.enable_boundary_aisles is False


def test_resolved_preset_is_fully_materialized(tmp_path: Path):
    preset = resolve_preset("greenhouse")
    output = tmp_path / "resolved_preset.yaml"
    payload = write_resolved_preset(preset, output)
    stored = yaml.safe_load(output.read_text())
    assert stored == payload
    assert stored["navigation"] == asdict(preset.navigation_config)
    assert stored["structure"] == asdict(preset.structure_config)
    assert stored["corridor"] == asdict(preset.corridor_config)
    assert stored["aisle_graph"] == asdict(preset.aisle_graph_config)


def test_unknown_preset_fails_closed():
    with pytest.raises(ValueError, match="unsupported preset"):
        resolve_preset("magic")
```

Profile override behavior for Plan A is intentionally narrow: if `--profile` is used, YAML must have schema `agt_map_pipeline_profile/v1` and may override only keys under `navigation`, `structure`, `corridor`, and `aisle_graph`; unknown sections or unknown dataclass fields are rejected. It must not contain planner names, planner weights, or benchmark outcome fields.

- [ ] **Step 2: Run RED**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_presets.py
```

Expected: FAIL because `agt_map_pipeline.presets` does not exist.

- [ ] **Step 3: Implement built-in greenhouse v1 with current algorithm defaults**

Use current config constructors as the v1 source of values, immediately materialized to YAML before derivation:

```python
MapPreset(
    name="greenhouse",
    version="1",
    navigation_config=GroundRelativeNavigationConfig(),
    structure_config=NavigationStructureConfig(),
    corridor_config=CorridorRefinementConfig(enable_boundary_aisles=False),
    aisle_graph_config=AisleGraphConfig(),
    traversability_config=TraversabilityConfig(),
    required_human_layers=(
        "frame",
        "navigation_overrides",
        "site_boundary",
        "row_aisle_candidates",
        "semantic_features",
    ),
)
```

Profile overrides are applied by `dataclasses.replace()` after validating exact dataclass field names. The resolved document must include `schema: agt_map_resolved_preset/v1`, name, version, all config dictionaries, required human layers, optional profile source SHA, and a canonical document SHA.

- [ ] **Step 4: Run Task 2 tests GREEN**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_presets.py
```

Expected: PASS.

- [ ] **Step 5: Commit Task 2**

```bash
git add src/agt_map_pipeline/agt_map_pipeline/presets.py src/agt_map_pipeline/test/test_presets.py
git commit -m "feat(map-pipeline): add greenhouse preset resolution"
```

---

### Task 3: Public Navigation Writer + Deterministic Layer Serialization

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/test/test_navigation_map_derivation.py`
- Create: `src/agt_map_pipeline/agt_map_pipeline/layer_io.py`
- Create: `src/agt_map_pipeline/agt_map_pipeline/traversability_preview.py`
- Create: `src/agt_map_pipeline/test/test_layer_io.py`

**Interfaces:**
- Produces in `agt_offline_assets`: `write_navigation_map_files(result: NavigationMapResult, output_dir: Path | str) -> dict[str, object]`, a public wrapper around the existing `_write_nav2_map_files()` semantics.
- Produces: `write_prepare_layers(project_dir, navigation, structure, corridor, aisle_graph) -> dict[str, dict]`
- Produces: `derive_unbounded_traversability_preview(navigation, corridor, *, frame_id="map") -> dict[str, object]`
- Produces: `write_unbounded_traversability_preview(preview, output_dir) -> dict[str, dict]`

- [ ] **Step 1: RED test the public writer without changing map bytes**

Add to `test_navigation_map_derivation.py`:

```python
def test_public_navigation_map_writer_matches_derivation_map_bytes(tmp_path):
    result = _example_navigation_result()
    direct = tmp_path / "direct"
    written = write_navigation_map_files(result, direct)
    assert (direct / "navigation_map.pgm").is_file()
    assert (direct / "navigation_map.yaml").is_file()
    assert written["pgm_sha256"] == sha256_file(direct / "navigation_map.pgm")
```

The implementation must rename or wrap the existing private `_write_nav2_map_files`; do not change PGM pixel semantics, Nav2 thresholds, or orientation.

- [ ] **Step 2: RED test project-layer serialization and pre-boundary safety**

```python
import numpy as np

from agt_map_pipeline.traversability_preview import derive_unbounded_traversability_preview


def test_unbounded_preview_never_recovers_unknown(navigation_result, corridor_result):
    preview = derive_unbounded_traversability_preview(navigation_result, corridor_result)
    occ = navigation_result.occupancy
    assert preview["status"] == "CANDIDATE_UNBOUNDED"
    assert np.array_equal(preview["observed_free"], occ == 254)
    assert np.array_equal(preview["sensor_obstacle"], occ == 0)
    assert np.array_equal(preview["unknown"], occ == 127)
    assert np.array_equal(
        preview["aisle_geometric_envelope"],
        corridor_result.aisle_geometric_envelope,
    )
    assert "inferred_traversable" not in preview
```

Also assert the following artifact set is written using deterministic `.npy` and YAML/vector writers:

```text
layers/terrain/ground_height.npy
layers/terrain/ground_valid.npy
layers/terrain/ground_confidence.npy
layers/terrain/slope_deg.npy
layers/terrain/step_m.npy
layers/obstacle/obstacle_count.npy
layers/obstacle/obstacle_mask.npy
layers/navigation/navigation_map.pgm
layers/navigation/navigation_map.yaml
layers/navigation/occupancy.npy
layers/structure/row_support.npy
layers/structure/row_regularized_obstacle.npy
layers/structure/row_centerline.npy
layers/structure/row_structural_band.npy
layers/structure/aisle_geometric_envelope.npy
layers/structure/aisle_candidate.npy
layers/structure/aisle_centerline.npy
layers/structure/row_model.yaml
layers/structure/aisle_graph.yaml
layers/traversability/observed_free.npy
layers/traversability/sensor_obstacle.npy
layers/traversability/unknown.npy
layers/traversability/aisle_geometric_envelope.npy
layers/traversability/preview.yaml
```

- [ ] **Step 3: Run RED tests**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_derivation.py \
  src/agt_map_pipeline/test/test_layer_io.py
```

Expected: FAIL on the missing public writer and missing pipeline layer modules.

- [ ] **Step 4: Implement serialization without duplicating algorithms**

`derive_unbounded_traversability_preview()` is exactly:

```python
occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
return {
    "schema": "agt_unbounded_traversability_preview/v1",
    "frame_id": frame_id,
    "status": "CANDIDATE_UNBOUNDED",
    "observed_free": occupancy == FREE,
    "sensor_obstacle": occupancy == OCCUPIED,
    "unknown": occupancy == UNKNOWN,
    "aisle_geometric_envelope": np.asarray(
        corridor.aisle_geometric_envelope, dtype=bool
    ).copy(),
    "message": "Site Boundary not human-approved; no bounded UNKNOWN recovery performed.",
}
```

No Site Boundary object may be synthesized in this function, and `derive_traversability_evidence()` must not be called.

For `row_model.yaml`, serialize only deterministic fields already present in `NavigationStructureResult.row_model`: direction, angle, centers, half-width, support fractions. Use the existing `write_agricultural_aisle_graph()` for `aisle_graph.yaml`.

Every writer returns relative path + SHA256 metadata for later registration; callers, not writers, assign READY/CANDIDATE state.

- [ ] **Step 5: Run Task 3 tests GREEN**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_derivation.py \
  src/agt_map_pipeline/test/test_layer_io.py
```

Expected: PASS.

- [ ] **Step 6: Commit Task 3**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_navigation_map_derivation.py \
  src/agt_map_pipeline/agt_map_pipeline/layer_io.py \
  src/agt_map_pipeline/agt_map_pipeline/traversability_preview.py \
  src/agt_map_pipeline/test/test_layer_io.py
git commit -m "feat(map-pipeline): serialize automatic candidate layers"
```

---

### Task 4: Deterministic `prepare()` Orchestration, Partial-Failure Preservation, and Resume

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/prepare.py`
- Create: `src/agt_map_pipeline/test/test_prepare.py`

**Interfaces:**
- Consumes Task 1 project contract, Task 2 preset, Task 3 writers.
- Produces:

```python
@dataclass(frozen=True)
class PrepareResult:
    project_dir: Path
    project_state: str
    completed_stages: tuple[str, ...]
    blocked_stages: tuple[str, ...]
    human_required: bool
```

- Produces:

```python
def prepare_project(
    pcd_path: Path | str,
    *,
    preset_name: str,
    output_dir: Path | str,
    declared_frame_id: str = "map",
    site_id: str | None = None,
    profile_path: Path | str | None = None,
    resume: bool = False,
) -> PrepareResult:
    ...
```

- [ ] **Step 1: Write RED orchestration tests with a small synthetic PCD**

Tests must use an actual tiny ASCII or binary PCD fixture created under `tmp_path`, not monkeypatch the whole pipeline. The fixture must contain enough ground/row obstacle structure for existing offline algorithms to run deterministically.

Pin stage order:

```python
EXPECTED_STAGE_ORDER = (
    "source_profile",
    "navigation",
    "structure",
    "corridor",
    "aisle_graph",
    "traversability_preview",
)
```

Core test:

```python
def test_prepare_finishes_at_waiting_human_review(tmp_path, greenhouse_pcd):
    result = prepare_project(
        greenhouse_pcd,
        preset_name="greenhouse",
        output_dir=tmp_path / "project",
        declared_frame_id="map",
        site_id="synthetic_greenhouse",
    )
    project = load_project(result.project_dir)
    assert result.project_state == "WAITING_HUMAN_REVIEW"
    assert project["project_state"] == "WAITING_HUMAN_REVIEW"
    assert tuple(project["stages"].keys()) == EXPECTED_STAGE_ORDER
    assert project["layers"]["navigation.raw_occupancy"]["status"] == "READY"
    assert project["layers"]["structure.row_support"]["status"] == "CANDIDATE"
    assert project["layers"]["structure.aisle_graph"]["status"] == "CANDIDATE"
    assert project["layers"]["traversability.preview"]["status"] == "CANDIDATE_UNBOUNDED"
    assert project["human_review"]["required"]
```

Add failure-preservation test by supplying a fixture with valid ground/navigation but no usable rows. Required result:

```text
navigation.raw_occupancy = READY
structure.row_support = CANDIDATE or BLOCKED_NO_ROWS with diagnostic artifacts retained
structure.aisle_graph = BLOCKED_NO_ROWS
traversability.preview = BLOCKED_NO_ROWS
project_state = WAITING_HUMAN_REVIEW
```

This is not a project-level runtime failure because a human can still inspect the generic map.

Add source mismatch resume test:

```python
def test_resume_rejects_changed_source_sha(tmp_path, greenhouse_pcd):
    project = tmp_path / "project"
    prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project)
    greenhouse_pcd.write_bytes(greenhouse_pcd.read_bytes() + b"changed")
    with pytest.raises(SourceIdentityError, match="SHA"):
        prepare_project(
            greenhouse_pcd,
            preset_name="greenhouse",
            output_dir=project,
            resume=True,
        )
```

- [ ] **Step 2: Run RED**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_prepare.py
```

Expected: FAIL because `prepare_project` does not exist.

- [ ] **Step 3: Implement stage orchestration with explicit state writes**

Exact stage algorithm:

```text
1. resolve/check source path, size, SHA; read PCD; summarize profile
2. resolve greenhouse preset and write config/resolved_preset.yaml
3. set project_state PREPARING
4. derive_ground_relative_navigation_map(cloud, preset.navigation_config)
5. write/register terrain + raw navigation/obstacle layers
6. derive_navigation_structure(navigation, preset.structure_config)
7. write/register ground-confidence + row evidence
8. if no usable row centers: record downstream BLOCKED_NO_ROWS and stop automatic structure path
9. else derive_corridor_refinement(navigation, structure, preset.corridor_config)
10. write/register row/aisle candidate layers
11. if no accepted rows/aisles: record aisle graph/preview BLOCKED_NO_ROWS but retain all earlier outputs
12. else derive_agricultural_aisle_graph(..., frame_id=declared_frame_id)
13. write/register aisle graph as CANDIDATE
14. derive/write unbounded traversability preview only
15. set project_state WAITING_HUMAN_REVIEW
```

For every stage, compute `inputs_sha256` from canonical parent identities + resolved preset SHA. Write stage/layer manifest records only after output files are successfully written and hashed.

Any implementation exception not classified as deterministic source/no-row block sets the project state to `FAILED`, records the failing stage with the error type/message, and re-raises as `PrepareRuntimeError`; do not delete earlier outputs.

`--resume` semantics for Plan A:

- project must already validate as `agt_map_project/v1`;
- project state must not be `FROZEN`;
- source absolute path + SHA and resolved preset identity must match;
- no `review/review_state.yaml` may exist yet (Plan A must not overwrite future human work);
- automatic stages are recomputed deterministically in place from the same source/preset, replacing only generated `layers/` and `evidence/prepare_manifest.json`; project ID is preserved;
- if identity differs, fail closed rather than creating a mixed-lineage project.

- [ ] **Step 4: Run Task 4 tests GREEN**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_prepare.py
```

Expected: PASS.

- [ ] **Step 5: Commit Task 4**

```bash
git add src/agt_map_pipeline/agt_map_pipeline/prepare.py src/agt_map_pipeline/test/test_prepare.py
git commit -m "feat(map-pipeline): orchestrate deterministic greenhouse prepare"
```

---

### Task 5: Read-Only Status Contract and Dual-Mode `agt-map` CLI

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/status.py`
- Create: `src/agt_map_pipeline/agt_map_pipeline/cli.py`
- Create: `src/agt_map_pipeline/scripts/agt_map_launcher.py`
- Create: `src/agt_map_pipeline/test/test_status_cli.py`
- Modify: `src/agt_map_pipeline/CMakeLists.txt`

**Interfaces:**
- Produces: `build_project_status(project_dir: Path | str) -> dict[str, object]`
- Produces: `main(argv: list[str] | None = None) -> int`
- Public shell commands for Plan A:

```text
agt-map prepare <pcd> --preset greenhouse --output <project> [--frame-id map] [--site-id ID] [--profile YAML] [--resume]
agt-map status <project> [--json]
```

- `review` and `freeze` must not be parser choices in Plan A. README states they arrive in Plan B.

- [ ] **Step 1: Write RED status tests**

Pin the JSON contract:

```python
def test_status_json_is_agent_oriented(prepared_project):
    status = build_project_status(prepared_project)
    assert status["schema"] == "agt_map_project_status/v1"
    assert status["project_state"] == "WAITING_HUMAN_REVIEW"
    assert status["preset"] == "greenhouse"
    assert status["layers"]["navigation.raw_occupancy"] == "READY"
    assert status["layers"]["traversability.preview"] == "CANDIDATE_UNBOUNDED"
    assert status["layers"]["site_boundary"] == "HUMAN_REQUIRED"
    assert status["layers"]["semantic.headland"] == "HUMAN_REQUIRED"
    assert status["next_action"] == {
        "human_required": True,
        "command": f"agt-map review {prepared_project.resolve()}",
        "reason": "site boundary and agricultural semantics require human review",
    }
```

Status must re-hash the source and all registered artifacts. A source mismatch returns an effective status:

```json
{
  "project_state": "BLOCKED",
  "block_code": "BLOCKED_SOURCE_HASH_MISMATCH",
  "next_action": {
    "human_required": true,
    "command": null,
    "reason": "source PCD identity changed; choose the intended source before resuming"
  }
}
```

It must not mutate `project.yaml` while doing this read-only check.

- [ ] **Step 2: Write RED CLI exit-code tests**

Call `main([...])` directly with `capsys`:

```python
def test_status_json_exits_zero_for_waiting_human(prepared_project, capsys):
    code = main(["status", str(prepared_project), "--json"])
    payload = json.loads(capsys.readouterr().out)
    assert code == 0
    assert payload["project_state"] == "WAITING_HUMAN_REVIEW"


def test_prepare_returns_two_when_project_is_ready_for_human_review(tmp_path, greenhouse_pcd):
    code = main([
        "prepare", str(greenhouse_pcd),
        "--preset", "greenhouse",
        "--output", str(tmp_path / "project"),
    ])
    assert code == 2
```

Error mapping:

```text
invalid/unsupported PCD or source missing -> 3
existing project without --resume -> 4
source/preset identity mismatch on resume -> 4
unexpected implementation/runtime exception -> 5
status --json valid project in BLOCKED state -> 0
```

- [ ] **Step 3: Run RED**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_status_cli.py
```

Expected: FAIL because status/CLI modules do not exist.

- [ ] **Step 4: Implement concise human status and stable JSON status**

Human output should be compact, for example:

```text
Project : /abs/map_project
State   : WAITING_HUMAN_REVIEW
Preset  : greenhouse/v1
Ready   : navigation.raw_occupancy
Candidate: structure.row_support, structure.aisle_graph, traversability.preview
Human   : frame, navigation_overrides, site_boundary, row_aisle_candidates, semantic_features
Next    : agt-map review /abs/map_project
```

Do not print planner suggestions.

- [ ] **Step 5: Install one launcher in both shell PATH and ROS2 package lib**

Because GitHub API writes do not preserve executable mode reliably, follow the repository's existing CMake launcher-copy pattern. Copy `scripts/agt_map_launcher.py` into `${CMAKE_CURRENT_BINARY_DIR}/cli` with explicit execute permissions, then install the same built copy twice:

```cmake
install(PROGRAMS "${AGT_MAP_PIPELINE_CLI_DIR}/agt_map_launcher.py"
  DESTINATION bin
  RENAME agt-map
)
install(PROGRAMS "${AGT_MAP_PIPELINE_CLI_DIR}/agt_map_launcher.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME agt-map
)
```

Launcher body remains tiny:

```python
#!/usr/bin/env python3
from agt_map_pipeline.cli import main
raise SystemExit(main())
```

This must enable both:

```bash
agt-map status <project> --json
ros2 run agt_map_pipeline agt-map status <project> --json
```

- [ ] **Step 6: Run Task 5 tests GREEN**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test/test_status_cli.py
```

Expected: PASS.

- [ ] **Step 7: Commit Task 5**

```bash
git add \
  src/agt_map_pipeline/agt_map_pipeline/status.py \
  src/agt_map_pipeline/agt_map_pipeline/cli.py \
  src/agt_map_pipeline/scripts/agt_map_launcher.py \
  src/agt_map_pipeline/test/test_status_cli.py \
  src/agt_map_pipeline/CMakeLists.txt
git commit -m "feat(map-pipeline): add agent-readable prepare status CLI"
```

---

### Task 6: End-to-End Acceptance, CI, Documentation, and Real-PCD Stop Gate

**Files:**
- Create: `src/agt_map_pipeline/test/test_prepare_e2e.py`
- Create: `src/agt_map_pipeline/README.md`
- Modify: `src/agt_map_pipeline/package.xml`
- Modify: `src/agt_map_pipeline/CMakeLists.txt`
- Modify: `.github/workflows/paper1-route-benchmark.yml`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/__init__.py`

**Interfaces:**
- Final Plan A public API: `agt-map prepare`, `agt-map status`.
- Final package dependencies: `agt_offline_assets`, `python3-numpy`, `python3-yaml`, plus transitive algorithm dependencies already declared by `agt_offline_assets`.

- [ ] **Step 1: Add a true end-to-end deterministic prepare test**

Create the same deterministic synthetic PCD twice at two paths, run prepare into two project roots, and compare all generated layer SHA records except project UUID, absolute source path, and source-reference file. The actual layer artifacts and resolved preset SHA must be identical.

Required assertions:

```python
assert status_a["project_state"] == "WAITING_HUMAN_REVIEW"
assert status_b["project_state"] == "WAITING_HUMAN_REVIEW"
assert project_a["preset"]["sha256"] == project_b["preset"]["sha256"]
for layer_id in sorted(project_a["layers"]):
    assert project_a["layers"][layer_id]["sha256"] == project_b["layers"][layer_id]["sha256"]
```

If the raw source SHA intentionally differs because PCD serialization differs, the fixture must instead use byte-identical source content. Do not weaken the layer comparison.

- [ ] **Step 2: Register all tests with ament and declare package dependencies**

`CMakeLists.txt` must register exactly these Plan A test files with `ament_add_pytest_test` and install the Python package. `package.xml` must declare `agt_offline_assets` as an `exec_depend` and `ament_cmake_pytest`/`python3-pytest` as test dependencies.

- [ ] **Step 3: Extend Paper I CI without removing existing gates**

Add `src/agt_map_pipeline/**` to workflow path triggers. Add one pure-Python command:

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python -m pytest -q src/agt_map_pipeline/test
```

Also include `src/agt_map_pipeline/agt_map_pipeline` and `src/agt_map_pipeline/scripts` in compileall. Do not remove the existing offline-assets, workbench, route-benchmark, or cusp tests.

- [ ] **Step 4: Write README for humans and agents**

README must start with the minimal workflow:

```bash
agt-map prepare /path/to/site.pcd --preset greenhouse --output ./map_project
agt-map status ./map_project --json
```

Document:

- what each automatic layer means;
- `CANDIDATE` vs `READY` vs `CANDIDATE_UNBOUNDED`;
- why Site Boundary/headland/no-go are not auto-accepted;
- source PCD is referenced, not copied;
- Plan A stops at `WAITING_HUMAN_REVIEW`;
- the agent contract and exit codes;
- `review/freeze` are intentionally deferred to Plan B.

Include this exact agent instruction snippet:

```text
Run `agt-map prepare <pcd> --preset greenhouse --output <project>`.
Then run `agt-map status <project> --json`.
If `next_action.human_required` is true, stop and report the project path,
project state, blocking reason, and next_action. Do not run planners and do not
invent Site Boundary or semantic features.
```

- [ ] **Step 5: Run fresh pure-Python verification**

```bash
cd ~/agt_navigation_v2_paper1

PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_navigation_map_derivation.py

PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_pipeline/test

python3 -m compileall -q \
  src/agt_map_pipeline/agt_map_pipeline \
  src/agt_map_pipeline/scripts \
  src/agt_offline_assets/agt_offline_assets
```

Acceptance: all commands exit `0`.

- [ ] **Step 6: Run fresh ROS2 build and isolated ament tests**

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to agt_map_pipeline
source install/setup.bash

rm -rf /tmp/agt_map_pipeline_test_results
colcon test \
  --packages-select agt_offline_assets agt_map_pipeline \
  --test-result-base /tmp/agt_map_pipeline_test_results \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base /tmp/agt_map_pipeline_test_results \
  --verbose
```

Acceptance: zero failures/errors.

- [ ] **Step 7: Verify both CLI invocation paths**

Use a small synthetic PCD, not the multi-GB real file, for executable smoke tests:

```bash
agt-map prepare /tmp/agt_map_pipeline_fixture.pcd \
  --preset greenhouse \
  --output /tmp/agt_map_pipeline_direct
DIRECT_PREPARE_RC=$?

test "$DIRECT_PREPARE_RC" -eq 2
agt-map status /tmp/agt_map_pipeline_direct --json

ros2 run agt_map_pipeline agt-map status \
  /tmp/agt_map_pipeline_direct --json
```

Both status commands must emit schema `agt_map_project_status/v1` and exit `0`.

- [ ] **Step 8: Run the real greenhouse PCD Plan A gate and stop**

Only after all synthetic/build tests pass, use the existing real PCD candidate selected by the researcher. Example form:

```bash
agt-map prepare \
  /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd \
  --preset greenhouse \
  --site-id greenhouse_01 \
  --frame-id map \
  --output "$PWD/runtime/maps/greenhouse_01/map_project"
PREPARE_RC=$?

test "$PREPARE_RC" -eq 2
agt-map status "$PWD/runtime/maps/greenhouse_01/map_project" --json \
  | tee "$PWD/runtime/maps/greenhouse_01/map_project/evidence/status_after_prepare.json"
```

This gate does **not** claim the source frame is verified merely because `--frame-id map` was supplied. Expected status remains `WAITING_HUMAN_REVIEW`, with frame/site-boundary/semantic review required. No planner is run.

If the real source file is not the intended formal PCD or its provenance is still disputed, do not improvise. Run the automatic gate only on an explicitly approved source path, otherwise stop with `BLOCKED_SOURCE_SELECTION` in the implementation report.

- [ ] **Step 9: Final Plan A verification report**

Report exactly:

```text
1. git HEAD
2. commits per task
3. pure Python test counts
4. colcon build result
5. isolated ament test counts
6. direct `agt-map` smoke result
7. `ros2 run agt_map_pipeline agt-map` smoke result
8. synthetic project state + layer statuses
9. real PCD path/SHA or BLOCKED_SOURCE_SELECTION
10. real project path if created
11. real `status --json` summary
12. confirmation that no Site Boundary/semantics were auto-accepted
13. confirmation that no planner ran
14. stop reason: `WAITING_HUMAN_REVIEW` or `BLOCKED_SOURCE_SELECTION`
```

- [ ] **Step 10: Commit Task 6**

```bash
git add \
  src/agt_map_pipeline \
  src/agt_offline_assets \
  .github/workflows/paper1-route-benchmark.yml
git commit -m "test(map-pipeline): close prepare-status plan A acceptance"
```

---

## Plan A Self-Review

### Spec coverage

- Generic orchestration package: Tasks 1–5.
- Project manifest/state/layer hashes: Tasks 1 and 4.
- Declarative greenhouse preset: Task 2.
- Automatic PCD -> navigation -> structure -> corridor -> aisle graph: Task 4.
- Required terrain/navigation/structure artifacts: Task 3.
- Pre-boundary `CANDIDATE_UNBOUNDED` rule with no UNKNOWN recovery: Task 3.
- PCD reference rather than copy: Task 4.
- `--resume`: Task 4.
- Agent-readable `status --json` and exit codes: Task 5.
- Direct `agt-map` plus `ros2 run`: Tasks 5–6.
- Failure preservation/no-row degradation: Task 4.
- Reproducibility and deterministic artifact hashes: Tasks 1, 3, 4, 6.
- Final real-PCD gate ending before human review/planners: Task 6.
- GUI review, semantic authoring, Site Boundary acceptance, canonical freeze, QA revision: intentionally excluded and reserved for Plan B exactly as the approved spec requires.

### Type consistency

Later tasks use only interfaces introduced earlier:

```text
canonical_sha256
create_project/load_project/write_project
register_stage/register_layer
MapPreset/resolve_preset/write_resolved_preset
write_navigation_map_files
write_prepare_layers
derive_unbounded_traversability_preview
prepare_project/PrepareResult
build_project_status
cli.main
```

No Plan B API is required to make Plan A useful.
