# AGT Map PCD Layer Pipeline Design

Date: 2026-08-19
Branch: `feat/paper1-agri-route-benchmark`

## 1. Goal

Build a reusable map-preparation tool that lets another developer or Codex provide a PCD file and obtain a deterministic set of map/evidence layers, then stop at a human review/edit window before any final semantic or navigation-map commitment is made.

The first production preset is `greenhouse`, while the orchestration core remains scene-agnostic enough to add future presets such as orchard, tea garden, or warehouse without changing the generic project/CLI contracts.

The intended user workflow is:

```text
PCD
 ↓
agt-map prepare <pcd> --preset greenhouse
 ↓
automatic candidate layers
 ↓
agt-map status <project>
 ↓
WAITING_HUMAN_REVIEW
 ↓
agt-map review <project>
 ↓
human occupancy / boundary / agricultural-structure / semantic edits
 ↓
agt-map freeze <project>
 ↓
immutable accepted assets + provenance + QA
```

The tool must also expose enough machine-readable state for Codex to operate it without reading internal implementation files.

## 2. Non-goals

This subsystem does not:

- run A*, Theta*, Hybrid A*, State Lattice, Fields2Cover, or the proposed Paper I planner;
- change Paper I benchmark outcomes based on planner behavior;
- infer a final site boundary without human confirmation;
- infer final headland, no-go, coverage, or task semantics without human confirmation;
- replace `agt_offline_assets` algorithms with duplicate implementations;
- silently convert an unverified input frame into the canonical `map` frame;
- make `CANDIDATE` structural layers equivalent to accepted semantic truth.

## 3. Chosen architecture

### 3.1 New orchestration package: `agt_map_pipeline`

Create a new ROS2/Python package whose responsibility is project orchestration only. It consumes the stable algorithms already implemented in `agt_offline_assets` and launches/extents the existing `agt_map_workbench` for human review.

`agt_map_pipeline` owns:

- project manifest schema;
- preset registry;
- deterministic stage orchestration;
- layer registry and artifact hashes;
- state machine / `next_action` contract;
- CLI commands `prepare`, `status`, `review`, and `freeze`;
- generic map-only QA and freeze gates.

It must not reimplement ground estimation, row detection, corridor extraction, traversability, or aisle graph algorithms.

### 3.2 Existing `agt_offline_assets` remains algorithm authority

Reuse the current primitives:

- `read_pcd()`;
- `derive_ground_relative_navigation_map()`;
- `derive_navigation_structure()`;
- `derive_corridor_refinement()`;
- `derive_agricultural_aisle_graph()`;
- `derive_traversability_evidence()` after a human-approved Site Boundary exists;
- `apply_navigation_overrides()`;
- `write_navigation_map_freeze_bundle()`;
- `SiteBoundary` validation/writing.

The generic tool must import these APIs rather than copy their math.

### 3.3 Existing `agt_map_workbench` remains GUI authority

The Workbench already has PCD display, map authoring, Site Boundary authoring, agricultural review planes, and Paper I override metadata. A new project-aware window will load an `agt_map_project/v1` project, display its generated layers, persist human review state back into the project, and export formal revisions using the canonical offline-assets writers.

The GUI is not the only API. Every project must remain inspectable with `agt-map status --json` so an agent can know exactly when human interaction is required.

## 4. CLI contract

After building and sourcing the workspace, the preferred shell API is:

```bash
agt-map prepare input.pcd --preset greenhouse --output map_project
agt-map status map_project
agt-map status map_project --json
agt-map review map_project
agt-map freeze map_project
```

The same executable must also be usable through ROS2:

```bash
ros2 run agt_map_pipeline agt-map ...
```

### 4.1 `prepare`

`prepare` creates a new project directory. It fails if the destination already contains a project unless `--resume` is explicitly supplied.

Minimum inputs:

```text
PCD path
preset name
project output directory
```

Optional inputs:

```text
--frame-id map
--site-id <name>
--profile <preset/profile yaml>
--resume
```

The source PCD is referenced, not copied by default. The project records:

- absolute source path;
- file size;
- SHA256;
- PCD schema/profile summary;
- declared frame id;
- `frame_verification: UNVERIFIED` unless a user explicitly verifies it later.

No 2+ GB PCD duplication occurs by default.

### 4.2 `status`

Human output is concise. JSON output is stable and agent-oriented.

Example:

```json
{
  "schema": "agt_map_project_status/v1",
  "project_state": "WAITING_HUMAN_REVIEW",
  "preset": "greenhouse",
  "source_sha256": "...",
  "layers": {
    "navigation.raw_occupancy": "READY",
    "structure.row_support": "CANDIDATE",
    "structure.aisle_graph": "CANDIDATE",
    "traversability.preview": "CANDIDATE_UNBOUNDED",
    "site_boundary": "HUMAN_REQUIRED",
    "semantic.headland": "HUMAN_REQUIRED"
  },
  "next_action": {
    "human_required": true,
    "command": "agt-map review /abs/project"
  }
}
```

Status must never tell an agent to run planners.

### 4.3 `review`

`review` launches the project-aware Workbench. The Workbench loads the source PCD and every generated candidate layer from the project rather than asking the user to manually rediscover paths.

### 4.4 `freeze`

`freeze` is fail-closed. It verifies source hashes, required human review decisions, canonical override replay, and required accepted layers before producing immutable accepted assets.

`freeze` must not set human acceptance booleans automatically.

## 5. Project directory contract

A prepared project uses:

```text
map_project/
├── project.yaml
├── source/
│   └── source_reference.yaml
├── config/
│   └── resolved_preset.yaml
├── layers/
│   ├── terrain/
│   │   ├── ground_height.npy
│   │   ├── ground_valid.npy
│   │   ├── ground_confidence.npy
│   │   ├── slope_deg.npy
│   │   └── step_m.npy
│   ├── obstacle/
│   │   ├── obstacle_count.npy
│   │   └── obstacle_mask.npy
│   ├── navigation/
│   │   ├── navigation_map.pgm
│   │   ├── navigation_map.yaml
│   │   └── occupancy.npy
│   ├── structure/
│   │   ├── row_support.npy
│   │   ├── row_regularized_obstacle.npy
│   │   ├── row_centerline.npy
│   │   ├── row_structural_band.npy
│   │   ├── aisle_candidate.npy
│   │   ├── aisle_centerline.npy
│   │   └── aisle_graph.yaml
│   └── traversability/
│       ├── observed_free.npy
│       ├── sensor_obstacle.npy
│       ├── unknown.npy
│       ├── aisle_geometric_envelope.npy
│       └── preview.yaml
├── evidence/
│   ├── prepare_manifest.json
│   └── layer_manifest.json
├── review/
│   ├── review_state.yaml
│   ├── navigation_overrides.yaml
│   ├── site_boundary.yaml
│   └── semantic_features.geojson
└── revisions/
```

`review/` files may be absent before human interaction.

`revisions/` remains empty until an explicit freeze/revision export.

## 6. Project manifest

`project.yaml` schema is `agt_map_project/v1`.

It contains:

- immutable project UUID/id;
- optional `site_id`;
- preset name/version;
- source PCD reference and SHA;
- declared frame;
- frame verification state;
- fully resolved configuration hash;
- stage records;
- layer records;
- human-review requirements;
- current project state;
- accepted revision reference once frozen.

Each stage record has:

```yaml
status: READY | CANDIDATE | HUMAN_REQUIRED | BLOCKED | FAILED | FROZEN
inputs_sha256: ...
outputs:
  - path: ...
    sha256: ...
message: ...
```

No stage status may be inferred merely from a file existing; hashes and manifest identity are authoritative.

## 7. Preset architecture

A preset is a declarative policy object, not a second algorithm implementation.

Generic interface:

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

First preset: `greenhouse`.

Its resolved values are written into `config/resolved_preset.yaml` so subsequent results do not depend on future code defaults.

The first version does not auto-tune parameters from planner outcomes.

## 8. Automatic `greenhouse` prepare stages

The automatic chain is:

```text
source PCD
  ↓
PCD validation/profile
  ↓
Ground-relative NavigationMapResult
  ↓
terrain layers
  ↓
raw occupancy/obstacle layers
  ↓
NavigationStructureResult
  ↓
row evidence / ground confidence
  ↓
CorridorRefinementResult
  ↓
row centerlines / aisle candidates
  ↓
AgriculturalAisleGraph candidate
  ↓
pre-boundary traversability preview
  ↓
WAITING_HUMAN_REVIEW
```

### 8.1 Automatically generated layers

The first version generates at least:

Generic terrain/navigation:

- ground height;
- ground-valid mask;
- ground confidence;
- robust/local slope;
- step evidence;
- obstacle count;
- obstacle mask;
- raw trinary occupancy.

Greenhouse structure:

- row support;
- row-regularized obstacle evidence;
- detected row model/direction;
- row centerline candidate;
- row structural band;
- aisle geometric envelope;
- aisle candidate;
- aisle centerline candidate;
- agricultural aisle graph candidate.

Pre-boundary traversability preview:

- observed free;
- sensor obstacle;
- unknown;
- aisle geometric envelope.

### 8.2 Traversability safety rule

The existing `derive_traversability_evidence()` requires a READY Site Boundary and must retain that contract.

Therefore automatic `prepare` does **not** call it before human boundary review. Instead the project writes an explicitly weaker `traversability.preview` with:

```text
status = CANDIDATE_UNBOUNDED
```

This preview is visualization/evidence only and performs no bounded UNKNOWN recovery.

After the human accepts `site_boundary.yaml`, the Workbench or a later stage may call the existing formal traversability derivation and produce the real bounded traversability candidate.

## 9. Human review boundary

After successful automatic prepare, project state is exactly:

```text
WAITING_HUMAN_REVIEW
```

The GUI must make the following distinction visually and structurally:

- GENERATED / measured evidence;
- CANDIDATE / algorithm-derived structure;
- HUMAN ACCEPTED;
- HUMAN REJECTED;
- OVERRIDE / manually edited.

Required human responsibilities in V1:

1. verify the project coordinate frame is suitable as canonical `map`;
2. inspect raw occupancy and add auditable `FORCE_FREE` / `FORCE_OCCUPIED` polygons where evidence supports corrections;
3. author/accept Site Boundary;
4. accept/reject detected row and aisle candidates;
5. author or confirm headland and permanent/no-go semantic polygons;
6. provide stable IDs for accepted semantic features;
7. explicitly mark the reviewed map/structure state ready for freeze.

The tool may pre-fill candidate labels but must not silently convert them to accepted semantics.

## 10. Workbench project mode

Add a project-aware Workbench window rather than a separate GUI application.

Project mode must:

- open `project.yaml` directly;
- resolve the source PCD automatically;
- load all raster/mask/vector candidate layers;
- expose layer visibility toggles grouped by Terrain / Navigation / Structure / Traversability / Semantics;
- preserve existing 2D editing interactions;
- reuse canonical Paper I override metadata validation for formal FREE/OCCUPIED edits;
- persist review records under `review/`;
- show project state and blocking requirements;
- never launch a planner.

Row/aisle V1 editing is bounded to candidate acceptance/rejection plus manual semantic geometry. Arbitrary spline-style centerline editing is outside the first implementation unless required by a failing real-map review.

## 11. Semantic feature contract

Human semantic edits are stored as `review/semantic_features.geojson`.

V1 feature types:

- `row`;
- `aisle`;
- `headland`;
- `permanent_obstacle`;
- `no_go`;
- `entrance_exit`.

Every feature requires:

- stable ID;
- `feature_type`;
- frame id `map`;
- geometry;
- provenance: `candidate_accepted` or `human_authored`;
- optional source candidate id;
- human note/reason when authored or materially changed.

Coverage/task semantics remain a downstream asset and are not fabricated during `prepare`.

## 12. Freeze semantics

A project may freeze only when all required gates are satisfied.

Minimum V1 gates:

- source PCD still exists and SHA matches;
- frame is explicitly human-verified;
- navigation overrides validate;
- Site Boundary validates;
- row/aisle candidate review has no unresolved required item;
- semantic feature IDs are unique;
- canonical accepted map is obtained only by replaying recorded overrides over generated occupancy;
- map replay QA has zero unexplained changed cells.

Freeze output:

```text
revisions/revision_XXXX/
├── generated/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── accepted/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── layers/
│   └── accepted layer snapshots
├── semantic/
│   ├── semantic_map.geojson
│   └── site_boundary.yaml
├── evidence/
│   ├── map_qa_report.json
│   ├── overrides.geojson
│   ├── map_curation_qa.svg
│   ├── map_curation_qa.pdf
│   └── map_curation_qa.png
└── revision_manifest.json
```

The generic freeze does not create Paper I `coverage.yaml` automatically. Paper I may consume a frozen revision and add its task-specific coverage asset afterward.

## 13. State machine

Project states:

```text
NEW
PREPARING
WAITING_HUMAN_REVIEW
REVIEW_IN_PROGRESS
READY_TO_FREEZE
FROZEN
BLOCKED
FAILED
```

Normal path:

```text
NEW -> PREPARING -> WAITING_HUMAN_REVIEW
                        ↓
                 REVIEW_IN_PROGRESS
                        ↓
                  READY_TO_FREEZE
                        ↓
                      FROZEN
```

`BLOCKED` means a deterministic prerequisite is missing or invalid but the project is recoverable. `FAILED` means a prepare/review/freeze stage encountered an implementation/runtime failure requiring diagnosis.

## 14. Codex contract

Codex should only need the public commands and `status --json`.

Recommended agent loop:

```text
agt-map prepare ...
 ↓
agt-map status --json
 ↓
if next_action.human_required:
    stop and request review
else:
    execute next_action.command
```

Machine-readable `next_action` must contain:

```json
{
  "human_required": true,
  "command": "agt-map review /abs/project",
  "reason": "site boundary and agricultural semantics require human review"
}
```

Exit codes:

- `0`: command completed successfully;
- `2`: valid project but human action is required;
- `3`: deterministic input/prerequisite block;
- `4`: project contract/validation failure;
- `5`: runtime/implementation failure.

`status --json` itself exits 0 for a valid project regardless of its state; agents inspect the JSON state.

## 15. Failure handling

The pipeline is fail-closed on provenance.

Examples:

- source SHA changed -> `BLOCKED_SOURCE_HASH_MISMATCH`;
- unsupported/invalid PCD -> `BLOCKED_SOURCE_INVALID`;
- row detection yields no usable rows -> project still keeps generic terrain/navigation layers and records `structure.* = BLOCKED_NO_ROWS`; human can inspect the map rather than losing all earlier outputs;
- Site Boundary missing -> official traversability remains `HUMAN_REQUIRED`, while pre-boundary preview remains visible;
- stale/tampered accepted map -> freeze rejects it and requires canonical replay;
- duplicate semantic IDs -> freeze rejects it;
- unverified frame -> freeze rejects it.

A later-stage failure must not delete valid earlier layer artifacts.

## 16. Reproducibility

Every generated layer record binds:

- source PCD SHA;
- resolved preset SHA;
- stage name/version;
- parent layer hashes;
- artifact hash;
- status.

`prepare` never chooses parameters after seeing planner results.

`freeze` creates immutable revision directories and refuses overwrite.

## 17. Testing strategy

Strict TDD is required.

Pure Python tests cover:

- project schema and state transitions;
- preset resolution;
- deterministic stage ordering;
- source hash validation;
- layer registry and hashes;
- small synthetic PCD end-to-end prepare;
- no-boundary traversability preview semantics;
- status JSON / exit-code contracts;
- replay-clean freeze gate;
- fail-closed source/frame/semantic validation.

Workbench tests cover:

- project loading;
- candidate layer discovery;
- review persistence;
- Site Boundary persistence;
- row/aisle accept/reject state;
- formal override persistence;
- no planner invocation.

ROS2 acceptance covers:

- package build;
- direct `agt-map` executable after sourcing;
- `ros2 run agt_map_pipeline agt-map`;
- launching `agt-map review` on Ubuntu 22.04 / ROS2 Humble.

A final real-PCD gate uses a real greenhouse PCD but stops at `WAITING_HUMAN_REVIEW` before any Paper I planner execution.

## 18. Implementation decomposition

This architectural feature is implemented in two sequential plans, each producing independently usable software.

### Plan A — Core Project + Prepare/Status

Produces:

```text
agt-map prepare
agt-map status
```

and the automatic generic/greenhouse candidate layer project.

It must stop in `WAITING_HUMAN_REVIEW` and is useful even before GUI integration is finished.

### Plan B — Project Review + Freeze

Adds:

```text
agt-map review
agt-map freeze
```

plus Workbench project mode, review persistence, canonical revision freeze, and QA evidence.

Paper I site snapshot / scenario matrix remains downstream and is not part of either plan.

## 19. Acceptance criteria

The architecture is accepted when an engineer who has not read the internal modules can do:

```bash
agt-map prepare /path/to/greenhouse.pcd --preset greenhouse --output /tmp/greenhouse_map
agt-map status /tmp/greenhouse_map --json
```

and obtain a valid project with terrain/navigation/greenhouse structure candidate layers and `WAITING_HUMAN_REVIEW`.

After project-review implementation, the same engineer can run:

```bash
agt-map review /tmp/greenhouse_map
agt-map freeze /tmp/greenhouse_map
```

and receive an immutable replay-clean revision only after required human review is recorded.

At no point does the tool require planner output to construct, edit, or freeze the map.
