# V25-12G-A3 Site-Boundary Reverse-Admission Amendment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct A3 connector evidence so an endpoint-safe forward Dubins Site-Boundary failure may enter the existing R6A/R6B reverse-aware fallback chain, while endpoint footprint conflicts remain hard rejections.

**Architecture:** Split the existing broad forward-audit Site-Boundary status into endpoint conflict versus forward-path conflict. Preserve the existing Site Boundary as a strict full-footprint invariant and keep every R6B search budget unchanged. Carry the new evidence through R6A, A3 `TransitionValidation`, and the real-data acceptance harness without changing the A3 motion-graph schema.

**Tech Stack:** Python 3.10, ROS 2 Humble, pytest/ament_cmake_pytest, NumPy, Shapely-backed `SiteBoundary`, existing R5/R6A/R6B connector modules, deterministic YAML.

## Global Constraints

- Branch remains `feat/v25-12g-maximum-feasible-coverage`; use the original repo, no extra worktree.
- Never touch/reset/clean the unrelated local `tools/rosbag_sensor_trimmer` modification or `tools/map_tools/render_pcd_top_views.py` untracked file.
- Strict TDD: production behavior changes only after the amendment RED tests are observed failing.
- Operator gates are batched: one amendment RED gate, then one final GREEN gate; no per-function user test interruptions unless blocked.
- Site Boundary remains `VEHICLE_PERMITTED_INNER_BOUNDARY`; touching/crossing is a conflict.
- `preview_footprint_padding_m = 0.05` remains frozen.
- R6B defaults remain unchanged, especially `max_expansions = 30000`, `max_path_length_m = 18.0`, `max_cusps = 2`, `primitive_length_m = 0.30`, `state_xy_resolution_m = 0.15`, and `state_yaw_resolution_deg = 15.0`.
- A1/A2, Turn Zones, Navigation Grid, Site Boundary geometry, vehicle profile, and service-motion semantics are immutable in this amendment.
- No A4 optimizer, R7, analytic Reeds-Shepp planner, budget tuning, map editing, footprint tuning, or START_POSE reachability work.
- A3 schema remains `agt_vehicle_feasible_motion_graph/v1` with top status `MOTION_EVIDENCE_ONLY`.
- Real-data rerun must not hard-code an executable-transition count.

---

## File Structure

**Modify:**
- `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py` — endpoint-vs-path Site-Boundary evidence and deterministic serialization.
- `src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py` — backward-compatible loading of new forward-audit endpoint evidence.
- `src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py` — R6A decisions for endpoint hard conflict and endpoint-safe forward-path boundary conflict.
- `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py` — A3 transition orchestration using amended audit/R6A evidence.
- `tools/v25_12g_a3_acceptance.py` — amended boundary-admission funnel/invariant diagnostics.

**Test:**
- `src/agt_offline_assets/test/test_forward_connector_candidate_audit.py`
- `src/agt_offline_assets/test/test_reverse_fallback_admission.py`
- `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- `tests/test_v25_12g_a3_contract.py`

No new production module is needed.

---

### Task 1: Freeze the Amendment RED Contract

**Files:**
- Modify: `src/agt_offline_assets/test/test_forward_connector_candidate_audit.py`
- Modify: `src/agt_offline_assets/test/test_reverse_fallback_admission.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Modify: `tests/test_v25_12g_a3_contract.py`

**Interfaces:**
- Consumes existing `derive_forward_connector_candidate_audit(...)`, `derive_reverse_fallback_admission(...)`, and `validate_transition_candidates(...)` signatures unchanged.
- Freezes new forward-audit statuses:
  - `CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT`
  - `LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT`
- Freezes new R6A decision:
  - `REJECT_HARD_CONSTRAINT`
- Freezes new harness invariant:
  - `endpoint_boundary_conflict_reverse_admission_count == 0`

- [ ] **Step 1: Replace the old broad forward-audit boundary test with endpoint/path split RED tests**

Add tests equivalent to:

```python
def test_candidate_audit_endpoint_boundary_conflict_is_connector_level_hard_evidence():
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(FREE), _vehicle(), site_boundary=_boundary()
    )
    result = plan.connectors[0]
    assert result.status == "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"
    assert result.start_endpoint_site_boundary_free is False
    assert result.goal_endpoint_site_boundary_free is False


def test_candidate_audit_endpoint_safe_forward_path_boundary_conflict_is_distinct(monkeypatch):
    import agt_offline_assets.forward_connector_candidate_audit as audit_api

    original = audit_api._candidate_inside_site_boundary

    def endpoint_safe_path_blocked(samples, footprint, boundary):
        samples = tuple(samples)
        if len(samples) == 1:
            return True
        return False

    monkeypatch.setattr(
        audit_api,
        "_candidate_inside_site_boundary",
        endpoint_safe_path_blocked,
    )
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(FREE), _vehicle(), site_boundary=_boundary(-2.0, 2.0)
    )
    result = plan.connectors[0]
    assert result.status == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
    assert result.start_endpoint_site_boundary_free is True
    assert result.goal_endpoint_site_boundary_free is True
    assert result.local_candidate_count >= 1
    assert all(not item.site_boundary_free for item in result.candidates if item.local_headland_candidate)
```

Also extend serialization/round-trip assertions so the result-level booleans survive write/load:

```python
assert item["start_endpoint_site_boundary_free"] is True
assert item["goal_endpoint_site_boundary_free"] is True
```

- [ ] **Step 2: Add R6A RED tests for the two new evidence classes**

Extend `_audit()` or add focused fixtures and assert:

```python
def test_admission_rejects_endpoint_boundary_conflict_without_reverse():
    audit = ForwardConnectorCandidateAuditPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="deadbeef",
        connectors=(
            _result("connector_005", "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"),
        ),
    )
    plan = derive_reverse_fallback_admission(audit)
    item = plan.items[0]
    assert item.decision == "REJECT_HARD_CONSTRAINT"
    assert plan.eligible_connector_ids == ()


def test_admission_allows_endpoint_safe_forward_path_boundary_conflict():
    audit = ForwardConnectorCandidateAuditPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="deadbeef",
        connectors=(
            _result("connector_005", "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"),
        ),
    )
    plan = derive_reverse_fallback_admission(audit)
    assert plan.items[0].decision == "ELIGIBLE_REVERSE_FALLBACK"
    assert plan.eligible_connector_ids == ("connector_005",)
```

Update the frozen source-policy assertion to the exact amended value:

```python
assert payload["source"]["admission_policy"] == (
    "AUTO_LOCAL_OCCUPANCY_OR_ENDPOINT_SAFE_FORWARD_PATH_BOUNDARY"
)
```

- [ ] **Step 3: Add A3 transition RED tests proving endpoint conflict never reaches R6B and path conflict may reach R6B**

Replace the old `LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT` parameter case with explicit tests:

```python
def test_transition_endpoint_boundary_conflict_is_hard_rejected_without_r6b(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *a, **k: _forward_gate_plan("NO_FORWARD_PREVIEW_FREE_CANDIDATE"),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *a, **k: _audit_plan("CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *a, **k: _admission("REJECT_HARD_CONSTRAINT"),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *a, **k: pytest.fail("endpoint conflict must not reach R6B"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(), _zones(), _navigation(), _vehicle(),
        VehicleFeasibleMotionGraphConfig(), site_boundary=_boundary(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION
    assert result.reverse_admission_evidence["decision"] == "REJECT_HARD_CONSTRAINT"


def test_transition_forward_path_boundary_conflict_can_be_executable_via_r6b(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *a, **k: _forward_gate_plan("NO_FORWARD_PREVIEW_FREE_CANDIDATE"),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *a, **k: _audit_plan("LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"),
    )
    monkeypatch.setattr(api, "derive_reverse_fallback_admission", lambda *a, **k: _admission())
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *a, **k: _reverse_plan("REVERSE_PRIMITIVE_PREVIEW_FREE"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(), _zones(), _navigation(), _vehicle(),
        VehicleFeasibleMotionGraphConfig(), site_boundary=_boundary(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.forward_evidence["audit_status"] == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
    assert result.reverse_admission_evidence["decision"] == "ELIGIBLE_REVERSE_FALLBACK"
```

Add the bounded-no-solution sibling and preserve the existing occupancy/map/mixed/forward-free tests.

- [ ] **Step 4: Add harness RED tests for the corrected safety invariant and funnel counters**

Extend fake transition fixtures so `_summary()` can be tested directly. Freeze these keys:

```python
assert summary["endpoint_boundary_conflict_count"] == 1
assert summary["forward_path_boundary_conflict_count"] == 2
assert summary["forward_path_boundary_reverse_admitted_count"] == 2
assert summary["forward_path_boundary_reverse_executable_count"] == 1
assert summary["forward_path_boundary_reverse_bounded_no_solution_count"] == 1
assert summary["endpoint_boundary_conflict_reverse_admission_count"] == 0
```

Add a negative case where an endpoint-conflict record carries `ELIGIBLE_REVERSE_FALLBACK`; `_summary()` must raise `ValueError`.

- [ ] **Step 5: Commit the complete RED test batch**

```bash
git add \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a3_contract.py
git commit -m "test(v25-12g): freeze A3 boundary admission amendment"
```

- [ ] **Step 6: Operator RED gate — run once and verify failures are amendment-specific**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m pytest -q \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a3_contract.py
```

Expected RED categories:
- old broad status returned instead of endpoint/path split;
- result-level endpoint evidence fields absent;
- R6A does not know `REJECT_HARD_CONSTRAINT` or path-boundary admission;
- A3 still rejects broad boundary evidence before R6B;
- harness still reports the old broad bypass diagnostic.

Do not implement production changes unless this RED gate is observed.

---

### Task 2: Split Forward-Audit Endpoint and Path Boundary Evidence

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py`
- Test already frozen in Task 1.

**Interfaces:**
- `derive_forward_connector_candidate_audit(...)` signature remains unchanged.
- Extend `ForwardConnectorCandidateAuditResult` with backward-compatible fields:

```python
start_endpoint_site_boundary_free: bool = True
goal_endpoint_site_boundary_free: bool = True
```

- Existing `ForwardCandidateAuditItem.site_boundary_free` remains path-candidate evidence.
- Forward-audit schema string remains `agt_forward_connector_candidate_audit/v1`; loader treats missing endpoint booleans in historical v1 assets as `True`.

- [ ] **Step 1: Add a single-pose full-footprint boundary helper**

Inside `forward_connector_candidate_audit.py`:

```python
def _endpoint_inside_site_boundary(pose, local_footprint, site_boundary):
    if site_boundary is None:
        return True
    sample = ForwardConnectorSample(
        x=float(pose[0]), y=float(pose[1]), z=float(pose[2]), yaw=float(pose[3])
    )
    return _candidate_inside_site_boundary((sample,), local_footprint, site_boundary)
```

Import `ForwardConnectorSample` from `.forward_connector` without changing any planner behavior.

- [ ] **Step 2: Classify endpoints before forward path families**

For each request, after Turn Zone metadata validation and before `_dubins_candidates(...)`:

```python
start_boundary_free = _endpoint_inside_site_boundary(
    request.start_pose, local_footprint, site_boundary
)
goal_boundary_free = _endpoint_inside_site_boundary(
    request.goal_pose, local_footprint, site_boundary
)
if site_boundary is not None and not (start_boundary_free and goal_boundary_free):
    results.append(
        ForwardConnectorCandidateAuditResult(
            ...,
            status="CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
            candidate_count=0,
            local_candidate_count=0,
            local_known_occupied_count=0,
            local_map_insufficient_count=0,
            candidates=(),
            reason="connector start or goal footprint touches or crosses the hard Site Boundary",
            start_endpoint_site_boundary_free=start_boundary_free,
            goal_endpoint_site_boundary_free=goal_boundary_free,
        )
    )
    continue
```

Every other result records the two booleans as `True` when Site Boundary is absent or both endpoints passed.

- [ ] **Step 3: Rename only the path-family boundary classification**

Keep candidate enumeration and `local_safe` filtering intact. Change:

```python
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT
```

to:

```python
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
```

with reason:

```text
all locally relevant forward candidates touch or cross the hard Site Boundary while connector endpoints remain legal
```

Do not allow boundary-conflicting candidates into occupancy/map evidence counts.

- [ ] **Step 4: Serialize and backward-compatibly load endpoint evidence**

Add to every serialized connector result:

```python
"start_endpoint_site_boundary_free": result.start_endpoint_site_boundary_free,
"goal_endpoint_site_boundary_free": result.goal_endpoint_site_boundary_free,
```

In `route_diagnostic_io.load_forward_connector_candidate_audit(...)` pass:

```python
start_endpoint_site_boundary_free=bool(
    item.get("start_endpoint_site_boundary_free", True)
),
goal_endpoint_site_boundary_free=bool(
    item.get("goal_endpoint_site_boundary_free", True)
),
```

No other route-diagnostic schema behavior changes.

- [ ] **Step 5: Commit forward-audit implementation**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py \
  src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py
git commit -m "feat(v25-12g): split A3 boundary endpoint and path evidence"
```

---

### Task 3: Amend R6A Without Weakening the Hard Boundary

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py`
- Test already frozen in Task 1.

**Interfaces:**
- `derive_reverse_fallback_admission(...)` signature remains unchanged.
- New automatic mappings:

```text
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT -> ELIGIBLE_REVERSE_FALLBACK
CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT -> REJECT_HARD_CONSTRAINT
```

- Existing occupancy/map/mixed/forward-free decisions remain unchanged.

- [ ] **Step 1: Add explicit amended decision branches**

Immediately after occupancy-blocked handling:

```python
elif status == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT":
    decision = "ELIGIBLE_REVERSE_FALLBACK"
    reason = (
        "connector endpoints are Site-Boundary-safe but all locally relevant "
        "forward Dubins candidates violate the hard boundary; R6B may test an "
        "alternative motion family under the same hard boundary"
    )
elif status == "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT":
    decision = "REJECT_HARD_CONSTRAINT"
    reason = (
        "connector start or goal full vehicle footprint violates the hard Site Boundary"
    )
```

Do not add endpoint-conflict IDs to `eligible_connector_ids`.

- [ ] **Step 2: Update deterministic source policy metadata**

Freeze:

```python
"admission_policy": "AUTO_LOCAL_OCCUPANCY_OR_ENDPOINT_SAFE_FORWARD_PATH_BOUNDARY",
"endpoint_boundary_conflict_never_reverse_admitted": True,
"map_insufficient_never_auto_admitted": True,
```

Keep `mixed_evidence_requires_operator_approval=True`.

- [ ] **Step 3: Keep serialization deterministic and backward readable**

No schema bump is needed for R6A; `decision` is already a string field. Ensure the new source keys serialize in deterministic insertion order.

- [ ] **Step 4: Commit R6A amendment**

```bash
git add src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py
git commit -m "feat(v25-12g): admit endpoint-safe boundary fallback"
```

---

### Task 4: Route Amended Boundary Evidence Through A3 Transition Validation

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`
- Test already frozen in Task 1.

**Interfaces:**
- `validate_transition_candidates(...)` signature remains unchanged.
- Preserve one `TransitionValidation` per A2 `ConnectorCandidate`.
- `forward_evidence` for audited transitions must include:

```python
{
    "audit_status": item.status,
    "reason": item.reason,
    "start_endpoint_site_boundary_free": item.start_endpoint_site_boundary_free,
    "goal_endpoint_site_boundary_free": item.goal_endpoint_site_boundary_free,
}
```

- `reverse_admission_evidence` retains `decision` and `reason`.

- [ ] **Step 1: Stop direct A3 rejection of forward-path boundary evidence**

Remove the old direct mapping for `LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT`.

The amended endpoint status maps to hard rejection only after R6A supplies `REJECT_HARD_CONSTRAINT`.

- [ ] **Step 2: Derive one R6A plan for the audited unresolved connector set**

After the candidate audit, call:

```python
admission = derive_reverse_fallback_admission(
    audit,
    ReverseFallbackAdmissionConfig(operator_approved_mixed_connector_ids=()),
    source={"acceptance_stage": "v25_12g_a3"},
)
admission_by_id = {item.connector_id: item for item in admission.items}
```

Fail closed if any audited connector is missing an admission item.

- [ ] **Step 3: Normalize non-R6B decisions from R6A**

Use exact decision mapping:

```text
REJECT_HARD_CONSTRAINT -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
HOLD_MAP_REVIEW        -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
HOLD_MIXED_EVIDENCE    -> UNRESOLVED / MIXED_EVIDENCE_REQUIRES_REVIEW
HOLD_POLICY_REVIEW     -> UNRESOLVED / POLICY_REVIEW_REQUIRED
KEEP_FORWARD           -> fail closed as gate/audit disagreement in this unresolved path
```

Preserve the original explicit `NO_FORWARD_DUBINS_CANDIDATE` handling if required by the frozen A3 contract; do not silently broaden this amendment into a different planner policy.

- [ ] **Step 4: Send every R6A-eligible connector to R6B regardless of whether admission came from occupancy or path-boundary evidence**

Build `reverse_requests` strictly from `admission.eligible_connector_ids`. Invoke the existing `derive_reverse_primitive_connector_plan(...)` with:

```python
ReversePrimitiveConnectorConfig(
    preview_footprint_padding_m=config.preview_footprint_padding_m,
)
```

Do not pass any altered search budget.

- [ ] **Step 5: Preserve R6B result normalization**

Keep:

```text
REVERSE_PRIMITIVE_PREVIEW_FREE / FORWARD_PRIMITIVE_PREVIEW_FREE
    -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE

SITE_BOUNDARY_CONFLICT
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION

NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
    -> REJECTED / BOUNDED_SEARCH_NO_SOLUTION

R6B_START_FOOTPRINT_NOT_FREE + OCCUPIED evidence
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION

R6B_START_FOOTPRINT_NOT_FREE + UNKNOWN/out-of-grid evidence
    -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
```

The accepted R6B path remains independently Site-Boundary checked by R6B itself.

- [ ] **Step 6: Commit transition orchestration**

```bash
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
git commit -m "feat(v25-12g): route boundary path conflicts through R6B"
```

---

### Task 5: Amend the A3 Acceptance Harness and Final Verification

**Files:**
- Modify: `tools/v25_12g_a3_acceptance.py`
- Test already frozen in Task 1.

**Interfaces:**
- Report schema remains `agt_v25_12g_a3_acceptance_report/v1`.
- Validation scope remains `A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY`.
- Replace the old broad bypass invariant with actual endpoint safety.

- [ ] **Step 1: Replace the obsolete bypass diagnostic**

In `_summary(...)`, compute:

```python
endpoint_conflict = "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"
path_conflict = "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"

endpoint_boundary_conflict_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == endpoint_conflict
    for t in transition_records
)
forward_path_boundary_conflict_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == path_conflict
    for t in transition_records
)
endpoint_boundary_conflict_reverse_admission_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == endpoint_conflict
    and t.get("reverse_admission_evidence", {}).get("decision")
        == "ELIGIBLE_REVERSE_FALLBACK"
    for t in transition_records
)
```

Also compute:

```python
forward_path_boundary_reverse_admitted_count
forward_path_boundary_reverse_executable_count
forward_path_boundary_reverse_bounded_no_solution_count
```

using `audit_status`, R6A decision, final status, backend, and proof scope.

- [ ] **Step 2: Fail closed only on the actual forbidden endpoint admission**

```python
if endpoint_boundary_conflict_reverse_admission_count:
    raise ValueError(
        "A3 reverse fallback admitted a connector whose endpoint footprint violates Site Boundary"
    )
```

Remove the old logic that treated every forward-path boundary conflict followed by R6B as a bypass.

- [ ] **Step 3: Preserve all existing structural invariants**

Continue enforcing:

```text
all_a2_service_states_preserved == True
all_a2_connector_candidates_preserved == True
dead_end_non_retrace_count == 0
forbidden_semantic_key_count == 0
```

Keep read-only default, sibling-only write, overwrite refusal, and all input-file immutability behavior unchanged.

- [ ] **Step 4: Commit the harness amendment**

```bash
git add tools/v25_12g_a3_acceptance.py
git commit -m "feat(v25-12g): report amended A3 boundary funnel"
```

- [ ] **Step 5: Final GREEN gate — focused and package tests**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash

colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash

python3 -m pytest -q \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a3_contract.py

colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

Do not claim GREEN until the operator provides fresh output with zero failures.

---

### Task 6: Rerun the Frozen Greenhouse A3 Checkpoint

**Files:**
- Runtime inputs only; do not commit generated runtime assets.
- Later documentation after evidence:
  - Create or update: `docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md`
  - Create or update: `docs/v2.5/V25_12G_A3_CURRENT_STATE.md`

**Interfaces:**
- Run directory: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`
- Vehicle profile: `profiles/platforms/mk_mini.yaml`
- Read-only harness first.

- [ ] **Step 1: Freeze hashes of protected upstream assets**

Hash at least:

```text
vehicle_feasible_service_graph.yaml
vehicle_feasible_segments.yaml
turn_zones.yaml
navigation_map.yaml
navigation_map.pgm
site_boundary.yaml
derivation.yaml
aisle_graph.yaml
```

- [ ] **Step 2: Run read-only A3 acceptance with resource timing**

```bash
/usr/bin/time -v \
  python3 tools/v25_12g_a3_acceptance.py \
    --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
    --vehicle-profile "$PWD/profiles/platforms/mk_mini.yaml" \
    --pretty \
    >/tmp/v25_12g_a3_amended_readonly.json
```

Required structural facts remain:

```text
service_validation_count == 32
transition_validation_count == 40
all_a2_service_states_preserved == True
all_a2_connector_candidates_preserved == True
dead_end_non_retrace_count == 0
endpoint_boundary_conflict_reverse_admission_count == 0
forbidden_semantic_key_count == 0
```

Do not assert any expected executable-transition count.

- [ ] **Step 3: Report amended LOW_U/HIGH_U funnel**

For each side, report:

```text
endpoint hard conflicts
forward-path boundary conflicts
R6A admitted connectors
R6B executable connectors
R6B bounded no-solution connectors
```

Also retain service coverage facts and the four exact-retrace dead-end records.

- [ ] **Step 4: Only after read-only evidence is internally consistent, write the A3 sibling once**

Use `--write-motion-graph` only if `vehicle_feasible_motion_graph.yaml` does not already exist. Never overwrite automatically.

- [ ] **Step 5: Prove upstream hashes unchanged, strict-load the A3 graph, and verify byte-stable rewrite**

Use `load_vehicle_feasible_motion_graph(...)` followed by a temporary `write_vehicle_feasible_motion_graph(...)`; bytes must match exactly.

- [ ] **Step 6: Freeze documentation only from observed evidence**

Strongest allowed classification is bounded by the actual rerun. Do not claim START reachability, route readiness, or optimality. If local transitions become executable, describe them as `LOCAL_MOTION_EXECUTABLE` only.

- [ ] **Step 7: Commit only docs, never runtime assets**

```bash
git add \
  docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md \
  docs/v2.5/V25_12G_A3_CURRENT_STATE.md
git commit -m "docs(v25-12g): freeze amended A3 real-data evidence"
```

A4 may resume only after this amended A3 real-data checkpoint is frozen.
