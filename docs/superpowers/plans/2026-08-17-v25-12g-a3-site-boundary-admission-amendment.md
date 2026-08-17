# V25-12G-A3 Site-Boundary Reverse-Admission Amendment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct A3 connector evidence so an endpoint-safe forward Dubins Site-Boundary failure may enter the existing R6A/R6B reverse-aware fallback chain, while endpoint footprint conflicts remain hard rejections.

**Architecture:** Split the current broad forward-audit Site-Boundary status into endpoint conflict versus forward-path conflict. Preserve Site Boundary as a strict full-footprint invariant, keep all R6B budgets unchanged, and carry the amended evidence through R6A, A3 `TransitionValidation`, and the acceptance harness without changing the A3 motion-graph schema.

**Tech Stack:** Python 3.10, ROS 2 Humble, pytest/ament_cmake_pytest, NumPy, Shapely-backed `SiteBoundary`, existing R5/R6A/R6B modules, deterministic YAML.

## Global Constraints

- Branch: `feat/v25-12g-maximum-feasible-coverage`; use the original repo, no extra worktree.
- Never touch/reset/clean unrelated local `tools/rosbag_sensor_trimmer` or untracked `tools/map_tools/render_pcd_top_views.py`.
- Strict TDD: no production behavior change before the amendment RED batch is observed failing.
- Operator gates are batched: one RED gate, one final GREEN gate; no tiny user test interruptions unless blocked.
- Site Boundary remains `VEHICLE_PERMITTED_INNER_BOUNDARY`; touching/crossing is a conflict.
- `preview_footprint_padding_m = 0.05` remains frozen.
- R6B defaults remain unchanged, especially `primitive_length_m=0.30`, `state_xy_resolution_m=0.15`, `state_yaw_resolution_deg=15.0`, `max_cusps=2`, `max_expansions=30000`, `max_path_length_m=18.0`.
- A1/A2, Turn Zones, Navigation Grid, Site Boundary geometry, vehicle profile, and service-motion semantics are immutable in this amendment.
- No A4, R7, analytic Reeds-Shepp, budget tuning, map/boundary editing, footprint tuning, or START_POSE reachability work.
- A3 schema remains `agt_vehicle_feasible_motion_graph/v1`, status `MOTION_EVIDENCE_ONLY`.
- Real-data rerun must not hard-code an executable-transition count.

---

## File Structure

**Modify production:**
- `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py`
- `src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py`
- `src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py`
- `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`
- `tools/v25_12g_a3_acceptance.py`

**Modify tests:**
- `src/agt_offline_assets/test/test_forward_connector_candidate_audit.py`
- `src/agt_offline_assets/test/test_reverse_fallback_admission.py`
- `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- `tests/test_v25_12g_a3_contract.py`

No new production module is needed.

---

### Task 1: Freeze the Complete Amendment RED Contract

**Files:** all four test files above.

**Interfaces frozen by tests:**

```text
CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
REJECT_HARD_CONSTRAINT
endpoint_boundary_conflict_reverse_admission_count == 0
```

- [ ] **Step 1: Replace the old broad forward-audit boundary test with endpoint/path split tests**

Add:

```python
def test_candidate_audit_endpoint_boundary_conflict_is_connector_level_hard_evidence():
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(FREE), _vehicle(), site_boundary=_boundary()
    )
    result = plan.connectors[0]
    assert result.status == "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"
    assert result.start_endpoint_site_boundary_free is False
    assert result.goal_endpoint_site_boundary_free is False
    assert result.candidate_count == 0


def test_candidate_audit_endpoint_safe_forward_path_boundary_conflict_is_distinct(monkeypatch):
    import agt_offline_assets.forward_connector_candidate_audit as audit_api

    def endpoint_safe_path_blocked(samples, footprint, boundary):
        samples = tuple(samples)
        return len(samples) == 1

    monkeypatch.setattr(
        audit_api,
        "_candidate_inside_site_boundary",
        endpoint_safe_path_blocked,
    )
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(FREE), _vehicle(),
        site_boundary=_boundary(-2.0, 2.0),
    )
    result = plan.connectors[0]
    assert result.status == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
    assert result.start_endpoint_site_boundary_free is True
    assert result.goal_endpoint_site_boundary_free is True
    assert result.local_candidate_count >= 1
    assert all(
        not item.site_boundary_free
        for item in result.candidates
        if item.local_headland_candidate
    )
```

Extend serialization assertions so both result-level and candidate-level boundary evidence survive YAML:

```python
payload = forward_connector_candidate_audit_to_dict(plan)
connector = payload["connectors"][0]
assert connector["start_endpoint_site_boundary_free"] is True
assert connector["goal_endpoint_site_boundary_free"] is True
assert "site_boundary_free" in connector["candidates"][0]
```

Extend the existing write/load round-trip test to assert the loaded `ForwardCandidateAuditItem.site_boundary_free` and endpoint booleans equal the original values.

- [ ] **Step 2: Add R6A RED tests for the two new evidence classes**

Add focused audit fixtures and assert:

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
    assert plan.items[0].decision == "REJECT_HARD_CONSTRAINT"
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

Update the source policy assertion to exactly:

```python
assert payload["source"]["admission_policy"] == (
    "AUTO_LOCAL_OCCUPANCY_OR_ENDPOINT_SAFE_FORWARD_PATH_BOUNDARY"
)
assert payload["source"]["endpoint_boundary_conflict_never_reverse_admitted"] is True
```

Keep occupancy/map/mixed/forward-free regression expectations unchanged.

- [ ] **Step 3: Update A3 transition test helper evidence**

Change `_audit_plan(...)` to provide endpoint booleans so all existing monkeypatched audit results match the amended result contract:

```python
def _audit_plan(status, *, start_boundary_free=True, goal_boundary_free=True):
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        reason=status,
        start_endpoint_site_boundary_free=start_boundary_free,
        goal_endpoint_site_boundary_free=goal_boundary_free,
    )
    return SimpleNamespace(connectors=(result,))
```

- [ ] **Step 4: Add A3 transition RED tests**

Endpoint hard conflict:

```python
def test_transition_endpoint_boundary_conflict_is_hard_rejected_without_r6b(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api, "derive_forward_connector_navigation_gate",
        lambda *a, **k: _forward_gate_plan("NO_FORWARD_PREVIEW_FREE_CANDIDATE"),
    )
    monkeypatch.setattr(
        api, "derive_forward_connector_candidate_audit",
        lambda *a, **k: _audit_plan(
            "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
            start_boundary_free=False,
            goal_boundary_free=True,
        ),
    )
    monkeypatch.setattr(
        api, "derive_reverse_fallback_admission",
        lambda *a, **k: _admission("REJECT_HARD_CONSTRAINT"),
    )
    monkeypatch.setattr(
        api, "derive_reverse_primitive_connector_plan",
        lambda *a, **k: pytest.fail("endpoint conflict must not reach R6B"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(), _zones(), _navigation(), _vehicle(),
        VehicleFeasibleMotionGraphConfig(), site_boundary=_boundary(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION
    assert result.reverse_admission_evidence["decision"] == "REJECT_HARD_CONSTRAINT"
```

Endpoint-safe path conflict + R6B success:

```python
def test_transition_forward_path_boundary_conflict_can_be_executable_via_r6b(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api, "derive_forward_connector_navigation_gate",
        lambda *a, **k: _forward_gate_plan("NO_FORWARD_PREVIEW_FREE_CANDIDATE"),
    )
    monkeypatch.setattr(
        api, "derive_forward_connector_candidate_audit",
        lambda *a, **k: _audit_plan("LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"),
    )
    monkeypatch.setattr(api, "derive_reverse_fallback_admission", lambda *a, **k: _admission())
    monkeypatch.setattr(
        api, "derive_reverse_primitive_connector_plan",
        lambda *a, **k: _reverse_plan("REVERSE_PRIMITIVE_PREVIEW_FREE"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(), _zones(), _navigation(), _vehicle(),
        VehicleFeasibleMotionGraphConfig(), site_boundary=_boundary(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.forward_evidence["audit_status"] == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
    assert result.forward_evidence["start_endpoint_site_boundary_free"] is True
    assert result.forward_evidence["goal_endpoint_site_boundary_free"] is True
    assert result.reverse_admission_evidence["decision"] == "ELIGIBLE_REVERSE_FALLBACK"
```

Add the bounded-no-solution sibling:

```python
assert result.status == REJECTED
assert result.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
assert result.backend_status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
```

Preserve existing occupancy/map/mixed/forward-free tests. `NO_FORWARD_DUBINS_CANDIDATE` remains a direct A3 `REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION`; `TURN_ZONE_METADATA_INVALID` remains `ValueError`.

- [ ] **Step 5: Add acceptance-harness RED tests**

Freeze summary keys:

```python
assert summary["endpoint_boundary_conflict_count"] == 1
assert summary["forward_path_boundary_conflict_count"] == 2
assert summary["forward_path_boundary_reverse_admitted_count"] == 2
assert summary["forward_path_boundary_reverse_executable_count"] == 1
assert summary["forward_path_boundary_reverse_bounded_no_solution_count"] == 1
assert summary["endpoint_boundary_conflict_reverse_admission_count"] == 0
```

Remove the old contract assertion for `site_boundary_reverse_bypass_count` and replace it with `endpoint_boundary_conflict_reverse_admission_count`.

Add a negative `_summary()` case where endpoint-conflict evidence carries `ELIGIBLE_REVERSE_FALLBACK`; assert `ValueError`.

- [ ] **Step 6: Commit all amendment tests before production changes**

```bash
git add \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a3_contract.py
git commit -m "test(v25-12g): freeze A3 boundary admission amendment"
```

- [ ] **Step 7: Operator RED gate**

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
- old broad boundary status instead of endpoint/path split;
- endpoint evidence fields absent;
- candidate `site_boundary_free` not preserved by YAML round-trip;
- R6A lacks new decision/admission mappings;
- A3 rejects path-boundary evidence before R6B;
- harness still uses the old broad bypass diagnostic.

Do not modify production code until this RED is observed.

---

### Task 2: Split Forward-Audit Endpoint and Path Evidence

**Files:**
- `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py`
- `src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py`

**Public signature:** `derive_forward_connector_candidate_audit(...)` unchanged.

**Result contract extension:** append after `reason`:

```python
start_endpoint_site_boundary_free: bool = True
goal_endpoint_site_boundary_free: bool = True
```

`ForwardCandidateAuditItem.site_boundary_free` remains the per-path evidence field.

- [ ] **Step 1: Add single-pose full-footprint endpoint checking**

Import `ForwardConnectorSample` and add:

```python
def _endpoint_inside_site_boundary(pose, local_footprint, site_boundary):
    if site_boundary is None:
        return True
    sample = ForwardConnectorSample(
        x=float(pose[0]),
        y=float(pose[1]),
        z=float(pose[2]),
        yaw=float(pose[3]),
    )
    return _candidate_inside_site_boundary((sample,), local_footprint, site_boundary)
```

- [ ] **Step 2: Classify endpoints before enumerating forward Dubins families**

After Turn Zone validation and before `_dubins_candidates(...)`:

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
            connector_id=request.connector_id,
            from_aisle_id=request.from_aisle_id,
            to_aisle_id=request.to_aisle_id,
            turn_zone_id=request.turn_zone_id,
            status="CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
            minimum_zone_extension_m=0.0,
            local_zone_extension_limit_m=0.0,
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

All non-endpoint-conflict results store the actual endpoint booleans; with no Site Boundary they are both `True`.

- [ ] **Step 3: Rename only the forward-path family status**

Change the old broad status to:

```text
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
```

only when both endpoints are legal, `local` is non-empty, and `local_safe` is empty.

Reason must state that endpoints are legal and the locally relevant forward family crosses/touches Site Boundary.

Boundary-conflicting candidates remain excluded from occupancy/map evidence counts.

- [ ] **Step 4: Serialize all boundary evidence deterministically**

For every connector result add:

```python
"start_endpoint_site_boundary_free": result.start_endpoint_site_boundary_free,
"goal_endpoint_site_boundary_free": result.goal_endpoint_site_boundary_free,
```

For every candidate add:

```python
"site_boundary_free": item.site_boundary_free,
```

Do not change `agt_forward_connector_candidate_audit/v1`.

- [ ] **Step 5: Backward-compatibly load historical v1 audit assets**

In `route_diagnostic_io.load_forward_connector_candidate_audit(...)`:

```python
site_boundary_free=bool(candidate.get("site_boundary_free", True)),
```

and for each result:

```python
start_endpoint_site_boundary_free=bool(
    item.get("start_endpoint_site_boundary_free", True)
),
goal_endpoint_site_boundary_free=bool(
    item.get("goal_endpoint_site_boundary_free", True)
),
```

Missing new fields in historical v1 assets therefore preserve old behavior rather than failing load.

- [ ] **Step 6: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py \
  src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py
git commit -m "feat(v25-12g): split A3 boundary endpoint and path evidence"
```

---

### Task 3: Amend R6A Admission Policy

**File:** `src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py`

**Signature:** unchanged.

- [ ] **Step 1: Add exact new decision mappings**

After occupancy-blocked handling:

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

Existing mappings remain:

```text
LOCAL_FORWARD_OCCUPANCY_BLOCKED -> ELIGIBLE_REVERSE_FALLBACK
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT -> HOLD_MAP_REVIEW
LOCAL_FORWARD_MIXED_EVIDENCE -> HOLD_MIXED_EVIDENCE unless explicitly approved
FORWARD_PREVIEW_FREE -> KEEP_FORWARD
other -> HOLD_POLICY_REVIEW
```

- [ ] **Step 2: Freeze amended source metadata**

```python
"admission_policy": "AUTO_LOCAL_OCCUPANCY_OR_ENDPOINT_SAFE_FORWARD_PATH_BOUNDARY",
"endpoint_boundary_conflict_never_reverse_admitted": True,
"mixed_evidence_requires_operator_approval": True,
"map_insufficient_never_auto_admitted": True,
"validation_scope": "R6B_INPUT_SELECTION_NOT_ROUTE_READY",
```

No R6A schema bump.

- [ ] **Step 3: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py
git commit -m "feat(v25-12g): admit endpoint-safe boundary fallback"
```

---

### Task 4: Route Amended Evidence Through A3 Transition Validation

**File:** `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`

**Signature:** `validate_transition_candidates(...)` unchanged.

**Forward evidence for every audited transition:**

```python
{
    "audit_status": item.status,
    "reason": item.reason,
    "start_endpoint_site_boundary_free": item.start_endpoint_site_boundary_free,
    "goal_endpoint_site_boundary_free": item.goal_endpoint_site_boundary_free,
}
```

- [ ] **Step 1: Audit all forward-gate failures, then derive one R6A plan for that audited set**

After `derive_forward_connector_candidate_audit(...)`:

```python
admission = derive_reverse_fallback_admission(
    audit,
    ReverseFallbackAdmissionConfig(operator_approved_mixed_connector_ids=()),
    source={"acceptance_stage": "v25_12g_a3"},
)
admission_by_id = {str(item.connector_id): item for item in admission.items}
```

Fail closed if any audited connector has no R6A item.

- [ ] **Step 2: Preserve exact non-R6B A3 mappings**

For each audited connector:

```text
CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT + REJECT_HARD_CONSTRAINT
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION

LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT + HOLD_MAP_REVIEW
    -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT

LOCAL_FORWARD_MIXED_EVIDENCE + HOLD_MIXED_EVIDENCE
    -> UNRESOLVED / MIXED_EVIDENCE_REQUIRES_REVIEW

LOCAL_FORWARD_POLICY_REVIEW + HOLD_POLICY_REVIEW
    -> UNRESOLVED / POLICY_REVIEW_REQUIRED

NO_FORWARD_DUBINS_CANDIDATE
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
    -> never R6B

TURN_ZONE_METADATA_INVALID
    -> ValueError

FORWARD_PREVIEW_FREE appearing in this unresolved path
    -> ValueError gate/audit disagreement
```

Always retain the R6A `decision` and `reason` in `reverse_admission_evidence` when an admission item exists.

- [ ] **Step 3: Send exactly R6A-eligible connectors to R6B**

Build requests from `admission.eligible_connector_ids`; this includes occupancy-blocked and endpoint-safe forward-path boundary conflicts only.

Invoke unchanged:

```python
ReversePrimitiveConnectorConfig(
    preview_footprint_padding_m=config.preview_footprint_padding_m,
)
```

Do not alter any other R6B field.

- [ ] **Step 4: Preserve existing R6B normalization**

```text
REVERSE_PRIMITIVE_PREVIEW_FREE / FORWARD_PRIMITIVE_PREVIEW_FREE
    -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE

SITE_BOUNDARY_CONFLICT
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION

NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
    -> REJECTED / BOUNDED_SEARCH_NO_SOLUTION

R6B_START_FOOTPRINT_NOT_FREE + OCCUPIED
    -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION

R6B_START_FOOTPRINT_NOT_FREE + UNKNOWN/out-of-grid
    -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
```

No accepted R6B path bypasses Site Boundary; R6B independently checks start, primitives, cusps, and goal shot.

- [ ] **Step 5: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
git commit -m "feat(v25-12g): route boundary path conflicts through R6B"
```

---

### Task 5: Amend Acceptance Harness Diagnostics

**File:** `tools/v25_12g_a3_acceptance.py`

**Report schema/scope unchanged:**

```text
agt_v25_12g_a3_acceptance_report/v1
A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY
```

- [ ] **Step 1: Replace old broad bypass logic with exact endpoint-admission safety**

Compute from transition records:

```python
endpoint_status = "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT"
path_status = "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
eligible = "ELIGIBLE_REVERSE_FALLBACK"

endpoint_boundary_conflict_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == endpoint_status
    for t in transition_records
)
forward_path_boundary_conflict_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == path_status
    for t in transition_records
)
endpoint_boundary_conflict_reverse_admission_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == endpoint_status
    and t.get("reverse_admission_evidence", {}).get("decision") == eligible
    for t in transition_records
)
forward_path_boundary_reverse_admitted_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == path_status
    and t.get("reverse_admission_evidence", {}).get("decision") == eligible
    for t in transition_records
)
```

Also compute:

```python
forward_path_boundary_reverse_executable_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == path_status
    and t["status"] == "EXECUTABLE"
    and t["backend"] == "BOUNDED_REVERSE_PRIMITIVE_SEARCH"
    for t in transition_records
)
forward_path_boundary_reverse_bounded_no_solution_count = sum(
    t.get("forward_evidence", {}).get("audit_status") == path_status
    and t["status"] == "REJECTED"
    and t["proof_scope"] == "BOUNDED_SEARCH_NO_SOLUTION"
    for t in transition_records
)
```

- [ ] **Step 2: Fail only on the actual forbidden endpoint admission**

```python
if endpoint_boundary_conflict_reverse_admission_count:
    raise ValueError(
        "A3 reverse fallback admitted a connector whose endpoint footprint violates Site Boundary"
    )
```

Remove `site_boundary_reverse_bypass_count` from the summary; it encoded the superseded broad interpretation.

- [ ] **Step 3: Preserve existing structural invariants**

Continue enforcing:

```text
all_a2_service_states_preserved == True
all_a2_connector_candidates_preserved == True
dead_end_non_retrace_count == 0
forbidden_semantic_key_count == 0
```

Read-only remains default; write remains sibling-only and overwrite-false.

- [ ] **Step 4: Commit**

```bash
git add tools/v25_12g_a3_acceptance.py
git commit -m "feat(v25-12g): report amended A3 boundary funnel"
```

---

### Task 6: Final GREEN Gate

- [ ] **Step 1: Build and run focused regression batch**

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
```

- [ ] **Step 2: Run package suite**

```bash
colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

Do not claim GREEN until fresh operator output shows zero failures.

---

### Task 7: Rerun the Frozen Greenhouse A3 Checkpoint

**Runtime directory:** `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`

**Vehicle profile:** `profiles/platforms/mk_mini.yaml`

**Protected upstream assets:**

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

- [ ] **Step 1: Freeze protected-input hashes**

Record `sha256sum` for all protected assets before any write.

- [ ] **Step 2: Run read-only amended A3 acceptance with timing**

```bash
/usr/bin/time -v \
  python3 tools/v25_12g_a3_acceptance.py \
    --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
    --vehicle-profile "$PWD/profiles/platforms/mk_mini.yaml" \
    --pretty \
    >/tmp/v25_12g_a3_amended_readonly.json
```

Required structural facts:

```text
service_validation_count == 32
transition_validation_count == 40
all_a2_service_states_preserved == True
all_a2_connector_candidates_preserved == True
dead_end_non_retrace_count == 0
endpoint_boundary_conflict_reverse_admission_count == 0
forbidden_semantic_key_count == 0
```

No expected executable-transition count is frozen.

- [ ] **Step 3: Report amended funnel by side**

For LOW_U and HIGH_U separately report:

```text
endpoint hard conflicts
forward-path boundary conflicts
R6A admitted connectors
R6B executable connectors
R6B bounded no-solution connectors
```

Also retain service coverage and four exact-retrace dead-end facts.

- [ ] **Step 4: Write the A3 sibling only after read-only evidence is internally consistent**

Use `--write-motion-graph` only if `vehicle_feasible_motion_graph.yaml` is absent. Never auto-overwrite.

- [ ] **Step 5: Prove protected inputs unchanged and A3 YAML deterministic**

Re-hash protected inputs; hashes must match. Strict-load `vehicle_feasible_motion_graph.yaml`, write to a temporary path, and assert byte-identical output.

- [ ] **Step 6: Freeze evidence docs only from observed results**

Create/update:

```text
docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md
docs/v2.5/V25_12G_A3_CURRENT_STATE.md
```

Strongest allowed wording remains local motion evidence only. Do not claim START reachability, route readiness, final coverage, or optimality.

- [ ] **Step 7: Commit docs only, never runtime assets**

```bash
git add \
  docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md \
  docs/v2.5/V25_12G_A3_CURRENT_STATE.md
git commit -m "docs(v25-12g): freeze amended A3 real-data evidence"
```

A4 may resume only after this amended A3 real-data checkpoint is frozen.
