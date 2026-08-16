# V25-12G-A2 Service / Connectivity Graph Design

Date: 2026-08-16
Branch: `feat/v25-12g-maximum-feasible-coverage`
Status: DESIGN APPROVED FOR SPEC REVIEW

## 1. Purpose

V25-12G-A2 converts the A1 vehicle-feasible physical segment evidence into a static, deterministic, directional service-state topology graph.

A2 answers only these questions:

- Which physical coverage segments exist?
- In which directions can each segment conceptually be serviced?
- Which one-headland segments admit a dead-end forward-in / reverse-out service candidate?
- Which service-state exits and entries are topologically compatible through a shared same-side Turn Zone?
- Which service states belong to the same candidate topology island?

A2 does not prove motion feasibility, robot reachability, route readiness, or optimality.

The stage boundary is:

```text
A1: where vehicle-feasible coverage segments exist
  -> A2: how those segments may be serviced and which transitions are worth trying
  -> A3: whether those transitions / dead-end motions are actually executable
  -> A4: which validated services and transitions maximize feasible coverage
```

## 2. Frozen design decisions

The following decisions are normative for A2.

### 2.1 Directional service-state graph

A2 uses a directional service-state graph.

Each physical A1 segment geometry exists once as a coverage resource. Direction and service mode are represented by separate service states that reference that resource.

```text
A1 VehicleFeasibleSegment
        | 1:1
        v
ServiceResource
        | 1:N
        v
ServiceState
```

### 2.2 Interior-to-interior segments are preserved

A segment with:

```text
INTERIOR_BLOCKED_END <-> INTERIOR_BLOCKED_END
```

is not discarded.

It receives ordinary directional service states, but no cross-aisle connector candidate is emitted from either interior endpoint. Its external reachability remains unproven.

This preserves A1 coverage evidence without inventing access.

### 2.3 One-headland dead-end mode

A segment with exactly one headland endpoint receives one additional service candidate:

```text
DEAD_END_FORWARD_IN_REVERSE_OUT
```

The first A2 version does not generate:

```text
DEAD_END_REVERSE_IN_FORWARD_OUT
```

The dead-end state is topology-only. A3 must validate the reverse service motion before it can become executable.

### 2.4 Headland connector topology uses TurnZoneSet

A headland connector candidate is emitted only when all of the following are true:

- the source service-state exit is a headland endpoint;
- the destination service-state entry is a headland endpoint;
- both endpoints are on the same headland side;
- a shared compatible Turn Zone supports both aisles;
- that Turn Zone permits turning;
- source and destination physical segment IDs differ.

Different-side headlands are never connected directly in A2.

A2 uses Turn Zones only as topology/search-domain evidence. A Turn Zone never implies collision-free motion.

### 2.5 START_POSE is excluded from A2

A2 is a reusable map-level static topology asset.

`START_POSE` is not part of the canonical A2 graph. Start anchoring and robot-specific reachability are deferred to later mission-level processing in A3/A4.

A2 may compute candidate topology components, but it must never claim that a component is reachable from the robot.

### 2.6 Coverage identity is the physical segment

Coverage is deduplicated by:

```text
segment_id
```

not by:

```text
service_state_id
```

The same physical segment may have multiple service states, but it can earn coverage reward only once in A4.

For a dead-end service candidate, forward-in and reverse-out still refer to the same physical segment and therefore the same coverage identity.

### 2.7 Connector candidates are directed

A2 connector candidates are directed.

```text
A -> B
```

and:

```text
B -> A
```

are distinct candidates with distinct identities and are validated separately by A3.

Topological compatibility does not imply symmetric executable motion.

### 2.8 Candidate topology components are diagnostic only

A2 computes:

```text
CANDIDATE_TOPOLOGY_COMPONENT
```

This means only that states belong to the same potential topology island under A2 candidate relations.

It does not mean:

- `EXECUTABLE`;
- `A3_VALIDATED`;
- `REACHABLE_FROM_START`;
- `ROUTE_READY`.

A3 may split a single A2 candidate component into multiple executable components after motion feasibility rejection.

### 2.9 Connector semantics are exit-to-entry

Every A2 headland connector candidate is strictly:

```text
from_service_state.exit -> to_service_state.entry
```

A physical segment merely having a headland endpoint does not make every service state connectable at that headland.

Only the actual entry/exit endpoint of the current directional service state participates in headland connectivity.

## 3. Inputs

A2 consumes two static map-level inputs:

```text
VehicleFeasibleSegmentPlan   # A1
TurnZoneSet                  # existing offline asset
```

A2 must not derive new vehicle-feasible geometry, mutate A1 evidence, or inspect START_POSE.

The A1 segment contract is the source of physical segment truth, including:

- `segment_id`;
- `aisle_id`;
- `ordinal_in_aisle`;
- `length_m`;
- `coverage_fraction_of_aisle`;
- `low_endpoint_type`;
- `high_endpoint_type`;
- `low_endpoint_pose`;
- `high_endpoint_pose`;
- `centerline_xyz`;
- `frame_id`;
- `platform_id`;
- `platform_profile_sha256`;
- `row_direction_xy`.

Rejected A1 short fragments remain A1 diagnostics and do not enter the A2 service graph.

## 4. Output artifact

A2 writes one sibling artifact:

```text
vehicle_feasible_service_graph.yaml
```

Schema:

```text
agt_vehicle_feasible_service_graph/v1
```

Top-level status:

```text
TOPOLOGY_CANDIDATE_ONLY
```

Conceptual top-level structure:

```text
VehicleFeasibleServiceGraph
├── frame_id
├── platform_id
├── platform_profile_sha256
├── row_direction_xy
├── service_resources[]
├── service_states[]
├── connector_candidates[]
├── candidate_components[]
├── diagnostics
├── source
├── schema
└── status
```

A2 may create or overwrite only its own artifact when explicitly requested. It must not modify:

- `vehicle_feasible_segments.yaml`;
- `navigation_map.yaml`;
- `navigation_map.pgm`;
- `derivation.yaml`;
- `aisle_graph.yaml`;
- `site_boundary.yaml`;
- `turn_zones.yaml`.

## 5. ServiceResource contract

Each active A1 segment produces exactly one `ServiceResource`.

The stable coverage identity remains the A1 `segment_id`.

Minimum fields:

```text
segment_id
aisle_id
ordinal_in_aisle
coverage_length_m
coverage_fraction_of_aisle
low_endpoint_type
high_endpoint_type
low_endpoint_pose
high_endpoint_pose
centerline_xyz
```

If A1 geometry is carried into the A2 artifact, it is copied A1 truth, not independently re-derived A2 geometry.

Normative invariant:

```text
resource_count == A1 active segment count
```

and:

```text
abs(
  sum(unique resource coverage_length_m)
  - sum(A1 active segment length_m)
) <= 1e-9 m
```

before serialization rounding.

## 6. ServiceState contract

A2 v1 supports exactly these service types:

```text
SERVICE_LOW_TO_HIGH
SERVICE_HIGH_TO_LOW
DEAD_END_FORWARD_IN_REVERSE_OUT
```

Each physical resource always receives two ordinary directional states.

### 6.1 Stable IDs

Ordinary state IDs are semantic and deterministic:

```text
<segment_id>.service_low_to_high
<segment_id>.service_high_to_low
```

Dead-end state ID:

```text
<segment_id>.dead_end_forward_in_reverse_out
```

### 6.2 Ordinary LOW -> HIGH state

```text
entry_endpoint_type = resource.low_endpoint_type
exit_endpoint_type  = resource.high_endpoint_type
entry_pose           = A1.low_endpoint_pose
exit_pose            = A1.high_endpoint_pose
service_motion_direction = FORWARD
forward_service_distance_m = resource.coverage_length_m
reverse_service_distance_m = 0
coverage_segment_id  = resource.segment_id
coverage_reward_length_m = resource.coverage_length_m
validation_status = TOPOLOGY_SERVICE_CANDIDATE
```

### 6.3 Ordinary HIGH -> LOW state

A1 endpoint yaw follows the canonical LOW -> HIGH row heading. Therefore HIGH -> LOW forward travel must reverse the heading by pi while keeping endpoint position unchanged.

```text
entry_endpoint_type = resource.high_endpoint_type
exit_endpoint_type  = resource.low_endpoint_type

entry xyz = A1.high_endpoint_pose.xyz
entry yaw = normalize_angle(A1.high_endpoint_pose.yaw + pi)

exit xyz  = A1.low_endpoint_pose.xyz
exit yaw  = normalize_angle(A1.low_endpoint_pose.yaw + pi)
```

`normalize_angle()` uses the canonical half-open range:

```text
[-pi, pi)
```

Other fields match ordinary forward service accounting:

```text
service_motion_direction = FORWARD
forward_service_distance_m = resource.coverage_length_m
reverse_service_distance_m = 0
coverage_segment_id = resource.segment_id
coverage_reward_length_m = resource.coverage_length_m
validation_status = TOPOLOGY_SERVICE_CANDIDATE
```

Reversing the canonical traversal direction is ordinary forward motion with reversed heading. It is not reverse gear.

### 6.4 Dead-end state generation

Generate exactly one dead-end state only when the resource has exactly one headland endpoint.

Valid endpoint patterns:

```text
LOW_U_HEADLAND + INTERIOR_BLOCKED_END
INTERIOR_BLOCKED_END + HIGH_U_HEADLAND
```

Do not generate a dead-end state for:

```text
LOW_U_HEADLAND + HIGH_U_HEADLAND
INTERIOR_BLOCKED_END + INTERIOR_BLOCKED_END
```

For a LOW_U headland dead-end:

```text
entry_endpoint_type = LOW_U_HEADLAND
exit_endpoint_type  = LOW_U_HEADLAND
entry_pose = A1.low_endpoint_pose
exit_pose  = A1.low_endpoint_pose
```

For a HIGH_U headland dead-end:

```text
entry_endpoint_type = HIGH_U_HEADLAND
exit_endpoint_type  = HIGH_U_HEADLAND
entry_pose = reverse_heading(A1.high_endpoint_pose)
exit_pose  = reverse_heading(A1.high_endpoint_pose)
```

`reverse_heading(pose)` preserves position and applies `normalize_angle(yaw + pi)`.

Dead-end accounting:

```text
forward_service_distance_m = resource.coverage_length_m
reverse_service_distance_m = resource.coverage_length_m
coverage_segment_id = resource.segment_id
coverage_reward_length_m = resource.coverage_length_m
validation_status = REQUIRES_A3_REVERSE_SERVICE_VALIDATION
```

The reverse distance in A2 is an accounting expectation, not an A3-validated reverse path length.

### 6.5 External reachability status

Reachability is orthogonal to service validity.

Allowed A2 values:

```text
HEADLAND_TOPOLOGY_CANDIDATE
EXTERNAL_REACHABILITY_UNPROVEN
```

The field is defined from the service state's actual entry endpoint only:

```text
if entry_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}:
    external_reachability_status = HEADLAND_TOPOLOGY_CANDIDATE
else:
    external_reachability_status = EXTERNAL_REACHABILITY_UNPROVEN
```

`HEADLAND_TOPOLOGY_CANDIDATE` means only that the state begins at a headland and may later receive a compatible incoming topology candidate. It does not mean such a connector exists or is executable.

A double-interior ordinary state therefore always carries:

```text
EXTERNAL_REACHABILITY_UNPROVEN
```

No A2 status may claim actual reachability.

## 7. Headland port model

A2 derives internal headland ports from service states.

An outgoing headland port exists when:

```text
state.exit_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
```

An incoming headland port exists when:

```text
state.entry_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
```

Each internal port carries:

```text
service_state_id
segment_id
aisle_id
side
pose
```

Ports are an implementation concept. They do not become public serialized objects in schema v1.

## 8. ConnectorCandidate contract

Minimum fields:

```text
connector_candidate_id
from_service_state_id
to_service_state_id
from_segment_id
to_segment_id
side
turn_zone_id
start_pose
goal_pose
candidate_kind = HEADLAND_CONNECTOR_CANDIDATE
validation_status = REQUIRES_A3_CONNECTOR_VALIDATION
```

For every candidate:

```text
start_pose = from_service_state.exit_pose
goal_pose  = to_service_state.entry_pose
```

### 8.1 Directed edge semantics

Every connector is:

```text
from_service_state.exit -> to_service_state.entry
```

A2 never generates an undirected executable edge.

### 8.2 Stable connector IDs

Connector identity is semantic, not traversal-order numbering.

The logical uniqueness key is exactly:

```text
(
  from_service_state_id,
  to_service_state_id,
  turn_zone_id
)
```

The serialized `connector_candidate_id` is exactly:

```text
headland.<turn_zone_id>.<from_service_state_id>.to.<to_service_state_id>
```

The ID is used as an opaque stable identifier; consumers must use the explicit fields rather than reparsing the string.

### 8.3 Connector emission rules

Generate a directed candidate if and only if:

```text
from.exit is a headland
AND
to.entry is a headland
AND
from.side == to.side
AND
zone.side == from.side
AND
from.aisle_id in zone.supported_aisle_ids
AND
to.aisle_id in zone.supported_aisle_ids
AND
zone.allow_turn == true
AND
from.segment_id != to.segment_id
```

The following never produce connector candidates:

- LOW_U -> HIGH_U direct transitions;
- HIGH_U -> LOW_U direct transitions;
- INTERIOR -> HEADLAND cross-aisle transitions;
- HEADLAND -> INTERIOR cross-aisle transitions;
- INTERIOR -> INTERIOR cross-aisle transitions;
- unsupported aisle pairs;
- `allow_turn == false`;
- self-segment transitions.

### 8.4 Same aisle, different segment

A2 must not add a separate rule forbidding:

```text
from.aisle_id == to.aisle_id
```

when physical segment IDs differ.

Such a candidate is decided by the same headland/Turn-Zone rules as any other pair. Current A1 endpoint classification normally limits this case, but A2 must not depend on that incidental behavior.

### 8.5 Multiple Turn Zones

If two different Turn Zones support the same directed state pair, they are distinct candidates because A3 search domains differ.

Therefore:

```text
same from/to + different turn_zone_id != duplicate
```

A duplicate of the same triple:

```text
(from_state, to_state, turn_zone_id)
```

is an error, not a silent overwrite.

## 9. CandidateTopologyComponent contract

A2 components are diagnostic candidate topology islands.

Each component contains at minimum:

```text
component_id
classification = CANDIDATE_TOPOLOGY_COMPONENT
service_state_ids
segment_ids
connector_candidate_ids
total_unique_coverage_length_m
distinct_aisle_count
```

### 9.1 Component graph

The diagnostic component graph uses `service_state_id` as nodes and two kinds of undirected diagnostic relation:

1. same-resource membership: states sharing the same `segment_id` belong to the same candidate topology component;
2. any directed connector candidate joins its source and destination states for component grouping.

The same-resource relation is not a motion edge and must not be serialized as a connector candidate.

The connector graph itself remains directed. Direction is ignored only for component grouping.

Equivalent definition:

```text
resource-membership relation
+
weak connectivity induced by directed connector candidates
-> CANDIDATE_TOPOLOGY_COMPONENT
```

### 9.2 Why same-resource grouping is required

A double-interior segment still has two ordinary directional service states. They must remain grouped as one physical coverage resource for diagnostics without inventing an executable state-to-state maneuver.

Therefore A2 must not create fake edges such as:

```text
SERVICE_LOW_TO_HIGH <-> SERVICE_HIGH_TO_LOW
```

simply to keep them in one component.

### 9.3 Component IDs

Components are deterministically ordered by their lexicographically smallest member `service_state_id`, then assigned:

```text
candidate_component_001
candidate_component_002
...
```

Component IDs are deterministic diagnostic labels, not permanent domain identities.

An isolated component is defined exactly as a component whose:

```text
connector_candidate_count == 0
```

It may still contain multiple service states belonging to the same physical segment through the non-motion resource-membership relation.

## 10. Deterministic generation algorithm

A2 derivation is fixed to the following sequence:

```text
[1] validate inputs
[2] build ServiceResource records
[3] expand directional ServiceState records
[4] derive internal headland ports
[5] generate directed ConnectorCandidate records
[6] derive CandidateTopologyComponent records
[7] compute diagnostics
[8] serialize deterministically
```

No partial graph may be returned when a fail-closed input contract is violated.

### 10.1 Resource ordering

Resources are derived from A1 active segments using deterministic order:

```text
(aisle_id, ordinal_in_aisle, segment_id)
```

Serialized `service_resources` are ordered lexically by `segment_id`.

### 10.2 State ordering

Serialized service states are ordered by:

```text
segment_id lexical
then service type order:
  SERVICE_LOW_TO_HIGH
  SERVICE_HIGH_TO_LOW
  DEAD_END_FORWARD_IN_REVERSE_OUT
```

### 10.3 Connector ordering

Serialized connector candidates are ordered by:

```text
(
  turn_zone_id,
  from_service_state_id,
  to_service_state_id
)
```

### 10.4 Component ordering

Components are ordered by the minimum lexical service-state ID in each component, then numbered sequentially.

Inside each component:

- `service_state_ids` are lexical;
- `segment_ids` are lexical and unique;
- `connector_candidate_ids` are lexical.

The same semantic input must produce byte-stable YAML under the project serializer conventions.

## 11. Fail-closed validation

A2 must reject the entire derivation for topology truth conflicts.

### 11.1 Required hard errors

The following are hard errors:

```text
A1.frame_id != TurnZoneSet.frame_id
```

Row direction must match in orientation, not merely axis. Define:

```text
a = normalize(A1.row_direction_xy)
b = normalize(TurnZoneSet.row_direction_xy)
```

and require:

```text
||a - b||_2 <= 1e-9
AND
dot(a, b) > 0
```

A reversed row direction is an error because it swaps LOW_U / HIGH_U meaning.

Also reject:

- duplicate `segment_id`;
- duplicate `service_state_id`;
- duplicate connector identity;
- duplicate `zone_id`;
- unknown endpoint type;
- unknown service type;
- unknown Turn Zone side;
- missing or non-finite endpoint pose;
- non-finite row direction;
- zero row direction;
- segment length `<= 0`;
- empty `platform_id`;
- empty `platform_profile_sha256`.

The A2 output copies `platform_id` and `platform_profile_sha256` exactly from A1. `TurnZoneSet` currently has no platform identity and therefore is not a platform-metadata comparison source in A2 v1.

### 11.2 Normal no-edge outcomes

These are valid topology outcomes, not system errors:

- aisle not supported by a zone;
- side incompatible with a zone;
- `allow_turn == false`;
- no shared Turn Zone;
- zero connector candidates;
- isolated double-interior resources;
- multiple isolated candidate components.

A2 must represent lack of connectivity honestly instead of manufacturing edges to create a larger graph.

## 12. Provenance

Top-level source metadata is:

```yaml
source:
  vehicle_feasible_segment_schema: agt_vehicle_feasible_segment_plan/v1
  vehicle_feasible_segments_asset: vehicle_feasible_segments.yaml
  turn_zone_schema: agt_turn_zones/v1
  turn_zones_asset: turn_zones.yaml
  derivation_kind: V25_12G_A2_STATIC_SERVICE_TOPOLOGY
```

Provenance is for auditability. The loader/deriver must still validate loaded objects rather than trusting source strings as truth.

## 13. Diagnostics

Top-level diagnostics include at least:

```text
resource_count
ordinary_service_state_count
dead_end_candidate_count
total_service_state_count
connector_candidate_count
low_u_connector_candidate_count
high_u_connector_candidate_count
candidate_component_count
isolated_component_count
externally_unproven_state_count
unique_coverage_length_m
distinct_aisle_count
```

`isolated_component_count` counts exactly those components with zero connector candidates.

Each component exposes at least:

```text
service_state_count
unique_segment_count
connector_candidate_count
unique_coverage_length_m
distinct_aisle_count
```

Required internal sanity checks before serialization rounding:

```text
resource_count == A1 active segment count
```

and:

```text
abs(
  unique_coverage_length_m
  - sum(A1 active segment lengths)
) <= 1e-9 m
```

## 14. Forbidden A2 claims and fields

Schema v1 must not expose fields that imply later-stage truth, including:

```text
executable: true
reachable_from_start: true
connector_feasible: true
route_ready: true
optimal: true
```

It must also not claim A3-derived motion values such as:

```text
validated_connector_length_m
validated_reverse_connector_length_m
cusp_count
search_expansions
```

A2 language is restricted to topology concepts such as:

```text
candidate
unvalidated
unproven
```

## 15. Workbench scope

A2-Core does not require executable-route visualization.

The core implementation scope is:

```text
schema
derivation
strict loader/writer
diagnostics
acceptance harness
```

A later A2-Workbench diagnostic follow-up may visualize:

- A1 segment geometry;
- entry/exit direction markers;
- abstract candidate relationships;
- component membership.

It must not draw unvalidated connector candidates as plausible executable Dubins or reverse paths.

Any A2 topology overlay must visibly state that it is:

```text
A2 TOPOLOGY CANDIDATE
UNVALIDATED
NOT EXECUTABLE
```

Workbench visualization is not required to freeze the A2 core data contract.

## 16. A2 / A3 hard boundary

A2 may answer:

- which segment service directions exist;
- which segment has a one-headland dead-end service candidate;
- where a service state actually enters and exits;
- whether source exit and destination entry are same-side headlands;
- whether a shared compatible Turn Zone supports a transition attempt;
- which states belong to the same candidate topology island.

A2 must not answer:

- whether Dubins can connect two poses;
- whether reverse fallback is necessary or valid;
- whether a connector footprint collides;
- whether a connector remains inside Site Boundary;
- whether Navigation Grid is free along a connector;
- whether minimum turning radius is satisfied;
- validated connector length;
- cusp count;
- START_POSE reachability;
- task route readiness.

The intended interface is:

```text
A2 Directed ConnectorCandidate
        |
        | start_pose
        | goal_pose
        | turn_zone_id
        | side
        v
A3 motion candidate generation
        +
full footprint / map / Site Boundary / kinematic gates
        v
Validated Connector Edge
or
Rejected Connector Evidence
```

Dead-end service candidates follow the same rule: A2 expresses topology; A3 proves forward-in / reverse-out motion feasibility.

## 17. Acceptance criteria

A2 is not accepted merely because a YAML file is produced.

### 17.1 Contract and unit acceptance

Tests must prove:

- every active A1 segment produces exactly one resource;
- every resource produces exactly two ordinary directional states;
- exactly-one-headland resource produces exactly one dead-end candidate;
- zero-headland and two-headland resources produce no dead-end state;
- HIGH -> LOW yaw is reversed by pi and normalized to `[-pi, pi)`;
- no interior cross-aisle connector exists;
- no LOW_U <-> HIGH_U direct connector exists;
- no self-segment connector exists;
- connector candidates are directed;
- all connector candidates require A3 validation;
- coverage is deduplicated by `segment_id`.

### 17.2 Serialization acceptance

Tests must prove:

- write -> load preserves semantic equality;
- same semantic input produces deterministic serialized output;
- unknown schema fails closed;
- duplicate stable IDs fail closed;
- frame mismatch fails closed;
- reversed/inconsistent row direction fails closed.

### 17.3 Topology acceptance

Tests must prove:

- same-side + shared compatible Turn Zone creates directed candidates;
- unsupported aisle produces no candidate;
- `allow_turn == false` produces no candidate;
- isolated interior-interior resource remains valid and preserved;
- same-resource states remain in the same diagnostic component without a fake motion edge;
- connector candidates join candidate topology components according to weak connectivity;
- component coverage totals deduplicate by `segment_id`.

### 17.4 Real-data acceptance

Use the frozen greenhouse run:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run
```

with the already accepted A1 asset and the existing Turn Zone asset.

The current A1 real-data checkpoint contains:

```text
14 active physical segments
97.35667393520538 m unique active coverage
```

A2 real-data acceptance must verify without preselecting expected connector counts:

- all 14 A1 active segments are preserved as resources;
- `abs(A2_unique_coverage_m - 97.35667393520538) <= 1e-6 m`;
- A2 does not regress to longest-only behavior;
- double-interior segments remain preserved;
- one-headland segments receive the correct dead-end candidate state;
- LOW_U and HIGH_U connector candidates remain strictly separated;
- no INTERIOR cross-row edge is generated;
- candidate component structure is explainable as topology-only evidence.

Permitted checkpoint classifications are:

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED
```

or:

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED WITH CONNECTIVITY / MAP CAVEATS
```

A2 real-data acceptance must not declare:

```text
ROUTE READY
EXECUTABLE COVERAGE GRAPH
```

## 18. Scope exclusions

V25-12G-A2 explicitly excludes:

- START anchoring;
- Dubins connector generation;
- reverse connector search;
- footprint connector feasibility;
- Navigation Grid connector collision validation;
- Site Boundary connector validation;
- minimum-turning-radius connector acceptance;
- validated connector length / reverse distance / cusp metrics;
- global route optimization;
- RL policies;
- human route forcing;
- manual safety override;
- canonical map mutation;
- A1 re-derivation;
- route-ready promotion.

## 19. Final architecture checkpoint

The intended 12G chain after A2 is:

```text
A1 Maximum Feasible Segment Extraction
        v
physical coverage resources
        v
A2 Directional Service-State / Candidate Connectivity Graph
        v
service states
dead-end candidates
directed headland connector candidates
candidate topology components
        v
A3 Connector + Reverse Motion Feasibility
        v
validated executable graph
        v
A4 Maximum Feasible Coverage Optimizer
```

A4 must eventually optimize over already validated service and transition truth. It must not be responsible for reconstructing map safety semantics that belong in A1-A3.
