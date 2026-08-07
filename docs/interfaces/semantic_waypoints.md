# Semantic Waypoint Contract — V2.5 / schema 1.1

## Purpose

V25-04 adds a named semantic waypoint library to the existing semantic GeoJSON model without turning
semantic data into an execution sequence.

```text
semantic_map.geojson
        |
        | feature_type: waypoint
        v
agt_semantic_map_server
        |
        v
/agt/map/waypoints
agt_interfaces/msg/SemanticWaypointArray
```

The fundamental boundary is:

```text
Semantic Waypoint Library != Waypoint Task
```

A semantic waypoint is a persistent named anchor in a map. A waypoint task/task group is a finite,
ordered execution request. Mission and navigation code may resolve or copy semantic anchors into a task,
but `/agt/map/waypoints` is never itself an executable goal list.

## Versioning

- Semantic GeoJSON schema `1.0` remains supported for existing coverage documents.
- `feature_type: waypoint` requires semantic GeoJSON schema `1.1`.
- `coverage.yaml` remains schema `1.0` in V25-04. Its map/frame/base-map hash fields are reused as the
  binding record for waypoint-only mode; coverage planning parameters are not waypoint requirements.
- A schema-1.1 document may contain only waypoints, or may combine existing coverage features and
  waypoints.

## GeoJSON feature

Example:

```json
{
  "type": "Feature",
  "geometry": {
    "type": "Point",
    "coordinates": [2.35, 5.10]
  },
  "properties": {
    "id": "home",
    "feature_type": "waypoint",
    "name": "Home",
    "enabled": true,
    "frame_id": "map",
    "yaw": 1.5707963268,
    "role": "home",
    "position_tolerance": 0.30,
    "yaw_tolerance": 0.35,
    "preferred_speed": 0.20,
    "tags": ["safe", "charging"]
  }
}
```

Required properties are the normal semantic common properties plus finite `yaw`.

Optional waypoint properties and V25-04 defaults:

| Property | Default | Contract |
| --- | ---: | --- |
| `role` | `navigation` | non-empty string |
| `position_tolerance` | `0.30` m | finite, >= 0 |
| `yaw_tolerance` | `0.35` rad | finite, >= 0 |
| `preferred_speed` | `0.0` m/s | finite, >= 0; zero means no waypoint-specific override |
| `tags` | `[]` | list of strings |

A waypoint coordinate must be finite and, when base-map geometry is available, inside the map extent.
V25-04 does not require a waypoint to be inside a `field_boundary`; that is a task/application policy,
not generic map-document validity.

## Validation profiles

Validation is deliberately split into three profiles:

### Document validation

Checks document identity, schema, feature IDs/types, finite geometry/topology, frames and base-map bounds.
It does not require coverage features or waypoint features to exist.

### Coverage validation

Preserves the existing coverage requirements:

- `field_boundary`
- `exclusion_zone`
- `entry_pose`
- `work_direction`

Existing schema-1.0 coverage behavior remains compatible.

### Waypoint validation

Requires at least one enabled `waypoint` and validates waypoint-specific properties plus map binding.
It does not require coverage geometry.

## ROS interface

Canonical topic:

```text
/agt/map/waypoints
agt_interfaces/msg/SemanticWaypointArray
```

QoS:

```text
RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
```

`SemanticWaypoint.msg`:

```text
string id
string name
string role
geometry_msgs/Pose pose
float32 position_tolerance
float32 yaw_tolerance
float32 preferred_speed
string[] tags
bool enabled
```

`SemanticWaypointArray.msg`:

```text
std_msgs/Header header
string schema_version
string map_id
string base_map_sha256
agt_interfaces/SemanticWaypoint[] waypoints
```

The array is map-bound and version-aware but intentionally contains no execution-order field.

## Server modes

`agt_semantic_map_server` accepts:

```text
semantic_mode:=coverage
semantic_mode:=waypoint
```

`coverage` is the backward-compatible default. It performs current coverage validation, publishes semantic
markers and keepout mask, and also publishes a waypoint array when schema-1.1 waypoint features are present.

`waypoint` validates a waypoint library independently from coverage feature counts, publishes semantic
markers and `/agt/map/waypoints`, and does not create or replace `/agt/map/keepout_mask`.

A failed candidate load is transactional: the last active products remain authoritative.

## Execution boundary

Consumers must not convert the entire waypoint array directly into Nav2 motion. Formal motion continues
through project-owned task interfaces such as:

```text
/agt/navigation/execute_waypoint_task
```

The existing task registry/task-group contract remains the authority for ordered execution, revisions,
content hashes, finite loops, cancellation and session recovery.

## Current V25-04 scope

Implemented in the core branch:

- schema-1.1 waypoint contract;
- document/coverage/waypoint validation split;
- typed ROS messages;
- typed transient-local waypoint publication;
- coverage and waypoint server modes;
- regression tests for legacy coverage and waypoint-only documents.

Qt authoring integration is a separate UI step in V25-04 and must reuse the same two-click position/yaw
contract rather than creating a second persistent waypoint format.
