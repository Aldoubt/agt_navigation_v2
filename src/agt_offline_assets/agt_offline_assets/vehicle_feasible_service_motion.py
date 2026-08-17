"""Directional A3 service-motion validation over frozen A2 resources."""

from __future__ import annotations

import math

from .forward_connector import ForwardConnectorSample
from .forward_connector_navigation_gate import (
    _candidate_inside_site_boundary,
    _evaluate_candidate,
    _preview_local_footprint,
)
from .navigation_grid import NavigationGridEvidence
from .site_boundary import SiteBoundary
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_feasible_motion_graph import (
    A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
    A1_CENTERLINE_EXACT_REVERSE_RETRACE,
    EXECUTABLE,
    LOCAL_MOTION_EXECUTABLE,
    MAP_EVIDENCE_INSUFFICIENT,
    MotionSample,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    REJECTED,
    ServiceActionValidation,
    UNRESOLVED,
    VehicleFeasibleMotionGraphConfig,
)
from .vehicle_feasible_service_graph import (
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    HIGH_U_HEADLAND,
    LOW_U_HEADLAND,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    VehicleFeasibleServiceGraph,
)


_HEADLAND_TYPES = {LOW_U_HEADLAND, HIGH_U_HEADLAND}
_VALID_SERVICE_TYPES = {
    SERVICE_LOW_TO_HIGH,
    SERVICE_HIGH_TO_LOW,
    DEAD_END_FORWARD_IN_REVERSE_OUT,
}


def _wrap_pi(angle: float) -> float:
    return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)


def _angle_error(a: float, b: float) -> float:
    return abs(_wrap_pi(float(a) - float(b)))


def _finite_pose(pose) -> bool:
    return len(pose) == 4 and all(math.isfinite(float(value)) for value in pose)


def _pose_matches(
    actual,
    expected,
    config: VehicleFeasibleMotionGraphConfig,
) -> bool:
    if not _finite_pose(actual) or not _finite_pose(expected):
        return False
    position_error = math.sqrt(
        sum((float(actual[index]) - float(expected[index])) ** 2 for index in range(3))
    )
    return (
        position_error <= config.pose_position_tolerance_m
        and _angle_error(actual[3], expected[3]) <= config.pose_yaw_tolerance_rad
    )


def _reverse_heading(pose):
    return (
        float(pose[0]),
        float(pose[1]),
        float(pose[2]),
        _wrap_pi(float(pose[3]) + math.pi),
    )


def _resource_index(service_graph: VehicleFeasibleServiceGraph):
    resources = {}
    for resource in service_graph.service_resources:
        segment_id = str(resource.segment_id)
        if not segment_id or segment_id in resources:
            raise ValueError(f"duplicate or empty service resource id: {segment_id}")
        if not resource.centerline_xyz:
            raise ValueError(f"service resource {segment_id} centerline must not be empty")
        for point in resource.centerline_xyz:
            if len(point) != 3 or not all(math.isfinite(float(value)) for value in point):
                raise ValueError(f"service resource {segment_id} centerline contains invalid point")
        if not _finite_pose(resource.low_endpoint_pose) or not _finite_pose(resource.high_endpoint_pose):
            raise ValueError(f"service resource {segment_id} endpoint pose is invalid")
        resources[segment_id] = resource
    return resources


def _validate_graph_identity(
    service_graph: VehicleFeasibleServiceGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig,
    site_boundary: SiteBoundary | None,
):
    config.validate()
    if service_graph.frame_id != navigation.frame_id:
        raise ValueError(
            f"service/navigation frame mismatch: {service_graph.frame_id} != {navigation.frame_id}"
        )
    if service_graph.platform_id != vehicle.profile_id:
        raise ValueError(
            f"service graph platform does not match vehicle profile: {service_graph.platform_id} != {vehicle.profile_id}"
        )
    if service_graph.platform_profile_sha256 != vehicle.profile_sha256:
        raise ValueError("service graph vehicle profile hash mismatch")
    if vehicle.kinematics != "ackermann" or not vehicle.planning_preview_ready:
        raise ValueError("A3 service validation requires a preview-ready Ackermann profile")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=service_graph.frame_id)


def _expected_poses(state, resource):
    if state.service_type == SERVICE_LOW_TO_HIGH:
        return resource.low_endpoint_pose, resource.high_endpoint_pose
    if state.service_type == SERVICE_HIGH_TO_LOW:
        return _reverse_heading(resource.high_endpoint_pose), _reverse_heading(resource.low_endpoint_pose)
    if state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT:
        low_headland = resource.low_endpoint_type in _HEADLAND_TYPES
        high_headland = resource.high_endpoint_type in _HEADLAND_TYPES
        if low_headland == high_headland:
            raise ValueError(
                f"dead-end service {state.service_state_id} requires exactly one headland endpoint"
            )
        endpoint = resource.low_endpoint_pose if low_headland else _reverse_heading(resource.high_endpoint_pose)
        return endpoint, endpoint
    raise ValueError(f"invalid A3 service type: {state.service_type}")


def _forward_points_and_yaw(state, resource):
    if state.service_type == SERVICE_LOW_TO_HIGH:
        return tuple(resource.centerline_xyz), float(state.entry_pose[3])
    if state.service_type == SERVICE_HIGH_TO_LOW:
        return tuple(reversed(resource.centerline_xyz)), float(state.entry_pose[3])
    if state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT:
        if resource.low_endpoint_type in _HEADLAND_TYPES:
            return tuple(resource.centerline_xyz), float(state.entry_pose[3])
        return tuple(reversed(resource.centerline_xyz)), float(state.entry_pose[3])
    raise ValueError(f"invalid A3 service type: {state.service_type}")


def _build_samples(state, resource):
    points, yaw = _forward_points_and_yaw(state, resource)
    forward = tuple(
        MotionSample(
            x=float(point[0]),
            y=float(point[1]),
            z=float(point[2]),
            yaw=_wrap_pi(yaw),
            motion_direction="FORWARD",
            segment_index=0,
            is_cusp=False,
        )
        for point in points
    )
    if state.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT:
        return forward
    terminal = forward[-1]
    cusp = MotionSample(
        x=terminal.x,
        y=terminal.y,
        z=terminal.z,
        yaw=terminal.yaw,
        motion_direction="REVERSE",
        segment_index=1,
        is_cusp=True,
    )
    reverse = tuple(
        MotionSample(
            x=sample.x,
            y=sample.y,
            z=sample.z,
            yaw=sample.yaw,
            motion_direction="REVERSE",
            segment_index=1,
            is_cusp=False,
        )
        for sample in reversed(forward[:-1])
    )
    return forward + (cusp,) + reverse


def _path_metrics(samples: tuple[MotionSample, ...]):
    forward = 0.0
    reverse = 0.0
    for previous, current in zip(samples[:-1], samples[1:]):
        distance = math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )
        if current.motion_direction == "REVERSE":
            reverse += distance
        else:
            forward += distance
    return forward + reverse, forward, reverse, sum(sample.is_cusp for sample in samples)


def _forward_samples(samples: tuple[MotionSample, ...]):
    return tuple(
        ForwardConnectorSample(
            x=sample.x,
            y=sample.y,
            z=sample.z,
            yaw=sample.yaw,
            motion_direction=sample.motion_direction,
        )
        for sample in samples
    )


def validate_service_actions(
    service_graph: VehicleFeasibleServiceGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig,
    *,
    site_boundary: SiteBoundary | None = None,
) -> tuple[ServiceActionValidation, ...]:
    """Revalidate every A2 service state using its actual directional footprint."""
    _validate_graph_identity(service_graph, navigation, vehicle, config, site_boundary)
    resources = _resource_index(service_graph)
    seen_states: set[str] = set()
    local_footprint = _preview_local_footprint(vehicle, config.preview_footprint_padding_m)
    actions: list[ServiceActionValidation] = []

    for state in sorted(service_graph.service_states, key=lambda item: str(item.service_state_id)):
        state_id = str(state.service_state_id)
        if not state_id or state_id in seen_states:
            raise ValueError(f"duplicate or empty service state id: {state_id}")
        seen_states.add(state_id)
        if state.service_type not in _VALID_SERVICE_TYPES:
            raise ValueError(f"invalid A3 service type: {state.service_type}")
        resource = resources.get(str(state.segment_id))
        if resource is None:
            raise ValueError(f"service state {state_id} references missing service resource {state.segment_id}")
        if state.aisle_id != resource.aisle_id:
            raise ValueError(f"service state {state_id} aisle/resource mismatch")
        if state.coverage_segment_id != resource.segment_id:
            raise ValueError(f"service state {state_id} coverage resource mismatch")
        if not _finite_pose(state.entry_pose) or not _finite_pose(state.exit_pose):
            raise ValueError(f"service state {state_id} contains invalid pose")

        expected_entry, expected_exit = _expected_poses(state, resource)
        if not _pose_matches(state.entry_pose, expected_entry, config):
            raise ValueError(f"service state {state_id} entry pose does not match resource")
        if not _pose_matches(state.exit_pose, expected_exit, config):
            raise ValueError(f"service state {state_id} exit pose does not match resource")

        samples = _build_samples(state, resource)
        path_length, forward_distance, reverse_distance, cusp_count = _path_metrics(samples)
        preview_samples = _forward_samples(samples)
        _centerline_evidence, footprint_evidence = _evaluate_candidate(
            preview_samples,
            navigation,
            local_footprint,
        )
        boundary_free = _candidate_inside_site_boundary(
            preview_samples,
            local_footprint,
            site_boundary,
        )
        if not boundary_free:
            status = REJECTED
            proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
            backend_status = "SITE_BOUNDARY_CONFLICT"
            reason = "service preview footprint touches or crosses the hard Site Boundary"
        elif footprint_evidence.occupied_fraction > 0.0:
            status = REJECTED
            proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
            backend_status = "OCCUPIED_FOOTPRINT_CONFLICT"
            reason = "service preview footprint intersects OCCUPIED Navigation Grid cells"
        elif (
            footprint_evidence.grid_coverage_fraction < 1.0 - 1.0e-12
            or footprint_evidence.unknown_fraction > 0.0
            or footprint_evidence.cell_count <= 0
        ):
            status = UNRESOLVED
            proof_scope = MAP_EVIDENCE_INSUFFICIENT
            backend_status = "MAP_EVIDENCE_INSUFFICIENT"
            reason = "service preview footprint depends on UNKNOWN or incomplete grid evidence"
        else:
            status = EXECUTABLE
            proof_scope = LOCAL_MOTION_EXECUTABLE
            backend_status = "PREVIEW_FOOTPRINT_FREE"
            reason = "directional service preview footprint is locally FREE"

        backend = (
            A1_CENTERLINE_EXACT_REVERSE_RETRACE
            if state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT
            else A1_CENTERLINE_DIRECTIONAL_REVALIDATION
        )
        actions.append(
            ServiceActionValidation(
                service_state_id=state_id,
                segment_id=str(state.segment_id),
                aisle_id=str(state.aisle_id),
                service_type=str(state.service_type),
                entry_pose=tuple(float(value) for value in state.entry_pose),
                exit_pose=tuple(float(value) for value in state.exit_pose),
                status=status,
                proof_scope=proof_scope,
                backend=backend,
                backend_status=backend_status,
                reason=reason,
                coverage_segment_id=str(state.coverage_segment_id),
                coverage_reward_length_m=float(state.coverage_reward_length_m),
                path_length_m=float(path_length),
                forward_distance_m=float(forward_distance),
                reverse_distance_m=float(reverse_distance),
                cusp_count=int(cusp_count),
                samples=samples,
                footprint_evidence=footprint_evidence,
            )
        )

    return tuple(actions)
