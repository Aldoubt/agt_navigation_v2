import math
from dataclasses import replace

from agt_coverage_planning.path_validator import GridMap

from agt_offline_assets import ConnectorPlanningContext, ConnectorRequest, create_connector_backend
from agt_offline_assets.hybrid_astar import grid_to_world, world_to_grid, yaw_to_bin, bin_to_yaw


def _context(*, blocked=(), unknown=()):
    data = [0] * (40 * 40)
    for x, y in blocked:
        data[y * 40 + x] = 100
    for x, y in unknown:
        data[y * 40 + x] = -1
    grid = GridMap(40, 40, 0.1, -2.0, -2.0, 0.0, tuple(data), "map")
    return ConnectorPlanningContext(
        occupancy_grid=grid,
        map_resolution_m=0.1,
        map_origin=(-2.0, -2.0, 0.0),
        footprint=((-0.05, -0.05), (0.05, -0.05), (0.05, 0.05), (-0.05, 0.05)),
        options={"max_iterations": 5000, "max_planning_time_s": 2.0, "angle_bins": 72},
    )


def test_hybrid_astar_world_grid_and_yaw_round_trip():
    context = ConnectorPlanningContext(
        occupancy_grid=GridMap(20, 20, 0.5, 10.0, 20.0, math.pi / 4.0, tuple([0] * 400), "map"),
        map_resolution_m=0.5,
        map_origin=(10.0, 20.0, math.pi / 4.0),
        footprint=((0.0, 0.0),),
    )
    assert world_to_grid(context, *grid_to_world(context, 3, 4)) == (3, 4)
    assert abs(bin_to_yaw(yaw_to_bin(0.37, 72), 72) - 0.37) < 2.0 * math.pi / 72.0


def test_hybrid_astar_open_space_and_determinism():
    request = ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.1, 0.5, True)
    backend = create_connector_backend("hybrid_astar")
    first = backend.plan(request, _context())
    second = backend.plan(request, _context())
    assert first.success
    assert first.samples == second.samples
    assert abs(first.samples[0].x - request.start_pose[0]) <= 1.0e-6
    assert abs(first.samples[-1].x - request.goal_pose[0]) <= 1.0e-6


def test_hybrid_astar_requires_context_and_rejects_start_collision():
    request = ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.1, 0.5, True)
    backend = create_connector_backend("hybrid_astar")
    missing = backend.plan(request)
    blocked = backend.plan(request, _context(blocked={(20, 20)}))
    assert not missing.success and missing.failure_reason == "HYBRID_ASTAR_CONTEXT_MISSING"
    assert not blocked.success and blocked.failure_reason == "HYBRID_ASTAR_START_COLLISION"


def test_hybrid_astar_obstacle_aware_gap_and_keepout():
    request = ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.1, 0.5, True)
    context = replace(
        _context(blocked={(25, y) for y in range(18, 23)}),
        options={"max_iterations": 100000, "max_planning_time_s": 10.0, "angle_bins": 72},
    )
    result = create_connector_backend("hybrid_astar").plan(request, context)
    assert result.success
    assert result.backend == "hybrid_astar"


def test_hybrid_astar_unknown_policy_and_impossible_barrier_fail_closed():
    request = ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.1, 0.5, True)
    unknown = _context(unknown={(x, y) for x in range(40) for y in range(40)})
    result = create_connector_backend("hybrid_astar").plan(request, unknown)
    assert not result.success
    assert result.failure_reason == "HYBRID_ASTAR_START_COLLISION"

    barrier = replace(
        _context(blocked={(25, y) for y in range(40)}),
        options={"max_iterations": 2000, "max_planning_time_s": 0.2, "angle_bins": 72},
    )
    result = create_connector_backend("hybrid_astar").plan(request, barrier)
    assert not result.success
    assert result.failure_reason in {"HYBRID_ASTAR_NO_PATH", "HYBRID_ASTAR_TIMEOUT", "HYBRID_ASTAR_MAX_ITERATIONS"}
