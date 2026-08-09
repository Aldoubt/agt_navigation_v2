"""Per-request 2D obstacle distance heuristic for Hybrid A*."""

from collections import deque


def build_obstacle_heuristic(context, goal_cell):
    grid = context.occupancy_grid
    distances = {goal_cell: 0}
    queue = deque([goal_cell])
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                continue
            if (nx, ny) in distances:
                continue
            value = grid.data[ny * grid.width + nx]
            if value >= 100 or (value < 0 and not context.unknown_space_allowed):
                continue
            distances[(nx, ny)] = distances[(x, y)] + 1
            queue.append((nx, ny))
    return distances
