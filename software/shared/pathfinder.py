"""
shared/pathfinder.py — 8-directional A* path planner on the occupancy grid.

Moved here from sim/world.py so it can be used identically by:
  - The simulator (SimBridge)
  - The hardware path (MotionService on Jetson)

The planner builds an *inflated* planning grid: obstacles are expanded by
ROBOT_RADIUS so the robot, treated as a point, never gets closer than its
own radius to any occupied cell.
Diagonal moves are only allowed when both orthogonal neighbours are free
(no corner-cutting).

Path simplification: collinear waypoints are merged to reduce stop count.
"""

import heapq
from typing import List

from shared.config import GRID_W, GRID_H, GRID_CELL, FIELD_W, FIELD_H, ROBOT_RADIUS, Vec2
from shared.occupancy import OccupancyGrid


class Pathfinder:
    """8-directional A* with obstacle inflation and path simplification."""

    DIRS = [
        (1,  0,  1.0),  (-1,  0,  1.0),
        (0,  1,  1.0),  (0,  -1,  1.0),
        (1,  1,  1.414), (-1,  1,  1.414),
        (1, -1,  1.414), (-1, -1,  1.414),
    ]

    # Inflation radius in cells: ceil(ROBOT_RADIUS / GRID_CELL).
    # With 126 mm / 150 mm → 1 cell.
    _INFLATE = max(1, int((ROBOT_RADIUS + GRID_CELL - 1) // GRID_CELL))

    def __init__(self, occupancy: OccupancyGrid):
        self.occ = occupancy

    # ── Inflated planning grid ────────────────────────────────────────────────

    def _build_plan_grid(self) -> List[List[bool]]:
        """Mark a cell blocked if a robot centred there would overlap any occupied cell."""
        pg = [[False] * GRID_H for _ in range(GRID_W)]
        r  = ROBOT_RADIUS
        for gx in range(GRID_W):
            for gy in range(GRID_H):
                cx = (gx + 0.5) * GRID_CELL
                cy = (gy + 0.5) * GRID_CELL
                # Check surrounding cells within inflation radius
                x0 = max(0, gx - self._INFLATE)
                x1 = min(GRID_W - 1, gx + self._INFLATE)
                y0 = max(0, gy - self._INFLATE)
                y1 = min(GRID_H - 1, gy + self._INFLATE)
                blocked = False
                for ox in range(x0, x1 + 1):
                    for oy in range(y0, y1 + 1):
                        if self.occ._static[ox][oy] or self.occ._dynamic[ox][oy]:
                            cell_cx = (ox + 0.5) * GRID_CELL
                            cell_cy = (oy + 0.5) * GRID_CELL
                            dx = max(abs(cx - cell_cx) - GRID_CELL / 2, 0.0)
                            dy = max(abs(cy - cell_cy) - GRID_CELL / 2, 0.0)
                            if dx * dx + dy * dy < r * r:
                                blocked = True
                                break
                    if blocked:
                        break
                pg[gx][gy] = blocked
                # Also block if robot would overlap field walls
                if (cx - r < 0 or cx + r > FIELD_W or
                        cy - r < 0 or cy + r > FIELD_H):
                    pg[gx][gy] = True
        return pg

    def _pg_blocked(self, pg, gx, gy) -> bool:
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return True
        return pg[gx][gy]

    # ── A* search ─────────────────────────────────────────────────────────────

    def find_path(self, start: Vec2, goal: Vec2) -> List[Vec2]:
        """Return a simplified list of waypoints from start to goal.
        Falls back to [start, goal] if no path is found."""
        gs = self.occ.world_to_grid(start)
        gg = self.occ.world_to_grid(goal)
        if gs == gg:
            return [start, goal]

        pg = self._build_plan_grid()

        # If goal cell is in the inflation zone (but not truly occupied), allow it.
        if 0 <= gg[0] < GRID_W and 0 <= gg[1] < GRID_H and pg[gg[0]][gg[1]]:
            if not (self.occ._static[gg[0]][gg[1]] or self.occ._dynamic[gg[0]][gg[1]]):
                pg[gg[0]][gg[1]] = False
        elif not (0 <= gg[0] < GRID_W and 0 <= gg[1] < GRID_H):
            return [start, goal]

        # Unblock start cell (robot is already there).
        if 0 <= gs[0] < GRID_W and 0 <= gs[1] < GRID_H:
            pg[gs[0]][gs[1]] = False

        g_score  = {gs: 0.0}
        came_from: dict = {}
        heap = [(self._h(gs, gg), 0.0, gs)]

        while heap:
            _, g, current = heapq.heappop(heap)
            if current == gg:
                path = self._reconstruct(came_from, current, start, goal)
                if len(path) > 2:
                    print(f"[Pathfinder] Path ({start.x:.0f},{start.y:.0f})→"
                          f"({goal.x:.0f},{goal.y:.0f}): {len(path)-2} waypoints")
                return path
            if g > g_score.get(current, float('inf')):
                continue
            cx, cy = current
            for dx, dy, cost in self.DIRS:
                nx, ny = cx + dx, cy + dy
                if self._pg_blocked(pg, nx, ny):
                    continue
                # Diagonal corner-cut guard
                if dx != 0 and dy != 0:
                    if (self._pg_blocked(pg, cx + dx, cy) or
                            self._pg_blocked(pg, cx, cy + dy)):
                        continue
                neighbor = (nx, ny)
                ng = g + cost
                if ng < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = ng
                    came_from[neighbor] = current
                    heapq.heappush(heap, (ng + self._h(neighbor, gg), ng, neighbor))

        # No path found — log and fall back to direct move
        print(f"[Pathfinder] NO PATH from ({start.x:.0f},{start.y:.0f}) to "
              f"({goal.x:.0f},{goal.y:.0f}) — falling back to direct move")
        return [start, goal]

    def _h(self, a, b) -> float:
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def _reconstruct(self, came_from, current, start, goal) -> List[Vec2]:
        cells = []
        while current in came_from:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        path = ([start] +
                [self.occ.grid_to_world(gx, gy) for gx, gy in cells] +
                [goal])
        return self._simplify(path)

    def _simplify(self, path: List[Vec2]) -> List[Vec2]:
        """Remove collinear intermediate waypoints."""
        if len(path) <= 2:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            v1 = path[i]     - path[i - 1]
            v2 = path[i + 1] - path[i]
            cross = v1.x * v2.y - v1.y * v2.x
            if abs(cross) > 200.0:
                result.append(path[i])
        result.append(path[-1])
        return result
