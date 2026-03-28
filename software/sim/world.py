"""
sim/world.py — OccupancyGrid and GameObjects for the simulator.
"""

import heapq
from shared.config import (
    GRID_W, GRID_H, GRID_CELL, FIELD_W, FIELD_H, ROBOT_RADIUS,
    OBS_RADIUS, Vec2, ObjectColor, POI,
)


class OccupancyGrid:
    """20×13 binary grid, 150mm/cell. Also stores dynamic obstacles."""

    def __init__(self):
        self._grid: list[list[bool]] = [[False] * GRID_H for _ in range(GRID_W)]
        self.dynamic_obstacles: list[Vec2] = []

    def reset(self):
        for col in self._grid:
            for i in range(len(col)):
                col[i] = False
        self.dynamic_obstacles.clear()

    def toggle_cell(self, gx: int, gy: int):
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            self._grid[gx][gy] = not self._grid[gx][gy]

    def set_cell(self, gx: int, gy: int, value: bool):
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            self._grid[gx][gy] = value

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return True
        return self._grid[gx][gy]

    def world_to_grid(self, pos: Vec2) -> tuple[int, int]:
        return (int(pos.x // GRID_CELL), int(pos.y // GRID_CELL))

    def grid_to_world(self, gx: int, gy: int) -> Vec2:
        return Vec2((gx + 0.5) * GRID_CELL, (gy + 0.5) * GRID_CELL)

    def is_occupied_world(self, pos: Vec2) -> bool:
        gx, gy = self.world_to_grid(pos)
        return self.is_cell_occupied(gx, gy)

    def is_occupied_circle(self, center: Vec2, radius: float) -> bool:
        if (center.x - radius < 0 or center.x + radius > FIELD_W or
                center.y - radius < 0 or center.y + radius > FIELD_H):
            return True
        x0 = max(0, int((center.x - radius) // GRID_CELL))
        x1 = min(GRID_W - 1, int((center.x + radius) // GRID_CELL))
        y0 = max(0, int((center.y - radius) // GRID_CELL))
        y1 = min(GRID_H - 1, int((center.y + radius) // GRID_CELL))
        for gx in range(x0, x1 + 1):
            for gy in range(y0, y1 + 1):
                if self._grid[gx][gy]:
                    cx = (gx + 0.5) * GRID_CELL
                    cy = (gy + 0.5) * GRID_CELL
                    dx = max(abs(center.x - cx) - GRID_CELL / 2, 0.0)
                    dy = max(abs(center.y - cy) - GRID_CELL / 2, 0.0)
                    if dx * dx + dy * dy < radius * radius:
                        return True
        for obs in self.dynamic_obstacles:
            if center.dist(obs) < radius + OBS_RADIUS:
                return True
        return False

    def add_dynamic_obstacle(self, pos: Vec2):
        self.dynamic_obstacles.append(Vec2(pos.x, pos.y))

    def remove_nearest_dynamic(self, pos: Vec2, threshold: float = 200.0):
        if not self.dynamic_obstacles:
            return
        closest = min(self.dynamic_obstacles, key=lambda o: o.dist(pos))
        if closest.dist(pos) < threshold:
            self.dynamic_obstacles.remove(closest)

    def to_list(self) -> list:
        cells = []
        for gx in range(GRID_W):
            for gy in range(GRID_H):
                if self._grid[gx][gy]:
                    cells.append({'gx': gx, 'gy': gy})
        return cells

    def to_hex_bytes(self) -> str:
        """Encode grid as hex string for telemetry (mirrors C++ Occupancy::compress)."""
        total_bits = GRID_W * GRID_H
        total_bytes = (total_bits + 7) // 8
        raw = bytearray(total_bytes)
        bit = 0
        for gx in range(GRID_W):
            for gy in range(GRID_H):
                if self._grid[gx][gy]:
                    raw[bit // 8] |= (1 << (bit % 8))
                bit += 1
        return raw.hex().upper()


class Pathfinder:
    """8-directional A* on the 150mm occupancy grid."""

    DIRS = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
    ]

    def __init__(self, occupancy: OccupancyGrid):
        self.occ = occupancy

    def find_path(self, start: Vec2, goal: Vec2) -> list[Vec2]:
        gs = self.occ.world_to_grid(start)
        gg = self.occ.world_to_grid(goal)
        if gs == gg:
            return [start, goal]
        if self.occ.is_cell_occupied(*gg):
            return [start, goal]

        g_score = {gs: 0.0}
        came_from: dict = {}
        heap = [(self._h(gs, gg), 0.0, gs)]

        while heap:
            _, g, current = heapq.heappop(heap)
            if current == gg:
                return self._reconstruct(came_from, current, start, goal)
            if g > g_score.get(current, float('inf')):
                continue
            for dx, dy, cost in self.DIRS:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if self.occ.is_cell_occupied(nx, ny):
                    continue
                ng = g + cost
                if ng < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = ng
                    came_from[neighbor] = current
                    heapq.heappush(heap, (ng + self._h(neighbor, gg), ng, neighbor))

        return [start, goal]

    def _h(self, a, b) -> float:
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def _reconstruct(self, came_from, current, start, goal):
        cells = []
        while current in came_from:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        path = [start] + [self.occ.grid_to_world(gx, gy) for gx, gy in cells] + [goal]
        return self._simplify(path)

    def _simplify(self, path):
        if len(path) <= 2:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            v1 = path[i]     - path[i - 1]
            v2 = path[i + 1] - path[i]
            cross = v1.x * v2.y - v1.y * v2.x
            if abs(cross) > 500.0:
                result.append(path[i])
        result.append(path[-1])
        return result


class GameObjects:
    """Colors assigned to named POIs (simulates TwinVision camera hub)."""

    MATCH_RADIUS = 80.0

    def __init__(self):
        self._colors: dict[str, ObjectColor] = {}
        for name, _ in POI.all_named():
            self._colors[name] = ObjectColor.UNKNOWN

    def set_color(self, poi_name: str, color: ObjectColor):
        self._colors[poi_name] = color

    def get_color(self, poi_name: str) -> ObjectColor:
        return self._colors.get(poi_name, ObjectColor.UNKNOWN)

    def query_color_at(self, pos: Vec2) -> tuple:
        best_name = None
        best_dist = float('inf')
        for name, poi_pos in POI.all_named():
            d = pos.dist(poi_pos)
            if d < self.MATCH_RADIUS and d < best_dist:
                best_dist = d
                best_name = name
        if best_name:
            return best_name, self._colors.get(best_name, ObjectColor.UNKNOWN)
        return None, ObjectColor.UNKNOWN

    def to_list(self) -> list:
        result = []
        for name, pos in POI.all_named():
            result.append({
                'name':  name,
                'x':     pos.x,
                'y':     pos.y,
                'color': self._colors.get(name, ObjectColor.UNKNOWN).name,
            })
        return result
