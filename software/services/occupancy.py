"""
services/occupancy.py — OccupancyService for Jetson brain.

Receives occupancy map from Teensy via telemetry, provides A* path planning.
The map is a 20×13 binary grid (GRID_W × GRID_H) packed as hex bytes.
"""

import threading
from typing import Optional

from transport.base import Transport
from shared.config import Vec2, GRID_W, GRID_H, GRID_CELL, FIELD_W, FIELD_H, ROBOT_RADIUS


class OccupancyService:

    def __init__(self, transport: Transport):
        self._t    = transport
        self._lock = threading.Lock()
        self._grid: list[list[bool]] = [[False] * GRID_H for _ in range(GRID_W)]
        self._dynamic_obstacles: list[Vec2] = []
        self._t.subscribe_telemetry("occ", self._on_occ_tel)

    # ── Telemetry handler ─────────────────────────────────────────────────────

    def _on_occ_tel(self, data: str) -> None:
        """
        Decode hex-encoded occupancy map.
        Format: 'AABBCC...' (ceil(GRID_W*GRID_H/8) bytes as hex pairs).
        """
        try:
            raw = bytes.fromhex(data.strip())
            with self._lock:
                bit = 0
                for gx in range(GRID_W):
                    for gy in range(GRID_H):
                        byte_idx = bit // 8
                        bit_idx  = bit % 8
                        if byte_idx < len(raw):
                            self._grid[gx][gy] = bool((raw[byte_idx] >> bit_idx) & 1)
                        bit += 1
        except Exception as e:
            print(f"[OccupancyService] Map decode error: {e}")

    # ── Queries ───────────────────────────────────────────────────────────────

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return True
        with self._lock:
            return self._grid[gx][gy]

    def is_occupied_circle(self, center: Vec2, radius: float) -> bool:
        if (center.x - radius < 0 or center.x + radius > FIELD_W or
                center.y - radius < 0 or center.y + radius > FIELD_H):
            return True
        x0 = max(0, int((center.x - radius) // GRID_CELL))
        x1 = min(GRID_W - 1, int((center.x + radius) // GRID_CELL))
        y0 = max(0, int((center.y - radius) // GRID_CELL))
        y1 = min(GRID_H - 1, int((center.y + radius) // GRID_CELL))
        with self._lock:
            for gx in range(x0, x1 + 1):
                for gy in range(y0, y1 + 1):
                    if self._grid[gx][gy]:
                        cx = (gx + 0.5) * GRID_CELL
                        cy = (gy + 0.5) * GRID_CELL
                        dx = max(abs(center.x - cx) - GRID_CELL / 2, 0.0)
                        dy = max(abs(center.y - cy) - GRID_CELL / 2, 0.0)
                        if dx * dx + dy * dy < radius * radius:
                            return True
        return False

    def is_zone_occupied(self, center: Vec2, radius: float) -> bool:
        return self.is_occupied_circle(center, radius)

    def world_to_grid(self, pos: Vec2) -> tuple:
        return (int(pos.x // GRID_CELL), int(pos.y // GRID_CELL))

    def grid_to_world(self, gx: int, gy: int) -> Vec2:
        return Vec2((gx + 0.5) * GRID_CELL, (gy + 0.5) * GRID_CELL)

    def add_dynamic_obstacle(self, pos: Vec2) -> None:
        with self._lock:
            self._dynamic_obstacles.append(Vec2(pos.x, pos.y))

    def clear_dynamic_obstacles(self) -> None:
        with self._lock:
            self._dynamic_obstacles.clear()

    def request_update(self) -> None:
        """Ask Teensy to push an occupancy map update."""
        self._t.fire("occ")

    def to_list(self) -> list:
        cells = []
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._grid[gx][gy]:
                        cells.append({'gx': gx, 'gy': gy})
        return cells
