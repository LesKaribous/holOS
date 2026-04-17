"""
shared/occupancy.py — Unified two-layer occupancy grid.

Used by both the simulator (via sim/world.py) and real hardware (via services/occupancy.py).

Layer 0 — static:  known permanent obstacles, persisted to JSON, deployed to T40.
Layer 1 — dynamic: live LIDAR detections from T40, volatile (lost on disconnect).

Design:
  - Path planner (A*) considers both layers.
  - Safety firmware only reacts to dynamic obstacles (faster / lighter).
  - Static layer is painted manually in holOS UI and saved to strategy/static_occupancy.json.
"""

import json
import threading
from typing import List, Tuple

from shared.config import GRID_W, GRID_H, GRID_CELL, FIELD_W, FIELD_H, ROBOT_RADIUS, Vec2


class OccupancyGrid:
    """
    20×13 binary grid, 150 mm/cell, two independent layers.

    Thread-safe: all mutations and queries acquire self._lock.
    """

    def __init__(self):
        self._lock    = threading.Lock()
        self._static:  List[List[bool]] = [[False] * GRID_H for _ in range(GRID_W)]
        self._dynamic: List[List[bool]] = [[False] * GRID_H for _ in range(GRID_W)]

    # ── Queries ───────────────────────────────────────────────────────────────

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        """True if the cell is blocked (static OR dynamic layer)."""
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return True
        with self._lock:
            return self._static[gx][gy] or self._dynamic[gx][gy]

    def is_static_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return True
        with self._lock:
            return self._static[gx][gy]

    def is_dynamic_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
            return False
        with self._lock:
            return self._dynamic[gx][gy]

    def is_occupied_circle(self, center: Vec2, radius: float, dynamic_only: bool = False) -> bool:
        """True if a circle overlaps any occupied cell.
        dynamic_only=True is used by the safety service (only react to live obstacles)."""
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
                    occ = (self._dynamic[gx][gy] if dynamic_only
                           else (self._static[gx][gy] or self._dynamic[gx][gy]))
                    if occ:
                        cx = (gx + 0.5) * GRID_CELL
                        cy = (gy + 0.5) * GRID_CELL
                        dx = max(abs(center.x - cx) - GRID_CELL / 2, 0.0)
                        dy = max(abs(center.y - cy) - GRID_CELL / 2, 0.0)
                        if dx * dx + dy * dy < radius * radius:
                            return True
        return False

    def is_zone_occupied(self, center: Vec2, radius: float) -> bool:
        """Convenience alias (used by strategy code)."""
        return self.is_occupied_circle(center, radius)

    def is_occupied_world(self, pos: Vec2) -> bool:
        gx, gy = self.world_to_grid(pos)
        return self.is_cell_occupied(gx, gy)

    # ── Static layer ──────────────────────────────────────────────────────────

    def set_static_cell(self, gx: int, gy: int, value: bool) -> None:
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            with self._lock:
                self._static[gx][gy] = value

    def toggle_static_cell(self, gx: int, gy: int) -> None:
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            with self._lock:
                self._static[gx][gy] = not self._static[gx][gy]

    def reset_static(self) -> None:
        with self._lock:
            for col in self._static:
                for i in range(len(col)):
                    col[i] = False

    # ── Dynamic layer ─────────────────────────────────────────────────────────

    def set_dynamic_cells(self, cells: List[Tuple[int, int]],
                          inflate: int = 1) -> None:
        """Replace the entire dynamic layer with the given (gx, gy) active-cell list.
        Called by OccupancyService when a TEL:occ_dyn frame arrives.

        inflate — radius in cells to expand each detected point.
            The T40 LIDAR marks single cells (~150 mm) at the beacon centre,
            but adversary robots can be up to 300 mm in diameter.  inflate=1
            marks a 3×3 neighbourhood (±150 mm) so the pathfinder treats them
            as ~450 mm obstacles, giving our robot safe clearance.
        """
        with self._lock:
            for col in self._dynamic:
                for i in range(len(col)):
                    col[i] = False
            for gx, gy in cells:
                for dx in range(-inflate, inflate + 1):
                    for dy in range(-inflate, inflate + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                            self._dynamic[nx][ny] = True

    def reset_dynamic(self) -> None:
        with self._lock:
            for col in self._dynamic:
                for i in range(len(col)):
                    col[i] = False

    # ── Backward-compatibility shims (sim/world.py paint brush, etc.) ─────────

    def toggle_cell(self, gx: int, gy: int) -> None:
        """Legacy: toggles the static layer (brush paint in sim)."""
        self.toggle_static_cell(gx, gy)

    def set_cell(self, gx: int, gy: int, value: bool) -> None:
        """Legacy: sets the static layer."""
        self.set_static_cell(gx, gy, value)

    def reset(self) -> None:
        """Reset both layers (sim reset button)."""
        with self._lock:
            for col in self._static:
                for i in range(len(col)): col[i] = False
            for col in self._dynamic:
                for i in range(len(col)): col[i] = False

    # ── Coordinate conversions ────────────────────────────────────────────────

    def world_to_grid(self, pos: Vec2) -> Tuple[int, int]:
        return (int(pos.x // GRID_CELL), int(pos.y // GRID_CELL))

    def grid_to_world(self, gx: int, gy: int) -> Vec2:
        return Vec2((gx + 0.5) * GRID_CELL, (gy + 0.5) * GRID_CELL)

    # ── Persistence ───────────────────────────────────────────────────────────

    def save_static(self, path: str) -> None:
        """Persist the static layer to a JSON file."""
        cells = []
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._static[gx][gy]:
                        cells.append([gx, gy])
        with open(path, 'w') as f:
            json.dump({'version': 1, 'cells': cells}, f, indent=2)

    def load_static(self, path: str) -> bool:
        """Load static layer from a JSON file. Returns True on success."""
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            with self._lock:
                for col in self._static:
                    for i in range(len(col)): col[i] = False
                for cell in data.get('cells', []):
                    gx, gy = int(cell[0]), int(cell[1])
                    if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                        self._static[gx][gy] = True
            return True
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
            return False

    # ── Serialization for UI / telemetry ──────────────────────────────────────

    def to_list(self) -> list:
        """Return all occupied cells with their layer ('static' or 'dynamic').
        Used by _build_state() to push occupancy to the web UI."""
        cells = []
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._static[gx][gy] or self._dynamic[gx][gy]:
                        # If both layers are set, static takes visual precedence.
                        layer = 'static' if self._static[gx][gy] else 'dynamic'
                        cells.append({'gx': gx, 'gy': gy, 'layer': layer})
        return cells

    def static_to_list(self) -> list:
        cells = []
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._static[gx][gy]:
                        cells.append({'gx': gx, 'gy': gy})
        return cells

    def dynamic_to_list(self) -> list:
        cells = []
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._dynamic[gx][gy]:
                        cells.append({'gx': gx, 'gy': gy})
        return cells

    def to_static_hex(self) -> str:
        """Encode static layer as hex-packed bytes for deploying to T40.
        Bit order: gx outer loop, gy inner loop (LSB first). 33 bytes for 20×13."""
        total_bits  = GRID_W * GRID_H
        total_bytes = (total_bits + 7) // 8
        raw = bytearray(total_bytes)
        bit = 0
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._static[gx][gy]:
                        raw[bit // 8] |= (1 << (bit % 8))
                    bit += 1
        return raw.hex().upper()

    def to_hex_bytes(self) -> str:
        """Legacy: full (static+dynamic) bitmap as hex (for SimBridge telemetry)."""
        total_bits  = GRID_W * GRID_H
        total_bytes = (total_bits + 7) // 8
        raw = bytearray(total_bytes)
        bit = 0
        with self._lock:
            for gx in range(GRID_W):
                for gy in range(GRID_H):
                    if self._static[gx][gy] or self._dynamic[gx][gy]:
                        raw[bit // 8] |= (1 << (bit % 8))
                    bit += 1
        return raw.hex().upper()
