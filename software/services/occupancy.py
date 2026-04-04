"""
services/occupancy.py — OccupancyService for the Jetson brain.

Wraps the shared OccupancyGrid and wires it to hardware telemetry:
  - Subscribes to TEL:occ_dyn  (sparse dynamic cells from T40 via T41, new protocol)
  - Static layer is loaded from JSON at startup and deployed to T40 on demand.

The grid is passed in from outside (Brain creates it and shares it with MotionService
so both use the same map for path planning).
"""

from transport.base import Transport
from shared.occupancy import OccupancyGrid
from shared.config    import Vec2


class OccupancyService:

    def __init__(self, transport: Transport, grid: OccupancyGrid):
        self._t    = transport
        self._grid = grid
        self._t.subscribe_telemetry("occ_dyn", self._on_occ_dyn_tel)

    # ── Telemetry handler ─────────────────────────────────────────────────────

    def _on_occ_dyn_tel(self, data: str) -> None:
        """Decode sparse dynamic-cell list from T40.

        Protocol: 'gx,gy;gx,gy;...' — semicolon-separated pairs.
        Empty string means no dynamic obstacles.
        """
        try:
            cells = []
            data = data.strip()
            if data:
                for token in data.split(';'):
                    token = token.strip()
                    if not token:
                        continue
                    gx_s, gy_s = token.split(',')
                    cells.append((int(gx_s), int(gy_s)))
            self._grid.set_dynamic_cells(cells)
        except Exception as e:
            print(f"[OccupancyService] occ_dyn decode error: {e}")

    # ── Grid access (for strategy / pathfinder) ───────────────────────────────

    @property
    def grid(self) -> OccupancyGrid:
        return self._grid

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        return self._grid.is_cell_occupied(gx, gy)

    def is_occupied_circle(self, center: Vec2, radius: float) -> bool:
        return self._grid.is_occupied_circle(center, radius)

    def is_zone_occupied(self, center: Vec2, radius: float) -> bool:
        return self._grid.is_zone_occupied(center, radius)

    def world_to_grid(self, pos: Vec2) -> tuple:
        return self._grid.world_to_grid(pos)

    def grid_to_world(self, gx: int, gy: int) -> Vec2:
        return self._grid.grid_to_world(gx, gy)

    def to_list(self) -> list:
        return self._grid.to_list()

    # ── On-demand update request ──────────────────────────────────────────────

    def request_update(self) -> None:
        """Ask T41 to push an immediate occ_dyn update (fire-and-forget)."""
        self._t.fire("occ")
