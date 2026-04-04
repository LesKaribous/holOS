"""
sim/world.py — OccupancyGrid, Pathfinder and GameObjects for the simulator.

OccupancyGrid and Pathfinder are now defined in shared/ and used identically
by the simulator and the real-hardware Jetson path.
This file re-exports them so existing sim imports keep working unchanged.
"""

from shared.config     import Vec2, ObjectColor, POI
from shared.occupancy  import OccupancyGrid   # noqa: F401 — re-exported
from shared.pathfinder import Pathfinder       # noqa: F401 — re-exported

__all__ = ['OccupancyGrid', 'Pathfinder', 'GameObjects']


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
