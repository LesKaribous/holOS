"""
services/vision.py — VisionService for Jetson brain.

Queries the TwinVision camera hub via the Teensy (forwarded over XBee).
"""

import time
from transport.base import Transport
from shared.config import Vec2, ObjectColor, POI


class VisionService:

    QUERY_TIMEOUT_MS = 500

    def __init__(self, transport: Transport):
        self._t      = transport
        self._cache: dict[str, ObjectColor] = {}

    def query_color_sync(self, pos: Vec2, timeout_ms: int = 500) -> ObjectColor:
        """
        Blocking color query at a position.
        Sends 'vision(<x>,<y>)' to Teensy, waits for response.
        """
        cmd = f"vision({pos.x:.0f},{pos.y:.0f})"
        ok, resp = self._t.execute(cmd, timeout_ms=timeout_ms + 200)
        if not ok:
            return ObjectColor.UNKNOWN
        return _parse_color(resp)

    def get_color(self, pos: Vec2) -> ObjectColor:
        """Cached (non-blocking) color lookup."""
        name = _nearest_poi(pos)
        return self._cache.get(name, ObjectColor.UNKNOWN)

    def set_color(self, poi_name: str, color: ObjectColor) -> None:
        """Override color (used by sim)."""
        self._cache[poi_name] = color


def _parse_color(resp: str) -> ObjectColor:
    try:
        return ObjectColor[resp.strip().upper()]
    except KeyError:
        return ObjectColor.UNKNOWN


def _nearest_poi(pos: Vec2) -> str:
    best_name = None
    best_dist = float('inf')
    for name, poi_pos in POI.all_named():
        d = pos.dist(poi_pos)
        if d < best_dist:
            best_dist = d
            best_name = name
    return best_name or ""
