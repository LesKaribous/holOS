"""
services/chrono.py — ChronoService for Jetson brain.

Tracks match time via Teensy telemetry. The Teensy is the time source.
"""

from transport.base import Transport
from shared.config import MATCH_DURATION_S


class ChronoService:

    def __init__(self, transport: Transport):
        self._t        = transport
        self._elapsed  = 0.0
        self._running  = False
        self._t.subscribe_telemetry("chrono", self._on_chrono_tel)

    def _on_chrono_tel(self, data: str) -> None:
        """Parse '<elapsed_ms>' from Teensy."""
        try:
            self._elapsed = float(data) / 1000.0
        except ValueError:
            pass

    def start(self) -> None:
        """Tell Teensy to start the match timer."""
        self._running = True
        self._t.fire("start")

    def stop(self) -> None:
        self._running = False

    def time_left_s(self) -> float:
        return max(0.0, MATCH_DURATION_S - self._elapsed)

    def time_elapsed_s(self) -> float:
        return self._elapsed

    def is_running(self) -> bool:
        return self._running

    def is_finished(self) -> bool:
        return self._elapsed >= MATCH_DURATION_S
