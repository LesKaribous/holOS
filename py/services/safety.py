"""
services/safety.py — SafetyService for Jetson brain.

Mirrors C++ Safety service. Tracks obstacle detection via Teensy telemetry.
"""

from transport.base import Transport


class SafetyService:

    def __init__(self, transport: Transport):
        self._t = transport
        self._enabled  = False
        self._detected = False
        self._t.subscribe_telemetry("safety", self._on_safety_tel)

    def _on_safety_tel(self, data: str) -> None:
        self._detected = (data.strip() == "1")

    def enable(self) -> None:
        self._enabled = True
        self._t.fire("enable(SAFETY)")

    def disable(self) -> None:
        self._enabled = False
        self._t.fire("disable(SAFETY)")

    @property
    def obstacle_detected(self) -> bool:
        return self._detected

    @property
    def enabled(self) -> bool:
        return self._enabled
