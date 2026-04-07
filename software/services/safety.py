"""
services/safety.py — SafetyService for Jetson brain.

Receives TEL:safety telemetry from T41 (sourced from T40 LIDAR).
Tracks obstacle state with timestamps and exposes callbacks so the
MotionService can react immediately (pause, replan, cancel).

Timeline (firmware side, for reference):
  - T41 sends ob(angle, dist) to T40 every 100 ms while moving
  - T40 replies 0/1 (200-300 ms latency)
  - T41 pauses motion on detection, resumes after 1000 ms silence
  - T41 pushes TEL:safety:0|1 at 10 Hz to Jetson

This service runs on the Jetson side and adds:
  - Rising/falling edge detection (obstacle_appeared / obstacle_cleared)
  - Duration tracking (how long has the obstacle been present?)
  - Callbacks for MotionService to hook into
"""

import time
import threading
from typing import Callable, List, Optional

from transport.base import Transport


class SafetyService:

    def __init__(self, transport: Transport):
        self._t = transport
        self._enabled  = False
        self._detected = False

        # Timing
        self._detected_since: float = 0.0   # monotonic timestamp of last rising edge
        self._cleared_since: float  = 0.0   # monotonic timestamp of last falling edge

        # Callbacks — called from the reader thread (telemetry handler)
        self._cb_lock = threading.Lock()
        self._on_obstacle: List[Callable[[], None]]  = []
        self._on_clear:    List[Callable[[], None]]  = []

        self._t.subscribe_telemetry("safety", self._on_safety_tel)

    # ── Telemetry handler (called from reader thread) ────────────────────────

    def _on_safety_tel(self, data: str) -> None:
        now = time.monotonic()
        new_state = (data.strip() == "1")

        if new_state and not self._detected:
            # Rising edge — obstacle appeared
            self._detected = True
            self._detected_since = now
            print("[Safety] ⚠ Obstacle DETECTED")
            with self._cb_lock:
                cbs = list(self._on_obstacle)
            for cb in cbs:
                try:
                    cb()
                except Exception as e:
                    print(f"[Safety] on_obstacle callback error: {e}")

        elif not new_state and self._detected:
            # Falling edge — obstacle cleared
            duration = now - self._detected_since
            self._detected = False
            self._cleared_since = now
            print(f"[Safety] Obstacle CLEARED (was present {duration:.1f}s)")
            with self._cb_lock:
                cbs = list(self._on_clear)
            for cb in cbs:
                try:
                    cb()
                except Exception as e:
                    print(f"[Safety] on_clear callback error: {e}")

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def obstacle_detected(self) -> bool:
        return self._detected

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def obstacle_duration(self) -> float:
        """Seconds since last rising edge (0.0 if no obstacle)."""
        if not self._detected:
            return 0.0
        return time.monotonic() - self._detected_since

    @property
    def clear_duration(self) -> float:
        """Seconds since last falling edge (0.0 if obstacle present)."""
        if self._detected:
            return 0.0
        if self._cleared_since == 0.0:
            return float('inf')  # never had an obstacle
        return time.monotonic() - self._cleared_since

    def on_obstacle(self, callback: Callable[[], None]) -> None:
        """Register a callback fired on rising edge (obstacle appeared)."""
        with self._cb_lock:
            self._on_obstacle.append(callback)

    def on_clear(self, callback: Callable[[], None]) -> None:
        """Register a callback fired on falling edge (obstacle cleared)."""
        with self._cb_lock:
            self._on_clear.append(callback)

    # ── Firmware commands ────────────────────────────────────────────────────

    def enable(self) -> None:
        self._enabled = True
        self._t.fire("enable(SAFETY)")
        print("[Safety] Enabled")

    def disable(self) -> None:
        self._enabled = False
        self._t.fire("disable(SAFETY)")
        print("[Safety] Disabled")
