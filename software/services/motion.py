"""
services/motion.py — MotionService for Jetson brain.

API mirrors the C++ Motion class and the existing sim MotionService so that
strategy.py code is 100% identical in both real and sim modes.

On real hardware:  commands go via XBeeTransport → Teensy.
In sim mode:       commands go via VirtualTransport → SimBridge → physics.

In both cases the API is blocking from the strategy thread's perspective.
"""

import math
from typing import Optional, List

from transport.base import Transport
from shared.config import (
    Vec2, RobotCompass, robot_compass_offset_deg, polar_vec,
)


class MotionService:
    """
    High-level motion service.

    Usage in strategy.py (blocking):
        motion.go(500, 300)
        motion.go_align(POI.stock_1, RobotCompass.AB, 270)
        motion.cancel_on_collide().feedrate(0.8).go(500, 300)
        motion.via(200, 0).via(200, 300).go(500, 300)
        motion.turn(90)
        motion.was_successful()

    Fluent options reset after each move command.
    """

    def __init__(self, transport: Transport):
        self._t = transport

        # Current position (updated via telemetry)
        self._pos   = Vec2(0, 0)
        self._theta = 0.0

        # Global feedrate
        self._feedrate = 1.0

        # Per-move pending options
        self._pending_cancel_on_collide = False
        self._pending_no_collide        = False
        self._pending_feedrate          = -1.0
        self._pending_via: List[Vec2]   = []

        # Result of last move
        self._last_ok = True

        # Subscribe to position telemetry
        self._t.subscribe_telemetry("pos", self._on_pos_tel)

    # ── Telemetry handler ─────────────────────────────────────────────────────

    def _on_pos_tel(self, data: str) -> None:
        """Parse 'x=<v>,y=<v>,theta=<v>' from Teensy telemetry."""
        try:
            parts = dict(kv.split('=') for kv in data.split(','))
            self._pos   = Vec2(float(parts.get('x', 0)), float(parts.get('y', 0)))
            self._theta = float(parts.get('theta', 0))
        except Exception:
            pass

    # ── Fluent option setters ─────────────────────────────────────────────────

    def no_collide(self) -> 'MotionService':
        self._pending_no_collide = True
        return self

    def with_collision(self, on: bool = True) -> 'MotionService':
        self._pending_no_collide = not on
        return self

    def cancel_on_collide(self, on: bool = True) -> 'MotionService':
        self._pending_cancel_on_collide = on
        return self

    def feedrate(self, f: float) -> 'MotionService':
        self._pending_feedrate = max(0.05, min(1.0, f))
        return self

    def via(self, x_or_vec, y: float = None) -> 'MotionService':
        if isinstance(x_or_vec, Vec2):
            self._pending_via.append(x_or_vec)
        else:
            self._pending_via.append(Vec2(x_or_vec, y))
        return self

    # ── Movement commands (blocking) ─────────────────────────────────────────

    def go(self, x_or_vec, y: float = None) -> 'MotionService':
        if isinstance(x_or_vec, Vec2):
            target = x_or_vec
        else:
            target = Vec2(x_or_vec, y)
        self._execute_go(target)
        return self

    def go_align(self, target: Vec2, rc: RobotCompass, orientation_deg: float) -> 'MotionService':
        theta = orientation_deg - robot_compass_offset_deg(rc)
        self._execute_go_align(target, theta)
        return self

    def go_polar(self, heading_deg: float, dist: float) -> 'MotionService':
        target = self._pos + polar_vec(heading_deg, dist)
        self._execute_go(target)
        return self

    def go_polar_align(self, heading_deg: float, dist: float,
                       rc: RobotCompass, orientation_deg: float) -> 'MotionService':
        target = self._pos + polar_vec(heading_deg, dist)
        theta  = orientation_deg - robot_compass_offset_deg(rc)
        self._execute_go_align(target, theta)
        return self

    def turn(self, angle_deg: float) -> 'MotionService':
        cmd = self._build_turn_cmd(angle_deg)
        self._last_ok, _ = self._t.execute(cmd, timeout_ms=15000)
        self._reset_pending()
        return self

    def align(self, rc: RobotCompass, orientation_deg: float) -> 'MotionService':
        cmd = f"align({rc.name},{orientation_deg:.1f})"
        self._last_ok, _ = self._t.execute(cmd, timeout_ms=15000)
        self._reset_pending()
        return self

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def cancel(self) -> None:
        self._t.fire("cancel")

    def pause(self) -> None:
        self._t.fire("pause")

    def resume(self) -> None:
        self._t.fire("resume")

    # ── State queries ─────────────────────────────────────────────────────────

    def was_successful(self) -> bool:
        return self._last_ok

    def position(self) -> Vec2:
        return Vec2(self._pos.x, self._pos.y)

    def theta(self) -> float:
        return self._theta

    # ── Global settings ───────────────────────────────────────────────────────

    def set_feedrate(self, f: float) -> None:
        self._feedrate = max(0.05, min(1.0, f))
        self._t.fire(f"feed({self._feedrate:.3f})")

    def get_feedrate(self) -> float:
        return self._feedrate

    def set_abs_position(self, x: float, y: float, angle_deg: float) -> None:
        self._t.fire(f"setAbsPosition({x:.1f},{y:.1f},{angle_deg:.2f})")

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _execute_go(self, target: Vec2) -> None:
        parts = []
        # via points
        for v in self._pending_via:
            parts.append(f"via({v.x:.1f},{v.y:.1f})")
        # Use go_coc (cancel-on-collide variant) when the flag is set
        if self._pending_cancel_on_collide:
            parts.append(f"go_coc({target.x:.1f},{target.y:.1f})")
        else:
            parts.append(f"go({target.x:.1f},{target.y:.1f})")
        cmd = ";".join(parts)
        # feedrate option
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._pending_feedrate:.3f})")
        self._last_ok, _ = self._t.execute(cmd, timeout_ms=60000)
        # Restore feedrate
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._feedrate:.3f})")
        self._reset_pending()

    def _execute_go_align(self, target: Vec2, theta_deg: float) -> None:
        cmd = f"goAlign({target.x:.1f},{target.y:.1f},{theta_deg:.2f})"
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._pending_feedrate:.3f})")
        self._last_ok, _ = self._t.execute(cmd, timeout_ms=60000)
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._feedrate:.3f})")
        self._reset_pending()

    def _build_turn_cmd(self, angle_deg: float) -> str:
        return f"turn({angle_deg:.2f})"

    def _reset_pending(self) -> None:
        self._pending_cancel_on_collide = False
        self._pending_no_collide        = False
        self._pending_feedrate          = -1.0
        self._pending_via               = []
