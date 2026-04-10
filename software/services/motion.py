"""
services/motion.py — MotionService for the Jetson brain.

API mirrors the C++ Motion class and the sim MotionService so strategy.py
code is 100% identical in both real and sim modes.

On real hardware:  commands go via XBeeTransport → Teensy.
In sim mode:       commands go via VirtualTransport → SimBridge → physics.

Path planning (A*):
  When a Pathfinder is provided AND use_pathfinding is True, every go() call
  computes a path and sends it as a chained via()/go() command so the firmware
  executes the full path without intermediate stops (pass-through waypoints).

  motion.use_pathfinding = False  → direct go() commands (no A*)
  motion.use_pathfinding = True   → A* + via() chaining (default when pathfinder present)

Reactive replanning:
  When a motion command fails with motion_timeout or stall (obstacle-related),
  the service cancels the current motion, replans A* with the updated dynamic
  occupancy grid, and re-sends the command.  This only happens when
  use_pathfinding is True and a SafetyService is attached.

Angle offset:
  theta_offset_deg (default 0 in sim, HW_THETA_OFFSET_DEG on hardware) converts
  between holOS frame (0=East) and the firmware frame (0=face A).
"""

import math
import time
from typing import Optional, List

from transport.base import Transport
from shared.config import (
    Vec2, RobotCompass, robot_compass_offset_deg, polar_vec,
)


# ── Replanning constants ─────────────────────────────────────────────────────

# How long to wait (seconds) after firmware pauses before attempting a replan.
# This gives the opponent time to move away before we burn time replanning.
REPLAN_DELAY_S   = 1.5

# Maximum number of replan attempts per single go() call.
MAX_REPLANS      = 3

# Minimum distance (mm) to target before we bother replanning.  If we're
# almost there, just wait for the obstacle to clear.
REPLAN_MIN_DIST  = 200.0


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

    def __init__(self, transport: Transport,
                 theta_offset_deg: float = 0.0,
                 pathfinder=None,
                 safety=None):
        self._t = transport

        # Angle offset between holOS frame and firmware frame (degrees / radians).
        self._theta_offset_deg = theta_offset_deg
        self._theta_offset_rad = math.radians(theta_offset_deg)

        # Path planner (shared.pathfinder.Pathfinder, or None for sim/no pathfinding).
        self._pathfinder = pathfinder

        # SafetyService reference for reactive replanning
        self._safety = safety

        # Toggle: when False, go() sends direct commands even if a pathfinder is present.
        self.use_pathfinding: bool = pathfinder is not None

        # Toggle: when True and pathfinding is on, replan around obstacles
        self.use_replanning: bool = True

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
        """Parse 'x=<v>,y=<v>,theta=<v>' from Teensy telemetry.
        theta from firmware is in radians (firmware frame); add offset → holOS frame."""
        try:
            parts = dict(kv.split('=') for kv in data.split(','))
            self._pos   = Vec2(float(parts.get('x', 0)), float(parts.get('y', 0)))
            self._theta = float(parts.get('theta', 0)) + self._theta_offset_rad
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

    def go_heading(self, x_or_vec, y: float = None,
                   theta_deg: float = 0.0) -> 'MotionService':
        """Move to (x, y) and set final heading to theta_deg (table frame, 0=East).
        Unlike go_align(), this does not require a RobotCompass side — the angle
        is the raw robot heading. Use this for map-click "go with rotation" commands."""
        target = x_or_vec if isinstance(x_or_vec, Vec2) else Vec2(x_or_vec, y)
        self._execute_go_align(target, theta_deg)
        return self

    def go_polar_align(self, heading_deg: float, dist: float,
                       rc: RobotCompass, orientation_deg: float) -> 'MotionService':
        target = self._pos + polar_vec(heading_deg, dist)
        theta  = orientation_deg - robot_compass_offset_deg(rc)
        self._execute_go_align(target, theta)
        return self

    def turn(self, angle_deg: float) -> 'MotionService':
        cmd = self._build_turn_cmd(angle_deg)
        self._last_ok, resp = self._t.execute(cmd, timeout_ms=15000)
        if not self._last_ok:
            print(f"[Motion] turn({angle_deg:.1f}°) FAILED — {resp}")
        self._reset_pending()
        return self

    def align(self, rc: RobotCompass, orientation_deg: float) -> 'MotionService':
        cmd = f"align({rc.name},{orientation_deg - self._theta_offset_deg:.1f})"
        self._last_ok, resp = self._t.execute(cmd, timeout_ms=15000)
        if not self._last_ok:
            print(f"[Motion] align({rc.name},{orientation_deg:.1f}°) FAILED — {resp}")
        self._reset_pending()
        return self

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def cancel(self) -> None:
        self._t.fire("cancel")

    def pause(self) -> None:
        self._t.fire("pause")

    def resume(self) -> None:
        self._t.fire("resume")

    def enable_apf(self, scale: float = 50000.0) -> None:
        """Enable Artificial Potential Fields on firmware (cruise mode only)."""
        self._t.fire(f"apf(on,{scale:.0f})")

    def disable_apf(self) -> None:
        self._t.fire("apf(off)")

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
        self._t.fire(f"setAbsPosition({x:.1f},{y:.1f},{angle_deg - self._theta_offset_deg:.2f})")

    # ── Internal helpers ──────────────────────────────────────────────────────

    # Max via points before the chained command exceeds firmware's CONTENT_MAX
    # (384 bytes). Each "via(XXXX.X,XXXX.X);" is ~22 chars; 14 via = ~308 chars
    # + go() = ~330 chars — leaves room for the uid:...|crc framing.
    MAX_VIA_POINTS = 14

    def _build_go_cmd(self, target: Vec2, extra_via: List[Vec2] = None) -> str:
        """Build a go command string with optional A* via points."""
        path_via: List[Vec2] = []
        if self.use_pathfinding and self._pathfinder is not None:
            try:
                path = self._pathfinder.find_path(self._pos, target)
                path_via = path[1:-1]
            except Exception as e:
                print(f"[MotionService] Pathfinding error, using direct path: {e}")
                path_via = []

        parts = []
        # Caller-specified via points (strategy-level override) come first
        if extra_via:
            for v in extra_via:
                parts.append(f"via({v.x:.1f},{v.y:.1f})")

        # A* intermediate waypoints as pass-through via points
        for v in path_via:
            parts.append(f"via({v.x:.1f},{v.y:.1f})")

        # Safety cap: if too many via points, simplify by keeping only a
        # subset (evenly spaced) to stay within firmware buffer limits.
        caller_via_count = len(extra_via) if extra_via else 0
        max_via = self.MAX_VIA_POINTS - caller_via_count
        if len(path_via) > max_via and max_via > 0:
            step = len(path_via) / max_via
            simplified = [path_via[int(i * step)] for i in range(max_via)]
            parts = []
            if extra_via:
                for v in extra_via:
                    parts.append(f"via({v.x:.1f},{v.y:.1f})")
            for v in simplified:
                parts.append(f"via({v.x:.1f},{v.y:.1f})")

        if self._pending_cancel_on_collide:
            parts.append(f"go_coc({target.x:.1f},{target.y:.1f})")
        else:
            parts.append(f"go({target.x:.1f},{target.y:.1f})")

        return ";".join(parts)

    def _execute_go(self, target: Vec2) -> None:
        """Build and send a go command, with optional replanning on obstacle."""
        # Snapshot caller via points (consumed on first send)
        caller_via = list(self._pending_via)
        replan_count = 0
        can_replan = (self.use_pathfinding and self.use_replanning
                      and self._pathfinder is not None
                      and self._safety is not None
                      and not self._pending_cancel_on_collide)

        while True:
            cmd = self._build_go_cmd(target, caller_via if replan_count == 0 else None)

            if self._pending_feedrate > 0:
                self._t.fire(f"feed({self._pending_feedrate:.3f})")

            ok, resp = self._t.execute(cmd, timeout_ms=60000)

            if self._pending_feedrate > 0 and replan_count == 0:
                # Restore global feedrate only once (not on replans)
                self._t.fire(f"feed({self._feedrate:.3f})")

            # ── Success or non-retriable failure ─────────────────────────
            if ok:
                self._last_ok = True
                if replan_count > 0:
                    print(f"[Motion] go({target.x:.0f},{target.y:.0f}) OK after "
                          f"{replan_count} replan(s)")
                break

            # Motion failed — check if we should try replanning
            if not can_replan or replan_count >= MAX_REPLANS:
                reason = f"resp={resp}"
                if not can_replan:
                    reason = "replanning disabled"
                elif replan_count >= MAX_REPLANS:
                    reason = f"max replans reached ({MAX_REPLANS})"
                print(f"[Motion] go({target.x:.0f},{target.y:.0f}) FAILED — {reason}")
                self._last_ok = False
                break

            # Only replan on motion_timeout or stall (obstacle-related failures)
            if resp not in ("motion_timeout", "stall"):
                print(f"[Motion] go({target.x:.0f},{target.y:.0f}) FAILED — {resp} "
                      f"(not retriable)")
                self._last_ok = False
                break

            # Distance check — don't replan if we're almost at the target
            dist = self._pos.dist(target)
            if dist < REPLAN_MIN_DIST:
                print(f"[Motion] go({target.x:.0f},{target.y:.0f}) FAILED — {resp}, "
                      f"too close to replan ({dist:.0f}mm < {REPLAN_MIN_DIST:.0f}mm)")
                self._last_ok = False
                break

            # ── Replan attempt ───────────────────────────────────────────
            replan_count += 1
            print(f"[MotionService] Replan attempt {replan_count}/{MAX_REPLANS}"
                  f" — {resp}, dist={dist:.0f}mm")

            # Brief pause to let the occupancy grid update with latest LIDAR data
            time.sleep(0.3)

            # Cancel any residual firmware motion state
            self._t.fire("cancel")
            self._t.fire("ack_done")
            time.sleep(0.1)

            # Loop back → _build_go_cmd will re-run A* with updated occupancy

        self._reset_pending()

    def _execute_go_align(self, target: Vec2, theta_deg: float) -> None:
        cmd = f"goAlign({target.x:.1f},{target.y:.1f},{theta_deg - self._theta_offset_deg:.2f})"
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._pending_feedrate:.3f})")
        self._last_ok, resp = self._t.execute(cmd, timeout_ms=60000)
        if not self._last_ok:
            print(f"[Motion] goAlign({target.x:.0f},{target.y:.0f},{theta_deg:.1f}°) "
                  f"FAILED — {resp}")
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._feedrate:.3f})")
        self._reset_pending()

    def _build_turn_cmd(self, angle_deg: float) -> str:
        # turn() is relative — no theta offset needed (unlike align which is absolute)
        return f"turn({angle_deg:.2f})"

    def _reset_pending(self) -> None:
        self._pending_cancel_on_collide = False
        self._pending_no_collide        = False
        self._pending_feedrate          = -1.0
        self._pending_via               = []
