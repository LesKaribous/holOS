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

import enum
import math
import time
from typing import Optional, List, Tuple

from transport.base import Transport
from shared.config import (
    Vec2, RobotCompass, robot_compass_offset_deg, polar_vec,
)


# ── Motion control modes ─────────────────────────────────────────────────────

class MotionMode(enum.Enum):
    """Mode de pilotage du firmware Motion.

    LEGACY_WAYPOINT : queue de waypoints (via;via;go), 1 commande par
                      _execute_go. Comportement historique, robuste mais
                      saccadé sur les chemins longs (le contrôleur reset
                      à chaque waypoint).

    LIVE_PURSUIT    : suivi de cible vivant. Le service Python envoie
                      `aim(x,y)` à ~20 Hz vers une « carotte » qui glisse
                      sur le chemin A*, replanifie en arrière-plan, et
                      verrouille la carotte sur la cible finale à l'approche.
                      Plus fluide et permet le replanning dynamique mais
                      n'est pas compatible avec les options de move
                      classiques (cancel_on_collide, goAlign).
    """
    LEGACY_WAYPOINT = 0
    LIVE_PURSUIT    = 1


# Pursuit-loop tuning constants
PURSUIT_TICK_S        = 0.05    # 20 Hz aim updates (must be < firmware watchdog 200ms)
PURSUIT_REPLAN_EVERY  = 10      # replan path every N ticks (~500ms)
PURSUIT_CARROT_DIST   = 300.0   # carrot lead distance (mm)
PURSUIT_LOCK_FACTOR   = 1.2     # lock carrot to final target when dist < CARROT*FACTOR
PURSUIT_ARRIVED_DIST  = 30.0    # arrival threshold (mm)
PURSUIT_TIMEOUT_S     = 30.0    # safety: max pursuit duration


# ── Replanning constants ─────────────────────────────────────────────────────

# How long to wait (seconds) after firmware pauses before attempting a replan.
# This gives the opponent time to move away before we burn time replanning.
REPLAN_DELAY_S   = 1.5

# Maximum number of replan attempts per single go() call.
MAX_REPLANS      = 3

# Minimum distance (mm) to target before we bother replanning.  If we're
# almost there, just wait for the obstacle to clear.
REPLAN_MIN_DIST  = 200.0

# Pathfinding-failure retry policy. When A* finds no path (start blocked,
# goal blocked, or fully enclosed), we wait NO_PATH_WAIT_S between attempts
# and re-run the search up to NO_PATH_MAX_RETRIES times. The grid may
# improve between attempts (LIDAR refresh, opponent moving, etc.) so a few
# patient retries are far better than blindly driving through obstacles.
NO_PATH_MAX_RETRIES = 5
NO_PATH_WAIT_S      = 0.4


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

        # Control mode (waypoint vs pursuit). The active firmware mode is
        # tracked separately so we can revert it after each pursuit move.
        self._mode: MotionMode          = MotionMode.LEGACY_WAYPOINT
        self._fw_mode: MotionMode       = MotionMode.LEGACY_WAYPOINT
        # When True, pursuit mode auto-reverts to LEGACY_WAYPOINT after each
        # _execute_go. Lets the user "arm" pursuit for one move at a time
        # from the UI.
        self.pursuit_auto_revert: bool  = True

        # Subscribe to position telemetry — firmware emits compact "T:p
        # <x_mm> <y_mm> <theta_mrad>"; keep the legacy "pos" channel too so
        # older firmwares / sim bridges still work.
        self._t.subscribe_telemetry("p",   self._on_pos_tel)
        self._t.subscribe_telemetry("pos", self._on_pos_tel)

    # ── Telemetry handler ─────────────────────────────────────────────────────

    def _on_pos_tel(self, data: str) -> None:
        """Update cached position from position telemetry.

        Supports both formats:
          * compact  — "<x_mm> <y_mm> <theta_mrad>"    (firmware T:p)
          * legacy   — "x=<mm>,y=<mm>,theta=<rad>"     (older firmware T:pos)

        theta is stored in the holOS frame (firmware frame + offset).
        """
        try:
            data = data.strip()
            if '=' in data:
                # Legacy key=value format
                parts = dict(kv.split('=') for kv in data.split(','))
                self._pos   = Vec2(float(parts.get('x', 0)),
                                   float(parts.get('y', 0)))
                self._theta = (float(parts.get('theta', 0))
                               + self._theta_offset_rad)
            else:
                # Compact space-separated format — theta is in mrad
                toks = data.split()
                if len(toks) >= 3:
                    self._pos   = Vec2(float(toks[0]), float(toks[1]))
                    self._theta = (float(toks[2]) / 1000.0
                                   + self._theta_offset_rad)
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

    # ── Motion mode (waypoint vs pursuit) ────────────────────────────────────

    def set_mode(self, mode: MotionMode) -> None:
        """Switch the firmware control mode and remember the user-armed mode.

        In LIVE_PURSUIT mode the next ``go()`` call drives the robot via the
        pursuit loop instead of the chained-via fast path. If
        ``pursuit_auto_revert`` is True (default), the mode flips back to
        LEGACY_WAYPOINT after the move so each pursuit run is opt-in.
        """
        self._mode = mode
        self._sync_fw_mode(mode)

    def get_mode(self) -> MotionMode:
        return self._mode

    def _sync_fw_mode(self, mode: MotionMode) -> None:
        """Send motion_mode(0|1) to firmware only when it actually changes."""
        if mode == self._fw_mode:
            return
        self._t.fire(f"motion_mode({1 if mode == MotionMode.LIVE_PURSUIT else 0})")
        self._fw_mode = mode

    # ── Internal helpers ──────────────────────────────────────────────────────

    # Max via points before the chained command exceeds firmware's CONTENT_MAX
    # (384 bytes). Each "via(XXXX.X,XXXX.X);" is ~22 chars; 14 via = ~308 chars
    # + go() = ~330 chars — leaves room for the uid:...|crc framing.
    MAX_VIA_POINTS = 14

    def _plan_path(self, target: Vec2) -> Tuple[List[Vec2], bool]:
        """Run the pathfinder once. Returns ``(via_points, found)``.

        ``found`` is True only when A* returned a real path. When the
        pathfinder is disabled, the start and goal are the same cell, or no
        pathfinder is attached, ``found`` is also True (no planning needed).
        On a NO PATH fallback ``found`` is False — callers must NOT execute
        the empty via list as a direct move; they should wait and retry.
        """
        if not (self.use_pathfinding and self._pathfinder is not None):
            return ([], True)
        try:
            path = self._pathfinder.find_path(self._pos, target)
        except Exception as e:
            print(f"[MotionService] Pathfinding error: {e}")
            return ([], False)
        found = not getattr(self._pathfinder, 'last_search_failed', False)
        return (path[1:-1], found)

    def _build_via_chain(self, target: Vec2,
                         extra_via: List[Vec2] = None,
                         path_via:  List[Vec2] = None) -> List[str]:
        """Build the list of via() command tokens leading up to `target`.

        If ``path_via`` is None, the A* pathfinder is run; otherwise the
        caller-supplied list is used (typically because the caller already
        ran the planner and validated the result).
        """
        if path_via is None:
            path_via, _ = self._plan_path(target)

        # Safety cap: if too many via points, keep an evenly-spaced subset
        # to stay within firmware buffer limits (MAX_VIA_POINTS).
        caller_via_count = len(extra_via) if extra_via else 0
        max_via = self.MAX_VIA_POINTS - caller_via_count
        if len(path_via) > max_via and max_via > 0:
            step = len(path_via) / max_via
            path_via = [path_via[int(i * step)] for i in range(max_via)]

        parts: List[str] = []
        if extra_via:
            for v in extra_via:
                parts.append(f"via({v.x:.1f},{v.y:.1f})")
        for v in path_via:
            parts.append(f"via({v.x:.1f},{v.y:.1f})")
        return parts

    def _build_go_cmd(self, target: Vec2,
                      extra_via: List[Vec2] = None,
                      path_via:  List[Vec2] = None) -> str:
        """Build a go command string with optional A* via points."""
        parts = self._build_via_chain(target, extra_via, path_via)
        if self._pending_cancel_on_collide:
            parts.append(f"go_coc({target.x:.1f},{target.y:.1f})")
        else:
            parts.append(f"go({target.x:.1f},{target.y:.1f})")
        return ";".join(parts)

    def _wait_for_path(self, target: Vec2) -> Optional[List[Vec2]]:
        """Run the planner with a wait+retry loop on NO PATH.

        Returns the via list (possibly empty if start and goal are in the
        same cell) on success, or None if every retry failed. The robot is
        NOT moved while we wait — we simply give the occupancy grid a few
        cycles to refresh in case a transient obstacle (opponent, LIDAR
        glitch) cleared the route.
        """
        if not (self.use_pathfinding and self._pathfinder is not None):
            return []
        for attempt in range(NO_PATH_MAX_RETRIES + 1):
            via, found = self._plan_path(target)
            if found:
                if attempt > 0:
                    print(f"[MotionService] Path acquired on attempt "
                          f"{attempt + 1}/{NO_PATH_MAX_RETRIES + 1}")
                return via
            if attempt < NO_PATH_MAX_RETRIES:
                print(f"[MotionService] NO PATH to ({target.x:.0f},"
                      f"{target.y:.0f}) — retry "
                      f"{attempt + 1}/{NO_PATH_MAX_RETRIES} "
                      f"in {NO_PATH_WAIT_S:.1f}s")
                time.sleep(NO_PATH_WAIT_S)
        print(f"[MotionService] NO PATH to ({target.x:.0f},{target.y:.0f})"
              f" after {NO_PATH_MAX_RETRIES} retries — aborting move")
        return None

    def _execute_go(self, target: Vec2) -> None:
        """Dispatch a go command according to the active control mode."""
        if self._mode == MotionMode.LIVE_PURSUIT:
            try:
                self._execute_go_pursuit(target)
            finally:
                # Auto-revert: pursuit is opt-in per move by default
                if self.pursuit_auto_revert:
                    self._mode = MotionMode.LEGACY_WAYPOINT
                    self._sync_fw_mode(MotionMode.LEGACY_WAYPOINT)
            return
        self._execute_go_waypoint(target)

    def _execute_go_waypoint(self, target: Vec2) -> None:
        """Legacy via;via;go path — chained waypoints with replanning on stall."""
        # Make sure firmware is in waypoint mode (in case a previous pursuit
        # crashed before the auto-revert ran).
        self._sync_fw_mode(MotionMode.LEGACY_WAYPOINT)

        # Snapshot caller via points (consumed on first send)
        caller_via = list(self._pending_via)
        replan_count = 0
        can_replan = (self.use_pathfinding and self.use_replanning
                      and self._pathfinder is not None
                      and self._safety is not None
                      and not self._pending_cancel_on_collide)

        while True:
            # Plan a path before each (re)send. wait_for_path retries on
            # NO PATH so we never silently fall back to a straight line
            # through obstacles.
            path_via = self._wait_for_path(target)
            if path_via is None:
                print(f"[Motion] go({target.x:.0f},{target.y:.0f}) FAILED — "
                      f"no path available")
                self._last_ok = False
                self._reset_pending()
                return

            cmd = self._build_go_cmd(
                target,
                caller_via if replan_count == 0 else None,
                path_via=path_via,
            )

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
        # Chain A* via() points ahead of the final goAlign() so that oriented
        # moves also avoid static/dynamic obstacles. Without this, the map
        # popup (which issues a heading when the pin angle is changed) would
        # drive the robot in a straight line through blue obstacle cells.
        caller_via = list(self._pending_via)
        path_via = self._wait_for_path(target)
        if path_via is None:
            print(f"[Motion] goAlign({target.x:.0f},{target.y:.0f},"
                  f"{theta_deg:.1f}°) FAILED — no path available")
            self._last_ok = False
            self._reset_pending()
            return

        parts = self._build_via_chain(target, caller_via, path_via=path_via)
        parts.append(
            f"goAlign({target.x:.1f},{target.y:.1f},"
            f"{theta_deg - self._theta_offset_deg:.2f})"
        )
        cmd = ";".join(parts)

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

    # ── Pursuit (LIVE_PURSUIT) implementation ─────────────────────────────────

    @staticmethod
    def _carrot_along_path(path: List[Vec2], pos: Vec2, lead: float) -> Vec2:
        """Project ``pos`` onto the polyline ``path`` and walk ``lead`` mm forward.

        Used by the pursuit loop to compute the moving "carrot" the robot
        chases. If we walk past the end of the path, the final point is
        returned (caller will then lock onto the target).
        """
        if not path:
            return pos
        if len(path) == 1:
            return path[0]

        # 1. Find the closest segment to `pos`
        best_i, best_t, best_d = 0, 0.0, float('inf')
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            dx, dy = b.x - a.x, b.y - a.y
            l2 = dx * dx + dy * dy
            if l2 < 1e-6:
                continue
            t = max(0.0, min(1.0, ((pos.x - a.x) * dx + (pos.y - a.y) * dy) / l2))
            px, py = a.x + t * dx, a.y + t * dy
            d = math.hypot(pos.x - px, pos.y - py)
            if d < best_d:
                best_d, best_i, best_t = d, i, t

        # 2. Walk `lead` mm forward along the polyline from that point
        i = best_i
        a, b = path[i], path[i + 1]
        seg_len = math.hypot(b.x - a.x, b.y - a.y)
        cur_t   = best_t
        remaining = lead
        while True:
            rem_in_seg = (1.0 - cur_t) * seg_len
            if remaining <= rem_in_seg:
                ratio = cur_t + (remaining / seg_len if seg_len > 1e-6 else 0.0)
                return Vec2(a.x + ratio * (b.x - a.x),
                            a.y + ratio * (b.y - a.y))
            remaining -= rem_in_seg
            i += 1
            if i >= len(path) - 1:
                return path[-1]
            a, b = path[i], path[i + 1]
            seg_len = math.hypot(b.x - a.x, b.y - a.y)
            cur_t   = 0.0

    def _execute_go_pursuit(self, target: Vec2) -> None:
        """Drive the robot to ``target`` using LIVE_PURSUIT mode.

        Plans an A* path, then loops at PURSUIT_TICK_S sending live ``aim()``
        commands to a moving carrot. Replans every PURSUIT_REPLAN_EVERY ticks
        to react to dynamic obstacles. Locks the carrot onto the final target
        as soon as we are within PURSUIT_CARROT_DIST × PURSUIT_LOCK_FACTOR.
        """
        print(f"[Motion] pursuit go({target.x:.0f},{target.y:.0f})")

        # 1. Plan the initial path. If A* can't find one we abort cleanly
        #    instead of driving blind.
        path_via = self._wait_for_path(target)
        if path_via is None:
            print(f"[Motion] pursuit go({target.x:.0f},{target.y:.0f}) "
                  f"FAILED — no initial path")
            self._last_ok = False
            self._reset_pending()
            return

        # 2. Switch firmware to pursuit mode and apply pending feedrate
        self._sync_fw_mode(MotionMode.LIVE_PURSUIT)
        if self._pending_feedrate > 0:
            self._t.fire(f"feed({self._pending_feedrate:.3f})")

        full_path: List[Vec2] = [Vec2(self._pos.x, self._pos.y)] + list(path_via) + [target]
        start_t = time.monotonic()
        tick    = 0
        success = False

        try:
            while True:
                now = time.monotonic()
                if (now - start_t) > PURSUIT_TIMEOUT_S:
                    print(f"[Motion] pursuit timeout after {PURSUIT_TIMEOUT_S:.0f}s")
                    break

                dist_final = self._pos.dist(target)

                # Arrival check
                if dist_final < PURSUIT_ARRIVED_DIST:
                    success = True
                    print(f"[Motion] pursuit arrived ({dist_final:.0f}mm)")
                    break

                # Periodic replan (skip first tick — we already have a path)
                if tick > 0 and (tick % PURSUIT_REPLAN_EVERY) == 0:
                    new_via, found = self._plan_path(target)
                    if found:
                        full_path = ([Vec2(self._pos.x, self._pos.y)]
                                     + list(new_via) + [target])
                    # If replan fails, keep using the previous path —
                    # APF / safety on the firmware side handles last-minute
                    # avoidance.

                # Pick the carrot. Lock to the final target when we are
                # close enough that a carrot ahead would overshoot.
                if dist_final < PURSUIT_CARROT_DIST * PURSUIT_LOCK_FACTOR:
                    carrot = target
                else:
                    carrot = self._carrot_along_path(full_path, self._pos,
                                                     PURSUIT_CARROT_DIST)

                self._t.fire(f"aim({carrot.x:.1f},{carrot.y:.1f})")

                tick += 1
                time.sleep(PURSUIT_TICK_S)
        finally:
            # Always stop the firmware pursuit job and restore feedrate.
            # Wait long enough for the controller to finish its deceleration
            # ramp so the next motion_mode() switch isn't refused with
            # "Cannot change control mode while moving".
            self._t.fire("cancel")
            time.sleep(0.3)
            if self._pending_feedrate > 0:
                self._t.fire(f"feed({self._feedrate:.3f})")

        self._last_ok = success
        self._reset_pending()
