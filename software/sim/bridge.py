"""
sim/bridge.py — SimBridge: connects VirtualTransport commands to physics sim.

This is the "fake Teensy" in simulator mode. It:
  - Receives command strings from VirtualTransport.dispatch()
  - Executes them on the physics objects (MotionSim, etc.)
  - Calls transport._resolve() when each command finishes
  - Pushes telemetry to the transport at regular intervals

Threading model:
  Strategy thread  → calls transport.execute("go(x,y)") → blocks on event
  Physics thread   → calls bridge.tick(dt) at ~60Hz
                   → updates physics, signals completion, injects telemetry
"""

import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from shared.config import (
    Vec2, angle_diff, ROBOT_RADIUS,
    MAX_SPEED, MAX_ACCEL, MAX_ROT_SPEED,
    MIN_DISTANCE, MIN_ANGLE, WAYPOINT_RADIUS,
    OBS_RADIUS, SAFETY_CHECK_S, SAFETY_RESUME_S,
    MATCH_DURATION_S, RobotCompass, robot_compass_offset_deg, polar_vec,
)
from sim.physics import RobotPhysics
from sim.world import OccupancyGrid, Pathfinder, GameObjects


# ── Motion states ─────────────────────────────────────────────────────────────

class _MS:
    IDLE = "IDLE"; RUNNING = "RUNNING"; PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"; CANCELED = "CANCELED"


# ── Waypoint ──────────────────────────────────────────────────────────────────

class _Waypoint:
    def __init__(self, pos: Vec2, theta: Optional[float], pass_through: bool,
                 cancel_on_collide: bool = False):
        self.pos             = pos
        self.theta           = theta
        self.pass_through    = pass_through
        self.cancel_on_collide = cancel_on_collide


# ── SimBridge ─────────────────────────────────────────────────────────────────

class SimBridge:
    """
    Central simulation bridge.  One instance per simulator session.

    Usage:
        bridge = SimBridge(robot, occupancy, pathfinder, game_objects)
        transport.attach_bridge(bridge)
        # Then call bridge.tick(dt) from the physics loop at ~60Hz
    """

    TEL_PERIOD_S = 0.05   # Push telemetry every 50ms
    OCC_PERIOD_S = 0.20   # Push occupancy map every 200ms

    def __init__(self, robot: RobotPhysics, occupancy: OccupancyGrid,
                 pathfinder: Pathfinder, game_objects: GameObjects):
        self._robot    = robot
        self._occ      = occupancy
        self._pf       = pathfinder
        self._go        = game_objects
        self._transport = None   # Set by attach_transport()

        # Motion state
        self._lock         = threading.Lock()
        self._motion_state = _MS.IDLE
        self._waypoints:   List[_Waypoint] = []
        self._wp_idx       = 0
        self._pending_uid: Optional[int]  = None
        self._pending_cb:  Optional[Callable] = None
        self._current_path: List[Vec2] = []

        # Pause state
        self._paused_target_pos   = None
        self._paused_target_theta = None

        # Safety
        self._safety_enabled  = False
        self._safety_detected = False
        self._safety_timer    = 0.0
        self._safety_last_t   = 0.0

        # Match chrono
        self._chrono_elapsed  = 0.0
        self._chrono_running  = False

        # Feedrate
        self._feedrate = 1.0

        # Telemetry timers
        self._tel_timer = 0.0
        self._occ_timer = 0.0

    def attach_transport(self, transport) -> None:
        self._transport = transport

    # ── Physics tick (called at ~60Hz from server.py) ─────────────────────────

    def tick(self, dt: float) -> None:
        # Update physics
        self._robot.update(dt, self._occ)

        # Update chrono
        if self._chrono_running:
            self._chrono_elapsed = min(self._chrono_elapsed + dt, MATCH_DURATION_S)
            if self._chrono_elapsed >= MATCH_DURATION_S:
                self._chrono_running = False

        # Motion state machine
        with self._lock:
            self._tick_motion(dt)

        # Safety
        self._safety_timer += dt
        if self._safety_timer >= SAFETY_CHECK_S:
            self._tick_safety(self._safety_timer)
            self._safety_timer = 0.0

        # Telemetry push
        self._tel_timer += dt
        self._occ_timer += dt
        if self._tel_timer >= self.TEL_PERIOD_S:
            self._push_telemetry()
            self._tel_timer = 0.0
        if self._occ_timer >= self.OCC_PERIOD_S:
            self._push_occupancy()
            self._occ_timer = 0.0

    def _tick_motion(self, dt: float) -> None:
        if self._motion_state != _MS.RUNNING:
            return

        wp = self._waypoints[self._wp_idx]

        # Collision cancel
        if wp.cancel_on_collide and self._robot.collided:
            self._complete_motion(success=False, locked=True)
            return

        dist = self._robot.pos.dist(wp.pos)

        if wp.pass_through and dist < WAYPOINT_RADIUS:
            self._advance_waypoint(locked=True)
            return

        if not wp.pass_through:
            pos_ok   = dist < MIN_DISTANCE
            theta_ok = True
            if wp.theta is not None:
                theta_ok = abs(angle_diff(wp.theta, self._robot.theta)) < MIN_ANGLE
            if pos_ok and theta_ok:
                self._complete_motion(success=True, locked=True)

    def _advance_waypoint(self, locked: bool = False) -> None:
        self._wp_idx += 1
        if self._wp_idx >= len(self._waypoints):
            self._complete_motion(success=True, locked=locked)
            return
        wp = self._waypoints[self._wp_idx]
        self._robot.target_pos   = wp.pos
        self._robot.target_theta = wp.theta

    def _complete_motion(self, success: bool, locked: bool = False) -> None:
        self._motion_state = _MS.COMPLETED if success else _MS.CANCELED
        self._robot.target_pos   = None
        self._robot.target_theta = None
        self._robot.vel          = Vec2(0, 0)
        self._robot.vtheta       = 0.0
        uid = self._pending_uid
        cb  = self._pending_cb
        self._pending_uid = None
        self._pending_cb  = None
        if cb and uid is not None:
            threading.Thread(
                target=cb, args=(uid, success, "ok" if success else "stall"),
                daemon=True
            ).start()
        # Push telemetry immediately
        if self._transport:
            self._transport.inject_telemetry(
                "motion", "DONE:ok" if success else "DONE:fail"
            )

    # ── Safety ────────────────────────────────────────

    def _tick_safety(self, dt: float) -> None:
        if not self._safety_enabled:
            return
        with self._lock:
            if self._motion_state != _MS.RUNNING or self._robot.vel.mag() < 50:
                self._safety_detected = False
                if self._motion_state == _MS.PAUSED:
                    self._resume_motion_locked()
                return
            direction = self._robot.vel.normalized()
            obs = self._check_lookahead(direction)
            now = time.time()
            if obs:
                self._safety_detected = True
                self._safety_last_t   = now
                if self._motion_state == _MS.RUNNING:
                    self._pause_motion_locked()
            else:
                if now - self._safety_last_t > SAFETY_RESUME_S:
                    self._safety_detected = False
                    if self._motion_state == _MS.PAUSED:
                        self._resume_motion_locked()

    def _check_lookahead(self, direction: Vec2) -> bool:
        for step in range(1, 5):
            probe = self._robot.pos + direction * (350.0 * step / 4)
            if self._occ.is_occupied_circle(probe, ROBOT_RADIUS + 30):
                return True
        return False

    def _pause_motion_locked(self) -> None:
        self._motion_state            = _MS.PAUSED
        self._paused_target_pos       = self._robot.target_pos
        self._paused_target_theta     = self._robot.target_theta
        self._robot.target_pos        = None
        self._robot.target_theta      = None

    def _resume_motion_locked(self) -> None:
        self._motion_state        = _MS.RUNNING
        self._robot.target_pos    = self._paused_target_pos
        self._robot.target_theta  = self._paused_target_theta

    # ── Command dispatch (called by VirtualTransport) ─────────────────────────

    def dispatch(self, uid: int, cmd: str, resolve_cb: Callable) -> None:
        """Execute a command. resolve_cb(uid, success, response) when done."""
        cmd = cmd.strip()

        # ── Chained commands: via(x,y);...;go(x,y) ────────────────────────
        # The motion service pre-computes A* waypoints as via() prefixes.
        # The sim bridge does its own pathfinding in _start_motion_xy, so we
        # simply discard the via() parts and execute the final motion command.
        if ';' in cmd:
            parts = [p.strip() for p in cmd.split(';')]
            final = next((p for p in reversed(parts) if not p.startswith('via(')), None)
            if final:
                cmd = final
            # (fall through to the normal command dispatch below)

        # ── Heartbeat ──────────────────────────────────────────────────────
        if cmd == "hb":
            resolve_cb(uid, True, "ok")
            return

        # ── Telemetry snapshot ─────────────────────────────────────────────
        if cmd == "tel":
            pos = self._robot.pos
            data = f"x={pos.x:.1f},y={pos.y:.1f},theta={self._robot.theta:.4f}"
            resolve_cb(uid, True, data)
            return

        # ── Occupancy request ──────────────────────────────────────────────
        if cmd == "occ":
            resolve_cb(uid, True, self._occ.to_hex_bytes())
            return

        # ── Motion commands ────────────────────────────────────────────────
        if cmd.startswith("go("):
            x, y = _parse_xy(cmd[3:-1])
            self._start_motion_xy(uid, Vec2(x, y), resolve_cb)
            return

        if cmd.startswith("go_coc("):
            x, y = _parse_xy(cmd[7:-1])
            self._start_motion_xy(uid, Vec2(x, y), resolve_cb, cancel_on_collide=True)
            return

        if cmd.startswith("goPolar("):
            parts = cmd[8:-1].split(",")
            hdg, dist = float(parts[0]), float(parts[1])
            target = self._robot.pos + polar_vec(hdg, dist)
            self._start_motion_xy(uid, target, resolve_cb)
            return

        if cmd.startswith("turn("):
            angle_deg = float(cmd[5:-1])
            self._start_motion_turn(uid, math.radians(angle_deg), resolve_cb)
            return

        if cmd.startswith("goAlign("):
            parts = cmd[8:-1].split(",")
            x, y, theta_deg = float(parts[0]), float(parts[1]), float(parts[2])
            self._start_motion_align(uid, Vec2(x, y), math.radians(theta_deg), resolve_cb)
            return

        # ── Motion control ────────────────────────────────────────────────
        if cmd == "cancel":
            with self._lock:
                self._complete_motion(success=False, locked=True)
            resolve_cb(uid, True, "ok")
            return

        if cmd == "pause":
            with self._lock:
                self._pause_motion_locked()
            resolve_cb(uid, True, "ok")
            return

        if cmd == "resume":
            with self._lock:
                self._resume_motion_locked()
            resolve_cb(uid, True, "ok")
            return

        # ── Safety ────────────────────────────────────────────────────────
        if cmd == "enable(SAFETY)":
            self._safety_enabled = True
            resolve_cb(uid, True, "ok")
            return

        if cmd == "disable(SAFETY)":
            self._safety_enabled = False
            resolve_cb(uid, True, "ok")
            return

        # ── Position override ─────────────────────────────────────────────
        if cmd.startswith("setAbsPosition("):
            parts = cmd[15:-1].split(",")
            self._robot.pos   = Vec2(float(parts[0]), float(parts[1]))
            self._robot.theta = math.radians(float(parts[2]))
            resolve_cb(uid, True, "ok")
            return

        # ── Feedrate ──────────────────────────────────────────────────────
        if cmd.startswith("feed("):
            self._feedrate = float(cmd[5:-1])
            self._robot.feedrate = self._feedrate
            resolve_cb(uid, True, "ok")
            return

        # ── Vision ────────────────────────────────────────────────────────
        if cmd.startswith("vision("):
            parts = cmd[7:-1].split(",")
            pos = Vec2(float(parts[0]), float(parts[1]))
            _, color = self._go.query_color_at(pos)
            resolve_cb(uid, True, color.name)
            return

        # ── Start match ───────────────────────────────────────────────────
        if cmd == "start":
            self._chrono_elapsed = 0.0
            self._chrono_running = True
            resolve_cb(uid, True, "ok")
            return

        # ── Unknown ───────────────────────────────────────────────────────
        print(f"[SimBridge] Unknown command: {cmd!r}")
        resolve_cb(uid, False, f"unknown:{cmd}")

    def dispatch_fire(self, cmd: str) -> None:
        """Fire-and-forget — same as dispatch but no callback."""
        def _noop(uid, ok, resp): pass
        self.dispatch(0, cmd, _noop)

    # ── Motion helpers ────────────────────────────────────────────────────────

    def _start_motion_xy(self, uid: int, target: Vec2, cb: Callable,
                         cancel_on_collide: bool = False) -> None:
        path = self._pf.find_path(self._robot.pos, target)
        wps  = []
        prev = self._robot.pos
        for i, p in enumerate(path[1:]):
            is_last = (i == len(path) - 2)
            # Compute bearing toward this waypoint so the robot faces the
            # direction of travel — matches real firmware behaviour.
            bearing = math.atan2(p.y - prev.y, p.x - prev.x)
            wps.append(_Waypoint(p, bearing, not is_last, cancel_on_collide))
            prev = p
        self._set_motion(uid, wps, cb)

    def _start_motion_turn(self, uid: int, theta: float, cb: Callable) -> None:
        wp = _Waypoint(self._robot.pos, theta, False)
        self._set_motion(uid, [wp], cb)

    def _start_motion_align(self, uid: int, target: Vec2, theta: float,
                            cb: Callable) -> None:
        path = self._pf.find_path(self._robot.pos, target)
        wps  = []
        for i, p in enumerate(path[1:]):
            is_last = (i == len(path) - 2)
            t = theta if is_last else None
            wps.append(_Waypoint(p, t, not is_last))
        self._set_motion(uid, wps, cb)

    def _set_motion(self, uid: int, wps: List[_Waypoint], cb: Callable) -> None:
        if not wps:
            cb(uid, True, "ok")
            return
        with self._lock:
            # Cancel any pending motion without triggering its callback
            self._pending_uid = None
            self._pending_cb  = None
            self._waypoints   = wps
            self._wp_idx      = 0
            self._pending_uid = uid
            self._pending_cb  = cb
            self._current_path = [self._robot.pos] + [w.pos for w in wps]
            self._robot.target_pos   = wps[0].pos
            self._robot.target_theta = wps[0].theta
            self._motion_state = _MS.RUNNING
            self._robot.feedrate = self._feedrate

    # ── Telemetry push ────────────────────────────────────────────────────────

    def _push_telemetry(self) -> None:
        if not self._transport:
            return
        p = self._robot.pos
        self._transport.inject_telemetry(
            "pos", f"x={p.x:.1f},y={p.y:.1f},theta={self._robot.theta:.4f}"
        )
        self._transport.inject_telemetry(
            "motion", self._motion_state
        )
        self._transport.inject_telemetry(
            "safety", "1" if self._safety_detected else "0"
        )
        self._transport.inject_telemetry(
            "chrono", str(int(self._chrono_elapsed * 1000))
        )

    def _push_occupancy(self) -> None:
        if not self._transport:
            return
        self._transport.inject_telemetry("occ", self._occ.to_hex_bytes())

    # ── State for web UI ──────────────────────────────────────────────────────

    def motion_state(self) -> str:
        return self._motion_state

    def current_path(self) -> List[Vec2]:
        return self._current_path

    def chrono_elapsed(self) -> float:
        return self._chrono_elapsed

    def chrono_running(self) -> bool:
        return self._chrono_running

    def safety_detected(self) -> bool:
        return self._safety_detected


# ── Helpers ───────────────────────────────────────────────────────────────────

def _parse_xy(s: str) -> Tuple[float, float]:
    parts = s.split(",")
    return float(parts[0]), float(parts[1])
