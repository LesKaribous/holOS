"""
sim/tests.py — Simulation test suite for holOS.

Tests run against the live SimBridge + physics and report results via
SocketIO callbacks. Each test uses transport.execute() (same stack as
real hardware) and asserts expected outcomes.

Suites:
  motors    — Go, Turn, GoAlign, Cancel, Feedrate
  collision — Pathfinding, Safety pause
  actuators — SetAbsPosition, Telemetry snapshot, Feedrate
  system    — Chrono, Telemetry stream, Multi-waypoints
"""

import math
import time
import threading
from typing import Callable, List, Optional

from shared.config import Vec2, MIN_DISTANCE, MIN_ANGLE


# ── Test catalog ─────────────────────────────────────────────────────────────

SUITES = {
    'motors': {
        'label': 'Moteurs',
        'icon':  '⚙',
        'tests': [
            {'id': 'mot_hb',       'name': 'Heartbeat',            'desc': 'Liaison transport OK'},
            {'id': 'mot_go_basic', 'name': 'Go(600,600)',           'desc': 'Déplacement simple vers (600,600)'},
            {'id': 'mot_go_far',   'name': 'Go(2800,1800)',         'desc': 'Long déplacement diagonal'},
            {'id': 'mot_turn_90',  'name': 'Turn(90°)',             'desc': 'Rotation 90° vers le haut'},
            {'id': 'mot_turn_neg', 'name': 'Turn(-45°)',            'desc': 'Rotation −45° (horaire)'},
            {'id': 'mot_align',    'name': 'GoAlign(600,600,180°)', 'desc': 'Navigation + orientation finale 180°'},
            {'id': 'mot_cancel',   'name': 'Cancel motion',         'desc': 'Annulation en cours de mouvement'},
        ],
    },
    'collision': {
        'label': 'Collision',
        'icon':  '🛡',
        'tests': [
            {'id': 'col_pathfind', 'name': 'Pathfinding détour',   'desc': 'Contournement d\'obstacle fixe'},
            {'id': 'col_safety',   'name': 'Safety pause',          'desc': 'Pause sur obstacle dynamique'},
        ],
    },
    'actuators': {
        'label': 'Actuateurs',
        'icon':  '⚡',
        'tests': [
            {'id': 'act_pos_set',  'name': 'SetAbsPosition',        'desc': 'Force x=1000 y=800 θ=45°'},
            {'id': 'act_tel_snap', 'name': 'Telemetry snapshot',    'desc': 'Snapshot pos/theta cohérent'},
            {'id': 'act_feed',     'name': 'Feedrate 0.3×',         'desc': 'feed(0.3) ralentit le robot'},
        ],
    },
    'system': {
        'label': 'Système',
        'icon':  '🖥',
        'tests': [
            {'id': 'sys_chrono',   'name': 'Chrono',                'desc': 'Démarrage et incrémentation'},
            {'id': 'sys_telemetry','name': 'Télémétrie flux',       'desc': 'Paquet pos reçu en < 150 ms'},
            {'id': 'sys_multi_go', 'name': 'Multi-waypoints',       'desc': 'Suite de 3 mouvements consécutifs'},
        ],
    },
}

# Flat lookup  id → test meta
ALL_TESTS: dict = {t['id']: t for s in SUITES.values() for t in s['tests']}


# ── TestResult ───────────────────────────────────────────────────────────────

class TestResult:
    def __init__(self, id: str, passed: bool, msg: str, duration_ms: float):
        self.id          = id
        self.passed      = passed
        self.msg         = msg
        self.duration_ms = duration_ms

    def to_dict(self) -> dict:
        return {
            'id':          self.id,
            'passed':      self.passed,
            'msg':         self.msg,
            'duration_ms': round(self.duration_ms),
        }


# ── TestRunner ────────────────────────────────────────────────────────────────

class TestRunner:
    """
    Runs tests against the live simulation.

    Usage:
        runner = TestRunner(transport, bridge, robot, occupancy)
        runner.run_suite('motors', on_progress, on_result, on_done)
    """

    TIMEOUT_MS = 12_000   # default per-command timeout

    def __init__(self, transport, bridge, robot, occupancy):
        self._transport = transport
        self._bridge    = bridge
        self._robot     = robot
        self._occ       = occupancy
        self._stop      = threading.Event()
        self._running   = False

    def is_running(self) -> bool:
        return self._running

    def stop(self) -> None:
        self._stop.set()

    # ── Public run methods ───────────────────────────────────────────────────

    def run_suite(self, suite_id: str,
                  on_progress: Callable, on_result: Callable,
                  on_done: Callable) -> None:
        """Run all tests of a named suite."""
        if suite_id not in SUITES:
            on_done()
            return
        ids = [t['id'] for t in SUITES[suite_id]['tests']]
        self._run_list(ids, on_progress, on_result, on_done)

    def run_all(self, on_progress: Callable, on_result: Callable,
                on_done: Callable) -> None:
        """Run every test across all suites in order."""
        ids = [t['id'] for s in SUITES.values() for t in s['tests']]
        self._run_list(ids, on_progress, on_result, on_done)

    def run_one(self, test_id: str,
                on_progress: Callable, on_result: Callable,
                on_done: Callable) -> None:
        """Run a single test."""
        self._run_list([test_id], on_progress, on_result, on_done)

    # ── Internal orchestration ───────────────────────────────────────────────

    def _run_list(self, ids: List[str],
                  on_progress: Callable, on_result: Callable,
                  on_done: Callable) -> None:
        """Spawn background thread and run tests in sequence."""
        self._stop.clear()
        self._running = True

        def _thread():
            try:
                for tid in ids:
                    if self._stop.is_set():
                        on_progress(tid, 'stopped')
                        break
                    on_progress(tid, 'running')
                    result = self._run_one(tid)
                    on_result(result)
            finally:
                self._running = False
                on_done()

        threading.Thread(target=_thread, daemon=True, name='test-runner').start()

    def _run_one(self, test_id: str) -> TestResult:
        """Run a single test, catch all exceptions, return TestResult."""
        fn = getattr(self, f'_test_{test_id}', None)
        if fn is None:
            return TestResult(test_id, False, f'Not implemented: {test_id}', 0)
        t0 = time.perf_counter()
        try:
            msg    = fn()
            passed = True
        except AssertionError as e:
            msg    = f'ASSERT: {e}'
            passed = False
        except Exception as e:
            msg    = f'ERROR: {type(e).__name__}: {e}'
            passed = False
        dt = (time.perf_counter() - t0) * 1000.0
        return TestResult(test_id, passed, msg or 'OK', dt)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _exec(self, cmd: str, timeout_ms: Optional[int] = None) -> str:
        """Send command via transport, assert success, return response."""
        ok, res = self._transport.execute(cmd, timeout_ms or self.TIMEOUT_MS)
        assert ok, f'Command {cmd!r} failed → {res}'
        return res

    def _reset_robot(self, x: float = 300, y: float = 300,
                     theta_deg: float = 0) -> None:
        """Force robot to a known position and clear dynamic obstacles."""
        self._exec(f'setAbsPosition({x:.0f},{y:.0f},{theta_deg:.1f})')
        self._occ.dynamic_obstacles.clear()
        time.sleep(0.08)   # let physics tick apply the new position

    def _pos_err(self, target: Vec2) -> float:
        """Distance between current robot pos and target (mm)."""
        return self._robot.pos.dist(target)

    def _angle_err_deg(self, target_rad: float) -> float:
        """Absolute angle error in degrees (shortest arc)."""
        diff = target_rad - self._robot.theta
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(math.degrees(diff))

    # ── MOTORS ───────────────────────────────────────────────────────────────

    def _test_mot_hb(self) -> str:
        res = self._exec('hb', timeout_ms=2_000)
        assert res == 'ok', f'Expected "ok", got {res!r}'
        return f'Response: {res}'

    def _test_mot_go_basic(self) -> str:
        self._reset_robot(300, 300)
        target = Vec2(600, 600)
        self._exec(f'go({target.x:.0f},{target.y:.0f})', timeout_ms=8_000)
        err = self._pos_err(target)
        assert err < MIN_DISTANCE + 10, \
            f'Position error {err:.1f} mm > {MIN_DISTANCE + 10:.0f} mm'
        return (f'Arrived ({self._robot.pos.x:.0f},{self._robot.pos.y:.0f})'
                f', err={err:.1f} mm')

    def _test_mot_go_far(self) -> str:
        self._reset_robot(300, 300)
        target = Vec2(2800, 1800)
        self._exec(f'go({target.x:.0f},{target.y:.0f})', timeout_ms=25_000)
        err = self._pos_err(target)
        assert err < MIN_DISTANCE + 20, f'Position error {err:.1f} mm'
        return (f'Arrived ({self._robot.pos.x:.0f},{self._robot.pos.y:.0f})'
                f', err={err:.1f} mm')

    def _test_mot_turn_90(self) -> str:
        self._reset_robot(1500, 1000, 0)
        self._exec('turn(90)', timeout_ms=8_000)
        err = self._angle_err_deg(math.radians(90))
        assert err < 3.0, f'Angle error {err:.1f}° > 3°'
        return f'θ={math.degrees(self._robot.theta):.1f}°, err={err:.1f}°'

    def _test_mot_turn_neg(self) -> str:
        self._reset_robot(1500, 1000, 0)
        self._exec('turn(-45)', timeout_ms=8_000)
        err = self._angle_err_deg(math.radians(-45))
        assert err < 3.0, f'Angle error {err:.1f}° > 3°'
        return f'θ={math.degrees(self._robot.theta):.1f}°, err={err:.1f}°'

    def _test_mot_align(self) -> str:
        self._reset_robot(300, 300)
        target = Vec2(600, 600)
        self._exec(f'goAlign({target.x:.0f},{target.y:.0f},180)', timeout_ms=15_000)
        pos_err = self._pos_err(target)
        ang_err = self._angle_err_deg(math.radians(180))
        assert pos_err < MIN_DISTANCE + 15, \
            f'Pos error {pos_err:.1f} mm > {MIN_DISTANCE + 15:.0f} mm'
        assert ang_err < 5.0, f'Angle error {ang_err:.1f}° > 5°'
        return (f'pos_err={pos_err:.1f} mm, ang_err={ang_err:.1f}°')

    def _test_mot_cancel(self) -> str:
        self._reset_robot(300, 300)
        # Start a long move in a background thread
        result: dict = {}

        def _go():
            ok, res = self._transport.execute('go(2800,1800)', timeout_ms=15_000)
            result['ok'] = ok
            result['res'] = res

        t = threading.Thread(target=_go, daemon=True)
        t.start()
        time.sleep(0.35)   # let the robot start moving
        ok, res = self._transport.execute('cancel', timeout_ms=3_000)
        assert ok, f'Cancel command failed: {res}'
        t.join(timeout=4.0)
        state = self._bridge.motion_state()
        assert state in ('CANCELED', 'IDLE', 'COMPLETED'), \
            f'Unexpected state after cancel: {state}'
        return f'Stopped, motion state = {state}'

    # ── COLLISION ────────────────────────────────────────────────────────────

    def _test_col_pathfind(self) -> str:
        self._reset_robot(300, 300)
        # Block direct path (300,300)→(1500,300) with obstacle at cell (6,2) ≈ (900,300)
        gx, gy = 6, 2
        self._occ.toggle_cell(gx, gy)
        try:
            target = Vec2(1500, 300)
            self._exec(f'go({target.x:.0f},{target.y:.0f})', timeout_ms=18_000)
            err = self._pos_err(target)
            assert err < MIN_DISTANCE + 30, \
                f'Did not reach target: err={err:.1f} mm'
            return f'Arrived via detour, err={err:.1f} mm'
        finally:
            self._occ.toggle_cell(gx, gy)   # always restore obstacle

    def _test_col_safety(self) -> str:
        self._reset_robot(300, 300)
        self._exec('enable(SAFETY)', timeout_ms=2_000)
        try:
            # Place obstacle ~400 mm ahead in the direction of travel
            self._occ.add_dynamic_obstacle(Vec2(700, 300))

            paused = False
            result: dict = {}

            def _go():
                ok, res = self._transport.execute('go(1500,300)', timeout_ms=10_000)
                result['ok'] = ok
                result['res'] = res

            t = threading.Thread(target=_go, daemon=True)
            t.start()

            # Poll for PAUSED state (should trigger within ~300 ms)
            deadline = time.time() + 4.0
            while time.time() < deadline:
                if self._bridge.motion_state() == 'PAUSED':
                    paused = True
                    break
                if self._stop.is_set():
                    break
                time.sleep(0.05)

            # Cancel to unblock the go() thread
            self._transport.execute('cancel', timeout_ms=2_000)
            t.join(timeout=3.0)

            assert paused, 'Safety never triggered PAUSE state'
            return 'Safety correctly paused motion on obstacle detection'
        finally:
            self._occ.dynamic_obstacles.clear()
            self._exec('disable(SAFETY)', timeout_ms=2_000)

    # ── ACTUATORS ────────────────────────────────────────────────────────────

    def _test_act_pos_set(self) -> str:
        self._exec('setAbsPosition(1000,800,45)', timeout_ms=2_000)
        time.sleep(0.1)   # wait one physics tick
        pos_err = self._robot.pos.dist(Vec2(1000, 800))
        ang_err = self._angle_err_deg(math.radians(45))
        assert pos_err < 3.0,  f'Position error {pos_err:.2f} mm'
        assert ang_err < 0.5,  f'Angle error {ang_err:.2f}°'
        return (f'pos=({self._robot.pos.x:.0f},{self._robot.pos.y:.0f})'
                f', θ={math.degrees(self._robot.theta):.1f}°')

    def _test_act_tel_snap(self) -> str:
        self._reset_robot(1200, 900, 0)
        res = self._exec('tel', timeout_ms=2_000)
        assert 'x=' in res and 'y=' in res and 'theta=' in res, \
            f'Malformed telemetry: {res!r}'
        parts = dict(kv.split('=') for kv in res.split(','))
        x = float(parts['x'])
        y = float(parts['y'])
        assert abs(x - 1200) < 10, f'x={x:.0f} expected ≈1200'
        assert abs(y - 900) < 10,  f'y={y:.0f} expected ≈900'
        return f'Snapshot: {res}'

    def _test_act_feed(self) -> str:
        # Measure travel time at feedrate 1.0 then 0.3 — ratio must be > 1.5
        dist_cmd = 'go(900,300)'

        self._reset_robot(300, 300)
        self._exec('feed(1.0)', timeout_ms=2_000)
        t0 = time.perf_counter()
        self._exec(dist_cmd, timeout_ms=10_000)
        dt_full = time.perf_counter() - t0

        self._reset_robot(300, 300)
        self._exec('feed(0.3)', timeout_ms=2_000)
        t1 = time.perf_counter()
        self._exec(dist_cmd, timeout_ms=30_000)
        dt_slow = time.perf_counter() - t1

        # Always restore feedrate
        self._exec('feed(1.0)', timeout_ms=2_000)

        ratio = dt_slow / dt_full if dt_full > 0.01 else 0.0
        assert ratio > 1.5, \
            f'feed(0.3) not slower: ratio={ratio:.2f} (expected > 1.5)'
        return (f'feed=1.0 → {dt_full:.2f}s, feed=0.3 → {dt_slow:.2f}s'
                f', ratio={ratio:.2f}×')

    # ── SYSTEM ───────────────────────────────────────────────────────────────

    def _test_sys_chrono(self) -> str:
        # Reset chrono manually then send 'start'
        self._bridge._chrono_elapsed = 0.0
        self._bridge._chrono_running = False
        self._exec('start', timeout_ms=2_000)
        assert self._bridge.chrono_running(), 'Chrono not running after start'
        time.sleep(0.5)
        elapsed = self._bridge.chrono_elapsed()
        assert 0.35 < elapsed < 0.75, \
            f'Elapsed {elapsed:.3f}s, expected 0.35–0.75 s after 0.5 s sleep'
        return f'Chrono = {elapsed:.3f} s after 0.5 s'

    def _test_sys_telemetry(self) -> str:
        received = threading.Event()

        def _cb(_data):
            received.set()

        self._transport.subscribe_telemetry('pos', _cb)
        try:
            ok = received.wait(timeout=0.15)
            assert ok, 'No pos telemetry within 150 ms'
            return 'Pos telemetry received within 150 ms'
        finally:
            self._transport.unsubscribe_telemetry('pos', _cb)

    def _test_sys_multi_go(self) -> str:
        self._reset_robot(300, 300)
        waypoints = [Vec2(600, 300), Vec2(600, 800), Vec2(300, 800)]
        for wp in waypoints:
            self._exec(f'go({wp.x:.0f},{wp.y:.0f})', timeout_ms=10_000)
            err = self._pos_err(wp)
            assert err < MIN_DISTANCE + 15, \
                f'Missed waypoint ({wp.x:.0f},{wp.y:.0f}): err={err:.1f} mm'
        final = self._robot.pos
        return (f'3/3 waypoints reached, '
                f'final=({final.x:.0f},{final.y:.0f})')
