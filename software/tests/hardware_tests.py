"""
tests/hardware_tests.py — Hardware integration test suite for holOS.

Tests run against a live Teensy 4.1 connected via USB-CDC or XBee.
Each test uses the transport.execute() stack (same as real match traffic)
and verifies firmware responses and telemetry channels.

Suites
------
  connection  — Handshake, sync, health snapshot
  telemetry   — All channels stream correctly (pos, motion, safety, occ)
  motion      — Minimal movements (turn 5°, goPolar 50 mm), cancel, DONE/ack flow
  safety      — Enable/disable, obstacle detection (semi-interactive)
  intercom    — T4.0 bridge alive, occupancy map delivery, latency
  reliability — DONE retransmit, heartbeat tuning, reconnect+sync

Usage (from software/)
------
  python run_hardware_tests.py --port /dev/ttyACM0
  python run_hardware_tests.py --port COM6
  python run_hardware_tests.py --port /dev/ttyACM0 --suite connection
  python run_hardware_tests.py --port /dev/ttyACM0 --skip-interactive
"""

import math
import re
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple


# ── Test catalog ──────────────────────────────────────────────────────────────

SUITES: Dict[str, dict] = {
    'connection': {
        'label': 'Connexion',
        'icon':  '🔗',
        'tests': [
            {'id': 'con_handshake', 'name': 'Heartbeat handshake',
             'desc': 'hb(5000) → ok en < 1 s'},
            {'id': 'con_sync',      'name': 'State sync',
             'desc': 'sync → mstate,x,y,theta parseable'},
            {'id': 'con_health',    'name': 'Health snapshot',
             'desc': 'health → 14 champs valides'},
        ],
    },
    'telemetry': {
        'label': 'Télémétrie',
        'icon':  '📡',
        'tests': [
            {'id': 'tel_pos',       'name': 'TEL:pos stream',
             'desc': 'Paquet pos reçu en < 300 ms'},
            {'id': 'tel_motion',    'name': 'TEL:motion stream',
             'desc': 'Paquet motion IDLE reçu en < 300 ms'},
            {'id': 'tel_safety',    'name': 'TEL:safety stream',
             'desc': 'Paquet safety reçu en < 300 ms'},
            {'id': 'tel_snapshot',  'name': 'tel snapshot',
             'desc': 'tel → x/y/theta cohérents, sans NaN'},
            {'id': 'tel_occ',       'name': 'TEL:occ (T4.0)',
             'desc': "Carte d'occupation reçue en < 2 s (T4.0 vivant)"},
        ],
    },
    'motion': {
        'label': 'Mouvement',
        'icon':  '⚙',
        'tests': [
            {'id': 'mot_turn_tiny', 'name': 'Turn 5°',
             'desc': 'Rotation 5° → DONE:ok + ack_done'},
            {'id': 'mot_go_tiny',   'name': 'goPolar 50 mm',
             'desc': 'Avance 50 mm → DONE:ok + ack_done'},
            {'id': 'mot_cancel',    'name': 'Cancel en vol',
             'desc': 'go(200mm) → cancel → DONE:fail ou transport cancel'},
            {'id': 'mot_done_ack',  'name': 'DONE retransmit',
             'desc': 'DONE reçu à nouveau si ack_done tardif (>2 s)'},
        ],
    },
    'safety': {
        'label': 'Sécurité',
        'icon':  '🛡',
        'tests': [
            {'id': 'saf_enable',      'name': 'Enable/disable safety',
             'desc': 'health vérifie sa=1 puis sa=0'},
            {'id': 'saf_obstacle_bc', 'name': 'Détection — face BC',
             'desc': '[interactif] Guide pas-à-pas : obstacle face BC (arrière) → TEL:safety:1'},
            {'id': 'saf_obstacle_a',  'name': 'Détection — face A',
             'desc': '[interactif] Guide pas-à-pas : obstacle face A (avant) → TEL:safety:1'},
            {'id': 'saf_pause',       'name': 'Pause sur obstacle',
             'desc': '[interactif] Obstacle en cours de go → motion pausée puis reprend'},
        ],
    },
    'intercom': {
        'label': 'Intercom (T4.0)',
        'icon':  '🔄',
        'tests': [
            {'id': 'ic_alive',      'name': 'T4.0 connecté',
             'desc': 'health → ic_ok=1'},
            {'id': 'ic_occ_map',    'name': 'Carte occ valide',
             'desc': 'occ → chaîne non-vide de longueur attendue'},
            {'id': 'ic_latency',    'name': 'Latence aller-retour',
             'desc': 'Temps de réponse moyen < 200 ms sur 5 requêtes'},
        ],
    },
    'reliability': {
        'label': 'Fiabilité',
        'icon':  '⚡',
        'tests': [
            {'id': 'rel_crc_reject',  'name': 'Rejet trame CRC invalide',
             'desc': 'Trame corrompue ignorée, prochaine commande OK'},
            {'id': 'rel_done_retry',  'name': 'DONE retransmit 2 s',
             'desc': 'Motion + pas de ack_done → DONE retransmis à 2 s'},
            {'id': 'rel_reconnect',   'name': 'Reconnexion + sync',
             'desc': 'Déconnexion simulée → reconnexion → sync cohérent'},
        ],
    },
    'actuators_hw': {
        'label': 'Actionneurs',
        'icon':  '🦾',
        'tests': [
            {'id': 'act_info_ca',     'name': 'Actuator info CA',
             'desc': 'act_info(CA) → infos servos CA (id, min, max, pos)'},
            {'id': 'act_info_ab',     'name': 'Actuator info AB',
             'desc': 'act_info(AB) → infos servos AB'},
            {'id': 'act_servo_limits','name': 'Servo limits runtime',
             'desc': 'servo_limits → mise à jour min/max en direct'},
            {'id': 'act_pump_init',   'name': 'Init pompe',
             'desc': 'pump_init → pompe initialisée sans erreur'},
            {'id': 'act_ev_toggle',   'name': 'Électrovanne toggle',
             'desc': 'ev(CA,1) + ev(CA,0) → pas d\'erreur'},
        ],
    },
}

# Flat lookup id → meta
ALL_TESTS: dict = {t['id']: t for s in SUITES.values() for t in s['tests']}

# Tests that need the user to physically interact with the robot
INTERACTIVE_TESTS = {'saf_obstacle_bc', 'saf_obstacle_a', 'saf_pause'}

# ── HEALTH field names (must match firmware command_health() order) ────────────
HEALTH_FIELDS = ['mo', 'mv', 'sa', 'ob', 'ch', 'el',
                 'ac', 'li', 'ic', 'ic_ok', 'jt', 'jt_ok', 'vi', 'lo']

# Expected occupancy grid dimensions (from settings.h)
GRID_BYTES_MIN = 33   # ceil(20*13/8) = 33 bytes → base64 ~= 44 chars


# ── TestResult ────────────────────────────────────────────────────────────────

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

    def __str__(self) -> str:
        status = '✓ PASS' if self.passed else '✗ FAIL'
        name   = ALL_TESTS.get(self.id, {}).get('name', self.id)
        return f'  {status}  [{self.duration_ms:6.0f} ms]  {name}: {self.msg}'


# ── HardwareTestRunner ────────────────────────────────────────────────────────

class HardwareTestRunner:
    """
    Runs hardware integration tests against a live Transport (USB-CDC or XBee).

    Usage:
        runner = HardwareTestRunner(transport, skip_interactive=False)
        runner.run_suite('connection', on_progress, on_result, on_done)

    Callbacks (all called from background thread):
        on_progress(test_id, status)   status = 'running' | 'skipped' | 'stopped'
        on_result(TestResult)
        on_done()
    """

    TIMEOUT_MS       = 8_000     # default per-command timeout
    TEL_WAIT_MS      = 300       # max wait for a single telemetry packet
    OCC_WAIT_MS      = 2_000     # max wait for occupancy map (T4.0 cycle is 500ms)
    MOTION_TIMEOUT   = 12_000    # max wait for a motion to complete (tiny moves only)
    RETRANSMIT_GRACE = 4_000     # max wait to see DONE retransmit after first receipt

    def __init__(self, transport, skip_interactive: bool = False,
                 prompt_fn: Optional[Callable[[str], None]] = None):
        self._t                = transport
        self._skip_interactive = skip_interactive
        self._stop             = threading.Event()
        self._running          = False
        # prompt_fn(msg) is called for interactive steps.
        # In web mode, inject a SocketIO-based fn; in CLI mode use the default (input()).
        self._prompt_fn = prompt_fn

    def is_running(self) -> bool:
        return self._running

    def stop(self) -> None:
        self._stop.set()

    # ── Public run methods ─────────────────────────────────────────────────────

    def run_suite(self, suite_id: str,
                  on_progress: Callable, on_result: Callable,
                  on_done: Callable) -> None:
        if suite_id not in SUITES:
            on_done()
            return
        ids = [t['id'] for t in SUITES[suite_id]['tests']]
        self._run_list(ids, on_progress, on_result, on_done)

    def run_all(self, on_progress: Callable, on_result: Callable,
                on_done: Callable) -> None:
        ids = [t['id'] for s in SUITES.values() for t in s['tests']]
        self._run_list(ids, on_progress, on_result, on_done)

    def run_one(self, test_id: str,
                on_progress: Callable, on_result: Callable,
                on_done: Callable) -> None:
        self._run_list([test_id], on_progress, on_result, on_done)

    # ── Internal orchestration ─────────────────────────────────────────────────

    def _run_list(self, ids: List[str],
                  on_progress: Callable, on_result: Callable,
                  on_done: Callable) -> None:
        self._stop.clear()
        self._running = True

        def _thread():
            try:
                for tid in ids:
                    if self._stop.is_set():
                        on_progress(tid, 'stopped')
                        break
                    if tid in INTERACTIVE_TESTS and self._skip_interactive:
                        on_progress(tid, 'skipped')
                        on_result(TestResult(tid, True, 'Skipped (--skip-interactive)', 0))
                        continue
                    on_progress(tid, 'running')
                    result = self._run_one(tid)
                    on_result(result)
            finally:
                self._running = False
                on_done()

        threading.Thread(target=_thread, daemon=True, name='hw-test-runner').start()

    def _run_one(self, test_id: str) -> TestResult:
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

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _exec(self, cmd: str, timeout_ms: Optional[int] = None) -> str:
        """Send command, assert success, return response string."""
        ok, res = self._t.execute(cmd, timeout_ms or self.TIMEOUT_MS)
        assert ok, f'Command {cmd!r} failed → {res!r}'
        return res

    def _exec_raw(self, cmd: str, timeout_ms: Optional[int] = None) -> Tuple[bool, str]:
        """Send command, return (ok, response) without asserting."""
        return self._t.execute(cmd, timeout_ms or self.TIMEOUT_MS)

    def _wait_telemetry(self, channel: str, timeout_ms: int) -> Optional[str]:
        """Block until a telemetry packet of the given channel arrives, or timeout."""
        received: list = []
        evt = threading.Event()

        def _cb(data: str):
            received.append(data)
            evt.set()

        self._t.subscribe_telemetry(channel, _cb)
        try:
            evt.wait(timeout=timeout_ms / 1000.0)
        finally:
            self._t.unsubscribe_telemetry(channel, _cb)

        return received[0] if received else None

    def _parse_health(self, raw: str) -> Dict[str, int]:
        """Parse 'key=val,...' health string into a dict. Raises on bad format."""
        result: Dict[str, int] = {}
        for part in raw.split(','):
            part = part.strip()
            if '=' not in part:
                continue
            k, v = part.split('=', 1)
            result[k.strip()] = int(v.strip())
        return result

    def _parse_sync(self, raw: str) -> Tuple[str, float, float, float]:
        """Parse 'mstate,x,y,theta' sync response. Returns (mstate, x, y, theta)."""
        parts = raw.split(',')
        assert len(parts) == 4, f'Expected 4 parts in sync, got {len(parts)}: {raw!r}'
        return parts[0], float(parts[1]), float(parts[2]), float(parts[3])

    def _prompt(self, msg: str) -> None:
        """Interactive pause.

        In CLI mode (no prompt_fn injected): blocks on stdin.
        In web mode: prompt_fn sends a SocketIO event to the browser
        and blocks until the user clicks Continue there.
        """
        if self._prompt_fn is not None:
            self._prompt_fn(msg)
        else:
            print(f'\n  [INTERACTIF] {msg}')
            input('  >> Appuyer sur ENTRÉE pour continuer...')

    def _restore_safety(self) -> None:
        """Best-effort cleanup: disable safety after interactive tests."""
        try:
            self._exec_raw('disable(SAFETY)', timeout_ms=2_000)
        except Exception:
            pass

    def _safe_cancel(self) -> None:
        """Best-effort cancel: stop any in-progress motion.  Always safe to call."""
        try:
            self._exec_raw('cancel', timeout_ms=2_000)
            self._exec_raw('ack_done', timeout_ms=1_000)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # CONNECTION tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_con_handshake(self) -> str:
        """hb(5000) sets heartbeat timeout and returns ok."""
        t0 = time.perf_counter()
        res = self._exec('hb(5000)', timeout_ms=1_500)
        dt = (time.perf_counter() - t0) * 1000.0
        assert res == 'ok', f'Expected "ok", got {res!r}'
        assert dt < 1_500, f'Handshake took {dt:.0f} ms (max 1500 ms)'
        return f'ok en {dt:.0f} ms'

    def _test_con_sync(self) -> str:
        """sync returns parseable mstate,x,y,theta."""
        raw = self._exec('sync', timeout_ms=2_000)
        mstate, x, y, theta = self._parse_sync(raw)
        assert mstate in ('IDLE', 'RUNNING', 'DONE_OK', 'DONE_FAIL'), \
            f'Unknown mstate {mstate!r}'
        assert math.isfinite(x) and math.isfinite(y) and math.isfinite(theta), \
            f'Non-finite value in sync: {raw!r}'
        return f'mstate={mstate}, pos=({x:.0f},{y:.0f}), θ={math.degrees(theta):.1f}°'

    def _test_con_health(self) -> str:
        """health returns all 14 expected fields."""
        raw = self._exec('health', timeout_ms=2_000)
        health = self._parse_health(raw)

        missing = [f for f in HEALTH_FIELDS if f not in health]
        assert not missing, f'Champs manquants dans health: {missing}'

        # Sanity checks
        assert health['mo'] == 1, 'motion service désactivé (mo=0)'
        assert health['jt'] == 1, 'JetsonBridge désactivé (jt=0)'
        assert health['ic'] == 1, 'Intercom désactivé (ic=0)'

        return (f'14/14 champs OK — '
                f'motion={health["mo"]}, T4.0={health["ic_ok"]}, '
                f'safety={health["sa"]}, lidar={health["li"]}')

    # ─────────────────────────────────────────────────────────────────────────
    # TELEMETRY tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_tel_pos(self) -> str:
        """TEL:pos packet arrives within TEL_WAIT_MS."""
        data = self._wait_telemetry('pos', self.TEL_WAIT_MS)
        assert data is not None, f'Aucun TEL:pos reçu en {self.TEL_WAIT_MS} ms'
        assert 'x=' in data and 'y=' in data and 'theta=' in data, \
            f'Format TEL:pos invalide: {data!r}'
        # Verify no NaN/inf
        nums = re.findall(r'[-\d.]+', data)
        for n in nums:
            try:
                v = float(n)
                assert math.isfinite(v), f'Valeur non-finie dans TEL:pos: {n}'
            except ValueError:
                pass
        return f'TEL:pos OK: {data}'

    def _test_tel_motion(self) -> str:
        """TEL:motion packet arrives within TEL_WAIT_MS."""
        data = self._wait_telemetry('motion', self.TEL_WAIT_MS)
        assert data is not None, f'Aucun TEL:motion reçu en {self.TEL_WAIT_MS} ms'
        assert data.startswith('IDLE') or data.startswith('RUNNING') or \
               data.startswith('DONE'), \
            f'Format TEL:motion inconnu: {data!r}'
        return f'TEL:motion OK: {data}'

    def _test_tel_safety(self) -> str:
        """TEL:safety packet arrives within TEL_WAIT_MS."""
        data = self._wait_telemetry('safety', self.TEL_WAIT_MS)
        assert data is not None, f'Aucun TEL:safety reçu en {self.TEL_WAIT_MS} ms'
        assert data in ('0', '1'), f'Format TEL:safety invalide: {data!r}'
        return f'TEL:safety OK: obstacle={data}'

    def _test_tel_snapshot(self) -> str:
        """tel command returns a coherent x/y/theta snapshot."""
        raw = self._exec('tel', timeout_ms=2_000)
        assert 'x=' in raw and 'y=' in raw and 'theta=' in raw, \
            f'Format tel invalide: {raw!r}'
        parts = {}
        for kv in raw.split(','):
            if '=' in kv:
                k, v = kv.strip().split('=', 1)
                parts[k.strip()] = float(v.strip())
        assert 'x' in parts and 'y' in parts and 'theta' in parts, \
            f'Champs manquants: {raw!r}'
        x, y, theta = parts['x'], parts['y'], parts['theta']
        assert math.isfinite(x) and math.isfinite(y) and math.isfinite(theta), \
            f'Valeur non-finie: x={x} y={y} theta={theta}'
        # Field sanity (robot should be within 3000×2000 table)
        assert -200 <= x <= 3200 and -200 <= y <= 2200, \
            f'Position hors table: ({x:.0f},{y:.0f})'
        return f'x={x:.0f}, y={y:.0f}, θ={math.degrees(theta):.1f}°'

    def _test_tel_occ(self) -> str:
        """TEL:occ packet arrives within OCC_WAIT_MS — T4.0 bridge is alive."""
        data = self._wait_telemetry('occ', self.OCC_WAIT_MS)
        assert data is not None, \
            (f'Aucun TEL:occ reçu en {self.OCC_WAIT_MS} ms — '
             f'T4.0 hors-ligne ou intercom coupé')
        assert len(data) >= 8, \
            f'Carte occ trop courte: {len(data)} chars (attendu ≥ 8)'
        return f'TEL:occ OK: {len(data)} chars (T4.0 vivant)'

    # ─────────────────────────────────────────────────────────────────────────
    # MOTION tests  (small movements only)
    # ─────────────────────────────────────────────────────────────────────────

    def _test_mot_turn_tiny(self) -> str:
        """Turn 5° (≈0.087 rad), wait for DONE:ok, then ack_done."""
        try:
            res = self._exec('turn(5)', timeout_ms=self.MOTION_TIMEOUT)
            # transport returns 'ok' for DONE:ok, 'stall' for DONE:fail
            assert res == 'ok', \
                ('Motion annulée ou stall — vérifier moteurs/sécurité'
                 if res == 'stall' else f'Réponse inattendue: {res!r}')
            # Acknowledge the DONE event
            self._exec('ack_done', timeout_ms=2_000)
            # Health sanity: robot no longer moving
            health = self._parse_health(self._exec('health', timeout_ms=2_000))
            assert health.get('mv', -1) == 0, 'isMoving toujours 1 après DONE'
            return 'Turn 5° OK → DONE:ok → ack_done → mv=0'
        except Exception:
            self._safe_cancel()
            raise

    def _test_mot_go_tiny(self) -> str:
        """Move 50 mm forward via goPolar(0,50), verify DONE:ok (not cancelled)."""
        try:
            res = self._exec('goPolar(0,50)', timeout_ms=self.MOTION_TIMEOUT)
            # transport returns 'ok' for DONE:ok, 'stall'/'fail' for DONE:fail
            assert res == 'ok', \
                ('Motion annulée ou stall' if res == 'stall'
                 else f'goPolar(0,50) inattendu: {res!r}')
            self._exec('ack_done', timeout_ms=2_000)
            # Health sanity: robot no longer moving
            health = self._parse_health(self._exec('health', timeout_ms=2_000))
            assert health.get('mv', -1) == 0, 'isMoving toujours 1 après DONE'
            return 'goPolar(0,50) OK → DONE:ok → ack_done → mv=0'
        except Exception:
            self._safe_cancel()
            raise

    def _test_mot_cancel(self) -> str:
        """Start goPolar(0,200), cancel after 200ms, expect motion stopped."""
        result: dict = {}

        def _go():
            ok, res = self._t.execute('goPolar(0,200)', timeout_ms=self.MOTION_TIMEOUT)
            result['ok']  = ok
            result['res'] = res

        t = threading.Thread(target=_go, daemon=True)
        t.start()
        time.sleep(0.25)   # let motion begin

        ok_c, res_c = self._exec_raw('cancel', timeout_ms=3_000)
        assert ok_c, f'Commande cancel échouée: {res_c}'

        t.join(timeout=5.0)
        # After cancel, motion should not be ok
        # (transport returns False when DONE:fail or motion_timeout)
        health = self._parse_health(self._exec('health', timeout_ms=2_000))
        assert health.get('mv', -1) == 0, 'Motion toujours active après cancel'
        # Acknowledge any pending DONE
        self._exec_raw('ack_done', timeout_ms=2_000)
        return f'Cancel OK — robot arrêté (mv=0), go result: ok={result.get("ok")}'

    def _test_mot_done_ack(self) -> str:
        """
        Complete a tiny motion but delay ack_done for > DONE_RETRY_MS (2 s).
        Verify a second DONE telemetry frame arrives before we acknowledge.

        IMPORTANT: We use fire() to start motion, NOT execute(), because
        execute() auto-sends ack_done on DONE receipt, which would prevent
        the firmware from retransmitting.
        """
        done_count = [0]
        done_evt   = threading.Event()
        retx_evt   = threading.Event()

        def _on_motion(data: str):
            if data.startswith('DONE:'):
                done_count[0] += 1
                if done_count[0] == 1:
                    done_evt.set()
                elif done_count[0] >= 2:
                    retx_evt.set()

        self._t.subscribe_telemetry('motion', _on_motion)
        try:
            # Use fire() so we don't auto-ack DONE — we need to observe retransmit
            self._t.fire('turn(5)')
            # Wait for first DONE (motion completes)
            got_first = done_evt.wait(timeout=8.0)
            assert got_first, 'Premier DONE non reçu en 8 s'
            # Wait for retransmit without sending ack_done
            got_retx = retx_evt.wait(timeout=self.RETRANSMIT_GRACE / 1000.0 + 1.0)
            assert got_retx, \
                (f'DONE non retransmis après {self.RETRANSMIT_GRACE} ms '
                 f'(PR-2 retransmit not working?)')
        finally:
            self._t.unsubscribe_telemetry('motion', _on_motion)
            # Always clear the pending DONE
            self._exec_raw('ack_done', timeout_ms=2_000)

        return f'DONE retransmis OK ({done_count[0]} fois)'

    # ─────────────────────────────────────────────────────────────────────────
    # SAFETY tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_saf_enable(self) -> str:
        """Enable safety service → health sa=1; disable → sa=0."""
        self._exec('enable(SAFETY)', timeout_ms=2_000)
        h = self._parse_health(self._exec('health', timeout_ms=2_000))
        assert h.get('sa') == 1, f'safety non activé (sa={h.get("sa")})'

        self._exec('disable(SAFETY)', timeout_ms=2_000)
        h2 = self._parse_health(self._exec('health', timeout_ms=2_000))
        assert h2.get('sa') == 0, f'safety non désactivé (sa={h2.get("sa")})'
        return 'enable→sa=1 OK, disable→sa=0 OK'

    # ── Guided obstacle detection helpers ─────────────────────────────────────

    _SIDE_INFO = {
        'BC': ('face BC (arrière, 180° — côté x− quand θ=0)',
               'BC est le secteur arrière du robot (opposé à la face A).'),
        'A':  ('face A (avant, 0° — côté x+ quand θ=0)',
               'A est le secteur avant principal.'),
        'AB': ('face AB (avant-gauche, ~120°)',
               'AB est entre les faces A et B.'),
        'CA': ('face CA (avant-droit, ~240°)',
               'CA est entre les faces C et A.'),
    }

    # Direction commands per side (robot moves TOWARD that side to put it in the movement cone)
    _SIDE_MOVE_CMD = {
        'BC': 'goPolar(180,350)',   # move backward (BC = rear, 180°)
        'A':  'goPolar(0,350)',     # move forward  (A   = front,  0°)
        'AB': 'goPolar(120,350)',
        'CA': 'goPolar(240,350)',
    }

    def _guided_obstacle_test(self, side: str) -> str:
        """Generic guided obstacle detection test for a given robot side.

        The LIDAR on T4.0 only triggers within the robot's movement cone.
        The robot MUST be moving toward the tested side for detection to work.
        """
        label, hint = self._SIDE_INFO.get(side, (side, ''))
        move_cmd = self._SIDE_MOVE_CMD.get(side, f'goPolar(0,350)')
        self._exec('enable(SAFETY)', timeout_ms=2_000)
        paused = threading.Event()

        def _on_motion(data: str):
            if 'PAUSED' in data or data.startswith('DONE:fail'):
                paused.set()
        self._t.subscribe_telemetry('motion', _on_motion)

        result: dict = {}
        def _go():
            ok, res = self._t.execute(move_cmd, timeout_ms=20_000)
            result['ok'] = ok; result['res'] = res
        motion_thread = threading.Thread(target=_go, daemon=True)

        try:
            # Step 1 — orient the user
            self._prompt(
                f'ÉTAPE 1/3 — Préparation\n\n'
                f'Test de détection obstacle sur la {label}.\n\n'
                f'{hint}\n\n'
                f'⚠ Le robot va se déplacer vers la {label} ({move_cmd}).\n'
                f'Assurez-vous qu\'il y a de la place dans cette direction.\n'
                f'Ne placez PAS encore d\'obstacle.\n\n'
                f'Cliquez Continuer ▶ pour lancer le mouvement.'
            )

            # Start motion toward the tested side
            motion_thread.start()
            time.sleep(0.4)   # let motion start

            # Step 2 — ask user to place obstacle in the movement cone
            self._prompt(
                f'ÉTAPE 2/3 — Placer l\'obstacle\n\n'
                f'Le robot est en mouvement vers la {label}.\n'
                f'Placez rapidement un obstacle à 15–25 cm\n'
                f'dans la direction {label} (dans le cône de déplacement).\n\n'
                f'Cliquez Continuer ▶ dès que l\'obstacle est en place.'
            )

            # Wait for safety to trigger (motion pause or DONE:fail via stall)
            deadline = time.time() + 6.0
            was_detected = False
            while time.time() < deadline:
                ok_r, raw = self._exec_raw('health', timeout_ms=1_000)
                if ok_r:
                    h = self._parse_health(raw)
                    if h.get('ob', 0) == 1:
                        was_detected = True
                        break
                if paused.is_set():
                    was_detected = True
                    break
                time.sleep(0.2)

            assert was_detected, \
                (f'Safety non déclenché face {side} — '
                 f'vérifier que SAFETY est actif, que le LIDAR T4.0 est vivant, '
                 f'et que l\'obstacle est dans le cône de déplacement ({label})')
            return f'Safety déclenché face {side} ✓ (ob=1 pendant déplacement {move_cmd})'

        finally:
            self._t.unsubscribe_telemetry('motion', _on_motion)
            self._safe_cancel()
            motion_thread.join(timeout=3.0)
            self._restore_safety()
            # Step 3 — remove obstacle
            self._prompt(
                f'ÉTAPE 3/3 — Retirer l\'obstacle\n\n'
                f'Test terminé. Retirez l\'obstacle de la face {side}.\n'
                f'Cliquez Continuer ▶ pour fermer.'
            )

    def _test_saf_obstacle_bc(self) -> str:
        """[INTERACTIVE] Guided obstacle detection on the BC sector (rear, x−)."""
        return self._guided_obstacle_test('BC')

    def _test_saf_obstacle_a(self) -> str:
        """[INTERACTIVE] Guided obstacle detection on the A sector (front, x+)."""
        return self._guided_obstacle_test('A')

    def _test_saf_pause(self) -> str:
        """[INTERACTIVE] Obstacle placed mid-motion → robot pauses (ob=1 detected)."""
        self._exec('enable(SAFETY)', timeout_ms=2_000)
        paused = threading.Event()

        def _on_motion(data: str):
            if 'PAUSED' in data or 'pause' in data.lower():
                paused.set()

        self._t.subscribe_telemetry('motion', _on_motion)
        result: dict = {}

        def _go():
            ok, res = self._t.execute('goPolar(0,400)', timeout_ms=25_000)
            result['ok']  = ok
            result['res'] = res

        try:
            # Step 1 — inform user
            self._prompt(
                'ÉTAPE 1/3 — Préparation\n\n'
                'Le robot va avancer de 400 mm (face A, x+).\n\n'
                'Assurez-vous qu\'il y a de la place devant lui.\n'
                'Ne placez PAS encore d\'obstacle.\n\n'
                'Cliquez Continuer ▶ pour lancer le mouvement.'
            )

            t = threading.Thread(target=_go, daemon=True)
            t.start()
            time.sleep(0.4)   # let motion start

            # Step 2 — ask for obstacle
            self._prompt(
                'ÉTAPE 2/3 — Placer l\'obstacle\n\n'
                'Le robot est en mouvement (face A).\n'
                'Placez rapidement un obstacle à ~15 cm devant lui.\n\n'
                'Cliquez Continuer ▶ dès que l\'obstacle est en place.'
            )

            # Check for pause via health polling
            deadline = time.time() + 6.0
            was_paused = False
            while time.time() < deadline:
                h = self._parse_health(self._exec('health', timeout_ms=1_000))
                if h.get('ob', 0) == 1:
                    was_paused = True
                    break
                if paused.is_set():
                    was_paused = True
                    break
                time.sleep(0.2)

            assert was_paused, \
                ('Safety n\'a pas détecté l\'obstacle ou n\'a pas pausé — '
                 'vérifier que SAFETY est activé et que l\'obstacle est bien '
                 'dans le champ LIDAR (face A, ≤ 25 cm)')

            # Cancel motion (safe cleanup)
            self._exec_raw('cancel', timeout_ms=2_000)
            t.join(timeout=5.0)
            self._exec_raw('ack_done', timeout_ms=2_000)
        finally:
            self._t.unsubscribe_telemetry('motion', _on_motion)
            self._restore_safety()
            # Step 3 — remove obstacle
            self._prompt(
                'ÉTAPE 3/3 — Retirer l\'obstacle\n\n'
                'Test réussi ! Retirez l\'obstacle.\n'
                'Cliquez Continuer ▶ pour terminer.'
            )

        return f'Motion pausée sur obstacle ✓ (ob=1, résultat go: ok={result.get("ok")})'

    # ─────────────────────────────────────────────────────────────────────────
    # INTERCOM tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_ic_alive(self) -> str:
        """health → ic_ok=1 (T4.0 bridge is connected and responding)."""
        raw    = self._exec('health', timeout_ms=2_000)
        health = self._parse_health(raw)
        ic     = health.get('ic',    -1)
        ic_ok  = health.get('ic_ok', -1)
        assert ic == 1,   f'Intercom service désactivé (ic={ic})'
        assert ic_ok == 1, \
            (f'T4.0 non connecté (ic_ok={ic_ok}) — '
             f'vérifier câble UART ou firmware T4.0 allumé')
        return 'Intercom OK, T4.0 connecté (ic_ok=1)'

    def _test_ic_occ_map(self) -> str:
        """occ command returns a non-empty map of expected length."""
        raw = self._exec('occ', timeout_ms=3_000)
        assert len(raw) >= 8, f'Carte occ trop courte: {len(raw)} chars'
        # Map should be a printable ASCII string (base64 or hex encoded)
        assert raw.isprintable(), 'Carte occ contient des caractères non-imprimables'
        return f'Carte occ OK: {len(raw)} chars'

    def _test_ic_latency(self) -> str:
        """Measure average round-trip latency over 5 health queries."""
        latencies = []
        for _ in range(5):
            t0 = time.perf_counter()
            self._exec('health', timeout_ms=2_000)
            latencies.append((time.perf_counter() - t0) * 1000.0)
            time.sleep(0.05)

        avg = sum(latencies) / len(latencies)
        mx  = max(latencies)
        assert avg < 200, f'Latence moyenne trop haute: {avg:.0f} ms (max 200 ms)'
        assert mx  < 500, f'Latence maximale trop haute: {mx:.0f} ms (max 500 ms)'
        return (f'Latence: avg={avg:.0f} ms, max={mx:.0f} ms '
                f'({[f"{l:.0f}" for l in latencies]} ms)')

    # ─────────────────────────────────────────────────────────────────────────
    # RELIABILITY tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_rel_crc_reject(self) -> str:
        """
        Send a frame with a deliberately wrong CRC.
        The firmware should ignore it; the next valid command must still work.

        IMPORTANT: We write raw bytes to the serial port, NOT using fire()
        which would wrap the payload in a valid CRC frame.  The firmware
        must discard the bad-CRC frame and process the next valid command.
        """
        # Write raw corrupted frame directly (wrong CRC=255)
        if hasattr(self._t, '_write'):
            self._t._write('99:corrupted_payload|255\n')
            time.sleep(0.15)   # give firmware time to discard it
        # Now send a valid health command — firmware must still respond
        raw    = self._exec('health', timeout_ms=3_000)
        health = self._parse_health(raw)
        assert 'mo' in health, f'Aucune réponse health après trame corrompue: {raw!r}'
        return 'Trame corrompue ignorée, commande suivante OK'

    def _test_rel_done_retry(self) -> str:
        """
        Start a motion and capture the first DONE.
        Deliberately do not send ack_done.
        Expect a second DONE (retransmit) within DONE_RETRY_MS + margin.

        IMPORTANT: Use fire() for motion, NOT execute(). execute() auto-sends
        ack_done on DONE receipt, which clears m_donePending on the firmware
        and prevents the retransmit we're trying to observe.
        """
        done_times: list = []
        evt_second = threading.Event()

        def _on_motion(data: str):
            if data.startswith('DONE:'):
                done_times.append(time.perf_counter())
                if len(done_times) >= 2:
                    evt_second.set()

        self._t.subscribe_telemetry('motion', _on_motion)
        try:
            # Use fire() so we don't auto-ack DONE
            self._t.fire('turn(5)')
            # Wait for retransmit (first DONE + at least one retransmit)
            got = evt_second.wait(timeout=10.0)
            assert got, \
                ('Pas de retransmission DONE après 10 s '
                 '(vérifier PR-2 DONE_RETRY_MS=2000 dans jetson_bridge.h)')
        finally:
            self._t.unsubscribe_telemetry('motion', _on_motion)
            self._exec_raw('ack_done', timeout_ms=2_000)

        if len(done_times) >= 2:
            gap = (done_times[1] - done_times[0]) * 1000.0
            assert 1_500 <= gap <= 3_500, \
                f'Intervalle retransmit hors plage: {gap:.0f} ms (attendu 1500-3500 ms)'
            return f'DONE retransmis OK — intervalle={gap:.0f} ms'
        return 'DONE retransmis OK'

    def _test_rel_reconnect(self) -> str:
        """
        Disconnect the transport (simulate link drop), reconnect,
        then verify sync returns a coherent state.
        NOTE: this test uses disconnect()/connect() — only works on
        XBeeTransport and WiredTransport, not VirtualTransport.
        """
        if not hasattr(self._t, 'disconnect') or not hasattr(self._t, 'connect'):
            return 'Skipped (transport ne supporte pas disconnect/connect)'

        # Get state before disconnect
        raw_before      = self._exec('sync', timeout_ms=2_000)
        mstate_before, x_before, y_before, _ = self._parse_sync(raw_before)

        # Simulate link drop
        self._t.disconnect()
        time.sleep(1.5)   # simulate a 1.5 s outage

        # Reconnect
        connected = self._t.connect()
        assert connected, 'Reconnexion échouée après déconnexion'
        time.sleep(0.5)   # wait for heartbeat handshake

        # Re-negotiate heartbeat
        self._exec('hb(5000)', timeout_ms=2_000)

        # Check state consistency via sync
        raw_after        = self._exec('sync', timeout_ms=2_000)
        mstate_after, x_after, y_after, _ = self._parse_sync(raw_after)

        # Position should not have changed (robot didn't move)
        dx = abs(x_after - x_before)
        dy = abs(y_after - y_before)
        assert dx < 50 and dy < 50, \
            f'Position incohérente après reconnexion: Δ=({dx:.0f},{dy:.0f}) mm'

        return (f'Reconnexion OK — état={mstate_after}, '
                f'pos=({x_after:.0f},{y_after:.0f}) Δ=({dx:.0f},{dy:.0f}) mm')

    # ─────────────────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # ACTUATORS (hardware) tests
    # ─────────────────────────────────────────────────────────────────────────

    def _test_act_info_ca(self) -> str:
        """act_info(CA) returns structured servo information."""
        ok, raw = self._exec_raw('act_info(CA)', timeout_ms=2_000)
        assert ok, f'act_info(CA) a échoué: {raw}'
        # Expect ACT_INFO lines with pipe-separated fields
        assert 'ACT_INFO' in raw or 'ok' in raw, \
            f'Réponse act_info(CA) inattendue: {raw!r}'
        return f'act_info(CA) OK: {raw[:80]}'

    def _test_act_info_ab(self) -> str:
        """act_info(AB) returns structured servo information."""
        ok, raw = self._exec_raw('act_info(AB)', timeout_ms=2_000)
        assert ok, f'act_info(AB) a échoué: {raw}'
        assert 'ACT_INFO' in raw or 'ok' in raw, \
            f'Réponse act_info(AB) inattendue: {raw!r}'
        return f'act_info(AB) OK: {raw[:80]}'

    def _test_act_servo_limits(self) -> str:
        """servo_limits updates min/max at runtime without crash.

        This test specifically validates the interpreter underscore fix —
        commands with underscores like servo_limits previously caused an
        infinite loop in parseIdentifier() → watchdog reboot.
        """
        # Read current limits first
        ok, info = self._exec_raw('act_info(CA)', timeout_ms=2_000)
        assert ok, f'act_info(CA) préalable échoué: {info}'

        # Set limits (use safe values within normal range)
        ok, res = self._exec_raw('servo_limits(CA,0,115,165)', timeout_ms=3_000)
        assert ok, f'servo_limits a échoué: {res} (crash firmware / watchdog ?)'

        # Verify robot is still responsive
        ok2, health = self._exec_raw('health', timeout_ms=2_000)
        assert ok2, f'Robot ne répond plus après servo_limits: {health}'

        return f'servo_limits OK — firmware stable, réponse: {res}'

    def _test_act_pump_init(self) -> str:
        """initPump initializes the PCA9685 PWM driver without error.

        The pump driver (PCA9685 I²C @ 1600 Hz) is now initialized at boot,
        but the command can be called again safely (idempotent).
        """
        # Try to initialize pump — this should be safe even if already init
        ok, res = self._exec_raw('initPump', timeout_ms=3_000)
        # Even if initPump is not registered, the robot should not crash
        if not ok:
            return f'initPump non disponible (commande non enregistrée): {res}'
        # Verify robot still responsive
        ok2, _ = self._exec_raw('health', timeout_ms=2_000)
        assert ok2, 'Robot ne répond plus après initPump'
        return f'initPump OK: {res}'

    def _test_act_ev_toggle(self) -> str:
        """Toggle an electro-valve on and off without error.

        Firmware commands: pump(side) and ev(side) where side is '1' (RIGHT)
        or '0' (LEFT).  pump() starts the pump, ev() stops pump + pulses EV.

        stopPump() timing breakdown (all hard-blocking delay(), NOT waitMs()):
          • 100 ms back-EMF settle (pump off → EV on gap, prevents brownout reset)
          • 500 ms EV pulse duration
          Total: ~600 ms → use a 3 000 ms command timeout.

        The PCA9685 is auto-initialized by startPump() on first use, so the
        "initPump" test does not need to run first.
        """
        # Start pump on RIGHT side (auto-inits PCA9685 if needed)
        ok1, res1 = self._exec_raw('pump(1)', timeout_ms=3_000)
        if not ok1:
            return f'pump(1) non disponible: {res1}'
        time.sleep(0.3)
        # Stop pump + pulse EV on RIGHT side (firmware blocks ~600ms total)
        ok2, res2 = self._exec_raw('ev(1)', timeout_ms=3_000)
        assert ok2, f'ev(1) a échoué: {res2}'
        # Verify robot still responsive
        ok3, _ = self._exec_raw('health', timeout_ms=2_000)
        assert ok3, 'Robot ne répond plus après ev toggle'
        return f'EV toggle OK: pump={res1}, ev={res2}'
