"""
run.py — Unified holOS entry point (PC + Jetson, simulator + hardware).

Single entry point for all platforms:
  - Windows (PC): simulator mode or USB wired connection
  - Linux (Jetson): auto-connects to Teensy via /dev/ttyUSB0

Modes:
  - Simulator mode (VirtualTransport + SimBridge physics)
  - USB wired mode (WiredTransport → Teensy USB-CDC)
  - XBee radio mode (XBeeTransport → Teensy Serial3)

The web UI is always served; connection mode can be selected from the
browser or pre-configured with --connect.

Platform detection:
  - Linux  → assumed Jetson: auto-connect /dev/ttyUSB0 if --connect omitted
  - Windows → assumed PC dev: start in idle/sim mode

Usage:
    python run.py                          # PC: idle mode / Jetson: auto-connect
    python run.py --sim                    # start in simulator mode
    python run.py --connect COM6           # auto-connect USB on startup
    python run.py --connect /dev/ttyUSB0 --baud 57600    # auto-connect XBee
    python run.py --port 8080 --host 0.0.0.0             # custom web server
"""

import sys
import os
import time
import threading
import traceback
import importlib.util
import argparse
import math
import platform

# ── Platform detection ────────────────────────────────────────────────────────
IS_JETSON = (platform.system() == 'Linux')
print(f"[run.py] Detected platform: {platform.system()} (IS_JETSON={IS_JETSON})")

_DEFAULT_JETSON_PORT  = '/dev/ttyTHS1'
# Jetson 40-pin header pins 8 & 10 → UART2 (ttyTHS1).
# Not enumerated by list_ports.comports(), so we inject it manually.
_ONBOARD_UART         = '/dev/ttyTHS1'
_ONBOARD_UART_DESC    = 'Onboard UART2 (pins 8 & 10) — XBee'

# Sequence runner state
_seq_stop   = threading.Event()
_seq_thread = None

# ── Path setup ────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
STRATEGY_DIR = os.path.join(_HERE, 'strategy')
MACROS_FILE    = os.path.join(STRATEGY_DIR, 'macros.json')
MISSIONS_FILE  = os.path.join(STRATEGY_DIR, 'missions.json')
OBJECTS_FILE   = os.path.join(STRATEGY_DIR, 'objects.json')
FALLBACK_PATH  = '/mission_fallback.cfg'   # legacy — SD removed
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from flask import Flask, render_template, jsonify, request
import json
import glob
from flask_socketio import SocketIO, emit

from shared.config import (
    FIELD_W, FIELD_H, GRID_W, GRID_H, GRID_CELL,
    Vec2, Team, ObjectColor, COLOR_HEX, COLOR_BY_NAME, ROBOT_RADIUS, POI,
    HW_THETA_OFFSET_DEG,
)
from shared.occupancy  import OccupancyGrid
from shared.settings   import SettingsStore
from shared.pathfinder import Pathfinder
from sim.physics   import RobotPhysics
from sim.world     import GameObjects
from sim.bridge    import SimBridge
from sim.tests     import TestRunner, SUITES, ALL_TESTS
from tests.hardware_tests import (
    HardwareTestRunner,
    SUITES as HW_SUITES,
)
from transport.virtual import VirtualTransport
from brain import Brain

# ── Flask / SocketIO ──────────────────────────────────────────────────────────

_STATIC = os.path.join(_HERE, 'sim', 'static')
app = Flask(__name__, static_folder=_STATIC, template_folder=_STATIC)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0   # no static cache in dev
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ── Static occupancy map path ─────────────────────────────────────────────────

_STATIC_OCC_PATH = os.path.join(_HERE, 'strategy', 'static_occupancy.json')

# ── Simulation objects ────────────────────────────────────────────────────────

robot     = RobotPhysics()
occupancy = OccupancyGrid()
pathfinder= Pathfinder(occupancy)
game_objs = GameObjects()

# Auto-load persisted static occupancy map
occupancy.load_static(_STATIC_OCC_PATH)

# ── Settings store (replaces firmware SD card) ───────────────────────────────
_SETTINGS_PATH = os.path.join(_HERE, 'data', 'settings.json')
settings_store = SettingsStore(_SETTINGS_PATH, log=lambda msg: print(f"  {msg}"))

bridge    = SimBridge(robot, occupancy, pathfinder, game_objs)
transport = VirtualTransport()

# Wire transport ↔ bridge
transport.attach_bridge(bridge)
bridge.attach_transport(transport)

# Brain (strategy + services) — share the same occupancy grid so the sim
# pathfinder and the strategy-level OccupancyService both see the same map.
brain = Brain(transport, occupancy_grid=occupancy)

# Test runner
test_runner = TestRunner(transport, bridge, robot, occupancy)

# ── Vision backend (optional — requires opencv-contrib-python) ────────────────
# Gracefully unavailable if OpenCV is not installed.
_vision = None
try:
    from services.vision_backend import VisionBackend, CV2_AVAILABLE as _CV2_AVAIL
    _vision = VisionBackend(OBJECTS_FILE)
except Exception as _ve:
    print(f"[Vision] Backend unavailable: {_ve}")

# ── Hardware transport (real robot, optional) ─────────────────────────────────
# When connected, terminal/actuator/strategy commands are routed here instead
# of the VirtualTransport.  The physics simulation continues running for map
# visualisation, but robot state is driven by real telemetry.

# ── Connection mode ───────────────────────────────────────────────────────────
# 'idle' — nothing connected, frozen state (default at startup)
# 'sim'  — simulator running (user explicitly chose Sim)
# 'usb'  — hardware connected via USB-CDC
# 'xbee' — hardware connected via XBee
_connection_mode    = 'idle'

_hw_transport       = None   # WiredTransport / XBeeTransport instance when connected
_hw_brain           = None   # Brain wired to real hardware
_hw_test_runner     = None   # TestRunner for real hardware tests
_hw_connecting      = False  # True while a connection attempt is in progress
_hw_serial_port     = None   # Serial port in use (e.g. 'COM6' or '/dev/ttyACM0')
_hw_serial_mode     = 'wired' # 'wired' | 'xbee'

# ── Match state (server-authoritative, broadcast to all clients) ──────────────
_match_running = False
_match_paused  = False


def _on_match_done():
    """Callback fired when the strategy thread finishes (success or error).

    Resets server-side state and notifies all clients so the Start button
    returns to its idle state.  Safe to call from any thread — socketio.emit
    is thread-safe.
    """
    global _match_running, _match_paused
    _match_running = False
    _match_paused  = False
    brain.log('[MATCH] Strategy finished — resetting match state')
    socketio.emit('match_state', {'running': False, 'paused': False})

# ── Calibration state (Python-side cache) ─────────────────────────────────────
# Mirrors Calibration::Current + OtosLinear/OtosAngular on the firmware.
# Updated whenever the user sends a calibration command; reset to firmware
# defaults when calib_reset is called.
_CALIB_DEFAULTS = {
    'cx': 1.203677, 'cy': -1.203677, 'cr': 0.831,  # Cartesian scale factors
    'ha': 1.0,   'hb': 1.0,   'hc': 1.0,     # per-wheel holonomic factors
    'ol': 0.990723, 'oa': 1.0,                  # OTOS linear / angular scalars
}
_calib = dict(_CALIB_DEFAULTS)

# ── Interactive test prompt — web-safe alternative to input() ────────────────
# The test thread calls _web_prompt(msg), which emits 'test_prompt' to the
# browser and blocks on this Event until the user clicks Continue (which
# triggers a 'test_prompt_ack' SocketIO event that sets the event).
_prompt_evt = threading.Event()

def _web_prompt(msg: str) -> None:
    """Emit a prompt to the browser, block until the user acknowledges it."""
    _prompt_evt.clear()
    socketio.emit('test_prompt', {'msg': msg})
    _prompt_evt.wait(timeout=120.0)   # user has 2 minutes to confirm

# Telemetry channel mask — updated live from TEL:mask: frames
_hw_tel_mask = {
    'pos':    True,
    'motion': True,
    'safety': True,
    'chrono': True,
    'occ':    True,
}

# Latest raw telemetry strings from the robot — overrides sim values in _build_state()
_hw_tel_data = {
    'motion': None,   # e.g. "RUNNING,tx=1500.0,ty=800.0,dist=200.0,feed=0.80"
    'safety': None,   # e.g. "1" or "0"
    'chrono': None,   # e.g. "42.3" (seconds elapsed)
    't40':    None,   # e.g. "ok" or "timeout" (T4.0 intercom health)
}

# Jetson edge-computer configuration (for XBee / remote holOS path)
_jetson_config = {
    'ip':   '',
    'user': 'robot',
    'port': 22,
}


def _active_transport():
    """Return the XBee transport if connected, else the sim VirtualTransport."""
    if _hw_transport is not None and _hw_transport.is_connected:
        return _hw_transport
    return transport


def _active_brain():
    """Return the hw Brain if XBee is connected, else the sim Brain."""
    if _hw_transport is not None and _hw_transport.is_connected and _hw_brain is not None:
        return _hw_brain
    return brain


def _is_hw_connected():
    """True if hardware transport is alive and connected."""
    return (_hw_transport is not None
            and _hw_transport.is_connected
            and _connection_mode in ('usb', 'xbee'))


def _active_test_runner():
    """Return the hw TestRunner if connected, else the sim TestRunner."""
    if _is_hw_connected() and _hw_test_runner is not None:
        return _hw_test_runner
    return test_runner

# ── Simulator UI state ────────────────────────────────────────────────────────

sim_state = {
    'team':     'yellow',
    'mode':     'target',
    'features': {
        'collision':    True,
        'safety':       True,
        'pathfinding':  True,
        'pursuit':      False,   # LIVE_PURSUIT mode for next go() (auto-revert)
        'show_grid':    False,
        'show_trail':   True,
        'show_path':    True,
    },
    'score': 0,
    'opponent': {'x': 0, 'y': 0, 'theta': 0, 'enabled': False},
}


# ── Physics loop (60 Hz) ──────────────────────────────────────────────────────

_PHYSICS_DT = 1.0 / 60.0

def _physics_loop():
    """60 Hz state broadcaster.  Only ticks sim when in 'sim' mode."""
    while True:
        t0 = time.perf_counter()
        # Only tick the simulator in explicit 'sim' mode
        # In 'idle' mode: frozen (no physics, but still broadcast state)
        # In 'usb'/'xbee' mode: state comes from real telemetry
        if _connection_mode == 'sim':
            bridge.tick(_PHYSICS_DT)
        socketio.emit('state', _build_state())
        elapsed = time.perf_counter() - t0
        sleep   = _PHYSICS_DT - elapsed
        if sleep > 0:
            time.sleep(sleep)


# ── State serialization ───────────────────────────────────────────────────────

def _build_state() -> dict:
    hw_on = _connection_mode in ('usb', 'xbee')
    sim_on = _connection_mode == 'sim'

    # ── Parse live hardware telemetry (when connected) ───────────────────────
    hw_motion_state = 'IDLE'
    hw_motion_feed  = 1.0
    hw_motion_target = None
    if hw_on and _hw_tel_data.get('motion'):
        parts = _hw_tel_data['motion'].split(',')
        hw_motion_state = parts[0] if parts else 'IDLE'
        _tx = _ty = None
        for p in parts[1:]:
            if p.startswith('feed='):
                try: hw_motion_feed = float(p[5:])
                except ValueError: pass
            elif p.startswith('tx='):
                try: _tx = float(p[3:])
                except ValueError: pass
            elif p.startswith('ty='):
                try: _ty = float(p[3:])
                except ValueError: pass
        if _tx is not None and _ty is not None:
            hw_motion_target = [_tx, _ty]

    hw_safety_detected = False
    if hw_on and _hw_tel_data.get('safety') is not None:
        hw_safety_detected = _hw_tel_data['safety'].strip() == '1'

    hw_chrono_elapsed = 0.0
    if hw_on and _hw_tel_data.get('chrono') is not None:
        try: hw_chrono_elapsed = float(_hw_tel_data['chrono'])
        except ValueError: pass

    # Sim data only available in 'sim' mode
    path_pts = [[p.x, p.y] for p in bridge.current_path()] if sim_on else []
    occ_list = occupancy.to_list()  # updated by sim physics or by _on_occ hw telemetry
    objs     = game_objs.to_list()   # always include (colors set via on_set_color)

    # Motion target: from hardware telemetry or sim bridge
    if hw_on:
        motion_target_pt = hw_motion_target
    elif sim_on:
        mt = bridge.motion_target() if hasattr(bridge, 'motion_target') else None
        motion_target_pt = [mt.x, mt.y] if mt else None
    else:
        motion_target_pt = None

    return {
        'robot':         robot.to_dict(),
        'path':          path_pts,
        'motion_target': motion_target_pt,
        'occupancy':     occ_list,
        'game_objs':     objs,
        'opponent':  sim_state.get('opponent', {'enabled': False}),
        'motion': {
            'state':    hw_motion_state if hw_on else (bridge.motion_state() if sim_on else 'IDLE'),
            'feedrate': hw_motion_feed  if hw_on else (brain.motion.get_feedrate() if sim_on else 1.0),
        },
        'safety': {
            'enabled':  True if hw_on else sim_state['features']['safety'],
            'detected': hw_safety_detected if hw_on else (bridge.safety_detected() if sim_on else False),
        },
        'chrono': {
            'elapsed': hw_chrono_elapsed if hw_on else (bridge.chrono_elapsed() if sim_on else 0.0),
            'left':    max(0, 100.0 - hw_chrono_elapsed) if hw_on
                       else max(0, 100.0 - bridge.chrono_elapsed()) if sim_on else 100.0,
            'running': hw_on or (bridge.chrono_running() if sim_on else False),
        },
        'score':    sim_state['score'],
        'team':     sim_state['team'],
        'features': sim_state['features'],
        'mode':     sim_state['mode'],
        'log':      _active_brain().get_log(30),
        # ── Connection metadata ──────────────────────────────────────────────
        'connection_mode': _connection_mode,     # idle | sim | usb | xbee
        'hw_mode':         hw_on,
        'hw_connecting':   _hw_connecting,
        'hw_type':         (getattr(_hw_transport, 'transport_type', 'sim') if hw_on else 'sim'),
        'hw_serial_port':  _hw_serial_port if hw_on else None,
        'hw_tel_mask':     dict(_hw_tel_mask),
        'hw_t40':          (_hw_tel_data.get('t40') or 'unknown') if hw_on else 'unknown',
        'jetson_ip':       _jetson_config.get('ip', ''),
        # ── Match state ──────────────────────────────────────────────────────────
        'match_running':   _match_running,
        'match_paused':    _match_paused,
        # ── Platform ─────────────────────────────────────────────────────────────
        'is_jetson':       IS_JETSON,
    }


# ── HTTP routes ───────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/api/colors')
def api_colors():
    return jsonify({c.name: COLOR_HEX[c] for c in ObjectColor})

@app.route('/api/poi')
def api_poi():
    return jsonify([{'name': n, 'x': v.x, 'y': v.y} for n, v in POI.all_named()])


# ── Hardware modules API ───────────────────────────────────────────────────────

_HW_SERVICES = [
    'LIDAR', 'CHRONO', 'IHM', 'SAFETY', 'MOTION',
    'NAVIGATION', 'NEOPIXEL', 'INTERCOM', 'TERMINAL',
    'ACTUATORS', 'LOCALISATION', 'VISION', 'JETSON',
]

@app.route('/api/hw/modules', methods=['GET'])
def api_hw_modules():
    """Return known service list + current telemetry mask."""
    return jsonify({
        'services':  _HW_SERVICES,
        'tel_mask':  dict(_hw_tel_mask),
        'connected': _hw_transport is not None and _hw_transport.is_connected,
    })

@app.route('/api/jetson', methods=['GET', 'POST'])
def api_jetson():
    """Get or update Jetson edge-computer configuration."""
    global _jetson_config
    if request.method == 'POST':
        data = request.get_json(force=True) or {}
        if 'ip'   in data: _jetson_config['ip']   = str(data['ip'])
        if 'user' in data: _jetson_config['user'] = str(data['user'])
        if 'port' in data: _jetson_config['port'] = int(data['port'])
        return jsonify({'ok': True, 'config': dict(_jetson_config)})
    return jsonify(dict(_jetson_config))

@app.route('/api/hw/cmd', methods=['POST'])
def api_hw_cmd():
    """Fire a command to the connected hardware (fire-and-forget)."""
    if _hw_transport is None or not _hw_transport.is_connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    cmd  = (data.get('cmd') or '').strip()
    if not cmd:
        return jsonify({'ok': False, 'error': 'empty cmd'}), 400
    try:
        _hw_transport.fire(cmd)
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/exec', methods=['POST'])
def api_exec():
    """Execute a command via the active transport (sim or hw), wait for reply.
    For motion commands this blocks until motion is complete (or timeout).
    """
    data       = request.get_json(force=True) or {}
    cmd        = (data.get('cmd') or '').strip()
    timeout_ms = int(data.get('timeout_ms', 35000))
    if not cmd:
        return jsonify({'ok': False, 'res': 'empty cmd'}), 400
    try:
        t = _active_transport()
        ok, res = t.execute(cmd, timeout_ms=timeout_ms)
        return jsonify({'ok': ok, 'res': res})
    except Exception as e:
        return jsonify({'ok': False, 'res': str(e)}), 500


_go_lock = threading.Lock()

@app.route('/api/go', methods=['POST'])
def api_go():
    """Move to (x, y) via the active brain's motion service (uses pathfinder if enabled).
    Optional 'theta' parameter (degrees, table frame 0=East): if provided, a goAlign
    command is issued so the robot reaches the target with that final heading.

    A threading lock prevents concurrent motion requests from cascading.
    If a go() is already in progress, the new request cancels the running
    motion (via fire('cancel')), waits for the lock, then executes."""
    data = request.get_json(force=True) or {}
    try:
        x = float(data['x'])
        y = float(data['y'])
    except (KeyError, ValueError) as e:
        return jsonify({'ok': False, 'res': f'bad params: {e}'}), 400

    # If a motion is already in progress, cancel it so we don't queue up
    if _go_lock.locked():
        try:
            b = _active_brain()
            b.motion.cancel()
        except Exception:
            pass

    try:
        with _go_lock:
            b = _active_brain()
            theta = data.get('theta', None)
            if theta is not None:
                b.motion.go_heading(x, y, float(theta))
            else:
                b.motion.go(x, y)
            ok = b.motion.was_successful()
            return jsonify({'ok': ok, 'res': 'ok' if ok else 'failed'})
    except Exception as e:
        return jsonify({'ok': False, 'res': str(e)}), 500


@app.route('/api/go_traj', methods=['POST'])
def api_go_traj():
    """Execute a list of waypoints sequentially, each using the motion service (pathfinder enabled)."""
    data = request.get_json(force=True) or {}
    waypoints = data.get('waypoints', [])
    if not waypoints:
        return jsonify({'ok': False, 'res': 'no waypoints'}), 400

    if _go_lock.locked():
        try:
            _active_brain().motion.cancel()
        except Exception:
            pass

    try:
        with _go_lock:
            b = _active_brain()
            for i, wp in enumerate(waypoints):
                b.motion.go(float(wp['x']), float(wp['y']))
                if not b.motion.was_successful():
                    return jsonify({'ok': False, 'res': f'waypoint {i+1} failed'})
            return jsonify({'ok': True, 'res': 'ok'})
    except Exception as e:
        return jsonify({'ok': False, 'res': str(e)}), 500


def _active_brain():
    """Return the brain wired to the active transport (hw if connected, else sim)."""
    if _hw_brain is not None and _hw_transport is not None and _hw_transport.is_connected:
        return _hw_brain
    return brain


# ── Actuator sequences API ────────────────────────────────────────────────────

_ACT_DIR = os.path.join(os.path.dirname(__file__), 'actuator_data')
os.makedirs(_ACT_DIR, exist_ok=True)

@app.route('/api/actuator/poses', methods=['GET'])
def api_act_poses_get():
    path = os.path.join(_ACT_DIR, 'poses.json')
    if os.path.exists(path):
        with open(path) as f:
            return jsonify(json.load(f))
    return jsonify({})

@app.route('/api/actuator/poses', methods=['POST'])
def api_act_poses_set():
    data = request.get_json(force=True) or {}
    path = os.path.join(_ACT_DIR, 'poses.json')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    return jsonify({'ok': True})

@app.route('/api/actuator/sequences', methods=['GET'])
def api_act_seq_list():
    seqs = {}
    for fp in glob.glob(os.path.join(_ACT_DIR, 'seq_*.json')):
        name = os.path.basename(fp)[4:-5]  # strip seq_ and .json
        with open(fp) as f:
            seqs[name] = json.load(f)
    return jsonify(seqs)

@app.route('/api/actuator/sequences/<name>', methods=['GET'])
def api_act_seq_get(name):
    path = os.path.join(_ACT_DIR, f'seq_{name}.json')
    if not os.path.exists(path):
        return jsonify({'ok': False, 'error': 'not found'}), 404
    with open(path) as f:
        return jsonify(json.load(f))

@app.route('/api/actuator/sequences/<name>', methods=['PUT'])
def api_act_seq_put(name):
    data = request.get_json(force=True) or {}
    path = os.path.join(_ACT_DIR, f'seq_{name}.json')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    return jsonify({'ok': True})

@app.route('/api/actuator/sequences/<name>', methods=['DELETE'])
def api_act_seq_del(name):
    path = os.path.join(_ACT_DIR, f'seq_{name}.json')
    if os.path.exists(path):
        os.remove(path)
    return jsonify({'ok': True})


# ── Settings API ──────────────────────────────────────────────────────────────

@app.route('/api/settings', methods=['GET'])
def api_settings_get():
    """Return all holOS-side settings."""
    return jsonify({'ok': True, 'settings': settings_store.all()})


@app.route('/api/settings', methods=['POST'])
def api_settings_set():
    """
    Set a config key-value pair.  Saves holOS-side and pushes to firmware.
    Returns a 'calibration_warning' flag if the key is a calibration value.
    """
    data = request.get_json(force=True)
    key   = data.get('key', '').strip()
    value = str(data.get('value', '')).strip()
    if not key:
        return jsonify({'ok': False, 'error': 'missing key'}), 400

    # Save holOS-side
    settings_store.set(key, value)

    # Push to firmware if connected
    pushed = False
    if _hw_transport and _hw_transport.is_connected:
        try:
            ok, _ = _hw_transport.execute(f"cfg_set({key},{value})", timeout_ms=2000)
            pushed = ok
        except Exception:
            pass

    # Calibration change warning
    is_calib = settings_store.is_calibration_key(key)
    resp = {'ok': True, 'pushed': pushed, 'calibration_warning': is_calib}
    if is_calib:
        resp['warning'] = (
            "Valeur de calibration modifiée ! "
            "Pensez à sauvegarder via 'Save Settings' pour que les valeurs "
            "soient restaurées automatiquement au prochain démarrage."
        )
    return jsonify(resp)


@app.route('/api/settings/pull', methods=['POST'])
def api_settings_pull():
    """Pull current config from firmware into holOS settings store."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    ok = settings_store.pull_from_firmware(_hw_transport)
    return jsonify({'ok': ok, 'settings': settings_store.all()})


@app.route('/api/settings/push', methods=['POST'])
def api_settings_push():
    """Push all holOS settings to firmware."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    n = settings_store.push_all(_hw_transport)
    return jsonify({'ok': True, 'pushed': n})


# ── Settings push helper ──────────────────────────────────────────────────────

def _push_settings_to_firmware(t):
    """Push all holOS-side settings to firmware via cfg_set on connect."""
    if settings_store.count() == 0:
        return
    def _push():
        try:
            n = settings_store.push_all(t)
            brain.log(f"[Settings] Pushed {n} settings to firmware")
        except Exception as e:
            print(f"  [Settings] Push error: {e}")
    # Run in background so we don't block the connect handler
    threading.Thread(target=_push, daemon=True).start()


# ── Calibration API ───────────────────────────────────────────────────────────

def _calib_connected():
    """True if a hardware robot is connected and can receive calib commands."""
    return _hw_transport is not None and _hw_transport.is_connected

def _calib_fire(cmd: str):
    """Send a calibration command to the robot (fire-and-forget)."""
    if _calib_connected():
        try:
            _hw_transport.fire(cmd)
        except Exception as e:
            print(f"[CALIB] fire error: {e}")

@app.route('/api/calibration', methods=['GET'])
def api_calib_get():
    """Return the current Python-side calibration cache."""
    return jsonify({'ok': True, 'calib': dict(_calib), 'connected': _calib_connected()})

@app.route('/api/calibration', methods=['POST'])
def api_calib_set():
    """Update one or more calibration values and forward commands to the robot.

    Accepted fields (all optional, send only what changed):
      cx, cy, cr  → fires  calib_cart(cx, cy, cr)
      ha, hb, hc  → fires  calib_holo(ha, hb, hc)
      ol          → fires  calib_otos_linear(ol)
      oa          → fires  calib_otos_angular(oa)
    """
    global _calib
    data = request.get_json(force=True) or {}

    # Merge valid float fields
    for k in ('cx','cy','cr','ha','hb','hc','ol','oa'):
        if k in data:
            try:
                _calib[k] = float(data[k])
            except (TypeError, ValueError):
                return jsonify({'ok': False, 'error': f'invalid value for {k}'}), 400

    # Determine which groups were changed and fire the appropriate commands
    changed = set(data.keys()) & {'cx','cy','cr','ha','hb','hc','ol','oa'}
    if changed & {'cx','cy','cr'}:
        _calib_fire(f"calib_cart({_calib['cx']},{_calib['cy']},{_calib['cr']})")
    if changed & {'ha','hb','hc'}:
        _calib_fire(f"calib_holo({_calib['ha']},{_calib['hb']},{_calib['hc']})")
    if 'ol' in changed:
        _calib_fire(f"calib_otos_linear({_calib['ol']})")
    if 'oa' in changed:
        _calib_fire(f"calib_otos_angular({_calib['oa']})")

    return jsonify({'ok': True, 'calib': dict(_calib)})

@app.route('/api/calibration/save', methods=['POST'])
def api_calib_save():
    """Save calibration values holOS-side (settings.json)."""
    # Pull current config from firmware and persist locally
    if _calib_connected():
        settings_store.pull_from_firmware(_hw_transport)
    return jsonify({'ok': True, 'msg': 'Calibration saved holOS-side'})

@app.route('/api/calibration/load', methods=['POST'])
def api_calib_load():
    """Push saved settings from holOS to firmware and refresh local cache."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    try:
        # Push all saved settings to firmware (includes servo limits etc.)
        n = settings_store.push_all(_hw_transport)
        return jsonify({'ok': True, 'pushed': n, 'calib': dict(_calib)})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500

@app.route('/api/calibration/reset', methods=['POST'])
def api_calib_reset():
    """Reset calibration to firmware defaults."""
    global _calib
    _calib_fire('calib_reset')
    _calib = dict(_CALIB_DEFAULTS)
    return jsonify({'ok': True, 'calib': dict(_calib)})

def _calib_error_from_raw(res: str) -> str:
    """Extract a human-readable error from a 'kind=error msg=...' firmware reply."""
    import re
    if not res:
        return 'empty reply'
    m = re.search(r'msg=([^\s]+)', res)
    return f'firmware: {m.group(1)}' if m else f'firmware: {res!r}'


def _parse_calib_report(text: str) -> dict:
    """Extract key=value pairs from a firmware 'Calib: ...' report line.

    Example payloads:
      'kind=move cmd=500.00 axis=x dx=487.10 dy=2.30 dth=0.021 od=487.15'
      'kind=turn cmd_deg=90.0 dth_rad=1.547 od_deg=88.65'
      'kind=error msg=motion_busy'

    Values can be floats (numeric keys) or bare tokens like 'move', 'turn',
    'error', 'x', 'y', 'motion_busy'. We capture any non-whitespace run, then
    opportunistically convert to float — non-numeric values stay as strings.
    """
    import re
    out = {}
    if not text:
        return out
    for m in re.finditer(r'([a-z_]+)=(\S+)', text):
        k, v = m.group(1), m.group(2)
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


@app.route('/api/calibration/open_move', methods=['POST'])
def api_calib_open_move():
    """Open-loop linear calibration move on X or Y axis.

    Body: { dist_mm: float, axis: 'x'|'y' }
    Returns: { ok, commanded, axis, otos_dx, otos_dy, otos_dth, otos_dist, raw }
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    try:
        dist = float(data.get('dist_mm', 500))
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'invalid dist_mm'}), 400
    axis = str(data.get('axis', 'x')).lower()

    if dist < 50 or dist > 1500:
        return jsonify({'ok': False, 'error': 'dist_mm must be 50–1500'}), 400
    if axis not in ('x', 'y'):
        return jsonify({'ok': False, 'error': "axis must be 'x' or 'y'"}), 400

    try:
        # Firmware has its own 20 s fuse; give the transport a bit of slack.
        # axis_id is numeric (0=X, 1=Y): the firmware interpreter evaluates
        # every argument through Expression, so a bare 'x'/'y' would be
        # mis-parsed as an empty variable lookup.
        axis_id = 0 if axis == 'x' else 1
        # execute_calib: short ACK wait + long wait on 'T:cal' telemetry.
        # Hard timeout ensures the wizard can recover even if the move hangs.
        ok, res = _hw_transport.execute_calib(
            f'calib_move_open({dist},{axis_id})',
            ack_timeout_ms=3000, calib_timeout_ms=35000)
        parsed = _parse_calib_report(res)
        kind = parsed.get('kind')
        if not ok:
            return jsonify({'ok': False,
                            'error': f'transport error (no reply in 30s) — raw={res!r}',
                            'raw': res})
        if kind == 'error':
            return jsonify({'ok': False, 'error': _calib_error_from_raw(res), 'raw': res})
        if kind != 'move':
            return jsonify({'ok': False,
                            'error': f'unexpected reply (kind={kind!r}) — raw={res!r}',
                            'raw': res})
        return jsonify({
            'ok':        True,
            'commanded': parsed.get('cmd', dist),
            'axis':      parsed.get('axis', axis),
            'otos_dx':   parsed.get('dx', 0.0),
            'otos_dy':   parsed.get('dy', 0.0),
            'otos_dth':  parsed.get('dth', 0.0),
            'otos_dist': parsed.get('od', 0.0),
            'raw':       res,
        })
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/calibration/open_turn', methods=['POST'])
def api_calib_open_turn():
    """Open-loop rotation calibration move.

    Body: { angle_deg: float }
    Returns: { ok, commanded_deg, otos_dth_rad, otos_dth_deg, raw }
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    try:
        angle = float(data.get('angle_deg', 90))
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'invalid angle_deg'}), 400

    if abs(angle) < 5 or abs(angle) > 720:
        return jsonify({'ok': False, 'error': 'angle_deg must be 5–720 (absolute)'}), 400

    try:
        ok, res = _hw_transport.execute_calib(
            f'calib_turn_open({angle})',
            ack_timeout_ms=3000, calib_timeout_ms=35000)
        parsed = _parse_calib_report(res)
        kind = parsed.get('kind')
        if not ok:
            return jsonify({'ok': False,
                            'error': f'transport error (no reply in 30s) — raw={res!r}',
                            'raw': res})
        if kind == 'error':
            return jsonify({'ok': False, 'error': _calib_error_from_raw(res), 'raw': res})
        if kind != 'turn':
            return jsonify({'ok': False,
                            'error': f'unexpected reply (kind={kind!r}) — raw={res!r}',
                            'raw': res})
        return jsonify({
            'ok':            True,
            'commanded_deg': parsed.get('cmd_deg', angle),
            'otos_dth_rad':  parsed.get('dth_rad', 0.0),
            'otos_dth_deg':  parsed.get('od_deg', 0.0),
            'raw':           res,
        })
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/calibration/probe', methods=['POST'])
def api_calib_probe():
    """Wall probe (recalage) — move robot to a wall and correct position.

    Body: { wall: 'NORTH'|'SOUTH'|'EAST'|'WEST', face: 'A'|'AB'|'B'|'BC'|'C'|'CA',
            clearance?: float (mm, default 100) }
    Returns: { ok, wall, face, x, y, theta, raw }

    The robot aligns the specified face toward the wall, approaches slowly,
    detects contact via stall, corrects its absolute position to the known
    wall coordinate, then backs off by `clearance` mm.
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    wall = str(data.get('wall', '')).upper()
    face = str(data.get('face', '')).upper()
    clearance = float(data.get('clearance', 100))

    if wall not in ('NORTH', 'SOUTH', 'EAST', 'WEST'):
        return jsonify({'ok': False, 'error': f"wall must be NORTH/SOUTH/EAST/WEST, got '{wall}'"}), 400
    if face not in ('A', 'AB', 'B', 'BC', 'C', 'CA'):
        return jsonify({'ok': False, 'error': f"face must be A/AB/B/BC/C/CA, got '{face}'"}), 400

    try:
        ok, res = _hw_transport.execute_calib(
            f'probe_open({wall},{face},{clearance:.0f})',
            ack_timeout_ms=3000, calib_timeout_ms=45000)
        parsed = _parse_calib_report(res)
        kind = parsed.get('kind')
        if not ok:
            return jsonify({'ok': False,
                            'error': f'transport error — raw={res!r}',
                            'raw': res})
        if kind == 'error':
            return jsonify({'ok': False, 'error': _calib_error_from_raw(res), 'raw': res})
        if kind != 'probe':
            return jsonify({'ok': False,
                            'error': f'unexpected reply (kind={kind!r}) — raw={res!r}',
                            'raw': res})
        return jsonify({
            'ok':    True,
            'wall':  parsed.get('wall', wall),
            'face':  parsed.get('face', face),
            'x':     parsed.get('x', 0.0),
            'y':     parsed.get('y', 0.0),
            'theta': parsed.get('theta', 0.0),
            'raw':   res,
        })
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/calibration/compute', methods=['POST'])
def api_calib_compute():
    """Stateless: compute suggested calibration values from a measured move.

    Does NOT apply any values — the UI then calls POST /api/calibration with
    the ones to apply.

    Body:
      { axis: 'x'|'y'|'theta',
        commanded:     float,   # what was asked (mm for x/y, deg for theta)
        otos_measured: float,   # what OTOS reported (mm or deg, same unit as commanded)
        actual:        float }  # what the user measured with ruler/protractor

    Math:
      motion scale:  new = old * (commanded / actual)
      OTOS scale:    new = old * (actual    / otos_measured)
    """
    data = request.get_json(force=True) or {}
    try:
        axis = str(data.get('axis', 'x')).lower()
        commanded     = float(data['commanded'])
        otos_measured = float(data['otos_measured'])
        actual        = float(data['actual'])
    except (KeyError, TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'missing/invalid fields'}), 400

    if axis not in ('x', 'y', 'theta'):
        return jsonify({'ok': False, 'error': "axis must be 'x','y','theta'"}), 400
    if commanded <= 0:
        return jsonify({'ok': False, 'error': 'commanded must be > 0'}), 400
    if actual <= 0:
        return jsonify({'ok': False, 'error': 'actual must be > 0'}), 400
    if abs(otos_measured) < 10:
        return jsonify({'ok': False,
                        'error': 'otos_measured too small (<10) — OTOS likely off'}), 400

    rel_err = abs(commanded - actual) / commanded
    if rel_err > 0.5:
        return jsonify({
            'ok': False,
            'error': f'|commanded-actual| is {rel_err*100:.0f}% of commanded — refused (likely typo)'
        }), 400

    motion_ratio = commanded / actual             # >1 → undershot → increase scale
    otos_ratio   = actual / abs(otos_measured)    # >1 → OTOS undershot → increase

    warnings = []
    def _check(name, old, new):
        if old == 0:
            warnings.append(f'{name}: current value is 0')
            return
        r = new / old
        if r < 0.5 or r > 1.5:
            warnings.append(f'{name}: correction factor {r:.3f} outside [0.5, 1.5]')

    current = {}
    suggested = {}

    if axis == 'x':
        old_cx = _calib['cx']; new_cx = old_cx * motion_ratio
        old_ol = _calib['ol']; new_ol = old_ol * otos_ratio
        _check('cx', old_cx, new_cx)
        _check('ol', old_ol, new_ol)
        current   = {'cx': old_cx, 'ol': old_ol}
        suggested = {'cx': round(new_cx, 6), 'ol': round(new_ol, 6)}

    elif axis == 'y':
        old_cy = _calib['cy']; new_cy = old_cy * motion_ratio
        old_ol = _calib['ol']; new_ol = old_ol * otos_ratio
        _check('cy', old_cy, new_cy)
        _check('ol', old_ol, new_ol)
        current   = {'cy': old_cy, 'ol': old_ol}
        suggested = {'cy': round(new_cy, 6), 'ol': round(new_ol, 6)}

    else:  # theta
        old_cr = _calib['cr']; new_cr = old_cr * motion_ratio
        old_oa = _calib['oa']; new_oa = old_oa * otos_ratio
        _check('cr', old_cr, new_cr)
        _check('oa', old_oa, new_oa)
        current   = {'cr': old_cr, 'oa': old_oa}
        suggested = {'cr': round(new_cr, 6), 'oa': round(new_oa, 6)}

    return jsonify({
        'ok':            True,
        'axis':          axis,
        'commanded':     commanded,
        'otos_measured': otos_measured,
        'actual':        actual,
        'motion_ratio':  round(motion_ratio, 6),
        'otos_ratio':    round(otos_ratio,   6),
        'current':       current,
        'suggested':     suggested,
        'warnings':      warnings,
    })

def _try_parse_calib_response(text: str):
    """Parse a key=value calibration string and update _calib cache."""
    global _calib
    if not text:
        return
    import re
    pairs = re.findall(r'([a-z]+)=([-\d.]+)', text)
    updated = False
    for k, v in pairs:
        if k in _calib:
            try:
                _calib[k] = float(v)
                updated = True
            except ValueError:
                pass
    if updated:
        socketio.emit('calib_updated', {'calib': dict(_calib)})


# ── Strategy file API ──────────────────────────────────────────────────────────

def _safe_name(name):
    return name and '..' not in name and '/' not in name and '\\' not in name

@app.route('/api/strategies', methods=['GET'])
def api_strategies():
    files = sorted(glob.glob(os.path.join(STRATEGY_DIR, '*.py')))
    return jsonify([os.path.basename(f) for f in files])

@app.route('/api/strategies', methods=['POST'])
def api_strategies_create():
    data = request.get_json() or {}
    name = (data.get('name') or 'strategy.py').strip()
    if not name.endswith('.py'): name += '.py'
    if not _safe_name(name):
        return jsonify({'error': 'invalid name'}), 400
    path = os.path.join(STRATEGY_DIR, name)
    if not os.path.exists(path):
        tpl = (
            f'"""\n{name} — holOS strategy\n"""\n'
            'import math\n'
            'from shared.config import POI, Team, Vec2\n'
            'from shared.mission import Mission, MissionMode\n\n\n'
            'def run_mission(brain):\n'
            '    """Strategy entry point. Called by Brain.\n'
            '    Use brain.motion, brain.log, brain.vision ...\n'
            '    """\n'
            '    brain.log("Strategy started")\n'
        )
        with open(path, 'w', encoding='utf-8') as f:
            f.write(tpl)
    return jsonify({'ok': True, 'name': name})

@app.route('/api/strategy/<name>', methods=['GET'])
def api_strategy_get(name):
    if not _safe_name(name):
        return jsonify({'error': 'invalid'}), 400
    path = os.path.join(STRATEGY_DIR, name)
    if not os.path.isfile(path):
        return jsonify({'error': 'not found'}), 404
    with open(path, 'r', encoding='utf-8') as f:
        return jsonify({'name': name, 'content': f.read()})

@app.route('/api/strategy/<name>', methods=['PUT'])
def api_strategy_put(name):
    if not _safe_name(name):
        return jsonify({'error': 'invalid'}), 400
    data = request.get_json() or {}
    path = os.path.join(STRATEGY_DIR, name)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(data.get('content', ''))
    return jsonify({'ok': True})

@app.route('/api/strategy/<name>', methods=['DELETE'])
def api_strategy_delete(name):
    if not _safe_name(name) or name == 'match.py':
        return jsonify({'error': 'cannot delete'}), 400
    path = os.path.join(STRATEGY_DIR, name)
    if os.path.isfile(path): os.remove(path)
    return jsonify({'ok': True})

@app.route('/api/strategy/<name>/rename', methods=['POST'])
def api_strategy_rename(name):
    if not _safe_name(name):
        return jsonify({'error': 'invalid'}), 400
    data = request.get_json() or {}
    new_name = (data.get('to') or '').strip()
    if not new_name.endswith('.py'): new_name += '.py'
    if not _safe_name(new_name):
        return jsonify({'error': 'invalid new name'}), 400
    old_path = os.path.join(STRATEGY_DIR, name)
    new_path = os.path.join(STRATEGY_DIR, new_name)
    if os.path.isfile(old_path): os.rename(old_path, new_path)
    return jsonify({'ok': True, 'name': new_name})


# ── Macros API ─────────────────────────────────────────────────────────────────

@app.route('/api/macros', methods=['GET'])
def api_macros_get():
    if os.path.isfile(MACROS_FILE):
        with open(MACROS_FILE, 'r', encoding='utf-8') as f:
            try: return jsonify(json.load(f))
            except: pass
    return jsonify([])

@app.route('/api/macros', methods=['PUT'])
def api_macros_put():
    data = request.get_json()
    if isinstance(data, list):
        with open(MACROS_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
    return jsonify({'ok': True})


# ── Missions API ───────────────────────────────────────────────────────────────
# missions.json format:
# [{id, name, desc, approach:{x,y,angle}, macro_id, score, priority, time_ms, enabled}]

def _load_missions():
    if os.path.isfile(MISSIONS_FILE):
        with open(MISSIONS_FILE, 'r', encoding='utf-8') as f:
            try: return json.load(f)
            except: pass
    return []

def _save_missions(data: list):
    with open(MISSIONS_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

@app.route('/api/missions', methods=['GET'])
def api_missions_get():
    return jsonify(_load_missions())

@app.route('/api/missions', methods=['PUT'])
def api_missions_put():
    data = request.get_json()
    if isinstance(data, list):
        _save_missions(data)
    return jsonify({'ok': True})


def _generate_fallback_cfg(missions: list, macros: list) -> str:
    """
    Generate a plain-text fallback strategy for the Teensy SD card.
    Format: one firmware command per line, sections separated by comments.
    The firmware executes these via the OS command interpreter on Jetson disconnect.

    Macro step → firmware command mapping:
        move_to {x,y}         → go(x,y)
        face {angle_deg}      → turn(angle_deg)
        actuator {cmd,wait_ms} → <cmd>
        wait {ms}             → delay(ms)    (if firmware supports it)
        call_macro {name}     → (inlined)
        log {msg}             → (skipped in fallback)
        if_occupied           → (skipped in fallback — no lidar-based skip)
    """
    macro_by_name = {m['name']: m for m in macros}

    def steps_to_cmds(steps: list, depth: int = 0) -> list:
        """Recursively resolve macro steps to firmware command strings."""
        if depth > 5:  # guard against circular references
            return []
        cmds = []
        for step in steps:
            t = step.get('type', '')
            if t == 'move_to':
                cmds.append(f"go({int(step.get('x',0))},{int(step.get('y',0))})")
            elif t == 'face':
                cmds.append(f"turn({step.get('angle_deg',0):.1f})")
            elif t == 'actuator':
                cmd = step.get('cmd', '').strip()
                if cmd:
                    cmds.append(cmd)
                wait = int(step.get('wait_ms', 0))
                if wait > 0:
                    cmds.append(f"delay({wait})")
            elif t == 'wait':
                ms = int(step.get('ms', 0))
                if ms > 0:
                    cmds.append(f"delay({ms})")
            elif t == 'call_macro':
                ref = macro_by_name.get(step.get('name', ''))
                if ref:
                    cmds += steps_to_cmds(ref.get('steps', []), depth + 1)
            # log / if_occupied → skipped in fallback
        return cmds

    lines = [
        '# holOS Mission Fallback Strategy',
        f'# Generated by holOS — {len(missions)} missions',
        '# One firmware command per line.',
        '# Edit carefully — no conditions, no branching.',
        '',
    ]

    enabled = [m for m in missions if m.get('enabled', True)]
    enabled.sort(key=lambda m: -m.get('priority', 0))

    for mission in enabled:
        name   = mission.get('name', '?')
        score  = mission.get('score', 0)
        prio   = mission.get('priority', 0)
        approach = mission.get('approach', {})
        macro_id = mission.get('macro_id', '')

        lines.append(f'# === Mission: {name} (score={score}, priority={prio}) ===')

        # Approach move
        ax = approach.get('x')
        ay = approach.get('y')
        aa = approach.get('angle')
        if ax is not None and ay is not None:
            lines.append(f'go({int(ax)},{int(ay)})')
        if aa is not None:
            lines.append(f'turn({float(aa):.1f})')

        # Macro steps
        macro = macro_by_name.get(macro_id)
        if macro:
            cmds = steps_to_cmds(macro.get('steps', []))
            lines += cmds
        else:
            lines.append(f'# (no macro: {macro_id!r})')

        lines.append('')

    lines.append('# === End of strategy ===')
    return '\n'.join(lines) + '\n'


@app.route('/api/missions/deploy-firmware', methods=['POST'])
@app.route('/api/missions/deploy-sd', methods=['POST'])  # legacy alias
def api_missions_deploy_firmware():
    """
    Generate the fallback strategy and push it to firmware memory.
    Writes to firmware in-memory buffer via mission_sd_open/line/close protocol.
    Returns the generated text so the UI can preview it.
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'Robot non connecté'}), 503

    missions = _load_missions()
    macros   = _load_missions_macros()
    cfg      = _generate_fallback_cfg(missions, macros)

    # Write to firmware in-memory via the legacy protocol (SD removed, now RAM-based)
    try:
        ok1, _ = _hw_transport.execute('mission_sd_open', timeout_ms=3000)
        if not ok1:
            return jsonify({'ok': False, 'error': 'mission_sd_open failed'}), 500
        for line in cfg.splitlines():
            escaped = line.replace('"', '\\"')
            ok2, _ = _hw_transport.execute(f'mission_sd_line {escaped}', timeout_ms=2000)
            if not ok2:
                _hw_transport.execute('mission_sd_close', timeout_ms=2000)
                return jsonify({'ok': False, 'error': f'Write failed at: {line}'}), 500
        ok3, _ = _hw_transport.execute('mission_sd_close', timeout_ms=3000)
        if not ok3:
            return jsonify({'ok': False, 'error': 'mission_sd_close failed'}), 500
        return jsonify({'ok': True, 'cfg': cfg, 'lines': len(cfg.splitlines())})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/missions/preview-fallback', methods=['POST'])
def api_missions_preview_fallback():
    """Generate and return the fallback .cfg without writing to SD."""
    missions = _load_missions()
    macros   = _load_missions_macros()
    cfg      = _generate_fallback_cfg(missions, macros)
    return jsonify({'ok': True, 'cfg': cfg})


def _load_missions_macros():
    """Load macros from macros.json (shared helper)."""
    if os.path.isfile(MACROS_FILE):
        with open(MACROS_FILE, 'r', encoding='utf-8') as f:
            try: return json.load(f)
            except: pass
    return []


@app.route('/api/missions/run-step', methods=['POST'])
def api_missions_run_step():
    """
    Execute a single macro step on the connected robot.
    Body: {type, ...step_fields}
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'Robot non connecté'}), 503
    step = request.get_json() or {}
    t = step.get('type', '')
    cmd = None
    if t == 'move_to':
        cmd = f"go({int(step.get('x',0))},{int(step.get('y',0))})"
    elif t == 'face':
        cmd = f"turn({step.get('angle_deg',0):.1f})"
    elif t == 'actuator':
        cmd = step.get('cmd', '').strip()
    elif t == 'wait':
        # Just delay on server side — firmware has no 'wait' command per se
        ms = int(step.get('ms', 0))
        time.sleep(ms / 1000.0)
        return jsonify({'ok': True, 'cmd': f'delay({ms}ms) — done locally'})
    if not cmd:
        return jsonify({'ok': False, 'error': f'No firmware command for step type {t!r}'}), 400
    ok, res = _hw_transport.execute(cmd, timeout_ms=15000)
    return jsonify({'ok': ok, 'cmd': cmd, 'response': res})


# ── Vision API ────────────────────────────────────────────────────────────────

@app.route('/api/vision/state')
def api_vision_state():
    if _vision is None:
        return jsonify({'cv2_available': False, 'enabled': False})
    return jsonify(_vision.get_state())


@app.route('/api/vision/enable', methods=['POST'])
def api_vision_enable():
    if _vision is None:
        return jsonify({'ok': False, 'error': 'Vision backend not available (opencv missing?)'}), 503
    ok = _vision.enable()
    return jsonify({'ok': ok})


@app.route('/api/vision/disable', methods=['POST'])
def api_vision_disable():
    if _vision:
        _vision.disable()
    return jsonify({'ok': True})


@app.route('/api/vision/source', methods=['POST'])
def api_vision_source():
    if _vision is None:
        return jsonify({'ok': False, 'error': 'Vision backend not available'}), 503
    data   = request.get_json(force=True) or {}
    source = str(data.get('source', '0'))
    ok     = _vision.set_source(source)
    return jsonify({'ok': ok, 'source': source})


@app.route('/api/vision/config', methods=['GET'])
def api_vision_config_get():
    if _vision is None:
        return jsonify({})
    return jsonify(_vision.get_config())


@app.route('/api/vision/config', methods=['POST'])
def api_vision_config_set():
    if _vision is None:
        return jsonify({'ok': False}), 503
    cfg = request.get_json(force=True) or {}
    _vision.set_config(cfg)
    return jsonify({'ok': True})


@app.route('/api/vision/calibration', methods=['POST'])
def api_vision_calibration_set():
    if _vision is None:
        return jsonify({'ok': False}), 503
    data = request.get_json(force=True) or {}
    _vision.set_calibration_dict(data.get('calibration'))
    return jsonify({'ok': True})


@app.route('/api/vision/playback', methods=['POST'])
def api_vision_playback():
    if _vision is None:
        return jsonify({'ok': False}), 503
    data = request.get_json(force=True) or {}
    action = data.get('action', '')
    if action == 'play_pause':
        _vision.play_pause()
    elif action == 'seek':
        _vision.seek(int(data.get('frame', 0)))
    elif action == 'step':
        _vision.step(int(data.get('delta', 1)))
    elif action == 'speed':
        _vision.set_speed(float(data.get('speed', 1.0)))
    return jsonify({'ok': True})


# ── Object registry API ───────────────────────────────────────────────────────

@app.route('/api/vision/objects', methods=['GET'])
def api_vision_objects_get():
    if _vision is None:
        return jsonify([])
    return jsonify(_vision.get_objects())


@app.route('/api/vision/objects', methods=['PUT'])
def api_vision_objects_put():
    if _vision is None:
        return jsonify({'ok': False}), 503
    objects = request.get_json(force=True) or []
    _vision.set_objects(objects)
    return jsonify({'ok': True})


@app.route('/api/vision/objects/<obj_id>', methods=['PATCH'])
def api_vision_object_patch(obj_id):
    if _vision is None:
        return jsonify({'ok': False}), 503
    patch = request.get_json(force=True) or {}
    _vision.update_object(obj_id, patch)
    return jsonify({'ok': True})


@app.route('/api/vision/objects/<obj_id>', methods=['DELETE'])
def api_vision_object_delete(obj_id):
    if _vision is None:
        return jsonify({'ok': False}), 503
    _vision.delete_object(obj_id)
    return jsonify({'ok': True})


@app.route('/api/vision/objects/<obj_id>/color', methods=['POST'])
def api_vision_object_color(obj_id):
    """Query endpoint: robot/strategy asks for color of a specific object."""
    if _vision is None:
        return jsonify({'color': 'unknown'})
    color = _vision.get_object_color(obj_id)
    return jsonify({'id': obj_id, 'color': color})


# ── Vision frame push loop (25 fps max, SocketIO) ────────────────────────────

_vision_clients = 0   # count of clients with vision view open

def _vision_push_loop():
    """Push vision frames to all connected clients at up to 25 fps."""
    import base64 as _b64
    while True:
        try:
            if _vision and _vision.enabled and _vision_clients > 0:
                raw_b64, rect_b64, proc_b64 = _vision.get_latest_frames_b64()
                if raw_b64:
                    socketio.emit('vision_frame', {
                        'raw':  raw_b64,
                        'rect': rect_b64,
                        'proc': proc_b64,
                        **_vision.get_frame_info(),
                        'detections': _vision.get_state().get('detections', []),
                        'has_h': _vision.get_state().get('has_homography', False),
                        'h_fresh': _vision.get_state().get('homography_fresh', False),
                        'proc_mode': _vision.get_config().get('proc_mode', 'none'),
                    })
        except Exception:
            pass
        time.sleep(1 / 25)


@socketio.on('vision_view_active')
def on_vision_view_active(data):
    global _vision_clients
    if data.get('active'):
        _vision_clients += 1
    else:
        _vision_clients = max(0, _vision_clients - 1)


# ── Serial ports API ───────────────────────────────────────────────────────────

@app.route('/api/serial/ports')
def api_serial_ports():
    import glob as _glob
    seen  = set()
    ports = []
    log   = []   # accumulated for UART tab broadcast

    def _add(device, desc, source):
        if device not in seen:
            seen.add(device)
            ports.append({'port': device, 'desc': desc})
            log.append(f'  + {device}  ({desc})  [{source}]')

    # On Jetson, always expose the onboard UART first.
    if IS_JETSON:
        _add(_ONBOARD_UART, _ONBOARD_UART_DESC, 'hardcoded')

    # pyserial enumeration (may return an empty list on Tegra/ARM kernels).
    try:
        import serial.tools.list_ports
        pl = list(serial.tools.list_ports.comports())
        log.append(f'  pyserial comports(): {len(pl)} result(s)')
        for p in pl:
            _add(p.device, p.description, 'pyserial')
    except Exception as e:
        log.append(f'  pyserial comports() ERROR: {e}')
        print(f"[ports] list_ports.comports() failed: {e}")

    # Direct /dev glob — fallback for kernels where pyserial sysfs walk misses devices.
    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyTHS*'):
        for dev in sorted(_glob.glob(pattern)):
            _add(dev, dev.split('/')[-1], 'glob')

    # Print full scan result to server log
    print(f"[ports] scan complete — {len(ports)} port(s):")
    for line in log:
        print(line)
    if not ports:
        print("[ports] WARNING: no ports found")

    # Broadcast to UART tab so the user can see results without looking at server logs
    socketio.emit('uart_raw', {'dir': 'sys', 'line': f'[port scan] {len(ports)} port(s) found'})
    for line in log:
        socketio.emit('uart_raw', {'dir': 'sys', 'line': line})

    return jsonify(ports)


# ── Static occupancy map API ──────────────────────────────────────────────────

@app.route('/api/occupancy/static', methods=['GET'])
def api_occ_static_get():
    """Return current static layer as cell list."""
    return jsonify({'cells': occupancy.static_to_list()})


@app.route('/api/occupancy/static', methods=['PUT'])
def api_occ_static_put():
    """Replace static layer and persist to JSON.

    If the body contains a 'cells' key the static map is replaced with those
    cells.  If no body / no 'cells' key is provided (e.g. the Save button just
    wants to persist the current in-memory map) the map is left untouched and
    only the save-to-disk step is performed.
    """
    data = request.get_json(force=True) or {}
    if 'cells' in data:
        occupancy.reset_static()
        for cell in data['cells']:
            occupancy.set_static_cell(int(cell['gx']), int(cell['gy']), True)
    occupancy.save_static(_STATIC_OCC_PATH)
    return jsonify({'ok': True})


@app.route('/api/occupancy/deploy', methods=['POST'])
def api_occ_deploy():
    """Deploy static map to T40 via T41.
    Sends 'setStaticMap(<hex>)' to T41 which relays it to T40 via intercom."""
    t = _active_transport()
    if not t.is_connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    hex_map = occupancy.to_static_hex()
    ok, res = t.execute(f'setStaticMap({hex_map})', timeout_ms=3000)
    return jsonify({'ok': ok, 'res': res})


# ── Path-planning toggle API ──────────────────────────────────────────────────

@app.route('/api/pathfinding', methods=['POST'])
def api_pathfinding():
    """Toggle pathfinding on/off.  Body: {'enabled': true|false}"""
    data = request.get_json(force=True) or {}
    enabled = bool(data.get('enabled', True))
    sim_state['features']['pathfinding'] = enabled
    # Apply to active brain (sim or hw)
    brain.motion.use_pathfinding = enabled
    if _hw_brain is not None:
        _hw_brain.motion.use_pathfinding = enabled
    return jsonify({'ok': True, 'pathfinding': enabled})


# ── Motion control mode (waypoint vs live pursuit) ───────────────────────────

@app.route('/api/motion_mode', methods=['POST'])
def api_motion_mode():
    """Toggle the motion control mode for the next go() call.

    Body: {'mode': 'waypoint' | 'pursuit'}
      waypoint → legacy chained-via behaviour (default)
      pursuit  → live target tracking with rolling carrot + replan

    Pursuit auto-reverts to waypoint after a single move (opt-in per click).
    """
    from services.motion import MotionMode
    data = request.get_json(force=True) or {}
    mode_str = str(data.get('mode', 'waypoint')).lower()
    mode = MotionMode.LIVE_PURSUIT if mode_str == 'pursuit' else MotionMode.LEGACY_WAYPOINT
    sim_state['features']['pursuit'] = (mode == MotionMode.LIVE_PURSUIT)
    brain.motion.set_mode(mode)
    if _hw_brain is not None:
        _hw_brain.motion.set_mode(mode)
    return jsonify({'ok': True, 'mode': mode.name})


# ── SocketIO events ───────────────────────────────────────────────────────────

@socketio.on('connect')
def on_connect():
    emit('state', _build_state())
    brain.log("Web client connected")


@socketio.on('field_click')
def on_field_click(data):
    x, y   = data['x'], data['y']
    button = data.get('button', 0)
    mode   = sim_state['mode']

    if button == 2:
        gx = int(x // GRID_CELL)
        gy = int(y // GRID_CELL)
        occupancy.toggle_cell(gx, gy)
        brain.log(f"Toggle cell ({gx},{gy})")
    elif mode == 'target':
        brain.motion._t._bridge._start_motion_xy(
            0, Vec2(x, y), lambda uid, ok, r: None
        )
        brain.log(f"Manual target → ({x:.0f}, {y:.0f})")


@socketio.on('set_color')
def on_set_color(data):
    poi_name   = data['name']
    color_name = data['color']
    color      = COLOR_BY_NAME.get(color_name, ObjectColor.UNKNOWN)
    brain.vision.set_color(poi_name, color)
    game_objs.set_color(poi_name, color)   # keep shared display state in sync
    if _hw_brain is not None:
        _hw_brain.vision.set_color(poi_name, color)
    brain.log(f"Color {poi_name} → {color_name}")


@socketio.on('run_strategy')
def on_run_strategy():
    global _match_running, _match_paused
    _match_running = True
    _match_paused  = False
    socketio.emit('match_state', {'running': True, 'paused': False})
    _active_brain().run_match(on_done=_on_match_done)


@socketio.on('stop_strategy')
def on_stop_strategy():
    global _match_running, _match_paused
    _active_brain().stop_match()
    _match_running = False
    _match_paused  = False
    socketio.emit('match_state', {'running': False, 'paused': False})


@socketio.on('reset')
def on_reset():
    global _match_running, _match_paused
    brain.stop_match()
    _match_running = False
    _match_paused  = False
    robot.reset_to_start(sim_state['team'])
    occupancy.reset_dynamic()   # Keep static layer across resets
    sim_state['score'] = 0
    brain.log("Simulation reset")


@socketio.on('set_team')
def on_set_team(data):
    team = data['team']
    sim_state['team'] = team
    robot.reset_to_start(team)
    brain.log(f"Team → {team}")


@socketio.on('set_mode')
def on_set_mode(data):
    sim_state['mode'] = data['mode']


@socketio.on('set_robot_pos')
def on_set_robot_pos(data):
    """Teleport robot to a given (x, y) position (and optionally theta in degrees)."""
    robot.pos = Vec2(float(data['x']), float(data['y']))
    theta_deg = float(data.get('theta', 0))
    if 'theta' in data:
        robot.theta = math.radians(theta_deg)

    t = _active_transport()
    if t.is_connected:
        brain.log(f"Teleporting to ({robot.pos.x:.0f}, {robot.pos.y:.0f}, {theta_deg:.1f}°)")
        ok, res = t.execute("setAbsPosition({},{},{})".format(robot.pos.x, robot.pos.y, theta_deg), timeout_ms=1000)
        if ok:
            brain.log(f"Robot teleported to ({robot.pos.x:.0f}, {robot.pos.y:.0f}, {theta_deg:.1f}°)")
        else:
            brain.log("Failed to teleport robot")


@socketio.on('paint_grid')
def on_paint_grid(data):
    """Set or clear a static occupancy-grid cell."""
    gx, gy = int(data['gx']), int(data['gy'])
    value = bool(data['value'])
    occupancy.set_static_cell(gx, gy, value)


@socketio.on('set_opponent_pos')
def on_set_opponent_pos(data):
    """Update the fake opponent's position (tracked for collision / display)."""
    sim_state['opponent'] = {
        'x':       float(data['x']),
        'y':       float(data['y']),
        'theta':   float(data.get('theta', 0)),
        'enabled': True,
    }


@socketio.on('set_opponent_enabled')
def on_set_opponent_enabled(data):
    if 'opponent' not in sim_state:
        sim_state['opponent'] = {'x': 0, 'y': 0, 'theta': 0, 'enabled': False}
    sim_state['opponent']['enabled'] = bool(data['enabled'])


@socketio.on('set_feature')
def on_set_feature(data):
    key = data['feature']
    val = data['value']
    sim_state['features'][key] = val
    brain.log(f"Feature {key} → {val}")


@socketio.on('set_feedrate')
def on_set_feedrate(data):
    f = float(data['value'])
    brain.motion.set_feedrate(f)
    brain.log(f"Feedrate → {f:.2f}")


@socketio.on('reload_strategy')
def on_reload_strategy():
    if brain.load_strategy():
        emit('reload', {'msg': 'strategy/match.py reloaded ✓'})


# ── Serial connection logic (shared between SocketIO handler and --connect) ──

def _do_connect(port: str):
    """Connect to the robot over serial. Called from a background thread.
    Transport type (USB vs XBee) is auto-detected from the firmware pong response.
    Updates globals and emits SocketIO status events."""
    global _hw_transport, _hw_brain, _hw_connecting, _hw_serial_port, _hw_serial_mode
    global _hw_tel_data, _connection_mode
    t = None
    try:
        from shared.config import BRIDGE_BAUDRATE
        from transport.xbee import XBeeTransport
        t = XBeeTransport(port=port, baudrate=BRIDGE_BAUDRATE)

        # Subscribe raw UART feed BEFORE connect() so handshake bytes are visible.
        def _on_raw_rx(line):
            socketio.emit('uart_raw', {'dir': 'rx', 'line': line})
        def _on_raw_tx(line):
            socketio.emit('uart_raw', {'dir': 'tx', 'line': line})
        t.subscribe_telemetry('_raw',    _on_raw_rx)
        t.subscribe_telemetry('_raw_tx', _on_raw_tx)
        socketio.emit('uart_raw', {'dir': 'sys', 'line': f'[connect] {port} @ {BRIDGE_BAUDRATE} bps'})

        ok = t.connect()
        if ok:
            global _hw_test_runner
            bridge_type     = t.bridge_type or 'xbee'
            transport_label = f'{"USB Wired" if bridge_type == "usb" else "XBee"} ({port})'
            _hw_transport   = t
            _hw_serial_port = port
            # Reset live telemetry on new connection
            _hw_tel_data = {k: None for k in _hw_tel_data}

            _hw_brain = Brain(t, theta_offset_deg=HW_THETA_OFFSET_DEG,
                              occupancy_grid=occupancy)
            # Sync pathfinding toggle from current UI state
            _hw_brain.motion.use_pathfinding = sim_state['features'].get('pathfinding', True)
            _hw_brain.load_strategy()
            # Hardware test runner — uses real transport, web-safe prompt
            _hw_test_runner = HardwareTestRunner(t, prompt_fn=_web_prompt)

            # ── T:p → compact position: "x_mm y_mm theta_mrad" ─────────
            def _on_pos(data_str):
                try:
                    parts = data_str.split()
                    if len(parts) >= 3:
                        robot.pos   = Vec2(float(parts[0]), float(parts[1]))
                        robot.theta = float(parts[2]) / 1000.0  # mrad → rad
                    else:
                        # Legacy format fallback
                        kv = dict(k.split('=') for k in data_str.split(','))
                        robot.pos   = Vec2(float(kv['x']), float(kv['y']))
                        robot.theta = float(kv['theta'])
                except Exception as e:
                    print(f"[TELEMETRY] pos parsing error: {e}")
            t.subscribe_telemetry('p', _on_pos)
            t.subscribe_telemetry('pos', _on_pos)  # legacy compat

            # ── T:m → compact motion: "R tx ty dist feed%" or "I feed%" ──
            def _on_motion(data_str):
                # Translate compact format to legacy-compatible string for
                # _build_state() parser.
                parts = data_str.split()
                if parts and parts[0] in ('R', 'I'):
                    # Compact format
                    if parts[0] == 'R' and len(parts) >= 5:
                        _hw_tel_data['motion'] = (
                            f"RUNNING,tx={parts[1]}.0,ty={parts[2]}.0,"
                            f"dist={parts[3]}.0,feed={int(parts[4])/100:.2f}")
                    elif parts[0] == 'I' and len(parts) >= 2:
                        _hw_tel_data['motion'] = f"IDLE,feed={int(parts[1])/100:.2f}"
                    else:
                        _hw_tel_data['motion'] = data_str
                else:
                    # Legacy or DONE frame — pass through
                    _hw_tel_data['motion'] = data_str
                if data_str.startswith('DONE:') and t.is_connected:
                    socketio.emit('motion_done', {
                        'ok':  data_str.startswith('DONE:ok'),
                        'raw': data_str,
                    })
                    if not t._waiting_motion:
                        def _auto_ack():
                            try:
                                t.execute('ack_done', timeout_ms=2000)
                            except Exception:
                                pass
                        threading.Thread(target=_auto_ack, daemon=True,
                                         name='auto-ack-done').start()
            t.subscribe_telemetry('m', _on_motion)
            t.subscribe_telemetry('motion', _on_motion)  # legacy compat

            # ── T:s → safety: "0" or "1" (same format) ──────────────────
            def _on_safety(data_str):
                _hw_tel_data['safety'] = data_str.strip()
            t.subscribe_telemetry('s', _on_safety)
            t.subscribe_telemetry('safety', _on_safety)  # legacy compat

            # ── T:c → chrono: elapsed ms (same format) ───────────────────
            def _on_chrono(data_str):
                _hw_tel_data['chrono'] = data_str.strip()
            t.subscribe_telemetry('c', _on_chrono)
            t.subscribe_telemetry('chrono', _on_chrono)  # legacy compat

            # ── TEL:t40 → T4.0 intercom health ───────────────────────────
            def _on_t40(data_str):
                _hw_tel_data['t40'] = data_str
            t.subscribe_telemetry('t40', _on_t40)

            # ── TEL:occ_dyn → dynamic cells only from T40 ─────────────────
            def _on_occ_dyn(data_str):
                try:
                    cells = []
                    data_str = data_str.strip()
                    if data_str:
                        for token in data_str.split(';'):
                            token = token.strip()
                            if not token:
                                continue
                            gx_s, gy_s = token.split(',')
                            cells.append((int(gx_s), int(gy_s)))
                    occupancy.set_dynamic_cells(cells)
                except Exception as e:
                    print(f"[TELEMETRY] occ_dyn decode error: {e}")
            t.subscribe_telemetry('od', _on_occ_dyn)        # compact
            t.subscribe_telemetry('occ_dyn', _on_occ_dyn)  # legacy

            # ── T:mask → compact "11110" or legacy "pos=1,motion=1,..." ──
            def _on_mask(data_str):
                global _hw_tel_mask
                try:
                    data_str = data_str.strip()
                    if '=' in data_str:
                        # Legacy format
                        for kv in data_str.split(','):
                            k, v = kv.split('=')
                            if k in _hw_tel_mask:
                                _hw_tel_mask[k] = (v.strip() == '1')
                    else:
                        # Compact: "11110" → pos,motion,safety,chrono,occ
                        keys = ['pos', 'motion', 'safety', 'chrono', 'occ']
                        for i, k in enumerate(keys):
                            if i < len(data_str):
                                _hw_tel_mask[k] = (data_str[i] == '1')
                except Exception:
                    pass
            t.subscribe_telemetry('mask', _on_mask)

            # ── Raw console output (help, errors, etc.) ──────────────
            def _on_console(line):
                socketio.emit('terminal_rx', {
                    'cmd': '', 'ok': True,
                    'res': line, 'mode': '[FW]'
                })
            t.subscribe_telemetry('_console', _on_console)

            # Transport type is auto-detected from pong response ('usb' or 'xbee')
            _hw_serial_mode  = 'wired' if t.bridge_type == 'usb' else 'xbee'
            _connection_mode = t.bridge_type if t.bridge_type in ('usb', 'xbee') else 'xbee'

            # ── Unexpected disconnect handler ─────────────────────────────
            # Do NOT clear _hw_transport — the transport object may reconnect
            # automatically (e.g. firmware reset, USB replug detected by OS).
            # Only clear brain/test runner; transport stays alive for reconnect.
            def _on_hw_disconnect():
                global _hw_brain, _hw_test_runner
                global _hw_tel_data, _connection_mode
                _hw_brain       = None
                _hw_test_runner = None
                _hw_tel_data    = {k: None for k in _hw_tel_data}
                _connection_mode = 'idle'
                socketio.emit('serial_status', {
                    'ok': False, 'msg': '⚠ Connexion perdue — reconnexion possible…'
                })
                socketio.emit('tests_catalog_changed', {'mode': 'idle'})
                brain.log('[HW] Connexion perdue (déconnexion inattendue)')

            t.on_disconnect(_on_hw_disconnect)

            # ── Reconnection handler ─────────────────────────────────────
            # Fires when transport receives a pong after having been
            # disconnected.  Restore backend state + notify frontend.
            _saved_mode  = _connection_mode
            _saved_label = transport_label

            def _on_hw_reconnect():
                global _connection_mode, _hw_brain, _hw_test_runner, _hw_transport
                _connection_mode = _saved_mode
                # Rebuild brain & test runner on the still-alive transport.
                # Use same signature as the initial _do_connect Brain creation.
                try:
                    if _hw_transport is not None:
                        _hw_brain = Brain(_hw_transport,
                                          theta_offset_deg=HW_THETA_OFFSET_DEG,
                                          occupancy_grid=occupancy)
                        _hw_brain.motion.use_pathfinding = sim_state['features'].get('pathfinding', True)
                        _hw_test_runner = HardwareTestRunner(
                            _hw_transport, prompt_fn=_web_prompt)
                        # Brain.__init__ overwrites on_connect/on_disconnect with
                        # its own callbacks.  Re-register ours so subsequent
                        # disconnect/reconnect cycles keep updating the UI.
                        _hw_transport.on_connect(_on_hw_reconnect)
                        _hw_transport.on_disconnect(_on_hw_disconnect)
                        # Push holOS-side settings to firmware on reconnect
                        _push_settings_to_firmware(_hw_transport)
                except Exception as e:
                    print(f"  [holOS] Reconnect rebuild error: {e}")
                socketio.emit('serial_status', {
                    'ok': True,
                    'msg': f'Reconnected — {_saved_label}',
                    'bridge': _saved_mode,
                    'port': _hw_serial_port,
                })
                socketio.emit('tests_catalog_changed', {'mode': _connection_mode})
                brain.log(f'[HW] Reconnected — {_saved_label}')
                print(f"  [holOS] Reconnected — {_saved_label}")

            t.on_connect(_on_hw_reconnect)

            # Push holOS-side settings to firmware on first connect
            _push_settings_to_firmware(t)

            socketio.emit('serial_status', {
                'ok': True, 'msg': f'Connected — {transport_label}',
                'bridge': _connection_mode, 'port': port,
            })
            socketio.emit('tests_catalog_changed', {'mode': _connection_mode})
            brain.log(f'[HW] Connected — {transport_label}')
            print(f"  [holOS] Connected — {transport_label}")
        else:
            _hw_transport   = None
            _hw_serial_port = None
            socketio.emit('serial_status', {
                'ok': False,
                'msg': f'No response from Teensy on {port} — check wiring, baud rate ({BRIDGE_BAUDRATE}), and XBee pairing',
            })
            print(f"  [holOS] No response from Teensy on {port}")
    except Exception as e:
        if t is not None:
            try:
                t.disconnect()
            except Exception:
                pass
        _hw_transport   = None
        _hw_serial_port = None
        socketio.emit('serial_status', {'ok': False, 'msg': str(e)})
        print(f"  [holOS] Connection error: {e}")
    finally:
        _hw_connecting = False


# ── XBee / serial connection (SocketIO handler) ─────────────────────────────

@socketio.on('serial_connect')
def on_serial_connect(data):
    global _hw_transport, _hw_brain, _hw_connecting
    port = (data.get('port') or '').strip()

    if not port:
        emit('serial_status', {'ok': False, 'msg': 'No port selected'})
        return

    if _hw_connecting:
        emit('serial_status', {'ok': False, 'msg': 'Connection already in progress…'})
        return

    # Disconnect any existing hw transport before opening a new one
    if _hw_transport is not None:
        try:
            _hw_transport.disconnect()
        except Exception:
            pass
        _hw_transport = None
        _hw_brain     = None

    _hw_connecting = True
    emit('serial_status', {'ok': False, 'msg': f'Connecting to {port}…', 'connecting': True})

    threading.Thread(target=_do_connect, args=(port,),
                     daemon=True, name='serial-connect').start()


@socketio.on('serial_disconnect')
def on_serial_disconnect():
    global _hw_transport, _hw_brain, _hw_test_runner, _hw_serial_port, _hw_tel_data
    global _connection_mode
    if _hw_transport is not None:
        try:
            _hw_transport.disconnect()
        except Exception:
            pass
        _hw_transport   = None
        _hw_brain       = None
        _hw_test_runner = None
        _hw_serial_port = None
        _hw_tel_data    = {k: None for k in _hw_tel_data}
        brain.log('[HW] Disconnected')
    # Go to idle — do NOT fall back to sim automatically
    _connection_mode = 'idle'
    emit('serial_status', {'ok': False, 'msg': 'Disconnected — idle'})
    emit('tests_catalog_changed', {'mode': 'idle'})


@socketio.on('connect_sim')
def on_connect_sim():
    """Explicitly connect to the simulator."""
    global _connection_mode
    # Disconnect hardware if any
    if _hw_transport is not None:
        on_serial_disconnect()
    _connection_mode = 'sim'
    brain.log('[SIM] Simulator started')
    emit('serial_status', {'ok': True, 'msg': 'Simulator active'})


# ── Terminal & actuator events ────────────────────────────────────────────────

@socketio.on('terminal_cmd')
def on_terminal_cmd(data):
    """Execute a command, wait for reply.  Routes to XBee if connected."""
    cmd = data.get('cmd', '').strip()
    if not cmd:
        return
    t = _active_transport()
    mode = '[HW]' if t is not transport else '[SIM]'
    timeout_ms = int(data.get('timeout_ms', 5000))
    try:
        print(f"[TERMINAL] Sending '{cmd}' via {mode} transport (timeout={timeout_ms}ms)")
        ok, res = t.execute(cmd, timeout_ms=timeout_ms)
        print(f"[TERMINAL] Got response: ok={ok}, res={res}")
        emit('terminal_rx', {'cmd': cmd, 'ok': ok, 'res': res, 'mode': mode})
    except Exception as e:
        print(f"[TERMINAL] Exception: {e}")
        emit('terminal_rx', {'cmd': cmd, 'ok': False, 'res': str(e), 'mode': mode})


@socketio.on('actuator_fire')
def on_actuator_fire(data):
    """Fire-and-forget actuator command.  Routes to XBee if connected."""
    cmd = data.get('cmd', '').strip()
    if not cmd:
        return
    t = _active_transport()
    mode = '[HW]' if t is not transport else '[SIM]'
    t.fire(cmd)
    _active_brain().log(f"[ACT]{mode} {cmd}")
    emit('terminal_rx', {'cmd': cmd, 'ok': True, 'res': '(fired)', 'mode': mode})


@socketio.on('uart_raw_tx')
def on_uart_raw_tx(data):
    """Send raw text directly to the serial port (no framing). Used by UART tab."""
    text = data.get('text', '')
    if not text:
        return
    if _hw_transport is None:
        emit('uart_raw', {'dir': 'sys', 'line': '[not connected to hardware]'})
        return
    # Append newline if missing and write — _raw_tx subscriber echoes it to UART tab
    if not text.endswith('\n'):
        text += '\n'
    _hw_transport._write(text)


@socketio.on('run_sequence')
def on_run_sequence(data):
    """Run a list of {cmd, delay_ms} steps in a background thread."""
    global _seq_thread, _seq_stop
    steps = data.get('steps', [])
    _seq_stop.clear()

    def _run():
        brain.log(f"[SEQ] Start — {len(steps)} steps")
        for i, step in enumerate(steps):
            if _seq_stop.is_set():
                brain.log("[SEQ] Stopped")
                socketio.emit('seq_done', {'stopped': True})
                return
            cmd      = step.get('cmd', '').strip()
            delay_ms = max(0, int(step.get('delay_ms', 0)))
            if cmd:
                transport.fire(cmd)
                brain.log(f"[SEQ] {i+1}/{len(steps)}: {cmd}")
                socketio.emit('terminal_rx', {'cmd': cmd, 'ok': True, 'res': '(seq)'})
            if delay_ms > 0:
                _seq_stop.wait(delay_ms / 1000.0)
        brain.log("[SEQ] Done ✓")
        socketio.emit('seq_done', {'stopped': False})

    _seq_thread = threading.Thread(target=_run, daemon=True, name="sequence")
    _seq_thread.start()


@socketio.on('stop_sequence')
def on_stop_sequence():
    _seq_stop.set()


# ── Test runner events ────────────────────────────────────────────────────────

@app.route('/api/tests/catalog')
def api_tests_catalog():
    """Return the test catalog appropriate for the current connection mode.

    Hardware connected (usb/xbee) → hardware test suites
    Simulator (sim)               → sim test suites
    Idle (no connection)          → empty dict (tests not available)
    """
    if _connection_mode == 'idle':
        return jsonify({})   # no tests when nothing is connected

    # Use the SAME check as _active_test_runner() so catalog and runner
    # always agree on which suite set is active.
    is_hw = _is_hw_connected() and _hw_test_runner is not None
    catalog = HW_SUITES if is_hw else SUITES
    return jsonify({
        sid: {
            'label': s['label'],
            'icon':  s.get('icon', '⚙'),
            'tests': s['tests'],
        }
        for sid, s in catalog.items()
    })


@socketio.on('test_prompt_ack')
def on_test_prompt_ack():
    """Browser clicked Continue on an interactive test prompt."""
    _prompt_evt.set()


@socketio.on('run_tests')
def on_run_tests(data):
    """
    data = {
      'suite': 'motors'   → run a full suite
      'test':  'mot_hb'   → run a single test
      (neither)           → run all tests
    }
    Routes to hw_test_runner if hardware is connected, else sim test_runner.
    """
    active_tr = _active_test_runner()
    if active_tr.is_running():
        emit('test_error', {'msg': 'Tests already running'})
        return

    # Guard: if user requests a HW test/suite but we fell back to sim runner,
    # emit an explicit error instead of silent "Not implemented".
    suite = data.get('suite')
    test  = data.get('test')
    is_hw_runner = _is_hw_connected() and _hw_test_runner is not None
    if not is_hw_runner:
        # Build set of all HW suite keys and test IDs
        hw_ids = set(HW_SUITES.keys())
        for s in HW_SUITES.values():
            for t_entry in s.get('tests', []):
                hw_ids.add(t_entry.get('id', ''))
        requested = test or suite
        if requested and requested in hw_ids:
            emit('test_error', {
                'msg': f'Hardware not connected — cannot run "{requested}". '
                       f'Reconnect the robot or switch to sim tests.'
            })
            return

    def _progress(test_id: str, status: str):
        socketio.emit('test_progress', {'id': test_id, 'status': status})

    def _result(r):
        socketio.emit('test_result', r.to_dict())

    def _done():
        socketio.emit('test_done', {})

    if test:
        active_tr.run_one(test, _progress, _result, _done)
    elif suite:
        active_tr.run_suite(suite, _progress, _result, _done)
    else:
        active_tr.run_all(_progress, _result, _done)


@socketio.on('stop_tests')
def on_stop_tests():
    _active_test_runner().stop()


@socketio.on('activate_strategy')
def on_activate_strategy(data):
    name = data.get('name', 'match.py')
    if not _safe_name(name): return
    if brain.load_strategy(name):
        emit('reload', {'msg': f'{name} activated ✓'})
        brain.log(f'Strategy → {name}')


@socketio.on('run_macro')
def on_run_macro(data):
    import math as _math
    steps  = data.get('steps', [])
    macros = data.get('macros', [])   # full macro list for call_macro support
    _seq_stop.clear()

    def _exec_steps(step_list):
        i = 0
        while i < len(step_list):
            if _seq_stop.is_set():
                return
            step  = step_list[i]
            stype = step.get('type', '')
            skip  = 0

            if stype == 'move_to':
                x, y = float(step.get('x', 0)), float(step.get('y', 0))
                brain.motion._t._bridge._start_motion_xy(
                    0, Vec2(x, y), lambda u, o, r: None)
                brain.log(f'[MACRO] move ({x:.0f},{y:.0f})')

            elif stype == 'actuator':
                cmd = step.get('cmd', '').strip()
                if cmd:
                    transport.fire(cmd)
                    brain.log(f'[MACRO] actuator: {cmd}')

            elif stype == 'wait':
                ms = max(0, int(step.get('ms', 0)))
                _seq_stop.wait(ms / 1000.0)

            elif stype == 'if_occupied':
                poi_name = step.get('poi', '')
                poi_pos  = getattr(POI, poi_name, None)
                occupied = False
                if poi_pos:
                    gx = int(poi_pos.x // GRID_CELL)
                    gy = int(poi_pos.y // GRID_CELL)
                    occ_cells = occupancy.to_list()
                    occupied  = any(c['gx'] == gx and c['gy'] == gy
                                    for c in occ_cells)
                if not occupied:
                    skip = int(step.get('skip', 1))
                    brain.log(f'[MACRO] {poi_name} clear → skip {skip}')
                else:
                    brain.log(f'[MACRO] {poi_name} OCCUPIED → execute branch')

            elif stype == 'call_macro':
                target = step.get('name', '')
                for m in macros:
                    if m.get('name') == target:
                        brain.log(f'[MACRO] call → {target}')
                        _exec_steps(m.get('steps', []))
                        break

            elif stype == 'log':
                brain.log(f'[MACRO] {step.get("msg", "")}')

            i += 1 + skip

    def _run():
        brain.log(f'[MACRO] Start — {len(steps)} steps')
        _exec_steps(steps)
        brain.log('[MACRO] Done ✓')
        socketio.emit('seq_done', {'stopped': False})

    threading.Thread(target=_run, daemon=True, name='macro').start()


# ── HW fire (Modules panel) ──────────────────────────────────────────────────

def _apply_tel_cmd(cmd: str):
    """Parse tel(channel, 0|1) and update _hw_tel_mask locally."""
    global _hw_tel_mask
    try:
        inner = cmd[4:-1]   # strip leading 'tel(' and trailing ')'
        comma = inner.index(',')
        ch  = inner[:comma].strip()
        val = inner[comma + 1:].strip() != '0'
        if ch in _hw_tel_mask:
            _hw_tel_mask[ch] = val
    except Exception:
        pass


@socketio.on('hw_fire')
def on_hw_fire(data):
    """Fire-and-forget command to real hardware (used by Modules panel)."""
    cmd = (data.get('cmd') or '').strip()
    if not cmd:
        return

    # Always apply tel() changes locally so the dashboard mask stays in sync.
    if cmd.startswith('tel(') and cmd.endswith(')'):
        _apply_tel_cmd(cmd)

    if _hw_transport is not None and _hw_transport.is_connected:
        try:
            _hw_transport.fire(cmd)
            brain.log(f'[HW] {cmd}')
        except Exception as e:
            emit('hw_ack', {'ok': False, 'cmd': cmd, 'err': str(e)})
            return
    else:
        if not cmd.startswith('tel('):
            transport.fire(cmd)
        brain.log(f'[SIM] {cmd}')
    emit('hw_ack', {'ok': True, 'cmd': cmd})


# ── Match control (remote start/stop via bridge) ─────────────────────────────

@socketio.on('match_start')
def on_match_start():
    """Remote match start — same effect as pulling the physical starter.

    After a successful firmware start, queries the strategy switch:
      strat=1 (remote/intelligent) → also starts the Python brain
      strat=0 (internal/sequential) → firmware runs C++ blocks autonomously

    UI feedback is broadcast to ALL clients immediately after the firmware ACK.
    """
    global _match_running, _match_paused
    t = _active_transport()
    if t is None or not t.is_connected:
        brain.log('[MATCH] No HW transport connected — cannot start firmware match')
        socketio.emit('match_state', {'running': False, 'error': 'not_connected'})
        return

    ok, res = t.execute('match_start', timeout_ms=3000)
    if not ok:
        brain.log(f'[MATCH] Remote start FAILED: {res}')
        socketio.emit('match_state', {'running': False})
        return

    # Firmware ACKed — match is starting.  Broadcast to all clients.
    _match_running = True
    _match_paused  = False
    brain.log('[MATCH] Firmware match started ✓')
    socketio.emit('match_state', {'running': True, 'paused': False})

    # Now check strategy switch to decide whether to also run the Python brain.
    h_ok, h_res = t.execute('health', timeout_ms=2000)
    if h_ok and 'strat=1' in (h_res or ''):
        brain.log('[MATCH] Remote strategy — starting Python brain')
        _active_brain().run_match(on_done=_on_match_done)
    else:
        brain.log('[MATCH] Internal strategy — C++ handles match autonomously')
        # For strat=0 (C++ autonomous), the firmware runs the match.
        # Watch chrono in a background thread to detect match end (~100 s).
        def _watch_chrono():
            global _match_running, _match_paused
            while _match_running:
                time.sleep(2.0)
                try:
                    elapsed = float(_hw_tel_data.get('chrono', '0') or '0')
                except (ValueError, TypeError):
                    elapsed = 0.0
                if elapsed >= 99.5:
                    _match_running = False
                    _match_paused  = False
                    brain.log('[MATCH] Chrono ≥ 100 s — match ended (C++ autonomous)')
                    socketio.emit('match_state', {'running': False, 'paused': False})
                    return
        threading.Thread(target=_watch_chrono, daemon=True, name="chrono-watch").start()


@socketio.on('match_stop')
def on_match_stop():
    """Remote match stop — pauses program, cancels in-flight motion."""
    global _match_running, _match_paused
    t = _active_transport()
    if t is not None and t.is_connected:
        ok, res = t.execute('match_stop', timeout_ms=3000)
        if ok:
            brain.log('[MATCH] Remote stop → paused')
        else:
            brain.log(f'[MATCH] Remote stop failed: {res}')
        _match_running = not ok
        _match_paused  = ok
        socketio.emit('match_state', {'running': not ok, 'paused': ok})
    else:
        brain.log('[MATCH] No transport connected')


@socketio.on('match_resume')
def on_match_resume():
    """Resume a paused match."""
    global _match_running, _match_paused
    t = _active_transport()
    if t is not None and t.is_connected:
        ok, res = t.execute('match_resume', timeout_ms=3000)
        if ok:
            brain.log('[MATCH] Resumed')
        _match_running = ok
        _match_paused  = not ok
        socketio.emit('match_state', {'running': ok, 'paused': not ok})


# ── C++ Block discovery & execution ──────────────────────────────────────────

@app.route('/api/cpp_blocks', methods=['GET'])
def api_cpp_blocks():
    """
    Fetch registered C++ blocks from the Teensy via the blocks_list bridge command.
    Returns a list of {name, priority, score, estimatedMs, done} objects.
    """
    t = _active_transport()
    if t is None or not t.is_connected:
        return jsonify([])

    ok, raw = t.execute('blocks_list', timeout_ms=3000)
    if not ok or not raw:
        return jsonify([])

    blocks = []
    for entry in raw.split(';'):
        entry = entry.strip()
        if not entry or '=' not in entry:
            continue
        name, vals = entry.split('=', 1)
        parts = vals.split(',')
        if len(parts) >= 4:
            blocks.append({
                'name':        name,
                'priority':    int(parts[0]),
                'score':       int(parts[1]),
                'estimatedMs': int(parts[2]),
                'done':        parts[3] == '1',
                'source':      'cpp',
            })
    return jsonify(blocks)


@socketio.on('run_cpp_block')
def on_run_cpp_block(data):
    """Execute a registered C++ block on the Teensy by name."""
    name = (data.get('name') or '').strip()
    if not name:
        emit('cpp_block_result', {'ok': False, 'error': 'no_name'})
        return

    t = _active_transport()
    if t is None or not t.is_connected:
        brain.log(f'[BLOCK] Cannot run {name} — not connected')
        emit('cpp_block_result', {'ok': False, 'name': name, 'error': 'not_connected'})
        return

    brain.log(f'[BLOCK] Running C++ block: {name}')
    ok, res = t.execute(f'run_block({name})', timeout_ms=60000)
    success = ok and res == 'SUCCESS'
    brain.log(f'[BLOCK] {name} → {"SUCCESS" if success else "FAILED"}')
    emit('cpp_block_result', {'ok': success, 'name': name, 'result': res})


@socketio.on('mark_cpp_block_done')
def on_mark_block_done(data):
    """Mark a C++ block as done from the Jetson side."""
    name = (data.get('name') or '').strip()
    if not name:
        return

    t = _active_transport()
    if t is not None and t.is_connected:
        ok, res = t.execute(f'block_done({name})', timeout_ms=3000)
        if ok:
            brain.log(f'[BLOCK] {name} marked done')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='holOS — Robot control & simulation')
    parser.add_argument('--port', type=int, default=5000,
                        help='Web server port (default: 5000)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Web server bind address (default: 0.0.0.0)')
    parser.add_argument('--connect', metavar='SERIAL_PORT', nargs='?', const='__auto__',
                        help='Auto-connect to robot on startup. On Jetson (Linux), '
                             'defaults to /dev/ttyUSB0 if no port given.')
    parser.add_argument('--baud', type=int, default=0,
                        help='Serial baud rate (default: auto-detect from config)')
    parser.add_argument('--sim', action='store_true',
                        help='Start directly in simulator mode')
    parser.add_argument('--auto-start', action='store_true',
                        help='Auto-start match strategy after connecting')
    parser.add_argument('--no-auto-connect', action='store_true',
                        help='On Jetson, skip automatic hardware connection')
    args = parser.parse_args()

    # ── Platform-aware defaults ──────────────────────────────────────────────
    # On Jetson (Linux): auto-connect to /dev/ttyUSB0 unless --sim or --no-auto-connect
    connect_port = None
    connect_baud = args.baud

    if args.connect == '__auto__':
        # --connect with no argument: use platform default
        connect_port = _DEFAULT_JETSON_PORT if IS_JETSON else None
        if connect_port is None:
            print("  [holOS] --connect with no port on Windows — use --connect COMx")
    elif args.connect:
        # --connect COMx  or  --connect /dev/ttyUSB0
        connect_port = args.connect
    elif IS_JETSON and not args.sim and not args.no_auto_connect:
        # Jetson with no flags: auto-connect by default
        connect_port = _DEFAULT_JETSON_PORT
        print(f"  [holOS] Jetson detected — auto-connecting to {connect_port}")

    # Default baud: use BRIDGE_BAUDRATE from config if not specified
    if connect_baud == 0:
        from shared.config import BRIDGE_BAUDRATE
        connect_baud = BRIDGE_BAUDRATE

    # ── Pre-start: sim mode ──────────────────────────────────────────────────
    global _connection_mode
    if args.sim:
        _connection_mode = 'sim'

    brain.load_strategy()
    brain.start_hot_reload(on_reload=lambda: socketio.emit(
        'reload', {'msg': 'strategy/match.py reloaded ✓'}
    ))
    socketio.start_background_task(_physics_loop)
    socketio.start_background_task(_vision_push_loop)

    # ── Auto-connect to hardware ─────────────────────────────────────────────
    if connect_port:
        def _auto_connect():
            global _hw_connecting
            time.sleep(1.0)  # Let Flask/SocketIO initialize
            from shared.config import BRIDGE_BAUDRATE as _AB
            _hw_connecting = True
            print(f"  [holOS] Auto-connecting to {connect_port} @ {_AB} bps…")
            _do_connect(connect_port)
            # Auto-start match if requested and connected
            if args.auto_start and _hw_transport is not None and _hw_transport.is_connected:
                time.sleep(0.5)
                print("  [holOS] Auto-starting match strategy…")
                _active_brain().run_match(on_done=_on_match_done)
        socketio.start_background_task(_auto_connect)

    plat_tag = 'Jetson' if IS_JETSON else 'PC'
    mode = 'sim' if args.sim else ('→ ' + connect_port if connect_port else 'idle')
    print(f"\n  holOS ({plat_tag})")
    print("  ┌────────────────────────────────┐")
    print(f"  │  http://localhost:{args.port:<14}│")
    print(f"  │  mode: {mode:<23}│")
    print("  └────────────────────────────────┘\n")

    socketio.run(app, host=args.host, port=args.port,
                 debug=False, use_reloader=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()