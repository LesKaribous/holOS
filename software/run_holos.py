"""
run.py — Unified holOS entry point (simulator + hardware).

Starts the holOS web UI with support for:
  - Simulator mode (VirtualTransport + SimBridge physics)
  - USB wired mode (WiredTransport → Teensy USB-CDC)
  - XBee radio mode (XBeeTransport → Teensy Serial3)

The web UI is always served; connection mode can be selected from the
browser or pre-configured with --connect.

Usage:
    python run.py                          # start in idle mode (connect via UI)
    python run.py --sim                    # start in simulator mode
    python run.py --connect COM6           # auto-connect USB on startup
    python run.py --connect /dev/ttyUSB0 --baud 115200   # auto-connect XBee
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

# Sequence runner state
_seq_stop   = threading.Event()
_seq_thread = None

# ── Path setup ────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
STRATEGY_DIR = os.path.join(_HERE, 'strategy')
MACROS_FILE    = os.path.join(STRATEGY_DIR, 'macros.json')
MISSIONS_FILE  = os.path.join(STRATEGY_DIR, 'missions.json')
OBJECTS_FILE   = os.path.join(STRATEGY_DIR, 'objects.json')
FALLBACK_PATH  = '/mission_fallback.cfg'   # path on Teensy SD card
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

# ── Calibration state (Python-side cache) ─────────────────────────────────────
# Mirrors Calibration::Current + OtosLinear/OtosAngular on the firmware.
# Updated whenever the user sends a calibration command; reset to firmware
# defaults when calib_reset is called.
_CALIB_DEFAULTS = {
    'cx': 1.089, 'cy': -1.089, 'cr': 0.831,  # Cartesian scale factors
    'ha': 1.0,   'hb': 1.0,   'hc': 1.0,     # per-wheel holonomic factors
    'ol': 0.9714, 'oa': 1.0,                  # OTOS linear / angular scalars
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
    if hw_on and _hw_tel_data.get('motion'):
        parts = _hw_tel_data['motion'].split(',')
        hw_motion_state = parts[0] if parts else 'IDLE'
        for p in parts[1:]:
            if p.startswith('feed='):
                try: hw_motion_feed = float(p[5:])
                except ValueError: pass

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

    return {
        'robot':     robot.to_dict(),
        'path':      path_pts,
        'occupancy': occ_list,
        'game_objs': objs,
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


@app.route('/api/go', methods=['POST'])
def api_go():
    """Move to (x, y) via the active brain's motion service (uses pathfinder if enabled)."""
    data = request.get_json(force=True) or {}
    try:
        x = float(data['x'])
        y = float(data['y'])
    except (KeyError, ValueError) as e:
        return jsonify({'ok': False, 'res': f'bad params: {e}'}), 400
    try:
        b = _active_brain()
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
    try:
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
    """Save calibration to SD card on the robot."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    _calib_fire('calib_save')
    return jsonify({'ok': True})

@app.route('/api/calibration/load', methods=['POST'])
def api_calib_load():
    """Load calibration from SD card and refresh local cache via calib_status."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    try:
        ok, res = _hw_transport.execute('calib_load', timeout_ms=5000)
        # Try to parse the status string if it came back in the response
        _try_parse_calib_response(res)
        return jsonify({'ok': ok, 'calib': dict(_calib)})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500

@app.route('/api/calibration/reset', methods=['POST'])
def api_calib_reset():
    """Reset calibration to firmware defaults."""
    global _calib
    _calib_fire('calib_reset')
    _calib = dict(_CALIB_DEFAULTS)
    return jsonify({'ok': True, 'calib': dict(_calib)})

@app.route('/api/calibration/measure', methods=['POST'])
def api_calib_measure():
    """Run calib_measure(dist_mm) on the robot, wait for result."""
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    try:
        dist = float(data.get('dist_mm', 500))
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'invalid dist_mm'}), 400

    if dist < 50 or dist > 3000:
        return jsonify({'ok': False, 'error': 'dist_mm must be 50–3000'}), 400

    try:
        # Long timeout — robot has to physically move
        ok, res = _hw_transport.execute(f'calib_measure({dist})', timeout_ms=20000)
        return jsonify({'ok': ok, 'result': res})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500

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


@app.route('/api/missions/deploy-sd', methods=['POST'])
def api_missions_deploy_sd():
    """
    Generate the fallback strategy and write it to the Teensy SD card.
    Sends 'mission_write_sd <content>' (chunked) to firmware.
    Returns the generated text so the UI can preview it.
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'Robot non connecté'}), 503

    missions = _load_missions()
    macros   = _load_missions_macros()
    cfg      = _generate_fallback_cfg(missions, macros)

    # Write to SD via firmware command (multi-line → send line by line)
    try:
        ok1, _ = _hw_transport.execute('mission_sd_open', timeout_ms=3000)
        if not ok1:
            return jsonify({'ok': False, 'error': 'mission_sd_open failed'}), 500
        for line in cfg.splitlines():
            # Escape any special characters if needed
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
    try:
        import serial.tools.list_ports
        return jsonify([
            {'port': p.device, 'desc': p.description}
            for p in serial.tools.list_ports.comports()
        ])
    except ImportError:
        return jsonify([])


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
    _active_brain().run_match()


@socketio.on('stop_strategy')
def on_stop_strategy():
    _active_brain().stop_match()


@socketio.on('reset')
def on_reset():
    brain.stop_match()
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

def _do_connect(port: str, baud: int):
    """Connect to the robot over serial. Called from a background thread.
    Updates globals and emits SocketIO status events."""
    global _hw_transport, _hw_brain, _hw_connecting, _hw_serial_port, _hw_serial_mode
    global _hw_tel_data, _connection_mode
    t = None
    try:
        from shared.config import USB_DIRECT_BAUDRATE, XBEE_BAUDRATE
        if baud == USB_DIRECT_BAUDRATE:
            from transport.wired import WiredTransport
            t = WiredTransport(port=port)
            transport_label = f'USB Wired @ {baud} bps'
            _hw_serial_mode = 'wired'
        else:
            from transport.xbee import XBeeTransport
            t = XBeeTransport(port=port, baudrate=baud)
            transport_label = f'XBee @ {baud} bps'
            _hw_serial_mode = 'xbee'

        ok = t.connect()
        if ok:
            global _hw_test_runner
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

            # ── TEL:pos → update robot position (used for map display) ────
            def _on_pos(data_str):
                try:
                    parts = dict(kv.split('=') for kv in data_str.split(','))
                    robot.pos   = Vec2(float(parts['x']), float(parts['y']))
                    robot.theta = float(parts['theta'])
                except Exception as e:
                    print(f"[TELEMETRY] pos parsing error: {e}")
            t.subscribe_telemetry('pos', _on_pos)

            # ── TEL:motion → live motion state override ───────────────────
            def _on_motion(data_str):
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
            t.subscribe_telemetry('motion', _on_motion)

            # ── TEL:safety → live safety override ────────────────────────
            def _on_safety(data_str):
                _hw_tel_data['safety'] = data_str
            t.subscribe_telemetry('safety', _on_safety)

            # ── TEL:chrono → live chrono override ────────────────────────
            def _on_chrono(data_str):
                _hw_tel_data['chrono'] = data_str
            t.subscribe_telemetry('chrono', _on_chrono)

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
            t.subscribe_telemetry('occ_dyn', _on_occ_dyn)

            # ── TEL:mask → update channel enable/disable state ────────────
            def _on_mask(data_str):
                global _hw_tel_mask
                try:
                    for kv in data_str.split(','):
                        k, v = kv.split('=')
                        if k in _hw_tel_mask:
                            _hw_tel_mask[k] = (v.strip() == '1')
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

            # Set connection mode based on transport type
            _connection_mode = 'usb' if _hw_serial_mode == 'wired' else 'xbee'

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
            _saved_mode  = 'usb' if _hw_serial_mode == 'wired' else 'xbee'
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

            socketio.emit('serial_status', {
                'ok': True, 'msg': f'Connected — {transport_label}'
            })
            socketio.emit('tests_catalog_changed', {'mode': _connection_mode})
            brain.log(f'[HW] Connected — {transport_label}')
            print(f"  [holOS] Connected — {transport_label}")
        else:
            _hw_transport   = None
            _hw_serial_port = None
            hint = ('Check USB cable and Teensy firmware'
                    if baud == USB_DIRECT_BAUDRATE
                    else 'Check XBee wiring and baud rate')
            socketio.emit('serial_status', {
                'ok': False,
                'msg': f'No response from Teensy on {port} — {hint}',
            })
            print(f"  [holOS] No response from Teensy on {port} — {hint}")
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
    baud = int(data.get('baud') or 115200)

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

    threading.Thread(target=_do_connect, args=(port, baud),
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

    UI feedback is emitted immediately after the firmware ACK so the webapp
    does not appear to "do nothing" while health is being queried.
    """
    t = _active_transport()
    if t is None or not t.is_connected:
        brain.log('[MATCH] No HW transport connected — cannot start firmware match')
        emit('match_state', {'running': False, 'error': 'not_connected'})
        return

    ok, res = t.execute('match_start', timeout_ms=3000)
    if not ok:
        brain.log(f'[MATCH] Remote start FAILED: {res}')
        emit('match_state', {'running': False})
        return

    # Firmware ACKed — match is starting.  Emit immediately so UI updates.
    brain.log('[MATCH] Firmware match started ✓')
    emit('match_state', {'running': True})

    # Now check strategy switch to decide whether to also run the Python brain.
    h_ok, h_res = t.execute('health', timeout_ms=2000)
    if h_ok and 'strat=1' in (h_res or ''):
        brain.log('[MATCH] Remote strategy — starting Python brain')
        _active_brain().run_match()
    else:
        brain.log('[MATCH] Internal strategy — C++ handles match autonomously')


@socketio.on('match_stop')
def on_match_stop():
    """Remote match stop — pauses program, cancels in-flight motion."""
    t = _active_transport()
    if t is not None and t.is_connected:
        ok, res = t.execute('match_stop', timeout_ms=3000)
        if ok:
            brain.log('[MATCH] Remote stop → paused')
        else:
            brain.log(f'[MATCH] Remote stop failed: {res}')
        emit('match_state', {'running': not ok, 'paused': ok})
    else:
        brain.log('[MATCH] No transport connected')


@socketio.on('match_resume')
def on_match_resume():
    """Resume a paused match."""
    t = _active_transport()
    if t is not None and t.is_connected:
        ok, res = t.execute('match_resume', timeout_ms=3000)
        if ok:
            brain.log('[MATCH] Resumed')
        emit('match_state', {'running': ok, 'paused': not ok})


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
    parser.add_argument('--connect', metavar='SERIAL_PORT',
                        help='Auto-connect to robot on startup (e.g., COM6, /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate for --connect (default: 115200)')
    parser.add_argument('--sim', action='store_true',
                        help='Start directly in simulator mode')
    parser.add_argument('--auto-start', action='store_true',
                        help='Auto-start match strategy after connecting')
    args = parser.parse_args()

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

    # ── Auto-connect to hardware if --connect provided ───────────────────────
    if args.connect:
        def _auto_connect():
            global _hw_connecting
            time.sleep(1.0)  # Let Flask/SocketIO initialize
            _hw_connecting = True
            print(f"  [holOS] Auto-connecting to {args.connect} @ {args.baud}…")
            _do_connect(args.connect, args.baud)
            # Auto-start match if requested and connected
            if args.auto_start and _hw_transport is not None and _hw_transport.is_connected:
                time.sleep(0.5)
                print("  [holOS] Auto-starting match strategy…")
                _active_brain().run_match()
        socketio.start_background_task(_auto_connect)

    mode = 'sim' if args.sim else ('→ ' + args.connect if args.connect else 'idle')
    print(f"\n  holOS")
    print("  ┌────────────────────────────────┐")
    print(f"  │  http://localhost:{args.port:<14}│")
    print(f"  │  mode: {mode:<23}│")
    print("  └────────────────────────────────┘\n")

    socketio.run(app, host=args.host, port=args.port,
                 debug=False, use_reloader=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
