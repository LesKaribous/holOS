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
# Real Jetson boards expose a device-tree node whose `model` file contains
# "NVIDIA Jetson …". x86 PCs (including Ubuntu dev laptops) don't have
# /proc/device-tree at all. The prior `platform.system()=='Linux'` check
# was wrong — it flagged every Linux box as Jetson, which broke the
# topbar host indicator and (worse) auto-connect on dev laptops.
def _detect_jetson() -> bool:
    try:
        with open('/proc/device-tree/model', 'rb') as f:
            return b'Jetson' in f.read()
    except Exception:
        return False

IS_JETSON = _detect_jetson()
print(f"[run.py] Detected platform: {platform.system()} (IS_JETSON={IS_JETSON})")

_DEFAULT_JETSON_PORT  = '/dev/ttyUSB0'
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

import logging as _logging
class _QuietPollEndpoints(_logging.Filter):
    # UI poll endpoints hit once or twice per second per client — the
    # access lines drown the actually-useful match/vision/path-planning
    # logs. Add new poll URLs here; everything else still gets logged.
    _MUTE = (
        '/api/log/status',
        '/api/vision_camera/status',
        '/api/vision_camera/source',   # also matches /source/clear
        '/api/vision/calibration',     # also matches /api/vision/calibration/pairs
        '/api/vision/robot_pose',
        '/api/vision/pipelines',
        '/api/vision/state',
        '/api/vision/feeds',
        '/api/state',
        '/api/status',
        '/socket.io/',
    )
    def filter(self, record):
        msg = record.getMessage()
        return not any(p in msg for p in self._MUTE)
_logging.getLogger('werkzeug').addFilter(_QuietPollEndpoints())

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

# ── Multi-source pipeline registry (TODO #2) ──────────────────────────────────
# Independent of the legacy match-time backend. The visio tab's React Flow
# editor drives this; pipelines persist to data/vision_pipelines.json.
# NB: package name is 'vision_pipelines' (NOT 'vision') so it doesn't shadow
# the existing services/vision.py (VisionService) that brain.py imports.
_pipeline_registry = None
try:
    from services.vision_pipelines import PipelineRegistry, Pipeline, NODE_KINDS
    _pipeline_registry = PipelineRegistry()
except Exception as _pe:
    print(f"[Vision] Pipeline registry unavailable: {_pe}")

VISION_PIPELINES_PATH = os.path.join(_HERE, 'data', 'vision_pipelines.json')
PARALLAX_CALIB_PATH   = os.path.join(_HERE, 'data', 'parallax_calibration.json')
VISION_CONFIG_PATH    = os.path.join(_HERE, 'data', 'vision_config.json')

_VISION_CONFIG_DEFAULTS = {
    # FPS knobs — single source of truth. Three independent rates that
    # all sample the same in-process 1-slot BGR buffer (vision_source.py):
    #   source_fps   — hardware capture rate. Advisory only — the actual
    #                  rate is determined by the camera/file. Used as the
    #                  GStreamer framerate hint for V4L2 sources.
    #   record_fps   — AVI writer rate (vision_recorder.py).
    #   pipeline_fps — pipeline tick rate (services.vision_pipelines).
    # Typical: 30 → 16 → 4. All three are independent samplers of the
    # same latest-frame slot, so there is no queue / backlog anywhere.
    'source_fps':   30,
    'record_fps':   16,
    'pipeline_fps': 4,

    # In-process FrameSource — owns the camera/file, single reader thread.
    'source_kind': 'video',         # video | camera | image
    'source_path': 'vision/homography/data/video.2.mp4',
    'gst_width':   1280,            # camera-only: GStreamer/V4L2 width
    'gst_height':  720,             # camera-only: GStreamer/V4L2 height
    'gst_fps':     30,              # camera-only: GStreamer/V4L2 fps
    'loop':        True,            # video-only: restart at EOF

    # World coordinate frame — interpretation of every (x_mm, y_mm) below.
    'world_origin_corner': 'bottom_right',
    'world_flip_theta':    True,
    'table_width_mm':      3000.0,
    'table_height_mm':     2000.0,

    # 4 corner ArUcos in world frame. Keys are legacy slot labels — only
    # the count (4) matters, not which key holds which corner.
    'anchors': {
        'top_left':     {'tag_id': 23, 'x_mm': 2400, 'y_mm': 1400, 'yaw_deg': 0},
        'top_right':    {'tag_id': 22, 'x_mm':  600, 'y_mm': 1400, 'yaw_deg': 0},
        'bottom_right': {'tag_id': 20, 'x_mm':  600, 'y_mm':  600, 'yaw_deg': 0},
        'bottom_left':  {'tag_id': 21, 'x_mm': 2400, 'y_mm':  600, 'yaw_deg': 0},
    },

    # Camera position per team (mirrors with team since the mount is
    # rigid but the camera always views the OWN half). z is camera height
    # above the table — used by parallax correction.
    'cam_yellow_xyz_mm': [1275.0, -100.0, 1100.0],
    'cam_blue_xyz_mm':   [1725.0, -100.0, 1100.0],
    'robot_z_mm':        550.0,

    'own_object_z_mm':   550.0,
    'opp_object_z_mm':   550.0,
}

def _load_vision_config() -> dict:
    cfg = {k: (list(v) if isinstance(v, list) else v)
           for k, v in _VISION_CONFIG_DEFAULTS.items()}
    try:
        if os.path.exists(VISION_CONFIG_PATH):
            with open(VISION_CONFIG_PATH) as f:
                cfg.update(json.load(f))
    except Exception as e:
        print(f'[Vision] vision_config.json load error: {e}')
    return cfg

def _save_vision_config(cfg: dict) -> None:
    os.makedirs(os.path.dirname(VISION_CONFIG_PATH), exist_ok=True)
    with open(VISION_CONFIG_PATH, 'w') as f:
        json.dump(cfg, f, indent=2)

# Auto-sync heading state — set when match_start fires; the heading-sync
# routine then waits for the recalage routine to settle.
_vision_heading_sync_pending = False

# Vision-pipeline recalage state. Populated by `T:vis cal_request` (sent by
# the firmware once the robot has reached its known starting pose) and
# consumed by every subsequent `T:vis pose_request` so the corrected pose
# returned to the firmware is in the ROBOT frame, not the (arbitrarily
# mounted) tag frame.
#
#   theta_robot = wrap_pi(theta_tag_vision + heading_offset_rad)
#
# None until cal_request succeeds. Reset by `T:vis homography_release` or
# by a fresh cal_request.
_vision_heading_offset_rad: 'Optional[float]' = None

# Full snapshot of the recalage handshake — published to the debug page
# so the operator can sanity-check the parallax/heading numbers without
# digging into logs. Updated by `_recalage_pick_own_tag`.
#   known_pose:           {x_mm, y_mm, theta_rad}      robot's reported pose
#   tag_pose_vision_raw:  {x_mm, y_mm, theta_rad}      what vision saw at calib
#   xy_offset_mm:         (dx, dy) = vision - known    parallax residual
#   heading_offset_rad:   theta_robot - theta_tag_vision
#   tag_id, team, robot_z_mm, captured_at_t
_vision_calibration_snapshot: 'Optional[dict]' = None

# Rolling list of (naive_xy, true_xy) pairs captured at every successful
# `_recalage_pick_own_tag`. Feeds the multi-pose parallax solver — with
# 2+ pairs we can least-squares fit cam_x, cam_y, and factor (=> z_obj).
# Single-pose calibration only gives a degenerate factor estimate
# because it cannot separate cam_xy errors from z_obj errors.
_vision_calibration_pairs: list = []

# Rolling buffer of vision-pipeline events. Surfaced via the
# /api/vision/calibration endpoint so the debug page can show them
# live — saves the operator from grepping the holOS console when
# trying to figure out why a recalage didn't take.
import collections as _collections
_vision_log_buffer: '_collections.deque' = _collections.deque(maxlen=100)

# pose_request invalid-streak tracking — fires a rate-limited warning into
# the vision-debug log when the pipeline keeps returning no own-team pose
# while the firmware is actively asking. Reset to 0 on the next valid reply.
_pose_request_invalid_streak: int = 0
_pose_request_invalid_last_warn_mono: float = 0.0


def _vlog(msg: str, level: str = 'info') -> None:
    """Mirror an event to (a) the holOS console and (b) the rolling
    vision-debug log. `level` is one of 'info' / 'warn' / 'err' and
    drives the row color in the debug tile."""
    try:
        _vision_log_buffer.append({
            't_mono': time.monotonic(),
            't_iso':  time.strftime('%H:%M:%S'),
            'level':  level,
            'msg':    msg,
        })
    except Exception:
        pass
    try:
        from services.match_logger import MATCH_LOGGER
        MATCH_LOGGER.log('vision', f'[{level}] {msg}')
    except Exception:
        pass
    try:
        brain.log(f'[VISION] {msg}')
    except Exception:
        pass

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

@app.route('/vision_debug')
def vision_debug():
    """Debug page that shows the FULL set of vision feeds (frames +
    pose data) — meant to be opened by `software/vision.bat` while holOS
    is running. The HTML is a static file under sim/static and connects
    to the same SocketIO endpoint as holOS, subscribing to vision_feed."""
    from flask import send_from_directory
    return send_from_directory(
        os.path.join(_HERE, 'sim', 'static'), 'vision_debug.html')


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


@app.route('/api/log/start', methods=['POST'])
def api_log_start():
    from services.match_logger import MATCH_LOGGER
    meta = {
        'team':              sim_state.get('team'),
        'connection_mode':   _connection_mode,
        'serial_port':       _hw_serial_port,
    }
    sid = MATCH_LOGGER.start(meta=meta)
    return jsonify({'ok': True, **MATCH_LOGGER.status()})


@app.route('/api/log/stop', methods=['POST'])
def api_log_stop():
    from services.match_logger import MATCH_LOGGER
    summary = MATCH_LOGGER.stop()
    return jsonify({'ok': True, 'summary': summary})


@app.route('/api/log/status', methods=['GET'])
def api_log_status():
    from services.match_logger import MATCH_LOGGER
    return jsonify(MATCH_LOGGER.status())


@app.route('/api/vision_config', methods=['GET'])
def api_vision_config_json_get():
    return jsonify({'ok': True, 'config': _load_vision_config()})


@app.route('/api/vision_config', methods=['POST'])
def api_vision_config_json_set():
    """Update vision_config.json values; re-push to pipeline via _apply_team."""
    data = request.get_json(force=True) or {}
    cfg = _load_vision_config()
    for k in ('cam_yellow_xyz_mm', 'cam_blue_xyz_mm'):
        if k in data:
            v = data[k]
            if isinstance(v, list) and len(v) == 3:
                cfg[k] = [float(v[0]), float(v[1]), float(v[2])]
    for k in ('own_object_z_mm', 'opp_object_z_mm'):
        if k in data:
            try: cfg[k] = float(data[k])
            except (TypeError, ValueError): pass
    try:
        _save_vision_config(cfg)
    except PermissionError as e:
        import pwd
        try: user = pwd.getpwuid(os.getuid()).pw_name
        except Exception: user = f'uid={os.getuid()}'
        return jsonify({'ok': False, 'error':
            f'save failed (permission denied as user {user!r}): {e}. '
            f'Fix: chown to match the service user, or chmod 666 the file.'
        }), 500
    except Exception as e:
        return jsonify({'ok': False, 'error': f'save failed: {e}'}), 500
    try:
        _apply_team(sim_state.get('team', 'blue'), source='vision_config', force=True)
    except Exception as e:
        return jsonify({'ok': True, 'config': cfg,
                        'warning': f'saved but re-push failed: {e}'})
    return jsonify({'ok': True, 'config': cfg})


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


@app.route('/api/calibration/stall_probe', methods=['POST'])
def api_calib_stall_probe():
    """Stall-detection calibration probe — approach wall, stall, correct position, back off.

    Body: { wall: 'NORTH'|'SOUTH'|'EAST'|'WEST', face: 'A'|'AB'|'B'|'BC'|'C'|'CA',
            clearance?: float (mm, default 200), degagement?: float (mm, default 80) }
    Returns: { ok, wall, face, x, y, theta, stalled, dur_ms, travel_mm, stall_min_trans, raw }
    """
    if not _calib_connected():
        return jsonify({'ok': False, 'error': 'not connected'}), 503
    data = request.get_json(force=True) or {}
    wall = str(data.get('wall', '')).upper()
    face = str(data.get('face', '')).upper()
    clearance  = float(data.get('clearance', 200))
    degagement = float(data.get('degagement', 80))

    if wall not in ('NORTH', 'SOUTH', 'EAST', 'WEST'):
        return jsonify({'ok': False, 'error': f"wall must be NORTH/SOUTH/EAST/WEST, got '{wall}'"}), 400
    if face not in ('A', 'AB', 'B', 'BC', 'C', 'CA'):
        return jsonify({'ok': False, 'error': f"face must be A/AB/B/BC/C/CA, got '{face}'"}), 400

    try:
        ok, res = _hw_transport.execute_calib(
            f'stall_probe({wall},{face},{clearance:.0f},{degagement:.0f})',
            ack_timeout_ms=3000, calib_timeout_ms=45000)
        parsed = _parse_calib_report(res)
        kind = parsed.get('kind')
        if not ok:
            return jsonify({'ok': False,
                            'error': f'transport error — raw={res!r}',
                            'raw': res})
        if kind == 'error':
            return jsonify({'ok': False, 'error': _calib_error_from_raw(res), 'raw': res})
        if kind != 'stall_probe':
            return jsonify({'ok': False,
                            'error': f'unexpected reply (kind={kind!r}) — raw={res!r}',
                            'raw': res})
        return jsonify({
            'ok':              True,
            'wall':            parsed.get('wall', wall),
            'face':            parsed.get('face', face),
            'x':               parsed.get('x', 0.0),
            'y':               parsed.get('y', 0.0),
            'theta':           parsed.get('theta', 0.0),
            'stalled':         int(parsed.get('stalled', 0)),
            'dur_ms':          int(parsed.get('dur_ms', 0)),
            'travel_mm':       parsed.get('travel_mm', 0.0),
            'stall_min_trans': parsed.get('stall_min_trans', 0.0),
            'raw':             res,
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


# ── Embedded camera (ESP32-CAM on the robot) ─────────────────────────────────
# Drives the Detection sub-tab. Fetches a single JPEG from
# http://<robot-cam>/capture, runs blob detection, returns the lateral
# offset + bias hint the strategy needs to align the gripper on the 4
# stock objects (see services/embed_cam.py for the algorithm).
try:
    from services import embed_cam as _embed_cam
except Exception as _ec_err:
    _embed_cam = None
    print(f"[EmbedCam] module unavailable: {_ec_err}")

# Soft-memory cache of the most recent embed-cam detection. Filled by
# every UI button press AND every firmware-triggered request, so the
# Detection sub-tab can re-render the last frame even when the user
# wasn't on the tab when it landed.
_last_embed_result: 'Optional[dict]' = None
_last_embed_jpeg_b64: 'Optional[str]' = None
_last_embed_raw_b64:  'Optional[str]' = None
_last_embed_t: float = 0.0
_last_embed_source: str = ''
_last_embed_lock = threading.Lock()


def _emit_embed_detect_status(stage: str, source: str = 'ui',
                              extra: dict = None) -> None:
    """Push a lightweight 'request fired / fetching / done / error'
    pill to the Detection sub-tab status feed. Lets the operator see
    a firmware-triggered detection LANDING on the host even when the
    camera fetch is still in flight (~1-2 s) — without this we only
    update once the JPEG is decoded, which masks 'host received the
    request but no reply could be sent in time'.

      stage   : 'request' | 'fetched' | 'analyzed' | 'done' | 'error'
      source  : 'firmware' (T:vis embed_detect) | 'ui' (button)
      extra   : appended to the status payload (n, offset, reason…)
    """
    payload = {'stage': stage, 'source': source,
               't_ms': int(time.time() * 1000)}
    if extra:
        payload.update(extra)
    try:
        socketio.emit('vision_feed', {
            'feed_id':  'detect_status',
            'pipeline': 'embed_cam',
            'kind':     'json',
            'data':     payload,
            'meta':     {'label': 'Embed cam · status'},
        })
    except Exception as e:
        print(f"[EmbedCam] status emit failed: {e}")


def _emit_embed_detect_feeds(result: dict, source: str = 'ui',
                             jpeg_b64: 'Optional[str]' = None,
                             raw_b64:  'Optional[str]' = None) -> None:
    """Publish a detection result to the Detection sub-tab.
    Pre-encoded `jpeg_b64` / `raw_b64` skip re-encoding when replaying.
    `raw_b64` is the JPEG as it came off the ESP32 wire — the UI shows
    it next to the annotated preview so the operator can tell a fetch
    failure apart from an ArUco miss without leaving the page."""
    if _embed_cam is None:
        _emit_embed_detect_status('error', source,
                                  {'reason': 'embed_cam-unavailable'})
        try:
            socketio.emit('vision_feed', {
                'feed_id':  'detect_results',
                'pipeline': 'embed_cam',
                'kind':     'json',
                'data':     [{'name': 'error', 'status': 'fail',
                              'value': 'embed_cam module unavailable'}],
                'meta':     {'label': 'Embed cam · checks'},
            })
        except Exception:
            pass
        return
    import base64 as _b64
    try:
        if raw_b64 is None:
            raw_bytes = result.get('raw_jpeg')
            if raw_bytes:
                raw_b64 = _b64.b64encode(raw_bytes).decode()
        if raw_b64:
            socketio.emit('vision_feed', {
                'feed_id':  'detect_raw',
                'pipeline': 'embed_cam',
                'kind':     'frame',
                'jpeg':     raw_b64,
                'meta':     {'label': f'Embed cam · raw ({source})'},
            })
        if jpeg_b64 is None:
            jpeg = _embed_cam.preview_to_jpeg(result)
            if jpeg:
                jpeg_b64 = _b64.b64encode(jpeg).decode()
        if jpeg_b64:
            socketio.emit('vision_feed', {
                'feed_id':  'detect_preview',
                'pipeline': 'embed_cam',
                'kind':     'frame',
                'jpeg':     jpeg_b64,
                'meta':     {'label': f'Embed cam · ESP32 ({source})'},
            })
        checks = _embed_cam.result_to_checks(result)
        socketio.emit('vision_feed', {
            'feed_id':  'detect_results',
            'pipeline': 'embed_cam',
            'kind':     'json',
            'data':     checks,
            'meta':     {'label': f'Embed cam · checks ({source})'},
        })
        _emit_embed_detect_status(
            'done' if not result.get('error') else 'error',
            source,
            {
                'n':          int(result.get('n', 0)),
                'expected':   int(result.get('expected', 4)),
                'offset_mm':  float(result.get('offset_mm', 0.0)),
                'team':       str(result.get('team', 'unknown')),
                'fetch_ms':   int(result.get('fetch_ms', 0)),
                'decode_ms':  int(result.get('decode_ms', 0)),
                'analyze_ms': int(result.get('analyze_ms', 0)),
                'source_kind': str(result.get('source_kind', '')),
                'reason':     result.get('error'),
            })
    except Exception as e:
        print(f"[EmbedCam] feed emit failed: {e}")
        _emit_embed_detect_status('error', source, {'reason': str(e)})


def _run_embed_detect(source: str, team_override: 'Optional[str]' = None) -> dict:
    """Single code path for the UI button AND firmware requests.
    Fetches the frame, runs detection, stores the result in soft
    memory, emits feeds, and returns the result dict."""
    global _last_embed_result, _last_embed_jpeg_b64, _last_embed_raw_b64
    global _last_embed_t, _last_embed_source
    if _embed_cam is None:
        _emit_embed_detect_feeds({'error': 'embed_cam-unavailable'},
                                 source=source)
        return {'error': 'embed_cam-unavailable', 'n': 0, 'valid': False}
    _emit_embed_detect_status('request', source, {'team': team_override})
    _emit_embed_detect_status('fetching', source)
    t0 = time.monotonic()
    result = _embed_cam.detect_once(team_override=team_override)
    dt_ms = int((time.monotonic() - t0) * 1000)
    result['_fetch_ms'] = dt_ms
    import base64 as _b64
    # Raw bytes already came off the wire — just b64 them for transport.
    raw_b64 = _b64.b64encode(result['raw_jpeg']).decode() if result.get('raw_jpeg') else None
    jpeg = _embed_cam.preview_to_jpeg(result)
    jpeg_b64 = _b64.b64encode(jpeg).decode() if jpeg else None
    with _last_embed_lock:
        _last_embed_result   = result
        _last_embed_jpeg_b64 = jpeg_b64
        _last_embed_raw_b64  = raw_b64
        _last_embed_t        = time.time()
        _last_embed_source   = source
    _vlog(
        f"embed_detect done in {dt_ms}ms "
        f"(fetch={result.get('fetch_ms',0)}ms "
        f"decode={result.get('decode_ms',0)}ms "
        f"analyze={result.get('analyze_ms',0)}ms): "
        f"n={result.get('n', 0)}/{result.get('expected', 4)} "
        f"offset={result.get('offset_mm', 0.0):+.1f}mm "
        f"valid={int(bool(result.get('valid', False)))} "
        f"reason={result.get('error') or '-'} "
        f"src={source}")
    try:
        from services.match_logger import MATCH_LOGGER
        MATCH_LOGGER.log('espcam',
            f"src={source} dt={dt_ms}ms "
            f"f={result.get('fetch_ms',0)}/d={result.get('decode_ms',0)}/"
            f"a={result.get('analyze_ms',0)}ms "
            f"n={result.get('n',0)}/"
            f"{result.get('expected',4)} off={result.get('offset_mm',0.0):+.1f}"
            f" valid={int(bool(result.get('valid', False)))} "
            f"reason={result.get('error') or '-'}")
    except Exception:
        pass
    _emit_embed_detect_feeds(result, source=source,
                             jpeg_b64=jpeg_b64, raw_b64=raw_b64)
    return result


@app.route('/api/embed_cam/detect', methods=['POST'])
def api_embed_cam_detect():
    """Mimics the firmware request flow: runs `_run_embed_detect`
    which caches the result in soft memory + emits feeds. Optional
    body `{config: {...}}` patches tuning knobs first."""
    if _embed_cam is None:
        return jsonify({'ok': False, 'error': 'embed_cam unavailable'}), 503
    body = request.get_json(silent=True) or {}
    if isinstance(body.get('config'), dict):
        _embed_cam.set_config(body['config'])
    result = _run_embed_detect(source='ui')
    return jsonify({'ok': True, 'result': _embed_cam.result_to_json(result)})


@app.route('/api/embed_cam/last', methods=['GET'])
def api_embed_cam_last():
    """Return the cached result + JPEG of the most recent detection.
    The Detection sub-tab calls this on activation so the preview tile
    shows the last frame the robot processed, even after a reload."""
    with _last_embed_lock:
        if _last_embed_result is None:
            return jsonify({'ok': False, 'error': 'no detection yet'})
        return jsonify({
            'ok':       True,
            'result':   _embed_cam.result_to_json(_last_embed_result)
                        if _embed_cam else _last_embed_result,
            'jpeg_b64': _last_embed_jpeg_b64,
            'raw_b64':  _last_embed_raw_b64,
            't':        _last_embed_t,
            'source':   _last_embed_source,
        })


@app.route('/api/embed_cam/replay', methods=['POST'])
def api_embed_cam_replay():
    """Re-emits the cached detection feeds without re-fetching. Used
    by the UI when entering the Detection tab so the preview repaints."""
    with _last_embed_lock:
        if _last_embed_result is None:
            return jsonify({'ok': False, 'error': 'no detection yet'})
        _emit_embed_detect_feeds(_last_embed_result,
                                 source=f"replay/{_last_embed_source}",
                                 jpeg_b64=_last_embed_jpeg_b64,
                                 raw_b64=_last_embed_raw_b64)
    return jsonify({'ok': True})


@app.route('/api/embed_cam/config', methods=['GET', 'POST'])
def api_embed_cam_config():
    if _embed_cam is None:
        return jsonify({'ok': False, 'error': 'embed_cam unavailable'}), 503
    if request.method == 'POST':
        body = request.get_json(silent=True) or {}
        before = _embed_cam.get_config()
        # IP-rotation convenience: when the Detection-tab URL field
        # changes host (e.g. 192.168.1.81 → 192.168.1.42) and the
        # streamer URLs were tracking the old host, auto-update them
        # so the operator doesn't have to retype URLs in three places.
        # Skipped if the user explicitly passes mjpeg_url / stream_url
        # in the same request.
        if 'url' in body and 'mjpeg_url' not in body and 'stream_url' not in body:
            try:
                from urllib.parse import urlparse, urlunparse
                old_cap = urlparse(str(before.get('url', '')))
                new_cap = urlparse(str(body.get('url', '')))
                if (old_cap.hostname and new_cap.hostname
                        and old_cap.hostname != new_cap.hostname):
                    for k in ('mjpeg_url', 'stream_url'):
                        old = urlparse(str(before.get(k, '')))
                        if old.hostname == old_cap.hostname:
                            port = f":{old.port}" if old.port else ''
                            body[k] = urlunparse((
                                old.scheme or new_cap.scheme,
                                f"{new_cap.hostname}{port}",
                                old.path, '', '', ''))
            except Exception:
                pass
        _embed_cam.set_config(body)
        after = _embed_cam.get_config()
        # `url` joins the streamer-restart key list: changing the IP
        # via the Detection-tab URL field must restart the grabber.
        streamer_keys = ('use_streamer', 'stream_url', 'mjpeg_url',
                         'fetch_timeout_s', 'url')
        if any(before.get(k) != after.get(k) for k in streamer_keys):
            try:
                _embed_cam.start_streamer()  # idempotent restart
            except Exception as e:
                print(f"[EmbedCam] streamer restart failed: {e}")
    return jsonify({'ok': True, 'config': _embed_cam.get_config()})


@app.route('/api/embed_cam/streamer', methods=['GET', 'POST'])
def api_embed_cam_streamer():
    """Status + control endpoint for the persistent MJPEG grabber.
    GET → current status (state, frames seen, age of cached frame, …).
    POST {action: 'start'|'stop'|'restart'} → toggle the reader thread."""
    if _embed_cam is None:
        return jsonify({'ok': False, 'error': 'embed_cam unavailable'}), 503
    if request.method == 'POST':
        body = request.get_json(silent=True) or {}
        action = str(body.get('action', '')).lower()
        if action == 'stop':
            _embed_cam.stop_streamer()
        elif action in ('start', 'restart'):
            _embed_cam.start_streamer()
        else:
            return jsonify({'ok': False,
                            'error': f'unknown action: {action!r}'}), 400
    return jsonify({'ok': True, 'status': _embed_cam.streamer_status()})


# ── Vision API ────────────────────────────────────────────────────────────────

@app.route('/api/vision/state')
def api_vision_state():
    if _vision is None:
        return jsonify({'cv2_available': False, 'enabled': False})
    return jsonify(_vision.get_state())


@app.route('/api/vision_camera/status')
def api_vision_camera_status():
    """Snapshot of the in-process FrameSource. The topbar polls this to
    show a green/red dot + last error."""
    try:
        import vision_source as _vs
        src = _vs.get()
        if src is None:
            return jsonify({'state': 'idle', 'last_error': 'FrameSource not started'})
        return jsonify(src.status())
    except Exception as e:
        return jsonify({'state': 'unknown', 'last_error': str(e)})


def _restart_active_pipeline() -> 'dict':
    """Restart the active pipeline's worker. Source nodes are stateless
    now (they read from the shared FrameSource), so this is mostly a
    courtesy — useful when a node param change wants a clean tick boundary.

    `shutdown()` clears the pipeline's SocketIO feed subscribers — re-wire
    them after `enable()` so the UI keeps receiving vision_feed events."""
    if _pipeline_registry is None:
        return {'restarted': False, 'reason': 'no registry'}
    p = _pipeline_registry.active()
    if p is None:
        return {'restarted': False, 'reason': 'no active pipeline'}
    try:
        was_enabled = bool(p.enabled)
        p.shutdown()
        if was_enabled:
            p.enable()
            _wire_pipeline_feeds(p)
            p.set_feed_gate(lambda _feed_id: _socketio_clients_count() > 0)
        return {'restarted': True, 'pipeline': p.name, 're_enabled': was_enabled}
    except Exception as e:
        return {'restarted': False, 'reason': f'{e}'}


@app.route('/api/vision_camera/source', methods=['POST'])
def api_vision_camera_source():
    """Runtime override: change FrameSource kind/path WITHOUT persisting
    to vision_config.json. Cleared on holOS restart or via .../source/clear.
    The pipeline keeps running — source nodes just start seeing frames
    from the new source on the next tick."""
    data = request.get_json(force=True) or {}
    kind = str(data.get('source_kind', '')).strip()
    path = str(data.get('source_path', '')).strip()
    if kind not in ('video', 'camera', 'image'):
        return jsonify({'ok': False, 'error': f'bad source_kind: {kind!r}'}), 400
    if not path:
        return jsonify({'ok': False, 'error': 'source_path required'}), 400
    try:
        import vision_source as _vs
        src = _vs.get()
        if src is None:
            return jsonify({'ok': False, 'error': 'FrameSource not started'}), 503
        res = src.set_source(kind, path)
        return jsonify({'ok': True, 'result': res})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/vision_camera/source/clear', methods=['POST'])
def api_vision_camera_source_clear():
    """Drop the runtime override and re-open on the disk config."""
    try:
        import vision_source as _vs
        src = _vs.get()
        if src is None:
            return jsonify({'ok': False, 'error': 'FrameSource not started'}), 503
        res = src.clear_override()
        return jsonify({'ok': True, 'result': res})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


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
    res    = _vision.set_source(source)
    if not res.get('ok'):
        brain.log(f"[VISION] Source open failed: {res.get('error')}")
    else:
        brain.log(f"[VISION] Source opened: {res.get('source', source)}")
    return jsonify(res)


@app.route('/api/vision/browse')
def api_vision_browse():
    """File-browser endpoint for the editor's source-path picker.

    Sandboxed to the holOS root (`software/..`) so the browser can't walk
    out of the project. Returns the directory listing for the requested
    relative path, plus the parent ref and an absolute path the caller
    can pass back to set_params.

    Query params:
        path: relative path under the project root (default = software/vision)
        kind: 'video' | 'image' | 'all' (filters files; dirs always shown)
    """
    project_root = os.path.abspath(os.path.join(_HERE, '..'))
    rel = request.args.get('path', 'software/vision') or 'software/vision'
    kind = request.args.get('kind', 'all')
    video_ext = ('.mp4', '.mkv', '.avi', '.mov', '.webm')
    image_ext = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp')
    if kind == 'video':
        allowed = video_ext
    elif kind == 'image':
        allowed = image_ext
    else:
        allowed = video_ext + image_ext

    # Normalize the requested path. The caller often passes whatever the
    # source node's `path` param currently holds — that might be a FILE
    # (e.g. "software/vision/data/foo.mp4"), an absolute path from a
    # different drive, or just garbage from a stale config. Be lenient:
    #   - strip leading slashes / drive letters that would break confinement
    #   - if the path resolves to a file, browse its parent directory
    #   - if confinement fails, fall back to software/vision
    rel = rel.replace('\\', '/').lstrip('/').lstrip('\\')
    # Strip Windows drive prefix if user pasted an abs path
    if len(rel) >= 2 and rel[1] == ':':
        rel = rel[2:].lstrip('/').lstrip('\\')

    target = os.path.abspath(os.path.join(project_root, rel))
    # If user passed a file path, use its parent
    if os.path.isfile(target):
        target = os.path.dirname(target)

    try:
        common = os.path.commonpath([project_root, target])
    except ValueError:
        common = ''
    if common != project_root or not os.path.isdir(target):
        # Fall back to software/vision so the user always lands somewhere.
        fallback = os.path.join(project_root, 'software', 'vision')
        if os.path.isdir(fallback):
            target = fallback
        else:
            target = project_root

    entries = []
    try:
        for name in sorted(os.listdir(target), key=str.lower):
            if name.startswith('.'):
                continue
            full = os.path.join(target, name)
            is_dir = os.path.isdir(full)
            if not is_dir and not name.lower().endswith(allowed):
                continue
            try:
                size = os.path.getsize(full) if not is_dir else 0
            except OSError:
                size = 0
            entries.append({
                'name':  name,
                'is_dir': is_dir,
                'rel':   os.path.relpath(full, project_root).replace('\\', '/'),
                'abs':   full,
                'size_mb': round(size / 1024 / 1024, 2),
            })
    except OSError as e:
        return jsonify({'ok': False, 'error': str(e)}), 500

    rel_norm = os.path.relpath(target, project_root).replace('\\', '/')
    parent = (
        os.path.relpath(os.path.dirname(target), project_root).replace('\\', '/')
        if target != project_root else ''
    )
    return jsonify({
        'ok': True,
        'project_root': project_root,
        'cwd':    rel_norm if rel_norm != '.' else '',
        'parent': parent if parent != '.' else '',
        'entries': entries,
    })


@app.route('/api/vision/list_files')
def api_vision_list_files():
    """List candidate video/image files under software/vision/ + data/ for
    the datalist autocomplete in the source picker. Browsers can't expose
    full paths, so we let the user pick from a server-side listing."""
    kind = request.args.get('kind', 'all')   # 'video' | 'image' | 'all'
    video_ext = ('.mp4', '.mkv', '.avi', '.mov', '.webm')
    image_ext = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp')
    if kind == 'video':
        allowed = video_ext
    elif kind == 'image':
        allowed = image_ext
    else:
        allowed = video_ext + image_ext

    roots = [
        os.path.join(_HERE, 'vision'),
        os.path.join(_HERE, 'vision', 'data'),
    ]
    seen = set()
    files = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        try:
            for entry in sorted(os.listdir(root)):
                full = os.path.join(root, entry)
                if not os.path.isfile(full):
                    continue
                if not entry.lower().endswith(allowed):
                    continue
                # Display: relative to software/vision/ for compactness.
                rel = os.path.relpath(full, os.path.join(_HERE, 'vision'))
                if rel in seen:
                    continue
                seen.add(rel)
                files.append({
                    'rel': rel,
                    'abs': full,
                    'size_mb': round(os.path.getsize(full) / 1024 / 1024, 1),
                })
        except OSError:
            pass
    return jsonify({'files': files, 'roots': roots, 'kind': kind})


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


# ── Pipeline registry (multi-source) endpoints ──────────────────────────────

def _pipeline_to_payload(p):
    return {
        'name':       p.name,
        'fps_limit':  p.fps_limit,
        'enabled':    p.enabled,
        'is_active':  (_pipeline_registry is not None
                       and _pipeline_registry.active_name == p.name),
        'is_default': (_pipeline_registry is not None
                       and _pipeline_registry.default_name == p.name),
        'state':      p.state(),
        'graph':      p.to_dict(),
    }


def _save_pipelines():
    """No-op now that pipelines live in vision_pipelines_def.py.
    Kept as a stub so existing call sites in this file (mostly the
    PUT/DELETE pipeline endpoints — also kept for API back-compat
    even though the React editor that drove them is gone) don't error.
    To change a pipeline, edit vision_pipelines_def.py and restart."""
    return


def _build_pipeline_from_dict(d: dict, force_video_paused: bool = False):
    """Reconstruct a Pipeline from a saved graph dict.

    `force_video_paused`: when True, any source.video node's `playback` is
    rewritten to `'pause'` regardless of what was saved. We do this on
    server startup so a previously-running video doesn't start hammering
    cv2.VideoCapture as soon as the process boots — let the user hit ▶.
    """
    p = Pipeline(name=d['name'], fps_limit=int(d.get('fps_limit', 25)))
    nodes = d.get('nodes', [])
    # First pass: instantiate all nodes
    for nd in nodes:
        kind = nd['kind']
        if kind not in NODE_KINDS:
            brain.log(f"[VISION] Unknown node kind {kind!r} in saved pipeline "
                      f"{d['name']!r} — skipped")
            continue
        cls = NODE_KINDS[kind]
        params = dict(nd.get('params', {}))
        if force_video_paused and kind == 'source.video':
            params['playback'] = 'pause'
        p.add_node(nd['id'], kind, cls(params))
        # Restore the saved editor position on the record so the next
        # Pipeline.to_dict() round-trips it back to the editor.
        pos = nd.get('position')
        if isinstance(pos, dict) and 'x' in pos and 'y' in pos:
            rec = p._nodes.get(nd['id'])
            if rec is not None:
                rec.position = {'x': float(pos['x']), 'y': float(pos['y'])}
    # Second pass: wire edges
    for nd in nodes:
        for inp in nd.get('inputs', []):
            try:
                p.connect(inp['src_node'], inp['src_port'],
                          nd['id'],        inp['port'])
            except Exception as e:
                brain.log(f"[VISION] connect failed: {e}")
    return p


def _load_pipelines():
    """Build pipelines from `vision_pipelines_def.py` (pure Python — no
    JSON anymore). Each entry in PIPELINES is called, registered, wired
    up to the dashboard SocketIO emit, and AUTO-ENABLED so vision starts
    immediately when holOS boots. The user no longer has to flip the
    Vision toggle to see anything."""
    if _pipeline_registry is None:
        return None
    try:
        import importlib
        import vision_pipelines_def as _vpdef
        # Reload on every call so live-edits to the file pick up without
        # restarting the whole server (mirror of what the runner does).
        importlib.reload(_vpdef)
    except Exception as e:
        brain.log(f'[VISION] Could not import vision_pipelines_def: {e}')
        return None

    built = []
    for name, builder in (_vpdef.PIPELINES or {}).items():
        try:
            p = builder()
            if p.name != name:
                p.name = name
            _pipeline_registry.register(p)
            _wire_pipeline_feeds(p)
            # Skip the JPEG encode + emit when nobody is watching. Big CPU
            # win on the Jetson when holOS runs headless or the user has no
            # browser tab open.
            p.set_feed_gate(lambda _feed_id: _socketio_clients_count() > 0)
            # Auto-enable every registered pipeline. Each runs in its own
            # worker thread so they don't fight for CPU at the Python level
            # (cv2 ops release the GIL anyway).
            _pipeline_registry.set_enabled(name, True)
            built.append(name)
        except Exception as e:
            brain.log(f'[VISION] build_{name} failed: {e}')

    default_name = getattr(_vpdef, 'DEFAULT_PIPELINE', None)
    if default_name and _pipeline_registry.get(default_name) is not None:
        _pipeline_registry.set_default(default_name)
        _pipeline_registry.set_active(default_name)   # focus only, no exclusivity
    brain.log(f'[VISION] Built {len(built)} pipeline(s) from '
              f'vision_pipelines_def.py: {built}'
              + (f' — default focus: {default_name!r}' if default_name else '')
              + ' (all auto-enabled)')
    return default_name


def _wire_pipeline_feeds(p):
    """Hook every output-kind node's feed onto SocketIO emissions.

    Supports three payload kinds:
        bytes/bytearray  → 'frame' (base64-encoded JPEG)
        dict / list      → 'pose_list' or 'json' (decided by meta.kind)
    The same SocketIO event (`vision_feed`) carries everything; the
    dashboard inspects `kind` to render appropriately.

    Fall back to the node id when feed_id is empty (matches the output
    nodes' _effective_feed_id default). Idempotent — clears existing
    subscribers first.
    """
    p.clear_feed_subscribers()
    OUTPUT_CLASSES = {'OutputNode', 'OutputPoseListNode',
                      'OutputObjectsNode', 'OutputJsonNode',
                      'OutputArucoListNode'}
    for rec_id in p.node_ids():
        node = p.get_node(rec_id)
        if node is None:
            continue
        if node.__class__.__name__ not in OUTPUT_CLASSES:
            continue
        feed_id = (node._effective_feed_id()
                   if hasattr(node, '_effective_feed_id')
                   else (node.get_params().get('feed_id') or rec_id))
        if not feed_id:
            continue
        # Default-arg capture so closures are bound BY VALUE, not by
        # reference (avoids the loop-variable trap when multiple outputs
        # exist in the same graph).
        def make_cb(fid=feed_id, pipeline_name=p.name):
            def cb(payload, meta):
                import base64 as _b64
                try:
                    msg = {
                        'feed_id':  fid,
                        'pipeline': pipeline_name,
                        'meta':     meta or {},
                    }
                    if isinstance(payload, (bytes, bytearray)):
                        msg['kind'] = 'frame'
                        b64 = _b64.b64encode(bytes(payload)).decode()
                        msg['jpeg'] = b64
                        _net_stats_add(len(b64) + 64)   # +64 ≈ JSON envelope
                    else:
                        # Non-frame payload (pose_list / json). The output
                        # node sets meta['kind'] so the dashboard knows.
                        msg['kind'] = (meta or {}).get('kind', 'json')
                        msg['data'] = payload
                        _net_stats_add(256)             # JSON payloads are tiny
                    socketio.emit('vision_feed', msg)
                except Exception as e:
                    print(f'[VISION] feed emit {fid} failed: {e}')
            return cb
        p.subscribe_feed(feed_id, make_cb())


def _get_localization_node():
    """Walk the active pipeline, return its localization node + record (or None)."""
    if _pipeline_registry is None:
        return None, None
    p = _pipeline_registry.active()
    if p is None or not p.enabled:
        return None, None
    with p._lock:
        for nid, rec in p._nodes.items():
            if rec.kind == 'localization':
                return rec.instance, rec
    return None, None


def _get_pose_source_record():
    """Return the record whose `pose_list` output is the most-corrected
    one available in the active pipeline. Preference order:
        parallax  → standalone parallax node, applies an EXPLICIT
                    cam_xyz correction on top of the tracker output.
                    This is the pose we send to the firmware and show
                    on the debug page.
        localization → tracker-internal correction (camera estimated
                    via solvePnP). Fallback when no parallax node is
                    wired in the graph.
    Returns (record, kind_str) or (None, None)."""
    if _pipeline_registry is None:
        return None, None
    p = _pipeline_registry.active()
    if p is None or not p.enabled:
        return None, None
    with p._lock:
        nodes = dict(p._nodes)
    # Prefer parallax over localization. Walking the dict twice is fine —
    # we have at most a handful of nodes in any pipeline.
    for nid, rec in nodes.items():
        if rec.kind == 'parallax':
            return rec, 'parallax'
    for nid, rec in nodes.items():
        if rec.kind == 'localization':
            return rec, 'localization'
    return None, None


def _get_rectify_node():
    """Walk the active pipeline, return its rectify node + record (or None).
    The rectify node owns the TableRectifier; locking the homography during
    the preparation phase happens on its rectifier instance."""
    if _pipeline_registry is None:
        return None, None
    p = _pipeline_registry.active()
    if p is None or not p.enabled:
        return None, None
    with p._lock:
        for nid, rec in p._nodes.items():
            if rec.kind == 'rectify':
                return rec.instance, rec
    return None, None


def _wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to (-π, π]."""
    a = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if a <= 0:
        a += 2.0 * math.pi
    return a - math.pi


def _recalage_pick_own_tag(known_x_mm, known_y_mm,
                           known_theta_rad=None,
                           tolerance_mm: float = 600.0):
    """At recalage time the robot is at a known (x, y, theta) world-frame
    pose. Find the OWN-team ArUco closest to that position and lock it
    as the OWN tag.

    The team is read from `sim_state['team']` (set by the IHM team
    poller — single source of truth, the physical color switch on the
    robot). We DO NOT infer the team from the picked tag id, because
    that race-conditions with the team poller: if the user mounts a
    blue-range tag on a yellow-IHM robot, inferring would flip the team
    until the next poller tick re-flips it back. So the team is fixed
    by the IHM and we only ever pick a tag that's already in the OWN
    range.

    When `known_theta_rad` is provided, also capture the heading offset
    between the robot frame and the (arbitrarily mounted) tag frame:
        heading_offset_rad = wrap_pi(known_theta_rad - tag_theta_rad)
    Stored in the module-level `_vision_heading_offset_rad`; applied on
    every subsequent `pose_request` so the firmware always receives a
    pose in the ROBOT frame.

    Returns dict {tag_id, team, vision_pose: {x_mm, y_mm, theta_rad}}
    on success (theta_rad is the corrected/robot-frame heading when
    known_theta_rad was provided, else the raw tag heading), None on
    failure (logs the reason). Side effects: locks track_ids to
    [tag_id] + opp_ids on the localization node, sets
    `_vision_heading_offset_rad`, and writes a full diagnostic
    snapshot to `_vision_calibration_snapshot` for the debug page."""
    global _vision_heading_offset_rad, _vision_calibration_snapshot
    inst, rec = _get_localization_node()
    if inst is None:
        _vlog('recalage failed: no localization node in active pipeline', 'err')
        return None
    p = _pipeline_registry.active()

    # Pull pose_list from the most-corrected source (parallax > localization).
    src_rec, src_kind = _get_pose_source_record()
    if src_rec is None:
        src_rec, src_kind = rec, 'localization'
    with p._lock:
        poses = list(src_rec.outputs.get('pose_list') or [])
    if not poses:
        _vlog(f'recalage failed: no poses emitted by {src_kind} node yet '
              f'(homography locked? localization running?)', 'err')
        return None

    # Determine OWN range from current team (IHM-driven). 'blue' →
    # [1..5], 'yellow' → [6..10]. Default 'blue' matches the topbar.
    team = sim_state.get('team', 'blue')
    if team == 'yellow':
        own_range = set(range(6, 11))
        opp_ids   = list(range(1, 6))
    else:
        own_range = set(range(1, 6))
        opp_ids   = list(range(6, 11))

    # Pick the closest OWN-range tag to the known position. Use the
    # parallax-corrected x_mm (post-correction is what the firmware
    # consumes downstream); fall back to naive only if missing.
    best, best_d2 = None, float('inf')
    candidates = []   # for diag log
    for q in poses:
        if not isinstance(q, dict):
            continue
        tid = q.get('tag_id')
        if tid is None:
            continue
        x = q.get('x_mm') if q.get('x_mm') is not None else q.get('naive_x_mm')
        y = q.get('y_mm') if q.get('y_mm') is not None else q.get('naive_y_mm')
        if x is None or y is None:
            continue
        d2 = (float(x) - known_x_mm) ** 2 + (float(y) - known_y_mm) ** 2
        in_own = int(tid) in own_range
        candidates.append((int(tid), d2 ** 0.5, in_own))
        if not in_own:
            continue
        if d2 < best_d2:
            best_d2, best = d2, q

    cand_str = ', '.join(f"#{tid}@{d:.0f}mm{'(own)' if own else '(other)'}"
                          for tid, d, own in sorted(candidates, key=lambda c: c[1]))
    if best is None:
        _vlog(f'recalage failed: no OWN-range tag detected for team={team} '
              f'(own range {sorted(own_range)}). Candidates this tick: '
              f'[{cand_str or "none"}]', 'err')
        return None
    if best_d2 > tolerance_mm * tolerance_mm:
        _vlog(f'recalage failed: closest OWN tag #{best.get("tag_id")} is '
              f'{best_d2 ** 0.5:.0f}mm from known pos ({known_x_mm:.0f}, '
              f'{known_y_mm:.0f}) — > tolerance {tolerance_mm:.0f}mm. '
              f'Source: {src_kind}. All candidates: [{cand_str}]', 'err')
        return None
    tag_id = int(best.get('tag_id', 0))
    if tag_id <= 0:
        _vlog(f'recalage failed: best candidate has invalid tag_id={tag_id}', 'err')
        return None

    # Lock down: only track the picked OWN tag + the opponent range.
    # Drops every other OWN-range tag (eg a spare on the bench) so the
    # tracker won't confuse the actual robot with stray markers.
    new_track = [tag_id] + opp_ids
    try:
        inst.set_params({'team': team, 'track_ids': new_track})
    except Exception as e:
        _vlog(f'recalage: set_params failed: {e}', 'err')
    # No _apply_team call: the team came from sim_state in the first
    # place, so the topbar / pipelines are already consistent.

    corrected_x = float(best.get('x_mm', 0) or 0)
    corrected_y = float(best.get('y_mm', 0) or 0)
    naive_x = (float(best['naive_x_mm'])
               if best.get('naive_x_mm') is not None else corrected_x)
    naive_y = (float(best['naive_y_mm'])
               if best.get('naive_y_mm') is not None else corrected_y)
    raw_tag_theta = float(best.get('theta_rad') or 0.0)
    # Capture the robot↔tag heading offset when the firmware sent its
    # known orientation. The tag is mounted on the robot at an arbitrary
    # angle; this offset is what we apply to every subsequent vision
    # pose so the firmware sees its own heading, not the tag's.
    if known_theta_rad is not None:
        _vision_heading_offset_rad = _wrap_pi(
            float(known_theta_rad) - raw_tag_theta)
        corrected_theta = float(known_theta_rad)
        _vlog(
            f"recalage OK — own tag #{tag_id}, team={team}, "
            f"d={best_d2 ** 0.5:.0f}mm  Δheading="
            f"{math.degrees(_vision_heading_offset_rad):+.1f}° "
            f"(tag={math.degrees(raw_tag_theta):+.1f}°, "
            f"robot={math.degrees(known_theta_rad):+.1f}°)")
    else:
        # Caller didn't pass a known theta — leave the offset alone (may be
        # set from a previous recalage) and echo back the raw tag heading.
        corrected_theta = raw_tag_theta
        _vlog(f"recalage OK — own tag #{tag_id}, team={team}, "
              f"d={best_d2 ** 0.5:.0f}mm  (no heading sync)")

    # Persist a snapshot for the debug page. Single source of truth for
    # the operator: which tag we locked, the (vision − known) residual
    # in xy, and the heading offset between robot and tag frames.
    try:
        robot_z_mm = float(inst._params.get('robot_z_mm', 490.0))
    except Exception:
        robot_z_mm = None
    # Pull the parallax node's actually-used cam_xyz + object_z_mm so
    # the snapshot carries enough state to suggest an "implied z_obj"
    # tuning hint. Falls back to None when there is no parallax node
    # in the graph (e.g. localization-only pipeline).
    cam_xyz_used = None
    parallax_object_z_mm = None
    try:
        if src_kind == 'parallax':
            par_state = src_rec.instance.get_state() or {}
            cam_xyz_used = par_state.get('cam_xyz_used') or par_state.get('cam_xyz_param')
            parallax_object_z_mm = par_state.get('object_z_mm')
    except Exception:
        pass

    # Implied object_z_mm: assuming cam_xyz is correct, what tag height
    # would make the parallax correction map naive_xy exactly onto
    # known_xy? Solves
    #     known = cam + factor*(naive - cam)
    #     factor = (cam_z - z_obj) / cam_z   →   z_obj = cam_z*(1 - factor)
    # for both axes and averages the two estimates (they should agree
    # tightly when cam_xyz is right; large disagreement = the camera
    # position itself is wrong, not just the tag height).
    implied_z_obj = None
    if cam_xyz_used is not None:
        try:
            cx, cy, cz = (float(cam_xyz_used[0]), float(cam_xyz_used[1]),
                          float(cam_xyz_used[2]))
            denom_x = naive_x - cx
            denom_y = naive_y - cy
            factors = []
            if abs(denom_x) > 1.0:
                factors.append((float(known_x_mm) - cx) / denom_x)
            if abs(denom_y) > 1.0:
                factors.append((float(known_y_mm) - cy) / denom_y)
            if factors:
                avg_factor = sum(factors) / len(factors)
                implied_z_obj = cz * (1.0 - avg_factor)
        except Exception:
            pass

    # Diagnostic-only: record this naive/true pair for the debug page.
    _vision_calibration_pairs.append({
        't_mono':  time.monotonic(),
        'tag_id':  tag_id,
        'naive_x': naive_x,
        'naive_y': naive_y,
        'true_x':  float(known_x_mm),
        'true_y':  float(known_y_mm),
    })

    # Auto-tune block removed — object_z_mm now comes from vision_config.json
    # via _apply_team. The solver code below remains dormant for now.
    # Parallax auto-tune disabled — object_z_mm now comes from vision_config.json
    # via _apply_team.
    auto_tuned_z_mm: 'Optional[float]' = None
    auto_tune_status: str = 'disabled (use vision_config.json)'
    auto_tune_rms_mm: 'Optional[float]' = None

    _vision_calibration_snapshot = {
        'captured_at_t':     time.monotonic(),
        'tag_id':            tag_id,
        'team':              team,
        'pose_source':       src_kind,
        'known_pose': {
            'x_mm':      float(known_x_mm),
            'y_mm':      float(known_y_mm),
            'theta_rad': (float(known_theta_rad)
                          if known_theta_rad is not None else None),
        },
        # naive_xy = raw projection of the tag pixel through the locked
        # H, no parallax correction. Reflects where the marker LOOKS to
        # be on the BEV image. The (naive − known) delta is mostly the
        # parallax bias caused by the tag's height above the table.
        'tag_pose_naive': {
            'x_mm':      naive_x,
            'y_mm':      naive_y,
        },
        # corrected_xy = parallax-corrected pose (what the firmware
        # consumes downstream). The (corrected − known) delta is the
        # residual error after parallax: ideally close to zero, larger
        # numbers point to bad cam_xyz / robot_z_mm or a tilted camera.
        'tag_pose_corrected': {
            'x_mm':      corrected_x,
            'y_mm':      corrected_y,
            'theta_rad': raw_tag_theta,
        },
        'xy_offset_naive_mm': {
            'dx_mm': naive_x - float(known_x_mm),
            'dy_mm': naive_y - float(known_y_mm),
        },
        'xy_offset_corrected_mm': {
            'dx_mm': corrected_x - float(known_x_mm),
            'dy_mm': corrected_y - float(known_y_mm),
        },
        'heading_offset_rad':   _vision_heading_offset_rad,
        'robot_z_mm':           robot_z_mm,
        # Tuning aids — current parallax params + a single-pose-derived
        # suggestion + the value that was actually applied this tick
        # (None when auto-tune was skipped/rejected — see auto_tune_status).
        'cam_xyz_used':         cam_xyz_used,
        'parallax_object_z_mm': parallax_object_z_mm,
        'implied_object_z_mm':  implied_z_obj,
        'auto_tuned_z_mm':      auto_tuned_z_mm,
        'auto_tune_status':     auto_tune_status,
        'auto_tune_rms_mm':     auto_tune_rms_mm,
        'pair_count':           len(_vision_calibration_pairs),
    }

    return {
        'tag_id': tag_id,
        'team':   team,
        'vision_pose': {
            'x_mm':      corrected_x,
            'y_mm':      corrected_y,
            'theta_rad': corrected_theta,
        },
    }


def _get_latest_own_pose():
    """Read-only snapshot of the latest 'own'-classified pose from the
    most-corrected pose source in the active pipeline. Returns None when
    the pipeline isn't running, no pose-emitting node exists, or the
    latest tick didn't detect the own robot.

    Source preference: parallax node (explicit cam_xyz correction) >
    localization node (tracker-internal solvePnP correction). Both nodes
    expose the same pose_list shape; the parallax node only overrides
    x_mm / y_mm. classification + tag_id + theta_rad are forwarded from
    upstream.

    When the heading offset has been captured (via a successful
    cal_request), the returned `theta_rad` is corrected to the robot
    frame. `theta_tag_rad` is the raw vision tag heading (useful for
    debug). Without an offset, theta_rad == theta_tag_rad.

    This is a PULL accessor — strategy code (the firmware or the brain)
    queries it on demand. Vision never auto-pushes the robot pose.
    """
    rec, source_kind = _get_pose_source_record()
    if rec is None:
        return None
    p = _pipeline_registry.active()
    if p is None:
        return None
    with p._lock:
        poses = list(rec.outputs.get('pose_list') or [])
    if not poses:
        return None
    # Pick the freshest 'own' pose; fall back to None if none.
    for q in poses:
        if q.get('classification') != 'own':
            continue
        if q.get('x_mm') is None or q.get('y_mm') is None:
            return None
        tag_theta = q.get('theta_rad')
        if (tag_theta is not None
                and _vision_heading_offset_rad is not None):
            robot_theta = _wrap_pi(
                float(tag_theta) + _vision_heading_offset_rad)
        else:
            robot_theta = tag_theta
        return {
            'tag_id':        q.get('tag_id'),
            'x_mm':          q['x_mm'],
            'y_mm':          q['y_mm'],
            'theta_rad':     robot_theta,
            'theta_tag_rad': tag_theta,
            'naive_x_mm':    q.get('naive_x_mm'),
            'naive_y_mm':    q.get('naive_y_mm'),
            'source':        source_kind,
        }
    return None   # no own pose in the latest tick


@app.route('/api/vision/robot_pose', methods=['GET'])
def api_vision_robot_pose():
    """Pull endpoint: returns the latest own-team robot pose if available.
    The caller (strategy code) decides whether to apply it — vision never
    auto-updates robot.pos.

    Response: 200 always with { ok: bool, pose?: {...}, error?: '...' }.
    "No pose yet" is a normal state (homography idle / no own tag in
    frame) and the vision_debug page polls this every 500ms — returning
    a 404 spammed the chromium console with red errors.
    """
    pose = _get_latest_own_pose()
    if pose is None:
        return jsonify({'ok': False, 'error': 'no own-team pose available'})
    return jsonify({'ok': True, 'pose': pose})


def _save_parallax_calibration() -> None:
    """Persist the current team's parallax calibration to disk so the
    next boot can restore it without re-running the firmware multi-pose
    procedure. The file groups configs by team name — calibrations for
    blue and yellow live side-by-side in one JSON.

    Called after every successful auto-tune in `_recalage_pick_own_tag`.
    Failures are non-fatal (logged + ignored) — the running pipeline
    keeps the in-memory tune either way.
    """
    rec, src_kind = _get_pose_source_record()
    if rec is None or src_kind != 'parallax':
        return
    try:
        st = rec.instance.get_state() or {}
        z_obj   = st.get('object_z_mm')
        cam_xyz = st.get('cam_xyz_used') or st.get('cam_xyz_param')
    except Exception:
        return
    if z_obj is None:
        return

    team = sim_state.get('team', 'blue')
    data = {}
    if os.path.exists(PARALLAX_CALIB_PATH):
        try:
            with open(PARALLAX_CALIB_PATH, 'r') as f:
                data = json.load(f) or {}
        except Exception:
            data = {}
    data[team] = {
        'object_z_mm': float(z_obj),
        'cam_xyz':     [float(c) for c in (cam_xyz or [])],
        'pairs':       list(_vision_calibration_pairs),
        't_iso':       time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    try:
        os.makedirs(os.path.dirname(PARALLAX_CALIB_PATH), exist_ok=True)
        with open(PARALLAX_CALIB_PATH, 'w') as f:
            json.dump(data, f, indent=2)
        _vlog(f'parallax calibration saved (team={team}, z_obj={z_obj:.0f}mm, '
              f'{len(_vision_calibration_pairs)} pairs)')
    except Exception as e:
        _vlog(f'parallax calibration save failed: {e}', 'err')


def _load_parallax_calibration_for_team(team: str) -> bool:
    """Restore the saved object_z_mm + (naive, true) pairs for `team`
    into the running pipeline. Idempotent — pushing an already-current
    z_obj is a no-op. Returns True iff something was actually applied.

    Called from `_apply_team` (boot via force=True, and at every team
    flip). When no saved config exists for the team, leaves the
    pipeline at its build-time default.
    """
    if not os.path.exists(PARALLAX_CALIB_PATH):
        return False
    try:
        with open(PARALLAX_CALIB_PATH, 'r') as f:
            data = json.load(f) or {}
    except Exception as e:
        _vlog(f'parallax calibration load failed: {e}', 'err')
        return False
    cfg = data.get(team)
    if not cfg:
        return False

    applied = False
    z_obj = cfg.get('object_z_mm')
    rec, src_kind = _get_pose_source_record()
    if rec is not None and src_kind == 'parallax' and z_obj is not None:
        try:
            rec.instance.set_params({'object_z_mm': float(z_obj)})
            applied = True
        except Exception as e:
            _vlog(f'parallax restore set_params failed: {e}', 'err')

    pairs = cfg.get('pairs') or []
    if pairs:
        # Reseed the pair history so the next force-recalage refines on
        # top of the saved data instead of starting from zero.
        _vision_calibration_pairs.extend(pairs)

    if applied:
        _vlog(f"parallax restored for team={team}: object_z_mm="
              f"{z_obj:.0f}mm, {len(pairs)} pair(s) reseeded "
              f"(saved {cfg.get('t_iso','?')})")
    return applied


def _solve_factor_lsq(pairs: list, cam_x: float, cam_y: float) -> 'Optional[float]':
    """1-D least-squares for the parallax factor with cam_xy held fixed.
    Model per axis: (true - cam) = factor * (naive - cam). Closed form:
        factor = Σ (ex·dx + ey·dy) / Σ (dx² + dy²)
    where d* = naive - cam, e* = true - cam over all pairs and axes
    pooled together. Returns None when the pairs span no parallax range
    (Σ d² ≈ 0, which means naive ≈ cam projection and there's nothing
    for the correction to do)."""
    num = 0.0
    den = 0.0
    for p in pairs:
        dx = p['naive_x'] - cam_x
        dy = p['naive_y'] - cam_y
        ex = p['true_x']  - cam_x
        ey = p['true_y']  - cam_y
        num += ex * dx + ey * dy
        den += dx * dx + dy * dy
    if den < 1.0:
        return None
    return num / den


def _solve_parallax_from_pairs(pairs: list, cam_x: float, cam_y: float,
                                cam_z: float) -> dict:
    """Multi-pose parallax solve with the camera position held fixed.
    The camera mount is physically rigid, so cam_xy / cam_z are taken
    as ground truth from the pipeline params; the only free parameter
    is the tag height, which we surface as the implied object_z_mm.

    Two pairs minimum but more is better — distribute the robot across
    the table to constrain the factor across the full naive range.
    """
    n = len(pairs)
    if n < 1:
        return {'ok': False, 'error': f'need at least 1 pair (have {n})'}

    factor = _solve_factor_lsq(pairs, cam_x, cam_y)
    if factor is None:
        return {'ok': False,
                'error': 'pairs cluster too close to the camera projection — '
                         'move the robot to varied positions'}

    z_obj = cam_z * (1.0 - factor)

    # Residuals after applying the SOLVED factor with FIXED cam_xy.
    residuals = []
    for p in pairs:
        cx_corr = cam_x + factor * (p['naive_x'] - cam_x)
        cy_corr = cam_y + factor * (p['naive_y'] - cam_y)
        dx = cx_corr - p['true_x']
        dy = cy_corr - p['true_y']
        residuals.append({'tag_id': p.get('tag_id'),
                          'dx_mm': dx, 'dy_mm': dy,
                          'norm_mm': (dx*dx + dy*dy) ** 0.5})
    rms = (sum(r['norm_mm']**2 for r in residuals) / n) ** 0.5

    # Per-pair, per-axis factor estimates so the operator can spot
    # outliers (a pair with wildly different factor → bad detection or
    # OTOS off at that position).
    per_pair = []
    for p in pairs:
        dx = p['naive_x'] - cam_x
        dy = p['naive_y'] - cam_y
        ex = p['true_x']  - cam_x
        ey = p['true_y']  - cam_y
        fx = (ex / dx) if abs(dx) > 1.0 else None
        fy = (ey / dy) if abs(dy) > 1.0 else None
        per_pair.append({'tag_id': p.get('tag_id'),
                         'naive_x': p['naive_x'], 'naive_y': p['naive_y'],
                         'true_x':  p['true_x'],  'true_y':  p['true_y'],
                         'factor_x': fx, 'factor_y': fy})

    return {
        'ok':               True,
        'pair_count':       n,
        'cam_x_mm':         cam_x,   # fixed input
        'cam_y_mm':         cam_y,   # fixed input
        'cam_z_mm':         cam_z,   # fixed input
        'factor':           factor,
        'implied_z_obj_mm': z_obj,
        'residuals':        residuals,
        'rms_mm':           rms,
        'per_pair':         per_pair,
    }


@app.route('/api/vision/calibration/pairs', methods=['GET'])
def api_vision_calibration_pairs_get():
    """Snapshot of the captured (naive, true) pairs for the multi-pose
    parallax solver."""
    pairs = list(_vision_calibration_pairs)
    return jsonify({
        'count': len(pairs),
        'pairs': pairs,
    })


@app.route('/api/vision/calibration/pairs', methods=['DELETE'])
def api_vision_calibration_pairs_clear():
    n = len(_vision_calibration_pairs)
    _vision_calibration_pairs.clear()
    _vlog(f'calibration pairs cleared ({n} pairs dropped)')
    return jsonify({'ok': True, 'cleared': n})


@app.route('/api/vision/calibration/solve', methods=['POST'])
def api_vision_calibration_solve():
    """Run the multi-pose parallax solver with cam_xyz held fixed (the
    physical camera mount is rigid — the only thing tag-mount-specific
    that can vary is the tag's height above the table). Returns the
    implied object_z_mm; the user updates ROBOT_Z_MM in
    vision_pipelines_def.py with it and restarts holOS."""
    src_rec, src_kind = _get_pose_source_record()
    cam = None
    if src_rec is not None and src_kind == 'parallax':
        try:
            st = src_rec.instance.get_state() or {}
            cam = st.get('cam_xyz_used') or st.get('cam_xyz_param')
        except Exception:
            cam = None
    if not cam or len(cam) < 3:
        return jsonify({'ok': False,
                        'error': 'cam_xyz unavailable — is the parallax node '
                                 'running?'}), 400
    cx, cy, cz = float(cam[0]), float(cam[1]), float(cam[2])
    if cz < 100.0:
        return jsonify({'ok': False,
                        'error': f'cam_z={cz:.0f}mm is implausibly low — '
                                 'check CAMERA_Z_MM'}), 400

    res = _solve_parallax_from_pairs(list(_vision_calibration_pairs),
                                     cx, cy, cz)
    if res.get('ok'):
        _vlog(f"solver: {res['pair_count']} pairs (cam fixed at "
              f"{cx:.0f},{cy:.0f},{cz:.0f}) → z_obj="
              f"{res['implied_z_obj_mm']:.0f}mm  rms={res['rms_mm']:.1f}mm")
    else:
        _vlog(f"solver: failed — {res.get('error')}", 'warn')
    return jsonify(res)


@app.route('/api/vision/force_recalage', methods=['POST'])
def api_vision_force_recalage():
    """Debug aid — runs the OWN-tag pick + heading-offset capture using
    the robot's current OTOS pose, WITHOUT firing the firmware motion
    routine. Useful to debug `_recalage_pick_own_tag` independently
    when the snapshot isn't populating: position the robot manually at
    a known pose, hit this endpoint, watch the vision log.

    Body (optional JSON): {x_mm, y_mm, theta_rad} to override the OTOS
    pose. Defaults to robot.pos.x / robot.pos.y / robot.theta.
    """
    try:
        body = request.get_json(silent=True) or {}
    except Exception:
        body = {}
    try:
        kx = float(body.get('x_mm', robot.pos.x))
        ky = float(body.get('y_mm', robot.pos.y))
        kt = float(body.get('theta_rad', robot.theta))
    except (TypeError, ValueError, AttributeError) as e:
        return jsonify({'ok': False, 'error': f'pose unavailable: {e}'}), 400

    _vlog(f'force_recalage: triggered manually (x={kx:.0f} y={ky:.0f} '
          f't={math.degrees(kt):+.1f}°)')
    result = _recalage_pick_own_tag(kx, ky, known_theta_rad=kt)
    if result is None:
        return jsonify({'ok': False,
                        'error': 'see vision log for the failure reason'})
    return jsonify({'ok': True, **result})


@app.route('/api/vision/calibration', methods=['GET'])
def api_vision_calibration():
    """Snapshot of the recalage handshake: known robot pose, raw vision
    tag pose at calibration, parallax xy residual, heading offset, the
    locked-in tag id + team, robot_z_mm. Drives the debug page so the
    operator can sanity-check the numbers without grepping logs.

    Plus the current rectify capture phase ('idle' / 'capturing' /
    'locked') and homography lock state — same JSON so the page only
    polls one endpoint.

    Response: 200 always. snapshot=null when no recalage has fired yet.
    """
    rect_node, _ = _get_rectify_node()
    rect_state = rect_node.get_state() if rect_node is not None else None

    snap = _vision_calibration_snapshot
    snap_payload = None
    if snap is not None:
        snap_payload = dict(snap)
        snap_payload['captured_age_s'] = (time.monotonic()
                                          - snap['captured_at_t'])
        ho = snap.get('heading_offset_rad')
        snap_payload['heading_offset_deg'] = (math.degrees(ho)
                                              if ho is not None else None)

    # Tail of the rolling vision-event log — debug page polls this
    # endpoint at 2 Hz and renders the entries. Copy each entry so
    # appending `age_s` doesn't mutate the buffer in place.
    now_t = time.monotonic()
    log_tail = [{**entry, 'age_s': now_t - entry.get('t_mono', now_t)}
                for entry in list(_vision_log_buffer)]

    return jsonify({
        'snapshot':         snap_payload,
        'heading_offset_rad': _vision_heading_offset_rad,
        'heading_offset_deg': (math.degrees(_vision_heading_offset_rad)
                               if _vision_heading_offset_rad is not None
                               else None),
        'rectify':          rect_state,
        'log':              log_tail,
    })


@app.route('/api/vision/pipelines', methods=['GET'])
def api_pipelines_list():
    if _pipeline_registry is None:
        return jsonify({'pipelines': [], 'available': False,
                        'node_kinds': [], 'active': None, 'default': None})
    return jsonify({
        'pipelines':   [_pipeline_to_payload(p) for p in _pipeline_registry.all()],
        'available':   True,
        'node_kinds':  _node_kinds_payload(),
        'active':      _pipeline_registry.active_name,
        'default':     _pipeline_registry.default_name,
    })


def _node_kinds_payload():
    """Return enough info for the editor to render the node palette + forms."""
    out = []
    for kind, cls in NODE_KINDS.items():
        io = getattr(cls, 'IO', None)
        out.append({
            'kind':    kind,
            'inputs':  [{'name': p.name, 'kind': p.kind,
                         'description': p.description,
                         'optional': getattr(p, 'optional', False)}
                        for p in (io.inputs if io else [])],
            'outputs': [{'name': p.name, 'kind': p.kind,
                         'description': p.description}
                        for p in (io.outputs if io else [])],
            'params_schema': getattr(cls, 'params_schema', {}),
        })
    return sorted(out, key=lambda x: x['kind'])


@app.route('/api/vision/pipelines/<name>', methods=['PUT'])
def api_pipeline_save(name):
    """Create or replace a pipeline graph. Saving does NOT activate it —
    use /active separately. This lets you edit a pipeline without
    disturbing whatever is currently producing dashboard feeds."""
    if _pipeline_registry is None:
        return jsonify({'ok': False, 'error': 'registry unavailable'}), 503
    data = request.get_json(force=True) or {}
    data['name'] = name
    try:
        p = _build_pipeline_from_dict(data)
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400
    _pipeline_registry.register(p)
    _wire_pipeline_feeds(p)
    # Inject the current global team into team-aware nodes so a freshly
    # added localization / filter node uses the right classification
    # without the user having to set it by hand.
    cur_team = sim_state.get('team', 'blue')
    with p._lock:
        for nid, rec in p._nodes.items():
            schema = getattr(rec.instance.__class__, 'params_schema', {}) or {}
            if 'team' in schema:
                try: rec.instance.set_params({'team': cur_team})
                except Exception: pass
    _save_pipelines()
    brain.log(f'[VISION] Pipeline saved: {name} ({len(data.get("nodes", []))} nodes)')
    return jsonify({'ok': True, 'pipeline': _pipeline_to_payload(p)})


@app.route('/api/vision/pipelines/<name>', methods=['DELETE'])
def api_pipeline_delete(name):
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    if not _pipeline_registry.remove(name):
        return jsonify({'ok': False, 'error': 'not found'}), 404
    _save_pipelines()
    return jsonify({'ok': True})


@app.route('/api/vision/active', methods=['POST'])
def api_pipeline_set_active():
    """Pick the pipeline that drives the dashboard feeds. Pass {name: null}
    to deactivate everything."""
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    data = request.get_json(force=True) or {}
    name = data.get('name')
    try:
        _pipeline_registry.set_active(name)
    except KeyError as e:
        return jsonify({'ok': False, 'error': str(e)}), 404
    _save_pipelines()
    brain.log(f'[VISION] Active pipeline → {name!r}')
    return jsonify({'ok': True, 'active': _pipeline_registry.active_name})


@app.route('/api/vision/pipelines/<name>/logs', methods=['GET'])
def api_pipeline_logs(name):
    """Return the pipeline's log ring buffer (recent INFO/ERROR lines).
    Used by the dashboard's running-pipeline status panel."""
    if _pipeline_registry is None:
        return jsonify({'ok': False, 'logs': []}), 503
    p = _pipeline_registry.get(name)
    if p is None:
        return jsonify({'ok': False, 'logs': []}), 404
    try:
        n = int(request.args.get('n', 50))
    except (TypeError, ValueError):
        n = 50
    return jsonify({'ok': True, 'logs': p.get_logs(n)})


@app.route('/api/vision/snapshot/<name>', methods=['GET'])
def api_pipeline_snapshot(name):
    """Capture one frame from the named pipeline's first source.* node and
    return it as a base64 JPEG. Used by the editor's freeze-frame debug
    view.

    Resolution order:
      1. _last_frame cached on the source instance (populated on start()
         and refreshed on every successful process()).
      2. rec.outputs['frame'] from the latest tick.
      3. Read one frame on-demand from the underlying VideoSource (without
         changing playback state).
    """
    if _pipeline_registry is None:
        return jsonify({'ok': False, 'error': 'registry unavailable'}), 503
    p = _pipeline_registry.get(name)
    if p is None:
        return jsonify({'ok': False, 'error': f'pipeline {name!r} not found'}), 404
    import cv2 as _cv2, base64 as _b64
    # Snapshot the source-kind records under the pipeline lock — the
    # worker thread can otherwise mutate _nodes mid-iteration.
    with p._lock:
        source_records = [
            (nid, rec) for nid, rec in p._nodes.items()
            if rec.kind.startswith('source.')
        ]
    for nid, rec in source_records:
        node = rec.instance
        # 1. cached frame from a previous read
        frame = getattr(node, '_last_frame', None)
        # 2. last output produced
        if frame is None:
            frame = rec.outputs.get('frame')
        # 3. last-resort: read on demand without flipping playback. We hold
        #    the pipeline lock so the worker thread doesn't race us.
        if frame is None:
            try:
                with p._lock:
                    s = getattr(node, '_source', None)
                    if s is not None and s.is_open:
                        f = s.read()
                        if f is not None:
                            node._last_frame = f
                            frame = f
                            try:
                                s.seek(0)
                            except Exception:
                                pass
            except Exception as e:
                return jsonify({'ok': False, 'error': f'read on demand: {e}'}), 500
        if frame is None:
            return jsonify({
                'ok': False,
                'error': 'no frame yet — open the source (load a video / connect a camera) first',
            }), 503
        try:
            ok, buf = _cv2.imencode('.jpg', frame, [_cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return jsonify({'ok': False, 'error': 'encode failed'}), 500
            return jsonify({
                'ok': True, 'source_node': nid,
                'jpeg': _b64.b64encode(bytes(buf)).decode(),
                'shape': list(frame.shape),
            })
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500
    return jsonify({'ok': False, 'error': 'no source node in pipeline'}), 404


@app.route('/api/vision/debug_run', methods=['POST'])
def api_pipeline_debug_run():
    """Run an arbitrary pipeline graph ONCE against a snapshot frame and
    return per-node frame outputs as base64 JPEGs.

    Body:
        graph: full pipeline dict (same shape as PUT /pipelines/<n>)
        snapshot_jpeg: base64 PNG/JPEG bytes — used as the source frame for
                       any source.* node (we substitute the source with a
                       canned frame).

    Response: { ok, frames: { node_id: {jpeg, shape} }, errors: { node_id: msg } }
    """
    if _pipeline_registry is None:
        return jsonify({'ok': False, 'error': 'registry unavailable'}), 503
    body = request.get_json(force=True) or {}
    graph = body.get('graph') or {}
    snapshot_b64 = body.get('snapshot_jpeg')
    if not snapshot_b64:
        return jsonify({'ok': False, 'error': 'snapshot_jpeg required'}), 400
    try:
        import cv2 as _cv2, numpy as _np, base64 as _b64
        buf = _b64.b64decode(snapshot_b64)
        snap = _cv2.imdecode(_np.frombuffer(buf, dtype=_np.uint8), _cv2.IMREAD_COLOR)
        if snap is None:
            return jsonify({'ok': False, 'error': 'snapshot decode failed'}), 400
    except Exception as e:
        return jsonify({'ok': False, 'error': f'snapshot decode: {e}'}), 400

    # Build a temporary pipeline (separate from the registry's named ones,
    # so we don't disturb anything that's running).
    try:
        tmp = _build_pipeline_from_dict(graph)
    except Exception as e:
        return jsonify({'ok': False, 'error': f'graph build: {e}'}), 400

    # Replace each source.* node's process() with a canned-frame returner.
    # We don't call .start() so no real cameras / files get opened.
    # The frame IS copied here (despite the per-tick perf cost) because
    # debug_run may be invoked while a preprocess node is still tweaked
    # by the user, and an in-place op would corrupt the snapshot for
    # subsequent runs.
    class _CannedSource:
        def __init__(self, frame): self._f = frame
        def process(self, inputs): return {'frame': self._f.copy()}
        def get_state(self): return {}
        def shutdown(self): pass
        # Mimic Node attach() so any helper code that touches it is happy.
        def attach(self, pipeline, node_id): pass
        def start(self): pass
        def get_params(self): return {}
        def set_params(self, p): pass

    # Walk the topological order once, ferrying outputs by hand.
    tmp._reorder()
    nodes = list(tmp._nodes.values())
    for rec in nodes:
        if rec.kind.startswith('source.'):
            rec.instance = _CannedSource(snap)
        else:
            try:
                rec.instance.start()
            except Exception:
                pass

    frames_out = {}
    errors_out = {}
    for nid in tmp._order:
        rec = tmp._nodes.get(nid)
        if rec is None:
            continue
        # Build inputs dict from upstream outputs
        inp = {}
        for port, (src_nid, src_port) in rec.inputs.items():
            src = tmp._nodes.get(src_nid)
            inp[port] = (src.outputs.get(src_port) if src else None)
        try:
            rec.outputs = rec.instance.process(inp) or {}
        except Exception as e:
            errors_out[nid] = f'{type(e).__name__}: {e}'
            rec.outputs = {}
            continue
        # Find any frame-typed outputs and JPEG-encode them
        try:
            import cv2 as _cv2
            for port_name, val in rec.outputs.items():
                if val is None:
                    continue
                # numpy ndarray with 2 or 3 dims → treat as a frame
                if hasattr(val, 'shape') and 2 <= len(val.shape) <= 3:
                    ok, jb = _cv2.imencode('.jpg', val,
                                           [_cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        frames_out.setdefault(nid, {})
                        frames_out[nid][port_name] = {
                            'jpeg':  _b64.b64encode(bytes(jb)).decode(),
                            'shape': list(val.shape),
                        }
            # "main" preference: preview > frame > first-other.
            # Inline node-card pulls main, so the user sees the annotated
            # version when the node exposes one.
            if nid in frames_out:
                bag = frames_out[nid]
                if 'preview' in bag:
                    bag['main'] = bag['preview']
                elif 'frame' in bag:
                    bag['main'] = bag['frame']
                else:
                    # Pick whatever frame-port slot we have
                    for k, v in bag.items():
                        if k != 'main':
                            bag['main'] = v
                            break
        except Exception as e:
            errors_out[nid] = f'encode: {e}'

    # Cleanup
    for rec in nodes:
        try:
            rec.instance.shutdown()
        except Exception:
            pass

    return jsonify({
        'ok': True,
        'frames': frames_out,
        'errors': errors_out,
    })


@app.route('/api/vision/pipelines/<name>/node/<node_id>/params', methods=['POST'])
def api_pipeline_node_params(name, node_id):
    """Update a single node's params live (e.g. flip playback live↔pause↔step
    without going through a full pipeline save). Body: { params: {key: val} }.
    """
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    p = _pipeline_registry.get(name)
    if p is None:
        return jsonify({'ok': False, 'error': 'pipeline not found'}), 404
    node = p.get_node(node_id)
    if node is None:
        return jsonify({'ok': False, 'error': 'node not found'}), 404
    body = request.get_json(force=True) or {}
    params = body.get('params', {})
    if not isinstance(params, dict):
        return jsonify({'ok': False, 'error': 'params must be a dict'}), 400
    try:
        node.set_params(params)
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400
    # If feed_id (or any param on an output node) changed, re-wire feeds
    # so the new feed_id has a SocketIO subscriber. Cheap — just iterates
    # the graph's output nodes once.
    if node.__class__.__name__ == 'OutputNode' or 'feed_id' in params:
        _wire_pipeline_feeds(p)
    return jsonify({'ok': True, 'state': node.get_state()})


@app.route('/api/vision/pipelines/<name>/enable', methods=['POST'])
def api_pipeline_enable(name):
    """Enable / disable a single pipeline WITHOUT touching the others.
    Lets multiple pipelines run in parallel (e.g. localization + game-element
    detection). Body: { enabled: true|false }. Returns 404 if not found."""
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    body = request.get_json(force=True) or {}
    want = bool(body.get('enabled', True))
    if not _pipeline_registry.set_enabled(name, want):
        return jsonify({'ok': False, 'error': 'pipeline not found'}), 404
    return jsonify({'ok': True, 'enabled': want, 'name': name})


@app.route('/api/vision/pipelines/<name>/source', methods=['POST'])
def api_pipeline_source(name):
    """Update the FIRST source.* node's params on a given pipeline.
    Body: { params: {playback, speed, seek_target, …} } (single shot, atomic).

    Convenience over the per-node endpoint — caller doesn't need to know
    the node id. Returns 404 if the pipeline has no source node."""
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    p = _pipeline_registry.get(name)
    if p is None:
        return jsonify({'ok': False, 'error': 'pipeline not found'}), 404
    body = request.get_json(force=True) or {}
    params = body.get('params', {})
    if not isinstance(params, dict) or not params:
        return jsonify({'ok': False, 'error': 'params dict required'}), 400
    # Find the first source.* node in the pipeline.
    src_id = None
    for nid in p.node_ids():
        rec = p._nodes.get(nid)
        if rec and rec.kind.startswith('source.'):
            src_id = nid
            break
    if src_id is None:
        return jsonify({'ok': False, 'error': 'no source.* node in pipeline'}), 404
    src = p.get_node(src_id)
    # Apply seek_target FIRST so a follow-up playback='seek' picks it up.
    try:
        if 'seek_target' in params:
            src.set_params({'seek_target': params['seek_target']})
        rest = {k: v for k, v in params.items() if k != 'seek_target'}
        if rest:
            src.set_params(rest)
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400
    return jsonify({'ok': True, 'source_node': src_id,
                    'state': src.get_state()})


@app.route('/api/vision/default', methods=['POST'])
def api_pipeline_set_default():
    """Mark a pipeline as default — auto-activated on server startup.
    Pass {name: null} to clear."""
    if _pipeline_registry is None:
        return jsonify({'ok': False}), 503
    data = request.get_json(force=True) or {}
    name = data.get('name')
    try:
        _pipeline_registry.set_default(name)
    except KeyError as e:
        return jsonify({'ok': False, 'error': str(e)}), 404
    _save_pipelines()
    brain.log(f'[VISION] Default pipeline → {name!r}')
    return jsonify({'ok': True, 'default': _pipeline_registry.default_name})


@app.route('/api/vision/team', methods=['POST'])
def api_vision_team():
    """Set the team color (drives which tag IDs are tracked as own/opp).
    Body: { team: 'blue'|'yellow' }"""
    if _vision is None:
        return jsonify({'ok': False}), 503
    data = request.get_json(force=True) or {}
    team = str(data.get('team', 'blue')).lower()
    _vision.set_team(team)
    return jsonify({'ok': True, 'team': _vision.team})


@app.route('/api/vision/sync_heading', methods=['POST'])
def api_vision_sync_heading():
    """Capture the heading offset between OTOS and the robot tag.
    Called once after the robot's recalage routine completes."""
    if _vision is None:
        return jsonify({'ok': False, 'error': 'vision backend unavailable'}), 503
    res = _vision.sync_heading()
    if res.get('ok'):
        brain.log(f"[VISION] Heading synced: offset = "
                  f"{res['offset_deg']:+.2f} deg "
                  f"(tag={math.degrees(res['tag_theta_rad']):+.1f}°, "
                  f"otos={math.degrees(res['otos_theta_rad']):+.1f}°)")
    else:
        brain.log(f"[VISION] Heading sync failed: {res.get('reason')}")
    return jsonify(res)


@app.route('/api/vision/reset_heading', methods=['POST'])
def api_vision_reset_heading():
    if _vision is None:
        return jsonify({'ok': False}), 503
    _vision.reset_heading_offset()
    brain.log('[VISION] Heading offset cleared')
    return jsonify({'ok': True})


# ── Vision frame push loop (25 fps max, SocketIO) ────────────────────────────

_vision_clients = 0   # count of clients with vision view open

def _vision_push_loop():
    """Push vision frames + tracker state to subscribed clients at up to 25 fps.
    JPEG encoding inside the backend is gated by the same client count, so
    when nobody is watching the visio tab the encoder stage is skipped."""
    while True:
        try:
            if _vision and _vision.enabled and _vision_clients > 0:
                raw_b64, rect_b64 = _vision.get_latest_frames_b64()
                state = _vision.get_state()
                if raw_b64:
                    socketio.emit('vision_frame', {
                        'raw':  raw_b64,
                        'rect': rect_b64,
                        **state.get('frame_info', {}),
                        'detections':    state.get('detections', []),
                        'anchor_status': state.get('anchor_status', {}),
                        'robot_pose':    state.get('robot_pose'),
                        'opponent_pose': state.get('opponent_pose'),
                        'heading_offset_deg': state.get('heading_offset_deg'),
                        'last_correction':    state.get('last_correction'),
                        'has_h':   state.get('has_homography', False),
                        'h_fresh': state.get('homography_fresh', False),
                        'team':    state.get('team', 'blue'),
                    })
        except Exception:
            pass
        time.sleep(1 / 25)


@socketio.on('vision_view_active')
def on_vision_view_active(data):
    global _vision_clients
    if _vision is None:
        return
    if data.get('active'):
        _vision_clients += 1
        _vision.add_stream_client()
    else:
        _vision_clients = max(0, _vision_clients - 1)
        _vision.remove_stream_client()


def _auto_sync_vision_heading():
    """Wait for the robot's recalage routine to deliver a stable OTOS pose
    AND for the OWN robot tag to be visible, then capture the heading
    offset. Retries for up to 15 s — if the tag never shows up, gives up
    and logs a warning (the user can still trigger a manual sync from the
    visio tab).

    The premise: recalage takes a few seconds. Once it's done OTOS reports
    a sane absolute pose. We sync as soon as both signals are available
    (vision OWN tag + OTOS pose), so the heading offset is locked in
    before any vision-driven correction can fire."""
    if _vision is None:
        return
    deadline = time.monotonic() + 15.0
    last_log_t = 0.0
    while time.monotonic() < deadline:
        if not _match_running:
            return
        res = _vision.sync_heading()
        if res.get('ok'):
            socketio.emit('vision_heading_synced', {
                'offset_deg': res['offset_deg'],
            })
            return
        # Throttle the noisy logs to once per 2 s
        now = time.monotonic()
        if now - last_log_t > 2.0:
            brain.log(f"[VISION] Heading sync waiting "
                      f"({res.get('reason', '?')})…")
            last_log_t = now
        time.sleep(0.5)
    brain.log('[VISION] Heading sync gave up after 15 s — '
              'no robot tag visible; correction disabled until manual sync')


# ── Vision-OTOS fusion: long-term drift correction ───────────────────────────
# When the match is running and the vision backend has a fresh corrected
# pose ready, push it to the firmware via setAbsPosition(...). The throttling
# (min distance, min period) is handled inside the backend itself.

def _vision_correction_loop():
    """Polls the vision backend for ready corrections and applies them via
    the active hardware transport. Idle when no match is running."""
    while True:
        time.sleep(0.1)   # 10 Hz polling — actual correction rate is gated
                          #                 inside the backend
        if _vision is None or not _vision.enabled:
            continue
        if not _match_running:
            continue
        t = _active_transport()
        if t is None or not t.is_connected:
            continue
        try:
            corr = _vision.pop_correction()
        except Exception as e:
            brain.log(f'[VISION] pop_correction error: {e}')
            continue
        if corr is None:
            continue
        x_mm, y_mm, theta_rad = corr
        theta_deg = math.degrees(theta_rad)
        cmd = f"setAbsPosition({x_mm:.0f},{y_mm:.0f},{theta_deg:.2f})"
        try:
            t.fire(cmd)
            brain.log(f'[VISION] corr → ({x_mm:.0f}, {y_mm:.0f}, '
                      f'{theta_deg:+.1f}°)')
            socketio.emit('vision_correction', {
                'x_mm': round(x_mm, 1),
                'y_mm': round(y_mm, 1),
                'theta_deg': round(theta_deg, 2),
            })
        except Exception as e:
            brain.log(f'[VISION] correction send failed: {e}')


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

_socketio_clients = 0           # live count, incremented on connect/decremented on disconnect
_socketio_clients_lock = threading.Lock()


def _socketio_clients_count() -> int:
    with _socketio_clients_lock:
        return _socketio_clients


# ── Network-stats widget ───────────────────────────────────────────────────
# Cumulative bytes + event count for SocketIO traffic. The browser pings
# 'net_ping' every 10 s; we echo back with the running totals so the
# client can derive RTT + kB/s + events/s deltas over the interval.
# Bytes are an approximation — the JPEG payload dominates so we only count
# that path precisely; small JSON events are tallied with a flat estimate.
_net_stats_bytes  = 0
_net_stats_events = 0
_net_stats_lock   = threading.Lock()


def _net_stats_add(n_bytes: int) -> None:
    global _net_stats_bytes, _net_stats_events
    with _net_stats_lock:
        _net_stats_bytes  += int(n_bytes)
        _net_stats_events += 1


@socketio.on('net_ping')
def on_net_ping(data):
    """Echo back: t_client_ms (for RTT), t_server_ms, and running totals
    for the SocketIO emit pipe. The client diffs successive totals to
    derive throughput + event rate."""
    with _net_stats_lock:
        b = _net_stats_bytes
        e = _net_stats_events
    emit('net_pong', {
        't_client_ms':    (data or {}).get('t_client_ms'),
        't_server_ms':    time.monotonic() * 1000.0,
        'bytes_emitted':  b,
        'events_emitted': e,
    })


@socketio.on('connect')
def on_connect():
    global _socketio_clients
    with _socketio_clients_lock:
        _socketio_clients += 1
        n = _socketio_clients
    emit('state', _build_state())
    brain.log(f"Web client connected ({n} total)")


@socketio.on('disconnect')
def on_disconnect():
    global _socketio_clients
    with _socketio_clients_lock:
        _socketio_clients = max(0, _socketio_clients - 1)
        n = _socketio_clients
    brain.log(f"Web client disconnected ({n} remaining)")


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
    """Set the team color (sim mode only; in HW mode the robot's
    physical switch is the source of truth and the topbar buttons are
    disabled — see cgSetConnectionMode in app.js)."""
    team = data['team']
    _apply_team(team, source='ui')


def _apply_team(team: str, source: str = 'ui',
                force: bool = False) -> None:
    """Single source of truth for team color. Called by:
        - on_set_team    (UI button in sim mode)
        - _hw_team_poll  (HW telemetry every 5 s)
        - boot path      (force=True so the pipelines get an initial
                          team push even when sim_state already happens
                          to equal the new value)

    Updates sim_state, the simulator robot, the legacy vision backend,
    and pushes the new team into every team-aware node in every saved
    pipeline (so the localization classification stays correct).

    Without force, an unchanged team short-circuits — that's how we
    avoid spamming the log + sim_state writes every 3 s when the team
    poller keeps reporting the same value. The boot path needs force=True
    because vision_pipelines_def.py picks its own TEAM default which
    can differ from sim_state's, and a no-op _apply_team would leave
    the pipeline classifying with the wrong team until the user toggles
    the topbar by hand.
    """
    if team not in ('blue', 'yellow'):
        return
    team_actually_changed = (sim_state.get('team') != team)
    if not force and not team_actually_changed:
        return   # already there — don't spam logs / save
    sim_state['team'] = team

    # Camera mirrors per team → calibration pairs captured under the
    # previous cam_xy are no longer valid for the parallax solver.
    # Drop them so the solver never mixes pre/post-mirror data. Only
    # fires when the team actually flipped (boot-time force=True with
    # the same team is a no-op).
    if team_actually_changed and len(_vision_calibration_pairs) > 0:
        n = len(_vision_calibration_pairs)
        _vision_calibration_pairs.clear()
        _vlog(f'team changed → cleared {n} stale calibration pair'
              f'{"s" if n > 1 else ""}')
    try:
        robot.reset_to_start(team)
    except Exception:
        pass
    brain.log(f'Team → {team} (source: {source})')
    # Legacy match-time backend
    if _vision is not None:
        try: _vision.set_team(team)
        except Exception: pass
    # Pipeline registry: push the team into every team-aware node so
    # downstream filter.classification etc. classify correctly without
    # the user having to hand-edit each node's `team` param.
    # Also mirror the camera X for parallax: the rig is the same
    # physical mount but the operator stands on the OWN side, so the
    # camera always views the team's own half — its X mirrors with team.
    vcfg = _load_vision_config()
    cam_xyz = vcfg.get(f'cam_{team}_xyz_mm') or [None, None, None]
    cam_x_for_team = cam_xyz[0] if len(cam_xyz) >= 1 else None
    cam_y_for_team = cam_xyz[1] if len(cam_xyz) >= 2 else None
    cam_z_for_team = cam_xyz[2] if len(cam_xyz) >= 3 else None
    own_z = vcfg.get('own_object_z_mm')
    opp_z = vcfg.get('opp_object_z_mm')
    if _pipeline_registry is not None:
        for p in _pipeline_registry.all():
            try:
                with p._lock:
                    for nid, rec in p._nodes.items():
                        schema = getattr(rec.instance.__class__, 'params_schema', {}) or {}
                        params_to_set = {}
                        if 'team' in schema:
                            params_to_set['team'] = team
                        if rec.kind in ('camera.manual', 'parallax'):
                            if cam_x_for_team is not None and 'cam_x_mm' in schema:
                                params_to_set['cam_x_mm'] = float(cam_x_for_team)
                            if cam_y_for_team is not None and 'cam_y_mm' in schema:
                                params_to_set['cam_y_mm'] = float(cam_y_for_team)
                            if cam_z_for_team is not None and 'cam_z_mm' in schema:
                                params_to_set['cam_z_mm'] = float(cam_z_for_team)
                        if rec.kind == 'parallax':
                            if own_z is not None and 'own_object_z_mm' in schema:
                                params_to_set['own_object_z_mm'] = float(own_z)
                            if opp_z is not None and 'opp_object_z_mm' in schema:
                                params_to_set['opp_object_z_mm'] = float(opp_z)
                        if params_to_set:
                            try: rec.instance.set_params(params_to_set)
                            except Exception: pass
            except Exception as e:
                brain.log(f'[VISION] team-sync failed for {p.name}: {e}')
    if cam_x_for_team is not None:
        _vlog(f'team={team} → cam=({cam_x_for_team:.0f},'
              f'{cam_y_for_team if cam_y_for_team is not None else "?"},'
              f'{cam_z_for_team if cam_z_for_team is not None else "?"}) '
              f'z_own={own_z} z_opp={opp_z}')

def _start_hw_team_poller():
    """In HW mode, the team color is set by a physical switch on the
    robot. We poll the firmware every ~5 s and mirror it locally so the
    UI + vision stay in sync.

    NOTE: requires firmware to expose the team via a telemetry channel
    or a `team_get` bridge command. Until that's wired this poller is a
    no-op stub — it logs once at startup so Jules sees the path is
    plumbed, then idles.
    """
    def _loop():
        first = True
        while True:
            time.sleep(3.0)
            try:
                if _connection_mode not in ('usb', 'xbee'):
                    continue   # sim / idle: topbar UI drives team
                # Match input freeze on the robot side: once the strategy is
                # running, the IHM color switch is locked and team_get would
                # just return the same value forever — no point hammering it.
                if globals().get('_match_running'):
                    continue
                t = _active_transport()
                if t is None or not t.is_connected:
                    continue
                # Try the bridge command. Firmware should respond with
                # 'blue' or 'yellow'. Until implemented, this errors out
                # gracefully and the poller stays dormant.
                ok, res = t.execute('team_get', timeout_ms=500)
                if not ok or not res:
                    if first:
                        brain.log('[VISION] team_get not implemented in firmware '
                                  '— falling back to UI-set team')
                        first = False
                    continue
                team = res.strip().lower()
                if team in ('blue', 'yellow'):
                    _apply_team(team, source='telemetry')
            except Exception as e:
                if first:
                    brain.log(f'[VISION] team poller: {e}')
                    first = False
    threading.Thread(target=_loop, daemon=True,
                     name='vision-team-poller').start()


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
        def _on_debug(line):
            socketio.emit('uart_raw', {'dir': 'dbg', 'line': line})
        t.subscribe_telemetry('_raw',    _on_raw_rx)
        t.subscribe_telemetry('_raw_tx', _on_raw_tx)
        t.subscribe_telemetry('_debug',  _on_debug)
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
                    # Forward to vision backend for sync_heading() and
                    # correction-distance gating.
                    if _vision is not None:
                        _vision.update_otos_pose(
                            robot.pos.x, robot.pos.y, robot.theta,
                        )
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

            # ── T:vis → vision recalage / pose request from Teensy ─────
            #
            # Three subcommands, all sent by the firmware as telemetry
            # frames `T:vis <subcommand> [k=v ...]`. Replies are framed
            # commands sent back to T41 (handled in JetsonBridge::handleRequest).
            #
            #   "homography_capture"
            #     Lock the active rectify node's homography on the next
            #     available frame so subsequent frames don't re-fit it.
            #     Sent at the start of recalage(), while the static anchor
            #     tags are still visible and before the robot moves into
            #     view (which would partially occlude the anchors).
            #     Reply: vis_h_locked(ok=1) | vis_h_locked(ok=0,reason=...)
            #
            #   "cal_request x=<mm> y=<mm> t=<rad>"
            #     Robot is at known (x, y, t) world pose. Pick the closest
            #     ArUco, lock it as OWN, infer team, and capture the
            #     heading offset between the robot frame and the (random)
            #     tag-mount orientation.
            #     Reply: vis_cal_done(own=..,team=..,x=..,y=..,t=..)
            #          | vis_cal_failed(reason=..)
            #
            #   "pose_request"
            #     Send back the latest own-team vision pose with the
            #     heading offset applied (so theta is in the robot frame).
            #     Reply: vis_pose(x=..,y=..,t=..,valid=0|1)
            #
            # Replies are sent as commands TO T41. Don't block in the
            # subscribe callback — kick off daemon threads so the
            # telemetry pipe stays unblocked.
            def _on_vis(data_str):
                line = (data_str or '').strip()
                def _kv(s, key, cast=float, default=None):
                    p = s.find(key + '=')
                    if p < 0: return default
                    p += len(key) + 1
                    e = p
                    while e < len(s) and s[e] not in (' ', ',', ')'):
                        e += 1
                    try: return cast(s[p:e])
                    except (ValueError, TypeError): return default
                def _send(reply_cmd):
                    try:
                        t.execute(reply_cmd, timeout_ms=2000)
                    except Exception as e:
                        print(f"[T:vis] reply failed: {e}")
                if line.startswith('homography_capture'):
                    _vlog('rx ← T:vis homography_capture')
                    def _do_lock():
                        global _vision_heading_offset_rad, _vision_calibration_snapshot
                        rect_node, _ = _get_rectify_node()
                        if rect_node is None:
                            _vlog('homography lock failed: no rectify node', 'err')
                            _send('vis_h_locked(ok=0,reason=no_rectify_node)')
                            return
                        # Idempotent reset of every derived vision state from
                        # any previous lock — recalage is the single entry,
                        # whatever was cached before is stale by definition.
                        _vision_heading_offset_rad = None
                        _vision_calibration_snapshot = None
                        loc_inst, _ = _get_localization_node()
                        if loc_inst is not None:
                            try:
                                loc_inst.set_params({'track_ids': list(range(1, 11))})
                            except Exception as e:
                                _vlog(f'track_ids reset failed: {e}', 'warn')
                        # Arm the node — its worker thread will fit H on
                        # the next available frame and lock + snapshot
                        # the BEV. request_capture() is idempotent (unlock+arm).
                        rect_node.request_capture()
                        deadline = time.monotonic() + 2.0
                        result = None
                        while time.monotonic() < deadline:
                            result = rect_node.get_capture_result()
                            if result is not None:
                                break
                            time.sleep(0.05)
                        if result and result.get('ok'):
                            # New H → previous heading offset is tied to
                            # the previous H, drop it so we can't apply
                            # a stale one.
                            _vision_heading_offset_rad = None
                            _vlog(
                                f"homography locked (mode={result.get('mode','?')}, "
                                f"anchors={result.get('detected_anchor_ids', [])})")
                            _send('vis_h_locked(ok=1)')
                        else:
                            # Timed out without a fit — cancel the pending
                            # request so we go back to IDLE instead of
                            # spinning forever on every subsequent tick.
                            rect_node.release_capture()
                            reason = (result or {}).get('reason', 'timeout_no_anchors')
                            _vlog(f'homography lock failed: {reason}', 'err')
                            _send(f'vis_h_locked(ok=0,reason={reason})')
                    threading.Thread(target=_do_lock, daemon=True,
                                     name='vis-hlock').start()
                elif line.startswith('homography_release'):
                    _vlog('rx ← T:vis homography_release')
                    def _do_unlock():
                        global _vision_heading_offset_rad, _vision_calibration_snapshot
                        rect_node, _ = _get_rectify_node()
                        if rect_node is not None:
                            rect_node.release_capture()
                        _vision_heading_offset_rad = None
                        _vision_calibration_snapshot = None
                        # Unlock track_ids so the next cal_request can pick
                        # any tag in the OWN range, not just the last one.
                        loc_inst, _ = _get_localization_node()
                        if loc_inst is not None:
                            try:
                                loc_inst.set_params({'track_ids': list(range(1, 11))})
                            except Exception as e:
                                _vlog(f'track_ids reset failed: {e}', 'warn')
                        _vlog('homography released')
                        _send('vis_h_locked(ok=0,reason=released)')
                    threading.Thread(target=_do_unlock, daemon=True,
                                     name='vis-hunlock').start()
                elif line.startswith('cal_request'):
                    # Firmware sends INTEGERS only — newlib-nano on the
                    # Teensy 4.1 silently breaks snprintf %f, so the
                    # protocol agrees on mm (x, y) and milliradians (t)
                    # as scaled integers. Same convention as T:a.
                    kx = _kv(line, 'x', float, 0.0)
                    ky = _kv(line, 'y', float, 0.0)
                    kt_mrad = _kv(line, 't', float, None)
                    kt = (kt_mrad / 1000.0) if kt_mrad is not None else None
                    kt_deg = (math.degrees(kt) if kt is not None else None)
                    _vlog(f'rx ← T:vis cal_request x={kx:.0f} y={ky:.0f} '
                          f't={kt_deg:+.1f}°' if kt is not None else
                          f'rx ← T:vis cal_request x={kx:.0f} y={ky:.0f} t=?')
                    def _do_cal():
                        # Retry on failure: tag detection sometimes
                        # misses a single pipeline tick (occlusion,
                        # remaining motion blur the firmware-side 3 s
                        # settle didn't kill, glare). 500 ms wait
                        # between attempts covers two pipeline ticks
                        # at FPS_LIMIT=4 so each retry definitely
                        # reads a fresh frame, not a re-pick of the
                        # same stale pose_list.
                        max_attempts = 3
                        retry_delay_s = 0.5
                        result = None
                        for attempt in range(1, max_attempts + 1):
                            if attempt > 1:
                                _vlog(f'cal_request: retry '
                                      f'{attempt}/{max_attempts} after '
                                      f'{int(retry_delay_s*1000)}ms wait',
                                      'warn')
                                time.sleep(retry_delay_s)
                            result = _recalage_pick_own_tag(
                                kx, ky, known_theta_rad=kt)
                            if result is not None:
                                if attempt > 1:
                                    _vlog(f'cal_request: OK on attempt '
                                          f'{attempt}/{max_attempts}')
                                break
                        if result is None:
                            _vlog(f'cal_request: GAVE UP after '
                                  f'{max_attempts} attempts — sending '
                                  f'vis_cal_failed', 'err')
                            _send('vis_cal_failed(reason=no_candidate)')
                            return
                        vp = result['vision_pose']
                        _send(
                            f"vis_cal_done(own={result['tag_id']},"
                            f"team={result['team']},"
                            f"x={vp['x_mm']:.1f},y={vp['y_mm']:.1f},"
                            f"t={vp['theta_rad']:.3f})"
                        )
                    threading.Thread(target=_do_cal, daemon=True,
                                     name='vis-cal').start()
                elif line.startswith('pose_request'):
                    # Don't log every single pose_request — they fire
                    # every time the strategy queries vision (potentially
                    # 5-10 Hz during match), would drown the buffer.
                    # We DO surface a rate-limited warning to the
                    # vision-debug log when the pipeline keeps returning
                    # no own pose (firmware retries on valid=0, so a real
                    # tag occlusion would otherwise be silent).
                    def _do_pose():
                        global _pose_request_invalid_streak
                        global _pose_request_invalid_last_warn_mono
                        pose = _get_latest_own_pose()
                        if pose is None:
                            _pose_request_invalid_streak += 1
                            now_mono = time.monotonic()
                            if (now_mono -
                                    _pose_request_invalid_last_warn_mono
                                    >= 1.0):
                                _pose_request_invalid_last_warn_mono = now_mono
                                _vlog(
                                    f'pose_request → valid=0 '
                                    f'(no own pose in last pipeline tick, '
                                    f'streak={_pose_request_invalid_streak})',
                                    'warn')
                            _send('vis_pose(valid=0)')
                            return
                        # valid=1 — log every successful syncToVision answer
                        # so the operator can see the vision→OTOS hand-off
                        # firing in real time. Includes a recovery suffix
                        # when this reply ended a valid=0 streak.
                        theta_rad = pose.get('theta_rad') or 0.0
                        recovery = (f' (recovered after '
                                    f'{_pose_request_invalid_streak} miss(es))'
                                    if _pose_request_invalid_streak > 0
                                    else '')
                        _vlog(
                            f"pose_request → valid=1 "
                            f"x={pose['x_mm']:.1f} y={pose['y_mm']:.1f} "
                            f"θ={math.degrees(theta_rad):+.1f}° "
                            f"src={pose.get('source','?')} "
                            f"tag=#{pose.get('tag_id','?')}{recovery}")
                        _pose_request_invalid_streak = 0
                        _send(
                            f"vis_pose(x={pose['x_mm']:.1f},"
                            f"y={pose['y_mm']:.1f},"
                            f"t={theta_rad:.3f},valid=1)"
                        )
                    threading.Thread(target=_do_pose, daemon=True,
                                     name='vis-pose').start()
                elif line.startswith('embed_detect'):
                    # Robot-mounted ESP32-CAM detection.  Same code path
                    # as the UI button (_run_embed_detect) → identical
                    # caching + feeds.
                    team_arg = _kv(line, 'team', str, None)
                    if isinstance(team_arg, str):
                        team_arg = team_arg.strip().lower()
                    _vlog(f'rx ← T:vis embed_detect team={team_arg}')
                    def _do_embed(_team=team_arg):
                        result = _run_embed_detect(source='firmware',
                                                   team_override=_team)
                        n      = int(result.get('n', 0))
                        off_mm = float(result.get('offset_mm', 0.0))
                        bias   = int(result.get('bias', 0))
                        valid  = 1 if result.get('valid', False) else 0
                        _send(
                            f"embed_detect_reply(n={n},"
                            f"offset={off_mm:.1f},bias={bias},"
                            f"valid={valid})"
                        )
                    threading.Thread(target=_do_embed, daemon=True,
                                     name='vis-embed-detect').start()
                else:
                    _vlog(f'rx ← T:vis unknown subcommand: {line!r}', 'warn')
            t.subscribe_telemetry('vis', _on_vis)

            # Diagnostic raw-line sniffer. The transport's `_raw` channel
            # fires for EVERY incoming line BEFORE parse_frame validates
            # the CRC, so we catch both:
            #   (1) properly-framed `T:vis cal_request …|crc` frames
            #       even when CRC mismatch would silently drop them at
            #       parse_frame, and
            #   (2) unframed firmware Console::info log lines (no `|crc`
            #       suffix → parse_frame drops them) — these prove the
            #       firmware function was at least entered.
            # Filtered to a few keywords to avoid drowning the buffer in
            # generic motion/safety telemetry.
            _RAW_KEYWORDS = ('cal_request', 'Vision recalage',
                             'Homography capture', '(Localisation)',
                             'T:vis ')
            def _on_raw_vis_sniff(line):
                s = line.strip()
                if not s:
                    return
                if not any(k in s for k in _RAW_KEYWORDS):
                    return
                # Skip echoes of our OWN reply commands going back to T41
                # (e.g. "42:vis_cal_done(...)|crc"). Those start with
                # a digit:colon and don't carry diagnostic value.
                if s and s[0].isdigit() and ':' in s.split('|', 1)[0]:
                    return
                if len(s) > 120:
                    s = s[:120] + '…'
                _vlog(f'raw RX: {s!r}', 'warn')
            t.subscribe_telemetry('_raw', _on_raw_vis_sniff)

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

@socketio.on('recalage')
def on_recalage():
    """Fire the firmware `recalage()` routine (= long-press on the robot's
    physical reset button). Hardware-only — in sim/idle mode there's no
    motion stack to drive. The handler is fire-and-monitor: we kick the
    command off in a daemon thread (the routine takes ~10 s of motion)
    and report progress / completion back via `recalage_state` events.
    """
    t = _active_transport()
    if t is None or not t.is_connected:
        _vlog('recalage: no HW transport connected', 'err')
        socketio.emit('recalage_state',
                      {'running': False, 'ok': False, 'error': 'not_connected'})
        return

    def _do():
        _vlog('recalage: command sent to firmware (HW)')
        socketio.emit('recalage_state', {'running': True})
        # 60 s ceiling: the routine probes a wall + moves to start pos +
        # waits for the vision recalage handshake (max ~700 ms). 60 s is
        # comfortably above the worst observed wallclock (~12 s) and
        # well below any human-noticeable hang.
        ok, res = t.execute('recalage()', timeout_ms=60000)
        if ok:
            _vlog(f'recalage: firmware reply OK ({res})')
        else:
            _vlog(f'recalage: firmware reply FAIL ({res})', 'err')
        socketio.emit('recalage_state',
                      {'running': False, 'ok': bool(ok), 'res': res})

    threading.Thread(target=_do, daemon=True, name='recalage-cmd').start()


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

    # Schedule auto heading-sync: the robot will execute its recalage
    # routine first; we retry every 500 ms until the OWN tag is seen and
    # an OTOS pose is available, capping at 15 s.
    if _vision is not None:
        _vision.reset_heading_offset()
        threading.Thread(target=_auto_sync_vision_heading,
                         daemon=True, name='vision-sync-heading').start()

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
    # Propagate the initial team to the vision tracker
    if _vision is not None:
        _vision.set_team(sim_state.get('team', 'blue'))
    # Start the in-process FrameSource BEFORE pipelines try to read frames.
    # One reader thread captures from V4L2 / file at hardware rate and
    # overwrites a 1-slot BGR buffer. Pipeline source nodes + recorder
    # snapshot that slot at their own rates — no queue, no MJPEG-over-HTTP.
    # Config lives in software/data/vision_config.json.
    try:
        import vision_source as _vs_module
        _vs_module.start(_load_vision_config())
        try:
            brain.log("[vision-source] in-process FrameSource started")
        except Exception:
            pass
    except Exception as _vc_err:
        print(f"[holOS] vision-source start failed: {_vc_err}")
        try:
            brain.log(f"[vision-source] start failed: {_vc_err}")
        except Exception:
            pass
    # Start the persistent ESP32-CAM MJPEG grabber. Background thread
    # tails the stream into a 1-slot frame cache so detect_once reads
    # the freshest frame instantly (no TCP handshake, no ESP capture
    # latency). Failure here is non-fatal — falls back to per-request
    # /capture fetch on each detect.
    if _embed_cam is not None:
        try:
            _embed_cam.start_streamer()
            print(f"[EmbedCam] streamer started "
                  f"({_embed_cam.streamer_status().get('url') or '(disabled)'})")
        except Exception as _es_err:
            print(f"[EmbedCam] streamer start failed: {_es_err}")
    # Load saved pipelines + activate the default one (if any). Feed
    # callbacks are wired inside _load_pipelines.
    # NB: vision pose updates are PULL-based now — the strategy queries
    # /api/vision/robot_pose on demand. We no longer spawn a driver
    # thread that auto-mutates robot.pos / robot.theta from vision.
    if _pipeline_registry is not None:
        _load_pipelines()
        # On boot, force-push the current team into the freshly built
        # pipelines. force=True is required: sim_state['team'] defaults
        # to 'yellow' but vision_pipelines_def.py builds the pipeline
        # with TEAM='blue', so the values often don't differ from the
        # bare-equality check's POV — yet the pipeline still needs to
        # be told. Without force, the localization node would keep
        # classifying with its build-time team until the user toggles
        # the topbar by hand.
        _apply_team(sim_state.get('team', 'blue'),
                    source='boot', force=True)
        # Boot banner — read from the vision log on /vision_debug to
        # confirm the new auto-tune code path is live (operators have
        # been bitten by a stale Python module surviving an "edit and
        # re-flash firmware" loop without restarting holOS itself).
        _vlog('boot: parallax auto-tune + per-team persistence active '
              '(vision_pipelines_def.py + parallax.py loaded)')
    # Start the HW team-color poller (no-op in sim/idle).
    _start_hw_team_poller()
    socketio.start_background_task(_physics_loop)
    socketio.start_background_task(_vision_push_loop)
    socketio.start_background_task(_vision_correction_loop)

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
