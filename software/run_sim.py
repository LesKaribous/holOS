"""
run_sim.py — Simulator entry point (Windows / Linux dev mode).

Starts the full simulator with:
  - VirtualTransport (no XBee needed)
  - SimBridge (fake Teensy running physics)
  - Brain (Jetson logic)
  - Flask+SocketIO web UI

Open http://localhost:5000 in a browser to interact.

Usage:
    cd py/
    python run_sim.py
    python run_sim.py --port 8080
"""

import sys
import os
import time
import threading
import traceback
import importlib.util
import argparse

# Sequence runner state
_seq_stop   = threading.Event()
_seq_thread = None

# ── Path setup ────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
STRATEGY_DIR = os.path.join(_HERE, 'strategy')
MACROS_FILE  = os.path.join(STRATEGY_DIR, 'macros.json')
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from flask import Flask, render_template, jsonify, request
import json
import glob
from flask_socketio import SocketIO, emit

from shared.config import (
    FIELD_W, FIELD_H, GRID_W, GRID_H, GRID_CELL,
    Vec2, Team, ObjectColor, COLOR_HEX, COLOR_BY_NAME, ROBOT_RADIUS, POI,
)
from sim.physics   import RobotPhysics
from sim.world     import OccupancyGrid, Pathfinder, GameObjects
from sim.bridge    import SimBridge
from sim.tests     import TestRunner, SUITES, ALL_TESTS
from transport.virtual import VirtualTransport
from brain import Brain

# ── Flask / SocketIO ──────────────────────────────────────────────────────────

_STATIC = os.path.join(_HERE, 'sim', 'static')
app = Flask(__name__, static_folder=_STATIC, template_folder=_STATIC)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0   # no static cache in dev
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ── Simulation objects ────────────────────────────────────────────────────────

robot     = RobotPhysics()
occupancy = OccupancyGrid()
pathfinder= Pathfinder(occupancy)
game_objs = GameObjects()

bridge    = SimBridge(robot, occupancy, pathfinder, game_objs)
transport = VirtualTransport()

# Wire transport ↔ bridge
transport.attach_bridge(bridge)
bridge.attach_transport(transport)

# Brain (strategy + services)
brain = Brain(transport)

# Test runner
test_runner = TestRunner(transport, bridge, robot, occupancy)

# ── Hardware transport (real XBee, optional) ──────────────────────────────────
# When connected, terminal/actuator/strategy commands are routed here instead
# of the VirtualTransport.  The physics simulation continues running for map
# visualisation, but robot state is driven by real telemetry.

_hw_transport  = None   # XBeeTransport instance when connected
_hw_brain      = None   # Brain wired to real hardware
_hw_connecting = False  # True while a connection attempt is in progress


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
}


# ── Physics loop (60 Hz) ──────────────────────────────────────────────────────

_PHYSICS_DT = 1.0 / 60.0

def _physics_loop():
    while True:
        t0 = time.perf_counter()
        bridge.tick(_PHYSICS_DT)
        socketio.emit('state', _build_state())
        elapsed = time.perf_counter() - t0
        sleep   = _PHYSICS_DT - elapsed
        if sleep > 0:
            time.sleep(sleep)


# ── State serialization ───────────────────────────────────────────────────────

def _build_state() -> dict:
    path_pts = [[p.x, p.y] for p in bridge.current_path()]
    return {
        'robot':     robot.to_dict(),
        'path':      path_pts,
        'occupancy': occupancy.to_list(),
        'dyn_obs':   [[o.x, o.y] for o in occupancy.dynamic_obstacles],
        'game_objs': game_objs.to_list(),
        'motion':    {
            'state':    bridge.motion_state(),
            'feedrate': brain.motion.get_feedrate(),
        },
        'safety': {
            'enabled':  sim_state['features']['safety'],
            'detected': bridge.safety_detected(),
        },
        'chrono': {
            'elapsed': bridge.chrono_elapsed(),
            'left':    max(0, 100.0 - bridge.chrono_elapsed()),
            'running': bridge.chrono_running(),
        },
        'score':    sim_state['score'],
        'team':     sim_state['team'],
        'features': sim_state['features'],
        'mode':     sim_state['mode'],
        'log':      _active_brain().get_log(30),
        'hw_mode':       (_hw_transport is not None and _hw_transport.is_connected),
        'hw_connecting': _hw_connecting,
        'hw_type':       ('xbee' if (_hw_transport is not None and _hw_transport.is_connected
                                     and getattr(_hw_transport, '_baudrate', 0) == 31250)
                          else 'usb' if (_hw_transport is not None and _hw_transport.is_connected)
                          else 'sim'),
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
    elif mode == 'obstacle':
        occupancy.add_dynamic_obstacle(Vec2(x, y))
        brain.log(f"Obstacle @ ({x:.0f}, {y:.0f})")
    elif mode == 'remove_obs':
        occupancy.remove_nearest_dynamic(Vec2(x, y))


@socketio.on('set_color')
def on_set_color(data):
    poi_name   = data['name']
    color_name = data['color']
    color      = COLOR_BY_NAME.get(color_name, ObjectColor.UNKNOWN)
    brain.vision.set_color(poi_name, color)
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
    occupancy.reset()
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


# ── XBee / serial connection ─────────────────────────────────────────────────

@socketio.on('serial_connect')
def on_serial_connect(data):
    global _hw_transport, _hw_brain, _hw_connecting
    port = (data.get('port') or '').strip()
    baud = int(data.get('baud') or 31250)

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

    def _do_connect():
        global _hw_transport, _hw_brain, _hw_connecting
        t = None
        try:
            from transport.xbee import XBeeTransport
            t  = XBeeTransport(port=port, baudrate=baud)
            ok = t.connect()
            if ok:
                _hw_transport = t
                _hw_brain     = Brain(t)
                _hw_brain.load_strategy()
                # Mirror real telemetry → update sim robot pos for map view
                def _on_pos(data_str):
                    try:
                        parts = dict(kv.split('=') for kv in data_str.split(','))
                        robot.pos   = Vec2(float(parts['x']), float(parts['y']))
                        robot.theta = float(parts['theta'])
                    except Exception:
                        pass
                t.subscribe_telemetry('pos', _on_pos)
                socketio.emit('serial_status', {
                    'ok': True, 'msg': f'Connected {port} @ {baud}bps'
                })
                brain.log(f'[XBee] Connected {port} @ {baud}bps')
            else:
                # t.connect() already closed the port — just report failure
                _hw_transport = None
                socketio.emit('serial_status', {
                    'ok': False,
                    'msg': f'No response from Teensy on {port} (check baud + wiring)',
                })
        except Exception as e:
            # Ensure the port is released even if an unexpected exception occurred
            if t is not None:
                try:
                    t.disconnect()
                except Exception:
                    pass
            _hw_transport = None
            socketio.emit('serial_status', {'ok': False, 'msg': str(e)})
        finally:
            _hw_connecting = False

    threading.Thread(target=_do_connect, daemon=True, name='serial-connect').start()


@socketio.on('serial_disconnect')
def on_serial_disconnect():
    global _hw_transport, _hw_brain
    if _hw_transport is not None:
        try:
            _hw_transport.disconnect()
        except Exception:
            pass
        _hw_transport = None
        _hw_brain     = None
        brain.log('[XBee] Disconnected')
    emit('serial_status', {'ok': False, 'msg': 'Disconnected'})


# ── Terminal & actuator events ────────────────────────────────────────────────

@socketio.on('terminal_cmd')
def on_terminal_cmd(data):
    """Execute a command, wait for reply.  Routes to XBee if connected."""
    cmd = data.get('cmd', '').strip()
    if not cmd:
        return
    t = _active_transport()
    mode = '[HW]' if t is not transport else '[SIM]'
    try:
        ok, res = t.execute(cmd, timeout_ms=int(data.get('timeout_ms', 5000)))
        emit('terminal_rx', {'cmd': cmd, 'ok': ok, 'res': res, 'mode': mode})
    except Exception as e:
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
    """Return the full test catalog (suites + tests)."""
    return jsonify({
        sid: {
            'label': s['label'],
            'icon':  s['icon'],
            'tests': s['tests'],
        }
        for sid, s in SUITES.items()
    })


@socketio.on('run_tests')
def on_run_tests(data):
    """
    data = {
      'suite': 'motors'   → run a full suite
      'test':  'mot_hb'   → run a single test
      (neither)           → run all tests
    }
    """
    if test_runner.is_running():
        emit('test_error', {'msg': 'Tests already running'})
        return

    def _progress(test_id: str, status: str):
        socketio.emit('test_progress', {'id': test_id, 'status': status})

    def _result(r):
        socketio.emit('test_result', r.to_dict())

    def _done():
        socketio.emit('test_done', {})

    suite = data.get('suite')
    test  = data.get('test')

    if test:
        test_runner.run_one(test, _progress, _result, _done)
    elif suite:
        test_runner.run_suite(suite, _progress, _result, _done)
    else:
        test_runner.run_all(_progress, _result, _done)


@socketio.on('stop_tests')
def on_stop_tests():
    test_runner.stop()


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


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='holOS Simulator')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()

    brain.load_strategy()
    brain.start_hot_reload(on_reload=lambda: socketio.emit(
        'reload', {'msg': 'strategy/match.py reloaded ✓'}
    ))
    socketio.start_background_task(_physics_loop)

    print("\n  holOS Simulator")
    print("  ┌────────────────────────────────┐")
    print(f"  │  http://localhost:{args.port:<14}│")
    print("  └────────────────────────────────┘\n")

    socketio.run(app, host=args.host, port=args.port,
                 debug=False, use_reloader=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
