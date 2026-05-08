"""Auto-start the virtual-camera server alongside holOS.

Reads `software/data/vision_camera_config.json` (seeded with sane
defaults on first run). When `auto_start` is true and port 5174 isn't
already serving, spawn `python -m vision_camera.camera ...` as a daemon
subprocess and tear it down on holOS exit. If something is already
serving on that port (e.g. user launched vision_camera.bat manually),
reuse it instead of double-launching.

Child stdout/stderr is teed to `software/data/vision_camera.log` so a
crash leaves a forensic trail. A monitor thread watches the process
and surfaces exits via the on_exit callback (used by run.py to push
the failure into the holOS log).
"""

from __future__ import annotations

import atexit
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Callable, Optional

_HERE = Path(__file__).resolve().parent
_SOFTWARE = _HERE.parent
_CONFIG = _SOFTWARE / 'data' / 'vision_camera_config.json'
_LOG    = _SOFTWARE / 'data' / 'vision_camera.log'

DEFAULT_CONFIG: dict = {
    'auto_start':  True,
    'source_kind': 'video',         # video | camera | image | jetson
    'source_path': 'vision/homography/data/video.2.mp4',
    'host':        '0.0.0.0',     # bind addr — 0.0.0.0 lets LAN clients reach the stream
    'port':        5174,
    'fps':         30,
    'quality':     80,
    # Jetson-only knobs (ignored for other source kinds)
    'gst_width':   1280,
    'gst_height':  720,
    'gst_fps':     30,
}

_proc: Optional[subprocess.Popen] = None
_state: dict = {
    'state':         'idle',       # idle | starting | running | external | exited | failed | disabled
    'pid':           None,
    'host':          None,
    'port':          None,
    'source_kind':   None,
    'source_path':   None,
    'started_at':    None,
    'exit_code':     None,
    'last_error':    None,         # one-line summary for UI
    'log_path':      str(_LOG),
}
_state_lock = threading.Lock()
_on_exit_cb: Optional[Callable[[str], None]] = None


def _log_holos(msg: str) -> None:
    """Forward a line to the holOS log via the registered callback."""
    cb = _on_exit_cb
    if cb is None:
        return
    try:
        cb(msg)
    except Exception:
        pass


def _set_state(**kw) -> None:
    with _state_lock:
        _state.update(kw)


def _load_config() -> dict:
    if _CONFIG.is_file():
        try:
            cfg = json.loads(_CONFIG.read_text())
            return {**DEFAULT_CONFIG, **cfg}
        except Exception as e:
            print(f"[vision-camera] config read failed ({e}) — using defaults")
            return dict(DEFAULT_CONFIG)
    try:
        _CONFIG.parent.mkdir(parents=True, exist_ok=True)
        _CONFIG.write_text(json.dumps(DEFAULT_CONFIG, indent=2) + '\n')
        print(f"[vision-camera] seeded default config → {_CONFIG}")
    except Exception as e:
        print(f"[vision-camera] could not seed config: {e}")
    return dict(DEFAULT_CONFIG)


def _resolve_source(kind: str, path: str) -> str:
    if kind == 'camera':
        return path
    if path.startswith(('/dev/', 'http://', 'https://', 'rtsp://')):
        return path
    p = Path(path)
    if not p.is_absolute():
        p = _SOFTWARE / path
    return str(p)


def _is_port_serving(host: str, port: int, timeout: float = 0.4) -> bool:
    # The bind address can be 0.0.0.0 (LAN) — that's not a valid client URL
    # on every OS, so always probe via loopback.
    probe = '127.0.0.1' if host in ('0.0.0.0', '::', '') else host
    try:
        with urllib.request.urlopen(
                f'http://{probe}:{port}/api/status', timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


def _tail_log(n_lines: int = 20) -> str:
    """Best-effort tail of the child log."""
    try:
        with open(_LOG, 'rb') as f:
            try:
                f.seek(-8192, os.SEEK_END)
            except OSError:
                f.seek(0)
            data = f.read().decode('utf-8', errors='replace')
        lines = [ln for ln in data.splitlines() if ln.strip()]
        return '\n'.join(lines[-n_lines:])
    except FileNotFoundError:
        return ''
    except Exception as e:
        return f'(tail failed: {e})'


def _summary_from_tail(tail: str) -> str:
    """Pick the most informative line out of the log tail for a 1-line UI summary."""
    if not tail:
        return 'no log output'
    for ln in reversed(tail.splitlines()):
        s = ln.strip()
        low = s.lower()
        if any(k in low for k in ('error', 'fail', 'traceback', 'cannot', 'could not')):
            return s[:180]
    last = tail.splitlines()[-1].strip()
    return last[:180] if last else 'unknown'


def _monitor() -> None:
    """Watch the child process. When it exits, capture the log tail and
    surface it through the holOS log + status state."""
    p = _proc
    if p is None:
        return
    rc = p.wait()
    tail = _tail_log()
    summary = _summary_from_tail(tail)
    # Don't yell if WE asked it to stop.
    benign = rc in (0, -15, 143)
    _set_state(state='exited' if benign else 'failed',
               exit_code=rc, last_error=summary if not benign else None)
    if benign:
        _log_holos(f"[vision-camera] subprocess exited (code={rc})")
    else:
        _log_holos(f"[vision-camera] CRASH (code={rc}): {summary}")


def start(on_exit: Optional[Callable[[str], None]] = None) -> dict:
    """Spawn the virtual-camera subprocess if config allows. Idempotent.

    on_exit: optional callback (str) -> None used to forward lifecycle
    events into the holOS log. Called from a worker thread.
    """
    global _proc, _on_exit_cb
    if on_exit is not None:
        _on_exit_cb = on_exit

    cfg = _load_config()
    _set_state(host=cfg['host'], port=int(cfg['port']),
               source_kind=cfg['source_kind'], source_path=cfg['source_path'])

    if not cfg.get('auto_start', True):
        print("[vision-camera] auto_start disabled in config — skipping")
        _set_state(state='disabled', last_error=None)
        _log_holos("[vision-camera] auto_start disabled in config")
        return {'started': False, 'reason': 'auto_start=false', 'config': cfg}

    if _proc is not None and _proc.poll() is None:
        return {'started': False, 'reason': 'already-spawned', 'config': cfg}

    host, port = cfg['host'], int(cfg['port'])
    if _is_port_serving(host, port):
        msg = f"http://{host}:{port}/ already responding — reusing existing instance"
        print(f"[vision-camera] {msg}")
        _set_state(state='external', pid=None, started_at=time.time(),
                   last_error=None)
        _log_holos(f"[vision-camera] {msg}")
        return {'started': False, 'reason': 'external-instance', 'config': cfg}

    src_path = _resolve_source(cfg['source_kind'], cfg['source_path'])
    cmd = [
        sys.executable, '-u', '-m', 'vision_camera.camera',
        cfg['source_kind'], src_path,
        '--host', str(host), '--port', str(port),
        '--fps',     str(int(cfg.get('fps', 30))),
        '--quality', str(int(cfg.get('quality', 80))),
    ]
    if cfg['source_kind'] == 'jetson':
        cmd += ['--gst-width',  str(int(cfg.get('gst_width', 1280))),
                '--gst-height', str(int(cfg.get('gst_height', 720))),
                '--gst-fps',    str(int(cfg.get('gst_fps', 30)))]

    print(f"[vision-camera] auto-start: {' '.join(cmd)}")
    print(f"[vision-camera] child log → {_LOG}")
    _set_state(state='starting', pid=None, started_at=time.time(),
               source_kind=cfg['source_kind'], source_path=src_path,
               exit_code=None, last_error=None)

    try:
        _LOG.parent.mkdir(parents=True, exist_ok=True)
        log_fh = open(_LOG, 'w', buffering=1)
    except Exception as e:
        print(f"[vision-camera] could not open log file ({e}) — DEVNULL")
        log_fh = subprocess.DEVNULL

    popen_kw = {
        'cwd': str(_SOFTWARE),
        'stdout': log_fh,
        'stderr': subprocess.STDOUT,
    }
    if os.name == 'posix':
        popen_kw['start_new_session'] = True
    else:
        popen_kw['creationflags'] = getattr(
            subprocess, 'CREATE_NEW_PROCESS_GROUP', 0)

    try:
        _proc = subprocess.Popen(cmd, **popen_kw)
    except Exception as e:
        print(f"[vision-camera] auto-start failed: {e}")
        _set_state(state='failed', last_error=str(e))
        _log_holos(f"[vision-camera] auto-start failed: {e}")
        _proc = None
        return {'started': False, 'reason': str(e), 'config': cfg}

    _set_state(pid=_proc.pid)
    atexit.register(stop)
    threading.Thread(target=_monitor, daemon=True,
                     name='vision-camera-monitor').start()

    # Wait a short window for /api/status to come up.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if _is_port_serving(host, port):
            print(f"[vision-camera] up at http://{host}:{port}/  (pid={_proc.pid})")
            _set_state(state='running', last_error=None)
            _log_holos(f"[vision-camera] running pid={_proc.pid} "
                       f"({cfg['source_kind']}:{cfg['source_path']})")
            return {'started': True, 'pid': _proc.pid, 'config': cfg}
        if _proc.poll() is not None:
            tail = _tail_log()
            summary = _summary_from_tail(tail)
            print(f"[vision-camera] child exited early "
                  f"(code={_proc.returncode}): {summary}")
            _set_state(state='failed', exit_code=_proc.returncode,
                       last_error=summary)
            _log_holos(f"[vision-camera] failed to start "
                       f"(code={_proc.returncode}): {summary}")
            _proc = None
            return {'started': False, 'reason': 'child-exit', 'config': cfg}
        time.sleep(0.2)

    print(f"[vision-camera] subprocess pid={_proc.pid} not yet serving — "
          f"continuing anyway")
    _set_state(state='running',
               last_error='slow boot — /api/status not responding within 5s')
    return {'started': True, 'pid': _proc.pid, 'config': cfg, 'warn': 'slow-boot'}


def stop() -> None:
    global _proc
    if _proc is None:
        return
    if _proc.poll() is None:
        try:
            _proc.terminate()
            _proc.wait(timeout=2.0)
        except Exception:
            try:
                _proc.kill()
            except Exception:
                pass
    _proc = None


def status() -> dict:
    """Snapshot of supervisor state — consumed by the topbar status endpoint."""
    with _state_lock:
        out = dict(_state)
    # Refresh "alive" each call; an external instance might have come/gone.
    if out['state'] == 'running' and out.get('host') and out.get('port'):
        if not _is_port_serving(out['host'], int(out['port'])):
            out['state'] = 'unreachable'
    elif out['state'] in ('idle', 'disabled', 'external') and out.get('host') and out.get('port'):
        if _is_port_serving(out['host'], int(out['port'])):
            # Caught one popping up
            out['state'] = 'external' if out['state'] != 'disabled' else out['state']
    return out
