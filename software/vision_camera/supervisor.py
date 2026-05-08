"""Auto-start the virtual-camera server alongside holOS.

Reads `software/data/vision_camera_config.json` (seeded with sane
defaults on first run). When `auto_start` is true and port 5174 isn't
already serving, spawn `python -m vision_camera.camera ...` as a daemon
subprocess and tear it down on holOS exit. If something is already
serving on that port (e.g. user launched vision_camera.bat manually),
reuse it instead of double-launching.
"""

from __future__ import annotations

import atexit
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent
_SOFTWARE = _HERE.parent
_CONFIG = _SOFTWARE / 'data' / 'vision_camera_config.json'

DEFAULT_CONFIG: dict = {
    'auto_start':  True,
    'source_kind': 'video',         # video | camera | image | jetson
    'source_path': 'vision/homography/data/video.2.mp4',
    'host':        '127.0.0.1',
    'port':        5174,
    'fps':         30,
    'quality':     80,
    # Jetson-only knobs (ignored for other source kinds)
    'gst_width':   1280,
    'gst_height':  720,
    'gst_fps':     30,
}

_proc: Optional[subprocess.Popen] = None


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
    try:
        with urllib.request.urlopen(
                f'http://{host}:{port}/api/status', timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


def start() -> dict:
    """Spawn the virtual-camera subprocess if config allows. Idempotent."""
    global _proc
    cfg = _load_config()
    if not cfg.get('auto_start', True):
        print("[vision-camera] auto_start disabled in config — skipping")
        return {'started': False, 'reason': 'auto_start=false', 'config': cfg}
    if _proc is not None and _proc.poll() is None:
        return {'started': False, 'reason': 'already-spawned', 'config': cfg}

    host, port = cfg['host'], int(cfg['port'])
    if _is_port_serving(host, port):
        print(f"[vision-camera] http://{host}:{port}/ already responding — "
              f"reusing existing instance")
        return {'started': False, 'reason': 'external-instance', 'config': cfg}

    src_path = _resolve_source(cfg['source_kind'], cfg['source_path'])
    cmd = [
        sys.executable, '-m', 'vision_camera.camera',
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
    popen_kw = {
        'cwd': str(_SOFTWARE),
        'stdout': subprocess.DEVNULL,
        'stderr': subprocess.DEVNULL,
    }
    # Detach from holOS's process group so a Ctrl-C in the holOS
    # terminal doesn't race our atexit cleanup.
    if os.name == 'posix':
        popen_kw['start_new_session'] = True
    else:
        popen_kw['creationflags'] = getattr(
            subprocess, 'CREATE_NEW_PROCESS_GROUP', 0)

    try:
        _proc = subprocess.Popen(cmd, **popen_kw)
    except Exception as e:
        print(f"[vision-camera] auto-start failed: {e}")
        _proc = None
        return {'started': False, 'reason': str(e), 'config': cfg}
    atexit.register(stop)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if _is_port_serving(host, port):
            print(f"[vision-camera] up at http://{host}:{port}/  (pid={_proc.pid})")
            return {'started': True, 'pid': _proc.pid, 'config': cfg}
        if _proc.poll() is not None:
            print(f"[vision-camera] child exited early "
                  f"(code={_proc.returncode}) — check source path / port")
            _proc = None
            return {'started': False, 'reason': 'child-exit', 'config': cfg}
        time.sleep(0.2)
    print(f"[vision-camera] subprocess pid={_proc.pid} not yet serving — "
          f"continuing anyway")
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
