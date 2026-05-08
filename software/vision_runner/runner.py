"""
Vision pipeline DEBUG runner.

Reads a video stream (typically from the vision_camera virtual-camera
server) and runs the full vision pipeline against it (rectify, aruco,
localization, parallax…). Surfaces every intermediate stage in a web UI
so you can iterate on the pipeline without rebuilding holOS.

Three-tool architecture:
    1. vision_camera   — emulates a camera (no analysis). Port 5174.
    2. vision_runner   — THIS tool: full pipeline + debug views. Port 5175.
    3. holOS           — production app. Port 5000.

vision_runner reads frames from the camera (default URL
http://127.0.0.1:5174/stream.mjpg), so launch vision_camera FIRST.

Usage:
    vision.bat                     (interactive picker; default port 5175)
    vision.bat <path-to-video.mp4>
    vision.bat camera 0
    vision.bat image <path.png>

The runner can also bypass the virtual camera and read a source directly
(file/USB camera) — handy for quick offline checks.

Hot-reload:
  - Watches `vision_runner/pipeline.py` mtime — rebuilds on save.
  - Only the pipeline-builder module is reloaded; nodes (services/...)
    require a full restart if you tweak their implementation.
  - Debounced 250ms after the last save event.
"""

from __future__ import annotations

import argparse
import importlib
import os
import sys
import time
import threading
import traceback
import base64
from pathlib import Path
from typing import Optional

# Make `software/` importable when launched via `python -m vision_runner.runner`
_HERE = Path(__file__).resolve().parent
_SOFTWARE = _HERE.parent
if str(_SOFTWARE) not in sys.path:
    sys.path.insert(0, str(_SOFTWARE))

from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO


# ─── Pipeline lifecycle (with hot-reload) ──────────────────────────────────

class PipelineHost:
    """Owns the current Pipeline + the build_pipeline() module. Watches the
    module file for changes and atomically swaps the pipeline.

    Also holds the latest source JPEG for the MJPEG stream endpoint —
    populated by a feed subscriber on feed_id 'stream' (the pipeline
    must include an `output` node with `feed_id='stream'` wired to the
    source's frame port; pipeline.py does this by default)."""

    def __init__(self, source_kind: str, source_path: str,
                 socketio: SocketIO):
        self.source_kind = source_kind
        self.source_path = source_path
        self.socketio = socketio
        self._pipeline = None              # type: Optional[object]
        self._module = None                # the live build_pipeline module
        self._module_path = _HERE / 'pipeline.py'
        self._last_mtime = 0.0
        self._lock = threading.RLock()
        self._reload_pending: Optional[float] = None
        self._reload_count = 0
        self._last_error: Optional[str] = None
        # MJPEG stream state
        self._stream_jpeg: Optional[bytes] = None
        self._stream_evt = threading.Event()    # set on each new frame

    # -- Build / shutdown ------------------------------------------

    def build(self) -> None:
        """(Re)load the module + build a fresh pipeline. Replaces any
        previous one atomically. Subscribes feed callbacks for the UI."""
        with self._lock:
            # 1. Tear down the old pipeline (if any)
            old = self._pipeline
            self._pipeline = None
            if old is not None:
                try:
                    old.shutdown()
                except Exception:
                    traceback.print_exc()

            # 2. (Re)load pipeline.py
            try:
                if self._module is None:
                    self._module = importlib.import_module(
                        'vision_runner.pipeline')
                else:
                    self._module = importlib.reload(self._module)
                self._last_mtime = self._module_path.stat().st_mtime
            except Exception as e:
                self._last_error = f'reload: {type(e).__name__}: {e}'
                print(f'[runner] {self._last_error}')
                traceback.print_exc()
                return

            # 3. Build a new pipeline
            try:
                p = self._module.build_pipeline(
                    self.source_kind, self.source_path)
            except Exception as e:
                self._last_error = f'build: {type(e).__name__}: {e}'
                print(f'[runner] {self._last_error}')
                traceback.print_exc()
                return

            # 4. Wire feed subscribers — every output node's feed_id
            #    becomes a SocketIO emit.
            for nid in p.node_ids():
                node = p.get_node(nid)
                if node is None:
                    continue
                feed_id = (getattr(node, '_params', {}) or {}).get('feed_id')
                if feed_id:
                    p.subscribe_feed(feed_id, self._make_emitter(feed_id))

            # 5. Start it
            self._pipeline = p
            p.enable()
            self._last_error = None
            self._reload_count += 1
            print(f'[runner] pipeline built (#{self._reload_count}), '
                  f'{len(p.node_ids())} nodes, source={self.source_kind}')
            self.socketio.emit('runner_status', self.status())

    def shutdown(self) -> None:
        with self._lock:
            if self._pipeline is not None:
                try:
                    self._pipeline.shutdown()
                except Exception:
                    pass
                self._pipeline = None

    # -- Hot-reload watcher ----------------------------------------

    def watch_loop(self, interval_s: float = 0.5) -> None:
        """Poll pipeline.py mtime; debounce + rebuild on change."""
        while True:
            try:
                m = self._module_path.stat().st_mtime
                if m > self._last_mtime + 0.001:
                    # File changed; debounce in case the editor is still flushing.
                    self._reload_pending = time.monotonic()
                    self._last_mtime = m
                if (self._reload_pending is not None
                        and (time.monotonic() - self._reload_pending) > 0.25):
                    self._reload_pending = None
                    print('[runner] pipeline.py changed → rebuilding')
                    self.build()
            except Exception:
                traceback.print_exc()
            time.sleep(interval_s)

    # -- Feed publishing -------------------------------------------

    def _make_emitter(self, feed_id: str):
        """Closure that emits a single feed_id. Polymorphic:
        pipeline calls (jpeg_bytes, meta) for image feeds OR (dict, meta)
        for pose_list / json feeds.

        Special-case: feed_id == 'stream' also feeds the MJPEG endpoint
        (raw JPEG bytes go into self._stream_jpeg, served by /stream.mjpg).
        """
        def emit(payload, meta):
            try:
                if isinstance(payload, (bytes, bytearray)):
                    raw = bytes(payload)
                    # MJPEG-stream side-channel: keep the latest JPEG for
                    # any /stream.mjpg client. They block on _stream_evt.
                    if feed_id == 'stream':
                        self._stream_jpeg = raw
                        self._stream_evt.set()
                    msg = {
                        'feed_id': feed_id,
                        'kind':    'frame',
                        'jpeg_b64': base64.b64encode(raw).decode(),
                        'meta':    meta or {},
                    }
                else:
                    msg = {
                        'feed_id': feed_id,
                        'kind':    'data',
                        'payload': payload,
                        'meta':    meta or {},
                    }
                self.socketio.emit('feed', msg)
            except Exception as e:
                print(f'[runner] feed {feed_id} emit failed: {e}')
        return emit

    # -- MJPEG stream generator ------------------------------------

    def mjpeg_stream(self, max_fps: int = 30):
        """Generator yielding multipart MJPEG chunks. Blocks on the
        _stream_evt event so it serves at the source's actual rate (no
        busy-loop). Times out the wait at 1s so a stalled source doesn't
        keep the connection forever-pending. max_fps caps the rate so a
        fast source doesn't drown a slow consumer."""
        boundary = b'--frame'
        period = 1.0 / max(1, max_fps)
        last_t = 0.0
        last_jpeg = None
        while True:
            self._stream_evt.wait(timeout=1.0)
            jpeg = self._stream_jpeg
            self._stream_evt.clear()
            now = time.monotonic()
            if jpeg is None or jpeg is last_jpeg:
                continue
            if (now - last_t) < period:
                continue
            last_t = now
            last_jpeg = jpeg
            yield (boundary + b'\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                   + jpeg + b'\r\n')

    # -- Control --------------------------------------------------

    def get_pipeline(self):
        return self._pipeline

    def set_source_params(self, params: dict) -> bool:
        """Update the source node's params live (playback, speed, seek_target).
        Accepts a dict so multiple keys can be set atomically — necessary for
        `seek` which needs `seek_target` set BEFORE `playback='seek'` so the
        worker reads them together on the next tick. Returns True on success."""
        if not isinstance(params, dict) or not params:
            return False
        with self._lock:
            p = self._pipeline
            if p is None:
                return False
            src = p.get_node('src')
            if src is None:
                return False
            try:
                # Set seek_target FIRST so playback='seek' picks it up cleanly
                if 'seek_target' in params:
                    src.set_params({'seek_target': params['seek_target']})
                rest = {k: v for k, v in params.items() if k != 'seek_target'}
                if rest:
                    src.set_params(rest)
                return True
            except Exception as e:
                print(f'[runner] set_source_params({params!r}) failed: {e}')
                return False

    def status(self) -> dict:
        with self._lock:
            p = self._pipeline
            if p is None:
                return {
                    'running':      False,
                    'source_kind':  self.source_kind,
                    'source_path':  self.source_path,
                    'reload_count': self._reload_count,
                    'last_error':   self._last_error,
                }
            st = p.state()
            # Trim verbose fields for the UI
            st['source_kind'] = self.source_kind
            st['source_path'] = self.source_path
            st['reload_count'] = self._reload_count
            st['last_error'] = self._last_error or st.get('last_error')
            return st


# ─── Flask + SocketIO web app ──────────────────────────────────────────────

app = Flask(__name__,
            template_folder=str(_HERE / 'web' / 'templates'),
            static_folder=str(_HERE / 'web' / 'static'),
            static_url_path='/static')
app.config['SECRET_KEY'] = 'vision-runner'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

_host: Optional[PipelineHost] = None


@app.route('/')
def index():
    h = _host
    return render_template(
        'index.html',
        source_kind=h.source_kind if h else 'unknown',
        source_path=h.source_path if h else '',
    )


@app.route('/api/status')
def api_status():
    return jsonify(_host.status() if _host else {})


@app.route('/api/source', methods=['POST'])
def api_source():
    """Update source node params live.
       Body: either {key, value}  OR  {params: {k1: v1, k2: v2, …}}.
       The bulk form is needed for actions like ⏮⏮ that combine
       seek_target + playback in one shot."""
    if _host is None:
        return jsonify({'ok': False, 'error': 'no host'}), 503
    body = request.get_json(force=True) or {}
    if 'params' in body and isinstance(body['params'], dict):
        params = body['params']
    elif 'key' in body and 'value' in body:
        params = {body['key']: body['value']}
    else:
        return jsonify({'ok': False,
                        'error': 'expected {key,value} or {params:{…}}'}), 400
    return jsonify({'ok': _host.set_source_params(params)})


@app.route('/api/reload', methods=['POST'])
def api_reload():
    """Force a pipeline rebuild even if pipeline.py hasn't changed."""
    if _host is None:
        return jsonify({'ok': False}), 503
    _host.build()
    return jsonify({'ok': True})


# /stream.mjpg moved to vision_camera (port 5174) — not served by the runner.


# ─── Entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Standalone vision-pipeline runner with hot-reload.')
    parser.add_argument('source_kind',
                        choices=['video', 'camera', 'image'],
                        help='What kind of source to feed the pipeline.')
    parser.add_argument('source_path',
                        help='File path (video/image) or device index (camera).')
    parser.add_argument('--port', type=int, default=5175,
                        help='HTTP port for the control UI (default 5175 — '
                             '5174 is reserved for vision_camera).')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Bind address (default 127.0.0.1, use 0.0.0.0 for LAN).')
    args = parser.parse_args()

    global _host
    _host = PipelineHost(args.source_kind, args.source_path, socketio)
    _host.build()

    # Watcher thread for hot-reload
    threading.Thread(target=_host.watch_loop, daemon=True,
                     name='pipeline-watcher').start()

    print(f'[runner] http://{args.host}:{args.port}/  (Ctrl-C to stop)')
    try:
        socketio.run(app, host=args.host, port=args.port,
                     allow_unsafe_werkzeug=True)
    finally:
        _host.shutdown()


if __name__ == '__main__':
    main()
