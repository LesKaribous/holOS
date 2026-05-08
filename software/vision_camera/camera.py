"""
Virtual-camera server.

Reads a source (video file, image, or USB camera) and exposes the frames
as an MJPEG-over-HTTP stream so any other process can consume them as if
they were a real camera. Comes with a tiny web UI to drive playback
(pause / play / step / speed / seek).

NO pipeline analysis happens here — that's the vision_runner's job.
This module is only about emulating a camera.

Usage:
    cd software
    python -m vision_camera.camera video path\to\clip.mp4
    python -m vision_camera.camera camera 0
    python -m vision_camera.camera image path\to\photo.png
    python -m vision_camera.camera jetson /dev/video0  # GStreamer + nvv4l2decoder

    # optional
    --port 5174     bind port (default 5174)
    --host 0.0.0.0  bind address (default 127.0.0.1, use 0.0.0.0 for LAN)
    --gst-width / --gst-height / --gst-fps  (jetson only, default 1280×720@30)

Endpoints (default base http://127.0.0.1:5174):
    GET  /              → web UI (preview + playback controls)
    GET  /stream.mjpg   → raw MJPEG stream (for cv2.VideoCapture / browsers)
    GET  /api/status    → JSON state {playback, speed, frame_idx, …}
    POST /api/control   → JSON {playback, speed, seek_target} — atomic
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent

import cv2
from flask import Flask, Response, render_template, request, jsonify


# ─── Source → JPEG worker ──────────────────────────────────────────────────

class VirtualCamera:
    """Worker thread that reads from cv2 + serves the latest JPEG.

    Playback model mirrors source.video for consistency with holOS:
        live  — read every tick (default for cameras)
        play  — advance one frame per tick (default for video files)
        pause — keep returning the last decoded frame; no advance
        step  — advance one frame, then pause
        step_back — go back one frame, then pause
        seek  — jump to seek_target (frame index), then pause
    """

    def __init__(self, source_kind: str, source_path: str,
                 jpeg_quality: int = 80, target_fps: int = 30,
                 gst_width: int = 1280, gst_height: int = 720,
                 gst_fps: int = 30):
        self.source_kind = source_kind
        self.source_path = source_path
        self.jpeg_quality = int(jpeg_quality)
        self.target_fps = max(1, int(target_fps))
        self.gst_width = int(gst_width)
        self.gst_height = int(gst_height)
        self.gst_fps = int(gst_fps)
        # User-driven state
        self.playback = 'pause' if source_kind == 'video' else 'live'
        self.speed = 1.0
        self.seek_target = 0
        # Internal state
        self.frame_idx = 0
        self.frame_count = -1
        self._cap: Optional[cv2.VideoCapture] = None
        self._still: Optional[any] = None      # for image source
        self._last_jpeg: Optional[bytes] = None
        self._lock = threading.RLock()
        self._evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._slow_acc = 0.0
        self._last_error: Optional[str] = None

    # -- lifecycle -------------------------------------------------

    def start(self):
        self._open()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name='virtual-camera')
        self._thread.start()

    def shutdown(self):
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _open(self):
        if self.source_kind == 'image':
            img = cv2.imread(self.source_path)
            if img is None:
                raise RuntimeError(f'cv2.imread failed: {self.source_path}')
            self._still = img
            self._encode(img)
            self.frame_count = 1
            return
        if self.source_kind == 'jetson':
            pipeline = self._gst_pipeline(self.source_path,
                                          self.gst_width, self.gst_height,
                                          self.gst_fps)
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                raise RuntimeError(
                    f'cv2.VideoCapture (GStreamer) could not open: {pipeline}')
            self.frame_count = -1
            ok, f = self._cap.read()
            if ok:
                self._encode(f)
            return
        if self.source_kind == 'camera':
            arg = (int(self.source_path)
                   if str(self.source_path).isdigit() else self.source_path)
        else:   # video
            arg = self.source_path
        self._cap = cv2.VideoCapture(arg)
        if not self._cap.isOpened():
            raise RuntimeError(f'cv2.VideoCapture could not open {arg!r}')
        self.frame_count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT) or -1)
        # Pre-decode one frame so /stream.mjpg has something to send right away.
        ok, f = self._cap.read()
        if ok:
            self._encode(f)
            try:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            except Exception:
                pass

    @staticmethod
    def _gst_pipeline(device: str, width: int, height: int, fps: int) -> str:
        """Jetson MJPEG → BGR pipeline with hardware decode.

        Camera must support MJPEG at the requested width/height/fps —
        check with `v4l2-ctl --device=/dev/videoN --list-formats-ext`."""
        return (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height},framerate={fps}/1 ! '
            f'nvv4l2decoder mjpeg=1 ! '
            f'nvvidconv ! video/x-raw,format=BGRx ! '
            f'videoconvert ! video/x-raw,format=BGR ! '
            f'appsink drop=true max-buffers=2 sync=false'
        )

    def _encode(self, frame):
        try:
            ok, buf = cv2.imencode('.jpg', frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        except Exception as e:
            self._last_error = f'imencode: {e}'
            return
        if not ok:
            return
        with self._lock:
            self._last_jpeg = bytes(buf)
            self._evt.set()

    def _read_one(self):
        """Read next frame from the capture, with looping."""
        ok, f = self._cap.read()
        if not ok and self.source_kind == 'video':
            try:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            except Exception:
                pass
            ok, f = self._cap.read()
        if ok:
            self._encode(f)
            self.frame_idx = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES) or 0)

    # -- worker loop ----------------------------------------------

    def _loop(self):
        period = 1.0 / self.target_fps
        last_t = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if (now - last_t) < period:
                time.sleep(0.005)
                continue
            last_t = now

            # Image source: nothing to do, the still already encoded once.
            if self.source_kind == 'image':
                # Re-pulse the event so long-poll clients that just
                # connected get something.
                if self._last_jpeg is not None:
                    self._evt.set()
                continue

            mode = self.playback

            if mode == 'pause':
                if self._last_jpeg is not None:
                    self._evt.set()
                continue

            if mode == 'step':
                self._read_one()
                self.playback = 'pause'
                continue

            if mode == 'step_back':
                cur = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES) or 0)
                try:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, cur - 2))
                except Exception:
                    pass
                self._read_one()
                self.playback = 'pause'
                continue

            if mode == 'seek':
                try:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES,
                                  max(0, int(self.seek_target)))
                except Exception:
                    pass
                self._read_one()
                self.playback = 'pause'
                continue

            # 'play' (video) or 'live' (camera)
            speed = float(self.speed) if self.speed > 0 else 1.0
            if mode == 'play' and speed < 1.0:
                self._slow_acc += speed
                if self._slow_acc < 1.0:
                    if self._last_jpeg is not None:
                        self._evt.set()
                    continue
                self._slow_acc -= 1.0

            self._read_one()

            # Fast-forward: skip extra frames per tick when speed > 1.
            if mode == 'play' and speed > 1.0:
                for _ in range(int(speed - 1)):
                    self._read_one()

    # -- API surface ----------------------------------------------

    def set_params(self, params: dict):
        # seek_target FIRST so a follow-up playback='seek' uses it cleanly
        if 'seek_target' in params:
            try:
                self.seek_target = int(params['seek_target'])
            except (TypeError, ValueError):
                pass
        if 'playback' in params:
            self.playback = str(params['playback'])
        if 'speed' in params:
            try:
                self.speed = float(params['speed'])
            except (TypeError, ValueError):
                pass
        if 'jpeg_quality' in params:
            try:
                self.jpeg_quality = max(30, min(95, int(params['jpeg_quality'])))
            except (TypeError, ValueError):
                pass

    def status(self) -> dict:
        return {
            'source_kind': self.source_kind,
            'source_path': self.source_path,
            'playback':    self.playback,
            'speed':       self.speed,
            'frame_idx':   self.frame_idx,
            'frame_count': self.frame_count,
            'jpeg_quality': self.jpeg_quality,
            'last_error':  self._last_error,
        }

    def mjpeg(self):
        """Generator yielding multipart MJPEG chunks.

        On a fresh frame → emit immediately.
        On pause (same JPEG hangs around) → re-emit every KEEPALIVE_S so
        downstream consumers (cv2.VideoCapture in holOS) don't think the
        stream has stalled and drop the connection. Without this, pause
        intermittently froze holOS's pipeline."""
        boundary = b'--frame'
        last_jpeg = None
        last_send_t = 0.0
        KEEPALIVE_S = 1.0
        import time as _t
        while not self._stop.is_set():
            self._evt.wait(timeout=0.5)
            with self._lock:
                jpeg = self._last_jpeg
                self._evt.clear()
            if jpeg is None:
                continue
            now = _t.monotonic()
            is_new = (jpeg is not last_jpeg)
            if not is_new and (now - last_send_t) < KEEPALIVE_S:
                continue
            last_jpeg = jpeg
            last_send_t = now
            yield (boundary + b'\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                   + jpeg + b'\r\n')


# ─── Flask app ─────────────────────────────────────────────────────────────

app = Flask(__name__,
            template_folder=str(_HERE / 'web' / 'templates'),
            static_folder=str(_HERE / 'web' / 'static'),
            static_url_path='/static')

_cam: Optional[VirtualCamera] = None


@app.route('/')
def index():
    return render_template('index.html',
        source_kind=_cam.source_kind if _cam else '',
        source_path=_cam.source_path if _cam else '')


@app.route('/stream.mjpg')
def stream_mjpg():
    if _cam is None:
        return Response('camera not initialised', status=503)
    return Response(_cam.mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-cache, private',
                             'Pragma': 'no-cache'})


@app.route('/api/status')
def api_status():
    return jsonify(_cam.status() if _cam else {})


@app.route('/api/control', methods=['POST'])
def api_control():
    if _cam is None:
        return jsonify({'ok': False, 'error': 'no camera'}), 503
    body = request.get_json(force=True) or {}
    if 'params' in body and isinstance(body['params'], dict):
        params = body['params']
    elif 'key' in body and 'value' in body:
        params = {body['key']: body['value']}
    else:
        return jsonify({'ok': False,
                        'error': 'expected {key,value} or {params:{…}}'}), 400
    _cam.set_params(params)
    return jsonify({'ok': True, 'state': _cam.status()})


# ─── Entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Virtual-camera server (MJPEG stream + playback UI).')
    parser.add_argument('source_kind',
        choices=['video', 'camera', 'image', 'jetson'])
    parser.add_argument('source_path',
        help='File path (video/image), device index (camera), '
             'or /dev/videoN (jetson).')
    parser.add_argument('--port', type=int, default=5174,
        help='HTTP port (default 5174).')
    parser.add_argument('--host', default='127.0.0.1',
        help='Bind address (default 127.0.0.1, use 0.0.0.0 for LAN).')
    parser.add_argument('--quality', type=int, default=80,
        help='JPEG quality 30-95 (default 80).')
    parser.add_argument('--fps', type=int, default=30,
        help='Max output FPS (default 30).')
    parser.add_argument('--gst-width', type=int, default=1280,
        help='Jetson GStreamer capture width (default 1280).')
    parser.add_argument('--gst-height', type=int, default=720,
        help='Jetson GStreamer capture height (default 720).')
    parser.add_argument('--gst-fps', type=int, default=30,
        help='Jetson GStreamer capture FPS (default 30).')
    args = parser.parse_args()

    global _cam
    _cam = VirtualCamera(args.source_kind, args.source_path,
                         jpeg_quality=args.quality, target_fps=args.fps,
                         gst_width=args.gst_width, gst_height=args.gst_height,
                         gst_fps=args.gst_fps)
    try:
        _cam.start()
    except Exception as e:
        print(f'[vision-camera] start failed: {e}', file=sys.stderr)
        sys.exit(2)

    print(f'[vision-camera] http://{args.host}:{args.port}/')
    print(f'[vision-camera]   stream → http://{args.host}:{args.port}/stream.mjpg')
    try:
        app.run(host=args.host, port=args.port, threaded=True,
                use_reloader=False)
    finally:
        _cam.shutdown()


if __name__ == '__main__':
    main()
