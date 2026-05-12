"""
FrameSource — single in-process source-of-truth for camera/video frames.

One reader thread captures at hardware rate and overwrites a 1-slot,
mutex-protected `_latest_bgr` buffer. Every consumer (pipeline source
nodes, recorder, snapshot endpoint) calls `read_latest()` which snapshots
that slot. There is no queue anywhere — old frames are discarded the
instant a new one arrives.

source_kind ∈ {'camera', 'video', 'image'}:
  camera — V4L2 device. On Jetson, uses GStreamer w/ nvv4l2decoder for
           GPU MJPEG decode. Elsewhere falls back to cv2.VideoCapture
           with MJPG fourcc forced before resolution (otherwise cv2
           negotiates uncompressed YUYV → driver throttles at 1080p).
  video  — local file. Supports playback ∈ {play, pause, step, step_back,
           seek}. Reader runs at the file's fps (× speed). On EOF the
           file loops by default.
  image  — single still ndarray, always returned by read_latest().

Replaces the prior vision_camera subprocess + MJPEG-over-HTTP design.
Both halves used to live on the same Jetson but talked through an HTTP
MJPEG stream — FFmpeg's input buffer FIFO'd frames so the pipeline saw
data several × 33 ms behind reality. Moving the reader in-process kills
the buffer entirely.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


# ─── Jetson detection ──────────────────────────────────────────────────────

def _is_jetson() -> bool:
    """True on NVIDIA Jetson boards — drives whether the camera source
    uses GStreamer (with hw MJPEG decode via nvv4l2decoder) or plain cv2."""
    try:
        with open('/proc/device-tree/model', 'rb') as f:
            return b'Jetson' in f.read()
    except Exception:
        return False

IS_JETSON = _is_jetson()


# ─── FrameSource ───────────────────────────────────────────────────────────

class FrameSource:
    """One owner of the camera/video, one reader thread, one slot.

    Construct with a config dict (the same shape as vision_config.json).
    Call `start()` once. Consumers call `read_latest()` whenever they want.
    """

    # Read-cap on the reader thread when the underlying source has no
    # natural rate-limit (file-based video can decode at hundreds of fps).
    # Set to hardware rate (≈ source_fps) so the reader doesn't burn CPU
    # spinning ahead. Camera sources are naturally rate-limited by V4L2.
    _DEFAULT_READ_HZ = 30

    def __init__(self, config: dict):
        self._cfg = dict(config or {})
        # Per-call source override — supersedes config until cleared.
        self._override: dict = {}
        # Underlying handles
        self._cap: Optional[cv2.VideoCapture] = None
        self._still: Optional[np.ndarray] = None
        # Latest BGR slot (the entire point of this module)
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_t: float = 0.0
        self._slot_lock = threading.Lock()
        # Reader thread
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        # Playback state — only meaningful for source_kind='video'
        self._playback: str = 'live'
        self._speed: float = 1.0
        self._seek_target: int = 0
        self._frame_idx: int = 0
        self._frame_count: int = -1
        self._fps_hint: float = 30.0
        # Diagnostics
        self._opened_at: Optional[float] = None
        self._last_error: Optional[str] = None
        self._read_count: int = 0
        # State lock — protects _cap swap during set_source()
        self._state_lock = threading.RLock()

    # ── lifecycle ───────────────────────────────────────────────────────

    def start(self) -> None:
        with self._state_lock:
            self._open_locked()
            if self._cap is not None or self._still is not None:
                self._stop.clear()
                self._thread = threading.Thread(
                    target=self._reader_loop, daemon=True,
                    name='frame-source-reader')
                self._thread.start()
                self._opened_at = time.time()

    def shutdown(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None
        with self._state_lock:
            self._release_locked()

    def _release_locked(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._still = None

    # ── source open ─────────────────────────────────────────────────────

    def _effective(self, key: str, default=None):
        if key in self._override:
            return self._override[key]
        return self._cfg.get(key, default)

    def _resolve_path(self, path: str) -> str:
        """Resolve relative paths against the software/ dir."""
        if not path:
            return path
        if path.startswith(('/dev/', 'http://', 'https://', 'rtsp://')):
            return path
        p = Path(path)
        if p.is_absolute():
            return str(p)
        return str(Path(__file__).resolve().parent / path)

    def _open_locked(self) -> None:
        kind = str(self._effective('source_kind', 'video'))
        path = self._resolve_path(str(self._effective('source_path', '')))
        self._last_error = None
        self._frame_idx = 0
        self._frame_count = -1

        if kind == 'image':
            img = cv2.imread(path)
            if img is None:
                self._last_error = f'cv2.imread failed: {path}'
                return
            self._still = img
            with self._slot_lock:
                self._latest_bgr = img
                self._latest_t = time.monotonic()
            self._frame_count = 1
            self._playback = 'live'
            return

        if kind == 'video':
            if not path or not os.path.exists(path):
                self._last_error = f'file not found: {path}'
                return
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                self._last_error = f'cv2.VideoCapture could not open {path!r}'
                return
            self._cap = cap
            fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            self._fps_hint = float(fps) if fps > 0 else 30.0
            fc = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or -1)
            self._frame_count = fc if fc > 0 else -1
            self._playback = 'play'
            return

        if kind == 'camera':
            # Jetson: GStreamer w/ nvv4l2decoder (GPU MJPEG decode).
            # Other Linux / Windows: plain cv2.VideoCapture with MJPG fourcc.
            gst_w = int(self._effective('gst_width', 1280))
            gst_h = int(self._effective('gst_height', 720))
            gst_fps = int(self._effective('gst_fps', 30))
            if IS_JETSON:
                pipeline = (
                    f'v4l2src device={path} ! '
                    f'image/jpeg,width={gst_w},height={gst_h},framerate={gst_fps}/1 ! '
                    f'nvv4l2decoder mjpeg=1 ! '
                    f'nvvidconv ! video/x-raw,format=BGRx ! '
                    f'videoconvert ! video/x-raw,format=BGR ! '
                    f'appsink drop=true max-buffers=2 sync=false'
                )
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    self._last_error = (
                        f'GStreamer pipeline failed for {path!r} — falling back '
                        f'to V4L2 direct'
                    )
                    cap = None
                else:
                    self._cap = cap
                    self._fps_hint = float(gst_fps)
                    self._playback = 'live'
                    return
            # Non-Jetson or GStreamer fallback
            arg = int(path) if path.isdigit() else path
            cap = cv2.VideoCapture(arg)
            if not cap.isOpened():
                self._last_error = f'cv2.VideoCapture could not open {arg!r}'
                return
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  gst_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, gst_h)
            cap.set(cv2.CAP_PROP_FPS,          gst_fps)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            self._cap = cap
            self._fps_hint = float(gst_fps)
            self._playback = 'live'
            return

        self._last_error = f'unknown source_kind: {kind!r}'

    # ── reader thread ──────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """Pull frames from the underlying source as fast as it produces
        them, overwriting `_latest_bgr` each time. For video files this
        loop also handles play/pause/step/seek by setting POS_FRAMES on
        the capture.

        The 1-slot design means we don't care about the rate downstream
        is sampling at — old frames are simply discarded."""
        while not self._stop.is_set():
            try:
                # Image source: nothing to do, _latest_bgr already set.
                if self._still is not None:
                    time.sleep(0.1)
                    continue
                if self._cap is None:
                    time.sleep(0.05)
                    continue
                kind = str(self._effective('source_kind', 'video'))
                if kind == 'video':
                    self._video_tick()
                else:   # camera / live
                    self._camera_tick()
            except Exception as e:
                self._last_error = f'reader: {type(e).__name__}: {e}'
                time.sleep(0.05)

    def _camera_tick(self) -> None:
        """Single read from a live source. V4L2 blocks until the next
        frame arrives; GStreamer's appsink with drop=true max-buffers=2
        means we always get a fresh frame, never a buffered one."""
        cap = self._cap
        if cap is None:
            return
        ok, f = cap.read()
        if not ok or f is None:
            time.sleep(0.005)
            return
        self._frame_idx += 1
        with self._slot_lock:
            self._latest_bgr = f
            self._latest_t = time.monotonic()
        self._read_count += 1

    def _video_tick(self) -> None:
        """File-based video — honour play/pause/seek/step/speed.

        Reader paces itself at native_fps × speed (so playback feels
        natural). 'pause' just sleeps without consuming frames so the
        last decoded frame stays in the slot."""
        cap = self._cap
        if cap is None:
            return

        mode = self._playback

        if mode == 'pause':
            time.sleep(0.05)
            return

        if mode == 'step':
            self._read_one_video()
            self._playback = 'pause'
            return

        if mode == 'step_back':
            cur = int(cap.get(cv2.CAP_PROP_POS_FRAMES) or 0)
            target = max(0, cur - 2)
            try:
                cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            except Exception:
                pass
            self._read_one_video()
            self._playback = 'pause'
            return

        if mode == 'seek':
            try:
                cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, int(self._seek_target)))
            except Exception:
                pass
            self._read_one_video()
            self._playback = 'pause'
            return

        # play / live: pace to native fps × speed
        speed = self._speed if self._speed > 0 else 1.0
        period = 1.0 / max(1.0, self._fps_hint * speed)
        t0 = time.monotonic()
        self._read_one_video()
        dt = time.monotonic() - t0
        sleep = period - dt
        if sleep > 0:
            # Cap sleep at 50 ms so a paused or stalled reader notices
            # state changes (stop, mode flip) within reasonable latency.
            time.sleep(min(sleep, 0.05))

    def _read_one_video(self) -> None:
        cap = self._cap
        if cap is None:
            return
        ok, f = cap.read()
        if not ok or f is None:
            if bool(self._effective('loop', True)) and self._frame_count > 0:
                try:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                except Exception:
                    pass
                ok, f = cap.read()
            if not ok or f is None:
                return
        self._frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES) or 0)
        with self._slot_lock:
            self._latest_bgr = f
            self._latest_t = time.monotonic()
        self._read_count += 1

    # ── consumer API ───────────────────────────────────────────────────

    def read_latest(self) -> Optional[np.ndarray]:
        """Snapshot the latest BGR frame. Returns None until the reader
        has produced its first frame. Never blocks."""
        with self._slot_lock:
            return self._latest_bgr

    def is_open(self) -> bool:
        return (self._cap is not None and self._cap.isOpened()) or \
               self._still is not None

    # ── runtime control ────────────────────────────────────────────────

    def set_source(self, source_kind: str, source_path: str) -> dict:
        """Swap source on the fly. Runtime override only — vision_config.json
        is untouched; clear_override() restores disk config."""
        with self._state_lock:
            self._override = {
                'source_kind': str(source_kind),
                'source_path': str(source_path),
            }
            # Stop reader, swap source, restart reader.
            self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None
        with self._state_lock:
            self._release_locked()
            self._open_locked()
        if self._cap is not None or self._still is not None:
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._reader_loop, daemon=True,
                name='frame-source-reader')
            self._thread.start()
        return self.status()

    def clear_override(self) -> dict:
        """Drop runtime override; re-open on the disk config."""
        self._override = {}
        return self.set_source(
            self._cfg.get('source_kind', 'video'),
            self._cfg.get('source_path', ''),
        ) if self._cfg.get('source_path') else self.status()

    def set_playback(self, mode: str) -> None:
        if mode in ('play', 'pause', 'step', 'step_back', 'seek', 'live'):
            self._playback = mode

    def set_seek_target(self, frame_idx: int) -> None:
        self._seek_target = max(0, int(frame_idx))

    def set_speed(self, speed: float) -> None:
        try:
            self._speed = max(0.05, float(speed))
        except (TypeError, ValueError):
            pass

    # ── status (UI surface) ────────────────────────────────────────────

    def status(self) -> dict:
        kind = str(self._effective('source_kind', 'video'))
        path = str(self._effective('source_path', ''))
        with self._slot_lock:
            has_frame = self._latest_bgr is not None
            shape = list(self._latest_bgr.shape) if has_frame else None
            age_s = (time.monotonic() - self._latest_t) if has_frame else None
        state = 'running' if (self.is_open() and self._thread and self._thread.is_alive()) \
                else ('failed' if self._last_error else 'idle')
        return {
            'state':            state,
            'source_kind':      kind,
            'source_path':      path,
            'on_jetson':        IS_JETSON,
            'frame_idx':        self._frame_idx,
            'frame_count':      self._frame_count,
            'playback':         self._playback,
            'speed':            self._speed,
            'seek_target':      self._seek_target,
            'last_error':       self._last_error,
            'has_frame':        has_frame,
            'frame_shape':      shape,
            'frame_age_s':      round(age_s, 3) if age_s is not None else None,
            'read_count':       self._read_count,
            'runtime_override': dict(self._override),
        }


# ─── Module-level singleton ────────────────────────────────────────────────
# Mirrors the old vision_camera supervisor's _proc global — one source per
# process. Constructed by run.py at boot; consumed by source.* pipeline
# nodes and the recorder.

_INSTANCE: Optional[FrameSource] = None
_INSTANCE_LOCK = threading.Lock()


def start(config: dict) -> FrameSource:
    """Construct + start the singleton FrameSource. Idempotent: a second
    call shuts down the previous instance first."""
    global _INSTANCE
    with _INSTANCE_LOCK:
        if _INSTANCE is not None:
            _INSTANCE.shutdown()
        _INSTANCE = FrameSource(config)
        _INSTANCE.start()
        return _INSTANCE


def get() -> Optional[FrameSource]:
    return _INSTANCE


def shutdown() -> None:
    global _INSTANCE
    with _INSTANCE_LOCK:
        if _INSTANCE is not None:
            _INSTANCE.shutdown()
            _INSTANCE = None
