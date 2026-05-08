"""
Source nodes — produce frames.

  source/camera   → frame   (USB index)
  source/video    → frame   (file path or HTTP/RTSP URL)
  source/image    → frame   (still image, looped)
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional

# Resolve TwinVision core path so we can reuse VideoSource.
_HERE = os.path.dirname(os.path.abspath(__file__))
_VISION_SRC = os.path.abspath(os.path.join(_HERE, '..', '..', '..', 'vision', 'src'))
if _VISION_SRC not in sys.path:
    sys.path.insert(0, _VISION_SRC)

try:
    import cv2
    from core.video_source import VideoSource, VideoInfo
    _CV2_OK = True
except Exception as _e:
    _CV2_OK = False
    print(f'[vision.nodes.source] cv2 unavailable: {_e}')

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


class _StaticImageSource:
    """Single-image source: always returns the same frame."""

    def __init__(self, path: str, img):
        h, w = img.shape[:2]
        self._img = img
        self._info = VideoInfo(
            width=w, height=h, fps=10.0,
            frame_count=-1, source_name=path,
        )
        self._opened = True
        self._idx = 0

    @property
    def is_open(self): return self._opened
    @property
    def info(self):    return self._info
    @property
    def current_frame_idx(self): return self._idx

    def read(self):
        # Return the SAME ndarray object every tick. At 1080p × 3 bytes
        # = ~6 MB; copying it 25×/s burned ~150 MB/s of memcpy for nothing
        # (downstream nodes only read it; the OutputNode does its own copy
        # before imencode). Downstream nodes that need to mutate must copy.
        if not self._opened:
            return None
        self._idx += 1
        return self._img

    def seek(self, frame_idx: int) -> bool: return False
    def grab_current_frame(self):
        return self._img if self._opened else None
    def release(self):
        self._opened = False


@register_node('source.camera')
class CameraSourceNode(Node):
    """USB camera source (cv2.VideoCapture index). Cameras are 'live' by
    default but the user can pause / step to freeze a frame for debugging."""

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame (clean, for downstream processing)'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        'camera_index': {
            'type': 'int', 'default': 0,
            'label': 'camera index',
            'description': 'OpenCV VideoCapture index (0 = default camera)',
            'min': 0, 'max': 9,
        },
        'playback': {
            'type': 'str', 'default': 'live',
            'label': 'playback',
            'enum': ['live', 'pause', 'step'],
            'description': 'live = stream, pause = freeze last frame, '
                           'step = grab one frame and pause',
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._last_frame = None

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        idx = int(self._params.get('camera_index', 0))
        self._source = VideoSource(idx)
        if not self._source.open():
            self._last_error = f'could not open camera #{idx}'
            self._source = None
            return
        # Try to grab one frame so the editor snapshot has something to show.
        try:
            f0 = self._source.read()
            if f0 is not None:
                self._last_frame = f0
        except Exception:
            pass

    def shutdown(self):
        if getattr(self, '_source', None):
            self._source.release()
            self._source = None

    def process(self, inputs):
        s = getattr(self, '_source', None)
        if s is None or not s.is_open:
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})
        mode = self._params.get('playback', 'live')
        if mode == 'pause':
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})
        f = s.read()
        if f is None:
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})
        self._last_frame = f
        if mode == 'step':
            self._params['playback'] = 'pause'
        # Sources don't add overlays themselves — frame and preview are
        # the same ndarray. Sharing the reference is fine because the
        # downstream OutputNode does its own .copy() before imencode.
        return {'frame': f, 'preview': f}

    def get_state(self):
        s = getattr(self, '_source', None)
        return {
            'open':       bool(s and s.is_open),
            'playback':   self._params.get('playback', 'live'),
            'last_error': self._last_error,
        }


@register_node('source.video')
class VideoSourceNode(Node):
    """Video file / RTSP / HTTP source. Honors a `playback` param so the
    dashboard / editor can drive frame advancement:

        live      — read every tick (default for cameras / live streams)
        play      — advance one frame per tick (default for video files)
        pause     — keep returning the last decoded frame; no advance
        step      — advance one frame forward,  then transition to pause
        step_back — go back one frame,           then transition to pause
        seek      — jump to `seek_target` (frame index), then pause
    """

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame (clean, for downstream processing)'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        'path': {
            'type': 'file_picker', 'kind': 'video', 'default': '',
            'label': 'file or URL',
            'description': 'video file path (relative to project root) or RTSP/HTTP URL',
        },
        'loop': {
            'type': 'bool', 'default': True, 'label': 'loop at EOF',
            'description': 'restart from frame 0 when the video ends',
        },
        'playback': {
            'type': 'str',  'default': 'pause',
            'label': 'playback',
            'enum': ['live', 'play', 'pause', 'step', 'step_back', 'seek'],
            'description': 'driven by the dashboard tile buttons; '
                           'pause is the default at server boot',
        },
        'seek_target': {
            'type': 'int',  'default': 0,
            'label': 'seek target',
            'unit': 'frame',
            'min': 0,
            'description': 'frame index used by playback=seek (the dashboard '
                           '⏭⏭ / ⏮⏮ buttons set this automatically)',
        },
        'speed': {
            'type': 'float', 'default': 1.0,
            'label': 'speed',
            'unit': '×',
            'min': 0.25, 'max': 8, 'step': 0.25,
            'description': '<1 = slow motion, >1 = skip frames (2 = 2× fast-forward)',
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._last_frame = None    # cached for pause / freeze-frame mode
        self._slow_acc   = 0.0     # accumulator for slow-motion (speed < 1)
        # Drainer state — only used for live HTTP/RTSP streams. cv2 buffers
        # MJPEG-over-HTTP internally, so a slow consumer (1 FPS pipeline tick)
        # ends up reading frames that are several seconds old. The drainer
        # thread reads as fast as the source delivers, keeping only the most
        # recent frame; process() returns that snapshot.
        self._drainer_thread: Optional[threading.Thread] = None
        self._drainer_stop = threading.Event()
        self._drainer_lock = threading.Lock()
        self._drainer_frame = None
        self._drainer_err: Optional[str] = None

    def _is_live_stream(self, path: str) -> bool:
        return path.lower().startswith(('rtsp://', 'http://', 'https://'))

    # Rate cap on the drainer. Must be ≥ source rate or the cv2/FFmpeg
    # internal MJPEG buffer grows unbounded — every frame we don't read
    # piles up in the TCP/decode buffer, and `s.read()` returns OLDER
    # frames first → the pipeline sees image data minutes behind reality.
    # 60 Hz is comfortably above any camera setting we run at; cv2.read()
    # naturally throttles itself to the server rate when caught up, so
    # this is a defensive cap, not a target.
    _DRAINER_HZ = 60.0

    def _drainer_loop(self):
        """Read frames as fast as the source produces them (capped at
        _DRAINER_HZ); keep only the latest. Pipeline tick reads from the
        cached slot."""
        s = self._source
        if s is None:
            return
        period = 1.0 / self._DRAINER_HZ
        next_read = time.monotonic()
        while not self._drainer_stop.is_set():
            now = time.monotonic()
            wait = next_read - now
            if wait > 0:
                time.sleep(min(wait, 0.1))
                continue
            next_read = now + period
            try:
                f = s.read()
            except Exception as e:
                self._drainer_err = f'read: {e}'
                time.sleep(0.05)
                continue
            if f is None:
                # Stream hiccup — short wait then retry.
                time.sleep(0.02)
                continue
            with self._drainer_lock:
                self._drainer_frame = f

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        path = str(self._params.get('path', '')).strip()
        if not path:
            self._last_error = 'no path set'
            return
        if not (self._is_live_stream(path) or os.path.exists(path)):
            self._last_error = f'file not found: {path}'
            return
        self._source = VideoSource(path)
        if not self._source.open():
            self._last_error = f'cv2 could not open: {path}'
            self._source = None
            return
        # Decode the first frame on-load so the editor's snapshot endpoint
        # has something to hand back even while the source sits paused.
        # We seek back to 0 so play / step starts from frame 0 too.
        try:
            f0 = self._source.read()
            if f0 is not None:
                self._last_frame = f0
                try:
                    self._source.seek(0)
                except Exception:
                    pass
        except Exception as e:
            self._last_error = f'first-frame read: {e}'

        # For live streams, spin up the drainer so the pipeline always sees
        # a fresh frame regardless of how slowly it ticks.
        if self._is_live_stream(path):
            self._drainer_stop.clear()
            self._drainer_thread = threading.Thread(
                target=self._drainer_loop, daemon=True,
                name=f'src-drainer:{path}')
            self._drainer_thread.start()

    def shutdown(self):
        # Stop the drainer first — it owns reads on _source.
        self._drainer_stop.set()
        t = self._drainer_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._drainer_thread = None
        if getattr(self, '_source', None):
            self._source.release()
            self._source = None

    def process(self, inputs):
        s = getattr(self, '_source', None)
        if s is None or not s.is_open:
            # Even when not open we may have a cached first frame from a
            # previous start(); surface it so downstream nodes can still
            # produce something during pause.
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        # Live streams (HTTP / RTSP / USB cam) — playback param is meaningless
        # here, you can't pause / seek / step a real-time stream. Always read
        # the latest frame from the drainer slot and bypass every playback
        # gate below. This also makes the node robust to a stale 'pause'
        # value left over in the params (which would otherwise freeze the
        # pipeline on the first frame even though the drainer keeps producing).
        if self._drainer_thread is not None and self._drainer_thread.is_alive():
            with self._drainer_lock:
                f = self._drainer_frame
            if f is not None:
                self._last_frame = f
                return {'frame': f, 'preview': f}
            # Drainer hasn't produced yet on the very first tick — surface
            # the cached first-frame so downstream still has something.
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        # Below this point: file-based sources only — playback param applies.
        mode = self._params.get('playback', 'play')

        if mode == 'pause':
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        # ── Discrete actions (do once, then transition to pause) ───────
        if mode == 'step_back':
            cur = s.current_frame_idx
            target = max(0, cur - 2)   # -2 because read() will advance by 1
            try:
                s.seek(target)
            except Exception:
                pass
            f = s.read()
            self._params['playback'] = 'pause'
            if f is not None:
                self._last_frame = f
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        if mode == 'seek':
            target = max(0, int(self._params.get('seek_target', 0)))
            try:
                s.seek(target)
            except Exception:
                pass
            f = s.read()
            self._params['playback'] = 'pause'
            if f is not None:
                self._last_frame = f
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        # ── Continuous play / live ────────────────────────────────────
        speed = float(self._params.get('speed', 1.0) or 1.0)
        if speed <= 0:
            speed = 1.0

        if mode == 'play' and speed < 1.0:
            # Slow motion: emit a fresh frame only on a fraction of ticks.
            self._slow_acc += speed
            if self._slow_acc < 1.0:
                return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})
            self._slow_acc -= 1.0

        # Drainer path — for live HTTP/RTSP streams we never read directly
        # from cv2 here; the drainer thread keeps a fresh frame in the slot
        # so the pipeline tick always sees something <100ms old, even if it
        # ticks once per second.
        if self._drainer_thread is not None and self._drainer_thread.is_alive():
            with self._drainer_lock:
                f = self._drainer_frame
            if f is not None:
                self._last_frame = f
                return {'frame': f, 'preview': f}
            # Drainer hasn't produced yet — fall through to direct read so
            # we don't return None on the very first tick.

        # Read the next frame
        f = s.read()
        if f is None and self._params.get('loop', True):
            info = s.info
            if info and info.frame_count > 0:
                s.seek(0)
                f = s.read()
        if f is None:
            return ({'frame': self._last_frame, 'preview': self._last_frame}
                    if self._last_frame is not None else {})

        # Fast-forward: skip (speed - 1) extra frames per tick when >1.
        if mode == 'play' and speed > 1.0:
            skip = int(speed - 1)
            for _ in range(skip):
                nxt = s.read()
                if nxt is None:
                    if self._params.get('loop', True):
                        info = s.info
                        if info and info.frame_count > 0:
                            s.seek(0)
                            nxt = s.read()
                if nxt is not None:
                    f = nxt

        self._last_frame = f
        if mode == 'step':
            self._params['playback'] = 'pause'
        # Sources don't add overlays themselves — frame and preview are
        # the same ndarray. Sharing the reference is fine because the
        # downstream OutputNode does its own .copy() before imencode.
        return {'frame': f, 'preview': f}

    def get_state(self):
        s = getattr(self, '_source', None)
        info = s.info if (s and s.is_open) else None
        path = str(self._params.get('path', ''))
        is_live = self._is_live_stream(path)
        drainer_alive = bool(self._drainer_thread
                             and self._drainer_thread.is_alive())
        with self._drainer_lock:
            has_drainer_frame = self._drainer_frame is not None
        return {
            'open':           bool(s and s.is_open),
            'frame_idx':      s.current_frame_idx if s else 0,
            'frame_count':    info.frame_count if info else -1,
            # For live streams the playback param is ignored — actual
            # behavior is always "always read latest from drainer".
            'playback':       'live' if is_live else self._params.get('playback', 'play'),
            'is_live_stream': is_live,
            'drainer_alive':  drainer_alive,
            'has_drainer_frame': has_drainer_frame,
            'drainer_err':    self._drainer_err,
            'last_error':     self._last_error,
        }


@register_node('source.image')
class ImageSourceNode(Node):
    """Static image source. Returns the same frame each tick by default.
    With `playback='once'` it emits the frame exactly once after each
    `refresh` ping (the dashboard increments the refresh counter)."""

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame (clean, for downstream processing)'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        'path': {
            'type': 'file_picker', 'kind': 'image', 'default': '',
            'label': 'image file',
            'description': 'PNG / JPG / BMP path (relative to project root)',
        },
        'playback': {
            'type': 'str', 'default': 'every_tick',
            'label': 'emission mode',
            'enum': ['every_tick', 'once'],
            'description': 'every_tick = re-emit each pipeline tick; '
                           'once = emit one frame on each refresh bump',
        },
        'refresh':  {
            'type': 'int', 'default': 0,
            'label': 'refresh count',
            'description': 'incremented by the dashboard ⟳ button to trigger '
                           'one emission in once-mode',
            'min': 0,
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._last_refresh_seen = -1

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        path = str(self._params.get('path', '')).strip()
        if not path or not os.path.exists(path):
            self._last_error = f'file not found: {path}'
            return
        img = cv2.imread(path)
        if img is None:
            self._last_error = f'cv2.imread returned None: {path}'
            return
        self._source = _StaticImageSource(path, img)
        # In 'once' mode, emit the initial frame exactly once.
        self._last_refresh_seen = -1

    def shutdown(self):
        if getattr(self, '_source', None):
            self._source.release()
            self._source = None

    def process(self, inputs):
        s = getattr(self, '_source', None)
        if s is None or not s.is_open:
            return {}
        mode = self._params.get('playback', 'every_tick')
        if mode == 'once':
            cur_refresh = int(self._params.get('refresh', 0))
            if cur_refresh == self._last_refresh_seen:
                return {}
            self._last_refresh_seen = cur_refresh
        f = s.read()
        # Sources don't add overlays themselves — frame and preview are
        # the same ndarray. Sharing the reference is fine because the
        # downstream OutputNode does its own .copy() before imencode.
        return {'frame': f, 'preview': f} if f is not None else {}

    def get_state(self):
        s = getattr(self, '_source', None)
        return {
            'open':     bool(s and s.is_open),
            'playback': self._params.get('playback', 'every_tick'),
            'refresh':  int(self._params.get('refresh', 0)),
            'last_error': self._last_error,
        }
