"""
Source nodes — pull frames from the in-process FrameSource singleton.

  source/camera   → frame   (V4L2 device — owned by FrameSource)
  source/video    → frame   (local file — owned by FrameSource)
  source/image    → frame   (still image)

Every node simply snapshots `FrameSource.read_latest()` on each pipeline
tick. There is no per-node cv2.VideoCapture, no drainer thread, no FFmpeg
buffer. The FrameSource owns one reader thread at hardware rate; all
consumers (these nodes, the recorder) sample the same 1-slot buffer.

Pre-refactor history: this file used to open its own cv2.VideoCapture on
http://127.0.0.1:5174/stream.mjpg (the legacy vision_camera subprocess).
That MJPEG-over-HTTP step had an FFmpeg input buffer that returned frames
FIFO, so the pipeline lagged behind real time even though the drainer ran
at 60 Hz. Moving the source in-process eliminates the buffer entirely.

Playback controls (pause/seek/step for video) are forwarded to the
FrameSource via the params channel — the dashboard tile buttons still
work, they just talk to the shared source.
"""

from __future__ import annotations

import os
import sys
from typing import Optional

_HERE = os.path.dirname(os.path.abspath(__file__))

# The FrameSource singleton lives at software/vision_source.py — three
# levels up from this file (nodes/ → vision_pipelines/ → services/ →
# software/). Make sure it's importable without depending on the way
# run.py was launched.
_SOFTWARE = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
if _SOFTWARE not in sys.path:
    sys.path.insert(0, _SOFTWARE)

try:
    import cv2
    _CV2_OK = True
except Exception as _e:
    _CV2_OK = False
    print(f'[vision.nodes.source] cv2 unavailable: {_e}')

try:
    import vision_source as _vs
    _VS_OK = True
except Exception as _e:
    _VS_OK = False
    print(f'[vision.nodes.source] vision_source import failed: {_e}')

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


# ─── Helpers ───────────────────────────────────────────────────────────────

def _latest_frame() -> Optional[object]:
    """Snapshot the FrameSource singleton's latest BGR frame, or None
    when the source isn't configured / hasn't produced a frame yet."""
    if not _VS_OK:
        return None
    src = _vs.get()
    if src is None:
        return None
    return src.read_latest()


# ─── source.camera ─────────────────────────────────────────────────────────

@register_node('source.camera')
class CameraSourceNode(Node):
    """V4L2 camera source. Wraps the shared FrameSource: a pipeline tick
    just snapshots the latest BGR slot — no per-node capture, no buffer.

    The actual /dev/videoN device + GStreamer pipeline lives in
    `vision_source.FrameSource`. This node only exists so the pipeline
    editor can wire something to its `frame`/`preview` ports."""

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame (clean, for downstream processing)'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        # Kept for backward-compat with saved pipelines; FrameSource now
        # owns the actual device path. This param is informational only —
        # change the device by editing vision_config.json (source_path).
        'camera_index': {
            'type': 'int', 'default': 0,
            'label': 'camera index (informational)',
            'description': 'Actual device path lives in vision_config.json '
                           '(source_path). This value is no longer used at '
                           'runtime.',
            'min': 0, 'max': 9,
        },
    }

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
        if not _VS_OK:
            self._last_error = 'vision_source unavailable'

    def shutdown(self):
        # Nothing to release — FrameSource is owned by run.py.
        pass

    def process(self, inputs):
        f = _latest_frame()
        if f is None:
            return {}
        return {'frame': f, 'preview': f}

    def get_state(self):
        src = _vs.get() if _VS_OK else None
        if src is None:
            return {'open': False, 'last_error': self._last_error or 'no FrameSource'}
        st = src.status()
        return {
            'open':       st.get('has_frame', False),
            'source':     f"{st.get('source_kind')}: {st.get('source_path')}",
            'frame_age_s': st.get('frame_age_s'),
            'last_error': st.get('last_error') or self._last_error,
        }


# ─── source.video ──────────────────────────────────────────────────────────

@register_node('source.video')
class VideoSourceNode(Node):
    """Video-file source — same single-slot pattern as the camera node.
    Playback (play/pause/step/seek/speed) is forwarded to the underlying
    FrameSource so the dashboard tile buttons still work."""

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame (clean, for downstream processing)'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        'path': {
            'type': 'file_picker', 'kind': 'video', 'default': '',
            'label': 'file (informational)',
            'description': 'Actual file path lives in vision_config.json '
                           '(source_path). This value is no longer used at '
                           'runtime — change it via the Cam panel.',
        },
        'playback': {
            'type': 'str', 'default': 'play',
            'label': 'playback',
            'enum': ['live', 'play', 'pause', 'step', 'step_back', 'seek'],
            'description': 'Forwarded to the shared FrameSource.',
        },
        'seek_target': {
            'type': 'int',  'default': 0,
            'label': 'seek target',
            'unit': 'frame',
            'min': 0,
        },
        'speed': {
            'type': 'float', 'default': 1.0,
            'label': 'speed',
            'unit': '×',
            'min': 0.25, 'max': 8, 'step': 0.25,
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._last_playback: str = ''
        self._last_seek: int = -1
        self._last_speed: float = -1.0

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
        if not _VS_OK:
            self._last_error = 'vision_source unavailable'

    def shutdown(self):
        pass

    def _sync_to_frame_source(self) -> None:
        """Push any param changes through to the underlying FrameSource.
        Cheap to call every tick — only writes when something changed."""
        if not _VS_OK:
            return
        src = _vs.get()
        if src is None:
            return
        pb = str(self._params.get('playback', 'play'))
        if pb != self._last_playback:
            src.set_playback(pb)
            self._last_playback = pb
        st = int(self._params.get('seek_target', 0) or 0)
        if st != self._last_seek:
            src.set_seek_target(st)
            self._last_seek = st
        sp = float(self._params.get('speed', 1.0) or 1.0)
        if sp != self._last_speed:
            src.set_speed(sp)
            self._last_speed = sp

    def process(self, inputs):
        self._sync_to_frame_source()
        f = _latest_frame()
        if f is None:
            return {}
        return {'frame': f, 'preview': f}

    def get_state(self):
        src = _vs.get() if _VS_OK else None
        if src is None:
            return {'open': False, 'last_error': self._last_error or 'no FrameSource'}
        st = src.status()
        return {
            'open':           st.get('has_frame', False),
            'source':         f"{st.get('source_kind')}: {st.get('source_path')}",
            'frame_idx':      st.get('frame_idx', 0),
            'frame_count':    st.get('frame_count', -1),
            'playback':       st.get('playback', 'play'),
            'speed':          st.get('speed', 1.0),
            'frame_age_s':    st.get('frame_age_s'),
            'last_error':     st.get('last_error') or self._last_error,
        }


# ─── source.image ──────────────────────────────────────────────────────────

@register_node('source.image')
class ImageSourceNode(Node):
    """Static-image source — still loaded by FrameSource (which keeps
    one ndarray in its slot and never overwrites it)."""

    IO = NodeIO(outputs=[
        Port('frame',   PortKind.FRAME,   'BGR uint8 frame'),
        Port('preview', PortKind.PREVIEW, 'same frame, intended for display nodes'),
    ])
    params_schema = {
        'path': {
            'type': 'file_picker', 'kind': 'image', 'default': '',
            'label': 'image (informational)',
            'description': 'Actual file path lives in vision_config.json.',
        },
    }

    def start(self):
        if not _VS_OK:
            self._last_error = 'vision_source unavailable'

    def shutdown(self):
        pass

    def process(self, inputs):
        f = _latest_frame()
        if f is None:
            return {}
        return {'frame': f, 'preview': f}

    def get_state(self):
        src = _vs.get() if _VS_OK else None
        if src is None:
            return {'open': False, 'last_error': self._last_error or 'no FrameSource'}
        st = src.status()
        return {
            'open':       st.get('has_frame', False),
            'last_error': st.get('last_error') or self._last_error,
        }
