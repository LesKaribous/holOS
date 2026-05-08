"""
output node — registers a frame as a dashboard feed. JPEG-encodes and
publishes via the pipeline's feed-subscriber callback. The dashboard
SocketIO push loop forwards it to subscribed clients.

Inputs:
    frame  — any FRAME port.
Params:
    feed_id (string)         — the stable id the dashboard refers to.
    label   (string)         — display label.
    jpeg_quality (int 30-95) — encoding quality.
    fps_limit (int)          — encoder rate; default 25.
"""

from __future__ import annotations

import time

try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


@register_node('output')
class OutputNode(Node):
    IO = NodeIO(
        inputs=[
            # Two optional ports — wire one or the other. Preview wins
            # when both are connected (it's the annotated version).
            Port('frame',   PortKind.FRAME,   optional=True,
                 description='clean frame'),
            Port('preview', PortKind.PREVIEW, optional=True,
                 description='annotated/overlay frame (takes precedence)'),
        ],
        outputs=[],
    )
    params_schema = {
        'feed_id':      {'type': 'str', 'default': '',
                         'help': 'dashboard tile id (defaults to node id when empty)'},
        'label':        {'type': 'str', 'default': ''},
        'jpeg_quality': {'type': 'int', 'default': 70, 'min': 30, 'max': 95},
        'fps_limit':    {'type': 'int', 'default': 25},
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._frames_emitted = 0
        self._last_emit_t    = 0.0

    def _effective_feed_id(self) -> str:
        """The feed_id used to publish — defaults to the node id if the
        param is empty so a freshly-added output node 'just works'
        without the user having to fill in feed_id by hand."""
        feed_id = str(self._params.get('feed_id', '')).strip()
        if feed_id:
            return feed_id
        return self._node_id or ''

    def process(self, inputs):
        # Preview wins — that's the annotated image the user wants to see.
        # Fall back to the clean frame when no preview is wired.
        frame = inputs.get('preview')
        if frame is None:
            frame = inputs.get('frame')
        if frame is None or not _CV2_OK:
            return {}
        feed_id = self._effective_feed_id()
        if not feed_id:
            return {}
        # Skip the entire JPEG path when no SocketIO client is connected —
        # imencode + base64 + emit add up fast on the Jetson and there's
        # nobody to consume the bytes anyway. Gate is set by the backend
        # (run.py wires it to the live client count).
        pipe = getattr(self, '_pipeline', None)
        if pipe is not None and not pipe.is_feed_active(feed_id):
            return {}
        # Rate-limit the encode (separate from the pipeline's own fps_limit
        # so output nodes can run slower than detection if needed).
        fps_lim = max(1, int(self._params.get('fps_limit', 25)))
        now = time.monotonic()
        if (now - self._last_emit_t) < (1.0 / fps_lim):
            return {}
        self._last_emit_t = now

        # cv2.imencode releases the GIL. If `frame` is a buffer shared with
        # an upstream cv2 op that's still running on another thread, the
        # memory can be reaped mid-encode and segfault the interpreter.
        # A copy is cheap relative to encode itself and guarantees we own
        # the bytes for the duration of the call.
        try:
            safe = frame.copy() if hasattr(frame, 'copy') else frame
            q = int(self._params.get('jpeg_quality', 70))
            ok, buf = cv2.imencode('.jpg', safe, [cv2.IMWRITE_JPEG_QUALITY, q])
        except Exception as e:
            self._last_error = f'imencode: {type(e).__name__}: {e}'
            return {}
        if not ok:
            self._last_error = 'imencode returned False'
            return {}
        jpeg_bytes = bytes(buf)
        meta = {
            'feed_id': feed_id,
            'label':   self._params.get('label') or feed_id,
            'shape':   list(safe.shape),
        }
        self._publish_feed(feed_id, jpeg_bytes, meta)
        self._frames_emitted += 1
        return {}

    def get_state(self):
        return {
            'feed_id':         self._params.get('feed_id', ''),
            'effective_feed':  self._effective_feed_id(),
            'frames_emitted':  self._frames_emitted,
            'last_error':      self._last_error,
        }
