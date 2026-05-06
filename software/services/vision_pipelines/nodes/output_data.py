"""
output_data — non-frame output nodes for the dashboard.

These mirror the JPEG-emitting OutputNode but carry structured data
(pose_list, generic JSON) over the same SocketIO `vision_feed` channel.
The dashboard inspects `meta.kind` to render the right tile body.

  output.pose_list  → small table of {tag_id, x_mm, y_mm, theta, class}
  output.json       → pretty-printed JSON (any value)
"""

from __future__ import annotations

import time

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


class _OutputDataBase(Node):
    """Common rate-limit + feed-id-fallback behavior shared by data outputs."""

    KIND_FOR_META = 'json'   # subclasses override

    def __init__(self, params=None):
        super().__init__(params)
        self._frames_emitted = 0
        self._last_emit_t    = 0.0

    def _effective_feed_id(self) -> str:
        feed_id = str(self._params.get('feed_id', '')).strip()
        return feed_id or (self._node_id or '')

    def _ratelimit(self) -> bool:
        """Returns True if we should emit this tick."""
        fps = max(1, int(self._params.get('fps_limit', 5)))
        now = time.monotonic()
        if (now - self._last_emit_t) < (1.0 / fps):
            return False
        self._last_emit_t = now
        return True

    def get_state(self):
        return {
            'feed_id':         self._params.get('feed_id', ''),
            'effective_feed':  self._effective_feed_id(),
            'frames_emitted':  self._frames_emitted,
            'last_error':      self._last_error,
        }


@register_node('output.pose_list')
class OutputPoseListNode(_OutputDataBase):
    """Emit a pose_list to the dashboard as a structured tile (a small
    table). Default fps_limit is intentionally low (5 Hz) — pose tables
    don't need to refresh at video rate and re-rendering them at 25 fps
    just hammers the browser's DOM."""

    IO = NodeIO(
        inputs=[Port('pose_list', PortKind.POSE_LIST)],
        outputs=[],
    )
    params_schema = {
        'feed_id':   {'type': 'str', 'default': '',
                      'help': 'dashboard tile id (defaults to node id)'},
        'label':     {'type': 'str', 'default': ''},
        'fps_limit': {'type': 'int', 'default': 5},
    }

    KIND_FOR_META = 'pose_list'

    def process(self, inputs):
        poses = inputs.get('pose_list')
        if poses is None:
            return {}
        feed_id = self._effective_feed_id()
        if not feed_id:
            return {}
        if not self._ratelimit():
            return {}
        meta = {
            'feed_id': feed_id,
            'kind':    'pose_list',
            'label':   self._params.get('label') or feed_id,
            'count':   len(poses) if isinstance(poses, (list, tuple)) else 0,
        }
        # Pass the list through as-is; the upstream localization node
        # already sanitized NaN → None so JSON.parse is safe in the browser.
        self._publish_feed(feed_id, list(poses) if poses else [], meta)
        self._frames_emitted += 1
        return {}


@register_node('output.objects')
class OutputObjectsNode(_OutputDataBase):
    """Emit a pose_list to the dashboard's Objects tab.

    Conventionally distinct from `output.pose_list`: this stream is
    consumed by the Vision · Objects subtab which keeps a persistent
    state per-tag-id (last-seen timestamp, fade-out, classification).
    Use this when you want the Objects tab to track everything the
    pipeline detected; use output.pose_list when you want a snapshot of
    the current frame as a tile."""

    IO = NodeIO(
        inputs=[Port('pose_list', PortKind.POSE_LIST)],
        outputs=[],
    )
    params_schema = {
        'feed_id':   {'type': 'str', 'default': '',
                      'help': 'feed id (defaults to node id)'},
        'label':     {'type': 'str', 'default': ''},
        'fps_limit': {'type': 'int', 'default': 5},
    }

    KIND_FOR_META = 'objects'

    def process(self, inputs):
        poses = inputs.get('pose_list')
        if poses is None:
            return {}
        feed_id = self._effective_feed_id()
        if not feed_id:
            return {}
        if not self._ratelimit():
            return {}
        meta = {
            'feed_id': feed_id,
            'kind':    'objects',
            'label':   self._params.get('label') or feed_id,
            'count':   len(poses) if isinstance(poses, (list, tuple)) else 0,
        }
        self._publish_feed(feed_id, list(poses) if poses else [], meta)
        self._frames_emitted += 1
        return {}


@register_node('output.json')
class OutputJsonNode(_OutputDataBase):
    """Emit any JSON-serializable value to the dashboard as a JSON tile.
    Useful for cam_xyz, homography_state, or any custom debug payload."""

    IO = NodeIO(
        inputs=[Port('value', PortKind.JSON)],
        outputs=[],
    )
    params_schema = {
        'feed_id':   {'type': 'str', 'default': '',
                      'help': 'dashboard tile id (defaults to node id)'},
        'label':     {'type': 'str', 'default': ''},
        'fps_limit': {'type': 'int', 'default': 5},
    }

    KIND_FOR_META = 'json'

    def process(self, inputs):
        value = inputs.get('value')
        if value is None:
            return {}
        feed_id = self._effective_feed_id()
        if not feed_id:
            return {}
        if not self._ratelimit():
            return {}
        meta = {
            'feed_id': feed_id,
            'kind':    'json',
            'label':   self._params.get('label') or feed_id,
        }
        # Defensive: only forward if it's already JSON-serializable.
        # Numpy scalars / arrays would explode the SocketIO encoder.
        try:
            import json as _json
            _json.dumps(value, default=str)
        except Exception as e:
            self._last_error = f'value not JSON-serializable: {type(e).__name__}'
            return {}
        self._publish_feed(feed_id, value, meta)
        self._frames_emitted += 1
        return {}
