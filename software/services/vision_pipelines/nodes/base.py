"""
Node base class + global registry.

A Node has:
  - static `IO`: a NodeIO describing its input + output ports (for the editor)
  - static `KIND`: short string identifier
  - per-instance `params`: dict of tunable knobs
  - `process(inputs)` → dict of output values, called once per pipeline tick
  - optional `start()` / `shutdown()` lifecycle hooks
"""

from __future__ import annotations

from typing import Any, Optional

from ..pipeline import NodeIO


# ── Registry ───────────────────────────────────────────────────────────────

NODE_KINDS: dict[str, type] = {}


def register_node(kind: str):
    """Decorator: register a Node subclass under the given kind string."""
    def _wrap(cls):
        cls.KIND = kind
        NODE_KINDS[kind] = cls
        return cls
    return _wrap


# ── Base class ─────────────────────────────────────────────────────────────

class Node:
    """Subclass and decorate with @register_node('kind').

    Override:
      IO         (class attribute, NodeIO) — describes ports to the editor
      params_schema (class attribute, dict) — describes tunables to the editor
      process(inputs) — called every pipeline tick; return {output_port: value}

    The pipeline ferries values from upstream `outputs[port]` to downstream
    `inputs[port]` based on the connection map. Missing inputs come through
    as `None` — your process() should be robust to that.
    """

    KIND: str = 'base'
    IO: NodeIO = NodeIO()
    params_schema: dict = {}        # informative only; UI uses it for forms

    def __init__(self, params: Optional[dict] = None):
        self._params: dict = dict(params or {})
        self._pipeline = None           # set by attach()
        self._node_id: Optional[str] = None
        self._last_error: Optional[str] = None

    # Lifecycle hooks ------------------------------------------------------

    def attach(self, pipeline, node_id: str) -> None:
        """Called when the node is added to a pipeline."""
        self._pipeline = pipeline
        self._node_id = node_id

    def start(self) -> None:
        """Called once when the pipeline worker thread begins."""
        pass

    def shutdown(self) -> None:
        """Called once when the pipeline shuts down."""
        pass

    # Per-tick -------------------------------------------------------------

    def process(self, inputs: dict[str, Any]) -> dict[str, Any]:
        """Override. Return a dict mapping output port name → value."""
        return {}

    # Param/state introspection -------------------------------------------

    def get_params(self) -> dict:
        return dict(self._params)

    def set_params(self, params: dict) -> None:
        self._params.update(params or {})

    def get_state(self) -> dict:
        """Override to add live state (frame count, last detection, …).
        Default: just last_error if any."""
        if self._last_error:
            return {'last_error': self._last_error}
        return {}

    # Helpers --------------------------------------------------------------

    def _publish_feed(self, feed_id: str, jpeg_bytes: bytes, meta: dict):
        """Send a JPEG feed up to the dashboard (if a subscriber exists)."""
        if self._pipeline is not None:
            self._pipeline._publish_feed(feed_id, jpeg_bytes, meta)
