"""
Pipeline — a directed-acyclic graph of vision Nodes that processes one
frame at a time.

Each pipeline owns its own worker thread and shared state. A pipeline can:
  - have multiple Source nodes (e.g. two cameras compared side-by-side)
  - branch — one source feeds both an undistorted view AND a raw ArUco view
  - register any Node's output as a "feed" exposed to the dashboard

A `Node` is identified by `id` (string), has typed inputs and outputs, and
a per-node `params` dict. Connections are made by referring to a peer node's
output port: `inputs = { port_name: (peer_node_id, peer_port_name) }`.

This module is intentionally small — node implementations live in
`services.vision.nodes.*`. The Pipeline just schedules them in topological
order each frame and ferries values between connected ports.
"""

from __future__ import annotations

import collections
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Optional


# ── Port type system ────────────────────────────────────────────────────────
# Connections are only allowed between matching kinds.

class PortKind:
    FRAME       = 'frame'         # numpy ndarray, BGR uint8 — clean, no overlays
    PREVIEW     = 'preview'       # numpy ndarray, BGR uint8 — frame WITH annotations
                                   # (markers / grid / debug labels). Only valid as
                                   # input on display nodes — processing nodes
                                   # (undistort / rectify / aruco / preprocess)
                                   # take FRAME so overlays don't compound.
    DETECTION   = 'detection'     # core.aruco_detector.DetectionResult
    POSE_LIST   = 'pose_list'     # list[dict] {tag_id, x_mm, y_mm, theta_rad}
    MASK        = 'mask'          # numpy ndarray, uint8 single-channel
    JSON        = 'json'          # any JSON-serializable Python value
    POSE        = 'pose'          # single dict {x_mm, y_mm, theta_rad}


@dataclass
class Port:
    name: str
    kind: str           # one of PortKind.*
    description: str = ''
    optional: bool = False   # not required for the node to produce output


@dataclass
class NodeIO:
    """Static description of a node's I/O — used by the editor UI to know
    which ports exist and which connections are legal."""
    inputs:  list[Port] = field(default_factory=list)
    outputs: list[Port] = field(default_factory=list)


# ── Pipeline machinery ─────────────────────────────────────────────────────

@dataclass
class _NodeRecord:
    """Internal: a node instance + its connection map."""
    id: str
    kind: str
    instance: Any                 # subclass of services.vision.nodes.Node
    inputs:  dict[str, tuple[str, str]] = field(default_factory=dict)
    # latest output values (port_name → value), refreshed each tick.
    outputs: dict[str, Any] = field(default_factory=dict)
    # Diagnostics: did this node produce a non-empty output on the last
    # processed tick? (Surfaced via state() so the dashboard can color it.)
    produced_last_tick: bool = False
    # And: did all of its connected upstream inputs deliver values?
    inputs_satisfied: bool = False
    # Total ticks where this node was invoked.
    tick_count: int = 0
    # Editor layout: pixel coords inside the React Flow canvas. Persisted
    # in to_dict so the graph re-opens with the same layout.
    position: Optional[dict] = None


class Pipeline:
    """A named pipeline. One worker thread, processes at fps_limit Hz.

    Use `Pipeline.from_dict(d)` to load a saved graph; `to_dict()` to save.
    """

    def __init__(self, name: str, fps_limit: int = 25):
        self.name = name
        self.fps_limit = max(1, int(fps_limit))
        self._nodes: dict[str, _NodeRecord] = {}
        self._order: list[str] = []        # topological order, recomputed on edits
        self._lock = threading.RLock()
        self._enabled = False
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # Single-shot lock used to defeat the double-spawn race in enable():
        # whoever acquires it first creates the worker thread; everyone else
        # sees `_thread.is_alive()` and bails.
        self._enable_lock = threading.Lock()
        # Diagnostics, surfaced via state()
        self._frames_processed = 0
        self._last_frame_t = 0.0
        self._last_error: Optional[str] = None
        self._worker_status = 'created'
        # Bounded log ring buffer (deque cheap to append + auto-truncate).
        # The dashboard polls these via state(); also useful when the
        # terminal dies before we can read its stdout.
        self._log_buffer = collections.deque(maxlen=200)
        # Subscribers for dashboard feeds: feed_id → callback(jpeg_bytes, meta)
        # Set by the backend; nodes call self._publish_feed(feed_id, jpeg, meta)
        self._feed_subscribers: dict[str, Callable] = {}
        # Per-feed rate-limiting (avoids flooding SocketIO when the worker
        # produces feeds faster than the network / browser can consume).
        # feed_id → last-emit monotonic timestamp.
        self._feed_last_emit: dict[str, float] = {}
        self._feed_min_period_s: float = 0.05   # 20 Hz cap per feed
        self._feed_drop_count:   dict[str, int] = {}  # diagnostics: dropped frames

    def _log(self, level: str, msg: str) -> None:
        """Append a line to the pipeline's log ring buffer + stdout.
        Only ERRORS (and INFOs that the user explicitly asks for) are
        printed to stdout — info-level lines stay in the in-memory ring
        for the dashboard, no console flood."""
        entry = {'t': time.time(), 'level': level, 'msg': msg}
        with self._lock:
            self._log_buffer.append(entry)   # deque trims automatically
        if level in ('error', 'warn'):
            try:
                print(f'[pipeline:{self.name}] [{level}] {msg}', flush=True)
            except Exception:
                pass

    def get_logs(self, n: int = 50) -> list[dict]:
        # NB: collections.deque doesn't support slicing — turn it into a
        # list first, then slice the tail.
        with self._lock:
            buf = list(self._log_buffer)
        return buf[-n:] if n > 0 else buf

    # ── Node management ────────────────────────────────────────────────

    def add_node(self, node_id: str, kind: str, instance,
                 inputs: Optional[dict[str, tuple[str, str]]] = None) -> None:
        with self._lock:
            self._nodes[node_id] = _NodeRecord(
                id=node_id, kind=kind, instance=instance,
                inputs=dict(inputs) if inputs else {},
            )
            instance.attach(self, node_id)
            self._reorder()

    def remove_node(self, node_id: str) -> None:
        with self._lock:
            self._nodes.pop(node_id, None)
            for rec in self._nodes.values():
                rec.inputs = {k: v for k, v in rec.inputs.items()
                              if v[0] != node_id}
            self._reorder()

    def connect(self, src_node_id: str, src_port: str,
                dst_node_id: str, dst_port: str) -> None:
        with self._lock:
            if dst_node_id not in self._nodes:
                raise KeyError(f'unknown destination node {dst_node_id!r}')
            if src_node_id not in self._nodes:
                raise KeyError(f'unknown source node {src_node_id!r}')
            self._nodes[dst_node_id].inputs[dst_port] = (src_node_id, src_port)
            self._reorder()

    def disconnect(self, dst_node_id: str, dst_port: str) -> None:
        with self._lock:
            if dst_node_id in self._nodes:
                self._nodes[dst_node_id].inputs.pop(dst_port, None)

    def get_node(self, node_id: str):
        rec = self._nodes.get(node_id)
        return rec.instance if rec else None

    def node_ids(self) -> list[str]:
        return list(self._nodes.keys())

    # ── Topological ordering ───────────────────────────────────────────

    def _reorder(self):
        """Recompute the topological order. Raises ValueError if a cycle
        exists; that's the caller's bug to fix."""
        # Kahn's algorithm — O(V + E), iterative, no recursion limit risk
        # on big graphs. Edges go src → dst (an input pulls from src node).
        in_degree: dict[str, int] = {nid: 0 for nid in self._nodes}
        adj: dict[str, list[str]] = {nid: [] for nid in self._nodes}
        for nid, rec in self._nodes.items():
            for src_nid, _src_port in rec.inputs.values():
                if src_nid not in self._nodes:
                    continue   # dangling edge (referent node was deleted)
                adj[src_nid].append(nid)
                in_degree[nid] += 1
        ready = collections.deque(
            nid for nid, d in in_degree.items() if d == 0
        )
        order: list[str] = []
        while ready:
            nid = ready.popleft()
            order.append(nid)
            for dst in adj[nid]:
                in_degree[dst] -= 1
                if in_degree[dst] == 0:
                    ready.append(dst)
        if len(order) != len(self._nodes):
            # Cycle detected — leave _order untouched and report.
            raise ValueError(
                f'pipeline {self.name!r} has a cycle (topological sort '
                f'visited {len(order)}/{len(self._nodes)} nodes)'
            )
        self._order = order

    # ── Lifecycle ──────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        # Use a dedicated lock so two callers can't both pass the
        # is_alive() check and end up creating two worker threads (which
        # would race on the same node state and almost certainly segfault
        # cv2). Whoever enters the critical section first creates the
        # thread; everyone else falls through.
        with self._enable_lock:
            if self._enabled and self._thread is not None and self._thread.is_alive():
                return
            self._enabled = True
            self._stop_evt.clear()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(
                    target=self._run_loop, daemon=True,
                    name=f'pipeline-{self.name}',
                )
                self._thread.start()

    def disable(self) -> None:
        # Don't tear down — just stop processing. The worker thread will
        # park itself in `if not self._enabled: time.sleep` until either
        # enable() comes back or shutdown() flips _stop_evt.
        self._enabled = False

    def shutdown(self) -> None:
        """Stop the worker, join it, release node-owned resources, and
        clear feed subscribers. Safe to call repeatedly."""
        self._stop_evt.set()
        self._enabled = False
        # Wait for the worker to actually exit before tearing down nodes.
        # Without this, node.shutdown() could race with the worker still
        # inside node.process() — which in cv2-land tends to segfault.
        t = self._thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=2.0)
            except Exception:
                pass
        # Release per-node resources (cv2 captures, etc.)
        with self._lock:
            for rec in list(self._nodes.values()):
                try:
                    rec.instance.shutdown()
                except Exception:
                    pass
            # Drop subscribers so old SocketIO callbacks (which capture
            # the previous Pipeline by closure) don't keep this instance
            # alive after the registry replaces it.
            self._feed_subscribers.clear()
            self._feed_last_emit.clear()

    # ── Worker loop ────────────────────────────────────────────────────

    def _run_loop(self):
        # Initialize each node once. Capture init failures so the user can
        # see which node broke.
        for nid in self._order:
            rec = self._nodes.get(nid)
            if rec is None:
                continue
            try:
                rec.instance.start()
            except Exception as e:
                self._last_error = f'{nid} start: {e}'
                self._worker_status = 'init failed'
                print(f'[pipeline:{self.name}] init {nid} failed: {e}')
                traceback.print_exc()
                return

        last_step_t = 0.0
        while not self._stop_evt.is_set():
            try:
                if not self._enabled:
                    self._worker_status = 'disabled'
                    time.sleep(0.05)
                    continue

                now = time.monotonic()
                if (now - last_step_t) < (1.0 / self.fps_limit):
                    time.sleep(0.004)
                    continue
                last_step_t = now

                # Snapshot order under lock, but evaluate without holding it
                with self._lock:
                    order_snapshot = list(self._order)
                    nodes_snapshot = {k: v for k, v in self._nodes.items()}

                # Tick: evaluate each node in topological order, ferrying
                # outputs to inputs of downstream nodes.
                produced_anything = False
                for nid in order_snapshot:
                    rec = nodes_snapshot.get(nid)
                    if rec is None:
                        continue
                    inp_values = {}
                    # Build a name → optional? map from the node class so
                    # we can ignore optional ports when computing whether
                    # the node has 'enough' inputs to run.
                    io = getattr(rec.instance.__class__, 'IO', None)
                    optional_ports = {
                        p.name for p in (io.inputs if io else [])
                        if getattr(p, 'optional', False)
                    }
                    all_required_have_values = True
                    for port, (src_nid, src_port) in rec.inputs.items():
                        src_rec = nodes_snapshot.get(src_nid)
                        v = src_rec.outputs.get(src_port) if src_rec else None
                        inp_values[port] = v
                        if v is None and port not in optional_ports:
                            all_required_have_values = False
                    rec.inputs_satisfied = all_required_have_values
                    rec.tick_count += 1
                    try:
                        outs = rec.instance.process(inp_values) or {}
                    except Exception as e:
                        self._last_error = f'{nid} process: {e}'
                        self._worker_status = 'frame error (recovering)'
                        self._log('error', f'{nid} process: {type(e).__name__}: {e}')
                        traceback.print_exc()
                        outs = {}
                    # Atomic publish: readers in other threads (localization
                    # driver, snapshot endpoint, JS state poll) all snapshot
                    # rec.outputs under self._lock, so we mutate under the
                    # same lock. Without this the dict can be observed
                    # half-written and a cv2.Mat reference can be aliased
                    # while the upstream thread is mid-encode → segfault.
                    with self._lock:
                        rec.outputs = outs
                        rec.produced_last_tick = bool(outs)
                    if outs:
                        produced_anything = True

                if produced_anything:
                    self._frames_processed += 1
                    self._last_frame_t = time.monotonic()
                    self._worker_status = 'running'
                    # Heartbeat in-memory only (dashboard log strip),
                    # no console spam. Reduced frequency: every ~30s
                    # at 25 fps = every 750 frames.
                    if self._frames_processed % 750 == 0:
                        entry = {'t': time.time(), 'level': 'info',
                                 'msg': f'heartbeat — {self._frames_processed} frames'}
                        with self._lock:
                            self._log_buffer.append(entry)
            except Exception as e:
                self._last_error = f'loop: {e}'
                self._worker_status = 'loop error (recovering)'
                self._log('error', f'loop: {type(e).__name__}: {e}')
                traceback.print_exc()
                time.sleep(0.1)

    # ── Feed publishing ────────────────────────────────────────────────

    def subscribe_feed(self, feed_id: str, callback: Callable) -> None:
        """Register a callback `cb(jpeg_bytes, meta_dict)` for any node
        that publishes under `feed_id`. Used by the backend to multiplex
        SocketIO emissions."""
        with self._lock:
            self._feed_subscribers[feed_id] = callback

    def clear_feed_subscribers(self) -> None:
        """Drop all feed subscribers. Used by the backend before re-wiring
        after a node param change (e.g. a feed_id rename)."""
        with self._lock:
            self._feed_subscribers.clear()
            self._feed_last_emit.clear()

    def _publish_feed(self, feed_id: str, jpeg_bytes: bytes, meta: dict):
        # Take the lock once for the read/check/write of rate-limit state
        # so two pipeline calls can't both decide they're under quota.
        with self._lock:
            cb = self._feed_subscribers.get(feed_id)
            if cb is None:
                return
            now = time.monotonic()
            last = self._feed_last_emit.get(feed_id, 0.0)
            if (now - last) < self._feed_min_period_s:
                self._feed_drop_count[feed_id] = (
                    self._feed_drop_count.get(feed_id, 0) + 1
                )
                return
            self._feed_last_emit[feed_id] = now
        # Call the SocketIO emit callback OUTSIDE the lock so a slow
        # network doesn't stall the worker thread. The callback itself
        # is expected to be quick + non-blocking (socketio.emit + base64).
        try:
            cb(jpeg_bytes, meta)
        except Exception as e:
            self._log('error', f'feed {feed_id} callback: {type(e).__name__}: {e}')

    # ── Persistence + state ────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            'name':      self.name,
            'fps_limit': self.fps_limit,
            'nodes': [
                {
                    'id':     rec.id,
                    'kind':   rec.kind,
                    'params': rec.instance.get_params(),
                    'inputs': [
                        {'port': p, 'src_node': src_nid, 'src_port': src_port}
                        for p, (src_nid, src_port) in rec.inputs.items()
                    ],
                    'position': rec.position,   # pixel coords for the editor
                }
                for rec in self._nodes.values()
            ],
        }

    def state(self) -> dict:
        return {
            'name':             self.name,
            'enabled':          self._enabled,
            'fps_limit':        self.fps_limit,
            'frames_processed': self._frames_processed,
            'logs':             self.get_logs(50),
            'feed_drops':       dict(self._feed_drop_count),
            'last_frame_age_s': (
                (time.monotonic() - self._last_frame_t)
                if self._last_frame_t else None
            ),
            'last_error':       self._last_error,
            'worker_status':    self._worker_status,
            'nodes': [
                {
                    'id':       rec.id,
                    'kind':     rec.kind,
                    'state':    rec.instance.get_state(),
                    # Per-node liveness flags surfaced to the dashboard so
                    # it can color each node correctly (running / idle /
                    # missing input / errored).
                    # Inputs declared by the node *class* (independent of
                    # wiring) — needed so the dashboard can tell apart:
                    #   declared but no edge   → "input not wired" (red)
                    #   wired but None value   → "no upstream data" (amber)
                    #   wired + value          → running (green)
                    'declared_inputs':    [p.name for p in
                                           getattr(getattr(rec.instance, 'IO', None),
                                                   'inputs', []) or []],
                    'wired_inputs':       list(rec.inputs.keys()),
                    'inputs_satisfied':   rec.inputs_satisfied,
                    'produced_last_tick': rec.produced_last_tick,
                    'tick_count':         rec.tick_count,
                    'output_ports':       list(rec.outputs.keys()),
                }
                for rec in self._nodes.values()
            ],
        }


# ── Pipeline registry ─────────────────────────────────────────────────────


class PipelineRegistry:
    """Holds N named pipelines.  ANY NUMBER may be enabled (running) at a
    time — the dashboard's localization view drives one pipeline, the
    detection view drives another, both run concurrently in their own
    worker threads.

    `active_name` is kept for back-compat with single-pipeline UIs but no
    longer enforces single-active. Use `set_enabled(name, True/False)` to
    drive individual pipelines.

    Also tracks a "default" pipeline name that gets auto-enabled at
    server startup.
    """

    def __init__(self):
        self._pipelines:    dict[str, Pipeline] = {}
        self._lock          = threading.RLock()
        self._active_name:  Optional[str] = None
        self._default_name: Optional[str] = None

    def register(self, pipeline: Pipeline) -> None:
        """Add or replace a pipeline. If we replace the currently-active
        pipeline, the old worker is stopped; the new one stays disabled
        until set_active() is called explicitly."""
        with self._lock:
            old = self._pipelines.get(pipeline.name)
            was_active = (self._active_name == pipeline.name)
            if old is not None:
                old.shutdown()
            self._pipelines[pipeline.name] = pipeline
            # Replacing the active pipeline → drop the active flag so the
            # caller has to opt-in again. This is what makes "save" non-
            # disruptive: editing the active pipeline doesn't auto-restart it.
            if was_active:
                self._active_name = None

    def get(self, name: str) -> Optional[Pipeline]:
        return self._pipelines.get(name)

    def names(self) -> list[str]:
        return list(self._pipelines.keys())

    def all(self) -> list[Pipeline]:
        return list(self._pipelines.values())

    @property
    def active_name(self) -> Optional[str]:
        return self._active_name

    @property
    def default_name(self) -> Optional[str]:
        return self._default_name

    def active(self) -> Optional[Pipeline]:
        return self._pipelines.get(self._active_name) if self._active_name else None

    def set_active(self, name: Optional[str]) -> Optional[Pipeline]:
        """Mark `name` as the focused pipeline + ensure it's running.
        Does NOT disable other pipelines (they keep running). Pass None
        to clear the focus only — running pipelines stay running."""
        with self._lock:
            if name and name not in self._pipelines:
                raise KeyError(f'unknown pipeline {name!r}')
            self._active_name = name
            if name:
                p = self._pipelines[name]
                p.enable()
                return p
            return None

    def set_enabled(self, name: str, enabled: bool) -> bool:
        """Enable / disable a single named pipeline without touching the
        others. Returns True if found, False if no such pipeline."""
        with self._lock:
            p = self._pipelines.get(name)
            if p is None:
                return False
        # Outside the registry lock — Pipeline.enable() / disable()
        # take their own internal locks.
        if enabled:
            p.enable()
        else:
            p.disable()
        return True

    def enabled_names(self) -> list[str]:
        """All currently-enabled pipeline names (any number)."""
        with self._lock:
            return [n for n, p in self._pipelines.items() if p.enabled]

    def set_default(self, name: Optional[str]) -> None:
        with self._lock:
            if name and name not in self._pipelines:
                raise KeyError(f'unknown pipeline {name!r}')
            self._default_name = name

    def remove(self, name: str) -> bool:
        """Atomic delete: drops the named pipeline + clears active/default
        flags if they pointed at it. Returns True if anything was deleted."""
        with self._lock:
            p = self._pipelines.pop(name, None)
            if p is None:
                return False
            if self._active_name == name:
                self._active_name = None
            if self._default_name == name:
                self._default_name = None
        try:
            p.shutdown()
        except Exception:
            pass
        return True

    def shutdown_all(self) -> None:
        for p in list(self._pipelines.values()):
            try:
                p.shutdown()
            except Exception:
                pass
        with self._lock:
            self._pipelines.clear()
            self._active_name = None
            self._default_name = None
