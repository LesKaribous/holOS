"""
transport/virtual.py — Virtual transport for simulator mode (Windows dev).

Instead of XBee serial, commands go directly to the SimBridge which runs the
physics simulation. Strategy code is 100% identical between sim and real modes.

The SimBridge is set externally (by run_sim.py) after both objects are created.
"""

import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from .base import Transport


class VirtualTransport(Transport):
    """
    Virtual transport for simulator.

    Commands are forwarded to a SimBridge instance that executes them on the
    physics simulation. Telemetry is injected by the SimBridge.

    This transport is always "connected" when a bridge is attached.
    """

    def __init__(self):
        self._bridge = None          # Set by run_sim.py after init
        self._connected_flag = False

        # Pending commands: uid → (event, result_container)
        self._lock   = threading.Lock()
        self._uid_ctr = 0
        self._pending: Dict[int, threading.Event] = {}
        self._results: Dict[int, Tuple[bool, str]] = {}

        # Telemetry subscribers
        self._tel_subs: Dict[str, List[Callable[[str], None]]] = {}

    # ── Bridge attachment ─────────────────────────────────────────────────────

    def attach_bridge(self, bridge) -> None:
        """Called by run_sim.py to wire up the physics bridge."""
        self._bridge = bridge
        self._connected_flag = True
        if self._on_connect:
            self._on_connect()

    # ── Connection lifecycle ──────────────────────────────────────────────────

    def connect(self) -> bool:
        """In virtual mode, connect succeeds immediately if bridge is attached."""
        if self._bridge is not None:
            self._connected_flag = True
            if self._on_connect:
                self._on_connect()
            return True
        return False

    def disconnect(self) -> None:
        self._connected_flag = False
        if self._on_disconnect:
            self._on_disconnect()

    @property
    def is_connected(self) -> bool:
        return self._connected_flag and self._bridge is not None

    # ── Command execution ─────────────────────────────────────────────────────

    def execute(self, cmd: str, timeout_ms: int = 5000) -> Tuple[bool, str]:
        """
        Forward command to SimBridge and block until it completes.
        The SimBridge calls _resolve() when done.
        """
        if self._bridge is None:
            return (False, "no_bridge")

        with self._lock:
            uid = self._uid_ctr
            self._uid_ctr += 1
            evt = threading.Event()
            self._pending[uid] = evt

        # Dispatch to bridge (non-blocking from our side)
        self._bridge.dispatch(uid, cmd, self._resolve)

        # Block strategy thread until bridge signals completion
        ok = evt.wait(timeout=timeout_ms / 1000.0)
        with self._lock:
            result = self._results.pop(uid, (False, "timeout"))
            self._pending.pop(uid, None)

        if not ok:
            return (False, "timeout")
        return result

    def _resolve(self, uid: int, success: bool, response: str = "ok") -> None:
        """Called by SimBridge from physics thread to signal command completion."""
        with self._lock:
            self._results[uid] = (success, response)
            if uid in self._pending:
                self._pending[uid].set()

    def fire(self, cmd: str) -> None:
        """Fire-and-forget to bridge."""
        if self._bridge:
            self._bridge.dispatch_fire(cmd)

    # ── Telemetry subscription ────────────────────────────────────────────────

    def subscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        self._tel_subs.setdefault(ttype, []).append(callback)

    def unsubscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        if ttype in self._tel_subs:
            try:
                self._tel_subs[ttype].remove(callback)
            except ValueError:
                pass

    def inject_telemetry(self, ttype: str, data: str) -> None:
        """Called by SimBridge to push telemetry to subscribers."""
        for cb in self._tel_subs.get(ttype, []):
            try:
                cb(data)
            except Exception as e:
                print(f"[VirtualTransport] Telemetry error ({ttype}): {e}")

