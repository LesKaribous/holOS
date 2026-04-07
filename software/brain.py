"""
brain.py — Jetson brain orchestrator.

Wires up transport, services and strategy.
Handles hot-reload of match.py, connection watchdog, and fallback.

Used by:
  run_jetson.py  — real hardware (XBeeTransport)
  run_sim.py     — simulator    (VirtualTransport)
"""

import os
import sys
import time
import threading
import traceback
import importlib.util
from typing import Optional, Callable

from transport.base import Transport
from services.motion    import MotionService
from services.safety    import SafetyService
from services.vision    import VisionService
from services.chrono    import ChronoService
from services.occupancy import OccupancyService
from shared.occupancy   import OccupancyGrid
from shared.pathfinder  import Pathfinder


_STRATEGY_PATH = os.path.join(os.path.dirname(__file__), 'strategy', 'match.py')


class Brain:
    """
    Central orchestrator for the Jetson side.

    Usage:
        brain = Brain(transport)
        brain.start()               # starts background services
        brain.run_match()           # starts match strategy in a thread
        brain.stop()
    """

    def __init__(self, transport: Transport,
                 theta_offset_deg: float = 0.0,
                 occupancy_grid: OccupancyGrid = None):
        self._t = transport

        # Shared occupancy grid — passed in from run_sim.py so sim and hw share the same instance.
        # If not provided, create a fresh one (standalone usage, e.g. run_jetson.py).
        grid = occupancy_grid if occupancy_grid is not None else OccupancyGrid()

        # Path planner built on the shared grid
        pathfinder = Pathfinder(grid)

        # Services — safety must be created before motion (motion hooks into it)
        self.safety    = SafetyService(transport)
        self.motion    = MotionService(transport, theta_offset_deg=theta_offset_deg,
                                       pathfinder=pathfinder, safety=self.safety)
        self.vision    = VisionService(transport)
        self.chrono    = ChronoService(transport)
        self.occupancy = OccupancyService(transport, grid)

        # Strategy module (hot-reloadable)
        self._strategy_module = None
        self._strategy_thread: Optional[threading.Thread] = None

        # Log buffer (for web UI)
        self._log_buffer: list[str] = []
        self._log_lock   = threading.Lock()

        # Connection state
        self._t.on_connect(self._on_connected)
        self._t.on_disconnect(self._on_disconnected)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Connect transport and start background services."""
        ok = self._t.connect()
        if ok:
            self._start_watchdog()
        return ok

    def stop(self) -> None:
        """Gracefully stop the brain."""
        self.motion.cancel()
        self._t.disconnect()

    # ── Strategy ──────────────────────────────────────────────────────────────

    def load_strategy(self) -> bool:
        """Hot-load (or reload) strategy/match.py."""
        try:
            spec = importlib.util.spec_from_file_location('match', _STRATEGY_PATH)
            mod  = importlib.util.module_from_spec(spec)

            # Inject services into module namespace
            mod.motion    = self.motion
            mod.vision    = self.vision
            mod.safety    = self.safety
            mod.chrono    = self.chrono
            mod.occupancy = self.occupancy
            mod.log       = self.log

            spec.loader.exec_module(mod)
            self._strategy_module = mod
            self.log("strategy/match.py loaded ✓")
            return True
        except Exception:
            self.log(f"strategy/match.py load error:\n{traceback.format_exc()}")
            return False

    def run_match(self) -> None:
        """Start match strategy in a background thread."""
        if self._strategy_thread and self._strategy_thread.is_alive():
            self.log("Strategy already running")
            return
        if not self.load_strategy():
            return
        self._strategy_thread = threading.Thread(
            target=self._strategy_entry, daemon=True, name="strategy"
        )
        self._strategy_thread.start()

    def stop_match(self) -> None:
        """Cancel current motion (strategy thread will exit when move fails)."""
        self.motion.cancel()
        self.log("Match stopped")

    def is_strategy_running(self) -> bool:
        return bool(self._strategy_thread and self._strategy_thread.is_alive())

    def _strategy_entry(self) -> None:
        if self._strategy_module is None:
            return
        try:
            self._strategy_module.run_mission()
        except Exception:
            self.log(f"Strategy error:\n{traceback.format_exc()}")

    # ── Hot-reload watcher ────────────────────────────────────────────────────

    def start_hot_reload(self, on_reload: Optional[Callable[[], None]] = None) -> None:
        """Watch strategy/match.py for changes and reload automatically."""
        _brain = self  # capture Brain instance for the inner class closure
        def _watch():
            try:
                from watchdog.observers import Observer
                from watchdog.events import FileSystemEventHandler

                class _Handler(FileSystemEventHandler):
                    _last = 0.0
                    def on_modified(self, event):
                        if 'match.py' not in str(event.src_path):
                            return
                        now = time.time()
                        if now - self._last < 0.5:
                            return
                        self._last = now
                        time.sleep(0.1)
                        if _brain.load_strategy():
                            if on_reload:
                                on_reload()

                obs = Observer()
                obs.schedule(_Handler(), path=os.path.dirname(_STRATEGY_PATH))
                obs.start()
                self.log("Hot-reload active (watchdog)")
            except ImportError:
                self.log("watchdog not installed — pip install watchdog for hot-reload")

        threading.Thread(target=_watch, daemon=True, name="hot-reload").start()

    # ── Connection events ─────────────────────────────────────────────────────

    def _on_connected(self) -> None:
        self.log("Jetson connected to Teensy ✓")

    def _on_disconnected(self) -> None:
        self.log("Jetson disconnected from Teensy !")

    # ── Connection watchdog ───────────────────────────────────────────────────

    def _start_watchdog(self) -> None:
        def _watch():
            while True:
                time.sleep(1.0)
                if not self._t.is_connected:
                    self.log("Transport disconnected — attempting reconnect…")
                    self._t.connect()
        threading.Thread(target=_watch, daemon=True, name="watchdog").start()

    # ── Logging ───────────────────────────────────────────────────────────────

    def log(self, msg: str) -> None:
        ts = f"{self.chrono.time_elapsed_s():.1f}s"
        entry = f"[{ts}] {msg}"
        with self._log_lock:
            self._log_buffer.append(entry)
            if len(self._log_buffer) > 200:
                self._log_buffer.pop(0)
        print(entry)

    def get_log(self, n: int = 30) -> list[str]:
        with self._log_lock:
            return self._log_buffer[-n:]
