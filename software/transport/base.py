"""
transport/base.py — Abstract transport interface between Jetson brain and Teensy.

Both XBeeTransport (real hardware) and VirtualTransport (simulator) implement
this interface, so strategy code is 100% identical in both modes.
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional, Tuple


class Transport(ABC):
    """
    Abstract transport layer between the Jetson brain and the Teensy.

    Usage pattern (in services):
        success, response = transport.execute("go(500,300)", timeout_ms=30000)
        transport.fire("cancel")
        transport.subscribe_telemetry("motion", on_motion_event)
    """

    # ── Command execution ─────────────────────────────────────────────────────

    @abstractmethod
    def execute(self, cmd: str, timeout_ms: int = 5000) -> Tuple[bool, str]:
        """
        Send a command and wait for acknowledgment.

        For quick commands (hb, tel, cancel, actuators):
            Returns immediately after Teensy acknowledges.

        For long-running motion commands (go, turn, …):
            Returns only when motion is DONE (Teensy sends TEL:motion:DONE:ok|fail).
            This blocking behaviour lets strategy.py write linear code:
                motion.go(500, 300)   # blocks until robot arrives
                actuators.grab("AB")  # then grab

        Returns (True, response_str) on success, (False, error_str) on failure/timeout.
        """
        ...

    @abstractmethod
    def fire(self, cmd: str) -> None:
        """Send a fire-and-forget command (no reply expected)."""
        ...

    def execute_calib(self, cmd: str,
                      ack_timeout_ms:  int = 3000,
                      calib_timeout_ms: int = 30000) -> Tuple[bool, str]:
        """Calibration fire-and-telemetry round-trip.

        Default implementation just delegates to execute() — real transports
        (XBee/Wired) override this with a two-phase wait (ACK first, then
        the 'T:cal <payload>' telemetry frame).

        Returns (True, 'kind=move …') on success, (False, error_str) otherwise.
        """
        return self.execute(cmd, timeout_ms=calib_timeout_ms)

    # ── Telemetry subscription ────────────────────────────────────────────────

    @abstractmethod
    def subscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        """
        Subscribe to a telemetry type pushed by Teensy.

        ttype examples: "pos", "motion", "safety", "chrono", "occ"
        callback(data: str) is called from a background thread.
        """
        ...

    @abstractmethod
    def unsubscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        """Unsubscribe a previously registered telemetry callback."""
        ...

    # ── Connection lifecycle ──────────────────────────────────────────────────

    @abstractmethod
    def connect(self) -> bool:
        """Open the connection. Returns True if successful."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the connection gracefully."""
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """True if the transport has an active connection."""
        ...

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def on_connect(self, callback: Callable[[], None]) -> None:
        """Optional: register a callback for connection events."""
        self._on_connect = callback

    def on_disconnect(self, callback: Callable[[], None]) -> None:
        """Optional: register a callback for disconnection events."""
        self._on_disconnect = callback

    # ── Default no-op callbacks ───────────────────────────────────────────────
    _on_connect:    Optional[Callable[[], None]] = None
    _on_disconnect: Optional[Callable[[], None]] = None
