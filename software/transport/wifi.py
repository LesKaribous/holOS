"""
transport/wifi.py — TCP transport via the ESP32-S3 Xiao WiFi bridge.

Physical topology:

    Teensy 4.1 BRIDGE_XBEE (Serial2 @ 57600)
        │  UART  D6/D7
        ▼
    ESP32-S3 Xiao  ── WiFi ──► home router ──► Jetson / PC
                                                 ▲
                                                 │  TCP socket
                                                 │  (this transport)

The Xiao runs a transparent UART↔TCP relay (firmware in firmware/esp_xiao/).
Bytes flow byte-for-byte in both directions, so the CRC8-framed protocol
that already works over the XBee link works here untouched — including the
ping/pong handshake. The Teensy can't tell anything has changed.

Bridge selection lives on the Jetson side only: this class replaces
XBeeTransport in run.py when BRIDGE_KIND == 'wifi'.
"""

import socket
import threading
import time
from typing import Optional

from .xbee import XBeeTransport


class WiFiTransport(XBeeTransport):
    """Bytes over TCP instead of pyserial — everything else is XBeeTransport.

    We subclass so the framing, heartbeat, telemetry dispatch, motion-done
    tracking, calibration round-trip, and _process_line() all come for free.
    Only the byte pipe (connect / disconnect / read / write) is replaced.
    """

    TRANSPORT_TYPE = 'wifi'

    @property
    def transport_type(self) -> str:
        return self.TRANSPORT_TYPE

    # Always advertise 'wifi' upstream regardless of what the Teensy answers
    # in its pong reply. From the Teensy's point of view the link is still
    # Serial2 (BRIDGE_XBEE), so it will send "pong:xbee" — but on the host
    # side this is genuinely a WiFi link and the UI should label it as such.
    @property
    def bridge_type(self) -> Optional[str]:
        return 'wifi' if self._connected else None

    def __init__(self, host: str, port: int = 9000,
                 connect_timeout_s: float = 5.0):
        # The parent constructor wants (port, baudrate). Pass dummies — the
        # serial-specific fields are never touched once we override connect().
        super().__init__(port="(wifi)", baudrate=0)
        self._host = host
        self._tcp_port = port
        self._connect_timeout_s = connect_timeout_s
        self._sock: Optional[socket.socket] = None

    # ── Connection lifecycle ──────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            sock = socket.create_connection(
                (self._host, self._tcp_port),
                timeout=self._connect_timeout_s,
            )
        except OSError as e:
            print(f"[WiFiTransport] Connect to {self._host}:{self._tcp_port} "
                  f"failed: {e}")
            return False

        # Switch to blocking with a per-read timeout — this lets the reader
        # loop poll _running every 100 ms (same cadence as XBeeTransport's
        # pyserial timeout) so disconnect() can shut it down cleanly.
        sock.settimeout(0.1)
        # Disable Nagle — telemetry frames are small and frequent, latency
        # matters more than throughput.
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        # Best-effort keepalive so a half-open socket (Xiao reboot, AP drop)
        # gets detected by the kernel within ~30 s instead of hanging forever.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "TCP_KEEPIDLE"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE,  10)
            if hasattr(socket, "TCP_KEEPINTVL"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
            if hasattr(socket, "TCP_KEEPCNT"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT,   3)
        except OSError:
            pass

        self._sock    = sock
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="wifi-reader"
        )
        self._reader_thread.start()

        # ping/pong handshake — identical to XBeeTransport.connect().
        # The Xiao bridge is transparent, so the pong comes from the Teensy.
        deadline = time.time() + 8.0
        while not self._connected and time.time() < deadline:
            try:
                self._sock.sendall(b"ping\n")
            except OSError as e:
                print(f"[WiFiTransport] ping send failed: {e}")
                self._cleanup()
                return False
            time.sleep(0.5)

        if self._connected:
            self._start_heartbeat()
            return True

        print(f"[WiFiTransport] No pong from {self._host}:{self._tcp_port} "
              f"— Teensy not responding via Xiao")
        self._cleanup()
        return False

    # _cleanup() is inherited but it touches self._serial. Override to close
    # the socket and then defer to the parent for thread-join + state reset.
    def _cleanup(self) -> None:
        self._running       = False
        self._connected     = False
        self._heartbeat_ok  = False
        self._bridge_type   = None

        # Release anyone waiting on a reply / motion-done.
        with self._lock:
            for evt in self._pending.values():
                evt.set()
            self._pending.clear()
            self._replies.clear()
        self._motion_done_evt.set()
        self._waiting_motion = False

        # Close the socket FIRST so the reader thread's recv unblocks.
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

        cur = threading.current_thread()
        if (self._reader_thread is not None
                and self._reader_thread.is_alive()
                and self._reader_thread is not cur):
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
        if (self._hb_thread is not None
                and self._hb_thread.is_alive()
                and self._hb_thread is not cur):
            self._hb_thread.join(timeout=2.0)
            self._hb_thread = None

        self._tel_subs.clear()

    # ── Byte pipe overrides ──────────────────────────────────────────────────

    def _write(self, data: str) -> None:
        # Same surface as XBeeTransport._write() — feeds raw TX subscribers,
        # logs to MATCH_LOGGER, swallows write errors so caller isn't broken.
        if self._sock is None:
            return
        with self._write_lock:
            try:
                self._sock.sendall(data.encode('ascii'))
                # MATCH_LOGGER import is done at class init; replicate the
                # parent's logging hook to keep parity.
                try:
                    from services.match_logger import MATCH_LOGGER as _M
                    if _M is not None:
                        _M.log('uart', '> ' + data.rstrip('\n'))
                except Exception:
                    pass
                for cb in self._tel_subs.get('_raw_tx', []):
                    try: cb(data.rstrip('\n'))
                    except Exception: pass
            except OSError as e:
                print(f"[WiFiTransport] Write error: {e}")

    def _reader_loop(self) -> None:
        # Mirror XBeeTransport._reader_loop() but read from the socket. The
        # 0.1 s socket timeout (set in connect) drives the _running poll.
        buf = b""
        while self._running:
            sock = self._sock
            if sock is None:
                return
            try:
                chunk = sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as e:
                if self._running:
                    print(f"[WiFiTransport] Reader error: {e}")
                break
            if not chunk:
                # Peer closed (FIN) — Xiao rebooted, WiFi dropped, or
                # another client took our slot.
                if self._running:
                    print("[WiFiTransport] Socket closed by peer")
                break

            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                decoded = line.decode('ascii', errors='ignore')
                try:
                    from services.match_logger import MATCH_LOGGER as _M
                    if _M is not None:
                        _M.log('uart', '< ' + decoded)
                except Exception:
                    pass
                for cb in self._tel_subs.get('_raw', []):
                    try: cb(decoded)
                    except Exception: pass
                self._process_line(decoded)

        if self._running and self._connected:
            print("[WiFiTransport] Reader exited unexpectedly — signaling disconnect")
            self._cleanup()
            if self._on_disconnect:
                self._on_disconnect()
