"""
transport/xbee.py - Real XBee serial transport for Jetson hardware.

Connects to the Teensy 4.1 via XBee modules on a serial port.
Implements the holOS intercom protocol (CRC8-SMBUS framed requests).

Usage:
    transport = XBeeTransport(port="/dev/ttyUSB0", baudrate=31250)
    transport.connect()
    ok, resp = transport.execute("go(500,300)", timeout_ms=30000)
"""

import threading
import time
import serial
from typing import Callable, Dict, List, Optional, Tuple

from .base import Transport
from shared.protocol import encode_request, encode_reply, parse_frame, encode_telemetry
from shared.config import HEARTBEAT_INTERVAL_S, JETSON_TIMEOUT_S


class XBeeTransport(Transport):
    """
    XBee serial transport (XBee radio via Jetson).

    For direct USB-CDC connections use WiredTransport (transport/wired.py).
    
    Background thread reads from serial continuously.
    execute() sends a request and blocks until a reply arrives or timeout.
    Telemetry callbacks are dispatched from the reader thread.
    """

    TRANSPORT_TYPE = 'xbee'

    @property
    def transport_type(self) -> str:
        return self.TRANSPORT_TYPE

    def __init__(self, port: str, baudrate: int = 31250):
        self._port     = port
        self._baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._connected = False

        # Request tracking
        self._lock    = threading.Lock()
        self._uid_ctr = 0
        self._pending: Dict[int, threading.Event] = {}
        self._replies: Dict[int, str]             = {}

        # Motion done tracking (separate from request reply)
        self._motion_done_evt   = threading.Event()
        self._motion_done_ok    = False
        self._waiting_motion    = False

        # Write-side mutex — prevents two threads (test runner + heartbeat)
        # from interleaving bytes on the serial port simultaneously.
        self._write_lock = threading.Lock()

        # Telemetry subscribers
        self._tel_subs: Dict[str, List[Callable[[str], None]]] = {}

        # Heartbeat
        self._hb_thread: Optional[threading.Thread] = None

        # Reader thread
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

    # ── Connection lifecycle ──────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.1,
            )
            self._running = True
            self._reader_thread = threading.Thread(
                target=self._reader_loop, daemon=True, name="xbee-reader"
            )
            self._reader_thread.start()

            # Wait for connection confirmation.
            # XBee/Jetson mode (31250 baud): raw "ping\n" → firmware replies "pong\n"
            # USB_DIRECT mode (115200 baud): firmware only understands CRC-framed
            #   requests, so send a framed hb — any valid reply confirms connectivity.
            _usb_direct = (self._baudrate == 115200)
            deadline = time.time() + 3.0
            while not self._connected and time.time() < deadline:
                if _usb_direct:
                    self._serial.write(encode_request(0, 'hb').encode('ascii'))
                else:
                    self._serial.write(b"ping\n")
                time.sleep(0.2)

            if self._connected:
                self._start_heartbeat()
                if self._on_connect:
                    self._on_connect()
                return True

            # No pong received - clean up fully before returning failure
            print(f"[Transport] No response on {self._port} - closing port")
            self._cleanup()
            return False

        except serial.SerialException as e:
            print(f"[Transport] Connect failed: {e}")
            self._cleanup()
            return False

    def disconnect(self) -> None:
        self._cleanup()
        if self._on_disconnect:
            self._on_disconnect()

    def _cleanup(self) -> None:
        """Stop reader/heartbeat threads and close the serial port unconditionally."""
        self._running    = False
        self._connected  = False
        # Release any pending execute() calls so they don't hang
        with self._lock:
            for evt in self._pending.values():
                evt.set()
            self._pending.clear()
        self._motion_done_evt.set()   # unblock any waiting motion call
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Command execution ─────────────────────────────────────────────────────

    def execute(self, cmd: str, timeout_ms: int = 5000) -> Tuple[bool, str]:
        """
        Send a command, wait for Teensy acknowledgment.
        For motion commands (go/turn/align/goPolar), also waits for motion completion.
        """
        is_motion_cmd = _is_motion_command(cmd)

        with self._lock:
            uid = self._uid_ctr
            self._uid_ctr += 1
            evt = threading.Event()
            self._pending[uid] = evt
            if is_motion_cmd:
                self._motion_done_evt.clear()
                self._motion_done_ok  = False
                self._waiting_motion  = True

        frame = encode_request(uid, cmd)
        self._write(frame)

        # Wait for acknowledgment
        ok = evt.wait(timeout=timeout_ms / 1000.0)
        with self._lock:
            response = self._replies.pop(uid, "")
            self._pending.pop(uid, None)

        if not ok:
            return (False, "timeout")

        if response.startswith("err"):
            return (False, response)

        # For motion commands, additionally wait for motion DONE telemetry.
        # Use the caller-specified timeout (minimum 30 s so strategy code with
        # the default 5 s command timeout still has a reasonable motion window).
        if is_motion_cmd and self._waiting_motion:
            motion_timeout = max(timeout_ms / 1000.0, 30.0)
            finished = self._motion_done_evt.wait(timeout=motion_timeout)
            self._waiting_motion = False
            if not finished:
                return (False, "motion_timeout")
            return (self._motion_done_ok, "ok" if self._motion_done_ok else "stall")

        return (True, response)

    def fire(self, cmd: str) -> None:
        """Send a fire-and-forget command (framed, no reply awaited).

        Uses the same uid:content|crc framing as execute() so T41's
        _readBridgeSerial() accepts it. The reply from T41 (if any) will
        arrive as an unknown uid and be silently discarded by _process_line().
        """
        with self._lock:
            uid = self._uid_ctr
            self._uid_ctr += 1
        frame = encode_request(uid, cmd)
        self._write(frame)

    # ── Telemetry subscription ────────────────────────────────────────────────

    def subscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        self._tel_subs.setdefault(ttype, []).append(callback)

    def unsubscribe_telemetry(self, ttype: str, callback: Callable[[str], None]) -> None:
        if ttype in self._tel_subs:
            try:
                self._tel_subs[ttype].remove(callback)
            except ValueError:
                pass

    # ── Internal: serial write ────────────────────────────────────────────────

    def _write(self, data: str) -> None:
        if self._serial and self._serial.is_open:
            with self._write_lock:
                try:
                    self._serial.write(data.encode('ascii'))
                except serial.SerialException as e:
                    print(f"[Transport] Write error: {e}")

    # ── Internal: reader loop ─────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                chunk = self._serial.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    self._process_line(line.decode('ascii', errors='ignore'))
            except Exception as e:
                if self._running:
                    print(f"[Transport] Reader error: {e}")
                break

        # If the loop exited while we were still supposed to be running,
        # it means the serial port dropped unexpectedly (USB unplugged, firmware
        # reset, etc.).  Signal disconnection so the server can update the UI.
        if self._running and self._connected:
            print("[Transport] Reader loop exited unexpectedly — signaling disconnect")
            self._cleanup()
            if self._on_disconnect:
                self._on_disconnect()

    def _process_line(self, line: str) -> None:
        kind, id_or_type, data = parse_frame(line)

        if kind == 'ping':
            print(f"[Transport] Received ping")
            self._write("pong\n")
            return

        if kind == 'pong':
            print(f"[Transport] Received pong")
            if not self._connected:
                self._connected = True
                if self._on_connect:
                    self._on_connect()
            return

        if kind == 'reply':
            uid = id_or_type
            #print(f"[Transport] Received reply uid={uid} data={data}")
            if not self._connected:
                # USB_DIRECT handshake: receiving any valid reply means the
                # firmware is alive and the framed protocol is working.
                self._connected = True
                return
            with self._lock:
                self._replies[uid] = data
                if uid in self._pending:
                    self._pending[uid].set()
            return

        if kind == 'tel':
            ttype = id_or_type
            #print(f"[Transport] Received telemetry type={ttype} data={data}")
            # Handle motion done internally.
            # Firmware sends "DONE:ok,dur=…,dist=…,stall=0" (with stats appended),
            # so we must use startswith(), NOT ==, to detect a successful DONE.
            if ttype == 'motion' and data.startswith('DONE:'):
                success = data.startswith('DONE:ok')
                if self._waiting_motion:
                    self._motion_done_ok = success
                    self._motion_done_evt.set()
            # Dispatch to subscribers
            for cb in self._tel_subs.get(ttype, []):
                try:
                    cb(data)
                except Exception as e:
                    print(f"[Transport] Telemetry callback error: {e}")
            return

        if kind == 'request':
            # Teensy sending us a request (rare - e.g. asking for team)
            uid  = id_or_type
            content = data
            resp = self._handle_teensy_request(content)
            reply_frame = encode_reply(uid, resp)
            self._write(reply_frame)
            return

        # ── Unframed console output ───────────────────────────────────────────
        # CONSOLE_SERIAL == BRIDGE_SERIAL on USB mode.  Commands like "help"
        # print raw text to Serial without CRC framing.  Forward these lines
        # to the '_console' subscribers so the terminal UI can display them.
        stripped = line.strip()
        if stripped and kind is None:
            for cb in self._tel_subs.get('_console', []):
                try:
                    cb(stripped)
                except Exception:
                    pass

    def _handle_teensy_request(self, content: str) -> str:
        """Handle incoming requests from Teensy (e.g. team query)."""
        # Can be overridden by brain
        return "ok"

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _start_heartbeat(self) -> None:
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="xbee-heartbeat"
        )
        self._hb_thread.start()

    def _heartbeat_loop(self) -> None:
        while self._running and self._connected:
            try:
                # Skip heartbeat while a motion command is in progress —
                # the Teensy is busy and can't reply within 1 s anyway, and
                # a concurrent write would risk interleaving bytes even with
                # the write lock (execute() holds the lock for the full write).
                if not self._waiting_motion:
                    ok, _ = self.execute("hb", timeout_ms=2000)
                    if not ok and self._connected:
                        print("[Transport] Heartbeat failed - connection lost")
                        # Call disconnect() — not just _connected=False — so that
                        # _cleanup() closes the serial port before on_disconnect fires.
                        # Without this, the port stays open and reconnect attempts fail
                        # with "device or resource busy".
                        self.disconnect()
                        return
            except Exception:
                pass
            time.sleep(HEARTBEAT_INTERVAL_S)


# ── Helpers ───────────────────────────────────────────────────────────────────

_MOTION_CMDS = ('go(', 'goPolar(', 'turn(', 'align(', 'goAlign(', 'move(')

def _is_motion_command(cmd: str) -> bool:
    return any(cmd.startswith(c) for c in _MOTION_CMDS)
