#!/usr/bin/env python3
"""
holos_cli.py — holOS command-line interface for robot connectivity and control.

A lightweight tool for diagnosing, testing, and interacting with the robot
without starting the full web UI.  Useful during development, at the pit,
and for automated testing.

Usage:
    python holos_cli.py                         # scan + interactive REPL
    python holos_cli.py -p COM6                 # connect to specific port
    python holos_cli.py scan                    # scan serial ports
    python holos_cli.py ping                    # ping all ports, find robot
    python holos_cli.py ping -p COM6            # ping specific port
    python holos_cli.py test -p COM6            # full connectivity self-test
    python holos_cli.py shell -p COM6           # interactive command shell
    python holos_cli.py exec -p COM6 "go(500,300)"  # single command
"""

import sys
import os
import time
import argparse
import threading
import signal

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


# ── ANSI colors ──────────────────────────────────────────────────────────────

def _green(s):  return f"\033[92m{s}\033[0m"
def _red(s):    return f"\033[91m{s}\033[0m"
def _yellow(s): return f"\033[93m{s}\033[0m"
def _cyan(s):   return f"\033[96m{s}\033[0m"
def _dim(s):    return f"\033[90m{s}\033[0m"
def _bold(s):   return f"\033[1m{s}\033[0m"


# ── Last port persistence ───────────────────────────────────────────────────

import pathlib as _pathlib

_LAST_PORT_FILE = _pathlib.Path(__file__).parent / '.holos_lastport'


def _save_last_port(port: str):
    try:
        _LAST_PORT_FILE.write_text(port.strip(), encoding='utf-8')
    except Exception:
        pass


def _load_last_port() -> str:
    try:
        return _LAST_PORT_FILE.read_text(encoding='utf-8').strip()
    except Exception:
        return ''


def _resolve_port(args_port):
    """Resolve port from args, then last saved, then first available."""
    if args_port:
        return args_port
    last = _load_last_port()
    if last:
        return last
    ports = scan_ports()
    if ports:
        return ports[0][0]
    return None


# ── Port scanning ────────────────────────────────────────────────────────────

def scan_ports():
    """List available serial ports with descriptions."""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        return [(p.device, p.description, p.hwid) for p in sorted(ports, key=lambda p: p.device)]
    except ImportError:
        print(_red("  pyserial not installed — run: pip install pyserial"))
        return []


def cmd_scan(args):
    """Scan and display available serial ports."""
    print(_bold("\n  Serial Port Scan"))
    print("  " + "─" * 50)
    ports = scan_ports()
    if not ports:
        print(_yellow("  No serial ports found."))
        return ports
    for device, desc, hwid in ports:
        print(f"  {_cyan(device):30s} {desc}")
        if hwid and hwid != 'n/a':
            print(f"  {' ':30s} {_dim(hwid)}")
    print(f"\n  {len(ports)} port(s) found.\n")
    return ports


# ── Ping ─────────────────────────────────────────────────────────────────────

def ping_port(port, baudrate=57600, timeout=2.0):
    """
    Try to connect to a serial port and get a pong from the firmware.
    Returns (success, bridge_type, latency_ms, message).

    Uses a background thread so we never hang if Serial() blocks (Windows
    Bluetooth/virtual ports can block on open indefinitely).
    """
    import serial as pyserial

    result = [False, None, 0.0, "timeout"]

    def _worker():
        try:
            ser = pyserial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        except pyserial.SerialException as e:
            result[0] = False
            result[3] = str(e)[:60]
            return

        try:
            # Drain stale data
            time.sleep(0.03)
            try:
                while ser.in_waiting:
                    ser.read(ser.in_waiting)
            except Exception:
                pass

            t0 = time.time()
            deadline = t0 + timeout

            while time.time() < deadline:
                try:
                    ser.write(b"ping\n")
                except Exception as e:
                    result[3] = f"write error: {e}"
                    return

                time.sleep(0.12)

                buf = b""
                try:
                    while ser.in_waiting:
                        buf += ser.read(ser.in_waiting)
                except Exception:
                    pass

                text = buf.decode('ascii', errors='replace')
                for line in text.strip().split('\n'):
                    line = line.strip()
                    if line.startswith('pong'):
                        latency_ms = (time.time() - t0) * 1000
                        bridge = line.split(':', 1)[1] if ':' in line else None
                        result[0] = True
                        result[1] = bridge
                        result[2] = latency_ms
                        result[3] = "ok"
                        return

            result[3] = "no response"
        finally:
            try:
                ser.close()
            except Exception:
                pass

    th = threading.Thread(target=_worker, daemon=True, name=f"ping-{port}")
    th.start()
    # Wait at most timeout + 2s grace for the thread (handles ports that block on open)
    th.join(timeout=timeout + 2.0)
    if th.is_alive():
        result[3] = "port blocked (hanging open)"
    return tuple(result)


def cmd_ping(args):
    """Ping serial ports to find the robot."""
    from shared.config import BRIDGE_BAUDRATE
    baud = BRIDGE_BAUDRATE

    if args.port:
        ports_to_try = [(args.port, "", "")]
    else:
        ports_to_try = scan_ports()
        if not ports_to_try:
            print(_red("  No serial ports found."))
            return []

    print(_bold(f"\n  Pinging {len(ports_to_try)} port(s) @ {baud} baud…"))
    print("  " + "─" * 55)

    found = []
    for device, desc, _ in ports_to_try:
        label = f"{device}"
        if desc and desc != device:
            label += f" ({desc})"
        sys.stdout.write(f"  {label:45s} ")
        sys.stdout.flush()
        ok, bridge, latency, msg = ping_port(device, baud, timeout=2.0)
        if ok:
            bridge_label = bridge or "unknown"
            print(_green(f"pong:{bridge_label}") + _dim(f"  {latency:.0f}ms"))
            found.append((device, bridge, latency))
        else:
            print(_dim(msg))

    print()
    if found:
        print(f"  {_green(f'{len(found)} robot(s) found:')}")
        for dev, br, lat in found:
            print(f"    {_cyan(dev)}  bridge={br or '?'}  latency={lat:.0f}ms")
    else:
        print(_yellow("  No robot responded. Check power and wiring."))
    print()
    return found


# ── Full connectivity test ───────────────────────────────────────────────────

def cmd_test(args):
    """Full connectivity self-test suite."""
    from shared.config import BRIDGE_BAUDRATE
    from transport.xbee import XBeeTransport

    print(_bold("\n  holOS Connectivity Test Suite"))
    print("  " + "=" * 50)

    results = []

    def _test(name, fn):
        sys.stdout.write(f"  [{len(results)+1:2d}] {name:40s} ")
        sys.stdout.flush()
        try:
            ok, detail = fn()
            if ok:
                print(_green("PASS") + (f"  {_dim(detail)}" if detail else ""))
            else:
                print(_red("FAIL") + (f"  {detail}" if detail else ""))
            results.append((name, ok, detail))
        except Exception as e:
            print(_red("ERR ") + f"  {e}")
            results.append((name, False, str(e)))

    # ── Test 1: Port scan ─────────────────────────────────────────────────
    ports = []
    def t_scan():
        nonlocal ports
        ports = scan_ports()
        return (len(ports) > 0, f"{len(ports)} port(s)")
    _test("Serial port scan", t_scan)

    # ── Test 2: Target port exists ────────────────────────────────────────
    target_port = _resolve_port(args.port)
    if not target_port:
        print(_red("\n  No port to test. Use -p COM6 or connect once first.\n"))
        return

    def t_port_exists():
        found = any(p[0] == target_port for p in ports)
        return (found, target_port)
    _test(f"Port {target_port} exists", t_port_exists)

    # ── Test 3: Serial open ───────────────────────────────────────────────
    import serial as pyserial
    ser = None
    def t_serial_open():
        nonlocal ser
        ser = pyserial.Serial(port=target_port, baudrate=BRIDGE_BAUDRATE, timeout=0.1)
        return (True, f"{BRIDGE_BAUDRATE} baud")
    _test("Serial port open", t_serial_open)
    if ser:
        ser.close()
        ser = None

    # ── Test 4: Ping/pong handshake ───────────────────────────────────────
    pong_bridge = [None]
    def t_ping():
        ok, bridge, latency, msg = ping_port(target_port, BRIDGE_BAUDRATE)
        pong_bridge[0] = bridge
        return (ok, f"bridge={bridge or '?'}, latency={latency:.0f}ms" if ok else msg)
    _test("Ping/pong handshake", t_ping)

    # ── Test 5: Bridge auto-detection ─────────────────────────────────────
    def t_bridge():
        return (pong_bridge[0] is not None, f"detected: {pong_bridge[0]}")
    _test("Bridge auto-identification", t_bridge)

    # ── Test 6: Transport connect ─────────────────────────────────────────
    transport = [None]
    def t_transport():
        t = XBeeTransport(port=target_port)
        ok = t.connect()
        if ok:
            transport[0] = t
            return (True, f"bridge={t.bridge_type}")
        return (False, "connect() returned False")
    _test("XBeeTransport.connect()", t_transport)

    t = transport[0]
    if not t:
        print(_red(f"\n  Cannot continue — transport connect failed on {target_port}\n"))
        _summary(results)
        return

    # ── Test 7: Heartbeat ─────────────────────────────────────────────────
    def t_heartbeat():
        ok, resp = t.execute('hb', timeout_ms=3000)
        return (ok, resp)
    _test("Heartbeat (hb -> ok)", t_heartbeat)

    # ── Test 8: Telemetry request ─────────────────────────────────────────
    def t_telemetry():
        ok, resp = t.execute('tel', timeout_ms=3000)
        return (ok, resp[:60] if ok else resp)
    _test("Telemetry request (tel)", t_telemetry)

    # ── Test 9: Position query ────────────────────────────────────────────
    pos_data = [None]
    def t_position():
        evt = threading.Event()
        def _on_pos(data):
            pos_data[0] = data
            evt.set()
        t.subscribe_telemetry('pos', _on_pos)
        t.fire('tel')  # trigger telemetry push
        ok = evt.wait(timeout=3.0)
        return (ok, pos_data[0] if ok else "no TEL:pos received")
    _test("Position telemetry (TEL:pos)", t_position)

    # ── Test 10: Round-trip latency ───────────────────────────────────────
    def t_latency():
        times = []
        for _ in range(5):
            t0 = time.time()
            ok, _ = t.execute('hb', timeout_ms=2000)
            if ok:
                times.append((time.time() - t0) * 1000)
        if not times:
            return (False, "all hb timed out")
        avg = sum(times) / len(times)
        mx = max(times)
        return (True, f"avg={avg:.0f}ms  max={mx:.0f}ms  ({len(times)}/5 ok)")
    _test("Round-trip latency (5x hb)", t_latency)

    # ── Test 11: Rapid fire stress ────────────────────────────────────────
    def t_stress():
        success = 0
        total = 20
        for i in range(total):
            ok, _ = t.execute('hb', timeout_ms=1000)
            if ok:
                success += 1
        rate = success / total * 100
        return (rate >= 90, f"{success}/{total} ({rate:.0f}%)")
    _test("Rapid-fire stress (20x hb)", t_stress)

    # ── Cleanup ───────────────────────────────────────────────────────────
    try:
        t.disconnect()
    except Exception:
        pass

    _summary(results)


def _summary(results):
    passed = sum(1 for _, ok, _ in results if ok)
    total  = len(results)
    print("\n  " + "=" * 50)
    color = _green if passed == total else (_yellow if passed >= total - 2 else _red)
    print(f"  {color(f'{passed}/{total} tests passed')}")
    if passed == total:
        print(_green("  Robot is ready."))
    print()


# ── Interactive shell ────────────────────────────────────────────────────────

def cmd_shell(args):
    """Interactive command shell — send commands, see telemetry."""
    from shared.config import BRIDGE_BAUDRATE
    from transport.xbee import XBeeTransport

    port = _resolve_port(args.port)
    if not port:
        print(_red("  No port available. Use -p COM6"))
        return

    print(f"  Connecting to {_cyan(port)} @ {BRIDGE_BAUDRATE}...")
    t = XBeeTransport(port=port)
    if not t.connect():
        print(_red(f"  Failed to connect on {port}"))
        return

    bridge = t.bridge_type or '?'
    print(_green(f"  Connected — bridge={bridge}"))
    _save_last_port(port)

    # Live telemetry display
    _tel_data = {}
    def _on_tel(ttype):
        def handler(data):
            _tel_data[ttype] = data
        return handler

    for tt in ['pos', 'motion', 'safety', 'chrono']:
        t.subscribe_telemetry(tt, _on_tel(tt))

    print(_bold("\n  holOS Shell"))
    print("  " + "-" * 40)
    print("  Commands: any firmware command (hb, tel, go(x,y), turn(a), ...)")
    print("  Special:  .tel     — show latest telemetry")
    print("            .ping    — measure latency")
    print("            .stress  — rapid-fire 20x hb")
    print("            .bridge  — show bridge type")
    print("            .quit    — disconnect and exit")
    print()

    def _sigint(sig, frame):
        print(_dim("\n  Disconnecting..."))
        t.disconnect()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    while t.is_connected:
        try:
            cmd = input(_cyan("holOS> ")).strip()
        except EOFError:
            break
        if not cmd:
            continue

        # Special commands
        if cmd in ('.quit', '.exit', 'q'):
            break
        if cmd == '.tel':
            if not _tel_data:
                print(_dim("  (no telemetry received yet)"))
            for k, v in _tel_data.items():
                print(f"  {k:10s} = {v}")
            continue
        if cmd == '.ping':
            times = []
            for _ in range(5):
                t0 = time.time()
                ok, _ = t.execute('hb', timeout_ms=2000)
                if ok:
                    times.append((time.time() - t0) * 1000)
            if times:
                print(f"  avg={sum(times)/len(times):.0f}ms  max={max(times):.0f}ms  ({len(times)}/5)")
            else:
                print(_red("  all timed out"))
            continue
        if cmd == '.stress':
            ok_count = 0
            for i in range(20):
                ok, _ = t.execute('hb', timeout_ms=1000)
                if ok: ok_count += 1
            print(f"  {ok_count}/20 ({ok_count/20*100:.0f}%)")
            continue
        if cmd == '.bridge':
            print(f"  bridge_type = {t.bridge_type}")
            print(f"  port        = {t._port}")
            print(f"  baudrate    = {t._baudrate}")
            print(f"  connected   = {t.is_connected}")
            continue
        if cmd == '.help':
            print("  .tel     — show latest telemetry")
            print("  .ping    — measure round-trip latency (5x hb)")
            print("  .stress  — rapid-fire 20x hb")
            print("  .bridge  — show bridge info")
            print("  .quit    — disconnect and exit")
            print("  (any other input is sent as a firmware command)")
            continue

        # Regular firmware command
        t0 = time.time()
        timeout = 30000 if any(cmd.startswith(p) for p in
                               ['go(', 'go_coc(', 'goAlign(', 'goPolar(', 'turn(', 'align(', 'via(']) else 5000
        ok, resp = t.execute(cmd, timeout_ms=timeout)
        dt = (time.time() - t0) * 1000
        if ok:
            print(f"  {_green('OK')}  {resp}  {_dim(f'{dt:.0f}ms')}")
        else:
            print(f"  {_red('FAIL')}  {resp}  {_dim(f'{dt:.0f}ms')}")

    print(_dim("  Disconnecting..."))
    t.disconnect()
    print("  Done.\n")


# ── Single command exec ──────────────────────────────────────────────────────

def cmd_exec(args):
    """Send a single command and exit."""
    from shared.config import BRIDGE_BAUDRATE
    from transport.xbee import XBeeTransport

    port = _resolve_port(args.port)
    if not port:
        print(_red("No port. Use -p COM6"))
        sys.exit(1)

    t = XBeeTransport(port=port)
    if not t.connect():
        print(_red(f"Failed to connect on {port}"))
        sys.exit(1)

    cmd_text = ' '.join(args.cmd_args)
    timeout = 30000 if any(cmd_text.startswith(p) for p in
                           ['go(', 'go_coc(', 'goAlign(', 'goPolar(', 'turn(', 'align(', 'via(']) else 5000
    ok, resp = t.execute(cmd_text, timeout_ms=timeout)
    t.disconnect()

    if ok:
        print(f"{_green('OK')}  {resp}")
    else:
        print(f"{_red('FAIL')}  {resp}")
        sys.exit(1)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    # ── Shared --port arg via parent parser ───────────────────────────────
    port_parent = argparse.ArgumentParser(add_help=False)
    port_parent.add_argument('-p', '--port', default=None,
                             help='Serial port (e.g. COM6, /dev/ttyACM0). '
                                  'Auto-detects if omitted.')

    parser = argparse.ArgumentParser(
        description='holOS CLI — robot connectivity and control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
        parents=[port_parent],
    )

    sub = parser.add_subparsers(dest='subcmd')

    sub.add_parser('scan',  parents=[port_parent], help='Scan serial ports')
    sub.add_parser('ping',  parents=[port_parent], help='Ping ports to find robot')
    sub.add_parser('test',  parents=[port_parent], help='Full connectivity self-test')
    sub.add_parser('shell', parents=[port_parent], help='Interactive command shell')

    exec_p = sub.add_parser('exec', parents=[port_parent], help='Send a single command')
    exec_p.add_argument('cmd_args', nargs='+', help='Command to send (e.g. "go(500,300)")')

    args = parser.parse_args()

    if args.subcmd == 'scan':
        cmd_scan(args)
    elif args.subcmd == 'ping':
        cmd_ping(args)
    elif args.subcmd == 'test':
        cmd_test(args)
    elif args.subcmd == 'shell':
        cmd_shell(args)
    elif args.subcmd == 'exec':
        cmd_exec(args)
    else:
        # Default: scan + ping, then shell on first found port
        print(_bold("\n  holOS CLI v1.0"))
        print("  " + "=" * 35)
        cmd_scan(args)
        found = cmd_ping(args)
        if found and not args.port:
            args.port = found[0][0]
        if args.port:
            cmd_shell(args)
        else:
            print(_yellow("  No robot found. Use -p COM6 to specify a port.\n"))


if __name__ == '__main__':
    main()
