"""
run_jetson.py — Jetson entry point (real hardware).

Connects to Teensy 4.1 via serial port (XBee or USB-CDC, auto-detected by
firmware at 57600 baud), then runs the Brain.

Usage on Jetson:
    python run_jetson.py --port /dev/ttyUSB0
    python run_jetson.py --port /dev/ttyTHS1

The serial port depends on how the XBee module is wired:
  - USB XBee dongle:   /dev/ttyUSB0
  - UART header:       /dev/ttyTHS1  (Jetson Nano) or /dev/ttyTHS0 (Orin)
"""

import sys
import os
import time
import signal
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from transport.xbee import XBeeTransport
from brain import Brain


def main():
    parser = argparse.ArgumentParser(description='holOS Jetson Brain')
    parser.add_argument('--port',  default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--auto-start', action='store_true',
                        help='Auto-start match when connected')
    args = parser.parse_args()

    transport = XBeeTransport(port=args.port)
    brain     = Brain(transport)

    # ── Connect ───────────────────────────────────────────────────────────────
    print(f"[Jetson] Connecting to Teensy on {args.port} @ 57600 bps…")
    if not brain.start():
        print("[Jetson] Connection failed. Check wiring and ensure Teensy is powered.")
        sys.exit(1)

    print("[Jetson] Connected ✓")

    # ── Hot-reload of strategy ────────────────────────────────────────────────
    brain.start_hot_reload()

    # ── Auto-start match if requested ─────────────────────────────────────────
    if args.auto_start:
        print("[Jetson] Auto-starting match…")
        brain.run_match()

    # ── Interactive REPL ──────────────────────────────────────────────────────
    print("\nCommands:")
    print("  start   — start match strategy")
    print("  stop    — stop match / cancel motion")
    print("  reload  — reload strategy/match.py")
    print("  status  — print connection + motion status")
    print("  q       — quit\n")

    def _signal_handler(sig, frame):
        print("\n[Jetson] Shutting down…")
        brain.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break

        if cmd in ('q', 'quit', 'exit'):
            brain.stop()
            break
        elif cmd == 'start':
            brain.run_match()
        elif cmd == 'stop':
            brain.stop_match()
        elif cmd == 'reload':
            brain.load_strategy()
        elif cmd == 'status':
            print(f"  connected: {transport.is_connected}")
            print(f"  strategy:  {'running' if brain.is_strategy_running() else 'idle'}")
            print(f"  chrono:    {brain.chrono.time_elapsed_s():.1f}s elapsed")
        else:
            print(f"  Unknown command: {cmd!r}")


if __name__ == '__main__':
    main()
