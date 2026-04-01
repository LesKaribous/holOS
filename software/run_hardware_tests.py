#!/usr/bin/env python3
"""
run_hardware_tests.py — CLI entry point for the holOS hardware test suite.

Connects to the Teensy 4.1 via USB-CDC or XBee and runs the test suite.
Results are printed in real-time.

Examples
--------
  # USB-CDC direct connection (115200 baud)
  python run_hardware_tests.py --port /dev/ttyACM0

  # XBee radio via Jetson (31250 baud)
  python run_hardware_tests.py --port /dev/ttyUSB0 --baudrate 31250

  # Run a single suite only
  python run_hardware_tests.py --port /dev/ttyACM0 --suite connection

  # Available suites: connection, telemetry, motion, safety, intercom, reliability
  python run_hardware_tests.py --port /dev/ttyACM0 --suite all

  # Skip interactive tests (no user input required)
  python run_hardware_tests.py --port /dev/ttyACM0 --skip-interactive

  # List all available tests without running them
  python run_hardware_tests.py --list
"""

import argparse
import sys
import time
import threading
import os

# ── Make sure 'software/' is in the Python path ──────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))

from transport.xbee   import XBeeTransport
from transport.wired  import WiredTransport
from tests.hardware_tests import (
    HardwareTestRunner, SUITES, ALL_TESTS, TestResult
)


# ── ANSI colours (disabled on Windows or when stdout is not a TTY) ─────────

def _supports_colour() -> bool:
    if sys.platform == 'win32':
        return False
    return hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()

_COLOUR = _supports_colour()

def _green(s):  return f'\033[32m{s}\033[0m' if _COLOUR else s
def _red(s):    return f'\033[31m{s}\033[0m' if _COLOUR else s
def _yellow(s): return f'\033[33m{s}\033[0m' if _COLOUR else s
def _bold(s):   return f'\033[1m{s}\033[0m'  if _COLOUR else s
def _dim(s):    return f'\033[2m{s}\033[0m'  if _COLOUR else s


# ── CLI output ────────────────────────────────────────────────────────────────

def _header(msg: str) -> None:
    width = 72
    print()
    print('─' * width)
    print(f'  {msg}')
    print('─' * width)

def _print_result(result: TestResult) -> None:
    meta   = ALL_TESTS.get(result.id, {})
    name   = meta.get('name', result.id)
    if result.passed:
        status = _green('✓ PASS')
    else:
        status = _red('✗ FAIL')

    dt_str = f'{result.duration_ms:6.0f} ms'
    print(f'  {status}  {_dim(dt_str)}  {_bold(name)}')
    if not result.passed or result.msg not in ('OK', ''):
        indent = '           '
        print(f'{indent}{_dim(result.msg)}')

def _print_summary(results: list, elapsed_s: float) -> None:
    total  = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = total - passed
    _header(f'RÉSULTATS — {passed}/{total} tests réussis  ({elapsed_s:.1f} s)')
    if failed:
        print(_red(f'  {failed} test(s) en échec:'))
        for r in results:
            if not r.passed:
                meta = ALL_TESTS.get(r.id, {})
                print(f'    • {meta.get("name", r.id)}: {r.msg}')
    else:
        print(_green('  Tous les tests réussis !'))
    print()


# ── List mode ─────────────────────────────────────────────────────────────────

def _list_tests() -> None:
    print()
    print(_bold('Tests disponibles:'))
    for sid, suite in SUITES.items():
        print(f'\n  {_bold(suite["label"])}  [suite: {sid}]')
        for t in suite['tests']:
            interactive = ' [INTERACTIF]' if 'INT' in t.get('name', '') else ''
            print(f'    {t["id"]:25s}  {t["name"]}{interactive}')
            print(f'    {"":<25s}  {_dim(t["desc"])}')
    print()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description='holOS hardware test suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--port',  '-p', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baud rate — 115200 for USB-CDC, 31250 for XBee (default: 115200)')
    parser.add_argument('--suite', '-s', default='all',
                        choices=list(SUITES.keys()) + ['all'],
                        help='Suite to run (default: all)')
    parser.add_argument('--skip-interactive', action='store_true',
                        help='Skip tests requiring user interaction')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List all tests without running them')
    parser.add_argument('--timeout', type=int, default=8000,
                        help='Default command timeout in ms (default: 8000)')

    args = parser.parse_args()

    if args.list:
        _list_tests()
        return 0

    # ── Transport selection ────────────────────────────────────────────────
    if args.baudrate == 115200:
        transport = WiredTransport(port=args.port)
        transport_name = f'USB-CDC @ {args.port} (115200)'
    else:
        transport = XBeeTransport(port=args.port, baudrate=args.baudrate)
        transport_name = f'XBee @ {args.port} ({args.baudrate})'

    _header(f'holOS Hardware Test Suite — {transport_name}')

    # ── Connect ────────────────────────────────────────────────────────────
    print(f'  Connexion en cours...')
    connected = transport.connect()
    if not connected:
        print(_red(f'\n  ERREUR: Impossible de se connecter sur {args.port}'))
        print('  • Vérifier que le Teensy est allumé et branché')
        print('  • Vérifier le port (--port) et le baudrate (--baudrate)')
        return 1

    print(_green(f'  Connecté via {transport_name}'))
    time.sleep(0.3)   # give heartbeat time to start

    # ── Run tests ──────────────────────────────────────────────────────────
    runner   = HardwareTestRunner(transport, skip_interactive=args.skip_interactive)
    results  = []
    done_evt = threading.Event()
    t_start  = time.perf_counter()

    def on_progress(test_id, status):
        meta = ALL_TESTS.get(test_id, {})
        name = meta.get('name', test_id)
        if status == 'running':
            print(f'\n  ► {_bold(name)} ...', flush=True)
        elif status == 'skipped':
            print(f'\n  {_yellow("○ SKIP")}  {_bold(name)}  {_dim("(--skip-interactive)")}')
        elif status == 'stopped':
            print(f'\n  {_yellow("⏹ STOP")}  {_bold(name)}')

    def on_result(result: TestResult):
        results.append(result)
        _print_result(result)

    def on_done():
        done_evt.set()

    if args.suite == 'all':
        runner.run_all(on_progress, on_result, on_done)
    else:
        runner.run_suite(args.suite, on_progress, on_result, on_done)

    # Wait for completion (or Ctrl-C)
    try:
        while not done_evt.is_set():
            done_evt.wait(timeout=0.5)
    except KeyboardInterrupt:
        print('\n\n  [Ctrl-C] Arrêt demandé...')
        runner.stop()
        done_evt.wait(timeout=3.0)

    elapsed = time.perf_counter() - t_start
    _print_summary(results, elapsed)

    transport.disconnect()

    # Return exit code: 0 = all passed, 1 = some failures
    all_passed = all(r.passed for r in results)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
