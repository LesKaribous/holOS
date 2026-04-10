"""
services/actuators.py — ActuatorsService for the Jetson brain.

Wraps all actuator firmware commands exposed over the wire protocol:
    grab(side)            — open gripper wide (ManipulatorPose::GRAB)
    drop(side)            — partial release   (ManipulatorPose::DROP)
    store(side)           — close to storage  (ManipulatorPose::STORE)
    elevator(side, pose)  — move elevator to named pose (ElevatorPose int)
    raise(side)           — shorthand: elevator UP
    lower(side)           — shorthand: elevator DOWN
    servo(side, id, angle)— raw servo angle control
    pump(channel)         — start pump (1=RIGHT, 0=LEFT)
    ev(channel)           — stop pump / open electrovanne

All actuator commands go through transport.execute() (not fire()) so that:
  - The firmware has time to process the command before the next one
  - Silent serial drops are detectable (we get an 'ok' or timeout)

The firmware's _executeCommand() auto-replies 'ok' immediately after the
servo is commanded (servo motion itself is asynchronous / PWM-driven).
Round-trip latency over XBee is typically 10–50 ms, which is well within
the 1000 ms waits used in collectStock / storeStock sequences.

Side convention (mirrors RobotCompass):
    "AB" — Hugger / front manipulator
    "BC" — Banner elevator (BC group, currently unused for grab/drop)
    "CA" — Right manipulator (pump + gripper servos)
"""

import time
from transport.base import Transport
from shared.config import RobotCompass, ElevatorPose


# Timeout for actuator commands — short because they reply immediately.
_ACT_TIMEOUT_MS = 2000


class ActuatorsService:
    """
    High-level actuator service.

    Usage in strategy.py (all calls are blocking until firmware acks):
        actuators.grab(RobotCompass.AB)
        actuators.move_elevator(RobotCompass.AB, ElevatorPose.DOWN)
        actuators.store(RobotCompass.AB)
        actuators.move_elevator(RobotCompass.AB, ElevatorPose.STORE)
        actuators.pump(right=True)
        actuators.ev(right=True)
    """

    def __init__(self, transport: Transport):
        self._t = transport

    # ── Side helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _side(rc: RobotCompass) -> str:
        """Convert RobotCompass enum to the wire string expected by firmware."""
        return rc.name  # AB, BC, CA

    def _send(self, cmd: str) -> bool:
        """Send an actuator command and return True if firmware acknowledged."""
        ok, resp = self._t.execute(cmd, timeout_ms=_ACT_TIMEOUT_MS)
        if not ok:
            print(f"[Actuators] '{cmd}' FAILED — {resp}")
        return ok

    # ── Primary manipulator actions ───────────────────────────────────────────

    def grab(self, rc: RobotCompass) -> bool:
        """Open gripper wide (ManipulatorPose::GRAB)."""
        return self._send(f"grab({self._side(rc)})")

    def drop(self, rc: RobotCompass) -> bool:
        """Partial release — let object settle (ManipulatorPose::DROP)."""
        return self._send(f"drop({self._side(rc)})")

    def store(self, rc: RobotCompass) -> bool:
        """Close gripper to storage position (ManipulatorPose::STORE)."""
        return self._send(f"store({self._side(rc)})")

    # ── Elevator ──────────────────────────────────────────────────────────────

    def move_elevator(self, rc: RobotCompass, pose: ElevatorPose) -> bool:
        """Move elevator to a named pose (STORE / UP / DOWN)."""
        return self._send(f"elevator({self._side(rc)},{pose.value})")

    def move_elevator_angle(self, rc: RobotCompass, angle: int) -> bool:
        """Move elevator to a raw servo angle."""
        return self._send(f"moveElevator({self._side(rc)},{angle})")

    def raise_elevator(self, rc: RobotCompass) -> bool:
        """Raise elevator to UP position."""
        return self._send(f"raise({self._side(rc)})")

    def lower_elevator(self, rc: RobotCompass) -> bool:
        """Lower elevator to DOWN position."""
        return self._send(f"lower({self._side(rc)})")

    # ── Raw servo ─────────────────────────────────────────────────────────────

    def servo(self, rc: RobotCompass, servo_id: int, angle: int) -> bool:
        """Move a specific servo to a raw angle position."""
        return self._send(f"servo({self._side(rc)},{servo_id},{angle})")

    # ── Pump / electrovanne ───────────────────────────────────────────────────

    def pump(self, right: bool = True) -> bool:
        """Start pump. right=True → RIGHT channel (CA_RIGHT), False → LEFT (CA_LEFT)."""
        return self._send(f"pump({'1' if right else '0'})")

    def ev(self, right: bool = True) -> bool:
        """Stop pump / open electrovanne. right=True → RIGHT, False → LEFT."""
        return self._send(f"ev({'1' if right else '0'})")

    # ── Convenience: full collect sequence helpers ────────────────────────────

    def wait_ms(self, ms: int) -> None:
        """Blocking wait (mirrors firmware waitMs). Use inside action sequences."""
        time.sleep(ms / 1000.0)
