"""
strategy/mission.py — Planner / Mission / Step engine.
Mirrors firmware src/program/mission.h / mission.cpp

Architecture:
    Planner  — continuous loop selecting the best feasible mission
    Mission  — ordered sequence of Steps (collect → store = one mission)
    Step     — atomic action with optional feasibility check

Retry logic:
    - FAILED missions get a cooldown (RETRY_COOLDOWN_S) before re-attempt
    - Feasibility failures do NOT count as retries
    - Max retries per mission (DEFAULT_MAX_RETRIES)
    - DONE / ABANDONED missions are removed from candidates

Identical API in sim and real modes.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional

from shared.config import BlockResult


class MissionState(Enum):
    PENDING   = 0
    DONE      = 1
    FAILED    = 2
    ABANDONED = 3


# ── Step ─────────────────────────────────────────────────────────────────────

@dataclass
class Step:
    """Atomic action within a Mission."""
    name:         str
    estimated_ms: int  = 0
    action:       Optional[Callable[[], BlockResult]] = None
    feasible:     Optional[Callable[[], bool]]        = None


# ── Mission ──────────────────────────────────────────────────────────────────

class Mission:
    """
    Ordered sequence of Steps.  If any step fails, the mission fails
    and may be retried later (resuming from the failed step).
    """

    MAX_STEPS          = 8
    DEFAULT_MAX_RETRIES = 3
    RETRY_COOLDOWN_S   = 3.0

    def __init__(self, name: str, priority: int, score: int):
        self.name       = name
        self.priority   = priority
        self.score      = score
        self.steps:     List[Step] = []
        self.state      = MissionState.PENDING
        self.retries    = 0
        self.max_retries = self.DEFAULT_MAX_RETRIES
        self.current_step = 0
        self._last_attempt_t: float = 0.0

    def add_step(self, name: str, estimated_ms: int,
                 action: Callable[[], BlockResult],
                 feasible: Optional[Callable[[], bool]] = None) -> 'Mission':
        if len(self.steps) < self.MAX_STEPS:
            self.steps.append(Step(name, estimated_ms, action, feasible))
        return self

    def set_max_retries(self, n: int) -> 'Mission':
        self.max_retries = n
        return self

    def total_estimated_ms(self) -> int:
        """Sum of remaining step estimates (from current_step onward)."""
        return sum(s.estimated_ms for s in self.steps[self.current_step:])

    def is_feasible(self) -> bool:
        """Check feasibility of the current step (if it has a check)."""
        if self.current_step >= len(self.steps):
            return False
        step = self.steps[self.current_step]
        if step.feasible is not None:
            return step.feasible()
        return True

    def is_on_cooldown(self) -> bool:
        if self.state != MissionState.FAILED:
            return False
        return (time.time() - self._last_attempt_t) < self.RETRY_COOLDOWN_S

    def can_retry(self) -> bool:
        return self.retries < self.max_retries

    def mark_failed(self) -> None:
        self.state = MissionState.FAILED
        self._last_attempt_t = time.time()

    def reset_for_retry(self) -> None:
        """Prepare for retry — keep current_step so we resume from failure."""
        self.state = MissionState.PENDING


# ── Planner ──────────────────────────────────────────────────────────────────

class Planner:
    """
    Continuous loop that selects and executes the best feasible mission.
    Mirrors firmware Planner::run().
    """

    MAX_MISSIONS    = 16
    IDLE_WAIT_S     = 0.5

    def __init__(self, chrono, log: Optional[Callable[[str], None]] = None,
                 stop_check: Optional[Callable[[], bool]] = None):
        """
        chrono     — object with .time_left_s() method
        log        — logging function
        stop_check — callable returning True when match is stopped
                     (mirrors os.getState() == OS::STOPPED)
        """
        self._chrono     = chrono
        self._log        = log or print
        self._stop_check = stop_check or (lambda: False)
        self._missions:  List[Mission] = []
        self._safety_margin_ms = 5000

        # Stats
        self.total_score = 0
        self.missions_done = 0
        self.missions_failed = 0

    def add_mission(self, name: str, priority: int, score: int) -> Mission:
        m = Mission(name, priority, score)
        if len(self._missions) < self.MAX_MISSIONS:
            self._missions.append(m)
        return m

    def set_safety_margin_ms(self, ms: int) -> 'Planner':
        self._safety_margin_ms = ms
        return self

    # ── Core loop ────────────────────────────────────────────────────────────

    def run(self) -> None:
        """Main planner loop — runs until time exhausted or match stopped."""
        self._log("[Planner] Started")

        while not self._stop_check():
            time_left_ms = int(self._chrono.time_left_s() * 1000)
            if time_left_ms <= self._safety_margin_ms:
                self._log(f"[Planner] Time budget exhausted "
                          f"({time_left_ms}ms left, margin={self._safety_margin_ms}ms)")
                break

            mission = self._select_next(time_left_ms)
            if mission is None:
                # No eligible mission — wait and retry
                all_terminal = all(
                    m.state in (MissionState.DONE, MissionState.ABANDONED)
                    for m in self._missions
                )
                if all_terminal:
                    self._log("[Planner] All missions terminal — exiting")
                    break
                time.sleep(self.IDLE_WAIT_S)
                continue

            self._log(f"[Planner] → {mission.name} "
                      f"(prio={mission.priority} score={mission.score} "
                      f"step={mission.current_step}/{len(mission.steps)})")

            success = self._execute_mission(mission)

            if success:
                mission.state = MissionState.DONE
                self.total_score += mission.score
                self.missions_done += 1
                self._log(f"[Planner] ✓ {mission.name} DONE +{mission.score}pts")
            else:
                if self._stop_check():
                    mission.state = MissionState.ABANDONED
                    self._log(f"[Planner] ⊘ {mission.name} ABANDONED (match stopped)")
                    break
                mission.retries += 1
                mission.mark_failed()
                if not mission.can_retry():
                    mission.state = MissionState.ABANDONED
                    self.missions_failed += 1
                    self._log(f"[Planner] ✗ {mission.name} ABANDONED "
                              f"(max retries {mission.max_retries})")
                else:
                    self._log(f"[Planner] ✗ {mission.name} FAILED "
                              f"(retry {mission.retries}/{mission.max_retries} "
                              f"in {Mission.RETRY_COOLDOWN_S:.0f}s)")

        self._log(f"[Planner] Finished — score={self.total_score} "
                  f"done={self.missions_done} failed={self.missions_failed}")

    # ── Selection ────────────────────────────────────────────────────────────

    def _select_next(self, time_left_ms: int) -> Optional[Mission]:
        """Pick the highest-priority feasible mission."""
        best: Optional[Mission] = None
        best_prio = -1

        for m in self._missions:
            # Skip terminal states
            if m.state in (MissionState.DONE, MissionState.ABANDONED):
                continue
            # Skip if on cooldown
            if m.is_on_cooldown():
                continue
            # Skip if not enough time
            if m.total_estimated_ms() + self._safety_margin_ms > time_left_ms:
                continue
            # Check feasibility (current step)
            if not m.is_feasible():
                continue
            # Best priority wins
            if m.priority > best_prio:
                best = m
                best_prio = m.priority

        return best

    # ── Execution ────────────────────────────────────────────────────────────

    def _execute_mission(self, mission: Mission) -> bool:
        """Execute remaining steps of a mission. Returns True if all succeed."""
        mission.state = MissionState.PENDING  # clear FAILED state for retry

        while mission.current_step < len(mission.steps):
            # Check match stopped between steps
            if self._stop_check():
                return False

            step = mission.steps[mission.current_step]

            # Step-level feasibility check
            if step.feasible is not None and not step.feasible():
                self._log(f"[Planner]   step '{step.name}' not feasible — mission fails")
                return False

            self._log(f"[Planner]   step {mission.current_step + 1}/"
                      f"{len(mission.steps)}: {step.name}")

            try:
                result = step.action()
            except Exception as e:
                self._log(f"[Planner]   step '{step.name}' EXCEPTION: {e}")
                result = BlockResult.FAILED

            if result != BlockResult.SUCCESS:
                self._log(f"[Planner]   step '{step.name}' FAILED")
                return False

            mission.current_step += 1

        return True
