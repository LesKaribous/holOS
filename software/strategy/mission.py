"""
strategy/mission.py — Planner / Mission / Step engine.
Mirrors firmware src/program/mission.h / mission.cpp

Architecture:
    Planner  — continuous loop selecting the best feasible mission
    Mission  — ordered sequence of Steps (collect → store = one mission)
    Step     — atomic action with optional feasibility check

Features:
    - Retry logic with configurable max_retries (-1 = infinite)
    - Cooldown between retries (RETRY_COOLDOWN_S)
    - Mission dependencies: require other missions to be DONE first
    - Feasibility checks at step level (e.g. zone occupied)
    - DONE / ABANDONED missions are removed from candidates
    - Planner exits early when all missions are terminal

Identical API in sim and real modes.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional

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

    Dependencies:
        Use requires=["mission_name"] to declare that this mission cannot
        start until the named missions are DONE.  Checked by the Planner
        during selection (not counted as a retry if dependency not met).

    Retries:
        max_retries = 3   → up to 3 retries after failure
        max_retries = -1  → infinite retries (never abandoned due to retries)
        max_retries = 0   → no retries (abandoned on first failure)
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
        self.requires: List[str] = []  # names of missions that must be DONE first

    def add_step(self, name: str, estimated_ms: int,
                 action: Callable[[], BlockResult],
                 feasible: Optional[Callable[[], bool]] = None) -> 'Mission':
        if len(self.steps) < self.MAX_STEPS:
            self.steps.append(Step(name, estimated_ms, action, feasible))
        return self

    def set_max_retries(self, n: int) -> 'Mission':
        """Set max retries. -1 = infinite retries."""
        self.max_retries = n
        return self

    def add_dependency(self, *mission_names: str) -> 'Mission':
        """Add dependency: this mission won't start until all named missions are DONE."""
        self.requires.extend(mission_names)
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
        """True if the mission can be retried. -1 = infinite retries."""
        if self.max_retries < 0:
            return True  # infinite
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
        self._by_name:   Dict[str, Mission] = {}
        self._safety_margin_ms = 5000

        # Stats
        self.total_score = 0
        self.missions_done = 0
        self.missions_failed = 0

    def add_mission(self, name: str, priority: int, score: int) -> Mission:
        m = Mission(name, priority, score)
        if len(self._missions) < self.MAX_MISSIONS:
            self._missions.append(m)
            self._by_name[name] = m
        return m

    def get_mission(self, name: str) -> Optional[Mission]:
        """Look up a mission by name (for dependency checks)."""
        return self._by_name.get(name)

    def set_safety_margin_ms(self, ms: int) -> 'Planner':
        self._safety_margin_ms = ms
        return self

    # ── Dependency check ────────────────────────────────────────────────────

    def _deps_satisfied(self, mission: Mission) -> bool:
        """True if all required missions are DONE."""
        for dep_name in mission.requires:
            dep = self._by_name.get(dep_name)
            if dep is None or dep.state != MissionState.DONE:
                return False
        return True

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
                # Nothing feasible right now — all terminal?
                all_terminal = all(
                    m.state in (MissionState.DONE, MissionState.ABANDONED)
                    for m in self._missions
                )
                if all_terminal:
                    self._log("[Planner] All missions terminal — exiting")
                    break

                # Time remains and missions are non-terminal → wait and retry
                # (cooldown, dependencies, zone occupied — can change)
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
                    retry_str = (f"{mission.retries}/∞" if mission.max_retries < 0
                                 else f"{mission.retries}/{mission.max_retries}")
                    self._log(f"[Planner] ✗ {mission.name} FAILED "
                              f"(retry {retry_str} "
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
            # Skip if dependencies not met
            if not self._deps_satisfied(m):
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
