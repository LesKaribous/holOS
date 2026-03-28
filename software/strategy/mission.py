"""
strategy/mission.py — Mission planner and Block execution engine.
Mirrors src/program/mission.h

Identical API in sim and real modes.
"""

import time
from dataclasses import dataclass, field
from typing import Callable, Optional
from shared.config import BlockResult


@dataclass
class Block:
    """
    A single atomic mission action.

    name        : display name
    priority    : higher = executed first in PRIORITY mode
    score       : points awarded on SUCCESS
    time_ms     : estimated execution time (ms) — used for budget check
    action      : callable → BlockResult
    feasible    : optional callable → bool (returns False to skip block)
    """
    name:      str
    priority:  int
    score:     int
    time_ms:   int
    action:    Callable[[], BlockResult]
    feasible:  Optional[Callable[[], bool]] = None

    # Internal tracking
    result:     Optional[BlockResult] = field(default=None, repr=False)
    elapsed_ms: int                   = field(default=0,    repr=False)


class Mission:
    """
    Executes a list of Blocks in priority or score-per-time order,
    respecting the match time budget.

    Usage in match.py:
        m = Mission(chrono, log)
        m.set_mode(Mission.PRIORITY)
        m.set_safety_margin_ms(5000)
        m.add(Block("collect_A", 10, 150, 8000, block_collect_A))
        m.run()
    """

    PRIORITY = "PRIORITY"
    SCORE    = "SCORE"

    def __init__(self, chrono, log: Optional[Callable[[str], None]] = None):
        self._chrono       = chrono
        self._blocks:      list[Block] = []
        self._mode:        str  = self.PRIORITY
        self._margin_ms:   int  = 3000
        self._log = log or print

        self.total_score     = 0
        self.potential_score = 0
        self.attempted       = 0
        self.succeeded       = 0
        self.failed          = 0
        self.skipped         = 0

    def add(self, block: Block) -> 'Mission':
        self._blocks.append(block)
        return self

    def set_mode(self, mode: str) -> 'Mission':
        assert mode in (self.PRIORITY, self.SCORE)
        self._mode = mode
        return self

    def set_safety_margin_ms(self, ms: int) -> 'Mission':
        self._margin_ms = ms
        return self

    def run(self):
        remaining = list(self._blocks)
        self.potential_score = sum(b.score for b in remaining)

        while remaining:
            time_left_ms = int(self._chrono.time_left_s() * 1000)
            eligible = [
                b for b in remaining
                if (b.feasible is None or b.feasible())
                and (b.time_ms + self._margin_ms) <= time_left_ms
            ]

            if not eligible:
                self._log(f"[Mission] No eligible blocks — stopping "
                          f"(time left: {time_left_ms}ms)")
                break

            block = self._select(eligible)
            remaining.remove(block)

            self._log(f"[Mission] → {block.name} "
                      f"(prio={block.priority} score={block.score} ~{block.time_ms}ms)")

            t_start = time.time()
            try:
                result = block.action()
            except Exception as e:
                self._log(f"[Mission] ✗ {block.name} EXCEPTION: {e}")
                result = BlockResult.FAILED

            block.elapsed_ms = int((time.time() - t_start) * 1000)
            block.result     = result
            self.attempted  += 1

            if result == BlockResult.SUCCESS:
                self.succeeded   += 1
                self.total_score += block.score
                self._log(f"[Mission] ✓ {block.name} SUCCESS "
                          f"+{block.score}pts in {block.elapsed_ms}ms")
            else:
                self.failed += 1
                self._log(f"[Mission] ✗ {block.name} FAILED "
                          f"in {block.elapsed_ms}ms")

        self.skipped = len([b for b in self._blocks if b.result is None])
        self._log(f"[Mission] Done — score={self.total_score} "
                  f"({self.succeeded}/{self.attempted} ok, "
                  f"{self.skipped} skipped)")

    def _select(self, eligible: list[Block]) -> Block:
        if self._mode == self.PRIORITY:
            return max(eligible, key=lambda b: b.priority)
        else:
            return max(eligible, key=lambda b: b.score / max(b.time_ms, 1))
