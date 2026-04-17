"""
strategy/match.py — Match strategy (HOT-RELOADABLE).
═══════════════════════════════════════════════════════════════════════════════
CDR 2025 — Thème "Cooking"

Mirrors firmware strategy.cpp — same missions, same actions, same order.

Architecture headless : ce fichier tourne IDENTIQUEMENT
  - sur le vrai Jetson (via XBeeTransport)
  - dans le simulateur (via VirtualTransport)

Les globals sont injectés par brain.py :
    motion    — MotionService   (go, turn, via, go_align, feedrate, …)
    vision    — VisionService   (query_color_sync)
    safety    — SafetyService   (enable, disable)
    chrono    — ChronoService   (time_left_s, is_running)
    occupancy — OccupancyService (is_zone_occupied)
    actuators — ActuatorsService (grab, drop, store, move_elevator, …)
    log(msg)  — console / UI log
═══════════════════════════════════════════════════════════════════════════════
"""

# ── Globals injectés ──────────────────────────────────────────────────────────
# motion, vision, safety, chrono, occupancy, actuators, log

import math
import time
from shared.config import (
    POI, Vec2, RobotCompass, TableCompass, ElevatorPose,
    BlockResult, compass_deg, polar_vec,
)
from strategy.mission import Planner


# ─────────────────────────────────────────────────────────────────────────────
#  Timing estimates (ms) — mirrors firmware Timing namespace
# ─────────────────────────────────────────────────────────────────────────────

class Timing:
    COLLECT_STOCK_A = 8000
    COLLECT_STOCK_B = 6000
    STORE_STOCK_A   = 8000
    STORE_STOCK_B   = 6000
    THERMO_SET      = 15000


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers — mirrors firmware collectStock / storeStock
# ─────────────────────────────────────────────────────────────────────────────

APPROACH_OFFSET = 350.0
GRAB_OFFSET     = 210.0
GRAB_DELAY_MS   = 1000
ZONE_CHECK_RADIUS = 450.0


def wait_ms(ms: int):
    time.sleep(ms / 1000.0)


def collect_stock(target: Vec2, tc: TableCompass, rc: RobotCompass) -> BlockResult:
    """
    Mirrors firmware collectStock().
    Approach → goAlign → grab → elevator down → store → elevator store.
    """
    tc_angle = compass_deg(tc)
    offset_vec = polar_vec(tc_angle, 1.0)  # unit direction
    approach = target - offset_vec * APPROACH_OFFSET
    grab_pos = target - offset_vec * GRAB_OFFSET

    actuators.grab(rc)
    motion.go_align(approach, rc, tc_angle)
    if not motion.was_successful():
        return BlockResult.FAILED

    motion.go_align(grab_pos, rc, tc_angle)
    if not motion.was_successful():
        return BlockResult.FAILED

    actuators.move_elevator(rc, ElevatorPose.DOWN)
    wait_ms(GRAB_DELAY_MS)
    actuators.store(rc)
    wait_ms(GRAB_DELAY_MS)
    actuators.move_elevator(rc, ElevatorPose.STORE)

    safety.enable()
    motion.set_feedrate(1.0)
    log("[collectStock] done")
    return BlockResult.SUCCESS


def store_stock(target: Vec2, tc: TableCompass, rc: RobotCompass) -> BlockResult:
    """
    Mirrors firmware storeStock().
    Approach → goAlign → drop → grab (open wide) → slow recal → retreat.
    """
    tc_angle = compass_deg(tc)
    offset_vec = polar_vec(tc_angle, 1.0)
    approach  = target - offset_vec * APPROACH_OFFSET
    grab_pos  = target - offset_vec * GRAB_OFFSET
    grab_recal = target - offset_vec * (GRAB_OFFSET - 20)

    motion.go_align(approach, rc, tc_angle)
    if not motion.was_successful():
        return BlockResult.FAILED

    motion.go_align(grab_pos, rc, tc_angle)
    if not motion.was_successful():
        return BlockResult.FAILED

    wait_ms(GRAB_DELAY_MS)
    actuators.drop(rc)     # release just enough
    wait_ms(GRAB_DELAY_MS)
    actuators.grab(rc)     # open wide

    motion.set_feedrate(0.3)
    motion.go_align(grab_recal, rc, tc_angle)
    motion.set_feedrate(1.0)
    motion.go_align(approach, rc, tc_angle)

    safety.enable()
    motion.set_feedrate(1.0)
    log("[storeStock] done")
    return BlockResult.SUCCESS


# ─────────────────────────────────────────────────────────────────────────────
#  Block actions — one per objective
# ─────────────────────────────────────────────────────────────────────────────

def block_collect_A() -> BlockResult:
    return collect_stock(POI.stockYellow_01, TableCompass.WEST, RobotCompass.AB)


def block_store_A() -> BlockResult:
    return store_stock(POI.pantry_03, TableCompass.WEST, RobotCompass.AB)


def block_collect_B() -> BlockResult:
    return collect_stock(POI.stockYellow_02, TableCompass.WEST, RobotCompass.AB)


def block_store_B() -> BlockResult:
    return store_stock(POI.pantry_04, TableCompass.EAST, RobotCompass.AB)


def block_thermo_set() -> BlockResult:
    """
    Mirrors firmware thermometer_set().
    goAlign to hot → servo drop → go target → servo store.
    """
    # goAlign to thermometer hot position, side C facing WEST
    motion.go_align(POI.thermometer_hot_yellow, RobotCompass.C,
                    compass_deg(TableCompass.WEST))
    if not motion.was_successful():
        return BlockResult.FAILED

    # Servo: GRABBER_RIGHT to DROP pose (servo id=0, pose=0)
    actuators.servo(RobotCompass.CA, 0, 0)  # DROP position

    motion.go(POI.thermometer_target_yellow.x, POI.thermometer_target_yellow.y)
    if not motion.was_successful():
        return BlockResult.FAILED

    # Servo: GRABBER_RIGHT to STORE pose (servo id=0, pose=2)
    actuators.servo(RobotCompass.CA, 0, 2)  # STORE position

    log("[thermo_set] done")
    return BlockResult.SUCCESS


# ─────────────────────────────────────────────────────────────────────────────
#  Feasibility checks — mirrors firmware isZone*Free()
# ─────────────────────────────────────────────────────────────────────────────

def is_zone_A_free() -> bool:
    occ = occupancy.is_zone_occupied(POI.stockYellow_01, ZONE_CHECK_RADIUS)
    if occ:
        log("Zone A occupied — skip")
    return not occ


def is_zone_B_free() -> bool:
    occ = occupancy.is_zone_occupied(POI.stockYellow_02, ZONE_CHECK_RADIUS)
    if occ:
        log("Zone B occupied — skip")
    return not occ


def is_zone_thermo_free() -> bool:
    occ = occupancy.is_zone_occupied(POI.thermometer_hot_yellow, ZONE_CHECK_RADIUS)
    if occ:
        log("Zone Thermo occupied — skip")
    return not occ


# ─────────────────────────────────────────────────────────────────────────────
#  Point d'entrée — appelé par brain.py au départ du match
# ─────────────────────────────────────────────────────────────────────────────

def run_mission():
    log("══════════════════════════════")
    log("  MATCH START — CDR 2025")
    log("══════════════════════════════")

    safety.enable()
    motion.set_feedrate(1.0)

    planner = Planner(chrono, log)
    planner.set_safety_margin_ms(5000)

    # ── Mission definitions — mirrors firmware match() ──────────────────────
    # Each mission = complete objective (collect → store).
    # Steps execute in sequence. If a step fails → retry later.
    # If zone occupied → skip, try another mission.

    m = planner.add_mission("stock_A", priority=10, score=150)
    m.add_step("collect_A", Timing.COLLECT_STOCK_A, block_collect_A, is_zone_A_free)
    m.add_step("store_A",   Timing.STORE_STOCK_A,   block_store_A)

    m = planner.add_mission("stock_B", priority=8, score=150)
    m.add_step("collect_B", Timing.COLLECT_STOCK_B, block_collect_B, is_zone_B_free)
    m.add_step("store_B",   Timing.STORE_STOCK_B,   block_store_B)

    m = planner.add_mission("thermo_set", priority=10, score=150)
    m.add_step("thermo_set", Timing.THERMO_SET, block_thermo_set, is_zone_thermo_free)

    # ── Run planner ─────────────────────────────────────────────────────────
    planner.run()

    # ── End-of-match — go to wait point ─────────────────────────────────────
    log("[Match] Going to wait point")
    # TODO: use team color to pick wait_yellow vs wait_blue
    motion.go(POI.wait_yellow.x, POI.wait_yellow.y)

    # Wait until ~4.5s before end
    time_left = chrono.time_left_s()
    if time_left > 5.0:
        wait_ms(int((time_left - 4.5) * 1000))

    log(f"══ Match finished — {planner.total_score} pts ══")
