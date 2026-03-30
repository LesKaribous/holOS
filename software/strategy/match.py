"""
strategy/match.py — Match strategy (HOT-RELOADABLE).
═══════════════════════════════════════════════════════════════════════════════
CDR 2025 — Thème "Cooking"

Architecture headless : ce fichier tourne IDENTIQUEMENT
  - sur le vrai Jetson (via XBeeTransport)
  - dans le simulateur (via VirtualTransport)

Les globals sont injectés par brain.py :
    motion    — MotionService   (go, turn, via, go_align, feedrate, …)
    vision    — VisionService   (query_color_sync)
    safety    — SafetyService   (enable, disable)
    chrono    — ChronoService   (time_left_s, is_running)
    occupancy — OccupancyService (is_zone_occupied)
    log(msg)  — console / UI log

Rappel API motion :
    motion.go(x, y)                        → bloquant jusqu'à arrivée
    motion.via(x1,y1).go(x2,y2)            → passage de via-point
    motion.cancel_on_collide().go(x, y)    → annule si obstacle
    motion.feedrate(0.7).go(x, y)          → vitesse réduite
    motion.turn(angle_deg)                 → rotation absolue
    motion.was_successful()  → bool
═══════════════════════════════════════════════════════════════════════════════
"""

# ── Globals injectés ──────────────────────────────────────────────────────────
# motion, vision, safety, chrono, occupancy, log

import time
from shared.config import (
    POI, Vec2, ObjectColor, RobotCompass, BlockResult,
)
from strategy.mission import Mission, Block


# ─────────────────────────────────────────────────────────────────────────────
#  Utilitaires
# ─────────────────────────────────────────────────────────────────────────────

def wait_ms(ms: int):
    time.sleep(ms / 1000.0)


def is_food(color: ObjectColor) -> bool:
    """On accepte toute couleur identifiable (l'objet est là)."""
    return color not in (ObjectColor.UNKNOWN, ObjectColor.NONE)


def move_safe(x: float, y: float, feedrate: float = 1.0) -> bool:
    """Déplacement avec cancel_on_collide. Retourne True si succès."""
    motion.cancel_on_collide().feedrate(feedrate).go(x, y)
    return motion.was_successful()


# ─────────────────────────────────────────────────────────────────────────────
#  Blocs de mission — Yellow team
# ─────────────────────────────────────────────────────────────────────────────

def block_stock_yellow_01() -> BlockResult:
    """
    Ramasser à stockYellow_01 (mur gauche, bas).
    Approche frontale depuis la zone de départ.
    """
    log("→ stockYellow_01")
    target = POI.stockYellow_01
    approach = Vec2(target.x + 300, target.y)

    if occupancy.is_zone_occupied(target, 400):
        log("  Zone occupée — skip")
        return BlockResult.FAILED

    if not move_safe(approach.x, approach.y):
        return BlockResult.FAILED

    if not move_safe(target.x + 80, target.y, feedrate=0.55):
        return BlockResult.FAILED

    log("  stockYellow_01 ✓")
    wait_ms(400)
    return BlockResult.SUCCESS


def block_stock_yellow_02() -> BlockResult:
    """
    Ramasser à stockYellow_02 (mur gauche, haut).
    """
    log("→ stockYellow_02")
    target = POI.stockYellow_02
    approach = Vec2(target.x + 350, target.y)

    if occupancy.is_zone_occupied(target, 400):
        log("  Zone occupée — skip")
        return BlockResult.FAILED

    # Via intermédiaire pour éviter les obstacles du centre
    motion.cancel_on_collide().via(300, 1200).go(approach.x, approach.y)
    if not motion.was_successful():
        return BlockResult.FAILED

    if not move_safe(target.x + 80, target.y, feedrate=0.55):
        return BlockResult.FAILED

    log("  stockYellow_02 ✓")
    wait_ms(400)
    return BlockResult.SUCCESS


def block_stock_yellow_04() -> BlockResult:
    """
    Ramasser à stockYellow_04 (centre, zone partagée avec le garde-manger).
    """
    log("→ stockYellow_04")
    target = POI.stockYellow_04
    approach = Vec2(target.x, target.y - 350)

    if occupancy.is_zone_occupied(target, 450):
        log("  Zone occupée — skip")
        return BlockResult.FAILED

    if not move_safe(approach.x, approach.y):
        return BlockResult.FAILED

    if not move_safe(target.x, target.y - 80, feedrate=0.5):
        return BlockResult.FAILED

    log("  stockYellow_04 ✓")
    wait_ms(400)
    return BlockResult.SUCCESS


def block_deposit_pantry_03() -> BlockResult:
    """
    Déposer dans le garde-manger pantry_03 (mur gauche, centre).
    Point partagé — priorité si c'est le plus proche de notre stock.
    """
    log("→ Dépôt pantry_03")
    target = POI.pantry_03
    approach = Vec2(target.x + 400, target.y)

    if not move_safe(approach.x, approach.y, feedrate=0.9):
        return BlockResult.FAILED

    if not move_safe(target.x + 120, target.y, feedrate=0.5):
        return BlockResult.FAILED

    log("  Dépôt pantry_03 ✓")
    wait_ms(300)
    return BlockResult.SUCCESS


def block_deposit_pantry_04() -> BlockResult:
    """
    Déposer dans le garde-manger pantry_04 (côté jaune).
    """
    log("→ Dépôt pantry_04")
    target = POI.pantry_04

    if not move_safe(target.x, target.y - 350, feedrate=0.9):
        return BlockResult.FAILED

    if not move_safe(target.x, target.y - 120, feedrate=0.5):
        return BlockResult.FAILED

    log("  Dépôt pantry_04 ✓")
    wait_ms(300)
    return BlockResult.SUCCESS


def block_deposit_fridge() -> BlockResult:
    """
    Déposer dans le frigo jaune FridgeYellow_01 (zone de score, haut).
    Approche prudente car zone de bord de table.
    """
    log("→ Dépôt FridgeYellow_01")
    target = POI.FridgeYellow_01
    approach = Vec2(target.x, target.y - 300)

    if not move_safe(approach.x, approach.y, feedrate=0.8):
        return BlockResult.FAILED

    if not move_safe(target.x, target.y - 100, feedrate=0.4):
        return BlockResult.FAILED

    log("  Frigo déposé ✓")
    wait_ms(500)
    return BlockResult.SUCCESS


def block_return_to_base() -> BlockResult:
    """Retour à la zone de départ avant fin de match."""
    log("→ Retour base")
    motion.feedrate(1.0).go(POI.startYellow.x, POI.startYellow.y)
    if motion.was_successful():
        log("  Base ✓")
        return BlockResult.SUCCESS
    return BlockResult.FAILED


# ─────────────────────────────────────────────────────────────────────────────
#  Feasibility checks
# ─────────────────────────────────────────────────────────────────────────────

def stock_01_ok() -> bool: return not occupancy.is_zone_occupied(POI.stockYellow_01, 400)
def stock_02_ok() -> bool: return not occupancy.is_zone_occupied(POI.stockYellow_02, 400)
def stock_04_ok() -> bool: return not occupancy.is_zone_occupied(POI.stockYellow_04, 450)
def pantry_03_ok() -> bool: return not occupancy.is_zone_occupied(POI.pantry_03, 350)
def pantry_04_ok() -> bool: return not occupancy.is_zone_occupied(POI.pantry_04, 350)


# ─────────────────────────────────────────────────────────────────────────────
#  Point d'entrée — appelé par brain.py au départ du match
# ─────────────────────────────────────────────────────────────────────────────

def run_mission():
    log("══════════════════════════════")
    log("  MATCH START — CDR 2025")
    log("══════════════════════════════")

    safety.enable()
    motion.set_feedrate(1.0)

    m = Mission(chrono, log)
    m.set_mode(Mission.PRIORITY)
    m.set_safety_margin_ms(6000)   # garantir retour base les 6 dernières secondes

    # ── Collecte ──────────────────────────────────────────────────────────────
    m.add(Block(
        name     = "stock_yellow_01",
        priority = 10,
        score    = 40,
        time_ms  = 7000,
        action   = block_stock_yellow_01,
        feasible = stock_01_ok,
    ))
    m.add(Block(
        name     = "stock_yellow_04",
        priority = 9,
        score    = 40,
        time_ms  = 8000,
        action   = block_stock_yellow_04,
        feasible = stock_04_ok,
    ))
    m.add(Block(
        name     = "stock_yellow_02",
        priority = 7,
        score    = 40,
        time_ms  = 9000,
        action   = block_stock_yellow_02,
        feasible = stock_02_ok,
    ))

    # ── Dépôts ────────────────────────────────────────────────────────────────
    m.add(Block(
        name     = "deposit_pantry_03",
        priority = 8,
        score    = 60,
        time_ms  = 6000,
        action   = block_deposit_pantry_03,
        feasible = pantry_03_ok,
    ))
    m.add(Block(
        name     = "deposit_pantry_04",
        priority = 6,
        score    = 50,
        time_ms  = 6000,
        action   = block_deposit_pantry_04,
        feasible = pantry_04_ok,
    ))
    m.add(Block(
        name     = "deposit_fridge",
        priority = 5,
        score    = 80,
        time_ms  = 7000,
        action   = block_deposit_fridge,
    ))

    # ── Retour ────────────────────────────────────────────────────────────────
    m.add(Block(
        name     = "return_to_base",
        priority = 1,
        score    = 10,
        time_ms  = 5000,
        action   = block_return_to_base,
    ))

    m.run()
    log(f"══ Fin mission — {m.total_score} pts ══")
