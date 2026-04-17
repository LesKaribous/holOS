"""
shared/config.py — holOS constants, enums and POIs.
Mirrors src/config/settings.h + src/config/poi.h

Used by:
  - Jetson brain (real hardware)
  - Simulator (Windows dev)
  - Strategy (identical code in both modes)
"""

import math
from enum import Enum
from dataclasses import dataclass

# ── Field ─────────────────────────────────────────────────────────────────────
FIELD_W          = 3000     # mm
FIELD_H          = 2000     # mm
MATCH_DURATION_S = 100.0    # seconds

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_RADIUS     = 125.98   # mm  (circumradius)
WHEEL_RADIUS     = 30.0     # mm

# Angle offset between the holOS frame (0 = East, CCW+) and the firmware frame.
# Firmware already tracks and sends theta in the table/holOS frame (0 = East),
# so no data conversion is needed.  Keep at 0.0.
HW_THETA_OFFSET_DEG = 0.0

# ── Motion parameters (mirror firmware settings.h::Motion) ────────────────────
MAX_SPEED        = 2800.0   # mm/s  (firmware: 2800)
MAX_ACCEL        = 500.0    # mm/s² (firmware: 500 — trapezoidal ramp)
MAX_ROT_SPEED    = 10.0     # rad/s
MAX_ROT_ACCEL    = 30.0     # rad/s²
MIN_DISTANCE     = 20.0     # mm   (position tolerance)
MIN_ANGLE        = math.radians(2.0)  # rad

# ── Pathfinding / waypoints ───────────────────────────────────────────────────
WAYPOINT_RADIUS  = 80.0     # mm  (pass-through acceptance)

# ── Occupancy grid ────────────────────────────────────────────────────────────
GRID_W           = 20
GRID_H           = 13
GRID_CELL        = 150.0    # mm/cell

# ── Safety ────────────────────────────────────────────────────────────────────
SAFETY_CHECK_S      = 0.10   # s
SAFETY_RESUME_S     = 1.00   # s after last detection before resume
OBS_RADIUS          = 150.0  # mm  radius used when placing dynamic obstacles

# ── Transport baudrate (mirror firmware settings.h BRIDGE_BAUDRATE) ──────────
# Both USB-CDC and XBee use the same baudrate.  The firmware auto-detects
# which physical port is active (Serial vs Serial3).
BRIDGE_BAUDRATE       = 57600
USB_DIRECT_BAUDRATE   = 115200  # USB-CDC wired (high-speed direct)
XBEE_BAUDRATE         = 57600   # XBee radio link
HEARTBEAT_INTERVAL_S  = 0.5   # s between heartbeats
JETSON_TIMEOUT_S      = 2.0   # s before Teensy considers Jetson lost
CMD_TIMEOUT_MS        = 30000 # ms — max time to wait for a motion command to complete
TELEMETRY_RATE_HZ     = 5    # Hz — telemetry push rate from Jetson to Teensy

# ── Enums ─────────────────────────────────────────────────────────────────────

class Team(Enum):
    YELLOW = "yellow"
    BLUE   = "blue"


class ObjectColor(Enum):
    UNKNOWN = 0
    NONE    = 1
    RED     = 2
    GREEN   = 3
    BLUE    = 4
    YELLOW  = 5
    WHITE   = 6
    BLACK   = 7


COLOR_HEX = {
    ObjectColor.UNKNOWN: "#888888",
    ObjectColor.NONE:    "#CCCCCC",
    ObjectColor.RED:     "#E03232",
    ObjectColor.GREEN:   "#28C228",
    ObjectColor.BLUE:    "#2828E0",
    ObjectColor.YELLOW:  "#E0C000",
    ObjectColor.WHITE:   "#F0F0F0",
    ObjectColor.BLACK:   "#222222",
}

COLOR_BY_NAME = {c.name: c for c in ObjectColor}


class RobotCompass(Enum):
    A  = 0   # BAU
    AB = 1   # Hugger / front
    B  = 2   # Screen
    BC = 3   # Empty
    C  = 4   # Pull tab / tirette
    CA = 5   # Manipulator


class TableCompass(Enum):
    NORTH = 90
    SOUTH = 270
    EAST  = 0
    WEST  = 180


class BlockResult(Enum):
    SUCCESS = 0
    FAILED  = 1


class ElevatorPose(Enum):
    """Mirrors firmware enum class ElevatorPose (actuators.h).
    Integer values are sent directly in the elevator(side, pose) wire command."""
    STORE = 0
    UP    = 1
    DOWN  = 2


class ManipulatorPose(Enum):
    """Mirrors firmware enum class ManipulatorPose (actuators.h)."""
    DROP  = 0
    GRAB  = 1
    STORE = 2


# ── 2D vector ─────────────────────────────────────────────────────────────────

@dataclass
class Vec2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, o):
        return Vec2(self.x + o.x, self.y + o.y) if isinstance(o, Vec2) else Vec2(self.x + o, self.y + o)
    def __sub__(self, o):
        return Vec2(self.x - o.x, self.y - o.y) if isinstance(o, Vec2) else Vec2(self.x - o, self.y - o)
    def __mul__(self, s): return Vec2(self.x * s, self.y * s)
    def __rmul__(self, s): return self.__mul__(s)
    def __neg__(self): return Vec2(-self.x, -self.y)
    def __iter__(self): yield self.x; yield self.y
    def __repr__(self): return f"Vec2({self.x:.1f}, {self.y:.1f})"
    def __eq__(self, o):
        if not isinstance(o, Vec2): return False
        return abs(self.x - o.x) < 0.01 and abs(self.y - o.y) < 0.01
    def __hash__(self): return hash((round(self.x, 1), round(self.y, 1)))

    def mag(self):      return math.sqrt(self.x**2 + self.y**2)
    def mag_sq(self):   return self.x**2 + self.y**2
    def dot(self, o):   return self.x * o.x + self.y * o.y
    def heading(self):  return math.atan2(self.y, self.x)
    def dist(self, o):  return (self - o).mag()

    def normalized(self):
        m = self.mag()
        return Vec2(self.x / m, self.y / m) if m > 1e-9 else Vec2(0, 0)

    def rotated(self, angle_rad: float):
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        return Vec2(c * self.x - s * self.y, s * self.x + c * self.y)

    def tuple(self): return (self.x, self.y)


def polar_vec(heading_deg: float, dist: float) -> Vec2:
    """Polar → Cartesian. heading_deg=0 → East, 90 → North."""
    a = math.radians(heading_deg)
    return Vec2(math.cos(a) * dist, math.sin(a) * dist)


def angle_diff(target: float, current: float) -> float:
    """Shortest signed angle difference, in [-π, π]."""
    d = (target - current) % (2 * math.pi)
    if d > math.pi: d -= 2 * math.pi
    return d


def compass_deg(direction) -> float:
    """Get angle in degrees for a TableCompass or RobotCompass."""
    if isinstance(direction, TableCompass): return float(direction.value)
    if isinstance(direction, RobotCompass): return direction.value * 60.0
    return float(direction)


def robot_compass_offset_deg(rc: RobotCompass) -> float:
    """Offset in degrees of a robot compass side from heading=0."""
    return rc.value * 60.0


# ── Points of Interest ────────────────────────────────────────────────────────
# Mirrors sim/StrategyEditor/data/poi.h  (CDR 2025 — "Cooking" theme)

class POI:
    # ── Departure Areas ──────────────────────────────────────────────────────
    startYellow       = Vec2(300,  1700)
    startYellow_ninja = Vec2(715,  100)
    startBlue         = Vec2(2700, 1700)
    startBlue_ninja   = Vec2(2285, 100)

    # ── Thermometer ──────────────────────────────────────────────────────────
    thermometer_hot_yellow    = Vec2(200,  1840)
    thermometer_hot_blue      = Vec2(2800, 1840)
    thermometer_target_yellow = Vec2(700,  1840)
    thermometer_target_blue   = Vec2(2300, 1840)

    # ── Wait points (end-of-match holding) ─────────────────────────────────
    wait_yellow = Vec2(1150, 850)
    wait_blue   = Vec2(1850, 850)

    # ── Garde Manger (Pantry) — 10 shared points ────────────────────────────
    pantry_01 = Vec2(1250, 550)
    pantry_02 = Vec2(1750, 550)
    pantry_03 = Vec2(100,  1200)
    pantry_04 = Vec2(800,  1200)
    pantry_05 = Vec2(1500, 1200)
    pantry_06 = Vec2(2200, 1200)
    pantry_07 = Vec2(2900, 1200)
    pantry_08 = Vec2(700,  1900)
    pantry_09 = Vec2(1500, 1900)
    pantry_10 = Vec2(2300, 1900)

    # ── Frigo (Fridge) — scoring zones ───────────────────────────────────────
    FridgeYellow_01 = Vec2(1100, 280)
    FridgeYellow_02 = Vec2(1350, 225)
    FridgeBlue_01   = Vec2(1900, 280)
    FridgeBlue_02   = Vec2(1650, 225)

    # ── Stock (pickup zones) — Yellow ────────────────────────────────────────
    stockYellow_01   = Vec2(175,  800)
    stockYellow_02   = Vec2(175,  1600)
    stockYellow_03   = Vec2(1100, 1825)
    stockYellow_04   = Vec2(1150, 1200)
    stockNinjaYellow = Vec2(800,  325)

    # ── Stock (pickup zones) — Blue ──────────────────────────────────────────
    stockBlue_01   = Vec2(2825, 800)
    stockBlue_02   = Vec2(2825, 1600)
    stockBlue_03   = Vec2(1900, 1825)
    stockBlue_04   = Vec2(1850, 1200)
    stockNinjaBlue = Vec2(2200, 325)

    # ── Helpers ──────────────────────────────────────────────────────────────

    @classmethod
    def all_named(cls) -> list:
        return [(k, v) for k, v in vars(cls).items()
                if isinstance(v, Vec2) and not k.startswith('_')]

    @classmethod
    def all_stocks(cls) -> list:
        return [(k, v) for k, v in vars(cls).items()
                if isinstance(v, Vec2) and 'stock' in k.lower()]

    @classmethod
    def all_pantries(cls) -> list:
        return [(k, v) for k, v in vars(cls).items()
                if isinstance(v, Vec2) and k.startswith('pantry')]

    @classmethod
    def all_fridges(cls) -> list:
        return [(k, v) for k, v in vars(cls).items()
                if isinstance(v, Vec2) and k.startswith('Fridge')]


# ── Starting positions per team ───────────────────────────────────────────────

TEAM_START = {
    Team.YELLOW: (POI.startYellow, 0.0),           # facing East
    Team.BLUE:   (POI.startBlue,   math.pi),       # facing West
}
