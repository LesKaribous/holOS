"""
sim/physics.py — Robot physics model (holonomic, PID velocity control).

Mirrors the firmware PositionController as closely as possible:
  - Sub-stepped at 500Hz (PID_INTERVAL = 2ms) to match firmware timing
  - PID with derivative (Kp/Kd) for XY and rotation
  - Linear acceleration ramp (MAX_ACCEL per axis)
  - Velocity attenuation near target (0.01 blend factor)
  - Snap-to-zero when error < threshold and velocity < threshold
  - Completion requires velocity ≈ 0

OTOS sensor simulation (updated at 100Hz inside sub-steps):
  - Gaussian noise on position and velocity readings
  - Linear/angular scale calibration (default 0.9714 / 1.0)
  - Tracking loss above 2500 mm/s
  - PID sees the *measured* position/velocity, not the true state
  - Between OTOS updates, readings are stale (derivative sees 0 change)

Reference:
  firmware/teensy41/src/services/motion/controller/positionController.cpp
  firmware/teensy41/src/services/localisation/localisation.cpp
  firmware/teensy41/src/config/settings.h  (PID_INTERVAL = 2000µs)
"""

import math
import random
from shared.config import (
    FIELD_W, FIELD_H, ROBOT_RADIUS,
    MAX_SPEED, MAX_ACCEL, MAX_ROT_SPEED, MAX_ROT_ACCEL,
    MIN_DISTANCE, MIN_ANGLE,
    Vec2, angle_diff, Team, TEAM_START, POI,
)


# ── Constants ─────────────────────────────────────────────────────────────────

SUB_DT        = 0.002    # 2ms — matches firmware PID_INTERVAL (500Hz)
OTOS_PERIOD_S = 0.010    # 10ms — OTOS I2C refresh rate (100Hz)


# ── OTOS Sensor Model ────────────────────────────────────────────────────────

class OTOSSensor:
    """
    Simulates the SparkFun Qwiic OTOS optical tracking sensor.

    Key behaviors:
      - Updates at 100Hz (every 10ms). Between updates, readings are stale.
      - Applies linear/angular scale calibration (systematic error).
      - Adds Gaussian measurement noise on each update.
      - Loses tracking above 2500 mm/s.

    The firmware PID runs at 500Hz but OTOS only updates at 100Hz,
    meaning 4 out of 5 PID cycles see the same reading (derivative = 0).
    This natural filtering is what makes Kd=100 stable on real hardware.
    """

    # ── Calibration (mirrors firmware calibration.h defaults) ─────────────
    LINEAR_SCALE  = 0.9714   # OTOS reads ~2.8% short on distance
    ANGULAR_SCALE = 1.0      # rotation is typically accurate

    # ── Noise standard deviations (conservative, OTOS has good filtering) ─
    POS_NOISE_MM     = 0.08   # mm per reading (~80µm, realistic for optical)
    VEL_NOISE_MM_S   = 3.0    # mm/s per reading
    THETA_NOISE_RAD  = 0.0003 # rad per reading (~0.017°)
    VTHETA_NOISE_RAD = 0.005  # rad/s per reading

    # ── Tracking limits ───────────────────────────────────────────────────
    MAX_TRACKING_SPEED = 2500.0  # mm/s — above this, OTOS loses tracking

    def __init__(self):
        # Accumulated OTOS position (integrates with scale error + noise)
        self._otos_pos   = Vec2(0, 0)
        self._otos_theta = 0.0
        self._otos_vel   = Vec2(0, 0)
        self._otos_vtheta = 0.0

        # Previous true position (for computing OTOS deltas)
        self._prev_true_pos   = Vec2(0, 0)
        self._prev_true_theta = 0.0

        # Update timer (only publish new readings at 100Hz)
        self._update_timer = 0.0

        # Tracking state
        self._tracking_lost = False

    def reset(self, pos: Vec2, theta: float):
        """Reset sensor state (equivalent to otos.resetTracking())."""
        self._otos_pos   = Vec2(pos.x, pos.y)
        self._otos_theta = theta
        self._otos_vel   = Vec2(0, 0)
        self._otos_vtheta = 0.0
        self._prev_true_pos   = Vec2(pos.x, pos.y)
        self._prev_true_theta = theta
        self._update_timer = 0.0
        self._tracking_lost = False

    def tick(self, true_pos: Vec2, true_theta: float,
             true_vel: Vec2, true_vtheta: float, dt: float):
        """
        Advance OTOS time. Only publishes new readings at 100Hz.
        Between updates, cached readings remain stale.
        """
        self._update_timer += dt
        if self._update_timer < OTOS_PERIOD_S:
            return  # readings stay stale — PID derivative sees 0 change

        self._update_timer -= OTOS_PERIOD_S
        self._publish(true_pos, true_theta, true_vel, true_vtheta)

    def _publish(self, true_pos: Vec2, true_theta: float,
                 true_vel: Vec2, true_vtheta: float):
        """Internal: compute new OTOS readings from true state."""
        speed = true_vel.mag()

        # ── Tracking loss check ───────────────────────────────────────────
        if speed > self.MAX_TRACKING_SPEED:
            self._tracking_lost = True
            self._otos_vel   = Vec2(0, 0)
            self._otos_vtheta = 0.0
            self._prev_true_pos   = Vec2(true_pos.x, true_pos.y)
            self._prev_true_theta = true_theta
            return

        if self._tracking_lost:
            # Tracking recovered — snap to current true pos
            self._otos_pos   = Vec2(true_pos.x, true_pos.y)
            self._otos_theta = true_theta
            self._tracking_lost = False

        # ── Compute true deltas ───────────────────────────────────────────
        dx     = true_pos.x - self._prev_true_pos.x
        dy     = true_pos.y - self._prev_true_pos.y
        dtheta = angle_diff(true_theta, self._prev_true_theta)

        self._prev_true_pos   = Vec2(true_pos.x, true_pos.y)
        self._prev_true_theta = true_theta

        # ── Apply scale calibration (systematic error) ────────────────────
        dx     *= self.LINEAR_SCALE
        dy     *= self.LINEAR_SCALE
        dtheta *= self.ANGULAR_SCALE

        # ── Add noise ─────────────────────────────────────────────────────
        dx     += random.gauss(0, self.POS_NOISE_MM)
        dy     += random.gauss(0, self.POS_NOISE_MM)
        dtheta += random.gauss(0, self.THETA_NOISE_RAD)

        # ── Accumulate OTOS position ──────────────────────────────────────
        self._otos_pos.x += dx
        self._otos_pos.y += dy
        self._otos_theta  = (self._otos_theta + dtheta) % (2 * math.pi)

        # ── Velocity (OTOS provides fused velocity from internal IMU) ─────
        self._otos_vel = Vec2(
            true_vel.x * self.LINEAR_SCALE  + random.gauss(0, self.VEL_NOISE_MM_S),
            true_vel.y * self.LINEAR_SCALE  + random.gauss(0, self.VEL_NOISE_MM_S),
        )
        self._otos_vtheta = (
            true_vtheta * self.ANGULAR_SCALE + random.gauss(0, self.VTHETA_NOISE_RAD)
        )

    @property
    def position(self) -> Vec2:
        return self._otos_pos

    @property
    def theta(self) -> float:
        return self._otos_theta

    @property
    def velocity(self) -> Vec2:
        return self._otos_vel

    @property
    def vtheta(self) -> float:
        return self._otos_vtheta

    @property
    def tracking_lost(self) -> bool:
        return self._tracking_lost


# ── PID Controller ────────────────────────────────────────────────────────────

class _PID:
    """Minimal PID with anti-windup (mirrors firmware pid.h)."""
    __slots__ = ('kp', 'ki', 'kd', '_integral', '_prev_error', '_init')

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0
        self._init = True

    def compute(self, error: float, dt: float, saturated: bool = False) -> float:
        if not saturated and self.ki != 0.0:
            self._integral += error * dt

        # Avoid derivative kick on first call (firmware pid.h)
        if self._init:
            self._init = False
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._init = True


# ── Robot Physics ─────────────────────────────────────────────────────────────

class RobotPhysics:
    """
    Holonomic robot model faithful to the firmware PositionController.

    The update() method sub-steps internally at 500Hz (SUB_DT = 2ms) to match
    the firmware PID rate. OTOS readings update at 100Hz (every 10ms), so
    4 out of 5 sub-steps see stale readings (derivative = 0). This is what
    makes Kd=100 stable — exactly like on real hardware.

    Two coordinate frames:
      - True state (pos, theta): ground truth for physics + rendering
      - OTOS state (otos.position, otos.theta): what the PID sees
    """

    # PID gains — mirror firmware positionController.cpp:23-25
    PID_KP_POS   = 4.0
    PID_KI_POS   = 0.0
    PID_KD_POS   = 100.0
    PID_KP_ROT   = 10.0
    PID_KI_ROT   = 0.0
    PID_KD_ROT   = 70.0

    # Velocity attenuation factor (firmware positionController.cpp:134)
    ATTEN_FACTOR = 0.01

    # Snap-to-zero thresholds (firmware positionController.cpp:138-140)
    SNAP_VEL_XY  = 20.0    # mm/s
    SNAP_VEL_ROT = 0.1     # rad/s

    TRAIL_DIST   = 15.0
    TRAIL_MAX    = 300

    def __init__(self):
        # ── True state (ground truth) ─────────────────────────────────────
        self.pos     = Vec2(POI.startYellow.x, POI.startYellow.y)
        self.theta   = 0.0
        self.vel     = Vec2(0, 0)
        self.vtheta  = 0.0

        # ── OTOS sensor ──────────────────────────────────────────────────
        self.otos = OTOSSensor()
        self.otos.reset(self.pos, self.theta)

        # ── Target velocity (ramped, = "target_velocity" in firmware) ─────
        self._tv     = Vec2(0, 0)
        self._tvr    = 0.0

        # ── Motion targets ────────────────────────────────────────────────
        self.target_pos:   Vec2  | None = None
        self.target_theta: float | None = None

        self.feedrate    = 1.0
        self.is_rotating = False
        self.collided    = False
        self.trail: list[tuple[float, float]] = []
        self.stall_counter = 0

        # ── PID controllers ───────────────────────────────────────────────
        self._pid_vx  = _PID(self.PID_KP_POS, self.PID_KI_POS, self.PID_KD_POS)
        self._pid_vy  = _PID(self.PID_KP_POS, self.PID_KI_POS, self.PID_KD_POS)
        self._pid_vr  = _PID(self.PID_KP_ROT, self.PID_KI_ROT, self.PID_KD_ROT)

        # Saturation flags
        self._sat_x = False
        self._sat_y = False
        self._sat_r = False

    def reset(self):
        self.pos     = Vec2(POI.startYellow.x, POI.startYellow.y)
        self.theta   = 0.0
        self.vel     = Vec2(0, 0)
        self.vtheta  = 0.0
        self._tv     = Vec2(0, 0)
        self._tvr    = 0.0
        self.target_pos   = None
        self.target_theta = None
        self.feedrate    = 1.0
        self.is_rotating = False
        self.collided    = False
        self.stall_counter = 0
        self.trail.clear()
        self._pid_vx.reset()
        self._pid_vy.reset()
        self._pid_vr.reset()
        self._sat_x = self._sat_y = self._sat_r = False
        self.otos.reset(self.pos, self.theta)

    def reset_to_start(self, team: str):
        team_enum = Team(team)
        start_pos, start_theta = TEAM_START[team_enum]
        self.pos    = Vec2(start_pos.x, start_pos.y)
        self.theta  = start_theta
        self.vel    = Vec2(0, 0)
        self.vtheta = 0.0
        self._tv    = Vec2(0, 0)
        self._tvr   = 0.0
        self.target_pos   = None
        self.target_theta = None
        self.is_rotating  = False
        self.collided     = False
        self.stall_counter = 0
        self.trail.clear()
        self._pid_vx.reset()
        self._pid_vy.reset()
        self._pid_vr.reset()
        self._sat_x = self._sat_y = self._sat_r = False
        self.otos.reset(self.pos, self.theta)

    def update(self, dt: float, occupancy):
        """
        Advance physics by dt seconds.
        Internally sub-steps at 500Hz to match firmware PID timing.
        OTOS sensor updates at 100Hz within the sub-steps.
        """
        remaining = dt
        while remaining > 1e-6:
            step = min(remaining, SUB_DT)
            self._physics_step(step, occupancy)
            remaining -= step

        # Trail (at render rate, not sub-step rate)
        if not self.trail or Vec2(*self.trail[-1]).dist(self.pos) > self.TRAIL_DIST:
            self.trail.append((self.pos.x, self.pos.y))
            if len(self.trail) > self.TRAIL_MAX:
                self.trail.pop(0)

    def _physics_step(self, dt: float, occupancy):
        """
        Single sub-step (~2ms). Mirrors firmware PositionController::onUpdate().
        """
        # ── Update OTOS sensor (publishes at 100Hz internally) ────────────
        self.otos.tick(self.pos, self.theta, self.vel, self.vtheta, dt)

        # ── Read measured state (what the firmware PID sees) ──────────────
        meas_pos    = self.otos.position
        meas_theta  = self.otos.theta
        meas_vel    = self.otos.velocity
        meas_vtheta = self.otos.vtheta

        # ── Effective feedrate ────────────────────────────────────────────
        eff_feed = self.feedrate * 0.5 if self.is_rotating else self.feedrate
        v_max    = MAX_SPEED     * eff_feed
        vr_max   = MAX_ROT_SPEED * eff_feed

        # ── Step 1: PID → desired velocity (uses MEASURED position) ───────
        if self.target_pos is not None:
            err_x = self.target_pos.x - meas_pos.x
            err_y = self.target_pos.y - meas_pos.y
            des_vx = max(-v_max, min(v_max, self._pid_vx.compute(err_x, dt, self._sat_x)))
            des_vy = max(-v_max, min(v_max, self._pid_vy.compute(err_y, dt, self._sat_y)))
            self._sat_x = (abs(des_vx) >= v_max)
            self._sat_y = (abs(des_vy) >= v_max)
        else:
            des_vx = 0.0
            des_vy = 0.0
            err_x  = 0.0
            err_y  = 0.0

        if self.target_theta is not None:
            err_r = angle_diff(self.target_theta, meas_theta)
            des_vr = max(-vr_max, min(vr_max, self._pid_vr.compute(err_r, dt, self._sat_r)))
            self._sat_r = (abs(des_vr) >= vr_max)
        else:
            des_vr = 0.0
            err_r  = 0.0

        # ── Step 2: Linear acceleration ramp ──────────────────────────────
        def ramp(des, cur, max_a, dt_):
            if des > cur:
                return min(des, cur + max_a * dt_)
            if des < cur:
                return max(des, cur - max_a * dt_)
            return cur

        self._tv.x = ramp(des_vx, self._tv.x, MAX_ACCEL, dt)
        self._tv.y = ramp(des_vy, self._tv.y, MAX_ACCEL, dt)
        self._tvr  = ramp(des_vr, self._tvr,  MAX_ROT_ACCEL, dt)

        # ── Step 3: Velocity attenuation (firmware line 134) ──────────────
        self._tv.x += (meas_vel.x - self._tv.x) * self.ATTEN_FACTOR
        self._tv.y += (meas_vel.y - self._tv.y) * self.ATTEN_FACTOR
        self._tvr  += (meas_vtheta - self._tvr) * self.ATTEN_FACTOR

        # ── Step 4: Snap-to-zero near target ──────────────────────────────
        final_vx = self._tv.x
        final_vy = self._tv.y
        final_vr = self._tvr

        if abs(err_x) < MIN_DISTANCE and abs(final_vx) < self.SNAP_VEL_XY:
            final_vx = 0.0
        if abs(err_y) < MIN_DISTANCE and abs(final_vy) < self.SNAP_VEL_XY:
            final_vy = 0.0
        if abs(err_r) < MIN_ANGLE and abs(final_vr) < self.SNAP_VEL_ROT:
            final_vr = 0.0

        # ── Step 5: Final clamp ───────────────────────────────────────────
        final_vx = max(-v_max,  min(v_max,  final_vx))
        final_vy = max(-v_max,  min(v_max,  final_vy))
        final_vr = max(-vr_max, min(vr_max, final_vr))

        # ── Apply to TRUE physics state ───────────────────────────────────
        new_vel = Vec2(final_vx, final_vy)
        new_pos = self.pos + new_vel * dt

        # Field boundary clamping
        new_pos.x = max(ROBOT_RADIUS, min(FIELD_W - ROBOT_RADIUS, new_pos.x))
        new_pos.y = max(ROBOT_RADIUS, min(FIELD_H - ROBOT_RADIUS, new_pos.y))

        # Collision check
        if occupancy.is_occupied_circle(new_pos, ROBOT_RADIUS):
            new_vel = Vec2(0, 0)
            new_pos = self.pos
            self.collided = True
            self.stall_counter += 1
        else:
            self.collided = False
            self.stall_counter = max(0, self.stall_counter - 1)

        self.vel    = new_vel
        self.pos    = new_pos
        self.vtheta = final_vr
        self.theta  = (self.theta + self.vtheta * dt) % (2 * math.pi)

    @property
    def final_vel_sq(self) -> float:
        """Squared magnitude of final velocity (for completion check)."""
        return self.vel.x**2 + self.vel.y**2 + self.vtheta**2

    def to_dict(self) -> dict:
        return {
            'x':      self.pos.x,
            'y':      self.pos.y,
            'theta':  self.theta,
            'vx':     self.vel.x,
            'vy':     self.vel.y,
            'vtheta': self.vtheta,
            'radius': ROBOT_RADIUS,
            'collided': self.collided,
            'trail':  self.trail[-80:],
        }
