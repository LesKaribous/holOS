"""
sim/physics.py — Robot physics model (holonomic, PD velocity control).
Extracted from sim_core.py for cleaner module separation.
"""

import math
from shared.config import (
    FIELD_W, FIELD_H, ROBOT_RADIUS,
    MAX_SPEED, MAX_ACCEL, MAX_ROT_SPEED, MAX_ROT_ACCEL,
    Vec2, angle_diff, Team, TEAM_START, POI,
)


class RobotPhysics:
    """
    Simplified holonomic robot model.
    PD velocity control toward target position/heading.
    """

    KP_POS  = 5.5    # position proportional (1/s)
    KP_ROT  = 5.0    # rotation proportional (1/s)

    TRAIL_DIST  = 15.0
    TRAIL_MAX   = 300

    def __init__(self):
        self.pos     = Vec2(POI.startYellow.x, POI.startYellow.y)
        self.theta   = 0.0
        self.vel     = Vec2(0, 0)
        self.vtheta  = 0.0

        self.target_pos:   Vec2  | None = None
        self.target_theta: float | None = None

        self.feedrate  = 1.0
        self.collided  = False
        self.trail: list[tuple[float, float]] = []
        self.stall_counter = 0

    def reset(self):
        self.pos     = Vec2(POI.startYellow.x, POI.startYellow.y)
        self.theta   = 0.0
        self.vel     = Vec2(0, 0)
        self.vtheta  = 0.0
        self.target_pos   = None
        self.target_theta = None
        self.feedrate  = 1.0
        self.collided  = False
        self.stall_counter = 0
        self.trail.clear()

    def reset_to_start(self, team: str):
        team_enum = Team(team)
        start_pos, start_theta = TEAM_START[team_enum]
        self.pos    = Vec2(start_pos.x, start_pos.y)
        self.theta  = start_theta
        self.vel    = Vec2(0, 0)
        self.vtheta = 0.0
        self.target_pos   = None
        self.target_theta = None
        self.collided  = False
        self.stall_counter = 0
        self.trail.clear()

    def update(self, dt: float, occupancy):
        v_max  = MAX_SPEED    * self.feedrate
        a_max  = MAX_ACCEL    * dt
        vr_max = MAX_ROT_SPEED * self.feedrate
        ar_max = MAX_ROT_ACCEL * dt

        # Translational PD
        if self.target_pos is not None:
            err = self.target_pos - self.pos
            v_desired = err * self.KP_POS
            if v_desired.mag() > v_max:
                v_desired = v_desired.normalized() * v_max
        else:
            v_desired = Vec2(0, 0)

        dv = v_desired - self.vel
        if dv.mag() > a_max:
            dv = dv.normalized() * a_max
        new_vel = self.vel + dv

        new_pos = self.pos + new_vel * dt
        new_pos.x = max(ROBOT_RADIUS, min(FIELD_W - ROBOT_RADIUS, new_pos.x))
        new_pos.y = max(ROBOT_RADIUS, min(FIELD_H - ROBOT_RADIUS, new_pos.y))

        if occupancy.is_occupied_circle(new_pos, ROBOT_RADIUS):
            new_vel = Vec2(0, 0)
            new_pos = self.pos
            self.collided = True
            self.stall_counter += 1
        else:
            self.collided = False
            self.stall_counter = max(0, self.stall_counter - 1)

        self.vel = new_vel
        self.pos = new_pos

        # Rotational PD
        if self.target_theta is not None:
            err_r  = angle_diff(self.target_theta, self.theta)
            vr_des = err_r * self.KP_ROT
            vr_des = max(-vr_max, min(vr_max, vr_des))
            dvr    = vr_des - self.vtheta
            dvr    = max(-ar_max, min(ar_max, dvr))
            self.vtheta = self.vtheta + dvr
        else:
            dvr = max(-ar_max, min(ar_max, -self.vtheta))
            self.vtheta += dvr
            if abs(self.vtheta) < 0.01:
                self.vtheta = 0.0

        self.theta = (self.theta + self.vtheta * dt) % (2 * math.pi)

        # Trail
        if not self.trail or Vec2(*self.trail[-1]).dist(self.pos) > self.TRAIL_DIST:
            self.trail.append((self.pos.x, self.pos.y))
            if len(self.trail) > self.TRAIL_MAX:
                self.trail.pop(0)

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
