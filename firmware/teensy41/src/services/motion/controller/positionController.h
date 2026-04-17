#pragma once
#include "utils/geometry.h"
#include "os/jobs/asyncExecutor.h"
#include "config/calibration.h"
#include "services/lidar/occupancy.h"
#include "velocityController.h"
#include "pid.h"
#include "services/motion/stallDetector.h"

// ============================================================
//  PositionController — PID position → velocity → steppers
//
//  Reçoit une position cible en coordonnées monde (mm, mm, rad),
//  pilote la VelocityController via trois PID indépendants
//  (Vx, Vy, Vrot).
//
//  Détection de blocage :
//    StallDetector — fenêtre glissante de déplacement/temps.
//    Robuste à la saturation PID et à la dérive OTOS.
//    Configurable via stall().config avant start().
//    Désactivable via setStallEnabled(false).
// ============================================================

class PositionController : public Controller {
public:
    PositionController();

    // Appelé à chaque cycle de contrôle (PID_INTERVAL µs)
    void step()    override;
    void control() override;

    void exec()     override;
    void reset()    override;
    void start()    override;
    void complete() override;

    void onUpdate();
    void onPausing();
    void onCanceling();

    void setPosition(const Vec3& t);
    void setTarget(const Vec3& t);
    void setSteppers(Stepper* a, Stepper* b, Stepper* c);
    void setFeedrate(float feed);
    void setStallEnabled(bool enabled);

    // ── Artificial Potential Fields avoidance ──────────────────────────────
    // When enabled, a repulsive velocity component from the occupancy map is
    // added to the PID velocity command, steering the robot away from detected
    // obstacles.  Completion check is NOT affected (robot still stops at goal).
    //
    //  scale : multiplier applied to repulsiveGradient() output → mm/s.
    //          Default 50000.  Increase for stronger avoidance, decrease if
    //          the robot oscillates near obstacles.
    void setAPF(bool enabled, float scale = -1.0f);
    bool  isAPFEnabled() const { return m_apfEnabled; }
    float getAPFScale()  const { return m_apfScale;   }

    Vec3 getPosition()     const { return position / Calibration::Current.Cartesian; }
    Vec3 getVelocity()     const { return velocity  / Calibration::Current.Cartesian; }
    Vec3 getAcceleration() const { return acceleration; }
    Vec3 getTarget()       const { return target; }

    // true si le stall a été détecté (et move non en cours d'annulation)
    bool isStalled() const;

    // Reset the stall flag (used by border snap after correcting position)
    void clearStalledFlag() { m_stalledFlag = false; }

    // Border snap: set the PID target on one axis to a known value.
    // This zeros the position error on that axis so the PID stops pushing.
    // axis: 0=X, 1=Y.  Also resets the velocity on that axis.
    void snapAxisTarget(int axis, float value);

    // Accès au détecteur de blocage (config + stats)
    StallDetector&       stall()       { return m_stall; }
    const StallDetector& stall() const { return m_stall; }

    // Compteurs de saturation PID — utiles pour le diagnostic
    int satXCycles() const { return m_satXCount; }
    int satYCycles() const { return m_satYCount; }
    int satZCycles() const { return m_satZCount; }

    static float shortestAngleDiff(float target, float current);

private:
    // ---- Feedrate ----
    float m_feedrate = 1.0f;

    // ---- APF ----
    bool  m_apfEnabled = false;
    float m_apfScale   = 50000.0f;

    // ---- Stall ----
    StallDetector m_stall;
    bool          m_stallEnabled = true;
    bool          m_stalledFlag  = false;

    // ---- Saturation PID (anti-windup + diagnostic) ----
    bool m_satX = false, m_satY = false, m_satZ = false;
    int  m_satXCount = 0, m_satYCount = 0, m_satZCount = 0;

    // ---- Cinématique ----
    Vec3 position;
    Vec3 last_position;
    Vec3 velocity;
    Vec3 last_velocity;
    Vec3 target_velocity;
    Vec3 acceleration;
    Vec3 target;
    Vec3 newTarget;

    // ---- Timers ----
    long     moveStart      = 0;
    uint32_t m_lastControlUs = 0;

    // ---- Sous-contrôleurs ----
    VelocityController controller;
    PIDController      vx_controller;
    PIDController      vy_controller;
    PIDController      vrot_controller;

    // ---- Sous-fonctions de onUpdate() ----
    Vec3 computeVelocity(float dt, Vec2 error, float angle);
    bool checkCompletion(Vec2 error, float angle, const Vec3& finalVel);
};
