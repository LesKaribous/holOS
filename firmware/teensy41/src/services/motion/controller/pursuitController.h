#pragma once
#include "utils/geometry.h"
#include "config/calibration.h"
#include "velocityController.h"
#include "pid.h"
#include "services/motion/stallDetector.h"

// ============================================================
//  PursuitController — live target following (pure pursuit)
//
//  Contrairement au PositionController, ce contrôleur ne décélère
//  pas en approchant de la cible (la décélération est gérée côté
//  haut niveau en déplaçant la "carotte"). Il accepte des mises à
//  jour de cible en temps réel via setTarget() sans reset des PID.
//
//  Mode heading : si activé, l'orientation cible est recalculée à
//  chaque cycle pour aligner une face du robot (RobotCompass) sur
//  la direction de déplacement vers la cible.
//
//  Watchdog : si aucun setTarget() n'est reçu depuis plus de
//  WATCHDOG_MS, le contrôleur freine (vitesse = 0) sans annuler
//  le job — il reprendra dès qu'un nouveau aim arrivera.
//
//  Le PursuitController ne se complète jamais tout seul : c'est au
//  système de plus haut niveau (Python pursuit loop) de l'annuler
//  via Motion::cancel() quand la cible finale est atteinte.
// ============================================================

class PursuitController : public Controller {
public:
    static constexpr uint32_t WATCHDOG_MS = 200;

    PursuitController();

    // Cycle ISR
    void step()    override;
    void control() override;

    // Cycle de vie
    void exec()     override;
    void reset()    override;
    void start()    override;
    void complete() override;

    void onUpdate();
    void onPausing();
    void onCanceling();

    // Position courante (MAJ depuis localisation à chaque cycle)
    void setPosition(const Vec3& p);

    // Met à jour la cible vivante. NE reset PAS les PID — c'est le
    // point essentiel de ce contrôleur. Met aussi à jour le watchdog.
    void setTarget(const Vec3& t);

    // Mode heading : si activé, l'orientation cible (target.c) est
    // remplacée à chaque cycle par l'angle qui aligne `face` sur la
    // direction (target - position).
    //   enabled=false → suit target.c tel quel
    //   enabled=true  → ignore target.c, recalcule depuis face
    void setHeadingMode(bool enabled, RobotCompass face = RobotCompass::A);

    void setSteppers(Stepper* a, Stepper* b, Stepper* c);
    void setFeedrate(float feed);

    Vec3 getPosition() const { return position; }
    Vec3 getTarget()   const { return target; }
    Vec3 getVelocity() const { return velocity; }

    bool watchdogTripped() const { return m_watchdogTripped; }
    bool isHeadingMode()   const { return m_headingMode; }

private:
    // ---- Feedrate ----
    float m_feedrate = 1.0f;

    // ---- Mode heading ----
    bool         m_headingMode = false;
    RobotCompass m_headingFace = RobotCompass::A;

    // ---- Watchdog ----
    uint32_t m_lastAimMs    = 0;
    bool     m_watchdogTripped = false;

    // ---- Cinématique ----
    Vec3 position;
    Vec3 velocity;
    Vec3 target_velocity;
    Vec3 target;

    // ---- Timers ----
    uint32_t m_lastControlUs = 0;

    // ---- Sous-contrôleurs ----
    VelocityController controller;
    PIDController      vx_controller;
    PIDController      vy_controller;
    PIDController      vrot_controller;

    Vec3 computeVelocity(float dt, Vec2 error, float angle);
    float effectiveTargetHeading() const;
};
