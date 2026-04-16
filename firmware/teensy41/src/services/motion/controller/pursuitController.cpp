#include "pursuitController.h"
#include "config/settings.h"
#include "os/console.h"
#include "services/localisation/localisation.h"
#include "services/motion/controller/positionController.h"
#include <algorithm>
#include <cmath>


// ============================================================
//  Lifecycle
// ============================================================

PursuitController::PursuitController()
    : position(0.0f),
      velocity(0.0f),
      target_velocity(0.0f),
      target(0.0f),
      vx_controller  (4.0f,  0.0f, 100.0f),
      vy_controller  (4.0f,  0.0f, 100.0f),
      vrot_controller(10.0f, 0.0f,  70.0f)
{}

FLASHMEM void PursuitController::setFeedrate(float feed) { m_feedrate = feed; }

FLASHMEM void PursuitController::exec() {}  // unused — control() est appelé par le cycle ISR

FLASHMEM void PursuitController::reset() {
    Job::reset();
    position        = Vec3(0.0f);
    velocity        = Vec3(0.0f);
    target_velocity = Vec3(0.0f);
    target          = Vec3(0.0f);
    m_lastControlUs = 0;
    m_lastAimMs     = 0;
    m_watchdogTripped = false;
    m_headingMode   = false;
    controller.reset();
    controller.disable();
    vx_controller.reset();
    vy_controller.reset();
    vrot_controller.reset();
}

FLASHMEM void PursuitController::start() {
    Job::start();
    controller.enable();

    // Initialise la cible avec la position actuelle pour ne pas sauter
    // si aucun aim() n'a été reçu avant le start.
    Vec3 startPos = localisation.getPosition();
    position = startPos;
    if (target.magSq() == 0.0f) target = startPos;

    target_velocity = Vec3(0.0f);
    m_lastControlUs = 0;
    // Le watchdog démarre actif : pas de mouvement tant qu'aucun aim
    // n'est arrivé.
    m_lastAimMs = 0;
    m_watchdogTripped = true;
    vx_controller.reset();
    vy_controller.reset();
    vrot_controller.reset();
}

FLASHMEM void PursuitController::complete() {
    reset();
    m_state = JobState::COMPLETED;
}


// ============================================================
//  Setters
// ============================================================

FLASHMEM void PursuitController::setPosition(const Vec3& p) { position = p; }

FLASHMEM void PursuitController::setTarget(const Vec3& t) {
    target            = t;
    m_lastAimMs       = millis();
    m_watchdogTripped = false;
}

FLASHMEM void PursuitController::setHeadingMode(bool enabled, RobotCompass face) {
    m_headingMode = enabled;
    m_headingFace = face;
}

FLASHMEM void PursuitController::setSteppers(Stepper* a, Stepper* b, Stepper* c) {
    controller.setSteppers(a, b, c);
    controller.setTargetVelocity(Vec3(0.0f));
}

// Calcule l'orientation cible effective. En mode heading, on aligne la
// face m_headingFace sur la direction (target - position) pour que le
// robot avance "face en avant".
FLASHMEM float PursuitController::effectiveTargetHeading() const {
    if (!m_headingMode) return target.c;

    Vec2 dir = Vec2(target - position);
    if (dir.magSq() < 100.0f) return position.c;  // évite l'instabilité près de la cible
    float heading = dir.heading();  // direction monde
    // Le robot doit tourner pour que `face` pointe vers `heading`.
    return heading - getCompassOrientation(m_headingFace);
}


// ============================================================
//  computeVelocity — pas de rampe agressive, pas de snap distance
// ============================================================

FLASHMEM Vec3 PursuitController::computeVelocity(float dt, Vec2 error, float angle) {
    const float maxSpeed    = Settings::Motion::MAX_SPEED     * m_feedrate;
    const float maxRotSpeed = Settings::Motion::MAX_ROT_SPEED * m_feedrate;

    Vec3 desired(0.0f);

    if (!isCanceling() && !isPausing()) {
        bool sat = false;
        desired.x = std::clamp(vx_controller.compute(error.x, dt, sat),
                               -maxSpeed, maxSpeed);
        desired.y = std::clamp(vy_controller.compute(error.y, dt, sat),
                               -maxSpeed, maxSpeed);
        desired.c = std::clamp(vrot_controller.compute(angle, dt, sat),
                               -maxRotSpeed, maxRotSpeed);
    }

    // Rampe d'accélération douce (identique au PositionController) pour
    // éviter les à-coups quand la carotte saute.
    auto ramp = [](float des, float cur, float maxA, float dt_) -> float {
        if (des > cur) return std::min(des, cur + maxA * dt_);
        if (des < cur) return std::max(des, cur - maxA * dt_);
        return cur;
    };

    target_velocity.x = ramp(desired.x, target_velocity.x, Settings::Motion::MAX_ACCEL,     dt);
    target_velocity.y = ramp(desired.y, target_velocity.y, Settings::Motion::MAX_ACCEL,     dt);
    target_velocity.c = ramp(desired.c, target_velocity.c, Settings::Motion::MAX_ROT_ACCEL, dt);

    Vec3 final_vel = target_velocity;

    // Pas de snap-zero ici : c'est la couche haute (Python) qui décide
    // de la stratégie d'approche en commutant la carotte vers la cible
    // finale. Le contrôleur reste réactif jusqu'au dernier moment.

    final_vel.x = std::clamp(final_vel.x, -maxSpeed,    maxSpeed);
    final_vel.y = std::clamp(final_vel.y, -maxSpeed,    maxSpeed);
    final_vel.c = std::clamp(final_vel.c, -maxRotSpeed, maxRotSpeed);

    return final_vel;
}


// ============================================================
//  onUpdate — séquenceur principal (~200 Hz)
// ============================================================

void PursuitController::onUpdate() {
    // ---- dt ----
    uint32_t now = micros();
    float dt = (m_lastControlUs == 0) ? (Settings::Motion::PID_INTERVAL * 1e-6f)
                                      : ((now - m_lastControlUs) * 1e-6f);
    m_lastControlUs = now;
    dt = std::clamp(dt, 1e-5f, Settings::Motion::PID_INTERVAL * 1e-6f * 2.0f);

    // ---- Cinématique depuis OTOS ----
    position = localisation.getPosition();
    velocity = localisation.getVelocity();
    velocity.rotateZ(-position.c);

    // ---- Watchdog ----
    if (m_lastAimMs == 0 || (millis() - m_lastAimMs) > WATCHDOG_MS) {
        if (!m_watchdogTripped) {
            Console::warn("Pursuit") << "watchdog tripped — braking" << Console::endl;
            m_watchdogTripped = true;
        }
        // Décélération douce vers zéro
        target_velocity *= 0.85f;
        if (fabsf(target_velocity.x) < 5.0f) target_velocity.x = 0.0f;
        if (fabsf(target_velocity.y) < 5.0f) target_velocity.y = 0.0f;
        if (fabsf(target_velocity.c) < 0.05f) target_velocity.c = 0.0f;

        Vec3 cmd = target_velocity;
        cmd.rotateZ(position.c);
        controller.setTargetVelocity(cmd);
        return;
    }

    // ---- Cible effective (heading mode) ----
    float tHeading = effectiveTargetHeading();

    // ---- Erreurs ----
    float angle = PositionController::shortestAngleDiff(tHeading, position.c);
    Vec2  error = target - position;

    // ---- Vitesse désirée ----
    Vec3 final_vel = computeVelocity(dt, error, angle);

    // ---- Envoi à la velocity controller (monde → robot frame) ----
    Vec3 cmd_robot = final_vel;
    cmd_robot.rotateZ(position.c);
    controller.setTargetVelocity(cmd_robot);
}


// ============================================================
//  onPausing / onCanceling
// ============================================================

FLASHMEM void PursuitController::onPausing() {
    onCanceling();
}

FLASHMEM void PursuitController::onCanceling() {
    const float dt = Settings::Motion::PID_INTERVAL * 1e-6f;

    // Décélération exponentielle (identique au PositionController)
    target_velocity *= 0.9f;

    velocity = localisation.getVelocity();
    velocity.rotateZ(-position.c);
    position = position + (velocity * dt);

    Vec3 final_vel = target_velocity;
    if (fabsf(final_vel.x) < 20.0f) final_vel.x = 0.0f;
    if (fabsf(final_vel.y) < 20.0f) final_vel.y = 0.0f;
    if (fabsf(final_vel.c) <  0.1f) final_vel.c = 0.0f;

    if (final_vel.magSq() > 0.0f) {
        final_vel.rotateZ(position.c);
        controller.setTargetVelocity(final_vel);
    } else {
        controller.setTargetVelocity(Vec3(0.0f));
        reset();
        m_state = JobState::CANCELING;
        onCanceled();
    }
}


// ============================================================
//  step / control
// ============================================================

void PursuitController::step() {
    if (!isBusy()) return;
    controller.step();
}

void PursuitController::control() {
    if (!isBusy()) return;

    static long lastTime = 0;
    if (micros() - lastTime < (long)Settings::Motion::PID_INTERVAL) return;
    lastTime = micros();

    if      (isPausing())   onCanceling();
    else if (isCanceling()) onCanceling();
    else if (isCanceled())  return;
    else if (isCompleted()) return;
    else                    onUpdate();
}
