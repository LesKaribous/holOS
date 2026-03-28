#include "positionController.h"
#include "config/settings.h"
#include "os/console.h"
#include "services/localisation/localisation.h"
#include <algorithm>

#define NORMALIZE(x) ((x) > 0.0f ? 1.0f : ((x) < 0.0f ? -1.0f : 0.0f))


// ============================================================
//  Lifecycle
// ============================================================

PositionController::PositionController()
    : position(0.0f),
      last_position(0.0f),
      velocity(0.0f),
      last_velocity(0.0f),
      target_velocity(0.0f),
      acceleration(0.0f),
      target(0.0f),
      newTarget(0.0f),
      vx_controller  (4.0f,  0.0f, 100.0f),
      vy_controller  (4.0f,  0.0f, 100.0f),
      vrot_controller(10.0f, 0.0f,  70.0f)
{}

void PositionController::setFeedrate(float feed) { m_feedrate = feed; }
void PositionController::setStallEnabled(bool enabled) { m_stallEnabled = enabled; }
void PositionController::exec() {}  // unused — control() est appelé par le cycle ISR

void PositionController::reset() {
    Job::reset();
    position        = Vec3(0.0f);
    last_position   = Vec3(0.0f);
    velocity        = Vec3(0.0f);
    last_velocity   = Vec3(0.0f);
    acceleration    = Vec3(0.0f);
    target          = Vec3(0.0f);
    target_velocity = Vec3(0.0f);
    newTarget       = Vec3(0.0f);
    m_stalledFlag   = false;
    m_satX = m_satY = m_satZ = false;
    m_satXCount = m_satYCount = m_satZCount = 0;
    m_lastControlUs = 0;
    moveStart       = 0;
    m_stall.reset();
    controller.reset();
    controller.disable();
    vx_controller.reset();
    vy_controller.reset();
    vrot_controller.reset();
}

void PositionController::start() {
    Job::start();
    controller.enable();

    if (!(newTarget == target)) target = newTarget;

    // Snapshot de départ pour le stall detector
    Vec3 startPos = localisation.getPosition();
    m_stall.begin(startPos, target);

    moveStart       = millis();
    m_lastControlUs = 0;  // force dt = PID_INTERVAL au premier cycle
    m_stalledFlag   = false;
    m_satX = m_satY = m_satZ = false;
    m_satXCount = m_satYCount = m_satZCount = 0;
}

void PositionController::complete() {
    reset();
    m_state   = JobState::COMPLETED;
    newTarget = target = position;
}


// ============================================================
//  Stall
// ============================================================

bool PositionController::isStalled() const {
    return m_stalledFlag && !isCanceling();
}


// ============================================================
//  onUpdate — sous-fonctions
// ============================================================

// Calcule la vitesse désirée (PID → rampe → snap → clamp).
// Retourne final_vel en coordonnées monde.
Vec3 PositionController::computeVelocity(float dt, Vec2 error, float angle) {
    const float maxSpeed    = Settings::Motion::MAX_SPEED     * m_feedrate;
    const float maxRotSpeed = Settings::Motion::MAX_ROT_SPEED * m_feedrate;

    Vec3 desired(0.0f);

    if (!isCanceling() && !isPausing()) {
        desired.x = std::clamp(vx_controller.compute(error.x, dt, m_satX),
                               -maxSpeed, maxSpeed);
        m_satX = (fabsf(desired.x) >= maxSpeed);
        if (m_satX) m_satXCount++;

        desired.y = std::clamp(vy_controller.compute(error.y, dt, m_satY),
                               -maxSpeed, maxSpeed);
        m_satY = (fabsf(desired.y) >= maxSpeed);
        if (m_satY) m_satYCount++;

        desired.c = std::clamp(vrot_controller.compute(angle, dt, m_satZ),
                               -maxRotSpeed, maxRotSpeed);
        m_satZ = (fabsf(desired.c) >= maxRotSpeed);
        if (m_satZ) m_satZCount++;
    }

    // Rampe d'accélération
    auto ramp = [](float des, float cur, float maxA, float dt_) -> float {
        if (des > cur) return cur + maxA * dt_;
        if (des < cur) return cur - maxA * dt_;
        return cur;
    };
    
    target_velocity.x = ramp(desired.x, target_velocity.x, Settings::Motion::MAX_ACCEL,     dt);
    target_velocity.y = ramp(desired.y, target_velocity.y, Settings::Motion::MAX_ACCEL,     dt);
    target_velocity.c = ramp(desired.c, target_velocity.c, Settings::Motion::MAX_ROT_ACCEL, dt);

    // Atténuation légère à l'approche (réduit les dépassements)
    target_velocity += (velocity - target_velocity) * 0.01f;

    // Snap à zéro près de la cible
    Vec3 final_vel = target_velocity;
    if (fabsf(error.x) < Settings::Motion::MIN_DISTANCE && fabsf(final_vel.x) < 20.0f)  final_vel.x = 0.0f;
    if (fabsf(error.y) < Settings::Motion::MIN_DISTANCE && fabsf(final_vel.y) < 20.0f)  final_vel.y = 0.0f;
    if (fabsf(angle)   < Settings::Motion::MIN_ANGLE    && fabsf(final_vel.c) <  0.1f)  final_vel.c = 0.0f;

    // Clamp final
    final_vel.x = std::clamp(final_vel.x, -maxSpeed,    maxSpeed);
    final_vel.y = std::clamp(final_vel.y, -maxSpeed,    maxSpeed);
    final_vel.c = std::clamp(final_vel.c, -maxRotSpeed, maxRotSpeed);

    return final_vel;
}

// Vérifie si la cible est atteinte. Appelle complete() et retourne true si oui.
bool PositionController::checkCompletion(Vec2 error, float angle, const Vec3& finalVel) {
    if (finalVel.magSq() > 0.0f) return false;

    if (fabsf(error.x) < Settings::Motion::MIN_DISTANCE &&
        fabsf(error.y) < Settings::Motion::MIN_DISTANCE &&
        fabsf(angle)   < Settings::Motion::MIN_ANGLE && isRunning()) {
        complete();
        return true;
    }
    return false;
}


// ============================================================
//  onUpdate — séquenceur principal (~200 Hz)
// ============================================================

void PositionController::onUpdate() {
    // ---- dt ----
    uint32_t now = micros();
    float dt = (m_lastControlUs == 0) ? (Settings::Motion::PID_INTERVAL * 1e-6f)
                                      : ((now - m_lastControlUs) * 1e-6f);
    m_lastControlUs = now;
    dt = std::clamp(dt, 1e-5f, Settings::Motion::PID_INTERVAL * 1e-6f * 2.0f);

    // ---- Cinématique depuis OTOS ----
    position = localisation.getPosition();
    velocity = localisation.getVelocity();
    velocity.rotateZ(-position.c);  // monde → robot frame

    // ---- Erreurs ----
    float angle = shortestAngleDiff(target.c, position.c);
    Vec2  error = target - position;

    // ---- Vitesse désirée ----
    Vec3 final_vel = computeVelocity(dt, error, angle);

    // ---- Envoi à la velocity controller ----
    if (!checkCompletion(error, angle, final_vel)) {
        Vec3 cmd_robot = final_vel;
        cmd_robot.rotateZ(position.c);  // monde → robot frame pour les steppers
        controller.setTargetVelocity(cmd_robot);
    } else {
        controller.setTargetVelocity(Vec3(0.0f));
    }

    // ---- Stall detection ----
    if (m_stallEnabled) {
        uint32_t elapsed = (uint32_t)(millis() - moveStart);
        if (m_stall.update(position, elapsed)) {
            m_stalledFlag = true;
        }
    }

    last_position = position;
    last_velocity = velocity;
}


// ============================================================
//  onPausing / onCanceling
// ============================================================

void PositionController::onPausing() {
    onCanceling();
}

void PositionController::onCanceling() {
    const float dt = Settings::Motion::PID_INTERVAL * 1e-6f;

    // Décélération exponentielle
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

void PositionController::step() {
    if (!isBusy()) return;
    controller.step();
}

void PositionController::control() {
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


// ============================================================
//  Setters
// ============================================================

void PositionController::setPosition(const Vec3& t) { position = t; }
void PositionController::setTarget(const Vec3& t)   { newTarget = t; }

void PositionController::setSteppers(Stepper* a, Stepper* b, Stepper* c) {
    controller.setSteppers(a, b, c);
    controller.setTargetVelocity(Vec3(0.0f));
}


// ============================================================
//  Utilitaire
// ============================================================

float PositionController::shortestAngleDiff(float tgt, float cur) {
    float diff = fmodf(tgt - cur + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    diff -= M_PI;
    if (diff == -M_PI) diff = M_PI;
    return diff;
}
