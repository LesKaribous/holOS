#include "stallDetector.h"
#include "os/console.h"
#include <cmath>

// ============================================================
//  API
// ============================================================

void StallDetector::begin(const Vec3& startPos, const Vec3& target) {
    m_startPos    = startPos;
    m_target      = target;
    m_lastPos     = startPos;
    m_lastCheckMs = 0;
    m_stats       = {};   // full reset: new move starts with clean stats

    // Expected travel per axis — used to gate stagnation checks. If an axis
    // wasn't supposed to move (|travel| small), any residual PID error on
    // that axis would otherwise trigger a false-positive stagnation stall.
    m_expectedTravelX = fabsf(target.x - startPos.x);
    m_expectedTravelY = fabsf(target.y - startPos.y);

    // Reset velocity accumulators
    m_velStallAccumX   = 0.0f;
    m_velStallAccumY   = 0.0f;
    m_velStallAccumRot = 0.0f;

    // Reset error stagnation accumulators
    m_stagBaseErrX = fabsf(target.x - startPos.x);
    m_stagBaseErrY = fabsf(target.y - startPos.y);
    m_stagAccumX   = 0.0f;
    m_stagAccumY   = 0.0f;
    m_stagWindowAccumX = 0.0f;
    m_stagWindowAccumY = 0.0f;
}

// reset() is called when the PositionController leaves an active move
// (e.g. during onCanceling() once velocity reaches zero). It must NOT
// clear m_stats — the caller (Motion::collectStats) reads the outcome
// of the move AFTER reset(). Only begin() wipes stats on the next move.
void StallDetector::reset() {
    m_startPos    = Vec3(0.0f);
    m_target      = Vec3(0.0f);
    m_lastPos     = Vec3(0.0f);
    m_lastCheckMs = 0;
    // m_stats preserved on purpose — see comment above.
    m_expectedTravelX = 0.0f;
    m_expectedTravelY = 0.0f;
    m_velStallAccumX   = 0.0f;
    m_velStallAccumY   = 0.0f;
    m_velStallAccumRot = 0.0f;
    m_stagBaseErrX = 0.0f;
    m_stagBaseErrY = 0.0f;
    m_stagAccumX   = 0.0f;
    m_stagAccumY   = 0.0f;
    m_stagWindowAccumX = 0.0f;
    m_stagWindowAccumY = 0.0f;
}

// ============================================================
//  Velocity mismatch detection (called at PID rate ~500 Hz)
//
//  For each axis independently:
//    if |commanded| > threshold AND |measured| < threshold
//      → accumulate mismatch time
//    else
//      → reset accumulator
//
//  When accumulated time exceeds velStallTimeS → stalled on that axis.
//  Per-axis detection means hitting a Y wall doesn't prevent X motion.
// ============================================================

void StallDetector::updateVelocity(const Vec3& cmdVel, const Vec3& otosVel, float dt) {
    // X axis
    bool cmdX  = fabsf(cmdVel.x) > config.velCmdMinMmS;
    bool moveX = fabsf(otosVel.x) > config.velOtosMaxMmS;
    if (cmdX && !moveX) {
        m_velStallAccumX += dt;
    } else {
        m_velStallAccumX = 0.0f;
        // Don't clear stalledX here — stagnation may have set it
    }

    // Y axis
    bool cmdY  = fabsf(cmdVel.y) > config.velCmdMinMmS;
    bool moveY = fabsf(otosVel.y) > config.velOtosMaxMmS;
    if (cmdY && !moveY) {
        m_velStallAccumY += dt;
    } else {
        m_velStallAccumY = 0.0f;
    }

    // Rotation
    bool cmdR  = fabsf(cmdVel.c) > config.velRotMinRadS;
    bool moveR = fabsf(otosVel.c) > config.velRotMaxRadS;
    if (cmdR && !moveR) {
        m_velStallAccumRot += dt;
    } else {
        m_velStallAccumRot = 0.0f;
        m_stats.stalledRot = false;
    }

    // Check thresholds
    bool newStall = false;
    if (m_velStallAccumX >= config.velStallTimeS && !m_stats.stalledX) {
        m_stats.stalledX   = true;
        m_stats.causeVelX  = true;  // sticky cause flag — survives reset()
        newStall = true;
    }
    if (m_velStallAccumY >= config.velStallTimeS && !m_stats.stalledY) {
        m_stats.stalledY   = true;
        m_stats.causeVelY  = true;
        newStall = true;
    }
    if (m_velStallAccumRot >= config.velStallTimeS && !m_stats.stalledRot) {
        m_stats.stalledRot   = true;
        m_stats.causeVelRot  = true;
        newStall = true;
    }

    if (newStall && !m_stats.velTriggered) {
        m_stats.velTriggered = true;
        m_stats.triggered    = true;   // unified flag for collectStats()
        Console::warn("StallDetector")
            << "Velocity stall: X=" << (m_stats.stalledX ? "STALL" : "ok")
            << " Y=" << (m_stats.stalledY ? "STALL" : "ok")
            << " R=" << (m_stats.stalledRot ? "STALL" : "ok")
            << " (cmd=" << (int)cmdVel.x << "," << (int)cmdVel.y
            << " otos=" << (int)otosVel.x << "," << (int)otosVel.y << ")"
            << Console::endl;
    }
}

// ============================================================
//  Error stagnation detection (called at PID rate ~500 Hz)
//
//  Per-axis: every STAG_SNAPSHOT_PERIOD (100 ms), check if the
//  PID error has decreased from a BASELINE captured when the
//  stagnation period started.
//
//  Using a baseline instead of snapshot-to-snapshot comparison
//  prevents oscillation (robot vibrating ±1-2 mm against a wall)
//  from resetting the accumulator.  The oscillation averages out
//  relative to the baseline.
//
//  Flow:
//    1. Error > stagErrorMm → start tracking (capture baseline)
//    2. Every 100 ms: if (baseline - currentError) < stagMoveMm
//       → no real progress → accumulate stagnation time
//    3. If real progress detected → reset accumulator + baseline
//    4. After stagTimeS of stagnation → stall on that axis
// ============================================================

void StallDetector::updateStagnation(const Vec3& pos, const Vec3& target, float dt) {
    // An axis that wasn't supposed to move can't meaningfully stagnate —
    // any residual PID error there will look like "constant error" forever
    // and trigger a false-positive. Require the expected travel on an axis
    // to be at least targetTransMm before enabling its stagnation check.
    const bool checkX = m_expectedTravelX > config.targetTransMm;
    const bool checkY = m_expectedTravelY > config.targetTransMm;

    // ── X axis ──
    if (checkX) {
        m_stagWindowAccumX += dt;
        if (m_stagWindowAccumX >= STAG_SNAPSHOT_PERIOD) {
            float errorX     = fabsf(target.x - pos.x);
            float errReduced = m_stagBaseErrX - errorX;   // positive = making progress
            m_stagBaseErrX   = errorX;
            m_stagWindowAccumX = 0.0f;

            if (errReduced < config.stagMoveMm && errorX > config.stagErrorMm) {
                // Error didn't decrease enough — stagnating
                m_stagAccumX += STAG_SNAPSHOT_PERIOD;
            } else {
                m_stagAccumX = 0.0f;
            }

            if (m_stagAccumX >= config.stagTimeS && !m_stats.stalledX) {
                m_stats.stalledX     = true;
                m_stats.causeStagX   = true;  // sticky cause flag
                m_stats.velTriggered = true;
                m_stats.triggered    = true;
                Console::warn("StallDetector")
                    << "Stall X (stag): err=" << (int)errorX << "mm"
                    << " after " << (int)(m_stagAccumX * 1000) << "ms"
                    << Console::endl;
            }
        }
    }

    // ── Y axis ──
    if (checkY) {
        m_stagWindowAccumY += dt;
        if (m_stagWindowAccumY >= STAG_SNAPSHOT_PERIOD) {
            float errorY     = fabsf(target.y - pos.y);
            float errReduced = m_stagBaseErrY - errorY;
            m_stagBaseErrY   = errorY;
            m_stagWindowAccumY = 0.0f;

            if (errReduced < config.stagMoveMm && errorY > config.stagErrorMm) {
                m_stagAccumY += STAG_SNAPSHOT_PERIOD;
            } else {
                m_stagAccumY = 0.0f;
            }

            if (m_stagAccumY >= config.stagTimeS && !m_stats.stalledY) {
                m_stats.stalledY     = true;
                m_stats.causeStagY   = true;
                m_stats.velTriggered = true;
                m_stats.triggered    = true;
                Console::warn("StallDetector")
                    << "Stall Y (stag): err=" << (int)errorY << "mm"
                    << " after " << (int)(m_stagAccumY * 1000) << "ms"
                    << Console::endl;
            }
        }
    }
}

// ============================================================
//  Legacy displacement-based detection (called at slow rate)
// ============================================================

bool StallDetector::update(const Vec3& currentPos, uint32_t elapsedMs) {
    if (elapsedMs < config.delayMs) return false;

    uint32_t sinceLastCheck = (m_lastCheckMs == 0) ? config.periodMs
                                                    : (elapsedMs - m_lastCheckMs);
    if (sinceLastCheck < config.periodMs) return false;

    m_lastCheckMs = elapsedMs;
    m_stats.checks++;

    float recentTransMm = Vec2(currentPos.x - m_lastPos.x,
                               currentPos.y - m_lastPos.y).mag();
    float recentAngRad  = fabsf(shortestAngleDiff(currentPos.c, m_lastPos.c));

    if (recentTransMm            < m_stats.minTransMm)  m_stats.minTransMm  = recentTransMm;
    if (recentAngRad * RAD_TO_DEG < m_stats.minAngleDeg) m_stats.minAngleDeg = recentAngRad * RAD_TO_DEG;

    float transTarget = Vec2(m_target.x - m_startPos.x,
                             m_target.y - m_startPos.y).mag();
    float angTarget   = fabsf(shortestAngleDiff(m_target.c, m_startPos.c));

    bool transStall = (transTarget > config.targetTransMm)
                   && (recentTransMm < config.transDispMm);
    bool angStall   = (angTarget > config.targetAngleRad)
                   && (recentAngRad  < config.angleDispRad);

    m_lastPos = currentPos;

    // Also check velocity-based or stagnation-based stall (faster detection)
    bool velStall = m_stats.velTriggered;

    if (transStall || angStall || velStall) {
        m_stats.triggered = true;
        if (!velStall) {
            Console::warn("StallDetector")
                << "Displacement stall: trans=" << (int)recentTransMm << "mm"
                << " ang=" << (int)(recentAngRad * RAD_TO_DEG) << "deg"
                << " elapsed=" << (int)(elapsedMs / 1000) << "s"
                << Console::endl;
        }
        return true;
    }

    return false;
}

// ============================================================
//  Utility
// ============================================================

float StallDetector::shortestAngleDiff(float a, float b) {
    float diff = fmodf(a - b + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    diff -= M_PI;
    if (diff == -M_PI) diff = M_PI;
    return diff;
}
