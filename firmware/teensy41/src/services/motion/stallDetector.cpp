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
    m_stats       = {};

    // Reset velocity accumulators
    m_velStallAccumX   = 0.0f;
    m_velStallAccumY   = 0.0f;
    m_velStallAccumRot = 0.0f;

    // Reset stagnation accumulators
    m_stagLastX  = startPos.x;
    m_stagLastY  = startPos.y;
    m_stagAccumX = 0.0f;
    m_stagAccumY = 0.0f;
    m_stagWindowAccumX = 0.0f;
    m_stagWindowAccumY = 0.0f;
}

void StallDetector::reset() {
    m_startPos    = Vec3(0.0f);
    m_target      = Vec3(0.0f);
    m_lastPos     = Vec3(0.0f);
    m_lastCheckMs = 0;
    m_stats       = {};
    m_velStallAccumX   = 0.0f;
    m_velStallAccumY   = 0.0f;
    m_velStallAccumRot = 0.0f;
    m_stagLastX  = 0.0f;
    m_stagLastY  = 0.0f;
    m_stagAccumX = 0.0f;
    m_stagAccumY = 0.0f;
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
        m_stats.stalledX = true;
        newStall = true;
    }
    if (m_velStallAccumY >= config.velStallTimeS && !m_stats.stalledY) {
        m_stats.stalledY = true;
        newStall = true;
    }
    if (m_velStallAccumRot >= config.velStallTimeS && !m_stats.stalledRot) {
        m_stats.stalledRot = true;
        newStall = true;
    }

    if (newStall && !m_stats.velTriggered) {
        m_stats.velTriggered = true;
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
//  Position stagnation detection (called at PID rate ~500 Hz)
//
//  Per-axis: every STAG_SNAPSHOT_PERIOD, check if position has
//  moved more than stagMoveMm since last snapshot.
//    - If NOT moved AND remaining error > stagErrorMm:
//      accumulate stagnation time.
//    - If moved: reset accumulator.
//
//  When accumulated stagnation time exceeds stagTimeS → stalled.
//
//  This catches low-speed stalls where the velocity mismatch
//  detector fails because commanded velocity is below its threshold.
// ============================================================

void StallDetector::updateStagnation(const Vec3& pos, const Vec3& target, float dt) {
    // ── X axis ──
    m_stagWindowAccumX += dt;
    if (m_stagWindowAccumX >= STAG_SNAPSHOT_PERIOD) {
        float deltaX = fabsf(pos.x - m_stagLastX);
        float errorX = fabsf(target.x - pos.x);
        m_stagLastX = pos.x;
        m_stagWindowAccumX = 0.0f;

        if (deltaX < config.stagMoveMm && errorX > config.stagErrorMm) {
            m_stagAccumX += STAG_SNAPSHOT_PERIOD;
        } else {
            m_stagAccumX = 0.0f;
        }

        if (m_stagAccumX >= config.stagTimeS && !m_stats.stalledX) {
            m_stats.stalledX = true;
            m_stats.velTriggered = true;  // so legacy displacement also sees it
            Console::warn("StallDetector")
                << "Stagnation stall X: pos=" << (int)pos.x
                << " target=" << (int)target.x
                << " error=" << (int)errorX << "mm"
                << " stag=" << (int)(m_stagAccumX * 1000) << "ms"
                << Console::endl;
        }
    }

    // ── Y axis ──
    m_stagWindowAccumY += dt;
    if (m_stagWindowAccumY >= STAG_SNAPSHOT_PERIOD) {
        float deltaY = fabsf(pos.y - m_stagLastY);
        float errorY = fabsf(target.y - pos.y);
        m_stagLastY = pos.y;
        m_stagWindowAccumY = 0.0f;

        if (deltaY < config.stagMoveMm && errorY > config.stagErrorMm) {
            m_stagAccumY += STAG_SNAPSHOT_PERIOD;
        } else {
            m_stagAccumY = 0.0f;
        }

        if (m_stagAccumY >= config.stagTimeS && !m_stats.stalledY) {
            m_stats.stalledY = true;
            m_stats.velTriggered = true;
            Console::warn("StallDetector")
                << "Stagnation stall Y: pos=" << (int)pos.y
                << " target=" << (int)target.y
                << " error=" << (int)errorY << "mm"
                << " stag=" << (int)(m_stagAccumY * 1000) << "ms"
                << Console::endl;
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
