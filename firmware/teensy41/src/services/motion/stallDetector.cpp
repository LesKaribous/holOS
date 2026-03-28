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
}

void StallDetector::reset() {
    m_startPos    = Vec3(0.0f);
    m_target      = Vec3(0.0f);
    m_lastPos     = Vec3(0.0f);
    m_lastCheckMs = 0;
    m_stats       = {};
}

bool StallDetector::update(const Vec3& currentPos, uint32_t elapsedMs) {
    // Pas encore dans la fenêtre active
    if (elapsedMs < config.delayMs) return false;

    // Pas encore arrivé à la prochaine période de vérification
    uint32_t sinceLastCheck = (m_lastCheckMs == 0) ? config.periodMs
                                                    : (elapsedMs - m_lastCheckMs);
    if (sinceLastCheck < config.periodMs) return false;

    m_lastCheckMs = elapsedMs;
    m_stats.checks++;

    // ---- Déplacement sur la fenêtre glissante ----
    float recentTransMm = Vec2(currentPos.x - m_lastPos.x,
                               currentPos.y - m_lastPos.y).mag();
    float recentAngRad  = fabsf(shortestAngleDiff(currentPos.c, m_lastPos.c));

    // Mise à jour du pire cas (pour le tuning)
    if (recentTransMm            < m_stats.minTransMm)  m_stats.minTransMm  = recentTransMm;
    if (recentAngRad * RAD_TO_DEG < m_stats.minAngleDeg) m_stats.minAngleDeg = recentAngRad * RAD_TO_DEG;

    // ---- Amplitude de la cible depuis le départ ----
    float transTarget = Vec2(m_target.x - m_startPos.x,
                             m_target.y - m_startPos.y).mag();
    float angTarget   = fabsf(shortestAngleDiff(m_target.c, m_startPos.c));

    bool transStall = (transTarget > config.targetTransMm)
                   && (recentTransMm < config.transDispMm);
    bool angStall   = (angTarget > config.targetAngleRad)
                   && (recentAngRad  < config.angleDispRad);

    // Snapshot pour la prochaine fenêtre
    m_lastPos = currentPos;

    if (transStall || angStall) {
        m_stats.triggered = true;
        Console::warn("StallDetector")
            << "Stall: trans=" << (int)recentTransMm << "mm"
            << " ang=" << (int)(recentAngRad * RAD_TO_DEG) << "deg"
            << " elapsed=" << (int)(elapsedMs / 1000) << "s"
            << Console::endl;
        return true;
    }

    return false;
}

// ============================================================
//  Utilitaire
// ============================================================

float StallDetector::shortestAngleDiff(float a, float b) {
    float diff = fmodf(a - b + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    diff -= M_PI;
    if (diff == -M_PI) diff = M_PI;
    return diff;
}
