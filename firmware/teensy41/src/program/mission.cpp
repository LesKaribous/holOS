#include "mission.h"
#include "os/console.h"
#include "os/os.h"
#include <Arduino.h>

// Forward — defined in strategy.cpp (uses os.wait which pumps the OS loop)
extern void waitMs(unsigned long time);

// ============================================================
//  Mission — Builder & helpers
// ============================================================

FLASHMEM Mission& Mission::addStep(const char* name, uint32_t estimatedMs,
                                    Block::ActionFn action,
                                    Block::FeasibleFn feasible) {
    if (m_stepCount >= MAX_STEPS) {
        Console::error("Mission") << "Max steps atteint (" << MAX_STEPS << ")" << Console::endl;
        return *this;
    }
    Step& s      = m_steps[m_stepCount++];
    s.name        = name;
    s.estimatedMs = estimatedMs;
    s.action      = action;
    s.feasible    = feasible;
    return *this;
}

FLASHMEM Mission& Mission::setFeasible(Block::FeasibleFn fn) {
    m_feasible = fn;
    return *this;
}

FLASHMEM Mission& Mission::setMaxRetries(uint8_t n) {
    m_maxRetries = n;
    return *this;
}

FLASHMEM uint32_t Mission::remainingEstimatedMs() const {
    uint32_t total = 0;
    for (uint8_t i = m_currentStep; i < m_stepCount; i++)
        total += m_steps[i].estimatedMs;
    return total;
}

FLASHMEM bool Mission::canRetry() const {
    return (m_state == MissionState::FAILED) && (m_retries < m_maxRetries);
}

FLASHMEM bool Mission::isCooledDown() const {
    if (m_lastAttemptMs == 0) return true;
    return (millis() - m_lastAttemptMs) >= RETRY_COOLDOWN_MS;
}


// ============================================================
//  Planner — Builder
// ============================================================

FLASHMEM Mission& Planner::addMission(const char* name, uint8_t priority, uint16_t score) {
    if (m_count >= MAX_MISSIONS) {
        Console::error("Planner") << "Max missions atteint (" << MAX_MISSIONS << ")" << Console::endl;
        // Return last slot to avoid crash — caller's steps will be ignored
        return m_missions[MAX_MISSIONS - 1];
    }
    Mission& m    = m_missions[m_count++];
    m              = Mission();   // clean slate
    m.m_name      = name;
    m.m_priority  = priority;
    m.m_score     = score;
    return m;
}

FLASHMEM Planner& Planner::setTimeProvider(uint32_t (*fn)()) {
    m_timeProvider = fn;
    return *this;
}

FLASHMEM Planner& Planner::setSafetyMargin(uint32_t ms) {
    m_safetyMs = ms;
    return *this;
}


// ============================================================
//  Planner::run() — Boucle principale
// ============================================================

FLASHMEM void Planner::run() {
    m_startMs = millis();
    m_score   = 0;

    Console::info("Planner")
        << "=== DEBUT | "
        << (int)m_count << " missions | "
        << "objectif " << (int)potentialScore() << " pts"
        << " ===" << Console::endl;

    while (true) {
        // ── OS stop check (match end, e-stop, etc.) ─────────────
        if (os.getState() == OS::STOPPED) {
            Console::info("Planner") << "OS stopped — sortie immediate." << Console::endl;
            break;
        }

        // ── Time check ──────────────────────────────────────────
        uint32_t rem = remainingMs();
        if (rem != UINT32_MAX && rem <= m_safetyMs) {
            Console::info("Planner") << "Plus de temps — sortie." << Console::endl;
            break;
        }

        // ── Select next mission ─────────────────────────────────
        Mission* m = selectNext();

        if (!m) {
            // Rien de faisable maintenant — on a encore des missions retryables ?
            if (hasRetryable()) {
                // Attendre un peu : les zones peuvent se libérer, les cooldowns expirer
                waitMs(IDLE_WAIT_MS);
                continue;
            }
            // Tout est DONE ou ABANDONED
            Console::info("Planner") << "Plus de missions disponibles." << Console::endl;
            break;
        }

        // ── Execute ─────────────────────────────────────────────
        executeMission(*m);
    }

    logSummary();
}


// ============================================================
//  Planner::selectNext()
// ============================================================

FLASHMEM Mission* Planner::selectNext() {
    Mission* best      = nullptr;
    float    bestValue = -1.0f;

    for (uint8_t i = 0; i < m_count; i++) {
        Mission& m = m_missions[i];

        // Skip terminées
        if (m.m_state == MissionState::DONE || m.m_state == MissionState::ABANDONED)
            continue;

        // Skip en cooldown
        if (m.m_state == MissionState::FAILED && !m.isCooledDown())
            continue;

        // Skip si plus assez de temps
        if (!canFitMission(m))
            continue;

        // Mission-level feasibility
        if (m.m_feasible && !m.m_feasible())
            continue;

        // Scoring : priorité dominante, score en tie-breaker
        float value = (float)m.m_priority * 1000.0f + (float)m.m_score;

        if (value > bestValue) {
            best      = &m;
            bestValue = value;
        }
    }

    return best;
}


// ============================================================
//  Planner::executeMission()
// ============================================================

FLASHMEM void Planner::executeMission(Mission& m) {
    m.m_lastAttemptMs = millis();

    Console::info("Planner")
        << "[" << m.m_name << "] "
        << "step " << (int)(m.m_currentStep + 1) << "/" << (int)m.m_stepCount
        << "  prio=" << (int)m.m_priority
        << "  retry=" << (int)m.m_retries << "/" << (int)m.m_maxRetries
        << "  restant ~" << (int)(remainingMs() == UINT32_MAX ? 9999 : remainingMs() / 1000) << "s"
        << Console::endl;

    for (uint8_t i = m.m_currentStep; i < m.m_stepCount; i++) {
        // ── OS stop check between steps ─────────────────────────
        if (os.getState() == OS::STOPPED) {
            Console::info("Planner")
                << "  [" << m.m_name << "] OS stopped — abort mission" << Console::endl;
            m.m_state = MissionState::ABANDONED;
            return;
        }

        Step& step = m.m_steps[i];

        // ── Per-step feasibility ────────────────────────────────
        if (step.feasible && !step.feasible()) {
            Console::warn("Planner")
                << "  [" << step.name << "] infaisable — mission suspendue" << Console::endl;
            // Ne PAS incrémenter retries (c'est temporaire, pas un échec)
            m.m_state = MissionState::FAILED;
            return;
        }

        // ── Execute step ────────────────────────────────────────
        uint32_t stepStart = millis();

        Console::info("Planner")
            << "  [" << step.name << "] executing..." << Console::endl;

        if (!step.action) {
            Console::error("Planner") << "  [" << step.name << "] null action!" << Console::endl;
            m.m_retries++;
            m.m_state = (m.m_retries >= m.m_maxRetries)
                ? MissionState::ABANDONED : MissionState::FAILED;
            return;
        }

        BlockResult result = step.action();
        uint32_t elapsed   = millis() - stepStart;

        if (result == BlockResult::FAILED) {
            // Si l'OS est stoppé (match end), c'est un abort, pas un vrai fail
            if (os.getState() == OS::STOPPED) {
                Console::info("Planner")
                    << "  [" << step.name << "] aborted (OS stopped)" << Console::endl;
                m.m_state = MissionState::ABANDONED;
                return;
            }
            Console::warn("Planner")
                << "  [" << step.name << "] FAILED (" << (int)(elapsed / 1000) << "s)"
                << Console::endl;
            m.m_retries++;
            m.m_state = (m.m_retries >= m.m_maxRetries)
                ? MissionState::ABANDONED : MissionState::FAILED;
            return;
        }

        // ── Step succeeded — advance ────────────────────────────
        m.m_currentStep = i + 1;
        Console::info("Planner")
            << "  [" << step.name << "] OK (" << (int)(elapsed / 1000) << "s)"
            << Console::endl;
    }

    // ── All steps done ──────────────────────────────────────────
    m.m_state = MissionState::DONE;
    m_score  += m.m_score;
    Console::info("Planner")
        << "[" << m.m_name << "] DONE +" << (int)m.m_score << "pts"
        << "  TOTAL=" << (int)m_score << "pts"
        << Console::endl;
}


// ============================================================
//  Helpers
// ============================================================

FLASHMEM bool Planner::canFitMission(const Mission& m) const {
    uint32_t est = m.remainingEstimatedMs();
    if (est == 0) return true;
    uint32_t rem = remainingMs();
    if (rem == UINT32_MAX) return true;
    return (est + m_safetyMs) <= rem;
}

FLASHMEM uint32_t Planner::remainingMs() const {
    if (!m_timeProvider) return UINT32_MAX;
    long t = (long)m_timeProvider();
    return (t > 0) ? (uint32_t)t : 0u;
}

FLASHMEM bool Planner::hasRetryable() const {
    for (uint8_t i = 0; i < m_count; i++) {
        const Mission& m = m_missions[i];
        if (m.m_state == MissionState::PENDING) return true;
        if (m.m_state == MissionState::FAILED && m.canRetry()) return true;
    }
    return false;
}


// ============================================================
//  logSummary()
// ============================================================

FLASHMEM void Planner::logSummary() const {
    Console::info("Planner") << "── Résumé ──" << Console::endl;

    for (uint8_t i = 0; i < m_count; i++) {
        const Mission& m = m_missions[i];
        const char* tag;
        switch (m.m_state) {
            case MissionState::DONE:      tag = "DONE";      break;
            case MissionState::FAILED:    tag = "FAILED";    break;
            case MissionState::ABANDONED: tag = "ABANDONED"; break;
            default:                      tag = "PENDING";   break;
        }
        Console::info("Planner")
            << "  [" << tag << "] " << m.m_name
            << "  step " << (int)m.m_currentStep << "/" << (int)m.m_stepCount
            << "  retries=" << (int)m.m_retries
            << Console::endl;
    }

    Console::info("Planner")
        << "=== FIN | "
        << (int)m_score << "/" << (int)potentialScore() << " pts | "
        << (int)doneCount() << " DONE, "
        << (int)failedCount() << " FAIL, "
        << (int)abandonedCount() << " ABANDON, "
        << (int)pendingCount() << " PENDING"
        << " | " << (int)(elapsedMs() / 1000) << "s"
        << " ===" << Console::endl;
}


// ============================================================
//  Diagnostics
// ============================================================

FLASHMEM uint16_t Planner::totalScore() const { return m_score; }

FLASHMEM uint16_t Planner::potentialScore() const {
    uint16_t total = 0;
    for (uint8_t i = 0; i < m_count; i++) total += m_missions[i].m_score;
    return total;
}

FLASHMEM uint8_t Planner::totalMissions() const { return m_count; }

FLASHMEM uint8_t Planner::doneCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < m_count; i++)
        if (m_missions[i].m_state == MissionState::DONE) n++;
    return n;
}

FLASHMEM uint8_t Planner::failedCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < m_count; i++)
        if (m_missions[i].m_state == MissionState::FAILED) n++;
    return n;
}

FLASHMEM uint8_t Planner::abandonedCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < m_count; i++)
        if (m_missions[i].m_state == MissionState::ABANDONED) n++;
    return n;
}

FLASHMEM uint8_t Planner::pendingCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < m_count; i++)
        if (m_missions[i].m_state == MissionState::PENDING) n++;
    return n;
}

FLASHMEM uint32_t Planner::elapsedMs() const { return millis() - m_startMs; }
