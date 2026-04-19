#pragma once
#include "utils/geometry.h"
#include "config/settings.h"

// ============================================================
//  StallDetector — three complementary stall detection methods
//
//  1) Sliding-window displacement (legacy):
//     Checks if robot has moved enough in a given time window.
//     Robust but slow to trigger (needs DELAY_MS + PERIOD_MS).
//
//  2) Velocity mismatch:
//     Compares commanded velocity to OTOS-measured velocity.
//     Fast response: triggers within ~200ms of wall contact.
//     Works per-axis: if robot hits Y wall, only Y stalls.
//     Limitation: misses low-speed stalls where cmdVel < threshold.
//
//  3) Position stagnation:
//     Tracks per-axis position change over a time window.
//     If position doesn't move while PID error remains → stall.
//     Catches low-speed stalls that velocity mismatch misses.
//
//  Usage:
//    m_stall.begin(startPos, target);
//    m_stall.updateVelocity(cmdVel, otosVel, dt);  // called at PID rate
//    m_stall.updateStagnation(pos, target, dt);     // called at PID rate
//    if (m_stall.update(pos, elapsedMs)) { … }     // called at slow rate
//    auto s = m_stall.getStats();
//
//  The per-axis stall info is available for wall-probing (recalage):
//    if (m_stall.isStalledX()) → hit wall on X axis
//    if (m_stall.isStalledY()) → hit wall on Y axis
// ============================================================

class StallDetector {
public:

    // ---- Tunable parameters ----
    // Source unique des defaults : Settings::Motion::Stall (config/settings.h).
    // Runtime override possible via RuntimeConfig (voir PositionController::start()).
    struct Config {
        // Sliding-window (legacy)
        uint32_t delayMs        = Settings::Motion::Stall::DELAY_MS;
        uint32_t periodMs       = Settings::Motion::Stall::PERIOD_MS;
        float    transDispMm    = Settings::Motion::Stall::TRANS_DISP_MM;
        float    angleDispRad   = Settings::Motion::Stall::ANGLE_DISP_RAD;
        float    targetTransMm  = Settings::Motion::Stall::TARGET_TRANS_MM;
        float    targetAngleRad = Settings::Motion::Stall::TARGET_ANGLE_RAD;

        // Velocity mismatch (cmd vs OTOS)
        float    velCmdMinMmS   = Settings::Motion::Stall::VEL_CMD_MIN_MMS;
        float    velOtosMaxMmS  = Settings::Motion::Stall::VEL_OTOS_MAX_MMS;
        float    velStallTimeS  = Settings::Motion::Stall::VEL_STALL_TIME_S;
        float    velRotMinRadS  = Settings::Motion::Stall::VEL_ROT_MIN_RADS;
        float    velRotMaxRadS  = Settings::Motion::Stall::VEL_ROT_MAX_RADS;

        // Error stagnation — catches low-speed stalls
        float    stagMoveMm     = Settings::Motion::Stall::STAG_MOVE_MM;
        float    stagTimeS      = Settings::Motion::Stall::STAG_TIME_S;
        float    stagErrorMm    = Settings::Motion::Stall::STAG_ERROR_MM;
    };

    // ---- Move statistics ----
    struct Stats {
        int   checks       = 0;
        float minTransMm   = 1e9f;
        float minAngleDeg  = 1e9f;
        bool  triggered    = false;

        // Velocity-based stall
        bool  velTriggered = false;
        bool  stalledX     = false;
        bool  stalledY     = false;
        bool  stalledRot   = false;

        // Per-axis trigger cause (for debugging / reporting).
        // Set once when a stagnation/velocity fires, never cleared by the live
        // detector — only begin() resets them.
        bool  causeStagX   = false;
        bool  causeStagY   = false;
        bool  causeVelX    = false;
        bool  causeVelY    = false;
        bool  causeVelRot  = false;
    };

    Config config;

    // ---- API ----
    void  begin(const Vec3& startPos, const Vec3& target);
    bool  update(const Vec3& currentPos, uint32_t elapsedMs);  // legacy displacement check
    void  updateVelocity(const Vec3& cmdVel, const Vec3& otosVel, float dt);  // velocity mismatch
    void  updateStagnation(const Vec3& pos, const Vec3& target, float dt);    // position stagnation
    void  reset();
    Stats getStats() const { return m_stats; }

    // Per-axis stall state (for wall probing / recalage)
    bool isStalledX()   const { return m_stats.stalledX; }
    bool isStalledY()   const { return m_stats.stalledY; }
    bool isStalledRot() const { return m_stats.stalledRot; }
    bool isStalledAny() const { return m_stats.stalledX || m_stats.stalledY || m_stats.stalledRot; }

    // Clear stall state on a single axis (after border snap) to prevent re-trigger.
    // Also recomputes velTriggered so the legacy displacement check doesn't re-fire.
    void clearStalledX() {
        m_stats.stalledX = false; m_velStallAccumX = 0.0f;
        m_stagAccumX = 0.0f; m_stagBaseErrX = 0.0f;
        m_stats.velTriggered = m_stats.stalledX || m_stats.stalledY || m_stats.stalledRot;
    }
    void clearStalledY() {
        m_stats.stalledY = false; m_velStallAccumY = 0.0f;
        m_stagAccumY = 0.0f; m_stagBaseErrY = 0.0f;
        m_stats.velTriggered = m_stats.stalledX || m_stats.stalledY || m_stats.stalledRot;
    }

    static float shortestAngleDiff(float a, float b);

private:
    Vec3     m_startPos;
    Vec3     m_target;
    Vec3     m_lastPos;
    uint32_t m_lastCheckMs = 0;
    Stats    m_stats;

    // Expected travel per axis, captured at begin() — used to gate
    // stagnation checks: an axis that wasn't supposed to move much
    // won't be allowed to trigger a stall.
    float    m_expectedTravelX = 0.0f;
    float    m_expectedTravelY = 0.0f;

    // Velocity mismatch accumulators (time spent in mismatch state)
    float m_velStallAccumX   = 0.0f;
    float m_velStallAccumY   = 0.0f;
    float m_velStallAccumRot = 0.0f;

    // Error stagnation accumulators — tracks whether PID error is decreasing.
    // Compares current error to a BASELINE captured when stagnation started,
    // not to the previous snapshot. This prevents oscillation (robot vibrating
    // against a wall) from resetting the accumulator.
    float m_stagBaseErrX = 0.0f;  // error when stagnation period started
    float m_stagBaseErrY = 0.0f;
    float m_stagAccumX   = 0.0f;  // time spent stagnating on X
    float m_stagAccumY   = 0.0f;
    float m_stagWindowAccumX = 0.0f;
    float m_stagWindowAccumY = 0.0f;
    static constexpr float STAG_SNAPSHOT_PERIOD = 0.10f;  // check every 100ms
};
