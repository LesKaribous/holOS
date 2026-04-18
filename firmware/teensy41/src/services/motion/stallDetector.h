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
    struct Config {
        // Sliding-window (legacy)
        uint32_t delayMs        = Settings::Motion::Stall::DELAY_MS;
        uint32_t periodMs       = Settings::Motion::Stall::PERIOD_MS;
        float    transDispMm    = Settings::Motion::Stall::TRANS_DISP_MM;
        float    angleDispRad   = Settings::Motion::Stall::ANGLE_DISP_RAD;
        float    targetTransMm  = Settings::Motion::Stall::TARGET_TRANS_MM;
        float    targetAngleRad = Settings::Motion::Stall::TARGET_ANGLE_RAD;

        // Velocity mismatch
        float    velCmdMinMmS   = 30.0f;   // minimum commanded speed to consider (mm/s)
        float    velOtosMaxMmS  = 10.0f;   // OTOS speed below this = "not moving" (mm/s)
        float    velStallTimeS  = 0.20f;   // sustained mismatch duration to trigger (s)
        float    velRotMinRadS  = 0.3f;    // minimum commanded rot speed (rad/s)
        float    velRotMaxRadS  = 0.1f;    // OTOS rot speed below this = "not turning" (rad/s)

        // Error stagnation — catches low-speed stalls
        float    stagMoveMm     = 0.5f;    // error must decrease by at least this much per 100ms window
        float    stagTimeS      = 0.40f;   // stagnation duration to trigger stall (s)
        float    stagErrorMm    = 3.0f;    // minimum PID error to consider (ignore settling noise)
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
        m_stats.stalledX = false; m_velStallAccumX = 0.0f; m_stagAccumX = 0.0f;
        m_stats.velTriggered = m_stats.stalledX || m_stats.stalledY || m_stats.stalledRot;
    }
    void clearStalledY() {
        m_stats.stalledY = false; m_velStallAccumY = 0.0f; m_stagAccumY = 0.0f;
        m_stats.velTriggered = m_stats.stalledX || m_stats.stalledY || m_stats.stalledRot;
    }

    static float shortestAngleDiff(float a, float b);

private:
    Vec3     m_startPos;
    Vec3     m_target;
    Vec3     m_lastPos;
    uint32_t m_lastCheckMs = 0;
    Stats    m_stats;

    // Velocity mismatch accumulators (time spent in mismatch state)
    float m_velStallAccumX   = 0.0f;
    float m_velStallAccumY   = 0.0f;
    float m_velStallAccumRot = 0.0f;

    // Error stagnation accumulators — tracks whether PID error is decreasing.
    // If error doesn't decrease by stagMoveMm per window → stagnating.
    float m_stagLastErrX = 0.0f;  // snapshot of |error| at start of window
    float m_stagLastErrY = 0.0f;
    float m_stagAccumX   = 0.0f;  // time spent stagnating on X
    float m_stagAccumY   = 0.0f;
    float m_stagWindowAccumX = 0.0f;  // time since last snapshot (X)
    float m_stagWindowAccumY = 0.0f;
    static constexpr float STAG_SNAPSHOT_PERIOD = 0.10f;  // re-snapshot every 100ms
};
