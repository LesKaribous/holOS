#pragma once
#include "utils/geometry.h"
#include "config/settings.h"

// ============================================================
//  StallDetector — détection de blocage par déplacement/temps
//
//  Vérifie périodiquement si le robot a progressé sur une
//  fenêtre glissante. Robuste à la saturation PID et à la
//  dérive OTOS : seul le déplacement réel est mesuré.
//  Fonctionne pour translation ET rotation.
//
//  Usage :
//    m_stall.begin(startPos, target);           // au départ du move
//    if (m_stall.update(pos, elapsedMs)) { … }  // à chaque cycle lent
//    auto s = m_stall.getStats();               // après le move
//
//  Tuning :
//    Les seuils par défaut viennent de Settings::Motion::Stall.
//    Overridable à chaud via le champ public `config`.
//    Lancer un move avec withDiagnostics(), regarder stats.minTransMm
//    → c'est le déplacement minimal observé en conditions normales
//    → choisir TRANS_DISP_MM légèrement en-dessous.
// ============================================================

class StallDetector {
public:

    // ---- Paramètres tunables (modifiables à chaud) ----
    struct Config {
        uint32_t delayMs        = Settings::Motion::Stall::DELAY_MS;
        uint32_t periodMs       = Settings::Motion::Stall::PERIOD_MS;
        float    transDispMm    = Settings::Motion::Stall::TRANS_DISP_MM;
        float    angleDispRad   = Settings::Motion::Stall::ANGLE_DISP_RAD;
        float    targetTransMm  = Settings::Motion::Stall::TARGET_TRANS_MM;
        float    targetAngleRad = Settings::Motion::Stall::TARGET_ANGLE_RAD;
    };

    // ---- Statistiques d'un move (pour calibration) ----
    struct Stats {
        int   checks       = 0;      // fenêtres évaluées
        float minTransMm   = 1e9f;   // pire déplacement trans observé (mm)
        float minAngleDeg  = 1e9f;   // pire déplacement rot observé (deg)
        bool  triggered    = false;
    };

    Config config;  // tunables directement accessibles

    // ---- API ----
    void  begin(const Vec3& startPos, const Vec3& target);
    bool  update(const Vec3& currentPos, uint32_t elapsedMs);  // true = stall
    void  reset();
    Stats getStats() const { return m_stats; }

    static float shortestAngleDiff(float a, float b);

private:
    Vec3     m_startPos;
    Vec3     m_target;
    Vec3     m_lastPos;
    uint32_t m_lastCheckMs = 0;
    Stats    m_stats;
};
