#pragma once
/**
 * calibration.h — Runtime-mutable calibration profile for holOS.
 *
 * Replaces the compile-time `Settings::Calibration::Primary` const struct.
 * All kinematics (ik / fk) and the Localisation OTOS service now use the
 * values held here, which can be updated live and persisted to SD card.
 *
 * Calibration groups
 * ------------------
 *  Cartesian   { x, y, rot }  — Cartesian kinematic scale factors
 *                               applied in ik() / fk() before the stepper matrix.
 *                               Tune to correct systematic XY and rotation error.
 *
 *  Holonomic   { a, b, c }    — Per-wheel (stepper A/B/C) scale factors.
 *                               Tune to compensate individual wheel/motor differences.
 *
 *  OtosLinear                  — OTOS linear scalar  (range 0.872 – 1.127).
 *                               Set so that measured OTOS distance == physical distance.
 *
 *  OtosAngular                 — OTOS angular scalar (range 0.872 – 1.127).
 *                               Set so that OTOS angle == physical rotation.
 */

#include "settings.h"   // for CalibrationProfile, Vec3, Settings::Calibration::*

struct CalibrationProfile;

namespace Calibration {

    // ── Compile-time defaults ────────────────────────────────────────────────
    // Source unique : Settings::Calibration (voir config/settings.h).
    // Deux profils y cohabitent : Settings::Calibration::Stepper (open-loop)
    // et Settings::Calibration::Cruise (closed-loop / OTOS). DEFAULTS pointe
    // sur Cruise (mode par défaut).
    // Ces alias sont gardés pour compat descendante — le nouveau code
    // devrait référencer Settings::Calibration::{Stepper|Cruise|DEFAULTS}.
    inline const CalibrationProfile& DEFAULTS      = Settings::Calibration::DEFAULTS;
    constexpr float           OTOS_LINEAR_DEFAULT  = Settings::Calibration::OTOS_LINEAR_DEFAULT;
    constexpr float           OTOS_ANGULAR_DEFAULT = Settings::Calibration::OTOS_ANGULAR_DEFAULT;

    // ── Live (mutable) values — modified by calibration commands and SD load ──
    extern CalibrationProfile Current;
    extern float OtosLinear;
    extern float OtosAngular;

    // ── Mutators ──────────────────────────────────────────────────────────────

    /// Reset all calibration to compile-time defaults.
    void reset();

    /// Set Cartesian scale factors (X mm/mm, Y mm/mm, ROT rad/rad).
    void setCartesian(float x, float y, float rot);

    /// Set per-wheel holonomic scale factors.
    void setHolonomic(float a, float b, float c);

    /// Set OTOS linear scalar (applied to linear OTOS readings).
    /// Also updates the live Localisation service if available.
    void setOtosLinear(float value);

    /// Set OTOS angular scalar (applied to angular OTOS readings).
    void setOtosAngular(float value);

    /// Fill a buffer with a compact key=value string of all calibration values.
    /// Format: "cx=1.089,cy=-1.089,cr=0.831,ha=1.0,hb=1.0,hc=1.0,ol=0.9714,oa=1.0"
    /// Returns number of bytes written (excluding null terminator).
    int  toString(char* buf, size_t bufSize);

    /// Parse a key=value string (same format as toString) and apply values.
    /// Ignores unknown keys.  Returns true if at least one key was applied.
    bool fromString(const char* str);

}  // namespace Calibration
