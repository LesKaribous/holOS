#pragma once
/**
 * sd_card.h — SD card utility module for holOS (Teensy 4.1 built-in slot).
 *
 * Usage
 * -----
 *   // In onRobotBoot() — before calibration commands are registered:
 *   SDCard::init();
 *   SDCard::loadCalibration();   // restore last saved profile
 *
 *   // From calibration commands:
 *   SDCard::saveCalibration();
 *   SDCard::loadCalibration();
 *
 * File format
 * -----------
 *   /calibration.cfg  — Calibration::toString() key=value string, one line.
 *   e.g. "cx=1.089,cy=-1.089,cr=0.831,ha=1.0,hb=1.0,hc=1.0,ol=0.9714,oa=1.0"
 *
 * Notes
 * -----
 *   - Uses BUILTIN_SDCARD (Teensy 4.1 built-in micro-SD slot, pin 254).
 *   - Not a Service — no polling loop needed; purely on-demand I/O.
 *   - All public functions are safe to call even if SD is not present
 *     (they return false and log an error).
 */

#include <Arduino.h>

namespace SDCard {

    /// Initialise the SD card.  Call once during boot.
    /// Returns true if the card was found and mounted successfully.
    bool init();

    /// True if the SD card was successfully initialised.
    bool isReady();

    // ── Generic helpers ──────────────────────────────────────────────────────

    /// Write `content` (null-terminated string) to `path`.
    /// Overwrites existing file.  Returns true on success.
    bool save(const char* path, const char* content);

    /// Read up to `bufSize-1` bytes from `path` into `buf`.
    /// Null-terminates the result.  Returns true if data was read.
    bool load(const char* path, char* buf, size_t bufSize);

    // ── Calibration helpers ──────────────────────────────────────────────────

    /// Serialize Calibration::Current + OtosLinear/OtosAngular to
    /// "/calibration.cfg".  Returns true on success.
    bool saveCalibration();

    /// Load "/calibration.cfg" and apply via Calibration::fromString().
    /// Returns true if at least one key was applied.
    bool loadCalibration();

}  // namespace SDCard
