#pragma once
/**
 * sd_card.h — DEPRECATED — SD card support has been removed.
 *
 * Settings and calibration are now persisted holOS-side (Python JSON store)
 * and pushed to firmware via cfg_set on every connect.
 *
 * This file is kept as an empty stub so that any stale #include does not
 * break the build.  All functions are no-ops.
 */

#include <Arduino.h>

namespace SDCard {
    inline bool init()       { return false; }
    inline bool isReady()    { return false; }
    inline bool save(const char*, const char*)           { return false; }
    inline bool load(const char*, char*, size_t)         { return false; }
    inline bool saveCalibration()                        { return false; }
    inline bool loadCalibration()                        { return false; }
    inline bool openWrite(const char*)                   { return false; }
    inline bool appendLine(const char*)                  { return false; }
    inline bool closeWrite()                             { return false; }
}
