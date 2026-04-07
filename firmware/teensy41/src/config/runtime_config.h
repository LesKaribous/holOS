#pragma once
/**
 * runtime_config.h — Key-value runtime configuration stored on SD card.
 *
 * Provides a lightweight key=value store that is:
 *   - Loaded from /config.cfg on SD at boot (after SDCard::init())
 *   - Modifiable at runtime via commands (cfg_set, cfg_save, cfg_list)
 *   - Saved back to SD on demand
 *
 * Values are stored as strings internally. Typed getters (getInt, getFloat)
 * parse on read and fall back to a caller-supplied default.
 *
 * Maximum capacity: 32 entries (fits actuator limits + headroom for future use).
 *
 * File format (/config.cfg):
 *   # comment lines ignored
 *   key=value
 *   servo.CA.0.min=110
 *   servo.CA.0.max=170
 *   motion.max_speed=2800
 */

#include <Arduino.h>

namespace RuntimeConfig {

    /// Load /config.cfg from SD card.  Safe to call if SD is not ready (no-op).
    void load();

    /// Save all current key=value pairs to /config.cfg on SD.
    bool save();

    /// Get a string value by key, or `defaultVal` if not found.
    const char* getString(const char* key, const char* defaultVal = "");

    /// Get an integer value by key, or `defaultVal` if not found / parse error.
    int getInt(const char* key, int defaultVal);

    /// Get a float value by key, or `defaultVal` if not found / parse error.
    float getFloat(const char* key, float defaultVal);

    /// Set a key=value pair (string).  Overwrites existing key if present.
    /// Returns false if the store is full and the key is new.
    bool set(const char* key, const char* value);

    /// Convenience: set an integer value.
    bool setInt(const char* key, int value);

    /// Convenience: set a float value.
    bool setFloat(const char* key, float value);

    /// Print all entries to console (for cfg_list command).
    void printAll();

    /// Serialize all entries into buf as "key1=val1;key2=val2;...".
    /// Returns the number of chars written (excluding NUL terminator).
    int serialize(char* buf, int bufSize);

    /// Number of entries currently stored.
    int count();

}  // namespace RuntimeConfig
