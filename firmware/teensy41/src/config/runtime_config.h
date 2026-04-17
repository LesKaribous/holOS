#pragma once
/**
 * runtime_config.h — Key-value runtime configuration (in-memory).
 *
 * Provides a lightweight key=value store that is:
 *   - Populated at runtime by holOS via cfg_set commands over the bridge
 *   - Modifiable at runtime via commands (cfg_set, cfg_list)
 *   - NOT persisted on-board — holOS is the source of truth for settings
 *
 * Values are stored as strings internally. Typed getters (getInt, getFloat)
 * parse on read and fall back to a caller-supplied default.
 *
 * Maximum capacity: 32 entries (fits actuator limits + headroom for future use).
 */

#include <Arduino.h>

namespace RuntimeConfig {

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
