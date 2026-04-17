#pragma once
/**
 * services/mission/mission_controller.h — Fallback Mission Executor
 *
 * Executes a pre-defined strategy pushed by holOS (or loaded from memory).
 * The strategy is a plain-text list of firmware commands (one per line).
 *
 * SD card support has been removed — holOS is the source of truth.
 * Commands can be pushed at runtime via mission_load(text) or via
 * the legacy mission_sd_open/line/close protocol (now in-memory only).
 *
 * Usage from routines / fallback:
 *   MissionController::execute();   // blocking — runs until end or abort
 *   MissionController::abort();     // call from another thread / ISR-safe
 */

#include <Arduino.h>

namespace MissionController {

    static constexpr size_t MAX_FILE_SIZE = 8192;   // 8 KB limit
    static constexpr size_t MAX_LINE_LEN  = 128;

    /// Load commands from a string buffer (newline-separated).
    /// Returns true if at least one command was parsed.
    bool loadFromString(const char* content);

    /// Returns true if a valid strategy is currently loaded.
    bool isLoaded();

    /// Execute the loaded strategy sequentially. Blocking call —
    /// runs until all commands are done or abort() is requested.
    void execute();

    /// Abort an in-progress execute(). Thread-safe (sets a flag).
    void abort();

    /// Returns true if execute() is currently running.
    bool isRunning();

    // ── In-memory write API (replaces SD write session) ─────────────────────
    // Used by firmware commands (backward compatible with holOS push protocol).

    /// Start a new write session (clears any previous commands).
    bool memOpen();

    /// Append one line of text to the in-memory buffer.
    bool memAppendLine(const char* line);

    /// Finalize the write session and parse commands into RAM.
    bool memClose();

    /// Legacy alias — kept so existing code compiles.
    inline bool load() { return false; }  // no-op, SD removed

} // namespace MissionController
