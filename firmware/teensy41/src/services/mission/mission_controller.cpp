/**
 * services/mission/mission_controller.cpp — Fallback Mission Executor
 *
 * Executes a pre-defined strategy pushed by holOS as a sequence of
 * firmware commands.  SD card support has been removed — all storage
 * is in-memory only.
 *
 * The strategy is a plain-text sequence:
 *   # Comment
 *   go(300,400)
 *   turn(90.0)
 *   delay(500)
 *   actuator_cmd(...)
 *
 * Execution model:
 *   - One command per line (comments stripped)
 *   - Motion commands (go, turn, goPolar, align) are executed synchronously:
 *     command is dispatched, then we poll motion.hasFinished() up to 30 s
 *   - delay(ms) is implemented natively (busy-wait with delay())
 *   - All other commands are dispatched and assumed instantaneous
 */

#include "mission_controller.h"
#include "services/motion/motion.h"
#include "os/console.h"
#include "os/os.h"
#include "utils/commandHandler.h"

#include <Arduino.h>

namespace MissionController {

// ── Internal state ────────────────────────────────────────────────────────────

static constexpr size_t MAX_LINES = 256;
static constexpr size_t LINE_LEN  = MAX_LINE_LEN;

/// Loaded command lines (stripped, no comments).
/// DMAMEM → placed in RAM2 (OCRAM, 512 KB free) instead of RAM1 (DTCM, 512 KB total).
static DMAMEM char  s_cmds[MAX_LINES][LINE_LEN];
static size_t s_cmdCount   = 0;
static bool  s_loaded     = false;
static bool  s_running    = false;
static bool  s_abort      = false;

// ── In-memory write session state ─────────────────────────────────────────────
static DMAMEM char s_writeBuf[MAX_FILE_SIZE];
static size_t s_writePos   = 0;
static bool   s_writeOpen  = false;

// ── Motion-command detection ──────────────────────────────────────────────────

static const char* MOTION_PREFIXES[] = {
    "go(", "go_coc(", "goPolar(", "turn(", "align(", "goAlign(", "move(", nullptr
};

static bool isMotionCmd(const char* line) {
    for (int i = 0; MOTION_PREFIXES[i]; ++i) {
        if (strncmp(line, MOTION_PREFIXES[i], strlen(MOTION_PREFIXES[i])) == 0) {
            return true;
        }
    }
    return false;
}

// ── delay(ms) parsing ─────────────────────────────────────────────────────────

static bool tryDelay(const char* line) {
    if (strncmp(line, "delay(", 6) != 0) return false;
    int ms = atoi(line + 6);
    if (ms > 0 && ms < 60000) delay((unsigned long)ms);
    return true;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static void stripLine(const char* src, char* dst, size_t dstLen) {
    size_t i = 0, j = 0;
    while (src[i] && src[i] != '#' && j + 1 < dstLen) {
        dst[j++] = src[i++];
    }
    while (j > 0 && (dst[j-1] == ' ' || dst[j-1] == '\t' ||
                     dst[j-1] == '\r' || dst[j-1] == '\n')) {
        j--;
    }
    dst[j] = '\0';
}

static bool parseBuffer(const char* buf) {
    s_loaded   = false;
    s_cmdCount = 0;

    char line[LINE_LEN];
    const char* p = buf;
    while (*p && s_cmdCount < MAX_LINES) {
        int len = 0;
        while (*p && *p != '\n' && *p != '\r' && len + 1 < (int)LINE_LEN) {
            line[len++] = *p++;
        }
        line[len] = '\0';
        while (*p == '\n' || *p == '\r') p++;

        char stripped[LINE_LEN];
        stripLine(line, stripped, sizeof(stripped));
        if (stripped[0] == '\0') continue;

        strncpy(s_cmds[s_cmdCount], stripped, LINE_LEN - 1);
        s_cmds[s_cmdCount][LINE_LEN - 1] = '\0';
        s_cmdCount++;
    }

    s_loaded = s_cmdCount > 0;
    Console::info("Mission") << "Parsed " << s_cmdCount << " commands" << Console::endl;
    return s_loaded;
}

// ── Public API ────────────────────────────────────────────────────────────────

FLASHMEM bool loadFromString(const char* content) {
    return parseBuffer(content);
}

bool isLoaded() { return s_loaded; }
bool isRunning() { return s_running; }

void abort() {
    s_abort = true;
    motion.cancel();
}

FLASHMEM void execute() {
    if (!s_loaded || s_cmdCount == 0) {
        Console::warn("Mission") << "No strategy loaded — aborting fallback" << Console::endl;
        return;
    }

    s_running = true;
    s_abort   = false;

    Console::info("Mission") << "═══ Fallback strategy start ("
                              << s_cmdCount << " cmds) ═══" << Console::endl;

    for (size_t i = 0; i < s_cmdCount && !s_abort; ++i) {
        const char* cmd = s_cmds[i];
        Console::info("Mission") << "[" << (i+1) << "/" << s_cmdCount << "] " << cmd << Console::endl;

        if (tryDelay(cmd)) continue;

        if (isMotionCmd(cmd)) {
            String scmd(cmd);
            os.execute(scmd);
            unsigned long t0 = millis();
            while (!motion.hasFinished() && (millis() - t0) < 30000UL && !s_abort) {
                delay(20);
            }
            if (!motion.wasSuccessful()) {
                Console::warn("Mission") << "Motion failed — continuing" << Console::endl;
            }
            continue;
        }

        {
            String scmd(cmd);
            if (scmd.indexOf('(') < 0) scmd += "()";
            if (CommandHandler::hasCommand(scmd.substring(0, scmd.indexOf('(')))) {
                os.execute(scmd);
            } else {
                Console::warn("Mission") << "Unknown cmd: " << cmd << Console::endl;
            }
        }
    }

    if (s_abort) {
        Console::warn("Mission") << "═══ Fallback strategy ABORTED ═══" << Console::endl;
    } else {
        Console::info("Mission") << "═══ Fallback strategy DONE ═══" << Console::endl;
    }

    s_running = false;
    s_abort   = false;
}

// ── In-memory write session (replaces SD write) ─────────────────────────────

FLASHMEM bool memOpen() {
    s_writePos  = 0;
    s_writeOpen = true;
    s_writeBuf[0] = '\0';
    Console::info("Mission") << "Memory write session opened" << Console::endl;
    return true;
}

FLASHMEM bool memAppendLine(const char* line) {
    if (!s_writeOpen) return false;
    size_t len = strlen(line);
    if (s_writePos + len + 2 >= MAX_FILE_SIZE) {
        Console::error("Mission") << "Write buffer full" << Console::endl;
        return false;
    }
    memcpy(s_writeBuf + s_writePos, line, len);
    s_writePos += len;
    s_writeBuf[s_writePos++] = '\n';
    s_writeBuf[s_writePos] = '\0';
    return true;
}

FLASHMEM bool memClose() {
    if (!s_writeOpen) return false;
    s_writeOpen = false;
    Console::info("Mission") << "Memory write session closed. Parsing…" << Console::endl;
    return parseBuffer(s_writeBuf);
}

} // namespace MissionController
