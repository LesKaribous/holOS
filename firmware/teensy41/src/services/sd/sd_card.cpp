#include "sd_card.h"
#include "config/calibration.h"
#include "os/console.h"
#include <SD.h>

namespace SDCard {

// ── Internal state ────────────────────────────────────────────────────────────

static bool s_ready    = false;
static File s_writeFile;
static bool s_writeOpen = false;

static constexpr const char* CALIB_PATH = "/calibration.cfg";

// ── Public API ────────────────────────────────────────────────────────────────

bool init() {
    // BUILTIN_SDCARD is defined by Teensy 4.1 core as the CS pin (254)
    s_ready = SD.begin(BUILTIN_SDCARD);
    if (s_ready)
        Console::success("SDCard") << "SD card mounted (BUILTIN_SDCARD)" << Console::endl;
    else
        Console::warn("SDCard") << "SD card not found — calibration will not persist" << Console::endl;
    return s_ready;
}

bool isReady() {
    return s_ready;
}

bool save(const char* path, const char* content) {
    if (!s_ready) {
        Console::error("SDCard") << "Not ready — cannot save " << path << Console::endl;
        return false;
    }
    // Remove existing file so FILE_WRITE starts from offset 0
    if (SD.exists(path)) SD.remove(path);

    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Console::error("SDCard") << "Cannot open for write: " << path << Console::endl;
        return false;
    }
    f.print(content);
    f.close();
    return true;
}

bool load(const char* path, char* buf, size_t bufSize) {
    if (!s_ready) {
        Console::error("SDCard") << "Not ready — cannot load " << path << Console::endl;
        return false;
    }
    File f = SD.open(path);
    if (!f) {
        Console::warn("SDCard") << "File not found: " << path << Console::endl;
        return false;
    }
    size_t n = f.readBytes(buf, bufSize - 1);
    buf[n] = '\0';
    f.close();
    return (n > 0);
}

bool saveCalibration() {
    char buf[256];
    Calibration::toString(buf, sizeof(buf));
    bool ok = save(CALIB_PATH, buf);
    if (ok)
        Console::success("SDCard") << "Calibration saved → " << buf << Console::endl;
    return ok;
}

bool loadCalibration() {
    char buf[256];
    if (!load(CALIB_PATH, buf, sizeof(buf))) return false;

    bool ok = Calibration::fromString(buf);
    if (ok)
        Console::success("SDCard") << "Calibration loaded: " << buf << Console::endl;
    else
        Console::error("SDCard") << "Calibration parse failed: " << buf << Console::endl;
    return ok;
}

// ── Sequential write session ──────────────────────────────────────────────────

FLASHMEM bool openWrite(const char* path) {
    if (!s_ready) {
        Console::error("SDCard") << "Not ready — cannot open " << path << " for write" << Console::endl;
        return false;
    }
    if (s_writeOpen) {
        s_writeFile.close();
        s_writeOpen = false;
    }
    if (SD.exists(path)) SD.remove(path);
    s_writeFile = SD.open(path, FILE_WRITE);
    if (!s_writeFile) {
        Console::error("SDCard") << "Cannot create: " << path << Console::endl;
        return false;
    }
    s_writeOpen = true;
    return true;
}

FLASHMEM bool appendLine(const char* line) {
    if (!s_writeOpen) return false;
    s_writeFile.println(line);
    return true;
}

FLASHMEM bool closeWrite() {
    if (!s_writeOpen) return false;
    s_writeFile.close();
    s_writeOpen = false;
    return true;
}

}  // namespace SDCard
