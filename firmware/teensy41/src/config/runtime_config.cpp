#include "runtime_config.h"
#include "services/sd/sd_card.h"
#include "os/console.h"
#include <cstring>

namespace RuntimeConfig {

// ── Internal storage ─────────────────────────────────────────────────────────

static constexpr int MAX_ENTRIES  = 32;
static constexpr int MAX_KEY_LEN  = 32;
static constexpr int MAX_VAL_LEN  = 32;
static constexpr const char* CONFIG_PATH = "/config.cfg";

struct Entry {
    char key[MAX_KEY_LEN];
    char val[MAX_VAL_LEN];
    bool used = false;
};

static Entry s_entries[MAX_ENTRIES];
static int   s_count = 0;

// ── Internal helpers ─────────────────────────────────────────────────────────

static int findKey(const char* key) {
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (s_entries[i].used && strncmp(s_entries[i].key, key, MAX_KEY_LEN) == 0)
            return i;
    }
    return -1;
}

static int findFreeSlot() {
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (!s_entries[i].used) return i;
    }
    return -1;
}

// ── Public API ───────────────────────────────────────────────────────────────

void load() {
    // Reset store
    for (int i = 0; i < MAX_ENTRIES; i++) s_entries[i].used = false;
    s_count = 0;

    char buf[1024];
    if (!SDCard::load(CONFIG_PATH, buf, sizeof(buf))) {
        Console::info("Config") << "No config file on SD — using defaults" << Console::endl;
        return;
    }

    // Parse line by line
    char* line = strtok(buf, "\n\r");
    while (line) {
        // Skip comments and empty lines
        while (*line == ' ' || *line == '\t') line++;
        if (*line == '#' || *line == '\0') {
            line = strtok(nullptr, "\n\r");
            continue;
        }

        char* eq = strchr(line, '=');
        if (eq) {
            *eq = '\0';
            const char* key = line;
            const char* val = eq + 1;
            // Trim trailing whitespace from key
            char* end = eq - 1;
            while (end > key && (*end == ' ' || *end == '\t')) { *end = '\0'; end--; }
            set(key, val);
        }
        line = strtok(nullptr, "\n\r");
    }

    Console::success("Config") << "Loaded " << s_count << " entries from SD" << Console::endl;
}

bool save() {
    // Build content string
    char buf[1024];
    int pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "# holOS runtime config\n");

    for (int i = 0; i < MAX_ENTRIES && pos < (int)sizeof(buf) - 64; i++) {
        if (s_entries[i].used) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%s=%s\n",
                            s_entries[i].key, s_entries[i].val);
        }
    }

    bool ok = SDCard::save(CONFIG_PATH, buf);
    if (ok)
        Console::success("Config") << "Saved " << s_count << " entries to SD" << Console::endl;
    else
        Console::error("Config") << "Failed to save config to SD" << Console::endl;
    return ok;
}

const char* getString(const char* key, const char* defaultVal) {
    int idx = findKey(key);
    return (idx >= 0) ? s_entries[idx].val : defaultVal;
}

int getInt(const char* key, int defaultVal) {
    int idx = findKey(key);
    if (idx < 0) return defaultVal;
    return atoi(s_entries[idx].val);
}

float getFloat(const char* key, float defaultVal) {
    int idx = findKey(key);
    if (idx < 0) return defaultVal;
    return atof(s_entries[idx].val);
}

bool set(const char* key, const char* value) {
    int idx = findKey(key);
    if (idx >= 0) {
        strncpy(s_entries[idx].val, value, MAX_VAL_LEN - 1);
        s_entries[idx].val[MAX_VAL_LEN - 1] = '\0';
        return true;
    }
    // New key
    idx = findFreeSlot();
    if (idx < 0) {
        Console::error("Config") << "Store full — cannot add key: " << key << Console::endl;
        return false;
    }
    strncpy(s_entries[idx].key, key, MAX_KEY_LEN - 1);
    s_entries[idx].key[MAX_KEY_LEN - 1] = '\0';
    strncpy(s_entries[idx].val, value, MAX_VAL_LEN - 1);
    s_entries[idx].val[MAX_VAL_LEN - 1] = '\0';
    s_entries[idx].used = true;
    s_count++;
    return true;
}

bool setInt(const char* key, int value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    return set(key, buf);
}

bool setFloat(const char* key, float value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.4f", value);
    return set(key, buf);
}

void printAll() {
    Console::info("Config") << "Runtime config (" << s_count << " entries):" << Console::endl;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (s_entries[i].used) {
            Console::info("Config") << "  " << s_entries[i].key << " = " << s_entries[i].val << Console::endl;
        }
    }
    Console::info("Config") << "SD card: " << (SDCard::isReady() ? "OK" : "NOT READY") << Console::endl;
}

int serialize(char* buf, int bufSize) {
    int pos = 0;
    bool first = true;
    for (int i = 0; i < MAX_ENTRIES && pos < bufSize - 1; i++) {
        if (s_entries[i].used) {
            if (!first && pos < bufSize - 1) buf[pos++] = ';';
            int n = snprintf(buf + pos, bufSize - pos, "%s=%s",
                             s_entries[i].key, s_entries[i].val);
            if (n < 0 || pos + n >= bufSize) break;
            pos += n;
            first = false;
        }
    }
    buf[pos] = '\0';
    return pos;
}

int count() {
    return s_count;
}

}  // namespace RuntimeConfig
