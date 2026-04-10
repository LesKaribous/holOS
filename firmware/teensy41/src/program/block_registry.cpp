#include "block_registry.h"
#include <string.h>
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Singleton
// ─────────────────────────────────────────────────────────────────────────────

BlockRegistry& BlockRegistry::instance() {
    static BlockRegistry s_instance;
    return s_instance;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Registration
// ─────────────────────────────────────────────────────────────────────────────

bool BlockRegistry::add(const char* name, uint8_t priority, uint16_t score,
                        uint32_t estimatedMs, Block::ActionFn action,
                        Block::FeasibleFn feasible) {
    if (m_count >= MAX_ENTRIES) {
        Console::error("BlockRegistry") << "Full — cannot add " << name << Console::endl;
        return false;
    }
    Entry& e     = m_entries[m_count++];
    e.name        = name;
    e.priority    = priority;
    e.score       = score;
    e.estimatedMs = estimatedMs;
    e.action      = action;
    e.feasible    = feasible;
    e.done        = false;
    e.used        = true;
    Console::info("BlockRegistry") << "Registered: " << name
        << " (p=" << (int)priority << " s=" << (int)score
        << " t=" << (int)estimatedMs << "ms)" << Console::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serialise — "name=priority,score,estimatedMs,done; ..."
// ─────────────────────────────────────────────────────────────────────────────

int BlockRegistry::serialize(char* buf, int bufSize) const {
    int pos = 0;
    bool first = true;
    for (uint8_t i = 0; i < m_count && pos < bufSize - 1; ++i) {
        const Entry& e = m_entries[i];
        if (!e.used) continue;
        if (!first && pos < bufSize - 1) buf[pos++] = ';';
        int n = snprintf(buf + pos, bufSize - pos, "%s=%d,%d,%lu,%d",
                         e.name, (int)e.priority, (int)e.score,
                         (unsigned long)e.estimatedMs, e.done ? 1 : 0);
        if (n < 0 || pos + n >= bufSize) break;
        pos += n;
        first = false;
    }
    buf[pos] = '\0';
    return pos;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Execution
// ─────────────────────────────────────────────────────────────────────────────

BlockResult BlockRegistry::execute(const char* name) {
    Entry* e = find(name);
    if (!e || !e->action) {
        Console::error("BlockRegistry") << "Block not found: " << name << Console::endl;
        return BlockResult::FAILED;
    }
    Console::info("BlockRegistry") << "Executing: " << name << Console::endl;
    BlockResult r = e->action();
    if (r == BlockResult::SUCCESS) {
        e->done = true;
        Console::info("BlockRegistry") << name << " → SUCCESS" << Console::endl;
    } else {
        Console::warn("BlockRegistry") << name << " → FAILED" << Console::endl;
    }
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Done tracking
// ─────────────────────────────────────────────────────────────────────────────

bool BlockRegistry::markDone(const char* name) {
    Entry* e = find(name);
    if (!e) return false;
    e->done = true;
    Console::info("BlockRegistry") << name << " marked done" << Console::endl;
    return true;
}

bool BlockRegistry::isDone(const char* name) const {
    const Entry* e = find(name);
    return e ? e->done : false;
}

void BlockRegistry::resetAll() {
    for (uint8_t i = 0; i < m_count; ++i) {
        m_entries[i].done = false;
    }
    Console::info("BlockRegistry") << "All done flags cleared" << Console::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build a Planner from registry (for fallback)
//  Each block becomes a single-step Mission.
//  Blocks already marked done are skipped.
// ─────────────────────────────────────────────────────────────────────────────

void BlockRegistry::buildPlanner(Planner& planner) const {
    for (uint8_t i = 0; i < m_count; ++i) {
        const Entry& e = m_entries[i];
        if (!e.used || e.done) continue;   // skip done blocks
        Mission& m = planner.addMission(e.name, e.priority, e.score);
        m.addStep(e.name, e.estimatedMs, e.action, e.feasible);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal lookup
// ─────────────────────────────────────────────────────────────────────────────

BlockRegistry::Entry* BlockRegistry::find(const char* name) {
    for (uint8_t i = 0; i < m_count; ++i) {
        if (m_entries[i].used && strcmp(m_entries[i].name, name) == 0)
            return &m_entries[i];
    }
    return nullptr;
}

const BlockRegistry::Entry* BlockRegistry::find(const char* name) const {
    for (uint8_t i = 0; i < m_count; ++i) {
        if (m_entries[i].used && strcmp(m_entries[i].name, name) == 0)
            return &m_entries[i];
    }
    return nullptr;
}
