#pragma once
#include "mission.h"
#include "os/console.h"
#include <stdint.h>

// ============================================================
//  BlockRegistry — Registre statique de blocs C++ accessibles
//  depuis le Jetson via les commandes bridge.
//
//  Chaque bloc enregistré peut être :
//    - listé   : blocks_list  → "collect_A=10,150,8000;..."
//    - exécuté : run_block(collect_A) → SUCCESS | FAILED
//    - marqué  : block_done(collect_A)  (par le Jetson quand il
//                a fini un bloc équivalent côté Python)
//
//  La stratégie embarquée (match()) peut lire les flags done[]
//  pour sauter les blocs déjà réalisés par le Jetson avant
//  un fallback.
// ============================================================

class BlockRegistry {
public:
    static constexpr uint8_t MAX_ENTRIES = 16;

    struct Entry {
        const char*      name        = nullptr;
        uint8_t          priority    = 0;
        uint16_t         score       = 0;
        uint32_t         estimatedMs = 0;
        Block::ActionFn  action      = nullptr;
        Block::FeasibleFn feasible   = nullptr;
        bool             done        = false;   // set by Jetson or after local exec
        bool             used        = false;
    };

    // ── Singleton ────────────────────────────────────────────
    static BlockRegistry& instance();

    // ── Registration (call at boot) ──────────────────────────
    bool add(const char* name, uint8_t priority, uint16_t score,
             uint32_t estimatedMs, Block::ActionFn action,
             Block::FeasibleFn feasible = nullptr);

    // ── Query ────────────────────────────────────────────────
    //  Serialise all entries into buf:
    //    "name1=priority,score,estimatedMs,done;name2=..."
    //  Returns number of chars written.
    int  serialize(char* buf, int bufSize) const;

    //  Number of registered entries
    uint8_t count() const { return m_count; }

    // ── Execution ────────────────────────────────────────────
    //  Run a block by name.  Returns SUCCESS/FAILED.
    //  Sets entry.done = true on SUCCESS.
    //  Returns FAILED if not found.
    BlockResult execute(const char* name);

    // ── Done tracking ────────────────────────────────────────
    bool markDone(const char* name);
    bool isDone(const char* name) const;
    void resetAll();           // clear all done flags

    // ── Build a Mission from the registry ────────────────────
    //  Populates `mission` with all registered blocks, skipping
    //  those already marked done.
    void buildMission(Mission& mission) const;

private:
    BlockRegistry() = default;
    Entry* find(const char* name);
    const Entry* find(const char* name) const;

    Entry   m_entries[MAX_ENTRIES];
    uint8_t m_count = 0;
};
