#pragma once
#include <stdint.h>

// ============================================================
//  Planner / Mission / Step — Planificateur de missions
//  multi-étapes avec retry, cooldown, dépendances et boucle continue.
//
//  Concepts :
//    Block       — action atomique (inchangé, pour BlockRegistry)
//    Step        — une étape dans une Mission (action + feasibility)
//    Mission     — séquence ordonnée de Steps formant un objectif
//    Planner     — sélectionne et exécute des Missions en boucle
//
//  Nouveautés :
//    - Dépendances : addDependency(&otherMission) — bloque tant que
//      la mission requise n'est pas DONE
//    - Retries infinis : setMaxRetries(INFINITE_RETRIES) — jamais abandonné
//    - Sortie anticipée : si plus aucune mission n'est viable, le Planner
//      quitte même s'il reste du temps
//
//  Usage :
//      static Planner planner;
//      planner = Planner();
//      planner
//          .setTimeProvider([]() -> uint32_t { return chrono.getTimeLeft(); })
//          .setSafetyMargin(5000);
//
//      Mission& mA = planner.addMission("stock_A", 10, 150);
//      mA.addStep("collect_A", 8000, blockCollectA, isZoneAFree);
//      mA.addStep("store_A",   8000, blockStoreA);
//      mA.setMaxRetries(Mission::INFINITE_RETRIES);
//
//      Mission& mB = planner.addMission("stock_B", 8, 150);
//      mB.addStep("collect_B", 6000, blockCollectB, isZoneBFree);
//      mB.addStep("store_B",   6000, blockStoreB);
//
//      Mission& mT = planner.addMission("thermo", 10, 150);
//      mT.addStep("thermo", 15000, blockThermo, isZoneThermoFree);
//      mT.addDependency(mB);   // thermo requires stock_B DONE
//
//      planner.run();
// ============================================================


// ------------------------------------------------------------
//  BlockResult — résultat d'une action atomique
// ------------------------------------------------------------

enum class BlockResult : uint8_t {
    SUCCESS = 0,
    FAILED  = 1,
};


// ------------------------------------------------------------
//  BlockStats — statistiques d'un bloc (pour BlockRegistry)
// ------------------------------------------------------------

struct BlockStats {
    BlockResult result      = BlockResult::FAILED;
    uint32_t    durationMs  = 0;
    bool        attempted   = false;
    bool        skippedTime = false;
};


// ------------------------------------------------------------
//  Block — Unité d'action atomique (gardé pour BlockRegistry)
// ------------------------------------------------------------

struct Block {
    using ActionFn   = BlockResult (*)();
    using FeasibleFn = bool        (*)();

    const char* name        = "unnamed";
    uint8_t     priority    = 0;
    uint16_t    score       = 0;
    uint32_t    estimatedMs = 0;
    ActionFn    action      = nullptr;
    FeasibleFn  feasible    = nullptr;

    BlockStats  stats;
    bool        done        = false;
};


// ============================================================
//  Step — une étape dans une Mission
// ============================================================

struct Step {
    const char*      name        = "unnamed";
    uint32_t         estimatedMs = 0;
    Block::ActionFn  action      = nullptr;
    Block::FeasibleFn feasible   = nullptr;   // check faisabilité (nullptr = toujours OK)
};


// ============================================================
//  MissionState
// ============================================================

enum class MissionState : uint8_t {
    PENDING   = 0,   // Jamais tentée
    DONE      = 1,   // Tous les steps OK
    FAILED    = 2,   // Un step a échoué ou infaisable — retryable
    ABANDONED = 3,   // Max retries dépassé
};


// ============================================================
//  Mission — séquence ordonnée de Steps
// ============================================================

class Mission {
public:
    static constexpr uint8_t  MAX_STEPS          = 8;
    static constexpr uint8_t  MAX_DEPENDENCIES   = 4;
    static constexpr uint8_t  DEFAULT_MAX_RETRIES = 3;
    static constexpr uint8_t  INFINITE_RETRIES    = 255;  // sentinelle "jamais abandonner"
    static constexpr uint32_t RETRY_COOLDOWN_MS   = 3000;

    Mission() = default;

    // ---- Builder API ----

    /// Ajoute un step à la mission.
    Mission& addStep(const char* name, uint32_t estimatedMs,
                     Block::ActionFn action,
                     Block::FeasibleFn feasible = nullptr);

    /// Check de faisabilité au niveau mission (avant de commencer).
    Mission& setFeasible(Block::FeasibleFn fn);

    /// Max retries avant abandon (défaut = 3).
    /// Utiliser INFINITE_RETRIES (255) pour ne jamais abandonner.
    Mission& setMaxRetries(uint8_t n);

    /// Ajoute une dépendance : cette mission ne peut pas démarrer
    /// tant que `dep` n'est pas DONE.
    Mission& addDependency(Mission& dep);

    // ---- Getters ----

    const char*  name()     const { return m_name; }
    uint8_t      priority() const { return m_priority; }
    uint16_t     score()    const { return m_score; }
    MissionState state()    const { return m_state; }
    uint8_t      retries()  const { return m_retries; }

    /// Durée estimée totale (somme des steps restants).
    uint32_t     remainingEstimatedMs() const;

    /// Peut-on retenter cette mission ?
    bool canRetry() const;

    /// Le cooldown après échec est-il écoulé ?
    bool isCooledDown() const;

    /// Toutes les dépendances sont-elles satisfaites ?
    bool depsSatisfied() const;

private:
    friend class Planner;

    const char*       m_name        = "unnamed";
    uint8_t           m_priority    = 0;
    uint16_t          m_score       = 0;
    Block::FeasibleFn m_feasible    = nullptr;

    Step    m_steps[MAX_STEPS];
    uint8_t m_stepCount    = 0;
    uint8_t m_currentStep  = 0;      // Prochain step à exécuter
    uint8_t m_retries      = 0;
    uint8_t m_maxRetries   = DEFAULT_MAX_RETRIES;

    MissionState m_state         = MissionState::PENDING;
    uint32_t     m_lastAttemptMs = 0;

    // Dépendances : pointeurs vers les missions qui doivent être DONE
    const Mission* m_deps[MAX_DEPENDENCIES] = {};
    uint8_t        m_depCount = 0;
};


// ============================================================
//  Planner — sélectionne et exécute des Missions en boucle
//
//  Boucle continue tant qu'il reste du temps :
//   1. Sélectionne la meilleure mission faisable
//   2. Exécute ses steps séquentiellement
//   3. Si FAIL → retry plus tard, essaie une autre
//   4. Si toutes skippées → attend et re-check
//   5. Si toutes les missions sont DONE/ABANDONED → sortie
// ============================================================

class Planner {
public:
    static constexpr uint8_t  MAX_MISSIONS          = 16;
    static constexpr uint32_t IDLE_WAIT_MS           = 500;
    static constexpr uint32_t DEFAULT_SAFETY_MARGIN  = 3000;

    Planner() = default;

    // ---- Builder API ----

    /// Ajoute une mission et retourne une référence pour y ajouter des steps.
    Mission& addMission(const char* name, uint8_t priority, uint16_t score);

    /// Cherche une mission par nom (nullptr si non trouvée).
    Mission* getMission(const char* name);

    /// Fournisseur de temps restant (ms). Sans provider → pas de contrainte temps.
    Planner& setTimeProvider(uint32_t (*fn)());

    /// Marge de sécurité avant fin de match (défaut 3000ms).
    Planner& setSafetyMargin(uint32_t ms);

    // ---- Exécution ----

    /// Lance le planner. Bloquant — retourne quand tout est terminé ou plus de temps.
    void run();

    // ---- Diagnostics post-run ----

    uint16_t totalScore()     const;
    uint16_t potentialScore() const;
    uint8_t  totalMissions()  const;
    uint8_t  doneCount()      const;
    uint8_t  failedCount()    const;
    uint8_t  abandonedCount() const;
    uint8_t  pendingCount()   const;
    uint32_t elapsedMs()      const;

private:
    Mission* selectNext();
    bool     canFitMission(const Mission& m) const;
    uint32_t remainingMs()                   const;
    void     executeMission(Mission& m);
    bool     allTerminal()                   const;
    void     logSummary()                    const;

    Mission    m_missions[MAX_MISSIONS];
    uint8_t    m_count      = 0;
    uint16_t   m_score      = 0;
    uint32_t   m_startMs    = 0;
    uint32_t   m_safetyMs   = DEFAULT_SAFETY_MARGIN;
    uint32_t (*m_timeProvider)() = nullptr;
};
