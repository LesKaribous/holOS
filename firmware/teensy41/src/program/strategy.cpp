#include "strategy.h"
#include "block_registry.h"
#include "config/poi.h"
#include "config/score.h"
#include "config/env.h"
#include "config/runtime_config.h"
#include "routines.h"
#include "mission.h"
#include "auto_tune.h"
#include "services/lidar/occupancy.h"
#include <algorithm>  // std::min / std::max (clamp anti-divergence)

// TODO : déplacer dans Actuators
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ============================================================
//  Exemples d'usage de l'API motion (collide / snap / feedrate)
//  — pseudo-code, pour documentation uniquement.
// ============================================================
//
//   // ─── Section à risque (approche d'un objet, passage étroit) ───
//   motion.collide(true);
//   async motion.go(target1);
//   async motion.go(target2);
//   if (motion.getLastStats().stalled) {
//       // collision détectée → repli / replan
//       handleCollision();
//   }
//   motion.collide(false);
//
//   // ─── Zone libre rapide ─────────────────────────────────────
//   motion.feedrate(0.5f);  // one-shot, override explicite
//   async motion.go(target3);
//
//   // ─── Recalage au mur ──────────────────────────────────────
//   motion.snap(true);
//   async motion.goPolar(heading, distance);
//   motion.snap(false);


// ============================================================
//  Helpers internes
// ============================================================

// Durée estimée des déplacements principaux (ms) — à affiner selon les mesures
namespace Timing {
    constexpr uint32_t COLLECT_STOCK_A = 8000;
    constexpr uint32_t COLLECT_STOCK_B = 8000;
    constexpr uint32_t COLLECT_STOCK_C = 8000;
    constexpr uint32_t STORE_STOCK_A = 8000;
    constexpr uint32_t STORE_STOCK_B = 8000;
    constexpr uint32_t STORE_STOCK_C = 8000;
    constexpr uint32_t THERMO_SET = 15000;
}

// ── Stack diagnostic ─────────────────────────────────────────────────────────
// Reports approximate free stack by reading the SP register.
// Teensy 4.1 DTCM stack starts at _estack (top) and grows down.
// A hard fault typically occurs below ~2 KB free.
static uint32_t freeStack() {
    uint32_t sp;
    asm volatile("mov %0, sp" : "=r"(sp));
    // _estack is at 0x20070000 on Teensy 4.1 (DTCM 512 KB)
    // Return distance from sp to the end of .bss / heap — approximate.
    return sp;  // raw SP; compare across calls to see consumption
}

// ─── Vision refinement (embed cam) ─────────────────────────────────
//
// Once the robot is at the approach point, ask holOS to run blob
// detection on a single ESP32-CAM JPEG (services/embed_cam.py).
// Returns the lateral offset (mm, table frame) to apply to the grab
// pose so the gripper centres on the 4 stock objects. When fewer
// than 4 tags are visible we nudge laterally (using the `bias` hint
// the host returns) and retry; if both directions fail we proceed
// with the original pose and just log a warning.
//
//   tc            : approach direction (also the camera-facing dir)
//   rc            : robot face doing the grab (camera is mounted here)
//   approach      : in/out. Updated to the position we ended up at —
//                   the caller re-uses it as the actual grab origin.
// Returns the cumulative lateral offset (mm) to apply to `grab`.
//
// Side direction convention: image-frame +X (right of the camera) maps
// to the direction `getCompassOrientation(tc) - 90°` in the table
// frame.  Positive offset_mm → tags lean right → robot moves +X.
static float refineWithEmbedCam(Vec2& approach, TableCompass tc, RobotCompass rc) {
    constexpr float    NUDGE_MM     = 50.0f;
    constexpr int      EXPECTED_N   = 4;
    constexpr uint32_t DETECT_TIMEOUT_MS = 2500;
    constexpr uint32_t SETTLE_MS    = 250;

    const float lateral_dir_rad =
        (getCompassOrientation(tc) - 90.0f) * DEG_TO_RAD;
    auto sideVec = [&](float mm) -> Vec2 {
        return PolarVec(lateral_dir_rad, mm).toVec2();
    };

    EmbedDetect r{};
    // Attempt #1 — straight read at the approach pose.
    if (vision.queryEmbedDetect(r, DETECT_TIMEOUT_MS)
            && r.n >= EXPECTED_N && r.valid) {
        Console::info("Strategy")
            << "[vision] full read, offset=" << r.offset_mm << "mm"
            << Console::endl;
        return r.offset_mm;
    }
    Console::info("Strategy")
        << "[vision] partial n=" << r.n << "/" << EXPECTED_N
        << " bias=" << r.bias << Console::endl;

    // Pick the direction to try first.  bias != 0 → use the hint;
    // bias == 0 (or no tag at all) → try +1 (right) first.
    int firstDir = (r.bias != 0) ? r.bias : +1;
    float accum  = 0.0f;

    // Attempt #2 — nudge in the hinted direction.
    accum = firstDir * NUDGE_MM;
    approach += sideVec(firstDir * NUDGE_MM);
    async motion.goAlign(approach, rc, getCompassOrientation(tc));
    waitMs(SETTLE_MS);
    if (vision.queryEmbedDetect(r, DETECT_TIMEOUT_MS)
            && r.n >= EXPECTED_N && r.valid) {
        Console::info("Strategy")
            << "[vision] full after nudge=" << accum
            << "mm, offset=" << r.offset_mm << "mm" << Console::endl;
        return accum + r.offset_mm;
    }

    // Attempt #3 — swing 2× in the opposite direction.
    int secondDir = -firstDir;
    accum += secondDir * (2.0f * NUDGE_MM);
    approach += sideVec(secondDir * (2.0f * NUDGE_MM));
    async motion.goAlign(approach, rc, getCompassOrientation(tc));
    waitMs(SETTLE_MS);
    if (vision.queryEmbedDetect(r, DETECT_TIMEOUT_MS)
            && r.n >= EXPECTED_N && r.valid) {
        Console::info("Strategy")
            << "[vision] full after swing=" << accum
            << "mm, offset=" << r.offset_mm << "mm" << Console::endl;
        return accum + r.offset_mm;
    }

    // Couldn't see all 4 — use whatever partial fix we got, or 0.
    Console::warn("Strategy")
        << "[vision] could not see " << EXPECTED_N
        << " tags — proceeding with partial offset accum=" << accum
        << " r.n=" << r.n << " r.offset=" << r.offset_mm
        << Console::endl;
    return (r.valid && r.n > 0) ? (accum + r.offset_mm) : accum;
}


// Offset commun factored
static void collectStock(Vec2 target, TableCompass tc, RobotCompass rc) {
    constexpr float    APPROACH_OFFSET = 350.0f;
    constexpr float    GRAB_OFFSET     = 180.0f;
    constexpr uint32_t GRAB_DELAY_MS   = 1000;
    const float sideOffset = 0;

    Vec2 approach = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, APPROACH_OFFSET).toVec2();
    Vec2 grab     = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET).toVec2();

    float sidewiseoffset_dir = getCompassOrientation(TableCompass::SOUTH);

    approach += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, sideOffset).toVec2(); // Offset latéral pour compenser la largeur du préhenseur
    grab += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, sideOffset).toVec2(); // Offset latéral pour compenser la largeur du préhenseur

    actuators.grab(rc); //wide open
    async motion.goAlign(approach, rc, getCompassOrientation(tc));

    // ── Vision-based lateral refinement on the embed cam ────────
    // Look at the 4 objects from the approach pose and shift the
    // gripper sideways so all 4 sit centred under it before we
    // close. If the camera is not reachable / nothing detected,
    // refineWithEmbedCam returns ~0 and we fall through to the
    // original pose. `approach` is updated in-place by the helper
    // to wherever the robot physically ended up after its nudges.
    {
        const float lateral_dir_rad =
            (getCompassOrientation(tc) - 90.0f) * DEG_TO_RAD;
        float lateral_mm = refineWithEmbedCam(approach, tc, rc);
        // grab is still expressed relative to the ORIGINAL target —
        // applying the full cumulative lateral offset re-centres the
        // gripper on the (now confirmed) midpoint of the 4 tags.
        grab += PolarVec(lateral_dir_rad, lateral_mm).toVec2();
        Console::info("Strategy")
            << "[vision] applied lateral offset = " << lateral_mm
            << "mm" << Console::endl;
    }

    RuntimeConfig::setInt("motion.timeout_ms", 2000); // 5 secondes
    motion.collide(true);
    async motion.goAlign(grab,     rc, getCompassOrientation(tc));
    motion.collide(false);

    actuators.moveElevator(rc, ElevatorPose::DOWN);
    waitMs(GRAB_DELAY_MS);
    actuators.drop(rc);//gather
    waitMs(GRAB_DELAY_MS);
    actuators.moveElevator(rc, ElevatorPose::STORE);
    waitMs(GRAB_DELAY_MS);
    actuators.grab(rc);//wide open
    waitMs(GRAB_DELAY_MS);
    actuators.moveElevator(rc, ElevatorPose::DOWN);
    waitMs(GRAB_DELAY_MS);
    actuators.store(rc);
    waitMs(GRAB_DELAY_MS);
    actuators.moveElevator(rc, ElevatorPose::STORE);

    motion.collide(false);
    safety.enable();
    motion.setFeedrate(1.0f);
    Console::info("Strategy") << "[collectStock] done      SP=" << String(freeStack()) << Console::endl;
}

// Offset commun factored
static void storeStock(Vec2 target, TableCompass tc, RobotCompass rc) {
    constexpr float    APPROACH_OFFSET = 450.0f;
    constexpr float    GRAB_OFFSET     = 50.0f;
    constexpr uint32_t GRAB_DELAY_MS   = 1000;
    constexpr float  sideOffset   = 0;

    Vec2 approach = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, APPROACH_OFFSET).toVec2();
    Vec2 grab     = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET).toVec2();
    Vec2 grabrecal= target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET-50).toVec2();

    float sidewiseoffset_dir = getCompassOrientation(TableCompass::SOUTH);
    approach += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, sideOffset).toVec2(); // Offset latéral pour compenser la largeur du préhenseur
    grab += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, sideOffset).toVec2(); // Offset latéral pour compenser la largeur du préhenseur
    grabrecal += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, sideOffset).toVec2(); // Offset latéral pour compenser la largeur du préhenseur

    RuntimeConfig::setInt("motion.timeout_ms", 6000); // 5 secondes
    async motion.goAlign(approach, rc, getCompassOrientation(tc));
    localisation.syncToVision(1000);
    async motion.goAlign(approach, rc, getCompassOrientation(tc));

    safety.disable();
    motion.setFeedrate(0.3f);
    RuntimeConfig::setInt("motion.timeout_ms", 5000); // 5 secondes
    motion.collide(true);
    async motion.goAlign(grab,     rc, getCompassOrientation(tc));
    

    waitMs(GRAB_DELAY_MS);
    actuators.drop(rc);//release just enough
    waitMs(GRAB_DELAY_MS);
    actuators.grab(rc);//Grab means wide open

    motion.setFeedrate(0.3f);
    async motion.goAlign(grabrecal,     rc, getCompassOrientation(tc));
    motion.setFeedrate(1.0f);
    motion.collide(false);
    async motion.goAlign(approach,     rc, getCompassOrientation(tc));
    actuators.store(rc); //close gripper avoid collision

    RuntimeConfig::setInt("motion.timeout_ms", 10000); // 5 secondes
    motion.collide(false);
    safety.enable();
    motion.setFeedrate(1.0f);
    Console::info("Strategy") << "[storeStock] done      SP=" << String(freeStack()) << Console::endl;
}

// ============================================================
//  Définition des blocs — une fonction par objectif
//  Retourne SUCCESS ou FAILED selon le résultat
// ============================================================

// Retourne true si la couleur de l'objet justifie une collecte.
// UNKNOWN = bénéfice du doute (on tente quand même).
// NONE    = pas d'objet, inutile d'y aller.
static bool isColorUseful(ObjectColor color) {
    if (color == ObjectColor::NONE)    return false;
    if (color == ObjectColor::UNKNOWN) return true; // lidar/vision non dispo → on tente
    // Adapter selon les règles du match :
    // ex: on ne collecte que les objets d'une certaine couleur
    return true;
}

static bool pantryEmpty = false;

static BlockResult blockCollectA() {
    waitMs(800);
    
    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_01 + Vec2(0,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_01 + Vec2(0,0), TableCompass::WEST, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreA() {
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        storeStock(POI::pantry_07 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        storeStock(POI::pantry_03 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::WEST, RobotCompass::AB);
    }
    
    
    pantryEmpty = true;
    return BlockResult::SUCCESS;
}

static BlockResult blockCollectB() {
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_02+ Vec2(0,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_02+ Vec2(0,0), TableCompass::WEST, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreB() {
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        storeStock(POI::pantry_07 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        storeStock(POI::pantry_03 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::WEST, RobotCompass::AB);
    }

    pantryEmpty = true;
    return BlockResult::SUCCESS;
}


static BlockResult blockCollectC() {
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_04+ Vec2(20,-30), TableCompass::NORTH, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_04+ Vec2(20,-30), TableCompass::NORTH, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreC() {
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        storeStock(POI::pantry_06 + Vec2(-30,-250) + pantryEmpty * Vec2(0, 50), TableCompass::SOUTH, RobotCompass::AB);
    } else {
        storeStock(POI::pantry_04 + Vec2(-30,-250) + pantryEmpty * Vec2(0, 50), TableCompass::SOUTH, RobotCompass::AB);
    }

    pantryEmpty = true;
    return BlockResult::SUCCESS;
}

static BlockResult thermometer_set() {
    RuntimeConfig::setInt("motion.timeout_ms", 5000); // 5 secondes
    motion.collide(false);
    motion.snap(false);
    motion.yield(true);

    waitMs(500); // Attente avant de démarrer (stabilisation éventuelle)
    
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::ELEVATOR, (int) ElevatorPose::UP, 100);

    if(ihm.isColor(Settings::BLUE)) {
        async motion.goAlign(POI::thermometer_hot_blue_approach - Vec2(0,200), RobotCompass::C, getCompassOrientation(TableCompass::SOUTH));
        localisation.syncToVision(1000); // Sync initial localisation to vision (blocking, 2s timeout)

        async motion.go(POI::thermometer_hot_blue_approach);
        probeBorder(TableCompass::SOUTH, RobotCompass::BC, 100, 300);
        probeBorder(TableCompass::EAST, RobotCompass::CA, 100, 300);
        
        async motion.go(POI::thermometer_hot_blue_approach);
        async motion.align(RobotCompass::C, getCompassOrientation(TableCompass::SOUTH));

        RuntimeConfig::setInt("motion.timeout_ms", 2000); // 5 secondes
        async motion.go(POI::thermometer_hot_blue);
    } else {
        async motion.goAlign(POI::thermometer_hot_yellow_approach - Vec2(0,200), RobotCompass::C, getCompassOrientation(TableCompass::WEST));
        localisation.syncToVision(1000); // Sync initial localisation to vision (blocking, 2s timeout)

        async motion.go(POI::thermometer_hot_yellow_approach);
        probeBorder(TableCompass::SOUTH, RobotCompass::CA, 100, 300);
        probeBorder(TableCompass::WEST, RobotCompass::C, 100, 300);

        async motion.go(POI::thermometer_hot_yellow_approach);
        async motion.align(RobotCompass::C, getCompassOrientation(TableCompass::WEST));

        RuntimeConfig::setInt("motion.timeout_ms", 2000); // 5 secondes
        async motion.go(POI::thermometer_hot_yellow);
    }

    //actuators.moveElevator(RobotCompass::CA, ElevatorPose::STORE);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_RIGHT, (int) ManipulatorPose::DROP, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::ELEVATOR, (int) ElevatorPose::UP, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_LEFT, (int) ManipulatorPose::DROP, 100);

    os.wait(1000);

    if(ihm.isColor(Settings::BLUE)) {
        async motion.goAlign(POI::thermometer_hot_blue, RobotCompass::CA, getCompassOrientation(TableCompass::SOUTH));
    } else {
        async motion.goAlign(POI::thermometer_hot_yellow, RobotCompass::CA, getCompassOrientation(TableCompass::SOUTH));
    }

    RuntimeConfig::setInt("motion.timeout_ms", 20000); // 5 secondes
    //motion.collide(true);
    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(POI::thermometer_target_blue);
    } else {
        async motion.go(POI::thermometer_target_yellow);
    }
    motion.collide(false);
    motion.snap(false);
    motion.yield(false);

    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(POI::thermometer_target_blue - Vec2(0,200));
    } else {
        async motion.go(POI::thermometer_target_yellow - Vec2(0,200));
    }

    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_RIGHT, (int) ManipulatorPose::STORE, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::ELEVATOR, (int) ElevatorPose::DOWN, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_LEFT, (int) ManipulatorPose::STORE, 100);

    return BlockResult::SUCCESS;
}

// ============================================================
//  Conditions de faisabilité
//  Appelées par Mission AVANT d'envoyer le robot vers un bloc.
//  Retourne false → bloc skippé immédiatement, Mission tente le suivant.
//
//  ZONE_CHECK_RADIUS : rayon (mm) autour du POI vérifié dans l'occupancy map.
//  ~450mm couvre le POI + la zone d'approche (offset 300mm).
//  Si la carte est vide (lidar secondaire non connecté),
//  isZoneOccupied retourne false → check passe et le robot tente quand même.
// ============================================================

namespace ZoneCheck {
    constexpr float RADIUS = 450.0f; // mm (~3 cellules de 150mm)
}

static bool isZoneAFree() {
    bool occupied = false;
    
          //Sortir zone départ 350 550
    if(ihm.isColor(Settings::BLUE)) {
        occupied = occupancy.isZoneOccupied(POI::stockBlue_01, ZoneCheck::RADIUS);
    } else {
        occupied = occupancy.isZoneOccupied(POI::stockYellow_01, ZoneCheck::RADIUS);
    }
    
    if (occupied)
        Console::warn("Strategy") << "Zone A occupee — bloc skipe" << Console::endl;
    return !occupied;
}

static bool isZoneBFree() {
    bool occupied = false;

    if(ihm.isColor(Settings::BLUE)) {
        occupied = occupancy.isZoneOccupied(POI::stockBlue_02, ZoneCheck::RADIUS);
    } else {
        occupied = occupancy.isZoneOccupied(POI::stockYellow_02, ZoneCheck::RADIUS);
    }
    
    if (occupied)
        Console::warn("Strategy") << "Zone B occupee — bloc skipe" << Console::endl;
    return !occupied;
}


static bool isZoneCFree() {
    bool occupied = false;

    if(ihm.isColor(Settings::BLUE)) {
        occupied = occupancy.isZoneOccupied(POI::stockBlue_04, ZoneCheck::RADIUS);
    } else {
        occupied = occupancy.isZoneOccupied(POI::stockYellow_04, ZoneCheck::RADIUS);
    }
    
    if (occupied)
        Console::warn("Strategy") << "Zone B occupee — bloc skipe" << Console::endl;
    return !occupied;
}

static bool isZoneThermoFree() {
    bool occupied = false;

    if(ihm.isColor(Settings::BLUE)) {
        occupied = occupancy.isZoneOccupied(POI::thermometer_hot_blue, ZoneCheck::RADIUS);
    } else {
        occupied = occupancy.isZoneOccupied(POI::thermometer_hot_yellow, ZoneCheck::RADIUS);
    }

    if (occupied)
        Console::warn("Strategy") << "Zone Thermo occupee — bloc skipe" << Console::endl;
    return !occupied;
}

// ============================================================
//  registerBlocks() — Register individual blocks into BlockRegistry
//  Called once at boot from onRobotBoot().
//  Blocks remain discoverable from Jetson via blocks_list
//  and individually executable via run_block(name).
// ============================================================

FLASHMEM void registerBlocks() {
    BlockRegistry& reg = BlockRegistry::instance();
    reg.add("collect_A", 10, 150, Timing::COLLECT_STOCK_A, blockCollectA, isZoneAFree);
    reg.add("store_A",   10, 150, Timing::STORE_STOCK_A,   blockStoreA);
    reg.add("collect_B",   10, 150, Timing::COLLECT_STOCK_B,   blockCollectB, isZoneBFree);
    reg.add("store_B",   10, 150, Timing::STORE_STOCK_B,   blockStoreB, isZoneBFree);
    reg.add("collect_C",   10, 150, Timing::COLLECT_STOCK_C,   blockCollectC, isZoneCFree);
    //reg.add("store_C",   10, 150, Timing::STORE_STOCK_C,   blockStoreC, isZoneCFree);
    reg.add("thermo_set", 8, 150, Timing::THERMO_SET,   thermometer_set, isZoneThermoFree);
    // reg.add("collect_B", 8, 80, Timing::COLLECT_STOCK_B, blockCollectB, isZoneBFree);
    // reg.add("store_B",   8, 80, Timing::STORE_STOCK_B,   blockStoreB);
}


// ============================================================
//  match() — point d'entrée du match (embedded fallback)
//
//  Utilise le Planner avec des Missions multi-étapes.
//  Chaque Mission = un objectif complet (collect → store).
//  Le Planner boucle tant qu'il reste du temps, retry les
//  missions qui ont échoué, et skip celles dont la zone
//  est occupée.
// ============================================================

FLASHMEM void match() {
    Console::info("Strategy") << "match() entry  SP=" << String(freeStack()) << Console::endl;

    motion.setFeedrate(0.8f);
    motion.enableCruiseMode();
    motion.collide(false);
    motion.snap(false);

    // Ensure PCA9685 pump driver is initialized even if recalage() was
    // skipped (defensive — initPump is idempotent).
    initPump();

    // IMPORTANT: Planner is static to avoid ~1KB on the stack.
    // match() is only called once per boot cycle (programAuto), so
    // re-initialization is safe.
    static Planner planner;
    planner = Planner();

    planner
        .setTimeProvider([]() -> uint32_t {
            long t = chrono.getTimeLeft();
            return (t > 0) ? (uint32_t)t : 0u;
        })
        .setSafetyMargin(5000)   // Ne pas démarrer si < 5s restantes
        .setSafetyAbortMs(8000); // Abandonner si safety bloque > 8s (sauf step non-annulable)

    // ── Définition des missions ──────────────────────────────
    //  Chaque mission = objectif complet.
    //  Les steps sont exécutés en séquence.
    //  Si un step fail → retry la mission plus tard (infini = jamais abandonner).
    //  Si zone occupée → skip, essayer une autre mission.
    //  Dépendances : thermo_set ne démarre qu'après stock_B DONE.
    //
    //  cancelable = false → point de non-retour (robot porte un objet).
    //  Le Planner attend indéfiniment si safety bloque pendant un step
    //  non-annulable, plutôt que d'abandonner la mission.

    { Mission& m = planner.addMission("stock_A", 10, 150);
      m.addStep("collect_A", Timing::COLLECT_STOCK_A, blockCollectA, isZoneAFree);
      m.addStep("store_A",   Timing::STORE_STOCK_A,   blockStoreA, nullptr, false);  // non-annulable : robot porte un objet
      m.setMaxRetries(Mission::INFINITE_RETRIES); }

    Mission& stockB = planner.addMission("stock_B", 8, 150);
    stockB.addStep("collect_B", Timing::COLLECT_STOCK_B, blockCollectB, isZoneBFree);
    stockB.addStep("store_B",   Timing::STORE_STOCK_B,   blockStoreB, nullptr, false);  // non-annulable : robot porte un objet
    stockB.setMaxRetries(Mission::INFINITE_RETRIES);


    Mission& m = planner.addMission("thermo_set", 10, 150);
    m.addStep("thermo_set", Timing::THERMO_SET, thermometer_set, isZoneThermoFree);
    m.addDependency(stockB);
    m.setMaxRetries(Mission::INFINITE_RETRIES);

    Mission& stockC = planner.addMission("stock_C", 5, 150);
    stockC.addStep("collect_C", Timing::COLLECT_STOCK_C, blockCollectC, isZoneCFree);
    //stockC.addStep("store_C",   Timing::STORE_STOCK_C,   blockStoreC, nullptr, false);  // non-annulable : robot porte un objet
    stockC.addDependency(m);
    stockC.setMaxRetries(1);

    //Sortir zone départ 350 550
    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(Vec2(3000-400,550));
    } else {
        async motion.go(Vec2(400,550));
    }

    planner.run();
    RuntimeConfig::setInt("motion.timeout_ms", 20000); // 5 secondes

    waitMs(500); // Attente avant de démarrer (stabilisation éventuelle)
    localisation.syncToVision(1000); // Sync initial localisation to vision (blocking, 2s timeout)

    bool success = false;
    for(int i = 0; i < 5; i++){
        if(ihm.isColor(Settings::BLUE)) {
            async motion.go(POI::wait_blue);
        } else {
            async motion.go(POI::wait_yellow);
        }
        success = motion.wasSuccessful();
        if(success) break;
    }

    if(chrono.getTimeLeft() > 5000) {
        waitMs(chrono.getTimeLeft() - 4500);
    }

    chrono.onMatchNearlyFinished();
    chrono.onMatchFinished();
}

FLASHMEM void waitMs(unsigned long time){
    os.wait(time);
    //delay(time);
}

FLASHMEM void nearEnd(){
    //if(motion.isPending())motion.forceCancel();
    motion.setFeedrate(1.0);
    //nav.setAbsolute();
    safety.enable();

    //point de passage
    motion.collide(false);
    motion.snap(false);
    motion.yield(true);

    localisation.syncToVision(1000); // Sync initial localisation to vision (blocking, 2s timeout)

    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_RIGHT, (int) ManipulatorPose::STORE, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::ELEVATOR, (int) ElevatorPose::DOWN, 100);
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_LEFT, (int) ManipulatorPose::STORE, 100);
    actuators.store(RobotCompass::AB);//take object in case we are still holding it
    actuators.moveElevator(RobotCompass::CA, ElevatorPose::STORE);
    actuators.moveElevator(RobotCompass::AB, ElevatorPose::STORE);
    waitMs(800);

    if(ihm.isColor(Settings::BLUE)) {
        async motion.goAlign(Vec2(3000-350,700), RobotCompass::BC, getCompassOrientation(TableCompass::NORTH));
    } else {
        async motion.goAlign(Vec2(350,700), RobotCompass::C, getCompassOrientation(TableCompass::NORTH));
    }

    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(POI::startBlue + Vec2(0,-100));
    }
    else {
        async motion.go(POI::startYellow + Vec2(0,-100));
    }
    motion.collide(false);
    actuators.drop(RobotCompass::AB);//take object in case we are still holding it
    // // Time to wait befor SIMAs leave the Backstage
    // unsigned long left = chrono.getTimeLeft();
    // unsigned long waitSima = (left > 5000) ? (left - 5000) : 0; 
    // // Wait for SIMAs
    // waitMs(waitSima);

    // // Got to the Backstage
    // if(ihm.isColor(Settings::BLUE)) async motion.go(POI::b1);
    // else async motion.go(POI::y1);

    // ihm.addScorePoints(Score::RobotInArrivalZonePoints);
    // waitMs(200);
    // //ihm.onUpdate();
    chrono.onMatchFinished();
}

// takeAllStock / takeStock ont été mergées en collectStock() — voir ci-dessus


RobotCompass nextActuator(RobotCompass rc){
    int RobotCompassSize = 6;
    return static_cast<RobotCompass>((static_cast<int>(rc) + 2) % RobotCompassSize);
}

RobotCompass previousActuator(RobotCompass rc){
    int RobotCompassSize = 6;
    return static_cast<RobotCompass>((static_cast<int>(rc) + RobotCompassSize - 2) % RobotCompassSize);
}



FLASHMEM void initPump(){
    pwm.begin();
    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting!
     * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     * is used for calculating things like writeMicroseconds()
     * Analog servos run at ~50 Hz updates, It is importaint to use an
     * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     * Setting the value here is specific to each individual I2C PCA9685 chip and
     * affects the calculations for the PWM update frequency. 
     * Failure to correctly set the int.osc value will cause unexpected PWM results
     */
    pwm.setOscillatorFrequency(50000000);
    pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  
    // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
    // some i2c devices dont like this so much so if you're sharing the bus, watch
    // out for this!
    //Wire.setClock(400000);  
}

// Tracks whether initPump() has already been called.
// startPump() and stopPump() call initPump() automatically the first time so
// that the test "act_ev_toggle" works even when "act_pump_init" was not run
// beforehand.  initPump() is idempotent (calling it again is safe).
static bool s_pumpInitialized = false;

FLASHMEM void setOutput(uint8_t pin, bool state) {
    if (state) {
      pwm.setPWM(pin, 4096, 0);  // ON
    } else {
      pwm.setPWM(pin, 0, 4096);  // OFF
    }
}

FLASHMEM void startPump(RobotCompass rc, bool side){
    if (!s_pumpInitialized) {
        initPump();
        s_pumpInitialized = true;
    }
    uint8_t evPin ;
    uint8_t pumpPin ;
    if(side) evPin = Pin::PCA9685::EV_CA_RIGHT ;
    else evPin = Pin::PCA9685::EV_CA_LEFT;
    if(side) pumpPin = Pin::PCA9685::PUMP_CA_RIGHT;
    else pumpPin = Pin::PCA9685::PUMP_CA_LEFT;
    setOutput(evPin, false);  // Fermer l'électrovanne

    // Soft-start ramp over ~200ms to avoid current spike that resets serial.
    // PCA9685 setPWM(channel, on_tick, off_tick): off_tick sets duty 0–4095.
    //
    // IMPORTANT: use delay() NOT waitMs().  waitMs() calls os.wait() which
    // enters a nested run() loop.  When pump/EV commands are executed via
    // the interpreter (os.execute → script → CommandHandler), a nested run()
    // re-enters the script job causing interpreter state corruption → freeze.
    // delay() hard-blocks but 200 ms is well within the 5 000 ms heartbeat
    // timeout, so no disconnect risk.
    constexpr uint16_t rampSteps[] = { 800, 1600, 2400, 3200, 4095 };
    for (uint16_t duty : rampSteps) {
        pwm.setPWM(pumpPin, 0, duty);
        delay(40);
    }
    setOutput(pumpPin, true); // Full power
}

FLASHMEM void stopPump(RobotCompass rc, uint16_t evPulseDuration, bool side){
    uint8_t evPin ;
    uint8_t pumpPin ;
    if(side) evPin = Pin::PCA9685::EV_CA_RIGHT ;
    else evPin = Pin::PCA9685::EV_CA_LEFT;
    if(side) pumpPin = Pin::PCA9685::PUMP_CA_RIGHT;
    else pumpPin = Pin::PCA9685::PUMP_CA_LEFT;
    setOutput(pumpPin, false); // Stopper la pompe

    // Wait for pump back-EMF to settle before energizing the EV solenoid.
    // Sudden pump deenergization creates a back-EMF spike from the motor
    // inductance; simultaneously opening the EV adds an inrush current spike.
    // The combined voltage dip can trigger a Teensy brownout reset (= USB-CDC
    // "serial closed").  100 ms is enough for the transient to dissipate while
    // keeping total stopPump() time well within the 3 000 ms command timeout.
    // IMPORTANT: use delay() NOT waitMs() — same reentrancy issue as startPump.
    delay(100);

    setOutput(evPin, true);    // Ouvrir l'EV
    delay(evPulseDuration);    // Maintenir l'EV ouverte
    setOutput(evPin, false);   // Fermer l'EV
}
