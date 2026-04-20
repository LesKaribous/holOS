#include "strategy.h"
#include "block_registry.h"
#include "config/poi.h"
#include "config/score.h"
#include "config/env.h"
#include "config/runtime_config.h"
#include "routines.h"
#include "mission.h"
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
    constexpr uint32_t COLLECT_STOCK_B = 6000;
    constexpr uint32_t STORE_STOCK_A = 8000;
    constexpr uint32_t STORE_STOCK_B = 6000;
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
    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_01 + Vec2(0,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_01 + Vec2(0,0), TableCompass::WEST, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreA() {
    if(ihm.isColor(Settings::BLUE)) {
        storeStock(POI::pantry_07 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        storeStock(POI::pantry_03 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::WEST, RobotCompass::AB);
    }
    
    
    pantryEmpty = true;
    return BlockResult::SUCCESS;
}

static BlockResult blockCollectB() {

    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_02+ Vec2(0,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_02+ Vec2(0,0), TableCompass::WEST, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreB() {
    if(ihm.isColor(Settings::BLUE)) {
        storeStock(POI::pantry_07 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::EAST, RobotCompass::AB);
    } else {
        storeStock(POI::pantry_03 + Vec2(0,0) + pantryEmpty * Vec2(30,0), TableCompass::WEST, RobotCompass::AB);
    }

    pantryEmpty = true;
    return BlockResult::SUCCESS;
}


static BlockResult blockCollectC() {
    probeBorder(TableCompass::SOUTH, RobotCompass::BC, 100, 300);
    if(ihm.isColor(Settings::BLUE)) {
        collectStock(POI::stockBlue_04+ Vec2(20,-30), TableCompass::NORTH, RobotCompass::AB);
    } else {
        collectStock(POI::stockYellow_04+ Vec2(20,-30), TableCompass::NORTH, RobotCompass::AB);
    }

    return BlockResult::SUCCESS;
}

static BlockResult blockStoreC() {
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
    if(ihm.isColor(Settings::BLUE)) {
        async motion.goAlign(POI::thermometer_hot_blue_approach, RobotCompass::C, getCompassOrientation(TableCompass::SOUTH));
        RuntimeConfig::setInt("motion.timeout_ms", 2000); // 5 secondes
        async motion.go(POI::thermometer_hot_blue);
    } else {
        async motion.goAlign(POI::thermometer_hot_yellow_approach, RobotCompass::C, getCompassOrientation(TableCompass::WEST));
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
    reg.add("collect_C",   10, 150, Timing::COLLECT_STOCK_B,   blockCollectC, isZoneCFree);
    reg.add("store_C",   10, 150, Timing::STORE_STOCK_B,   blockStoreC, isZoneCFree);
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

    motion.setFeedrate(0.6f);
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
    stockC.addStep("collect_B", Timing::COLLECT_STOCK_B, blockCollectC, isZoneCFree);
    stockC.addStep("store_B",   Timing::STORE_STOCK_B,   blockStoreC, nullptr, false);  // non-annulable : robot porte un objet
    stockC.addDependency(m);
    stockC.setMaxRetries(1);


    //Sortir zone départ 350 550
    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(Vec2(3000-350,550));
    } else {
        async motion.go(Vec2(350,550));
    }

    planner.run();
    RuntimeConfig::setInt("motion.timeout_ms", 20000); // 5 secondes

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

FLASHMEM void recalage(){
    motion.engage();
    //motion.disableCruiseMode();
    motion.setFeedrate(0.3);
    waitMs(600);

    if(ihm.isColor(Settings::BLUE)){
        
        motion.setAbsPosition(Vec3( Vec2(3000-140,100), 150 * DEG_TO_RAD));
        motion.goAlign(Vec2(3000-350, 300), RobotCompass::AB, getCompassOrientation(TableCompass::WEST));
        actuators.moveElevator(RobotCompass::CA, ElevatorPose::DOWN);
        /*
        motion.setFeedrate(0.2);
        probeBorder(TableCompass::SOUTH, RobotCompass::BC,100);
        probeBorder(TableCompass::EAST,  RobotCompass::CA,100);//when starting this line
        motion.setFeedrate(1.0);
        */
        //calibrate();

        //async motion.go(POI::testB);
        //async motion.go(POI::b2);
        
        //async motion.align(RobotCompass::BC, getCompassOrientation(TableCompass::SOUTH));
        //motion.setAbsPosition(Vec3(POI::b2, motion.getOrientation()));

    }else{
        motion.setAbsPosition(Vec3(140, 125 ,-90 * DEG_TO_RAD));
        motion.goAlign(Vec2(350, 300), RobotCompass::AB, getCompassOrientation(TableCompass::EAST));
        /*
        motion.setFeedrate(0.2);
        probeBorder(TableCompass::SOUTH, RobotCompass::BC,100);
        probeBorder(TableCompass::WEST,  RobotCompass::AB,100);
        motion.setFeedrate(1.0);
        */
        //calibrate();

        //async motion.go(POI::y2);
        //async motion.go(POI::y2);

        //async motion.align(RobotCompass::BC, getCompassOrientation(TableCompass::SOUTH));
        //motion.setAbsPosition(Vec3(POI::y2, motion.getOrientation()));
    }
    //motion.disengage();
    motion.setFeedrate(1.0);
    
    initPump(); //TODO : Integrate into Actuators 
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


    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(Vec2(3000-350,700));
    } else {
        async motion.go(Vec2(350,700));
    }


    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(POI::startBlue + Vec2(0,-100));
    }
    else {
        async motion.go(POI::startYellow + Vec2(0,-100));
    }
    motion.collide(false);
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


FLASHMEM void calibrate(){
    motion.disableCruiseMode();
    //motion.collide(false);

    float start = localisation.getPosition().x;
    float distance = 0;
    float distanceGoal = 400;
    float scale = 0;
    float current = 0;

    for(int i = 0; i < 3; i++){
        start = localisation.getPosition().x;
	    async motion.goPolar(0,distanceGoal);
        current = Vec2(localisation.getPosition()).mag();
        distance = fabs(current - start);
        Console::info() << "distance : " << current - start << "|" << 400 << Console::endl;
        scale = distanceGoal/distance;
        localisation.setLinearScale(scale);

        start = localisation.getPosition().x;
	    async motion.goPolar(0,-distanceGoal);
        current = Vec2(localisation.getPosition()).mag();
        distance = fabs(current - start);
        Console::info() << "distance : " << current - start << "|" << 400 << Console::endl;
        scale = distanceGoal/distance;
        localisation.setLinearScale(scale);

        
    }

    //motion.collide(false);
    motion.enableCruiseMode();
}


FLASHMEM void probeBorder(TableCompass tc, RobotCompass rc, float clearance, float approachDist, float probeDist, float feedrate){

    boolean wasAbsolute = motion.isAbsolute();
    float currentFeedrate = motion.getFeedrate();
    actuators.moveElevator(rc, ElevatorPose::UP);

    motion.setFeedrate(feedrate);
    async motion.align(rc, getCompassOrientation(tc));
    motion.setRelative();

    // Cruise mode requis pour que snap() fonctionne (PID + OTOS + stall detector).
    // snap(true) : si on stalle près d'une bordure, la coord de l'axe stallé est
    // automatiquement snappée au mur connu (margin côté Xmin/Ymin, TABLE-margin
    // côté Xmax/Ymax) et le move termine proprement.
    motion.enableCruiseMode();
    motion.snap(true);
    async motion.goPolar(getCompassOrientation(rc), approachDist);
    async motion.goPolar(getCompassOrientation(rc), probeDist);
    motion.snap(false);

    // Correction explicite de l'orientation : snap ne touche pas à position.c.
    // On recale aussi la coord selon tc pour rester aligné avec les conventions
    // des appelants (au cas où snap aurait raté, typiquement si on probe un
    // obstacle loin des bordures).
    float _offset  = getOffsets(rc);
    Vec3  position = motion.estimatedPosition();
    Console::println(_offset);
    Console::println(position);

    if      (tc == TableCompass::NORTH) position.y = 0.0    + _offset;
    else if (tc == TableCompass::SOUTH) position.y = 2000.0 - _offset;
    else if (tc == TableCompass::EAST)  position.x = 3000.0 - _offset;
    else if (tc == TableCompass::WEST)  position.x = 0.0    + _offset;
    position.c = DEG_TO_RAD * (getCompassOrientation(tc) - getCompassOrientation(rc));

    motion.setAbsPosition(position);
    delay(200);

    if (clearance != 0) {
        async motion.goPolar(getCompassOrientation(rc), -clearance);
    }

    if (wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(currentFeedrate);
    actuators.moveElevator(rc, ElevatorPose::DOWN);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  calibrateStall — Auto-tune de la détection de collision par stagnation.
//
//  Procédure (en boucle, jusqu'à 2 phases OK ou maxIter atteint) :
//
//    ① Recalage WEST   : probeBorder pour se placer à x≈clearance du mur.
//    ② Phase A (VP)    : goPolar(face, +PROBE_BUMP) avec stall actif.
//                          → doit stall. Sinon : params trop rigides → assouplir.
//    ③ Phase B (FP)    : goPolar(face, -PROBE_REVERSE) dans zone libre.
//                          → NE doit PAS stall. Sinon : trop sensible → durcir.
//
//  Détection stall vs timeout : on lit motion.getLastStats().stalled après
//  chaque async — clean, pas besoin de proxy sur millis().
//
//  Pendant la calibration, velocity-mismatch est désactivé (velCmdMin → 1e6)
//  pour isoler la contribution de la stagnation. Restauré en fin.
//
//  Les probeBorder intermédiaires utilisent toujours les params ORIGINELS
//  (lambda safeProbeBorder) pour ne pas faire foirer le recalage lui-même
//  avec un jeu de params extrême.
// ═══════════════════════════════════════════════════════════════════════════════
FLASHMEM StallCalibResult calibrateStall(RobotCompass face, int maxIter) {
    // ─── Paramètres de la procédure ─────────────────────────────────────────
    // Flux par itération :
    //   (initial) probeBorder aveugle → retraite à CLEARANCE du mur
    //   Phase A   : goPolar(+PROBE_BUMP)  vers mur → doit stall  (> CLEARANCE)
    //   back-off  : goPolar(-CLEARANCE)   éloignement relatif (pas de probeBorder)
    //   Phase B   : goPolar(+PROBE_CLEAR) vers mur → NE doit PAS stall  (< CLEARANCE)
    //   (si échec) safeProbeBorder avant la prochaine itération
    const float    CLEARANCE   = 500.0f;  // mm — distance au mur entre les phases
    const float    PROBE_BUMP  = 600.0f;  // mm — Phase A (> CLEARANCE → hit wall)
    const float    PROBE_CLEAR = 400.0f;  // mm — Phase B (< CLEARANCE → reste 100mm du mur)
    // NB : cruise mode → feedrate × Settings::Motion::MAX_SPEED (1800 mm/s).
    // 0.1 ⇒ ~180 mm/s, safe pour un bump-test. NE PAS monter au-dessus de 0.2
    // sans retester : le robot slammerait le mur avant que le stall detector
    // n'ait le temps de firer (stag_time typique 400 ms).
    const float    FEEDRATE    = 0.1f;    // ~180 mm/s — vitesse de calibration safe

    // ─── Snapshot état motion / RuntimeConfig pour restauration fin ─────────
    const boolean wasAbsolute    = motion.isAbsolute();
    const float   savedFeedrate  = motion.getFeedrate();
    const float   saved_stagMove = RuntimeConfig::getFloat("stall.stag_move_mm",
                                        Settings::Motion::Stall::STAG_MOVE_MM);
    const float   saved_stagTime = RuntimeConfig::getFloat("stall.stag_time",
                                        Settings::Motion::Stall::STAG_TIME_S);
    const float   saved_velCmdMin= RuntimeConfig::getFloat("stall.vel_cmd_min",
                                        Settings::Motion::Stall::VEL_CMD_MIN_MMS);

    // Valeurs de travail (partent des valeurs courantes, pas des defaults)
    float stagMove = saved_stagMove;
    float stagTime = saved_stagTime;

    StallCalibResult result;
    result.stagMoveMm = stagMove;
    result.stagTimeS  = stagTime;

    // ─── Lambdas internes ───────────────────────────────────────────────────
    // Applique les params courants dans RuntimeConfig (lu au prochain start()).
    auto applyTestParams = [&]() {
        RuntimeConfig::setFloat("stall.stag_move_mm", stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    stagTime);
    };
    // Restaure les params originels pour un probeBorder intermédiaire sans
    // risque de false-positive corrompant le recalage.
    auto applySafeParams = [&]() {
        RuntimeConfig::setFloat("stall.stag_move_mm", saved_stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    saved_stagTime);
        RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);
    };
    // Rend velocity-mismatch aveugle (isole stagnation).
    auto blindVelocity = [&]() {
        RuntimeConfig::setFloat("stall.vel_cmd_min", 1.0e6f);
    };
    // Recalage WEST avec params safe, puis réapplique blinding + params de test.
    auto safeProbeBorder = [&](float clearance) {
        applySafeParams();
        probeBorder(TableCompass::WEST, face, clearance);
        blindVelocity();
        applyTestParams();
    };

    // ─── Setup initial ──────────────────────────────────────────────────────
    motion.setFeedrate(FEEDRATE);
    blindVelocity();

    Console::info("CalibStall")
        << "Initial: touch wall + back off " << (int)CLEARANCE << "mm"
        << Console::endl;
    safeProbeBorder(CLEARANCE);

    // Formate la cause per-axe du stall pour les logs (ex: "stagX+velY").
    // Vide = aucune cause détectée (triggered sans cause = displacement legacy).
    auto fmtCause = [](const Motion::MoveStats& s) -> const char* {
        // Ordre de priorité : stagnation avant velocity-mismatch.
        if (s.stallCauseStagX) return "stagX";
        if (s.stallCauseStagY) return "stagY";
        if (s.stallCauseVelX)  return "velX";
        if (s.stallCauseVelY)  return "velY";
        if (s.stallCauseVelRot)return "velR";
        return s.stalled ? "disp" : "-";
    };

    int iter = 0;
    for (iter = 0; iter < maxIter; iter++) {
        Console::info("CalibStall")
            << "───── Iter " << iter
            << "  stagMove=" << stagMove << "mm"
            << "  stagTime=" << (int)(stagTime * 1000) << "ms ─────"
            << Console::endl;

        applyTestParams();

        // ══════ PHASE A : test VRAI-POSITIF (doit stall contre le mur) ══════
        // IMPORTANT : cruise mode obligatoire — le StallDetector vit dans
        // cruise_controller.stall() (PositionController). En stepper mode
        // le détecteur n'est pas actif et collectStats() zérote m_lastStats.
        // Convention probeBorder : +D dans direction face = vers le mur.
        motion.setRelative();
        motion.collide(true);    // sticky : stall detect + cancel actifs
        async motion.goPolar(getCompassOrientation(face), +PROBE_BUMP);

        Motion::MoveStats statsA = motion.getLastStats();
        result.phaseAOk = statsA.stalled;
        Console::info("CalibStall")
            << "  Phase A (bump): " << (statsA.stalled ? "OK   " : "FAIL ")
            << "cause=" << fmtCause(statsA)
            << "  dur=" << (int)statsA.durationMs << "ms"
            << "  dist=" << (int)statsA.traveledMm << "/" << (int)PROBE_BUMP << "mm"
            << Console::endl;

        if (!statsA.stalled) {
            // Pas de stall → params trop rigides → on assouplit.
            const float oldMove = stagMove;
            const float oldTime = stagTime;
            stagMove *= 1.4f;
            stagTime *= 0.8f;
            stagMove = std::min(stagMove, 4.0f);
            stagTime = std::max(stagTime, 0.10f);
            Console::warn("CalibStall")
                << "  → loosening: stagMove " << oldMove << "→" << stagMove
                << "  stagTime " << (int)(oldTime*1000) << "→" << (int)(stagTime*1000) << "ms"
                << Console::endl;
            safeProbeBorder(CLEARANCE);
            continue;
        }

        // Phase A OK → back-off RELATIF de CLEARANCE (pas de probeBorder !).
        // On vient de stall contre le mur : pousser encore dedans n'a aucun sens.
        // On s'éloigne simplement du mur pour avoir ~CLEARANCE de jeu avant Phase B.
        Console::info("CalibStall") << "  Back-off " << (int)CLEARANCE << "mm" << Console::endl;
        motion.setRelative();
        motion.collide(false);           // désactive stall en s'éloignant du mur
        async motion.goPolar(getCompassOrientation(face), -CLEARANCE);

        // ══════ PHASE B : test FAUX-POSITIF (ne doit PAS stall) ══════════════
        // Cruise mode. On se déplace VERS le mur sur PROBE_CLEAR < CLEARANCE,
        // on reste donc en zone libre (~100 mm du mur). Tout stall ici = faux+.
        applyTestParams();
        motion.setRelative();
        motion.collide(true);
        async motion.goPolar(getCompassOrientation(face), +PROBE_CLEAR);
        motion.collide(false);

        Motion::MoveStats statsB = motion.getLastStats();
        result.phaseBOk = !statsB.stalled;
        Console::info("CalibStall")
            << "  Phase B (free): " << (!statsB.stalled ? "OK   " : "FAIL ")
            << "cause=" << fmtCause(statsB)
            << "  dur=" << (int)statsB.durationMs << "ms"
            << "  dist=" << (int)statsB.traveledMm << "/" << (int)PROBE_CLEAR << "mm"
            << Console::endl;

        if (statsB.stalled) {
            // Faux-positif → params trop sensibles → on durcit.
            const float oldMove = stagMove;
            const float oldTime = stagTime;
            stagMove *= 0.7f;
            stagTime *= 1.2f;
            stagMove = std::max(stagMove, 0.1f);
            stagTime = std::min(stagTime, 1.0f);
            Console::warn("CalibStall")
                << "  → tightening: stagMove " << oldMove << "→" << stagMove
                << "  stagTime " << (int)(oldTime*1000) << "→" << (int)(stagTime*1000) << "ms"
                << Console::endl;
            safeProbeBorder(CLEARANCE);
            continue;
        }

        // Les deux phases passent → convergence atteinte.
        result.converged = true;
        break;
    }

    result.iter       = iter + (result.converged ? 1 : 0);
    result.stagMoveMm = stagMove;
    result.stagTimeS  = stagTime;

    if (!result.converged) {
        Console::error("CalibStall")
            << "Did NOT converge in " << maxIter << " iterations — rolling back"
            << Console::endl;
        // Rollback : on remet les params originels
        result.stagMoveMm = saved_stagMove;
        result.stagTimeS  = saved_stagTime;
    }

    // ─── Écriture finale + restauration ─────────────────────────────────────
    RuntimeConfig::setFloat("stall.stag_move_mm", result.stagMoveMm);
    RuntimeConfig::setFloat("stall.stag_time",    result.stagTimeS);
    RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);

    if (wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(savedFeedrate);

    Console::info("CalibStall")
        << "DONE iter=" << result.iter
        << " stagMove=" << result.stagMoveMm
        << " stagTime=" << result.stagTimeS
        << " status=" << (result.converged ? "converged" : "abandoned")
        << Console::endl;

    return result;
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
