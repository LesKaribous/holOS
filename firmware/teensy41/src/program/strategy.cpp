#include "strategy.h"
#include "block_registry.h"
#include "config/poi.h"
#include "config/score.h"
#include "config/env.h"
#include "routines.h"
#include "mission.h"
#include "services/lidar/occupancy.h"

// TODO : déplacer dans Actuators
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


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

    Vec2 approach = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, APPROACH_OFFSET).toVec2();
    Vec2 grab     = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET).toVec2();

    float sidewiseoffset_dir = getCompassOrientation(RobotCompass::AB) - 90;
    grab += PolarVec(sidewiseoffset_dir * DEG_TO_RAD, 50).toVec2(); // Offset latéral pour compenser la largeur du préhenseur

    actuators.grab(rc);
    async motion.goAlign(approach, rc, getCompassOrientation(tc));
    motion.cancelOnStall(true); // Le robot stack contre le mur
    async motion.goAlign(grab,     rc, getCompassOrientation(tc)).withStall();

    actuators.moveElevator(rc, ElevatorPose::DOWN);
    waitMs(GRAB_DELAY_MS);
    actuators.store(rc);
    waitMs(GRAB_DELAY_MS);
    actuators.moveElevator(rc, ElevatorPose::STORE);

    motion.cancelOnStall(false);
    safety.enable();
    motion.setFeedrate(1.0f);
    Console::info("Strategy") << "[collectStock] done      SP=" << String(freeStack()) << Console::endl;
}

// Offset commun factored
static void storeStock(Vec2 target, TableCompass tc, RobotCompass rc) {
    constexpr float    APPROACH_OFFSET = 350.0f;
    constexpr float    GRAB_OFFSET     = 120.0f;
    constexpr uint32_t GRAB_DELAY_MS   = 1000;

    Vec2 approach = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, APPROACH_OFFSET).toVec2();
    Vec2 grab     = target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET).toVec2();

    Vec2 grabrecal= target - PolarVec(getCompassOrientation(tc) * DEG_TO_RAD, GRAB_OFFSET-20).toVec2();

    async motion.goAlign(approach, rc, getCompassOrientation(tc));
    safety.disable();
    motion.cancelOnStall(true); // Le robot stack contre le mur
    async motion.goAlign(grab,     rc, getCompassOrientation(tc)).withStall();

    waitMs(GRAB_DELAY_MS);
    actuators.drop(rc);//release just enough
    waitMs(GRAB_DELAY_MS);
    actuators.grab(rc);//Grab means wide open

    motion.setFeedrate(0.3f);
    async motion.goAlign(grabrecal,     rc, getCompassOrientation(tc));
    motion.setFeedrate(1.0f);
    async motion.goAlign(approach,     rc, getCompassOrientation(tc));

    motion.cancelOnStall(false);
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
    collectStock(POI::stockYellow_01, TableCompass::WEST, RobotCompass::AB);
    return motion.wasSuccessful() ? BlockResult::SUCCESS : BlockResult::FAILED;
}

static BlockResult blockStoreA() {
    storeStock(POI::pantry_03 + pantryEmpty * Vec2(50,0), TableCompass::WEST, RobotCompass::AB);
    pantryEmpty = true;
    return motion.wasSuccessful() ? BlockResult::SUCCESS : BlockResult::FAILED;
}

static BlockResult blockCollectB() {
    collectStock(POI::stockYellow_02, TableCompass::WEST, RobotCompass::AB);
    return motion.wasSuccessful() ? BlockResult::SUCCESS : BlockResult::FAILED;
}

static BlockResult blockStoreB() {
    storeStock(POI::pantry_03 + pantryEmpty * Vec2(50,0), TableCompass::WEST, RobotCompass::AB);
    pantryEmpty = true;
    return motion.wasSuccessful() ? BlockResult::SUCCESS : BlockResult::FAILED;
}

static BlockResult thermometer_set() {
    motion.cancelOnStall(true);
    async motion.goAlign(POI::thermometer_hot_yellow, RobotCompass::C, getCompassOrientation(TableCompass::WEST)).withStall();
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_RIGHT, (int) ManipulatorPose::DROP, 100);

    
    async motion.go(POI::thermometer_target_yellow).withStall();
    actuators.getActuatorGroup(RobotCompass::CA).moveServoToPose((int)ServoIDs::GRABBER_RIGHT, (int) ManipulatorPose::STORE, 100);
    motion.cancelOnStall(false);
    return motion.wasSuccessful() ? BlockResult::SUCCESS : BlockResult::FAILED;
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
    bool occupied = occupancy.isZoneOccupied(POI::stockYellow_01, ZoneCheck::RADIUS);
    if (occupied)
        Console::warn("Strategy") << "Zone A occupee — bloc skipe" << Console::endl;
    return !occupied;
}

static bool isZoneBFree() {
    bool occupied = occupancy.isZoneOccupied(POI::stockYellow_02, ZoneCheck::RADIUS);
    if (occupied)
        Console::warn("Strategy") << "Zone B occupee — bloc skipe" << Console::endl;
    return !occupied;
}

static bool isZoneThermoFree() {
    bool occupied = occupancy.isZoneOccupied(POI::thermometer_hot_yellow, ZoneCheck::RADIUS);
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
    reg.add("thermo_set", 10, 150, Timing::THERMO_SET,   thermometer_set, isZoneThermoFree);
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

    motion.setFeedrate(1.0f);
    motion.enableCruiseMode();

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

    { Mission& m = planner.addMission("thermo_set", 10, 150);
      m.addStep("thermo_set", Timing::THERMO_SET, thermometer_set, isZoneThermoFree);
      m.addDependency(stockB);
      m.setMaxRetries(Mission::INFINITE_RETRIES); }

    

    planner.run();
    Vec2 poi_y = Vec2(600,850);//TODO Remove
    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(poi_y /*POI::wait_blue*/).withStall();
    } else {
        async motion.go(POI::wait_yellow).withStall();
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
        motion.setAbsPosition(Vec3( Vec2(3000-125,145), -90 * DEG_TO_RAD));
        motion.goAlign(Vec2(3000-350, 300), RobotCompass::AB, getCompassOrientation(TableCompass::WEST));

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
        motion.setAbsPosition(Vec3(125, 145 ,-90 * DEG_TO_RAD));
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

    // Go to the waiting point near SIMAs
    if(ihm.isColor(Settings::BLUE)) {
        async motion.go(POI::startBlue);
    }
    else {
        async motion.go(POI::startYellow);
    }

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
    //motion.cancelOnStall(false);

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

    //motion.cancelOnStall(false);
    motion.enableCruiseMode();
}


FLASHMEM void probeBorder(TableCompass tc, RobotCompass rc, float clearance, float approachDist, float probeDist, float feedrate){
    
	boolean wasAbsolute = motion.isAbsolute();
    float currentFeedrate = motion.getFeedrate();
    actuators.moveElevator(rc, ElevatorPose::UP);
    
    motion.setFeedrate(feedrate);
	async motion.align(rc, getCompassOrientation(tc));
    motion.setRelative();

    motion.disableCruiseMode();
    motion.cancelOnStall(true);
	async motion.goPolar(getCompassOrientation(rc),approachDist);
	async motion.goPolar(getCompassOrientation(rc),probeDist);
    motion.cancelOnStall(false);
    motion.enableCruiseMode();
    
	float _offset = getOffsets(rc);
    Console::println(_offset);
	Vec3 position = motion.estimatedPosition();
    Console::println(position);

	if(tc == TableCompass::NORTH){
		position.y = 0.0 + _offset; //We hit Xmax
		//_probedX = true;
	}else if(tc == TableCompass::SOUTH){
		position.y = 2000.0 - _offset; //We hit Xmin
		//_probedX = true;
	}else if(tc == TableCompass::EAST){
		position.x = 3000.0 - _offset; //We hit Ymax
		//_probedY = true;
	}else if(tc == TableCompass::WEST){
		position.x = 0.0 + _offset; //We hit Ymin
		//_probedY = true;
	}
    position.c = DEG_TO_RAD * (getCompassOrientation(tc) - getCompassOrientation(rc));
	
    motion.setAbsPosition(position);
    delay(200);
    
    if(clearance != 0){ 
        async motion.goPolar(getCompassOrientation(rc),-clearance);
    }
    
	if(wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(currentFeedrate);
    actuators.moveElevator(rc, ElevatorPose::DOWN);
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