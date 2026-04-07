#include "routines.h"
#include "os/commands.h"
#include "config/env.h"
#include "config/calibration.h"
#include "config/runtime_config.h"
#include "services/sd/sd_card.h"
#include "services/mission/mission_controller.h"
#include "strategy.h"
#include "block_registry.h"
#include "config/poi.h"
#include "config/score.h"
#include "os/os.h"
#include "utils/timer/timer.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Main programs
// ─────────────────────────────────────────────────────────────────────────────

/**
 * programAuto — executed once when match starts (os.start()).
 *
 * Architecture headless :
 *   - Si le Jetson est connecté (jetsonBridge.isRemoteControlled()),
 *     on entre dans une boucle d'attente de commandes.
 *     Le Jetson envoie des commandes via XBee ; le JetsonBridge les exécute.
 *
 *   - Si le Jetson n'est pas connecté (fallback local),
 *     on exécute match() — la stratégie embarquée sur Teensy.
 *
 *   La boucle d'attente remote est non bloquante : os.run() tourne normalement.
 */
FLASHMEM void programAuto() {
    Console::println("Started match");
    ihm.setPage(IHM::Page::MATCH);
    lidar.enable();
    safety.enable();
    chrono.start();

    // Reset block done flags at match start so fallback runs everything
    BlockRegistry::instance().resetAll();

    // Strategy selection:
    //   strategySwitch ON  (1) → intelligente (Jetson-based, full pathfinding)
    //   strategySwitch OFF (0) → séquentielle (dumb T41-only, stop-on-obstacle)
    bool useIntelligent = ihm.strategySwitch.getState();

    if (useIntelligent && jetsonBridge.isRemoteControlled()) {
        Console::info("OS") << "Intelligent strategy — Jetson remote controlled mode." << Console::endl;
        // The Jetson drives the robot. We stay in loop and let JetsonBridge
        // dispatch incoming commands. Match ends when chrono fires onMatchEnd().
        while (chrono.getTimeLeft() > 0) {
            // Honour pause from match_stop — spin without processing commands
            if (jetsonBridge.isMatchPaused()) {
                os.flush();
                continue;
            }
            os.flush(); // process one iteration
        }
    } else {
        if (!useIntelligent) {
            Console::info("OS") << "Sequential strategy — running embedded match()." << Console::endl;
        } else {
            Console::warn("OS") << "Intelligent strategy selected but Jetson not connected — fallback to embedded match()." << Console::endl;
        }
        match();
    }
}

/**
 * programManual — executed before match (preparation phase).
 * Handles: IHM, starter, recalage, terminal commands, team broadcast.
 */
FLASHMEM void programManual() {
    static bool hadStarter      = false;
    static bool buttonWasPressed = false;

    // ── Remote start from webapp (no arming required) ─────────────────────
    // match_start bridge command sets the flag; we consume it here and
    // trigger the same os.start() path as pulling the physical starter.
    if (jetsonBridge.consumeRemoteStart()) {
        Console::info("OS") << "Remote start — freezing settings, engaging motors." << Console::endl;
        ihm.freezeSettings();
        motion.engage();
        ihm.setPage(IHM::Page::MATCH);
        os.start();
        return;
    }

    // ── Broadcast team to Jetson every second ──────────────────────────────
    {
    RUN_EVERY(
        if(ihm.teamSwitch.getState() == Settings::YELLOW){
            intercom.sendRequest("team(Y)");
        }else{
            intercom.sendRequest("team(B)");
        }
    ,1000);
    }

    // ── Starter inserted → arm ─────────────────────────────────────────────
    if(ihm.hasStarter() && !hadStarter){
        robotArmed();
        hadStarter = true;
        return;
    }

    // ── Starter removed → start match ─────────────────────────────────────
    if(!ihm.hasStarter() && hadStarter && !ihm.buttonPressed()){
        ihm.setPage(IHM::Page::MATCH);
        os.start();
        return;
    }

    // ── Button + starter → abort arm ──────────────────────────────────────
    if(!ihm.hasStarter() && hadStarter && ihm.buttonPressed()){
        motion.disengage();
        ihm.unfreezeSettings();

        ihm.playTone(440, 150);
        ihm.playTone(349.2, 150);
        ihm.playTone(329.2, 150);
        noTone(Pin::Outputs::buzzer);

        lidar.showStatusLED();
        hadStarter = false;
        buttonWasPressed = true;
        return;
    }

    // ── Long press → recalage ─────────────────────────────────────────────
    static bool toneOK = false;
    static bool toneOn = false;

    if(ihm.buttonPressed() && ihm.resetButton.pressDuration() > 1500){
        if(!toneOK){
            tone(Pin::Outputs::buzzer, 442);
            toneOK = true;
            toneOn = true;
        }
        if(ihm.resetButton.pressDuration() > 1950 && toneOn){
            ihm.playTone(783.99, 150);
            noTone(Pin::Outputs::buzzer);
            toneOn = false;
            toneOK = false;
        }
    }

    if((!ihm.buttonPressed()) && (ihm.resetButton.pressDuration() > 1500)
       && (ihm.resetButton.pressDuration() < 1950) && toneOn){
        noTone(Pin::Outputs::buzzer);
        toneOn = false;
        toneOK = false;
        ihm.playTone(440, 150);
        ihm.playTone(349.2, 150);
    }

    if((!ihm.buttonPressed()) &&
       (ihm.resetButton.pressDuration() > 2000) &&
       (ihm.resetButton.pressDuration() < 6000) &&
       (!buttonWasPressed) && (!hadStarter) && (!ihm.hasStarter())){
        Console::println("Recalage :");
        noTone(Pin::Outputs::buzzer);
        ihm.resetButton.resetDuration();

        ihm.playTone(523.25, 150);
        ihm.playTone(659.25, 150);
        ihm.playTone(783.99, 150);
        ihm.playTone(880.00, 100);
        ihm.playTone(783.99, 100);
        ihm.playTone(659.25, 100);
        ihm.playTone(987.77, 250);

        recalage();
        toneOK = false;
        return;
    }

    if(!ihm.buttonPressed() && buttonWasPressed){
        buttonWasPressed = false;
        return;
    }

    // ── Terminal commands ─────────────────────────────────────────────────
    if(terminal.commandAvailable()){
        onTerminalCommand();
        return;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Boot
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void onRobotBoot() {

    // ── Print CrashReport if the Teensy rebooted after a hard fault ────────
    if (CrashReport) {
        Serial.begin(115200);
        Serial.println("======== CRASH REPORT ========");
        Serial.print(CrashReport);
        Serial.println("==============================");
        // Give time for the report to be read over serial
        delay(2000);
    }

    os.attachService(&ihm);
    ihm.drawBootProgress("Linking ihm...");
    ihm.addBootProgress(10);
    ihm.run();

    ihm.drawBootProgress("Linking actuators...");
    os.attachService(&actuators); ihm.addBootProgress(10);

    ihm.drawBootProgress("Linking motion...");
    os.attachService(&motion); ihm.addBootProgress(10);

    ihm.drawBootProgress("Linking chrono...");
    chrono.setNearEndCallback(onMatchNearEnd);
    chrono.setEndCallback(onMatchEnd);
    os.attachService(&chrono); ihm.addBootProgress(10);

    ihm.drawBootProgress("Linking safety...");
    os.attachService(&safety); ihm.addBootProgress(10);
    safety.disable();

    ihm.drawBootProgress("Linking SD card...");
    SDCard::init(); ihm.addBootProgress(5);
    // Load runtime config from SD (actuator limits, motion params, etc.)
    RuntimeConfig::load(); ihm.addBootProgress(1);
    // Restore calibration from SD (if card present and file exists)
    SDCard::loadCalibration(); ihm.addBootProgress(3);
    // Init PCA9685 pump/EV driver early so pump is ready for match start
    ihm.drawBootProgress("Init pump driver...");
    initPump(); ihm.addBootProgress(1);
    // Load mission fallback strategy from SD (if available)
    MissionController::load(); ihm.addBootProgress(1);

    motion.engage();
    ihm.drawBootProgress("Linking Localisation...");
    os.attachService(&localisation); ihm.addBootProgress(10);
    localisation.calibrate();
    motion.disengage();

    ihm.drawBootProgress("Linking intercom...");
    intercom.setConnectLostCallback(onIntercomDisconnected);
    intercom.setConnectionSuccessCallback(onIntercomConnected);
    intercom.setRequestCallback(onIntercomRequest);
    os.attachService(&intercom); ihm.addBootProgress(10);

    ihm.drawBootProgress("Linking JetsonBridge...");
    os.attachService(&jetsonBridge); ihm.addBootProgress(5);
    jetsonBridge.enable();

    // Register custom fallback: return to base
    jetsonBridge.registerFallback(FallbackID::RETURN_TO_BASE, []() {
        motion.cancel();
        safety.enable();

        if(ihm.isColor(Settings::BLUE)) {
            async motion.go(POI::startBlue);
        } else {
            async motion.go(POI::startYellow);
        }
    });

    // Register mission fallback: when Jetson disconnects during intelligent mode,
    // build a mission from the BlockRegistry (skipping already-done blocks) and run it.
    // Falls back to SD mission strategy if no blocks are registered.
    jetsonBridge.registerFallback(FallbackID::CUSTOM_1, []() {
        BlockRegistry& reg = BlockRegistry::instance();
        if (reg.count() > 0) {
            Console::info("Fallback") << "Building mission from BlockRegistry (skipping done blocks)" << Console::endl;
            static Mission fallbackMission;
            fallbackMission = Mission();   // reinit
            reg.buildMission(fallbackMission);
            fallbackMission.run();
        } else if (MissionController::isLoaded()) {
            Console::info("Fallback") << "Executing SD mission strategy" << Console::endl;
            MissionController::execute();
        } else {
            Console::warn("Fallback") << "No fallback available — stopping" << Console::endl;
            motion.cancel();
            motion.disengage();
        }
    });

    ihm.drawBootProgress("Linking lidar...");
    os.attachService(&lidar); ihm.addBootProgress(10);
    lidar.showStatusLED();
    lidar.enable();

    ihm.drawBootProgress("Linking terminal...");
    os.attachService(&terminal); ihm.addBootProgress(10);

    ihm.drawBootProgress("Linking vision...");
    os.attachService(&vision); ihm.addBootProgress(10);

    ihm.drawBootProgress("Registering Commands...");
    registerCommands(); ihm.addBootProgress(5);

    // Register C++ blocks into the BlockRegistry so Jetson can discover
    // and execute them via blocks_list / run_block bridge commands.
    ihm.drawBootProgress("Registering C++ blocks...");
    registerBlocks();

    ihm.resetButton.resetDuration();

    // Register interpreter variables (unchanged from before)
    Expression::registerVariables("A",    "A");
    Expression::registerVariables("AB",   "AB");
    Expression::registerVariables("B",    "B");
    Expression::registerVariables("BC",   "BC");
    Expression::registerVariables("C",    "C");
    Expression::registerVariables("CA",   "CA");
    Expression::registerVariables("NORTH","NORTH");
    Expression::registerVariables("SOUTH","SOUTH");
    Expression::registerVariables("WEST", "WEST");
    Expression::registerVariables("EAST", "EAST");
    Expression::registerVariables("MOTION",      "MOTION");
    Expression::registerVariables("ACTUATORS",   "ACTUATORS");
    Expression::registerVariables("SAFETY",      "SAFETY");
    Expression::registerVariables("CHRONO",      "CHRONO");
    Expression::registerVariables("IHM",         "IHM");
    Expression::registerVariables("INTERCOM",    "INTERCOM");
    Expression::registerVariables("LOCALISATION","LOCALISATION");
    Expression::registerVariables("LIDAR",       "LIDAR");
    Expression::registerVariables("TERMINAL",    "TERMINAL");
    Expression::registerVariables("JETSON",      "JETSON");
    Expression::registerVariables("UP",    "1");
    Expression::registerVariables("DOWN",  "2");
    Expression::registerVariables("GRAB",  "1");
    Expression::registerVariables("DROP",  "0");
    Expression::registerVariables("RIGHT", "1");
    Expression::registerVariables("LEFT",  "0");

    ihm.drawBootProgress("Boot done.");
    ihm.setPage(IHM::Page::INIT);

    ihm.playTone(880.00,  120);
    ihm.playTone(1174.66, 120);
    ihm.playTone(1318.51, 200);
    ihm.playTone(1760.00, 250);
}


// ─────────────────────────────────────────────────────────────────────────────
//  Loop callbacks
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void onRobotManual() {
    RUN_EVERY(
        ihm.setRobotPosition(motion.estimatedPosition());
    ,300);
}

FLASHMEM void onRobotAuto() {
    RUN_EVERY(
        ihm.setRobotPosition(motion.estimatedPosition());
    ,100);
}

FLASHMEM void onRobotStop() {
    ihm.run();
}


// ─────────────────────────────────────────────────────────────────────────────
//  Control ISR (registered with CycleManager)
// ─────────────────────────────────────────────────────────────────────────────

void step() {
    motion.step();
}

void control() {
    motion.control();
}


// ─────────────────────────────────────────────────────────────────────────────
//  Intercom events
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void onIntercomConnected() {
    ihm.setIntercomState(true);
    Console::info("Intercom") << "Connected." << Console::endl;
}

FLASHMEM void onIntercomDisconnected() {
    ihm.setIntercomState(false);
    Console::warn("Intercom") << "Disconnected." << Console::endl;
}

/**
 * onIntercomRequest — Called when a request arrives from Jetson via XBee.
 * All command routing is delegated to JetsonBridge.
 */
FLASHMEM void onIntercomRequest(Request& req) {
    jetsonBridge.handleRequest(req);
}

/**
 * onIntercomRequestReply — Called when a reply arrives to a request WE sent.
 * Currently used to receive occupancy map from Teensy 4.0.
 */
FLASHMEM void onIntercomRequestReply(Request& req) {
    if (strncmp(req.getContent(), "oM", 2) == 0) {
        occupancy.decompress(req.getResponse());
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Match events
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void onMatchNearEnd() {
    nearEnd();
}

FLASHMEM void onMatchEnd() {
    safety.disable();
    lidar.disable();
    motion.cancel();
    motion.disable();
    motion.disengage();
    actuators.disable();
    os.stop();
}


// ─────────────────────────────────────────────────────────────────────────────
//  Terminal
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void onTerminalCommand() {
    if (terminal.commandAvailable() > 0) {
        String rawcmd = terminal.dequeCommand();
        os.execute(rawcmd);
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Utilities
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void robotArmed() {
    lidar.showRadarLED();
    ihm.freezeSettings();
    motion.engage();

    ihm.playTone(329.2, 150);
    ihm.playTone(349.2, 150);
    ihm.playTone(440,   150);

    noTone(Pin::Outputs::buzzer);
}
