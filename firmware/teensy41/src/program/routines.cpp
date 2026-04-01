#include "routines.h"
#include "os/commands.h"
#include "config/env.h"
#include "config/calibration.h"
#include "services/sd/sd_card.h"
#include "services/mission/mission_controller.h"
#include "strategy.h"
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

    if (jetsonBridge.isRemoteControlled()) {
        Console::info("OS") << "Jetson connected — remote controlled mode." << Console::endl;
        // The Jetson drives the robot. We stay in loop and let JetsonBridge
        // dispatch incoming commands. Match ends when chrono fires onMatchEnd().
        while (chrono.getTimeLeft() > 0) {
            os.flush(); // process one iteration
        }
    } else {
        Console::warn("OS") << "Jetson not connected — running embedded fallback strategy." << Console::endl;
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
    // Restore calibration from SD (if card present and file exists)
    SDCard::loadCalibration(); ihm.addBootProgress(4);
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
        async motion.go(POI::home);
    });

    // Register mission fallback: execute SD strategy when Jetson disconnects
    jetsonBridge.registerFallback(FallbackID::CUSTOM_1, []() {
        if (MissionController::isLoaded()) {
            Console::info("Fallback") << "Executing SD mission strategy" << Console::endl;
            MissionController::execute();
        } else {
            Console::warn("Fallback") << "No mission strategy on SD — stopping" << Console::endl;
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
