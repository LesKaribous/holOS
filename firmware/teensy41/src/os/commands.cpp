#include "commands.h"
#include "config/env.h"
#include "config/calibration.h"
#include "config/runtime_config.h"
#include "services/sd/sd_card.h"
#include "services/localisation/localisation.h"
#include "services/motion/motion.h"
#include "services/mission/mission_controller.h"
#include "program/routines.h"
#include "program/strategy.h"
#include "config/poi.h"

FLASHMEM void registerCommands() {
    CommandHandler::registerCommand("start", "Start Match", command_start);
    CommandHandler::registerCommand("stop", "Stop Robot", command_stop);
    CommandHandler::registerCommand("reboot", "Reboot Robot", command_reboot);

    CommandHandler::registerCommand("enable(service)", "Enable a specific service", command_enable);
    CommandHandler::registerCommand("disable(service)", "Disable a specific service", command_disable);
    CommandHandler::registerCommand("status(service)", "Display single status", command_status);
    CommandHandler::registerCommand("wait(duration)", "Wait a bit for duration", command_wait);
    CommandHandler::registerCommand("lidarMode(mode)", "Change neopixel display mode on lidar", command_lidarMode);
    CommandHandler::registerCommand("go(x,y)", "Move to a specific position", command_go);
    CommandHandler::registerCommand("go_coc(x,y)", "Move with cancel on collide/stall", command_go_coc);
    CommandHandler::registerCommand("via(x,y)", "Add pass-through waypoint (for chained via;go)", command_via);
    CommandHandler::registerCommand("goPolar(angle,dist)", "Move to a relative polar position", command_goPolar);
    CommandHandler::registerCommand("goAlign(x,y,angle)", "Move to position and align to angle (degrees)", command_goAlign);
    CommandHandler::registerCommand("move(x,y,angle)", "Move to a specific position", command_move);
    CommandHandler::registerCommand("turn(angle)", "Turn to a specific angle", command_turn);
    CommandHandler::registerCommand("rawTurn(angle)", "Turn to a specific angle without optimization", command_rawTurn);
    CommandHandler::registerCommand("pause", "Pause motion", command_pause);
    CommandHandler::registerCommand("resume", "Resume motion", command_resume);
    CommandHandler::registerCommand("cancel", "Cancel motion", command_cancel);
    CommandHandler::registerCommand("probe(tableCompass, side)", "Probe border", command_probe);
    CommandHandler::registerCommand("sleep", "Put motion to sleep", command_sleep);
    CommandHandler::registerCommand("wake", "Wake up motion", command_wake);
    CommandHandler::registerCommand("align(side,angle)", "Align to a specific side and angle", command_align);
    CommandHandler::registerCommand("setAbsolute", "Set motion to absolute mode", command_setAbsolute);
    CommandHandler::registerCommand("setRelative", "Set motion to relative mode", command_setRelative);
    CommandHandler::registerCommand("setAbsPosition(x,y,angle)", "Set absolute position", command_setAbsPosition);
    CommandHandler::registerCommand("setAbsolutePosition(x,y,angle)", "Set absolute position", command_setAbsPosition);
    CommandHandler::registerCommand("resetCompass", "Reset compass and set to 0", command_resetCompass);
    CommandHandler::registerCommand("elevator(side, pose)", "Raise elevator to desired pose", command_elevator);
    CommandHandler::registerCommand("moveElevator(side,angle)", "Raise elevator to desired angle", command_move_elevator);
    CommandHandler::registerCommand("raise(side)", "Raise elevator", command_raise);
    CommandHandler::registerCommand("lower(side)", "Lower elevator", command_lower);
    CommandHandler::registerCommand("grab(side)", "Grab object using actuator", command_grab);
    CommandHandler::registerCommand("drop(side)", "Drop object using actuator", command_drop);
    CommandHandler::registerCommand("store(side)", "Store object using actuator", command_store);
    CommandHandler::registerCommand("pump(side)", "enable pump", command_pump);
    CommandHandler::registerCommand("ev(side)", " disable pump", command_ev);
    CommandHandler::registerCommand("initPump", " Init Pump", command_initPump);
    CommandHandler::registerCommand("cruise", " Enable cruise mode", command_cruise);
    CommandHandler::registerCommand("feed(feedrate)", " Set Move feedrate", command_feed);
    CommandHandler::registerCommand("music", " Play a sound", command_music);
    CommandHandler::registerCommand("radar", " Toogle radar view on neopixel", command_radar);
    CommandHandler::registerCommand("test", " Dummy Test function ", command_test);
    CommandHandler::registerCommand("scale(value)", " Set otos linear scale", command_otos_scale);
    CommandHandler::registerCommand("calibrate", " calibrate otos linear scale", command_otos_calibration);
    
    CommandHandler::registerCommand("servo(side, servoID, pose)", "move servo to position", command_servo);
    //CommandHandler::registerCommand("servo_pos(side, servoID, pose)", "move servo to pose", command_servo_pos);
    CommandHandler::registerCommand("printServo(side)", "print servo mapping", command_printServo);
    CommandHandler::registerCommand("collisionDetect(state)", "Toggle collision detection", command_collision_detect);

    //CommandHandler::registerCommand("open(side)", "Open actuator on a specific side", command_open);
    //CommandHandler::registerCommand("close(side)", "Close actuator on a specific side", command_close);
    CommandHandler::registerCommand("recalage()", "Execute recalage routine", command_recalage);
    CommandHandler::registerCommand("print(value)", "Print the result of an expression in the terminal", command_print);
    CommandHandler::registerCommand("stats", "Print cyclic stats", command_stats);
    CommandHandler::registerCommand("health", "Print full service health snapshot", command_health);
    CommandHandler::registerCommand("help", "Display help", command_help);
    CommandHandler::registerCommand("debug(service)", "Toggle debug/trace for a service", command_debug);

    // Calibration
    CommandHandler::registerCommand("calib_status",              "Print all calibration values",              command_calib_status);
    CommandHandler::registerCommand("calib_save",                "Save calibration to SD",                    command_calib_save);
    CommandHandler::registerCommand("calib_load",                "Load calibration from SD",                  command_calib_load);
    CommandHandler::registerCommand("calib_reset",               "Reset calibration to defaults",             command_calib_reset);
    CommandHandler::registerCommand("calib_cart(x,y,rot)",       "Set Cartesian scale factors",               command_calib_cart);
    CommandHandler::registerCommand("calib_holo(a,b,c)",         "Set per-wheel holonomic scale factors",     command_calib_holo);
    CommandHandler::registerCommand("calib_otos_linear(value)",  "Set OTOS linear scalar",                    command_calib_otos_linear);
    CommandHandler::registerCommand("calib_otos_angular(value)", "Set OTOS angular scalar",                   command_calib_otos_angular);
    CommandHandler::registerCommand("calib_measure(dist_mm)",    "Open-loop move + report OTOS measurement",  command_calib_measure);

    // Motion — arc / rotation around arbitrary point
    CommandHandler::registerCommand("goAround(cx,cy,angleDeg)",  "Rotate around point (cx,cy) by angle",      command_goAround);

    // Motion — APF obstacle avoidance
    CommandHandler::registerCommand("apf(on[,scale])",           "Enable/disable APF avoidance (cruise mode only)", command_apf);

    // Mission SD write session (used by holOS deploy-to-SD)
    CommandHandler::registerCommand("mission_sd_open",           "Open /mission_fallback.cfg for writing",     command_mission_sd_open);
    CommandHandler::registerCommand("mission_sd_line(text)",     "Append a line to the open mission file",    command_mission_sd_line);
    CommandHandler::registerCommand("mission_sd_close",          "Close mission file and reload strategy",     command_mission_sd_close);

    // Mission execution (test fallback from terminal / routine)
    CommandHandler::registerCommand("mission_run",               "Execute SD fallback strategy now",          command_mission_run);
    CommandHandler::registerCommand("mission_abort",             "Abort running fallback strategy",           command_mission_abort);

    // Log & telemetry control (useful without holOS, on plain serial terminal)
    CommandHandler::registerCommand("log(source,0|1)",           "Enable/disable a service log source",       command_log);
    CommandHandler::registerCommand("loglevel(level)",           "Set global log level (VERBOSE/INFO/…)",     command_loglevel);
    CommandHandler::registerCommand("tel(channel,0|1)",          "Enable/disable a telemetry channel",        command_tel);
    CommandHandler::registerCommand("logstatus",                 "Print log level + source mask + tel state", command_logstatus);

    // Runtime config (SD-backed key=value store)
    CommandHandler::registerCommand("cfg_list",                  "Print all runtime config entries",          command_cfg_list);
    CommandHandler::registerCommand("cfg_set(key,value)",        "Set a config value at runtime",             command_cfg_set);
    CommandHandler::registerCommand("cfg_save",                  "Save runtime config to SD card",            command_cfg_save);
    CommandHandler::registerCommand("cfg_load",                  "Reload runtime config from SD card",        command_cfg_load);

    // Actuator info (structured, for holOS UI)
    CommandHandler::registerCommand("act_info(side)",            "Print servo info (id,name,min,max,pos)",    command_act_info);
    CommandHandler::registerCommand("servo_limits(side,id,min,max)", "Set servo min/max limits at runtime",   command_servo_limits);
}

FLASHMEM void command_stats(const args_t& args){
    Console::info("Interpreter") << "Stats: " << Console::endl;
    Console::info() << "10us cycle footprint: " << cycle_manager.getCycleFootprint_ms(CycleFrequency::T_10US) << "ms" << Console::endl;
    Console::info() << "1ms cycle footprint: " << cycle_manager.getCycleFootprint_ms(CycleFrequency::T_1MS) << "ms" << Console::endl;
}

FLASHMEM void command_enable(const args_t& args){
    if(args.size() != 1) return;
    ServiceID serviceID = Service::toID(args[0]);
    if(serviceID != ID_NOT_A_SERVICE){
        os.enable(serviceID);
        Console::info("Interpreter") << args[0] <<  " enabled" << Console::endl;
    }else  Console::error("Interpreter") << "unknown service" << Console::endl;
}

FLASHMEM void command_disable(const args_t& args){
    if(args.size() != 1) return;
    ServiceID serviceID = Service::toID(args[0]);
    if(serviceID != ID_NOT_A_SERVICE){
        os.disable(serviceID);
        Console::info("Interpreter") << args[0] <<  " disabled" << Console::endl;
    }else  Console::error("Interpreter") << "unknown service" << Console::endl;
}

FLASHMEM void command_status(const args_t& args){
    if(args.size() != 1){
        for ( int id = 0; id != ServiceID::ID_NOT_A_SERVICE; id++ ){
           ServiceID sID = static_cast<ServiceID>(id);
           Console::info("Interpreter") << Service::toString(sID) <<  " : " << (OS::instance().statusService(sID) ? "ON" : "OFF") << Console::endl;
        }
    }else{
        ServiceID serviceID = Service::toID(args[0]);
        Console::info("Interpreter") << args[0] <<  " : " << (OS::instance().statusService(serviceID) ? "ON" : "OFF") << Console::endl;
    }
} 


FLASHMEM void command_debug(const args_t& args){
    if(args.size() != 1){
        for ( int id = 0; id != ServiceID::ID_NOT_A_SERVICE; id++ ){
           ServiceID sID = static_cast<ServiceID>(id);
           os.toggleDebug(sID);
           Console::info("Interpreter") << Service::toString(sID) <<  " debug : " << (OS::instance().debug(sID) ? "ON" : "OFF") << Console::endl;
        }
    }else{
        ServiceID serviceID = Service::toID(args[0]);
        os.toggleDebug(serviceID);
        Console::info("Interpreter") << args[0] <<  " debug : "  << (OS::instance().debug(serviceID) ? "ON" : "OFF") << Console::endl;
    }
}


//Lidar

FLASHMEM void command_lidarMode(const args_t& args){
    if(args.size() != 1) return;
    if(args[0].equalsIgnoreCase("radar"))lidar.showRadarLED();
    else if(args[0].equalsIgnoreCase("intercom"))lidar.showStatusLED();
}

FLASHMEM void command_wait(const args_t &args){
    if(args.size() != 1) return;
    float duration = args[0].toFloat();
    os.wait(duration);
}

FLASHMEM void command_start(const args_t &args){
    robotArmed();
    os.start();
}

FLASHMEM void command_stop(const args_t &args){
    os.stop();
}

FLASHMEM void command_reboot(const args_t &args){
    os.reboot();
}

FLASHMEM void command_cruise(const args_t& args){
    motion.enableCruiseMode();
}

FLASHMEM void command_feed(const args_t &args){
    if(args.size() != 1) return;
    float feedrate = args[0].toFloat();
    std::clamp(feedrate, 0.05f, 1.0f);
    motion.setFeedrate(feedrate);
    //motion.feed(1000);
}

FLASHMEM void command_music(const args_t &args){
    ihm.playTone(659, 167); // E5, 8th note
    ihm.playTone(587, 167); // D5
    ihm.playTone(370, 333); // F#4, quarter note
    ihm.playTone(415, 333); // G#4

    ihm.playTone(554, 167); // C#5
    ihm.playTone(494, 167); // B4
    ihm.playTone(294, 333); // D4
    ihm.playTone(330, 333); // E4

    ihm.playTone(494, 167); // B4
    ihm.playTone(440, 167); // A4
    ihm.playTone(280, 333); // C4
    //ihm.playTone(262, 333); // C4
    ihm.playTone(330, 333); // E4

    ihm.playTone(440, 667); // A4, half note

}

FLASHMEM void command_radar(const args_t &args){
    static bool on = false;
    on = !on;
    Console::println( on ? "on" : "off");
    intercom.sendRequest(on ? "on" : "off",100);
}

FLASHMEM void command_test(const args_t &args){
    motion.setAbsPosition({913, 1440, 90 * DEG_TO_RAD});
    bool isYellow = ihm.isColor(Settings::YELLOW);
    initPump();
}

FLASHMEM void command_probe(const args_t &args){
    if(args.size() != 2)return;
    String tableCompass = args[0];
    String side = args[1];

    RobotCompass rc = RobotCompass::A;
    if(side.equalsIgnoreCase("A"))         rc = RobotCompass::A;
    else if(side.equalsIgnoreCase("AB"))   rc = RobotCompass::AB;
    else if(side.equalsIgnoreCase("B"))    rc = RobotCompass::B;
    else if(side.equalsIgnoreCase("BC"))   rc = RobotCompass::BC;
    else if(side.equalsIgnoreCase("C"))    rc = RobotCompass::C;
    else if(side.equalsIgnoreCase("CA"))   rc = RobotCompass::CA;

    TableCompass tc = TableCompass::NORTH;
    if(tableCompass.equalsIgnoreCase("NORTH"))         tc = TableCompass::NORTH;
    else if(tableCompass.equalsIgnoreCase("EAST"))   tc = TableCompass::EAST;
    else if(tableCompass.equalsIgnoreCase("SOUTH"))    tc = TableCompass::SOUTH;
    else if(tableCompass.equalsIgnoreCase("WEST"))   tc = TableCompass::WEST;

    probeBorder(tc, rc, 100);

}

// Motion — pass-through waypoint (used in chained via(x,y);go(x,y) commands)
FLASHMEM void command_via(const args_t& args){
    if(args.size() != 2) return;
    float x = args[0].toFloat();
    float y = args[1].toFloat();
    motion.via(x, y);
}

// Motion — go with cancel-on-stall (sent by Python as go_coc)
FLASHMEM void command_go_coc(const args_t& args){
    if(args.size() != 2) return;
    float x = args[0].toFloat();
    float y = args[1].toFloat();
    async motion.cancelOnStall().go(x, y);
}

// Motion — move to (x,y) and align to angle (degrees, firmware frame)
// Python pre-computes the final angle from RobotCompass + orientation.
FLASHMEM void command_goAlign(const args_t& args){
    if(args.size() != 3) return;
    float x     = args[0].toFloat();
    float y     = args[1].toFloat();
    float angle = args[2].toFloat();
    async motion.move(Vec3(x, y, angle));
}

//Motion
FLASHMEM void command_go(const args_t& args){
    /**/
    if(args.size() != 2) return;
    float x = args[0].toFloat();
    float y = args[1].toFloat();
    async motion.go(x, y);
    /**/
}

//Motion
FLASHMEM void command_goPolar(const args_t& args){
    /**/
    if(args.size() != 2) return;
    float angle = args[0].toFloat();
    float dist = args[1].toFloat();
    async motion.goPolar(angle, dist);
    /**/
}

FLASHMEM void command_move(const args_t& args){
    /**/
    if(args.size() != 3) return;
    float x = args[0].toFloat();
    float y = args[1].toFloat();
    float z = args[2].toFloat();
    Console::info("Interpreter") << "Move to (" << x << ", " << y << ", " << z << ")" << Console::endl;
    async motion.move({x, y, z});
    /**/
}


FLASHMEM void command_turn(const args_t& args){
    /**/
    if(args.size() != 1)return;
    float x = args[0].toFloat();
    async motion.turn(x);
    /**/
}

FLASHMEM void command_rawTurn(const args_t& args){
    /**/
    if(args.size() != 1)return;
    float x = args[0].toFloat();

    async motion.withOptimization(false).turn(x);
    /**/
}

FLASHMEM void command_pause(const args_t& args){
    motion.pause();
}

FLASHMEM void command_resume(const args_t& args){
    motion.resume();
}

FLASHMEM void command_cancel(const args_t& args){
    motion.cancel();
}


FLASHMEM void command_otos_calibration(const args_t& args){
    calibrate();
}


FLASHMEM void command_otos_scale(const args_t& args){
    if(args.size() != 1)return;
    float x = args[0].toFloat();
    localisation.setLinearScale(x);
}

FLASHMEM void command_align(const args_t& args){
    /**/
    if(args.size() != 2)return;
    String side = args[0];
    float orientation = args[1].toFloat();
    if(side.equalsIgnoreCase("A"))         async motion.align(RobotCompass::A, orientation);
    else if(side.equalsIgnoreCase("AB"))   async motion.align(RobotCompass::AB, orientation);
    else if(side.equalsIgnoreCase("B"))    async motion.align(RobotCompass::B, orientation);
    else if(side.equalsIgnoreCase("BC"))   async motion.align(RobotCompass::BC, orientation);
    else if(side.equalsIgnoreCase("C"))    async motion.align(RobotCompass::C, orientation);
    else if(side.equalsIgnoreCase("CA"))   async motion.align(RobotCompass::CA, orientation);
    /**/
}



FLASHMEM void command_setAbsolute(const args_t& args){
    /**/
    motion.setAbsolute();
    /**/
}


FLASHMEM void command_setRelative(const args_t& args){
    /**/
    motion.setRelative();
    /**/
}


FLASHMEM void command_setAbsPosition(const args_t& args){
    if(args.size() != 3)return;
    float x = args[0].toFloat();
    float y = args[1].toFloat();
    float angle = args[2].toFloat() * DEG_TO_RAD;
    motion.setAbsPosition({x, y, angle});
}


FLASHMEM void command_resetCompass(const args_t& args){
    //Not implemented yet
}


FLASHMEM void command_collision_detect(const args_t& args){
    if(args.size() != 1) return;
    bool state = args[0] == "1" || args[0].equalsIgnoreCase("true") || args[0].equalsIgnoreCase("on");
    motion.cancelOnStall(state);
    if(state) Console::info("Interpreter") << "Stall detection enabled" << Console::endl;
    else Console::info("Interpreter") << "Stall detection disabled" << Console::endl;
}



//Actuators

FLASHMEM void command_drop(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(side.equals("AB")) actuators.drop(RobotCompass::AB);
    //else if(side.equals("BC")) actuators.drop(RobotCompass::BC);
    else if(side.equals("CA")) actuators.drop(RobotCompass::CA);
}


FLASHMEM void command_grab(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(side.equals("AB")) actuators.grab(RobotCompass::AB);
    //else if(side.equals("BC")) actuators.grab(RobotCompass::BC);
    else if(side.equals("CA")) actuators.grab(RobotCompass::CA);
}

FLASHMEM void command_store(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(side.equals("AB")) actuators.store(RobotCompass::AB);
    //else if(side.equals("BC")) actuators.store(RobotCompass::BC);
    else if(side.equals("CA")) actuators.store(RobotCompass::CA);
}


FLASHMEM void command_pump(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(side.equals("1")) startPump(RobotCompass::CA, RIGHT);
    //else if(side.equals("BC")) actuators.grab(RobotCompass::BC);
    else if(side.equals("0")) startPump(RobotCompass::CA, LEFT);
}

FLASHMEM void command_ev(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(side.equals("1")) stopPump(RobotCompass::CA, 500, RIGHT);
    //else if(side.equals("BC")) actuators.grab(RobotCompass::BC);
    else if(side.equals("0")) stopPump(RobotCompass::CA, 500, LEFT);
}
FLASHMEM void command_initPump(const args_t& args){
    initPump();
}


FLASHMEM void command_elevator(const args_t& args){
    if(args.size() != 2)return;
    const String& side = args[0];
    const String& poseStr = args[1];
    int pose = poseStr.toInt();

    if(side.equals("AB")) actuators.moveElevator(RobotCompass::AB, ElevatorPose(pose));
    else if(side.equals("BC")) actuators.moveElevator(RobotCompass::BC, ElevatorPose(pose));
    else if(side.equals("CA")) actuators.moveElevator(RobotCompass::CA, ElevatorPose(pose));
}

FLASHMEM void command_move_elevator(const args_t& args){
    if(args.size() != 2)return;
    const String& side = args[0];
    const String& poseStr = args[1];
    int pose = poseStr.toInt();

    if(side.equals("AB")) actuators.moveElevatorAngle(RobotCompass::AB, pose);
    else if(side.equals("BC")) actuators.moveElevatorAngle(RobotCompass::BC, pose);
    else if(side.equals("CA")) actuators.moveElevatorAngle(RobotCompass::CA, pose);
}

FLASHMEM void command_servo(const args_t &args){
    if(args.size() != 3)return;
    const String& side = args[0];
    const String& servoStr = args[1];
    const String& poseStr = args[2];
    int servo = servoStr.toInt();
    int pose = poseStr.toInt();
    

    if(!validCompassString(side)) return;
    RobotCompass rc = compassFromString(side);
    ActuatorGroup& group = actuators.getActuatorGroup(rc);

    if(group.hasServo(servo)){
        group.getServo(servo).moveTo(pose, 100);
    } else Console::error("Interpreter") << "Servo " << servo << " not found in group " << side << Console::endl;
}

FLASHMEM void command_printServo(const args_t &args){
    if(args.size() != 1)return;
    const String& side = args[0];
    if(!validCompassString(side)) return;
    RobotCompass rc = compassFromString(side);
    ActuatorGroup& group = actuators.getActuatorGroup(rc);

    Console::info("Interpreter") << "Servos in group " << side << " :" << Console::endl;
    group.listServo();

}


FLASHMEM void command_raise(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];

    if(side.equals("AB")) actuators.moveElevator(RobotCompass::AB, ElevatorPose::UP);
    else if(side.equals("BC")) actuators.moveElevator(RobotCompass::BC, ElevatorPose::UP);
    else if(side.equals("CA")) actuators.moveElevator(RobotCompass::CA, ElevatorPose::UP);
}

FLASHMEM void command_lower(const args_t& args){
    if(args.size() != 1)return;
    const String& side = args[0];

    if(side.equals("AB")) actuators.moveElevator(RobotCompass::AB, ElevatorPose::DOWN);
    else if(side.equals("BC")) actuators.moveElevator(RobotCompass::BC, ElevatorPose::DOWN);
    else if(side.equals("CA")) actuators.moveElevator(RobotCompass::CA, ElevatorPose::DOWN);
}

//Routines 

FLASHMEM void command_recalage(const args_t& args){
    recalage();
}


//Terminal
FLASHMEM void command_help(const args_t& args){
    auto& commands = CommandHandler::getCommands();
    for (auto i = commands.begin(); i != commands.end(); i++){
        Command c = i->second;
        Console::print(c.getSyntax()); 
        Console::print(" : "); 
        Console::println(c.getDesc()); 
    }
}


FLASHMEM void command_print(const args_t& args){
    for(arg_t a : args){
        Console::print(a);   
    }
}

FLASHMEM void command_wake(const args_t& args){
    motion.engage();
    actuators.enable();
}

FLASHMEM void command_sleep(const args_t& args){
    motion.disengage();
    actuators.disable();
}

FLASHMEM void command_health(const args_t& args){
    // mo=motion,mv=isMoving,sa=safety,ob=obstacle,ch=chrono,el=elapsed(ms),
    // ac=actuators,li=lidar,ic=intercom,ic_ok=T4.0 connected,
    // jt=jetsonBridge,jt_ok=jetsonConnected,vi=vision,lo=localisation,
    // apf=APF avoidance enabled
    char buf[160];
    snprintf(buf, sizeof(buf),
        "mo=%d,mv=%d,sa=%d,ob=%d,ch=%d,el=%ld,"
        "ac=%d,li=%d,ic=%d,ic_ok=%d,jt=%d,jt_ok=%d,vi=%d,lo=%d,apf=%d",
        motion.enabled()              ? 1 : 0,
        motion.isMoving()             ? 1 : 0,
        safety.enabled()              ? 1 : 0,
        safety.obstacleDetected()     ? 1 : 0,
        chrono.enabled()              ? 1 : 0,
        chrono.getElapsedTime(),
        actuators.enabled()           ? 1 : 0,
        lidar.enabled()               ? 1 : 0,
        intercom.enabled()            ? 1 : 0,
        intercom.isConnected()        ? 1 : 0,
        jetsonBridge.enabled()        ? 1 : 0,
        jetsonBridge.jetsonConnected()? 1 : 0,
        vision.enabled()              ? 1 : 0,
        localisation.enabled()        ? 1 : 0,
        motion.isAPFEnabled()         ? 1 : 0);
    Console::println(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Calibration commands
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void command_calib_status(const args_t& args) {
    char buf[256];
    Calibration::toString(buf, sizeof(buf));
    Console::info("Calib") << "Current profile: " << buf << Console::endl;
    Console::info("Calib") << "SD ready: " << (SDCard::isReady() ? "yes" : "no") << Console::endl;
}

FLASHMEM void command_calib_save(const args_t& args) {
    if (!SDCard::isReady()) {
        Console::error("Calib") << "SD not available — insert card and reboot" << Console::endl;
        return;
    }
    SDCard::saveCalibration();
}

FLASHMEM void command_calib_load(const args_t& args) {
    if (!SDCard::isReady()) {
        Console::error("Calib") << "SD not available" << Console::endl;
        return;
    }
    SDCard::loadCalibration();
}

FLASHMEM void command_calib_reset(const args_t& args) {
    Calibration::reset();
    char buf[256];
    Calibration::toString(buf, sizeof(buf));
    Console::info("Calib") << "Reset to defaults: " << buf << Console::endl;
}

FLASHMEM void command_calib_cart(const args_t& args) {
    if (args.size() != 3) {
        Console::error("Calib") << "Usage: calib_cart(x, y, rot)" << Console::endl;
        return;
    }
    float x   = args[0].toFloat();
    float y   = args[1].toFloat();
    float rot = args[2].toFloat();
    Calibration::setCartesian(x, y, rot);
    Console::info("Calib") << "Cartesian → cx=" << x << " cy=" << y << " cr=" << rot << Console::endl;
}

FLASHMEM void command_calib_holo(const args_t& args) {
    if (args.size() != 3) {
        Console::error("Calib") << "Usage: calib_holo(a, b, c)" << Console::endl;
        return;
    }
    float a = args[0].toFloat();
    float b = args[1].toFloat();
    float c = args[2].toFloat();
    Calibration::setHolonomic(a, b, c);
    Console::info("Calib") << "Holonomic → ha=" << a << " hb=" << b << " hc=" << c << Console::endl;
}

FLASHMEM void command_calib_otos_linear(const args_t& args) {
    if (args.size() != 1) {
        Console::error("Calib") << "Usage: calib_otos_linear(value)" << Console::endl;
        return;
    }
    float v = args[0].toFloat();
    if (v < 0.872f || v > 1.127f) {
        Console::warn("Calib") << "OTOS linear scalar out of spec (0.872–1.127): " << v << Console::endl;
    }
    Calibration::setOtosLinear(v);
    Console::info("Calib") << "OTOS linear → " << v << Console::endl;
}

FLASHMEM void command_calib_otos_angular(const args_t& args) {
    if (args.size() != 1) {
        Console::error("Calib") << "Usage: calib_otos_angular(value)" << Console::endl;
        return;
    }
    float v = args[0].toFloat();
    if (v < 0.872f || v > 1.127f) {
        Console::warn("Calib") << "OTOS angular scalar out of spec (0.872–1.127): " << v << Console::endl;
    }
    Calibration::setOtosAngular(v);
    Console::info("Calib") << "OTOS angular → " << v << Console::endl;
}

/**
 * calib_measure(dist_mm) — open-loop move + OTOS measurement.
 *
 * Procedure:
 *   1. Records current OTOS position (with current linear scale applied).
 *   2. Moves forward dist_mm using stepper mode (no OTOS feedback → scale error
 *      doesn't affect the move distance, steppers give ground truth).
 *   3. Reads final OTOS position.
 *   4. Reports measured distance and the suggested corrected OtosLinear.
 *
 * Use this to calibrate calib_otos_linear:
 *   - Place a ruler; run calib_measure(500)
 *   - If OTOS reports 480mm for a 500mm move: ratio = 500/480 ≈ 1.042
 *   - New OtosLinear = current * ratio  (or use the printed suggestion directly)
 *
 * Note: the robot must be in a clear straight path of at least dist_mm.
 * Stall detection is disabled for this move to avoid false positives.
 */
FLASHMEM void command_calib_measure(const args_t& args) {
    if (args.size() != 1) {
        Console::error("Calib") << "Usage: calib_measure(dist_mm)" << Console::endl;
        return;
    }
    float dist = args[0].toFloat();
    if (dist < 50.0f || dist > 3000.0f) {
        Console::error("Calib") << "dist_mm must be 50–3000" << Console::endl;
        return;
    }

    // Force stepper mode so the move doesn't use OTOS feedback
    // (we want to measure OTOS, not correct with it)
    motion.disableCruiseMode();

    Vec3 startPos = localisation.getPosition();
    Console::info("Calib") << "Start pos: x=" << startPos.x << " y=" << startPos.y << Console::endl;

    // Move forward, no stall detection (avoid false positive on calibration surface)
    async motion.noStall().goPolar(0, dist);

    // Wait for completion while keeping ISR-driven services alive
    // (delay() is safe here — motion ISR runs via interrupt)
    unsigned long t0 = millis();
    static constexpr unsigned long TIMEOUT_MS = 15000UL;
    while (!motion.hasFinished() && (millis() - t0) < TIMEOUT_MS) {
        delay(20);
    }

    motion.enableCruiseMode();

    Vec3 endPos = localisation.getPosition();
    float dx = endPos.x - startPos.x;
    float dy = endPos.y - startPos.y;
    float otosDistance = sqrtf(dx * dx + dy * dy);

    Console::info("Calib") << "─────────────────────────────────" << Console::endl;
    Console::info("Calib") << "Target distance  : " << dist << " mm" << Console::endl;
    Console::info("Calib") << "OTOS measured    : " << otosDistance << " mm" << Console::endl;

    if (otosDistance > 10.0f) {
        float ratio = dist / otosDistance;
        float suggested = Calibration::OtosLinear * ratio;
        Console::info("Calib") << "Current OtosLinear : " << Calibration::OtosLinear << Console::endl;
        Console::info("Calib") << "Ratio (target/otos): " << ratio << Console::endl;
        Console::info("Calib") << "→ Suggested : calib_otos_linear(" << suggested << ")" << Console::endl;
        Console::info("Calib") << "─────────────────────────────────" << Console::endl;
    } else {
        Console::error("Calib") << "OTOS reading too small — is localisation enabled?" << Console::endl;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Motion — rotation around arbitrary point
// ─────────────────────────────────────────────────────────────────────────────

/**
 * goAround(cx, cy, angleDeg) — rotate around pivot (cx,cy) by angleDeg.
 *
 * The robot traces a circular arc of radius = distance(robot, pivot).
 * Its absolute orientation changes by the same angleDeg (robot rotates in-place
 * relative to the pivot — i.e. it always faces the same direction relative to
 * the arc tangent unless the motion controller adjusts otherwise).
 *
 * The arc is approximated with N intermediate waypoints (passThrough).
 * A final go() targets the exact endpoint with the correct heading.
 *
 * Example:
 *   goAround(1500, 1000, 90)   // orbit 90° CW around table centre
 *   goAround(0, 0, -180)       // half-circle CCW around origin
 */
FLASHMEM void command_goAround(const args_t& args) {
    if (args.size() != 3) {
        Console::error("Interpreter") << "Usage: goAround(cx, cy, angleDeg)" << Console::endl;
        return;
    }
    float cx       = args[0].toFloat();
    float cy       = args[1].toFloat();
    float angleDeg = args[2].toFloat();

    Vec3  pos   = motion.getAbsPosition();
    float rx    = pos.x - cx;           // vector robot → pivot
    float ry    = pos.y - cy;
    float total = angleDeg * DEG_TO_RAD;

    // Number of arc segments — ≥4, ~one segment per 22.5° (16/rev)
    int N = max(4, (int)(fabsf(angleDeg) / 22.5f));

    // Intermediate passThrough waypoints
    for (int i = 1; i < N; i++) {
        float theta = total * (float)i / (float)N;
        float nx = cx + rx * cosf(theta) - ry * sinf(theta);
        float ny = cy + rx * sinf(theta) + ry * cosf(theta);
        motion.via(nx, ny);
    }

    // Final target with heading rotated by angleDeg
    float nx  = cx + rx * cosf(total) - ry * sinf(total);
    float ny  = cy + rx * sinf(total) + ry * cosf(total);
    float nz  = pos.z + total;  // maintain orientation change
    async motion.move({nx, ny, nz});
}

// ─────────────────────────────────────────────────────────────────────────────
//  APF — Artificial Potential Fields obstacle avoidance
// ─────────────────────────────────────────────────────────────────────────────

/**
 * apf(on [, scale]) — Enable or disable APF avoidance.
 *
 * Arguments:
 *   on    : 1 / true / on  → enable
 *           0 / false / off → disable
 *   scale : optional float — multiplier applied to the repulsive gradient
 *           output (mm/s per gradient unit).  Default 50000.
 *           Increase for stronger avoidance; decrease if oscillations occur.
 *
 * Notes:
 *   - Only works in cruise mode (OTOS odometry required).
 *   - The occupancy map must be up-to-date (lidar connected to T4.0).
 *   - APF does NOT affect the completion check — robot still stops at goal.
 *   - APF does NOT replace the safety service — both can be active together.
 *
 * Examples:
 *   apf(1)             → enable with default scale
 *   apf(1, 80000)      → enable with stronger avoidance
 *   apf(0)             → disable
 */
FLASHMEM void command_apf(const args_t& args) {
    if (args.size() < 1) {
        // Print current state
        Console::info("APF") << (motion.isAPFEnabled() ? "enabled" : "disabled") << Console::endl;
        return;
    }

    const String& s = args[0];
    bool enable = (s == "1" || s.equalsIgnoreCase("on") || s.equalsIgnoreCase("true"));

    float scale = -1.0f;
    if (args.size() >= 2) scale = args[1].toFloat();

    if (enable) {
        motion.enableAPF(scale);
    } else {
        motion.disableAPF();
    }
}

//std::vector<String> arguments = extractArguments(args);

/*
 if(arguments.size() == 1){
            Expression e(arguments[0]);
            Vec2 v = Vec2::fromString(e.evaluate());
            execute_go(v);
        }else if(arguments.size() == 2){
            float x = Expression(arguments[0]).evaluate().toFloat();
            float y = Expression(arguments[1]).evaluate().toFloat();
            execute_go(x, y);
        }

    } else if (command == "move") {
        float x = arguments[0].toFloat();
        float y = arguments[1].toFloat();
        float a = arguments[2].toFloat();

*/
// ─────────────────────────────────────────────────────────────────────────────
//  Mission SD write commands
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void command_mission_sd_open(const args_t& args) {
    if (!SDCard::isReady()) {
        Console::error("Mission") << "SD not ready" << Console::endl;
        return;
    }
    if (MissionController::sdOpen()) {
        Console::info("Mission") << "SD write session opened" << Console::endl;
    } else {
        Console::error("Mission") << "Failed to open mission file for writing" << Console::endl;
    }
}

FLASHMEM void command_mission_sd_line(const args_t& args) {
    // Reconstruct the raw line from args (could contain commas inside strings)
    String line = "";
    for (size_t i = 0; i < args.size(); i++) {
        if (i > 0) line += ",";
        line += args[i];
    }
    // Trim leading/trailing whitespace
    line.trim();
    if (!MissionController::sdAppendLine(line.c_str())) {
        Console::error("Mission") << "Write failed — did you call mission_sd_open first?" << Console::endl;
    }
}

FLASHMEM void command_mission_sd_close(const args_t& args) {
    if (MissionController::sdClose()) {
        Console::info("Mission") << "Strategy loaded from SD successfully" << Console::endl;
    } else {
        Console::error("Mission") << "Close failed or no commands parsed" << Console::endl;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mission execution commands (for terminal testing + fallback registration)
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void command_mission_run(const args_t& args) {
    if (!MissionController::isLoaded()) {
        Console::error("Mission") << "No strategy loaded — use mission_sd_open/line/close first" << Console::endl;
        return;
    }
    if (MissionController::isRunning()) {
        Console::warn("Mission") << "Strategy already running" << Console::endl;
        return;
    }
    Console::info("Mission") << "Starting fallback strategy…" << Console::endl;
    MissionController::execute();   // blocking
}

FLASHMEM void command_mission_abort(const args_t& args) {
    MissionController::abort();
    Console::info("Mission") << "Abort requested" << Console::endl;
}


// ─────────────────────────────────────────────────────────────────────────────
//  Log & telemetry control commands
//  Designed for use on a plain serial terminal (no holOS).
// ─────────────────────────────────────────────────────────────────────────────

// log(SOURCE, 0|1)  — enable/disable a service log source
//   SOURCE : MOTION | SAFETY | CHRONO | INTERCOM | LOCALISATION | … | * (all)
//   1 = enable, 0 = disable
// Example: log(MOTION,0)   → mute Motion logs
//          log(*,1)        → enable all sources
FLASHMEM void command_log(const args_t& args) {
    String src = args[0];
    bool   on  = (args[1].toInt() != 0);

    if (src == "*") {
        if (on) Console::enableAllSources();
        else    Console::setSourceMask(0);
        Console::info("Log") << "All sources " << (on ? "enabled" : "disabled") << Console::endl;
        return;
    }

    ServiceID id = Service::toID(src);
    if (id == ID_NOT_A_SERVICE) {
        Console::warn("Log") << "Unknown source: " << src << Console::endl;
        return;
    }

    if (on) Console::enableSource(id);
    else    Console::disableSource(id);
    Console::info("Log") << src << " " << (on ? "enabled" : "disabled") << Console::endl;
}

// loglevel(LEVEL)  — set global console log level
//   LEVEL : VERBOSE | INFO | WARNING | CRITICAL | DISABLED
// Example: loglevel(VERBOSE)   → show all messages including trace
//          loglevel(DISABLED)  → silence all Console output
FLASHMEM void command_loglevel(const args_t& args) {
    String lvlStr = args[0];
    lvlStr.toUpperCase();

    ConsoleLevel lvl;
    if      (lvlStr == "VERBOSE"  || lvlStr == "TRACE") lvl = ConsoleLevel::VERBOSE;
    else if (lvlStr == "INFO")                           lvl = ConsoleLevel::INFO;
    else if (lvlStr == "SUCCESS")                        lvl = ConsoleLevel::SUCCESS;
    else if (lvlStr == "WARNING"  || lvlStr == "WARN")   lvl = ConsoleLevel::WARNING;
    else if (lvlStr == "CRITICAL" || lvlStr == "ERROR")  lvl = ConsoleLevel::CRITICAL;
    else if (lvlStr == "DISABLED" || lvlStr == "OFF")    lvl = ConsoleLevel::DISABLED;
    else {
        Console::warn("Log") << "Unknown level: " << lvlStr
            << " (VERBOSE/INFO/WARNING/CRITICAL/DISABLED)" << Console::endl;
        return;
    }
    Console::setLevel(lvl);
    Console::info("Log") << "Log level set to " << lvlStr << Console::endl;
}

// tel(CHANNEL, 0|1)  — enable/disable a JetsonBridge telemetry channel
//   CHANNEL : pos | motion | safety | chrono | occ | * (all)
//   1 = enable, 0 = disable
// Example: tel(occ,0)   → stop sending occupancy map frames
//          tel(*,0)     → silence all telemetry (pure terminal debug mode)
FLASHMEM void command_tel(const args_t& args) {
    String ch = args[0];
    bool   on = (args[1].toInt() != 0);
    ch.toLowerCase();

    if      (ch == "pos")    jetsonBridge.setTelemetry(0, on);
    else if (ch == "motion") jetsonBridge.setTelemetry(1, on);
    else if (ch == "safety") jetsonBridge.setTelemetry(2, on);
    else if (ch == "chrono") jetsonBridge.setTelemetry(3, on);
    else if (ch == "occ")    jetsonBridge.setTelemetry(4, on);
    else if (ch == "*") {
        for (int i = 0; i < 5; i++) jetsonBridge.setTelemetry(i, on);
    } else {
        Console::warn("Log") << "Unknown telemetry channel: " << ch
            << " (pos/motion/safety/chrono/occ/*)" << Console::endl;
        return;
    }
    Console::info("Log") << "tel:" << ch << " " << (on ? "on" : "off") << Console::endl;
}

// logstatus  — print current log configuration (useful when connecting via serial)
FLASHMEM void command_logstatus(const args_t& args) {
    Console::line();
    Console::println("[Log status]");

    // Global level
    ConsoleLevel lvl = Console::getLevel();
    const char* lvlName =
        lvl == VERBOSE  ? "VERBOSE"  :
        lvl == INFO     ? "INFO"     :
        lvl == SUCCESS  ? "SUCCESS"  :
        lvl == WARNING  ? "WARNING"  :
        lvl == CRITICAL ? "CRITICAL" : "DISABLED";
    Console::println(String("  Level    : ") + lvlName);

    // Per-source mask
    Console::println("  Sources  :");
    const ServiceID ids[] = {
        ID_LIDAR, ID_CHRONO, ID_IHM, ID_SAFETY, ID_MOTION, ID_NAVIGATION,
        ID_NEOPIXEL, ID_INTERCOM, ID_TERMINAL, ID_ACTUATORS, ID_LOCALISATION,
        ID_VISION, ID_JETSON
    };
    for (auto id : ids) {
        Console::println(String("    ") + Service::toString(id)
            + " : " + (Console::isSourceEnabled(id) ? "ON" : "off"));
    }

    // Telemetry state — delegated to JetsonBridge
    Console::println("  Telemetry: (use logstatus after jetson attach)");
    Console::line();
}

// ── Runtime config commands ──────────────────────────────────────────────────

FLASHMEM void command_cfg_list(const args_t& args) {
    RuntimeConfig::printAll();
}

FLASHMEM void command_cfg_set(const args_t& args) {
    if (args.size() != 2) {
        Console::error("Config") << "Usage: cfg_set(key, value)" << Console::endl;
        return;
    }
    const char* key = args[0].c_str();
    const char* val = args[1].c_str();
    if (RuntimeConfig::set(key, val))
        Console::success("Config") << key << " = " << val << Console::endl;
    else
        Console::error("Config") << "Failed to set " << key << Console::endl;
}

FLASHMEM void command_cfg_save(const args_t& args) {
    RuntimeConfig::save();
}

FLASHMEM void command_cfg_load(const args_t& args) {
    RuntimeConfig::load();
}

// ── Actuator info (structured for holOS UI) ──────────────────────────────────

FLASHMEM void command_act_info(const args_t& args) {
    if (args.size() != 1) {
        Console::error("Interpreter") << "Usage: act_info(AB|CA)" << Console::endl;
        return;
    }
    const String& side = args[0];
    if (!validCompassString(side)) return;
    RobotCompass rc = compassFromString(side);
    ActuatorGroup& group = actuators.getActuatorGroup(rc);

    // Output structured data: one line per servo, pipe-separated for easy parsing.
    // Format: ACT_INFO:<group>|<id>|<min>|<max>|<default>|<position>
    const int servoIds[] = {0, 1, 2, 3, 4};
    for (int id : servoIds) {
        if (group.hasServo(id)) {
            SmartServo& s = group.getServo(id);
            Console::info("ACT_INFO") << side << "|" << id
                << "|" << s.getMinPos()
                << "|" << s.getMaxPos()
                << "|" << s.getDefaultPos()
                << "|" << s.getPosition() << Console::endl;
        }
    }
}

FLASHMEM void command_servo_limits(const args_t& args) {
    // servo_limits(CA, 0, 110, 170)
    if (args.size() != 4) {
        Console::error("Interpreter") << "Usage: servo_limits(side, id, min, max)" << Console::endl;
        return;
    }
    const String& side = args[0];
    if (!validCompassString(side)) return;
    RobotCompass rc = compassFromString(side);
    ActuatorGroup& group = actuators.getActuatorGroup(rc);

    int id     = args[1].toInt();
    int newMin = args[2].toInt();
    int newMax = args[3].toInt();

    if (!group.hasServo(id)) {
        Console::error("Interpreter") << "Servo " << id << " not found in " << side << Console::endl;
        return;
    }
    if (newMin >= newMax) {
        Console::error("Interpreter") << "min must be < max" << Console::endl;
        return;
    }

    SmartServo& s = group.getServo(id);
    s.setMinPos(newMin);
    s.setMaxPos(newMax);

    // Also update runtime config so cfg_save persists the change
    char keyMin[32], keyMax[32];
    snprintf(keyMin, sizeof(keyMin), "servo.%s.%d.min", side.c_str(), id);
    snprintf(keyMax, sizeof(keyMax), "servo.%s.%d.max", side.c_str(), id);
    RuntimeConfig::setInt(keyMin, newMin);
    RuntimeConfig::setInt(keyMax, newMax);

    Console::success("Interpreter") << "Servo " << side << "." << id
        << " limits: [" << newMin << ", " << newMax << "]" << Console::endl;
}
