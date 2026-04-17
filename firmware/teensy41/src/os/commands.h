#pragma once
#include "utils/geometry.h"
#include "utils/commandHandler.h"
#include <Arduino.h>

void registerCommands();

void command_stats(const args_t& args);
void command_enable(const args_t& args);
void command_disable(const args_t& args);
void command_status(const args_t& args); //Display all status
void command_debug(const args_t& args); //Display all status
void command_lidarMode(const args_t& args);
void command_wait(const args_t& args);
void command_start(const args_t& args); //Start match
void command_stop(const args_t& args); //Start match
void command_reboot(const args_t& args); //Start match
void command_probe(const args_t& args); //Start match
void command_cruise(const args_t& args); //Start match
void command_feed(const args_t& args); //Start match
void command_music(const args_t& args); //Start match
void command_otos_calibration(const args_t& args); //Start match
void command_otos_scale(const args_t& args); //Start match
void command_radar(const args_t& args); //Start match
void command_test(const args_t& args); //Start match

//Motion
void command_go(const args_t& args);
void command_go_coc(const args_t& args);
void command_goAlign(const args_t& args);
void command_via(const args_t& args);
void command_goPolar(const args_t& args);
void command_move(const args_t& args);
void command_turn(const args_t& args);
void command_rawTurn(const args_t& args); //turn without optimization (without modulo)
void command_pause(const args_t& args);
void command_resume(const args_t& args);
void command_cancel(const args_t& args);
//void command_sleep(const args_t& args);
//void command_wake(const args_t& args);
void command_align(const args_t& args);
void command_setAbsolute(const args_t& args);
void command_setRelative(const args_t& args);
void command_setAbsPosition(const args_t& args);
void command_resetCompass(const args_t& args);
void command_collision_detect(const args_t& args);

// Motion — pursuit / live-target mode
void command_motion_mode(const args_t& args);    // motion_mode(0|1)
void command_aim(const args_t& args);            // aim(x,y)
void command_aim_heading(const args_t& args);    // aim_heading(face|off)

//Actuators
void command_raise(const args_t& args);
void command_lower(const args_t& args);
void command_grab(const args_t& args);
void command_drop(const args_t& args);
void command_store(const args_t& args);
void command_pump(const args_t& args);
void command_ev(const args_t& args);
void command_initPump(const args_t& args);
//void command_close(const args_t& args);
void command_elevator(const args_t& args);
void command_move_elevator(const args_t& args);

void command_servo(const args_t& args);
void command_printServo(const args_t& args);

//Routine
void command_recalage(const args_t& args);

//Terminal
void command_help(const args_t& args);
void command_print(const args_t& args);

void command_wake(const args_t& args);
void command_sleep(const args_t& args);

// Diagnostics
void command_health(const args_t& args);

// Calibration
void command_calib_status(const args_t& args);
void command_calib_save(const args_t& args);
void command_calib_load(const args_t& args);
void command_calib_reset(const args_t& args);
void command_calib_cart(const args_t& args);
void command_calib_holo(const args_t& args);
void command_calib_otos_linear(const args_t& args);
void command_calib_otos_angular(const args_t& args);
void command_calib_move_open(const args_t& args);
void command_calib_turn_open(const args_t& args);

// Probe: wall probing for position recalibration (recalage)
void command_probe_open(const args_t& args);

// Last calibration report payload (populated by calib_move_open / calib_turn_open / probe_open).
// Format: "kind=move cmd=... axis=... dx=... dy=... dth=... od=..."
//      or "kind=turn cmd_deg=... dth_rad=... od_deg=..."
//      or "kind=probe wall=... face=... x=... y=... theta=..."
//      or "kind=error msg=..."
// Empty string if no calibration command has run yet.
const char* getLastCalibReport();

// Motion — rotation around arbitrary point
void command_goAround(const args_t& args);

// Motion — APF obstacle avoidance
void command_apf(const args_t& args);

// Mission fallback write (in-memory, SD removed)
void command_mission_sd_open(const args_t& args);
void command_mission_sd_line(const args_t& args);
void command_mission_sd_close(const args_t& args);

// Mission — execute fallback
void command_mission_run(const args_t& args);
void command_mission_abort(const args_t& args);

// Diagnostics — log / telemetry control
// log(SOURCE, 0|1)     — disable/enable a service's console output
//                        SOURCE = MOTION|SAFETY|CHRONO|INTERCOM|LOCALISATION … or * for all
// loglevel(LEVEL)      — set global log level: VERBOSE|INFO|WARNING|CRITICAL|DISABLED
// tel(CHANNEL, 0|1)    — disable/enable a telemetry channel
//                        CHANNEL = pos|motion|safety|chrono|occ
// logstatus            — print current level + source mask + telemetry state
void command_log(const args_t& args);
void command_loglevel(const args_t& args);
void command_tel(const args_t& args);
void command_logstatus(const args_t& args);

// Runtime config (in-memory, managed by holOS)
void command_cfg_list(const args_t& args);
void command_cfg_set(const args_t& args);

// Actuator info (structured for UI) & runtime limit adjustment
void command_act_info(const args_t& args);
void command_servo_limits(const args_t& args);