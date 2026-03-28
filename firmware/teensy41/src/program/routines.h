#pragma once
#include "os/console.h"
#include "services/intercom/intercom.h"

// ============================================================
//  routines.h — Program entry points for the holOS on Teensy 4.1
//
//  Architecture headless + remote controlled (Jetson) :
//
//    programAuto()  — En mode REMOTE : attend les commandes Jetson.
//                     En mode FALLBACK : exécute match() embarqué.
//    programManual() — Préparation (config équipe, recalage…).
//                      Écoute aussi les commandes Jetson entrantes.
//
//    La décision remote vs fallback est gérée par JetsonBridge.
// ============================================================

// Programs
void programManual();   // Execute infinitely before match
void programAuto();     // Execute during match

// Event routines
void onRobotBoot();     // Execute once at boot
void onRobotManual();   // Execute before program (idle loop)
void onRobotAuto();     // Execute during program (run loop)
void onRobotStop();     // Execute while robot stopped

// Terminal
void onTerminalCommand();

// Intercom / Jetson
void onIntercomConnected();
void onIntercomDisconnected();
void onIntercomRequest(Request&);
void onIntercomRequestReply(Request&);

// Match events
void onMatchNearEnd();
void onMatchEnd();

// Control ISR (registered with CycleManager)
void step();     // 10µs — stepper pulse generation
void control();  // 1ms  — PID control loop

// Utilities
void robotArmed();
