#pragma once
#include "utils/geometry.h"
#include <Adafruit_PWMServoDriver.h>

// Permet la selection du POI selon l'équipe
template<typename T>
inline const T& choose(bool cond, const T& a, const T& b) {
    return cond ? a : b;
}

#define RIGHT true
#define LEFT false


void match();
void registerBlocks();   // Register all C++ blocks into BlockRegistry (called at boot)

// Macros
void waitMs(unsigned long time);
void nearEnd();

// Grab-only sequence (vision-correction + grab choreography). The
// caller must already be parked at the approach pose for the given
// target. Used by collectStock (full collect) and autoGrab (bench test).
// Vec2 / TableCompass / RobotCompass come from utils/geometry.h above.
void grabStockHere(Vec2 target, TableCompass tc, RobotCompass rc);

// Bench-test command target: assumes the robot is currently at the
// approach pose, picks tc/rc from team color, calls grabStockHere.
void autoGrab();

//------------------------------------------------------
// TODO : Integrate Pump and EV into Actuators <3
void initPump();
void startPump(RobotCompass rc, bool side);
void stopPump(RobotCompass rc, uint16_t evPulseDuration, bool side);
extern Adafruit_PWMServoDriver pwm;
//------------------------------------------------------
