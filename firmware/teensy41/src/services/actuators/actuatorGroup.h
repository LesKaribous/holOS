#pragma once
#include "smartServo.h"
#include "sensor.h"
#include <vector>

// Renamed from MAX_SERVOS to avoid macro collision with Servo.h from the
// Teensy Arduino framework which defines its own MAX_SERVOS.
#define MAX_SERVOS_PER_GROUP 12

class ActuatorGroup{
private:
    std::unordered_map<int, SmartServo> m_servos;

public:
    ActuatorGroup();  

    void enable();
    void disable();
    void sleep();
    void listServo();
    void createServo(int id, int pin, int defaultPos, int minPos = 0, int maxPos = 180);
    SmartServo& getServo(int id);
    bool hasServo(int id);
    void moveServoToPose(int servo, int pose, int speed);
};
