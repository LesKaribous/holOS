#pragma once
#include <Servo.h>
#include <unordered_map>

#define MAX_POSES 10

class SmartServo{
public:
    SmartServo();
    SmartServo(int pin, int defaultPos = 90, int minPos = 0, int maxPos = 180);
    SmartServo(const SmartServo& cpy);

    bool moveToDefault(int speed = 100, bool runAsync = false);
    bool moveTo(int pos, int speed = 100, bool runAsync = false);
    bool moveToPose(int index, int speed = 100, bool runAsync = false);

    int getPosition();

    void setPose(int index, int pose);
    int getPose(int i) const;
    bool hasPose(int i) const;

    void enable();
    void disable();
    void sleep();
    void wakeUp();
    
    inline bool isSleeping() const {return m_sleeping;}
    inline bool isEnable() const {return m_enabled;}
//private:
public:
    Servo m_servo;
    long m_lastUpdate = 0;
    int m_speed;
    int m_position;
    int m_target;
    std::unordered_map<int, int> m_poses;
    
    bool m_enabled, m_sleeping;
    const int m_pin;
    int m_minPos, m_maxPos, m_defaultPos;

    // Runtime limit setters (for SD config system)
    void setMinPos(int v) { m_minPos = v; }
    void setMaxPos(int v) { m_maxPos = v; }
    int getMinPos() const { return m_minPos; }
    int getMaxPos() const { return m_maxPos; }
    int getDefaultPos() const { return m_defaultPos; }
};