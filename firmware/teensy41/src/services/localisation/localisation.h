#pragma once
#include "services/service.h"
#include "utils/geometry.h"
#include "config/settings.h"   // Settings::Calibration::OTOS_*_DEFAULT
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

class Localisation : public Service{
public:
   
    void attach() override;
    void run() override;
    
    void enable()override;
    void disable() override;
    //void onUpdateThread(void* arg) override;
    
    void setPosition(Vec3);
    Vec3 getPosition();
    Vec3 getVelocity();
    void read();
    void calibrate();
    void setLinearScale(float value);
    void setAngularScale(float value);

    float getLinearScale()  const { return m_scale; }
    float getAngularScale() const { return m_angular_scale; }

    inline bool useIMU() const {return m_use_IMU && m_connected & m_calibrated;}
    
    Localisation(): Service(ID_LOCALISATION){};
    SINGLETON(Localisation)

private :
    // Scalars OTOS : defaults depuis Settings::Calibration (source unique).
    // Valeurs écrasées au boot par Calibration::reset() qui synchronise aussi
    // l'état interne de l'OTOS hardware via setLinearScale / setAngularScale.
    float m_scale         = Settings::Calibration::OTOS_LINEAR_DEFAULT;
    float m_angular_scale = Settings::Calibration::OTOS_ANGULAR_DEFAULT;
    bool m_use_IMU = false;
    bool m_connected = false;
    bool m_calibrated = false;
    bool m_isMoving = false;
    bool m_isRotating = false;
    long m_refresh = 10; //ms  previous 300

    Vec3 _unsafePosition = {0,0,0};
    Vec3 _unsafeVelocity = {0,0,0};

    QwiicOTOS otos;
};
SINGLETON_EXTERN(Localisation, localisation)