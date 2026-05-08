#include "localisation.h"
#include "os/console.h"
#include "services/jetson/jetson_bridge.h"

#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>   // millis()

SINGLETON_INSTANTIATE(Localisation, localisation);

#ifdef OLD_BOARD
#define I2C_OTOS I2C_OTOS
#else
#define I2C_OTOS Wire
#endif

FLASHMEM void Localisation::attach(){
    
    Console::info() << "Localisation activated" << Console::endl;
    I2C_OTOS.begin();

    m_connected = false;
    for(int i = 0; i < 10; i++){
        if(otos.begin(I2C_OTOS) == true){
            m_connected = true;           
            break;
        }
    }

    if(!m_connected){
        Console::error("Localisation") << "Ooops, no OTOS detected ... It may be unplugged. Make sure you use I2C_OTOS" << Console::endl;
    }
    else{
        otos.setLinearUnit(kSfeOtosLinearUnitMeters);
        otos.setAngularUnit(kSfeOtosAngularUnitRadians);
        otos.resetTracking();
        Console::success("Localisation") << "OTOS connected" << Console::endl;
    }
}

// Main loop
FLASHMEM void Localisation::run(){ 
    //THROW(1)

    static long elapsed = 0;
    if(millis() - elapsed > m_refresh){
        elapsed = millis();
        read();
    }
}

// Service routines
FLASHMEM void Localisation::enable(){
    if(!m_connected) return;
    Service::enable();
    //servicethread = new std::thread(&Localisation::runThread, this);
 	//servicethread->detach();
    m_use_IMU = true;
}

FLASHMEM void Localisation::disable(){
    Service::disable();
    //delete servicethread;
    m_use_IMU = false;
}

FLASHMEM void Localisation::setPosition(Vec3 newPos){
    sfe_otos_pose2d_t pos;
    pos.x = -newPos.y/1000.0;
    pos.y = -newPos.x/1000.0;
    pos.h = newPos.z;
    otos.resetTracking(); // WIP
    otos.setPosition(pos);
    _unsafePosition = newPos;
}

Vec3 Localisation::getPosition(){
    run();
    return _unsafePosition;
}

Vec3 Localisation::getVelocity(){
    run();
    return _unsafeVelocity;
}

FLASHMEM void Localisation::read()
{
    sfe_otos_pose2d_t myPosition;
    sfe_otos_pose2d_t myVelocity;

    otos.getVelocity(myVelocity);
    otos.getPosition(myPosition);

    //Console::info() << "x:" << myPosition.x << "y:" << myPosition.y << "h:" << myPosition.h << Console::endl;

    _unsafePosition.x = -myPosition.y * 1000.0; //to millimeters
    _unsafePosition.y = -myPosition.x * 1000.0; //to millimeters
    
    _unsafeVelocity.x = -myVelocity.y * 1000.0; //to millimeters /s
    _unsafeVelocity.y = -myVelocity.x * 1000.0; //to millimeters /s
    _unsafeVelocity.z =  myVelocity.h; //to radians /s

    float orientation = myPosition.h;
    _unsafePosition.z = orientation;
    //Console::info() << _unsafePosition << Console::endl;
}

FLASHMEM void Localisation::calibrate() {
    Console::println("Ensure the OTOS is flat and stationary");
    delay(2000);
    Console::info("Localisation") << "Calibrating IMU...";
    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    otos.calibrateImu(400, true);
    //otos.setLinearScalar(1.05f);//maison
    otos.setLinearScalar(m_scale);//coupe
    Console::println("done.");
    m_calibrated = true;
}

FLASHMEM void Localisation::setLinearScale(float value){
    m_scale = value;
    Console::info("Localisation") << "Linear scale → ";
    Serial.println(m_scale, 6);
    otos.setLinearScalar(m_scale);
}

FLASHMEM void Localisation::setAngularScale(float value){
    m_angular_scale = value;
    Console::info("Localisation") << "Angular scale → ";
    Serial.println(m_angular_scale, 6);
    otos.setAngularScalar(m_angular_scale);
}

// ─────────────────────────────────────────────────────────────────────────
//  Vision recalage / sync
// ─────────────────────────────────────────────────────────────────────────

// Push a "calibrate me at this known position" frame to holOS. Async —
// the response comes back asynchronously via onVisionCalibrationReply().
FLASHMEM void Localisation::requestVisionCalibration(Vec3 known_pos) {
    char frame[80];
    // Format: T:vis cal_request x=600.0 y=500.0 t=0.000
    // holOS parses this in run.py and replies with:
    //   T:vis cal_done own=1 team=blue x=623.4 y=498.1 t=0.012
    // (or cal_failed reason=...)
    snprintf(frame, sizeof(frame),
             "T:vis cal_request x=%.1f y=%.1f t=%.3f",
             (double)known_pos.x, (double)known_pos.y, (double)known_pos.z);
    jetsonBridge.pushVisionFrame(frame);
    Console::info("Localisation")
        << "Vision recalage requested at (" << known_pos.x << ", "
        << known_pos.y << ", " << known_pos.z << ")" << Console::endl;
    m_visionCalibrated = false;   // set true by reply handler
}

// Block (with timeout) waiting for the next pose reply from holOS.
FLASHMEM bool Localisation::queryVisionPose(Vec3& out_pose,
                                            unsigned long timeout_ms) {
    if (!m_visionCalibrated) {
        Console::warn("Localisation")
            << "queryVisionPose() called before recalage — no own tag locked"
            << Console::endl;
        return false;
    }
    // Mark a pending request so onVisionPoseReply can flip the flag back.
    m_pendingPoseReply = true;
    m_lastPoseValid    = false;
    // Send the request frame
    jetsonBridge.pushVisionFrame("T:vis pose_request");
    // Spin-wait for the reply (or timeout). millis() is monotonic.
    unsigned long t0 = millis();
    while (m_pendingPoseReply && (millis() - t0) < timeout_ms) {
        // Let other services run — JetsonBridge::run() drains the bridge
        // serial buffer and dispatches replies.
        jetsonBridge.run();
        delay(2);
    }
    if (m_pendingPoseReply) {
        // Timed out
        m_pendingPoseReply = false;
        Console::warn("Localisation")
            << "queryVisionPose timeout after " << timeout_ms << "ms"
            << Console::endl;
        return false;
    }
    if (!m_lastPoseValid) {
        return false;
    }
    out_pose = m_lastVisionPose;
    return true;
}

// Convenience: query then setPosition() to overwrite OTOS.
FLASHMEM Vec3 Localisation::syncToVision(unsigned long timeout_ms) {
    Vec3 vp;
    if (!queryVisionPose(vp, timeout_ms)) {
        return Vec3{0, 0, 0};
    }
    Vec3 before = getPosition();
    setPosition(vp);
    Vec3 offset = {vp.x - before.x, vp.y - before.y, vp.z - before.z};
    Console::success("Localisation")
        << "OTOS recalibrated to vision: ("
        << vp.x << ", " << vp.y << ", " << vp.z
        << ")  Δ=(" << offset.x << ", " << offset.y << ", " << offset.z
        << ")" << Console::endl;
    return offset;
}

// Reply handlers — called by JetsonBridge after parsing an inbound frame.
FLASHMEM void Localisation::onVisionCalibrationReply(int own_tag,
                                                     const char* team,
                                                     Vec3 vision_pos) {
    m_visionOwnTag = own_tag;
    if (team && *team) {
        strncpy(m_visionTeam, team, sizeof(m_visionTeam) - 1);
        m_visionTeam[sizeof(m_visionTeam) - 1] = 0;
    }
    m_visionCalibrated = (own_tag > 0);
    if (m_visionCalibrated) {
        Console::success("Localisation")
            << "Vision recalage OK — own tag #" << own_tag
            << " (team " << m_visionTeam << ") at ("
            << vision_pos.x << ", " << vision_pos.y << ", "
            << vision_pos.z << ")" << Console::endl;
    } else {
        Console::error("Localisation")
            << "Vision recalage FAILED — no candidate tag found"
            << Console::endl;
    }
}

FLASHMEM void Localisation::onVisionPoseReply(Vec3 pos, bool valid) {
    m_lastVisionPose   = pos;
    m_lastPoseValid    = valid;
    m_pendingPoseReply = false;   // unblock any spinning queryVisionPose
}
