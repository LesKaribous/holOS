#include "localisation.h"
#include "os/console.h"
#include "services/jetson/jetson_bridge.h"
#include "services/motion/motion.h"

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

FLASHMEM bool Localisation::calibrate() {
    if (!m_connected) {
        Console::warn("Localisation")
            << "calibrate() skipped — OTOS not connected" << Console::endl;
        m_calibrated = false;
        return false;
    }

    // SparkFun OTOS firmware accepts up to 255 samples per calibrateImu()
    // call (numSamples is uint8_t — register is one byte). The previous
    // value 400 silently overflowed to 144 (400 % 256), giving a
    // half-strength calibration. 255 = full strength = ~612 ms blocking.
    constexpr uint8_t  SAMPLES   = 255;
    constexpr int      MAX_TRIES = 3;

    for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
        Console::info("Localisation")
            << "IMU calibration (" << (int)SAMPLES
            << " samples, try " << attempt << "/" << MAX_TRIES << ")..."
            << Console::endl;

        // calibrateImu(true) blocks polling kRegImuCalib until it reaches
        // 0 (= done) or the polling loop hits its numSamples attempts
        // ceiling. The "early exit" failure mode the field reports is
        // exactly that ceiling — comm latency or a slow OTOS boot can
        // push the register past the polling window.
        sfTkError_t err = otos.calibrateImu(SAMPLES, true);

        // Belt-and-suspenders verification: re-read the progress register.
        // After a successful calibrateImu(true), it MUST read 0. If the
        // lib returned Ok but the register still has remaining samples,
        // we don't trust it (treat as early exit).
        uint8_t remaining = 0xFF;
        otos.getImuCalibrationProgress(remaining);

        if (err == ksfTkErrOk && remaining == 0) {
            otos.setLinearScalar(m_scale);
            m_calibrated = true;
            Console::success("Localisation")
                << "IMU calibration OK (try " << attempt
                << ", linear scale " << m_scale << ")" << Console::endl;
            return true;
        }

        Console::warn("Localisation")
            << "IMU calibration early-exit — err=" << (int)err
            << " remaining=" << (int)remaining
            << " — retrying after 200 ms" << Console::endl;
        delay(200);
    }

    Console::error("Localisation")
        << "IMU calibration FAILED after " << MAX_TRIES
        << " attempts — running with stale offsets, drift expected"
        << Console::endl;
    m_calibrated = false;
    return false;
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

// Push a "freeze the homography on the next frame" request to holOS. Async —
// the response comes back via onHomographyLockReply(). Call this at the
// start of the recalage routine, while the static anchor tags are still
// visible to the camera (before the robot moves into the field of view).
FLASHMEM void Localisation::requestHomographyCapture() {
    m_homographyLocked = false;   // set true by reply handler
    jetsonBridge.pushVisionFrame("T:vis homography_capture");
    Console::info("Localisation")
        << "Homography capture requested" << Console::endl;
}

// Push a "calibrate me at this known position" frame to holOS. Async —
// the response comes back asynchronously via onVisionCalibrationReply().
//
// IMPORTANT — built with Arduino String, NOT snprintf. On this build
// (newlib-nano without -u _printf_float) snprintf silently breaks
// when given %f, returning -1 / leaving the buffer uninitialised,
// which then corrupts _pushFrame's outer snprintf and drops the
// frame entirely. String += handles ints directly and never reaches
// libc's vsnprintf, so it's bulletproof regardless of build flags.
//
// Wire format (integer-only, same convention as T:a):
//   x in mm (int), y in mm (int), theta in milliradians (int).
// holOS divides t by 1000 to recover radians.
FLASHMEM void Localisation::requestVisionCalibration(Vec3 known_pos) {
    // Build the cal_request frame with snprintf %d (newlib-nano-safe —
    // only %f is unsupported in this build). Convention:
    //   x in mm (int), y in mm (int), theta in milliradians (int).
    //
    // Use pushVisionFrameDirect to bypass the bridge's ring buffer.
    // Earlier diagnostic showed our 37-char cal_request frame would
    // sit in the ring buffer for >2 s on the XBee link with no T:vis
    // emission, while the shorter literal markers around it both
    // got out fine. Direct write spins on TX FIFO + flushes — costs
    // ~10 ms on a busy bridge, fully acceptable for a one-shot
    // recalage handshake.
    char frame[80];
    int n = snprintf(frame, sizeof(frame),
                     "T:vis cal_request x=%d y=%d t=%d",
                     (int)known_pos.x,
                     (int)known_pos.y,
                     (int)(known_pos.z * 1000.0f));
    if (n <= 0 || n >= (int)sizeof(frame)) {
        jetsonBridge.pushVisionFrameDirect("T:vis cal_format_failed");
        return;
    }
    jetsonBridge.pushVisionFrameDirect(frame);
    Console::info("Localisation")
        << "Vision recalage requested at (" << known_pos.x << ", "
        << known_pos.y << ", " << known_pos.z << ")" << Console::endl;
    m_visionCalibrated = false;   // set true by reply handler
}

// Manual / human-in-the-loop variant of cal_request. holOS opens a
// modal on the webapp telling the operator to push the robot to the
// supplied target pose, then captures (tag_xy, target_xy) once the
// operator clicks OK. The caller must have disengaged the steppers
// before calling, and re-engage after isVisionCalibrated() flips true
// (or after the firmware-side wait times out).
FLASHMEM void Localisation::requestVisionCalibrationManual(Vec3 target_pos) {
    char frame[80];
    int n = snprintf(frame, sizeof(frame),
                     "T:vis cal_request_manual x=%d y=%d t=%d",
                     (int)target_pos.x,
                     (int)target_pos.y,
                     (int)(target_pos.z * 1000.0f));
    if (n <= 0 || n >= (int)sizeof(frame)) {
        jetsonBridge.pushVisionFrameDirect("T:vis cal_format_failed");
        return;
    }
    jetsonBridge.pushVisionFrameDirect(frame);
    Console::info("Localisation")
        << "Vision recalage (manual) requested — push robot to ("
        << target_pos.x << ", " << target_pos.y << ", "
        << target_pos.z << ")" << Console::endl;
    m_visionCalibrated = false;   // set true by reply handler when user OKs
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
            << "queryVisionPose timeout after " << long(timeout_ms) << "ms"
            << Console::endl;
        return false;
    }
    if (!m_lastPoseValid) {
        return false;
    }
    out_pose = m_lastVisionPose;
    return true;
}

// Convenience: query then push the vision fix into Motion + OTOS.
// Goes through motion.setAbsPosition() (not just localisation.setPosition)
// so the motion module's cached _position / _startPosition stay in sync —
// otherwise the next pursuit/cruise cycle would still reference the stale
// OTOS-only pose.
FLASHMEM Vec3 Localisation::syncToVision(unsigned long timeout_ms) {
    Vec3 vp;
    if (!queryVisionPose(vp, timeout_ms)) {
        return Vec3{0, 0, 0};
    }
    Vec3 before = getPosition();
    motion.setAbsPosition(vp);   // updates Motion::_position AND OTOS
    Vec3 offset = {vp.x - before.x, vp.y - before.y, vp.z - before.z};
    Console::success("Localisation")
        << "OTOS recalibrated to vision: ("
        << vp.x << ", " << vp.y << ", " << vp.z
        << ")  Δ=(" << offset.x << ", " << offset.y << ", " << offset.z
        << ")" << Console::endl;
    return offset;
}

// Reply handlers — called by JetsonBridge after parsing an inbound frame.
FLASHMEM void Localisation::onHomographyLockReply(bool ok) {
    m_homographyLocked = ok;
    if (ok) {
        Console::success("Localisation")
            << "Homography locked by holOS" << Console::endl;
    } else {
        Console::warn("Localisation")
            << "Homography lock refused (no H ready or no rectify node)"
            << Console::endl;
    }
}

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
