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
    /// Run the OTOS IMU calibration over the full 255-sample window
    /// (hardware max — the register is uint8_t). Verifies the OTOS
    /// progress register reaches 0 and retries up to 3 times if the
    /// firmware-side polling loop early-exits before convergence.
    /// Returns true iff calibration completed and m_calibrated is set.
    bool calibrate();
    void setLinearScale(float value);
    void setAngularScale(float value);

    float getLinearScale()  const { return m_scale; }
    float getAngularScale() const { return m_angular_scale; }

    inline bool useIMU() const {return m_use_IMU && m_connected & m_calibrated;}

    // ── Vision recalage / sync ──────────────────────────────────────
    //
    // requestHomographyCapture()
    //   Tell holOS to capture and freeze the table↔camera homography
    //   from the next available frame. Sent at the start of recalage()
    //   while the static anchor tags are still fully visible. Once
    //   locked, holOS stops re-fitting the homography even if anchors
    //   are later occluded by the robot or operator. Asynchronous —
    //   completion polled via isHomographyLocked().
    //
    // requestVisionCalibration(known_pos)
    //   Send the robot's known starting pose to holOS at the end of
    //   the recalage procedure. holOS picks the ArUco closest to that
    //   position, locks it as the OWN robot tag, infers team color
    //   from the tag id, and captures the heading offset between the
    //   robot frame and the (arbitrarily mounted) tag frame so all
    //   subsequent vision poses are returned in the robot frame.
    //   Asynchronous: completion is signalled via isVisionCalibrated()
    //   flipping to true (or staying false on timeout / no candidate).
    //
    // queryVisionPose(out_pose, timeout_ms)
    //   Blocking-with-timeout: ask holOS for the current vision pose of
    //   the locked OWN tag.  Returns true on fresh data (out_pose set
    //   to {x_mm, y_mm, theta_rad}), false on timeout / no fix / not
    //   calibrated.  Call ONLY when the robot is stationary — vision
    //   has its own latency (~50-100 ms) so any motion makes the result
    //   stale relative to OTOS.
    //
    // syncToVision(timeout_ms)
    //   Convenience: queryVisionPose() then setPosition() to overwrite
    //   OTOS with the vision fix.  Returns the offset (vision − otos)
    //   for logging.  Vec3{0,0,0} on failure.
    void requestHomographyCapture();
    bool isHomographyLocked() const { return m_homographyLocked; }
    void requestVisionCalibration(Vec3 known_pos);
    // Manual variant: target_pos is the *requested* pose. holOS shows a
    // modal asking the operator to push the robot to that exact pose,
    // then captures (tag_xy, target_xy) as a parallax-calibration pair.
    // Use this in vision_recalage where OTOS imprecision would otherwise
    // pollute the parallax fit.
    void requestVisionCalibrationManual(Vec3 target_pos);
    bool isVisionCalibrated() const { return m_visionCalibrated; }
    // True once a vision_cal reply has landed (success OR failure).
    // Lets the visionRecalage() poll loop break out of its 125 s
    // wait the moment holOS sends vis_cal_failed, instead of
    // burning the full timeout per failed point.
    bool visionCalibrationReplyReceived() const { return m_visionCalibrationReplyReceived; }
    bool queryVisionPose(Vec3& out_pose, unsigned long timeout_ms = 300);
    Vec3 syncToVision(unsigned long timeout_ms = 500);

    // Called by JetsonBridge when a vision reply arrives from holOS.
    // Not called by user code.
    void onHomographyLockReply(bool ok);
    void onVisionCalibrationReply(int own_tag, const char* team,
                                  Vec3 vision_pos);
    void onVisionPoseReply(Vec3 pos, bool valid);

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

    // ── Vision sync state ─────────────────────────────────────────
    // Set by onHomographyLockReply() once holOS has frozen the H.
    // Survives the whole match — the only legitimate way to clear it
    // is to request `homography_release` (not done in normal flow).
    bool   m_homographyLocked = false;

    // Set by onVisionCalibrationReply() once holOS has identified the
    // OWN tag at our reported starting position.
    bool   m_visionCalibrated = false;
    int    m_visionOwnTag     = -1;
    char   m_visionTeam[8]    = {0};
    // Cleared by request*(), set true on ANY reply (success or fail).
    bool   m_visionCalibrationReplyReceived = false;

    // queryVisionPose() is request/reply across XBee; we wait for the
    // reply by polling these flags (filled by onVisionPoseReply()).
    volatile bool m_pendingPoseReply = false;
    volatile bool m_lastPoseValid    = false;
    Vec3          m_lastVisionPose   = {0,0,0};
};
SINGLETON_EXTERN(Localisation, localisation)