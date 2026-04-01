#include "kinematics.h"
#include "config/settings.h"
#include "config/calibration.h"   // Runtime-mutable calibration profile

// ── Inverse kinematics (Cartesian/world → stepper steps) ─────────────────────
// Uses Calibration::Current (runtime-mutable) so scale factors can be updated
// live via calibration commands or loaded from SD card.
Vec3 ik(Vec3 target){
    float fsq3s2 = sqrt(3.0f)/2.0,
            L = Settings::Geometry::RADIUS;

    Matrix3x3 P = {
        0,      1,  L,
     -fsq3s2, -0.5, L,
      fsq3s2, -0.5, L
    };

    target *= Calibration::Current.Cartesian;
    target.mult(P);
    target *= Calibration::Current.Holonomic * Settings::Stepper::STEP_MODE;
    return target;
}

// ── Forward kinematics (stepper steps → Cartesian/world) ─────────────────────
Vec3 fk(Vec3 target){
    float   fsq3s3 = sqrt(3.0f)/3.0,
            f1s3 = 1.0f/3.0f,
            f2s3 = 2.0f/3.0f,
            L = Settings::Geometry::RADIUS;

    Matrix3x3 P = {
            0, -fsq3s3, fsq3s3,
        f2s3,   -f1s3 , -f1s3,
        f1s3/L, f1s3/L, f1s3/L
    };

    target /= (Calibration::Current.Holonomic * Settings::Stepper::STEP_MODE);
    target.mult(P);
    target /= Calibration::Current.Cartesian;
    return target;
}