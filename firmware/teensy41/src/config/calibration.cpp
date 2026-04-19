#include "calibration.h"
#include "services/localisation/localisation.h"
#include <stdio.h>
#include <string.h>

// ── Live mutable calibration state ──────────────────────────────────────────

namespace Calibration {


    
// Live state initialisé depuis Settings::Calibration (source unique).
// Toute modification runtime passe par setCartesian/setHolonomic/setOtos*.
CalibrationProfile Current     = Settings::Calibration::DEFAULTS;
float              OtosLinear  = Settings::Calibration::OTOS_LINEAR_DEFAULT;
float              OtosAngular = Settings::Calibration::OTOS_ANGULAR_DEFAULT;

// ── Mutators ─────────────────────────────────────────────────────────────────

void reset() {
    Current     = Settings::Calibration::DEFAULTS;
    OtosLinear  = Settings::Calibration::OTOS_LINEAR_DEFAULT;
    OtosAngular = Settings::Calibration::OTOS_ANGULAR_DEFAULT;
    localisation.setLinearScale(OtosLinear);
    localisation.setAngularScale(OtosAngular);
}

void setCartesian(float x, float y, float rot) {
    Current.Cartesian = { x, y, rot };
}

void setHolonomic(float a, float b, float c) {
    Current.Holonomic = { a, b, c };
}

void setOtosLinear(float value) {
    OtosLinear = value;
    localisation.setLinearScale(value);
}

void setOtosAngular(float value) {
    OtosAngular = value;
    localisation.setAngularScale(value);
}

// ── Serialization ─────────────────────────────────────────────────────────────

int toString(char* buf, size_t bufSize) {
    return snprintf(buf, bufSize,
        "cx=%.4f,cy=%.4f,cr=%.4f,"
        "ha=%.4f,hb=%.4f,hc=%.4f,"
        "ol=%.6f,oa=%.6f",
        Current.Cartesian.x,   Current.Cartesian.y,   Current.Cartesian.z,
        Current.Holonomic.x,   Current.Holonomic.y,   Current.Holonomic.z,
        OtosLinear, OtosAngular
    );
}

bool fromString(const char* str) {
    bool applied = false;
    // Parse comma-separated "key=value" pairs
    char buf[256];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* token = strtok(buf, ",");
    while (token) {
        char key[16] = {};
        float val = 0.0f;
        if (sscanf(token, "%15[^=]=%f", key, &val) == 2) {
            if      (strcmp(key, "cx") == 0) { Current.Cartesian.x = val; applied = true; }
            else if (strcmp(key, "cy") == 0) { Current.Cartesian.y = val; applied = true; }
            else if (strcmp(key, "cr") == 0) { Current.Cartesian.z = val; applied = true; }
            else if (strcmp(key, "ha") == 0) { Current.Holonomic.x = val; applied = true; }
            else if (strcmp(key, "hb") == 0) { Current.Holonomic.y = val; applied = true; }
            else if (strcmp(key, "hc") == 0) { Current.Holonomic.z = val; applied = true; }
            else if (strcmp(key, "ol") == 0) { setOtosLinear(val);         applied = true; }
            else if (strcmp(key, "oa") == 0) { setOtosAngular(val);        applied = true; }
        }
        token = strtok(nullptr, ",");
    }
    return applied;
}

}  // namespace Calibration
