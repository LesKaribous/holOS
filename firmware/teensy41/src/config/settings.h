#pragma once
#include <Arduino.h>
#include "pin.h"
#include "utils/geometry.h"
#include "os/debug/consoleLevel.h"

// ── T4.1 ↔ T4.0 Intercom — TOUJOURS sur Serial1 @ 31250 ─────────────────────
// Ce canal gère le ping/pong et les échanges directs avec T4.0.
// Ne jamais rediriger sur USB : cela coupe la connexion T4.0.
#define INTERCOM_SERIAL   Serial1   // UART physique T4.1↔T4.0
#define INTERCOM_BAUDRATE 31250

#define OTOS_AUX_SERIAL Serial8

// ── Bridge holOS / Jetson — auto-détection USB ↔ XBee ────────────────────────
// Au boot, JetsonBridge écoute sur les deux ports :
//   – Serial  (USB-CDC)  : connexion filaire PC ↔ Teensy
//   – Serial3 (XBee 868) : connexion radio via module XBee sur T4.1
// Le premier port à recevoir un "ping\n" ou une trame CRC valide devient le
// transport actif. BRIDGE_SERIAL est un macro qui déréférence le pointeur
// g_bridgeSerial → tout le code existant (telemetry, request reply, ring
// buffer drain) fonctionne sans modification.
//
// Aucun #define de compilation nécessaire pour changer de mode.

extern Stream* g_bridgeSerial;           // Defined in jetson_bridge.cpp
#define BRIDGE_SERIAL    (*g_bridgeSerial)
#define BRIDGE_USB       Serial          // USB-CDC — /dev/ttyACM0 / COMx
#define BRIDGE_XBEE      Serial2         // XBee 868 MHz radio module
#define BRIDGE_BAUDRATE  57600

// Console debug — toujours sur USB (lignes sans |crc ignorées par le parser Python)
#define CONSOLE_SERIAL   Serial
#define CONSOLE_BAUDRATE 57600

#define TW_STRAT1 CHERRY
#define TW_STRAT2 CAKE
#define TW_STRAT_OPT1 BROWN
#define TW_STRAT_OPT2 NOBROWN


// Occupancy map size
#define GRID_WIDTH    20
#define GRID_HEIGHT   13
#define GRID_CELLSIZE 150
#define GRID_BITS     (GRID_WIDTH * GRID_HEIGHT)
#define GRID_BYTES    ((GRID_BITS + 7) / 8)

#define TABLE_SIZE_X 3000  // mm
#define TABLE_SIZE_Y 2000  // mm


// Struct definition
struct CalibrationProfile {
    Vec3 Holonomic,
         Cartesian;
};

typedef bool Color;

// Namespace
namespace Settings {

    namespace Threads {
        constexpr long DEFAULT_REFRESH   = 100;  // ms
        constexpr int  DEFAULT_STACKSIZE = 1024;
    }

    constexpr bool
    PRIMARY   = false,
    SECONDARY = !PRIMARY,
    COLOR_A   = true,
    COLOR_B   = !COLOR_A,
    YELLOW    = COLOR_A,
    BLUE      = COLOR_B;

    namespace Match {
        constexpr bool
        AVOIDANCE    = true,
        NO_AVOIDANCE = !AVOIDANCE;

        constexpr short
        STRAT_PRIMARY_A   = false,
        STRAT_PRIMARY_B   = !STRAT_PRIMARY_A,
        STRAT_SECONDARY_A = false,
        STRAT_SECONDARY_B = !STRAT_SECONDARY_A;

        constexpr unsigned long
        DURATION      = 100 * 1000,  // 100s
        NEARLY_FINISH =  10 * 1000,  // 10s before the end
        ENDMATCH      = 200;         // 200ms before the end
    }

    namespace Inputs {}

    namespace Actuators {
        const int speed = 20;

        // ── Soft-stop limits (degrees) ──────────────────────────────────────
        // These are hardware safety bounds passed to SmartServo::constrain().
        // All preset positions MUST be within these ranges.
        // Adjust if mechanical design changes — this is the ONLY place to edit.
        namespace AB {
            constexpr int LIFT_MIN     =  80;   // lift servo min (down ~90°)
            constexpr int LIFT_MAX     = 165;   // lift servo max (up ~155°)
            constexpr int GRIPPER_MIN  =  20;   // gripper servo min (grab ~30°)
            constexpr int GRIPPER_MAX  = 100;   // gripper servo max (drop ~90°)
        }
        namespace CA {
            constexpr int ELEVATOR_MIN =   0;   // elevator servo min (down ~5°)
            constexpr int ELEVATOR_MAX =  60;   // elevator servo max (up ~50°)
            constexpr int LEFT_MIN     =  10;   // left grabber min (grab ~20°)
            constexpr int LEFT_MAX     =  75;   // left grabber max (store ~65°)
            constexpr int RIGHT_MIN    = 110;   // right grabber min (store ~121°)
            constexpr int RIGHT_MAX    = 170;   // right grabber max (grab ~162°)
        }
    }

    namespace Geometry {
        constexpr double
        RADIUS       = 125.98,
        WHEEL_RADIUS = 30;
    }

    namespace Motion {
        constexpr bool
        ABSOLUTE = true;

        // ---- PID timing ----
        const int
        PID_INTERVAL     = 2000;              // µs — période cible du cycle de contrôle

        // ---- Limites cinématiques ----
        const float
        MAX_SPEED     = 3800,             // mm/s  (OTOS max tracking speed : 2.5 m/s)
        MAX_ACCEL     = 3500,             // mm/s²
        MAX_ROT_SPEED = 10,               // rad/s
        MAX_ROT_ACCEL = 30.0f;            // rad/s²

        // ---- Seuils d'arrêt ----
        const float
        MIN_DISTANCE = 20,                // mm — tolérance position XY
        MIN_ANGLE    = 2.0f * DEG_TO_RAD; // rad — tolérance angulaire (~2°)

        // ---- Détection de blocage (stall) ----
        namespace Stall {
            constexpr uint32_t DELAY_MS         = 1000;
            constexpr uint32_t PERIOD_MS         = 500;
            constexpr float    TRANS_DISP_MM     = 5.0f;
            constexpr float    ANGLE_DISP_RAD    = 0.02f;
            constexpr float    TARGET_TRANS_MM   = 20.0f;
            constexpr float    TARGET_ANGLE_RAD  = 0.05f;
        }
    }

    namespace Stepper {
        constexpr bool
        ENABLE_POLARITY = false,
        DIR_A_POLARITY  = false,
        DIR_B_POLARITY  = false,
        DIR_C_POLARITY  = false;

        constexpr int
        PULSE_WIDTH          = 14,
        STEPS_PER_REVOLUTION = 200,
        STOP_DECCEL          = 3000,
        MAX_ACCEL            = 3000,
        MAX_SPEED            = 15000,
        STEPPER_DELAY        = 100,
        MIN_STEP_DELAY       = 20,
        MIN_STEPS            = 5,
        STEPPER_COMPUTE_DELAY= 1000,
        PULL_IN              = 100,
        PULL_OUT             = 100;

        constexpr uint8_t
        STEP_MODE = 8;
    }

    namespace Lidar {
        constexpr unsigned long persitency = 1000;  // ms
    }

    namespace Calibration {
        const CalibrationProfile Primary = {
            { 1.0f,   1.0f,   1.0f  },  // Holonomic : ABC
            { 1.089f,-1.089f, 0.831f}   // Cartesian : XY ROT
        };
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Log & Telemetry — initial state at boot
    //
    //  When holOS is connected (USB-CDC), all channels can stay on.
    //  When debugging via a plain serial terminal (no holOS), high-frequency
    //  sources flood the output — disable them here and re-enable at runtime
    //  with: log(MOTION,1)  loglevel(VERBOSE)  tel(pos,0)
    // ─────────────────────────────────────────────────────────────────────────
    namespace Log {

        // Global console log level at boot
        // VERBOSE(0) < INFO(1) < SUCCESS(2) < WARNING(3) < CRITICAL(4) < DISABLED(5)
        constexpr ConsoleLevel BOOT_LEVEL = ConsoleLevel::INFO;

        // Per-service console output enabled at boot.
        // Disable noisy services when using a plain serial terminal (no holOS).
        constexpr bool SRC_LIDAR        = true;
        constexpr bool SRC_CHRONO       = true;
        constexpr bool SRC_IHM          = true;
        constexpr bool SRC_SAFETY       = true;
        constexpr bool SRC_MOTION       = true;
        constexpr bool SRC_NAVIGATION   = true;
        constexpr bool SRC_NEOPIXEL     = false;  // LED driver noise — off by default
        constexpr bool SRC_INTERCOM     = false;  // Protocol-level noise — off by default
        constexpr bool SRC_TERMINAL     = true;
        constexpr bool SRC_ACTUATORS    = true;
        constexpr bool SRC_LOCALISATION = false;  // High-frequency OTOS updates — off by default
        constexpr bool SRC_VISION       = true;
        constexpr bool SRC_JETSON       = true;

        // JetsonBridge telemetry channel initial state.
        // Disable high-rate channels when debugging without holOS (e.g. plain serial).
        namespace Telemetry {
            constexpr bool POS    = true;
            constexpr bool MOTION = true;
            constexpr bool SAFETY = true;
            constexpr bool CHRONO = true;
            constexpr bool OCC    = true;
        }

    }  // namespace Log

}  // namespace Settings
