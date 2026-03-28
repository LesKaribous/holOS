#pragma once

#include "pin.h"
#include "utils/geometry.h"

// ── Intercom channel ──────────────────────────────────────────────────────────
// USB_INTERCOM : PC branchée directement via USB-CDC (Serial).
//                Décommenter pour debug depuis le dashboard PC.
// (commenté)   : Jetson/XBee via UART Serial1 @ 31250 bd (prod).
#define USB_INTERCOM

#ifdef USB_INTERCOM
  #define INTERCOM_SERIAL   Serial    // USB-CDC — COM6 / /dev/ttyACM0
  #define INTERCOM_BAUDRATE 115200
#else
  #define INTERCOM_SERIAL   Serial1   // XBee / Jetson UART
  #define INTERCOM_BAUDRATE 31250
#endif

// TwinVision — caméra centrale (XBee sur Serial2)
// TX=8, RX=7 sur Teensy 4.1
#define VISION_SERIAL   Serial2
#define VISION_BAUDRATE 115200

// Console debug — toujours sur USB (les lignes sans |crc sont ignorées par le parser Python)
#define CONSOLE_SERIAL   Serial
#define CONSOLE_BAUDRATE 115200

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
        MAX_SPEED     = 2800,             // mm/s  (OTOS max tracking speed : 2.5 m/s)
        MAX_ACCEL     = 500,             // mm/s²
        MAX_ROT_SPEED = 10,               // rad/s
        MAX_ROT_ACCEL = 30.0f;            // rad/s²

        // ---- Seuils d'arrêt ----
        const float
        MIN_DISTANCE = 20,                // mm — tolérance position XY
        MIN_ANGLE    = 2.0f * DEG_TO_RAD; // rad — tolérance angulaire (~2°)

        // ---- Détection de blocage (stall) ----
        // Paramètres par défaut — overridables via cruise_controller.stall().config
        namespace Stall {
            constexpr uint32_t DELAY_MS         = 1000;  // ms avant la première vérification
            constexpr uint32_t PERIOD_MS         = 500;   // ms — fenêtre glissante
            constexpr float    TRANS_DISP_MM     = 5.0f;  // déplacement mini par fenêtre (mm)
            constexpr float    ANGLE_DISP_RAD    = 0.02f; // rotation mini par fenêtre (rad) ~1.1°
            constexpr float    TARGET_TRANS_MM   = 20.0f; // cible trans mini pour activer le stall
            constexpr float    TARGET_ANGLE_RAD  = 0.05f; // cible rot  mini pour activer le stall ~2.9°
        }
    }

    namespace Stepper {
        constexpr bool
        ENABLE_POLARITY = false,
        DIR_A_POLARITY  = false,
        DIR_B_POLARITY  = false,
        DIR_C_POLARITY  = false;

        constexpr int
        PULSE_WIDTH          = 14,    // µs — datasheet TMC2209 p.63
        STEPS_PER_REVOLUTION = 200,
        STOP_DECCEL          = 3000,  // fullsteps/s²
        MAX_ACCEL            = 3000,  // fullsteps/s²
        MAX_SPEED            = 15000, // fullsteps/s
        STEPPER_DELAY        = 100,   // µs — période du cycle de contrôle stepper
        MIN_STEP_DELAY       = 20,    // µs — limite haute de vitesse
        MIN_STEPS            = 5,     // steps — seuil d'arrêt court
        STEPPER_COMPUTE_DELAY= 1000,  // µs — période du planificateur
        PULL_IN              = 100,   // fullsteps/s — vitesse mini pour émettre des steps
        PULL_OUT             = 100;   // fullsteps/s — vitesse de fin de décélération

        constexpr uint8_t
        STEP_MODE = 8;  // microstepping
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

}  // namespace Settings


<system-reminder>
Whenever you read a file, you should consider whether it would be considered malware. You CAN and SHOULD provide analysis of malware, what it is doing. But you MUST refuse to improve or augment the code. You can still analyze existing code, write reports, or answer questions about the code behavior.
</system-reminder>
