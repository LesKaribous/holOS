#pragma once
#include "config/settings.h"
#include "services/service.h"
#include "os/jobs/job.h"
#include "utils/geometry.h"
#include "services/motion/controller/positionController.h"
#include "services/motion/controller/pursuitController.h"
#include "services/motion/controller/stepperController.h"
#include "services/motion/stepper.h"

#include <Wire.h>
#include <SPI.h>

// ============================================================
//  Motion — service de déplacement holonomique
//
//  Deux contrôleurs disponibles :
//    - CruiseMode (PositionController) : PID + OTOS, précis,
//      utilisé quand localisation est active.
//    - StepperMode (StepperController) : open-loop pas-à-pas,
//      fallback ou déplacements courts.
//
//  Usage typique dans un bloc Mission :
//
//    // Move simple (auto-cancel si coincé)
//    async motion.go(500, 200);
//    if (!motion.wasSuccessful()) return BlockResult::FAILED;
//
//    // Move sans détection de blocage :
//    async motion.noStall().go(500, 200);
//
//    // Move avec feedrate réduit :
//    async motion.feedrate(0.7f).go(500, 200);
//
//    // Chemin multi-waypoints :
//    async motion.via(200, 0).via(200, 300).go(500, 300);
//
//  Diagnostic post-move :
//    motion.printDiagReport();
//    MoveStats s = motion.getLastStats();
//    // s.stallMinTransMm → seuil optimal pour Stall::TRANS_DISP_MM
// ============================================================

class Motion : public Service, public Job {
public:

    // ============================================================
    //  Modes de contrôle
    //   LEGACY_WAYPOINT : queue de waypoints (cruise/stepper),
    //                     comportement historique.
    //   LIVE_PURSUIT    : suivi de cible vive (aim()), idéal pour
    //                     navigation haut niveau replanifiée à la
    //                     volée par le bridge Python.
    // ============================================================
    enum class ControlMode { LEGACY_WAYPOINT, LIVE_PURSUIT };

    // ============================================================
    //  Options par move — POD appliqué dans move(), reset ensuite
    // ============================================================
    struct MoveOptions {
        bool  stallEnabled     = false;   // active la stall detection
        bool  cancelOnStall    = false;   // annule le move si stall détecté
        bool  optimizeRotation = true;   // minimise la rotation (stepper mode)
        float feedrate         = -1.0f;  // -1 → utilise le feedrate global
    };

    // ============================================================
    //  Waypoint interne — entrée de la queue
    // ============================================================
    struct Waypoint {
        Vec3        target;
        bool        passThrough;  // true = ne pas s'arrêter, accepter dans WAYPOINT_RADIUS
        MoveOptions opts;
    };

    // ============================================================
    //  Statistiques d'un move — disponibles après complete/cancel
    //  Utiles pour calibrer les seuils de stall.
    // ============================================================
    struct MoveStats {
        uint32_t durationMs       = 0;
        float    targetDistMm     = 0.0f;   // distance euclidienne start→target
        float    traveledMm       = 0.0f;   // distance réelle parcourue (OTOS)
        float    targetAngleDeg   = 0.0f;   // rotation demandée
        float    traveledAngleDeg = 0.0f;   // rotation réelle
        bool     stalled          = false;  // stall déclenché pendant le move
        // Détail stall (pour tuner Stall::TRANS_DISP_MM / ANGLE_DISP_RAD)
        int      stallChecks      = 0;      // fenêtres évaluées
        float    stallMinTransMm  = 0.0f;   // pire fenêtre trans observée
        float    stallMinAngleDeg = 0.0f;   // pire fenêtre rot observée
        // Saturation PID (pour diagnostiquer les feedrates trop élevés)
        int      satXCycles       = 0;
        int      satYCycles       = 0;
        int      satZCycles       = 0;
        bool     wasSuccessful    = false;
    };

    Motion();

    void attach()  override;
    void run()     override;
    void exec()    override;

    void enable()  override;
    void disable() override;

    // Engage = moteurs alimentés et prêts ; Disengage = moteurs hors tension
    void engage();
    void disengage();

    // ---- Fluent — options pour le prochain move ----
    // Enchaîner avant la commande de déplacement :
    //   async motion.noStall().feedrate(0.8f).go(x, y);

    Motion& noStall();                      // désactive stall detection
    Motion& withStall(bool on = true);      // contrôle fin de la stall detection
    Motion& cancelOnStall(bool on = true);  // annule si stall détecté
    Motion& withOptimization(bool on = true);
    Motion& feedrate(float f);

    // ---- Waypoints ----
    Motion& via(Vec2 wp);
    Motion& via(float x, float y);

    // ---- Commandes de mouvement ----
    Motion& go(Vec2 target);
    Motion& go(float x, float y);
    Motion& goAlign(Vec2 target, RobotCompass rc, float orientation);
    Motion& goPolar(float heading, float dist);
    Motion& goPolarAlign(float heading, float dist, RobotCompass rc, float orientation);
    Motion& turn(float angle);
    Motion& align(RobotCompass rc, float orientation);
    Motion& move(Vec3 target);  // commande bas niveau — applique m_pendingOpts

    // ---- Cycle ISR ----
    void step();
    void control();

    // ---- Cycle de vie du Job ----
    void start()       override;
    void pause()       override;
    void resume()      override;
    void cancel()      override;
    void forceCancel() override;
    void complete()    override;

    void onPausing()   override;
    void onCanceling() override;
    void onPaused()    override;
    void onCanceled()  override;

    // ---- État ----
    Vec3 estimatedPosition();
    bool hasFinished();
    bool wasSuccessful() const;

    bool isAbsolute()  const;
    bool isRelative()  const;
    bool isRotating()  const;
    bool isSleeping()  const;
    bool isMoving()    const;

    // ---- Modes globaux ----
    void setAbsolute();
    void setRelative();
    void setAsync();
    void setSync();
    void enableCruiseMode();
    void disableCruiseMode();

    // ---- Mode contrôle (waypoint vs pursuit) ----
    void        setControlMode(ControlMode m);
    ControlMode getControlMode() const { return m_controlMode; }

    // ---- Pursuit mode API (LIVE_PURSUIT uniquement) ----
    // aim(x,y) installe/actualise la cible vive et démarre le job
    // pursuit s'il n'est pas déjà en cours. setHeadingMode active
    // l'alignement automatique d'une face robot sur la direction
    // de la cible.
    void aim(float x, float y);
    void aim(Vec2 target);
    void setHeadingMode(bool enabled, RobotCompass face = RobotCompass::A);
    bool isHeadingMode() const;
    bool isPursuitWatchdogTripped() const;

    // ---- Artificial Potential Fields avoidance ──────────────────────────────
    // Requires cruise mode (OTOS feedback).  Has no effect in stepper mode.
    // scale > 0 overrides the default gain; pass -1 to keep current value.
    void enableAPF(float scale = -1.0f);
    void disableAPF();
    bool isAPFEnabled() const;

    // ---- Position / target ----
    void  setAbsPosition(Vec3);
    Vec3  getAbsPosition() const;
    void  setAbsTarget(Vec3);
    Vec3  getAbsTarget()   const;

    float getTargetDirection()         const;
    float getAbsoluteTargetDirection() const;
    float getTargetDistance()          const;

    float getOrientation();
    void  setOrientation(float angle);

    // ---- Feedrate global [0.05 – 1.0] ----
    void  setFeedrate(float feed);
    float getFeedrate() const;

    // ---- Diagnostic ----
    MoveStats  getLastStats()    const { return m_lastStats; }
    void       printDiagReport() const;

    SINGLETON(Motion);

private:
    // ---- Steppers ----
    Stepper m_sA, m_sB, m_sC;

    // ---- Contrôleurs ----
    PositionController cruise_controller;
    StepperController  stepper_controller;
    PursuitController  pursuit_controller;

    bool        use_cruise_mode      = true;
    bool        current_move_cruised = false;
    bool        current_move_pursuit = false;
    ControlMode m_controlMode        = ControlMode::LEGACY_WAYPOINT;

    // ---- Options ----
    MoveOptions m_pendingOpts;
    MoveOptions m_activeOpts;

    // ---- Waypoint queue ----
    static constexpr int   WAYPOINT_CAPACITY = 8;
    static constexpr float WAYPOINT_RADIUS   = 80.0f;  // mm

    Waypoint m_waypoints[WAYPOINT_CAPACITY];
    int  m_waypointCount = 0;
    int  m_waypointIndex = 0;

    void enqueueWaypoint(Vec3 target, bool passThrough);
    bool advanceWaypoint();
    void clearWaypoints();

    // ---- Modes globaux ----
    bool m_async           = true;
    bool _optimizeRotation = true;
    bool _absolute         = true;

    // ---- État ----
    bool _engaged    = false;
    bool _sleeping   = false;
    bool _isMoving   = false;
    bool _isRotating = false;

    float m_feedrate = 1.0f;

    Vec3 _startPosition = {0, 0, 0};
    Vec3 _position      = {0, 0, 0};
    Vec3 _target        = {0, 0, 0};
    Vec2 _controlPoint  = {0, 0};

    // ---- Diagnostic ----
    MoveStats m_lastStats;
    uint32_t  m_moveStartMs = 0;  // timestamp millis() au start du move

    void collectStats(bool success);  // remplit m_lastStats à la fin du move

    // ---- Helpers ----
    void onRunning();
    void startWaypoint(const Waypoint& wp);
    Vec3 optimizeRelTarget(Vec3 relTarget);
    Vec3 toRelativeTarget(Vec3 absTarget);
    Vec3 toAbsoluteTarget(Vec3 relTarget);
};

SINGLETON_EXTERN(Motion, motion)
