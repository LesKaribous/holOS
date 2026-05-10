#include "auto_tune.h"
#include "strategy.h"
#include "block_registry.h"
#include "config/poi.h"
#include "config/score.h"
#include "config/env.h"
#include "config/runtime_config.h"
#include "routines.h"
#include "mission.h"


FLASHMEM void calibrate(){
    motion.disableCruiseMode();
    //motion.collide(false);

    float start = localisation.getPosition().x;
    float distance = 0;
    float distanceGoal = 400;
    float scale = 0;
    float current = 0;

    for(int i = 0; i < 3; i++){
        start = localisation.getPosition().x;
	    async motion.goPolar(0,distanceGoal);
        current = Vec2(localisation.getPosition()).mag();
        distance = fabs(current - start);
        Console::info() << "distance : " << current - start << "|" << 400 << Console::endl;
        scale = distanceGoal/distance;
        localisation.setLinearScale(scale);

        start = localisation.getPosition().x;
	    async motion.goPolar(0,-distanceGoal);
        current = Vec2(localisation.getPosition()).mag();
        distance = fabs(current - start);
        Console::info() << "distance : " << current - start << "|" << 400 << Console::endl;
        scale = distanceGoal/distance;
        localisation.setLinearScale(scale);

        
    }

    //motion.collide(false);
    motion.enableCruiseMode();
}


FLASHMEM void probeBorder(TableCompass tc, RobotCompass rc, float clearance, float approachDist, float probeDist, float feedrate){

    boolean wasAbsolute = motion.isAbsolute();
    float currentFeedrate = motion.getFeedrate();
    actuators.moveElevator(rc, ElevatorPose::UP);

    motion.setFeedrate(feedrate);
    async motion.align(rc, getCompassOrientation(tc));
    motion.setRelative();

    // Cruise mode requis pour que snap() fonctionne (PID + OTOS + stall detector).
    // snap(true) : si on stalle près d'une bordure, la coord de l'axe stallé est
    // automatiquement snappée au mur connu (margin côté Xmin/Ymin, TABLE-margin
    // côté Xmax/Ymax) et le move termine proprement.
    motion.enableCruiseMode();
    motion.snap(true);
    async motion.goPolar(getCompassOrientation(rc), approachDist);
    async motion.goPolar(getCompassOrientation(rc), probeDist);
    motion.snap(false);

    // Correction explicite de l'orientation : snap ne touche pas à position.c.
    // On recale aussi la coord selon tc pour rester aligné avec les conventions
    // des appelants (au cas où snap aurait raté, typiquement si on probe un
    // obstacle loin des bordures).
    float _offset  = getOffsets(rc);
    Vec3  position = motion.estimatedPosition();
    Console::println(_offset);
    Console::println(position);

    if      (tc == TableCompass::NORTH) position.y = 0.0    + _offset;
    else if (tc == TableCompass::SOUTH) position.y = 2000.0 - _offset;
    else if (tc == TableCompass::EAST)  position.x = 3000.0 - _offset;
    else if (tc == TableCompass::WEST)  position.x = 0.0    + _offset;
    position.c = DEG_TO_RAD * (getCompassOrientation(tc) - getCompassOrientation(rc));

    motion.setAbsPosition(position);
    delay(200);

    if (clearance != 0) {
        async motion.goPolar(getCompassOrientation(rc), -clearance);
    }

    if (wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(currentFeedrate);
    actuators.moveElevator(rc, ElevatorPose::DOWN);
}


FLASHMEM StallCalibResult calibrateStall(RobotCompass face, int maxIter) {
    // ─── Paramètres de la procédure ─────────────────────────────────────────
    // Flux par itération :
    //   (initial) probeBorder aveugle → retraite à CLEARANCE du mur
    //   Phase A   : goPolar(+PROBE_BUMP)  vers mur → doit stall  (> CLEARANCE)
    //   back-off  : goPolar(-CLEARANCE)   éloignement relatif (pas de probeBorder)
    //   Phase B   : goPolar(+PROBE_CLEAR) vers mur → NE doit PAS stall  (< CLEARANCE)
    //   (si échec) safeProbeBorder avant la prochaine itération
    const float    CLEARANCE   = 500.0f;  // mm — distance au mur entre les phases
    const float    PROBE_BUMP  = 600.0f;  // mm — Phase A (> CLEARANCE → hit wall)
    const float    PROBE_CLEAR = 400.0f;  // mm — Phase B (< CLEARANCE → reste 100mm du mur)
    // NB : cruise mode → feedrate × Settings::Motion::MAX_SPEED (1800 mm/s).
    // 0.1 ⇒ ~180 mm/s, safe pour un bump-test. NE PAS monter au-dessus de 0.2
    // sans retester : le robot slammerait le mur avant que le stall detector
    // n'ait le temps de firer (stag_time typique 400 ms).
    const float    FEEDRATE    = 0.1f;    // ~180 mm/s — vitesse de calibration safe

    // ─── Snapshot état motion / RuntimeConfig pour restauration fin ─────────
    const boolean wasAbsolute    = motion.isAbsolute();
    const float   savedFeedrate  = motion.getFeedrate();
    const float   saved_stagMove = RuntimeConfig::getFloat("stall.stag_move_mm",
                                        Settings::Motion::Stall::STAG_MOVE_MM);
    const float   saved_stagTime = RuntimeConfig::getFloat("stall.stag_time",
                                        Settings::Motion::Stall::STAG_TIME_S);
    const float   saved_velCmdMin= RuntimeConfig::getFloat("stall.vel_cmd_min",
                                        Settings::Motion::Stall::VEL_CMD_MIN_MMS);

    // Valeurs de travail (partent des valeurs courantes, pas des defaults)
    float stagMove = saved_stagMove;
    float stagTime = saved_stagTime;

    StallCalibResult result;
    result.stagMoveMm = stagMove;
    result.stagTimeS  = stagTime;

    // ─── Lambdas internes ───────────────────────────────────────────────────
    // Applique les params courants dans RuntimeConfig (lu au prochain start()).
    auto applyTestParams = [&]() {
        RuntimeConfig::setFloat("stall.stag_move_mm", stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    stagTime);
    };
    // Restaure les params originels pour un probeBorder intermédiaire sans
    // risque de false-positive corrompant le recalage.
    auto applySafeParams = [&]() {
        RuntimeConfig::setFloat("stall.stag_move_mm", saved_stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    saved_stagTime);
        RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);
    };
    // Rend velocity-mismatch aveugle (isole stagnation).
    auto blindVelocity = [&]() {
        RuntimeConfig::setFloat("stall.vel_cmd_min", 1.0e6f);
    };
    // Recalage WEST avec params safe, puis réapplique blinding + params de test.
    auto safeProbeBorder = [&](float clearance) {
        applySafeParams();
        probeBorder(TableCompass::WEST, face, clearance);
        blindVelocity();
        applyTestParams();
    };

    // ─── Setup initial ──────────────────────────────────────────────────────
    motion.setFeedrate(FEEDRATE);
    blindVelocity();

    Console::info("CalibStall")
        << "Initial: touch wall + back off " << (int)CLEARANCE << "mm"
        << Console::endl;
    safeProbeBorder(CLEARANCE);

    // Formate la cause per-axe du stall pour les logs (ex: "stagX+velY").
    // Vide = aucune cause détectée (triggered sans cause = displacement legacy).
    auto fmtCause = [](const Motion::MoveStats& s) -> const char* {
        // Ordre de priorité : stagnation avant velocity-mismatch.
        if (s.stallCauseStagX) return "stagX";
        if (s.stallCauseStagY) return "stagY";
        if (s.stallCauseVelX)  return "velX";
        if (s.stallCauseVelY)  return "velY";
        if (s.stallCauseVelRot)return "velR";
        return s.stalled ? "disp" : "-";
    };

    int iter = 0;
    for (iter = 0; iter < maxIter; iter++) {
        Console::info("CalibStall")
            << "───── Iter " << iter
            << "  stagMove=" << stagMove << "mm"
            << "  stagTime=" << (int)(stagTime * 1000) << "ms ─────"
            << Console::endl;

        applyTestParams();

        // ══════ PHASE A : test VRAI-POSITIF (doit stall contre le mur) ══════
        // IMPORTANT : cruise mode obligatoire — le StallDetector vit dans
        // cruise_controller.stall() (PositionController). En stepper mode
        // le détecteur n'est pas actif et collectStats() zérote m_lastStats.
        // Convention probeBorder : +D dans direction face = vers le mur.
        motion.setRelative();
        motion.collide(true);    // sticky : stall detect + cancel actifs
        async motion.goPolar(getCompassOrientation(face), +PROBE_BUMP);

        Motion::MoveStats statsA = motion.getLastStats();
        result.phaseAOk = statsA.stalled;
        Console::info("CalibStall")
            << "  Phase A (bump): " << (statsA.stalled ? "OK   " : "FAIL ")
            << "cause=" << fmtCause(statsA)
            << "  dur=" << (int)statsA.durationMs << "ms"
            << "  dist=" << (int)statsA.traveledMm << "/" << (int)PROBE_BUMP << "mm"
            << Console::endl;

        if (!statsA.stalled) {
            // Pas de stall → params trop rigides → on assouplit.
            const float oldMove = stagMove;
            const float oldTime = stagTime;
            stagMove *= 1.4f;
            stagTime *= 0.8f;
            stagMove = std::min(stagMove, 4.0f);
            stagTime = std::max(stagTime, 0.10f);
            Console::warn("CalibStall")
                << "  → loosening: stagMove " << oldMove << "→" << stagMove
                << "  stagTime " << (int)(oldTime*1000) << "→" << (int)(stagTime*1000) << "ms"
                << Console::endl;
            safeProbeBorder(CLEARANCE);
            continue;
        }

        // Phase A OK → back-off RELATIF de CLEARANCE (pas de probeBorder !).
        // On vient de stall contre le mur : pousser encore dedans n'a aucun sens.
        // On s'éloigne simplement du mur pour avoir ~CLEARANCE de jeu avant Phase B.
        Console::info("CalibStall") << "  Back-off " << (int)CLEARANCE << "mm" << Console::endl;
        motion.setRelative();
        motion.collide(false);           // désactive stall en s'éloignant du mur
        async motion.goPolar(getCompassOrientation(face), -CLEARANCE);

        // ══════ PHASE B : test FAUX-POSITIF (ne doit PAS stall) ══════════════
        // Cruise mode. On se déplace VERS le mur sur PROBE_CLEAR < CLEARANCE,
        // on reste donc en zone libre (~100 mm du mur). Tout stall ici = faux+.
        applyTestParams();
        motion.setRelative();
        motion.collide(true);
        async motion.goPolar(getCompassOrientation(face), +PROBE_CLEAR);
        motion.collide(false);

        Motion::MoveStats statsB = motion.getLastStats();
        result.phaseBOk = !statsB.stalled;
        Console::info("CalibStall")
            << "  Phase B (free): " << (!statsB.stalled ? "OK   " : "FAIL ")
            << "cause=" << fmtCause(statsB)
            << "  dur=" << (int)statsB.durationMs << "ms"
            << "  dist=" << (int)statsB.traveledMm << "/" << (int)PROBE_CLEAR << "mm"
            << Console::endl;

        if (statsB.stalled) {
            // Faux-positif → params trop sensibles → on durcit.
            const float oldMove = stagMove;
            const float oldTime = stagTime;
            stagMove *= 0.7f;
            stagTime *= 1.2f;
            stagMove = std::max(stagMove, 0.1f);
            stagTime = std::min(stagTime, 1.0f);
            Console::warn("CalibStall")
                << "  → tightening: stagMove " << oldMove << "→" << stagMove
                << "  stagTime " << (int)(oldTime*1000) << "→" << (int)(stagTime*1000) << "ms"
                << Console::endl;
            safeProbeBorder(CLEARANCE);
            continue;
        }

        // Les deux phases passent → convergence atteinte.
        result.converged = true;
        break;
    }

    result.iter       = iter + (result.converged ? 1 : 0);
    result.stagMoveMm = stagMove;
    result.stagTimeS  = stagTime;

    if (!result.converged) {
        Console::error("CalibStall")
            << "Did NOT converge in " << maxIter << " iterations — rolling back"
            << Console::endl;
        // Rollback : on remet les params originels
        result.stagMoveMm = saved_stagMove;
        result.stagTimeS  = saved_stagTime;
    }

    // ─── Écriture finale + restauration ─────────────────────────────────────
    RuntimeConfig::setFloat("stall.stag_move_mm", result.stagMoveMm);
    RuntimeConfig::setFloat("stall.stag_time",    result.stagTimeS);
    RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);

    if (wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(savedFeedrate);

    Console::info("CalibStall")
        << "DONE iter=" << result.iter
        << " stagMove=" << result.stagMoveMm
        << " stagTime=" << result.stagTimeS
        << " status=" << (result.converged ? "converged" : "abandoned")
        << Console::endl;

    return result;
}



// ============================================================
//  recalage() — classical match-day recalage.
//
//  Just two things: ask holOS to freeze the homography, then drive
//  the robot to its known starting pose. NO vision-tag pick, NO
//  parallax sweep — those moved into visionRecalage() (separate
//  command, opt-in, must run AFTER classical recalage so the H is
//  already locked).
//
//  Match-day flow:
//    1. Press the on-robot reset button (long press) or the holOS
//       UI button → this routine runs (~5 s). The persisted parallax
//       calibration is already in effect (loaded at holOS boot from
//       data/parallax_calibration.json).
//    2. Match starts.
//
//  Vision-calib day flow (once per camera (re)mount):
//    1. Run classical recalage above. ✓ appears.
//    2. Press the holOS "Vision calib" button → multi-pose sweep,
//       refits parallax z_obj, persists to disk.
// ============================================================

FLASHMEM void recalage(){
    // Ask holOS to freeze the table↔camera homography NOW, while the
    // robot is still outside the field of view (or far from the
    // anchor tags). The request is async — the lock is acknowledged
    // within 1-2 frames, well before the start-zone goAlign finishes.
    localisation.requestHomographyCapture();

    // Lock the chassis in place BEFORE calibrating the IMU. Steppers
    // produce detent torque while energised, so motion.engage() pins
    // the wheels and the operator can't accidentally nudge the robot
    // during the ~612 ms calibration window. The 1 s settle then gives
    // any residual chassis oscillation time to dampen (the operator
    // just dropped the robot or pressed the on-robot button) so the
    // gyro samples a truly stationary IMU.
    motion.engage();
    waitMs(1000);

    if (!localisation.calibrate()) {
        Console::warn("Strategy") << "[recalage] IMU re-calibration failed — "
                                  << "keeping previous offsets" << Console::endl;
    }

    //motion.disableCruiseMode();
    motion.setFeedrate(0.3);
    waitMs(600);

    if(ihm.isColor(Settings::BLUE)){
        motion.setAbsPosition(Vec3( Vec2(3000-140,100), 150 * DEG_TO_RAD));
        motion.goAlign(Vec2(3000-350, 300), RobotCompass::AB, getCompassOrientation(TableCompass::WEST));
        actuators.moveElevator(RobotCompass::CA, ElevatorPose::DOWN);
    }else{
        motion.setAbsPosition(Vec3(140, 125 ,-90 * DEG_TO_RAD));
        motion.goAlign(Vec2(350, 300), RobotCompass::AB, getCompassOrientation(TableCompass::EAST));
    }
    motion.setFeedrate(1.0);

    initPump(); //TODO : Integrate into Actuators
}


// ============================================================
//  visionRecalage() — multi-pose vision parallax calibration.
//
//  REQUIRES the homography to be locked first (= classical recalage
//  must have run). Bails with an error log if not.
//
//  Drives the robot through N known waypoints across its half of
//  the table. At each waypoint the robot stops, settles, and fires
//  a cal_request to holOS. Server-side, every pair refits the
//  parallax solver and the resulting object_z_mm is pushed to the
//  live parallax node + persisted to disk.
//
//  Run this once per camera (re)mount. The persisted config is
//  reloaded at every holOS boot so match-day recalage doesn't
//  re-run the sweep.
//
//  To improve reliability: add more entries to WAYPOINTS below.
//  Spread them across the team's half of the table to give the
//  solver coverage in both X and Y.
// ============================================================

FLASHMEM void visionRecalage(){
    if (!localisation.isHomographyLocked()) {
        Console::error("Strategy")
            << "[visionRecalage] homography NOT locked — run classical "
            << "recalage() first" << Console::endl;
        return;
    }

    Console::info("Strategy")
        << "[visionRecalage] start (multi-pose parallax sweep)" << Console::endl;

    const bool isBlue = ihm.isColor(Settings::BLUE);
    // Mirror X for the BLUE team. Yellow frame is canonical.
    auto X = [isBlue](float x) -> float {
        return isBlue ? (3000.0f - x) : x;
    };
    // Drive to a pose, settle, fire one cal_request, wait for reply.
    // 3 s settle covers the chassis velocity tail + camera shutter.
    // 1.5 s reply wait covers up to 3 server-side retries.
    auto captureAt = [](Vec2 pos) {
        async motion.go(pos);
        os.wait(3000);
        localisation.requestVisionCalibration(localisation.getPosition());
        os.wait(1500);
    };

    motion.engage();
    motion.setFeedrate(0.6);

    // Initial cal_request at the current (start zone) pose. Adds
    // a near-edge sample to the calibration set. The robot is
    // assumed to be at the recalage start position from a prior
    // classical recalage.
    os.wait(3000);
    localisation.requestVisionCalibration(localisation.getPosition());
    os.wait(1500);
    if (localisation.isVisionCalibrated()) {
        Vec3 offset = localisation.syncToVision();
        (void)offset;
    }

    // ── Calibration waypoints (add more here for better coverage) ───
    // Coordinates in YELLOW frame, auto-mirrored to (3000 - x) for BLUE.
    // Spread across the half-table — different X and Y to constrain
    // the parallax factor in both axes. RMS in /vision_debug should
    // drop as more pairs are added.
    captureAt(Vec2(X(350),  650));
    captureAt(Vec2(X(500),  1000));
    captureAt(Vec2(X(1000), 850));
    // Examples to add later (uncomment + adjust to your table layout):
    // captureAt(Vec2(X(800),  500));
    // captureAt(Vec2(X(1300), 1500));
    // captureAt(Vec2(X(200),  1800));

    // Return to start zone with proper orientation.
    const float startCompass = isBlue
        ? getCompassOrientation(TableCompass::WEST)
        : getCompassOrientation(TableCompass::EAST);
    async motion.goAlign(Vec2(X(350), 650), RobotCompass::AB, startCompass);
    async motion.goAlign(Vec2(X(350), 300), RobotCompass::AB, startCompass);
    motion.setFeedrate(1.0);

    Console::info("Strategy")
        << "[visionRecalage] done — calibration saved by holOS"
        << Console::endl;
}

