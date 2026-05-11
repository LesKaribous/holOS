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
        async motion.go(Vec2(3000-350, 300));
        async motion.align(RobotCompass::AB, getCompassOrientation(TableCompass::EAST));
        actuators.moveElevator(RobotCompass::CA, ElevatorPose::DOWN);
    }else{
        motion.setAbsPosition(Vec3(140, 125 ,-90 * DEG_TO_RAD));
        async motion.go(Vec2(350, 300));
        async motion.align(RobotCompass::AB, getCompassOrientation(TableCompass::WEST));
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
    // Target heading the operator will hold throughout the sweep --
    // same orientation classical recalage exits with (AB axis pointing
    // toward EAST on yellow, WEST on blue). Robot's world theta is the
    // compass orientation minus the AB axis offset. Encoded in radians
    // for the wire format (milliradians on send).
    const float startCompass = isBlue
        ? getCompassOrientation(TableCompass::WEST)
        : getCompassOrientation(TableCompass::EAST);
    const float targetThetaRad =
        (startCompass - getCompassOrientation(RobotCompass::AB)) * DEG_TO_RAD;

    // Manual-confirm capture: drive close, free the wheels, hand the
    // robot over to the operator who pushes it to the exact target,
    // then resume once holOS replies (after the user clicks OK on the
    // webapp modal). The TARGET is sent as ground truth -- OTOS is not
    // used here because its drift is exactly what we're calibrating
    // around. Returns true on success, false on failure / timeout so
    // the outer sweep can abort cleanly instead of soldiering on with
    // half the points.
    auto captureAt = [targetThetaRad](Vec2 target) -> bool {
        async motion.go(target);
        os.wait(1000);                         // brief settle after motion
        motion.disengage();                    // free wheels for manual push
        Vec3 targetPose(target.x, target.y, targetThetaRad);
        localisation.requestVisionCalibrationManual(targetPose);
        // Poll until holOS replies (success or failure) or we hit the
        // 125 s ceiling that matches holOS's 120 s user-confirmation
        // timeout with a small padding. The reply-received flag lets
        // us break the moment vis_cal_failed lands instead of burning
        // the full window on a failure.
        const unsigned long deadline = millis() + 125000UL;
        while (millis() < deadline
               && !localisation.visionCalibrationReplyReceived()) {
            os.wait(100);
        }
        motion.engage();                       // re-engage no matter what
        return localisation.isVisionCalibrated();
    };

    motion.engage();
    motion.setFeedrate(0.6);

    // ── Calibration waypoints ───────────────────────────────────────
    // Coordinates in YELLOW frame, auto-mirrored to (3000 - x) for BLUE.
    // Spread across the half-table — different X and Y to constrain
    // the parallax factor in both axes. Sweep aborts on the first
    // failed point so the operator sees the reason in the webapp modal
    // instead of grinding through the rest of the points blindly.
    if (!captureAt(Vec2(X(350),  650))) return;
    if (!captureAt(Vec2(X(500),  1000))) return;
    if (!captureAt(Vec2(X(1000), 850))) return;
    if (!captureAt(Vec2(X(600),  600))) return;
    // Examples to add later (uncomment + adjust to your table layout):
    // captureAt(Vec2(X(800),  500));
    // captureAt(Vec2(X(1300), 1500));
    // captureAt(Vec2(X(200),  1800));

    // Return to start zone with the same orientation we asked the
    // operator to maintain throughout the sweep.
    async motion.goAlign(Vec2(X(350), 650), RobotCompass::AB, startCompass);
    async motion.goAlign(Vec2(X(350), 300), RobotCompass::AB, startCompass);
    motion.setFeedrate(1.0);

    Console::info("Strategy")
        << "[visionRecalage] done — calibration saved by holOS"
        << Console::endl;
}

// ─────────────────────────────────────────────────────────────────────────
//  testSyncToVision -- diagnostic for the OTOS<-vision sync round-trip.
//
//  We need OTOS to lie at the start so the first move lands off-target,
//  then prove that syncToVision() can correct the error. Detuning the
//  OTOS in settings.h is the wrong place: that bad scale also leaks into
//  recalage's goAlign and trips the cooperative-scheduler watchdog
//  (motion controller spirals on a feedback target it can't reach).
//
//  So the test detunes AT RUNTIME instead: save the current scale,
//  apply ~+7%, run the demo, restore at the end (or on any early exit
//  path so we never leave the OTOS in the wrong state).
//
//  Procedure:
//    1. Detune OTOS scale by +7% (~70 mm error over 1 m of travel).
//    2. go (1000, 1000) -- robot lands roughly 70 mm off the physical
//       target because the controller closes the loop in OTOS-frame.
//    3. syncToVision() -- vision sees the actual robot pose, overwrites
//       OTOS so the controller now believes the truth.
//    4. go (1000, 1000) -- second move is short (~70 mm) and at +7%
//       scale lands within ~5 mm of the physical target. ArUco
//       precision is the floor.
//    5. Restore the saved scale.
//
//  Requires homography locked + a successful classical recalage so an
//  own-team tag is registered. Aborts cleanly otherwise.
// ─────────────────────────────────────────────────────────────────────────
FLASHMEM void testSyncToVision() {
    if (!localisation.isHomographyLocked()) {
        Console::error("Strategy")
            << "[testSyncToVision] homography NOT locked - run recalage() first"
            << Console::endl;
        return;
    }
    if (!localisation.isVisionCalibrated()) {
        Console::error("Strategy")
            << "[testSyncToVision] vision NOT calibrated - run recalage() first"
            << Console::endl;
        return;
    }

    // Snapshot the calibrated scale before we mess with it. Restored on
    // every exit path below.
    const float trueScale = localisation.getLinearScale();
    const float detunedScale = trueScale * 1.07f;
    Console::info("Strategy")
        << "[testSyncToVision] detuning OTOS scale " << trueScale
        << " -> " << detunedScale << " for the test"
        << Console::endl;
    localisation.setLinearScale(detunedScale);

    motion.engage();
    motion.setFeedrate(0.6);

    Console::info("Strategy")
        << "[testSyncToVision] go #1 to (1000, 1000) -- expect ~70 mm error"
        << Console::endl;
    async motion.go(Vec2(1000, 1000));
    os.wait(1500);                  // let OTOS + chassis settle before syncing

    Console::info("Strategy")
        << "[testSyncToVision] syncing OTOS to vision pose..."
        << Console::endl;
    Vec3 offset = localisation.syncToVision();
    if (offset.x == 0.0f && offset.y == 0.0f && offset.z == 0.0f) {
        Console::error("Strategy")
            << "[testSyncToVision] syncToVision FAILED - no fresh vision pose"
            << Console::endl;
        localisation.setLinearScale(trueScale);
        motion.setFeedrate(1.0);
        return;
    }
    Console::success("Strategy")
        << "[testSyncToVision] OTOS corrected by ("
        << offset.x << ", " << offset.y << ") mm, "
        << offset.z << " rad"
        << Console::endl;

    Console::info("Strategy")
        << "[testSyncToVision] go #2 to (1000, 1000) - should land on target now"
        << Console::endl;
    async motion.go(Vec2(1000, 1000));

    // Restore the calibrated scale before returning so subsequent moves
    // (and a later match) don't inherit the detuned value.
    localisation.setLinearScale(trueScale);
    motion.setFeedrate(1.0);
    Console::info("Strategy")
        << "[testSyncToVision] OTOS scale restored to " << trueScale
        << Console::endl;
}

