#include "motion.h"
#include "kinematics.h"
#include "config/settings.h"
#include "os/console.h"
#include "utils/timer/timer.h"
#include "services/localisation/localisation.h"
#include <cmath>

SINGLETON_INSTANTIATE(Motion, motion)

// ============================================================
//  Construction / attach
// ============================================================

FLASHMEM Motion::Motion()
    : Service(ID_MOTION),
      m_sA(Pin::Stepper::stepA, Pin::Stepper::dirA, !Settings::Stepper::DIR_A_POLARITY),
      m_sB(Pin::Stepper::stepB, Pin::Stepper::dirB, !Settings::Stepper::DIR_B_POLARITY),
      m_sC(Pin::Stepper::stepC, Pin::Stepper::dirC, !Settings::Stepper::DIR_C_POLARITY)
{
    disengage();
}

FLASHMEM void Motion::attach() {
    Console::info("Motion") << "Motion attached" << Console::endl;

    Job::reset();
    stepper_controller.setSteppers(&m_sA, &m_sB, &m_sC);
    cruise_controller.setSteppers(&m_sA, &m_sB, &m_sC);
    pursuit_controller.setSteppers(&m_sA, &m_sB, &m_sC);

    _absolute      = Settings::Motion::ABSOLUTE;
    _startPosition = {0, 0, 0};
    _position      = {0, 0, 0};
    _target        = {0, 0, 0};
    _controlPoint  = {0, 0};
    clearWaypoints();

    pinMode(Pin::Stepper::enable, OUTPUT);
}


// ============================================================
//  Service::run — machine d'état principale
// ============================================================

FLASHMEM void Motion::run() {
    if (!enabled()) return;
    if      (isPausing())   onPausing();
    else if (isCanceling()) onCanceling();
    else if (isRunning())   onRunning();
}

FLASHMEM void Motion::exec() { run(); }


// ============================================================
//  États en cours
// ============================================================

FLASHMEM void Motion::onRunning() {
    if (!isBusy()) return;

    // ── Mode pursuit : pas de queue, pas de complétion automatique ──
    // Le contrôleur tourne tant qu'il reçoit des aim() et qu'il n'est
    // pas annulé. C'est la couche haute (Python) qui décide quand
    // l'arrêter via cancel().
    if (current_move_pursuit) {
        if (pursuit_controller.isCanceled()) {
            Console::warn("Motion") << "Pursuit canceled by controller" << Console::endl;
            onCanceled();
        }
        return;
    }

    // Annulation venue du contrôleur
    bool controllerCanceled = current_move_cruised ? cruise_controller.isCanceled()
                                                   : stepper_controller.isCanceled();
    if (controllerCanceled) {
        Console::warn("Motion") << "Move canceled by controller" << Console::endl;
        clearWaypoints();
        onCanceled();
        return;
    }

    // Stall → cancel si l'option est active
    if (current_move_cruised && m_activeOpts.stallEnabled && cruise_controller.isStalled()) {
        if (m_activeOpts.cancelOnStall) {
            Console::warn("Motion") << "Stall detected — canceling" << Console::endl;
            clearWaypoints();
            cruise_controller.cancel();
        }
        return;
    }

    // Pass-through : enchaîner si suffisamment proche du waypoint intermédiaire
    bool passThrough = (m_waypointIndex < m_waypointCount - 1)
                    && (m_waypoints[m_waypointIndex].passThrough);

    if (passThrough) {
        float distToWp = Vec2(_target - _position).mag();
        if (distToWp < WAYPOINT_RADIUS) {
            if (!advanceWaypoint()) {
                startWaypoint(m_waypoints[m_waypointIndex]);
            } else {
                complete();
            }
            return;
        }
    }

    // Fin normale du move courant
    if (hasFinished()) {
        if (!advanceWaypoint()) {
            startWaypoint(m_waypoints[m_waypointIndex]);
        } else {
            complete();
        }
        return;
    }

    // Tick du contrôleur stepper (le cruise est piloté par son propre cycle ISR)
    if (!current_move_cruised) stepper_controller.exec();
}

FLASHMEM void Motion::onPausing() {
    bool finished = false;

    if (current_move_pursuit) {
        // Pursuit n'a pas de notion propre de pause : on rentre en
        // canceling et on sort dès que la vélocité a atteint zéro.
        finished = pursuit_controller.isCanceled() ||
                   pursuit_controller.isCompleted();
    } else if (current_move_cruised) {
        Vec3 pos = estimatedPosition();
        if (pos != _position) {
            _position = pos;
            cruise_controller.setPosition(pos);
        }
        cruise_controller.exec();
        finished = cruise_controller.isCanceled();
    } else {
        stepper_controller.exec();
        finished = stepper_controller.isCanceled();
    }

    if (finished) onPaused();
}

FLASHMEM void Motion::onCanceling() {
    bool finished = false;

    if (current_move_pursuit) {
        // Le pursuit_controller décélère seul dans son onCanceling().
        finished = pursuit_controller.isCanceled() ||
                   pursuit_controller.isCompleted();
    } else if (current_move_cruised) {
        Vec3 pos = estimatedPosition();
        if (pos != _position) {
            _position = pos;
            cruise_controller.setPosition(pos);
        }
        cruise_controller.exec();
        finished = cruise_controller.isCompleted();
    } else {
        stepper_controller.exec();
        finished = stepper_controller.isCompleted();
    }

    if (finished) onCanceled();
}

FLASHMEM void Motion::onPaused() {
    Job::onPaused();
    _isMoving   = false;
    _isRotating = false;
    Console::info("Motion") << "Move paused" << Console::endl;

    cruise_controller.reset();
    stepper_controller.reset();
    _startPosition = _position = estimatedPosition();
}

FLASHMEM void Motion::onCanceled() {
    collectStats(false);

    Job::onCanceled();
    _isMoving   = false;
    _isRotating = false;

    // Drain any pending waypoints so the next go()/turn() does not replay
    // a stale queue head (root cause of the "robot re-visits previous
    // positions" bug when a soft cancel leaves the queue dirty).
    clearWaypoints();

    cruise_controller.reset();
    stepper_controller.reset();
    pursuit_controller.reset();
    current_move_pursuit = false;
    _startPosition = _position = estimatedPosition();

    Console::info("Motion") << "Move canceled" << Console::endl;
}


// ============================================================
//  Cycle ISR
// ============================================================

void Motion::step() {
    SERVICE_METHOD_HEADER
    if      (current_move_pursuit) pursuit_controller.step();
    else if (current_move_cruised) cruise_controller.step();
    else                           stepper_controller.step();
}

void Motion::control() {
    SERVICE_METHOD_HEADER
    if      (current_move_pursuit) pursuit_controller.control();
    else if (current_move_cruised) cruise_controller.control();
    else                           stepper_controller.control();
}


// ============================================================
//  Fluent — options pour le prochain move
// ============================================================

FLASHMEM Motion& Motion::noStall() {
    m_pendingOpts.stallEnabled = false;
    return *this;
}

FLASHMEM Motion& Motion::withStall(bool on) {
    m_pendingOpts.stallEnabled = on;
    return *this;
}

FLASHMEM Motion& Motion::cancelOnStall(bool on) {
    m_pendingOpts.cancelOnStall = on;
    return *this;
}

FLASHMEM Motion& Motion::withOptimization(bool on) {
    m_pendingOpts.optimizeRotation = on;
    return *this;
}

FLASHMEM Motion& Motion::feedrate(float f) {
    m_pendingOpts.feedrate = std::max(std::min(f, 1.0f), 0.05f);
    return *this;
}


// ============================================================
//  Waypoints
// ============================================================

FLASHMEM Motion& Motion::via(float x, float y) {
    return via(Vec2(x, y));
}

FLASHMEM Motion& Motion::via(Vec2 wp) {
    Vec3 target3(_absolute ? wp.a : wp.a,
                 _absolute ? wp.b : wp.b,
                 _absolute ? _position.c * RAD_TO_DEG : 0.0f);
    enqueueWaypoint(target3, true);
    return *this;
}

FLASHMEM void Motion::enqueueWaypoint(Vec3 target, bool passThrough) {
    if (m_waypointCount >= WAYPOINT_CAPACITY) {
        Console::error("Motion") << "Waypoint queue full" << Console::endl;
        return;
    }
    m_waypoints[m_waypointCount] = { target, passThrough, m_pendingOpts };
    m_waypointCount++;
}

FLASHMEM bool Motion::advanceWaypoint() {
    m_waypointIndex++;
    return (m_waypointIndex >= m_waypointCount);
}

FLASHMEM void Motion::clearWaypoints() {
    m_waypointCount = 0;
    m_waypointIndex = 0;
}


// ============================================================
//  API publique — mouvements
// ============================================================

FLASHMEM Motion& Motion::go(float x, float y) {
    _isMoving = true;
    return go(Vec2(x, y));
}

FLASHMEM Motion& Motion::go(Vec2 target) {
    _isMoving = true;
    Vec3 t3 = _absolute ? Vec3(target.a, target.b, _position.c * RAD_TO_DEG)
                        : Vec3(target.a, target.b, 0.0f);
    enqueueWaypoint(t3, false);
    startWaypoint(m_waypoints[0]);
    return *this;
}

FLASHMEM Motion& Motion::goPolar(float heading, float dist) {
    _isMoving = true;
    PolarVec pv(heading * DEG_TO_RAD, dist);
    Vec3 t3;
    if (_absolute) {
        Vec2 t = _position + pv.toVec2();
        t3 = Vec3(t.a, t.b, _position.c * RAD_TO_DEG);
    } else {
        Vec2 t = pv.toVec2();
        t3 = Vec3(t.a, t.b, 0.0f);
    }
    enqueueWaypoint(t3, false);
    startWaypoint(m_waypoints[0]);
    return *this;
}

FLASHMEM Motion& Motion::goPolarAlign(float heading, float dist, RobotCompass rc, float orientation) {
    _isMoving = true;
    PolarVec pv(heading * DEG_TO_RAD, dist);
    float angle = orientation - getCompassOrientation(rc);
    Vec3 t3;
    if (_absolute) {
        Vec2 t = _position + pv.toVec2();
        t3 = Vec3(t.a, t.b, angle);
    } else {
        Vec2 t = pv.toVec2();
        t3 = Vec3(t.a, t.b, angle);
    }
    enqueueWaypoint(t3, false);
    startWaypoint(m_waypoints[0]);
    return *this;
}

FLASHMEM Motion& Motion::turn(float angle) {
    _isMoving   = true;
    _isRotating = true;
    Vec3 t3 = _absolute ? Vec3(_position.a, _position.b, angle)
                        : Vec3(0.0f, 0.0f, angle);
    enqueueWaypoint(t3, false);
    startWaypoint(m_waypoints[0]);
    return *this;
}

FLASHMEM Motion& Motion::align(RobotCompass rc, float orientation) {
    return turn(orientation - getCompassOrientation(rc));
}

FLASHMEM Motion& Motion::goAlign(Vec2 target, RobotCompass rc, float orientation) {
    _isMoving   = true;
    _isRotating = true;
    float angle = orientation - getCompassOrientation(rc);
    enqueueWaypoint(Vec3(target.a, target.b, angle), false);
    startWaypoint(m_waypoints[0]);
    return *this;
}

// move() : commande bas niveau — ne passe plus par la queue.
FLASHMEM Motion& Motion::move(Vec3 target) {
    if (!enabled()) {
        Console::error("Motion") << "Motion not enabled" << Console::endl;
        return *this;
    }

    if (isRunning()) cancel();
    Job::reset();
    engage();

    target.c *= DEG_TO_RAD;

    if (isRelative()) {
        if (target.magSq() == 0.0f) {
            Console::error("Motion") << "Move is null" << Console::endl;
            complete();
            return *this;
        }
        target = toAbsoluteTarget(target);
    } else {
        if (target == _position) {
            Console::error("Motion") << "Move is null" << Console::endl;
            complete();
            return *this;
        }
    }

    _target = target;
    Console::info("Motion") << "Target: " << _target << "  Pos: " << _position << Console::endl;

    float effectiveFeedrate = (m_activeOpts.feedrate > 0.0f) ? m_activeOpts.feedrate : m_feedrate;
    Vec3  relTarget = toRelativeTarget(_target);

    cruise_controller.reset();
    stepper_controller.reset();

    if (use_cruise_mode && localisation.enabled()) {
        float feed = isRotating() ? effectiveFeedrate * 0.5f : effectiveFeedrate;
        cruise_controller.setFeedrate(feed);
        cruise_controller.setPosition(_position);
        cruise_controller.setTarget(_target);
        cruise_controller.setStallEnabled(m_activeOpts.stallEnabled);
        current_move_cruised = true;
    } else {
        if (m_activeOpts.optimizeRotation) relTarget = optimizeRelTarget(relTarget);
        Vec3 steps = ik(relTarget);
        Console::info("Motion") << "Stepper move: steps=" << steps << Console::endl;
        float feed = isRotating() ? effectiveFeedrate * 0.5f : effectiveFeedrate;
        stepper_controller.setFeedrate(feed);
        stepper_controller.setTarget(steps.a, steps.b, steps.c);
        current_move_cruised = false;
    }

    start();
    return *this;
}

// startWaypoint() : installe les options du waypoint, puis lance move().
FLASHMEM void Motion::startWaypoint(const Waypoint& wp) {
    m_activeOpts  = wp.opts;
    m_pendingOpts = MoveOptions{};  // reset aux defaults
    move(wp.target);
}


// ============================================================
//  start / pause / resume / cancel / complete
// ============================================================

FLASHMEM void Motion::start() {
    if (isPending()) return;
    Job::start();
    m_moveStartMs = millis();

    const char* mode = current_move_cruised ? "(cruise)" : "(stepper)";

    if (m_async) {
        if (current_move_cruised) cruise_controller.start();
        else                      stepper_controller.start();
        Console::info("Motion") << "Start " << mode << Console::endl;
    } else {
        if (current_move_cruised) {
            cruise_controller.start();
            Console::info("Motion") << "Start sync " << mode << Console::endl;
            while (cruise_controller.isBusy()) cruise_controller.exec();
        } else {
            stepper_controller.start();
            Console::info("Motion") << "Start sync " << mode << Console::endl;
            while (stepper_controller.isBusy()) stepper_controller.exec();
        }
        complete();
    }
}

FLASHMEM void Motion::pause() {
    Job::pause();
    if (!isPausing()) return;
    if (current_move_cruised) cruise_controller.cancel();
    else                      stepper_controller.cancel();
}

FLASHMEM void Motion::resume() {
    if (!isPaused()) return;
    Job::resume();

    if (current_move_cruised) {
        Vec3 pos = estimatedPosition();
        cruise_controller.reset();
        stepper_controller.reset();
        cruise_controller.setPosition(pos);
        float effectiveFeedrate = (m_activeOpts.feedrate > 0.0f) ? m_activeOpts.feedrate : m_feedrate;
        cruise_controller.setFeedrate(effectiveFeedrate);
        cruise_controller.setTarget(_target);
        if (m_async) {
            cruise_controller.start();
        } else {
            cruise_controller.start();
            while (cruise_controller.isPending()) cruise_controller.exec();
            complete();
        }
    } else {
        _position = estimatedPosition();
        cruise_controller.reset();
        stepper_controller.reset();
        Vec3 rel = toRelativeTarget(_target);
        if (m_activeOpts.optimizeRotation) rel = optimizeRelTarget(rel);
        Vec3 steps = ik(rel);
        float effectiveFeedrate = (m_activeOpts.feedrate > 0.0f) ? m_activeOpts.feedrate : m_feedrate;
        stepper_controller.setFeedrate(effectiveFeedrate);
        stepper_controller.setTarget(steps.a, steps.b, steps.c);
        if (m_async) {
            stepper_controller.start();
        } else {
            stepper_controller.start();
            while (stepper_controller.isPending()) stepper_controller.exec();
            complete();
        }
    }
}

FLASHMEM void Motion::cancel() {
    if (isCanceling() || isCanceled() || isCompleted()) return;
    Job::cancel();
    if (isCanceling()) {
        if      (current_move_pursuit) pursuit_controller.cancel();
        else if (current_move_cruised) cruise_controller.cancel();
        else                           stepper_controller.cancel();
    }
}

FLASHMEM void Motion::complete() {
    collectStats(true);

    _isMoving   = false;
    _isRotating = false;
    clearWaypoints();
    m_pendingOpts  = MoveOptions{};
    _startPosition = _position = estimatedPosition();
    cruise_controller.reset();
    stepper_controller.reset();
    pursuit_controller.reset();
    current_move_pursuit = false;
    Job::complete();
    Console::info("Motion") << "Move completed" << Console::endl;
}

FLASHMEM void Motion::forceCancel() {
    _isMoving   = false;
    _isRotating = false;
    clearWaypoints();
    m_pendingOpts  = MoveOptions{};
    _startPosition = _position = estimatedPosition();
    cruise_controller.reset();
    stepper_controller.reset();
    pursuit_controller.reset();
    current_move_pursuit = false;
    Job::forceCancel();
    Console::info("Motion") << "Move force-canceled" << Console::endl;
}


// ============================================================
//  enable / disable / engage / disengage
// ============================================================

FLASHMEM void Motion::enable()  { Service::enable(); }
FLASHMEM void Motion::disable() { Service::disable(); }

FLASHMEM void Motion::engage() {
    SERVICE_METHOD_HEADER
    _engaged = true;
    digitalWrite(Pin::Stepper::enable, Settings::Stepper::ENABLE_POLARITY);
}

FLASHMEM void Motion::disengage() {
    SERVICE_METHOD_HEADER
    _engaged = false;
    digitalWrite(Pin::Stepper::enable, !Settings::Stepper::ENABLE_POLARITY);
}


// ============================================================
//  Position / estimation
// ============================================================

FLASHMEM Vec3 Motion::estimatedPosition() {
    if (localisation.enabled()) {
        localisation.run();
        return localisation.getPosition();
    }
    return _position + stepper_controller.getDisplacement();
}

FLASHMEM bool Motion::hasFinished() {
    if (current_move_cruised) return cruise_controller.isCompleted();
    else                      return stepper_controller.isCompleted();
}

FLASHMEM bool Motion::wasSuccessful() const { return isCompleted(); }


// ============================================================
//  Diagnostic
// ============================================================

FLASHMEM void Motion::collectStats(bool success) {
    if (!current_move_cruised) {
        m_lastStats = {};
        m_lastStats.wasSuccessful = success;
        return;
    }

    Vec3 endPos   = estimatedPosition();
    Vec3 startPos = _startPosition;

    m_lastStats.durationMs     = (uint32_t)(millis() - m_moveStartMs);
    m_lastStats.targetDistMm   = Vec2(_target    - startPos).mag();
    m_lastStats.traveledMm     = Vec2(endPos     - startPos).mag();
    m_lastStats.targetAngleDeg = fabsf(PositionController::shortestAngleDiff(
                                           _target.c, startPos.c)) * RAD_TO_DEG;
    m_lastStats.traveledAngleDeg = fabsf(PositionController::shortestAngleDiff(
                                             endPos.c, startPos.c)) * RAD_TO_DEG;

    auto stallStats = cruise_controller.stall().getStats();
    m_lastStats.stalled          = stallStats.triggered;
    m_lastStats.stallChecks      = stallStats.checks;
    m_lastStats.stallMinTransMm  = (stallStats.minTransMm  > 1e8f) ? 0.0f : stallStats.minTransMm;
    m_lastStats.stallMinAngleDeg = (stallStats.minAngleDeg > 1e8f) ? 0.0f : stallStats.minAngleDeg;

    m_lastStats.satXCycles    = cruise_controller.satXCycles();
    m_lastStats.satYCycles    = cruise_controller.satYCycles();
    m_lastStats.satZCycles    = cruise_controller.satZCycles();
    m_lastStats.wasSuccessful = success;
}

FLASHMEM void Motion::printDiagReport() const {
    const auto& s = m_lastStats;
    Console::info("Motion::Diag")
        << "--- Move Report ---"          << Console::endl
        << "  result    : " << (s.wasSuccessful ? "OK" : "CANCELED") << Console::endl
        << "  duration  : " << (int)s.durationMs   << " ms"   << Console::endl
        << "  dist      : " << (int)s.traveledMm   << " / "
                            << (int)s.targetDistMm << " mm"   << Console::endl
        << "  angle     : " << (int)s.traveledAngleDeg << " / "
                            << (int)s.targetAngleDeg   << " deg" << Console::endl
        << "  stall     : " << (s.stalled ? "YES" : "no")
                            << "  checks=" << s.stallChecks     << Console::endl
        << "  minTrans  : " << s.stallMinTransMm  << " mm  (seuil actuel: "
                            << cruise_controller.stall().config.transDispMm << " mm)" << Console::endl
        << "  minAngle  : " << s.stallMinAngleDeg << " deg (seuil actuel: "
                            << cruise_controller.stall().config.angleDispRad * RAD_TO_DEG << " deg)" << Console::endl
        << "  PID sat   : X=" << s.satXCycles
                             << " Y=" << s.satYCycles
                             << " Z=" << s.satZCycles << " cycles" << Console::endl;
}


// ============================================================
//  Modes globaux
// ============================================================

FLASHMEM void Motion::enableCruiseMode() {
    if (isMoving()) Console::error("Motion") << "Cannot toggle cruise while moving" << Console::endl;
    else use_cruise_mode = true;
}

FLASHMEM void Motion::disableCruiseMode() {
    if (isMoving()) Console::error("Motion") << "Cannot toggle cruise while moving" << Console::endl;
    else use_cruise_mode = false;
}

FLASHMEM void Motion::enableAPF(float scale) {
    cruise_controller.setAPF(true, scale);
    Console::info("Motion") << "APF enabled (scale=" << cruise_controller.getAPFScale() << ")" << Console::endl;
}

FLASHMEM void Motion::disableAPF() {
    cruise_controller.setAPF(false);
    Console::info("Motion") << "APF disabled" << Console::endl;
}

FLASHMEM bool Motion::isAPFEnabled() const {
    return cruise_controller.isAPFEnabled();
}


// ============================================================
//  Conversions cibles
// ============================================================

FLASHMEM Vec3 Motion::toRelativeTarget(Vec3 abs) {
    abs.sub(_position);
    abs.rotateZ(_position.c);
    return abs;
}

FLASHMEM Vec3 Motion::toAbsoluteTarget(Vec3 rel) {
    rel.rotateZ(-_position.c);
    rel.add(_position);
    return rel;
}

FLASHMEM Vec3 Motion::optimizeRelTarget(Vec3 rel) {
    while (rel.c >   M_PI) rel.c -= 2.0f * M_PI;
    while (rel.c <= -M_PI) rel.c += 2.0f * M_PI;
    return rel;
}


// ============================================================
//  Setters / Getters
// ============================================================

void  Motion::setOrientation(float a)     { _position.c = a; }
FLASHMEM float Motion::getOrientation()            { return _position.c; }
Vec3  Motion::getAbsPosition() const      { return _position; }
Vec3  Motion::getAbsTarget()   const      { return _target; }

FLASHMEM float Motion::getTargetDirection()         const { return 0.0f; }  // stub
FLASHMEM float Motion::getAbsoluteTargetDirection() const { return Vec2(_target - _position).heading(); }
FLASHMEM float Motion::getTargetDistance()          const { return Vec2(_target - _position).mag(); }

FLASHMEM bool Motion::isAbsolute()  const { return _absolute; }
FLASHMEM bool Motion::isRelative()  const { return !_absolute; }
FLASHMEM bool Motion::isRotating()  const { return _isRotating; }
FLASHMEM bool Motion::isMoving()    const { return _isMoving; }
FLASHMEM bool Motion::isSleeping()  const { return _sleeping; }

FLASHMEM void Motion::setAbsPosition(Vec3 p) {
    _position = _startPosition = p;
    localisation.setPosition(p);
}

FLASHMEM void Motion::setAbsTarget(Vec3 t) { _target = t; }
FLASHMEM void Motion::setAbsolute()        { _absolute = true; }
FLASHMEM void Motion::setRelative()        { _absolute = false; }

FLASHMEM void Motion::setFeedrate(float f) { m_feedrate = std::max(std::min(f, 1.0f), 0.05f); }
FLASHMEM float Motion::getFeedrate() const { return m_feedrate; }

FLASHMEM void Motion::setAsync() { m_async = true; }
FLASHMEM void Motion::setSync()  { m_async = false; }


// ============================================================
//  Control mode (LEGACY_WAYPOINT vs LIVE_PURSUIT)
// ============================================================

FLASHMEM void Motion::setControlMode(ControlMode m) {
    if (m == m_controlMode) return;
    if (isMoving() || isRunning()) {
        Console::error("Motion") << "Cannot change control mode while moving" << Console::endl;
        return;
    }
    m_controlMode = m;
    Console::info("Motion") << "Control mode = "
        << (m == ControlMode::LIVE_PURSUIT ? "LIVE_PURSUIT" : "LEGACY_WAYPOINT")
        << Console::endl;
}


// ============================================================
//  Pursuit mode API — aim() / heading
// ============================================================

FLASHMEM void Motion::aim(float x, float y) { aim(Vec2(x, y)); }

FLASHMEM void Motion::aim(Vec2 t) {
    if (m_controlMode != ControlMode::LIVE_PURSUIT) {
        Console::error("Motion") << "aim() requires LIVE_PURSUIT mode" << Console::endl;
        return;
    }
    if (!enabled()) {
        Console::error("Motion") << "Motion not enabled" << Console::endl;
        return;
    }
    if (!localisation.enabled()) {
        Console::error("Motion") << "Pursuit requires localisation" << Console::endl;
        return;
    }

    Vec3 t3(t.a, t.b, _position.c);  // garde l'orientation actuelle par défaut

    // Premier aim : démarre le job pursuit
    if (!current_move_pursuit || !isRunning()) {
        // Si un autre job est en cours, on l'annule (force) avant
        // de basculer en pursuit.
        if (isRunning()) cancel();
        Job::reset();
        engage();

        cruise_controller.reset();
        stepper_controller.reset();
        pursuit_controller.reset();

        float effFeed = (m_activeOpts.feedrate > 0.0f) ? m_activeOpts.feedrate : m_feedrate;
        pursuit_controller.setFeedrate(effFeed);
        pursuit_controller.setPosition(_position);
        pursuit_controller.setTarget(t3);

        current_move_pursuit = true;
        current_move_cruised = false;
        _isMoving = true;
        _target   = t3;

        Console::info("Motion") << "Pursuit start, aim=" << t3 << Console::endl;
        pursuit_controller.start();
        Job::start();
        m_moveStartMs = millis();
    } else {
        // Mise à jour vivante de la cible — pas de reset
        pursuit_controller.setTarget(t3);
        _target = t3;
    }
}

FLASHMEM void Motion::setHeadingMode(bool enabled, RobotCompass face) {
    pursuit_controller.setHeadingMode(enabled, face);
    Console::info("Motion") << "Pursuit heading mode "
        << (enabled ? "ON" : "OFF") << Console::endl;
}

FLASHMEM bool Motion::isHeadingMode() const {
    return pursuit_controller.isHeadingMode();
}

FLASHMEM bool Motion::isPursuitWatchdogTripped() const {
    return pursuit_controller.watchdogTripped();
}
