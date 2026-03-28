#include "jetson_bridge.h"
#include "os/os.h"
#include "os/console.h"
#include "os/commands.h"
#include "services/motion/motion.h"
#include "services/safety/safety.h"
#include "services/chrono/chrono.h"
#include "services/lidar/occupancy.h"
#include "services/intercom/intercom.h"
#include "config/settings.h"

SINGLETON_INSTANTIATE(JetsonBridge, jetsonBridge)

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

JetsonBridge::JetsonBridge() : Service(ID_JETSON) {}

void JetsonBridge::attach() {
    // Register default fallback : stop motors
    registerFallback(FallbackID::STOP, []() {
        motion.cancel();
        motion.disengage();
    });

    // Default fallback : wait in place (motors engaged but no motion)
    registerFallback(FallbackID::WAIT_IN_PLACE, []() {
        motion.cancel();
    });

    Console::info("JetsonBridge") << "Attached." << Console::endl;
}

void JetsonBridge::enable() {
    Service::enable();
    m_lastHeartbeatMs = millis();  // Give a grace period on boot
    Console::info("JetsonBridge") << "Enabled — waiting for Jetson." << Console::endl;
}

void JetsonBridge::disable() {
    Service::disable();
}

// ─────────────────────────────────────────────────────────────────────────────
//  run() — called each OS loop iteration
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::run() {
    if (!enabled()) return;

    _checkWatchdog();

    // Push telemetry at fixed rate
    if (millis() - m_lastTelPushMs > TEL_PERIOD_MS) {
        pushTelemetry();
        m_lastTelPushMs = millis();
    }

    // Push occupancy map at lower rate
    if (millis() - m_lastOccPushMs > OCC_PERIOD_MS) {
        pushOccupancy();
        m_lastOccPushMs = millis();
    }

    // If a motion command is pending, check if motion is done
    if (m_motionPending && !motion.isMoving()) {
        bool ok = motion.wasSuccessful();
        _replyMotionDone(ok);
        m_motionPending = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  handleRequest — called from onIntercomRequest in routines.cpp
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::handleRequest(Request& req) {
    const String& cmd = req.getContent();
    Console::trace("JetsonBridge") << "CMD: " << cmd << Console::endl;

    // ── Heartbeat ─────────────────────────────────────────────────────────────
    if (cmd == "hb") {
        m_lastHeartbeatMs = millis();
        if (!m_jetsonConnected) {
            m_jetsonConnected = true;
            m_inFallback      = false;
            Console::info("JetsonBridge") << "Jetson connected." << Console::endl;
        }
        req.reply("ok");
        return;
    }

    // ── Telemetry snapshot ────────────────────────────────────────────────────
    if (cmd == "tel") {
        req.reply(_buildPositionTel());
        return;
    }

    // ── Occupancy map ─────────────────────────────────────────────────────────
    if (cmd == "occ") {
        req.reply(occupancy.compress());
        return;
    }

    // ── Trigger fallback by ID ────────────────────────────────────────────────
    if (cmd.startsWith("fb(")) {
        int id = cmd.substring(3, cmd.length() - 1).toInt();
        triggerFallback(static_cast<FallbackID>(id));
        req.reply("ok");
        return;
    }

    // ── Enable/disable safety ─────────────────────────────────────────────────
    if (cmd == "enable(SAFETY)") {
        safety.enable();
        req.reply("ok");
        return;
    }
    if (cmd == "disable(SAFETY)") {
        safety.disable();
        req.reply("ok");
        return;
    }

    // ── Motion commands (long-running — reply AFTER completion) ───────────────
    if (cmd.startsWith("go(")       ||
        cmd.startsWith("goPolar(")  ||
        cmd.startsWith("turn(")     ||
        cmd.startsWith("align(")    ||
        cmd.startsWith("goAlign(")  ||
        cmd.startsWith("move(")) {

        if (m_motionPending) {
            // Preempt current motion
            motion.forceCancel();
        }

        m_motionRequestId = req.ID();
        m_motionPending   = true;

        // Execute command through the standard interpreter (async)
        os.execute(const_cast<String&>(cmd));
        // Reply is deferred — see run() / _replyMotionDone()
        // We do NOT call req.reply() here
        return;
    }

    // ── All other commands: execute and reply ok ──────────────────────────────
    _executeCommand(cmd, req);
}

// ─────────────────────────────────────────────────────────────────────────────
//  _executeCommand — execute via OS command interpreter, reply immediately
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::_executeCommand(const String& cmd, Request& req) {
    String mutable_cmd = cmd;
    os.execute(mutable_cmd);
    req.reply("ok");
}

// ─────────────────────────────────────────────────────────────────────────────
//  _replyMotionDone — send deferred reply for a completed motion command
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::_replyMotionDone(bool success) {
    // Push TEL:motion telemetry so Jetson's transport layer unblocks
    String tel = "TEL:motion:DONE:";
    tel += success ? "ok" : "fail";
    _pushFrame(tel);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Watchdog
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::_checkWatchdog() {
    if (!m_jetsonConnected) return;

    if (millis() - m_lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
        Console::warn("JetsonBridge")
            << "Jetson heartbeat timeout! Activating fallback." << Console::endl;
        m_jetsonConnected = false;
        triggerFallback(FallbackID::STOP);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Remote control state
// ─────────────────────────────────────────────────────────────────────────────

bool JetsonBridge::isRemoteControlled() const {
    return m_jetsonConnected;
}

bool JetsonBridge::jetsonConnected() const {
    return m_jetsonConnected;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::pushTelemetry() {
    // Position
    _pushFrame("TEL:pos:" + _buildPositionTel());

    // Motion state
    String motionState = motion.isMoving() ? "RUNNING" : "IDLE";
    _pushFrame("TEL:motion:" + motionState);

    // Safety
    _pushFrame(String("TEL:safety:") + (safety.obstacleDetected() ? "1" : "0"));

    // Chrono
    _pushFrame("TEL:chrono:" + String(chrono.getElapsedTime()));
}

void JetsonBridge::pushOccupancy() {
    String compressed = occupancy.compress();
    _pushFrame("TEL:occ:" + compressed);
}

String JetsonBridge::_buildPositionTel() const {
    Vec3 pos = motion.estimatedPosition();
    String s = "x=";
    s += String(pos.x, 1);
    s += ",y=";
    s += String(pos.y, 1);
    s += ",theta=";
    s += String(pos.c, 4);
    return s;
}

void JetsonBridge::_pushFrame(const String& msg) {
    intercom.sendMessage(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fallback programs
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::registerFallback(FallbackID id, fallback_fn fn) {
    uint8_t idx = static_cast<uint8_t>(id);
    if (idx < 5) {
        m_fallbacks[idx] = fn;
    }
}

void JetsonBridge::triggerFallback(FallbackID id) {
    uint8_t idx = static_cast<uint8_t>(id);
    m_inFallback = true;
    Console::warn("JetsonBridge") << "Fallback triggered: " << idx << Console::endl;

    if (idx < 5 && m_fallbacks[idx]) {
        m_fallbacks[idx]();
    } else {
        // Ultimate safety: stop everything
        motion.cancel();
        motion.disengage();
    }
}
