#include "jetson_bridge.h"
#include "os/os.h"
#include "os/console.h"
#include "os/commands.h"
#include "services/motion/motion.h"
#include "services/safety/safety.h"
#include "services/chrono/chrono.h"
#include "services/lidar/lidar.h"
#include "services/lidar/occupancy.h"
#include "services/actuators/actuators.h"
#include "services/vision/vision.h"
#include "services/localisation/localisation.h"
#include "services/intercom/intercom.h"
#include "services/intercom/comUtilities.h"
#include "services/ihm/ihm.h"
#include "config/settings.h"
#include "config/runtime_config.h"
#include "program/block_registry.h"

SINGLETON_INSTANTIATE(JetsonBridge, jetsonBridge)

// Runtime-switchable bridge serial port (defaults to XBee = Serial3).
// JetsonBridge::_readPort() updates this pointer when auto-detection
// determines which port is active (USB-CDC or XBee).
Stream* g_bridgeSerial = &BRIDGE_XBEE;

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

JetsonBridge::JetsonBridge() : Service(ID_JETSON) {}

FLASHMEM void JetsonBridge::attach() {
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

FLASHMEM void JetsonBridge::enable() {
    Service::enable();
    m_lastHeartbeatMs = millis();  // Give a grace period on boot
    m_bridgeSource    = BridgeSource::INTERCOM;  // Unknown until first ping
    m_bridgeDetected  = false;
    _bridgeBufLen     = 0;
    _wiredBufLen      = 0;

    // Start both candidate bridge ports.
    // Serial (USB-CDC) is always active on Teensy 4.1, begin() is idempotent.
    // Serial3 (XBee) needs explicit begin at BRIDGE_BAUDRATE.
    BRIDGE_USB.begin(BRIDGE_BAUDRATE);
    BRIDGE_XBEE.begin(BRIDGE_BAUDRATE);

    // Drain any stale RX bytes left from a previous session
    while (BRIDGE_USB.available())  BRIDGE_USB.read();
    while (BRIDGE_XBEE.available()) BRIDGE_XBEE.read();

    // Default: telemetry goes to XBee until detection picks a port.
    g_bridgeSerial = &BRIDGE_XBEE;

    Console::info("JetsonBridge") << "Enabled — listening on USB + XBee for auto-detect." << Console::endl;
}

FLASHMEM void JetsonBridge::disable() {
    Service::disable();
}

// ─────────────────────────────────────────────────────────────────────────────
//  run() — called each OS loop iteration
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::run() {
    if (!enabled()) return;

    // PR-1: Flush any pending command reply (reply queue from request.cpp)
    Request::flushPendingReply();

    // PR-2: Retransmit DONE event until host acknowledges
    if (m_donePending && millis() - m_doneRetryMs > DONE_RETRY_MS) {
        _pushFrame(m_lastDoneBuf);
        m_doneRetryMs = millis();
    }

    // Always poll BRIDGE_SERIAL — if USB is connected it reads frames,
    // if nothing is connected it costs nothing (available() returns 0).
    _readBridgeSerial();

    _checkWatchdog();

    // JB-1: Drain ring buffer every run() call (was: every 10ms timer)
    while (_tqCount > 0) {
        int flen = (int)strlen(_tqPool[_tqHead]);
        if (BRIDGE_SERIAL.availableForWrite() < flen + 1) break;
        BRIDGE_SERIAL.write((const uint8_t*)_tqPool[_tqHead], flen);
        BRIDGE_SERIAL.write('\n');
        _tqHead = (_tqHead + 1) % TQUEUE_SLOTS;
        --_tqCount;
    }

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

FLASHMEM void JetsonBridge::handleRequest(Request& req) {
    // IC-3: Create one String from content for dispatch — this is acceptable since it's
    // only one allocation per incoming command (not per telemetry tick).
    const String cmd = String(req.getContent());
    // NOTE: Console::trace intentionally omitted here — this is a hot path
    // called at up to 10 Hz.  Each trace call allocates a temporary String on
    // the Arduino heap which, over time, fragments free memory and triggers the
    // watchdog.  Use Console::info selectively only for state-change events.

    // ── Heartbeat (PR-3: tunable timeout via hb(ms)) ──────────────────────
    if (cmd == "hb" || cmd.startsWith("hb(")) {
        m_lastHeartbeatMs = millis();
        if (cmd.startsWith("hb(")) {
            // PR-3: host declares its heartbeat tolerance
            unsigned long ms = (unsigned long)cmd.substring(3, cmd.length() - 1).toInt();
            if (ms >= 1000 && ms <= 30000) m_heartbeatTimeoutMs = ms;
        }
        if (!m_jetsonConnected) {
            m_jetsonConnected = true;
            m_inFallback      = false;
            Console::info("JetsonBridge") << "Jetson connected (timeout=" << (int)m_heartbeatTimeoutMs << "ms)" << Console::endl;
        }
        req.reply("ok");
        return;
    }

    // ── Acknowledge DONE event (PR-2) ─────────────────────────────────────────
    if (cmd == "ack_done") {
        m_donePending = false;
        req.reply("ok");
        return;
    }

    // ── Cancel in-flight motion ───────────────────────────────────────────────
    // forceCancel() immediately sets _isMoving=false and stops hardware.
    // If a motion command was pending (execute() on the holOS side is waiting
    // for DONE), we send DONE:fail now so it unblocks immediately rather than
    // waiting for the slow CANCELING→onCanceled() deceleration path.
    if (cmd == "cancel") {
        if (m_motionPending) {
            _replyMotionDone(false);
            m_motionPending = false;
        }
        motion.forceCancel();
        req.reply("ok");
        return;
    }

    // ── State sync — call after any reconnection (PR-4) ───────────────────────
    if (cmd == "sync") {
        Vec3        pos    = motion.estimatedPosition();
        const char* mstate = m_donePending
                             ? (m_lastDoneWasOk ? "DONE_OK" : "DONE_FAIL")
                             : (motion.isMoving() ? "RUNNING" : "IDLE");
        char buf[128];
        snprintf(buf, sizeof(buf), "%s,%.1f,%.1f,%.2f", mstate, pos.x, pos.y, pos.c);
        req.reply(buf);
        return;
    }

    // ── Full service health snapshot ──────────────────────────────────────────
    // Returns compact key=val pairs for all 13 services + runtime flags.
    // Format: "mo=1,mv=0,sa=0,ob=0,ch=0,el=0,ac=1,li=1,ic=1,ic_ok=1,jt=1,jt_ok=1,vi=0,lo=1"
    if (cmd == "health") {
        // strat: 0 = séquentielle (dumb T41-only), 1 = intelligente (Jetson-based)
        int strat = ihm.strategySwitch.getState() ? 1 : 0;
        char buf[Request::CONTENT_MAX];
        snprintf(buf, sizeof(buf),
            "mo=%d,mv=%d,sa=%d,ob=%d,ch=%d,el=%ld,"
            "ac=%d,li=%d,ic=%d,ic_ok=%d,jt=%d,jt_ok=%d,vi=%d,lo=%d,"
            "strat=%d,paused=%d",
            motion.enabled()              ? 1 : 0,
            motion.isMoving()             ? 1 : 0,
            safety.enabled()              ? 1 : 0,
            safety.obstacleDetected()     ? 1 : 0,
            chrono.enabled()              ? 1 : 0,
            chrono.getElapsedTime(),
            actuators.enabled()           ? 1 : 0,
            lidar.enabled()               ? 1 : 0,
            intercom.enabled()            ? 1 : 0,
            intercom.isConnected()        ? 1 : 0,
            jetsonBridge.enabled()        ? 1 : 0,
            jetsonBridge.jetsonConnected()? 1 : 0,
            vision.enabled()              ? 1 : 0,
            localisation.enabled()        ? 1 : 0,
            strat,
            m_matchPaused                 ? 1 : 0);
        req.reply(buf);
        return;
    }

    // ── Telemetry snapshot ────────────────────────────────────────────────────
    if (cmd == "tel") {
        char tel_buf[64];
        Vec3 pos = motion.estimatedPosition();
        snprintf(tel_buf, sizeof(tel_buf), "x=%.1f,y=%.1f,theta=%.4f",
                 pos.x, pos.y, pos.c);
        req.reply(tel_buf);
        return;
    }

    // ── Occupancy map (legacy full bitmap) ───────────────────────────────────
    if (cmd == "occ") {
        req.reply(occupancy.compress().c_str());
        return;
    }

    // ── Deploy static map to T40 ──────────────────────────────────────────────
    // Command format: setStaticMap(HEX_STRING)
    // Jetson sends the packed hex bitmap; we relay it to T40 via intercom.
    if (cmd.startsWith("setStaticMap(")) {
        int open  = cmd.indexOf('(');
        int close = cmd.lastIndexOf(')');
        if (open >= 0 && close > open) {
            String hex = cmd.substring(open + 1, close);
            // Forward to T40 as sM(hex)
            String ic_cmd = "sM(" + hex + ")";
            intercom.sendRequest(ic_cmd.c_str(), 3000);
            Console::info("JetsonBridge") << "Static map deployed to T40 (" << hex.length() << " hex chars)" << Console::endl;
        }
        req.reply("ok");
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

    // ── Telemetry channel control — tel(channel, 0|1) ─────────────────────────
    // Examples: tel(occ,0) tel(pos,1) tel(chrono,0)
    if (cmd.startsWith("tel(")) {
        int open  = cmd.indexOf('(');
        int comma = cmd.indexOf(',');
        int close = cmd.indexOf(')');
        if (open >= 0 && comma > open && close > comma) {
            String chan = cmd.substring(open + 1, comma);
            chan.trim();
            bool en = cmd.substring(comma + 1, close).toInt() != 0;
            if      (chan == "pos")    m_telPos    = en;
            else if (chan == "motion") m_telMotion = en;
            else if (chan == "safety") m_telSafety = en;
            else if (chan == "chrono") m_telChrono = en;
            else if (chan == "occ")    m_telOcc    = en;
            Console::info("JetsonBridge") << "tel " << chan << "=" << (en?"1":"0") << Console::endl;
            m_maskDirty = true;  // FW-006: mark mask dirty so it gets pushed once
            req.reply("ok");
        } else {
            req.reply("err:bad_args");
        }
        return;
    }

    // ── Runtime config (SD card) — handled directly, NOT via interpreter ──────
    // These bypass os.execute() so that (a) the reply carries the actual data
    // instead of a generic "ok", and (b) dotted config keys like "servo.CA.0.min"
    // are never mangled by the expression evaluator.
    if (cmd == "cfg_list") {
        char buf[Request::CONTENT_MAX];
        RuntimeConfig::serialize(buf, sizeof(buf));
        req.reply(buf);
        return;
    }

    if (cmd.startsWith("cfg_set(")) {
        // Format: cfg_set(key,value)
        int open  = cmd.indexOf('(');
        int comma = cmd.indexOf(',');
        int close = cmd.lastIndexOf(')');
        if (open >= 0 && comma > open && close > comma) {
            String key = cmd.substring(open + 1, comma);
            String val = cmd.substring(comma + 1, close);
            key.trim(); val.trim();
            bool ok = RuntimeConfig::set(key.c_str(), val.c_str());
            req.reply(ok ? "ok" : "err:store_full");
        } else {
            req.reply("err:bad_args");
        }
        return;
    }

    if (cmd == "cfg_save") {
        bool ok = RuntimeConfig::save();
        req.reply(ok ? "ok" : "err:save_failed");
        return;
    }

    if (cmd == "cfg_load") {
        RuntimeConfig::load();
        req.reply("ok");
        return;
    }

    // ── Match control ────────────────────────────────────────────────────────
    // match_start: triggers the same sequence as pulling the physical starter.
    //   Sets a flag consumed by programManual → os.start() → programAuto.
    //   No arming required (webapp start bypasses the arm phase).
    if (cmd == "match_start") {
        m_remoteStartRequested = true;
        m_matchPaused = false;
        Console::info("JetsonBridge") << "Remote match start requested" << Console::endl;
        req.reply("ok");
        return;
    }

    // match_stop: pauses the current program, cancels in-flight motion.
    //   The cancelled target is saved in motion for later replay.
    if (cmd == "match_stop") {
        m_matchPaused = true;
        if (m_motionPending) {
            _replyMotionDone(false);
            m_motionPending = false;
        }
        motion.forceCancel();
        Console::info("JetsonBridge") << "Remote match stop (paused)" << Console::endl;
        req.reply("ok");
        return;
    }

    // match_resume: clear pause flag so programAuto loop continues
    if (cmd == "match_resume") {
        m_matchPaused = false;
        Console::info("JetsonBridge") << "Match resumed" << Console::endl;
        req.reply("ok");
        return;
    }

    // ── Block registry commands ──────────────────────────────────────────────

    // blocks_list: returns all registered C++ blocks and their state.
    //   Format: "name=priority,score,estimatedMs,done;..."
    if (cmd == "blocks_list") {
        char buf[Request::CONTENT_MAX];
        BlockRegistry::instance().serialize(buf, sizeof(buf));
        req.reply(buf);
        return;
    }

    // run_block(name): execute a registered C++ block by name.
    //   Returns "SUCCESS" or "FAILED".
    if (cmd.startsWith("run_block(")) {
        int open  = cmd.indexOf('(');
        int close = cmd.lastIndexOf(')');
        if (open >= 0 && close > open) {
            String name = cmd.substring(open + 1, close);
            name.trim();
            BlockResult r = BlockRegistry::instance().execute(name.c_str());
            req.reply(r == BlockResult::SUCCESS ? "SUCCESS" : "FAILED");
        } else {
            req.reply("err:bad_args");
        }
        return;
    }

    // block_done(name): mark a block as completed (e.g. Jetson finished it).
    //   Used for fallback awareness — skips done blocks in embedded match().
    if (cmd.startsWith("block_done(")) {
        int open  = cmd.indexOf('(');
        int close = cmd.lastIndexOf(')');
        if (open >= 0 && close > open) {
            String name = cmd.substring(open + 1, close);
            name.trim();
            bool ok = BlockRegistry::instance().markDone(name.c_str());
            req.reply(ok ? "ok" : "err:not_found");
        } else {
            req.reply("err:bad_args");
        }
        return;
    }

    // blocks_reset: clear all done flags (e.g. before a new match)
    if (cmd == "blocks_reset") {
        BlockRegistry::instance().resetAll();
        req.reply("ok");
        return;
    }

    // ── Motion commands (long-running — reply AFTER completion) ───────────────
    // For chained commands like "via(x,y);via(x,y);go(x,y)", the LAST
    // command in the chain determines whether this is a motion command.
    // Via points are always intermediate; the final go/turn/align is what
    // actually triggers motion and needs DONE tracking.
    String lastCmd = cmd;
    {
        int lastSemi = cmd.lastIndexOf(';');
        if (lastSemi >= 0) {
            lastCmd = cmd.substring(lastSemi + 1);
            lastCmd.trim();
        }
    }

    if (lastCmd.startsWith("go(")       ||
        lastCmd.startsWith("go_coc(")   ||
        lastCmd.startsWith("goPolar(")  ||
        lastCmd.startsWith("turn(")     ||
        lastCmd.startsWith("align(")    ||
        lastCmd.startsWith("goAlign(")  ||
        lastCmd.startsWith("move(")) {

        // FW-004: reply fail to old pending motion before preempting
        if (m_motionPending) {
            _replyMotionDone(false);  // send DONE:fail for the old request
            m_motionPending = false;
            motion.forceCancel();
        }

        m_motionRequestId = req.ID();
        m_motionPending   = true;

        // Send immediate ACK so holOS unblocks its initial reply wait.
        // The DONE telemetry (TEL:motion:DONE:...) is sent separately when
        // motion completes — holOS waits for that event after receiving this reply.
        req.reply("ok");

        // Execute command through the standard interpreter (async)
        os.execute(const_cast<String&>(cmd));
        return;
    }

    // ── All other commands: execute and reply ok ──────────────────────────────
    _executeCommand(cmd, req);
}

// ─────────────────────────────────────────────────────────────────────────────
//  _executeCommand — execute via OS command interpreter, reply immediately
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::_executeCommand(const String& cmd, Request& req) {
    String mutable_cmd = cmd;
    os.execute(mutable_cmd);
    req.reply("ok");
}

// ─────────────────────────────────────────────────────────────────────────────
//  _replyMotionDone — send deferred reply for a completed motion command
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::_replyMotionDone(bool success) {
    Motion::MoveStats stats = motion.getLastStats();

    // PR-2: build into persistent buffer for retransmit until ack_done
    snprintf(m_lastDoneBuf, sizeof(m_lastDoneBuf),
        "TEL:motion:DONE:%s,dur=%lu,dist=%.1f,stall=%d",
        success ? "ok" : "fail",
        (unsigned long)stats.durationMs,
        stats.traveledMm,
        stats.stalled ? 1 : 0);

    m_donePending   = true;
    m_lastDoneWasOk = success;
    m_doneRetryMs   = millis();

    _pushFrame(m_lastDoneBuf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Watchdog
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::_checkWatchdog() {
    if (!m_jetsonConnected) return;

    if (millis() - m_lastHeartbeatMs > m_heartbeatTimeoutMs) {
        Console::warn("JetsonBridge")
            << "Jetson heartbeat timeout!" << Console::endl;
        m_jetsonConnected = false;

        // Fallback only makes sense during an active match (AUTO state).
        // In MANUAL/MANUAL_PROGRAM the match hasn't started — triggering a
        // blocking Mission::run() here would freeze the entire OS and prevent
        // programManual() (and consumeRemoteStart()) from ever executing.
        if (os.getState() == OS::AUTO) {
            Console::warn("JetsonBridge")
                << "Match running — activating fallback strategy." << Console::endl;
            triggerFallback(FallbackID::CUSTOM_1);
        } else {
            Console::info("JetsonBridge")
                << "No active match — skipping fallback." << Console::endl;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Remote control state
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM bool JetsonBridge::isRemoteControlled() const {
    return m_jetsonConnected;
}

FLASHMEM bool JetsonBridge::jetsonConnected() const {
    return m_jetsonConnected;
}

FLASHMEM bool JetsonBridge::consumeRemoteStart() {
    if (m_remoteStartRequested) {
        m_remoteStartRequested = false;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::pushTelemetry() {
    char buf[128];

    if (m_telPos) {
        Vec3 pos = motion.estimatedPosition();
        snprintf(buf, sizeof(buf), "TEL:pos:x=%.1f,y=%.1f,theta=%.4f",
            pos.x, pos.y, pos.c);
        _pushFrame(buf);
    }

    if (m_telMotion) {
        if (motion.isMoving()) {
            Vec3 tgt = motion.getAbsTarget();
            snprintf(buf, sizeof(buf), "TEL:motion:RUNNING,tx=%.1f,ty=%.1f,dist=%.1f,feed=%.2f",
                tgt.x, tgt.y, motion.getTargetDistance(), motion.getFeedrate());
        } else {
            snprintf(buf, sizeof(buf), "TEL:motion:IDLE,feed=%.2f", motion.getFeedrate());
        }
        _pushFrame(buf);
    }

    if (m_telSafety) {
        snprintf(buf, sizeof(buf), "TEL:safety:%d", safety.obstacleDetected() ? 1 : 0);
        _pushFrame(buf);
    }

    if (m_telChrono) {
        snprintf(buf, sizeof(buf), "TEL:chrono:%lu", (unsigned long)chrono.getElapsedTime());
        _pushFrame(buf);
    }

    // FW-006: only push mask when dirty
    if (m_maskDirty) {
        snprintf(buf, sizeof(buf), "TEL:mask:pos=%d,motion=%d,safety=%d,chrono=%d,occ=%d",
            m_telPos ? 1 : 0, m_telMotion ? 1 : 0,
            m_telSafety ? 1 : 0, m_telChrono ? 1 : 0, m_telOcc ? 1 : 0);
        _pushFrame(buf);
        m_maskDirty = false;
    }
}

FLASHMEM void JetsonBridge::pushOccupancy() {
    if (!m_telOcc) return;
    // Build sparse "gx,gy;gx,gy;…" from the dynamic-only occupancy map.
    // The map is updated at ~5 Hz by Lidar::onOccDynResponse().
    char buf[TQUEUE_FRAME];
    int  pos = 0;
    bool first = true;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "TEL:occ_dyn:");
    for (int gx = 0; gx < GRID_WIDTH && pos < (int)sizeof(buf) - 8; ++gx) {
        for (int gy = 0; gy < GRID_HEIGHT && pos < (int)sizeof(buf) - 8; ++gy) {
            if (occupancy.isCellOccupied(gx, gy)) {
                if (!first) buf[pos++] = ';';
                pos += snprintf(buf + pos, sizeof(buf) - pos, "%d,%d", gx, gy);
                first = false;
            }
        }
    }
    _pushFrame(buf);
}

FLASHMEM void JetsonBridge::_pushFrame(const char* msg) {
    // Compute CRC and build framed message into a stack buffer.
    // Buffer sized to TQUEUE_FRAME so occupancy frames always fit.
    char framed[TQUEUE_FRAME];
    uint8_t crc = CRC8.smbus((const uint8_t*)msg, strlen(msg));
    int n = snprintf(framed, sizeof(framed), "%s|%d", msg, (int)crc);
    if (n <= 0 || n >= (int)sizeof(framed)) return;  // truncated — drop

    if (m_bridgeSource == BridgeSource::USB) {
        int needed = n + 1;  // +1 for '\n'
        if (BRIDGE_SERIAL.availableForWrite() >= needed) {
            BRIDGE_SERIAL.write((const uint8_t*)framed, n);
            BRIDGE_SERIAL.write('\n');
        } else {
            // JB-1: push to ring buffer — drop oldest if full
            strncpy(_tqPool[_tqTail], framed, TQUEUE_FRAME - 1);
            _tqPool[_tqTail][TQUEUE_FRAME - 1] = '\0';
            _tqTail = (_tqTail + 1) % TQUEUE_SLOTS;
            if (_tqCount < TQUEUE_SLOTS) {
                ++_tqCount;
            } else {
                // Full: advance head (drop oldest)
                _tqHead = (_tqHead + 1) % TQUEUE_SLOTS;
            }
        }
    } else {
        // XBee / Intercom path — Intercom adds its own framing
        intercom.sendMessage(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _readBridgeSerial — auto-detect & parse bridge frames from USB-CDC or XBee
//
//  Before detection: polls BOTH Serial (USB-CDC) and Serial3 (XBee).
//  After detection:  polls only the active port (the one that won).
//
//  Protocol (same as Intercom): "{id}:{content}|{crc}\n"
//  Replies go back via Request::reply() → BRIDGE_SERIAL (see request.cpp).
// ─────────────────────────────────────────────────────────────────────────────
FLASHMEM void JetsonBridge::_readBridgeSerial() {
    if (!m_bridgeDetected) {
        // Discovery phase: poll both ports
        _readPort(BRIDGE_USB,  _wiredBuf,  _wiredBufLen);
        _readPort(BRIDGE_XBEE, _bridgeBuf, _bridgeBufLen);
    } else {
        // Active phase: poll only the detected port
        if (g_bridgeSerial == (Stream*)&BRIDGE_USB) {
            _readPort(BRIDGE_USB, _wiredBuf, _wiredBufLen);
        } else {
            _readPort(BRIDGE_XBEE, _bridgeBuf, _bridgeBufLen);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _readPort — parse one serial port for ping/pong handshake or CRC frames
//
//  Uses a caller-provided static char buffer to avoid heap fragmentation.
//  On first valid ping or CRC frame, sets g_bridgeSerial and m_bridgeDetected
//  so subsequent calls only poll the winning port.
// ─────────────────────────────────────────────────────────────────────────────
FLASHMEM void JetsonBridge::_readPort(Stream& port, char* buf, uint16_t& bufLen) {
    while (port.available()) {
        char c = (char)port.read();

        if (c == '\n' || c == '\r') {
            if (bufLen == 0) continue;  // blank line

            buf[bufLen] = '\0';

            // ── Connection handshake ─────────────────────────────────────────
            if (strcmp(buf, "ping") == 0) {
                g_bridgeSerial  = &port;
                m_bridgeSource  = BridgeSource::USB;
                m_bridgeDetected = true;
                bool isUsb = (&port == (Stream*)&BRIDGE_USB);
                port.print(isUsb ? "pong:usb\n" : "pong:xbee\n");
                Console::info("JetsonBridge")
                    << "Bridge detected on "
                    << (isUsb ? "USB-CDC" : "XBee")
                    << Console::endl;
                bufLen = 0;
                continue;
            }
            if (strncmp(buf, "pong", 4) == 0) {
                // Stray echo — ignore (matches "pong", "pong:usb", "pong:xbee")
                bufLen = 0;
                continue;
            }

            // ── Framed protocol: "{id}:{content}|{crc}" ─────────────────────
            char* sep    = strchr (buf, ':');
            char* crcSep = strrchr(buf, '|');

            if (sep && crcSep && crcSep > sep) {
                int crcVal = atoi(crcSep + 1);

                // CRC covers all bytes before '|'
                uint8_t computed = CRC8.smbus(
                    (uint8_t*)buf,
                    (size_t)(crcSep - buf));

                if (computed == (uint8_t)crcVal) {
                    // Lock in this port as the active bridge
                    if (!m_bridgeDetected) {
                        g_bridgeSerial   = &port;
                        m_bridgeDetected = true;
                        Console::info("JetsonBridge")
                            << "Bridge detected on "
                            << ((&port == (Stream*)&BRIDGE_USB) ? "USB-CDC" : "XBee")
                            << " (framed)"
                            << Console::endl;
                    }

                    // Split in-place to extract id and content
                    *sep    = '\0';
                    *crcSep = '\0';
                    int    id      = atoi(buf);
                    String content = String(sep + 1);   // single unavoidable alloc
                    *sep    = ':';   // restore (good practice)
                    *crcSep = '|';

                    m_bridgeSource = BridgeSource::USB;
                    Request req(id, content.c_str(), BridgeSource::USB);
                    handleRequest(req);
                } else {
                    Console::warn("JetsonBridge") << "Bad CRC on bridge line" << Console::endl;
                }
            } else {
                // Not a valid frame — ignore silently during discovery,
                // warn only when bridge is active (likely garbled data).
                if (m_bridgeDetected) {
                    Console::warn("JetsonBridge") << "Unparseable bridge frame" << Console::endl;
                }
            }

            bufLen = 0;

        } else if (c != '\r') {
            // Append character — hard cap at 511 to leave room for null terminator
            if (bufLen < 511) buf[bufLen++] = c;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry channel runtime control
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::setTelemetry(uint8_t channel, bool on) {
    switch (channel) {
        case 0: m_telPos    = on; break;
        case 1: m_telMotion = on; break;
        case 2: m_telSafety = on; break;
        case 3: m_telChrono = on; break;
        case 4: m_telOcc    = on; break;
        default: break;
    }
    m_maskDirty = true;  // push updated mask to holOS on next cycle
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fallback programs
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void JetsonBridge::registerFallback(FallbackID id, fallback_fn fn) {
    uint8_t idx = static_cast<uint8_t>(id);
    if (idx < 5) {
        m_fallbacks[idx] = fn;
    }
}

FLASHMEM void JetsonBridge::triggerFallback(FallbackID id) {
    uint8_t idx = static_cast<uint8_t>(id);
    m_inFallback = true;
    Console::warn("JetsonBridge") << "Fallback triggered: " << idx << Console::endl;

    // Emit telemetry so remote side sees the fallback event (JB-2: no String alloc)
    char buf[48];
    snprintf(buf, sizeof(buf), "TEL:error:fallback=%d", (int)idx);
    _pushFrame(buf);

    if (idx < 5 && m_fallbacks[idx]) {
        m_fallbacks[idx]();
    } else {
        // Ultimate safety: stop everything
        motion.cancel();
        motion.disengage();
    }
}
