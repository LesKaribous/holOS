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
#include "config/settings.h"

SINGLETON_INSTANTIATE(JetsonBridge, jetsonBridge)

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
    _bridgeBufLen     = 0;

    // Always start BRIDGE_SERIAL — it is USB-CDC and idempotent.
    // Runtime mode (USB vs XBee) is determined when the first ping/frame arrives.
    BRIDGE_SERIAL.begin(BRIDGE_BAUDRATE);
    // Drain any stale RX bytes left from a previous session
    while (BRIDGE_SERIAL.available()) BRIDGE_SERIAL.read();

    Console::info("JetsonBridge") << "Enabled — waiting for transport (USB or XBee)." << Console::endl;
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
        char buf[Request::CONTENT_MAX];
        snprintf(buf, sizeof(buf),
            "mo=%d,mv=%d,sa=%d,ob=%d,ch=%d,el=%ld,"
            "ac=%d,li=%d,ic=%d,ic_ok=%d,jt=%d,jt_ok=%d,vi=%d,lo=%d",
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
            localisation.enabled()        ? 1 : 0);
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

    // ── Occupancy map ─────────────────────────────────────────────────────────
    if (cmd == "occ") {
        req.reply(occupancy.compress().c_str());
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

    // ── Motion commands (long-running — reply AFTER completion) ───────────────
    if (cmd.startsWith("go(")       ||
        cmd.startsWith("goPolar(")  ||
        cmd.startsWith("turn(")     ||
        cmd.startsWith("align(")    ||
        cmd.startsWith("goAlign(")  ||
        cmd.startsWith("move(")) {

        // FW-004: reply fail to old pending motion before preempting
        if (m_motionPending) {
            _replyMotionDone(false);  // send DONE:fail for the old request
            m_motionPending = false;
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
            << "Jetson heartbeat timeout! Activating fallback." << Console::endl;
        m_jetsonConnected = false;
        triggerFallback(FallbackID::CUSTOM_1);  // Execute SD mission strategy if loaded
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
    // occupancy.compress() returns a String — one unavoidable alloc until T4.0 API changes
    String compressed = occupancy.compress();
    char buf[128];
    snprintf(buf, sizeof(buf), "TEL:occ:%s", compressed.c_str());
    _pushFrame(buf);
}

FLASHMEM void JetsonBridge::_pushFrame(const char* msg) {
    // Compute CRC and build framed message into a stack buffer
    char framed[128];
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
//  _readBridgeSerial — parse incoming request frames from USB-CDC
//
//  Protocol (same as Intercom): "{id}:{content}|{crc}\n"
//  Replies go back via Request::reply() → BRIDGE_SERIAL (see request.cpp).
//
//  Uses a static char array (_bridgeBuf / _bridgeBufLen) instead of a String
//  to avoid heap fragmentation: the buffer is reused every line without any
//  malloc/free cycle.  Only one unavoidable String is created per valid frame
//  (for the content field of the Request object).
// ─────────────────────────────────────────────────────────────────────────────
FLASHMEM void JetsonBridge::_readBridgeSerial() {
    while (BRIDGE_SERIAL.available()) {
        char c = (char)BRIDGE_SERIAL.read();

        if (c == '\n' || c == '\r') {
            if (_bridgeBufLen == 0) continue;  // blank line

            _bridgeBuf[_bridgeBufLen] = '\0';

            // ── Connection handshake ─────────────────────────────────────────
            if (strcmp(_bridgeBuf, "ping") == 0) {
                m_bridgeSource = BridgeSource::USB;
                BRIDGE_SERIAL.print("pong\n");
                _bridgeBufLen = 0;
                continue;
            }
            if (strcmp(_bridgeBuf, "pong") == 0) {
                // Stray echo — ignore
                _bridgeBufLen = 0;
                continue;
            }

            // ── Framed protocol: "{id}:{content}|{crc}" ─────────────────────
            char* sep    = strchr (_bridgeBuf, ':');
            char* crcSep = strrchr(_bridgeBuf, '|');

            if (sep && crcSep && crcSep > sep) {
                int crcVal = atoi(crcSep + 1);

                // CRC covers all bytes before '|' (i.e., "{id}:{content}")
                uint8_t computed = CRC8.smbus(
                    (uint8_t*)_bridgeBuf,
                    (size_t)(crcSep - _bridgeBuf));

                if (computed == (uint8_t)crcVal) {
                    // Split in-place to extract id and content
                    *sep    = '\0';
                    *crcSep = '\0';
                    int    id      = atoi(_bridgeBuf);
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
                Console::warn("JetsonBridge") << "Unparseable bridge frame" << Console::endl;
            }

            _bridgeBufLen = 0;

        } else if (c != '\r') {
            // Append character — hard cap at 511 to leave room for null terminator
            if (_bridgeBufLen < 511) _bridgeBuf[_bridgeBufLen++] = c;
        }
    }
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
