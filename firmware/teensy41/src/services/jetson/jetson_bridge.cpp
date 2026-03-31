#include "jetson_bridge.h"
#include "os/os.h"
#include "os/console.h"
#include "os/commands.h"
#include "services/motion/motion.h"
#include "services/safety/safety.h"
#include "services/chrono/chrono.h"
#include "services/lidar/occupancy.h"
#include "services/intercom/intercom.h"
#include "services/intercom/comUtilities.h"
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
    m_bridgeSource    = BridgeSource::INTERCOM;  // Unknown until first ping
    _bridgeBufLen     = 0;

    // Always start BRIDGE_SERIAL — it is USB-CDC and idempotent.
    // Runtime mode (USB vs XBee) is determined when the first ping/frame arrives.
    BRIDGE_SERIAL.begin(BRIDGE_BAUDRATE);
    // Drain any stale RX bytes left from a previous session
    while (BRIDGE_SERIAL.available()) BRIDGE_SERIAL.read();

    Console::info("JetsonBridge") << "Enabled — waiting for transport (USB or XBee)." << Console::endl;
}

void JetsonBridge::disable() {
    Service::disable();
}

// ─────────────────────────────────────────────────────────────────────────────
//  run() — called each OS loop iteration
// ─────────────────────────────────────────────────────────────────────────────

void JetsonBridge::run() {
    if (!enabled()) return;

    // Always poll BRIDGE_SERIAL — if USB is connected it reads frames,
    // if nothing is connected it costs nothing (available() returns 0).
    _readBridgeSerial();

    _checkWatchdog();

    // Drain queued telemetry periodically when USB buffer has space
    if (millis() - m_lastTelDrainMs > TEL_DRAIN_PERIOD_MS) {
        while (!_telQueue.empty() && BRIDGE_SERIAL.availableForWrite() >= 128) {
            String frame = _telQueue.front();
            _telQueue.pop_front();
            BRIDGE_SERIAL.print(frame);
            BRIDGE_SERIAL.write('\n');
        }
        m_lastTelDrainMs = millis();
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

void JetsonBridge::handleRequest(Request& req) {
    const String& cmd = req.getContent();
    // NOTE: Console::trace intentionally omitted here — this is a hot path
    // called at up to 10 Hz.  Each trace call allocates a temporary String on
    // the Arduino heap which, over time, fragments free memory and triggers the
    // watchdog.  Use Console::info selectively only for state-change events.

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

    // Include last move stats for remote debug visibility
    Motion::MoveStats stats = motion.getLastStats();
    tel += ",dur=";   tel += String(stats.durationMs);
    tel += ",dist=";  tel += String(stats.traveledMm, 1);
    tel += ",stall="; tel += (stats.stalled ? "1" : "0");

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
    if (m_telPos)    _pushFrame("TEL:pos:"    + _buildPositionTel());

    // Enriched motion telemetry: state + target + feedrate + stall info
    if (m_telMotion) {
        String motionTel = "TEL:motion:";
        if (motion.isMoving()) {
            motionTel += "RUNNING";
            Vec3 tgt = motion.getAbsTarget();
            motionTel += ",tx=";  motionTel += String(tgt.x, 1);
            motionTel += ",ty=";  motionTel += String(tgt.y, 1);
            motionTel += ",dist="; motionTel += String(motion.getTargetDistance(), 1);
        } else {
            motionTel += "IDLE";
        }
        motionTel += ",feed="; motionTel += String(motion.getFeedrate(), 2);
        _pushFrame(motionTel);
    }

    if (m_telSafety) _pushFrame(String("TEL:safety:") + (safety.obstacleDetected() ? "1" : "0"));
    if (m_telChrono) _pushFrame("TEL:chrono:" + String(chrono.getElapsedTime()));

    // FW-006: only push mask when it changed
    if (m_maskDirty) {
        String mask = "TEL:mask:pos=";
        mask += (m_telPos    ? "1" : "0"); mask += ",motion=";
        mask += (m_telMotion ? "1" : "0"); mask += ",safety=";
        mask += (m_telSafety ? "1" : "0"); mask += ",chrono=";
        mask += (m_telChrono ? "1" : "0"); mask += ",occ=";
        mask += (m_telOcc    ? "1" : "0");
        _pushFrame(mask);
        m_maskDirty = false;
    }
}

void JetsonBridge::pushOccupancy() {
    if (!m_telOcc) return;
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
    // Append CRC8 to match the wire protocol Python expects:
    //   TEL:pos:x=100,y=200|<crc>\n
    // Without CRC, parse_frame() in protocol.py rejects the frame.
    uint8_t crc = CRC8.smbus((const uint8_t*)msg.c_str(), msg.length());
    String framed = msg + "|" + String((int)crc);

    if (m_bridgeSource == BridgeSource::USB) {
        // Guard against a full TX FIFO: a blocking write here would stall the
        // main loop for milliseconds and eventually trip the hardware WDT.
        // If buffer is full, queue the frame instead of silently dropping it.
        int needed = (int)(framed.length() + 1);  // +1 for '\n'
        if (BRIDGE_SERIAL.availableForWrite() >= needed) {
            BRIDGE_SERIAL.print(framed);
            BRIDGE_SERIAL.write('\n');
        } else {
            // Queue the fully-framed message for retry when buffer is available
            if (_telQueue.size() < TEL_QUEUE_MAX) {
                _telQueue.push_back(framed);
            } else {
                // Drop oldest frame to make room (sacrifice old data for fresh)
                _telQueue.pop_front();
                _telQueue.push_back(framed);
            }
        }
    } else {
        // XBee / Jetson path — relay over Intercom (Serial1 @ 31250).
        // Intercom has its own framing, no CRC needed here.
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
void JetsonBridge::_readBridgeSerial() {
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
                    Request req(id, content, BridgeSource::USB);
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

    // Emit telemetry so remote side sees the fallback event
    String tel = "TEL:error:fallback=";
    tel += String(idx);
    _pushFrame(tel);

    if (idx < 5 && m_fallbacks[idx]) {
        m_fallbacks[idx]();
    } else {
        // Ultimate safety: stop everything
        motion.cancel();
        motion.disengage();
    }
}
