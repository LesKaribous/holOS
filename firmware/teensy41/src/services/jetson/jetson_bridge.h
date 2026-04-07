#pragma once
#include "services/service.h"
#include "services/intercom/request.h"
#include "config/settings.h"
#include "utils/timer/timer.h"
#include <functional>

// ============================================================
//  JetsonBridge — Service de communication avec le Jetson.
//
//  Rôles :
//    1. Dispatch des commandes reçues du Jetson (via Intercom)
//       vers l'interpréteur de commandes existant.
//    2. Envoi de télémétrie proactive (position, état motion,
//       safety, chrono, occupancy map).
//    3. Watchdog : si le Jetson ne répond plus au heartbeat,
//       basculement vers un programme de fallback.
//    4. Gestion du flag "remote controlled" : tant que le
//       Jetson est connecté, la stratégie embarquée sur Teensy
//       est suspendue.
//
//  Utilisation dans routines.cpp :
//
//    // Boot :
//    os.attachService(&jetsonBridge);
//
//    // onIntercomRequest :
//    void onIntercomRequest(Request& req) {
//        jetsonBridge.handleRequest(req);
//    }
//
//    // Dans programAuto (loop pendant le match) :
//    if (!jetsonBridge.isRemoteControlled()) {
//        match();  // fallback embarqué
//    }
// ============================================================

// Fallback IDs
enum class FallbackID : uint8_t {
    STOP           = 0,  // Arrêt moteurs (défaut)
    WAIT_IN_PLACE  = 1,  // Attente sur place
    RETURN_TO_BASE = 2,  // Retour zone départ
    CUSTOM_1       = 3,
    CUSTOM_2       = 4,
};

using fallback_fn = std::function<void()>;

class JetsonBridge : public Service {
public:

    // ── Lifecycle ────────────────────────────────────────────────────────────

    JetsonBridge();
    void attach()  override;
    void run()     override;
    void enable()  override;
    void disable() override;

    // ── Called from onIntercomRequest (routines.cpp) ─────────────────────────

    void handleRequest(Request& req);

    // ── Remote control state ──────────────────────────────────────────────────

    bool isRemoteControlled() const;   ///< True while Jetson is connected
    bool jetsonConnected()    const;   ///< True if heartbeat received recently

    // ── Remote match control ─────────────────────────────────────────────────

    bool consumeRemoteStart();         ///< Returns true once if remote start was requested
    bool isMatchPaused() const { return m_matchPaused; }
    void clearMatchPaused()   { m_matchPaused = false; }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    void pushTelemetry();              ///< Push pos + motion + safety + chrono
    void pushOccupancy();              ///< Push compressed occupancy map

    /// Enable/disable a telemetry channel at runtime (called by command_tel).
    /// channel: 0=pos  1=motion  2=safety  3=chrono  4=occ
    void setTelemetry(uint8_t channel, bool on);

    // ── Fallback programs ─────────────────────────────────────────────────────

    void registerFallback(FallbackID id, fallback_fn fn);
    void triggerFallback(FallbackID id);

    SINGLETON(JetsonBridge)

private:

    // ── Internal command execution ────────────────────────────────────────────

    void _executeCommand(const String& cmd, Request& req);
    void _replyMotionDone(bool success);

    // ── Watchdog ─────────────────────────────────────────────────────────────

    void _checkWatchdog();

    // ── Telemetry helpers ─────────────────────────────────────────────────────

    void   _pushFrame(const char* msg);
    void   _readBridgeSerial();   ///< Poll active (or both) serial ports for bridge frames
    void   _readPort(Stream& port, char* buf, uint16_t& bufLen);  ///< Parse one serial port

    // ── State ─────────────────────────────────────────────────────────────────

    bool   m_jetsonConnected    = false;
    bool   m_inFallback         = false;
    bool   m_motionPending      = false;

    // Remote match control
    bool   m_remoteStartRequested = false;   ///< Set by match_start, consumed by programManual
    bool   m_matchPaused          = false;   ///< Set by match_stop, cleared by match_start/resume

    // PR-2: Persistent DONE state — retransmitted until host acks
    char          m_lastDoneBuf[80]  = {};
    bool          m_donePending      = false;
    bool          m_lastDoneWasOk    = false;
    unsigned long m_doneRetryMs      = 0;
    static constexpr unsigned long DONE_RETRY_MS = 2000;

    // ── Telemetry channel mask ─────────────────────────────────────────────────
    // Initial state driven by Settings::Log::Telemetry (set in settings.h).
    // Toggle at runtime with: tel(pos,0)  tel(occ,1)
    bool   m_telPos    = Settings::Log::Telemetry::POS;    ///< TEL:pos   (position)
    bool   m_telMotion = Settings::Log::Telemetry::MOTION; ///< TEL:motion (motion state)
    bool   m_telSafety = Settings::Log::Telemetry::SAFETY; ///< TEL:safety
    bool   m_telChrono = Settings::Log::Telemetry::CHRONO; ///< TEL:chrono
    bool   m_telOcc    = Settings::Log::Telemetry::OCC;    ///< TEL:occ   (occupancy map)

    // FW-006: only push TEL:mask when dirty
    bool   m_maskDirty          = true;   ///< Push mask on first telemetry cycle

    // ── Bridge source tracking ──────────────────────────────────────────────────
    // Updated at runtime: set to USB when a ping or framed request arrives on
    // either BRIDGE_USB or BRIDGE_XBEE. Drives _pushFrame() routing.
    BridgeSource m_bridgeSource   = BridgeSource::INTERCOM;
    bool         m_bridgeDetected = false;  ///< True once auto-detection picked a port

    // Per-port read buffers — static char arrays to avoid heap fragmentation.
    // _bridgeBuf is for BRIDGE_XBEE (Serial3), _wiredBuf for BRIDGE_USB (Serial).
    char     _bridgeBuf[512];
    uint16_t _bridgeBufLen = 0;
    char     _wiredBuf[512];
    uint16_t _wiredBufLen  = 0;

    // JB-1: Fixed ring buffer — zero heap allocation for queued telemetry.
    // TQUEUE_FRAME raised to 256 to accommodate TEL:occ_dyn sparse frames
    // (up to ~30 dynamic cells × 6 chars + 12-char header = ~192 chars + CRC).
    static constexpr uint8_t  TQUEUE_SLOTS = 16;
    static constexpr uint16_t TQUEUE_FRAME = 256;
    char    _tqPool[TQUEUE_SLOTS][TQUEUE_FRAME];
    uint8_t _tqHead  = 0;
    uint8_t _tqTail  = 0;
    uint8_t _tqCount = 0;

    // ID of the request currently waiting for motion completion
    int    m_motionRequestId    = -1;

    unsigned long m_lastHeartbeatMs  = 0;
    unsigned long m_lastTelPushMs    = 0;
    unsigned long m_lastOccPushMs    = 0;

    unsigned long m_heartbeatTimeoutMs = 5000;  // PR-3: tunable, default 5s for strategy mode
    static constexpr unsigned long TEL_PERIOD_MS        = 100;  // 10Hz
    static constexpr unsigned long OCC_PERIOD_MS        = 500;  // 2Hz

    // Fallback registry
    fallback_fn m_fallbacks[5] = {};
};

SINGLETON_EXTERN(JetsonBridge, jetsonBridge)
