#pragma once
#include "services/service.h"
#include "services/intercom/request.h"
#include "config/settings.h"
#include "utils/timer/timer.h"
#include <functional>
#include <deque>

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

    // ── Telemetry ─────────────────────────────────────────────────────────────

    void pushTelemetry();              ///< Push pos + motion + safety + chrono
    void pushOccupancy();              ///< Push compressed occupancy map

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

    String _buildTelemetry() const;
    String _buildPositionTel() const;
    void   _pushFrame(const String& msg);
    void   _readBridgeSerial();   ///< Parse incoming requests from USB-CDC bridge

    // ── State ─────────────────────────────────────────────────────────────────

    bool   m_jetsonConnected    = false;
    bool   m_inFallback         = false;
    bool   m_motionPending      = false;

    // ── Telemetry channel mask ─────────────────────────────────────────────────
    bool   m_telPos    = true;   ///< TEL:pos   (position)
    bool   m_telMotion = true;   ///< TEL:motion (motion state)
    bool   m_telSafety = true;   ///< TEL:safety
    bool   m_telChrono = true;   ///< TEL:chrono
    bool   m_telOcc    = true;   ///< TEL:occ   (occupancy map)

    // FW-006: only push TEL:mask when dirty
    bool   m_maskDirty          = true;   ///< Push mask on first telemetry cycle

    // ── USB bridge source tracking ─────────────────────────────────────────────
    // Updated at runtime: set to USB when a ping or framed request arrives on
    // BRIDGE_SERIAL. Drives _pushFrame() routing without any compile-time #define.
    BridgeSource m_bridgeSource  = BridgeSource::INTERCOM;

    // Static char array instead of String to avoid repeated heap allocs in the
    // hot path — each character appended to a String can trigger malloc/free.
    char     _bridgeBuf[512];
    uint16_t _bridgeBufLen = 0;

    // Telemetry queue — buffers frames when USB TX FIFO is full to prevent loss
    std::deque<String> _telQueue;
    static constexpr size_t TEL_QUEUE_MAX = 32;  // ~4KB when full (max 128 bytes per frame)

    // ID of the request currently waiting for motion completion
    int    m_motionRequestId    = -1;

    unsigned long m_lastHeartbeatMs  = 0;
    unsigned long m_lastTelPushMs    = 0;
    unsigned long m_lastOccPushMs    = 0;
    unsigned long m_lastTelDrainMs   = 0;  // Last time we tried to drain queued telemetry

    static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 2000;
    static constexpr unsigned long TEL_PERIOD_MS        = 100;  // 10Hz
    static constexpr unsigned long OCC_PERIOD_MS        = 500;  // 2Hz
    static constexpr unsigned long TEL_DRAIN_PERIOD_MS  = 10;   // Try to drain queue every 10ms

    // Fallback registry
    fallback_fn m_fallbacks[5] = {};
};

SINGLETON_EXTERN(JetsonBridge, jetsonBridge)
