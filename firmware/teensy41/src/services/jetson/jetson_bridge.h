#pragma once
#include "services/service.h"
#include "services/intercom/request.h"
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

    // ── State ─────────────────────────────────────────────────────────────────

    bool   m_jetsonConnected    = false;
    bool   m_inFallback         = false;
    bool   m_motionPending      = false;

    // ID of the request currently waiting for motion completion
    int    m_motionRequestId    = -1;

    unsigned long m_lastHeartbeatMs  = 0;
    unsigned long m_lastTelPushMs    = 0;
    unsigned long m_lastOccPushMs    = 0;

    static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 2000;
    static constexpr unsigned long TEL_PERIOD_MS        = 100;  // 10Hz
    static constexpr unsigned long OCC_PERIOD_MS        = 500;  // 2Hz

    // Fallback registry
    fallback_fn m_fallbacks[5] = {};
};

SINGLETON_EXTERN(JetsonBridge, jetsonBridge)
