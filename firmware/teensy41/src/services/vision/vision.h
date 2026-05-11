#pragma once
#include "services/service.h"
#include "utils/geometry.h"

// ============================================================
//  TwinVision — vision déportée sur holOS (Jetson)
//
//  La vision tourne entièrement côté holOS. Le Teensy n'a PAS de
//  port série dédié à la vision : toute requête vision passe par
//  le bridge unique holOS↔T41 (JetsonBridge). Cette classe ne
//  reste ici que pour conserver les types ObjectColor et l'API
//  attendue par le code stratégie — les méthodes sont des stubs
//  qui retournent UNKNOWN. Le service n'est pas attaché.
// ============================================================


enum class ObjectColor : uint8_t {
    UNKNOWN = 0,
    NONE,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    WHITE,
    BLACK,
};

const char* colorName(ObjectColor c);
ObjectColor colorFromChar(char c);
char        colorToChar(ObjectColor c);

using ColorCallback = void (*)(Vec2 pos, ObjectColor color);
using AICallback    = void (*)(const char* result);


// ─── Embedded-camera detection (ESP32-CAM mounted on the robot) ──────
// Returned by TwinVision::queryEmbedDetect() — see vision.cpp.
// Strategy uses `offset_mm` to nudge the gripper laterally so all 4
// stock objects sit centred in the camera frame before grabbing.
struct EmbedDetect {
    int   n        = 0;        ///< tags detected (0..expected)
    float offset_mm= 0.0f;     ///< lateral offset (image frame, +X = right)
    int   bias     = 0;        ///< when n<expected: -1 → move LEFT, +1 → RIGHT, 0 → no hint
    bool  valid    = false;    ///< true iff scale calibrated and n >= 1
};


class TwinVision : public Service {
public:
    TwinVision();

    void attach() override;
    void run() override;

    ObjectColor queryColorSync(Vec2 pos, uint32_t timeoutMs = 500);
    void        queryColor(Vec2 pos, ColorCallback cb, uint32_t timeoutMs = 500);
    void        requestMapUpdate();
    ObjectColor getColor(Vec2 pos) const;
    void        setCacheMaxAge(uint32_t ms);
    void        requestAI(const char* type, const char* params,
                          AICallback cb = nullptr, uint32_t timeoutMs = 2000);

    /// Blocking-with-timeout query of the robot-mounted ESP32-CAM via
    /// holOS (services/embed_cam.py). Returns false on timeout.  The
    /// host runs ArUco detection on a single JPEG fetched from the
    /// embed cam and replies with `embed_detect_reply(...)`.
    ///
    /// `team` ∈ {"", "blue", "yellow"}: when non-empty, the host
    /// filters to that team's ArUco ID (36 blue / 47 yellow) and the
    /// opposite-team stock in the FOV is ignored.  Empty == accept
    /// both IDs (cfg.team default).
    bool queryEmbedDetect(EmbedDetect& out,
                          const char* team = nullptr,
                          uint32_t timeoutMs = 6000);

    /// Called from JetsonBridge when an embed_detect_reply lands.
    /// Not for user code.
    void onEmbedDetectReply(const EmbedDetect& r);

    bool isConnected() const;

    SINGLETON(TwinVision);

private:
    volatile bool m_pendingEmbedReply = false;
    EmbedDetect   m_lastEmbed{};
};

SINGLETON_EXTERN(TwinVision, vision)
