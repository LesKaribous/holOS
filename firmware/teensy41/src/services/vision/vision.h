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

    bool isConnected() const;

    SINGLETON(TwinVision);
};

SINGLETON_EXTERN(TwinVision, vision)
