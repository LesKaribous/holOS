#include "vision.h"

SINGLETON_INSTANTIATE(TwinVision, vision)

TwinVision::TwinVision() : Service(ID_VISION) {}

FLASHMEM const char* colorName(ObjectColor c) {
    switch (c) {
        case ObjectColor::RED:     return "rouge";
        case ObjectColor::GREEN:   return "vert";
        case ObjectColor::BLUE:    return "bleu";
        case ObjectColor::YELLOW:  return "jaune";
        case ObjectColor::WHITE:   return "blanc";
        case ObjectColor::BLACK:   return "noir";
        case ObjectColor::NONE:    return "aucun";
        default:                   return "inconnu";
    }
}

FLASHMEM ObjectColor colorFromChar(char c) {
    switch (c) {
        case 'R': return ObjectColor::RED;
        case 'G': return ObjectColor::GREEN;
        case 'B': return ObjectColor::BLUE;
        case 'Y': return ObjectColor::YELLOW;
        case 'W': return ObjectColor::WHITE;
        case 'K': return ObjectColor::BLACK;
        case 'N': return ObjectColor::NONE;
        default:  return ObjectColor::UNKNOWN;
    }
}

FLASHMEM char colorToChar(ObjectColor c) {
    switch (c) {
        case ObjectColor::RED:    return 'R';
        case ObjectColor::GREEN:  return 'G';
        case ObjectColor::BLUE:   return 'B';
        case ObjectColor::YELLOW: return 'Y';
        case ObjectColor::WHITE:  return 'W';
        case ObjectColor::BLACK:  return 'K';
        case ObjectColor::NONE:   return 'N';
        default:                  return '?';
    }
}

// Service n'est jamais attaché — vision est gérée côté holOS via JetsonBridge.
void TwinVision::attach() {}
void TwinVision::run()    {}

ObjectColor TwinVision::queryColorSync(Vec2, uint32_t)          { return ObjectColor::UNKNOWN; }
void        TwinVision::queryColor(Vec2 pos, ColorCallback cb, uint32_t) {
    if (cb) cb(pos, ObjectColor::UNKNOWN);
}
void        TwinVision::requestMapUpdate()                       {}
ObjectColor TwinVision::getColor(Vec2) const                     { return ObjectColor::UNKNOWN; }
void        TwinVision::setCacheMaxAge(uint32_t)                 {}
void        TwinVision::requestAI(const char*, const char*, AICallback cb, uint32_t) {
    if (cb) cb("unavailable");
}
bool        TwinVision::isConnected() const                      { return false; }
