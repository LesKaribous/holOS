#pragma once
#include "services/service.h"
#include "utils/geometry.h"
#include "services/lidar/ld06.h"
#include "services/intercom/comUtilities.h"
#include <U8g2lib.h>

class Lidar : public Service{
public:
    Lidar();

    void onAttach() override;
    void onUpdate() override;

    int getCount(int angle, bool absolute=true);
    int getDistance(int angle, bool absolute=true);

    bool isStaticOccupied(int x, int y);

    // Load static map from a hex-encoded bitmap (gx outer, gy inner, LSB first).
    // Format: 66 hex nibbles (33 bytes for 20×13 = 260 bits), no CRC suffix.
    // Sent by T41 relaying the Jetson's deploy command.
    void setStaticMapHex(const String& hex);

    void setPosition(float x, float y, float theta);
    Vec3 getPosition();

    // Full occupancy (static + dynamic) packed as 33-byte bitmap — kept for
    // OLED rendering and any legacy callers.
    BinaryPayload getOccupancyMap();

    // Dynamic-only occupancy in sparse text format: "gx,gy;gx,gy;…"
    // Empty string if no dynamic cells.  Sent as TEL:occ_dyn on T41.
    String getOccupancyDyn();

    void drawOccupancyGrid();

private:
    LD06 sensor;
    U8G2_SH1106_128X64_NONAME_F_HW_I2C  u8g2;
    float m_x, m_y, m_theta;

    // RAM-based static map (replaces compile-time team presets).
    // Writable at runtime via setStaticMapHex() / setStaticMap().
    uint8_t m_ram_static[GRID_WIDTH][GRID_HEIGHT];

private:
    SERVICE(Lidar)
};

