#include "occupancy.h"
#include "os/console.h"
#include "utils/timer/timer.h"
#include <cmath>
#include <limits>

SINGLETON_INSTANTIATE(OccupancyMap, occupancy);

// ============================================================
//  Système de coordonnées
//  Monde  : origine coin bas-gauche, x ∈ [0, TABLE_SIZE_X], y ∈ [0, TABLE_SIZE_Y]
//  Grille : gx ∈ [0, GRID_WIDTH-1],  gy ∈ [0, GRID_HEIGHT-1]
//  Taille d'une cellule : GRID_CELLSIZE mm (150 mm)
//
//  Conversion :  gx = (int)(x * GRID_WIDTH  / TABLE_SIZE_X)
//                gy = (int)(y * GRID_HEIGHT / TABLE_SIZE_Y)
// ============================================================

OccupancyMap::OccupancyMap() {
    for (int i = 0; i < GRID_WIDTH; ++i)
        for (int j = 0; j < GRID_HEIGHT; ++j)
            m_map[i][j] = 0;
}


// ============================================================
//  Conversions monde ↔ grille
// ============================================================

// Centre monde (mm) de la cellule (gx, gy)
FLASHMEM Vec2 OccupancyMap::gridToWorld(int gx, int gy) const {
    float wx = (gx + 0.5f) * TABLE_SIZE_X / GRID_WIDTH;
    float wy = (gy + 0.5f) * TABLE_SIZE_Y / GRID_HEIGHT;
    return Vec2(wx, wy);
}

// Cellule contenant le point monde (x, y)
FLASHMEM Vec2 OccupancyMap::worldToGrid(float x, float y) const {
    int gx = (int)(x * GRID_WIDTH  / TABLE_SIZE_X);
    int gy = (int)(y * GRID_HEIGHT / TABLE_SIZE_Y);
    // Clamp pour les points exactement sur le bord
    if (gx >= GRID_WIDTH)  gx = GRID_WIDTH  - 1;
    if (gy >= GRID_HEIGHT) gy = GRID_HEIGHT - 1;
    return Vec2(gx, gy);
}


// ============================================================
//  Requêtes d'occupation
// ============================================================

FLASHMEM bool OccupancyMap::isInBounds(int gx, int gy) const {
    return (gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT);
}

FLASHMEM bool OccupancyMap::isCellOccupied(int gx, int gy) const {
    if (!isInBounds(gx, gy)) return false;
    return m_map[gx][gy] > 0;
}

// Coordonnées monde entières (mm)
FLASHMEM bool OccupancyMap::isOccupied(int x, int y) const {
    Vec2 g = worldToGrid((float)x, (float)y);
    return isCellOccupied((int)g.x, (int)g.y);
}

// Coordonnées monde flottantes (mm)
FLASHMEM bool OccupancyMap::isOccupied(const Vec2& pos) const {
    Vec2 g = worldToGrid(pos.x, pos.y);
    return isCellOccupied((int)g.x, (int)g.y);
}

// Retourne true si au moins une cellule occupée se trouve dans le rayon
// `radius` (mm) autour du point `center` (coordonnées monde).
FLASHMEM bool OccupancyMap::isZoneOccupied(const Vec2& center, float radius) const {
    // Cellule centrale
    Vec2 gc     = worldToGrid(center.x, center.y);
    int  gcx    = (int)gc.x;
    int  gcy    = (int)gc.y;
    int  rCells = (int)ceilf(radius / GRID_CELLSIZE) + 1; // marge d'une cellule

    for (int dx = -rCells; dx <= rCells; ++dx) {
        for (int dy = -rCells; dy <= rCells; ++dy) {
            int gx = gcx + dx;
            int gy = gcy + dy;

            if (!isInBounds(gx, gy)) continue;
            if (!isCellOccupied(gx, gy)) continue;

            // Vérification précise sur le centre de la cellule
            Vec2  cellCenter = gridToWorld(gx, gy);
            float dx_mm      = cellCenter.x - center.x;
            float dy_mm      = cellCenter.y - center.y;
            float distSq     = dx_mm * dx_mm + dy_mm * dy_mm;

            if (distSq <= radius * radius) return true;
        }
    }
    return false;
}


// ============================================================
//  Distance au plus proche obstacle
// ============================================================

FLASHMEM float OccupancyMap::distanceToNearestObstacle(const Vec2& pos) const {
    float minDist = std::numeric_limits<float>::max();

    for (int gx = 0; gx < GRID_WIDTH; ++gx) {
        for (int gy = 0; gy < GRID_HEIGHT; ++gy) {
            if (!isCellOccupied(gx, gy)) continue;

            Vec2  cell  = gridToWorld(gx, gy);
            float dx    = cell.x - pos.x;
            float dy    = cell.y - pos.y;
            float dist  = sqrtf(dx * dx + dy * dy);
            if (dist < minDist) minDist = dist;
        }
    }

    return minDist;
}


// ============================================================
//  Gradient répulsif (champ de potentiel)
// ============================================================

FLASHMEM Vec2 OccupancyMap::repulsiveGradient(const Vec2& pos) const {
    constexpr int   RANGE          = 5;      // rayon de recherche en cellules
    constexpr float REPULSION_GAIN = 10.0f;
    constexpr float CUTOFF_RADIUS  = 300.0f; // mm
    constexpr float CUTOFF_SQ      = CUTOFF_RADIUS * CUTOFF_RADIUS;
    constexpr float FALL_OFF       = 2.5f;

    Vec2 force(0.0f, 0.0f);

    Vec2 gc  = worldToGrid(pos.x, pos.y);
    int  gcx = (int)gc.x;
    int  gcy = (int)gc.y;

    for (int dx = -RANGE; dx <= RANGE; ++dx) {
        for (int dy = -RANGE; dy <= RANGE; ++dy) {
            int nx = gcx + dx;
            int ny = gcy + dy;

            if (!isInBounds(nx, ny) || !isCellOccupied(nx, ny)) continue;

            Vec2  obs    = gridToWorld(nx, ny);
            float diffX  = pos.x - obs.x;
            float diffY  = pos.y - obs.y;
            float distSq = diffX * diffX + diffY * diffY;

            if (distSq < 1e-6f)  distSq = 1e-6f;
            if (distSq > CUTOFF_SQ) continue;

            float scale = REPULSION_GAIN / powf(distSq, FALL_OFF / 2.0f);
            force.x += diffX * scale;
            force.y += diffY * scale;
        }
    }

    return force;
}


// ============================================================
//  Compression (envoi vers Jetson)
// ============================================================

FLASHMEM String OccupancyMap::compress() const {
    uint8_t data[GRID_BYTES] = {0};

    // Même ordre que decompress : gy en boucle externe, gx en interne
    int bitIndex = 0;
    for (int gy = 0; gy < GRID_HEIGHT; ++gy) {
        for (int gx = 0; gx < GRID_WIDTH; ++gx) {
            if (m_map[gx][gy]) {
                data[bitIndex / 8] |= (1 << (bitIndex % 8));
            }
            ++bitIndex;
        }
    }

    // XOR CRC
    uint8_t crc = 0;
    for (int i = 0; i < GRID_BYTES; ++i) crc ^= data[i];

    // Encodage hex
    static const char HEX_CHARS[] = "0123456789abcdef";
    String out;
    out.reserve(GRID_BYTES * 2 + 3);
    for (int i = 0; i < GRID_BYTES; ++i) {
        out += HEX_CHARS[(data[i] >> 4) & 0xF];
        out += HEX_CHARS[ data[i]       & 0xF];
    }
    out += '-';
    out += HEX_CHARS[(crc >> 4) & 0xF];
    out += HEX_CHARS[ crc       & 0xF];
    return out;
}


// ============================================================
//  Décompression sparse (cellules dynamiques depuis T40 en format "gx,gy;…")
// ============================================================

FLASHMEM void OccupancyMap::decompressSparse(const String& sparse) {
    // Reset dynamic layer
    for (int i = 0; i < GRID_WIDTH; ++i)
        for (int j = 0; j < GRID_HEIGHT; ++j)
            m_map[i][j] = 0;

    if (sparse.length() == 0) return;

    // Parse "gx,gy;gx,gy;…"
    int start = 0;
    while (start < (int)sparse.length()) {
        int semicolon = sparse.indexOf(';', start);
        int end = (semicolon == -1) ? (int)sparse.length() : semicolon;

        String token = sparse.substring(start, end);
        int comma = token.indexOf(',');
        if (comma > 0) {
            int gx = token.substring(0, comma).toInt();
            int gy = token.substring(comma + 1).toInt();
            if (isInBounds(gx, gy)) m_map[gx][gy] = 1;
        }

        start = end + 1;
    }
}

// ============================================================
//  Décompression (données reçues du robot secondaire)
// ============================================================

FLASHMEM void OccupancyMap::decompress(const String& encoded) {
    if (encoded.length() < (GRID_BYTES * 2 + 3))
        return;

    int sep = encoded.lastIndexOf('-');
    if (sep == -1 || sep + 2 > (int)encoded.length()) return;

    String dataHex = encoded.substring(0, sep);
    String crcHex  = encoded.substring(sep + 1);

    uint8_t data[GRID_BYTES] = {0};
    for (int i = 0; i < GRID_BYTES; ++i) {
        String byteStr = dataHex.substring(i * 2, i * 2 + 2);
        data[i] = (uint8_t)strtoul(byteStr.c_str(), nullptr, 16);
    }

    // CRC XOR
    uint8_t crc = 0;
    for (int i = 0; i < GRID_BYTES; ++i) crc ^= data[i];
    if (crc != (uint8_t)strtoul(crcHex.c_str(), nullptr, 16)) return;

    // Décodage bit → cellule
    int bitIndex = 0;
    for (int gy = 0; gy < GRID_HEIGHT; ++gy) {
        for (int gx = 0; gx < GRID_WIDTH; ++gx) {
            int byteIdx  = bitIndex / 8;
            int bitInByte = bitIndex % 8;
            m_map[gx][gy] = (data[byteIdx] >> bitInByte) & 0x01;
            ++bitIndex;
        }
    }
}
