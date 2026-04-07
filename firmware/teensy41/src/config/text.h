#pragma once
#include <string>

namespace Text{
    constexpr char
    //Colors
    COLOR_A[] = "Yellow",
    COLOR_B[] = "Blue",

    //Strategy
    // Switch OFF (false) = Séquentielle (dumb T41-only, stop-on-obstacle)
    // Switch ON  (true)  = Intelligente  (Jetson-based, full pathfinding)
    STRAT_PRIMARY_A[] = "Sequentiel",
    STRAT_PRIMARY_B[] = "Intelligen",
    STRAT_SECONDARY_A[] = "Sequentiel",
    STRAT_SECONDARY_B[] = "Intelligen",

    //Lidar
    LIDAR_DISCONNECTED[] = "Waiting...",
    LIDAR_CONNECTED[] = "Connected",

    //Starter
    STARTER_ARMED[] = "armed !",
    STARTER_UNARMED[] = "unarmed !",

    //Probe State
    PROBE_NOT[] = "Not probed",
    PROBING[] = "Probing ...",
    PROBED[] = "Probed.",
    PROBE_UNKNOWN[] = "?";
}