#pragma once
#include <string>

namespace Text{
    constexpr char
    //Colors
    COLOR_A[] = "Yellow",
    COLOR_B[] = "Blue",

    //Strategy
    // Switch OFF (false) = Internal   (dumb T41-only, stop-on-obstacle)
    // Switch ON  (true)  = Remote     (Jetson-based, full pathfinding)
    STRAT_PRIMARY_A[] = "Internal",
    STRAT_PRIMARY_B[] = "Remote",
    STRAT_SECONDARY_A[] = "Internal",
    STRAT_SECONDARY_B[] = "Remote",

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