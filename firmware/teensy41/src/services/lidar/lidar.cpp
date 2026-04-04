#include "lidar.h"
#include "os/console.h"
#include "services/intercom/intercom.h"
#include "services/lidar/occupancy.h"
//#include "services/navigation/navigation.h"
#include "services/motion/motion.h"
#include <stdio.h>

SINGLETON_INSTANTIATE(Lidar, lidar)

Lidar::Lidar() : Service(ID_LIDAR){}

FLASHMEM void Lidar::attach(){
    if(!intercom.enabled()){
        Console::error("Lidar") << "is not enabled" << Console::endl;
    }
}

FLASHMEM void Lidar::run(){
    static Vec3 pos;
    if(enabled()){
        // Position update: send robot pose to T40 so the LIDAR grid is aligned.
        if(millis() - m_lastPosUpdate > 50 || millis() - m_lastPosUpdate > 1000){
            m_lastPosUpdate = millis();
            pos = motion.estimatedPosition();
            char buf[80];
            snprintf(buf, sizeof(buf), "pos(%.1f,%.1f,%.1f)", pos.x, pos.y, pos.z*RAD_TO_DEG);
            intercom.sendRequest(buf);
        }

        // Occupancy poll: request dynamic-only sparse cells from T40 at ~5 Hz.
        if(millis() - m_lastOccupancyRequest > 200){
            m_lastOccupancyRequest = millis();
            intercom.sendRequest("oD", 500, onOccDynResponse, nullptr);
        }
    }
}

// Static callback — called by Intercom when T40 replies to "oD".
// Parses sparse "gx,gy;…" into the OccupancyMap singleton used by safety.
FLASHMEM void Lidar::onOccDynResponse(Request& req) {
    occupancy.decompressSparse(String(req.getResponse()));
}

FLASHMEM void Lidar::enable(){
    Service::enable();
}

FLASHMEM void Lidar::disable(){
    Service::disable();
}

FLASHMEM void Lidar::showRadarLED(){
    intercom.sendMessage("on");
    Console::println("displayLidar");
}

FLASHMEM void Lidar::showStatusLED(){
    intercom.sendMessage("off");
    //Console::println("displayIntercom");
}

