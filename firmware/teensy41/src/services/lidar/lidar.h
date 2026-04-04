#pragma once
#include "services/service.h"
#include "config/settings.h"
#include "occupancy.h"
#include "services/intercom/request.h"

class Lidar : public Service{
public:

    Lidar();

    void attach() override;
    void run() override;

    void enable() override;
    void disable() override;

    void showRadarLED();
    void showStatusLED();

    // Push current dynamic occupancy into the OccupancyMap singleton.
    // Called by the intercom request callback when T40 replies to "oD".
    // Signature matches requestCallback_ptr (void (*)(Request&)).
    static void onOccDynResponse(Request& req);

private :
    long m_lastPosUpdate = 0;
    long m_lastOccupancyRequest = 0;

    float x, y, theta;//abs

    SINGLETON(Lidar);
};

SINGLETON_EXTERN(Lidar, lidar)