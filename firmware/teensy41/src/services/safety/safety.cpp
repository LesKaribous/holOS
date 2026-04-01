#include "safety.h"

#include "os/console.h"
#include "services/motion/motion.h"
#include "services/intercom/intercom.h"
#include "utils/timer/timer.h"
#include <stdio.h>

SINGLETON_INSTANTIATE(Safety, safety)

void getDistanceCallback(Request& req){
    int d = atoi(req.getResponse());

    /*
    if(d > 200 && d < 3000) safety.setSafeDistance(d);
    else safety.setSafeDistance(infinityf());*/

    if(d == 1)safety.setObstacleDetected(true);
    else safety.setObstacleDetected(false);

    //Console::println( d );
}


FLASHMEM void Safety::enable(){
    Service::enable();
}

FLASHMEM void Safety::disable(){
    Service::disable();
    m_obstacleDetected = false;
    Console::println("obstable forced to false");
}


FLASHMEM void Safety::attach(){
}

FLASHMEM void Safety::run(){
    if(!enabled()){return;}

    RUN_EVERY(
        if(!motion.hasFinished()){
            int streer = RAD_TO_DEG * motion.getAbsoluteTargetDirection(); //We are moving in this direction
            int distanceToGo = motion.getTargetDistance() + 350;
            if(distanceToGo > 600) distanceToGo = 600;
            if(distanceToGo < 300) distanceToGo = 300;

            char buf[64];
            snprintf(buf, sizeof(buf), "ob(%d,%d)", streer, distanceToGo);
            intercom.sendRequest(buf, 300, getDistanceCallback);
            //intercom.sendRequest("pos(" + String(pos.x)  + "," + String(pos.y) + "," + String(pos.z*RAD_TO_DEG) + ")");
        }

        if(m_obstacleDetected/*m_currentDistance <= 350*/){
            if(!motion.hasFinished() /*&& !motion.isRotating()*/) motion.pause();
            if(!motion.isRotating()) m_lastSeen = millis();
         }
     
         if(millis() - m_lastSeen > 1500){
            if(m_obstacleDetected){
                m_obstacleDetected = false;
                Console::print("obstable gone (last seen ");
                Console::print(millis() - m_lastSeen);
                Console::println("ms ago");
            }

         }
         
         if(motion.isPaused() && !m_obstacleDetected/*m_currentDistance > 350 */&& millis() - m_lastSeen > 1000){
            motion.resume();
            Console::println("resume");
         }
    , 100)
}

FLASHMEM void Safety::setSafeDistance(int d){
    m_currentDistance = d;
}

FLASHMEM void Safety::setObstacleDetected(bool state){
    
    if(m_obstacleDetected && state){
        Console::println("obstable detected");
        Console::print("obstable detected (seen at");
        Console::print(millis());
        Console::println("ms");
    }
    else if(m_obstacleDetected && !state) Console::println("obstable gone");
    m_obstacleDetected = state;
}