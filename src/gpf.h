/**
 * @file gpf.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-18
 */

#include "Arduino.h"

//#define GPF_RC_MAX_CHANNELS    16
#define GPF_RC_CHANNEL_AILERON         1
#define GPF_RC_CHANNEL_ELEVATOR        2
#define GPF_RC_CHANNEL_THROTTLE        3
#define GPF_RC_CHANNEL_RUDDER          4

#define GPF_RC_CHANNEL_ROLL            GPF_RC_CHANNEL_AILERON
#define GPF_RC_CHANNEL_PITCH           GPF_RC_CHANNEL_ELEVATOR
#define GPF_RC_CHANNEL_YAW             GPF_RC_CHANNEL_RUDDER

#define GPF_MAIN_LOOP_RATE             500 //us (250=4000hz, 500=2000hz, 1000=1000hz, 2000=500hz, ...) //Maximum atteignable d'environ 6000hz avec un Teensy 4.1
#define GPF_MAIN_LED_TOGGLE_DURATION   500 //ms

class GPF {

    public:
        GPF();
        void initialize();
        void iMStartingLoopNow(bool);
        void waitUntilNextLoop();
        void toggMainBoardLed();

        elapsedMicros since_lastLoop = 0;
        
    private:
        elapsedMillis debug_sincePrint;
        unsigned long loopCount          = 0;
        unsigned long loopStartedAt      = 0; //us
        unsigned long firstLoopStartedAt = 0; //us

};
