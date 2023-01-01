/**
 * @file gpf_dshot.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-28
 * 
 * Voir fichier gpf_touch.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_TOUCH_H
#define GPF_TOUCH_H

#include "Arduino.h"
#include "XPT2046_Touchscreen.h"

#define GPF_TOUCH_UNTOUCH_TRESHOLD 50 //ms
#define GPF_TOUCH_TOUCH_TRESHOLD   50 //ms

class GPF_TOUCH {

    public:
        GPF_TOUCH();
        void initialize();
        XPT2046_Touchscreen * get_ts();
        static int16_t mapTouchXToPixelX(int16_t);
        static int16_t mapTouchYToPixelY(int16_t);

        bool     ts_touched();
        TS_Point ts_getPoint();
        void     set_waitForUnTouch(bool);
        
    private:        
        XPT2046_Touchscreen ts = XPT2046_Touchscreen(GPF_SPI_TOUCH_DC);
        bool          waitForUnTouch                         = false;
        unsigned long waitForUntouch_startedOn               = 0;        
        unsigned long waitForTouchMinimumTimeReach_startedOn = 0;
};

#endif