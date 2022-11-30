/**
 * @file gpf_touch.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-11-06
 * 
 * Class pour la partie "touch screen" d'un écran de type TFT d'une résolution de 320x240.
 * Le contrôleur pour la partie touch est un XPT2046
 * Cette classe s'utilise conjointement avec la librairie https://github.com/PaulStoffregen/XPT2046_Touchscreen
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_dshot.h"
#include "gpf_debug.h"
#include "gpf_util.h"
#include "gpf_touch.h"

GPF_TOUCH::GPF_TOUCH() {
}

void GPF_TOUCH::initialize() {    
    ts.begin();
}

XPT2046_Touchscreen * GPF_TOUCH::get_ts() { 
 return &ts;
}

int16_t GPF_TOUCH::mapTouchXToPixelX(int16_t x) { 
 //200 et 3950 sont les valeurs minimum et maximum de X retournées par le touchscreen lors de tests
 const int16_t TOUCH_MIN_X = 200;
 const int16_t TOUCH_MAX_X = 3950;

 const int16_t PIXEL_MIN_X = 0;
 const int16_t PIXEL_MAX_X = 239;

 return min(PIXEL_MAX_X,max(PIXEL_MIN_X,map(x,TOUCH_MIN_X,TOUCH_MAX_X,PIXEL_MIN_X,PIXEL_MAX_X)));
}

int16_t GPF_TOUCH::mapTouchYToPixelY(int16_t y) { 
 //110 et 3950 sont les valeurs minimum et maximum de Y retournées par le touchscreen lors de tests
 const int16_t TOUCH_MIN_Y = 110;
 const int16_t TOUCH_MAX_Y = 3950;

 const int16_t PIXEL_MIN_Y = 0;
 const int16_t PIXEL_MAX_Y = 319;

 return min(PIXEL_MAX_Y,max(PIXEL_MIN_Y,map(y,TOUCH_MIN_Y,TOUCH_MAX_Y,PIXEL_MIN_Y,PIXEL_MAX_Y)));
}

bool GPF_TOUCH::ts_touched() {    
    bool          touched         = get_ts()->touched();
    bool          retour          = touched;
    unsigned long current_millis  = millis();
    static bool   touchedPreviousReal  = false;    

    if (touched) {
      if (!touchedPreviousReal) {
       waitForTouchMinimumTimeReach_startedOn = 0;
       DEBUG_GPF_TOUCH_PRINTLN(F("Reset Chrono touch"));
      }
    }

    touchedPreviousReal = touched;

    if (waitForUnTouch) {
      // Pour éviter des cascades d'évènement, c'est mieux d'attendre que l'utilisateur cesse de toucher un certaint temps 
      // avant de pouvoir accepter un touch à nouveau.

      if (!touched) { //L'utilisateur ne touch plus l'écran donc on peut partir le chronomètre si il n'est pas déjà parti.
        if (waitForUntouch_startedOn == 0) {
            waitForUntouch_startedOn = current_millis; //On part le chronomètre
            DEBUG_GPF_TOUCH_PRINTLN(F("Start Chrono untouch"));
        } else {
            //Le chronomètre est déjà parti alors on vérifie si ca fait assez de temps que le untouch a été détecté
            if ((current_millis - waitForUntouch_startedOn) > GPF_TOUCH_UNTOUCH_TRESHOLD) {
              DEBUG_GPF_TOUCH_PRINTLN(F("Ok untouch treshold passé"));  
              set_waitForUnTouch(false);                      
            }      
        }
      }

      retour = touched && !waitForUnTouch;
    }

    if (touched && !waitForUnTouch) { //Il faut attendre que la touch soit pressé depuis un certain temsp pour avoir une valeur correcte/précise. En effet, on dirait que les premières lectures sont erronnées ou du moin moin précises.
      if (waitForTouchMinimumTimeReach_startedOn == 0) {
        waitForTouchMinimumTimeReach_startedOn = current_millis; //On part le chronomètre
        retour = false;
        DEBUG_GPF_TOUCH_PRINTLN(F("Start Chrono touch"));
      }
                  
      if ((current_millis - waitForTouchMinimumTimeReach_startedOn) > GPF_TOUCH_TOUCH_TRESHOLD) {          
          DEBUG_GPF_TOUCH_PRINTLN(F("Ok touch treshold passé"));  
      } else {        
        retour = false;
      }      
    }

    touchedPreviousReal = touched;

    return retour;
}

TS_Point GPF_TOUCH::ts_getPoint() {    
    return get_ts()->getPoint();
}

void GPF_TOUCH::set_waitForUnTouch(bool b) {    
    waitForUnTouch = b;
    waitForUntouch_startedOn = 0;
}
