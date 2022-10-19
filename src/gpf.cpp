/**
 * @file gpf.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-18
 * 
 * Class principale du projet
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_debug.h"

GPF::GPF() {
    //Rien de spécial dans le constructeur pour le moment    
}

void GPF::initialize() {    
    debug_sincePrint = 0;
}

void GPF::iMStartingLoopNow(bool syncLoop) {
 if (firstLoopStartedAt == 0) {
     // Uniquement lors de la première loop, on attend le début de la prochaine milliseconde 
     // histoire que nos stats soient le plus précis possible
     unsigned long us       = micros();
     unsigned long delay_us = 1000 - (us % 1000);
     delayMicroseconds(delay_us);
     firstLoopStartedAt = micros();
 }

 if (syncLoop) {
  //On attend...
  while ((micros() - loopStartedAt) < GPF_MAIN_LOOP_RATE) {
   //Attend
  }
 } 

 loopStartedAt = micros();
 
 #ifdef DEBUG_GPF_ENABLED 
  if (debug_sincePrint > DEBUG_GPF_DELAY) {
   DEBUG_GPF_PRINT(F("GPF: loopCount="));
   DEBUG_GPF_PRINT(loopCount);
   DEBUG_GPF_PRINT(F(" "));
   DEBUG_GPF_PRINT((float)(1000000 / (((float)(micros() - firstLoopStartedAt))/loopCount)));
   DEBUG_GPF_PRINTLN(F("hz"));
   debug_sincePrint = 0;
  }
 #endif
 
 loopCount++; 
}

void GPF::waitUntilNextLoop() {
 static elapsedMicros since_lastLoop_static = 0;

 while (since_lastLoop_static > 2000) {
    delayMicroseconds(1);
 }
 since_lastLoop_static = 0;
}

void GPF::toggMainBoardLed() {    
  static bool ledState = true;
  static elapsedMillis since_ledToggled = 0;

  if (since_ledToggled > GPF_MAIN_LED_TOGGLE_DURATION) {
   ledState = !ledState;
   digitalWrite(LED_BUILTIN, ledState);
   since_ledToggled = 0;
  }    
}



