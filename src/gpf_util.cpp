/**
 * @file gpf_util.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-26
 * 
 */

#include "Arduino.h"
#include "gpf_util.h"
#include "gpf_cons.h"


void gpf_util_blinkMainBoardLed(uint8_t nbrBlink) { 
 // Attention, la LED_BUILTIN (pin 13) est la même pin que le SPI SCK sur un Teensy 4.1 
 // alors une fois mon SPI initialisé, je ne devrait plus me servir de cette fonction.
 for (uint8_t i = 0; i < nbrBlink; i++) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
 }
}

uint16_t gpf_util_shiftBitsToBigEndian_16(uint16_t val) {    
    return (val << 8) | (val >> 8 );
}

/*
int32_t gpf_util_shiftBitsToBigEndian_int32_t(int32_t val) {    
    return (val >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8) | (val << 24);
}
*/

uint32_t gpf_util_shiftBitsToBigEndian_32(uint32_t val) { //Ok ca garde le signe cat le int32 recu en paramètre se fait casté en uint32    
    return (val >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8) | (val << 24);
}

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;

int gpf_util_freeRam() {
  return (char *)&_heap_end - __brkval;
}

bool gpf_util_isPwmChannelAtPos(int currentPos, int posRequired) {
  return  ( (currentPos >= (posRequired - GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION)) &&
          (currentPos <= (posRequired + GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION)) );
}

void  gpf_util_resetConfigToDefault(gpf_config_struct *ptr) {
   DEBUG_GPF_PRINTLN(F("***** Oups, reset de la config!!! *****"));

   ptr->version          = GPF_MISC_CONFIG_CURRENT_VERSION;

   ptr->channelMaps[GPF_RC_STICK_ROLL]     = GPF_RC_STICK_ROLL_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_PITCH]    = GPF_RC_STICK_PITCH_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_THROTTLE] = GPF_RC_STICK_THROTTLE_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_YAW]      = GPF_RC_STICK_YAW_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_ARM]      = GPF_RC_STICK_ARM_DEFAULT_CHANNEL;   

   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_PROPORTIONAL]   = 1;
   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_INTEGRAL]       = 1;
   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_DERIVATIVE]     = 1;

   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_PROPORTIONAL]  = 1;
   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_INTEGRAL]      = 1;
   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_DERIVATIVE]    = 1;

   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_PROPORTIONAL]    = 1;
   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_INTEGRAL]        = 1;
   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_DERIVATIVE]      = 1;

   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]  = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]  = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]  = 0;

   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]      = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]      = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]      = 0;

   //On averti l'utilisateur de cette situation puis on entre directement enmode calibration
   //tone(pinBuzzer, 200);
   //delay(300);
   //noTone(pinBuzzer);   
   
   //tone(pinBuzzer, 150);
   //delay(300);
   //noTone(pinBuzzer);
   
   //tone(pinBuzzer, 100);
   //delay(300);
   //noTone(pinBuzzer);
   
   //calibrationMode = true;


  //myConfig_ptr->channelMaps[GPF_RC_STICK_ARM]

}