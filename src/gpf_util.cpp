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
#include <TimeLib.h>

char   gpf_util_dateTimeString[30] = ""; //Augmenter au besoin si on ajoute des choses dans la fonction ci-dessous.

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

gpf_rc_channel_position_type_enum gpf_util_getPwmChannelToPosType(int currentPos) {
  gpf_rc_channel_position_type_enum retour = GPF_RC_CHANNEL_POSITION_UNKNOWN;

  //Low
  if (currentPos < (GPF_RC_CHANNEL_POSITION_LOW - GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION)) {
   retour = GPF_RC_CHANNEL_POSITION_LOW;
  } else {
   //Mid
   if (
       ((GPF_RC_CHANNEL_POSITION_MID + GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION) > currentPos) &&
       ((GPF_RC_CHANNEL_POSITION_MID - GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION) < currentPos)
      ) {
    retour = GPF_RC_CHANNEL_POSITION_MID;
   } else {
    //High
    if (currentPos > (GPF_RC_CHANNEL_POSITION_HIGH + GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION)) {
     retour = GPF_RC_CHANNEL_POSITION_HIGH;
    } 
   }  
  }

  return retour;
}

bool gpf_util_isPwmChannelAtPos(int currentPos, gpf_rc_channel_position_type_enum channel_position_required) {
  bool retour = false;

  switch (channel_position_required) {
  case GPF_RC_CHANNEL_POSITION_LOW:
    retour = (currentPos < (channel_position_required - GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION));
    break;

  case GPF_RC_CHANNEL_POSITION_MID:
    retour = (
              ((channel_position_required + GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION) > currentPos) &&
              ((channel_position_required - GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION) < currentPos)
             );
    break;  

  case GPF_RC_CHANNEL_POSITION_HIGH:
    retour = (currentPos > (channel_position_required + GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION));
    break;

  }

  return  retour;
}

void  gpf_util_resetConfigToDefault(gpf_config_struct *ptr) {
   DEBUG_GPF_PRINTLN(F("***** Oups, reset de la config!!! *****"));

   ptr->version          = GPF_MISC_CONFIG_CURRENT_VERSION;

   ptr->channelMaps[GPF_RC_STICK_ROLL]                = GPF_RC_STICK_ROLL_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_PITCH]               = GPF_RC_STICK_PITCH_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_THROTTLE]            = GPF_RC_STICK_THROTTLE_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_YAW]                 = GPF_RC_STICK_YAW_DEFAULT_CHANNEL; 
   ptr->channelMaps[GPF_RC_STICK_ARM]                 = GPF_RC_STICK_ARM_DEFAULT_CHANNEL;   
   ptr->channelMaps[GPF_RC_STICK_FLIGHT_MODE]         = GPF_RC_STICK_FLIGHT_MODE_DEFAULT_CHANNEL;   
   ptr->channelMaps[GPF_RC_STICK_BLACK_BOX]           = GPF_RC_STICK_BLACK_BOX_DEFAULT_CHANNEL;   
   ptr->channelMaps[GPF_RC_STICK_MUSIC_PLAYER_VOLUME] = GPF_RC_STICK_MUSIC_PLAYER_VOLUME_DEFAULT_CHANNEL;
   ptr->channelMaps[GPF_RC_STICK_MUSIC_PLAYER_TRACK]  = GPF_RC_STICK_MUSIC_PLAYER_TRACK_DEFAULT_CHANNEL;
   ptr->channelMaps[GPF_RC_STICK_MUSIC_PLAYER_LIST]   = GPF_RC_STICK_MUSIC_PLAYER_LIST_DEFAULT_CHANNEL;

   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_PROPORTIONAL]   = 0.2     * GPF_PID_STORAGE_MULTIPLIER; //0.2  //Roll P-gain - angle mode 
   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_INTEGRAL]       = 0.3     * GPF_PID_STORAGE_MULTIPLIER; //0.3  //Roll I-gain - angle mode 
   ptr->pids[GPF_AXE_ROLL][GPF_PID_TERM_DERIVATIVE]     = 0.05    * GPF_PID_STORAGE_MULTIPLIER; //0.05 //Roll D-gain - angle mode (has no effect on controlANGLE2)

   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_PROPORTIONAL]  = 0.2     * GPF_PID_STORAGE_MULTIPLIER; //0.2  //Pitch P-gain - angle mode
   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_INTEGRAL]      = 0.3     * GPF_PID_STORAGE_MULTIPLIER; //0.3  //Pitch I-gain - angle mode
   ptr->pids[GPF_AXE_PITCH][GPF_PID_TERM_DERIVATIVE]    = 0.05    * GPF_PID_STORAGE_MULTIPLIER; //0.05 //Pitch D-gain - angle mode (has no effect on controlANGLE2)

   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_PROPORTIONAL]    = 0.3     * GPF_PID_STORAGE_MULTIPLIER; //0.3     //Yaw P-gain
   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_INTEGRAL]        = 0.05    * GPF_PID_STORAGE_MULTIPLIER; //0.05    //Yaw I-gain
   ptr->pids[GPF_AXE_YAW][GPF_PID_TERM_DERIVATIVE]      = 0.00015 * GPF_PID_STORAGE_MULTIPLIER; //0.00015 //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]  = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]  = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]  = 0;

   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]      = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]      = 0;
   ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]      = 0;   

}

time_t gpf_util_getTeensy3Time() {
  return Teensy3Clock.get();
}

void  gpf_util_beep(uint16_t frequency, uint32_t duration) {

  tone(GPF_MISC_PIN_BUZZER, frequency, duration);
  
}


float gpf_util_invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

char* gpf_util_get_dateTimeString(uint8_t format, bool addSpace) {
   int i;
   char tmpBuffer[10]       = "";
   long l;

   strcpy(gpf_util_dateTimeString,"");

   itoa(year(), tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //4

   if ( (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY) || (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US) ) {
    strcat(gpf_util_dateTimeString,"-"); //1
   }
    
   i = month();
   if (i < 10) {
    strcat(gpf_util_dateTimeString,"0");
   }
   itoa(i, tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //2

   if ( (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY) || (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US) ) {
    strcat(gpf_util_dateTimeString,"-"); //1
   }

   i = day();
   if (i < 10) {
    strcat(gpf_util_dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //2

   if (format == GPF_MISC_FORMAT_DATE_TIME_LOGGING) {      
    strcat(gpf_util_dateTimeString, " "); //1 //,
   } else {
    strcat(gpf_util_dateTimeString, " "); //1
   }

   i = hour();
   if (i < 10) {
    strcat(gpf_util_dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //2

   if ( (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY) || (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US) ) {
    strcat(gpf_util_dateTimeString,":"); //1
   }

   i = minute();
   if (i < 10) {
    strcat(gpf_util_dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //2

   if ( (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY) || (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US) ) {
    strcat(gpf_util_dateTimeString,":"); //1
   }

   i = second();
   if (i < 10) {
    strcat(gpf_util_dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(gpf_util_dateTimeString, tmpBuffer); //2

   if ( (format == GPF_MISC_FORMAT_DATE_TIME_LOGGING) || (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US) ) {

     strcat(gpf_util_dateTimeString,"."); //1

     //i = millis() % 1000;
     l = micros() % 1000000;
     if (l < 10) {
      strcat(gpf_util_dateTimeString,"0");    
     }
     if (l < 100) {
      strcat(gpf_util_dateTimeString,"0");    
     }
     if (l < 1000) {
      strcat(gpf_util_dateTimeString,"0");    
     }
     if (l < 10000) {
      strcat(gpf_util_dateTimeString,"0");    
     }
     if (l < 100000) {
      strcat(gpf_util_dateTimeString,"0");    
     }
     ltoa(l, tmpBuffer, 10);
     strcat(gpf_util_dateTimeString, tmpBuffer); //6     
     
   }

   if (addSpace) {
     strcat(gpf_util_dateTimeString," "); //1
   }

   return gpf_util_dateTimeString;
}

uint16_t  gpf_util_getVoltage() { 
 const uint16_t CALIBRATION_ADC = 12; 
 const float    CALIBRATION_VAL_PER_DECIVOLT = 5.48823;

 uint16_t voltage = floor((constrain(analogRead(PIN_A8),0,1023) - CALIBRATION_ADC) / CALIBRATION_VAL_PER_DECIVOLT);
 
 //Return un entier. Ex.: 1=0.1v, 168=16.8v
 return voltage;

}