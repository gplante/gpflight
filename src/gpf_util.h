/**
 * @file gpf_util.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-26
 */

#include "Arduino.h"
#include "gpf_cons.h"
#include "gpf_debug.h"

#ifndef GPF_UTIL_H
#define GPF_UTIL_H

#define GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH   14 //Sembre être le maximum qui s'affiche sur ma radion OpenTx RadioMaster TX16S (13 chars + null= 14)

#define GPF_UTIL_BEEP_TONE_ERROR           100 //hz
#define GPF_UTIL_BEEP_TONE_INFO            500 //hz
#define GPF_UTIL_BEEP_TONE_ALARM          1000 //hz

#define GPF_UTIL_BEEP_DURATION_SHORT        75 //ms
#define GPF_UTIL_BEEP_DURATION_LONG        150 //ms
#define GPF_UTIL_BEEP_DURATION_ULTRA_LONG 1000 //ms

void gpf_util_blinkMainBoardLed(uint8_t nbrBlink);

uint16_t gpf_util_shiftBitsToBigEndian_16(uint16_t val);
//int32_t gpf_util_shiftBitsToBigEndian_int32_t(int32_t);
uint32_t gpf_util_shiftBitsToBigEndian_32(uint32_t val);

int    gpf_util_freeRam();
bool   gpf_util_isPwmChannelAtPos(int currentPos, gpf_rc_channel_position_type_enum posRequired);
void   gpf_util_resetConfigToDefault(gpf_config_struct *ptr);
time_t gpf_util_getTeensy3Time();
void   gpf_util_beep(uint16_t frequency, uint32_t duration = 0UL);
float  gpf_util_invSqrt(float x);


#endif