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

void gpf_util_blinkMainBoardLed(uint8_t nbrBlink);

uint16_t gpf_util_shiftBitsToBigEndian_16(uint16_t val);
//int32_t gpf_util_shiftBitsToBigEndian_int32_t(int32_t);
uint32_t gpf_util_shiftBitsToBigEndian_32(uint32_t val);

int   gpf_util_freeRam();
bool  gpf_util_isPwmChannelAtPos(int currentPos, int posRequired);
void  gpf_util_resetConfigToDefault(gpf_config_struct *ptr);

#endif