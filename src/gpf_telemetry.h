/**
 * @file gpf_telemetry.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-24
 */

#ifndef GPF_TELEMETRY_H
#define GPF_TELEMETRY_H

#include "gpf_util.h"

struct gpf_telemetry_info_s {
       uint16_t battery_voltage          = 0; // Voltage (volt * 10)
       uint16_t battery_current          = 0; // Current (amp * 10)
       uint32_t battery_capacity_used    = 0; // 1=1
       uint8_t  battery_remaining_percent= 0; // 1=1

       int16_t  vario_vertival_speed = 0;

       int32_t  gps_latitude     = 0; // degree / 10`000`000
       int32_t  gps_longitude    = 0; // degree / 10`000`000
       uint16_t gps_groundspeed  = 0; // km/h / 100
       uint16_t gps_heading      = 0; // degree * 100
       uint16_t gps_altitude     = 0; // meter 
       uint8_t  gps_satellites   = 0; // # of sats in view

       uint16_t baro_altitude    = 0; //100 = 10 mètres, 1235=123.5 mètres

       int16_t  attitude_pitch   = 0; // Pitch angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       int16_t  attitude_roll    = 0;  // Roll  angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       int16_t  attitude_yaw     = 0;   // Yaw   angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       
       char     flight_mode_description[GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH]; 
};

#endif