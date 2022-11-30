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
       uint16_t battery_voltage;           // Voltage (volt * 10)
       uint16_t battery_current;           // Current (amp * 10)
       uint32_t battery_capacity_used;     // 1=1
       uint8_t  battery_remaining_percent; // 1=1

       int16_t  vario_vertival_speed;

       int32_t  gps_latitude; // degree / 10`000`000
       int32_t  gps_longitude; // degree / 10`000`000
       uint16_t gps_groundspeed; // km/h / 100
       uint16_t gps_heading; // degree * 100
       uint16_t gps_altitude; // meter 
       uint8_t  gps_satellites; // # of sats in view

       uint16_t baro_altitude; //100 = 10 mètres, 1235=123.5 mètres

       int16_t  attitude_pitch; // Pitch angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       int16_t  attitude_roll;  // Roll  angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       int16_t  attitude_yaw;   // Yaw   angle ( rad / 10000 ) //WARNING: Angle values must be in -180° +180° range!
       
       char     flight_mode_description[GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH]; 
};

#endif