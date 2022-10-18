/**
 * @file gpf_debug.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-07
 */

#include "Arduino.h"

#define DEBUG_ENABLED

#define DEBUG_CRSF_ENABLED
#define DEBUG_CRSF_DELAY    1000

#define DEBUG_MPU6050_ENABLED
#define DEBUG_MPU6050_DELAY    1000


#ifdef DEBUG_ENABLED
 #define DebugStream              Serial //Port USB
 #define DEBUG_PRINT(...)         DebugStream.print(__VA_ARGS__)
 #define DEBUG_PRINTLN(...)       DebugStream.println(__VA_ARGS__)
#else
 #define DebugStream              
 #define DEBUG_PRINT(...)         
 #define DEBUG_PRINTLN(...)       
#endif


#ifdef DEBUG_CRSF_ENABLED
 #define DebugStream_CRSF              Serial //Port USB
 #define DEBUG_CRSF_PRINT(...)         DebugStream_CRSF.print(__VA_ARGS__)
 #define DEBUG_CRSF_PRINTLN(...)       DebugStream_CRSF.println(__VA_ARGS__)
#else
 #define DebugStream_CRSF              
 #define DEBUG_CRSF_PRINT(...)         
 #define DEBUG_CRSF_PRINTLN(...)       
#endif

#ifdef DEBUG_MPU6050_ENABLED
 #define DebugStream_MPU6050           Serial //Port USB
 #define DEBUG_MPU6050_PRINT(...)      DebugStream_MPU6050.print(__VA_ARGS__)
 #define DEBUG_MPU6050_PRINTLN(...)    DebugStream_MPU6050.println(__VA_ARGS__)
#else
 #define DebugStream_MPU6050
 #define DEBUG_MPU6050_PRINT(...)         
 #define DEBUG_MPU6050_PRINTLN(...)       
#endif
