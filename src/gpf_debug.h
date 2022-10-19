/**
 * @file gpf_debug.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-07
 */

#include "Arduino.h"

#define DEBUG_GPF_ENABLED
#define DEBUG_GPF_DELAY            1000

//#define DEBUG_GPF_CRSF_ENABLED
#define DEBUG_GPF_CRSF_DELAY       1000

#define DEBUG_GPF_MPU6050_ENABLED
#define DEBUG_GPF_MPU6050_DELAY    1000


#ifdef DEBUG_GPF_ENABLED
 #define DebugStream_GPF                   Serial //Port USB
 #define DEBUG_GPF_PRINT(...)              DebugStream_GPF.print(__VA_ARGS__)
 #define DEBUG_GPF_PRINTLN(...)            DebugStream_GPF.println(__VA_ARGS__)
#else
 #define DebugStream_GPF              
 #define DEBUG_GPF_PRINT(...)         
 #define DEBUG_GPF_PRINTLN(...)       
#endif


#ifdef DEBUG_GPF_CRSF_ENABLED
 #define DebugStream_GPF_CRSF              Serial //Port USB
 #define DEBUG_GPF_CRSF_PRINT(...)         DebugStream_GPF_CRSF.print(__VA_ARGS__)
 #define DEBUG_GPF_CRSF_PRINTLN(...)       DebugStream_GPF_CRSF.println(__VA_ARGS__)
#else
 #define DebugStream_GPF_CRSF              
 #define DEBUG_GPF_CRSF_PRINT(...)         
 #define DEBUG_GPF_CRSF_PRINTLN(...)       
#endif

#ifdef DEBUG_GPF_MPU6050_ENABLED
 #define DebugStream_GPF_MPU6050           Serial //Port USB
 #define DEBUG_GPF_MPU6050_PRINT(...)      DebugStream_GPF_MPU6050.print(__VA_ARGS__)
 #define DEBUG_GPF_MPU6050_PRINTLN(...)    DebugStream_GPF_MPU6050.println(__VA_ARGS__)
#else
 #define DebugStream_GPF_MPU6050
 #define DEBUG_GPF_MPU6050_PRINT(...)         
 #define DEBUG_GPF_MPU6050_PRINTLN(...)       
#endif
