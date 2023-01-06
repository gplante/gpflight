/**
 * @file gpf_imu.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2023-01-02
 * 
 * Voir gpf_imu.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_IMU_H
#define GPF_IMU_H

#include "Arduino.h"
#include "gpf_cons.h"
#include "MPU6050.h"

//Uncomment only one full scale gyro range (deg/sec)
//#define GPF_IMU_GYRO_250DPS //Default
//#define GPF_IMU_GYRO_500DPS
#define GPF_IMU_GYRO_1000DPS
//#define GPF_IMU_GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
//#define GPF_IMU_ACCEL_2G //Default
#define GPF_IMU_ACCEL_4G
//#define GPF_IMU_ACCEL_8G
//#define GPF_IMU_ACCEL_16G

#define GPF_IMU_GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GPF_IMU_GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GPF_IMU_GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GPF_IMU_GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define GPF_IMU_ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define GPF_IMU_ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define GPF_IMU_ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define GPF_IMU_ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

#if defined GPF_IMU_GYRO_250DPS
  #define GPF_IMU_GYRO_SCALE        GPF_IMU_GYRO_FS_SEL_250
  #define GPF_IMU_GYRO_SCALE_FACTOR 131.0
#elif defined GPF_IMU_GYRO_500DPS
  #define GPF_IMU_GYRO_SCALE        GPF_IMU_GYRO_FS_SEL_500
  #define GPF_IMU_GYRO_SCALE_FACTOR 65.5
#elif defined GPF_IMU_GYRO_1000DPS
  #define GPF_IMU_GYRO_SCALE        GPF_IMU_GYRO_FS_SEL_1000
  #define GPF_IMU_GYRO_SCALE_FACTOR 32.8
#elif defined GPF_IMU_GYRO_2000DPS
  #define GPF_IMU_GYRO_SCALE        GPF_IMU_GYRO_FS_SEL_2000
  #define GPF_IMU_GYRO_SCALE_FACTOR 16.4
#endif

#if defined GPF_IMU_ACCEL_2G
  #define GPF_IMU_ACCEL_SCALE        GPF_IMU_ACCEL_FS_SEL_2
  #define GPF_IMU_ACCEL_SCALE_FACTOR 16384.0
#elif defined GPF_IMU_ACCEL_4G
  #define GPF_IMU_ACCEL_SCALE        GPF_IMU_ACCEL_FS_SEL_4
  #define GPF_IMU_ACCEL_SCALE_FACTOR 8192.0
#elif defined GPF_IMU_ACCEL_8G
  #define GPF_IMU_ACCEL_SCALE        GPF_IMU_ACCEL_FS_SEL_8
  #define GPF_IMU_ACCEL_SCALE_FACTOR 4096.0
#elif defined GPF_IMU_ACCEL_16G
  #define GPF_IMU_ACCEL_SCALE        GPF_IMU_ACCEL_FS_SEL_16
  #define GPF_IMU_ACCEL_SCALE_FACTOR 2048.0
#endif

#define GPF_IMU_FUSION_TYPE_MADGWICK              0
#define GPF_IMU_FUSION_TYPE_COMPLEMENTARY_FILTER  1

#define GPF_IMU_FUSION_TYPE_SELECTED              GPF_IMU_FUSION_TYPE_COMPLEMENTARY_FILTER

#define GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER  0.995 // Min 0, Max 1


class GPF_IMU {
    public:
        GPF_IMU();

        void initialize(gpf_config_struct *);        
        bool getIMUData();
        void doFusion_madgwick6DOF();
        void doFusion_complementaryFilter();

        void calibrate();
        void meansensors();

        int16_t calibration_offset_ax, calibration_offset_ay, calibration_offset_az, calibration_offset_gx, calibration_offset_gy, calibration_offset_gz;
        int16_t accX_raw_no_offsets,   accY_raw_no_offsets,   accZ_raw_no_offsets,   gyrX_raw_no_offsets,   gyrY_raw_no_offsets,   gyrZ_raw_no_offsets;
        int16_t accX_raw_plus_offsets, accY_raw_plus_offsets, accZ_raw_plus_offsets, gyrX_raw_plus_offsets, gyrY_raw_plus_offsets, gyrZ_raw_plus_offsets;
        float fusion_degree_roll, fusion_degree_pitch, fusion_degree_yaw; //degres

        float accX_output,      accY_output,      accZ_output;
        float gyrX_output,      gyrY_output,      gyrZ_output;
        
        unsigned long errorCount = 0;
    private:
        gpf_config_struct *myConfig_ptr = NULL;

        MPU6050 theImu;

        uint8_t buffer[14];
        elapsedMillis debug_sincePrint;

        

        float accX_output_prev, accY_output_prev, accZ_output_prev;        
        float gyrX_output_prev, gyrY_output_prev, gyrZ_output_prev;
        
        float q0 = 1.0f; //Initialize quaternion for madgwick filter
        float q1 = 0.0f;
        float q2 = 0.0f;
        float q3 = 0.0f;

        //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
        float B_madgwick = 0.99; //0.04 //Madgwick filter parameter //Higher B madgwick leads to a noisier estimate, while lower B madgwick leads to a slower to respond estimate.
        float B_accel = 0.14;    //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
        float B_gyro = 0.1;      //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
        
};

#endif