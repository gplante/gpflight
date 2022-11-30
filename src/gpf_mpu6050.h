/**
 * @file gpf_mpu6050.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-01-06 
 * 
 * Voir gpf_mpu6050.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_MPU6050_H
#define GPF_MPU6050_H

#include "Arduino.h"
#include "gpf_cons.h"

#define GPF_MPU6050_ADDRESS_AD0_LOW         0x68 // address pin low (GND), default for InvenSense evaluation board
#define GPF_MPU6050_ADDRESS_AD0_HIGH        0x69 // address pin high (VCC)
#define GPF_MPU6050_DEFAULT_ADDRESS         GPF_MPU6050_ADDRESS_AD0_LOW
#define GPF_MPU6050_RA_WHO_AM_I             0x75
#define GPF_MPU6050_WHO_AM_I_BIT            6
#define GPF_MPU6050_WHO_AM_I_LENGTH         6
#define GPF_MPU6050_RA_PWR_MGMT_1           0x6B
#define GPF_MPU6050_PWR1_CLKSEL_BIT         2
#define GPF_MPU6050_PWR1_CLKSEL_LENGTH      3
#define GPF_MPU6050_RA_GYRO_CONFIG          0x1B
#define GPF_MPU6050_GCONFIG_FS_SEL_BIT      4
#define GPF_MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define GPF_MPU6050_RA_ACCEL_CONFIG         0x1C
#define GPF_MPU6050_ACONFIG_AFS_SEL_BIT     4
#define GPF_MPU6050_ACONFIG_AFS_SEL_LENGTH  2
#define GPF_MPU6050_RA_PWR_MGMT_1           0x6B
#define GPF_MPU6050_PWR1_SLEEP_BIT          6
#define GPF_MPU6050_CLOCK_PLL_XGYRO         0x01
#define GPF_MPU6050_GYRO_FS_250             0x00
#define GPF_MPU6050_GYRO_FS_500             0x01
#define GPF_MPU6050_GYRO_FS_1000            0x02
#define GPF_MPU6050_GYRO_FS_2000            0x03
#define GPF_MPU6050_ACCEL_FS_2              0x00
#define GPF_MPU6050_ACCEL_FS_4              0x01
#define GPF_MPU6050_ACCEL_FS_8              0x02
#define GPF_MPU6050_ACCEL_FS_16             0x03
#define GPF_MPU6050_RA_ACCEL_XOUT_H         0x3B

#define GPF_MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define GPF_MPU6050_RA_XA_OFFS_L_TC     0x07
#define GPF_MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define GPF_MPU6050_RA_YA_OFFS_L_TC     0x09
#define GPF_MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define GPF_MPU6050_RA_ZA_OFFS_L_TC     0x0B

#define GPF_MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define GPF_MPU6050_RA_XG_OFFS_USRL     0x14
#define GPF_MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define GPF_MPU6050_RA_YG_OFFS_USRL     0x16
#define GPF_MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define GPF_MPU6050_RA_ZG_OFFS_USRL     0x18

#define GPF_MPU6050_GYRO_FS_250_SENSITIVITY     131.0 //LSB/dps
#define GPF_MPU6050_GYRO_FS_500_SENSITIVITY      65.5 //LSB/dps
#define GPF_MPU6050_GYRO_FS_1000_SENSITIVITY     32.8 //LSB/dps
#define GPF_MPU6050_GYRO_FS_2000_SENSITIVITY     16.4 //LSB/dps

#define GPF_MPU6050_ACCEL_FS_2_SENSITIVITY    16384.0 //LSB/g
#define GPF_MPU6050_ACCEL_FS_4_SENSITIVITY     8192.0 //LSB/g
#define GPF_MPU6050_ACCEL_FS_8_SENSITIVITY     4096.0 //LSB/g
#define GPF_MPU6050_ACCEL_FS_16_SENSITIVITY    2048.0 //LSB/g

#define GPF_IMU_LOW_PASS_FILTER_WEIGHT_OLD        0.95
#define GPF_IMU_LOW_PASS_FILTER_WEIGHT_CURRENT    0.05

#define GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_NO_FILTER                  0
#define GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_LOW_PASS_FILTER            1
#define GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER           2
#define GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER               3
#define GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER  4


class GPF_MPU6050 {
    public:
        GPF_MPU6050();

        void initialize(gpf_config_struct *);        
        void setClockSource(uint8_t);
        void setSleepEnabled(bool);

        void setFullScaleAccelRange(uint8_t);
        void setFullScaleGyroRange(uint8_t);
        void setComplementaryFilterGyroWeightPercent(float);

        void setOrientationCalcMode(uint8_t);

        uint8_t getDeviceID();
        bool getMotion6(int16_t*, int16_t*, int16_t*, int16_t*, int16_t*, int16_t*);
        void readSensorsAndDoCalculations();

        float convert_90_90_to_0_360(float, float);
        float convert_0_360_to_90_90(float);

/*
        // XA_OFFS_* registers
        int16_t getXAccelOffset();
        void setXAccelOffset(int16_t offset);

        // YA_OFFS_* register
        int16_t getYAccelOffset();
        void setYAccelOffset(int16_t offset);

        // ZA_OFFS_* register
        int16_t getZAccelOffset();
        void setZAccelOffset(int16_t offset);

        // XG_OFFS_USR* registers
        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);

        // YG_OFFS_USR* register
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);

        // ZG_OFFS_USR* register
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);
*/

        void calibrate();
        void meansensors();

        int8_t orientationCalcMode = GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER;
        float complementaryFilterGyroWeightPercent;
        float complementaryFilterGyroWeight;
        float complementaryFilterAccWeight;
        
        float acc_pitch_radiant;
        float acc_pitch_degree;
        float acc_pitch_degree_0_360_z_normal;
        float acc_pitch_degree_0_360_z_inversed;
        float acc_pitch_degree_old; //sert dans le mode low pass filter

        float acc_roll_radiant;
        float acc_roll_degree;
        float acc_roll_degree_0_360_z_normal;
        float acc_roll_degree_0_360_z_inversed;
        float acc_roll_degree_old; //sert dans le mode low pass filter

        float acc_z_radiant;
        float acc_z_degree;
        float acc_z_degree_0_360;

        float gyr_pitch_degree = 0;
        float gyr_roll_degree = 0;
        float gyr_pitch_degree_sum = 0;
        float gyr_roll_degree_sum = 0;

        float cadranPitchFilter;
        float cadranPitchAcc;
        float cadranPitchDelta_used;
        float cadranPitchDelta_z_normal;
        float cadranPitchDelta_z_inversed;

        float cadranRollFilter;
        float cadranRollAcc;
        float cadranRollDelta_used;
        float cadranRollDelta_z_normal;
        float cadranRollDelta_z_inversed;

        float complementary_filter_0_360_pitch = 0;
        float complementary_filter_0_360_roll = 0;

        float time_elapsed;
        unsigned long millis_old = 0;
        float gyr_lsb_sensitivity;
        float acc_lsb_sensitivity;

        float output_pitch; // Normalement de -180 à 180 degrés mais peut-être plus dans certains modes test mais pas dans le mode GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER
        float output_roll;  // Normalement de -180 à 180 degrés mais peut-être plus dans certains modes test mais pas dans le mode GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER

        int16_t calibration_offset_ax,calibration_offset_ay,calibration_offset_az,calibration_offset_gx,calibration_offset_gy,calibration_offset_gz;
        int16_t accX_with_offsets,   accY_with_offsets,   accZ_with_offsets,   gyrX_with_offsets,   gyrY_with_offsets,   gyrZ_with_offsets;        
    private:
        gpf_config_struct *myConfig_ptr = NULL;

        uint8_t devAddr;
        uint8_t buffer[14];
        elapsedMillis debug_sincePrint;

        int16_t accX_raw_no_offsets, accY_raw_no_offsets, accZ_raw_no_offsets, gyrX_raw_no_offsets, gyrY_raw_no_offsets, gyrZ_raw_no_offsets;
        
};

#endif