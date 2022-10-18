/**
 * @file gpf_mpu6050.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-01-06 
 */

#include "Arduino.h"

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

        void initialize();        
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

        // DMP_CFG_2 register
        uint8_t getDMPConfig2();
        void setDMPConfig2(uint8_t config);

        int8_t orientationCalcMode = GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER;
        float complementaryFilterGyroWeightPercent;
        float complementaryFilterGyroWeight;
        float complementaryFilterAccWeight;

        int16_t accX, accY, accZ;
        int16_t gyrX, gyrY, gyrZ;
        unsigned long accX2, accY2, accZ2;
        unsigned long errCpt1 = 0, errCpt2 = 0, errCpt3 = 0;
        float acc_pitch_radiant;
        float acc_pitch_degree;
        //float acc_pitch_degree_0_360;
        float acc_pitch_degree_0_360_z_normal;
        float acc_pitch_degree_0_360_z_inversed;
        float acc_pitch_degree_old;

        float acc_roll_radiant;
        float acc_roll_degree;
        //float acc_roll_degree_0_360;
        float acc_roll_degree_0_360_z_normal;
        float acc_roll_degree_0_360_z_inversed;
        float acc_roll_degree_old;

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

        unsigned long millis_timer_timeToprint = 0;
    private:
        uint8_t devAddr;
        uint8_t buffer[14];
        elapsedMillis debug_sincePrint;
        
};
