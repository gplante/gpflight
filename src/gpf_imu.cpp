/**
 * @file gpf_imu.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2023-01-02
 *  
 * J'ai appris beaucoup sur les IMU en écoutant cette série de vidéos https://toptechboy.com/arduino-based-9-axis-inertial-measurement-unit-imu-based-on-bno055-sensor/
 * Je me suis servi de ses idées pour faire mon Complementary Filter.
 * J'ai aussi emprunté une partie de code de l'excellent projet https://github.com/nickrehm/dRehmFlight/
 */
 
#include "Arduino.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMI088.h"
#include "gpf_imu.h"
#include "gpf_util.h"
#include "gpf_debug.h"

GPF_IMU::GPF_IMU() {
    //
}

void GPF_IMU::initialize(gpf_config_struct *ptr) {
    myConfig_ptr = ptr;

    #if defined GPF_IMU_SENSOR_INSTALLED_MPU6050
     theImu_mpu6050.initialize();
     delay(5); //delay(1); //Patch pour IMU de type GY-521 sinon setFullScaleAccelRange() n'est pas pris en considération ci-dessous et reste à MPU6050_ACCEL_FS_2

     theImu_mpu6050.setFullScaleGyroRange(GPF_IMU_MPU6050_GYRO_SCALE);
     theImu_mpu6050.setFullScaleAccelRange(GPF_IMU_MPU6050_ACCEL_SCALE);
     theImu_mpu6050.setRate(7);

     #ifdef DEBUG_GPF_IMU_ENABLED
     
      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getDeviceID(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu_mpu6050.getDeviceID(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();
      
      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getDLPFMode(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu_mpu6050.getDLPFMode(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();

      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getRate(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu_mpu6050.getRate(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();

      //DEBUG_GPF_IMU_PRINT(F("IMU:"));
      //DEBUG_GPF_IMU_PRINT(F("getFIFOEnabled(): hex:"));
      //DEBUG_GPF_IMU_PRINT(theImu_mpu6050.getFIFOEnabled(), HEX);     
      //DEBUG_GPF_IMU_PRINTLN();
     
     #endif
    #endif

    #if defined GPF_IMU_SENSOR_INSTALLED_BMI088
     int  status_i;
     bool status_b;

      #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_A
       theImu_bmi088_accel = new Bmi088Accel(Wire,0x19); //0x18
       theImu_bmi088_gyro  = new Bmi088Gyro(Wire,0x69); //0x68

       status_i = theImu_bmi088_accel->begin();
       status_b = theImu_bmi088_accel->setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
       status_b = theImu_bmi088_accel->setRange(GPF_IMU_BMI088_ACCEL_SCALE);

       if (status_i < 0) {
         #ifdef DEBUG_GPF_IMU_ENABLED
          DEBUG_GPF_IMU_PRINT("Accel Initialization Error");
          DEBUG_GPF_IMU_PRINTLN(status_i);      
         #endif
       }

       status_i = theImu_bmi088_gyro->begin();
       if (status_i < 0) {
         #ifdef DEBUG_GPF_IMU_ENABLED
          DEBUG_GPF_IMU_PRINT("Gyro Initialization Error");
          DEBUG_GPF_IMU_PRINTLN(status_i);      
         #endif
       }

       status_b = theImu_bmi088_gyro->setOdr(Bmi088Gyro::ODR_2000HZ_BW_532HZ);
       status_b = theImu_bmi088_gyro->setRange(GPF_IMU_BMI088_GYRO_SCALE); 

       if (status_b) {
        #ifdef DEBUG_GPF_IMU_ENABLED
          DEBUG_GPF_IMU_PRINT("status_b=");
          DEBUG_GPF_IMU_PRINTLN(status_b);      
        #endif
       }
      #endif

      #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_B //Pour fin de tests seulement
       theImu_bmi088_bmi = new Bmi088(Wire,0x19,0x69);

       status_i = theImu_bmi088_bmi->begin();
       status_b = theImu_bmi088_bmi->setOdr(Bmi088::ODR_2000HZ);
       status_b = theImu_bmi088_bmi->setRange(Bmi088::ACCEL_RANGE_3G,Bmi088::GYRO_RANGE_500DPS);

       if (status_i < 0) {
        #ifdef DEBUG_GPF_IMU_ENABLED
         DEBUG_GPF_IMU_PRINT("Bmi Initialization Error");
         DEBUG_GPF_IMU_PRINTLN(status_i);      
        #endif
       }
      #endif
    
    #endif
    
}


bool GPF_IMU::getIMUData() {

    // Cette fonction, légèrement adaptée pour ce projet, provient du projet dRehmFlight VTOL Flight Controller de Nicholas Rehm à https://github.com/nickrehm/dRehmFlight
    // Il n'y a cependant pas de boussole dans ce cas-ci.

    //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
    /*
     * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
     * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
     * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
     * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
     * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
     * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
     */

    accX_raw_no_offsets = 0;
    accY_raw_no_offsets = 0;
    accZ_raw_no_offsets = 0;
    gyrX_raw_no_offsets = 0;
    gyrY_raw_no_offsets = 0;
    gyrZ_raw_no_offsets = 0;

    #if defined GPF_IMU_SENSOR_INSTALLED_MPU6050
     theImu_mpu6050.getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
    #endif

    #if defined GPF_IMU_SENSOR_INSTALLED_BMI088
     #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_A
       theImu_bmi088_accel->readSensor();
       theImu_bmi088_accel->getSensorRawValues(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets);
       theImu_bmi088_gyro->readSensor();
       theImu_bmi088_gyro->getSensorRawValues(&gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
     #endif

     #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_B
      theImu_bmi088_bmi->readSensor();
      theImu_bmi088_bmi->getSensorRawValues(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
     #endif
    #endif
    
    // *** Protection *** 
    //Protection si on ne peu pas lire le IMU. C'est mieux de sortir de la fonction que de faire des calculs erronés
    #if defined GPF_IMU_SENSOR_INSTALLED_MPU6050
     if(((accX_raw_no_offsets == 0) && (accY_raw_no_offsets == 0) && (accZ_raw_no_offsets == 0)) || ((gyrX_raw_no_offsets == 0) && (gyrY_raw_no_offsets == 0) && (gyrZ_raw_no_offsets == 0))) {
       DEBUG_GPF_IMU_PRINT(F("GPF_IMU:"));
       DEBUG_GPF_IMU_PRINTLN(F("***** ERREUR (Valeurs à 0) ******"));
       errorCount++;
       return false;
     }
    #endif

    // **************************************************************************************
    // On dirait que des fois, la fonction getMotion6() retournait des valeurs à 0 puis ensuite les calculs donnaient comme résultat Nan.
    // Puisque dans la boucle, on se sert toujours des valauers des calculs précédents pour refaire les nouveaux calculs et bien les nouveaux calculs restaient toujours à Nan.
    // Donc au lieu d'obtenir des calculs à Nan on ne fait pas les calculs puis on sort immédiatement de la fonction.
    if (isnan(atan(accX_raw_no_offsets/sqrt((accY_raw_no_offsets*accY_raw_no_offsets)+(accZ_raw_no_offsets*accZ_raw_no_offsets))))) {
     DEBUG_GPF_IMU_PRINT(F("GPF_IMU:"));
     DEBUG_GPF_IMU_PRINTLN(F("***** ERREUR (Calcul donne NAN) ******"));
     errorCount++;
     return false;
    }
    // *** Fin Protection *** 


    //Accelerometer
    //Correct the outputs with the calculated error values
    accX_raw_plus_offsets = accX_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]  );
    accY_raw_plus_offsets = accY_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]  );
    accZ_raw_plus_offsets = accZ_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]  );

    //Accelerometer
    accX_output = accX_raw_plus_offsets / GPF_IMU_ACCEL_SCALE_FACTOR; //G's
    accY_output = accY_raw_plus_offsets / GPF_IMU_ACCEL_SCALE_FACTOR; //G's
    accZ_output = accZ_raw_plus_offsets / GPF_IMU_ACCEL_SCALE_FACTOR; //G's
  
    if (fusion_type == GPF_IMU_FUSION_TYPE_MADGWICK) {
     //LP filter accelerometer data
     accX_output = (1.0 - B_accel)*accX_output_prev + B_accel*accX_output;
     accY_output = (1.0 - B_accel)*accY_output_prev + B_accel*accY_output;
     accZ_output = (1.0 - B_accel)*accZ_output_prev + B_accel*accZ_output;
     accX_output_prev = accX_output;
     accY_output_prev = accY_output;
     accZ_output_prev = accZ_output;
    }

    //Gyro
    //Correct the outputs with the calculated error values
    gyrX_raw_plus_offsets = gyrX_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]  );
    gyrY_raw_plus_offsets = gyrY_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]  );
    gyrZ_raw_plus_offsets = gyrZ_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]  );

    gyrX_output = gyrX_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    gyrY_output = gyrY_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    gyrZ_output = gyrZ_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    
    if (fusion_type == GPF_IMU_FUSION_TYPE_MADGWICK) {
     //LP filter gyro data
     gyrX_output = (1.0 - B_gyro)*gyrX_output_prev + B_gyro*gyrX_output;
     gyrY_output = (1.0 - B_gyro)*gyrY_output_prev + B_gyro*gyrY_output;
     gyrZ_output = (1.0 - B_gyro)*gyrZ_output_prev + B_gyro*gyrZ_output;
     gyrX_output_prev = gyrX_output;
     gyrY_output_prev = gyrY_output;
     gyrZ_output_prev = gyrZ_output;
    }


/*
    #ifdef DEBUG_GPF_IMU_ENABLED
     if (debug_sincePrint > DEBUG_GPF_IMU_DELAY) {
       DEBUG_GPF_IMU_PRINT(accX_output);            DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(accY_output);            DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(accZ_output);            DEBUG_GPF_IMU_PRINT(F(", "));       

       //DEBUG_GPF_IMU_PRINT(GyroX);            DEBUG_GPF_IMU_PRINT(F(", "));       
       //DEBUG_GPF_IMU_PRINT(GyroY);            DEBUG_GPF_IMU_PRINT(F(", "));       
       //DEBUG_GPF_IMU_PRINT(GyroZ);            DEBUG_GPF_IMU_PRINT(F(", "));       
       
       DEBUG_GPF_IMU_PRINTLN();

       debug_sincePrint = 0;
     }
    #endif
*/
    return true;
}

void GPF_IMU::set_fusion_type(uint8_t flight_mode) {
  if (flight_mode == GPF_FLIGHT_MODE_3_FUSION_TYPE_MADGWICK) {
   fusion_type = GPF_IMU_FUSION_TYPE_MADGWICK; //fm-3
  } else {
   fusion_type = GPF_IMU_FUSION_TYPE_COMPLEMENTARY_FILTER; //fm-2 et fm-1
  }
}

void GPF_IMU::doFusion() {
  
 if (fusion_type == GPF_IMU_FUSION_TYPE_MADGWICK) {
  doFusion_madgwick6DOF();
 } else {
  doFusion_complementaryFilter();
 }

}

void GPF_IMU::doFusion_madgwick6DOF() {
  // Cette fonction, légèrement adaptée pour ce projet, provient du projet dRehmFlight VTOL Flight Controller de Nicholas Rehm à https://github.com/nickrehm/dRehmFlight

  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   * https://github.com/nickrehm/dRehmFlight/blob/master/dRehmFlight%20VTOL%20Documentation.pdf
   */

  float gx =  gyrX_output;
  float gy = -gyrY_output; 
  float gz = -gyrZ_output;
  float ax = -accX_output; 
  float ay =  accY_output; 
  float az =  accZ_output; 

  static unsigned long micros_previous = 0;
  unsigned long current_time = micros();
  float time_elapsed = (current_time - micros_previous)/1000000.0;
  micros_previous = micros();

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = gpf_util_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = gpf_util_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * time_elapsed;
  q1 += qDot2 * time_elapsed;
  q2 += qDot3 * time_elapsed;
  q3 += qDot4 * time_elapsed;

  //Normalise quaternion
  recipNorm = gpf_util_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  fusion_degree_roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  fusion_degree_pitch = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  fusion_degree_yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees

  #ifdef DEBUG_GPF_IMU_ENABLED
     if (debug_sincePrint > DEBUG_GPF_IMU_DELAY) {
       DEBUG_GPF_IMU_PRINT(fusion_degree_roll);            DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(fusion_degree_pitch);           DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(fusion_degree_yaw);             DEBUG_GPF_IMU_PRINT(F(", "));       
       
       DEBUG_GPF_IMU_PRINTLN();

       debug_sincePrint = 0;
     }
  #endif

}

void GPF_IMU::doFusion_complementaryFilter() {
    // Simple filtre complémentaire par Guylain Plante:
    //
    // Je fait un low pass filter sur l'accéléromètre parceque l'accéléromètre est très sensible aux vibrations.
    // (Donc non fiable sur des lectures à court terme mais fiable sur des lectures à long terme).
    //
    // Je fait un high pass filter sur le gyro car celui-ci n'est pas sensible aux vibrations mais malheureusement, il "drift" (dérive) dans le temps.
    // (Donc fiable sur des lectures à court terme mais non fiable sur des lectures à long terme à cause du "drift")
    // (Drift is the term used to describe the accumulation of small errors)
    //
    // En termes clairs, je me fie à la mesure du gyro à court terme mais à long terme je me fie à l'accéléromètre.
    // C'est pour celà que nous devons "fusionner" les valeurs du gyro et de l'accéléromètre pour avoir une mesure relativement fiable.
    //
    // Les moteurs du drone causent beaucoup de vibrations d'ou le besoin de fusionner les deux sensors. 
    // Cependant, juste à titre indicatif, dans une autre application ou il n'y aurait pas de vibrations de moteurs ou autre, et qu'il n'y aurait pas 
    // de mouvements brusques non plus, nous pourrions dans ce cas se fier uniquement à l'accéléromètre qui nous retourne 
    // la force de gravité. Le problème avec les mouvements brusques et/ou vibrations c'est que dans un tel cas, 
    // l'accéléromère retourne une force de gravité qui inclus la force d'accélération de la vibration/mouvement. Nous ce qu'on a besoin dans ce cas-ci
    // c'est uniquement la force de gravité pour connaitre l'attitude du drone.
    //
    // Une fois mon IMU calibré, si je ne déplace pas l'IMU, mes tests indiquent que le gyro dérive d'environ 0.01 degrés 
    // lorsque je fait 2000 lectures par seconde.    

    float gx =  gyrX_output;
    float gy = -gyrY_output; 
    //float gz = -gyrZ_output;
    float ax = -accX_output; 
    float ay =  accY_output; 
    float az =  accZ_output; 

    float acc_pitch_radiant, acc_roll_radiant, acc_yaw_radiant;
    float acc_pitch_degree,  acc_roll_degree,  acc_yaw_degree;
    float gyr_pitch_degree,  gyr_roll_degree;

    float fusion_degree_pitch_temp, fusion_degree_roll_temp, fusion_degree_yaw_temp;

    static unsigned long micros_previous = 0;
    unsigned long current_time = micros();
    float time_elapsed = (current_time - micros_previous)/1000000.0;
    micros_previous = micros();

    // Z Axis (-90 degrés à 90 degrés)
    acc_yaw_radiant = atan(az/sqrt(pow(ax,2)+pow(ay,2))); 
    acc_yaw_degree  = acc_yaw_radiant/2/PI*360;
    fusion_degree_yaw_temp = acc_yaw_degree;

    // X Axis // Z Axis (-90 degrés à 90 degrés)
    acc_pitch_radiant = atan(ax/sqrt(pow(ay,2)+pow(az,2)));
    acc_pitch_degree  = acc_pitch_radiant/2/PI*360;

    // Y Axis // Z Axis (-90 degrés à 90 degrés)
    acc_roll_radiant = atan(ay/sqrt(pow(ax,2)+pow(az,2)));
    acc_roll_degree  = acc_roll_radiant/2/PI*360;

    gyr_pitch_degree = -(gy * time_elapsed);
    gyr_roll_degree  =   gx * time_elapsed;

    fusion_degree_pitch_temp = (GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER*(fusion_degree_pitch + gyr_pitch_degree)) + ((1.0 - GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER)*acc_pitch_degree);
    fusion_degree_roll_temp  = (GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER*(fusion_degree_roll  + gyr_roll_degree))  + ((1.0 - GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER)*acc_roll_degree);

    if ( (!isnan(fusion_degree_pitch_temp)) && (!isnan(fusion_degree_roll_temp)) && (!isnan(fusion_degree_yaw_temp)) ) {
      fusion_degree_pitch = fusion_degree_pitch_temp;
      fusion_degree_roll  = fusion_degree_roll_temp;
      fusion_degree_yaw   = fusion_degree_yaw_temp;
    }
   
    #ifdef DEBUG_GPF_IMU_ENABLED
     if (debug_sincePrint > DEBUG_GPF_IMU_DELAY) {
       //DEBUG_GPF_IMU_PRINT(acc_roll_degree);            DEBUG_GPF_IMU_PRINT(F(", "));       
       //DEBUG_GPF_IMU_PRINT(acc_pitch_degree);           DEBUG_GPF_IMU_PRINT(F(", "));       
       //DEBUG_GPF_IMU_PRINT(acc_yaw_degree);             DEBUG_GPF_IMU_PRINT(F(", "));       

       DEBUG_GPF_IMU_PRINT(fusion_degree_roll);            DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(fusion_degree_pitch);           DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(fusion_degree_yaw);             DEBUG_GPF_IMU_PRINT(F(", "));       

       DEBUG_GPF_IMU_PRINT(gyr_roll_degree);             DEBUG_GPF_IMU_PRINT(F(", "));       
       DEBUG_GPF_IMU_PRINT(gyr_pitch_degree);             DEBUG_GPF_IMU_PRINT(F(", "));       
       
       DEBUG_GPF_IMU_PRINTLN();

       debug_sincePrint = 0;
     }
    #endif

}

// Fonction qui fait plusieurs séries de lecture et garde le min/max de chaque série de lecture puis ensuite
// fait une moyenne qui sera l'offset final.
void GPF_IMU::calibrate() {

  int16_t calibration_offset_ax_min,calibration_offset_ay_min,calibration_offset_az_min,calibration_offset_gx_min,calibration_offset_gy_min,calibration_offset_gz_min;
  int16_t calibration_offset_ax_max,calibration_offset_ay_max,calibration_offset_az_max,calibration_offset_gx_max,calibration_offset_gy_max,calibration_offset_gz_max;

  calibration_offset_ax_min = 32767;
  calibration_offset_ay_min = 32767;
  calibration_offset_az_min = 32767;
  calibration_offset_gx_min = 32767;
  calibration_offset_gy_min = 32767;
  calibration_offset_gz_min = 32767;

  calibration_offset_ax_max = -32767;
  calibration_offset_ay_max = -32767;
  calibration_offset_az_max = -32767;
  calibration_offset_gx_max = -32767;
  calibration_offset_gy_max = -32767;
  calibration_offset_gz_max = -32767;

  calibration_offset_ax = 0;
  calibration_offset_ay = 0;
  calibration_offset_az = 0;
  calibration_offset_gx = 0;
  calibration_offset_gy = 0;
  calibration_offset_gz = 0;

  DEBUG_GPF_IMU_PRINTLN("Calibrating...");
  for (size_t i = 0; i < 10; i++) { //On fait 10 séries de lecture 
  
    meansensors();

    calibration_offset_ax_min = min(calibration_offset_ax_min, calibration_offset_ax);
    calibration_offset_ax_max = max(calibration_offset_ax_max, calibration_offset_ax);

    calibration_offset_ay_min = min(calibration_offset_ay_min, calibration_offset_ay);
    calibration_offset_ay_max = max(calibration_offset_ay_max, calibration_offset_ay);

    calibration_offset_az_min = min(calibration_offset_az_min, calibration_offset_az);
    calibration_offset_az_max = max(calibration_offset_az_max, calibration_offset_az);

    calibration_offset_gx_min = min(calibration_offset_gx_min, calibration_offset_gx);
    calibration_offset_gx_max = max(calibration_offset_gx_max, calibration_offset_gx);

    calibration_offset_gy_min = min(calibration_offset_gy_min, calibration_offset_gy);
    calibration_offset_gy_max = max(calibration_offset_gy_max, calibration_offset_gy);

    calibration_offset_gz_min = min(calibration_offset_gz_min, calibration_offset_gz);
    calibration_offset_gz_max = max(calibration_offset_gz_max, calibration_offset_gz);    
    
    DEBUG_GPF_IMU_PRINT(" moy-min-max ax=");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_ax);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_ax_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_ax_max);
    
    DEBUG_GPF_IMU_PRINT(" ay=");    
    DEBUG_GPF_IMU_PRINT((calibration_offset_ay));
    DEBUG_GPF_IMU_PRINT("/");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_ay_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_ay_max);
    
    DEBUG_GPF_IMU_PRINT(" az=");    
    DEBUG_GPF_IMU_PRINT((calibration_offset_az));
    DEBUG_GPF_IMU_PRINT("/");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_az_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_az_max);

    DEBUG_GPF_IMU_PRINT(" moy-min-max gx=");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_gx);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_gx_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_gx_max);
    
    DEBUG_GPF_IMU_PRINT(" gy=");    
    DEBUG_GPF_IMU_PRINT((calibration_offset_gy));
    DEBUG_GPF_IMU_PRINT("/");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_gy_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_gy_max);
    
    DEBUG_GPF_IMU_PRINT(" gz=");    
    DEBUG_GPF_IMU_PRINT((calibration_offset_gz));
    DEBUG_GPF_IMU_PRINT("/");    
    DEBUG_GPF_IMU_PRINT(calibration_offset_gz_min);
    DEBUG_GPF_IMU_PRINT("/");
    DEBUG_GPF_IMU_PRINT(calibration_offset_gz_max);

    DEBUG_GPF_IMU_PRINTLN();

  }

  //Moyenne du min/max qui sera l'offset final
  calibration_offset_ax = calibration_offset_ax_min + ((calibration_offset_ax_max - calibration_offset_ax_min)/2); 
  calibration_offset_ay = calibration_offset_ay_min + ((calibration_offset_ay_max - calibration_offset_ay_min)/2); 
  calibration_offset_az = GPF_IMU_ACCEL_SCALE_FACTOR - (calibration_offset_az_min + ((calibration_offset_az_max - calibration_offset_az_min)/2)); //Parce que lors de la calibration, le IMU est à plat donc la force de gravité est de 1G dans l'axe Z. Donc GPF_IMU_ACCEL_SCALE_FACTOR correspond à 1G selon la résolution choisie.

  calibration_offset_gx = calibration_offset_gx_min + ((calibration_offset_gx_max - calibration_offset_gx_min)/2); 
  calibration_offset_gy = calibration_offset_gy_min + ((calibration_offset_gy_max - calibration_offset_gy_min)/2); 
  calibration_offset_gz = calibration_offset_gz_min + ((calibration_offset_gz_max - calibration_offset_gz_min)/2); 

  DEBUG_GPF_IMU_PRINTLN();  
  DEBUG_GPF_IMU_PRINTLN("***FIN***");  
  DEBUG_GPF_IMU_PRINTLN("ax min-moy-max\tay\t\taz\t\tgx\t\tgy\tgz\t\t");

  DEBUG_GPF_IMU_PRINT(calibration_offset_ax_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_ax); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_ax_max); 
  DEBUG_GPF_IMU_PRINT("\t");

  DEBUG_GPF_IMU_PRINT(calibration_offset_ay_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_ay); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_ay_max); 
  DEBUG_GPF_IMU_PRINT("\t");

  DEBUG_GPF_IMU_PRINT(calibration_offset_az_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_az); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_az_max); 
  DEBUG_GPF_IMU_PRINT("\t");


  DEBUG_GPF_IMU_PRINT(calibration_offset_gx_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gx); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gx_max); 
  DEBUG_GPF_IMU_PRINT("\t");

  DEBUG_GPF_IMU_PRINT(calibration_offset_gy_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gy); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gy_max); 
  DEBUG_GPF_IMU_PRINT("\t");

  DEBUG_GPF_IMU_PRINT(calibration_offset_gz_min); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gz); 
  DEBUG_GPF_IMU_PRINT(" ");
  DEBUG_GPF_IMU_PRINT(calibration_offset_gz_max); 
  DEBUG_GPF_IMU_PRINT("\t");

  DEBUG_GPF_IMU_PRINTLN();

}

//Fonction qui fait une série de lecture puis fait une moyenne qui sera l'offset
void GPF_IMU::meansensors() {    
   long i = 0;
   long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
   int16_t calibration_buffersize = 250;  

   while (i<(calibration_buffersize+101)) {
    // read raw accel/gyro measurements from device

    #if defined GPF_IMU_SENSOR_INSTALLED_MPU6050
     theImu_mpu6050.getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
    #endif    

    #if defined GPF_IMU_SENSOR_INSTALLED_BMI088
     #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_A
       theImu_bmi088_accel->readSensor();
       theImu_bmi088_accel->getSensorRawValues(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets);
       theImu_bmi088_gyro->readSensor();
       theImu_bmi088_gyro->getSensorRawValues(&gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
     #endif

     #if defined GPF_IMU_SENSOR_INSTALLED_BMI088_B
      theImu_bmi088_bmi->readSensor();
      theImu_bmi088_bmi->getSensorRawValues(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
     #endif
    #endif
    
    if (i>100 && i<=(calibration_buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+accX_raw_no_offsets;
      buff_ay=buff_ay+accY_raw_no_offsets;
      buff_az=buff_az+accZ_raw_no_offsets;
      buff_gx=buff_gx+gyrX_raw_no_offsets;
      buff_gy=buff_gy+gyrY_raw_no_offsets;
      buff_gz=buff_gz+gyrZ_raw_no_offsets;
    }

    if (i==(calibration_buffersize+100)){
      calibration_offset_ax=buff_ax/calibration_buffersize;
      calibration_offset_ay=buff_ay/calibration_buffersize;
      calibration_offset_az=buff_az/calibration_buffersize;
      calibration_offset_gx=buff_gx/calibration_buffersize;
      calibration_offset_gy=buff_gy/calibration_buffersize;
      calibration_offset_gz=buff_gz/calibration_buffersize;
    }

    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
