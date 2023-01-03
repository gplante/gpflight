/**
 * @file gpf_imu.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2023-01-02
 *  
 * Une partie de code emprunté de l'excellent projet https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino
 */
 
#include "Arduino.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "gpf_imu.h"
#include "gpf_util.h"
#include "gpf_debug.h"

GPF_IMU::GPF_IMU() {
    //
}

void GPF_IMU::initialize(gpf_config_struct *ptr) {
    myConfig_ptr = ptr;
    theImu.initialize();
    delay(1); //Patch pour IMU de type GY-521 sinon setFullScaleAccelRange() n'est pas pris en considération ci-dessous et reste à MPU6050_ACCEL_FS_2

    theImu.setFullScaleGyroRange(GPF_IMU_GYRO_SCALE);
    theImu.setFullScaleAccelRange(GPF_IMU_ACCEL_SCALE);
    theImu.setRate(7);
    
    #ifdef DEBUG_GPF_IMU_ENABLED
     
      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getDeviceID(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu.getDeviceID(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();
      
      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getDLPFMode(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu.getDLPFMode(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();

      DEBUG_GPF_IMU_PRINT(F("IMU:"));
      DEBUG_GPF_IMU_PRINT(F("getRate(): hex:"));
      DEBUG_GPF_IMU_PRINT(theImu.getRate(), HEX);      
      DEBUG_GPF_IMU_PRINTLN();

      //DEBUG_GPF_IMU_PRINT(F("IMU:"));
      //DEBUG_GPF_IMU_PRINT(F("getFIFOEnabled(): hex:"));
      //DEBUG_GPF_IMU_PRINT(theImu.getFIFOEnabled(), HEX);     
      //DEBUG_GPF_IMU_PRINTLN();
     
    #endif
}


bool GPF_IMU::getIMUData() {

    accX_raw_no_offsets = 0;
    accY_raw_no_offsets = 0;
    accZ_raw_no_offsets = 0;
    gyrX_raw_no_offsets = 0;
    gyrY_raw_no_offsets = 0;
    gyrZ_raw_no_offsets = 0;

    theImu.getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
    
    // *** Protection *** 
    //Protection si on ne peu pas lire le IMU. C'est mieux de sortir de la fonction que de faire des calculs erronés
    if(((accX_raw_no_offsets == 0) && (accY_raw_no_offsets == 0) && (accZ_raw_no_offsets == 0)) || ((gyrX_raw_no_offsets == 0) && (gyrY_raw_no_offsets == 0) && (gyrZ_raw_no_offsets == 0))) {
      DEBUG_GPF_IMU_PRINT(F("GPF_IMU:"));
      DEBUG_GPF_IMU_PRINTLN(F("***** ERREUR (Valeurs à 0) ******"));
      errorCount++;
      return false;
    }

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
  
    //LP filter accelerometer data
    accX_output = (1.0 - B_accel)*accX_output_prev + B_accel*accX_output;
    accY_output = (1.0 - B_accel)*accY_output_prev + B_accel*accY_output;
    accZ_output = (1.0 - B_accel)*accZ_output_prev + B_accel*accZ_output;
    accX_output_prev = accX_output;
    accY_output_prev = accY_output;
    accZ_output_prev = accZ_output;

    //Gyro
    //Correct the outputs with the calculated error values
    gyrX_raw_plus_offsets = gyrX_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]  );
    gyrY_raw_plus_offsets = gyrY_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]  );
    gyrZ_raw_plus_offsets = gyrZ_raw_no_offsets - (myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]  );

    gyrX_output = gyrX_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    gyrY_output = gyrY_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    gyrZ_output = gyrZ_raw_plus_offsets / GPF_IMU_GYRO_SCALE_FACTOR; //deg/sec
    
    //LP filter gyro data
    gyrX_output = (1.0 - B_gyro)*gyrX_output_prev + B_gyro*gyrX_output;
    gyrY_output = (1.0 - B_gyro)*gyrY_output_prev + B_gyro*gyrY_output;
    gyrZ_output = (1.0 - B_gyro)*gyrZ_output_prev + B_gyro*gyrZ_output;
    gyrX_output_prev = gyrX_output;
    gyrY_output_prev = gyrY_output;
    gyrZ_output_prev = gyrZ_output;


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


void GPF_IMU::doFusion_madgwick6DOF() {
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

// Fonction qui fait PLUSIEUR série de lecture et garde le min/max de chaque série de lecture puis ensuite
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
    theImu.getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
    
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
