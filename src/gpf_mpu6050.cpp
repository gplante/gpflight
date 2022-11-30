/**
 * @file gpf_mpu6050.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-01-06
 * 
 *  Code inspiré en grande partie par
 *  https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * 
 */
 
#include "Arduino.h"
#include <math.h>
#include "I2Cdev.h"
#include "gpf_mpu6050.h"
#include "gpf_debug.h"

GPF_MPU6050::GPF_MPU6050() {
    devAddr =  GPF_MPU6050_DEFAULT_ADDRESS;
    millis_old = millis();
}

void GPF_MPU6050::initialize(gpf_config_struct *ptr) {
    myConfig_ptr = ptr;
    setClockSource(GPF_MPU6050_CLOCK_PLL_XGYRO);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

    setComplementaryFilterGyroWeightPercent(99.9);
    setFullScaleAccelRange(GPF_MPU6050_ACCEL_FS_4); //Important, mettre cette ligne avant setFullScaleGyroRange() sinon le calcul gyro ne semble pas bien se faire ???
    setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_1000);

    setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER);
    //setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER);
    //setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER);


       // *** TESTS *****
       //myFc.myImu.setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER);
       //myFc.myImu.setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER);
    
       //myFc.myImu.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_250); //marche
       //myFc.myImu.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_500); //marche
       //myFc.myImu.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_1000); //marche

       //myFc.myImu.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_500);


    millis_old = millis();

    #ifdef DEBUG_GPF_MPU6050_ENABLED
     //if (debug_sincePrint > DEBUG_GPF_MPU6050_DELAY) {
      //DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      //DEBUG_GPF_MPU6050_PRINT(F("getDeviceID(): hex:"));
      //DEBUG_GPF_MPU6050_PRINT(getDeviceID(), HEX);      
      //DEBUG_GPF_MPU6050_PRINTLN();
      //debug_sincePrint = 0;
     //}
    #endif
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 */
void GPF_MPU6050::setClockSource(uint8_t source) {
    I2Cdev::writeBits(devAddr, GPF_MPU6050_RA_PWR_MGMT_1, GPF_MPU6050_PWR1_CLKSEL_BIT, GPF_MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status 
 */
void GPF_MPU6050::setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, GPF_MPU6050_RA_PWR_MGMT_1, GPF_MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting 
 */
void GPF_MPU6050::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, GPF_MPU6050_RA_ACCEL_CONFIG, GPF_MPU6050_ACONFIG_AFS_SEL_BIT, GPF_MPU6050_ACONFIG_AFS_SEL_LENGTH, range);

    switch (range) {
     case GPF_MPU6050_ACCEL_FS_2:
      gyr_lsb_sensitivity = GPF_MPU6050_ACCEL_FS_2_SENSITIVITY;
      break;

     case GPF_MPU6050_ACCEL_FS_4: 
      gyr_lsb_sensitivity = GPF_MPU6050_ACCEL_FS_4_SENSITIVITY;
      break;

     case GPF_MPU6050_ACCEL_FS_8: 
      gyr_lsb_sensitivity = GPF_MPU6050_ACCEL_FS_8_SENSITIVITY;
      break;

     case GPF_MPU6050_ACCEL_FS_16: 
      gyr_lsb_sensitivity = GPF_MPU6050_ACCEL_FS_16_SENSITIVITY;
      break;
    
     default:
      break;
    }
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 */
void GPF_MPU6050::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, GPF_MPU6050_RA_GYRO_CONFIG, GPF_MPU6050_GCONFIG_FS_SEL_BIT, GPF_MPU6050_GCONFIG_FS_SEL_LENGTH, range);

    switch (range) {
     case GPF_MPU6050_GYRO_FS_250:
      gyr_lsb_sensitivity = GPF_MPU6050_GYRO_FS_250_SENSITIVITY;
      break;

     case GPF_MPU6050_GYRO_FS_500: 
      gyr_lsb_sensitivity = GPF_MPU6050_GYRO_FS_500_SENSITIVITY;
      break;

     case GPF_MPU6050_GYRO_FS_1000: 
      gyr_lsb_sensitivity = GPF_MPU6050_GYRO_FS_1000_SENSITIVITY;
      break;

     case GPF_MPU6050_GYRO_FS_2000: 
      gyr_lsb_sensitivity = GPF_MPU6050_GYRO_FS_2000_SENSITIVITY;
      break;
    
     default:
      break;
    }
    
}

/** Set complementaryFilterGyroWeightPercent
 
 */
void GPF_MPU6050::setComplementaryFilterGyroWeightPercent(float weightPercent) {
 complementaryFilterGyroWeightPercent = max(0,min(weightPercent,100));
 complementaryFilterGyroWeight = complementaryFilterGyroWeightPercent / 100;
 complementaryFilterAccWeight  = (100 - complementaryFilterGyroWeightPercent) / 100;
}

/** Change le mode de calcul pour déterminer l'orientation (pitch et roll)
 * @param calcMode Le mode désiré
 * 
 * GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_NO_FILTER: 
 *  (Ce mode est pour fins de test seulement)
 *  Se sert uniquement de l'accéléromètre pour déterminer l'orientation. 
 *  Problème principal: Sensible aux vibrations. 
 *  Si on bouge le IMU tranquillement, on peut voir l'inclinaison de pitch et du roll de -180 à 180 degrés par exemple 
 *  mais si on shake l'IMU plus fort et bien on se ramasse avec des résultats beaucoup plus grands que 180 degrés 
 *  et c’est normal car l'accéléromètre enregistre des "poussées" assez forte et en plus je ne limite pas la sortie 
 *  à -180 ou 180 degrés dans ce mode.
 * 
 * GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_LOW_PASS_FILTER:
 *  (Ce mode est pour fins de test seulement)
 *  Ce mode est le même que si haut mais on ajoute un low pass filter donc ca élimine beaucoup de vibrations mais 
 *  au détriment de la vitesse de réaction. Bref, En ajoutant un low pass filter, on réussit à filtrer les "poussés subites" 
 *  mais au prix que ca réagit moins vite.
 * 
 * GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER
 *  (Ce mode est pour fins de test seulement)
 *  Ce mode ne se sert plus de l'accéléromètre pour déterminer l'orientation mais plutôt du gyroscope.
 *  Donc En utilisant le gyro à la place de l'accéléromètre, on peut obtenir l'angle d'inclinaison sans 
 *  noise/vibrations contrairement à l'accéléromètre ci-haut. Cependant, le problème avec le gyro est que 
 *  ca drift dans le temps et ca drift beaucoup même. L'accéléromètre lui ne drift pas mais sans filtre on se ramasse 
 *  avec du noise (vibrations). Donc le meilleur des deux mondes est le complementary filter ci-dessous.
 * 
 * GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER
 * (Ce mode est pour fins de test seulement)
 * Ce mode donne l'orientation à l'aide du gyro mais seulement lorsque l'IMU est en mouvement.
 * Bref, ca donne le nombre de degrés que l'imu a tourné pendant l'instant présent.
 * 
 * GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER 
 * Ce mode est le meilleur des deux mondes car il utilise l'accéléromètre et le gyroscope pour déterminer l'orientation.
 * Ca s'appel un filtre complémentaire. On se sert de la précision à court terme du gyro (car à long terme ca drift) et de la 
 * stabilité à long terme de l'accéléromètre (car à court terme ce produit des vibrations que nous ne désirons pas avoir).
 * Bref, on applique un filtre passe-haut au gyroscope et un filtre passe-bas à l'accéléromètre. 
 * 
 */
void GPF_MPU6050::setOrientationCalcMode(uint8_t calcMode) {
 orientationCalcMode = calcMode;
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34) 
 */
uint8_t GPF_MPU6050::getDeviceID() {
    I2Cdev::readBits(devAddr, GPF_MPU6050_RA_WHO_AM_I, GPF_MPU6050_WHO_AM_I_BIT, GPF_MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value 
 */
bool GPF_MPU6050::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    if (I2Cdev::readBytes(devAddr, GPF_MPU6050_RA_ACCEL_XOUT_H, 14, buffer) == -1) {
      return false;
    }

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];

    return true;
}

/** Get raw 6-axis motion sensor readings (accel/gyro) and do some math.
 * Retrieves all currently available motion sensor values and do some math. 
 */
void GPF_MPU6050::readSensorsAndDoCalculations() {
    unsigned long accX2_with_offsets, accY2_with_offsets, accZ2_with_offsets;
    static unsigned long errCpt_type_1 = 0, errCpt_type_2 = 0;

    float pitch_delta   = 0;
    float pitch_delta_1 = 0;
    float pitch_delta_2 = 0;
    float acc_pitch_0_360_previous = 0;
    float acc_pitch_0_360_current  = 0;

    float roll_delta    = 0;
    float roll_delta_1  = 0;
    float roll_delta_2  = 0;
    float acc_roll_0_360_previous  = 0;
    float acc_roll_0_360_current   = 0;

    bool b = getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);

    if (!b) {
      errCpt_type_1++;
      DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      DEBUG_GPF_MPU6050_PRINTLN(F("***** getMotion6() a retourne false"));
      return;
    }

    // **************************************************************************************
    // *** Condition jamais testée, plus capable de reproduire le problème
    // On dirait que des fois, la fonction getMotion6() retournait des valeurs à 0 puis ensuite les calculs donnaient comme résultat Nan.
    // Puisque dans la boucle, on se sert toujours des valauers des calculs précédents pour refaire les nouveaux calculs et bien les nouveaux calculs restaient toujours à Nan.
    // Donc au lieu d'obtenir des calculs à Nan on ne fait pas les calculs puis on sort immédiatement de la fonction.
    if (isnan(atan(accX_raw_no_offsets/sqrt((accY_raw_no_offsets*accY_raw_no_offsets)+(accZ_raw_no_offsets*accZ_raw_no_offsets))))) {
     errCpt_type_2++;
     DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
     DEBUG_GPF_MPU6050_PRINTLN(F("***** NAN return"));     
     return;
    }

    if ( (accX_raw_no_offsets == 0) && (accY_raw_no_offsets == 0) && (accZ_raw_no_offsets == 0) && (gyrX_raw_no_offsets == 0) && (gyrY_raw_no_offsets == 0) && (gyrZ_raw_no_offsets == 0)) {
      DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      DEBUG_GPF_MPU6050_PRINTLN(F("***** Tout a zero return"));     
      return;
    }    
    //*** Fin de Condition jamais testée, plus capable de reproduire le problème
    // **************************************************************************************

    //Ajoute les offsets

    accX_with_offsets = accX_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X];
    accY_with_offsets = accY_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y];
    accZ_with_offsets = accZ_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z];

    gyrX_with_offsets = gyrX_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X];
    gyrY_with_offsets = gyrY_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y];
    gyrZ_with_offsets = gyrZ_raw_no_offsets - myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z];

/*
    accX_with_offsets = accX_raw_no_offsets;
    accY_with_offsets = accY_raw_no_offsets;
    accZ_with_offsets = accZ_raw_no_offsets;

    gyrX_with_offsets = gyrX_raw_no_offsets;
    gyrY_with_offsets = gyrY_raw_no_offsets;
    gyrZ_with_offsets = gyrZ_raw_no_offsets;
*/

    accX2_with_offsets = (unsigned long)(accX_with_offsets*accX_with_offsets);
    accY2_with_offsets = (unsigned long)(accY_with_offsets*accY_with_offsets);
    accZ2_with_offsets = (unsigned long)(accZ_with_offsets*accZ_with_offsets);

    // Z Axis (-90 degrés à 90 degrés)
    acc_z_radiant = atan(accZ_with_offsets/sqrt(accX2_with_offsets+accY2_with_offsets)); 
    acc_z_degree  = acc_z_radiant/2/PI*360;

    // X Axis // Z Axis (-90 degrés à 90 degrés)
    acc_pitch_radiant = atan(accX_with_offsets/sqrt(accY2_with_offsets+accZ2_with_offsets));
    acc_pitch_degree  = acc_pitch_radiant/2/PI*360;

    // Y Axis // Z Axis (-90 degrés à 90 degrés)
    acc_roll_radiant = atan(accY_with_offsets/sqrt(accX2_with_offsets+accZ2_with_offsets));
    acc_roll_degree  = acc_roll_radiant/2/PI*360;

    if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_NO_FILTER) {
     output_pitch = acc_pitch_degree;
     output_roll  = acc_roll_degree;
    }

    if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_LOW_PASS_FILTER) {
      output_pitch = GPF_IMU_LOW_PASS_FILTER_WEIGHT_OLD*acc_pitch_degree_old + GPF_IMU_LOW_PASS_FILTER_WEIGHT_CURRENT*acc_pitch_degree;
      output_roll  = GPF_IMU_LOW_PASS_FILTER_WEIGHT_OLD*acc_roll_degree_old  + GPF_IMU_LOW_PASS_FILTER_WEIGHT_CURRENT*acc_roll_degree;
 
      acc_pitch_degree_old = output_pitch;
      acc_roll_degree_old  = output_roll;
    }

    if (
        (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER) || 
        (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER) || 
        (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER) 
       ) {
     time_elapsed = (millis()-millis_old)/1000.;
     millis_old = millis();

     gyr_pitch_degree = -(gyrY_with_offsets / gyr_lsb_sensitivity * time_elapsed);
     gyr_roll_degree  =   gyrX_with_offsets / gyr_lsb_sensitivity * time_elapsed;
    }
    
    if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER) {
     output_pitch = gyr_pitch_degree;
     output_roll  = gyr_roll_degree;      
    }

    if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER) {
     gyr_pitch_degree_sum = gyr_pitch_degree_sum + gyr_pitch_degree;
     gyr_roll_degree_sum  = gyr_roll_degree_sum + gyr_roll_degree;

     output_pitch = gyr_pitch_degree_sum;
     output_roll  = gyr_roll_degree_sum;      
    }

    if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER) {
      // pitch:
      // Je détermine la différence en degrés entre l'attitude d'avant et l'attitude actuelle.
      // Pour déterminer la différence, j'imagine l'attitude comme un cadran gradué en 360 degrés que je tournerait pour atteindre la position actuelle 
      // soit dans un sens (pitch_delta_1) ou soit dans l'autre sens (pitch_delta_2) puis je 
      // garde le plus petit des deux dans la variable pitch_delta.
      // C'est exactement le même principle pour le roll un peu plus bas ci-après.
      acc_pitch_0_360_previous = complementary_filter_0_360_pitch;
      acc_pitch_0_360_current  = convert_90_90_to_0_360(acc_pitch_degree,acc_z_degree);
      pitch_delta_1          = acc_pitch_0_360_current - acc_pitch_0_360_previous;
      pitch_delta_2          = 360 - abs(pitch_delta_1);

      if (abs(pitch_delta_1) <= abs(pitch_delta_2)) {
       pitch_delta = pitch_delta_1;
      } else {
        if (pitch_delta_1 <= 0) {
          pitch_delta = pitch_delta_2;
        } else {
          pitch_delta = -pitch_delta_2;
        }
      }
      
      complementary_filter_0_360_pitch = (complementaryFilterGyroWeight*(complementary_filter_0_360_pitch + gyr_pitch_degree)) + (complementaryFilterAccWeight*(complementary_filter_0_360_pitch + pitch_delta));
      complementary_filter_0_360_pitch = fmod((360  + complementary_filter_0_360_pitch),360);
      output_pitch = convert_0_360_to_90_90(complementary_filter_0_360_pitch);

      // roll:
      // Voir commentaires de pitch ci-haut.
      acc_roll_0_360_previous = complementary_filter_0_360_roll;
      acc_roll_0_360_current  = convert_90_90_to_0_360(acc_roll_degree,acc_z_degree);
      roll_delta_1          = acc_roll_0_360_current - acc_roll_0_360_previous;
      roll_delta_2          = 360 - abs(roll_delta_1);

      if (abs(roll_delta_1) <= abs(roll_delta_2)) {
       roll_delta = roll_delta_1;
      } else {
        if (roll_delta_1 <= 0) {
          roll_delta = roll_delta_2;
        } else {
          roll_delta = -roll_delta_2;
        }
      }
      
      complementary_filter_0_360_roll = (complementaryFilterGyroWeight*(complementary_filter_0_360_roll + gyr_roll_degree)) + (complementaryFilterAccWeight*(complementary_filter_0_360_roll + roll_delta));
      complementary_filter_0_360_roll = fmod((360  + complementary_filter_0_360_roll),360);
      output_roll = convert_0_360_to_90_90(complementary_filter_0_360_roll);
      
    }


    #ifdef DEBUG_GPF_MPU6050_ENABLED
     if (debug_sincePrint > DEBUG_GPF_MPU6050_DELAY) {

      if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER) {
       DEBUG_GPF_MPU6050_PRINT(gyr_pitch_degree);                        DEBUG_GPF_MPU6050_PRINT(F(", "));
       DEBUG_GPF_MPU6050_PRINT(gyr_roll_degree);                        DEBUG_GPF_MPU6050_PRINT(F(", "));
       DEBUG_GPF_MPU6050_PRINTLN();
      }

      if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER) {
       DEBUG_GPF_MPU6050_PRINT(gyr_pitch_degree_sum);                        DEBUG_GPF_MPU6050_PRINT(F(", "));
       DEBUG_GPF_MPU6050_PRINT(gyr_roll_degree_sum);                        DEBUG_GPF_MPU6050_PRINT(F(", "));
       DEBUG_GPF_MPU6050_PRINTLN();
      }

      if (orientationCalcMode == GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER) {
       DEBUG_GPF_MPU6050_PRINT(output_pitch);                       DEBUG_GPF_MPU6050_PRINT(F(", "));       
       DEBUG_GPF_MPU6050_PRINT(output_roll);                       DEBUG_GPF_MPU6050_PRINT(F(", "));       
       DEBUG_GPF_MPU6050_PRINTLN();
      }

       debug_sincePrint = 0;
     }
    #endif

}

float GPF_MPU6050::convert_90_90_to_0_360(float x_or_y_Degree, float z_Degree) {

 /* Example
 
 z_Degree   x_or_y_Degree   return
    +90          0            0
    +45         45           45
     +1         89           89
     -0         90           90
    -45         45          135
    -89          1          179
    -90          0          180
    -89         -1          181
    -45        -45          225
     -1        -89          269
     +0        -90          270
    +45        -45          315
    +89         -1          359    
 */

 float newDegree = 0;   

 if (x_or_y_Degree < 0) {
  if (z_Degree < 0) {
   newDegree = 180 + abs(x_or_y_Degree);
  } else {
   newDegree = 270 + (90 - abs(x_or_y_Degree));
  }    
 } else {
  if (z_Degree < 0) {
   newDegree =  90 + (90 - abs(x_or_y_Degree));     
  } else {
   newDegree = x_or_y_Degree;   
  }
 }

 return newDegree;
}

float GPF_MPU6050::convert_0_360_to_90_90(float degree) {

 /* Example

 x_or_y_Degree   return
     0            0
    45           45
    90           90
   135           45
   180            0
   225          -45
   270          -90
   315          -45
   359           -1

 */   

 float newDegree = 0;   

 newDegree = degree;

 if (degree >= 270) {
  newDegree = (90 - fmod(degree,90)) * -1;
 } else {
   if (degree >= 180) {
     newDegree = fmod(degree,90) * -1;
   } else {
     if (degree >= 90) {
       newDegree = (90 - fmod(degree,90));
     }
   }
 }

 return newDegree;
}

/*
// XA_OFFS_* registers

int16_t GPF_MPU6050::getXAccelOffset() {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_XA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250  
  uint8_t SaveAddress = 0x77; //Patch - non documenté ???
	I2Cdev::readBytes(devAddr, SaveAddress, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setXAccelOffset(int16_t offset) {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_XA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
  uint8_t SaveAddress = 0x77; //Patch - non documenté ???
	I2Cdev::writeWord(devAddr, SaveAddress, offset);
}

// YA_OFFS_* register

int16_t GPF_MPU6050::getYAccelOffset() {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_YA_OFFS_H:0x7A); // MPU6050,MPU9150 Vs MPU6500,MPU9250
  uint8_t SaveAddress = 0x7A; //Patch - non documenté ???
	I2Cdev::readBytes(devAddr, SaveAddress, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setYAccelOffset(int16_t offset) {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_YA_OFFS_H:0x7A); // MPU6050,MPU9150 Vs MPU6500,MPU9250
  uint8_t SaveAddress = 0x7A; //Patch - non documenté ???
	I2Cdev::writeWord(devAddr, SaveAddress, offset);
}

// ZA_OFFS_* register

int16_t GPF_MPU6050::getZAccelOffset() {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_ZA_OFFS_H:0x7D); // MPU6050,MPU9150 Vs MPU6500,MPU9250
  uint8_t SaveAddress = 0x7D; //Patch - non documenté ???
	I2Cdev::readBytes(devAddr, SaveAddress, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setZAccelOffset(int16_t offset) {
	//uint8_t SaveAddress = ((getDeviceID() < 0x38 )? GPF_MPU6050_RA_ZA_OFFS_H:0x7D); // MPU6050,MPU9150 Vs MPU6500,MPU9250
  uint8_t SaveAddress = 0x7D; //Patch - non documenté ???
	I2Cdev::writeWord(devAddr, SaveAddress, offset);
}

// XG_OFFS_USR* registers

int16_t GPF_MPU6050::getXGyroOffset() {
    I2Cdev::readBytes(devAddr, GPF_MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setXGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, GPF_MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t GPF_MPU6050::getYGyroOffset() {
    I2Cdev::readBytes(devAddr, GPF_MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setYGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, GPF_MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t GPF_MPU6050::getZGyroOffset() {
    I2Cdev::readBytes(devAddr, GPF_MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void GPF_MPU6050::setZGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, GPF_MPU6050_RA_ZG_OFFS_USRH, offset);
}

*/

// Fonction qui fait PLUSIEUR série de lecture et garde le min/max de chaque série de lecture puis ensuite
// fait une moyenne qui sera l'offset final.
void GPF_MPU6050::calibrate() {

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

  DEBUG_GPF_MPU6050_PRINTLN("Calibrating...");
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
    
    DEBUG_GPF_MPU6050_PRINT(" moy-min-max ax=");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax_max);
    
    DEBUG_GPF_MPU6050_PRINT(" ay=");    
    DEBUG_GPF_MPU6050_PRINT((calibration_offset_ay));
    DEBUG_GPF_MPU6050_PRINT("/");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_ay_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_ay_max);
    
    DEBUG_GPF_MPU6050_PRINT(" az=");    
    DEBUG_GPF_MPU6050_PRINT((calibration_offset_az));
    DEBUG_GPF_MPU6050_PRINT("/");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_az_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_az_max);

    DEBUG_GPF_MPU6050_PRINT(" moy-min-max gx=");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx_max);
    
    DEBUG_GPF_MPU6050_PRINT(" gy=");    
    DEBUG_GPF_MPU6050_PRINT((calibration_offset_gy));
    DEBUG_GPF_MPU6050_PRINT("/");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gy_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gy_max);
    
    DEBUG_GPF_MPU6050_PRINT(" gz=");    
    DEBUG_GPF_MPU6050_PRINT((calibration_offset_gz));
    DEBUG_GPF_MPU6050_PRINT("/");    
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gz_min);
    DEBUG_GPF_MPU6050_PRINT("/");
    DEBUG_GPF_MPU6050_PRINT(calibration_offset_gz_max);

    DEBUG_GPF_MPU6050_PRINTLN();

  }
  

  //Moyenne du min/max qui sera l'offset final
  calibration_offset_ax = calibration_offset_ax_min + ((calibration_offset_ax_max - calibration_offset_ax_min)/2); 
  calibration_offset_ay = calibration_offset_ay_min + ((calibration_offset_ay_max - calibration_offset_ay_min)/2); 
  //calibration_offset_az = calibration_offset_az_min + ((calibration_offset_az_max - calibration_offset_az_min)/2); 
  calibration_offset_az = 16384 - (calibration_offset_az_min + ((calibration_offset_az_max - calibration_offset_az_min)/2)); 
  //az_offset=(16384-mean_az)/8

  calibration_offset_gx = calibration_offset_gx_min + ((calibration_offset_gx_max - calibration_offset_gx_min)/2); 
  calibration_offset_gy = calibration_offset_gy_min + ((calibration_offset_gy_max - calibration_offset_gy_min)/2); 
  calibration_offset_gz = calibration_offset_gz_min + ((calibration_offset_gz_max - calibration_offset_gz_min)/2); 

  DEBUG_GPF_MPU6050_PRINTLN();  
  DEBUG_GPF_MPU6050_PRINTLN("***FIN***");  
  DEBUG_GPF_MPU6050_PRINTLN("ax min-moy-max\tay\t\taz\t\tgx\t\tgy\tgz\t\t");

  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ax_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");

  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ay_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ay); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_ay_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");

  DEBUG_GPF_MPU6050_PRINT(calibration_offset_az_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_az); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_az_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");


  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gx_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");

  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gy_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gy); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gy_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");

  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gz_min); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gz); 
  DEBUG_GPF_MPU6050_PRINT(" ");
  DEBUG_GPF_MPU6050_PRINT(calibration_offset_gz_max); 
  DEBUG_GPF_MPU6050_PRINT("\t");

  DEBUG_GPF_MPU6050_PRINTLN();

}

//Fonction qui fait une série de lecture puis fait une moyenne qui sera l'offset
void GPF_MPU6050::meansensors() {    
   long i = 0;
   long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
   int16_t calibration_buffersize = 250;  

   while (i<(calibration_buffersize+101)) {
    // read raw accel/gyro measurements from device
    getMotion6(&accX_raw_no_offsets, &accY_raw_no_offsets, &accZ_raw_no_offsets, &gyrX_raw_no_offsets, &gyrY_raw_no_offsets, &gyrZ_raw_no_offsets);
    
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