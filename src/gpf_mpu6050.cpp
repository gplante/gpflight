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

void GPF_MPU6050::initialize() {
    
    setClockSource(GPF_MPU6050_CLOCK_PLL_XGYRO);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

    setComplementaryFilterGyroWeightPercent(80);
    setFullScaleAccelRange(GPF_MPU6050_ACCEL_FS_4); //Important, mettre cette ligne avant setFullScaleGyroRange() sinon le calcul gyro ne semble pas bien se faire ???
    setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_1000);

    setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER);
    millis_old = millis();

    #ifdef DEBUG_GPF_MPU6050_ENABLED
     //if (debug_sincePrint > DEBUG_GPF_MPU6050_DELAY) {
      DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      DEBUG_GPF_MPU6050_PRINT(F("getDeviceID(): hex:"));
      DEBUG_GPF_MPU6050_PRINT(getDeviceID(), HEX);      
      DEBUG_GPF_MPU6050_PRINTLN();
      debug_sincePrint = 0;
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
   errCpt3++;
    bool timeToPrint = false;
    if (millis() >= millis_timer_timeToprint + 500) {
      timeToPrint = true;
      millis_timer_timeToprint = millis();
    }

    bool b = getMotion6(&accX, &accY, &accZ, &gyrX, &gyrY, &gyrZ);
    if (!b) {
      errCpt1++;
      DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      DEBUG_GPF_MPU6050_PRINTLN(F("***** getMotion6() a retourne false"));
      return;
    }

    // *** Condition jamais testée, plus capable de reproduire le problème
    // On dirait que des fois, la fonction getMotion6() retournait des valeurs à 0 puis ensuite les calculs donnaient comme résultat Nan.
    // Puisque dans la boucle, on se sert toujours des valauers des calculs précédents pour refaire les nouveaux calculs et bien les nouveaux calculs restaient toujours à Nan.
    // Donc au lieu d'obtenir des calculs à Nan on ne fait pas les calculs puis on sort immédiatement de la fonction.
    if (isnan(atan(accX/sqrt(accY2+accZ2)))) {
     errCpt2++;
     DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
     DEBUG_GPF_MPU6050_PRINTLN(F("***** NAN return"));     
     return;
    }

    if ( (accX == 0) && (accY == 0) && (accZ == 0) && (gyrX == 0) && (gyrY == 0) && (gyrZ == 0)) {
      DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
      DEBUG_GPF_MPU6050_PRINTLN(F("***** Tout a zero return"));     
      return;
    }    
    //*** Fin de Condition jamais testée, plus capable de reproduire le problème

    if (timeToPrint) {
      DEBUG_GPF_MPU6050_PRINTLN(F("MPU6050:"));
     DEBUG_GPF_MPU6050_PRINT(accX);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(accY);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(accZ);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(gyrX);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(gyrY);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(gyrZ);    DEBUG_GPF_MPU6050_PRINT(F(", "));
    }

    accX2 = (unsigned long)(accX*accX);
    accY2 = (unsigned long)(accY*accY);
    accZ2 = (unsigned long)(accZ*accZ);

    // Z Axis (-90 degrés à 90 degrés)
    acc_z_radiant = atan(accZ/sqrt(accX2+accY2)); 
    acc_z_degree  = acc_z_radiant/2/PI*360;

    // X Axis // Z Axis (-90 degrés à 90 degrés)
    acc_pitch_radiant = atan(accX/sqrt(accY2+accZ2));
    acc_pitch_degree  = acc_pitch_radiant/2/PI*360;

    

    // Y Axis // Z Axis (-90 degrés à 90 degrés)
    acc_roll_radiant = atan(accY/sqrt(accX2+accZ2));
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

     gyr_pitch_degree = -(gyrY / gyr_lsb_sensitivity * time_elapsed);
     gyr_roll_degree  =   gyrX / gyr_lsb_sensitivity * time_elapsed;

     //gyr_pitch_degree = 0; //test
     //gyr_roll_degree  = 0; //test
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
      //pitch

      // On check le pitch normal et aussi le pitch avec l'axe des Z inversé. C'est parceque si le pitch monte par exemple près de 90 degrés,
      // et bien l'axe des Z aussi change en même temps et se ramasse près de 0 degrés. Rendu près de 0 degrés un changement minime de quelques degrés 
      // par exemple pourrait faire passer l'axe des Z de -1 degré à 1 degré. Tout ca est bien normal et est logique. 
      // Cependant, le problème c'est que ma routine convert_90_90_to_0_360() convertie un angle de -90 à 90 vers 0 à 180 et si l'axe des Z est négatif elle convertie plutôt de 190 à 360 degrés. 
      // Donc, si l'axe des Z change de polarité la routine va retourner 180 degrés de plus ou de moin. En temps normal tout fonctionne bien  mais si l'axe des Z est près de 0 et bien l'axe des Z peut facilement 
      // changer de polarité simplement en une rotation de quelques degrés.
      // Puisqe je me sert de cette routine pour calculer mon filtre complémentaire, si l'axe des Z change de polarité parce qu'il est trop près de 0 et bien ma routine convert_90_90_to_0_360() 
      // retourne 180 degrés de plus ou moins donc le programme pense que le pitch a changé de 180 degrés mais ce n'est pas vrai en réalité. 
      // Donc pour détecter ce problème ja fait le calcul avec mon Z normal et mon Z inversé puis je prend celui qui a changé le moin par rapport à la valeur précédente de mon filtre complémentaire.
      // Ouf. pas facile à expliquer ;-)

      acc_pitch_degree_0_360_z_normal   = convert_90_90_to_0_360(acc_pitch_degree,acc_z_degree);
      acc_pitch_degree_0_360_z_inversed = convert_90_90_to_0_360(acc_pitch_degree,-acc_z_degree);

      //acc_pitch_degree_0_360 = acc_pitch_degree_0_360_z_normal;
      //if (abs(acc_z_degree) < 45) {
      // if ( abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_inversed) < abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_normal) ) {
        //acc_pitch_degree_0_360 = acc_pitch_degree_0_360_z_inversed;
      // }
      //}

      //Calcul avec Z normal
      cadranPitchFilter          = abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_normal);
      cadranPitchAcc             = 360 - abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_normal);
      cadranPitchDelta_z_normal  = min(cadranPitchFilter, cadranPitchAcc);

      if (
          ((complementary_filter_0_360_pitch < acc_pitch_degree_0_360_z_normal) && (cadranPitchAcc < cadranPitchFilter)) ||
          ((complementary_filter_0_360_pitch > acc_pitch_degree_0_360_z_normal) && (cadranPitchAcc > cadranPitchFilter)) 
         ) {
         //Ecart négative  
         cadranPitchDelta_z_normal = -cadranPitchDelta_z_normal;
      }

      //Calcul avec Z inversé
      cadranPitchFilter           = abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_inversed);
      cadranPitchAcc              = 360 - abs(complementary_filter_0_360_pitch - acc_pitch_degree_0_360_z_inversed);
      cadranPitchDelta_z_inversed = min(cadranPitchFilter, cadranPitchAcc);

      if (
          ((complementary_filter_0_360_pitch < acc_pitch_degree_0_360_z_inversed) && (cadranPitchAcc < cadranPitchFilter)) ||
          ((complementary_filter_0_360_pitch > acc_pitch_degree_0_360_z_inversed) && (cadranPitchAcc > cadranPitchFilter)) 
         ) {
         //Ecart négative  
         cadranPitchDelta_z_inversed = -cadranPitchDelta_z_inversed;
      }

      if (abs(cadranPitchDelta_z_normal) < abs(cadranPitchDelta_z_inversed)) {
       cadranPitchDelta_used = cadranPitchDelta_z_normal;
      } else {
       cadranPitchDelta_used = cadranPitchDelta_z_inversed;
      }

      if (timeToPrint) {
        //
      }

      //complementary_filter_0_360_pitch = (GPF_IMU_COMPLEMENTARY_FILTER_WEIGHT_GYRO*(complementary_filter_0_360_pitch + gyr_pitch_degree)) + (GPF_IMU_COMPLEMENTARY_FILTER_WEIGHT_ACC*(acc_pitch_degree_0_360));

      //Bug ligne ci-dessous?
      complementary_filter_0_360_pitch = (complementaryFilterGyroWeight*(complementary_filter_0_360_pitch + gyr_pitch_degree)) + (complementaryFilterAccWeight*(complementary_filter_0_360_pitch + cadranPitchDelta_used));

      if (timeToPrint) {
       //DEBUG_GPF_MPU6050_PRINT(F("MPU6050:"));
       //DEBUG_GPF_MPU6050_PRINT(complementary_filter_0_360_pitch,4);     DEBUG_GPF_MPU6050_PRINT(F(", ")); 
      }

      if (complementary_filter_0_360_pitch < 0) {
       complementary_filter_0_360_pitch = 360 - fmod(abs(complementary_filter_0_360_pitch),360);
      } else {
       complementary_filter_0_360_pitch = fmod(abs(complementary_filter_0_360_pitch),360);
      }

      output_pitch = convert_0_360_to_90_90(complementary_filter_0_360_pitch);


      //roll

      // On check le roll normal et aussi le roll avec l'axe des Z inversé. C'est parceque si le roll monte par exemple près de 90 degrés,
      // et bien l'axe des Z aussi change en même temps et se ramasse près de 0 degrés. Rendu près de 0 degrés un changement minime de quelques degrés 
      // par exemple pourrait faire passer l'axe des Z de -1 degré à 1 degré. Tout ca est bien normal et est logique. 
      // Cependant, le problème c'est que ma routine convert_90_90_to_0_360() convertie un angle de -90 à 90 vers 0 à 180 et si l'axe des Z est négatif elle convertie plutôt de 190 à 360 degrés. 
      // Donc, si l'axe des Z change de polarité la routine va retourner 180 degrés de plus ou de moin. En temps normal tout fonctionne bien  mais si l'axe des Z est près de 0 et bien l'axe des Z peut facilement 
      // changer de polarité simplement en une rotation de quelques degrés.
      // Puisqe je me sert de cette routine pour calculer mon filtre complémentaire, si l'axe des Z change de polarité parce qu'il est trop près de 0 et bien ma routine convert_90_90_to_0_360() 
      // retourne 180 degrés de plus ou moins donc le programme pense que le roll a changé de 180 degrés mais ce n'est pas vrai en réalité. 
      // Donc pour détecter ce problème ja fait le calcul avec mon Z normal et mon Z inversé puis je prend celui qui a changé le moin par rapport à la valeur précédente de mon filtre complémentaire.
      // Ouf. pas facile à expliquer ;-)
      acc_roll_degree_0_360_z_normal   = convert_90_90_to_0_360(acc_roll_degree,acc_z_degree);
      acc_roll_degree_0_360_z_inversed = convert_90_90_to_0_360(acc_roll_degree,-acc_z_degree);

      //acc_roll_degree_0_360 = acc_roll_degree_0_360_z_normal;
      //if (abs(acc_z_degree) < 45) {
      // if ( abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_inversed) < abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_normal) ) {
        //acc_roll_degree_0_360 = acc_roll_degree_0_360_z_inversed;
      // }
      //}

      //Calcul avec Z normal
      cadranRollFilter         = abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_normal);
      cadranRollAcc            = 360 - abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_normal);
      cadranRollDelta_z_normal = min(cadranRollFilter, cadranRollAcc);

      if (
          ((complementary_filter_0_360_roll < acc_roll_degree_0_360_z_normal) && (cadranRollAcc < cadranRollFilter)) ||
          ((complementary_filter_0_360_roll > acc_roll_degree_0_360_z_normal) && (cadranRollAcc > cadranRollFilter)) 
         ) {
         //Ecart négative  
         cadranRollDelta_z_normal = -cadranRollDelta_z_normal;
      }

      //Calcul avec Z inversé

      //acc_roll_degree_0_360  = convert_90_90_to_0_360(acc_roll_degree,-acc_z_degree);

      cadranRollFilter            = abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_inversed);
      cadranRollAcc               = 360 - abs(complementary_filter_0_360_roll - acc_roll_degree_0_360_z_inversed);
      cadranRollDelta_z_inversed  = min(cadranRollFilter, cadranRollAcc);

      if (
          ((complementary_filter_0_360_roll < acc_roll_degree_0_360_z_inversed) && (cadranRollAcc < cadranRollFilter)) ||
          ((complementary_filter_0_360_roll > acc_roll_degree_0_360_z_inversed) && (cadranRollAcc > cadranRollFilter)) 
         ) {
         //Ecart négative  
         cadranRollDelta_z_inversed = -cadranRollDelta_z_inversed;
      }

      if (abs(cadranRollDelta_z_normal) < abs(cadranRollDelta_z_inversed)) {
       cadranRollDelta_used = cadranRollDelta_z_normal;
      } else {
       cadranRollDelta_used = cadranRollDelta_z_inversed;
      }

      //complementary_filter_0_360_roll = (GPF_IMU_COMPLEMENTARY_FILTER_WEIGHT_GYRO*(complementary_filter_0_360_roll + gyr_roll_degree)) + (GPF_IMU_COMPLEMENTARY_FILTER_WEIGHT_ACC*(acc_roll_degree_0_360));
      complementary_filter_0_360_roll = (complementaryFilterGyroWeight*(complementary_filter_0_360_roll + gyr_roll_degree)) + (complementaryFilterAccWeight*(complementary_filter_0_360_roll + cadranRollDelta_used));

      if (complementary_filter_0_360_roll < 0) {
       complementary_filter_0_360_roll = 360 - fmod(abs(complementary_filter_0_360_roll),360);
      } else {
       complementary_filter_0_360_roll = fmod(abs(complementary_filter_0_360_roll),360);
      }

      output_roll = convert_0_360_to_90_90(complementary_filter_0_360_roll);
    }

    if (timeToPrint) {
     DEBUG_GPF_MPU6050_PRINT(errCpt1);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(errCpt2);    DEBUG_GPF_MPU6050_PRINT(F(", "));
     DEBUG_GPF_MPU6050_PRINT(errCpt3);    DEBUG_GPF_MPU6050_PRINT(F(", "));
    }

    if (timeToPrint) {
      DEBUG_GPF_MPU6050_PRINTLN("eol");
    }

}

float GPF_MPU6050::convert_90_90_to_0_360(float x_or_y_Degree, float z_Degree) {

 /* Example
 
 z_Degree sign  x_or_y_Degree   return
    +                 0            0
    +                45           45
    +                89           89
    -                90           90
    -                45          135
   -                  1          179
    -                 0          180
   -                 -1          181
    -               -45          225
    -               -89          269 
    +               -90          270
    +               -45          315
    +                -1          359    
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

 degree   return
 0          0
 45        45
 90        90
 135       45
 180        0
 225      -45
 270      -90
 315      -45
 359       -1

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
