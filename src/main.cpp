/**
 * @file GPCD.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-01-06 
 * 
 * GPCD project - Stabilisateur de vol pour avion
 * Certaines parties de codes emprumtés de diverses sources dont:
 * https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *   
 */

#include "Arduino.h"
//#include "MPU6050.h"

#include <Wire.h>
#include "I2Cdev.h"
#include "gpf_mpu6050.h"
#include "gpf_debug.h"
#include "gpf_crsf.h"

GPF_MPU6050 myGPF_MPU6050;
GPF_CRSF    myGPF_CRSF;

bool          ledState = false;
unsigned long ledToggleLastTime = 0;
int16_t       ledToggleDuration = 250; //ms


void toggleLed() {
    if (millis() > (ledToggleLastTime + ledToggleDuration)) {
     ledState = !ledState;
     digitalWrite(LED_BUILTIN, ledState);
     ledToggleLastTime = millis();
    }    
}

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin as an output.

  Serial.begin(115200);  //Port USB
  while (!Serial) { }; //Attend que le port serie Serial (USB) soit prêt
  DEBUG_PRINTLN("Port serie Serial (USB) ouvert.");
  DEBUG_PRINTLN("v112");

  Wire.begin();           //La librairie I2Cdev en a besoin
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

  myGPF_CRSF.initialize(&Serial1); 
    
  myGPF_MPU6050.initialize();
  myGPF_MPU6050.setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_ACCEL_GYRO_COMPLEMENTARY_FILTER);
  myGPF_MPU6050.setComplementaryFilterGyroWeightPercent(80);

    //myGPF_MPU6050.setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_SUM_NO_FILTER);
    //myGPF_MPU6050.setOrientationCalcMode(GPF_IMU_ORIENTATION_CALC_MODE_GYRO_CURRENT_NO_FILTER);
    
    //myGPF_MPU6050.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_250); //marche
    //myGPF_MPU6050.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_500); //marche
    //myGPF_MPU6050.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_1000); //marche

    //myGPF_MPU6050.setFullScaleGyroRange(GPF_MPU6050_GYRO_FS_500);

    
    /*
    Serial.print("getFullScaleAccelRange A:");
    Serial.println(accelgyro.getFullScaleAccelRange()); // 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
    accelgyro.setFullScaleAccelRange(1);
    Serial.print("getFullScaleAccelRange B:");
    Serial.println(accelgyro.getFullScaleAccelRange());

    Serial.print("getFullScaleGyroRange:");
    Serial.println(accelgyro.getFullScaleGyroRange()); // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
    accelgyro.setFullScaleGyroRange(2); //Changer valeur de GPF_IMU_GYR_LSB_SENSITIVITY si changement ici
    Serial.print("getFullScaleGyroRange:");
    Serial.println(accelgyro.getFullScaleGyroRange()); 

    Serial.print("getDHPFMode():");
    Serial.print(accelgyro.getDHPFMode());
    Serial.println("");
    */

/*
    while (true) {

     for (int i = 0; i < 10; i++) {
      Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(89.6,0.5)),4); Serial.print(", "); //x
      Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(-0.6,0.5)),4); Serial.print(", "); //y
      Serial.println("");
      delay(200);    
     }

     for (int i = 0; i < 10; i++) {
      Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(89.6,-0.5)),4); Serial.print(", "); //x
      Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(-0.6,-0.5)),4); Serial.print(", "); //y
      Serial.println("");
      delay(200);
     }
        
    }
*/
 //myGPF_CRSF.duration_betwen_frame = 0;
}

bool timeToPrint_2 = false;
unsigned long millis_timer_timeToprint_2 = 0;
unsigned long loop_count_2 = 0;
elapsedMillis sincePrint;

void loop() {
    loop_count_2++;
    //Serial2.println("A");

    myGPF_CRSF.readRx();
    if (sincePrint > 1000) {
     sincePrint = 0;     
    }
    


    
    myGPF_MPU6050.readSensorsAndDoCalculations();
/*
    bool timeToPrint_2 = false;
    if (millis() >= millis_timer_timeToprint_2 + 1000) {
      timeToPrint_2 = true;
      millis_timer_timeToprint_2 = millis();
    }

    if (timeToPrint_2) {
     DEBUG_PRINTLN(loop_count_2);
    }
  */  

    
    //Serial.print(myGPF_MPU6050.complementary_filter_0_360_pitch);     Serial.print(", ");
    

    //Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.complementary_filter_0_360_pitch),4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.complementary_filter_0_360_roll),4);     Serial.print(", ");

    ////Serial.print(myGPF_MPU6050.output_pitch,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.output_roll,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.complementary_filter_0_360_pitch,4);     Serial.print(", ");
    

    //Serial.print(myGPF_MPU6050.acc_pitch_degree_0_360,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.acc_pitch_degree_0_360_z_normal,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.acc_pitch_degree_0_360_z_inversed,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.output_roll,4);     Serial.print(", ");
    
    ////Serial.print(myGPF_MPU6050.gyr_pitch_degree,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.acc_pitch_degree,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.acc_z_degree,4);     Serial.print(", ");

    ////Serial.print(myGPF_MPU6050.complementaryFilterGyroWeight,4);     Serial.print(", ");
    ////Serial.print(myGPF_MPU6050.complementaryFilterAccWeight,4);     Serial.print(", ");


    //Serial.print(myGPF_MPU6050.acc_roll_degree,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.acc_z_degree,4);     Serial.print(", ");
    
    //Serial.print(myGPF_MPU6050.acc_z_degree,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.accX,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.accY,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.accZ,4);     Serial.print(", ");

    //Serial.print(myGPF_MPU6050.complementary_filter_0_360_roll,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.acc_roll_degree_0_360,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.acc_pitch_degree,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.acc_roll_degree,4);     Serial.print(", ");
    //Serial.print(myGPF_MPU6050.acc_z_degree,4);     Serial.print(", ");

    //Serial.print(myGPF_MPU6050.convert_90_90_to_0_360(myGPF_MPU6050.acc_pitch_degree,myGPF_MPU6050.acc_z_degree),4); Serial.print(", ");
    //Serial.print(myGPF_MPU6050.convert_90_90_to_0_360(myGPF_MPU6050.acc_roll_degree,myGPF_MPU6050.acc_z_degree),4); Serial.print(", ");
    //Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(myGPF_MPU6050.acc_pitch_degree,myGPF_MPU6050.acc_z_degree)),4); Serial.print(", ");
    //Serial.print(myGPF_MPU6050.convert_0_360_to_90_90(myGPF_MPU6050.convert_90_90_to_0_360(myGPF_MPU6050.acc_roll_degree,myGPF_MPU6050.acc_z_degree)),4); Serial.print(", ");

    //Serial.print(ecartPitch);     Serial.print(", ");
    //Serial.println("");
    
    
    
    toggleLed();
    //delay(100);
}





