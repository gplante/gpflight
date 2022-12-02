/**
 * @file main.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-01-06 
 * 
 * GPFlight - Stabilisateur de vol pour drone
 * Certaines parties de codes emprumtés de diverses sources dont en voici une liste non exhaustive:  
 * https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * https://github.com/ArduPilot/ardupilot
 * https://github.com/iNavFlight/inav
 * https://github.com/betaflight/betaflight
 * https://github.com/dncoder/crsf-link-tester
 * https://github.com/PX4/PX4-Autopilot
 *   
 */

#include "Arduino.h"
#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"

#include "gpf_cons.h"
#include "gpf_telemetry.h"
#include "gpf.h"
#include "gpf_mpu6050.h"
#include "gpf_debug.h"
#include "gpf_crsf.h"
#include "gpf_util.h"

#include "ILI9341_t3.h"
#include "gpf_display.h"
#include "XPT2046_Touchscreen.h"
#include "gpf_touch.h"

gpf_config_struct myConfig;

GPF          myFc; //My Flight Controller GPFlight

elapsedMillis sinceDummy;   

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin as an output. // Attention, la LED_BUILTIN (pin 13) est la même pin que le SPI SCK sur un Teensy 4.1
  gpf_util_blinkMainBoardLed(3);
  Serial.begin(115200);  //Port USB //C'est long genre 2 secondes ???
  sinceDummy = 0;
  while ((!Serial) && (sinceDummy < 1000)) { }; //Attend maximum 1 seconde que le port serie Serial (USB) soit prêt sinon si port usb non branché, la programme jam ici.
  DEBUG_GPF_PRINTLN(F("Port serie Serial (USB) ouvert."));
  DEBUG_GPF_PRINT(F("prog version:"));  
  DEBUG_GPF_PRINTLN(GPF_MISC_PROG_CURRENT_VERSION);  
  
  gpf_util_blinkMainBoardLed(2);

  //xxxxxxxxxxxxxxxxxxxxxx
  EEPROM.get(0, myConfig); 
  DEBUG_GPF_PRINT(F("eeprom config version:"));
  DEBUG_GPF_PRINTLN(myConfig.version);
  DEBUG_GPF_PRINT(F("current config version:"));
  DEBUG_GPF_PRINTLN(GPF_MISC_CONFIG_CURRENT_VERSION);

  if (myConfig.version != GPF_MISC_CONFIG_CURRENT_VERSION) { //Ca veut dire qu'il n'y a rien d'enregistré dans le eeprom donc on met des valeurs par défaut.
   gpf_util_resetConfigToDefault(&myConfig);
   
  }
  //xxxxxxxxxxxxxx

  Wire.begin();           //La librairie I2Cdev en a besoin
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

  myFc.initialize(&myConfig);
  myFc.genDummyTelemetryData();
  
  //myFc.myImu.calibrate();
  
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
      Serial.print(myFc.myImu.convert_0_360_to_90_90(myFc.myImu.convert_90_90_to_0_360(89.6,0.5)),4); Serial.print(", "); //x
      Serial.print(myFc.myImu.convert_0_360_to_90_90(myFc.myImu.convert_90_90_to_0_360(-0.6,0.5)),4); Serial.print(", "); //y
      Serial.println("");
      delay(200);    
     }

     for (int i = 0; i < 10; i++) {
      Serial.print(myFc.myImu.convert_0_360_to_90_90(myFc.myImu.convert_90_90_to_0_360(89.6,-0.5)),4); Serial.print(", "); //x
      Serial.print(myFc.myImu.convert_0_360_to_90_90(myFc.myImu.convert_90_90_to_0_360(-0.6,-0.5)),4); Serial.print(", "); //y
      Serial.println("");
      delay(200);
     }
        
    }
*/  
}


void loop() {
    static bool isArmed_previous = true;

    myFc.iAmStartingLoopNow(true);
    myFc.debugDisplayLoopStats();
    myFc.myRc.readRx(); //Armé ou non, on va toujours lire la position des sticks
    myFc.myImu.readSensorsAndDoCalculations(); //Armé ou non, on va toujours lire le IMU

    myFc.set_arm_IsArmed(false);

    if (myFc.get_arm_IsArmed()) {
      // Si armé, on ne va pas dans le menu et on n'affiche rien sur le display non plus. (C'est beaucoup trop long afficher de toute facon)
      // On se contente de ne pas ralentir la loop pour faire voler le drone :-)
      
      if (!isArmed_previous) { 
        //On était pas armé le tour d'avant donc on pourrait faire un traitement spécial vue qu'on vient tout juste d'armer        
        myFc.displayArmed();
        myFc.resetLoopStats();

        isArmed_previous = true;
      }
      
      

    } else {
      //Si pas armé, et bien on affiche le menu et on peut faire la gestion des paramètres
      
      //if ((isArmed_previous) || (myFc.menu_current != myFc.menu_previous)) {
      if (isArmed_previous) {
        //On était armé le tour d'avant ou qu'on revient d'un autre menu donc on pourrait faire un traitement spécial vu qu'on vient tout juste de désarmer
        isArmed_previous = false;
        myFc.resetLoopStats();
      }

      myFc.displayAndProcessMenu();
    }

    //myFc.toggMainBoardLed();
    
}





