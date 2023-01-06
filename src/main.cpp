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
#include "gpf_imu.h"
#include "gpf_debug.h"
#include "gpf_crsf.h"
#include "gpf_util.h"

#include "ILI9341_t3.h"
#include "gpf_display.h"
#include "XPT2046_Touchscreen.h"
#include "gpf_touch.h"
#include "gpf_sdcard.h"
#include <TimeLib.h>

gpf_config_struct myConfig;

GPF          myFc; //My Flight Controller GPFlight

elapsedMillis sinceDummy;   


void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin as an output. // Attention, la LED_BUILTIN (pin 13) est la même pin que le SPI SCK sur un Teensy 4.1
  pinMode(GPF_MISC_PIN_BUZZER, OUTPUT); // buzzer/piezo

  gpf_util_blinkMainBoardLed(3);
  Serial.begin(115200);  //Port USB //C'est long genre 2 secondes ???
  sinceDummy = 0;
  while ((!Serial) && (sinceDummy < 1000)) { }; //Attend maximum 1 seconde que le port serie Serial (USB) soit prêt sinon si port usb non branché, la programme jam ici.
  DEBUG_GPF_PRINTLN(F("Port serie Serial (USB) ouvert."));
  DEBUG_GPF_PRINT(F("prog version:"));  
  DEBUG_GPF_PRINTLN(GPF_MISC_PROG_CURRENT_VERSION);    
  
  gpf_util_blinkMainBoardLed(2);

  //Config eeprom
  EEPROM.get(0, myConfig); 
  DEBUG_GPF_PRINT(F("eeprom config version:"));
  DEBUG_GPF_PRINTLN(myConfig.version);
  DEBUG_GPF_PRINT(F("current config version:"));
  DEBUG_GPF_PRINTLN(GPF_MISC_CONFIG_CURRENT_VERSION);

  if (myConfig.version != GPF_MISC_CONFIG_CURRENT_VERSION) { //Ca veut dire qu'il n'y a rien d'enregistré dans le eeprom donc on met des valeurs par défaut.
   gpf_util_resetConfigToDefault(&myConfig);

   //On averti l'utilisateur de cette situation
   gpf_util_beep(GPF_UTIL_BEEP_TONE_ERROR,GPF_UTIL_BEEP_DURATION_ULTRA_LONG);
   delay(GPF_UTIL_BEEP_DURATION_ULTRA_LONG);

   //Puis on entre directement enmode calibration
   //myFc.menu_current = GPF_MENU_CONFIG_CALIBRATION_MENU;
  }
  //Fin Config eeprom

  // Date/Heure - RTC
  // set the Time library to use Teensy 3.0's RTC to keep time    
  setSyncProvider(gpf_util_getTeensy3Time);

  if (timeStatus()!= timeSet) {
    DEBUG_GPF_PRINTLN("Oups, unable to sync with the RTC");    
  } else {
    DEBUG_GPF_PRINTLN("RTC has set the system time :-)");
  }
  // Fin Date/Heure

  Wire.begin();           //La librairie I2Cdev en a besoin
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
  //Wire.setTimeout(3000); //test //On dirait moins d'erreur sur le MPU6050 avec celà... ??? Non finalement je pense ... Mettre AD0 de MPU6050 au ground...
  
  myFc.initialize(&myConfig);
  myFc.genDummyTelemetryData();
  
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
   //myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
   DEBUG_GPF_PRINTLN("*******************************************");    
   DEBUG_GPF_PRINTLN(myFc.get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_LOGGING,true));    
   DEBUG_GPF_PRINTLN(myFc.get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_1,true));    
}

elapsedMillis sinceChange;
uint16_t      lastDshotCommandSent = 0;

void loop() {
    static bool isArmed_previous = true;

    myFc.iAmStartingLoopNow(true);
    myFc.debugDisplayLoopStats();
    myFc.myRc.readRx(); //Armé ou non, on va toujours lire la position des sticks
    //Todo - Ajuste les variables selon la position des switchs genre armé, mode de vol, etc.

    myFc.set_arm_IsArmed(myFc.get_IsStickInPositionEnabled(GPF_RC_STICK_ARM)); //Dans certains cas, on ne permet pas d'armer
    myFc.set_black_box_IsEnabled(myFc.get_IsStickInPositionEnabled(GPF_RC_STICK_BLACK_BOX));

    if (myFc.myImu.getIMUData()) { //Armé ou non, on va toujours lire le IMU
     if (GPF_IMU_FUSION_TYPE_SELECTED == GPF_IMU_FUSION_TYPE_MADGWICK) {
      myFc.myImu.doFusion_madgwick6DOF();
     }
     if (GPF_IMU_FUSION_TYPE_SELECTED == GPF_IMU_FUSION_TYPE_COMPLEMENTARY_FILTER) {
      myFc.myImu.doFusion_complementaryFilter();
     }
    }

    myFc.getDesiredState(); //Compute desired state //Convert raw commands to normalized values based on saturated control limits
    myFc.controlANGLE();    //PID Controller //Stabilize on angle setpoint
    myFc.controlMixer();    //Actuator mixing and scaling to PWM values //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

    //myFc.set_arm_IsArmed(true); //test remporaire

    if (myFc.get_arm_IsArmed()) {
      // Si armé, on ne va pas dans le menu et on n'affiche rien sur le display non plus. (C'est beaucoup trop long afficher de toute facon)
      // On se contente de ne pas ralentir la loop pour faire voler le drone :-)
      
      if (!isArmed_previous) {
        //On était pas armé le tour d'avant donc on pourrait faire un traitement spécial vue qu'on vient tout juste d'armer        
        myFc.displayArmed();

        if (myFc.get_black_box_IsEnabled()) {
         //myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
         //myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_BLACK_BOX)->print(myFc.get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_LOGGING,true));
         //myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_BLACK_BOX)->println("Armed");
        }
        myFc.resetLoopStats();

        isArmed_previous = true;
      }
            
      myFc.scaleCommands();   //Scales motor commands to DSHOT commands

      //Sécurité - Fail Safe
      if (myFc.myRc.get_isInFailSafe()) { //A penser
       for (uint8_t motorNumber = 0; motorNumber < GPF_MOTOR_ITEM_COUNT; motorNumber++) { 
        myFc.motor_command_DSHOT[motorNumber] = GPF_DSHOT_CMD_MOTOR_STOP; 
       }
      }

      // On envoi les commandes aux ESC seulement lorsqu'on est armé.
      // Lorsque la commande des moteurs ne change pas, ca ne donnerait normalement rien de réenvoyer la commande continuellement 
      // puisque en réalité ce n'est pas la fonction sendCommand() qui envoi le signal aux ESC mais plutôt les DMA. 
      // Mais j'appel cette fonction continuellement quand-même car c'est plus simple comme celà.
      for (uint8_t motorNumber = 0; motorNumber < GPF_MOTOR_ITEM_COUNT; motorNumber++) { 
       myFc.myDshot.sendCommand(motorNumber, myFc.motor_command_DSHOT[motorNumber], false);
      }

    } else {
      //Si pas armé, et bien on affiche le menu sur l'écran tactile.
      
      if (isArmed_previous) {
        //On était armé le tour d'avant ou qu'on revient d'un autre menu donc on pourrait faire un traitement spécial vu qu'on vient tout juste de désarmer
        isArmed_previous = false;

        //On est jamais trop prudent.
        for (uint8_t motorNumber = 0; motorNumber < GPF_MOTOR_ITEM_COUNT; motorNumber++) { 
          myFc.motor_command_DSHOT[motorNumber] = GPF_DSHOT_CMD_MOTOR_STOP; 
          myFc.myDshot.sendCommand(motorNumber, myFc.motor_command_DSHOT[motorNumber], false);
        }

        if (myFc.get_black_box_IsEnabled()) {
         //myFc.mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
        }

        myFc.resetLoopStats();
      }

      myFc.displayAndProcessMenu();

    }

    myFc.manageAlarms();    
}





