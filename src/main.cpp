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

void setup() {
  elapsedMillis sinceDummy;
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin as an output. // Attention, la LED_BUILTIN (pin 13) est la même pin que le SPI SCK sur un Teensy 4.1
  pinMode(GPF_MISC_PIN_BUZZER, OUTPUT); // buzzer/piezo
  pinMode(PIN_A8, INPUT) ;//Diviseur de tension pour lecture du voltage de la batterie sur la pin A8. Voir fonction gpf_util_getVoltage();

  gpf_util_blinkMainBoardLed(3);
  Serial.begin(115200);  //Port USB //C'est long genre 2 secondes ???
  sinceDummy = 0;
  while ((!Serial) && (sinceDummy < 1000)) { }; //Attend maximum 1 seconde que le port serie Serial (USB) soit prêt sinon si port usb non branché, la programme jam ici.
  DEBUG_GPF_PRINTLN(F("Port serie Serial (USB) ouvert."));
  DEBUG_GPF_PRINT(F("prog version:"));  
  DEBUG_GPF_PRINTLN(GPF_MISC_PROG_CURRENT_VERSION);    
  
  gpf_util_blinkMainBoardLed(2);

  //while (1) {
  //  DEBUG_GPF_PRINTLN(gpf_util_getVoltage());
  //  delay(500);
  //}

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

  #if defined GPF_IMU_SENSOR_INSTALLED_MPU6050 
   Wire.begin();           //La librairie I2Cdev ne l'initialise pas donc on doit le faire nous même
   Wire.setClock(1000000); //Important pour pouvoir une loop à 2khz // Note this is 2.5 times the spec sheet 400 kHz max...
   //Wire.setClock(800000); //Important pour pouvoir une loop à 2khz // Note this is 2.5 times the spec sheet 400 kHz max...
   //Wire.setClock(400000); //Important pour pouvoir une loop à 2khz // Note this is 2.5 times the spec sheet 400 kHz max...
   //Wire.setTimeout(3000); //test //On dirait moins d'erreur sur le MPU6050 avec celà... ??? Non finalement je pense ... Mettre AD0 de MPU6050 au ground...
  #endif

  #if defined GPF_IMU_SENSOR_INSTALLED_BMI088
    //La vitesse se fait initialiser à 400 kHz par la librairie bmi088
    //J'ai tenté de changer le code de la librairie bmi088 pour augmenter la vitesse mais ca ne va pas plus vite ???
  #endif

  myFc.initialize(&myConfig);

  //myFc.genDummyTelemetryData(); //Pour fin de tests
  strncpy(myFc.gpf_telemetry_info.flight_mode_description, "GPFlight :-)", GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH);
  
  //myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
  DEBUG_GPF_PRINTLN("*******************************************");    
  DEBUG_GPF_PRINTLN(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_LOGGING,true));    
  DEBUG_GPF_PRINTLN(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US,true));    

  myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);
  myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY,true));
  myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("setup() - Fin");
  myFc.mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);

}

elapsedMillis sinceChange;
uint16_t      lastDshotCommandSent = 0;

void loop() {
    static bool          isArmed_previous             = true;
    static bool          isInFailSafe_local           = false;   
    static bool          isInFailSafe_local_previous  = false;   
    static unsigned long failSafeMotorDecelarationLoopCount = 0;
    static float         failSafeMotorDecelarationStep = 0.0;
    static int           failSafePwmThrottleValue = 0;
    static unsigned long dummyCpt = 0;

    dummyCpt++;

    myFc.iAmStartingLoopNow(true);
    myFc.debugDisplayLoopStats();

    myFc.gpf_telemetry_info.battery_voltage = gpf_util_getVoltage(); 
    
    myFc.myRc.readRx(); //Armé ou non, on va toujours lire la position des sticks
    myFc.set_arm_IsArmed(myFc.get_IsStickInPosition(GPF_RC_STICK_ARM, GPF_RC_CHANNEL_POSITION_HIGH)); //Dans certains cas, on ne permet pas d'armer
    myFc.set_black_box_IsEnabled(myFc.get_IsStickInPosition(GPF_RC_STICK_BLACK_BOX, GPF_RC_CHANNEL_POSITION_HIGH));
    myFc.get_set_flightMode();
    myFc.myImu.set_fusion_type(myFc.flight_mode);

    if (myFc.myImu.getIMUData()) { //Armé ou non, on va toujours lire le IMU
      myFc.myImu.doFusion(); //Madwick ou Complementary filder selon la position de la switch mode de vol.
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

        myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US,true));
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("Arm");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("Kp_roll_angle,Ki_roll_angle,Kd_roll_angle,Kp_pitch_angle,Ki_pitch_angle,Kd_pitch_angle,Kp_yaw,Ki_yaw,Kd_yaw,GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER,B_madgwick,B_accel,B_gyro");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_ROLL][GPF_PID_TERM_PROPORTIONAL]  / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_ROLL][GPF_PID_TERM_INTEGRAL]      / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_ROLL][GPF_PID_TERM_DERIVATIVE]    / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_PITCH][GPF_PID_TERM_PROPORTIONAL] / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_PITCH][GPF_PID_TERM_INTEGRAL]     / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_PITCH][GPF_PID_TERM_DERIVATIVE]   / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_YAW][GPF_PID_TERM_PROPORTIONAL]   / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_YAW][GPF_PID_TERM_INTEGRAL]       / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myConfig.pids[GPF_AXE_YAW][GPF_PID_TERM_DERIVATIVE]     / GPF_PID_STORAGE_MULTIPLIER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(GPF_IMU_FUSION_WEIGHT_GYRO_COMPLEMENTARY_FILTER,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myFc.myImu.B_madgwick,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myFc.myImu.B_accel,6);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(myFc.myImu.B_gyro,6);
        //myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(",");

        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("");
        myFc.mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);

        if (myFc.get_black_box_IsEnabled()) {
         myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
         myFc.black_box_writeHeader();
        }
        myFc.resetLoopStats();

        isArmed_previous = true;
      }
            
      myFc.scaleCommands();   //Scales motor commands to DSHOT commands

      //Sécurité - Fail Safe
      isInFailSafe_local_previous = isInFailSafe_local;
      isInFailSafe_local = myFc.myRc.get_isInFailSafe();

      if (isInFailSafe_local) {
       //if ((dummyCpt % 250) == 0) {
       // DEBUG_GPF_PRINTLN("isInFailSafe=oui"); 
       //}

       // Lorsqu'on entre en failSafe, on remet tous les channels à 1500 (sauf le throttle) pour laisser le drone se 
       // stabiliser. Cependant pendant 5 secondes, je force le throttle à diminuer jusqu'à 0.
       // Todo: Ajouter condition avec lidar si distance avec le sol est plus petite que 0.5 mètre environ.
 
       if (!isInFailSafe_local_previous) { //On entre en failSafe 
         failSafeMotorDecelarationLoopCount = 0;

         //Faudrait peut-être pas mettre tous les channels à 1500 genre Throttle, ARM et mode de vol, etc...
         //Faudrait plutot forcer que le yaw, roll et pitch
         myFc.myRc.forcePwmChannelYawRollPitchToNeutral(myConfig.channelMaps[GPF_RC_STICK_YAW],myConfig.channelMaps[GPF_RC_STICK_ROLL],myConfig.channelMaps[GPF_RC_STICK_PITCH]); 
         failSafePwmThrottleValue       = myFc.myRc.getPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_THROTTLE]); 
         failSafeMotorDecelarationStep  = max(0.0,failSafePwmThrottleValue - GPF_RC_CHANNEL_VALUE_MIN) / (GPF_FAILSAFE_MOTORS_DECELERATION_DURATION / GPF_MAIN_LOOP_RATE);
       
         DEBUG_GPF_PRINTLN("*****On entre en failSafe ******************************************************** ");
         DEBUG_GPF_PRINT("***** failSafeMotorDecelarationStep=");
         DEBUG_GPF_PRINTLN(failSafeMotorDecelarationStep);       
       }     

       if (myFc.myRc.getFailSafeDuration() < GPF_FAILSAFE_MOTORS_DECELERATION_DURATION) { //Pendant 5 secondes, on rallenti les moteurs jusqu'à 0.
        myFc.myRc.setPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_THROTTLE],failSafePwmThrottleValue - (failSafeMotorDecelarationLoopCount * failSafeMotorDecelarationStep));

        if ((dummyCpt % 250) == 0) {
          DEBUG_GPF_PRINT("GPF_RC_STICK_THROTTLE=");
          DEBUG_GPF_PRINT(myFc.myRc.getPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_THROTTLE]));
        
          DEBUG_GPF_PRINT(" (f..Count * f..Step)=");
          DEBUG_GPF_PRINTLN((failSafeMotorDecelarationLoopCount * failSafeMotorDecelarationStep));
        }
       } else {
         // Une fois qu'on a laissé le temps aux moteurs de ralentir pendant 5 secondes pour que le drone descendre/tombe en douceur,
         // après ce délai on s'assure que les moteurs arrêtent complètement.
         for (uint8_t motorNumber = 0; motorNumber < GPF_MOTOR_ITEM_COUNT; motorNumber++) { 
          myFc.motor_command_DSHOT[motorNumber] = GPF_DSHOT_CMD_MOTOR_STOP; 
         }
       }

       failSafeMotorDecelarationLoopCount++;
      }

      // On envoi les commandes aux ESC seulement lorsqu'on est armé.
      // Lorsque la commande des moteurs ne change pas, ca ne donnerait normalement rien de réenvoyer la commande continuellement 
      // puisque en réalité ce n'est pas la fonction sendCommand() qui envoi le signal aux ESC mais plutôt les DMA. 
      // Mais j'appel cette fonction continuellement quand-même car c'est plus simple comme celà.
      for (uint8_t motorNumber = 0; motorNumber < GPF_MOTOR_ITEM_COUNT; motorNumber++) { 
       myFc.myDshot.sendCommand(motorNumber, myFc.motor_command_DSHOT[motorNumber], false);
      }

      if (myFc.get_black_box_IsEnabled()) {       
       myFc.black_box_writeRow();
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

        //if (myFc.get_black_box_IsEnabled()) {
         myFc.mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);
        //}

        myFc.mySdCard.openFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_US,true));
        myFc.mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("Desarm");
        myFc.mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);

        myFc.resetLoopStats();
        myFc.menu_pleaseRefresh = true;
      }

      myFc.update_arm_allowArming(); //Call cette fonction seulement lorsque désarmé sinon on ne pourra jamais armer. //Anyway, si on est armé on a plus besoin de savoir si on peut armer.
      myFc.displayAndProcessMenu();

    }

    myFc.manageAlarms();    
    myFc.myMusicPlayer.updatePlaying(myFc.myRc.getPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_MUSIC_PLAYER_VOLUME]), myFc.myRc.getPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_MUSIC_PLAYER_TRACK]), myFc.myRc.getPwmChannelValue(myConfig.channelMaps[GPF_RC_STICK_MUSIC_PLAYER_LIST]));
}





