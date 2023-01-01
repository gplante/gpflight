/**
 * @file gpf.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-18
 * 
 * Class principale du projet
 * 
 */
 
#include "Arduino.h"
#include <EEPROM.h>
#include "Math.h"
#include <TimeLib.h>
#include "gpf.h"
#include "gpf_debug.h"


GPF::GPF() {
    gpf_telemetry_info.battery_voltage           = 0;
    gpf_telemetry_info.battery_current           = 0;
    gpf_telemetry_info.battery_capacity_used     = 0;
    gpf_telemetry_info.battery_remaining_percent = 0;    

    gpf_telemetry_info.vario_vertival_speed      = 0;

    gpf_telemetry_info.gps_latitude              = 0;
    gpf_telemetry_info.gps_longitude             = 0;
    gpf_telemetry_info.gps_groundspeed           = 0;
    gpf_telemetry_info.gps_heading               = 0;
    gpf_telemetry_info.gps_altitude              = 0;
    gpf_telemetry_info.gps_satellites            = 0;

    gpf_telemetry_info.baro_altitude             = 0;

    gpf_telemetry_info.attitude_pitch            = 0;
    gpf_telemetry_info.attitude_roll             = 0;
    gpf_telemetry_info.attitude_yaw              = 0;
}

void GPF::initialize(gpf_config_struct *ptr) {    
    myConfig_ptr = ptr;
    debug_sincePrint = 0;
    mySdCard.initialize();
    
    myImu.initialize(ptr);
    myRc.initialize(&Serial7); 
    myRc.setupTelemetry(&gpf_telemetry_info); 
    myDshot.initialize();

    myDisplay.initialize();
    myDisplay.println("GPFlight");   
    myTouch.initialize();
    
}

void GPF::iAmStartingLoopNow(bool syncLoop) {
 /*
 static int cpt = 0;

 cpt++;
 if (cpt == 3) {
  resetLoopStats();
 }
 */

 if (firstLoopStartedAt == 0) {
     // Uniquement lors de la première loop, on attend le début de la prochaine milliseconde 
     // histoire que nos stats soient le plus précis possible
     //unsigned long current_micros = micros();
     //unsigned long delay_us = 1000 - (current_micros % 1000);
     //delayMicroseconds(delay_us);
     firstLoopStartedAt = micros();
 }
 
 loopBusyTime    = micros() - loopStartedAt;
 loopBusyTimeMin = min(loopBusyTimeMin,loopBusyTime);
 loopBusyTimeMax = max(loopBusyTimeMax,loopBusyTime);
 loopFreeTime    = GPF_MAIN_LOOP_RATE - loopBusyTime; 

 if (loopFreeTime < 0) {
  loopTimeOverFlowCount++;
 }

 loopBusyTimePercent = (float)(loopBusyTime / (float)GPF_MAIN_LOOP_RATE) * 100.0;
 loopFreeTimePercent = 100.0 - loopBusyTimePercent;

 if (syncLoop) {
  //On attend pour que chaque loop commence au même moment
  while ((micros() - loopStartedAt) < GPF_MAIN_LOOP_RATE) {
   //Attend (Dans la mesure du possible, ne jamais  mettre de delay() ou delayMicroseconds() dans un programme!!!)
  }
 } 

 loopStartedAt = micros(); 
 loopCount++; 
}

void GPF::debugDisplayLoopStats() {
 #ifdef DEBUG_GPF_ENABLED 
  if (debug_sincePrint > DEBUG_GPF_DELAY) {
   DEBUG_GPF_PRINT("GPF: loopCount=");
   DEBUG_GPF_PRINT(loopCount);
   DEBUG_GPF_PRINT(F(" "));
   DEBUG_GPF_PRINT(getLoopFrequency());
   DEBUG_GPF_PRINT("hz loopBusyTime=");
   DEBUG_GPF_PRINT(loopBusyTime);
   DEBUG_GPF_PRINT("/");
   DEBUG_GPF_PRINT(GPF_MAIN_LOOP_RATE);
   DEBUG_GPF_PRINT(" loopBusyTimePercent=");
   DEBUG_GPF_PRINT(loopBusyTimePercent);
   DEBUG_GPF_PRINT("%");
   DEBUG_GPF_PRINT(" free ram=");
   DEBUG_GPF_PRINT(gpf_util_freeRam());
   DEBUG_GPF_PRINT(" loopBusyTimeMin=");
   DEBUG_GPF_PRINT(loopBusyTimeMin);
   DEBUG_GPF_PRINT(" loopBusyTimeMax=");
   DEBUG_GPF_PRINT(loopBusyTimeMax);
   DEBUG_GPF_PRINT(" loopTimeOverFlowCount=");
   DEBUG_GPF_PRINT(loopTimeOverFlowCount);

   DEBUG_GPF_PRINTLN();

   debug_sincePrint = 0;

   //ptrMemoryLeakTest = (byte*)malloc(5000); //Test pour créer un memory leak
  }
 #endif
}

void GPF::resetLoopStats() {
 firstLoopStartedAt    = 0; 
 loopCount             = 0; 
 loopBusyTimeMin       = 999999;
 loopBusyTimeMax       = 0;
 loopTimeOverFlowCount = 0;

 //if (firstLoopStartedAt == 0) {
     // Uniquement lors de la première loop, on attend le début de la prochaine milliseconde 
     // histoire que nos stats soient le plus précis possible
     //unsigned long us       = micros();
     //unsigned long delay_us = 1000 - (us % 1000);
     //delayMicroseconds(delay_us);
     //firstLoopStartedAt = micros();
 //}

}

void GPF::waitUntilNextLoop() {
 static elapsedMicros since_lastLoop_static = 0;

 while (since_lastLoop_static > 2000) {
    delayMicroseconds(1);
 }
 since_lastLoop_static = 0;
}

void GPF::toggMainBoardLed() {    
  // Attention, la LED_BUILTIN (pin 13) est la même pin que le SPI SCK sur un Teensy 4.1 
  // alors une fois mon SPI initialisé, je ne devrait plus me servir de cette fonction.
  static bool ledState = true;
  static elapsedMillis since_ledToggled = 0;

  if (since_ledToggled > GPF_MAIN_LED_TOGGLE_DURATION) {
   ledState = !ledState;
   digitalWrite(LED_BUILTIN, ledState);
   since_ledToggled = 0;
  }    
}

void GPF::genDummyTelemetryData() {
  gpf_telemetry_info.battery_voltage = 168; //1=0.1v, 168=16.8v
  gpf_telemetry_info.battery_current     = 457; //1=0.1A, 457=45.7A
  gpf_telemetry_info.battery_capacity_used = 1234;
  gpf_telemetry_info.battery_remaining_percent = 63;
  gpf_telemetry_info.vario_vertival_speed = 5432; //-30000;

  gpf_telemetry_info.gps_latitude    =  466589756; //466589756=46.6589756 = 46 deg 39 min...
  gpf_telemetry_info.gps_longitude   = -713100184; // -713100184=-71.3100184 = -71 deg 18 min...
  gpf_telemetry_info.gps_groundspeed = 123; //123=12.3kh
  gpf_telemetry_info.gps_heading     = 16500; //16500=16.50, 31400=31.40, 32000=32, 0=0, 18000=18, 1800=1.80, ??? Pas plus que 32 degreés ???
  gpf_telemetry_info.gps_altitude    = 500;
  gpf_telemetry_info.gps_satellites  = 6;

  gpf_telemetry_info.baro_altitude   = 1235;

  gpf_telemetry_info.attitude_pitch = 31200;  //rad  * 10000
  gpf_telemetry_info.attitude_roll  = 349;    //rad  * 10000
  gpf_telemetry_info.attitude_yaw   = -12000; //rad  * 10000

  strncpy(gpf_telemetry_info.flight_mode_description, "Yhe2 :-)ABCDEFGH", GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH);

}

void GPF::menu_gotoTestTouchScreen(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {
  bool    previousTouched = true;
  int16_t minX = 9999, minY = 9999;
  int16_t maxX = 0, maxY = 0;

  Adafruit_GFX_Button_2 exitButton;

  myDisplay.setTextSize(4);
  myDisplay.setRotation(0);
  myDisplay.get_tft()->setScroll(0);

  myTouch.get_ts()->setRotation(2);

  while (true) {
    boolean istouched = myTouch.ts_touched();

    if (istouched) {
     TS_Point p = myTouch.ts_getPoint();

     minX = min(minX, p.x);
     minY = min(minY, p.y);
     maxX = max(maxX, p.x);
     maxY = max(maxY, p.y);

     int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
     int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

     if (!previousTouched) {
      if (button_Exit.contains(pixelX, pixelY)) {
        DEBUG_GPF_PRINT("On sort de la fonction ");
        DEBUG_GPF_PRINTLN(__func__);
        
        //menu_current = GPF_MENU_TEST_MENU;
        menu_current = gpf_menuItems[GPF_MENU_TEST_TOUCH].parent_id;
        menu_pleaseRefresh = true;
        myTouch.set_waitForUnTouch(true);
        return;
      }

      myDisplay.get_tft()->fillScreen(ILI9341_BLACK);
      myDisplay.get_tft()->setTextColor(ILI9341_YELLOW);
      myDisplay.get_tft()->setFont(Arial_32);
      myDisplay.get_tft()->setCursor(60, 80);
      myDisplay.get_tft()->print(F("Touch"));
     }

     myDisplay.get_tft()->fillRect(0, 150, myDisplay.getDisplayWidth(), 60, ILI9341_BLACK);
     myDisplay.get_tft()->setTextColor(ILI9341_GREEN);
     myDisplay.get_tft()->setFont(Arial_20);
     myDisplay.get_tft()->setCursor(0, 150);
     myDisplay.get_tft()->print(F("X= "));
     myDisplay.get_tft()->print(p.x);
     myDisplay.get_tft()->print(F(" pX= "));
     myDisplay.get_tft()->print(pixelX);
     myDisplay.get_tft()->setCursor(0, 180);
     myDisplay.get_tft()->print(F("Y= "));
     myDisplay.get_tft()->print(p.y);
     myDisplay.get_tft()->print(F(" pY= "));
     myDisplay.get_tft()->print(pixelY);
     
    } else {
     if (previousTouched) {
      myDisplay.get_tft()->fillScreen(ILI9341_BLACK);
      myDisplay.get_tft()->setTextColor(ILI9341_RED);
      myDisplay.get_tft()->setFont(Arial_32);
      myDisplay.get_tft()->setCursor(100, 50);
      myDisplay.get_tft()->print(F("No"));
      myDisplay.get_tft()->setCursor(70, 120);
      myDisplay.get_tft()->print(F("Touch"));

      myDisplay.get_tft()->setFontAdafruit();
      
      menu_display_button_Exit();
     }
     //DEBUG_GPF_PRINTLN(F("no touch"));
    }

    previousTouched = istouched;
  }
  
}

bool GPF::get_IsStickInPositionEnabled(uint8_t stick) {    
  bool retour = false;
  unsigned int pos;

  if (myConfig_ptr != NULL) {
    pos    = myRc.getPwmChannelPos(myConfig_ptr->channelMaps[stick]);
    retour = gpf_util_isPwmChannelAtPos(pos, GPF_RC_CHANNEL_ENABLED_VALUE);
  }
  
  return retour;
}

bool GPF::get_arm_IsArmed() {    
  return arm_isArmed;
}

bool GPF::set_arm_IsArmed(bool b) {
  arm_isArmed = b && arm_allowArming;
  return arm_isArmed;
}

bool GPF::get_black_box_IsEnabled() {    
  return black_box_isEnabled;
}

bool GPF::set_black_box_IsEnabled(bool b) {
  black_box_isEnabled = b;
  return black_box_isEnabled;
}

void GPF::displayArmed() {    
  myDisplay.setTextSize(8);
  myDisplay.setRotation(1);
  myDisplay.get_tft()->setScroll(0);
  myDisplay.get_tft()->fillScreen(ILI9341_BLACK);
  myDisplay.get_tft()->setTextColor(ILI9341_RED);      
  myDisplay.get_tft()->println();
  myDisplay.get_tft()->println(F(" ARMED"));
  myDisplay.get_tft()->println();    
}

void GPF::menu_gotoInfoStats(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {  
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;
  const uint16_t x_pos = 120;
  static elapsedMillis sincePrint = 1001; //pour que le tout s'affiche tout de suite dès le premier appel de la fonction.
  const uint16_t sincePrint_delay = 1000;

  if (firstTime) {    
    myDisplay.clearScreen();
    menu_display_button_Exit();
    menu_display_button_Save("Reset Sta");

    myDisplay.setTextSize(2);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.println("**** Main Loop ****");
    myDisplay.println("    Count");  
    myDisplay.println("     Freq");
    myDisplay.println("Busy Time");
    myDisplay.println("   Busy %");
    myDisplay.println("BusyT.Min");
    myDisplay.println("BusyT.Max");
    myDisplay.println("OverF Cpt");
    myDisplay.println(" Free RAM");
    myDisplay.println("Ver. Prog");
    myDisplay.println("Ver. Conf");
    
    /*
    u_int64_t taille = mySdCard.getFileObject(GPF_SDCARD_FILE_TYPE_BLACK_BOX)->size();
    myDisplay.print("ferme: ");

    unsigned long myMillis = millis();
    mySdCard.closeFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);     
    myDisplay.println(millis() - myMillis);

    myDisplay.print("ouver: ");
    myMillis = millis();
    mySdCard.openFile(GPF_SDCARD_FILE_TYPE_BLACK_BOX);     
    myDisplay.println(millis() - myMillis);
    
    myDisplay.print("taill: ");
    myDisplay.println(taille);
    */
  }

  if (sincePrint > sincePrint_delay) {
    sincePrint = 0;
    myDisplay.setTextSize(2);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.println();

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(loopCount);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.print(getLoopFrequency());
    myDisplay.println("hz");

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.print(loopBusyTime);
    myDisplay.print("/");
    myDisplay.println(GPF_MAIN_LOOP_RATE);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.print(loopBusyTimePercent);
    myDisplay.println("%");

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(loopBusyTimeMin);    

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(loopBusyTimeMax);    

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(loopTimeOverFlowCount);    

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(gpf_util_freeRam());

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(GPF_MISC_PROG_CURRENT_VERSION);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(GPF_MISC_CONFIG_CURRENT_VERSION);
    

  }

  boolean istouched = myTouch.ts_touched();

  if (istouched) { //Check si il a cliqué sur bouton "Sortir"
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   if (button_Exit.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);
    
    menu_current = gpf_menuItems[GPF_MENU_INFO_STATS].parent_id;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Save.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Reset"
    DEBUG_GPF_PRINT("Reset loop stats");
    DEBUG_GPF_PRINTLN(__func__);
    resetLoopStats();
    sincePrint = 1001; //Pour que ca s'affiche tout de suite au prochain appel de cette fonction. Genre pour ne pas attendre 1 seconde...
    myTouch.set_waitForUnTouch(true);
   }
  }  
}

void GPF::menu_gotoTestRC(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;

  if (firstTime) {
    myDisplay.clearScreen();
    menu_display_button_Exit();
  }

  myDisplay.setTextSize(2);
  myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

  for (size_t channel = 1; channel <= GPF_RC_NUMBER_CHANNELS; channel++) {     
   myDisplay.get_tft()->fillRect(0, (channel -1) * charHeight, myDisplay.getDisplayWidth(), charHeight, ILI9341_BLACK);
   myDisplay.get_tft()->setCursor(0,(channel -1) * charHeight);  
   myDisplay.print(channel);
   myDisplay.print(F(": "));
   myDisplay.print(myRc.getPwmChannelValue(channel));
   myDisplay.print(" ");

   //Si c'est un channel qui est utilisé par un des stick/switch, on affiche le nom du stick/switch
   for (size_t rcStick = 0; rcStick < GPF_RC_STICK_ITEM_COUNT; rcStick++) {
    if (channel == myConfig_ptr->channelMaps[rcStick]) {
      myDisplay.print(gpf_rc_stick_descriptions[rcStick]);
      myDisplay.print(" ");
    }
   }   

   myDisplay.println();
  }
  
  boolean istouched = myTouch.ts_touched();

  if (istouched) { //Check si il a cliqué sur bouton "Sortir"
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   if (button_Exit.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    menu_current = gpf_menuItems[GPF_MENU_TEST_RC].parent_id;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }
  }        

}

void GPF::menu_gotoConfigurationChannels(bool firstTime, int rcStick=0, int not_used_param_3=0) {  
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;
  uint16_t x          = 0;
  uint16_t y          = 1;

  static uint16_t x_new_channel = 0;
  static uint16_t y_new_channel = 0;

  static uint8_t newChannel     = 0;

  const uint16_t buttonHeight = 50;
  const uint16_t buttonWidth  = 50;
  const uint16_t buttonSpace  = 10;

  myDisplay.setTextSize(2);
  myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

  if (firstTime) {
    newChannel = myConfig_ptr->channelMaps[rcStick];
    myDisplay.clearScreen();
    menu_display_button_Exit();

    myDisplay.get_tft()->setCursor(0,y);

    myDisplay.print("Stick ");
    myDisplay.print(gpf_rc_stick_descriptions[rcStick]);
    myDisplay.print(" *");
    myDisplay.print(myConfig_ptr->channelMaps[rcStick]);
    myDisplay.println();

    myDisplay.print("New Channel = ");
    x_new_channel = myDisplay.get_tft()->getCursorX();
    y_new_channel = myDisplay.get_tft()->getCursorY();
    myDisplay.println();
    
    x = 5;
    y = myDisplay.get_tft()->getCursorY();    
    y = y + buttonSpace;

    for (size_t channel = 1; channel <= GPF_RC_NUMBER_CHANNELS; channel++) {
     menu_display_button_n(channel,x,y,buttonWidth,buttonHeight); 
      x = x + buttonWidth + buttonSpace;
      if (x + buttonWidth >= myDisplay.getDisplayWidth()) {
        x = 5;
        y = y + buttonHeight + buttonSpace;
      }
    }

    menu_display_button_Save("Save");
    
  }

  //Affiche la valeur sélectionnée par l'utilisateur  
  myDisplay.get_tft()->fillRect(x_new_channel, y_new_channel, myDisplay.getDisplayWidth() - x_new_channel, charHeight, ILI9341_BLACK);
  myDisplay.get_tft()->setCursor(x_new_channel,y_new_channel);  
  myDisplay.print(newChannel);
  
  boolean istouched = myTouch.ts_touched();

  if (istouched) { 
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   for (size_t channel = 1; channel <= GPF_RC_NUMBER_CHANNELS; channel++) { 
     if (buttons[channel].contains(pixelX, pixelY)) { //Check si il a cliqué sur un des boutons
      newChannel = channel;
      myTouch.set_waitForUnTouch(true);
     }
   }

   if (button_Save.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Save"
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);
    DEBUG_GPF_PRINT("Stick= ");
    DEBUG_GPF_PRINT(rcStick);
    DEBUG_GPF_PRINT(" newChannel= ");
    DEBUG_GPF_PRINT(newChannel);
    DEBUG_GPF_PRINTLN();

    //Met à jour la config et save config
    myConfig_ptr->channelMaps[rcStick] = newChannel; 
    saveConfig();

    menu_current = GPF_MENU_CONFIG_CHANNELS_MENU;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Exit.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Sortir"
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    menu_current = GPF_MENU_CONFIG_CHANNELS_MENU;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }
  }
  

}

void GPF::displayAndProcessMenu() {
  bool   firstTime       = false;
         uint16_t y      = 60;  
  const  uint8_t  y_step = 40;  
         int new_menu_id = menu_current;
  static int menuFunction_param_2 = 0;  
  static int menuFunction_param_3 = 0;  
  
  arm_allowArming = (menu_current == GPF_MENU_MAIN_MENU) && (false); //todo //Il faut être au menu principal et que la switch Arm soit à Off
  
  if (menu_pleaseRefresh) {
   DEBUG_GPF_PRINT(F("menu_current=")); 
   DEBUG_GPF_PRINTLN(menu_current);

   myDisplay.get_tft()->setFontAdafruit();
   myTouch.get_ts()->setRotation(2);

   myDisplay.setTextSize(2);
   myDisplay.setRotation(0);
   myDisplay.get_tft()->setScroll(0);
   myDisplay.get_tft()->fillScreen(ILI9341_BLACK);
   
   myDisplay.get_tft()->setTextColor(ILI9341_GREEN);      
   myDisplay.get_tft()->setCursor(15,1);
   myDisplay.setTextSize(2);
   myDisplay.print("GP");   
   myDisplay.get_tft()->setTextColor(ILI9341_CYAN);      
   myDisplay.print("Flight ");   
   myDisplay.get_tft()->setTextColor(ILI9341_GREEN);      
   myDisplay.setTextSize(1);
   myDisplay.get_tft()->setCursor(116,8);
   myDisplay.print("par Guylain Plante");

   myDisplay.setTextSize(2);
   myDisplay.get_tft()->setTextColor(ILI9341_YELLOW);      

   for (size_t menu_id = 0; menu_id < GPF_MENU_ITEM_COUNT; menu_id++) {
    if ((gpf_menuItems[menu_id].parent_id == menu_current) && ((gpf_menuItems[menu_id].id != GPF_MENU_MAIN_MENU))) {
     gpf_menuItems[menu_id].button.initButton(myDisplay.get_tft(),120,y,236,36,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,gpf_menuItems[menu_id].caption,2);
     gpf_menuItems[menu_id].button.drawButton();
     y = y + y_step;
    }
   }

   //On ajoute un bouton "Retour" pour revenir en arrière
   if (gpf_menuItems[menu_current].id != GPF_MENU_MAIN_MENU) {
    menu_display_button_Exit();
   }
   
   menu_pleaseRefresh = false;

   //Test
   if (gpf_menuItems[menu_current].id == GPF_MENU_MAIN_MENU) {
    resetLoopStats();
   }   
  }
  
  boolean istouched = myTouch.ts_touched();
  if (istouched) {
     //DEBUG_GPF_PRINTLN(F("touched*")); 
     TS_Point p = myTouch.ts_getPoint();

     int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
     int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

     for (size_t menu_id = 0; menu_id < GPF_MENU_ITEM_COUNT; menu_id++) {
      if ((gpf_menuItems[menu_id].parent_id == menu_current) && ((gpf_menuItems[menu_id].id != GPF_MENU_MAIN_MENU))) {
        if (gpf_menuItems[menu_id].button.contains(pixelX, pixelY)) {
         DEBUG_GPF_PRINT(F("menu_id=")); 
         DEBUG_GPF_PRINTLN(menu_id); 
         firstTime = true;
         //menu_pleaseRefresh = true;

         if (gpf_menuItems[menu_id].menuFunction == NULL) { 
          //C'est un sous-menu
          DEBUG_GPF_PRINTLN(F("C'est un menu")); 
          menu_pleaseRefresh = true;
         } else {
          //C'est une fonction
          DEBUG_GPF_PRINTLN(F("C'est une fonction")); 

          switch (menu_id) {
          case GPF_MENU_CONFIG_CHANNELS_ROLL:
            menuFunction_param_2 = GPF_RC_STICK_ROLL;
            break;

          case GPF_MENU_CONFIG_CHANNELS_PITCH:
            menuFunction_param_2 = GPF_RC_STICK_PITCH;
            break;

          case GPF_MENU_CONFIG_CHANNELS_THROTTLE:
            menuFunction_param_2 = GPF_RC_STICK_THROTTLE;
            break;

          case GPF_MENU_CONFIG_CHANNELS_YAW:
            menuFunction_param_2 = GPF_RC_STICK_YAW;
            break;

          case GPF_MENU_CONFIG_CHANNELS_ARM:
            menuFunction_param_2 = GPF_RC_STICK_ARM;
            break;        

          case GPF_MENU_CONFIG_CHANNELS_FLIGHT_MODE:
            menuFunction_param_2 = GPF_RC_STICK_FLIGHT_MODE;
            break;          

          case GPF_MENU_CONFIG_CHANNELS_BLACK_BOX:
            menuFunction_param_2 = GPF_RC_STICK_BLACK_BOX;
            break;            

          case GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_PROPORTIONAL:
            menuFunction_param_2 = GPF_AXE_ROLL;
            menuFunction_param_3 = GPF_PID_TERM_PROPORTIONAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_INTEGRAL:
            menuFunction_param_2 = GPF_AXE_ROLL;
            menuFunction_param_3 = GPF_PID_TERM_INTEGRAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_DERIVATIVE:
            menuFunction_param_2 = GPF_AXE_ROLL;
            menuFunction_param_3 = GPF_PID_TERM_DERIVATIVE;
            break;

          case GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_PROPORTIONAL:
            menuFunction_param_2 = GPF_AXE_PITCH;
            menuFunction_param_3 = GPF_PID_TERM_PROPORTIONAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_INTEGRAL:
            menuFunction_param_2 = GPF_AXE_PITCH;
            menuFunction_param_3 = GPF_PID_TERM_INTEGRAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_DERIVATIVE:
            menuFunction_param_2 = GPF_AXE_PITCH;
            menuFunction_param_3 = GPF_PID_TERM_DERIVATIVE;
            break;      

          case GPF_MENU_CONFIG_PID_AXE_YAW_TERM_PROPORTIONAL:
            menuFunction_param_2 = GPF_AXE_YAW;
            menuFunction_param_3 = GPF_PID_TERM_PROPORTIONAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_YAW_TERM_INTEGRAL:
            menuFunction_param_2 = GPF_AXE_YAW;
            menuFunction_param_3 = GPF_PID_TERM_INTEGRAL;
            break;

          case GPF_MENU_CONFIG_PID_AXE_YAW_TERM_DERIVATIVE:
            menuFunction_param_2 = GPF_AXE_YAW;
            menuFunction_param_3 = GPF_PID_TERM_DERIVATIVE;
            break;  
          
          default:
            break;
          }
                    
         }

         myTouch.set_waitForUnTouch(true);
         new_menu_id = menu_id;
         break;
        }
      }
     }

     //Vérifi aussi si le bouton "Retour" aurait été pressé
     if (button_Exit.contains(pixelX, pixelY)) {
      DEBUG_GPF_PRINTLN("Retour");
      new_menu_id = gpf_menuItems[menu_current].parent_id;
      menu_pleaseRefresh = true;
      myTouch.set_waitForUnTouch(true);
     }
  }

  menu_current = new_menu_id;
  
  if (gpf_menuItems[new_menu_id].menuFunction != NULL) { 
   method_function func = gpf_menuItems[new_menu_id].menuFunction;
   (this->*func)(firstTime, menuFunction_param_2, menuFunction_param_3);
  }  

}

void GPF::menu_display_button_Exit() {
  button_Exit.initButton(myDisplay.get_tft(),60,300,118,36,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,"Retour",2);
  button_Exit.drawButton();
}

void GPF::menu_display_button_Save(const char *caption) { //Peut servir aussi comme bouton avec un caption différent
  button_Save.initButton(myDisplay.get_tft(),180,300,118,36,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,caption,2);  
  button_Save.drawButton();
}

void GPF::menu_display_button_BackSpace() {  
  button_BackSpace.initButton(myDisplay.get_tft(),180,188,118,50,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,"Back",2);
  button_BackSpace.drawButton();
}

void GPF::menu_display_button_Start() {
  button_Start.initButton(myDisplay.get_tft(),60,260,118,36,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,"Start",2);
  button_Start.drawButton();
}

void GPF::menu_display_button_Reset() {
  button_Reset.initButton(myDisplay.get_tft(),180,260,118,36,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,"Reset",2);
  button_Reset.drawButton();
}

void GPF::menu_display_button_n(uint8_t buttonNumber, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  char caption[3];
  itoa(buttonNumber, caption, 10);
  buttons[buttonNumber].initButton(myDisplay.get_tft(),x + round(w/2),y + round(h/2),w,h,ILI9341_YELLOW, ILI9341_BLACK,ILI9341_YELLOW,caption,2);
  buttons[buttonNumber].drawButton();
}

void GPF::saveConfig() {
 EEPROM.put(0, *myConfig_ptr);  
 DEBUG_GPF_PRINT("Save de la config ");
 DEBUG_GPF_PRINTLN(__func__);
}

void GPF::menu_gotoConfigurationPID(bool firstTime, int axe=0, int pid_term=0) {  
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;
  uint16_t x          = 0;
  uint16_t y          = 1;

  static uint16_t x_new_value = 0;
  static uint16_t y_new_value = 0;

  const uint8_t  newValueMaxChar   = 6;
  static char    newValue[newValueMaxChar + 1] = ""; //999999 \0
  static uint8_t newValueCharCount = 0;

  const uint16_t buttonHeight = 50;
  const uint16_t buttonWidth  = 50;
  const uint16_t buttonSpace  = 10;

  myDisplay.setTextSize(2);
  myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

  if (firstTime) {
    itoa(myConfig_ptr->pids[axe][pid_term], newValue, 10);
    newValueCharCount = strlen(newValue);
    myDisplay.clearScreen();
    menu_display_button_Exit();

    myDisplay.get_tft()->setCursor(0,y);

    myDisplay.print("PID ");
    myDisplay.print(gpf_axe_descriptions[axe]);
    myDisplay.print(" ");
    myDisplay.print(gpf_pid_term_descriptions[pid_term]);
    myDisplay.print(" *");
    myDisplay.print(myConfig_ptr->pids[axe][pid_term]);
    myDisplay.println();

    myDisplay.print("New Gain = ");
    x_new_value = myDisplay.get_tft()->getCursorX();
    y_new_value = myDisplay.get_tft()->getCursorY();
    myDisplay.println();
    
    x = 5;
    y = myDisplay.get_tft()->getCursorY();    
    y = y + buttonSpace;

    for (size_t numero = 0; numero <= 9; numero++) {
     //DEBUG_GPF_PRINT(" x="); 
     //DEBUG_GPF_PRINT(x); 
     //DEBUG_GPF_PRINT(" y="); 
     //DEBUG_GPF_PRINT(y); 
     menu_display_button_n(numero,x,y,buttonWidth,buttonHeight); 
      x = x + buttonWidth + buttonSpace;
      if (x + buttonWidth >= myDisplay.getDisplayWidth()) {
        x = 5;
        y = y + buttonHeight + buttonSpace;
      }
    }

    menu_display_button_Save("Save");
    menu_display_button_BackSpace();
    
  }

  //Affiche la valeur sélectionnée par l'utilisateur  
  myDisplay.get_tft()->fillRect(x_new_value, y_new_value, myDisplay.getDisplayWidth() - x_new_value, charHeight, ILI9341_BLACK);
  myDisplay.get_tft()->setCursor(x_new_value,y_new_value);  
  myDisplay.print(newValue);
  
  boolean istouched = myTouch.ts_touched();

  if (istouched) { 
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   for (size_t numero = 0; numero <= 9; numero++) {
     if (buttons[numero].contains(pixelX, pixelY)) { //Check si il a cliqué sur un des boutons
      if (newValueCharCount + 1 <= newValueMaxChar) {
       //newValue[newValueCharCount] = 
       switch (numero) {
       case 0:
        if (newValueCharCount > 0) {
         newValue[newValueCharCount] = '0'; //On ne permet pas de saisir un 0 en début
        }
        break;
       
       case 1:
        newValue[newValueCharCount] = '1';
        break;

       case 2:
        newValue[newValueCharCount] = '2';
        break;

       case 3:
        newValue[newValueCharCount] = '3';
        break;

       case 4:
        newValue[newValueCharCount] = '4';
        break;

       case 5:
        newValue[newValueCharCount] = '5';
        break;

       case 6:
        newValue[newValueCharCount] = '6';
        break;

       case 7:
        newValue[newValueCharCount] = '7';
        break;

       case 8:
        newValue[newValueCharCount] = '8';
        break;

       case 9:
        newValue[newValueCharCount] = '9';
        break;        
       }

       if (!((numero == 0) && (newValueCharCount == 0))) {
        newValueCharCount++;
        newValue[newValueCharCount] = '\0';
       }
      }
      

      //newValue = numero;
      DEBUG_GPF_PRINT("numero=");
      DEBUG_GPF_PRINTLN(numero);
      myTouch.set_waitForUnTouch(true);
     }
   }

   if (button_BackSpace.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Back Space"
     if (newValueCharCount > 0) {      
      newValueCharCount--;
      newValue[newValueCharCount] = '\0';      
     }
     DEBUG_GPF_PRINTLN("Back Space");
     myTouch.set_waitForUnTouch(true);
   }

   if (button_Save.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Save"
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);
    DEBUG_GPF_PRINT("axe=");
    DEBUG_GPF_PRINT(axe);
    DEBUG_GPF_PRINT(" pid_term=");
    DEBUG_GPF_PRINT(pid_term);
    DEBUG_GPF_PRINT(" newValue= ");
    DEBUG_GPF_PRINT(newValue);
    DEBUG_GPF_PRINTLN();

    //Met à jour la config et save config
    myConfig_ptr->pids[axe][pid_term] = atoi(newValue);
    saveConfig();

    switch (axe) {
     case GPF_AXE_ROLL:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_ROLL_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_PROPORTIONAL].parent_id;
      break;
    
     case GPF_AXE_PITCH:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_PITCH_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_PROPORTIONAL].parent_id;
      break;

     case GPF_AXE_YAW:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_YAW_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_YAW_TERM_PROPORTIONAL].parent_id;
      break;  
    }
    
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Exit.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Sortir"
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    switch (axe) {
     case GPF_AXE_ROLL:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_ROLL_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_PROPORTIONAL].parent_id;
      break;
    
     case GPF_AXE_PITCH:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_PITCH_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_PROPORTIONAL].parent_id;
      break;

     case GPF_AXE_YAW:
      //menu_current = GPF_MENU_CONFIG_PID_AXE_YAW_MENU;
      menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_AXE_YAW_TERM_PROPORTIONAL].parent_id;
      break;  
    }

    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }
  }  

}

void GPF::menu_gotoCalibrationIMU(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;

  if (firstTime) {
    myDisplay.clearScreen();
    menu_display_button_Start();
    menu_display_button_Exit();
    menu_display_button_Reset();

    myDisplay.setTextSize(2);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 


    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.println("Current");
    myDisplay.print("ax ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]);
    myDisplay.print("ay ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]);
    myDisplay.print("az ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]);
    myDisplay.print("gx ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]);
    myDisplay.print("gy ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]);
    myDisplay.print("gz ");
    myDisplay.println(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]);
    
  }

  boolean istouched = myTouch.ts_touched();

  if (istouched) { //Check si il a cliqué sur bouton "Sortir"
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);
   int16_t x_pos  = 120;

   if (button_Exit.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    //menu_current = GPF_MENU_CONFIG_MENU;
    menu_current = gpf_menuItems[GPF_MENU_CONFIG_CALIBRATION_IMU].parent_id;
    
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Start.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On démarre la calibration");
    DEBUG_GPF_PRINTLN(__func__);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

    myDisplay.get_tft()->setCursor(0,140);  
    myDisplay.println("Calibration en");
    myDisplay.println("cours veuillez");
    myDisplay.println("patienter s.v.p....");

    myImu.calibrate();

    myDisplay.get_tft()->fillRect(0, 0, myDisplay.getDisplayWidth(), charHeight * 15, ILI9341_BLACK);    

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.print("Current");
    myDisplay.get_tft()->setCursor(x_pos,0);  
    myDisplay.println("Nouvelles");

    myDisplay.print("ax ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_ax);

    myDisplay.print("ay ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_ay);

    myDisplay.print("az ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_az);

    myDisplay.print("gx ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gx);

    myDisplay.print("gy ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gy);

    myDisplay.print("gz ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gz);

    menu_display_button_Save("Save");
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Save.contains(pixelX, pixelY)) { //Check si il a cliqué sur bouton "Save"
    DEBUG_GPF_PRINT("Save et on sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    //Met à jour la config et save config
    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]  = myImu.calibration_offset_ax;
    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]  = myImu.calibration_offset_ay;
    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]  = myImu.calibration_offset_az;

    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]      = myImu.calibration_offset_gx;
    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]      = myImu.calibration_offset_gy;
    myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]      = myImu.calibration_offset_gz;

    saveConfig();
    //menu_current = GPF_MENU_CONFIG_MENU;
    menu_current = gpf_menuItems[GPF_MENU_CONFIG_CALIBRATION_IMU].parent_id;
        
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }

   if (button_Reset.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("Reset offsets ");
    DEBUG_GPF_PRINTLN(__func__);

    myImu.calibration_offset_ax = 0;
    myImu.calibration_offset_ay = 0;
    myImu.calibration_offset_az = 0;

    myImu.calibration_offset_gx = 0;
    myImu.calibration_offset_gy = 0;
    myImu.calibration_offset_gz = 0;

    myDisplay.get_tft()->fillRect(0, 0, myDisplay.getDisplayWidth(), charHeight * 15, ILI9341_BLACK);    

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.print("Current");
    myDisplay.get_tft()->setCursor(x_pos,0);  
    myDisplay.println("Nouvelles");

    myDisplay.print("ax ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_X]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_ax);

    myDisplay.print("ay ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Y]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_ay);

    myDisplay.print("az ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_ACCELEROMETER][GPF_IMU_AXE_Z]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_az);

    myDisplay.print("gx ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_X]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gx);

    myDisplay.print("gy ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Y]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gy);

    myDisplay.print("gz ");
    myDisplay.print(myConfig_ptr->imuOffsets[GPF_IMU_SENSOR_GYROSCOPE][GPF_IMU_AXE_Z]);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.calibration_offset_gz);

    menu_display_button_Save("Save");
    
    myTouch.set_waitForUnTouch(true);
   }

  }        

}

void GPF::menu_gotoTestImu(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {
  uint16_t charHeight = 0;
  uint16_t charWidth  = 0;
  const uint16_t x_pos = 130;
  static elapsedMillis sincePrint = 251; //pour que le tout s'affiche tout de suite dès le premier appel de la fonction.
  const uint16_t sincePrint_delay = 250;

  if (firstTime) {
    myDisplay.clearScreen();
    menu_display_button_Exit();

    myDisplay.setTextSize(2);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.println(" IMU");

    myDisplay.println(" ax");  
    myDisplay.println(" ay");
    myDisplay.println(" az");
    myDisplay.println(" gx");
    myDisplay.println(" gy");
    myDisplay.println(" gz");
    myDisplay.println(" Pitch deg");
    myDisplay.println("       180");
    myDisplay.println("       360");
    myDisplay.println(" Roll deg");
    myDisplay.println("      180");
    myDisplay.println("      360");
    myDisplay.println(" Z deg");
    myDisplay.println(" Z/pitch");
    myDisplay.println(" Z/roll");
    myDisplay.println("Err count");
  }

  if (sincePrint > sincePrint_delay) {
    sincePrint = 0;
    myDisplay.setTextSize(2);
    myDisplay.get_tft()->measureChar('X',&charWidth,&charHeight); 

    myDisplay.get_tft()->setCursor(0,0);  
    myDisplay.println();

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.accX_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.accY_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.accZ_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.gyrX_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.gyrY_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.gyrZ_with_offsets);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_pitch_90_90);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.convert_0_360_to_180_180(myImu.output_pitch_0_360));

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_pitch_0_360);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_roll_90_90);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.convert_0_360_to_180_180(myImu.output_roll_0_360));

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_roll_0_360);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_z_90_90);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_z_0_360_pitch);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.output_z_0_360_roll);

    myDisplay.get_tft()->fillRect(x_pos, myDisplay.get_tft()->getCursorY(), myDisplay.getDisplayWidth()-x_pos, charHeight, ILI9341_BLACK);
    myDisplay.get_tft()->setCursor(x_pos,myDisplay.get_tft()->getCursorY());  
    myDisplay.println(myImu.errorCount);

  }

  boolean istouched = myTouch.ts_touched();

  if (istouched) { //Check si il a cliqué sur bouton "Sortir"
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   if (button_Exit.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    //menu_current = GPF_MENU_TEST_MENU;
    menu_current = gpf_menuItems[GPF_MENU_TEST_IMU].parent_id;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }
  }

}

void GPF::menu_gotoDisplayAllPIDs(bool firstTime, int not_used_param_2=0, int not_used_param_3=0) {
  if (firstTime) {
    myDisplay.clearScreen();
    menu_display_button_Exit();

    myDisplay.get_tft()->setCursor(0,0);
    myDisplay.println("PIDs ");

    for (size_t axe = 0; axe < GPF_AXE_ITEM_COUNT; axe++) {
      myDisplay.println();
      myDisplay.get_tft()->setCursor(0,myDisplay.get_tft()->getCursorY());
      myDisplay.print(gpf_axe_descriptions[axe]);  

     for (size_t pid_term = 0; pid_term < GPF_PID_TERM_ITEM_COUNT; pid_term++) {
      myDisplay.get_tft()->setCursor(75,myDisplay.get_tft()->getCursorY());
      myDisplay.print(gpf_pid_term_descriptions[pid_term]);
      myDisplay.print(" = ");
      myDisplay.println(myConfig_ptr->pids[axe][pid_term]);
     }

    }

  }

  boolean istouched = myTouch.ts_touched();

  if (istouched) { //Check si il a cliqué sur bouton "Sortir"
   TS_Point p = myTouch.ts_getPoint();

   int16_t pixelX = GPF_TOUCH::mapTouchXToPixelX(p.x);
   int16_t pixelY = GPF_TOUCH::mapTouchYToPixelY(p.y);

   if (button_Exit.contains(pixelX, pixelY)) {
    DEBUG_GPF_PRINT("On sort de la fonction ");
    DEBUG_GPF_PRINTLN(__func__);

    menu_current = gpf_menuItems[GPF_MENU_CONFIG_PID_DISPLAY_ALL_PIDS].parent_id;
    menu_pleaseRefresh = true;
    myTouch.set_waitForUnTouch(true);
   }
  }        

}

float GPF::getLoopFrequency() {
  float         retour                        = 0;
  unsigned long current_micros                = micros();
  unsigned long totalDuration                 = current_micros - firstLoopStartedAt;
  unsigned long totalDurationTakenIntoAccount = totalDuration - (totalDuration % GPF_MAIN_LOOP_RATE);

  retour = (float)(loopCount-1) * 1000000 / totalDurationTakenIntoAccount;
  
/*
  #ifdef DEBUG_GPF_ENABLED   
   DEBUG_GPF_PRINT(" current_micros=");
   DEBUG_GPF_PRINT(current_micros);
   DEBUG_GPF_PRINT(" totalDuration=");
   DEBUG_GPF_PRINT(totalDuration);
   DEBUG_GPF_PRINT(" totalDurationTakenIntoAccount=");
   DEBUG_GPF_PRINT(totalDurationTakenIntoAccount);
   DEBUG_GPF_PRINT(" firstLoopStartedAt=");
   DEBUG_GPF_PRINT(firstLoopStartedAt);
   DEBUG_GPF_PRINT(" loopCount=");
   DEBUG_GPF_PRINT(loopCount);
   DEBUG_GPF_PRINT(" retour=");
   DEBUG_GPF_PRINT(retour);   
   DEBUG_GPF_PRINTLN();   
  #endif
*/
  return retour;
}

unsigned long GPF::get_loopCount() {
  return loopCount;
}

void GPF::manageAlarms() {
  const uint16_t ALARM_TOGGLE_RURATION = 1000; //ms
  static elapsedMillis sinceChange = 0;
  static bool          sirenAlarmToneToggle  = false;
  //alarmVoltageLow = true; //test

  if (alarmVoltageLow | alarmImuProblem) {
    if (sinceChange > ALARM_TOGGLE_RURATION) {
      if (sirenAlarmToneToggle)  {
       gpf_util_beep(GPF_UTIL_BEEP_TONE_ALARM); 
      } else {       
       gpf_util_beep(GPF_UTIL_BEEP_TONE_INFO); 
      }
      sirenAlarmToneToggle = !sirenAlarmToneToggle;
      sinceChange = 0;
    }

  } else {
    noTone(GPF_MISC_PIN_BUZZER);     // Stop sound...
  }
}

char* GPF::get_dateTimeString(uint8_t format, bool addSpace) {
   int i;
   char tmpBuffer[6]       = "";

   strcpy(dateTimeString,"");

   itoa(year(), tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //4

   if (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_1) {
    strcat(dateTimeString,"-"); //1
   }
    
   i = month();
   if (i < 10) {
    strcat(dateTimeString,"0");
   }
   itoa(i, tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //2

   if (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_1) {
    strcat(dateTimeString,"-"); //1
   }

   i = day();
   if (i < 10) {
    strcat(dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //2

   strcat(dateTimeString, " "); //1

   i = hour();
   if (i < 10) {
    strcat(dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //2

   if (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_1) {
    strcat(dateTimeString,":");
   }

   i = minute();
   if (i < 10) {
    strcat(dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //2

   if (format == GPF_MISC_FORMAT_DATE_TIME_FRIENDLY_1) {
    strcat(dateTimeString,":");
   }

   i = second();
   if (i < 10) {
    strcat(dateTimeString,"0");    
   }
   itoa(i, tmpBuffer, 10);
   strcat(dateTimeString, tmpBuffer); //2

   if (format == GPF_MISC_FORMAT_DATE_TIME_LOGGING) {      
     
     strcat(dateTimeString,".");         

     i = millis() % 1000;
     if (i < 10) {
      strcat(dateTimeString,"0");    
     }
     if (i < 100) {
      strcat(dateTimeString,"0");    
     }
     itoa(i, tmpBuffer, 10);
     strcat(dateTimeString, tmpBuffer); //3
     
     if (addSpace) {
        strcat(dateTimeString," "); //1
     }
   }

   return dateTimeString;
}
