/**
 * @file gpf.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-18
 */

#ifndef GPF_H
#define GPF_H

#include "gpf_cons.h"
#include "gpf_mpu6050.h"
#include "gpf_telemetry.h"
#include "gpf_crsf.h"
#include <font_Arial.h> // from ILI9341_t3
#include "gpf_display.h"
#include "gpf_touch.h"
#include "gpf_sdcard.h"
#include "gpf_dshot.h"

class GPF {
    typedef void (GPF::*method_function)(bool, int, int);

    struct gpf_menu_item_struct {
         uint8_t   id;
         uint8_t   parent_id;
         char      caption[30];         
         method_function menuFunction;
         Adafruit_GFX_Button_2 button;
}   ;

    public:
        GPF();
        void initialize(gpf_config_struct *);
        void iAmStartingLoopNow(bool);
        void resetLoopStats();
        void waitUntilNextLoop();
        void toggMainBoardLed();
        void genDummyTelemetryData();
        unsigned long get_loopCount();
        void manageAlarms();
        
        bool get_IsStickInPositionEnabled(uint8_t stick);
        bool get_arm_IsArmed();
        bool set_arm_IsArmed(bool);
        bool get_black_box_IsEnabled();
        bool set_black_box_IsEnabled(bool);
        void displayArmed();        

        void displayAndProcessMenu();
        void debugDisplayLoopStats();
        void menu_display_button_Exit();
        void menu_display_button_Save(const char *caption); //Peut servir aussi comme bouton avec un caption différent
        void menu_display_button_BackSpace();
        void menu_display_button_Start();
        void menu_display_button_Reset();
        void menu_display_button_n(uint8_t buttonNumber, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

        void menu_gotoTestTouchScreen(bool, int, int);        
        void menu_gotoInfoStats(bool, int, int);
        void menu_gotoTestRC(bool, int, int);
        void menu_gotoTestImu(bool, int, int);
        void menu_gotoConfigurationChannels(bool, int, int);
        void menu_gotoConfigurationPID(bool, int, int);
        void menu_gotoCalibrationIMU(bool, int, int);
        void menu_gotoDisplayAllPIDs(bool, int, int);
        
        GPF_MPU6050  myImu;
        GPF_CRSF     myRc;
        GPF_DISPLAY  myDisplay;
        GPF_TOUCH    myTouch;
        GPF_SDCARD   mySdCard;
        GPF_DSHOT    myDshot;

        gpf_telemetry_info_s gpf_telemetry_info;
        int16_t  menu_current  = GPF_MENU_MAIN_MENU;
        //int16_t  menu_previous = -1; //Pour forcer un affichage du menu le premier coup
        bool     menu_pleaseRefresh = true;

        Adafruit_GFX_Button_2 button_Exit;
        Adafruit_GFX_Button_2 button_Save;
        Adafruit_GFX_Button_2 button_BackSpace;
        Adafruit_GFX_Button_2 button_Start;
        Adafruit_GFX_Button_2 button_Reset;
        Adafruit_GFX_Button_2 buttons[GPF_MISC_NUMBER_OF_BUTTONS];
        elapsedMillis black_box_sinceLog    = 0;

        char     dateTimeString[25] = ""; //Augmenter au besoin si on ajoute des choses dans la fonction ci-dessous.
        char*    get_dateTimeString(uint8_t format, bool addSpace);

        bool          alarmVoltageLow = false;  
        bool          alarmImuProblem = false;  

    private:
        elapsedMillis debug_sincePrint;        
        unsigned long loopCount             = 0;
        unsigned long loopStartedAt         = 0; //us
        unsigned long firstLoopStartedAt    = 0; //us
                 long loopFreeTime          = 0; //us
                 long loopBusyTime          = 0; //us
                 long loopBusyTimeMin       = 999999; //us
                 long loopBusyTimeMax       = 0; //us 
        unsigned long loopTimeOverFlowCount = 0; 

        float         loopFreeTimePercent = 0.0; 
        float         loopBusyTimePercent = 0.0; 

        gpf_config_struct *myConfig_ptr = NULL;
        bool          arm_isArmed = false;
        //bool          arm_pleaseDesarmFirst = true;
        bool          arm_allowArming     = false;
        bool          black_box_isEnabled = false;
        
        char   gpf_rc_stick_descriptions[GPF_RC_STICK_ITEM_COUNT][12] = {"Roll", "Pitch", "Throttle", "Yaw", "Arm", "Flight Mode", "Black Box"}; //Max 11 carac. sinon augmenter taille tableau
        char   gpf_axe_descriptions[GPF_AXE_ITEM_COUNT][6]            = {"Roll", "Pitch", "Yaw"}; //Max 5 carac. sinon augmenter taille tableau
        char   gpf_pid_term_descriptions[GPF_PID_TERM_ITEM_COUNT][2]  = {"P", "I", "D"}; //Max 1 carac. sinon augmenter taille tableau

        gpf_menu_item_struct gpf_menuItems[GPF_MENU_ITEM_COUNT] = 
           { 
              { GPF_MENU_MAIN_MENU, 0, ""}, 
                 { GPF_MENU_INFO_MENU, GPF_MENU_MAIN_MENU, "Info/Stats",NULL},
                    { GPF_MENU_INFO_STATS, GPF_MENU_INFO_MENU, "Statistiques",&GPF::menu_gotoInfoStats},
                 { GPF_MENU_TEST_MENU, GPF_MENU_MAIN_MENU, "Tests",NULL},
                    { GPF_MENU_TEST_RC, GPF_MENU_TEST_MENU, "Test RC",&GPF::menu_gotoTestRC},
                    { GPF_MENU_TEST_IMU, GPF_MENU_TEST_MENU, "Test IMU",&GPF::menu_gotoTestImu},
                    { GPF_MENU_TEST_TOUCH, GPF_MENU_TEST_MENU, "Test Touch Screen",&GPF::menu_gotoTestTouchScreen},
                 { GPF_MENU_CONFIG_MENU, GPF_MENU_MAIN_MENU, "Configuration",NULL},
                    { GPF_MENU_CONFIG_CHANNELS_MENU, GPF_MENU_CONFIG_MENU, "Channels",NULL},
                       { GPF_MENU_CONFIG_CHANNELS_ROLL, GPF_MENU_CONFIG_CHANNELS_MENU, "Roll",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_PITCH, GPF_MENU_CONFIG_CHANNELS_MENU, "Pitch",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_THROTTLE, GPF_MENU_CONFIG_CHANNELS_MENU, "Throttle",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_YAW, GPF_MENU_CONFIG_CHANNELS_MENU, "Yaw",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_ARM, GPF_MENU_CONFIG_CHANNELS_MENU, "Arm",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_FLIGHT_MODE, GPF_MENU_CONFIG_CHANNELS_MENU, "Flight Mode",&GPF::menu_gotoConfigurationChannels},
                       { GPF_MENU_CONFIG_CHANNELS_BLACK_BOX, GPF_MENU_CONFIG_CHANNELS_MENU, "Black Box",&GPF::menu_gotoConfigurationChannels},
                    { GPF_MENU_CONFIG_PID_MENU, GPF_MENU_CONFIG_MENU, "PIDs",NULL},
                       { GPF_MENU_CONFIG_PID_AXE_ROLL_MENU, GPF_MENU_CONFIG_PID_MENU, "PID Roll",NULL},
                          { GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_PROPORTIONAL, GPF_MENU_CONFIG_PID_AXE_ROLL_MENU, "Roll P Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_INTEGRAL, GPF_MENU_CONFIG_PID_AXE_ROLL_MENU, "Roll I Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_DERIVATIVE, GPF_MENU_CONFIG_PID_AXE_ROLL_MENU, "Roll D Gain",&GPF::menu_gotoConfigurationPID},
                       { GPF_MENU_CONFIG_PID_AXE_PITCH_MENU, GPF_MENU_CONFIG_PID_MENU, "PID Pitch",NULL},
                          { GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_PROPORTIONAL, GPF_MENU_CONFIG_PID_AXE_PITCH_MENU, "Pitch P Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_INTEGRAL, GPF_MENU_CONFIG_PID_AXE_PITCH_MENU, "Pitch I Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_DERIVATIVE, GPF_MENU_CONFIG_PID_AXE_PITCH_MENU, "Pitch D Gain",&GPF::menu_gotoConfigurationPID},   
                       { GPF_MENU_CONFIG_PID_AXE_YAW_MENU, GPF_MENU_CONFIG_PID_MENU, "PID Yaw",NULL},
                          { GPF_MENU_CONFIG_PID_AXE_YAW_TERM_PROPORTIONAL, GPF_MENU_CONFIG_PID_AXE_YAW_MENU, "Yaw P Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_YAW_TERM_INTEGRAL, GPF_MENU_CONFIG_PID_AXE_YAW_MENU, "Yaw I Gain",&GPF::menu_gotoConfigurationPID},
                          { GPF_MENU_CONFIG_PID_AXE_YAW_TERM_DERIVATIVE, GPF_MENU_CONFIG_PID_AXE_YAW_MENU, "Yaw D Gain",&GPF::menu_gotoConfigurationPID},   
                       { GPF_MENU_CONFIG_PID_DISPLAY_ALL_PIDS, GPF_MENU_CONFIG_PID_MENU, "Voir All PIDs",&GPF::menu_gotoDisplayAllPIDs},   
                    { GPF_MENU_CONFIG_CALIBRATION_MENU, GPF_MENU_CONFIG_MENU, "Calibration",NULL},
                       { GPF_MENU_CONFIG_CALIBRATION_IMU, GPF_MENU_CONFIG_CALIBRATION_MENU, "IMU Calibration",&GPF::menu_gotoCalibrationIMU}                             
           };

         void saveConfig();  
         float getLoopFrequency();
                
};

#endif
