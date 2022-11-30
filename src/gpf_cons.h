/**
 * @file gpf_cons.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-11-12 
 * 
 */

#ifndef GPF_CONS_H
#define GPF_CONS_H

#define GPF_MISC_PROG_CURRENT_VERSION    101
#define GPF_MISC_CONFIG_CURRENT_VERSION  7
#define GPF_MISC_NUMBER_OF_BUTTONS       20

typedef enum { // *** Ne pas changer l'ordre car sert aussi pour enregistrer config dans eeprom ***
    GPF_IMU_SENSOR_ACCELEROMETER,
    GPF_IMU_SENSOR_GYROSCOPE,

    GPF_IMU_SENSOR_ITEM_COUNT // MUST BE LAST
} gpf_imu_sensor_type_enum;

typedef enum { // *** Ne pas changer l'ordre car sert aussi pour enregistrer config dans eeprom ***
    GPF_AXE_ROLL,  // accY gyrX (accY positif = Avion tourne à droite)
    GPF_AXE_PITCH, // accX gyrY (accX positif = Avion monte)
    GPF_AXE_YAW,   // accZ gyrZ

    GPF_AXE_ITEM_COUNT // MUST BE LAST
} gpf_axe_type_enum;

#define GPF_IMU_AXE_X             GPF_AXE_ROLL   // *** Ne pas changer la valeur car sert aussi pour enregistrer config dans eeprom ***
#define GPF_IMU_AXE_Y             GPF_AXE_PITCH  // *** Ne pas changer la valeur car sert aussi pour enregistrer config dans eeprom ***
#define GPF_IMU_AXE_Z             GPF_AXE_YAW    // *** Ne pas changer la valeur car sert aussi pour enregistrer config dans eeprom ***

#define GPF_IMU_GYRO_AXE_X_ROLL   GPF_AXE_ROLL
#define GPF_IMU_GYRO_AXE_Y_PITCH  GPF_AXE_PITCH
#define GPF_IMU_GYRO_AXE_Z_YAW    GPF_AXE_YAW

#define GPF_IMU_ACC_AXE_Y_ROLL    GPF_AXE_ROLL
#define GPF_IMU_ACC_AXE_X_PITCH   GPF_AXE_PITCH
#define GPF_IMU_ACC_AXE_Z_YAW     GPF_AXE_YAW

typedef enum { // *** Ne pas changer l'ordre car sert aussi pour enregistrer config dans eeprom ***
    GPF_PID_TERM_PROPORTIONAL,    
    GPF_PID_TERM_INTEGRAL,    
    GPF_PID_TERM_DERIVATIVE,    

    GPF_PID_TERM_ITEM_COUNT // MUST BE LAST
} gpf_pid_term_type_enum;

typedef enum { // *** Ne pas changer l'ordre car sert aussi pour enregistrer config dans eeprom ***
    GPF_RC_STICK_ROLL,    
    GPF_RC_STICK_PITCH,
    GPF_RC_STICK_THROTTLE,
    GPF_RC_STICK_YAW,
    GPF_RC_STICK_ARM,

    GPF_RC_STICK_ITEM_COUNT // MUST BE LAST
} gpf_rc_stick_type_enum;

struct gpf_config_struct {
         uint16_t  version;
         uint8_t   channelMaps[GPF_RC_STICK_ITEM_COUNT];
         uint32_t  pids[GPF_AXE_ITEM_COUNT][GPF_PID_TERM_ITEM_COUNT];
         int16_t   imuOffsets[GPF_IMU_SENSOR_ITEM_COUNT][GPF_AXE_ITEM_COUNT];
};
        
typedef enum {
    GPF_MENU_MAIN_MENU,    
       GPF_MENU_TEST_MENU,
          GPF_MENU_TEST_RC,
          GPF_MENU_TEST_IMU,
          GPF_MENU_TEST_TOUCH,
       GPF_MENU_CONFIG_MENU,
          GPF_MENU_CONFIG_CHANNELS_MENU,
             GPF_MENU_CONFIG_CHANNELS_ROLL,
             GPF_MENU_CONFIG_CHANNELS_PITCH,
             GPF_MENU_CONFIG_CHANNELS_THROTTLE,
             GPF_MENU_CONFIG_CHANNELS_YAW,
             GPF_MENU_CONFIG_CHANNELS_ARM,
          GPF_MENU_CONFIG_PID_MENU,             
             GPF_MENU_CONFIG_PID_AXE_ROLL_MENU,
                GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_PROPORTIONAL,
                GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_INTEGRAL,
                GPF_MENU_CONFIG_PID_AXE_ROLL_TERM_DERIVATIVE,
             GPF_MENU_CONFIG_PID_AXE_PITCH_MENU,
                GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_PROPORTIONAL,
                GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_INTEGRAL,
                GPF_MENU_CONFIG_PID_AXE_PITCH_TERM_DERIVATIVE,
             GPF_MENU_CONFIG_PID_AXE_YAW_MENU,
                GPF_MENU_CONFIG_PID_AXE_YAW_TERM_PROPORTIONAL,
                GPF_MENU_CONFIG_PID_AXE_YAW_TERM_INTEGRAL,
                GPF_MENU_CONFIG_PID_AXE_YAW_TERM_DERIVATIVE,
             GPF_MENU_CONFIG_PID_DISPLAY_ALL_PIDS,   
          GPF_MENU_CONFIG_CALIBRATION_MENU, 
             GPF_MENU_CONFIG_CALIBRATION_IMU, 
           
       
    GPF_MENU_ITEM_COUNT // MUST BE LAST
} gpf_menu_type_enum;


#define GPF_RC_NUMBER_CHANNELS        16

#define GPF_RC_STICK_AILERON          GPF_RC_STICK_ROLL
#define GPF_RC_STICK_ELEVATOR         GPF_RC_STICK_PITCH           
#define GPF_RC_STICK_RUDDER           GPF_RC_STICK_YAW             

#define GPF_RC_STICK_ROLL_DEFAULT_CHANNEL      1
#define GPF_RC_STICK_PITCH_DEFAULT_CHANNEL     2
#define GPF_RC_STICK_THROTTLE_DEFAULT_CHANNEL  3
#define GPF_RC_STICK_YAW_DEFAULT_CHANNEL       4
#define GPF_RC_STICK_ARM_DEFAULT_CHANNEL       5

#define GPF_RC_CHANNEL_COMPARE_VALUE_PRECISION 150 //us
#define GPF_RC_CHANNEL_ARM_VALUE               1750 //us

#define GPF_MAIN_LOOP_RATE             500 //us (250=4000hz, 500=2000hz, 1000=1000hz, 2000=500hz, ...) //Maximum atteignable d'environ 6000hz avec un Teensy 4.1
#define GPF_MAIN_LED_TOGGLE_DURATION   500 //ms

#define GPF_SPI_MOSI            11 // Pin MOSI sur Teensy 4.1
#define GPF_SPI_SCLK            13 // Pin SCK sur Teensy 4.1
#define GPF_SPI_MISO            12 // Pin MISO sur Teensy 4.1

#define GPF_SPI_DISPLAY_DC       9 // SPI Data/Command Select pour écran TFT
#define GPF_SPI_DISPLAY_CS      10 // SPI Chip Select pour écran TFT
#define GPF_DISPLAY_RESET_PIN  255 // Display TFT reset pin: 255 = unused, connect to 3.3v.

#define GPF_SPI_TOUCH_DC         8 // SPI Data/Command Select pour touchcreen

#define GPF_TOUCH_UNTOUCH_TRESHOLD 50 //ms
#define GPF_TOUCH_TOUCH_TRESHOLD   50 //ms

#endif