/**
 * @file gpf_crsf.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-12
 * 
 * Voir fichier gpf_crsf.cpp pour plus d'informations.
 * 
 */

//
// Broadcast Frames:
// < Device address or Sync Byte> <Frame length> <Type><Payload> <CRC>
//
// Extended header frames:
//< Device address or Sync Byte> <Frame length> <Type><Destination Address> <Origin Address> <Payload> <CRC>
//

#ifndef GPF_CRSF_H
#define GPF_CRSF_H

#include "gpf_util.h"

#define GPF_CRSF_BAUDRATE	                              416666  // From TBS doc: Only non-inverted ( regular ) UART is supported in this configuration. The UART runs at 416666baud 8N1 at 3.0 to 3.3V level.
#define GPF_CRSF_MIN_DURATION_BETWEEN_FRAME	              1000    // (us) Sert à savoir lorsqu'un nouveau frame commence. Pour le moment il y a environ 6200us entre chaque frame donc si on ne recoit rien pendant 1000us on est pratiquement certain que c'est le "silence" entre deux frames. 1000us c'est beaucup mais bon, on semble avoir du temps en masse.
#define GPF_CRSF_DELAY_FOR_FAILSAFE       	              1000000 //500000  // (us) (500000us = 500ms = 1/2 seconde) Si on ne recoit rien pendant ce délai, on entre en failsafe.
#define GPF_CRSF_SYNC_BYTE                                0xC8    // Sync Byte
#define GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH	      64      // Each CRSF frame is not longer than 64 bytes (including the Sync and CRC bytes).
                                                                  // Broadcast Frames: <Device address or Sync Byte> <Frame length> <Type><Payload> <CRC>
                                                                  //                                       1 byte      1 byte      -- 2 to 62 bytes --
#define GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE   0       // Device address or Sync Byte
#define GPF_CRSF_BYTE_POSITION_FRAME_LENGTH               1       // Frame length
#define GPF_CRSF_BYTE_POSITION_FRAME_TYPE                 2       // Frame type
#define GPF_CRSF_BYTE_POSITION_PAYLOAD                    3       // Payload (Pour les frames de type Broadcast seulement) sinon ce serait 5 pour les frames de type "Extender Header Frame".

#define GPF_CRSF_DEVICE_ADDRESS_BROADCAST_ADDRESS         0x00 // Broadcast address
#define GPF_CRSF_DEVICE_ADDRESS_CLOUD_AKA_MQTT_BROKER     0x0E // Cloud (a.k.a. MQTT broker)
#define GPF_CRSF_DEVICE_ADDRESS_USB_DEVICE                0x10 // USB Device
#define GPF_CRSF_DEVICE_ADDRESS_BLUETOOTH_MODULE_WIFI     0x12 // Bluetooth Module/WIFI
#define GPF_CRSF_DEVICE_ADDRESS_VIDEO_RECEIVER            0x14 // Video Receiver
// 0x20-0x7F Dynamic address space for NAT                     // Dynamic address space for NAT 
#define GPF_CRSF_DEVICE_ADDRESS_OSD_TBS_CORE_PNP_PRO      0x80 // OSD / TBS CORE PNP PRO
#define GPF_CRSF_DEVICE_ADDRESS_RESERVED_0X8A             0x8A // Reserved
#define GPF_CRSF_DEVICE_ADDRESS_TX_INTEG_TEST_TERMINAL    0xB0 // Crossfire TX integration test terminal
#define GPF_CRSF_DEVICE_ADDRESS_RX_INTEG_TEST_TERMINAL    0xB2 // Crossfire RX integration test terminal
#define GPF_CRSF_DEVICE_ADDRESS_CURRENT_SENSOR            0xC0 // Current Sensor / PNP PRO digital current sensor
#define GPF_CRSF_DEVICE_ADDRESS_GPS                       0xC2 // GPS / PNP PRO GPS
#define GPF_CRSF_DEVICE_ADDRESS_TBS_BLACKBOX              0xC4 // TBS Blackbox
#define GPF_CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER         0xC8 // Flight controller
#define GPF_CRSF_DEVICE_ADDRESS_RESERVED_0XCA             0xCA // Reserved
#define GPF_CRSF_DEVICE_ADDRESS_RACE_TAG                  0xCC // Race tag
#define GPF_CRSF_DEVICE_ADDRESS_VTX                       0xCE // VTX
#define GPF_CRSF_DEVICE_ADDRESS_REMOTE_CONTROL            0xEA // Remote Control
#define GPF_CRSF_DEVICE_ADDRESS_RECEIVER_CROSSFIRE_RX     0xEC // R/C Receiver / Crossfire Rx
#define GPF_CRSF_DEVICE_ADDRESS_TRANSMITTER_CROSSFIRE_TX  0xEE // R/C Transmitter Module / Crossfire Tx
#define GPF_CRSF_DEVICE_ADDRESS_PPG_CONTROLLER            0xF0 // PPG Controller
#define GPF_CRSF_DEVICE_ADDRESS_PPG_MAIN_BOARD            0xF2 // PPG Mainboard

// From TBS Crossfire documentation
// Frame Types
// The following list shows the content of each frame type. The most common frames have an own type. Commands and others share one type.
// If a value is unknown, variable max value will be sent instead.
// 
// Broadcast frames
// Type range: 0x00 to 0x27. There are also some other broadcast frames (e.g. Mavlink!)
#define GPF_CRSF_FRAME_TYPE_GPS                           0x02 // GPS
#define GPF_CRSF_FRAME_TYPE_GPS_TIME_FRAME                0x03 // GPS Time Frame. This frame need to be synchronized with the ublox time pulse. The maximum offset of time is +/-10ms.
#define GPF_CRSF_FRAME_TYPE_GPS_EXTENDED_FRAME            0x06 // GPS Extended Frame
#define GPF_CRSF_FRAME_TYPE_VARIO_SENSOR                  0x07 // Vario sensor
#define GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR                0x08 // Battery sensor
#define GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR    0x09 // Barometric Altitude sensor
#define GPF_CRSF_FRAME_TYPE_HEARTBEAT                     0x0B // Heartbeat
                                                       // 0x0F // VTX Discontinued
#define GPF_CRSF_FRAME_TYPE_VIDEO_TRANSMITTER_TELEMETRY   0x10 // Video transmitter telemetry
#define GPF_CRSF_FRAME_TYPE_LINK_STATISTICS               0x14 // Link statistics
#define GPF_CRSF_FRAME_TYPE_RC_CHANNELS                   0x16 // RC channels packedPayload: 11bits Channel 1, 11bits Channel 2 … 11bits Channel 16. 16 channels packed into 22 bytes.
#define GPF_CRSF_FRAME_TYPE_RC_CHANNELS_SUBSET            0x17 // Subset RC channels packed
#define GPF_CRSF_FRAME_TYPE_RC_CHANNELS_UNUSUED           0x18 // RC channels packed 11bits - unused - (same as 0x16, but same conversion style as 0x17)
                                                       // 0x1B // Reserved Crossfire
#define GPF_CRSF_FRAME_TYPE_LINK_STATISTICS_RX            0x1C // Link statistics rx
#define GPF_CRSF_FRAME_TYPE_LINK_STATISTICS_TX            0x1D // Link statistics tx
#define GPF_CRSF_FRAME_TYPE_ATTITUDE                      0x1E // Attitude
#define GPF_CRSF_FRAME_TYPE_MACLINK_FC                    0x1F // MAVLink FC
#define GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED       0x21 // Flight mode text based
#define GPF_CRSF_FRAME_TYPE_PRODUCTION                    0x27 // Production

// Extended Header Frames, range: 0x28 to 0x96 
#define GPF_CRSF_FRAME_TYPE_PARAM_DEVICE_PING             0x28 // Trouvé dans source Ardupilot (pas dans la doc recue de TBS)
#define GPF_CRSF_FRAME_TYPE_PARAM_DEVICE_INFO             0x29 // Trouvé dans source Ardupilot (pas dans la doc recue de TBS) 
#define GPF_CRSF_FRAME_TYPE_PARAMETER_SETTINGS_ENTRY      0x2B // Trouvé dans source Ardupilot (pas dans la doc recue de TBS)
#define GPF_CRSF_FRAME_TYPE_PARAMETER_READ                0x2C // Trouvé dans source Ardupilot (pas dans la doc recue de TBS)
#define GPF_CRSF_FRAME_TYPE_PARAMETER_WRITE               0x2D // Trouvé dans source Ardupilot (pas dans la doc recue de TBS)
#define GPF_CRSF_FRAME_TYPE_COMMAND                       0x32 // Trouvé dans source Ardupilot (pas dans la doc recue de TBS)

                                                       // 0x78 - 0x79 // KISS FC Reserved range
                                                       // 0x7A - 0x7F // Betaflight MSP
#define GPF_CRSF_FRAME_TYPE_MSP_REQUEST                   0x7A // MSP_Request
#define GPF_CRSF_FRAME_TYPE_MSP_RESPONSE                  0x7B // MSP_Response
#define GPF_CRSF_FRAME_TYPE_AP_CUSTOM_TELEM_LEGACY        0x7F // Trouvé dans source Ardupilot (pas dans la doc recue de TBS) // as suggested by Remo Masina for fw < 4.06
#define GPF_CRSF_FRAME_TYPE_ARDUPILOT                     0x80 // Ardupilot Reserved range
#define GPF_CRSF_FRAME_TYPE_CRSF_MAVLINK_ENVELOPE         0xAA // CRSF MAVLink envelope
#define GPF_CRSF_FRAME_TYPE_CRSF_MAV_SYS_STATUS_SENSOR    0xAC // CRSF_MAV_SYS_STATUS_SENSOR

typedef enum {
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE,
    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED,

    ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT // MUST BE LAST
} telemetry_frame_type_enum;


class GPF_CRSF {

    struct libCrsf_link_statistics_s {
       uint8_t up_rssi_ant1;       // Uplink RSSI Ant. 1 ( dBm * -1 )
       uint8_t up_rssi_ant2;       // Uplink RSSI Ant. 2 ( dBm * -1 )
       uint8_t up_link_quality;    // Uplink Package success rate / Link quality ( % )
       int8_t  up_snr;             // Uplink SNR ( dB )
       uint8_t active_antenna;     // number of antenna
       uint8_t rf_profile;         // enum 4fps = 0 , 50fps, 150hz
       uint8_t up_rf_power;        // enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000
                                   // mW, 2000mW, 250mW, 50mW
       uint8_t down_rssi;          // Downlink RSSI ( dBm * -1 )
       uint8_t down_link_quality;  // Downlink Package success rate / Link quality ( % )
       int8_t  down_snr;           // Downlink SNR ( dB )
    };

    struct __attribute__ ((packed)) crsf_channels_t {
       unsigned int channel_1  : 11;
       unsigned int channel_2  : 11;
       unsigned int channel_3  : 11;
       unsigned int channel_4  : 11;
       unsigned int channel_5  : 11;
       unsigned int channel_6  : 11;
       unsigned int channel_7  : 11;
       unsigned int channel_8  : 11;
       unsigned int channel_9  : 11;
       unsigned int channel_10 : 11;
       unsigned int channel_11 : 11;
       unsigned int channel_12 : 11;
       unsigned int channel_13 : 11;
       unsigned int channel_14 : 11;
       unsigned int channel_15 : 11;
       unsigned int channel_16 : 11;
    };

    struct __attribute__ ((packed)) crsf_sensor_battery_s { //Big Endian
     uint16_t voltage; // Voltage ( mV * 100 )
     uint16_t current; // Current ( mA * 100 )
     //int24_t capacity_used; // uint24_t Capacity used ( mAh )
     uint8_t capacity_used[3];
     uint8_t remaining; // uint8_t Battery remaining ( percent )
    };

    struct crsf_heartbeat_s {
     uint16_t origin_add; // Origin Device address
    };

    struct crsf_sensor_vario_s { //Big Endian
     int16_t v_speed; // Vertical speed ( m/s * 100 ) //-32768 à 32767. //10=0.1m/s, 100=1m/s, 1234=12.3m/s, -30000=-299.5m/s
    };

    /*
    struct ParameterDeviceInfoFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t payload[58];   // largest possible frame is 60
    };
    */

    struct crsf_sensor_gps_s { //Big Endian
     int32_t latitude; // degree / 10`000`000
     int32_t longitude; // degree / 10`000`000
     uint16_t groundspeed; // km/h / 100
     uint16_t heading; // degree / 100 //Ca marche pas ???
     uint16_t altitude; // meter - 1000m offset 
     uint8_t satellites; // # of sats in view  
    };

    struct crsf_sensor_altitude_baro_s { //Big Endian
     uint16_t altitude_packed; // Altitude above start (calibration) point.
                               // value depends on MSB:
                               // MSB = 0: altitude is in decimeters - 10000dm offset
                               // MSB = 1: altitude is in meters. Without any offset
    };

    struct crsf_sensor_attitude_s { //WARNING: Angle values must be in -180° +180° range!
     int16_t pitch; // Pitch angle ( rad / 10000 )
     int16_t roll; // Roll angle ( rad / 10000 )
     int16_t yaw; // Yaw angle ( rad / 10000 )
    };
     
    struct crsf_sensor_flight_mode_text_based_s{ //WARNING: Angle values must be in -180° +180° range!
     char description[GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH]; //Flight mode ( Null-terminated string )
    }; 

    public:
        GPF_CRSF();
        void initialize(HardwareSerial *);
        void setupTelemetry(gpf_telemetry_info_s *);
        void readRx();
        uint16_t      getPwmChannelValue(uint8_t);
        void          setPwmChannelValue(uint8_t channelNumber, uint16_t channelValue);
        void          forcePwmChannelYawRollPitchToNeutral(uint8_t yawChannelNumber, uint8_t rollChannelNumber, uint8_t pitchChannelNumber);
        unsigned int  getPwmChannelPos(uint8_t);
        bool          get_isInFailSafe();
        unsigned long getFailSafeDuration();
        

        libCrsf_link_statistics_s link_statistics;
        gpf_telemetry_info_s *gpf_telemetry_info_ptr = NULL;
        
    private:
        void    reset_bytesReceivedCount_and_buffer();
        void    CRC8_createLut(uint8_t);
        uint8_t CRC8_calculate(uint8_t *, int);
        bool    parseFrame();
        
        
        void    refreshTelemetry();
        void    sendTelemetryToTx();
        bool    sendTelemetryItemToTx(uint8_t);
        

        HardwareSerial *serialPort; //Print -> Stream -> HardwareSerial => [Serial]
        uint8_t         frame_length                    = 0;
        uint8_t         frame_type                      = 0;
        bool            new_frame_is_about_to_start     = false;
        bool            isInFailSafe                    = false;
        elapsedMicros   duration_between_frame          = 0;
        unsigned long   debug_duration_between_frame_longest  = 0;
        uint8_t         bytesReceivedCount              = 0;
        unsigned long   bytesReceivedTotal              = 0;
        uint8_t         bytesReceivedBuffer[GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH];
        uint8_t         bytesSendBuffer[GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH];
        elapsedMillis   debug_sincePrint;
        uint8_t         crc8_lut [256];        
        crsf_channels_t crsf_channels;
        uint16_t        pwm_channels[GPF_RC_NUMBER_CHANNELS + 1] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //L'indice 0 ne servira pas. C'est parceque je désique que l'indice corresponde au numéro de canal réel pour éviter d'éventuelles confusion.
        
        crsf_heartbeat_s                     crsf_heartbeat;
        crsf_sensor_battery_s                crsf_sensor_battery;
        crsf_sensor_vario_s                  crsf_sensor_vario;
        crsf_sensor_gps_s                    crsf_sensor_gps;
        crsf_sensor_altitude_baro_s          crsf_sensor_altitude_baro;
        crsf_sensor_attitude_s               crsf_sensor_attitude;
        crsf_sensor_flight_mode_text_based_s crsf_sensor_flight_mode_text_based;

        bool            telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT];
        uint16_t        telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT];
        elapsedMillis   telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT];

        #define DEBUG_PACKET_RECEIVED_DEVICE_ADDRESS_LIST_ITEM_COUNT  21 //Mettre le nombre d'item de l'array debug_packet_received_device_address_list ci-dessous.
        unsigned long debug_packet_received_device_address_count[DEBUG_PACKET_RECEIVED_DEVICE_ADDRESS_LIST_ITEM_COUNT]; //Sert pour dubug seulement
        const uint8_t debug_packet_received_device_address_list[DEBUG_PACKET_RECEIVED_DEVICE_ADDRESS_LIST_ITEM_COUNT] = {     //Sert pour dubug seulement
            GPF_CRSF_DEVICE_ADDRESS_BROADCAST_ADDRESS         ,
            GPF_CRSF_DEVICE_ADDRESS_CLOUD_AKA_MQTT_BROKER     ,
            GPF_CRSF_DEVICE_ADDRESS_USB_DEVICE                ,
            GPF_CRSF_DEVICE_ADDRESS_BLUETOOTH_MODULE_WIFI     ,
            GPF_CRSF_DEVICE_ADDRESS_VIDEO_RECEIVER            ,
            GPF_CRSF_DEVICE_ADDRESS_OSD_TBS_CORE_PNP_PRO      ,
            GPF_CRSF_DEVICE_ADDRESS_RESERVED_0X8A             ,
            GPF_CRSF_DEVICE_ADDRESS_TX_INTEG_TEST_TERMINAL    ,
            GPF_CRSF_DEVICE_ADDRESS_RX_INTEG_TEST_TERMINAL    ,
            GPF_CRSF_DEVICE_ADDRESS_CURRENT_SENSOR            ,
            GPF_CRSF_DEVICE_ADDRESS_GPS                       ,
            GPF_CRSF_DEVICE_ADDRESS_TBS_BLACKBOX              ,
            GPF_CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER         ,
            GPF_CRSF_DEVICE_ADDRESS_RESERVED_0XCA             ,
            GPF_CRSF_DEVICE_ADDRESS_RACE_TAG                  ,
            GPF_CRSF_DEVICE_ADDRESS_VTX                       ,
            GPF_CRSF_DEVICE_ADDRESS_REMOTE_CONTROL            ,
            GPF_CRSF_DEVICE_ADDRESS_RECEIVER_CROSSFIRE_RX     ,
            GPF_CRSF_DEVICE_ADDRESS_TRANSMITTER_CROSSFIRE_TX  ,
            GPF_CRSF_DEVICE_ADDRESS_PPG_CONTROLLER            ,
            GPF_CRSF_DEVICE_ADDRESS_PPG_MAIN_BOARD                                                                                         
        }; 

        #define DEBUG_PACKET_RECEIVED_FRAME_TYPE_LIST_ITEM_COUNT  2 //Mettre le nombre d'item de l'array debug_packet_received_frame_type_list ci-dessous.
        unsigned long debug_packet_received_frame_type_count[DEBUG_PACKET_RECEIVED_FRAME_TYPE_LIST_ITEM_COUNT];     //Sert pour dubug seulement 
        const uint8_t debug_packet_received_frame_type_list[DEBUG_PACKET_RECEIVED_FRAME_TYPE_LIST_ITEM_COUNT] = {   //Sert pour dubug seulement
            //GPF_CRSF_FRAME_TYPE_HEARTBEAT,
            GPF_CRSF_FRAME_TYPE_LINK_STATISTICS,
            GPF_CRSF_FRAME_TYPE_RC_CHANNELS,
            //GPF_CRSF_FRAME_TYPE_RC_CHANNELS_SUBSET,
            //GPF_CRSF_FRAME_TYPE_RC_CHANNELS_UNUSUED,
            //GPF_CRSF_FRAME_TYPE_LINK_STATISTICS_RX,
            //GPF_CRSF_FRAME_TYPE_LINK_STATISTICS_TX,
            //GPF_CRSF_FRAME_TYPE_MACLINK_FC,
            //GPF_CRSF_FRAME_TYPE_PRODUCTION,
            //GPF_CRSF_FRAME_TYPE_MSP_REQUEST,
            //GPF_CRSF_FRAME_TYPE_MSP_RESPONSE,
            //GPF_CRSF_FRAME_TYPE_CRSF_MAVLINK_ENVELOPE,
            //GPF_CRSF_FRAME_TYPE_CRSF_MAV_SYS_STATUS_SENSOR,
        }; 

        #define DEBUG_PACKET_SENT_FRAME_TYPE_LIST_ITEM_COUNT  9 //Mettre le nombre d'item de l'array debug_packet_sent_frame_type_list ci-dessous.
        unsigned long  debug_packet_sent_frame_type_count[DEBUG_PACKET_SENT_FRAME_TYPE_LIST_ITEM_COUNT]; //Sert pour dubug seulement 
        const uint8_t  debug_packet_sent_frame_type_list[DEBUG_PACKET_SENT_FRAME_TYPE_LIST_ITEM_COUNT] = { //Sert pour dubug seulement
            GPF_CRSF_FRAME_TYPE_GPS,
            GPF_CRSF_FRAME_TYPE_GPS_TIME_FRAME,
            GPF_CRSF_FRAME_TYPE_GPS_EXTENDED_FRAME,
            GPF_CRSF_FRAME_TYPE_VARIO_SENSOR,
            GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR,
            GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR,
            GPF_CRSF_FRAME_TYPE_HEARTBEAT,            
            GPF_CRSF_FRAME_TYPE_ATTITUDE,            
            GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED,            
        }; 
        
};

#endif