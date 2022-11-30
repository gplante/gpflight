/**
 * @file gpf_crsf.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-12
 * 
 * Class pour communiquer avec un receiver TBS Nano à l'aide du protocole CRSF (Crossfire) via un UART d'un Teensy 4.1.
 * Le protocole CRSF a été développé par Team Black Sheep (https://www.team-blacksheep.com)
 * Je n'ai trouvé aucune librairie "simple" ou "toute faite" sur Internet j'ai donc décidé de coder le tout moi même 
 * à l'aide de la documentation gentiment fournie par par Raphael Pirker de Team Black Sheep.
 *
 * Je me suis quand même inspiré du code de certains projets trouvés sur GitHub. En voici une liste non exhaustive:  
 * https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_RCProtocol
 * https://github.com/iNavFlight/inav/tree/master/src/main/rx
 * https://github.com/betaflight/betaflight/tree/master/src/main/rx
 * https://github.com/dncoder/crsf-link-tester
 * https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/rc
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_crsf.h"
#include "gpf_debug.h"
#include "gpf_util.h"

GPF_CRSF::GPF_CRSF() {
    //Rien de spécial dans le constructeur pour le moment    

    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS]                        = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR]               = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR]             = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR] = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT]                  = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE]                   = true;
    telemetryEnabled[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED]    = true;

    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS]                          = 333;  // 3hz (Envoi environ au 333ms)
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR]                 = 333;  // 3hz (Envoi environ au 333ms)
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR]               = 500;  // 2hz (Envoi environ au 500ms)
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR]   = 333;  // 3hz (Envoi environ au 333ms)
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT]                    = 100;  // 10hz (Envoi environ au 100ms) //Pas certain que ce soit nécessaire???
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE]                     = 125;  // 8hz (Envoi environ au 125ms)
    telemetryRates[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED]      = 500;  // 2hz (Envoi environ au 500ms)

    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS]                         = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR]                = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR]              = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR]  = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT]                   = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE]                    = 0;
    telemetryTimers[ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED]     = 0;

    crsf_heartbeat.origin_add = GPF_CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER;

    crsf_sensor_battery.voltage = 0;
    crsf_sensor_battery.current = 0;
    crsf_sensor_battery.capacity_used[0] = 0;
    crsf_sensor_battery.capacity_used[1] = 0;
    crsf_sensor_battery.capacity_used[2] = 0;
    crsf_sensor_battery.remaining = 0;

    crsf_sensor_vario.v_speed = 0;

    crsf_sensor_gps.latitude    = 0;
    crsf_sensor_gps.longitude   = 0;
    crsf_sensor_gps.groundspeed = 0;
    crsf_sensor_gps.heading     = 0;
    crsf_sensor_gps.altitude    = 0;
    crsf_sensor_gps.satellites  = 0;

    crsf_sensor_altitude_baro.altitude_packed = 0;
}

void GPF_CRSF::setupTelemetry(gpf_telemetry_info_s *ptr) {
    //Fait pointer la struct des données de télémétrie directement sur une variable de l'objet GPF_CRSF pour que
    //celui-ci soit capable de se mettre à jour tout seul (voir refreshTelemetry())
    gpf_telemetry_info_ptr = ptr;
}

void GPF_CRSF::initialize(HardwareSerial *p_serialPort) {    
    serialPort = p_serialPort;
    serialPort->begin(GPF_CRSF_BAUDRATE, SERIAL_8N1);
    DEBUG_GPF_CRSF_PRINT(F("CRSF:Ouvre port serie..."));
    while (!serialPort) { };
    DEBUG_GPF_CRSF_PRINTLN(F("Ok"));
    CRC8_createLut(0xD5);
}

void GPF_CRSF::readRx() {
   //bool telemetryItemSent = false; //On n'envoi qu'une télémétrie entre deux frames. Pour le moment, c'est correcte, on a pas grand chose à envoyer.

   debug_duration_between_frame_longest = max(debug_duration_between_frame_longest, (unsigned long)duration_between_frame);

   if (duration_between_frame > GPF_CRSF_MIN_DURATION_BETWEEN_FRAME) {
    new_frame_is_about_to_start = true;
    reset_bytesReceivedCount_and_buffer();
   }

   if (new_frame_is_about_to_start) { //On en profite pour envoyer la télémétrie entre deux frames recus mais je pense qu'on pourrait l'envoyer n'importe quand.
    
    sendTelemetryToTx();


    
   }

   isInFailSafe =(duration_between_frame > GPF_CRSF_DELAY_FOR_FAILSAFE);

   if (bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH) { //Protection (On ne devrait jamais entrer ici normalement)
     // Ici, si on a 64 bytes dans le bytesReceivedCount c'est que le frame n'a pas été traité. 
     // Ce n'est pas normal et ca ne devrait jamais arriver alors on réinitialise et on oublie ce frame. 
     // De toute facon, il ne faut pas défoncer la taille du buffer (bytesReceivedBuffer) dont la taille est GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH.
     reset_bytesReceivedCount_and_buffer();
     DEBUG_GPF_CRSF_PRINTLN(F("Oups, bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH ???"));
   }
      
   while (serialPort->available() > 0) {
    duration_between_frame = 0; //Lorsqu'on recoit quelque chose, on reset la durée
    bytesReceivedBuffer[bytesReceivedCount++] = serialPort->read();
    bytesReceivedTotal++;

    if (bytesReceivedCount >= 3) { //Ca prend les 3 premiers octets pour voir le device address, la longeur totale du frame et le  type.
     if (bytesReceivedCount == 3) { //Lorsqu'on a exactement 3 bytes de recu, on peut connaitre la longeur totale du frame ainsi que son type
      frame_length = bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_LENGTH];
      frame_type   = bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE];
            
      #ifdef DEBUG_GPF_CRSF_ENABLED 
        bool debugKnownFrameTypeReceived = false;
        for (int i = 0; i < DEBUG_PACKET_RECEIVED_FRAME_TYPE_LIST_ITEM_COUNT; i++) { //On garde des stats pour le mode DEBUG
         if (frame_type == debug_packet_received_frame_type_list[i]) {  
          debug_packet_received_frame_type_count[i]++;
          debugKnownFrameTypeReceived = true;
         }
        }

        if (!debugKnownFrameTypeReceived) {
          DEBUG_GPF_CRSF_PRINT(F("*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X*X* Oups, frame HORS LIST frame_type ="));
          DEBUG_GPF_CRSF_PRINTLN(frame_type,HEX);
        }
      #endif      
     }
     
     if (bytesReceivedCount == (frame_length + 2)) { //+2 sont les deux premiers octets <Device address or Sync Byte> et <Frame length>
      // Si on a recu le frame au complet, on peut le traiter
      parseFrame();
     }     
    }

    
   
    if (new_frame_is_about_to_start) {
     #ifdef DEBUG_GPF_CRSF_ENABLED 
      for (int i = 0; i < DEBUG_PACKET_RECEIVED_DEVICE_ADDRESS_LIST_ITEM_COUNT; i++) { //On garde des stats pour le mode DEBUG
       if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == debug_packet_received_device_address_list[i]) { 
        debug_packet_received_device_address_count[i]++;
       }
      }
     #endif
     new_frame_is_about_to_start = false; //Une fois le premier byte du frame recu, on peut le remettre à false
     //telemetryItemSent = false; //Un autre frame de télémétrie sera envoyé lorsque le frame en cours de réception sera recu en entier.
    }
   }

   #ifdef DEBUG_GPF_CRSF_ENABLED
    if (debug_sincePrint > DEBUG_GPF_CRSF_DELAY) {

      DEBUG_GPF_CRSF_PRINTLN();
      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("bytesReceivedTotal="));
      DEBUG_GPF_CRSF_PRINT(bytesReceivedTotal);
      DEBUG_GPF_CRSF_PRINT(F(" drbug_duration_between_frame_longest="));
      DEBUG_GPF_CRSF_PRINT(debug_duration_between_frame_longest);
      DEBUG_GPF_CRSF_PRINT(F(" get_isInFailSafe()="));
      DEBUG_GPF_CRSF_PRINT(get_isInFailSafe());
      DEBUG_GPF_CRSF_PRINTLN();

      debug_duration_between_frame_longest = 0;

      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("Frame count per device address: "));
      for (int i = 0; i < DEBUG_PACKET_RECEIVED_DEVICE_ADDRESS_LIST_ITEM_COUNT; i++) {
        if (debug_packet_received_device_address_count[i] > 0) {
         DEBUG_GPF_CRSF_PRINT(debug_packet_received_device_address_list[i], HEX); 
         DEBUG_GPF_CRSF_PRINT(F("="));
         DEBUG_GPF_CRSF_PRINT(debug_packet_received_device_address_count[i]);
         DEBUG_GPF_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("Frame count per frame type (received): "));
      for (int i = 0; i < DEBUG_PACKET_RECEIVED_FRAME_TYPE_LIST_ITEM_COUNT; i++) {
        if (debug_packet_received_frame_type_count[i] > 0) {
         DEBUG_GPF_CRSF_PRINT(debug_packet_received_frame_type_list[i], HEX); 
         DEBUG_GPF_CRSF_PRINT(F("="));
         DEBUG_GPF_CRSF_PRINT(debug_packet_received_frame_type_count[i]);
         DEBUG_GPF_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("Frame count per frame type (sent, telemetry): "));
      for (int i = 0; i < DEBUG_PACKET_SENT_FRAME_TYPE_LIST_ITEM_COUNT; i++) {
        if (debug_packet_sent_frame_type_count[i] > 0) {
         DEBUG_GPF_CRSF_PRINT(debug_packet_sent_frame_type_list[i], HEX); 
         DEBUG_GPF_CRSF_PRINT(F("="));
         DEBUG_GPF_CRSF_PRINT(debug_packet_sent_frame_type_count[i]);
         DEBUG_GPF_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF:Channels ch1="));
      DEBUG_GPF_CRSF_PRINT(crsf_channels.channel_1);
      DEBUG_GPF_CRSF_PRINT(F(" "));
      DEBUG_GPF_CRSF_PRINT(pwm_channels.channel_1);
      //DEBUG_GPF_CRSF_PRINT(getPwmChannelValue(1));

      DEBUG_GPF_CRSF_PRINT(F(" ch2="));
      DEBUG_GPF_CRSF_PRINT(crsf_channels.channel_2);
      DEBUG_GPF_CRSF_PRINT(F(" "));
      DEBUG_GPF_CRSF_PRINT(pwm_channels.channel_2);

      DEBUG_GPF_CRSF_PRINT(F(" ch3="));
      DEBUG_GPF_CRSF_PRINT(crsf_channels.channel_3);
      DEBUG_GPF_CRSF_PRINT(F(" "));
      DEBUG_GPF_CRSF_PRINT(pwm_channels.channel_3);

      DEBUG_GPF_CRSF_PRINT(F(" ch4="));
      DEBUG_GPF_CRSF_PRINT(crsf_channels.channel_4);
      DEBUG_GPF_CRSF_PRINT(F(" "));
      DEBUG_GPF_CRSF_PRINT(pwm_channels.channel_4);

      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF:link_statistics active_antenna="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.active_antenna);

      DEBUG_GPF_CRSF_PRINT(F(" up_rssi_ant1="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.up_rssi_ant1);
      DEBUG_GPF_CRSF_PRINT(F(" up_rssi_ant2="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.up_rssi_ant2);
      DEBUG_GPF_CRSF_PRINT(F(" down_rssi="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.down_rssi);
      
      DEBUG_GPF_CRSF_PRINT(F(" up_snr="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.up_snr);

      DEBUG_GPF_CRSF_PRINT(F(" rf_profile="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.rf_profile);

      DEBUG_GPF_CRSF_PRINT(F(" up_rf_power="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.up_rf_power);
      DEBUG_GPF_CRSF_PRINT(F(" down_link_quality="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.down_link_quality);
      DEBUG_GPF_CRSF_PRINT(F(" up_link_quality="));
      DEBUG_GPF_CRSF_PRINT(link_statistics.up_link_quality);
      DEBUG_GPF_CRSF_PRINTLN();

      debug_sincePrint = 0;
    }
   #endif
   
}

void GPF_CRSF::reset_bytesReceivedCount_and_buffer() {
  bytesReceivedCount = 0;
  memset(bytesReceivedBuffer, 0, sizeof bytesReceivedBuffer);
}

bool GPF_CRSF::parseFrame() {  
  #define CHANNEL_SCALE(x) ((int32_t(x) * 5U) / 8U + 880U)

  bool retour = false;
  uint8_t frame_crc =bytesReceivedBuffer[frame_length + 1];

  if (CRC8_calculate(&bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE], frame_length - 1) == frame_crc) { //Si le crc est valide
    if ( (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == GPF_CRSF_DEVICE_ADDRESS_BROADCAST_ADDRESS) ||
         (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == GPF_CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER) ) {

    if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE] == GPF_CRSF_FRAME_TYPE_LINK_STATISTICS) {
      memcpy(&link_statistics, &bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], sizeof(libCrsf_link_statistics_s));
    }

    if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE] == GPF_CRSF_FRAME_TYPE_RC_CHANNELS) {
      memcpy(&crsf_channels, &bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], sizeof(crsf_channels_t));

      //CRSF a son propre format pour la valeur de chaque canal alors on converti en format pwm qui est plus universel et plus facile à travailler.
      pwm_channels.channel_1  = CHANNEL_SCALE(crsf_channels.channel_1);
      pwm_channels.channel_2  = CHANNEL_SCALE(crsf_channels.channel_2);
      pwm_channels.channel_3  = CHANNEL_SCALE(crsf_channels.channel_3);
      pwm_channels.channel_4  = CHANNEL_SCALE(crsf_channels.channel_4);
      pwm_channels.channel_5  = CHANNEL_SCALE(crsf_channels.channel_5);
      pwm_channels.channel_6  = CHANNEL_SCALE(crsf_channels.channel_6);
      pwm_channels.channel_7  = CHANNEL_SCALE(crsf_channels.channel_7);
      pwm_channels.channel_8  = CHANNEL_SCALE(crsf_channels.channel_8);
      pwm_channels.channel_9  = CHANNEL_SCALE(crsf_channels.channel_9);
      pwm_channels.channel_10 = CHANNEL_SCALE(crsf_channels.channel_10);
      pwm_channels.channel_11 = CHANNEL_SCALE(crsf_channels.channel_11);
      pwm_channels.channel_12 = CHANNEL_SCALE(crsf_channels.channel_12);
      pwm_channels.channel_13 = CHANNEL_SCALE(crsf_channels.channel_13);
      pwm_channels.channel_14 = CHANNEL_SCALE(crsf_channels.channel_14);
      pwm_channels.channel_15 = CHANNEL_SCALE(crsf_channels.channel_15);
      pwm_channels.channel_16 = CHANNEL_SCALE(crsf_channels.channel_16);
    }

   }

   retour = true;
  } else {
    DEBUG_GPF_CRSF_PRINTLN(F("CRSF: Oups, crc du frame non valide!"));
  }
  
  reset_bytesReceivedCount_and_buffer(); //Une fois le frame parsed, on réinitialise le buffer

  return retour;
}

bool GPF_CRSF::get_isInFailSafe() {
  return isInFailSafe;
}

uint8_t GPF_CRSF::CRC8_calculate(uint8_t * data, int len) {
	uint8_t crc = 0;

	for (int i=0; i < len; i++) {
		crc = crc8_lut[crc ^ *data++];
	}
	return crc;
}

void GPF_CRSF::CRC8_createLut(uint8_t poly) {
 for (int i=0; i < 256; i++) {
		uint8_t crc = i;
		for (int j=0; j < 8; j++) {
			crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
		}
		crc8_lut[i] = crc;
	}
}

unsigned int GPF_CRSF::getPwmChannelValue(uint8_t channelNumber) {
  unsigned int retour = 0;

  switch (channelNumber) {
  case 1:
    retour = pwm_channels.channel_1;
    break;
  case 2:
    retour = pwm_channels.channel_2;
    break;
  case 3:
    retour = pwm_channels.channel_3;
    break;
  case 4:
    retour = pwm_channels.channel_4;
    break;
  case 5:
    retour = pwm_channels.channel_5;
    break;
  case 6:
    retour = pwm_channels.channel_6;
    break;
  case 7:
    retour = pwm_channels.channel_7;
    break;
  case 8:
    retour = pwm_channels.channel_8;
    break;
  case 9:
    retour = pwm_channels.channel_9;
    break;
  case 10:
    retour = pwm_channels.channel_10;
    break;
  case 11:
    retour = pwm_channels.channel_11;
    break;
  case 12:
    retour = pwm_channels.channel_12;
    break;
  case 13:
    retour = pwm_channels.channel_13;
    break;
  case 14:
    retour = pwm_channels.channel_14;
    break;
  case 15:
    retour = pwm_channels.channel_15;
    break;
  case 16:
    retour = pwm_channels.channel_16;
    break;                              
  
  default:
    break;
  }

  return retour;
}

unsigned int GPF_CRSF::getPwmChannelPos(uint8_t channelNumber) {
  return getPwmChannelValue(channelNumber);
}

void GPF_CRSF::refreshTelemetry() {
 
 //gpf_telemetry_info_ptr->battery_remaining_percent++;
 //if (gpf_telemetry_info_ptr->battery_remaining_percent > 99) {
 //  gpf_telemetry_info_ptr->battery_remaining_percent = 1;
 //}

 if (gpf_telemetry_info_ptr != NULL) {
  crsf_sensor_battery.voltage = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->battery_voltage); //Doit convertir en format Big Endian pour CRSF
  crsf_sensor_battery.current = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->battery_current); //Doit convertir en format Big Endian pour CRSF

  crsf_sensor_battery.capacity_used[0] = (gpf_telemetry_info_ptr->battery_capacity_used & 0xFF0000) >> 16;
  crsf_sensor_battery.capacity_used[1] = (gpf_telemetry_info_ptr->battery_capacity_used & 0xFF00) >> 8;
  crsf_sensor_battery.capacity_used[2] = (gpf_telemetry_info_ptr->battery_capacity_used & 0xFF);
  crsf_sensor_battery.remaining        = gpf_telemetry_info_ptr->battery_remaining_percent; 

  crsf_sensor_vario.v_speed = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->vario_vertival_speed);

  crsf_sensor_gps.latitude    = gpf_util_shiftBitsToBigEndian_32(gpf_telemetry_info_ptr->gps_latitude); 
  crsf_sensor_gps.longitude   = gpf_util_shiftBitsToBigEndian_32(gpf_telemetry_info_ptr->gps_longitude); 
  crsf_sensor_gps.groundspeed = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->gps_groundspeed); 
  //gpf_telemetry_info_ptr->gps_heading = 16500;
  crsf_sensor_gps.heading     = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->gps_heading); //Doit convertir en format Big Endian pour CRSF
  crsf_sensor_gps.altitude    = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->gps_altitude + 1000);
  crsf_sensor_gps.satellites  = gpf_telemetry_info_ptr->gps_satellites; 

  //                                              xxxxxxxxyyyyyyyy 
  //crsf_sensor_altitude_baro.altitude_packed = 0b0000101011000000; 
  //crsf_sensor_altitude_baro.altitude_packed = gpf_util_shiftBitsToBigEndian_16((uint16_t)((100+5)/10) | 0x8000);
  //crsf_sensor_altitude_baro.altitude_packed = 0x8000 | gpf_util_shiftBitsToBigEndian_16((uint16_t)((100+5)/10));
  //crsf_sensor_altitude_baro.altitude_packed = 0x8000 | gpf_util_shiftBitsToBigEndian_16((uint16_t)100);
  //https://github.com/betaflight/betaflight/issues/11069
  
  crsf_sensor_altitude_baro.altitude_packed = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->baro_altitude + 10000);

  crsf_sensor_attitude.pitch = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->attitude_pitch);
  crsf_sensor_attitude.roll  = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->attitude_roll);
  crsf_sensor_attitude.yaw   = gpf_util_shiftBitsToBigEndian_16(gpf_telemetry_info_ptr->attitude_yaw);

  strncpy(crsf_sensor_flight_mode_text_based.description, gpf_telemetry_info_ptr->flight_mode_description, GPF_UTIL_FLIGHT_MODE_DESCRIPTION_MAX_LENGTH);

 }
}

void GPF_CRSF::sendTelemetryToTx() {

  static uint8_t telemetryItemIndex = ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT;
         uint8_t cpt = 0;

  while (cpt < ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT) {
   telemetryItemIndex++;
   if (telemetryItemIndex >= ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ITEM_COUNT ) {
      telemetryItemIndex = 0;
   }

   if (telemetryEnabled[telemetryItemIndex]) {
     if (telemetryTimers[telemetryItemIndex] > telemetryRates[telemetryItemIndex]) {
       refreshTelemetry(); //Avant d'envoyer la télémétrie, on s'assure qu'on a les dernières valeurs.
       sendTelemetryItemToTx(telemetryItemIndex);
       telemetryTimers[telemetryItemIndex] = 0;
       break;
     }
   }
          
   cpt++;
  }

}

bool GPF_CRSF::sendTelemetryItemToTx(uint8_t telemetryItemIndex) {
  bool    retour = false;
  uint8_t payloadLength = 0;
  uint8_t my_frame_type;

  switch (telemetryItemIndex) {
  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT:
    payloadLength = sizeof(crsf_heartbeat_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_HEARTBEAT;
    break;

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR:
    payloadLength = sizeof(crsf_sensor_battery_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR;
    break;  

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR:
    payloadLength = sizeof(crsf_sensor_vario_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_VARIO_SENSOR;
    break;    

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS:
    payloadLength = sizeof(crsf_sensor_gps_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_GPS;
    break;      

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR:
    payloadLength = sizeof(crsf_sensor_altitude_baro_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR;
    break;

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE:
    payloadLength = sizeof(crsf_sensor_attitude_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_ATTITUDE;
    break;  

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED:
    payloadLength = sizeof(crsf_sensor_flight_mode_text_based_s);
    my_frame_type = GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED;
    break;    
  
  default:
    return false; //Pas supposé jamais arriver ici.
    break;
  }
  
  const uint8_t frameLength      = payloadLength + 2; //+2 sont <Type> et <CRC>
  const uint8_t bytesToSendCount = frameLength + 2;   //+2 est <Device address or Sync Byte> et <Frame length>

  memset(bytesSendBuffer, 0, sizeof bytesSendBuffer); 
  bytesSendBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] = GPF_CRSF_SYNC_BYTE;
  bytesSendBuffer[GPF_CRSF_BYTE_POSITION_FRAME_LENGTH]             = frameLength;
  bytesSendBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE]               = my_frame_type;  
  
  switch (telemetryItemIndex) {
  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_HEARTBEAT:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_heartbeat, payloadLength);
    break;

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BATTERY_SENSOR:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_battery, payloadLength);
    break;  

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_VARIO_SENSOR:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_vario, payloadLength);
    break;    

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_GPS:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_gps, payloadLength);
    break;      
  
  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_BAROMETRIC_ALTITUDE_SENSOR:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_altitude_baro, payloadLength);
    break;

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_ATTITUDE:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_attitude, payloadLength);
    break;  

  case ENUM_TELEMETRY_GPF_CRSF_FRAME_TYPE_FLIGHT_MODES_TEXT_BASED:
    memcpy(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], &crsf_sensor_flight_mode_text_based, payloadLength);
    break;    

  default:
    return false; //Pas supposé jamais arriver ici.
    break;
  }

  //CRC includes Type and Payload of each frame.
  bytesSendBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD + payloadLength] = CRC8_calculate(&bytesSendBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE], frameLength - 1);

  if (serialPort->availableForWrite() >= bytesToSendCount) {
   serialPort->write(bytesSendBuffer,bytesToSendCount);
   //serialPort->flush();
   retour = true;
   
   #ifdef DEBUG_GPF_CRSF_ENABLED 
      for (int i = 0; i < DEBUG_PACKET_SENT_FRAME_TYPE_LIST_ITEM_COUNT; i++) { //On garde des stats pour le mode DEBUG
       if (my_frame_type == debug_packet_sent_frame_type_list[i]) {  
        debug_packet_sent_frame_type_count[i]++;
       }
      }
   #endif    

   /*
   DEBUG_GPF_CRSF_PRINT(F("*************************["));
   for (size_t i = 0; i < sizeof(bytesSendBuffer); i++) {
    DEBUG_GPF_CRSF_PRINT(bytesSendBuffer[i],HEX);
    DEBUG_GPF_CRSF_PRINT(F(" "));
   }
   DEBUG_GPF_CRSF_PRINTLN(F("]"));
   */

  //DEBUG_GPF_CRSF_PRINT(F("*************************["));
  //DEBUG_GPF_CRSF_PRINTLN(telemetryItemIndex);

  }

  return retour;
}

