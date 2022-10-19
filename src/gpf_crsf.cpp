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

GPF_CRSF::GPF_CRSF() {
    //Rien de spécial dans le constructeur pour le moment    
}

void GPF_CRSF::initialize(HardwareSerial *p_serialPort) {    
    serialPort = p_serialPort;
    serialPort->begin(GPF_CRSF_BAUDRATE, SERIAL_8N1);
    DEBUG_GPF_CRSF_PRINT(F("CRSF:Ouvre port serie..."));
    while (!serialPort) { };
    DEBUG_GPF_CRSF_PRINTLN("Ok");
    CRC8_createLut(0xD5);
}

void GPF_CRSF::readRx() {
   duration_between_frame_longest = max(duration_between_frame_longest, (unsigned long)duration_between_frame);

   if (duration_between_frame > GPF_CRSF_MIN_DURATION_BETWEEN_FRAME) {
    new_frame_is_about_to_start = true;
    reset_bytesReceivedCount_and_buffer();
   }

   if (bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH) { //Protection (On ne devrait jamais entrer ici normalement)
     // Ici, si on a 64 bytes dans le bytesReceivedCount c'est que le frame n'a pas été traité. 
     // Ce n'est pas normal et ca ne devrait jamais arriver alors on réinitialise et on oublie ce frame. 
     // De toute facon, il ne faut pas défoncer la taille du buffer (bytesReceivedBuffer) dont la taille est GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH.
     reset_bytesReceivedCount_and_buffer();
     DEBUG_GPF_CRSF_PRINTLN("Oups, bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH ???");
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
        for (int i = 0; i < GPF_CRSF_FRAME_TYPE_ITEM_COUNT; i++) { //On garde des stats pour le mode DEBUG
         if (frame_type == frame_type_list[i]) {  
          frame_type_frame_count[i]++;
         }
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
      for (int i = 0; i < GPF_CRSF_DEVICE_ADDRESS_ITEM_COUNT; i++) { //On garde des stats pour le mode DEBUG
       if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == device_address_list[i]) { 
        device_address_frame_count[i]++;
       }
      }
     #endif
     new_frame_is_about_to_start = false; //Une fois le premier byte du frame recu, on peut le remettre à false
    }
   }

   #ifdef DEBUG_GPF_CRSF_ENABLED
    if (debug_sincePrint > DEBUG_GPF_CRSF_DELAY) {

      DEBUG_GPF_CRSF_PRINTLN();
      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("bytesReceivedTotal="));
      DEBUG_GPF_CRSF_PRINT(bytesReceivedTotal);
      DEBUG_GPF_CRSF_PRINT(F(" duration_between_frame_longest="));
      DEBUG_GPF_CRSF_PRINT(duration_between_frame_longest);
      DEBUG_GPF_CRSF_PRINTLN();

      duration_between_frame_longest = 0;

      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("Frame count per device address: "));
      for (int i = 0; i < GPF_CRSF_DEVICE_ADDRESS_ITEM_COUNT; i++) {
        if (device_address_frame_count[i] > 0) {
         DEBUG_GPF_CRSF_PRINT(device_address_list[i], HEX); 
         DEBUG_GPF_CRSF_PRINT(F("="));
         DEBUG_GPF_CRSF_PRINT(device_address_frame_count[i]);
         DEBUG_GPF_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF:"));
      DEBUG_GPF_CRSF_PRINT(F("Frame count per frame type: "));
      for (int i = 0; i < GPF_CRSF_FRAME_TYPE_ITEM_COUNT; i++) {
        if (frame_type_frame_count[i] > 0) {
         DEBUG_GPF_CRSF_PRINT(frame_type_list[i], HEX); 
         DEBUG_GPF_CRSF_PRINT(F("="));
         DEBUG_GPF_CRSF_PRINT(frame_type_frame_count[i]);
         DEBUG_GPF_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_GPF_CRSF_PRINTLN();

      DEBUG_GPF_CRSF_PRINT(F("CRSF: Channels ch1="));
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

      DEBUG_GPF_CRSF_PRINT(F("CRSF: link_statistics active_antenna="));
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