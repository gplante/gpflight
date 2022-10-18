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
#include "gpf_crsf.h"
#include "gpf_debug.h"

GPF_CRSF::GPF_CRSF() {
    //Print -> Stream -> HardwareSerial => [Serial]
    //devAddr =  GPF_MPU6050_DEFAULT_ADDRESS;
    //millis_old = millis();        
}

void GPF_CRSF::initialize(HardwareSerial *p_serialPort) {
    
    serialPort = p_serialPort;
    serialPort->begin(GPF_CRSF_BAUDRATE, SERIAL_8N1);
    //serialPort->begin(416666);
    //serialPort->begin(115200, SERIAL_8N1);
    DEBUG_CRSF_PRINT(F("CRSF:Ouvre port serie..."));
    while (!serialPort) { };
    DEBUG_CRSF_PRINTLN("Ok");

    CRC8_createLut(0xD5);
}

void GPF_CRSF::readRx() {
   duration_between_frame_longest = max(duration_between_frame_longest, (unsigned long)duration_between_frame);

   if (duration_between_frame > GPF_CRSF_MIN_DURATION_BETWEEN_FRAME) {
    new_frame_is_about_to_start = true;
    reset_bytesReceivedCount_and_buffer();
   }

   if (bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH) { //Protection
     // C'est pas normal qu'on ait 64 bytes dans le bytesReceivedCount ici et que le frame n'ait pas été traité. 
     // Il s'est passé quelque chose ???, bref, on reset, anyway faut pas défoncer le buffer.
     reset_bytesReceivedCount_and_buffer();
     DEBUG_CRSF_PRINTLN("Oups, bytesReceivedCount >= GPF_CRSF_BYTES_RECEIVED_BUFFER_MAX_LENGTH ???");
   }
      
   while (serialPort->available() > 0) {
    duration_between_frame = 0; //Lorsqu'on recoit quelque chose, on reset la durée
    //bytesReceivedBuffer[bytesReceivedCount] = serialPort->read();
    bytesReceivedBuffer[bytesReceivedCount++] = serialPort->read();
    bytesReceivedTotal++;

    if (bytesReceivedCount >= 3) { //Ca prend les 3 premiers octets pour voir le device address, la longeur totale du frame et le  type.

     if (bytesReceivedCount == 3) { //Lorsqu'on a exactement 3 bytes de recu, on peut connaitre la longeur totale du frame ainsi que son type
      frame_length = bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_LENGTH];
      frame_type   = bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE];

      #ifdef DEBUG_CRSF_ENABLED 
        for (int i = 0; i < GPF_CRSF_FRAME_TYPE_ITEM_COUNT; i++) { //On garde des starts pour le mode DEBUG
         //if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE] == frame_type_list[i]) { 
         if (frame_type == frame_type_list[i]) {  
          frame_type_frame_count[i]++;
         }
        }
      #endif      
     }

     if (bytesReceivedCount > 3) { 
      if (bytesReceivedCount == (frame_length + 2)) { //+2 sont les deux premiers octets <Device address or Sync Byte> et <Frame length>
       // Si on a recu le frame au complet, on peut le traiter
       parseFrame();
      }
      
     }
    }
   
    if (new_frame_is_about_to_start) {
     #ifdef DEBUG_CRSF_ENABLED 
      for (int i = 0; i < GPF_CRSF_DEVICE_ADDRESS_ITEM_COUNT; i++) { //On garde des starts pour le mode DEBUG
       if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == device_address_list[i]) { 
        device_address_frame_count[i]++;
       }
      }
     #endif
     new_frame_is_about_to_start = false; //Une fois le premier byte du frame recu, on peut le remettre à false
    }
   }


   #ifdef DEBUG_CRSF_ENABLED
    if (debug_sincePrint > DEBUG_CRSF_DELAY) {

      DEBUG_CRSF_PRINTLN();
      DEBUG_CRSF_PRINT(F("CRSF:"));
      DEBUG_CRSF_PRINT(F("bytesReceivedTotal="));
      DEBUG_CRSF_PRINT(bytesReceivedTotal);
      DEBUG_CRSF_PRINT(F(" duration_between_frame_longest="));
      DEBUG_CRSF_PRINT(duration_between_frame_longest);
      DEBUG_CRSF_PRINTLN();

      duration_between_frame_longest = 0;

      DEBUG_CRSF_PRINT(F("CRSF:"));
      DEBUG_CRSF_PRINT(F("Frame count per device address: "));
      for (int i = 0; i < GPF_CRSF_DEVICE_ADDRESS_ITEM_COUNT; i++) {
        if (device_address_frame_count[i] > 0) {
         DEBUG_CRSF_PRINT(device_address_list[i], HEX); 
         DEBUG_CRSF_PRINT(F("="));
         DEBUG_CRSF_PRINT(device_address_frame_count[i]);
         DEBUG_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_CRSF_PRINTLN();

      DEBUG_CRSF_PRINT(F("CRSF:"));
      DEBUG_CRSF_PRINT(F("Frame count per frame type: "));
      for (int i = 0; i < GPF_CRSF_FRAME_TYPE_ITEM_COUNT; i++) {
        if (frame_type_frame_count[i] > 0) {
         DEBUG_CRSF_PRINT(frame_type_list[i], HEX); 
         DEBUG_CRSF_PRINT(F("="));
         DEBUG_CRSF_PRINT(frame_type_frame_count[i]);
         DEBUG_CRSF_PRINT(F(" "));
        }
      }
      DEBUG_CRSF_PRINTLN();

      DEBUG_CRSF_PRINT(F("CRSF: Channels ch1="));
      DEBUG_CRSF_PRINT(crsf_channels.channel_1);
      DEBUG_CRSF_PRINT(F(" "));
      DEBUG_CRSF_PRINT(pwm_channels.channel_1);

      DEBUG_CRSF_PRINT(F(" ch2="));
      DEBUG_CRSF_PRINT(crsf_channels.channel_2);
      DEBUG_CRSF_PRINT(F(" "));
      DEBUG_CRSF_PRINT(pwm_channels.channel_2);

      DEBUG_CRSF_PRINT(F(" ch3="));
      DEBUG_CRSF_PRINT(crsf_channels.channel_3);
      DEBUG_CRSF_PRINT(F(" "));
      DEBUG_CRSF_PRINT(pwm_channels.channel_3);

      DEBUG_CRSF_PRINT(F(" ch4="));
      DEBUG_CRSF_PRINT(crsf_channels.channel_4);
      DEBUG_CRSF_PRINT(F(" "));
      DEBUG_CRSF_PRINT(pwm_channels.channel_4);

      DEBUG_CRSF_PRINTLN();

      DEBUG_CRSF_PRINT(F("CRSF: link_statistics active_antenna="));
      DEBUG_CRSF_PRINT(link_statistics.active_antenna);

      DEBUG_CRSF_PRINT(F(" up_rssi_ant1="));
      DEBUG_CRSF_PRINT(link_statistics.up_rssi_ant1);
      DEBUG_CRSF_PRINT(F(" up_rssi_ant2="));
      DEBUG_CRSF_PRINT(link_statistics.up_rssi_ant2);
      DEBUG_CRSF_PRINT(F(" down_rssi="));
      DEBUG_CRSF_PRINT(link_statistics.down_rssi);
      
      DEBUG_CRSF_PRINT(F(" up_snr="));
      DEBUG_CRSF_PRINT(link_statistics.up_snr);

      DEBUG_CRSF_PRINT(F(" rf_profile="));
      DEBUG_CRSF_PRINT(link_statistics.rf_profile);

      DEBUG_CRSF_PRINT(F(" up_rf_power="));
      DEBUG_CRSF_PRINT(link_statistics.up_rf_power);
      DEBUG_CRSF_PRINT(F(" down_link_quality="));
      DEBUG_CRSF_PRINT(link_statistics.down_link_quality);
      DEBUG_CRSF_PRINT(F(" up_link_quality="));
      DEBUG_CRSF_PRINT(link_statistics.up_link_quality);
      DEBUG_CRSF_PRINTLN();

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

  if (CRC8_calculate(&bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE], frame_length - 1) == frame_crc) { //Si le crc est bon
    if ( (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == GPF_CRSF_DEVICE_ADDRESS_BROADCAST_ADDRESS) ||
         (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_DEV_ADDRESS_OR_SYNC_BYTE] == GPF_CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER) ) {

    if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE] == GPF_CRSF_FRAME_TYPE_LINK_STATISTICS) {
      memcpy(&link_statistics, &bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], sizeof(libCrsf_link_statistics_s));
    }

    if (bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_FRAME_TYPE] == GPF_CRSF_FRAME_TYPE_RC_CHANNELS) {
      memcpy(&crsf_channels, &bytesReceivedBuffer[GPF_CRSF_BYTE_POSITION_PAYLOAD], sizeof(crsf_channels_t));

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
    DEBUG_CRSF_PRINTLN(F("CRSF: Oups, crcframe pas bon!"));
  }
  reset_bytesReceivedCount_and_buffer();

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