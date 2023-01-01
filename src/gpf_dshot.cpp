/**
 * @file gpf_dshot.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-12-26
 * 
 * Class pour communiquer avec un ou des ESC qui supporte le protocole DSHOT.
 * On ne peut pas utiliser de uart pour celà. 
 * On doit utiliser des DMA sinon écrire sur les pins simplement à l'aide du code via la loop() principale ne serait pas assez rapide à ce que j'ai pu lire sur Internet.
 * Je me suis inspiré du code du projet suivant:
 * https://github.com/jacqu/teensyshot
 * (Un Merci spécial à Arda Yiğit qui a bien voulu répondre à certaines interrogations à propos de ce projet)
 * 
 * Autres sources utiles:
 * https://github.com/betaflight/betaflight/pull/1282
 * https://github.com/betaflight/betaflight/wiki/DSHOT-ESC-Protocol
 * https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs#:~:text=DShot%20is%20a%20digital%20protocol,ranging%20from%200%20to%202047
 * https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
 * https://github.com/betaflight/betaflight/issues/673
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_dshot.h"
#include "gpf_debug.h"
#include "gpf_util.h"
#include <DMAChannel.h>

GPF_DSHOT::GPF_DSHOT() {

}

void GPF_DSHOT::initialize() {  
  int motorNumero, j;
  // Initialize DMA data
  for ( motorNumero = 0; motorNumero < GPF_MOTOR_ITEM_COUNT; motorNumero++ ) {
    for ( j = 0; j < DSHOT_DMA_LENGTH; j++ ) {
      dma_data[motorNumero][j] = 0;
    }
  }  

  // Configure pins on the board as DSHOT outputs
  // These pins are configured as eFlexPWM (FLEXPWMn) PWM outputs
  for ( motorNumero = 0; motorNumero < GPF_MOTOR_ITEM_COUNT; motorNumero++ ) {
    *(portConfigRegister( eFlexPWM_pin[motorNumero] ))  = eFlexPWM_mux_alt[motorNumero];
  }

  // Configure eFlexPWM module and submodule for PWM generation
  // --- submodule specific registers ---
  // INIT: initial counter value
  // VAL0: PWM_X compare value
  // VAL1: counter max value
  // VAL2: must be 0 for edge-aligned PWM
  // VAL3: PWM_A compare value
  // VAL4: must be 0 for edge-aligned PWM
  // VAL5: PWM_B compare value
  // OCTRL: invert polarity of PWMq FLEXPWM_SMOCTRL_POLq
  // DMAEN: FLEXPWM_SMDMAEN_VALDE to enable DMA
  // --- module specific registers ---
  // OUTEN: output enable for submodule n and PWM q FLEXPWM_OUTEN_PWMq_EN( 1 << n )
  for ( motorNumero = 0; motorNumero < GPF_MOTOR_ITEM_COUNT; motorNumero++ ) {
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].INIT = 0;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL0 = 0;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL1 = DSHOT_bit_length;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL2 = 0;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL3 = 0;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL4 = 0;
    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL5 = 0;

    if ( eFlexPWM_submodule_channel[motorNumero] == 2 ) { //A=0, B=1, X=2 
      (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].OCTRL = FLEXPWM_SMOCTRL_POLX;
      (*eFlexPWM_module[motorNumero]).OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << eFlexPWM_submodule[motorNumero]);
    } else if ( eFlexPWM_submodule_channel[motorNumero] == 1 ) { //A=0, B=1, X=2 
      (*eFlexPWM_module[motorNumero]).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << eFlexPWM_submodule[motorNumero]);
    } else { //A=0, B=1, X=2 
      (*eFlexPWM_module[motorNumero]).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << eFlexPWM_submodule[motorNumero]);      
    }

    (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].DMAEN = FLEXPWM_SMDMAEN_VALDE;
  }

  // Each DMA channel is linked to a unique eFlexPWM submodule
  // DMA channels are triggered by independant hardware events
  for ( motorNumero = 0; motorNumero < GPF_MOTOR_ITEM_COUNT; motorNumero++ ) {
    dma_channel[motorNumero].sourceBuffer( dma_data[motorNumero], DSHOT_DMA_LENGTH * sizeof( uint16_t ) );
    if ( eFlexPWM_submodule_channel[motorNumero]  == 2 ) {
      dma_channel[motorNumero].destination( (uint16_t&) (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL0 );
    } else if ( eFlexPWM_submodule_channel[motorNumero]  == 1 ) {
      dma_channel[motorNumero].destination( (uint16_t&) (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL5 );
    } else {
      dma_channel[motorNumero].destination( (uint16_t&) (*eFlexPWM_module[motorNumero]).SM[eFlexPWM_submodule[motorNumero]].VAL3 );      
    }
    dma_channel[motorNumero].triggerAtHardwareEvent( eFlexPWM_mux_dma_source[motorNumero] );
    //dma_channel[motorNumero].enable( );  
  }

  for ( motorNumero = 0; motorNumero < GPF_MOTOR_ITEM_COUNT; motorNumero++ ) {
   sendCommand(motorNumero, DSHOT_CMD_MOTOR_STOP, false);
  }  
}

void GPF_DSHOT::sendCommand( uint8_t motorNumero, uint16_t dshotCommand, bool requestTelemetry) {
  int       j;  
  uint16_t  data;

  // Check cmd value
  if ( dshotCommand > DSHOT_MAX_VALUE ) {
    dshotCommand = DSHOT_CMD_MOTOR_STOP; 
  }

  // Initialize DMA buffer  
  // Compute the packet to send
  // 11 first MSB = command
  // 12th MSB = telemetry request
  // 4 LSB = CRC
  data = ( dshotCommand << 5 ) | ( ((uint8_t)requestTelemetry) << 4 ); 
  data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;
  
  dma_channel[motorNumero].disable();
  // Generate DSHOT timings corresponding to the packet
  for ( j = 0; j < DSHOT_DSHOT_LENGTH; j++ )  {
    if ( data & ( 1 << ( DSHOT_DSHOT_LENGTH - 1 - j ) ) ) {
      dma_data[motorNumero][j] = DSHOT_long_pulse;
    } else {
      dma_data[motorNumero][j] = DSHOT_short_pulse;
    }
  }
  
  // Clear error flag on  DMA channel  
  dma_channel[motorNumero].clearError( );
  dma_channel[motorNumero].enable();
}