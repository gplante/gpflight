/**
 * @file gpf_dshot.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-12-26
 * 
 * Voir fichier gpf_dshot.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_DSHOT_H
#define GPF_DSHOT_H

#include "gpf_cons.h"
#include <DMAChannel.h>

#define F_TMR F_BUS_ACTUAL // teensy 4

/* Defining DSHOT600 timings expressed in F_TMR periods
 * DSHOT600 has the following timings:
 *
 *          1670ns
 *          --------->
 *          ______
 * 1 bit :  |     |___|
 *          1250ns
 *          ____
 * 0 bit :  |   |_____|
 *          625ns
 * 
 * On the teensy 4.0, F_TMR == 600000000 (600Mhz)
 */

#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
//#define DSHOT_DMA_MARGIN          2             // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence

#define DSHOT_DURATION_MULTIPLIER_600 1 //DSHOT 600
#define DSHOT_DURATION_MULTIPLIER_300 2 //DSHOT 300
#define DSHOT_DURATION_MULTIPLIER_150 4 //DSHOT 150

#define DSHOT_DURATION_MULTIPLIER     DSHOT_DURATION_MULTIPLIER_150 //DSHOT 150 C'est amplement pour ce projet

#define DSHOT_BT_DURATION         (1670 * DSHOT_DURATION_MULTIPLIER)  // Duration of 1 DSHOT bit in ns
#define DSHOT_LP_DURATION         (1250 * DSHOT_DURATION_MULTIPLIER)  // Duration of a DSHOT long pulse in ns
#define DSHOT_SP_DURATION         (625 * DSHOT_DURATION_MULTIPLIER)   // Duration of a DSHOT short pulse in ns
#define DSHOT_MAX_VALUE           2047          // Maximum DSHOT value


class GPF_DSHOT {

    public:
        GPF_DSHOT();
        void     initialize();
        void     sendCommand( uint8_t motorNumero, uint16_t dshotCommand, bool requestTelemetry);
        uint16_t convertThrottlePercentToDshotValue( uint8_t percent);

    private:
        const uint16_t DSHOT_short_pulse  = uint64_t(F_TMR) * DSHOT_SP_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
        const uint16_t DSHOT_long_pulse   = uint64_t(F_TMR) * DSHOT_LP_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
        const uint16_t DSHOT_bit_length   = uint64_t(F_TMR) * DSHOT_BT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)

        DMAChannel                dma_channel[GPF_MOTOR_ITEM_COUNT];
        volatile uint16_t         dma_data[GPF_MOTOR_ITEM_COUNT][DSHOT_DMA_LENGTH];
        
        volatile uint8_t          eFlexPWM_pin[GPF_MOTOR_ITEM_COUNT] = {                                                                        
                                                                        2, // Moteur 1 = Output pin: 2 = EMC_04 = FLEXPWM4_PWM2_A ALT1 //See Table 10-1. Muxing Options at page 298
                                                                        4, // Moteur 2 = Output pin: 4 = EMC_06 = FLEXPWM2_PWM0_A ALT1 //See Table 10-1. Muxing Options at page 297
                                                                        5, // Moteur 3 = Output pin: 5 = EMC_08 = FLEXPWM2_PWM1_A ALT1 //See Table 10-1. Muxing Options at page 297
                                                                        6, // Moteur 4 = Output pin: 6 = B0_10  = FLEXPWM2_PWM2_A ALT2 //See Table 10-1. Muxing Options at page 298                                                                        
                                                                       };

        volatile IMXRT_FLEXPWM_t *eFlexPWM_module[GPF_MOTOR_ITEM_COUNT] = {                                                                          
                                                                           &IMXRT_FLEXPWM4, // FLEXPWM4_PWM2_A -> FLEXPWM4
                                                                           &IMXRT_FLEXPWM2, // FLEXPWM2_PWM0_A -> FLEXPWM2
                                                                           &IMXRT_FLEXPWM2, // FLEXPWM2_PWM1_A -> FLEXPWM2
                                                                           &IMXRT_FLEXPWM2, // FLEXPWM2_PWM2_A -> FLEXPWM2                                                                           
                                                                          };

        volatile uint8_t          eFlexPWM_submodule[GPF_MOTOR_ITEM_COUNT] = {                                                                              
                                                                              2, // FLEXPWM4_PWM2_A -> _PWM2 = submodule 2
                                                                              0, // FLEXPWM2_PWM0_A -> _PWM0 = submodule 0
                                                                              1, // FLEXPWM2_PWM1_A -> _PWM1 = submodule 1
                                                                              2, // FLEXPWM2_PWM2_A -> _PWM2 = submodule 2                                                                              
                                                                            };

        volatile uint8_t  	      eFlexPWM_submodule_channel[GPF_MOTOR_ITEM_COUNT] = {                                                                                      
                                                                                      0, // FLEXPWM4_PWM2_A -> _A    = A=0, B=1, X=2 
                                                                                      0, // FLEXPWM2_PWM0_A -> _A    = A=0, B=1, X=2 
                                                                                      0, // FLEXPWM2_PWM1_A -> _A    = A=0, B=1, X=2 
                                                                                      0, // FLEXPWM2_PWM2_A -> _A    = A=0, B=1, X=2                                                                                       
                                                                                     };

        volatile uint8_t  	      eFlexPWM_mux_alt[GPF_MOTOR_ITEM_COUNT] = {                                                                            
                                                                            1, // ALT1 -> 1
                                                                            1, // ALT1 -> 1
                                                                            1, // ALT1 -> 1
                                                                            2, // ALT2 -> 2                                                                            
                                                                           };

        volatile uint8_t  	      eFlexPWM_mux_dma_source[GPF_MOTOR_ITEM_COUNT] = {                                                                                   
                                                                                   DMAMUX_SOURCE_FLEXPWM4_WRITE2, // FLEXPWM4_PWM2_A -> _PWM2 = submodule 2 = WRITE2 //Table 4-3. DMA MUX Mapping, page 52
                                                                                   DMAMUX_SOURCE_FLEXPWM2_WRITE0, // FLEXPWM2_PWM0_A -> _PWM0 = submodule 0 = WRITE0 //Table 4-3. DMA MUX Mapping, page 52
                                                                                   DMAMUX_SOURCE_FLEXPWM2_WRITE1, // FLEXPWM2_PWM1_A -> _PWM1 = submodule 1 = WRITE1 //Table 4-3. DMA MUX Mapping, page 52
                                                                                   DMAMUX_SOURCE_FLEXPWM2_WRITE2, // FLEXPWM2_PWM2_A -> _PWM2 = submodule 2 = WRITE2 //Table 4-3. DMA MUX Mapping, page 52                                                                                   
                                                                                  };
        
};

#endif