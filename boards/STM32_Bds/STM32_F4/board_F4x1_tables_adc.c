
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            board_F4x1_tables_adc.c
//
//
//                                STM32  F4  01  RE  Nucleo         STM32F401xE
//                                STM32  F4  01  VC  Discovery      STM32F401xC
//                                STM32  F4  11  RE  Nucleo         STM32F411xE
//                                STM32  F4  11  VE  Discovery      STM32F411xE
//
//
//                    MCU SPECIFIC   ADC   GPIO  PIN   DEFINITIONS
//
//  History:
//    08/24/15 - Verified tables. 16 physical channels on ADC1.  Duq
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//*****************************************************************************

#if defined(STM32F401xE)
#include "stm32f401xe.h"    // pull in hardware defs
#elif defined(STM32F401xC)
#include "stm32f401xC.h"
#else
#include "stm32f411xe.h"
#endif

                     //---------------------------------------------------
                     //       Key portability constants / #defines
                     //---------------------------------------------------
#define  ADC_MAX_MODULES     1
#define  ADC_MAX_CHANNELS   16



//------------------------------------------------------------------------------
//                             ADC Pinout Definitions


//
//  ??? !!!   THE FOLLOWING NEED TO BE UPDATED/CHANGED FOR ADCs      ??? !!!


//                 TIMER_2            TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1    TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5
//   A1     PA_1   TIMER_CHANNEL_2    TIMER_CHANNEL_2

//   A2     PA_4   - none -

//                 TIMER_3            TIMER_1
//   A3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N

//   A4     PC_1   - none -

//   A5     PC_0   - none -

//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//                 TIMER_2
//   D13    PA_5   TIMER_CHANNEL_1   -                     -

//                 TIMER_3
//   D12    PA_6   TIMER_CHANNEL_1_ALT2

//                 TIMER_1            TIMER_3
//   D11    PA_7   TIMER_CHANNEL_1_N  TIMER_CHANNEL_2_ALT2

//                 TIMER_4
//   D10    PB_6   TIMER_CHANNEL_1

//                                    TIMER_3
//   D9     PC_7                      TIMER_CHANNEL_2_ALT2

//                 TIMER_1
//   D8     PA_9   TIMER_CHANNEL_2

//                 TIMER_1
//   D7     PA_8   TIMER_CHANNEL_1

//                 TIMER_2
//   D6     PB_10  TIMER_CHANNEL_3

//                 TIMER_3
//   D5     PB_4   TIMER_CHANNEL_1

//                 TIMER_3
//   D4     PB_5   TIMER_CHANNEL_2

//                 TIMER_2
//   D3     PB_3   TIMER_CHANNEL_2

//                 TIMER_1
//   D2     PA_10  TIMER_CHANNEL_3

//                 TIMER_2            TIMER_5                TIMER_9
//   D1     PA_2   TIMER_CHANNEL_3    TIMER_CHANNEL_3_ALT1   TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5                TIMER_9
//   D0     PA_3   TIMER_CHANNEL_4    TIMER_CHANNEL_4_ALT1   TIMER_CHANNEL_2

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4

//                 TIMER_3            TIMER_1
//   CNx    PB_1   TIMER_CHANNEL_4    TIMER_CHANNEL_3_N
//                 TIMER_4
//------------------------------------------------------------------------------

// ADC1 is defined as: #define ADC1   ((ADC_TypeDef *) ADC1_BASE) in stm32f401xe.h
// note: on STM32, user indexes start at 0, to handle channel 0 on STM's
//       Mask of 1 = supported only on ADC1
const ADC_CHANNEL_BLK  _g_adc_channels [] =             // Arduino  User Index ADC
        {
          {   0,       0,   0,        0,           0,ADC1 }, // no Internal TEMP  -4
          {   0,       0,   0,        0,           0,ADC1 }, // no Internal VCOMP -3
          {   0,       0,   0, ADC_CHANNEL_VBAT,   1,ADC1 }, // Internal VBAT -2    ADC_CHANNEL_18
          {   0,       0,   0, ADC_CHANNEL_VREFINT,1,ADC1 }, // Internal VREF -1    ADC_CHANNEL_17
                                                                // Arduino Index ADC
          { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_0,  1, ADC1 }, // PA0   A0     0   1
          { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_1,  1, ADC1 }, // PA1   A1     1   1
          { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_2,  1, ADC1 }, // PA2   VCP    2   1
          { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_3,  1, ADC1 }, // PA3   VCP    3   1
          { GPIOA, GPIO_PIN_4, 0, ADC_CHANNEL_4,  1, ADC1 }, // PA4   A2     4   1
          { GPIOA, GPIO_PIN_5, 0, ADC_CHANNEL_5,  1, ADC1 }, // PA5          5   1
          { GPIOA, GPIO_PIN_6, 0, ADC_CHANNEL_6,  1, ADC1 }, // PA6          6   1
          { GPIOA, GPIO_PIN_7, 0, ADC_CHANNEL_7,  1, ADC1 }, // PA7          7   1
          { GPIOB, GPIO_PIN_0, 0, ADC_CHANNEL_8,  1, ADC1 }, // PB0   A3     8   1
          { GPIOB, GPIO_PIN_1, 0, ADC_CHANNEL_9,  1, ADC1 }, // PB1          9   1
          { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_10, 1, ADC1 }, // PC0   A5    10   1
          { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_11, 1, ADC1 }, // PC1   A4    11   1
          { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_12, 1, ADC1 }, // PC2         12   1
          { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_13, 1, ADC1 }, // PC3         13   1
          { GPIOC, GPIO_PIN_4, 0, ADC_CHANNEL_14, 1, ADC1 }, // PC4         14   1
          { GPIOC, GPIO_PIN_5, 0, ADC_CHANNEL_15, 1, ADC1 }  // PC5         15   1
        };

              //-----------------------------------------------
              // Triggerable Timers/Events available on F4 ADC
              //    - TIM1_TRGO      - TIM1_CC4
              //    - TIM2_TRGO      - TIM3_TRGO
              //    - TIM5_TRGO      - etc
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
         { ADC_TRIGGER_TIMER_1_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T1_CC1 },  // TImer1 CC1
         { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T1_CC2 },  // TImer1 CC2
         { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T1_CC3 },  // TImer1 CC3

         { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T2_CC2 },  // TImer2 CC2
         { ADC_TRIGGER_TIMER_2_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T2_CC3 },  // TImer2 CC3
         { ADC_TRIGGER_TIMER_2_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T2_CC4 },  // TImer2 CC4
         { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T2_TRGO},  // TImer2 Update/TRG0

         { ADC_TRIGGER_TIMER_3_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T3_CC1 },  // TImer3 CC1
         { ADC_TRIGGER_TIMER_3,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T3_TRGO},  // TImer3 Update/TRG0

         { ADC_TRIGGER_TIMER_4_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T4_CC4 },  // TImer4 CC4

         { ADC_TRIGGER_TIMER_5_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T5_CC1 },  // TImer5 CC1
         { ADC_TRIGGER_TIMER_5_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T5_CC2 },  // TImer5 CC2
         { ADC_TRIGGER_TIMER_5_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T5_CC3 },  // TImer5 CC3

         { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIGCONV_Ext_IT11 },// External GPIO
         {     -1,                        0,                     0               }   // end of table
        };



//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_adc_get_channel_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the channels supported, and which GPIOs are used.
//******************************************************************************
ADC_CHANNEL_BLK  *board_adc_get_channel_block (int module_id, int channel_num)
{
    ADC_CHANNEL_BLK  *chan_blkp;

       // Note that addressing starts at -4, so we must add 4 to get 0 based
    chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num+INT_OFFSET];

    return (chan_blkp);
}


//******************************************************************************
//  board_adc_get_trigger_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the triggers supported, and which ExTSEL and MMS
//            Masks are used.
//******************************************************************************
ADC_TRIGGER_BLK  *board_adc_get_trigger_block (int module_id, int trigger_type)
{
    ADC_TRIGGER_BLK  *trigblkp;

                       // Point at the first entry in the table
    trigblkp = (ADC_TRIGGER_BLK*) &_g_adc_trigger_config_table[0];

                       // scan through the table to find a matching trigger type
    while (1)
      { if (trigblkp->trigger_user_api_id == -1)
           return (0L);             // end of table and no match
           if (trigblkp->trigger_user_api_id == trigger_type)
              {     // we got a successful math. Return it to caller
                 break;             // bail out of loop
              }
             else trigblkp++;       // otherwise, step to nexct entry in array
      }

    return (trigblkp);
}


//******************************************************************************
//  board_adc_enable_clock
//
//           Enable the clock for a specifc ADC module.
//           Requires the caller previously invoked board_adc_get_status_block()
//
//  Note that ADC_AUTO_MODULE = 0, so it is the first entry in the lookup table
//******************************************************************************
void  board_adc_enable_clocks (int module_id)
{
    __HAL_RCC_ADC1_CLK_ENABLE();       // Enable ADC Periph clock

         // Enable the ADC module's associated DMA module's clocks
    __HAL_RCC_DMA2_CLK_ENABLE();       // Enable DMA2 clock            F4 / F7
                                       // F4 uses DMA2_Stream0 / DMA_CHANNEL_0
}

/******************************************************************************/
