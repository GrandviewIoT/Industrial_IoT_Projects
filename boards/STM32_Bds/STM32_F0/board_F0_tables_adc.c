
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            board_F0_tables_adc.c
//
//
//                             STM32  F0  30 R8                     STM32F030x8
//                             STM32  F0  70 RB                     STM32F070xB
//                             STM32  F0  72 RB                     STM32F072xB
//                             STM32  F0  91 RC                     STM32F091xC
//
//
//                      MCU SPECIFIC    ADC    GPIO  PIN   DEFINITIONS
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
//******************************************************************************

#define  ADC_MAX_MODULES     1
#define  ADC_MAX_CHANNELS   16


              //-----------------------------------------------
              // ADC Channel Definitions
              //    - Which channels supported and their GPIOs
              //------------------------------------------------
// note: on STM32, user_api Channel indexes start at 0, to handle channel 0 on STM's

// Note: ADC1 is defined as: #define ADC1  ((ADC_TypeDef *) ADC1_BASE)  in  STM32F072xB.h
//       Mask of 1 = supported only on ADC1
const ADC_CHANNEL_BLK  _g_adc_channels [] =             // Arduino  User Index ADC
        {
          {   0,       0, 0,ADC_CHANNEL_TEMPSENSOR,1,ADC1 }, // Internal TEMP   -4  ADC_CHANNEL_16
          {   0,       0,   0,        0,           0,ADC1 }, // no Internal VCOMP -3
          {   0,       0,   0, ADC_CHANNEL_VBAT,   1,ADC1 }, // Internal VBAT   -2  ADC_CHANNEL_18
          {   0,       0,   0, ADC_CHANNEL_VREFINT,1,ADC1 }, // Internal VREF   -1  ADC_CHANNEL_17
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
              // Triggerable Timers/Events available on F0 ADC
              //    - TIM1_TRGO      - TIM1_CC4
              //    - TIM2_TRGO      - TIM3_TRGO
              //    - TIM15_TRGO     - etc
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
          { ADC_TRIGGER_TIMER_1,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO},  // TImer1 Update/TRG0
          { ADC_TRIGGER_TIMER_1_CC4, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T1_CC4 },  // TImer1 CC4

          { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T2_TRGO},  // TImer2 Update/TRG0 - not on all F0 MCUs

          { ADC_TRIGGER_TIMER_3,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T3_TRGO},  // TImer3 Update/TRG0

          { ADC_TRIGGER_TIMER_15,    TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T15_TRGO}, // TImer15 Update/TRG0

          {     -1,                        0,                     0                }  // end of table
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
    __ADC1_CLK_ENABLE();              // Enable F0 ADC Periph clock

         // Enable the ADC module's associated DMA module's clocks
    __DMA1_CLK_ENABLE();              // Enable MA clock - F0 has only
                                      // 1 DMA unit, except F09x which has 2
}

/******************************************************************************/
