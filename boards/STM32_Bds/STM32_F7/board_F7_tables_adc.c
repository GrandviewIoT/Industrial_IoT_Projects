
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_F7_tables_adc.c
//
//
//                                STM32  F7  01
//
//
//                      MCU SPECIFIC    ADC    GPIO  PIN   DEFINITIONS
//
//  History:
//    10/14/15 - Verified tables. Duq
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

#define  ADC_MAX_MODULES     4
#define  ADC_MAX_CHANNELS   16

    ADC_IO_CONTROL_BLK  *board_adc_get_control_block (int adc_module_id);

    void  ADC1_DMA_IRQHandler (void);              // Function Prototypes


// note: on STM32, user indexes start at 0, to handle channel 0 on STM's                User  Physical
const ADC_CHANNEL_BLK  _g_adc_channels [] =                            // STM32 Arduino Index  ADC
        {
          {    0,     0,  0,         0,           0, ADC3},  // no Internal TEMP -4  ADC_CHANNEL_17   ??? !!! WVD VERIFY
          {    0,     0,  0,         0,           0, ADC3 }, // no Internal VCOMP -3
          {    0,     0,  0, ADC_CHANNEL_VBAT,    4, ADC3 }, // Internal VBAT -2  ADC_CHANNEL_18
          {    0,     0,  0, ADC_CHANNEL_VREFINT, 4, ADC3 }, // Internal VREF -1  ADC_CHANNEL_17
                                                            //    Arduino Index ADC
          { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_0,  4, ADC3 }, // PA0   A0    0    ADC3_IN0
          { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_1,  7, ADC3 }, // PA1         1    ADC123_IN1
          { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_2,  7, ADC3 }, // PA2         2    ADC123_IN2
          { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_3,  7, ADC3 }, // PA3         3    ADC123_IN3
          { GPIOF, GPIO_PIN_6, 0, ADC_CHANNEL_4,  4, ADC3 }, // PF6   A5    4    ADC3_IN4
          { GPIOF, GPIO_PIN_7, 0, ADC_CHANNEL_5,  4, ADC3 }, // PF7   A4    5    ADC3_IN5
          { GPIOF, GPIO_PIN_8, 0, ADC_CHANNEL_6,  4, ADC3 }, // PF8   A3    6    ADC3_IN6
          { GPIOF, GPIO_PIN_9, 0, ADC_CHANNEL_7,  4, ADC3 }, // PF9   A2    7    ADC3_IN7
          { GPIOF, GPIO_PIN_10,0, ADC_CHANNEL_8,  4, ADC3 }, // PF10  A1    8    ADC3_IN8
          { GPIOF, GPIO_PIN_3, 0, ADC_CHANNEL_9,  4, ADC3 }, // PF3         9    ADC3_IN9
          { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_10, 4, ADC3 }, // PC0        10    ADC123_10
          { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_11, 7, ADC3 }, // PC1        11    ADC123_11
          { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_12, 7, ADC3 }, // PC2        12    ADC123_12
          { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_13, 7, ADC3 }, // PC3        13    ADC123_13
          { GPIOF, GPIO_PIN_4, 0, ADC_CHANNEL_14, 4, ADC3 }, // PF4        14    ADC3_IN14
          { GPIOF, GPIO_PIN_5, 0, ADC_CHANNEL_15, 4, ADC3 }  // PF5        15    ADC3_IN15
        };

              //-----------------------------------------------
              // Triggerable Timers/Events available on F7 ADC
              //    - TIM1_CC1       - TIM3_CC4
              //    - TIM1_CC2       - TIM2_CC2
              //    - TIM1_CC3       - TIM4_CC4
              //    - TIM1_TRGO      - TIM1_TRGO_2
              //    - TIM2_TRGO      - TIM4_TRGO
              //    - TIM5_TRGO      - TIM6_TRG0
              //    - TIM8_TRGO      - TIM8_TRGO_2
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
          { ADC_TRIGGER_TIMER_1_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T1_CC1 },  // TImer1 CC1
          { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T1_CC2 },  // TImer1 CC2
          { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T1_CC3 },  // TImer1 CC3
          { ADC_TRIGGER_TIMER_1,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO},  // TImer1 Update/TRG0
          { ADC_TRIGGER_TIMER_1_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO2}, // TImer1 Update/TRG0-2

          { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T2_CC2 },  // TImer2 CC2
          { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T2_TRGO},  // TImer2 Update/TRG0

          { ADC_TRIGGER_TIMER_3_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T3_CC4 },  // TImer3 CC4

          { ADC_TRIGGER_TIMER_4_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T4_CC4 },  // TImer4 CC4
          { ADC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T4_TRGO},  // TImer4 Update/TRG0

          { ADC_TRIGGER_TIMER_5,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T5_TRGO},  // TImer5 Update/TRG0

          { ADC_TRIGGER_TIMER_6,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T6_TRGO},  // TImer6 Update/TRG0

          { ADC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO},  // TImer8 Update/TRG0
          { ADC_TRIGGER_TIMER_8_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO2}, // TImer8 Update/TRG0-2

          { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIGCONV_EXT_IT11 },// External GPIO
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

    chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num+INT_OFFSET];

#if NOT_NEEDED
    if (adc_module_id == 2)
       chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_2_channels [channel_num];       // F3/F7
       else chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_1_channels [channel_num];  // F3/F7
#endif

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
       // ON L4 / F7, default module must be ADC 3 !!!
    if (module_id == 1)
       __HAL_RCC_ADC1_CLK_ENABLE();                   // Enable ADC Periph clock
       else if (module_id == 2)
               __HAL_RCC_ADC2_CLK_ENABLE();
       else if (module_id == 3 || module_id == ADCMD)
               __HAL_RCC_ADC3_CLK_ENABLE();      // F7 46G just supports 3 ADCs

         // Enable the ADC module's associated DMA module's clocks
    __HAL_RCC_DMA2_CLK_ENABLE();       // Enable DMA2 clock        F4/F7
    __DMA1_CLK_ENABLE();               // Enable F3_03/L0/F0 DMA clock - F0 has only
                                       // 1 DMA unit, except F09x which has 2 and F3_34 has 3
}

//*****************************************************************************
