
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_L4_tables_dac.c
//
//
//                              STM32  L4 76 VG   (Discovery)        STM32L476xx / USE_STM32L476G_DISCO_REVB
//                              STM32  L4 76 RG   (Nucleo)           STM32L476xx / USE_STM32L4XX_NUCLEO
//
//
//                      MCU SPECIFIC    DAC    SUPPORT
//
//  History:
//    08/28/15 - Verified tables. 2 physical channels on pins PA4 and PA5.  Duq
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


                     //---------------------------------------------------
                     //       Key portability constants / #defines
                     //---------------------------------------------------
#define  DAC_MAX_MODULES     1
#define  DAC_MAX_CHANNELS    2


//------------------------------------------------------------------------------
//                             DAC Pinout Definitions

//   A2     PA_4   DAC1_OUT1

//   CNx    PA_5   DAC1_OUT2

//------------------------------------------------------------------------------

const DAC_CHANNEL_BLK  _g_dac_channels [] =
       {
         {   0,       0,             0,           0,         0L  }, // no Channel 0
         { GPIOA, GPIO_PIN_4, DAC_CHANNEL_1, DMA1_Channel4, DAC1 }, // PA4  A2
         { GPIOA, GPIO_PIN_5, DAC_CHANNEL_2, DMA1_Channel5, DAC1 }, // PA5
//       { GPIOA, GPIO_PIN_5, DAC_CHANNEL_3, DMA1_Channelx, DAC2 }, // PA6
       };


              //-----------------------------------------------
              // Triggerable Timers/Events available on L4 DAC
              //    - TIM1_TRGO      - TIM1_CC4
              //    - TIM2_TRGO      - TIM3_TRGO
              //    - TIM15_TRGO     -  etc
              //------------------------------------------------
const DAC_TRIGGER_BLK _g_dac_trigger_config_table [] =
        {      // User API          TIM MMS         DAC EXTSEL
         { DAC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, DAC_TRIGGER_T2_TRGO },  // TImer2 Update/TRG0

         { DAC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, DAC_TRIGGER_T4_TRGO },  // TImer4 Update/TRG0

         { DAC_TRIGGER_TIMER_5,     TIM_TRGO_UPDATE, DAC_TRIGGER_T5_TRGO },  // TImer5 Update/TRG0

         { DAC_TRIGGER_TIMER_6,     TIM_TRGO_UPDATE, DAC_TRIGGER_T6_TRGO },  // TImer6 Update/TRG0

         { DAC_TRIGGER_TIMER_7,     TIM_TRGO_UPDATE, DAC_TRIGGER_T7_TRGO }, // TImer7 Update/TRG0

         { DAC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, DAC_TRIGGER_T8_TRGO },  // TImer8 Update/TRG0

         { DAC_TRIGGER_GPIO_PIN,          0,         DAC_TRIGGER_EXT_IT9 },  // External GPIO
         {     -1,                        0,                     0       }   // end of table
        };




//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_dac_get_channel_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the channels supported, and which GPIOs are used.
//******************************************************************************
DAC_CHANNEL_BLK  *board_dac_get_channel_block (int module_id, int channel_num)
{
    DAC_CHANNEL_BLK  *chan_blkp;

    chan_blkp = (DAC_CHANNEL_BLK*) &_g_dac_channels [channel_num];

    return (chan_blkp);
}


//******************************************************************************
//  board_dac_get_trigger_block
//
//            Locate and return the channel block for this DAC module.
//            It contains all the triggers supported, and which ExTSEL and MMS
//            Masks are used.
//******************************************************************************
DAC_TRIGGER_BLK  *board_dac_get_trigger_block (int module_id, int trigger_type)
{
    DAC_TRIGGER_BLK  *trigblkp;

                       // Point at the first entry in the table
    trigblkp = (DAC_TRIGGER_BLK*) &_g_dac_trigger_config_table[0];

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
//  board_dac_enable_clock
//
//           Enable the clock for a specifc ADC module.
//           Requires the caller previously invoked board_adc_get_status_block()
//
//  Note that ADC_AUTO_MODULE = 0, so it is the first entry in the lookup table
//******************************************************************************
void  board_dac_enable_clocks (int module_id)
{
    __HAL_RCC_DAC1_CLK_ENABLE();     // Enable DAC Periph clock

    __HAL_RCC_DMA1_CLK_ENABLE();     // Enable DAC module's DMA clock.
                                     // PA4 and PA5 operate as sub-channels on DMA1

    __HAL_RCC_GPIOA_CLK_ENABLE();    // Ensure associated GPIO clock is enabled
}

//******************************************************************************
