
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            board_F446_tables_dac.c
//
//
//                                STM32  F4  46
//
//
//                      MCU SPECIFIC    DAC    GPIO  PIN   DEFINITIONS
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

#define  DAC_MAX_MODULES     1
#define  DAC_MAX_CHANNELS    2


//------------------------------------------------------------------------------
//                             DAC Pinout Definitions

//   A2     PA_4   DAC1_OUT1

//   CNx    PA_5   DAC1_OUT2

//------------------------------------------------------------------------------

const DAC_CHANNEL_BLK  _g_dac_channels [] =
       {
         {   0,       0,             0,           0,         0L }, // no Channel 0
         { GPIOA, GPIO_PIN_4, DAC_CHANNEL_1, DMA1_Stream5, DMA_CHANNEL_7, DAC }, // PA4  A2
         { GPIOA, GPIO_PIN_5, DAC_CHANNEL_2, DMA1_Stream6, DMA_CHANNEL_7, DAC }, // PA5
         {   0,       0,             0,           0,         0L }, // no Channel 3
       };



              //-----------------------------------------------
              // Triggerable Timers/Events available on F0 DAC
              //    - TIM2_TRGO      - TIM3_CC4
              //    - TIM6_TRGO      - TIM7_TRGO
              //    - TIM15_TRGO     - EXT Pin 9
              //    - User App SW
              //------------------------------------------------

const DAC_TRIGGER_BLK _g_dac_trigger_config_table [] =
        {      // User API          TIM MMS             DAC TSEL
          { DAC_TRIGGER_TIMER_2,  TIM_TRGO_UPDATE, DAC_TRIGGER_T2_TRGO  }, // TImer2 Update/TRG0

          { DAC_TRIGGER_TIMER_4,  TIM_TRGO_UPDATE, DAC_TRIGGER_T4_TRGO  }, // TImer4 Update/TRG0

          { DAC_TRIGGER_TIMER_5,  TIM_TRGO_UPDATE, DAC_TRIGGER_T5_TRGO  }, // TImer5 Update/TRG0

          { DAC_TRIGGER_TIMER_6,  TIM_TRGO_UPDATE, DAC_TRIGGER_T6_TRGO  }, // TImer6 Update/TRG0

          { DAC_TRIGGER_TIMER_7,  TIM_TRGO_UPDATE, DAC_TRIGGER_T7_TRGO  }, // TImer7 Update/TRG0

          { DAC_TRIGGER_TIMER_8,  TIM_TRGO_UPDATE, DAC_TRIGGER_T8_TRGO  }, // TImer8 Update/TRG0

          { DAC_TRIGGER_GPIO_PIN,          0,      DAC_TRIGGER_EXT_IT9  }, // External GPIO

          { DAC_TRIGGER_USER_APP,          0,      DAC_TRIGGER_SOFTWARE }, // User App triggers DAC

          {     -1,                        0,                     0     }  // end of table
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
//           Enable the clock for a specifc DAC module.
//           Requires the caller previously invoked board_adc_get_status_block()
//
//  Note that DAC_AUTO_MODULE = 0, so it is the first entry in the lookup table
//******************************************************************************
void  board_dac_enable_clocks (int module_id)
{
    __HAL_RCC_DAC_CLK_ENABLE();             // Enable F0 DAC Periph clock

         // Enable the DAC module's associated DMA module's clocks
    __HAL_RCC_DMA1_CLK_ENABLE();            // Enable DMA clock - F4_46 has only
                                            // 1 DAC module with 2 channels
}

/******************************************************************************/
