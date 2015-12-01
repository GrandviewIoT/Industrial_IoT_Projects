// 08/31/15 - Blows up when try to Enable/Start the DACs with HAL_DAC_Start_DMA
//              ==> DMA not properly setup or DAC/DMA Clocks not on, or Rupt Handler not setup ...
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           board_STM32_dacs.c
//
//
//  Common Logic for DAC support for STM32 MCUs.
//
//  The following MCUs with built-in DACs are supported:
//        - F0  72,  91
//        - F3  03,  34    (F3_34 has a 2nd DAC2 using PA6/DAC2_OUT1)
//        - F4  46
//        - F7  46
//        - L0  53         (1 channel, PA4 only)
//        - L1  52
//        - L4  76
//
//  All STM32's use the following fixed DAC outputs:
//        Channel_1 = PA4  (DAC_OUT1)     Arduino Header A2
//        Channel_2 = PA5  (DAC_OUT2)     Arduino Header D13
//
//  Uses a combination of HAL_Library (mostly for configuration) and direct
//  register calls (main execution).
//
//  Specific chip dependences are mainly constrained to tables (GPIOs used
//  specific DAC modules supported, etc). They are pulled in as #includes
//  based MCU type.
//
//  In general, the use of #ifdefs within the code has been minimized, in
//  order to keep things read-able and maintainable.
//
//  History:
//    12/30/14 - Created for Industrial IoT OpenSource project.  Duq
//    06/08/15 - Integrate in ADC, PWM, CRC changes to match rest of STM32 bds.
//    07/31/15 - Reworked to provide better factoring for DAC support. Duq
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

#include "user_api.h"
#include <math.h>

     //---------------------------------------------------------
     // Global Defines needed by DAC support
     //---------------------------------------------------------

typedef struct dac_channel_def        /* ADC Channel definitions */
    {
        GPIO_TypeDef *chan_gpio_port; /* Associated GPIO port                 */
        uint32_t     chan_gpio_pin;   /* Associated GPIO pin                  */
        uint16_t     chan_dac_id;     /* HAL Logical Channel id for this chan */
#if defined(STM32F446xx) || defined(STM32F746xx)
        DMA_Stream_TypeDef  *chan_dma_stream;      /* DMA stream to use       */
        uint32_t            chan_dma_subchannel;   /* DMA sub-channel to use  */
#else
        DMA_Channel_TypeDef *chan_dma_subchannel;  /* DMA sub-channel to use  */
#endif
        DAC_TypeDef         *chan_dac_module;      /* DAC Module to use       */
    } DAC_CHANNEL_BLK;

typedef struct dac_trigger_def             /* DAC Trigger definitions */
    {
        uint16_t     trigger_user_api_id;  /* User API id for the trigger */
        uint16_t     trigger_tmr_mmsmask;  /* Associated Mask for TIM MMS */
        uint32_t     trigger_dac_tselmask; /* Associated Mask for DAC TSEL*/
    } DAC_TRIGGER_BLK;


// move the following to boarddef.h at some point
int  board_dac_check_conversions_done (int dac_module_id, int sequencer);
int  board_dac_clear_conversions_done (int dac_module_id, int sequencer);
int  board_dac_SineWave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_Trianglewave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_SquareWave  (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int  board_timerpwm_enable_clock (int module_id);
long board_timerpwm_compute_prescalar (long period_val);
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id);

     //----------------------------------------
     //        Function Prototype refs
     //           internal use only
     //----------------------------------------
void  board_error_handler(void);
void  board_dac_enable_clocks (int module_id);
DAC_CHANNEL_BLK  *board_dac_get_channel_block (int module_id, int channel_num);
DAC_TRIGGER_BLK  *board_dac_get_trigger_block (int module_id, int trigger_type);


     //------------------------------------------------------------------------
     //
     // Pull in        MCU dependent GPIO / TIM tables and #defines
     //
     // The top of each module include key defines for:
     //          MAX_TIMERS,
     //          TMRPWM_NUM_MODULES
     //          TMRPWM_MAX_CHANNELS
     //
     // The following tables are pulled in:
     //          _g_timer_module_base[]       table/array
     //          _g_TIMx_TimPwmHandle's
     //          _g_timer_typedef_handle[]    table/array
     //          _g_TMPWM_IRQ_table[]         table/array
     //          _g_timer_base_channel_num[]  table/array
     //          _g_tmrpwm_mod_x_channels []  set of tables/arrays for GPIOs
     //          _g_tmrpwm_mod_channel_blk_lookup []    table/array
     //
     // Also, the following two utility routines are included in those files:
     //        - board_timerpwm_compute_prescalar()  because that is MCU speed
     //                                              and clock specific
     //        - board_timerpwm_enable_clock()       because which Timers are
     //                                              supported and the logic to
     //                                              start them is MCU specific
     //------------------------------------------------------------------------

#if defined(STM32F072xB)
//                                         STM32 - F072  Nucleo
#include "STM32_F0/board_F0_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel2_3_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_5_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel2_3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_5_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif



#if defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_F0_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel2_3_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_5_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel2_3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_5_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif


#if defined(STM32F303xE) || defined(STM32F303xC)
//                                         STM32 - F303  Nucleo and Discovery
#include "STM32_F3/board_F303_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel3_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo and Discovery
#include "STM32_F3/board_F334_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4

#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel3_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_IRQn

#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
#endif


#if defined(STM32F446xx)
//                                         STM32 - F446  Nucleo
#include "STM32_F4/board_F446_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA_CHANNEL_7
#define  DAC_CHAN_2_DMA_CHANNEL        DMA_CHANNEL_8
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Stream5_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Stream6_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Stream5_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Stream6_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "STM32_F7/board_F7_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA_CHANNEL_7
#define  DAC_CHAN_2_DMA_CHANNEL        DMA_CHANNEL_8
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Stream5_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Stream6_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Stream5_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Stream6_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo and Discovery
#include "STM32_L0/board_L0_tables_dac.c"

#define  DAC_CHAN_1_DMA_REQUEST_ID     DMA_REQUEST_9
#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel2
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel2_3_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel2_3_IRQn
#define  DAC_SINGLE_MODULE             1
#define  DAC_SINGLE_CHANNEL_ONLY       1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo and Discovery
#include "STM32_L1/board_L1_tables_dac.c"

#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel3_IRQHandler
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
  #ifndef  DAC1
  #define  DAC1      DAC
  #endif
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Nucleo and Discovery
#include "STM32_L4/board_L4_tables_dac.c"

//#define  DAC_CHAN_1_DMA_CHANNEL        DMA1_Channel3
//#define  DAC_CHAN_2_DMA_CHANNEL        DMA1_Channel4
#define  DAC_CHAN_1_DMA_ISR_IRQHandler DMA1_Channel3_IRQHandler  // verified 08/28
#define  DAC_CHAN_2_DMA_ISR_IRQHandler DMA1_Channel4_IRQHandler
#define  DAC_CHAN_1_NVIC_IRQn          DMA1_Channel3_IRQn
#define  DAC_CHAN_2_NVIC_IRQn          DMA1_Channel4_IRQn
#define  DAC_SINGLE_MODULE             1
#define  GPIO_SET_ANALOG_MODE          GPIO_MODE_ANALOG
void  DMA1_Channel3_IRQHandler (void);               // DAC DMA ISR - Channel 1
void  DMA1_Channel4_IRQHandler (void);               // DAC DMA ISR - Channel 2
#endif



//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES  and  DEFINEs
//
//                               for
//
//                            DAC  Support
//*****************************************************************************
//*****************************************************************************

#if defined(DAC2)
                               //---------------------------------------
                               //  Two  DAC  Modules  (e.g. F3_34)
                               //---------------------------------------
    DAC_HandleTypeDef      _g_DacHandle_1;     // Master handle for DAC1 module
    DAC_HandleTypeDef      _g_DacHandle_2;     // Master handle for DAC2 module

    DAC_ChannelConfTypeDef _g_Dac_Chan_Config_1;         // DAC channel 1 I/O
    DMA_HandleTypeDef      _g_DMA_dac_ch1;               // DAC channel 1 DMA

    DAC_ChannelConfTypeDef _g_Dac_Chan_Config_2;         // DAC channel 2 I/O
    DMA_HandleTypeDef      _g_DMA_dac_ch2;               // DAC channel 2 DMA

    DAC_ChannelConfTypeDef _g_Dac_Chan_Config_3;         // DAC channel 3 I/O
    DMA_HandleTypeDef      _g_DMA_dac_ch3;               // DAC channel 3 DMA

const  DAC_ChannelConfTypeDef  *_g_dac_channel_handle [] =
                                  { 0L,
                                    &_g_Dac_Chan_Config_1,
                                    &_g_Dac_Chan_Config_2,
                                    &_g_Dac_Chan_Config_3
                                  };

const  DMA_HandleTypeDef  *_g_dac_dma_handle [] =
                                  { 0L,
                                    &_g_DMA_dac_ch1,
                                    &_g_DMA_dac_ch2,
                                    &_g_DMA_dac_ch3
                                  };

    uint32_t               _g_DMA_dac_ch1_rupt_seen = 0;         // DEBUG COUNTER
    uint32_t               _g_DMA_dac_ch2_rupt_seen = 0;         // DEBUG COUNTER
    uint32_t               _g_DMA_dac_ch3_rupt_seen = 0;         // DEBUG COUNTER
#else
    DAC_HandleTypeDef      _g_DacHandle_1;       // Master handle for DAC module

    DAC_ChannelConfTypeDef _g_Dac_Chan_Config_1;         // DAC channel 1 I/O
    DMA_HandleTypeDef      _g_DMA_dac_ch1;               // DAC channel 1 DMA

    DAC_ChannelConfTypeDef _g_Dac_Chan_Config_2;         // DAC channel 2 I/O
    DMA_HandleTypeDef      _g_DMA_dac_ch2;               // DAC channel 2 DMA

const  DAC_ChannelConfTypeDef  *_g_dac_channel_handle [] =
                                  { 0L,
                                    &_g_Dac_Chan_Config_1,
                                    &_g_Dac_Chan_Config_2,
                                  };

const  DMA_HandleTypeDef  *_g_dac_dma_handle [] =
                                  { 0L,
                                    &_g_DMA_dac_ch1,
                                    &_g_DMA_dac_ch2,
                                  };

    uint32_t               _g_DMA_dac_ch1_rupt_seen = 0;         // DEBUG COUNTER
    uint32_t               _g_DMA_dac_ch2_rupt_seen = 0;         // DEBUG COUNTER
#endif

                      //-------------------------------------------------
                      //    DAC  Channel 1    Status and Config Flags
                      //-------------------------------------------------
    char        _g_DMA_DAC_Ch1_complete = 0; // status: 1 = DAC and DMA completed
    long        _g_dac_chan_1_frequency = 0; // SPS frequency for DAC Channel 1
    short       *_g_dac_chan_1_table;        // pointer to sample table for chan 1
    int         _g_dac_chan_1_table_length=0;// length of sample table (steps)

    uint16_t    _g_trigger_dtmr_mmsmask_1 = 0; // Chan 1 Associated Mask for TIM's  MMS
    uint16_t    _g_trigger_duser_api_id_1 = 0; //   "    Associated DAC trigger type

                      //-------------------------------------------------
                      //    DAC  Channel 2    Status and Config Flags
                      //-------------------------------------------------
    char        _g_DMA_DAC_Ch2_complete = 0; // status: 1 = DAC and DMA completed
    long        _g_dac_chan_2_frequency = 0; // SPS frequency for DAC Channel 2
    short       *_g_dac_chan_2_table;        // pointer to sample table for chan 2
    int         _g_dac_chan_2_table_length=0;// length of sample table (steps)

    uint16_t    _g_trigger_dtmr_mmsmask_2 = 0; // Chan 2 Associated Mask for TIM's  MMS
    uint16_t    _g_trigger_duser_api_id_2 = 0; //   "    Associated DAC trigger type

    int         _g_dac_init_error         = 0; // Saved error code from HAL_DAC_Init()
    int         _g_dac_chan_config_error  = 0; // Saved error code from HAL_DAC_ConfigChannel()
    int         _g_dac_start_error        = 0; // Saved error code from HAL_DAC_Start_DMA()

#define  pi     3.141592654
    float       _g_pi_div_180 = pi / 180.0;    // get radians ratio to degrees

                                               // generic proptotypes
void  DAC_CHAN_1_DMA_ISR_IRQHandler (void);          // DAC DMA ISR - Channel 1
void  DAC_CHAN_2_DMA_ISR_IRQHandler (void);          // DAC DMA ISR - Channel 2
void  DAC_CHAN_3_DMA_ISR_IRQHandler (void);          // DAC DMA ISR - Channel 3


//*****************************************************************************
//*****************************************************************************
//                               DAC   Routines
//
//                     COMMON CODE  -  GENERATE WAVEFORMS
//
//                     Used by built-in DACs and External DACs
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_dac_gen_sample_table
//
//          Generate a wave form table, with a given number of steps
//
//         Valid wave_types are:    DAC_GEN_SINEWAVE     DAC_GEN_SAWTOOTH
//                                  DAC_GEN_TRIANGLE     DAC_GEN_SQUAREWAVE
//
//*****************************************************************************

//  ??? include an amplitude factor/multiplier in here or in  dac_enable_channel() ???

//  ??? in future, allow a DC_Offset and max amplitude option for nore flexibility
//      ex: sine wave requires an implicit offset to get top/bottom of sinewave right.

int  board_dac_gen_sample_table (unsigned int wave_type, short *table_buf, int num_steps)
{
    int    i,  halfway_point;
    short  tbl_value;
    float  increment,  dc_offset_value,  curr_value;
    float  degree,  radians,  sine_val;
    float  degree_increment;

    if (wave_type < DAC_GEN_SINEWAVE || wave_type > DAC_GEN_SQUAREWAVE)
       return (-1);     // set APPROP ERROR CODE  ??? !!!
    if (num_steps < 2)
       return (-1);     // set APPROP ERROR CODE  ??? !!!

    halfway_point = (num_steps >> 1);           // divide by 2 to get halfway point
    curr_value = 0.0;                           // starting value
    switch (wave_type)
      { case DAC_GEN_SINEWAVE:          // standard sinewave, 1 cycle, 0 to 2 pi (360 degrees)
                  degree_increment = (360.0 / (float) num_steps); // how much each step is worth
                  degree = 0.0;                          // starting value (0 degrees)
                     // to get a "centered" sinewave, we need to add a DC offset,
                     // so that the top half is from 2048 -> 4095 -> 2048, and
                     // the bottom half is 2047 -> 0 -> 2047
                  dc_offset_value = 2047.0;
                  for (i = 0;  i < num_steps; i++)
                    { radians  = degree * _g_pi_div_180; // convert degrees to radians
                      sine_val = sin (radians);          // get associated sine value 0.0 to 1.0
//                    curr_value   = sine_val * 4095.0;  // convert to 0 - 4095 range
                      curr_value   = sine_val * dc_offset_value;
                      if (i < halfway_point)
                         {     // process top half of sine wave: 2047 -> 4095 -> 2047
                           tbl_value = ((short) curr_value);  // convert to int
                           if (tbl_value > 2048)
                              tbl_value = 2048;               // handle any float->int round off that would cause us to creep above 4095
                           table_buf[i] = tbl_value + 2047;   // add DC offset
                         }
                        else
                         {     // process bottom half of sine wave: 2047 -> 0 -> 2047
                               // sine_values at this point will all be negative
                           tbl_value = ((short) curr_value);  // convert to int
                           if (tbl_value < -2047)
                              tbl_value = -2047;              // handle any float->int round off that would cause us to creep below -2047
                           table_buf[i] = tbl_value + 2047;   // add DC offset
                         }
                      degree += degree_increment;        // step to next increment step
                    }
                  break;

        case DAC_GEN_SAWTOOTH:          // linear rising sawtooth, then drops to 0
///               for (i = 0;  i < 4095; i++)    // simple minded 1 tick per SPS point
///                  table_buf[i] = i;
                      // use float to minimize integer round-off issues.
                      // we calculate full scale 12 bits (4096), and apply amplitude later.
                  increment  = (4095.0 / (float) num_steps);  // how much each step is worth
                  for (i = 0;  i < num_steps; i++)
                    { table_buf[i] = (short) curr_value;  // convert to int
                      curr_value  += increment;           // step linearly to next value
                    }
                  break;

        case DAC_GEN_TRIANGLE :        // symmetric triangle centered at middle
                     // technically, a sawtooth is set of odd harmonic sine values,
                     // but a cheap way to calculate is to treat it as a rising linear
                     // ramp for the first 1/2 period, and a declining linjear ramp for
                     // the second half period.
                  increment  = (4095.0 / (float) halfway_point);  // how much each step is worth
                  for (i = 0;  i < halfway_point; i++)
                    {    // first half of period = linear rising
                      table_buf[i] = (short) curr_value;  // convert to int
                      curr_value  += increment;           // step up linearly to next value
                    }
                  for (i = halfway_point;  i < num_steps; i++)
                    {    // second half of period = linear declining
                      curr_value  -= increment;           // step down linearly to next value
                      table_buf[i] = (short) curr_value;  // convert to int
                      if (table_buf[i] < 0)
                         table_buf[i] = 0;                // clean up last value in case just dip below 0
                    }
                  break;

        case DAC_GEN_SQUAREWAVE:       // square wave = simple 50% on, 50 % off
                  for (i = 0; i < halfway_point; i++)
                     table_buf[i] = 0;                    // first half of table is zeros
                  for (i = halfway_point; i < num_steps; i++)
                     table_buf[i] = 4095;                 // second half of table is max
                  break;
      }

    return (0);               // denote completed successfully
}



//*****************************************************************************
//*****************************************************************************
//                               DAC   Routines
//
//                         SUPPORT  for  BUILT-IN  DACs
//
//
//*****************************************************************************
//*****************************************************************************


//*****************************************************************************
//  board_dac_init
//
//         Initialize a DAC module, and Configure
//         the overall sampling clock used for the DAC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//*****************************************************************************

int  board_dac_init (unsigned int dac_module_id, uint32_t clock_rate, int flags)
{
    int                rc;
    DAC_HandleTypeDef  *dacHandle;

#if defined(DAC2)
    if (dac_module_id == DACM2)
       { memset (&_g_DacHandle_2, 0, sizeof(_g_DacHandle_2));   // clear struct
         _g_DacHandle_2.Instance = DAC2;
         dacHandle = & _g_DacHandle_2;
       }
       else { memset (&_g_DacHandle_1, 0, sizeof(_g_DacHandle_1)); // clear struct
              _g_DacHandle_1.Instance = DAC1;
              dacHandle = & _g_DacHandle_1;
            }
#else
    memset (&_g_DacHandle_1, 0, sizeof(_g_DacHandle_1));  // clear struct

    _g_DacHandle_1.Instance = DAC1;   // Most STM32s have just a single DAC module
    dacHandle = &_g_DacHandle_1;
#endif

    board_dac_enable_clocks (dac_module_id);

    rc = HAL_DAC_Init (dacHandle);
    if (rc != HAL_OK)
       { _g_dac_init_error = rc;
         return (ERR_DAC_INITIALIZIATION_ERROR);    /* Initiliazation Error */
       }
    return (0);
}


//*****************************************************************************
//  board_dac_config_channel
//
//        Configure a single DAC channel.
//
//        Note that the sampling rate will be determined by the trigger source.
//
//        adc_module parm is really just an index number (0-3)
//        that denote which ADC module to use.
//        For the STM32 F4, there is only 1 ADC module: DAC1
//
//        trigger_type specifies the type of trigger that will be used
//           - User App
//           - Timer
//           - GPIO EXTI
//
//        The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//
//        STM32 built-in DAC logic supports 1 DAC module with two channels,
//        that are on dedicated pins:  Channel_1 = PA4 (DAC_OUT1)   Arduino A2
//                                     Channel_2 = PA5 (DAC_OUT2)   Arduino D13
//*****************************************************************************

int  board_dac_config_channel (unsigned int dac_module_id, int channel_num,
                               int trigger_type,
                               long sps_frequency, int flags)

{
    DAC_CHANNEL_BLK         *chan_blkp;
    DAC_TRIGGER_BLK         *trigblk;
    DAC_ChannelConfTypeDef  *dac_chan_handle;
    DMA_HandleTypeDef       *dac_dma_ptr;
    GPIO_InitTypeDef        GPIO_InitStruct;
    int                     rc;

    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));  // clear struct

       // there is only 1 DAC module on the each of the supported STM32s,
       // so for now  we ignore the module id

       // verify channel number is valid
    if (channel_num == DAC_CHAN_1  ||  channel_num == DAC_CHAN_2)
       ;
      else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE);

       //---------------------------------------------------------
       //                   Trigger Support
       //
       // Lookup and Save needed information for Trigger support.
       // This will be used at dac_Enable() time to turn on
       // the triggering that will be used by the DAC channel.
       //---------------------------------------------------------
    trigblk = (DAC_TRIGGER_BLK*) board_dac_get_trigger_block (dac_module_id,
                                                              trigger_type);
    if (trigblk == 0)
       return (ERR_DAC_UNSUPPORTED_TRIGGER_TYPE);

       //-------------------------------------------------------
       //        GPIO  Configuration  for  DAC  Channel
       //-------------------------------------------------------
    chan_blkp = (DAC_CHANNEL_BLK*) board_dac_get_channel_block (dac_module_id,
                                                                channel_num);
    GPIO_InitStruct.Pin  = chan_blkp->chan_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;         // tag it as Analog
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (chan_blkp->chan_gpio_port, &GPIO_InitStruct);

    if (channel_num == DAC_CHAN_1)
       {         // DAC Channel1: save operational parms
         _g_dac_chan_1_frequency  = sps_frequency; // save SPS frequency for this DAC
         _g_trigger_dtmr_mmsmask_1 = trigblk->trigger_tmr_mmsmask;  // save Mask for TIM's  MMS
         _g_trigger_duser_api_id_1 = trigger_type; // save associated DAC trigger type
       }
      else {     // DAC Channel2: save operational parms
             _g_dac_chan_2_frequency  = sps_frequency; // save SPS freq for this DAC
             _g_trigger_dtmr_mmsmask_2 =  trigblk->trigger_tmr_mmsmask;  // save Mask for TIM's  MMS
             _g_trigger_duser_api_id_2 = trigger_type; // save associated DAC trigger
           }

              //-----------------------------------------------------------------
              // get pointers to DAC Channel and DAC DMA objects we will be using
              //-----------------------------------------------------------------
    dac_chan_handle = (DAC_ChannelConfTypeDef*) _g_dac_channel_handle [channel_num];
    dac_dma_ptr     = (DMA_HandleTypeDef*) _g_dac_dma_handle [channel_num];

              //-----------------------------------------------------------
              // Setup DAC channel's Configuration
              //-----------------------------------------------------------
    dac_chan_handle->DAC_Trigger      = trigblk->trigger_dac_tselmask; // trigger to use
    dac_chan_handle->DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    rc = HAL_DAC_ConfigChannel (&_g_DacHandle_1, dac_chan_handle, chan_blkp->chan_dac_id);

    if (rc != HAL_OK)
       { _g_dac_chan_config_error = rc;
         return (ERR_DAC_CHANNEL_CONFIG_ERROR);   // Channel configuration Error
       }

              //-----------------------------------------------------------
              // Setup DAC's DMA Configuration for that channel
              //-----------------------------------------------------------
    memset (dac_dma_ptr, 0, sizeof(DMA_HandleTypeDef));  // clear struct
#if defined(STM32F446xx) || defined(STM32F746xx)
    dac_dma_ptr->Instance     = chan_blkp->chan_dma_stream;    // DMA1_Stream5/6
    dac_dma_ptr->Init.Channel = chan_blkp->chan_dma_subchannel; // DMA_CHANNEL_7
#else
    dac_dma_ptr->Instance     = chan_blkp->chan_dma_subchannel;
#endif
#if defined(DAC_CHAN_1_DMA_REQUEST_ID)
    dac_dma_ptr->Init.Request  = DAC_CHAN_1_DMA_REQUEST_ID;   // L0 special case
#endif
    dac_dma_ptr->Init.Direction           = DMA_MEMORY_TO_PERIPH;
    dac_dma_ptr->Init.PeriphInc           = DMA_PINC_DISABLE;
    dac_dma_ptr->Init.MemInc              = DMA_MINC_ENABLE;
    dac_dma_ptr->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dac_dma_ptr->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    dac_dma_ptr->Init.Mode                = DMA_CIRCULAR;
    dac_dma_ptr->Init.Priority            = DMA_PRIORITY_HIGH;
    rc = HAL_DMA_Init (dac_dma_ptr);
    if (rc != HAL_OK)
       { _g_dac_chan_config_error = rc;
         return (ERR_DAC_CHANNEL_DMA_CONFIG_ERROR);   // Channel configuration Error
       }

             // Associate the initialized DMA handle to the DAC handle
    if (channel_num == DAC_CHAN_1)
       __HAL_LINKDMA (&_g_DacHandle_1, DMA_Handle1, *dac_dma_ptr);   // DAC1 CH1
#if ! defined(DAC_SINGLE_CHANNEL_ONLY)
       else if (channel_num == DAC_CHAN_2)
               __HAL_LINKDMA (&_g_DacHandle_1,DMA_Handle2,*dac_dma_ptr); // DAC1 CH2
#endif
#if defined(DAC_CHAN_3_DMA_CHANNEL)
       else __HAL_LINKDMA (&_g_DacHandle_2, DMA_Handle1, *dac_dma_ptr);  // DAC2
#endif

    return (0);                  // denote worked OK
}


//*****************************************************************************
//  board_dac_check_conversions_completed
//
//          Checks if the ALL the conversions for a sinple DAC are done.
//
//          Returns:
//              True  (1) = Completed
//              False (0) = Busy
//              ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE
//*****************************************************************************
int  board_dac_check_conversions_completed (unsigned int dac_module_id, int channel_num)
{
       // there is only 1 DAC module on the each of the supported STM32s,
       // so for now  we ignore the module id

    if (channel_num == DAC_CHAN_1)
       return (_g_DMA_DAC_Ch1_complete);               // channel 1 status
       else if (channel_num == DAC_CHAN_2)
               return (_g_DMA_DAC_Ch2_complete);       // channel 2 status
       else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE); // invalid channel #
}


//*****************************************************************************
//  board_dac_clear_conversions_completed
//
//          Clears the DAC conversions "completed" flag to setup for next pass.
//
//*****************************************************************************
int  board_dac_clear_conversions_completed (unsigned int dac_module_id, int channel_num)
{
    if (channel_num == DAC_CHAN_1)
       _g_DMA_DAC_Ch1_complete = 0;       // reset channel 1 status for new pass
       else if (channel_num == DAC_CHAN_2)
               _g_DMA_DAC_Ch2_complete = 0;  //    ditto channel 2 status
       else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE); // invalid channel #

    return (0);             // denote worked OK
}


//*****************************************************************************
//  board_dac_disable_channel
//
//          Turn off one of the DAC channels.
//*****************************************************************************
int  board_dac_disable_channel (unsigned int dac_module_id, int channel_num, int flags)
{

    if (channel_num == DAC_CHAN_1)
       HAL_DAC_Stop (&_g_DacHandle_1, DAC_CHANNEL_1);
#if ! defined(DAC_SINGLE_CHANNEL_ONLY)
       else if (channel_num == DAC_CHAN_2)
               HAL_DAC_Stop (&_g_DacHandle_1, DAC_CHANNEL_2);
#endif
       else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE);

    return (0);                          // denote succeeded
}


//*****************************************************************************
//  board_dac_enable_channel
//
//          Turn on one or all the sequencers that have been configured.
//*****************************************************************************

int  board_dac_enable_channel (unsigned int dac_module_id, int channel_num, int flags)
{
    int   rc;
volatile long  period_ticks;

       // there is only 1 DAC module on the each of the supported STM32s,
       // so for now  we ignore the module id

       //-----------------------------------------------------------------------
       // Convert the frequecy * # steps into a Timer period value dictating when
       // each sample step should be fed into the DAC (via DMA trigger from TIM6)
       //-----------------------------------------------------------------------
    period_ticks = board_frequency_to_period_ticks (_g_dac_chan_1_frequency * _g_dac_chan_1_table_length);

    if (channel_num == DAC_CHAN_1)
       {     //-----------------------------------------------------------------
             // Configure the NVIC interrupt for the DMA used for DAC Channel 1
             //-----------------------------------------------------------------
         HAL_NVIC_SetPriority (DAC_CHAN_1_NVIC_IRQn, 2, 0);
         HAL_NVIC_EnableIRQ (DAC_CHAN_1_NVIC_IRQn);   // will invoke DMA1_Channel2_3_IRQHandler()

             //---------------------------------------------------
             //      Startup up DAC Channel 1 using DMA
             //---------------------------------------------------
// F0 blows up on this when HAL-Lib calls:   08/28/15
//       HAL_DMA_Start_IT (hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);
         rc = HAL_DAC_Start_DMA (&_g_DacHandle_1, DAC_CHANNEL_1,
                                 (uint32_t*) _g_dac_chan_1_table,
                                 _g_dac_chan_1_table_length,
                                 DAC_ALIGN_12B_R);
       }
#if ! defined(DAC_SINGLE_CHANNEL_ONLY)
      else if (channel_num == DAC_CHAN_2)
       {     //-----------------------------------------------------------------
             // Configure the NVIC interrupt for the DMA used for DAC Channel 2
             //-----------------------------------------------------------------
         HAL_NVIC_SetPriority (DAC_CHAN_2_NVIC_IRQn, 2, 0);
         HAL_NVIC_EnableIRQ (DAC_CHAN_2_NVIC_IRQn);   // will invoke DMA1_Channel4_5_IRQHandler()

             //---------------------------------------------------
             //      Startup up DAC Channel 2 using DMA
             //---------------------------------------------------
         rc = HAL_DAC_Start_DMA (&_g_DacHandle_1, DAC_CHANNEL_2,
                                 (uint32_t*) _g_dac_chan_2_table,
                                 _g_dac_chan_2_table_length,
                                 DAC_ALIGN_12B_R);
       }
#endif
      else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE);

    if (rc != HAL_OK)
       { _g_dac_start_error = rc;
         return (ERR_DAC_START_ERROR);   // DAC DMA Start Error
       }

    return (0);                          // denote succeeded
}


//*****************************************************************************
//  board_dac_get_app_trigger_masks
//
//          Pass back the trigger related masks that the User App requested
//          via ADC calls.
//          This is invoked by Timer logic to make sure both side are in
//          sync, when board_timerpwm_config_trigger_mode() is called.
//*****************************************************************************

void  board_dac_get_app_trigger_masks (unsigned int dac_channel,
                                       uint16_t *app_trigger_tmr_mmsmask,
                                       uint16_t *app_trigger_user_api_id)
{
#if FUTURE
    DAC_IO_CONTROL_BLK  *dac_blk;

    dac_blk = (DAC_IO_CONTROL_BLK*) board_dac_get_io_control_block (dac_channel);

    *app_trigger_tmr_mmsmask = dac_blk->dac_trigger_tmr_mmsmask;
    *app_trigger_user_api_id = dac_blk->dac_trigger_user_api_id;
#endif
                     if (dac_channel == 1)
                        {    // extract DAC channel 1 trigger parms
                          *app_trigger_tmr_mmsmask = _g_trigger_dtmr_mmsmask_1;
                          *app_trigger_user_api_id = _g_trigger_duser_api_id_1;
                        }
                       else
                        {    // extract DAC channel 2 trigger parms
                          *app_trigger_tmr_mmsmask = _g_trigger_dtmr_mmsmask_2;
                          *app_trigger_user_api_id = _g_trigger_duser_api_id_2;
                        }

}


//*****************************************************************************
//  board_dac_set_sample_table
//
//         Save the sample table address and length to be used for this channel.
//         At channel enable, will feed these into the DMA fot the channel.
//*****************************************************************************
int  board_dac_set_sample_table (unsigned int module_id, int channel_num,
                                 short *table_buf, int num_steps)
{
    if (channel_num == DAC_CHAN_1)
       { _g_dac_chan_1_table = table_buf;
         _g_dac_chan_1_table_length = num_steps;
       }
      else if (channel_num == DAC_CHAN_2)
              { _g_dac_chan_2_table = table_buf;
                _g_dac_chan_2_table_length = num_steps;
              }
      else return (ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE);

    return (0);                 // denote worked OK
}


/**************************************************************************
*                          DMA   DAC  CHANNEL 1   ISR
*
* @brief  This handles the DMA interrupt request for the DAC DMA channel 1
*         DMA1_CHANNEL3.
*         It routes it to the ST HAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
**************************************************************************/

void  DAC_CHAN_1_DMA_ISR_IRQHandler (void)   // eg DMA1_Channel2_3_IRQHandler
{
    _g_DMA_dac_ch1_rupt_seen++;                           // DEBUG COUNTER

    _g_DMA_DAC_Ch1_complete = 1; // set status that DAC and DMA completed.
                             // Used by dac_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// 09/02/15 - L1 52 testing
//           Somehow, this is causing the ADC_DMA callback handler to be called !!!
//           from the DAC_CHAN_1_DMA_ISR_IRQHandler.
//  ==> are they sharing the same rupt, and we must query which bit is set. ?
//  ==> is causing a severe nested, tight rupt loop. main uis never getting control back.
//  ==> only occurs when turn on DAC support. ADC runs fine otherwise.

    HAL_DMA_IRQHandler (_g_DacHandle_1.DMA_Handle1);  // Call post rupt cleanup
                                                    //   and reset rupt flags
}


#if ! defined(DAC_SINGLE_CHANNEL_ONLY)
/**************************************************************************
*                          DMA   DAC  CHANNEL 2   ISR
*
* @brief  This handles the DMA interrupt request for the DAC DMA channel
*         DMA1_CHANNEL4.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/
void  DAC_CHAN_2_DMA_ISR_IRQHandler (void)   // eg DMA1_Channel4_5_IRQHandler
{
    _g_DMA_dac_ch2_rupt_seen++;                           // DEBUG COUNTER

    _g_DMA_DAC_Ch2_complete = 1; // set status that DAC and DMA completed.
                             // Used by dac_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.


    HAL_DMA_IRQHandler (_g_DacHandle_1.DMA_Handle2);  // Call post rupt cleanup
                                                    //   and reset rupt flags
}
#endif


/******************************************************************************/
