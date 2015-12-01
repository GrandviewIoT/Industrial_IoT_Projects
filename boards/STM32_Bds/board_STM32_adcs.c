
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           board_STM32_adc.c
//
//
//  Common Logic for ADC support for STM32 MCUs.
//
//  Uses a combination of HAL_Library (mostly for configuration) and direct
//  register calls (main execution).
//
//  Specific chip dependences are mainly constrained to tables (GPIOs used
//  specific ADC modules supported, etc). They are pulled in as #includes
//  based MCU type.
//
//  In general, the use of #ifdefs within the code has been minimized, in
//  order to keep things read-able and maintainable.
//
//  History:
//    12/30/14 - Created for Industrial IoT OpenSource project.  Duq
//    04/12/15 - Added multi-channel ADC support. Duq
//    05/25/15 - Fixed lingering issues in ADC DMA support. Duqu
//    07/18/15 - Reworked to provide better factoring. Duq
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

#include "user_api.h"               // pull in defs for User API calls

#include <math.h>

#define  INT_OFFSET      4          // Indexing offset used for Internal Sensors

   //---------------------------------------------------------------------------
   //               Summary of key ADC Characteristics by MCU
   //
   //  MCU     Num    Num Ext    DMA           DMA            IRQ
   //          ADC    Channels   Used          Channel        Vector
   //          ----   --------   -----         --------       -------
   //  F0_72   ADC1    0-15      DMA1          DMA1_Channel1  DMA1_Channel1_IRQn
   //  F0_91   ADC1    0-15      DMA1          DMA1_Channel1  DMA1_Channel1_IRQn
   //
   //  F4_01   ADC1    0-15      DMA2_Stream0  DMA_CHANNEL_0  DMA2_Stream0_IRQn
   //  F4_11   ADC1    0-15      DMA2_Stream0  DMA_CHANNEL_0  DMA2_Stream0_IRQn
   //  F4_46   ADC1    0-15 ?    DMA2_Stream0  DMA_CHANNEL_0  DMA2_Stream0_IRQn
   //
   //---------------------------------------------------------------------------

     //-------------------------------------------------------------------------
     // Global Defines needed by ADC support
     //-------------------------------------------------------------------------

typedef struct adc_module_def           /* ADC Module definitions */
    {
        ADC_TypeDef  *adc_base;         /* ADC base address */
    } ADC_MODULE_BLK;


typedef struct adc_ctl_io_ctl_def       /* ADC Control and Status definition */
   {
     ADC_HandleTypeDef  adc_Handle;     // ADC module typedef handle
     DMA_HandleTypeDef  dma_Handle;     // DMA module typedef handle
     ADC_TypeDef  *adc_hw_base;         // Hardware address of ADC Module
       uint16_t   dma_stream_IRQn;      // IRQn for this module's DMA

       short      adc_module_id;
       char       adc_clocks_on;        // 1 = ADC/DMA clocks were enabled
       char       adc_active_channels;  // total # active configured channels
       uint16_t   adc_step_num;         // current step number to use for sequencer
       uint8_t    adc_step_map [28];    // copy of which ADC channel used in which step

       char       ADC_complete;         // 1 = all non-DMA ADC conversions are complete
       char       ADC_DMA_complete;     // 3 = all DMA'ed  ADC conversions are complete
       uint16_t   adc_DMA_overrun;      // 1 = ADC overrun detected ==> bug !

       uint16_t   adc_conv_results[22]; // Internal buf to hold ADC DMAed results

       ADC_CB_EVENT_HANDLER  adc_callback_handler; // optional callback routine
       void       *adc_callback_parm;              // user callback parm

       uint16_t   adc_trigger_user_api_id; // User API id for the trigger
       uint16_t   adc_trigger_timer;       // Index to correct Timer/PWM - was _g_trigger_atmrpwm
       uint16_t   adc_trigger_tmr_mmsmask; // Associated Mask for TIM MMS
       uint32_t   adc_trigger_adc_extmask; // Associated Mask for ADC EXTSEL
   } ADC_IO_CONTROL_BLK;


typedef struct adc_channel_def        /* ADC Channel definitions */
    {
        GPIO_TypeDef *chan_gpio_port; /* Associated GPIO port             */
        uint32_t     chan_gpio_pin;   /* Associated GPIO pin              */
        uint32_t     chan_alt_func;   /* Channel Alternate Function Id    */
        uint16_t     chan_adc_id;     /* Logical Channel id for this ADC  */
        uint16_t     chan_adc_mask;   /* Allowed ADC usage mask: 1/3/7/15 */
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  BIGGIE F7 and F3
        ADC_TypeDef  *chan_adc_module;/* Default ADC Module to use        */
    } ADC_CHANNEL_BLK;

            // states for ADC_DMA_complete
#define  ADC_DMA_STATE_RESET         0
#define  ADC_DMA_STATE_BUSY          1     // User Trigger I/O request started
#define  ADC_DMA_STATE_INITAL_START  2
#define  ADC_DMA_STATE_IO_COMPLETE   3     // DMA has signaled completion of I/O
#define  ADC_DMA_STATE_RESULTS_READ  4     // adc_Read() was called
#define  ADC_DMA_STATE_IO_ERROR      5     // Overrun or ADC_Start error

typedef struct adc_trigger_def         /* ADC Trigger definitions */
    {
        uint16_t     trigger_user_api_id; /* User API id for the trigger   */
        uint16_t     trigger_tmr_mmsmask; /* Associated Mask for TIM MMS   */
        uint32_t     trigger_adc_extmask; /* Associated Mask for ADC EXTSEL*/
    } ADC_TRIGGER_BLK;


     //----------------------------------------
     //        Function Prototype refs
     //           internal use only
     //----------------------------------------
void  DMA1_Channel1_IRQHandler (void);             // ADC DMA ISR - all channels
void  DMA2_Stream0_IRQHandler (void);              // ADC DMA ISR - all channels
void  DMA2_Stream2_IRQHandler (void);              // DMA for ADC2
void  DMA2_Stream1_IRQHandler (void);              // DMA for ADC3

void  ADC1_DMA_IRQHandler (void);                  // ADC only (no-DMA) ISR

ADC_IO_CONTROL_BLK  *board_adc_get_io_control_block (unsigned int module_id);
ADC_CHANNEL_BLK  *board_adc_get_channel_block (int module_id, int channel_num);
ADC_TRIGGER_BLK  *board_adc_get_trigger_block (int module_id, int trigger_type);
ADC_TRIGGER_BLK  *board_adc_lookup_trigger (unsigned int module_id, int trigger_type,
                                        ADC_IO_CONTROL_BLK *adc_blk, int flags);
void  board_adc_enable_clocks (int module_id);


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
#include "STM32_F0/board_F0_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_F0_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1                  // THIS MAY BE INCORRECT - DOES F0_91 support 2 ADCs ?
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
#include "STM32_F1/board_F1_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  DMA_HAS_FIFO_MODE       1
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F303xE)
//                                         STM32 - F303  Nucleo
#include "STM32_F3/board_F303_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo
#include "STM32_F3/board_F334_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA1_Channel1       // 09/03/15 ??? !!! WVD 0- NEEDS WORK - ARE TWO ADC WITH TWO DIFF DMAs (Chan1 and Chan2)
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401 / F411 Nucleo and Discovery
#include "STM32_F4/board_F4x1_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
//#define  ADC_4_MODULES           1
//#define  ADC_MULTIPLE_MODULES    1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_56CYCLES    // 56 because of Grove high impedance
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA2_Stream0
//#define  DMA_CHANNEL_DEFAULT     DMA_CHANNEL_0
#define  DMA_CHANNEL_ADC1        DMA_CHANNEL_0
//#define  DMA_CHANNEL_ADC2        DMA_CHANNEL_1
//#define  DMA_CHANNEL_ADC3        DMA_CHANNEL_2
//#define  DMA_CHANNEL_ADC4        DMA_CHANNEL_3
#define  DMA_STREAM_IRQ          DMA2_Stream0_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA2_Stream0_IRQHandler
#define  DMA_HAS_FIFO_MODE       1
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32F446xx)
//                                         STM32 - F446 Nucleo
#include "STM32_F4/board_F446_tables_adc.c"

#define  ADC_1_MODULES          1
#define  ADC_SINGLE_MODULE      1
//#define  ADC_4_MODULES           1
//#define  ADC_MULTIPLE_MODULES    1
#define  ADC_RESOLUTION_DEFLT   ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT     ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT   ADC_SAMPLETIME_56CYCLES    // 56 because of Grove high impedance
#define  ADC_HAS_SQR_L          1
#define  DMA_INSTANCE           DMA2_Stream0
//#define  DMA_CHANNEL_DEFAULT   DMA_CHANNEL_0
#define  DMA_CHANNEL_ADC1       DMA_CHANNEL_0
//#define  DMA_CHANNEL_ADC2      DMA_CHANNEL_1
//#define  DMA_CHANNEL_ADC3      DMA_CHANNEL_2
//#define  DMA_CHANNEL_ADC4      DMA_CHANNEL_3
#define  DMA_STREAM_IRQ         DMA2_Stream0_IRQn
#define  DMA_ADC1_ISR_IRQHandler DMA2_Stream0_IRQHandler
#define  DMA_HAS_FIFO_MODE      1
#define  GPIO_SET_ANALOG_MODE   GPIO_MODE_ANALOG
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "STM32_F7/board_F7_tables_adc.c"

#define  ADC_3_MODULES           1
#define  ADC_MULTIPLE_MODULES    1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES
#define  ADC_HAS_SQR_L           1
#define  DMA_INSTANCE            DMA2_Stream0
#define  DMA_CHANNEL_DEFAULT     DMA_CHANNEL_3    // ADC 3 is F7 default ADC
#define  DMA_CHANNEL_ADC1        DMA_CHANNEL_0
#define  DMA_CHANNEL_ADC2        DMA_CHANNEL_1
#define  DMA_CHANNEL_ADC3        DMA_CHANNEL_2
#define  DMA_STREAM_IRQ          DMA2_Stream0_IRQn
#define  DMA_ADC1_ISR_IRQHandler DMA2_Stream0_IRQHandler   // ADC1 uses Stream0/Channel_0
#define  DMA_ADC2_ISR_IRQHandler DMA2_Stream2_IRQHandler   // ADC2 uses Stream2/Channel_1
#define  DMA_ADC3_ISR_IRQHandler DMA2_Stream1_IRQHandler   // ADC3 uses Stream1/Channel_2
#define  DMA_HAS_FIFO_MODE       1
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo / Discovery
#include "STM32_L0/board_L0_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_LOWPOWER_MODE   1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo / Discovery
#include "STM32_L1/board_L1_tables_adc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT    ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT      ADC_CLOCK_ASYNC_DIV2
#define  ADC_SAMPLETIME_DEFLT    ADC_SAMPLETIME_24CYCLES
#define  ADC_HAS_SQR_L           1
#define  ADC_HAS_LOWPOWER_MODE   1
#define  DMA_INSTANCE            DMA1_Channel1
#define  DMA_STREAM_IRQ          DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler  DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE    GPIO_MODE_ANALOG
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
#include "STM32_L4/board_L4_tables_adc.c"

#define  ADC_1_MODULES          1
#define  ADC_SINGLE_MODULE      1
//#define  ADC_3_MODULES        1
//#define  ADC_MULTIPLE_MODULE  1
#define  ADC_RESOLUTION_DEFLT   ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT     ADC_CLOCKPRESCALER_PCLK_DIV2  // ADC_CLOCK_ASYNC_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT   ADC_SAMPLETIME_24CYCLES_5
#define  ADC_HAS_SQR_L          1
#define  ADC_HAS_LOWPOWER_MODE  1
#define  OVERSAMPLING_ALLOWED   1
#define  DMA_INSTANCE           DMA1_Channel1
#define  DMA_REQUEST_NUM        DMA_REQUEST_0
#define  DMA_STREAM_IRQ         DMA1_Channel1_IRQn
#define  DMA_ADC1_ISR_IRQHandler DMA1_Channel1_IRQHandler
#define  GPIO_SET_ANALOG_MODE   GPIO_MODE_ANALOG_ADC_CONTROL
#endif

//------------------------------------------------------------------------------
//       Handle cross-platform HAL inconsistencies   (e.g. L0, ...)
//------------------------------------------------------------------------------
#ifndef ADC_EXTERNALTRIGCONVEDGE_NONE
#define ADC_EXTERNALTRIGCONVEDGE_NONE           ADC_EXTERNALTRIG_EDGE_NONE
#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ADC_EXTERNALTRIG_EDGE_FALLING
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ADC_EXTERNALTRIG_EDGE_RISINGFALLING
#define ADC_EXTERNALTRIGCONVEDGE_RISING         ADC_EXTERNALTRIG_EDGE_RISING
#endif

#ifndef ADC_SOFTWARE_START
#define  ADC_SOFTWARE_START    0
#endif


//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES  and  DEFINEs
//
//                               for
//
//                            ADC  Support
//*****************************************************************************
//*****************************************************************************

    ADC_ChannelConfTypeDef  _g_ChanConfig;

       //-----------------------------------------------------------------
       //          Single  ADC Moducle NCUs        (most of product line)
       //-----------------------------------------------------------------
#if defined(ADC_1_MODULES)
    ADC_IO_CONTROL_BLK  _g_adc_io_ctl_block_1;    // key status for ADC 1 (default)

const  ADC_TypeDef   *_g_adc_modules [] =
        {    ADC1,              // [0] = default module = ADCMD
             ADC1,
              0L,
              0L,
              0L
        };
#endif

       //-----------------------------------------------------------------
       //            Two  ADC Moducle NCUs                   (F0_91, ...)
       //-----------------------------------------------------------------
#if defined(ADC_2_MODULES)
    ADC_IO_CONTROL_BLK   _g_adc_io_ctl_block_1;  // key status for ADC 1 (default)
    ADC_IO_CONTROL_BLK   _g_adc_io_ctl_block_2;  // key status for ADC 2 module

const  ADC_TypeDef   *_g_adc_modules [] =
        {  { ADC1 },        // [0] = default module = ADC_AUTO_MODULE
           { ADC1 },
           { ADC2 },
           {  0L  },
           {  0L  }
        };
#endif

       //-----------------------------------------------------------------
       //            Three  ADC Moducle NCUs                   (F7, ...)
       //-----------------------------------------------------------------
#if defined(ADC_3_MODULES)
    ADC_IO_CONTROL_BLK   _g_adc_io_ctl_block_1;  // key status for ADC 1 (default)
    ADC_IO_CONTROL_BLK   _g_adc_io_ctl_block_2;  // key status for ADC 2 module
    ADC_IO_CONTROL_BLK   _g_adc_io_ctl_block_3;  // key status for ADC 3 module

const  ADC_TypeDef   *_g_adc_modules [] =
        {  { ADC3 },        // [0] = default module = ADCMD
           { ADC1 },
           { ADC2 },
           { ADC3 },
           {  0L  }
        };
#endif

       //-----------------------------------------------------------------
       //            Four  ADC Moducle NCUs                 (L4, F3, ...)
       //-----------------------------------------------------------------
#if defined(ADC_4_MODULES)
    ADC_IO_CONTROL_BLK     _g_adc_io_ctl_block_1;  // key status for ADC 1 (default)
    ADC_IO_CONTROL_BLK     _g_adc_io_ctl_block_2;  // key status for ADC 2 module
    ADC_IO_CONTROL_BLK     _g_adc_io_ctl_block_3;  // key status for ADC 3 module
    ADC_IO_CONTROL_BLK     _g_adc_io_ctl_block_4;  // key status for ADC 4 module

const  ADC_TypeDef   *_g_adc_modules [] =
        {  { ADC3 },        // [0] = default module = ADC_AUTO_MODULE
           { ADC1 },
           { ADC2 },
           { ADC3 },
           { ADC4 }
        };
#endif


///  char             _g_ADC_complete    = 0;  // 1 = all ADC conversions/DMAs are complete
///  char             _g_adc_clocks_on   = 0;
///  char             _g_active_channels = 0;
///  uint16_t         _g_sequencer       = 0;
///  uint16_t         _g_step_num        = 1;  // first sequencer step always = 1

///  unsigned char    _g_adc_step_map [16] = { 0 };  // indexed by channel #

///  unsigned short   _g_adc_conv_results[16];    // Internal buf to hold ADC DMAed results

//ADC_CB_EVENT_HANDLER _g_adc_callback       = 0L;
//   void              *_g_adc_callback_parm = 0;

///  uint16_t         _g_trigger_atmrpwm      = 0; // index to correct Timer/PWM
///  uint16_t         _g_trigger_auser_api_id = 0; // User API id for the trigger
///  uint16_t         _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
///  uint32_t         _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

///  int              _g_DMA_complete  = 0;       // 1 = all ADC conversion DMAs are complete
///  char             _g_DMA_overrun   = 0;       // 1 = DMA completed a new set before the
                                                 // previous one was processed

    unsigned long    dma_callback_seen = 0;      // DEBUG COUNTERs
    unsigned long    dma_rupt_seen     = 0;
    unsigned long    adc_rupt_cb_seen  = 0;


//*****************************************************************************
//*****************************************************************************
//                           COMMON   ADC   Code
//
//                                    for
//
//                                STM32  MCUs
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_adc_init
//
//         Initialize an ADC module, and configure the overall sampling
//         clock used for the ADC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - PWM Module 0  Generator 0 / 1 / 2 / 3
//           - PWM Module 1  Generator 0 / 1 / 2 / 3
//           - Timer n
//           - Comparator n
//
// Note that on most platforms, the Triggering is _module_ wide, in other
//      words, all channels (ADC 1-16) will be triggered off the same timer/pwm.
// Note that on most platforms, the Triggering is _module_ wide, in other
//      words, all channels (ADC 1-16) will be triggered off the same timer/pwm.
//
// STM32-F0 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// STM32-F7 has 3 ADC moodules, with a single sequencer allowing up to 16 channels.
// STM32-F4 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC_SQR1 -> ADC_SQR3.
// ADC results are stored in a single 32-bit register: ADC_DR
// Results are stored in the lower order 16-bits.
// We use DMA to process multiple ADC conversion results, to avoid
// "interrupt per channel" overhead and complexity.
//
// The STM32-F0 Nucleo 64-pin device physically supports up to 16 channels,
// The STM32-F7 Discovery 200-pin device physically supports up to 16 channels,
// The STM32-F4 Nucleo 64-pin device physically supports up to 16 channels,
// of which 5 are wired out to the Nucleo/Arduino pins.
// Two internal sources (Temperature Sensor, Battery Monitor) are also available.
// The other ADCs are wired out to the "Morpho" connector.
// We allow support for all 16 channels.
//
// The STM32-F0 supports a single 12-bit ADC, with up to 16 Channels.
// The STM32-F7 supports three 12-bit ADCs, with up to 16 Channels.
// The STM32-F4 supports a single 12-bit ADC, with up to 16 Channels.
//*****************************************************************************

int  board_adc_init (unsigned int module_id, uint32_t clock_rate,  int trigger_type,
                     int flags)
{
    ADC_IO_CONTROL_BLK  *adc_blk;
    ADC_TRIGGER_BLK     *trigblkp;
    int                 rc;

       //------------------------------------------------------------------
       // Get the associated control and status params for this ADC module
       //------------------------------------------------------------------
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (adc_blk->adc_clocks_on == 0)
       {    //------------------------------------------------
            // turn on the ADC and DMA clocks for this module
            //------------------------------------------------
         board_adc_enable_clocks (module_id);

            //-------------------------------------------------------------
            // reset this module's ADC Control/Status Block to its default state
            //-------------------------------------------------------------
         memset (adc_blk, 0, sizeof(ADC_IO_CONTROL_BLK));
         adc_blk->adc_module_id = module_id;
         adc_blk->adc_step_num  = 1;      // first sequencer step always = 1
         adc_blk->adc_hw_base   = (ADC_TypeDef*) _g_adc_modules [module_id];  // set ADC Hardware Base Address
         adc_blk->dma_stream_IRQn = DMA_STREAM_IRQ; // save associated NVIC IRQn

         adc_blk->adc_clocks_on = 1;      // denote clocks are now on

             //-----------------------------------------------------------------
             //                           ADC   CONFIG
             //
             // This is the opportune time to do initial config of the ADC and
             // DMA modules. Note that we will later have to manuually override
             // the initial number of ADC channels (set SQR1 xxxx value).
             //
// ??? CALIBRATION
             // If calibration is required, call that out as a separate subrtn.
             // Because F4 only has 1 ADC module, we can hard-code it.
             //-----------------------------------------------------------------
         adc_blk->adc_Handle.Instance = adc_blk->adc_hw_base; // instance = ADC module HW address
         adc_blk->adc_Handle.Init.DMAContinuousRequests = ENABLE;  // Use DMA
         adc_blk->adc_Handle.Init.EOCSelection          = DISABLE; // DMA will interrupt
                                                                   //   not ADC EOC.
#if ! defined(STM32L053xx)
         adc_blk->adc_Handle.Init.ScanConvMode          = ENABLE;  // scan multiple channels
#endif
         adc_blk->adc_Handle.Init.ContinuousConvMode    = DISABLE; // no continuous firing
         adc_blk->adc_Handle.Init.DiscontinuousConvMode = DISABLE;
         adc_blk->adc_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
         adc_blk->adc_Handle.Init.Resolution            = ADC_RESOLUTION_DEFLT; // 12 bit resol
#if defined(ADC_CLOCKPRESCALER_PCLK_DIV2)
         adc_blk->adc_Handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
#else
         adc_blk->adc_Handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
#endif

#if defined(ADC_MODULE_LEVEL_SAMPLING_TIME)
         adc_blk->adc_Handle.Init.SamplingTime          = ADC_SAMPLETIME_DEFLT;
#endif
         if (trigger_type == ADC_TRIGGER_USER_APP)
            {     //-----------------------------------------------------------
                  // no external triggers - use SW trigger calls from User App
                  //-----------------------------------------------------------
              adc_blk->adc_Handle.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
              adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
            }
          else
            {      //-------------------------------------------------------------
                   // loop thru trigger table, find entry, and save needed flags.
                   //
                   // We do not turn on all the trigger related TIM flags
                   // until adc_enable() is requested.
                   // We also require the User App issue a timer_ADC_Trigger_Start()
                   // at some point, to cause the ADC to start sampling.
                   //-------------------------------------------------------------
              trigblkp = (ADC_TRIGGER_BLK*) board_adc_get_trigger_block (module_id,
                                                                  trigger_type);
              if (trigblkp == 0L)
                 return (ERR_ADC_UNSUPPORTED_TRIGGER_TYPE);  // end of table and no match

                   // setup adc_blk->adc_Handle.Init with proper Trigger flags
              adc_blk->adc_Handle.Init.ExternalTrigConv = trigblkp->trigger_adc_extmask;
              if (flags & ADC_TRIGGER_FALLING)
                 adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
                 else if (flags & ADC_TRIGGER_RISEFALL)
                         adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
                         else adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
                   //---------------------------------------------------------------------
                   // save rest of parms to be used at adc_enable() and timerpwm_enable()
                   //---------------------------------------------------------------------
              adc_blk->adc_trigger_adc_extmask = trigblkp->trigger_adc_extmask; // ADC EXTSEL
              adc_blk->adc_trigger_tmr_mmsmask = trigblkp->trigger_tmr_mmsmask; // TIM MMS
              adc_blk->adc_trigger_user_api_id = trigger_type;
                   // the User API trigger type is an encoded field
                   // where the low order 4 bits = CCR, and the rest = index
                   // number of associated Timer/PWM
              if (trigblkp->trigger_tmr_mmsmask != 0)             // if is a EXTI GPIO pin = 0
                adc_blk->adc_trigger_timer = (trigger_type >> 4); // else save Timer/PWM index
            }

#if defined(ADC_HAS_SQR_L)
  #if defined(STM32F072xB) || defined(STM32F091xC)
              // F0 uses ADC_CHSEL sequencer bits that was setup in call to
              // HAL_ADC_Channel_Config
  #else
         adc_blk->adc_Handle.Init.NbrOfConversion  = 1;  // Initial value - this gets modified
                                                         // Does NOT exist as a parm on L0/F0
  #endif
#endif
               // L0 / L1 / L4 have various LowPower options
#if defined(ADC_HAS_LOWPOWER_MODE)
         adc_blk->adc_Handle.Init.LowPowerAutoWait      = ENABLE;
  #if defined(STM32L152xE) || defined(STM32L152xC)
         adc_blk->adc_Handle.Init.LowPowerAutoWait      = DISABLE;  // 09/02/15 - WVD DEBUG see ADC code  0x201 (CR2) vs 0x200 (failure_bits)
         adc_blk->adc_Handle.Init.LowPowerAutoPowerOff  = DISABLE;
  #endif
  #if defined(STM32L476xx) || defined(STM32L152xE) || defined(STM32L152xC)
            // these do not have LowPowerFrequencyMode / LowPowerAutoOff
  #else
         adc_blk->adc_Handle.Init.LowPowerAutoOff       = DISABLE; // ONLY exist for L0
         adc_blk->adc_Handle.Init.LowPowerFrequencyMode = ENABLE;
  #endif
#endif

#if defined(OVERSAMPLING_ALLOWED)
         adc_blk->adc_Handle.Init.OversamplingMode      = DISABLE;
#endif
         rc = HAL_ADC_Init (&adc_blk->adc_Handle);
         if (rc != HAL_OK)
            {
                 /* Initialization Error */
              return (ERR_ADC_INITIALIZATION_ERROR);
            }

             //-----------------------------------------------------------------
             //                           DMA   CONFIG
             //
             // Initialize the DMA associated with the ADC module.

             // F3_34 has only 1 DMA module with 7 channels, and no FIFO, no Burst.
             //-----------------------------------------------------------------
#if defined(DMA_CHANNEL_ADC1)               // single separate DMA channel
         adc_blk->dma_Handle.Init.Channel = DMA_CHANNEL_ADC1;
#endif
#if defined(DMA_CHANNEL_ADC2)               // several separate DMA channels
             // MCUs with multiple ADC modules will have separate channels
         if (module_id == 1)
            adc_blk->dma_Handle.Init.Channel = DMA_CHANNEL_ADC1;
            else if (module_id == 2)
                    adc_blk->dma_Handle.Init.Channel = DMA_CHANNEL_ADC2;
            else if (module_id == 3)
                    adc_blk->dma_Handle.Init.Channel = DMA_CHANNEL_ADC3;
            else if (module_id == ADCMD)
                    adc_blk->dma_Handle.Init.Channel = DMA_CHANNEL_DEFAULT;
#endif
#if defined(DMA_REQUEST_NUM)
         adc_blk->dma_Handle.Init.Request    = DMA_REQUEST_NUM;       // L4 only
#endif
         adc_blk->dma_Handle.Instance        = DMA_INSTANCE;
         adc_blk->dma_Handle.Init.Direction  = DMA_PERIPH_TO_MEMORY;  // ADC -> RAM
         adc_blk->dma_Handle.Init.PeriphInc  = DMA_PINC_DISABLE;
         adc_blk->dma_Handle.Init.MemInc     = DMA_MINC_ENABLE;       // step buf ptr
         adc_blk->dma_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         adc_blk->dma_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
         adc_blk->dma_Handle.Init.Mode       = DMA_CIRCULAR;     // Treat as circular buf
         adc_blk->dma_Handle.Init.Priority   = DMA_PRIORITY_HIGH;
#if defined(DMA_HAS_FIFO_MODE)
         adc_blk->dma_Handle.Init.FIFOMode   = DMA_FIFOMODE_DISABLE;
         adc_blk->dma_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
         adc_blk->dma_Handle.Init.MemBurst   = DMA_MBURST_SINGLE;
         adc_blk->dma_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
              // Note that F3/L0/F0 DMA do not have tunable FIFOs, so no FIFO parms
#endif
         rc = HAL_DMA_Init (&adc_blk->dma_Handle);

              // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&adc_blk->adc_Handle, DMA_Handle, adc_blk->dma_Handle);

              //----------------------------------------------------------
              //          Configure the NVIC for DMA interrupts
              // Configure NVIC for DMA transfer complete interrupt.
              //----------------------------------------------------------
         HAL_NVIC_SetPriority (DMA_STREAM_IRQ, 0, 0);
         HAL_NVIC_EnableIRQ (DMA_STREAM_IRQ);
       }

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate will be determined by the trigger source.
//
//         adc_module parm is really just an index number (0-3)
//         that denote which ADC module to use.
//         For the STM32 F4, there is only 1 ADC module: ADC1
//         For the STM32 F7, there is only 3 ADC modules: ADC1, ADC2, ADC3
//         Nearly all of the 16 channels are connected to ADC3, and alternative
//         paths are defined to allow parallel operation with ADC1 or ADC2.
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//
//         STM32 F4 supports a max of 16 ADC channels and 1 ADC module.
//         It has only 1 main sequencer.

//         STM32 F7 supports a max of 16 ADC channels and 3 ADC modules.
//         It has only 1 main sequencer.
//                    ^^^^^^^^^^^^^^^^^^^^
//                  sequencer per ADC ?  ==> array of sequncer/step values[module]id]

//         STM32 F3_34 supports a max of 19 ADC channels and 2 ADC modules.
//         It has only 2 main sequencers (one per ADC module).
//         We default to use ADC1, unless overridcen by user.

//         We do not use the sequencer parameter. It is for cross-platform
//         compatibility, for those platforms that have multiple sequencers.
//*****************************************************************************

int  board_adc_config_channel (unsigned int module_id, int channel_num,
                               int sequencer,  int step_num,
                               int last,  int flags)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    int               orig_step_num;
    int               rc;
    ADC_IO_CONTROL_BLK   *adc_blk;
    ADC_CHANNEL_BLK   *chan_blkp;

    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));  // clear struct

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( ! adc_blk->adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    if (channel_num < ADC_INTERNAL_TEMP  ||  channel_num > ADC_MAX_CHANNELS)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

          //-----------------------------------------------------
          //                Config  GPIO  pin  for  ADC
          //
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of 16 entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[]
          //----------------------------------------------------
    chan_blkp = (ADC_CHANNEL_BLK*) board_adc_get_channel_block (module_id,
                                                                channel_num);
    if (chan_blkp == 0L)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

    if (chan_blkp->chan_gpio_port != 0L)
       {   //---------------------------------------------------------------------
           // this is a real physical channel - setup associated GPIO pin for ADC.
           // Internal ADC channels (TEMP, VBAT, VREF) do not have any GPIO pins.
           //---------------------------------------------------------------------
           // first configure it as a straight GPIO Input, to clear everything out
         GPIO_InitStruct.Pin   = chan_blkp->chan_gpio_pin;
         GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull  = GPIO_NOPULL;
         GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
         HAL_GPIO_Init (chan_blkp->chan_gpio_port, &GPIO_InitStruct);
           // then configure it over to an Analog Input
         GPIO_InitStruct.Pin  = chan_blkp->chan_gpio_pin;
         GPIO_InitStruct.Mode = GPIO_SET_ANALOG_MODE;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init (chan_blkp->chan_gpio_port, &GPIO_InitStruct);
       }
     else ;   // chan_blkp->chan_gpio_port == 0L denotes an internal TEMP/VBTA/VREF sensor





// ??? SEE WHAT KEY OTHER THINGS NEED TO BE SETUP FOR INTERNAL ADC CHANNELS: TEMP, VBAT/VREF  WVD ??? !!!




          //----------------------------------------------------------------
          // setup the step/RANK within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save the step number passed on entry
    if (step_num == ADC_AUTO_STEP)
       step_num = adc_blk->adc_step_num;  // use our internally managed rank #

       //-----------------------------------------------------------------
       // Configure the associated ADC channel     (as a regular channel)
       //
       // Note: Based on the intterupts (IT) that occur after each number
       //       "uhADCxConvertedValue"  ADC conversions (IT by DMA end
       //       of transfer), select sampling time and ADC clock with sufficient
       //       duration as to not create an overhead situation in IRQHandler.
       //
       // Note that "Rank" is really sequence id - the sequence in which
       // these will fire, starting at one, and going up.
      //-----------------------------------------------------------------
    _g_ChanConfig.Channel      = chan_blkp->chan_adc_id; // set ADC channel #
#if defined(ADC_CHANNEL_LEVEL_SAMPLING_TIME)
    _g_ChanConfig.SamplingTime = ADC_SAMPLETIME_DEFLT;
#endif
#if defined(ADC_HAS_SQR_L)
    _g_ChanConfig.Rank         = step_num;       // set step/RANK # in sequencer
#else
           // some MCUs like L0, assign the channels in the order they
           // were configured and do not have a formal SQR_L setting.
           // They're ADC channel selection is done in fixed fashion
           // via the ADC_CHSELR register.
#endif
#if defined(ADC_USES_CONFIG_OFFSET)
    _g_ChanConfig.Offset       = 0;
#endif

    rc = HAL_ADC_ConfigChannel (&adc_blk->adc_Handle, &_g_ChanConfig);
    if (rc != HAL_OK)
       {
            /* Channel Configuration Error */
         return (ERR_ADC_CHANNEL_INITIALIZATION_ERROR);
       }

          // save the step number that was used for this channel
    adc_blk->adc_step_map [channel_num+INT_OFFSET] = (char) step_num;

    if (orig_step_num == ADC_AUTO_STEP)
       adc_blk->adc_step_num++;          // inc to next step slot in the sequencer

          // track how many channels have been configured for use
    adc_blk->adc_active_channels++;

    return (0);                          // denote success
}


//*****************************************************************************
//  board_adc_check_conversions_completed
//
//          Checks if the ALL the conversions for a sinple ADC are done.
//
//          For groups, the starting channel of the group, as specified as
//          the first channel listed in the board_adc_group_init() channels[]
//          array, is passed.
//
//          Returns:
//              True  (1) = Completed
//              False (0) = Busy
//*****************************************************************************
int  board_adc_check_conversions_completed (unsigned int module_id, int sequencer)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (adc_blk->ADC_DMA_complete == ADC_DMA_STATE_IO_COMPLETE)
       return (1);                       // All conversions are completed

    return (0);                          // ADC and DMA are still busy
}


//*****************************************************************************
//  board_adc_disable
//
//          Turn off one or all the sequencers that have been configured.
//
//          Reset everything in preparattioon for a re-configuratioon of
//          channels.
//*****************************************************************************
int  board_adc_disable (unsigned int module_id, int sequencer)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    HAL_ADC_Stop_DMA (&adc_blk->adc_Handle);

    return (0);                                     // denote success
}


//*****************************************************************************
//  board_adc_enable
//
//         Turn on one or all the sequencers that have been configured.
//
//         We use DMA for all ADC transfers, which by definition always runs
//         in interruipt mode. Upon DMA completion, the XXXX rtn is called.
//
//         callback_function and callback_parm are optional, to provide callback
//         when the ADC conversions are complete. If you want to poll for
//         result instead, set both of them to 0L (zero + L) aka NULL.
//*****************************************************************************
int  board_adc_enable (unsigned int module_id, int sequencer)
{
    uint32_t   DataEntries;
    int        rc;
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( !  adc_blk->adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_INITAL_START;  // Denote initial
                                    // Start conversion was issued.
                                    // The HAL_ADC_Start_DMA() below _always_
                                    // initiates the first ADC conversion,
                                    // even if tagged as USER_APP Trigger.
    adc_blk->adc_DMA_overrun  = 0;  // clear overflow flag

        //------------------------------------------------------------------
        // Update the "Number of Copnversions" in the ADC block
        // based upon the final number of channels configured.
        // To do this, we must update the ADC_SQR1_L field in SQR1 register.

        // Note that L count field is always N - 1 channels.

        // Note:  L0/F0 do not have SQRx registers. or max counts.
        // Instead, each channel to be scanned has its bit turned on in the
        // ADC_CHSELR reg, which is setup during HAL_ADC_ConfigChannel()

        //------------------------------------------------------------------
#if defined(ADC_HAS_SQR_L) || defined(STM32F334x8) || defined(STM32F303xE)
  #if defined(STM32F072xB) || defined(STM32F091xC)
              // F0 uses ADC_CHSEL sequencer bits that was setup in call to
              // HAL_ADC_Channel_Config
  #else
    adc_blk->adc_Handle.Instance->SQR1 &= ~(ADC_SQR1_L);       // clear the bits
    #if defined(ADC_SQR1)
           // note that the macro ADC_SQR1 subtracts one from the count _and_
           // shifts it by 20 bits to put the value into the ADC_SQR1_L position
       adc_blk->adc_Handle.Instance->SQR1 |=  ADC_SQR1(adc_blk->adc_active_channels);    // F4, F7, L4
    #elif defined(__ADC_SQR1_L)
       adc_blk->adc_Handle.Instance->SQR1 |= __ADC_SQR1_L(adc_blk->adc_active_channels); // L1
    #else
       MODIFY_REG (adc_blk->adc_Handle.Instance->SQR1, ADC_SQR1_L, (adc_blk->adc_active_channels - 1));  // L0
    #endif
  #endif
#endif
        //----------------------------------------------------------------------
        // the following both starts the ADC and apparently auto-initiates
        // the first Conversion IFF SW initiated, else lets trigger do its thing
        //----------------------------------------------------------------------
    DataEntries = adc_blk->adc_active_channels * 1;  // set # entries that DMA should xfer
    rc = HAL_ADC_Start_DMA (&adc_blk->adc_Handle, (uint32_t*) &adc_blk->adc_conv_results,
                            DataEntries);
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         return (ERR_ADC_ENABLE_ERROR);
       }
    return (0);                                    // denote success
}


//******************************************************************************
//  board_adc_get_io_control_block
//
//            Locate and return the control/status block, containing key
//            operating fields, flags, and values, based on module id.
//******************************************************************************
ADC_IO_CONTROL_BLK  *board_adc_get_io_control_block (unsigned int module_id)
{
    if (module_id > ADC_MAX_MODULES)
       return (0L);

#if defined(ADC_1_MODULES)
    return (&_g_adc_io_ctl_block_1);        // there is only 1 ADC module on MCU
#endif

#if defined(ADC_2_MODULES)
    if (module_id == 2)
       return (&_g_adc_io_ctl_block_2);
       else return (&_g_adc_io_ctl_block_1);
#endif

#if defined(ADC_3_MODULES)
    if (module_id == 3 || module_id == ADCMD)
       return (&_g_adc_io_ctl_block_3);               // L4 ADC default is ADC3
       else if (module_id == 2)
               return (&_g_adc_io_ctl_block_2);
       else return (&_g_adc_io_ctl_block_1);
#endif

#if defined(ADC_4_MODULES)
    if (module_id == 4)
       return (&_g_adc_io_ctl_block_4);
       else if (module_id == 3)
               return (&_g_adc_io_ctl_block_3);
       else if (module_id == 2)
               return (&_g_adc_io_ctl_block_2);
       else return (&_g_adc_io_ctl_block_1);
#endif

}


//*****************************************************************************
//  board_adc_get_resolution
//
//          Get the resolution of the ADC as either 6, 8, 10, or 12 bits.
//*****************************************************************************
int  board_adc_get_resolution (unsigned int module_id)
{
    ADC_IO_CONTROL_BLK  *adc_blk;
    uint32_t         hw_value;
    int              bit_resolution;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

bit_resolution = 12;
// TBD - get ADC register directly or call HAL API to do so ...

// then convert result to one of the following (or via table)
//  case ADC_6_BIT_RESULITION:
//  case ADC_8_BIT_RESULITION:
//  case ADC_10_BIT_RESULITION:
//  case ADC_128_BIT_RESULITION:

    return (bit_resolution);          // return bit resolution setting
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//
//          Return code:  0 = ADC conversion not completed yet (error)
//                        N (1-16) = number of channels processed/completed
//                       -1 = error  (e.g. DMA overrun)
//*****************************************************************************
int  board_adc_get_results (unsigned int module_id, int sequencer,
                            uint16_t *channel_results)
{
    uint32_t  adc_module;
    int      i;
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( ! adc_blk->adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    if (adc_blk->ADC_DMA_complete < ADC_DMA_STATE_IO_COMPLETE)
       return (0);     // denote 0 results because DMA rupt has not happened yet

    if (adc_blk->adc_DMA_overrun)
       {    // we had an overrun. Discard the results and tell user try again
         adc_blk->adc_DMA_overrun  = 0;                    // clear error flag
         adc_blk->ADC_DMA_complete = ADC_DMA_STATE_RESET;  // reset for new pass
         return (-1);          // denote we had an overrun condition
       }
    for (i = 0;  i < adc_blk->adc_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         channel_results[i] = adc_blk->adc_conv_results[i];
       }

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_RESULTS_READ; //setup for new pass

    return (adc_blk->adc_active_channels); // pass back number of completed conversions
}


//*****************************************************************************
//  board_adc_get_app_trigger_masks
//
//          Pass back the trigger related masks that the User App requested
//          via ADC calls.
//          This is invoked by Timer logic to make sure both side are in
//          sync, when board_timerpwm_config_trigger_mode() is called.
//*****************************************************************************

void  board_adc_get_app_trigger_masks (unsigned int adc_module_id,
                                       uint16_t *app_trigger_tmr_mmsmask,
                                       uint16_t *app_trigger_user_api_id)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (adc_module_id);

    *app_trigger_tmr_mmsmask = adc_blk->adc_trigger_tmr_mmsmask;
    *app_trigger_user_api_id = adc_blk->adc_trigger_user_api_id;
}


//*****************************************************************************
//  board_adc_lookup_trigger
//
//          Lookup up specific trigger entry that was requested by User App.
//          This walks through the trigger table, to find the associated entry
//          and all its associated parameters.
//*****************************************************************************
ADC_TRIGGER_BLK  *board_adc_lookup_trigger (unsigned int module_id, int trigger_type,
                                            ADC_IO_CONTROL_BLK *adc_blk, int flags)
{
    ADC_TRIGGER_BLK  *trigblkp;

    trigblkp = (ADC_TRIGGER_BLK*) board_adc_get_trigger_block (module_id, trigger_type);

    while (1)
      { if (trigblkp->trigger_user_api_id == -1)
           return (0L);                            // end of table and no match
        if (trigblkp->trigger_user_api_id == trigger_type)
           {     // setup adc_blk->adc_Handle.Init with proper Trigger flags
             adc_blk->adc_Handle.Init.ExternalTrigConv = trigblkp->trigger_adc_extmask;
             if (flags & ADC_TRIGGER_FALLING)
                   adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
                          else if (flags & ADC_TRIGGER_RISEFALL)
                                  adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
                                  else adc_blk->adc_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
                           // save rest of parms to tb used at adc_enable() and timerpwm_enable()
                       adc_blk->adc_trigger_adc_extmask  = trigblkp->trigger_adc_extmask; // ADC EXTSEL
                       adc_blk->adc_trigger_tmr_mmsmask = trigblkp->trigger_tmr_mmsmask; // TIM MMS
                       adc_blk->adc_trigger_user_api_id = trigger_type;
                           // the User API trigger type is an encoded field
                           // where the low order 4 bits = CCR, and the rest = index
                           // number of associated Timer/PWM
                       if (trigblkp->trigger_tmr_mmsmask != 0)      // a GEXTI GPIO pin = 0
                          adc_blk->adc_trigger_timer = (trigger_type >> 4); // save Timer/PWM index
                       break;             // bail out of loop
           }
        trigblkp++;             // step to next entry in array
      }

    return (trigblkp);          // return the entry we found
}


//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a group of ADCs, on a sequenced group
//*****************************************************************************
int  board_adc_user_trigger_start (unsigned int module_id, int trigger_type)
{
    uint32_t         DataEntries;
    int              rc;
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( ! adc_blk->adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    if (adc_blk->ADC_DMA_complete < ADC_DMA_STATE_IO_COMPLETE)
       return (ERR_ADC_STILL_BUSY);  // did not complete previous conversion yet

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_BUSY;  // denote starting I/O
    adc_blk->adc_DMA_overrun  = 0;               // clear I/O flags for new pass

        // apparently, to start another ADC conversion, you must issue
        // another xxx_Start_IT()  or  xxx_Start_DMA() request !
// DMA was already initialized and setup for circular mode (effectively resets
// after every cycle). So all we want to do now is trigger the ADC Start again.
// Do not re-init everything, it can cause crashes/faults
//  DataEntries = adc_blk->adc_active_channels * 1;   // set # entries that DMA should xfer
//  rc = HAL_ADC_Start_DMA (&adc_blk->adc_Handle, (uint32_t*) &adc_blk->adc_conv_results,
//                          DataEntries);
    rc = HAL_ADC_Start_IT (&adc_blk->adc_Handle); // Issue software start (SW) to ADC and DMA
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         adc_blk->ADC_DMA_complete = ADC_DMA_STATE_IO_ERROR;
         return (ERR_ADC_START_CONVERSION_ERROR);
       }

    return (0);                                   // denote success
}


//*****************************************************************************
//  board_adc_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when the ADC conversions are complete.
//*****************************************************************************
int  board_adc_set_callback (unsigned int module_id,
                             ADC_CB_EVENT_HANDLER  callback_function,
                             void *callback_parm)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

        //----------------------------------------------------------------
        // Save ADC completion callback parameters.
        //----------------------------------------------------------------
    adc_blk->adc_callback_handler = callback_function;
    adc_blk->adc_callback_parm    = callback_parm;

    return (0);                           // denote success
}


//*****************************************************************************
//  board_adc_set_resolution
//
//          Change the resolution of the ADC to 6, 8, 10, or 12 bits.
//          The default is always 12 bits.
//*****************************************************************************
int  board_adc_set_resolution (unsigned int module_id, int bit_resolution)
{
    ADC_IO_CONTROL_BLK  *adc_blk;
    uint32_t         hw_value;

       // Get the associated control and status params for this ADC module
    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (module_id);
    if (adc_blk == 0L)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    switch (bit_resolution)
      { case ADC_6_BIT_RESOLUTION:
            hw_value = 6;            // FILL IN REAL VALUE -
//#define ADC_RESOLUTION12b     ((uint32_t)0x00000000)       /*  ADC 12-bit resolution */
//#define ADC_RESOLUTION10b     ((uint32_t)ADC_CFGR1_RES_0)  /*  ADC 10-bit resolution */
//#define ADC_RESOLUTION8b      ((uint32_t)ADC_CFGR1_RES_1)  /*  ADC 8-bit resolution */
//#define ADC_RESOLUTION6b      ((uint32_t)ADC_CFGR1_RES)    /*  ADC 6-bit resolution */
            break;
        case ADC_8_BIT_RESOLUTION:
            hw_value = 8;            // FILL IN REAL VALUE -
            break;
        case ADC_10_BIT_RESOLUTION:
            hw_value = 10;           // FILL IN REAL VALUE -
            break;
        case ADC_12_BIT_RESOLUTION:
            hw_value = 12;           // FILL IN REAL VALUE -
            break;
         default:
            return (ERR_ADC_INVALID_BIT_RESOLUTION);
      }

// TBD - update ADC register directly or call HAL API to do so ...

    return (0);                           // denote success
}


/****************************************************************************
*                             ADC  ISR   Callback
*
*         ADC Conversion complete callback in non-blocking mode.
*         Called from DMA/ADC ISR Handler.
*
*         If running straight interrupts (xxx_IT), this gets called by
*         the HAL ADC library support when the ADC interrupt completes.
*
*         If running with DMA (xxx_DMA), this gets called
*         when the DMA interrupt completes, i.e. this gets
*         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
*
* @param  adc_blk->adc_Handle : adc_blk->adc_Handle handle
* @note   This example shows a simple way to report end of conversion,
*         and you can add your own implementation.
* @retval None
****************************************************************************/
void   HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *adc_Handle)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    adc_rupt_cb_seen++;                        // DEBUG COUNTER

// ??? !!!  ADD REST OF SUPPORT FOR THIS on 06/26/15 ---- ??? !!!
#if (ADC_SINGLE_MODULE)
    adc_blk = board_adc_get_io_control_block (ADC_MD);          // only 1 ADC module
#else
    if (adc_Handle->Instance == _g_adc_io_ctl_block_1.adc_Handle.Instance)
       adc_blk = &_g_adc_io_ctl_block_1;
       else if (adc_Handle->Instance == _g_adc_io_ctl_block_2.adc_Handle.Instance)
               adc_blk = &_g_adc_io_ctl_block_2;
  #if defined(ADC_3_MODULES) || defined(ADC_4_MODULES)
       else if (adc_Handle->Instance == _g_adc_io_ctl_block_3.adc_Handle.Instance)
               adc_blk = &_g_adc_io_ctl_block_3;
  #endif
  #if defined(ADC_4_MODULES)
       else if (adc_Handle->Instance == _g_adc_io_ctl_block_4.adc_Handle.Instance)
               adc_blk = &_g_adc_io_ctl_block_4;
  #endif
       else return;           // completely unknwon Handle ==> user specific
#endif


#if (ALWAYS_USING_DMA)
       // we are always using DMA, so this causes false trigger/callbacks to
       // user app, which can cause confusion and hangs at User App level,
       // because this callback is invoked by HAL at ADC complete, and it
       // also calls the DMA callback (below) at DMA complete.
       // Worse yet, the HAL turd calls this twice (once at Half-Convert and
       // once at Full convert complete), even  though we did not ask for
       // the Half-Complete callback. Thanks for nothing !
       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (adc_blk->adc_callback_handler != 0L)
       {
          (adc_blk->adc_callback_handler) (adc_blk->adc_callback_parm,
                                           adc_blk->adc_conv_results,
                                           adc_blk->adc_active_channels,
                                           0);       // invoke user handler
       }
#endif
}


/************************************************************************
*                              DMA    ADC1    ISR
*
* @brief  This handles the DMA interrupt request for the 1st ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/


// 09/03/15 ??? !!! WVD FIX - F3 03 has 4 separate ADCs and 4 separate DMAs - need to add
//                            F7 has 3 ADC and 3 separate DMAs

void  DMA_ADC1_ISR_IRQHandler (void)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    dma_rupt_seen++;                               // DEBUG COUNTER

    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (ADC_M1);

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_IO_COMPLETE;   // set status that
                             // ADCs and DMA I/O has completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???
// 07/26/15 - BAD BAD  The following is potentially screwing things up - since it
//            calls the above HAL_ADC_ConvCpltCallback() so the User App gets
//            two ADC callbacks for 1 completion.  BADDD !!
    HAL_DMA_IRQHandler (adc_blk->adc_Handle.DMA_Handle);  // Call post rupt cleanup
                                                   //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (adc_blk->adc_callback_handler != 0L)
       {
          (adc_blk->adc_callback_handler) (adc_blk->adc_callback_parm,
                                           adc_blk->adc_conv_results,
                                           adc_blk->adc_active_channels,
                                           0);        // Invoke user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}


#if defined(ADC_2_MODULES) || defined(ADC_3_MODULES) || defined(ADC_4_MODULES)
/************************************************************************
*                              DMA    ADC2    ISR
*
* @brief  This handles the DMA interrupt request for the 2nd ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA_ADC2_ISR_IRQHandler (void)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    dma_rupt_seen++;                               // DEBUG COUNTER

    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (ADC_M2);

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_IO_COMPLETE;   // set status that
                             // ADCs and DMA I/O has completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???
// 07/26/15 - BAD BAD  The following is potentially screwing things up - since it
//            calls the above HAL_ADC_ConvCpltCallback() so the User App gets
//            two ADC callbacks for 1 completion.  BADDD !!
    HAL_DMA_IRQHandler (adc_blk->adc_Handle.DMA_Handle);  // Call post rupt cleanup
                                                       //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (adc_blk->adc_callback_handler != 0L)
       {
          (adc_blk->adc_callback_handler) (adc_blk->adc_callback_parm,
                                           adc_blk->adc_conv_results,
                                           adc_blk->adc_active_channels,
                                           0);        // Invoke user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}
#endif


#if defined(ADC_3_MODULES) || defined(ADC_4_MODULES)
/************************************************************************
*                              DMA    ADC3    ISR
*
* @brief  This handles the DMA interrupt request for the 3rd ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA_ADC3_ISR_IRQHandler (void)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    dma_rupt_seen++;                               // DEBUG COUNTER

    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (ADC_M3);

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_IO_COMPLETE;   // set status that
                             // ADCs and DMA I/O has completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???
// 07/26/15 - BAD BAD  The following is potentially screwing things up - since it
//            calls the above HAL_ADC_ConvCpltCallback() so the User App gets
//            two ADC callbacks for 1 completion.  BADDD !!
    HAL_DMA_IRQHandler (adc_blk->adc_Handle.DMA_Handle);  // Call post rupt cleanup
                                                   //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (adc_blk->adc_callback_handler != 0L)
       {
          (adc_blk->adc_callback_handler) (adc_blk->adc_callback_parm,
                                           adc_blk->adc_conv_results,
                                           adc_blk->adc_active_channels,
                                           0);        // Invoke user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}
#endif


#if defined(ADC_4_MODULES)
/************************************************************************
*                              DMA    ADC4    ISR
*
* @brief  This handles the DMA interrupt request for the 4th ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA_ADC4_ISR_IRQHandler (void)
{
    ADC_IO_CONTROL_BLK  *adc_blk;

    dma_rupt_seen++;                               // DEBUG COUNTER

    adc_blk = (ADC_IO_CONTROL_BLK*) board_adc_get_io_control_block (ADC_M4);

    adc_blk->ADC_DMA_complete = ADC_DMA_STATE_IO_COMPLETE;   // set status that
                             // ADCs and DMA I/O has completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???
// 07/26/15 - BAD BAD  The following is potentially screwing things up - since it
//            calls the above HAL_ADC_ConvCpltCallback() so the User App gets
//            two ADC callbacks for 1 completion.  BADDD !!
    HAL_DMA_IRQHandler (adc_blk->adc_Handle.DMA_Handle);  // Call post rupt cleanup
                                                   //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (adc_blk->adc_callback_handler != 0L)
       {
          (adc_blk->adc_callback_handler) (adc_blk->adc_callback_parm,
                                           adc_blk->adc_conv_results,
                                           adc_blk->adc_active_channels,
                                           0);        // Invoke user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}
#endif

/******************************************************************************/
