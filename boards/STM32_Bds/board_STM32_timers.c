
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           board_STM32_timers.c
//
//
//  Common Logic for Timer and PWM support for STM32 MCUs.
//
//  Uses a combination of HAL_Library (mostly for configuration) and direct
//  register calls (main execution).
//
//  Specific chip dependences are mainly constrained to tables (GPIOs used
//  specific TIMx modules supported, etc). They are pulled in as #includes
//  based on MCU type.
//
//  In general, the use of #ifdefs within the code has been minimized, in
//  order to keep things read-able and maintainable.
//
//  History:
//    12/30/14 - Created for Industrial IoT OpenSource project.  Duq
//    04/30/15 - Added multi-channel PWM support.  WORKS.   Duq
//    07/03/15 - Merge in changes for Timer / PWM enhancements. Duq
//    07/16/15 - Reworked to provide better factoring. Duq
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

//*****************************************************************************
//*****************************************************************************
//               REAL  TIMER / PWM   Support           Physical Timers on chip
//*****************************************************************************
//*****************************************************************************
#include "user_api.h"                // pull in defs for User API calls

long  board_timerpwm_compute_prescalar (long period_val);    // Function protos
int   board_timerpwm_enable_clock (int module_id);
int   board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int   board_timerpwm_config_gpios (void *pwmblkp, int chan_id);

    unsigned long   tim3_TIM_rupts_seen = 0;

     //-------------------------------------------------------------------------
     // Global Variables needed to ensure consistency for ADC and DAC triggering
     //-------------------------------------------------------------------------
  extern uint16_t   _g_trigger_auser_api_id;   // User API id for the trigger
  extern uint16_t   _g_trigger_atmr_mmsmask;   // Associated Mask for TIM's  MMS
  extern uint16_t   _g_trigger_dtmr_mmsmask_1; // Chan 1 Associated Mask for TIM's  MMS
  extern uint16_t   _g_trigger_duser_api_id_1; //   "    Associated DAC trigger type
  extern uint16_t   _g_trigger_dtmr_mmsmask_2; // Chan 2 Associated Mask for TIM's  MMS
  extern uint16_t   _g_trigger_duser_api_id_2; //   "    Associated DAC trigger type


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
     //          _g_timer_typedef_handle[]    table/array       was _g_timer_module_handle
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

#if defined(__STM32F072__) || defined(STM32F072xB)
//                                         STM32 - F072  Nucleo
#include "STM32_F0/board_F0_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM14
#define  HAS_TIM15
#define  HAS_TIM16
#define  HAS_TIM17
#endif


#if defined(__STM32F091__) || defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_tables_F0_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM14
#define  HAS_TIM15
#define  HAS_TIM16
#define  HAS_TIM17
#endif


#if defined(__STM32F103__) || defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
#include "STM32_F1/board_F1_tables_timer.c"
#endif


#if defined(STM32F303xE)
//                                         STM32 - F303  Nucleo
#include "STM32_F3/board_F303_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM8
#define  HAS_TIM15
#define  HAS_TIM16
#define  HAS_TIM17
#define  HAS_TIM20
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo
#include "STM32_F3/board_F334_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM15
#define  HAS_TIM16
#define  HAS_TIM17
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401/F411  Nucleos
#include "STM32_F4/board_F4x1_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM5
#define  HAS_TIM10
#define  HAS_TIM11
#endif

#if defined(STM32F446xx)
//                                         STM32 - F446  Nucleo
#include "STM32_F4/board_F446_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM5
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM8
#define  HAS_TIM9
#define  HAS_TIM10
#define  HAS_TIM11
#define  HAS_TIM12
#define  HAS_TIM13
#define  HAS_TIM14
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "STM32_F7/board_F7_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM5
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM8
#define  HAS_TIM9
#define  HAS_TIM10
#define  HAS_TIM11
#define  HAS_TIM12
#define  HAS_TIM13
#define  HAS_TIM14
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo
#include "STM32_L0/board_L0_tables_timer.c"
#define  HAS_TIM2
#define  HAS_TIM6
#define  HAS_TIM21
#define  HAS_TIM22
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo
#include "STM32_L1/board_L1_tables_timer.c"
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM5
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM9
#define  HAS_TIM10
#define  HAS_TIM11
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
#include "STM32_L4/board_L4_tables_timer.c"
#define  HAS_TIM1
#define  HAS_TIM2
#define  HAS_TIM3
#define  HAS_TIM4
#define  HAS_TIM5
#define  HAS_TIM6
#define  HAS_TIM7
#define  HAS_TIM8
#define  HAS_TIM15
#define  HAS_TIM16
#define  HAS_TIM17
#endif


//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES and DEFINEs
//
//                                for
//
//                       PWM   and   OC/IC   Timers
//*****************************************************************************
//*****************************************************************************

              // flags for _g_pwm_module_status[] entries
#define  TMR_PWM_NORMAL_INIT      0x01  /* Module setup for normal output mode*/
#define  TMR_PWM_COMPLEMENT_INIT  0x08  /* Module setup for Complementary mode*/
#define  TMR_PWM_INTERRUPTS_USED  0x40  /* Module uses Timer interrupts */

              // flags for _g_tmpwm_channels_config[] entries
              //       and _g_tmpwm_channels_pwm[]
#define  TMR_CCR3N_CONFIGURED     0x80        /* Comple channel 3N configured */
#define  TMR_CCR2N_CONFIGURED     0x40        /* Chan 6 or Comple channel 2N configured */
#define  TMR_CCR1N_CONFIGURED     0x20        /* Chan 5 or Comple channel 1N configured */
#define  TMR_CCR4_CONFIGURED      0x10        /* Channel 4 configured for use */
#define  TMR_CCR3_CONFIGURED      0x08        /* Channel 3 configured for use */
#define  TMR_CCR2_CONFIGURED      0x04        /* Channel 2 configured for use */
#define  TMR_CCR1_CONFIGURED      0x02        /* Channel 1 configured for use */
#define  TMR_CCR0_CONFIGURED      0x01        /* Channel 0 configured for use */

#define   TIMER_IRQ_STARTED       0x01        /* Timer/PWM module IRQ setup, and Started */

                           //--------------------------------------------------------
                           // TIM_HandleTypeDef struct that is used at run-time.
                           // can be a combination of pre-defined
                           // _g_TIMxx_TimPwmHandle from board_XX_timer_tables.c
                           // and User App supplied TIM_HandleTypeDefs (Advanced)
                           //--------------------------------------------------------
    TIM_HandleTypeDef  *_g_timer_runtime_TIM_handle [MAX_TIMER+1] = { 0,0,0,0,0,0 };

                           //--------------------------------------------------------
                           // Bit mask status of each Timer Module (initialized, ...)
                           //--------------------------------------------------------
    char              _g_tmpwm_module_status [MAX_TIMER+1] = { 0,0,0,0,0,0 };

                           //--------------------------------------------------------
                           // Started status of each Timer Module (IRQ setup, Start issued)
                           //--------------------------------------------------------
    char              _g_timer_enable_status [MAX_TIMER+1] = { 0,0,0,0,0,0 };

                           //--------------------------------------------------------
                           // Bit Mask status of CCRs in each module (enabled, PWM, ...)
                           // One entry per timer module.
                           //--------------------------------------------------------
    unsigned char     _g_tmpwm_channels_config [MAX_TIMER+1] = { 0,0,0,0,0,0,0,0 };


                           //--------------------------------------------------------
                           // Bit Mask PWM status of CCRs in each module (Normal, Complementary,...)
                           // One entry per timer module.
                           //--------------------------------------------------------
    unsigned char     _g_tmpwm_channels_pwm [MAX_TIMER+1] = { 0,0,0,0,0 };


                           //--------------------------------------------------------
                           // Bit mask of which CCRs are using interrupts in each module.
                           // One entry per timer module.
                           //--------------------------------------------------------
    uint16_t          _g_timer_app_enabled_interrupts [MAX_TIMER+1] = { 0,0,0,0,0,0 };


                           //--------------------------------------------------------
                           // Interrupt Callback addresses. One entry per timer module.
                           //--------------------------------------------------------
TMR_CB_EVENT_HANDLER  _g_ptimer_callback [MAX_TIMER+1] = { 0,0,0,0,0,0,0,0,0,0 };
    void              *_g_ptimer_callback_parm [MAX_TIMER+1];

                           //--------------------------------------------------------
                           // Timer Prescalars used for each Timer module.
                           //--------------------------------------------------------
    uint32_t          _g_tmpwm_prescalars [MAX_TIMER+1]    = { 0,0,0,0,0,0 };


        //----------------------------------------------------------------------
        //     Global constants and Tables used to manage Timer/PWM Logic
        //----------------------------------------------------------------------
const unsigned char    _g_pwm_chan_mask [8]     // Bit masks used to track if a channel
                                                // is enabled and/or using COMPLE
                                  = {      0,         /* Channel 0 is unused */
                                      TMR_CCR1_CONFIGURED,  /* Channels 1-4 */
                                      TMR_CCR2_CONFIGURED,  /*   0x02  */
                                      TMR_CCR3_CONFIGURED,  /*   0x04  */
                                      TMR_CCR4_CONFIGURED,  /*   0x08  */
                                      TMR_CCR1N_CONFIGURED, /* Channels 1N-3N */
                                      TMR_CCR2N_CONFIGURED, /*   0x20  */
                                      TMR_CCR3N_CONFIGURED  /*   0x40  */
                                    };                      // there is no 4N

                           //-------------------------------------------------------
                           // List of HAL constants to specify interrupt mask for a CCR
                           //-------------------------------------------------------
const unsigned char   _g_tmpwm_CCR_rupt_flags [5]
                                  = { TIM_SR_UIF,        /* CCR0 aka Rollover */
                                      TIM_SR_CC1IF,      /* CCR1 interrupt    */
                                      TIM_SR_CC2IF,      /* CCR2 interrupt    */
                                      TIM_SR_CC3IF,      /* CCR3 interrupt    */
                                      TIM_SR_CC4IF       /* CCR4 interrupt    */
                                    };

                           //-------------------------------------------------------
                           // List of constants used by HAL to identify CCR channels
                           //-------------------------------------------------------
const unsigned int    _g_timer_channel_id [5]
                                   = { 0,       // unused
                                       TIM_CHANNEL_1,       /* 0x0000 */
                                       TIM_CHANNEL_2,       /* 0x0004 */
                                       TIM_CHANNEL_3,       /* 0x0008 */
                                       TIM_CHANNEL_4        /* 0x000C */
                                     };


                             //-------------------------------------------------
                             // _g_timer_base_channel_num
                             //
                             //     converts user API logical channel numbers, such as
                             //     TIMER_CHANNEL_1  / TIMER_CHANNEL_1_ALT1 / TIMER_CHANNEL_1_N
                             //     which are used to denote which Pin-Mux setting to
                             //     use for outputs that can be routed to different pins.
                             //
                             //     The _ALT1 / _ALT2 / _N / _N_ALT settings are
                             //     used to select the proper GPIO onto which to
                             //     route the signal.
                             //-------------------------------------------------
const unsigned char    _g_timer_base_channel_num []
                            = {   -1,               //  0 is not valid
                                TIM_CHANNEL_1,      //  1 TIMER_CHANNEL_1
                                TIM_CHANNEL_2,      //  2 TIMER_CHANNEL_2
                                TIM_CHANNEL_3,      //  3 TIMER_CHANNEL_3
                                TIM_CHANNEL_4,      //  4 TIMER_CHANNEL_4
                                  -1,               //  5 unused
                                  -1,               //  6 unused
                                  -1,               //  7 unused
                                TIM_CHANNEL_1,      //  8 TIMER_CHANNEL_1_ALT1
                                TIM_CHANNEL_2,      //  9 TIMER_CHANNEL_2_ALT1
                                TIM_CHANNEL_3,      // 10 TIMER_CHANNEL_3_ALT1
                                TIM_CHANNEL_4,      // 11 TIMER_CHANNEL_4_ALT1
                                TIM_CHANNEL_1,      // 12 TIMER_CHANNEL_1_ALT2
                                TIM_CHANNEL_2,      // 13 TIMER_CHANNEL_2_ALT2
                                TIM_CHANNEL_3,      // 14 TIMER_CHANNEL_3_ALT2
                                TIM_CHANNEL_4,      // 15 TIMER_CHANNEL_4_ALT2
                                TIM_CHANNEL_1,      // 16 TIMER_CHANNEL_1_N
                                TIM_CHANNEL_2,      // 17 TIMER_CHANNEL_2_N
                                TIM_CHANNEL_3,      // 18 TIMER_CHANNEL_3_N
                                  -1,               // 19 unused (no Channel 4N)
                                TIM_CHANNEL_1,      // 20 TIMER_CHANNEL_1_N_ALT1
                                TIM_CHANNEL_2,      // 21 TIMER_CHANNEL_2_N_ALT1
                                TIM_CHANNEL_3,      // 22 TIMER_CHANNEL_3_N_ALT1
                                  -1,               // 23 unused (no Channel 4N)
                                TIM_CHANNEL_1,      // 24 TIMER_CHANNEL_1_N_ALT2
                                TIM_CHANNEL_2,      // 25 TIMER_CHANNEL_2_N_ALT2
                                TIM_CHANNEL_3,      // 26 TIMER_CHANNEL_3_N_ALT2
                                TIM_CHANNEL_1,      // 27 TIMER_CHANNEL_1_N_ALT3
                                TIM_CHANNEL_2,      // 28 TIMER_CHANNEL_2_N_ALT3
                                TIM_CHANNEL_3,      // 29 TIMER_CHANNEL_3_N_ALT3
                                TIM_CHANNEL_1,      // 30 TIMER_CHANNEL_1_N_ALT4
                                TIM_CHANNEL_2,      // 31 TIMER_CHANNEL_2_N_ALT4

                                TIM_CHANNEL_1,      // 32 TIMER_CHANNEL_1_ALT3
                                TIM_CHANNEL_2,      // 33 TIMER_CHANNEL_2_ALT3
                                TIM_CHANNEL_3,      // 34 TIMER_CHANNEL_3_ALT3
                                TIM_CHANNEL_4,      // 35 TIMER_CHANNEL_4_ALT3
                                TIM_CHANNEL_1,      // 36 TIMER_CHANNEL_1_ALT4
                                TIM_CHANNEL_2,      // 37 TIMER_CHANNEL_2_ALT4
                                TIM_CHANNEL_3,      // 38 TIMER_CHANNEL_3_ALT4
                                TIM_CHANNEL_4,      // 39 TIMER_CHANNEL_4_ALT4
                              };


//*****************************************************************************
//*****************************************************************************
//                    COMMON   TIMER  and  PWM   Code
//
//                                for
//
//                       PWM   and   OC/IC   Timers
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//            COMMON  Utility  Routines   between   TIMERs  and  PWMs
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_timerpwm_channel_lookup
//
//           Converts an extended channel id: ALT1 / ALT2 / _N / N_ALT1 / N_ALT2
//           into the associated base channel number needed by the HAL_TIM_OC_xx
//           calls.
//
//           returned hal_channel_num is a bit encoded pattern (0x0000 - 0x000C)
//           used by the various HAL_TIM_OC_xx() and HAL_TIM_PWM_xx() calls
//
//           chan_index is a linear index 1 to 4, that is returned for any
//           operations that required straight indxed table lookups for
//           the base underlying channel.
//******************************************************************************
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index)
{
    int  hal_channel_num;

    if (chan_id < TIMER_CHANNEL_1 || chan_id  > TIMER_CHANNEL_3_N_ALT2)
       return (-1);                         // chan id is outside of valid range

    hal_channel_num = (int) _g_timer_base_channel_num [chan_id];

    *chan_index = (hal_channel_num >> 2) + 1;    // yields index of 1 - 4
    return (hal_channel_num);
}


//******************************************************************************
//  board_timerpwm_config_gpios
//
//            Configure the GPIO for Timer PWM, Output Compare, or Input Capture
//            by setting up associated ALTERNATE FUNCTION bits for Pin Muxing.
//
//            Also, if an ALT1/ALT2 channel id is requested, scan through the
//            table to find the matching entry.
//******************************************************************************
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    TMPWM_CHANNEL_BLK   *tmrblkp;
    uint8_t             user_chan_id;

    user_chan_id = (uint8_t) chan_id;             // co-erce to correct datatype
    tmrblkp = (TMPWM_CHANNEL_BLK*) pwmblkp;
    if (tmrblkp->user_channel_id != user_chan_id)
       {     // scan across the table looking for a matching ALT channel id
         while (tmrblkp->user_channel_id != user_chan_id)
           {
             if (tmrblkp->user_channel_id == user_chan_id)
                break;                                       // we found a match
             tmrblkp++;                           // step to next entry in table
             if (tmrblkp->user_channel_id == 0)
                return (ERR_PWM_CHANNEL_NUM_NOT_SUPPORTED);  // hit end of list
           }
       }

    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));   // clear struct

    GPIO_InitStruct.Pin       = tmrblkp->chan_gpio_pin;

// ??? !!! need to specify INPUT if for INPUT CAPTURE  !!! ???
//                             vvvvvvvvvvvvvvvvvvv
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = tmrblkp->chan_alt_func;  // set Pin Mux Alt Function code
    HAL_GPIO_Init (tmrblkp->chan_gpio_port, &GPIO_InitStruct);

// ??? in future,  do  pchanConfig. stuff as well ???

    return (0);
}



//*****************************************************************************
//  board_timerpwm_init
//
//         Initializes a Timer/PWM with correct mode, and initial period.
//
// parms:
//     Counter Type:  up/down/center-aligned, single shot. Valid values:
//        - TIMER_COUNT_UP
//        - TIMER_COUNT_DOWN
//        - TIMER_COUNT_UPDOWN
//        - TIMER_ONE_SHOT_COUNT_UP
//        - TIMER_ONE_SHOT_COUNT_DOWN
//
//     timer_clock_source:
//        -
//
//     Flags:  interrupts used, auto-prescale. Valid values:
//        - TIMER_AUTO_PRESCALE    automaticall calculate PSC value and pre-scale values
//        - PWM_COMPLEMENTARY_OUTPUTS
//
//     aller_htim_ptr:  caller supplied TIM_TypeDef
//        - if non-zero, caller has special processing requests, so is passing
//          in his own pre-setup TIM_Handle and Init structiure
//
//  NOTE:
//         HAL code analysis shows that there is almost no difference between
//         the HAL_TIM_OC_Init(), HAL_TIM_PWM_Init, and HAL_TIM_Base_Init().
//         The only difference is their invocations of differnt MspInit()
//         callbacks, which we do not use.
//
//         So except where there is a substantive code difference (mainly on
//         Config Channels), we utilize the HAL_TIM_OC_xx or HAL_TIM_Base_xx
//         to avoid importing redundant amounts of bloatware. Hence we avoid
//         using the (mostly redundant) HAL_TIM_PWM_xx calls except on Config Channels.
//*****************************************************************************
int  board_timerpwm_init (unsigned int module_id, int counter_type, long period_value,
                          int timer_clock_source, int flags,
                          TIM_HandleTypeDef *caller_htim_ptr) // extended support

{
    TIM_TypeDef         *timbase;
    TIM_HandleTypeDef   *hdltimer;
    uint32_t            counter_mode;
    long                prescalar_val;
    int                 rc;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated timer objects
    if (caller_htim_ptr != 0L)
       {   // caller wants to use _his_ SPI_TypeDef handle, not our internal one
         hdltimer = caller_htim_ptr;              // Use caller's TIM_HandelDef
       }
      else {    // use our internal TypeDef Handle for all calls
             hdltimer = (TIM_HandleTypeDef*) _g_timer_typedef_handle [module_id];
             if (hdltimer == 0L)
                return (ERR_TIMER_NUM_NOT_SUPPORTED);
           }
    memset (hdltimer, 0, sizeof(TIM_HandleTypeDef));   // clear struct

         // Save the TIM TypeDef handle to use during all runtime I/O processing
    _g_timer_runtime_TIM_handle [module_id]= hdltimer;

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    if (counter_type == TIMER_COUNT_UP)
       counter_mode = TIM_COUNTERMODE_UP;
       else if (counter_type == TIMER_COUNT_DOWN)
               counter_mode = TIM_COUNTERMODE_DOWN;
       else if (counter_type == TIMER_COUNT_UPDOWN)
               counter_mode = TIM_COUNTERMODE_CENTERALIGNED3;
       else if (counter_type == TIMER_ONE_SHOT_COUNT_UP)
               counter_mode = TIM_COUNTERMODE_UP;
       else if (counter_type == TIMER_ONE_SHOT_COUNT_DOWN)
               counter_mode = TIM_COUNTERMODE_DOWN;
       else if (counter_type == TIMER_INPUT_CAPTURE)
               ;
       else return (ERR_TIMER_INVALID_COUNTER_TYPE);

      //----------------------------------------------------------
      // If Timer was already initialized - ignore the second call
      //-----------------------------------------------------------
   if (_g_tmpwm_module_status [module_id] != 0)
      return (WARN_TIMER_WAS_ALREADY_INITIALIZED);

      //--------------------------------------------------------
      // Turn on clock for the associated TIMx module.
      //--------------------------------------------------------
    board_timerpwm_enable_clock (module_id);    // ensure Timer clock is turn on

    prescalar_val = 0;
#if (SCREWING_UP_DAC_TIMERS_BIG_TIME) // 06/26/15 - makes length of Sine/Triangle hugely long (seconds duration for 1 cycle)
#endif
    if (flags & TIMER_AUTO_PRESCALE)
       {
              //---------------------------------------------------------------------
              //                     perform   AUTO-PRE-SCALING
              // Compute any needed prescaler value to fit 16-bit TIMx counter clock.
              // Also adjust period_value to reflect effect of pre-scalar.
              //---------------------------------------------------------------------
         prescalar_val = board_timerpwm_compute_prescalar (period_value);
         if (prescalar_val > 0)
            period_value = (period_value / prescalar_val);
         _g_tmpwm_prescalars[module_id] = prescalar_val;  // save PRE-SCALAR for duty cycle calcs
       }

              //---------------------------------------------------------------------
              // Call HAL logic to configure the TIM module for usage.
              //---------------------------------------------------------------------
    if (counter_type >= TIMER_COUNT_UP && counter_type <= TIMER_ONE_SHOT_COUNT_DOWN)
       {    // Configure the TIMx peripheral for Timer Output
         hdltimer->Instance               = timbase;
         hdltimer->Init.Period            = (period_value - 1); // set # ticks needed for that frequency
         hdltimer->Init.Prescaler         = prescalar_val;
         hdltimer->Init.ClockDivision     = 0;
         hdltimer->Init.CounterMode       = counter_mode;
#if defined(STM32L053xx) || defined(STM32L152xE)
               // L0 / L1 do not support RepetitionCounter
#else
         hdltimer->Init.RepetitionCounter = 0;
#endif
         rc = HAL_TIM_Base_Init (hdltimer);
       }
      else
       {    // Configure the TIMx peripheral for Timer Input
         hdltimer->Instance               = timbase;
//       rc = HAL_TIM_IC_Init (hdltimer);    // count mode = Input capture

       }
    if (rc != HAL_OK)
       {    // got a Base Timer/PWM Initialization Error - bail
         return (ERR_PWM_MODULE_INITIALIZE_FAILED);
       }

        //-----------------------------------------------------------------------
        // denote module has been initialized, and if Complementary PWM is used
        //-----------------------------------------------------------------------
    if (flags & PWM_COMPLEMENTARY_OUTPUTS)
            // user wants to run a pair of channels in complementary mode
       _g_tmpwm_module_status [module_id] = TMR_PWM_COMPLEMENT_INIT;
       else _g_tmpwm_module_status [module_id] = TMR_PWM_NORMAL_INIT; // no complementAry mode

    return (0);                              // denote completed OK
}

                       // The rest of the Physical Timers logic

//*****************************************************************************
//  board_timerpwm_check_completed
//
//        Checks to see if the timer has expired / completed it's cycle.
//        Optionally, resets the completion flag if denoted in reset_flags parm.
//
//        This is frowned upon (callbacks should be used instead), but some
//        people still want to occassionally do polling on this stuff.
//
//        Returns:
//              True  (1) = Completed
//              False (0) = Busy
//*****************************************************************************
int  board_timerpwm_check_completed (unsigned int module_id, int mask_flags, int reset_flags)
{
    TIM_HandleTypeDef   *hdltimer;
    TIM_TypeDef         *timbase;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //-----------------------------------------------------------------------
       // Check Timer's UPDATE flag status, to see if a timer rollover occurred
       //-----------------------------------------------------------------------
    if (timbase->SR & TIM_SR_UIF)
       {      // yes, Timer has popped/rolled over
         if (reset_flags == 1)
            timbase->SR = ~(TIM_SR_UIF);
         return (1);                    // tell caller it popped
       }

//        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
//#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)
//             ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
// #define TIM_IT_UPDATE     ((uint32_t)0x00000001)  =  TIM_SR_UIF

    return (0);                         // Timer has not popped yet
}


//*****************************************************************************
//  board_timerpwm_config_channel
//
//         Configure a specific channel (1-4) on a specific PWM module (TIM1/2/..)
//
//         For TIM1, allow complmentary mode usage.
//*****************************************************************************

int  board_timerpwm_config_channel (unsigned int module_id, int chan_id,
                                    long initial_duty, int mode, int flags)
{
    TIM_OC_InitTypeDef  pchanConfig;      // Timer Output CCR Config
    TIM_HandleTypeDef   *hdltimer;
    TMPWM_CHANNEL_BLK   *pwmblkp;
    int                 rc;
    int                 chan_index;
    uint32_t            hal_channel_num;
    unsigned char       chan_mask;
    unsigned char       rupt_mask;

    if (module_id > MAX_TIMER)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated TIM module handle
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

//  if ((_g_tmpwm_module_status[modgen_id] & TMR_PWM_NORMAL_INIT) == 0)
//     return (ERR_PWM_MODULE_IN_COMPLEMENTARY); // need to use pwm_config_channel_pair

    if (mode < 0 || mode > TIMER_MODE_PWM)
       return (ERR_PWM_INVALID_TIMER_MODE);

    memset (&pchanConfig, 0, sizeof(pchanConfig));   // clear struct

       //-----------------------------------------------
       // do any needed Auto Pre-Scaling on duty_cycle
       //-----------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0  &&  (flags & TIMER_AUTO_PRESCALE))
       { initial_duty = (initial_duty / _g_tmpwm_prescalars[module_id]);
       }

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

    chan_mask = _g_pwm_chan_mask [chan_index];   // get associated CCR mask bits

       //--------------------------------------------------------------
       // turn on PWM flag if this is a PWM based module/channel
       //--------------------------------------------------------------
    if (mode >= TIMER_MODE_PWM)
       {     // Denote that this channel is running as a PWM on this module.
             // Turn associated mask bit on for this channel in PWM status flags.
         _g_tmpwm_channels_pwm [module_id] |= chan_mask;
       }

       //------------------------------------------------------------------
       //                Check if running as TIMER ONLY
       //
       // Note: user can elect to run the CCR in _Timer mode only_, and not
       //       send any output to the GPIOs, e.g. simple timer using CCR.
       //
       // _HOWEVER_ for usage as a triggering source (ADC trigger or DAC
       // trigger), we still to enable the output compare (OC) options,
       // such as SET/RESET, TOGGLE, ... because the OC outputs are used to
       // fire the internal HW flip-flops that drive the ADC/DAC triggers.
       //------------------------------------------------------------------

    if (flags & TIMER_CCR_TIMER_ONLY)
       {     // explicitly skip over GPIO config, but continue on to
             // configure OC options
//       pchanConfig.OCPolarity  = TIM_OCPOLARITY_LOW;
       }
      else
       {   //------------------------------------------------------------------
           //     CONFIGURE  CCR's  associated  GPIO  pin  into Timer/PWM mode.
           //
           // Configure Timer/PWM Channel's GPIO pin into Alternate function,
           // push-pull, 100MHz speed.
           //
           // Note: user can elect to run the CCR in _Trigger mode only_, and not
           //       not send _external_ output to the GPIOs, e.g. Trigger for ADC/DAC.
           //       In that case, we configure all CCRs settings but leave
           //       the external GPIOs disabled. The internal CCR wiring only
           //       drives the ADC or DAC with the CCR timer outputs/flip-flops.
           //------------------------------------------------------------------
        pwmblkp = (TMPWM_CHANNEL_BLK*) _g_tmrpwm_mod_channel_blk_lookup [module_id];
        rc = board_timerpwm_config_gpios (pwmblkp, chan_id); // configure assoc GPIO
        if (rc < 0)
           return (rc);     // must be "not supported channel number" on this module
       }

       //------------------------------------------------------------
       // prepare to configure the associated Timer/PWM channel CCR
       //------------------------------------------------------------
    memset (&pchanConfig, '\0', sizeof(pchanConfig));   // ensure is cleared out

    if (mode == TIMER_MODE_OC_PASSTHRU)
       pchanConfig.OCMode = TIM_OCMODE_TIMING;           // Output is frozen to whatever POLARITY is set to
       else if (mode == TIMER_MODE_OC_SET)
               pchanConfig.OCMode = TIM_OCMODE_ACTIVE;   // set pin Active when CCR reached
       else if (mode == TIMER_MODE_OC_RESET)
               pchanConfig.OCMode = TIM_OCMODE_INACTIVE; // set pin Inactive when CCR hit
       else if (mode == TIMER_MODE_OC_TOGGLE)
               pchanConfig.OCMode = TIM_OCMODE_TOGGLE;   // TOGGLE pin when CCR hit
       else if (mode == TIMER_MODE_PWM)
               pchanConfig.OCMode = TIM_OCMODE_PWM1;     // PWM_1 mode
       else if (mode == TIMER_MODE_PWM_INV)
               pchanConfig.OCMode = TIM_OCMODE_PWM2;     // PWM_2 mode
       else return (ERR_PWM_INVALID_TIMER_MODE);

    if (flags & TIMER_PIN_POLARITY_LOW)
       pchanConfig.OCPolarity = TIM_OCPOLARITY_LOW;      // Pin is LOW when Active (inverted)
       else pchanConfig.OCPolarity = TIM_OCPOLARITY_HIGH;// Pin is HIGH when Active

#if defined(STM32L053xx) || defined(STM32L152xE)
          // L0 and L1 do not support OCIdleState
#else
    pchanConfig.OCIdleState = TIM_OCIDLESTATE_SET;      // make flags option in future
#endif
    pchanConfig.OCFastMode  = TIM_OCFAST_DISABLE;
    pchanConfig.Pulse       = initial_duty;      //Set initial duty cycle for channel

// ??? where how is this flag ever set ??? should it be derived from GPIO pin lookup ? !!!  PROBABLY
    if (flags & PWM_COMPLEMENTARY_OUTPUTS)
       {    // user wants to run a pair of channels in complementary mode.
            // turn on the extra parms needed for Complementary support.
            // "N" alternate channel settings
// *** NEEDS MORE WORK - ALSO NEED TO ENABLE ASSOCIATE GPIO PINS  ??? !!!  05/19/15
         if (module_id != 1 || chan_index > 3)
            return (ERR_PWM_MODULE_NO_COMPLEMENTARY); // only TIM1 supports complementary
#if defined(TIM_OCNPOLARITY_HIGH)      // L0 not support
         pchanConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;   // Flags option in future
         pchanConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET; // Flags option in future
#endif
         _g_tmpwm_channels_pwm [module_id] |= _g_pwm_chan_mask [chan_index+4];  // turn on comple N bits
         chan_mask |= _g_pwm_chan_mask [chan_index+4];      // add in comple N bits
       }

       //-----------------------------------------------------------------------
       // Have HAL routines configure associated CCTL register bits for Channel
       //-----------------------------------------------------------------------
    if ((_g_tmpwm_channels_pwm [module_id] & chan_mask) != 0)
       rc = HAL_TIM_PWM_ConfigChannel (hdltimer, &pchanConfig,
                                       hal_channel_num); // config for PWM mode output
       else rc = HAL_TIM_OC_ConfigChannel (hdltimer, &pchanConfig,
                                       hal_channel_num); // config for OC mode output
    if (rc != HAL_OK)
       {
         return (ERR_PWM_CHANNEL_CONFIG_FAILED);  // Config Channel Error - bail
       }

       //--------------------------------------------------------------
       // turn on any needed interrupt flags for this CCR
       //--------------------------------------------------------------
    if (flags & TIMER_CCR_INTERRUPT_ENABLED)
       { rupt_mask = _g_tmpwm_CCR_rupt_flags [chan_index];
             // save what interrupts the User App wants to see
         _g_timer_app_enabled_interrupts [module_id] |= rupt_mask;
       }

       //----------------------------------------------------------------------
       // for this Timer/PWM module, set the bit corresponding for this channel
       // CCR to denote that it has been configured, and should be enabled.
       //----------------------------------------------------------------------
    _g_tmpwm_channels_config [module_id] |= chan_mask;

    return (0);               // denote successful configuration
}


//*****************************************************************************
//  board_timerpwm_config_trigger_mode                         TRIGGER
//
//         Sets up a Timer to provide Trigger Output (CCR or Rollover based
//         TRGO triggers), which is used to trigger either ADCs or DACs.
//
//         As per the STM32 Tech Ref, the _ADC Timer should already be enabled_
//         _before calling this routine_.
//
//         The trigger type must match what was specified on the adc_init()
//         or dac_init() call, otherwise an error is signalled.
//
// flags parm can have TIMER_TRIGGER_MODE set, indicating to ensure that the
//       CCR OUTMOD is set to some kind of toggle (MOD_3 / MOD_6/ MOD_7) state.
//*****************************************************************************
int  board_timerpwm_config_trigger_mode (unsigned int module_id, int adc_dac_id,
                                         int trigger_type,
                                         int flags)
{
    TIM_MasterConfigTypeDef   MasterSlaveConfig;
    TIM_HandleTypeDef         *hdltimer;
    TIM_TypeDef               *timbase;
    int                       ccr_bits;
    int                       channel_mask;
    uint16_t                  app_trigger_tmr_mmsmask;
    uint16_t                  app_trigger_user_api_id;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    memset (&MasterSlaveConfig, 0, sizeof(MasterSlaveConfig));   // clear struct

        //------------------------------------------------------------------
        //                Handle any ADC or DAC Triggering details
        //
        // Need to handle the case where user issued adc_init() _after_
        // timer_ADC_Trigger_Start(), in which case we need to also
        // turn on the associated Timer's MMS flags, to enable it to broadcast
        // trigger signals to the ADC.
        //
        // If GPIO EXTI triggering was used instead, need to ensure it is setup
        //----------------------------------------------------------------------
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    hdltimer->Instance = timbase;   // ensure Instance is setup: for Trigger
                                    // only scenarios it may not have been setup
    if (trigger_type != 0)
       {
         if (trigger_type  < ADC_TRIGGER_USER_APP || flags & TIMER_ADC_TRIGGER_MODE)
            {      //------------------------------
                   //  Process ADC based triggers
                   //------------------------------
                   // for ADCs, adc_dac_id = ADC module id
              board_adc_get_app_trigger_masks (adc_dac_id,
                                               &app_trigger_tmr_mmsmask,
                                               &app_trigger_user_api_id);
              if (app_trigger_tmr_mmsmask == 0)
                 return (ERR_TIMER_ADC_NOT_INITIALIZED);
              if (app_trigger_user_api_id != trigger_type)
                 return (ERR_TIMER_TRIGGER_MISMATCH);
                    //----------------------------------------------------------
                    // See if this is a CCR type trigger, and if so, see if the
                    // associated Timer's CCR channel was configured by the user.
                    // If not, we will auto-configure it with a default set of
                    // values. If the caller later calls timer_Enable_CCR_Output()
                    // it will just overlay our default CCR values.
                    //----------------------------------------------------------
              ccr_bits = (app_trigger_user_api_id & 0x07); // L.O. 3 bits = CCR
              channel_mask = _g_pwm_chan_mask [ccr_bits];  // get assoc config bit mask
              if (ccr_bits != 0)
                 {       // This trigger is using a Timer CCR
                  if ((_g_tmpwm_channels_config [module_id] & channel_mask) == 0)
                    {    //-----------------------------------------------------
                         // CCR has not been configured. Set it up with default
                         // values. Set default CCR value = Timer's period (ARR)
                         //-----------------------------------------------------
                      board_timerpwm_config_channel (module_id,    // timer
                                              (int)  ccr_bits,     // channel
                                              (long) timbase->ARR, // Period->CCR
                                              (int)  TIMER_MODE_OC_TOGGLE,
                                              (int)  (TIMER_CCR_TIMER_ONLY) );
                    }
                 }
                  // Then setup Timer to issue a TRG0 event whenever the Timer
                  // performs the requested Update or CCR event
              MasterSlaveConfig.MasterOutputTrigger = app_trigger_tmr_mmsmask;
              MasterSlaveConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
              HAL_TIMEx_MasterConfigSynchronization (hdltimer, &MasterSlaveConfig);
            }
           else if (trigger_type == ADC_TRIGGER_GPIO_PIN)
                   {       // process ADC trigger caused by GPIO
// ??? TBD
                     return (0);     // no MMS update needed for EXTI GPIO
                   }
           else if (trigger_type == DAC_TRIGGER_GPIO_PIN)
                   {       // process DAC trigger caused by GPIO
// ??? TBD
                     return (0);     // no MMS update needed for EXTI GPIO
                   }
           else if (trigger_type >= DAC_TRIGGER_TIMER_2 && trigger_type < DAC_TRIGGER_GPIO_PIN)
                   {    //------------------------------
                        //   Process DAC triggers
                        //------------------------------
#if defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F411xE)
                        //******************************************************
                        //  Handle MCUs that do not have ang native DAC support
                        //******************************************************
                     return (ERR_DAC_NOT_SUPPORTED_ON_THIS_MCU); //no native DAC
#else
                        // for DACs, adc_dac_id = DAC _channel_ id
                     board_dac_get_app_trigger_masks (adc_dac_id,
                                                      &app_trigger_tmr_mmsmask,
                                                      &app_trigger_user_api_id);
                     if (app_trigger_tmr_mmsmask == 0)
                        return (ERR_TIMER_DAC_NOT_INITIALIZED);
                     if (app_trigger_user_api_id != trigger_type)
                        return (ERR_TIMER_TRIGGER_MISMATCH);
                       //-------------------------------------------------------
                       // See if this is a CCR type trigger, and if so, see if the
                       // associated Timer's CCR channel was configured by the user.
                       // If not, we will auto-configure it with a default set of
                       // values. If the caller later calls timer_Enable_CCR_Output()
                       // it will just overlay our default CCR values.
                       //-------------------------------------------------------
                     ccr_bits     = (app_trigger_tmr_mmsmask & 0x07); // L.O. 3 bits = CCR
                     channel_mask = _g_pwm_chan_mask [ccr_bits];  // get assoc config bit mask
                     if (ccr_bits != 0)
                        if ((_g_tmpwm_channels_config [module_id] & channel_mask) == 0)
                          {  //-------------------------------------------------
                             // Has not been configured. Set it up with default
                             // values. Set default CCR value = Timer's period (ARR)
                             //-------------------------------------------------
                            board_timerpwm_config_channel (module_id,
                                                     (int) ccr_bits,
                                                    (long) timbase->ARR,
                                                     (int) TIMER_MODE_OC_SET_RESET,
                                                     (int) (TIMER_CCR_TIMER_ONLY) );
                          }
                        // Then setup Timer to issue a TRG0 event whenever the Timer
                        // performs the requested Update or CCR event
                     MasterSlaveConfig.MasterOutputTrigger = app_trigger_tmr_mmsmask;
                     MasterSlaveConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
                     HAL_TIMEx_MasterConfigSynchronization (hdltimer, &MasterSlaveConfig);
#endif
                   }
           else return (ERR_TIMER_INVALID_TRIGGER_TYPE);
       }

    return (0);              // denote completed ok
}


//*****************************************************************************
//  board_timerpwm_disable
//
//            Turns off a timer/pwm, and disables any interrupts for it.
//            (e.g. in preparation for Stopping or Reconfiguring a motor, ...)
//*****************************************************************************
int  board_timerpwm_disable (unsigned int module_id, int flags)
{
    TIM_HandleTypeDef   *hdltimer;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

      // Turn off the Timer Enabled flag
    _g_timer_enable_status [module_id] &= ~(TIMER_IRQ_STARTED);

// ??? TBD   stop every PWM channel that is enabled on this module    ??? !!!

    if (_g_tmpwm_module_status [module_id] & TMR_PWM_INTERRUPTS_USED)
       HAL_TIM_Base_Stop_IT (hdltimer);    // Disable the timer and interrupts
       else HAL_TIM_Base_Stop (hdltimer);  // Disable the Timer


// ??? !!! Disable before/after OC Timer_Disable ??? !!!  FUTURE
// ???     associated Compare/Capture OC elements must also be disabled !!!
//  HAL_TIM_OC_Stop_IT (hdltimer, Channel);
// ??? must associated Compare/Capture OC elements also be stopped ?
//  HAL_TIM_IC_Stop_IT (hdltimer, Channel);  // input capture

    return (0);                         // denote completed OK
}



//*****************************************************************************
//  board_timerpwm_enable
//
//         Enables and Starts the timer/PWM.
//
//         Sets up any desired Timer interrupt flags, then enables the Timer/PWM.
//         Saves optional callback function address to call when interrupt occurs.
//
//  PARMS:
//      interrupt_flags  interrupts used. Valid values:
//       - TIMER_PERIOD_INTERRUPT_ENABLED
//*****************************************************************************

int  board_timerpwm_enable (unsigned int module_id, int interrupt_flags)
{
    TIM_HandleTypeDef  *hdltimer;
    int                irqn;
    int                rc;
    int                app_ccr_rupts;
    unsigned char      channels_config;
    unsigned char      pwm_config;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

// 07/26/15: CAUTION: if the Timer was already previously enabled
//           the HAL code will blow up into a Hard Fault if we re-do
//           the Set_IRQ call and the __HAL_TIM_ENABLE() call in HAL_TIM_Base_Start()

       //-----------------------------------------------------------------------
       // If user requested a Rollover interrupt,  OR  setup interrupts on a CCR
       // then enable the necessary NVIC entry
       //-----------------------------------------------------------------------
                   // find and set associated NVIC IRQ value
    irqn = _g_TMPWM_IRQ_table [module_id];
    if ((interrupt_flags & TIMER_PERIOD_INTERRUPT_ENABLED)
      || _g_timer_app_enabled_interrupts[module_id])     // configured CCR rupts
       {
             //------------------------------------------------
             // If this timer was already started, and/or IRQ already setup
             // re-executing this logica will cause HAL_timxxx() code
             // to drop into a Hard Fault. So check if already enabled
             // and hand back a waring to the User App that it was already started.
             //------------------------------------------------
         if (_g_timer_enable_status [module_id] &= TIMER_IRQ_STARTED)
            return (WARN_TIMER_WAS_ALREADY_STARTED);

             // denote this module is running with Interrupts
         _g_tmpwm_module_status [module_id] |= TMR_PWM_INTERRUPTS_USED;
         if (interrupt_flags & TIMER_PERIOD_INTERRUPT_ENABLED)
            {     //------------------------------------------------------------
                  // set assoc Period ROLLOVER/Update flag for the overall Timer
                  //------------------------------------------------------------
              _g_timer_app_enabled_interrupts[module_id] |= TIM_SR_UIF;
#if defined(HAS_TIM20)
              if (module_id == 20)
                 {     // TIM20 has a separate IRQn for ROLLOVERs/Updates
                   NVIC_EnableIRQ (TIM20_UP_IRQn);              // Enable NVIC
                   NVIC_SetPriority (TIM20_UP_IRQn, 1);
                 }
#endif
#if defined(HAS_TIM8)
              if (module_id == 8)
                 {          // TIM8 has a separate IRQn for ROLLOVERs/Updates
   #if defined(STM32F303xE) || defined(STM32F303xC) || defined(STM32L476xx)
                            // just to be annoying, F3_03 uses diff NVIC id
                   NVIC_EnableIRQ (TIM8_UP_IRQn);                // Enable NVIC
                   NVIC_SetPriority (TIM8_UP_IRQn, 1);
   #else
                   NVIC_EnableIRQ (TIM8_UP_TIM13_IRQn);          // Enable NVIC
                   NVIC_SetPriority (TIM8_UP_TIM13_IRQn, 1);
   #endif
                 }
               else if (module_id == 1)
                 {          // TIM1 has a separate IRQn for ROLLOVERs/Updates
   #if defined(STM32F303xE) || defined(STM32F303xC) || defined(STM32L476xx)
                            // just to be annoying, F3_03 uses diff NVIC id
                    NVIC_EnableIRQ (TIM1_UP_TIM16_IRQn);         // Enable NVIC
                    NVIC_SetPriority (TIM1_UP_TIM16_IRQn, 1);
   #else
                    NVIC_EnableIRQ (TIM1_UP_TIM10_IRQn);         // Enable NVIC
                    NVIC_SetPriority (TIM1_UP_TIM10_IRQn, 1);
   #endif
                 }
               else
#elif defined(HAS_TIM10)
              if (module_id == 1)
                 {          // TIM1 has a separate IRQn for ROLLOVERs
  #if ! defined(STM32L152xE)
                    NVIC_EnableIRQ (TIM1_UP_TIM10_IRQn);         // Enable NVIC
                    NVIC_SetPriority (TIM1_UP_TIM10_IRQn, 1);
  #endif
                 }
               else
#endif
                {           // handle all the other timers ROLLOVERs
                            // find and set associated NVIC IRQ value
                    NVIC_EnableIRQ (irqn);                       // Enable NVIC
                    NVIC_SetPriority (irqn, 1);
                }
            }

           else {         //-----------------------------------------------
                          // handle all CCRs and TIM1/TIM8 CCRs conditions
                          // find and set associated NVIC IRQ value
                          //-----------------------------------------------
                     NVIC_EnableIRQ (irqn);                      // Enable NVIC
                     NVIC_SetPriority (irqn, 1);
                }

                   //---------------------------------------------------
                   //  Enable the Timer/PWM to start, _with_ Interrupts
                   //---------------------------------------------------
          HAL_TIM_Base_Start_IT (hdltimer);
          _g_timer_enable_status [module_id] |= TIMER_IRQ_STARTED;
        }

       else {      // No interrupts - just start the timer
                   // First check if it was already started. Bail if it already was
              if (_g_timer_enable_status [module_id] &= TIMER_IRQ_STARTED)
                 return (WARN_TIMER_WAS_ALREADY_STARTED);
                   //------------------------------------------------
                   // Enable Timer/PWM to start,  _no_ interrupts
                   //------------------------------------------------
              HAL_TIM_Base_Start (hdltimer);
              _g_timer_enable_status [module_id] |= TIMER_IRQ_STARTED;
            }

       //------------------------------------------------------------------------
       //                           CCR Startup
       // Startup each Timer/PWM channel that has been configured for this module
       //
       // After the main timer has been started, walk through and enable
       // each CCR requested by the user app. This will turn on the Output at
       // the GPIO pin, and turn on the associated CCR interrupt if requested.
       // Note: there is no substanitive difference between HAL_TIM_PWM_Start()
       //       and HAL_TIM_OC_Start()
       //------------------------------------------------------------------------
    channels_config = _g_tmpwm_channels_config [module_id];        // get local copy of CCR configed flags
    pwm_config      = _g_tmpwm_channels_pwm [module_id];
    app_ccr_rupts   = _g_timer_app_enabled_interrupts [module_id]; // get local copy of CCR interrupts
    if (channels_config  &  TMR_CCR1_CONFIGURED)
       {     // start channel 1, and optionally, channel 1N
         if (app_ccr_rupts & TIM_SR_CC1IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC1);      // Enable CCR1 rupt
            }
         if (pwm_config & TMR_CCR1_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_1);  // start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_1);
#if defined(HAL_TIMEx_PWMN_Start)      // L0 not support complementary
         if (channels_config  &  TMR_CCR1N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in (PWM) complementary mode - turn on channel 1N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_1);
            }
#endif
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);        // start failed - bail
       }

    if (channels_config  &  TMR_CCR2_CONFIGURED)
       {     // start channel 2, and optionally, channel 2N
         if (app_ccr_rupts & TIM_SR_CC2IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC2);  // Enable CCR2 rupt
            }
         if (pwm_config & TMR_CCR2_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_2);  // Start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_2);
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
#if defined(HAL_TIMEx_PWMN_Start)      // L0 not support complementary
         if (channels_config  &  TMR_CCR2N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in complementary mode - turn on channel 2N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_2);
            }
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
#endif
       }

    if (channels_config  &  TMR_CCR3_CONFIGURED)
       {     // start channel 3, and optionally, channel 3N
         if (app_ccr_rupts & TIM_SR_CC3IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC3);  // Enable CCR3 rupt
            }
         if (pwm_config & TMR_CCR3_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_3);  // Start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_3);
#if defined(HAL_TIMEx_PWMN_Start)      // L0 not support complementary
         if (channels_config  &  TMR_CCR3N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in complementary mode - turn on channel 3N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_3);
            }
#endif
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
       }

    if (channels_config  &  TMR_CCR4_CONFIGURED)
       {     // start channel 4
         if (app_ccr_rupts & TIM_SR_CC4IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC4);  // Enable CCR4 rupt
            }
         if (pwm_config & TMR_CCR4_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_4);
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_4);
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail

         // Channel 4 does _NOT_ have a complementary output (no channel 4N) !
       }

    return (0);                         // denote completed OK
}

//#endif                                  //  USES_PWM  ||  USES_TIMER


//#if defined(USES_TIMER) || defined(USES_PWM)

//*****************************************************************************
//  board_timerpwm_enable_CCR_output
//
//        Configures and enables Timer Output Compare wave generation on the
//        requested channel, and its associated GPIO pin and CCR.
//
//    Parameters:
//        initial_duty - specifies the associated "duty cycle" for the pin's
//                       CCR register to be set to
//
//        action_flags:
//          - TIMER_ACTION_TOGGLE     toggle the OC GPIO output every time
//                                    the CCR value reached,
//          - TIMER_ACTION_GO_ACTIVE  wait till CCR value reached, then drive
//                                    the OC GPIO output high
//                                    (and leave it there forever until reset)
//          - TIMER_ACTION_PULSE      similar to the above
//          - TIMER_ACTION_GO_INACTIVE  wait till CCR value reached, then drive
//                                    the OC GPIO output low
//                                    (and leave it there forever until reset)
//          - TIMER_PIN_POLARITY_HIGH the GPIO pin goes high when CCR hit
//          - TIMER_PIN_POLARITY_LOW  the GPIO pin goes low when CCR hit (Inverted output)
//
//        interrupt_flags:
//          - TIMER_NO_INTERRUPT     no interrupts are signalled when CCR value
//                                   is reached.
//          - TIMER_ENABLE_CCR_INTERRUPTS   an interrupt is signalled when CCR
//                                   value hit is reached. It will invoke the
//                                   callback_function that was provided on the
//                                   board_timerpwm_enable() call. It will pass
//                                   to the callback function the callback_parm
//                                   and a TIMER_xxxx_INTERRUPT flag.
//*****************************************************************************
int  board_timerpwm_enable_CCR_output (unsigned int module_id, int chan_id,
                                       long initial_duty,
                                       int action_flags, int extended_flags)
{
    int         mode;
    int         rc;
    int         flags;

       // tweak parms to match board_timerpwm_config_channel() format
    mode   = action_flags   & 0x000F;   // extract mode bits only
    flags  = extended_flags & 0xFFF0;
    flags |= (action_flags  & 0x0FF0);  // move over to extended flags

       // all heavy lifting is done by board_timerpwm_config_channel()
    rc = board_timerpwm_config_channel (module_id,
                                        chan_id,
                                        initial_duty,
                                        mode,  flags);

    return (rc);                      // denote worked successfully
}



//*****************************************************************************
//  board_timerpwm_get_current_value
//
//         Gets the current count value (CNT) in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_current_value (unsigned int module_id)
{
    TIM_TypeDef      *timbase;
    long             curr_count;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    curr_count = timbase->CNT;      // get currnet COUNT value

       //---------------------------------------------------------------
       // re-adjust COUNT value based on pre-scalar to get user's view
       //---------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_count = (curr_count * _g_tmpwm_prescalars[module_id]);

    return (timbase->CNT);          // pass back adjusted current counter value
}


//*****************************************************************************
//  board_timerpwm_get_CCR_capture_value
//
//         Gets the recorded input capture value in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_CCR_capture_value (unsigned int module_id, int CCR_channel_id)
{
    TIM_HandleTypeDef   *hdltimer;
    long                cap_value;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    cap_value = HAL_TIM_ReadCapturedValue (hdltimer, CCR_channel_id);

// ??? !!! must I adjust the value based on pre-scalar used ??? !!!

    return (cap_value);             // passback capture value
}


//*****************************************************************************
//  board_timerpwm_get_duty_cycle
//
//        Get the Timer/PWM's current duty cycle.      Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_duty_cycle (unsigned int module_id, int chan_id)
{
    TIM_TypeDef      *timbase;
    long             curr_duty;
    int              hal_channel_num;
    int              chan_index;

    if (module_id > MAX_TIMER)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

    switch (chan_index)
      { case 1:
              curr_duty = timbase->CCR1;
              break;
        case 2:
              curr_duty = timbase->CCR2;
              break;
        case 3:
              curr_duty = timbase->CCR3;
              break;
        case 4:
              curr_duty = timbase->CCR4;
              break;
        default:
                // this must be one of the N channels -
                // it does _NOT_ have its own CCRxN.  It relies on CCR1/1/3/4
              return (ERR_PWM_CHANNEL_COMPLE_CHAN_NO_CCR);
      }

       //-------------------------------------------------------------
       // re-adjust duty value based on pre-scalar to get user's view
       //-------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_duty = (curr_duty * _g_tmpwm_prescalars[module_id]);

    return (curr_duty);
}


//*****************************************************************************
//  board_timerpwm_get_period
//
//         Gets the period value (ARR) being used by the Timer/PWM.
//         Used for monitoring, etc
//*****************************************************************************
long  board_timerpwm_get_period (unsigned int module_id)
{
    TIM_TypeDef      *timbase;
    long             curr_period;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    curr_period = timbase->ARR;     // get currnet PERIOD value

       //---------------------------------------------------------------
       // re-adjust period value based on pre-scalar to get user's view
       //---------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_period = (curr_period * _g_tmpwm_prescalars[module_id]);

    return (curr_period);          // pass back adjusted current period value
}


//*****************************************************************************
//  board_timerpwm_get_handle
//
//         debugging hook used during board bring up to test new MCUs Timer/PWM.
//*****************************************************************************

TIM_HandleTypeDef *board_timerpwm_get_handle (unsigned int module_id)
{
    TIM_HandleTypeDef   *hdltimer;

       // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];

    return (hdltimer);               // return HAL related SPI handle
}


//*****************************************************************************
//  board_timerpwm_reset_CCR_output
//
//        Re-Configures and shuts off Timer Output Compare wave generation on
//        the requested channel.
//
//        This is used to turn off a CCR GPIO pin that was triggerred to a
//        constant value (e.g. reset a one-shot, in preparation for another
//        action).
//
//        This function is mainly used for OC modes:
//          - TIMER_ACTION_GO_ACTIVE  which will drive the OC output high
//                                    (and leave it there forever until reset)
//          - TIMER_ACTION_PULSE
//          - TIMER_ACTION_GO_INACTIVE  which will drive the OC output low
//                                    (and leave it there forever until reset)
//
//        flags - future use
//*****************************************************************************
int  board_timerpwm_reset_CCR_output (unsigned int module_id, int chan_num, int flags)
{
    uint32_t    timer_base_addr;
    int         mode;

    if (module_id >= MAX_TIMER)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > TMRPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);


#if (MSP432_CODE)      //  !!! NEED TO CREATE EQUIV FOR STM32  ??? !!!

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

    mode = OUTMOD_5;                    // config CCR to shutoff GPIO   or use OUTMOD_0 ?
    switch (chan_num)
       { case 1:                        //   TAn  CCR1
               HWREG16(timer_base_addr + OFS_TA0CCTL1)  = mode;
               break;
         case 2:                        //   TAn  CCR2
               HWREG16(timer_base_addr + OFS_TA0CCTL2) = mode;
               break;
         case 3:                        //   TAn  CCR3
               HWREG16(timer_base_addr + OFS_TA0CCTL3) = mode;
               break;
         case 4:                        //   TAn  CCR4
               HWREG16(timer_base_addr + OFS_TA0CCTL4) = mode;
               break;
         case 5:                        //   TAn  CCR5
               HWREG16(timer_base_addr + OFS_TA0CCTL5) = mode;
               break;
         case 6:                        //   TAn  CCR6
               HWREG16(timer_base_addr + OFS_TA0CCTL6) = mode;
               break;
       }
#endif

    return (0);                      // denote worked successfully
}


//*****************************************************************************
//  board_timerpwm_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when any Timer rollover or CCR interrupts occur.
//*****************************************************************************
int  board_timerpwm_set_callback (unsigned int module_id,
                                  TMR_CB_EVENT_HANDLER callback_function,
                                  void *callback_parm)
{
    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts (CCRs or Rollover)
       //-------------------------------------------------------------------
    _g_ptimer_callback [module_id]      = callback_function;
    _g_ptimer_callback_parm [module_id] = callback_parm;

    return (0);                 // denote completed OK
}



//*****************************************************************************
//  board_timerpwm_set_channel_output
//
//         Set polarity of Timer/PWM channel output (start high or low)
//*****************************************************************************
int  board_timerpwm_set_channel_output (unsigned int module_id, int chan_id,
                                        int output_mode, int flags)
{
    TIM_HandleTypeDef   *tmrpwm_mhdl;
    int                 rc;
    int                 hal_channel_num;
    int                 chan_index;

    if (module_id > MAX_TIMER)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed a bad chan_id value

            // get ptr to associated TIM module handle
    tmrpwm_mhdl = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [module_id];
    if (tmrpwm_mhdl == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);


//   ???   TBD



    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_set_dead_time
//
//         Set the PWM's dead time between complementary PWM
//         arrangements.  Use to avoid "shoot through" on the
//         output FETs when working with Power or Motor H-bridges.
//*****************************************************************************

int  board_timerpwm_set_dead_time (unsigned int module_id, int rising_edge,
                                   int falling_edge)
{
    TIM_TypeDef         *timbase;
    int                 rc;
    uint32_t            scaleduty;

#if defined(STM32L152xE)
    return (ERR_PWM_MODULE_NO_DEADTIME);
#else
    if (module_id != 1 && module_id != 8)
       return (ERR_PWM_MODULE_NO_DEADTIME);  // Only TIM1/TIM8 support Deadtime

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    timbase->BDTR &= ~(0xFF00);               // clear out any previous DT value
    timbase->BDTR |= (rising_edge & 0x00FF);  // set 8 bit deadtime

       // trailing edge is ignored, because STM32 F7 TIM1 only supports a
       // single deadtime value, that is used on both rising and falling edges.

    return (0);                // denote completed successfully
#endif

}


//******************************************************************************
//  board_timerpwm_set_duty_cycle
//
//        Set the Timer/PWM's duty cycle, aka
//        sets a CCR (compare count register) on the specified timer.
//
//        This is used to set different "duty cycles" for wave form values
//        being generated by a Timer, e.g. CCR1 might generate a 25 % duty
//        cycle on channel 1 pin, 33 % on channel 2 pin, and 50 % on chan 3 pin.
//
//        The flags parms is used to specify if a CCR interrupt should be
//        generated when the CCR compare value is hit by the Timer/PWM.
//******************************************************************************
int  board_timerpwm_set_duty_cycle (unsigned int module_id, int chan_id,
                                    long ccr_duty_cycle, int flags)
{
    TIM_TypeDef         *timbase;
    int                 hal_channel_num;
    int                 chan_index;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

       //----------------------------------------------
       // do any needed Auto Pre-Scaling on duty_cycle
       //----------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0 && (flags & TIMER_AUTO_PRESCALE))
       { ccr_duty_cycle = (ccr_duty_cycle / _g_tmpwm_prescalars[module_id]);
       }

    switch (chan_index)
      {
        case 1:
               timbase->CCR1 = ccr_duty_cycle;
               break;
        case 2:
               timbase->CCR2 = ccr_duty_cycle;
               break;
        case 3:
               timbase->CCR3 = ccr_duty_cycle;
               break;
        case 4:
               timbase->CCR4 = ccr_duty_cycle;
               break;
      }

    return (0);                         // denote completed OK
}


//*****************************************************************************
//  board_timerpwm_set_period
//
//         Sets the Timer/PWM to use a new period value
//*****************************************************************************
int  board_timerpwm_set_period (unsigned int module_id, long new_period_value, int flags)
{
    TIM_TypeDef         *timbase;
    uint32_t            period_val;
    uint32_t            prescalar_val;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    prescalar_val = 0;
    period_val    = new_period_value;

    if (flags & TIMER_AUTO_PRESCALE)
       {       //-------------------------------------------
               //  do any needed Auto Pre-Scaling on new period
               //-------------------------------------------
               // Compute any needed prescaler value to fit 16-bit TIMx counter clock
         prescalar_val = board_timerpwm_compute_prescalar (period_val);
         if (prescalar_val > 0)
            period_val = (period_val / prescalar_val);
         _g_tmpwm_prescalars[module_id] = prescalar_val; // save for duty cycle PRE-SCALAR calcs
       }
      else prescalar_val = _g_tmpwm_prescalars[module_id];  // use any explicitly set pre-scalar

    timbase->ARR = period_val;

    timbase->PSC = prescalar_val;

    return (0);                         // denote completed OK
}



//*****************************************************************************
//  board_timerpwm_set_phase
//
//         Set the PWM's phase between complementary PWM
//         arrangements.
//*****************************************************************************

int  board_timerpwm_set_phase (unsigned int module_id, int chan_num, long phase_offset)
{
#if defined(HAS_TIM8) || defined(HAS_TIM1)
                    // ??? RESEARCH  doesn't TIM1/TIM8 support a poor man's phase shift ?
                    //                   e.g. F3_03 without HRTIM module ?
   return (ERR_PWM_MODULE_NO_PHASE);

#else
   return (ERR_PWM_MODULE_NO_PHASE);    // STM32 F1, F0, L1, L0 do NOT support
                                        // phase shifting.
#endif

}


//*****************************************************************************
//  board_timerpwm_set_prescalar
//
//          Sets the pre-scalar value to use
//          when any Timer rollover or CCR interrupts occur.
//*****************************************************************************
int  board_timerpwm_set_prescalar (unsigned int module_id, long prescalar_val, int flags)
{
    TIM_TypeDef         *timbase;

    if (module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       //-------------------------------------------------------------------
       // Save the pre-scalar value, then load it into PSC
       //-------------------------------------------------------------------
    _g_tmpwm_prescalars[module_id] = prescalar_val;

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];

    timbase->PSC = prescalar_val;      // load it into PSC prescalar reg

    return (0);                        // denote completed OK
}



//******************************************************************************
//                           TIMER / PWM   ISRs
//
//   For those platforms that do not support TIM8/TIM13/TIM14/etc, the linker
//   will automatically discard those routines that are not used.
//   So defining them all here in one place has no negative effect.
//******************************************************************************

void  TIM_Common_IRQHandler(TIM_TypeDef *TIMbase, int tim_index);
void  TIM1_UP_TIM10_IRQHandler(void);  // -- CAUTION SHARED BETWEEN TIM1 and TIM10
void  TIM1_CC_IRQHandler(void);
void  TIM2_IRQHandler(void);
void  TIM3_IRQHandler(void);
void  TIM4_IRQHandler(void);
void  TIM5_IRQHandler(void);
void  TIM6_IRQHandler(void);
void  TIM7_IRQHandler(void);
void  TIM6_DAC_IRQHandler (void);
void  TIM8_CC_IRQHandler (void);
void  TIM1_BRK_TIM9_IRQHandler(void);
void  TIM1_BRK_TIM10_IRQHandler(void);
void  TIM1_TRG_COM_TIM11_IRQHandler(void);
void  TIM8_BRK_TIM12_IRQHandler(void);
void  TIM8_UP_TIM13_IRQHandler(void);  // -- CAUTION SHARED BETWEEN TIM8 and TIM13
void  TIM8_TRG_COM_TIM14_IRQHandler(void);
void  TIM12_IRQHandler (void);
void  TIM13_IRQHandler (void);
void  TIM14_IRQHandler (void);
void  TIM15_IRQHandler (void);
void  TIM16_IRQHandler (void);
void  TIM17_IRQHandler (void);
void  TIM20_IRQHandler (void);
void  TIM21_IRQHandler (void);
void  TIM22_IRQHandler (void);


//******************************************************************************
// TIM_Common_IRQHandler
//
//         To minimize redundant logic in each IRQ handler, we just route every
//         Timer IRQ to here, passing in the relevant parms (TIMx base, ...).
//
//         Note: on ST processors, unused interrupt flags may be set on
//               even though we are not using them, or the app is ignoring them
//               for callbacks. So we have to first filter what interrupts we
//               are going to look at based on User App's interrupt_flag
//               settings on timer_Enable() and timer_Enable_CCR_Output(),
//               and ignore the rest.
//******************************************************************************
volatile uint16_t      temp_hack_SR;
volatile uint16_t      app_rupts_mask2;

void  TIM_Common_IRQHandler (TIM_TypeDef *TIMbase, int tim_index)
{
    register uint16_t  app_rupts_mask;
    int                interrupt_type;
    TIM_HandleTypeDef  *timHandle;

       //--------------------------------------------------------------
       // get a local copy of what interrupts the User App wants
       // to be called back on. In optimized compilers, this should
       // get loaded into a local register.
       //--------------------------------------------------------------
  app_rupts_mask  = _g_timer_app_enabled_interrupts [tim_index];
app_rupts_mask2   = app_rupts_mask;
temp_hack_SR = (uint16_t) TIMbase->SR;
    app_rupts_mask &= (uint16_t) TIMbase->SR; // Mask the app specified interrupts
                                        // against interrupt status register SR.
                                        // The result is those interrupts the
                                        // app is interested in/configed for

       // Handle timing window, where App requested Timer/PWM Disable, but there
       // were still some interrupts left in the pipe.
    if ((TIMbase->CR1 & TIM_CR1_CEN) == 0)
       {    // Timer/PWM is Disabled. Clear any remaining interrupts
         TIMbase->SR = ~(TIMbase->SR);
         return;                               // then bail out
       }

    if (app_rupts_mask == 0)
       {   // we got an interrupt, but have no active config/setup for that CCR.
           // Must have been a private deal/setup by user app, or timing window.
           // Call HAL Library to handle it, and possibly invoke user callback.
        timHandle = (TIM_HandleTypeDef*) _g_timer_runtime_TIM_handle [tim_index];
        if (timHandle != 0L)
           HAL_TIM_IRQHandler (timHandle);    // Have HAL deal with it
           else {     // We got an interrupt but no handler for it.
                      // Just discard it, otherwise we get int a looping
                      // "Hot I/O" interrupt scenario.
                  TIMbase->SR = ~(TIMbase->SR);         // discard the interrupt
                }
        return;                               // then bail out
       }

       //---------------------------------------------------------------------
       // We sequentially walk down each key interrupt type, so that if
       // multiple interrupts are signalled at the same time, we handle them.
       //---------------------------------------------------------------------
  if (app_rupts_mask & TIM_SR_UIF)       // is it an update/rollover interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_ROLLOVER_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_UPDATE);            // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for UPDATE / rollover rupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC1IF)                // is it a CCR1  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR1_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC1);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC2IF)                // is it a CCR2  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR2_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC2);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC3IF)                // is it a CCR3  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR3_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC3);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC4IF)               // is it a CCR4  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR4_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC4);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }
}

#if defined(HAS_TIM10)
void  TIM1_UP_TIM10_IRQHandler (void)      // NVIC IRQn is: TIM1_UP_TIM10_IRQn
{
#if defined(HAS_TIM1)
    if (TIM1->SR & TIM_SR_UIF)
       TIM_Common_IRQHandler (TIM1, 1);     // TIM1
#endif

    if (TIM10->SR & (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF))
       TIM_Common_IRQHandler (TIM10, 10);   // TIM10
}
#endif

#if defined(TIM1)
void  TIM1_CC_IRQHandler(void)              // NVIC IRQn is:    TIM1_CC_IRQn
{
    TIM_Common_IRQHandler (TIM1, 1);        // TIM1
}
#endif


#if defined(TIM2)
void  TIM2_IRQHandler (void)                // NVIC IRQn is:    TIM2_IRQn
{
    TIM_Common_IRQHandler (TIM2, 2);        // TIM2
}
#endif

#if defined(TIM3)




#if defined(USES_L6474)
extern TIM_HandleTypeDef hTimPwm1;
void  EasySpin_StepClockHandler (int device_id);
#endif

#if MAIN_COPY_IN_easy_spin_main_
       // the following is called back by the HAL_TIM_IRQHandler()
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{

    EasySpin_StepClockHandler (0);   // passes in Device Id

}
#endif

void  TIM3_IRQHandler (void)                // NVIC IRQn is:    TIM3_IRQn
{

tim3_TIM_rupts_seen++;

  #if defined(USES_L6474)
    HAL_TIM_IRQHandler (&hTimPwm1);         // TIM3    09/08/15 TEMP HACK

    EasySpin_StepClockHandler (0);  // passes in Device Id  BRUTE FORCE callback
  #else
    TIM_Common_IRQHandler (TIM2, 3);       // TIM3
  #endif

}
#endif





#if defined(HAS_TIM4)
void  TIM4_IRQHandler(void)                 // NVIC IRQn is:    TIM4_IRQn
{
    TIM_Common_IRQHandler (TIM4, 4);            // TIM4
}
#endif

#if defined(HAS_TIM5)
void  TIM5_IRQHandler (void)                // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM5, 5);        // TIM5
}
#endif

#if defined(HAS_TIM6)
void  TIM6_DAC_IRQHandler (void)            // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM6, 6);        // TIM6
}
#endif

#if defined(HAS_TIM7)
void  TIM7_IRQHandler (void)                // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM7, 7);        // TIM6
}
#endif



// HAVE A WHOLE BUNCH OF VARIATIONS:  TIM8 + TIM11/12/13

#if defined(HAS_TIM8)
void  TIM8_CC_IRQHandler (void)             // NVIC IRQn is:    TIM8_CC_IRQ
{
    TIM_Common_IRQHandler (TIM8, 8);        // TIM8  CCR rupts only
}

//  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
//  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
//  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
//  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
#endif


#if defined(HAS_TIM9)
void  TIM1_BRK_TIM9_IRQHandler (void)     // NVIC IRQn is:    TIM1_BRK_TIM9_IRQn
{
        // We do not support TIM1_BRK rupts, so only TIM9 rupts should show up
    TIM_Common_IRQHandler (TIM9, 9);
}
#endif


#if defined(HAS_TIM10)
void  TIM1_BRK_TIM10_IRQHandler (void)     // NVIC IRQn is:   TIM1_BRK_TIM10_IRQn
{
        // We do not support TIM1_BRK rupts, so only TIM9 rupts should show up
    TIM_Common_IRQHandler (TIM10, 10);
}
#endif

#if defined(HAS_TIM11)
void  TIM1_TRG_COM_TIM11_IRQHandler (void)  // NVIC IRQn is:  TIM1_TRG_COM_TIM11_IRQn
{
        // We do not support TIM1_COM/TRIGGER rupts, so only TIM11 rupts should show up
    TIM_Common_IRQHandler (TIM11, 11);
}
#endif

#if defined(HAS_TIM12)
void  TIM12_IRQHandler (void)          // NVIC IRQn is:    TIM12_IRQn
{
    TIM_Common_IRQHandler (TIM12, 12);
}
#endif

#if defined(HAS_TIM13)
void  TIM13_IRQHandler (void)          // NVIC IRQn is:    TIM13_IRQn
{
    TIM_Common_IRQHandler (TIM13, 13);
}
#endif

#if defined(HAS_TIM14)
void  TIM14_IRQHandler (void)          // NVIC IRQn is:    TIM14_IRQn
{
    TIM_Common_IRQHandler (TIM14, 14);
}
#endif

#if defined(HAS_TIM15)
void  TIM15_IRQHandler (void)          // NVIC IRQn is:    TIM15_IRQn
{
    TIM_Common_IRQHandler (TIM15, 15);
}
#endif

#if defined(HAS_TIM16)
void  TIM16_IRQHandler (void)          // NVIC IRQn is:    TIM16_IRQn
{
    TIM_Common_IRQHandler (TIM16, 16);
}
#endif

#if defined(HAS_TIM17)
void  TIM17_IRQHandler (void)          // NVIC IRQn is:    TIM17_IRQn
{
    TIM_Common_IRQHandler (TIM17, 17);
}
#endif

#if defined(HAS_TIM20)
void  TIM20_IRQHandler (void)          // NVIC IRQn is:    TIM20_IRQn
{
    TIM_Common_IRQHandler (TIM20, 20);
}
#endif

#if defined(HAS_TIM21)
void  TIM21_IRQHandler (void)          // NVIC IRQn is:    TIM21_IRQn
{
    TIM_Common_IRQHandler (TIM21, 21);
}
#endif

#if defined(HAS_TIM22)
void  TIM22_IRQHandler (void)          // NVIC IRQn is:    TIM22_IRQn
{
    TIM_Common_IRQHandler (TIM22, 20);
}
#endif

/******************************************************************************/
