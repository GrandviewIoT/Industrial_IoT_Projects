
//*****************************************************************************
//*****************************************************************************
//
//                            board_F7_tables_timer.c
//
//
//                                STM32  F7  46
//
//
//             MCU SPECIFIC   TIMER  and  PWM   GPIO  PIN   DEFINITIONS
//
//                                   for
//
//                       PWM   and   OC/IC   Timers
//
//  History:
//    08/18/15 - Verified tables. Duq    -- BUT -- 08/22/15 - Table below needs to get _ALTx in line with final tables (tables are correct, below diagram is out of sync)
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
//
//   These GPIO definitions for TIMx_CHx GPIO mappings are used in common by
//   the PWM logic when setting up PWM outputs, and by the Timer Output Compare
//   and Input Capture functions that use GPIOs.
//
//------------------------------------------------------------------------------
//                    Timer / PWM Summary     available on STM32 F0
//                                                                  Supports
//  PWM Timer  Num CCRs   Complementary    DMA     Up/Down/Center     PWM
//    TIM1        4           Yes           Yes       All             Yes
//    TIM2        4           No            Yes       All             Yes
//    TIM3        4           No            Yes       All             Yes
//    TIM4        4           No            Yes       All             Yes
//    TIM5        4           No            Yes       All             Yes
//    TIM6        0           No            Yes       Up only         No
//    TIM7        0           No            Yes       Up only         No
//    TIM8        4           Yes           Yes       Up only         Yes
//    TIM9        2           No            No        Up only         Yes
//    TIM10       1           No            No        Up only         Yes
//    TIM11       1           No            No        Up only         Yes
//    TIM12       2           No            No        Up only         Yes
//    TIM13       1           No            No        Up only         Yes
//    TIM14       1           No            No        Up only         Yes
//
// TIM6 and TIM7 are "Basic" Timers with no useful PWM capability.
// They are only used as timers, mainly for DAC triggering.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                       Timer Pinout Definitions  -  DISCOVERY
//
//
//   _CAUTION_:  F7 DISCOVERY PINOUT is _MUCH_ _MUCH_ DIFFERENT THAN NUCLEOS
//
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//   D13    PI_1   - none -

//                 TIMER_1            TIMER_8                TIMER_12
//   D12    PB_14  TIMER_CHANNEL_2_N  TIMER_CHANNEL_2_N      TIMER_CHANNEL_1
//                 TIMER_1            TIMER_8                TIMER_12
//   D11    PB_15  TIMER_CHANNEL_3_N  TIMER_CHANNEL_3_N      TIMER_CHANNEL_2

//                 TIMER_5
//   D10    PI_0   TIMER_CHANNEL_4

//                 TIMER_2
//   D9     PA_15  TIMER_CHANNEL_1_ALT_1

//                 TIMER_8
//   D8     PI_2   TIMER_CHANNEL_4

//   D7     PI_3   - none -

//                 TIMER_12
//   D6     PH_6   TIMER_CHANNEL_1

//                 TIMER_1
//   D5     PA_8   TIMER_CHANNEL_1

//   D4     PG_7    - none -

//                 TIMER_3
//   D3     PB_4   TIMER_CHANNEL_1

//   D2     PG_6    - none -

//                 TIMER_3              TIMER_8
//   D1 vcp PC_6   TIMER_CHANNEL_1      TIMER_CHANNEL_1

//                 TIMER_3              TIMER_8
//   D0 vcp PC_7   TIMER_CHANNEL_2_ALT2 TIMER_CHANNEL_2

//                 TIMER_2              TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1      TIMER_CHANNEL_1

//   A1     PF_10  - none -

//   A2     PF_9   - none -

//   A3     PF_8   - none -

//                 TIMER_11
//   A4     PF_7   TIMER_CHANNEL_2

//                 TIMER_10
//   A5     PF_6   TIMER_CHANNEL_1

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_2            TIMER_5
//  xA1     PA_1   TIMER_CHANNEL_2    TIMER_CHANNEL_2
//                 TIMER_2            TIMER_5                TIMER_9
//  xD1     PA_2   TIMER_CHANNEL_3    TIMER_CHANNEL_3_ALT1   TIMER_CHANNEL_1
//                 TIMER_2            TIMER_5                TIMER_9
//  xD0     PA_3   TIMER_CHANNEL_4    TIMER_CHANNEL_4_ALT1   TIMER_CHANNEL_2
//  xA2     PA_4   - none -
//                 TIMER_2            TIMER_8
//  xD13    PA_5   TIMER_CHANNEL_1    TIMER_CHANNEL_1_N      -
//                 TIMER_3                                  TIMER_13
//  xD12    PA_6   TIMER_CHANNEL_1_ALT2                     TIMER_CHANNEL_1
//                 TIMER_1            TIMER_3               TIMER_14         TIMER_8
//  xD11    PA_7   TIMER_CHANNEL_1_N  TIMER_CHANNEL_2_ALT2  TIMER_CHANNEL_1  TIMER_CHANNEL_1_N
//                 TIMER_1
//  xD7     PA_8   TIMER_CHANNEL_1
//                 TIMER_1
//  xD8     PA_9   TIMER_CHANNEL_2
//                 TIMER_1
//  xD2     PA_10  TIMER_CHANNEL_3
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4
//                 TIMER_2
//   CNx    PA_15  TIMER_CHANNEL_1

//                 TIMER_3            TIMER_1               TIMER_8
//  xA3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N     TIMER_CHANNEL_2_N
//                 TIMER_3            TIMER_1               TIMER_8
//   CNx    PB_1   TIMER_CHANNEL_4    TIMER_CHANNEL_3_N     TIMER_CHANNEL_3_N
//                 TIMER_2
//  xD3     PB_3   TIMER_CHANNEL_2
//                 TIMER_3
//  xD5     PB_4   TIMER_CHANNEL_1
//                 TIMER_3
//  xD4     PB_5   TIMER_CHANNEL_2
//                 TIMER_4                                  TIMER_16  (AF14)
//  xD10    PB_6   TIMER_CHANNEL_1                          TIMER_CHANNEL_1_N
//                 TIMER_4
//   CNx    PB_7   TIMER_CHANNEL_2
//                 TIMER_2
//  xD6     PB_10  TIMER_CHANNEL_3
//                 TIMER_2
//   CNx    PB_11  TIMER_CHANNEL_4_ALT1
//          PB_12 - none -
//                 TIMER_1
//   CNx    PB_13  TIMER_CHANNEL_1_N_ALT
//                 TIMER_1            TIMER_8               TIMER_12
//   CNx    PB_14  TIMER_CHANNEL_2_N  TIMER_CHANNEL_2_N     TIMER_CHANNEL_1
//                 TIMER_1            TIMER_8               TIMER_12
//   CNx    PB_15  TIMER_CHANNEL_3_N  TIMER_CHANNEL_3_N     TIMER_CHANNEL_2

//  xA4     PC_1   - none -
//  xA5     PC_0   - none -
//   PC_2 - PC_5   - none -
//                 TIMER_3            TIMER_8
//   CNx    PC_6   TIMER_CHANNEL_1    TIMER_CHANNEL_1
//                 TIMER_8            TIMER_3
//   xD9    PC_7   TIMER_CHANNEL_2    TIMER_CHANNEL_2_ALT2
//                 TIMER_3            TIMER_8
//   CNx    PC_8   TIMER_CHANNEL_3    TIMER_CHANNEL_3
//                 TIMER_3            TIMER_8
//   CNx    PC_9   TIMER_CHANNEL_4    TIMER_CHANNEL_4
//  PC_10 - PC_15  - none -

//   PD_0 - PD_11  - none -
//                 TIMER_4
//   CNx    PD_12  TIMER_CHANNEL_1
//                 TIMER_4
//   CNx    PD_13  TIMER_CHANNEL_2
//                 TIMER_4
//   CNx    PD_14  TIMER_CHANNEL_3
//                 TIMER_4
//   CNx    PD_15  TIMER_CHANNEL_4

//
//   CNx    PE_0   - none -
//   CNx    PE_1   - none -
//          PE_2   - none -
//   CNx    PE_3   - none -
//   CNx    PE_4   - none -
//                 TIMER_9
//   CNx    PE_5   TIMER_CHANNEL_1
//                 TIMER_9
//   CNx    PE_6   TIMER_CHANNEL_2
//          PE_7   - none -
//                 TIMER_1
//   CNx    PE_8   TIMER_CHANNEL_1_N
//                 TIMER_1
//   CNx    PE_9   TIMER_CHANNEL_1
//                 TIMER_1
//   CNx    PE_10  TIMER_CHANNEL_2_N
//                 TIMER_1
//   CNx    PE_11  TIMER_CHANNEL_2
//                 TIMER_1
//   CNx    PE_12  TIMER_CHANNEL_3_N
//                 TIMER_1
//   CNx    PE_13  TIMER_CHANNEL_3
//                 TIMER_1
//   CNx    PE_14  TIMER_CHANNEL_4
//          PE_15  - none -

//   PF_0 - PF_5   - none -
//                 TIMER_10
//   CNx    PF_6   TIMER_CHANNEL_1
//                 TIMER_11
//   CNx    PF_7   TIMER_CHANNEL_2
//   CNx    PF_8   - none -
//   CNx    PF_9   - none -
//   CNx    PF_10  - none -
//   CNx    PF_11  - none -
//  PF_10 - PF1_5  - none -

//   PG_0 - PG_11  - none -
//   CNx    PG_12  - none -
//   CNx    PG_13  - none -
//   CNx    PG_14  - none -

//   CNx    PH_2   - none -
//                 TIMER_12
//   CNx    PH_6   TIMER_CHANNEL_1
//   CNx    PH_7   - none -
//   CNx    PH_8   - none -
//                 TIMER_12
//   CNx    PH_9   TIMER_CHANNEL_2
//                 TIMER_5
//   CNx    PH_10  TIMER_CHANNEL_1
//                 TIMER_5
//   CNx    PH_11  TIMER_CHANNEL_2
//                 TIMER_5
//   CNx    PH_12  TIMER_CHANNEL_4
//                 TIMER_8
//   CNx    PH_13  TIMER_CHANNEL_1_N
//                 TIMER_8
//   CNx    PH_13  TIMER_CHANNEL_2_N
//                 TIMER_8
//   CNx    PH_13  TIMER_CHANNEL_3_N

//                 TIMER_5
//   CNx    PI_0   TIMER_CHANNEL_4
//   CNx    PI_1   - none -
//                 TIMER_8
//   CNx    PI_2   TIMER_CHANNEL_4
//   CNx    PI_3   - none -
//   CNx    PI_4   - none -
//                 TIMER_8
//   CNx    PI_5   TIMER_CHANNEL_1
//                 TIMER_8
//   CNx    PI_6   TIMER_CHANNEL_2
//                 TIMER_8
//   CNx    PI_7   TIMER_CHANNEL_3      - Last timer pin
//--------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                      Timer Pinout Definitions - NUCLEO    -- FUTURE --
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//                 TIMER_2           TIMER_8
//   D13    PA_5   TIMER_CHANNEL_1   TIMER_CHANNEL_1_N      -

//                 TIMER_3                                  TIMER_13
//   D12    PA_6   TIMER_CHANNEL_1_ALT2                     TIMER_CHANNEL_1

//                 TIMER_1            TIMER_3               TIMER_14         TIMER_8
//   D11    PA_7   TIMER_CHANNEL_1_N  TIMER_CHANNEL_2_ALT2  TIMER_CHANNEL_1  TIMER_CHANNEL_1_N

//                 TIMER_4                                  TIMER_16  (AF14)
//   D10    PB_6   TIMER_CHANNEL_1                          TIMER_CHANNEL_1_N

//                 TIMER_8            TIMER_3
//   D9     PC_7   TIMER_CHANNEL_2    TIMER_CHANNEL_2_ALT2

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

//                 TIMER_2            TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1    TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5
//   A1     PA_1   TIMER_CHANNEL_2    TIMER_CHANNEL_2

//   A2     PA_4   - none -

//                 TIMER_3            TIMER_1                TIMER_8
//   A3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N      TIMER_CHANNEL_2_N

//   A4     PC_1   - none -

//   A5     PC_0   - none -


#define  MAX_TIMER        TIMER_14

#define  TMRPWM_NUM_MODULES     14  // We allow a range of 1 to 11 for Timer
                                    // module Ids, but not all modules are
                                    // enabled or present. This just specifies
                                    // the max size of the array. Timer modules
                                    // not present will have a value of 0 and be
                                    // rejected if the user tries to invoke.
#define  TMRPWM_MAX_CHANNELS     4  // are max 4 channels per PWM module


                             //-------------------------------------------------
                             //  List of Physical Base Address of TIMx registers
                             //-------------------------------------------------
const TIM_TypeDef      *_g_timer_module_base []   /* Register base address */
                                = { 0L,                    // no TIM0  on F7
                                    TIM1,  // full, w/deadtime, w/complementary,
                                    TIM2,  // no dead time, no complementary
                                    TIM3,  //   ditto       (servos, encoders)
                                    TIM4,  //   ditto
                                    TIM5,  //   ditto
                                    TIM6,  //   ditto
                                    TIM7,  //   ditto
                                    TIM8,  //   full, w/deadtime, w/complementary,
                                    TIM9,  //   no dead time, no complementary, ...
                                    TIM10, //   ditto
                                    TIM11, //   ditto
                                    TIM12, //   ditto
                                    TIM13, //   ditto
                                    TIM14  //   ditto      last valid timer on F7
                                  };


                           //----------------------------------------
                           //   TIMx handles needed, one per module
                           //----------------------------------------
    TIM_HandleTypeDef  _g_TIM1_TimPwmHandle;   // Timer/PWM  handle for TIM1
    TIM_HandleTypeDef  _g_TIM2_TimPwmHandle;   // Timer/PWM  handle for TIM2
    TIM_HandleTypeDef  _g_TIM3_TimPwmHandle;   // Timer/PWM  handle for TIM3
    TIM_HandleTypeDef  _g_TIM4_TimPwmHandle;   // Timer/PWM  handle for TIM4
    TIM_HandleTypeDef  _g_TIM5_TimPwmHandle;   // Timer/PWM  handle for TIM5
    TIM_HandleTypeDef  _g_TIM6_TimPwmHandle;   // Timer/PWM  handle for TIM6
    TIM_HandleTypeDef  _g_TIM7_TimPwmHandle;   // Timer/PWM  handle for TIM7
    TIM_HandleTypeDef  _g_TIM8_TimPwmHandle;   // Timer/PWM  handle for TIM8
    TIM_HandleTypeDef  _g_TIM9_TimPwmHandle;   // Timer/PWM  handle for TIM9
    TIM_HandleTypeDef  _g_TIM10_TimPwmHandle;  // Timer/PWM  handle for TIM10
    TIM_HandleTypeDef  _g_TIM11_TimPwmHandle;  // Timer/PWM  handle for TIM11
    TIM_HandleTypeDef  _g_TIM12_TimPwmHandle;  // Timer/PWM  handle for TIM12
    TIM_HandleTypeDef  _g_TIM13_TimPwmHandle;  // Timer/PWM  handle for TIM13
    TIM_HandleTypeDef  _g_TIM14_TimPwmHandle;  // Timer/PWM  handle for TIM14


                             //----------------------------------------------------------
                             //  List of HAL Handle Address for each separate TIM Handle
                             //----------------------------------------------------------
const TIM_HandleTypeDef  *_g_timer_typedef_handle [] /* HAL API Handles */
                       = {             0L,                     // no TIM0  on F7
                           (TIM_HandleTypeDef*) &_g_TIM1_TimPwmHandle,  // TIM1
                           (TIM_HandleTypeDef*) &_g_TIM2_TimPwmHandle,  // TIM2
                           (TIM_HandleTypeDef*) &_g_TIM3_TimPwmHandle,  // TIM3
                           (TIM_HandleTypeDef*) &_g_TIM4_TimPwmHandle,  // TIM4
                           (TIM_HandleTypeDef*) &_g_TIM5_TimPwmHandle,  // TIM5
                           (TIM_HandleTypeDef*) &_g_TIM6_TimPwmHandle,  // TIM6
                           (TIM_HandleTypeDef*) &_g_TIM7_TimPwmHandle,  // TIM7
                           (TIM_HandleTypeDef*) &_g_TIM8_TimPwmHandle,  // TIM8
                           (TIM_HandleTypeDef*) &_g_TIM9_TimPwmHandle,  // TIM9
                           (TIM_HandleTypeDef*) &_g_TIM10_TimPwmHandle, // TIM10
                           (TIM_HandleTypeDef*) &_g_TIM11_TimPwmHandle, // TIM11
                           (TIM_HandleTypeDef*) &_g_TIM12_TimPwmHandle, // TIM12
                           (TIM_HandleTypeDef*) &_g_TIM13_TimPwmHandle, // TIM13
                           (TIM_HandleTypeDef*) &_g_TIM14_TimPwmHandle  // TIM14 - last timer on F7
                         };

                             //----------------------------------------------------------
                             // _g_TMPWM_IRQ_table
                             //
                             //     Table of IRQ numbers used to setup the associated
                             //     NVIC interrupt enable for a specific timer.
                             //----------------------------------------------------------
const unsigned char    _g_TMPWM_IRQ_table []
                                = { 0,              //  0 is not valid
                                    TIM1_CC_IRQn,   //  TIMER_1 CCRs only
                                                    //  Timer 1 updates handled in code (TIM1_UP_TIM10_IRQn)
                                    TIM2_IRQn,      //  TIMER_2 update and CCRs
                                    TIM3_IRQn,      //  TIMER_3   "     "   "
                                    TIM4_IRQn,      //  TIMER_4   "     "   "
                                    TIM5_IRQn,      //  TIMER_5   "     "   "
                                    TIM6_DAC_IRQn,  //  TIMER_6 update only
                                    TIM7_IRQn,      //  TIMER_7 update only
                                    TIM8_CC_IRQn,   //  TIMER_8 CCRs only
// ??? need to handle TIM8 rollovers as specuial case ala TIM1 (TIM8_UP_TIM13_IRQn)
                                TIM1_BRK_TIM9_IRQn, //  TIMER_9 update
                                TIM1_UP_TIM10_IRQn, //  TIMER_10 update  -- AND --  SHARED WITH TIM1 Update
                           TIM1_TRG_COM_TIM11_IRQn, //  TIMER_11 update      "               "    "
                           TIM8_BRK_TIM12_IRQn,     //  TIMER_12 update  -- AND --  SHARED WITH TIM8 Update
                           TIM8_UP_TIM13_IRQn,      //  TIMER_13 update      "               "    "
                           TIM8_TRG_COM_TIM14_IRQn  //  TIMER_14 update      "               "    "
                                  };

               //--------------------------------------------------------------
               //                       GPIO   MAPPING   TABLES
               //
               // These tables contain the GPIO mapping for every GPIO pin that
               // can be used for Input (Input Capture) or Output (PWM or
               // Timer Output Compare) by the Timer/PWM modules.
               //
               // They are also correlated to a Timer Channel number (1,2,3,4)
               // which are in turn controlled by a CCR with the same number,
               // e.g. Timer Channel 1 (ex TIM1_CH1) is controlled by CCR1 on
               // the Timer 1 module; Timer Channel 3 (ex TIM2_CH3) is
               // controlled by CCR3 on the Timer 2 module, etc.
               //
               // A number of channels have alternative pin mappings, e.g.
               // TIM1_CH1 can be routed out to the PA8, PA7, or PB13 pins.
               // When multiple mappings exist, the 2nd and 3rd mappings
               // are distinguished as ALT1 and ALT2 in the user_api.h file
               // board specific entries. E.g. for TIM1_CH1, it would be
               // specified by the user as:
               //       CHANNEL_1           // default = PA8
               //       CHANNEL_1_ALT1      // use  PA7
               //       CHANNEL_1_ALT2      // use  PB13
               // See comments in the API documentation that describe which GPIO
               // pin each alternative is assigned to (e.g. PA8, PA7, or PB13)
               //--------------------------------------------------------------

typedef struct tim_pwm_channel_def     /* TIMER/PWM Channel definitions   */
    {
        GPIO_TypeDef *chan_gpio_port;  /* Associated GPIO port            */
        uint32_t     chan_gpio_pin;    /* Associated GPIO pin             */
        uint8_t      chan_alt_func;    /* Channel Alternate Function Id   */
        uint8_t      user_channel_id;  /* USER API designation: CHANNEL_1, CHANNEL_1_ALT1, ... */
        uint16_t     chan_pwm_id;      /* Chip Channel id for this CCR    */
    } TMPWM_CHANNEL_BLK;

                                   // ??? !!!  WVD  ONLY FIELDS WITH  ARDUINO #s  are  verified !!! ???
const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_0_channels [] =                 // TIM0
       { {   0L,       0,             0,              0,              0       }, // Dummy entry
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_1_channels [] =                 // TIM1
       { {   0L,       0,             0,              0,              0       }, // STM32            Arduino
         { GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA8  TIM1_CH1       D5
         { GPIOA, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PA9  TIM1_CH2
         { GPIOA, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PA10 TIM1_CH3
         { GPIOA, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PA11 TIM1_CH4
                     // Complementary pins
         { GPIOA, GPIO_PIN_7, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N, TIM_CHANNEL_1 }, // PA7  TIM1_CH1N    D11
         { GPIOB, GPIO_PIN_0, GPIO_AF1_TIM1, TIMER_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB0  TIM1_CH2N    A3
         { GPIOB, GPIO_PIN_1, GPIO_AF1_TIM1, TIMER_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB1  TIM1_CH3N
                     // Complementary - Alternates
         { GPIOB, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT1,TIM_CHANNEL_1}, // PB13 TIM1_CH1N
         { GPIOB, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT1,TIM_CHANNEL_2}, // PB14 TIM1_CH2N
         { GPIOB, GPIO_PIN_15,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT1,TIM_CHANNEL_3}, // PB15 TIM1_CH3N
    // Alternates
         { GPIOE, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PE9  TIM1_CH1
         { GPIOE, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 }, // PE11 TIM1_CH2
         { GPIOE, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_3_ALT1, TIM_CHANNEL_3 }, // PE13 TIM1_CH3
         { GPIOE, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_4_ALT1, TIM_CHANNEL_1 }, // PE14 TIM1_CH4
                     // Complementary - Alternates
         { GPIOE, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT2,TIM_CHANNEL_1 }, // PE8  TIM1_CH1N
         { GPIOE, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT2,TIM_CHANNEL_2 }, // PE10 TIM1_CH2N
         { GPIOE, GPIO_PIN_12,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT2,TIM_CHANNEL_3 }, // PE12 TIM1_CH3N
         { 0,          0,             0,              0,              0           }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_2_channels [] =                 // TIM2
       { {    0L,      0,             0,             0,             0         }, // STM32         Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM2_CH1     A0
         { GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB3  TIM2_CH2     D3
         { GPIOB, GPIO_PIN_10,GPIO_AF1_TIM2, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB10 TIM2_CH3     D6
         { GPIOB, GPIO_PIN_11,GPIO_AF1_TIM2, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB11 TIM2_CH4
     // Alternates
         { GPIOA, GPIO_PIN_5, GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PA5  TIM2_CH1
         { GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PA1  TIM2_CH2  A1
         { GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3 }, // PA2  TIM2_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4 }, // PA3  TIM2_CH4  D0 VCP
         { GPIOA, GPIO_PIN_15,GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1 }, // PA15 TIM2_CH1 JTAG/JTDI
         { 0,          0,             0,              0,              0           }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_3_channels [] =                 // TIM3
       { {   0L,      0L,            0,               0,              0       }, // STM32          Arduino
         { GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PB4  TIM3_CH1      D3
         { GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB5  TIM3_CH2
         { GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB0  TIM3_CH3
         { GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB1  TIM3_CH4
     // Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PA6  TIM3_CH1  D12
         { GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PA7  TIM3_CH2  D11
         { GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1 }, // PC6  TIM3_CH1
         { GPIOC, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT2,TIM_CHANNEL_2 }, // PC7  TIM3_CH2
         { GPIOC, GPIO_PIN_8, GPIO_AF2_TIM3, TIMER_CHANNEL_3_ALT2,TIM_CHANNEL_3 }, // PC8  TIM3_CH3
         { GPIOC, GPIO_PIN_9, GPIO_AF2_TIM3, TIMER_CHANNEL_4_ALT2,TIM_CHANNEL_4 }, // PC9  TIM3_CH4
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_4_channels [] =                 // TIM4
       { {   0L,      0L,            0,               0,              0       }, // STM32            Arduino
         { GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PB6  TIM4_CH1
         { GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB7  TIM4_CH2
         { GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB8  TIM4_CH3       D15
         { GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB9  TIM4_CH4       D14
     // Alternates
         { GPIOD, GPIO_PIN_12,GPIO_AF2_TIM4, TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PD12 TIM4_CH1
         { GPIOD, GPIO_PIN_13,GPIO_AF2_TIM4, TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 }, // PD13 TIM4_CH2
         { GPIOD, GPIO_PIN_14,GPIO_AF2_TIM4, TIMER_CHANNEL_3_ALT1, TIM_CHANNEL_3 }, // PD14 TIM4_CH3
         { GPIOD, GPIO_PIN_15,GPIO_AF2_TIM4, TIMER_CHANNEL_4_ALT1, TIM_CHANNEL_4 }, // PD15 TIM4_CH4
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_5_channels [] =                 // TIM5
       { {   0L,      0L,            0,               0,            0       }, // STM32           Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM5_CH1      A0
         { GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PA1  TIM5_CH2
         { GPIOH, GPIO_PIN_12,GPIO_AF2_TIM5, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PH12 TIM5_CH3
         { GPIOI, GPIO_PIN_0, GPIO_AF2_TIM5, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PI0  TIM5_CH4
     // Alternates
         { GPIOH, GPIO_PIN_10,GPIO_AF2_TIM5, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PH10 TIM5_CH1
         { GPIOH, GPIO_PIN_11,GPIO_AF2_TIM5, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PH11 TIM5_CH2
         { GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3 }, // PA2  TIM5_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4 }, // PA3  TIM5_CH4  D0 VCP
         { 0,          0,             0,              0,              0         }  // end of table
       };

              //--------------------------------------------------------------------------------
              // Note that TIMER6 and TIMER7 have no external pins for PWM, IC, nor OC Channels
              //--------------------------------------------------------------------------------

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_8_channels [] =                  // TIM8
       { {   0L,      0L,             0,              0,              0     }, // STM32            Arduino
         { GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PC6  TIM8_CH1
         { GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PC7  TIM8_CH2
         { GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PC8  TIM8_CH3
         { GPIOI, GPIO_PIN_2, GPIO_AF3_TIM8, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PI2  TIM8_CH4     D8
                        // Complementary pins
         { GPIOA, GPIO_PIN_5, GPIO_AF3_TIM8, TIMER_CHANNEL_1_N, TIM_CHANNEL_1 }, // PA5 TIM8_CH1N   D13
         { GPIOB, GPIO_PIN_0, GPIO_AF3_TIM8, TIMER_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB0 TIM8_CH2N   A3
         { GPIOB, GPIO_PIN_1, GPIO_AF3_TIM8, TIMER_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB1 TIM8_CH3N
                        // Complementary - Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF3_TIM8, TIMER_CHANNEL_1_N_ALT1,TIM_CHANNEL_1},// PA6  TIM8_CH1N D12
         { GPIOB, GPIO_PIN_14,GPIO_AF3_TIM8, TIMER_CHANNEL_2_N_ALT1,TIM_CHANNEL_2},// PB14 TIM8_CH2N
         { GPIOB, GPIO_PIN_15,GPIO_AF3_TIM8, TIMER_CHANNEL_3_N_ALT1,TIM_CHANNEL_3},// PB15 TIM8_CH3N
                        // Complementary - Alternates
         { GPIOH, GPIO_PIN_13,GPIO_AF3_TIM8, TIMER_CHANNEL_1_N_ALT2,TIM_CHANNEL_1},// PH13 TIM8_CH1N
         { GPIOH, GPIO_PIN_14,GPIO_AF3_TIM8, TIMER_CHANNEL_2_N_ALT2,TIM_CHANNEL_2},// PH14 TIM8_CH2N
         { GPIOH, GPIO_PIN_15,GPIO_AF3_TIM8, TIMER_CHANNEL_3_N_ALT2,TIM_CHANNEL_3},// PH15 TIM8_CH3N
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_9_channels [] =                 // TIM9
       { {   0L,      0L,            0,               0,              0     }, // STM32            Arduino
         { GPIOE, GPIO_PIN_5, GPIO_AF3_TIM9, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PE5 TIM9_CH1
         { GPIOE, GPIO_PIN_6, GPIO_AF3_TIM9, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PE6 TIM9_CH2
     // Alternates
         { GPIOA, GPIO_PIN_2, GPIO_AF3_TIM9, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PA2 TIM9_CH1    D1
         { GPIOA, GPIO_PIN_3, GPIO_AF3_TIM9, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2}, // PA3 TIM9_CH2    D0
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_10_channels [] =                // TIM10
       { {   0L,      0L,            0,               0,              0     }, // STM32            Arduino
         { GPIOB, GPIO_PIN_8,GPIO_AF3_TIM10,TIMER_CHANNEL_1, TIM_CHANNEL_1  }, // PB8  TIM10_CH1    D15
     // Alternates
         { GPIOF, GPIO_PIN_6,GPIO_AF3_TIM10,TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1},// PF6 TIM10_CH1
         { 0,          0,             0,              0,              0     }   // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_11_channels [] =                // TIM11
       { {   0L,      0L,            0,               0,              0     }, // STM32            Arduino
         { GPIOB, GPIO_PIN_9,GPIO_AF3_TIM11,TIMER_CHANNEL_1, TIM_CHANNEL_1  }, // PB9  TIM11_CH1    D14
     // Alternates
         { GPIOF, GPIO_PIN_7,GPIO_AF3_TIM11,TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PF7 TIM11_CH1
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_12_channels [] =                // TIM12
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOB, GPIO_PIN_14,GPIO_AF9_TIM12,TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PB14 TIM12_CH1    D12
         { GPIOB, GPIO_PIN_15,GPIO_AF9_TIM12,TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB15 TIM12_CH2    D11
     // Alternates
         { GPIOH, GPIO_PIN_6, GPIO_AF9_TIM12,TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PH6  TIM12_CH1
         { GPIOH, GPIO_PIN_9, GPIO_AF9_TIM12,TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2}, // PH9  TIM12_CH2
         { 0,          0,             0,              0,              0        }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_13_channels [] =                // TIM13
       { {   0L,      0L,            0,               0,              0     }, // STM32
         { GPIOA, GPIO_PIN_6, GPIO_AF9_TIM13,TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA6  TIM13_CH1
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_14_channels [] =                // TIM14
       { {   0L,      0L,            0,               0,              0     }, // STM32
         { GPIOA, GPIO_PIN_7, GPIO_AF9_TIM14,TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA7  TIM14_CH1
         { 0,          0,             0,              0,              0     }  // end of table
       };


//                 TIMER_2              TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1      TIMER_CHANNEL_1
//                 TIMER_2              TIMER_5
//   A1     PA_1   TIMER_CHANNEL_2      TIMER_CHANNEL_2
//                 TIMER_2              TIMER_5               TIMER_9
//   D1     PA_2   TIMER_CHANNEL_3_ALT1 TIMER_CHANNEL_3_ALT1  TIMER_CHANNEL_1_ALT1  -- USELESS TX/RX Pins --
//                 TIMER_2              TIMER_5               TIMER_9
//   D0     PA_3   TIMER_CHANNEL_4_ALT1 TIMER_CHANNEL_4_ALT1  TIMER_CHANNEL_2_ALT1  -- USELESS TX/RX Pins --
//                 TIMER_2              TIMER_8
//   D13    PA_5   TIMER_CHANNEL_1_ALT1 TIMER_CHANNEL_1_N      -
//                 TIMER_3                                    TIMER_13
//   D12    PA_6   TIMER_CHANNEL_1_ALT2                       TIMER_CHANNEL_1
//                 TIMER_1              TIMER_3               TIMER_14         TIMER_8
//   D11    PA_7   TIMER_CHANNEL_1_N    TIMER_CHANNEL_2_ALT1  TIMER_CHANNEL_1  TIMER_CHANNEL_1_N_ALT1
//                 TIMER_1
//   D7     PA_8   TIMER_CHANNEL_1
//                 TIMER_1
//   D8     PA_9   TIMER_CHANNEL_2
//                 TIMER_1
//   D2     PA_10  TIMER_CHANNEL_3
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4
//                 TIMER_2
//   CNx    PA_15  TIMER_CHANNEL_1_ALT2


//                 TIMER_3            TIMER_1                TIMER_8
//   A3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N      TIMER_CHANNEL_2_N
//                 TIMER_3            TIMER_1                TIMER_8
//   CNx    PB_1   TIMER_CHANNEL_4    TIMER_CHANNEL_3_N      TIMER_CHANNEL_3_N
//                 TIMER_2
//   D3     PB_3   TIMER_CHANNEL_2_ALT1
//                 TIMER_3
//   D5     PB_4   TIMER_CHANNEL_1
//                 TIMER_3
//   D4     PB_5   TIMER_CHANNEL_2
//                 TIMER_4
//   D10    PB_6   TIMER_CHANNEL_1
//                 TIMER_4
//   CNx    PB_7   TIMER_CHANNEL_2
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1
//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1
//                 TIMER_2
//   D6     PB_10  TIMER_CHANNEL_3
//                 TIMER_2
//   CNx    PB_11  TIMER_CHANNEL_4_ALT1
//                 TIMER_1
//   CNx    PB_13  TIMER_CHANNEL_1_N_ALT1
//                 TIMER_1                 TIMER_8                 TIMER_12
//   CNx    PB_14  TIMER_CHANNEL_2_N_ALT1  TIMER_CHANNEL_2_N_ALT1  TIMER_CHANNEL_1
//                 TIMER_1                 TIMER_8                 TIMER_12
//   CNx    PB_15  TIMER_CHANNEL_3_N_ALT1  TIMER_CHANNEL_3_N_ALT1  TIMER_CHANNEL_2


//                 TIMER_3               TIMER_8
//   CNx    PC_6   TIMER_CHANNEL_1_ALT2  TIMER_CHANNEL_1
//                 TIMER_3               TIMER_8
//   D9     PC_7   TIMER_CHANNEL_2_ALT2  TIMER_CHANNEL_2
//                 TIMER_3               TIMER_8
//   CNx    PC_8   TIMER_CHANNEL_3_ALT2  TIMER_CHANNEL_3
//                 TIMER_3               TIMER_8
//   CNx    PC_9   TIMER_CHANNEL_4_ALT2  TIMER_CHANNEL_4


//                 TIMER_4
//   CNx    PD_12  TIMER_CHANNEL_1_ALT1
//                 TIMER_4
//   CNx    PD_13  TIMER_CHANNEL_2_ALT1
//                 TIMER_4
//   CNx    PD_14  TIMER_CHANNEL_3_ALT1
//                 TIMER_4
//   CNx    PD_15  TIMER_CHANNEL_4_ALT1


//                 TIMER_9
//   CNx    PE_5   TIMER_CHANNEL_1
//                 TIMER_9
//   CNx    PE_6   TIMER_CHANNEL_2
//                 TIMER_1
//   CNx    PE_8   TIMER_CHANNEL_1_N_ALT2
//                 TIMER_1
//   CNx    PE_9   TIMER_CHANNEL_1_ALT1
//                 TIMER_1
//   CNx    PE_10  TIMER_CHANNEL_2_N_ALT2
//                 TIMER_1
//   CNx    PE_11  TIMER_CHANNEL_2_ALT1
//                 TIMER_1
//   CNx    PE_12  TIMER_CHANNEL_3_N_ALT2
//                 TIMER_1
//   CNx    PE_13  TIMER_CHANNEL_3_ALT1
//                 TIMER_1
//   CNx    PE_14  TIMER_CHANNEL_4_ALT1


//                 TIMER_10
//   CNx    PF_6   TIMER_CHANNEL_1_ALT1
//                 TIMER_11
//   CNx    PF_7   TIMER_CHANNEL_1_ALT1


//                 TIMER_12
//   CNx    PH_6   TIMER_CHANNEL_1_ALT1
//                 TIMER_12
//   CNx    PH_9   TIMER_CHANNEL_2_ALT1

//                 TIMER_5
//   CNx    PH_10  TIMER_CHANNEL_1_ALT1
//                 TIMER_5
//   CNx    PH_11  TIMER_CHANNEL_2_ALT1
//                 TIMER_5
//   CNx    PH_12  TIMER_CHANNEL_3
//                 TIMER_8
//   CNx    PH_13  TIMER_CHANNEL_1_N_ALT2
//                 TIMER_8
//   CNx    PH_14  TIMER_CHANNEL_2_N_ALT2
//                 TIMER_8
//   CNx    PH_15  TIMER_CHANNEL_3_N_ALT2


//                 TIMER_5
//   CNx    PI_0   TIMER_CHANNEL_4
//                 TIMER_8
//   CNx    PI_2   TIMER_CHANNEL_4_ALT1
//                 TIMER_8
//   CNx    PI_5   TIMER_CHANNEL_1_ALT1
//                 TIMER_8
//   CNx    PI_6   TIMER_CHANNEL_2_ALT1
//                 TIMER_8
//   CNx    PI_7   TIMER_CHANNEL_3_ALT1    - Last timer pin





                             //-----------------------------------------------------
                             // _g_tmrpwm_mod_channel_blk_lookup
                             //
                             //     Lookup table to get associated TMPWM_CHANNEL_BLK
                             //     entry for a given Timer module_id
                             //-----------------------------------------------------

const TMPWM_CHANNEL_BLK  *_g_tmrpwm_mod_channel_blk_lookup [] =
          { (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_0_channels[0],  // dummy TIM0
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_1_channels[0], // TIM1
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_2_channels[0], // TIM2
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_3_channels[0],  // TIM3
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_4_channels[0],  // TIM4
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_5_channels[0],  // TIM5
                                   0,             // TIM6 has no channels > CCR0
                                   0,             // TIM7 has no channels > CCR0
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_8_channels[0],  // TIM8
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_9_channels[0],  // TIM9
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_10_channels[0], // TIM10
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_11_channels[0], // TIM11
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_12_channels[0], // TIM12
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_13_channels[0], // TIM13
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_14_channels[0]  // TIM14  Last
          };



//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_timerpwm_compute_prescalar
//
//            Dynamically compute pre-scalar value needed to make user
//            requested period fit into 16-bit PWM counters
//******************************************************************************
long  board_timerpwm_compute_prescalar (long period_val)
{
    long   prescalar;

       // Compute any needed prescaler value to fit 16-bit TIMx counter clock
    if (period_val > 6550000)            // 6.50 M ticks
       prescalar = 1200;
       else if (period_val > 655000)     // 0.65 M ticks
               prescalar = 120;
       else if (period_val > 65500)      // 0.065 M ticks
               prescalar = 12;
               else prescalar = 0;

    return (prescalar);
}


//******************************************************************************
//  board_timerpwm_enable_clock
//
//           Ensures that the specified Timer's clock is turned on.
//******************************************************************************
int  board_timerpwm_enable_clock (int module_id)
{
      //--------------------------------------------------------
      // Turn on clock for the associated TIMx module.
      //--------------------------------------------------------
    switch (module_id)
     { case 1:                           // TIM1
             __TIM1_CLK_ENABLE();
             break;
        case 2:                          // TIM2
             __TIM2_CLK_ENABLE();
             break;
        case 3:                          // TIM3
             __TIM3_CLK_ENABLE();
             break;
        case 4:                          // TIM4
             __TIM4_CLK_ENABLE();
             break;
        case 5:                          // TIM5
             __TIM5_CLK_ENABLE();
             break;
        case 6:                          // TIM6
             __TIM6_CLK_ENABLE();
             break;
        case 7:                          // TIM7
             __TIM7_CLK_ENABLE();
             break;
        case 8:                          // TIM8
             __TIM8_CLK_ENABLE();
             break;
        case 9:                          // TIM9
             __TIM9_CLK_ENABLE();
             break;
        case 10:                         // TIM10
             __TIM10_CLK_ENABLE();
             break;
        case 11:                         // TIM11
             __TIM11_CLK_ENABLE();
             break;
        case 12:                         // TIM12
             __TIM12_CLK_ENABLE();
             break;
        case 13:                         // TIM13
             __TIM13_CLK_ENABLE();
             break;
        case 14:                         // TIM14
             __TIM14_CLK_ENABLE();
             break;
        default:
             return (ERR_TIMER_NUM_NOT_SUPPORTED);
     }

    return (0);          // denote everything worked OK
}

//*****************************************************************************
