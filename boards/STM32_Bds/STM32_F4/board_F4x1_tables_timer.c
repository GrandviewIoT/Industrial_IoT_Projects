
// 07/03/15 - ADC TRIGGERing - associated TIMx CCR2's MMS field needs to be set
//            to 0x2 (Update) or 0x4 (CCR1, 0x5 (CCR2), 0x6 (CCR3), 0x7 (CCR4)
//            Right now, are only doing it for TIM6/7, but needs to be done
//            ofr ANY timer that needs to drive ADC (TIM1/TIM2/...)


//*******1*********2*********3*********4*********5*********6*********7**********
//******************************************************************************
//
//                            board_F4x1_tables_timer.c
//
//
//                                STM32  F4  01                     STM32F401xE
//                                STM32  F4  11                     STM32F411xE
//
//
//             MCU SPECIFIC   TIMER  and  PWM   GPIO  PIN   DEFINITIONS
//
//                                   for
//
//                       PWM   and   OC/IC   Timers
//
//  History:
//    08/23/15 - Verified tables. Duq
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
//
//   These GPIO definitions for TIMx_CHx GPIO mappings are used in common by
//   the PWM logic when setting up PWM outputs, and by the Timer Output Compare
//   and Input Capture functions that use GPIOs.
//
//------------------------------------------------------------------------------
//                    Timer / PWM Summary     available on STM32 F4
//                                                                  Supports
//  PWM Timer  Num CCRs   Complementary    DMA     Up/Down/Center     PWM
//    TIM1        4           Yes           Yes       All             Yes
//    TIM2        4           No            Yes       All             Yes
//    TIM3        4           No            Yes       All             Yes
//    TIM4        4           No            Yes       All             Yes
//    TIM5        4           No            Yes       All             Yes
//    TIM9        2           No            No        Up only         Yes
//    TIM10       1           No            No        Up only         Yes
//    TIM11       1           No            No        Up only         Yes
//
// Note: F4_01 does _NOT_ have a TIM8 module.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                           Timer Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//                 TIMER_2
//   D13    PA_5   TIMER_CHANNEL_1_ALT1   -                     -

//                 TIMER_3
//   D12    PA_6   TIMER_CHANNEL_1_ALT1

//                 TIMER_1              TIMER_3
//   D11    PA_7   TIMER_CHANNEL_1_N    TIMER_CHANNEL_2_ALT1

//                 TIMER_4
//   D10    PB_6   TIMER_CHANNEL_1

//                 TIMER_3
//   D9     PC_7   TIMER_CHANNEL_2_ALT2

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
//   D3     PB_3   TIMER_CHANNEL_2_ALT1

//                 TIMER_1
//   D2     PA_10  TIMER_CHANNEL_3

//                 TIMER_2               TIMER_5               TIMER_9
//   D1     PA_2   TIMER_CHANNEL_3_ALT1  TIMER_CHANNEL_3_ALT1  TIMER_CHANNEL_1_ALT1  VCP

//                 TIMER_2               TIMER_5               TIMER_9
//   D0     PA_3   TIMER_CHANNEL_4_ALT1  TIMER_CHANNEL_4_ALT1  TIMER_CHANNEL_2_ALT1  VCP

//                 TIMER_2               TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1       TIMER_CHANNEL_1

//                 TIMER_2               TIMER_5
//   A1     PA_1   TIMER_CHANNEL_2       TIMER_CHANNEL_2

//   A2     PA_4   - none -

//                 TIMER_3               TIMER_1
//   A3     PB_0   TIMER_CHANNEL_3       TIMER_CHANNEL_2_N

//   A4     PC_1   - none -

//   A5     PC_0   - none -

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4
//                 TIMER_2
//   CNx    PA_15  TIMER_CHANNEL_1_ALT2

//                 TIMER_3               TIMER_1
//   CNx    PB_1   TIMER_CHANNEL_4       TIMER_CHANNEL_3_N
//                 TIMER_4
//   CNx    PB_7   TIMER_CHANNEL_2
//                 TIMER_2
//   CNx    PB_11  TIMER_CHANNEL_4
//   CNx    PB_12 - none -
//                 TIMER_1
//   CNx    PB_13  TIMER_CHANNEL_1_N_ALT1
//                 TIMER_1
//   CNx    PB_14  TIMER_CHANNEL_2_N_ALT1
//                 TIMER_1
//   CNx    PB_15  TIMER_CHANNEL_3_N_ALT1

//   PC_2 - PC_5   - none -
//                 TIMER_3
//   CNx    PC_6   TIMER_CHANNEL_1_ALT2
//                 TIMER_3
//   CNx    PC_8   TIMER_CHANNEL_3_ALT2
//                 TIMER_3
//   CNx    PC_9   TIMER_CHANNEL_4_ALT2
//  PC_10 - PC_15  - none -

//   PD_0 - PD_11  - none -
//                 TIMER_4
//   CNx    PD_12  TIMER_CHANNEL_1_ALT1
//                 TIMER_4
//   CNx    PD_13  TIMER_CHANNEL_2_ALT1
//                 TIMER_4
//   CNx    PD_14  TIMER_CHANNEL_3_ALT1
//                 TIMER_4
//   CNx    PD_15  TIMER_CHANNEL_4_ALT1

//   CNx    PE_0   - none -
//                 TIMER_1
//   CNx    PE_1   TIMER_CHANNEL_2_N (F1_01 only)
//   PE_2 - PE_4   - none -
//                 TIMER_9
//   CNx    PE_5   TIMER_CHANNEL_1
//                 TIMER_9
//   CNx    PE_6   TIMER_CHANNEL_2
//          PE_7   - none -
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
//          PE_15  - none -

//   PF_0 - PF_8   - none -        ??? WVD 08/22/15  - Verify no other entries for TIMER_10 and TIMER_11 (PF_6 / PF_7)
//                 TIMER_15
//   CNx    PF_9   TIMER_CHANNEL_1
//                 TIMER_15
//   CNx    PF_10  TIMER_CHANNEL_2      - last pin -

//------------------------------------------------------------------------------

#define  MAX_TIMER        TIMER_11

#define  TMRPWM_NUM_MODULES     11  // We allow a range of 1 to 11 for Timer
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
                                = { 0L,                    // no TIM0  on F4
                                    TIM1,  // full, w/deadtime, w/complementary,
                                    TIM2,  // no dead time, no complementary
                                    TIM3,  //   ditto       (servos, encoders)
                                    TIM4,  //   ditto
                                    TIM5,  //   ditto
                                    0L,                    // no TIM6  on F4
                                    0L,                    // no TIM7  on F4
                                    0L,                    // no TIM8  on F4
                                    TIM9,  //   ditto
                                    TIM10, //   ditto
                                    TIM11  //   ditto      last valid timer on F4
                                  };


                           //----------------------------------------
                           //   TIMx handles needed, one per module
                           //----------------------------------------
    TIM_HandleTypeDef  _g_TIM1_TimPwmHandle;   // Timer/PWM  handle for TIM1
    TIM_HandleTypeDef  _g_TIM2_TimPwmHandle;   // Timer/PWM  handle for TIM2
    TIM_HandleTypeDef  _g_TIM3_TimPwmHandle;   // Timer/PWM  handle for TIM3
    TIM_HandleTypeDef  _g_TIM4_TimPwmHandle;   // Timer/PWM  handle for TIM4
    TIM_HandleTypeDef  _g_TIM5_TimPwmHandle;   // Timer/PWM  handle for TIM5
    TIM_HandleTypeDef  _g_TIM9_TimPwmHandle;   // Timer/PWM  handle for TIM9
    TIM_HandleTypeDef  _g_TIM10_TimPwmHandle;  // Timer/PWM  handle for TIM10
    TIM_HandleTypeDef  _g_TIM11_TimPwmHandle;  // Timer/PWM  handle for TIM11


                             //----------------------------------------------------------
                             // List of HAL Handle Addresses for each separate TIM Handle
                             //----------------------------------------------------------
const TIM_HandleTypeDef *_g_timer_typedef_handle [] /* HAL API Handles */
                                = { 0L,                     // no TIM0  on F4
                                    (TIM_HandleTypeDef*) &_g_TIM1_TimPwmHandle,  // TIM1
                                    (TIM_HandleTypeDef*) &_g_TIM2_TimPwmHandle,  // TIM2
                                    (TIM_HandleTypeDef*) &_g_TIM3_TimPwmHandle,  // TIM3
                                    (TIM_HandleTypeDef*) &_g_TIM4_TimPwmHandle,  // TIM4
                                    (TIM_HandleTypeDef*) &_g_TIM5_TimPwmHandle,  // TIM5
                                    (TIM_HandleTypeDef*) 0L,                     // no TIM6  on F4
                                    (TIM_HandleTypeDef*) 0L,                     // no TIM7  on F4
                                    (TIM_HandleTypeDef*) 0L,                     // no TIM8  on F4
                                    (TIM_HandleTypeDef*) &_g_TIM9_TimPwmHandle,  // TIM9
                                    (TIM_HandleTypeDef*) &_g_TIM10_TimPwmHandle, // TIM10
                                    (TIM_HandleTypeDef*) &_g_TIM11_TimPwmHandle  // TIM11 - last timer on F4
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
                                                    //  Timer 1 updates handled in code (TIM1_TRG_COM_TIM11_IRQn)
                                    TIM2_IRQn,      //  TIMER_2 update and CCRs
                                    TIM3_IRQn,      //  TIMER_3   "     "   "
                                    TIM4_IRQn,      //  TIMER_4
                                    TIM4_IRQn,      //  TIMER_5
                                    0,              //  no TIM6
                                    0,              //  no TIM7
                                    0,              //  no TIM8
                                TIM1_BRK_TIM9_IRQn, //  TIMER_9 update
                                TIM1_UP_TIM10_IRQn, //  TIMER_10 update  -- AND --  SHARED WITH TIM1 Update
                           TIM1_TRG_COM_TIM11_IRQn  //  TIMER_11 update
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
               // NOTE: Ports D and E are _NOT_ hooked up on F0_72

typedef struct tim_pwm_channel_def     /* TIMER/PWM Channel definitions   */
    {
        GPIO_TypeDef *chan_gpio_port;  /* Associated GPIO port            */
        uint32_t     chan_gpio_pin;    /* Associated GPIO pin             */
        uint8_t      chan_alt_func;    /* Channel Alternate Function Id   */
        uint8_t      user_channel_id;  /* USER API designation: CHANNEL_1, CHANNEL_1_ALT1, ... */
        uint16_t     chan_pwm_id;      /* Chip Channel id for this CCR    */
    } TMPWM_CHANNEL_BLK;


const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_0_channels [] =                  // TIM0
       { {   0L,       0,             0,              0,              0     }, // Dummy entry
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_1_channels [] =                  // TIM1
       { {   0L,      0L,             0,              0,              0     }, // STM32           Arduino
         { GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PA8  TIM1_CH1      D7
         { GPIOA, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // =-PA9  TIM1_CH2      D8
         { GPIOA, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // =-PA10 TIM1_CH3      D2
         { GPIOA, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // =-PA11 TIM1_CH4
                     // Complementary pins
         { GPIOA, GPIO_PIN_7, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N, TIM_CHANNEL_1 }, // =-PA7  TIM1_CH1N   D11   = MEANS f4_11
         { GPIOB, GPIO_PIN_0, GPIO_AF1_TIM1, TIMER_CHANNEL_2_N, TIM_CHANNEL_2 }, // =-PB0  TIM1_CH2N
         { GPIOB, GPIO_PIN_1, GPIO_AF1_TIM1, TIMER_CHANNEL_3_N, TIM_CHANNEL_3 }, // =-PB1  TIM1_CH3N
                     // Complementary - Alternates
         { GPIOB, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT1, TIM_CHANNEL_1}, // =-PB13 TIM1_CH1N
         { GPIOB, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT1, TIM_CHANNEL_2}, // =-PB14 TIM1_CH2N
         { GPIOB, GPIO_PIN_15,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT1, TIM_CHANNEL_3}, // =-PB15 TIM1_CH3N
#if defined(STM32F401xE)
         { GPIOE, GPIO_PIN_1, GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT2, TIM_CHANNEL_2 }, // -PE1  TIM1_CH2N
#endif
    // Alternates
         { GPIOE, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 },  // PE9  TIM1_CH1
         { GPIOE, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 },  // PE11 TIM1_CH2
         { GPIOE, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_3_ALT1, TIM_CHANNEL_3 },  // PE13 TIM1_CH3
         { GPIOE, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_4_ALT1, TIM_CHANNEL_4 },  // PE14 TIM1_CH4
                     // Complementary - Alternates
         { GPIOE, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT2, TIM_CHANNEL_1}, // PE8  TIM1_CH1N
         { GPIOE, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT2, TIM_CHANNEL_2}, // PE10 TIM1_CH2N
         { GPIOE, GPIO_PIN_12,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT2, TIM_CHANNEL_3}, // PE12 TIM1_CH3N
         { 0,          0,             0,              0,              0           }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_2_channels [] =                  // TIM2
       { {    0L,      0L,             0,           0,             0        }, // STM32           Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PA0  TIM2_CH1  A0
         { GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // =-PB3  TIM2_CH2  D3
         { GPIOB, GPIO_PIN_10,GPIO_AF1_TIM2, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // =-PB10 TIM2_CH3  D6
         { GPIOB, GPIO_PIN_11,GPIO_AF1_TIM2, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // =?PB11 TIM2_CH4 missing line on chart
    // Alternates
         { GPIOA, GPIO_PIN_5, GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // =-PA5  TIM2_CH1  D13
         { GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // =-PA1  TIM2_CH2  A1
         { GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3 }, // =-PA2  TIM2_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4 }, // =-PA3  TIM2_CH4  D0 VCP
         { GPIOA, GPIO_PIN_15,GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1 }, // =-PA15 TIM2_CH1 JTAG/JTDI
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_3_channels [] =                  // TIM3
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PB4  TIM3_CH1      D5
         { GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // =-PB5  TIM3_CH2      D4
         { GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // =-PB0  TIM3_CH3      A3
         { GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // =-PB1  TIM3_CH4
    // Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // =-PA6  TIM3_CH1 D12
         { GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 }, // =-PA7  TIM3_CH2 D11
         { GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT2, TIM_CHANNEL_1 }, // =-PC6  TIM3_CH1
         { GPIOC, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT2, TIM_CHANNEL_2 }, // =-PC7  TIM3_CH2 D9
         { GPIOC, GPIO_PIN_8, GPIO_AF2_TIM3, TIMER_CHANNEL_3_ALT2, TIM_CHANNEL_3 }, // =-PC8  TIM3_CH3
         { GPIOC, GPIO_PIN_9, GPIO_AF2_TIM3, TIMER_CHANNEL_4_ALT2, TIM_CHANNEL_4 }, // =-PC9  TIM3_CH4
         { 0,          0,             0,              0,              0          }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_4_channels [] =                  // TIM4
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PB6  TIM4_CH1     D10
         { GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // =-PB7  TIM4_CH2
         { GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // =-PB8  TIM4_CH3     D15
         { GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // =-PB9  TIM4_CH4     D14
    // Alternates
         { GPIOD, GPIO_PIN_12,GPIO_AF2_TIM4, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // =-PD12 TIM4_CH1
         { GPIOD, GPIO_PIN_13,GPIO_AF2_TIM4, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2}, // =-PD13 TIM4_CH2
         { GPIOD, GPIO_PIN_14,GPIO_AF2_TIM4, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3}, // =-PD14 TIM4_CH3
         { GPIOD, GPIO_PIN_14,GPIO_AF2_TIM4, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4}, // =-PD15 TIM4_CH4
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_5_channels [] =                  // TIM5
       { {   0L,      0L,            0,               0,              0     }, // STM32            Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5, TIMER_CHANNEL_1, TIM_CHANNEL_1 },    // =-PA0  TIM5_CH1
         { GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5, TIMER_CHANNEL_2, TIM_CHANNEL_2 },    // =-PA1  TIM5_CH2
         { GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5, TIMER_CHANNEL_3_ALT1, TIM_CHANNEL_3},// =-PA2  TIM5_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5, TIMER_CHANNEL_4_ALT1, TIM_CHANNEL_4},// =-PA3  TIM5_CH4  D0 VCP
         { 0,          0,             0,              0,              0     }     // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_9_channels [] =                 // TIM9
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOE, GPIO_PIN_5, GPIO_AF3_TIM9, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PE5 TIM9_CH1
         { GPIOE, GPIO_PIN_6, GPIO_AF3_TIM9, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // =-PE6 TIM9_CH2
     // Alternates
         { GPIOA, GPIO_PIN_2, GPIO_AF3_TIM9, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // =-PA2 TIM9_CH1   D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF3_TIM9, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2}, // =-PA3 TIM9_CH2   D0 VCP
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_10_channels [] =                // TIM10
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOB, GPIO_PIN_8, GPIO_AF3_TIM10,TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PB8 TIM10_CH1      D15
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_11_channels [] =                // TIM11
       { {   0L,      0L,            0,               0,              0     }, // STM32           Arduino
         { GPIOB, GPIO_PIN_9, GPIO_AF3_TIM11,TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // =-PB9 TIM11_CH1      D14
         { 0,          0,             0,              0,              0     }  // end of table
       };

                             //-----------------------------------------------------
                             // _g_tmrpwm_mod_channel_blk_lookup
                             //
                             //     Lookup table to get associated TMPWM_CHANNEL_BLK
                             //     entry for a given Timer module_id
                             //-----------------------------------------------------

const TMPWM_CHANNEL_BLK  *_g_tmrpwm_mod_channel_blk_lookup [] =
          { (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_0_channels[0],  // dummy TIM0
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_1_channels[0],  // TIM1
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_2_channels[0],  // TIM2
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_3_channels[0],  // TIM3
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_4_channels[0],  // TIM4
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_5_channels[0],  // TIM5
                                       0,                        // no TIM6
                                       0,                        // no TIM7
                                       0,                        // no TIM8
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_9_channels[0],  // TIM9
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_10_channels[0], // TIM10
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_11_channels[0], // TIM11  Last
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
        case 9:                          // TIM9
             __TIM9_CLK_ENABLE();
             break;
        case 10:                         // TIM10
             __TIM10_CLK_ENABLE();
             break;
        case 11:                         // TIM11
             __TIM11_CLK_ENABLE();
             break;
        default:
             return (ERR_TIMER_NUM_NOT_SUPPORTED);
     }
    return (0);          // denote everything worked OK
}

/******************************************************************************/
