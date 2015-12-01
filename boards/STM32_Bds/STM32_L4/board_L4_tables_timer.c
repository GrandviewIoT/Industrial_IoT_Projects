
// 07/03/15 - ADC TRIGGERing - associated TIMx CCR2's MMS field needs to be set
//            to 0x2 (Update) or 0x4 (CCR1, 0x5 (CCR2), 0x6 (CCR3), 0x7 (CCR4)
//            Right now, are only doing it for TIM6/7, but needs to be done
//            ofr ANY timer that needs to drive ADC (TIM1/TIM2/...)

//2******1*********2*********3*********4*********5*********6*********7**********
//                                                                      STM32 L4
//                            board_L4_tables_timer.c
//
//
//                              STM32  L4 76 VG   (Discovery)        STM32L476xx / USE_STM32L476G_DISCO_REVB
//                              STM32  L4 76 RG   (Nucleo)           STM32L476xx / USE_STM32L4XX_NUCLEO
//
//
//               MCU SPECIFIC   TIMER  and  PWM   GPIO  PIN   DEFINITIONS
//
//                                     for
//
//                            PWM   and   OC/IC   Timers
//  History:
//    08/18/15 - Verified tables. Duq
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
//                    Timer / PWM Summary     available on STM32 L4
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
//    TIM15       2           Yes (1)       Yes       Up only         Yes
//    TIM16       1           Yes           Yes       Up only         Yes
//    TIM17       1           Yes           Yes       Up only         Yes
//
// Note: TIM6 and TIM7 are have _NO_ pin outputs on the L4 chip.
//       They are strictly "Basic" Timers only (but can be used for internal
//       triggers, such as for DACs).
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                           Timer Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_16  (AF14)
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_17  (AF14)
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//                 TIMER_2
//   D13    PA_5   TIMER_CHANNEL_1   -                     -

//                 TIMER_3                                  TIMER_16  (AF14)
//   D12    PA_6   TIMER_CHANNEL_1_ALT2                     TIMER_CHANNEL_1

//   TIMER_41            TIMER_3               TIMER_17  (AF14)
//   D11    PA_7   TIMER_CHANNEL_1_N  TIMER_CHANNEL_2_ALT2  TIMER_CHANNEL_1

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

//                 TIMER_2            TIMER_5                TIMER_9          TIMER_15 (AF14)
//   D1     PA_2   TIMER_CHANNEL_3    TIMER_CHANNEL_3_ALT1   TIMER_CHANNEL_1  TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5                TIMER_9          TIMER_15 (AF14)
//   D0     PA_3   TIMER_CHANNEL_4    TIMER_CHANNEL_4_ALT1   TIMER_CHANNEL_2  TIMER_CHANNEL_2


//                 TIMER_2            TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1    TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5                TIMER_15 (AF14)
//   A1     PA_1   TIMER_CHANNEL_2    TIMER_CHANNEL_21       TIMER_CHANNEL_1

//   A2     PA_4   - none -

//                 TIMER_3            TIMER_1                TIMER_8
//   A3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N1     TIMER_CHANNEL_2_N

//   A4     PC_1   - none -

//   A5     PC_0   - none -

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4

//                 TIMER_3            TIMER_1                TIMER_8
//   CNx    PB_1   TIMER_CHANNEL_4    TIMER_CHANNEL_3_N      TIMER_CHANNEL_3_N
//                 TIMER_4            TIMER_17 (AF14)
//   CNx    PB_7   TIMER_CHANNEL_2    TIMER_CHANNEL_1_N
//                 TIMER_2
//   CNx    PB_11  TIMER_CHANNEL_4_ALT1
//          PB_12 - none -
//                 TIMER_1                                   TIMER_15
//   CNx    PB_13  TIMER_CHANNEL_1_N_ALT                     TIMER_CHANNEL_1_N
//                 TIMER_1            TIMER_8                TIMER_15
//   CNx    PB_14  TIMER_CHANNEL_2_N  TIMER_CHANNEL_2_N      TIMER_CHANNEL_1
//                 TIMER_1            TIMER_8                TIMER_15
//   CNx    PB_15  TIMER_CHANNEL_3_N  TIMER_CHANNEL_3_N      TIMER_CHANNEL_2

//   PC_2 - PC_5   - none -
//                 TIMER_3            TIMER_8
//   CNx    PC_6   TIMER_CHANNEL_1    TIMER_CHANNEL_1
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

//                 TIMER_16  (AF14)
//   CNx    PE_0   TIMER_CHANNEL_1
//                 TIMER_17  (AF14)
//   CNx    PE_1   TIMER_CHANNEL_1
//          PE_2   - none -
//                 TIMER_3
//   CNx    PE_3   TIMER_CHANNEL_1
//                 TIMER_3
//   CNx    PE_4   TIMER_CHANNEL_2
//                 TIMER_3
//   CNx    PE_5   TIMER_CHANNEL_4
//                 TIMER_3
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
//                 TIMER_5
//   CNx    PF_6   TIMER_CHANNEL_1
//                 TIMER_5
//   CNx    PF_7   TIMER_CHANNEL_2
//                 TIMER_5
//   CNx    PF_8   TIMER_CHANNEL_3
//                 TIMER_5
//   CNx    PF_9   TIMER_CHANNEL_4
//                 TIMER_15
//   CNx    PF_10  TIMER_CHANNEL_1
//                 TIMER_15
//   CNx    PF_11  TIMER_CHANNEL_2
//  PF_10 - PF1_5  - none -

//   PG_0 - PG_8   - none -
//                 TIMER_15
//   CNx    PG_9   TIMER_CHANNEL_1_N
//                 TIMER_15
//   CNx    PG_10  TIMER_CHANNEL_1
//                 TIMER_15
//   CNx    PG_11  TIMER_CHANNEL_2   - Last timer pin

//------------------------------------------------------------------------------

#define  MAX_TIMER        TIMER_17

#define  TMRPWM_NUM_MODULES     17  // We allow a range of 1 to 17 for Timer
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
                                    TIM6,  //   ditto
                                    TIM7,  //   ditto
                                    TIM8,  //   full, w/deadtime, w/complementary,
                                     0L,   // no TIM9  on L4
                                     0L,   // no TIM10 on L4
                                     0L,   // no TIM11 on L4
                                     0L,   // no TIM12 on L4
                                     0L,   // no TIM13 on L4
                                     0L,   // no TIM14 on L4
                                    TIM15, //
                                    TIM16, //
                                    TIM17  //      last valid timer on L4
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
    TIM_HandleTypeDef  _g_TIM15_TimPwmHandle;  // Timer/PWM  handle for TIM15
    TIM_HandleTypeDef  _g_TIM16_TimPwmHandle;  // Timer/PWM  handle for TIM16
    TIM_HandleTypeDef  _g_TIM17_TimPwmHandle;  // Timer/PWM  handle for TIM17


                             //----------------------------------------------------------
                             //  List of HAL Handle Address for each separate TIM Handle
                             //----------------------------------------------------------
const TIM_HandleTypeDef *_g_timer_typedef_handle []              /* HAL API Handles */
                                = {     0L,                      // no TIM0  on F4
                                    (TIM_HandleTypeDef*) &_g_TIM1_TimPwmHandle,  // TIM1
                                    (TIM_HandleTypeDef*) &_g_TIM2_TimPwmHandle,  // TIM2
                                    (TIM_HandleTypeDef*) &_g_TIM3_TimPwmHandle,  // TIM3
                                    (TIM_HandleTypeDef*) &_g_TIM4_TimPwmHandle,  // TIM4
                                    (TIM_HandleTypeDef*) &_g_TIM5_TimPwmHandle,  // TIM5
                                    (TIM_HandleTypeDef*) &_g_TIM6_TimPwmHandle,  // TIM6
                                    (TIM_HandleTypeDef*) &_g_TIM7_TimPwmHandle,  // TIM7
                                    (TIM_HandleTypeDef*) &_g_TIM8_TimPwmHandle,  // TIM8
                                                                  0L,            // no TIM9
                                                                  0L,            // no TIM10
                                                                  0L,            // no TIM11
                                                                  0L,            // no TIM12
                                                                  0L,            // no TIM13
                                                                  0L,            // no TIM14
                                    (TIM_HandleTypeDef*) &_g_TIM15_TimPwmHandle, // TIM15
                                    (TIM_HandleTypeDef*) &_g_TIM16_TimPwmHandle, // TIM16
                                    (TIM_HandleTypeDef*) &_g_TIM17_TimPwmHandle  // TIM17
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
                                    TIM6_DAC_IRQn,  //  TIMER_6 update only
                                    TIM7_IRQn,      //  TIMER_7 update only
                                    TIM8_CC_IRQn,   //  TIMER_8 CCRs only
// ??? need to handle TIM8 rollovers as specuial case ala TIM1 (TIM8_UP_TIM13_IRQn)
                                         0,         //  no TIMER_9
                                         0,         //  no TIMER_10
                                         0,         //  no TIMER_11
                                         0,         //  no TIMER_12
                                         0,         //  no TIMER_13
                                         0,         //  no TIMER_14
                                    TIM1_BRK_TIM15_IRQn,    //  TIMER_15 update and CCRs
                                    TIM1_UP_TIM16_IRQn,     //  TIMER_16   "
                                    TIM1_TRG_COM_TIM17_IRQn,//  TIMER_17   "
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
       { {   0L,      0L,             0,              0,              0     }, // STM32          Arduino
         { GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA8  TIM1_CH1     D7
         { GPIOA, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PA9  TIM1_CH2     D8
         { GPIOA, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PA10 TIM1_CH3     D2
         { GPIOA, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PA11 TIM1_CH4
                     // Complementary pins
         { GPIOA, GPIO_PIN_7, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N, TIM_CHANNEL_1 }, // PA7  TIM1_CH1N  D11
         { GPIOB, GPIO_PIN_0, GPIO_AF1_TIM1, TIMER_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB0  TIM1_CH2N  A3
         { GPIOB, GPIO_PIN_1, GPIO_AF1_TIM1, TIMER_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB1  TIM1_CH3N
                     // Complementary - Alternates
         { GPIOB, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT1, TIM_CHANNEL_1}, // PB13 TIM1_CH1N
         { GPIOB, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT1, TIM_CHANNEL_2}, // PB14 TIM1_CH2N
         { GPIOB, GPIO_PIN_15,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT1, TIM_CHANNEL_3}, // PB15 TIM1_CH3N
    // Alternates
         { GPIOE, GPIO_PIN_9, GPIO_AF1_TIM1, TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PE9  TIM1_CH1
         { GPIOE, GPIO_PIN_11,GPIO_AF1_TIM1, TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 }, // PE11 TIM1_CH2
         { GPIOE, GPIO_PIN_13,GPIO_AF1_TIM1, TIMER_CHANNEL_3_ALT1, TIM_CHANNEL_3 }, // PE13 TIM1_CH3
         { GPIOE, GPIO_PIN_14,GPIO_AF1_TIM1, TIMER_CHANNEL_4_ALT1, TIM_CHANNEL_1 }, // PE14 TIM1_CH4
                     // Complementary - Alternates
         { GPIOE, GPIO_PIN_8, GPIO_AF1_TIM1, TIMER_CHANNEL_1_N_ALT2, TIM_CHANNEL_1 }, // PE8  TIM1_CH1N
         { GPIOE, GPIO_PIN_10,GPIO_AF1_TIM1, TIMER_CHANNEL_2_N_ALT2, TIM_CHANNEL_2 }, // PE10 TIM1_CH2N
         { GPIOE, GPIO_PIN_12,GPIO_AF1_TIM1, TIMER_CHANNEL_3_N_ALT2, TIM_CHANNEL_3 }, // PE12 TIM1_CH3N
         { 0,          0,             0,              0,              0            }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_2_channels [] =                  // TIM2
       { {    0L,      0L,             0,           0,             0        }, // STM32         Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM2_CH1    A0
         { GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB3  TIM2_CH2    D3
         { GPIOB, GPIO_PIN_10,GPIO_AF1_TIM2, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB10 TIM2_CH3    D6
         { GPIOB, GPIO_PIN_11,GPIO_AF1_TIM2, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB11 TIM2_CH4
    // Alternates
         { GPIOA, GPIO_PIN_5, GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PA5  TIM2_CH1
         { GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PA1  TIM2_CH2  A1
         { GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3 }, // PA2  TIM2_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF1_TIM2, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4 }, // PA3  TIM2_CH4  D0 VCP
         { GPIOA, GPIO_PIN_15,GPIO_AF1_TIM2, TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1 }, // PA15 TIM2_CH1 JTAG/JTDI
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_3_channels [] =                  // TIM3
       { {   0L,      0L,            0,               0,              0     }, // STM32            Arduino
         { GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PB4  TIM3_CH1      D5
         { GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB5  TIM3_CH2      D4
         { GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB0  TIM3_CH3      A3
         { GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB1  TIM3_CH4
    // Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PA6  TIM3_CH1  D12
         { GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PA7  TIM3_CH2  D11
         { GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1 }, // PC6  TIM3_CH1
         { GPIOC, GPIO_PIN_7, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT2,TIM_CHANNEL_2 }, // PC7  TIM3_CH2
         { GPIOC, GPIO_PIN_8, GPIO_AF2_TIM3, TIMER_CHANNEL_3_ALT2,TIM_CHANNEL_3 }, // PC8  TIM3_CH3
         { GPIOC, GPIO_PIN_9, GPIO_AF2_TIM3, TIMER_CHANNEL_4_ALT2,TIM_CHANNEL_4 }, // PC9  TIM3_CH4
    // Alternates
         { GPIOE, GPIO_PIN_3, GPIO_AF2_TIM3, TIMER_CHANNEL_1_ALT3,TIM_CHANNEL_1 }, // PE3  TIM3_CH1
         { GPIOE, GPIO_PIN_4, GPIO_AF2_TIM3, TIMER_CHANNEL_2_ALT3,TIM_CHANNEL_2 }, // PE4  TIM3_CH2
         { GPIOE, GPIO_PIN_5, GPIO_AF2_TIM3, TIMER_CHANNEL_3_ALT3,TIM_CHANNEL_3 }, // PE5  TIM3_CH3
         { GPIOE, GPIO_PIN_6, GPIO_AF2_TIM3, TIMER_CHANNEL_4_ALT3,TIM_CHANNEL_4 }, // PE6  TIM3_CH4
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_4_channels [] =                  // TIM4
       { {   0L,      0L,            0,               0,              0     }, // STM32        Arduino
         { GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PB6  TIM4_CH1
         { GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PB7  TIM4_CH2
         { GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PB8  TIM4_CH3
         { GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PB9  TIM4_CH4
    // Alternates
         { GPIOD, GPIO_PIN_12,GPIO_AF2_TIM4, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PD12 TIM4_CH1
         { GPIOD, GPIO_PIN_13,GPIO_AF2_TIM4, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PD13 TIM4_CH2
         { GPIOD, GPIO_PIN_14,GPIO_AF2_TIM4, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PD14 TIM4_CH3
         { GPIOD, GPIO_PIN_15,GPIO_AF2_TIM4, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PD15 TIM4_CH4
         { 0,          0,             0,              0,              0     }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_5_channels [] =                  // TIM5
       { {   0L,      0L,            0,               0,              0     }, // STM32          Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM5_CH1      A0
         { GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PA1  TIM5_CH2      A1
         { GPIOF, GPIO_PIN_8, GPIO_AF2_TIM5, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PF8  TIM5_CH3
         { GPIOF, GPIO_PIN_9, GPIO_AF2_TIM5, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PF9  TIM5_CH4
    // Alternates
         { GPIOF, GPIO_PIN_6, GPIO_AF2_TIM5, TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PF6  TIM5_CH1
         { GPIOF, GPIO_PIN_7, GPIO_AF2_TIM5, TIMER_CHANNEL_2_ALT1,TIM_CHANNEL_2 }, // PF7  TIM5_CH2
         { GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5, TIMER_CHANNEL_3_ALT1,TIM_CHANNEL_3 }, // PA2  TIM5_CH3  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5, TIMER_CHANNEL_4_ALT1,TIM_CHANNEL_4 }, // PA3  TIM5_CH4  D0 VCP
         { 0,          0,             0,              0,              0     }  // end of table
       };

              //--------------------------------------------------------------------------------
              // Note that TIMER6 and TIMER7 have no external pins for PWM, IC, nor OC Channels
              //--------------------------------------------------------------------------------

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_8_channels [] =                  // TIM8
       { {   0L,      0L,             0,              0,              0       }, // STM32         Arduino
         { GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8, TIMER_CHANNEL_1, TIM_CHANNEL_1 }, // PC6  TIM8_CH1
         { GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8, TIMER_CHANNEL_2, TIM_CHANNEL_2 }, // PC7  TIM8_CH2     D9
         { GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8, TIMER_CHANNEL_3, TIM_CHANNEL_3 }, // PC8  TIM8_CH3
         { GPIOC, GPIO_PIN_9, GPIO_AF3_TIM8, TIMER_CHANNEL_4, TIM_CHANNEL_4 }, // PC9  TIM8_CH4
                        // Complementary pins
         { GPIOA, GPIO_PIN_5, GPIO_AF3_TIM8, TIMER_CHANNEL_1_N, TIM_CHANNEL_1 }, // PA5 TIM8_CH1N   D13
         { GPIOB, GPIO_PIN_0, GPIO_AF3_TIM8, TIMER_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB0 TIM8_CH2N   A3
         { GPIOB, GPIO_PIN_1, GPIO_AF3_TIM8, TIMER_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB1 TIM8_CH3N
                        // Complementary - Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF3_TIM8, TIMER_CHANNEL_1_N_ALT1,TIM_CHANNEL_1}, // PA6  TIM8_CH1N D12
         { GPIOB, GPIO_PIN_14,GPIO_AF3_TIM8, TIMER_CHANNEL_2_N_ALT1,TIM_CHANNEL_2}, // PB14 TIM8_CH2N
         { GPIOB, GPIO_PIN_15,GPIO_AF3_TIM8, TIMER_CHANNEL_3_N_ALT1,TIM_CHANNEL_3}, // PB15 TIM8_CH3N
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_15_channels [] =                // TIM15
       { {   0L,       0L,          0,                0,               0    }, // STM32            Arduino
         { GPIOB, GPIO_PIN_14, GPIO_AF14_TIM15,TIMER_CHANNEL_1, TIM_CHANNEL_1},  // PB14 TIM15_CH1
         { GPIOB, GPIO_PIN_15, GPIO_AF14_TIM15,TIMER_CHANNEL_2, TIM_CHANNEL_2},  // PB15 TIM15_CH2
                        // Complementary pins
         { GPIOA, GPIO_PIN_1, GPIO_AF14_TIM15,TIMER_CHANNEL_1_N,TIM_CHANNEL_1},     // PA1  TIM15_CH1N A1
         { GPIOB, GPIO_PIN_15,GPIO_AF14_TIM15,TIMER_CHANNEL_1_N_ALT1,TIM_CHANNEL_1},// PB13 TIM15_CH1N
         { GPIOG, GPIO_PIN_9, GPIO_AF14_TIM15,TIMER_CHANNEL_1_N_ALT3,TIM_CHANNEL_1},// PG9 TIM15_CH1N
    // Alternates
         { GPIOA, GPIO_PIN_2, GPIO_AF14_TIM15,TIMER_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PA2  TIM15_CH1  D1 VCP
         { GPIOA, GPIO_PIN_3, GPIO_AF14_TIM15,TIMER_CHANNEL_2_ALT1, TIM_CHANNEL_2 }, // PA3  TIM15_CH2  D0 VCP
         { GPIOF, GPIO_PIN_9, GPIO_AF14_TIM15,TIMER_CHANNEL_1_ALT2, TIM_CHANNEL_1},  // PF9  TIM15_CH1
         { GPIOF, GPIO_PIN_10,GPIO_AF14_TIM15,TIMER_CHANNEL_2_ALT2, TIM_CHANNEL_2},  // PF10 TIM15_CH2
         { GPIOG, GPIO_PIN_10,GPIO_AF14_TIM15,TIMER_CHANNEL_1_ALT3, TIM_CHANNEL_1},  // PG10 TIM15_CH1
         { GPIOG, GPIO_PIN_11,GPIO_AF14_TIM15,TIMER_CHANNEL_2_ALT3, TIM_CHANNEL_2},  // PF11 TIM15_CH2
         { 0,          0,             0,              0,              0          }   // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_16_channels [] =                // TIM16
       { {   0L,       0L,             0,            0,                 0   },  // STM32           Arduino
         { GPIOB, GPIO_PIN_8, GPIO_AF14_TIM16,TIMER_CHANNEL_1, TIM_CHANNEL_1 },  // PB8 TIM16_CH1      D15
                     // Complementary pins
         { GPIOB, GPIO_PIN_6, GPIO_AF14_TIM16,TIMER_CHANNEL_1_N, TIM_CHANNEL_1}, // PB6 TIM16_CH1N     D10
    // Alternates
         { GPIOA, GPIO_PIN_6, GPIO_AF14_TIM16,TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1},// PA6 TIM16_CH1    D12
         { GPIOE, GPIO_PIN_0, GPIO_AF14_TIM16,TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1},// PE0 TIM16_CH1
         { 0,          0,             0,              0,              0         } // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_17_channels [] =                // TIM17
       { {   0L,       0L,             0,            0,                0   },   // STM32           Arduino
         { GPIOB, GPIO_PIN_9, GPIO_AF14_TIM17,TIMER_CHANNEL_1,  TIM_CHANNEL_1}, // PB9 TIM17_CH1      D14
                     // Complementary pins
         { GPIOB, GPIO_PIN_7, GPIO_AF14_TIM17,TIMER_CHANNEL_1_N,TIM_CHANNEL_1}, // PB7 TIM17_CH1N
    // Alternates
         { GPIOA, GPIO_PIN_7, GPIO_AF14_TIM17,TIMER_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PA7 TIM17_CH1   D11
         { GPIOE, GPIO_PIN_1, GPIO_AF14_TIM17,TIMER_CHANNEL_1_ALT2,TIM_CHANNEL_1}, // PE1 TIM17_CH1
         { 0,          0,             0,              0,              0         }  // end of table
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
                                   0,             // TIM6 has no channels > CCR0
                                   0,             // TIM7 has no channels > CCR0
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_8_channels[0],  // TIM8
                                   0,                            // no TIM9
                                   0,                            // no TIM10
                                   0,                            // no TIM11
                                   0,                            // no TIM12
                                   0,                            // no TIM13
                                   0,                            // no TIM14
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_15_channels[0], // TIM15
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_16_channels[0], // TIM16
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_17_channels[0], // TIM17
          };


//                 TIMER_2              TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1      TIMER_CHANNEL_1
//                 TIMER_2              TIMER_5              TIMER_15    (AF14)
//   A1     PA_1   TIMER_CHANNEL_2      TIMER_CHANNEL_2      TIMER_CHANNEL_1_N
//                 TIMER_2              TIMER_5              TIMER_15    (AF14)
//   D1     PA_2   TIMER_CHANNEL_3_ALT1 TIMER_CHANNEL_3_ALT1 TIMER_CHANNEL_1_ALT1  -- THESE PINS ARE USELESS TX/RX --
//                 TIMER_2              TIMER_5              TIMER_15    (AF14)
//   D0     PA_3   TIMER_CHANNEL_4_ALT1 TIMER_CHANNEL_4_ALT1 TIMER_CHANNEL_2_ALT1  -- THESE PINS ARE USELESS TX/RX --
//                 TIMER_2              TIMER_8
//   D13    PA_5   TIMER_CHANNEL_1_ALT1 TIMER_CHANNEL_1_N         -
//                 TIMER_3                                   TIMER_16    (AF14)
//   D12    PA_6   TIMER_CHANNEL_1_ALT1 TIM8_BKIN            TIMER_CHANNEL_1_ALT1
//                 TIMER_1              TIMER_3              TIMER_8                 TIMER_17  (AF14)
//   D11    PA_7   TIMER_CHANNEL_1_N    TIMER_CHANNEL_2_ALT1 TIMER_CHANNEL_1_N_ALT1  TIMER_CHANNEL_1_ALT1
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
//                 TIMER_4            TIMER_16     (AF14)
//   D10    PB_6   TIMER_CHANNEL_1    TIMER_CHANNEL_1_N
//                 TIMER_4            TIMER_17     (AF14)
//   CNx    PB_7   TIMER_CHANNEL_2    TIMER_CHANNEL_1_N
//                 TIMER_4            TIMER_16     (AF14)
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1
//                 TIMER_4            TIMER_17     (AF14)
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1
//                 TIMER_2
//   D6     PB_10  TIMER_CHANNEL_3
//                 TIMER_2
//   CNx    PB_11  TIMER_CHANNEL_4_ALT1
//                 TIMER_1                                        TIMER_15     (AF14)
//   CNx    PB_13  TIMER_CHANNEL_1_N_ALT1                         TIMER_CHANNEL_1_N_ALT1
//                 TIMER_1                 TIMER_8                TIMER_15     (AF14)
//   CNx    PB_14  TIMER_CHANNEL_2_N_ALT1  TIMER_CHANNEL_2_N_ALT1 TIMER_CHANNEL_1
//                 TIMER_1                 TIMER_8                TIMER_15     (AF14)
//   CNx    PB_15  TIMER_CHANNEL_3_N_ALT1  TIMER_CHANNEL_3_N_ALT1 TIMER_CHANNEL_2


//                 TIMER_3              TIMER_8
//   CNx    PC_6   TIMER_CHANNEL_1_ALT2 TIMER_CHANNEL_1
//                 TIMER_3              TIMER_8
//   D9     PC_7   TIMER_CHANNEL_2_ALT2 TIMER_CHANNEL_2
//                 TIMER_3              TIMER_8
//   CNx    PC_8   TIMER_CHANNEL_3_ALT2 TIMER_CHANNEL_3
//                 TIMER_3              TIMER_8
//   CNx    PC_9   TIMER_CHANNEL_4_ALT2 TIMER_CHANNEL_4


//                 TIMER_4
//   CNx    PD_12  TIMER_CHANNEL_1_ALT1
//                 TIMER_4
//   CNx    PD_13  TIMER_CHANNEL_2_ALT1
//                 TIMER_4
//   CNx    PD_14  TIMER_CHANNEL_3_ALT1
//                 TIMER_4
//   CNx    PD_15  TIMER_CHANNEL_4_ALT1


//                 TIMER_16     (AF14)
//   CNx    PE_0   TIMER_CHANNEL_1_ALT2
//                 TIMER_17     (AF14)
//   CNx    PE_1   TIMER_CHANNEL_1_ALT2
//                 TIMER_3
//   CNx    PE_3   TIMER_CHANNEL_1_ALT3
//                 TIMER_3
//   CNx    PE_4   TIMER_CHANNEL_2_ALT3
//                 TIMER_3
//   CNx    PE_5   TIMER_CHANNEL_3_ALT3
//                 TIMER_3
//   CNx    PE_6   TIMER_CHANNEL_4_ALT3
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


//                 TIMER_5
//   CNx    PF_6   TIMER_CHANNEL_1_ALT1
//                 TIMER_5
//   CNx    PF_7   TIMER_CHANNEL_2_ALT1
//                 TIMER_5
//   CNx    PF_8   TIMER_CHANNEL_3
//                 TIMER_5
//                 TIMER_5                   TIMER_15      (AF14)
//   CNx    PF_9   TIMER_CHANNEL_4           TIMER_CHANNEL_1_ALT2
//                                           TIMER_15      (AF14)
//   CNx    PF_10                            TIMER_CHANNEL_2_ALT2


//                 TIMER_15     (AF14)
//   CNx    PG_9   TIMER_CHANNEL_1_N_ALT3
//                 TIMER_15     (AF14)
//   CNx    PG_10  TIMER_CHANNEL_1_ALT3
//                 TIMER_15     (AF14)
//   CNx    PG_11  TIMER_CHANNEL_2_ALT3   - Last timer pin





//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************


//******************************************************************************
//  board_timerpwm_compute_prescalar
//
//            Dynamically compute pre-scalar value needed to make user
//            requested period fit into 16-bit PWM counters.
//
//            Max clock on L4 is 40 MHz.     
//******************************************************************************
long  board_timerpwm_compute_prescalar (long period_val)
{
    long   prescalar;

       // Compute any needed prescaler value to fit 16-bit TIMx counter clock
    if (period_val > 6550000)            // 6.50 M ticks
       prescalar = 620;
       else if (period_val > 655000)     // 0.65 M ticks
               prescalar = 62;
       else if (period_val > 65500)      // 0.065 M ticks
               prescalar = 6;
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
        case 15:                         // TIM15
             __TIM15_CLK_ENABLE();
             break;
        case 16:                         // TIM16
             __TIM16_CLK_ENABLE();
             break;
        case 17:                         // TIM17
             __TIM17_CLK_ENABLE();
             break;
        default:
             return (ERR_TIMER_NUM_NOT_SUPPORTED);
     }
    return (0);          // denote everything worked OK
}

/******************************************************************************/
