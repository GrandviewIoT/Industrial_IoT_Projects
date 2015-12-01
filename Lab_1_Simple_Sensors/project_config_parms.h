
/********1*********2*********3*********4*********5*********6*********7**********
*
*                            project_config_parms.h
*
* Provides a set of network and debug parms that are used ONLY for THIS specific
* projects.
*
* It is used to specify such things as CC3100/CC3200 Access Point and Password
* info, common BLE parms, whether the DEBUG_LOG or CONSOLE_WRITE/READ is to be
* used, how long we should wait for console config information, etc.
*
* It should only be used when you need to overrride the config parameters for a
* specific project.
*
* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
*
* The MIT License (MIT)
*
* Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*******************************************************************************/

#ifndef __PROJ_CONF_PARMS_H__
#define __PROJ_CONF_PARMS_H__

// In this example, we are overriding just the CONSOLE_WRITE/CONSOLE_READ
// parameters (so we can write messages out to the UART). We then us the rest
// of the Network parameters that were setup  in default_project_config_parms.h

#if defined(USE_STM32L4XX_NUCLEO) || defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F3XX_NUCLEO)
        // CONSOLE IS BROKE ON Nucleo L4 and L0 and F3 - hangs forever waiting on TXBE
        //  - do not enable CONSOLE I/O -
#else
#define USES_CONSOLE_WRITE          1
#define USES_CONSOLE_READ           1
#endif

//#define CONSOLE_STARTUP_INPUT_WAIT  5


           //---------------------------------------------
           //  put any project specific settings in here.
           //---------------------------------------------


#if defined(STM32F072xB)
             //-------------------------------
             //         STM32 F0 72
             //-------------------------------
//    F0 72 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//    F0 91    ditto                           D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//
 #if defined(USE_STM32F0XX_NUCLEO)
             //----------------------------------
             //       STM32 F0 72   Nucleo
             //----------------------------------
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_15
#define  ADC_TRIGGER_TIMER   TIMER_15

#define  DAC_1_TRIGGER_TYPE  DAC_TRIGGER_TIMER_6     /* DAC1 = A2   (PA4) */
#define  DAC_1_TRIGGER_TIMER TIMER_6
#define  DAC_2_TRIGGER_TYPE  DAC_TRIGGER_TIMER_7     /* DAC2 = D13  (PA5) */
#define  DAC_2_TRIGGER_TIMER TIMER_7

#define  TIMER_XR            TIMER_14        // XR = Interrupt on Rollover

#define  TIMER_OC            TIMER_3         // OC = Output compare test timer
#define   OC_CHANNEL_A        TIMER_CHANNEL_1      /* drives D5  pin for LED     */
#define   OC_CHANNEL_B        TIMER_CHANNEL_2_ALT1 /* shunt to D11 pin for OScope*/

#define  PWM_A_MODULE        TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   PWM_A_TEST_CHAN_A   PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   PWM_A_TEST_CHAN_B   PWM_CHANNEL_3     /* drives BLUE LED on D6     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED1     D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */

#define  MANUAL_RED_LED1     D4                 /*  PB_5   D4  -MANUAL- */
#define  MANUAL_RED_LED2     D8                 /*  PA_9   D8  -MANUAL- */

#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */

 #else
             //----------------------------------
             //       STM32 F0 72   Discovery    <<== CAUTION: this is being turned on even for NUCLEO builds ??? !!!
             //----------------------------------
      // Discovery has 4 built in LEDs on pins:
      //      LD3  Orange  PC8  TIM3_CH3  AF_0
      //      LD4  Green   PC9  TIM3_CH4  AF_0
      //      LD5  Red     PC6  TIM3_CH1  AF_0
      //      LD6  Blue    PC7  TIM3_CH2  AF_0
      //
      // Advanced Sensors:
      //   It also contains: L3GD20 - ST MEMS motion sensor
      //                            a 3-axis digital output gyroscope
      //                     Cap Touch linear sensor or four touch keys

#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_15
#define  ADC_TRIGGER_TIMER   TIMER_15

#define  TIMER_XR            TIMER_14        // XR = Interrupt on Rollover

#define  TIMER_OC            TIMER_3         // OC = Output compare test timer
#define   OC_CHANNEL_A        TIMER_CHANNEL_3_ALT2  // PC8  LD3  Orange
#define   OC_CHANNEL_B        TIMER_CHANNEL_2_ALT2  // PC7  LD6  Blue

#define  PWM_A_MODULE        TIMER_OC        // PWM and OC share the same module
#define   PWM_A_TEST_CHAN_A   TIMER_CHANNEL_1_ALT2  // PC6  LD5  Red
#define   PWM_A_TEST_CHAN_B   TIMER_CHANNEL_4_ALT2  // PC9  LD4  Green

#define  RED_LED1            D3              //  D3   PB_3
#define  RED_LED2            D6              //  D6   PB_10

#define  MAG_READ_SW1        D2              //  D2   PA_10
#define  FIXED_SLIDE_SW1     D7              //  D7   PA_8
 #endif                                      // #if NUCLEO
#endif


#if defined(STM32F303xE) && defined(USE_STM32F3XX_NUCLEO)
             //-------------------------------
             //         STM32 F3 03
             //-------------------------------
//    F3 03 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_4_CC4
#define  ADC_TRIGGER_TIMER   TIMER_4

#define  LED_PWM_MODULE_1    TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   LED_1_A_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   LED_1_B_CHANNEL     PWM_CHANNEL_3     /* drives BLUE LED on D6     */
#define  LED_PWM_MODULE_2    TIMER_3            /* drives RED  LEDs on D5/D6 */
#define   LED_2_A_CHANNEL     PWM_CHANNEL_1     /* drives BLUE LED on D5     */
#define   LED_2_B_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D4     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED      D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */
#define  SEEED_RED_LED       D5                 /*  PB_4   D5  TIM3_CH1 */
#define  SEEED_RED_LED2      D4                 /*  PB_5   D4  TIM3_CH2 */
#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */
#endif


#if defined(STM32F334x8) && defined(USE_STM32F3XX_NUCLEO)
             //-------------------------------
             //         STM32 F3 34
             //-------------------------------
//    F3 34 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_15
#define  ADC_TRIGGER_TIMER   TIMER_15

#define  DAC_1_TRIGGER_TYPE  DAC_TRIGGER_TIMER_6     /* DAC1 = A2   (PA4) */
#define  DAC_1_TRIGGER_TIMER TIMER_6
#define  DAC_2_TRIGGER_TYPE  DAC_TRIGGER_TIMER_7     /* DAC2 = D13  (PA5) */
#define  DAC_2_TRIGGER_TIMER TIMER_7

#define  TIMER_XR            TIMER_1           // XR = Interrupt on Rollover

#define  TIMER_OC            TIMER_3           // OC = Output compare test timer
#define   OC_CHANNEL_A        TIMER_CHANNEL_1      /* drives D5  pin for LED     */
#define   OC_CHANNEL_B        TIMER_CHANNEL_2_ALT1 /* shunt to D11 pin for OScope*/

#define  PWM_A_MODULE        TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   PWM_A_TEST_CHAN_A   PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   PWM_A_TEST_CHAN_B   PWM_CHANNEL_3     /* drives BLUE LED on D6     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED1     D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */

#define  MANUAL_RED_LED1     D4                 /*  PB_5   D4  -MANUAL- */
#define  MANUAL_RED_LED2     D8                 /*  PA_9   D8  -MANUAL- */

#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */

#endif


#if defined(STM32F401xE) || defined(STM32F446xx) || defined(USE_STM32L4XX_NUCLEO)
             //------------------------------------------
             //       STM32  F4 01  /  F4 46  /  L4 76
             //------------------------------------------
//    F4 01 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//    F4 11    ditto                           D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//    F4 46 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//    L4 76 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//
//#define  ADC_TRIGGER     TRIGGER_TIMER_5_CC1   // OUCH - IS BROKE RIGHT NOW 07/03/15  ??? !!!  ARGGG
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_4_CC4
#define  ADC_TRIGGER_TIMER   TIMER_4

#define  LED_PWM_MODULE_1    TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   LED_1_A_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   LED_1_B_CHANNEL     PWM_CHANNEL_3     /* drives BLUE LED on D6     */
#define  LED_PWM_MODULE_2    TIMER_3            /* drives RED  LEDs on D5/D6 */
#define   LED_2_A_CHANNEL     PWM_CHANNEL_1     /* drives BLUE LED on D5     */
#define   LED_2_B_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D4     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED      D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */
#define  SEEED_RED_LED       D5                 /*  PB_4   D5  TIM3_CH1 */
#define  SEEED_RED_LED2      D4                 /*  PB_5   D4  TIM3_CH2 */
#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */
#endif


#if  defined(USE_STM32F7_DISCO)
             //-------------------------------
             //         STM32 F7 46
             //-------------------------------
//    F7 46 PWM outputs are easily available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
//                                             D7 (TIM1_CH1),   D8 (TIM1_CH2)
//
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_4_CC4
#define  ADC_TRIGGER_TIMER   TIMER_4

#define  LED_PWM_MODULE_1    TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   LED_1_A_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   LED_1_B_CHANNEL     PWM_CHANNEL_3     /* drives BLUE LED on D6     */
#define  LED_PWM_MODULE_2    TIMER_3            /* drives RED  LEDs on D5/D6 */
#define   LED_2_A_CHANNEL     PWM_CHANNEL_1     /* drives BLUE LED on D5     */
#define   LED_2_B_CHANNEL     PWM_CHANNEL_2     /* drives BLUE LED on D4     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED      D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */
#define  SEEED_RED_LED       D5                 /*  PB_4   D5  TIM3_CH1 */
#define  SEEED_RED_LED2      D4                 /*  PB_5   D4  TIM3_CH2 */
#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */
#endif


#if  defined(USE_STM32L0XX_NUCLEO)
             //-------------------------------
             //         STM32 L0 53
             //-------------------------------
// Note that the L0 is fairly constrained in terms of physical timers: 2/6/21/22 none on D7/D8
//    F0 L0 PWM outputs are mainly available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM22_CH1),  D4 (TIM22_CH2)
//
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_21_CC2
#define  ADC_TRIGGER_TIMER   TIMER_21

#define  LED_PWM_MODULE_1    TIMER_2             /* drives BLUE LED on D3/D6 */
#define  LED_1_A_CHANNEL      TIMER_CHANNEL_2    /* drives BLUE LED on D3    */
#define  LED_1_B_CHANNEL      TIMER_CHANNEL_3    /* drives BLUE LED on D6    */
#define  LED_PWM_MODULE_2    TIMER_22            /* drives RED  LED on D5/D4 */
#define  LED_2_A_CHANNEL      TIMER_CHANNEL_1    /* drives RED LED on D5     */
#define  LED_2_B_CHANNEL      TIMER_CHANNEL_2    /* drives RED LED on D4     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED      D3                  /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                  /*  PB_10  D6  TIM2_CH3 */
#define  SEEED_RED_LED       D5                  /*  PB_4   D5  TIM3_CH1 */
#define  SEEED_RED_LED2      D4                  /*  PB_5   D4  TIM3_CH2 */
#define  MAG_READ_SW1        D2                  /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                  /*  PA_8   D7   */
#endif


#if  defined(USE_STM32L1XX_NUCLEO)
             //-------------------------------
             //         STM32 L1 52
             //-------------------------------
// Note that the L1 is constrained in terms of physical timers on Arduino Dxx pins: none on D7/D8
//    L1 52 PWM outputs are mainly available:  D3 (TIM2_CH2),   D6 (TIM2_CH3)
//                                             D5 (TIM3_CH1),   D4 (TIM3_CH2)
#define  ADC_TRIGGER_TYPE    ADC_TRIGGER_TIMER_9_CC2
#define  ADC_TRIGGER_TIMER   TIMER_9

#define  DAC_1_TRIGGER_TYPE  DAC_TRIGGER_TIMER_6     /* DAC1 = A2   (PA4) */
#define  DAC_1_TRIGGER_TIMER TIMER_6
#define  DAC_2_TRIGGER_TYPE  DAC_TRIGGER_TIMER_7     /* DAC2 = D13  (PA5) */
#define  DAC_2_TRIGGER_TIMER TIMER_7

#define  TIMER_XR            TIMER_14        // XR = Interrupt on Rollover

#define  TIMER_OC            TIMER_3         // OC = Output compare test timer
#define   OC_CHANNEL_A        TIMER_CHANNEL_1      /* drives D5  pin for LED     */
#define   OC_CHANNEL_B        TIMER_CHANNEL_2_ALT1 /* shunt to D11 pin for OScope*/

#define  PWM_A_MODULE        TIMER_2            /* drives BLUE LEDs on D3/D6 */
#define   PWM_A_TEST_CHAN_A   PWM_CHANNEL_2     /* drives BLUE LED on D3     */
#define   PWM_A_TEST_CHAN_B   PWM_CHANNEL_3     /* drives BLUE LED on D6     */

                           // Example uses Grove Shield: D2 / D3 / D7 / D8
#define  SEEED_BLUE_LED1     D3                 /*  PB_3   D3  TIM2_CH2 */
#define  SEEED_BLUE_LED2     D6                 /*  PB_10  D6  TIM2_CH3 */

#define  MANUAL_RED_LED1     D4                 /*  PB_5   D4  -MANUAL- */
#define  MANUAL_RED_LED2     D8                 /*  PA_9   D8  -MANUAL- */

#define  MAG_READ_SW1        D2                 /*  PA_10  D2   */
#define  FIXED_SLIDE_SW1     D7                 /*  PA_8   D7   */
#endif



#if defined(__MSP432P401R__)                                   // PIN Syntax ONLY works for fixed function pins ala GPIO !
             //-------------------------------
             //         MSP432  ARM
             //-------------------------------
#define  ADC_TRIGGER_TYPE    TRIGGER_TIMER_1
#define  ADC_TRIGGER_TIMER   TIMER TIMER_1
                                          // Grove  MCU    LP    Energia    // PIN SYNTAX ACROSS THE BOARD WOULD
#define  MAG_READ_SW1           2,BIT4       /* J14  P 2.4  J4-3   Pin38 */  // GREAT FOR MSP430, but MESS FOR TIVA/STM32
                                                                          // because same Tiva/STM32 pin can have ALIAS TIM1_CH1/TIM3_CH2
#define  FIXED_SLIDE_SW1        6,BIT6       /* J16  P 6.6  J4-5   Pin36 */

#define  LIGHT_SENSOR_ADC       Adc14        /*  J5  P 6.1  J3-3   Pin23 */  // Port 6 not enabled ?
#define  POTENTIOMETER_ADC      Adc13        /*  J6  P 4.0  J3-4   Pin24 */

  // there is not a lot of consistency across TI platforms for PWM channels.
  // each one has to be customized.  J4 connector is easiest
#define  PWM_Demo_1_MODULE      PWM_MODULE_0
#define  PWM_Demo_2_MODULE      PWM_MODULE_2
#define  PWM_Demo_1A_CHANNEL    PWM_CHANNEL_3  // CCR3  P 2.6 J4-2 Pin39  J13
#define  PWM_Demo_1B_CHANNEL    PWM_CHANNEL_4  // CCR4  P 2.7 J4-1 Pin40
#define  PWM_Demo_2A_CHANNEL    PWM_CHANNEL_1  // CCR1  P 5.6 J4-4 Pin37  J15
#define  PWM_Demo_2B_CHANNEL    PWM_CHANNEL_4  // CCR4  P 6.7 J4-6 Pin35  J17

#define  STEP_MOTOR_Ax_MODULE   PWM_MODULE_x   // Port 3
#define  STEP_MOTOR_AIN1_PIN    PWM_MODULE_x   // P3.5 <-- FUed ?
#define  STEP_MOTOR_AIN2_PIN    Pin11          // P3.6    in 3,BIT5 syntax

#define  STEP_MOTOR_Bx_MODULE   PWM_MODULE_x   // Port 1
#define  STEP_MOTOR_BIN1_PIN    PWM_MODULE_x   // P1.5    in 1,BIT5 syntax
#define  STEP_MOTOR_BIN2_PIN    PWM_MODULE_x   // P1.4 <-- FUed ?

#define  STEP_MOTOR_RESET_PIN    // P3.4  <-- FUed ?
#define  STEP_MOTOR_nSTALL_PIN   // P1.2
#define  STEP_MOTOR_nFAULT_PIN   // P3.0
#define  STEP_MOTOR_nSLEEP_PIN   // P2.4
#define  STEP_MOTOR_POT_PIN      // P4.2 Analog
#endif


// use the default_project_config_parms.h (in the ~/boards directory) as
// the template for what parameters are supported.

// If you want to override all of the parms, then it is simplest just to
// copy the entire contents of the default_project_config_parms.h file
// into here, and then modify them as necessary.
// Then comment out the #include "default_project_config_parms.h" statement below

// Otherwise. if you want to use the rest of the (non-overriden) parms located
// in the default parms config file, then enable the include for it below.
#include "default_project_config_parms.h"

#endif                          //  __PROJ_CONF_PARMS_H__

//*****************************************************************************
