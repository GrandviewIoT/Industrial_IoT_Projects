
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                 device_config_common.h
//
//
// The majority of Nucleo boards have the same MCU pins mapped to the same
// Arduino and CN Headers. So they share common definitions for a range of
// X-Nucleo boards and Arduino shields.
//
// This file collects all the common Nucleo and X-Nucleo pin definitions
// into 1 file.
//
//
// In addition to a suite of most of the Nucleo boards, the corresponding
// definitions for the following X-Nucleo boards are included in this file:
//         - L6474 Stepper         XNUCLEO-IHM01A1
//         - powerSTEP01 Stepper   XNUCLEO-IHM03A1
//         - Sub GHz Spirit1       XNUCLEO-IDS01A5
//         - BLE BLUE-NRG          XNUCLEO-IDB04A1
//         - BLE SPBTLE-RF         XNUCLEO-IDB05A1
//         - MEMS NMotion/Env      XNUCLEO-IDB05A1
//
// And the following Arduino compatible Shields for IoT connectivity:
//         - GSM / GPS FONA         Adafruit xxx-xxx
//         - WiFi V 2.0             Arduino xxx-xxx
//         - W5200 V 2.0 Ethernet   Arduino zzz-zzz  (Radio Shack ccc-ccc)  Seeed Studio xxx
//
// History:
//   08/11/15 - Consolidated separate files into one spot for easier porting.Duq
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


#ifndef  __DEVICE_CONFIG_COMMON_STM32_H__
#define  __DEVICE_CONFIG_COMMON_STM32_H__

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                 Pin Configuration Definitions for STM32 MCUs
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  PA0       0          /* 0x0x = PORT A */   /* logical pin id 0 - 255 */
#define  PA1       1
#define  PA2       2
#define  PA3       3
#define  PA4       4
#define  PA5       5
#define  PA6       6
#define  PA7       7
#define  PA8       8
#define  PA9       9
#define  PA10      10
#define  PA11      11
#define  PA12      12
#define  PA13      13
#define  PA14      14
#define  PA15      15
#define  PB0       16+0       /* 0x1x = PORT B */
#define  PB1       16+1
#define  PB2       16+2
#define  PB3       16+3
#define  PB4       16+4
#define  PB5       16+5
#define  PB6       16+6
#define  PB7       16+7
#define  PB8       16+8
#define  PB9       16+9
#define  PB10      16+10
#define  PB11      16+11
#define  PB12      16+12
#define  PB13      16+13
#define  PB14      16+14
#define  PB15      16+15
#define  PC0       32+0       /* 0x2x = PORT C */
#define  PC1       32+1
#define  PC2       32+2
#define  PC3       32+3
#define  PC4       32+4
#define  PC5       32+5
#define  PC6       32+6
#define  PC7       32+7
#define  PC8       32+8
#define  PC9       32+9
#define  PC10      32+10
#define  PC11      32+11
#define  PC12      32+12
#define  PC13      32+13
#define  PC14      32+14
#define  PC15      32+15
#define  PD0       48+0       /* 0x3x = PORT D */
#define  PD1       48+1
#define  PD2       48+2
#define  PD3       48+3
#define  PD4       48+4
#define  PD5       48+5
#define  PD6       48+6
#define  PD7       48+7
#define  PD8       48+8
#define  PD9       48+9
#define  PD10      48+10
#define  PD11      48+11
#define  PD12      48+12
#define  PD13      48+13
#define  PD14      48+14
#define  PD15      48+15
#define  PE0       64+0       /* 0x4x = PORT E */
#define  PE1       64+1
#define  PE2       64+2
#define  PE3       64+3
#define  PE4       64+4
#define  PE5       64+5
#define  PE6       64+6
#define  PE7       64+7
#define  PE8       64+8
#define  PE9       64+9
#define  PE10      64+10
#define  PE11      64+11
#define  PE12      64+12
#define  PE13      64+13
#define  PE14      64+14
#define  PE15      64+15
#define  PF0       80+0       /* 0x5x = PORT F */
#define  PF1       80+1
#define  PF2       80+2
#define  PF3       80+3
#define  PF4       80+4
#define  PF5       80+5
#define  PF6       80+6
#define  PF7       80+7
#define  PF8       80+8
#define  PF9       80+9
#define  PF10      80+10
#define  PF11      80+11
#define  PF12      80+12
#define  PF13      80+13
#define  PF14      80+14
#define  PF15      80+15
#define  PG0       96+0       /* 0x6x = PORT G */
#define  PG1       96+1
#define  PG2       96+2
#define  PG3       96+3
#define  PG4       96+4
#define  PG5       96+5
#define  PG6       96+6
#define  PG7       96+7
#define  PG8       96+8
#define  PG9       96+9
#define  PG10      96+10
#define  PG11      96+11
#define  PG12      96+12
#define  PG13      96+13
#define  PG14      96+14
#define  PG15      96+15
#define  PH0       112+0      /* 0x7x = PORT H */
#define  PH1       112+1
#define  PH2       112+2
#define  PH3       112+3
#define  PH4       112+4
#define  PH5       112+5
#define  PH6       112+6
#define  PH7       112+7
#define  PH8       112+8
#define  PH9       112+9
#define  PH10      112+10
#define  PH11      112+11
#define  PH12      112+12
#define  PH13      112+13
#define  PH14      112+14
#define  PH15      112+15
#define  PI0       128+0      /* 0x8x = PORT I */
#define  PI1       128+1
#define  PI2       128+2
#define  PI3       128+3
#define  PI4       128+4
#define  PI5       128+5
#define  PI6       128+7
#define  PI7       128+7
#define  PI8       128+8
#define  PI9       128+9
#define  PI10      128+10
#define  PI11      128+11
#define  PI12      128+12
#define  PI13      128+13
#define  PI14      128+14
#define  PI15      128+15
#define  PJ0       144+0      /* 0x9x = PORT J */
#define  PJ1       144+1
#define  PJ2       144+2
#define  PJ3       144+3
#define  PJ4       144+4
#define  PJ5       144+5
#define  PJ6       144+7
#define  PJ7       144+7
#define  PJ8       144+8
#define  PJ9       144+9
#define  PJ10      144+10
#define  PJ11      144+11
#define  PJ12      144+12
#define  PJ13      144+13
#define  PJ14      144+14
#define  PJ15      144+15
#define  PK0       160+0      /* 0xAx = PORT K */
#define  PK1       160+1
#define  PK2       160+2
#define  PK3       160+3
#define  PK4       160+4
#define  PK5       160+5
#define  PK6       160+7
#define  PK7       160+7
#define  PK8       160+8
#define  PK9       160+9
#define  PK10      160+10
#define  PK11      160+11
#define  PK12      160+12
#define  PK13      160+13
#define  PK14      160+14
#define  PK15      160+15



#define  PA0_PAP   0,GPIO_PIN_0       /* PAP = pin and port */
#define  PA1_PAP   0,GPIO_PIN_1
#define  PA2_PAP   0,GPIO_PIN_2
#define  PA3_PAP   0,GPIO_PIN_3
#define  PA4_PAP   0,GPIO_PIN_4
#define  PA5_PAP   0,GPIO_PIN_5
#define  PA6_PAP   0,GPIO_PIN_6
#define  PA7_PAP   0,GPIO_PIN_7
#define  PA8_PAP   0,GPIO_PIN_8
#define  PA9_PAP   0,GPIO_PIN_9
#define  PA10_PAP  0,GPIO_PIN_10
#define  PA11_PAP  0,GPIO_PIN_11
#define  PA12_PAP  0,GPIO_PIN_12
#define  PA13_PAP  0,GPIO_PIN_13
#define  PA14_PAP  0,GPIO_PIN_14
#define  PA15_PAP  0,GPIO_PIN_15
#define  PB0_PAP   1,GPIO_PIN_0
#define  PB1_PAP   1,GPIO_PIN_1
#define  PB2_PAP   1,GPIO_PIN_2
#define  PB3_PAP   1,GPIO_PIN_3
#define  PB4_PAP   1,GPIO_PIN_4
#define  PB5_PAP   1,GPIO_PIN_5
#define  PB6_PAP   1,GPIO_PIN_6
#define  PB7_PAP   1,GPIO_PIN_7
#define  PB8_PAP   1,GPIO_PIN_8
#define  PB9_PAP   1,GPIO_PIN_9
#define  PB10_PAP  1,GPIO_PIN_10
#define  PB11_PAP  1,GPIO_PIN_11
#define  PB12_PAP  1,GPIO_PIN_12
#define  PB13_PAP  1,GPIO_PIN_13
#define  PB14_PAP  1,GPIO_PIN_14
#define  PB15_PAP  1,GPIO_PIN_15
#define  PC0_PAP   2,GPIO_PIN_0
#define  PC1_PAP   2,GPIO_PIN_1
#define  PC2_PAP   2,GPIO_PIN_2
#define  PC3_PAP   2,GPIO_PIN_3
#define  PC4_PAP   2,GPIO_PIN_4
#define  PC5_PAP   2,GPIO_PIN_5
#define  PC6_PAP   2,GPIO_PIN_6
#define  PC7_PAP   2,GPIO_PIN_7
#define  PC8_PAP   2,GPIO_PIN_8
#define  PC9_PAP   2,GPIO_PIN_9
#define  PC10_PAP  2,GPIO_PIN_10
#define  PC11_PAP  2,GPIO_PIN_11
#define  PC12_PAP  2,GPIO_PIN_12
#define  PC13_PAP  2,GPIO_PIN_13
#define  PC14_PAP  2,GPIO_PIN_14
#define  PC15_PAP  2,GPIO_PIN_15
#define  PD0_PAP   3,GPIO_PIN_0
#define  PD1_PAP   3,GPIO_PIN_1
#define  PD2_PAP   3,GPIO_PIN_2
#define  PD3_PAP   3,GPIO_PIN_3
#define  PD4_PAP   3,GPIO_PIN_4
#define  PD5_PAP   3,GPIO_PIN_5
#define  PD6_PAP   3,GPIO_PIN_6
#define  PD7_PAP   3,GPIO_PIN_7
#define  PD8_PAP   3,GPIO_PIN_8
#define  PD9_PAP   3,GPIO_PIN_9
#define  PD10_PAP  3,GPIO_PIN_10
#define  PD11_PAP  3,GPIO_PIN_11
#define  PD12_PAP  3,GPIO_PIN_12
#define  PD13_PAP  3,GPIO_PIN_13
#define  PD14_PAP  3,GPIO_PIN_14
#define  PD15_PAP  3,GPIO_PIN_15
#define  PE0_PAP   4,GPIO_PIN_0
#define  PE1_PAP   4,GPIO_PIN_1
#define  PE2_PAP   4,GPIO_PIN_2
#define  PE3_PAP   4,GPIO_PIN_3
#define  PE4_PAP   4,GPIO_PIN_4
#define  PE5_PAP   4,GPIO_PIN_5
#define  PE6_PAP   4,GPIO_PIN_6
#define  PE7_PAP   4,GPIO_PIN_7
#define  PE8_PAP   4,GPIO_PIN_8
#define  PE9_PAP   4,GPIO_PIN_9
#define  PE10_PAP  4,GPIO_PIN_10
#define  PE11_PAP  4,GPIO_PIN_11
#define  PE12_PAP  4,GPIO_PIN_12
#define  PE13_PAP  4,GPIO_PIN_13
#define  PE14_PAP  4,GPIO_PIN_14
#define  PE15_PAP  4,GPIO_PIN_15
#define  PF0_PAP   5,GPIO_PIN_0
#define  PF1_PAP   5,GPIO_PIN_1
#define  PF2_PAP   5,GPIO_PIN_2
#define  PF3_PAP   5,GPIO_PIN_3
#define  PF4_PAP   5,GPIO_PIN_4
#define  PF5_PAP   5,GPIO_PIN_5
#define  PF6_PAP   5,GPIO_PIN_6
#define  PF7_PAP   5,GPIO_PIN_7
#define  PF8_PAP   5,GPIO_PIN_8
#define  PF9_PAP   5,GPIO_PIN_9
#define  PF10_PAP  5,GPIO_PIN_10
#define  PF11_PAP  5,GPIO_PIN_11
#define  PF12_PAP  5,GPIO_PIN_12
#define  PF13_PAP  5,GPIO_PIN_13
#define  PF14_PAP  5,GPIO_PIN_14
#define  PF15_PAP  5,GPIO_PIN_15
#define  PG0_PAP   6,GPIO_PIN_0
#define  PG1_PAP   6,GPIO_PIN_1
#define  PG2_PAP   6,GPIO_PIN_2
#define  PG3_PAP   6,GPIO_PIN_3
#define  PG4_PAP   6,GPIO_PIN_4
#define  PG5_PAP   6,GPIO_PIN_5
#define  PG6_PAP   6,GPIO_PIN_6
#define  PG7_PAP   6,GPIO_PIN_7
#define  PG8_PAP   6,GPIO_PIN_8
#define  PG9_PAP   6,GPIO_PIN_9
#define  PG10_PAP  6,GPIO_PIN_10
#define  PG11_PAP  6,GPIO_PIN_11
#define  PG12_PAP  6,GPIO_PIN_12
#define  PG13_PAP  6,GPIO_PIN_13
#define  PG14_PAP  6,GPIO_PIN_14
#define  PG15_PAP  6,GPIO_PIN_15
#define  PH0_PAP   7,GPIO_PIN_0
#define  PH1_PAP   7,GPIO_PIN_1
#define  PH2_PAP   7,GPIO_PIN_2
#define  PH3_PAP   7,GPIO_PIN_3
#define  PH4_PAP   7,GPIO_PIN_4
#define  PH5_PAP   7,GPIO_PIN_5
#define  PH6_PAP   7,GPIO_PIN_6
#define  PH7_PAP   7,GPIO_PIN_7
#define  PH8_PAP   7,GPIO_PIN_8
#define  PH9_PAP   7,GPIO_PIN_9
#define  PH10_PAP  7,GPIO_PIN_10
#define  PH11_PAP  7,GPIO_PIN_11
#define  PH12_PAP  7,GPIO_PIN_12
#define  PH13_PAP  7,GPIO_PIN_13
#define  PH14_PAP  7,GPIO_PIN_14
#define  PH15_PAP  7,GPIO_PIN_15
#define  PI0_PAP   8,GPIO_PIN_0
#define  PI1_PAP   8,GPIO_PIN_1
#define  PI2_PAP   8,GPIO_PIN_2
#define  PI3_PAP   8,GPIO_PIN_3
#define  PI4_PAP   8,GPIO_PIN_4
#define  PI5_PAP   8,GPIO_PIN_5
#define  PI6_PAP   8,GPIO_PIN_7
#define  PI7_PAP   8,GPIO_PIN_7
#define  PI8_PAP   8,GPIO_PIN_8
#define  PI9_PAP   8,GPIO_PIN_9



#if defined(USE_STM32746G_DISCO) || defined(USE_STM32F401_DISCO)  \
   || defined(USE_STM32F3348_DISCO) || defined(USE_STM32F3_DISCO) \
   || defined(USE_STM32F072B_DISCO) || defined(USE_STM32L0538_DISCO) \
   || defined(USE_STM32L152C_DISCO) || defined(USE_STM32L476G_DISCO_REVB)
                  //------------------------------------------------------------
                  //                       DISCOVERY
                  //
                  // If any Discovery Boards are referenced, pull in their defs
                  //------------------------------------------------------------
#include  "device_config_discovery.h"
#endif




//   || defined(STM32F030x8) || defined(STM32F070xB) || defined(STM32F411xE) || defined(STM32F446xx)
//   || defined(STM32F303xE) || defined(STM32F334x8)
//   || defined(STM32F103xB)

#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F091xC)     \
     || defined(USE_STM32F3XX_NUCLEO) || defined(USE_STM32F4XX_NUCLEO) \
     || defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32L1XX_NUCLEO) \
     || defined(USE_STM32L4XX_NUCLEO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      Common  Features  on  most  NUCLEOs
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

                    //----------------------------------------------
                    //                Digital Inputs
                    //----------------------------------------------
#define  D15      PB8               /* default I2C SCL    */
#define  D14      PB9                /* default I2C SDA    */
#define  D13      PA5                /* default SPI SCLK   */
#define  D12      PA6                /* default SPI MISO   */
#define  D11      PA7                /* default SPI MOSI   */
#define  D10      PB6                /* default SPI CS pin */
#define  D9       PC7
#define  D8       PA9
#define  D7       PA8
#define  D6       PB10
#define  D5       PB4
#define  D4       PB5
#define  D3       PB3
#define  D2       PA10
#define  D1       PA2             /* TX VCP - do not use unless re-solder */
#define  D0       PA3             /* RX VCP - do not use unless re-solder */

#define  LED1     D13         // on-board LED is connected to D13 pin (PA5 or PB13)

#define  BOARD_USER_BUTTON       PC13   // on-board Pushbutton always connected to PC13
#define  BOARD_USER_BUTTON_PIN   GPIO_PIN_13

#if defined(USE_STM32F4XX_NUCLEO)
#define  BOARD_USER_BUTTON_IRQn  EXTI15_10_IRQn          // associated EXTI interrupt
#endif
#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F091xC)
#define  BOARD_USER_BUTTON_IRQn  EXTI4_15_IRQn           // associated EXTI interrupt
#endif

                     //----------------------------------------------
                     //                Analog Inputs
                     //----------------------------------------------
             //-----------------------------------------------------------------
             // Common ADC assignments across F0, F1, F3, F4, L0, and L1 Nucleos
             //-----------------------------------------------------------------
#define  A0_PIN      PA0
#define  A1_PIN      PA1
#define  A2_PIN      PA4
#define  A3_PIN      PB0
#define  A4_PIN      PC1
#define  A5_PIN      PC0

                     //------------------------------------------------
                     //             Virtual Comm Port  to  PC    (VCP)
                     //------------------------------------------------
#define  VCP_TX_PIN  D1
#define  VCP_RX_PIN  D0


                     //---------------------------------------------------------
                     // Assignment of PA/PB/PC pins to ADC channels has some
                     // variation between MCUs. esp F3
                     //---------------------------------------------------------
#if defined(STM32F334x8)
#define  A0_CHANNEL  ADC_CHAN_1     // PA0 => ADC1  -  physical channel number on F3
#define  A1_CHANNEL  ADC_CHAN_2     // PA1     "
#define  A2_CHANNEL  ADC_CHAN_1     // PA4 => ADC2     ??? !!! FIX NEEDED  WVD 09/03/15
#define  A3_CHANNEL  ADC_CHAN_8     // PB0 => ADC1
#define  A4_CHANNEL  ADC_CHAN_7     // PC1     "
#define  A5_CHANNEL  ADC_CHAN_6     // PC0     "
#elif defined(STM32F303xE) || defined(STM32F303xC)
#define  A0_CHANNEL  ADC_CHAN_1     // PA0 => ADC1  -  physical channel number on F3
#define  A1_CHANNEL  ADC_CHAN_2     // PA1     "
#define  A2_CHANNEL  ADC_CHAN_1     // PA4 => ADC2     ??? !!! FIX NEEDED  WVD 09/03/15
#define  A3_CHANNEL  ADC_CHAN_12    // PB0 => ADC3     ??? !!! FIX NEEDED
#define  A4_CHANNEL  ADC_CHAN_7     // PC1     "
#define  A5_CHANNEL  ADC_CHAN_6     // PC0     "
#else
#define  A0_CHANNEL  ADC_CHAN_0     // PA0  -  physical channel number on F0, F1, ...
#define  A1_CHANNEL  ADC_CHAN_1     // PA1
#define  A2_CHANNEL  ADC_CHAN_4     // PA4
#define  A3_CHANNEL  ADC_CHAN_8     // PB0
#define  A4_CHANNEL  ADC_CHAN_11    // PC1
#define  A5_CHANNEL  ADC_CHAN_10    // PC0
#endif

              //----------------------------------------------
              //         Grove Base Shield Definitions
              //----------------------------------------------
#define  GROVE_A0   A0_CHANNEL
#define  GROVE_A1   A1_CHANNEL
#define  GROVE_A2   A2_CHANNEL
#define  GROVE_A3   A3_CHANNEL

#define  GROVE_Jx   D5

#endif                           // generic Nucleos  (sans L4)



//******************************************************************************
//******************************************************************************
//
//               Definitions for Key Xnucleo Boads and Shields
//
//******************************************************************************
//******************************************************************************

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                           L6474  Stepper  Board                     (NUCLEOs)
//
//                              XNUCLEO-IHM01A1
//
//  SPI:    uses D13/D12/D11 (PA5/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  PWM1:   uses D9  (PC7) for PWM output to drive the stepper.(Step clock input)
//  RESET:  uses D8  (PA9) for Chip Reset/Standby
//  DIR:    uses D7  (PA8) for Direction control of Stepper
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_L6474)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#define  L6474_SPI_MODULE    SPI_ID_1_A           /*  PA5 / PA6 / PA7         */
#define  L6474_SPI_MODE      SPI_MODE_3           /* 2x and 2y                */
// SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;       // SPI Mode 2 ?
// SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
#define  L6474_SPI_BAUD      SPI_BAUDRATEPRESCALER_8  /*STM32 uses pre-scalars*/
#define  L6474_CS_PIN        D10                  /*  PB6   Chip Select       */
#define  L6474_RESET_PIN     D8                   /*  PA9   Chip Reset        */
#define  L6474_IRQ_PIN       D2                   /*  PA10  Interrupt Request */
#define  L6474_DIR_1_PIN     D7                   /*  PA8   Stepper Direction */
#define  L6474_PWM_1_PIN     D9                   /*  PC7   Stepper PWM       */
                                                  /*        Uses Timer3 Ch2   */
 // the following is MCU dependent
#define  L6474_PWM_1_MODULE  TIMER_3
#define  L6474_PWM_1_CHANNEL TIMER_CHANNEL_2_ALT2  /* ouputs on pin Ardu D9   */
#define  L6474_EXTI_IRQ_NUM  EXTI15_10_IRQn




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  powerSTEP01  High Power  Stepper  Board            (NUCLEOs)
//
//                              XNUCLEO-IHM03A1
//
//  SPI:    uses D13/D12/D11 (PA5/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  PWM1:   uses D9  (PC7) for PWM output to drive the stepper.(Step clock input)
//  RESET:  uses D8  (PA9) for Chip Reset/Standby
//  BUSY:   uses D4  (PB5)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_POWERSTEP01)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#define  POWERSTEP_SPI_ID    SPI_ID_1_A                        /* PA5/PA6/PA7 */
#define  POWERSTEP_SPI_CS    D10
#define  POWERSTEP_PWM       D9
#define  POWERSTEP_RESET     D8
#define  POWERSTEP_DIR       D7   ???
#define  POWERSTEP_BUSY      D4
#define  POWERSTEP_FLAG      D2   ???    /* PA10 */




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                         Sub GHz  Spirit1  Board     915 MHz         (NUCLEOs)
//
//                              XNUCLEO-IDS01A5
//
//  SPI:    uses D3/D12/D11 (PB3/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  ENABLE: uses D9  (PC7) for Chip Enable
//  LED:    uses D5  (PB4) for on-board LED
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(SPITIR1_ST_SHIELD)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#define  SUB_GHZ_SPI_ID          SPI_ID_1_B    /* PB3 / PA6 / PA7   */
#define  SUB_GHZ_SPI_CS          D10           /* PB6  Chip Select  */
#define  SUB_GHZ_GPIO3_ENABLE    D7
#define  SubGhz_SDN_ENABLE       D2            /* PA10  Chip Enable */
#define  SUB_GHZ_ON_BD_LED       D5            /* PB4  GPIO pin for LED on Spirit1 Board */
#define  SubGhz_IRQ_GPIO3        D9            /* PC7  GPIO_3 EXTI Interrupt IRQ */
#define  SUB_GHZ_GPIO_3_EXTI_PIN GPIO_PIN_7





//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                           BLE  BLUE-NRG  Board                      (NUCLEOs)
//
//                              XNUCLEO-IDB04A1
//
//  SPI:    uses D3/D12/D11 (PB3/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses A1  (PA1) for Chip Select
//  RESET:  uses D7  (PA8) for Chip Reset
//  IRQ:    uses A0  (PA0) for Interrupt Request
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_BLUENRG_BLE)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#if (MCU_CLOCK_SPEED > 84000000)
#define  BLE_SPI_BAUDRATE_25_MHZ  SPI_BAUDRATEPRESCALER_16  // yileds approx 20 MHz for fast CPUs with Max Clock 100 - 200 MHz (F7, F4_46)
#else
#define  BLE_SPI_BAUDRATE_25_MHZ  SPI_BAUDRATEPRESCALER_4   // yileds approx 25 MHz for CPUs with Max Clock <= 84 MHz
#endif
#define  BLE_SPI_TIMEOUT_DURATION 20                  /* max of 20 milli-secs */
#define  BLE_BLUENRG_SPI_MODE     SPI_MODE_0
#define  BLE_BLUENRG_SPI_ID       SPI_ID_1_B                /* PB3/PA6/PA7 */
#define  BLE_BLUENRG_SPI_CS       A1_PIN                    /* PA1 */
#define  BLE_BLUENRG_CS_GPIO      GPIO_PIN_1
#define  BLE_BLUENRG_RESET        D7                        /* PA8 */
#define  BLE_BLUENRG_RESET_GPIO   GPIO_PIN_8
#define  BLE_BLUENRG_IRQ_PIN      A0_PIN                    /* PA0 */
#define  BLE_BLUENRG_EXTI_GPIO    GPIO_PIN_0

#if defined(STM32F103xB) \
 || defined(USE_STM32F3XX_NUCLEO) \
 || defined(USE_STM32F4XX_NUCLEO) \
 || defined(USE_STM32L1XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO) \
 || defined(USE_STM32746G_DISCO)
#define  BLE_BLUENRG_IRQn         EXTI0_IRQn       /* EXT 0 rupt for PA0 */
#endif

#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F091xC) \
 || defined(USE_STM32L0XX_NUCLEO)
#define  BLE_BLUENRG_IRQn         EXTI0_1_IRQn     /* EXT 0 rupt  F0_72 DIFF  */
#endif

#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F091xC) || defined(STM32F103xB) \
 || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L0XX_NUCLEO) \
 || defined(USE_STM32L1XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO) \
 || defined(USE_STM32746G_DISCO)
#define  ASSERT_CS_BlueNRG()      GPIOA->BSRR = ((uint32_t) BLE_BLUENRG_CS_GPIO << 16)
#define  DEASSERT_CS_BlueNRG()    GPIOA->BSRR = BLE_BLUENRG_CS_GPIO
#define  ASSERT_BlueNRG_RESET()   GPIOA->BSRR = ((uint32_t) BLE_BLUENRG_RESET_GPIO << 16)
#define  DEASSERT_BlueNRG_RESET() GPIOA->BSRR = BLE_BLUENRG_RESET_GPIO
#endif

#if defined(USE_STM32F3XX_NUCLEO)
#define  ASSERT_CS_BlueNRG()      GPIOA->BSRRH = BLE_BLUENRG_CS_GPIO
#define  DEASSERT_CS_BlueNRG()    GPIOA->BSRRL = BLE_BLUENRG_CS_GPIO
#define  ASSERT_BlueNRG_RESET()   GPIOA->BSRRH = BLE_BLUENRG_RESET_GPIO
#define  DEASSERT_BlueNRG_RESET() GPIOA->BSRRL = BLE_BLUENRG_RESET_GPIO
#endif




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                            BLE  SPBTLE-RF  Board                    (NUCLEOs)
//
//                              XNUCLEO-IDB05A1
//
//  SPI:    uses D3/D12/D11 (PB3/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses A1  (PA1) for Chip Select
//  RESET:  uses D7  (PA8) for Chip Reset
//  IRQ:    uses A0  (PA0) for Interrupt Request
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_SPBTLE)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#define  BLE_SPBTLE_SPI_ID   SPI_ID_1_B                        /* PB3/PA6/PA7 */
#define  BLE_SPBTLE_SPI_CS   A1_PIN
#define  BLE_SPBTLE_RESET    D7
#define  BLE_SPBTLE_IRQ      A0_PIN




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      MEMS  Motion/Environment  Sensor  Board        (NUCLEOs)
//
//                              XNUCLEO-IDB05A1
//
//  I2C:    uses D15/D14 (PB8/PB9) for I2C SCL/SDA
//  IRQ:    uses A0  (PA0) for Interrupt Request
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_MEMS_ENV)
  #ifndef USES_I2C     // ensure USES_I2C is defined to pickup SPI interrupt ISRs
  #define  USES_I2C
  #endif
#endif
//#define  MEMS_ENV_I2C_ID          I2C_ID_1_A             /* PB8/PB9 on I2C1 */
#define  MEMS_ENV_I2C_ID            I2C_M1                 /* I2C1 module */
#define  MEMS_ENV_SCL_PIN           PB8
#define  MEMS_ENV_SDA_PIN           PB9
#define  MEMS_MY_OWN_ADDRESS        0x33

#define  LSM6DS0_ACCEL_GYRO_I2C_ADDR xxx     /* Acceleometer and Gyroscope */
#define  LIS3MDL_MAG_I2C_ADDR        yyy     /* Magnetometer */
#define  LPS25HB_PRESSURE_I2C_ADDR   zzz     /* Pressure sensor */
#define  HTS221_HUMIDITY_I2C_ADDR    www     /* Humdity and temperature sensor*/

#if defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) \
 || defined(USE_STM32L1XX_NUCLEO)
             // F4 and L1 use Baud based Clock Speed
#define  MEMS_ENV_BAUD_TIMING   400000

#elif defined(USE_STM32F0XX_NUCLEO) || defined(STM32F072xB) || defined(STM32F091xC) \
 || defined(USE_STM32F3XX_NUCLEO) || defined(USE_STM32746G_DISCO) \
 || defined(STM32L053xx) || defined(STM32L476xx)
             // F0, F3, F7, L0, and L4 use encoded Timing construct
#define  MEMS_ENV_BAUD_TIMING   0x0070D8FF
#else
#error "No I2C Baud Clock or Timing parameter Specified"
#endif

#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F091xC) \
 || defined(USE_STM32L0XX_NUCLEO) \
 || defined(STM32F103xB)          \
 || defined(USE_STM32F3XX_NUCLEO) \
 || defined(USE_STM32L4XX_NUCLEO)
#define MEMS_INT1_EXTI_IRQn                EXTI4_15_IRQn
#define MEMS_INT2_EXTI_IRQn                EXTI0_1_IRQn
#define NUCLEO_I2C_SHIELDS_EV_IRQn         I2C1_IRQn
#endif

#if defined(USE_STM32F4XX_NUCLEO) \
 || defined(USE_STM32F3XX_NUCLEO) \
 || defined(USE_STM32L1XX_NUCLEO) \
 || defined(USE_STM32746G_DISCO)
#define MEMS_INT1_EXTI_IRQn                EXTI4_IRQn
#define MEMS_INT2_EXTI_IRQn                EXTI0_IRQn
#define NUCLEO_I2C_SHIELDS_EV_IRQn         I2C1_EV_IRQn
#endif
#define NUCLEO_I2C_SHIELDS_ER_IRQn         I2C1_ER_IRQn




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                           GSM / GPS   FONA   Board                  (NUCLEOs)
//
//                              Adafruit  FONA-808
//
//  UART:   uses UART1 pin D8 (STM32 TX) and D2 (STM32 RX).
//
//          FONA-808 is wired to TX on D3, and RX on D2, so we have to do some
//          jumper stunting to re-route the UART pins to correspond.
//            - Unhook GSM D2 connector from STM32 Arduino connector
//            - Jumper D8 to D2
//                 FONA_RX  2  -  unhook, then route to Arduino TX pin D8
//                 FONA_TX  3  -  unhook, then route to Arduino RX pin D2
//
//  RESET:  uses D4 (PB5) for Chip Reset
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_GSM_FONA808)
  #ifndef USES_UART   // ensure USES_UART is defined to pickup UART interrupt ISRs
  #define  USES_UART
  #endif
#endif

//  Only common UART pins across MCUs are D8 (PA9/TX) and D2 (PA10/RX)
//              VCP pins output nothing on Arduino Header (weird/dumb)

#define  GSM_FONA_UART_MODULE    UART_M1          // TX=D8 / RX=D2  (PA9/PA10)
#define  GSM_FONA_UART_TX        D8
#define  GSM_FONA_UART_RX        D2
#define  GSM_FONA_RESET          D4
//#define  GSM_FONA_UART_BAUD_RATE 115200  // FONA overruns and hangs at 115Kbps
#define  GSM_FONA_UART_BAUD_RATE 4800




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                          WiFi ESP8266 Sparkfun Shield               (NUCLEOs)
//
//                             using ESP8266 WiFi Chip
//
//  UART:   default uses either D1/D0 (VCP) or D9/D8 (partial)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_ESP8266)
  #ifndef USES_UART   // ensure USES_UART is defined to pickup UART interrupt ISRs
  #define  USES_UART
  #endif
#endif

#define  ESP8266_UART_MODULE       UART_M1         // TX=D8 / RX=D2  (PA9/PA10)
#define  ESP8266_UART_TX           D8
#define  ESP8266_UART_RX           D2
#define  ESP8266_UART_BAUD_RATE  9600


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                         STX/ETX UART^  WiFi Controller              (NUCLEOs)
//
//                             using ESP8266 WiFi Chip
//
//  SPI:    uses D13/D12/D11 (PA5/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  IRQ:    uses D7  (PB3) for Interrupt Request   (Schematic says D3 but code says D7 !)
//  LED:    uses D9  (PC7) controls LED that is on-board the Arduino WiFi shield
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  WIFI_CTLR_UART_MODULE  UART_M1     // USART1
#define  WIFI_CTLR_TX_PIN       D8          // Master TX pin PA9      Slave RX
#define  WIFI_CTLR_RX_PIN       D2          // Master RX pin PA10     Slave TX
#define  WIFI_CTLR_BAUD         115200




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                       W5200  v 2.0  Wired  Ethernet Shield          (NUCLEOs)
//
//                      Radio Shack xxxx /  Seeed Studio xxx
//
//  SPI:    uses D3/D12/D11 (PB3/PA6/PA7) for SPI SCLK/MISO/MOSI (default stdalone)
//  SPI CS: uses D10 (PB6) for Chip Select
//  IRQ:    uses A0  (PA0) for Interrupt Request
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(USES_W5200)
  #ifndef USES_SPI     // ensure USES_SPI is defined to pickup SPI interrupt ISRs
  #define  USES_SPI
  #endif
#endif
#define  W5200_SPI_CS          D10
#define  DEASSERT_CS_W5200()   pin_Low(W5200_SPI_CS)          /* sets CS low  */
#define  ASSERT_CS_W5200()     pin_High(W5200_SPI_CS)         /* sets CS high */
#define  W5200_IRQ             A0_PIN
#define  W5200_SPI_MODE        SPI_MODE_0

#if defined(STANDARD_ARDUINO_PINS)
               // standalone, no other boards mounted, jumpers routed to std pins
//#define  W5200_SPI_ID        SPI_ID_1_B                     /* PB3/PA6/PA7 */
  #define  W5200_SPI_ID        SPI_M1
  #define  W5200_SCLK_PIN      PB3                       /* using PB3/PA6/PA7 */
  #define  W5200_MISO_PIN      PA6
  #define  W5200_MOSI_PIN      PA7
#else
               // other Xnucleo boards are mounted on same MCU, so
               // must use different SPI for W5200 e.g. CN pins PC10/PC11/PC12)
  #if defined(STM32F303xC) || defined(STM32F303xE)  \
   || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L1XX_NUCLEO) \
   || defined(USE_STM32L4XX_NUCLEO)
//#define  W5200_SPI_ID        SPI_ID_3_C
  #define  W5200_SPI_ID        SPI_M3
  #define  W5200_SCLK_PIN      PC10                   /* using PC10/PC11/PC12 */
  #define  W5200_MISO_PIN      PC11
  #define  W5200_MOSI_PIN      PC12

  #elif defined(STM32F334x8)
                 // F3_34 Nucleo has only 1 SPI module - SPI1 on PA5/PA6/PA7 (A) or PB3/PA6/PA7 (B)
//#define  W5200_SPI_ID        SPI_ID_1_A
  #define  W5200_SPI_ID        SPI_M1
  #define  W5200_SCLK_PIN      PA5                    /* using PA5/PA6/PA7 */
  #define  W5200_MISO_PIN      PA6
  #define  W5200_MOSI_PIN      PA7

  #elif defined(USE_STM32746G_DISCO)
                 // F7 Discovery Arduino uses different pinout
//#define  W5200_SPI_ID        SPI_ID_2_I
  #define  W5200_SPI_ID        SPI_M2

  #else
                 // F0, F1, L0, ... need to use different CN pin set e.g. CN pins PB13/PB14/PB15
//#define  W5200_SPI_ID        SPI_ID_2_B
  #define  W5200_SPI_ID        SPI_M2
  #define  W5200_SCLK_PIN      PB13                  /* using PPB13/PB14/PB15 */
  #define  W5200_MISO_PIN      PB14
  #define  W5200_MOSI_PIN      PB15
  #endif
#endif



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                           WiFi V 2.0  Arduino Shield                (NUCLEOs)
//
//                             using HDG204 WiFi Chip
//
//  SPI:    uses D13/D12/D11 (PA5/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  IRQ:    uses D7  (PB3) for Interrupt Request   (Schematic says D3 but code says D7 !)
//  LED:    uses D9  (PC7) controls LED that is on-board the Arduino WiFi shield
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  ARD_WIFI_SPI_CS        D10
#define  DEASSERT_CS_ARD_WIFI() pin_High(ARD_WIFI_SPI_CS)    /* sets CS low  */
#define  ASSERT_CS_ARD_WIFI()   pin_Low(ARD_WIFI_SPI_CS)     /* sets CS high */
#define  ARD_WIFI_IRQ           D7                           /* PA8 */
#define  ARD_WIFI_LED           D9
#define  ARD_WIFI_SPI_MODE      SPI_MODE_0

#if defined(STANDARD_ARDUINO_PINS)
               // standalone, no other boards mounted, jumpers routed to std pins
  #define  ARD_WIFI_SPI_ID        SPI_ID_1_A                  /* PA5/PA6/PA7 */
#else
               // other X boards are mounted, so must use different SPI e.g. CN pins PC10/PC11/PC12)
  #if defined(STM32F303xC) || defined(STM32F303xE)  \
   || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L1XX_NUCLEO) \
   || defined(USE_STM32L4XX_NUCLEO)
  #define  ARD_WIFI_SPI_ID        SPI_ID_1_A        //SPI1 on D13/D12/D11 config    // SPI_ID_3_C

  #elif defined(STM32F334x8)
                 // F3_34 Nucleo has only 1 SPI module - SPI1 on PA5/PA6/PA7 (A) or PB3/PA6/PA7 (B)
  #define  ARD_WIFI_SPI_ID        SPI_ID_1_A

  #elif defined(USE_STM32746G_DISCO)
                 // F7 Discovery Arduino uses different pinout
  #define  ARD_WIFI_SPI_ID        SPI_ID_2_I

  #else
                 // F0, F1, L0, ... need to use different CN pin set e.g. CN pins PB13/PB14/PB15
  #define  ARD_WIFI_SPI_ID        SPI_ID_2_B
  #endif
#endif



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                            CC3000  WiFi Board                       (NUCLEOs)
//
//                              Adafruit xxx
//
//  SPI:    uses D3/D12/D11 (PB3/PA6/PA7) for SPI SCLK/MISO/MOSI
//  SPI CS: uses D10 (PB6) for Chip Select
//  IRQ:    uses A0  (PA0) for Interrupt Request
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CC3000_SPI_ID       SPI_ID_1_B                        /* PB3/PA6/PA7 */
#define  CC3000_SPI_CS       D10
#define  CC3000_IRQ          xx

#endif                        // __DEVICE_CONFIG_COMMON_STM32_H__

