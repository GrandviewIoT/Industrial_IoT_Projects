
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                 device_config_discovery.h
//
//
// This contains the definitions for the various STM32 Discovery boards,
// including assignments for built-in sensors like Accelerometrs, Gyros, etc
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


#ifndef  __DEVICE_CONFIG_DISCOVERY_STM32_H__
#define  __DEVICE_CONFIG_DISCOVERY_STM32_H__


           // 10/29/15  -- ADD BEGIN --
#if defined(USE_STM32L476G_DISCO_REVB)
               // Discovery:  SPI1   PE13/PE14/PE15
#define  SPI_MODULE  SPI_M1
#define  SPI_SCLK    PE13
#define  SPI_MISO    PE14
#define  SPI_MOSI    PE15
#define  GPIO_PORT   GPIOE
#define  GPIO_SPI_AF GPIO_AF5_SPI1
#define  LED_X       PE8
#else
               // Nucleo:     SPI3   PC10/PC11/PC12
#define  SPI_MODULE  SPI_M3
#define  SPI_SCLK    PC10
#define  SPI_MISO    PC11
#define  SPI_MOSI    PC12
#define  GPIO_PORT   GPIOC
#define  GPIO_SPI_AF GPIO_AF6_SPI3
#define  LED_X       PA5
#endif
// Both boards use PB6 for SPI CS Chip Select
           // 10/29/15  -- ADD  END  --



#if defined(USE_STM32746G_DISCO) || defined(USE_STM32746G_DISCOVERY)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                            F7  Discovery  Definitions
//
//        F7 Discovery includes an Arduino-Compatible header similar to Nucleos
//
//        A number of pins are mapped differently on the F7's Arduino Header
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS   0        // Startup with HSI as default

                    //----------------------------------------------
                    //                Digital Inputs
                    //----------------------------------------------
#define  D15      PB8_PAP            /* default I2C1 SCL   PB8  TIM4_CH3/TIM10_CH1  */
#define  D14      PB9_PAP            /* default I2C1 SDA   PB9  TIM4_CH4/TIM11_CH1  */
#define  D13      PI1_PAP            /* default SPI2 SCLK  PI1                */
#define  D12      PB14_PAP           /* default SPI2 MISO  PB14 TIM1_CH2N/TIM8_CH2N */
#define  D11      PB15_PAP           /* default SPI2 MOSI  PB15 TIM1_CH3N/TIM8_CH3N */
#define  D10      PI0_PAP            /* default SPI CS pin      TIM5_CH4       */
#define  D9       PA15_PAP           /*                         TIM2_CH1_ALT_1 */

#define  D8       PI2_PAP            /*                         TIM8_CH4       */
#define  D7       PI3_PAP
#define  D6       PH6_PAP            /*                         TIM12_CH1      */
#define  D5       PA8_PAP            /*                         TIM1_CH1       */
#define  D4       PG7_PAP
#define  D3       PB4_PAP            /*                         TIM3_CH1       */
#define  D2       PG6_PAP

#define  D1       PC6                /* TX VCP - do not use unless re-solder */
#define  D0       PC7                /* RX VCP - do not use unless re-solder */

#define  LED1     D13             // on-board LED is connected to D13 pin (PI_1)

#define  USER_BUTTON       PI11   // on-board Pushbutton connected to PI_11
#define  USER_BUTTON_IRQn  EXTI15_10_IRQn            // associated EXTI interrupt

             //----------------------------------------------------------------------
             //                            Analog Inputs
             //
             // Note that F7 ADC assignments are much different than generic Nucleos.
             // The A0-A5 pin numbers remain the same, but the actual _physical_ ADC
             // channel that they refer to has completely changed in all cases.
             //----------------------------------------------------------------------
#define  A0_PIN      PA0
#define  A1_PIN      PF10
#define  A2_PIN      PF9
#define  A3_PIN      PF8
#define  A4_PIN      PF7
#define  A5_PIN      PF6
#define  A0_CHANNEL  ADC_CHAN_0    // ADC123
#define  A1_CHANNEL  ADC_CHAN_8    // ADC3    -- all ADC3 --
#define  A2_CHANNEL  ADC_CHAN_7    // ADC3
#define  A3_CHANNEL  ADC_CHAN_6    // ADC3
#define  A4_CHANNEL  ADC_CHAN_5    // ADC3
#define  A5_CHANNEL  ADC_CHAN_3    // ADC3

                     //------------------------------------------------
                     //             Virtual Comm Port  to  PC    (VCP)
                     //------------------------------------------------
#define  VCP_TX_PIN  D1
#define  VCP_RX_PIN  D0

              //----------------------------------------------
              //         Grove Base Shield Definitions
              //----------------------------------------------
#define  GROVE_A0   A0_CHANNEL
#define  GROVE_A1   A1_CHANNEL
#define  GROVE_A2   A2_CHANNEL
#define  GROVE_A3   A3_CHANNEL
#define  GROVE_A4   A4_CHANNEL
#define  GROVE_A5   A5_CHANNEL

#endif                                // F7 Discovery



#if defined(USE_STM32746G_DISCO) || defined(USE_STM32746G_DISCOVERY)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   F7 46  NG    Discovery
//
//   Gyro       = L3GD20     (SPI)
//   Accel      = LSM303C    (SPI)    callouts to LSM303DLHC exist from old REVA
//   MagCompass = LSM303C    (SPI)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//  TBD

#endif




#if defined(USE_STM32F401_DISCO) || defined(USE_STM32F4XX_Discovery)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   F4 01  xx    Discovery
//
//   Gyro = L3GD20  (SPI)
//   Accel/MagCompass = LSM303DLHC  (I2C)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS   0        // Startup with HSI as default

#define  LED1             PD13_PAP  // map to on-board "LED3" which is on GPIO PD13
#define  LED2             PD12_PAP  // map to on-board "LED4" which is on GPIO PD12

#define  USER_BUTTON      PA0_PAP   // on-board Pushbutton always connected to PA0
#define  USER_BUTTON_IRQn EXTI0_IRQn                // associated EXTI interrupt

#define  MAGNET                /* comment out to remove code for MagCompass */

#define  I2C_DUTY_CYCLE_VAL   I2C_DUTYCYCLE_2    // F4 requires a I2C_DUTYCYCLE
#define  I2C_CLOCK_SPEED      400000

#define  USES_ACCELEROMETER
#define  ACCEL_CMP_MAX_COMMUNICATION_FREQ       ((uint32_t) 100000)

#define  ACCEL_CMP_I2C_MODULE I2C_ID_1_B        /*  PB6 / PB9          I2C1 */
#define  ACCEL_CMP_DRDY_IRQ   PE2               /*  PE2   Data Ready IRQ    */
#define  ACCEL_CMP_IRQ_1      PE4               /*  PE4   Interrupt Req 1   */
#define  ACCEL_CMP_IRQ_2      PE5               /*  PE5   Interrupt Req 1   */

#define  GYRO_SPI_MODULE      SPI_ID_1_A        /*  PA5 / PA6 / PA7    SPI1 */
#define  GYRO_CS              PE3               /*  PE3   Chip Select       */
#define  GYRO_IRQ_1           PE0               /*  PE0   Interrupt Req 1   */
#define  GYRO_IRQ_2           PE1               /*  PE1   Interrupt Req 1   */

#if defined(USES_GYRO)
#include "stm32f401_discovery_accelerometer.h"
#include "stm32f401_discovery_gyroscope.h"
#endif

 #ifndef USES_I2C
 #error "need to add USES_I2C to compiler defines"
 #endif
 #ifndef USES_SPI
 #error "need to add USES_SPI to compiler defines"
 #endif

#endif                              // defined(USE_STM32F4XX_Discovery)




#if defined(USE_STM32F3348_DISCO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   F3 34  xx    Discovery             (SMPS)
//
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS      0    // Use default HSI with PLL

#define  LED1             PB6  // map to on-board "LED3" which is on GPIO PB6
#define  LED2             PB8  // map to on-board "LED4" which is on GPIO PB8


/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 72 MHz */
/* This example use TIMING to 0x00C4092A to reach 1 MHz speed (Rise time = 26ns, Fall time = 2ns) */
#define I2C_TIMING      0x00C4092A              // F3 requires a I2C_TIMING

#endif                         // defined(USE_STM32F3348_DISCO)




#if defined(USE_STM32F3_DISCO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   F3 03  xx    Discovery
//
//
//   Gyro = L3GD20  (SPI)
//   Accel/MagCompass = LSM303DLHC  (I2C)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//#define  CLOCK_FLAGS   1       // Need to start F3 Discovery using HSE, not HSI
#define  CLOCK_FLAGS  80       // Do not use PLL. Run with initial startup clock (clock debugging)

#define  LED1             PE9  // map to on-board "LED3" which is on GPIO PE9
#define  LED2             PE8  // map to on-board "LED4" which is on GPIO PE8

#define  MAGNET                /* comment out to remove code for MagCompass */

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 72 MHz */
/* This example use TIMING to 0x00C4092A to reach 1 MHz speed (Rise time = 26ns, Fall time = 2ns) */
#define I2C_TIMING      0x00C4092A              // F3 requires a I2C_TIMING

#define  USES_ACCELEROMETER
#define  ACCEL_CMP_MAX_COMMUNICATION_FREQ       ((uint32_t) 100000)

#define  ACCEL_CMP_I2C_MODULE I2C_ID_1_C        /*  PB6 / PB7          I2C1 */
#define  ACCEL_CMP_DRDY_IRQ   PE2               /*  PE2   Data Ready IRQ    */
#define  ACCEL_CMP_IRQ_1      PE4               /*  PE4   Interrupt Req 1   */
#define  ACCEL_CMP_IRQ_2      PE5               /*  PE5   Interrupt Req 1   */

#define DISCOVERY_I2Cx        I2C1              /* uses PB6 / PB7           */
#define DISCOVERY_SPIx        SPI1              /* uses PA5 / PA6 / PA7     */

#define  GYRO_SPI_MODULE      SPI_ID_1_A        /*  PA5 / PA6 / PA7    SPI1 */
#define  GYRO_CS              PE3               /*  PE3   Chip Select       */
#define  GYRO_IRQ_1           PE0               /*  PE0   Interrupt Req 1   */
#define  GYRO_IRQ_2           PE1               /*  PE1   Interrupt Req 1   */

#if defined(USES_GYRO)
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#endif

 #ifndef USES_I2C
 #error "need to add USES_I2C to compiler defines"
 #endif
 #ifndef USES_SPI
 #error "need to add USES_SPI to compiler defines"
 #endif
#endif                                // defined(USE_STM32F303_Discov)




#if defined(USE_STM32F072B_DISCO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   F0 72  xx    Discovery
//
//
//   F072 has Gyro = L3GD20 _only_  (SPI)
//        No Accleometer or MagCompass
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS   0          // Use default HSI with PLL

#define  LED1             PC6     // map to on-board "LED3" which is on GPIO PC6
#define  LED2             PC7     // map to on-board "LED4" which is on GPIO PC7

#define  USER_BUTTON      PA0     // on-board Pushbutton always connected to PA0
#define  USER_BUTTON_IRQn EXTI0_1_IRQn              // associated EXTI interrupt

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 72 MHz */
/* This example use TIMING to 0x00C4092A to reach 1 MHz speed (Rise time = 26ns, Fall time = 2ns) */
//#define I2C_TIMING      0x0070D8FF    // F0 requires a I2C_TIMING
                                        // Refer to AN4235 Application Note
#define I2C_TIMING    DISCOVERY_I2Cx_TIMING     // F0 requires a I2C_TIMING

#define  GYRO_SPI_MODULE      SPI_ID_2_B        /*  PB13 / PB14 / PB15  SPI2 */
#define  GYRO_CS              PC0               /*  PC0   Chip Select        */
#define  GYRO_IRQ_1           PC1               /*  PC1   Interrupt Req 1    */
#define  GYRO_IRQ_2           PC2               /*  PC2   Interrupt Req 1    */

#define  I2C_MODULE           I2C_ID_2_B        /*  PB10 / PB11         I2C2 */

#if defined(USES_GYRO)
#include "stm32f072b_discovery_gyroscope.h"
#endif

 #ifndef USES_I2C
 #error "need to add USES_I2C to compiler defines"
 #endif
 #ifndef USES_SPI
 #error "need to add USES_SPI to compiler defines"
 #endif
#endif                                  // defined(USE_STM32F072B_DISCO)




#if defined(USE_STM32L0538_DISCO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   L0 53  C8    Discovery
//
//
//   L053 has e-paper display
//   Has built-in VCP
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS      0       // Use default HSI with PLL

#define  LED1             PB4     // map to on-board "LED3" which is on GPIO PB4
#define  LED2             PA5     // map to on-board "LED4" which is on GPIO PA5

#define  USER_BUTTON      PA0     // on-board Pushbutton always connected to PA0
#define  USER_BUTTON_IRQn EXTI0_1_IRQn              // associated EXTI interrupt

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 72 MHz */
/* This example use TIMING to 0x00C4092A to reach 1 MHz speed (Rise time = 26ns, Fall time = 2ns) */
#define I2C_TIMING      0x0070D8FF    // F0 requires a I2C_TIMING
                                        // Refer to AN4235 Application Note
//#define I2C_TIMING    DISCOVERY_I2Cx_TIMING     // F0 requires a I2C_TIMING

#define DISCOVERY_SPIx   SPI_ID_1_C     // SPI1 on PB3/PB4/PB5

#define EPD_SPI          SPI_ID_1_C     // SPI1 on PB3/PB4/PB5
#define EPD_CS           PA15           // Chip Select on PA15

#define EPD_DC           PB11           // Command/Data select on PB11
#define EPD_RESET        PB2            // Reset line on PB2
#define EPD_BUSY         PA8            // Busy indication on PA8
#define EPD_PWR_ENABLE   PB10           // Power to screen enable on PB10
#endif                                  // defined(USE_STM32L0538_DISCO)




#if defined(USE_STM32L152C_DISCO)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   L1 52  RC    Discovery
//
//
//   L053 has e-paper display
//   Has built-in VCP
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS      0       // Use default HSI with PLL

#define  LED1             PB7     // map to on-board "LED3" which is on GPIO PB7
#define  LED2             PB6     // map to on-board "LED4" which is on GPIO PB6

#define  USER_BUTTON      PA0     // on-board Pushbutton always connected to PA0
#define  USER_BUTTON_IRQn EXTI0_1_IRQn              // associated EXTI interrupt

#define  I2C_DUTY_CYCLE_VAL   I2C_DUTYCYCLE_2    // F4 requires a I2C_DUTYCYCLE
#define  I2C_CLOCK_SPEED      400000
#endif                                  // defined(USE_STM32L152C_DISCO)




#if defined(USE_STM32L476G_DISCO_REVB)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                      STM32   L4 76  NG    Discovery
//
//   Gyro       = L3GD20     (SPI)
//   Accel      = LSM303C    (SPI)    callouts to LSM303DLHC exist from old REVA
//   MagCompass = LSM303C    (SPI)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define  CLOCK_FLAGS   0        // Startup with HSI as default

#define  LED1             PB2   // map to on-board "LED4" which is on GPIO PB2
#define  LED2             PE8   // map to on-board "LED5" which is on GPIO PE8

#define  USER_BUTTON      PA0     // on-board Pushbutton always connected to PA0
#define  USER_BUTTON_IRQn EXTI0_IRQn                // associated EXTI interrupt

#define  MAGNET                /* comment out to remove code for MagCompass */

#define  I2C1_TIMING          0x90112626         // L4 requires a I2C1_TIMING (100 KHz)

#define  USES_ACCELEROMETER
#define  ACCEL_CMP_MAX_COMMUNICATION_FREQ       ((uint32_t) 100000)

#define  ACCEL_CMP_I2C_MODULE I2C_ID_1_x        /*  PB6 / PB7          I2C1 */
#define  ACCEL_CMP_DRDY_IRQ   PE2               /*  PE2   Data Ready IRQ    */
#define  ACCEL_CMP_IRQ_1      PE4               /*  PE4   Interrupt Req 1   */
#define  ACCEL_CMP_IRQ_2      PE5               /*  PE5   Interrupt Req 1   */

//-- MAG COMPASS DEFs ARE SEPARATE UNIT !

#define  GYRO_SPI_MODULE      SPI_ID_2_F        /*  PD1 / PD3 / PD4    SPI2 */
#define  GYRO_CS              PD7               /*  PE3   Chip Select       */
#define  GYRO_IRQ_1           PE0               /* ??? PE0   Interrupt Req 1   */
#define  GYRO_IRQ_2           PE1               /* ??? PE1   Interrupt Req 1   */

#if defined(USES_GYRO)
#include "stm32l476g_discovery_accelerometer.h"
#include "stm32l476g_discovery_compass.h"
//#include "stm32l476g_discovery_gyroscope.h"      // ESC 2015 API conflicts
#endif

 #ifndef USES_I2C
//#error "need to add USES_I2C to compiler defines"
 #define USES_I2C        1           // force the definition on
 #endif
 #ifndef USES_SPI
//#error "need to add USES_SPI to compiler defines"
 #define USES_SPI        1           // force the definition on
 #endif
#endif                               // defined(USE_STM32L476G_DISCO_REVB)

#endif                               // __DEVICE_CONFIG_DISCOVERY_STM32_H__

