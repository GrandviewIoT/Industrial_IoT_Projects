 // 06/02/05
          // -- F0_72 SPI WIRING IS DIFFERENT THAN F4_02 which uses UPPER LEFT CORNER
          //    PC10/PC11/PC12 and uses SPI 3 ---
          // -- F0_72 USES THE LOWER RIGHT CORNER SPI (PB13/PB14/PB15) on MORPHO
          //    and uses SPI 2 --

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                 board_STM32.h
//
//
// See board.h for "common functions". It includes this file, based on MCU type
//
//
// Board dependent code, moved to a single module, used for different STM32 MCUs
// e.g. STM32 F3-02, F3-34, F4-01, L1-52, ...
//
// Mainly used for low-cost ST Nucleo boards, to allow fast prototyping.
//
//
// Also includes API defs for common routines implemented for each board.
//
// History:
//   01/04/15 - Consolidated separate files into one spot for easier porting.Duq
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


#ifndef  __BOARD_STM32_H__
#define  __BOARD_STM32_H__

#ifndef FALSE
#define  FALSE  0
#define  TRUE   1
#endif

void TimingDelay_Decrement(void);            // Common STM32 Function Prototypes
void Delay(unsigned long delay_interval);


//------------------------------------------------------------------------------
//                     Seeed W5200 Wired Ethernet Summary
//
//  Seeed W5200          Cortex-M       Associated
//  Function             Pins Used      "Arduino" Pin
//  ---------            ---------      -------------
//                        SPI1  SPI2
//    SPI CLK              PA5   PB13     D13   SPI 1  "  F1-03, L0-53, L1-52
//    SPI MISO             PA6   PB14     D12   SPI 1 for F3-34, F0-30, F0-72
//    SPI MOSI             PA7   PB15     D11   SPI 2 for F3-02
//    W5200 SPI CS rewired PB3   PB3      D3      -- re-wired --
//
//    RESET                PB8            D8    -- Not used on Seeed Studio --
//    PWRDN_STANDBY        PB9            D7    -- Not used on Seeed Studio --
//    Interrupt IRQ Flag   PB0            D2    -- Not currently used in code
//
//  CAUTION:
//   (1) STM32 Nucleo series does not natively support the UNO 6-pin ICSP Header
//       which is used by a number of UNO shields like Ethernet and WiFi.
//       The Seeed W5200 shield (as well as others), does NOT run any wires to
//       the standard "Arduino" SPI header pins of D11/D12/D13. Instead, it
//       ONLY runs SPI I/O pins to the 6-pin ICSP Header in the bottom middle
//       of the shield, and the UNO board. (See associated pictures on GitHub).
//
//       So to get the Seeed Studio W5200 shield to work with any STM32 Nucleo
//       board, you MUST run 4 male to female jumber wires from the following
//       STM32 Nucleo D11-D13 pins to the 6-pin ICSP Header on the Seeed Shield:
//           Nucleo Header           Seeed ICSP   (Arduino UNO Schematic Naming)
//           -------------           ----------
//           SPI CLK   D13             pin 3  (middle top of connector)
//           SPI MISO  D12             pin 1  (right  top  "    "     )
//           SPI MOSI  D11             pin 4  (middle bottom "   "    )
//           SPI CS    D6            N/A - is on Arduino D10 instead
//
//   (2) The standard W5200 SPI CS normally uses Nucleo pin D10 (chip pin PB6).
//       BUT, this conflicts with the EasySpin Stepper Shield, which is wired
//       to use the same Nucleo pin D10 (chip pin PB6).
//
//       Since we need to run male to female wires for the W5200 already, we
//       route the W5200 CS ICSP pin to Nucleo Arduino pin D4 (chip PB5) instead
//       and change the W5200 code for WIZ_SCS to use the chip's PB5 pin
//       instead. This eliminates the SPI CS conflict, and allows both boards
//       to be mounted and work together on any of the STM32 Nucleo boards.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//                     L6424 Easy Spin XNucleo Summary
//
//  EasySpin             Cortex-M       Associated
//  Function             Pins Used      "Arduino" Pin
//  ---------            ---------      -------------
//                        SPI1  SPI2
//    SPI CLK              PA5   PB13       D13   SPI 1  "  F1-03, L0-53, L1-52
//    SPI MISO             PA6   PB14       D12   SPI 1 for F3-34, F0-30, F0-72
//    SPI MOSI             PA7   PB15       D11   SPI 2 for F3-02
//    SPI CS               PB6   PB6        D10
//
//    Interrupt Flag       PA10             D2
//    RESET                PA9              D8
//    DIR_1                PA8              D7
//    Step_1 / PWM_1       PC7              D9   Uses Timer3 Ch2  NOT ON F3-02 !
//
//  CAUTION:  PC7 has Timer3 Ch2 for _all_ chips _except_ F3-02 and L0-53
//
//                ==> F3-02 cannot support Easy-Spin !
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                      BlueNRG BLE XNucleo Summary
//
//  NFC                  Cortex-M       Associated
//  Function             Pins Used      Arduino Pin
//  ---------            ---------      ------------
//    tbd                  PCx            Dx
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                      MEMS/ENV XNucleo Summary
//
//  NFC                  Cortex-M       Associated
//  Function             Pins Used      Arduino Pin
//  ---------            ---------      ------------
//    tbd                  PCx            Dx
//------------------------------------------------------------------------------



                  //--------------------------------------------------
                  //   STM32  Key  I/O Pin  definitions by processor
                  //--------------------------------------------------

#if defined(STM32F030x8) || defined(STM32F070xB) || defined(STM32F072xB) || defined(STM32F091xC)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F0_72_RB  / F0_91_RC   Pin mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                 STM32  F0  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        48000000           // Max Clock Frequency 48 MHz

#include "stm32f0xx_hal.h"            // pull in basic MCU clock and SYSCFG defs
#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_hal_i2c.h"        // Keil is acting stupid
#include "stm32f0xx_hal_tim.h"        //        ditto
#include "stm32f0xx_hal_uart.h"
#include "stm32f0xx_hal_usart.h"

#include "STM32_F0\stm32f0xx.h"       // Pull in chip detailed defs
#include "STM32_F0\system_stm32f0xx.h"
#include "STM32_F0\stm32f0xx_it.h"

#if defined(USE_STM32F0XX_NUCLEO)
      // uses IRQ for EXTI4_15_IRQn on pin PC_13 for push button
  #include "stm32f0xx_nucleo.h"
#endif

#if defined(USE_STM32F072B_DISCO)
#include "stm32f072b_discovery.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_1_IRQn on pin PA_0          (EXTI0_1_IRQHandler ISR)
  #include "stm32f0xx_nucleo_bluenrg.h"      // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

             // LED1 port and pin Definitions
#define  LED1_PORT               GPIOA
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN;

             // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT     GPIOC
#define  KEY_BUTTON_PIN           GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
#define  KEY_BUTTON_EXTI_IRQn     EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - F0_72 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

#define  SPI_1_A_SCLK_PIN_AF    5,GPIO_AF_SPI1       /* Alt Funtion defs */
#define  SPI_1_A_MISO_PIN_AF    6,GPIO_AF_SPI1
#define  SPI_1_A_MOSI_PIN_AF    7,GPIO_AF_SPI1

//#define  SPI_ID_2_B             2           // SPI 2 using pins PB13/PB14/PB15
#define  SPI_2_B_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_B_MISO_PIN       GPIO_PIN_14  //   PB14 - MOSI
#define  SPI_2_B_MOSI_PIN       GPIO_PIN_15  //   PB15 - MISO
#define  SPI_2_B_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_B_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_2_C             12           // SPI 2 using pins PB13/PC2/PC3
#define  SPI_2_C_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_C_MISO_PIN       GPIO_PIN_2   //   PC2  - MOSI
#define  SPI_2_C_MOSI_PIN       GPIO_PIN_3   //   PC3  - MISO
#define  SPI_2_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_C_MOSx_PULL_MODE GPIO_PULLUP

#if defined(USES_BLUENRG_BLE)
            //-----------------------------------------------------------
            //
            //        --- These use the identical settings as F4 01 ---
            //
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F0 72
            //                     using SPI 1      (PB3/PA6/PA7)
            //-----------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    F0 72
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //--------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs   PA_0   F0 72
            //--------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_1_IRQn      /* Uses EXT 0 rupt  F0_72 DIFF  */
//#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt  F4_01 DIFF  */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //----------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs   PA_8   F0 72
            //----------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE

#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //-----------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F0 72
            //                     using SPI 2      (PB13/PB14/PB15)
            // -- F0_72 WIRING IS DIFFERNT THAN F4_02 which uses UPPER LEFT CORNER
            //    PC10/PC11/PC12 and uses SPI 3 ---
            // -- F0_72 USES THE LOWER RIGHT CORNER (PB13/PB14/PB15) on MORPHO
            //    and uses SPI 2 --
            //-----------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_2_B
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

// 06/02/15  --- APPARENTLY THESE ARE NO LONG USED ---
#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#endif                                //   defined(__STM32F072__)
                                      //----------------------------



#if defined(STM32F103xB)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F1_03_RB   Pin mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                    STM32 F1 Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        72000000           // Max Clock Frequency 72 MHz

#include "stm32f1xx_hal.h"            // pull in basic MCU clock and SYSCFG defs
//#include "stm32f103b_discovery.h"

#include "STM32_F1\stm32f1xx.h"
#include "STM32_F1\system_stm32f1xx.h"
#include "STM32_F1\stm32f1xx_it.h"

#if defined(USE_STM32F1XX_NUCLEO)
      // uses IRQ for EXTI15_10_IRQn on pin PC_13 for push button
  #include "stm32f1xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32f1xx_nucleo_bluenrg.h"      // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ    MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

void TimingDelay_Decrement(void);             // Function Prototypes
void Delay(unsigned long nTime);

             // LED port and pin Definitions
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN;

             // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT     GPIOC
#define  KEY_BUTTON_PIN           GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
#define  KEY_BUTTON_EXTI_IRQn     EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - F1_03 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MISO
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MOSI
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MISO
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MOSI
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_1_C             21           // SPI 1 using pins PB3/PB4/PB5
#define  SPI_1_C_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_C_MISO_PIN       GPIO_PIN_4   //   PB4 - MISO
#define  SPI_1_C_MOSI_PIN       GPIO_PIN_5   //   PA7 - MOSI
#define  SPI_1_C_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_C_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_2_B             2           // SPI 2 using pins PB13/PB14/PB15
#define  SPI_2_B_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_B_MISO_PIN       GPIO_PIN_14  //   PB14 - MOSI
#define  SPI_2_B_MOSI_PIN       GPIO_PIN_15  //   PB15 - MISO
#define  SPI_2_B_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_B_MOSx_PULL_MODE GPIO_PULLUP

#if defined(USES_BLUENRG_BLE)
            //-----------------------------------------------------------
            //
            //        --- These use the identical settings as F4 01 ---
            //
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F1 03
            //                     using SPI 1      (PB3/PA6/PA7)
            //-----------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    F1 03
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //--------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs   PA_0   F1 03
            //--------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
//#define  BlueNRG_EXTI_IRQn           EXTI0_1_IRQn      /* Uses EXT 0 rupt  F0_72 DIFF  */
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt  F1_03 and F4_01 DIFF  */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //----------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs   PA_8   F1 03
            //----------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE

#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //-----------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F1 03
            //                     using SPI 2      (PB13/PB14/PB15)
            // -- F1_03 WIRING IS DIFFERNT THAN F4_02 which uses UPPER LEFT CORNER   ???
            //    PC10/PC11/PC12 and uses SPI 3 ---
            // -- F0_72 USES THE LOWER RIGHT CORNER (PB13/PB14/PB15) on MORPHO
            //    and uses SPI 2 --
            //-----------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_2_B
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

// 06/02/15  --- APPARENTLY THESE ARE NO LONG USED ---
#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#endif                                //   defined(STM32F103xB)
                                      //----------------------------


#if defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F302x8)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F3_03_R8 and F3_02_R8  Pin mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//
// F303 Discovery = STM32F303xC
// F303 Nucleo    = STM32F303xE
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           // STM32 F3_03  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED           72000000       // Main Clock Frequency 72 MHz

#include "stm32f3xx_hal.h"            // pull in basic MCU clock and SYSCFG defs

#include "stm32f3xx_hal_adc.h"
#include "stm32f3xx_hal_adc_ex.h"

#include "STM32_F3\stm32f3xx.h"
#include "STM32_F3\system_stm32f3xx.h"
#include "STM32_F3\stm32f3xx_it.h"

#if defined(USE_STM32F3XX_NUCLEO)
#include "stm32f3xx_nucleo.h"
#endif

#if defined(USE_STM32F303_DISCO) || defined(USE_STM32F303_DISCOVERY)
#include "stm32f3_discovery.h"
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

            // LED port and pin Definitions
#define  LED_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  LED_PORT                GPIOA
#define  LED_3_PIN               GPIO_PIN_5      // out  Nucleo is PA5, Discovery PA0
#define  LED_4_PIN               GPIO_PIN_1      // out

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

      //===============================================================
      //                    Nucleo - F3_02 Platform    -    uses SPI2
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PB13     Digital # 13
      //   SPI MISO             PB14     Digital # 12
      //   SPI MOSI             PB15     Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //===============================================================
      // SPI General Defines
#define TWEAK_RX_TX_FIFO          1                // STM32 F3's can tweak FIFO
#define DMA_WINDOW_SIZE           1024
#define SPI_WINDOW_SIZE           DMA_WINDOW_SIZE
#define FWT_DELAY                 4000
#define SPI_TIMEOUT_MAX           100              // worst case

            //------------------------------------------
            // SPI port and pin Definitions  - F3 02R8
            //------------------------------------------
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB1Periph_SPI2
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOB
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI2

#define  SPI_GPIOs_PORT      GPIOB //Generic def used when all pins on same port
#define  SPI_SCLK_PORT       GPIOB
#define  SPI_MISO_PORT       GPIOB
#define  SPI_MOSI_PORT       GPIOB
#define  SPI_WIRED_CS_PORT   GPIOB
#define  SPI_WIFI_CS_PORT    GPIOB

#define  SPI_SCLK_PIN        GPIO_PIN_13  // PB13 - SCLK
#define  SPI_MISO_PIN        GPIO_PIN_14  // PB14 - MOSI
#define  SPI_MOSI_PIN        GPIO_PIN_15  // PB15 - MISO
#define  SPI_WIRED_CS_PIN    GPIO_PIN_3   // PB3  - CS for W5200 Wired Ethernet
#define  SPI_WIFI_CS_PIN     GPIO_PIN_6   // PB6  - CS for CC3000 WiFi Ethernet

#define  SPI_SCLK_PIN_AF     13,GPIO_AF_5
#define  SPI_MISO_PIN_AF     14,GPIO_AF_5
#define  SPI_MOSI_PIN_AF     15,GPIO_AF_5

            //----------------------------------------------------
            // L6474 Stepper SPI CS GPIO Port and Pin Defs  F3 02
            //----------------------------------------------------
#define  L6474_SPI_CS_PORT   GPIOB
#define  L6474_SPI_CS_PIN    GPIO_PIN_6                  // out PB6   D10 (Ardu)

#if defined(USES_W5200)
#define  W5200_SPI_PORT_ID     SPI_ID_3_C
#define  W5200_SPI_MODE        0
#define  W5200_CS_NORMAL_PORT  1                         // GPIOB
#define  W5200_CS_NORMAL_PIN   GPIO_PIN_3                // out PB3     D3
            //------------------------------------------
            // W5200 WIRED GPIO Port and Pin Defs  F3 02
            //------------------------------------------ // Seed Studio Pinouts
                                                         //           Arduino
#define  WIZ_SCLK            SPI_SCLK_PIN                // out PB13    D13
#define  WIZ_MISO            SPI_MISO_PIN                // in  PB14    D12
#define  WIZ_MOSI            SPI_MOSI_PIN                // out PB15    D11
#define  WIZ_SCS             GPIO_PIN_3                  // out PB3     D3
#define  WIZ_INT             GPIO_PIN_0                  // in  PB0
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

  // BSRRL sets a bit.  BSRRH resets/clears a bit.
#define  ASSERT_WIRED_CS()   SPI_WIRED_CS_PORT->BSRRL = WIZ_SCS
#define  DEASSERT_WIRED_CS() SPI_WIRED_CS_PORT->BSRRH = WIZ_SCS
#define  DEASSERT_CS_W5200_NORMAL() SPI_WIRED_CS_PORT->BSRRH = WIZ_SCS

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
#if defined(SEEED_STUDIO_W5200)
          // Note: if Seeed Studio version, PWDN, INT, ENABLE pins are not used.
          // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                                   // defined(SEEED_STUDIO_W5200)
#endif                                   // defined(USES_W5200)

            //------------------------------------------
            // CC3000 WIFI GPIO Port and Pin Definitions
            //------------------------------------------
             // CC3000 uses SPI2 on F3_02 Nucleo
#define  CC3000_SPI          SPI2
#define  CC3000_SPI_CLK      RCC_APB1Periph_SPI2

#define  SPI_SCK_PIN         SPI_SCLK_PIN                // PB13
#define  SPI_SCK_GPIO_PORT   SPI_SCLK_PORT               // GPIOB
#define  SPI_SCK_GPIO_CLK    RCC_AHBPeriph_GPIOB
#define  SPI_SCK_SOURCE      GPIO_PinSource13
#define  SPI_SCK_AF          GPIO_AF_5

#ifdef MERGE_THESE
#define  SPI_MISO_GPIO_PORT  SPI_MISO_PORT               // GPIOB
#define  SPI_MISO_GPIO_CLK   RCC_AHBPeriph_GPIOB
#define  SPI_MISO_SOURCE     GPIO_PinSource14
#define  SPI_MISO_AF         GPIO_AF_5

#define  SPI_MOSI_GPIO_PORT  SPI_MOSI_PORT               // GPIOB
#define  SPI_MOSI_GPIO_CLK   RCC_AHBPeriph_GPIOB
#define  SPI_MOSI_SOURCE     GPIO_PinSource15
#define  SPI_MOSI_AF         GPIO_AF_5
#endif
            // CC3000 Chip Select pin - PB.6  Arduino Hdr D-10
#define SPI_CS_CC3000_PIN         SPI_WIFI_CS_PIN        // PB.6
#define SPI_CS_CC3000_GPIO_PORT   GPIOB                  /* GPIOB */
#define SPI_CS_CC3000_GPIO_CLK    RCC_AHBPeriph_GPIOB

#define ASSERT_WIFI_CS()     SPI_CS_CC3000_GPIO_PORT->BSRRH = SPI_CS_CC3000_PIN
#define DEASSERT_WIFI_CS()   SPI_CS_CC3000_GPIO_PORT->BSRRL = SPI_CS_CC3000_PIN

#define  CC3000_INT_PORT     GPIOB       // Interrupt Pin CC3000 WiFi Ethernet
#define  CC3000_INT_PIN      GPIO_PIN_x

#define  CC3000_ENABLE_PORT  GPIOB       // ENABLE Pin CC3000 WiFi Ethernet
#define  CC3000_ENABLE_PIN   GPIO_PIN_x

            // CC3000 VBAT ENABLE pin - PB.4  Arduino Hdr D-5
#ifdef MERGE_THESE
#define CC3000_ENABLE_PIN         GPIO_PIN_4               /* PB.4  */
#define CC3000_ENABLE_GPIO_PORT   GPIOB                    /* GPIOB */
#define CC3000_ENABLE_GPIO_CLK    RCC_AHBPeriph_GPIOB
#endif
            // CC3000 IRQ (Interrupt Req In) pin - PB.3  Arduino Hdr D-3
#define CC3000_IRQ_IN_PIN              GPIO_PIN_3          /* PB.3  */
#define CC3000_IRQ_IN_GPIO_PORT        GPIOB               /* GPIOB */
#define CC3000_IRQ_IN_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define CC3000_IRQ_IN_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define CC3000_IRQ_IN_EXTI_PIN_SOURCE  EXTI_PinSource3
#define CC3000_IRQ_IN_EXTI_LINE        EXTI_Line3
#define CC3000_IRQ_IN_EXTI_IRQn        EXTI3_IRQn

            //  CC3000 control lines (ENABLE, IRQ) management
// Setting the HIGH half causes a Bit Reset, setting the LOW half causes Bit Set
#define CC3000_ENABLE_HIGH()  CC3000_ENABLE_GPIO_PORT->BSRRL = CC3000_ENABLE_PIN
#define CC3000_ENABLE_LOW()   CC3000_ENABLE_GPIO_PORT->BSRRH = CC3000_ENABLE_PIN

            // Unit Testing Definitions for Pin Operation Checkout
#define SPI_CLK_HIGH()        SPI_SCK_GPIO_PORT->BSRRL  = SPI_SCK_PIN
#define SPI_CLK_LOW()         SPI_SCK_GPIO_PORT->BSRRH  = SPI_SCK_PIN
#define SPI_MISO_HIGH()       SPI_MISO_GPIO_PORT->BSRRL = SPI_MISO_PIN
#define SPI_MISO_LOW()        SPI_MISO_GPIO_PORT->BSRRH = SPI_MISO_PIN
#define SPI_MOSI_HIGH()       SPI_MOSI_GPIO_PORT->BSRRL = SPI_MOSI_PIN
#define SPI_MOSI_LOW()        SPI_MOSI_GPIO_PORT->BSRRH = SPI_MOSI_PIN
#define CC3000_IRQ_HIGH()     CC3000_IRQ_IN_GPIO_PORT->BSRRL = CC3000_IRQ_IN_PIN
#define CC3000_IRQ_LOW()      CC3000_IRQ_IN_GPIO_PORT->BSRRH = CC3000_IRQ_IN_PIN

#define USER_BUTTON_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USER_BUTTON_EXTI_PORT_SOURCE  GPIO_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE   GPIO_PinSource0
#define USER_BUTTON_EXTI_LINE         EXTI_Line0
#define USER_BUTTON_EXTI_IRQn         EXTI0_IRQn

            //-----------------------------------------------------
            // DRV8711 GPIO Port and Pin Definitions - STM32 F3-02
            //-----------------------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT                 BIT2      // P4.2
#define  nSLEEP              BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET_PIN           BIT4      // P3.4
#define  STEP_AIN1           BIT5      // P3.5
#define  DIR_AIN2            BIT6      // P3.6
#define  BIN2                BIT4      // P1.4
#define  BIN1                BIT5      // P1.5
#define  nSTALL              BIT2      // P1.2
#define  nFAULT              BIT0      // P3.0

#define  POT_PORT            P4IN    // Note: Function selectors are SEL0/SEL1
#define  nSLEEP_PORT         P2OUT
#define  RESET_PORT          P3OUT
#define  STEP_AIN1_PORT      P3OUT
#define  DIR_AIN2_PORT       P3OUT
#define  BIN2_PORT           P1OUT
#define  BIN1_PORT           P1OUT
#define  nSTALL_PORT         P1IN
#define  nFAULT_PORT         P3IN

#define  NSLEEP_HI_ENABLE    nSLEEP_PORT |= nSLEEP
#define  NSLEEP_LO_DISABLE   nSLEEP_PORT &= ~nSLEEP

#define  RESET_HI_RESET      RESET_PORT  |= RESET_PIN
#define  RESET_LO_RUN        RESET_PORT  &= ~RESET_PIN
#define  STEP_AIN1_HIGH      STEP_AIN1_PORT |= STEP_AIN1
#define  STEP_AIN1_LOW       STEP_AIN1_PORT &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD    DIR_AIN2_PORT |= DIR_AIN2
#define  DIR_AIN2_BACKWARD   DIR_AIN2_PORT &= ~DIR_AIN2

#define  BIN2_HI             BIN2_PORT |= BIN2
#define  BIN2_LO             BIN2_PORT &= ~BIN2

#define  BIN1_HI             BIN1_PORT |= BIN1
#define  BIN1_LO             BIN1_PORT &= ~BIN1

//#define  ASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)  // fails
//#define  DEASSERT_DRV8711_CS() (P1OUT |= BIT3)   // fails
#define  ASSERT_DRV8711_CS()     (P1OUT |= BIT3)   /* P 1.3 */
#define  DEASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)

            // DRV8711 related Defs
//#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz * 1000000)
#define  TOTAL_CCR_CLOCK_CYCLES   (MCU_CLOCK_SPEED)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                                //   defined(__STM32F302__)
                                      //----------------------------




#if defined(STM32F334x8)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F3_34_R8   GPIO mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                    STM32  F3_34_R8 Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED          72000000        // Main Clock Frequency 72 MHz

#include "STM32_F3\stm32f3xx.h"                   // pulls in MCU specific defs
#include "STM32_F3\system_stm32f3xx.h"
#include "STM32_F3\stm32f3xx_it.h"                // Master Interrupt defs

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
#include "stm32f3xx_hal_adc.h"
#include "stm32f3xx_hal_adc_ex.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_hal.h"            // pull in basic MCU clock and SYSCFG defs

#if defined(USE_STM32F3XX_NUCLEO)
      // uses IRQ for EXTI15_10_IRQn on pin PC_13 for push button
  #include "stm32f3xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32f3xx_nucleo_bluenrg.h"      // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif
            // LED port and pin Definitions
#define  LED1_PORT               GPIOA
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
//#define  LED_4_PIN             GPIO_PIN_0    // out    PA0  Only on Discoverys
#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRRL = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRRH = LED_1_PIN
#define  LED1_TOGGLE()           GPIOA->ODR  ^= LED_1_PIN

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

      //==============================================================
      //                   Nucleo - F3_34 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin     Morpho Pin
      //   ----------        ---------   ------------    ----------
      //   SPI SCLK             PA5      Digital # 13     CN10 11  Red  6th down
      //   SPI MISO             PA6      Digital # 12     CN10 13  Yellow
      //   SPI MOSI             PA7      Digital # 11     CN10 15  Green
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //
      // SPI Pinouts summary:   There is only 1 SPI on the STM32 F3_34
      //                           ==> WizNet W5x00 and EasySpin do not co-exist
      //              SCLK / MISO / MOSI
      //       SPI1:   PA5 / PA6  / PA7   - MAIN
      //               PB3 / PA6  / PA7   - ALTERNATE B
      //               PB3 / PB4  / PB5   - ALTERNATE C
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK  AF5
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MISO  AF5
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MOSI  AF5
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK  AF5
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MISO  AF5
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO  AF5
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_1_C             21           // SPI 1 using pins PB3/PB4/PB5
#define  SPI_1_C_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK  AF5
#define  SPI_1_C_MISO_PIN       GPIO_PIN_4   //   PB4 - MISO  AF5
#define  SPI_1_C_MOSI_PIN       GPIO_PIN_5   //   PB5 - MOSI  AF5
#define  SPI_1_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_C_MOSx_PULL_MODE GPIO_NOPULL

#if defined(USES_BLUENRG_BLE)
            //-----------------------------------------------------------
            //
            //        --- These use the identical settings as F4 01 ---
            //
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F0 72
            //                     using SPI 1      (PB3/PA6/PA7)
            //-----------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    F3 34
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRRH = BlueNRG_CS_PIN
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRRL = BlueNRG_CS_PIN

            //--------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs   PA_0   F3 34
            //--------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
//#define  BlueNRG_EXTI_IRQn           EXTI0_1_IRQn    /* Uses EXT 0 rupt  F0_72 DIFF  */
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt  F3_34 DIFF  */
//#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt  F4_01 DIFF  */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //----------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs   PA_8   F3 34
            //----------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRRH = BlueNRG_RESET_PIN
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRRL = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE

#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //-----------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F3 34
            //                     using SPI 2      (PB13/PB14/PB15)
            // -- F0_72 WIRING IS DIFFERNT THAN F4_02 which uses UPPER LEFT CORNER
            //    PC10/PC11/PC12 and uses SPI 3 ---
            // -- F0_72 USES THE LOWER RIGHT CORNER (PB13/PB14/PB15) on MORPHO
            //    and uses SPI 2 --
            //-----------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_1_A
#define  W5200_SPI_MODE              SPI_MODE_0

//#define  W5200_CS_NORMAL_PORT      GPIOB
//#define  W5200_CS_ALTERNATE_PORT   GPIOA
#define  W5200_CS_NORMAL_PORT        1           // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0           // GPIOA

                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRRH = W5200_CS_NORMAL_PIN
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRRL = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

// 06/02/15  --- APPARENTLY THESE ARE NO LONG USED ---
#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

            //----------------------------------------------------
            // L6474 Stepper SPI CS GPIO Port and Pin Defs  F3 34
            //----------------------------------------------------
#define  L6474_SPI_CS_PORT   GPIOB
#define  L6474_SPI_CS_PIN    GPIO_PIN_6                  // out PB6   D10 (Ardu)
            //----------------------------------------
            // CC3000 GPIO Port and Pin Definitions
            //----------------------------------------
#define  CC3000_INT_PORT     GPIOB       // Interrupt Pin CC3000 WiFi Ethernet
#define  CC3000_INT_PIN      GPIO_PIN_x

#define  CC3000_ENABLE_PORT  GPIOB       // ENABLE Pin CC3000 WiFi Ethernet
#define  CC3000_ENABLE_PIN   GPIO_PIN_x

#define  CC3000_CS      BIT3      // P 1.3  SPI CS
#define  CC3000_ENABLE  BIT2      // P 1.2  aka nHIB   may be P 4.1 instead !
#define  CC3000_IRQ     BIT3      // P 2.3

             // CC3000 uses SPI1 on F3_34 Nucleo
#define CC3000_SPI                SPI1
#define CC3000_SPI_CLK            RCC_APB1Periph_SPI1

#define SPI_SCK_PIN               GPIO_PIN_5               /* PA.5  */
#define SPI_SCK_GPIO_PORT         GPIOA                    /* GPIOA */
#define SPI_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define SPI_SCK_SOURCE            GPIO_PinSource5
#define SPI_SCK_AF                GPIO_AF_5

#ifdef MERGE_THESE
#define SPI_MISO_PIN              GPIO_PIN_6               /* PA.6  */
#define SPI_MISO_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MISO_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MISO_SOURCE           GPIO_PinSource6
#define SPI_MISO_AF               GPIO_AF_5

#define SPI_MOSI_PIN              GPIO_PIN_7               /* PA.7  */
#define SPI_MOSI_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MOSI_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MOSI_SOURCE           GPIO_PinSource7
#define SPI_MOSI_AF               GPIO_AF_5
#endif

            // CC3000 Chip Select pin - PB.6  Arduino Hdr D-10
#define SPI_CS_CC3000_PIN         GPIO_PIN_6               /* PB.6  */
#define SPI_CS_CC3000_GPIO_PORT   GPIOB                    /* GPIOB */
#define SPI_CS_CC3000_GPIO_CLK    RCC_AHBPeriph_GPIOB

            // CC3000 VBAT ENABLE pin - PB.4  Arduino Hdr D-5
#ifdef MERGE_THESE
#define CC3000_ENABLE_PIN         GPIO_PIN_4               /* PB.4  */
#define CC3000_ENABLE_GPIO_PORT   GPIOB                    /* GPIOB */
#define CC3000_ENABLE_GPIO_CLK    RCC_AHBPeriph_GPIOB
#endif
            // CC3000 IRQ (Interrupt Req In) pin - PB.3  Arduino Hdr D-3
#define CC3000_IRQ_IN_PIN              GPIO_PIN_3          /* PB.3  */
#define CC3000_IRQ_IN_GPIO_PORT        GPIOB               /* GPIOB */
#define CC3000_IRQ_IN_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define CC3000_IRQ_IN_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define CC3000_IRQ_IN_EXTI_PIN_SOURCE  EXTI_PinSource3
#define CC3000_IRQ_IN_EXTI_LINE        EXTI_Line3
#define CC3000_IRQ_IN_EXTI_IRQn        EXTI3_IRQn

#define ASSERT_WIFI_CS()      SPI_CS_CC3000_GPIO_PORT->BSRRH = SPI_CS_CC3000_PIN
#define DEASSERT_WIFI_CS()    SPI_CS_CC3000_GPIO_PORT->BSRRL = SPI_CS_CC3000_PIN

            //  CC3000 control lines (ENABLE, IRQ) management
// Setting the HIGH half causes a Bit Reset, setting the LOW half causes Bit Set
#define CC3000_ENABLE_HIGH()  CC3000_ENABLE_GPIO_PORT->BSRRL = CC3000_ENABLE_PIN
#define CC3000_ENABLE_LOW()   CC3000_ENABLE_GPIO_PORT->BSRRH = CC3000_ENABLE_PIN

            // Unit Testing Definitions for Pin Operation Checkout
#define SPI_CLK_HIGH()        SPI_SCK_GPIO_PORT->BSRRL  = SPI_SCK_PIN
#define SPI_CLK_LOW()         SPI_SCK_GPIO_PORT->BSRRH  = SPI_SCK_PIN
#define SPI_MISO_HIGH()       SPI_MISO_GPIO_PORT->BSRRL = SPI_MISO_PIN
#define SPI_MISO_LOW()        SPI_MISO_GPIO_PORT->BSRRH = SPI_MISO_PIN
#define SPI_MOSI_HIGH()       SPI_MOSI_GPIO_PORT->BSRRL = SPI_MOSI_PIN
#define SPI_MOSI_LOW()        SPI_MOSI_GPIO_PORT->BSRRH = SPI_MOSI_PIN
#define CC3000_IRQ_HIGH()     CC3000_IRQ_IN_GPIO_PORT->BSRRL = CC3000_IRQ_IN_PIN
#define CC3000_IRQ_LOW()      CC3000_IRQ_IN_GPIO_PORT->BSRRH = CC3000_IRQ_IN_PIN

            //-----------------------------------------------------
            // DRV8711 GPIO Port and Pin Definitions - STM32 F3 34
            //-----------------------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT            BIT2      // P4.2
#define  nSLEEP         BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET_PIN      BIT4      // P3.4
#define  STEP_AIN1      BIT5      // P3.5
#define  DIR_AIN2       BIT6      // P3.6
#define  BIN2           BIT4      // P1.4
#define  BIN1           BIT5      // P1.5
#define  nSTALL         BIT2      // P1.2
#define  nFAULT         BIT0      // P3.0

#define  POT_PORT              P4IN    // Note: Function selectors are SEL0/SEL1
#define  nSLEEP_PORT           P2OUT
#define  RESET_PORT            P3OUT
#define  STEP_AIN1_PORT        P3OUT
#define  DIR_AIN2_PORT         P3OUT
#define  BIN2_PORT             P1OUT
#define  BIN1_PORT             P1OUT
#define  nSTALL_PORT           P1IN
#define  nFAULT_PORT           P3IN

#define  NSLEEP_HI_ENABLE      nSLEEP_PORT |= nSLEEP
#define  NSLEEP_LO_DISABLE     nSLEEP_PORT &= ~nSLEEP

#define  RESET_HI_RESET        RESET_PORT  |= RESET_PIN
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET_PIN

#define  STEP_AIN1_HIGH        STEP_AIN1_PORT |= STEP_AIN1
#define  STEP_AIN1_LOW         STEP_AIN1_PORT &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      DIR_AIN2_PORT |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     DIR_AIN2_PORT &= ~DIR_AIN2

#define  BIN2_HI               BIN2_PORT |= BIN2
#define  BIN2_LO               BIN2_PORT &= ~BIN2

#define  BIN1_HI               BIN1_PORT |= BIN1
#define  BIN1_LO               BIN1_PORT &= ~BIN1

//#define  ASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)  // fails
//#define  DEASSERT_DRV8711_CS() (P1OUT |= BIT3)   // fails
#define  ASSERT_DRV8711_CS()     (P1OUT |= BIT3)   /* P 1.3 */
#define  DEASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)

            // DRV8711 related Defs
//#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz * 1000000)
#define  TOTAL_CCR_CLOCK_CYCLES   (MCU_CLOCK_SPEED)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                                //   defined(__STM32F334__)
                                      //----------------------------




#if defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F411xE) || defined(STM32F446xx)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F4_01_RE / _VC   GPIO mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc.
//         _RE = F401 Nucleo,    _VC = F401 Discovery
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                      STM32   F4_01  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED            72000000      // Main Clock Frequency 72 MHz
#define  MCU_CLOCK_SPEED_84_MHz     84000000      /* F4_01 */
#define  MCU_CLOCK_SPEED_100_MHz   100000000      /* F4_11 */
#define  MCU_CLOCK_SPEED_180_MHz   180000000      /* F4_46 */

#include "STM32_F4\stm32f4xx.h"
#include "STM32_F4\system_stm32f4xx.h"
#include "STM32_F4\stm32f4xx_it.h"

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"           // pull in basic MCU clock and SYSCFG defs

#if defined(USE_STM32F4XX_NUCLEO) || defined(USES_NUCLEO)
      // uses IRQ for EXTI15_10_IRQn on pin PC_13 for push button
  #include "stm32f4xx_nucleo.h"
#endif

#if defined(USE_STM32F4XX_Discovery) || defined(USES_DISCOVERY)
  #include "stm32f401_discovery.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32f4xx_nucleo_bluenrg.h"     // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"       // pull in needed W5200 include files
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

            // LED port and pin Definitions
#define  LED1_PORT               GPIOA
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
//#define  LED_4_PIN             GPIO_PIN_1    // out    PA1  Only on Discoverys

#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT     GPIOC
#define  KEY_BUTTON_PIN           GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
#define  KEY_BUTTON_EXTI_IRQn     EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - F4_01 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

#define  SPI_1_A_SCLK_PIN_AF    5,GPIO_AF_SPI1       /* Alt Funtion defs */
#define  SPI_1_A_MISO_PIN_AF    6,GPIO_AF_SPI1
#define  SPI_1_A_MOSI_PIN_AF    7,GPIO_AF_SPI1

//#define  SPI_ID_2_C             2            // SPI 2 using pins PB13/PC2/PC3
#define  SPI_2_C_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_C_MISO_PIN       GPIO_PIN_2   //   PC2  - MOSI
#define  SPI_2_C_MOSI_PIN       GPIO_PIN_3   //   PC3  - MISO
#define  SPI_2_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_C_MOSx_PULL_MODE GPIO_PULLUP


//#define  SPI_ID_3_C             3           // SPI 3 using pins PC10/PC11/PC12
#define  SPI_3_C_SCLK_PIN       GPIO_PIN_10  //   PC10 - SCLK
#define  SPI_3_C_MISO_PIN       GPIO_PIN_11  //   PC11 - MOSI
#define  SPI_3_C_MOSI_PIN       GPIO_PIN_12  //   PC12 - MISO
#define  SPI_3_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_3_C_MOSx_PULL_MODE GPIO_PULLUP

#if defined(USES_L6474)
            //--------------------------------------------------------
            // ST's L6474 Stepper/BDC Motro Ctl Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 1      (PA5/PA6/PA7)
            //--------------------------------------------------------
            //----------------------------------------------------
            // L6474 Stepper SPI CS GPIO Port and Pin Defs  F4 01
            //----------------------------------------------------
#define  L6474_SPI_PORT_ID    SPI_ID_1_A
#define  L6474_SPI_MODE       SPI_MODE_3
#define  L6474_SPI_CS_PORT    GPIOB
#define  L6474_SPI_CS_PIN     GPIO_PIN_6             // out PB6   D10 (Ardu)
#endif                             // USES_L6474_STEPPER

#if defined(USES_BLUENRG_BLE)
            //--------------------------------------------------------
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 1      (PB3/PA6/PA7)
            //--------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    F4 01
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //---------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs    PA_0   F4 01
            //---------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //---------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs  PA_8   F4 01
            //---------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE


#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //--------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 3      (PC10/PC11/PC12)
            //--------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_3_C
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#if defined(USES_CC3000)
            //-------------------------------------------------------
            // Adafruit's TI CC3000 WiFi Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 3      (PC10/PC11/PC12)
            //-------------------------------------------------------
#define  CC3000_INT_PORT     GPIOB       // Interrupt Pin CC3000 WiFi Ethernet
#define  CC3000_INT_PIN      GPIO_PIN_x

#define  CC3000_ENABLE_PORT  GPIOB       // ENABLE Pin CC3000 WiFi Ethernet
#define  CC3000_ENABLE_PIN   GPIO_PIN_x

            //----------------------------------------
            // CC3000 CS GPIO Port and Pin Definitions
            //----------------------------------------
#define ASSERT_WIFI_CS()     (P3OUT &= ~BIT0)     /* P 3.0 */
#define DEASSERT_WIFI_CS()   (P3OUT |= BIT0)

            //----------------------------------------
            // CC3000 GPIO Port and Pin Definitions
            //----------------------------------------
#define  CC3000_CS                BIT3      // P 1.3  SPI CS
#define  CC3000_ENABLE            BIT2      // P 1.2  aka nHIB   may be P 4.1 instead !
#define  CC3000_IRQ               BIT3      // P 2.3

#define  ASSERT_CC3000_CS()       (P1OUT &= ~BIT3)     /* P 1.3 */
#define  DEASSERT_CC3000_CS()     (P1OUT |= BIT3)

             // CC3000 uses SPI1 on F3_34 Nucleo
#define CC3000_SPI                SPI1
#define CC3000_SPI_CLK            RCC_APB1Periph_SPI1

#define SPI_SCK_PIN               GPIO_PIN_5               /* PA.5  */
#define SPI_SCK_GPIO_PORT         GPIOA                    /* GPIOA */
#define SPI_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define SPI_SCK_SOURCE            GPIO_PinSource5
#define SPI_SCK_AF                GPIO_AF_5

#ifdef MERGE_THESE
#define SPI_MISO_PIN              GPIO_PIN_6               /* PA.6  */
#define SPI_MISO_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MISO_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MISO_SOURCE           GPIO_PinSource6
#define SPI_MISO_AF               GPIO_AF_5

#define SPI_MOSI_PIN              GPIO_PIN_7               /* PA.7  */
#define SPI_MOSI_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MOSI_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MOSI_SOURCE           GPIO_PinSource7
#define SPI_MOSI_AF               GPIO_AF_5
#endif

            // CC3000 Chip Select pin - PB.6  Arduino Hdr D-10
#define SPI_CS_CC3000_PIN         GPIO_PIN_6               /* PB.6  */
#define SPI_CS_CC3000_GPIO_PORT   GPIOB                    /* GPIOB */
#define SPI_CS_CC3000_GPIO_CLK    RCC_AHBPeriph_GPIOB

            // CC3000 VBAT ENABLE pin - PB.4  Arduino Hdr D-5
#ifdef MERGE_THESE
#define CC3000_ENABLE_PIN         GPIO_PIN_4               /* PB.4  */
#define CC3000_ENABLE_GPIO_PORT   GPIOB                    /* GPIOB */
#define CC3000_ENABLE_GPIO_CLK    RCC_AHBPeriph_GPIOB
#endif
            // CC3000 IRQ (Interrupt Req In) pin - PB.3  Arduino Hdr D-3
#define CC3000_IRQ_IN_PIN              GPIO_PIN_3          /* PB.3  */
#define CC3000_IRQ_IN_GPIO_PORT        GPIOB               /* GPIOB */
#define CC3000_IRQ_IN_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define CC3000_IRQ_IN_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define CC3000_IRQ_IN_EXTI_PIN_SOURCE  EXTI_PinSource3
#define CC3000_IRQ_IN_EXTI_LINE        EXTI_Line3
#define CC3000_IRQ_IN_EXTI_IRQn        EXTI3_IRQn

            //  CC3000 control lines (ENABLE, IRQ) management
// Setting the HIGH half causes a Bit Reset, setting the LOW half causes Bit Set
#define CC3000_ENABLE_HIGH()  CC3000_ENABLE_GPIO_PORT->BSRRL = CC3000_ENABLE_PIN
#define CC3000_ENABLE_LOW()   CC3000_ENABLE_GPIO_PORT->BSRRH = CC3000_ENABLE_PIN

            // Unit Testing Definitions for Pin Operation Checkout
#define SPI_CLK_HIGH()        SPI_SCK_GPIO_PORT->BSRRL  = SPI_SCK_PIN
#define SPI_CLK_LOW()         SPI_SCK_GPIO_PORT->BSRRH  = SPI_SCK_PIN
#define SPI_MISO_HIGH()       SPI_MISO_GPIO_PORT->BSRRL = SPI_MISO_PIN
#define SPI_MISO_LOW()        SPI_MISO_GPIO_PORT->BSRRH = SPI_MISO_PIN
#define SPI_MOSI_HIGH()       SPI_MOSI_GPIO_PORT->BSRRL = SPI_MOSI_PIN
#define SPI_MOSI_LOW()        SPI_MOSI_GPIO_PORT->BSRRH = SPI_MOSI_PIN
#define CC3000_IRQ_HIGH()     CC3000_IRQ_IN_GPIO_PORT->BSRRL = CC3000_IRQ_IN_PIN
#define CC3000_IRQ_LOW()      CC3000_IRQ_IN_GPIO_PORT->BSRRH = CC3000_IRQ_IN_PIN
#endif                            // if defined(USES_CC3000)

#define USER_BUTTON_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USER_BUTTON_EXTI_PORT_SOURCE  GPIO_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE   GPIO_PinSource0
#define USER_BUTTON_EXTI_LINE         EXTI_Line0
#define USER_BUTTON_EXTI_IRQn         EXTI0_IRQn

            //-----------------------------------------------------
            // DRV8711 GPIO Port and Pin Definitions - STM32 F4 01
            //-----------------------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT                   BIT2      // P4.2
#define  nSLEEP                BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET_PIN             BIT4      // P3.4
#define  STEP_AIN1             BIT5      // P3.5
#define  DIR_AIN2              BIT6      // P3.6
#define  BIN2                  BIT4      // P1.4
#define  BIN1                  BIT5      // P1.5
#define  nSTALL                BIT2      // P1.2
#define  nFAULT                BIT0      // P3.0

#define  POT_PORT              P4IN    // Note: Function selectors are SEL0/SEL1
#define  nSLEEP_PORT           P2OUT
#define  RESET_PORT            P3OUT
#define  STEP_AIN1_PORT        P3OUT
#define  DIR_AIN2_PORT         P3OUT
#define  BIN2_PORT             P1OUT
#define  BIN1_PORT             P1OUT
#define  nSTALL_PORT           P1IN
#define  nFAULT_PORT           P3IN

#define  NSLEEP_HI_ENABLE      nSLEEP_PORT |= nSLEEP
#define  NSLEEP_LO_DISABLE     nSLEEP_PORT &= ~nSLEEP

#define  RESET_HI_RESET        RESET_PORT  |= RESET_PIN
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET_PIN

#define  STEP_AIN1_HIGH        STEP_AIN1_PORT |= STEP_AIN1
#define  STEP_AIN1_LOW         STEP_AIN1_PORT &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      DIR_AIN2_PORT |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     DIR_AIN2_PORT &= ~DIR_AIN2

#define  BIN2_HI               BIN2_PORT |= BIN2
#define  BIN2_LO               BIN2_PORT &= ~BIN2

#define  BIN1_HI               BIN1_PORT |= BIN1
#define  BIN1_LO               BIN1_PORT &= ~BIN1

//#define  ASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)  // fails
//#define  DEASSERT_DRV8711_CS() (P1OUT |= BIT3)   // fails
#define  ASSERT_DRV8711_CS()     (P1OUT |= BIT3)   /* P 1.3 */
#define  DEASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)

            // DRV8711 related Defs
//#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz * 1000000)
#define  TOTAL_CCR_CLOCK_CYCLES   (MCU_CLOCK_SPEED)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                                //    defined(STM32F4x1xE)
                                      //----------------------------




#if defined(STM32F746xx) || defined(STM32F746NGHx)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - F7_46_NG   GPIO mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                      STM32   F7_46  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED           216000000     // Main Clock Frequency 216 MHz

#include "STM32_F7\stm32f7xx.h"
#include "STM32_F7\stm32f7xx_hal_conf.h"
#include "STM32_F7\system_stm32f7xx.h"
#include "STM32_F7\stm32f7xx_it.h"
#include "stm32f7xx_hal_spi.h"        // SYSWORKBENCH NEEDS THS

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
//#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"           // pull in basic MCU clock and SYSCFG defs

#if defined(USE_STM32746G_DISCOVERY)
      // uses IRQ for EXTI15_10_IRQn on pin PC_13 for push button

                 // NAMING CONFLICTS WITH OUR LED1 DEFINITION
                 //   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//#include "STM32F746G-Discovery\stm32746g_discovery.h"
#endif

#if defined(USE_STM32F7XX_NUCLEO) || defined(USES_NUCLEO)
  #include "STM32F7xx-Nucleo\stm32f7xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32f7xx_nucleo_bluenrg.h"     // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"       // pull in needed W5200 include files
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

            // LED port and pin Definitions
#define  LED1_PORT               GPIOI
#define  LED_1_PIN               GPIO_PIN_1    // out    PI1  aka  Arduino  D13
#define  LED1_INIT()             board_gpio_pin_config (9, LED_1_PIN, \
                                            GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOI->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOI->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOI->ODR ^= LED_1_PIN

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCK()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define  USART_PERIPH_CLOCK()    __HAL_RCC_USART1_CLK_ENABLE()
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_7      // in
#define  USART_TX_PIN_AF         GPIO_AF7_USART1
#define  USART_RX_PIN_AF         GPIO_AF7_USART1
#define  USART_IRQn              USART1_IRQn

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT    GPIOI
#define  KEY_BUTTON_PIN          GPIO_PIN_11
#define  KEY_BUTTON_EXTI_IRQn    EXTI15_10_IRQn      /* IRQ  = EXTI 11 */

#define  PB1_INIT()              board_gpio_pin_config (9, KEY_BUTTON_PIN, \
                                            GPIOINPUT, GPIO_NOPULL)

      //==============================================================
      //                 Discovery - F7_46 Platform    -    uses SPI2
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PI1      Digital # 13
      //   SPI MISO             PB14     Digital # 12
      //   SPI MOSI             PB15     Digital # 11
      //   SPI CS  for W5200    PB4      Digital #  3
      //   SPI CS  for SD Card  PG7      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PI0      Digital # 10
      //   VBAT_ENABLE CC3000   PA8      Digital #  5
      //   IRQ         CC3000   PB4      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

//#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
//#define  SPI_SCLK_PORT          GPIOI
//#define  SPI_MISO_PORT          GPIOB
//#define  SPI_MOSI_PORT          GPIOB
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

#define  SPI_1_A_SCLK_PIN_AF    5,GPIO_AF_SPI1       /* Alt Funtion defs */
#define  SPI_1_A_MISO_PIN_AF    6,GPIO_AF_SPI1
#define  SPI_1_A_MOSI_PIN_AF    7,GPIO_AF_SPI1

//#define  SPI_ID_2_I             2            // SPI 2 using pins PI1/PB14/PB14 - Default for "Arduino Headers"
#define  SPI_2_I_SCLK_PIN       GPIO_PIN_1   //   PI1  - SCLK
#define  SPI_2_I_MISO_PIN       GPIO_PIN_14  //   PB14 - MOSI
#define  SPI_2_I_MOSI_PIN       GPIO_PIN_15  //   PB15 - MISO
#define  SPI_2_I_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_I_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_3_C             3           // SPI 3 using pins PC10/PC11/PC12
#define  SPI_3_C_SCLK_PIN       GPIO_PIN_10  //   PC10 - SCLK
#define  SPI_3_C_MISO_PIN       GPIO_PIN_11  //   PC11 - MOSI
#define  SPI_3_C_MOSI_PIN       GPIO_PIN_12  //   PC12 - MISO
#define  SPI_3_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_3_C_MOSx_PULL_MODE GPIO_PULLUP

#if defined(USES_L6474)
            //--------------------------------------------------------
            // ST's L6474 Stepper/BDC Motor Ctl Shield
            //                     GPIO Port and Pin Defs  F7 46
            //                     using SPI 2      (PI1/PB14/PB14)
            //--------------------------------------------------------
            //----------------------------------------------------
            // L6474 Stepper SPI CS GPIO Port and Pin Defs  F7 46
            //----------------------------------------------------
#define  L6474_SPI_PORT_ID    SPI_ID_2_I
#define  L6474_SPI_MODE       SPI_MODE_3
#define  L6474_SPI_CS_PORT    GPIOI
#define  L6474_SPI_CS_PIN     GPIO_PIN_0     // out PI0   D10 (Ardu)
#endif                                       // USES_L6474_STEPPER

#if defined(USES_BLUENRG_BLE)
            //--------------------------------------------------------
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 2      (PI1/PB14/PB14)
//                                                ^^^^^^^^
//                                              Need to jumper because
//                                              BLE clock is wired to D3 / PB4 !
            //--------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   A1  PF_10    F7 46
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_2_I
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             6               // GPIOF
#define  BlueNRG_CS_PIN              GPIO_PIN_10     // out PF_10    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //---------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs  A0  PA_0   F7 46
            //---------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //---------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port/Pin Defs    D7  PI_3   F7 46
            //---------------------------------------------------------
#define  BlueNRG_RESET_PORT          9               // GPIOI
#define  BlueNRG_RESET_PIN           GPIO_PIN_3      // out PI_3   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE


#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //--------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F7 46
            //                     using SPI 2      (PI1/PB14/PB14)
            //                  default CS on D10   PI0
//                                               ^^^^^^^^^^^^^^^^^
//                                               Need to use Kludge Interloper
//                                               to re-route CS is using other boards
            //--------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_2_I
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        8               // GPIOI
#define  W5200_CS_ALTERNATE_PORT     6               // GPIOG
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_0      // out PI0      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_6      // out PG6      D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOI->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOI->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOG->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOG->BSRR = W5200_CS_ALTERNATE_PIN

#define  WIZ_SCLK               SPI_2_I_SCLK_PIN       // out PI1      D13
#define  WIZ_MISO               SPI_2_I_MISO_PIN       // in  PB14     D12
#define  WIZ_MOSI               SPI_2_I_MOSI_PIN       // out PB15     D11
#if defined(USES_L6474)
#define  WIZ_SCS                GPIO_PIN_6             // out PG6      D2  Alt
#else
#define  WIZ_SCS                GPIO_PIN_0             // out PI0      D10 Norm
#endif
//#define  WIZ_INT              GPIO_PIN_0             // in  PB0 ???
//#define  WIZ_RESET            GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
//#define  WIZ_PWDN             GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
//#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
//#define  W5200_INT_PORT       GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
//#define  W5200_INT_PIN        GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                                  // if USES_W5200

#endif                                  //    defined(STM32F746xx)
                                        //----------------------------





#if defined(__STM32L053__) || defined(STM32L053xx)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - L0_53_R8   Pin mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                       STM32   L0_53   Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED       32000000            // Max Clock Frequency 32 MHz

#include "stm32l0xx_hal.h"            // pull in basic MCU clock and SYSCFG defs
//#include "stm32l053b_discovery.h"

#include "STM32_L0\stm32l0xx.h"
#include "STM32_L0\system_stm32l0xx.h"
#include "STM32_L0\stm32l0xx_it.h"

#if defined(USE_STM32L0XX_NUCLEO)
      // uses IRQ for EXTI4_15_IRQn on pin PC_13 for push button
  #include "stm32l0xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_1_IRQn on pin PA_0          (EXTI0_1_IRQHandler ISR)
  #include "stm32l0xx_nucleo_bluenrg.h"      // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

            // LED port and pin Definitions
#define  LED1_PORT               GPIOA
#define  LED_3_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
//#define  LED_4_PIN             GPIO_PIN_0    // out    PA0  Only on Discoverys
#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN;

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT     GPIOC
#define  KEY_BUTTON_PIN           GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
//#define  KEY_BUTTON_EXTI_IRQn     EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - L0_53 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_1_C             21           // SPI 1 using pins PA5/PA11/PA12
#define  SPI_1_C_SCLK_PIN       GPIO_PIN_5   //   PA5  - SCLK
#define  SPI_1_C_MISO_PIN       GPIO_PIN_11  //   PA11 - MOSI
#define  SPI_1_C_MOSI_PIN       GPIO_PIN_12  //   PA12 - MISO
#define  SPI_1_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_C_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_D             31           // SPI 1 using pins PB3/PB4/PB5
#define  SPI_1_D_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_D_MISO_PIN       GPIO_PIN_4   //   PB4 - MOSI
#define  SPI_1_D_MOSI_PIN       GPIO_PIN_5   //   PB5 - MISO
#define  SPI_1_D_SCLK_PULL_MODE GPIO_PULLDOWN
#define  SPI_1_D_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_2_B             2           // SPI 2 using pins PB13/PB14/PB15
#define  SPI_2_B_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_B_MISO_PIN       GPIO_PIN_14  //   PB14 - MOSI
#define  SPI_2_B_MOSI_PIN       GPIO_PIN_15  //   PB15 - MISO
#define  SPI_2_B_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_B_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_2_C             12           // SPI 2 using pins PB10/PC2/PC3
#define  SPI_2_C_SCLK_PIN       GPIO_PIN_10  //   PB10 - SCLK
#define  SPI_2_C_MISO_PIN       GPIO_PIN_2   //   PC2  - MOSI
#define  SPI_2_C_MOSI_PIN       GPIO_PIN_3   //   PC3  - MISO
#define  SPI_2_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_C_MOSx_PULL_MODE GPIO_PULLUP

     // There is no SPI3 on L0 53

#if defined(USES_BLUENRG_BLE)
            //-----------------------------------------------------------
            //
            //        --- These use the identical settings as F4 01 ---
            //
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  L0 53
            //                     using SPI 1      (PB3/PA6/PA7)
            //-----------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    L0 53
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //--------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs   PA_0   L0 53
            //--------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_1_IRQn      /* Uses EXT 0 rupt  F0_72 DIFF  */
//#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt  F4_01 DIFF  */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //----------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs   PA_8   L0 53
            //----------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE

#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //-----------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  L0 53
            //                     using SPI 2      (PB13/PB14/PB15)
            // -- F0_72 WIRING IS DIFFERNT THAN F4_02 which uses UPPER LEFT CORNER
            //    PC10/PC11/PC12 and uses SPI 3 ---
            // -- F0_72 USES THE LOWER RIGHT CORNER (PB13/PB14/PB15) on MORPHO
            //    and uses SPI 2 --
            //-----------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_2_B
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

// 06/02/15  --- APPARENTLY THESE ARE NO LONG USED ---
#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#endif                                //   defined(__STM32L053__)
                                      //---------------------------



#if defined(STM32L152xE) || defined(STM32L152xC)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                  STM32 - L1_52_RE/RC   Pin mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           // STM32 L1 Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED          32000000         // Max Clock Frequency 32 MHz

#include "stm32l1xx_hal.h"            // pull in basic MCU clock and SYSCFG defs
//#include "stm32l152b_discovery.h"

#include "STM32_L1\stm32l1xx.h"
#include "STM32_L1\system_stm32l1xx.h"
#include "STM32_L1\stm32l1xx_it.h"

#if defined(USE_STM32L1XX_NUCLEO)
      // uses IRQ for EXTI15_10_IRQn on pin PC_15 for push button
  #include "stm32l1xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32l1xx_nucleo_bluenrg.h"      // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED      //   and  ~/STM32_Code\X_NUCLEO_BLUENRG_BLE\includes/stm32_bluenrg_ble.h
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

            // LED port and pin Definitions
#define  LED1_PORT               GPIOA
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13
//#define  LED_4_PIN             GPIO_PIN_0    // out    PA0  Only on Discoverys
#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN;

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT    GPIOC
#define  KEY_BUTTON_PIN          GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
#define  KEY_BUTTON_EXTI_IRQn    EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - L1_52 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_1_C             21           // SPI 1 using pins PA5/PA11/PA12
#define  SPI_1_C_SCLK_PIN       GPIO_PIN_5   //   PA5  - SCLK
#define  SPI_1_C_MISO_PIN       GPIO_PIN_11  //   PA11 - MISO
#define  SPI_1_C_MOSI_PIN       GPIO_PIN_12  //   PA12 - MOSI
#define  SPI_1_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_C_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_D             31           // SPI 1 using pins PB3/PB4/PB5
#define  SPI_1_D_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_D_MISO_PIN       GPIO_PIN_4   //   PB4 - MISO
#define  SPI_1_D_MOSI_PIN       GPIO_PIN_5   //   PB5 - MOSI
#define  SPI_1_D_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_D_MOSx_PULL_MODE GPIO_NOPULL

//#define  SPI_ID_2_B             2           // SPI 2 using pins PB13/PB14/PB15
#define  SPI_2_B_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_B_MISO_PIN       GPIO_PIN_14  //   PB14 - MISO
#define  SPI_2_B_MOSI_PIN       GPIO_PIN_15  //   PB15 - MOSI
#define  SPI_2_B_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_B_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_2_C             12           // SPI 2 using pins PB13/PC2/PC3
#define  SPI_2_C_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_C_MISO_PIN       GPIO_PIN_2   //   PC2  - MISO
#define  SPI_2_C_MOSI_PIN       GPIO_PIN_3   //   PC3  - MOSI
#define  SPI_2_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_C_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_3_B             3            // SPI 3 using pins PB3/PB4/PB5
#define  SPI_3_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_3_B_MISO_PIN       GPIO_PIN_4   //   PB4 - MISO
#define  SPI_3_B_MOSI_PIN       GPIO_PIN_5   //   PB5 - MOSI
#define  SPI_3_B_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_3_B_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_3_C             13          // SPI 3 using pins PC10/PC11/PC12
#define  SPI_3_C_SCLK_PIN       GPIO_PIN_10  //   PC10 - SCLK
#define  SPI_3_C_MISO_PIN       GPIO_PIN_11  //   PC11 - MISO
#define  SPI_3_C_MOSI_PIN       GPIO_PIN_12  //   PC12 - MOSI
#define  SPI_3_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_3_C_MOSx_PULL_MODE GPIO_NOPULL


#if defined(USES_BLUENRG_BLE)
            //-----------------------------------------------------------
            //
            //        --- These use the identical settings as F4 01 ---
            //
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F0 72
            //                     using SPI 1      (PB3/PA6/PA7)
            //-----------------------------------------------------------
            //----------------------------------------------------------
            // Blue_NRG SPI  CS  GPIO Port and Pin Defs   PA_1    F0 72
            //----------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //----------------------------------------------------------
            // Blue_NRG BLE  IRQ  GPIO Port and Pin Defs   PA_0   F0 72
            //----------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //------------------------------------------------------------
            // Blue_NRG BLE  RESET  GPIO Port and Pin Defs   PA_8   F0 72
            //------------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE

#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //-----------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  L1_52
            //                     using SPI 3      (PC10/PC11/PC12)
            // -- L1_52 WIRING IS SAME AS F4_02 which uses UPPER LEFT CORNER
            //    PC10/PC11/PC12 and uses SPI 3 ---
            //-----------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_3_C
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

// 06/02/15  --- APPARENTLY THESE ARE NO LONG USED ---
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions - PB0
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: in Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#endif                                //   defined(STM32L152xE/STM32L152xC)
                                      //----------------------------




#if defined(STM32L476xx)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// CAUTION: WAS A BLIND COPTY OF STM32 F4 section - need to tweak for L4 Discovery
//
//                  STM32 - L4_76_VG   GPIO mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
           //                      STM32   L4_76  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED           40000000    // Main Clock Frequency 40 MHz

#include "STM32_L4\stm32l4xx.h"
#include "STM32_L4\system_stm32l4xx.h"
#include "STM32_L4\stm32l4xx_it.h"

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal.h"           // pull in basic MCU clock and SYSCFG defs

#if defined(USE_STM32L4XX_NUCLEO) || defined(USES_NUCLEO)
      // uses IRQ for EXTI15_10_IRQn on pin PC_13 for push button
  #include "stm32l4xx_nucleo.h"
#endif

#if defined(USES_BLUENRG_BLE)
      // uses IRQ for EXTI0_IRQn on pin PA_0              (EXTI0_IRQHandler ISR)
  #include "stm32l4xx_nucleo_bluenrg.h"     // see ~/ble_drivers/STM32_BlueNRG
  #define SYSCLK_FREQ   MCU_CLOCK_SPEED
#endif

#if defined(USES_W5200)
#include "W5200_Lib\include\w5200.h"       // pull in needed W5200 include files
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#endif

#if defined(USE_STM32L476G_DISCO_REVB)
            // L4 Discovery LEDs are PB2 and PE8
#define  LED1_PORT               GPIOB
#define  LED_1_PIN               GPIO_PIN_2    // out    PA5  aka  Arduino  D13

#define  LED1_INIT()             board_gpio_pin_config (2, LED_1_PIN, \
                                                       GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOB->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOB->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOB->ODR ^= LED_1_PIN
#else
            // NUCLEO LED port and pin Definitions - LED1 is PA5 = Arduino  D13
#define  LED1_PORT               GPIOA
#define  LED_1_PIN               GPIO_PIN_5    // out    PA5  aka  Arduino  D13

#define  LED1_INIT()             board_gpio_pin_config (1, LED_1_PIN, \
                                 GPIO_OUTPUT, GPIO_NOPULL)
#define  LED1_ON()               GPIOA->BSRR = LED_1_PIN
#define  LED1_OFF()              GPIOA->BSRR = ((uint32_t) LED_1_PIN << 16)
#define  LED1_TOGGLE()           GPIOA->ODR ^= LED_1_PIN
#endif

            // USART port and TX/RX pin Definitions
#define  USART_GPIOs_CLOCKS_BIT  RCC_AHBPeriph_GPIOA
#define  USART_PERIPH_CLOCKS_BIT RCC_APB2Periph_USART1
#define  USART_MODULE            USART1
#define  USART_PORT              GPIOA
#define  USART_TX_PIN            GPIO_PIN_9      // out
#define  USART_RX_PIN            GPIO_PIN_10     // in
#define  USART_TX_PIN_AF         1x,GPIO_AF_x    // TBD
#define  USART_RX_PIN_AF         1x,GPIO_AF_5

             // User Push Button Defs
#define  KEY_BUTTON_GPIO_PORT     GPIOC
#define  KEY_BUTTON_PIN           GPIO_PIN_13     /* PC13 = Morpho CN7-23 */
#define  KEY_BUTTON_EXTI_IRQn     EXTI15_10_IRQn  /* IRQ  = EXTI 13 */

      //==============================================================
      //                   Nucleo - F4_01 Platform    -    uses SPI1
      //
      //   Function          STM32 pin   Arduino Pin
      //   ----------        ---------   ------------
      //   SPI SCLK             PA5      Digital # 13
      //   SPI MISO             PA6      Digital # 12
      //   SPI MOSI             PA7      Digital # 11
      //   SPI CS  for W5200    PB3      Digital #  3
      //   SPI CS  for SD Card  PB5      Digital #  4   (Optional)
      //   SPI CS  for CC3000   PB6      Digital # 10
      //   VBAT_ENABLE CC3000   PB4      Digital #  5
      //   IRQ         CC3000   PB3      Digital #  3
      //==============================================================
      // SPI General Defines
#define  DMA_WINDOW_SIZE         1024
#define  SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define  FWT_DELAY               4000
#define  SPI_TIMEOUT_MAX          100     // worst case

      // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
      // Adafruit LCD uses 1.6 MHz
#define  SPI_BAUDRATE_50_MHZ     SPI_BAUDRATEPRESCALER_2  // approx 50 MHz
#define  SPI_BAUDRATE_25_MHZ     SPI_BAUDRATEPRESCALER_4  // approx 25 MHz ST logic ! for Blue NRG
#define  SPI_BAUDRATE_6_MHZ      SPI_BAUDRATEPRESCALER_16 // approx 6.4 MHz
#define  SPI_BAUDRATE_3_MHZ      SPI_BAUDRATEPRESCALER_32 // approx 3.2 MHz
#define  SPI_BAUDRATE_1_MHZ      SPI_BAUDRATEPRESCALER_64 // approx 1.6 MHz

            // SPI port and pin Definitions
#define  SPI_PERIPH_CLOCKS_BIT   RCC_APB2Periph_SPI1
#define  SPI_GPIOs_CLOCKS_BIT    RCC_AHBPeriph_GPIOA
#define  SPI_CS_CLOCKS_BIT       RCC_AHBPeriph_GPIOB
#define  SPI_MODULE              SPI1

#define  SPI_GPIOs_PORT         GPIOA //Generic def used when all pins on same port
#define  SPI_SCLK_PORT          GPIOA
#define  SPI_MISO_PORT          GPIOA
#define  SPI_MOSI_PORT          GPIOA
#define  SPI_WIRED_CS_PORT      GPIOB
#define  SPI_WIFI_CS_PORT       GPIOB

//#define  SPI_ID_1_A             1            // SPI 1 using pins PA5/PA6/PA7
#define  SPI_1_A_SCLK_PIN       GPIO_PIN_5   //   PA5 - SCLK
#define  SPI_1_A_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_A_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_A_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_1_A_MOSx_PULL_MODE GPIO_PULLUP

//#define  SPI_ID_1_B             11           // SPI 1 using pins PB3/PA6/PA7
#define  SPI_1_B_SCLK_PIN       GPIO_PIN_3   //   PB3 - SCLK
#define  SPI_1_B_MISO_PIN       GPIO_PIN_6   //   PA6 - MOSI
#define  SPI_1_B_MOSI_PIN       GPIO_PIN_7   //   PA7 - MISO
#define  SPI_1_B_SCLK_PULL_MODE GPIO_PULLDOWN   // mainly for Blue NRG
#define  SPI_1_B_MOSx_PULL_MODE GPIO_NOPULL

#define  SPI_1_A_SCLK_PIN_AF    5,GPIO_AF_SPI1       /* Alt Funtion defs */
#define  SPI_1_A_MISO_PIN_AF    6,GPIO_AF_SPI1
#define  SPI_1_A_MOSI_PIN_AF    7,GPIO_AF_SPI1

//#define  SPI_ID_2_C             2            // SPI 2 using pins PB13/PC2/PC3
#define  SPI_2_C_SCLK_PIN       GPIO_PIN_13  //   PB13 - SCLK
#define  SPI_2_C_MISO_PIN       GPIO_PIN_2   //   PC2  - MOSI
#define  SPI_2_C_MOSI_PIN       GPIO_PIN_3   //   PC3  - MISO
#define  SPI_2_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_2_C_MOSx_PULL_MODE GPIO_PULLUP


//#define  SPI_ID_3_C             3           // SPI 3 using pins PC10/PC11/PC12
#define  SPI_3_C_SCLK_PIN       GPIO_PIN_10  //   PC10 - SCLK
#define  SPI_3_C_MISO_PIN       GPIO_PIN_11  //   PC11 - MOSI
#define  SPI_3_C_MOSI_PIN       GPIO_PIN_12  //   PC12 - MISO
#define  SPI_3_C_SCLK_PULL_MODE GPIO_PULLUP
#define  SPI_3_C_MOSx_PULL_MODE GPIO_PULLUP

#if defined(USES_L6474)
            //--------------------------------------------------------
            // ST's L6474 Stepper/BDC Motro Ctl Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 1      (PA5/PA6/PA7)
            //--------------------------------------------------------
            //----------------------------------------------------
            // L6474 Stepper SPI CS GPIO Port and Pin Defs  F4 01
            //----------------------------------------------------
#define  L6474_SPI_PORT_ID    SPI_ID_1_A
#define  L6474_SPI_MODE       SPI_MODE_3
#define  L6474_SPI_CS_PORT    GPIOB
#define  L6474_SPI_CS_PIN     GPIO_PIN_6             // out PB6   D10 (Ardu)
#endif                             // USES_L6474_STEPPER

#if defined(USES_BLUENRG_BLE)
            //--------------------------------------------------------
            // ST's Blue NRG BLE Bluetooh Low Energy Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 1      (PB3/PA6/PA7)
            //--------------------------------------------------------
            //--------------------------------------------------------
            // Blue_NRG SPI CS GPIO Port and Pin Defs   PA_1    F4 01
            //--------------------------------------------------------
#define  BlueNRG_BLE_SPI_PORT_ID     SPI_ID_1_B
#define  BlueNRG_BLE_SPI_MODE        SPI_MODE_0
#define  BlueNRG_CS_PORT             1               // GPIOA
#define  BlueNRG_CS_PIN              GPIO_PIN_1      // out PA_1    A1 (Ardu)
#define  ASSERT_CS_BlueNRG()         GPIOA->BSRR = ((uint32_t) BlueNRG_CS_PIN << 16)
#define  DEASSERT_CS_BlueNRG()       GPIOA->BSRR = BlueNRG_CS_PIN

            //---------------------------------------------------------
            // Blue_NRG BLE IRQ GPIO Port and Pin Defs    PA_0   F4 01
            //---------------------------------------------------------
#define  BlueNRG_IRQ_PORT            1               /* GPIOA  PA_0 */
#define  BlueNRG_IRQ_PIN             GPIO_PIN_0      /* IRQ = PA_0  A0 (Ardu) */
#define  BlueNRG_IRQ_RISING_MODE     GPIO_RUPT_MODE_RISING
#define  BlueNRG_IRQ_PULLUP_SETTING  GPIO_NOPULL
#define  BlueNRG_EXTI_IRQn           EXTI0_IRQn      /* Uses EXT 0 rupt */
#define  BlueNRG_EXTI_IRQ_Handler    EXTI0_IRQHandler

            //---------------------------------------------------------
            // Blue_NRG BLE RESET GPIO Port and Pin Defs  PA_8   F4 01
            //---------------------------------------------------------
#define  BlueNRG_RESET_PORT          1               // GPIOA
#define  BlueNRG_RESET_PIN           GPIO_PIN_8      // out PA_8   D7 (Ardu)
#define  ASSERT_BlueNRG_RESET()      GPIOA->BSRR = ((uint32_t) BlueNRG_RESET_PIN << 16)
#define  DEASSERT_BlueNRG_RESET()    GPIOA->BSRR = BlueNRG_RESET_PIN
#endif                        // USES_BLUENRG_BLE


#if defined(USES_W5200) || defined(SEEED_STUDIO_W5200)
            //--------------------------------------------------------
            // Seeed Studio W5200 wired Ethernet Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 3      (PC10/PC11/PC12)
            //--------------------------------------------------------
#define  W5200_SPI_PORT_ID           SPI_ID_3_C
#define  W5200_SPI_MODE              SPI_MODE_0

#define  W5200_CS_NORMAL_PORT        1               // GPIOB
#define  W5200_CS_ALTERNATE_PORT     0               // GPIOA
                                                     // Morpho     Arduino
#define  W5200_CS_NORMAL_PIN         GPIO_PIN_6      // out PB6      D10
#define  W5200_CS_ALTERNATE_PIN      GPIO_PIN_3      // out PA15     D2

#define  ASSERT_CS_W5200_NORMAL()      GPIOB->BSRR = ((uint32_t) W5200_CS_NORMAL_PIN << 16)
#define  DEASSERT_CS_W5200_NORMAL()    GPIOB->BSRR = W5200_CS_NORMAL_PIN
#define  ASSERT_CS_W5200_ALTERNATE()   GPIOA->BSRR = ((uint32_t) W5200_CS_ALTERNATE_PIN << 16)
#define  DEASSERT_CS_W5200_ALTERNATE() GPIOA->BSRR = W5200_CS_ALTERNATE_PIN

#define  WIZ_SCLK            SPI_SCLK_PIN              // out PA5      D13
#define  WIZ_MISO            SPI_MISO_PIN              // in  PA6      D12
#define  WIZ_MOSI            SPI_MOSI_PIN              // out PA7      D11
#if defined(USES_L6474)
#define  WIZ_SCS             GPIO_PIN_3                // out PB3      D3  Alt
#else
#define  WIZ_SCS             GPIO_PIN_6                // out PB6      D10 Norm
#endif
#define  WIZ_INT             GPIO_PIN_0                // in  PB0 ???
#define  WIZ_RESET           GPIO_PIN_8             // out PB8 Tied to +3.3 Seed
#define  WIZ_PWDN            GPIO_PIN_9             // out PB9 Tied to Gnd  Seed

            // W5200 INT port and pin Definitions
#define  W5200_EXTI_PERIPH_CLOCKS_BIT  RCC_APB2Periph_SYSCFG
#define  W5200_INT_PORT      GPIOB       // PB0 Interrupt Pin W5200 Wired Ethernet
#define  W5200_INT_PIN       GPIO_PIN_0

            // W5200 RESET and PWRDN port and pin Definitions
            // Note: if Seeed Studio version, these pins are not used.
            // Seeed version permanently ties PWDN to Gnd,  and RESET to +3.3V
#else
#define  W5200_PWDN_PORT     GPIOB       // PB8 PWDN  Pin W5200 Wired Ethernet
#define  W5200_PWDN_PIN      GPIO_PIN_8
#define  W5200_RESET_PORT    GPIOB       // PB0 RESET Pin W5200 Wired Ethernet
#define  W5200_RESET_PIN     GPIO_PIN_9
#endif                          // if USES_W5200

#if defined(USES_CC3000)
            //-------------------------------------------------------
            // Adafruit's TI CC3000 WiFi Shield
            //                     GPIO Port and Pin Defs  F4 01
            //                     using SPI 3      (PC10/PC11/PC12)
            //-------------------------------------------------------
#define  CC3000_INT_PORT     GPIOB       // Interrupt Pin CC3000 WiFi Ethernet
#define  CC3000_INT_PIN      GPIO_PIN_x

#define  CC3000_ENABLE_PORT  GPIOB       // ENABLE Pin CC3000 WiFi Ethernet
#define  CC3000_ENABLE_PIN   GPIO_PIN_x

            //----------------------------------------
            // CC3000 CS GPIO Port and Pin Definitions
            //----------------------------------------
#define ASSERT_WIFI_CS()     (P3OUT &= ~BIT0)     /* P 3.0 */
#define DEASSERT_WIFI_CS()   (P3OUT |= BIT0)

            //----------------------------------------
            // CC3000 GPIO Port and Pin Definitions
            //----------------------------------------
#define  CC3000_CS                BIT3      // P 1.3  SPI CS
#define  CC3000_ENABLE            BIT2      // P 1.2  aka nHIB   may be P 4.1 instead !
#define  CC3000_IRQ               BIT3      // P 2.3

#define  ASSERT_CC3000_CS()       (P1OUT &= ~BIT3)     /* P 1.3 */
#define  DEASSERT_CC3000_CS()     (P1OUT |= BIT3)

             // CC3000 uses SPI1 on F3_34 Nucleo
#define CC3000_SPI                SPI1
#define CC3000_SPI_CLK            RCC_APB1Periph_SPI1

#define SPI_SCK_PIN               GPIO_PIN_5               /* PA.5  */
#define SPI_SCK_GPIO_PORT         GPIOA                    /* GPIOA */
#define SPI_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define SPI_SCK_SOURCE            GPIO_PinSource5
#define SPI_SCK_AF                GPIO_AF_5

#ifdef MERGE_THESE
#define SPI_MISO_PIN              GPIO_PIN_6               /* PA.6  */
#define SPI_MISO_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MISO_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MISO_SOURCE           GPIO_PinSource6
#define SPI_MISO_AF               GPIO_AF_5

#define SPI_MOSI_PIN              GPIO_PIN_7               /* PA.7  */
#define SPI_MOSI_GPIO_PORT        GPIOA                    /* GPIOA */
#define SPI_MOSI_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_MOSI_SOURCE           GPIO_PinSource7
#define SPI_MOSI_AF               GPIO_AF_5
#endif

            // CC3000 Chip Select pin - PB.6  Arduino Hdr D-10
#define SPI_CS_CC3000_PIN         GPIO_PIN_6               /* PB.6  */
#define SPI_CS_CC3000_GPIO_PORT   GPIOB                    /* GPIOB */
#define SPI_CS_CC3000_GPIO_CLK    RCC_AHBPeriph_GPIOB

            // CC3000 VBAT ENABLE pin - PB.4  Arduino Hdr D-5
#ifdef MERGE_THESE
#define CC3000_ENABLE_PIN         GPIO_PIN_4               /* PB.4  */
#define CC3000_ENABLE_GPIO_PORT   GPIOB                    /* GPIOB */
#define CC3000_ENABLE_GPIO_CLK    RCC_AHBPeriph_GPIOB
#endif
            // CC3000 IRQ (Interrupt Req In) pin - PB.3  Arduino Hdr D-3
#define CC3000_IRQ_IN_PIN              GPIO_PIN_3          /* PB.3  */
#define CC3000_IRQ_IN_GPIO_PORT        GPIOB               /* GPIOB */
#define CC3000_IRQ_IN_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define CC3000_IRQ_IN_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define CC3000_IRQ_IN_EXTI_PIN_SOURCE  EXTI_PinSource3
#define CC3000_IRQ_IN_EXTI_LINE        EXTI_Line3
#define CC3000_IRQ_IN_EXTI_IRQn        EXTI3_IRQn

            //  CC3000 control lines (ENABLE, IRQ) management
// Setting the HIGH half causes a Bit Reset, setting the LOW half causes Bit Set
#define CC3000_ENABLE_HIGH()  CC3000_ENABLE_GPIO_PORT->BSRRL = CC3000_ENABLE_PIN
#define CC3000_ENABLE_LOW()   CC3000_ENABLE_GPIO_PORT->BSRRH = CC3000_ENABLE_PIN

            // Unit Testing Definitions for Pin Operation Checkout
#define SPI_CLK_HIGH()        SPI_SCK_GPIO_PORT->BSRRL  = SPI_SCK_PIN
#define SPI_CLK_LOW()         SPI_SCK_GPIO_PORT->BSRRH  = SPI_SCK_PIN
#define SPI_MISO_HIGH()       SPI_MISO_GPIO_PORT->BSRRL = SPI_MISO_PIN
#define SPI_MISO_LOW()        SPI_MISO_GPIO_PORT->BSRRH = SPI_MISO_PIN
#define SPI_MOSI_HIGH()       SPI_MOSI_GPIO_PORT->BSRRL = SPI_MOSI_PIN
#define SPI_MOSI_LOW()        SPI_MOSI_GPIO_PORT->BSRRH = SPI_MOSI_PIN
#define CC3000_IRQ_HIGH()     CC3000_IRQ_IN_GPIO_PORT->BSRRL = CC3000_IRQ_IN_PIN
#define CC3000_IRQ_LOW()      CC3000_IRQ_IN_GPIO_PORT->BSRRH = CC3000_IRQ_IN_PIN
#endif                            // if defined(USES_CC3000)

#define USER_BUTTON_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USER_BUTTON_EXTI_PORT_SOURCE  GPIO_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE   GPIO_PinSource0
#define USER_BUTTON_EXTI_LINE         EXTI_Line0
#define USER_BUTTON_EXTI_IRQn         EXTI0_IRQn

            //-----------------------------------------------------
            // DRV8711 GPIO Port and Pin Definitions - STM32 F4 01
            //-----------------------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT                   BIT2      // P4.2
#define  nSLEEP                BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET_PIN             BIT4      // P3.4
#define  STEP_AIN1             BIT5      // P3.5
#define  DIR_AIN2              BIT6      // P3.6
#define  BIN2                  BIT4      // P1.4
#define  BIN1                  BIT5      // P1.5
#define  nSTALL                BIT2      // P1.2
#define  nFAULT                BIT0      // P3.0

#define  POT_PORT              P4IN    // Note: Function selectors are SEL0/SEL1
#define  nSLEEP_PORT           P2OUT
#define  RESET_PORT            P3OUT
#define  STEP_AIN1_PORT        P3OUT
#define  DIR_AIN2_PORT         P3OUT
#define  BIN2_PORT             P1OUT
#define  BIN1_PORT             P1OUT
#define  nSTALL_PORT           P1IN
#define  nFAULT_PORT           P3IN

#define  NSLEEP_HI_ENABLE      nSLEEP_PORT |= nSLEEP
#define  NSLEEP_LO_DISABLE     nSLEEP_PORT &= ~nSLEEP

#define  RESET_HI_RESET        RESET_PORT  |= RESET_PIN
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET_PIN

#define  STEP_AIN1_HIGH        STEP_AIN1_PORT |= STEP_AIN1
#define  STEP_AIN1_LOW         STEP_AIN1_PORT &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      DIR_AIN2_PORT |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     DIR_AIN2_PORT &= ~DIR_AIN2

#define  BIN2_HI               BIN2_PORT |= BIN2
#define  BIN2_LO               BIN2_PORT &= ~BIN2

#define  BIN1_HI               BIN1_PORT |= BIN1
#define  BIN1_LO               BIN1_PORT &= ~BIN1

//#define  ASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)  // fails
//#define  DEASSERT_DRV8711_CS() (P1OUT |= BIT3)   // fails
#define  ASSERT_DRV8711_CS()     (P1OUT |= BIT3)   /* P 1.3 */
#define  DEASSERT_DRV8711_CS()   (P1OUT &= ~BIT3)

            // DRV8711 related Defs
//#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz * 1000000)
#define  TOTAL_CCR_CLOCK_CYCLES   (MCU_CLOCK_SPEED)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                                //    defined(STM32L476xx)
                                      //----------------------------



#if defined(YOUR_CUSTOM_STM32_BOARD)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  STM32 - xxxx   GPIO / SPI mappings
//
//
// Your custom board
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

     // enter associated GPIO settings for your board here

#endif                               // defined(YOUR_CUSTOM_STM32_BOARD)
                                     //---------------------------------



#if defined(USES_W5200)
#include "W5200_Lib\include\socket.h"
#include "W5200_Lib\include\spi2.h"
#include "W5200_Lib\include\w5200.h"       // Pull in W5200 Wired Ethernet defs
#include "W5200_Lib\include\md5.h"
#endif

#endif                        // __BOARD_STM32_H__

