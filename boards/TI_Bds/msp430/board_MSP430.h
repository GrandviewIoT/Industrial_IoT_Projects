
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_MSP430.h
//
//
// See board.h for "common functions". It includes this file, based on MCU type
//
//
// Board dependent code, moved to a single module, in prep for Tiva, C2000.
//
// APIs for common routines implemented for each board.
//
// History:
//   11/29/14 - Created.
//   12/01/14 - Moved MCU-specific constants (MSP430 BIT0/BIT1/... vs Tiva
//              GPIO_PIN_0/GPIO_PIN_1/...) from DRV8711_Spin_Routines.h to here.
//   12/01/14 - Replace boolean enum with stdbool.h's bool instead, for better
//              portability (Tiva compiler barfs on that enum). Duquaine
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


#ifndef  __BOARD_MSP430_H__
#define  __BOARD_MSP430_H__

//#include "msp430/spi.h"     // causes heartbrun on F5529 spi.c and board.c compiles 04/11/15

#ifndef FALSE
#define  FALSE  0
#define  TRUE   1
#endif

                    //-------------------------------------------
                    //  MSP430  I/O Pin definitions by processor
                    //-------------------------------------------

#if defined(__MSP430F5529__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//
//                      MSP430 - F5529          GPIO mappings
//
//
// Sets pin/bit definitions for SPI/nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1        = P1.0                LED2        = P4.7
//    PushButton1 = P2.1                PushButton2 = P1.1
//
//    Timers:     =  TA0 (5 ccr)     TA1 (4 ccr)    TA2 (3 ccr)
//                   TB0 (7 ccr)
//
//    UART (USB) UCA1 = P4.4 (TX) / P4.5 (RX)
//    UART Sep   UCA0 = P3.4 (TX) / P3.3 (RX)        (J1-3/J1-4)
//    I2C         = EUSCI_B0 = P3.0/P3.1             (J2-6/J2-7)
//    SPI         = EUSCI_B0 = P3.2/P3.0/P3.1   (J1-7/J2-6/J2-7)
//    I2C         = EUSCI_B1 = P4.2/P4.1             (J1-9/J1-10)
//    SPI         = EUSCI_B1 = P4.3/P4.2/P4.1   (J4-7/J1-9/J1-10)
//                = EUSCI_A0 = P2.7/P3.4/P3.3   (J1-8/J1-3/J1-4)
//
//    FLASH = 128 KB       RAM = 8 KB    MAX Clock = 25 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
           // MSP430 F5529  Clock Frequencies.
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        25000000   // Max Clock Frequency 25 MHz
#define  MCU_DEFAULT_SPEED      16000000   // Default Clock Frequency 16 MHz
//#define  MCLK_MHz                   25               // Main Clock Frequency
//#define  SMCLK_MHz                  25               // Sub Main Clock Frequency
           // Note: USB clock requires a separate oscillator (XT2) at 4 MHz * PLL
#define LF_CRYSTAL_FREQUENCY_IN_HZ       32768                   // 32 KHz
#define HF_CRYSTAL_FREQUENCY_IN_HZ     4000000                   //  4 MHz

//#define MCLK_DESIRED_FREQUENCY_IN_KHZ   8000                   //  8 MHz
#define MCLK_DESIRED_FREQUENCY_IN_KHZ    25000                   // 25 MHz
#define MCLK_FLLREF_RATIO    MCLK_DESIRED_FREQUENCY_IN_KHZ / (UCS_REFOCLK_FREQUENCY/1024)  // Ratio = 250

     // Adjust the following if change MCLK_DESIRED_FREQUENCY
#define WDT_DELAY_32ms       WDT_MDLY_0_5      /* approx 32ms interval (0.5ms at 1 MHz CPU) */
#define WDT_DELAY_1ms        WDT_MDLY_0_064    /* approx  1ms interval (0.064ms at 1 MHz CPU) = 1.6 ms */


#define  LED1_INIT()       { P1OUT &= ~BIT0; P1DIR |= BIT0; P1SEL &= ~BIT0; }
#define  LED1_ON()           P1OUT |= BIT0
#define  LED1_OFF()          P1OUT &= ~BIT0
#define  LED1_TOGGLE()       P1OUT ^= BIT0

#define  LED2_INIT()       { P4OUT &= ~BIT7; P4DIR |= BIT7; P4SEL &= ~BIT7; }
#define  LED2_ON()           P4OUT |= BIT7
#define  LED2_OFF()          P4OUT &= ~BIT7
#define  LED2_TOGGLE()       P4OUT ^= BIT7

#define  PUSHBUTTON1_INIT() { P2DIR &= ~BIT1; P2SEL &= ~BIT1; }
#define  PUSHBUTTON1_IN()     (P2IN & BIT1)
#define  PUSHBUTTON1_IRQ_INIT { P2IES |= BIT1; /* set P2.1 for Hi/Lo edge*/ \
                                P2IFG &= ~BIT1;  /* clear any prev IFG */ \
                                P2IE  |= BIT1; } /* enable P2.1 interrupts*/

            // UCB0 SPI port and pin Definitions   (used by CC3000, CC3100, ...)
            //     SCLK = P3.2    MOSI = P3.0    MISO = P3.1
#define  SCLK_PORT      P3OUT
#define  MOSI_PORT      P3OUT
#define  MISO_PORT      P3IN

#define  SCLK           BIT2      // P3.2 - SCLK
#define  SDATO          BIT0      // P3.0 - MOSI
#define  SDATI          BIT1      // P3.1 - MISO

#define  UCB0_SPI_GPIO_INIT()  { P3DIR  |= BIT2; P3SEL |= BIT2; \
                            P3DIR  |= (BIT0); P3DIR &= ~BIT1; \
                            P3SEL  |= (BIT0 + BIT1);  \
                            P3REN  |= BIT1; }             /* pull-ups on MISO */

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID           0xB0
#define CC3100_CS_NORMAL_PORT        P2OUT           /* J2-3 */
#define CC3100_CS_NORMAL_PIN         BIT2
#define CC3100_SPI_MODE              0
#define INIT_CC3100_CS()           { P2DIR |= BIT2; P2SEL &= ~BIT2; }
#define ASSERT_CC3100_CS()          (P2OUT &= ~BIT2)     /* P 2.2 */
#define DEASSERT_CC3100_CS()        (P2OUT |= BIT2)
#define ASSERT_CC3100_CS_NORMAL()   (P2OUT &= ~BIT2)     /* P 2.2 */
#define DEASSERT_CC3100_CS_NORMAL() (P2OUT |= BIT2)


// THe FOLLOWING WERE COPIED FROM FR5969 AND NEED TO BE RE-WORKED

            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions
            //----------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT            BIT2      // P4.2
#define  nSLEEP         BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET          BIT4      // P3.4
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

#define  RESET_HI_RESET        RESET_PORT  |= RESET
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET

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
#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz*1000000)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                   // defined(__MSP430F5529__)



#if defined(__MSP430FR5969__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//
//                   MSP430 - FR5969          GPIO mappings
//
//
// Sets pin/bit definitions for SPI/nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1        = P4.6                LED2        = P1.0
//    PushButton1 = P4.5                PushButton2 = P1.1
//
//    Timers:     =  TA0 (3 ccr)     TA1 (3 ccr)    TA2 (3 ccr)
//                   TA3 (2 ccr)     TB0 (7 ccr)
//
//    UART (USB)  = EUSCI_A0 = P2.0 (TX) / P2.1 (RX) (J1-3/J1-4)
//    I2C         = EUSCI_B0 = P1.6/P1.7             (J2-6/J2-7)
//    SPI         = EUSCI_B0 = P2.2/P1.6/P1.7   (J1-7/J2-6/J2-7)
//                = EUSCI_A1 = P2.4/P1.6/P1.7   (J1-6/J1-3/J1-4)
//
//    FLASH = 64 KB       RAM = 2 KB    MAX Clock = 16 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
           // MSP430 -FR5969  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        16000000   // Max Clock Frequency 16 MHz
//#define  MCU_DEFAULT_SPEED       8000000   // Default Clock Frequency 8 MHz

//#define  MCLK_MHz                   16               // Main Clock Frequency
//#define  SMCLK_MHz                  16               // Sub Main Clock Frequency

#define  LED1_INIT()       { P4OUT &= ~BIT6; P4DIR |= BIT6; P4SEL0 &= ~BIT6; \
                             P4SEL1 &= ~BIT6; }
#define  LED1_ON()           P4OUT |= BIT6
#define  LED1_OFF()          P4OUT &= ~BIT6
#define  LED1_TOGGLE()       P4OUT ^= BIT6

#define  LED2_INIT()       { P1OUT &= ~BIT0; P1DIR |= BIT0; P1SEL0 &= ~BIT0; \
                             P1SEL1 &= ~BIT0; }
#define  LED2_ON()           P1OUT |= BIT0
#define  LED2_OFF()          P1OUT &= ~BIT0
#define  LED2_TOGGLE()       P1OUT ^= BIT0

#define  PUSHBUTTON1_INIT() { P4DIR &= ~BIT5; P4SEL0 &= ~BIT5; \
                              P4SEL1 &= ~BIT5; }
#define  PUSHBUTTON1_IN()    (P4IN & BIT7)

            // UCB0 SPI port and pin Definitions   (used by CC3000, CC3100, ...)
            //     SCLK = P2.2    MOSI = P1.6    MISO = P1.7
#define  SCLK_PORT      P2OUT
#define  MOSI_PORT      P1OUT
#define  MISO_PORT      P1IN

#define  SCLK           BIT2      // P2.2 - SCLK
#define  SDATO          BIT6      // P1.6 - MOSI
#define  SDATI          BIT7      // P1.7 - MISO

#define  UCB0_SPI_GPIO_INIT() { P2DIR  |= BIT2; P2SEL0 |= BIT2; P2SEL1 &= ~BIT2; \
                            P1DIR  |= (BIT6); P1DIR &= ~BIT7; \
                            P1SEL0 |= (BIT6 + BIT7);  \
                            P1SEL1 &= ~(BIT6 + BIT7); \
                            P1REN |= BIT7; }             /* pull-ups on MISO */

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID          0xB0
#define CC3100_CS_NORMAL_PORT       P3OUT           /* J2-3 */
#define CC3100_CS_NORMAL_PIN        BIT0
#define CC3100_SPI_MODE             0
#define INIT_CC3100_CS()    { P3DIR |= BIT0; P3SEL0 &= ~BIT0; P3SEL1 &= ~BIT0; }
#define ASSERT_CC3100_CS()          (P3OUT &= ~BIT0)     /* P 3.0 */
#define DEASSERT_CC3100_CS()        (P3OUT |= BIT0)
#define ASSERT_CC3100_CS_NORMAL()   (P3OUT &= ~BIT0)     /* P 3.0 */
#define DEASSERT_CC3100_CS_NORMAL() (P3OUT |= BIT0)

            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions
            //----------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT            BIT2      // P4.2
#define  nSLEEP         BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET          BIT4      // P3.4
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

#define  RESET_HI_RESET        RESET_PORT  |= RESET
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET

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
#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz*1000000)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec


#endif                   // defined(__MSP430FR5969__)



#if defined(__MSP430FR6989__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//
//                     MSP430 - FR6989           GPIO mappings
//
//
// Sets pin/bit definitions for SPI/nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1        = P1.0                LED2        = P9.7
//    PushButton1 = P1.1                PushButton2 = P1.2
//
//    Timers:     =  TA0 (3 ccr)     TA1 (3 ccr)    TA2 (3 ccr)
//                   TA3 (5 ccr)     TB0 (7 ccr)
//
//    UART (USB)  = EUSCI_A1 = P3.4 (TX) / P3.5 (RX) (USB Only)
//    UART (LP)   = EUSCI_A0 = P4.3 (TX) / P4.2 (RX) (J1-3/J1-4)
//    I2C         = EUSCI_B0 = P1.6/P1.7             (J2-6/J2-7)
//                = EUSCI_B1 = P4.1/P4.0             (J1-9/J1-10)
//                = EUSCI_B1 = P3.1/P3.2             (J4-9/J1-5)
//                = EUSCI_B1 = P3.1/P4.7             (J4-9/J2-10)
//    SPI         = EUSCI_B0 = P1.6/P1.7/P1.0   (J1-7/J2-6/J2-7)
//                = EUSCI_B1 = P4.1/P4.0/P3.0   (J4-8/J1-9/J1-10)
//                = EUSCI_B1 = P3.1/P3.2/P4.2   (J1-4/J4-9/J1-5)
//                = EUSCI_A0 = P2.0/P2.1/P2.2   (J4-5/J1-8/J2-2)
//                = EUSCI_A0 = P4.2/P4.3/P1.5   (J2-3/J1-3/J1-4) UART conflict
//
//    FLASH = 128 KB       RAM = 2 KB     MAX Clock = 16 MHz
//
//    1 ADC module @ 16 channels     4 SPI    2 I2C
//
//    5 Timers:   TA0 (3 CCR)    TA1 (3 CCR)    TA2 (2 CCR)
//                TA3 (5 CCR)    TB0 (7 CCR)
//
//    Extended Scan Interface (ESI)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
#define  MCU_CLOCK_SPEED        16000000   // Max Clock Frequency 16 MHz
#define  MCU_DEFAULT_SPEED       8000000   // Default Clock Frequency 8 MHz

//#define  MCLK_MHz                   16             // Main Clock Frequency
//#define  SMCLK_MHz                  16             // Sub Main Clock Frequency

#define  LED1_INIT()       { P1OUT  &= ~BIT0; P1DIR |= BIT0; P1SEL0 &= ~BIT0; \
                             P1SEL1 &= ~BIT0; }
#define  LED1_ON()           P1OUT  |= BIT0
#define  LED1_OFF()          P1OUT  &= ~BIT0
#define  LED1_TOGGLE()       P1OUT  ^= BIT0

#define  LED2_INIT()       { P9OUT  &= ~BIT7; P9DIR |= BIT7; P9SEL0 &= ~BIT7; \
                             P9SEL1 &= ~BIT7; }
#define  LED2_ON()           P9OUT  |= BIT7
#define  LED2_OFF()          P9OUT  &= ~BIT7
#define  LED2_TOGGLE()       P9OUT ^= BIT7

#define  PUSHBUTTON1_INIT() { P1DIR  &= ~BIT1; P1SEL0 &= ~BIT1; \
                              P1SEL1 &= ~BIT1; }
#define  PUSHBUTTON1_IN()    (P1IN & BIT1)

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID           0xB0
#define CC3100_CS_NORMAL_PORT        P1OUT                /* J2-3 */
#define CC3100_CS_NORMAL_PIN         BIT5
#define CC3100_SPI_MODE              0
#define INIT_CC3100_CS()             { P1DIR |= BIT5; P1SEL0 &= ~BIT5; P1SEL1 &= ~BIT5; }
#define ASSERT_CC3100_CS()           (P1OUT &= ~BIT5)     /* P 1.5 */
#define DEASSERT_CC3100_CS()         (P1OUT |= BIT5)
#define ASSERT_CC3100_CS_NORMAL()    (P1OUT &= ~BIT5)     /* P 1.5 */
#define DEASSERT_CC3100_CS_NORMAL()  (P1OUT |= BIT5)

        // configure CC3100 output nHIB/ENABLE on P 3.2 = J1-5
#define INIT_CC3100_NHIB_ENABLE()    { P3DIR  |=  BIT2; P3SEL0 &= ~BIT2; P3SEL1 &= ~BIT2; }
#define NHIB_CC3100_ENABLE()         (P3OUT |= BIT2)
#define NHIB_CC3100_DISABLE()        (P3OUT &= ~BIT2)

        // configure CC3100 host input IRQ line on P 2.1 = J2-2
#define INIT_CC3100_IRQ()            { P2DIR  &= ~BIT1; P2SEL0 &= ~BIT1; P2SEL1 &= ~BIT1; P2REN |= BIT1; }
#define IRQ_CC3100_ENABLE_INTERRUPTS()  { P2DIR &= ~BIT1;  P2IES &= ~BIT1; P2IE  |= BIT1; }
#define IRQ_CC3100_DISABLE_INTERRUPTS() { P2DIR &= ~BIT1;  P1IE  &= ~BIT2; }

#endif                   // defined(__MSP430FR6989__)



#if defined(__MSP430FR4133__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//
//                      MSP430 - FR4133           GPIO mappings
//
//
// Sets pin/bit definitions for SPI/nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1        = P1.0 <--            LED2        = P4.0
//    PushButton1 = P1.2                PushButton2 = P2.6
//
//    Timers:     =  TA0 (3 ccr)     TA1 (3 ccr)
//
//    UART (USB) UCA0 = P1.0 (TX) and P1.1 (RX)      (J1-3/J1-4)
//                      ^^^^^^^^^^
//                      Note conflict with LED1, must use Jumpers
//    I2C         = EUSCI_B0 = P5.2/P5.3             (J2-6/J2-7)
//    SPI         = EUSCI_B0 = P5.1/P5.2/P5.3   (J1-7/J2-6/J2-7)
//    6 Segment LCD + Specifal Symbols
//
//    FLASH = 15.3 KB     RAM = 2 KB    MAX Clock = 16 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
           // MSP430 FR4133  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        16000000   // Max Clock Frequency 16 MHz
#define  MCU_DEFAULT_SPEED      16000000   // Default Clock Frequency ditto

//#define  MCLK_MHz                    16           // Main Clock Frequency
//#define  SMCLK_MHz                   16           // Sub Main Clock Frequency
 
#define XT1_CRYSTAL_FREQUENCY_IN_HZ  32768
#define REFOCLK_FREQUENCY             32768
#define ONE_SECOND                  1600000

#define MCLK_DESIRED_FREQUENCY_IN_KHZ 16000       // 16 * 1K hz
#define MCLK_FLLREF_RATIO   MCLK_DESIRED_FREQUENCY_IN_KHZ / (REFOCLK_FREQUENCY/1024)

#define  LED1_INIT()       { P1OUT &= ~BIT0; P1DIR |= BIT0; P1SEL0 &= ~BIT0; }
#define  LED1_ON()           P1OUT |= BIT0
#define  LED1_OFF()          P1OUT &= ~BIT0
#define  LED1_TOGGLE()       P1OUT ^= BIT0

#define  LED2_INIT()       { P4OUT &= ~BIT0; P4DIR |= BIT0; P4SEL0 &= ~BIT0; }
#define  LED2_ON()           P4OUT |= BIT0
#define  LED2_OFF()          P4OUT &= ~BIT0
#define  LED2_TOGGLE()       P4OUT ^= BIT0

#define  PUSHBUTTON1_INIT() { P1DIR &= ~BIT2; P1SEL0 &= ~BIT2; }
#define  PUSHBUTTON1_IN()    (P1IN & BIT2)

#define  PUSHBUTTON2_INIT() { P2DIR &= ~BIT6; P2SEL0 &= ~BIT6; }
#define  PUSHBUTTON2_IN()    (P2IN & BIT6)

            // UCB0 SPI port and pin Definitions   (used by CC3000, CC3100, ...)
            //     SCLK = P5.1    MOSI = P5.2    MISO = P5.3
#define  SCLK_PORT      P5OUT
#define  MOSI_PORT      P5OUT
#define  MISO_PORT      P5IN

#define  SCLK           BIT1      // P5.1 - SCLK
#define  SDATO          BIT2      // P5.2 - MOSI
#define  SDATI          BIT3      // P5.3 - MISO

#define  UCB0_SPI_GPIO_INIT()  { P5DIR  |= BIT1;  P5SEL0 |= BIT1; \
                            P5DIR  |= (BIT2); P5DIR &= ~BIT3; \
                            P5SEL0 |= (BIT2 + BIT3);  \
                            P5REN  |= BIT3; }             /* pull-ups on MISO */

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID          0xB0
#define CC3100_CS_NORMAL_PORT       P1OUT           /* J2-3 */
#define CC3100_CS_NORMAL_PIN        BIT6
#define CC3100_SPI_MODE             0
#define INIT_CC3100_CS()         { P1DIR |= BIT6; P1SEL0 &= ~BIT6; }
#define ASSERT_CC3100_CS()        (P1OUT &= ~BIT6)     /* P 1.6 */
#define DEASSERT_CC3100_CS()      (P1OUT |= BIT6)
#define ASSERT_CC3100_CS_NORMAL()    (P1OUT &= ~BIT6)  /* P 1.6 */
#define DEASSERT_CC3100_CS_NORMAL()  (P1OUT |= BIT6)


// THe FOLLOWING WERE COPIED FROM FR5969 AND NEED TO BE RE-WORKED

            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions
            //----------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT            BIT2      // P4.2
#define  nSLEEP         BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET          BIT4      // P3.4
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

#define  RESET_HI_RESET        RESET_PORT  |= RESET
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET

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
#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz*1000000)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec

#endif                   // defined(__MSP430FR4133__)





#if defined(__MSP430FR5739__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  MSP430 - FR5739   GPIO mappings     NOT LAUNCHPAD pin COMPLIANT
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
           // MSP430 -FR5739  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCLK_MHz                    8               // Main Clock Frequency
#define  SMCLK_MHz                   8               // Sub Main Clock Frequency

            // SPI port and pin Definitions
#define  MISO_PORT      P1IN
#define  MOSI_PORT      P1OUT
#define  SCLK_PORT      P2OUT

#define  SCLK           BIT2      // P2.2 - SCLK
#define  SDATO          BIT6      // P1.6 - MOSI
#define  SDATI          BIT7      // P1.7 - MISO

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID          0xB0
#define CC3100_CS_NORMAL_PORT       P1OUT
#define CC3100_CS_NORMAL_PIN        BIT3
#define CC3100_SPI_MODE             0
#define  CC3100_CS      BIT3      // P 1.3  SPI CS
#define  CC3100_ENABLE  BIT2      // P 1.2  aka nHIB   may be P 4.1 instead !
#define  CC3100_IRQ     BIT3      // P 2.3

#define  ASSERT_CC3100_CS()       (P1OUT &= ~BIT3)     /* P 1.3 */
#define  DEASSERT_CC3100_CS()     (P1OUT |= BIT3)

            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions
            //----------------------------------------
            // GPIO Port 2 and 4 pin Definitions
#define  POT            BIT2      // P4.2
#define  nSLEEP         BIT4      // P2.4

            // GPIO Port 3 pin Definitions
#define  RESET          BIT4      // P3.4
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

#define  RESET_HI_RESET        RESET_PORT  |= RESET
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET

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
#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz*1000000)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec


#endif                   // defined(__MSP430FR5739__)



#if defined(__MSP430G2553__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//
//                      MSP430 - G2553           GPIO mappings
//
//
//    LED1        = P1.0               LED2        = P1.6 (SPI conflict)
//    PushButton1 = P1.3  (SW2)
//
//    Timers:     =  TA0 (3 ccr)     TA1 (3 ccr)
//
//    UART (USB) UCA0 = P1.2 (TX) / P1.1 (RX)        (J1-3/J1-4)
//    I2C         = EUSCI_B0 = P1.6/P1.7             (J2-6/J2-7)
//    SPI         = EUSCI_B0 = P2.2/P1.6/P1.7   (J1-7/J2-6/J2-7)
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//
//    FLASH = 16 KB      RAM = 0.5 KB     MAX Clock = 16 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
           // MSP430 - G2553  Clock Frequencies
           // CPU or I/O SM Clock values used to generate timing cycles values
           // e.g. total # of CPU clock cycles in a second = CPU MHz * 1,000,000
#define  MCU_CLOCK_SPEED        16000000   // Max Clock Frequency 16 MHz
#define  MCU_DEFAULT_SPEED       8000000   // Default Clock Frequency 8 MHz
//#define  MCLK_MHz                   16               // Main Clock Frequency
//#define  SMCLK_MHz                   2               // Sub Main Clock Frequency

            // SPI port and pin Definitions
#define  MISO_PORT       P1IN
#define  MOSI_PORT       P1OUT
#define  SCLK_PORT       P1OUT

#define  SCLK            BIT5      // P1.5 - SCLK
#define  SDATO           BIT6      // P1.6 - MOSI   ???  DATASHEET:  1.7 = MOSI
#define  SDATI           BIT7      // P1.7 - MISO   ???              1.6 = MISO

#define  LED1_INIT()       { P1OUT &= ~BIT0; P1DIR |= BIT0; P1SEL &= ~BIT0;  P1SEL2 &= ~BIT0;}
#define  LED1_ON()           P1OUT |= BIT0
#define  LED1_OFF()          P1OUT &= ~BIT0
#define  LED1_TOGGLE()       P1OUT ^= BIT0

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------


            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions
            //----------------------------------------
            // GPIO Port 1 pin Definitions
#define  POT             BIT0      // P 1.0
#define  nSLEEP          BIT4      // P 1.4

            // GPIO Port 2 pin Definitions
#define  RESET           BIT0      // P 2.0
#define  STEP_AIN1       BIT1      // P 2.1
#define  DIR_AIN2        BIT2      // P 2.2
#define  BIN2            BIT4      // P 2.4
#define  BIN1            BIT5      // P 2.5
#define  nSTALL          BIT6      // P 2.6
#define  nFAULT          BIT7      // P 2.7
#define  DRV8711_CS      BIT3      // P 2.3
            
#define  POT_PORT              P1IN     // Note: Function selectors are SEL/SEL2
#define  nSLEEP_PORT           P1OUT
#define  RESET_PORT            P2OUT
#define  STEP_AIN1_PORT        P2OUT
#define  DIR_AIN2_PORT         P2OUT
#define  BIN2_PORT             P2OUT
#define  BIN1_PORT             P2OUT
#define  nSTALL_PORT           P2IN
#define  nFAULT_PORT           P2IN

#define  NSLEEP_HI_ENABLE      nSLEEP_PORT |= nSLEEP
#define  NSLEEP_LO_DISABLE     nSLEEP_PORT &= ~nSLEEP

#define  RESET_HI_RESET        RESET_PORT  |= RESET
#define  RESET_LO_RUN          RESET_PORT  &= ~RESET

#define  STEP_AIN1_HIGH        STEP_AIN1_PORT |= STEP_AIN1
#define  STEP_AIN1_LOW         STEP_AIN1_PORT &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      DIR_AIN2_PORT |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     DIR_AIN2_PORT &= ~DIR_AIN2

#define  BIN2_HI               BIN2_PORT |= BIN2
#define  BIN2_LO               BIN2_PORT &= ~BIN2

#define  BIN1_HI               BIN1_PORT |= BIN1
#define  BIN1_LO               BIN1_PORT &= ~BIN1

#define  ASSERT_DRV8711_CS()   (P2OUT |= BIT3)     /* P 2.3 */
#define  DEASSERT_DRV8711_CS() (P2OUT &= ~BIT3)

            // DRV8711 related Defs
#define  TOTAL_CCR_CLOCK_CYCLES   (SMCLK_MHz*1000000)
#define  SINGLE_STEP_PULSE_WIDTH  1000   // Width of "single step" pulse 63 usec


#endif                   // defined(__MSP430G2553__)


#if defined(YOUR_CUSTOM_MSP430_BOARD)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  MSP430 - xxxx   GPIO mappings
//
//
// Your custom board
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

     // enter associated GPIO settings for your board here

#endif                   // defined(YOUR_CUSTOM_MSP430_BOARD)


#endif                   // __BOARD_MSP430_H__
