

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_MSP432.h
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
//   03/29/15 - Board arrived - Created initial version.
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


#ifndef  __BOARD_MSP432_H__
#define  __BOARD_MSP432_H__

//#include "msp432/spi.h"

#ifndef FALSE
#define  FALSE  0
#define  TRUE   1
#endif


                    //-------------------------------------------
                    //  MSP432  I/O Pin definitions by processor
                    //-------------------------------------------

#if defined(__MSP432P401R__) || defined(TARGET_IS_MSP432P4XX)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  MSP432  ARM - P401   GPIO mappings
//
//
// See utility for pin/bit definitions for nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1 = P1.0                                  PushButton1 = P6.7
//    RGB LED2:    Red = P2.0     Green = P2.1     Blue = P2.2
//
//    Timers:     =  TA0 (5 ccr)     TA1 (5 ccr)    TA2 (5 ccr)
//                   TA3 (7 ccr)
//
//                   Timer32 (2)
//
//    UART (USB)  = EUSCI_A0 = P1.2 / P1.3
//    UART Sep    = EUSCI_A2 = P3.2 / P3.3           (J1-3/J1-4)
//    I2C         = EUSCI_B0 = P1.6/P1.7             (J2-6/J2-7)
//    SPI         = EUSCI_B0 = P1.5/P1.6/P1.7   (J1-7/J2-6/J2-7)
//    I2C         = EUSCI_B1 = P6.5/P6.4             (J1-9/J1-10)
//                = EUSCI_B2 = P3.6/P3.7             (J2-10/J4-10)
//    SPI         = EUSCI_B2 = P3.5/P3.6/P3.7   (J4-9/J2-10/J4-10)
//    I2C         = EUSCI_B3 = P6.6/P6.7             (J4-5/J4-6)
//
//    FLASH = 256 KB       RAM = 64 KB    MAX Clock = 48 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
#define  MCU_CLOCK_SPEED        48000000   // Max Clock Frequency 48 MHz
#define  MCU_DEFAULT_SPEED      48000000   // Default Clock Frequency ditto

//#define  MCLK_MHz                   48               // Main Clock Frequency
//#define  SMCLK_MHz                  48               // Sub Main Clock Frequency

#define  LED1_INIT()       { P1OUT &= ~BIT0; P1DIR |= BIT0; P1SEL0 &= ~BIT0; \
                             P1SEL1 &= ~BIT0; }
#define  LED1_ON()           P1OUT |= BIT0
#define  LED1_OFF()          P1OUT &= ~BIT0
#define  LED1_TOGGLE()       P1OUT ^= BIT0

#define  LED2_INIT()       { P2OUT &= ~BIT2; P2DIR |= BIT2; P2SEL0 &= ~BIT2; \
                             P2SEL1 &= ~BIT2; }        /* Use Blue P2.2 */
#define  LED2_ON()           P2OUT |= BIT2
#define  LED2_OFF()          P2OUT &= ~BIT2
#define  LED2_TOGGLE()       P2OUT ^= BIT2

#define  PUSHBUTTON1_INIT() { P6DIR &= ~BIT7; P6SEL0 &= ~BIT7; \
                              P6SEL1 &= ~BIT7; }
#define  PUSHBUTTON1_IN()    (P6IN & BIT7)
#define  PUSHBUTTON1_IRQ_INIT { P6IES |= BIT7; /* set P6.7 for Hi/Lo edge*/ \
                                P6IFG &= ~BIT7;  /* clear any prev IFG */ \
                                P6IE  |= BIT7; } /* enable P6.7 interrupts*/

            // UCB0 SPI port and pin Definitions   (used by CC3000, CC3100, ...)
            //     SCLK = P1.5    MOSI = P1.6    MISO = P1.7
#define  SCLK_PORT           P1OUT
#define  MOSI_PORT           P1OUT
#define  MISO_PORT           P1IN

#define  SCLK                BIT5      // P1.5 - SCLK
#define  SDATO               BIT6      // P1.6 - MOSI
#define  SDATI               BIT7      // P1.7 - MISO

#define  UCB0_SPI_GPIO_INIT()  { P1DIR  |= (BIT5 + BIT6); P1DIR &= ~BIT7; \
                             P1SEL0 |= (BIT5 + BIT6 + BIT7);  \
                             P1SEL1 &= ~(BIT5 + BIT6 + BIT7); \
                             P1REN |= BIT7; }             /* pull-ups on MISO */

           //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions
            //----------------------------------------
#define CC3100_SPI_PORT_ID      0xB0
#define CC3100_CS_NORMAL_PORT   P3OUT
#define CC3100_CS_NORMAL_PIN    BIT0
#define CC3100_SPI_MODE         0
#define INIT_CC3100_CS()      { P3DIR |= BIT0; P3SEL0 &= ~BIT0; P3SEL1 &= ~BIT0; }
#define ASSERT_CC3100_CS()     (P3OUT &= ~BIT0)     /* CC3x00 CS = P 3.0 */
#define DEASSERT_CC3100_CS()   (P3OUT |= BIT0)
#define ASSERT_CC3100_CS_NORMAL()   (P3OUT &= ~BIT0)     /* CC3x00 CS = P 3.0 */
#define DEASSERT_CC3100_CS_NORMAL() (P3OUT |= BIT0)


// See ~/--MQTT--/spi_cc3100/spi_cc3100.c

    /* P4.1 on MSP432 - WLAN enable full DS */
    /* Configure SPI IRQ line on P2.5 on MSP432 */



// The following were copied from MSP430-FR5969 - need to be re-worked

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

#endif             // defined(__MSP432P401R__) || defined(TARGET_IS_MSP432P4XX)



#if defined(YOUR_CUSTOM_MSP432_BOARD)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  MSP432 - xxxx   GPIO mappings
//
//
// Your custom board
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

     // enter associated GPIO settings for your board here

#endif                   // defined(YOUR_CUSTOM_MSP432_BOARD)


#endif                   // __BOARD_MSP432_H__
