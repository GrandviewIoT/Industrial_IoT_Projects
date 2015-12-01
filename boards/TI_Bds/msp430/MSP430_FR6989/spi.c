/*
 * spi.c - msp430 FR6989 launchpad spi interface implementation - with WVD MODIFICATIONS
 *
 *                       SPI support for MSP430 FR6989
 *
 * Copyright (C) WVD did total rewrite
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "boarddef.h"
#include <msp430fr6989.h>
#include "msp430/spi.h"

#if defined(USES_CC3100)
#include "simplelink.h"
#endif

    unsigned long  outbuf [2];          // DEBUG
    unsigned long  inbuf  [2];          // DEBUG

    unsigned long  num_spi_writes = 0;  // DEBUG counters
    unsigned long  num_spi_reads  = 0;

//**************************************************************************
//**************************************************************************
//
//                       SPI   -   COMMON   Routines
//
//**************************************************************************
//**************************************************************************

//--------------------------------------------------------------
//  spi_Open
//
//          In future, add DMA and Interrupts
//
//          Called by:
//--------------------------------------------------------------

Fd_t  spi_Open (char *ifName, unsigned long flags)
{

#if defined(__MSP430FR6989__) || defined(__MSP430FR5969__) || defined(__MSP430FR4133__)
    PM5CTL0 &= ~LOCKLPM5;    // Disable the GPIO power-on default high-impedance mode
                             // to activate previously configured port settings
#endif

    P1SEL0 |= BIT4;          // Select SCLK as SPI line: CLK on P1.4
    P1SEL1 &= ~BIT4;
 
        // Select MOSI/MISO as SPI lines: MOSI/MISO on P1.6, P1.7
    P1SEL0 |=  (BIT6 + BIT7);     // 05/27/15 - The SEL1/SEL0 settings were just backwards, causing SPI failure !
    P1SEL1 &= ~(BIT6 + BIT7);

// ??? need ???  causing "issues" ???
    P1OUT  |= BIT7;          // MISO is an open-drain input - set it for Pull-Up
    P1REN  |= BIT7;          // turn on MISO pullups


     //----------------------------
     //  Configure GPIOs for SPI - TESTED
     //----------------------------
    P1SEL0 |= BIT4;                // set USCI_B0 GPIOs for SPI operation: Clock on     P 1.4
    P1SEL0 |= BIT6 | BIT7;         // set USCI_B0 GPIOs for SPI operation  MOSI/MISO on P 1.6/P 1.7

    UCB0CTLW0 |= UCSWRST;    // Put SPI state machine in reset - on UCB0

        // Configure UCB0 SPI: 3-pin, 8-bit SPI master
    UCB0CTLW0 = UCMSB + UCMST + UCSYNC + UCCKPH;
    UCB0CTL1  = UCSWRST + UCSSEL_2;             // Use SMCLK, keep in RESET

        // Set UCB0 SPI clock / Baud Rate
//  UCB0BR0 = 0x00; // f_UCxCLK = 8MHz - UBSTABLE on FR5969 12/11/14
    UCB0BR0 = 0x02; // yields 415 KHz
    UCB0BR1 = 0;

    UCB0CTL1 &= ~UCSWRST;    // Take SPI state machine out of reset

        //-------------------------------------
        // Enable MCU general interrupts (GIE)
        //-------------------------------------
    __enable_interrupt();

    Delay (50);     // 50 ms delay needed at CC3100 startup

        //------------------------------------
        // Enable CC3100 interrupt (IRQ line)
        //------------------------------------
    CC3100_InterruptEnable();

    return (0);              // denote OK return code
}


//--------------------------------------------------------------
//  spi_Close
//
//          In future, add DMA and Interrupts
//--------------------------------------------------------------

int  spi_Close (Fd_t fd)
{
#if defined(USES_CC3100)
        // Disable WLAN Interrupt
    CC3100_InterruptDisable();
#endif

    return (0);              // denote OK return code
}



#if defined(USES_CC3100)
//**************************************************************************
//**************************************************************************
//
//                       SPI   -   CC3100  Specific   Routines
//
//**************************************************************************
//**************************************************************************
int  spi_Write (Fd_t fd, unsigned char *pBuff, int len)
{
    int len_to_return = len;

    ASSERT_CC3100_CS();

    while (len)
      {
        while ( ! (UCB0IFG & UCTXIFG))
          ;
        UCB0TXBUF = *pBuff;

        while ( ! (UCB0IFG & UCRXIFG))
          ;
        UCB0RXBUF;

        len --;
        pBuff++;
      }

    DEASSERT_CC3100_CS();

num_spi_writes++;                  // DEBUG - track how many SPI writes issued

    return len_to_return;
}


int  spi_Read (Fd_t fd, unsigned char *pBuff, int len)
{
    int i = 0;

    ASSERT_CC3100_CS();

//while (1)                         // WVD DEBUG HOOK  05/27/15 - at startup, a whole bunch of reads are issued
    for (i = 0; i < len; i ++)
      {
        while ( ! (UCB0IFG & UCTXIFG))
          ;
//      UCB0TXBUF = 0xFF;
        UCB0TXBUF = 0x55;

        while ( ! (UCB0IFG & UCRXIFG))
          ;
        pBuff[i] = UCB0RXBUF;
      }

    DEASSERT_CC3100_CS();

num_spi_reads++;                  // DEBUG - track how many SPI writes issued

    return len;
}
#endif                        // defined(CC3100)



#if defined(DRV8711)
//**************************************************************************
//**************************************************************************
//
//                       SPI   -   DRV8711  Specific   Routines    Stepper
//
//**************************************************************************
//**************************************************************************

void  SPI_DRV8711_init (unsigned long flags)
{
          // call spi_open, but no Fd_t nor filename required
    spi_Open (NULL, flags);
}


//--------------------------------------------------------------
//  SPI_DRV8711_ReadWrite
//
//          SPI  I/O  routine  for  DRV8711 Stepper Controller - uses USCI/UCB0
//--------------------------------------------------------------

unsigned short  SPI_DRV8711_ReadWrite (unsigned char data_Hi, unsigned char data_Lo)
{
    unsigned short  readData = 0;

    outbuf[0] = data_Hi;             // DEBUG TRACE
    outbuf[1] = data_Lo;
    readData = 0;                    // DEBUG HOOK

    ASSERT_DRV8711_CS();             // Assert CS
//G2553 P2OUT |= CS;                   // Assert CS  TILT !!!  BUT IT WORKS !
//  P1OUT |= CS;       // CHANGE for FR5969   -- WORKS --

    while ( ! (UCB0IFG & UCTXIFG));  // loop till SPI transmitter is free
    UCB0TXBUF = data_Hi;             // Send first byte
    while ( ! (UCB0IFG & UCRXIFG));  // wait till it completes
/// readData |= (UCB0RXBUF << 8);    // read any reply
    inbuf[0] = UCB0RXBUF;            // read any reply

    while ( ! (UCB0IFG & UCTXIFG));  // loop till SPI transmitter is free
    UCB0TXBUF = data_Lo;             // Send second byte
    while ( ! (UCB0IFG & UCRXIFG));  // wait till it completes
/// readData |= UCB0RXBUF;           // read any reply
    inbuf[1] = UCB0RXBUF;            // read any reply

    DEASSERT_DRV8711_CS();           // De-assert CS
//G2553   P2OUT &= ~CS;                  // De-assert CS   Ditto TILT !!!
//  P1OUT &= ~CS;       // CHANGE for FR5969   -- WORKS --


// DEBUG --- PUT DEBUG STOP HERE, _AFTER_ CS dropped ---
    readData  = (unsigned short) (inbuf[0] << 8);
    readData |= (unsigned short) (inbuf[1]);

    return readData;               // return any SPI reply as a 2 byte int/short
}
#endif                             // defined(DRV8711)
