/*
 * spi.c - tiva-c launchpad spi interface implementation
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _SPI_TIVA_H
#define   _SPI_TIVA_H

#include "boarddef.h"
#include "tiva/spi.h"
#include "tiva/board_tiva_123g.h"


    int            tiva_spi_mode = 0;   // DEBUG
    unsigned long  spi_Mode      = 0;   // DEBUG
    unsigned long  outbuf [2];          // DEBUG
    unsigned long  inbuf  [2];          // DEBUG


//**************************************************************************
//**************************************************************************
//
//                       SPI   -   COMMON   Routines
//
//**************************************************************************
//**************************************************************************

//-----------------------------------------------------------------------
//  spi_Open
//
//          In future, add DMA and Interrupts.
//
//          This is called by our DRV8xxx code for our Motor stuff.
//          It is also automatically called by TI's SimpleLink software
//          stack's sl_start() routine, when CC3100 is used.
//-----------------------------------------------------------------------

Fd_t  spi_Open (char *ifName, unsigned long flags)
{
    uint32_t   dummy;

    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI2);  // turn on SPI periph clock
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB); // turn on clocks for SPI pins

       // Configure the pin muxing for SPI2 functions on port PB4, PB6, PB7.
       // PB4, PB6, PB7 are used for SPI2 on the Tiva 123 booster pack.
    MAP_GPIOPinConfigure (GPIO_PB4_SSI2CLK);
    MAP_GPIOPinConfigure (GPIO_PB6_SSI2RX);
    MAP_GPIOPinConfigure (GPIO_PB7_SSI2TX);

       // Configure GPIOs for SPI usage.  This gives
       // control of these pins to the SPI hardware.
    MAP_GPIOPinTypeSSI (GPIO_PORTB_BASE, 
                        GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

//     part of original Tiva code
// GPIOPinConfigure (GPIO_PB5_SSI2FSS);
// MAP_GPIOPinTypeSSI (GPIO_PORTB_BASE, GPIO_PIN_5);
       /* CLOCK IS CONFIGURED AT 12 MHz */
// SSIConfigSetExpClk (SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
//                     SSI_MODE_MASTER, 12000000, 8);


//if (tiva_spi_mode == 0)
//   spi_Mode = SSI_FRF_MOTO_MODE_0;
//   else if (tiva_spi_mode == 1)
//           spi_Mode = SSI_FRF_MOTO_MODE_1;
//   else if (tiva_spi_mode == 2)
//           spi_Mode = SSI_FRF_MOTO_MODE_2;
//           else spi_Mode = SSI_FRF_MOTO_MODE_3;


       // Configure and enable the SPI port for SPI master mode. For SPI2,
       // use system clock supply, master mode, 8-bit data, 1 MHz Baud Rate
       // with SPI Data mode 0: idle clock level low and active low clock.
    MAP_SSIConfigSetExpClk (SSI2_BASE, SysCtlClockGet(), 
                            SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 
                            1000000, 8);

    MAP_SSIEnable (SSI2_BASE);

    while (MAP_SSIDataGetNonBlocking(SSI2_BASE, &dummy))
      {
         // clear out any trash left in SPI FIFO
      }

#if defined(USES_CC3100)
        //-----------------------------------------------------------
        // before returning to a SimpleLink CC3100 caller, finish up
        // the Interrupt initialization that it expects.
        //-----------------------------------------------------------
    MAP_GPIOIntClear (GPIO_PORTB_BASE, GPIO_PIN_2);
    MAP_GPIOIntDisable (GPIO_PORTB_BASE, GPIO_PIN_2);
        //--------------------------------
        // Enable CC3100 IRQ interrupts
        //--------------------------------
    IntEnable (INT_GPIOB);
    IntMasterEnable();

        // 1 ms delay to let CC3100 settle out
    MAP_SysCtlDelay ( (MAP_SysCtlClockGet()/(3*1000))*50 );

        //----------------------------------
        // Enable WLAN interrupt (IRQ line)
        //----------------------------------
    CC3100_InterruptEnable();
#endif                                 // #if defined(USES_CC3100)

    return (0);                // denote OK return code
}



//--------------------------------------------------------------
//  spi_Close
//
//          In future, add DMA and Interrupts
//--------------------------------------------------------------

int spi_Close (Fd_t fd)
{

#if defined(USES_CC3100)
        // Disable WLAN Interrupt ...
    CC3100_InterruptDisable();
#endif

    return (0);                // denote OK return code
}



#if defined(USES_CC3100)
//**************************************************************************
//**************************************************************************
//
//                       SPI   -   CC3100  Specific   Routines
//
//**************************************************************************
//**************************************************************************
#ifndef SL_IF_TYPE_UART
#include "simplelink.h"

int spi_Write (Fd_t fd, unsigned char *pBuff, int len)
{
    int      len_to_return = len;
    uint32_t ulDummy;

    ASSERT_CC3100_CS_NORMAL();

    while(len)
      {
        while (MAP_SSIDataPutNonBlocking(SSI2_BASE, (unsigned long)*pBuff) != TRUE);
        while (MAP_SSIDataGetNonBlocking(SSI2_BASE, &ulDummy) != TRUE);
        pBuff++;
        len--;
      }
    DEASSERT_CC3100_CS_NORMAL();

    return len_to_return;
}


int spi_Read (Fd_t fd, unsigned char *pBuff, int len)
{
    int      i = 0;
    uint32_t ulBuff;

    ASSERT_CC3100_CS_NORMAL();

    for (i = 0;  i < len;  i++)
      {
        while (MAP_SSIDataPutNonBlocking(SSI2_BASE, 0xFF) != TRUE);
        while (MAP_SSIDataGetNonBlocking(SSI2_BASE, &ulBuff) != TRUE);
        pBuff[i] = (unsigned char) ulBuff;
      }
    DEASSERT_CC3100_CS_NORMAL();

    return len;
}
#endif                        /* SL_IF_TYPE_UART */
#endif                        // defined(USES_CC3100)



#if defined(USES_DRV8711)
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
//          SPI  I/O  routine  for  DRV8711 Stepper Controller.
//
//  Note:   SSI_SR_TNF: 1 = transmit FIFO is not full, i.e. it
//                  _only_ denotes if room is available in the
//                  TX FIFO buffer. It does _not_ guarantee that
//                  the data bytes have been sent out.
//
//          SSI_SR_BSY: 1 = SPI is currently transmitting and/or
//                          receiving a frame.
//                      0 = all TX bytes and remaining bits in
//                          the last word have all been sent out,
//                          and reply byte has been received.
//
//          SSI_SR_RNE: 1 = receive FIFO is not empty (data has
//                          been received and is queued in FIFO)
//                      0 = receive FIFO is empty
//--------------------------------------------------------------

unsigned short  SPI_DRV8711_ReadWrite (unsigned char data_Hi,
                                       unsigned char data_Lo)
{
    int             i;
    uint32_t        rdata;
    volatile unsigned short  read_Data = 0;

    ASSERT_DRV8711_CS();         // Assert CS

#if (ORIGINAL)
        //---------------------------
        //     Send dataHi byte
        //---------------------------
        // Wait until there is space in TX FIFO.
    while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_TNF))
      {
      }
        // Write the data to the SSI.
    HWREG(SSI2_BASE + SSI_O_DR) = dataHi;

        // Wait until there is data to be read from RX FIFO.
    while (!(HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_RNE))
      {
      }
        // Read reply data from SSI.
    rdata     = HWREG(SSI2_BASE + SSI_O_DR);
    readData |= (unsigned short) ((rdata & 0x000000FF) << 8);

        //---------------------------
        //     Send dataLo byte
        //---------------------------
        // Wait until there is space in TX FIFO.
    while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_TNF))
      {
      }
        // Write the data to the SSI.
    HWREG(SSI2_BASE + SSI_O_DR) = dataLo;

        // Wait until there is data to be read from RX FIFO.
    while (!(HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_RNE))
      {
      }
        // Read reply data from SSI.
    rdata     = HWREG(SSI2_BASE + SSI_O_DR);
    readData |= (unsigned short) (rdata & 0x000000FF);
#endif

#if (TRY_2)
        //---------------------------
        //     Send dataHi byte
        //---------------------------
        // Wait until there is space in the TX FIFO.
    while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_TNF))
      {
      }
        // Write the data to the SPI.
    HWREG(SSI2_BASE + SSI_O_DR) = data_Hi;

        // Wait until all sent, and there is data to be read from RX.
    while ((HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_BSY))    // CHANGE from ! (HWREG...)
      {
      }
//   while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_RNE))
      ;                                // ensure burbled up thru FIFO -- NEW -- 12/08
        // Read reply data from SPI.
    rdata      = HWREG(SSI2_BASE + SSI_O_DR);
    read_Data |= (unsigned short) ((rdata & 0x000000FF) << 8);

        //---------------------------
        //     Send dataLo byte
        //---------------------------
        // Ensure there is space in TX FIFO.
    while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_TNF))
      {
      }
        // Write the data to the SPI.
    HWREG(SSI2_BASE + SSI_O_DR) = data_Lo;

        // Wait until all sent, and there is data to be read from RX.
    while ((HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_BSY))    // CHANGE from ! (HWREG...)
      {
      }
//  while ( ! (HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_RNE))
      ;                                // ensure burbled up thru FIFO -- NEW -- 12/08

        // Read reply data from SPI.
    rdata      = HWREG(SSI2_BASE + SSI_O_DR);
    read_Data |= (unsigned short) (rdata & 0x000000FF);
#endif                             // (TRY_2)

        outbuf[0] = data_Hi;
        outbuf[1] = data_Lo;
        read_Data = 0;             // DEBUG HOOK

             //-----------------------------------
             //  Send 2 bytes of data (thru FIFO)
             //-----------------------------------
        for (i = 0; i < 2; i++)
          {
              // Send the data using the "blocking" put function.  This function
              // will wait until there is room in the send FIFO before returning.
              // This allows you to assure that all the data you send makes it into
              // the send FIFO.
           SSIDataPut (SSI2_BASE, outbuf[i]);
          }
             // Wait until SSI2 is done transferring all the data in the transmit FIFO.
        while (SSIBusy(SSI2_BASE))
          {
          }
             //--------------------------------------
             //  Receive 2 bytes of data (from FIFO)
             //--------------------------------------
// DEBUG  --- DO _NOT_ PUT DEBUG STOP HERE --- FAILS !!! Screws up SSI2 FIFO
        for (i = 0; i < 2; i++)
          {
               // Receive the data using the "blocking" Get function. This function
               // will wait until there is data in the receive FIFO before returning.
            SSIDataGet (SSI2_BASE, &inbuf[i]);

               // Since we are using 8-bit data, mask off the MSB.
            inbuf[i] &= 0x000000FF;
          }
             // Wait until SSI2 is done transferring all the data in the receive FIFO.
        while (SSIBusy(SSI2_BASE))
          {
          }

/// CS_DEASSERT;                   // De-assert CS
    DEASSERT_DRV8711_CS();         // De-assert CS

// DEBUG --- PUT DEBUG STOP HERE, _AFTER_ CS dropped ---
    read_Data  = (unsigned short) ((inbuf[0] & 0x000000FF) << 8);
    read_Data |= (unsigned short) (inbuf[1] & 0x000000FF);

    return (read_Data);       // return any SPI reply as a 2 byte short
}
#endif                        // defined(USES_DRV8711)

#endif                        // #define _SPI_TIVA_H

