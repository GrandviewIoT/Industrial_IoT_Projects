
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             board_STM32_NO_RTOS.c
//
//
//  Handle "bare metal" implementations, that do not use an RTOS.
//
//  COMPARE THIS TO WHAT  TI  DOES WITH NO_OS COMPONENTS OF CC3100 Library
//  register calls (main execution).
//
//  History:
//    08/08/15 - Created for Industrial IoT OpenSource project.  Duq
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

#include "user_api.h"         // pull in high level User API defs
#include "boarddef.h"         // pull in MCU platform defs and board_xx() protos

    uint32_t  semaphore_waited   = 0;     // DEBUG TESTING
    uint32_t  semaphore_released = 0;


//----------------------------------------------------------------------------
// IO_SEMAPHORE_WAIT
//
//                 Wait on an I/O semaphore until an associated
//                 I/O interrupt handler releases it.
//
//                 In a Low Power platform, we would also execute a LPM sleep
//----------------------------------------------------------------------------
void  IO_SEMAPHORE_WAIT (int *semaphore_waited_on)
{
semaphore_waited++;

    __disable_irq();    // Disable interrupts while checking/updating sema flag
    if (*semaphore_waited_on == 1)
       {  *semaphore_waited_on = 0;  // clear semaphore for next pass
         __enable_irq();             // re-enable interrupts
         return;                     // I/O Handler already posted it complete
       }

//  *semaphore_waited_on = 0;        // clear the I/O flag to be waited on (redundant, is already 0 per above)
    __enable_irq();                  // re-enable interrupts

      // for low power systems, would enter a LPM mode here ...

    while ( ! *semaphore_waited_on)  // Otherwise     (no LPM used)
      {

// NEXT PASS IMPROVEMENT !!! ??? for low power systems, enter LPM mode here ...

         ;                           // loop till semaphore is released by
                                     // I/O interrupt handler / User callback
      }

    *semaphore_waited_on = 0;        // then, clear semaphore for next pass
}


//----------------------------------------------------------------------------
// IO_SEMAPHORE_RELEASE
//
//                Release on an I/O semaphore because the event has occurred.
//
//                This should be executed by I/O interrupt handler ISR,
//                or a user callback operating with an ISR context: e.g.
//                ADC callback, I2C callback, SPI callback, Vtimer callback, ...
//
//                In a Low Power platform, we would trigger the CPU to
//                fully wakeup from an LPM sleep.
//----------------------------------------------------------------------------
void  IO_SEMAPHORE_RELEASE (int *semaphore_waited_on)
{
semaphore_released++;

    *semaphore_waited_on = 1;        // release the blocked semaphore

// NEXT PASS IMPROVEMENT !!! ??? for low power systems, exit LPM mode here ...
}


//--------------------------------------------------------
//  OS_WAIT
//                    is a NOP unless using Interrupts
//--------------------------------------------------------
int  OS_WAIT (uint32_t timeout_value)
{
     return (0);      // perhaps need to pass callback info as well ?
                      // How does a Wait get cancelled ?
}

/******************************************************************************/
