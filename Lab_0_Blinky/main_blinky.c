
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                  main_blinky.c
//
//
// Program to checkout and confirm basic board operation including MCU clock
// setup, Systick 1 millisec Timer setup, and basic GPIO setup.
//
//
// History:
//   04/15/15 - Created to provide clocks and timer checkout. WORKS.  Duquaine
//   05/26/15 - Verified with shiny new MSP430-FR6989.
//              Basic board_xxx() calls all good, first shot out of the barrel. Duq
//   07/13/15 - Verified with shiny new STM32F7 Discovery board.  Duq
//   07/21/15 - Worked first shot with newly launched L4 Discovery from ESC-SJO.Duq
//   08/21/15 - Worked first shot with just shipped 180 MHz F4_46 Nucleo. Duq
//
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
                           // ??? make the use of the project file optional ???
#define  NO_PROJECT_FILE
#define  USE_PROJECT_FILE

#include "user_api.h"                 // pull in defs for User API calls

#include "device_config_common.h"     // TEMP HACK

/*******************************************************************************
*                                MAIN                Application's entry point
*******************************************************************************/

int  main (int argc, char **argv)
{
    sys_Init (MCU_CLOCK_SPEED,0);       // Turn off WDT, init MCU clocks, ...

    pin_Config (LED1, GPIO_OUTPUT, 0);  // Setup LED1 for output

    while (1)
      {
         pin_Toggle (LED1);             // Toggle LED1 to blink it
         sys_Delay_Millis (333);        // wait 333 ms (1/3 sec) between blinks
      }
}
