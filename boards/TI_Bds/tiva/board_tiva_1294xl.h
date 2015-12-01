/*******************************************************************************
*
*          board_tiva_1294xl.h             tiva-c launchpad configuration
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

#ifndef _BOARD_TIVA_1294XL_H
#define   _BOARD_TIVA_1294XL_H

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"

#include "inc/tm4c1294ncad.h"

#ifndef FALSE
#define  FALSE  0
#define  TRUE   1
#endif


//--------------------------------------------------------------------------
//                     BOARD   -   COMMON   Routines
//--------------------------------------------------------------------------


/*!
    \brief     Produce "busy loop" delay in ms
    \param[in]         interval - Time in ms
    \return            none
*/
void  Delay (unsigned long interval);


/*!
    \brief          Initialize the system clock of MCU
    \param[in]      none
    \return         none
*/
void  initClk();


/*!
    \brief          Stops the Watch Dog timer
    \param[in]      none
    \return         none
*/
void  stopWDT();


/*!
    \brief register an interrupt handler for the host IRQ

    \param[in]      InterruptHdl    -    pointer to interrupt handler function

    \param[in]      pValue          -    pointer to a memory strcuture that is 
                    passed to the interrupt handler.

    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned

    \note           If there is already registered interrupt handler, the 
                    function should overwrite the old handler with the new one
*/
int  registerInterruptHandler (P_EVENT_HANDLER InterruptHdl , void* pValue);


/*!
    \brief          UART interrupt handler
    \param[in]      none
    \return         none
*/
void  UART1_intHandler ();


//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  Tiva - 123G  GPIO mappings
//
//
// Sets pin/bit definitions for SPI/nSLEEP/RESET/STEP_AIN1/.. etc
//
//    LED1 (Blue) = PF2                  LED2 (Red)  = PF1
//    PushButton1 = PF4                  PushButton2 = PF0
//
//    UART (USB)  = 
//    UART Sep    = UART-1 =  PB0/PB1              (J1-3/J1-4)
//    I2C         = I2C-3  =  PD1/PD0              (J2-6/J2-7)
//    SPI         = SSI-2  =  PB4/PB7/PB6     (J1-7/J2-6/J2-7)
//    I2C         = I2C-1  =  PA6/PA7              (J1-9/J1-10)
//                = I2C-1  =  PE4/PE5              (J1-5/J1-6)
//                = I2C-0  =  PB2/PB3              (J2-2/J4-3)
//    SPI         = SSI-1  =  PF2/PF1/PF0     (J4-1/J3-10/J2-4)
//                = SSI-0  =  PA2/PA4/PA5     (J2-10/J3-8/J1-8)
//
//    FLASH = 256 KB      RAM = 32 KB    MAx Clock = 80 MHz
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------


#define  LED1_INIT()       { MAP_GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE, GPIO_PIN_2); }
#define  LED1_ON()           MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,PIN_HIGH)
#define  LED1_OFF()          MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,PIN_LOW)
                                // see if LED PF2 is currently set, and toggle it the other way
#define  LED1_TOGGLE()     { if (GPIO_PORTF_DATA_R & 0x04) \
                                GPIO_PORTF_DATA_R &= ~(0x04); \
                                else GPIO_PORTF_DATA_R |= 0x04; }

#define  LED2_INIT()       { MAP_GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE, GPIO_PIN_1); }
#define  LED2_ON()           MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,PIN_HIGH)
#define  LED2_OFF()          MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,PIN_LOW)
                                // see if LED PF1 is currently set, and toggle it the other way
#define  LED2_TOGGLE()     { if (GPIO_PORTF_DATA_R & 0x02) \
                                GPIO_PORTF_DATA_R &= ~(0x02); \
                                else GPIO_PORTF_DATA_R |= 0x02; }

#define  PUSHBUTTON1_INIT() { MAP_GPIOPinTypeGPIOInput (GPIO_PORTF_BASE, GPIO_PIN_4); }
#define  PUSHBUTTON1_IN()     GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)
#define  PUSHBUTTON1_IRQ_INIT() { MAP_GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE); \
                                  MAP_GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4); \
                                  MAP_GPIOIntDisable(GPIO_PORTF_BASE,GPIO_PIN_4); } /* caller must enable */

            // SSI2 SPI port and pin Definitions   (used by CC3000, CC3100, ...)
            //     SCLK = PB4    MOSI = PB7    MISO = PB6
#define  MISO_PORT       GPIO_PORTB_DATA_R
#define  MOSI_PORT       GPIO_PORTB_DATA_R
#define  SCLK_PORT       GPIO_PORTB_DATA_R

            // SPI pin Definitions
#define  SCLK            GPIO_PIN_4         // PB4 - SCLK
#define  SDATO           GPIO_PIN_7         // PB7 - MOSI
#define  SDATI           GPIO_PIN_6         // PB6 - MISO

                                 // Setup pin-muxing first, then set pins to SPI mode
#define  SPI2_GPIO_INIT() { MAP_GPIOPinConfigure (GPIO_PB4_SSI2CLK); \
                            MAP_GPIOPinConfigure (GPIO_PB6_SSI2RX); \
                            MAP_GPIOPinConfigure (GPIO_PB7_SSI2TX); \
                            MAP_GPIOPinTypeSSI (GPIO_PORTB_BASE,    \
                                           GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7); }

            //----------------------------------------
            // CC3100 GPIO Port and Pin Definitions - PE0
            //----------------------------------------
#define CC3100_SPI_PORT_ID          2
#define CC3100_CS_NORMAL_PORT       GPIO_PORTE_BASE
#define CC3100_CS_NORMAL_PIN        GPIO_PIN_0
#define CC3100_SPI_MODE             0
#define INIT_CC3100_CS()            MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0)
#define ASSERT_CC3100_CS_NORMAL()   MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0, PIN_LOW)
#define DEASSERT_CC3100_CS_NORMAL() MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0, PIN_HIGH)


            //----------------------------------------
            // DRV8711 GPIO Port and Pin Definitions - PA2
            //----------------------------------------
#define  INIT_DRV8711_CS()     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTa_BASE,GPIO_PIN_2)
#define  ASSERT_DRV8711_CS()   (GPIO_PORTA_DATA_R |= GPIO_PIN_2)
#define  DEASSERT_DRV8711_CS() (GPIO_PORTA_DATA_R &= ~GPIO_PIN_2)


#if defined(USES_CC3100)
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//                     BOARD   -   CC3100  Specific   Routines
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

#include "simplelink.h"


/*!
    \brief             Enables the CC3100
    \param[in]         none
    \return            none
*/
void  CC3100_enable();


/*!
    \brief             Disables the CC3100
    \param[in]         none
    \return            none
*/
void  CC3100_disable();


/*!
    \brief          Enables the interrupt from the CC3100
    \param[in]      none
    \return         none
*/
void  CC3100_InterruptEnable();


/*!
    \brief          Disables the interrupt from the CC3100
    \param[in]      none
    \return         none
*/
void  CC3100_InterruptDisable();


/*!
    \brief      Masks the Host IRQ
    \param[in]      none
    \return         none
*/
void  MaskIntHdlr();


/*!
    \brief     Unmasks the Host IRQ
    \param[in]      none
    \return         none
*/
void  UnMaskIntHdlr();


/*!
    \brief          GPIOB interrupt handler
    \param[in]      none
    \return         none
*/
void  GPIOB_intHandler();
#endif                        //  #if defined(USES_CC3100)


#if defined(USES_DRV8711)
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//                   BOARD   -   DRV8711  Specific   Routines    Stepper
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

//#include "devices/DRV8711_Spin_Routines.h"    TEMP - NEED TO INTEGRATE IN FUTURE 04/28/15
//#include "devices/DRV8711_registers.h"

void  ISR_Interval_Timer (void);
void  ISR_PWM_Generator (void);

    extern   uint32_t  g_SysClkTicks;       // CPU Clock ticks/sec (50 MHz)
    extern   uint32_t  g_PWMclock;          // PWM Clock ticks/sec (1/32 of CPU)

//--------------------------------------------------------------------------
// CPU and PWM Clock values used to generate timing cycles values
//
//  e.g. total number of CPU clock cycles in a second = CPU MHz * 1,000,000
//--------------------------------------------------------------------------
            // Tiva Clock Frequencies
#define CPU_CLOCK_CYCLES_PER_SEC   g_SysClkTicks
#define PWM_CLOCK_CYCLES_PER_SEC   g_PWMclock
#define TOTAL_CCR_CLOCK_CYCLES     (PWM_CLOCK_CYCLES_PER_SEC)
#define SINGLE_STEP_PULSE_WIDTH    63   // Width of a "single step" pulse usec

            //-----------------------------------------------
            // DRV8711 GPIO Port and Pin Mapping Definitions
            //-----------------------------------------------
#define  nSLEEP     GPIO_PIN_5
#define  RESET      GPIO_PIN_5
#define  STEP_AIN1  GPIO_PIN_6
#define  DIR_AIN2   GPIO_PIN_7
#define  BIN1       GPIO_PIN_4
#define  BIN2       GPIO_PIN_3

#define  nSTALL     GPIO_PIN_2
#define  nFAULT     GPIO_PIN_0

#define  POT_PORT              GPIO_PORTB_DATA_R
#define  nSLEEP_PORT           GPIO_PORTE_DATA_R
#define  RESET_PORT            GPIO_PORTA_DATA_R
#define  STEP_AIN1_PORT        GPIO_PORTA_DATA_R
#define  DIR_AIN2_PORT         GPIO_PORTA_DATA_R
#define  BIN2_PORT             GPIO_PORTA_DATA_R
#define  BIN1_PORT             GPIO_PORTA_DATA_R
#define  nSTALL_PORT           GPIO_PORTB_DATA_R
#define  nFAULT_PORT           GPIO_PORTE_DATA_R

#define  NSLEEP_HI_ENABLE      GPIO_PORTE_DATA_R |= nSLEEP
#define  NSLEEP_LO_DISABLE     GPIO_PORTE_DATA_R &= ~nSLEEP

#define  RESET_HI_RESET        GPIO_PORTA_DATA_R |= RESET
#define  RESET_LO_RUN          GPIO_PORTA_DATA_R &= ~RESET

#define  STEP_AIN1_HIGH        GPIO_PORTA_DATA_R |= STEP_AIN1
#define  STEP_AIN1_LOW         GPIO_PORTA_DATA_R &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      GPIO_PORTA_DATA_R |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     GPIO_PORTA_DATA_R &= ~DIR_AIN2

#define  BIN1_HI               GPIO_PORTA_DATA_R |= BIN1
#define  BIN1_LO               GPIO_PORTA_DATA_R &= ~BIN1

#define  BIN2_HI               GPIO_PORTA_DATA_R |= BIN2
#define  BIN2_LO               GPIO_PORTA_DATA_R &= ~BIN2

//#define  CS_ASSERT           GPIO_PORTA_DATA_R &= ~SPI_CS   // fails
//#define  CS_DEASSERT         GPIO_PORTA_DATA_R |= SPI_CS    // fails
#define  ASSERT_DRV8711_CS()   (GPIO_PORTA_DATA_R |= GPIO_PIN_2)
#define  DEASSERT_DRV8711_CS() (GPIO_PORTA_DATA_R &= ~GPIO_PIN_2)

#endif                        //  #if defined(USES_DRV8711)


#endif                        //  _BOARD_TIVA_1294XL_H
