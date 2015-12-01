//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            user_api_msp432_pin_map.h
//
//  Provide a higher level user API for newbies.
//  More experienced programmers can directly use the board_xxx calls
//  or the platform's DriverLib APIs.
//
//  See user_api.h in ~/boards/TI_Bds directory for more information
//
//  This provides the specific pin mapping for the Tiva 123G Launchpad.
//
// History:
//   07/10/15 - Created.  Duquaine
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
*******************************************************************************/

#ifndef  __USER_API_BOARD_H__
#define  __USER_API_BOARD_H__


//*******************************************************************************
//*******************************************************************************
//                    LOGICAL  PIN  and  CHANNEL  MAPPINGS
//
// These supply an index to a port table, that contains the all the addresses
// used with the port, e.g.
//
//  index    Port Address entries
//    0    { P0OUT, P0IN, P0DIR, P0SEL0, P0SEL1. P0REN }  or nulls if no Port 0
//    1    { P1OUT, P1IN, P1DIR, P1SEL0, P1SEL1. P1REN }
//    2    { P2OUT, P2IN, P2DIR, P2SEL0, P2SEL1. P2REN }
//   ...       ...   ...    ...    ...     ...     ...  
//    6    { P6OUT, P6IN, P6DIR, P6SEL0, P6SEL1. P6REN, P6IE, P6IFG, P6IES }
//
// We then use the index to lookup the actual address from the above table.
//*******************************************************************************
//*******************************************************************************

//     Logical
//     Pin number
//     to use in
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2         6,BIT0     /* P 6.0  J1-2 on MSP432 ARM */
#define  Pin3         3,BIT2     /* P 3.2  J1-3  */
#define  Pin4         3,BIT3     /* P 3.3  J1-4  */
#define  Pin5         4,BIT1     /* P 4.1  J1-5  */
#define  Pin6         4,BIT3     /* P 4.3  J1-6  */
#define  Pin7         1,BIT5     /* P 1.5  J1-7  */
#define  Pin8         4,BIT6     /* P 4.6  J1-8  */
#define  Pin9         6,BIT5     /* P 6.5  J1-9  */
#define  Pin10        6,BIT4     /* P 6.4  J1-10 */

#define  Pin11        3,BIT6     /* P 3.6  J2-10 */
#define  Pin12        5,BIT2     /* P 5.2  J2-9  */
#define  Pin13        5,BIT0     /* P 5.0  J2-8  */
#define  Pin14        1,BIT7     /* P 1.7  J2-7  */
#define  Pin15        1,BIT6     /* P 1.6  J2-6  */
//#define  Pin16      0,BIT0     /* no pin J2-5 RESET */
#define  Pin17        5,BIT7     /* P 5.7  J2-4  */
#define  Pin18        3,BIT0     /* P 3.0  J2-3  */
#define  Pin19        2,BIT5     /* P 2.5  J2-2  */
                        // note: pin  20 is Gnd (top upper right)

                        // note: pins 21 and 22 are VBUS and Gnd (top upper left)
#define  Pin23        GPIO_PORTD_BASE,BIT0     /* PD_0  */
#define  Pin23        6,BIT1     /* P 6.1  J3-3  */
#define  Pin24        4,BIT0     /* P 4.0  J3-4  */
#define  Pin25        4,BIT2     /* P 4.2  J3-5  */
#define  Pin26        4,BIT4     /* P 4.4  J3-6  */
#define  Pin27        4,BIT5     /* P 4.5  J3-7  */
#define  Pin28        4,BIT7     /* P 4.7  J3-8  */
#define  Pin29        5,BIT4     /* P 5.4  J3-9  */
#define  Pin30        5,BIT5     /* P 5.5  J3-10 */

#define  Pin31        3,BIT7     /* P 3.7  J4-10 */
#define  Pin32        3,BIT5     /* P 3.5  J4-9  */
#define  Pin33        5,BIT1     /* P 5.1  J4-8  */
#define  Pin34        2,BIT3     /* P 2.3  J4-7  */
#define  Pin35        6,BIT7     /* P 6.7  J4-6  */
#define  Pin36        6,BIT6     /* P 6.6  J4-5  */
#define  Pin37        5,BIT6     /* P 5.6  J4-4  */
#define  Pin38        2,BIT4     /* P 2.4  J4-3  */
#define  Pin39        2,BIT6     /* P 2.6  J4-2  */
#define  Pin40        2,BIT7     /* P 2.7  J4-1  */


#define  LED1         1,BIT0       /* P 1.0     */
#define  LED2         2,BIT0       /* P 2.0 Red */
#define  LED3         2,BIT1       /* P 2.1 Grn */
#define  LED4         2,BIT3       /* P 2.2 Blue*/


#ifndef ADC0_BASE
                  // TEMPORARY TEST HACK  MSP432_ARM
#define  ADC0_BASE      0
#endif

//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                     /*     associated   LP   Grove */
                                     /*     Energia Pin  Pin  Conn  */
#define  Adc0       ADC14INCH_0      /* P 5.5    30     J3-10  -    */
#define  Adc1       ADC14INCH_1      /* P 5.4    29     J3-9   -    */
#define  Adc3       ADC14INCH_3      /* P 5.2    12     J2-9    -   */
#define  Adc4       ADC14INCH_4      /* P 5.1    33     J4-8    -   */
#define  Adc5       ADC14INCH_5      /* P 5.0    13     J2-8    -   */
#define  Adc6       ADC14INCH_6      /* P 4.7    28     J3-8    -   */
#define  Adc7       ADC14INCH_7      /* P 4.6     8     J1-8    -   */
#define  Adc8       ADC14INCH_8      /* P 4.5    27     J3-7   J9   */
#define  Adc9       ADC14INCH_9      /* P 4.4    26     J3-6   J8   */
#define  Adc10      ADC14INCH_10     /* P 4.3     6     J1-6    -   */
#define  Adc11      ADC14INCH_11     /* P 4.2    25     J3-5   J7   */
#define  Adc12      ADC14INCH_12     /* P 4.1     5     J1-5    -   */
#define  Adc13      ADC14INCH_13     /* P 4.0    24     J3-4   J6   */
#define  Adc14      ADC14INCH_14     /* P 6.1    23     J3-3   J5   */
#define  Adc15      ADC14INCH_15     /* P 6.0     2     J1-2    -   */ // IS ACTUALLY ADC14INCH_16 !!  (A16)
#define  Adc16      ADC14INCH_16     /* internal Temperature Sensor */
#define  Adc17      ADC14INCH_17     /* internal Battery Monitor    */


            //-----------------------------------------
            //           ADC Trigger Values
            //-----------------------------------------
            //           values for MSP432
#define  ADC_TRIGGER_TIMER_0_CC1  0x0001  /* Timer TA0 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_0_CC2  0x0002  /* Timer TA0 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0003  /* Timer TA1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0004  /* Timer TA1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC1  0x0005  /* Timer TA2 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0006  /* Timer TA2 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC1  0x0007  /* Timer TA3 CCR 1 match is trigger source */


//  I2C
// -----
/*     Module 1:  UCB0          Channel 1  CCR1   P 2.4    38     J4-3        */
#define  I2C_MODULE_1    1                      /* UCB0 with CCRs 1,2,3,4     */
#define  I2C_MODULE_2    2                      /* UCB1 with CCRs 1,2,3,4  GROVE    */


//  PWM
// -----                                                               Grove
/*     Module 1:  TA0           Channel 1  CCR1   P 2.4   Pin38   J4-3  J14 */
/*                              Channel 2  CCR2   P 2.5   Pin19   J2-2      */
/*                              Channel 3  CCR3   P 2.6   Pin39   J4-2  J13 */
/*                              Channel 4  CCR4   P 2.7   Pin40   J4-1      */
/*
/*     Module 2:  TA2           Channel 1  CCR1   P 5.6   Pin37   J4-4  J15 */
/*                              Channel 2  CCR2   P 5.7   Pin17   J2-4      */
/*                              Channel 3  CCR3   P 6.6   Pin36   J4-5  J16 */
/*                              Channel 4  CCR4   P 6.7   Pin35   J4-6  J17 */
#define  PWM_MODULE_0  TIMER_0               /* TA0   with CCRs 1,2,3,4  LP */
#define  PWM_MODULE_1  TIMER_1               /* TA1   with CCRs 1,2,3,4     */
#define  PWM_MODULE_2  TIMER_2               /* TA2   with CCRs 1,2,3,4  LP */
#define  PWM_MODULE_3  TIMER_3               /* TA3   with CCRs 1,2,3,4     */

                                /* PWM_MODULE_1  |  PWM_MODULE_2 */
                                /* ------------  |  ------------ */
#define  PWM_CHANNEL_1   1      /* TA0.1  P 2.4 Pin38 |  TA2.1  P 5.6 Pin37 */
#define  PWM_CHANNEL_2   2      /* TA0.2  P 2.5 Pin19 |  TA2.2  P 5.7 Pin17 */
#define  PWM_CHANNEL_3   3      /* TA0.3  P 2.6 Pin39 |  TA2.3  P 6.6 Pin36 */
#define  PWM_CHANNEL_4   4      /* TA0.4  P 2.7 Pin40 |  TA2.4  P 6.7 Pin35 */


//**************************************************************************
//**************************************************************************
//                  STANDARD   ADD-ON  SHIELD   Definitions
//**************************************************************************
//**************************************************************************
       //--------------------------------------------------------------
       //         GROVE  Boosterpack   Base   Board   Connectors
       //--------------------------------------------------------------
#define  Grove_J5   ADC14INCH_14   /* P 6.1  Pin23    J3-3   J5   */
#define  Grove_J6   ADC14INCH_13   /* P 4.0  Pin24    J3-4   J6   */
#define  Grove_J7   ADC14INCH_11   /* P 4.2  Pin25    J3-5   J7   */
#define  Grove_J8   ADC14INCH_9    /* P 4.4  Pin26    J3-6   J8   */
#define  Grove_J9   ADC14INCH_8    /* P 4.5  Pin27    J3-7   J9   */

#define  Grove_J13  Pin39            /* P 2.6  Pin39    J4-2   J13  */
#define  Grove_J14  Pin38            /* P 2.4  Pin38    J4-3   J14  */
#define  Grove_J15  Pin37            /* P 5.6  Pin37    J4-4   J15  */
#define  Grove_J16  Pin36            /* P 6.6  Pin36    J4-5   J16  */
#define  Grove_J17  Pin35            /* P 6.7  Pin35    J4-6   J17  */


       //--------------------------------------------------------------
       //         DACxxxxxx  Boosterpack      external  DAC
       //--------------------------------------------------------------


//*******************************************************************************
//*******************************************************************************
//                  LOW LEVEL MAPPING OF GPIO PIN calls on MSP432 ARM
//*******************************************************************************
//*******************************************************************************

inline void   inline_pin_High (uint32_t gpio_port_id,  unsigned long pin)
{
    GPIO_setOutputHighOnPin (gpio_port_id,pin);
}

inline void   inline_pin_Low (uint32_t gpio_port_id,  unsigned long pin)
{
    GPIO_setOutputLowOnPin (gpio_port_id,pin);
}
inline void   inline_pin_Toggle (uint32_t gpio_port_id,  unsigned long pin)
{
    GPIO_toggleOutputOnPin (gpio_port_id,pin);
}

inline int   inline_pin_Read (uint32_t gpio_port_id,  unsigned long pin)
{
      return (GPIO_getInputPinValue (gpio_port_id,pin));
}

inline int   inline_pins_Read (uint16_t gpio_port_id,  uint16_t pins)
{
    return (GPIO_getInputPinValue(gpio_port_id, pins));  // reads one or more pins
}

inline void  inline_set_pins_Output (uint16_t gpio_port_id,  uint16_t pins)
{
    GPIO_setAsOutputPin (gpio_port_id, pins);   // set one or more pins on port to output
}

inline void  inline_set_pins_Input (uint16_t gpio_port_id,  uint16_t pins)
{
    GPIO_setAsInputPin (gpio_port_id, pins);   // set one or more pins on port to input
}

#endif                             //  __USER_BOARD_API_H__
