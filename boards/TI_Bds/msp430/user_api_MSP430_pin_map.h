/********1*********2*********3*********4*********5*********6*********7**********
*
*                             user_api_msp430_pin_map.h
*
*  Provide a higher level user API for newbies.
*  More experienced programmers can directly use the board_xxx calls
*  or the platform's DriverLib APIs.
*
*  See user_api.h in ~/boards/TI_Bds directory for more information
*
*  This provides the specific pin mapping for the Tiva 123G Launchpad.
*
* History:
*   07/11/15 - Created.  Duquaine
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

#ifndef  __USER_API_BOARD_H__
#define  __USER_API_BOARD_H__


//*****************************************************************************
//*****************************************************************************
//                    LOGICAL  PIN  and  CHANNEL  MAPPINGS
//
// These supply an index to a port table, that contains the all the addresses
// used with the port, e.g.
//
//  index    Port Address entries
//    0    { P0OUT, P0IN, P0DIR, P0SEL0, P0SEL1. P0REN, P0IE, P0IFG, P0IES  }
//    1    { P1OUT, P1IN, P1DIR, P1SEL0, P1SEL1. P1REN, P1IE, P1IFG, P1IES  }
//    2    { P2OUT, P2IN, P2DIR, P2SEL0, P2SEL1. P2REN, P2IE, P2IFG, P2IES  }
//   ...       ...   ...    ...    ...     ...     ...  
//    6    { P6OUT, P6IN, P6DIR, P6SEL0, P6SEL1. P6REN, P6IE, P6IFG, P6IES }
//
// We then use the index to lookup the actual address from the above table.
//******************************************************************************
//******************************************************************************


#if defined(__MSP430F5529__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//               MSP430 - F5529    Launchpad    GPIO/ADC/PWM  mappings
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//     Logical
//     Pin number
//     to use in
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2         6,BIT5     /* P 6.5 top upper left J1 - on MSP430 F5529 */
#define  Pin3         3,BIT3     /* P 3.4  */
#define  Pin4         3,BIT3     /* P 3.3  */
#define  Pin5         1,BIT6     /* P 1.6  */
#define  Pin6         6,BIT6     /* P 6.6  */
#define  Pin7         3,BIT2     /* P 3.2  */
#define  Pin8         2,BIT7     /* P 2.7  */
#define  Pin9         4,BIT2     /* P 4.2  */
#define  Pin10        4,BIT1     /* P 4.1  */

#define  Pin11        3,BIT1     /* P 8.1  bottom lower right J2 */
#define  Pin12        5,BIT2     /* P 2.3  */
#define  Pin13        5,BIT6     /* P 2.6  */
#define  Pin14        1,BIT1     /* P 3.1  */
#define  Pin15        1,BIT0     /* P 3.0  */
//#define  Pin16      0,BIT0     /* no pin RESET */
#define  Pin17        5,BIT4     /* P 7.4  */
#define  Pin18        3,BIT2     /* P 2.2  */
#define  Pin19        2,BIT0     /* P 2.0  */
                        // note: pin  20 is Gnd (top upper right J2)

                        // note: pins 21 and 22 are VBUS and Gnd - top of J3
#define  Pin23        6,BIT0     /* P 6.0  */
#define  Pin24        6,BIT1     /* P 6.1  */
#define  Pin25        6,BIT2     /* P 6.2  */
#define  Pin26        6,BIT3     /* P 6.3  */
#define  Pin27        6,BIT4     /* P 6.4  */
#define  Pin28        7,BIT0     /* P 7.0  */
#define  Pin29        3,BIT6     /* P 3.6  */
#define  Pin30        3,BIT5     /* P 3.5  */

#define  Pin31        8,BIT1     /* P 8.2  bottom of J4 connector */
#define  Pin32        3,BIT7     /* P 3.7  */
#define  Pin33        4,BIT0     /* P 4.0  */
#define  Pin34        4,BIT3     /* P 4.3  */
#define  Pin35        1,BIT2     /* P 1.2  */
#define  Pin36        1,BIT3     /* P 1.3  */
#define  Pin37        1,BIT4     /* P 1.4  */
#define  Pin38        1,BIT5     /* P 1.5  */
#define  Pin39        2,BIT4     /* P 2.4  */
#define  Pin40        2,BIT5     /* P 2.5  top of J4 connector */

#define  LED1         1,BIT0     /* P 1.0 */
#define  LED2         4,BIT7     /* P 4.7 */
                                 // Grove  MCU    LP    Energia
#define  MAG_READ_SW1     1,BIT5   /* J15  P 1.5  J4-3   En 38 */
#define  FIXED_SLIDE_SW1  1,BIT3   /* J17  P 1.3  J4-5   En 36 */

//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                     /*     associated   LP   Grove */
                                     /*     Energia Pin  Pin  Conn  */
#define  Adc0       ADC12INCH_0      /* P 6.0    23     J3-3   J5   */
#define  Adc1       ADC12INCH_1      /* P 6.1    24     J3-4   J6   */
#define  Adc2       ADC12INCH_2      /* P 6.2    25     J3-5   J7   */
#define  Adc3       ADC12INCH_3      /* P 6.3    26     J3-6   J8   */
#define  Adc4       ADC12INCH_4      /* P 6.4    27     J3-7   J9   */
#define  Adc5       ADC12INCH_5      /* P 6.5     2     J1-2    -   */
#define  Adc6       ADC12INCH_6      /* P 6.6     6     J1-6    -   */
#define  Adc12      ADC12INCH_12     /* P 7.0    28     J3-8    -   */

#define  GROVE_J5   ADC12INCH_0      /* P 6.0    23     J3-3   J5   */
#define  GROVE_J6   ADC12INCH_1      /* P 6.1    24     J3-4   J6   */
#define  GROVE_J7   ADC12INCH_2      /* P 6.2    25     J3-5   J7   */
#define  GROVE_J8   ADC12INCH_3      /* P 6.3    26     J3-6   J8   */
#define  GROVE_J9   ADC12INCH_4      /* P 6.4    27     J3-7   J9   */


//  PWM
// -----
/*     Module 0:  TA0           Channel 1  CCR1   P 1.2    35     J4-6       */
/*                              Channel 2  CCR2   P 1.3    36     J4-5       */
/*                              Channel 3  CCR3   P 1.4    37     J4-4       */
/*                              Channel 4  CCR4   P 1.5    38     J4-3       */
/*     Module 1:  TA1           Channel 1  CCR1   P 2.0    19     J2-1       */
/*     Module 2:  TA2           Channel 1  CCR1   P 2.4    39     J4-2       */
/*                              Channel 2  CCR2   P 2.5    40     J4-1       */
#define  PWM_MODULE_0   0                   /* TA0   with CCRs 1,2,3,4       */
#define  PWM_MODULE_1   1                   /* TA1   with CCR  1             */
#define  PWM_MODULE_2   2                   /* TA2   with CCRs 1,2           */

#define  PWM_CHANNEL_1  1                   /* TA0.1  P 1.2  |  TA2.1  P 2.4 */
#define  PWM_CHANNEL_2  2                   /* TA0.2  P 1.3  |  TA2.2  P 2.5 */
#define  PWM_CHANNEL_3  3                   /* TA0.3  P 1.4  |               */
#define  PWM_CHANNEL_4  4                   /* TA0.4  P 1.5  |               */

#endif                          // (__MSP430F5529__)



#if defined(__MSP430FR6989__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                MSP430 - FR6989    Launchpad    GPIO/ADC/PWM  mappings
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//     Logical
//    Pin number
//    to use in      -
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2         4,BIT2     /* P 4.2 top upper left J1 - on MSP430 FR5969*/
#define  Pin3         2,BIT6     /* P 2.6  J1-3 */      //  -- NEEDS REMAP - was copied from FR5969 --
#define  Pin4         2,BIT5     /* P 2.5  J1-4 */
#define  Pin5         4,BIT3     /* P 4.3  J1-5 */
#define  Pin6         2,BIT4     /* P 2.4  J1-6 */
#define  Pin7         2,BIT2     /* P 2.2  J1-7 */
#define  Pin8         3,BIT4     /* P 3.4  J1-8 */
#define  Pin9         3,BIT5     /* P 3.5  J1-9 */
#define  Pin10        3,BIT6     /* P 3.6  J1-10*/

#define  Pin11        1,BIT3     /* P 1.3  bottom lower right J2-10 */
#define  Pin12        1,BIT4     /* P 1.4  J2-9 */
#define  Pin13        1,BIT5     /* P 1.5  J2-8 */
#define  Pin14        1,BIT7     /* P 1.7  J2-7 */
#define  Pin15        1,BIT6     /* P 1.6  J2-6 */
//#define  Pin16      0,BIT0     /* no pin RESET */
#define  Pin17        0,BIT0     /*  n/c   J2-4 */
#define  Pin18        3,BIT0     /* P 3.0  J2-3 */
#define  Pin19        1,BIT2     /* P 1.2  J2-2 */
                        // note: pin  20 is Gnd (top upper right J2-1)

#define  LED1         1,BIT0     /* P 1.0  */
#define  LED2         9,BIT7     /* P 9.7  */


//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                           /*       associated  LP   Grove */
                                           /*          Energia  Pin  Conn  */
                                           /* FR6989     Pin   Conn        */
                                           /* ------      --   ----   --   */
                                           /*   -         23   J3-3   J5   */
                                           /*   -         24   J3-4   J6   */
                                           /*   -         25   J3-5   J7   */
                                           /*   -         26   J3-6   J8   */
                                           /*   -         27   J3-7   J9   */

                        /* AD Ch   Pin   Energia   LP Conn   Grove  DRV Use */
#define  A2     0       /*  10     P9.2     2        J1-2           */
#define  A6     1       /*  11     P9.3     6        J1-6           */
#define  A17    2       /*  12     P9.4    17        J2-4           */
#define  A23    3       /*   7     P8.4    23        J3-3           */
#define  A24    4       /*   6     P8.5    24        J3-4           */
#define  A25    5       /*   5     P8.6    25        J3-5           */
#define  A26    6       /*   4     P8.7    26        J3-6           */
#define  A27    7       /*   8     P9.0    27        J3-7           */
#define  A28    8       /*   9     P9.1    28        J3-8           */
#define  A29    9       /*  13     P9.5    29        J3-9           */
#define  A30   10       /*  14     P9.6    30        J3-10          */
#define  A34   11       /*   3     P1.3    34        J4-7           */

/*#define  P7           /* TA1.0   P1.4     7        J1-7           */
#define  P8             /* TB0.6   P2.0     8        J1-8  - dup -  */
#define  P11            /* TA1.2   P4.7    11        J2-10 - dup -  */
#define  P12            /* TB0.3   P2.4    12        J2-9  - dup -  */
#define  P13            /* TB0.4   P2.5    13        J2-8  - dup -  */
#define  P14            /* TA0.2   P1.7    14        J2-7           */
#define  P15            /* TA0.1   P1.6    15        J2-6           */
/*#define  P18          /* TA0.0   P1.5    18        J2-3           */
#define  P19            /* TB0.5   P2.1    19        J2-2           */
#define  P34            /* TA1.2   P1.3    34        J4-7  -     -  */
#define  P35            /* TB0.4   P2.2    35        J4-6           */
#define  P36            /* TB0.3   P3.7    36        J4-5           */
#define  P37            /* TB0.2   P3.6    37        J4-4           */
#define  P38            /* TA1.1   P3.3    38        J4-3           */
#define  P39            /* TB0.5   P2.6    39        J4-2           */
#define  P40            /* TB0.6   P2.7    40        J4-1           */

              //----------------------------------------
              // GROVE   Base Boosterpack   Pin Mapping
              //----------------------------------------
                //  ANALOG  I/O   Grove  Energia  LP    MCU Pin ADC
#define  GROVE_J5    A23       /*  J5     Pin23   J3-3   P 8.4   ADC12INCH_7 */
#define  GROVE_J6    A24       /*  J6     Pin24   J4-4   P 8.5   ADC12INCH_6 */
#define  GROVE_J7    A25       /*  J7     Pin25   J4-5   P 8.6   ADC12INCH_5 */
#define  GROVE_J8    A26       /*  J8     Pin26   J4-6   P 8.6   ADC12INCH_4 */
#define  GROVE_J9    A27       /*  J9     Pin27   J4-7   P 9.0   ADC12INCH_8 */

                //  DIGITAL I/O   Grove  Energia  LP    MCU Pin
#define  GROVE_J13   2,BIT7    /*  J13    Pin40   J4-1   P 2.7  */
#define  GROVE_J14   2,BIT6    /*  J14    Pin39   J4-2   P 2.6  */
#define  GROVE_J15   3,BIT3    /*  J15    Pin38   J4-3   P 3.3  */
#define  GROVE_J16   3,BIT6    /*  J16    Pin37   J4-4   P 3.6  */
#define  GROVE_J17   3,BIT7    /*  J17    Pin36   J4-5   P 3.7  */

                           // GPIO   ADC 12 Chan   Energia   Launchpd  Grove
#define  GROVE_J5   A23    /* P 8.4  ADC12INCH_7    A23 23     J3-3     J5 */
#define  GROVE_J6   A24    /* P 8.5  ADC12INCH_6    A24 24     J3-4     J6 */
#define  GROVE_J7   A25    /* P 8.6  ADC12INCH_5    A25 25     J3-5     J7 */
#define  GROVE_J8   A26    /* P 8.7  ADC12INCH_4    A26 26     J3-6     J8 */
#define  GROVE_J9   A27    /* P 9.0  ADC12INCH_8    A27 27     J3-7     J9 */


            //-----------------------------------------
            //           ADC Trigger Values
            //-----------------------------------------
            //         values for MSP430-FR6989
#define  ADC_TRIGGER_TIMER_0_CC1  0x0001  /* Timer TA0 CCR 1 match is trigger source TA0/CCR1 */
#define  ADC_TRIGGER_TIMER_4_CC0  0x0002  /* Timer TB0 CCR 0 match is trigger source TB0/CCR0 */
#define  ADC_TRIGGER_TIMER_4_CC0  0x0003  /* Timer TB0 CCR 1 match is trigger source TB0/CCR1 */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0004  /* Timer TA1 CCR 2 match is trigger source TA1/CCR1 */
#define  ADC_TRIGGER_TIMER_2_CC1  0x0005  /* Timer TA2 CCR 1 match is trigger source TA2/CCR1 */
#define  ADC_TRIGGER_TIMER_3_CC1  0x0006  /* Timer TA3 CCR 1 match is trigger source TA3/CCR1 */


//  PWM
// -----                            -- need to re-work pins - copied from FR5969
/*     Module 0:  TA0           Channel 1  CCR1   P 1.2   Pin19   J2-2        */
/*                              Channel 2  CCR2   P 1.3   Pin11   J2-10       */
/*     Module 1:  TA1           Channel 1  CCR1   P 1.2   Pin19   J2-2    ?   */
/*                              Channel 2  CCR2   P 1.3   Pin11   J2-10   ?   */
/*     Module 4:  TB0           Channel 2  CCR2   P 1.5   Pin13   J2-8        */
/*                              Channel 3  CCR3   P 3.4   Pin8    J1-8        */
/*                              Channel 4  CCR4   P 3.5   Pin9    J1-9        */
/*                              Channel 5  CCR5   P 3.6   Pin10   J1-10       */
/*                              Channel 6  CCR1   P 1.4   Pin12   J2-9        */
/*                                                                            */
/*    CAUTION: several TB0.x pins overlap with the SPI pins, so we do _NOT_   */
/*             hook them up (P1.6, P1.7, P2.2).                               */
/*             So many projects use SPI (CC3100, DRV8xxx, ...) that it would  */
/*             be too easy to shoot oneself in the foot, so we don't use them.*/
/*             Ditto for TA0.1 and TA1.0  (MOSI/MISO conflicts)               */

#define  PWM_MODULE_0   0                   /* TA0   with CCRs 1,2   [0,3]    */
#define  PWM_MODULE_1   1                   /* TA1   with CCRs 1,2   [0,3]    */
#define  PWM_MODULE_2   2                   /* TA2      TIMER ONLY  no GPIOs  */
#define  PWM_MODULE_3   3                   /* TA3      TIMER ONLY  no GPIOs  */
#define  PWM_MODULE_4   4                   /* TB0   with CCRs [0,1],2,3,4,5,6 */
                                              // -- needs work copied from FR5969
                              /* PWM_MODULE_0  | PWM_MODULE_1  |  PWM_MODULE_4 */
                              /* ------------- | ------------- |  ------------ */
#define  PWM_CHANNEL_1  1     /* TA0.1         | TA1.1  P 1.2  |-not pinned out-*/
#define  PWM_CHANNEL_2  2     /* TA0.2         | TA1.2  P 1.3  |  TB0.2  P 1.5 */
#define  PWM_CHANNEL_3  3     /*      -        |      -        |  TB0.3  P 3.4 */
#define  PWM_CHANNEL_4  4     /*      -        |      -        |  TB0.4  P 3.5 */
#define  PWM_CHANNEL_5  5     /*      -        |      -        |  TB0.5  P 3.6 */
#define  PWM_CHANNEL_6  6     /*      -        |      -        |  TB0.6  P 3.6 */

#endif                          // defined(__MSP430FR6989__)



#if defined(__MSP430FR5969__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                MSP430 - FR5969    Launchpad    GPIO/ADC/PWM  mappings
//
// CAUTION:  as per the Grove Launchpad documentation, Grove
//           ADC connectors J5-J9 only connect to the pins on the
//           Launchpad inner connector J3  (logical pins 23-27).
//
//           So on "single-row devices like the FR5969,
//           you need to add female-to-female jumpers, that will
//           jumper from the Grove J3 connector pins 23-27 over to the
//           Launchpad J2 or J1 connectors.
//
//           In this example, I have jumpered them over to the
//           FR5969 ADC Channels A10, A7, A3, A4, and A5,  which map to
//           the Launchpad J1-2, J1-6, J2-10, J2-9, and J2-8
//           connector pins respectively.
//
//           You may choose a different mapping for your application,
//           by modifing the GROVE_J5 - Grove_J9 definitions below.
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//     Logical
//    Pin number
//    to use in
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2         4,BIT2     /* P 4.2 top upper left J1 - on MSP430 FR5969*/
#define  Pin3         2,BIT6     /* P 2.6  J1-3 */
#define  Pin4         2,BIT5     /* P 2.5  J1-4 */
#define  Pin5         4,BIT3     /* P 4.3  J1-5 */
#define  Pin6         2,BIT4     /* P 2.4  J1-6 */
#define  Pin7         2,BIT2     /* P 2.2  J1-7 */
#define  Pin8         3,BIT4     /* P 3.4  J1-8 */
#define  Pin9         3,BIT5     /* P 3.5  J1-9 */
#define  Pin10        3,BIT6     /* P 3.6  J1-10*/

#define  Pin11        1,BIT3     /* P 1.3  bottom lower right J2-10 */
#define  Pin12        1,BIT4     /* P 1.4  J2-9 */
#define  Pin13        1,BIT5     /* P 1.5  J2-8 */
#define  Pin14        1,BIT7     /* P 1.7  J2-7 */
#define  Pin15        1,BIT6     /* P 1.6  J2-6 */
//#define  Pin16      0,BIT0     /* no pin RESET */
#define  Pin17        0,BIT0     /*  n/c   J2-4 */
#define  Pin18        3,BIT0     /* P 3.0  J2-3 */
#define  Pin19        1,BIT2     /* P 1.2  J2-2 */
                        // note: pin  20 is Gnd (top upper right J2-1)


#define  LED1         4,BIT6     /* P 4.6  */
#define  LED2         1,BIT0     /* P 1.0  */

                                  // ALL GROVE CONNECTS MUST BE RE-MAPPED ON FR5969
                                  // Grove  MCU    LP   REMAP  Energia
#define  MAG_READ_SW1     2,BIT6    /* J15  P 2.6  J4-3 -> J1-3    En 3 */
#define  FIXED_SLIDE_SW1  4,BIT3    /* J17  P 4.3  J4-5 -> J1-5    En 5 */


//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                           /*       associated  LP   Grove */
                                           /*          Energia  Pin  Conn  */
                                           /* FR5969     Pin   Conn        */
                                           /* ------      --   ----   --   */
                                           /*   -         23   J3-3   J5   */
                                           /*   -         24   J3-4   J6   */
                                           /*   -         25   J3-5   J7   */
                                           /*   -         26   J3-6   J8   */
                                           /*   -         27   J3-7   J9   */

                             /*     Energia Launchpad conn */
                             /*        Pin   J1/J2 */
                             /*        ---   ----- */
#define  Adc2        0       /* P 1.2 ADC12INCH_2  19   J2-2  */
#define  Adc3        1       /* P 1.3 ADC12INCH_3  11   J2-10 */
#define  Adc4        2       /* P 1.4 ADC12INCH_4  12   J2-9  */
#define  Adc5        3       /* P 1.5 ADC12INCH_5  13   J2-8  */
#define  Adc7        4       /* P 2.4 ADC12INCH_7   6   J1-6  */
#define  Adc10       5       /* P 4.2 ADC12INCH_10  2   J1-2  */
#define  Adc11       6       /* P 4.3 ADC12INCH_11  5   J1-5  */
#define  Adc12       7       /* P 3.0 ADC12INCH_12 18   J2-3  */

                             // ALL GROVE CONNECTS MUST BE RE-MAPPED ON FR5969
                             // GPIO  ADC 12 Chan    J1/J2    Jumpers     Grove
#define  GROVE_J5   Adc10    /* P 4.2 ADC12INCH_10  A2   2   J3-3 -> J1-2   J5 */
#define  GROVE_J6   Adc7     /* P 2.4 ADC12INCH_7   A7   6   J3-4 -> J1-6   J6 */
#define  GROVE_J7   Adc3     /* P 1.3 ADC12INCH_3  A11  11   J3-5 -> J2-10  J7 */
#define  GROVE_J8   Adc4     /* P 1.4 ADC12INCH_4  A12  12   J3-6 -> J2-9   J8 */
#define  GROVE_J9   Adc5     /* P 1.5 ADC12INCH_5  A13  13   J3-7 -> J2-8   J9 */


//  PWM
// -----
/*     Module 1:  TA1           Channel 1  CCR1   P 1.2    19     J2-2        */
/*                              Channel 2  CCR2   P 1.3    11     J2-10       */
/*     Module 4:  TB0           Channel 1  CCR1   P 1.4    12     J2-9        */
/*                              Channel 2  CCR2   P 1.5    13     J2-8        */
/*                              Channel 3  CCR3   P 3.4     8     J1-8        */
/*                              Channel 4  CCR4   P 3.5     9     J1-9        */
/*                              Channel 5  CCR5   P 3.6    10     J1-10       */
/*                                                                            */
/*    CAUTION: several TB0.x pins overlap with the SPI pins, so we do _NOT_   */
/*             hook them up (P1.6, P1.7, P2.2).                               */
/*             So many projects use SPI (CC3100, DRV8xxx, ...) that it would  */
/*             be too easy to shoot oneself in the foot, so we don't use them.*/
/*             Ditto for TA0.1 and TA1.0  (MOSI/MISO conflicts)               */
  
#define  PWM_MODULE_1   1                    /* TA1   with CCRs 1,2         */
#define  PWM_MODULE_4   4                    /* TB0   with CCRs 1,2,3,4,5     */

                                             /* PWM_MODULE_1  |  PWM_MODULE_4 */
                                             /* ------------- |  ------------ */
#define  PWM_CHANNEL_1  1                    /* TA1.1  P 1.2  |  TB0.1  P 1.4 */
#define  PWM_CHANNEL_2  2                    /* TA1.2  P 1.3  |  TB0.2  P 1.5 */
#define  PWM_CHANNEL_3  3                    /*               |  TB0.3  P 3.4 */
#define  PWM_CHANNEL_4  4                    /*               |  TB0.4  P 3.5 */
#define  PWM_CHANNEL_5  5                    /*               |  TB0.5  P 3.6 */

#endif                          // defined(__MSP430FR5969__)



#if defined(__MSP430FR4133__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                MSP430 - FR4133    Launchpad    GPIO/ADC/PWM  mappings
//
// CAUTION:  as per the Grove Launchpad documentation, Grove
//           ADC connectors J5-J9 only connect to the pins on the
//           Launchpad inner connector J3  (logical pins 23-27).
//
//           So on "single-row devices like the FR4133,
//           you need to add female-to-female jumpers, that will
//           jumper from the Grove J3 connector pins 23-27 over to the
//           Launchpad J2 or J1 connectors.
//
//           In this example, I have jumpered them over to the
//           FR5969 ADC Channels A2, A3, A4, A5, and A10 which map to
//           the Launchpad J2-2, J2-10, J2-9, J2-8, and J1-2
//           connector pins respectively.
//
//           You may choose a different mapping for your application,
//           by modifing the GROVE_J5 - Grove_J9 definitions below.
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//     Logical
//    Pin number
//    to use in
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2         8,BIT1     /* P 8.1 top upper left J1 - on MSP430 FR4133*/
#define  Pin3         1,BIT1     /* P 1.1  J1-3 */
#define  Pin4         1,BIT0     /* P 1.0  J1-4 */
#define  Pin5         2,BIT7     /* P 2.7  J1-5 */
#define  Pin6         8,BIT0     /* P 8.0  J1-6 */
#define  Pin7         5,BIT1     /* P 5.1  J1-7 */
#define  Pin8         2,BIT5     /* P 2.5  J1-8 */
#define  Pin9         8,BIT2     /* P 8.2  J1-9 */
#define  Pin10        8,BIT3     /* P 8.3  J1-10*/

#define  Pin11        1,BIT5     /* P 1.5  bottom lower right J2-10 */
#define  Pin12        1,BIT4     /* P 1.4  J2-9 */
#define  Pin13        1,BIT3     /* P 1.3  J2-8 */
#define  Pin14        5,BIT3     /* P 5.3  J2-7 */
#define  Pin15        5,BIT2     /* P 5.2  J2-6 */
//#define  Pin16      0,BIT0     /* no pin RESET */
#define  Pin17        5,BIT0     /* P 5.0  J2-4 */
#define  Pin18        1,BIT6     /* P 1.6  J2-3 */
#define  Pin19        1,BIT7     /* P 1.7  J2-2 */
                        // note: pin  20 is Gnd (top upper right J2-1)


#define  LED1         1,BIT0     /* P 1.0  ->  CLOBBERS  UART TX */
#define  LED2         4,BIT0     /* P 4.0          */

                                  // ALL GROVE CONNECTS MUST BE RE-MAPPED ON FR5969
                                  // Grove  MCU    LP   REMAP  Energia
#define  MAG_READ_SW1     1,BIT1    /* J15  P 1.1  J4-3 -> J1-3    En 3 */
#define  FIXED_SLIDE_SW1  2,BIT7    /* J17  P 2.7  J4-5 -> J1-5    En 5 */


//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                           /*       associated  LP   Grove */
                                           /*          Energia  Pin  Conn  */
                                           /* FR4133     Pin   Conn        */
                                           /* ------     ---   ----   --   */
                                           /*   -         23   J3-3   J5   */
                                           /*   -         24   J3-4   J6   */
                                           /*   -         25   J3-5   J7   */
                                           /*   -         26   J3-6   J8   */
                                           /*   -         27   J3-7   J9   */

                             /*               Energia Launchpad conn */
                             /*                  Pin   J1/J2 */
                             /*                  ---   ----- */
#define  Adc0        0       /* P 1.0  ADCINCH_0   4   J1-4  -> LED1 CONFLICT ! */
#define  Adc1        1       /* P 1.1  ADCINCH_1   3   J1-3  */
#define  Adc3        2       /* P 1.3  ADCINCH_3  13   J2-8  */
#define  Adc4        3       /* P 1.4  ADCINCH_4  12   J2-9  */
#define  Adc5        4       /* P 1.5  ADCINCH_5  11   J2-10 */
#define  Adc6        5       /* P 1.6  ADCINCH_6  18   J2-3  */
#define  Adc7        6       /* P 1.7  ADCINCH_7  19   J2-2  */
#define  Adc8        7       /* P 8.0  ADCINCH_8   6   J1-6  */
#define  Adc9        8       /* P 8.1  ADCINCH_9   2   J1-2  */

                             // ALL GROVE CONNECTS MUST BE RE-MAPPED ON FR4133
                             // GPIO  ADC 10 Chan    J1/J2    Jumpers     Grove
#define  GROVE_J5   Adc9     /* P 8.1  ADCINCH_9    A2   2   J3-3 -> J1-2   J5 */
#define  GROVE_J6   Adc8     /* P 8.0  ADCINCH_8    A6   6   J3-4 -> J1-6   J6 */
#define  GROVE_J7   Adc5     /* P 1.5  ADCINCH_5   A11  11   J3-5 -> J2-10  J7 */
#define  GROVE_J8   Adc4     /* P 1.4  ADCINCH_4   A12  12   J3-6 -> J2-9   J8 */
#define  GROVE_J9   Adc3     /* P 1.3  ADCINCH_3   A13  13   J3-7 -> J2-8   J9 */


//  PWM
// -----
/*     Module 0:  TA0           Channel 1  CCR1   P 1.7    19     J2-2        */
/*                              Channel 2  CCR2   P 1.6    18     J2-3        */
#define  PWM_MODULE_0   0                      /* TA0   with CCRs 1,2         */

#define  PWM_CHANNEL_1  1                      /* TA0.1  P 1.7                */
#define  PWM_CHANNEL_2  2                      /* TA0.2  P 1.6                */

#endif                          // defined(__MSP430FR4133__)



#if defined(__MSP430G2553__)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//              MSP430 - G2553    Launchpad    GPIO/ADC/PWM  mappings
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

#define  LED1         1,BIT0     /* P 1.0  ->  CLOBBERS  UART TX and Adc0 */
//#define  LED2         4,BIT0     /* P 4.0          */

                                  // ALL GROVE CONNECTS MUST BE RE-MAPPED ON FR5969
                                  // Grove  MCU    LP   REMAP  Energia
#define  MAG_READ_SW1     1,BIT1    /* J15  P 1.1  J4-3 -> J1-3    En 3 */
#define  FIXED_SLIDE_SW1  2,BIT7    /* J17  P 2.7  J4-5 -> J1-5    En 5 */


//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------

// ADC's listed by "Energia Pin" Number
// ------------------------------------
#define  A2        Adc_Chan_0
#define  A3        Adc_Chan_1
#define  A4        Adc_Chan_2
#define  A5        Adc_Chan_3
#define  A6        Adc_Chan_4
#define  A7        Adc_Chan_5
#define  A14       Adc_Chan_6
#define  A15       Adc_Chan_7

// ADC's listed by Channel Number                Energia  Launchpad connector
// ------------------------------                  Pin    J1/J2
                               //                   ---   -----
#define  Adc_Chan_0     0      /* P 1.0  ADCINCH_0   A2   J1-2  -> LED1 CONFLICT ! */
#define  Adc_Chan_1     1      /* P 1.1  ADCINCH_1   A3   J1-3  */
#define  Adc_Chan_2     2      /* P 1.2  ADCINCH_2   A4   J1-4  */
#define  Adc_Chan_3     3      /* P 1.3  ADCINCH_3   A5   J1-5  */
#define  Adc_Chan_4     4      /* P 1.4  ADCINCH_4   A6   J1-6  */
#define  Adc_Chan_5     5      /* P 1.5  ADCINCH_5   A7   J1-7  */
#define  Adc_Chan_6     6      /* P 1.6  ADCINCH_6  A14   J2-7  */
#define  Adc_Chan_7     7      /* P 1.7  ADCINCH_7  A15   J2-6  */

                               // ALL GROVE CONNECTS MUST BE RE-MAPPED ON F2553
                               // GPIO  ADC 10 Chan    J1/J2      Jumpers     Grove
#define  GROVE_J5  Adc_Chan_2  /* P 1.2  ADCINCH_2    A2   2   J3-3 -> J1-4 <- J5  */
#define  GROVE_J6  Adc_Chan_4  /* P 1.4  ADCINCH_4    A6   6   J3-4 -> J1-6    J6  */
#define  GROVE_J7  Adc_Chan_3  /* P 1.3  ADCINCH_3    A5   5   J3-5 -> J1-5    J7  */
#define  GROVE_J8  Adc_Chan_7  /* P 1.7  ADCINCH_7   A15  15   J3-6 -> J2-6    J8  */
#define  GROVE_J9  Adc_Chan_6  /* P 1.6  ADCINCH_6   A14  14   J3-7 -> J2-7    J9  */


// PWM's listed by "Energia Pin" Number
// ------------------------------------
#define  P0       PWM_CHANNEL_0
#define  P1       PWM_CHANNEL_1
#define  P2       PWM_CHANNEL_2
#define  P0A      PWM_CHANNEL_0A
#define  P1A      PWM_CHANNEL_1A
#define  P2A      PWM_CHANNEL_2A

//  PWM
// -----                                         Default Pins
/*     Module 1:  TA1           Channel 0  CCR1   P 2.0     8     J1-8        */
/*                              Channel 1  CCR1   P 2.1     9     J1-9        */
/*                              Channel 2  CCR2   P 2.4    12     J2-9        */
#define  PWM_MODULE_1    1                     /* TA1   with CCRs 0,1,2       */


// PWM's listed by Channel Number
// ------------------------------------
   // Note that on G2553, the TA1.0/TA1.1/TA1.2 pins can be routed out to alternate
   //      pins. If you want to route out the alternate pins then specify
   //      PWM_CHANNEL_1A and/or PWM_CHANNEL_2A to utilize the alternative pins
#define  PWM_CHANNEL_0   0                     /* TA1.0  P 2.0                */
#define  PWM_CHANNEL_1   1                     /* TA1.1  P 2.1                */
#define  PWM_CHANNEL_2   2                     /* TA1.2  P 2.4                */
#define  PWM_CHANNEL_0A  3                     /* TA1.0  P 2.3                */
#define  PWM_CHANNEL_1A  4                     /* TA1.1  P 2.2                */
#define  PWM_CHANNEL_2A  5                     /* TA1.2  P 2.5                */

                         // optional flags on board_pwm_config_channel() call
#define  PWM_STANDARD_PWM   0       /* standard PWM, CCR0 = period (OUTMOD_7) */
#define  PWM_TOGGLE_OUTPUT  1       /* quasi-PWM - toggle outputs  (OUTMOD_4) */

#endif                   // defined(__MSP430G2553__)


#if defined(__MSP430G2553__)
                   // provide defs for F2553 DriverLib emulation routines
void GPIO_setOutputHighOnPin (uint16_t gpio_port_id,  uint16_t pin);
void GPIO_setOutputLowOnPin (uint16_t gpio_port_id,  uint16_t pin);
void GPIO_toggleOutputOnPin (uint16_t gpio_port_id,  uint16_t pin);
int GPIO_getInputPinValue (uint16_t gpio_port_id,  uint16_t pin);
#endif

//******************************************************************************
//******************************************************************************
//                  LOW LEVEL MAPPING OF GPIO PIN calls on MSP430
//******************************************************************************
//******************************************************************************

// 06/02/15 - Engineers think in Ports.  DIY-ers think in pins (due to Arduino).
//            Probably need a set_logical_pin/get_logical_pin/toggle_logical_pin
//            API for DIY-ers. But make it "pin at a time" or you will go crazy
//            trying to convert each bit into different port/pin combos.
//            Too bad- you want speed - use ports !

inline void   inline_pins_High (int gpio_port_id,  unsigned long pins_mask)
{
    GPIO_setOutputHighOnPin (gpio_port_id, pins_mask);  // sets one or more pins high
}

inline void   inline_pins_Low (int gpio_port_id,   unsigned long pins)
{
    GPIO_setOutputLowOnPin (gpio_port_id, pins);   // sets one or more pins low
}

inline void   inline_pins_Toggle (int gpio_port_id,  unsigned long pins)
{
    GPIO_toggleOutputOnPin (gpio_port_id, pins);   // toggles one or more pins
}

inline int   inline_pins_Read (uint16_t gpio_port_id,  uint16_t pins)
{
    return (GPIO_getInputPinValue(gpio_port_id, pins));  // reads one or more pins
}

inline int   inline_pin_Value (uint16_t gpio_port_id,  uint16_t pins)
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

// This needs more thinking - should really be "Configure ADC Channel" that does both GPIO and ADC setup
// In that case, it can only be done "pin at a time", else gets too nuts.
inline int   inline_set_pins_Analog (uint16_t gpio_port_id,  uint16_t pins)
{
    // not such function as GPIO_setAsAnalogPin() in DriverLib !!! // set one or more pins on port to Analog
}

#endif                             //  __USER_BOARD_API_H__
