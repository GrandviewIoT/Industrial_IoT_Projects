/********1*********2*********3*********4*********5*********6*********7**********
*
*                             user_api_tiva_pin_map.h
*
*  Provide a higher level user API for newbies.
*  More experienced programmers can directly use the board_xxx calls
*  or the platform's DriverLib APIs.
*
*  See user_api.h in ~/boards/TI_Bds directory for more information
*
*  This provides the specific pin mapping for the Tiva 123G Launchpad.
*
*
* History:
*   xx/xx/xx - Created.  Duquaine
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


//*******************************************************************************
//*******************************************************************************
//                    LOGICAL  PIN  and  CHANNEL  MAPPINGS
//*******************************************************************************
//*******************************************************************************


#if defined(PART_TM4C123GH6PM)
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
//                  Tiva - 123G      Launchpad    GPIO/ADC/PWM  mappings
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//     Logical
//     Pin number
//     to use in
//    your program   Physical Port and Pin number that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
#define  Pin2       GPIO_PORTB_BASE,GPIO_PIN_5     /* PB_5  J1-2 on Tiva 123G */
#define  Pin3       GPIO_PORTB_BASE,GPIO_PIN_0     /* PB_0  J1-3  */
#define  Pin4       GPIO_PORTB_BASE,GPIO_PIN_1     /* PB_1  J1-4  */
#define  Pin5       GPIO_PORTE_BASE,GPIO_PIN_4     /* PE_4  J1-5  */
#define  Pin6       GPIO_PORTE_BASE,GPIO_PIN_5     /* PE_5  J1-6  */
#define  Pin7       GPIO_PORTB_BASE,GPIO_PIN_4     /* PB_4  J1-7  */
#define  Pin8       GPIO_PORTA_BASE,GPIO_PIN_5     /* PA_5  J1-8  */
#define  Pin9       GPIO_PORTA_BASE,GPIO_PIN_6     /* PA_6  J1-9  */
#define  Pin10      GPIO_PORTA_BASE,GPIO_PIN_7     /* PA_7  J1-10 */
#define  Pin11      GPIO_PORTA_BASE,GPIO_PIN_2     /* PA_2  J2-10 */
#define  Pin12      GPIO_PORTA_BASE,GPIO_PIN_3     /* PA_3  J2-9  */
#define  Pin13      GPIO_PORTA_BASE,GPIO_PIN_4     /* PA_4  J2-8  */
#define  Pin14      GPIO_PORTB_BASE,GPIO_PIN_6     /* PB_6  J2-7  */
#define  Pin15      GPIO_PORTB_BASE,GPIO_PIN_0     /* PB_0  J2-6  */
//#define  Pin16    GPIO_PORTB_BASE,GPIO_PIN_0     /* no pin RESET J2-5 */
#define  Pin17      GPIO_PORTF_BASE,GPIO_PIN_0     /* PF_0  J2-4  */
#define  Pin18      GPIO_PORTE_BASE,GPIO_PIN_0     /* PE_0  J2-3  */
#define  Pin19      GPIO_PORTB_BASE,GPIO_PIN_2     /* PB_2  J2-2  */
                      // note: pin 20 (J2-1) is Gnd

                      // note: pins 21 and 22 are VBUS and Gnd
#define  Pin23      GPIO_PORTD_BASE,GPIO_PIN_0     /* PD_0  J3-3  */

#define  LED1       GPIO_PORTF_BASE,GPIO_PIN_2     /* PF_2 */
#define  LED2       GPIO_PORTF_BASE,GPIO_PIN_1     /* PF_1 */

                      // Example uses Grove J15 (38/PB-3) and J17 (36/PC-6)
#define  MAG_READ_SW1    GPIO_PORTB_BASE,GPIO_PIN_3  /* J15 PB_3  J4-3  En 38 */
#define  FIXED_SLIDE_SW1 GPIO_PORTC_BASE,GPIO_PIN_5  /* J17 PC_5  J4-5  En 36 */

// Grove Experiments:  J13 - energizes pin 40  J4-1  PF_2  RGB/PWM  This was a Screwed up decision on their part !  J4 is mostly dedicated to PWMs, not GPIOs !
//                     J14 - energizes pin 39  J4-2  PF_3  RGB/PWM  ==> Go to Jumper Config and hook to J1/J2 pins instead. Quasi bread board !!! ARG
//                     J15 - energizes pin 38  J4-3  PB_3  GPIO
//                     J16 - energizes pin 37  J4-4  PC_4  GPIO/PWM
//                     J17 - energizes pin 36  J4-5  PC_5  GPIO/PWM
//inline_High (GPIO_PORTF_BASE,GPIO_PIN_2);       // J13  pin 40
//inline_High (GPIO_PORTF_BASE,GPIO_PIN_3);       // J14  pin 39
//inline_High (GPIO_PORTB_BASE,GPIO_PIN_3);       // J15  pin 38
//inline_High (GPIO_PORTC_BASE,GPIO_PIN_4);       // J16  pin 37
//inline_High (GPIO_PORTC_BASE,GPIO_PIN_5);       // J17  pin 36

//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                       /*      associated    LP   Grove  */
                                       /*      Energia Pin   Pin  Conn   */
#define  Adc0       ADC_CTL_CH0        /* PE_3  AIN0  29     J3-9   -    */
#define  Adc1       ADC_CTL_CH1        /* PE_2  AIN1  28     J3-8    -   */
#define  Adc2       ADC_CTL_CH2        /* PE_1  AIN2  27     J3-7   J9   */
#define  Adc3       ADC_CTL_CH3        /* PE_0  AIN3  18     J4-3    -   */
#define  Adc4       ADC_CTL_CH4        /* PD_3  AIN4  26     J3-6   J8   */
#define  Adc5       ADC_CTL_CH5        /* PD_2  AIN5  25     J3-5   J7   */
#define  Adc6       ADC_CTL_CH6        /* PD_1  AIN6  24     J3-4   J6   */
#define  Adc7       ADC_CTL_CH7        /* PD_0  AIN7  23     J3-3   J5   */
#define  Adc8       ADC_CTL_CH8        /* PE_5  AIN8   6     J1-6    -   */
#define  Adc9       ADC_CTL_CH9        /* PE_4  AIN9   5     J1-5    -   */
#define  Adc10      ADC_CTL_CH10       /* PB_4  AIN10  7     J1-7    -   */
#define  Adc11      ADC_CTL_CH11       /* PB_5  AIN11  2     J1-2    -   */

#define  Grove_J5   ADC_CTL_CH7        /* PD_0  AIN7  23     J3-3   J5   */
#define  Grove_J6   ADC_CTL_CH6        /* PD_1  AIN6  24     J3-4   J6   */
#define  Grove_J7   ADC_CTL_CH5        /* PD_2  AIN5  25     J3-5   J7   */
#define  Grove_J8   ADC_CTL_CH4        /* PD_3  AIN4  26     J3-6   J8   */
#define  Grove_J9   ADC_CTL_CH2        /* PE_1  AIN2  27     J3-7   J9   */


//  I2C
// -----
/*     Module 1:  UCB0          Channel 1  CCR1   P 2.4    38     J4-3        */
#define  I2C_MODULE_1    1                      /* UCB0 with CCRs 1,2,3,4     */
#define  I2C_MODULE_2    2                      /* UCB1 with CCRs 1,2,3,4  GROVE    */


//  PWM                                                Energia   Launchpad
// -----                                                 Pin    Connector Pin
/*     Module 1:  PWM0  GEN0    Channel A  M0-PWM0  PB6   38     J2-7  J3-3  */
/*                              Channel B  M0-PWM1  PB7   19     J2-6  J3-4  */
/*     Module 2:  PWM0  GEN1    Channel A  M0-PWM2  PB4    7     J1-7     */
/*                              Channel B  M0-PWM3  PB5    2     J1-2     */
/*     Module 3:  PWM0  GEN2    Channel A  M0-PWM4  PE4    5     J1-5  J1-5 ALT */
/*                              Channel B  M0-PWM5  PE5    6     J1-6  J1-6 ALT */
/*     Module 4:  PWM0  GEN3    Channel A  M0-PWM6  PC4   37     J4-4     */
/*                              Channel B  M0-PWM7  PC5   36     J4-5     */

/*     Module 5:  PWM1  GEN0    Channel A  M1-PWM0  PD0   37     J2-7   J3-3  J2-7 ALT */
/*                              Channel B  M1-PWM1  PD1   17     J2-6   J3-4  J2-6 ALT */
/*     Module 6:  PWM1  GEN1    Channel A  M1-PWM2  PA6    9     J1-9     */
/*                              Channel B  M1-PWM3  PA7   10     J1-10    */
/*     Module 7:  PWM1  GEN2    Channel A  M1-PWM4  PF0   37     J2-4     */
/*                              Channel B  M1-PWM5  PF1   17     J3-10    */
/*     Module 8:  PWM1  GEN3    Channel A  M1-PWM6  PF2   36     J4-1     */
/*                              Channel B  M1-PWM7  PF3   35     J4-2     */
#define  PWM_MODULE_1     1            /* PWM0   with Channels 0 - 1  PB6/PB7 */
#define  PWM_MODULE_2     2            /* PWM0   with Channels 2 - 3  PB4/PB5 */
#define  PWM_MODULE_3     3            /* PWM0   with Channels 4 - 5  PE4/PE5 BDC motor AIN1/AIN2 remap J1-5/J1-6 */
#define  PWM_MODULE_4     4            /* PWM0   with Channels 6 - 7  PC4/PC5 */

#define  PWM_MODULE_5     5            /* PWM1   with Channels 0 - 1  PD0/PD1 */
#define  PWM_MODULE_6     6            /* PWM1   with Channels 2 - 3  PA6/PA7 BDC Motor BIN1/BIN2 J1-9/J1-10 */
#define  PWM_MODULE_7     7            /* PWM1   with Channels 4 - 5  PF0/PF1 */
#define  PWM_MODULE_8     8            /* PWM1   with Channels 6 - 7  PF2/PF3 */

   // Note that on Tiva, the TA1.1/TA1.2 pins can be routed out to alternate
   //      pins. If you want to route out the alternate pins then specify
   //      PWM_CHANNEL_1A and/or PWM_CHANNEL_2A to utilize the alternative pins
// go to a neutral channel A / Channel B  and allow Channel_1/Channel_2 as synonyms  ???
#define  PWM_CHANNEL_1    1            /* PWM0 GEN0  PB6   PWM1 GEN0  PD0  */
#define  PWM_CHANNEL_2    2            /* PWM0 GEN0  PB7   PWM1 GEN0  PD1  */
#define  PWM_CHANNEL_3    3            /* PWM0 GEN1  PB4   PWM1 GEN1  PA6  */
#define  PWM_CHANNEL_4    4            /* PWM0 GEN1  PB5   PWM1 GEN1  PA7  */
#define  PWM_CHANNEL_5    5            /* PWM0 GEN2  PE4   PWM1 GEN2  PF0  */
#define  PWM_CHANNEL_6    6            /* PWM0 GEN2  PE5   PWM1 GEN2  PF1  */
#define  PWM_CHANNEL_7    7            /* PWM0 GEN3  PC4   PWM1 GEN3  PF2  */
#define  PWM_CHANNEL_8    8            /* PWM0 GEN3  PC5   PWM1 GEN3  PF3  */
#define  PWM_CHANNEL_7A   9            /* PWM0 GEN3  PD0   --------------  */
#define  PWM_CHANNEL_8A  10            /* PWM0 GEN3  PD1   --------------  */
#define  PWM_CHANNEL_3A   9            /* --------------   PWM1 GEN1  PE4  */
#define  PWM_CHANNEL_4A  10            /* --------------   PWM1 GEN1  PE5  */

#endif                          // defined(PART_TM4C123GH6PM)



//*******************************************************************************
//*******************************************************************************
//                  LOW LEVEL MAPPING OF GPIO PIN calls on Tiva
//*******************************************************************************
//*******************************************************************************

inline void   inline_pin_High (uint32_t gpio_port,  unsigned long pin)
{
    HWREG(gpio_port + GPIO_O_DATA + (pin << 2))  |=  pin;
}

inline void   inline_pin_Low (uint32_t gpio_port,  unsigned long pin)
{
    HWREG(gpio_port + GPIO_O_DATA + (pin << 2))  &=  ~(pin);
}

inline void   inline_pin_Toggle (uint32_t gpio_port,  unsigned long pin)
{
    HWREG(gpio_port + GPIO_O_DATA + (pin << 2))  ^=  pin;
}

inline char   inline_pin_Read (uint32_t gpio_port,  unsigned long pin)
{
    if ( HWREG(gpio_port + GPIO_O_DATA + (pin << 2)) )
       return (1);                 // pin is high
    return (0);                    // pin is low
}

#endif                             //  __USER_BOARD_API_H__
