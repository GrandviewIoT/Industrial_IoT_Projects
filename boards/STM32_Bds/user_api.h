
// 06/02/15 - inline_set_pins_Output() and inline_set_pins_Input() need re-work

/********1*********2*********3*********4*********5*********6*********7**********
*                                                             STM32 Nucleo/Disco
*                                  User_api.h
*
*  Provide a higher level user API for newbies.
*
*  This provides the high level call prototypes, and then imports the
*  associated pin_map file for the specific platform/Launchpad (Tiva, MSP432, .)
*
*  More experienced programmers can directly use the board_xxx calls defined in
*  boarddef.h or in the platform's vendor DriverLib APIs.
*
*  This is just a thumbnail sketch of the APIs and their key parameters.
*  Use it as reference for coding, in order to determine what #define values
*  are needed for a specific call, etc.
*
*  Detailed discussion of the APIs is found in the xxxx docs.
*
*  I _may_ put in doxygen style docs as well at some point (no promises),
*  but in general, I regard them as "plastic fuzzies", since they are always
*  overly cryptic, and usually offer no examples of actual use.
*  They are a poor substitute for real documentation.
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

#ifndef  __USER_API_H__
#define  __USER_API_H__
                             // standard definitions used throughout the project
#include <stdint.h>          // contains uint32_t / uint16_t  etc  defs
#include <stdbool.h>
#include <string.h>          // strcpy/strlen  memcpy/memset  etc
#include <errno.h>

#include "device_config_common.h"

                         //*****************************************************
                         //*****************************************************
                         //
                         //                 Call  Back  Definitions
                         //
                         // pCbParm points to the Callback Parameter, that was
                         //  passed in on the original set_callback() invocation
                         //
                         // flags contain ADC/I2C/SPI/TIM Module Id in low order
                         //  4 bits, as well as any module dependent status bits
                         //*****************************************************
                         //*****************************************************
typedef  void (*P_EVENT_HANDLER)(void *pValue);
typedef  void (*ADC_CB_EVENT_HANDLER)(void *pCbParm, uint16_t *channel_results,
                int num_channels, int flags);
typedef  void (*I2C_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*SPI_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*TMR_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id);
typedef  void (*UART_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*IO_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);


#include "boarddef.h"     // pull in defs for the MCU board being used


#ifndef USES_RTOS
                          //************************************************
                          //************************************************
                          // No RTOS is being used.
                          //
                          //      Very lightweigtht  RTOS   API
                          //      for bare-metal (no RTOS) MCUs.
                          //
                          // Mainly used for coordinating callbacks invoked
                          // from an interrupt ISR to wakeup main loop() that
                          // is waiting on an event / callback to occur.
                          //************************************************
                          //************************************************
#define  SEMAPHORE    int
void  IO_SEMAPHORE_WAIT(int *semaphore_waited_on);
void  IO_SEMAPHORE_RELEASE(int *semaphore_waited_on);
int   OS_WAIT (uint32_t timeout_value);        // is NOP unless using Interrupts

            // No RTOS, just bare-metal.
//#define  IO_SEMAPHORE_SET(obj)      obj=1; // Grab a a semaphore to wait on
//#define  IO_SEMAPHORE_RELEASE(obj)  obj=0; // Release a semaphore being waited on
//void  IO_SEMAPHORE_WAIT (int *semaphore_to_wait_on);
#endif




 //*****************************************************************************
 //*****************************************************************************
 //
 //                                   SYSTEM    APIs
 //
 //*****************************************************************************
 //*****************************************************************************
#define  sys_Init(mcu_speed,opt_flags) { board_init(mcu_speed,opt_flags); /* initialize board: setup clocks, GPIOs,...*/ \
                                         board_systick_timer_config(); /* turn on 1 ms system timer */ }
#define  sys_Delay(ms_value)             board_delay_ms(ms_value)
#define  sys_Delay_Millis(ms_value)      board_delay_ms(ms_value)
#define  sys_Get_Time()                  board_systick_timer_get_value()
#define  sys_Disable_Interrupts()        board_disable_global_interrupts()
#define  sys_Enable_Interrupts()         board_enable_global_interrupts()
#define  frequency_to_period_ticks(freq) board_frequency_to_period_ticks (freq)



 //*****************************************************************************
 //*****************************************************************************
 //
 //                                 GPIO    APIs
 //
 //*****************************************************************************
 //*****************************************************************************
#define  PIN_HIGH                0x01
#define  PIN_LOW                 0x00

#define  pin_Config(pin_id,dir,flags)   board_gpio_pin_config(pin_id,dir,flags)
#define  pin_Config_PinMux(pin_id,dir,pull,altfunc)  board_gpio_pin_config_pinmux(pin_id,dir,pull,altfunc)
#define  pin_Config_IRQ_Pin(pin_id,rise_fall,pullup,irq_vector_num,priority) \
                board_irq_pin_config(pin_id,rise_fall,pullup,irq_vector_num,priority)
#define  pin_Disable_IRQ(pin_id,irq_vector_num,clear_rupts) \
                board_irq_pin_disable(pin_id,irq_vector_num,clear_rupts)
#define  pin_Enable_IRQ(pin_id,irq_vector_num,clear_rupts) \
                board_irq_pin_enable(pin_id,irq_vector_num,clear_rupts)
#define  pin_High(pin_id)            board_gpio_write_pin(pin_id,PIN_HIGH)
#define  pin_Low(pin_id)             board_gpio_write_pin(pin_id,PIN_LOW)
#define  pin_Toggle(pin_id)          board_gpio_toggle_pin(pin_id)
#define  pin_Read(pin_id)            board_gpio_read_pin(pin_id,0)

               // valid values for dir parm on pin_Config()/board_gpio_pin_config.
#define  GPIO_INPUT              0
#define  GPIO_OUTPUT             1

               // valid values for flags parm on pin_Config() call
               // and for pullup parm on pin_Config_IRQ_Pin() call
               // These are for expeirienced programmers that want to tweak
               // the pullups/pulldowns on the GPIO pin.
               // For newbies, just set flags = 0
#define  PIN_USE_NO_PULL         0x0000     // normal, no pull ups/downs
#define  PIN_USE_PULLUP          0x0001     // turn on pullups    (inputs)
#define  PIN_USE_PULLDOWN        0x0002     // turn on pulldowns  (inputs)
#define  PIN_USE_OPEN_DRAIN      0x0003     // turn on open drain (I2C, etc)

                // valid values for rise_fall on pin_Config_IRQ_Pin() call
                // and optionally on flags parm for pin_Config()
#define  GPIO_RUPT_MODE_RISING   0x0100
#define  GPIO_RUPT_MODE_FALLING  0x0200
#define  GPIO_RUPT_MODE_BOTH     0x0300

                // values for irq_vector_num on pin_Config_IRQ_Pin() call
                // are from the MCU's  ????  file,
                // Typical values are EXTI15_10_IRQn, EXTI4_15_IRQn, etc



 //*****************************************************************************
 //*****************************************************************************
 //
 //                            CONSOLE  /  DEBUG_LOG      APIs
 //
 //*****************************************************************************
 //*****************************************************************************

#if ! defined(CONSOLE_BAUD_RATE)
#define  CONSOLE_BAUD_RATE            115200
#endif

#if defined(USES_CONSOLE) || defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
#define  CONSOLE_INIT(baud_rate)    uart_Init(UART_MD,VCP_TX_PIN,VCP_RX_PIN,baud_rate,UART_ENABLE_ECHOPLEX)
#else
             // discard all calls to CONSOLE_INIT if CONSOLE/DEBUG is not enabled
#define  CONSOLE_INIT(baud_rate)
#endif

#if ! defined(USES_CONSOLE_WRITE) && ! defined(USES_CONSOLE)
             // discard all calls to CONSOLE_WRITE if it is not enabled
#define  CONSOLE_WRITE(output_string)
#define  CONSOLE_WRITE_CHAR(out_char)
#define  CONSOLE_GET_CHAR()               0
#else
#define  CONSOLE_WRITE(output_string)     board_uart_write_string(UART_MD,output_string,UART_WAIT_FOR_COMPLETE)
#define  CONSOLE_WRITE_CHAR(out_char)     board_uart_write_char(UART_MD,out_char,UART_WAIT_FOR_COMPLETE)
#define  CONSOLE_GET_CHAR()               board_uart_get_char(UART_MD,0,UART_WAIT_FOR_COMPLETE)
#endif

#ifndef CONSOLE_PAUSE_TIME
             // no CONSOLE_PAUSE_TIME defined by the application.  Use a default of 25 ms
#define  CONSOLE_PAUSE_TIME    25
#endif
///#ifndef CONSOLE_MAX_TIMEOUT
///            // no CONSOLE_MAX_TIMEOUT defined by the application. Use a default of 5000 ms (5 seconds)
///#define  CONSOLE_MAX_TIMEOUT   5000
///#endif

#if ! defined(USES_CONSOLE_READ) && ! defined(USES_CONSOLE)
             // discard all calls to CONSOLE_READ if it is not enabled
#define  CONSOLE_READ_LINE(in_buf,max_lng)  0
#define  CONSOLE_CHECK_FOR_INPUT()          0
#else
             //    CONSOLE_READ_LINE performs echo-plexing for applcations like putty, teleterm, ...
             //    CONSOLE_READ_LINE_NO_ECHO does not perform any echo-plexing.
#define  CONSOLE_READ_LINE_NO_ECHO(in_buf,max_lng)  board_uart_read_text_line(UART_MD,in_buf,max_lng,UART_WAIT_FOR_COMPLETE)
#define  CONSOLE_READ_LINE(in_buf,max_lng,timeout_val)  board_uart_console_read_fdx(UART_MD,in_buf,max_lng,CONSOLE_PAUSE_TIME,timeout_val,UART_WAIT_FOR_COMPLETE)
#define  CONSOLE_CHECK_FOR_INPUT()              board_uart_rx_data_check(UART_MD)
#endif

#if ! defined(USES_DEBUG_LOG) && ! defined(USES_CONSOLE)
             // discard all calls to DEBUG_LOG if it is not enabled
#define  DEBUG_LOG(output_string)
#else
#define  DEBUG_LOG(output_string)          board_uart_write_string(UART_MD,output_string,UART_WAIT_FOR_COMPLETE)
#endif




 //*****************************************************************************
 //*****************************************************************************
 //
 //                                   ADC     APIs
 //
 //
 // Critical notes for STM32 F3 series MCUs (e.g. F3_03 and F3_34):
 //         F3 03 series has 4 separate ADC modules.
 //            ...
 //
 //         F3 34 series has 2 separate ADC modules,
 //         and two _completely separate_ sets of ADC channels.
 //         Although each set is labelled ADC_CHAN_0 through ADC_CHAN_15,
 //         they are phsyically hooked up to separate GPIO pins.
 //         So you _must_ explicitly specify which ADC module you want to use.
 //         The default ADCMD setting goes to ADC1.
 //         To use channels 0 - 15 on ADC2, you must explcitly pass ADCM2 on
 //         the adc_Config_Channel() call.
 //*****************************************************************************
 //*****************************************************************************

// ADC init requires ADCn instance.  Everything else uses Typedef handle after that !
// ADC_Typedef handle is always passed as a pointer

//     ==> pass in just the (unique) pin number (possibly a pair- board PIN# AND gpio PIN #)
//         and have an internal struct that maps ADCn instance to that pin (const), and
//         a different  struct (RAM) that contains assoicated TypeDef handle and ptr to it
//
//     ==> need a separate typedef for each ADC module - how many modules are there ?
//
// Channel Config requires ADC_Typedef ptr and channel number
//   Total number of ADC channels =
//

#define  adc_Init(mod_id,trigger_type,flags)  board_adc_init(mod_id,ADC_DEFAULT_CLOCK,trigger_type,flags)
#define  adc_Config_Channel(mod_id,channel,last) board_adc_config_channel(mod_id,channel,ADC_AUTO_SEQUENCE,ADC_AUTO_STEP,last,0)
#define  adc_Check_All_Completed(module_id)   board_adc_check_conversions_completed(module_id,ADC_AUTO_SEQUENCE)
#define  adc_Enable(module_id)                board_adc_enable(module_id,ADC_AUTO_SEQUENCE)
#define  adc_Disable(module_id)               board_adc_disable(module_id,ADC_AUTO_SEQUENCE)
#define  adc_GetResolution(module_id)         board_adc_get_resolutionn(module_id)
#define  adc_Read(module_id,channel_results)  while ( ! board_adc_check_conversions_completed(module_id,ADC_AUTO_SEQUENCE)) ; \
                                               board_adc_get_results(module_id,ADC_AUTO_SEQUENCE,channel_results)
#define  adc_Set_Callback(module_id,callback_rtn,callback_parm) \
                                              board_adc_set_callback(module_id,callback_rtn,callback_parm)
#define  adc_SetResolution(module_id,bit_resolution)  board_adc_set_resolutionn(module_id,bit_resolution)
#define  adc_User_Trigger_Start(module_id)    board_adc_user_trigger_start(module_id,ADC_AUTO_SEQUENCE)

                  // the following is to allow for platform specific ADC options
#define  adc_Set_Option(module_id,option_type,opt_flags1,opt_flags2)  \
                                              board_adc_set_option(module_id,option_type,opt_flags1,opt_flags2)

                  // the following is for more experienced programmers that want
                  // to have more fine grain control of the sequencing process.
                  // It is used in lieu of adc_Config_Channel().
#define  adc_Config_Channel_Seq(adc_module,pinX,seq,step,last,flags)  \
                                              board_adc_config_channel(adc_module,pinX,seq,step,last,flags)

            // Valid values for module_id used on all adc_ calls
#define  ADC_MD                0    /* ADC "Default Module" id - single ADC module MCUs */
#define  ADC_M1                1    /* ADC Module 1 - advanced, multi-ADC module MCUs (STM32_F3, C2000, ...) */
#define  ADC_M2                2    /* ADC Module 2 -    ditto     */
#define  ADC_M3                3    /* ADC Module 3 -    ditto     */
#define  ADC_M4                4    /* ADC Module 4 -    ditto     */

            // Valid values for adc_init() clock option: 0, or a valid CPU clock
            // clock speed of 1000000 (1 MHz) up to the max speed to the MCU.
            // Using 0 (default clock speed), always sets the MCU to its max speed
#define  ADC_DEFAULT_CLOCK     0    /* use default clocking on adc_init()         */

            // Valid values for adc_init() flags parm
#define  ADC_TRIGGER_RISING    0    /* Trigger ADC on rising  edge of Timer/GPIO trigger */
#define  ADC_TRIGGER_FALLING   1    /* Trigger ADC on falling edge of Timer/GPIO trigger */
#define  ADC_TRIGGER_RISEFALL  2    /* Trigger ADC on rising and falling edge of Timer/GPIO trigger */

            // Valid values for adc_set_resolution() bit_resolution parm
#define  ADC_12_BIT_RESOLUTION 12   /* 12 bit resolution */
#define  ADC_10_BIT_RESOLUTION 10   /* 10 bit resolution */
#define  ADC_8_BIT_RESOLUTION   8   /*  8 bit resolution */
#define  ADC_6_BIT_RESOLUTION   6   /*  6 bit resolution */

            // Valid values for adc_Config_Channel channel parm
            // See critical notes vis-a-vis F3_03 and F3_34 usage of channels above.
#define  ADC_INTERNAL_TEMP    -4    /* Internal Temperature   (MCU dependent)    */
#define  ADC_INTERNAL_VLCD    -3    /* VLCD and VCOMP are mutually exclusive  L0 */
#define  ADC_INTERNAL_VCOMP   -3    /* Internal Comparator    (MCU dependent) L1 */
#define  ADC_INTERNAL_VBAT    -2    /* Internal Voltage on Battery  (MCU dep)    */
#define  ADC_INTERNAL_VREF    -1    /* Internal VREF          (MCU dependent)    */
#define  ADC_CHAN_0            0    /* physical ADC channel 0 on the MCU         */
#define  ADC_CHAN_1            1
#define  ADC_CHAN_2            2
#define  ADC_CHAN_3            3
#define  ADC_CHAN_4            4
#define  ADC_CHAN_5            5
#define  ADC_CHAN_6            6
#define  ADC_CHAN_7            7
#define  ADC_CHAN_8            8
#define  ADC_CHAN_9            9
#define  ADC_CHAN_10           10
#define  ADC_CHAN_11           11
#define  ADC_CHAN_12           12
#define  ADC_CHAN_13           13
#define  ADC_CHAN_14           14
#define  ADC_CHAN_15           15
#define  ADC_CHAN_16           16    /* MCU model dependent - Not all MCUs support 21 channels */
#define  ADC_CHAN_17           17
#define  ADC_CHAN_18           18
#define  ADC_CHAN_19           19
#define  ADC_CHAN_20           20
#define  ADC_CHAN_21           21
#define  ADC_CHAN_VREF_OPAMP1  15    // OP AMP1 VREF: STM32 F3_03 ADC1 only  */
#define  ADC_CHAN_VREF_OPAMP2  17    // OP AMP2 VREF: STM32 F3_03 and F3_34 ADC2 only  */
#define  ADC_CHAN_VREF_OPAMP3  17    // OP AMP3 VREF: STM32 F3_03 ADC3 only  */
#define  ADC_CHAN_VREF_OPAMP4  17    // OP AMP4 VREF: STM32 F3_03 ADC4 only  */

            // Valid values for adc_Config_Channel last flag
#define  ADC_NOT_LAST          0    /* not the last channel, more to configure    */
#define  ADC_LAST              1    /* this is the last channel to be configured  */

            // Valid values for step (RANK) number  - for Experience Programmers
#define  ADC_AUTO_STEP         9    /* automatically assign the step number entry to use */
#define  ADC_STEP_0            0    /* explicit assign by experienced programmers */
#define  ADC_STEP_1            1    /*                ditto                       */
#define  ADC_STEP_2            2    /*                 " "                        */
#define  ADC_STEP_3            3
#define  ADC_STEP_4            4
#define  ADC_STEP_5            5
#define  ADC_STEP_6            6
#define  ADC_STEP_7            7    /* only sequencer 0 allows 8 steps           */

            // Valid values for sequencer_id  (not applicable to STM32)
#define  ADC_AUTO_SEQUENCE     4    /* automatically assign the sequencer to use  */
#define  ADC_SEQUENCER_0       0    /* explicit assign by experienced programmers */
#define  ADC_SEQUENCER_1       1    /*                ditto                       */
#define  ADC_SEQUENCER_2       2    /*                 " "                        */
#define  ADC_SEQUENCER_3       3

            //---------------------------------------------------------------------------
            // Valid values for adc_Init() trigger_type  (How the ADCs will be triggered)
            //
            // Note: not all values are supported on each platform. See list below
            //       that breaks out what each MCU platform supports
            //---------------------------------------------------------------------------
#define  ADC_TRIGGER_USER_APP     0x0400  /* user application triggers, via calls to adc_Trigger_Start() */
#define  ADC_TRIGGER_GPIO_PIN     0x0300  /* A GPIO pin rising up is trigger source. flags contains pin # */


#if defined(STM32F030x8) || defined(STM32F070xB)
                                     //-----   F0_30 and F0_70  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC4  0x0014  /* Timer 1 CCR 4 match is trigger source       */
#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration is trigger source */
#define  ADC_TRIGGER_TIMER_15     0x00F0  /* STM32 only - TIM15 is trigger source        */
#endif


#if defined(STM32F072xB) || defined(STM32F091xC)
                                     //-----   F0_72 and F0_91  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC4  0x0014  /* Timer 1 CCR 4 match is trigger source       */
#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration is trigger source */
#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration is trigger source */
#define  ADC_TRIGGER_TIMER_15     0x00F0  /* STM32 only - TIM15 is trigger source        */
#endif


#if defined(STM32F303xC) || defined(STM32F303xE)
                                     //-----   F3_03   MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_P2   0x0018  /* Timer 1(2) period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC4  0x0034  /* Timer 3 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_8      0x0080  /* Timer 8 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_8_P2   0x0088  /* Timer 8(2) period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_15     0x00F0  /* Timer 15 period expiration/rollover is trigger source */

#define ADC_TRIGGER_HRTIMER_TRIG1 0x0301  /* HR Timer Trig 1 is trigger source */
#define ADC_TRIGGER_HRTIMER_TRIG3 0x0303  /* HR Timer Trig 3 is trigger source */
#endif


#if defined(STM32F334x8)
                                     //-----   F3_34   MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_P2   0x0018  /* Timer 1(2) period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC4  0x0034  /* Timer 3 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_15     0x00F0  /* Timer 15 period expiration/rollover is trigger source */

#define ADC_TRIGGER_HRTIMER_TRIG1 0x0301  /* HR Timer Trig 1 is trigger source */
#define ADC_TRIGGER_HRTIMER_TRIG3 0x0303  /* HR Timer Trig 3 is trigger source */
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
                                     //-----   F1_01 and F1_11  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC4  0x0014  /* Timer 1 CCR 4 match is trigger source  ??? NOT FOUND IN HAL ADC .H file !!!*/

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC3  0x0023  /* Timer 2 CCR 3 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC4  0x0024  /* Timer 2 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC1  0x0031  /* Timer 3 CCR 1 match is trigger source */

#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_5_CC1  0x0051  /* Timer 5 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_5_CC2  0x0052  /* Timer 5 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_5_CC3  0x0053  /* Timer 5 CCR 3 match is trigger source */
#endif


#if defined(STM32F446xx)
                                     //-----   F4_46 a  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC4  0x0014  /* Timer 1 CCR 4 match is trigger source  ??? NOT FOUND IN HAL ADC .H file !!!*/

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC3  0x0023  /* Timer 2 CCR 3 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC4  0x0024  /* Timer 2 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC1  0x0031  /* Timer 3 CCR 1 match is trigger source */

#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_5_CC1  0x0051  /* Timer 5 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_5_CC2  0x0052  /* Timer 5 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_5_CC3  0x0053  /* Timer 5 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_8      0x0080  /* Timer 8 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_8_CC1  0x0081  /* Timer 8 CCR 1 match is trigger source _F4_46 only_ */
#endif


#if defined(STM32F746xx)
                                     //-----   F7  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_P2   0x0018  /* Timer 1(2) period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */

#define  ADC_TRIGGER_TIMER_3_CC4  0x0034  /* Timer 3 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_4      0x0040  /* Timer 4 CCR period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_5      0x0050  /* Timer 5 CCR period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 CCR period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_8      0x0080  /* Timer 8 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_8_P2   0x0088  /* Timer 8(2) period expiration/rollover is trigger source */
#endif


#if defined(STM32L053xx)
                                     //-----   L0  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC4  0x0024  /* Timer 2 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_21_CC2 0x0152  /* Timer 21 CCR 2 match is trigger source */

#define  ADC_TRIGGER_TIMER_22     0x0160  /* Timer 22 period expiration/rollover is trigger source */
#endif


#if defined(STM32L152xC) || defined(STM32L152xE)
                                     //-----   L1  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC3  0x0023  /* Timer 2 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC1  0x0031  /* Timer 3 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC3  0x0033  /* Timer 3 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_4      0x0040  /* Timer 4 CCR period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_9      0x0080  /* Timer 9 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_9_CC2  0x0082  /* Timer 9 CCR 2 match is trigger source */
#endif


#if defined(STM32L476xx)
                                     //-----   L4  MCU  Model  Dependent  Triggers   -----
#define  ADC_TRIGGER_TIMER_1      0x0010  /* Timer 1 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_P2   0x0018  /* Timer 1(2) period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC1  0x0011  /* Timer 1 CCR 1 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC2  0x0012  /* Timer 1 CCR 2 match is trigger source */
#define  ADC_TRIGGER_TIMER_1_CC3  0x0013  /* Timer 1 CCR 3 match is trigger source */

#define  ADC_TRIGGER_TIMER_2      0x0020  /* Timer 2 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_2_CC2  0x0022  /* Timer 2 CCR 2 match is trigger source */

#define  ADC_TRIGGER_TIMER_3      0x0030  /* Timer 3 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_3_CC4  0x0034  /* Timer 3 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_4      0x0040  /* Timer 4 CCR period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_4_CC4  0x0044  /* Timer 4 CCR 4 match is trigger source */

#define  ADC_TRIGGER_TIMER_6      0x0060  /* Timer 6 CCR period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_8      0x0080  /* Timer 8 period expiration/rollover is trigger source */
#define  ADC_TRIGGER_TIMER_8_P2   0x0088  /* Timer 8(2) period expiration/rollover is trigger source */

#define  ADC_TRIGGER_TIMER_15     0x00F0  /* Timer 15 period expiration/rollover is trigger source */
#endif



 //*****************************************************************************
 //*****************************************************************************
 //
 //                           DAC     APIs             STM32:  F0_72, F0_91, F3_34, F4_46,
 //                                                            L0_53, L1_52, L4_76
 //                                                    _not_ on F1_03, F4_01, F4_11
 //
 //*****************************************************************************
 //*****************************************************************************
#define  dac_Gen_Sample_Table(wave_type,table_buf,num_steps)  board_dac_gen_sample_table(wave_type,table_buf,num_steps)
#if defined(EXTERNAL_DAC) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F411xE) || defined(STM32F103xB)
                         // F4_01, F4_11, and F1_03 do not have built-in DACs. Must use external
#define  dac_Init(module_id)                    extern_dac_init(module_id,DAC_DEFAULT_CLOCK,0)
#define  dac_Config_Channel(module_id,chan,trigger_type,sps_frequency,flags)  extern_dac_config_channel(module_id,chan,trigger_type,sps_frequency,0)
#define  dac_Set_Sample_Table(module_id,chan,table_buf,num_steps)   extern_dac_set_sample_table(module_id,chan,table_buf,num_steps)
#define  dac_Check_All_Completed(module_id,chan) extern_dac_check_conversions_completed(module_id,chan,0)
#define  dac_Enable_Channel(module_id,chan)     extern_dac_enable_channel(module_id,chan,0)
#define  dac_Disable_Channel(module_id,chan)    extern_dac_disable_channel(module_id,chan,0)
#else
                         // Use built-in DACs on F0_72, F0_91, F3_34, L0_53, L1_52, ...
#define  dac_Init(module_id)                    board_dac_init(module_id,DAC_DEFAULT_CLOCK,0)
#define  dac_Config_Channel(module_id,chan,trigger_type,sps_frequency,flags)  board_dac_config_channel(module_id,chan,trigger_type,sps_frequency,0)
#define  dac_Set_Sample_Table(module_id,chan,table_buf,num_steps)   board_dac_set_sample_table(module_id,chan,table_buf,num_steps)
#define  dac_Check_All_Completed(module_id,chan) board_dac_check_conversions_completed(module_id,chan,0)
#define  dac_Enable_Channel(module_id,chan)     board_dac_enable_channel(module_id,chan,0)
#define  dac_Disable_Channel(module_id,chan)    board_dac_disable_channel(module_id,chan,0)
#endif

            // Valid values for module_id used on all dac_ calls
#define  DAC_MD                 0    /* DAC "Default Module" id - single DAC module MCUs */
#define  DAC_M1                 1    /* DAC Module 1 - advanced, multi-DAC module MCUs   */
#define  DAC_M2                 2    /* DAC Module 2 -    ditto     */

               // chanX values to use for dac_Config_Channel(), dac_Enable(),...
#define  DAC_CHAN_1             1
#define  DAC_CHAN_2             2

               // wave_type values for dac_Gen_Sample_Table() call
#define  DAC_GEN_SINEWAVE       1
#define  DAC_GEN_SAWTOOTH       2
#define  DAC_GEN_TRIANGLE       3
#define  DAC_GEN_SQUAREWAVE     4

#define  DAC_DEFAULT_CLOCK      0    /* use default clocking on adc_init()         */

               //------------------------------------------------------------------------
               // valid  values for trigger_type in dac_Config_Channel()
               //
               // Note that for DACs, the dac_Enable_Channel() logic will _automatically_
               //      configure any Timer, based on the frequency and number of steps
               //      that were configured by dac_Config_Channel() and
               //      dac_Set_Sample_Table().
               //------------------------------------------------------------------------
#define  DAC_TRIGGER_TIMER_2      0x0520  /* trigger off standard Timer 2 period expire */
#define  DAC_TRIGGER_TIMER_3      0x0530  /* trigger off standard Timer 3 period expire */
#define  DAC_TRIGGER_TIMER_4      0x0540  /* trigger off standard Timer 4 period expire */
#define  DAC_TRIGGER_TIMER_5      0x0550  /* trigger off standard Timer 5 period expire */
#define  DAC_TRIGGER_TIMER_6      0x0560  /* trigger off Timer 6, used for DAC */
#define  DAC_TRIGGER_TIMER_7      0x0570  /* trigger off Timer 7, used for DAC */
#define  DAC_TRIGGER_TIMER_8      0x0580  /* trigger off standard Timer 8 period expire */
#define  DAC_TRIGGER_TIMER_9      0x0590  /* trigger off standard Timer 9 period expire */
#define  DAC_TRIGGER_TIMER_15     0x05F0  /* trigger off standard Timer 15 period expire */
#define  DAC_TRIGGER_TIMER_21     0x05F6  /* trigger off standard Timer 21 period expire */
#define  DAC_TRIGGER_GPIO_PIN     0x0690  /* trigger off EXTI 9 (GPIO) rising edge  */
#define  DAC_TRIGGER_USER_APP     0x0700  /* User app will manually trigger the DAC */


 //*****************************************************************************
 //*****************************************************************************
 //
 //                                  I2C     APIs
 //
 //*****************************************************************************
 //*****************************************************************************
#define  i2c_Init(i2c_mod_id,scl_pin_id,sda_pin_id,master_slave,my_loc_addr,baud_rate_timing,flags) \
             board_i2c_init(i2c_mod_id,scl_pin_id,sda_pin_id,master_slave,my_loc_addr,baud_rate_timing,flags,0L)
#define  i2c_Init_Extended(i2c_mod_id,scl_pin_id,sda_pin_id,master_slave,my_loc_addr,baud_rate_timing,flags,ptr_I2cHdl) \
             board_i2c_init(i2c_mod_id,scl_pin_id,sda_pin_id,master_slave,my_loc_addr,baud_rate_timing,flags,ptr_I2cHdl)
#define  i2c_Check_IO_Completed(module_id,flags) \
             board_i2c_check_io_completed(module_id,flags)

// ??? SHOULD I PASS BACK ACTUAL AMOUNT READ ???  Does HAL EVEN GIVE ME THAT CAPABILITY ???
#define  i2c_Read(i2c_mod_id,slave_addr,receive_buffer,max_buf_length,flags) \
             board_i2c_read(i2c_mod_id,slave_addr,receive_buffer,max_buf_length,flags)
#define  i2c_Read_Header_Data(i2c_mod_id,slave_addr,hdr_buf,hbuf_leng,receive_buffer,rbuf_length,flags) \
             board_i2c_read_header_data(i2c_mod_id,slave_addr,hdr_buf,hbuf_leng, \
                                        receive_buffer,rbuf_length,flags)
#define  i2c_Set_Callback(i2c_mod_id,callback_func,callback_parm) \
             board_i2c_set_callback(i2c_mod_id,callback_func,callback_parm)
#define  i2c_Set_Max_Timeout(i2c_mod_id,max_timeout) \
             board_i2c_set_max_timeout(i2c_mod_id,max_timeout)
#define  i2c_Write(i2c_mod_id,slave_addr,transmit_buffer,buf_length,flags) \
             board_i2c_write(i2c_mod_id,slave_addr,transmit_buffer,buf_length,flags)
#define  i2c_Write_Header_Data(i2c_mod_id,slave_addr,header_buffer,hbuf_length,transmit_buffer,xbuf_length,flags) \
             board_i2c_write_header_data(i2c_mod_id,slave_addr,header_buffer,hbuf_length,transmit_buffer,xbuf_length,flags)
//#define  i2c_Write_Read(i2c_mod_id,slave_addr,transmit_buffer,xbuf_length,receive_buffer,max_rbuf_length,flags) \
// FUTURE    board_i2c_write_read(i2c_mod_id,slave_addr,transmit_buffer,xbuf_length,receive_buffer,max_rbuf_length,flags)

                         //-----------------------------------------------------
                         //  I2C constants for master_slave parm in i2c_Init()
                         //-----------------------------------------------------
#define  I2C_MASTER              1
#define  I2C_SLAVE               2

// ??? Have INTERRUPTs BE A SEPARATE FLAG FROM NON_BLOCKING ??? !!!  WVD

               // valid values for flags parm on i2c_Init()
#define  I2C_IO_USE_INTERRUPTS 0x4000   // If this _is_ set, I2C will use interrupts
#define  I2C_IO_NON_BLOCKING   0x2000   // If this _is_ set, the I2C I/O will
                                        // immediately return after the I/O
                                        // is scheduled. It will use interrupts,
                                        // and will invoke the I2C callback if
                                        // configured, when the interrupts are
                                        // complete, or poll with i2c_Check_IO_Completed().
                                        // Attempting to start another I2C read
                                        // or write before the operation is complete
                                        // will be rejected with ERR_IO_ALREADY_IN_PROGRESS
                                        //
                                        // If this is _not_ set, every I2C I/O
                                        // all will block, and will wait until
                                        // the I2C I/O operation is complete.

               // valid flag values for i2c_Check_All_Completed
#define  I2C_WAIT_FOR_COMPLETE  0x8000   // Do not return until I/O is complete

                         //------------------------------------------------------------------
                         //           I2C Module Id and associated Pin Configurations
                         //
                         // Defines I2C Module id (i2c_mod_id) to be used in every SPI call.
                         //
                         // This defines the standardized I2C module ids to use, and what
                         // specific (pin-mux) combination of GPIO pins that each
                         // specific spi_mod_id automatically manages.
                         //
                         // The Library code will automatically initialize both the SPI periperal
                         // and the associated GPIOs for I2C mode, when i2c_Init() is called,
                         // and will manage the I2C connection during read/write calls.
                         //
                         // NOTE: not all platforms support all of these variations.
                         //       See the supported variations for each Nucleo/Board in docs.
                         //------------------------------------------------------------------
#define  I2C_M1              1    /* I2C Module 1  */
#define  I2C_M2              2    /* I2C Module 2  */
#define  I2C_M3              3    /* I2C Module 3  */
#define  I2C_M4              4    /* I2C Module 4  */

// 10/13/15 - SCRAP ALL OF THE FOLLOWING   vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
                         // options for i2c_Init_Raw()            (Module only init)
                         //                           Must then use pin_MuxConfig() to setup pins
#define  I2C_1                0x0010        // Configure I2C 1 raw, without initializing I2C pins
#define  I2C_2                0x0020        // Configure I2C 2 raw, without initializing I2C pins
#define  I2C_3                0x0030        // Configure I2C 3 raw, without initializing I2C pins
#define  I2C_4                0x0040        // Configure I2C 4 raw, without initializing I2C pins

                         // options for i2c_Init()
                         // Configure I2C module and initialize associated I2C pins
                         // low order 4 bits denote the I2C variation.
                         // high order 4 bits denote the actual module 0x001x - 0x006x
#define  I2C_ID_1_A           0x0011        // I2C 1 using pins PB8/PB9   (MEMS, ...)
#define  I2C_ID_1_B           0x0012        // I2C 1 using pins PB6/PB9
#define  I2C_ID_1_C           0x0013        // I2C 1 using pins PB6/PB7
#define  I2C_ID_1_D           0x0014        // I2C 1 using pins PB8/PB7
#define  I2C_ID_1_E           0x0015        // I2C 1 using pins PA14/PA15
#define  I2C_ID_1_F           0x0016        // I2C 1 using pins PF1/PF0
#define  I2C_ID_1_G           0x0017        // I2C 1 using pins PF6/PF0
#define  I2C_ID_1_H           0x0018        // I2C 1 using pins PA9/PA10
#define  I2C_ID_1_I           0x0019        // I2C 1 using pins PG14/PG13  (L4)
#define  I2C_ID_1_J           0x002A        // I2C 2 using pins PA11/PA12  (F334)
#define  I2C_ID_1_K           0x002B        // I2C 2 using pins PC6/PC7    (F446 FMP)
#define  I2C_ID_1_L           0x002C        // I2C 2 using pins PD12/PD13  (F446 FMP)
#define  I2C_ID_1_M           0x002D        // I2C 2 using pins PD14/PD15  (F446 FMP)
#define  I2C_ID_1_N           0x002E        // I2C 2 using pins PF14/PF15  (F446 FMP)

#define  I2C_ID_2_A           0x0021        // I2C 2 using pins PB10/PB3
#define  I2C_ID_2_B           0x0022        // I2C 2 using pins PB10/PB11
#define  I2C_ID_2_C           0x0023        // I2C 2 using pins PB13/PB14
#define  I2C_ID_2_D           0x0024        // I2C 2 using pins PB10/PB14
#define  I2C_ID_2_E           0x0025        // I2C 2 using pins PB10/PB9     <-- VERIFY THIS ON F4/F0 MAY BE WRONG
#define  I2C_ID_2_F           0x0026        // I2C 2 using pins PA11/PA12    <--   Ditto
#define  I2C_ID_2_G           0x0027        // I2C 2 using pins PF1/PF0
#define  I2C_ID_2_H           0x0028        // I2C 2 using pins PF1/PC12
#define  I2C_ID_2_I           0x0029        // I2C 2 using pins PH4/PH5   (F7)
#define  I2C_ID_2_J           0x002A        // I2C 2 using pins PF6/PF0   (F303)
#define  I2C_ID_2_K           0x002B        // I2C 2 using pins PA9/PA10  (F303)
#define  I2C_ID_2_L           0x002C        // I2C 2 using pins PB10/PC12

#define  I2C_ID_3_A           0x0031        // I2C 3 using pins PA8/PB4
#define  I2C_ID_3_B           0x0032        // I2C 3 using pins PA8/PC9
#define  I2C_ID_3_C           0x0033        // I2C 3 using pins PA8/PB8
#define  I2C_ID_3_D           0x0034        // I2C 3 using pins PC0/PC1
#define  I2C_ID_3_E           0x0035        // I2C 3 using pins PG7/PG8
#define  I2C_ID_3_F           0x0036        // I2C 3 using pins PH7/PH8   (F7)
#define  I2C_ID_3_G           0x0037        // I2C 3 using pins PA8/PH8   (F7)
#define  I2C_ID_3_H           0x0038        // I2C 3 using pins PH7/PC9   (F7)
#define  I2C_ID_3_I           0x0039        // I2C 3 using pins PA8/PB5   (F303)


#define  I2C_ID_4_A           0x0031        // I2C 4 using pins PF14/PF15 (F7)
#define  I2C_ID_4_B           0x0032        // I2C 4 using pins PH11/PH12 (F7)
#define  I2C_ID_4_C           0x0033        // I2C 4 using pins PF14/PH12 (F7)
#define  I2C_ID_4_D           0x0034        // I2C 4 using pins PH11/PF15 (F7)


 //*****************************************************************************
 //*****************************************************************************
 //
 //                                  SPI     APIs
 //
 //*****************************************************************************
 //*****************************************************************************
                         // Note: the user code calling the spi_xx() routines is
                         //      responsible for setting and clearing the CS pin
#define  spi_Init(spi_mod_id,sclk_pin_id,miso_pin_id,mosi_pin_id,master_slave,spi_mode,baud_rate_scalar,flags) \
             board_spi_init(spi_mod_id,sclk_pin_id,miso_pin_id,mosi_pin_id,\
                            master_slave,spi_mode,baud_rate_scalar,flags,0L)
#define  spi_Init_Extended(spi_mod_id,sclk_pin_id,miso_pin_id,mosi_pin_id,master_slave,spi_mode,baud_rate_scalar,flags,ptr_SpiHdl) \
             board_spi_init(spi_mod_id,sclk_pin_id,miso_pin_id,mosi_pin_id,\
                        master_slave,spi_mode,baud_rate_scalar,flags,ptr_SpiHdl)
#define  spi_Check_IO_Completed(module_id,flags)  board_spi_check_io_completed(module_id,flags)

// ??? SHOULD I PASS BACK ACTUAL AMOUNT READ ???  Does HAL EVEN GIVE ME THAT CAPABILITY ???
#define  spi_Read(spi_mod_id,receive_buffer,max_buf_length,flags)  \
             board_spi_read(spi_mod_id,receive_buffer,max_buf_length,flags)
#define  spi_Set_Callback(spi_mod_id,callback_func,callback_parm)     board_spi_set_callback(spi_mod_id,callback_func,callback_parm)
#define  spi_Set_Max_Timeout(spi_mod_id,max_timeout)   board_spi_set_max_timeout(spi_mod_id,max_timeout)
#define  spi_Write(spi_mod_id,transmit_buffer,buf_length,flags)  \
             board_spi_write(spi_mod_id,transmit_buffer,buf_length,flags)
#define  spi_Write_Read(spi_mod_id,transmit_buffer,receive_buffer,buf_length,flags) \
             board_spi_write_read(spi_mod_id,transmit_buffer,receive_buffer,buf_length,flags)

                         //------------------------------------------------------------------
                         //           SPI Module Id and associated Pin Configurations
                         //
                         // Defines SPI Module id (spi_mod_id) to be used in every SPI call.
                         //
                         // This defines the standardized SPI module ids to use, and what
                         // specific (pin-mux) combination of GPIO pins that each
                         // specific spi_mod_id automatically manages.
                         //
                         // The Library code will automatically initialize both the SPI periperal
                         // and the associated GPIOs for SPI mode, when spi_Init() is called,
                         // and will manage the SPI connection during read/write calls.
                         //
                         // NOTE: not all platforms support all of these variations.
                         //       See the supported variations for each Nucleo/Board in docs.
                         //------------------------------------------------------------------
#define  SPI_M1              1    /* SPI Module 1  */
#define  SPI_M2              2    /* SPI Module 2  */
#define  SPI_M3              3    /* SPI Module 3  */
#define  SPI_M4              4    /* SPI Module 4  */
#define  SPI_M5              5    /* SPI Module 5  */
                         // options for spi_Init_Raw()            (Module only init)
                         //                           Must then use pin_MuxConfig() to setup pins
#define  SPI_1                0x0010        // Configure SPI 1 raw, without initializing SPI pins
#define  SPI_2                0x0020        // Configure SPI 2 raw, without initializing SPI pins
#define  SPI_3                0x0030        // Configure SPI 3 raw, without initializing SPI pins
#define  SPI_4                0x0040        // Configure SPI 4 raw, without initializing SPI pins
#define  SPI_5                0x0050        // Configure SPI 5 raw, without initializing SPI pins
#define  SPI_6                0x0060        // Configure SPI 6 raw, without initializing SPI pins

                         // options for spi_Init()
                         // low order 4 bits denote the SPI variation.
                         // high order 4 bits denote the actual module 0x001x - 0x006x
#define  SPI_ID_1_A           0x0011        // SPI 1 using pins PA5/PA6/PA7      (L6474, ...)
#define  SPI_ID_1_B           0x0012        // SPI 1 using pins PB3/PA6/PA7      (SubGHz, ...)
#define  SPI_ID_1_C           0x0013        // SPI 1 using pins PB3/PB4/PB5
#define  SPI_ID_1_D           0x0014        // SPI 1 using pins PA5/PA11/PA12
#define  SPI_ID_1_E           0x0015        // SPI 1 using pins PE13/PE14/PE15
#define  SPI_ID_1_F           0x0016        // SPI 1 using pins PA5/PA6/PA1
#define  SPI_ID_1_G           0x0017        // SPI 1 using pins PB3/PA6/PA1
#define  SPI_ID_1_H           0x0018        // SPI 1 using pins PA5/PE14/PE15
#define  SPI_ID_1_I           0x0019        // SPI 1 using pins PE13/PE14/PE15
#define  SPI_ID_1_J           0x001A        // SPI 1 using pins PB3/PE14/PE15
#define  SPI_ID_1_K           0x001B        // SPI 1 using pins PG2/PG3/PG4
#define  SPI_ID_1_L           0x001C        // SPI 1 using pins PB3/PA11/PA12


#define  SPI_ID_2_A           0x0021        // SPI 2 using pins PB10/PB14/PB15
#define  SPI_ID_2_B           0x0022        // SPI 2 using pins PB13/PB14/PB15   (W5200, ...)
#define  SPI_ID_2_C           0x0023        // SPI 2 using pins PB13/PC2/PC3
#define  SPI_ID_2_D           0x0024        // SPI 2 using pins PD3/PB14/PB15
#define  SPI_ID_2_E           0x0025        // SPI 2 using pins PD3/PC2/PC3
#define  SPI_ID_2_F           0x0026        // SPI 2 using pins PD1/PD3/PD4
#define  SPI_ID_2_G           0x0027        // SPI 2 using pins PF9/PB14/PB15
#define  SPI_ID_2_H           0x0028        // SPI 2 using pins PF10/PB14/PB15
#define  SPI_ID_2_I           0x0029        // SPI 2 using pins PI1/PB14/PB14 - Default for F7 "Arduino Headers"  -- F7 --
#define  SPI_ID_2_J           0x002A        // SPI 2 using pins PB10/PC2/PC3
#define  SPI_ID_2_K           0x002B        // SPI 2 using pins PC7/PB14/PB15
#define  SPI_ID_2_L           0x002C        // SPI 2 using pins PC7/PC2/PC3
#define  SPI_ID_2_M           0x002D        // SPI 2 using pins PF1/PA10/PA11
#define  SPI_ID_2_N           0x002E        // SPI 2 using pins PA9/PC2/PC1
#define  SPI_ID_2_Q           0x002F        // SPI 2 using pins PI1/PI2/PI3

#define  SPI_ID_3_B           0x0032        // SPI 3 using pins PB3/PB4/PB5
#define  SPI_ID_3_C           0x0033        // SPI 3 using pins PC10/PC11/PC12
#define  SPI_ID_3_D           0x0034        // SPI 3 using pins PB12/PB4/PB5
#define  SPI_ID_3_E           0x0035        // SPI 3 using pins PB12/PC11/PC12
#define  SPI_ID_3_F           0x0036        // SPI 3 using pins PB3/PC11/PC12
#define  SPI_ID_3_G           0x0037        // SPI 3 using pins PC10/PB4/PB5
#define  SPI_ID_3_H           0x0038        // SPI 3 using pins PG9/PG10/PG11
#define  SPI_ID_3_I           0x0039        // SPI 3 using pins PC10/PC11/PB2
#define  SPI_ID_3_J           0x003A        // SPI 3 using pins PC10/PC11/PD0
#define  SPI_ID_3_K           0x003B        // SPI 3 using pins PC10/PC11/PD6
#define  SPI_ID_3_L           0x003C        // SPI 3 using pins PB0/PB4/PB5
#define  SPI_ID_3_M           0x003D        // SPI 3 using pins PB0/PC11/PC12

#define  SPI_ID_4_A           0x0041        // SPI 4 using pins PB13/PE5/PE6
#define  SPI_ID_4_B           0x0042        // SPI 4 using pins PB13/PE13/PE14
#define  SPI_ID_4_C           0x0043        // SPI 4 using pins PB13/PA11/PE6
#define  SPI_ID_4_D           0x0044        // SPI 4 using pins PB13/PA11/PE14
#define  SPI_ID_4_E           0x0045        // SPI 4 using pins PE2/PE5/PE6
#define  SPI_ID_4_F           0x0046        // SPI 4 using pins PE12/PE13/PE14
#define  SPI_ID_4_G           0x0047        // SPI 4 using pins PE12/PE5/PE6
#define  SPI_ID_4_H           0x0048        // SPI 4 using pins PE2/PE13/PE14
#define  SPI_ID_4_I           0x0049        // SPI 4 using pins PG11/PG12/PG13
#define  SPI_ID_4_J           0x004A        // SPI 4 using pins PE2/PG12/PG13
#define  SPI_ID_4_K           0x004B        // SPI 4 using pins PE12/PG12/PG13
#define  SPI_ID_4_L           0x004C        // SPI 4 using pins PE2/PD0/PE6

#define  SPI_ID_5_A           0x0051        // SPI 5 using pins PB0/PA12/PA10
#define  SPI_ID_5_B           0x0052        // SPI 5 using pins PB0/PA12/PB8
#define  SPI_ID_5_C           0x0053        // SPI 5 using pins PE2/PE5/PE6
#define  SPI_ID_5_D           0x0054        // SPI 5 using pins PE12/PE13/PE14
#define  SPI_ID_5_E           0x0055        // SPI 5 using pins PE2/PE13/PE14
#define  SPI_ID_5_F           0x0056        // SPI 5 using pins PE12/PE5/PE6
#define  SPI_ID_5_G           0x0057        // SPI 5 using pins PF7/PF8/PF9
#define  SPI_ID_5_H           0x0058        // SPI 5 using pins PF7/PF8/PF11
#define  SPI_ID_5_I           0x0059        // SPI 5 using pins PH6/PF8/PF9
#define  SPI_ID_5_J           0x005A        // SPI 5 using pins PH6/PH7/PF9
#define  SPI_ID_5_K           0x005B        // SPI 5 using pins PH6/PH7/PF11

#define  SPI_ID_6_A           0x0061        // SPI 6 using pins PG12/PG13/PG14


                         //-----------------------------------------------------
                         //  SPI constants for master_slave parm in spi_Init()
                         //-----------------------------------------------------
#define  SPI_MASTER              1
#define  SPI_SLAVE               2

                         //--------------------------------------
                         // spi_mode constants - for spi_Init()
                         //--------------------------------------
#define  SPI_MODE_0              0
#define  SPI_MODE_1              1
#define  SPI_MODE_2              2
#define  SPI_MODE_3              3

               // valid values for flags parm on spi_Init()/board_spi_init.
               // These are for expeirienced programmers that want to tweak
               // the pullups/pulldowns on the SPI pins.
               // For newbies, just set flags = 0
#define  SCLK_USE_PULLUP      0x0001
#define  SCLK_USE_PULLDOWN    0x0002
#define  MISO_USE_PULLUP      0x0004
#define  MISO_USE_PULLDOWN    0x0008
#define  MOSI_USE_PULLUP      0x0010
#define  MOSI_USE_PULLDOWN    0x0020


// ??? Have INTERRUPTs BE A SEPARATE FLAG FROM NON_BLOCKING ??? !!!  WVD

               // Additional valid values for flags parm on spi_Init()
#define  SPI_IO_USE_INTERRUPTS 0x4000   // If this _is_ set, SPI will use interrupts
#define  SPI_IO_NON_BLOCKING   0x2000   // If this _is_ set, the SPI I/O will
                                        // immediately return after the I/O
                                        // is scheduled. It will use interrupts,
                                        // and will invoke the SPI callback if
                                        // configured, when the interrupts are
                                        // complete, or poll with spi_Check_IO_Completed().
                                        // Attempting to start another SPI read
                                        // or write before the operation is complete
                                        // will be rejected with ERR_IO_ALREADY_IN_PROGRESS
                                        //
                                        // If this is _not_ set, every SPI I/O
                                        // all will block, and will wait until
                                        // the SPI I/O operation is complete.

               // valid flag values for spi_Check_All_Completed
#define  SPI_WAIT_FOR_COMPLETE  0x8000   // Do not return until I/O is complete




 //*****************************************************************************
 //*****************************************************************************
 //
 //                                   PWM     APIs
 //
 //*****************************************************************************
 //*****************************************************************************
#define  PWM_COUNT_MODE   TIMER_COUNT_UP
#define  pwm_Init(module_id,period,flags) \
             board_timerpwm_init(module_id,PWM_COUNT_MODE,period,0,flags,0L)
                  // pwm_Init_Extended() is only for special operations
#define  pwm_Init_Extended(module_id,period,flags,ptr_TimHdl) \
             board_timerpwm_init(module_id,PWM_COUNT_MODE,period,0,flags,ptr_TimHdl)
#define  pwm_Config_Channel(module_id,chan,duty_cycle,flags) \
             board_timerpwm_config_channel(module_id,chan,duty_cycle,TIMER_MODE_PWM,flags)
//int    board_pwm_config_channel_pair(module_id,channelA_id,channelB_id,0)
#define  pwm_Disable(module_id,flags)       board_timerpwm_disable(module_id,flags)
#define  pwm_Enable(module_id,flags)        board_timerpwm_enable(module_id,flags)
#define  pwm_Get_Duty_Cycle(module_id,chan) board_timerpwm_get_duty_cycle(module_id,chan)
#define  pwm_Get_Period(module_id)          board_timerpwm_get_period(module_id)
#define  pwm_Set_Period(module_id,period,flags)  board_timerpwm_set_period(module_id,period,flags)
#define  pwm_Set_Duty_Cycle(module_id,chan,duty,flags) board_timerpwm_set_duty_cycle(module_id,chan,duty,flags)
#define  pwm_Set_Dead_Time(module_id,deadtime)   board_timerpwm_set_dead_time(module_id,deadtime,deadtime)
#define  pwm_Set_Phase(chanX,phase_offset)       board_timerpwm_set_phase(chanX, phase_offset)
#define  pwm_Set_Channel_Output(module_id,chan,duty)  board_timerpwm_set_channel_output(module_id,chan,output_mode,0)
#define  pwm_Set_Prescalar(module_id,prescalar_val,flags)  board_timerpwm_set_prescalar (module_id,prescalar_val,flags)

// PWM module_id values (PWM_MODULE_1, PWM_MODULE_2, ...) is product dependent, and found in XXXX

               // allowed  values for channel numbers in dac_Config_Channel(), etc
#define  PWM_CHANNEL_1   1
#define  PWM_CHANNEL_2   2
#define  PWM_CHANNEL_3   3
#define  PWM_CHANNEL_4   4

               // allowed values for flags in pwm_Config_Channel() calls
#define  PWM_NO_FLAGS               0
#define  PWM_COMPLEMENTARY_OUTPUTS  1

               // valid  values for rupt_flags on pwm_Enable()
#define  PWM_PERIOD_INTERRUPT_ENABLED     0x0001    // raise Interrupt when PWM/Timer period reached




 //*****************************************************************************
 //*****************************************************************************
 //
 //                                  Timer  APIs
                         //
                         // USER_TIMER_2 and USER_TIMER_3 are
                         // universal across all boards.
                         // Other timers are board dependent.
 //
 //*****************************************************************************
 //*****************************************************************************
#define  timer_Init(module_id,counter_type,period_value,flags) \
                              board_timerpwm_init(module_id,counter_type,period_value,TIMER_DEFAULT_CLOCK,flags)

                         // CONVENIENCE FUNTION: Automatically init and enable a simple Timer (No CCRs used)
#define  timer_AutoStart(module_id,counter_type,period_value,interrupt_flags) \
                           { board_timerpwm_init(tmrmod_id,counter_type,period_value,TIMER_DEFAULT_CLOCK,0); \
                             board_timerpwm_enable(tmrmod_id,interrupt_flags); }

                         // CONVENIENCE FUNTION: Automatically config a Timer used for ADC triggering
                         // Note: the ADC _must_ be initialized via adc_Init() _before_ timer_ADC_Trigger_Start()
                         //       is called, as per the STM32 Tech Ref
#define  timer_ADC_Trigger_Start(tmrmod_id,adcmod_id,trig_type,sps_frequency,flags) \
                              {board_timerpwm_init(tmrmod_id,TIMER_COUNT_UP,(frequency_to_period_ticks(sps_frequency)),TIMER_DEFAULT_CLOCK,0); \
                              board_timerpwm_config_trigger_mode(tmrmod_id,adcmod_id,trig_type,(flags|TIMER_ADC_TRIGGER_MODE)); \
                              if (flags & TIMER_AUTOSTART) \
                                 board_timerpwm_enable(tmrmod_id,0);}

                         // CONVENIENCE FUNTION: Automatically config a Timer used for DAC triggering
                         // Note: the DAC _must_ be initialized via dac_Init() _before_ timer_DAC_Trigger_Start()
                         //       is called, as per the STM32 Tech Ref
#define  timer_DAC_Trigger_Start(tmrmod_id,dac_chan_id,trig_type,dac_frequency,num_steps,flags) \
                              {board_timerpwm_init(tmrmod_id,TIMER_COUNT_UP,(frequency_to_period_ticks(dac_frequency)*num_steps),TIMER_DEFAULT_CLOCK,TIMER_DISABLE_DAC_PRESCALING); \
                              board_timerpwm_config_trigger_mode(tmrmod_id,dac_chan_id,trig_type,(flags|TIMER_DAC_TRIGGER_MODE)); \
                              if (flags & TIMER_AUTOSTART) \
                                 board_timerpwm_enable(tmrmod_id,0);}

#define  timer_ADC_Trigger_Config(tmr_mod_id,adc_mod_id,trig_type,flags) \
                              board_timerpwm_config_trigger_mode(tmr_mod_id,adc_mod_id,trig_type,(flags|TIMER_ADC_TRIGGER_MODE));

#define  timer_DAC_Trigger_Config(tmrmod_id,dac_chan_id,trig_type,dac_frequency,num_steps,flags) \
                              board_timerpwm_config_trigger_mode(tmrmod_id,dac_chan_id,trig_type,(flags|TIMER_DAC_TRIGGER_MODE));

#define  timer_Check_Completed(module_id,check_mask,reset_flags) \
                              board_timerpwm_check_completed(module_id,check_mask,reset_flags)
#define  timer_Disable(module_id) \
                              board_timerpwm_disable(module_id,0)
#define  timer_Enable_CCR_Input(module_id,channel_id,initial_duty,flags)  \
                              board_timerpwm_enable_CCR_input(module_id,channel_id,initial_duty,flags)
#define  timer_Enable_CCR_Output(module_id,channel_id,initial_duty,action_flags,rupt_flags) \
                              board_timerpwm_enable_CCR_output(module_id,channel_id,initial_duty,action_flags,rupt_flags)
#define  timer_Enable(module_id,interrupt_flags)         board_timerpwm_enable(module_id,interrupt_flags)
#define  timer_Get_Current_Value(module_id)              board_timerpwm_get_current_value(module_id)
#define  timer_Get_CCR_Capture_Value(module_id,CCR_Num)  board_timerpwm_get_CCR_capture_value(module_id,CCR_Num)
#define  timer_Get_Period(module_id)                     board_timerpwm_get_period(module_id)
#define  timer_Reset_CCR_Output(module_id,channel_id)    board_timerpwm_reset_CCR_output(module_id,channel_id,0)
#define  timer_Set_CCR_Duty(module_id,CCR_Num,CCR_duty_compare_value,flags) \
                              board_timerpwm_set_duty_cycle(module_id,CCR_Num,CCR_duty_compare_value,flags)
#define  timer_Set_Callback(mod_id,callback_rtn,callback_parm)  board_timerpwm_set_callback(mod_id,callback_rtn,callback_parm);
#define  timer_Set_Period(module_id,new_period_value,flags) board_timerpwm_set_period(module_id,new_period_value,flags)
#define  timer_Set_Prescalar(module_id,prescalar_val,flags)  board_timerpwm_set_prescalar (module_id,prescalar_val,flags)




//  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV

// ??? !!! NEED ABILITY TO HAVE PROGRAMMER EXPLICITLY SPECIFY TIMER CC BY PIN RATHER THAN BY ID - 08/24/15
                         // options for timer_Enable_CCR_By_Pin()        (configure using pin name, not logicial id)
#define  CCR_1                0x0010        // Configure SPI 1 raw, without initializing SPI pins

                         // or  timer_Enable_CCR_Raw()
                         //                           Must use pin_MuxConfig() to setup pins before call Enable

                         // This is for experienced programmers that have to create a specifically crafted
                         // configuration for a specific MCU.
                         // This is 100 % MCU model dependent.
                         // This code will break when you try to port it to a differnt MCU model.

//  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


#define  TIMER_DEFAULT_CLOCK          0   /* use default clock source on board_timer_init() */

               // allowed  values for counter_type in timer_Init()
#define  TIMER_COUNT_DOWN             1    /* periodic, continuous Count Down */
#define  TIMER_COUNT_UP               2    /* periodic, continuous Count Up   */
#define  TIMER_COUNT_UPDOWN           3    /* periodic, continuous Count Up then Count Down, aka Center Aligned mode */
#define  TIMER_ONE_SHOT_COUNT_UP      4    /* single, one-shot pulse Up   */
#define  TIMER_ONE_SHOT_COUNT_DOWN    5    /* single, one-shot pulse Down */
#define  TIMER_INPUT_CAPTURE          6
#define  PWM_COUNT_DOWN               TIMER_COUNT_DOWN    // pwm_init_module count_mode   NEED TO BE REMOVED WVD 07/11/15
#define  PWM_COUNT_UP                 TIMER_COUNT_UP
#define  PWM_COUNT_UPDOWN             TIMER_COUNT_UPDOWN  // count up then down (aka "center mode")

               // valid values for trig_type parameter on board_timer_ADC_Trigger_Start()
               // This must match the value specified on adc_Init()
// Platform dependent. See ADC_TRIGGER_xxx entries.

               // valid values for trig_type parameter on board_timer_ADC_Trigger_Start()
               // This must match the value specified on dac_Init()
// Platform dependent. See DAC_TRIGGER_TIMER_xxx entries

               // valid values for flags parameter on board_timer_ADC_Trigger_Start()
               // and timer_DAC_Trigger_Start() calls
#define  TIMER_AUTOSTART         0x0100    // Automatically start/enable the timer
#define  TIMER_NO_AUTOSTART      0x0000    // Leave timer disabled. App will enable it later.
#define  TIMER_ADC_TRIGGER_MODE  0x0200    // Timer is to operate as a ADC trigger
#define  TIMER_DAC_TRIGGER_MODE  0x0400    // Timer is to operate as a DAC trigger

               // valid values for MODE parameter on board_timerpwm_config_channel()
               // Note: rollover means the Timer's counter has reached its max period value, and then rolls over to 0
#define  TIMER_MODE_OC_PASSTHRU       0    /* eg. OUTMOD_0 / Frozen    GPIO = Polarity setting */
#define  TIMER_MODE_OC_SET            1    /* eg. OUTMOD_1 / GO_ACTIVE GPIO = Set on when CCRx hit, and stays on    */
#define  TIMER_MODE_OC_TOGGLE_RESET   2    /* eg. OUTMOD_2 /  -        GPIO = Toggle when CCRx hit, Resets on rollover */
#define  TIMER_MODE_OC_SET_RESET      3    /* eg. OUTMOD_3 /  -        GPIO = Set when CCRx hit, Resets on rollover */
#define  TIMER_MODE_OC_TOGGLE         4    /* eg. OUTMOD_4 / Toggle    GPIO = Toggles every time when CCRx hit      */
#define  TIMER_MODE_OC_RESET          5    /* eg. OUTMOD_5 / GO_INACTIVE GPIO = Reset on when CCRx hit, and stays on  */
#define  TIMER_MODE_PWM_INV           6    /* eg. OUTMOD_6 / PWM_2     GPIO = High -> Low  at CCR trip  */
#define  TIMER_MODE_PWM               7    /* eg. OUTMOD_7 / PWM_1     GPIO = Low  -> High at CCR trip  */

               // valid  values for action_flags on timer_Enable_CCR_Output()
#define  TIMER_ACTION_GO_ACTIVE    TIMER_MODE_OC_SET     /* GPIO pin goes active   when CCR value is reached */
#define  TIMER_ACTION_GO_INACTIVE  TIMER_MODE_OC_RESET   /* GPIO pin goes inactive when CCR value is reached */
#define  TIMER_ACTION_TOGGLE       TIMER_MODE_OC_TOGGLE  /* GPIO pin toggles state when CCR value is reached */
#define  TIMER_ACTION_PULSE        TIMER_MODE_PWM_INV    /* GPIO pin PWM pulse state when CCR value is reached */


               // valid  values for extended_flags on timer_Enable_CCR_Output()
#define  TIMER_AUTO_PRESCALE            0x0080   /* Auto-pre-scale values on Period/Duty Cycle Set cmds*/
//#define  TIMER_SKIP_PRESCALING         0x0080   /* do not perform pre-scaling on Period/Duty Cycle */
#define  TIMER_DISABLE_DAC_PRESCALING  0x0040   /* use on board_timerpwm_init() flags for DAC timers */
#define  TIMER_PIN_POLARITY_HIGH       0x0000   /* "active" means GPIO pin goes high           */
#define  TIMER_PIN_POLARITY_LOW        0x0020   /* "active" means GPIO pin goes low (inverted) */
#define  TIMER_CCR_TIMER_ONLY          0x0200   /* no CCR output to GPIOs. Run CCR as timer only */
//#define  TIMER_ENABLE_CCR_INTERRUPTS   0x0800   /* raise an interrupt when CCR value is reached  */
#define  TIMER_CCR_INTERRUPT_ENABLED   0x0800   /* raise an interrupt when CCR value is reached  */


               // valid  values for rupt_flags on timer_Enable()
//#define  TIMER_ENABLE_ROLLOVER_INTERRUPTS  0x0001
#define  TIMER_PERIOD_INTERRUPT_ENABLED     0x0001    // raise Interrupt when Timer period reached

#define  TIMER_NO_INTERRUPT     0x0000          // timer_enable()/enable_CCR() function does
                                                // NOT want any interrupts to be used

#define  TIMER_1                   1
#define  TIMER_2                   2
#define  TIMER_3                   3
#define  TIMER_4                   4
#define  TIMER_5                   5
#define  TIMER_6                   6
#define  TIMER_7                   7
#define  TIMER_8                   8
#define  TIMER_9                   9
#define  TIMER_10                 10
#define  TIMER_11                 11
#define  TIMER_14                 14
#define  TIMER_15                 15
#define  TIMER_16                 16
#define  TIMER_17                 17
#define  TIMER_20                 20
#define  TIMER_21                 21
#define  TIMER_22                 22
#define  TIMER_HRTIM              23

// ??? SHOULD CHANNEL NUMBER also go to PIN_ID INSTEAD ????
#define  TIMER_CHANNEL_1           1  // Up to 4 channels per Timer / PWM Module
#define  TIMER_CHANNEL_2           2  // except for MSP430/MSP432, which allow up to 6 channels
#define  TIMER_CHANNEL_3           3
#define  TIMER_CHANNEL_4           4
#define  TIMER_CHANNEL_5           5  // MSP432 / MSP430 only
#define  TIMER_CHANNEL_6           6  // MSP432 / MSP430 only

#define  TIMER_CHANNEL_1_ALT1      8  // Alternate Pinouts (Pin_Mux) for Channel 1 outputs
#define  TIMER_CHANNEL_2_ALT1      9  //   See associated API notes for each platform
#define  TIMER_CHANNEL_3_ALT1     10  //   that show what alternate pins these route to
#define  TIMER_CHANNEL_4_ALT1     11
#define  TIMER_CHANNEL_1_ALT2     12
#define  TIMER_CHANNEL_2_ALT2     13
#define  TIMER_CHANNEL_3_ALT2     14
#define  TIMER_CHANNEL_4_ALT2     15

#define  TIMER_CHANNEL_1_ALT3     32
#define  TIMER_CHANNEL_2_ALT3     33
#define  TIMER_CHANNEL_3_ALT3     34
#define  TIMER_CHANNEL_4_ALT3     35
#define  TIMER_CHANNEL_1_ALT4     36
#define  TIMER_CHANNEL_2_ALT4     37
#define  TIMER_CHANNEL_3_ALT4     38
#define  TIMER_CHANNEL_4_ALT4     39

#define  TIMER_CHANNEL_1_N        16  // Complementary, Inverted output pins (TIM1_CH1N, ...)
#define  TIMER_CHANNEL_2_N        17
#define  TIMER_CHANNEL_3_N        18
#define  TIMER_CHANNEL_1_N_ALT1   20  // Alternate Pinouts (Pin_Mux) for Complementary, Inverted output pins
#define  TIMER_CHANNEL_2_N_ALT1   21
#define  TIMER_CHANNEL_3_N_ALT1   22
#define  TIMER_CHANNEL_1_N_ALT2   24
#define  TIMER_CHANNEL_2_N_ALT2   25
#define  TIMER_CHANNEL_3_N_ALT2   26
#define  TIMER_CHANNEL_1_N_ALT3   27
#define  TIMER_CHANNEL_2_N_ALT3   28
#define  TIMER_CHANNEL_3_N_ALT3   29
#define  TIMER_CHANNEL_1_N_ALT4   30
#define  TIMER_CHANNEL_2_N_ALT4   31

               // interrupt flags passed back when User Timer Callback Handler is invoked.
               // Only one flag type will be passed back per callback invocation.
#define  TIMER_CCR1_INTERRUPT       1
#define  TIMER_CCR2_INTERRUPT       2
#define  TIMER_CCR3_INTERRUPT       3
#define  TIMER_CCR4_INTERRUPT       4
#define  TIMER_CCR5_INTERRUPT       5      // MSP432 / MSP430 only
#define  TIMER_CCR6_INTERRUPT       6      // MSP432 / MSP430 only
#define  TIMER_ROLLOVER_INTERRUPT   8


 //*****************************************************************************
 //*****************************************************************************
 //
 //                              UART     APIs
 //
 // Note: UART routines _always_ use interrupts, to properly support FDX devices.
 //*****************************************************************************
 //*****************************************************************************
#define  uart_Init(mod_id,TX_pin_id,RX_pin_id,baud_rate,flags)  \
                 board_uart_init(mod_id,TX_pin_id,RX_pin_id,baud_rate,flags)
#define  uart_Check_IO_Completed(mod_id,flags)       board_uart_check_io_completed(mod_id,flags)
#define  uart_Check_Data_Available(mod_id,flags)     board_uart_rx_data_check(mod_id)
#define  uart_Flush_RX(module_id,flags)              board_uart_rx_flush(module_id,flags)
#define  uart_Get_Char(mod_id,flags,max_wait)        board_uart_get_char(mod_id,flags,max_wait)
#define  uart_Read_Line(mod_id,string,maxlen,flags)  board_uart_read_text_line(mod_id,string,maxlen,flags)
#define  uart_Read_Binary(mod_id,bytebuf,len,flags)  board_uart_read_bytes(mod_id,bytebuf,len,flags)
#define  uart_Set_Callback(mod_id,callback_func,callback_parm) board_uart_set_callback(mod_id,callback_func,callback_parm)
////#define  uart_Set_Echoplex(module_id,on_off_flag)    board_uart_set_echoplex(module_id,on_off_flag)
#define  uart_Set_Max_Timeout(mod_id,max_timeout)    board_uart_set_max_timeout(mod_id,max_timeout)
#define  uart_Write_String(mod_id,string,flags)      board_uart_write_string(mod_id,string,flags)
#define  uart_Write_Binary(mod_id,bytebuf,len,flags) board_uart_write_bytes(mod_id,bytebuf,len,flags)
#define  uart_Write_Char(mod_id,outchar,flags)       board_uart_write_char(mod_id,outchar,flags)
//   ?? add uart_Set_Callback() in future, and add flags for INTERRUPT_IO on uart_Init()

            // Valid values for module_id used on all uart_ calls
#define  UART_MD                  0     /* UART "Default Module" id = VCP D1/D0 */
#define  UART_M1                  1     /* UART Module 1  */
#define  UART_M2                  2     /* UART Module 2  */
#define  UART_M3                  3     /* UART Module 3  */
#define  UART_M4                  4     /* UART Module 4  */
#define  UART_M5                  5     /* UART Module 5  */
#define  UART_M6                  6     /* UART Module 6  */

            // Valid values for flags used on uart_Init
#define  UART_ENABLE_PULL_UPS     0x0001  /* Enable Pull_Ups    on TX/RX lines */
#define  UART_ENABLE_PULL_DOWNS   0x0002  /* Enable Pull_Downs  on TX/RX lines */
#define  UART_ENABLE_OVERSAMPLING 0x0004  /* Enable 16x oversampling/averaging */
#define  UART_ENABLE_ECHOPLEX     0x1000  /* automatically echo-back received characters */
#define  UART_IO_NON_BLOCKING     0x2000  /* If this _IS_ set, the UART I/O will return if no rcv data available */
                                          /* and will wait until all write chars are complete */

            // valid flags for flags used on uart_Read_String/Bytes
            //                           and uart_Write_String/Bytes
#define  UART_WAIT_FOR_COMPLETE   0x8000  // Do not return until I/O is complete (default)

            // status flags passed back on UART callback
#define  UART_TX_COMPLETE           1
#define  UART_RX_COMPLETE           2



 //-----------------------------------------------------------------------------
 //-----------------------------------------------------------------------------
                         //        UNIQUE-ID / CRC    APIs
 //-----------------------------------------------------------------------------
 //-----------------------------------------------------------------------------

 //*****************************************************************************
 //*****************************************************************************
 //
 //                          VTIMER   (Virtual Timer)  APIs
 //
 //*****************************************************************************
 //*****************************************************************************
#define  vtimer_Start(vtimer_id,timer_duration_millis,callback_function,callback_parm)  board_vtimer_start (vtimer_id,timer_duration_millis,callback_function,callback_parm)
#define  vtimer_Check_Completed(vtimer_id)   board_vtimer_completed(vtimer_id)
#define  vtimer_Stop(vtimer_id)              board_vtimer_reset (vtimer_id)




 //*****************************************************************************
 //*****************************************************************************
 //
 //                           Generic  WARNING  CODEs
 //
 //                          Positive values = warnings.
 //*****************************************************************************
 //*****************************************************************************
#define  WARN_WOULD_BLOCK                   450   // NON_BLOCKING was requested, and I/O would block
#define  WARN_ADC_WAS_ALREADY_INITIALIZED   451   // adc_Init() was issued a 2nd time on the same module
#define  WARN_COMPARATOR_WAS_ALREADY_INITIALIZED  452   // comp_Init() was issued a 2nd time on the same module
#define  WARN_I2C_WAS_ALREADY_INITIALIZED   453   // i2c_Init() was issued a 2nd time on the same module
#define  WARN_SPI_WAS_ALREADY_INITIALIZED   454   // spiInit() was issued a 2nd time on the same module
#define  WARN_TIMER_WAS_ALREADY_INITIALIZED 455   // timer_Init() was issued a 2nd time on the same module
                                                  // to a timer that was already initialized.
                                                  // The second enable was ignored, and its period was not changed.
#define  WARN_TIMER_WAS_ALREADY_STARTED     456   // timer_Enable() was issued a 2nd time
                                                  // to a timer that was already started.
                                                  // The second enable was ignored.


 //*****************************************************************************
 //*****************************************************************************
 //
 //                           Generic  ERROR  CODEs
 //
 //                          Negative values = errors.
 //*****************************************************************************
 //*****************************************************************************
#define  ERR_GPIO_INVALID_PIN_ID            -200   /* Pin Id supplied on i2c_Init((), ... uart_Init((), is out of range */
#define  ERR_GPIO_INVALID_PORT_ID           -201   /* Port Id supplied on port_Read() or port_Write(), is out of range */
#define  ERR_IO_ALREADY_IN_PROGRESS         -208   /* A previous I/O call has not completed yet.*/
                                                   /* The request to start a new I/O is rejected*/

#define  ERR_NETWORK_CABLE_DISCONNECTED     -210   /* Ethernet PHY denotes Cable disconnected */
#define  ERR_NETWORK_ARP_TIMEOUT            -211   /* ARP request for an IP address timed out.
                                                   ** This means either the address could not be
                                                   ** found, or your Router/Gateway is mis-configured
                                                   ** and not sending the request out beyond your local subnetwork */

#define  ERR_ADC_MODULE_ID_OUT_OF_RANGE     -220   /* Valid range is 0 to 2 (ANY)  */
#define  ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE   -221
#define  ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE  -222   /* Valid range is 0 to 8 (ANY)  */
#define  ERR_ADC_STEP_NUM_OUT_OF_RANGE      -223   /* Valid range is 0 to 4 (ANY)  */
#define  ERR_ADC_MODULE_NOT_INITIALIZED     -224   /* need to call adc_init first */
#define  ERR_ADC_UNSUPPORTED_TRIGGER_TYPE   -225   /* trigger_type not supported/valid */
#define  ERR_ADC_INVALID_BIT_RESOLUTION     -226   /* bit_resolution specified adc_SetResolution() needs to be 6,8,10, or 12 */
#define  ERR_ADC_STILL_BUSY                 -227   /* Issued a adc_User_Trigger_Start() but ADC stiill busy on a previous conversion */
#define  ERR_ADC_INITIALIZATION_ERROR       -228   /* Call to initialze ADC module failed. */
#define  ERR_ADC_CHANNEL_INITIALIZATION_ERROR -229 /* Call to initialze ADC channel failed. */
#define  ERR_ADC_ENABLE_ERROR               -230   /* Call to enable the ADC module failed. */
#define  ERR_ADC_START_CONVERSION_ERROR     -231   /* Call to adc_user_trigger_start()_failed. */

#define  ERR_DAC_INITIALIZIATION_ERROR      -240   /* HAL_DAC_Init() failed to initialize the hardware   */
#define  ERR_DAC_NOT_SUPPORTED_ON_THIS_MCU  -241   /* No native DAC is supported on this  STM32 MCU */
#define  ERR_DAC_CHANNEL_NUM_OUT_OF_RANGE   -242   /* Only DAC_CHANNEL_1 and DAC_CHANNEL_2 are supported */
#define  ERR_DAC_UNSUPPORTED_TRIGGER_TYPE   -243   /* trigger_type not supported/valid */
#define  ERR_DAC_CHANNEL_CONFIG_ERROR       -244   /* HAL_DAC_ConfigChannel() failed to initialize the channel */
#define  ERR_DAC_CHANNEL_DMA_CONFIG_ERROR   -245   /* HAL_DAC_ConfigChannel() failed to initialize the DMA for that DAC channel */
#define  ERR_DAC_START_ERROR                -246   /* HAL_DAC_Start_DMA() failed to start up the channel */

#define  ERR_I2C_MODULE_ID_OUT_OF_RANGE     -250   /* i2c_module id is not within vazlid range of 0 to 6 */
#define  ERR_I2C_MODULE_NUM_NOT_SUPPORTED   -251   /* That I2C Module Number is not supported on this platform */
#define  ERR_I2C_INVALID_I2C_MS_MODE        -252   /* i2c_ms_mode is not I2C_MASTER/I2C_SLAVE   */
#define  ERR_I2C_EXCEEDS_MAX_BAUD_RATE      -253   /* baud_rate exceeds max allowed by the chip */
#define  ERR_I2C_PIN_ID_NOT_SUPPORTED       -254   /* scl_pin_id or sda_pin_id on i2c_Init() is not valid for this I2C module */
#define  ERR_I2C_INVALID_REQUEST            -255   /* Operation is not supported in this mode   */

#define  ERR_SPI_NUM_OUT_OF_RANGE           -260   /* SPI Number is ouside the valid range of 0 to nn   */
#define  ERR_SPI_MODULE_NUM_NOT_SUPPORTED   -261   /* That SPI Module Number is not supported on this platform */
#define  ERR_SPI_PIN_ID_NOT_SUPPORTED       -262   /* sclk_pin_id or miso_pin_id or mosi_pin_id on spi_Init() is not valid for this SPImodule */

#define  ERR_PWM_MODULE_ID_OUT_OF_RANGE     -270
#define  ERR_PWM_MODULE_INITIALIZE_FAILED   -271
#define  ERR_PWM_MODULE_NOT_INITIALIZED     -272
#define  ERR_PWM_MODULE_NO_COMPLEMENTARY    -273   /* Complementary mode not supported by this module */
#define  ERR_PWM_MODULE_NO_DEADTIME         -274   /* Dead Time not supported by this module */
#define  ERR_PWM_MODULE_NO_PHASE            -275   /* Phase Shifting not supported by this module */
#define  ERR_PWM_COUNT_MODE_INVALID         -276   /* Invalid value specified for count_mode */
#define  ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE   -277   /* Channel number is invalid e.g. < 0 or > 25 */
#define  ERR_PWM_CHANNEL_NUM_NOT_SUPPORTED  -278   /* That channel number is not supported on this module */
#define  ERR_PWM_CHANNEL_CONFIG_FAILED      -279
#define  ERR_PWM_CHANNEL_COMPLE_CHAN_NO_CCR -280   /* issuing pwm_set_duty_cycle to one of the
                                                   ** "complementary channels" (5-8) is not allowed.
                                                   ** They are always driven off the duty cycle
                                                   ** from the "primary channel" (1-4) instead.   */
#define  ERR_PWM_CHANNEL_START_FAILED       -281
#define  ERR_PWM_INVALID_TIMER_MODE         -282   /* bad mode parameter value on board_timerrpwm_config_channel() */

#define  ERR_TIMER_NUM_OUT_OF_RANGE         -285   /* Timer Module Number is ouside the valid range of 0 to 22   */
#define  ERR_TIMER_NUM_NOT_SUPPORTED        -286   /* That Timer Module Number is not supported on this platform */
#define  ERR_TIMER_INVALID_COUNTER_TYPE     -287   /* counter_type in timer_Init() call is invalid  */
#define  ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE -288   // REDUNDANT WITH PWM
#define  ERR_TIMER_CHANNEL_CONFIG_FAILED    -289   // REDUNDANT WITH PWM
#define  ERR_TIMER_CHANNEL_START_FAILED     -290   // REDUNDANT WITH PWM
#define  ERR_TIMER_ADC_NOT_INITIALIZED      -291   // A trigger_ADC_Start() call was issued, but the ADC
                                                   // has not been initialized yet.
#define  ERR_TIMER_DAC_NOT_INITIALIZED      -292   // A trigger_DAC_Start() call was issued, but the DAC
                                                   // has not been initialized yet.
#define  ERR_TIMER_TRIGGER_MISMATCH         -293   // The trigger type specified on the trigger_ADC/DAC_Start() call
                                                   // does not match what was issued on the init_ADC() or init_DAC()
#define  ERR_TIMER_INVALID_TRIGGER_TYPE     -294   // The trigger type specified on trigger_ADC/DAC_Start() call was invalid

#define  ERR_UART_MODULE_NUM_OUT_OF_RANGE   -300   /* Module Number is ouside the valid range of 0 to 6    */
#define  ERR_UART_MODULE_NOT_SUPPORTED      -301   /* That Module Number is not supported on this platform */
#define  ERR_UART_PIN_ID_NOT_SUPPORTED      -302   /* tx_pin_id or rx_pin_id on usart_Init() is not valid for this UART module */
#define  ERR_UART_RCV_TIMED_OUT             -305   /* uart_Read_String() or uart_Read_Bytes() timed out    */

#define  ERR_VTIMER_ID_OUT_OF_RANGE         -320   /* VTIMER id ranges is 0 to 9. Is outside that range */
#define  ERR_VTIMER_IN_USE                  -321   /* requested VTIMER has already been started and is in use */
#define  ERR_VTIMER_MILLISEC_EXCEED_LIMIT   -322   /* max limit for timer_duration_millis is 1000000000 */

#define  ERR_WIFI_MODULE_NUM_OUT_OF_RANGE   -350   /* Module Number is ouside the valid range of 0 to 6 */
#define  ERR_WIFI_SPI_WRITE_FAILED          -352   /* Arduino WiFi Shield error codes. Write to Shield failed */
#define  ERR_WIFI_SPI_READ_FAILED           -353   /* Read from Shield failed   */
#define  ERR_WIFI_MSG_HDR_INVALID_START     -354   /* Invalid MSG_HDR START_CMD */
#define  ERR_WIFI_MSG_MAX_LENGTH_EXCEEDED   -355   /* Length of MSG from Ardu Wifi processor exceedesd max allowed */
#define  ERR_WIFI_MSG_MAX_PARMS_EXCEEDED    -356   /* Number of parms in MSG from Ardu Wifi processor > max allowed */
#define  ERR_WIFI_MSG_HDR_INVALID_END       -357   /* Invalid MSG_HDR END_CMD   */







#if defined(STM32L476xx)
//******************************************************************************
//******************************************************************************
//                            STM32L476xx    Channel Mappings
//******************************************************************************
//******************************************************************************

                 //----------------------
                 //   PWM definitions
                 //----------------------
#define  PWM_MODULE_1   TIMER_1   // TIM1  advanced PWM
#define  PWM_MODULE_2   TIMER_2   // TIM2  intermediate PWM (no Comple/DeadTuime)
#define  PWM_MODULE_3   TIMER_3   // TIM3      ditto
#define  PWM_MODULE_4   TIMER_4   // TIM4        "
#define  PWM_MODULE_5   TIMER_5   // TIM5        "
#define  PWM_MODULE_6   TIMER_16  // TIM16       "
#define  PWM_MODULE_7   TIMER_17  // TIM17       "
#define  PWM_MODULE_8   TIMER_8   // TIM8  advanced PWM

//#define  PWM_CHANNEL_1      1     // Up to 4 channels per PWM Module
//#define  PWM_CHANNEL_2      2
//#define  PWM_CHANNEL_3      3
//#define  PWM_CHANNEL_4      4

#endif                              // end  defined(STM32L476xx)



#if defined(STM32F030x8) \
 || defined(STM32F070xB) || defined(STM32F072xB) || defined(STM32F091xC) \
 || defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) \
 || defined(STM32F303xE) || defined(STM32F334x8) \
 || defined(STM32F103xB) || defined(STM32L053xx) || defined(STM32L152xE) || defined(STM32L152xC)


//******************************************************************************
//******************************************************************************
//                  STM32  F0_72  and  F4_01   User API   specific mappings
//                         F3_34
//******************************************************************************
//******************************************************************************

                 //----------------------
                 //   PWM definitions
                 //----------------------
#define  PWM_MODULE_1   TIMER_1   // TIM1  advanced PWM
#define  PWM_MODULE_2   TIMER_2   // TIM2  intermediate PWM (no Comple/DeadTuime)
#define  PWM_MODULE_3   TIMER_3   // TIM3                    ditto
#define  PWM_MODULE_4   TIMER_14  // TIM14 (F0)  TIM4  (F4)  ditto
#define  PWM_MODULE_5   TIMER_15  // TIM15 (F0)  TIM5  (F4)  ditto
#define  PWM_MODULE_6   TIMER_16  // TIM16 (F0)  TIM9  (F4)  ditto
#define  PWM_MODULE_7   TIMER_17  // TIM17 (F0)  TIM10 (F4)  ditto

//#define  PWM_CHANNEL_1      1     // Up to 4 channels per PWM Module
//#define  PWM_CHANNEL_2      2
//#define  PWM_CHANNEL_3      3
//#define  PWM_CHANNEL_4      4

            //-----------------------------------------------------
            //           Common SPI Configurations
            //-----------------------------------------------------
            // low order 4 bits denote the SPI variation.
            // high order 4 bits denote the actual module 0x001x - 0x006x
#define  SPI_ID_1_A           0x0011        // SPI 1 using pins PA5/PA6/PA7
#define  SPI_ID_1_B           0x0012        // SPI 1 using pins PB3/PA6/PA7
#define  SPI_ID_1_C           0x0013        // SPI 1 using pins PB3/PB4/PB5
#define  SPI_ID_1_D           0x0014        // SPI 1 using pins PA5/PA11/PA12     - Not on F4.
#define  SPI_ID_1_E           0x0015        // SPI 1 using pins PE13/PE14/PE15

#define  SPI_ID_2_A           0x0021        // SPI 2 using pins PB10/PB14/PB15
#define  SPI_ID_2_B           0x0022        // SPI 2 using pins PB13/PB14/PB15
#define  SPI_ID_2_C           0x0023        // SPI 2 using pins PB13/PC2/PC3
#define  SPI_ID_2_D           0x0024        // SPI 2 using pins PD3/PB14/PB15
#define  SPI_ID_2_E           0x0025        // SPI 2 using pins PD3/PC2/PC3
#define  SPI_ID_2_F           0x0026        // SPI 2 using pins PD1/PD3/PD4
#define  SPI_ID_2_I           0x0029        // SPI 2 using pins PI1/PB14/PB14 - Default for F7 "Arduino Headers"  -- F7 --

#define  SPI_ID_3_B           0x0032        // SPI 3 using pins PB3/PB4/PB5
#define  SPI_ID_3_C           0x0033        // SPI 3 using pins PC10/PC11/PC12

#endif                   // ====-- generic F0 / F1 / F4 -----------
                         //-----------------------------------------

// Ardu   ADC
// Name  Mappings  F3_03      F3_34      F0_91      F4_01
//  A0    PA_0     ADC1_IN1   ADC1_IN1   ADC_IN0    ADC1_IN0
//  A1    PA_1     ADC1_IN2   ADC1_IN2   ADC_IN1    ADC1_IN1
//  A2    PA_4     ADC2_IN1   ADC2_IN1   ADC_IN4    ADC1_IN4
//  A3    PB_0     ADC3_IN12  ADC1_IN11  ADC_IN8    ADC1_IN8 => ADC1 / ADC_CHANNEL_8
//  A4    PC_1     ADC12_IN7  ADC12_IN7  ADC_IN11   ADC1_IN11
//  A5    PC_0     ADC12_IN6  ADC12_IN6  ADC_IN10   ADC1_IN10



#if defined(USE_STM32F4XX_NUCLEO) || defined(NUCLEO_F030)  \
 || defined(NUCLEO_F070) || defined(NUCLEO_F072) || defined(NUCLEO_F091)
//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
//               STM32             STM32  Chip  Morpho   assoc     Grove
//              Channel #          Name   Pin    Pin    Ard Pin#   Conn
#define  A0        0            /* PA_0    3              A0        J9  */
#define  A1        1            /* PA_1    4              A1       J10  */
#define  A2        4            /* PA_4    5              A2       J11  */
#define  A3        8            /* PB_0    6              A3       J12  */
#define  A4       11            /* PC_1    7              A4        -   */
#define  A5       10            /* PC_0    8              A5        -   */
                      // The above pins are connected directly to the "Arduino"
                      // headers, and accessible on the silk-screen labelled
                      // A0 - A5 on bottom left Arduino connector on the board.

                      // Assign the rest of the 10 channels on the STM32.
                      // Note these are only hooked up to the "Morpho" connector
#define  A6        2            /* PA_2    -               -        -   */
#define  A7        3            /* PA_3    -               -        -   */
#define  A8        5            /* PA_5    -               -        -   */
#define  A9        6            /* PA_6    -               -        -   */
#define  A10       7            /* PA_7    -               -        -   */
#define  A11       9            /* PB_1    -               -        -   */
#define  A12      12            /* PC_2    -               -        -   */
#define  A13      13            /* PC_3    -               -        -   */
#define  A14      14            /* PC_4    -               -        -   */
#define  A15      15            /* PC_5    -               -        -   */
#endif                   // ----  ADC for  F0 and F4 ---


#if defined(NUCLEO_F303) || defined(NUCLEO_F334)
//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
//               STM32                 STM32     assoc     Grove
//              Channel #             Name Pin   Ard Pin#   Conn
#define  A0        1               /* PA_0  3      A0        J9  */
#define  A1        2               /* PA_1  4      A1       J10  */
#define  A2       21               /* PA_4  5      A2       J11  */
#define  A3       32               /* PB_0  6      A3       J12  */
#define  A4        7               /* PC_1  7      A4        -   */
#define  A5        6               /* PC_0  8      A5        -   */
#endif                   // ----  ADC for  F3 -----


// #define ADC_CHANNEL_TEMPSENSOR  ADC_CHANNEL_16
// #define ADC1_BASE             (APB2PERIPH_BASE + 0x2000)
// #define ADC2_BASE             (AHB3PERIPH_BASE + 0x00000100)
// #define ADC1      <-- Exampl  ((ADC_TypeDef *) ADC1_BASE)
// #define ADC2                  ((ADC_TypeDef *) ADC2_BASE)
// #define ADC3                  ((ADC_TypeDef *) ADC3_BASE)
// #define ADC4                  ((ADC_TypeDef *) ADC4_BASE)
// #define ADC1_2_COMMON         ((ADC_Common_TypeDef *) ADC1_2_COMMON_BASE)
// #define ADC3_4_COMMON         ((ADC_Common_TypeDef *) ADC3_4_COMMON_BASE)

//#endif                    // defined(NUCLEO_F103)  ...  defined(NUCLEO_F303)




//******************************************************************************
//******************************************************************************
//                            F3_02
//
//                    LOGICAL  PIN  and  CHANNEL  MAPPINGS
//
//******************************************************************************
//******************************************************************************

#if defined(NUCLEO_F302)

//   Logical ADC
//   Channel number
//     to use in
//    your program   Physical Port and Channel that pin corresponds to on MCU
//      ------      ------------------------------------------------------------
                                                   /*      assoc    03 Grove */
                                                   /*     Ard Pin# ADC  Conn */
#define  Adc0       3,ADC_CTL_CH0                  /* PA_0  A0  3    1   J9  */
#define  Adc1       4,ADC_CTL_CH1                  /* PA_1  A1  4    1  J10  */
#define  Adc2       5,ADC_CTL_CH2                  /* PA_4  A2  5    2  J11  */
#define  Adc3       6,ADC_CTL_CH3                  /* PB_0  A3  6    3  J12  */
#define  Adc4       7,ADC_CTL_CH4                  /* PC_1  A4  7   12   -   */
#define  Adc5       8,ADC_CTL_CH5                  /* PC_0  A5  8   12   -   */
#endif                         // defined(NUCLEO_F302)


// 06/02/15 - ??? !!! THIS DUPLICATES LOGIC ALREADY IN user_api_stm32_F3.h

#if defined(NUCLEO_F303) || defined(NUCLEO_F334)
//******************************************************************************
//******************************************************************************
//                  STM32  F3_xx    User API   specific mappings
//******************************************************************************
//******************************************************************************


#endif                         // defined(NUCLEO_F303) || defined(NUCLEO_F334)




// 06/02/15 - ??? !!! THIS DUPLICATES LOGIC ALREADY IN user_api_stm32_F4.h
//******************************************************************************
//******************************************************************************
//                            F4_01
//
//                    LOGICAL  PIN  and  CHANNEL  MAPPINGS
//
//******************************************************************************
//******************************************************************************

#if defined(USE_STM32F4XX_NUCLEO) \
 || defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)

//******************************************************************************
//******************************************************************************
//                  STM32  F4_01 / F4_11   User API   specific mappings
//******************************************************************************
//******************************************************************************

                 //----------------------
                 //   PWM definitions
                 //----------------------
#define  PWM_MODULE_1   TIMER_1   // TIM1  advanced PWM
#define  PWM_MODULE_2   TIMER_2   // TIM2  intermediate PWM (no Comple/DeadTuime)
#define  PWM_MODULE_3   TIMER_3   // TIM3       ditto
#define  PWM_MODULE_4   TIMER_4   // TIM4       ditto
#define  PWM_MODULE_5   TIMER_5   // TIM5       ditto

//#define  PWM_CHANNEL_1      1   // Up to 4 channels per PWM Module
//#define  PWM_CHANNEL_2      2
//#define  PWM_CHANNEL_3      3
//#define  PWM_CHANNEL_4      4

#endif                            // defined(NUCLEO_F401)


#endif                          //  __USER_API_H__

