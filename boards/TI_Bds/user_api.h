/********1*********2*********3*********4*********5*********6*********7**********
*                                                                 TI Launchpad
*                                  user_api.h
*
*  Provide a higher level user API for newbies.
*
*  It provides the high level call prototypes, and then imports the
*  associated pin_map file for the specific platform/Launchpad (Tiva, MSP432, .)
*
*  More experienced programmers can directly use the board_xxx calls defined in
*  boarddef.h or in the platform's vendor DriverLib APIs.
*
*
* History:
*   xx/xx/xx - Created.  Texas Instruments
*   10/12/15 - Revisedto match changes in STM32/MAX10 part of code tree. Duq
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

























































//------------------------------------------------------------------------------
//                                Simple Pin Mapping
//
// Modelled after Arduino/Energia/etc to make it easy for newbies to use
// the MCU's GPIOs, ADCs, PWMs, etc without having to know the details of
// modules/ports/pins. It maps those into a simple format that allows
// the board_xxx calls to be invoked via macros.
//
// Also because all Launchpads use a similar layout, where pins in the same
// position on the board (usually) provide the same function (GPIO, ADC, PWM),
// this enhances program portability across different Launchpads.
//
// The pins are mapped in a couter-clockwise fashion, starting in the 
// upper left hand corner, and ending at the upper right hand corner.
// Since pin 1 on Launchpads (top pin upper left) is always VCC, and
// pin 20 (top pin uuper right) is always Ground on Launchpads, they are omitted.
//
// When 40-pin Lauchpads are used, they operate in a similar fashion, where the
// top upper left of the second row of pins starts at xx and the top upper right
// ends at pin yy
//
// Rather than providing a number for each pin (and the associated table lookup
// overhead, labels of "PinN" (where N is the number of the pin).
//
// No, I am not going to directly emulate Arduino/Energia. They are tuned for 
// newbies and have a nuumber of restrictions on what can be done, due to the
// simple APIs. E.g. you cannot take advantage of ADC sequencing of multiple
// channels at once, sophisticated PWM options, ... With those platforms. You
// have to DIY it and start going completely around the entire Arduino API.
//
// This is meant to be an intermediate approach to allow newbies to get
// started with a much lower pain threshhold, but be able ramp up as they
// gain experience to use more sophisticated approaches useing the lower
// level board_xxx interfaces for such things as Motor control, Power Control,
// LED control, etc.
//
//              IN FUTURE, MAKE THIS A SEPARATE FILE FOR EACH BOARD
//              THIS IS AUTO-included by boarddef.h
//-------------------------------------------------------------------------------

#ifndef  __USER_API_H__
#define  __USER_API_H__

#include "boarddef.h"                // pull in defs for the MCU board being used

              //-------------------------------
              // Generic GPIO constants/flags
              //-------------------------------
#define  GPIO_INPUT              0
#define  GPIO_OUTPUT             1

#ifndef GPIO_NOPULL
#define  GPIO_NOPULL             0
#define  GPIO_PULLUP             1
#define  GPIO_PULLDOWN           2
#define  GPIO_OPEN_DRAIN         3
#endif

#define  GPIO_RUPT_MODE_RISING   0
#define  GPIO_RUPT_MODE_FALLING  1
#define  GPIO_RUPT_MODE_BOTH     2

#define  PIN_HIGH             0xFF
#define  PIN_LOW      ( ! PIN_HIGH)


              //-------------------------------
              // Generic PWM constants/flags
              //-------------------------------
//#define  PWM_COUNT_DOWN             1  // pwm_init_module count_mode   NEED TO BE REMOVED WVD 07/11/15
//#define  PWM_COUNT_UP               2
//#define  PWM_COUNT_UPDOWN           3  // count up then down (aka "center mode")


              //-------------------------------
              // Generic SPI constants/flags
              //-------------------------------
#define  SPI_MODE_0              0
#define  SPI_MODE_1              1
#define  SPI_MODE_2              2
#define  SPI_MODE_3              3


              //-------------------------------
              //   I2C constants
              //-------------------------------
#define  I2C_MASTER   1           /* ms_i2c_mode values */
#define  I2C_SLAVE    2



                         //---------------
                         //---------------
                         //  System APIs
                         //---------------
                         //---------------
#define  sys_Init(mcu_speed)      { board_init(mcu_speed); /* initialize board: setup clocks, GPIOs,...*/ \
                                    board_systick_timer_config(); /* turn on 1 ms system timer */ }
#define  sys_Delay(ms_value)        board_delay_ms(ms_value)
#define  sys_Delay_Millis(ms_value) board_delay_ms(ms_value)
#define  frequency_to_period_ticks(freq)   board_frequency_to_period_ticks (freq)


                         //---------------
                         //---------------
                         //   GPIO APIs
                         //---------------
                         //---------------
#define  pin_Config(pinX,dir)       board_gpio_pin_config(pinX,dir,GPIO_NOPULL)
#define  pin_High(pinX)             inline_pin_High(pinX)
#define  pin_Low(pinX)              inline_pin_Low(pinX)
#define  pin_Toggle(pinX)           inline_pin_Toggle(pinX)
#define  pin_Read(pinX)             inline_pin_Read(pinX)


                          //---------------
                          //---------------
                          //  inline  APIs
                          //---------------
                          //---------------
#define  pin_High(pinX)       inline_pins_High(pinX)
#define  pin_Low(pinX)        inline_pins_Low(pinX)
#define  pin_Toggle(pinX)     inline_pins_Toggle(pinX)
#define  pin_Read(pinX)       inline_pin_Value(pinX)

inline void  inline_pins_High (int gpio_port_num,  unsigned long pins_mask);
inline void  inline_pins_Low (int gpio_port_num,   unsigned long pins_mask);
inline void  inline_pins_Toggle (int gpio_port_num,  unsigned long pins_mask);
//inline int   inline_pins_Read (int gpio_port_num,  unsigned long pins_mask);
//inline int   inline_pin_Value (int gpio_num,  unsigned long pin_mask);


                         //---------------
                         //---------------
                         //    ADC APIs
                         //---------------
                         //---------------
#define  adc_Init(mod_id,trigger_type,flags)  board_adc_init(mod_id,ADC_DEFAULT_CLOCK,trigger_type,flags)
#define  adc_Config_Channel(mod_id,chan,last) board_adc_config_channel(mod_id,chan,ADC_AUTO_SEQUENCE,ADC_AUTO_STEP,last,0)
#define  adc_Check_All_Complete(module_id)    board_adc_check_conversions_done(module_id,ADC_AUTO_SEQUENCE)
#define  adc_Enable(module_id)                board_adc_enable(module_id,ADC_AUTO_SEQUENCE)
#define  adc_Disable(module_id)               board_adc_disable(module_id,ADC_AUTO_SEQUENCE)
#define  adc_Read(module_id,channel_results)  while ( ! board_adc_check_conversions_done(module_id,ADC_AUTO_SEQUENCE)) ; \
                                               board_adc_get_results(module_id,ADC_AUTO_SEQUENCE,channel_results)
#define  adc_Set_Callback(module_id,callback_rtn,callback_parm) \
                                              board_adc_set_callback(module_id,callback_rtn,callback_parm)
#define  adc_User_Trigger_Start(module_id)    board_adc_user_trigger_start(module_id,ADC_AUTO_SEQUENCE)

                  // the following is to allow for platform specific ADC options
#define  adc_Set_Option(module_id,option_type,opt_flags1,opt_flags2)  \
                                              board_adc_set_option(module_id,option_type,opt_flags1,opt_flags2)

                  // the following is for more experienced programmers that want
                  // to have more fine grain control of the sequencing process. 
                  // It is used in lieu of adc_Config_Channel().
#define  adc_Config_Channel_Seq(adc_module,pinX,seq_id,step_num,last,flags)  board_adc_config_channel(adc_module,pinX,seq_id,step_num,last,flags)


            // Valid values for module_id used on all adc_ calls
#define  ADCMD                 0    /* ADC "Default Module" id - single ADC module MCUs */
#define  ADCM1                 1    /* ADC Module 1 - advanced, multi-ADC module MCUs (STM32_F3, C2000, ...) */
#define  ADCM2                 2    /* ADC Module 2 -    ditto     */
#define  ADCM3                 3    /* ADC Module 3 -    ditto     */

            // Valid values for adc_init() clock option: 0, or a valid CPU clock
            // clock speed of 1000000 (1 MHz) up to the max speed to the MCU.
            // Using 0 (default clock speed), always sets the MCU to its max speed
#define  ADC_DEFAULT_CLOCK     0    /* use default clocking on adc_init()         */

            // Valid values for adc_init() flags parm
#define  ADC_TRIGGER_RISING    0    /* Trigger ADC on rising  edge of Timer/GPIO trigger */
#define  ADC_TRIGGER_FALLING   1    /* Trigger ADC on falling edge of Timer/GPIO trigger */
#define  ADC_TRIGGER_RISEFALL  2    /* Trigger ADC on rising and falling edge of Timer/GPIO trigger */

            // Valid values for adc_init() clock option: 0, or a valid CPU clock

            // Valid values for adc_Config_Channel last flag
#define  ADC_NOT_LAST          0    /* not the last channel, more to configure    */
#define  ADC_LAST              1    /* this is the last channel to be configured  */

            // Valid values for sequencer_id
#define  ADC_AUTO_SEQUENCE     4    /* automatically assign the sequencer to use  */
#define  ADC_SEQUENCER_0       0    /* explicit assign by experienced programmers */
#define  ADC_SEQUENCER_1       1    /*                ditto                       */
#define  ADC_SEQUENCER_2       2    /*                 " "                        */
#define  ADC_SEQUENCER_3       3

            // Valid values for step number
#define  ADC_AUTO_STEP         9    /* automatically assign the step number entry to use */
#define  ADC_STEP_0            0    /* explicit assign by experienced programmers */
#define  ADC_STEP_1            1    /*                ditto                       */
#define  ADC_STEP_2            2    /*                 " "                        */
#define  ADC_STEP_3            3
#define  ADC_STEP_4            4
#define  ADC_STEP_5            5
#define  ADC_STEP_6            6
#define  ADC_STEP_7            7    /* only sequencer 0 allows 8 steps           */

            //---------------------------------------------------------------------------
            // Valid values for adc_Init() trigger_type  (How the ADCs will be triggered)
            //
            // Note: not all values are supported on each platform. See list below
            //       that breaks out what each MCU platform supports
            //---------------------------------------------------------------------------
#define  ADC_TRIGGER_USER_APP     0x0400  /* user application triggers, via calls to adc_Trigger_Start() */
#define  ADC_TRIGGER_GPIO_PIN     0x0300  /* A GPIO pin rising up is trigger source. flags contains pin # */
#define  ADC_TRIGGER_COMPARATOR_0 0x0380  /* Compator 0 is trigger source. flags contain conditions */
#define  ADC_TRIGGER_COMPARATOR_1 0x0390  /* Compator 1 is trigger source */
#define  ADC_TRIGGER_COMPARATOR_2 0x03A0  /* Compator 2 is trigger source */

//#define  ADC_TRIGGER_PWM_0        0x0200  /* PWM 0 compare is trigger source. flags contain CCR A/B */
//#define  ADC_TRIGGER_PWM_1        0x0210  /* PWM 1 compare is trigger source */
//#define  ADC_TRIGGER_PWM_2        0x0220  /* PWM 2 compare is trigger source */
//#define  ADC_TRIGGER_PWM_3        0x0230  /* PWM 3 compare is trigger source */
//#define  ADC_TRIGGER_PWM_4        0x0240  /* PWM 4 compare is trigger source */
//#define  ADC_TRIGGER_PWM_5        0x0250  /* PWM 5 compare is trigger source */
//#define  ADC_TRIGGER_GPIO_PIN     0x0300  /* A GPIO pin rising up is trigger source. flags contains pin # */

                         //---------------
                         //---------------
                         //   DAC APIs
                         //---------------
                         //---------------
#define  dac_Init(module_id)                    board_dac_init(module_id,DAC_DEFAULT_CLOCK,0)
#define  dac_Config_Channel(module_id,chan,trigger_type,sps_frequency,flags)  board_dac_config_channel(module_id,chan,trigger_type,sps_frequency,0)
#define  dac_Gen_Sample_Table(wave_type,table_buf,num_steps)  board_dac_gen_sample_table(wave_type,table_buf,num_steps)
#define  dac_Set_Sample_Table(module_id,chan,table_buf,num_steps)   board_dac_set_sample_table(module_id,chan,table_buf,num_steps)
#define  dac_Check_All_Complete(module_id,chan) board_dac_check_conversions_done(module_id,chan,0)
#define  dac_Enable_Channel(module_id,chan)     board_dac_enable_channel(module_id,chan,0)
#define  dac_Disable_Channel(module_id,chan)    board_dac_disable_channel(module_id,chan,0)

            // Valid values for module_id used on all dac_ calls
#define  DACMD                  0    /* DAC "Default Module" id - single DAC module MCUs */
#define  DACM1                  1    /* DAC Module 1 - advanced, multi-DAC module MCUs   */
#define  DACM2                  2    /* DAC Module 2 -    ditto     */

               // chanX values to use for dac_Config_Channel(), dac_Enable(),...
#define  DAC_CHAN_1            1
#define  DAC_CHAN_2            2

               // wave_type values for dac_Gen_Sample_Table() call
#define  DAC_GEN_SINEWAVE      1
#define  DAC_GEN_SAWTOOTH      2
#define  DAC_GEN_TRIANGLE      3
#define  DAC_GEN_SQUAREWAVE    4

#define  DAC_DEFAULT_CLOCK     0    /* use default clocking on dac_init()         */

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
#define  DAC_TRIGGER_TIMER_CHAN_1 0x0560  /* trigger off Timer 6, used for DAC */
#define  DAC_TRIGGER_TIMER_CHAN_2 0x0570  /* trigger off Timer 7, used for DAC */
#define  DAC_TRIGGER_TIMER_15     0x05F0  /* trigger off standard Timer 15 period expire */
#define  DAC_TRIGGER_EXTI_GPIO    0x0690  /* trigger off EXTI 9 (GPIO) rising edge */


                          //----------------------
                          //----------------------
                          //      PWM  APIs
                          //----------------------
                          //----------------------
#define  PWM_COUNT_MODE   TIMER_PERIODIC_COUNT_UP
#define  pwm_Init(module_id,period,flags)   board_timerpwm_init(module_id,PWM_COUNT_MODE,period,0,flags)
#define  pwm_Config_Channel(module_id,chan,duty_cycle,flags)  board_timerpwm_config_channel(module_id,chan,duty_cycle,TIMER_MODE_PWM,flags)
//int    board_pwm_config_channel_pair(module_id,channelA_id,channelB_id,0)
#define  pwm_Disable(module_id)             board_timerpwm_disable(module_id,0)
#define  pwm_Enable(module_id)              board_timerpwm_enable(module_id,0)
#define  pwm_get_Duty_Cycle(module_id,chan) board_timerpwm_get_duty_cycle(module_id,chan)
#define  pwm_get_Period(module_id)          board_timerpwm_get_period(module_id)
#define  pwm_set_Duty_Cycle(module_id,chan,duty) board_timerpwm_set_duty_cycle(module_id,chan,duty,0)
#define  pwm_set_Period(module_id,period)        board_timerpwm_set_period(module_id,period,0)
#define  pwm_set_Dead_Time(module_id,deadtime)   board_timerpwm_set_dead_time(module_id,deadtime,deadtime)
#define  pwm_set_Phase(chanX,phase_offset)       board_timerpwm_set_phase(chanX, phase_offset)
#define  pwm_set_Channel_Output(module_id,chan,duty) board_timerpwm_set_channel_output(module_id,chan,output_mode,0)
// PWM module_id values (PWM_MODULE_0, PWM_MODULE_1, ...) is product dependent, and found in XXXX

               // allowed  values for channel numbers in pwm_Config_Channel(), etc
               // Note: These channel numbers match the equivalent TIMER_CHANNEL_x numbers
#define  PWM_CHANNEL_0     0
#define  PWM_CHANNEL_1     1
#define  PWM_CHANNEL_2     2
#define  PWM_CHANNEL_3     3
#define  PWM_CHANNEL_4     4
#define  PWM_CHANNEL_5     5
#define  PWM_CHANNEL_6     6

               // allowed values for flags in pwm_Config_Channel() calls
#define  PWM_NO_FLAGS               0
#define  PWM_COMPLEMENTARY_OUTPUTS  1


                         //------------------------------------
                         //------------------------------------
                         //           TIMER  APIs
                         //
                         // USER_TIMER_2 and USER_TIMER_3 are
                         // universal across all boards.
                         // Other timers are board dependent.
                         //------------------------------------
                         //------------------------------------
#define  timer_Init(module_id,counter_type,period_value,flags) \
                              board_timerpwm_init(module_id,counter_type,period_value,TIMER_DEFAULT_CLOCK,flags)

                         // CONVENIENCE FUNTION: Automatically init and enable a simple Timer (No CCRs used)
#define  timer_AutoStart(module_id,counter_type,period_value,interrupt_flags) {board_timerpwm_init(tmrmod_id,counter_type,period_value,TIMER_DEFAULT_CLOCK,0); \
                              board_timerpwm_enable(tmrmod_id,interrupt_flags);}

                         // CONVENIENCE FUNTION: Automatically config a Timer used for ADC triggering
                         // Note: the ADC _must_ be initialized via adc_Init() _before_ timer_ADC_Trigger_Start()
                         //       is called, as per the STM32 Tech Ref
#define  timer_ADC_Trigger_Start(tmrmod_id,trig_type,sps_frequency,flags)  {board_timerpwm_init(tmrmod_id,TIMER_PERIODIC_COUNT_UP,(frequency_to_period_ticks(sps_frequency)),TIMER_DEFAULT_CLOCK,0); \
                              board_timerpwm_config_trigger_mode(tmrmod_id,trig_type,(flags|TIMER_TRIGGER_MODE)); \
                              if (flags&TIMER_AUTOSTART) \
                                 board_timerpwm_enable(tmrmod_id,0);}

                         // CONVENIENCE FUNTION: Automatically config a Timer used for DAC triggering
                         // Note: the DAC _must_ be initialized via dac_Init() _before_ timer_DAC_Trigger_Start()
                         //       is called, as per the STM32 Tech Ref
#define  timer_DAC_Trigger_Start(module_id,trig_type,dac_frequency,num_steps,flags) {board_timerpwm_init(module_id,TIMER_PERIODIC_COUNT_UP,(frequency_to_period_ticks(dac_frequency)*num_steps),TIMER_DEFAULT_CLOCK,TIMER_DISABLE_DAC_PRESCALING); \
                              board_timerpwm_config_trigger_mode(module_id,trig_type,(flags|TIMER_TRIGGER_MODE)); \
                              if (flags & TIMER_AUTOSTART) \
                                 board_timerpwm_enable(module_id,0);}

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
#define  timer_Set_CCR_Duty(module_id,CCR_Num,CCR_compare_value,flags) \
                              board_timerpwm_set_duty_cycle(module_id,CCR_Num,CCR_compare_value,flags)
#define  timer_Set_Callback(mod_id,callback_rtn,callback_parm)  board_timerpwm_set_callback(mod_id,callback_rtn,callback_parm);
#define  timer_Set_Period(module_id,new_period_value,flags) board_timerpwm_set_period(module_id,new_period_value,flags)



#define  TIMER_DEFAULT_CLOCK       0   /* use default clock source on board_timer_init()  */

#define  TIMER_DISABLE_DAC_PRESCALING  0x0001    /* use on board_timerpwm_init() and
                                                 ** board_timerpwm_set_period() flags parm for DAC timers */

               // valid  values for counter_type in timer_Init()
#define  TIMER_PERIODIC_COUNT_DOWN    1
#define  TIMER_PERIODIC_COUNT_UP      2
#define  TIMER_PERIODIC_COUNT_UPDOWN  3       /* aka Center Aligned mode */
#define  TIMER_ONE_SHOT_COUNT_UP      4
#define  TIMER_ONE_SHOT_COUNT_DOWN    5
#define  TIMER_INPUT_CAPTURE          6

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
#define  TIMER_TRIGGER_MODE      0x0400    // Timer is to operate as a trigger

               // valid values for MODE parameter on board_timerpwm_config_channel()
               // Note: rollover means the Timer's counter has reached its max period value, and then rolls over to 0
#define  TIMER_CCR_TIMER_ONLY         0    /* no CCR output to GPIOs. Run CCR as timer only */
#define  TIMER_MODE_OC_PASSTHRU       0    /* eg. OUTMOD_0 / Frozen    GPIO = Polarity setting */
#define  TIMER_MODE_OC_SET            1    /* eg. OUTMOD_1 / GO_ACTIVE GPIO = Set on when CCRx hit, and stays on    */
#define  TIMER_MODE_OC_TOGGLE_RESET   2    /* eg. OUTMOD_2 /  -        GPIO = Toggle when CCRx hit, Resets on rollover */
#define  TIMER_MODE_OC_SET_RESET      3    /* eg. OUTMOD_3 /  -        GPIO = Set when CCRx hit, Resets on rollover */
#define  TIMER_MODE_OC_TOGGLE         4    /* eg. OUTMOD_4 / Toggle    GPIO = Toggles every time when CCRx hit      */
#define  TIMER_MODE_OC_RESET          5    /* eg. OUTMOD_5 / GO_INACTIVE GPIO = Reset on when CCRx hit, and stays on  */
#define  TIMER_MODE_PWM_INV           6    /* eg. OUTMOD_6 / PWM_2     GPIO = High -> Low  at CCR trip  */
#define  TIMER_MODE_PWM               7    /* eg. OUTMOD_7 / PWM_1     GPIO = Low  -> High at CCR trip  */

               // valid  values for action_flags in timer_Enable_CCR_Output()
#define  TIMER_ACTION_GO_ACTIVE    TIMER_MODE_OC_SET     /* GPIO pin goes active   when CCR value is reached */
#define  TIMER_ACTION_GO_INACTIVE  TIMER_MODE_OC_RESET   /* GPIO pin goes inactive when CCR value is reached */
#define  TIMER_ACTION_TOGGLE       TIMER_MODE_OC_TOGGLE  /* GPIO pin toggles state when CCR value is reached */
#define  TIMER_ACTION_PULSE        TIMER_MODE_PWM_INV    /* GPIO pin PWM pulse state when CCR value is reached */

#define  TIMER_PIN_POLARITY_HIGH     0x0000              /* "active" means GPIO pin goes high           */
#define  TIMER_PIN_POLARITY_LOW      0x0020              /* "active" means GPIO pin goes low (inverted) */

               // valid  values for rupt_flags in timer_Enable_CCR_Output()
#define  TIMER_ENABLE_CCR_INTERRUPTS 0x0080              /* raise an interrupt when CCR value is reached */

               // valid  values for rupt_flags in timer_Enable()
#define  TIMER_ENABLE_ROLLOVER_INTERRUPTS  0x0001

#define  TIMER_NO_INTERRUPT       0x0000         /* timer_enable()/enable_CCR() function does NOT want */
                                                 // any interrupts to be used

#define  TIMER_0                  0              // Typically TA0 on MSP430/432       Timer0/PWM0 on Tiva
#define  TIMER_1                  1              // Typically TA1 on MSP430/432       Timer1/PWM1 on Tiva
#define  TIMER_2                  2              // Typically TA2 on MSP430/432       Timer2/PWM2 on Tiva
#define  TIMER_3                  3              // Typically TA3 on MSP430/432       Timer3/PWM3 on Tiva
#define  TIMER_4                  4              // Typically TB0 on MSP430/432       Timer4/PWM4 on Tiva
#define  TIMER_5                  5              // Typically TB1 on MSP430/432       Timer5/PWM5 on Tiva
#define  TIMER_6                  6              // Typically 1st Timer32 on MSP432
#define  TIMER_7                  7              // Typically @nd Timer32 on MSP432
#define  TIMER_8                  8

               // allowed  values for channel numbers in timer_Enable_CCR_Input(), etc
// ??? !!! why the brain-damged difference in API calls:  vs pwm_Config_Channel()
// create a conveinece function:  timer_Config_Channel() that has a flag denoting NO_AUTO_ENABLE ???
               // Note: These channel numbers match the equivalent PWM_CHANNEL_x numbers
//#define  TMR_PWM_CHANNEL_0      0  // CCR0 on TI chips (MSP430/MSP432) denotes the PERIOD register
#define  TIMER_CHANNEL_0          0  // CCR0 on TI chips (MSP430/MSP432) denotes the PERIOD register
#define  TIMER_CHANNEL_1          1  // Up to 4 channels per Timer / PWM Module
#define  TIMER_CHANNEL_2          2  // except for MSP430/MSP432, which allow up to 6 channels
#define  TIMER_CHANNEL_3          3
#define  TIMER_CHANNEL_4          4
#define  TIMER_CHANNEL_5          5  // MSP432 / MSP430 only
#define  TIMER_CHANNEL_6          6  // MSP432 / MSP430 only

#define  TIMER_CHANNEL_1_ALT1     8  // Alternate Pinouts (Pin_Mux) for Channel 1 outputs
#define  TIMER_CHANNEL_2_ALT1     9  //   See associated API notes for each platform
#define  TIMER_CHANNEL_3_ALT1    10  //   that show what alternate pins these route to
#define  TIMER_CHANNEL_4_ALT1    11
#define  TIMER_CHANNEL_1_ALT2    12
#define  TIMER_CHANNEL_2_ALT2    13
#define  TIMER_CHANNEL_3_ALT2    14
#define  TIMER_CHANNEL_4_ALT2    15

#define  TIMER_CHANNEL_1_N       16  // Complementary, Inverted output pins (TIM1_CH1N, ...)
#define  TIMER_CHANNEL_2_N       17
#define  TIMER_CHANNEL_3_N       18
#define  TIMER_CHANNEL_1_N_ALT1  20  // Alternate Pinouts (Pin_Mux) for Complementary, Inverted output pins
#define  TIMER_CHANNEL_2_N_ALT1  21
#define  TIMER_CHANNEL_3_N_ALT1  22
#define  TIMER_CHANNEL_1_N_ALT2  24
#define  TIMER_CHANNEL_2_N_ALT2  25
#define  TIMER_CHANNEL_3_N_ALT2  26

               // interrupt flags passed back when User Timer Callback Handler is invoked.
               // Only one flag type will be passed back per callback invocation.
#define  TIMER_CCR1_INTERRUPT       1
#define  TIMER_CCR2_INTERRUPT       2
#define  TIMER_CCR3_INTERRUPT       3
#define  TIMER_CCR4_INTERRUPT       4
#define  TIMER_CCR5_INTERRUPT       5      // MSP432 / MSP430 only
#define  TIMER_CCR6_INTERRUPT       6      // MSP432 / MSP430 only
#define  TIMER_ROLLOVER_INTERRUPT   8


                         //-----------------
                         //-----------------
                         //   UART  APIs
                         //-----------------
                         //-----------------

                         //-------------------------
                         //-------------------------
                         //  UNIQUE-ID / CRC  APIs
                         //-------------------------
                         //-------------------------

                         //-------------------------------
                         //-------------------------------
                         //  VTIMER (Virtual Timer)  APIs
                         //-------------------------------
                         //-------------------------------
#define  vtimer_Start(vtimer_id,timer_duration_millis,callback_function,callback_parm)  board_vtimer_start (vtimer_id,timer_duration_millis,callback_function,callback_parm)
#define  vtimer_Check_Completed(vtimer_id)   board_vtimer_completed(vtimer_id)
#define  vtimer_Stop(vtimer_id)              board_vtimer_reset (vtimer_id)


                         //-------------------------------
                         //-------------------------------
                         //    Generic  Error  Codes
                         //-------------------------------
                         //-------------------------------
#define  ERR_ADC_MODULE_ID_OUT_OF_RANGE     -110   /* Valid range is 0 to 2 (ANY)  */
#define  ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE   -111
#define  ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE  -112   /* Valid range is 0 to 8 (ANY)  */
#define  ERR_ADC_STEP_NUM_OUT_OF_RANGE      -113   /* Valid range is 0 to 4 (ANY)  */
#define  ERR_ADC_MODULE_NOT_INITIALIZED     -114   /* need to call adc_init first */
#define  ERR_ADC_UNSUPPORTED_TRIGGER_TYPE   -115   /* trigger_type not supported/valid */

#define  ERR_I2C_MODULE_ID_OUT_OF_RANGE     -120   /* i2c_module is not within valid range 1..n */
#define  ERR_I2C_INVALID_I2C_MS_MODE        -121   /* i2c_ms_mode is not I2C_MASTER/I2C_SLAVE   */
#define  ERR_I2C_EXCEEDS_MAX_BAUD_RATE      -122   /* baud_rate exceeds max allowed by the chip */
#define  ERR_I2C_INVALID_REQUEST            -123   /* Operation is not supported in this mode   */

#define  ERR_PWM_MODULE_ID_OUT_OF_RANGE     -130
#define  ERR_PWM_MODULE_INITIALIZE_FAILED   -131
#define  ERR_PWM_MODULE_NOT_INITIALIZED     -132
#define  ERR_PWM_MODULE_NO_COMPLEMENTARY    -133   /* Complementary mode not supported by this module */
#define  ERR_PWM_MODULE_NO_DEADTIME         -134   /* Dead Time not supported by this module */
#define  ERR_PWM_MODULE_NO_PHASE            -135   /* Phase Shifting not supported by this module */
//#define  ERR_PWM_MODULE_IN_COMPLEMENTARY    -134 /* Module in complementary mode, single channel not supported*/
//#define  ERR_PWM_MODULE_NOT_IN_COMPLEMENTARY -135 /* Module not in complementary state, pair channel not supported*/
#define  ERR_PWM_COUNT_MODE_INVALID         -136   /* Invalid value specified for count_mode */
#define  ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE   -137   /* Channel number is invalid e.g. < 0 or > 25 */
#define  ERR_PWM_CHANNEL_NUM_NOT_SUPPORTED  -138   /* That channel number is not supported on this module */
#define  ERR_PWM_CHANNEL_CONFIG_FAILED      -139
#define  ERR_PWM_CHANNEL_COMPLE_CHAN_NO_CCR -140   /* issuing pwm_set_duty_cycle to one of the
                                                   ** "complementary channels" (5-8) is not allowed.
                                                   ** They are always driven off the duty cycle
                                                   ** from the "primary channel" (1-4) instead.   */
#define  ERR_PWM_CHANNEL_START_FAILED       -141
#define  ERR_PWM_INVALID_TIMER_MODE         -142   /* bad mode parameter value on board_timerrpwm_config_channel() */

#define  ERR_TIMER_MOD_OUT_OF_RANGE         -150   /* Timer Module Number is ouside the valid range of 0 to 22   */
#define  ERR_TIMER_MOD_NOT_SUPPORTED        -151   /* That Timer Module Number is not supported on this platform */
#define  ERR_TIMER_INVALID_COUNTER_TYPE     -152   /* counter_type in timer_Init() call is invalid  */
#define  ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE -153   // REDUNDANT WITH PWM
#define  ERR_TIMER_CHAN_NUM_NOT_SUPPORTED   -154   /* That Timer/PWM Module does not have that CCR on this platform */
#define  ERR_TIMER_CHANNEL_CONFIG_FAILED    -155   // REDUNDANT WITH PWM
#define  ERR_TIMER_CHANNEL_START_FAILED     -156   // REDUNDANT WITH PWM
#define  ERR_TIMER_ADC_NOT_INITIALIZED      -157   // A trigger_ADC_Start() call was issued, but the ADC
                                                   // has not been initialized yet.
#define  ERR_TIMER_DAC_NOT_INITIALIZED      -158   // A trigger_DAC_Start() call was issued, but the DAC
                                                   // has not been initialized yet.
#define  ERR_TIMER_TRIGGER_MISMATCH         -159   // The trigger type specified on the trigger_ADC/DAC_Start() call
                                                   // does not match what was issued on the init_ADC() or init_DAC()
#define  ERR_TIMER_INVALID_TRIGGER_TYPE     -160   // The trigger type specified on trigger_ADC/DAC_Start() call was invalid
#define  ERR_TIMER_MOD_BAD_TRIGGER_TYPE     -161   // The trigger type does not matches the Timer module used on

#define  ERR_VTIMER_ID_OUT_OF_RANGE         -170   /* VTIMER id ranges is 0 to 9. Is outside that range */
#define  ERR_VTIMER_IN_USE                  -171   /* requested VTIMER has already been started and is in use */
#define  ERR_VTIMER_MILLISEC_EXCEED_LIMIT   -172   /* max limit for timer_duration_millis is 1000000000 */




                  //-------------------------------------------------------
                  //-------------------------------------------------------
                  //-------------------------------------------------------
                  //  Pull in specific pin mapping for each Launchpad.
                  //
                  //  It provides a logical name for each pin (Pin2, Pin3,
                  //  Adc0, Adc1, ...) for each pin, to help provide cross
                  //  platform re-use of code.
                  //-------------------------------------------------------
                  //-------------------------------------------------------
                  //-------------------------------------------------------

#if defined(__MSP430F5529__) || defined(__MSP430FR6989__) || defined(__MSP430FR5969__) || defined(__MSP430FR4133__) || defined(__MSP430G2553__)

#include "msp430/user_api_MSP430_pin_map.h"   // Board specific defs

#endif                             // MSP430       (16 bit)



#if defined(__MSP432P401R__) || defined(TARGET_IS_MSP432P4XX)

#include "msp432/user_api_MSP432_pin_map.h"   // Board specific defs

#endif                             // MSP432       (ARM 32 bit)



#if defined(PART_TM4C123GH6PM)

#include "tiva/user_api_tiva_pin_map.h"       // Board specific defs

#endif                             // Tiva



#if defined(cc3200)

#include "cc3200/user_api_CC3200_pin_map.h"   // Board specific defs

#endif                             // CC3200



#if defined(C2000) || defined(F28027)

#include "c2000/user_api_C2000_pin_map.h"

#endif                             // C2000


#endif                             //  __USER_API_H__
