// 09/02/15 - L1 has a DAC IRQ problem
//    Somehow, DAC_CHAN_1_DMA_ISR_IRQHandler is causing the ADC_DMA callback handler to be called !!!
//           from the DAC_CHAN_1_DMA_ISR_IRQHandler.
//      ==> are they sharing the same rupt, and we must query which bit is set. ?
//      ==> is causing a severe nested, tight rupt loop. main uis never getting control back.
//      ==> only occurs when turn on DAC support. ADC runs fine otherwise.


// 09/01/15 - L4 Nucleo is pretty much a basket case:
//     - Blinking lights are running at 1/4 (or less) other platforms.
//           ==> System Clock logic aapears FUed
//     - Combination of lights and timers are stepping on each other
//           ==> D6 is blinking (should be switch driven)
//           ==> D7 is unlit
//     - DAC is not working. Outputs are frozen low.  Timer6/Timer7 are fine, even MMS.
//           ==> DAC init logic FUed, most probably related to DMA.
//               Also check if any SYSCFG CTLREG1 bits need to be tweaked.

// 07/28/15 - F4: COMPLETELY WORKS

// 07/28/15 - F0: COMPLETELY WORKS

// 07/27/15 - Hard Fault has been cured (issue warning and return) on Timers,

// 07/25/15 - SHOULD: change logic to have Slider Potentiometer control speed of
//            PWM flahsing of BLUE LEDs

// 07/21/15 - Add support for F0 Discovery with built in LEDs
//            WORKS, but: do_timerXR_callbacks only counts to 1 then stops
//                   All the other callbacks works
// Next step would be an Advanced Sensors for Discovery to show off
//      L3GD20 gyro (pull from their demo lib) and on TSC (touch slider)
//      i.e. usze built-in sensors. User slider to adjust PWM speed of LEDs
//      Could also run DAC, since the pin come out on the F0 Discovery and use
//      use Slide to adjust speed of sine wave.
//
// NOTE NEW pinConfigPinmux call - should that bne made generic ?
//          probably at least at board_xxx call level, to allow experienced
//          programmers to do their own thing ...

// 07/14/15 - STM32_F7:   - Timer XR and TimerOC callbacks working.
//                        - PWM8 is working.  PWM1 is not. ==> CHECK ERRATA
//                        - ADCs are dead - is the proper DMA channel being used and enabled ???  (a) check if timer 6 triggered, (b) check if ADC3 Enabled.  And do a standalone ADC APP.
//                          ADC WORKS IN MANUAL TRIGGER MODE !!!  Both DMA and ADC is working
//                             ==> something goofy with Timer4 <-> ADC logic
//                                 either try a different timer, or find an S7 example !
//                        - Input switches now working and assoc LED outputs showing state too.
//                        - Tight "Hot I/O" Timer loop has been eliminated.
//                        - XR and OC Timer callbacks are working.

/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                         main_simple_sensors.c
*
*
*  Hook up one or more simple sensors and actuators and verify they work
*  are working properly (and the underlying ADC, GPIO, PWM, ... support).
*
*  Input sensors include:
*     - Analog Potentiometer (Rotary and/or Slider)
*     - Analog Light Sensors
*     - Analog Temperature Sensors
*     - Simple Digital On/Off Switch
*     - Magnetic Reed On/Off Switch
-     - Proximity Sensor/Switch
*
*  Output actuators include:
*     - LEDs
*     - Buzzer
*     - Simple DC Motor  (?) fed PWM
*     - Simple Servo     (?) ditto
*     - . . .
*     - DAC ?
*
*  You can hook up all of the sensors, if you have them, or just a subset.
*  This lab is mainly designed to ensure you can properly hook up sensors to the
*  board and get them running. These sensors will be combined later with
*  communication code, allowing them being remotely reported, and remotely
*  controlled.
*
*  MCU Senors and Actuators used:
*  ------------------------------
*    Analog Inputs:
*         - Light Sensor
*         - Temperature Sensor
*         - Potentiometer         (to simulate any other type of sensor)
*    Digital Inputs:
*         - Magnetic Reed switch  (acts as a limit switch)
*         - Fixed switch
*    "Analog" Outputs
*         - PWM to xxx   BDC motor ???
*         - DAC (on those MCUs that have a native DAC
*    Digital Outputs
*         - LED
*         - Relay ?
*         - Motor Direction ?
*
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*  Grandview DB/DC Systems             DRDA Portable Toolkit
*    Licensed Property of Grandview DC/DC Systems
*    (c) Copyright  Grandview DB/DC Systems  1994, 2002
*    All rights reserved
*
*  History:
*    05/12/15 - Created to work with a group of sensors and allow
*               bi-directional communication between the MCU and a PLC.Duquaine
*    05/15/15 - Tiva 123G  - Works. Duqu
*    05/17/15 - MSP432 ARM - Works. Duqu
*    05/17/15 - MSP430-F5529  - Works. Duqu
*    05/18/15 - MSP430-FR5969 - Works. Duqu
*    05/18/15 - MSP430-FR4133 - Works standalone. Duqu
*    05/18/15 - MSP430-F2553  - Mostly Works standalone. Duqu  (J5 ADC input issue - flatlines at 433)
*    05/26/15 - MSP430-FR6989 - Mostly Works standalone. Duqu  (Not all PWMs working, 1/2 do 1/2 don't. J4-1 and J4-3 do NOT)
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

#define   STM32_F0_DISCOVERY    // Test in class

#include "user_api.h"           // pull in high level User API defs as well
                                //    as stdint.h / stdbool.h / DriverLib / HAL

#include "device_config_common.h"    // TEMP HACK


         // Timer_1 is being used by PWM support
void  timerXR_callback (void *callback_parm, int interrupt_flags);
void  timerOC_CCR_callback (void *callback_parm, int interrupt_flag);
void  adc_callback (void *callback_parm, uint16_t *adc_conv_results,
                    int num_active_channels, int flags);


#if defined(PART_TM4C123GH6PM)                         // Tiva 123G
inline void   inline_High (uint32_t gpio_port,  unsigned long pin)
{
    HWREG(gpio_port + GPIO_O_DATA + (pin << 2))  |=  pin;
}
#endif

#define  ADC_SAMPLES_PER_SECOND   60    // get 60 samples per second
#define  DAC_WAVE_FREQUENCY       60    // set DAC frequency to 60 cycles per second for Wave output


         char     using_adc_callbacks = 1;         // denote if we are using ADC callbacks
         char     enable_sensor_read_updates = 1;  // 1 = call sensor_read_updates()

         int      adc_timer_triggering_in_use = 1;  // 1 = use TIMER, 0 = USER_APP
         long     adc_trigger_period      = 60;     // ADC Timer's ARR (period) value
         long     adc_callback_count      = 0;
         uint16_t adc_callback_chan_results [16];
         uint32_t vtimr_adc_callback_done = 0;    // VTIMER DEBUG TEST
         uint32_t adc_trigger_starts      = 0;
         long     adc_User_Start_rejected = 0;
         int      adc_issue_trigger_flag  = 0;    // VTIMER Used to signal a new SW ADC conversion should be issued

         int      loop_on_pwms       = 0;   // PWM    DEBUG TEST

         char     test_wave     = 1;            // Type of wave to test: 1 - 4
         char     startup_dacs  = 3;            // 1 = chan 1, 2 = chan 2, 3 = both
         int      dac_frequency = DAC_WAVE_FREQUENCY;

         int      use_slow_pwm  = 1;    // 1 Run PWM slow for LED testing (10 SPS)

#define  PWM_FREQUENCY_FAST    20000    /* Fast PWM - 20 KHz (scope trace)   */
#define  PWM_FREQUENCY_SLOW       10    /* Slow PWM - 10 Hz  (see LED Blink) */


         long     timerXR_frequency      = 10;  // Timer 2/TB0 freq = 10 CPS (cycles/sec)
         long     timerXR_period         = 0;
         int      do_timerXR_callbacks   = 1;   // 0 = do polled check
         long     timerXR_callback_count = 0;

         int      timerOC_action_flags   = TIMER_ACTION_TOGGLE + TIMER_PIN_POLARITY_HIGH;
///      int      timerOC_action_flags   = TIMER_ACTION_GO_ACTIVE + TIMER_PIN_POLARITY_HIGH;
///      long     timerOC_frequency      = 12;     // Timer 1 freq = 12 CPS - pre-scaling issues
         long     timerOC_frequency      = 1000;   // Timer 1 freq = 1000 CPS (cycles/sec)
         long     timerOC_period         = 0;
         long     timerOC_duty           = 0;
         int      timerOC_rupt_flags     = 0;      // or TIMER_ENABLE_CCR_INTERRUPTS 2
         long     timerOC_callback_count = 0;

         int      timer_popped_flag     = 0;
volatile int      timer_popped_count    = 0;

         long     curr_cnt           = 0;         // ADC Timer's CNT value
         long     new_arr            = 0;         // New value to set ARR / period to ADC Timer

         long     nonBlockingValue  = 1;    // we want non-blocking turned on

      //------------------------------------------------------------------------
      //  DAC  Generation Tables
      //------------------------------------------------------------------------
#define  NUM_SAMPLE_STEPS   256

    short  dac_chan1_gen_table [NUM_SAMPLE_STEPS]  = { 0 };
    short  composite_wave_table [NUM_SAMPLE_STEPS] = { 0 };
    short  adc_capture_table [NUM_SAMPLE_STEPS]    = { 0 };

#define maxSamplesNum 120


      //------------------------------------------------------------------------
      //                      "Process Image"
      //
      // This keeps a copy of all the latest sensor/actuator values.
      //
      // It is the primary data area that is processed by MODBUS function
      // commands when sensor or actuuator data needs to be sent to or updated
      // from another MODBUS node (PLC).
      //------------------------------------------------------------------------
typedef struct mcu_processimage     /* MCU Process Image data block    */
    {
                      //--------------  INPUTS  --------------------------------
        bool      mag_reed_switch;  /* on/off status of Mag Reed Switch  4000 */
        bool      fixed_switch;     /* on/off status of Fixed Switch     4001 */
        uint16_t  light_sense_adc;  /* current value of light sensor     5000 */
        uint16_t  potentiometer_adc;/* current value of potentiometer    5002 */
        uint16_t  temp_sense_adc;   /* current value of temperature      5004 */

                      //--------------  OUTPUTS  -------------------------------
        bool      led_1;            /* on/off status of LED1             4002 */
        bool      direction;        /* on/off status of Direction bit    4002 */
        uint16_t  pwm_1_value;      /* current duty cycle for PWM_1      6000 */
        uint16_t  dac_1_value;      /* current output value for DAC_1    6002 */
    } MCU_PROCESS_IMAGE;



//------------------------------------------------------------------------------
// Key I/O Areas
//------------------------------------------------------------------------------
    int        ret_code = 0;
    uint32_t   io_Clock;
    uint32_t   pwm_Period;
    int        count_mode;

    uint16_t   adc_result_array [3];         // ADC I/O results


    MCU_PROCESS_IMAGE  mcu_proc_image = { 0 };


//--------------------------------------------------------------------
//                    Function prototypes
//--------------------------------------------------------------------
void  uart_get_config_info (void);    // FUNCTION PROTOYPES Forward refs
int   modbus_client_process (int netId,  int  clnt_sockId, MCU_PROCESS_IMAGE *procimg);
int   modbus_server_process (int netId, int clnt_sockId, MCU_PROCESS_IMAGE *procimg);
void  sensor_read_process (MCU_PROCESS_IMAGE *procimg);
void  sensor_write_process (MCU_PROCESS_IMAGE *procimg);
void  vtimer_adc_callback (void *callback_parm);
int   init_sensor_peripherals (void);


//******************************************************************************
//******************************************************************************
//                               main
//******************************************************************************
//******************************************************************************

int  main (int argc, char** argv)
{
    setup();                                     // Initialize everything

    CONSOLE_WRITE ("\n\rAll initialization complete. Starting main process loop.\n\r");

        //---------------------------------------------------------------------
        // Loop through all tasks to see if any new work needs to be processed
        //---------------------------------------------------------------------
    while (1)
      {
        sensor_read_process (&mcu_proc_image);   // Handle any sensor updates
      }

    CONSOLE_WRITE (" Terminating.\n\r");
    while (1)
      {          // just hang, so debugger can see we completed OK
      }
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>   BOILERPLATE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//******************************************************************************
//******************************************************************************
// setup
//
//           do initial configuration and setup of the MCU
//           and port number from UART.     (Optionally, also coil_on/off)
//******************************************************************************
//******************************************************************************

int  setup (void)
{   int   i;
    long  trig_period_value;

    sys_Init (0,0);              // start up system with default CPU clock rate

    board_disable_global_interrupts();  // disable global CPU interrupts until config complete

       //--------------------------------------------------------
       //               Configure Digital Inputs  -  Switches
       //                (Magetics Reed, Fixed Slide Switch)
       //--------------------------------------------------------
    pin_Config (MAG_READ_SW1, GPIO_INPUT, 0);     // Grove J15  J4-3   Arduino D2
    pin_Config (FIXED_SLIDE_SW1, GPIO_INPUT, 0);  // Grove J17  J4-5   Arduino D7

       //--------------------------------------------------------
       //               Configure Digital Outputs  -  LEDs
       //              (Will reflect state of above inputs)
       //--------------------------------------------------------
    pin_Config (MANUAL_RED_LED1, GPIO_OUTPUT, 0);  // Grove J15  J4-3   Arduino D4
    pin_Config (MANUAL_RED_LED2, GPIO_OUTPUT, 0);  // Grove J17  J4-5   Arduino D8
pin_High (MANUAL_RED_LED1);                        // initialize to on  - WORKS
pin_High (MANUAL_RED_LED2);
    pin_Low (MANUAL_RED_LED1);                     // initialize to off
    pin_Low (MANUAL_RED_LED2);

#if defined(STM32_F0_DISCOVERY)
             // ??? Not needed - GPIOs should already be setup by pwm_config_Channel, ... !!! ???
//   board_gpio_pin_config (DISCOVERY_PORT, DISCOVERY_LD3, GPIO_OUTPUT,
//                         GPIO_NOPULL, DISCOVERY_ALT_FUNC);
#endif

#if (USES_DAC)  // STM32 F4 01/11 do _NOT_ support native DAC. F4_46 and F0 _do_ support.
       //------------------------------------------------------------------
       //                        DAC Setup
       //------------------------------------------------------------------
    dac_Init (DACMD);             // Initialize the DAC module

                                  // Configure DAC Channels
    dac_Config_Channel (DACMD, DAC_CHAN_1, DAC_TRIGGER_TIMER_6, dac_frequency, 0);
    dac_Config_Channel (DACMD, DAC_CHAN_2, DAC_TRIGGER_TIMER_7, dac_frequency, 0);

       //-------------------------------------------------------------------
       // generate a wave function table, and then set it up for DAC channel
       //-------------------------------------------------------------------
    dac_Gen_Sample_Table (DAC_GEN_SINEWAVE, dac_chan1_gen_table, NUM_SAMPLE_STEPS);
    dac_Set_Sample_Table (DACMD, DAC_CHAN_1, dac_chan1_gen_table, NUM_SAMPLE_STEPS);
    dac_Gen_Sample_Table (DAC_GEN_SAWTOOTH, composite_wave_table, NUM_SAMPLE_STEPS);
    dac_Set_Sample_Table (DACMD, DAC_CHAN_2, composite_wave_table, NUM_SAMPLE_STEPS);

       //-------------------------------------------------------------
       // Configure and start Trigger Timers for DAC channels.
       // These must be started _after_ the DAC has been initialized.
       //-------------------------------------------------------------
    timer_DAC_Trigger_Start (TIMER_6, DAC_CHAN_1, DAC_TRIGGER_TIMER_6,
                 dac_frequency, NUM_SAMPLE_STEPS, TIMER_AUTOSTART); // DAC Ch 1  A2 (PA4)
    timer_DAC_Trigger_Start (TIMER_7, DAC_CHAN_2,  DAC_TRIGGER_TIMER_7,
                 dac_frequency, maxSamplesNum, TIMER_AUTOSTART);    // DAC Ch 2  D13 (PA5)

       //---------------------------------------------------------
       // Startup the DACs, and have them begin generating output
       //---------------------------------------------------------
if (startup_dacs == 1 || startup_dacs == 3)
    dac_Enable_Channel (DACMD, DAC_CHAN_1);   // startup the DAC channel = PA4 / A3
if (startup_dacs == 2 || startup_dacs == 3)
    dac_Enable_Channel (DACMD, DAC_CHAN_2);   // startup the DAC channel = PA5 / D13 SPI

#endif                                        // (USES_DAC)


       //-------------------------------------------------------------
       //                  Configure ADCs
       //
       // Note that we will be using Callbacks to handle ADC
       // conversion completes, rather than polling to check if done.
       //-------------------------------------------------------------
// if (ADC_TRIGGER_TYPE != ADC_TRIGGER_USER_APP)
//     adc_timer_triggering_in_use = 1;
    if (adc_timer_triggering_in_use)
        adc_Init (ADCMD, ADC_TRIGGER_TYPE, 0); // Initialialize ADC module and clocks.
                                  // Use a TIMER to auto-trigger ADC conversions
                                  // ADC_TRIGGER_TYPE is defined in project_config_parms.h
                                  // as well as its associated Timer ADC_TRIGGER_TIMER
        else {     // is going to be manually triggered by the User App
//            if (using_adc_callbacks)  ???
              adc_Init (ADCMD, ADC_TRIGGER_USER_APP, 0); // force it to USER SW STARTs
             }

#if defined(STM32_MCU)   // ST platforms use Grove J1 and J2 (and J3)
    adc_Config_Channel (ADCMD, A0_CHANNEL, 0);          // config light sensor
    adc_Config_Channel (ADCMD, A1_CHANNEL, 0);          // config potentionmeter
    adc_Config_Channel (ADCMD, A2_CHANNEL, ADC_LAST);   // config misc and
                                                // tag it as last one of the set
#else                    // TI platforms use Grove J5 and J6
    adc_Config_Channel (ADCMD, GROVE_J5, 0);        // config light sensor
    adc_Config_Channel (ADCMD, GROVE_J6, ADC_LAST); // config potentiometer
                                            // and tag it as last one of the set
#endif

         //-----------------------------------------------------------------
         // Configure TIMER_XR to use for checking out simple timer routines.
         // Have it use a callback to handle rollover interrupts.
         //        TIMER_XR  =  Timer_4/14 on ST,  and  TA3 on TI
         //-----------------------------------------------------------------
    timer_Set_Callback (TIMER_XR, timerXR_callback, &mcu_proc_image);   // TB0 on TI MSP430/MSP432
    timerXR_period = frequency_to_period_ticks (timerXR_frequency);
    timer_Init (TIMER_XR, TIMER_COUNT_UP, timerXR_period, 0);

// if put timer_Enable() right here for Timer_XR, it kills debugging !!!  Hmmm 07/15/15

       //--------------------------------------------------------
       //                  Setup PWM and OC Periods
       //--------------------------------------------------------
       // compute PWM period to use
    io_Clock =  board_sys_IO_clock_get_frequency();  // on MSP432, SMCLK = 1/2 MCLK
    if (use_slow_pwm)
       pwm_Period  = (io_Clock / PWM_FREQUENCY_SLOW) - 1;    // set period = 10  Hz
       else pwm_Period  = (io_Clock / PWM_FREQUENCY_FAST)-1; // set period = 20K Hz

         //-----------------------------------------------------------------
         // Configure TIMER_OC to use for checking out the OC timer routines
         //              TIMER_CHANNEL_1 = PB4 = Arduino D5      STM32
         //              TIMER_CHANNEL_2 = PA7 = Arduino D11     STM32
         // Have it use a callback for CCR interrupts.
         //        On STM32,  TIMER_OC = TIMER_3
         //-----------------------------------------------------------------
    timer_Set_Callback (TIMER_OC, timerOC_CCR_callback, &mcu_proc_image);

    timerOC_period = frequency_to_period_ticks (timerOC_frequency); // yields 4000000
/// timerOC_duty = (long) ((float) timerOC_period * 0.25);          // yields 1000000
    timerOC_duty = (long) (timerOC_period / 4);                     // yields

    timer_Init (TIMER_OC, TIMER_COUNT_UP, timerOC_period, 0);
    timer_Enable_CCR_Output (TIMER_OC, OC_CHANNEL_A, timerOC_duty,
                             timerOC_action_flags,timerOC_rupt_flags);// PB4 aka D5 runs 50 % duty at 500 Hz  CCR1 = 0x2EE0  CCMR1 Out = 0x07830  (Why is one setting = 7 (PWM) and the other 3 (OC) ???  ==> THAT IS THE PROBLEM.
    timer_Enable_CCR_Output (TIMER_OC, OC_CHANNEL_B, timerOC_duty,                                                   //    CCMR2 Out = 0x0000
                             TIMER_ACTION_PULSE + TIMER_PIN_POLARITY_HIGH,
                             TIMER_ENABLE_CCR_INTERRUPTS);            // PA7 aka D11  runs 75% duty at 1000 Hz CCR2 = 0x@EE0
///                          TIMER_ACTION_TOGGLE + TIMER_PIN_POLARITY_HIGH,
///                          TIMER_ACTION_PULSE + TIMER_PIN_POLARITY_HIGH,  // rises, then hangs there
///                          timerOC_rupt_flags);                     // PA7  aka D11


         //------------------------------------------------------------
         //              configure rest of sensors - PWMs, etc
         //------------------------------------------------------------
    init_sensor_peripherals();    // startup ADCs, GPIOs, and PWMs we are using

         //------------------------------------------------------------
         //              Enable ADC support
         //------------------------------------------------------------
    adc_Set_Callback (ADCMD, adc_callback, &mcu_proc_image);  // Setup callback
                                                // to read ADCs when DMA is done
    using_adc_callbacks = 1;     // ensure flag is sert to denote we are using ADC callback

    adc_Enable (ADCMD);          // Startup the ADCs, so they
                                 // are ready to sample when Timer trigger fires

       //-------------------------------------------------------------
       // Startup the ADC sampling, using auto-triggering via TIMER.
       //-------------------------------------------------------------
    if (adc_timer_triggering_in_use)
       timer_ADC_Trigger_Start (ADC_TRIGGER_TIMER, ADCMD,
                                ADC_TRIGGER_TYPE,
                                ADC_SAMPLES_PER_SECOND, TIMER_AUTOSTART);

    adc_trigger_period = timer_Get_Period (ADC_TRIGGER_TIMER); // get ADC Timer 's actual period value
    new_arr = adc_trigger_period;            //     and save it as initial value

       //-----------------------------------------------------------------------
       // Setup a VTIMER using a callback, to manually trigger the ADCs
       // every 1/2 second, IFF we have set the ADC flag to manual triggering
       //-----------------------------------------------------------------------
    ret_code =  board_vtimer_start (1, 500,           // wait for 1/2 second
                                    &vtimer_adc_callback,  &mcu_proc_image);

// 07/26/15 - WE are getting DMA/ADC interrupts (manual triggerd) here, before Timer14 IRQ is setup.
//            It appears to die after we setup the IRQ for Timer 14.
//            Is that IRQ Handler disabled/not present - causing the Hard_Fault ?

       //------------------------------------------------------------
       // Turn on ALL the Timers
       //------------------------------------------------------------
    ret_code = timer_Enable (TIMER_XR, TIMER_ENABLE_ROLLOVER_INTERRUPTS);
    ret_code = timer_Enable (TIMER_OC, 0);           // Rupts are on OCs/CCRs

// 07/26/15 - this is where the Hard_Fault bug is caused. If I blindly call
//            timer_Enable (ADC_TRIGGER_TIMER) when _manual_ triggering
//            is turned on, it causes a Hard Fault.
//               ==> Routine just skip the Timer_Enable logic and return an error
//  if (adc_timer_triggering_in_use)     // if this is used, everything works OK
       ret_code = timer_Enable (ADC_TRIGGER_TIMER, 0);  // No rupts, just ADC Trigger

    board_enable_global_interrupts();  // enable global CPU interrupts (or include in adc_enable logic !)

while (1)
 {       // standalone DAC/Timer/ADC testing
         // Check out API using some TIMER_XR facilities
    curr_cnt = timer_Get_Current_Value (TIMER_XR); // get current CNT value on timer
    if (do_timerXR_callbacks == 0)
       timer_popped_flag = timer_Check_Completed (TIMER_XR, 1, 1);  // poll for status
    if (timer_popped_flag)
       timer_popped_count++;

         // See if period was changed for ADC_TRIGGER_TIMER
    if (adc_trigger_period != new_arr)
       { timer_Set_Period (ADC_TRIGGER_TIMER, new_arr,0); // set new period value for ADC
         adc_trigger_period = timer_Get_Period (ADC_TRIGGER_TIMER); // save updated period value
       }

         // pull in any ADC and Digital I/O changes/updates
    if (enable_sensor_read_updates)
       sensor_read_process (&mcu_proc_image);  // ??? !!! This screws up ADC callbackk/interrupts !!! 07/11/15 MSP430
 }

    CONSOLE_WRITE ("\n\rMCU is initialized. Starting connection to Network/AP.\n\r");

    return (0);
}


//  >>>>>>>>>>>>>>>>>>>>>>>  APP CUSTOM  LOGIC  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//******************************************************************************
//******************************************************************************
// init_sensor_peripherals
//
//           Initialize the ADCs, GPIOs, and PWMs that will be used.
//******************************************************************************
//******************************************************************************

int  init_sensor_peripherals (void)
{


// CAUTION: Need to select PWMs and ADCs and GPIOs on connectors J3 and J4 -ONLY-
//          to avoid conflicts with CC3100 pins !!!

       //--------------------------------------------------------
       //                  Configure PWMs
       //--------------------------------------------------------
       // compute PWM period to use
    io_Clock =  board_sys_IO_clock_get_frequency();  // on MSP432, SMCLK = 1/2 MCLK
    if (use_slow_pwm)
       pwm_Period  = (io_Clock / PWM_FREQUENCY_SLOW) - 1;    // set period = 10  Hz
       else pwm_Period  = (io_Clock / PWM_FREQUENCY_FAST)-1; // set period = 20K Hz

#if defined(STM32F072xB) || defined(STM32F401xE) || defined(STM32F446xx) \
 || defined(STM32F746xx) || defined(STM32L053xx) || defined(STM32L152xE) \
 || defined(STM32L152xC) || defined(STM32L476xx)

 #if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F401xE) || defined(STM32F446xx) \
 || defined(STM32F746xx) || defined(STM32L053xx) || defined(STM32L152xE) \
 || defined(STM32L152xC) || defined(STM32L476xx)
       //-----------------------------------------------------
       // Configure PWM modules to be used   ==>> NUCLEO <<==
       //     On STM32:  TIMER_2  using  D3 / D6  (Blue LEDs)
       //-----------------------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, pwm_Period, 0);        // AIN1/AIN2 remap

       //-------------------------------------
       // Configure PWM channel(s) to be used    05/19/15 - Issues with channel masks at PWM_Enable because of multiple modules !
       //-------------------------------------                           Arduino
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_A,   // PA8  D7/D5 TIM1
                                    (pwm_Period/4), 0);  // 25 % duty cycle
   #if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_B,   // PA9  D8    TIM1
                                    (pwm_Period/3), 0);  // 33 % duty cycle
   #endif
 #else
       //------------------------------------------------------------
       // Configure PWM modules to be used   ==>> DISCOVERY <<==
       //     On STM32:  TIMER_3  using  LD5 (Green) and LD6 (Blue)
       //------------------------------------------------------------
         // All LEDs share same TIM3 timer, so no need to re-init PWM timer,
         // since timer_Init() was already called for the same TIM3
         // use 50 % duty cycle from PWM (PWM drives Channels 1 and 2)
         // Timer OC drives Channels 3 and 4 with timer interrupts and callbacks.
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_A,   // PA8  D7  TIM1
                                    (pwm_Period/4), 0);  // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_B,   // PA9  D8  TIM1
                                    (pwm_Period/3), 0);  // 33 % duty cycle
    ret_code = pwm_Enable (PWM_A_MODULE);       // enable PWM
    return (0);                                 // then return to caller
 #endif

       //------------------------------------------------------
       // Enable the PWM A module and its associated channels
       //     On STM32:  TIMER_2  using  D3 / D6  (Blue LEDs)
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);

       //-------------------------------------------------------------------
       // Configure and Enable the PWM B module and its associated channels
       //-------------------------------------------------------------------
 #if defined(PWM_B_MODULE)
    ret_code = pwm_Init (PWM_B_MODULE, pwm_Period, 0);        // BIN1/BIN2

    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_B_TEST_CHAN_A,  // PA2   D1 VCP  05/19/15 - fails
                                    (pwm_Period/4), 0);  // 25 % duty cycle
 #if defined(PWM_B_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_B_TEST_CHAN_B,  // PA3   D0 VCP  05/19/15 - fails
                                     (pwm_Period/3), 0);  // 33 % duty cycle
 #endif
    ret_code = pwm_Enable (PWM_B_MODULE);
 #endif

#endif



#if defined(__MSP432P401R__) || defined(TARGET_IS_MSP432P4XX)    // MSP432

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, pwm_Period, 0);   // TA1

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_A,
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_B,
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM modules and their associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);

 #if defined(PWM_B_MODULE)
    ret_code = pwm_Init (PWM_B_MODULE2, pwm_Period, 0);
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_B_TEST_CHAN_A,  //PWM_CHANNEL_3,
                                    (pwm_Period/4), 0);   // 25 % duty cycle
 #if defined(PWM_B_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_B_TEST_CHAN_B,  // PWM_CHANNEL_4,
                                    (pwm_Period/3), 0);   // 33 % duty cycle
 #endif
    ret_code = pwm_Enable (PWM_B_MODULE);
 #endif

#endif



#if defined(__MSP430F5529__)

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, pwm_Period, 0);   // TA0

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_A,
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_B,
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM modules and their associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);

 #if defined(PWM_B_MODULE)
    ret_code = pwm_Init (PWM_B_MODULE2, pwm_Period, 0);
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_3,
                                    (pwm_Period/4), 0);   // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_4,
                                    (pwm_Period/3), 0);   // 33 % duty cycle

    ret_code = pwm_Enable (PWM_B_MODULE);
 #endif

#endif



#if defined(__MSP430FR6989__)

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, pwm_Period, 0);   // TB0

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_A,
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_A_TEST_CHAN_B,
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM modules and their associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);

 #if defined(PWM_B_MODULE)
    ret_code = pwm_Init (PWM_B_MODULE, pwm_Period, 0);   // TA1

    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_1,  // P 3.3  J4-3 TA1   FAILS on MSP430_FR6989
                                     (pwm_Period/4), 0);   // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_2,  // P 1.3  J4-7       WORKS on MSP430_FR6989
                                    (pwm_Period/3), 0);   // 33 % duty cycle

    ret_code = pwm_Enable (PWM_B_MODULE);
 #endif

#endif



#if defined(__MSP430FR5969__)

       //-----------------------------------------
       // Jumper the following pins on Grove:
       //   ADC   J5   J3-3 -> J1-2
       //   ADC   J6   J3-4 -> J1-6
       //   GPIO J15   J4-3 -> J1-3  MAG SWITCH
       //   GPIO J17   J4-5 -> J1-5  FIXED SWITCH
       //-----------------------------------------

    count_mode = TIMER_PERIODIC_COUNT_UP;

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, wm_Period, 0);    // TB0

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_CHANNEL_3,  // P 3.4 J1-8  TB0
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_CHANNEL_4,  // P 3.5 J1-9
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM modules and their associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);

  #if defined(PWM_B_MODULE)
    ret_code = pwm_Init (PWM_B_MODULE, pwm_Period, 0);   // TA1

    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_1,  // P 3.3  J4-3 TA1   FAILS
                                     (pwm_Period/4), 0);   // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_B_MODULE, PWM_CHANNEL_2,  // P 1.3  J4-7       WORKS
                                    (pwm_Period/3), 0);   // 33 % duty cycle

    ret_code = pwm_Enable (PWM_B_MODULE);
 #endif

#endif



#if defined(__MSP430FR4133__)

       //-----------------------------------------
       // Jumper the following pins on Grove:
       //   ADC   J5   J3-3 -> J1-2
       //   ADC   J6   J3-4 -> J1-6
       //   GPIO J15   J4-3 -> J1-3  MAG SWITCH
       //   GPIO J17   J4-5 -> J1-5  FIXED SWITCH
       //-----------------------------------------

    count_mode = TIMER_PERIODIC_COUNT_UP;

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_MODULE_0, pwm_Period, 0);   // TA0

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_MODULE_0, PWM_CHANNEL_1,  // P 1.7  J2-2 TA0
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_MODULE_0, PWM_CHANNEL_2,  // P 1.6  J2-3
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM A module and its associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_MODULE_0);   // PWM_A_MODULE
#endif



#if defined(__MSP430G2553__)

       //-----------------------------------------
       // Jumper the following pins on Grove:
       //   ADC   J5   J3-3 -> J1-4   <===
       //   ADC   J6   J3-4 -> J1-6
       //   GPIO J15   J4-3 -> J1-3  MAG SWITCH
       //   GPIO J17   J4-5 -> J1-5  FIXED SWITCH
       //-----------------------------------------

    count_mode = TIMER_PERIODIC_COUNT_UP;

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_A_MODULE, pwm_Period, 0);   // TA1

       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_CHANNEL_1,  // P 2.1  J1-9
                                    (pwm_Period/4), 0);   // 25 % duty cycle
#if defined(PWM_A_TEST_CHAN_B)
    ret_code = pwm_Config_Channel (PWM_A_MODULE, PWM_CHANNEL_2,  // P 2.4  J2-9
                                    (pwm_Period/3), 0);   // 33 % duty cycle
#endif

       //------------------------------------------------------
       // Enable the PWM modules and their associated channels
       //------------------------------------------------------
    ret_code = pwm_Enable (PWM_A_MODULE);
#endif


#if defined(PART_TM4C123GH6PM)                         // Tiva 123G

    count_mode = TIMER_PERIODIC_COUNT_DOWN;

       //------------------------------------
       // Configure PWM modules to be used
       //------------------------------------
    ret_code = pwm_Init (PWM_MODULE_3, pwm_Period, 0);        // AIN1/AIN2 remap
    ret_code = pwm_Init (PWM_MODULE_6, pwm_Period, 0);        // BIN1/BIN2
       //-------------------------------------
       // Configure PWM channel(s) to be used
       //-------------------------------------
    ret_code = pwm_Config_Channel (PWM_MODULE_3, PWM_CHANNEL_5,  // PE4 J1-5 <--- ONLY THIS ONE IS WORKING ! CMPA = 0x01554
                                    (pwm_Period/4), 0);   // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_MODULE_3, PWM_CHANNEL_6,  // PE5 J1-6 <--- SOLID HIGH CMPB = huge value (0x0063B5  Period = 0x4E23)
                                    (pwm_Period/3), 0);   // 33 % duty cycle       When force CMPB to 0x15B5 it cranks up and PWMs !
    ret_code = pwm_Config_Channel (PWM_MODULE_6, PWM_CHANNEL_3,  // PA6 J1-9 <--- FLOATING   CMPA = 0x01554
                                    (pwm_Period/4), 0);   // 25 % duty cycle
    ret_code = pwm_Config_Channel (PWM_MODULE_6, PWM_CHANNEL_4,  // PA7 J1-10 <--- FLOATING  CMPB = huge value (0x0063B5  Period = 0x4E23)
                                    (pwm_Period/3), 0);   // 33 % duty cycle

       //---------------------------------------------------
       // Enable the PWM modules and its associated channels
       //---------------------------------------------------
    ret_code = pwm_Enable (PWM_MODULE_3);
    ret_code = pwm_Enable (PWM_MODULE_6);
#endif

}


//******************************************************************************
//******************************************************************************
// sensor_read_process
//
//          Reads in latest sensor inputs, and save them in the "Process Image"
//          structure.
//******************************************************************************
//******************************************************************************
void  sensor_read_process (MCU_PROCESS_IMAGE *procimg)
{

// checkout PWMs, ADCs, and Switches, and LEDs/Direction operation

       //---------------------------------------------------------
       // Should we trigger the ADCs on this process loop pass ?
       // Only do this if USER_APP triggering, otherwise it
       // will totally screw up the ADC if we have TIMER triggers
       // and then hit it with User Starts !
       //---------------------------------------------------------
    if (adc_issue_trigger_flag == 1 && adc_timer_triggering_in_use == 0)
       {     // yes, manually start next input conversion on the sensors
         while ( ! adc_Check_All_Completed(0))
           ;                           // wait till any previous conversion done
         ret_code = adc_User_Trigger_Start (0);
         if (ret_code < 0)
            adc_User_Start_rejected++;
adc_trigger_starts++;
         adc_issue_trigger_flag = 0;     // reset the trigger falg
       }

       //----------------------------------------------------------------------
       // Check if new ADC results are available.
       // If so, read them in and update associated fields in "Process Image"
       //----------------------------------------------------------------------
    if (using_adc_callbacks)
       {      // pull the information from the callback buffer.
              // update associated fields in "Process Image"
         mcu_proc_image.light_sense_adc   = adc_callback_chan_results[0];
         mcu_proc_image.potentiometer_adc = adc_callback_chan_results[1];
         mcu_proc_image.temp_sense_adc    = 0;  // adc_callback_chan_results[2];
       }
      else
       {      // polling for results.  ==> NOTE: THIS ALWAYS RETURNS FALSE IF USING CALLBACKS !!!
         if (adc_Check_All_Completed(0))
            {                           // wait
                  // retrieve the results of the ADC conversions.
                  // They will be returned in the result_array[], and will be
                  // returned in the order that they were configured, i.e.
                  //    result_array[0] = light sensor   value
                  //    result_array[1] = potentionmeter value
               adc_Read (0, adc_result_array);     // read in converted results

                  // update associated fields in "Process Image"
               mcu_proc_image.light_sense_adc   = adc_result_array[0];
               mcu_proc_image.potentiometer_adc = adc_result_array[1];
               mcu_proc_image.temp_sense_adc    = 0;    // adc_result_array[2];
            }
       }
// need to scale these in future: light to lumens, temp to Celcius or Farenh or Kelvin, potentiometer to volts

             //------------------------------------------------------------
             // Update Process Image using latest set of GPIO Input results
             //------------------------------------------------------------
         mcu_proc_image.fixed_switch    = pin_Read (FIXED_SLIDE_SW1);
         mcu_proc_image.mag_reed_switch = pin_Read (MAG_READ_SW1);

             //------------------------------------------------------------
             // Set RED LEDs to match DIGITAL INPUT switches manual state
             //------------------------------------------------------------
         if (mcu_proc_image.fixed_switch)
            { pin_High (MANUAL_RED_LED1); }
            else { pin_Low (MANUAL_RED_LED1); }     // COMPILER WACKS OUT WHEN remove the { }   !!! WVD FIX !!!???
         if (mcu_proc_image.mag_reed_switch)
            { pin_High (MANUAL_RED_LED2); }
            else { pin_Low (MANUAL_RED_LED2); }


// in future rev, use Potentiometer input to control PWM duty cycle

}


//******************************************************************************
// timerXR_callback                                  XR = Interrupt at Rollover
//
//          Invoked when Timer_2 (ST) or TA3 (TI) rolls over to new start count
//          The interrupt flags identify which CCR posted the interrupt
//
//          The interrupt flags parm denote which Update and/or CCR interrupt(s)
//          are being reported.
//******************************************************************************
void  timerXR_callback (void *callback_parm, int interrupt_flags)
{
    timerXR_callback_count++;                    // signal callback was performed
}


//******************************************************************************
// timerOC_CCR_callback                              OC TIMER CCR value reached
//
//          Invoked when one or more of Timer_OC's CCRs have an interrupt.
//          Timer_OC = Timer_3 on STM32,  and TA0 on TI
//
//          The interrupt flags parm denotes which CCR interrupt(s)
//          are being reported.
//******************************************************************************
void  timerOC_CCR_callback (void *callback_parm, int interrupt_flags)
{
    timerOC_callback_count++;                    // signal callback was performed
}


//******************************************************************************
// vtimer_adc_callback               VTIMER
//
//           Vtimer for ADC triggering
//******************************************************************************
void  vtimer_adc_callback (void *callback_parm)
{
    MCU_PROCESS_IMAGE  *pip;

//  pip = (MCU_PROCESS_IMAGE*) callback_parm;
//  pip->dac_1_value = 999;                     // show that we got called back

    vtimr_adc_callback_done++;                  // signal callback was performed

    adc_issue_trigger_flag = 1;       // flag to trigger ADC on next process loop pass
}


//******************************************************************************
// adc_callback                      DMA ADC Conversion Complete
//
//           Invoked when an ADC sequence of conversions has completed
//******************************************************************************
void  adc_callback (void *callback_parm, uint16_t *adc_conv_results,
                    int num_active_channels, int flags)
{
    int   i;

    for (i = 0;  i < num_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         adc_callback_chan_results[i] = adc_conv_results[i];
       }

    adc_issue_trigger_flag = 1;       // flag to trigger ADC on next process loop pass

    adc_callback_count++;             // signal callback was performed
}
