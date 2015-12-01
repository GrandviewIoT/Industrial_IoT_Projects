/********1*********2*********3*********4*********5*********6*********7**********
*
*    board.c - tiva-c launchpad configuration   for   Tiva 1294XL
*
*
* History:
*   12/12/14 - Significantly revised for IoT/PLC project. Duquaine
*   04/10/15 - Added a simple string edit for board_uart_read_string() support.
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

#include "boarddef.h"
#include "user_api.h"

#include "tiva/DRV8711_Spin_Routines.h"   // need for BASE_PWM_FREQUENCY def
static  void  board_vtimer_check_expiration (uint32_t curr_value);

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------
    P_EVENT_HANDLER   pIrqEventHandler = 0;
    uint8_t           IntIsMasked;


    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    float     _g_fusec_to_ticks;    // Float version of how many ticks in 1 usec

    uint32_t  _g_systick_millisecs = 0; // Global to how many 1 ms ticks have
                                        // accumulated startup (poor mans TOD)

    char      _g_vtimers_active = 0;    // Optional VTIMER support

    char      _g_pwm_mclock_configured = 0;      // PWM master clock was configured
    uint32_t  _g_pwm_mdivider = 1;  // PWM Master clock : CPU divider ratio

    uint32_t  _g_PWM_IO_clock = 0;  // Global to hold PWM Max Clock Frequency
    uint32_t  _g_PWMperiod    = 0;  // Global to hold PWM period  (MaxClock/Freq)
    uint32_t  _g_PWMduty      = 0;  // Global to hold latest PWM duty cycle
    uint32_t  _g_PWM_RuptFlags = 0;

                 // mainly for DRV8711 logic
    uint32_t  _g_ADC_value = 0;

#if defined(USES_CONSOLE_READ)
    char      _g_uart_buf_idx   = 0;   // keeps track of user input via UART
    char      _g_uart_last_char = 0;
#endif

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    unsigned long   _g_crcid_Result;
#endif

               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
const uint32_t  _g_gpio_base[] = 
    {
        0L,
        (uint32_t) GPIO_PORTA_BASE,
        (uint32_t) GPIO_PORTB_BASE,
        (uint32_t) GPIO_PORTC_BASE,
        (uint32_t) GPIO_PORTD_BASE,
        (uint32_t) GPIO_PORTE_BASE,
        (uint32_t) GPIO_PORTF_BASE
    };
#define  GP_PORT_A   1        // indexes into above table, based on port id
#define  GP_PORT_B   2
#define  GP_PORT_C   3
#define  GP_PORT_D   4
#define  GP_PORT_E   5
#define  GP_PORT_F   6



//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   Routines
//
//**************************************************************************
//**************************************************************************

//*****************************************************************************
//  board_init
//
//           Initialize system clocks, and basic GPIOs
//*****************************************************************************

void  board_init (long mcu_clock_rate)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (80000000);  // use default: run at 80 MHz

    board_gpio_init();              // turn on key GPIO clocks, ...

#if defined(USES_SYSTICK) || defined(USES_MQTT) || defined(USES_ADC) || defined(USES_VTIMER)
    board_systick_timer_config();   // ensure "Systick" timer is turned on
#endif

        // Enable the UART if user wants to invoke CONSOLE or DEBUG_LOG calls
#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    board_uart_init();              // go ahead and enable the default UART
#endif

}


//*****************************************************************************
//  board_busy_wait_usec
//
//           Continuously loop until a fixed time delay has completed.
//           This function provides a means of generating a constant
//           length. The function delay (in cycles) = 3 * parameter, so
//           divide the SystemClock to yield the approperiate usec value
//           1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
//*****************************************************************************

void  board_busy_wait_usec (long usec_delay)
{
   long  total_ticks;

   total_ticks = (long) (_g_fusec_to_ticks * (float) usec_delay);

   MAP_SysCtlDelay (total_ticks);
}


//*****************************************************************************
//  board_delay_ms
//
//           Continuously loop until a fixed time delay has completed.
//           This function provides a means of generating a constant
//           length. The function delay (in cycles) = 3 * parameter, so
//           divide the SystemClock to yield the approperiate usec value.
//             1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
//             1 usec Delay = (80 MHz / 1 million) / 3 = 26.6667
//*****************************************************************************

void  board_delay_ms (long msec_delay)
{
   long  total_ticks;

   total_ticks = (long) (_g_fusec_to_ticks * (float) msec_delay);

   MAP_SysCtlDelay (total_ticks * 1000);
}


//*****************************************************************************
// Delay
//
//          Produce a delay in milli-seconds (ms)      Used by CC3xxx Simplelink
//*****************************************************************************
void  Delay (unsigned long interval)
{
    board_delay_ms ((long) interval);
}


//*****************************************************************************
//  board_disable_global_interrupts
//
//         Turn off Global interrupts  (for Interval Timer, ...)
//*****************************************************************************
void  board_disable_global_interrupts (void)
{
    IntMasterDisable();
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for Interval Timer,
//                                     PWM Timer, ADC, ...)
//*****************************************************************************
void  board_enable_global_interrupts (void)
{
    IntMasterEnable();
}


//*****************************************************************************
//  board_error_handler
//
//      Catastrophic error occurred. Stop the train.
//*****************************************************************************

void  board_error_handler (void)
{
           /* Turn LED on */
    // BSP_LED_On (LED2);
    while (1)
      {                 // loop in here so Debugger can find it
      }
}


//*****************************************************************************
//  board_frequency_to_period_ticks
//
//      Takes a frequency (e.g. for PWM or Timer), and converts it to the 
//      equivalent clock ticks period value to be loaded into PWM registers.
//      If the result value would be too large, PWM/Timer code would have
//      to further sub-divide it using pre-scalars
//*****************************************************************************

long  board_frequency_to_period_ticks (long frequency)
{
    long   pticks;

        // take the current clock frequency, and compute the associated ticks
        // needed to generate that frequency.
     pticks = _g_SysClk_Ticks / frequency;

     return (pticks);
}


#if defined(USES_ADC)

//*****************************************************************************
//*****************************************************************************
//                               ADC   Routines
//*****************************************************************************
//*****************************************************************************

    char           _g_adc_clocks_on   = 0;
    unsigned char  _g_active_channels = 0;
    unsigned char  _g_adc_sequencer_map [12] = { 0 }; // indexed by channel num
    int            _g_sequencer       = 0;
    int            _g_step_num        = 0;

    int            _g_trigger_type    = 0;  // used to configure sequencers
    int            _g_trigger_flags   = 0;



typedef struct adc_module_def          /* ADC Module definitions */
    {
        uint32_t  adc_base;            /* ADC base address */
    } ADC_MODULE_BLK;

typedef struct adc_channel_def         /* ADC Channel definitions */
    {
        uint32_t  chan_gpio_port;      /* Associated GPIO port            */
        uint32_t  chan_gpio_pin;       /* Associated GPIO pin             */
        uint32_t  chan_adc_num;        /* Associated ADC AINx channel num */
    } ADC_CHANNEL_BLK;


const ADC_MODULE_BLK  _g_adc_modules [] =
        {  { ADC0_BASE },    // ADC_MODULE_0
           { ADC1_BASE },    // ADC_MODULE_1
           { ADC0_BASE }     // ADC_MODULE_ANY - defaults to ADC0
        };

const ADC_CHANNEL_BLK  _g_adc_channels [] =  //                  LP    Grove
        {  { GPIO_PORTE_BASE, GPIO_PIN_3, ADC_CTL_CH0 },  // PE_3  AIN0  CH0  J3-9
           { GPIO_PORTE_BASE, GPIO_PIN_2, ADC_CTL_CH1 },  // PE_2  AIN1  CH1  J3-8
           { GPIO_PORTE_BASE, GPIO_PIN_1, ADC_CTL_CH2 },  // PE_1  AIN2  CH2  J3-7    J9
           { GPIO_PORTE_BASE, GPIO_PIN_0, ADC_CTL_CH3 },  // PE_0  AIN3  CH3  J4-3
           { GPIO_PORTD_BASE, GPIO_PIN_3, ADC_CTL_CH4 },  // PD_3  AIN4  CH4  J3-6    J8
           { GPIO_PORTD_BASE, GPIO_PIN_2, ADC_CTL_CH5 },  // PD_2  AIN5  CH5  J3-5    J7
           { GPIO_PORTD_BASE, GPIO_PIN_1, ADC_CTL_CH6 },  // PD_1  AIN6  CH6  J3-4    J6
           { GPIO_PORTD_BASE, GPIO_PIN_0, ADC_CTL_CH7 },  // PD_0  AIN7  CH7  J3-3    J5
           { GPIO_PORTE_BASE, GPIO_PIN_5, ADC_CTL_CH8 },  // PE_5  AIN8  CH8  J1-6
           { GPIO_PORTE_BASE, GPIO_PIN_4, ADC_CTL_CH9 },  // PE_4  AIN9  CH9  J1-5
           { GPIO_PORTB_BASE, GPIO_PIN_4, ADC_CTL_CH10},  // PB_4  AIN10 CH10 J1-7
           { GPIO_PORTB_BASE, GPIO_PIN_5, ADC_CTL_CH11}   // PB_5  AIN11 CH11 J1-2
        };

const uint32_t  _g_adc_trigger_src [] =
        { ADC_TRIGGER_PROCESSOR,     // TRIGGER_USER_APP
          ADC_TRIGGER_PWM_MOD0,      // TRIGGER_PWM   - modified by trigger flags
          ADC_TRIGGER_TIMER,         // TRIGGER_TIMER - modified by trigger flags
          ADC_TRIGGER_COMP0,         // TRIGGER_COMPARATOR - modified by trigger flags
          ADC_TRIGGER_PWM0,          // TRIGGER_EVENT  - modified by trigger flags
          TRIGGER_VTIMER             // TRIGGER_VTIMER - modified by trigger flags
        };


//*****************************************************************************
//  board_adc_init
//
//         Initialize an ADC module, and Configure
//         the overall sampling clock used for the ADCs on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the specific PWM, TIMER, COMPARATOR, ... used
//           - User App triggered
//           - PWM Module 0  Generator 0 / 1 / 2 / 3
//           - PWM Module 1  Generator 0 / 1 / 2 / 3
//           - Timer n
//           - Comparator n
//*****************************************************************************

int  board_adc_init (int adc_module_id, uint32_t clock_rate, int trigger_type, 
                     int flags)
{
    uint32_t    adc_module;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

            // turn on the clocks for the ADC modules
//SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC0); // turn on ADC mod 0
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC1); // turn on ADC mod 1
    _g_adc_clocks_on = 1;                     // denote clocks are now on

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

// 05r/15/15 - Technically the following is optional !!!

//  MAP_ADCClockConfigSet (adc_module, uint32_t ui32Config,
//                         uint32_t ui32ClockDiv);

       // Configure the ADC to use PLL at 480 MHz divided by 24 to get an ADC
       // clock of 20 MHz.
       //
//  MAP_ADCClockConfigSet (adc_module, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24);

       // Configure the ADC to use PIOSC divided by one (16 MHz) and sample at
       // half the rate.
// ADCClockConfigSet (adc_module, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);   // ?? CHANGE IN FUTURE ???

    if (trigger_type < 0  ||  trigger_type > yy)
       return (ERR_ADC_UNSUPPORTED_TRIGGER_TYPE);

    _g_trigger_type  = trigger_type;   // save for when configure sequencers
    _g_trigger_flags = flags;

    return (0);                        // denote success

}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate will be determined by the trigger source.
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//
//         Tiva Sequencer Notes:
//                Sequencer 0      8 steps total
//                Sequencer 1      4 steps total
//                Sequencer 2      4 steps total
//                Sequencer 3      1 step  total
//
//         Tiva 123G supports a max of 12 ADC channels and 2 ADC modules (0/1)
//*****************************************************************************

int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer, int step_num,
                               int last,  int flags)
{
    ADC_CHANNEL_BLK  *adcblkp;
    uint32_t         adc_module;
    uint32_t         trigger_source;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (channel_num < 0  ||  channel_num > 11)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

          //-------------------------------------------------
          //      set the GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of 12 entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[] 
          //-------------------------------------------------
    adcblkp = &_g_adc_channels [channel_num];
    GPIOPinTypeADC (adcblkp->chan_gpio_port, adcblkp->chan_gpio_pin);
//GPIOPinTypeADC (GPIO_PORTD_BASE, GPIO_PIN_1);
//GPIOPinTypeADC (GPIO_PORTD_BASE, GPIO_PIN_0);


          //----------------------------------------------------------------
          // Handle "automatic" sequencing support.
          // This feature is for newbies or others that do not want to
          // figure out and keep track of the auto-sequencer details.
          // We start with Sequencer 0. If user configures more than
          // 8 channels, then we automatically move to Sequencer 1.
          // We also automatically jupdate the step number as well.
          //
          // Experienced users can explicity set sequencer and step_num
          // to any valid value (0-3, 0-7 respectively) if they want to
          // programmatically control the sequencing of ADCs by themselves.
          //----------------------------------------------------------------
    if (sequencer == ADC_AUTO_SEQUENCE)
       {
         if (_g_active_channels < 8)
            sequencer = 0;   // use sequencer 0, which handles up to 8 channels
            else {           // more than 8 channels in use - move to sequencer 1
                   sequencer = 1;
                   if (_g_active_channels == 8)
                      {    // we are starting channel 8, which will be the first
                           // entry for sequencer 1.
                           // So, reset step number to 0 for sequencer 1
                        if (step_num == ADC_ANY_STEP)
                           _g_step_num = 0;
                      }
                 }
       }

    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

          //--------------------------------------------------------------------
          // setup the sequencer
          //
          //  Note: trigerring is based on SEQUENCER - i.e. all channels (steps)
          //        assignhed to that sequencer use the same trigerring source.
          //--------------------------------------------------------------------
          // The sequencer must only be setup on the first channel of each
          // set of step (channel_numbers 0 and 8 in our case)
    if (step_num == 0)
       { trigger_source = _g_adc_trigger_src [_g_trigger_type];
// ??? !!! TRIGGER SOURCE NEEDS MORE WORK FOR TIMER AND PWM TRIGGERING !!!
         ADCSequenceConfigure (adc_module, 0, trigger_source, flags);  // sequencer 0
       }
      else if (step_num == 8)
              { trigger_source =  _g_adc_trigger_src [_g_trigger_type];
                ADCSequenceConfigure (adc_module, 1, trigger_source, flags);  // sequencer 1
              }
//ADCSequenceConfigure (ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

          //----------------------------------------------------------------
          // setup the step within the sequencer to use
          //----------------------------------------------------------------
    if (last)
       {     // this is the last channel of the sequence
         ADCSequenceStepConfigure (adc_module, sequencer, _g_step_num,
                                   adcblkp->chan_adc_num | ADC_CTL_IE | ADC_CTL_END);
       }
      else ADCSequenceStepConfigure (adc_module, sequencer, _g_step_num,
                                     adcblkp->chan_adc_num);
//ADCSequenceStepConfigure (ADC0_BASE, 0, 0, ADC_CTL_CH7);
//ADCSequenceStepConfigure (ADC0_BASE, 0, 1, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);

          // save the sequencer id that was used for this channel
    _g_adc_sequencer_map [channel_num] = (char) sequencer;

          // track how many channels have been configured for use
    _g_active_channels++;

    _g_step_num++;          // inc to next step slot in the sequencer

    return (0);             // denote success
}


//*****************************************************************************
//  board_adc_check_conversions_done
//
//          Checks if the ALL the conversions for a sinple ADC are done.
//
//          For groups, the starting channel of the group, as specified as
//          the first channel listed in the board_adc_group_init() channels[]
//          array, is passed.
//*****************************************************************************
int  board_adc_check_conversions_done (int adc_module_id, int sequencer)
{
    uint32_t  adc_module;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
       return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

    if (sequencer == ADC_AUTO_SEQUENCE)
       sequencer = _g_sequencer;  // use current auto-managed sequencer

//if ( ! ADCIntStatus(ADC0_BASE, 0, false))
    if ( ! ADCIntStatus (adc_module, sequencer, false))
       return (0);                                     // is still busy

    if (_g_active_channels > 8  &&  sequencer == ADC_AUTO_SEQUENCE)
       {    // need to also check the other sequencer, since > 8 channels means 2 sequencers
         if ( ! ADCIntStatus (adc_module, 0, false))
            return (0);                                // is still busy
       }

    return (1);                                        // is completed
}


//*****************************************************************************
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    uint32_t   adc_module;
    int        orig_sequencer;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
       return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

    if ( !  _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

    orig_sequencer = sequencer;    // save value upon entry

    if (sequencer == ADC_AUTO_SEQUENCE)
       sequencer = _g_sequencer;  // use current auto-managed sequencer

    MAP_ADCSequenceEnable (adc_module, sequencer);  // sequencer 0 or 1
    ADCIntClear (adc_module, sequencer);

    if (_g_active_channels > 8  &&  sequencer == ADC_AUTO_SEQUENCE)
       {    // need to also enable the other sequencer, since > 8 channels means 2 sequencers
         MAP_ADCSequenceEnable (adc_module, 0);  // sequencer # 0
         MAP_ADCIntClear (adc_module, 0);
       }
//ADCSequenceEnable (ADC0_BASE, 0);
//ADCIntClear (ADC0_BASE, 0);

// ??? Add initial trigger if USER_TRIGGERED to match MSP430/432 stuff  04/29/15 ??? !!!
    board_adc_user_trigger_start (adc_module_id, orig_sequencer);

    return (0);                                    // denote success
}


//*****************************************************************************
//  board_adc_disable
//
//          Turn off one or all the sequencers that have been configured.
//
//          Reset everything in preparattioon for a re-configuratioon of
//          channels.
//*****************************************************************************
int  board_adc_disable (int adc_module_id, int sequencer)
{
    uint32_t   adc_module;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
       return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

    if (sequencer == ADC_AUTO_SEQUENCE)
       sequencer = _g_sequencer;  // use current auto-managed sequencer

    MAP_ADCSequenceDisable (adc_module, sequencer);     // sequencer 0 or 1

    if (_g_active_channels > 8  &&  sequencer == ADC_AUTO_SEQUENCE)
       {    // need to also disable the other sequencer, since > 8 channels means 2 sequencers
         MAP_ADCSequenceDisable (adc_module, sequencer); // sequencer # 0
       }

    return (0);                                     // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer, int *channel_results)
{
    uint32_t   adc_module;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
       return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

    if (sequencer == ADC_AUTO_SEQUENCE)
       sequencer = _g_sequencer;      // use current auto-managed sequencer

// CAUTION: TI's code will automatically read an entire ARRAY (from FIFO)
//          if a sequence group is involved

//  rc = ADCSequenceDataGet (uint32_t ui32Base, uint32_t ui32SequenceNum,
//                           uint32_t *pui32Buffer)

// ??? !!!   HMMMM.  This is a problem if multiple ADC modules involved
       // Pass a sequencer number (including ADC_ANY_SEQUENCER) and walk thru
       // a table of ADC modules ?

       // possibly put a MAX_LENGTH / TOTAL_CHANNELs value on the call
       // to avoid any accidental ovverrun ?

// BEST OPTION !!! (below)
       // Or just always read all values into our internal ADC result values
       // and just copy them back to caller
       //    ==> interrupt based and any sequencer/adc_module stuff
       //        extracted and taken care of in the ISR

    if (_g_active_channels > 8  &&  sequencer == ADC_AUTO_SEQUENCE)
       {    // need to extract from both sequencers, since > 8 channels means 2 sequencers
          ADCSequenceDataGet (adc_module, 0,
                               (unsigned long*) channel_results);   // get sequencer 0 data first

          MAP_ADCIntClear (adc_module, 0);    // clear any lingering interrupt flags

          channel_results += 8;  // step past first sequencer results read in
       }

    ADCSequenceDataGet (adc_module, sequencer,
                            (unsigned long*) channel_results);   // get rest of ADC results

//       ==>  BIG PROBLEM  LONG vs SHORT !  ^^^^^^^^^^^^^^^^^^^^^^
//            CALLER IS PASSING IN SHORT, esp since all MSP430s use SHORTs

       // clear any lingering interrupt flags
    MAP_ADCIntClear (adc_module, sequencer);
//ADCSequenceDataGet (ADC0_BASE, 0, (unsigned long*) channel_results);
//ADCIntClear (ADC0_BASE, 0);

    return (_g_active_channels);   // return total number of successful conversions
}


//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a group of ADCs, on a sequenced group
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
    uint32_t    adc_module;

    if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
       return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
       return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    adc_module = (&_g_adc_modules[adc_module_id])->adc_base; //Get ADC Module Addr

    if (sequencer == ADC_AUTO_SEQUENCE)
       sequencer = _g_sequencer;       // use current auto-managed sequencer

    if (_g_active_channels > 8  &&  sequencer == ADC_AUTO_SEQUENCE)
       {    // need to trigger both sequencers, since > 8 channels means 2 sequencers
         ADCProcessorTrigger (adc_module, 0);     // Trigger sequencer 0 first
       }

    ADCProcessorTrigger (adc_module, sequencer);
//ADCProcessorTrigger (ADC0_BASE, 0);

    return (0);                 // denote success
}

#endif                          // defined(USES_ADC)



//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_gpio_init
//
//          Configure Port Directions and Peripherals as needed
//*****************************************************************************
void  board_gpio_init (void)
{
        // Enable clocks for most common GPIO GPIO peripherals: Ports A, B, E
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);

#if defined(USES_CC3100)
        //-----------------------------------------------------
        // Configure CC3100 SPI CS GPIO pin we are using (PE0)
        //-----------------------------------------------------
    MAP_GPIOPinWrite (GPIO_PORTE_BASE,GPIO_PIN_0,PIN_HIGH); //Ensure De-Asserted
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTE_BASE, GPIO_PIN_0);

        //------------------------------------------
        // Configure CC3100 nHIB/ENABLE (PE4) line
        //------------------------------------------
    MAP_GPIOPinWrite (GPIO_PORTE_BASE,GPIO_PIN_4, PIN_LOW); // Init as DISABLED
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTE_BASE, GPIO_PIN_4);

        //--------------------------------
        // configure CC3100 host IRQ line
        //--------------------------------
    MAP_GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_2);
    MAP_GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA,
                          GPIO_PIN_TYPE_STD_WPD);
    MAP_GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
    MAP_GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_2);
    MAP_GPIOIntDisable (GPIO_PORTB_BASE,GPIO_PIN_2);

#if defined(MOVED_TO_SPI_LOGIC)
        // the following was moved to the spi_Open() logic, to model
        // TI's support, and avoid any "early interrupts" that 
        // SimpleLink may not be expecting
    MAP_GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_2);
    MAP_GPIOIntDisable (GPIO_PORTB_BASE,GPIO_PIN_2);
        //--------------------------------
        // Enable CC3100 IRQ interrupts
        //--------------------------------
    IntEnable (INT_GPIOB);
    IntMasterEnable();
        // CC3100 needs 50 ms delay at startup
    MAP_SysCtlDelay ( (ROM_SysCtlClockGet()/(3*1000))*50 );
        //----------------------------------
        // Enable WLAN interrupt (IRQ line)
        //----------------------------------
    CC3100_InterruptEnable();
#endif                              // #if defined(MOVED_TO_SPI_LOGIC)
#endif                              // #if defined(USES_CC3100)

#if defined(USES_DRV8711)
        //------------------------------------------------------
        // Configure DRV8711 SPI CS GPIO pin we are using (PA2)
        //------------------------------------------------------
    DEASSERT_DRV8711_CS();                           // Ensure it is De-asserted
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_2);   // PA2 for CS

        // Setup DRV8711 input GPIO pins.  Best to be done _before_ outputs set
    MAP_GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_2);  // PB2 nSTALL
    MAP_GPIOPinTypeGPIOInput (GPIO_PORTE_BASE, GPIO_PIN_0);  // PE0 nFAULT

        // Setup DRV8711 output GPIO pins
    NSLEEP_LO_DISABLE;                     // Ensure DRV8711 is set disabled
        // PE5 nSLEEP/ENABLE
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTE_BASE, GPIO_PIN_5); 
    RESET_LO_RUN;                          // But leave DRV8711 reset turned off
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_5); // PA5 RESET
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_7); // PA7 DIR/A2
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_4); // PA4 BIN1
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_3); // PA3 BIN2

#endif                              // #if defined(DRV8711)
}


/*******************************************************************************
*  Board GPIO Pin Config
*
*        Configure an individual GPIO pin.   Use default strength = 2ma/pin
*******************************************************************************/
void  board_gpio_pin_config (uint32_t gpio_port, unsigned long pin,
                             int dir, int pull)
{
    unsigned long  res_pull_usage;


// need to ensure associated GPIO_BASE has had clocks turned on !!!  05/12/15
// else get random FaultISRs

    if (pull == GPIO_NOPULL)
       res_pull_usage = GPIO_PIN_TYPE_STD;
       else if (pull == GPIO_PULLUP)
               res_pull_usage = GPIO_PIN_TYPE_STD_WPU;
       else if (pull == GPIO_PULLDOWN)
               res_pull_usage = GPIO_PIN_TYPE_STD_WPD;
       else if (pull == GPIO_OPEN_DRAIN)
               res_pull_usage = GPIO_PIN_TYPE_OD;
       else res_pull_usage = GPIO_PIN_TYPE_STD;          // set to default

    if (dir == GPIO_OUTPUT)
       MAP_GPIOPinTypeGPIOOutput (gpio_port, pin);       // set pin to OUTPUT
       else MAP_GPIOPinTypeGPIOInput (gpio_port, pin);   // set pin to INPUT

    MAP_GPIOPadConfigSet (gpio_port, pin, GPIO_STRENGTH_2MA,
                          res_pull_usage);               // Setup Pull Up/Down
}


/*******************************************************************************
*  board_irq_pin_config
*
*        Configure an individual GPIO pin to support incoming Interrupt requests
*******************************************************************************/
void  board_irq_pin_config (uint32_t gpio_port,  unsigned long pin,
                            int rise_fall, int pullup, uint32_t irq_vector_num,
                            unsigned long priority)
{
    unsigned long   rupt_mode;
//  IRQn_Type       irq_rupt_vector;

    board_gpio_pin_config (gpio_port, pin, GPIO_INPUT, pullup);  // config base pin props
    if (rise_fall == GPIO_RUPT_MODE_RISING)
       rupt_mode = GPIO_RISING_EDGE;
       else if (rise_fall == GPIO_RUPT_MODE_FALLING)
               rupt_mode = GPIO_FALLING_EDGE;
               else rupt_mode = GPIO_BOTH_EDGES;

    MAP_GPIOIntTypeSet (gpio_port, pin, rupt_mode);     // set type of interrupt
    MAP_GPIOIntClear (gpio_port, pin);

        // Model TI's support, and avoid any "early interrupts" that 
        // ISR may not be expecting (e.g. CC3100 Simplelink)
    MAP_GPIOIntDisable (gpio_port, pin);

        //-----------------------------------------
        // Enable the associated IRQ's interrupts
        //-----------------------------------------
//  irq_rupt_vector = (IRQn_Type) irq_vector_num;
    IntEnable (irq_vector_num);                      // e.g. INT_GPIOB

// caller must call  board_irq_pin_enable();  when ready/init complete
}


/*******************************************************************************
*  board_irq_pin_enable
*
*        Enable the GPIO pin for an incoming interrupt
*******************************************************************************/
void  board_irq_pin_enable (uint32_t gpio_port,  unsigned long pin,
                            uint32_t irq_vector_num, int clear_pending)
{
   if (clear_pending)
      MAP_GPIOIntClear (gpio_port, pin);
    MAP_GPIOIntEnable (gpio_port, pin);
}


/*******************************************************************************
* board_spi_init                                  was   SPI2_Init
*
*         Configures this MCU's SPI peripheral and its associated GPIOs.
*         Returns the associated SPI handle/port id.
*
*                SCLK     MISO     MOSI
*                ----     ----     ----
*         SPI1 = PA5   /  PA6   /  PA7
*           "    PB3   /  PB4   /  PB5    (alternate pin mode)
*         SPI2 = PB13  /  PC2   /  PC3
*           "    PB13  /  PB14  /  PB15   (alternate pin mode)
*         SPI3 = PC10  /  PC11  /  PC12
*           "    PB3   /  PB4   /  PB5    (alternate pin mode)
*******************************************************************************/

uint32_t  board_spi_init (int spi_module_id,  int spi_mode,
                          int baud_rate_scalar,  int use_dma)
{
   uint32_t       spi_port_id;
   int            rc;

   spi_port_id = 0;

#if NEED_TO_BUILD_FOR_TIVA

#if defined(USE_DMA)
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  DMAx_CLK_ENABLE();

      /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = SPIx_TX_DMA_STREAM;

  hdma_tx.Init.Channel             = SPIx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init (&hdma_tx);

      /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA (hspi, hdmatx, hdma_tx);

      /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = SPIx_RX_DMA_STREAM;

  hdma_rx.Init.Channel             = SPIx_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init (&hdma_rx);

      /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA (hspi, hdmarx, hdma_rx);

      /*##-4- Configure the NVIC for DMA #####################################*/
      /* NVIC configuration for DMA transfer complete interrupt (SPI3_TX) */
  HAL_NVIC_SetPriority (SPIx_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ (SPIx_DMA_TX_IRQn);

      /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
  HAL_NVIC_SetPriority (SPIx_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (SPIx_DMA_RX_IRQn);
#endif                                        //  #if defined(USE_DMA)

#if defined(USE_DMA)
              //------------------------------------------
              // DMA init for SPI RX and TX directions
              //------------------------------------------
           hdma_spi2_rx.Instance                 = DMA1_Channel4;    // RX
           hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
           hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
           hdma_spi2_rx.Init.MemInc              = DMA_MINC_DISABLE; // ISSUE ???
           hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
           hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
           hdma_spi2_rx.Init.Mode                = DMA_NORMAL;
           hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
           HAL_DMA_Init (&hdma_spi2_rx);

           __HAL_LINKDMA (&hspi2, hdmarx, hdma_spi2_rx);

           hdma_spi2_tx.Instance                 = DMA1_Channel5;  // TX
           hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
           hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
           hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
           hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
           hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
           hdma_spi2_tx.Init.Mode                = DMA_NORMAL;
           hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
           HAL_DMA_Init (&hdma_spi2_tx);

           __HAL_LINKDMA (&hspi2, hdmatx, hdma_spi2_tx);
#endif                            // defined(USE_DMA)

         //--------------------------------------------------------------
         // This is a little bit overkill, but it handles all current
         // SPI variations of supported X-Nucleo boards. In most cases
         // this allows many diffrent combinations of stacked X-Nucleo
         // boards (subject to resolving CS conflicts, etc).
         //--------------------------------------------------------------

    switch (spi_id)                    // STM32  F401RE  SPI   Support
      {
       case SPI_ID_1_A:                // SPI 1 A    uses pins PA5/PA6/PA7
                                       // Typically used by L6474 Stepper shield
               __SPI1_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
               phspi = &hspi1;         // point at associated SPI "Handle"
               phspi->Instance = SPI1; //    and set associated SPI module
               hspi1_type = SPI_ID_1_A;
                    // SPI 1 A uses GPIO pins PA5/PA6/PA7. 
                    // Set them to SPI Alt Function
               GPIO_InitStruct.Pin       = SPI_1_A_SCLK_PIN;
               GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
               GPIO_InitStruct.Pull      = SPI_1_A_SCLK_PULL_MODE;
               GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
               HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);     // Setup PA5 SPI pin
               GPIO_InitStruct.Pin       = SPI_1_A_MISO_PIN | SPI_1_A_MOSI_PIN;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
               GPIO_InitStruct.Pull      = SPI_1_A_MOSx_PULL_MODE;
               HAL_GPIO_Init (GPIOA, &GPIO_InitStruct); //Setup PA6/PA7 SPI pins
               break;

       case SPI_ID_1_B:                // SPI 1 B    uses pins PB3/PA6/PA7
                                       // Typically used by Blue_NRG BLE shield
               __SPI1_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
#if (USES_BLUENRG_BLE)
               phspi = &SpiHandle;   // point at BlueNRG's required SPI "Handle"
#else
               phspi = &hspi1;       // point at our associated SPI "Handle"
#endif
               phspi->Instance = SPI1; //    and set associated SPI module
               hspi1_type = SPI_ID_1_B;
                    // SPI 1 B uses GPIO pins PB3/PA6/PA7. 
                    // Set them to SPI Alt Function
               GPIO_InitStruct.Pin       = SPI_1_B_SCLK_PIN;
               GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
               GPIO_InitStruct.Pull      = SPI_1_B_SCLK_PULL_MODE; // ST logic !
               GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
               HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);    // Setup PB3 SPI pin
               GPIO_InitStruct.Pin       = SPI_1_B_MISO_PIN | SPI_1_B_MOSI_PIN;
               GPIO_InitStruct.Pull      = SPI_1_B_MOSx_PULL_MODE; // ST logic !
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
               HAL_GPIO_Init (GPIOA, &GPIO_InitStruct); //Setup PA6/PA7 SPI pins
               break;

       case SPI_ID_2_C:                // SPI 2 C    uses pins PB13/PC2/PC3
               __SPI2_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
               phspi = &hspi2;         // point at associated SPI "Handle"
               phspi->Instance = SPI2; //    and set associated SPI module
               hspi2_type = SPI_ID_2_C;
                    // SPI 2 C uses GPIO pins PB13/PC2/PC3. 
                    // Set them to SPI Alt Function
               GPIO_InitStruct.Pin       = SPI_2_C_SCLK_PIN;
               GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
               GPIO_InitStruct.Pull      = SPI_2_C_SCLK_PULL_MODE;
               GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
               HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);    // Setup PB13 SPI pin
               GPIO_InitStruct.Pin       = SPI_2_C_MISO_PIN | SPI_2_C_MOSI_PIN;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
               GPIO_InitStruct.Pull      = SPI_2_C_MOSx_PULL_MODE;
               HAL_GPIO_Init (GPIOC, &GPIO_InitStruct); //Setup PC2/PC3 SPI pins
#if defined(USE_DMA)
                   //----------------------------------------
                   //    SPI2  Interrupts  Enable
                   //----------------------------------------
               board_spi_dma_init();
               HAL_NVIC_SetPriority (SPI2_IRQn, 0, 0);
               HAL_NVIC_EnableIRQ (SPI2_IRQn);
#endif
               break;

       case SPI_ID_3_C:               // SPI 3 C    uses pins PC10/PC11/PC12
                                      // Typically used by W5200 Ethernet shield
               __SPI3_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
               phspi = &hspi3;         // point at associated SPI "Handle"
               phspi->Instance = SPI3; //    and set associated SPI module
               hspi3_type = SPI_ID_3_C;
                    // SPI 3 C uses GPIO pins PC10/PC11/PC12. 
                    // Set them to SPI Alt Function
               GPIO_InitStruct.Pin       = SPI_3_C_SCLK_PIN;
               GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
               GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
               GPIO_InitStruct.Pull      = SPI_3_C_SCLK_PULL_MODE;
               GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
               HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);  // Setup PC10/ SPI pin
               GPIO_InitStruct.Pin       = SPI_3_C_MISO_PIN | SPI_3_C_MOSI_PIN;
               GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
               GPIO_InitStruct.Pull      = SPI_3_C_MOSx_PULL_MODE;
               HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);  // Setup PC11/PC12 SPI
               break;
      }
         //------------------------------------------------------------
         //              Setup rest of SPI Configuration
         //------------------------------------------------------------
    phspi->Init.Mode            = SPI_MODE_MASTER;
    phspi->Init.Direction       = SPI_DIRECTION_2LINES;
    phspi->Init.DataSize        = SPI_DATASIZE_8BIT;
    if (spi_mode == 0)
      { phspi->Init.CLKPolarity = SPI_POLARITY_LOW;
        phspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spi_mode == 1)
      { phspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
        phspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spi_mode == 2)
      { phspi->Init.CLKPolarity = SPI_POLARITY_LOW;  // CPOL = 0, CPHA = 1
        phspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      }
     else if (spi_mode == 3)
      { phspi->Init.CLKPolarity = SPI_POLARITY_HIGH; //EASY_SPIN, PRESCALE = 32
        phspi->Init.CLKPhase    = SPI_PHASE_2EDGE;   //Ada ST7735 LCD SCALE = 8
      }
    phspi->Init.NSS             = SPI_NSS_SOFT;
    phspi->Init.FirstBit        = SPI_FIRSTBIT_MSB;
    phspi->Init.CRCCalculation  = SPI_CRCCALCULATION_DISABLED;
    phspi->Init.CRCPolynomial   = 7;
    phspi->Init.TIMode          = SPI_TIMODE_DISABLED;
//  phspi->Init.NSSPMode        = SPI_NSS_Soft;          // WVD CHange 03/11/15

       // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
    phspi->Init.BaudRatePrescaler = baud_rate_scalar;

    HAL_SPI_Init (phspi);

// UNTIL CONVERT ALL ST CODE OVER, YOU MUST ALSO INITIALIZE THEIR SpiHandle
// struct WHICH THEIR CODE USES.  ELSE, SPI GOES OUT TO LUNCH. 
//     3.5+ HOUR PAINFUL LESSON.
// memcpy (&SpiHandle, phspi, sizeof(SpiHandle));
// HAL_SPI_Init (&SpiHandle);

    __HAL_SPI_ENABLE (phspi);

#endif                      // NEED_TO_BUILD_FOR_TIVA


    return (spi_port_id);   // return ptr to SPI module that will be used
                            // for caller's SPI device I/O
}


/*******************************************************************************
* board_spi_dma_init
*
*         Enable DMA controller clock and initialize DMA Interrupts
*******************************************************************************/
void  board_spi_dma_init (void)
{
#if defined(DMA_ENABLED)
    __DMA1_CLK_ENABLE();               // DMA controller clock enable

        //--------------------------------------------
        //    DMA interrupts init
        //--------------------------------------------
    HAL_NVIC_SetPriority (DMA1_Channel4_IRQn, 0, 0);   // SPI RX
    HAL_NVIC_EnableIRQ (DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority (DMA1_Channel5_IRQn, 0, 0);   // SPI TX
    HAL_NVIC_EnableIRQ (DMA1_Channel5_IRQn);
#endif
}



#if defined(USES_PWM) || defined(USES_DRV8711) || defined(USES_DRV8848) || defined(USES_DRV8301) 

//*****************************************************************************
//*****************************************************************************
//                               PWM   Routines
//*****************************************************************************
//*****************************************************************************
#define  PWM_NUM_MODULES       8   /* are 8 PWM capable modules on Tiva123 LP */
#define  PWM_MAX_CHANNELS     10   /* are max 10 channels per PWM module group*/

#define  PWM_DOWN_COUNT_INIT   1   /* Module setup for DOWN count mode */
#define  PWM_UP_COUNT_INIT     2   /* Module setup for UP   count mode */
#define  PWM_CENTER_COUNT_INIT 3   /* Module setup for UP/DOWN count mode */

      char               _g_pwm_module_status [PWM_NUM_MODULES+1]  = { 0,0,0,0 };
      unsigned short     _g_pwm_channel_config [PWM_NUM_MODULES+1] = { 0,0,0,0 };

const unsigned short     _g_pwm_chan_mask [13]
                                   = { 0,       // unused
                                       0x0001, 0x0002, 0x0004, 0x0008,  // Channels  1 - 4
                                       0x0010, 0x0020, 0x0040, 0x0080,  // Channels  5 - 8
                                       0x0100, 0x0200, 0x0000, 0x0000   // Channels  9 - 10
                                     };

typedef struct pwm_module_def       /* PWM Module/Generator definitions */
    {
        uint32_t  pwm_base;         /* PWM base address */
        uint32_t  pwm_generator;    /* PWM generator id */
    } PWM_MODULE_BLK;

typedef struct pwm_interrupt_def    /* PWM Module/Generator Interrupt definitions */
    {
        uint32_t  pwm_base;         /* PWM base address */
        uint32_t  pwm_gen_int_id;   /* PWM generator interrupt id   */
        uint32_t  pwm_nvic_int_id;  /* Associated NVIC interrupt id */
    } PWM_INTERRUPT_BLK;

typedef struct pwm_channel_def         /* PWM Channel definitions */
    {
        uint32_t  chan_alt_func;       /* Channel Alternate Function Id */
        uint32_t  chan_gpio_pin;       /* Associated GPIO pin           */
        uint32_t  chan_gpio_port;      /* Associated GPIO port           */
        uint32_t  chan_out_logical_id; /* Logical Output  id for this PWM channel */
        uint32_t  chan_out_phys_bit;   /* Physical Output id for this PWM channel */
    } PWM_CHANNEL_BLK;


// ALTERNATIVE MAPPING:  Map according to Jx-n connector so same
//                       Jx-n pins on Tiva map to same Jx-n pins on MSP430
//                       FLY in Ointment is that Module Naming is so
//                       different on Tiva vs MSP430, _and_
//                       a MSP430 module controls 4 channels, but a Tiva controls 2 channels.
//                         ==> App has to have some sensitive to module mapping vs Chip
//                       NOODLE ON THIS
//                         ==> Anyway to make PWM Module epheral, and just go off channel_name
//                             or generic PWM_OUT_n
//                         ==> Problem is for complementary stuff, you MUST be sensitive !
//                             since a given period is associated with a given set of channels
//                       Supplying standard USER API mappings for CC3000, DRV8848, ... that
//                       hide the differences between the diff platforms is a partial solution.
const PWM_MODULE_BLK  _g_pwm_modules [] =
        {  {    0L,        0L,    },
           { PWM0_BASE, PWM_GEN_0 },
           { PWM0_BASE, PWM_GEN_1 },
           { PWM0_BASE, PWM_GEN_2 },
           { PWM0_BASE, PWM_GEN_3 },

           { PWM1_BASE, PWM_GEN_0 },
           { PWM1_BASE, PWM_GEN_1 },
           { PWM1_BASE, PWM_GEN_2 },
           { PWM1_BASE, PWM_GEN_3 }
        };


const PWM_INTERRUPT_BLK  _g_pwm_interrupts [] =
        {                   // Used to Enable PWM Generator interrupts on NVIC
           {    0L,           0L,          0L     },
           { PWM0_BASE, PWM_INT_GEN_0, INT_PWM0_0 },
           { PWM0_BASE, PWM_INT_GEN_1, INT_PWM0_1 },
           { PWM0_BASE, PWM_INT_GEN_2, INT_PWM0_2 },
           { PWM0_BASE, PWM_INT_GEN_3, INT_PWM0_3},

           { PWM1_BASE, PWM_INT_GEN_0, INT_PWM1_0 },
           { PWM1_BASE, PWM_INT_GEN_1, INT_PWM1_1 },
           { PWM1_BASE, PWM_INT_GEN_2, INT_PWM1_2 },
           { PWM1_BASE, PWM_INT_GEN_3, INT_PWM1_3}
        };


const PWM_CHANNEL_BLK  _g_pwm_channels_pwm0 [] =
        {  {       0L,           0L,         0L,         0L        },
           { GPIO_PB6_M0PWM0, GPIO_PIN_6, GPIO_PORTB_BASE, PWM_OUT_0, PWM_OUT_0_BIT }, // M0-PWM0 Gen0
           { GPIO_PB7_M0PWM1, GPIO_PIN_7, GPIO_PORTB_BASE, PWM_OUT_1, PWM_OUT_1_BIT }, // M0-PWM1
           { GPIO_PB4_M0PWM2, GPIO_PIN_4, GPIO_PORTB_BASE, PWM_OUT_2, PWM_OUT_2_BIT }, // M0-PWM2 Gen1
           { GPIO_PB5_M0PWM3, GPIO_PIN_5, GPIO_PORTB_BASE, PWM_OUT_3, PWM_OUT_3_BIT }, // M0-PWM3
           { GPIO_PE4_M0PWM4, GPIO_PIN_4, GPIO_PORTE_BASE, PWM_OUT_4, PWM_OUT_4_BIT }, // M0-PWM4 Gen2
           { GPIO_PE5_M0PWM5, GPIO_PIN_5, GPIO_PORTE_BASE, PWM_OUT_5, PWM_OUT_5_BIT }, // M0-PWM5
           { GPIO_PC4_M0PWM6, GPIO_PIN_4, GPIO_PORTC_BASE, PWM_OUT_6, PWM_OUT_6_BIT }, // M0-PWM6 Gen3
           { GPIO_PC5_M0PWM7, GPIO_PIN_5, GPIO_PORTC_BASE, PWM_OUT_7, PWM_OUT_7_BIT }, // M0-PWM7
           { GPIO_PD0_M0PWM6, GPIO_PIN_0, GPIO_PORTD_BASE, PWM_OUT_6, PWM_OUT_6_BIT },  // Alternate PWM6/PWM7
           { GPIO_PD1_M0PWM7, GPIO_PIN_1, GPIO_PORTD_BASE, PWM_OUT_7, PWM_OUT_7_BIT },
        };

const PWM_CHANNEL_BLK  _g_pwm_channels_pwm1 [] =
        {  {       0L,           0L,         0L,         0L        },
           { GPIO_PD0_M1PWM0, GPIO_PIN_0, GPIO_PORTD_BASE, PWM_OUT_0, PWM_OUT_0_BIT }, // M1-PWM0 Gen0
           { GPIO_PD1_M1PWM1, GPIO_PIN_1, GPIO_PORTD_BASE, PWM_OUT_1, PWM_OUT_1_BIT }, // M1-PWM1
           { GPIO_PA6_M1PWM2, GPIO_PIN_6, GPIO_PORTA_BASE, PWM_OUT_2, PWM_OUT_2_BIT }, // M1-PWM2 Gen1
           { GPIO_PA7_M1PWM3, GPIO_PIN_7, GPIO_PORTA_BASE, PWM_OUT_3, PWM_OUT_3_BIT }, // M1-PWM3
           { GPIO_PF0_M1PWM4, GPIO_PIN_0, GPIO_PORTF_BASE, PWM_OUT_4, PWM_OUT_4_BIT }, // M1-PWM4 Gen2
           { GPIO_PF1_M1PWM5, GPIO_PIN_1, GPIO_PORTF_BASE, PWM_OUT_5, PWM_OUT_5_BIT }, // M1-PWM5
           { GPIO_PF2_M1PWM6, GPIO_PIN_2, GPIO_PORTF_BASE, PWM_OUT_6, PWM_OUT_6_BIT }, // M1-PWM6 Gen3
           { GPIO_PF3_M1PWM7, GPIO_PIN_3, GPIO_PORTF_BASE, PWM_OUT_7, PWM_OUT_7_BIT }, // M1-PWM7
           { GPIO_PE4_M1PWM2, GPIO_PIN_4, GPIO_PORTE_BASE, PWM_OUT_2, PWM_OUT_2_BIT },  // Alternate PWM2/PWM3
           { GPIO_PE5_M1PWM3, GPIO_PIN_5, GPIO_PORTE_BASE, PWM_OUT_3, PWM_OUT_3_BIT }
        };


//*****************************************************************************
//  board_pwm_init
//                  was  timer_pwm_init
//
//         Initialize PWM mode
//
//         flags parm is for future use.
//
//  Tiva pin PA6 uses M1PWM2, which is Module 1 Output 2.
//  It is controlled by                Module 1 PWM, Generator 1.
//
//  Tiva PWM nomenclature:
//      PWM_GEN_0 Covers M1PWM0 and M1PWM2 = PWM_OUT_0_BIT / _1_BIT
//      PWM_GEN_1 Covers M1PWM2 and M1PWM3 = PWM_OUT_2_BIT / _3_BIT
//      PWM_GEN_2 Covers M1PWM4 and M1PWM5 = PWM_OUT_4_BIT / _5_BIT
//      PWM_GEN_3 Covers M1PWM6 and M1PWM7 = PWM_OUT_6_BIT / _7_BIT
//*****************************************************************************

int  board_pwm_init (int modgen_id, int count_mode, long period, int flags)
{                                                  // ^^^^^^^^^^^^  pass in frequency instead and convert internally. RAISE THE API for OBVIOUS STUFF
    PWM_MODULE_BLK    *pwmodg;
    PWM_INTERRUPT_BLK *pwrupt;
    uint32_t          prescalar;
    uint32_t          scaledperiod;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (count_mode == TIMER_PERIODIC_COUNT_UP || count_mode == TIMER_PERIODIC_COUNT_UPDOWN)
       ;                                           // mode is valid
      else return (ERR_PWM_COUNT_MODE_INVALID);    // count_mode is bad - bail

    pwmodg = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];
    pwrupt = (PWM_INTERRUPT_BLK*) &_g_pwm_interrupts [modgen_id];

// ??? THIS ALMOST SETs THE PRE_SCALAR FOR A MODULE - need to generisize it for MULTIPLE MODULES
//     and put sanity checks in that user does not pass in an out of range new value when run split timers/sep pins
//     OR do module level pwm_init, then separate inits for pins attached to it
    if ( ! _g_pwm_mclock_configured)
       {
              //-------------------------------------------------------------
              // do basic setup for PWM and Interval Timer  (turn on clocks)   ??? MOVE TO  board_pwm_init with configured_flag
              // 80,000,000 / 65,000 = 1230  ==> need to vivide by 8 or 16 for worst case ?
              // 80,000,000 / 60 Hz  =   24  ==> need to divide by 32
              //-------------------------------------------------------------
// ??? STILL HAVE ISSUES !!!  UNLESS CAN PUT PRE-SCALARS on EACH module
          _g_pwm_mdivider = 1;          // default
          prescalar = SYSCTL_PWMDIV_1;
          if (period < 10)
             { _g_pwm_mdivider = 64;
               prescalar = SYSCTL_PWMDIV_64;
             }
            else if (period < 100)                // VERIFY RANGES
                    { _g_pwm_mdivider = SYSCTL_PWMDIV_32;
                      prescalar = SYSCTL_PWMDIV_32;
                    }
            else if (period < 500)
                    { _g_pwm_mdivider = SYSCTL_PWMDIV_8;
                      prescalar = SYSCTL_PWMDIV_8;
                    }
            else if (period < 2000)
                    { _g_pwm_mdivider = SYSCTL_PWMDIV_4;
                      prescalar = SYSCTL_PWMDIV_4;
                    }
             // setup the base PWM clock for ALL modules
          MAP_SysCtlPWMClockSet (prescalar);      // set PWM clock as a sub-multiple of CPU clock
//                              ^^^^^^^^^^^^^
//                             This is SYSTEM level

          MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM0);  // turn on PWM0 module's clock
          MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM1);  // turn on PWM1 module's clock
          _g_pwm_mclock_configured = 1;
       }

       //------------------------------------------------
       // Setup PWM module and associated Count mode.
       // Tiva only supports DOWN counts, not UP counts
       //------------------------------------------------
    if (count_mode == TIMER_PERIODIC_COUNT_DOWN)
       MAP_PWMGenConfigure (pwmodg->pwm_base, pwmodg->pwm_generator, PWM_GEN_MODE_DOWN);
       else MAP_PWMGenConfigure (pwmodg->pwm_base, pwmodg->pwm_generator, PWM_GEN_MODE_UP_DOWN);

       //-------------------------------------------
       //  setup initial PWM Period
       //-------------------------------------------
/// _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value - THIS IS REDUNDANT IF CALLER SUPPLIES PERIOD ALREADY !
/// scaledperiod = (_g_PWM_IO_clock / period) - 1;               // set initial period         - THIS IS REDUNDANT IF CALLER SUPPLIES PERIOD ALREADY !
    scaledperiod = (period / _g_pwm_mdivider) - 1;               // scale period to match PWM pre-scalar
// ??? can any pre-scalars be applied to this PWM1_BASE clock ???  allegedly there should be a way
    MAP_PWMGenPeriodSet (pwmodg->pwm_base, pwmodg->pwm_generator, scaledperiod);

       //--------------------------------------------------------------
       // Setup PWM Interrupts but do not enable all of them yet
       // Final PWMGenIntTrigEnable() is issued in timer_enable_pwm().
       //--------------------------------------------------------------
       //--------------------------------------------------------
       // Setup any needed interrupts for this PWM module.
       //
       // At the module level, allow Interrupts from GEN_1
       // if the Generator has them enabled.
       //--------------------------------------------------------
    if (flags == PWM_ENABLE_MODULE_INTERRUPTS)
       { MAP_PWMIntEnable (pwmodg->pwm_base, pwrupt->pwm_gen_int_id);

             // Enable the associated PWM Generator interrupt on the processor (NVIC).
         IntEnable (pwrupt->pwm_nvic_int_id);
       }

    _g_pwm_module_status [modgen_id] = count_mode;  // save count mode

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_config_channel
//
//         Configure a specific channel (1-8) on a specific PWM Generator
//
//         For TIM1, allow complmentary mode usage.
//*****************************************************************************

int  board_pwm_config_channel (int modgen_id, int chan_num,
                               long initial_duty, int flags)
{
    PWM_MODULE_BLK    *pwmodg;
    PWM_CHANNEL_BLK   *pwmchan;
    uint32_t          scaleduty;
    unsigned char     chan_mask;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    pwmodg  = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

    if (modgen_id <= 4)
       pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm0 [chan_num];
       else pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm1 [chan_num];

    chan_mask = _g_pwm_chan_mask [chan_num];   // get associated channel mask

       //---------------------------------------------------------------
       // Set up GPIO pin to be used for this PWM Channel into PWM mode
       //---------------------------------------------------------------
    MAP_GPIOPinTypePWM (pwmchan->chan_gpio_port, pwmchan->chan_gpio_pin);
    MAP_GPIOPinConfigure (pwmchan->chan_alt_func);      // set assoc Alt Function

// ??? have to divide duty cycle based on what did for initial period ???

//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / initial_duty) - 1;   // set new scaled duty cycle   - THIS GENERATE INCORRECT DUTY CYCLES !!!  05/12/15
    scaleduty = (initial_duty / _g_pwm_mdivider) - 1;   // set new scaled duty cycle
                                                        // Use PERIOD and apply same divider direct to duty cycle
            //-------------------------------------
            // set PWM duty cycle for this Channel
            //-------------------------------------
    MAP_PWMPulseWidthSet (pwmodg->pwm_base, pwmchan->chan_out_logical_id, scaleduty);

// be sure to clear out any stale interrupts from RIS !     05/12/15

            //-------------------------------------
            // Enable PWM pin output 
            //-------------------------------------
    MAP_PWMOutputState (pwmodg->pwm_base, pwmchan->chan_out_phys_bit, true);

       //--------------------------------------------------------------------------
       // On any kind of fault, including Debug Stops, set PWM Channel output LOW.
       // (During debugging, PWM was normally being frozen HIGH ! )
       //--------------------------------------------------------------------------
    MAP_PWMOutputFaultLevel (pwmodg->pwm_base, pwmchan->chan_out_phys_bit, false); // drive low
    MAP_PWMOutputFault (pwmodg->pwm_base, pwmchan->chan_out_phys_bit, true); // enable fault handling

    _g_pwm_channel_config [modgen_id] |= chan_mask; // denote channel was configured

    return (0);               // denote successful configuration
}


//*****************************************************************************
// board_pwm_disable
//
//            Disable the Speed or PWM timer.
//            (in preparation for Stopping or Reconfiguring the motor)
//*****************************************************************************
int  board_pwm_disable (int modgen_id, int flags)
{
    PWM_MODULE_BLK    *pwmodg;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    pwmodg = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

       // Disable PWM Interrupts
    MAP_PWMGenIntTrigDisable (pwmodg->pwm_base, pwmodg->pwm_generator,
                              PWM_INT_CNT_LOAD | PWM_INT_CNT_AD);

/// MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false); //Disable PWM pin output  ??? NEED ??? - no hook for PWM_OUT_2_BIT

    MAP_PWMGenDisable (pwmodg->pwm_base, pwmodg->pwm_generator);

    return (0);                // denote processed successfully
}


//*****************************************************************************
// board_pwm_enable
//
//            Enable and Start the timer in Speed or PWM mode.
//*****************************************************************************
int  board_pwm_enable (int modgen_id, int flags)
{
    PWM_MODULE_BLK    *pwmodg;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    pwmodg = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

#if (NEEDS_WORK_CAUSING_DefaultIsr_Rupts)
       // Re-enable PWM Interrupts
    MAP_PWMGenIntTrigEnable (pwmodg->pwm_base, pwmodg->pwm_generator,
                             PWM_INT_CNT_LOAD | PWM_INT_CNT_AD);
#endif

    MAP_PWMGenEnable (pwmodg->pwm_base, pwmodg->pwm_generator);  // start up the PWM ??? need

/// MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, true); // Enable PWM pin output  ??? NEED ??? - no hook for PWM_OUT_2_BIT

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_period
//
//         Set aa new PWM Period.
//*****************************************************************************

int  board_pwm_set_period (int modgen_id, long period)
{
    PWM_MODULE_BLK    *pwmodg;
    uint32_t          scaledperiod;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    pwmodg = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

/// _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value - THIS IS REDUNDANT IF CALLER SUPPLIES PERIOD ALREADY !
/// scaledperiod = (_g_PWM_IO_clock / period) - 1;               // set initial period         - THIS IS REDUNDANT IF CALLER SUPPLIES PERIOD ALREADY !
    scaledperiod = (period / _g_pwm_mdivider) - 1;               // scale period to match PWM pre-scalar

            // set PWM period and starting duty cycle into PWM module
    MAP_PWMGenPeriodSet (pwmodg->pwm_base, pwmodg->pwm_generator, scaledperiod);

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_duty_cycle
//
//         Set the PWM's duty cycle.
//         Scale it if necessary, based on any pre-scalars we had
//         to use on the PWM's base Clock.
//*****************************************************************************

int  board_pwm_set_duty_cycle (int modgen_id, int chan_num, long duty_cycle)
{
    PWM_MODULE_BLK   *pwmodg;
    PWM_CHANNEL_BLK  *pwmchan;
    uint32_t         scaleduty;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    pwmodg  = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

    if (modgen_id <= 4)
       pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm0 [chan_num];
       else pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm1 [chan_num];

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle   - THIS GENERATE INCORRECT DUTY CYCLES !!!  05/12/15
    scaleduty = (duty_cycle / _g_pwm_mdivider) - 1;   // set new scaled duty cycle
                                                      // Use PERIOD and apply same divider direct to duty cycle

            // set PWM duty cycle for this Channel
    MAP_PWMPulseWidthSet (pwmodg->pwm_base, pwmchan->chan_out_logical_id, scaleduty);

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_dead_time
//
//         Set the PWM's dead time between complementary PWM
//         arrangements.  Use to avoid "shoot through" on the 
//         output FETs when working with Power or Motor H-bridges.
//*****************************************************************************

int  board_pwm_set_dead_time (int modgen_id, int rising_edge, int falling_edge)
{
    PWM_MODULE_BLK    *pwmodg;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    pwmodg = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

// TBD    ???   !!!

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_phase
//
//         Set the PWM's phase between complementary PWM
//         arrangements.  
//*****************************************************************************

int  board_pwm_set_phase (int modgen_id, int channel_num, long phase_offset)
{
   return (ERR_PWM_MODULE_NO_PHASE);  // Tiva PWMs do NOT support
                                      // phase shifting. Only C2000 modules do
}


//*****************************************************************************
//  board_pwm_set_channel_output
//
//         Set polarity of channel output (start high or low)
//*****************************************************************************
int  board_pwm_set_channel_output (int modgen_id, int chan_num,
                                   int output_mode, int flags)
{
    PWM_MODULE_BLK    *pwmodg;
    PWM_CHANNEL_BLK   *pwmchan;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    pwmodg  = (PWM_MODULE_BLK*) &_g_pwm_modules [modgen_id];

    if (modgen_id <= 4)
       pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm0 [chan_num];
       else pwmchan = (PWM_CHANNEL_BLK*) &_g_pwm_channels_pwm1 [chan_num];


      // ??? is this supported on MSP432 - perhaps tweak TAxCCTLx = OUTMOD_
      //     to do set/reset instead of reset/set ?


//  return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    return (0);                // denote completed successfully
}

#endif                         // defined(USES_PWM)



//*****************************************************************************
//*****************************************************************************
//                          System CPU / Systick   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_stop_WDT
//
//           Shutoff WDT
//*****************************************************************************

void  board_stop_WDT (void)
{
    // WDT is disabled on startup on Tiva
}


//*****************************************************************************
//  board_system_clock_config
//
//          Setup CPU clocks.
//          Run Tiva 123G CPU at 50 MHz.
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{
       // The FPU should be enabled because some compilers use floating-
       // point registers, even for non-floating-point code.  
       // If the FPU is not enabled this will cause a fault.
       // This also ensures that floating-point operations could be
       // added to this application and would work correctly and use the
       // hardware floating-point unit. Finally, lazy stacking is 
       // enabled for interrupt handlers.  This allows floating-point
       // instructions to be used within interrupt handlers, but at the
       // expense of extra stack usage.
    FPUEnable();
    FPULazyStackingEnable();

       // Initialize the device using the on-board 16 MHz Crystal as clock.
       // The PLL multiplies the 16 MHz crystal up to 200 MHz.
       // For 20 MHz, configure M4 Clock with PLL to (200 MHz /10) = SYSCTL_SYSDIV_10
       // For 50 MHz, configure M4 Clock with PLL to (200 MHz / 4) = SYSCTL_SYSDIV_4
       // For 80 MHz, configure M4 Clock with PLL to (200 MHz/2.5) = SYSCTL_SYSDIV_2_5
    MAP_SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                        | SYSCTL_XTAL_16MHZ);

    _g_SysClk_Ticks = MAP_SysCtlClockGet();   // save total # clock ticks/sec

       // For busy_wait, 1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
       //                               (80 MHz / 1 million) / 3 = 26.6667
    _g_fusec_to_ticks = ((float) _g_SysClk_Ticks / 3000000.0);
}


//*****************************************************************************
//  board_system_clock_get_frequency
//
//       Return the board's MCU clock frequency in ticks. (usually = MHz value)
//*****************************************************************************

long  board_system_clock_get_frequency (void)
{
    return (_g_SysClk_Ticks);     // return the MCU clock frequency in ticks
}


//*****************************************************************************
//  board_sys_IO_clock_get_frequency
//
//       Return the board's Peripheral IO clock frequency in ticks.
//       On Tiva, we set I/O clock to same rate as MCU clock.
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SysClk_Ticks);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
// board_systick_timer_config
//
//          Provide a "SYSTICK" style Interval Timer.
//
//*****************************************************************************
void  board_systick_timer_config (void)
{
        // Initialize the SysTick Timer and its interrupt
    MAP_SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();
    IntMasterEnable();
}


//*****************************************************************************
// board_systick_timer_get_value
//
//            Get the current value of the "SYSTICK" style Interval Timer.
//            It returns a unsigned long of how many millseconds have
//            elapsed since the board started up.
//*****************************************************************************
unsigned long  board_systick_timer_get_value (void)
{
    return (_g_systick_millisecs);
}


//*****************************************************************************
//                               SYSTICK   ISR
//
// Handles all Systick interrupts
//
// Note that the max SYSTICK value = 0xFFFFFFFF = 1193 hours (49 days)
//      so logic to handle that kind of wrap-around is required in code that
//      uses SYSTICK  e.g. in board_delay_ms(), board_vtimer_check(), etc
//*****************************************************************************

void  SysTick_ISR_Handler (void)
{
    _g_systick_millisecs++;    // update # elapsed Systicks (poor man's TOD)

#if defined(USES_MQTT)
    SysTickIntHandler();       // invoke MQTTCC3100.c to update its Timer
#endif

#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration();
#endif

}



//*****************************************************************************
//*****************************************************************************
//                               UART   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
// board_uart_init
//*****************************************************************************

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)

void  board_uart_init (void)
{
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);

        // Set GPIOs PA0 and PA1 into UART mode.
    MAP_GPIOPinConfigure (GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure (GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART (GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Configure the UART Module for 115,200, 8-N-1 operation, then enable it
    MAP_UARTConfigSetExpClk (UART0_BASE, MAP_SysCtlClockGet(), 115200,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
#endif


//*****************************************************************************
//  board_uart_rx_data_check                    aka   CONSOLE_CHECK_FOR_INPUT
//
//             Checks if any pending character input from UART.
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

#if defined(USES_CONSOLE_READ)

int  board_uart_rx_data_check (void)
{
    int  rc;

    rc = MAP_UARTCharsAvail (UART0_BASE); // see if any rcvd chars are in UART
    if (rc)
       return (1);                        // we have some data

    return (0);                           // user has not typed in any char
}
#endif


//*****************************************************************************
//  board_uart_get_char                                 aka   CONSOLE_GET_CHAR
//
//             Read in a single raw character from the UART.
//
//             if there is no character available, then we loop in here
//             for UART input. Hence, this is a blocking call.
//*****************************************************************************

char  board_uart_get_char (void)
{

#if defined(USES_CONSOLE_READ) || defined(USES_CONSOLE_WRITE)
    char  in_char;
    int   rc;

           // read in any character that the user typed in
     rc = MAP_UARTCharsAvail (UART0_BASE);  // see if any rcvd chars are in UART
     if ( ! rc)
        return (0);                         // no UART data is present

     in_char = MAP_UARTCharGet (UART0_BASE);  // read in char from UART

#if defined(USES_CONSOLE_READ)
	// Check if a previous board_uart_read_string() had left a dangling \r\n situation.
     if (_g_uart_last_char == '\r'  &&  in_char == '\n')
        {      // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
           _g_uart_last_char = 0;    // clear out the \r, so we treat any
                                    // new \n as real.
          return (0);               // "nothing to see here"
        }

     _g_uart_last_char = 0;          // always clear out any old \r condition
#endif

     return (in_char);
#endif

}


//*****************************************************************************
//  board_uart_read_string                           aka   CONSOLE_READ_STRING
//
//             Reads a full string of characters from the UART.
//
//             When we hit an end of string sequence, e.g. \r\n (Windows)
//             or \n (Linux/Unix), we return a 1 to denote a valid cmd present.
//
//             We normally strip off the trailing \n or \r and replace it with
//             a \0, UNLESS the buffer is empty. In that case we put in \n\0
//             so that the App can determine that it is a "null" line (no
//             string, just a carriage return/line feed was hit at the console).
//
//             If it is not and end of line \n or \r, we continue to loop,
//             waiting for more UART input. (So yes, this is a blocking call).
//
//             If we reach the end of the buffer with no \n end of sequence,
//             then we truncate the cmd and hand up the buffer with \0 at end.
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//             Also note that if we use TI's uartstdio.c support, then we need
//             to worry about "viral" software. Horrors !
//*****************************************************************************

#if defined(USES_CONSOLE_READ)

int  board_uart_read_string (char *read_buf, int buf_max_length)
{

    char  in_char;
    int   rc;
                  //---------------------------------------------
                  // loop until we get \r, \n, or a full buffer
                  //---------------------------------------------
  while (1)
    {
           // read in any character that the user typed in
     rc = MAP_UARTCharsAvail (UART0_BASE);  // see if any rcvd chars are in UART
     if ( ! rc)
        continue;                           // user needs to type in more chars

     in_char = MAP_UARTCharGet (UART0_BASE);    // read in char from uART

        // note: board_uart_read_string MUST ALSO echo back the char to user
     if (in_char != '\n')
        MAP_UARTCharPut (UART0_BASE, in_char);  // echo the char  (\n is special)

        // Some PC apps/keyboards send a backspace code of 0x7F (putty)
     if (in_char == '\b' || in_char == 0x7F)
        {                  // handle BACKSPACE character - erase previous char
          if (_g_uart_buf_idx > 0)
             { _g_uart_buf_idx--;     // "erase" the character by stepping back
               read_buf [_g_uart_buf_idx] = '\0';
             }
          continue;        // cmd not finished cmd yet, keep looking for more
        }

     if (in_char == '\r')
        {                  // handle CARRIAGE RETURN char - treat as end of line
             //----------------------------------------------------------------
             // Check if \r is the only input char. in such a case, it is
             // telling the user that he wanted to enter a null line  (CR only).
             // For cross-platform usage (Windows vs Unix), we always pass
             // back a \n\0 sequence to denote a NULL line from user.
             //----------------------------------------------------------------
          if (_g_uart_buf_idx == 0)
             read_buf [_g_uart_buf_idx++] = '\n';   // tell user is a NULL line
          read_buf [_g_uart_buf_idx] = '\0'; // append \0 to denote end of string
          _g_uart_buf_idx   = 0;             // reset for next pass
          _g_uart_last_char = '\r';    // save \r, to handle any \r\n sequence check
          MAP_UARTCharPut (UART0_BASE, '\n');  // send a \n in anticipation,
                                           // otherwise the terminal can overwrite
                                           // the current line if the App issues a
                                           // CONSOLE_WRITE() before we rcv the \n
          return (1);                 // let user know a command is complete
        }

     if (in_char == '\n')
        {                  // handle NEWLINE character - treat as end of line
             // Handle a DOS/Windows \r\n sequence - ignore the \n
             // in such a case.
          if (_g_uart_last_char == '\r')
             {      // ignore the \n, because we already gave end of cmd signal
                    // to the user. Avoid handling up a second "end of cmd"
               _g_uart_last_char = 0;  // clear out the \r, so we treat any
                                      // new \n as real.
               continue;              // "nothing to see here"
             }
            else MAP_UARTCharPut (UART0_BASE, '\n');  // no preceding \r, so echo \n
             //----------------------------------------------------------------
             // Check if \n is the only input char. in such a case, it is
             // telling the user that he wanted to enter a null line  (NL only)
             // It is to help App cross-platform usage (Windows vs Unix/Linux)
             //----------------------------------------------------------------
          if (_g_uart_buf_idx == 0)
             read_buf [_g_uart_buf_idx++] = '\n';   // tell user is a NULL line
          read_buf [_g_uart_buf_idx] = '\0';  // append \0 to denote end of string
          _g_uart_buf_idx = 0;              // reset for next pass
          return (1);                      // let user know a command is complete
        }

     _g_uart_last_char = in_char;    // save char, to handle any \r\n sequence check
     if (_g_uart_buf_idx < buf_max_length)
        {        // add the char to the command buffer
          read_buf [_g_uart_buf_idx] = in_char;    // save into cmd buffer
          _g_uart_buf_idx++;                       // step to next position in buf
        }
       else {    // we've filled the buffer. Hand cmd as is to caller
              read_buf [_g_uart_buf_idx - 1] = '\0'; // append \0 to denote end of string
              _g_uart_buf_idx = 0;          // reset for next pass
              return (1);                  // let user know a command is complete
            }
   }

  return (1);                      // denote string input is is complete
}
#endif


//******************************************************************************
//  board_uart_write_char                              aka   CONSOLE_WRITE_CHAR
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//******************************************************************************

#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)

void  board_uart_write_char (char outchar)
{
    MAP_UARTCharPut (UART0_BASE, outchar);       // send a char
}
#endif


//******************************************************************************
//  board_uart_write_string                               aka   CONSOLE_WRITE
//                                                        or    DEBUG_LOG
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//******************************************************************************

#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)

void  board_uart_write_string (char *outstr)
{
    while (*outstr != '\0')
      {
        MAP_UARTCharPut (UART0_BASE, *outstr);   // send a char

        outstr++;                                // step to next char in buffer
      }
}
#endif



#if defined(USES_UNIQUEID) || defined(USES_MQTT)

//*****************************************************************************
//*****************************************************************************
//                           UNIQUE-ID  /  CRC   Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_unique_crcid_init
//
//             Use the CRC engine to generate a unique ID.
//             The optional seed and flags can be used to help generate the id.
//
//             For MQTT, we use this to create a unique publisher id so we
//             can uniquely identify ourselves to the MQTT Broker.
//******************************************************************************

void  board_unique_crcid_init (unsigned long seed, int flags)
{
    unsigned int  CRC_Init = 0xFFFF;

            // Tiva: generate random seed from CRC unit
    SysCtlPeripheralEnable (SYSCTL_PERIPH_CCM0);     // turn on clocks for CRC module
    SysCtlDelay (4000);                              // delay for 1/2 millisec
    SysCtlPeripheralReset (SYSCTL_PERIPH_CCM0);      // ensure CRC module gets reset
    SysCtlDelay (4000);

    _g_crcid_Result = 0;                     // clear the end result

// Debug this undocumented black hole later !!!
#if PROBLEMCHILD
    CRCConfigSet (CCM0_BASE, CRC_CFG_SIZE_8BIT | CRC_CFG_INIT_SEED);
    CRCSeedSet (CCM0_BASE, 12345678);                // set initial seed as 12345678
#endif

/// CRCINIRES = CRC_Init;                   // Init CRC engine with 0xFFFF
}


//******************************************************************************
//  board_unique_crcid_init
//
//             Taking an input buffer (usually the LAN chip's MAC address,
//             compute a unique ID from it, and return the result as a 4 byte
//             long.
//
//             The flags_32_8 dictate if the incoming data is 32-bit words,
//
//             For MQTT, we use this to create a unique publisher id so we
//             can uniquely identify ourselves to the MQTT Broker.
//******************************************************************************

unsigned long  board_unique_crcid_compute (void *in_buf, int in_buf_length,
                                           int flags_32_8)
{
    unsigned char  *in_buf8;
    unsigned long  *in_buf32;
    unsigned int   crc_input,   i;
    uint32_t       crcResult;

    if (flags_32_8 == 8)
       in_buf8 = (unsigned char*) in_buf;
       else in_buf32 = (unsigned long*) in_buf;

    if (flags_32_8 == 8)
       {       // we need to cast the 8-bit data intio an "int", which is what
               // the CRC engine requires for input
         for (i=0; i < in_buf_length; i++)
           {
                // Feed the user's "random values" into CRC Hardware
             crc_input = (unsigned int) *in_buf8;
//           CRCDIRB = crc_input;                 // Input data into CRC
             in_buf8++;                           // step to next entry
           }
       }
      else {    // co-erce/truncate 4 byte word values into "int"
             for (i=0; i < in_buf_length; i++)
               {
                    // Feed the user's "random values" into CRC Hardware
                 crc_input = (unsigned int) *in_buf32;
//               CRCDIRB = crc_input;             // Input data into CRC
                 in_buf32++;                      // step to next entry
               }
           }

// Debug this undocumented black hole later !!!
#if PROBLEMCHILD
    CRCConfigSet (CCM0_BASE, CRC_CFG_SIZE_8BIT | CRC_CFG_INIT_SEED);
    CRCSeedSet (CCM0_BASE, 12345678);                // set initial seed as 12345678

    CRCConfigSet (CCM0_BASE, CRC_CFG_SIZE_8BIT);     // process as 8 bits
    CRCDataWrite (CCM0_BASE, 2);
    CRCDataWrite (CCM0_BASE, 3);
    CRCDataWrite (CCM0_BASE, 4);
    for (i = 0; i < 6; i++)
        CRCDataWrite (CCM0_BASE, macAddressVal[i]);  // process as 8 bits
    crcResult = CRCResultRead (CCM0_BASE, 1);        // get post processed result
#endif

    _g_crcid_Result = crcResult;                     // all done, Save results

    return (_g_crcid_Result);
}

#endif                        // USES_UNIQUEID  ||  USES_MQTT


#if defined(USES_VTIMER)
//*****************************************************************************
//*****************************************************************************
//                             VIRTUAL  TIMER   Routines
//*****************************************************************************
//*****************************************************************************

//    Virtual Timers are based upon Systick's 1 ms timer.
//    They provide a "logical timer" capability, without typing up a real timer.
//    On smaller MCUs (e.g. MSP430), this is particularly useful, since the
//    number of real timers is often very limited.
//    It is also similar to the TON/TOFF logical timers used in PLCs.
//    We support 10 VTIMERs, labelled VTIMER_0 through VTIMER_9
#define  VTIMER_RESET      0     /* VTIMER is not in use */
#define  VTIMER_BUSY       1     /* VTIMER is active, but not yet reached value*/
#define  VTIMER_COMPLETED  2     /* VTIMER has reached it requested value */

    char            _g_vtimer_flags [10]      = { 0,0,0,0,0,0,0,0,0,0 };
    char            _g_vtimer_user_flags [10] = { 0,0,0,0,0,0,0,0,0,0 };
    uint32_t        _g_vtimer_duration [10];
    uint32_t        _g_vtimer_expire   [10];
    P_EVENT_HANDLER _g_vtimer_callback [10]   = { 0,0,0,0,0,0,0,0,0,0 };
    void            *_g_vtimer_callback_parm [10];

//*****************************************************************************
//  board_vtimer_check_expiration
//
//               Internally called routine (from SYSTICK ISR) to update timers
//               and flag any that have been completed.
//
//      CAUTION:  this is TIME CRITICAL CODE that was called from SYSTICK ISR
//*****************************************************************************

static  void  board_vtimer_check_expiration (void)
{
    int   i,  expired;

          //-------------------------------------------------------------
          // See if any virtual timers have reached their expiration time
          //-------------------------------------------------------------
    for (i = 0; i < 10;  i++)
      {
        if (_g_vtimer_flags[i] != VTIMER_BUSY)
           continue;                       // timer is not active, skip it

        if (_g_vtimer_expire [i] > _g_systick_millisecs)
           continue;                       // still not reach the timeout value
// ??? TBD - CHECK TIMER, MUST ALSO HANDLE 49-day wrap-around ??? !!!

                 //----------------------------------------------------------
                 // The VTIMER has reached its expiration time, so set
                 // the COMPLETED flag, and invoke any associated callback.
                 //----------------------------------------------------------
             _g_vtimer_user_flags[i] = VTIMER_COMPLETED;  // update user state

             if (_g_vtimer_callback[i] != 0L)
                {      //  invoke user supplied callback, passing parm
                  (_g_vtimer_callback[i]) (_g_vtimer_callback_parm [i]);
                }

                   // Then setup next timeout deadline
                   // This keeps the VTIMER keep firing until user issue vtimer_reset() to cancel it
             _g_vtimer_expire [i] += _g_vtimer_duration[i];
           }
}


//*****************************************************************************
//  board_vtimer_start
//
//              Start a VTIMER, with a given milli-second timeout value.
//              Optionally, a callback and parameter can be provided.
//
//              Timer duration must be specified in milliseconds.
//              It has a maximum limit of 1,000,000,000  (277 hours or 11 days)
//*****************************************************************************

int  board_vtimer_start (int vtimer_id, uint32_t timer_duration_millis,
                         P_EVENT_HANDLER callback_function,  void *callback_parm)
{
     if (vtimer_id < 0 || vtimer_id > 9)
        return (ERR_VTIMER_ID_OUT_OF_RANGE);
     if (_g_vtimer_flags[vtimer_id] == VTIMER_BUSY)
        return (ERR_VTIMER_IN_USE);
     if (timer_duration_millis > 1000000000)
        return (ERR_VTIMER_MILLISEC_EXCEED_LIMIT);

     _g_vtimer_flags[vtimer_id] = VTIMER_BUSY;       // internal state
     _g_vtimer_user_flags [vtimer_id] = VTIMER_BUSY; // user view of state

          // Add VTIMER millis to current SYSTICK value. If it
          // wraps around, we depend upon the VTIMER expiration logic to handle it.
     _g_vtimer_expire[vtimer_id] = _g_systick_millisecs + timer_duration_millis;

          // save the VTIMER duration, so we can step expiration to next
          // interval, after each VTIMER "pop"
     _g_vtimer_duration[vtimer_id] = timer_duration_millis;

          // save any callback information
     _g_vtimer_callback [vtimer_id]     = callback_function;
     _g_vtimer_callback_parm [vtimer_id] = callback_parm;

          // increment the number of active VTIMERs
    _g_vtimers_active++;

    return (0);               // denote completed successfully
}


//*****************************************************************************
//  board_vtimer_completed
//
//              Check if a VTIMER that is busy or completed.
//              0 = Busy,   1 = completed,  -1 = error
//*****************************************************************************

int  board_vtimer_completed (int vtimer_id)
{
     if (vtimer_id < 0 || vtimer_id > 9)
        return (ERR_VTIMER_ID_OUT_OF_RANGE);

     if (_g_vtimer_user_flags[vtimer_id] == VTIMER_BUSY)
        return (0);                       // not completed, still busy

     if (_g_vtimer_user_flags[vtimer_id] == VTIMER_COMPLETED)
        {     // we told the user, so clear it's state so he knows when it pops next
          _g_vtimer_user_flags [vtimer_id] = VTIMER_BUSY;  // reset user flag
          return (1);                     // has completed
        }

     return (-1);                         // else is reset, and was not active
}


//*****************************************************************************
//  board_vtimer_reset
//
//              Stop and reset a VTIMER that is busy or completed.
//*****************************************************************************

int  board_vtimer_reset (int vtimer_id)
{
     if (vtimer_id < 0 || vtimer_id > 9)
        return (ERR_VTIMER_ID_OUT_OF_RANGE);

     _g_vtimer_flags[vtimer_id] = VTIMER_RESET;
     _g_vtimer_user_flags [vtimer_id] = VTIMER_RESET;

    return (0);               // denote completed successfully
}
#endif



//*****************************************************************************
//*****************************************************************************
//                               TIMER   Routines
//*****************************************************************************
//*****************************************************************************

//--------------------------------------------------------------------
//
//                        TIMER   and   PWM   SUPPORT
//
// The DRV8711 can be used to drive either 1 bi-polar stepper motor,
// or up to two Brushed DC (BDC) motors.
//
// When supporting a Stepper motor, the timer is used to time motor
// movements, especially for accelerating or decelerating, as part of
// the "speed profile" ramp up, run, then ramp down sequencing.
// So in that mode, the timer is configured as a pure timer, and the
// CCRs (timer compare registers) are set to the proper durations
// of a programmed speed timing sequence.
//
// When supporting BDC motors, the timer is used to generate PWM signals
// to drive the motors. The DRV8711 is re-configured for BDC mode, and
// is used to act as a gate driver for the PWM signals to the FETs
// that drive the BDC motors. In that mode, the timer is configured as
// a PWM generator, and the CCRs are set to the "duty cycle" for the
// PWMs.
//
// During testing, the user can dynamically change the motor
// configuration, from Stepper to BDC or vice versa. When that occurs,
// the Timer needs to be stopped and re-configured for the new Mode.
//
// MSP430 Timer1 Notes:
//  - TACCR0 is the master period register
//  - TAR    is the incrementing/decrementing timer counter
//  - TACCR1 is Compare value 1  (with separate interrupt)
//  - TACCR2 is compare value 2
//  - Counter Modes:  1 = count up to TACCR0      2 = count up to 0xFFFF
//                    3 = count up/down to TACCR0
//  - Output Modes:   0 = force timer out to immediate value  (0 or 1)
//                    3 = (PWM) set/reset mode: low while < TACCR1,
//                       high while > TACCR1,  reset low when hit TACCR0
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// interval_timer_init
//
//            Initialize the periodic Interval Timer.
//            This pops every 32 milliseconds to update ramp up/down.
//---------------------------------------------------------------------
void  interval_timer_init (void)
{

        // The Timer0 peripheral must be enabled for use.
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER0);

        // The Timer0 peripheral must be enabled for use.
    MAP_TimerConfigure (TIMER0_BASE, 
                        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

        // Set the Timer0A to run with a 32 ms period
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A,  _g_SysClk_Ticks / 32000);

        // Configure the Timer0A interrupt for timer timeout.
    MAP_TimerIntEnable (TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        // Enable the Timer0A interrupt on the processor (NVIC).
    MAP_IntEnable (INT_TIMER0A);

           // Enable Timer0A
    MAP_TimerEnable (TIMER0_BASE, TIMER_A);
}


//--------------------------------------------------------------------
// interval_timer_disable
//
//            Shut off the Interval Timer
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_disable (void)
{
//  WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog Timer
//  IE1   &= ~WDTIE;            // Disable Watchdog Interrupts

           // Disable Timer0A
    MAP_TimerDisable (TIMER0_BASE, TIMER_A);

}


//--------------------------------------------------------------------
// interval_timer_enable
//
//            Turn on the Interval Timer, used for Accel/Decel Updates
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_enable (void)
{
//  WDTCTL = WDT_MDLY_32;   // setup Watch dog time duration
//  IE1   |= WDTIE;         // Enable Watch dog interrupt

           // Enable Timer0A
    MAP_TimerEnable (TIMER0_BASE, TIMER_A);
}



//-------------------------------------------------------------------
//  timer_config_pwm_mode
//
//         Turn on PWM mode, output goes to associated STEP1_AIN pin.
//         Used when want to drive BDC motors with PWM.       VERIFY
//
//         flags parm is for future use.
//
//  Tiva pin PA6 uses M1PWM2, which is Module 1 Output 2.
//  i.e. it is controlled by           Module 1 PWM, Generator 1.
//-------------------------------------------------------------------

void  timer_config_pwm_mode (long flags)
{
       // re-setup STEP_AIN1 pin for PWM mode in case it got clobbered
    MAP_GPIOPinTypePWM (GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinConfigure (GPIO_PA6_M1PWM2);

       // When timer_disable_pwm() was called,  We just
       // disabled the PWM output pin and stopped the PWM module/timer,
       // so there is no need to completely re-configure the entire PWM

       // Use whatever PWM Period/Duty was last set ???
       // or ensure some minimum value, else Output stays stuck at HIGH !!!
}


//------------------------------------------------------------------
//  timer_config_period_duty_cycle
//
//         Set the PWM Period, and starting duty cycle.
//------------------------------------------------------------------

void  timer_config_period_duty_cycle (long period, long duty_cycle)
{


// ??? RE_THINK THIS - DO PERIOD CALCS IN HERE, NOT IN utility.c ???

            // set PWM period and starting duty cycle into PWM module
    MAP_PWMGenPeriodSet (PWM1_BASE, PWM_GEN_1, period);
    MAP_PWMPulseWidthSet (PWM1_BASE, PWM_OUT_2, duty_cycle);

            // Save copy for debug
    _g_PWMperiod = period;
    _g_PWMduty   = duty_cycle;
}


//-------------------------------------------------------------------
// timer_disable_pwm
//
//            Disable the Speed or PWM timer.
//            (in preparation for Stopping or Reconfiguring the motor)
//--------------------------------------------------------------------
void  timer_disable_pwm (void)
{
       // Disable PWM Interrupts
    MAP_PWMGenIntTrigDisable (PWM1_BASE, PWM_GEN_1,
                              PWM_INT_CNT_LOAD | PWM_INT_CNT_AD);

    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false); //Disable PWM pin output

    MAP_PWMGenDisable (PWM1_BASE, PWM_GEN_1); // turn off the PWM ?? need
}


//-------------------------------------------------------------------
// timer_enable_pwm
//
//            Enable and Start the timer in Speed or PWM mode.
//--------------------------------------------------------------------
void  timer_enable_pwm (void)
{
       // Re-enable PWM Interrupts
    MAP_PWMGenIntTrigEnable (PWM1_BASE, PWM_GEN_1,
                             PWM_INT_CNT_LOAD | PWM_INT_CNT_AD);

    MAP_PWMGenEnable (PWM1_BASE, PWM_GEN_1);  // start up the PWM ??? need

    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, true); // Enable PWM pin output
}



//---------------------------------------------------------------------
//  registerInterruptHandler
//
//         register an interrupt handler for the host IRQ line  (CC3100, ...)
//         Is called by TI's SimpleLink sl_start() logic, at startup.
//---------------------------------------------------------------------

int  registerInterruptHandler (P_EVENT_HANDLER InterruptHdl, void *pValue)
{
    pIrqEventHandler = InterruptHdl;

    return 0;
}


#if defined(USES_CC3100)
//**************************************************************************
//**************************************************************************
//
//                   BOARD   -   CC3100  Specific   Routines
//
//**************************************************************************
//**************************************************************************

//-------------------------------------------------------------------
// CC3100_disable
//
//            Disables the CC3100       (nHIB/ENABLE line)
//
//            Called by TI's SimpleLink code when using CC3100.
//--------------------------------------------------------------------
void  CC3100_disable()
{
    MAP_GPIOPinWrite (GPIO_PORTE_BASE, GPIO_PIN_4, PIN_LOW);
}


//-------------------------------------------------------------------
// CC3100_enable
//
//            Enables the CC3100        (nHIB/ENABLE line)
//
//            Called by: sl_Start() in simplelink device.c 
//
// This KEY routine is called by the simplelink sl_Start() routine,
// after it has setup the IRQ (PORTE) interrupt handler's address.
//
// Enabling the CC3100 kicks off its internal "startup" logic.
// It takes about 50 milliseconds to complete.
// When the CC3100 startup completes, it will toggle an IRQ interrupt.
// The (PORTE) IRQ handler will then read in the CC3100 status and
// signal "startup complete" to the sl_Start() routine.
//
// Calling this routine too early (before IRQ handler address is setup)
// will cause severe interrupt looping, continously calling the ISR.
//
// Failing to have the MCU PortE interrupt enabled (and general MCU/NVIC)
// interrupts enabled) will cause the sl_Start() logic to loop forever,
// waiting for "startup complete".
//
// Note that the CC3100 startup logic is _extremely_ timing sensitive,
// such that even tweaking the MCU clock rate can cause "startup hangs"
// because the IRQ interrupt gets missed or not manifested.
//--------------------------------------------------------------------
void  CC3100_enable()
{
    MAP_GPIOPinWrite (GPIO_PORTE_BASE, GPIO_PIN_4, PIN_HIGH);
}


//-------------------------------------------------------------------
// CC3100_InterruptEnable
//
//            Enables the interrupt (IRQ) from the CC3100 on P 1.2
//
//            Called by TI's SimpleLink code when using CC3100.
//--------------------------------------------------------------------
void  CC3100_InterruptEnable()
{
    MAP_GPIOIntEnable (GPIO_PORTB_BASE, GPIO_PIN_2);
#ifdef SL_IF_TYPE_UART
    MAP_UARTIntEnable (UART1_BASE, UART_INT_RX);
#endif
}


//-------------------------------------------------------------------
// CC3100_InterruptDisable
//
//            Disables the interrupt (IRQ) from the CC3100 on P 1.2
//
//            Called by TI's SimpleLink code when using CC3100.
//--------------------------------------------------------------------
void  CC3100_InterruptDisable()
{
     MAP_GPIOIntDisable (GPIO_PORTB_BASE, GPIO_PIN_2);
#ifdef SL_IF_TYPE_UART
     MAP_UARTIntDisable (UART1_BASE, UART_INT_RX);
#endif
}


//-------------------------------------------------------------------
// MaskIntHdlr
//
//            Denotes the Host IRQ is Masked
//--------------------------------------------------------------------
void  MaskIntHdlr()
{
	IntIsMasked = TRUE;
}


//-------------------------------------------------------------------
// MaskIntHdlr
//
//            Denotes the Host IRQ is Unmasked
//--------------------------------------------------------------------
void  UnMaskIntHdlr()
{
	IntIsMasked = FALSE;
}


//-------------------------------------------------------------------
//                      CC3100   IRQ   ISR   Handler
//-------------------------------------------------------------------

void  GPIOB_intHandler (void)
{
    unsigned long  intStatus;

    intStatus = MAP_GPIOIntStatus (GPIO_PORTB_BASE, 0);  // get interrupt cause
    MAP_GPIOIntClear (GPIO_PORTB_BASE, intStatus);       // clear interrupt flag

    if (intStatus & GPIO_PIN_2)
       {
#ifndef SL_IF_TYPE_UART
    	if (pIrqEventHandler)
           {
             pIrqEventHandler (0);    // call CC3100 handler - WVD this is called on every rupt
           }
#endif
       }
}

#endif                            //  #if defined(USES_CC3100)


#if defined(USES_DRV8711)
//**************************************************************************
//**************************************************************************
//
//                 BOARD   -   DRV8711  Specific   Routines    Stepper
//
//**************************************************************************
//**************************************************************************

//---------------------------------------------------------------------
// adc_init
//
//            Initialize the ADC connected to the Potentiometer.
//---------------------------------------------------------------------
void  adc_init (void)
{
       // Setup ADC pin for Potentiometer
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC0);   // Turn on ADC clock
    MAP_GPIOPinTypeADC (GPIO_PORTB_BASE, GPIO_PIN_5);  // PB5 / AIN11

       // Enable sample sequence 3 with a processor signal trigger.
       // Sequence 3 will do a single sample when the processor
       // sends a signal to start the conversion.
    MAP_ADCSequenceConfigure (ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

// ??? how does PB5 / AIN11 get mapped to sequence 3 sample channel ???
// For this example ADC0 was used with AIN0 on port E7.
       // Configure step 0 on sequence 3. Sample channel 0 (ADC_CTL_CH0)
       // in single-ended mode (default) and configure the interrupt flag
       // (ADC_CTL_IE) to be set when the sample is done.
       // Tell the ADC logic  that this is the last conversion on
       // sequence 3 (ADC_CTL_END). Seq 3 has only one programmable step.
    MAP_ADCSequenceStepConfigure (ADC0_BASE, 3, 0,
                              ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
       // And then enable Sequence 3.
    MAP_ADCSequenceEnable (ADC0_BASE, 3);

       // Clear the interrupt status flag. This is done to make sure the
       // interrupt flag is cleared before we sample.
    MAP_ADCIntClear (ADC0_BASE, 3);

    MAP_ADCProcessorTrigger (ADC0_BASE, 3);  // Kick off first ADC conversion
}


//---------------------------------------------------------------------
// adc_start_sample
//
//            Initiate a sample on the Potentiometer ADC
//---------------------------------------------------------------------
void  adc_start_sample (void)
{
    MAP_ADCProcessorTrigger (ADC0_BASE, 3);  // Trigger the ADC conversion
}


//---------------------------------------------------------------------
// adc_check_if_completed
//
//            Tests if the ADC sample conversion is complete.
//            Returns 1 if completed, or 0 if still busy
//---------------------------------------------------------------------
int  adc_check_if_completed (void)
{
    if ( ! MAP_ADCIntStatus(ADC0_BASE, 3, false))
       return (0);                   // Denote ADC is still busy

    MAP_ADCSequenceDataGet (ADC0_BASE, 3, &_g_ADC_value); //read converted value

    MAP_ADCIntClear (ADC0_BASE, 3);      // clear interrupt flag for next pass

    return (1);                      // Denote ADC has completed
}


//---------------------------------------------------------------------
// adc_read_result
//
//           Read the data from the latest Potentiometer ADC conversion
//---------------------------------------------------------------------
uint16_t  adc_read_result (void)
{
    return (_g_ADC_value);
}


//--------------------------------------------------------------------
//  gpio_force_all_control_pins_low                   PA3-PA4,PA6-PA7
//
//        Force all DRV8711 pins LOW.
//--------------------------------------------------------------------
void  gpio_force_all_control_pins_low (void)
{
        //Disable PWM pin output. This also drops the output low.
    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false);
/// HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) &= 0xF0FFFFFF;    // force to GPIO
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1

    GPIO_PORTA_DATA_R &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
}


//-------------------------------------------------------------------
//  gpio_quiesce_all_control_pins                    PA3-PA4,PA6-PA7
//
//        Configure Pins for IN/IN Control and Set All Outputs Low.
//        Force all DRV8711 pins back to GPIO mode, and set them LOW.
//-------------------------------------------------------------------
void  gpio_quiesce_all_control_pins (void)
{
        //Disable PWM pin output. This also drops the output low.
    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false);
/// HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) &= 0xF0FFFFFF;  // force back GPIO
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1

        // The following are overkill ?   See MSP430
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_7); // PA7 DIR/A2
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_4); // PA4 BIN1
/// MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_3); // PA3 BIN2

    GPIO_PORTA_DATA_R &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
}


//--------------------------------------------------------------------
// timer_force_pwm_pin_low
//
//            Forcibly turn off PWM mode, and force output pin low
//
//            pin_num parm is for future use.
//--------------------------------------------------------------------
void  timer_force_pwm_pin_low (short pin_num)
{
    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false); //Disable PWM pin output

//          TA1CCTL1 = OUTMOD_0;       // Turn off Timer PWM mode
//          P2OUT   &= ~STEP_AIN1;     // force timer output pin LOW

    HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) &= 0xF0FFFFFF;
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1
    GPIO_PORTA_DATA_R &= ~(STEP_AIN1);        // force timer output pin LOW
}


//------------------------------------------------------------------
//  timer_set_pin_for_pwm
//
//         Setup GPIO pin and associated "Alternate Function" bits
//         to run the pin as a PWM output from the Timer
//------------------------------------------------------------------

void  timer_set_pin_for_pwm (short pin_num)
{
       // setup STEP_AIN1 pin for PWM mode Output
    MAP_GPIOPinTypePWM (GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinConfigure (GPIO_PA6_M1PWM2);
}


//------------------------------------------------------------------
//  timer_set_pin_for_gpio
//
//         Setup GPIO pin and associated "Alternate Function" bits
//         to run the pin as a standard GPIO pin   (turn off PWM)
//------------------------------------------------------------------

void  timer_set_pin_for_gpio (short pin_num)
{
       // Disable PWM pin output
    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, false);

       // Re-configure pin back to operate as normal GPIO
    HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) &= 0xF0FFFFFF;
    MAP_GPIOPinTypeGPIOOutput (GPIO_PORTA_BASE, GPIO_PIN_6); // PA6 STEP/A1
    GPIO_PORTA_DATA_R &= ~(STEP_AIN1);                    // and set it LOW
}



//------------------------------------------------------------------
//                ISR  Handler    PWM
//
// The PWM interrupt pops for two separate events:
//      - CCR match/completion: This is used to increment the
//        Step Counter, to count the number of steps (PWM pulses)
//        that have been issued to the Stepper motor.
//
//      - PWM end of period: This is used to roll in the new
//        Period and Duty cycle to be used for the next cycle.
//        In general, the period remains the same, and
//        only the duty cycle changes, in reponse to ramp up
//        or ramp down situations.
//        At constant speeds, the duty cycle is 50 %.
//
// Note:
//   Unlike the MSP430 Timers which have separate interrupt vectors
//   (CCR vs Timer End), Tiva PWMs have only 1 interrupt but return
//   separate flags for each event in the PWM_O_X_ISC reg
//------------------------------------------------------------------

    volatile long   rupt_load_count = 0;     // Is now working

void  ISR_PWM_Generator (void)
{
    uint32_t  ui32Gen;
    uint32_t  ui32GenBase;

    _g_PWM_RuptFlags = MAP_PWMGenIntStatus (PWM1_BASE, PWM_GEN_1, true);

    if (_g_PWM_RuptFlags & PWM_INT_CNT_AD)
       {
         G_CUR_NUM_STEPS++;        // Increment Step (pulse) Counter
       }

       // Clear PWM Interrupt flags
    MAP_PWMGenIntClear (PWM1_BASE, PWM_GEN_1, _g_PWM_RuptFlags);

    if (_g_PWM_RuptFlags & PWM_INT_CNT_LOAD)       // never hitting this, instead
                                                  // flag 0x0002 is seen
       {    // If requested, update Timer PWM period and duty cycle
            // at End of a PWM Period
         rupt_load_count++;   // DEBUG
         if (G_LOAD_CCR_VALS == true)       // Did mainline request an update ?
            {
              G_CUR_SPEED = G_CUR_SPEED_TEMP;   // yes, update period/duty

                  // Compute Generator address within a given PWM Module
//            ui32Gen = PWM_GEN_BADDR (PWM1_BASE, PWM_GEN_1);
//            HWREG(ui32Gen + PWM_O_X_LOAD) = G_TA1CCR0_TEMP; // set (new) Period

                  // Compute Duty Cycle address within a given PWM Module
//            ui32GenBase = PWM_OUT_BADDR (PWM1_BASE, PWM_OUT_2);
//            HWREG(ui32GenBase + PWM_O_X_CMPA) = G_TA1CCR1_TEMP;// set new Duty Cycle

              MAP_PWMGenPeriodSet (PWM1_BASE, PWM_GEN_1,
                                   G_TA1CCR0_TEMP);       // set (new) Period
              MAP_PWMPulseWidthSet (PWM1_BASE, PWM_OUT_2,
                                    G_TA1CCR1_TEMP);      // set new Duty Cycle

                  // Save copy for debug
              _g_PWMperiod = G_TA1CCR0_TEMP;
              _g_PWMduty   = G_TA1CCR1_TEMP;

              G_LOAD_CCR_VALS = false;
            }
       }
}


//--------------------------------------------------------------------
//                ISR  Handler    Interval  Timer
//
// This pops every 32 milli-seconds, and is used to process
// the acceleration/deceleration ramp.
// It sets the flag for the mainline to compute the next speed
// step, ir a ramp up or ramp down is occurring. If just
// a constant speed is being performed, the speed stays as is.
//
// This also captures and saves the last ADC reading from the
// Potentiometer, and initiates a new ADC conversion sequence for it.
//--------------------------------------------------------------------
void  ISR_Interval_Timer (void)
{
    int  arc;

       //---------------------------------------------------------
       // Signal Main Thread to Calculate Next (Ramp) Speed Value
       //---------------------------------------------------------
    G_ACCEL_FLAG = true;

       // Clear Timer Interrupt flag
    MAP_TimerIntClear (TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    arc = adc_check_if_completed();  // Should normally already be completed
    while (arc == 0)
      arc = adc_check_if_completed();
    adc_start_sample();              // kick off a new ADC conversion
}

#endif                            //  #if defined(USES_DRV8711)


//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   ISRs
//
//**************************************************************************
//**************************************************************************

//---------------------------------------------------------------------
//  UART1_intHandler
//---------------------------------------------------------------------

void  UART1_intHandler()
{
	unsigned long   intStatus;

	intStatus = UARTIntStatus (UART1_BASE, 0);
	UARTIntClear (UART1_BASE, intStatus);

#ifdef SL_IF_TYPE_UART
	if ((pIrqEventHandler != 0) && (IntIsMasked == FALSE))
	   {
		 pIrqEventHandler (0);
	   }
#endif
}
