// 07/12/15 - ROLLOVER interrupts are workingg (TIMER_XR)
//          - CCR interrupts (TIMER_OC) are broken.

// 07/11/15 - Timer ISR interrupts are NOT working

// 07/11/15 - NEED TO ADD PRE-SCALARS TO ALL PERIOD/DUTY_CYCLES.
//            THINGS ARE OFF BECUASE PERIOD VALUES ARE EXCEEDING 16 BITS

// 05/03/15 - PWM works, but is "jittery" and runs an 19.8 Khz because
//            8 MHz clock causes very small CCR values (0x00E1) leading to
//            loss of accuracy.  Suggest using a /2 or /4 pre-scalar. Ditto F5529

// 04/17/15 Status:  MCLK at 8 MHz:  TCP_Client, TCP_Server, MQTT_Client,
//                   and Console are all running successfully !  YAH !
//                   BUT - still need to add MSP430 CRC support for MQTT.

// 04/13/15 STATUS   ==> TIMING WINDOW
//   - when run the older TI_WVD_CC3x00_MODBUS_Workspace_CCSv6 version of
//     the software, it works fine.
//   - when run this version, it hangs forever waiting for the IRQ interrupt
//     signalling "startup complete".  (Running at 16 MHz)
//         ==> faster clock rate in this version is creating a timing window
//         ==> 50 ms delay is taking too long, mising the rising edge of
//             the interrupt.  (P1IE2 = 1,  P1IFG2 = 0, P1IN bit 2 = 0)
//         ==> if I manually re-toggle the enable line P4.3 from 1 to 0 to 1
//             things move forward, but then hang later in read status rtn.
//   - Note that even TI's sample app wlan_station fails !
//
//  ==> Temporarily change clocks block to prior version and see if things chg
//                 -- 04/13/15 10:09 am update --
//          MCU CLOCKS WAS THE PROBLEM !!!
//          AS SOON AS I WENT BACK TO THE OLD METHOD using 8 MHz, IT WORKED.
//          So much for using DriverLib !
//            ==> CC3100 startup is extremely timing dependent !
//
//  ==> Verify optimization logic is set 1 to (Register optimize only)

//*******1*********2*********3*********4*********5*********6*********7**********
//                           board_MSP430_FR6989.c
//
// Board dependent code, moved to a single module for MSP430 FR6989 Launchpad
//
// CC3100 Usage:  You _MUST_ us both the MCU USB _AND_ the separate CC3100 power
//                (USB) connector. The FR5669 Launchpad does not provide quite
//                enough juice to fully power the CC3100. If you only plug in
//                the Launchpad USB cable, the CC3100 will hang due to 
//                insufficient power.
//                
// MSP430 Notes:
//   SMCLK_MHz (used to drive Timer PWM) is set to  2 MHz
//   MCLK_MHz  (used to drive main CPU)  is set to 16 MHz
//   WDT       (used as interval timer)  is WDT_MDLY_32        = 32 ms delay/pop
//   Single Step pulse width             is delay_cycles(1000) = 0.0000625 sec
//                                                             = 62.5  usec
//                                         (delay_cycles = value * MCLK cycles)
//
//   PWM frequency range:                SMCLK / TARGET_SPEED_in_PULSES_per_SEC
//       Target PPS = 512-1024           Period = 3906 - 1953
//
// History:
//   05/26/15 - Board arrived. Created, based on its cousin FR5969.  Duqu 
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

#include "boarddef.h"
#include "user_api.h"

#include "msp430/spi.h"

#if defined(USES_CC3100)
#include "simplelink.h"        // pull in defs for CC3100 SimpleLink
    unsigned long  num_CC3100_irq_rupts  = 0;        // DEBUG COUNTER
#endif

#if defined(USES_DRV8711)
#include "devices/DRV8711_Spin_Routines.h"
#endif

int  board_timerpwm_lookup (int module_id, int channel_id, uint16_t *timerbase,
                            uint16_t *timercctl, uint16_t *timerccr);


//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    P_EVENT_HANDLER   pIraEventHandler = 0;
    uint8_t           IntIsMasked;

    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_MCLK          = 0; // CPU MCLK setting
    uint32_t  _g_SMCLK         = 0; // peripheral SMCLK setting
    uint32_t  _g_TA1_1ms_ticks = 0; // number of SMCLK ticks in 1 millisecond

    uint32_t  _g_systick_millisecs = 0;   // used by "Systick" emulator logic.
                                          // provides a 1 ms periodic timer pop

    char      _g_vtimers_active = 0;      // Optional VTIMER support


                 // mainly for DRV8711 logic
    uint32_t  _g_ADC_value = 0;

#if (USES_ADC)
    ADC12_B_configureMemoryParam  _g_ADC_param = {0};
#endif

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ)
    char      _g_uart_buf_idx   = 0;    // keeps track of user input via UART
    char      _g_uart_last_char = 0;
#endif

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    uint32_t  _g_crcid_Result;
#endif


               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
const uint32_t  _g_gpio_port_base_address [] =
            {   0x00,
              __MSP430_BASEADDRESS_PORT1_R__,     // Port 1
              __MSP430_BASEADDRESS_PORT2_R__,     // Port 2
              __MSP430_BASEADDRESS_PORT3_R__,     // Port 3
              __MSP430_BASEADDRESS_PORT4_R__,     // Port 4
              __MSP430_BASEADDRESS_PORT5_R__,     // Port 5
              __MSP430_BASEADDRESS_PORT6_R__,     // Port 6
              __MSP430_BASEADDRESS_PORT7_R__,     // Port 7
              __MSP430_BASEADDRESS_PORT8_R__,     // Port 8
              __MSP430_BASEADDRESS_PORT9_R__,     // Port 9
              __MSP430_BASEADDRESS_PORTJ_R__      // Port J
            };
#define  GP_PORT_1   1        // indexes into above table, based on port id
#define  GP_PORT_2   2
#define  GP_PORT_3   3
#define  GP_PORT_4   4
#define  GP_PORT_5   5
#define  GP_PORT_6   6
#define  GP_PORT_7   7
#define  GP_PORT_8   8
#define  GP_PORT_9   9
#define  GP_PORT_J  10

#define  MAX_GPIO_PORTS   GP_PORT_J


void  ClearBufferRelatedParam (void)
{
     // dummy entry - not used
}


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
       else board_system_clock_config (MCU_DEFAULT_SPEED); // use default: run at 8 MHz

    board_gpio_init();              // turn on key GPIO clocks, ...

#if defined(USES_SYSTICK) || defined(USES_MQTT) || defined(USES_ADC) || defined(USES_VTIMER)
    board_systick_timer_config();   // turn on "Systick" timer
#endif

        // Enable the UART if user wants to invoke CONSOLE or DEBUG_LOG calls
#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    board_uart_init();              // go ahead and enable the default UART
#endif

}


//*****************************************************************************
// board_busy_wait_usec
//
//           Continuously loop until a fixed time delay has completed.
//           This function provides a means of generating a constant
//           length. The function delay (in cycles) = 3 * parameter.
//*****************************************************************************

void  board_busy_wait_usec (long usec_delay)
{
   int   i;
   long  usec_cycles;

// ??? NEEDS TWEAKING ???   assumes takes 4 cycles per loop at 16 MHz
      // 16 MHz / 4 cycles per pass = 4 delay_cyles for 1 usec
   usec_cycles = usec_delay >> 2;      // divide by 4 because 4 cycles/pass
   for (i = 0; i < usec_cycles; i++)   // assume 4 cycles/pass
      __delay_cycles (4);              // requires a constant !
}


//*****************************************************************************
// board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************
void  board_delay_ms (long  ms_delay)
{
    uint32_t  tick_begin,   tick_endtime,   i;

#if defined(USES_MQTT) || defined(USES_ADC)
        // use Systick timer to do delay timing
    tick_begin   = _g_systick_millisecs;    //  get the current time

    tick_endtime = tick_begin + ms_delay;

// WVD: THIS LOGIC IS BROKEN  04/25/15
 #if (TO_FIX)
       // handle the oddball case of if the end time wraps past max long value
    if (tick_endtime < tick_begin)
       {          // handle the once in a blue moon wrap-around condition
          while (_g_systick_millisecs != tick_endtime)
            ;                // loop till we hit the wrap-around value
       }
      else {     // loop until the current time hits the requested delay time 
             while (_g_systick_millisecs < tick_endtime)     // hangs in here a LOOOOONG TIME
               ;
           }
 #else
   for (i = 0; i < ms_delay; i++)
      __delay_cycles (8000);     // requires a constant !

 #endif

#else
        // Systick not in use - do delay by spinning cycles
   for (i = 0; i < ms_delay; i++)
      __delay_cycles (8000);     // requires a constant !
#endif

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
    __disable_interrupt();     // Globally disable interrupts
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for Interval Timer,
//                                     PWM Timer, ADC, ...)
//*****************************************************************************
void  board_enable_global_interrupts (void)
{
    __enable_interrupt();      // Globally enable interrupts
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
//      If the result value would be too large, PWM/Timer code has
//      to further sub-divide it using pre-scalars
//
// Note we use SMCLK as base for determining Ticks, because that is used to 
//      drive all our MSP430 peripherals (PWM, Timer, ADC, ...)
//*****************************************************************************

long  board_frequency_to_period_ticks (long frequency)
{
    long   pticks;

        // take the current I/O clock frequency, and compute the associated
        // ticks needed to generate that frequency.(e.g. for Timers, PWMs, ...)
     pticks = _g_SMCLK / frequency;

     return (pticks);
}



#if defined(USES_ADC)

//*****************************************************************************
//*****************************************************************************
//                               ADC   Routines
//*****************************************************************************
//*****************************************************************************
       //                  ADC  Rules/Gotchas  That  must  handled
       // (1) When using Repeated mode (CONSEQ_2/CONSEQ_3), leave ENC alone.
       //     But if _NOT_ using repeated mode (e.g. CONSEQ 1/2), you _MUST_
       //     toggle ENC off/on after _EACH_ conversion (i.e. in ADC ISR).
       //     On FR6989 it can be very quick, but on MSP432 the toggle
       //     must last at least 2 ADCCLK cycle (so a bunch of instructions).
       //
       // (2) The ADC internally is toggled by the JK flip-flops on the outputs
       //     of the Timer, so even though you are not externally routing the
       //     OC pins out to GPIOs, internally you still need to toggle those
       //     internal flip-flops, so OUTMOD _MUST_ be set to some kind of
       //     MOD_3 / MOD_6 / MOD_7 toggle type operation. Otherwise, if
       //     set to MOD_0, the ADC will hang, with BUSY turned on, but will
       //     never complete the conversion, and never signal an interrupt.

    int                _g_ADC_complete    = 0;  // 1 = all ADC conversions/DMAs are complete
    char               _g_active_channels = 0;
    char               _g_step_num        = 0;
    char               _g_adc_configured  = 0;
    char               _g_trigger_source  = 0;
    char               _g_DMA_overrun     = 0;  // 1 = DMA completed a new set before the previous one was processed
    char               _g_toggle_ENC_required = 1;  // 1 = ADC12ENC bit must be toggled after every
                                                    // conversion set, per TECH REF

    int                _g_trigger_atmrpwm  = 0;     // index to correct Trigger Timer/PWM
    uint16_t           _g_trigger_auser_api_id = 0; // User API id for the trigger
    uint16_t           _g_trigger_shs_mask = 0;     // Associated SHS Mask for ADC timer select

    unsigned char      _g_adc_step_map [16];        // indexed by channel number

    unsigned short     _g_adc_conv_results [16];    // Internal buffer to hold DMA results

 ADC_CB_EVENT_HANDLER  _g_adc_callback       = 0L;
    void               *_g_adc_callback_parm = 0;

    unsigned long      dma_callback_seen = 0;       // DEBUG COUNTERs
    unsigned long      dma_rupt_seen     = 0;


#define  ADC_COMPARATOR_THRESHOLD_0      0x00
#define  ADC_COMPARATOR_THRESHOLD_1      0x80
#define  ADC_ENABLE_COMPARATOR_THRESHOLD 0x40
#define  ADC_DIFFERENTIAL_MODE           0x20
#define  ADC_VREF_AVCC_AVSS              0x00
#define  ADC_VREF_BUF_AVSS               0x01
#define  ADC_VREF_VEREFP_VEREFM          0x0E
#define  ADC_VREF_VEREFP_BUF_VEREFM      0x0F

//#define  ADC_EOS_BIT                   0x80     // denotes last channel in the sequence
#define  ADC_EOS_BIT                        1     // denotes last channel in the sequence

typedef struct seqstep_reg_def         /* ADC definition for ADC14 Sequence Step reg */
    {      // Note: because MSP432 is little endian, we need to byte reverse everything
/////   uint8_t   seq_step_channel;    /* ADC channel number and EOS bits */
/////   uint8_t   comparator_dif_bits; /* Bit masks for comparator, differential mode, VRef    */
/////   uint16_t  seqstep_reserved;

        uint32_t  seq_step_channel          :   5;  /* Input channel select */
        uint32_t  seqstep_reserved0         :   2;  /* Reserved */
        uint32_t  seq_step_eos              :   1;  /* End of sequence */
        uint32_t  seqstep_vref_select       :   4;  /* Selects combinations of V(R+) and V(R-) sources */
        uint32_t  seqstep_reserved1         :   1;  /* Reserved */
        uint32_t  seqstep_differential_mode :   1;  /* Differential mode */
        uint32_t  seqstep_cmp_thresh_enable :   1;  /* Comparator window enable */
        uint32_t  seqstep_cmp_thresh_sel    :   1;  /* Window comparator threshold register selection */
        uint32_t  seqstep_reserved          :  16;  /* Reserved */
    } ADC_SEQSTEP_BLK;


      //-----------------------------------------------------------------------
      // table of addresses of ADC sequencing result registers: ADC12MEM 0 - 13
      //-----------------------------------------------------------------------
const uint8_t  _g_adc_seqstep_regs[] =
    { ADC12_B_MEMORY_0,
      ADC12_B_MEMORY_1,
      ADC12_B_MEMORY_2,
      ADC12_B_MEMORY_3,
      ADC12_B_MEMORY_4,
      ADC12_B_MEMORY_5,
      ADC12_B_MEMORY_6,
      ADC12_B_MEMORY_7,
      ADC12_B_MEMORY_8,
      ADC12_B_MEMORY_9,
      ADC12_B_MEMORY_10,
      ADC12_B_MEMORY_11,
      ADC12_B_MEMORY_12,
      ADC12_B_MEMORY_13
    };

               //--------------------------------------------------------------
               //                 ADC   Trigger Mapping Table              SHS
               //--------------------------------------------------------------
               //-----------------------------------------------
               // Triggerable Timers/Events available on MSP432
               //   Timer/ CCR      SHS Value    (from MSP432 datasheet p 91)
               //     TA0 / CCR1        1
               //     TB0 / CCR0        2
               //     TB0 / CCR1        3
               //     TA1 / CCR1        4
               //     TA2 / CCR1        5
               //     TA3 / CCR1        6
               //------------------------------------------------
const uint32_t         _g_adctrigger_shs_mask []
                               = { ADC12SHS_0,      // SHS 0 denotes no trigger
                                   ADC12SHS_1,      // TRIGGER_TIMER_0_CC1  TA0/CCR1
                                   ADC12SHS_2,      // TRIGGER_TIMER_4_CC0  TB0/CCR0
                                   ADC12SHS_3,      // TRIGGER_TIMER_4_CC1  TB0/CCR1
                                   ADC12SHS_4,      // TRIGGER_TIMER_1_CC1  TA1/CCR1
                                   ADC12SHS_5,      // TRIGGER_TIMER_2_CC1  TA2/CCR1
                                   ADC12SHS_6,      // TRIGGER_TIMER_3_CC1  TA3/CCR1
                                   ADC12SHS_7       //  - none -
                                 };


               //--------------------------------------------------------------
               //                 ADC    GPIO  PIN     Mapping Table
               //--------------------------------------------------------------

typedef struct adc_channel_def        /* ADC Channel definitions */
    {
        uint32_t  chan_gpio_port_id;  /* Associated GPIO port    */
        uint32_t  chan_gpio_pin_bit;  /* Associated GPIO pin     */
        uint8_t   chan_adc_num;       /* Associated ADC12_B_INPUT_x channel num */
        uint8_t   chan_mem_num;       /* Associated ADC12_B_MEMORY_x memory num */
        uint8_t   chan_even;          /* 1 = channel PORT is even ==> GPIO shift*/
    } ADC_CHANNEL_BLK;

const ADC_CHANNEL_BLK  _g_adc_channels [] =  //                          Idx Pin   User   Energia   LP   Grove
        {  { GP_PORT_9, BIT2, ADC12_B_INPUT_A10,ADC12_B_MEMORY_7, 0 }, // 0  P 9.2 Adc10  A2    2   J1-2
           { GP_PORT_9, BIT3, ADC12_B_INPUT_A11,ADC12_B_MEMORY_8, 0 }, // 1  P 9.3 Adc11  A6    6   J1-6
           { GP_PORT_9, BIT4, ADC12_B_INPUT_A12,ADC12_B_MEMORY_9, 0 }, // 2  P 9.4 Adc12  A17  17   J2-4
           { GP_PORT_8, BIT4, ADC12_B_INPUT_A7, ADC12_B_MEMORY_4, 1 }, // 3  P 8.4 Adc7   A23  23   J3-3   J5
           { GP_PORT_1, BIT5, ADC12_B_INPUT_A6, ADC12_B_MEMORY_3, 1 }, // 4  P 8.5 Adc6   A24  24   J3-4   J6
           { GP_PORT_8, BIT6, ADC12_B_INPUT_A5, ADC12_B_MEMORY_2, 1 }, // 5  P 8.6 Adc5   A25  25   J3-5   J7
           { GP_PORT_8, BIT7, ADC12_B_INPUT_A4, ADC12_B_MEMORY_1, 1 }, // 6  P 8.7 Adc4   A26  26   J3-6   J8
           { GP_PORT_9, BIT0, ADC12_B_INPUT_A8, ADC12_B_MEMORY_5, 0 }, // 7  P 9.0 Adc8   A27  27   J3-7
           { GP_PORT_9, BIT1, ADC12_B_INPUT_A9, ADC12_B_MEMORY_6, 0 }, // 8  P 9.1 Adc9   A28  28   J3-8
           { GP_PORT_9, BIT5, ADC12_B_INPUT_A13,ADC12_B_MEMORY_10,0 }, // 9  P 9.5 Adc13  A29  29   J3-9
           { GP_PORT_9, BIT6, ADC12_B_INPUT_A14,ADC12_B_MEMORY_11,0 }, //10  P 9.6 Adc14  A30  30   J3-10
           { GP_PORT_1, BIT3, ADC12_B_INPUT_A3, ADC12_B_MEMORY_0, 0 }  //11  P 1.3 Adc3   A34  34   J4-7
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC12_B_INPUT_TCMAP  },  // internal Temperature Sensor
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC12_B_INPUT_BATMAP },   // internal Battery Monitor Sensor
        };


//*****************************************************************************
//  board_adc_init
//
//         Configure the overall sampling clock used for the ADCs on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - PWM Module 1  Channel   1 / 2 / 3 / 4
//           - PWM Module 2  Channel   1 / 2 / 3 / 4
//           - Timer n
//
// FR6989 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC12MCTL0 -> ADC12MCTL15.
// ADC results are stored in corresponding registers: ADC12MEM0 -> ADC12MEM15.
// Results are stored in the 16-bit ADC12MEMx registers.
//
// The Launchpad MSP430 FR6989 pin device physically supports up to 16 channels,
// of which 12 are wired out to the Launchpad pins, and 2 internal channels
// (Temperature Sensor, Battery Monitor) are available.
// Due to this layout, we support 14 total channels.
//*****************************************************************************
int  board_adc_init (int adc_module_id, uint32_t clock_rate, 
                     int trigger_type, int flags)
{
    ADC12_B_initParam  initParam  =  {0};
    int                rc;

    if (_g_adc_configured == 0)
       {

         if (trigger_type != ADC_TRIGGER_USER_APP)
            {      //-------------------------------------------------------------
                   // Setup SHS trigger field in ADC12CTL0
                   //-------------------------------------------------------------
              rc = board_adc_config_trigger_mode (trigger_type, flags);
              if (rc != 0)
                 return (rc);   // must be a bad trigger_type
            }

         _g_trigger_auser_api_id = trigger_type;  // save the trigger type we are using

             //--------------------------------------
             // do basic configuration of ADC module
             //--------------------------------------

#if (NOT_SURE_IF_NEED)
     // Initialize the shared reference module
     // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
  REFCTL0 |= REFVSEL_0 + REFON;             // Enable internal 1.2V reference
#endif

       //Initialize the ADC12B Module
       /*
       * Use internal ADC12B bit as sample/hold signal to start conversion
       * USE MODOSC 5MHZ Digital Oscillator as clock source
       * Use default clock divider/pre-divider of 1
       * Not using internal channel
       */
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect            = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider           = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider        = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap           = ADC12_B_NOINTCH;
    ADC12_B_init (ADC12_B_BASE, &initParam);
       /*
       *  For memory buffers 0-7  sample/hold for 16 clock cycles
       *  For memory buffers 8-15 sample/hold for 16 clock cycles (default)
       *  Disable Multiple Sampling
       */
    ADC12_B_setupSamplingTimer (ADC12_B_BASE,
                                ADC12_B_CYCLEHOLD_16_CYCLES,
                                ADC12_B_CYCLEHOLD_16_CYCLES,
                                ADC12_B_MULTIPLESAMPLESDISABLE);

// ADC12CTL0 = ADC12SHT0_2 | ADC12ON;        // Sampling time, S&H=16, ADC12 on
// ADC12CTL1 = ADC12SHP;                     // Use sampling timer
// ADC12CTL2 |= ADC12RES_2;                  // 12-bit conversion results

             //--------------------------------------
             // do basic configuration of ADC module
             //--------------------------------------
             // CONSEQ_1 = single Sequence-of-Channels operation
         ADC12CTL0  = ADC12ON  + ADC12SHT0_2 + ADC12MSC; // Turn on ADC12, set sampling time = 16
         ADC12CTL1  = ADC12SHP + ADC12CONSEQ_1;          // Use sampling timer, Sequence-of-Channels
         ADC12CTL2 |= ADC12RES_2;                        // 12-bit conversion results

// ADC12CTL1  = ADC12SHS_0  | ADC12SSEL_0 | ADC12SHP | ADC12CONSEQ_0;
// ADC12CTL2 |= ADC12DF;

         _g_adc_configured = 1;              // denote basic configuration was done

#if (DMA_FUTURE)
         case DMA_CH7_ADC12C:
            DMA->rCH7_SRCCFG.r = (mapping >> 24) & 0x1F;

             //-----------------------------------------------------------------
             // Initialize the DMA associated with the ADC module.
             //   ?? Or should we hold off on this until we hit adc_enable() ?
             //-----------------------------------------------------------------
         DmaHandle.Instance         = DMA2_Stream0;
         DmaHandle.Init.Channel     = DMA_CHANNEL_0;
         DmaHandle.Init.Direction   = DMA_PERIPH_TO_MEMORY;    // ADC -> RAM

         DmaHandle.Init.PeriphInc   = DMA_PINC_DISABLE;
         DmaHandle.Init.MemInc      = DMA_MINC_ENABLE;         // step buf ptr
         DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
//       DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//       DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
         DmaHandle.Init.Mode        = DMA_CIRCULAR;     // Treat as circular buf
         DmaHandle.Init.Priority    = DMA_PRIORITY_HIGH;
         DmaHandle.Init.FIFOMode    = DMA_FIFOMODE_DISABLE;
         DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
         DmaHandle.Init.MemBurst    = DMA_MBURST_SINGLE;
         DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

            // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&AdcHandle, DMA_Handle, DmaHandle);

            // Configure the NVIC for DMA interrupts.
            // NVIC configuration for DMA transfer complete interrupt.
         HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
         HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
#endif
       }

//  ADCClockConfigSet (uint32_t ui32Base, uint32_t ui32Config,
//                     uint32_t ui32ClockDiv);

       // Configure the ADC to use PLL at 480 MHz divided by 24 to get an ADC
       // clock of 20 MHz.
       //
//  ADCClockConfigSet (ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24);

       // Configure the ADC to use PIOSC divided by one (16 MHz) and sample at
       // half the rate.
//  ADCClockConfigSet (ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate is determined by the trigger source.
//
//         adc_module parm is really just an index number (0-6)
//         that denote which ADC module to use.
//         For the FR5959, there is only 1 ADC module: ADC12
//         so we effectively ignore the adc_module_id
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference             (FUTURE_UPGRAGDE)
//
//         ADC12 has just one module.
//         ADC12 has just one sequencer.
//         It suppports up to 16 channels, of which 8 are hooked up on the LP.
//*****************************************************************************

int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer,     int step_num,
                               int last,          int flags)
{
    ADC_CHANNEL_BLK  *chblk;
//  ADC_SEQSTEP_BLK  *ssreg;
    uint8_t          mem_step;
    uint8_t          eos_flag;
    uint16_t         bit_mask;
    uint32_t         gpio_base_addr;
    int              orig_step_num;
    int              rc;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

    if (channel_num < 0  ||  channel_num > 16)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

          //***********************************************
          //      Add a new ADC channel to be sampled
          //***********************************************

          //------------------------------------------------------------------
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of n entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[]
          //------------------------------------------------------------------
    chblk = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num];
    gpio_base_addr = _g_gpio_port_base_address[chblk->chan_gpio_port_id];
          // set associated SEL bits to denote this is being used
          // as a ADC channel
    bit_mask = chblk->chan_gpio_pin_bit;
    if (chblk->chan_even)
       bit_mask <<= 8;                       // must shift by 8 if port is even
    HWREG16(gpio_base_addr + OFS_PADIR) &= ~(bit_mask); // set for Input
    HWREG16(gpio_base_addr + OFS_PASEL0) |= bit_mask;   // Set for TERNARY
    HWREG16(gpio_base_addr + OFS_PASEL1) |= bit_mask;   //    Periph Function

          //----------------------------------------------------------------
          //         Setup the step within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save original step number passed in
    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

    if (last)
       eos_flag = ADC12_B_ENDOFSEQUENCE;
       else eos_flag = ADC12_B_NOTENDOFSEQUENCE;
    mem_step = _g_adc_seqstep_regs [step_num];
    _g_ADC_param.memoryBufferControlIndex = mem_step;
    _g_ADC_param.inputSourceSelect        = chblk->chan_adc_num;
    _g_ADC_param.refVoltageSourceSelect   = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    _g_ADC_param.windowComparatorSelect   = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    _g_ADC_param.differentialModeSelect   = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    _g_ADC_param.endOfSequence            = eos_flag;
    ADC12_B_configureMemory (ADC12_B_BASE, &_g_ADC_param);

    if (orig_step_num == ADC_AUTO_STEP)
       { _g_adc_step_map [_g_step_num] = channel_num; // save what channel is assigned to this step
         _g_step_num++;            // inc to next step slot in the sequencer
       }
      else _g_adc_step_map [step_num] = channel_num;

    _g_active_channels++;  // inc # channels we have configured for the ADC

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_trigger_mode
//
//         Sets up ADC to use a Timer's Trigger Output (TRGO), which is used
//         to automatically trigger the ADC.
//
//         The trigger field is specified in the ADC12SHSxx bits of the
//         ADC14CTL0 register.
//         For the MSP430-FR6989 these are:
//
//                 Timer/ CCR      SHS Value
//                  TA0 / CCR1        1
//                  TB0 / CCR0        2
//                  TB0 / CCR1        3
//                  TA1 / CCR1        4
//                  TA2 / CCR1        5
//                  TA3 / CCR1        6
//*****************************************************************************

int  board_adc_config_trigger_mode (int trigger_type, int flags)
{
    uint32_t  shs_value;

    if (trigger_type != ADC_TRIGGER_USER_APP)
       {    // user wants a timer to trigger the ADC14
         if (trigger_type < 1 || trigger_type > 7)
            return (ERR_TIMER_INVALID_TRIGGER_TYPE);
         shs_value = _g_adctrigger_shs_mask [trigger_type];

            // then apply the SHS to the ADC12CTL1 register
         ADC12CTL1 &= ~(ADC12SHS_7);         // Shutoff all the SHS bits
         ADC12CTL1 |= shs_value;             // apply desired value
       }
    _g_trigger_auser_api_id = trigger_type;  // save type of trigger configured

    return (0);                              // denote completed ok
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
    int     rc;

// this is always returning busy after first Seqence-of_Conversions complete, even when SW triggerd !
//    rc = ADC12_A_isBusy (ADC12_A_BASE);

    if ( ! _g_ADC_complete)
       return (0);                     // ADC and DMA are still busy

    return (1);                        // ADC/DMA conversion sequenc is complete
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
//   if (ADC12_A_isBusy(ADC12_A_BASE)  &&  _g_ADC_complete == 0)
     if (_g_ADC_complete == 0)
        return (-1);                 // must wait till all conversions are done
                                     // ??? do it in here - loop ???

    ADC12CTL0 &= ~(ADC12ON | ADC12ENC);  // turn off the ADC module for sampling

    return (0);                      // denote success
}


//*****************************************************************************
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//          To ensure cross-platform compatibility, we also kick off
//          an initial ADC conversion, if we are configured for USER SW Trigger
//
//    NOTE: if using Timer triggering of the ADC, the user app _MUST_ issue
//          either a timer_ADC_Trigger_Start() call or a timer_Enable_CCR_Output()
//          call to startup the timer and its associated CCR, otherwise the
//          ADC will never start sampling.
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    uint16_t  ie_mask;
    uint16_t  shs_mask;

    _g_ADC_complete = 0;                 // clear I/O flags
    _g_DMA_overrun  = 0;

        //----------------------------------------------------------------------------
        // clear any previous rupts, then turn on ADC interrupts
        //----------------------------------------------------------------------------
//  ADC12_B_clearInterrupt (ADC12_B_BASE, 0, ADC12_B_IFG0);
//  ADC12_B_enableInterrupt (ADC12_B_BASE, ADC12_B_IE0, 0, 0);
    ADC12IFGR0 &= ~(ie_mask);            // Clear any old ADC12 IFG.x interrupts
        // ADC12IER0 handles channels 0 - 15
    ie_mask = 1 << (_g_active_channels - 8);
    ADC12IER0   = ie_mask;          // Enable the correct ADC12 IFG.x interrupt
    if (_g_active_channels >= 16)
       {     // ADC12IER1 handles channels 16 - 31
         ie_mask = 1 << (_g_active_channels - 16);
         ADC12IER1   = ie_mask;          // Enable the correct ADC12 IFG.x interrupt
       }                                 //        based on last EOS channel.

    if (ADC12CTL1 & ADC12CONSEQ_2)
       _g_toggle_ENC_required = 0;       // is a REPEAT sequence, no need to toggle ENC

         //----------------------------------------------------------------------------
         // if automatic (timer) triggering was requested, turn it on in ADC12CTL1 SHS
         //----------------------------------------------------------------------------
    if (_g_trigger_auser_api_id !=  ADC_TRIGGER_USER_APP)
       { shs_mask = _g_adctrigger_shs_mask [_g_trigger_auser_api_id];
            // then apply the SHS to the ADC14CTL0 register
         ADC12CTL1 &= ~(ADC12SHS_7);     // Shutoff any existing SHS bits
         ADC12CTL1 |= shs_mask;          // Turn on requested triggering
       }

    ADC12CTL0 |= ADC12ENC;               // Enable conversions
    ADC12CTL0 |= ADC12ON;                // turn on the ADC module for sampling

       //---------------------------------------------------------
       // Even for (timer) triggered conversions, we should issue
       // an initial start (SC) to "prime the pump"
       //---------------------------------------------------------
/// if (_g_trigger_auser_api_id == ADC_TRIGGER_USER_APP)
    board_adc_user_trigger_start (0, 0); // kick off initial Conversion

    return (0);                          // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer,
		            uint16_t *channel_results)
{
    int             i;
    unsigned short  *adc_mem_data;

#if (TO_DO)
//  channel_results[0] = HAL_ADC_GetValue (ADC1);   DMA will have already put it in channel_results

    if (_g_DMA_complete == 0)
       return (0);             // denote 0 results because DMA rupt has not happened yet
    if (_g_DMA_overrun)
       {    // we had an overrun. Discard the results and tell user try again
         _g_DMA_overrun  = 0;  // clear error flag
         _g_DMA_complete = 0;  // reset for new pass
         return (-1);          // denote we had an overrun condition
       }
#endif

#if (INTERRUPTS)
#else
       // interrupts are not working - so copy from MEMx area first
    adc_mem_data = &ADC12MEM0;       // point at begin of ADC results memory buf
    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally DMA staged results into staging buffer
         _g_adc_conv_results[i] = *adc_mem_data;  // copy data to internal staging area
         adc_mem_data++;                          // step to next ADC12MEMx
       }
           // TEMP HACK until get address of ADCMEM stuff working
//  _g_adc_conv_results[0] = ADC14MEM0;
//  _g_adc_conv_results[1] = ADC14MEM1;
//  _g_adc_conv_results[2] = ADC14MEM2;
//  _g_adc_conv_results[3] = ADC14MEM3;
#endif

    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally ADC/DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_ADC_complete = 0;                 // reset for new pass

    return (_g_active_channels); // pass back number of completed conversions
}



//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a sequenced group.
//
//          For MSP430, the various SHP/SHSx settings require different
//          combinations of manually toggling SC and/or ENC to start a new
//          conversion. For some combinations of SHP/SHSx, just triggering
//          ADC12SC is insufficient. So rather than try to track the 6
//          different SHP/SHSx combinations, just brute force it and toggle
//          ENC each time. That handles all the weird combinations.
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
    ADC12CTL0 &= ~(ADC12ENC);        // clear ENC to force state machine reset
    ADC12CTL0 |= ADC12SC + ADC12ENC; // Start new conversion via software trig

    return (0);                      // denote success
}


//*****************************************************************************
//  board_adc_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when the ADC conversions are complete.
//*****************************************************************************
int  board_adc_set_callback (int module_id,
                   ADC_CB_EVENT_HANDLER callback_function, void *callback_parm)
{
       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

        //----------------------------------------------------------------
        // Save ADC completion callback parameters.
        //----------------------------------------------------------------
    _g_adc_callback      = callback_function;
    _g_adc_callback_parm = callback_parm;

    return (0);                             // denote success
}


/****************************************************************************
*                                ADC    ISR
*
* @brief  ADC Conversion complete callback
*
*         If running straight interrupts (xxx_IT), this gets called by
*         the HAL ADC library support when the ADC interrupt completes.
*
*         If running with DMA (xxx_DMA), this gets called
*         when the DMA interrupt completes, i.e. this gets
*         invoked as a callback indirectly via the HAL_DMA_IRQHandler().
*
*         If ENC toggling is required, allow a bunch of instructions to
*         execute, before turning it back on, to ensure the ADC properly
*         reset itself for the next conversion. PITA
****************************************************************************/

#ifndef TEST_TRIGGER

// ADC12 interrupt service routine
#pragma vector=ADC12_VECTOR
__interrupt void  ADC12_ISR (void)
{
    int             i;
    unsigned short  *adc_mem_data;

    if (ADC12IV > 6 && ADC12IV <= 30)
       {     // this is the ending IFG0-IFG14 signal denoting end of sequence completed.
        if (_g_toggle_ENC_required)
           ADC12CTL0 &= ~(ADC12ENC);   // non-repeat sequences require that we 
                                       // toggle ENC after each conversion set

             // Move converted results to our internal staging buf.
        adc_mem_data = &ADC12MEM0;       // point at begin of ADC results memory buf

        for (i = 0;  i < _g_active_channels;  i++)
          {       // copy the internally DMA staged results into our staging buffer.
            _g_adc_conv_results[i] = *adc_mem_data;  // copy data to internal staging area
            adc_mem_data++;                          // step to next ADC12MEMx
          }

        _g_ADC_complete = 1;           // set status that ADCs and DMA completed.
                                       // Alternative = invoke user callback rtn.
                                       // Used by adc_Check_All_Complete() logic

           //-------------------------------------------------------------
           // If a ADC completion callback has been configured, invoke it
           //-------------------------------------------------------------
        if (_g_adc_callback != 0L)
           {
              (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                                 _g_active_channels, 0);    // Call user handler
           }

        if (_g_toggle_ENC_required)
           ADC12CTL0 |= ADC12ENC;      // toggle ENC back on

       }
     else switch (__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
      {
        case ADC12IV_NONE:        break;        // Vector  0:  No interrupt
        case ADC12IV_ADC12OVIFG:  break;        // Vector  2:  ADC12MEMx Overflow
        case ADC12IV_ADC12TOVIFG: break;        // Vector  4:  Conversion time overflow
        case ADC12IV_ADC12HIIFG:  break;        // Vector  6:  ADC12BHI Window High Interrupt
        case ADC12IV_ADC12LOIFG:  break;        // Vector  8:  ADC12BLO Window Low Interrupt
        case ADC12IV_ADC12INIFG:  break;        // Vector 10:  ADC12BIN Window Between
        case ADC12IV_ADC12IFG0:   break;        // Vector 12:  ADC12MEM0
        case ADC12IV_ADC12IFG1:   break;        // Vector 14:  ADC12MEM1
        case ADC12IV_ADC12IFG2:   break;        // Vector 16:  ADC12MEM2

        case ADC12IV_ADC12IFG3:                 // Vector 18:  ADC12MEM3
            _g_adc_conv_results[0] = ADC12MEM0;  // Move results, IFG is cleared
            _g_adc_conv_results[1] = ADC12MEM1;  // Move results, IFG is cleared
            _g_adc_conv_results[2] = ADC12MEM2;  // Move results, IFG is cleared
            _g_adc_conv_results[3] = ADC12MEM3;  // Move results, IFG is cleared
//          __bic_SR_register_on_exit(LPM4_bits);// Exit active CPU, SET BREAKPOINT HERE
            break;

        case ADC12IV_ADC12IFG4:   break;        // Vector 20:  ADC12MEM4
        case ADC12IV_ADC12IFG5:   break;        // Vector 22:  ADC12MEM5
        case ADC12IV_ADC12IFG6:   break;        // Vector 24:  ADC12MEM6
        case ADC12IV_ADC12IFG7:   break;        // Vector 26:  ADC12MEM7
        case ADC12IV_ADC12IFG8:   break;        // Vector 28:  ADC12MEM8
        case ADC12IV_ADC12IFG9:   break;        // Vector 30:  ADC12MEM9
        case ADC12IV_ADC12IFG10:  break;        // Vector 32:  ADC12MEM10
        case ADC12IV_ADC12IFG11:  break;        // Vector 34:  ADC12MEM11
        case ADC12IV_ADC12IFG12:  break;        // Vector 36:  ADC12MEM12
        case ADC12IV_ADC12IFG13:  break;        // Vector 38:  ADC12MEM13
        case ADC12IV_ADC12IFG14:  break;        // Vector 40:  ADC12MEM14
        case ADC12IV_ADC12IFG15:  break;        // Vector 42:  ADC12MEM15
        case ADC12IV_ADC12IFG16:  break;        // Vector 44:  ADC12MEM16
        case ADC12IV_ADC12IFG17:  break;        // Vector 46:  ADC12MEM17
        case ADC12IV_ADC12IFG18:  break;        // Vector 48:  ADC12MEM18
        case ADC12IV_ADC12IFG19:  break;        // Vector 50:  ADC12MEM19
        case ADC12IV_ADC12IFG20:  break;        // Vector 52:  ADC12MEM20
        case ADC12IV_ADC12IFG21:  break;        // Vector 54:  ADC12MEM21
        case ADC12IV_ADC12IFG22:  break;        // Vector 56:  ADC12MEM22
        case ADC12IV_ADC12IFG23:  break;        // Vector 58:  ADC12MEM23
        case ADC12IV_ADC12IFG24:  break;        // Vector 60:  ADC12MEM24
        case ADC12IV_ADC12IFG25:  break;        // Vector 62:  ADC12MEM25
        case ADC12IV_ADC12IFG26:  break;        // Vector 64:  ADC12MEM26
        case ADC12IV_ADC12IFG27:  break;        // Vector 66:  ADC12MEM27
        case ADC12IV_ADC12IFG28:  break;        // Vector 68:  ADC12MEM28
        case ADC12IV_ADC12IFG29:  break;        // Vector 70:  ADC12MEM29
        case ADC12IV_ADC12IFG30:  break;        // Vector 72:  ADC12MEM30
        case ADC12IV_ADC12IFG31:  break;        // Vector 74:  ADC12MEM31
        case ADC12IV_ADC12RDYIFG: break;        // Vector 76:  ADC12RDY

        default: break; 
      }

    dma_callback_seen++;                        // DEBUG COUNTER

}
#endif                         // ifndef TEST_TRIGGER  TEMP TEST HACK


/************************************************************************
*                              DMA   ISR
*
* @brief  This function handles DMA interrupt request.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

#if (TO_DO)

void  DMA2_Stream0_IRQHandler (void)
{
    dma_rupt_seen++;                        // DEBUG COUNTER
    HAL_DMA_IRQHandler (AdcHandle.DMA_Handle);

//  HAL_ADC_Stop_DMA(hadc);   // ??? need - bit is blow up when re-enable ADC_IT
}
#endif

#endif                        // USES_ADC



//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************


//*****************************************************************************
//  board_gpio_init
//
//        Configure Port Directions and Peripherals as needed.
//
//        CAUTION: at chip startup, FR6989 drives all GPIOs
//                 (and clocks) to High Impendance. Must hit
//                 it with a PM5CTL0 &= ~LOCKLPM5 to kick it
//                 out of high-impandance mode, and Enable the
//                 GPIOs. Otherwise all the pins will be
//                 floating, even though they have been configured.
//*****************************************************************************
void  board_gpio_init (void)
{
// The following causes grief at startup because I have no ISR defined for PORT4_ISR
#if (SCRUB_IT)
       // Configure the left button (S2) connected to P4.6. 
       // For this enable the internal pull-up resistor and
       // setup the pin interrupt to trigger on rising edges.
    GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P4, GPIO_PIN5);
    GPIO_selectInterruptEdge (GPIO_PORT_P4, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt (GPIO_PORT_P4, GPIO_PIN5);
    GPIO_enableInterrupt (GPIO_PORT_P4, GPIO_PIN5);
#endif

// The following causes grief with CC3100 - rains extraneous rupts on PORT1_ISR
#if (SCRUB_IT)
       // Configure the right button (S3) connected to P1.1. 
       // For this enable the internal pull-up resistor and
       // setup the pin interrupt to trigger on rising edges.
    GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P1, GPIO_PIN1);
    GPIO_selectInterruptEdge (GPIO_PORT_P1, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt (GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt (GPIO_PORT_P1, GPIO_PIN1);
#endif

#if defined(USES_CC3100)

 #if defined(__MSP430FR5969__)
        //-------------------------------------------------------
        // Configure CC3100 SPI CS GPIO pin we are using (P 3.0) = J2-3
        //-------------------------------------------------------
    P3OUT  &= ~BIT0;             // ensure it is set as De-asserted (LOW)  ???
    P3SEL1 &= ~BIT0;             // Set it as a GPIO
    P3SEL0 &= ~BIT0;
    P3DIR  |= BIT0;              // set as OUTPUT

        //----------------------------------------
        // configure CC3100 nHIB/ENABLE on P 4.3 = J1-5
        //----------------------------------------
    P4SEL1 &= ~BIT3;             // Set it as a GPIO
    P4SEL0 &= ~BIT3;

    P4OUT &= ~BIT3;              // set CC3100 nHIB to DISABLED
    P4DIR |=  BIT3;              // set as OUTPUT

        //-----------------------------------------
        // configure CC3100 host IRQ line on P 1.2 = J2-2
        //-----------------------------------------
    P1SEL1 &= ~BIT2;             // SEL0/1 = 00 = Set it as a GPIO
    P1SEL0 &= ~BIT2;
    P1DIR  &= ~BIT2;             // set it for INPUT

    P1REN |= BIT2;               // Turn on pullups for IRQ line
 #endif                          //  if defined()

 #if defined(__MSP430FR6989__)
        //-------------------------------------------------------
        // Configure CC3100 SPI CS GPIO pin we are using (P 1.5) - J2-3
        //-------------------------------------------------------
    DEASSERT_CC3100_CS_NORMAL();
    INIT_CC3100_CS();

        //----------------------------------------
        // configure CC3100 nHIB/ENABLE on P 3.2 = J1-5
        //----------------------------------------
     NHIB_CC3100_DISABLE();
     INIT_CC3100_NHIB_ENABLE();

        //-----------------------------------------
        // configure CC3100 host IRQ line on P 2.1 = J2-2
        //-----------------------------------------
    INIT_CC3100_IRQ();
 #endif

        //---------------------------------------
        // Enable MCU gerneral interrupts (GIE)
        //---------------------------------------
///  __enable_interrupt();                  //  Do later after SPI module configured

    Delay (50);         // 50 ms delay needed so CC3100 can perform its startup

        //-------------------------------------------------------------------
        // Enable CC3100 interrupt (IRQ line)
        //
        // When CC3100 startup completes, it will toggle an IRQ interrupt.
        // The IRQ handler will then read in the CC3100 status and complete
        // signal startup complete to the sl_Start() routine.
        //--------------------------------------------------------------------
/// CC3100_InterruptEnable();               //  Do later after SPI module configured
                                            // else will get false interrupts because SL ISR handler was not setup yet
#endif                           // #if defined(USES_CC3100)

#if defined(USES_DRV8711)
       //------------------------------------------------------
       // Configure DRV8711 SPI CS GPIO pin we are using (PA2)
       //------------------------------------------------------
    DEASSERT_DRV8711_CS();  // FR6989 CHG   // Ensure CS is _De-Asserted_ (Low)
    P1DIR  |= BIT3;             // set Chip Select (CS) P 1.3 to output

       // Configure DRV8711 GPIOs: Direction + Function Mode (GPIO, Analog, PWM)
    P4SEL1 |= (POT);            // POT = Potentiometer = Analog In  (A10)
    P4SEL0 |= (POT);

       // Setup DRV8711 input GPIO pins
    P1DIR  &= ~(nSTALL);        // FAULT/ STALL: ensure direction = IN
    P3DIR  &= ~(nFAULT);
    P1REN  |=  (nSTALL);        // Turn on Pull-Ups for FAULT and STALL
    P3REN  |=  (nFAULT);

       // Setup DRV8711 output GPIO pins
    P2SEL1 &= ~(nSLEEP);        // Ensure nSLEEP/ENABLE reset to GPIO mode
    P2SEL0 &= ~(nSLEEP);

    P2OUT  |= (nSLEEP);         // Set output LOW
    P2DIR  |= (nSLEEP);         // ensure direction = OUT

    P3SEL1 &= ~(RESET | STEP_AIN1 | DIR_AIN2 | nFAULT);  // Ensure reset to GPIO mode
    P3SEL0 &= ~(RESET | STEP_AIN1 | DIR_AIN2 | nFAULT);
    P1SEL1 &= ~(BIN2  | BIN1 | nSTALL);
    P1SEL0 &= ~(BIN2  | BIN1 | nSTALL);

    P3OUT  &= ~(RESET | STEP_AIN1 | DIR_AIN2);  // Set output LOW
    P1OUT  &= ~(BIN2  | BIN1);

    P3DIR  |=  (RESET | STEP_AIN1 | DIR_AIN2);  // ensure direction = OUT
    P1DIR  |=  (BIN2  | BIN1);
#endif                          // #if defined(USES_DRV8711)

    PM5CTL0 &= ~LOCKLPM5;  // Disable GPIO power-on default High-Impedance mode
                           // to activate configured port settings.
                           // This is ONLY for FRxxx FRAM devices !
                           // Normal Fxxx (F5529, ...) MCUs will crash on this instr.
}


/*******************************************************************************
*  Board GPIO Pin Config
*
*        Configure an individual GPIO pin.   Use default strength = 2ma/pin
*******************************************************************************/
void  board_gpio_pin_config (uint32_t gpio_port, unsigned long pin,
                             int dir, int pull)
{
    if (dir == GPIO_OUTPUT)
       GPIO_setAsOutputPin (gpio_port, pin);              // set pin to OUTPUT
       else {      // set as input, based on Pullup Resistor settings
              if (pull == GPIO_NOPULL)
                 GPIO_setAsInputPin (gpio_port, pin);     // set pin to INPUT
                 else if (pull == GPIO_PULLUP)
                         GPIO_setAsInputPinWithPullDownResistor (gpio_port,pin);
                 else if (pull == GPIO_PULLDOWN)
                         GPIO_setAsInputPinWithPullUpResistor (gpio_port, pin);
            }
}


/*******************************************************************************
*  board_gpio_read_pins
*
*        Read in the pins from a GPIO port.  Desired pins are passed via mask.
*        If flag = 0 = read input port.
*        If flag = 1 = read output port.
*******************************************************************************/

uint16_t  board_gpio_read_pins (int gpio_port_num,  uint16_t pins_mask, int flag)
{
    uint32_t  gpio_base_addr;
    uint16_t  port_values;

    if (gpio_port_num < 1 || gpio_port_num > MAX_GPIO_PORTS)
       return (0xFFFF);                               // denote error

    gpio_base_addr = _g_gpio_port_base_address [gpio_port_num];

    if ((gpio_port_num & 1) ^ 1)   // Handle MSP430 register "packing"
       pins_mask <<= 8;            // Shift by 8 if port is even (upper 8-bits)

    if ( ! flag)
       port_values = HWREG16(gpio_base_addr + OFS_PAIN) & (pins_mask);       // get INPUT port pins
       else port_values = HWREG16(gpio_base_addr + OFS_PAOUT) & (pins_mask); // get OUTPUT port pins

    return (port_values);
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
    unsigned char  rupt_mode;

    board_gpio_pin_config (gpio_port, pin, GPIO_INPUT,
                           pullup);                     // config base pin props

    if (rise_fall == GPIO_RUPT_MODE_RISING)
       rupt_mode = GPIO_LOW_TO_HIGH_TRANSITION;         // rising edge
       else rupt_mode = GPIO_HIGH_TO_LOW_TRANSITION;    // falling edge

    GPIO_clearInterruptFlag (gpio_port, pin);     // clear out out any old rupts

    GPIO_interruptEdgeSelect (gpio_port,pin,rupt_mode); // set type of interrupt

        // Model TI's support, and avoid any "early interrupts" that 
        // ISR may not be expecting (e.g. CC3100 Simplelink)
    GPIO_disableInterrupt (gpio_port, pin);

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
       GPIO_clearInterrupt (gpio_port, pin);   // clear out any prev interrupts
    GPIO_enableInterrupt (gpio_port, pin);
}




#if defined(USES_I2C)
//*****************************************************************************
//*****************************************************************************
//                               I2C   Routines
//*****************************************************************************
//*****************************************************************************


#endif


//*****************************************************************************
//*****************************************************************************
//                               SPI   Routines
//*****************************************************************************
//*****************************************************************************

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

uint32_t  board_spi_init (int spi_id,  int spi_mode,
                          int baud_rate_scalar,  int use_dma)
{
    uint32_t       spi_port_id;
    int            rc;

    spi_port_id = 0;


     ///  ------  TBD   -----


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




//*****************************************************************************
//*****************************************************************************
//                      System CPU Clocks  /  Systick   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_stop_WDT
//
//           Shutoff WDT
//*****************************************************************************

void  board_stop_WDT (void)
{
    WDTCTL = WDTPW | WDTHOLD;
}


//*****************************************************************************
//  board_system_clock_config
//
//          Setup CPU clocks.                          Max is 16 MHz on FR6989
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{
        // this is the original clock logic from WORKING version, that was running at 8 MHz
    WDTCTL   = WDTPW | WDTHOLD;  // turn off watchdog timer

    if (mcu_clock_hz >= 16000000)
       {            // Setup for 16 MHz DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 16 MHz   CPU clock
                    //---------------------------------
         FRCTL0 = FRCTLPW | NWAITS_1;   // > 8 MHz requires adding a wait state
                                        // to FRAM access

         CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
         CSCTL1   = DCORSEL | DCOFSEL_4;           // Set DCO to 16 MHz
         CSCTL2   = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // Set SMCLK = MCLK = DCO
                                                                // ACLK = VLOCLK
         CSCTL3   = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1
         CSCTL0_H = 0;                             // Lock CS registers
       }
      else if (mcu_clock_hz >= 8000000)
              {     // Setup for 8 MHz DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 8MHz   CPU clock  =  ORIGINAL WORKING LOGIC
                    //---------------------------------
                CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
                CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
                CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // Set SMCLK = MCLK = DCO
                                                                     // ACLK = VLOCLK
                CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
                CSCTL0_H = 0;                             // Lock CS registers
              }
      else    {     // Setup for 1 MHz (startup default) DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 1 MHz   CPU clock
                    //---------------------------------
                CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
                CSCTL1 = DCOFSEL_0;                       // Set DCO to 1MHz
                CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // Set SMCLK = MCLK = DCO
                                                                     // ACLK = VLOCLK
                CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
                CSCTL0_H = 0;                             // Lock CS registers
              }

        // save the configured clock ticks (MHz) settings
    _g_SysClk_Ticks = CS_getMCLK();       // save the MCU clock ticks setting
    _g_MCLK  = CS_getMCLK();             // save main CPU clock ticks
    _g_SMCLK = CS_getSMCLK();            // save the SMCLK peripheral clock ticks

        // compute Timer divisor factor for 1 ms timer. Yields needed CCR0 value
    _g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # of SMCLK ticks in 1 milli-sec
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
//       Return the board's SMCLK Peripheral clock frequency in ticks.
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SMCLK);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
// board_systick_timer_config                                        uses  TA2
//
//          Emulate a "SYSTICK" style Interval Timer.
//
//          This is required by MQTTCC3100.c support, as well as other
//          packages that need a milli-second iterval timer.
//
//          The FR6989 has two 16-bit Timer_A3 style timers (TA0/TA1),
//                         two 16-bit Timer_A2 style timers (TA2/TA3),
//                         one 16-bit Timer_B7 stype timer  (TB0).
//
//          We use Timer TA2 since it is free and not pinned out to any
//          external pins, and leaves TA0 available for use by an RTOS.
//*****************************************************************************
void  board_systick_timer_config (void)
{
    TA2CCTL0 = CCIE;                     // enable CCR0 interrupt
    TA2CCR0  = _g_TA1_1ms_ticks;         // number of SMCLKS that = 1 milli-sec
    TA2CTL   = TASSEL_2 + MC_1 + TACLR;  // use SMCLK, up mode, clear TAR
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
//                            TA2   CCR0   ISR
//
// SysTick interrupt handler.
//
//                               Increment Systick 1 ms count, on every 1ms pop
//*****************************************************************************
#pragma vector=TIMER2_A0_VECTOR
__interrupt void  TIMER_A2_A0CCR0_ISR (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)

#if defined(USES_MQTT)
    SysTickIntHandler();     // invoke MQTTCC3100.c to update its Timer
#endif

#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration (_g_systick_millisecs);
#endif
}




#if defined(USES_PWM) || defined(USES_TIMER) || defined(USES_ADC) || defined(USES_DAC) || defined(USES_DRV8711) || defined(USES_DRV8848) || defined(USES_DRV8301)

//*****************************************************************************
//*****************************************************************************
//                            TIMER  /  PWM     Routines
//*****************************************************************************
//*****************************************************************************
//     This section provides basic timer/pwm setup and trigger mode support

              //---------------------------------------------------------------
              //               Supported Physical Timer Modules    FR6989-PZ100
              //
              //   Module 0  TA0  3 CCR  pinned out to Launchpad: CCR 1,2
              //   Module 1  TA1  3 CCR  pinned out to Launchpad: CCR 1,2
              //   Module 2  TA2  2 CCR  none pinned out to LP              <=== TA2 USED BY SYSTICK
              //   Module 3  TA3  5 CCR  none pinned out to LP
              //   Module 4  TB0  7 CCR  pinned out to Launchpad: CCR 2,3,4,5,6
              //
              // We do _NOT_ support TA2, because it is needed for "Systick".
              // We do not support TA3 for PWM, because no pins for it are
              // routed out to the Launchpad.
              //
              // None of them support built-in complementary operation.
              // None of them support built-in Deadtime, but there are
              // ways to hack it (CCR2/CCR5 for deadtime, CCR3/CCR4 for PWM)
              //---------------------------------------------------------------
#define  TMRPWM_NUM_MODULES        5   /* are 5 Timer/PWM capable modukess on MSP432 LP */
#define  TMRPWM_MAX_CHANNELS       7   /* are max 7 channels per Timer/PWM module */

              // flags for _g_tmpwm_module_status[] entries
#define  PWM_DOWN_COUNT_INIT      0x01   /* Module setup for DOWN count mode */
#define  PWM_UP_COUNT_INIT        0x02   /* Module setup for UP   count mode */
#define  PWM_CENTER_COUNT_INIT    0x04   /* Module setup for UP/DOWN count mode */
#define  TMR_PWM_INTERRUPTS_USED  0x40   /* Module uses Timer interrupts     */

              // flags for _g_tmpwm_channels_config[] entries
              //       and _g_tmpwm_channels_pwm[]
#define  TMR_CCR3N_CONFIGURED     0x80        /* Comple channel 3N configured */
#define  TMR_CCR2N_CONFIGURED     0x40        /* Chan 6 or Comple channel 2N configured */
#define  TMR_CCR1N_CONFIGURED     0x20        /* Chan 5 or Comple channel 1N configured */
#define  TMR_CCR4_CONFIGURED      0x10        /* Channel 4 configured for use */
#define  TMR_CCR3_CONFIGURED      0x08        /* Channel 3 configured for use */
#define  TMR_CCR2_CONFIGURED      0x04        /* Channel 2 configured for use */
#define  TMR_CCR1_CONFIGURED      0x02        /* Channel 1 configured for use */
#define  TMR_CCR0_CONFIGURED      0x01        /* Channel 0 configured for use */

//    uint32_t           uwPrescalerValue = 0;

                           // Bit mask status of each Timer Module (initialized, ...)
      char             _g_tmpwm_module_status [TMRPWM_NUM_MODULES+1]  = { 0,0,0,0,0 };

                           // Bit Mask ENABLE status of CCRs in each module (enabled, ...)
                           // One entry per timer module.
      unsigned char    _g_tmpwm_channels_config [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0 };

                           // Bit Mask PWM status of CCRs in each module (Normal, Complementary,...)
                           // One entry per timer module.
      unsigned char    _g_tmpwm_channels_pwm [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0 };

                           // Bit mask of which CCRs are using interrupts in each module.
                           // One entry per timer module.
      uint16_t         _g_timer_app_enabled_interrupts [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0,0 };

 TMR_CB_EVENT_HANDLER  _g_ptimer_callback [TMRPWM_NUM_MODULES+1]   = { 0,0,0,0,0 };
      void             *_g_ptimer_callback_parm [TMRPWM_NUM_MODULES+1];

                           // Timer Prescalars used for each Timer module.
      uint32_t         _g_tmpwm_prescalars [TMRPWM_NUM_MODULES+1]     = { 0,0,0,0,0,0 };

                           // Timer OC reload values for IRQs servicing CCRs in Mode 4,5, ...
      uint16_t         _g_timer_A0_reload_value_array [TMRPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      uint16_t         _g_timer_A1_reload_value_array [TMRPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      uint16_t         _g_timer_A2_reload_value_array [TMRPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      uint16_t         _g_timer_A3_reload_value_array [TMRPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      uint16_t         _g_timer_B0_reload_value_array [TMRPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };


        //----------------------------------------------------------------------
        //     Global constants and Tables used to manage Timer/PWM Logic
        //----------------------------------------------------------------------

               //--------------------------------------------------------------
               //         Bit masks used to track if a channel is enabled
               //         and/or using COMPLEMENTARY PWM channel (1N, 2N, 3N)
               //--------------------------------------------------------------
const unsigned char   _g_pwm_chan_mask [8]
                               = {         0,            /* Channel 0 is unused */
                                   TMR_CCR1_CONFIGURED,  /* Channels 1-4 */
                                   TMR_CCR2_CONFIGURED,  /*   0x02  */
                                   TMR_CCR3_CONFIGURED,  /*   0x04  */
                                   TMR_CCR4_CONFIGURED,  /*   0x08  */
                                   TMR_CCR1N_CONFIGURED, /* Channels 1N-3N, 5-7 */
                                   TMR_CCR2N_CONFIGURED, /*   0x20  */
                                   TMR_CCR3N_CONFIGURED  /*   0x40  */
                                 };                      // there is no 4N

               //--------------------------------------------------------------
               //         Bit masks for associated OUTMOD_xx values for OC
               //--------------------------------------------------------------
const uint16_t        _g_OUTMOD_mask_table [8]
                               = { OUTMOD_0,       /* pass thru       */
                                   OUTMOD_1,       /* Set and hold    */
                                   OUTMOD_2,       /* poor man PWM toggle/reset  */
                                   OUTMOD_3,       /* poor man PWM set/reset     */
                                   OUTMOD_4,       /* toggle (output is 1/2 rate)*/
                                   OUTMOD_5,       /* Reset and hold      */
                                   OUTMOD_6,       /* real PWM toggle/set */
                                   OUTMOD_7,       /* real PWM reset/set  */
                                 };


               //--------------------------------------------------------------
               //       Bit masks for associated INTERRUPT value for each CCR
               //--------------------------------------------------------------
const uint16_t        _g_CCR_interrupt_mask []
                               = { TAxIV_TAIFG,    /* CCR0 / Timer Rollover  */
                                   TAxIV_TACCR1,   /* CCR1 reached interrupt */
                                   TAxIV_TACCR2,   /* CCR2 reached interrupt */
                                   TAxIV_TACCR3,   /* CCR3 reached interrupt */
                                   TAxIV_TACCR4,   /* CCR4 reached interrupt */
                                   TAxIV_TACCR5,   /* CCR5 reached interrupt */
                                   TAxIV_TACCR6    /* CCR6 reached interrupt */
                                 };

               //--------------------------------------------------------------
               //                    Trigger _CHANNEL_ Mapping Table
               //  Timer/ CCR            for ADC Trigger Usage
               //   TA0 / CCR1
               //   TB0 / CCR0
               //   TB0 / CCR1
               //   TA1 / CCR1
               //   TA2 / CCR1
               //   TA3 / CCR1
               //--------------------------------------------------------------
const int         _g_tmpwm_trigger_channel []
                               = {   -1,             // -1 denotes no trigger
                                   TIMER_CHANNEL_1,  // TRIGGER_TIMER_0_CC1
                                   TIMER_CHANNEL_0,  // TRIGGER_TIMER_4_CC0
                                   TIMER_CHANNEL_1,  // TRIGGER_TIMER_4_CC1
                                   TIMER_CHANNEL_1,  // TRIGGER_TIMER_1_CC1
                                   TIMER_CHANNEL_1,  // TRIGGER_TIMER_2_CC1
                                   TIMER_CHANNEL_1,  // TRIGGER_TIMER_3_CC1
                                      -1,            // -1 denotes no trigger entry
                                 };

               //--------------------------------------------------------------
               //                    Trigger _TIMER_ Mapping Table
               //--------------------------------------------------------------
const int         _g_tmpwm_trigger_timer []
                               = {   -1,        // -1 denotes no trigger
                                   TIMER_0,     // TRIGGER_TIMER_0_CC1
                                   TIMER_4,     // TRIGGER_TIMER_4_CC0
                                   TIMER_4,     // TRIGGER_TIMER_4_CC1
                                   TIMER_1,     // TRIGGER_TIMER_1_CC1
                                   TIMER_2,     // TRIGGER_TIMER_2_CC1
                                   TIMER_3,     // TRIGGER_TIMER_3_CC1
                                     -1,        // -1 denotes no trigger
                                 };


               //---------------------------------------------------------
               //  Lookup table to obtain TIMER/PWM Module Base Addresses
               //
               //  Used to generate Base Address, CCTLx addr, CCRx addr
               //---------------------------------------------------------
#define  CCR_OFFSET     OFS_TAxCCR0-OFS_TAxCCTL0

typedef struct tmpwm_baseaddr_def      /* Timer Address definitions */
    {
        uint16_t  tmrpwm_base_addr;    /* Base Address of Timer/PWM module */
        uint16_t  tmrpwm_cctlx_addr[7];/* Address of TAxCCTLx (0 if not present) */
    } TMRPWM_BASE_ADDR;


const TMRPWM_BASE_ADDR  _g_timerpwm_base_address [] =
              { { TIMER_A0_BASE, TIMER_A0_BASE+OFS_TAxCCTL0, TIMER_A0_BASE+OFS_TAxCCTL1,  // TA0  Timer_0 - 3 CCRs
                                 TIMER_A0_BASE+OFS_TAxCCTL2,         0, 
                                           0,                        0,        0        },

                { TIMER_A1_BASE, TIMER_A1_BASE+OFS_TAxCCTL0, TIMER_A1_BASE+OFS_TAxCCTL1,  // TA1  Timer_1 - 3 CCRs
                                 TIMER_A1_BASE+OFS_TAxCCTL2,         0, 
                                           0,                        0,        0        },

                    // We do not allow TA2 to be used by user. We have it dedicated to SYSTICK usage
                {       0,       TIMER_A2_BASE+OFS_TAxCCTL0, TIMER_A2_BASE+OFS_TAxCCTL1,  // TA2  Timer_2 - 2 CCRs
                                           0,                        0,
                                           0,                        0,        0        },

                { TIMER_A3_BASE, TIMER_A3_BASE+OFS_TAxCCTL0, TIMER_A3_BASE+OFS_TAxCCTL1,  // TA3  Timer_3 - 5 CCRs
                                 TIMER_A3_BASE+OFS_TAxCCTL2, TIMER_A3_BASE+OFS_TAxCCTL3,
                                 TIMER_A3_BASE+OFS_TAxCCTL4,         0,        0        },
                { TIMER_B0_BASE, TIMER_B0_BASE+OFS_TAxCCTL0, TIMER_B0_BASE+OFS_TAxCCTL1,  // TB0  Timer_4 - 7 CCRs
                                 TIMER_B0_BASE+OFS_TAxCCTL2, TIMER_B0_BASE+OFS_TAxCCTL3,
                                 TIMER_B0_BASE+OFS_TAxCCTL4, TIMER_B0_BASE+OFS_TAxCCTL5,
                                 TIMER_B0_BASE+OFS_TAxCCTL6                             }
              };

                //--------------------------------------------------------------
                //                      GPIO   MAPPING   TABLES
                //
                // These tables contain the GPIO mapping for every GPIO pin that
                // can be used for Input (Input Capture) or Output (PWM or
                // Timer Output Compare) by the Timer/PWM modules.
                //
                // They are also correlated to a Timer Channel number (1,2,3,4)
                // which are in turn controlled by a CCR with the same number,
                // e.g. Timer Channel 1 (ex TA0.1) is controlled by CCR1 on
                // the Timer TA0 module; Timer Channel 3 (ex TA.3) is
                // controlled by CCR3 on the Timer TA2 module, etc.
                //--------------------------------------------------------------
typedef struct tmpwm_channel_def      /* Timer/PWM Channel definitions */
    {
        uint32_t  pwm_gpio_port_id;   /* Associated GPIO port         */
        uint32_t  pwm_gpio_pin_bit;   /* Associated GPIO pin          */
        uint32_t  pwm_ccr_num;        /* Associated PWM/Timer CCR num */
        uint8_t   pwm_even;           /* port is even = must shift bits by 8 */
    } TMRPWM_CHANNEL_BLK;

                           // Timer/PWM_MODULE_0  -  TA0  3 CCRs  (0-2)
const TMRPWM_CHANNEL_BLK  _g_tmpwm_mod_0_channels [] =                 // entry MCU   Func   Energia Launchpad
        {  { GP_PORT_1, BIT5, TIMER_A_CAPTURECOMPARE_REGISTER_0, 0 },  // CCR0  P 1.5  TA0.0  not pinned out to LP
           { GP_PORT_1, BIT6, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0 },  // CCR1  P 1.6  TA0.1  15     J2-6 
           { GP_PORT_1, BIT7, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0 },  // CCR2  P 1.7  TA0.2  14     J2-7
        };

                           // Timer/PWM_MODULE_1  -  TA1  3 CCRs  (0-2)
const TMRPWM_CHANNEL_BLK  _g_tmpwm_mod_1_channels [] =                 // entry MCU   Func  Energia Launchpad
        {  { GP_PORT_1, BIT5, TIMER_A_CAPTURECOMPARE_REGISTER_0, 0 },  // CCR0  P 1.4  TA1.0  not pinned out to LP
           { GP_PORT_3, BIT3, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0 },  // CCR1  P 3.3  TA1.1  38     J4-3
           { GP_PORT_1, BIT3, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0 },  // CCR2  P 1.3  TA1.2  34     J4-7  also on P4.7 11 J2-11
        };

                           // Timer/PWM_MODULE_2  -  TA2  2 CCRs  (0-1) none are pinned out - TIMER ONLY
const TMRPWM_CHANNEL_BLK  _g_tmpwm_mod_2_channels [] =
        {  {    0,        0,           0,                        0 },  // CCR0 - not connected to any GPIO
           {    0,        0,           0,                        0 },  // CCR1 - not connected to any GPIO
        };

                           // Timer/PWM_MODULE_3  -  TA3  5 CCRs  (0-4) none are pinned out - TIMER ONLY
const TMRPWM_CHANNEL_BLK  _g_tmpwm_mod_3_channels [] =
        {  {    0,        0,           0,                        0 },  // CCR0 - not connected to any GPIO
           {    0,        0,           0,                        0 },  // CCR1 - not connected to any GPIO
           {    0,        0,           0,                        0 },  // CCR2 - not connected to any GPIO
           {    0,        0,           0,                        0 },  // CCR3 - not connected to any GPIO
           {    0,        0,           0,                        0 },  // CCR4 - not connected to any GPIO
        };

                          // Timer/PWM_MODULE_4  -  TB0  7 CCRs  (0-6)
const TMRPWM_CHANNEL_BLK  _g_tmpwm_mod_4_channels [] =                 // entry MCU   Func   Energia Launchpad
        {  { GP_PORT_3, BIT4, TIMER_A_CAPTURECOMPARE_REGISTER_0, 0 },  // CCR0  P 3.4  TB0.0   not pinned out to LP
           { GP_PORT_3, BIT5, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0 },  // CCR1  P 3.5  TB0.1   not pinned out to LP
           { GP_PORT_3, BIT6, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0 },  // CCR2  P 3.6  TB0.2   37     J4-4
           { GP_PORT_3, BIT7, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0 },  // CCR3  P 3.7  TB0.3   36     J4-5  also on P2.4  12  J2-9
           { GP_PORT_2, BIT2, TIMER_A_CAPTURECOMPARE_REGISTER_4, 1 },  // CCR4  P 2.2  TB0.4   35     J4-6  also on P2.5  13  J2-8
           { GP_PORT_2, BIT6, TIMER_A_CAPTURECOMPARE_REGISTER_5, 1 },  // CCR5  P 2.6  TB0.5   39     J4-2
           { GP_PORT_2, BIT7, TIMER_A_CAPTURECOMPARE_REGISTER_6, 1 },  // CCR6  P 2.7  TB0.6   40     J4-1  also on P2.0   8  J1-8
        };


//******************************************************************************
//  board_timerpwm_compute_prescalar
//
//            Dynamically compute pre-scalar value needed to make user
//            requested period fit into 16-bit PWM counters
//******************************************************************************
long  board_timerpwm_compute_prescalar (long period_val)
{
    long   prescalar;

       // Compute any needed prescaler value to fit 16-bit TIMx counter clock
    if (period_val > 6550000)            // 6.50 M ticks
       prescalar = 800;
       else if (period_val > 655000)     // 0.65 M ticks
               prescalar = 80;
       else if (period_val > 65500)      // 0.065 M ticks
               prescalar = 8;
               else prescalar = 0;

    return (prescalar);
}


//*****************************************************************************
//  board_timerpwm_init
//
//         Initialize Timer/PWM mode
//
//         flags parm is for future use.
//*****************************************************************************

int  board_timerpwm_init (int module_id, int count_mode, long period_val,
                          int timer_clock_source, int flags)
{
    uint32_t     prescalar;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (count_mode == TIMER_PERIODIC_COUNT_UP || count_mode == TIMER_PERIODIC_COUNT_UPDOWN)
       ;                                           // mode is valid
      else return (ERR_PWM_COUNT_MODE_INVALID);    // count_mode is bad - bail

// ??? revisit the following
        /* Compute the prescaler value to have TIMx counter clock equal to 18 MHz */
//  prescalar = ((SystemCoreClock /2) / 18000000) - 1;

      //--------------------------------------------------------
      // Configure the associated TAx / TBx  module by setting
      // the period into the Timer's CCR0 register.
      // Save the Timer count mode for later configuration.
      //--------------------------------------------------------
    HWREG16(timerccr) = period_val;

    _g_tmpwm_module_status [module_id] = count_mode;    // save count mode

    return (0);                                         // denote success
}


//*****************************************************************************
//  board_timerpwm_check_completed
//
//        Checks to see if the timer has expired / completed it's cycle.
//        Optionally, resets the completion flag if denoted in reset_flags parm.
//*****************************************************************************
int  board_timerpwm_check_completed (int module_id, int mask_flags, int reset_flags)
{

// ??? !!!  WVD   NEED TO ADD CODE   ??? !!!

#if (FIX_LATER_TODAY)
    TIM_HandleTypeDef   *hdltimer;
    uint32_t          timer_base_addr;

    if (module_id < 0  ||  module_id >= TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3
    if (timer_base_addr == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //-----------------------------------------------------------------------
       // Check Timer's UPDATE flag status, to see if a timer rollover occurred
       //-----------------------------------------------------------------------
    if (module_id->SR & TIM_SR_UIF)
       {      // yes, Timer has popped/rolled over
         if (reset_flags == 1)
            timbase->SR = ~(TIM_SR_UIF);
         return (1);                    // tell caller it popped
       }

//        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
//#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)
//             ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
// #define TIM_IT_UPDATE     ((uint32_t)0x00000001)  =  TIM_SR_UIF
#endif

    return (0);                         // Timer has not popped yet
}


//*****************************************************************************
//  board_timerpwm_config_channel
//
//         Configure a specific channel (1-6) on a specific Timer/PWM module
//         (TA0/TA1/...).
//
//         There is no pin-muxing on MSP432, so there should be no _ALT1 / ALT2
//         etc channel numbers, only TIMER_CHANNEL_1 through TIMER_CHANNEL_6.
//
//         However, we do need to tweak the SELxx bits, to change the pin from
//         GPIO mode to TIMER mode.
//*****************************************************************************

int  board_timerpwm_config_channel (int module_id, int chan_num,
                                    long initial_duty, int mode, int flags)
{
    TMRPWM_CHANNEL_BLK  *tmpwmcblk;
    uint32_t            gpio_base_addr;
    uint16_t            bit_mask;
    long                reload_value;
    int                 ccr_ctl_value;
    int                 rupt_mask;
    unsigned char       chan_mask;
    uint16_t            timerbase;
    uint16_t            timercctl;
    uint16_t            timerccr;
    int                 rc;

    rc = board_timerpwm_lookup (module_id, chan_num, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (mode < 0 || mode > TIMER_MODE_PWM)
       return (ERR_PWM_INVALID_TIMER_MODE);

       //-------------------------------------------------------------
       // Setup associated GPIO(s) into Timer/PWM mode
       //
       // Convert PWM channel number to an index into table of 12 entries
       // that contains pin # and GPIO base index, then use  _g_gpio_base[]
       //----------------------------------------------------
    if (module_id == TIMER_0)
       tmpwmcblk = (TMRPWM_CHANNEL_BLK*) &_g_tmpwm_mod_0_channels [chan_num];         // PWM_MODULE_0
       else if (module_id == TIMER_1)
               tmpwmcblk = (TMRPWM_CHANNEL_BLK*) &_g_tmpwm_mod_1_channels [chan_num]; // PWM_MODULE_1
       else if (module_id == TIMER_2)
               tmpwmcblk = (TMRPWM_CHANNEL_BLK*) &_g_tmpwm_mod_2_channels [chan_num]; // PWM_MODULE_2
       else if (module_id == TIMER_3)
               tmpwmcblk = (TMRPWM_CHANNEL_BLK*) &_g_tmpwm_mod_3_channels [chan_num]; // PWM_MODULE_3
       else if (module_id == TIMER_4)
               tmpwmcblk = (TMRPWM_CHANNEL_BLK*) &_g_tmpwm_mod_4_channels [chan_num]; // PWM_MODULE_4
               else return (ERR_TIMER_MOD_NOT_SUPPORTED);           // TB1 not on FR6989

    if (flags & TIMER_CCR_TIMER_ONLY)
       ;      // do not turn on GPIOs if TIMER_ONLY mode
      else
       {      // User wants GPIOs turned on - do it
         if (tmpwmcblk->pwm_gpio_port_id != 0L)
            {    // skip Timers with no GPIOs (TA2, TA3)
              gpio_base_addr = _g_gpio_port_base_address[tmpwmcblk->pwm_gpio_port_id];
                 // set associated SEL bits to denote this is being used
                 // as a PWM channel
              bit_mask = tmpwmcblk->pwm_gpio_pin_bit;

              if (tmpwmcblk->pwm_even)
                 bit_mask <<= 8;                 // must shift by 8 if port is even
                   //---------------------------------------------
                   // Setup the associated DIR and SEL registers
                   // for this Timer/PWM OC/IC channel.
                   //---------------------------------------------
              HWREG16(gpio_base_addr + OFS_PADIR)  |= bit_mask;    // set for Output
              HWREG16(gpio_base_addr + OFS_PASEL0) |= bit_mask;    // Set for PRIMARY
              HWREG16(gpio_base_addr + OFS_PASEL1) &= ~(bit_mask); //    Periph Function
            }
       }

       //-------------------------------------------
       // do any needed pre-scaling on duty_cycle
       //-------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0)
       { initial_duty = (initial_duty / _g_tmpwm_prescalars[module_id]);
       }

    chan_mask = _g_pwm_chan_mask [chan_num];  // get associated channel mask

       //----------------------------------------------------------------
       // Get the OC OUTMOD value to be used for this channel. Then turn
       // on any needed polarity or interrupt flags for this channel/CCR
       //----------------------------------------------------------------
    ccr_ctl_value = _g_OUTMOD_mask_table [mode];   // get associated OUTMOD bit settings
    if (ccr_ctl_value >= OUTMOD_6)
       {     // denote that this channel is running as a PWM on this module.
             // turn associated mask bit on for this channel in PWM status flags.
         _g_tmpwm_channels_pwm [module_id] |= chan_mask;
       }
    if ((flags & TIMER_PIN_POLARITY_LOW) == 0)
       ccr_ctl_value |= OUT;         // invert pin's polarity
    if (flags & TIMER_ENABLE_CCR_INTERRUPTS)
       { ccr_ctl_value |= CCIE;      // enable interrupt when CCR value is hit
         rupt_mask = _g_CCR_interrupt_mask [chan_num];
       }
      else rupt_mask = 0;

       //---------------------------------------------------------
       // for certain OC modes (e.g. OCMODE_4, ...), we need to
       // reload the duty value after every interrupt.
       //---------------------------------------------------------
    if (mode == TIMER_MODE_OC_TOGGLE || mode == TIMER_MODE_OC_RESET)
       reload_value = initial_duty;
       else reload_value  = 0;

          //------------------------------------------------
          // Setup the associated TxCCTLx and TxCCRx registers
          // for this channel.
          //------------------------------------------------
    HWREG16(timercctl) = ccr_ctl_value; // config CCTLx OC bits (OUTMOD, CCIE, ...)
    HWREG16(timerccr)  = initial_duty;  // set CCRx duty value

          //------------------------------------------------
          // save what interrupts the User App wants to see
          //------------------------------------------------
    _g_timer_app_enabled_interrupts [module_id] |= rupt_mask;

    _g_tmpwm_channels_config [module_id] |= chan_mask; // denote channel was configured

    return (0);               // denote successful configuration
}


//*****************************************************************************
//  board_timerpwm_config_trigger_mode
//
//         Sets up timer to provide Trigger Output (TRGO), which is used
//         to trigger ADCs.
//
//         The trigger field is specified in the ADC12SHSxx bits of the
//         ADC14CTL0 register.
//         For the MSP430-FR6989 these are:
//
//                 Timer/ CCR      SHS Value    (from MSP430-FR6989 datasheet)
//                  TA0 / CCR1        1
//                  TB0 / CCR0        2
//                  TB0 / CCR1        3
//                  TA1 / CCR1        4
//                  TA2 / CCR1        5
//                  TA3 / CCR1        6
//
// flags parm can have TIMER_TRIGGER_MODE set, indicating to ensure that the
//       CCR OUTMOD is set to some kind of toggle (MOD_3 / MOD_6/ MOD_7) state.
//*****************************************************************************
int  board_timerpwm_config_trigger_mode (int module_id, int trigger_type,
                                         int flags)
{
    int       trig_timer_mod_id;
    int       channel_num;
    int       chan_mask;
    uint16_t  timerbase;
    uint16_t  timercctl;
    uint16_t  timerccr;
    int       rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (trigger_type != ADC_TRIGGER_USER_APP)
       {      //-------------------------------------------------------------
              // Setup SHS trigger field in ADC12CTL0
              //-------------------------------------------------------------
          if (trigger_type <= 0 || trigger_type > 7)
             return (ERR_TIMER_INVALID_TRIGGER_TYPE);   // bail out - invalid Trigger
          rc = board_adc_config_trigger_mode (trigger_type, flags);
          if (rc != 0)
             return (rc);   // must be a bad trigger_type

               //---------------------------------------------------------------
               // Then check if the associated Timer/CCR was already configured.
               // If not, auto-configure it here.
               //---------------------------------------------------------------
          channel_num       = _g_tmpwm_trigger_channel [trigger_type];
          trig_timer_mod_id = _g_tmpwm_trigger_timer [trigger_type];
             // ensure that the Trigger timer specified to the ADC matches this timer.
          if (trig_timer_mod_id  != module_id)
             return (ERR_TIMER_MOD_BAD_TRIGGER_TYPE);  // SNAFU. e.g. passed TRIG_TIMER_3 to a TIMER_1
          if (channel_num == TIMER_CHANNEL_0)
             chan_mask = TMR_CCR1_CONFIGURED;               // Channel 0
             else if (channel_num == TIMER_1)
                     chan_mask = TMR_CCR1_CONFIGURED;       // Channel 1
                     else chan_mask = TMR_CCR2_CONFIGURED;  // Channel 2

          if ((_g_tmpwm_channels_config [module_id] & chan_mask) == 0)
             {       // Associated CCR has not been configured. Set it up with default
                     // values. Set default CCR value = Timer's period (CCR0)
               board_timerpwm_config_channel (module_id, channel_num,
                                              ((long) board_timerpwm_get_period(module_id)),
                                              TIMER_MODE_OC_SET_RESET,
                                              (int) TIMER_CCR_TIMER_ONLY);
             }
       }

    return (0);                              // denote completed ok
}



//*****************************************************************************
// board_timerpwm_disable
//
//            Turns off a timer/pwm, and disables any interrupts for it.
//            (e.g. in preparation for Stopping or Reconfiguring a motor, ...)
//*****************************************************************************
int  board_timerpwm_disable (int module_id, int flags)
{
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

      //--------------------------------------------------------
      // Disable the associated TAx / TBx  module
      //--------------------------------------------------------
    HWREG16(timerbase + OFS_TAxCTL) &= ~(MC_3);      // Disable PWM counting

    return (0);                      // denote completed successfully
}


//*****************************************************************************
// board_timerpwm_enable
//
//         Enables and Starts the timer/PWM.
//
//         Sets up any desired Timer interrupt flags, then enables the Timer/PWM.
//         Saves optional callback function address to call when interrupt occurs.
//*****************************************************************************
int  board_timerpwm_enable (int module_id, int interrupt_flags)
{
    int        count_mode;
    uint16_t   timerbase;
    uint16_t   timercctl;
    uint16_t   timerccr;
    int        rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (_g_tmpwm_module_status [module_id] == TIMER_PERIODIC_COUNT_UP)
       count_mode = MC_1;                        // up mode
       else count_mode = MC_3;                   // up/down mode

       //--------------------------------------------------------------
       // CCR Interrupts: CCR 1-6's interupt flag CCIE was enabled in
       // board_timerpwm_config_channel().
       // Check if we need to enable CCR0' interrupt for ROLLOVERs.
       //--------------------------------------------------------------
    if (interrupt_flags & TIMER_ENABLE_ROLLOVER_INTERRUPTS)
       {    // turn on CCR0 rollover interrupts, and denote App is enabled for it
         HWREG16(timercctl) |= CCIE;
         _g_timer_app_enabled_interrupts [module_id] |= TAxIV_TAIFG;
       }

       //----------------------------------------------------------------
       // Enable the Timer/PWM counting, by turning on the MC bits in
       // the main TAxCTL register:  enable SMCLK + MC count + clear TAR
       //----------------------------------------------------------------
    HWREG16(timerbase + OFS_TAxCTL) = TASSEL__SMCLK | count_mode | TACLR;

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_enable_CCR_output
//
//        Configures and enables Timer Output Compare wave generation on the
//        requested channel, and its associated GPIO pin and CCR.
//
//    Parameters:
//        initial_duty - specifies the associated "duty cycle" for the pin's
//                       CCR register to be set to
//
//        action_flags:
//          - TIMER_ACTION_TOGGLE     toggle the OC GPIO output every time
//                                    the CCR value reached, 
//          - TIMER_ACTION_GO_ACTIVE  wait till CCR value reached, then drive 
//                                    the OC GPIO output high 
//                                    (and leave it there forever until reset)
//          - TIMER_ACTION_PULSE      similar to the above
//          - TIMER_ACTION_GO_INACTIVE  wait till CCR value reached, then drive
//                                    the OC GPIO output low 
//                                    (and leave it there forever until reset)
//          - TIMER_PIN_POLARITY_HIGH the GPIO pin goes high when CCR hit
//          - TIMER_PIN_POLARITY_LOW  the GPIO pin goes low when CCR hit (Inverted output)
//
//        interrupt_flags:
//          - TIMER_NO_INTERRUPT     no interrupts are signalled when CCR value
//                                   is reached.
//          - TIMER_ENABLE_CCR_INTERRUPTS   an interrupt is signalled when CCR 
//                                   value hit is reached. It will invoke the 
//                                   callback_function that was provided on the
//                                   board_timerpwm_enable() call. It will pass
//                                   to the callback function the callback_parm
//                                   and a TIMER_xxxx_INTERRUPT flag.
//                                   
//*****************************************************************************
int  board_timerpwm_enable_CCR_output (int module_id, int chan_num, long initial_duty,
                                       int action_flags, int interrupt_flags)
{
    int           mode;
    int           flags;
    int           rc;

       //----------------------------------------------------------------------
       // Call board_timerrpwm_config_channel() to perform baseline functions:
       // i.e. Setup associated GPIO(s) into Timer/PWM mode and put
       // duty cycle into CCR.
       //----------------------------------------------------------------------
    mode  = action_flags & 0x0F;       // get low  order bits timer mode control
    flags = action_flags & 0xF0;       // get high order bits polarity and interrupts
    rc = board_timerpwm_config_channel (module_id, chan_num,
                                        initial_duty, mode, flags);

       // on MSP432 and MSP430, the board_timerpwm_config_channel() routine is able
       // to perform all the work needed by board_timer_enable_CCR_output().
       // On other platforms, additional logic (NVIC setup, etc) would
       // be performed in this routine.

    return (rc);      // we are done, return results to caller
}


//*****************************************************************************
//  board_timerpwm_get_current_value
//
//         Gets the current count value (CNT) in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_current_value (int module_id)
{
    uint32_t     scaledcount;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    scaledcount = HWREG16(timerbase + OFS_TAxR);       // get current Count


// ??? probably have to re-divide period based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;  // get Master PWM Clock value
//  scaledcount = (_g_PWM_IO_clock / period_val) - 1;    // set new scaled period


    return (scaledcount);
}


//*****************************************************************************
//  board_timerpwm_get_CCR_capture_value
//
//         Gets the recorded input capture value in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_CCR_capture_value (int module_id, int chan_id)
{
    long         cap_value;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, chan_id, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    cap_value = HWREG16(timerccr);

// ??? !!!  NEED  TO  SCALE  IT   !!!???

// ??? !!!  WVD  must djust the value based on pre-scalar used !!!

    return (cap_value);             // passback capture value
}



//*****************************************************************************
//  board_timerpwm_get_duty_cycle
//
//         Get the Timer/PWM's current duty cycle.     Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_duty_cycle (int module_id, int chan_id)
{
    uint32_t     scaleduty;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, chan_id, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

      //--------------------------------------------------------
      // Read the desired CCR channel in the TAx / TBx module
      //--------------------------------------------------------
    scaleduty = HWREG16(timerccr);

/// ??? !!!  MUST  SCALE  IT    !!! ???   WVD

    return (scaleduty);
}


//*****************************************************************************
//  board_timerpwm_get_period
//
//         Get current Timer/PWM Period.         Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_period (int module_id)
{
    uint32_t     scaledperiod;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    scaledperiod = HWREG16(timerccr);    // period = CCR0 value

// ??? !!!   SCALE THIS   !!! ???   WVD

    return (scaledperiod);
}


//*****************************************************************************
//  board_timerpwm_lookup
//
//        Looks up a Timer/PWM module entry, verifies the module_id and 
//        channel_id are valid, and then returns timer base address, as well
//        as the address of the associated TxCCTLx control register address,
//        and the TxCCRx compare/capture register address.
//
//        These are then used to directly manipulate the registers as needed
//        by the operation being performed.
//*****************************************************************************

int  board_timerpwm_lookup (int module_id, int channel_id, uint16_t *timerbase,
                            uint16_t *timercctl, uint16_t *timerccr)
{
    TMRPWM_BASE_ADDR  *tmrbasep;

    if ( module_id < 0  ||  module_id > TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

       // If we ever do aliasing on channel_id, will need to do a reverse lookup to find base channel num
    if (channel_id < 0  ||  channel_id > TMRPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

       //----------------------------------------------------------------
       // based on module_id and channel_id, pass back timer base address,
       // timer CCTLx address, and CCRx address to caller
       //----------------------------------------------------------------
    tmrbasep   = (TMRPWM_BASE_ADDR*) &_g_timerpwm_base_address [module_id];  // lookup timer module
    *timerbase = tmrbasep->tmrpwm_base_addr;
    *timercctl = tmrbasep->tmrpwm_cctlx_addr [channel_id];
    *timerccr  = (tmrbasep->tmrpwm_cctlx_addr[channel_id] + CCR_OFFSET);
    if (*timercctl == 0)
       return (ERR_TIMER_CHAN_NUM_NOT_SUPPORTED);  // Timer does not have that CCTL/CCR
    return (0);                                    // denote completed OK
}


//*****************************************************************************
//  board_timerpwm_reset_CCR_output
//
//        Re-Configures and shuts off Timer Output Compare wave generation on
//        the requested channel.
//
//        This is used to turn off a CCR GPIO pin that was triggerred to a
//        constant value (e.g. reset a one-shot, in preparation for another
//        action).
//
//        This function is mainly used for OC modes:
//          - TIMER_ACTION_GO_ACTIVE  which will drive the OC output high 
//                                    (and leave it there forever until reset)
//          - TIMER_ACTION_PULSE
//          - TIMER_ACTION_GO_INACTIVE  which will drive the OC output low 
//                                    (and leave it there forever until reset)
//
//        flags - future use
//*****************************************************************************
int  board_timerpwm_reset_CCR_output (int module_id, int chan_id, int flags)
{
    uint16_t     mode;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, chan_id, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    mode = OUTMOD_0;

    HWREG16(timercctl) &= ~(OUTMOD_7);   // clear out old OUTMOD setting
    HWREG16(timercctl) |= mode;      // set CCTL OC output to passthru (OUTMOD_0)

// ??? should I also reset to GPIO mode, or let caller do explicit call for that ?

    return (0);                      // denote worked successfully
}


//*****************************************************************************
//  board_timerpwm_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when the Timer's rollover or CCR interrupts occur.
//*****************************************************************************
int  board_timerpwm_set_callback (int module_id, 
                   TMR_CB_EVENT_HANDLER callback_function, void *callback_parm)
{
    if (module_id < 0  ||  module_id >= TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

       //------------------------------------------------------------
       // Save any CALLBACK info, for interrupt handlers
       //------------------------------------------------------------
    _g_ptimer_callback [module_id]      = callback_function;
    _g_ptimer_callback_parm [module_id] = callback_parm;

    return (0);                 // denote completed OK
}


//*****************************************************************************
//  board_timerpwm_set_duty_cycle
//
//        Set the Timer/PWM's duty cycle, aka
//        sets a CCR (compare count register) on the specified timer.
//
//        Scale it if necessary, based on any pre-scalars we had
//        to use on the PWM's base Clock.
//
//        This is used to set different "duty cycles" for wave form values
//        being generated by a Timer, e.g. CCR1 might generate a 25 % duty
//        cycle on channel 1 pin, 33 % on channel 2 pin, and 50 % on chan 3 pin.
//
//        The flags parms is used to specify if a CCR interrupt should be
//        generated when the CCR compare value is hit by the timer.
//*****************************************************************************

int  board_timerpwm_set_duty_cycle (int module_id, int chan_id, long duty_cycle,
                                    int flags)
{
    uint32_t     scaleduty;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, chan_id, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

      //--------------------------------------------------------
      // Update the desired CCR channel in the TAx / TBx module
      //--------------------------------------------------------
    HWREG16(timerccr) = duty_cycle;

    return (0);                  // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_set_period
//
//         Sets the timer/PWM to use a new period value.
//*****************************************************************************

int  board_timerpwm_set_period (int module_id, long period_val, int flags)
{
    uint32_t     scaledperiod;
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller


// ??? probably have to re-divide period based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;  // get Master PWM Clock value
//  scaledperiod = (_g_PWM_IO_clock / period_val) - 1;    // set new scaled period


    HWREG16(timerccr) = period_val;

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_set_dead_time
//
//         Set the PWM's dead time between complementary PWM
//         arrangements.  Use to avoid "shoot through" on the
//         output FETs when working with Power or Motor H-bridges.
//
//         Dead-time is not technically supported by TAx type timers as used
//         on the MSP432. Through hacks, CCR1 and CCR4 can be tricked up to
//         simulate a Dead-time. We leave that as an exercise for the
//         experienced user.
//*****************************************************************************

int  board_timerpwm_set_dead_time (int module_id, int rising_edge, int falling_edge)
{
// in future - could consider a Hack version, using CCR1/CCR4, but would require
//            specific restrictions on user of what channels allowed (CCR2/CCR3)

    return (ERR_PWM_MODULE_NO_DEADTIME);      // Only TDx supports Deadtime
}


//*****************************************************************************
//  board_timerpwm_set_phase
//
//         Set the PWM's phase between complementary PWM
//         arrangements.
//
//         Phase shifting is not technically supported by TAx type timers as
//         used on the MSP432.
//*****************************************************************************

int  board_timerpwm_set_phase (int module_id, int chan_num, long phase_offset)
{
   return (ERR_PWM_MODULE_NO_PHASE);  // MSP432 PWMs do NOT support
                                      // phase shifting. Only C2000 modules do
}


//*****************************************************************************
//  board_timerpwm_set_channel_output
//
//         Set polarity of channel output (start high or low)
//*****************************************************************************
int  board_timerpwm_set_channel_output (int module_id, int channel_num,
                                        int output_mode, int flags)
{
    uint16_t     timerbase;
    uint16_t     timercctl;
    uint16_t     timerccr;
    int          rc;

    rc = board_timerpwm_lookup (module_id, 0, &timerbase, &timercctl, &timerccr);
    if (rc != 0)
       return (rc);                      // return error to caller

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);


      // ??? is this supported on MSP432 - perhaps tweak TAxCCTLx = OUTMOD_
      //     to do set/reset instead of reset/set ?


//  return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    return (0);                // denote completed successfully
}

#endif                       // #if defined(USES_PWM) || defined(USES_TIMER) ...


#if defined(USES_TIMER)
                 // we only enable interrupts if TIMER mode is (also) specified.
                 // As a rule, PWMs do not normally generate interrupts.
//******************************************************************************
//                         TIMER / PWM     ISRs
//******************************************************************************

//******************************************************************************
// timer_common_IRQHandler
//
//         To minimize redundant logic in each IRQ handler, we just route every
//         Timer IRQ to here, passing in the relevant parms (TIMx base, ...).
//
//         Note: on ST processors, unused interrupt flags may be set on
//               even though we are not using them, or the app is ignoring them
//               for callbacks. So we have to first filter what interrupts we
//               are going to look at based on User App's interrupt_flag
//               settings on timer_Enable() and timer_Enable_CCR_Output(),
//               and ignore the rest.
//
//         Also note that for some OC_MODes (5, xxx), the CCR value needs to be
//         reloaded after every interrupt.
//******************************************************************************

void  timer_common_IRQHandler (uint32_t timer_base_addr, int tim_index,
                               int  timer_reload_array[], int rollover_rupt)
{
    uint32_t      rupt_status;
   register  int  app_rupts_mask;

       //--------------------------------------------------------------
       // get a local copy of what interrupts the User App wants
       // to be called back on. In optimized compilers, this should
       // get loaded into a local register.
       //--------------------------------------------------------------
    app_rupts_mask = _g_timer_app_enabled_interrupts [tim_index];

       //------------------------------------------------------------------
       // CAUTION:  FR6989 provides two separate interrupts for each Timer:
       //             - a TIMERx_A0_VECTOR which signifies CCR0 rollover
       //             - a TIMERx_A1_VECTOR which signifies CCR1-n reached.
       // KEY: the CCR0 rollover interruupt is _NOT_ reported in
       //      the TAIV register. Instead, it is in the TAxCTL as TAIFG.
       //      So CCR0 interrupts need to be handled differently.
       //------------------------------------------------------------------
    if (rollover_rupt)
       {      // process a CCR0 interrupt
         if (app_rupts_mask == TAxIV_TAIFG)
            {     // Handle UPDATE / rollover CCR0 interrupt
              HWREG16(timer_base_addr + OFS_TAxCCTL0)  &= ~CCIFG;  // Ack/Clear the interrupt
              if (timer_reload_array[0] != 0)
                 HWREG16(timer_base_addr + OFS_TAxCCR0) = timer_reload_array[0]; // reload value for next pass
              if (_g_ptimer_callback[tim_index] != 0L)
                 {
                   (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                                    TIMER_ROLLOVER_INTERRUPT);
                 }
            }
         HWREG16(timer_base_addr + OFS_TAxCTL) &= ~(TAIFG);   // turn off the interrupt
       }

    rupt_status = HWREG16(timer_base_addr + OFS_TAxIV);



// ??? !!!  IS THERE A MORE EFFICIENT WEAY OF DOING THIS   !!! ???  WVD   07/11/156


       //---------------------------------------------------------------------
       // We sequentially walk down each key interrupt type, so that if
       // multiple interrupts are signalled at the same time, we handle them.
       //---------------------------------------------------------------------
    if (rupt_status & TAxIV_TACCR1  &&  app_rupts_mask == TAxIV_TACCR1)
       {     // Handle CCR1 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL1)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[1] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR1) = timer_reload_array[1]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR1_INTERRUPT);
            }
       }
    if (rupt_status & TAxIV_TACCR1  &&  app_rupts_mask == TAxIV_TACCR2)
       {     // Handle CCR2 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL2)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[2] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR2) = timer_reload_array[2]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR2_INTERRUPT);
            }
       }
    if (rupt_status & TAxIV_TACCR3  &&  app_rupts_mask == TAxIV_TACCR3)
       {     // Handle CCR3 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL3)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[3] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR3) = timer_reload_array[3]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR3_INTERRUPT);
            }
       }
    if (rupt_status & TAxIV_TACCR4  &&  app_rupts_mask == TAxIV_TACCR4)
       {     // Handle CCR4 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL4)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[4] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR4) = timer_reload_array[4]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR4_INTERRUPT);
            }
       }
    if (rupt_status & TAxIV_TACCR5  &&  app_rupts_mask == TAxIV_TACCR5)
       {     // Handle CCR5 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL5)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[5] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR5) = timer_reload_array[5]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR5_INTERRUPT);
            }
       }
    if (rupt_status & TAxIV_TACCR6  &&  app_rupts_mask == TAxIV_TACCR6)
       {     // Handle CCR6 interrupt
         HWREG16(timer_base_addr + OFS_TAxCCTL6)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[6] != 0)
            HWREG16(timer_base_addr + OFS_TAxCCR6) = timer_reload_array[6]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR6_INTERRUPT);
            }
       }

        //----------------------------
        //  ??? WHAT ABOUT THE FOLLOWING ??? !!!
        //----------------------------
//  if (TA0CTL & TAIFG)
//     TA0CTL &= ~TAIFG;    // handle Timer overflow  ??  same as rollover ???

}

          //-------------------------------------------------------------
          // These handlers _must_ be defined in the startup file,
          //  e.g. msp432_startup_ccs.c
          //-------------------------------------------------------------
#pragma vector=TIMER0_A0_VECTOR
__interrupt void  TIMER_TA0_CCR0_ISR (void)     // Handle period rollover
{
    timer_common_IRQHandler (_g_timerpwm_base_address[0].tmrpwm_base_addr, 0,
                             _g_timer_A0_reload_value_array, 1);
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void  TIMER_TA0_CCR1_4_ISR (void)   // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timerpwm_base_address[0].tmrpwm_base_addr, 0,
                             _g_timer_A0_reload_value_array, 0);
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void  TIMER_TA1_CCR0_ISR (void)     // Handle period rollover
{
    timer_common_IRQHandler (_g_timerpwm_base_address[1].tmrpwm_base_addr, 1,
                             _g_timer_A1_reload_value_array, 1);
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void  TIMER_TA1_CCR1_4_ISR (void)   // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timerpwm_base_address[1].tmrpwm_base_addr, 1,
                             _g_timer_A1_reload_value_array, 0);
}

#pragma vector=TIMER2_A1_VECTOR
__interrupt void  TIMER_TA2_CCR1_4_ISR (void)   // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timerpwm_base_address[2].tmrpwm_base_addr, 2,
                             _g_timer_A2_reload_value_array, 0);
}

#pragma vector=TIMER3_A0_VECTOR
__interrupt void  TIMER_TA3_CCR0_ISR (void)     // Handle period rollover
{
    timer_common_IRQHandler (_g_timerpwm_base_address[3].tmrpwm_base_addr, 3,
                             _g_timer_A3_reload_value_array, 1);
}

#pragma vector=TIMER3_A1_VECTOR
__interrupt void  TIMER_TA3_CCR1_4_ISR (void)   // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timerpwm_base_address[3].tmrpwm_base_addr, 3,
                             _g_timer_A3_reload_value_array, 0);
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void  TIMER_TB0_CCR0_ISR (void)     // Handle period rollover
{
    timer_common_IRQHandler (_g_timerpwm_base_address[4].tmrpwm_base_addr, 3,
                             _g_timer_B0_reload_value_array, 1);
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void  TIMER_TB0_CCR1_4_ISR (void)   // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timerpwm_base_address[4].tmrpwm_base_addr, 3,
                             _g_timer_B0_reload_value_array, 0);
}

#endif                         // defined(USES_TIMER)



//*****************************************************************************
//*****************************************************************************
//                               UART   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_uart_init
//
//               FR6989 Launchpad default _USB_ Console UART port is USCI_A1
//
//               The code is hardwired for 115,200 baud rate.
//               Use the lookup tables in the FR6989's TECH REF to select the
//               UCA1BR0 and UCBRS (UCA1MCTL) values for different Baud rates.
//
//               We support 3 different MCLK clock rates:
//                  1.000 Mhz (default startup), 8.0, and 16.0 MCLK
//*****************************************************************************

// make the eUSCI A UART module to operate with a 115200 baud rate when main clock = 3 MHz
//const EUSCI_A_UART_initParam  uartConfig =
//                 {
//                   EUSCI_A_UART_CLOCKSOURCE_SMCLK, // use SMCLK as Clock Source
//                   138,                            // BR     48MHz/115.2K = 138
//                    0,                             // UCxBRF = 0
//                    0,                             // UCxBRS = 0
//                   EUSCI_A_UART_NO_PARITY,         // No Parity
//                   EUSCI_A_UART_LSB_FIRST,         // MSB First
//                   EUSCI_A_UART_ONE_STOP_BIT,      // One stop bit
//                   EUSCI_A_UART_MODE,              // UART mode
//                   EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // No oversampling
//                 };

void  board_uart_init (void)
{

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)

    unsigned short  baud_UCBR0,   baud_CTLW;

        //----------------------------------------------------------------------
        // Generate Baud Rate pre-scalars, based on fixed clock rates we support
        //----------------------------------------------------------------------
     if (_g_SysClk_Ticks >= 16000000)
        {             // assume MCLK is set to 16.000 MHz
               baud_UCBR0 = 8;       // 16000000/115200 = 138.88  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0xF7, UCBRFx=10
               baud_CTLW = 0xF700 + UCBRF_10 + UCOS16;
        }
     else if (_g_SysClk_Ticks >= 8000000)
             {        // assume MCLK is set to 8.000 MHz
               baud_UCBR0 = 4;       // 8000000/115200 = 69.44  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0x55, UCBRFx=5
               baud_CTLW = 0x5500 + UCBRF_5 + UCOS16;
             }
     else    {        // assume MCLK is 1.000 MHz - the default startup clock
               baud_UCBR0 = 8;       // 1000000/115200 = 8.48
               baud_CTLW = 0xD600;  // Table: Modulation UCBRSx=0xD6, UCBRFx=0
             }

    PM5CTL0 &= ~LOCKLPM5;        // FRAM Enable GPIOs: disable the power-on
                                 // default setting GPIOs to high-impedance mode

        //-------------------------------------------------
        //  setup GPIO pins P2.0 and P2.1 to use for UART
        //-------------------------------------------------
        // Set GPIOs P2.0 (TX) and P2.1 (RX) into UART mode (routed to USB CDC)
//    GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P3,
//                                                GPIO_PIN4 | GPIO_PIN5,
//                                                GPIO_PRIMARY_MODULE_FUNCTION);

    P3SEL1 |=  (BIT4 + BIT5);   // set P3.4/P3.5 for use as USCI_A1 UART TXD/RXD
    P3SEL0 &=  ~(BIT4 + BIT5);

        //-------------------------------------------------
        //            Configure UART
        //-------------------------------------------------
//  EUSCI_A_UART_init (EUSCI_A1_BASE, &uartConfig);

        // Startup the UART module
//  EUSCI_A_UART_enable (EUSCI_A1_BASE);

    UCA1CTLW0 = UCSSEL__SMCLK + UCSWRST;  // Use SMCLK, 8N1, and turn on RESET

        //-------------------------------------------------
        //            setup  Baud  Rate
        //-------------------------------------------------
    UCA1BR0   = baud_UCBR0;               // set basic baud rate pre-scalar
    UCA1BR1   = 0;
    UCA1MCTLW = baud_CTLW;                // set baud Modulation and UCOS values

    UCA1CTLW0 &= ~UCSWRST;                // Enable and Initialize USCI UART
#endif

}


//*****************************************************************************
//  board_uart_rx_data_check                    aka   CONSOLE_CHECK_FOR_INPUT
//
//             Checks if any pending character input from UART.
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

int  board_uart_rx_data_check (void)
{
#if defined(USES_CONSOLE_READ)
    int  rc;

//  rc = UART_getInterruptStatus (EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA1IFG & UCRXIFG;
    if (rc)
       return (1);                        // we have some data

    return (0);                           // user has not typed in any char
#endif

}



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
    rc = UCA1IFG & UCRXIFG;
    if ( ! rc)
       return (0);                                 // no UART data is present

//  in_char = EUSCI_A_UART_receiveData (EUSCI_A1_BASE);  // read in char from UART
    in_char  = UCA1RXBUF;           // read in char from uART

#if defined(USES_CONSOLE_READ)
	// Check if a previous board_uart_read_string() had left a dangling \r\n situation.
    if (_g_uart_last_char == '\r'  &&  in_char == '\n')
        {      // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
           _g_uart_last_char = 0;   // clear out the \r, so we treat any
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
//*****************************************************************************

int  board_uart_read_string (char *read_buf, int buf_max_length)
{

#if defined(USES_CONSOLE_READ)
    char  in_char;
    int   rc;
                  //---------------------------------------------
                  // loop until we get \r, \n, or a full buffer
                  //---------------------------------------------
  while (1)
    {
           // read in any character that the user typed in
//  rc = UART_getInterruptStatus (EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA1IFG & UCRXIFG;
    if ( ! rc)
       continue;                            // user needs to type in more chars

//  in_char = EUSCI_A_UART_receiveData (EUSCI_A1_BASE);   // read in char from uART
    in_char  = UCA1RXBUF;                   // read in char from uART

    if (in_char <= 0)
       return (0);        // no character or error occurred. bail out

       // note: board_uart_read MUST ALSO echo back the char to user
    if (in_char != '\n')
       board_uart_write_char (in_char);      // echo the char (\n is special)
/// EUSCI_A_UART_transmitData (EUSCI_A1_BASE, in_char);   // echo the char
//  while ( ! (UCA1IFG & UCTXIFG))   ;       // wait till TX buffer is empty
//    UCA1TXBUF = in_char;                   // echo the rcvd char


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
         _g_uart_buf_idx = 0;          // reset for next pass
         _g_uart_last_char = '\r';     // save \r, to handle any \r\n sequence check
         board_uart_write_char ('\n');    // send a \n in anticipation,
                                          // otherwise the terminal can overwrite
                                          // the current line if the App issues a
                                          // CONSOLE_WRITE() before we rcv the \n
         return (1);                  // let user know a command is complete
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
           else board_uart_write_char ('\n');  // no preceding \r, so echo \n
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

    _g_uart_last_char = in_char;    // save it, to handle any \r\n sequence check
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
     }                                    //  end  while

  return (1);                      // denote string input is is complete
#endif

}


//******************************************************************************
//  board_uart_write_char                              aka   CONSOLE_WRITE_CHAR
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//******************************************************************************

void  board_uart_write_char (char outchar)
{

#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)
//  EUSCI_A_UART_transmitData (EUSCI_A1_BASE, outchar);   // send a char
    while ( ! (UCA1IFG & UCTXIFG))   ;       // wait till TX buffer is empty
       UCA1TXBUF  = outchar;                 // send a char
#endif

}


//******************************************************************************
//  board_uart_write_string                               aka   CONSOLE_WRITE
//                                                        or    DEBUG_LOG
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//******************************************************************************

void  board_uart_write_string (char *outstr)
{

#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)
    while (*outstr != '\0')
      {
//      EUSCI_A_UART_transmitData (EUSCI_A1_BASE, *outstr);   // send a char
        while ( ! (UCA1IFG & UCTXIFG))   ;       // wait till TX buffer is empty
           UCA1TXBUF  = *outstr;                 // send a char

        outstr++;                                // step to next char in buffer
      }
#endif

}



#if defined(USES_UNIQUEID) || defined(USES_MQTT)

//*****************************************************************************
//*****************************************************************************
//                          CRC  /  UNIQUE-ID    Routines
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

    _g_crcid_Result = 0;                     // clear the end result

    CRCINIRES = CRC_Init;                   // Init CRC engine with 0xFFFF
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
             CRCDIRB = crc_input;                 // Input data into CRC
             in_buf8++;                           // step to next entry
           }
       }
      else {    // co-erce/truncate 4 byte word values into "int"
             for (i=0; i < in_buf_length; i++)
               {
                    // Feed the user's "random values" into CRC Hardware
                 crc_input = (unsigned int) *in_buf32;
                 CRCDIRB = crc_input;             // Input data into CRC
                 in_buf32++;                      // step to next entry
               }
           }

    _g_crcid_Result = CRCINIRES;                   // all done, Save results

    return (_g_crcid_Result);
}

#endif              // defined(USES_MQTT) || defined(USES_UNIQUEID)



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

void  board_vtimer_check_expiration (uint32_t  current_systick_value)
{
    int   i,  expired;

          //-------------------------------------------------------------
          // See if any virtual timers have reached their expiration time
          //-------------------------------------------------------------
    for (i = 0; i < 10;  i++)
      {
        if (_g_vtimer_flags[i] == VTIMER_RESET)
           continue;                       // timer is not active, skip it

        if (_g_vtimer_expire [i] > current_systick_value)
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
          // wraps around, we depend upon the VTIMER timer expiration logic to handle it.
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

     _g_vtimer_flags [vtimer_id] = VTIMER_RESET;
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
//                          TIMER  SUPPORT
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
//---------------------------------------------------------------------
void  interval_timer_init (void)
{
    // WDT requires no additional configuration.
    // It is used in "Interval Timer" mode.
    // This routine is mainly for Tiva and C2000 implementations
}


//--------------------------------------------------------------------
// interval_timer_disable
//
//            Shut off the Interval Timer
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_disable (void)
{
    WDTCTL  = WDTPW | WDTHOLD;   // Stop Watchdog Timer
    SFRIE1 &= ~WDTIE;            // Disable Watch dog interrupts

#if (DRIVERLIB)
       // Setup DCO to use internal resistor. DCO calibrated at 16.384 MHz
    CS_setupDCO (CS_INTERNAL_RESISTOR);

       // Configure interval to ACLK / 32768 = 1s
    WDT_intervalTimerInit (WDT_BASE,
                           WDT_CLOCKSOURCE_ACLK,
                           WDT_CLOCKDIVIDER_32K);
    WDT_resetTimer (WDT_BASE);

       // Start the WDT with interrupts enabled
    SFR_clearInterrupt (SFR_WATCHDOG_INTERRUPT);
    SFR_enableInterrupt (SFR_WATCHDOG_INTERRUPT);
    WDT_start (WDT_BASE);
#endif
}


//--------------------------------------------------------------------
// interval_timer_enable
//
//            Turn on the Interval Timer, used for Accel/Decel Updates
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_enable (void)
{
    WDTCTL  = WDT_MDLY_32;  // setup Watch dog as interval timer with 32ms duration
    SFRIE1 |= WDTIE;        // Enable Watch dog interrupt

//  HWREG16 (baseAddress + OFS_WDTCTL) =
//          WDTPW + WDTCNTCL + WDTHOLD + WDTTMSEL + clockSelect + clockDivider;

}


//------------------------------------------------------------------
//  timer_pwm_init
//
//         Initialize PWM mode
//
//         flags parm is for future use.
//
//  Tiva pin PA6 uses M1PWM2, which is Module 1 PWM 2.  (PWM2EN)
//  It is controlled by                Module 1 PWM, Generator 1.
//------------------------------------------------------------------

void  timer_pwm_init (long flags)
{
    // No special PWM startup processing required by MSP430.
    // This routine is mainly for Tiva and C2000 implementations
}


//-------------------------------------------------------------------
//  timer_config_pwm_mode
//
//         Turn on PWM mode, output goes to associated STEP1_AIN pin.
//         Used when want to drive BDC motors with PWM.       VERIFY
//
//         flags parm is for future use.
//
//         On xxxx, STEP1_AIN pin is connected to TB0.4 on pin P3.5
//         TB0.4 uses Timer TB0 CCR4
//-------------------------------------------------------------------

void  timer_config_pwm_mode (long flags)
{
       // Setup PWM Output mode and enable Interrupts
    TB0CCTL0 = CCIE;             // enable TBCCR0 interrupt
    TB0CCTL4 = OUTMOD_3 | CCIE;  // We use CCR4 to drive PWM output
}


//------------------------------------------------------------------
//  timer_config_period_duty_cycle
//
//         Set the PWM Period, and starting duty cycle.
//------------------------------------------------------------------

void  timer_config_period_duty_cycle (long period, long duty_cycle)
{
    TB0CCR0 = period;        // set PWM period into timer base
    TB0CCR4 = duty_cycle;    // set starting duty cycle into CCR.
                             // We use CCR4 to drive PWM output
}


//-------------------------------------------------------------------
// timer_disable_pwm
//
//            Disable the Speed or PWM timer.
//            (in preparation for Stopping or Reconfiguring the motor)
//--------------------------------------------------------------------
void  timer_disable_pwm (void)
{
    TB0CTL = TBSSEL_2 | MC_0 | TBCLR;     // turn off the timer
}


//-------------------------------------------------------------------
// timer_enable_pwm
//
//            Enable and Start the timer in Speed or PWM mode.
//--------------------------------------------------------------------
void  timer_enable_pwm (void)
{
    TB0CTL = TBSSEL_2 | MC_1 | TBCLR;     // start up the timer
}



//--------------------------------------------------------------------
//  registerInterruptHandler
//
//         register an interrupt handler for the host IRQ line (CC3100, ...)
//
//         param[in]  InterruptHdl    -    pointer to interrupt handler function
//         param[in]  pValue          -    pointer to a memory strcuture that is
//                    passed to the interrupt handler.
//         return     upon successful registration, the function shall return 0.
//                    Otherwise, -1 shall be returned
//         note       If there is already registered interrupt handler, the
//                    function should overwrite the old handler with the new one
//--------------------------------------------------------------------
int  registerInterruptHandler (P_EVENT_HANDLER InterruptHdl, void *pValue)
{
    pIraEventHandler = InterruptHdl;
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
//--------------------------------------------------------------------
void  CC3100_disable()
{
    NHIB_CC3100_DISABLE();            // set CC3100 nHIB to DISABLED
}

//------------------------------------------------------------------------
// CC3100_enable
//
//            Enables the CC3100        (nHIB/ENABLE line)
//
//            Called by: sl_Start() in simplelink device.c 
//
// This KEY routine is called by the simplelink sl_Start() routine,
// after it has setup the IRQ (PORT1/2) interrupt handler's address.
//
// Enabling the CC3100 kicks off its internal "startup" logic.
// It takes about 50 milliseconds to complete.
// When the CC3100 startup completes, it will toggle an IRQ interrupt.
// The (PORT1/2) IRQ handler will then read in the CC3100 status and
// signal "startup complete" to the sl_Start() routine.
//
// Calling this routine too early (before IRQ handler address is setup)
// will cause severe interrupt looping, continously calling the ISR.
//
// Failing to have the MCU Port1/2 interrupt enabled (and general MCU/NVIC)
// interrupts enabled) will cause the sl_Start() logic to loop forever,
// waiting for "startup complete".
//
// Note that the CC3100 startup logic is _extremely_ timing sensitive,
// such that even tweaking the MCU clock rate can cause "startup hangs"
// because the IRQ interrupt gets missed or not manifested.
//--------------------------------------------------------------------------
void  CC3100_enable()
{
    NHIB_CC3100_ENABLE() ;            // set CC3100 nHIB to ENABLED
}


//-------------------------------------------------------------------
// CC3100_InterruptEnable
//
//            Enables the interrupt (IRQ) from the CC3100 on P 2.1
//--------------------------------------------------------------------
void  CC3100_InterruptEnable()
{
    IRQ_CC3100_ENABLE_INTERRUPTS();
}


//-------------------------------------------------------------------
// CC3100_InterruptDisable
//
//            Disables the interrupt (IRQ) from the CC3100 on P 1.2
//--------------------------------------------------------------------
void  CC3100_InterruptDisable()
{
    IRQ_CC3100_DISABLE_INTERRUPTS();
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
//                      CC3100   IRQ   ISR   Handler           P2.1
//-------------------------------------------------------------------

#pragma vector=PORT2_VECTOR
__interrupt void  Port2_ISR (void)
{
    volatile int   other_bits;

    switch (__even_in_range(P2IV, P2IV_P2IFG7))
      {
                  // Vector  P2IV_P2IFG1:  P2IV P2IFG.1  -  CC3100 IRQ is on P 2.1
        case  P2IV_P2IFG1:
 num_CC3100_irq_rupts++;                  // we got an interupt from CC3100 on P 2.1
            if (pIraEventHandler)
               {
                 pIraEventHandler (0);    // call CC3100 handler
               }
            break;

                  // Default case
        default:
            other_bits = P1IV;            // snapshot what other rupts came in
            break;
      }
}

#endif                            // #if defined(USES_CC3100)



#if defined(USES_DRV8711)
//**************************************************************************
//**************************************************************************
//
//                 BOARD   -   DRV8711  Specific   Routines    Stepper
//
//**************************************************************************
//**************************************************************************

//--------------------------------------------------------------------
// adc_init
//
//            Initialize the ADC connected to the Potentiometer.
//---------------------------------------------------------------------
void  adc_init (void)
{
       // Setup ADC pin for Potentiometer
// TBD
}


//--------------------------------------------------------------------
// adc_start_sample
//
//            Initiate a sample on the Potentiometer ADC
//---------------------------------------------------------------------
void  adc_start_sample (void)
{
//  ADCProcessorTrigger (ADC0_BASE, 3);  // Trigger the ADC conversion
// TBD
}


//--------------------------------------------------------------------
// adc_check_if_completed
//
//            Tests if the ADC sample conversion is complete.
//            Returns 1 if completed, or 0 if still busy
//---------------------------------------------------------------------
int  adc_check_if_completed (void)
{
//  if ( ! ADCIntStatus(ADC0_BASE, 3, false))
//     return (0);                   // Denote ADC is still busy
//  ADCSequenceDataGet (ADC0_BASE, 3, &_g_ADC_value); //read converted value
//  ADCIntClear (ADC0_BASE, 3);      // clear interrupt flag for next pass
//  return (1);                      // Denote ADC has completed
// TBD
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


//-------------------------------------------------------------------
//  gpio_force_all_control_pins_low
//
//        Force all DRV8711 pins LOW.
//-------------------------------------------------------------------
void  gpio_force_all_control_pins_low (void)
{
    P3OUT  &= ~(STEP_AIN1 | DIR_AIN2);  // Set output LOW
    P1OUT  &= ~(BIN2 | BIN1);
}


//-------------------------------------------------------------------
//  gpio_quiesce_all_control_pins
//
//        Configure Pins for IN/IN Control and Set All Outputs Low
//        Force all DRV8711 pins back to GPIO mode, and set them LOW.
//-------------------------------------------------------------------
void  gpio_quiesce_all_control_pins (void)
{
    P3SEL0 &= ~(STEP_AIN1 | DIR_AIN2);  // Ensure reset to GPIO mode
    P3SEL1 &= ~(STEP_AIN1 | DIR_AIN2);
    P1SEL0 &= ~(BIN2 | BIN1);
    P1SEL1 &= ~(BIN2 | BIN1);

    P3OUT  &= ~(STEP_AIN1 | DIR_AIN2);  // Set output LOW
    P1OUT  &= ~(BIN2 | BIN1);

    P3DIR  |=  (STEP_AIN1 | DIR_AIN2);  // ensure direction = OUT
    P1DIR  |=  (BIN2 | BIN1);
}


//-------------------------------------------------------------------
// timer_force_pwm_pin_low
//
//            Forcibly turn off PWM mode, and force output pin low
//
//            pin_num parm is for future use.
//--------------------------------------------------------------------
void  timer_force_pwm_pin_low (short pin_num)
{
    TB0CCTL1        = OUTMOD_0;    // Turn off Timer PWM mode
    STEP_AIN1_PORT &= ~STEP_AIN1;  // force timer output pin LOW
}


//------------------------------------------------------------------
//  timer_set_pin_for_pwm
//
//         Setup GPIO pins and associated "Alternate Function" bits
//         to run the pin(s) as PWM outputs from the Timer
//------------------------------------------------------------------

void  timer_set_pin_for_pwm (short pin_num)
{
       // Setup pin(s) for Timer PWM Output
    P3DIR  |= STEP_AIN1;    // set to Output mode

    P3SEL1 &= ~STEP_AIN1;   // Route Timer1 CCR4 output (PWM) to pin
    P3SEL0 |= STEP_AIN1;    // SEL1/SEL0 = 01
}


//------------------------------------------------------------------
//  timer_set_pin_for_gpio
//
//         Setup GPIO pins and associated "Alternate Function" bits
//         to run the pin(s) as standard GPIO pins  (turn off PWM)
//------------------------------------------------------------------

void  timer_set_pin_for_gpio (short pin_num)
{
       // Re-configure pin(s) to operate as normal GPIO
    P3OUT  &= ~STEP_AIN1;   // set Output = Low

    P3DIR  |= STEP_AIN1;    // set to Output mode

    P3SEL1 &= ~STEP_AIN1;   // Disable Timer PWM output. Reset to std GPIO
    P3SEL0 &= ~STEP_AIN1;
}


/****************************Interrupt Service Routines*****************************/

//---------------------------------------------------------------
//                       ISR   Timer1 - A0
//
// This Timer (PWM) interrupt pops for the following event:
//      - PWM end of period: This is used to roll in the new
//        Period and Duty cycle to be used for the next cycle.
//        In general, the period remains the same, and
//        only the duty cycle changes, in reponse to ramp up
//        or ramp down situations.
//        At constant speeds, the duty cycle is 50 %.
//---------------------------------------------------------------

#pragma  vector=TIMER0_B1_VECTOR
__interrupt void  Timer0_B1_ISR (void)
{
       // Update Timer PWM period and duty cycle at End of a PWM Period
    if (G_LOAD_CCR_VALS == true)
       {
         G_CUR_SPEED = G_CUR_SPEED_TEMP;
         TB0CCR0     = G_TA1CCR0_TEMP;   // set (new) Period
         TB0CCR4     = G_TA1CCR1_TEMP;   // set new Duty Cycle CCR
         G_LOAD_CCR_VALS = false;
       }
}


//---------------------------------------------------------------
//                      ISR   Timer0 - B0
//
// This Timer (PWM) interrupt pops for the following event:
//      - CCR match/completion: This is used to increment the
//        Step Counter, to count the number of steps (PWM pulses)
//        that have been issued to the Stepper motor.
//---------------------------------------------------------------
#pragma vector= TIMER0_B0_VECTOR
__interrupt void  Timer0_B0_ISR (void)
{
    switch (TB0IV)
    {
        case TB0IV_NONE: break;         // Vector 0: No Interrupt

               //-----------------------------------------
               // CCR1 (Duty Cycle) completion interrupt
               //-----------------------------------------
        case TB0IV_TBCCR1:              // Vector 2: CCR1 CCIFG
          {
                // Increment Step Counter
            G_CUR_NUM_STEPS++;
            if (G_CUR_NUM_STEPS == G_TOTAL_NUM_STEPS)
               {
//               __bic_SR_register_on_exit (LPM0_bits);
               }
            TB0CCTL1 &= ~CCIFG;         // Clear interrupt flag
            break;
          }

        case TB0IV_TBCCR2:              // Vector 4: CCR2 CCIFG
          {
            TB0CCTL2 &= ~CCIFG;
            break;
          }

        case TB0IV_TBCCR4:              // Vector 4: CCR2 CCIFG
          {
            G_CUR_NUM_STEPS++;
            if (G_CUR_NUM_STEPS == G_TOTAL_NUM_STEPS)
               {
//               __bic_SR_register_on_exit (LPM0_bits);
               }
            TB0CCTL4 &= ~CCIFG;         // Clear interrupt flag
            break;
          }

               //-----------------------------------------
               // CCR0 (Period) completion interrupt   ???
               //-----------------------------------------
///     case TB0IV_6: break;            // Vector 6: Reserved CCIFG
///     case TB0IV_8: break;            // Vector 8: Reserved CCIFG - Not used FR6989

        case TB0IV_TBIFG:               // Vector 10: Overflow
          {
            TB0CTL &= ~TBIFG;           // Clear interrupt flag
            break;
          }

        default: break;
    }
}


//---------------------------------------------------------------
//                 ISR HANDLER  -  Interval Timer mode WDT
//
// This pops every 32 milli-seconds, and is used to process
// the acceleration/deceleration ramp.
// It sets the flag for the mainline to compute the next speed
// step, ir a ramp up or ramp down is occurring. If just
// a constant speed is being performed, the speed stays as is.
//---------------------------------------------------------------
#pragma vector=WDT_VECTOR
__interrupt void  WatchDog_Timer (void)
{
       // Signal Main Thread to Calculate Next Speed Value
    G_ACCEL_FLAG = true;

       // Wake Up the Main Thread
//  __bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector=PORT1_VECTOR
__interrupt void  Trap_ISR2 (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}

#endif                            // #if defined(USES_DRV8711)


//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   ISRs
//
//**************************************************************************
//**************************************************************************

//---------------------------------------------------------------
//                     Un-assigned ISRs
//---------------------------------------------------------------
//      PORT2_VECTOR,
//      TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, \
//
#pragma vector=ADC10_VECTOR, USCIAB0TX_VECTOR, \
        COMPARATORA_VECTOR, NMI_VECTOR
__interrupt void  Trap_ISR (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}

