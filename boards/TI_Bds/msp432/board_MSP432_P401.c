// 07/09/15 - Verify that ADC14ENC is being set ON, even if Timer Triggers are being used !!
//            STILL NO JOY.   ==> build a standalone version like did with G2553 and FR6989

// 06/28/15 - Need to get ADC Triggering working

// 06/28/15 - Need to get generic Timers and OC mode working

// 05/16/15 - Need to get ADC interrupts working.  Rest of ADC logic now works !!!

// 04/17/15 Status:  MCLK at 48 MHz:  TCP_Client, TCP_Server, MQTT_Client,
//                   and Console are all running successfully !  YAH !

// 04/17/15 - BUT, need to add selectable MCLK support for:  3 MHz (startup), 12 MHz (0 wait), 
//                 24 MHz (1 wait), and 48 MHz (2 waits)

//*******1*********2*********3*********4*********5*********6*********7**********
//                           board_MSP432_P401.c
//
// Board dependent code, moved to a single module, in prep for ARM based MSP432
//
// Chip specifics:  MSP432P401RIPZ      256K Flash       64K RAM       48MHz max
//                  4 SPI, 4 I2C        4 TA Timers (0 + 6 CCRs)   8 Comparators
//                  24 ADCs channels    Systick Timer
//
// MSP432 Launchpad brings out timer TA0 and TA2 pins to std 40 pin headers. 
// TA1 and TA3 timer pins are only available via separate headers at bottom of board.
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
//   03/29/15 - Board arrived - Created initial version.
//   04/04/15 - Got working with TCP Client/Server test apps using CC3100.
//   04/24/15 - Upgraded to add ADC "Multiple Sequenced Conversions" support. Duq
//   04/26/15 - Fixed config issue that caused timing problems with "Sequenced
//              Conversions". D Dob
//   05/16/15 - Removed dependency on DriverLib for ADC channel config. WORKS. Duqu
//   06/29/15 - Merged latest PWM/Timer changes in for consistency across
//              platforms, and to provide added features (OC_Mode). Duqu
//   06/30/15 - Rolled in DAC support, using external 8-channel SPI-based
//              DAC128S085 EVM/Boosterpack. Duqu
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
#include <math.h>

#include <gpio.h>              // ensure have DriverLib GPIO defs

//#include "msp432/spi.h"

#if defined(USES_CC3100)
#include "simplelink.h"        // pull in defs for CC3100 SimpleLink
spi_Open (char *ifName, unsigned long flags);  // Use TI SPI driver for now
void  Port2_ISR (void);                        // ISR for CC3100 IRQ pin
typedef enum
   {
       ANT1,
       ANT2
   } antenna;
#endif

#if defined(USES_DRV8711)
#include "devices/DRV8711_Spin_Routines.h"
#endif

// move the following to boarddef.h at some point
void board_system_clock_config (long mcu_clock_hz);                    // subroutine prototypes
int  board_dac_check_conversions_done (int dac_module_id, int sequencer);
int  board_dac_clear_conversions_done (int dac_module_id, int sequencer);
int  board_dac_SineWave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_Trianglewave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_SquareWave  (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int  board_timerpwm_enable_clock (int module_id);
long board_timerpwm_compute_prescalar (long period_val);
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id);
void board_error_handler (void);
int  board_adc_config_trigger_mode (int trigger_type, int flags);


//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    P_EVENT_HANDLER   pIraEventHandler = 0;
    uint8_t           IntIsMasked;

    uint32_t  _g_SysClk_Ticks;          // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_systick_millisecs = 0; //  current SYSTICK time in ticks

    char      _g_vtimers_active    = 0; // Optional VTIMER support

    char      _g_pwm_mclock_configured = 0; // PWM master clock was configured
    uint32_t  _g_pwm_mdivider = 1;          // PWM Master clock : CPU divider ratio

    uint32_t  _g_MCLK     = 0;          // CPU MCLK setting
    uint32_t  _g_SMCLK    = 0;          // peripheral SMCLK setting
    uint32_t  _g_ACLK     = 0;          // low speed  ACLK setting


                 // mainly for DRV8711 logic
    uint32_t  g_ADC_value = 0;


#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    char   _g_uart_buf_idx   = 0;        // keeps track of user input via UART
    char   _g_uart_last_char = 0;
#endif

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    unsigned long   _g_crcid_Result;
#endif

               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
const uint32_t  _g_gpio_port_base_address [] =
            {   0x00,
                0x40004C00,    // Port 1
                0x40004C01,    // Port 2
                0x40004C20,    // Port 3
                0x40004C21,    // Port 4
                0x40004C40,    // Port 5
                0x40004C41,    // Port 6
                0x40004C60,    // Port 7
                0x40004C61,    // Port 8
                0x40004C80,    // Port 9
                0x40004C81,    // Port 10
                0x40004D20     // Port J
            };
#define  GP_PORT_1    1        // indexes into above table, based on port id
#define  GP_PORT_2    2
#define  GP_PORT_3    3
#define  GP_PORT_4    4
#define  GP_PORT_5    5
#define  GP_PORT_6    6
#define  GP_PORT_7    7
#define  GP_PORT_8    8
#define  GP_PORT_9    9
#define  GP_PORT_10  10
#define  GP_PORT_J   11


const uint32_t  _g_timer_base_address [] =
            {   0x40000000,    // TIMER_A0  TA0
                0x40000400,    // TIMER_A1  TA1
                0x40000800,    // TIMER_A2  TA2
                0x40000C00,    // TIMER_A3  TA3
            };



void  ClearBufferRelatedParam (void)
{
     // dummy entry - not used
}


//******************************************************************************
//******************************************************************************
//
//                     BOARD   -   COMMON   Routines
//
//******************************************************************************
//******************************************************************************

//******************************************************************************
//  board_init
//
//           Initialize system clocks, and basic GPIOs
//******************************************************************************

void  board_init (long mcu_clock_rate)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (MCU_DEFAULT_SPEED); // use default

       //----------------------------------------------------------
       //   Configure basic peripherals needed for W5200 / CC3000
       //----------------------------------------------------------
    board_gpio_init();              // turn on key GPIO clocks, ...

#if defined(USES_SYSTICK) || defined(USES_MQTT) || defined(USES_ADC) || defined(USES_VTIMER)
    board_systick_timer_config();   // ensure "Systick" timer is turned on
#endif

        // Enable the UART if user wants to invoke CONSOLE or DEBUG_LOG calls
//#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    board_uart_init();              // go ahead and enable the default UART
//#endif

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

// ??? NEEDS TWEAKING ???   assumes takes 4 cycles per loop at 48 MHz
   for (i = 0; i < usec_delay; i++)
      __delay_cycles (8);     // requires a constant !
}


//*****************************************************************************
// board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************
void  board_delay_ms (long  ms_delay)
{
    uint32_t  tick_begin,   tick_endtime,    i;

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
//      If the result value would be too large, PWM/Timer code would have
//      to further sub-divide it using pre-scalars
//*****************************************************************************

long  board_frequency_to_period_ticks (long frequency)
{
    long   pticks;

        // take the current I/O clock frequency, and compute the associated
        // ticks needed to generate that frequency.(e.g. for Timers, PWMs, ...)
     pticks = _g_SysClk_Ticks / frequency;

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

    int                _g_ADC_complete    = 0;   // 1 = all ADC conversions/DMAs are complete
    char               _g_active_channels = 0;
    char               _g_step_num        = 0;
    char               _g_adc_configured  = 0;
    char               _g_trigger_source  = 0;
    char               _g_toggle_ENC_required = 1;  // 1 = ADC12ENC bit must be toggled after every
                                                    // conversion set, per TECH REF

    int                _g_trigger_atmrpwm      = 0; // index to correct Trigger Timer/PWM
    uint16_t           _g_trigger_auser_api_id = 0; // User API id for the trigger
    uint16_t           _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
    uint32_t           _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

    unsigned char      _g_adc_step_map [32];       // indexed by channel number

    unsigned short     _g_adc_conv_results [32];   // Internal buffer to hold ADC DMAed results

 ADC_CB_EVENT_HANDLER  _g_adc_callback       = 0L;
    void               *_g_adc_callback_parm = 0;

    int                _g_DMA_complete   = 0;      // 1 = all ADC conversion DMAs are complete
    char               _g_DMA_overrun    = 0;      // 1 = DMA completed a new set before the previous one was processed
    unsigned long      dma_callback_seen = 0;      // DEBUG COUNTERs
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


//#if (TO_DO)
      //-----------------------------------------------------------------------
      // table of addresses of ADC sequencing step registers: ADC14MCTL 0 - 17
      //-----------------------------------------------------------------------
 volatile uint32_t * _g_adc_seqstep_regs[] =   //&ADC14->rMCTL0.r, &ADC14->rMCTL1.r, &ADC14->rMCTL2.r, &ADC14->rMCTL3.r,
                { &ADC14->rMCTL0.r, &ADC14->rMCTL1.r,   // aka ADC14MCTL0,  ADC14MCTL1,
                  &ADC14->rMCTL2.r, &ADC14->rMCTL3.r,   // aka ADC14MCTL2,  ADC14MCTL3,
                  &ADC14->rMCTL4.r, &ADC14->rMCTL5.r,   // aka ADC14MCTL4,  ADC14MCTL5,
                  &ADC14->rMCTL6.r, &ADC14->rMCTL7.r,   // aka ADC14MCTL6,  ADC14MCTL7,
                  &ADC14->rMCTL8.r, &ADC14->rMCTL9.r,   // aka ADC14MCTL8,  ADC14MCTL9,
                  &ADC14->rMCTL10.r, &ADC14->rMCTL11.r, // aka ADC14MCTL10, ADC14MCTL11,
                  &ADC14->rMCTL12.r, &ADC14->rMCTL13.r, // aka ADC14MCTL12, ADC14MCTL13,
                  &ADC14->rMCTL14.r, &ADC14->rMCTL15.r, // aka ADC14MCTL14, ADC14MCTL15,
                  &ADC14->rMCTL16.r, &ADC14->rMCTL17.r, // aka ADC14MCTL16, ADC14MCTL17
                };
//#endif


typedef struct adc_channel_def         /* ADC Channel definitions */
    {
        uint32_t  chan_gpio_port_id;   /* Associated GPIO port         */
//      uint32_t  chan_gpio_SEL1_port; /* Associated GPIO SEL1 port    */
//      uint32_t  chan_gpio_SEL0_port; /* Associated GPIO SEL0 port    */
        uint32_t  chan_gpio_pin_bit;   /* Associated GPIO pin          */
        uint32_t  chan_adc_num;        /* Associated ADC14INCH_x channel num */
    } ADC_CHANNEL_BLK;

               //--------------------------------------------------------------
               //                ADC  Trigger Mapping Table
               //--------------------------------------------------------------
               //-----------------------------------------------
               // Triggerable Timers/Events available on MSP432
               //   Timer/ CCR      SHS Value    (from MSP432 datasheet p 91)
               //     TA0 / CCR1        1
               //     TA0 / CCR2        2
               //     TA1 / CCR1        3
               //     TA1 / CCR2        4
               //     TA2 / CCR1        5
               //     TA2 / CCR1        6
               //     TA3 / CCR1        7
               //------------------------------------------------
const uint32_t         _g_adctrigger_shs_mask []
                               = { ADC14SHS_0,      // SHS 0 denotes no trigger
                                   ADC14SHS_1,      // TRIGGER_TIMER_0_CC1
                                   ADC14SHS_2,      // TRIGGER_TIMER_0_CC2
                                   ADC14SHS_3,      // TRIGGER_TIMER_1_CC1
                                   ADC14SHS_4,      // TRIGGER_TIMER_1_CC2
                                   ADC14SHS_5,      // TRIGGER_TIMER_2_CC1
                                   ADC14SHS_6,      // TRIGGER_TIMER_2_CC2
                                   ADC14SHS_7       // TRIGGER_TIMER_3_CC1
                                 };


               //--------------------------------------------------------------
               //                 ADC    GPIO  PIN     Mapping Table
               //--------------------------------------------------------------

const ADC_CHANNEL_BLK  _g_adc_channels [] =  //                Energia  LP    Grove
        {  { GP_PORT_5, BIT5, ADC14INCH_0  },  // P 5.5  Adc0   Pin30   J3-10
           { GP_PORT_5, BIT4, ADC14INCH_1  },  // P 5.4  Adc1   Pin29   J3-9
           {    0,      0,     0,       0  },  // - no channel 2 on MSP432 -
           { GP_PORT_5, BIT2, ADC14INCH_3  },  // P 5.2  Adc3   Pin12   J2-9
           { GP_PORT_5, BIT1, ADC14INCH_4  },  // P 5.1  Adc4   Pin33   J4-8
           { GP_PORT_5, BIT0, ADC14INCH_5  },  // P 5.0  Adc5   Pin13   J2-8
           { GP_PORT_4, BIT7, ADC14INCH_6  },  // P 4.7  Adc6   Pin28   J3-8
           { GP_PORT_4, BIT6, ADC14INCH_7  },  // P 4.6  Adc7   Pin8    J1-8
           { GP_PORT_4, BIT5, ADC14INCH_8  },  // P 4.5  Adc8   Pin27   J3-7   J9
           { GP_PORT_4, BIT4, ADC14INCH_9  },  // P 4.4  Adc9   Pin26   J3-6   J8
           { GP_PORT_4, BIT3, ADC14INCH_10 },  // P 4.3  Adc10  Pin6    J1-6
           { GP_PORT_4, BIT2, ADC14INCH_11 },  // P 4.2  Adc11  Pin25   J3-5   J7
           { GP_PORT_4, BIT1, ADC14INCH_12 },  // P 4.1  Adc12  Pin5    J1-5
           { GP_PORT_4, BIT0, ADC14INCH_13 },  // P 4.0  Adc13  Pin24   J3-4   J6
           { GP_PORT_6, BIT1, ADC14INCH_14 },  // P 6.1  Adc14  Pin23   J3-3   J5
           { GP_PORT_6, BIT0, ADC14INCH_15 }   // P 6.0  Adc16  Pin2    J1-2    !!! ??? Is really ADC14INCH_16 !!!
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC14INCH_16 },  // internal Temperature Sensor
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC14INCH_17 }   // internal Battery Monitor
        };


//*****************************************************************************
//  board_adc_init
//
//         Initialize the ADC module, and configure the overall sampling 
//         clock used for the ADC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User app triggered
//           - PWM Module 1  Channel   1 / 2 / 3 / 4
//           - PWM Module 2  Channel   1 / 2 / 3 / 4
//           - Timer n
//
// MSP432 has 1 ADC moodule, with a single sequencer allowing up to 32 channels
// The sequencer steps are individually identified:  ADC14MCTL0 -> ADC14MCTL31.
// ADC results are stored in corresponding registers: ADC14MEM0 -> ADC14MEM31.
// Results are stored in the low order 16 bits of the 32-bit ADC14MEMx reg.
//
// The Launchpad MSP432 100 pin device physcially supports up to 24 channels,
// of which 15 are wired out to the Launchpad pins, and 2 internal channels
// (Temperature Sensor and Battery Monitor) are available.
// Due to this layout, we support 17 total channels.
//*****************************************************************************
int  board_adc_init (int adc_module_id, uint32_t clock_rate, int trigger_type,
                     int flags)
{
    int  rc;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.

    if (_g_adc_configured == 0)
       {     //--------------------------------------
             // do basic configuration of ADC module
             //--------------------------------------
             // Turn on ADC14, extend sampling time to avoid overflow of results.
             // CONSEQ_1 = single Sequence-of-Channels operation
             // ADC14SHT0__192 takes FOREVER to convert the results. ( > 1 second for 4 channels)
//       ADC14CTL0 = ADC14ON |ADC14SHT0__192 | ADC14SHP | ADC14MSC | ADC14CONSEQ_1;
         ADC14CTL0 = ADC14ON | ADC14SHT0_2   | ADC14SHP | ADC14MSC | ADC14CONSEQ_1; // Sampling time, S&H=16, ADC14 on
         ADC14CTL1 = ADC14RES_2;                // NEW 04/26/15 - Use sampling timer, 12-bit conversion results

         if (trigger_type != ADC_TRIGGER_USER_APP)
            {      //-------------------------------------------------------------
                   // Setup SHS trigger field in ADC12CTL0
                   //-------------------------------------------------------------
              rc = board_adc_config_trigger_mode (trigger_type, flags);
              if (rc != 0)
                 return (rc);   // must be a bad trigger_type
            }

         _g_trigger_auser_api_id = trigger_type;  // save the trigger type we are using

// 05/16/15 vvvvvvvvvvvvvvvvvvvv IS NOT WORKING - NO INTERRUPTS ARE OCCURING vvvvvvvvvvvvv
              // Enable ADC interrupts in the NVIC module - since changed to monitor Adc Chan IFG13, it fails to interrupt !
         NVIC_ISER0 = 1 << ((INT_ADC14 - 16) & 31);

         _g_adc_configured = 1;                   // denote basic configuration was done

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
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference             (FUTURE_UPGRAGDE)
//
//         ADC14 has just one module.
//         ADC14 has just one sequencer.
//         It suppports up to 24 channels, of which 17 are hooked up on the LP.
//*****************************************************************************
int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer,     int step_num,
                               int last,          int flags)
{
    ADC_CHANNEL_BLK  *chblk;
    ADC_SEQSTEP_BLK  *ssreg;
    uint32_t         gpio_base_addr;
    int              orig_step_num;
    int              rc;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

    if (channel_num < 0  ||  channel_num > 16)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

           // FOR INITIAL BRINGUP, JUST ALWAYS USE SEQUENCER 1
           // to see how this API plays out across platforms
// ??? will need to make this code more sophisticated in the future to handle
//     newbie user that specifies ADC_ANY_SEQUENCER

             //***********************************************
             //      Add a new ADC channel to be sampled
             //***********************************************

          //-----------------------------------------------------
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of 12 entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[]
          //----------------------------------------------------
    chblk = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num];
    gpio_base_addr = _g_gpio_port_base_address[chblk->chan_gpio_port_id];
          // set associated SEL1 / SEL0 pins to denote this is being used
          // as an ADC channel  (aka TERTIARY function)
    HWREG16(gpio_base_addr + OFS_PASEL0) |= chblk->chan_gpio_pin_bit;
    HWREG16(gpio_base_addr + OFS_PASEL1) |= chblk->chan_gpio_pin_bit;

          //----------------------------------------------------------------
          // setup the Channel's step within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save original step number passed in
    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

          // setup ssreg to point to the appropriate ADCMCTLx entry
    ssreg = (ADC_SEQSTEP_BLK*) _g_adc_seqstep_regs [step_num];
    ssreg->seq_step_channel  = channel_num;
    if (last)
       ssreg->seq_step_eos  |= ADC_EOS_BIT;   // tell ADC14 that this is the
                                              // last chennel in the sequence

    if (orig_step_num == ADC_AUTO_STEP)
       { _g_adc_step_map [_g_step_num] = channel_num; // save what channel is assigned to this step
         _g_step_num++;            // inc to next step slot in the sequencer
       }
      else _g_adc_step_map [step_num] = channel_num;

          // track how many channels have been configured for use
    _g_active_channels++;

    return (0);                    // denote success
}


//*****************************************************************************
//  board_adc_config_trigger_mode
//
//         Sets up ADC to use a Timer's Trigger Output (TRGO), which is used
//         to automatically trigger the ADC.
//
//         The trigger field is specified in the ADC14SHSxx bits of the
//         ADC14CTL0 register.
//         For the MSP432 these are:
//
//                 Timer/ CCR      SHS Value    (from MSP432 datasheet p 91)
//                  TA0 / CCR1        1
//                  TA0 / CCR2        2
//                  TA1 / CCR1        3
//                  TA1 / CCR2        4
//                  TA2 / CCR1        5
//                  TA2 / CCR1        6
//                  TA3 / CCR1        7
//*****************************************************************************

int  board_adc_config_trigger_mode (int trigger_type, int flags)
{
    uint32_t  shs_value;

    if (trigger_type != ADC_TRIGGER_USER_APP)
       {    // user wants a timer to trigger the ADC14
         if (trigger_type < 1 || trigger_type > 7)
            return (ERR_TIMER_INVALID_TRIGGER_TYPE);
         shs_value = _g_adctrigger_shs_mask [trigger_type];

            // then apply the SHS to the ADC14CTL0 register
         ADC14CTL0 &= ~(ADC14SHS_7);         // Shutoff all the SHS bits
         ADC14CTL0 |= shs_value;             // apply desired value
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
    int    rc;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

// this is always returning busy after first Seqence-of_Conversions complete, even when SW triggerd !
    rc = ADC14_isBusy();
    if (rc)
       return (0);                    // ADC and DMA are still busy

#if (INTERRUPTS)
    if ( ! _g_ADC_complete)
       return (0);                    // ADC and DMA are still busy
#endif

    return (1);                       // ADC/DMA conversion sequenc is complete
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
       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

     if (ADC14_isBusy()  &&  _g_ADC_complete == 0)
        return (-1);                 // must wait till all conversions are done
                                     // ??? do wait in here - as a loop ???

#if (INTERRUPTS)
        // turn off ADC inteerupts
#endif

    ADC14CTL0 &= ~(ADC14ON | ADC14ENC); // turn off the ADC module for sampling

    return (0);                         // denote success
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
    uint16_t   ie_mask;
    uint32_t   shs_mask;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

    _g_ADC_complete = 0;             // clear I/O flags
    _g_DMA_overrun  = 0;

        //----------------------------------------------------------------------------
        // clear any previous rupts, then turn on ADC inteerupts
        //----------------------------------------------------------------------------
    ie_mask = 1 << (_g_active_channels - 1);
    ADC14IFGR0 &= ~(ie_mask);            // Clear any old ADC12 IFG.x interrupts
    ADC14IER0   = ie_mask;               // Enable ADC12 IFG.x interrupts
                                         //        based on last EOS channel.

    if (ADC14CTL0 & ADC14CONSEQ_2)
       _g_toggle_ENC_required = 0;       // is a REPEAT sequence, no need to toggle ENC

        //----------------------------------------------------------------------------
        // if automatic (timer) triggering was requested, turn it on in ADC12CTL1 SHS
        //----------------------------------------------------------------------------
    if (_g_trigger_auser_api_id !=  ADC_TRIGGER_USER_APP)
       { shs_mask = _g_adctrigger_shs_mask [_g_trigger_auser_api_id];
            // then apply the SHS to the ADC14CTL0 register
         ADC14CTL0 &= ~(ADC12SHS_7);     // Shutoff any existing SHS bits
         ADC14CTL0 |= shs_mask;          // Turn on requested triggering
       }

    ADC14CTL0 |= ADC12ENC;               // Enable conversions
    ADC14CTL0 |= ADC14ON;                // turn on the ADC module for sampling

       //---------------------------------------------------------
       // Even for (timer) triggered conversions, we should issue
       // an initial start (SC) to "prime the pump"
       //---------------------------------------------------------
/// if (_g_trigger_auser_api_id == ADC_TRIGGER_USER_APP)
    board_adc_user_trigger_start (adc_module_id, sequencer); // kick off initial Conversion

    return (0);                          // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer,
                            uint16_t  *channel_results)
{
    int       i;
    uint32_t  *adc_mem_data;

       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

#if (TO_DO)
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

//  adc_module = ADC0_BASE;    // HARD CODED HACK FOR INITIAL TESTING
    sequencer  = 1;

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
    adc_mem_data = &ADC14MEM0;       // point at begin of ADC results memory buf
    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         _g_adc_conv_results[i] = *adc_mem_data;  // copy data to internal staging area
         adc_mem_data++;                          // step to next ADC14MEMx
       }
           // TEMP HACK until get address of ADCMEM stuff working
//  _g_adc_conv_results[0] = ADC14MEM0;
//  _g_adc_conv_results[1] = ADC14MEM1;
//  _g_adc_conv_results[2] = ADC14MEM2;
//  _g_adc_conv_results[3] = ADC14MEM3;

#endif
    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_ADC_complete = 0;                 // reset for new pass

    return (_g_active_channels); // pass back number of completed conversions
}



//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a group of ADCs, on a sequenced group
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
       // There is only 1 ADC module on MSP432, so we ignore adc_module_id.
       // Ditto, there is only 1 sequencer module (built in to ADC module).

    ADC14CTL0 |= ADC14ENC | ADC14SC;   // Start conversion via software trigger

    return (0);                        // denote success
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
*                             ADC    ISR
*
* @brief  ADC14 interrupt service routine
*
*         If running straight interrupts (xxx_IT), this gets called by
*         the HAL ADC library support when the ADC interrupt completes.
*
*         If running with DMA (xxx_DMA), this gets called
*         when the DMA interrupt completes, i.e. this gets
*         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
*
*         If ENC toggling is required, allow a bunch of instructions to
*         execute, before turning it back on, to ensure the ADC properly
*         reset itself for the next conversion. PITA
****************************************************************************/

void  ADC14_ISR_Handler (void)
{


// ??? !!!   WVD   COMPLETE   THIS   !!! ??? !!!

    if (_g_toggle_ENC_required)
       ADC12CTL0 &= ~(ADC12ENC);    // non-repeat sequences require that we 
                                    // toggle ENC after each conversion set

#if (TO_DO)
       // Do I really need to copy anything at this point ?
       // Probably not, unless going DMA route.

    if (ADC12IV > 6 && ADC12IV <= 30)
       {     // this is the ending IFG0-IFG14 signal denoting end of sequence completed.

       // Check to make sure it is the interrupt we expect
    if (ADC14IFGR0 & ADC14IFG3)
      {
        A0results[index] = ADC14MEM0;       // Move A0 results, IFG is cleared
        A1results[index] = ADC14MEM1;       // Move A1 results, IFG is cleared
        A2results[index] = ADC14MEM2;       // Move A2 results, IFG is cleared
        A3results[index] = ADC14MEM3;       // Move A3 results, IFG is cleared
        index = (index + 1) & 0x7;          // Increment results index, modulo
        __no_operation();                   //Set Breakpoint1 here
      }
#endif

    _g_adc_conv_results[0] = ADC14MEM0;
    _g_adc_conv_results[1] = ADC14MEM1;
    _g_adc_conv_results[2] = ADC14MEM2;
    _g_adc_conv_results[3] = ADC14MEM3;

    dma_callback_seen++;                        // DEBUG COUNTER

    _g_ADC_complete = 1;     // set status that ADCs and DMA completed.
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
       ADC12CTL0 |= ADC12ENC;          // toggle ENC back on
}


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

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (_g_adc_callback != 0L)
       {
          (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                             _g_active_channels, 0);    // Call user handler
       }

//  HAL_ADC_Stop_DMA(hadc);   // ??? need - bit is blow up when re-enable ADC_IT
}
#endif

#endif                        //  defined(USES_ADC)




#if defined(USES_DAC)

//*****************************************************************************
//*****************************************************************************
//                               DAC   Routines
//*****************************************************************************
//*****************************************************************************

#if (STM32_LOGIC)
    DAC_HandleTypeDef       _g_DacHandle;
    DAC_ChannelConfTypeDef  _g_Dac_Chan_Config_1;         // DAC channel 1 I/O
    DAC_ChannelConfTypeDef  _g_Dac_Chan_Config_2;         // DAC channel 2 I/O
    DMA_HandleTypeDef       _g_DMA_dac1;                  // DAC channel 1 DMA
    DMA_HandleTypeDef       _g_DMA_dac2;                  // DAC channel 2 DMA
#endif

    uint32_t                _g_dma_dac1_rupt_seen = 0;         // DEBUG COUNTER
    uint32_t                _g_dma_dac2_rupt_seen = 0;         // DEBUG COUNTER

    char        _g_DMA_DAC_Ch1_complete = 0; // status: 1 = DAC and DMA completed
    char        _g_DMA_DAC_Ch2_complete = 0; // status: 1 = DAC and DMA completed

    long        _g_dac_chan_1_frequency = 0; // SPS frequency for DAC Channel 1
    long        _g_dac_chan_2_frequency = 0; // SPS frequency for DAC Channel 2

    uint16_t    _g_trigger_dtmr_mmsmask = 0; // Associated Mask for TIM's  MMS
    uint16_t    _g_trigger_duser_api_id = 0; // Associated DAC trigger type from dac_Init()

    short       *_g_dac_chan_1_table;       // pointer to sample table for chan
    int         _g_dac_chan_1_table_length; // length of sample table (steps)
    short       *_g_dac_chan_2_table;       // pointer to sample table for chan
    int         _g_dac_chan_2_table_length; // length of sample table (steps)

#define  pi     3.141592654
    float       _g_pi_div_180 = pi / 180.0; // get radians ratio to degrees

//*****************************************************************************
//  board_dac_init
//
//         Initialize a DAC module, and Configure
//         the overall sampling clock used for the DAC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//*****************************************************************************

int  board_dac_init (int dac_module_id, uint32_t clock_rate, int flags)
{

#if (STM32_LOGIC)
    __DAC1_CLK_ENABLE();
    __DMA1_CLK_ENABLE();

    _g_DacHandle.Instance = DAC;        // is just a single DAC module on F0_72

    if (HAL_DAC_Init(&_g_DacHandle) != HAL_OK)
       {
         board_error_handler();    /* Initiliazation Error */  // RETURN ERROR CODE !!!
       }
#endif

    return (0);
}


//*****************************************************************************
//  board_dac_config_channel
//
//         Configure a single DAC channel.
//
//         Note that the sampling rate will be determined by the trigger source.
//
//         adc_module parm is really just an index number (0-3)
//         that denote which ADC module to use.
//         For the STM32 F4, there is only 1 ADC module: DAC1
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - Timer
//           - GPIO EXTI
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//
//         STM32 F0_72 supports 1 DAC module with two channels, that are on
//         dedicated pins: Channel_1 = PA4 (DAC_OUT1),  Channel_2 = PA5 (DAC_OUT2)
//*****************************************************************************

int  board_dac_config_channel (int dac_module_id, int channel_num,
                               int trigger_type,
                               long sps_frequency, int flags)

{
    GPIO_InitTypeDef   GPIO_InitStruct;
    int                rc;

      // there is only 1 DAC module on the F0_72, so we ignore the module id

       // verify channel number is valid
    if (channel_num == 1  ||  channel_num == 2)
       ;
      else return (-1);          // set SUITABLE ERROR CODE  ??? !!!

// ??? !!! Add support for other trigger types:  TIMER_2, TIMER_3, GPIO_EXTI, ...


#if (STM32_LOGIC)
      /*##-2- Configure peripheral GPIO #####################################*/
      /* DAC Channel1 GPIO pin configuration */
    if (channel_num == 1)
       {         // configure GPIO PA4 to operate as DAC_OUT1 channel
         GPIO_InitStruct.Pin = GPIO_PIN_4;         // PA4
         _g_dac_chan_1_frequency = sps_frequency;  // save SPS frequency for this DAC
       }
      else {     // configure GPIO PA5 to operate as DAC_OUT2 channel
             GPIO_InitStruct.Pin = GPIO_PIN_5;         // PA5
             _g_dac_chan_2_frequency = sps_frequency;  // save SPS freq for this DAC
           }
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    // tag it as Analog
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
#endif

}


//*****************************************************************************
//  board_dac_check_conversions_done
//
//          Checks if the ALL the conversions for a sinple DAC are done.
//
//*****************************************************************************
int  board_dac_check_conversions_done (int dac_module_id, int channel_num)
{
    if (channel_num == 1)
       return (_g_DMA_DAC_Ch1_complete);    // channel 1 status
    return (_g_DMA_DAC_Ch2_complete);       // channel 2 status
}


//*****************************************************************************
//  board_dac_clear_conversions_done
//
//          Clears the DAC conversions flag to setup for next pass.
//
//*****************************************************************************
int  board_dac_clear_conversions_done (int dac_module_id, int channel_num)
{
    if (channel_num == 1)
       _g_DMA_DAC_Ch1_complete = 0;       // reset channel 1 status for new pass
       else _g_DMA_DAC_Ch2_complete = 0;  //    ditto channel 2 status
}


//*****************************************************************************
//  board_dac_enable_channel
//
//          Turn on one or all the sequencers that have been configured.
//*****************************************************************************
//extern  const uint8_t  aEscalator8bit[];      // 06/22/15 - TEMP HACK

int  board_dac_enable_channel (int dac_module_id, int channel_num, int flags)
{
    int   rc;
volatile long  period_ticks;

       //-----------------------------------------------------------------------
       // Convert the fequecy * # steps into a Timer period value dictating when
       // each sample step should be fed into the DAC (via DMA trigger from TIM6)
       //-----------------------------------------------------------------------
    period_ticks = board_frequency_to_period_ticks (_g_dac_chan_1_frequency * _g_dac_chan_1_table_length);


#if (STM32_LOGIC)
       /*##-2- Enable DAC Channel1 and associated DMA ######################*/
    if (channel_num == 1)
       {      //-----------------------------------------------------------
              // DAC channel 1 Configuration - default uses TIM6 and DMA1_CHANNEL3
              //-----------------------------------------------------------
//       __TIM6_CLK_ENABLE();                   // enable TIM6 Periph clock
//       board_TIM6_Config (period_ticks);      // configure TIM6
              // setup DAC trigger to use TIM6
// ??? !!!                                      VVVVVVVVVVVVVVVVVVVVVV need to base of module_id
         _g_Dac_Chan_Config_1.DAC_Trigger      = DAC_TRIGGER_T6_TRGO;
         _g_Dac_Chan_Config_1.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
         rc = HAL_DAC_ConfigChannel (&_g_DacHandle, &_g_Dac_Chan_Config_1, DAC_CHANNEL_1);
         if (rc != HAL_OK)
            {
              board_error_handler();   /* Channel configuration Error */  // RETURN ERROR CODE !!!
            }

            /*##-3- Configure the DMA CHANNEL to service the DAC  ######*/
            /* Set the parameters to be configured for DACx_DMA1_CHANNEL3 */
         _g_DMA_dac1.Instance                 = DMA1_Channel3;
         _g_DMA_dac1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
         _g_DMA_dac1.Init.PeriphInc           = DMA_PINC_DISABLE;
         _g_DMA_dac1.Init.MemInc              = DMA_MINC_ENABLE;
         _g_DMA_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
         _g_DMA_dac1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
         _g_DMA_dac1.Init.Mode                = DMA_CIRCULAR;
         _g_DMA_dac1.Init.Priority            = DMA_PRIORITY_HIGH;
         HAL_DMA_Init (&_g_DMA_dac1);
             /* Associate the initialized DMA handle to the the DAC handle */
         __HAL_LINKDMA (&_g_DacHandle, DMA_Handle1, _g_DMA_dac1);
             /*##-4- Configure the NVIC for DMA #############################*/
             /* Enable the DMA1_Channel3 IRQ Channel */
         HAL_NVIC_SetPriority (DMA1_Channel2_3_IRQn, 2, 0);
         HAL_NVIC_EnableIRQ (DMA1_Channel2_3_IRQn);   // will invoke DMA1_Channel2_3_IRQHandler()

         rc = HAL_DAC_Start_DMA (&_g_DacHandle, DAC_CHANNEL_1,
                                 (uint32_t*) _g_dac_chan_1_table,
                                 _g_dac_chan_1_table_length,
                                 DAC_ALIGN_12B_R);
       }
      else if (channel_num == 2)
       {      //-----------------------------------------------------------
              // DAC channel 2 Configuration - uses TIM7 and DMA1_CHANNEL4
              //-----------------------------------------------------------
//       __TIM7_CLK_ENABLE();                   // enable TIM7 Periph clock
//       board_TIM7_Config (period_ticks);      // configure TIM7
//            // setup DAC trigger to use TIM7
// ??? !!!                                      VVVVVVVVVVVVVVVVVVVVVV need to base of module_id
         _g_Dac_Chan_Config_2.DAC_Trigger      = DAC_TRIGGER_T7_TRGO;
         _g_Dac_Chan_Config_2.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        rc = HAL_DAC_ConfigChannel (&_g_DacHandle, &_g_Dac_Chan_Config_2, DAC_CHANNEL_2);
         if (rc != HAL_OK)
            {
              board_error_handler();   /* Channel configuration Error */  // RETURN ERROR CODE !!!
            }

            /*##-3- Configure the DMA CHANNEL to service the DAC  ######*/
            /* Set the parameters to be configured for DACx_DMA1_CHANNEL4 */
         _g_DMA_dac2.Instance                 = DMA1_Channel4;
         _g_DMA_dac2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
         _g_DMA_dac2.Init.PeriphInc           = DMA_PINC_DISABLE;
         _g_DMA_dac2.Init.MemInc              = DMA_MINC_ENABLE;
         _g_DMA_dac2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
         _g_DMA_dac2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
         _g_DMA_dac2.Init.Mode                = DMA_CIRCULAR;
         _g_DMA_dac2.Init.Priority            = DMA_PRIORITY_HIGH;
         HAL_DMA_Init (&_g_DMA_dac2);

// ??? do I need a second DAC module def, since this does not use the CHANNEL config def ???

             /* Associate the initialized DMA handle to the the DAC handle */
         __HAL_LINKDMA (&_g_DacHandle, DMA_Handle2, _g_DMA_dac2);

             /*##-4- Configure the NVIC for DMA #############################*/
             /* Enable the DMA1_Channel4 IRQ Channel */
         HAL_NVIC_SetPriority (DMA1_Channel4_5_IRQn, 2, 0);
         HAL_NVIC_EnableIRQ (DMA1_Channel4_5_IRQn);   // will invoke DMA1_Channel4_5_IRQHandler()

         rc = HAL_DAC_Start_DMA (&_g_DacHandle, DAC_CHANNEL_2,
                                 (uint32_t*) _g_dac_chan_2_table,
                                 _g_dac_chan_2_table_length,
                                 DAC_ALIGN_12B_R);
       }
      else return (-1);     // set APPROPRIATE ERROR CODE  invalid chan #
#endif

    if (rc != HAL_OK)
       {
         board_error_handler();   /* DMA Start DMA Error */  // RETURN ERROR CODE !!!
       }
    return (0);             // denote succeeded
}


//*****************************************************************************
//  board_dac_disable_channel
//
//          Turn off one or all the sequencers that have been configured.
//*****************************************************************************
int  board_dac_disable_channel (int dac_module_id, int channel_num, int flags)
{
}


//*****************************************************************************
//  board_dac_gen_sample_table
//
//          Generate a wave form table, with a given number of steps
//
//         Valid wave_types are:    DAC_GEN_SINEWAVE     DAC_GEN_SAWTOOTH
//                                  DAC_GEN_TRIANGLE     DAC_GEN_SQUAREWAVE
//
//*****************************************************************************

//  ??? include an amplitude factor/multiplier in here or in  dac_enable_channel() ???

//  ??? in future, allow a DC_Offset and max amplitude option for nore flexibility
//      ex: sine wave requires an implicit offset to get top/bottom of sinewave right.

int  board_dac_gen_sample_table (int wave_type, short *table_buf, int num_steps)
{
    int    i,  halfway_point;
    short  tbl_value;
    float  increment,  dc_offset_value,  curr_value;
    float  degree,  radians,  sine_val;
    float  degree_increment;

    if (wave_type < DAC_GEN_SINEWAVE || wave_type > DAC_GEN_SQUAREWAVE)
       return (-1);     // set APPROP ERROR CODE  ??? !!!
    if (num_steps < 2)
       return (-1);     // set APPROP ERROR CODE  ??? !!!

    halfway_point = (num_steps >> 1);           // divide by 2 to get halfway point
    curr_value = 0.0;                           // starting value
    switch (wave_type)
      { case DAC_GEN_SINEWAVE:          // standard sinewave, 1 cycle, 0 to 2 pi (360 degrees)
                  degree_increment = (360.0 / (float) num_steps); // how much each step is worth
                  degree = 0.0;                          // starting value (0 degrees)
                     // to get a "centered" sinewave, we need to add a DC offset,
                     // so that the top half is from 2048 -> 4095 -> 2048, and
                     // the bottom half is 2047 -> 0 -> 2047
                  dc_offset_value = 2047.0;
                  for (i = 0;  i < num_steps; i++)
                    { radians  = degree * _g_pi_div_180; // convert degrees to radians
                      sine_val = sin (radians);          // get associated sine value 0.0 to 1.0
//                    curr_value   = sine_val * 4095.0;  // convert to 0 - 4095 range
                      curr_value   = sine_val * dc_offset_value;
                      if (i < halfway_point)
                         {     // process top half of sine wave: 2047 -> 4095 -> 2047
                           tbl_value = ((short) curr_value);  // convert to int
                           if (tbl_value > 2048)
                              tbl_value = 2048;               // handle any float->int round off that would cause us to creep above 4095
                           table_buf[i] = tbl_value + 2047;   // add DC offset
                         }
                        else
                         {     // process bottom half of sine wave: 2047 -> 0 -> 2047
                               // sine_values at this point will all be negative
                           tbl_value = ((short) curr_value);  // convert to int
                           if (tbl_value < -2047)
                              tbl_value = -2047;              // handle any float->int round off that would cause us to creep below -2047
                           table_buf[i] = tbl_value + 2047;   // add DC offset
                         }
                      degree += degree_increment;        // step to next increment step
                    }
                  break;

        case DAC_GEN_SAWTOOTH:          // linear rising sawtooth, then drops to 0
///               for (i = 0;  i < 4095; i++)    // simple minded 1 tick per SPS point
///                  table_buf[i] = i;
                      // use float to minimize integer round-off issues.
                      // we calculate full scale 12 bits (4096), and apply amplitude later.
                  increment  = (4095.0 / (float) num_steps);  // how much each step is worth
                  for (i = 0;  i < num_steps; i++)
                    { table_buf[i] = (short) curr_value;  // convert to int
                      curr_value  += increment;           // step linearly to next value
                    }
                  break;

        case DAC_GEN_TRIANGLE :        // symmetric triangle centered at middle
                     // technically, a sawtooth is set of odd harmonic sine values,
                     // but a cheap way to calculate is to treat it as a rising linear
                     // ramp for the first 1/2 period, and a declining linjear ramp for
                     // the second half period.
                  increment  = (4095.0 / (float) halfway_point);  // how much each step is worth
                  for (i = 0;  i < halfway_point; i++)
                    {    // first half of period = linear rising
                      table_buf[i] = (short) curr_value;  // convert to int
                      curr_value  += increment;           // step up linearly to next value
                    }
                  for (i = halfway_point;  i < num_steps; i++)
                    {    // second half of period = linear declining
                      curr_value  -= increment;           // step down linearly to next value
                      table_buf[i] = (short) curr_value;  // convert to int
                      if (table_buf[i] < 0)
                         table_buf[i] = 0;                // clean up last value in case just dip below 0
                    }
                  break;

        case DAC_GEN_SQUAREWAVE:       // square wave = simple 50% on, 50 % off
                  for (i = 0; i < halfway_point; i++)
                     table_buf[i] = 0;                    // first half of table is zeros
                  for (i = halfway_point; i < num_steps; i++)
                     table_buf[i] = 4095;                 // second half of table is max
                  break;
      }

    return (0);               // denote completed successfully
}


//*****************************************************************************
//  board_dac_set_sample_table
//
//         Save the sample table address and length to be used for this channel.
//         At channel enable, will feed these into the DMA fot the channel.
//*****************************************************************************
int  board_dac_set_sample_table (int module_id, int channel_num,
                                 short *table_buf, int num_steps)
{
    if (channel_num == 1)
       { _g_dac_chan_1_table = table_buf;
         _g_dac_chan_1_table_length = num_steps;
       }
      else if (channel_num == 2)
              { _g_dac_chan_2_table = table_buf;
                _g_dac_chan_2_table_length = num_steps;
              }
             else return (-1);     // set APPROP ERROR CODE    ??? !!! invalid channel_id
}


/**************************************************************************
*                              DMA  DAC  ISR                 DAC Channel 1
*
* @brief  This handles the DMA interrupt request for the DAC DMA channel
*         DMA1_CHANNEL3.
*         It routes it to the ST HAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
**************************************************************************/

void  DMA1_Channel2_3_IRQHandler (void)
{
    _g_dma_dac1_rupt_seen++;                           // DEBUG COUNTER

    _g_DMA_DAC_Ch1_complete = 1; // set status that DAC and DMA completed.
                             // Used by dac_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.


//  HAL_DMA_IRQHandler (_g_DacHandle.DMA_Handle1);  // Call post rupt cleanup
                                                    //   and reset rupt flags
}

/**************************************************************************
*                              DMA  DAC  ISR                 DAC Channel 2
*
* @brief  This handles the DMA interrupt request for the DAC DMA channel
*         DMA1_CHANNEL4.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA1_Channel4_5_IRQHandler (void)
{
    _g_dma_dac2_rupt_seen++;                           // DEBUG COUNTER

    _g_DMA_DAC_Ch2_complete = 1; // set status that DAC and DMA completed.
                             // Used by dac_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.


//  HAL_DMA_IRQHandler (_g_DacHandle.DMA_Handle2);  // Call post rupt cleanup
                                                    //   and reset rupt flags
}

#endif                       //  defined(USES_DAC)



//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************


//*****************************************************************************
//  board_gpio_init
//
//        Configure Port Directions and Peripherals as needed.
//*****************************************************************************
void  board_gpio_init (void)
{

#if defined(USES_CC3100)
    spi_Open (0L, 0);    // Use TI SPI driver for now
 #endif

#if defined(USES_CC3100_BUT_USE_TI_SPI_CODE)
        //-------------------------------------------------------
        // Configure CC3100 SPI CS GPIO pin we are using (P 3.0)
        //-------------------------------------------------------
    P3OUT  &= ~BIT0;             // ensure it is set as De-asserted (LOW)  ???
    P3SEL1 &= ~BIT0;             // Set it as a GPIO
    P3SEL0 &= ~BIT0;
    P3DIR  |= BIT0;              // set as OUTPUT

        //----------------------------------------
        // configure CC3100 nHIB/ENABLE on P 4.3
        //----------------------------------------
    P4SEL1 &= ~BIT3;             // Set it as a GPIO
    P4SEL0 &= ~BIT3;

    P4OUT &= ~BIT3;              // set CC3100 nHIB to DISABLED
    P4DIR |=  BIT3;              // set as OUTPUT

        //-----------------------------------------
        // configure CC3100 host IRQ line on P 1.2
        //-----------------------------------------
    P1SEL1 &= ~BIT2;             // SEL0/1 = 00 = Set it as a GPIO
    P1SEL0 &= ~BIT2;
    P1DIR  &= ~BIT2;             // set it for INPUT

    P1REN |= BIT2;               // Turn on pullups for IRQ line

        //--------------------------------
        // Enable CC3100 IRQ interrupts
        //--------------------------------
    __enable_interrupt();

    Delay (50);     // 50 ms delay needed at CC3100 startup

        //------------------------------------
        // Enable CC3100 interrupt (IRQ line)
        //------------------------------------
    CC3100_InterruptEnable();

#endif                           // #if defined(USES_CC3100)

#if defined(USES_DRV8711)
       //------------------------------------------------------
       // Configure DRV8711 SPI CS GPIO pin we are using (PA2)
       //------------------------------------------------------
    DEASSERT_DRV8711_CS();  // FR5969 CHG   // Ensure CS is _De-Asserted_ (Low)
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

#if MSP430_ONLY
    PM5CTL0 &= ~LOCKLPM5;  // Disable GPIO power-on default High-Impedance mode
                           // to activate configured port settings
#endif
}


/*******************************************************************************
*  Board GPIO Pin Config
*
*        Configure an individual GPIO pin.
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

uint16_t  board_gpio_read_pins (int gpio_port_num, uint16_t pins_mask, int flag)
{
    uint32_t      gpio_base_addr;
    uint16_t      port_values;

//  if (gpio_port_num < 1 || gpio_port_num > MAX_GPIO_PORTS)
//     return (0xFFFF);                               // denote error

    gpio_base_addr = _g_gpio_port_base_address [gpio_port_num];
    port_values    = HWREG16(gpio_base_addr + OFS_PAIN);  // read the input port

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
       GPIO_clearInterruptFlag (gpio_port, pin);   // clear any prev interrupts
    GPIO_enableInterrupt (gpio_port, pin);
}



#if defined(USES_I2C)
//*****************************************************************************
//*****************************************************************************
//                               I2C   Routines
//*****************************************************************************
//*****************************************************************************

#define  I2C_NUM_MODULES     4     /* are 4 I2C capable modukess on MSP432 LP */

    char      _g_i2c_module_status [I2C_NUM_MODULES+1]  = { 0,0,0,0,0 };
    char      _g_i2c_io_status [I2C_NUM_MODULES+1]      = { 0,0,0,0,0 };

    uint8_t   *_g_i2c_mod_data_buf [I2C_NUM_MODULES+1];
    short     _g_i2c_mod_data_length [I2C_NUM_MODULES+1];

    uint32_t  i2c_ucb0_rupt = 0;     // DEBUG COUNTER
    uint32_t  i2c_ucb1_rupt = 0;     // DEBUG COUNTER

#define  I2C_IO_DONE           0      // _g_i2c_io_status[] settings
#define  I2C_IO_BUSY_RECV      1
#define  I2C_IO_BUSY_SEND      2
#define  I2C_IO_BUSY_SENDRECV  3


//*****************************************************************************
//  board_i2c_init
//
//               Initialize the designated I2C module as either a Master 
//               or a Slave.
//
//               We support 4 different SMCLK clock rates:
//                  1.000 Mhz (default startup), 12.0, 24.0, and 48.0 MCLK
//
// MSP432 Supports 4 eUSCI_B I2C modules:
//
//   Module 1   UCB0:  P 1.6 / P 1.7  on  J2-6 / J2-7   (Shared with SPI)
//   Module 2   UCB1:  P 6.4 / P 6.5  on  J1-10/ J1-9
//   Module 3   UCB2:  P 3.6 / P 3.7  on  J2-10/ J4-10     SDA / SCL
//   Module 4   UCB3:  P 6.6 / P 6.7  on  J4-5 / J4-6
//*****************************************************************************

int  board_i2c_init (int i2c_module, long baud_rate, int i2c_ms_mode, int flags)
{
    long  baud_UCBRW;

       // validate input arguments
    if (i2c_module < 1 || i2c_module > I2C_NUM_MODULES)
       return (ERR_I2C_MODULE_ID_OUT_OF_RANGE);

    if (i2c_ms_mode == I2C_MASTER || i2c_ms_mode == I2C_SLAVE)
       ;                                // mode is good
       else return (ERR_I2C_INVALID_I2C_MS_MODE);

    if (baud_rate > 400000)             // Max supported I2C speed is 400K
       return (ERR_I2C_EXCEEDS_MAX_BAUD_RATE);

        //----------------------------------------------------------------------
        //  Generate I2C Baud Rate pre-scalar, based on SMCLK
        //----------------------------------------------------------------------
    baud_UCBRW = (_g_SMCLK / baud_rate);      // setup baud-rate pre-scalar

        //--------------------------------------------------------------
        //     Initialize the associated GPIOs to run in I2C mode.
        //     Then configure the I2C module.
        //
        //     I2C pins are bi-directional, so DIR is not set, only SEL
        //--------------------------------------------------------------
    switch (i2c_module)
      { case 1:
              P1SEL0 |= BIT6 | BIT7;         // setup GPIOs for I2C mode
                   // Enable eUSCIB0 interrupt in NVIC module
              NVIC_ISER0 = 1 << ((INT_EUSCIB0 - 16) & 31);
                   // Configure USCI_B0 for I2C mode
              UCB0CTLW0 |= UCSWRST;                   // Turn on Software reset
              if (i2c_ms_mode == I2C_MASTER)
                 { UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;  // I2C mode, Master mode, sync
//                 UCB0BRW    = 0x0018;                     // baudrate = SMCLK / 8
                   UCB0BRW    = (unsigned short) baud_UCBRW;// set Baud rate prescalar
                   UCB0IE    |= UCRXIE | UCTXIE0 | UCNACKIE; // setup Interrupts
                 }
                else 
                 {    // note: slave mode does NOT set baud rate - master drives it
                   UCB0CTLW0 |= UCMODE_3 | UCSYNC;    // I2C Slave mode
                   UCB0IE    |= UCRXIE   | UCTXIE0 | UCSTPIE;  // setup Interrupts
                 }
              UCB0CTLW0 &= ~UCSWRST;                  // Enable I2C module
              break;

        case 2:
              P6SEL0 |= BIT4 | BIT5;    // setup GPIOs for I2C mode    P 6.4 / P6.5
                   // Enable eUSCIB1 interrupt in NVIC module
              NVIC_ISER0 = 1 << ((INT_EUSCIB1 - 16) & 31);
                   // Configure USCI_B1 for I2C mode
              UCB1CTLW0 |= UCSWRST;                   // Turn on Software reset
              if (i2c_ms_mode == I2C_MASTER)
                 { UCB1CTLW0 |= UCMODE_3 | UCMST | UCSYNC;  // I2C mode, Master mode, sync
//                 UCB1BRW    = 0x0018;                     // baudrate = SMCLK / 8
                   UCB1BRW    = (unsigned short) baud_UCBRW;// set Baud rate prescalar
//                 UCB1IE |= UCRXIE | UCTXIE0 | UCNACKIE; // setup Interrupts    THESE ARE NOT BEING APPLIED - WHY ???
                 }
                else
                 {    // note: slave mode does NOT set baud rate - master drives it
                   UCB1CTLW0 |= UCMODE_3 | UCSYNC;    // I2C Slave mode
                   UCB1IE    |= UCRXIE   | UCTXIE0 | UCSTPIE;  // setup Interrupts
                 }
              UCB1CTLW0 &= ~UCSWRST;                  // Enable I2C module
                      // the following only "take" if applied _AFTER_ the module is enabled. Else stays slammed at 0
              if (i2c_ms_mode == I2C_MASTER)
                 UCB1IE |= UCRXIE | UCTXIE0 | UCNACKIE;      // setup Interrupts
                 else UCB1IE |= UCRXIE | UCTXIE0 | UCSTPIE;
              break;

        case 3:
              P3SEL0 |= BIT6 | BIT7;    // setup GPIOs for I2C mode
              break;

        case 4:
              P6SEL0 |= BIT6 | BIT7;    // setup GPIOs for I2C mode
              break;
      }

    _g_i2c_module_status[i2c_module] = i2c_ms_mode; // save what mode it is initialized in

    return (0);
}


//******************************************************************************
//  board_i2c_check_io_complete
//
//             Checks to see if an I2C master or slave's send/receive data has been
//             processed, or if it is still in process, waiting
//             for the Master to complete an I2C sequence.
//
//             Returns 0 if still in process/busy.
//             Returns 1 if completed.
//
//  ??? Offer the option of a user app callback instead ???
//
//******************************************************************************

int  board_i2c_check_io_complete (int i2c_module, int flags)
{
    if (_g_i2c_io_status [i2c_module] != I2C_IO_DONE)
       return (0);

    return (1);                // denote data I/O is complete
}


//*****************************************************************************
//  board_i2c_nack
//
//             Generates an I2C NACK response.
//*****************************************************************************
int  board_i2c_nack (int i2c_module)
{

    if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
       ;            // salve must be treated differently

//   TBD

    return (0);                // denote success
}


//*****************************************************************************
//  board_i2c_rx_data_check
//
//             Checks if any pending input from I2C.
//
//   ??? FEASIBLE    ???  NEEDED
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

int  board_i2c_rx_data_check (void)
{
#if defined(USES_YAMS)
    int  rc;

//  rc = UART_getInterruptStatus (EUSCI_A0_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA0IFG & UCRXIFG;
    if (rc)
       return (1);                        // we have some data

    return (0);                           // user has not typed in any char
#endif

}


//******************************************************************************
//  board_i2c_receive
//
//             Issues a I2C sequence to read a block of data from partner.
//
//             We receive directly into the caller's buffer, so they must
//             not change anything, util they get I2C I/O complete.
//******************************************************************************

int  board_i2c_receive (int i2c_module, uint8_t *rcv_buf, int max_rcv_length,
                        int flags)
{
    _g_i2c_mod_data_buf [i2c_module]    = rcv_buf;     // save user I/O buf addr
    _g_i2c_mod_data_length [i2c_module] = max_rcv_length;   // and length

    _g_i2c_io_status [i2c_module] = I2C_IO_BUSY_SEND;  // denote I/O in progress

//   TBD

    if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
       ;    // slave needs to be handled differently - queue it

    return (0);                // denote data is queued, no errors
}


//******************************************************************************
//  board_i2c_send
//
//             Sends a I2C sequence with data.
//
//             We send directly out of the caller's buffer, so they must
//             not change anything, util they get I2C I/O complete.
//******************************************************************************

int  board_i2c_send (int i2c_module, uint8_t *send_buf, int send_length, int flags)
{
    _g_i2c_mod_data_buf [i2c_module]    = send_buf;    // save user I/O buf addr
    _g_i2c_mod_data_length [i2c_module] = send_length; //     and length

    _g_i2c_io_status [i2c_module] = I2C_IO_BUSY_SEND;  // denote I/O in progress

    switch (i2c_module)
      { case 1:
              if (_g_i2c_module_status[1] == I2C_MASTER)
                 {
                      // ensure any previous STOP condition send was completed
                   while (UCB0CTLW0 & UCTXSTP)  ;

                      // Issue a I2C START condition, to send address to slave
                      // Ensure that read/write bit is set as needed
                   UCB0CTLW0 |= UCTXSTT | UCTR;    // Issue a new I2C start as Sender
                 }
                ;    // slave needs to be handled differently - queue it
              break;
        case 2:
              if (_g_i2c_module_status[2] == I2C_MASTER)
                 {
                      // ensure any previous STOP condition send was completed
                   while (UCB1CTLW0 & UCTXSTP)  ;

                      // Issue a I2C START condition, to send address to slave
                      // Ensure that read/write bit is set as needed
                   UCB1CTLW0 |= UCTXSTT | UCTR;    // Issue a new I2C start as Sender
                 }
                ;    // slave needs to be handled differently - queue it
              break;
      }

    return (0);                // denote data is queued, no errors
}


//******************************************************************************
//  board_i2c_send_receive
//
//             Sends a I2C sequence with data, then issues a I2C RESTART to
//             shift direction, and read back a reply.
//
//             We send directly out of the caller's buffer, so they must
//             not change anything, util they get I2C I/O complete.
//******************************************************************************

int  board_i2c_send_receive (int i2c_module, uint8_t *send_buf, int send_length, 
                             uint8_t *rcv_buf, int max_rcv_length, int flags)
{
    _g_i2c_mod_data_buf [i2c_module]    = send_buf;    // save user I/O buf addr
    _g_i2c_mod_data_length [i2c_module] = send_length; //     and length

    _g_i2c_io_status [i2c_module] = I2C_IO_BUSY_SENDRECV; // set I/O in progress

    if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
       ;    // slave needs to be handled differently - queue it

    return (0);                // denote data is queued, no errors
}


//*****************************************************************************
//  board_i2c_set_slave_address
//
//             Sets the I2C slave address to use
//*****************************************************************************
int  board_i2c_set_slave_address (int i2c_module, short slave_addr)
{
    switch (i2c_module)
      { case 1:
              if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
                 UCB0I2CSA  = slave_addr;           // Slave address to send to
                 else UCB0I2COA0 = slave_addr | UCOAEN; // set our addr + enable
              break;

        case 2:
              if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
                 UCB1I2CSA  = slave_addr;         // Slave address to send to
                 else UCB1I2COA0 = slave_addr | UCOAEN; // set our addr + enable
              break;

        case 3:
              if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
                 UCB2I2CSA  = slave_addr;         // Slave address to send to
                 else UCB2I2COA0 = slave_addr | UCOAEN; // set our addr + enable
              break;

        case 4:
              if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
                 UCB3I2CSA  = slave_addr;         // Slave address to send to
                 else UCB3I2COA0 = slave_addr | UCOAEN; // set our addr + enable
              break;
      }

    return (0);                // denote success
}


//*****************************************************************************
//  board_i2c_stop
//
//             Generates an I2C STOP response.
//*****************************************************************************
int  board_i2c_stop (int i2c_module)
{

//   TBD

    if (_g_i2c_module_status[i2c_module] == I2C_MASTER)
       ;    // slave needs to be handled differently - queue it

    return (0);                // denote success
}


//------------------------------------------
//               I2C    ISR
//
//       I2C interrupt service routine
//------------------------------------------
void  eUSCIB0_ISR_Handler (void)
{
    uint8_t  *buf_addr;
    short    data_lng;

    i2c_ucb0_rupt++;                       // DEBUG COUNTER

    buf_addr = _g_i2c_mod_data_buf [0];
    data_lng = _g_i2c_mod_data_length [0];

    if (_g_i2c_module_status[1] == I2C_MASTER)
       {     //-----------------------------------------
             //  process interrupts when in MASTER mode
             //-----------------------------------------
         if (UCB0IFG & UCNACKIFG)
            {
              UCB0IFG &= ~ UCNACKIFG;    // clear NAK rupt flag
              UCB0CTLW0 |= UCTXSTT;      // Issue a new I2C start condition
            }
         if (UCB0IFG & UCRXIFG0)
            {
              UCB0IFG  &= ~ UCRXIFG0;    // clear data RX rupt flag
              *buf_addr = UCB0RXBUF;     // read in and store I2C RX data
              data_lng--;
              if (data_lng <= 0)
                 {    // we received the last byte of expected data. Send a STOP
                   UCB0CTLW0 |= UCTXSTP;  // Issue a I2C STOP
                   _g_i2c_io_status [1] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // more data is expected. update buffer address and remain lng
                   _g_i2c_mod_data_length [1] = data_lng;
                   _g_i2c_mod_data_buf [1]    = (++buf_addr);
                 }
            }
         if (UCB0IFG & UCTXIFG0)
            {
              UCB0IFG   &= ~ UCTXIFG0;    // clear any prev I2X TX rupt
// need to check if I2C_IO_SENDRECV - issue a RESTART instead, and set recv_buffer
              if (data_lng <= 0)
                 {    // we sent the last byte of data. Send a STOP
                   UCB0CTLW0 |= UCTXSTP;  // Issue a I2C STOP
                   _g_i2c_io_status [1] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // send next byte, update buffer address and remain lng
                   UCB0TXBUF  = *buf_addr;       // send the next byte of data
                   _g_i2c_mod_data_length [1] = (--data_lng);
                   _g_i2c_mod_data_buf [1]    = (++buf_addr);
                 }
            }
       }

      else
       {     //-----------------------------------------
             //  process interrupts when in SLAVE mode
             //-----------------------------------------
         if (UCB0IFG & UCSTPIFG)
            {
              UCB0IFG &= ~UCSTPIFG;      // Clear stop condition rupt flag
              _g_i2c_io_status [1] = I2C_IO_DONE;  // signal I/O complete
            }
         if (UCB0IFG & UCRXIFG0)
            {
              UCB0IFG &= ~ UCRXIFG0;     // clear data RX rupt flag
              if (data_lng <= 0)
                 {    // we received the last byte of expected data. Send a NAK
                   UCB0CTLW0 |= UCTXNACK;  // Issue a I2C NACK
                   _g_i2c_io_status [1] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // more data is expected. update buffer address and remain lng
                   *buf_addr = UCB0RXBUF;     // read in and store I2C RX data
                   _g_i2c_mod_data_length [1] = (--data_lng);
                   _g_i2c_mod_data_buf [1]    = (++buf_addr);
                 }
            }
         if (UCB0IFG & UCTXIFG0)
            {
              UCB0IFG   &= ~ UCTXIFG0;   // clear any prev I2X TX rupt
// need to check if I2C_IO_SENDRECV - issue a RESTART instead, and set recv_buffer
              if (data_lng <= 0)
                 {    // we sent the last byte of data. Send a NACK
                   UCB0CTLW0 |= UCTXNACK;  // Issue a I2C NACK
                 }
                else
                 {    // send next byte, update buffer address and remain lng
                   UCB0TXBUF  = *buf_addr;       // send the next byte of data
                   _g_i2c_mod_data_length [1] = (--data_lng);
                   _g_i2c_mod_data_buf [1]    = (++buf_addr);
                 }
            }
       }
}


//--------------------------------------------------
//               I2C    ISR           UCB1 - Grove
//
//       I2C interrupt service routine
//---------------------------------------------------
void  eUSCIB1_ISR_Handler (void)
{
    uint8_t  *buf_addr;
    short    data_lng;

    i2c_ucb1_rupt++;                       // DEBUG COUNTER

    buf_addr = _g_i2c_mod_data_buf [2];
    data_lng = _g_i2c_mod_data_length [2];

    if (_g_i2c_module_status[2] == I2C_MASTER)
       {     //-----------------------------------------
             //  process interrupts when in MASTER mode
             //-----------------------------------------
         if (UCB1IFG & UCNACKIFG)
            {
              UCB1IFG &= ~ UCNACKIFG;    // clear NAK rupt flag
              UCB1CTLW0 |= UCTXSTT;      // Issue a new I2C start condition
            }
         if (UCB1IFG & UCRXIFG0)
            {
              UCB1IFG  &= ~ UCRXIFG0;    // clear data RX rupt flag
              *buf_addr = UCB1RXBUF;     // read in and store I2C RX data
              data_lng--;
              if (data_lng <= 0)
                 {    // we received the last byte of expected data. Send a STOP
                   UCB1CTLW0 |= UCTXSTP;  // Issue a I2C STOP
                   _g_i2c_io_status [2] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // more data is expected. update buffer address and remain lng
                   _g_i2c_mod_data_length [2] = data_lng;
                   _g_i2c_mod_data_buf [2]    = (++buf_addr);
                 }
            }
         if (UCB1IFG & UCTXIFG0)
            {
              UCB1IFG   &= ~ UCTXIFG0;    // clear any prev I2X TX rupt
// need to check if I2C_IO_SENDRECV - issue a RESTART instead, and set recv_buffer
              if (data_lng <= 0)
                 {    // we sent the last byte of data. Send a STOP
                   UCB1CTLW0 |= UCTXSTP;  // Issue a I2C STOP
                   _g_i2c_io_status [2] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // send next byte, update buffer address and remain lng
                   UCB1TXBUF  = *buf_addr;       // send the next byte of data
                   _g_i2c_mod_data_length [2] = (--data_lng);
                   _g_i2c_mod_data_buf [2]    = (++buf_addr);
                 }
            }
       }

      else
       {     //-----------------------------------------
             //  process interrupts when in SLAVE mode
             //-----------------------------------------
         if (UCB1IFG & UCSTPIFG)
            {
              UCB1IFG &= ~UCSTPIFG;      // Clear stop condition rupt flag
              _g_i2c_io_status [2] = I2C_IO_DONE;  // signal I/O complete
            }
         if (UCB1IFG & UCRXIFG0)
            {
              UCB1IFG &= ~ UCRXIFG0;     // clear data RX rupt flag
              if (data_lng <= 0)
                 {    // we received the last byte of expected data. Send a NAK
                   UCB1CTLW0 |= UCTXNACK;  // Issue a I2C NACK
                   _g_i2c_io_status [2] = I2C_IO_DONE;  // signal I/O complete
                 }
                else
                 {    // more data is expected. update buffer address and remain lng
                   *buf_addr = UCB1RXBUF;     // read in and store I2C RX data
                   _g_i2c_mod_data_length [2] = (--data_lng);
                   _g_i2c_mod_data_buf [2]    = (++buf_addr);
                 }
            }
         if (UCB1IFG & UCTXIFG0)
            {
              UCB1IFG   &= ~ UCTXIFG0;   // clear any prev I2X TX rupt
// need to check if I2C_IO_SENDRECV - issue a RESTART instead, and set recv_buffer
              if (data_lng <= 0)
                 {    // we sent the last byte of data. Send a NACK
                   UCB1CTLW0 |= UCTXNACK;  // Issue a I2C NACK
                 }
                else
                 {    // send next byte, update buffer address and remain lng
                   UCB1TXBUF  = *buf_addr;       // send the next byte of data
                   _g_i2c_mod_data_length [2] = (--data_lng);
                   _g_i2c_mod_data_buf [2]    = (++buf_addr);
                 }
            }
       }
}

#endif                                   // USES_I2C



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
//                       System CPU / Systick   Routines
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
//          Setup CPU clocks and turn off Watch_Dog_Timer
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{

    _g_MCLK  = CS_getMCLK();           // The default startup MCLK is 3000000 (3 MHz)

// the following code was pulled from Richard S's FreeRTOS demo for MSP432.
	    //-----------------------------------------------------------------
            // Configure the clocks for maximum frequency.
	    // From the datasheet:  For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes,
	    // the maximum CPU operating frequency is 48 MHz
	    // and maximum input clock frequency for peripherals is 24 MHz.
	    //-----------------------------------------------------------------
    PCM_setCoreVoltageLevel (PCM_VCORE1);

// Add support for 3 MHz (default startup), 12 MHz (0 wait states), 24 MHz (sweetspot 1 wait state), 
//             and 48 MHz (2 wait states)
//   3 MHz  DCO = 0 ?,   12 MHz  DCO = 3,   24 MHz  DCO = 4,   48 MHz  DCO = 5


    CS_setDCOCenteredFrequency (CS_DCO_FREQUENCY_48);

    CS_initClockSignal (CS_HSMCLK, CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
    CS_initClockSignal (CS_MCLK,   CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
//  if (cpu_clock > 24000000)
       CS_initClockSignal (CS_SMCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_2);  // SMCLK cannot exceed 24 MHz
//     else CS_initClockSignal (CS_SMCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
    CS_initClockSignal (CS_ACLK,   CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);


        // save the configured clock ticks (MHz) settings
    _g_SysClk_Ticks = CS_getMCLK();       // save the MCU clock ticks setting
    _g_MCLK  = CS_getMCLK();             // save main CPU clock ticks
    _g_SMCLK = CS_getSMCLK();            // save the SMCLK peripheral clock ticks
    _g_ACLK  = CS_getACLK();             // save the ACLK  slow clock ticks

//      // compute Timer divisor factor for 1 ms timer. Yields needed CCR0 value
//  g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # of SMCLK ticks in 1 milli-sec
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
//
//       If MSP432 MCU is running at 48 MHz, then SMCLK is running at only
//       one-half that, because max recommended limit for SMCLK is 24 MHz.
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SMCLK);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
// board_systick_timer_config
//
//          Provide a "SYSTICK" style Interval Timer.
//
//*****************************************************************************
void  board_systick_timer_config (void)
{
    uint32_t  systick_config;

        // Configure and enable SysTick for use as interval timer
    systick_config = (_g_SysClk_Ticks / 1000);   // set for 1 ms period/pop
    SysTick_setPeriod (systick_config);
    SysTick_enableModule();
    SysTick_enableInterrupt();
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


/*******************************************************************
 *                             SYSTICK    ISR
 *
 * SysTick interrupt handler.
 *
 *                   Increment Systick 1 ms count, on every 1ms pop
 *******************************************************************/
void  SysTick_ISR_Handler (void)
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

// may need to subset this for ADC and DAC, since they just need:  init/set_trigger/set_period    WVD   ??? !!!

//*****************************************************************************
//*****************************************************************************
//                            TIMER  /  PWM     Routines
//*****************************************************************************
//*****************************************************************************
//     This section provides basic timer/pwm setup and trigger mode support

               //--------------------------------------------------------------
               //            Supported Physical Timer Modules
               //
               //   Module 0  TA0      CCR 1,2,3,4,5,6    on Launcpad pins
               //   Module 1  TA1      CCR 1,2,3,4,5,6    on bottom pins
               //   Module 2  TA2      CCR 1,2,3,4,5,6    on Launcpad pins
               //   Module 3  TA3      CCR 1,2,3,4,5,6    on bottom pins
               //
               // None of them support built-in complementary operation.
               // None of them support built-in Deadtime, but there are
               // ways to hack it (CCR1/CCR4 for deadtime, CCR2/CCR3 for PWM).
               //--------------------------------------------------------------
#define  TMPWM_NUM_MODULES      4   /* are 4 Timer/PWM capable modukess on MSP432 LP */
#define  TMPWM_MAX_CHANNELS     6   /* are max 6 channels per Timer/PWM module */
#define  MAX_TIMER        TIMER_3

              // flags for _g_pwm_module_status[] entries
#define  PWM_DOWN_COUNT_INIT      0x01   /* Module setup for DOWN count mode */
#define  PWM_UP_COUNT_INIT        0x02   /* Module setup for UP   count mode */
#define  PWM_CENTER_COUNT_INIT    0x04   /* Module setup for UP/DOWN count mode */
#define  TMR_PWM_INTERRUPTS_USED  0x40   /* Module uses Timer interrupts */

              // flags for _g_tmpwm_channels_config[] entries
#define  TMR_USING_PWM_MODE       0x80        /* module is using PWM          */
#define  TMR_CCR3N_CONFIGURED     0x40        /* Comple channel 3N configured */
#define  TMR_CCR2N_CONFIGURED     0x20        /* Comple channel 2N configured */
#define  TMR_CCR1N_CONFIGURED     0x10        /* Comple channel 1N configured */
#define  TMR_CCR4_CONFIGURED      0x08        /* Channel 4 configured for use */
#define  TMR_CCR3_CONFIGURED      0x04        /* Channel 3 configured for use */
#define  TMR_CCR2_CONFIGURED      0x02        /* Channel 2 configured for use */
#define  TMR_CCR1_CONFIGURED      0x01        /* Channel 1 configured for use */

                           // Bit mask status of each Timer Module (initialized, ...)
      char              _g_tmpwm_module_status [TMPWM_NUM_MODULES+1]  = { 0,0,0,0,0,0 };

                           // Bit Mask status of CCRs in each module (enabled, PWM, ...)
                           // One entry per timer module.
      unsigned char     _g_tmpwm_channels_config [TMPWM_NUM_MODULES+1] = { 0,0,0 };

                           // Bit mask of which CCRs are using interrupts in each module.
                           // One entry per timer module.
      uint16_t          _g_timer_app_enabled_interrupts [TMPWM_NUM_MODULES+1] = { 0,0,0,0,0,0 };

  TMR_CB_EVENT_HANDLER  _g_ptimer_callback [TMPWM_NUM_MODULES+1]   = { 0,0,0,0,0 };
      void              *_g_ptimer_callback_parm [TMPWM_NUM_MODULES+1];

                           // Timer Prescalars used for each Timer module.
      uint32_t         _g_tmpwm_prescalars [TMPWM_NUM_MODULES+1]     = { 0,0,0,0,0,0 };

                           // Timer OC reload values for IRQs servicing CCRs in Mode 4,5, ...
      int              _g_timer_A0_reload_value_array [TMPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      int              _g_timer_A1_reload_value_array [TMPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      int              _g_timer_A2_reload_value_array [TMPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };
      int              _g_timer_A3_reload_value_array [TMPWM_MAX_CHANNELS+1] = { 0,0,0,0,0,0 };

        //----------------------------------------------------------------------
        //     Global constants and Tables used to manage Timer/PWM Logic
        //----------------------------------------------------------------------

const unsigned char   _g_pwm_chan_mask [8]     // Bit masks used to track if a channel
                                               // is enabled and/or using COMPLE
                               = {         0,            /* Channel 0 is unused */
                                   TMR_CCR1_CONFIGURED,  /* Channels 1-4 */
                                   TMR_CCR2_CONFIGURED,  /*   0x02  */
                                   TMR_CCR3_CONFIGURED,  /*   0x04  */
                                   TMR_CCR4_CONFIGURED,  /*   0x08  */
                                   TMR_CCR1N_CONFIGURED, /* Channels 1N-3N */
                                   TMR_CCR2N_CONFIGURED, /*   0x20  */
                                   TMR_CCR3N_CONFIGURED  /*   0x40  */
                                 };                      // there is no 4N

               //--------------------------------------------------------------
               //                    Trigger _CHANNEL_ Mapping Table
               //--------------------------------------------------------------
const int         _g_tmpwm_trigger_channel []
                               = {   -1,              // -1 denotes no trigger
                                   TMR_PWM_CHANNEL_1, // TRIGGER_TIMER_0_CC1
                                   TMR_PWM_CHANNEL_2, // TRIGGER_TIMER_0_CC2
                                   TMR_PWM_CHANNEL_1, // TRIGGER_TIMER_1_CC1
                                   TMR_PWM_CHANNEL_2, // TRIGGER_TIMER_1_CC2
                                   TMR_PWM_CHANNEL_1, // TRIGGER_TIMER_2_CC1
                                   TMR_PWM_CHANNEL_2, // TRIGGER_TIMER_2_CC2
                                   TMR_PWM_CHANNEL_1  // TRIGGER_TIMER_3_CC1
                                 };

               //--------------------------------------------------------------
               //                    Trigger _TIMER_ Mapping Table
               //--------------------------------------------------------------
const int         _g_tmpwm_trigger_timer []
                               = {   -1,        // -1 denotes no trigger
                                   TIMER_0,     // TRIGGER_TIMER_0_CC1
                                   TIMER_0,     // TRIGGER_TIMER_0_CC2
                                   TIMER_1,     // TRIGGER_TIMER_1_CC1
                                   TIMER_1,     // TRIGGER_TIMER_1_CC2
                                   TIMER_2,     // TRIGGER_TIMER_2_CC1
                                   TIMER_2,     // TRIGGER_TIMER_2_CC2
                                   TIMER_3      // TRIGGER_TIMER_3_CC1
                                 };

               //--------------------------------------------------------------
               //                       GPIO   MAPPING   TABLES
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
typedef struct pwm_channel_def         /* ADC Channel definitions */
    {
        uint16_t  pwm_gpio_port_id;   /* Associated GPIO port         */
        uint32_t  pwm_gpio_pin_bit;   /* Associated GPIO pin          */
        uint16_t  pwm_chan_num;       /* Associated PWM/Timer channel num */
    } PWM_CHANNEL_BLK;


                           // PWM_MODULE_0
const PWM_CHANNEL_BLK  _g_tmpwm_mod_0_channels [] =  //          Energia  LP   Grove
        { {    0,       0,     0,          },  // no CCR0 entry
          { GP_PORT_2, BIT4, PWM_CHANNEL_1 },  // P 2.4  TA0.1  Pin38   J4-3
          { GP_PORT_2, BIT5, PWM_CHANNEL_2 },  // P 2.5  TA0.2  Pin19   J2-2 "Official" LP PWM
          { GP_PORT_2, BIT6, PWM_CHANNEL_3 },  // P 2.6  TA0.3  Pin39   J4-2
          { GP_PORT_2, BIT7, PWM_CHANNEL_4 },  // P 2.7  TA0.4  Pin40   J4-1
        };

                           // PWM_MODULE_1 - channels are pinned out to bottom 40 pin connector
const PWM_CHANNEL_BLK  _g_tmpwm_mod_1_channels [] =  //
        { {    0,       0,     0,          },  // no CCR0 entry
          { GP_PORT_7, BIT7, PWM_CHANNEL_1 },  // P 7.7  TA1.1
          { GP_PORT_7, BIT6, PWM_CHANNEL_2 },  // P 7.6  TA1.2
          { GP_PORT_7, BIT5, PWM_CHANNEL_3 },  // P 7.5  TA1.3
          { GP_PORT_7, BIT4, PWM_CHANNEL_4 },  // P 7.4  TA1.4
        };

                          // PWM_MODULE_2
const PWM_CHANNEL_BLK  _g_tmpwm_mod_2_channels [] =  //          Energia   LP   Grove
        { {    0,       0,     0,          },  // no CCR0 entry
          { GP_PORT_5, BIT6, PWM_CHANNEL_1 },  // P 5.6  TA2.1  Pin13   J4-4
          { GP_PORT_5, BIT7, PWM_CHANNEL_2 },  // P 5.7  TA2.2  Pin28   J2-4
          { GP_PORT_6, BIT6, PWM_CHANNEL_3 },  // P 6.6  TA2.3  Pin8    J4-5
          { GP_PORT_6, BIT7, PWM_CHANNEL_4 },  // P 6.7  TA2.4  Pin27   J4-6
        };

                           // PWM_MODULE_3 - channels are pinned out to bottom 40 pin connector
const PWM_CHANNEL_BLK  _g_tmpwm_mod_3_channels [] =  //
        { {    0,       0,     0,          },  // no CCR0 entry
          { GP_PORT_10,BIT5, PWM_CHANNEL_1 },  // P 10.5  TA3.1
          { GP_PORT_8, BIT2, PWM_CHANNEL_2 },  // P 8.2   TA3.2
          { GP_PORT_9, BIT2, PWM_CHANNEL_3 },  // P 9.2   TA3.3
          { GP_PORT_9, BIT3, PWM_CHANNEL_4 },  // P 9.3   TA3.4
        };

                                          /* TMPWM_MODULE_1 |  TMPWM_MODULE_2 */
                                          /* ------------   |  ------------   */
//#define  PWM_CHANNEL_1   1              /*  TA0.1  P 2.4  |   TA2.1  P 5.6  */
//#define  PWM_CHANNEL_2   2              /*  TA0.2  P 2.5  |   TA2.2  P 5.7  */
//#define  PWM_CHANNEL_3   3              /*  TA0.3  P 2.6  |   TA2.3  P 6.6  */
//#define  PWM_CHANNEL_4   4              /*  TA0.4  P 2.7  |   TA2.4  P 6.7  */

//#define  PWM_MODULE_0    0              /* TA0   with CCRs 1,2,3,4       */
//#define  PWM_MODULE_2    2              /* TA2   with CCRs 1,2,3,4       */


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
//
//  Tiva pin PA6 uses M1PWM2, which is Module 1 Output 2.
//  It is controlled by                Module 1 PWM, Generator 1.
//
//  Tiva PWM nomenclature:
//      PWM_GEN_0 Covers M1 PWM 0 and M1PWM1 = PWM_OUT_0_BIT / _1_BIT
//      PWM_GEN_1 Covers M1 PWM 2 and M1PWM3 = PWM_OUT_2_BIT / _3_BIT
//      PWM_GEN_2 Covers M1 PWM 4 and M1PWM5 = PWM_OUT_4_BIT / _5_BIT
//      PWM_GEN_3 Covers M1 PWM 6 and M1PWM7 = PWM_OUT_6_BIT / _7_BIT
//*****************************************************************************

int  board_timerpwm_init (int module_id, int count_mode, long period_val,
                          int timer_clock_source, int flags)
{
    uint32_t      timer_base_addr;
    uint32_t      prescalar;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (count_mode == TIMER_PERIODIC_COUNT_UP || count_mode == TIMER_PERIODIC_COUNT_UPDOWN)
       ;                                           // mode is valid
      else return (ERR_PWM_COUNT_MODE_INVALID);    // count_mode is bad - bail

// ??? revisit the following
        /* Compute the prescaler value to have TIMx counter clock equal to 18 MHz */
//  prescalar = ((SystemCoreClock /2) / 18000000) - 1;

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       // set the Timer module's CCR0, which acts as the period register
    HWREG16(timer_base_addr + OFS_TA0CCR0) = period_val;

    _g_tmpwm_module_status [module_id] = count_mode;      // save count mode

    return (0);                                           // denote success
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

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
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
    uint32_t          timer_base_addr;
    PWM_CHANNEL_BLK   *pwmcblk;
    uint32_t          gpio_base_addr;
    unsigned char     chan_mask;
    int               chan_index;
    uint32_t          hal_channel_num;
    int               ccr_ctl_value;
    int               rupt_mask;
    int               reload_value;

       // do standard sanity tests
    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (chan_num < 1  ||  chan_num > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (mode < 0 || mode > TIMER_MODE_PWM)
       return (ERR_PWM_INVALID_TIMER_MODE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       //-------------------------------------------------------------------
       //    Setup associated GPIO(s) into Timer/PWM mode via SELxx bits
       //
       // Convert PWM channel number to an index into table of 12 entries
       // that contains pin # and GPIO base index, then use  _g_gpio_base[]
       //-------------------------------------------------------------------
    if (module_id == PWM_MODULE_0)
       pwmcblk = (PWM_CHANNEL_BLK*) &_g_tmpwm_mod_0_channels [chan_num];         // PWM_MODULE_0
       else if (module_id == PWM_MODULE_1)
               pwmcblk = (PWM_CHANNEL_BLK*) &_g_tmpwm_mod_1_channels [chan_num]; // PWM_MODULE_1
       else if (module_id == PWM_MODULE_2)
               pwmcblk = (PWM_CHANNEL_BLK*) &_g_tmpwm_mod_2_channels [chan_num]; // PWM_MODULE_2
       else pwmcblk = (PWM_CHANNEL_BLK*) &_g_tmpwm_mod_3_channels [chan_num];    // PWM_MODULE_3

    if (flags & TIMER_CCR_TIMER_ONLY)
       ;       // do not turn on GPIOs if TIMER_ONLY mode
      else
       {       // User wants GPIOs turned on - do it
         gpio_base_addr = _g_gpio_port_base_address[pwmcblk->pwm_gpio_port_id];
               // set associated SEL1 / SEL0 pins to denote this is being used
               // as a PWM channel  (aka TERTIARY function)
         HWREG16(gpio_base_addr + OFS_PADIR)  |= pwmcblk->pwm_gpio_pin_bit;    // set for Output
         HWREG16(gpio_base_addr + OFS_PASEL0) |= pwmcblk->pwm_gpio_pin_bit;    // Set for SECONDARY
         HWREG16(gpio_base_addr + OFS_PASEL1) &= ~(pwmcblk->pwm_gpio_pin_bit); //   function
       }

       //-------------------------------------------
       // do any needed pre-scaling on duty_cycle
       //-------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0)
       { initial_duty = (initial_duty / _g_tmpwm_prescalars[module_id]);
       }

    chan_mask = _g_pwm_chan_mask [chan_num];  // get associated channel mask

       //--------------------------------------------------------------
       // turn on any needed polarity or interrupt flags for this CCR
       //--------------------------------------------------------------
    ccr_ctl_value = mode;
    if ((flags & TIMER_PIN_POLARITY_LOW) == 0)
       ccr_ctl_value |= OUT;         // invert pin's polarity
    if (flags & TIMER_ENABLE_CCR_INTERRUPTS)
       ccr_ctl_value |= CCIE;        // enable interrupt when CCR value is hit

       //---------------------------------------------------------
       // for certain OC modes (e.g. OCMODE_4, ...), we need to
       // reload the duty value after every interrupt.
       //---------------------------------------------------------
    if (mode == TIMER_MODE_OC_TOGGLE || mode == TIMER_MODE_OC_RESET)
       reload_value = initial_duty;
       else reload_value  = 0;

      //-------------------------------------------------------------------
      // Configure the Timer module's desired CCR channel for PWM / OC / ...
      //
      // We support 4 TA timers for Timer/PWM usage:  TA0, TA1, TA2 and TA3.
      // Each timer supports up to 6 channels via CCRs 1-6.
      // We reserve CCR0 for holding the Timer/PWM module's Max Period.
      //-------------------------------------------------------------------
    switch (chan_num)
       { case 1:                        //   TAn  CCR1
               HWREG16(timer_base_addr + OFS_TA0CCTL1) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR1)  = initial_duty;
               rupt_mask = 0x0001;
               break;
         case 2:                        //   TAn  CCR2
               HWREG16(timer_base_addr + OFS_TA0CCTL2) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR2)  = initial_duty;
               rupt_mask = 0x0002;
               break;
         case 3:                        //   TAn  CCR3
               HWREG16(timer_base_addr + OFS_TA0CCTL3) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR3)  = initial_duty;
               rupt_mask = 0x0004;
               break;
         case 4:                        //   TAn  CCR4
               HWREG16(timer_base_addr + OFS_TA0CCTL4) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR4)  = initial_duty;
               rupt_mask = 0x0008;
               break;
         case 5:                        //   TAn  CCR5
               HWREG16(timer_base_addr + OFS_TA0CCTL5) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR5)  = initial_duty;
               rupt_mask = 0x0010;
               break;
         case 6:                        //   TAn  CCR6
               HWREG16(timer_base_addr + OFS_TA0CCTL6) = ccr_ctl_value;
               HWREG16(timer_base_addr + OFS_TA0CCR6)  = initial_duty;
               rupt_mask = 0x0020;
               break;
       }

// ??? is this NVIC_RESET really needed ??? for every Timer ??? !!!
    if (module_id == 0)
       { _g_timer_A0_reload_value_array[chan_num] = reload_value;  // save any needed reload value

            // clear any old TA0 interrupt flags
         NVIC_ISER0 |= 1 << ((INT_TA0_N - 16) & 0x1F);
         NVIC_ISER0 |= 1 << ((INT_TA0_0 - 16) & 0x1F);
       }
      else if (module_id == 1)
              { _g_timer_A1_reload_value_array[chan_num] = reload_value;  // save any needed reload value

                    // clear any old TA1 interrupt flags
                NVIC_ISER0 |= 1 << ((INT_TA1_N - 16) & 0x1F);
                NVIC_ISER0 |= 1 << ((INT_TA1_0 - 16) & 0x1F);
              }
      else if (module_id == 2)
              { _g_timer_A2_reload_value_array[chan_num] = reload_value;  // save any needed reload value

                    // clear any old TA2 interrupt flags
                NVIC_ISER0 |= 1 << ((INT_TA2_N - 16) & 0x1F);
                NVIC_ISER0 |= 1 << ((INT_TA2_0 - 16) & 0x1F);
              }
      else
              { _g_timer_A3_reload_value_array[chan_num] = reload_value;  // save any needed reload value

                    // clear any old TA3 interrupt flags
                NVIC_ISER0 |= 1 << ((INT_TA3_N - 16) & 0x1F);
                NVIC_ISER0 |= 1 << ((INT_TA3_0 - 16) & 0x1F);
              }

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
//         The trigger field is specified in the ADC14SHSxx bits of the
//         ADC14CTL0 register.
//         For the MSP432 these are:
//
//                 Timer/ CCR      SHS Value    (from MSP432 datasheet p 91)
//                  TA0 / CCR1        1
//                  TA0 / CCR2        2
//                  TA1 / CCR1        3
//                  TA1 / CCR2        4
//                  TA2 / CCR1        5
//                  TA2 / CCR1        6
//                  TA3 / CCR1        7
//
// flags parm can have TIMER_TRIGGER_MODE set, indicating to ensure that the
//       CCR OUTMOD is set to some kind of toggle (MOD_3 / MOD_6/ MOD_7) state.
//*****************************************************************************
int  board_timerpwm_config_trigger_mode (int module_id, int trigger_type,
                                         int flags)
{
    int    rc;
    int    trig_timer_mod_id;
    int    channel_num;
    int    chan_mask;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (trigger_type != ADC_TRIGGER_USER_APP)
       {      //-------------------------------------------------------------
              // Setup SHS trigger field in ADC14CTL0
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
          if (channel_num == TMR_PWM_CHANNEL_1)
             chan_mask = 0x01;        // Channel 1
             else chan_mask = 0x02;   // Channel 2

          if ((_g_tmpwm_channels_config [module_id] & chan_mask) == 0)
             {       // Associated CCR has not been configured. Set it up with default
                     // values. Set default CCR value = Timer's period (CCR0)
               board_timerpwm_config_channel (module_id, channel_num,
                                              board_timerpwm_get_period(module_id),
                                              TIMER_MODE_OC_SET_RESET,
                                              TIMER_CCR_TIMER_ONLY);
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
    uint32_t    timer_base_addr;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       // Disable PWM counting, by turning off the MC bits in the CTL register
    HWREG16(timer_base_addr + OFS_TA0CTL) &= ~(MC_3);

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
    uint32_t   timer_base_addr;
    int        count_mode;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (_g_tmpwm_module_status [module_id] == TIMER_PERIODIC_COUNT_UP)
       count_mode = MC_1;                             // up mode
       else count_mode = MC_3;                        // up/down mode

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       //------------------------------------------------------------
       // CCR Interrupts: each CCR's interupt flag CCIE was enabled in
       // board_timerpwm_config_channel(), so no additional work is needed in here.
       //------------------------------------------------------------

       //------------------------------------------------------------
       // Enable the Timer/PWM counting, by turning on the MC bits 
       // in the CTL register:  SMCLK + MC count + clear TAR
       //------------------------------------------------------------
    HWREG16(timer_base_addr + OFS_TA0CTL) = TASSEL__SMCLK | count_mode | TACLR;

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
    uint32_t      timer_base_addr;
    int           mode;
    int           flags;
    int           rc;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

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
    uint32_t   timer_base_addr;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

    return (HWREG16(timer_base_addr + OFS_TA0R));  // pass back curr counter value
}


//*****************************************************************************
//  board_timerpwm_get_CCR_capture_value
//
//         Gets the recorded input capture value in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_CCR_capture_value (int module_id, int chan_id)
{
    uint32_t   timer_base_addr;
    long       cap_value;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (chan_id < 1  ||  chan_id > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       // pull capture value from the correct CCR, based on 1-6 index
    switch (chan_id)
      { case 1:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR1);
             break;
        case 2:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR2);
             break;
        case 3:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR3);
             break;
        case 4:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR4);
             break;
        case 5:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR5);
             break;
        case 6:
             cap_value = HWREG16(timer_base_addr + OFS_TA0CCR6);
             break;
      }

// ??? !!!  WVD  must I adjust the value based on pre-scalar used ??? !!!

    return (cap_value);             // passback capture value
}


//*****************************************************************************
//  board_timerpwm_get_duty_cycle
//
//         Get the Timer/PWM's current duty cycle.     Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_duty_cycle (int module_id, int chan_id)
{
    uint32_t   timer_base_addr;
    long       duty_cycle;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (chan_id < 1  ||  chan_id > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

       // pull duty cycle from the correct CCR, based on 1-6 index
    switch (chan_id)
      { case 1:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR1);
             break;
        case 2:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR2);
             break;
        case 3:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR3);
             break;
        case 4:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR4);
             break;
        case 5:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR5);
             break;
        case 6:
             duty_cycle = HWREG16(timer_base_addr + OFS_TA0CCR6);
             break;
      }

    return (duty_cycle);
}


//*****************************************************************************
//  board_timerpwm_get_period
//
//         Get current Timer/PWM Period.         Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_period (int module_id)
{
    uint32_t   timer_base_addr;
    long       period_val;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

    period_val = HWREG16(timer_base_addr + OFS_TA0CCR0);  // grab CCR0 = period

    return (period_val);
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
int  board_timerpwm_reset_CCR_output (int module_id, int chan_num, int flags)
{
    uint32_t    timer_base_addr;
    int         mode;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

    mode = OUTMOD_5;                    // config CCR to shutoff GPIO   or use OUTMOD_0 ?
    switch (chan_num)
       { case 1:                        //   TAn  CCR1
               HWREG16(timer_base_addr + OFS_TA0CCTL1)  = mode;
               break;
         case 2:                        //   TAn  CCR2
               HWREG16(timer_base_addr + OFS_TA0CCTL2) = mode;
               break;
         case 3:                        //   TAn  CCR3
               HWREG16(timer_base_addr + OFS_TA0CCTL3) = mode;
               break;
         case 4:                        //   TAn  CCR4
               HWREG16(timer_base_addr + OFS_TA0CCTL4) = mode;
               break;
         case 5:                        //   TAn  CCR5
               HWREG16(timer_base_addr + OFS_TA0CCTL5) = mode;
               break;
         case 6:                        //   TAn  CCR6
               HWREG16(timer_base_addr + OFS_TA0CCTL6) = mode;
               break;
       }

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
    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
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

int  board_timerpwm_set_duty_cycle (int module_id, int chan_num, long duty_cycle,
                                    int flags)
{
    uint32_t   timer_base_addr;
    uint32_t   scaleduty;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (chan_num < 1  ||  chan_num > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

      //--------------------------------------------------------
      // Update the desired CCR channel in the Timer/PWM module
      //--------------------------------------------------------

    switch (chan_num)
      { case 1:
              HWREG16(timer_base_addr + OFS_TA0CCR1) = duty_cycle;
              break;
        case 2:
              HWREG16(timer_base_addr + OFS_TA0CCR2) = duty_cycle;
              break;
        case 3:
              HWREG16(timer_base_addr + OFS_TA0CCR3) = duty_cycle;
              break;
        case 4:
              HWREG16(timer_base_addr + OFS_TA0CCR4) = duty_cycle;
              break;
        case 5:
              HWREG16(timer_base_addr + OFS_TA0CCR5) = duty_cycle;
              break;
        case 6:
              HWREG16(timer_base_addr + OFS_TA0CCR6) = duty_cycle;
              break;
      }

    return (0);                  // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_set_period
//
//         Sets the timer/PWM to use a new period value.
//*****************************************************************************

int  board_timerpwm_set_period (int module_id, long period_val, int flags)
{
    uint32_t   timer_base_addr;
    uint32_t   scaleperiod;

    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    timer_base_addr = _g_timer_base_address [module_id];  // get approp TA0-TA3

// ??? probably have to re-divide period based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;  // get Master PWM Clock value
//  scaleperiod = (_g_PWM_IO_clock / period_val) - 1;    // set new scaled period

       // set new Timer/PWM period into Timer's CCR0 register 
    HWREG16(timer_base_addr + OFS_TA0CCR0);

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
    if (module_id < 0  ||  module_id >= TMPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (channel_num < 1  ||  channel_num > TMPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);


      // ??? is this supported on MSP432 - perhaps tweak TAxCCTLx = OUTMOD_
      //     to do set/reset instead of reset/set ?


//  return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    return (0);              // denote completed successfully
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
                               int  timer_reload_array[])
{
    uint32_t      rupt_status;
   register  int  app_rupts_mask;

    rupt_status = HWREG16(timer_base_addr + OFS_TA0IV);

       //--------------------------------------------------------------
       // get a local copy of what interrupts the User App wants
       // to be called back on. In optimized compilers, this should
       // get loaded into a local register.
       //--------------------------------------------------------------
    app_rupts_mask = _g_timer_app_enabled_interrupts [tim_index];

       //---------------------------------------------------------------------
       // We sequentially walk down each key interrupt type, so that if
       // multiple interrupts are signalled at the same time, we handle them.
       //---------------------------------------------------------------------
    if (rupt_status = 0x02  &&  app_rupts_mask == 0x0001)
       {     // Handle CCR1 interrupt
         HWREG16(timer_base_addr + TA0CCTL1)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[1] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[1]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR1_INTERRUPT);
            }
       }
    if (rupt_status = 0x04  &&  app_rupts_mask == 0x0002)
       {     // Handle CCR2 interrupt
         HWREG16(timer_base_addr + TA0CCTL2)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[2] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[2]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR2_INTERRUPT);
            }
       }
    if (rupt_status = 0x06  &&  app_rupts_mask == 0x0004)
       {     // Handle CCR3 interrupt
         HWREG16(timer_base_addr + TA0CCTL3)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[3] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[3]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR3_INTERRUPT);
            }
       }
    if (rupt_status = 0x08  &&  app_rupts_mask == 0x0008)
       {     // Handle CCR4 interrupt
         HWREG16(timer_base_addr + TA0CCTL4)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[4] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[4]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR4_INTERRUPT);
            }
       }
    if (rupt_status = 0x0A  &&  app_rupts_mask == 0x0010)
       {     // Handle CCR5 interrupt
         HWREG16(timer_base_addr + TA0CCTL5)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[5] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[5]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR5_INTERRUPT);
            }
       }
    if (rupt_status = 0x0C  &&  app_rupts_mask == 0x0020)
       {     // Handle CCR6 interrupt
         HWREG16(timer_base_addr + TA0CCTL6)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[6] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[6]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_CCR6_INTERRUPT);
            }
       }
    if (rupt_status = 0x0C  &&  app_rupts_mask == 0x0020)
       {     // Handle UPDATE / rollover CCR0 interrupt
         HWREG16(timer_base_addr + TA0CCTL0)  &= ~CCIFG;  // Ack/Clear the interrupt
         if (timer_reload_array[0] != 0)
            HWREG16(timer_base_addr + OFS_TA0CCR1) = timer_reload_array[0]; // reload value for next pass
         if (_g_ptimer_callback[tim_index] != 0L)
            {
              (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                               TIMER_ROLLOVER_INTERRUPT);
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
void Timer_A0_Base_IsrHandler (void)          // Handle period rollover
{
    timer_common_IRQHandler (_g_timer_base_address[0], 0,
                             _g_timer_A0_reload_value_array);
}

void  Timer_A0_CCR_IsrHandler (void)         // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timer_base_address[0], 0,
                             _g_timer_A0_reload_value_array);
}

void Timer_A1_Base_IsrHandler (void)          // Handle period rollover
{
    timer_common_IRQHandler (_g_timer_base_address[1], 1,
                             _g_timer_A1_reload_value_array);
}

void  Timer_A1_CCR_IsrHandler (void)         // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timer_base_address[1], 1,
                             _g_timer_A1_reload_value_array);
}

void Timer_A2_Base_IsrHandler (void)          // Handle period rollover
{
    timer_common_IRQHandler (_g_timer_base_address[2], 2,
                             _g_timer_A2_reload_value_array);
}

void  Timer_A2_CCR_IsrHandler (void)         // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timer_base_address[2], 2,
                             _g_timer_A2_reload_value_array);
}

void Timer_A3_Base_IsrHandler (void)          // Handle period rollover
{
    timer_common_IRQHandler (_g_timer_base_address[3], 3,
                             _g_timer_A3_reload_value_array);
}

void  Timer_A3_CCR_IsrHandler (void)         // Handle CCRx value reached
{
    timer_common_IRQHandler (_g_timer_base_address[3], 3,
                             _g_timer_A3_reload_value_array);
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
//               MSP432 ARM Launchpad default Console UART port is USCI_A0
//
//               The code is hardwired for 115,200 baud rate.
//               Use the lookup tables in the MSP432's TECH REF to select the
//               UCA1BR0 and UCBRS (UCA1MCTL) values for different Baud rates.
//
//               We support 4 different MCLK clock rates:
//                  1.000 Mhz (default startup), 12.0, 24.0, and 48.0 MCLK
//*****************************************************************************

void  board_uart_init (void)
{

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)

    unsigned short  baud_UCBR0,   baud_CTLW0;

        //----------------------------------------------------------------------
        // Generate Baud Rate pre-scalars, based on fixed clock rates we support
        //----------------------------------------------------------------------
    if (_g_SMCLK >= 48000000)
       {              // assume MCLK is set to 48.000 MHz     BUT WON'T SMCLK be 1/2 (24 MHz max ?)
               baud_UCBR0 = 416;     // 48000000/115200 = 416.66  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0xD6, UCBRFx=?
               baud_CTLW0 = 0xD600 ; //+ UCBRF_0;
       }
      else if (_g_SMCLK >= 24000000)
             {        // assume MCLK is set to 24.000 MHz
               baud_UCBR0 = 208;     // 24000000/115200 = 208.33  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0x25, UCBRFx=?
               baud_CTLW0 = 0x2500 ; // + UCBRF_0;   COMPILER SAYS NO SUCH CONST
             }
      else if (_g_SMCLK >= 12000000)
             {        // assume MCLK is set to 12.000 MHz
               baud_UCBR0 = 6;       // 12000000/115200 = 104.16  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0x20, UCBRFx=8
               baud_CTLW0 = 0x2000 + UCOS16; // + UCBRF_8 + UCOS16;
             }
      else   {        // assume MCLK is 3.000 MHz - the default startup clock
               baud_UCBR0 = 26;      // 3000000/115200 = 26.04
               baud_CTLW0 = 0x0000;  // Table: Modulation UCBRSx=0x00, UCBRFx=0
             }

        //-------------------------------------------------
        //  setup GPIO pins P1.2 and P1.3 to use for UART
        //-------------------------------------------------
        // Set GPIOs P1.2 and P1.3 into UART mode
//  MAP_GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P1,
//                                                  GPIO_PIN2 | GPIO_PIN3, 
//                                                  GPIO_PRIMARY_MODULE_FUNCTION);
    P1SEL0 |= BIT2 | BIT3;   // set P1.2/P1.3 for use as USCI_A0 UART TXD/RXD

        //-------------------------------------------------
        //            Configure UART
        //-------------------------------------------------
    UCA0CTLW0 |= UCSWRST;                     // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // use SMCLK and 8N1 (default)

        //-------------------------------------------------
        //            setup  Baud  Rate
        //-------------------------------------------------
       // Baud Rate calculation
    UCA0BR0   = baud_UCBR0;                   // set basic baud rate pre-scalar
//  UCA0BR0    = 26;                          // 3000000/115200 = 26.042
    UCA0BR1    = 0;
    UCA0MCTLW = baud_CTLW0;                   // set baud Modulation and UCOS values
//  UCA0MCTLW  = 0x0000;                      // 3000000/115200 - INT(3000000/115200)=0.042
                                              // UCBRSx value = 0x00 (See UG)
    UCA0CTLW0 &= ~UCSWRST;                    // Enable and Initialize eUSCI

#endif

      // Baud Rate calculation for a 12 MHz CPU Clock
      // 12000000/(16*9600) = 78.125
      // Fractional portion = 0.125
      // User's Guide Table 21-4: UCBRSx = 0x10
      // UCBRFx = int ( (78.125-78)*16) = 2
  //UCA0BR0 = 78;                           // 12000000/16/9600
  //UCA0BR1 = 0x00;
  //UCA0MCTLW = 0x1000 | UCOS16 | 0x0020;

      // Baud Rate calculation for a 3 MHz CPU clock
//  UCA0BR0   = 26;                         // 3000000/115200 = 26.042
//  UCA0MCTLW = 0x0000;                     // 3000000/115200 - INT(3000000/115200)=0.042
//                                          // UCBRSx value = 0x00 (See User Guide)
//
//  UCA0BR0   = 416;                        // 48000000/115200 = 416
// 
//  UCA0BR1 = 0;

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

//  rc = UART_getInterruptStatus (EUSCI_A0_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA0IFG & UCRXIFG;
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

//#if defined(USES_CONSOLE_READ) || defined(USES_CONSOLE_WRITE)
    char  in_char;
    int   rc;

           // read in any character that the user typed in
    rc = UCA0IFG & UCRXIFG;
    if ( ! rc)
       return (0);                                 // no UART data is present

//  in_char = UART_receiveData (EUSCI_A0_MODULE);  // read in char from UART
    in_char  = UCA0RXBUF;                          // read in char from uART

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
//#endif

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
//  rc = UART_getInterruptStatus (EUSCI_A0_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA0IFG & UCRXIFG;
    if ( ! rc)
       continue;                            // user needs to type in more chars

//  in_char = UART_receiveData (EUSCI_A0_MODULE);   // read in char from uART
    in_char  = UCA0RXBUF;                   // read in char from uART

    if (in_char <= 0)
       return (0);        // no character or error occurred. bail out

       // note: board_uart_read MUST ALSO echo back the char to user
    if (in_char != '\n')
       board_uart_write_char (in_char);      // echo the char (\n is special)
//  EUSCI_A_UART_transmitData (EUSCI_A0_MODULE, in_char);   // echo the char
//  while ( ! (UCA0IFG&UCTXIFG))   ;         // wait till TX buffer is empty
//    UCA0TXBUF = in_char;                   // echo the rcvd char


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
           else board_uart_write_char ('\n');   // no preceding \r, so echo \n
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

//#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)
//  UART_transmitData (EUSCI_A0_MODULE, outchar);   // send a char
    while ( ! (UCA0IFG & UCTXIFG))   ;          // wait till TX buffer is empty
      UCA0TXBUF = outchar;                      // send a char
//#endif

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
//      UART_transmitData (EUSCI_A0_MODULE, *outstr);   // send a char
        while ( ! (UCA0IFG & UCTXIFG))   ;       // wait till TX buffer is empty
           UCA0TXBUF = *outstr;                  // send a char

        outstr++;                                // step to next char in buffer
      }
#endif

}


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

//#if defined(USES_MQTT)


//#enduf

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

//#if defined(USES_MQTT)

//#enduf

}



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
    uint32_t        _g_vtimer_expire [10];
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

void  board_vtimer_check_expiration (uint32_t gsystick_millisecs)
{
    int   i,  expired;

          //-------------------------------------------------------------
          // See if any virtual timers have reached their expiration time
          //-------------------------------------------------------------
    for (i = 0; i < 10;  i++)
      {
        if (_g_vtimer_flags[i] == VTIMER_RESET)
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
                   // This keeps the VTIMER firing until user issues vtimer_reset() to cancel it
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

          // Add VTIMER mills to current SYSTICK value. If it
          // wraps around, we depend upon the expiration logic to handle it.
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

     if (_g_vtimer_flags[vtimer_id] == VTIMER_BUSY)
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
//
//         Called by: sl_Start() in simplelink device.c 
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
//
//            Called by: sl_Start() in simplelink device.c 
//--------------------------------------------------------------------
void  CC3100_disable()
{
        // MSP430_F5529 = P1.6 --> P1OUT &= ~BIT6;
	    // MSP432_P401R = P4.1
	P4OUT &= ~BIT1;              // set CC3100 nHIB to DISABLED
}

//-------------------------------------------------------------------
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
//--------------------------------------------------------------------
void  CC3100_enable()
{
	    // MSP430_F5529 = P1.6 --> P1OUT |= BIT6;
	    // MSP432_P401R = P4.1
	P4OUT |= BIT1;               // set CC3100 nHIB to ENABLED
}


//-------------------------------------------------------------------
// CC3100_InterruptEnable
//
//            Enables the interrupt (IRQ) from the CC3100 on P 2.5
//--------------------------------------------------------------------
void  CC3100_InterruptEnable()
{
	    // MSP430_F5529 P2.0,   MSP432_P401R = P2.5
    P2IES &= ~BIT5;
    P2IE  |= BIT5;
    Interrupt_enableInterrupt (INT_PORT2);
    Interrupt_enableMaster();
}


//-------------------------------------------------------------------
// CC3100_InterruptDisable
//
//            Disables the interrupt (IRQ) from the CC3100 on P 2.5
//--------------------------------------------------------------------
void  CC3100_InterruptDisable()
{
	    // MSP430_F5529 P2.0,   MSP432_P401R = P2.5
    P2IE &= ~BIT5;
    Interrupt_disableInterrupt (INT_PORT2);
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
//                      CC3100  Antenna  Select  logic
//-------------------------------------------------------------------
void  initAntSelGPIO (void)
{
    P2OUT  &= ~BIT7;
    P2OUT  |=  BIT6;           /* Select Antenna 1 */
    P2SEL0 &= ~(BIT6 + BIT7);
    P2SEL1 &= ~(BIT6 + BIT7);
    P2DIR  |= (BIT6 + BIT7);
}

void  SelAntenna (int antenna)
{
    switch (antenna)
      {
        case ANT1:
            P2OUT &= ~BIT7;
            P2OUT |=  BIT6;
            break;
        case ANT2:
            P2OUT &= ~BIT6;
            P2OUT |=  BIT7;
            break;
      }
}


//-------------------------------------------------------------------
//                  CC3100   IRQ   ISR   Handler            uses P2.5
//-------------------------------------------------------------------

//#pragma vector=PORT2_VECTOR
//__interrupt
void  Port2_ISR (void)
{
    if (P2IFG & BIT5)
      {
        if (pIraEventHandler)
           {
             pIraEventHandler(0);
           }

        P2IFG &= ~ BIT5;              // clear HW Interrupt Flag
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
//  ADCSequenceDataGet (ADC0_BASE, 3, &g_ADC_value); //read converted value
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
    return (g_ADC_value);
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
///     case TB0IV_8: break;            // Vector 8: Reserved CCIFG - Not used FR5969

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
#if MSP430_ONLY_SUPPORT
#pragma vector=PORT2_VECTOR, ADC10_VECTOR, \
        USCIAB0TX_VECTOR, TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, \
        COMPARATORA_VECTOR, NMI_VECTOR
__interrupt void  Trap_ISR (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}
#endif                       // MSP430_ONLY_SUPPORT

