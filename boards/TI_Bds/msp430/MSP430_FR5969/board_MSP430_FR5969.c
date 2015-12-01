
// 05/02/15 - NEED TO RESOLVE TA1.1 vs TB0 CONFLICT FOR PWMOUT on J2 connector !
//            Use Timer3_A2 !   (is on FR5969)

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
//                           board_MSP430_FR5969.c
//
// Board dependent code, moved to a single module for MSP430 FR5969 Launchpad
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
//   11/29/14 - Created.
//   12/11/14 - Works properly after fixing CS issues and Pull-Ups issues.
//   04/27/15 - Added ADC12 sequence of channels support to optimize ADC. Duqu 
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
#endif

#if defined(USES_DRV8711)
#include "devices/DRV8711_Spin_Routines.h"
#endif


//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_MCLK          = 0; // CPU MCLK setting
    uint32_t  _g_SMCLK         = 0; // peripheral SMCLK setting
    uint32_t  _g_TA1_1ms_ticks = 0; // number of SMCLK ticks in 1 millisecond

    uint32_t  _g_systick_millisecs = 0;   // used by "Systick" emulator logic.
                                          // provides a 1 ms periodic timer pop

    char      _g_vtimers_active = 0;      // Optional VTIMER support

    P_EVENT_HANDLER   pIraEventHandler = 0;
    uint8_t           IntIsMasked;

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    uint32_t  _g_crcid_Result;
#endif

                 // mainly for DRV8711 logic
    uint32_t  _g_ADC_value = 0;

#if (USES_ADC)
    ADC12_B_configureMemoryParam  _g_ADC_param = {0};
#endif

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ)
    char      _g_uart_buf_idx   = 0;    // keeps track of user input via UART
    char      _g_uart_last_char = 0;
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
              __MSP430_BASEADDRESS_PORTJ_R__      // Port J
            };
#define  GP_PORT_1   1        // indexes into above table, based on port id
#define  GP_PORT_2   2
#define  GP_PORT_3   3
#define  GP_PORT_4   4
#define  GP_PORT_J   5



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
       else board_system_clock_config (8000000);   // use default: run at 8 MHz

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

        // take the current clock frequency, and compute the associated ticks
        // needed to generate that frequency.
     pticks = _g_SMCLK / frequency;

     return (pticks);
}



#if defined(USES_ADC)

//*****************************************************************************
//*****************************************************************************
//                               ADC   Routines
//*****************************************************************************
//*****************************************************************************

    int                _g_ADC_complete   = 0;      // 1 = all ADC conversions/DMAs are complete
    char               _g_DMA_overrun    = 0;      // 1 = DMA completed a new set before the previous one was processed
    char               _g_adc_configured = 0;
    char               _g_trigger_source = 0;
    char               _g_num_configured_channels = 0;
    char                _g_step_num      = 0;

    unsigned char      _g_adc_step_map [32];       // indexed by channel number
    unsigned short     _g_adc_conv_results [32];   // Internal buffer to hold DMA results

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
      ADC12_B_MEMORY_9
    };


typedef struct adc_channel_def        /* ADC Channel definitions */
    {
        uint32_t  chan_gpio_port_id;  /* Associated GPIO port    */
        uint32_t  chan_gpio_pin_bit;  /* Associated GPIO pin     */
        uint8_t   chan_adc_num;       /* Associated ADC12_B_INPUT_x channel num */
        uint8_t   chan_mem_num;       /* Associated ADC12_B_MEMORY_x memory num */
        uint8_t   chan_even;          /* 1 = channel PORT is even ==> GPIO shift*/
    } ADC_CHANNEL_BLK;

const ADC_CHANNEL_BLK  _g_adc_channels [] =  //                                 User   Energia     LP   Grove
        {  { GP_PORT_1, BIT2, ADC12_B_INPUT_A2, ADC12_B_MEMORY_0, 0 }, // P 1.2 Adc2   A23   23    J2-2
           { GP_PORT_1, BIT3, ADC12_B_INPUT_A3, ADC12_B_MEMORY_1, 0 }, // P 1.3 Adc3   A24   24    J2-10  J7
           { GP_PORT_1, BIT4, ADC12_B_INPUT_A4, ADC12_B_MEMORY_2, 0 }, // P 1.4 Adc4   A25   25    J2-9   J8
           { GP_PORT_1, BIT5, ADC12_B_INPUT_A5, ADC12_B_MEMORY_3, 0 }, // P 1.5 Adc5   A26   26    J2-8   J9
           { GP_PORT_2, BIT4, ADC12_B_INPUT_A7, ADC12_B_MEMORY_4, 1 }, // P 2.4 Adc7   A27   27    J1-6   J6
           { GP_PORT_4, BIT2, ADC12_B_INPUT_A10,ADC12_B_MEMORY_5, 1 }, // P 4.2 Adc10  A2     2    J1-2   J5
           { GP_PORT_4, BIT3, ADC12_B_INPUT_A11,ADC12_B_MEMORY_6, 1 }, // P 4.3 Adc11  A6     6    J1-5
           { GP_PORT_3, BIT0, ADC12_B_INPUT_A12,ADC12_B_MEMORY_7, 0 }  // P 3.0 Adc12  A28   28    J2-3
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC12_B_INPUT_TEMPSENSOR },  // internal Temperature Sensor
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC12_B_INPUT_BATSENSOR },   // internal Battery Monitor Sensor
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
// FR5969 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC12MCTL0 -> ADC12MCTL15.
// ADC results are stored in corresponding registers: ADC12MEM0 -> ADC12MEM15.
// Results are stored in the 16-bit ADC12MEMx registers.
//
// The Launchpad MSP430 FR5969 pin device physically supports up to 16 channels,
// of which 8 are wired out to the Launchpad pins, and 2 internal channels
// (Temperature Sensor, Battery Monitor) are available.
// Due to this layout, we support 10 total channels.
//*****************************************************************************
int  board_adc_init (int adc_module_id, uint32_t clock_rate, 
                     int trigger_type, int flags)
{
    ADC12_B_initParam initParam  =  {0};


// ??? !!!  WVD   ??? !!!  Add suppoert for Trigger_type


    _g_trigger_source = (char) trigger_type;

    if (_g_adc_configured == 0)
       {
       //Initialize the ADC12B Module
       /*
       * Use internal ADC12B bit as sample/hold signal to start conversion
       * USE MODOSC 5MHZ Digital Oscillator as clock source
       * Use default clock divider/pre-divider of 1
       * Not use internal channel
       */
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect            = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider           = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider        = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap           = ADC12_B_NOINTCH;
    ADC12_B_init (ADC12_B_BASE, &initParam);
       /*
       *  For memory buffers 0-7  sample/hold for 64 clock cycles
       *  For memory buffers 8-15 sample/hold for 4 clock cycles (default)
       *  Disable Multiple Sampling
       */
    ADC12_B_setupSamplingTimer (ADC12_B_BASE,
                                ADC12_B_CYCLEHOLD_16_CYCLES,
                                ADC12_B_CYCLEHOLD_4_CYCLES,
                                ADC12_B_MULTIPLESAMPLESDISABLE);

// ADC12CTL0 = ADC12SHT0_2 | ADC12ON;        // Sampling time, S&H=16, ADC12 on
// ADC12CTL1 = ADC12SHP;                     // Use sampling timer
// ADC12CTL2 |= ADC12RES_2;                  // 12-bit conversion results

             //--------------------------------------
             // do basic configuration of ADC module
             //--------------------------------------
             // CONSEQ_1 = single Sequence-of-Channels operation
         ADC12CTL0  = ADC12ON  + ADC12SHT0_2 + ADC12MSC; // Turn on ADC12, set sampling time
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

// P1SEL1 |= BIT5 | BIT4 | BIT3 | BIT2;       // Enable A/D channel A2-A5 for GROVE CODE
// P1SEL0 |= BIT5 | BIT4 | BIT3 | BIT2;       // P 1.2, P 1.3, P 1.4, P 1.5   WORKS !


          //----------------------------------------------------------------
          //         Setup the step within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save original step number passed in
    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

//  Step 1: use hardcodecd value
//  Step 2: convert to generic use, using ADCx lookup table, etc
// ADC12MCTL0 = ADC12INCH_2;                 // ref+=AVcc, channel = A0
// ADC12MCTL1 = ADC12INCH_3;                 // ref+=AVcc, channel = A1  Grove J6 P 6.1
// ADC12MCTL2 = ADC12INCH_4;                 // ref+=AVcc, channel = A2
// ADC12MCTL3 = ADC12INCH_5 + ADC12EOS;      // ref+=AVcc, channel = A3, end seq.

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

    _g_num_configured_channels++;  // inc # channels we have configured for the ADC

    return (0);           // denote success
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
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//          To ensure cross-platform compatibility, we also kick off
//          an initial ADC conversion, if we are configured for USER SW Trigger
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    unsigned short  ie_mask;

    _g_ADC_complete = 0;                 // clear I/O flags
    _g_DMA_overrun  = 0;

// ADC12IER0  = ADC12IE3;                // Enable ADC12 IFG.3 interrupts

// ADC12_B_clearInterrupt (ADC12_B_BASE, 0, ADC12_B_IFG0);
// ADC12_B_enableInterrupt (ADC12_B_BASE, ADC12_B_IE0, 0, 0);

    ie_mask = 1 << (_g_num_configured_channels - 1);
    ADC12IFGR0 &= ~(ie_mask);            // Clear any old ADC12 IFG.x interrupts
    ADC12IER0   = ie_mask;               // Enable ADC12 IFG.x interrupts
                                         //        based on last EOS channel.

    ADC12CTL0 |= ADC12ENC;               // Enable conversions
    ADC12CTL0 |= ADC12ON;                // turn on the ADC module for sampling

#if (TO_DO)
    if (_g_trigger_source == TRIGGER_USER_APP)
#endif
       board_adc_user_trigger_start (0, 0);  // kick off initial Conversion

    return (0);                          // denote success
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

    ADC12CTL0 &= ~(ADC12ON);         // turn off the ADC module for sampling

    return (0);                      // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer,
                            uint16_t  channel_results[])
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
    for (i = 0;  i < _g_num_configured_channels;  i++)
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

    for (i = 0;  i < _g_num_configured_channels;  i++)
       {       // copy the internally ADC/DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_ADC_complete = 0;                 // reset for new pass

    return (_g_num_configured_channels); // pass back number of completed conversions
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


/****************************************************************************
  * @brief  ADC Conversion complete callback          (in non blocking mode)
  *
  *         If running straight interrupts (xxx_IT), this gets called by
  *         the HAL ADC library support when the ADC interrupt completes.
  *
  *         If running with DMA (xxx_DMA), this gets called
  *         when the DMA interrupt completes, i.e. this gets
  *         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
  *
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion,
  *         and you can add your own implementation.
  * @retval None
****************************************************************************/

#ifndef TRIGGER_TEST
// ADC12 interrupt service routine
#pragma vector=ADC12_VECTOR
__interrupt void  ADC12_ISR (void)
{
    int             i;
    unsigned short  *adc_mem_data;

    if (ADC12IV > 6 && ADC12IV <= 30)
       {     // this is the ending IFG0 - IFG14 that signals end of sequence
             // completed.  Move converted results to our internal staging buf.
        adc_mem_data = &ADC12MEM0;       // point at begin of ADC results memory buf
        for (i = 0;  i < _g_num_configured_channels;  i++)
          {       // copy the internally DMA staged results into our staging buffer.
            _g_adc_conv_results[i] = *adc_mem_data;  // copy data to internal staging area
            adc_mem_data++;                          // step to next ADC12MEMx
          }
        _g_ADC_complete = 1;   // set status that ADCs and DMA completed.
                               // Alternative = invoke user callback rtn.
                               // Used by adc_Check_All_Complete() logic
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
#endif


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
//        CAUTION: at chip startup, FR5969 drives all GPIOs
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



#if defined(USES_PWM) || defined(USES_DRV8711) || defined(USES_DRV8848) || defined(USES_DRV8301)

//*****************************************************************************
//*****************************************************************************
//                               PWM   Routines
//*****************************************************************************
//*****************************************************************************

               //--------------------------------------------------------------
               //          Supported Modules
               //
               //   Module 1  TA1      CCR 1,2
               //   Module 2  TB0      CCR 1,2,3,4,5
               //
               // We do _NOT_ support TA0, because it is needed for "Systick"
               // support, and TI is mandating TA1.x for standard Launchpad
               // "PWM Out" on connector J2-2.
               // So for MSP430-FR5969 support, we have to use TA0 for "SysTick"
               //
               // None of them support built-in complementary operation.
               // None of them support built-in Deadtime, but there are
               // ways to hack it (CCR1/CCR4 for deadtime, CCR2/CCR3 for PWM)
               //--------------------------------------------------------------
#define  PWM_NUM_MODULES       2   /* are 2 PWM capable modukess on MSP432 LP */
#define  PWM_MAX_CHANNELS      5   /* are max 5 channels per PWM module       */

#define  PWM_DOWN_COUNT_INIT   1   /* Module setup for DOWN count mode */
#define  PWM_UP_COUNT_INIT     2   /* Module setup for UP   count mode */
#define  PWM_CENTER_COUNT_INIT 3   /* Module setup for UP/DOWN count mode */

      uint32_t           uwPrescalerValue = 0;

      char               _g_pwm_module_status [PWM_NUM_MODULES+1]  = { 0,0,0,0,0 };
      unsigned char      _g_pwm_channel_config [PWM_NUM_MODULES+1] = { 0,0,0,0,0 };

const unsigned char      _g_pwm_chan_mask [9]
                                   = { 0,       // unused
                                       0x01, 0x02, 0x04, 0x08,  // Channels  1 - 4
                                       0x10, 0x00, 0x00, 0x00   // Channels  5 - n
                                     };

typedef struct pwm_channel_def         /* PWM Channel definitions */
    {
        uint32_t  pwm_gpio_port_id;   /* Associated GPIO port         */
        uint32_t  pwm_gpio_pin_bit;   /* Associated GPIO pin          */
        uint32_t  pwm_ccr_num;        /* Associated PWM/Timer CCR num */
        uint8_t   pwm_even;           /* port is even = must shift bits by 8 */
    } PWM_CHANNEL_BLK;

                           // PWM_MODULE_1  -  TA1
const PWM_CHANNEL_BLK  _g_pwm_mod_1_channels [] =  //          Energia   LP   Grove
        {  {    0,      0,     0,          },  // no CCR0 entry
           { GP_PORT_1, BIT2, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0 },  // P 1.2  TA1.1   35   J2-2
           { GP_PORT_1, BIT3, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0 },  // P 1.3  TA1.2   36   J2-10
        };
                          // PWM_MODULE_2  -  TB0
const PWM_CHANNEL_BLK  _g_pwm_mod_2_channels [] =  //          Energia   LP   Grove
        {  {    0,      0,     0,          },  // no CCR0 entry
           { GP_PORT_1, BIT4, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0 },  // P 1.4  TB0.1   39   J2-9
           { GP_PORT_1, BIT5, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0 },  // P 1.5  TB0.2   40   J2-8
           { GP_PORT_3, BIT4, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0 },  // P 3.4  TB0.3   37   J1-8
           { GP_PORT_3, BIT5, TIMER_A_CAPTURECOMPARE_REGISTER_4, 0 },  // P 3.5  TB0.4   38   J1-9
           { GP_PORT_3, BIT6, TIMER_A_CAPTURECOMPARE_REGISTER_5, 0 },  // P 3.6  TB0.4   38   J1-10
        };


//*****************************************************************************
//  board_pwm_init
//                  was  timer_pwm_init
//
//         Initialize PWM module
//
//         flags parm is for future use.
//*****************************************************************************

int  board_pwm_init (int modgen_id, int count_mode, long period_val, int flags)
{
    uint32_t          prescalar;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (count_mode == TIMER_PERIODIC_COUNT_UP || count_mode == TIMER_PERIODIC_COUNT_UPDOWN)
       ;                                           // mode is valid
      else return (ERR_PWM_COUNT_MODE_INVALID);    // count_mode is bad - bail

// ??? revisit the following
        /* Compute the prescaler value to have TIMx counter clock equal to 18 MHz */
//  prescalar = ((SystemCoreClock /2) / 18000000) - 1;

      //--------------------------------------------------------
      // Configure the associated TAx / TBx  module
      //--------------------------------------------------------
    switch (modgen_id)
      { case 1:                                 // TA1  module
             TA1CCR0 = period_val;
             _g_pwm_module_status [modgen_id] = count_mode;  // save count mode
             break;
        case 2:                                 // TB0  module
             TB0CCR0 = period_val;
             _g_pwm_module_status [modgen_id] = count_mode;  // save count mode
             break;
      }

    return (0);                   // denote success
}


//*****************************************************************************
//  board_pwm_config_channel
//
//         Configure a specific channel (1-4) on a specific PWM module (TIM1/2/..)
//
//         For TIM1, allow complmentary mode usage.
//*****************************************************************************

int  board_pwm_config_channel (int modgen_id, int chan_num,
                               long initial_duty, int flags)
{
    PWM_CHANNEL_BLK     *pwmcblk;
    uint32_t            gpio_base_addr;
    uint32_t            scaleduty;
    uint16_t            bit_mask;
    unsigned char       chan_mask;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);


       //-------------------------------------------------------------
       // Setup associated GPIO(s) into Timer/PWM mode
       //
       // Convert PWM channel number to an index into table of 12 entries
       // that contains pin # and GPIO base index, then use  _g_gpio_base[]
       //----------------------------------------------------
    if (modgen_id == PWM_MODULE_1)
       pwmcblk = (PWM_CHANNEL_BLK*) &_g_pwm_mod_1_channels [chan_num];
       else pwmcblk = (PWM_CHANNEL_BLK*) &_g_pwm_mod_2_channels [chan_num];  // PWM_MODULE_2
    gpio_base_addr = _g_gpio_port_base_address[pwmcblk->pwm_gpio_port_id];
          // set associated SEL bits to denote this is being used
          // as a PWM channel
    bit_mask = pwmcblk->pwm_gpio_pin_bit;
    if (pwmcblk->pwm_even)
       bit_mask <<= 8;                        // must shift by 8 if port is even
    HWREG16(gpio_base_addr + OFS_PADIR)  |= bit_mask;    // set for Output
    HWREG16(gpio_base_addr + OFS_PASEL0) |= bit_mask;    // Set for PRIMARY
    HWREG16(gpio_base_addr + OFS_PASEL1) &= ~(bit_mask); //    Periph Function

//  P1DIR  |= BIT4 | BIT5;                    // P1.4 and P1.5 output
//  P1SEL0 |= BIT4 | BIT5;                    // P1.4 and P1.5 options select


// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

    chan_mask = _g_pwm_chan_mask [chan_num];  // get associated channel mask

      //-------------------------------------------------------------------
      // Configure the desired CCR channel for PWM in the TAx / TBx module
      //-------------------------------------------------------------------
    switch (modgen_id)
      { case 1:
             switch (chan_num)
               { case 1:                                 // TA1  module
                       TA1CCTL1 = OUTMOD_7;     // config CCR1 PWM as reset/set
                       TA1CCR1  = initial_duty;
                       break;
                 case 2:
                       TA1CCTL2 = OUTMOD_7;     // config CCR2 PWM as reset/set
                       TA1CCR2  = initial_duty;
                       break;
                 default:
                       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
               }
             break;

        case 2:                                 // TB0  module
             switch (chan_num)
               { case 1:
                       TB0CCTL1 = OUTMOD_7;     // config CCR1 PWM as reset/set
                       TB0CCR1  = initial_duty;
                       break;
                 case 2:
                       TB0CCTL2 = OUTMOD_7;     // config CCR2 PWM as reset/set
                       TB0CCR2  = initial_duty;
                       break;
                 case 3:
                       TB0CCTL3 = OUTMOD_7;     // config CCR3 PWM as reset/set
                       TB0CCR3  = initial_duty;
                       break;
                 case 4:
                       TB0CCTL4 = OUTMOD_7;     // config CCR4 PWM as reset/set
                       TB0CCR4  = initial_duty;
                       break;
                 case 5:
                       TB0CCTL5 = OUTMOD_7;     // config CCR5 PWM as reset/set
                       TB0CCR5  = initial_duty;
                       break;
               }
             break;
      }

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
    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

      //--------------------------------------------------------
      //Disable the associated TAx / TBx  module
      //--------------------------------------------------------
    switch (modgen_id)
      { case 1:                                 // TA1  module
             TA1CTL &= ~(MC_3);      // Disable PWM counting
             break;
        case 2:                                 // TB0  module
             TB0CTL &= ~(MC_3);      // Disable PWM counting
             break;
      }

    return (0);                      // denote completed successfully
}


//*****************************************************************************
// board_pwm_enable
//
//            Enable and Start the timer in Speed or PWM mode.
//*****************************************************************************
int  board_pwm_enable (int modgen_id, int flags)
{
    int  count_mode;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (_g_pwm_module_status [modgen_id] == TIMER_PERIODIC_COUNT_UP)
       count_mode = MC_1;                             // up mode
       else count_mode = MC_3;                        // up/down mode

      //--------------------------------------------------------
      // Enable the associated TAx / TBx  module
      //--------------------------------------------------------
    switch (modgen_id)
      { case 1:                                 // TA1  module
             TA1CTL = TASSEL__SMCLK | count_mode | TACLR; // Enable PWM: SMCLK, clear TAR
             break;
        case 2:                                 // TB0  module
             TB0CTL = TASSEL__SMCLK | count_mode | TACLR; // Enable PWM: SMCLK, clear TAR
             break;
      }

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_period
//
//         Set a new PWM Period.
//*****************************************************************************

int  board_pwm_set_period (int modgen_id, long period_val)
{
    uint32_t            scaleperiod;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

// ??? probably have to re-divide period based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;  // get Master PWM Clock value
//  scaleperiod = (_g_PWM_IO_clock / period_val) - 1;    // set new scaled period

      //--------------------------------------------------------
      // set new PWM period into PWM module TAx / TBx
      //--------------------------------------------------------
    switch (modgen_id)
      { case 1:                                 // TA1  module
             TA1CCR0 = period_val;
             break;
        case 2:                                 // TB0  module
             TB0CCR0 = period_val;
             break;
      }

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
    uint32_t            scaleduty;

    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

      //--------------------------------------------------------
      // Update the desired CCR channel in the TAx / TBx module
      //--------------------------------------------------------
    switch (modgen_id)
      { case 1:
             switch (chan_num)
               { case 1:                                 // TA1  module
                       TA1CCR1  = duty_cycle;
                       break;
                 case 2:
                       TA1CCR2  = duty_cycle;
                       break;
                 default:
                       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
               }
             break;

        case 2:
             switch (chan_num)
               { case 1:                                 // TB0  module
                       TB0CCR1  = duty_cycle;
                       break;
                 case 2:
                       TB0CCR2  = duty_cycle;
                       break;
                 case 3:
                       TB0CCR3  = duty_cycle;
                       break;
                 case 4:
                       TB0CCR4  = duty_cycle;
                       break;
                 case 5:
                       TB0CCR5  = duty_cycle;
                       break;
               }
             break;
      }

    return (0);                  // denote completed successfully
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
// in future - could consider a Hack version, using CCR1/CCR4, but would require
//            specific restrictions on user of what channels allowed (CCR2/CCR3)

    return (ERR_PWM_MODULE_NO_DEADTIME);      // Only TDx supports Deadtime
}


//*****************************************************************************
//  board_pwm_set_phase
//
//         Set the PWM's phase between complementary PWM
//         arrangements.
//*****************************************************************************

int  board_pwm_set_phase (int modgen_id, int chan_num, long phase_offset)
{
   return (ERR_PWM_MODULE_NO_PHASE);  // MSP432 PWMs do NOT support
                                      // phase shifting. Only C2000 modules do
}


//*****************************************************************************
//  board_pwm_set_channel_output
//
//         Set polarity of channel output (start high or low)
//*****************************************************************************
int  board_pwm_set_channel_output (int modgen_id, int channel_num,
                                   int output_mode, int flags)
{
    if (modgen_id < 1  ||  modgen_id > PWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (channel_num < 1  ||  channel_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);


      // ??? is this supported on MSP432 - perhaps tweak TAxCCTLx = OUTMOD_
      //     to do set/reset instead of reset/set ?


//  return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    return (0);                // denote completed successfully
}

#endif                         // defined(USES_PWM)



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
//          Setup CPU clocks.                          Max is 16 MHz on FR5969
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
                    // Set ACLK = VLOCLK, Set SMCLK and MCLK = DCO,
         CSCTL2   = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
         CSCTL3   = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1
         CSCTL0_H = 0;                             // Lock CS registers

#if (HAL_API)
          CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_4);  // Set DCO freq to 16 MHz
             // Set MCLK = DCO with frequency divider of 1
          CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
             // Set SMCLK = DCO with frequency divider of 1
          CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
#endif
       }
      else if (mcu_clock_hz >= 8000000)
              {     // Setup for 8 MHz DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 8MHz   CPU clock  =  ORIGINAL WORKING LOGIC
                    //---------------------------------
                CSCTL0_H = 0xA5;                          // Unlock CS registers
                CSCTL0_L = 0;
                CSCTL1   = DCORSEL + DCOFSEL0 + DCOFSEL1; // Set DCO to 8 MHz
                CSCTL2   = SELA_1  + SELS_3 + SELM_3;     // set ACLK - VLO, the 
                                                          // rest: MCLK/SMCLK = DCO
                CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers to 0

// And see msp430fr59xx_cs_04.c as to how to set this up using XT1 clock as ACLK ref
// ALT          CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
// ALT          CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
// ALT          CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK and MCLK = DCO
                                                                      // set ACLK = VLOCLK
// ALT          CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
// ALT          CSCTL0_H = 0;                             // Lock CS registers
              }
      else    {     // Setup for 1 MHz (startup default) DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 1 MHz   CPU clock
                    //---------------------------------
                CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_1);  // Set DCO freq to 1 MHz  <== IS GUESS _ TWEAK THIS !!!
                    // Set MCLK = DCO with frequency divider of 1
                CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
                    // Set SMCLK = DCO with frequency divider of 1
                CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
              }


#if NEW_API_CLOCK_LOGIC
       // Configure one FRAM wait state as required by the device datasheet for
       // MCLK operation beyond 8 MHz _before_ configuring the clock system.
/// FRCTL0 = FRCTLPW | NWAITS_1;

       // Set DCO frequency to 16 MHz
/// CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_4);  // Set DCO frequency to 16 MHz
    CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_6);  // Set DCO frequency to 8 MHz

       // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set ACLK=LFXT
    CS_initClockSignal (CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource (32768, 0);

        // Start XT1 with no time out
//  CS_turnOnLFXT (CS_LFXT_DRIVE_0);    HANGING ==> Needs Crystal Mounted - WVD
#endif                    // NEW_CLOCK_LOGIC


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
// board_systick_timer_config
//
//          Emulate a "SYSTICK" style Interval Timer.
//
//          This is required by MQTTCC3100.c support, as well as other
//          packages that need a milli-second iterval timer.
//
//          The FR5969 has two 16-bit Timer_A3 style timers (TA0/TA1),
//                         two 16-bit Timer_A2 style timers (TA2/TA3),
//                         one 16-bit Timer_B7 stype timer  (TB0).
//
//          We use Timer TA3 since it is free and not pinned out to any
//           external pins, and leaves TA0 available for use by an RTOS.
//*****************************************************************************
void  board_systick_timer_config (void)
{
    TA3CCTL0 = CCIE;                     // enable CCR0 interrupt
    TA3CCR0  = _g_TA1_1ms_ticks;         // number of SMCLKS that = 1 milli-sec
    TA3CTL   = TASSEL_2 + MC_1 + TACLR;  // use SMCLK, up mode, clear TAR
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
//                            TA3   CCR0   ISR
//
// SysTick interrupt handler.
//
//                               Increment Systick 1 ms count, on every 1ms pop
//*****************************************************************************
#pragma vector=TIMER3_A0_VECTOR
__interrupt void  TIMER_A3_A0CCR0_ISR (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)

#if defined(USES_MQTT)
    SysTickIntHandler();     // invoke MQTTCC3100.c to update its Timer
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
//  board_uart_init
//
//               FR5969 Launchpad default Console UART port is USCI_A0
//
//               The code is hardwired for 115,200 baud rate.
//               Use the lookup tables in the F5529's TECH REF to select the
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

    unsigned short  baud_UCBR0,   baud_CTLW0;

        //----------------------------------------------------------------------
        // Generate Baud Rate pre-scalars, based on fixed clock rates we support
        //----------------------------------------------------------------------
     if (_g_SysClk_Ticks >= 16000000)
        {             // assume MCLK is set to 16.000 MHz
               baud_UCBR0 = 8;       // 16000000/115200 = 138.88  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0xF7, UCBRFx=10
               baud_CTLW0 = 0xF700 + UCBRF_10 + UCOS16;
        }
     else if (_g_SysClk_Ticks >= 8000000)
             {        // assume MCLK is set to 8.000 MHz
               baud_UCBR0 = 4;       // 8000000/115200 = 69.44  but
                                     // turning on UCOS16 changes factors
                      // TECH REF UART Table: Modulation UCBRSx=0x55, UCBRFx=5
               baud_CTLW0 = 0x5500 + UCBRF_5 + UCOS16;
             }
     else    {        // assume MCLK is 1.000 MHz - the default startup clock
               baud_UCBR0 = 8;       // 1000000/115200 = 8.48
               baud_CTLW0 = 0xD600;  // Table: Modulation UCBRSx=0xD6, UCBRFx=0
             }

    PM5CTL0 &= ~LOCKLPM5;        // FRAM Enable GPIOs: disable the power-on
                                 // default setting GPIOs to high-impedance mode

        //-------------------------------------------------
        //  setup GPIO pins P2.0 and P2.1 to use for UART
        //-------------------------------------------------
        // Set GPIOs P2.0 (TX) and P2.1 (RX) into UART mode (routed to USB CDC)
//    GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P2,
//                                                GPIO_PIN0 | GPIO_PIN1,
//                                                GPIO_PRIMARY_MODULE_FUNCTION);

    P2SEL1 |=  (BIT0 + BIT1);   // set P2.0/P2.1 for use as USCI_A0 UART TXD/RXD
    P2SEL0 &=  ~(BIT0 + BIT1);

        //-------------------------------------------------
        //            Configure UART
        //-------------------------------------------------
//  EUSCI_A_UART_init (EUSCI_A0_BASE, &uartConfig);

        // Startup the UART module
//  EUSCI_A_UART_enable (EUSCI_A0_BASE);

    UCA0CTLW0 = UCSSEL__SMCLK + UCSWRST;  // Use SMCLK, 8N1, and turn on RESET

        //-------------------------------------------------
        //            setup  Baud  Rate
        //-------------------------------------------------
    UCA0BR0   = baud_UCBR0;               // set basic baud rate pre-scalar
    UCA0BR1   = 0;
    UCA0MCTLW = baud_CTLW0;               // set baud Modulation and UCOS values

    UCA0CTLW0 &= ~UCSWRST;                // Enable and Initialize USCI UART
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

#if defined(USES_CONSOLE_READ) || defined(USES_CONSOLE_WRITE)
    char  in_char;
    int   rc;

           // read in any character that the user typed in
    rc = UCA0IFG & UCRXIFG;
    if ( ! rc)
       return (0);                                 // no UART data is present

//  in_char = EUSCI_A_UART_receiveData (EUSCI_A0_BASE);  // read in char from UART
    in_char  = UCA0RXBUF;           // read in char from uART

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
//  rc = UART_getInterruptStatus (EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    rc = UCA0IFG & UCRXIFG;
    if ( ! rc)
       continue;                            // user needs to type in more chars

//  in_char = EUSCI_A_UART_receiveData (EUSCI_A0_BASE);   // read in char from uART
    in_char  = UCA0RXBUF;                   // read in char from uART

    if (in_char <= 0)
       return (0);        // no character or error occurred. bail out

       // note: board_uart_read MUST ALSO echo back the char to user
    if (in_char != '\n')
       board_uart_write_char (in_char);      // echo the char (\n is special)
/// EUSCI_A_UART_transmitData (EUSCI_A0_BASE, in_char);   // echo the char
//  while ( ! (UCA0IFG & UCTXIFG))   ;       // wait till TX buffer is empty
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
//  EUSCI_A_UART_transmitData (EUSCI_A0_BASE, outchar);   // send a char
    while ( ! (UCA0IFG & UCTXIFG))   ;       // wait till TX buffer is empty
       UCA0TXBUF  = outchar;                 // send a char
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
//      EUSCI_A_UART_transmitData (EUSCI_A0_BASE, *outstr);   // send a char
        while ( ! (UCA0IFG & UCTXIFG))   ;       // wait till TX buffer is empty
           UCA0TXBUF  = *outstr;                 // send a char

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

void  board_vtimer_check_expiration (void)
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
//         On FR5969, STEP1_AIN pin is connected to TB0.4 on pin P3.5
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
    P4OUT &= ~BIT3;              // set CC3100 nHIB to DISABLED
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
    P4OUT |= BIT3;               // set CC3100 nHIB to ENABLED
}


//-------------------------------------------------------------------
// CC3100_InterruptEnable
//
//            Enables the interrupt (IRQ) from the CC3100 on P 1.2
//--------------------------------------------------------------------
void  CC3100_InterruptEnable()
{
    P1DIR &= ~BIT2;      // ensure is in INPUT mode

    P1IES &= ~BIT2;      // Turn on interrupt enable masks
    P1IE  |= BIT2;
}


//-------------------------------------------------------------------
// CC3100_InterruptDisable
//
//            Disables the interrupt (IRQ) from the CC3100 on P 1.2
//--------------------------------------------------------------------
void  CC3100_InterruptDisable()
{
    P1DIR &= ~BIT2;      // ensure is in INPUT mode

    P1IE  &= ~BIT2;      // Turn off interrupt enable masks
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
//                      CC3100   IRQ   ISR   Handler           P1.2
//-------------------------------------------------------------------

#pragma vector=PORT1_VECTOR
__interrupt void  Port1_ISR (void)
{
    volatile int   other_bits;

    switch (__even_in_range(P1IV, P1IV_P1IFG7))
      {
                  // Vector  P1IV_P1IFG2:  P1IV P1IFG.2
        case  P1IV_P1IFG2:
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
//      TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, \
//
#pragma vector=PORT2_VECTOR, ADC10_VECTOR, USCIAB0TX_VECTOR, \
        COMPARATORA_VECTOR, NMI_VECTOR
__interrupt void  Trap_ISR (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}

