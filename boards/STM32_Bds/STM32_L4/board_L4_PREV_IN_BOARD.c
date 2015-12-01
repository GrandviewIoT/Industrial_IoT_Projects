
// 07/03/15 - ADC TRIGGERing - associated TIMx CCR2's MMS field needs to be set
//            to 0x2 (Update) or 0x4 (CCR1, 0x5 (CCR2), 0x6 (CCR3), 0x7 (CCR4)
//            Right now, are only doing it for TIM6/7, but needs to be done
//            ofr ANY timer that needs to drive ADC (TIM1/TIM2/...)

/********1*********2*********3*********4*********5*********6*********7**********
*
*                       board_L4.c - STM32 L4        --> Direct clone of F4 except clocks is new
*
*
* STM32Cube / HAL based version of board_xxx() functions
*
*
* Supported/Tested Boards
* -----------------------
*      STM32_L4_76_VG    Discovery   1024 K Flash   128K RAM     80 MHz
*                                        1 ADC @ 16 channels
*                                        3 SPI       3 I2C       1 USB
*
*
*  History:
*    07/20/14 - Created as prep for SJO ESC 2015 class.  Duq
********************************************************************************
*   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
*  Grandview DB/DC Systems             DRDA Portable Toolkit
*    Licensed Property of Grandview DC/DC Systems
*    (c) Copyright  Grandview DB/DC Systems  1994, 2002
*    All rights reserved
*
* CHANGE Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "boarddef.h"
#include "user_api.h"
#include <math.h>

#include "stm32l4xx_hal.h"              // HAL standard includes
#include "stm32l476xx.h"                // MCU specific defs
#include "user_api.h"
#include "boarddef.h"



#if defined(USES_CC3000) || defined(CC3000_USE_BOOSTERPACK1)
long hci_unsolicited_event_handler(void);
#define  FALSE   0
#define  TRUE    1
#endif

// move the following to boarddef.h at some point
void board_system_clock_config (long mcu_clock_hz);     // internal Function protos
int  board_dac_check_conversions_done (int dac_module_id, int sequencer);
int  board_dac_clear_conversions_done (int dac_module_id, int sequencer);
int  board_dac_SineWave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_Trianglewave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_SquareWave  (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int  board_timerpwm_enable_clock (int module_id);
long board_timerpwm_compute_prescalar (long period_val);
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id);
void board_error_handler (void);                    // subroutine prototypes

// internal Function protos
uint32_t HAL_GetTick (void);   // overrides for HAL Systick functions
void     HAL_IncTick (void);

void  DMA1_Channel1_IRQHandler (void);              // ADC DMA ISR - all channels
void  DMA2_Stream0_IRQHandler (void);               // DAC DMA ISR - all channels

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------
    P_EVENT_HANDLER      pIrqEventHandler = 0;
    int8_t               IntIsMasked;

    uint32_t   _g_SysClk_Ticks = MCU_CLOCK_SPEED; // Global to hold clock frequency (ticks/sec)

    uint32_t   _g_systick_millisecs = 0; // Global to how many 1 ms ticks have
                                         // accumulated startup (poor mans TOD)

    char       _g_vtimers_active    = 0; // Optional VTIMER support


    char       _g_pwm_mclock_configured = 0; // PWM master clock was configured
    uint32_t   _g_pwm_mdivider = 1;          // PWM Master clock : CPU divider ratio

    long       Systick_Frequency  = 100;    // 10 ms ticks
    long       SysTick_Reload_Val = 0;      // actual TMR reload value

    uint32_t   ulTickCount = 0;             // SysTick count


    SPI_HandleTypeDef  _g_hspi1;           // SPI 1 support
    SPI_HandleTypeDef  _g_hspi2;           // SPI 2 support
    SPI_HandleTypeDef  _g_hspi3;           // SPI 3 support
    DMA_HandleTypeDef  _g_hdma_spi2_rx;
    DMA_HandleTypeDef  _g_hdma_spi2_tx;

    short              _g_hspi1_type = 0;
    short              _g_hspi2_type = 0;
    short              _g_hspi3_type = 0;

    short              _g_spi_mode = 1;


#if (USES_BLUENRG_BLE)
extern    SPI_HandleTypeDef  SpiHandle;  // NOTE: ST's hci routines depend upon
                                         //    this hardcoded  SpiHandle  label.
                                         //    They have direct externs to it.
#endif



    short              IRQ_interrupts_enabled = -1;

    long               irq_state;
    long               ulSpiIRQState;

    long               brc;

    unsigned char      Hci_Link_Start []   = { 0x01,0x00,0x05,0x00,0x00,
                                               0x01,0x00,0x40,0x01,0x00  };
    unsigned char      Hci_Link_Reply [10] = { 0,0,0,0,0,0,0,0,0,0 };

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    char   _g_uart_buf_idx   = 0;        // keeps track of user input via UART
    char   _g_uart_last_char = 0;
    UART_HandleTypeDef   _g_UartHandle;
#endif

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    unsigned long   _g_crcid_Result;
#endif

               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
const  GPIO_TypeDef * _g_gpio_base[] =
    {
        0L,
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
        GPIOE
    };
#define  GP_PORT_A   1        // indexes into above table, based on port id
#define  GP_PORT_B   2
#define  GP_PORT_C   3
#define  GP_PORT_D   4
#define  GP_PORT_E   5

//******************************************************************************
//******************************************************************************
//                    INLINE CALLs FAUIL on L4    !!! ???
//         LOW LEVEL MAPPING OF GPIO PIN calls on STM32  F0_72, F1_03, and F4_01
//******************************************************************************
//******************************************************************************

//extern  const  GPIO_TypeDef  * _g_gpio_base[];

// Turn ON one or more pins on a given a port
void   inline_pins_High (int gpio_num,  unsigned long pins)
{
    GPIO_TypeDef  *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_num];
    gpio_port->BSRR = pins;
}

// Turn OFF one or more pins on a given a port
void   inline_pins_Low (int gpio_num,  unsigned long pins)
{
    GPIO_TypeDef  *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_num];
    gpio_port->BSRR = ((uint32_t) pins << 16);
}

// Toggle one or more pins on a given a port
void   inline_pins_Toggle (int gpio_num,  unsigned long pins)
{
    GPIO_TypeDef  *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_num];
    gpio_port->ODR ^= pins;
}

// Read in one or more pins from a port, and return their (hex) bit settings
int   inline_pins_Read (int gpio_num,  unsigned long pins_mask)
{
    GPIO_TypeDef  *gpio_port;
    int           pins_input;

    gpio_port  = (GPIO_TypeDef*) _g_gpio_base [gpio_num];
    pins_input = (gpio_port->IDR & pins_mask);

    return (pins_input);
}

// Read in a specific pin, and return if it is on (1) or off (0)
int   inline_pin_Value (int gpio_num,  unsigned long pin_mask)
{
    GPIO_TypeDef  *gpio_port;
    int           pins_input;

    gpio_port  = (GPIO_TypeDef*) _g_gpio_base [gpio_num];
    if (gpio_port->IDR & pin_mask)
       return (1);                          // pin is ON
    return (0);                             // pin is OFF
}

// Configure one or more pins on a given a port for OUTPUT
void   inline_set_pins_Output (uint32_t gpio_port,  unsigned long pins)
{
    uint32_t  temp;

// 06/02/15 ??? !!! Can this do multiple pins or 1 at a time ?  This came from GPIO.C init() logic
//                  As is, this will not work, because need to walk thru each pin in the mask
//                  and generate it's position in the MODER regsiter
// for (position = 0;  position < 32;  position++)
//   { temp = gpio_port->MODER;
//     temp &= ~(GPIO_MODER_MODER0 << (position * 2));
//     temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
//     gpio_port->MODER = temp;
//  }
}

// Configure one or more pins on a given a port for INPUT
void   inline_set_pins_Input (uint32_t gpio_port,  unsigned long pins)
{
// see above discussion - needs work ??? !!!
}

//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   Routines
//
//**************************************************************************
//**************************************************************************

//******************************************************************************
//  board_init
//
//            Initializes board clocks and basic GPIOs.
//******************************************************************************

void  board_init (long mcu_clock_rate)
{
       //-----------------------------------------------------------------------
       // Reset all peripherals, Initialize Flash interface and Systick.
       // This calls the generic HAL_Init() in stm32f3xx_hal.c
       // which in turn does a callback to our specific peripherals GPIO init
       // for SPI, DMA, ... by invoking HAL_MspInit() in our stm32f3xx_hal_msp.c
       //-----------------------------------------------------------------------
    board_stop_WDT();                 // ensure watchdog timer is off/disabled

    HAL_Init();                       // Invoke ST's HAL startup logic

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED);  // use default MCU speed

#if defined(USES_SYSTICK) || defined(USES_MQTT) || defined(USES_ADC) || defined(USES_VTIMER)
    board_systick_timer_config();     // ensure Systick timer is turned on
#endif

       //------------------------------------------------------------------
       //   Configure basic GPIO clocks and any GPIOs needed
       //------------------------------------------------------------------
    board_gpio_init();

        // Enable the UART if user wants to invoke CONSOLE or DEBUG_LOG calls
#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    board_uart_init();          // go ahead and enable the default UART
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

//  total_ticks = (long) (g_fusec_to_ticks * (float) usec_delay);
    total_ticks = (long) (8 * (float) usec_delay);       // TEMP HACK 03/11/15

    while (total_ticks > 0)
       total_ticks -= 3;                  // each iteration is approx 3 instr
}


//*****************************************************************************
//  board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************

void  board_delay_ms (long num_millis)    // Delay of 1 ms
{

    HAL_Delay ((uint32_t) num_millis);
}


//*****************************************************************************
//  Delay
//
//          Produce a delay in milli-seconds (ms)      Used by CC3xxx Simplelink
//*****************************************************************************

void  Delay (unsigned long delay_interval)
{
     board_delay_ms (delay_interval);
}


//*****************************************************************************
//  board_disable_global_interrupts
//
//
//         Turn off Global interrupts  (for CC3000, Timer config, ...)
//
//         ASM ("cpsid i");   // CAUTION: do NOT do any SVC calls or will crash
//*****************************************************************************

void  board_disable_global_interrupts (void)
{
        //------------------------------
        // Disable processor interrupts.
        //------------------------------
     __disable_irq();              // GNU and IAR synonym
//   ___disableinterrupt();        // native IAR
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for CC3000, Interval Timer,
//                                     PWM Timer, ADC, config ...)
//         ASM ("cpsie i");
//*****************************************************************************

void  board_enable_global_interrupts (void)
{
        //------------------------------
        // Enable processor interrupts.
        //------------------------------
     __enable_irq();              // GNU and IAR synonym
//   ___enableinterrupt();        // native IAR
}


//*****************************************************************************
//  board_error_handler
//
//          Catastrophic error occurred. Stop the train.
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



//#if defined(USES_ADC)

//*****************************************************************************
//*****************************************************************************
//                               ADC   Routines
//*****************************************************************************
//*****************************************************************************

    int              _g_ADC_complete    = 0;  // 1 = all ADC conversions/DMAs are complete
    char             _g_adc_clocks_on   = 0;
    char             _g_active_channels = 0;
    int              _g_sequencer       = 0;
    int              _g_step_num        = 1;  // first sequencer step always = 1

    int              _g_trigger_atmrpwm      = 0; // index to correct Timer/PWM
    uint16_t         _g_trigger_auser_api_id = 0; // User API id for the trigger
    uint16_t         _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
    uint32_t         _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

    unsigned char    _g_adc_step_map [16] = { 0 };  // indexed by channel #

    unsigned short   _g_adc_conv_results[16];    // Internal buf to hold ADC DMAed results

ADC_CB_EVENT_HANDLER _g_adc_callback       = 0L;
    void             *_g_adc_callback_parm = 0;

    int              _g_DMA_complete  = 0;       // 1 = all ADC conversion DMAs are complete
    char             _g_DMA_overrun   = 0;       // 1 = DMA completed a new set before the previous one was processed
    unsigned long    dma_callback_seen = 0;      // DEBUG COUNTERs
    unsigned long    dma_rupt_seen     = 0;

    ADC_HandleTypeDef       _g_AdcHandle;
    ADC_ChannelConfTypeDef  _g_ChanConfig;
    DMA_HandleTypeDef       _g_DmaHandle;

    void  ADC1_DMA_IRQHandler (void);              // Function Prototypes

typedef struct adc_module_def          /* ADC Module definitions */
    {
        ADC_TypeDef  *adc_base;        /* ADC base address */
    } ADC_MODULE_BLK;

typedef struct adc_channel_def         /* ADC Channel definitions */
    {
        GPIO_TypeDef *chan_gpio_port;  /* Associated GPIO port           */
        uint32_t     chan_gpio_pin;    /* Associated GPIO pin            */
        uint32_t     chan_alt_func;    /* Channel Alternate Function Id  */
        uint32_t     chan_adc_id;      /* Logical Channel id for this ADC */
    } ADC_CHANNEL_BLK;


typedef struct adc_trigger_def         /* ADC Trigger definitions */
    {
        uint16_t     trigger_user_api_id; /* User API id for the trigger   */
        uint16_t     trigger_tmr_mmsmask; /* Associated Mask for TIM MMS   */
        uint32_t     trigger_adc_extmask; /* Associated Mask for ADC EXTSEL*/
    } ADC_TRIGGER_BLK;


const ADC_MODULE_BLK  _g_adc_modules [] =
        {  {    0L     },
           { ADC1 },
//         { ADC2 },
//         { ADC3 }     // ADC_MODULE_ANY defaults to ADC0     FROM TIVA
        };


// note: on STM32, user indexes start at 0, to handle channel 0 on STM's
const ADC_CHANNEL_BLK  _g_adc_channels [] =        // STM32  Arduino  User Index
        {  { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_0  },  // PA0    A0      0
           { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_1  },  // PA1    A1      1
           { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_2  },  // PA2            2
           { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_3  },  // PA3            3
           { GPIOA, GPIO_PIN_4, 0, ADC_CHANNEL_4  },  // PA4    A2      4
           { GPIOA, GPIO_PIN_5, 0, ADC_CHANNEL_5  },  // PA5            5
           { GPIOA, GPIO_PIN_6, 0, ADC_CHANNEL_6  },  // PA6            6
           { GPIOA, GPIO_PIN_7, 0, ADC_CHANNEL_7  },  // PA7            7
           { GPIOB, GPIO_PIN_0, 0, ADC_CHANNEL_8  },  // PB0    A3      8
           { GPIOB, GPIO_PIN_1, 0, ADC_CHANNEL_9  },  // PB1            9
           { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_10 },  // PC0    A5     10
           { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_11 },  // PC1    A4     11
           { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_12 },  // PC2           12
           { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_13 },  // PC3           13
           { GPIOC, GPIO_PIN_4, 0, ADC_CHANNEL_14 },  // PC4           14
           { GPIOC, GPIO_PIN_5, 0, ADC_CHANNEL_15 }   // PC5           15
        };

              //-----------------------------------------------
              // Triggerable Timers/Events available on F4 ADC
              //    - TIM1_TRGO      - TIM1_CC4
              //    - TIM2_TRGO      - TIM3_TRGO
              //    - TIM15_TRGO
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
// This was a direct lift/hack from F4 so needs to be tweaked for L4
          { ADC_TRIGGER_TIMER_1, TIM_TRGO_OC1REF,     ADC_CFGR_EXTSEL_0 /* HACK ? */ },  // TImer1 CC1
//        { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T1_CC2 },  // TImer1 CC2
//        { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T1_CC3 },  // TImer1 CC3

//        { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T2_CC2 },  // TImer2 CC2
//        { ADC_TRIGGER_TIMER_2_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T2_CC3 },  // TImer2 CC3
//        { ADC_TRIGGER_TIMER_2_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T2_CC4 },  // TImer2 CC4
          { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_CFGR_EXTSEL_1           },  // TImer2 Update/TRG0

//        { ADC_TRIGGER_TIMER_3_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T3_CC1 },  // TImer3 CC1
          { ADC_TRIGGER_TIMER_3,     TIM_TRGO_UPDATE, ADC_CFGR_EXTSEL_2           },  // TImer3 Update/TRG0

          { ADC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, ADC_CFGR_EXTSEL_3           },  // TImer4 CC4

//        { ADC_TRIGGER_TIMER_5,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T5_TRGO},  // TImer5 CC1
//        { ADC_TRIGGER_TIMER_5_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T5_CC2 },  // TImer5 CC2
//        { ADC_TRIGGER_TIMER_5_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T5_CC3 },  // TImer5 CC3

//  ???   { ADC_TRIGGER_TIMER_5_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T8_CC1 },  // TImer8 CC1
//  ???   { ADC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO},  // TImer8 Update/TRG0   ??? !!! TIM8 does _NOOT_ exist on F4_01 !!! ???
//        { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIGCONV_Ext_IT11 },// External GPIO
          {     -1,                        0,                     0               }   // end of table
        };



//*****************************************************************************
//  board_adc_init
//
//         Initialize an ADC module, and configure the overall sampling
//         clock used for the ADC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - PWM Module 0  Generator 0 / 1 / 2 / 3
//           - PWM Module 1  Generator 0 / 1 / 2 / 3
//           - Timer n
//           - Comparator n
//
// Note that on most platforms, the Triggering is _module_ wide, in other
//      words, all channels (ADC 1-16) will be triggered off the same timer/pwm.
//
// STM32-F4 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC_SQR1 -> ADC_SQR3.
// ADC results are stored in a single 32-bit register: ADC_DR
// Results are stored in the lower order 16-bits.
// We use DMA to process multiple ADC conversion results, to avoid
// "interrupt per channel" overhead and complexity.
//
// The STM32-F4 Nucleo 64-pin device physically supports up to 16 channels,
// of which 5 are wired out to the Nucleo/Arduino pins.
// Two internal sources (Temperature Sensor, Battery Monitor) are also available.
// The other ADCs are wired out to the "Morpho" connector.
// We allow support for all 16 channels.
//
// The STM32-F4 supports a single 12-bit ADC, with up to 16 Channels.
//*****************************************************************************

int  board_adc_init (int adc_module_id, uint32_t clock_rate,  int trigger_type,
                     int flags)
{
    ADC_TRIGGER_BLK  *trigblkp;
    int              rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (_g_adc_clocks_on == 0)
       {     // turn on the clocks for the ADC module  (Is only 1 on STM32 F4)
         __HAL_RCC_ADC_CLK_ENABLE();             // Enable ADC Periph clock
         __HAL_RCC_DMA1_CLK_ENABLE();            // Enable DMA2 clock
         _g_adc_clocks_on = 1;                   // denote clocks are now on

             //-----------------------------------------------------------------
             // This is the opportune time to do initial config of the ADC and
             // DMA modules. Note that we will later have to manuually override
             // the initial number of ADC channels (set SQR1 xxxx value).
             //
             // If calibration is required, call that out as a separate subrtn.
             // Because F4 only has 1 ADC module, we can hard-code it.
             //-----------------------------------------------------------------
         _g_AdcHandle.Instance                   = ADC1; // single module: ADC1
         _g_AdcHandle.Init.NbrOfConversion       = 1;    // - this gets modified -
         _g_AdcHandle.Init.DMAContinuousRequests = ENABLE;             // Use DMA
         _g_AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B; // 12 bit resol
         _g_AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

         if (trigger_type == ADC_TRIGGER_USER_APP)
            {     //-----------------------------------------------------------
                  // no external triggers - use SW trigger calls from User App
                  //-----------------------------------------------------------
              _g_AdcHandle.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
              _g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
            }
          else
            {      //-------------------------------------------------------------
                   // loop thru trigger table, find entry, and save needed flags.
                   //
                   // We do not turn on all the trigger related TIM flags
                   // until adc_enable() is requested.
                   // We also require the User App issue a timer_ADC_Trigger_Start()
                   // at some point, to cause the ADC to start sampling.
                   //-------------------------------------------------------------
              trigblkp = (ADC_TRIGGER_BLK*) &_g_adc_trigger_config_table[0];
              while (1)
                { if (trigblkp->trigger_user_api_id == -1)
                     return (ERR_ADC_UNSUPPORTED_TRIGGER_TYPE);  // end of table and no match
                  if (trigblkp->trigger_user_api_id == trigger_type)
                     {     // setup _g_AdcHandle.Init with proper Trigger flags
                       _g_AdcHandle.Init.ExternalTrigConv = trigblkp->trigger_adc_extmask;
                       if (flags & ADC_TRIGGER_FALLING)
                          _g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
                          else if (flags & ADC_TRIGGER_RISEFALL)
                                  _g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
                                  else _g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
                           // save rest of parms to tb used at adc_enable() and timerpwm_enable()
                       _g_trigger_adc_extmask  = trigblkp->trigger_adc_extmask; // ADC EXTSEL
                       _g_trigger_atmr_mmsmask = trigblkp->trigger_tmr_mmsmask; // TIM MMS
                       _g_trigger_auser_api_id = trigger_type;
                           // the User API trigger type is an encoded field
                           // where the low order 4 bits = CCR, and the rest = index
                           // number of associated Timer/PWM
                       if (trigblkp->trigger_tmr_mmsmask != 0)      // a GEXTI GPIO pin = 0
                          _g_trigger_atmrpwm = (trigger_type >> 4); // save Timer/PWM index
                       break;             // bail out of loop
                     }
                  trigblkp++;             // step to nexct entry in array
                }
            }

         _g_AdcHandle.Init.ScanConvMode          = ENABLE;  // scan multiple channels
         _g_AdcHandle.Init.ContinuousConvMode    = DISABLE; // only when trigger
         _g_AdcHandle.Init.DiscontinuousConvMode = DISABLE; // not used
         _g_AdcHandle.Init.NbrOfDiscConversion   = 0;
         _g_AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
         _g_AdcHandle.Init.EOCSelection          = DISABLE; // DMA will interrupt
                                                            //   not ADC EOC.
         rc = HAL_ADC_Init (&_g_AdcHandle);
         if (rc != HAL_OK)
            {
                 /* Initialization Error */
              board_error_handler();           // return error code instead  ???
            }

             //-----------------------------------------------------------------
             // Initialize the DMA associated with the ADC module.
             //   ?? Or should we hold off on this until we hit adc_enable() ?
             //-----------------------------------------------------------------
         __HAL_RCC_DMA1_CLK_ENABLE();                    // L4 uses DMA1
         _g_DmaHandle.Instance         = DMA1_Channel1;
         _g_DmaHandle.Init.Request     = DMA_REQUEST_0;
///      _g_DmaHandle.Init.Channel     = DMA_CHANNEL_0;  // not used on L4
         _g_DmaHandle.Init.Direction   = DMA_PERIPH_TO_MEMORY;  // ADC -> RAM

         _g_DmaHandle.Init.PeriphInc   = DMA_PINC_DISABLE;
         _g_DmaHandle.Init.MemInc      = DMA_MINC_ENABLE;       // step buf ptr
         _g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         _g_DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
         _g_DmaHandle.Init.Mode        = DMA_CIRCULAR;     // Treat as circular buf
         _g_DmaHandle.Init.Priority    = DMA_PRIORITY_HIGH;
//       _g_DmaHandle.Init.FIFOMode    = DMA_FIFOMODE_DISABLE;   ??? NO FIFO on L4 ???
//       _g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//       _g_DmaHandle.Init.MemBurst    = DMA_MBURST_SINGLE;
//       _g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

         rc = HAL_DMA_Init (&_g_DmaHandle);

            // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&_g_AdcHandle, DMA_Handle, _g_DmaHandle);

            //----------------------------------------------------------
            //          Configure the NVIC for DMA interrupts
            // Configure NVIC for DMA transfer complete interrupt.
            //----------------------------------------------------------
         HAL_NVIC_SetPriority (DMA1_Channel1_IRQn, 0, 0);
         HAL_NVIC_EnableIRQ (DMA1_Channel1_IRQn);
       }

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate will be determined by the trigger source.
//
//         adc_module parm is really just an index number (0-3)
//         that denote which ADC module to use.
//         For the STM32 F4, there is only 1 ADC module: ADC1
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//
//         STM32 F4 supports a max of 16 ADC channels and 1 ADC module.
//         It has only 1 main sequencer.
//*****************************************************************************

int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer,  int step_num,
                               int last,  int flags)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    ADC_CHANNEL_BLK  *adcblkp;
    int               orig_step_num;
    int               rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (channel_num < 0  ||  channel_num > 15)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

          //-----------------------------------------------------
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of 16 entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[]
          //----------------------------------------------------
    adcblkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num];
         // first configure it as a straight GPIO Input, to clear everything out
    GPIO_InitStruct.Pin   = adcblkp->chan_gpio_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (adcblkp->chan_gpio_port, &GPIO_InitStruct);
         // then configure it over to an Analog Input
    GPIO_InitStruct.Pin  = adcblkp->chan_gpio_pin;
 ///GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (adcblkp->chan_gpio_port, &GPIO_InitStruct);

          //----------------------------------------------------------------
          // setup the step within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save the step number passed on entry
    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

       //-----------------------------------------------------------------
       // Configure the associated ADC channel     (as a regular channel)
       //
       // Note: Based on the intterupts (IT) that occur after each number
       //       "uhADCxConvertedValue"  ADC conversions (IT by DMA end
       //       of transfer), select sampling time and ADC clock with sufficient
       //       duration as to not create an overhead situation in IRQHandler.
       //
       // Note that "Rank" is really sequence id - the sequence in which
       // these will fire, starting at one, and going up.
       //-----------------------------------------------------------------
    _g_ChanConfig.Channel      = adcblkp->chan_adc_id;    // set ADC channel #
////_g_ChanConfig.Rank         = ADC_REGULAR_RANK_1;      // OFFICIAL L4 - Rank of sampled channel number ADCx_CHANNEL
    _g_ChanConfig.Rank         = step_num;            // set step # in sequencer
    _g_ChanConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;   // <--- MAKE LARGER
    _g_ChanConfig.Offset       = 0;

    rc = HAL_ADC_ConfigChannel (&_g_AdcHandle, &_g_ChanConfig);
    if (rc != HAL_OK)
       {
            /* Channel Configuration Error */
         board_error_handler();           // return error code instead  ???
       }

          // save the step number that was used for this channel
    _g_adc_step_map [channel_num] = (char) step_num;

    if (orig_step_num == ADC_AUTO_STEP)
       _g_step_num++;          // inc to next step slot in the sequencer

          // track how many channels have been configured for use
    _g_active_channels++;

    return (0);                // denote success
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
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( ! _g_DMA_complete)
       return (0);                       // ADC and DMA are still busy

    return (1);                          // All conversions are completed
}


//*****************************************************************************
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    uint32_t   DataEntries;
    int        rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

//  if (sequencer == ADC_AUTO_SEQUENCE)
//     sequencer = _g_sequencer;  // use current auto-managed sequencer

    if ( !  _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    _g_DMA_complete = 0;  // clear I/O flags
    _g_DMA_overrun  = 0;

        //------------------------------------------------------------------
        // Update the "Number of Copnversions" in the ADC block
        // based upon the final number of channels configured.
        // To do this, we must update the ADC_SQR1_L field in SQR1 register
        //------------------------------------------------------------------
    _g_AdcHandle.Instance->SQR1 &= ~(ADC_SQR1_L);         // clear the bits
    _g_AdcHandle.Instance->SQR1 |= _g_active_channels;

        //----------------------------------------------------------------------
        // the following both starts the ADC and apparently auto-initiates
        // the first Conversion IFF SW initiated, else lets trigger do its thing
        //----------------------------------------------------------------------
    DataEntries = _g_active_channels * 1;  // set # entries that DMA should xfer
    rc = HAL_ADC_Start_DMA (&_g_AdcHandle, (uint32_t*) &_g_adc_conv_results,
                            DataEntries);
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         board_error_handler();           // return error code instead  ???
       }
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
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    HAL_ADC_Stop_DMA (&_g_AdcHandle);

    return (0);                                     // denote success
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
    uint32_t  adc_module;
    int      i;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

// CAUTION: TI's code will automatically read an entire ARRAY (from FIFO)
//          if a sequence group is involved

//  rc = ADCSequenceDataGet (uint32_t ui32Base, uint32_t ui32SequenceNum,
//                           uint32_t *pui32Buffer)

//  channel_results[0] = HAL_ADC_GetValue (ADC1);   DMA will have already put it in channel_results

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    if (_g_DMA_complete == 0)
       return (0);     // denote 0 results because DMA rupt has not happened yet

    if (_g_DMA_overrun)
       {    // we had an overrun. Discard the results and tell user try again
         _g_DMA_overrun  = 0;  // clear error flag
         _g_DMA_complete = 0;  // reset for new pass
         return (-1);          // denote we had an overrun condition
       }
    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_DMA_complete = 0;         // reset for new pass

    return (_g_active_channels); // pass back number of completed conversions
}


//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a group of ADCs, on a sequenced group
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
    uint32_t   adc_module;
    uint32_t   DataEntries;
    int        rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

//  if (sequencer == ADC_AUTO_SEQUENCE)
//     sequencer = _g_sequencer;      // use current auto-managed sequencer

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    _g_DMA_complete = 0;              // clear I/O flags for new pass
    _g_DMA_overrun  = 0;

        // apparently, to start another ADC conversion, you must issue
        // another xxx_Start_IT()  or  xxx_Start_DMA() request !
// DMA was already initialized and setup for circular mode (effectively resets
// after every cycle). So all we want to do now is trigger the ADC Start again.
// Do not re-init everything, it can cause crashes/faults
//  DataEntries = _g_active_channels * 1;   // set # entries that DMA should xfer
//  rc = HAL_ADC_Start_DMA (&_g_AdcHandle, (uint32_t*) &_g_adc_conv_results,
//                          DataEntries);
    rc = HAL_ADC_Start_IT (&_g_AdcHandle); // Issue software start (SW) to ADC and DMA
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         board_error_handler();            // return error code instead  ???
       }

    return (0);                                    // denote success
}


//*****************************************************************************
//  board_adc_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when the ADC conversions are complete.
//*****************************************************************************
int  board_adc_set_callback (int adc_module_id,
                             ADC_CB_EVENT_HANDLER callback_function,
                             void *callback_parm)
{
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

        //----------------------------------------------------------------
        // Save ADC completion callback parameters.
        //----------------------------------------------------------------
    _g_adc_callback      = callback_function;
    _g_adc_callback_parm = callback_parm;

    return (0);                           // denote success
}


/****************************************************************************
*                             ADC  ISR   Callback
*
*         ADC Conversion complete callback in non-blocking mode.
*         Called from DMA/ADC ISR Handler.
*
*         If running straight interrupts (xxx_IT), this gets called by
*         the HAL ADC library support when the ADC interrupt completes.
*
*         If running with DMA (xxx_DMA), this gets called
*         when the DMA interrupt completes, i.e. this gets
*         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
*
* @param  _g_AdcHandle : _g_AdcHandle handle
* @note   This example shows a simple way to report end of conversion,
*         and you can add your own implementation.
* @retval None
****************************************************************************/
void   HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *_g_AdcHandle)
{
    dma_callback_seen++;                        // DEBUG COUNTER

// ??? !!!  ADD REST OF SUPPORT FOR THIS on 06/26/15 ---- ??? !!!

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (_g_adc_callback != 0L)
       {
          (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                             _g_active_channels, 0);    // Call user handler
       }
}


/************************************************************************
*                              DMA  ADC  ISR
*
* @brief  This handles the DMA interrupt request for the ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA1_Channel1_IRQHandler (void)
{
    dma_rupt_seen++;                               // DEBUG COUNTER

    _g_DMA_complete = 1;     // set status that ADCs and DMA completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the foolowing really needed - should I just clear it in here and be done with it ???

    HAL_DMA_IRQHandler (_g_AdcHandle.DMA_Handle);  // Call post rupt cleanup
                                                   //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (_g_adc_callback != 0L)
       {
          (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                             _g_active_channels, 0);    // Call user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}

//#endif                       //  defined(USES_ADC)




     uint16_t    _g_trigger_dtmr_mmsmask = 0; // Associated Mask for TIM's  MMS
     uint16_t    _g_trigger_duser_api_id = 0; // Associated DAC trigger type from dac_Init()
/////                MasterSlaveConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

    uint16_t     _g_trigger_dtmr_mmsmask_1 = 0; // Chan 1 Associated Mask for TIM's  MMS
    uint16_t     _g_trigger_duser_api_id_1 = 0; //   "    Associated DAC trigger type
    uint16_t     _g_trigger_dtmr_mmsmask_2 = 0; // Chan 2 Associated Mask for TIM's  MMS
    uint16_t     _g_trigger_duser_api_id_2 = 0; //   "    Associated DAC trigger type

//#if defined(USES_DAC)

//*****************************************************************************
//*****************************************************************************
//                               DAC   Routines
//*****************************************************************************
//*****************************************************************************

    // STM32 F4 has no on-board DACs.  Must use an external SPI attached DAC

#//endif                       //  defined(USES_DAC)




//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
*  Board GPIO Init
*
*        Initialize GPIO pins needed by CC3000:  SPI CS, ENABLE,  IRQ
*******************************************************************************/
void  board_gpio_init (void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();     // turn on GPIO clocks for Ports A, B, C
    __HAL_RCC_GPIOB_CLK_ENABLE();     // since they are used in many spots
    __HAL_RCC_GPIOC_CLK_ENABLE();

#if defined(USES_CC3000) || defined(USES_CC3100) || defined(USES_CC3200)
        //--------------------------------------------
        // Configure GPIO pins:  PB4 (CC3000_ENABLE)
        //                       PB6 (SPI_CS_CC3000)
        //--------------------------------------------
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

        //---------------------------------------------------------
        // Configure GPIO pin:   PB3 (CC3000_IRQ)
        //
        // Note:  IRQ is signalled by a FALLING edge.
        //        Beware: bug in STM32CubeMX can set it to
        //        use Events (GPIO_MODE_EVT_FALLING) rather
        //        than HW interruptes (GPIO_MODE_IT_FALLING)
        //---------------------------------------------------------
    GPIO_InitStruct.Pin   = GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

        //--------------------------------------------
        // EXTI interrupt init for PB3 (CC3000_IRQ)
        //--------------------------------------------
    HAL_NVIC_SetPriority (EXTI3_IRQn, 0, 0);
/// HAL_NVIC_EnableIRQ (EXTI3_IRQn); // Let CC3000 logic call WlanInterruptEnable()
#endif         // defined(USES_CC3000) || defined(USES_CC3100) || defined(USES_CC3200)

}


/*******************************************************************************
*  board GPIO pin config
*
*        Configure an individual GPIO pin
*******************************************************************************/
void  board_gpio_pin_config (int gpio_port_num, unsigned long gp_pin,
                             int dir,  int pull)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_TypeDef       *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_port_num];

    GPIO_InitStruct.Pin = gp_pin;
    if (dir == GPIO_OUTPUT)
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // GPIO Output
       else GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    // GPIO Input
    GPIO_InitStruct.Alternate = 0;
    if (pull == GPIO_PULLUP)
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       else if (pull == GPIO_PULLDOWN)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;
               else GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

    HAL_GPIO_Init (gpio_port, &GPIO_InitStruct);        // Setup GPIO pin
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
    GPIO_TypeDef  *gpio_port;
    uint16_t      port_values;

//  if (gpio_port_num < 1 || gpio_port_num > MAX_GPIO_PORTS)
//     return (0xFFFF);                               // denote error

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_port_num];

    if ( ! flag)
       port_values = gpio_port->IDR & (pins_mask);      // get INPUT port pins
       else port_values = gpio_port->ODR & (pins_mask); // get OUTPUT port pins

    return (port_values);
}


/*******************************************************************************
*  board_irq_pin_config
*
*        Configure an individual GPIO pin to support incoming Interrupt requests
*******************************************************************************/
void  board_irq_pin_config (int gpio_port_num,  unsigned long gp_pin,
                            int rise_fall, int pullup, uint32_t irq_vector_num,
                            unsigned long priority)

{
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_TypeDef       *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_port_num];

    GPIO_InitStruct.Pin  = gp_pin;
    if (rise_fall == GPIO_RUPT_MODE_RISING)
       GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // GPIO Rising Input triggers
       else if (rise_fall == GPIO_RUPT_MODE_FALLING)
            GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //Falling Input trigger
    GPIO_InitStruct.Alternate = 0;
    if (pullup == GPIO_PULLUP)
       GPIO_InitStruct.Pull = GPIO_PULLUP;               // Engage pullups
       else if (pullup == GPIO_PULLDOWN)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;     // Engage pulldowns
               else GPIO_InitStruct.Pull = GPIO_NOPULL;  // Disengage pullups
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (gpio_port, &GPIO_InitStruct);         // Setup GPIO pin

        // Configure the NVIC for the Interrupt coming from the BlueNRG shield
    HAL_NVIC_SetPriority (irq_vector_num, priority, 0);

        // Model TI's support, and avoid any "early interrupts" that
        // ISR may not be expecting (e.g. CC3100 Simplelink)   04/20/15 change
    HAL_NVIC_DisableIRQ (irq_vector_num);
}


/*******************************************************************************
*  board_irq_pin_enable
*
*        Enable the GPIO pin for an incoming interrupt
*******************************************************************************/
void  board_irq_pin_enable (int gpio_port_num,  unsigned long pin,
                            uint32_t irq_vector_num, int clear_pending)
{
    GPIO_TypeDef       *gpio_port;

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_port_num];

    if (clear_pending)
       HAL_NVIC_ClearPendingIRQ (irq_vector_num);//clear any previous interrupts
    HAL_NVIC_EnableIRQ (irq_vector_num);
}


#if defined(USES_I2C)
//*****************************************************************************
//*****************************************************************************
//                               I2C   Routines
//*****************************************************************************
//*****************************************************************************

   //  need to supply for STM32  F4

#endif                                   // USES_I2C



//*****************************************************************************
//*****************************************************************************
//                               SPI   Routines
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
* board spi init function          was   SPI2_Init
*
*         Configures this MCU's SPI peripheral and its associated GPIOs.
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
    GPIO_InitTypeDef   GPIO_InitStruct;

SPI_HandleTypeDef  *board_spi_init (int spi_module_id,  int _g_spi_mode,
                                    int baud_rate_scalar,  int use_dma)
{
    SPI_HandleTypeDef  *phspi;
    int                rc;
    GPIO_InitTypeDef   GPIO_InitStruct;

                                  /* Definition for SPIx clock resources */
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_TX_DMA_STREAM               DMA1_Stream4
#define SPIx_RX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_RX_DMA_STREAM               DMA1_Stream3

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Stream4_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Stream3_IRQn
#define SPIx_DMA_TX_IRQHandler           DMA1_Stream4_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Stream3_IRQHandler


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

      /*##-4- Configure the NVIC for DMA #########################################*/
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
           _g_hdma_spi2_rx.Instance                 = DMA1_Channel4;    // RX
           _g_hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
           _g_hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
           _g_hdma_spi2_rx.Init.MemInc              = DMA_MINC_DISABLE; // ISSUE ???
           _g_hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
           _g_hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
           _g_hdma_spi2_rx.Init.Mode                = DMA_NORMAL;
           _g_hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
           HAL_DMA_Init (&_g_hdma_spi2_rx);

           __HAL_LINKDMA (&_g_hspi2, hdmarx, _g_hdma_spi2_rx);

           _g_hdma_spi2_tx.Instance                 = DMA1_Channel5;  // TX
           _g_hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
           _g_hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
           _g_hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
           _g_hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
           _g_hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
           _g_hdma_spi2_tx.Init.Mode                = DMA_NORMAL;
           _g_hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
           HAL_DMA_Init (&_g_hdma_spi2_tx);

           __HAL_LINKDMA (&_g_hspi2, hdmatx, _g_hdma_spi2_tx);
#endif                            // defined(USE_DMA)

         //--------------------------------------------------------------
         // This is a little bit overkill, but it handles all current
         // SPI variations of supported X-Nucleo boards. In most cases
         // this allows many diffrent combinations of stacked X-Nucleo
         // boards (subject to resolving CS conflicts, etc).
         //--------------------------------------------------------------

    switch (spi_module_id)             // STM32  F401RE  SPI   Support
      {
       case SPI_ID_1_A:                // SPI 1 A    uses pins PA5/PA6/PA7
                                       // Typically used by L6474 Stepper shield
               __SPI1_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
               phspi = &_g_hspi1;         // point at associated SPI "Handle"
               phspi->Instance = SPI1; //    and set associated SPI module
               _g_hspi1_type = SPI_ID_1_A;
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
               phspi = &_g_hspi1;       // point at our associated SPI "Handle"
#endif
               phspi->Instance = SPI1; //    and set associated SPI module
               _g_hspi1_type = SPI_ID_1_B;
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
               phspi = &_g_hspi2;         // point at associated SPI "Handle"
               phspi->Instance = SPI2; //    and set associated SPI module
               _g_hspi2_type = SPI_ID_2_C;
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
               phspi = &_g_hspi3;         // point at associated SPI "Handle"
               phspi->Instance = SPI3; //    and set associated SPI module
               _g_hspi3_type = SPI_ID_3_C;
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
    if (_g_spi_mode == 0)
      { phspi->Init.CLKPolarity = SPI_POLARITY_LOW;
        phspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (_g_spi_mode == 1)
      { phspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
        phspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (_g_spi_mode == 2)
      { phspi->Init.CLKPolarity = SPI_POLARITY_LOW;  // CPOL = 0, CPHA = 1
        phspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      }
     else if (_g_spi_mode == 3)
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

    return (phspi);      // return ptr to SPI module that will be used for
                         // caller's SPI device I/O
}


/*******************************************************************************
* Enable DMA controller clock and initialize DMA Interrupts
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
//*****************************************************************************

void board_stop_WDT (void)
{
      // chip default is it comes up with WWDG and IWDG in reset state
      // This is done in SystemInit(), which is part of the CMSIS startup logic.
}


/*******************************************************************************
* System Clock Configuration
*
*         System Clock is configured as follows:
*            System Clock source            = PLL (HSI)
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000   40 MHz  1/2 speed
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
*******************************************************************************/
void  board_system_clock_config (long  mcu_clock_hz)
{
  RCC_ClkInitTypeDef  RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef  RCC_OscInitStruct = {0};

      /* The following clock configuration sets the Clock configuration sets after System reset                */
      /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
      /* and to be eventually adapted to new clock configuration                                               */

      /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
     {
         /* Initialization Error */
     while(1);
     }

      /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
      /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
     {
         /* Initialization Error */
     while(1);
     }

     /* The voltage scaling allows optimizing the power consumption when the device is
     *  clocked below the maximum system frequency, to update the voltage scaling value
     *  regarding system frequency refer to product datasheet.  */

     /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
     {
           /* Initialization Error */
     while(1);
     }

      /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

/// _g_SysClk_Ticks = MCLK_MHz * 1000000;   // save MCU clock frequency in ticks
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
//       On STM32, we set I/O clock to same rate as MCU clock.
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
     // Nothing to do here, because HAL_Init() automatically starts
     // up the SysteickTimer with a 1 milli-second period
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
//                                HAL_GetTick
//
//            Get the current value of the "SYSTICK" style Interval Timer.
//
//THIS OVERRIDEs the DEFAULT STM32 HAL Systick function weak in stm32f0xxx_hal.c
//*****************************************************************************

uint32_t  HAL_GetTick (void)
{
    return (_g_systick_millisecs);
}


//*****************************************************************************
//                                HAL_IncTick
//
// SysTick interrupt handler.
// Directly called from Systick ISR !
//
//                               Increment Systick 1 ms count, on every 1ms pop
//
//THIS OVERRIDEs the DEFAULT STM32 HAL Systick function weak in stm32f0xxx_hal.c
//*****************************************************************************
void  HAL_IncTick (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)

#if defined(USES_MQTT)
    extern  unsigned long    MilliTimer;
    MilliTimer++;            // update MQTTCC3100.c's associated Timer
/// SysTickIntHandler();     // invoke MQTTCC3100.c to update its Timer
#endif

#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration (_g_systick_millisecs);
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
//          CAUTION: you MUST set _g_UartHandle.Init.Parity to UART_PARITY_NONE,
//                   else HAL Lib will default to odd parity and turn on the
//                   UART CR1 PCE and PS bits, whioh totally screws things up !
//*****************************************************************************

void  board_uart_init (void)
{
    // see Clive Example that works fine  (uses old Std Lib, not HAL)

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    GPIO_InitTypeDef    GPIO_InitStruct;

       //--------------------------------------------------
       // Configure the GPIOs that used for the UART pins
       // PA.2 = USART2_TX    PA.3 = USART2_RX
       //--------------------------------------------------
    GPIO_InitStruct.Pin        = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Alternate  = GPIO_AF7_USART2;     // set Alt Function - UART
    GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull       = GPIO_PULLUP;
    GPIO_InitStruct.Speed      = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);          // Setup UART GPIO pins

       //--------------------------------------------------
       // Configure the UART module.  Default = 115200
       //--------------------------------------------------
    __HAL_RCC_USART2_CLK_ENABLE();                  // Turn on UART clocks

    _g_UartHandle.Instance        = USART2;         //   Set UART module to use
    _g_UartHandle.Init.BaudRate   = 115200;
    _g_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;     // setup as 8N1
    _g_UartHandle.Init.StopBits   = UART_STOPBITS_1;
    _g_UartHandle.Init.Parity     = UART_PARITY_NONE;       // no parity  KEY !
    _g_UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;    // No flow ctl
    _g_UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    _g_UartHandle.Init.Mode       = UART_MODE_TX_RX;        // Enable RX and TX

    HAL_UART_Init (&_g_UartHandle);
#endif

}


//*****************************************************************************
//  board_uart_rx_data_check                 aka   CONSOLE_CHECK_FOR_READ_DATA
//
//             Checks if any pending character input from UART.
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

int  board_uart_rx_data_check (void)
{
#if defined(USES_CONSOLE_READ)
    int  rc;

          // see if any rcvd chars are in UART
    rc = (_g_UartHandle.Instance->ISR & USART_FLAG_RXNE);
    if (rc == RESET)
       return (0);                         // no, UART data is present

    return (1);                            // yes, we have some data
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
                 //----------------------------------------------
                 // read in any character that the user typed in
                 //----------------------------------------------

          // see if any rcvd chars are in UART
/// rc = USART_GetFlagStatus (USART2, USART_FLAG_RXNE);
    rc = (_g_UartHandle.Instance->ISR & USART_FLAG_RXNE);
    if (rc == RESET)
       return (0);                         // no UART data is present

///  in_char = USART_ReceiveData (USART2);  // read in char from UART
    in_char = (char) (_g_UartHandle.Instance->RDR & (uint8_t) 0x007F);

#if defined(USES_CONSOLE_READ)
	// Check if a previous board_uart_read_string() had left a dangling \r\n situation.
    if (_g_uart_last_char == '\r'  &&  in_char == '\n')
       {      // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
         _g_uart_last_char = 0;     // clear out the \r, so we treat any
                                    // new \n as real.
         return (0);                // "nothing to see here"
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
           // read in any character that the user typed in.
           // first - see if any rcvd chars are in UART
     rc = (_g_UartHandle.Instance->ISR & USART_FLAG_RXNE);
     if (rc == RESET)
        continue;                           // user needs to type in more chars

           // read in char from uART
     in_char = (char) (_g_UartHandle.Instance->RDR & (uint8_t) 0x007F);

        // note: board_uart_read_string MUST ALSO echo back the char to user
     if (in_char != '\n')
        {
          board_uart_write_char (in_char);  // echo the char  (\n is special)
        }

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

          board_uart_write_char ('\n');   // send a \n in anticipation,
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
          else {    // there was no preceding \r, so go ahead and echo the \n
                 board_uart_write_char ('\n');
               }
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
    int    rc;

    rc = RESET;
    while (rc == RESET)
       rc = (_g_UartHandle.Instance->ISR & USART_FLAG_TXE); // Wait for TX buf Empty

    _g_UartHandle.Instance->TDR = (outchar & (uint8_t )0xFF);   // Send the Char
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
    int    rc;

    while (*outstr != '\0')
      {
        rc = RESET;
        while (rc == RESET)
           rc = (_g_UartHandle.Instance->ISR & USART_FLAG_TXE); // Wait for TX buf Empty

        _g_UartHandle.Instance->TDR = (*outstr & (uint8_t )0xFF);   // Send the Char

        outstr++;                                 // step to next char in buffer
      }
#endif

}



#if defined(USES_UNIQUEID) || defined(USES_MQTT)

//*****************************************************************************
//*****************************************************************************
//                           UNIQUE-ID  /  CRC   Routines
//*****************************************************************************
//*****************************************************************************

    CRC_HandleTypeDef   CrcHandle;
    uint32_t            u32Buffer[8];   // STM32 only supports 32-bit data bufs


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
    int           i;

    __HAL_RCC_CRC_CLK_ENABLE();      // turn on CRC module's clocks

            // Configure the CRC peripheral
    CrcHandle.Instance = CRC;
    i = HAL_CRC_Init (&CrcHandle);
    if (i != HAL_OK)
       {
          while (1);                 // Initialization Error - hang for debugger
       }
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

    in_buf8 = (unsigned char*) in_buf;
    if (flags_32_8 == 8)
       { for (i = 0;  i < in_buf_length;  i++)
            u32Buffer[i] = in_buf8[i];  // convert from 8-bit to L.O. 32-bit
            // STM32: generate random seed from CRC unit
         crcResult = HAL_CRC_Accumulate (&CrcHandle, u32Buffer, 6);  // yields 0
         crcResult = HAL_CRC_Calculate (&CrcHandle, u32Buffer, 6);   // ditto WTF
       }
      else
       {    // is already in 32-bit format - use in place
            // STM32: generate random seed from CRC unit
         crcResult = HAL_CRC_Accumulate (&CrcHandle, in_buf,
                                         in_buf_length);  // yields 0
         crcResult = HAL_CRC_Calculate  (&CrcHandle, in_buf,
                                         in_buf_length);  // ditto WTF
       }

    _g_crcid_Result = crcResult;                     // all done, Save results

    return (_g_crcid_Result);
}

#endif                          // USES_UNIQUEID  ||  USES_MQTT



//#if defined(USES_VTIMER)

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

void  board_vtimer_check_expiration (uint32_t gsystick_millisecs)
{
    int   i;

          //-------------------------------------------------------------
          // See if any virtual timers have reached their expiration time
          //-------------------------------------------------------------
    for (i = 0; i < 10;  i++)
      {
        if (_g_vtimer_flags[i] != VTIMER_BUSY)
           continue;                       // timer is not active, skip it

        if (_g_vtimer_expire [i] > gsystick_millisecs)
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
//#endif



//*****************************************************************************
//*****************************************************************************
//                               CC3000   DEVICE   Routines
//*****************************************************************************
//*****************************************************************************




#if defined(USES_CC3000)  ||  defined(CC3000_USE_BOOSTERPACK1)
//*****************************************************************************
// The following routines/callbacks are invoked by the CC3000 Host Driver
// library, as part of its normal course of operation
//*****************************************************************************

//*****************************************************************************
//
//  ReadWlanInterruptPin
//
//
//       return CC3000 interrupt pin status     IRQ   PB3  Arduino Digital # 3
//*****************************************************************************
long  ReadWlanInterruptPin (void)
{
    irq_state = HAL_GPIO_ReadPin (CC3000_IRQ_IN_GPIO_PORT, CC3000_IRQ_IN_PIN);
/// irq_state = __HAL_GPIO_EXTI_GET_IT (CC3000_IRQ_IN_PIN);

    return (irq_state);
}


//*****************************************************************************
//  WlanInterruptEnable
//
//       Enable interrupts from the CC3000 IrQ pin
//
//       Note: this ultimately invokes CMSIS's core_cm4.h NVIC_EnableIRQ()
//             macro, which updates NVIC->ISER
//             For software testing, can use NVIC_SetPendingIRQ(IRQn_Type IRQn)
//             to force it on.
//*****************************************************************************
void  WlanInterruptEnable()
{
    HAL_NVIC_EnableIRQ (EXTI3_IRQn);

    IRQ_interrupts_enabled = 1;
}

//*****************************************************************************
//  WlanInterruptDisable
//
//       Disable interrupts from the CC3000 IRQ pin
//
//       Note: this ultimately invokes CMSIS's core_cm4.h NVIC_EnableIRQ()
//             macro, which updates NVIC->ICER
//*****************************************************************************
void  WlanInterruptDisable()
{
    HAL_NVIC_DisableIRQ (EXTI3_IRQn);

    IRQ_interrupts_enabled = 0;

//  __HAL_GPIO_EXTI_CLEAR_IT (CC3000_IRQ_IN_PIN);  // clear any left over rupts
}


//*****************************************************************************
//
//  WriteWlanPin                              ENABLE  PB4  Arduino Digital # 5
//
//       Toggle CC3000 PIN to ENABLE and DISABLE the CC3000 Radio.
//
//       Note: Raising the ENABLE pin, causes and IRQ low/high toggle sequence
//             from the CC3000 to inidcate that its internal RESET is complete.
//             This sequence is handled
//*****************************************************************************
void  WriteWlanPin (unsigned char hi_lo_val)
{
    if (hi_lo_val)
       {
         CC3000_ENABLE_HIGH();  // Enable  WIFI chip - will cause an IRQ toggle
       }
      else
       {
         CC3000_ENABLE_LOW();   // Disable WIFI chip
       }
}
#endif                          //  end  #if defined(USES_CC3000)


/******************************************************************************/
