// 07/15/15   ADC still broke. Is it got an incorrect count/EOS setting that is screwing up the DMA,
//            or is DMA count off ???

/********1*********2*********3*********4*********5*********6*********7**********
*
*                             board_F7.c - STM32 F7
*
*
* STM32Cube / HAL based version of board_xxx() functions
*
* NOTE: for this initial release, it only handles the key functions (SPI, I2C,
*       Timers, ...) that are pinned out to the "Arduino" headers.
*       Later releases will include wider support for the rest of the peripherals
*       on the board.
*
*
* Supported/Tested Boards
* -----------------------
*      STM32_F7_46_NG    Discovery   1024K Flash     96K RAM   216 MHz
*                                        3 ADC @ 16 channels
*                                        3 SPI       3 I2C       1 USB
*
*  History:
*    07/13/15 - Created for Industrial IoT OpenSource project.  Duq
*    07/15/15 - Convert over to use USART1 for the Virtual Com port on F7 Disco.
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

#include "stm32f7xx_hal.h"


#if defined(USES_CC3000)
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
        GPIOE,
        GPIOF,
        GPIOG,
        GPIOH,
        GPIOI
    };
#define  GP_PORT_A   1        // indexes into above table, based on port id
#define  GP_PORT_B   2
#define  GP_PORT_C   3
#define  GP_PORT_D   4
#define  GP_PORT_E   5
#define  GP_PORT_F   6
#define  GP_PORT_G   7
#define  GP_PORT_H   8
#define  GP_PORT_I   9


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
           //----------------------------------------------------------------
           // Enable the CPU caches, both Instruction (Flash) and Data (RAM)
           //----------------------------------------------------------------
    SCB_EnableICache();               // Enable I-Cache */
    SCB_EnableDCache();               // Enable D-Cache

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




    int              _g_trigger_atmrpwm      = 0; // index to correct Timer/PWM
    uint16_t         _g_trigger_auser_api_id = 0; // User API id for the trigger
    uint16_t         _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
    uint32_t         _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

#if defined(USES_ADC)

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

    unsigned char    _g_adc_step_map [16] = { 0 };  // indexed by channel #

    unsigned short   _g_adc_conv_results[16];    // Internal buf to hold ADC DMAed results

ADC_CB_EVENT_HANDLER _g_adc_callback       = 0L;
    void             *_g_adc_callback_parm = 0;

    int              _g_DMA_complete  = 0;       // 1 = all ADC conversion DMAs are complete
    char             _g_DMA_overrun   = 0;       // 1 = DMA completed a new set before the previous one was processed
    unsigned long    dma_callback_seen = 0;      // DEBUG COUNTERs
    unsigned long    dma_rupt_seen     = 0;

    ADC_HandleTypeDef       _g_AdcHandle_Mod_1;  // ADC 1 handle
    ADC_HandleTypeDef       _g_AdcHandle_Mod_2;  // ADC 2 handle
    ADC_HandleTypeDef       _g_AdcHandle_Mod_3;  // ADC 2 handle
    ADC_ChannelConfTypeDef  _g_ChanConfig;
    DMA_HandleTypeDef       _g_DmaHandle;

    void  ADC1_DMA_IRQHandler (void);              // Function Prototypes

typedef struct adc_module_def          /* ADC Module definitions */
    {
        ADC_TypeDef  *adc_base;        /* ADC base address */
    } ADC_MODULE_BLK;

typedef struct adc_channel_def         /* ADC Channel definitions */
    {
        GPIO_TypeDef      *chan_gpio_port;  /* Associated GPIO port           */
        uint32_t          chan_gpio_pin;    /* Associated GPIO pin            */
        uint32_t          chan_alt_func;    /* Channel Alternate Function Id  */
        uint32_t          chan_adc_id;      /* Logical Channel id for this ADC */
        ADC_HandleTypeDef *chan_adc_module; /* Default ADC Module to use       */
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
           { ADC2 },
           { ADC3 }     // ADC_MODULE_ANY defaults to ADC3
        };


// note: on STM32, user indexes start at 0, to handle channel 0 on STM's                User  Physical
const ADC_CHANNEL_BLK  _g_adc_channels [] =                            // STM32 Arduino Index  ADC
        {  { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_0, &_g_AdcHandle_Mod_3 },  // PA0   A0    0    ADC3_IN0
           { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_1, &_g_AdcHandle_Mod_3 },  // PA1         1    ADC123_IN1
           { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_2, &_g_AdcHandle_Mod_3 },  // PA2         2    ADC123_IN2
           { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_3, &_g_AdcHandle_Mod_3 },  // PA3         3    ADC123_IN3
           { GPIOF, GPIO_PIN_6, 0, ADC_CHANNEL_4, &_g_AdcHandle_Mod_3 },  // PF6   A5    4    ADC3_IN4
           { GPIOF, GPIO_PIN_7, 0, ADC_CHANNEL_5, &_g_AdcHandle_Mod_3 },  // PF7   A4    5    ADC3_IN5
           { GPIOF, GPIO_PIN_8, 0, ADC_CHANNEL_6, &_g_AdcHandle_Mod_3 },  // PF8   A3    6    ADC3_IN6
           { GPIOF, GPIO_PIN_9, 0, ADC_CHANNEL_7, &_g_AdcHandle_Mod_3 },  // PF9   A2    7    ADC3_IN7
           { GPIOF, GPIO_PIN_10,0, ADC_CHANNEL_8, &_g_AdcHandle_Mod_3 },  // PF10  A1    8    ADC3_IN8
           { GPIOF, GPIO_PIN_3, 0, ADC_CHANNEL_9, &_g_AdcHandle_Mod_3 },  // PF3         9    ADC3_IN9
           { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_10,&_g_AdcHandle_Mod_3 },  // PC0        10    ADC123_10
           { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_11,&_g_AdcHandle_Mod_3 },  // PC1        11    ADC123_11
           { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_12,&_g_AdcHandle_Mod_3 },  // PC2        12    ADC123_12
           { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_13,&_g_AdcHandle_Mod_3 },  // PC3        13    ADC123_13
           { GPIOF, GPIO_PIN_4, 0, ADC_CHANNEL_14,&_g_AdcHandle_Mod_3 },  // PF4        14    ADC3_IN14
           { GPIOF, GPIO_PIN_5, 0, ADC_CHANNEL_15,&_g_AdcHandle_Mod_3 }   // PF5        15    ADC3_IN15
        };

              //-----------------------------------------------
              // Triggerable Timers/Events available on F7 ADC
              //    - TIM1_CC1       - TIM3_CC4
              //    - TIM1_CC2       - TIM2_CC2
              //    - TIM1_CC3       - TIM4_CC4
              //    - TIM1_TRGO      - TIM1_TRGO_2
              //    - TIM2_TRGO      - TIM4_TRGO
              //    - TIM5_TRGO      - TIM6_TRG0
              //    - TIM8_TRGO      - TIM8_TRGO_2
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
          { ADC_TRIGGER_TIMER_1_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T1_CC1 },  // TImer1 CC1
          { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T1_CC2 },  // TImer1 CC2
          { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T1_CC3 },  // TImer1 CC3

          { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T2_CC2 },  // TImer2 CC2
          { ADC_TRIGGER_TIMER_5,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T5_TRGO},  // TImer5 Update/TRG0
          { ADC_TRIGGER_TIMER_4_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T4_CC4 },  // TImer4 CC4
          { ADC_TRIGGER_TIMER_3_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T3_CC4 },  // TImer3 CC4

          { ADC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO},  // TImer8 Update/TRG0
          { ADC_TRIGGER_TIMER_8_2,   TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO2}, // TImer8 Update/TRG0-2

          { ADC_TRIGGER_TIMER_1,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO},  // TImer1 Update/TRG0
          { ADC_TRIGGER_TIMER_1_2,   TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO2}, // TImer1 Update/TRG0-2

          { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T2_TRGO},  // TImer2 Update/TRG0
          { ADC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T4_TRGO},  // TImer4 Update/TRG0
          { ADC_TRIGGER_TIMER_6,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T6_TRGO},  // TImer6 Update/TRG0

          { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIGCONV_EXT_IT11 },// External GPIO
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
// STM32-F7 has 3 ADC moodules, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC_SQR1 -> ADC_SQR3.
// ADC results are stored in a single 32-bit register: ADC_DR
// Results are stored in the lower order 16-bits.
// We use DMA to process multiple ADC conversion results, to avoid
// "interrupt per channel" overhead and complexity.
//
// The STM32-F7 Discovery 200-pin device physically supports up to 16 channels,
// of which 5 are wired out to the Nucleo/Arduino pins.
// Two internal sources (Temperature Sensor, Battery Monitor) are also available.
// The other ADCs are wired out to the "Morpho" connector.
// We allow support for all 16 channels.
//
// The STM32-F7 supports three 12-bit ADCs, with up to 16 Channels.
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
       {     // turn on the clocks for ADC module  (There are 3 on STM32 F7)
         __HAL_RCC_ADC3_CLK_ENABLE();            // Enable ADC 3 Periph clock
         __HAL_RCC_DMA2_CLK_ENABLE();            // Enable DMA2 clock
         _g_adc_clocks_on = 1;                   // denote clocks are now on

             //-----------------------------------------------------------------
             // This is the opportune time to do initial config of the ADC and
             // DMA modules. Note that we will later have to manuually override
             // the initial number of ADC channels (set SQR1 xxxx value).
             //
             // If calibration is required, call that out as a separate subrtn.
             //-----------------------------------------------------------------

// ??? !!!  WVD   in future, create a pointer to the handle from an array (ala SPI routines) with ADC3 as default  !!! ???

         _g_AdcHandle_Mod_3.Instance                   = ADC3; // single module: ADC1
         _g_AdcHandle_Mod_3.Init.NbrOfConversion       = 1;    // - this gets modified -
         _g_AdcHandle_Mod_3.Init.DMAContinuousRequests = ENABLE;             // Use DMA
         _g_AdcHandle_Mod_3.Init.Resolution            = ADC_RESOLUTION_12B; // 12 bit resol
         _g_AdcHandle_Mod_3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

         if (trigger_type == ADC_TRIGGER_USER_APP)
            {     //-----------------------------------------------------------
                  // no external triggers - use SW trigger calls from User App
                  //-----------------------------------------------------------
              _g_AdcHandle_Mod_3.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
              _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
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
                       _g_AdcHandle_Mod_3.Init.ExternalTrigConv = trigblkp->trigger_adc_extmask;
                       if (flags & ADC_TRIGGER_FALLING)
                          _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
                          else if (flags & ADC_TRIGGER_RISEFALL)
                                  _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
                                  else _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
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

         _g_AdcHandle_Mod_3.Init.ScanConvMode          = ENABLE;  // scan multiple channels
         _g_AdcHandle_Mod_3.Init.ContinuousConvMode    = DISABLE; // only when trigger
         _g_AdcHandle_Mod_3.Init.DiscontinuousConvMode = DISABLE; // not used
         _g_AdcHandle_Mod_3.Init.NbrOfDiscConversion   = 0;
         _g_AdcHandle_Mod_3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
         _g_AdcHandle_Mod_3.Init.EOCSelection          = DISABLE; // DMA will interrupt
                                                            //   not ADC EOC.
         rc = HAL_ADC_Init (&_g_AdcHandle_Mod_3);
         if (rc != HAL_OK)
            {
                 /* Initialization Error */
              board_error_handler();           // return error code instead  ???
            }

             //-----------------------------------------------------------------
             // Initialize the DMA associated with the ADC module.
             //   ?? Or should we hold off on this until we hit adc_enable() ?
             //-----------------------------------------------------------------
         _g_DmaHandle.Instance         = DMA2_Stream0;  // F7 ADC3 uses DMA2
         _g_DmaHandle.Init.Channel     = DMA_CHANNEL_2; // F7 ADC3 uses DMA Chan 2
         _g_DmaHandle.Init.Direction   = DMA_PERIPH_TO_MEMORY;  // ADC -> RAM

         _g_DmaHandle.Init.PeriphInc   = DMA_PINC_DISABLE;
         _g_DmaHandle.Init.MemInc      = DMA_MINC_ENABLE;       // step buf ptr
         _g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         _g_DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
         _g_DmaHandle.Init.Mode        = DMA_CIRCULAR;     // Treat as circular buf
         _g_DmaHandle.Init.Priority    = DMA_PRIORITY_HIGH;
         _g_DmaHandle.Init.FIFOMode    = DMA_FIFOMODE_DISABLE;
         _g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
         _g_DmaHandle.Init.MemBurst    = DMA_MBURST_SINGLE;
         _g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

         rc = HAL_DMA_Init (&_g_DmaHandle);

            // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&_g_AdcHandle_Mod_3, DMA_Handle, _g_DmaHandle);

            //----------------------------------------------------------
            //          Configure the NVIC for DMA2 interrupts
            // Configure NVIC for DMA transfer complete interrupt.
            //----------------------------------------------------------
         HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
         HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
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
//         For the STM32 F7, there is only 3 ADC modules: ADC1, ADC2, ADC3
//         Nearly all of the 16 channels are connected to ADC3, and alternative
//         paths are defined to allow parallel operation with ADC1 or ADC2.
//
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//


//         STM32 F7 supports a max of 16 ADC channels and 3 ADC modules.
//         It has only 1 main sequencer.
//                    ^^^^^^^^^^^^^^^^^^^^
//                       sequencer per ADC ?  ==> array of sequncer/step values[module]id]


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
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
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
    _g_ChanConfig.Rank         = step_num;            // set step # in sequencer
    _g_ChanConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    _g_ChanConfig.Offset       = 0;

    rc = HAL_ADC_ConfigChannel (&_g_AdcHandle_Mod_3, &_g_ChanConfig);
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
    _g_AdcHandle_Mod_3.Instance->SQR1 &= ~(ADC_SQR1_L);         // clear the bits
    _g_AdcHandle_Mod_3.Instance->SQR1 |= ADC_SQR1(_g_active_channels);

        //----------------------------------------------------------------------
        // the following both starts the ADC and apparently auto-initiates
        // the first Conversion IFF SW initiated, else lets trigger do its thing
        //----------------------------------------------------------------------
    DataEntries = _g_active_channels * 1;  // set # entries that DMA should xfer
    rc = HAL_ADC_Start_DMA (&_g_AdcHandle_Mod_3, (uint32_t*) &_g_adc_conv_results,
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

    HAL_ADC_Stop_DMA (&_g_AdcHandle_Mod_3);

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
    rc = HAL_ADC_Start_IT (&_g_AdcHandle_Mod_3); // Issue software start (SW) to ADC and DMA
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

void  DMA2_Stream0_IRQHandler (void)
{
    dma_rupt_seen++;                               // DEBUG COUNTER

    _g_DMA_complete = 1;     // set status that ADCs and DMA completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???

    HAL_DMA_IRQHandler (_g_AdcHandle_Mod_3.DMA_Handle);  // Call post rupt cleanup
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

#endif                       //  defined(USES_ADC)




     uint16_t        _g_trigger_dtmr_mmsmask = 0; // Associated Mask for TIM's  MMS
     uint16_t        _g_trigger_duser_api_id = 0; // Associated DAC trigger type from dac_Init()
/////                MasterSlaveConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

#if defined(USES_DAC)

//*****************************************************************************
//*****************************************************************************
//                               DAC   Routines
//*****************************************************************************
//*****************************************************************************

    // STM32 F7 has two on-board DACs, but they are not pinned out to the
    // STM32 Discovery's Arduino pads.

    // Possible alternative is to use the WM8299 CODEC in DSP mode, to pass
    // thru the DAC results, at the CODEC's fixed frequencies 8/16/32/44/48 ... 198 KHz

#endif                       //  defined(USES_DAC)




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

    __HAL_RCC_GPIOA_CLK_ENABLE();     // turn on GPIO clocks used by Discovery's
    __HAL_RCC_GPIOB_CLK_ENABLE();     //        "Arduino" related pins
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

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

   //  need to supply for STM32  F7

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

SPI_HandleTypeDef  *board_spi_init (int spi_module_id,  int _g_spi_mode,
                                    int baud_rate_scalar,  int use_dma)
{
    SPI_HandleTypeDef  *phspi;
    int                rc;
    GPIO_InitTypeDef   GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();    // Turn on key closck used by F7 Discovery
    __HAL_RCC_GPIOB_CLK_ENABLE();    //      especially on "Arduino" connectors
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

         //--------------------------------------------------------------
         // This is a little bit overkill, but it handles all current
         // SPI variations of supported X-Nucleo boards. In most cases
         // this allows many diffrent combinations of stacked X-Nucleo
         // boards (subject to resolving CS conflicts, etc).
         //--------------------------------------------------------------

    switch (spi_module_id)             // STM32  F7  SPI   Support
      {
#if (FIX_LATER)
       case SPI_ID_1_A:                // SPI 1 A    uses pins PA5/PA6/PA7
                                       // Typically used by L6474 Stepper shield
               __SPI1_CLK_ENABLE();    // Ensure SPI peripheral clock turned on
               phspi = &_g_hspi1;      // point at associated SPI "Handle"
               phspi->Instance = SPI1; //    and set associated SPI module
               _g_hspi1_type   = SPI_ID_1_A;
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
#endif

       case SPI_ID_2_I:                // SPI I C    uses pins PI1/PB14/PB15
               __HAL_RCC_SPI2_CLK_ENABLE(); // Ensure SPI peripheral clock turned on
#if (USES_BLUENRG_BLE)
               phspi = &SpiHandle;     // point at BlueNRG's required SPI "Handle"
#else
               phspi = &_g_hspi2;      // point at our associated SPI "Handle"
#endif
               phspi->Instance = SPI2; //    and set associated SPI module
               _g_hspi2_type = SPI_ID_2_I;
                    // SPI 2 I uses GPIO pins PI1/PB14/PB15.
                    // Set them to SPI Alt Function
               GPIO_InitStruct.Pin       = SPI_2_I_SCLK_PIN;
               GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
               GPIO_InitStruct.Pull      = SPI_2_I_SCLK_PULL_MODE;
               GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
               HAL_GPIO_Init (GPIOI, &GPIO_InitStruct);    // Setup PI1 SPI pin
               GPIO_InitStruct.Pin       = SPI_2_I_MISO_PIN | SPI_2_I_MOSI_PIN;
               GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
               GPIO_InitStruct.Pull      = SPI_2_I_MOSx_PULL_MODE;
               HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); //Setup PB14/PB15 SPI pins
               break;

#if (FIX_LATER)
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
#endif
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
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 216000000
*            HCLK(Hz)                       = 216000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 4
*            APB2 Prescaler                 = 2
*            HSE Frequency(Hz)              = 25000000
*            PLL_M                          = 25
*            PLL_N                          = 432
*            PLL_P                          = 2
*            PLL_Q                          = 9
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale1 mode
*            Flash Latency(WS)              = 7
*******************************************************************************/
void  board_system_clock_config (long  mcu_clock_hz)
{
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;
    RCC_OscInitTypeDef  RCC_OscInitStruct;
    int                 rc;

         //----------------------------------------------------------------
         // Enable the HSE Oscillator and activate PLL with HSE as source
         //----------------------------------------------------------------
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 432;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 9;
    rc = HAL_RCC_OscConfig (&RCC_OscInitStruct);
    if (rc != HAL_OK)
       {
         board_error_handler();      // Clock is goofy. Dead in the water !
       }
         //----------------------------------------------------------------
         // Activate the OverDrive to reach the 216 MHz Frequency
         //----------------------------------------------------------------
    rc = HAL_PWREx_EnableOverDrive();
    if (rc != HAL_OK)
       {
         board_error_handler();      // Clock is goofy. Dead in the water !
       }
         //----------------------------------------------------------------
         // Select PLL as system clock source and configure the HCLK,
         // PCLK1 and PCLK2 clocks dividers
         //----------------------------------------------------------------
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    rc = HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_7);
    if (rc != HAL_OK)
       {
         board_error_handler();      // Clock is goofy. Dead in the water !
       }

    _g_SysClk_Ticks = mcu_clock_hz;  // save our clock frequency for Tick calcs
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



//#if defined(USES_PWM) || defined(USES_TIMER)

//*****************************************************************************
//*****************************************************************************
//               REAL  TIMER / PWM   Support           Physical Timers on chip
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//             COMMON   TIMER  and  PWM   GPIO  PIN   DEFINITIONS
//
//                                for
//
//                       PWM   and   OC/IC   Timers
//*****************************************************************************
//*****************************************************************************
//
//   These GPIO definitions for TIMx_CHx GPIO mappings are used in common by
//   the PWM logic when setting up PWM outputs, and by the Timer Output Compare
//   and Input Capture functions that use GPIOs.
//
// NOTE: due to the limited access to all of the pins (no Morpho headers),
//       only a subset of the Timer OCs will be supported, and little or no ALT
//       versions, except for those that map to the Arduino headers.
//
//------------------------------------------------------------------------------
//                    Timer / PWM Summary     available on STM32 F0
//                                                                  Supports
//  PWM Timer  Num CCRs   Complementary    DMA     Up/Down/Center     PWM
//    TIM1        4           Yes           Yes       All             Yes
//    TIM2        4           No            Yes       All             Yes
//    TIM3        4           No            Yes       All             Yes
//    TIM4        4           No            Yes       All             Yes
//    TIM5        4           No            Yes       All             Yes
//    TIM6        0           No            No        Up only         No (DAC)
//    TIM7        0           No            No        Up only         No (DAC)
//    TIM8        4           Yes           Yes       All             Yes
//    TIM9        2           No            No        Up only         Yes
//    TIM10       1           No            No        Up only         Yes
//    TIM11       1           No            No        Up only         Yes
//    TIM12       2           No            No        Up only         Yes
//    TIM13       1           No            No        Up only         Yes
//    TIM14       1           No            No        Up only         Yes
//------------------------------------------------------------------------------

#define  MAX_TIMER        TIMER_14

#define  TMRPWM_NUM_MODULES    14    // We allow a range of 1 to 11 for Timer
                                    // module Ids, but not all modules are
                                    // enabled or present. This just specifies
                                    // the max size of the array. Timer modules
                                    // not present will have a value of 0 and be
                                    // rejected if the user tries to invoke.
#define  TMRPWM_MAX_CHANNELS    4    // are max 4 channels per PWM module

              // flags for _g_pwm_module_status[] entries
#define  TMR_PWM_NORMAL_INIT      0x01  /* Module setup for normal output mode*/
#define  TMR_PWM_COMPLEMENT_INIT  0x08  /* Module setup for Complementary mode*/
#define  TMR_PWM_INTERRUPTS_USED  0x40  /* Module uses Timer interrupts */

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

                           // TIMx handles needed, one per module
    TIM_HandleTypeDef  _g_TIM1_TimPwmHandle;   // Timer/PWM  handle for TIM1
    TIM_HandleTypeDef  _g_TIM2_TimPwmHandle;   // Timer/PWM  handle for TIM2
    TIM_HandleTypeDef  _g_TIM3_TimPwmHandle;   // Timer/PWM  handle for TIM3
    TIM_HandleTypeDef  _g_TIM4_TimPwmHandle;   // Timer/PWM  handle for TIM4
    TIM_HandleTypeDef  _g_TIM5_TimPwmHandle;   // Timer/PWM  handle for TIM5
    TIM_HandleTypeDef  _g_TIM6_TimPwmHandle;   // Timer/PWM  handle for TIM6
    TIM_HandleTypeDef  _g_TIM7_TimPwmHandle;   // Timer/PWM  handle for TIM7
    TIM_HandleTypeDef  _g_TIM8_TimPwmHandle;   // Timer/PWM  handle for TIM8
    TIM_HandleTypeDef  _g_TIM9_TimPwmHandle;   // Timer/PWM  handle for TIM9
    TIM_HandleTypeDef  _g_TIM10_TimPwmHandle;  // Timer/PWM  handle for TIM10
    TIM_HandleTypeDef  _g_TIM11_TimPwmHandle;  // Timer/PWM  handle for TIM11
    TIM_HandleTypeDef  _g_TIM12_TimPwmHandle;  // Timer/PWM  handle for TIM12
    TIM_HandleTypeDef  _g_TIM13_TimPwmHandle;  // Timer/PWM  handle for TIM13
    TIM_HandleTypeDef  _g_TIM14_TimPwmHandle;  // Timer/PWM  handle for TIM14

                           // Bit mask status of each Timer Module (initialized, ...)
    char               _g_tmpwm_module_status [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0,0 };

                           // Bit Mask status of CCRs in each module (enabled, PWM, ...)
                           // One entry per timer module.
    unsigned char      _g_tmpwm_channels_config [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0,0,0,0 };

                           // Bit Mask PWM status of CCRs in each module (Normal, Complementary,...)
                           // One entry per timer module.
    unsigned char      _g_tmpwm_channels_pwm [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0 };

                           // Bit mask of which CCRs are using interrupts in each module.
                           // One entry per timer module.
    uint16_t           _g_timer_app_enabled_interrupts [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0,0 };

                           // Interrupt Callback addresses. One entry per timer module.
 TMR_CB_EVENT_HANDLER  _g_ptimer_callback [TMRPWM_NUM_MODULES+1] = { 0,0,0,0,0,0,0,0,0,0 };
    void               *_g_ptimer_callback_parm [TMRPWM_NUM_MODULES+1];

                           // Timer Prescalars used for each Timer module.
    uint32_t           _g_tmpwm_prescalars [TMRPWM_NUM_MODULES+1]    = { 0,0,0,0,0,0 };

        //----------------------------------------------------------------------
        //     Global constants and Tables used to manage Timer/PWM Logic
        //----------------------------------------------------------------------

const unsigned char    _g_pwm_chan_mask [8]     // Bit masks used to track if a channel
                                                // is enabled and/or using COMPLE
                                  = {      0,         /* Channel 0 is unused */
                                      TMR_CCR1_CONFIGURED,  /* Channels 1-4 */
                                      TMR_CCR2_CONFIGURED,  /*   0x02  */
                                      TMR_CCR3_CONFIGURED,  /*   0x04  */
                                      TMR_CCR4_CONFIGURED,  /*   0x08  */
                                      TMR_CCR1N_CONFIGURED, /* Channels 1N-3N */
                                      TMR_CCR2N_CONFIGURED, /*   0x20  */
                                      TMR_CCR3N_CONFIGURED  /*   0x40  */
                                    };                      // there is no 4N

                             //-------------------------------------------------------
                             // List of HAL constants to specify interrupt mask for a CCR
                             //-------------------------------------------------------
const unsigned char    _g_tmpwm_CCR_rupt_flags [5]
                                  = { TIM_SR_UIF,        /* CCR0 aka Rollover */
                                      TIM_SR_CC1IF,      /* CCR1 interrupt    */
                                      TIM_SR_CC2IF,      /* CCR2 interrupt    */
                                      TIM_SR_CC3IF,      /* CCR3 interrupt    */
                                      TIM_SR_CC4IF       /* CCR4 interrupt    */
                                    };

                             //-------------------------------------------------------
                             // List of constants used by HAL to identify CCR channels
                             //-------------------------------------------------------
const unsigned int     _g_timer_channel_id [5]
                                   = { 0,       // unused
                                       TIM_CHANNEL_1,       /* 0x0000 */
                                       TIM_CHANNEL_2,       /* 0x0004 */
                                       TIM_CHANNEL_3,       /* 0x0008 */
                                       TIM_CHANNEL_4        /* 0x000C */
                                     };

                             //-------------------------------------------------
                             //  List of Physical Base Address of TIMx registers
                             //-------------------------------------------------
const TIM_TypeDef      *_g_timer_module_base []   /* Register base address */
                                = { 0L,                    // no TIM0  on F7
                                    TIM1,  // full, w/deadtime, w/complementary,
                                    TIM2,  // no dead time, no complementary
                                    TIM3,  //   ditto       (servos, encoders)
                                    TIM4,  //   ditto
                                    TIM5,  //   ditto
                                    TIM6,  //   ditto
                                    TIM7,  //   ditto
                                    TIM8,  //   full, w/deadtime, w/complementary,
                                    TIM9,  //   no dead time, no complementary, ...
                                    TIM10, //   ditto
                                    TIM11, //   ditto
                                    TIM12, //   ditto
                                    TIM13, //   ditto
                                    TIM14  //   ditto      last valid timer on F7
                                  };

                             //----------------------------------------------------------
                             //  List of HAL Handle Address for each separate TIM Handle
                             //----------------------------------------------------------
const TIM_HandleTypeDef  *_g_timer_module_handle [] /* HAL API Handles */
                       = {             0L,                     // no TIM0  on F7
                           (TIM_HandleTypeDef*) &_g_TIM1_TimPwmHandle,  // TIM1
                           (TIM_HandleTypeDef*) &_g_TIM2_TimPwmHandle,  // TIM2
                           (TIM_HandleTypeDef*) &_g_TIM3_TimPwmHandle,  // TIM3
                           (TIM_HandleTypeDef*) &_g_TIM4_TimPwmHandle,  // TIM4
                           (TIM_HandleTypeDef*) &_g_TIM5_TimPwmHandle,  // TIM5
                           (TIM_HandleTypeDef*) &_g_TIM6_TimPwmHandle,  // TIM6
                           (TIM_HandleTypeDef*) &_g_TIM7_TimPwmHandle,  // TIM7
                           (TIM_HandleTypeDef*) &_g_TIM8_TimPwmHandle,  // TIM8
                           (TIM_HandleTypeDef*) &_g_TIM9_TimPwmHandle,  // TIM9
                           (TIM_HandleTypeDef*) &_g_TIM10_TimPwmHandle, // TIM10
                           (TIM_HandleTypeDef*) &_g_TIM11_TimPwmHandle, // TIM11
                           (TIM_HandleTypeDef*) &_g_TIM12_TimPwmHandle, // TIM12
                           (TIM_HandleTypeDef*) &_g_TIM13_TimPwmHandle, // TIM13
                           (TIM_HandleTypeDef*) &_g_TIM14_TimPwmHandle  // TIM14 - last valid timer on F7
                         };

                             //----------------------------------------------------------
                             // _g_TMPWM_IRQ_table
                             //
                             //     Table of IRQ numbers used to setup the associated
                             //     NVIC interrupt enable for a specific timer.
                             //----------------------------------------------------------
const unsigned char    _g_TMPWM_IRQ_table []
                                = { 0,              //  0 is not valid
                                    TIM1_CC_IRQn,   //  TIMER_1 CCRs only
                                                    //  Timer 1 updates handled in code (TIM1_UP_TIM10_IRQn)
                                    TIM2_IRQn,      //  TIMER_2 update and CCRs
                                    TIM3_IRQn,      //  TIMER_3   "     "   "
                                    TIM4_IRQn,      //  TIMER_4   "     "   "
                                    TIM5_IRQn,      //  TIMER_5   "     "   "
                                    TIM6_DAC_IRQn,  //  TIMER_6 update only
                                    TIM7_IRQn,      //  TIMER_7 update only
                                    TIM8_CC_IRQn,   //  TIMER_8 CCRs only
// ??? need to handle TIM8 rollovers as specuial case ala TIM1 (TIM8_UP_TIM13_IRQn)
                                TIM1_BRK_TIM9_IRQn, //  TIMER_9 update
                                TIM1_UP_TIM10_IRQn, //  TIMER_10 update  -- AND --  SHARED WITH TIM1 Update
                           TIM1_TRG_COM_TIM11_IRQn, //  TIMER_11 update      "               "    "
                           TIM8_BRK_TIM12_IRQn,     //  TIMER_12 update  -- AND --  SHARED WITH TIM8 Update
                           TIM8_UP_TIM13_IRQn,      //  TIMER_13 update      "               "    "
                           TIM8_TRG_COM_TIM14_IRQn  //  TIMER_14 update      "               "    "
                                  };

                             //-------------------------------------------------
                             // _g_timer_base_channel_num
                             //
                             //     converts user API logical channel numbers, such as
                             //     TMR_PWM_CHANNEL_1  / TMR_PWM_CHANNEL_1_ALT1 / TMR_PWM_CHANNEL_1_N
                             //     which are used to denote which Pin-Mux setting to
                             //     use for outputs that can be routed to different pins.
                             //
                             //     The _ALT1 / _ALT2 / _N / _N_ALT settings are
                             //     used to select the proper GPIO onto which to
                             //     route the signal.
                             //-------------------------------------------------
const unsigned char    _g_timer_base_channel_num []
                            = {   -1,               //  0 is not valid
                                TIM_CHANNEL_1,      //  1 TMR_PWM_CHANNEL_1
                                TIM_CHANNEL_2,      //  2 TMR_PWM_CHANNEL_2
                                TIM_CHANNEL_3,      //  3 TMR_PWM_CHANNEL_3
                                TIM_CHANNEL_4,      //  4 TMR_PWM_CHANNEL_4
                                  -1,               //  5 unused
                                  -1,               //  6 unused
                                  -1,               //  7 unused
                                TIM_CHANNEL_1,      //  8 TMR_PWM_CHANNEL_1_ALT1
                                TIM_CHANNEL_2,      //  9 TMR_PWM_CHANNEL_2_ALT1
                                TIM_CHANNEL_3,      // 10 TMR_PWM_CHANNEL_3_ALT1
                                TIM_CHANNEL_4,      // 11 TMR_PWM_CHANNEL_4_ALT1
                                TIM_CHANNEL_1,      // 12 TMR_PWM_CHANNEL_1_ALT2
                                TIM_CHANNEL_2,      // 13 TMR_PWM_CHANNEL_2_ALT2
                                TIM_CHANNEL_3,      // 14 TMR_PWM_CHANNEL_3_ALT2
                                TIM_CHANNEL_4,      // 15 TMR_PWM_CHANNEL_4_ALT2
                                TIM_CHANNEL_1,      // 16 TMR_PWM_CHANNEL_1_N
                                TIM_CHANNEL_2,      // 17 TMR_PWM_CHANNEL_2_N
                                TIM_CHANNEL_3,      // 18 TMR_PWM_CHANNEL_3_N
                                  -1,               // 19 unused (no Channel 4N)
                                TIM_CHANNEL_1,      // 20 TMR_PWM_CHANNEL_1_N_ALT1
                                TIM_CHANNEL_2,      // 21 TMR_PWM_CHANNEL_2_N_ALT1
                                TIM_CHANNEL_3,      // 22 TMR_PWM_CHANNEL_3_N_ALT1
                                  -1,               // 23 unused (no Channel 4N)
                                TIM_CHANNEL_1,      // 24 TMR_PWM_CHANNEL_1_N_ALT2
                                TIM_CHANNEL_2,      // 25 TMR_PWM_CHANNEL_2_N_ALT2
                                TIM_CHANNEL_3,      // 26 TMR_PWM_CHANNEL_3_N_ALT2
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
               // e.g. Timer Channel 1 (ex TIM1_CH1) is controlled by CCR1 on
               // the Timer 1 module; Timer Channel 3 (ex TIM2_CH3) is
               // controlled by CCR3 on the Timer 2 module, etc.
               //
               // A number of channels have alternative pin mappings, e.g.
               // TIM1_CH1 can be routed out to the PA8, PA7, or PB13 pins.
               // When multiple mappings exist, the 2nd and 3rd mappings
               // are distinguished as ALT1 and ALT2 in the user_api.h file
               // board specific entries. E.g. for TIM1_CH1, it would be
               // specified by the user as:
               //       CHANNEL_1           // default = PA8
               //       CHANNEL_1_ALT1      // use  PA7
               //       CHANNEL_1_ALT2      // use  PB13
               // See comments in the API documentation that describe which GPIO
               // pin each alternative is assigned to (e.g. PA8, PA7, or PB13)
               //--------------------------------------------------------------
               // NOTE: Ports D and E are _NOT_ hooked up on F0_72

typedef struct tim_pwm_channel_def     /* TIMER/PWM Channel definitions   */
    {
        GPIO_TypeDef *chan_gpio_port;  /* Associated GPIO port            */
        uint32_t     chan_gpio_pin;    /* Associated GPIO pin             */
        uint8_t      chan_alt_func;    /* Channel Alternate Function Id   */
        uint8_t      user_channel_id;  /* USER API designation: CHANNEL_1, CHANNEL_1_ALT1, ... */
        uint16_t     chan_pwm_id;      /* Chip Channel id for this CCR    */
    } TMPWM_CHANNEL_BLK;

                                                                                 // ??? !!!  WVD  ONLY FIELDS WITH  ARDUINO #s  are  verified !!! ???
const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_0_channels [] =                 // TIM0
       { {   0L,       0,             0,              0,              0       }, // Dummy entry
       };

const TMPWM_CHANNEL_BLK  __g_tmrpwm_mod_1_channels [] =                 // TIM1
       { {   0L,       0,             0,              0,              0       }, // STM32               Arduino
         { GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PA8  TIM1_CH1         D5
         { GPIOA, GPIO_PIN_9, GPIO_AF1_TIM1, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PA9  TIM1_CH2
         { GPIOA, GPIO_PIN_10,GPIO_AF1_TIM1, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PA10 TIM1_CH3
         { GPIOA, GPIO_PIN_11,GPIO_AF1_TIM1, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PA11 TIM1_CH4
                     // Complementary pins
         { GPIOB, GPIO_PIN_13,GPIO_AF1_TIM1, TMR_PWM_CHANNEL_1_N, TIM_CHANNEL_1 }, // PB13 TIM1_CH1N
         { GPIOB, GPIO_PIN_14,GPIO_AF1_TIM1, TMR_PWM_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB14 TIM1_CH2N      D12
         { GPIOB, GPIO_PIN_15,GPIO_AF1_TIM1, TMR_PWM_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB15 TIM1_CH3N      D11
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  __g_tmrpwm_mod_2_channels [] =                 // TIM2
       { {    0L,      0,             0,             0,             0         },  // STM32               Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM2_CH1         A0
         { GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PA1  TIM2_CH2
         { GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PA2  TIM2_CH3
         { GPIOA, GPIO_PIN_3, GPIO_AF1_TIM2, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PA3  TIM2_CH
     // Alternates
         { GPIOA, GPIO_PIN_15,GPIO_AF1_TIM2, TMR_PWM_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PA15  TIM2_CH1   D9
         { 0,          0,             0,              0,              0            }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_3_channels [] =                 // TIM3
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PB4  TIM3_CH1         D3
         { GPIOC, GPIO_PIN_7, GPIO_AF2_TIM3, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PC7  TIM3_CH2         D0
         { GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PB0  TIM3_CH3
         { GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PB1  TIM3_CH4
     // Alternates
         { GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3, TMR_PWM_CHANNEL_1_ALT1, TIM_CHANNEL_1 }, // PC6  TIM3_CH1    D1
         { 0,          0,             0,              0,              0            }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_4_channels [] =                 // TIM4
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PB6  TIM4_CH1
         { GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PB7  TIM4_CH2
         { GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PB8  TIM4_CH3         D15
         { GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PB9  TIM4_CH4         D14
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_5_channels [] =                 // TIM5
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PA0  TIM5_CH1         A0
         { GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PA1  TIM5_CH2
         { GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PA2  TIM5_CH3
         { GPIOI, GPIO_PIN_0, GPIO_AF2_TIM5, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PI0  TIM5_CH4         D10
         { 0,          0,             0,              0,              0       }  // end of table
       };

              //--------------------------------------------------------------------------------
              // Note that TIMER6 and TIMER7 have no external pins for PWM, IC, nor OC Channels
              //--------------------------------------------------------------------------------

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_8_channels [] =                  // TIM8
       { {   0L,      0L,             0,              0,              0       }, // STM32               Arduino
         { GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PC6  TIM8_CH1         D1
         { GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PC7  TIM8_CH2         D0
         { GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8, TMR_PWM_CHANNEL_3, TIM_CHANNEL_3 }, // PC8  TIM8_CH3
         { GPIOI, GPIO_PIN_2, GPIO_AF3_TIM8, TMR_PWM_CHANNEL_4, TIM_CHANNEL_4 }, // PI2  TIM8_CH4         D8
                     // Complementary pins
         { GPIOB, GPIO_PIN_13,GPIO_AF3_TIM8, TMR_PWM_CHANNEL_1_N, TIM_CHANNEL_1 }, // PB13 TIM8_CH1N
         { GPIOB, GPIO_PIN_14,GPIO_AF3_TIM8, TMR_PWM_CHANNEL_2_N, TIM_CHANNEL_2 }, // PB14 TIM8_CH2N      D12
         { GPIOB, GPIO_PIN_15,GPIO_AF3_TIM8, TMR_PWM_CHANNEL_3_N, TIM_CHANNEL_3 }, // PB15 TIM8_CH3N      D11
         { 0,          0,             0,              0,              0         }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_9_channels [] =                 // TIM9
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOA, GPIO_PIN_2, GPIO_AF3_TIM9, TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PA2 TIM9_CH1
         { GPIOA, GPIO_PIN_3, GPIO_AF3_TIM9, TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PA3 TIM9_CH2
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_10_channels [] =                // TIM10
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOF, GPIO_PIN_6,GPIO_AF3_TIM10,TMR_PWM_CHANNEL_1, TIM_CHANNEL_1  }, // PF6 TIM10_CH1         A5
     // Alternates
         { GPIOB, GPIO_PIN_8,GPIO_AF3_TIM10,TMR_PWM_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PB8  TIM10_CH1      D15
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_11_channels [] =                // TIM11
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOF, GPIO_PIN_7, GPIO_AF3_TIM11,TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PF7 TIM11_CH1         A4
     // Alternates
         { GPIOB, GPIO_PIN_9,GPIO_AF3_TIM11,TMR_PWM_CHANNEL_1_ALT1,TIM_CHANNEL_1}, // PB9  TIM11_CH1      D14
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_12_channels [] =                // TIM12
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOH, GPIO_PIN_6, GPIO_AF9_TIM12,TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PH6  TIM12_CH1        D6
         { GPIOB, GPIO_PIN_15,GPIO_AF9_TIM12,TMR_PWM_CHANNEL_2, TIM_CHANNEL_2 }, // PB15 TIM12_CH2        D11
     // Alternates
         { GPIOB, GPIO_PIN_14,GPIO_AF9_TIM12,TMR_PWM_CHANNEL_1_ALT1,TIM_CHANNEL_1 }, // PB14 TIM12_CH1    D12
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_13_channels [] =                // TIM13
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOF, GPIO_PIN_8, GPIO_AF9_TIM13,TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PF8 TIM13_CH1         A3
         { 0,          0,             0,              0,              0       }  // end of table
       };

const TMPWM_CHANNEL_BLK  _g_tmrpwm_mod_14_channels [] =                // TIM14
       { {   0L,      0L,            0,               0,              0       }, // STM32               Arduino
         { GPIOF, GPIO_PIN_9, GPIO_AF9_TIM14,TMR_PWM_CHANNEL_1, TIM_CHANNEL_1 }, // PF9 TIM14_CH1         A2
         { 0,          0,             0,              0,              0       }  // end of table
       };


                             //-----------------------------------------------------
                             // _g_tmrpwm_mod_channel_blk_lookup
                             //
                             //     Lookup table to get associated TMPWM_CHANNEL_BLK
                             //     entry for a given Timer module_id
                             //-----------------------------------------------------

const TMPWM_CHANNEL_BLK  *_g_tmrpwm_mod_channel_blk_lookup [] =
          { (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_0_channels[0],  // dummy TIM0
            (TMPWM_CHANNEL_BLK*)  &__g_tmrpwm_mod_1_channels[0],  // TIM1
            (TMPWM_CHANNEL_BLK*)  &__g_tmrpwm_mod_2_channels[0],  // TIM2
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_3_channels[0],  // TIM3
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_4_channels[0],  // TIM4
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_5_channels[0],  // TIM5
                                   0,             // TIM6 has no channels > CCR0
                                   0,             // TIM7 has no channels > CCR0
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_8_channels[0],  // TIM8
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_9_channels[0],  // TIM9
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_10_channels[0], // TIM10
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_11_channels[0], // TIM11
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_12_channels[0], // TIM12
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_13_channels[0], // TIM13
            (TMPWM_CHANNEL_BLK*)  &_g_tmrpwm_mod_14_channels[0]  // TIM14  Last
          };


//*****************************************************************************
//*****************************************************************************
//            COMMON  Utility  Routines   between   TIMERs  and  PWMs
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_timerpwm_channel_lookup
//
//           Converts an extended channel id: ALT1 / ALT2 / _N / N_ALT1 / N_ALT2
//           into the associated base channel number needed by the HAL_TIM_OC_xx
//           calls.
//
//           returned hal_channel_num is a bit encoded pattern (0x0000 - 0x000C)
//           used by the various HAL_TIM_OC_xx() and HAL_TIM_PWM_xx() calls
//
//           chan_index is a linear index 1 to 4, that is returned for any
//           operations that required straight indxed table lookups for
//           the base underlying channel.
//******************************************************************************
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index)
{
    int  hal_channel_num;

    if (chan_id < TMR_PWM_CHANNEL_1 || chan_id  > TMR_PWM_CHANNEL_3_N_ALT2)
       return (-1);                         // chan id is outside of valid range

    hal_channel_num = (int) _g_timer_base_channel_num [chan_id];

    *chan_index = (hal_channel_num >> 2) + 1;    // yields index of 1 - 4
    return (hal_channel_num);
}


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
       prescalar = 1200;
       else if (period_val > 655000)     // 0.65 M ticks
               prescalar = 120;
       else if (period_val > 65500)      // 0.065 M ticks
               prescalar = 12;
               else prescalar = 0;

    return (prescalar);
}


//******************************************************************************
//  board_timerpwm_config_gpios
//
//            Configure the GPIO for Timer PWM, Output Compare, or Input Capture
//            by setting up associated ALTERNATE FUNCTION bits for Pin Muxing.
//
//            Also, if an ALT1/ALT2 channel id is requested, scan through the
//            table to find the matching entry.
//******************************************************************************
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    TMPWM_CHANNEL_BLK   *tmrblkp;
    uint8_t             user_chan_id;

    user_chan_id = (uint8_t) chan_id;             // co-erce to correct datatype
    tmrblkp = (TMPWM_CHANNEL_BLK*) pwmblkp;
    if (tmrblkp->user_channel_id != user_chan_id)
       {     // scan across the table looking for a matching ALT channel id
         while (tmrblkp->user_channel_id != user_chan_id)
           {
             if (tmrblkp->user_channel_id == user_chan_id)
                break;                                       // we found a match
             tmrblkp++;                           // step to next entry in table
             if (tmrblkp->user_channel_id == 0)
                return (ERR_PWM_CHANNEL_NUM_NOT_SUPPORTED);  // hit end of list
           }
       }

    GPIO_InitStruct.Pin       = tmrblkp->chan_gpio_pin;

// ??? !!! need to specify INPUT if for INPUT CAPTURE  !!! ???
//                             vvvvvvvvvvvvvvvvvvv
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = tmrblkp->chan_alt_func;  // set Pin Mux Alt Function code
    HAL_GPIO_Init (tmrblkp->chan_gpio_port, &GPIO_InitStruct);

// ??? in future,  do  pchanConfig. stuff as well ???

    return (0);
}


//******************************************************************************
//  board_timerpwm_enable_clock
//
//           Ensures that the specified Timer's clock is turned on.
//******************************************************************************
int  board_timerpwm_enable_clock (int module_id)
{
      //--------------------------------------------------------
      // Turn on clock for the associated TIMx module.
      //--------------------------------------------------------
    switch (module_id)
     { case 1:                           // TIM1
             __TIM1_CLK_ENABLE();
             break;
        case 2:                          // TIM2
             __TIM2_CLK_ENABLE();
             break;
        case 3:                          // TIM3
             __TIM3_CLK_ENABLE();
             break;
        case 4:                          // TIM4
             __TIM4_CLK_ENABLE();
             break;
        case 5:                          // TIM5
             __TIM5_CLK_ENABLE();
             break;
        case 6:                          // TIM6
             __TIM6_CLK_ENABLE();
             break;
        case 7:                          // TIM7
             __TIM7_CLK_ENABLE();
             break;
        case 8:                          // TIM8
             __TIM8_CLK_ENABLE();
             break;
        case 9:                          // TIM9
             __TIM9_CLK_ENABLE();
             break;
        case 10:                         // TIM10
             __TIM10_CLK_ENABLE();
             break;
        case 11:                         // TIM11
             __TIM11_CLK_ENABLE();
             break;
        case 12:                         // TIM12
             __TIM12_CLK_ENABLE();
             break;
        case 13:                         // TIM13
             __TIM13_CLK_ENABLE();
             break;
        case 14:                         // TIM14
             __TIM14_CLK_ENABLE();
             break;
        default:
             return (ERR_TIMER_NUM_NOT_SUPPORTED);
     }
    return (0);          // denote everything worked OK
}



//*****************************************************************************
//       This section provides basic timer setup and trigger mode support
//
//*****************************************************************************

//*****************************************************************************
//  board_timerpwm_init
//
//         Initializes a Timer/PWM with correct mode, and initial period.
//
//  NOTE:
//         HAL code analysis shows that there is almost no difference between
//         the HAL_TIM_OC_Init(), HAL_TIM_PWM_Init, and HAL_TIM_Base_Init().
//         The only difference is their invocations of differnt MspInit()
//         callbacks, which we do not use.
//
//         So except where there is a substantive code difference (mainly on
//         Config Channels), we utilize the HAL_TIM_OC_xx or HAL_TIM_Base_xx
//         to avoid importing redundant amounts of bloatware. Hence we avoid
//         using the (mostly redundant) HAL_TIM_PWM_xx calls except on Config Channels.
//*****************************************************************************
int  board_timerpwm_init (int module_id, int counter_type, long period_value,
                          int timer_clock_source, int flags)
{
    TIM_TypeDef         *timbase;
    TIM_HandleTypeDef   *hdltimer;
    uint32_t            counter_mode;
    long                prescalar_val;
    int                 rc;

    if (module_id < 0 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    if (counter_type == TIMER_PERIODIC_COUNT_UP)
       counter_mode = TIM_COUNTERMODE_UP;
       else if (counter_type == TIMER_PERIODIC_COUNT_DOWN)
               counter_mode = TIM_COUNTERMODE_DOWN;
       else if (counter_type == TIMER_PERIODIC_COUNT_UPDOWN)
               counter_mode = TIM_COUNTERMODE_CENTERALIGNED3;
       else if (counter_type == TIMER_ONE_SHOT_COUNT_UP)
               counter_mode = TIM_COUNTERMODE_UP;
       else if (counter_type == TIMER_ONE_SHOT_COUNT_DOWN)
               counter_mode = TIM_COUNTERMODE_DOWN;
       else if (counter_type == TIMER_INPUT_CAPTURE)
               ;
       else return (ERR_TIMER_INVALID_COUNTER_TYPE);

      //--------------------------------------------------------
      // Turn on clock for the associated TIMx module.
      //--------------------------------------------------------
    board_timerpwm_enable_clock (module_id);    // ensure Timer clock is turn on

    prescalar_val = 0;
#if (SCREWING_UP_DAC_TIMERS_BIG_TIME) // 06/26/15 - makes length of Sine/Triangle hugely long (seconds duration for 1 cycle)
#endif
    if ((flags & TIMER_DISABLE_DAC_PRESCALING) == 0)
       {
              //---------------------------------------------------------------------
              // Compute any needed prescaler value to fit 16-bit TIMx counter clock.
              // Also adjust period_value to reflect effect of pre-scalar.
              //---------------------------------------------------------------------
         prescalar_val = board_timerpwm_compute_prescalar (period_value);
         if (prescalar_val > 0)
            period_value = (period_value / prescalar_val);
         _g_tmpwm_prescalars[module_id] = prescalar_val;  // save for duty cycle calcs
       }
      else _g_tmpwm_prescalars[module_id] = 0;            // no pre-scaling

              //---------------------------------------------------------------------
              // Call HAL logic to configure the TIM module for usage.
              //---------------------------------------------------------------------
    if (counter_type >= TIMER_PERIODIC_COUNT_UP && counter_type <= TIMER_ONE_SHOT_COUNT_DOWN)
       {    // Configure the TIMx peripheral for Timer Output
         hdltimer->Instance               = timbase;
         hdltimer->Init.Period            = (period_value - 1); // set # ticks needed for that frequency
         hdltimer->Init.Prescaler         = prescalar_val;
         hdltimer->Init.ClockDivision     = 0;
         hdltimer->Init.CounterMode       = counter_mode;
         hdltimer->Init.RepetitionCounter = 0;
         rc = HAL_TIM_Base_Init (hdltimer);      //   <---- THIS HAMMERS THE TABLE !!!  ==> TIMBASE VALUE IS FUED ???  07/15/15
       }
      else
       {    // Configure the TIMx peripheral for Timer Input
         hdltimer->Instance               = timbase;
//       rc = HAL_TIM_IC_Init (hdltimer);    // count mode = Input capture

       }
    if (rc != HAL_OK)
       {    // got a Base Timer/PWM Initialization Error - bail
         return (ERR_PWM_MODULE_INITIALIZE_FAILED);
       }

        //-----------------------------------------------------------------------
        // denote module has been initialized, and if Complementary PWM is used
        //-----------------------------------------------------------------------
    if (flags & PWM_COMPLEMENTARY_OUTPUTS)
            // user wants to run a pair of channels in complementary mode
       _g_tmpwm_module_status [module_id] = TMR_PWM_COMPLEMENT_INIT;
       else _g_tmpwm_module_status [module_id] = TMR_PWM_NORMAL_INIT; // no complementAry mode

    return (0);                              // denote completed OK
}

                       // The rest of the Physical Timers logic

//*****************************************************************************
//  board_timerpwm_check_completed
//
//        Checks to see if the timer has expired / completed it's cycle.
//        Optionally, resets the completion flag if denoted in reset_flags parm.
//
//        This is frowned upon (callbacks should be used instead), but some
//        people still want to occassionally do polling on this stuff.
//*****************************************************************************
int  board_timerpwm_check_completed (int module_id, int mask_flags, int reset_flags)
{
    TIM_HandleTypeDef   *hdltimer;
    TIM_TypeDef         *timbase;

    if (module_id < 0 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //-----------------------------------------------------------------------
       // Check Timer's UPDATE flag status, to see if a timer rollover occurred
       //-----------------------------------------------------------------------
    if (timbase->SR & TIM_SR_UIF)
       {      // yes, Timer has popped/rolled over
         if (reset_flags == 1)
            timbase->SR = ~(TIM_SR_UIF);
         return (1);                    // tell caller it popped
       }

//        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
//#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)
//             ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
// #define TIM_IT_UPDATE     ((uint32_t)0x00000001)  =  TIM_SR_UIF

    return (0);                         // Timer has not popped yet
}


//*****************************************************************************
//  board_timerpwm_config_channel
//
//         Configure a specific channel (1-4) on a specific PWM module (TIM1/2/..)
//
//         For TIM1, allow complmentary mode usage.
//*****************************************************************************

int  board_timerpwm_config_channel (int module_id, int chan_id,
                                    long initial_duty, int mode, int flags)
{
    TIM_OC_InitTypeDef  pchanConfig;      // Timer Output CCR Config
    TIM_HandleTypeDef   *hdltimer;
    TMPWM_CHANNEL_BLK   *pwmblkp;
    int                 rc;
    int                 chan_index;
    uint32_t            hal_channel_num;
    unsigned char       chan_mask;
    unsigned char       rupt_mask;

    if (module_id < 1  ||  module_id > TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated TIM module handle
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    if (_g_tmpwm_module_status [module_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

//  if ((_g_tmpwm_module_status[modgen_id] & TMR_PWM_NORMAL_INIT) == 0)
//     return (ERR_PWM_MODULE_IN_COMPLEMENTARY); // need to use pwm_config_channel_pair

    if (mode < 0 || mode > TIMER_MODE_PWM)
       return (ERR_PWM_INVALID_TIMER_MODE);

       //-------------------------------------------
       // do any needed pre-scaling on duty_cycle
       //-------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0)
       { initial_duty = (initial_duty / _g_tmpwm_prescalars[module_id]);
       }

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

    chan_mask = _g_pwm_chan_mask [chan_index];   // get associated CCR mask bits

       //------------------------------------------------------------------
       //                Check if running as TIMER ONLY
       //
       // Note: user call elect to run the CCR in Timer mode only, and
       //       not sending output out the GPIOs, e.g. Trigger for ADC/DAC.
       //------------------------------------------------------------------
    if (flags & TIMER_CCR_TIMER_ONLY)
       {
         memset (&pchanConfig, '\0',sizeof(pchanConfig));  // ensure cleared out

                // Set the OC mode to "Frozen" so that it only acts as a timer
         pchanConfig.OCMode      = TIM_OCMODE_TIMING;        // Output is frozen
         pchanConfig.OCPolarity  = TIM_OCPOLARITY_LOW;
         pchanConfig.OCIdleState = TIM_OCIDLESTATE_SET;
         pchanConfig.OCFastMode  = TIM_OCFAST_DISABLE;
         pchanConfig.Pulse       = initial_duty;       // Set duty cycle for CCR
         rc = HAL_TIM_OC_ConfigChannel (hdltimer, &pchanConfig,
                                      hal_channel_num); // config for Timer only
                // denote that it has been configured, and should be enabled.
         _g_tmpwm_channels_config [module_id] |= chan_mask;

         return (0);               // denote completed OK
       }

       //------------------------------------------------------------------
       //        Setup associated GPIO pin into Timer/PWM mode.
       //
       // Configure Timer/PWM Channel's GPIO pin into Alternate function,
       // push-pull, 100MHz speed.
       //
       // Note: user call elect to run the CCR in Timer mode only, and
       //       not sending output out the GPIOs, e.g. Trigger for ADC/DAC.
       //------------------------------------------------------------------
    pwmblkp = (TMPWM_CHANNEL_BLK*) _g_tmrpwm_mod_channel_blk_lookup [module_id];

    rc = board_timerpwm_config_gpios (pwmblkp, chan_id); // configure assoc GPIO
    if (rc < 0)
       return (rc);     // must be "not supported channel number" on this module

       //------------------------------------------------
       // configure the associated Timer/PWM channel CCR
       //------------------------------------------------
    memset (&pchanConfig, '\0', sizeof(pchanConfig));   // ensure is cleared out

    if (mode == TIMER_MODE_OC_PASSTHRU)
       pchanConfig.OCMode = TIM_OCMODE_TIMING;           // Output is frozen to whatever POLARITY is set to
       else if (mode == TIMER_MODE_OC_SET)
               pchanConfig.OCMode = TIM_OCMODE_ACTIVE;   // set pin Active when CCR reached
       else if (mode == TIMER_MODE_OC_RESET)
               pchanConfig.OCMode = TIM_OCMODE_INACTIVE; // set pin Inactive when CCR hit
       else if (mode == TIMER_MODE_OC_TOGGLE)
               pchanConfig.OCMode = TIM_OCMODE_TOGGLE;   // TOGGLE pin when CCR hit
       else if (mode == TIMER_MODE_PWM)
               pchanConfig.OCMode = TIM_OCMODE_PWM1;     // PWM_1 mode
       else if (mode == TIMER_MODE_PWM_INV)
               pchanConfig.OCMode = TIM_OCMODE_PWM2;     // PWM_2 mode
       else return (ERR_PWM_INVALID_TIMER_MODE);

    if (flags & TIMER_PIN_POLARITY_LOW)
       pchanConfig.OCPolarity = TIM_OCPOLARITY_LOW;      // Pin is LOW when Active (inverted)
       else pchanConfig.OCPolarity = TIM_OCPOLARITY_HIGH;// Pin is HIGH when Active

    pchanConfig.OCIdleState = TIM_OCIDLESTATE_SET;       // flags option in future
    pchanConfig.OCFastMode  = TIM_OCFAST_DISABLE;
    pchanConfig.Pulse       = initial_duty; //Set initial duty cycle for channel

    if (mode >= TIMER_MODE_PWM)
       {     // denote that this channel is running as a PWM on this module.
             // turn associated mask bit on for this channel in PWM status flags.
         _g_tmpwm_channels_pwm [module_id] |= chan_mask;
       }

// ??? where how is this flag ever set ??? should it be derived from GPIO pin lookup ? !!!  PROBABLY  WVD
    if (flags & PWM_COMPLEMENTARY_OUTPUTS)
       {    // user wants to run a pair of channels in complementary mode.
            // turn on the extra parms needed for Complementary support.
            // "N" alternate channel settings
// *** NEEDS MORE WORK - ALSO NEED TO ENABLE ASSOCIATE GPIO PINS  ??? !!!  05/19/15
         if (module_id != 1 || chan_index > 3)
            return (ERR_PWM_MODULE_NO_COMPLEMENTARY); // only TIM1 supports complementary
         pchanConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;   // Flags option in future
         pchanConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET; // Flags option in future
         _g_tmpwm_channels_pwm [module_id] |= _g_pwm_chan_mask [chan_index+4];  // turn on comple N bits
       }

    if ((_g_tmpwm_channels_pwm [module_id] & chan_mask) != 0)
       rc = HAL_TIM_PWM_ConfigChannel (hdltimer, &pchanConfig,
                                       hal_channel_num); // config for PWM mode output
       else rc = HAL_TIM_OC_ConfigChannel (hdltimer, &pchanConfig,
                                       hal_channel_num); // config for OC mode output
    if (rc != HAL_OK)
       {
         return (ERR_PWM_CHANNEL_CONFIG_FAILED);  // Config Channel Error - bail
       }

       //--------------------------------------------------------------
       // turn on any needed interrupt flags for this CCR
       //--------------------------------------------------------------
    if (flags & TIMER_ENABLE_CCR_INTERRUPTS)
       { rupt_mask = _g_tmpwm_CCR_rupt_flags [chan_index];
             // save what interrupts the User App wants to see
         _g_timer_app_enabled_interrupts [module_id] |= rupt_mask;
       }

       //----------------------------------------------------------------------
       // for this Timer/PWM module, set the bit corresponding for this channel
       // CCR to denote that it has been configured, and should be enabled.
       //----------------------------------------------------------------------
    _g_tmpwm_channels_config [module_id] |= chan_mask;

    return (0);               // denote successful configuration
}


//*****************************************************************************
//  board_timerpwm_config_trigger_mode                         TRIGGER
//
//         Sets up a Timer to provide Trigger Output (CCR or Rollover based
//         TRGO triggers), which is used to trigger either ADCs or DACs.
//
//         As per the STM32 Tech Ref, the _ADC Timer should already be enabled_
//         _before calling this routine_.
//
//         The trigger type must match what was specified on the adc_init()
//         or dac_init() call, otherwise an error is signalled.
//
// flags parm can have TIMER_TRIGGER_MODE set, indicating to ensure that the
//       CCR OUTMOD is set to some kind of toggle (MOD_3 / MOD_6/ MOD_7) state.
//*****************************************************************************
int  board_timerpwm_config_trigger_mode (int module_id, int trigger_type,
                                         int flags)
{
    TIM_MasterConfigTypeDef   MasterSlaveConfig;
    TIM_HandleTypeDef         *hdltimer;
    TIM_TypeDef              *timbase;
    int                       ccr_bits;
    int                       channel_mask;

    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

        //------------------------------------------------------------------
        //                Handle any ADC or DAC Triggering details
        //
        // Need to handle the case where user issued adc_init() _after_
        // timer_ADC_Trigger_Start(), in which case we need to also
        // turn on the associated Timer's MMS flags, to enable it to broadcast
        // trigger signals to the ADC.
        //
        // If GPIO EXTI triggering was used instead, need to ensure it is setup
        //----------------------------------------------------------------------
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (trigger_type != 0)
       { if (trigger_type  < ADC_TRIGGER_USER_APP)
            {    // Process ADC based triggers
              if (_g_trigger_atmr_mmsmask == 0)
                 return (ERR_TIMER_ADC_NOT_INITIALIZED);
              if (_g_trigger_auser_api_id != trigger_type)
                 return (ERR_TIMER_TRIGGER_MISMATCH);
                    //----------------------------------------------------------
                    // See if this is a CCR type trigger, and if so, see if the
                    // associated Timer's CCR channel was configured by the user.
                    // If not, we will auto-configure it with a default set of
                    // values. If the caller later calls timer_Enable_CCR_Output()
                    // it will just overlay our default CCR values.
                    //----------------------------------------------------------
/* ??? BUG 07/15/15  Wants to config a CCR on Timer2 Upd only*/   ccr_bits     = _g_trigger_auser_api_id;
              channel_mask = _g_pwm_chan_mask [ccr_bits];  // get assoc config bit mask
              if (ccr_bits != 0)
                 if ((_g_tmpwm_channels_config [module_id] & channel_mask) == 0)
                    {    // Has not been configured. Set it up with default
                         // values. Set default CCR value = Timer's period (ARR)
                      board_timerpwm_config_channel (module_id, ccr_bits,
                                                     timbase->ARR,
                                                     TIMER_MODE_OC_SET_RESET,
                                                     TIMER_CCR_TIMER_ONLY);
                    }
                  // Then setup Timer to issue a TRG0 event whenever the Timer
                  // performs the requested Update or CCR event
              MasterSlaveConfig.MasterOutputTrigger = _g_trigger_atmr_mmsmask;
              MasterSlaveConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
              HAL_TIMEx_MasterConfigSynchronization (hdltimer, &MasterSlaveConfig);
            }
           else if (trigger_type == ADC_TRIGGER_GPIO_PIN)
                   {       // process ADC trigger caused by GPIO
// ??? TBD  WVD
                     return (0);     // no MMS update needed for EXTI GPIO
                   }
           else if (trigger_type == DAC_TRIGGER_EXTI_GPIO)
                   {       // process DAC trigger caused by GPIO
// ??? TBD  WVD
                     return (0);     // no MMS update needed for EXTI GPIO
                   }
           else if (trigger_type >= DAC_TRIGGER_TIMER_2 && trigger_type < DAC_TRIGGER_EXTI_GPIO)
                   {
                     if (_g_trigger_dtmr_mmsmask == 0)
                        return (ERR_TIMER_DAC_NOT_INITIALIZED);
                     if (_g_trigger_duser_api_id != trigger_type)
                        return (ERR_TIMER_TRIGGER_MISMATCH);
                       //-------------------------------------------------------
                       // See if this is a CCR type trigger, and if so, see if the
                       // associated Timer's CCR channel was configured by the user.
                       // If not, we will auto-configure it with a default set of
                       // values. If the caller later calls timer_Enable_CCR_Output()
                       // it will just overlay our default CCR values.
                       //-------------------------------------------------------
                     ccr_bits     = _g_trigger_duser_api_id;
                     channel_mask = _g_pwm_chan_mask [ccr_bits];  // get assoc config bit mask
                      if (ccr_bits != 0)
                        if ((_g_tmpwm_channels_config [module_id] & channel_mask) == 0)
                          {  // Has not been configured. Set it up with default
                             // values. Set default CCR value = Timer's period (ARR)
                            board_timerpwm_config_channel (module_id, ccr_bits,
                                                           timbase->ARR,
                                                           TIMER_MODE_OC_SET_RESET,
                                                           TIMER_CCR_TIMER_ONLY);
                          }
                        // Then setup Timer to issue a TRG0 event whenever the Timer
                        // performs the requested Update or CCR event
                     MasterSlaveConfig.MasterOutputTrigger = _g_trigger_dtmr_mmsmask;
                     MasterSlaveConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
                     HAL_TIMEx_MasterConfigSynchronization (hdltimer, &MasterSlaveConfig);
                   }
           else return (ERR_TIMER_INVALID_TRIGGER_TYPE);
       }

    return (0);              // denote completed ok
}


//*****************************************************************************
//  board_timerpwm_disable
//
//            Turns off a timer/pwm, and disables any interrupts for it.
//            (e.g. in preparation for Stopping or Reconfiguring a motor, ...)
//*****************************************************************************
int  board_timerpwm_disable (int module_id, int flags)
{
    TIM_HandleTypeDef   *hdltimer;

    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

      // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

// ??? TBD   stop every PWM channel that is enabled on this module    ??? !!!

    if (_g_tmpwm_module_status [module_id] & TMR_PWM_INTERRUPTS_USED)
       HAL_TIM_Base_Stop_IT (hdltimer);    // Disable the timer and interrupts
       else HAL_TIM_Base_Stop (hdltimer);  // Disable the Timer


// ??? !!! Disable before/after OC Timer_Disable ??? !!!  FUTURE
// ???     associated Compare/Capture OC elements must also be disabled !!!
//  HAL_TIM_OC_Stop_IT (hdltimer, Channel);
// ??? must associated Compare/Capture OC elements also be stopped ?
//  HAL_TIM_IC_Stop_IT (hdltimer, Channel);  // input capture

    return (0);                         // denote completed OK
}



//*****************************************************************************
//  board_timerpwm_enable
//
//         Enables and Starts the timer/PWM.
//
//         Sets up any desired Timer interrupt flags, then enables the Timer/PWM.
//         Saves optional callback function address to call when interrupt occurs.
//*****************************************************************************

int  board_timerpwm_enable (int module_id, int interrupt_flags)
{
    TIM_HandleTypeDef  *hdltimer;
    int                irqn;
    int                rc;
    int                app_ccr_rupts;
    unsigned char      channels_config;
    unsigned char      pwm_config;

    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //-----------------------------------------------------------------------
       // If user requested a Rollover interrupt,  OR  setup interrupts on a CCR
       // then enable the necessary NVIC entry
       //-----------------------------------------------------------------------
                   // find and set associated NVIC IRQ value
    irqn = _g_TMPWM_IRQ_table [module_id];
    if (interrupt_flags != 0 || _g_timer_app_enabled_interrupts[module_id])
       {           // denote this module is running with Interrupts
         _g_tmpwm_module_status [module_id] |= TMR_PWM_INTERRUPTS_USED;
         if (interrupt_flags & TIMER_ENABLE_ROLLOVER_INTERRUPTS)
            {      //------------------------------------------------------
                   // set assoc ROLLOVER/Update flag for the overall Timer
                   //------------------------------------------------------
              _g_timer_app_enabled_interrupts[module_id] |= TIM_SR_UIF;
              if (module_id == 1)
                 {          // TIM1 has a separate IRQn for Rollovers
                   NVIC_EnableIRQ (TIM1_UP_TIM10_IRQn);           // Enable NVIC
                   NVIC_SetPriority (TIM1_UP_TIM10_IRQn, 1);
                 }
              else if (module_id == 8)
                      {     // TIM8 has a separate IRQn for Rollovers
                        NVIC_EnableIRQ (TIM8_UP_TIM13_IRQn);      // Enable NVIC
                        NVIC_SetPriority (TIM8_UP_TIM13_IRQn, 1);
                      }
              else {        // handle all the other timers Rollovers
                            // find and set associated NVIC IRQ value
                     NVIC_EnableIRQ (irqn);                       // Enable NVIC
                     NVIC_SetPriority (irqn, 1);
                   }

            }
           else {         // handle all the other timers CCRs and TIM1/TIM8 CCRs
                          // find and set associated NVIC IRQ value
                     NVIC_EnableIRQ (irqn);                       // Enable NVIC
                     NVIC_SetPriority (irqn, 1);
                }
                   //---------------------------------------------------
                   //  Enable the Timer/PWM to start, _with_ Interrupts
                   //---------------------------------------------------
          HAL_TIM_Base_Start_IT (hdltimer);
        }
       else {
                   //------------------------------------------------
                   // Enable Timer/PWM to start,  _no_ interrupts
                   //------------------------------------------------
              HAL_TIM_Base_Start (hdltimer);
            }

       //------------------------------------------------------------------------
       // Startup each Timer/PWM channel that has been configured for this module
       //
       // After the main timer has been started, walk through and enable
       // each CCR requested by the user app. This will turn on the Output at
       // the GPIO pin, and turn on the associated CCR interrupt if requested.
       // Note: there is no substanitive difference between HAL_TIM_PWM_Start()
       //       and HAL_TIM_OC_Start()
       //------------------------------------------------------------------------
    channels_config = _g_tmpwm_channels_config [module_id];        // get local copy of CCR configed flags
    pwm_config      = _g_tmpwm_channels_pwm [module_id];
    app_ccr_rupts   = _g_timer_app_enabled_interrupts [module_id]; // get local copy of CCR interrupts
    if (channels_config & TMR_CCR1_CONFIGURED)
       {     // start channel 1, and optionally, channel 1N
         if (app_ccr_rupts & TIM_SR_CC1IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC1);      // Enable CCR1 rupt
            }
         if (pwm_config & TMR_CCR1_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_1);  // start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_1);
         if (channels_config  &  TMR_CCR1N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in (PWM) complementary mode - turn on channel 1N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_1);
            }
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);         // start failed - bail
       }

    if (channels_config  &  TMR_CCR2_CONFIGURED)
       {     // start channel 2, and optionally, channel 2N
         if (app_ccr_rupts & TIM_SR_CC2IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC2);  // Enable CCR2 rupt
            }
         if (pwm_config & TMR_CCR2_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_2);  // Start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_2);
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
         if (channels_config  &  TMR_CCR2N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in complementary mode - turn on channel 2N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_2);
            }
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
       }

    if (channels_config  &  TMR_CCR3_CONFIGURED)
       {     // start channel 3, and optionally, channel 3N
         if (app_ccr_rupts & TIM_SR_CC3IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC3);  // Enable CCR3 rupt
            }
         if (pwm_config & TMR_CCR3_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_3);  // Start as PWM
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_3);
         if (channels_config  &  TMR_CCR3N_CONFIGURED  &&  rc == HAL_OK)
            {    // operating in complementary mode - turn on channel 3N too
              rc = HAL_TIMEx_PWMN_Start (hdltimer, TIM_CHANNEL_3);
            }
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail
       }

    if (channels_config  &  TMR_CCR4_CONFIGURED)
       {     // start channel 4
         rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_4);
         if (app_ccr_rupts & TIM_SR_CC4IF)
            {    // User wants to see interrupts on this CCR
              __HAL_TIM_ENABLE_IT (hdltimer, TIM_IT_CC4);  // Enable CCR4 rupt
            }
         if (pwm_config & TMR_CCR4_CONFIGURED)
            rc = HAL_TIM_PWM_Start (hdltimer, TIM_CHANNEL_4);
            else rc = HAL_TIM_OC_Start (hdltimer, TIM_CHANNEL_4);
         if (rc != HAL_OK)
            return (ERR_PWM_CHANNEL_START_FAILED);      // start failed - bail

         // Channel 4 does _NOT_ have a complementary output (no channel 4N) !
       }

    return (0);                         // denote completed OK
}

//#endif                                  //  USES_PWM  ||  USES_TIMER


//#if defined(USES_TIMER) || defined(USES_PWM)

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
//*****************************************************************************
int  board_timerpwm_enable_CCR_output (int module_id, int chan_id, long initial_duty,
                                       int action_flags, int interrupt_flags)
{
    int         mode;
    int         rc;
    int         flags;

       // tweak parms to match board_timerpwm_config_channel() format
    mode  = action_flags & 0x0F;
    flags = interrupt_flags & 0xF0;

       // all heavy lifting is done by board_timerpwm_config_channel()
    rc = board_timerpwm_config_channel (module_id, chan_id,
                                        initial_duty, mode, flags);

    return (rc);                      // denote worked successfully
}



//*****************************************************************************
//  board_timerpwm_get_current_value
//
//         Gets the current count value (CNT) in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_current_value (int module_id)
{
    TIM_TypeDef      *timbase;
    long             curr_count;

    if (module_id < 1 || module_id > TMRPWM_NUM_MODULES)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    curr_count = timbase->CNT;      // get currnet COUNT value

       //---------------------------------------------------------------
       // re-adjust COUNT value based on pre-scalar to get user's view
       //---------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_count = (curr_count * _g_tmpwm_prescalars[module_id]);

    return (timbase->CNT);          // pass back adjusted current counter value
}


//*****************************************************************************
//  board_timerpwm_get_CCR_capture_value
//
//         Gets the recorded input capture value in the Timer/PWM.
//*****************************************************************************
long  board_timerpwm_get_CCR_capture_value (int module_id, int CCR_channel_id)
{
    TIM_HandleTypeDef   *hdltimer;
    long                cap_value;

    if (module_id < 1 || module_id > TMRPWM_NUM_MODULES)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       // based on module_id, setup pointer to associated timer objects
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    cap_value = HAL_TIM_ReadCapturedValue (hdltimer, CCR_channel_id);

// ??? !!! must I adjust the value based on pre-scalar used ??? !!!

    return (cap_value);             // passback capture value
}


//*****************************************************************************
//  board_timerpwm_get_duty_cycle
//
//        Get the Timer/PWM's current duty cycle.      Used for monitoring, etc
//*****************************************************************************

long  board_timerpwm_get_duty_cycle (int module_id, int chan_id)
{
    TIM_HandleTypeDef   *hdltimer;
    long                curr_duty;
    int                 hal_channel_num;
    int                 chan_index;

    if (module_id < 1  ||  module_id > TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

            // get ptr to associated TIM module handle
    hdltimer = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (hdltimer == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

    switch (chan_index)
      { case 1:
              curr_duty = hdltimer->Instance->CCR1;
              break;
        case 2:
              curr_duty = hdltimer->Instance->CCR2;
              break;
        case 3:
              curr_duty = hdltimer->Instance->CCR3;
              break;
        case 4:
              curr_duty = hdltimer->Instance->CCR4;
              break;
        default:
                // this must be one of the N channels -
                // it does _NOT_ have its own CCRxN.  It relies on CCR1/1/3/4
              return (ERR_PWM_CHANNEL_COMPLE_CHAN_NO_CCR);
      }

       //-------------------------------------------------------------
       // re-adjust duty value based on pre-scalar to get user's view
       //-------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_duty = (curr_duty * _g_tmpwm_prescalars[module_id]);

    return (curr_duty);
}


//*****************************************************************************
//  board_timerpwm_get_period
//
//         Gets the period value (ARR) being used by the Timer/PWM.
//         Used for monitoring, etc
//*****************************************************************************
long  board_timerpwm_get_period (int module_id)
{
    TIM_TypeDef      *timbase;
    long             curr_period;

    if (module_id < 1 || module_id > TMRPWM_NUM_MODULES)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    curr_period = timbase->ARR;     // get currnet PERIOD value

       //---------------------------------------------------------------
       // re-adjust period value based on pre-scalar to get user's view
       //---------------------------------------------------------------
    if (_g_tmpwm_prescalars[module_id] > 0)
       curr_period = (curr_period * _g_tmpwm_prescalars[module_id]);

    return (curr_period);          // pass back adjusted current period value
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

    if (module_id < 1  ||  module_id >= TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > TMRPWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);


#if (MSP432_CODE)      //  WVD  ??? !!! NEED TO CREATE EQUIV FOR STM32  ??? !!!

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
#endif

    return (0);                      // denote worked successfully
}


//*****************************************************************************
//  board_timerpwm_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when any Timer rollover or CCR interrupts occur.
//*****************************************************************************
int  board_timerpwm_set_callback (int module_id, TMR_CB_EVENT_HANDLER callback_function,
                                  void *callback_parm)
{
    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts (CCRs or Rollover)
       //-------------------------------------------------------------------
    _g_ptimer_callback [module_id]      = callback_function;
    _g_ptimer_callback_parm [module_id] = callback_parm;

    return (0);                 // denote completed OK
}



//******************************************************************************
//  board_timerpwm_set_duty_cycle
//
//        Set the Timer/PWM's duty cycle, aka
//        sets a CCR (compare count register) on the specified timer.
//
//        This is used to set different "duty cycles" for wave form values
//        being generated by a Timer, e.g. CCR1 might generate a 25 % duty
//        cycle on channel 1 pin, 33 % on channel 2 pin, and 50 % on chan 3 pin.
//
//        The flags parms is used to specify if a CCR interrupt should be
//        generated when the CCR compare value is hit by the Timer/PWM.
//******************************************************************************
int  board_timerpwm_set_duty_cycle (int module_id, int chan_id,
                                    long ccr_duty_cycle, int flags)
{
    TIM_TypeDef         *timbase;
    int                 hal_channel_num;
    int                 chan_index;

    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

            // get ptr to associated TIM register
    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed bad chan_id value

       //-------------------------------------------
       // do any needed pre-scaling on duty_cycle
       //-------------------------------------------
    if ( _g_tmpwm_prescalars[module_id] > 0)
       { ccr_duty_cycle = (ccr_duty_cycle / _g_tmpwm_prescalars[module_id]);
       }

    switch (chan_index)
      {
        case 1:
               timbase->CCR1 = ccr_duty_cycle;
               break;
        case 2:
               timbase->CCR2 = ccr_duty_cycle;
               break;
        case 3:
               timbase->CCR3 = ccr_duty_cycle;
               break;
        case 4:
               timbase->CCR4 = ccr_duty_cycle;
               break;
      }

    return (0);                         // denote completed OK
}


//*****************************************************************************
//  board_timerpwm_set_period
//
//         Sets the Timer/PWM to use a new period value
//*****************************************************************************
int  board_timerpwm_set_period (int module_id, long new_period_value, int flags)
{
    TIM_TypeDef         *timbase;
    uint32_t            period_val;
    uint32_t            prescalar_val;

    if (module_id < 1 || module_id > MAX_TIMER)
       return (ERR_TIMER_NUM_OUT_OF_RANGE);

    timbase  = (TIM_TypeDef*) _g_timer_module_base [module_id];
    if (timbase == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);

    prescalar_val = 0;
    period_val    = new_period_value;

    if ((flags & TIMER_DISABLE_DAC_PRESCALING) == 0)
       {       //-------------------------------------------
               //  do any needed pre-scaling on new period
               //-------------------------------------------
               // Compute any needed prescaler value to fit 16-bit TIMx counter clock
         prescalar_val = board_timerpwm_compute_prescalar (period_val);
         if (prescalar_val > 0)
            period_val = (period_val / prescalar_val);
         _g_tmpwm_prescalars[module_id] = prescalar_val; // save for duty cycle calcs
       }
      else _g_tmpwm_prescalars[module_id] = 0;  // no pre-scalars used

    timbase->ARR = period_val;

    timbase->PSC = prescalar_val;

    return (0);                         // denote completed OK
}


//*****************************************************************************
//  board_timerpwm_set_dead_time
//
//         Set the PWM's dead time between complementary PWM
//         arrangements.  Use to avoid "shoot through" on the
//         output FETs when working with Power or Motor H-bridges.
//*****************************************************************************

int  board_timerpwm_set_dead_time (int module_id, int rising_edge, int falling_edge)
{
    TIM_HandleTypeDef   *tmrpwm_mhdl;
    int                 rc;
    uint32_t            scaleduty;

    if (module_id != 1)
       return (ERR_PWM_MODULE_NO_DEADTIME);   // Only TIM1 supports Deadtime

            // get ptr to associated TIM module handle
    tmrpwm_mhdl = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];

    tmrpwm_mhdl->Instance->BDTR &= ~(0xFF00); // clear out any previous DT value
    tmrpwm_mhdl->Instance->BDTR |= (rising_edge & 0x00FF);   // set 8 bit deadtime

       // trailing edge is ignored, because STM32 F7 TIM1 only supports a
       // single deadtime value, that is used on both rising and falling edges.

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_timerpwm_set_phase
//
//         Set the PWM's phase between complementary PWM
//         arrangements.
//*****************************************************************************

int  board_timerpwm_set_phase (int module_id, int chan_num, long phase_offset)
{
   return (ERR_PWM_MODULE_NO_PHASE); // STM32 F7, F4, F1, F0, L1, L0 do NOT support
                                     // phase shifting. Only STM32 F3 modules do

                    // ??? WVD RESEARCH  doesn't TIM1/TIM8 support a poor man's phase shift ?
                    //                   e.g. F3_03 without HRTIM module ?

}



//*****************************************************************************
//  board_timerpwm_set_channel_output
//
//         Set polarity of Timer/PWM channel output (start high or low)
//*****************************************************************************
int  board_timerpwm_set_channel_output (int module_id, int chan_id,
                                        int output_mode, int flags)
{
    TIM_HandleTypeDef   *tmrpwm_mhdl;
    int                 rc;
    int                 hal_channel_num;
    int                 chan_index;

    if (module_id < 1  ||  module_id > TMRPWM_NUM_MODULES)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

       //----------------------------------------------------------------
       // Convert extended channel numbers: ALT1 / ALT2 / _N / _N_ALT to
       // the underlying base channel number needed for HAL_TIM_PWM calls
       //----------------------------------------------------------------
    hal_channel_num = board_timerpwm_channel_lookup (chan_id, &chan_index);
    if (hal_channel_num == -1)
       return (ERR_TIMER_CHANNEL_NUM_OUT_OF_RANGE);  // passed a bad chan_id value

            // get ptr to associated TIM module handle
    tmrpwm_mhdl = (TIM_HandleTypeDef*) _g_timer_module_handle [module_id];
    if (tmrpwm_mhdl == 0L)
       return (ERR_TIMER_NUM_NOT_SUPPORTED);


//   ???   TBD   ???     WVD   ???



    return (0);                // denote completed successfully
}



//******************************************************************************
//                           TIMER / PWM   ISRs
//******************************************************************************

void  TIM_Common_IRQHandler(TIM_TypeDef *TIMbase, int tim_index);
void  TIM1_UP_TIM10_IRQHandler(void);  // -- CAUTION SHARED BETWEEN TIM1 and TIM10
void  TIM1_CC_IRQHandler(void);
void  TIM2_IRQHandler(void);
void  TIM3_IRQHandler(void);
void  TIM4_IRQHandler(void);
void  TIM5_IRQHandler(void);
void  TIM6_IRQHandler(void);
void  TIM7_IRQHandler(void);
void  TIM6_DAC_IRQHandler (void);
void  TIM8_CC_IRQHandler (void);
void  TIM1_BRK_TIM9_IRQHandler(void);
void  TIM1_TRG_COM_TIM11_IRQHandler(void);
void  TIM8_BRK_TIM12_IRQHandler(void);
void  TIM8_UP_TIM13_IRQHandler(void);  // -- CAUTION SHARED BETWEEN TIM8 and TIM13
void  TIM8_TRG_COM_TIM14_IRQHandler(void);


//******************************************************************************
// TIM_Common_IRQHandler
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
//******************************************************************************
volatile uint16_t      temp_hack_SR;
volatile uint16_t      app_rupts_mask2;

void  TIM_Common_IRQHandler (TIM_TypeDef *TIMbase, int tim_index)
{
    register uint16_t  app_rupts_mask;
    int                interrupt_type;
    TIM_HandleTypeDef  *timHandle;

       //--------------------------------------------------------------
       // get a local copy of what interrupts the User App wants
       // to be called back on. In optimized compilers, this should
       // get loaded into a local register.
       //--------------------------------------------------------------
  app_rupts_mask  = _g_timer_app_enabled_interrupts [tim_index];
app_rupts_mask2   = app_rupts_mask;
temp_hack_SR = (uint16_t) TIMbase->SR;
    app_rupts_mask &= (uint16_t) TIMbase->SR; // Mask the app specified interrupts
                                        // against interrupt status register SR.
                                        // The result is those interrupts the
                                        // app is interested in/configed for

       // Handle timing window, where App requested Timer/PWM Disable, but there
       // were still some interrupts left in the pipe.
    if ((TIMbase->CR1 & TIM_CR1_CEN) == 0)
       {    // Timer/PWM is Disabled. Clear any remaining interrupts
         TIMbase->SR = ~(TIMbase->SR);
         return;                               // then bail out
       }

    if (app_rupts_mask == 0)
       {   // we got an interrupt, but have no active config/setup for that CCR.
           // Must have been a private deal/setup by user app, or timing window.
           // Call HAL Library to handle it, and possibly invoke user callback.
        timHandle = (TIM_HandleTypeDef*) _g_timer_module_handle [tim_index];
        if (timHandle != 0L)
           HAL_TIM_IRQHandler (timHandle);    // Have HAL deal with it
           else {     // We got an interrupt but no handler for it.
                      // Just discard it, otherwise we get int a looping
                      // "Hot I/O" interrupt scenario.
                  TIMbase->SR = ~(TIMbase->SR);         // discard the interrupt
                }
        return;                               // then bail out
       }

       //---------------------------------------------------------------------
       // We sequentially walk down each key interrupt type, so that if
       // multiple interrupts are signalled at the same time, we handle them.
       //---------------------------------------------------------------------
  if (app_rupts_mask & TIM_SR_UIF)       // is it an update/rollover interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_ROLLOVER_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_UPDATE);            // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for UPDATE / rollover rupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC1IF)                // is it a CCR1  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR1_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC1);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC2IF)                // is it a CCR2  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR2_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC2);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC3IF)                // is it a CCR3  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR3_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC3);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }

  if (app_rupts_mask & TIM_SR_CC4IF)               // is it a CCR4  interrupt ?
     {      //---------------------------------------------------------
            // clear the interrupt, then invoke any optional callback
            //---------------------------------------------------------
       interrupt_type = TIMER_CCR4_INTERRUPT;
       TIMbase->SR    = ~(TIM_IT_CC4);               // clear the interrupt flag
       if (_g_ptimer_callback[tim_index] != 0L)
          {             // Invoke user callback for the interrupt
            (_g_ptimer_callback[tim_index]) (_g_ptimer_callback_parm[tim_index],
                                             interrupt_type);
          }
     }
}


void  TIM1_UP_TIM10_IRQHandler (void)      // NVIC IRQn is: TIM1_UP_TIM10_IRQn
{
    if (TIM1->SR & TIM_SR_UIF)
       TIM_Common_IRQHandler (TIM1, 1);     // TIM1

    if (TIM10->SR & (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF))
       TIM_Common_IRQHandler (TIM10, 10);   // TIM10
}

void  TIM1_CC_IRQHandler(void)              // NVIC IRQn is:    TIM1_CC_IRQn
{
    TIM_Common_IRQHandler (TIM1, 1);        // TIM1
}

void  TIM2_IRQHandler (void)                // NVIC IRQn is:    TIM2_IRQn
{
    TIM_Common_IRQHandler (TIM2, 2);        // TIM2
}

void  TIM3_IRQHandler (void)                // NVIC IRQn is:    TIM3_IRQn
{
    TIM_Common_IRQHandler (TIM3, 3);        // TIM3
}

void  TIM4_IRQHandler(void)                 // NVIC IRQn is:    TIM4_IRQn
{
    TIM_Common_IRQHandler (TIM4, 4);        // TIM4
}

void  TIM5_IRQHandler (void)                // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM5, 5);        // TIM5
}

void  TIM6_DAC_IRQHandler (void)            // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM6, 6);        // TIM6
}

void  TIM7_IRQHandler (void)                // NVIC IRQn is:    TIM5_IRQn
{
    TIM_Common_IRQHandler (TIM7, 7);        // TIM6
}



// HAVE A WHOLE BUNCH OF VARIATIONS:  TIM8 + TIM11/12/13

void  TIM8_CC_IRQHandler (void)             // NVIC IRQn is:    TIM8_CC_IRQ
{
    TIM_Common_IRQHandler (TIM8, 8);        // TIM8  CCR rupts only
}

//  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
//  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
//  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
//  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */



void  TIM1_BRK_TIM9_IRQHandler (void)     // NVIC IRQn is:    TIM1_BRK_TIM9_IRQn
{
        // We do not support TIM1_BRK rupts, so only TIM9 rupts should show up
    TIM_Common_IRQHandler (TIM9, 9);
}

void  TIM1_TRG_COM_TIM11_IRQHandler (void)  // NVIC IRQn is:  TIM1_TRG_COM_TIM11_IRQn
{
        // We do not support TIM1_COM/TRIGGER rupts, so only TIM11 rupts should show up
    TIM_Common_IRQHandler (TIM11, 11);
}

//#endif                                    //  USES_TIMER





//*****************************************************************************
//*****************************************************************************
//                               UART   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_uart_init
//
//          F7 Discovery uses USART1 for its Virtual Comm port from USB to PC.
//          Uses pins PA9 (TX) and PB7 (RX).
//
//          CAUTION: you MUST set _g_UartHandle.Init.Parity to UART_PARITY_NONE,
//                   else HAL Lib will default to odd parity and turn on the
//                   UART CR1 PCE and PS bits, whioh totally screws things up !
//*****************************************************************************

void  board_uart_init (void)
{
    // see Clive Example that works fine  (uses old Std Lib, not HAL)

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    GPIO_InitTypeDef    GPIO_InitStruct = { 0 };

       //--------------------------------------------------
       // Configure the GPIOs that used for the UART pins
       // PA.9 = USART1_TX    PB.7 = USART1_RX
       //--------------------------------------------------
    GPIO_InitStruct.Pin        = GPIO_PIN_9;
    GPIO_InitStruct.Alternate  = GPIO_AF7_USART1;     // set Alt Function - UART
    GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull       = GPIO_PULLUP;
    GPIO_InitStruct.Speed      = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);          // Setup UART GPIO TX pin

    GPIO_InitStruct.Pin        = GPIO_PIN_7;
    GPIO_InitStruct.Alternate  = GPIO_AF7_USART1;     // set Alt Function - UART
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);          // Setup UART GPIO RX pin

       //--------------------------------------------------
       // Configure the UART module.  Default = 115200
       //--------------------------------------------------
    __HAL_RCC_USART1_CLK_ENABLE();                    // Turn on UART clocks

    _g_UartHandle.Instance        = USART1;           // Set UART module to use
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
    rc = (_g_UartHandle.Instance->ISR & UART_FLAG_RXNE);
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
    rc = (_g_UartHandle.Instance->ISR & UART_FLAG_RXNE);
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
     rc = (_g_UartHandle.Instance->ISR & UART_FLAG_RXNE);
     if (rc == RESET)
        continue;                           // user needs to type in more chars

           // read in char from UART
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
       rc = (_g_UartHandle.Instance->ISR & UART_FLAG_TXE); // Wait for TX buf Empty

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
           rc = (_g_UartHandle.Instance->ISR & UART_FLAG_TXE); // Wait for TX buf Empty

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
#endif



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
