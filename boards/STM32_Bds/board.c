
/********1*********2*********3*********4*********5*********6*********7**********
*
*                                  board.c                                STM32
*
*
*  Copyright (C) 2014 Grandview Systems
*
*  History:
*    03/10/15 - Simplify file includes. Duquaine
*    08/17/15 - Refactored common sys/gpio routines to simplify maintenance. Duq
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


#ifndef __BOARD_COMMON_C__
#define __BOARD_COMMON_C__

#include "user_api.h"         // pull in high level User API defs
#include "boarddef.h"         // pull in MCU platform defs and board_xx() protos


/*******************************************************************************
*     Pull in the appropriate board_xxx.c file, based on MCU device type
*******************************************************************************/

#if defined(__STM32F072__) || defined(STM32F072xB)
//                                         STM32 - F0_72  Nucleo
//#include "STM32_Bds/STM32_F0/board_F0.c"
#include "board_F0.c"
#define  USES_BSRR
#endif


#if defined(STM32F091xC)
//                                         STM32 - F0_91  Nucleo
#include "STM32_Bds/STM32_F0/board_F0.c"
#define  USES_BSRR
#endif


#if defined(STM32F103xB)
//                                         STM32 - F1_03  Nucleo
#include "STM32_F1/board_F1.c"
#define  USES_BSRR
#endif



#if defined(STM32F303xC) || defined(STM32F303xE)
//                                         STM32 - F3_03  Nucleo and Discovery
#include "STM32_F3/board_F3.c"
#define  USES_BSRRH_BSRRL
#endif


#if defined(STM32F334x8)
//                                         STM32 - F3_34  Nucleo and Discovery
#include "STM32_F3/board_F3.c"
#define  USES_BSRRH_BSRRL
#endif

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F4_01 Nucleo     xE
#include "STM32_F4/board_F4.c"    //               F4_01 Discovery  xC
#define  USES_BSRR
#endif                            //               F4_11 Nucleo     xE

#if defined(STM32F446xx) || defined(STM32F429xx)
//                                         STM32 - F4_46 Nucleo / F4_29 Discovery
#include "STM32_F4/board_F4.c"
#define  USES_BSRR
#endif

#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F7_46  Discovery
#include "STM32_F7/board_F7.c"
#define  USES_BSRR
#endif

#if defined(STM32L053xx)
//                                         STM32 - L0_53  Nucleo and Discovery
#include "STM32_L0/board_L0.c"
#define  USES_BSRR
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L1_52  Nucleo and Discovery
#include "STM32_L1/board_L1.c"
#define  USES_BSRR
#endif


#if defined(STM32L476xx)
//                                         STM32 - L4_76  Nucleo and Discovery
#include "STM32_L4/board_L4.c"
#define  USES_BSRR
#endif


//******************************************************************************
//******************************************************************************
//
//                      COMMON  SYSTEM  and  GPIO  Code
//
// This code is common across all STM32 MCUs for:
//        - system startup and system delay services,
//        - simplified GPIO config and on/off/toggle services
//        - Virtual Timer support   (based on SYSTICK timer)
//******************************************************************************
//******************************************************************************
// The following callouts to board/MCU specific system and GPIO ports are used:
//      _g_gpio_base[]     Array of MCU dependent ports supported and their addr
//
//      board_gpio_init()  MCU dependent logic to turn on clocks for GPIO ports
//
//      board_system_clock_config()  MCU dependent logic to setup CPU clocks



             //********************************************************
             //                 Common System Variables
             //********************************************************
    P_EVENT_HANDLER      pIrqEventHandler = 0;
    int8_t               IntIsMasked;

    uint32_t   _g_SysClk_Ticks = MCU_CLOCK_SPEED; // Global to hold clock frequency (ticks/sec)

    uint32_t   _g_systick_millisecs = 0; // Global to how many 1 ms ticks have
                                         // accumulated startup (poor mans TOD)

    char       _g_vtimers_active    = 0; // Optional VTIMER support


    long       Systick_Frequency  = 100;    // 10 ms ticks
    long       SysTick_Reload_Val = 0;      // actual TMR reload value

    uint32_t   ulTickCount = 0;             // SysTick count

    char      _g_pwm_mclock_configured = 0; // PWM master clock was configured
    uint32_t  _g_pwm_mdivider = 1;          // PWM Master clock : CPU divider ratio


extern const  GPIO_TypeDef * _g_gpio_base[];  // MCU dependent array of GPIO Ports

    float      _g_fusec_to_ticks = 1.0;        // ??? !!! FIX THIS - BAD PERF ON F0

                    //--------------------------------------------------------
                    // Common lookup table for getting STM32 bit settings for
                    // GPIO pin number. STM32 has Up to 16 (0-15) pins per port.
                    //--------------------------------------------------------
const  uint32_t  _g_gpio_pin_num[] = {
                                       GPIO_PIN_0,
                                       GPIO_PIN_1,
                                       GPIO_PIN_2,
                                       GPIO_PIN_3,
                                       GPIO_PIN_4,
                                       GPIO_PIN_5,
                                       GPIO_PIN_6,
                                       GPIO_PIN_7,
                                       GPIO_PIN_8,
                                       GPIO_PIN_9,
                                       GPIO_PIN_10,
                                       GPIO_PIN_11,
                                       GPIO_PIN_12,
                                       GPIO_PIN_13,
                                       GPIO_PIN_14,
                                       GPIO_PIN_15
                                     };

                    //--------------------------------------------------------
                    // Common lookup table for getting STM32 bit settings for
                    // PullUp/PullDown/... options
                    //--------------------------------------------------------
const  int  _g_gpio_pull_flags[] = {  GPIO_PULLDOWN,   // index = 0x00
                                      GPIO_PULLUP,     // index = 0x01
                                      GPIO_PULLDOWN,   // index = 0x10
                                      GPIO_PULLDOWN    // index = 0x11 (invalid combo)
                                   };



//**************************************************************************
//**************************************************************************
//
//                        COMMON   SYSTEM   Routines
//
//**************************************************************************
//**************************************************************************


//******************************************************************************
//  board_init
//
//            Initializes board clocks and basic GPIOs.
//******************************************************************************

void  board_init (long mcu_clock_rate, int option_flags)
{
    uint32_t  SysFreq;
    uint32_t  SysCoreClk;

       //-----------------------------------------------------------------------
       // Reset all peripherals, Initialize Flash interface and Systick.
       // This calls the generic HAL_Init() in stm32f3xx_hal.c
       // which in turn does a callback to our specific peripherals GPIO init
       // for SPI, DMA, ... by invoking HAL_MspInit() in our stm32f3xx_hal_msp.c
       //-----------------------------------------------------------------------

#if defined(STM32F746NGHx) || defined(STM32F746xx)
           //------------------------------------------------------------------
           // Enable the F7 CPU caches, both Instruction (Flash) and Data (RAM)
           //------------------------------------------------------------------
    SCB_EnableICache();             // Enable F7 I-Cache
    SCB_EnableDCache();             // Enable F7 D-Cache
#endif

    board_stop_WDT();               // ensure any watchdog timer is off/disabled

#if defined(STM32F334x8)   // || defined(STM32F303xE) || defined(STM32F303xC)
        // For STM32 F3_34, we need to initialize the System Clocks first,
        // before we invoke HAL_Init(), otherwise it tries to set the Systick
        // period to 0, because default SystemCoreClock = 0 at startup on F3_34.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,option_flags); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,option_flags);  // use default MCU speed
    SysFreq    = HAL_RCC_GetSysClockFreq();        // ensure we now have valid clocks
    SysCoreClk = HAL_RCC_GetHCLKFreq();

    HAL_Init();                     // Invoke ST's HAL startup logic

#else

    HAL_Init();                     // Invoke ST's HAL startup logic
                                    // Note that it always enables SYSTICK timer.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,
                                  option_flags); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,
                                  option_flags);  // use default MCU speed

    __enable_irq();                 // Ensure interrupts enabled for SysTick
#endif

       //------------------------------------------
       // Invoke MCU dependent GPIO clock startup
       //------------------------------------------
    board_gpio_init();
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

//  total_ticks = (long) (g_fusec_to_ticks * (float) usec_delay);  //  ??? !!! FIX  BAD PERF on M0 PROCESSORS !!!
    total_ticks = (long) (8 * (float) usec_delay);       // TEMP HACK 03/11/15

   while (total_ticks > 0)
     total_ticks -= 3;                  // each iteration is approx 3 instr
}


//*****************************************************************************
//  board_delay_ms
//*****************************************************************************

void  board_delay_ms (long  num_millis)    // Delay of 1 ms
{

// NEXT PASS IMPROVEMENT !!! ??? for low power systems, enter LPM mode here ...
//                                               and wait for SYSTICK interrupt
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
//         Turn off Global interrupts  (for CC3000, Timer config, ...)
//
//      ASM ("cpsid i");      // CAUTION: do NOT do any SVC calls or will crash
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

//#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration (_g_systick_millisecs);
//#endif
}



//*****************************************************************************
//*****************************************************************************
//
//                            COMMON   GPIO   Routines
//
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
*  board_gpio_pin_lookup
*
*          Lookup the associated GPIO Port and Pin # for a given pin_id (0-255)
*          On STM32, there are normally up to 16 pins (0-15) per GPIO port.
*******************************************************************************/
int  board_gpio_pin_lookup (unsigned int pin_id, GPIO_TypeDef **GPIO_port, uint32_t *GPIO_pin)
{
    if (pin_id > GPIO_MAX_PIN_ID)
       return (ERR_GPIO_INVALID_PIN_ID);

    *GPIO_port = (GPIO_TypeDef*) _g_gpio_base [(pin_id >> 4)];
    *GPIO_pin  = _g_gpio_pin_num [(pin_id & 0x000F)];

    return (0);                                  // denote operation woprked OK
}


/*******************************************************************************
*  Board GPIO Pin Config
*
*        Configure an individual GPIO pin - straight GPIO
*******************************************************************************/
int  board_gpio_pin_config (unsigned int pin_id, int dir, int flags)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_TypeDef       *gpio_port;
    uint32_t           gpio_pin;
    int                rc,   pull,   rise_fall;

    rc = board_gpio_pin_lookup (pin_id, &gpio_port, &gpio_pin);
    if (rc)
       return (rc);                                    // return error code

       //---------------------------------------
       //      Initialize GPIO_InitStruct
       //---------------------------------------
    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));   // clear struct

    GPIO_InitStruct.Pin = gpio_pin;

    if (dir == GPIO_OUTPUT)
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // GPIO Output
       else GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    // GPIO Input

    rise_fall = flags & 0xFF00;
    if (rise_fall)
       {    // override mode if Rise/Fall requested
         if (rise_fall == GPIO_RUPT_MODE_RISING)
            GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // GPIO Rising Input triggers
            else if (rise_fall == GPIO_RUPT_MODE_FALLING)
                    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //Falling Input trigger
       }

#if ! defined(STM32F103xB)
    GPIO_InitStruct.Alternate = 0;              // configure a normal GPIO pin
#endif

    pull = flags & 0x00FF;
    if (pull == GPIO_PULLUP)
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       else if (pull == GPIO_PULLDOWN)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;
               else GPIO_InitStruct.Pull = GPIO_NOPULL;

#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#else
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#endif

    HAL_GPIO_Init (gpio_port, &GPIO_InitStruct);        // Setup GPIO pin
}


/*******************************************************************************
*  Board GPIO Pin Config with PINMUX ALT-FUNCTION
*
*       Configure an individual GPIO pin - similar to above but PinMux ALT-FUNC.
*       This capability is provided, for Advanced programmers to do custom things.
*******************************************************************************/
int  board_gpio_pin_config_pinmux (unsigned int pin_id,
                                   int dir, int pull, int alt_func)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_TypeDef       *gpio_port;
    uint32_t           gpio_pin;
    int                rc;

    rc = board_gpio_pin_lookup (pin_id, &gpio_port, &gpio_pin);
    if (rc)
       return (rc);                                    // return error code

       //---------------------------------------
       //      Initialize GPIO_InitStruct
       //---------------------------------------
    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));   // clear struct

    GPIO_InitStruct.Pin = gpio_pin;

    if (dir == GPIO_OUTPUT)
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // GPIO Output
       else GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    // GPIO Input

#if ! defined(STM32F103xB)
    GPIO_InitStruct.Alternate = alt_func;  // set ALT FUNCTION  (Timer, SPI, I2C, I2S, ...)
#endif

    if (pull == GPIO_PULLUP)
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       else if (pull == GPIO_PULLDOWN)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;
               else GPIO_InitStruct.Pull = GPIO_NOPULL;

#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#else
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#endif

    HAL_GPIO_Init (gpio_port, &GPIO_InitStruct);      // Setup GPIO pin
}


/*******************************************************************************
*  board_irq_pin_config
*
*        Configure an individual GPIO pin to support incoming EXTI Interrupt
*        requests
*******************************************************************************/
int  board_irq_pin_config (unsigned int pin_id,
                           int rise_fall, int pullup, uint32_t irq_vector_num,
                           unsigned long priority)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_TypeDef       *gpio_port;
    uint32_t           gpio_pin;
    int                rc;

    rc = board_gpio_pin_lookup (pin_id, &gpio_port, &gpio_pin);
    if (rc)
       return (rc);                                    // return error code

       //---------------------------------------
       //      Initialize GPIO_InitStruct
       //---------------------------------------
    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));   // clear struct

    GPIO_InitStruct.Pin = gpio_pin;

    if (rise_fall == GPIO_RUPT_MODE_RISING)
       GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // GPIO Rising Input triggers
       else if (rise_fall == GPIO_RUPT_MODE_FALLING)
            GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //Falling Input trigger

#if ! defined(STM32F103xB)
    GPIO_InitStruct.Alternate = 0;
#endif

    if (pullup == GPIO_PULLUP)
       GPIO_InitStruct.Pull = GPIO_PULLUP;               // Engage pullups
       else if (pullup == GPIO_PULLDOWN)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;     // Engage pulldowns
               else GPIO_InitStruct.Pull = GPIO_NOPULL;  // Disengage pullups

#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#else
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#endif

    HAL_GPIO_Init (gpio_port, &GPIO_InitStruct);         // Setup GPIO pin

       //--------------------------------------------------------
       // Setup associated EXTI IRQn vector.
       //
       // A -1 denote do NOT setup EXTI vector.
       //--------------------------------------------------------
    if (irq_vector_num != -1)
       {      // Configure the NVIC for the Interrupt coming from the shield
          HAL_NVIC_SetPriority (irq_vector_num, priority, 0);

              // Model TI's support, and avoid any "early interrupts" that
              // ISR may not be expecting (e.g. CC3100 Simplelink)   04/20/15 change
          HAL_NVIC_DisableIRQ (irq_vector_num);
       }
}


/*******************************************************************************
*  board_irq_pin_disable
*
*        Enable a GPIO pin for an incoming interrupt
*******************************************************************************/
int  board_irq_pin_disable (unsigned int pin_id,
                            uint32_t irq_vector_num, int clear_pending_rupts)
{
    HAL_NVIC_DisableIRQ (irq_vector_num);

    if (clear_pending_rupts)
       HAL_NVIC_ClearPendingIRQ (irq_vector_num); //clear any previous interrupts

    return (0);
}


/*******************************************************************************
*  board_irq_pin_enable
*
*        Enable a GPIO pin for an incoming interrupt
*******************************************************************************/
int  board_irq_pin_enable (unsigned int pin_id,
                           uint32_t irq_vector_num, int clear_pending_rupts)
{
    if (clear_pending_rupts)
       HAL_NVIC_ClearPendingIRQ (irq_vector_num); //clear any previous interrupts

    HAL_NVIC_EnableIRQ (irq_vector_num);

    return (0);
}



/*******************************************************************************
*  board_gpio_read_port
*
*        Read in MULTIPLE pins from a GPIO port.  Desired pins are passed via mask.
*
*        If flag = 0 = read input port.
*        If flag = 1 = read output port.
*******************************************************************************/

uint16_t  board_gpio_read_port (unsigned int gpio_port_num, uint16_t pins_mask, int flag)
{
    GPIO_TypeDef  *gpio_port;
    uint16_t      port_values;

//  if (gpio_port_num > MAX_GPIO_PORTS)
//     return (ERR_GPIO_INVALID_PORT_ID);               // return error

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [gpio_port_num];

    if ( ! flag)
       port_values = gpio_port->IDR & (pins_mask);      // get INPUT port pins
       else port_values = gpio_port->ODR & (pins_mask); // get OUTPUT port pins

    return (port_values);
}



/*******************************************************************************
*  board_gpio_read_pin
*
*        Read in a SINGLE pin from a GPIO port.
*
*        If flag = 0 = read input port.
*        If flag = 1 = read output port.
*******************************************************************************/

uint16_t  board_gpio_read_pin (unsigned int pin_id, int flag)
{
    GPIO_TypeDef  *gpio_port;
    uint32_t      gpio_pin;
    uint16_t      port_value;

    if (pin_id > GPIO_MAX_PIN_ID)
       return (ERR_GPIO_INVALID_PIN_ID);               // return error

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [(pin_id >> 4)];
    gpio_pin  = _g_gpio_pin_num [(pin_id & 0x000F)];

    if ( ! flag)
       port_value = gpio_port->IDR & (gpio_pin);       // get INPUT port pin
       else port_value = gpio_port->ODR & (gpio_pin);  // get OUTPUT port pin

    return (port_value);
}


/*******************************************************************************
*  board_gpio_toggle_pin
*
*       Toggle a SPECIFIC pin on a port.
*******************************************************************************/

int  board_gpio_toggle_pin (unsigned int pin_id)
{
    GPIO_TypeDef  *gpio_port;
    uint32_t      gpio_pin;

    if (pin_id > GPIO_MAX_PIN_ID)
       return (ERR_GPIO_INVALID_PIN_ID);

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [(pin_id >> 4)];
    gpio_pin  = _g_gpio_pin_num [(pin_id & 0x000F)];

    gpio_port->ODR ^= (gpio_pin);            // XOR toggle the pin on that port

    return (0);
}


/*******************************************************************************
*  board_gpio_write_pin
*
*        Write to a SPECIFIC pin on a port.
*        high_low_flag:  0 = low   1 = high
*******************************************************************************/

int  board_gpio_write_pin (unsigned int pin_id, int high_low_flag)
{
    GPIO_TypeDef  *gpio_port;
    uint32_t      gpio_pin;

    if (pin_id > GPIO_MAX_PIN_ID)
       return (ERR_GPIO_INVALID_PIN_ID);

    gpio_port = (GPIO_TypeDef*) _g_gpio_base [(pin_id >> 4)];
    gpio_pin  = _g_gpio_pin_num [(pin_id & 0x000F)];

#if defined(USES_BSRR)
    if (high_low_flag)
       gpio_port->BSRR = gpio_pin;                          // set it HIGH
       else gpio_port->BSRR = ((uint32_t) gpio_pin << 16);  // set it LOW

#else

    if (high_low_flag)                    // mainly for F3_03  and  F3_34
       gpio_port->BSRRH = gpio_pin;       // Turn ON  a pin on a given a port
       else gpio_port->BSRRL = gpio_pin;  // Turn OFF a pin on a given a port

#endif

    return (0);
}



//*****************************************************************************
//*****************************************************************************
//
//                      COMMON     VIRTUAL   TIMER    Routines
//
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

int  board_vtimer_start (unsigned int vtimer_id, uint32_t timer_duration_millis,
                         P_EVENT_HANDLER callback_function,  void *callback_parm)
{
     if (vtimer_id > 9)
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
//
//         Returns:
//              True  (1) = completed
//              False (0) = Busy
//                    -1  = error
//*****************************************************************************

int  board_vtimer_completed (unsigned int vtimer_id)
{
     if (vtimer_id > 9)
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

int  board_vtimer_reset (unsigned int vtimer_id)
{
     if (vtimer_id > 9)
        return (ERR_VTIMER_ID_OUT_OF_RANGE);

     _g_vtimer_flags[vtimer_id] = VTIMER_RESET;
     _g_vtimer_user_flags [vtimer_id] = VTIMER_RESET;

    return (0);               // denote completed successfully
}

#endif                          //  __BOARD_COMMON_C__
