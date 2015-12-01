
// 07/03/15 - ADC TRIGGERing - associated TIMx CCR2's MMS field needs to be set
//            to 0x2 (Update) or 0x4 (CCR1, 0x5 (CCR2), 0x6 (CCR3), 0x7 (CCR4)
//            Right now, are only doing it for TIM6/7, but needs to be done
//            ofr ANY timer that needs to drive ADC (TIM1/TIM2/...)

/********1*********2*********3*********4*********5*********6*********7**********
*
*                                  board_F4.c   -   STM32 F4
*
*
* STM32Cube / HAL based version of board_xxx() functions
*
*
* Supported/Tested Boards
* -----------------------
*      STM32_F4_01_RE    Nucleo-F4    512K Flash     96K RAM     84 MHz
*                                        1 ADC @ 16 channels
*                                        3 SPI       3 I2C       1 USB
*      STM32_F4_01_RE    Discovery
*
*
*  History:
*    12/30/14 - Created for Industrial IoT OpenSource project.  Duq
*    04/xx/15 - Added multi-channel ADC support. Duq
*    04/30/15 - Added multi-channel PWM support.  WORKS.   Duq
*    05/25/15 - Fixed lingering issues in ADC DMA support. Duq
*    07/03/15 - Merge in changes for Timer / PWM enhancements. Duq
*    09/18/15 - Additional factoring on UART support for GSM and ESP8266. Duq
*    09/22/15 - Even when transmitting, run with RXNE Receive interrupts turned
*               on to handle any echo-plexed responses (at startup) from FDX
*               devices like SIM-80x and ESP8266. Otherwise you get all kinds of
*               of ORE errors during Transmit and missed sync up on Receives.
*               Incoming received characters are saved in internal RX circular buffer,
*               that user app can either process or discard. (Under the covers
*               this is how the Arduino UART support works, and that is what
*               those FDX devices are tuned for.) ST's HAL logic only turns on
*               RXNE for receives, and immediately turns them off when complete.
*               That causes "issues" when working with SIM-80x, ESP8266, etc.
*   09/23/15 - Finished final tweaks to RX interrupts and circular queue support.
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

#include "user_api.h"                   // pull in API defs and MCU depdent defs
#include "device_config_common.h"

#include <math.h>

#include "stm32f4xx_hal.h"


#if defined(USES_CC3000) || defined(CC3000_USE_BOOSTERPACK1)
long hci_unsolicited_event_handler(void);
#define  FALSE   0
#define  TRUE    1
#endif

// move the following to boarddef.h at some point
                 // internal Function protos
int  board_dac_check_conversions_done (int dac_module_id, int sequencer);
int  board_dac_clear_conversions_done (int dac_module_id, int sequencer);
int  board_dac_SineWave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_Trianglewave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_SquareWave  (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int  board_timerpwm_enable_clock (int module_id);
long board_timerpwm_compute_prescalar (long period_val);
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id);
int  board_uart_process_activation_chars (UART_HandleTypeDef *pUartHdl, uint8_t in_char, char *read_buf);
void board_error_handler (void);                    // subroutine prototypes

// internal Function protos
uint32_t HAL_GetTick (void);   // overrides for HAL Systick functions
void     HAL_IncTick (void);

void  DMA1_Channel1_IRQHandler (void);              // ADC DMA ISR - all channels
void  DMA2_Stream0_IRQHandler (void);               // DAC DMA ISR - all channels

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

extern  uint32_t       _g_SysClk_Ticks;
extern  uint32_t       _g_systick_millisecs;

    short              IRQ_interrupts_enabled = -1;

    long               irq_state;
    long               ulSpiIRQState;

    long               brc;

    unsigned char      Hci_Link_Start []   = { 0x01,0x00,0x05,0x00,0x00,
                                               0x01,0x00,0x40,0x01,0x00  };
    unsigned char      Hci_Link_Reply [10] = { 0,0,0,0,0,0,0,0,0,0 };


#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    unsigned long   _g_crcid_Result;
#endif

               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
#define  GPIO_MAX_PIN_ID     PH1           /* F4_01/F4_11/F4_46 max is PH1 */

const  GPIO_TypeDef * _g_gpio_base[] =
    {
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
        GPIOE,
#if defined(STM32F446xx)
        GPIOF,
        GPIOG,
#else
          0L,                 // No PORT F on F4_01 and F4_11
          0L,                 // No PORT G on F4_01 and F4_11
#endif
        GPIOH,
    };

#define  GP_PORT_A   0        // indexes into above table, based on port id
#define  GP_PORT_B   1
#define  GP_PORT_C   2
#define  GP_PORT_D   3
#define  GP_PORT_E   4
#define  GP_PORT_F   5
#define  GP_PORT_G   6
#define  GP_PORT_H   7


//*****************************************************************************
//*****************************************************************************
//     MCU Dependent       GPIO   Routines
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

    __GPIOA_CLK_ENABLE();     // turn on GPIO clocks for Ports A, B, C
    __GPIOB_CLK_ENABLE();     // since they are used in many spots
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();

}



//*****************************************************************************
//*****************************************************************************
//
//     MCU Dependent       System CPU / Systick   Routines
//
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



#if defined(STM32F401xE) || defined(STM32F401xC)

/*******************************************************************************
*
*                             F4_01   CPU  Clock  Init
*
* System Clock Configuration
*
*         The system Clock is configured as follow :
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 84000000
*            HCLK(Hz)                       = 84000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 2
*            APB2 Prescaler                 = 1
*            HSI Frequency(Hz)              = 16000000
*            PLL_M                          = 16
*            PLL_N                          = 336
*            PLL_P                          = 4
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale2 mode
*            Flash Latency(WS)              = 2
*******************************************************************************/

void  board_system_clock_config (long mcu_clock_hz, int option_flags)
{
    RCC_ClkInitTypeDef   RCC_ClkInitStruct;
    RCC_OscInitTypeDef   RCC_OscInitStruct;

    memset (&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));  // clear struct
    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct

    __PWR_CLK_ENABLE();                  // Enable Power Control clock

        // The voltage scaling allows optimizing the power consumption when
        // the device is clocked below the maximum system frequency.
        // To update the voltage scaling value, refer to product datasheet
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE2);

    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED_84_MHz; // use default clock rate = 84 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

    if (option_flags != 0)
       {     // Use external HSE Oscillator
             // Enable HSE Oscillator and activate PLL with HSE as source
         RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
         RCC_OscInitStruct.HSIState       = RCC_HSE_ON;   // EXTERNAL Crystal clock
       }
      else
       {
             // Enable internal  HSI Oscillator and activate PLL with HSI as source
         RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
         RCC_OscInitStruct.HSIState       = RCC_HSI_ON;   // INTERNAL RC clock
       }
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
       {
         board_error_handler();
       }

        // Select the PLL as the system clock source and configure the HCLK,
        // PCLK1, and PCLK2  clocks dividers to facilitate 84 MHz
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
       {
         board_error_handler();
       }

/// _g_SysClk_Ticks = MCLK_MHz * 84000000;  // save MCU clock frequency in ticks

}

#endif                                      // defined(STM32F401xE)


#if defined(STM32F411xE)
/*******************************************************************************
*
*                            F4_11   CPU  Clock  Init
*
* System Clock Configuration
*
*         System Clock is configured as follows:
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 100000000       (100 MHz)
*            HCLK(Hz)                       = 100000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 2
*            APB2 Prescaler                 = 1
*            HSI Frequency(Hz)              = 16000000
*            PLL_M                          = 16
*            PLL_N                          = 400
*            PLL_P                          = 4
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale2 mode
*            Flash Latency(WS)              = 3
*******************************************************************************/
void  board_system_clock_config (long  mcu_clock_hz, int option_flags)
{
    RCC_ClkInitTypeDef   RCC_ClkInitStruct;
    RCC_OscInitTypeDef   RCC_OscInitStruct;

    memset (&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));  // clear struct
    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct

    __HAL_RCC_PWR_CLK_ENABLE();              // Enable Power Control clock

       // Voltage scaling allows optimizing power consumption when device
       // is clocked below the maximum system frequency. To update the voltage
       // scaling value regarding, refer to the product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE2);

    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED_100_MHz; // use default clock rate = 100 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

        /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;          // INTERNAL RC clock
    RCC_OscInitStruct.HSICalibrationValue = 0x10;           // BLE uses 6
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 400;                 // BLE uses 256
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;       // BLE uses DIV8
    RCC_OscInitStruct.PLL.PLLQ       = 7;                   // BLE uses 4
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
       {
         board_error_handler();
       }

        // Select PLL as system clock source and configure the HCLK,
        // PCLK1 and PCLK2 clock dividers to facilitate 100 MHz
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1
                                      | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
       {
         board_error_handler();
       }

/// _g_SysClk_Ticks = MCLK_MHz * 1000000000; // save MCU clock frequency in ticks

}

#endif                                       // defined(STM32F411xE)



#if defined(STM32F429xx) || defined(STM32F446xx)

/*******************************************************************************
*
*                           F4_29  and  F4_46   CPU  Clock  Init
*
* System Clock Configuration
*         The F4_46 system Clock is configured as follows:
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 180000000
*            HCLK(Hz)                       = 180000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 4
*            APB2 Prescaler                 = 2
*            HSI Frequency(Hz)              = 16000000
*            PLL_M                          = 16
*            PLL_N                          = 360
*            PLL_P                          = 2
*            PLL_Q                          = 7
*            PLL_R                          = 6
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale1 mode
*            Flash Latency(WS)              = 5
*******************************************************************************/

void  board_system_clock_config (long  mcu_clock_hz, int options_flags)
{
    RCC_ClkInitTypeDef   RCC_ClkInitStruct;
    RCC_OscInitTypeDef   RCC_OscInitStruct;
    HAL_StatusTypeDef    ret;

    memset (&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));  // clear struct
    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct

    ret = HAL_OK;

    __HAL_RCC_PWR_CLK_ENABLE();            // Enable Power Control clock

         // Voltage scaling allows optimizing the power consumption when
         // the device is clocked below the maximum system frequency.
         // To update the voltage scaling value, refer to product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED_180_MHz; // use default clock rate = 180 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

        // Enable HSI Oscillator and activate PLL with HSI as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;          // INTERNAL RC clock
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    RCC_OscInitStruct.PLL.PLLR       = 6;
    ret = HAL_RCC_OscConfig (&RCC_OscInitStruct);
    if (ret != HAL_OK)
       {
         board_error_handler();
       }

        // Activate the OverDrive to reach the 180 MHz Frequency
    ret = HAL_PWREx_EnableOverDrive();
    if (ret != HAL_OK)
       {
         board_error_handler();
       }
        // Select PLL as system clock source and configure the HCLK,
        // PCLK1 and PCLK2 clocks dividers to facilitate 180 MHz
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    ret = HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5);
    if (ret != HAL_OK)
       {
         board_error_handler();
       }

/// _g_SysClk_Ticks = MCLK_MHz * 1800000000; // save MCU clock frequency in ticks

}

#endif                                       // defined(STM32F446xx)


#if SHUTOFF_UART_RTNS
//*****************************************************************************
//*****************************************************************************
//                              UART   Routines
//
//        Routines to Init, Read, and Write to a UART module.
//        Only provides for TX/RX support, no support for RTS/CTS.
//
//  VCP Notes:
//        The VCP (Virtual Comm Port), is the default UART TX/RX pins connected
//        from the Nucleo or Discovery board to the onboard ST-Link component,
//        that provides serial UART emulation to a PC via USB CDC protocol.
//        On Nucleos, this is wired to the "Arduino" D1/D0 pins on the bottom
//        right Arduino connector. These pins are effectively dedicated to
//        the VCP function, and will not work as GPIOs, unless you re-solder
//        the solder bridges on the Nucleo (in which case, you lose VCP support)
//*****************************************************************************
//*****************************************************************************

//            TX Pin  AF    RX Pin   AF    Arduino
// USART 1     PA9     7     PA10     7
//             PB6     7     PB7      7
//             PA15    7     PB3      7         -- F4_11 only --
//
//
// USART 2     PA2     7     PA3      7     VCP
//             PD5     7     PD6      7
//
// USART 3     PB10    7     PB11     7         -- F4_46 only --
//             -       -     PC5      7         -- F4_46 only --
//             PC10    7     PC11     7         -- F4_46 only --
//             PD8     7     PD9      7         -- F4_46 only --
//
//
// UART 4      PA0     8     PA1      8         -- F4_46 only --
// ^^^^        PC10    8     PC11     8         -- F4_46 only --
//  note !
//
// UART 5      PC12    8     PD2      8         -- F4_46 only --
// ^^^^        PE8     8     PE7      8         -- F4_46 only --
//
// USART 6     PC6     8     PC7      8
//             PA11    8     PA12     8         -- NOT on F4_46 --
//             PG14    8     PG9      8         -- F4_46 only --
//----------------------------------------------------------------------

//----------------------------------------------------------------------
int  board_uart_enable_clock (int module_id);       // internal routines
void board_uart_enable_nvic_irq (int module_id);
int  board_uart_read_common (UART_HandleTypeDef *pUartHdl, char *read_buf,
                             int buf_max_length, int is_string_IO);
int  board_get_uart_handle (int module_id, UART_HandleTypeDef **ret_UartHdl);
void board_common_UART_IRQHandler (USART_TypeDef *uart_module, int uart_module_id);
void USART1_IRQHandler (void);
void USART2_IRQHandler (void);
void USART6_IRQHandler (void);

#define  UART_TRACE_BUF_SIZE   0x1FF
    uint8_t   long_trace[UART_TRACE_BUF_SIZE+1];   // 512 bytes
    int       trace_idx = 0;

#define  UART_INTERNAL_BUF_SIZE   0x7F  // modulo value for 128 byte buffer

    char      _g_uart_last_char   = 0;  // keeps track of user input via UART
    uint32_t  _g_uart_end_time    = 0;
    uint32_t  _g_uart_timeout_val = 0;

    uint8_t   _g_rx_buf [UART_INTERNAL_BUF_SIZE+1];    // 128 bytes
    int       _g_uart_rx_head;         // head and tail ptrs for _g_rx_buf
    int       _g_uart_rx_tail;
    int       _g_rx_amt_rcvd;
    int       _g_rcv_max_length;
    uint8_t   *_g_rcv_user_buf = 0L;  // TEMP HACK

    uint8_t   _g_tx_buf [UART_INTERNAL_BUF_SIZE+1];        //TEMP HACK
    int       _g_tx_idx;
    int       _g_tx_length;
    uint8_t   *_g_snd_user_buf = 0L;  // TEMP HACK

UART_CB_EVENT_HANDLER  _g_uart_callback_function = 0L;
    void              *_g_uart_callback_parm;

    uint8_t   _g_uart_do_interrupts = 0;   // use interrupts for all UART I/O
    uint8_t   _g_uart_do_echoplex   = 1;   // echo-plexing is turned on by default
    uint8_t   _g_uart_is_string_IO  = 0;   // current operation is a read_string, etc

    uint8_t   _g_uart_tx_state = 5;        // in full duplex, both can be active
    uint8_t   _g_uart_rx_state = 5;

#define  UART_STATE_XMIT_BUSY       1
#define  UART_STATE_RCV_BUSY        2
#define  UART_STATE_XMIT_COMPLETE   3
#define  UART_STATE_RCV_COMPLETE    4
#define  UART_STATE_RESET           5

#define  UART_MAX_MODULE       6

const  USART_TypeDef   *_g_uart_module [] =
        {    USART2,              // [0] = default module = UARTMD / VCP
             USART1,              // [1] = UARTCM1
             USART2,              // [2] = UARTM2
#if defined(STM32F446xx)
             USART3,              // [3] = USARTM3
             UART4,               // [4] = UARTM4
             UART5,               // [5] = UARTM5
#else
              0L,                 // [3] no USART3  on F4_01 and F4_11
              0L,                 // [4] no UART4       "         "
              0L,                 // [5] no UART5       "         "
#endif
             USART6,              // [6] = UARTM6
        };


                            //----------------------------------------
                            //   SPIn handles needed, one per module
                            //----------------------------------------
    UART_HandleTypeDef  _g_UartHandle_1;           // UART 1 support
    UART_HandleTypeDef  _g_UartHandle_2;           // UART 2 support (also VCP)
#if defined(STM32F446xx)
    UART_HandleTypeDef  _g_UartHandle_3;           // UART 3 support
    UART_HandleTypeDef  _g_UartHandle_4;           // UART 4 support
    UART_HandleTypeDef  _g_UartHandle_5;           // UART 5 support
#endif
    UART_HandleTypeDef  _g_UartHandle_6;           // UART 6 support

                            //----------------------------------------------------------
                            // List of HAL Handle Addresses, one for each separate UART Handle
                            //----------------------------------------------------------
const UART_HandleTypeDef *_g_uart_typedef_handle_addr []          // HAL API Handles
                     = { (UART_HandleTypeDef*) &_g_UartHandle_2,  // UARTMD / VCP
                         (UART_HandleTypeDef*) &_g_UartHandle_1,  // UARTM1
                         (UART_HandleTypeDef*) &_g_UartHandle_2,  // UARTM2
#if defined(STM32F446xx)
                         (UART_HandleTypeDef*) &_g_UartHandle_3,  // UARTM3
                         (UART_HandleTypeDef*) &_g_UartHandle_4,  // UARTM4
                         (UART_HandleTypeDef*) &_g_UartHandle_5,  // UARTM5
#else
                                                      0L,         // no UARTM3
                                                      0L,         // no UARTM4
                                                      0L,         // no UARTM5
#endif
                         (UART_HandleTypeDef*) &_g_UartHandle_6,  // UARTM5
                       };



typedef struct uart_gpio_def              /* UART GPIO definitions */
    {
        USART_TypeDef *uart_module;        /* UART module: USART1/2/...  */
        GPIO_TypeDef  *uart_gpio_port;     /* Associated GPIO port       */
        uint16_t      uart_gpio_pin;       /* Associated GPIO pin        */
        uint16_t      uart_chan_alt_func;  /* UART Alternate Function Id */
        uint16_t      uart_pin_role;       /* TX or RX                   */
    } UART_GPIO_BLK;

#define TX_PIN_ROLE   1
#define RX_PIN_ROLE   1

               //--------------------------------------------------------------
               //                       GPIO   MAPPING   TABLES
               //
               // These tables contain the GPIO mapping for every GPIO pin that
               // can be used for Input (Input Capture) or Output (PWM or
               // Timer Output Compare) by the Timer/PWM modules.
               // pin each alternative is assigned to (e.g. PA8, PA7, or PB13)
               //--------------------------------------------------------------
const UART_GPIO_BLK  _g_uart_1_gpio_table [] =
      {
        { USART1, GPIOA, GPIO_PIN_9,  GPIO_AF7_USART1, TX_PIN_ROLE },  // PA9  D8
        { USART1, GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, RX_PIN_ROLE },  // PA10 D2
      //  ^^^^^^
      //  redundant if we have separate tables by UART: _g_uart_1_gpio_table / _g_uart_2_gpio_table / ...
        { USART1, GPIOB, GPIO_PIN_6,  GPIO_AF7_USART1, TX_PIN_ROLE },  // PB6  D10
        { USART1, GPIOB, GPIO_PIN_7,  GPIO_AF7_USART1, RX_PIN_ROLE },  // PB7 - left CN, halfway up
#if defined(STM32F411xE)
        { USART1, GPIOA, GPIO_PIN_15, GPIO_AF7_USART1, TX_PIN_ROLE },  // PA15
        { USART1, GPIOB, GPIO_PIN_3,  GPIO_AF7_USART1, RX_PIN_ROLE },  // PB3  D3
#endif
      };


const UART_GPIO_BLK  _g_uart_2_gpio_table [] =
      {
        { USART2, GPIOA, GPIO_PIN_2,  GPIO_AF7_USART2, TX_PIN_ROLE },  // PA2 VCP
        { USART2, GPIOA, GPIO_PIN_3,  GPIO_AF7_USART2, RX_PIN_ROLE },  // PA3 VCP
        { USART2, GPIOD, GPIO_PIN_5,  GPIO_AF7_USART2, TX_PIN_ROLE },  // PD5 - not routed to CN pins
        { USART2, GPIOD, GPIO_PIN_6,  GPIO_AF7_USART2, RX_PIN_ROLE },  // PD6 -   "
      };


const UART_GPIO_BLK  _g_uart_6_gpio_table [] =
      {
        { USART6, GPIOC, GPIO_PIN_6,  GPIO_AF8_USART6, TX_PIN_ROLE },  // PC6
        { USART6, GPIOC, GPIO_PIN_7,  GPIO_AF8_USART6, RX_PIN_ROLE },  // PC7  D9
#if ! defined(STM32F446xx)
        { USART6, GPIOA, GPIO_PIN_11, GPIO_AF8_USART6, TX_PIN_ROLE },  // PA11 - right CN, halfway up
        { USART6, GPIOA, GPIO_PIN_12, GPIO_AF8_USART6, RX_PIN_ROLE },  // PA12 -   "         "
#endif
#if defined(STM32F446xx)
        { USART6, GPIOG, GPIO_PIN_14, GPIO_AF8_USART6, TX_PIN_ROLE },  // PG14 - not routed to CN pins
        { USART6, GPIOG, GPIO_PIN_9,  GPIO_AF8_USART6, RX_PIN_ROLE },  // PG9  -   "
#endif
      };



//******************************************************************************
//  board_uart_enable_clock
//
//           Ensures that the specified UART's clock is turned on.
//******************************************************************************
int  board_uart_enable_clock (int module_id)
{
      //--------------------------------------------------------
      // Turn on clock for the associated SPIx module.
      //--------------------------------------------------------
    switch (module_id)
     { case 0:                           // USARTMD / VCP
             __USART2_CLK_ENABLE();
             break;
       case 1:                           // USART1
             __USART1_CLK_ENABLE();
             break;
        case 2:                          // USART2
             __USART2_CLK_ENABLE();
             break;
#if defined(STM32F446xx)
        case 3:                          // USART3
             __USART3_CLK_ENABLE();
             break;
        case 4:                          // UART4
             __UART4_CLK_ENABLE();
             break;
        case 5:                          // UART5
             __UART5_CLK_ENABLE();
             break;
#endif
        case 6:                          // USART6
             __USART6_CLK_ENABLE();
             break;
     }
    return (0);          // denote everything worked OK
}



//******************************************************************************
//  board_uart_enable_nvic_irq
//
//           Ensures that the specified IRQ in NVIC turned on for Interrupts.
//******************************************************************************
void  board_uart_enable_nvic_irq (int module_id)
{
    uint32_t   nvic_irq;

      //--------------------------------------------------------
      // Enable the NVIC for the associated UART module
      //--------------------------------------------------------
    switch (module_id)
     { case 0:
             nvic_irq = USART2_IRQn;    // UARTMD = VCP = USART2
             break;
       case 1:
             nvic_irq = USART1_IRQn;
             break;
       case 2:
             nvic_irq = USART2_IRQn;
             break;
#if defined(STM32F446xx)
       case 3:
             nvic_irq = USART3_IRQn;
             break;
       case 4:
             nvic_irq = UART4_IRQn;
             break;
       case 5:
             nvic_irq = UART5_IRQn;
             break;
#endif
       case 6:
             nvic_irq = USART6_IRQn;
             break;
     }

    HAL_NVIC_SetPriority (nvic_irq, 1, 1);
    HAL_NVIC_EnableIRQ (nvic_irq);
}


//*****************************************************************************
// board_get_uart_handle
//
//          Gets the HAL UART_Typedef handle to use for this UART instance
//*****************************************************************************
int  board_get_uart_handle (int module_id, UART_HandleTypeDef **ret_UartHdl)
{
    if (module_id < 0 || module_id > UART_MAX_MODULE)
       return (ERR_WIFI_MODULE_NUM_OUT_OF_RANGE);
    if (_g_uart_module[module_id] == 0L)
       return (ERR_UART_MODULE_NOT_SUPPORTED);

    *ret_UartHdl = (UART_HandleTypeDef*) _g_uart_typedef_handle_addr [module_id];

    return (0);               // denote worked OK
}



//*****************************************************************************
//  board_uart_init
//
//
//       CAUTION: you MUST set _g_UartHandle_x.Init.Parity to UART_PARITY_NONE,
//                otherwise HAL Lib will default to odd parity and turn on the
//                UART CR1 PCE and PS bits, which totally screws things up !
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//*****************************************************************************

int  board_uart_init (int module_id, int tx_gpio_port, int tx_gpio_pin,
                      int rx_gpio_port, int rx_gpio_pin, long baud_rate, int flags)
{
    UART_GPIO_BLK       *gpioTx;
    UART_GPIO_BLK       *gpioRx;
    GPIO_InitTypeDef    GPIO_InitStruct;
    UART_HandleTypeDef  *pUartHdl;
    int                 rc;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

       //-----------------------------------------------------------
       // Locate the associated UART module and GPIO Pin, to locate
       // the associated AF information
       //-----------------------------------------------------------
// brute force for now, add lookup routine in future    TEMP HACK
if (module_id == UARTMD)
 {  gpioTx = (UART_GPIO_BLK*) &_g_uart_2_gpio_table [0];  // PA2 VCP simulated testing USART2
    gpioRx = (UART_GPIO_BLK*) &_g_uart_2_gpio_table [1];  // PA3
 }
else
 {             // USES_FONA808
    gpioTx = (UART_GPIO_BLK*) &_g_uart_1_gpio_table [0];  // D8 TX USART1
    gpioRx = (UART_GPIO_BLK*) &_g_uart_1_gpio_table [1];  // D2 RX
 }

       //--------------------------------------------------
       //                   GPIO  Init
       //
       // Configure the GPIOs that used for the UART pins for VCP
       //   PA.2 = USART2_TX    PA.3 = USART2_RX
       //--------------------------------------------------
    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));    // clear struct
    GPIO_InitStruct.Pin       = gpioTx->uart_gpio_pin;
    GPIO_InitStruct.Alternate = gpioTx->uart_chan_alt_func;   // set Alt Function - UART
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    HAL_GPIO_Init (gpioTx->uart_gpio_port, &GPIO_InitStruct); // Setup UART GPIO TX pin

    GPIO_InitStruct.Pin       = gpioRx->uart_gpio_pin;
    GPIO_InitStruct.Alternate = gpioRx->uart_chan_alt_func;   // set Alt Function - UART
    HAL_GPIO_Init (gpioRx->uart_gpio_port, &GPIO_InitStruct); // Setup UART GPIO RX pin

       //--------------------------------------------------
       // Configure the UART module.  Default = 115200
       //--------------------------------------------------
    board_uart_enable_clock (module_id);               // Turn on UART clock

    memset (pUartHdl, 0, sizeof(UART_HandleTypeDef));  // clear the structure

    pUartHdl->Instance        = (USART_TypeDef*) _g_uart_module[module_id]; // Set UART module to use
    pUartHdl->Init.BaudRate   = baud_rate;
    pUartHdl->Init.WordLength = UART_WORDLENGTH_8B;    // setup as 8N1
    pUartHdl->Init.StopBits   = UART_STOPBITS_1;
    pUartHdl->Init.Parity     = UART_PARITY_NONE;      // no parity  KEY !
    pUartHdl->Init.HwFlowCtl  = UART_HWCONTROL_NONE;   // No flow ctl
    pUartHdl->Init.OverSampling = UART_OVERSAMPLING_16;
    pUartHdl->Init.Mode       = UART_MODE_TX_RX;       // Enable RX and TX

    HAL_UART_Init (pUartHdl);

       //---------------------------------------------
       // UART interrrupts are _always_ used.
       //---------------------------------------------
    board_uart_enable_nvic_irq (module_id);
    _g_uart_do_interrupts = 1;           // denote all UART I/O will use interrupts

    _g_uart_do_echoplex = 1;      // default is to "echo plex", i.e. automatically
                                  // echo back any received character. This is
                                  // typical in Async "Full Duplex" systems.
    if (flags && UART_DO_NOT_ECHOPLEX)
      _g_uart_do_echoplex = 0;    // Turn off auto-echo of received characters

         //-------------------------------------------------
         // prep for any rcv queuing and/or echo-plexing
         //-------------------------------------------------
    _g_uart_rx_head   = _g_uart_rx_tail  = 0;  // reset head and tail ptrs to clr buf

    __HAL_USART_ENABLE_IT (pUartHdl,USART_IT_RXNE); // ALWAYS TURN ON RX interrupts

    return (0);            // completed OK
}


//*****************************************************************************
//  board_uart_rx_flush
//
//             Flush/discards any data left over in receive buffer.
//             Typically used to discard echo-plex characters from FDX devices, etc.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//*****************************************************************************

int  board_uart_rx_flush (int module_id, int flags)
{
    uint8_t            in_byte;
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle (module_id, &pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

                 // reset/clear out the buffer, by resetting internal buf index
    _g_uart_rx_head = _g_uart_rx_tail  = 0;  // reset head/tail ptrs to clr buf

    return (0);   // completed OK
}


//*****************************************************************************
//  board_uart_rx_data_check                 aka   CONSOLE_CHECK_FOR_READ_DATA
//
//             Checks if any pending character input from UART.
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

int  board_uart_rx_data_check (int module_id)
{
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

// ??? return actual amt of data queued instead, that way app has a clue how much rcvd ???

    if (_g_uart_rx_head != _g_uart_rx_tail)
       return (1);                     // yes, we have some data in internal RX buf

    return (0);                        // no, no data has been received
}


//*****************************************************************************
//  board_uart_get_char                                 aka   CONSOLE_GET_CHAR
//
//             Read in a single raw character from the UART.
//
//             if there is no character available, then we loop in here
//             for UART input. Hence, this is a blocking call.
//             Note that no processing (\r \n) and no echo-plexing is performed.
//
//  Returns:  a char  (0-255)
//            or ERR_UART_RCV_TIMED_OUT (-305)
//            or WARN_WOULD_BLOCK       (450)
//            or ERR_xxxMODULE_ID_OUT_OF_RANGE (-300/-301)
//*****************************************************************************

char  board_uart_get_char (int module_id, int flags, int max_wait_time)
{
    char               in_char;
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

return_data_to_user:
   if (_g_uart_rx_head != _g_uart_rx_tail)
      {          // pull a char that is queued in internal RX buffer
        in_char = _g_rx_buf [_g_uart_rx_tail];
        _g_uart_rx_tail = (_g_uart_rx_tail + 1) % UART_INTERNAL_BUF_SIZE;  // update tail ptr
        if (_g_uart_last_char == '\r'  &&  in_char == '\n')
           {      // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
             _g_uart_last_char = 0; // clear out the \r, so we treat any new \n as real.
           }
        return (in_char);           // hand it up to caller
      }

   if (flags & UART_IO_NON_BLOCKING)
      return (WARN_WOULD_BLOCK);

                 //----------------------------------------------
                 // wait until we receive a char or timeout
                 //----------------------------------------------
                         // for TIMEOUT checking, generate expected end time
   _g_uart_end_time = _g_systick_millisecs + max_wait_time;
   while (1)
     {
       if (_g_uart_rx_head != _g_uart_rx_tail)
          goto return_data_to_user;             // we finally got some data
       if (_g_systick_millisecs > _g_uart_end_time)
          return (ERR_UART_RCV_TIMED_OUT);      // hit the TIMEOUT limit - bail !
     }


    return (ERR_UART_RCV_TIMED_OUT);
}


//*****************************************************************************
//  board_process_activation_chars
//
//             Perform CP-V style "activation character" processing for
//             received string/char data:  i.e. if we get a \r or \n, then
//             treat it as an end of string, and hand the current data up to
//             the calling user application.
//             For backspaces, delete the current character in the buffer
//             and back up 1, in the input string.
//
//             This style of processing takes a major burden of the User App
//             programmer, since he will see a "string_data\r\n" as a
//             single record, and does not need to parse the data to find
//             ending \r\n indicators. The app can wait for a complete
//             "record" or string, rather than having to do byte-at-a-time
//             (read_char) processing.
//
//       Returns status to UART_Common_IRQ_Handler which called us
//*****************************************************************************

int  board_uart_process_activation_chars (UART_HandleTypeDef *pUartHdl,
                                          uint8_t in_char, char *read_buf)
{
    int   event;

    event = 0;          // categorize if incoming char is a signficant event
    if (in_char == '\b' || in_char == 0x7F)
       event = 1;                          // backspace event
       else if (in_char == '\r')
               event = 2;                  // carriage return event
       else if (in_char == '\n')
               event = 3;                  // new line event

     switch (event)
       {
         case 0:
               return (0);                 // not an activation char

         case 1:                           // backspace activation char
                  // handle BACKSPACE character - erase previous char by stepping back one.
               if (_g_uart_rx_state == UART_STATE_RCV_BUSY)
                  {       // we are working directly with user buffer via active READ
                    if (_g_rx_amt_rcvd == 0)   // 0 indicates 1st char in buf
                       return (2);             // no chars in buf. discard it
                    _g_rcv_user_buf--;         // backspace rcv buf 1 char
                    _g_rcv_max_length++;
                    return (2);                // discard the backspace
                  }
                 else
                  {       // update our internal RX queue
                    if (_g_uart_rx_head == _g_uart_rx_tail)
                       return (2);             // no chars in buf. discard it
                    if (_g_uart_rx_head == 0)
                       _g_uart_rx_head = UART_INTERNAL_BUF_SIZE;  // handle a wrap around
                       else _g_uart_rx_head--;                    // update head ptr
                    _g_rx_buf [_g_uart_rx_head] = '\0';           // clear out the prev char
                    return (2);                 // discard this char, but allow any echo
                  }
               break;

         case 2:                           // carriage return \r activation char
                          // handle CARRIAGE RETURN char - treat as end of line
               if (_g_uart_rx_state == UART_STATE_RCV_BUSY)
                  {       // we are working directly with user buffer via active READ
                          //----------------------------------------------------------------
                          // Check if \r is the only input char. in such a case, it is
                          // telling the user that he received a null line  (CR only).
                          // For cross-platform usage (Windows vs Unix), we always pass
                          // back a \n\0 sequence, to denote a NULL line rcvd.
                          //----------------------------------------------------------------
                    _g_uart_last_char = '\r';       // save \r, to handle any \r\n sequence check
                    if (_g_rx_amt_rcvd == 0)        // 0 indicates 1st char in buf
                       {
                         *_g_rcv_user_buf++ = '\n'; // move in a \n to denote standalone line (CR only)
                         *_g_rcv_user_buf   = '\0'; // set end of string null terminator
                         return (1);                // tell rupt handler it is end of string/text_line
                       }
                      else {     // strip it off, and denote end of string/text_line
                             *_g_rcv_user_buf   = '\0'; // set end of string null terminator (to an existing string)
                             return (1);     // tell rupt handler it is end of string/text_line
                           }
                  }
                 else
                  {       // update our internal RX queue
                          // convert it to a NL to denote an end of string/text_line
                    read_buf [_g_uart_rx_head] = '\n';   // denote end of line
                    _g_uart_rx_head = (_g_uart_rx_head + 1) % UART_INTERNAL_BUF_SIZE; // update head ptr
                    _g_uart_last_char = '\r';  // save \r, to handle any \r\n sequence check
                    return (2);
                  }
               break;

         case 3:                           // new line \n activation char
               if (_g_uart_rx_state == UART_STATE_RCV_BUSY)
                  {       // we are working directly with user buffer via an active READ
                    if (_g_uart_last_char == '\r')
                       {      // this is the end of a \r\n sequence. Discard the \n
                         _g_uart_last_char = 0;  // clear the \r\n sequence flag
                         if (_g_rx_amt_rcvd > 0)
                            return (1);          // tell rupt handler it is end of text_line
                         return (2);             // otherwise, just discard it.
                       }
                    if (_g_rx_amt_rcvd == 0)               // 0 indicates 1st char in buf
                       {      // Otherwise, this is a standalone \n, without a previous \r.
                              // treat similar to above - return a \n to tell user that he got a null line (NL only)
                         *_g_rcv_user_buf++ = '\n';  // copy it
                         *_g_rcv_user_buf   = '\0';  // set end of string null terminator
                         return (1);                 // tell rupt handler it is end of string/text_line
                       }
                      else {     // we already have a bunch of chars in the buffer
                             *_g_rcv_user_buf++ = '\n'; // copy it, is a standalone \n with no preceding \r
                             *_g_rcv_user_buf   = '\0';    // set end of string null terminator
                             return (1);                   // tell rupt handler it is end of string/text_line
                           }
                  }
                 else
                  {       // update our internal RX queue
                    if (_g_uart_last_char == '\r')
                       { _g_uart_last_char = 0;  // clear \r\n sequence flag
                         return (2);             // discard it. strip the \n when part of a \r\n sequence
                       }
                      else return (0);           // just copy the \n to internal buf
                  }
               break;
       }                                         //   end   switch()

    return (0);                 // denote more string data needs to be read in
}


//*****************************************************************************
//  board_uart_read_bytes
//
//             Reads an arbitrary block of bytes from the UART.
//             _No_ "Activation Character" processing of the datastream is
//             performed and _NO_ echo-plexing performed.
//
//        Returns:   1 if complete    or     ERR_UART_RCV_TIMED_OUT
//                                    or     ERR_xxxMODULE_ID_OUT_OF_RANGE
//*****************************************************************************

int  board_uart_read_bytes (int module_id, char *read_buf, int buf_length,
                            int flags)
{
    uint8_t            in_byte;
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle (module_id, &pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

    _g_uart_is_string_IO = 0; // denote is BINARY I/O, so no Activation Char processing

    rc = board_uart_read_common (pUartHdl, read_buf, buf_length, 0);

    return (rc);
}


//*****************************************************************************
//  board_uart_read_common
//
//           Reads a full string of characters or binary data from the UART.
//
//           For strings, the I/O Interrupt Handler does the following:
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
//             libraries have major bloat-ware in there, so I am avoiding them.
//
//        Returns:   1 if complete    or     ERR_UART_RCV_TIMED_OUT
//
// UART read_Xxxx() routines update only the RX tail pointer for internal RX circular queue.
//*****************************************************************************

int  board_uart_read_common (UART_HandleTypeDef *pUartHdl, char *read_buf,
                             int buf_max_length, int is_string_IO)
{
    char     *begin_buf;
    char     in_char;
    int      rc;
    int      amt_left;

   begin_buf = read_buf;
   _g_rx_amt_rcvd = 0;

               //-----------------------------------------------------------------
               // first, move any data from internal RX queue buffer to user buf.
               //-----------------------------------------------------------------
   amt_left = buf_max_length;
   while (_g_uart_rx_head != _g_uart_rx_tail)
     {                // copy from our internal buf to user app buf
       in_char   = _g_rx_buf [_g_uart_rx_tail];
       *read_buf = in_char;
       amt_left--;
       _g_uart_rx_tail = (_g_uart_rx_tail + 1) % UART_INTERNAL_BUF_SIZE; // update tail ptr
       if (is_string_IO  &&  (in_char == '\n' || amt_left <= 1))
          {        // we hit end of a string.  return this string to user
                   // see if this is a standalone \n  (NL only or CR only)
            if (read_buf == begin_buf)
               {     // this is first char in buf so pass thru the standalone \n
                 read_buf++;        // step to next position in buf
               }                    // otherwise, overlay the trailing \n with \0
            *(read_buf) = '\0';     // ensure null terminator \0 is appended
            _g_uart_last_char = 0;  // clear out any \r indicator
            return (1);             // tell caller we got a string
          }
         else if (amt_left == 0)
                 return (1);        // tell caller we filled buffer
       read_buf++;                  // step to next char in user app buffer
     }
              //----------------------------------
              // there is still more data needed
              //----------------------------------
    _g_rx_amt_rcvd    = buf_max_length - amt_left;
    _g_rcv_max_length = amt_left;         // Save I/O length and user buf ptr
    _g_rcv_user_buf   = read_buf;
    _g_uart_rx_state  = UART_STATE_RCV_BUSY;  // denote I/O in progress

              // for TIMEOUT checking, generate expected end time
    _g_uart_end_time = _g_systick_millisecs + _g_uart_timeout_val;

// in future, call SEMAPHORE in NO_RTOS instead (to allow low power)

    while (_g_uart_rx_state == UART_STATE_RCV_BUSY)
      {       // if there is a Timeout limit, see if we reached it
        if ( _g_uart_timeout_val > 0)
           { if (_g_systick_millisecs > _g_uart_end_time)
                return (ERR_UART_RCV_TIMED_OUT);  // hit the limit - bail !
           }
      }                            //   end   while()

  return (1);                      // tell caller we filled buffer
}


//*****************************************************************************
//  board_uart_read_text_line                           aka   CONSOLE_READ_LINE
//
//              Reads a full line of of characters from the UART.
//              We keep reading until we see a \r\n or \n which denotes
//              the end of a line of text.
//
//        Returns:   1 if complete    or     ERR_UART_RCV_TIMED_OUT
//*****************************************************************************

int  board_uart_read_text_line (int module_id, char *read_buf, int buf_max_length,
                                int flags)
{
    char               in_char;
    int                rc;
    int                amt_left;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

   _g_uart_is_string_IO = 1; // denote is STRING I/O, so enable Activation Char processing

    rc = board_uart_read_common (pUartHdl, read_buf, buf_max_length, 1);

    return (rc);
}


//*****************************************************************************
//  board_uart_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when any UART interrupt completion (or terminating error) occurs.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//*****************************************************************************
int  board_uart_set_callback (int module_id,
                              UART_CB_EVENT_HANDLER callback_function,
                              void *callback_parm)
{
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts
       //-------------------------------------------------------------------
    _g_uart_callback_function = callback_function;
    _g_uart_callback_parm     = callback_parm;

    return (0);                              // indicate it worked OK
}


//*****************************************************************************
//  board_uart_set_echoplex
//
//          Turns echo-plexing of characters received on or off.
//          on_off_flag:  1 = turn on,   0 = turn off
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//*****************************************************************************
int  board_uart_set_echoplex (int module_id, int on_off_flag)
{
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

       //-------------------------------------------------------------------
       // set echoplexing flag as user app reqiuests
       //-------------------------------------------------------------------
    _g_uart_do_echoplex = on_off_flag;

    return (0);                              // indicate it worked OK
}


//*****************************************************************************
//  board_uart_set_max_timeout
//
//          Set the maximum time that the MCU should wait for a UART
//          send/receive transaction to complete.
//          If it exceeds this value, signal an error, and cancel the I/O
//          that was in progress.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//*****************************************************************************
int  board_uart_set_max_timeout (int module_id, uint32_t max_timeout)
{
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle(module_id,&pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

    _g_uart_timeout_val = max_timeout;       // save the receive timeout value we should use

    return (0);                              // indicate it worked OK
}


//******************************************************************************
//  board_uart_write_char                              aka   CONSOLE_WRITE_CHAR
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//******************************************************************************

int  board_uart_write_char (int module_id, char outchar, int flags)
{
    return (board_uart_write_bytes(module_id, &outchar, 1, flags));
}


//******************************************************************************
//  board_uart_write_bytes
//                                                        or    DEBUG_LOG
//
//             Write an arbitrary block of bytes out the UART channel.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//******************************************************************************

int  board_uart_write_bytes (int module_id, uint8_t *bytebuf, int buf_len, int flags)
{
    int                rc;
    UART_HandleTypeDef *pUartHdl;

    rc = board_get_uart_handle (module_id, &pUartHdl);  // get associated HAL UART_Typedef_Handle
    if (rc != 0)
       return (rc);

    if (_g_uart_tx_state < UART_STATE_XMIT_COMPLETE)
       return (ERR_IO_ALREADY_IN_PROGRESS);       // user had started a previous transmit that
                                                  // has not yet completed

    _g_tx_idx    = 0;                             // reset for new pass
    _g_tx_length = buf_len;                       // save I/O length and contents

// ??? NOT NEEDED ?? ==> max length issues
    memcpy (_g_tx_buf, bytebuf, _g_tx_length);

    _g_uart_tx_state = UART_STATE_XMIT_BUSY;      // denote I/O in progress

    pUartHdl->Instance->DR = _g_tx_buf[_g_tx_idx++];  // send first byte in user buf

               // _Enable_ the USART TXE Transmit data register empty Interrupt bit in CR1.
    __HAL_USART_ENABLE_IT (pUartHdl, USART_IT_TXE);
               // and _Enable_ the USART RXNE Interrupt bit in CR1 for echo-plex
    __HAL_USART_ENABLE_IT (pUartHdl, USART_IT_RXNE);

                  //---------------------------------------------------
                  // see if we need to wait for the transmission to complete
                  //--------------------------------------------------------
        if (flags & UART_WAIT_FOR_COMPLETE)
           {      // calling app wants us to wait till I/O is complete
             while (_g_uart_tx_state == UART_STATE_XMIT_BUSY)
               ;
// in future, call SEMAPHORE in NO_RTOS instead (low power)

           }
         else return (0);       // Non-blocking: denote we started transmit I/O OK.
                                // Final status will be via callback/check_complete

    return (0);                 // Transmit I/O completed OK
}


//******************************************************************************
//  board_uart_write_string                               aka   CONSOLE_WRITE
//                                                        or    DEBUG_LOG
//
//             Note this is an absolute BARE BONES implementation, in order
//             to be able to run on small memory MCUs. The vendor's HAL
//             libraries have major bloat-ware in here, so I am avoiding them.
//
//        Returns:   0 if OK    or     ERR_UART_MODULE_NUM_OUT_OF_RANGE
//******************************************************************************

int  board_uart_write_string (int module_id, char *outstr, int flags)
{

    return (board_uart_write_bytes(module_id, outstr, strlen(outstr), flags));
}


//*****************************************************************************
//*****************************************************************************
//
//                            UART      ISRs
//
// ISR routine updates only the RX head pointer for internal RX circular queue.
//*****************************************************************************
//*****************************************************************************
    long  rxne_rupt_count = 0;    // DEBUG COUNTERs
    long  txe_rupt_count  = 0;
    long  tc_rupt_count   = 0;


void  board_common_UART_IRQHandler (USART_TypeDef *uart_module, int uart_module_id)
{
    UART_HandleTypeDef  *pUartHdl;
    uint32_t            rupt_flag;
    uint8_t             in_char;
    int                 next_to_last_byte_idx;
    int                 rc;

    pUartHdl = (UART_HandleTypeDef*) _g_uart_typedef_handle_addr [uart_module_id];

      //----------------------------------------------------------------
      // Determine what flag caused the interrupt:  RXNE, TXE, TC, OVR
      //----------------------------------------------------------------

         //----------------------------------------
         //    Process a receive RXNE
         //----------------------------------------
    rupt_flag = (pUartHdl->Instance->SR & USART_SR_RXNE);
    if (rupt_flag != 0  &&  (pUartHdl->Instance->CR1 & USART_CR1_RXNEIE))  // verify rupt is enabled
       {
rxne_rupt_count++;
                  //------------------------------
                  //     process a rcvd byte.
                  //-------------------------------
                  // Note: reading in the byte automatically clears the RXNE rupt
         in_char = (uint8_t) pUartHdl->Instance->DR;

long_trace[trace_idx] = in_char;          // trace everything to a 512 byte buf
trace_idx = (trace_idx + 1) % UART_TRACE_BUF_SIZE;     // auto-wrap trace index

         rc = 0;
         if (_g_uart_is_string_IO && (in_char <= '\r' || in_char == 0x7F))
            {     //---------------------------------------------------------------------
                  // this is string I/O, so process any special chars like \b \r or \n
                  // it returns 1 of 4 return codes:  0 = process normally,   do any echo
                  //                                  1 = operation complete, do any echo
                  //                                  2 = backspace: discard, do any echo
                  //                                 -3 = other:     discard, do NOT echo
                  //---------------------------------------------------------------------
              rc = board_uart_process_activation_chars (pUartHdl, in_char, _g_rx_buf);
            }

         if (rc == 0)
            { _g_uart_last_char = in_char;    // save char, to handle any \r\n sequence check
                        //------------------------------------------------
                        // determine where to store the received char
                        //------------------------------------------------
              if (_g_uart_rx_state == UART_STATE_RCV_BUSY)
                 {      //------------------------------------------------
                        // we are in the middle of a user app active READ.
                        // copy it directly to the user app's buffer.
                        //------------------------------------------------
                   *_g_rcv_user_buf++ = in_char;     // save char into user rcv buffer
                   _g_rcv_max_length--;              // deduct # bytes left to rcv
                  if (_g_uart_is_string_IO)
                     {     // see if we have hit the max size string/text_line we can put in buf
                       if (_g_rcv_max_length == 1)
                          {    // we've filled the buffer. Return completed buf to caller
                            *_g_rcv_user_buf = '\0'; // append \0 to denote end of string/text_line
                            rc = 1;                  // set condition as "read is complete"
                          }
                     }
                    else if (_g_rcv_max_length == 0) // is a uart_read_bytes() op
                            rc = 1;                  // set condition as "read is complete"
                   _g_rx_amt_rcvd++;                 // inc # bytes rcvd
                 }
                else
                 {      //--------------------------------------
                        //  queue into our internal RX buffer.
                        //--------------------------------------
                  _g_rx_buf [_g_uart_rx_head] = in_char;   // save char into our internal RX queue
                  _g_uart_rx_head = (_g_uart_rx_head + 1) % UART_INTERNAL_BUF_SIZE; // update head ptr
                 }

            }

//  ??? NEED TO HANDLE ECHO-PLEX on  in_char   !!! ???   WVD  !!! ???    (if rc != 3)

// 09/22/15
//    RE-WORK THIS TO SLIDING / ROTATING INTERNAL BUFFER ala ARDUINO, and
//    DISCARD CHARS IF REACH LIMIT, W/FLAG WARNING SET ON ANY USER RCV CALL
//    ALSO, CHANGE uart_read_string() logic to operate as a higher level,
//    that call lower level to just read chars, and upper level processes them
//    Possibly change it to a "uart_console_read() call instead ???

         if (_g_uart_rx_state == UART_STATE_RCV_BUSY  &&  rc == 1)
            {        //-----------------------------------------------------------------------
                     // we are all done with a user app "active READ".
                     // Post the operation complete and issue any callback to user.
                     //
                     // Note: Unlike HAL, we do _NOT_ turn off RX rupts, because with full
                     // duplex devices like SIM808 or ESP8266, there is always a timing window
                     // to bite us. So any subsequent rcvd chars get queued in internal buf.
                     //-----------------------------------------------------------------------
              _g_uart_rx_state = UART_STATE_RCV_COMPLETE;   // set ending status
if (_g_rx_amt_rcvd > 1)    // DEBUG HOOK for strings >  just \n
              _g_rcv_user_buf = 0L;           // clear ptr to denote no more active user READ
              if (_g_uart_callback_function != 0L)
                 {                            // Invoke any user callback for the interrupt
                    (_g_uart_callback_function) (_g_uart_callback_parm,
                                                 uart_module_id, UART_RX_COMPLETE);
                 }
            }
       }                    //  end  if rupt_flag != 0

         //----------------------------------------
         //  Process a transmit TXE buffer empty
         //----------------------------------------
    rupt_flag = pUartHdl->Instance->SR & USART_SR_TXE;
    if (rupt_flag != 0 && (pUartHdl->Instance->CR1 & USART_CR1_TXEIE))
       {    // process a transmitted byte. Send next char if there is one.
            // Note: transmitting a new byte automatically clears the TXE rupt.
txe_rupt_count++;
         if (_g_tx_idx < _g_tx_length)
            pUartHdl->Instance->DR = _g_tx_buf [_g_tx_idx++];  // send next byte in user buf
         if (_g_tx_idx == _g_tx_length)
            {     // we are all done sending. Turn off TXE interrupt, and wait for TC interrupt
                  // to signal that the last byte's bits have cleared the transmitter.
                  // _Disable_ the USART TXE Transmit data register empty Interrupt bit in CR1.
              __HAL_USART_DISABLE_IT (pUartHdl, USART_IT_TXE);
                  // _Enable_ the USART TC Transmit Complete Interrupt bit in CR1
              __HAL_USART_ENABLE_IT (pUartHdl, USART_IT_TC);
            }
       }

         //--------------------------------------------
         // Process a transmit TC - transmit complete
         //--------------------------------------------
    rupt_flag = pUartHdl->Instance->SR & USART_SR_TC;
    if (rupt_flag != 0)
      { if (pUartHdl->Instance->CR1 & USART_CR1_TCIE)
           {      // process a transmitted byte. Send next char if there is one.
tc_rupt_count++;
            if (_g_tx_idx == _g_tx_length)
               {     // we are all done sending. Post operation complete and
                     // issue any callback to user
                     // _Disable_ the USART TC Transmit Complete Interrupt bit in CR1.
                 __HAL_USART_DISABLE_IT (pUartHdl, USART_IT_TC);
                     // Disable any USART Error Interrupts: (Frame error, noise error, overrun error)
////             __HAL_USART_DISABLE_IT (pUartHdl, USART_IT_ERR);

                 _g_uart_tx_state = UART_STATE_XMIT_COMPLETE;  // set ending status
                 if (_g_uart_callback_function != 0L)
                    {             // Invoke user callback for the interrupt
                      (_g_uart_callback_function) (_g_uart_callback_parm,
                                              uart_module_id, UART_TX_COMPLETE);
                    }
               }
           }
        pUartHdl->Instance->DR = 0;  // per tech ref, clear TC flag by writing dummy byte to DR
      }
}                                    //   end   board_common_UART_IRQHandler()


//#if (USES_UART)                 // THIS MOVES TO stm32xx_it.c with (USES_UART) enabled
void USART1_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART1, 1);
}


void USART2_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART2, 2);
}


void USART6_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART6, 6);
}
//#endif                                       // (USES_UART)

#endif                           // SHUTOFF_UART_RTNS





#if defined(USES_UNIQUEID) || defined(USES_MQTT)

//*****************************************************************************
//*****************************************************************************
//
//                           UNIQUE-ID  /  CRC   Routines
//
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

    memset (&CrcHandle, 0, sizeof(CrcHandle));  // clear struct

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

/******************************************************************************/
