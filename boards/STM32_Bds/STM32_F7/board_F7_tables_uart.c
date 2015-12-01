
//2******1*********2*********3*********4*********5*********6*********7**********
//
//                                board_F7_tables_uart.c
//
//
//                            STM32  F7  46  NG        Discovery     STM32F746xx / USE_STM32746G_DISCOVERY
//
//
//             MCU SPECIFIC   UART  Tables  and  GPIO  PIN   DEFINITIONS
//
//                                    for
//
//                                 UART Support
//
//  History:
//    09/24/15 - Verified tables. Duq
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

#include "user_api.h"                   // pull in API defs and MCU depdent defs
#include "device_config_common.h"
#include "boarddef.h"


//------------------------------------------------------------------------------
//                           UART Pinout Definitions
//
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

//-----------------------------------------
// MCU DEPENDENT
//-----------------------------------------
#define  UART_MAX_MODULE       8

const  USART_TypeDef   *_g_uart_HW_module [] =
        {    USART1,              // [0] = default module = UARTMD / VCP
             USART1,              // [1] = UARTCM1
             USART2,              // [2] = UARTM2
             USART3,              // [3] = UARTM3
             UART4,               // [4] = UARTM4
             UART5,               // [5] = UARTM5
             USART6,              // [6] = UARTM6
             UART7,               // [7] = UARTM7
             UART8,               // [8] = UARTM8
        };

                            //----------------------------------------
                            //   UARTn handles needed, one per module
                            //----------------------------------------
    UART_HandleTypeDef  _g_UartHandle_1;           // UART 1 support (also VCP)
    UART_HandleTypeDef  _g_UartHandle_2;           // UART 2 support
    UART_HandleTypeDef  _g_UartHandle_3;           // UART 3 support
    UART_HandleTypeDef  _g_UartHandle_4;           // UART 4 support
    UART_HandleTypeDef  _g_UartHandle_5;           // UART 5 support
    UART_HandleTypeDef  _g_UartHandle_6;           // UART 6 support
    UART_HandleTypeDef  _g_UartHandle_7;           // UART 7 support
    UART_HandleTypeDef  _g_UartHandle_8;           // UART 8 support

                            //----------------------------------------------------------
                            // List of HAL Handle Addresses, one for each separate UART Handle
                            //----------------------------------------------------------
const UART_HandleTypeDef *_g_uart_typedef_handle_addr []          // HAL API Handles
                     = { (UART_HandleTypeDef*) &_g_UartHandle_1,  // UARTMD / VCP
                         (UART_HandleTypeDef*) &_g_UartHandle_1,  // UARTM1
                         (UART_HandleTypeDef*) &_g_UartHandle_2,  // UARTM2
                         (UART_HandleTypeDef*) &_g_UartHandle_3,  // UARTM3
                         (UART_HandleTypeDef*) &_g_UartHandle_4,  // UARTM4
                         (UART_HandleTypeDef*) &_g_UartHandle_5,  // UARTM5
                         (UART_HandleTypeDef*) &_g_UartHandle_6,  // UARTM6
                         (UART_HandleTypeDef*) &_g_UartHandle_7,  // UARTM7
                         (UART_HandleTypeDef*) &_g_UartHandle_8,  // UARTM8
                       };

                       //-------------------------------------------------------
                       // UART I/O Buffer Control blocks needed, one per module
                       //-------------------------------------------------------
    IO_BUF_BLK      _g_uart_1_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_2_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_3_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_4_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_5_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_6_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_7_io_blk = { 0 };
    IO_BUF_BLK      _g_uart_8_io_blk = { 0 };

                        //----------------------------------------------------------
                        // List of Addresses, one for each separate UART IO_BLK
                        //----------------------------------------------------------
const IO_BUF_BLK    *_g_uart_io_blk_address []   // UART I/O Block addresses
                       = { (IO_BUF_BLK*) &_g_uart_1_io_blk,  // UART1 / VCP default
                           (IO_BUF_BLK*) &_g_uart_1_io_blk,  // UART1
                           (IO_BUF_BLK*) &_g_uart_2_io_blk,  // UART2
                           (IO_BUF_BLK*) &_g_uart_3_io_blk,  // UART3
                           (IO_BUF_BLK*) &_g_uart_4_io_blk,  // UART4
                           (IO_BUF_BLK*) &_g_uart_5_io_blk,  // UART5
                           (IO_BUF_BLK*) &_g_uart_6_io_blk,  // UART6 / VCP on F7 Discovery
                           (IO_BUF_BLK*) &_g_uart_7_io_blk,  // UART7
                           (IO_BUF_BLK*) &_g_uart_8_io_blk,  // UART8
                         };



typedef struct uart_gpio_def              /* UART GPIO definitions */
    {
        GPIO_TypeDef  *uart_gpio_port;     /* Associated GPIO port       */
        uint16_t      uart_gpio_pin;       /* Associated GPIO pin        */
        uint32_t      uart_chan_alt_func;  /* UART Alternate Function Id */
        int8_t        uart_pin_role;       /* acts in TX or RX role      */
        uint8_t       uart_pin_number;     /* logical pin number 0-255   */
    } UART_GPIO_BLK;

#define TX_PIN_ROLE   1
#define RX_PIN_ROLE   2

               //--------------------------------------------------------------
               //                       GPIO   MAPPING   TABLES
               //
               // These tables contain the GPIO mapping for every GPIO pin that
               // can be used for Input (Input Capture) or Output (PWM or
               // Timer Output Compare) by the Timer/PWM modules.
               // pin each alternative is assigned to (e.g. PA8, PA7, or PB13)
               //
               // F7 Discovery VCP pins are: TX = PA9,  RX = PB7  on USART1
               //--------------------------------------------------------------
const UART_GPIO_BLK  _g_uart_1_gpio_table [] =
      {
        { GPIOA, GPIO_PIN_9,  GPIO_AF7_USART1, TX_PIN_ROLE, PA9  }, // PA9  D8   VCP
        { GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, RX_PIN_ROLE, PA10 }, // PA10 D2
        { GPIOB, GPIO_PIN_6,  GPIO_AF7_USART1, TX_PIN_ROLE, PB6  }, // PB6  D10
        { GPIOB, GPIO_PIN_7,  GPIO_AF7_USART1, RX_PIN_ROLE, PB7  }, // PB7       VCP
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_2_gpio_table [] =
      {
        { GPIOA, GPIO_PIN_2,  GPIO_AF7_USART2, TX_PIN_ROLE, PA2  }, // PA2
        { GPIOA, GPIO_PIN_3,  GPIO_AF7_USART2, RX_PIN_ROLE, PA3  }, // PA3
        { GPIOD, GPIO_PIN_5,  GPIO_AF7_USART2, TX_PIN_ROLE, PD5  }, // PD5  not routed out to CN pins
        { GPIOD, GPIO_PIN_6,  GPIO_AF7_USART2, RX_PIN_ROLE, PD6  }, // PD6   "     "    "      "   "
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_3_gpio_table [] =
      {
        { GPIOB, GPIO_PIN_10, GPIO_AF7_USART3, TX_PIN_ROLE, PB10 }, // PB10
        { GPIOB, GPIO_PIN_11, GPIO_AF7_USART3, RX_PIN_ROLE, PB11 }, // PB11
        { GPIOC, GPIO_PIN_10, GPIO_AF7_USART3, TX_PIN_ROLE, PC10 }, // PC10
        { GPIOC, GPIO_PIN_11, GPIO_AF7_USART3, RX_PIN_ROLE, PC11 }, // PC11
        { GPIOD, GPIO_PIN_8,  GPIO_AF7_USART3, TX_PIN_ROLE, PD8  }, // PD8  not routed out to CN pins
        { GPIOD, GPIO_PIN_9,  GPIO_AF7_USART3, RX_PIN_ROLE, PD9  }, // PD9   "     "    "      "   "
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_4_gpio_table [] =
      {
        { GPIOA, GPIO_PIN_0,  GPIO_AF8_UART4, TX_PIN_ROLE, PA0  }, // PA0
        { GPIOA, GPIO_PIN_1,  GPIO_AF8_UART4, RX_PIN_ROLE, PA1  }, // PA1
        { GPIOC, GPIO_PIN_10, GPIO_AF8_UART4, TX_PIN_ROLE, PC10 }, // PC10
        { GPIOC, GPIO_PIN_11, GPIO_AF8_UART4, RX_PIN_ROLE, PC11 }, // PC11

        { GPIOE, GPIO_PIN_8,  GPIO_AF8_UART4, TX_PIN_ROLE, PE8  }, // PE8  not routed out to CN pins
        { GPIOE, GPIO_PIN_9,  GPIO_AF8_UART4, RX_PIN_ROLE, PE9  }, // PE9   "     "    "      "   "
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_5_gpio_table [] =
      {
        { GPIOC, GPIO_PIN_12, GPIO_AF8_UART5, TX_PIN_ROLE, PC12 }, // PC12
        { GPIOD, GPIO_PIN_2,  GPIO_AF8_UART5, RX_PIN_ROLE, PD2  }, // PD2
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_6_gpio_table [] =
      {
        { GPIOC, GPIO_PIN_6,  GPIO_AF8_USART6, TX_PIN_ROLE, PC6  }, // PC6
        { GPIOC, GPIO_PIN_7,  GPIO_AF8_USART6, RX_PIN_ROLE, PC7  }, // PC7
        { GPIOG, GPIO_PIN_14, GPIO_AF8_USART6, TX_PIN_ROLE, PG14 }, // PG14 not routed out to CN pins
        { GPIOG, GPIO_PIN_9,  GPIO_AF8_USART6, RX_PIN_ROLE, PG9  }, // PG9   "     "    "      "   "
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_7_gpio_table [] =
      {
        { GPIOE, GPIO_PIN_8,  GPIO_AF8_UART7, TX_PIN_ROLE, PE8  }, // PE8
        { GPIOE, GPIO_PIN_7,  GPIO_AF8_UART7, RX_PIN_ROLE, PE7  }, // PE7
        { GPIOF, GPIO_PIN_7,  GPIO_AF8_UART7, TX_PIN_ROLE, PF7  }, // PF7
        { GPIOF, GPIO_PIN_6,  GPIO_AF8_UART7, RX_PIN_ROLE, PF6  }, // PF6
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


const UART_GPIO_BLK  _g_uart_8_gpio_table [] =
      {
        { GPIOE, GPIO_PIN_1,  GPIO_AF8_UART8, TX_PIN_ROLE, PE1  }, // PE1
        { GPIOE, GPIO_PIN_0,  GPIO_AF8_UART8, RX_PIN_ROLE, PE0  }, // PE0
        {   0L,       0,              0,             -1,     0   }  // end of table
      };


               //--------------------------------------------------------------
               //  Lookup table for valid RX/TX combos for each different UART
               //--------------------------------------------------------------
const UART_GPIO_BLK *  _g_uart_gpio_table_addr[] =
      { (UART_GPIO_BLK*) &_g_uart_1_gpio_table,  // UART 1 / VCP is default module for F7
        (UART_GPIO_BLK*) &_g_uart_1_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_2_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_3_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_4_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_5_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_6_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_7_gpio_table,
        (UART_GPIO_BLK*) &_g_uart_8_gpio_table,
      };



//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************


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
             __USART1_CLK_ENABLE();
             break;
       case 1:                           // USART1
             __USART1_CLK_ENABLE();
             break;
        case 2:                          // USART2
             __USART2_CLK_ENABLE();
             break;
        case 3:                          // USART3
             __USART3_CLK_ENABLE();
             break;
        case 4:                          // USART4
             __UART4_CLK_ENABLE();
             break;
        case 5:                          // USART5
             __UART5_CLK_ENABLE();
             break;
        case 6:                          // USART6
             __USART6_CLK_ENABLE();
             break;
        case 7:                          // USART7
             __UART7_CLK_ENABLE();
             break;
        case 8:                          // USART8
             __UART8_CLK_ENABLE();
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
             nvic_irq = USART1_IRQn;    // UARTMD = VCP = USART6 on F7
             break;
       case 1:
             nvic_irq = USART1_IRQn;
             break;
       case 2:
             nvic_irq = USART2_IRQn;
             break;
       case 3:
             nvic_irq = USART3_IRQn;
             break;
       case 4:
             nvic_irq = UART4_IRQn;
             break;
       case 5:
             nvic_irq = UART5_IRQn;
             break;
       case 6:
             nvic_irq = USART6_IRQn;
             break;
       case 7:
             nvic_irq = UART7_IRQn;
             break;
       case 8:
             nvic_irq = UART8_IRQn;
             break;
     }

    HAL_NVIC_SetPriority (nvic_irq, 1, 1);
    HAL_NVIC_EnableIRQ (nvic_irq);
}



//******************************************************************************
//  board_uart_ALTFUNC_lookup
//
//        Finds/scans for the associated GPIO Port and GPIO Pin, as well as
//        the associated ALTERNATE FUNCTION (AF) information.
//        Returns ptrs to the _g_uart_NN_gpio_table[] entries for TX and RX pins
//******************************************************************************

int  board_uart_ALTFUNC_lookup (int module_id, int tx_gpio_pin, uint32_t *tx_AltFuncId,
                                int rx_gpio_pin, uint32_t *rx_AltFuncId)
{
    UART_GPIO_BLK   *gpio_blk_base;
    UART_GPIO_BLK   *gpio_curr_ptr;

    gpio_blk_base = (UART_GPIO_BLK*) _g_uart_gpio_table_addr [module_id];
    if (gpio_blk_base == 0L)
       return (ERR_UART_MODULE_NOT_SUPPORTED);

    *tx_AltFuncId = 0xFFFF;      // initially clear out the entries
    *rx_AltFuncId = 0xFFFF;

       // first, scan to locate the associated TX pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->uart_pin_role != -1)
      { if (gpio_curr_ptr->uart_pin_number == tx_gpio_pin)
           { if (gpio_curr_ptr->uart_pin_role == TX_PIN_ROLE)
                { *tx_AltFuncId = gpio_curr_ptr->uart_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                       // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                       // else step to the next entry
      }

       // then, scan to locate the associated RX pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->uart_pin_role != -1)
      { if (gpio_curr_ptr->uart_pin_number == rx_gpio_pin)
           { if (gpio_curr_ptr->uart_pin_role == RX_PIN_ROLE)
                { *rx_AltFuncId = gpio_curr_ptr->uart_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                       // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                       // else step to the next entry
      }

    if (*tx_AltFuncId == 0xFFFF || *rx_AltFuncId == 0xFFFF)
       return (ERR_UART_PIN_ID_NOT_SUPPORTED);   // no match found in table

    return (0);                // denote operation completed successfully
}
/******************************************************************************/
