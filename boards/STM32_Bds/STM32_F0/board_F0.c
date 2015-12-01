
/********1*********2*********3*********4*********5*********6*********7**********
*
*                       board_F0.c - STM32 F0
*
*
* STM32Cube / HAL based version of board_xxx() functions
*
*
* Supported/Tested Boards
* -----------------------
*      STM32_F0_91RC     Nucleo-F0     256K Flash     32K RAM     48 MHz
*
*  History:
*    12/15/14 - Created for Industrial IoT OpenSource project.  Duquaine
*    06/08/15 - Integrate in ADC, PWM, CRC changes to match rest of STM32 bds.
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
********************************************************************************

#include "user_api.h"
//#include "boarddef.h"
#include <math.h>

#include "stm32f0xx_hal.h"


// move the following to boarddef.h at some point
void board_system_clock_config (long mcu_clock_hz, int option_flags);     // internal Function protos
int  board_dac_check_conversions_done (int dac_module_id, int sequencer);
int  board_dac_clear_conversions_done (int dac_module_id, int sequencer);
int  board_dac_SineWave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_Trianglewave (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_dac_SquareWave  (int dac_module_id, int channel_num, long hz_frequency, long sps, int flags);
int  board_timerpwm_channel_lookup (int chan_id, int *chan_index);
int  board_timerpwm_enable_clock (int module_id);
long board_timerpwm_compute_prescalar (long period_val);
int  board_timerpwm_config_gpios (void *pwmblkp, int chan_id);

// internal Function protos
uint32_t HAL_GetTick (void);   // overrides for HAL Systick functions
void     HAL_IncTick (void);

void board_error_handler(void);
void  DMA1_Channel1_IRQHandler (void);               // ADC DMA ISR - all channels
void  DMA1_Channel2_3_IRQHandler (void);             // DAC DMA ISR - Channel 1
void  DMA1_Channel4_5_IRQHandler (void);             // DAC DMA ISR - Channel 2

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

extern  uint32_t       _g_SysClk_Ticks;

    long               irq_state;
    long               ulSpiIRQState;
    long               brc;

    short              IRQ_interrupts_enabled = -1;

    unsigned char      Hci_Link_Start []   = { 0x01,0x00,0x05,0x00,0x00,
                                               0x01,0x00,0x40,0x01,0x00  };
    unsigned char      Hci_Link_Reply [10] = { 0,0,0,0,0,0,0,0,0,0 };


//#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    char   _g_uart_buf_idx   = 0;        // keeps track of user input via UART
    char   _g_uart_last_char = 0;
    UART_HandleTypeDef   _g_UartHandle;
//#endif

#if defined(USES_UNIQUEID) || defined(USES_MQTT)
    unsigned long   _g_crcid_Result;
#endif


               //----------------------------------------------
               //  Lookup table to obtain GPIO Port Addresses
               //----------------------------------------------
#define  GPIO_MAX_PIN_ID     PF10          /* F0_30 / F0_70 max is PF1 */

const  GPIO_TypeDef * _g_gpio_base[] =
    {
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
//#if defined(STM32F072xB) || defined(STM32F091xC)
        GPIOE,
        GPIOF
//#endif
    };

#define  GP_PORT_A   0        // indexes into above table, based on port id
#define  GP_PORT_B   1
#define  GP_PORT_C   2
#define  GP_PORT_D   3
#define  GP_PORT_E   4
#define  GP_PORT_F   5

#if OLD_CODE
const  GPIO_TypeDef * _g_gpio_base[] =
    {
        0L,
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
//#if defined(STM32F072xB) || defined(STM32F091xC)
        GPIOE,
        GPIOF
//#endif
    };

#define  GP_PORT_A   1        // indexes into above table, based on port id
#define  GP_PORT_B   2
#define  GP_PORT_C   3
#define  GP_PORT_D   4
#define  GP_PORT_E   5
#define  GP_PORT_F   6
#endif                        // OLD_CODE

extern const  uint32_t  _g_gpio_pin_num[];
extern const  int       _g_gpio_pull_flags[];



//*****************************************************************************
//*****************************************************************************
//
//           MCU Dependent         GPIO   Routines
//
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
*  board GPIO Init
*
*          Configure Port Directions and Peripherals as needed.
*******************************************************************************/
void  board_gpio_init (void)
{
    __GPIOA_CLK_ENABLE();             // turn on GPIO clocks for Ports A, B, C
    __GPIOB_CLK_ENABLE();             // since they are used in many spots
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
//#if defined(STM32F072xB) || defined(STM32F091xC)
    __GPIOE_CLK_ENABLE();
    __GPIOF_CLK_ENABLE();
//#endif

}


//*****************************************************************************
//*****************************************************************************
//
//           MCU Dependent     System CPU / Systick   Routines
//
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_stop_WDT
//*****************************************************************************

void  board_stop_WDT (void)
{
      // chip default is it comes up with WWDG and IWDG in reset state
      // This is done in SystemInit(), which is part of the CMSIS startup logic.
}


/*******************************************************************************
* System Clock Configuration
*
*            System Clock source            = PLL (HSI/2)
*            SYSCLK(Hz)                     = 48000000        (48 MHz)
*            HCLK(Hz)                       = 48000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            HSI Frequency(Hz)              = 8000000
*            PREDIV                         = 1
*            PLLMUL                         = 12
*            Flash Latency(WS)              = 1
*******************************************************************************/
void  board_system_clock_config (long  mcu_clock_hz, int option_flags)
{
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;
    RCC_OscInitTypeDef  RCC_OscInitStruct;
    int                 rc;

        /* There is no HSE Oscillator on Nucleo, so Activate PLL with HSI/2 as source - R8 */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
//  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PREDIV     = RCC_PREDIV_DIV1;
//  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL12;
//  rc = HAL_RCC_OscConfig (&RCC_OscInitStruct);

    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = 48000000;    // use default clock rate = 48 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

        /* No HSE Oscillator on Nucleo, Activate PLL with HSI as source - RB */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PREDIV          = RCC_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
    rc = HAL_RCC_OscConfig (&RCC_OscInitStruct);
    if (rc != HAL_OK)
       {
         board_error_handler();    // 06/03/15 - THIS WAS OCCURRING ON F0 NUCLEO ! because it was trying to use HSI with crystal
       }
        /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    rc = HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_1);
    if (rc != HAL_OK)
       {
         board_error_handler();
       }
}


#if SHUTOFF_UART
//*****************************************************************************
//*****************************************************************************
//
//                             VCP   UART   Routines
//
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_uart_init
//
//          CAUTION: you MUST set _g_UartHandle.Init.Parity to UART_PARITY_NONE,
//                   else HAL Lib will default to odd parity and turn on the
//                   UART CR1 PCE and PS bits, whioh totally screws things up !
//*****************************************************************************

void  board_uart_init (long  baud_rate)
{
    // see Clive Example that works fine  (uses old Std Lib, not HAL)

//#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    GPIO_InitTypeDef    GPIO_InitStruct;

       //--------------------------------------------------
       // Configure the GPIOs that used for the UART pins
       //   Most Nucleos use:
       //       PA.2 = USART2_TX    PA.3 = USART2_RX
       //   But F0_72 Discovery uses:
       //       PA.9 = USART1_TX    PA.10 = USART1_RX
       //--------------------------------------------------
#if defined(USE_STM32F072B_DISCO)
    GPIO_InitStruct.Pin        = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Alternate  = GPIO_AF1_USART1;     // set Alt Function - UART
#else
    GPIO_InitStruct.Pin        = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Alternate  = GPIO_AF1_USART2;     // set Alt Function - UART
#endif
    GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull       = GPIO_PULLUP;
    GPIO_InitStruct.Speed      = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);          // Setup UART GPIO pins

       //--------------------------------------------------
       // Configure the UART module.  Default = 115200
       //--------------------------------------------------
    __USART1_CLK_ENABLE();                          // Turn on UART clocks
    __USART2_CLK_ENABLE();                          // Turn on UART clocks

#if defined(USE_STM32F072B_DISCO)
    _g_UartHandle.Instance        = USART1;         //   Set UART module to use
#else
    _g_UartHandle.Instance        = USART2;         //   Set UART module to use
#endif
    _g_UartHandle.Init.BaudRate   = baud_rate;
    _g_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;     // setup as 8N1
    _g_UartHandle.Init.StopBits   = UART_STOPBITS_1;
    _g_UartHandle.Init.Parity     = UART_PARITY_NONE;       // no parity  KEY !
    _g_UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;    // No flow ctl
    _g_UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    _g_UartHandle.Init.Mode       = UART_MODE_TX_RX;        // Enable RX and TX

    HAL_UART_Init (&_g_UartHandle);
//#endif

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
//#if defined(USES_CONSOLE_READ)
    int  rc;

          // see if any rcvd chars are in UART
    rc = (_g_UartHandle.Instance->ISR & USART_ISR_RXNE);
    if (rc == RESET)
       return (0);                         // no, UART data is present

    return (1);                            // yes, we have some data
//#endif

}


//*****************************************************************************
//  board_uart_get_char                                 aka   CONSOLE_GET_CHAR
//
//             Read in a single raw character from the UART.
//
//             if there is no character available, then we loop in here
//             for UART input. Hence, this is a blocking call.
//*****************************************************************************

char  board_uart_get_char (int max_wait_time)
{

//#if defined(USES_CONSOLE_READ) || defined(USES_CONSOLE_WRITE)
    char  in_char;
    int   rc;
                 //----------------------------------------------
                 // read in any character that the user typed in
                 //----------------------------------------------
retry_uart_read:
          // see if any rcvd chars are in UART
/// rc = USART_GetFlagStatus (USART2, USART_FLAG_RXNE);
    rc = (_g_UartHandle.Instance->ISR & USART_ISR_RXNE);
    if (rc == RESET)
       {       // no char in RX buffer
         if (max_wait_time)
            goto retry_uart_read;         // user wants to wait till get a reply
            else return (0);              // tell user no UART data is present
       }

///  in_char = USART_ReceiveData (USART2);  // read in char from UART
    in_char = (char) (_g_UartHandle.Instance->RDR & (uint8_t) 0x007F);

//#if defined(USES_CONSOLE_READ)
	// Check if a previous board_uart_read_string() had left a dangling \r\n situation.
    if (_g_uart_last_char == '\r'  &&  in_char == '\n')
       {      // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
         _g_uart_last_char = 0;     // clear out the \r, so we treat any
                                    // new \n as real.
         return (0);                // "nothing to see here"
       }

    _g_uart_last_char = 0;          // always clear out any old \r condition
//#endif

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

//#if defined(USES_CONSOLE_READ)
    char  in_char;
    int   rc;
                  //---------------------------------------------
                  // loop until we get \r, \n, or a full buffer
                  //---------------------------------------------
  while (1)
    {
           // read in any character that the user typed in.
           // first - see if any rcvd chars are in UART
     rc = (_g_UartHandle.Instance->ISR & USART_ISR_RXNE);
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
//#endif

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
    int    rc;

    rc = RESET;
    while (rc == RESET)
       rc = (_g_UartHandle.Instance->ISR & USART_ISR_TXE); // Wait for TX buf Empty

    _g_UartHandle.Instance->TDR = (outchar & (uint8_t )0xFF);   // Send the Char
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

//#if defined(USES_CONSOLE_WRITE) || defined(USES_DEBUG_LOG)
    int    rc;

    while (*outstr != '\0')
      {
        rc = RESET;
        while (rc == RESET)
           rc = (_g_UartHandle.Instance->ISR & USART_ISR_TXE); // Wait for TX buf Empty

        _g_UartHandle.Instance->TDR = (*outstr & (uint8_t )0xFF);   // Send the Char

        outstr++;                                 // step to next char in buffer
      }
//#endif

}
#endif                          // SHUTOFF_UART



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

            // Turn on clock for CRC peripheral
    __CRC_CLK_ENABLE();

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
