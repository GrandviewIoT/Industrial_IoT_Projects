// 07/15/15   ADC still partly broke. Is it got an incorrect count/EOS setting that is screwing up the DMA,
//            or is DMA count off ???

//2******1*********2*********3*********4*********5*********6*********7**********
//
//                             board_F7.c - STM32 F7
//
//
// STM32Cube / HAL based version of board_xxx() functions
//
// NOTE: for this initial release, it only handles the key functions (SPI, I2C,
//       Timers, ...) that are pinned out to the "Arduino" headers.
//       Later releases will include wider support for the rest of the peripherals
//       on the board.
//
//
// Supported/Tested Boards
// -----------------------
//      STM32_F7_46_NG    Discovery   1024K Flash     96K RAM   216 MHz
//                                        3 ADC @ 16 channels
//                                        3 SPI       3 I2C       1 USB
//
//  History:
//    07/13/15 - Created for Industrial IoT OpenSource project.  Duq
//    07/15/15 - Convert over to use USART1 for the Virtual Com port on F7 Disco.
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

#include "stm32f7xx_hal.h"


#if defined(USES_CC3000)
long hci_unsolicited_event_handler(void);
#define  FALSE   0
#define  TRUE    1
#endif

// move the following to boarddef.h at some point
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

extern  uint32_t       _g_SysClk_Ticks;

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
#define  GPIO_MAX_PIN_ID     PK7           /* F7_46 max is PK7 */

const  GPIO_TypeDef * _g_gpio_base[] =
    {
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
        GPIOE,
        GPIOF,
        GPIOG,
        GPIOH,
        GPIOI,
        GPIOJ,
        GPIOK
    };
#define  GP_PORT_A   0        // indexes into above table, based on port id
#define  GP_PORT_B   1
#define  GP_PORT_C   2
#define  GP_PORT_D   3
#define  GP_PORT_E   4
#define  GP_PORT_F   5
#define  GP_PORT_G   6
#define  GP_PORT_H   7
#define  GP_PORT_I   8
#define  GP_PORT_J   9
#define  GP_PORT_K  10


    int              _g_trigger_atmrpwm      = 0; // index to correct Timer/PWM
    uint16_t         _g_trigger_auser_api_id = 0; // User API id for the trigger
    uint16_t         _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
    uint32_t         _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

#if PROBLEMCHILD
//------------------------------------------------------------------
//         TEMP KLUDGE TO HANDLE INLINE BARF ISSUE ON F7 compiles
//------------------------------------------------------------------
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
#endif




//*****************************************************************************
//*****************************************************************************
//
//      MCU Dependent          GPIO   Routines
//
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

}



//*****************************************************************************
//*****************************************************************************
//
//      MCU Dependent          System CPU / Systick   Routines
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
void  board_system_clock_config (long  mcu_clock_hz, int option_flags)
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



#if MOVED_TO_COMMON_CODE
//*****************************************************************************
//*****************************************************************************
//
//                            VCP    UART    Routines
//
//
//        The VCP (Virtual Comm Port), is the default UART TX/RX pins connected
//        from the Nucleo or Discovery board to the onboard ST-Link component,
//        that provides serial UART emulation to a PC via USB CDC protocol.
//        On Nucleos, this is wired to the "Arduino" D1/D0 pins on the bottom
//        right Arduino connector. These pins are effectively dedicated to
//        the VCP function, and will not work as GPIOs, unless you re-solder
//        the solder bridges on the Nucleo (in which case, you lose VCP support)
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

void  board_uart_init (long baud_rate)
{
    // see Clive Example that works fine  (uses old Std Lib, not HAL)

//#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    GPIO_InitTypeDef    GPIO_InitStruct = { 0 };

       //--------------------------------------------------
       // Configure the GPIOs that used for the UART pins
       // PA.9 = USART1_TX    PB.7 = USART1_RX

// CAUTION: F7 Discovery and Nucelo _DOES_ use different UASRTs and different TX/RX pins

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
    rc = (_g_UartHandle.Instance->ISR & UART_FLAG_RXNE);
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
    if (rc == RESET)
      {    // RXNE == RESET  => no received character in UART
        if (max_wait_time)
           goto retry_uart_read;          // user wants to wait till get a reply
           else return (0);               // tell user no UART data is present
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
       rc = (_g_UartHandle.Instance->ISR & UART_FLAG_TXE); // Wait for TX buf Empty

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
           rc = (_g_UartHandle.Instance->ISR & UART_FLAG_TXE); // Wait for TX buf Empty

        _g_UartHandle.Instance->TDR = (*outstr & (uint8_t )0xFF);   // Send the Char

        outstr++;                                 // step to next char in buffer
      }
//#endif

}
#endif                           // MOVED_TO_COMMON_CODE



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
