
/********1*********2*********3*********4*********5*********6*********7**********
*
*                                  board_STM32_uart.c
*
*
*   Common Logic for UART support for STM32 MCUs.
*
*   Uses a combination of HAL_Library (mostly for configuration) and direct
*   register calls (main execution).
*
*   Specific chip dependences are mainly constrained to tables (GPIOs used
*   specific UART modules supported, etc). They are pulled in as #includes
*   based on MCU type.
*
*   In general, the use of #ifdefs within the code has been minimized, in
*   order to keep things read-able and maintainable.
*
*   Note: _All_ UART Read/Write routines are interrupt based.
*         HAL libraries calls are only used for GPIO Init and UART Init.
*
*
*
*  History:
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
*   09/24/15 - Re-factored for easier cross-MCU support. Duq
*   11/01/15 - Add more checks for RX errors that can lead to persistent ORE
*              errors, causing looping in the UART common ISR.
*              And clear associated ICR for those platforms that support it. Duq
*   11/02/15 - Turn off TE (Transmit Enable) after last transmit byte sent, else
*              can get lots of bogus trailing zeros rcvd by RX side, because
*              Xmitter is still active (TDR = 0), even though rupts are turned off.
*              Downside is it can cause intermitent FE Framing Errors.  ARGGGGG
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

#define  USES_CIRC_BUF                1  // UART uses Circular buffer facility
#define  IO_INTERNAL_CIRC_BUF_SIZE 0x7F  // modulo value for 128 byte buffer

#include "user_api.h"                   // pull in API defs and MCU depdent defs
#include "device_config_common.h"
#include "boarddef.h"


extern  uint32_t   _g_systick_millisecs;

//-------------------------------------------------
//          Trace / Debug     Globals
//-------------------------------------------------
#define  UART_TRACE_BUF_SIZE   0x1FF

        uint8_t    _g_rx_trace[UART_TRACE_BUF_SIZE+1];  // 512 byte RX trace
        int        _g_rx_trace_idx = 0;
        uint8_t    _g_tx_trace[UART_TRACE_BUF_SIZE+1];  // 512 byte TX trace
        int        _g_tx_trace_idx = 0;

        long       tx_char_sent    = 0;                 // API  DEBUG COUNTERs
        long       rx_char_rcvd    = 0;
        long       tx_send_calls   = 0;
        long       rxne_rupt_count = 0;                 // RUPT DEBUG COUNTERs
        long       ore_rupt_count  = 0;
        long       txe_rupt_count  = 0;
        long       tc_rupt_count   = 0;
        long       other_rx_err_rupt_count = 0;
        long       idle_rupt_count = 0;

typedef struct periph_io_block        // Peripheral I/O Control Block (I/O Buffers)
    {
        uint8_t    *io_user_buffer;   // Ptr to current spot in User I/O buf
        uint16_t   io_buf_length;     // Total amount of data to send/rcv
        uint16_t   io_tx_amt_sent;    // Amount of data send/rcv so far
        uint8_t    io_init;           //     1 = has been initialized
        uint8_t    io_state_T;        // Current state of the module: Reset/Busy/Complete
        uint8_t    io_state_R;        // RX state (if needed) Reset/Busy/Complete
        uint8_t    io_use_interrupts; // 1 = use interrupts,     0 = poll
        uint8_t    io_blocking;       // 1 = I/O is blocking. Wait till I/O is
                                      //  complete before return to user app
        uint8_t    io_master_slave;   // 1 = Master, 2 = Slave
        int        io_semaphore;      // I/O wait semaphore
        uint32_t   io_max_timeout_val;// max time to wait for I/O (millisec)
        uint32_t   io_expiry_time;    // timeout deadline for I/O response
        void       *io_HAL_handle;    // HAL Typedef Handle to use for this I/O
     IO_CB_EVENT_HANDLER io_callback_handler; // optional callback routine
        void             *io_callback_parm;   // user callback parm
        uint8_t    *io_rcv_user_buf;
        uint16_t   io_rcv_max_length;
        uint16_t   io_rx_amt_rcvd;
#if defined(USES_CIRC_BUF)
        uint8_t    io_rx_buf [IO_INTERNAL_CIRC_BUF_SIZE+1];    // 128 bytes
        int        io_buf_rx_head;    // head and tail ptrs for ioblock->io_rx_buf
        int        io_buf_rx_tail;
        int        io_buf_echo_head;  // head and tail ptrs for ioblock->io_echo_buf
        int        io_buf_echo_tail;
        uint8_t    *io_rcv_echo_ptr;  // current char being echo-plexed in user rcv buf
        uint8_t    io_do_echoplex;    // echo-plexing is turned on
        uint8_t    io_is_string_IO;   // current operation is a read_text_line, etc
        uint8_t    io_str_last_char;  // keeps track of user text input (\r \n ...)
#endif
    } IO_BUF_BLK;

    IO_BUF_BLK   *_g_ioblock_uart_trc;   // DEBUG trace of current I/O Buf Block

               // valid states for UART io_state_T and io_state_R
#define  UART_STATE_XMIT_BUSY           1
#define  UART_STATE_RCV_VIA_INTERRUPTS  2
#define  UART_STATE_XMIT_COMPLETE       3
#define  UART_STATE_RCV_COMPLETE        4
#define  UART_STATE_RESET               5

int  board_get_uart_io_block (unsigned int module_id, IO_BUF_BLK **ret_ioblock);
int  board_uart_enable_clock (int module_id);       // internal routines
void board_uart_enable_nvic_irq (int module_id);
void  board_uart_perform_echoplex (int module_id, IO_BUF_BLK *ioblock, uint8_t *rcv_end_buf);
int  board_uart_process_activation_chars (IO_BUF_BLK  *ioblock,
                                          uint8_t in_char, uint8_t *read_buf);
int  board_uart_read_common (unsigned int module_id, IO_BUF_BLK *ioblock, uint8_t *read_buf,
                             int buf_max_length, int is_string_IO);
int  board_get_uart_handle (int module_id, UART_HandleTypeDef **ret_UartHdl);
void board_common_UART_IRQHandler (USART_TypeDef *uart_module, int uart_module_id);
void USART1_IRQHandler (void);
void USART2_IRQHandler (void);
void USART3_IRQHandler (void);
void USART6_IRQHandler (void);

     //------------------------------------------------------------------------

#if defined(STM32F030x8) || defined(STM32F070xB)
//                                         STM32 - F030 / F072  Nucleo
#include "STM32_F0/board_F0_tables_uart.c"
#endif

#if defined(STM32F072xB)
//                                         STM32 - F072  Nucleo
#include "STM32_F0/board_F0_tables_uart.c"
#endif


#if defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_tables_F0_uart.c"
#endif


#if defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
#include "STM32_F1/board_F1_tables_uart.c"
#define  MAX_SPI   4
#endif


#if defined(STM32F303xC) || defined(STM32F303xE)
//                                         STM32 - F303  Nucleo and Discovery
//#include "STM32_F3/board_F303_tables_uart.c"
#include "STM32_F3/board_F3xx_tables_uart.c"
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo and Discovery
#include "STM32_F3/board_F3xx_tables_uart.c"
#endif

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401/F411  Nucleos
#include "STM32_F4/board_F4xx_tables_uart.c"
#endif

#if defined(STM32F446xx)
//                                         STM32 - F446  Nucleo
#include "STM32_F4/board_F4xx_tables_uart.c"
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "stm32f7xx_hal_usart.h"
#include "STM32_F7/board_F7_tables_uart.c"
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo
#include "stm32l0xx_hal_usart.h"
#include "STM32_L0/board_L0_tables_uart.c"
#define  MAX_SPI   3
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo
#include "stm32l1xx_hal_usart.h"
#include "STM32_L1/board_L1_tables_uart.c"
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
#include "STM32_L4/board_L4_tables_uart.c"
#endif


//----------------------------------------------------------------------
//  Handle differences in ST's naming of UART Xmit/Rcv/Status Registers
//----------------------------------------------------------------------
#if defined(STM32F030x8)  || defined(STM32F070xB) || defined(STM32F072xB) || defined(STM32F091xC) \
  || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F334x8) \
  || defined(STM32F746xx) || defined(STM32F746NGHx) \
  || defined(STM32L053xx) || defined(STM32L476xx)
#define  RCV_REG         RDR
#define  STATUS_REG      ISR
#define  XMIT_REG        TDR
#define  USART_SR_RXNE   USART_ISR_RXNE
#define  USART_SR_TXE    USART_ISR_TXE
#define  USART_SR_TC     USART_ISR_TC
#define  USART_SR_TEACK  USART_ISR_TEACK
#define  USART_SR_IDLE   USART_ISR_IDLE        /* IDLE DETECT vs TE ? */
#define  USART_SR_ORE    USART_ISR_ORE
#define  USART_SR_OTHER_RE_ERRS  (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE | USART_ISR_RTOF | USART_ISR_ABRE)
#define  USART_ICR_CLEAR_FLAGS   (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_RTOCF)
#define  USART_ICR_CLEAR_IDLE    (USART_ICR_IDLECF)
#else
#define  RCV_REG         DR
#define  STATUS_REG      SR
#define  XMIT_REG        DR
#define  USART_SR_OTHER_RE_ERRS  (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_RTOF | USART_SR_ABRE)
#define  USART_ICR_CLEAR_FLAGS   (USART_CR_PECF | USART_CR_FECF | USART_CR_NCF | USART_CR_ORECF | USART_CR_RTOCF)
#define  USART_ICR_CLEAR_IDLE    (USART_CR_IDLECF)
#endif


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


//*****************************************************************************
//*****************************************************************************
//                        GENERIC    UART    Routines
//*****************************************************************************
//*****************************************************************************


//*****************************************************************************
// board_get_uart_io_block
//
//          Gets the UART IO_BLOCK to use for this UART instance
//*****************************************************************************
int  board_get_uart_io_block (unsigned int module_id, IO_BUF_BLK **ret_ioblock)
{
    IO_BUF_BLK    *ioblock;

    if (module_id > UART_MAX_MODULE)
       return (ERR_UART_MODULE_NUM_OUT_OF_RANGE);

    if (_g_uart_io_blk_address[module_id] == 0L)
       return (ERR_UART_MODULE_NOT_SUPPORTED);

    *ret_ioblock = (IO_BUF_BLK*) _g_uart_io_blk_address [module_id];

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

int  board_uart_init (unsigned int module_id, int tx_pin_id,
                      int rx_pin_id, long baud_rate, int flags)
{
    IO_BUF_BLK          *ioblock;
    UART_HandleTypeDef  *pUartHdl;
    USART_TypeDef       *uart_hwbase;
    GPIO_TypeDef        *tx_GPIO_port;     /* TX GPIO port   */
    uint32_t            tx_GPIO_pin;       /* TX GPIO pin    */
    uint32_t            tx_AltFuncId;      /* TX Alt Func Id (pin mux) */
    GPIO_TypeDef        *rx_GPIO_port;     /* RX GPIO port   */
    uint32_t            rx_GPIO_pin;       /* RX GPIO pin    */
    uint32_t            rx_AltFuncId;      /* RX Alt Func Id (pin mux) */
    int                 rc;
    GPIO_InitTypeDef    GPIO_InitStruct;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

    _g_rx_trace_idx = 0;        // reset index used for DEBUG "long trace buf"

        //---------------------------------------------------------------------
        // Get pointers to UART module Hardware base address, and TypeDef Handle
        //---------------------------------------------------------------------
    uart_hwbase = (USART_TypeDef*) _g_uart_HW_module [module_id];
                     // use our internal TypeDef Handle for all calls
    pUartHdl = (UART_HandleTypeDef*) _g_uart_typedef_handle_addr [module_id];

       //-----------------------------------------------------------------------
       // Locate the associated GPIO Port and GPIO Pin for TX and RX pins
       //-----------------------------------------------------------------------
    rc = board_gpio_pin_lookup (tx_pin_id, &tx_GPIO_port, &tx_GPIO_pin);
    if (rc == 0)
       rc = board_gpio_pin_lookup (rx_pin_id, &rx_GPIO_port, &rx_GPIO_pin);
    if (rc != 0)
       return (rc);                  // invalid pin_id was given to us

       //-----------------------------------------------------------------------
       // Locate the associated GPIO Port and GPIO Pin, as well as
       // the associated AF information.
       // Returns ptrs to the _g_uart_NN_gpio_table[] entries for TX and RX pins
       //-----------------------------------------------------------------------
    rc = board_uart_ALTFUNC_lookup (module_id, tx_pin_id, &tx_AltFuncId,
                                    rx_pin_id, &rx_AltFuncId);
    if (rc != 0)
       return (rc);                  // pin_id not valid for this UART instance

       //--------------------------------------------------
       //                   GPIO  Init
       //
       // Configure the GPIOs that used for the UART pins for VCP
       //  e.g.  PA.2 = USART2_TX    PA.3 = USART2_RX
       //--------------------------------------------------
    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));  // clear struct
    GPIO_InitStruct.Pin       = tx_GPIO_pin;
    GPIO_InitStruct.Alternate = tx_AltFuncId;         // set TX Alt Function - UART
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    if (flags & UART_ENABLE_PULL_UPS)
       GPIO_InitStruct.Pull   = GPIO_PULLUP;          // Pull Ups = normal TX/RX operation
       else if (flags & UART_ENABLE_PULL_DOWNS)
               GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       else GPIO_InitStruct.Pull    = GPIO_NOPULL;
#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
#else
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
#endif
    HAL_GPIO_Init (tx_GPIO_port, &GPIO_InitStruct);   // Setup UART GPIO TX pin

    GPIO_InitStruct.Pin       = rx_GPIO_pin;
    GPIO_InitStruct.Alternate = rx_AltFuncId;         // set RX Alt Function - UART
    HAL_GPIO_Init (rx_GPIO_port, &GPIO_InitStruct);   // Setup UART GPIO RX pin

       //--------------------------------------------------
       // Configure the UART module.  Default = 115200
       //--------------------------------------------------
    board_uart_enable_clock (module_id);               // Turn on UART clock

    memset (pUartHdl, 0, sizeof(UART_HandleTypeDef));  // clear the structure

    pUartHdl->Instance        = (USART_TypeDef*) _g_uart_HW_module[module_id]; // Set UART module to use
    pUartHdl->Init.BaudRate   = baud_rate;
    pUartHdl->Init.WordLength = UART_WORDLENGTH_8B;    // setup as 8N1
    pUartHdl->Init.StopBits   = UART_STOPBITS_1;
    pUartHdl->Init.Parity     = UART_PARITY_NONE;      // no parity  KEY !
    pUartHdl->Init.HwFlowCtl  = UART_HWCONTROL_NONE;   // No flow ctl
    if (flags & UART_ENABLE_OVERSAMPLING)
       pUartHdl->Init.OverSampling = UART_OVERSAMPLING_16;   // Normal UART operation
       else pUartHdl->Init.OverSampling = UART_OVERSAMPLING_16;
    pUartHdl->Init.Mode       = UART_MODE_TX_RX;       // Enable RX and TX

    HAL_UART_Init (pUartHdl);   // Keil is wacking out on this  10/02/15
//  HAL_USART_Init ((USART_HandleTypeDef*) pUartHdl);  // and screws up on this - what is wrong with that TURD ??? !!!

       //---------------------------------------------
       // UART interrrupts are _always_ used.
       //---------------------------------------------
    board_uart_enable_nvic_irq (module_id);
    ioblock->io_use_interrupts = 1; // denote all UART I/O will use interrupts

    ioblock->io_do_echoplex    = 0; // default is no auto-echo of received characters

    if (flags && UART_ENABLE_ECHOPLEX)
       ioblock->io_do_echoplex = 1; // Turn on "echo plexing" of characters received
                                    // on a READ I/O operation, i.e. automatically
                                    // echo back any received character. This is
                                    // typical in Async "Full Duplex" systems.

    ioblock->io_state_T = UART_STATE_RESET; // init TX and RX states
    ioblock->io_state_R = UART_STATE_RESET;

         //-------------------------------------------------
         // prep for any rcv queuing and/or echo-plexing
         //-------------------------------------------------
    ioblock->io_buf_rx_head   = ioblock->io_buf_rx_tail  = 0;  // reset head and tail ptrs to clr buf

//#if defined(STM32L053xx)
//#else
    __HAL_USART_ENABLE_IT (pUartHdl,USART_IT_RXNE); // ALWAYS TURN ON RX interrupts
//#endif

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

int  board_uart_rx_flush (unsigned int module_id, int flags)
{
    uint8_t        in_byte;
    int            rc;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

                 // reset/clear out the buffer, by resetting internal buf index
    ioblock->io_buf_rx_head = ioblock->io_buf_rx_tail  = 0;  // reset head/tail ptrs to clr buf
    ioblock->io_buf_echo_head = ioblock->io_buf_echo_tail = 0;

    return (0);  // denote completed OK
}


//*****************************************************************************
//  board_uart_console_read_fdx                         aka   CONSOLE_READ_LINE
//
//             Reads in a line of text from the UART in full duplex mode.
//
//             It keeps reading from the UART until it receives an Activation
//             character of either \n or \r.
//             In addition, it automatically echos back any character that
//             the user types in, to support full-duplex "echo-plex" mode.
//
//             If you do not need the echo-plexing support, then just
//             issue a uart_Read_Line() call instead, which does a similar
//             job, but without the echo-plexing, and hence lower ovverhead.
//
//             This is primarily a convenience function, so that the user app
//             does not have to redundantly implement echo-plex support, when
//             communicating with PC or Linux "teletype" application like
//             putty, teleterm, etc.
//
//             if there is no character available, then we loop in here
//             for UART input, up until the max timeout limit (max_wait_time).
//
//             However, we pause after each check, by calling sys_Delay_Millis()
//             so that in lower power apps, we can go to sleep temporarily,
//             rather than burn up the battery doing continuious busy looping.
//
//  Returns:  0   +   a line of text in user_buf, ending with \0
//            or ERR_UART_RCV_TIMED_OUT (-305)
//            or ERR_xxxMODULE_ID_OUT_OF_RANGE (-300/-301)
//*****************************************************************************

int  board_uart_console_read_fdx (unsigned int module_id, uint8_t *user_buf, int max_buf_len,
                                  int pause_time, int max_wait_time, int flags)
{
    char           in_char;
    int            rc;
    int            amt_to_read;
    uint8_t        *buf_begin;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

             //----------------------------------------------------------------
             //              by definition, this is blocking logic.
             //
             // If user does NOT want blocking logic, then he needs to use
             // board_uart_rx_data_check() and board_uart_get_char() instead !
             //----------------------------------------------------------------
    ioblock->io_state_R = UART_STATE_RCV_COMPLETE;   // denote we a NOT using a user buffer
    ioblock->io_expiry_time = _g_systick_millisecs + max_wait_time;  // save any timeout
    buf_begin   = user_buf;
    *user_buf   = '\0';        // ensure buf is initially cleared
    amt_to_read = max_buf_len;
    while (amt_to_read)
      {      //------------------------------------------------------------------------
             //  we wait and process anything that gets queued in our internal buffers.
             //------------------------------------------------------------------------
        if (ioblock->io_buf_rx_tail != ioblock->io_buf_rx_head)
           { in_char = ioblock->io_rx_buf [ioblock->io_buf_rx_tail];
             ioblock->io_buf_rx_tail = (ioblock->io_buf_rx_tail + 1) % IO_INTERNAL_CIRC_BUF_SIZE;  // update tail ptr

                      //----------------------------------------------
                      // determine if it is an activation char or not
                      //----------------------------------------------
            rc = 0;
            if (in_char == '\b' || in_char == 0x7F)
               {     // process a backspace
                 if (user_buf != buf_begin)
                    {     // backspace the user buffer to erase the previous character
                      user_buf--;
                      *user_buf = '\0';
                      amt_to_read++;
                    }
                 rc = 2;                       // echo, but do not save the char
               }
              else if (in_char == '\r' || in_char == '\n' )
                      { if (ioblock->io_str_last_char == '\r' && user_buf != buf_begin)
                           rc = 2;  // ignore this. is last part of \r\n sequence
                           else { if (user_buf == buf_begin)
                                     *user_buf++ = '\n';  // treat as a standalone \n only line
                                   *user_buf = '\0';      // set end of string
                                   rc = 1;
                                }
                        ioblock->io_str_last_char = in_char; // save for \r\n check
                      }
                      // echo the char
            board_uart_write_bytes (module_id, &in_char, 1, UART_WAIT_FOR_COMPLETE);

            if (rc == 0)             // add a normal char to user buffer
               {
                 *user_buf++ = in_char;
                 amt_to_read--;
                 if (amt_to_read == 1)
                    {      // buffer is essentially full. add \0 and post complete
                      *user_buf = '\0';
                      rc = 1;
                    }
               }
              else if (in_char == '\r')
                      rc = 1;              // force end of line/string
            if (rc == 1)
               return (0); // Tell caller we have a complete line of text.
                           // For anything else, keep going. Backspaces are auto-deleted
                           // inside board_uart_process_activation_chars()
               // we just processed a char. update any max timeput
            ioblock->io_expiry_time = _g_systick_millisecs + max_wait_time;
           }

        if (ioblock->io_buf_rx_tail == ioblock->io_buf_rx_head)
           {          //------------------------------------------------------------
                      // buffer is temporarily empty. Wait till we rcv another char.
                      // We depend upon the interrupt handler to add any newly
                      // received characters to the internal io_rx_buf[] queue.
                      //------------------------------------------------------------
             if (pause_time != 0)
                HAL_Delay (pause_time);  // sleep for low power mode (LPM) to save battery

             if (max_wait_time)          // user has a max_timeout
                if (_g_systick_millisecs > ioblock->io_expiry_time)
                   return (ERR_UART_RCV_TIMED_OUT);  // hit the TIMEOUT limit - bail !
           }
         // loop back and process any newly received character(s)
      }

    return (0);
}


//*****************************************************************************
//  board_uart_rx_data_check                 aka   CONSOLE_CHECK_FOR_READ_DATA
//
//             Checks if any pending character input from UART.
//
//             Returns 0 if no pending data, and 1 if there is a char present.
//*****************************************************************************

int  board_uart_rx_data_check (unsigned int module_id)
{
    int            rc;
    IO_BUF_BLK     *ioblock;

// ??? 10/12/15 - May be some timeing issues in here when FULL-DUPLEX is turned on !!!
//                Might have an echoed back character in our buffer (e.g. talking to putty ?

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

// ??? return actual amt of data queued instead, that way app has a clue how much rcvd ???

    if (ioblock->io_buf_rx_head != ioblock->io_buf_rx_tail)
       return (1);                  // yes, we have some data in internal RX buf

    return (0);                     // no, no data has been received
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

int  board_uart_get_char (unsigned int module_id, int flags, int max_wait_time)
{
    int            in_char;
    int            rc;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);
    _g_ioblock_uart_trc = ioblock;         // DEBUG trace current I/O Buf Block

return_data_to_user:
  __disable_irq();           // Disable interrupts to avoid Head/Tail corruption
   if (ioblock->io_buf_rx_head != ioblock->io_buf_rx_tail)
      {          // pull a char that is queued in internal RX buffer
        in_char = ioblock->io_rx_buf [ioblock->io_buf_rx_tail];
        ioblock->io_buf_rx_tail = (ioblock->io_buf_rx_tail + 1) % IO_INTERNAL_CIRC_BUF_SIZE;  // update tail ptr
        if (ioblock->io_str_last_char == '\r'  &&  in_char == '\n')
           {   // Yes, so ignore the \n, because we already gave end of cmd signal
               // to the user. Avoid handling up a second "end of cmd".
             ioblock->io_str_last_char = 0; // clear out \r, so we treat any new \n as real.
           }
       __enable_irq();              // Re-enable interrupts
        return (in_char);           // hand it up to caller
      }
  __enable_irq();                   // Re-enable interrupts

   if (flags & UART_IO_NON_BLOCKING)
      return (WARN_WOULD_BLOCK);

                 //----------------------------------------------
                 // wait until we receive a char or timeout
                 //----------------------------------------------
                         // for TIMEOUT checking, generate expected end time
   ioblock->io_expiry_time = _g_systick_millisecs + max_wait_time;
   while (1)
     {
       if (ioblock->io_buf_rx_head != ioblock->io_buf_rx_tail)
          goto return_data_to_user;            // we finally got some data
       if (max_wait_time)                      // user has a max timeout value
          if (_g_systick_millisecs > ioblock->io_expiry_time)
             return (ERR_UART_RCV_TIMED_OUT);  // hit the TIMEOUT limit - bail !
     }

    return (ERR_UART_RCV_TIMED_OUT);
}



//*****************************************************************************
//  board_uart_process_activation_chars
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

int  board_uart_process_activation_chars (IO_BUF_BLK  *ioblock,
                                          uint8_t in_char, uint8_t *read_buf)
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
               if (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS)
                  {       // we are working directly with user buffer via active READ
                    if (ioblock->io_rx_amt_rcvd == 0) // 0 indicates 1st char in buf
                       return (2);                // no chars in buf. discard it
                    ioblock->io_rcv_user_buf--;   // backspace rcv buf 1 char
                    ioblock->io_rcv_max_length++;
                    return (2);                   // discard the backspace
                  }
                 else
                  {       // update our internal RX queue
                    if (ioblock->io_buf_rx_head == ioblock->io_buf_rx_tail)
                       return (2);             // no chars in buf. discard it
                    if (ioblock->io_buf_rx_head == 0)
                       ioblock->io_buf_rx_head = IO_INTERNAL_CIRC_BUF_SIZE; // handle a wrap around
                       else ioblock->io_buf_rx_head--;                    // update head ptr
                    ioblock->io_rx_buf [ioblock->io_buf_rx_head] = '\0';  // clear out the prev char
                    return (2);         // discard this char, but allow any echo
                  }
               break;

         case 2:                           // carriage return \r activation char
                          // handle CARRIAGE RETURN char - treat as end of line
               if (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS)
                  {       // we are working directly with user buffer via active READ
                          //----------------------------------------------------------------
                          // Check if \r is the only input char. in such a case, it is
                          // telling the user that he received a null line  (CR only).
                          // For cross-platform usage (Windows vs Unix), we always pass
                          // back a \n\0 sequence, to denote a NULL line rcvd.
                          //----------------------------------------------------------------
                    ioblock->io_str_last_char = '\r';       // save \r, to handle any \r\n sequence check
                    if (ioblock->io_rcv_user_buf != 0L)
                       { if (ioblock->io_rx_amt_rcvd == 0)        // 0 indicates 1st char in buf
                            {
                              *ioblock->io_rcv_user_buf++ = '\n'; // move in a \n to denote standalone line (CR only)
                              *ioblock->io_rcv_user_buf   = '\0'; // set end of string null terminator
                            }
                           else {     // strip it off, and denote end of string/text_line
                                  *ioblock->io_rcv_user_buf   = '\0'; // set end of string null terminator (to an existing string)
                                }
                       }
                    return (1);   // tell caller it is end of string/text_line
                  }
                 else
                  {       // update our internal RX queue
                          // convert it to a NL to denote an end of string/text_line
                    read_buf [ioblock->io_buf_rx_head] = '\n';   // denote end of line
                    ioblock->io_buf_rx_head = (ioblock->io_buf_rx_head + 1) % IO_INTERNAL_CIRC_BUF_SIZE; // update head ptr
                    ioblock->io_str_last_char = '\r';  // save \r, to handle any \r\n sequence check
                    return (2);
                  }
               break;

         case 3:                           // new line \n activation char
               if (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS)
                  {       // we are working directly with user buffer via an active READ
                    if (ioblock->io_str_last_char == '\r')
                       {       // this is end of a \r\n sequence. Discard the \n
                         ioblock->io_str_last_char = 0;  // clear \r\n sequence flag
                         if (ioblock->io_rx_amt_rcvd > 0)
                            return (1);   // tell rupt handler it is end of text_line
                         return (2);      // otherwise, just discard it.
                       }
                    if (ioblock->io_rcv_user_buf != 0L)
                       {
                         if (ioblock->io_rx_amt_rcvd == 0)            // 0 indicates 1st char in buf
                            {       // Otherwise, this is a standalone \n, without a previous \r.
                               // treat similar to above - return a \n to tell user that he got a null line (NL only)
                              *ioblock->io_rcv_user_buf++ = '\n';     // copy it
                              *ioblock->io_rcv_user_buf   = '\0';     // set end of string null terminator
                            }
                           else {   // we already have a bunch of chars in the buffer
                                  *ioblock->io_rcv_user_buf   = '\0'; // set end of string null terminator
                                }
                       }
                    return (1);      // tell caller it is end of string/text_line
                  }
                 else
                  {       // update our internal RX queue
                    if (ioblock->io_str_last_char == '\r')
                       { ioblock->io_str_last_char = 0;  // clear \r\n sequence flag
                         return (2);      // discard it. strip the \n when part of a \r\n sequence
                       }
                      else return (0);    // just copy the \n to internal buf
                  }
               break;
       }                                  //   end   switch()

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

int  board_uart_read_bytes (unsigned int module_id,
                            uint8_t *read_buf, int buf_length, int flags)
{
    uint8_t        in_byte;
    int            rc;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

    ioblock->io_is_string_IO = 0; // denote is BINARY I/O, so _no_ Activation Char processing

    rc = board_uart_read_common (module_id,ioblock, read_buf, buf_length, 0);

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

int  board_uart_read_common (unsigned int module_id, IO_BUF_BLK *ioblock,
                             uint8_t *user_read_buf, int buf_max_length,
                             int is_string_IO)
{
    char     *begin_buf;
    char     in_char;
    int      rc;
    int      amt_left;

    _g_ioblock_uart_trc = ioblock;         // DEBUG trace current I/O Buf Block

    begin_buf = user_read_buf;
    ioblock->io_rx_amt_rcvd   = 0;
    ioblock->io_rcv_echo_ptr  = user_read_buf;

             //-----------------------------------------------------------------
             // first, move any data from internal RX queue buffer to user buf.
             //-----------------------------------------------------------------
   amt_left = buf_max_length;
__disable_irq();             // Disable interrupts to avoid Head/Tail corruption
   while (ioblock->io_buf_rx_head != ioblock->io_buf_rx_tail)
     {                       // copy from our internal buf to user app buf
       in_char = ioblock->io_rx_buf [ioblock->io_buf_rx_tail];
       *user_read_buf = in_char;
       amt_left--;
                             // update tail ptr
       ioblock->io_buf_rx_tail = (ioblock->io_buf_rx_tail + 1) % IO_INTERNAL_CIRC_BUF_SIZE;
       if (is_string_IO  &&  (in_char == '\n' || amt_left <= 1))
          {        //------------------------------------------------------
                   // we hit end of a string.  return this string to user
                   // see if this is a standalone \n  (NL only or CR only)
                   //------------------------------------------------------
            if (user_read_buf == begin_buf)
               {     // this is first char in buf so pass thru the standalone \n
                 user_read_buf++;  // step to next position in buf to put \0
               }                   // otherwise, overlay the trailing \n with \0
            *(user_read_buf) = '\0';  // ensure null terminator \0 is appended
            ioblock->io_str_last_char = 0;  // clear out any \r indicator
          __enable_irq();          // Macro to re-enable rupts
            return (1);            // tell caller we got a string
          }
         else if (amt_left == 0)
                  {
                  __enable_irq();  // Macro to re-enable rupts
                    return (1);    // tell caller we filled buffer
                  }
                                   //----------------------------------------
       user_read_buf++;            //  step to next char in user app buffer
     }                             //----------------------------------------
              //----------------------------------
              // there is still more data needed
              //----------------------------------
    ioblock->io_rx_amt_rcvd    = buf_max_length - amt_left;
    ioblock->io_rcv_max_length = amt_left;   // Save I/O length and user buf ptr
         // start Interrupt reads _after_ the chars that were already rcvd in the above head/tail loop
    ioblock->io_rcv_user_buf   = user_read_buf;   // points to next open position in user buf
    ioblock->io_state_R  = UART_STATE_RCV_VIA_INTERRUPTS; // denote I/O in progress

              // for TIMEOUT checking, generate expected end time
    ioblock->io_expiry_time = _g_systick_millisecs + ioblock->io_max_timeout_val;

              //--------------------------------------------------------------
              // Wait for intyerrupt handler to complete reading in the
              // rest of the characters. It will move them direct to user buf
              //--------------------------------------------------------------
__enable_irq();              // Macro to re-enable interrupts

// in future, call SEMAPHORE in NO_RTOS instead (to allow low power)

    while (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS)
      {    // ioblock->io_state_R will be set to UART_STATE_RCV_COMPLETE in ISR
           // when the max length has been reached, or an activation character rcvd.

           // if there is a Timeout limit, see if we reached it
        if (ioblock->io_max_timeout_val)          // user specified a max timeout
           { if (_g_systick_millisecs > ioblock->io_expiry_time)
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

int  board_uart_read_text_line (unsigned int module_id,
                                char *read_buf, int buf_max_length, int flags)
{
    char           in_char;
    int            rc;
    int            amt_left;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

    ioblock->io_is_string_IO = 1; // denote is STRING I/O, so enable Activation Char processing

    rc = board_uart_read_common (module_id, ioblock, (uint8_t*) read_buf,
                                 buf_max_length, 1);

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
int  board_uart_set_callback (unsigned int module_id,
                              IO_CB_EVENT_HANDLER callback_function,
                              void *callback_parm)
{
    int            rc;
    IO_BUF_BLK     *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts
       //-------------------------------------------------------------------
    ioblock->io_callback_handler = callback_function;
    ioblock->io_callback_parm     = callback_parm;

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
int  board_uart_set_max_timeout (unsigned int module_id, uint32_t max_timeout)
{
    int                rc;
    IO_BUF_BLK         *ioblock;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);

    ioblock->io_max_timeout_val = max_timeout;       // save the receive timeout value we should use

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

int  board_uart_write_char (unsigned int module_id, char outchar, int flags)
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

int  board_uart_write_bytes (unsigned int module_id, uint8_t *bytebuf,
                             int buf_len, int flags)
{
    uint8_t            tx_char ;
    int                rc;
    UART_HandleTypeDef *pUartHdl;
    IO_BUF_BLK         *ioblock;

tx_send_calls++;

    rc = board_get_uart_io_block (module_id, &ioblock);
    if (rc != 0)
       return (rc);
    _g_ioblock_uart_trc = ioblock;         // DEBUG trace current I/O Buf Block

    pUartHdl = (UART_HandleTypeDef*) _g_uart_typedef_handle_addr [module_id];

    if (ioblock->io_state_T < UART_STATE_XMIT_COMPLETE)
       return (ERR_IO_ALREADY_IN_PROGRESS);    // user had started a previous transmit that
                                               // has not yet completed
    ioblock->io_user_buffer = bytebuf;         // save address of data to be sent
    ioblock->io_buf_length  = buf_len;         // save I/O length and contents
    ioblock->io_tx_amt_sent = 0;               // reset for new pass

    ioblock->io_state_T = UART_STATE_XMIT_BUSY;         // denote I/O in progress

// Skipping the first send in here works, apparently with no ill effects.
#if WAIT_FOR_FIRST_TXE_RUPT    // 11/02/15 HAL's code just turns on the TXE rupt and waits for rupt before sends 1st char !
           // send first byte in user buf   // Hopefully above eliminates the truncated 1st char we are seeing on RX side !
    tx_char = *ioblock->io_user_buffer++;      // Trace it
    _g_tx_trace[_g_tx_trace_idx] = tx_char;   // trace everything to a 512 byte buf
    _g_tx_trace_idx = (_g_tx_trace_idx + 1) % UART_TRACE_BUF_SIZE; // auto-wrap trace index

    pUartHdl->Instance->XMIT_REG = (uint32_t) tx_char;   // Send it
    ioblock->io_tx_amt_sent++;
tx_char_sent++;
#endif
               // _Enable_ the USART TXE Transmit data register empty Interrupt bit in CR1.
    __HAL_USART_ENABLE_IT (pUartHdl, USART_IT_TXE);
               // and _Enable_ the USART RXNE Interrupt bit in CR1 for echo-plex
    __HAL_USART_ENABLE_IT (pUartHdl, USART_IT_RXNE);

                  //---------------------------------------------------
                  // see if we need to wait for the transmission to complete
                  //--------------------------------------------------------
        if (flags & UART_IO_NON_BLOCKING)
           { return (0);    // Non-blocking: denote we started transmit I/O OK.
                            // Final status will be via callback/check_complete
           }
          else {            // calling app wants us to wait till I/O is complete
                 while (ioblock->io_state_T == UART_STATE_XMIT_BUSY)
                   ;
// in future, call SEMAPHORE in NO_RTOS instead (low power)

               }

    return (0);             // Transmit I/O completed OK
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

int  board_uart_write_string (unsigned int module_id, char *outstr, int flags)
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

void  board_common_UART_IRQHandler (USART_TypeDef *uart_module, int uart_module_id)
{
    UART_HandleTypeDef  *pUartHdl;
    uint32_t            rupt_flag;
    uint8_t             in_char;
    uint8_t             tx_char;
    int                 next_to_last_byte_idx;
    int                 rc;
    IO_BUF_BLK          *ioblock;

    pUartHdl = (UART_HandleTypeDef*) _g_uart_typedef_handle_addr [uart_module_id];
    ioblock = (IO_BUF_BLK*) _g_uart_io_blk_address [uart_module_id];
    _g_ioblock_uart_trc = ioblock;         // DEBUG trace current I/O Buf Block

      //----------------------------------------------------------------
      // Determine what flag caused the interrupt:  RXNE, TXE, TC, OVR
      //----------------------------------------------------------------

         //----------------------------------------
         // Fileter out / Discard any IDLEs, that can screw up RX logic
         //--------------------------------------------------------------
    rupt_flag = (pUartHdl->Instance->STATUS_REG & USART_SR_IDLE);
    if (rupt_flag != 0)
       {
idle_rupt_count++;
         pUartHdl->Instance->ICR |= USART_ICR_CLEAR_IDLE;  // clear and discard it
//       in_char = (uint8_t) pUartHdl->Instance->RCV_REG;  // read and discard it ?
       }
         //----------------------------------------
         //    Process a receive RXNE
         //----------------------------------------
    rupt_flag = (pUartHdl->Instance->STATUS_REG & USART_SR_RXNE);
    if (rupt_flag != 0  &&  (pUartHdl->Instance->CR1 & USART_CR1_RXNEIE))  // verify rupt is enabled
       {
rxne_rupt_count++;
rx_char_rcvd++;
                  //------------------------------
                  //     process a rcvd byte.
                  //-------------------------------
                  // Note: reading in the byte automatically clears the RXNE rupt
         in_char = (uint8_t) pUartHdl->Instance->RCV_REG;

_g_rx_trace[_g_rx_trace_idx] = in_char;          // trace everything to a 512 byte buf
_g_rx_trace_idx = (_g_rx_trace_idx + 1) % UART_TRACE_BUF_SIZE;     // auto-wrap trace index

         rc = 0;
         if (ioblock->io_is_string_IO && (in_char <= '\r' || in_char == 0x7F))
            {     //---------------------------------------------------------------------
                  // this is string I/O, so process any special chars like \b \r or \n
                  // it returns 1 of 4 return codes:  0 = process normally,   do any echo
                  //                                  1 = operation complete, do any echo
                  //                                  2 = backspace: discard, do any echo
                  //                                 -3 = other:     discard, do NOT echo
                  //---------------------------------------------------------------------
              rc = board_uart_process_activation_chars (ioblock, in_char, ioblock->io_rx_buf);
            }

         if (rc == 0)
            { ioblock->io_str_last_char = in_char;    // save char, to handle any \r\n sequence check
                        //------------------------------------------------
                        // determine where to store the received char
                        //------------------------------------------------
              if (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS)
                 {      //------------------------------------------------
                        // we are in the middle of a user app active READ.
                        // copy it directly to the user app's buffer.
                        //------------------------------------------------
                   *ioblock->io_rcv_user_buf++ = in_char;     // save char into user rcv buffer
                   ioblock->io_rcv_max_length--;              // deduct # bytes left to rcv
                  if (ioblock->io_is_string_IO)
                     {     // see if we have hit the max size string/text_line we can put in buf
                       if (ioblock->io_rcv_max_length == 1)
                          {    // we've filled the buffer. Return completed buf to caller
                            *ioblock->io_rcv_user_buf = '\0'; // append \0 to denote end of string/text_line
                            rc = 1;                  // set condition as "read is complete"
                          }
                     }
                    else if (ioblock->io_rcv_max_length == 0) // is a uart_read_bytes() op
                            rc = 1;                  // set condition as "read is complete"
                   ioblock->io_rx_amt_rcvd++;                 // inc # bytes rcvd
                 }
                else
                 {      //--------------------------------------
                        //  queue into our internal RX buffer.
                        //--------------------------------------
                  ioblock->io_rx_buf [ioblock->io_buf_rx_head] = in_char; // save char into our internal RX queue
                        // update head ptr
                  ioblock->io_buf_rx_head = (ioblock->io_buf_rx_head + 1) % IO_INTERNAL_CIRC_BUF_SIZE;
                 }

            }

//  ??? NEED TO HANDLE ECHO-PLEX on  in_char   !!! ???   WVD  !!! ???    (if rc != 3)

// 09/22/15
//    RE-WORK THIS TO SLIDING / ROTATING INTERNAL BUFFER ala ARDUINO, and
//    DISCARD CHARS IF REACH LIMIT, W/FLAG WARNING SET ON ANY USER RCV CALL
//    ALSO, CHANGE uart_read_string() logic to operate as a higher level,
//    that call lower level to just read chars, and upper level processes them
//    Possibly change it to a "uart_console_read() call instead ???

         if (ioblock->io_state_R == UART_STATE_RCV_VIA_INTERRUPTS  &&  rc == 1)
            {        //-----------------------------------------------------------------------
                     // we are all done with a user app "active READ".
                     // Post the operation complete and issue any callback to user.
                     //
                     // Note: Unlike HAL, we do _NOT_ turn off RX rupts, because with full
                     // duplex devices like SIM808 or ESP8266, there is always a timing window
                     // to bite us. So any subsequent rcvd chars get queued in internal buf.
                     //-----------------------------------------------------------------------
              ioblock->io_state_R = UART_STATE_RCV_COMPLETE;   // set ending status
if (ioblock->io_rx_amt_rcvd > 1)    // DEBUG HOOK for strings >  just \n
              ioblock->io_rcv_user_buf = 0L;  // clear ptr to denote no more active user READ
              if (ioblock->io_callback_handler != 0L)
                 {                            // Invoke any user callback for the interrupt
                    (ioblock->io_callback_handler) (ioblock->io_callback_parm,
                                                    uart_module_id, UART_RX_COMPLETE);
                 }
            }
       }                    //  end  if rupt_flag != 0

         //------------------------------------------
         //  Process a Receive Overrun ORE condition
         //------------------------------------------
    rupt_flag = pUartHdl->Instance->STATUS_REG & USART_SR_ORE;
    if (rupt_flag != 0)  //  &&  (pUartHdl->Instance->CR1 & USART_CR1_OREIE))  // verify rupt is enabled
       {
ore_rupt_count++;
         in_char = (uint8_t) pUartHdl->Instance->RCV_REG;   // discard whatr is in buffer
         in_char = (uint8_t) pUartHdl->Instance->RCV_REG;   // hit a second time to be sure
       }

         //----------------------------------------
         //  Process a transmit TXE buffer empty
         //----------------------------------------
    rupt_flag = pUartHdl->Instance->STATUS_REG & USART_SR_TXE;
    if (rupt_flag != 0 && (pUartHdl->Instance->CR1 & USART_CR1_TXEIE))
       {    // process a transmitted byte. Send next char if there is one.
            // Note: transmitting a new byte automatically clears the TXE rupt.
txe_rupt_count++;
         if (ioblock->io_tx_amt_sent < ioblock->io_buf_length)
            {     // send next byte in user buffer
              tx_char = *ioblock->io_user_buffer++;
              _g_tx_trace[_g_tx_trace_idx] = tx_char;   // trace everything to a 512 byte buf
              _g_tx_trace_idx = (_g_tx_trace_idx + 1) % UART_TRACE_BUF_SIZE; // auto-wrap trace index
              pUartHdl->Instance->XMIT_REG = (uint32_t) tx_char;
              ioblock->io_tx_amt_sent++;
tx_char_sent++;
            }
         if (ioblock->io_tx_amt_sent == ioblock->io_buf_length)
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
    rupt_flag = pUartHdl->Instance->STATUS_REG & USART_SR_TC;
    if (rupt_flag != 0)
      { if (pUartHdl->Instance->CR1 & USART_CR1_TCIE)
           {      // process a transmitted byte. Send next char if there is one.
tc_rupt_count++;
//          CLEAR_BIT (pUartHdl->Instance->CR1, USART_CR1_TE); // Turn Off TRANSMIT ENABLE to avoid sending training 0x00s/0xFFs from XMIT_REG
            if (ioblock->io_tx_amt_sent == ioblock->io_buf_length)
               {     // we are all done sending. Post operation complete and
                     // issue any callback to user
                     // _Disable_ the USART TC Transmit Complete Interrupt bit in CR1.
                 __HAL_USART_DISABLE_IT (pUartHdl, USART_IT_TC);
                     // Disable any USART Error Interrupts: (Frame error, noise error, overrun error)
////             __HAL_USART_DISABLE_IT (pUartHdl, USART_IT_ERR);

                 ioblock->io_state_T = UART_STATE_XMIT_COMPLETE;  // set ending status
                 if (ioblock->io_callback_handler != 0L)
                    {             // Invoke user callback for the interrupt
                      (ioblock->io_callback_handler) (ioblock->io_callback_parm,
                                                      uart_module_id, UART_TX_COMPLETE);
                    }
               }
           }
// 11/02/15 - Loading the XMIT_REG with 0x00 or 0xFF causes that to be sent out !!! even if rupts are off ! So this SPEC advice was total bullshit
///    pUartHdl->Instance->XMIT_REG = 0xFF;  // per tech ref, clear TC flag by writing dummy byte to DR/TDR
      }
           //-------------------------------------------------------------
           // handle any other RX errors, that can lead to an ORE lockup
           //-------------------------------------------------------------
    rupt_flag = pUartHdl->Instance->STATUS_REG & USART_SR_OTHER_RE_ERRS;
    if (rupt_flag != 0)  //  &&  (pUartHdl->Instance->CR1 & USART_CR1_OREIE))  // verify rupt is enabled
       {
other_rx_err_rupt_count++;
         in_char = (uint8_t) pUartHdl->Instance->RCV_REG;   // discard what is in buffer
         pUartHdl->Instance->ICR |= USART_ICR_CLEAR_FLAGS;  // and clear any RX error flags
       }
}                                 //   end   board_common_UART_IRQHandler()


//#if (USES_UART)                 // THIS MOVES TO stm32xx_it.c with (USES_UART) enabled
void USART1_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART1, 1);
}


void USART2_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART2, 2);
}

//  !!! ??? WVD need to add other IRQ support F3 and F4_46 e.g. USART4/5/.../8

#if defined(STM32F091xC) \
  || defined(STM32F446xx) \
  || defined(STM32F746xx) || defined(STM32F746NGHx) || defined(STM32L476xx)

void USART3_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART3, 3);
}
#endif

#if defined(STM32F091xC) \
  || defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) \
  || defined(STM32F746xx) || defined(STM32F746NGHx)

void USART6_IRQHandler (void)
{
    board_common_UART_IRQHandler (USART6, 6);
}
#endif

//#endif                                       // (USES_UART)


/******************************************************************************/
