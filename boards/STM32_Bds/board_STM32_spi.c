
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             board_STM32_spi.c
//
//
//  Common Logic for SPI support for STM32 MCUs.
//
//  Uses a combination of HAL_Library (mostly for configuration) and direct
//  register calls (main execution).
//
//  Specific chip dependences are mainly constrained to tables (GPIOs used
//  specific TIMx modules supported, etc). They are pulled in as #includes
//  based on MCU type.
//
//  In general, the use of #ifdefs within the code has been minimized, in
//  order to keep things read-able and maintainable.
//
//  History:
//    12/30/14 - Created for Industrial IoT OpenSource project.  Duq
//    07/30/15 - Reworked to provide better factoring. Worked first shot. Duq
//    08/09/15 - Tweaked Interrrupt Handling. Duquaine
//    08/10/15 - Set default to SPI_IO_BLOCKING because so much vendor code
//               depends upon blocking behavior. I do not have the time to
//               re-write all their code. Duq
//    10/17/15 - Ensure a minimal "spi timeout" value is set for SPI read/write
//               (in case user app forgets), otherwise get immediate timeouts. Duq
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

#include "user_api.h"         // pull in high level User API defs
#include "device_config_common.h"     // device config stuff for SPI

#define  MAX_SPI_ID    100                // ??? !!  TWEAK THIS  WVD   !!! ???

typedef struct spi_gio_def                   /* SPI GPIO pin definitions */
    {
//      short        spi_api_id;             /* User API Logical SPI identifier*/
//      short        spi_module_id;          /* Associated SPI module index  */
//      GPIO_TypeDef *spi_SCLK_gpio_port;    /* Associated GPIO port         */
//      uint16_t     spi_SCLK_gpio_pin;      /* Associated GPIO pin          */

//      uint16_t     spi_SCLK_chan_alt_func; /* Channel Alternate Function Id*/

//      GPIO_TypeDef *spi_MISO_gpio_port;    /* Associated GPIO port         */
//      uint16_t     spi_MISO_gpio_pin;      /* Associated GPIO pin          */

//      uint16_t     spi_MISO_chan_alt_func; /* Channel Alternate Function Id*/

//      GPIO_TypeDef *spi_MOSI_gpio_port;    /* Associated GPIO port         */
//      uint16_t     spi_MOSI_gpio_pin;      /* Associated GPIO pin          */

//      uint16_t     spi_MOSI_chan_alt_func; /* Channel Alternate Function Id*/

        uint32_t     spi_chan_alt_func;     /* SPI Alternate Function Id     */
        int8_t       spi_pin_role;          /* acts in SCLK, MISO, or MOSI role*/
        uint8_t      spi_pin_number;        /* logical pin number 0-255      */
    } SPI_GPIO_BLK;

                    // valid values for spi_pin_role
#define  SCLK_PIN_ROLE   1
#define  MISO_PIN_ROLE   2
#define  MOSI_PIN_ROLE   3


typedef struct spi_io_block            /* SPI I/O Control Block (I/O Buffers) */
    {
        uint8_t      *spi_buffer;      /* Ptr to current spot in User I/O buf */
        uint16_t     spi_length;       /* Amount of data left to send/rcv     */
        uint8_t      spi_init;         /* 1 = module + clocks were initialized*/
        uint8_t      spi_state;        /* Current state of the SPI module     */
        uint8_t      spi_blocking;     /* 1 = I/O is blocking. Wait till I/O is
                                       **  complete before return to user app */
        uint8_t      spi_master_slave; /* 1 = Master, 2 = Slave               */
        uint8_t      spi_mode;         /* SPI Mode: 0, 1, 2, or 3             */
        int          spi_semaphore;    /* I/O wait semaphore                  */
        uint32_t     spi_max_timeout;  /* max time to wait for I/O (millisec) */
        uint32_t     spi_expiry_time;  /* timeout deadline for SPI I/O response */
     SPI_HandleTypeDef    *spi_handle; /* HAL SPI Handle to use for this I/O  */
     SPI_CB_EVENT_HANDLER spi_callback_handler;  /* optional callback routine */
        void         *spi_callback_parm;         /* user callback parm        */
    } SPI_IO_BUF_BLK;

                    // valid values for spi_state
#define  SPI_STATE_RESET            0   /* SPI is reset, not in use  */
#define  SPI_STATE_IO_PROCESSING    1   /* SPI I/O is in progress    */
#define  SPI_STATE_IO_PEND_COMPLETE 2   /* SPI I/O waiting for final complete */
#define  SPI_STATE_IO_COMPLETED     3   /* SPI I/O is fully complete */
#define  SPI_STATE_ERROR_COMPLETE   4   /* SPI I/O completed with an error */


void board_spi_enable_nvic_irq (int module_id);
void board_spi_stop_io (SPI_IO_BUF_BLK *ioblock);  // internal routines protos
int  board_spi_write_byte (SPI_IO_BUF_BLK *ioblock, uint8_t byte_Value);
int  board_spi_ALTFUNC_lookup (unsigned int module_id, int sclk_gpio_pin,
                               uint32_t *sclk_AltFuncId,
                               int miso_gpio_pin, uint32_t *miso_AltFuncId,
                               int mosi_gpio_pin, uint32_t *mosi_AltFuncId);


     //------------------------------------------------------------------------
     //
     // Pull in        MCU dependent GPIO / TIM tables and #defines
     //
     // The top of each module include key defines for:
     //          MAX_TIMERS,
     //          TMRPWM_NUM_MODULES
     //          TMRPWM_MAX_CHANNELS
     //
     // The following tables are pulled in:
     //          _g_timer_module_base[]       table/array
     //          _g_TIMx_TimPwmHandle's
     //          _g_timer_typedef_handle[]    table/array       was _g_timer_module_handle
     //          _g_TMPWM_IRQ_table[]         table/array
     //          _g_timer_base_channel_num[]  table/array
     //          _g_tmrpwm_mod_x_channels []  set of tables/arrays for GPIOs
     //          _g_tmrpwm_mod_channel_blk_lookup []    table/array
     //
     // Also, the following two utility routines are included in those files:
     //        - board_timerpwm_compute_prescalar()  because that is MCU speed
     //                                              and clock specific
     //        - board_timerpwm_enable_clock()       because which Timers are
     //                                              supported and the logic to
     //                                              start them is MCU specific
     //------------------------------------------------------------------------

#if defined(STM32F072xB) || defined(__STM32F072__)
//                                         STM32 - F072  Nucleo
#include "STM32_F0/board_F0_tables_spi.c"
#define  MAX_SPI   2
#endif


#if defined(STM32F091xC) || defined(__STM32F091__)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_tables_F0_spi.c"
#define  MAX_SPI   2
#endif


#if defined(__STM32F103__) || defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
#include "STM32_F1/board_F1_tables_spi.c"
#define  MAX_SPI   4
#endif


#if defined(STM32F303xC) || defined(STM32F303xE)
//                                         STM32 - F303  Nucleo and Discovery
#include "STM32_F3/board_F303_tables_spi.c"
#define  MAX_SPI   3
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo and Discovery
#include "STM32_F3/board_F334_tables_spi.c"
#define  MAX_SPI   1
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401/F411  Nucleos
#include "STM32_F4/board_F4x1_tables_spi.c"
#define  MAX_SPI   4
#endif

#if defined(STM32F446xx)
//                                         STM32 - F446  Nucleo
#include "STM32_F4/board_F446_tables_spi.c"
#define  MAX_SPI   4
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "STM32_F7/board_F7_tables_spi.c"
#define  MAX_SPI   6
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo
#include "STM32_L0/board_L0_tables_spi.c"
#define  MAX_SPI   3
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo
#include "STM32_L1/board_L1_tables_spi.c"
#define  MAX_SPI   3
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
#include "STM32_L4/board_L4_tables_spi.c"
#define  MAX_SPI   3
#endif


//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES and DEFINEs
//
//                                for
//
//                                SPI
//*****************************************************************************
//*****************************************************************************

#if (USES_BLUENRG_BLE)
extern    SPI_HandleTypeDef  SpiHandle;  // NOTE: ST's hci routines depend upon
                                         //    this hardcoded  SpiHandle  label.
                                         //    They have direct externs to it.
#endif


int  board_spi_enable_clock (int module_id);    // Prototypes for internal rtns

extern   const  int  _g_gpio_pull_flags[];      // defined in board_XX.c


//*****************************************************************************
//*****************************************************************************
//                               SPI   Routines
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
* board spi init function
*
*         Configures this MCU's SPI peripheral and its associated GPIOs.
*
*         The SPI Module Id specifies, both the SPI Module and the sub-variant
*         pins to use for that that module.
*
*         The module_id is split:
*             bottom low order 4 bits (0..3) = pin variant.
*             next nibble up (bits 4-7)      = actual SPI module number 0-5
*
*
*                SCLK     MISO     MOSI
*                ----     ----     ----
*         SPI1 = PA5   /  PA6   /  PA7
*           "    PB3   /  PB4   /  PB5    (alternate pin mode)
*         SPI2 = PB13  /  PC2   /  PC3
*           "    PB13  /  PB14  /  PB15   (alternate pin mode)
*         SPI3 = PC10  /  PC11  /  PC12
*           "    PB3   /  PB4   /  PB5    (alternate pin mode)
*
*  flags:   use_dma,   USES_BLUENRG_BLE, GPIO_OUT_PULLUP, GPIO_IN_PULL_UP/DOWN
*
* 07/31/15
*    Added option for user to pass in his own SPI_HandleTypeDef handle, instead
*    of using our internal one. This preserves some vendor SPI legacy code, that
*    have a very high dependency on using their own SPI_HandleTypeDef handle.
*******************************************************************************/

int  board_spi_init (unsigned int spi_module_id,
                     int sclk_pin_id, int miso_pin_id, int mosi_pin_id,
                     int master_slave, int spimode,
                     int baud_rate_scalar,  int flags,
                     SPI_HandleTypeDef *caller_hspi_ptr)     // extended support
{
    SPI_HandleTypeDef  *pSpiHdl;
    SPI_TypeDef        *spi_hwbase;
    SPI_IO_BUF_BLK     *ioblock;
    int                rc;
    int                idx;
    int                pull_flags_idx;
    int                sclk_pull_flags;
    int                miso_pull_flags;
    int                mosi_pull_flags;
    GPIO_TypeDef       *sclk_GPIO_port;    /* SCLK GPIO port   */
    uint32_t           sclk_GPIO_pin;      /* SCLK GPIO pin    */
    uint32_t           sclk_AltFuncId;     /* SCLK Alt Func Id (pin mux) */
    GPIO_TypeDef       *miso_GPIO_port;    /* MISO GPIO port   */
    uint32_t           miso_GPIO_pin;      /* MISO GPIO pin    */
    uint32_t           miso_AltFuncId;     /* MISO Alt Func Id (pin mux) */
    GPIO_TypeDef       *mosi_GPIO_port;    /* MOSI GPIO port   */
    uint32_t           mosi_GPIO_pin;      /* MOSI GPIO pin    */
    uint32_t           mosi_AltFuncId;     /* MOSI Alt Func Id (pin mux) */
    GPIO_InitTypeDef   GPIO_InitStruct;

    if (spi_module_id > MAX_SPI_ID)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

         //--------------------------------------------------------------
         // Get pointer to the SPI I/O Block to use for this module
         //--------------------------------------------------------------
    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];

         //---------------------------------------------------------------------
         // If module was already initialized - skip doing it again, and issue a warning
         //---------------------------------------------------------------------
    if (ioblock->spi_init)
       return (WARN_SPI_WAS_ALREADY_INITIALIZED);

       //---------------------------------------------------------------------
       // Get pointers to SPI MODULE Hardware base address, and TypeDef Handle
       //---------------------------------------------------------------------
    spi_hwbase = (SPI_TypeDef*) _g_spi_module_base [spi_module_id];
    pSpiHdl    = (SPI_HandleTypeDef*) _g_spi_typedef_handle [spi_module_id];

    if (caller_hspi_ptr != 0L)
       {   // caller wants to use _his_ SPI_TypeDef handle, not our internal one
         pSpiHdl = caller_hspi_ptr;                     // extended support
       }
    memset (pSpiHdl, 0, sizeof(SPI_HandleTypeDef));     // ensure is cleared out

       //---------------------------------------------------------------------
       // Locate associated GPIO Port and GPIO Pin for SCLK, MISO, MOSI pins
       //---------------------------------------------------------------------
    rc = board_gpio_pin_lookup (sclk_pin_id, &sclk_GPIO_port, &sclk_GPIO_pin);
    if (rc == 0)
       rc = board_gpio_pin_lookup (miso_pin_id, &miso_GPIO_port, &miso_GPIO_pin);
    if (rc == 0)
       rc = board_gpio_pin_lookup (mosi_pin_id, &mosi_GPIO_port, &mosi_GPIO_pin);
    if (rc != 0)
       return (rc);                  // invalid pin_id was given to us

       //---------------------------------------------------------------------
       // Locate the associated GPIO ALTERNATE_FUNCTION (AF) information.
       //---------------------------------------------------------------------
    rc = board_spi_ALTFUNC_lookup (spi_module_id, sclk_pin_id, &sclk_AltFuncId,
                                   miso_pin_id, &miso_AltFuncId,
                                   mosi_pin_id, &mosi_AltFuncId);
    if (rc != 0)
       return (rc);                  // pin_id not valid for this SPI instance

         //-----------------------------------------------------------------
         //           GPIO  Pin  Config  -  for SPI operation
         //
         // Setup the GPIO pins to operate in SPI mode  (pin muxing).
         // Configures SCLK, MISO, and MOSI.
         //-----------------------------------------------------------------
    pull_flags_idx  = (flags & 0x0003);                    // flags for SCLK
    sclk_pull_flags = _g_gpio_pull_flags[pull_flags_idx];
    pull_flags_idx  = ((flags & 0x000C) >> 2);             // flags for MISO
    miso_pull_flags = _g_gpio_pull_flags[pull_flags_idx];
    pull_flags_idx  = ((flags & 0x0030) >> 4);             // flags for MOSI
    mosi_pull_flags = _g_gpio_pull_flags[pull_flags_idx];

    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct)); // clear GPIO struct

#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;   // Handle F0 vs F4 etc HAL inconsistencies
#else
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
#endif

    GPIO_InitStruct.Pin       = sclk_GPIO_pin;
    GPIO_InitStruct.Alternate = sclk_AltFuncId;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // Alt Func Push/Pull
    GPIO_InitStruct.Pull      = sclk_pull_flags;      // Blue NRG uses GPIO_PULLDOWN
    HAL_GPIO_Init (sclk_GPIO_port, &GPIO_InitStruct); // Setup SCLK pin

    GPIO_InitStruct.Pin       = miso_GPIO_pin;
    GPIO_InitStruct.Alternate = miso_AltFuncId;
    GPIO_InitStruct.Pull      = miso_pull_flags;      // Blue NRG uses GPIO_PULLUP
    HAL_GPIO_Init (miso_GPIO_port, &GPIO_InitStruct); // Setup MOSI pin

    GPIO_InitStruct.Pin       = mosi_GPIO_pin;
    GPIO_InitStruct.Alternate = mosi_AltFuncId;
    GPIO_InitStruct.Pull      = mosi_pull_flags;      // Blue NRG uses GPIO_PULLUP
    HAL_GPIO_Init (mosi_GPIO_port, &GPIO_InitStruct); // Setup MOSI pin

#if defined(USE_DMA)
         //----------------------------------------
         //    SPI2  DMA and Interrupts  Enable
         //----------------------------------------
    board_spi_dma_init();
    HAL_NVIC_SetPriority (SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (SPI2_IRQn);
#endif

       //---------------------------------------------------------------------
       // Initialize the SPI MODULE I/O Block
       //---------------------------------------------------------------------
    memset (ioblock, 0, sizeof(SPI_IO_BUF_BLK));   // ensure is cleared out
    ioblock->spi_init   = 1;                       // tag it as been initialized
    ioblock->spi_state  = SPI_STATE_RESET;         // set to initial state

    ioblock->spi_max_timeout = 100;                // ensure minial SPI timeout

    ioblock->spi_master_slave = master_slave;      // save type and mode
    ioblock->spi_mode         = spimode;
    ioblock->spi_handle = pSpiHdl; // Save the SPI handle to use during I/O processing

         //--------------------------------------------------------------
         // Turn on the clock for the physical SPI module
         //--------------------------------------------------------------
    board_spi_enable_clock (spi_module_id);

         //------------------------------------------------------------
         //                      SPI Module Config
         //
         //          Setup HAL info needed for SPI Configuration
         //------------------------------------------------------------
    pSpiHdl->Instance             = spi_hwbase;  // set assoc SPI HW module in HAL TypeDef
    pSpiHdl->Init.Mode            = SPI_MODE_MASTER;
    pSpiHdl->Init.Direction       = SPI_DIRECTION_2LINES;
    pSpiHdl->Init.DataSize        = SPI_DATASIZE_8BIT;
    if (spimode == 0)
      { pSpiHdl->Init.CLKPolarity = SPI_POLARITY_LOW;
        pSpiHdl->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spimode == 1)
      { pSpiHdl->Init.CLKPolarity = SPI_POLARITY_HIGH;
        pSpiHdl->Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spimode == 2)
      { pSpiHdl->Init.CLKPolarity = SPI_POLARITY_LOW;  // CPOL = 0, CPHA = 1
        pSpiHdl->Init.CLKPhase    = SPI_PHASE_2EDGE;
      }
     else if (spimode == 3)
      { pSpiHdl->Init.CLKPolarity = SPI_POLARITY_HIGH; //EASY_SPIN, PRESCALE = 32
        pSpiHdl->Init.CLKPhase    = SPI_PHASE_2EDGE;   //Ada ST7735 LCD SCALE = 8
      }
    pSpiHdl->Init.NSS             = SPI_NSS_SOFT;
    pSpiHdl->Init.FirstBit        = SPI_FIRSTBIT_MSB;
    pSpiHdl->Init.CRCPolynomial   = 7;
    pSpiHdl->Init.CRCCalculation  = SPI_CRCCALCULATION_DISABLED;
    pSpiHdl->Init.TIMode          = SPI_TIMODE_DISABLED;

       //----------------------------------------------------------------------
       // This is literally a pre-scaler value. It only indirectly sets the SPI baud rate.
       // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
       //
       // That's just the way ST's "baud rate generation" works. It's just a
       // prescalar off of a system clock. No fine grain, separate, baud generator.
       //----------------------------------------------------------------------
    pSpiHdl->Init.BaudRatePrescaler = baud_rate_scalar;

    rc = HAL_SPI_Init (pSpiHdl);       // go intialize the SPI module

    __HAL_SPI_ENABLE (pSpiHdl);   // turn on SPI Enable flag SPI_CR1_SPE in CR1

    if (flags & SPI_IO_NON_BLOCKING)
       {       // que I/O and return to caller
         ioblock->spi_blocking = 0;
         board_spi_enable_nvic_irq (spi_module_id);   // turn on interrupts
       }
      else {   // We must block on return of Read/Write calls until I/O complete
             ioblock->spi_blocking = 1;               // turn on blocking mode
           }

    if (rc != HAL_OK)
       return (-1);             // denote SPI init failed
    return (0);                 // denote it worked OK
}


//--------------------------------------------------------------
//                FUTURE  -   DMA  Support
//--------------------------------------------------------------
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


//******************************************************************************
//  board_spi_ALTFUNC_lookup
//
//        Finds/scans for the associated SCLK/MISO/MOSI GPIO Pin's
//        associated ALTERNATE FUNCTION (AF) information.
//        Returns the ALTERNATE_FUNCTION (AF) id for each pin.
//******************************************************************************

int  board_spi_ALTFUNC_lookup (unsigned int module_id, int sclk_gpio_pin, uint32_t *sclk_AltFuncId,
                               int miso_gpio_pin, uint32_t *miso_AltFuncId,
                               int mosi_gpio_pin, uint32_t *mosi_AltFuncId)
{
    SPI_GPIO_BLK   *gpio_blk_base;
    SPI_GPIO_BLK   *gpio_curr_ptr;

    gpio_blk_base = (SPI_GPIO_BLK*) _g_spi_gpio_table_addr [module_id];
    if (gpio_blk_base == 0L)
       return (ERR_SPI_MODULE_NUM_NOT_SUPPORTED);

    *sclk_AltFuncId = 0xFFFF;      // initially clear out the entries
    *miso_AltFuncId = 0xFFFF;
    *mosi_AltFuncId = 0xFFFF;

       // first, scan to locate the associated SCLK pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->spi_pin_role != -1)
      { if (gpio_curr_ptr->spi_pin_number == sclk_gpio_pin)
           { if (gpio_curr_ptr->spi_pin_role == SCLK_PIN_ROLE)
                { *sclk_AltFuncId = gpio_curr_ptr->spi_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                     // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                     // else step to the next entry
      }

       // then, scan to locate the associated MISO pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->spi_pin_role != -1)
      { if (gpio_curr_ptr->spi_pin_number == miso_gpio_pin)
           { if (gpio_curr_ptr->spi_pin_role == MISO_PIN_ROLE)
                { *miso_AltFuncId = gpio_curr_ptr->spi_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                     // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                     // else step to the next entry
      }

       // finally, scan to locate the associated MOSI pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->spi_pin_role != -1)
      { if (gpio_curr_ptr->spi_pin_number == mosi_gpio_pin)
           { if (gpio_curr_ptr->spi_pin_role == MOSI_PIN_ROLE)
                { *mosi_AltFuncId = gpio_curr_ptr->spi_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                     // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                     // else step to the next entry
      }

    if (*sclk_AltFuncId == 0xFFFF || *miso_AltFuncId == 0xFFFF || *mosi_AltFuncId == 0xFFFF)
       return (ERR_SPI_PIN_ID_NOT_SUPPORTED);   // no match found in table

    return (0);                // denote operation completed successfully
}


//******************************************************************************
// board_spi_check_io_completed
//
//          Check if I/O has completed.
//
//          Flags:    SPI_WAIT_FOR_COMPLETE  -  wait till I/O is complete.
//
//          Returns:
//              True  (1) = Completed
//              False (0) = Busy
//                    -1  = completed with error
//******************************************************************************
int  board_spi_check_io_completed (unsigned int spi_module_id, int flags)
{
    SPI_IO_BUF_BLK  *ioblock;

    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

    if (ioblock->spi_state == SPI_STATE_IO_PROCESSING)
       { if ((flags & SPI_WAIT_FOR_COMPLETE) == 0)
            return (0);                   // I/O still busy

             //-----------------------------------------------------------------
             // caller has requested: SPI_WAIT_FOR_COMPLETE.
             //
             // for Low Power or RTOS versions we would do a wait, until
             // the I/O interrupts completed.
             //-----------------------------------------------------------------
//       IO_SEMAPHORE_SET (ioblock->spi_semaphore);
         IO_SEMAPHORE_WAIT (&ioblock->spi_semaphore);  // go into LPM or RTOS proc switch --> See CC3100 LOGIC
       }

    if (ioblock->spi_state == SPI_STATE_ERROR_COMPLETE)
       return (-1);                      // I/O Completed with error

    return (1);                          // I/O Completed successfully
}


//*****************************************************************************
//  board_spi_get_handle
//
//             debugging hook used during board bring up to test new MCUs SPI.
//*****************************************************************************

SPI_HandleTypeDef *board_spi_get_handle (unsigned int spi_module_id)
{
    SPI_HandleTypeDef  *pSpiHdl;

             // get assoc SPI HAL Handle we are using
    pSpiHdl = (SPI_HandleTypeDef*) _g_spi_typedef_handle [spi_module_id];

    return (pSpiHdl);               // return HAL related SPI handle
}


//*****************************************************************************
//  board_spi_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when any SPI interrupt completion (or terminating error) occurs.
//*****************************************************************************
int  board_spi_set_callback (unsigned int spi_module_id,
                             SPI_CB_EVENT_HANDLER callback_function,
                             void *callback_parm)
{
    SPI_IO_BUF_BLK  *ioblock;

    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);
    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts
       //-------------------------------------------------------------------

    ioblock->spi_callback_handler = callback_function;  // save callback info
    ioblock->spi_callback_parm    = callback_parm;

    return (0);                        // denote completed OK
}


//*****************************************************************************
//  board_spi_set_max_timeout
//
//          Set the maximum time that the MCU should wait for an SPI
//          send/receive transaction to complete.
//          If it exceeds this value, signal an error, and cancel the I/O
//          that was in progress.
//
// NOTE THAT WITH ST code, TIMEOUTs are _only_ used with Polled I/O.
//      Are _NOT_ used with Interrupt or DMA driven I/O  !!!
//*****************************************************************************
int  board_spi_set_max_timeout (unsigned int spi_module_id, uint32_t max_timeout)
{
    SPI_IO_BUF_BLK  *ioblock;

    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

    ioblock->spi_max_timeout = max_timeout;  // save Max Timeout value

    return (0);                        // denote completed OK
}


//*****************************************************************************
//  board_spi_stop_io
//
//          An SPI I/O call (read/write/write_read) timed out.
//          Terminate any I/O in progress and reset back to start a new operation.
//
// NOTE THAT WITH ST code, TIMEOUTs are _only_ used with Polled I/O.
//      Are _NOT_ used with Interrupt or DMA driven I/O  !!!
//*****************************************************************************
void  board_spi_stop_io (SPI_IO_BUF_BLK *ioblock)
{
        // ??? CALL HAL IF WE ARE USING HAL for I/O
        // ??? HOW STOP AN SPI OPERATION ?   MUST IT BE RESET BACK LIKE INIT ?

        /* Disable TXE, RXNE and ERR interrupts when using interrupt process */
/// __HAL_SPI_DISABLE_IT (ioblock->spi_handle, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    __HAL_SPI_DISABLE(ioblock->spi_handle);  // issue SPI disable to stop I/O

    ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;  // set our state


//   ??? !!!   Issue error callback to the user    ??? !!!


    __HAL_SPI_ENABLE(ioblock->spi_handle);   // then re-anble SPI for new I/O
}



//*****************************************************************************
//  board_spi_read
//
//           derived from:    EasySpin  SPI   I/O   Routine
//
// @brief  Sets the registers of the EasySpin to their predefined values
//         from easyspin_target_config.h
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************

int  board_spi_read (unsigned int spi_module_id, uint8_t *receive_buffer,
                     int max_buf_length, int flags)
{
    SPI_IO_BUF_BLK     *ioblock;
    SPI_HandleTypeDef  *pSpiHdl;
    SPI_TypeDef        *spibase;
    int                rc;

       // extract the actual SPI module from the encoded version of spi_module_id
    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

    pSpiHdl   = ioblock->spi_handle;           // get associated HAL handle
    if (ioblock->spi_state == SPI_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);  // still working on a previous I/O

    ioblock->spi_buffer = receive_buffer;    // save ptr to User I/O buf
    ioblock->spi_length = max_buf_length;    // save amount of data to rcv
    ioblock->spi_expiry_time = ioblock->spi_max_timeout;   // set any max timeout

    ioblock->spi_state = SPI_STATE_IO_PROCESSING; // denote I/O is now in progress

// for now, issue HAL Send calls.  In future, low level equivalent   ??? !!! WVD

    spibase = (SPI_TypeDef*) pSpiHdl->Instance;     // Point to SPI peripheral HW
    if (ioblock->spi_blocking)
       {      // BLOCKING I/O - wait until I/O is complete, before return to app
//       if (ioblock->spi_mode == SPI_SLAVE)
//          { spibase->DR = (*transmit_buffer);     // load a byte of data
//            buf_length--;
//          }
         while (max_buf_length > 0)
            {     // wait until see TXE (TX buffer empty) before send next byte

// ??? add timeout logic
// tickstart = HAL_GetTick();
// if (ioblock->spi_max_timeout > 0)
//    tickend = tickstart + ioblock->spi_max_timeout;
//    else tickend > 0;
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_TXE) == RESET)  // wait till get TX empty
                {
//                if (_g_systick_millisecs > tickend && tickend != 0)   // WRAP_AROUND ISSUE
//                   break - timeout;
                }
              spibase->DR = 0xAA;                  // send a dummy byte of data
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET) ; // wait till get RX rcvd
              *receive_buffer++ = spibase->DR;    // read and save rcvd SPI byte
              max_buf_length--;                   // deduct amount left
            }
///      while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET) ; // wait till get last RX rcvd  ??? IS THIS FINAL RCV NEEDED (or TX if SLAVE) ???
///      *receive_buffer++ = spibase->DR;          // read + save any final SPI rcvd byte
         ioblock->spi_state = SPI_STATE_IO_COMPLETED;   // Tag as successful I/O
         rc = HAL_OK;
//       rc = HAL_SPI_Receive (ioblock->spi_handle,
//                             receive_buffer, max_buf_length,
//                             ioblock->spi_max_timeout); // ST timeout ONLY used in Poll mode
         if (rc == HAL_OK)
            {
//            rc = OS_WAIT (ioblock->spi_expiry_time); // Wait for the I/O to complete
//            if (rc != 0)
//               board_spi_stop_io (ioblock); // I/O timed out. Stop any I/O call in progress
              ioblock->spi_state = SPI_STATE_IO_COMPLETED;  // Tag as successful I/O
            }
       }
      else {    // NON-BLOCKING I/O - schedule it, then return to user app.
             rc = HAL_SPI_Receive_IT (ioblock->spi_handle,
                                      receive_buffer,
                                      max_buf_length);
           }

    if (rc != HAL_OK)
       { ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;
         return (-1);
       }

    return (0);        // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//*****************************************************************************

int  board_spi_write (unsigned int spi_module_id, uint8_t *transmit_buffer,
                      int buf_length, int flags)
{
    SPI_IO_BUF_BLK     *ioblock;
    SPI_HandleTypeDef  *pSpiHdl;
    SPI_TypeDef        *spibase;
    int                rc;
    uint32_t           tickstart,  tickend;
    uint8_t            dummy;

    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

    pSpiHdl   = ioblock->spi_handle;            // get associated HAL handle
    if (ioblock->spi_state == SPI_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

    ioblock->spi_buffer = transmit_buffer;    // save ptr to User I/O buf
    ioblock->spi_length = buf_length;         // save amount of data to send
    ioblock->spi_expiry_time = ioblock->spi_max_timeout;   // set any max timeout

    ioblock->spi_state = SPI_STATE_IO_PROCESSING; // denote I/O is now in progress


// for now, issue HAL Send calls.  In future, low level equivalent   ??? !!! WVD

    spibase = (SPI_TypeDef*) pSpiHdl->Instance;     // Point to SPI peripheral HW
    if (ioblock->spi_blocking)
       {        // BLOCKING I/O - wait until operation is complete, before return to app
//       if (ioblock->spi_mode == SPI_SLAVE)
//          { spibase->DR = (*transmit_buffer);     // load a byte of data
//            buf_length--;
//          }
#if BROKEN_2015_08_10
         while (buf_length > 0)
            {     // wait until see TXE (TX buffer empty) before send next byte

// ??? add timeout logic
// tickstart = HAL_GetTick();
// if (ioblock->spi_max_timeout > 0)
//    tickend = tickstart + ioblock->spi_max_timeout;
//    else tickend > 0;
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_TXE) == RESET)     // wait till get TX empty
                {
//                if (_g_systick_millisecs > tickend && tickend != 0)   // WRAP_AROUND ISSUE
//                   break - timeout;
                }
              spibase->DR = (*transmit_buffer++);   // send a byte of data
              buf_length--;                         // deduct amount left
// 08/10/15 - HANGS IN HERE - TILT !!!
// Is this another FIFO screw-over ???  -  What is the trick to get turn them off ?
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET) ; // wait till get last RX rcvd
              dummy = spibase->DR;                  // read and discard any final SPI rcvd byte
            }
         while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET) ; // wait till get last RX rcvd
///      while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_BSY) == RESET) ;  // wait till get SPI Transmit/Rcv complete (Not as reliable)
         dummy = spibase->DR;                       // read and discard any final SPI rcvd byte
         rc = HAL_OK;
#else
         rc = HAL_SPI_Transmit (ioblock->spi_handle,
                                transmit_buffer, buf_length,
                                ioblock->spi_max_timeout); // ST timeout ONLY used in Poll mode
#endif
         if (rc == HAL_OK)
            {
//            rc = OS_WAIT (ioblock->spi_expiry_time); // Wait for the I/O to complete
//            if (rc != 0)
//               board_spi_stop_io (ioblock); // I/O timed out. Stop any I/O call in progress
              ioblock->spi_state = SPI_STATE_IO_COMPLETED;  // Tag as successful I/O
            }
       }

      else {    // NON-BLOCKING I/O - schedule it, then return to user app.
             rc = HAL_SPI_Transmit_IT (ioblock->spi_handle,
                                       transmit_buffer,
                                       buf_length);
           }

    if (rc != HAL_OK)
       { ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;
         return (-1);
       }

    return (0);        // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//  board_spi_write_read
//
// @brief  Sets the registers of the EasySpin to their predefined values
//         from easyspin_target_config.h
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************
#if defined(SPIRIT1_ST_SHIELD)
#define RADIO_SPI_TIMEOUT_MAX                   ((uint32_t)1000)

      extern  SPI_HandleTypeDef   pSpiHandle;           // ORIGINAL SPIRIT1 code
              HAL_StatusTypeDef  status = HAL_OK;       // ORIGINAL CODE
      extern  uint32_t           SpiTimeout; // = RADIO_SPI_TIMEOUT_MAX;   /* Value of Timeout when SPI communication fails */
              uint8_t            Value;
#endif

int  board_spi_write_read (unsigned int spi_module_id, uint8_t *transmit_buffer,
                           uint8_t *receive_buffer, int buf_length,
                           int flags)
{
    SPI_IO_BUF_BLK     *ioblock;
    SPI_HandleTypeDef  *pSpiHdl;
    SPI_TypeDef        *spibase;
    uint32_t           tickstart,  tickend;
    int                rc;

       // extract the actual SPI module from the encoded version of spi_module_id
    if (spi_module_id > MAX_SPI)
       return (ERR_SPI_NUM_OUT_OF_RANGE);

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_module_id];  // get assoc I/O block

    pSpiHdl   = ioblock->spi_handle;            // get associated HAL handle
    if (ioblock->spi_state == SPI_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

    ioblock->spi_buffer = transmit_buffer;    // save ptr to User I/O buf
    ioblock->spi_length = buf_length;         // save amount of data to send
    ioblock->spi_expiry_time = ioblock->spi_max_timeout;   // set any max timeout

    ioblock->spi_state = SPI_STATE_IO_PROCESSING; // denote I/O is now in progress

// for now, issue HAL Send calls.  In future, low level equivalent   ??? !!! WVD
#if defined(_RXNE_IS_HANGING)          // F3 02 and F3 34
//#if defined(STM32F302x8) || defined(STM32F334x8)          // F3 02 and F3 34
    while ((hspi_w5200.Instance->SR & SPI_FLAG_TXE) == RESET) ;
//  while (SPI_I2S_GetFlagStatus(SPI_MODULE, SPI_I2S_FLAG_TXE) == RESET) ;
//  if ((hspi->Instance->SR & SPI_FLAG_TXE) == SPI_FLAG_TXE)

    hspi_w5200.Instance->DR = data_byte;
//  SPI_SendData8 (SPI_MODULE, data_byte);
//  hspi->Instance->DR = *((uint16_t*) hspi->pTxBuffPtr);

    while ((hspi_w5200.Instance->SR & SPI_FLAG_RXNE) == RESET) ;
/// while (SPI_I2S_GetFlagStatus(SPI_MODULE, SPI_I2S_FLAG_RXNE) == RESET) ;
//  if ((hspi->Instance->SR & SPI_FLAG_RXNE) == SPI_FLAG_RXNE)

         // EasySpin debug lesson:
         //        for 1 byte I/O, the reply byte is in H.O of halfword !!!
         //        xic = 0x0055   dic = 0x5500  !!!   ==> shift needed

    rcvd_char = hspi_w5200.Instance->DR;
    return (rcvd_char);
//  return SPI_ReceiveData8 (SPI_MODULE);
//  *((uint16_t*) hspi->pRxBuffPtr) = hspi->Instance->DR;
#endif

    spibase = (SPI_TypeDef*) pSpiHdl->Instance;     // Point to SPI peripheral HW
    if (ioblock->spi_blocking)
       {      //----------------------------------------------------------------
              // BLOCKING I/O - wait until I/O is complete, before return to app
              //----------------------------------------------------------------
//#if defined(SPIRIT1_ST_SHIELD)
              //-----------------------------------------------------------------------------------------
              //  09/11/15 the following logic works for W5200 and L6474, but fails for Spirit1.  WTF !!!
              //-----------------------------------------------------------------------------------------
//       if (ioblock->spi_mode == SPI_SLAVE)
//          { spibase->DR = (*transmit_buffer);     // load a byte of data
//            buf_length--;
//          }
         while (buf_length > 0)
            {     // wait until see TXE (TX buffer empty) before send next byte

// ??? add timeout logic
// tickstart = HAL_GetTick();
// if (ioblock->spi_max_timeout > 0)
//    tickend = tickstart + ioblock->spi_max_timeout;
//    else tickend > 0;
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_TXE) == RESET)  // wait till get TX empty
                {
//                if (_g_systick_millisecs > tickend && tickend != 0)   // WRAP_AROUND ISSUE
//                   break - timeout;
                }
              spibase->DR = (*transmit_buffer++);  // send a byte of data
              while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET)
                 ;                   // wait till get RX rcvd  <------  09/11/15 HANGS FOREVER ON THIS.  Verify spibase is correct !
              *receive_buffer++ = spibase->DR;    // read and save rcvd SPI byte
              buf_length--;                       // deduct amount left
            }
///      while (__HAL_SPI_GET_FLAG(pSpiHdl, SPI_FLAG_RXNE) == RESET) ; // wait till get last RX rcvd  ??? IS THIS FINAL RCV NEEDED (or TX if SLAVE) ???
///      *receive_buffer++ = spibase->DR;          // read + save any final SPI rcvd byte
         rc = HAL_OK;
//#else
#if BADLY_BROKEN_ST_CODE
   Value = *transmit_buffer;
            //--------------------------------------------------
            //                ORIGINAL  ST  CODE  - THIS IS HORRIBLY BROKEN !!!
            //--------------------------------------------------
// while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET)
     ;                                        // wait till SPI TX buffer is free
// status = HAL_SPI_Transmit (&pSpiHandle, (uint8_t*) &Value, 1, SpiTimeout);

  while (__HAL_SPI_GET_FLAG(pSpiHdl , SPI_FLAG_TXE) == RESET)
     ;                                        // wait till SPI TX buffer is free
   status = HAL_SPI_Transmit (pSpiHdl, (uint8_t*) &Value, 1, SpiTimeout);

      /* Check the communication status */
  if (status != HAL_OK)
     {
            /* Execute user timeout callback */
    // SPI_Error();
       ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;
       return (-1);
     }

#endif
         ioblock->spi_state = SPI_STATE_IO_COMPLETED;   // Tag as successful I/O
       }
      else {    // NON-BLOCKING I/O - schedule it, then return to user app.
             rc = HAL_SPI_TransmitReceive_IT (ioblock->spi_handle,
                                              transmit_buffer,
                                              receive_buffer,
                                              buf_length);
           }

    if (rc != HAL_OK)
       { ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;
         return (-1);
       }

    return (0);        // tell user it was started (Non-Block) or completed OK
}


   extern    int   rupt_module_id;            // TEMP_HACK


/*************************************************************************
*                                 TEMP  ISR   HACK
* @brief  SPI error callbacks.
* @param  hspi: SPI handle
*
* @note   This example shows a simple way to report transfer error,
*         and you canadd your own implementation.
* @retval None
*/
void HAL_SPI_ErrorCallback (SPI_HandleTypeDef *hspi)
{
    SPI_IO_BUF_BLK     *ioblock;

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [rupt_module_id];    // get assoc I/O block
    ioblock->spi_state = SPI_STATE_ERROR_COMPLETE;   // set ending status

    if (ioblock->spi_callback_handler != 0L)
       {             // Invoke user callback for the interrupt
         (ioblock->spi_callback_handler) (ioblock->spi_callback_parm,
                                          rupt_module_id, -1);     // signal error
       }
}


/************************************************************************
*                                 TEMP  ISR   HACK
*
* @brief  TxRx Transfer completed callback.
* @param  hspi: SPI handle.
*
* @note   This example shows a simple way to report end of
*         Interrupt TxRx transfer, and
*         you can add your own implementation.
* @retval None
*/
void  HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_IO_BUF_BLK     *ioblock;

    ioblock = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [rupt_module_id];    // get assoc I/O block
    ioblock->spi_state = SPI_STATE_IO_COMPLETED;  // Tag as successful I/O

    if (ioblock->spi_callback_handler != 0L)
       {             // Invoke user callback for the interrupt
         (ioblock->spi_callback_handler) (ioblock->spi_callback_parm,
                                          rupt_module_id, 0);   // signal good completion
       }
}


//******************************************************************************
//                     COMMON    SPI     ISR / IRQ     Handler
//
// The spi_interrupt_number ranges from 1 to n, and it indicates which
// SPIx_IRQHandler invoked us: e.g. 1 = SPI1_IRQHandler, 2 = SPI2_IRQHandler.
// The handler is directly tied to the SPI modules SPI1, SPI2, ...
//******************************************************************************
    int   rupt_module_id = 0;            // TEMP_HACK

void  board_spi_IRQ_Handler (int spi_interrupt_number)
{
    SPI_IO_BUF_BLK  *ioblk;

    rupt_module_id = spi_interrupt_number;

    ioblk = (SPI_IO_BUF_BLK*) _g_spi_io_blk_address [spi_interrupt_number];

    IO_SEMAPHORE_RELEASE (&ioblk->spi_semaphore);  // clear any semaphore

    HAL_SPI_IRQHandler (ioblk->spi_handle);            // initial pass
}

/******************************************************************************/
