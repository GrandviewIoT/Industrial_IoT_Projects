
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             board_STM32_i2c.c
//
//
//  Common Logic for I2C support for STM32 MCUs.
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
//    05/30/15 - Created for Industrial IoT OpenSource project.  Duq
//    07/30/15 - Reworked to provide better factoring. Worked first shot. Duq
//    08/10/15 - Tweaked Interrrupt Handling. Duquaine
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
#include "device_config_common.h"     // device config stuff for I2C_TIMING, etc
#include "boarddef.h"


#define  MAX_I2C_ID    100                // ??? !!  TWEAK THIS  WVD   !!! ???

typedef struct i2c_gio_def                  /* I2C GPIO pin definitions */
    {
        uint32_t     i2c_chan_alt_func;     /* I2C Alternate Function Id     */
        int8_t       i2c_pin_role;          /* acts in SCL or SDA role       */
        uint8_t      i2c_pin_number;        /* logical pin number 0-255      */
    } I2C_GPIO_BLK;

                    // valid values for i2c_pin_role
#define  SCL_PIN_ROLE   1
#define  SDA_PIN_ROLE   2



typedef struct i2c_io_block            /* I2C I/O Control Block (I/O Buffers) */
    {
        uint8_t      *i2c_buffer;      /* Ptr to current spot in User I/O buf */
        uint16_t     i2c_length;       /* Amount of data left to send/rcv     */
        uint8_t      i2c_init;         /*     1 = has been initialized        */
        uint8_t      i2c_state;        /* Current state of the I2C module     */
        uint8_t      i2c_use_interrupts; /* 1 = use interrupts,     0 = poll  */
        uint8_t      i2c_blocking;     /* 1 = I/O is blocking. Wait till I/O is
                                       **  complete before return to user app */
        uint8_t      i2c_master_slave; /* 1 = Master, 2 = Slave               */
        int          i2c_semaphore;    /* I/O wait semaphore                  */
        uint32_t     i2c_max_timeout;  /* max time to wait for I/O (millisec) */
        uint32_t     i2c_expiry_time;  /* timeout deadline for I2C I/O response */
     I2C_HandleTypeDef    *i2c_handle; /* HAL I2C Handle to use for this I/O  */
     I2C_CB_EVENT_HANDLER i2c_callback_handler;  /* optional callback routine */
        void              *i2c_callback_parm;    /* user callback parm        */
    } I2C_IO_BUF_BLK;

                    // valid values for i2c_state
#define  I2C_STATE_UNINITIALIZED    0   /* I2C has not been initialized */
#define  I2C_STATE_RESET            1   /* I2C is reset, not in use     */
#define  I2C_STATE_IO_PROCESSING    2   /* I2C I/O is in progress       */
#define  I2C_STATE_IO_PEND_COMPLETE 3   /* I2C I/O waiting for final complete */
#define  I2C_STATE_IO_COMPLETED     4   /* I2C I/O is fully complete    */
#define  I2C_STATE_ERROR_COMPLETE   5   /* I2C I/O completed with an error */


void board_i2c_enable_nvic_irq (int module_id);
void board_i2c_stop_io (I2C_IO_BUF_BLK *ioblock);  // internal routines protos
int  board_i2c_write_byte (I2C_IO_BUF_BLK *ioblock, uint8_t byte_Value);
void Internal_HAL_I2C_TxRxCpltCallback (I2C_HandleTypeDef *hi2c, int rupt_num);


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

#if defined(__STM32F072__) || defined(STM32F072xB)
//                                         STM32 - F072  Nucleo
#include "STM32_F0/board_F0_tables_i2c.c"
#define  MAX_I2C   2
#endif


#if defined(__STM32F091__) || defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
#include "STM32_F0/board_tables_F0_i2c.c"
#define  MAX_I2C   2
#endif


#if defined(__STM32F103__) || defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
#include "STM32_F1/board_F1_tables_i2c.c"
#define  MAX_I2C   4
#endif


#if defined(STM32F303xC) || defined(STM32F303xE)
//                                         STM32 - F303  Nucleo and Discovery
#include "STM32_F3/board_F303_tables_i2c.c"
#define  MAX_I2C   2
#endif


#if defined(STM32F334x8)
//                                         STM32 - F334  Nucleo and Discovery
#include "STM32_F3/board_F334_tables_i2c.c"
#define  MAX_I2C   1
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401/F411  Nucleos
#include "STM32_F4/board_F4x1_tables_i2c.c"
#define  MAX_I2C   4
#endif

#if defined(STM32F446xx)
//                                         STM32 - F446  Nucleo
#include "STM32_F4/board_F446_tables_i2c.c"
#define  MAX_I2C   4
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
#include "STM32_F7/board_F7_tables_i2c.c"
#define  MAX_I2C   6
#endif


#if defined(__STM32L053__)  ||  defined(STM32L053xx)
//                                         STM32 - L053  Nucleo
#include "STM32_L0/board_L0_tables_i2c.c"
#define  MAX_I2C   2
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo
#include "STM32_L1/board_L1_tables_i2c.c"
#define  MAX_I2C   2
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
#include "STM32_L4/board_L4_tables_i2c.c"
#define  MAX_I2C   3
#endif


//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES and DEFINEs
//
//                                for
//
//                                I2C
//*****************************************************************************
//*****************************************************************************

    int           init_mod_id   = 0;     // DEBUG ONLY
    int           init_mod_num  = 0;     // DEBUG ONLY
I2C_IO_BUF_BLK    *init_ioblock = 0L;    // DEBUG ONLY
I2C_HandleTypeDef *init_pI2cHdl = 0L;    // DEBUG ONLY

    int           writ_mod_id   = 0;     // DEBUG ONLY
    int           writ_mod_num  = 0;     // DEBUG ONLY
I2C_IO_BUF_BLK    *writ_ioblock = 0L;    // DEBUG ONLY
I2C_HandleTypeDef *writ_pI2cHdl = 0L;    // DEBUG ONLY


int  board_i2c_enable_clock (int module_id);    // Prototypes for internal rtns
int  board_i2c_get_module_ioblk (unsigned int i2c_module_lid, I2C_IO_BUF_BLK **ret_ioblock);

extern   const  int  _g_gpio_pull_flags[];  // defined in board_XX_tables_i2c.c


//*****************************************************************************
//*****************************************************************************
//                               I2C   Routines
//*****************************************************************************
//*****************************************************************************

/*******************************************************************************
* board i2c init function
*
*         Configures this MCU's I2C peripheral and its associated GPIOs.
*
*         The I2C Module Id specifies, both the I2C Module and the sub-variant
*         pins to use for that that module.
*
*         The module_id is split:
*             bottom low order 4 bits (0..3) = pin variant.
*             next nibble up (bits 4-7)      = actual I2C module number 0-5
*
*
*                SCL      SDA
*                ----     ----
*         I2C1 = PA5   /  PA7
*           "    PB3   /  PB5    (alternate pin mode)
*         I2C2 = PB13  /  PC3
*           "    PB13  /  PB15   (alternate pin mode)
*         I2C3 = PC10  /  PC12
*           "    PB3   /  PB5    (alternate pin mode)
*
*  flags:   use_dma,   USES_BLUENRG_BLE, GPIO_OUT_PULLUP, GPIO_IN_PULL_UP/DOWN
*
* 07/31/15
*    Added option for user to pass in his own I2C_HandleTypeDef handle, instead
*    of using our internal one. This preserves some vendor I2C legacy code, that
*    have a very high dependency on using their own I2C_HandleTypeDef handle.
*******************************************************************************/

int  board_i2c_init (unsigned int i2c_module_id, int scl_pin_id, int sda_pin_id,
                     int master_slave, int my_local_address,
                     long i2c_baud_timing,  int flags,
                     I2C_HandleTypeDef *caller_hi2c_ptr)   // TEMP HACK 07/30/15
{
    I2C_HandleTypeDef  *pI2cHdl;
    I2C_TypeDef        *i2c_hwbase;
    I2C_IO_BUF_BLK     *ioblock;
    int                rc;
    int                pull_flags_idx;
    int                scl_pull_flags;
    int                sda_pull_flags;
    GPIO_TypeDef       *scl_GPIO_port;    /* SCL GPIO port   */
    uint32_t           scl_GPIO_pin;      /* SCL GPIO pin    */
    uint32_t           scl_AltFuncId;     /* SCL Alt Func Id (pin mux) */
    GPIO_TypeDef       *sda_GPIO_port;    /* SDA GPIO port   */
    uint32_t           sda_GPIO_pin;      /* SDA GPIO pin    */
    uint32_t           sda_AltFuncId;     /* SDA Alt Func Id (pin mux) */
    GPIO_InitTypeDef   GPIO_InitStruct;

init_mod_id  = i2c_module_id;  // DEBUG ONLY

    if (i2c_module_id > MAX_I2C_ID)
       return (ERR_I2C_MODULE_ID_OUT_OF_RANGE);

         //--------------------------------------------------------------
         // Get pointer to the I2C I/O Block to use for this module
         //--------------------------------------------------------------
    ioblock = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [i2c_module_id];
init_ioblock = ioblock;         // DEBUG ONLY

         //---------------------------------------------------------------------
         // If module was already initialized - skip doing it again, and issue a warning
         //---------------------------------------------------------------------
    if (ioblock->i2c_init)
       return (WARN_I2C_WAS_ALREADY_INITIALIZED);

       //---------------------------------------------------------------------
       // Get pointers to I2C MODULE Hardware base address, and TypeDef Handle
       //---------------------------------------------------------------------
    i2c_hwbase = (I2C_TypeDef*) _g_i2c_HW_module_base [i2c_module_id];
    pI2cHdl    = (I2C_HandleTypeDef*) _g_i2c_typedef_handle_addr[i2c_module_id];
    if (caller_hi2c_ptr != 0L)
       {   // caller wants to use _his_ I2C_TypeDef handle, not our internal one
         pI2cHdl = caller_hi2c_ptr;              // TEMP HACK 07/30/15t
       }
init_pI2cHdl   = pI2cHdl;          // DEBUG ONLY

       //---------------------------------------------------------------------
       // Locate the associated GPIO Port and GPIO Pin for SCL and SDA pins
       //---------------------------------------------------------------------
    rc = board_gpio_pin_lookup (scl_pin_id, &scl_GPIO_port, &scl_GPIO_pin);
    if (rc == 0)
       rc = board_gpio_pin_lookup (sda_pin_id, &sda_GPIO_port, &sda_GPIO_pin);
    if (rc != 0)
       return (rc);                  // invalid pin_id was given to us

       //---------------------------------------------------------------------
       // Locate the associated GPIO ALTERNATE_FUNCTION (AF) information.
       //---------------------------------------------------------------------
    rc = board_i2c_ALTFUNC_lookup (i2c_module_id, scl_pin_id, &scl_AltFuncId,
                                   sda_pin_id, &sda_AltFuncId);
    if (rc != 0)
       return (rc);                  // pin_id not valid for this I2C instance

         //-----------------------------------------------------------------
         //           GPIO  Pin  Config  -  for I2C operation
         //
         // Setup the GPIO pins to operate in I2C mode  (pin muxing).
         // Configures SCL and SDA pins.
         //-----------------------------------------------------------------
    pull_flags_idx = (flags & 0x0003);                     // flags for SCL
    scl_pull_flags = _g_gpio_pull_flags[pull_flags_idx];
    pull_flags_idx = ((flags & 0x0030) >> 4);              // flags for SDA
    sda_pull_flags = _g_gpio_pull_flags[pull_flags_idx];

    memset (&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct)); // clear struct

#if defined(GPIO_SPEED_FAST)
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;   // Handle F0 vs F4 etc HAL inconsistencies
#else
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
#endif
    GPIO_InitStruct.Pin       = scl_GPIO_pin;;
    GPIO_InitStruct.Alternate = scl_AltFuncId;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;     // Alt Func Open_Drain
    GPIO_InitStruct.Pull      = scl_pull_flags;
    HAL_GPIO_Init (scl_GPIO_port, &GPIO_InitStruct); // Setup SCL pin

    GPIO_InitStruct.Pin       = sda_GPIO_pin;
    GPIO_InitStruct.Alternate = sda_AltFuncId;
    GPIO_InitStruct.Pull      = sda_pull_flags;
    HAL_GPIO_Init (sda_GPIO_port, &GPIO_InitStruct); // Setup SDA pin

       //---------------------------------------------------------------------
       // Initialize the I2C MODULE I/O Block
       //---------------------------------------------------------------------
    ioblock->i2c_init  = 1;                      // tag it as been initialized
    ioblock->i2c_state = I2C_STATE_RESET;        // set to initial state

    ioblock->i2c_master_slave = master_slave;    // save type and mode

    ioblock->i2c_handle = pI2cHdl; // Save I2C handle to use during I/O processing

         //--------------------------------------------------------------
         // Turn on the clock for the physical I2C module
         //--------------------------------------------------------------
    board_i2c_enable_clock (i2c_module_id);

#if defined(USE_DMA)
         //----------------------------------------
         //    I2C2  DMA and Interrupts  Enable
         //----------------------------------------
    board_i2c_dma_init();
    HAL_NVIC_SetPriority (I2C2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (I2C2_IRQn);
#endif

         //------------------------------------------------------------
         //                      I2C Module Config
         //
         //          Setup HAL info needed for I2C Configuration
         //------------------------------------------------------------
    memset (pI2cHdl, 0, sizeof(I2C_HandleTypeDef));    // ensure is cleared out

    pI2cHdl->Instance             = i2c_hwbase;  // set assoc I2C HW module in HAL TypeDef
    pI2cHdl->Init.OwnAddress1     = my_local_address;  // set our local address

/// pI2cHdl->Init.Mode            = I2C_MODE_MASTER;   // Master vs Slave is selected by API calls

    pI2cHdl->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
/// pI2cHdl->Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;  // F303 Discovery - MAKE CONFIG OPTION FLAG ?

#if defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) \
 || defined(USE_STM32L1XX_NUCLEO)
             // F4 and L1 use Baud based Clock Speed
    pI2cHdl->Init.ClockSpeed = i2c_baud_timing;      // default is 400K
    pI2cHdl->Init.DutyCycle  = 0;   // ??? !!! COMPUTE THIS BASED ON BAUD ??? WVD ??? !!!
#endif

#if defined(USE_STM32F0XX_NUCLEO) || defined(STM32F072xB) || defined(STM32F091xC) \
 || defined(USE_STM32F3XX_NUCLEO) || defined(USE_STM32746G_DISCO) \
 || defined(STM32L053xx) || defined(STM32L476xx)
             // F0, F3, F7, L0, and L4 use encoded Timing construct
    pI2cHdl->Init.Timing     = i2c_baud_timing;
#endif

#if defined(USE_STM32F3XX_NUCLEO) || defined(USE_STM32746G_DISCO)
              // F303 & F7 Discovery
    pI2cHdl->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
    pI2cHdl->Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
#endif
            //----------------------------------------
            //     Initialize/configure I2C module
            //----------------------------------------
    rc = HAL_I2C_Init (pI2cHdl);     // go intialize the I2C module

#if defined(USE_STM32F3XX_NUCLEO) || defined(USE_STM32746G_DISCO)
          // Enable the Analog I2C Filter - F303, F7_46 Discovery
    HAL_I2CEx_AnalogFilter_Config (pI2cHdl, I2C_ANALOGFILTER_ENABLED);  // !!! CONFIG OPTION !!!
//  HAL_I2CEx_ConfigAnalogFilter (pI2cHdl, I2C_ANALOGFILTER_ENABLE);
#endif

            //----------------------------------------
            //     Enable I2C module via CR1
            //----------------------------------------
//  __HAL_I2C_ENABLE (pI2cHdl);      // turn on I2C Enable flag I2C_CR1_SPE in CR1 WVD disabled 09/15/15

    if (flags & I2C_IO_USE_INTERRUPTS)
       {    //----------------------------------------
            //     Turn on Interrupts for I2C
            //----------------------------------------
         ioblock->i2c_use_interrupts = 1;          // denote using rupts

// NEED TO FIX THE FOLLOWING        ??? !!! WVD ??? !!!
// FIX   board_i2c_enable_nvic_irq (i2c_module_id);  // turn on interrupts

       }

    if (flags & I2C_IO_NON_BLOCKING)
       {       // que I/O and return to caller
         ioblock->i2c_blocking = 0;
       }
      else {   // We must block on Read/Write calls until I/O is complete
             ioblock->i2c_blocking = 1;            // turn on blocking mode
           }

    if (rc != HAL_OK)
       return (-1);             // denote I2C init failed
    return (0);                 // denote it worked OK
}


//--------------------------------------------------------------
//                FUTURE  -   DMA  Support
//--------------------------------------------------------------
                                  /* Definition for I2Cx clock resources */
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
/* Definition for I2Cx's DMA */
#define I2Cx_TX_DMA_CHANNEL              DMA_CHANNEL_0
#define I2Cx_TX_DMA_STREAM               DMA1_Stream4
#define I2Cx_RX_DMA_CHANNEL              DMA_CHANNEL_0
#define I2Cx_RX_DMA_STREAM               DMA1_Stream3

/* Definition for I2Cx's NVIC */
#define I2Cx_DMA_TX_IRQn                 DMA1_Stream4_IRQn
#define I2Cx_DMA_RX_IRQn                 DMA1_Stream3_IRQn
#define I2Cx_DMA_TX_IRQHandler           DMA1_Stream4_IRQHandler
#define I2Cx_DMA_RX_IRQHandler           DMA1_Stream3_IRQHandler

#if defined(USE_DMA)
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  DMAx_CLK_ENABLE();

  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = I2Cx_TX_DMA_STREAM;

  hdma_tx.Init.Channel             = I2Cx_TX_DMA_CHANNEL;
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

      /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA (hspi, hdmatx, hdma_tx);

      /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = I2Cx_RX_DMA_STREAM;

  hdma_rx.Init.Channel             = I2Cx_RX_DMA_CHANNEL;
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

      /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA (hspi, hdmarx, hdma_rx);

      /*##-4- Configure the NVIC for DMA #########################################*/
      /* NVIC configuration for DMA transfer complete interrupt (I2C3_TX) */
  HAL_NVIC_SetPriority (I2Cx_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ (I2Cx_DMA_TX_IRQn);

      /* NVIC configuration for DMA transfer complete interrupt (I2C3_RX) */
  HAL_NVIC_SetPriority (I2Cx_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (I2Cx_DMA_RX_IRQn);
#endif                                        //  #if defined(USE_DMA)

#if defined(USE_DMA)
              //------------------------------------------
              // DMA init for I2C RX and TX directions
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
void  board_i2c_dma_init (void)
{
#if defined(DMA_ENABLED)
    __DMA1_CLK_ENABLE();               // DMA controller clock enable

        //--------------------------------------------
        //    DMA interrupts init
        //--------------------------------------------
    HAL_NVIC_SetPriority (DMA1_Channel4_IRQn, 0, 0);   // I2C RX
    HAL_NVIC_EnableIRQ (DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority (DMA1_Channel5_IRQn, 0, 0);   // I2C TX
    HAL_NVIC_EnableIRQ (DMA1_Channel5_IRQn);
#endif
}


//******************************************************************************
//  board_i2c_ALTFUNC_lookup
//
//        Finds/scans for the associated SCLK/MISO/MOSI GPIO Pin's
//        associated ALTERNATE FUNCTION (AF) information.
//        Returns the ALTERNATE_FUNCTION (AF) id for each pin.
//******************************************************************************

int  board_i2c_ALTFUNC_lookup (unsigned int module_id,
                               int scl_gpio_pin, uint32_t *scl_AltFuncId,
                               int sda_gpio_pin, uint32_t *sda_AltFuncId)
{
    I2C_GPIO_BLK   *gpio_blk_base;
    I2C_GPIO_BLK   *gpio_curr_ptr;

    gpio_blk_base = (I2C_GPIO_BLK*) _g_i2c_gpio_table_addr [module_id];
    if (gpio_blk_base == 0L)
       return (ERR_I2C_MODULE_NUM_NOT_SUPPORTED);

    *scl_AltFuncId = 0xFFFF;      // initially clear out the entries
    *sda_AltFuncId = 0xFFFF;

       // first, scan to locate the associated SCL pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->i2c_pin_role != -1)
      { if (gpio_curr_ptr->i2c_pin_number == scl_gpio_pin)
           { if (gpio_curr_ptr->i2c_pin_role == SCL_PIN_ROLE)
                { *scl_AltFuncId = gpio_curr_ptr->i2c_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                       // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                       // else step to the next entry
      }

       // then, scan to locate the associated SDA pin support in the table
    gpio_curr_ptr = gpio_blk_base;
    while (gpio_curr_ptr->i2c_pin_role != -1)
      { if (gpio_curr_ptr->i2c_pin_number == sda_gpio_pin)
           { if (gpio_curr_ptr->i2c_pin_role == SDA_PIN_ROLE)
                { *sda_AltFuncId = gpio_curr_ptr->i2c_chan_alt_func;  // pass back associated Alternate Function Id
                  break;                       // we found desired pin. bail out
                }
           }
        gpio_curr_ptr++;                       // else step to the next entry
      }

    if (*scl_AltFuncId == 0xFFFF || *sda_AltFuncId == 0xFFFF)
       return (ERR_I2C_PIN_ID_NOT_SUPPORTED);   // no match found in table

    return (0);                // denote operation completed successfully
}


//******************************************************************************
// board_i2c_check_io_completed
//
//          Check if I/O has completed.
//
//          Flags:    I2C_WAIT_FOR_COMPLETE  -  wait till I/O is complete.
//
//          Returns:
//              True  (1) = Completed
//              False (0) = Busy
//                    -1  = completed with error
//******************************************************************************
int  board_i2c_check_io_completed (unsigned int i2c_module_id, int flags)
{
    I2C_IO_BUF_BLK  *ioblock;
    int             rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       { if ((flags & I2C_WAIT_FOR_COMPLETE) == 0)
            return (0);                   // I/O still busy

             //-----------------------------------------------------------------
             // caller has requested: I2C_WAIT_FOR_COMPLETE.
             //
             // for Low Power or RTOS versions we would do a wait, until
             // the I/O interrupts completed.
             //-----------------------------------------------------------------
//       IO_SEMAPHORE_SET (ioblock->i2c_semaphore);
         IO_SEMAPHORE_WAIT (&ioblock->i2c_semaphore);  // go into LPM or RTOS proc switch --> See CC3100 LOGIC
       }

    if (ioblock->i2c_state == I2C_STATE_ERROR_COMPLETE)
       return (-1);                      // I/O Completed with error

    return (1);                          // I/O Completed successfully
}


//*****************************************************************************
//  board_i2c_get_handle
//
//             debugging hook used during board bring up to test new MCUs I2C.
//*****************************************************************************

I2C_HandleTypeDef *board_i2c_get_handle (unsigned int i2c_module_id)
{
    I2C_HandleTypeDef  *pI2cHdl;

    if (i2c_module_id < 0 || i2c_module_id > MAX_I2C_ID)
       return (0L);                    // invalid I2C module id

    pI2cHdl = (I2C_HandleTypeDef*) _g_i2c_typedef_handle_addr[i2c_module_id];

    return (pI2cHdl);                  // return HAL related I2C handle
}


/*******************************************************************************
* board_i2c_get_module_ioblk
*
*           Checks if the logical module id is valid, and returns
*           associated I2C io_block.
*
*           Returns:  0   if completed successfully
*                    -2xx if completed with error
*******************************************************************************/
int  board_i2c_get_module_ioblk (unsigned int i2c_module_id, I2C_IO_BUF_BLK **ret_ioblock)
{
    I2C_IO_BUF_BLK  *ioblock;

    if (i2c_module_id > MAX_I2C)
       return (ERR_I2C_MODULE_ID_OUT_OF_RANGE);

    ioblock = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [i2c_module_id];  // get assoc I/O block
    *ret_ioblock = ioblock;           // pass back the associated I2C I/O block
                                      //      for that module
    return (0);                       // denote worked OK
}


//*****************************************************************************
//  board_i2c_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when any I2C interrupt completion (or terminating error) occurs.
//*****************************************************************************
int  board_i2c_set_callback (unsigned int i2c_module_id,
                             I2C_CB_EVENT_HANDLER callback_function,
                             void *callback_parm)
{
    I2C_IO_BUF_BLK  *ioblock;
    int             rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

       //-------------------------------------------------------------------
       // Save any call back information, for interrupts
       //-------------------------------------------------------------------

    ioblock->i2c_callback_handler = callback_function;  // save callback info
    ioblock->i2c_callback_parm    = callback_parm;

    return (0);                        // denote completed OK
}


//*****************************************************************************
//  board_i2c_set_max_timeout
//
//          Set the maximum time that the MCU should wait for an I2C
//          send/receive transaction to complete.
//          If it exceeds this value, signal an error, and cancel the I/O
//          that was in progress.
//
// NOTE THAT WITH ST code, TIMEOUTs are _only_ used with Polled I/O.
//      Are _NOT_ used with Interrupt or DMA driven I/O  !!!
//*****************************************************************************
int  board_i2c_set_max_timeout (unsigned int i2c_module_id, uint32_t max_timeout)
{
    I2C_IO_BUF_BLK  *ioblock;
    int             rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    ioblock->i2c_max_timeout = max_timeout;  // save Max Timeout value

    return (0);                        // denote completed OK
}


//*****************************************************************************
//  board_i2c_stop_io
//
//          An I2C I/O call (read/write/write_read) timed out.
//          Terminate any I/O in progress and reset back to start a new operation.
//
// NOTE THAT WITH ST code, TIMEOUTs are _only_ used with Polled I/O.
//      Are _NOT_ used with Interrupt or DMA driven I/O  !!!
//*****************************************************************************
void  board_i2c_stop_io (I2C_IO_BUF_BLK *ioblock)
{
        // ??? CALL HAL IF WE ARE USING HAL for I/O   -> use pi2c ptr in IOBLK
        // ??? HOW STOP AN I2C OPERATION ?   MUST IT BE RESET BACK LIKE INIT ?

        /* Disable TXE, RXNE and ERR interrupts when using interrupt process */
/// __HAL_I2C_DISABLE_IT (ioblock->i2c_handle, (I2C_IT_TXE | I2C_IT_RXNE | I2C_IT_ERR));

    __HAL_I2C_DISABLE(ioblock->i2c_handle);  // issue I2C disable to stop I/O

    ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;  // set our state


//   ??? !!!   Issue error callback to the user    ??? !!!


    __HAL_I2C_ENABLE(ioblock->i2c_handle);   // then re-anble I2C for new I/O
}



//*****************************************************************************
//  board_i2c_read
//
// @brief  Sets the registers of the EasySpin to their predefined values
//         from easyspin_target_config.h
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************

int  board_i2c_read (unsigned int i2c_module_id,  int slave_addr,
                     uint8_t *receive_buffer, int max_buf_length, int flags)
{
    I2C_HandleTypeDef *pI2cHdl;
    I2C_TypeDef       *spibase;
    I2C_IO_BUF_BLK    *ioblock;
    int               rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

       /* Before starting a new communication transfer, you need to check the current
       *  state of the peripheral; if it’s busy you need to wait for the end of current
       *  transfer before starting a new one.
       *  For simplicity reasons, this example is just waiting till the end of the
       *  transfer, but application may perform other tasks while transfer operation
       *  is ongoing. */
/// while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)

    pI2cHdl   = ioblock->i2c_handle;           // get associated HAL handle
    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);  // still working on a previous I/O

    ioblock->i2c_buffer = receive_buffer;    // save ptr to User I/O buf
    ioblock->i2c_length = max_buf_length;    // save amount of data to rcv
    ioblock->i2c_expiry_time = ioblock->i2c_max_timeout;   // set any max timeout

    ioblock->i2c_state = I2C_STATE_IO_PROCESSING; // denote I/O is now in progress

    if ( ! ioblock->i2c_use_interrupts)
       {        // polled I/O
         if (ioblock->i2c_master_slave == I2C_MASTER)
            rc = HAL_I2C_Master_Receive (ioblock->i2c_handle, slave_addr,
                                         receive_buffer,
                                         max_buf_length, ioblock->i2c_max_timeout);
           else rc = HAL_I2C_Slave_Receive (ioblock->i2c_handle,
                                            receive_buffer,
                                            max_buf_length, ioblock->i2c_max_timeout);
       }
      else {    // INTERRUPT driven I/O
             if (ioblock->i2c_master_slave == I2C_MASTER)
                rc = HAL_I2C_Master_Receive_IT (ioblock->i2c_handle, slave_addr,
                                                receive_buffer,
                                                max_buf_length);
               else rc = HAL_I2C_Slave_Receive_IT (ioblock->i2c_handle,
                                                   receive_buffer,
                                                   max_buf_length);
           }

// ??? WVD ??? !!! need to add check and wait for Blocking I/O
      // BLOCKING I/O - wait until I/O is complete, before return to app

    if (rc != HAL_OK)
       { ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;
            /* Error_Handler() function is called when Timout error occurs.
            ** When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
            ** Master restarts communication */
////     if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)   return (xyz);
         return (-1);
       }
      else ioblock->i2c_state = I2C_STATE_IO_COMPLETED;  // Tag as successful I/O

    return (0);        // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//  board_i2c_write_header_data
//
// @brief  This is used to write a header (like a Read Cmd + EPROM address),
//         and then do an I2C Re-Start to read back the requested data.
//
//         It is analogous to ST's HAL_I2C_Mem_Read().
//         But it is brain damaged, because you can only write a fixed 2 bytes max !
//         DUMB !    See  HAL_I2C_Mem_Read() and  I2C_RequestMemoryRead()
//
//         For now, kow-tow to that goofiness, but later on make it more
//         general purpose, because sometimes Cmd + EPROM address exceeds 2 bytes !!!
//
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************

int  board_i2c_read_header_data (unsigned int i2c_module_id, int slave_addr,
                                 uint16_t header_buffer,    int hbuf_length,
                                 uint8_t  *receive_buffer, int rbuf_length,
                                 int flags)
{
    I2C_HandleTypeDef  *pI2cHdl;
    I2C_IO_BUF_BLK     *ioblock;
    int                rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    pI2cHdl   = ioblock->i2c_handle;            // get associated HAL handle

extern  I2C_HandleTypeDef  I2C_SHIELDS_Handle;  // TESTING HACK
extern  int   use_ST_i2c_read_code;             // TESTING HACK
if (pI2cHdl == 0L && use_ST_i2c_read_code == 1) // TESTING HACK (allow code Mix and Match)
   { pI2cHdl = &I2C_SHIELDS_Handle;   // TESTING HACK
     ioblock->i2c_handle = pI2cHdl;   // TESTING_HACK
     ioblock->i2c_max_timeout = 1000; // TESTING HACK
   }                                  // TESTING HACK

writ_ioblock = ioblock;             // DEBUG ONLY
writ_pI2cHdl = pI2cHdl;             // DEBUG ONLY
    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

    if ( ! ioblock->i2c_use_interrupts)
       {        // Blocking and POLLING Mode
         rc = HAL_I2C_Mem_Read (ioblock->i2c_handle, slave_addr,
                                 header_buffer,  hbuf_length,
                                 receive_buffer, rbuf_length, ioblock->i2c_max_timeout);
       }
      else {    // Non-Blocking and INTERRUPT mode
             rc = HAL_I2C_Mem_Read_IT (ioblock->i2c_handle, slave_addr,
                                       header_buffer,  hbuf_length,
                                       receive_buffer, rbuf_length);
           }

// ??? WVD - add check and wait for BLOCKING I/O

    if (rc != HAL_OK)
       { ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;
         return (-1);
       }
      else ioblock->i2c_state = I2C_STATE_IO_COMPLETED; // Tag as successful I/O

    return (0);          // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//*****************************************************************************

int  board_i2c_write (unsigned int i2c_module_id,  int slave_addr,
                      uint8_t *transmit_buffer, int buf_length, int flags)
{
    I2C_HandleTypeDef  *pI2cHdl;
    I2C_TypeDef        *spibase;
    uint32_t           tickstart,  tickend;
    uint8_t            dummy;
    I2C_IO_BUF_BLK     *ioblock;
    int                rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    pI2cHdl   = ioblock->i2c_handle;            // get associated HAL handle
    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

    ioblock->i2c_buffer = transmit_buffer;    // save ptr to User I/O buf
    ioblock->i2c_length = buf_length;         // save amount of data to send
    ioblock->i2c_expiry_time = ioblock->i2c_max_timeout;   // set any max timeout

    ioblock->i2c_state = I2C_STATE_IO_PROCESSING; // denote I/O is now in progress

    spibase = (I2C_TypeDef*) pI2cHdl->Instance;     // Point to I2C peripheral HW
    if ( ! ioblock->i2c_use_interrupts)
       {        // POLLED I/O - wait until operation is complete, before return to app
//       if (ioblock->i2c_mode == I2C_SLAVE)
//          { spibase->DR = (*transmit_buffer);     // load a byte of data
//            buf_length--;
//          }

         if (ioblock->i2c_master_slave == I2C_MASTER)
            rc = HAL_I2C_Master_Transmit (ioblock->i2c_handle, slave_addr,
                                              transmit_buffer,
                                              buf_length, ioblock->i2c_max_timeout);
           else rc = HAL_I2C_Slave_Transmit (ioblock->i2c_handle,
                                                 transmit_buffer,
                                                 buf_length, ioblock->i2c_max_timeout);
         if (rc == HAL_OK)
            {
//            rc = OS_WAIT (ioblock->i2c_expiry_time); // Wait for the I/O to complete
//            if (rc != 0)
//               board_i2c_stop_io (ioblock); // I/O timed out. Stop any I/O call in progress
              ioblock->i2c_state = I2C_STATE_IO_COMPLETED;  // Tag as successful I/O
            }
       }

      else {    // INTERRUPT driven I/O
             if (ioblock->i2c_master_slave == I2C_MASTER)
                rc = HAL_I2C_Master_Transmit_IT (ioblock->i2c_handle, slave_addr,
                                                 transmit_buffer,
                                                 buf_length);
               else rc = HAL_I2C_Slave_Transmit_IT (ioblock->i2c_handle,
                                                    transmit_buffer,
                                                    buf_length);
           }

// ??? WVD !!! Add check and wait for Blocking I/O

    if (rc != HAL_OK)
       { ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;
         return (-1);
       }
      else ioblock->i2c_state = I2C_STATE_IO_COMPLETED;  // Tag as successful I/O

    return (0);        // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//  board_i2c_write_read
//
// @brief  This is used to write a header (like a Read Cmd + EPROM address),
//         and then do an I2C Re-Start to read in the reply.
//
//         It is analogous to ST's HAL_I2C_Mem_Read().
//         But it is brain damaged, because you can only write a fixed 2 bytes max !
//         DUMB !    See  HAL_I2C_Mem_Read() and  I2C_RequestMemoryRead()
//
//         For now, kow-tow to that goofiness, but later on make it more
//         general purpose, because sometimes Cmd + EPROM address exceeds 2 bytes !!!
//
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************
int  board_i2c_write_read (unsigned int i2c_module_id,  int slave_addr,
                           uint16_t header_buffer,   int hbuf_length,
// future                  uint8_t *transmit_buffer, int xbuf_length,
                           uint8_t *receive_buffer,  int rbuf_length,
                           int flags)
{
    I2C_HandleTypeDef  *pI2cHdl;
    I2C_IO_BUF_BLK     *ioblock;
    int                rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    pI2cHdl   = ioblock->i2c_handle;            // get associated HAL handle
    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

// ??? HUGH ==> UNFINISHED ??? !!!   WVD

    if ( ! ioblock->i2c_use_interrupts)
       {        // Blocking and POLLING Mode
         rc = HAL_I2C_Mem_Read (ioblock->i2c_handle, slave_addr,
                                header_buffer,  hbuf_length,
                                receive_buffer, rbuf_length, ioblock->i2c_max_timeout);
       }
      else {    // Non-Blocking and INTERRUPT mode
             rc = HAL_I2C_Mem_Read_IT (ioblock->i2c_handle, slave_addr,
                                       header_buffer,  hbuf_length,
                                       receive_buffer, rbuf_length);
           }

    if (rc != HAL_OK)
       { ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;
         return (-1);
       }
      else I2C_STATE_IO_COMPLETED;  // Tag as successful I/O

    return (0);        // tell user it was started (Non-Block) or completed OK
}


//*****************************************************************************
//  board_i2c_write_header_data
//
// @brief  This is used to write a header (like a Write Cmd + EPROM address),
//         and then do an I2C Re-Start to write out the rest of the data.
//
//         It is analogous to ST's HAL_I2C_Mem_Write().
//         But it is brain damaged, because you can only write a fixed 2 bytes max !
//         DUMB !    See  HAL_I2C_Mem_Write() and  I2C_RequestMemoryWrite()
//
//         For now, kow-tow to that goofiness, but later on make it more
//         general purpose, because sometimes Cmd + EPROM address exceeds 2 bytes !!!
//
// @param[in] pByteToTransmit pointer to the byte to transmit
// @param[in] pReceivedByte pointer to the received byte
// @retval    None
//*****************************************************************************

int  board_i2c_write_header_data (unsigned int i2c_module_id,  int slave_addr,
                                  uint16_t header_buffer,    int hbuf_length,
///   future                      uint8_t  *header_buffer,   int hbuf_length,
                                  uint8_t  *transmit_buffer, int xbuf_length,
                                  int flags)
{
    I2C_HandleTypeDef  *pI2cHdl;
    I2C_IO_BUF_BLK     *ioblock;
    int                rc;

    rc = board_i2c_get_module_ioblk (i2c_module_id, &ioblock); // get I2C IOBLK
    if (rc != 0)
       return (rc);        // module not valid. return error code to caller.

    pI2cHdl   = ioblock->i2c_handle;            // get associated HAL handle
writ_ioblock = ioblock;         // DEBUG ONLY
writ_pI2cHdl   = pI2cHdl;           // DEBUG ONLY
    if (ioblock->i2c_state == I2C_STATE_IO_PROCESSING)
       return (ERR_IO_ALREADY_IN_PROGRESS);   // still working on a previous I/O

    if ( ! ioblock->i2c_use_interrupts)
       {        // Blocking and POLLING Mode
         rc = HAL_I2C_Mem_Write (ioblock->i2c_handle, slave_addr,
                                 header_buffer,   hbuf_length,
                                 transmit_buffer, xbuf_length, ioblock->i2c_max_timeout);
       }
      else {    // Non-Blocking and INTERRUPT mode
             rc = HAL_I2C_Mem_Write_IT (ioblock->i2c_handle, slave_addr,
                                        header_buffer,   hbuf_length,
                                        transmit_buffer, xbuf_length);
           }

// ??? WVD - add check and wait for BLOCKING I/O

    if (rc != HAL_OK)
       { ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;
         return (-1);
       }
      else ioblock->i2c_state = I2C_STATE_IO_COMPLETED; // Tag as successful I/O

    return (0);          // tell user it was started (Non-Block) or completed OK
}


   extern    int   i2c_rupt_module_id;            // TEMP_HACK


/*************************************************************************
*                                 TEMP  ISR   HACK
* @brief  I2C error callbacks.
* @param  hspi: I2C handle
*
* @note   This example shows a simple way to report transfer error,
*         and you canadd your own implementation.
* @retval None
*/
void  HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hspi)
{
    I2C_IO_BUF_BLK     *ioblock;

    ioblock = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [i2c_rupt_module_id];    // get assoc I/O block
    ioblock->i2c_state = I2C_STATE_ERROR_COMPLETE;   // set ending status

    if (ioblock->i2c_callback_handler != 0L)
       {             // Invoke user callback for the interrupt
         (ioblock->i2c_callback_handler) (ioblock->i2c_callback_parm,
                                          i2c_rupt_module_id, -1);  // signal error
       }
}


/************************************************************************
*                                 TEMP  ISR   HACK
*
* @brief  TxRx Transfer completed callback.
* @param  hspi: I2C handle.
*
* @note   This example shows a simple way to report end of
*         Interrupt TxRx transfer, and
*         you can add your own implementation.
* @retval None
*/
void  Internal_HAL_I2C_TxRxCpltCallback (I2C_HandleTypeDef *hi2c, int rupt_num)
{
    I2C_IO_BUF_BLK     *ioblock;

    ioblock = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [rupt_num];  // get assoc I/O block
    ioblock->i2c_state = I2C_STATE_IO_COMPLETED;        // Tag as successful I/O

    if (ioblock->i2c_callback_handler != 0L)
       {             // Invoke user callback for the interrupt
         (ioblock->i2c_callback_handler) (ioblock->i2c_callback_parm,
                                          rupt_num, 0);  // signal good completion
       }
}


//void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
//void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
//void HAL_I2C_SlaveTxCpltCallback (I2C_HandleTypeDef *hi2c)
//void HAL_I2C_SlaveRxCpltCallback (I2C_HandleTypeDef *hi2c)
//void HAL_I2C_MemTxCpltCallback (I2C_HandleTypeDef *hi2c)
//void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef *hi2c)
//{
//    Internal_HAL_I2C_TxRxCpltCallback (hi2c);
//}


//******************************************************************************
//                     COMMON    I2C     ISR / IRQ     Handler
//
// The i2c_interrupt_number ranges from 1 to n, and it indicates which
// I2Cx_IRQHandler invoked us: e.g. 1 = I2C1_IRQHandler, 2 = I2C2_IRQHandler.
// The handler is directly tied to the I2C modules I2C1, I2C2, ...
//******************************************************************************
    int   i2c_rupt_module_id = 0;            // TEMP_HACK

void  board_i2c_IRQ_Handler (int i2c_interrupt_number)
{
    I2C_IO_BUF_BLK  *ioblk;

    i2c_rupt_module_id = i2c_interrupt_number;

    ioblk = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [i2c_interrupt_number];

    IO_SEMAPHORE_RELEASE (&ioblk->i2c_semaphore);  // clear any semaphore

// ??? !!! should also update ioblk's io_status to COMPLETED, to eliminate above i2c_rupt_module_id  ??? !!! WVD

    Internal_HAL_I2C_TxRxCpltCallback (ioblk->i2c_handle, i2c_interrupt_number);

    HAL_I2C_EV_IRQHandler (ioblk->i2c_handle);    // initial pass
    HAL_I2C_ER_IRQHandler (ioblk->i2c_handle);
}



//******************************************************************************
//                     COMMON    I2C     ERROR   IRQ     Handler
//
// The i2c_interrupt_number ranges from 1 to n, and it indicates which
// I2Cx_IRQHandler invoked us: e.g. 1 = I2C1_IRQHandler, 2 = I2C2_IRQHandler.
// The handler is directly tied to the I2C modules I2C1, I2C2, ...
//******************************************************************************
void  board_i2c_ERRIRQ_Handler (int i2c_interrupt_number)
{
    I2C_IO_BUF_BLK  *ioblk;

    i2c_rupt_module_id = i2c_interrupt_number;

    ioblk = (I2C_IO_BUF_BLK*) _g_i2c_io_blk_address [i2c_interrupt_number];

    IO_SEMAPHORE_RELEASE (&ioblk->i2c_semaphore);  // clear any semaphore

    HAL_I2C_ER_IRQHandler (ioblk->i2c_handle);    // initial pass
}

/******************************************************************************/
