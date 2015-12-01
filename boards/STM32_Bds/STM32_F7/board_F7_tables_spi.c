
//2******1*********2*********3*********4*********5*********6*********7**********
//                                                                      STM32 F7
//                            board_F7_tables_spi.c
//
//
//                            STM32  F7  46  NG        Discovery     STM32F746xx / USE_STM32746G_DISCOVERY
//
//
//               MCU SPECIFIC   SPI   GPIO  PIN   DEFINITIONS
//
//                                   for
//
//                               SPI Support
//
//
//  History:
//    10/16/15 - Verified tables. Duq
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
//
//   These GPIO definitions for SPI Module GPIO mappings are used in common by
//   the SPI logic when setting up GPIO pin connections/combinations, that
//   will be used by the requested SPI module.
//
//------------------------------------------------------------------------------
//                    SPI Module  Summary     available on STM32 F0
//                                                                  Supports
//          SCLK     MISO     MOSI
//          ----     ----     ----
//   SPI1 = PA5   /  PA6   /  PA7
//     "    PB3   /  PB4   /  PB5    (alternate pin mode)
//   SPI2 = PB13  /  PC2   /  PC3
//     "    PB13  /  PB14  /  PB15   (alternate pin mode)
//   SPI3 = PC10  /  PC11  /  PC12
//     "    PB3   /  PB4   /  PB5    (alternate pin mode)
//------------------------------------------------------------------------------

/*******************************************************************************
//------------------------------------------------------------------------------
//                           SPI Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//   D15    PB_8   - none -

//   D14    PB_9   SPI2_NSS

//   D13    PA_5   SPI1_SCK

//   D12    PA_6   SPI1_MISO

//   D11    PA_7   SPI1_MOSI

//   D10    PB_6   - none -

//   D9     PC_7   - none -

//   D8     PA_9   - none -

//   D7     PA_8   - none -

//   D6     PB_10  SPI2_SCK / I2S2_CK

//   D5     PB_4   SPI1_MISO           SPI3_MISO

//   D4     PB_5   SPI1_MOSI           SPI3_MOSI / I2S3_SD

//   D3     PB_3   SPI1_SCK            SPI3_SCK / I2S3_CK

//   D2     PA_10  - none -

//   D1     PA_2   - none -

//   D0     PA_3   - none -

//   A0     PA_0   - none -

//   A1     PA_1   - none -

//   A2     PA_4   SPI1_NSS            SPI3_NSS / I2S3_WS

//   A3     PB_0   - none -

//   A4     PC_1   - none -

//   A5     PC_0   - none -

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//   CNx    PA_11  - none -
//   CNx    PA_12  - none -
//   CNx  PA_13-14 - none -
//   CNx    PA_15  SPI1_NSS              SPI3_NSS / I2S3_WS

//   CNx    PB_1   - none -
//   CNx    PB_2   -                     SPI3_MOSI
//   CNx    PB_7   - none -
//   CNx    PB_11  - none -
//   CNx    PB_12  SPI2_NSS  / I2S2_WS
//   CNx    PB_13  SPI2_SCK  / I2S2_C
//   CNx    PB_14  SPI2_MISO
//   CNx    PB_15  SPI2_MOSI / I2S2_SD

//   CNx    PC_2   SPI2_MISO             I2S2ext_SD
//   CNx    PC_3   SPI2_MOSI / I2S2_SD
//   PC_4 - PC_5   - none -
//   CNx    PC_6   - none -
//   CNx    PC_7   - none -
//   CNx    PC_8   - none -
//   CNx    PC_9   - none -
//   CNx    PC_10  SPI3_SCK  / I2S3_CK
//   CNx    PC_11  SPI3_MISO             I2S3ext_SD
//   CNx    PC_12  SPI3_MOSI / I2S3_SD
//  PC_13 - PC_15  - none -

//   PD_0 - PD_2   - none -
//   CNx    PD_3   SPI2_SCK  / I2S2_CK
//   PD_4 - PD_5   - none -
//   CNx    PD_6   SPI3_MOSI / I2S3_SD
//   PD_7 - PD_15  - none -

//   PE_0 - PE_1   - none -
//   CNx    PE_2   SPI4_SCK
//   CNx    PE_3   - none -
//   CNx    PE_4   SPI4_NSS
//   CNx    PE_5   SPI4_MISO
//   CNx    PE_6   SPI4_MOSI
//   PE_7 - PE_10  - none -
//   CNx    PE_11  SPI4_NSS
//   CNx    PE_12  SPI4_SCK
//   CNx    PE_13  SPI4_MISO
//   CNx    PE_14  SPI4_MOSI
//   CNx    PE_15  - none -

//   PH_0 - PF_1   - none -

//------------------------------------------------------------------------------

int  board_spi_enable_clock (int module_id);         // internal routines


const SPI_TypeDef  *_g_spi_module_base [] =
          {   0L,                   // [0] = no default SPI module
             SPI1,
             SPI2,
             SPI3,
             SPI4,
             SPI5,
             SPI6,
              0L                    // Max allowed SPI modules is 8 (F7)
          };

                            //----------------------------------------
                            //   SPIn handles needed, one per module
                            //----------------------------------------
    SPI_HandleTypeDef  _g_hspi1_handle;           // SPI 1 support
    SPI_HandleTypeDef  _g_hspi2_handle;           // SPI 2 support
    SPI_HandleTypeDef  _g_hspi3_handle;           // SPI 3 support
    SPI_HandleTypeDef  _g_hspi4_handle;           // SPI 4 support
    SPI_HandleTypeDef  _g_hspi5_handle;           // SPI 5 support
    SPI_HandleTypeDef  _g_hspi6_handle;           // SPI 6 support

    DMA_HandleTypeDef  _g_hdma_spi2_rx;
    DMA_HandleTypeDef  _g_hdma_spi2_tx;


                            //----------------------------------------------------------
                            // List of HAL Handle Addresses, one for each separate SPI Handle
                            //----------------------------------------------------------
const SPI_HandleTypeDef *_g_spi_typedef_handle []     // HAL API Handles
                     = {    0L,                       // no SPI0  on F4
                         (SPI_HandleTypeDef*) &_g_hspi1_handle,  // SPI1
                         (SPI_HandleTypeDef*) &_g_hspi2_handle,  // SPI2
                         (SPI_HandleTypeDef*) &_g_hspi3_handle,  // SPI3
                         (SPI_HandleTypeDef*) &_g_hspi4_handle,  // SPI4
                         (SPI_HandleTypeDef*) &_g_hspi5_handle,  // SPI5
                         (SPI_HandleTypeDef*) &_g_hspi6_handle,  // SPI6
                       };


                            //-------------------------------------------------------
                            // SPIn I/O Buffer Control blocks needed, one per module
                            //-------------------------------------------------------
    SPI_IO_BUF_BLK  _g_spi_1_io_blk = { 0 };  // ensure SPI I/O blocks cleared at startup
    SPI_IO_BUF_BLK  _g_spi_2_io_blk = { 0 };
    SPI_IO_BUF_BLK  _g_spi_3_io_blk = { 0 };
    SPI_IO_BUF_BLK  _g_spi_4_io_blk = { 0 };
    SPI_IO_BUF_BLK  _g_spi_5_io_blk = { 0 };
    SPI_IO_BUF_BLK  _g_spi_6_io_blk = { 0 };

                            //----------------------------------------------------------
                            // List of Addresses, one for each separate SPI_IO_BLK
                            //----------------------------------------------------------
const SPI_IO_BUF_BLK *_g_spi_io_blk_address []   // SPI I/O Block addresses
                       = {       0L,             // no SPI0  on STM32
                           (SPI_IO_BUF_BLK*) &_g_spi_1_io_blk,  // SPI1
                           (SPI_IO_BUF_BLK*) &_g_spi_2_io_blk,  // SPI2
                           (SPI_IO_BUF_BLK*) &_g_spi_3_io_blk,  // SPI3
                           (SPI_IO_BUF_BLK*) &_g_spi_4_io_blk,  // SPI4
                           (SPI_IO_BUF_BLK*) &_g_spi_5_io_blk,  // SPI5
                           (SPI_IO_BUF_BLK*) &_g_spi_6_io_blk,  // SPI6
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
const SPI_GPIO_BLK  _g_spi_1_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF5_SPI1, SCLK_PIN_ROLE, PA5  },    // PA5   SCLK
        { GPIO_AF5_SPI1, SCLK_PIN_ROLE, PB3  },    // PB3   SCLK
        { GPIO_AF5_SPI1, MISO_PIN_ROLE, PA6  },    // PA6   MISO
        { GPIO_AF5_SPI1, MOSI_PIN_ROLE, PA7  },    // PA7   MOSI    PA5/PA6/PA7   D13/D12/D11
                                                   //               PB3/PA6/PA7
        { GPIO_AF5_SPI1, MISO_PIN_ROLE, PB4  },    // PB4   MISO
        { GPIO_AF5_SPI1, MOSI_PIN_ROLE, PB5  },    // PB5   MOSI    PB3/PB4/PB5   D3/D5/D4                                                   //               PB3/PA6/PA1
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_2_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PA9  },    // PA9   SCLK
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PB10 },    // PB10  SCLK
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PB13 },    // PB13  SCLK
        { GPIO_AF5_SPI2, MISO_PIN_ROLE, PB14 },    // PB14  MISO    PA9/PB14/PB15
        { GPIO_AF5_SPI2, MOSI_PIN_ROLE, PB15 },    // PB15  MOSI    PB10/PB14/PB15
                                                   //               PB13/PB14/PB15
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PD3  },    // PD3   SCLK
        { GPIO_AF5_SPI2, MISO_PIN_ROLE, PC2  },    // PC2   MISO
        { GPIO_AF5_SPI2, MOSI_PIN_ROLE, PC1  },    // PC1   MOSI    PD3/PC2/PC1
        { GPIO_AF5_SPI2, MOSI_PIN_ROLE, PC3  },    // PC3   MOSI    PD3/PC2/PC3
                                                   //               PD3/PB14/PB15
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PI1  },    // PI1   SCLK
        { GPIO_AF5_SPI2, MISO_PIN_ROLE, PI2  },    // PI2   MISO
        { GPIO_AF5_SPI2, MOSI_PIN_ROLE, PI3  },    // PI3   MOSI    PI1/PI2/PI3
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_3_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF6_SPI3, SCLK_PIN_ROLE, PB3  },    // PB3   SCLK
        { GPIO_AF6_SPI3, MISO_PIN_ROLE, PB4  },    // PB4   MISO
        { GPIO_AF6_SPI3, MOSI_PIN_ROLE, PB5  },    // PB5   MOSI    PB3/PB4/PB5
        { GPIO_AF7_SPI3, MOSI_PIN_ROLE, PB2  },    // PB2   MOSI    PB3/PB4/PB2

        { GPIO_AF6_SPI3, SCLK_PIN_ROLE, PC10 },    // PC10  SCLK
        { GPIO_AF6_SPI3, MISO_PIN_ROLE, PC11 },    // PC11  MISO
        { GPIO_AF6_SPI3, MOSI_PIN_ROLE, PC12 },    // PC12  MOSI    PC10/PC11/PC12
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_4_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF5_SPI4, SCLK_PIN_ROLE, PE2  },    // PE2   SCLK
        { GPIO_AF5_SPI4, MISO_PIN_ROLE, PE5  },    // PE5   MISO
        { GPIO_AF5_SPI4, MOSI_PIN_ROLE, PE6  },    // PE6   MOSI    PE2/PE5/PE6
                                                   //               PE2/PE13/PE14
        { GPIO_AF5_SPI4, SCLK_PIN_ROLE, PE12 },    // PE12  SCLK
        { GPIO_AF5_SPI4, MISO_PIN_ROLE, PE13 },    // PE13  MISO
        { GPIO_AF5_SPI4, MOSI_PIN_ROLE, PE14 },    // PE14  MOSI    PE12/PE13/PE14
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_5_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF5_SPI5, SCLK_PIN_ROLE, PF7  },    // PF7   SCLK
        { GPIO_AF5_SPI5, MISO_PIN_ROLE, PF8  },    // PF8   MISO
        { GPIO_AF5_SPI5, MOSI_PIN_ROLE, PF9  },    // PF9   MOSI    PF7/PF8/PF9
        { GPIO_AF5_SPI5, MOSI_PIN_ROLE, PF11 },    // PF11  MOSI    PF7/PF8/PF11

        { GPIO_AF5_SPI5, SCLK_PIN_ROLE, PH6  },    // PH6   SCLK
        { GPIO_AF5_SPI5, MISO_PIN_ROLE, PH7  },    // PH7   MISO    PH6/PH7/PF9
                                                   //               PH6/PH7/PF11
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_6_gpio_table [] =
      {                                            //    SCLK / MISO / MOSI
        { GPIO_AF5_SPI6, SCLK_PIN_ROLE, PG13 },    // PG13  SCLK
        { GPIO_AF5_SPI6, MISO_PIN_ROLE, PG12 },    // PG12  MISO
        { GPIO_AF5_SPI6, MOSI_PIN_ROLE, PG14 },    // PG14  MOSI    PG13/PG12/PG14
        {      0,              -1,      0,  }      // End of Table
      };


              //---------------------------------------------------------------
              // Lookup table for valid SCLK/MISO/MOSI combos for each diff SPI
              //---------------------------------------------------------------
const SPI_GPIO_BLK *  _g_spi_gpio_table_addr[] =
      {                        0L,            // no SPI0 nor SPI default module
        (SPI_GPIO_BLK*) &_g_spi_1_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_2_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_3_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_4_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_5_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_6_gpio_table,
      };



//   A2     PA_4   SPI1_NSS             SPI3_NSS
//   D13    PA_5   SPI1_SCK
//   D12    PA_6   SPI1_MISO
//   D11    PA_7   SPI1_MOSI
//   D2     PA_9   SPI2_SCK  -- new --
//   CNx    PA_15  SPI1_NSS             SPI3_NSS

//   CNx    PB_2   -                    SPI3_MOSI -- new --
//   D3     PB_3   SPI1_SCK             SPI3_SCK
//   D5     PB_4   SPI1_MISO            SPI3_MISO              SPI2_NSS
//   D4     PB_5   SPI1_MOSI            SPI3_MOSI
//   D14    PB_9   SPI2_NSS
//   D6     PB_10  SPI2_SCK
//   CNx    PB_12  SPI2_NSS             SPI4_NSS - GONE -    SPI3_SCK - GONE -
//   CNx    PB_13  SPI2_SCK             SPI4_SCK - GONE -
//   CNx    PB_14  SPI2_MISO
//   CNx    PB_15  SPI2_MOSI


//   CNx    PC_1   SPI2_MOSI -- new --
//   CNx    PC_2   SPI2_MISO
//   CNx    PC_3   SPI2_MOSI
//   CNx    PC_10  SPI3_SCK
//   CNx    PC_11  SPI3_MISO
//   CNx    PC_12  SPI3_MOSI


//   CNx    PD_3   SPI2_SCK
//   CNx    PD_6   SPI3_MOSI


//   CNx    PE_2   SPI4_SCK             SPI5_SCK  - GONE - All SPI5 stuff is gone from Port E
//   CNx    PE_4   SPI4_NSS             SPI5_NSS  - GONE -
//   CNx    PE_5   SPI4_MISO            SPI5_MISO - GONE -
//   CNx    PE_6   SPI4_MOSI            SPI5_MOSI - GONE -
//   CNx    PE_11  SPI4_NSS             SPI5_NSS  - GONE -
//   CNx    PE_12  SPI4_SCK             SPI5_SCK  - GONE -
//   CNx    PE_13  SPI4_MISO            SPI5_MISO - GONE -
//   CNx    PE_14  SPI4_MOSI            SPI5_MOSI - GONE -

//   CNx    PF_6   SPI5_NSS -- new --
//   CNx    PF_7   SPI5_SCK
//   CNx    PF_8   SPI5_MISO
//   CNx    PF_9   SPI5_MOSI
//   CNx    PF_11  SPI5_MOSI

//   CNx    PG_8   SPI6_NSS  -- new --
//   CNx    PG_12  SPI6_MISO
//   CNx    PG_13  SPI6_SCK
//   CNx    PG_14  SPI6_MOSI

//   CNx    PH_5   SPI5_NSS  -- new --
//   CNx    PH_6   SPI5_SCK
//   CNx    PH_7   SPI5_MISO

//   CNx    PI_0   SPI2_NSS  -- new --
//   CNx    PI_1   SPI2_SCK
//   CNx    PI_2   SPI2_MISO
//   CNx    PI_3   SPI2_MOSI  - Last SPI pin on F7


//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************


//******************************************************************************
//  board_spi_enable_clock
//
//           Ensures that the specified Timer's clock is turned on.
//******************************************************************************
int  board_spi_enable_clock (int module_id)
{
      //--------------------------------------------------------
      // Turn on clock for the associated SPIx module.
      //--------------------------------------------------------
    switch (module_id)
     { case 1:                           // SPI1
             __SPI1_CLK_ENABLE();
             break;
        case 2:                          // SPI2
             __SPI2_CLK_ENABLE();
             break;
        case 3:                          // SPI3
             __SPI3_CLK_ENABLE();
             break;
        case 4:                          // SPI4
             __SPI4_CLK_ENABLE();
             break;
     }
    return (0);          // denote everything worked OK
}



//******************************************************************************
//  board_spi_enable_nvic_irq
//
//           Ensures that the specified IRQ in NVIC turned on.
//******************************************************************************
void  board_spi_enable_nvic_irq (int module_id)
{
    uint32_t   nvic_irq;

      //--------------------------------------------------------
      // Enable the NVIC for the associated SPI module
      //--------------------------------------------------------
    if (module_id == 1)
       nvic_irq = SPI1_IRQn;
       else if (module_id == 2)
               nvic_irq = SPI2_IRQn;
       else if (module_id == 3)
               nvic_irq = SPI3_IRQn;
       else nvic_irq = SPI4_IRQn;

    HAL_NVIC_SetPriority (nvic_irq, 1, 1);
    HAL_NVIC_EnableIRQ (nvic_irq);
}

/******************************************************************************/
