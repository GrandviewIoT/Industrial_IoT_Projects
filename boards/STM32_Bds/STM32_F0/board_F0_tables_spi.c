
//2******1*********2*********3*********4*********5*********6*********7**********
//                                                                      STM32 F0
//                             board_F0_tables_spi.c
//
//
//                                STM32  F0  30                      STM32F030x8
//                                STM32  F0  70                      STM32F070xB
//                                STM32  F0  72                      STM32F072xB
//                                STM32  F0  91                      STM32F091xC
//
//
//               MCU SPECIFIC   SPI   GPIO  PIN   DEFINITIONS
//
//                                   for
//
//                               SPI Support
//
//  History:
//    08/16/15 - Verified tables. Duq
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
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                           SPI Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//   D15    PB_8   - none -

//   D14    PB_9   SPI2_NSS  / I2S2_SD

//   D13    PA_5   SPI1_SCK  / I2S1_CK

//   D12    PA_6   SPI1_MISO / I2S1_MCK

//   D11    PA_7   SPI1_MOSI / I2S1_SD

//   D10    PB_6   - none -

//   D9     PC_7   - none -

//   D8     PA_9   - none -

//   D7     PA_8   - none -

//   D6     PB_10  SPI2_SCK  / I2S2_CK

//   D5     PB_4   SPI1_MISO / I2S1_MCK

//   D4     PB_5   SPI1_MOSI / I2S1_SD

//   D3     PB_3   SPI1_SCK  / I2S1_CK

//   D2     PA_10  - none -

//   D1     PA_2   - none -

//   D0     PA_3   - none -

//   A0     PA_0   - none -

//   A1     PA_1   - none -

//   A2     PA_4   SPI1_NSS / I2S1_WS

//   A3     PB_0   - none -

//   A4     PC_1   - none -

//   A5     PC_0   - none -

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//   CNx  PA_11-14 - none -
//   CNx    PA_15  SPI1_NSS / I2S1_WS

//   CNx    PB_1   - none -
//   CNx    PB_7   - none -
//   CNx    PB_11  - none -
//   CNx    PB_12  SPI2_NSS  / I2S2_WS
//   CNx    PB_13  SPI2_SCK  / I2S2_CK
//   CNx    PB_14  SPI2_MISO / I2S2_MCK
//   CNx    PB_15  SPI2_MOSI / I2S2_SD

//   CNx    PC_2   SPI2_MISO / I2S2_MCK
//   CNx    PC_3   SPI2_MOSI / I2S2_SD
//   PC_4 - PC_15  - none -

//   CNx    PD_0   SPI2_NSS  / I2S2_WS
//   CNx    PD_1   SPI2_SCK  / I2S2_CK
//   CNx    PD_2   - none -
//   CNx    PD_3   SPI2_MISO / I2S2_MCK
//   CNx    PD_4   SPI2_MOSI / I2S2_SD
//   PD_5 - PD_15  - none -

//   PE_0 - PE_11  - none -
//   CNx    PE_12  SPI1_NSS  / I2S1_WS  All AF1
//   CNx    PE_13  SPI1_SCK  / I2S1_CK
//   CNx    PE_14  SPI1_MISO / I2S1_MCK
//   CNx    PE_15  SPI1_MOSI / I2S1_SD

//   PH_0 - PF_1   - none -

//------------------------------------------------------------------------------

int  board_spi_enable_clock (int module_id);         // internal routines


const SPI_TypeDef  *_g_spi_module_base [] =
          {   0L,                   // [0] = no default SPI module
             SPI1,
             SPI2,
              0L,
              0L,
              0L,
              0L,
              0L,
              0L                    // Max allowed SPI modules is 8 (F7)
          };

                           //----------------------------------------
                           //   SPIx handles needed, one per module
                           //----------------------------------------
    SPI_HandleTypeDef  _g_hspi1_handle;           // SPI 1 support
    SPI_HandleTypeDef  _g_hspi2_handle;           // SPI 2 support

    DMA_HandleTypeDef  _g_hdma_spi2_rx;
    DMA_HandleTypeDef  _g_hdma_spi2_tx;


                             //----------------------------------------------------------
                             // List of HAL Handle Addresses for each separate SPI Handle
                             //----------------------------------------------------------
const SPI_HandleTypeDef *_g_spi_typedef_handle []   /* HAL API Handles */
                     = {    0L,                     // no SPI0  on F0
                         (SPI_HandleTypeDef*) &_g_hspi1_handle,  // SPI1
                         (SPI_HandleTypeDef*) &_g_hspi2_handle,  // SPI2
                       };

                            //-------------------------------------------------------
                            // SPIn I/O Buffer Control blocks needed, one per module
                            //-------------------------------------------------------
    SPI_IO_BUF_BLK  _g_spi_1_io_blk = { 0 };    // clear out to zeros at startup
    SPI_IO_BUF_BLK  _g_spi_2_io_blk = { 0 };

                            //----------------------------------------------------------
                            // List of Addresses, one for each separate SPI_IO_BLK
                            //----------------------------------------------------------
const SPI_IO_BUF_BLK *_g_spi_io_blk_address []   // SPI I/O Block addresses
                       = {       0L,             // no SPI0  on STM32
                           (SPI_IO_BUF_BLK*) &_g_spi_1_io_blk,  // SPI1
                           (SPI_IO_BUF_BLK*) &_g_spi_2_io_blk,  // SPI2
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
        { GPIO_AF0_SPI1, SCLK_PIN_ROLE, PA5  },    // PA5   SCLK
        { GPIO_AF0_SPI1, SCLK_PIN_ROLE, PB3  },    // PB3   SCLK
        { GPIO_AF0_SPI1, MISO_PIN_ROLE, PA6  },    // PA6   MISO
        { GPIO_AF0_SPI1, MOSI_PIN_ROLE, PA7  },    // PA7   MOSI    PA5/PA6/PA7   D13/D12/D11
                                                                 // PB3/PA6/PA7
        { GPIO_AF0_SPI1, MISO_PIN_ROLE, PB4  },    // PB4   MISO
        { GPIO_AF0_SPI1, MOSI_PIN_ROLE, PB5  },    // PB5   MOSI    PB3/PB4/PB5   D3/D5/D4

#if defined(STM32F072xB) || defined(STM32F091xC)
        { GPIO_AF0_SPI1, SCLK_PIN_ROLE, PE13 },    // PE13  SCLK
        { GPIO_AF0_SPI1, MISO_PIN_ROLE, PE14 },    // PE14  MISO
        { GPIO_AF0_SPI1, MOSI_PIN_ROLE, PE15 },    // PE15  MOSI    PE13/PE14/PE15
#endif
        {      0,              -1,      0,  }      // End of Table
      };


const SPI_GPIO_BLK  _g_spi_2_gpio_table [] =
      {                                                    // SCLK / MISO / MOSI
        { GPIO_AF0_SPI2, SCLK_PIN_ROLE, PB13 },    // PB13  SCLK
        { GPIO_AF0_SPI2, MISO_PIN_ROLE, PB14 },    // PB14  MISO
        { GPIO_AF0_SPI2, MOSI_PIN_ROLE, PB15 },    // PB15  MOSI    PB13/PB14/PB15

#if defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F070xB)
        { GPIO_AF5_SPI2, SCLK_PIN_ROLE, PB10 },    // PB10  SCLK
        { GPIO_AF1_SPI2, MISO_PIN_ROLE, PC2  },    // PC2   MISO
        { GPIO_AF1_SPI2, MOSI_PIN_ROLE, PC3  },    // PC3   MOSI    PB10/PC2/PC3   D6
                                                   //               PB13/PC2/PC3
#endif


#if defined(STM32F072xB) || defined(STM32F091xC)
        { GPIO_AF1_SPI2, SCLK_PIN_ROLE, PD1  },    // PD1   SCLK    PD1/PB14/PB15
                                                                 // PD1/PC2/PC3
        { GPIO_AF1_SPI2, MISO_PIN_ROLE, PD3  },    // PD3   MISO
        { GPIO_AF1_SPI2, MOSI_PIN_ROLE, PD4  },    // PD4   MOSI    PD1/PD3/PD4
#endif
        {      0,              -1,      0,  }      // End of Table
      };


              //---------------------------------------------------------------
              // Lookup table for valid SCLK/MISO/MOSI combos for each diff SPI
              //---------------------------------------------------------------
const SPI_GPIO_BLK *  _g_spi_gpio_table_addr[] =
      {                        0L,            // no I2C0 nor I2C default module
        (SPI_GPIO_BLK*) &_g_spi_1_gpio_table,
        (SPI_GPIO_BLK*) &_g_spi_2_gpio_table,
      };


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
       else nvic_irq = SPI2_IRQn;

    HAL_NVIC_SetPriority (nvic_irq, 1, 1);
    HAL_NVIC_EnableIRQ (nvic_irq);
}

/******************************************************************************/
