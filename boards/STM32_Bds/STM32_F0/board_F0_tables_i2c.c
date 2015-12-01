
//2******1*********2*********3*********4*********5*********6*********7**********
//                                                                      STM32 F0
//                           board_F0_tables_i2c.c
//
//
//                                STM32  F0  30                      STM32F030x8
//                                STM32  F0  70                      STM32F070xB
//                                STM32  F0  72                      STM32F072xB
//                                STM32  F0  91                      STM32F091xC
//
//
//             MCU SPECIFIC   I2C   GPIO  PIN   DEFINITIONS
//
//                                   for
//
//                                I2C Support
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
//   These GPIO definitions for TIMx_CHx GPIO mappings are used in common by
//   the PWM logic when setting up PWM outputs, and by the Timer Output Compare
//   and Input Capture functions that use GPIOs.
//
//------------------------------------------------------------------------------
//
//   These GPIO definitions for SPI Module GPIO mappings are used in common by
//   the SPI logic when setting up GPIO pin connections/combinations, that
//   will be used by the requested SPI module.
//
//------------------------------------------------------------------------------
//                    I2C  Module  Summary     available on STM32 F0
//
//           SCL       SDA     Arduino
//          -----     -----    --------
//   I2C1 =  PB8   /   PB9      D15 / D14
//     "     PB6   /   PB7      D10 / -
//   I2C2 =  PB10  /   PB11     D6  / -
//     "     PB13  /   PB14
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
//                           I2C Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------

//   D15    PB_8   I2C1_SCL           -

//   D14    PB_9   I2C1_SDA

//   D13    PA_5   - none -

//   D12    PA_6   - none -

//   D11    PA_7   - none -

//   D10    PB_6   I2C1_SCL

//   D9     PC_7   - none -

//   D8     PA_9   I2C1_SCL  (F0_91 only)

//   D7     PA_8   - none -

//   D6     PB_10  I2C2_SCL

//   D5     PB_4   - none -

//   D4     PB_5   I2C1_SMBA

//   D3     PB_3   - none -

//   D2     PA_10  I2C1_SDA  (F0_91 only)

//   D1     PA_2   - none -

//   D0     PA_3   - none -

//   A0     PA_0   - none -

//   A1     PA_1   - none -

//   A2     PA_4   - none -

//   A3     PB_0   - none -

//   A4     PC_1   - none -

//   A5     PC_0   - none -

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//
//   CNx  PA_0-10  - none -
//   CNx    PA_11  I2C2_SCL  (F0_91 only)
//   CNx    PA_12  I2C2_SDA  (F0_91 only)
//   CNx PA_13-15  - none -

//   CNx    PB_1   - none -
//   CNx    PB_7   I2C1_SDA
//   CNx    PB_11  I2C2_SDA
//   CNx    PB_12  - none -
//   CNx    PB_13  I2C2_SCL
//   CNx    PB_14  I2C2_SDA
//   CNx    PB_15  - none -

//   PC_0 - PC_15  - none -

//   PD_0 - PD_15  - none -

//   PE_0 - PE_15  - none -

//   CNx    PF_0   I2C1_SDA  (F0_91 only)
//   CNx    PF_1   I2C1_SCL  (F0_91 only)
//   PF_2 - PF_10  - none -

//------------------------------------------------------------------------------


int  board_i2c_enable_clock (int module_id);         // internal routines


const I2C_TypeDef  *_g_i2c_HW_module_base [] =
          {  I2C1,                  // [0] = default module = I2C_AUTO_MODULE
             I2C1,
             I2C2,
              0L,
              0L                    // Max allowed I2C modules is 6 (F7)
          };

                           //----------------------------------------
                           //   I2Cx handles needed, one per module
                           //----------------------------------------
    I2C_HandleTypeDef  _g_i2c1_hal_handle;           // I2C 1 support
    I2C_HandleTypeDef  _g_i2c2_hal_handle;           // I2C 2 support

//  DMA_HandleTypeDef  _g_hdma_i2c1_rx;
//  DMA_HandleTypeDef  _g_hdma_i2c1_tx;


                            //----------------------------------------------------------
                            // List of HAL Handle Addresses for each separate I2C Handle
                            //----------------------------------------------------------
const I2C_HandleTypeDef *_g_i2c_typedef_handle_addr []   // HAL API Handles
                     = {    0L,                          // no SPI0  on F0
                         (I2C_HandleTypeDef*) &_g_i2c1_hal_handle,  // I2C1
                         (I2C_HandleTypeDef*) &_g_i2c2_hal_handle,  // I2C2
                       };

                            //-------------------------------------------------------
                            // I2Cn I/O Buffer Control blocks needed, one per module
                            //-------------------------------------------------------
    I2C_IO_BUF_BLK  _g_i2c_1_io_blk = { 0 };      // clear out to zeros at startup
    I2C_IO_BUF_BLK  _g_i2c_2_io_blk = { 0 };

                            //----------------------------------------------------------
                            // List of Addresses, one for each separate I2C_IO_BLK
                            //----------------------------------------------------------
const I2C_IO_BUF_BLK *_g_i2c_io_blk_address []   // I2C I/O Block addresses
                       = {       0L,             // no I2C0  on STM32
                           (I2C_IO_BUF_BLK*) &_g_i2c_1_io_blk,  // I2C1
                           (I2C_IO_BUF_BLK*) &_g_i2c_2_io_blk,  // I2C2
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
const I2C_GPIO_BLK  _g_i2c_1_gpio_table [] =
      {                                            //     SCL/SDA
        { GPIO_AF1_I2C1, SCL_PIN_ROLE, PB6  },     // PB6   SCL    D10
        { GPIO_AF1_I2C1, SCL_PIN_ROLE, PB8  },     // PB8   SCL    D15
        { GPIO_AF1_I2C1, SDA_PIN_ROLE, PB7  },     // PB7   SDA
        { GPIO_AF1_I2C1, SDA_PIN_ROLE, PB9  },     // PB9   SDA    D14

#if defined(STM32F091xC) || defined(STM32F070xB)
        { GPIO_AF1_I2C1, SCL_PIN_ROLE, PF1  },     // PF1   SCL        // PF1/PF0
        { GPIO_AF1_I2C1, SDA_PIN_ROLE, PF0  },     // PF0   SDA
        { GPIO_AF4_I2C1, SCL_PIN_ROLE, PA9  },     // PA9   SCL        // PA9/PA10
        { GPIO_AF4_I2C1, SDA_PIN_ROLE, PA10 },     // PA10  SDA
#endif
        {      0,              -1,      0,  }      // End of Table
      };


const I2C_GPIO_BLK  _g_i2c_2_gpio_table [] =
      {
        { GPIO_AF1_I2C2, SCL_PIN_ROLE, PB10 },     // PB10  SCL    D6  // PB10/PB11
        { GPIO_AF1_I2C2, SDA_PIN_ROLE, PB11 },     // PB11  SDA

#if ! defined(STM32F030x8)
        { GPIO_AF5_I2C2, SCL_PIN_ROLE, PB13 },     // PB13  SCL        // PB13/PB14
        { GPIO_AF5_I2C2, SDA_PIN_ROLE, PB14 },     // PB14  SDA
#endif
#if defined(STM32F091xC) || defined(STM32F030x8)
        { GPIO_AF5_I2C2, SCL_PIN_ROLE, PA11 },     // PA11  SCL        // PA11/PA12
        { GPIO_AF5_I2C2, SDA_PIN_ROLE, PA12 },     // PA12  SDA
#endif
        {      0,              -1,      0, }           // End of Table
      };


               //--------------------------------------------------------------
               //  Lookup table for valid RX/TX combos for each different UART
               //--------------------------------------------------------------
const I2C_GPIO_BLK *  _g_i2c_gpio_table_addr[] =
      {                        0L,            // no I2C0 nor I2C default module
        (I2C_GPIO_BLK*) &_g_i2c_1_gpio_table,
        (I2C_GPIO_BLK*) &_g_i2c_2_gpio_table,
      };




//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************


//******************************************************************************
//  board_i2c_enable_clock
//
//           Ensures that the specified Timer's clock is turned on.
//******************************************************************************
int  board_i2c_enable_clock (int module_id)
{
      //--------------------------------------------------------
      // Turn on clock for the associated I2Cx module.
      //--------------------------------------------------------
    switch (module_id)
     { case 1:                           // I2C1
             __I2C1_CLK_ENABLE();
             break;
        case 2:                          // I2C2
             __I2C2_CLK_ENABLE();
             break;
     }

    return (0);          // denote everything worked OK
}


/******************************************************************************/
