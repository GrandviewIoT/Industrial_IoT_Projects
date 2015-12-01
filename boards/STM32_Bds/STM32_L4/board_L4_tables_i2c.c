
//2******1*********2*********3*********4*********5*********6*********7**********
//                                                                      STM32 L4
//                           board_L4_tables_i2c.c
//
//
//                              STM32  L4 76 VG   (Discovery)        STM32L476xx / USE_STM32L476G_DISCO_REVB
//                              STM32  L4 76 RG   (Nucleo)           STM32L476xx / USE_STM32L4XX_NUCLEO
//
//
//               MCU SPECIFIC   I2C   GPIO  PIN   DEFINITIONS
//
//                                     for
//
//                                 I2C Support
//
//  History:
//    08/17/15 - Verified tables. Duq
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

/********1*********2*********3*********4*********5*********6*********7**********
*
*  Copyright (C) 2015 Grandview Systems
*  All rights reserved
*
* MIT License
*
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*
* - - - - ALTERNATIVE  -  -  -  -
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Grandview Systems nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


//*******1*********2*********3*********4*********5*********6*********7**********
//******************************************************************************
//
//                            board_F4x1_tables_timer.c
//
//
//                                STM32  F4  01                     STM32F401xE
//                                STM32  F4  11                     STM32F411xE
//
//
//                       MCU  SPECIFIC  GPIO  PIN  DEFINITIONS
//
//                                   for
//
//                                   I2C
//******************************************************************************
//******************************************************************************

//------------------------------------------------------------------------------
//                           I2C Pinout Definitions
//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//   D15    PB_8   I2C1_SCL

//   D14    PB_9   I2C1_SDA

//   D13    PA_5   - none -

//   D12    PA_6   - none -

//   D11    PA_7   - none -

//   D10    PB_6   I2C1_SCL

//   D9     PC_7   - none -

//   D8     PA_9   - none -

//   D7     PA_8   - none -

//   D6     PB_10  I2C2_SCL

//   D5     PB_4   - none -

//   D4     PB_5   I2C1_SMBA

//   D3     PB_3   - none -

//   D2     PA_10  - none -

//   D1     PA_2   - none -

//   D0     PA_3   - none -

//   A0     PA_0   - none -

//   A1     PA_1   - none -

//   A2     PA_4   - none -

//   A3     PB_0   - none -

//   A4     PC_1   I2C3_SCL

//   A5     PC_0   I2C3_SDA

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_1
//   CNx  PA_11-14 - none -
//   CNx    PA_15  - none -

//   CNx    PB_1   - none -
//   CNx    PB_2   I2C3_SMBA
//   CNx    PB_7   I2C1_SDA
//   CNx    PB_11  I2C2_SDA
//   CNx    PB_12  I2C2_SMBA
//   CNx    PB_13  I2C2_SCL
//   CNx    PB_14  I2C2_SDA
//   CNx    PB_15  - none -

//   CNx    PC_2   - none -
//   CNx    PC_3   - none -
//   PC_4 - PC_5   - none -
//   CNx    PC_6   - none -
//   CNx    PC_7   - none -
//   CNx    PC_8   - none -
//   CNx    PC_9   - none -
//   CNx    PC_10  - none -
//   CNx    PC_11  - none -
//   CNx    PC_12  - none -
//  PC_13 - PC_15  - none -

//   PD_0 - PD_2   - none -
//   CNx    PD_3   - none -
//   PD_4 - PD_5   - none -
//   CNx    PD_6   - none -
//   PD_7 - PD_15  - none -

//   PE_0 - PE_1   - none -
//   CNx    PE_2   - none -
//   CNx    PE_3   - none -
//   CNx    PE_4   - none -
//   CNx    PE_5   - none -
//   CNx    PE_6   - none -
//   PE_7 - PE_10  - none -
//   CNx    PE_11  - none -
//   CNx    PE_12  - none -
//   CNx    PE_13  - none -
//   CNx    PE_14  - none -
//   CNx    PE_15  - none -

//   CNx    PF_0   I2C2_SDA
//   CNx    PF_1   I2C2_SCL
//   CNx    PF_2   I2C2_SMBA

//   CNx    PG_6   I2C3_SMBA
//   CNx    PG_7   I2C3_SCL
//   CNx    PG_8   I2C3_SDA
//   CNx    PG_13  I2C1_SDA
//   CNx    PG_14  I2C1_SCL
//   CNx    PG_15  I2C1_SMBA

//------------------------------------------------------------------------------


int  board_i2c_enable_clock (int module_id);         // internal routines


const I2C_TypeDef  *_g_i2c_HW_module_base [] =
          {  I2C1,                  // [0] = default module = I2C_AUTO_MODULE
             I2C1,
             I2C2,
             I2C3,
              0L,
              0L,
              0L                    // Max allowed I2C modules is 8 (F7)
          };

                            //----------------------------------------
                            //   I2Cn handles needed, one per module
                            //----------------------------------------
    I2C_HandleTypeDef  _g_hi2c1_handle;           // I2C 1 support
    I2C_HandleTypeDef  _g_hi2c2_handle;           // I2C 2 support
    I2C_HandleTypeDef  _g_hi2c3_handle;           // I2C 3 support

    DMA_HandleTypeDef  _g_hdma_i2c2_rx;
    DMA_HandleTypeDef  _g_hdma_i2c2_tx;


                            //----------------------------------------------------------
                            // List of HAL Handle Addresses, one for each separate I2C Handle
                            //----------------------------------------------------------
const I2C_HandleTypeDef *_g_i2c_typedef_handle_addr []   // HAL API Handles
                     = {    0L,                          // no I2C0  on L4
                         (I2C_HandleTypeDef*) &_g_hi2c1_handle,     // I2C1
                         (I2C_HandleTypeDef*) &_g_hi2c2_handle,     // I2C2
                         (I2C_HandleTypeDef*) &_g_hi2c3_handle,     // I2C3
                       };


                            //-------------------------------------------------------
                            // I2Cn I/O Buffer Control blocks needed, one per module
                            //-------------------------------------------------------
    I2C_IO_BUF_BLK  _g_i2c_1_io_blk = { 0 };      // clear out to zeros at startup
    I2C_IO_BUF_BLK  _g_i2c_2_io_blk = { 0 };
    I2C_IO_BUF_BLK  _g_i2c_3_io_blk = { 0 };

                            //----------------------------------------------------------
                            // List of Addresses, one for each separate I2C_IO_BLK
                            //----------------------------------------------------------
const I2C_IO_BUF_BLK *_g_i2c_io_blk_address []   // I2C I/O Block addresses
                       = {       0L,             // no I2C0  on STM32
                           (I2C_IO_BUF_BLK*) &_g_i2c_1_io_blk,   // I2C1
                           (I2C_IO_BUF_BLK*) &_g_i2c_2_io_blk,   // I2C2
                           (I2C_IO_BUF_BLK*) &_g_i2c_3_io_blk,   // I2C3
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
        { GPIO_AF4_I2C1, SCL_PIN_ROLE, PB6  },     // PB6   SCL   D10
        { GPIO_AF4_I2C1, SCL_PIN_ROLE, PB8  },     // PB8   SCL   D15
        { GPIO_AF4_I2C1, SDA_PIN_ROLE, PB7  },     // PB7   SDA
        { GPIO_AF4_I2C1, SDA_PIN_ROLE, PB9  },     // PB9   SDA   D14

        { GPIO_AF4_I2C1, SCL_PIN_ROLE, PG14 },     // PG14  SCL
        { GPIO_AF4_I2C1, SDA_PIN_ROLE, PG13 },     // PG13  SDA    // PG14/PG13
        {      0,              -1,      0,  }      // End of Table
      };


const I2C_GPIO_BLK  _g_i2c_2_gpio_table [] =
      {
        { GPIO_AF4_I2C2, SCL_PIN_ROLE, PB10 },     // PB10  SCL  D6
        { GPIO_AF4_I2C2, SCL_PIN_ROLE, PB13 },     // PB13  SCL
        { GPIO_AF4_I2C2, SCL_PIN_ROLE, PF1  },     // PF1   SCL
        { GPIO_AF4_I2C2, SDA_PIN_ROLE, PB11 },     // PB11  SDA       // PB10/PB11
        { GPIO_AF4_I2C2, SDA_PIN_ROLE, PB14 },     // PB14  SDA       // PB13/PB14
        { GPIO_AF4_I2C2, SDA_PIN_ROLE, PF0  },     // PF0   SDA       // PF1/PF0
        {      0,              -1,      0,  }      // End of Table
      };


const I2C_GPIO_BLK  _g_i2c_3_gpio_table [] =
      {
        { GPIO_AF4_I2C3, SCL_PIN_ROLE, PC0  },     // PC0   SCL  D7
        { GPIO_AF4_I2C3, SCL_PIN_ROLE, PG7  },     // PG7   SCL
        { GPIO_AF4_I2C3, SDA_PIN_ROLE, PC1  },     // PC1   SDA  D5   // PC0/PC1  A4/A5
        { GPIO_AF4_I2C3, SDA_PIN_ROLE, PG8  },     // PG8   SDA       // PG7/PG8
        {      0,              -1,      0,  }      // End of Table
      };

               //--------------------------------------------------------------
               //  Lookup table for valid RX/TX combos for each different UART
               //--------------------------------------------------------------
const I2C_GPIO_BLK *  _g_i2c_gpio_table_addr[] =
      {                        0L,            // no I2C0 nor I2C default module
        (I2C_GPIO_BLK*) &_g_i2c_1_gpio_table,
        (I2C_GPIO_BLK*) &_g_i2c_2_gpio_table,
        (I2C_GPIO_BLK*) &_g_i2c_3_gpio_table,
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
        case 3:                          // I2C3
             __I2C3_CLK_ENABLE();
             break;
     }
    return (0);          // denote everything worked OK
}



//******************************************************************************
//  board_i2c_enable_clock
//
//           Ensures that the specified Timer's clock is turned on.
//******************************************************************************
void  board_i2c_enable_nvic_irq (int module_id)
{
    uint32_t   nvic_irq;
    uint32_t   nvic_errq;

      //--------------------------------------------------------
      // Enable the NVIC for the associated I2C module
      //--------------------------------------------------------
    if (module_id == 1)
       { nvic_irq  = I2C1_EV_IRQn;
         nvic_errq = I2C1_ER_IRQn;
       }
      else if (module_id == 2)
       { nvic_irq  = I2C2_EV_IRQn;
         nvic_errq = I2C2_ER_IRQn;
       }
      else { nvic_irq  = I2C3_EV_IRQn;
             nvic_errq = I2C2_ER_IRQn;
           }

         /* Enable and set I2C _NORMAL_ Interrupt to the highest priority */
    HAL_NVIC_SetPriority (nvic_irq, 1, 1);
    HAL_NVIC_EnableIRQ (nvic_irq);

         /* Enable and set I2C _ERROR_ Interrupt to the highest priority */
    HAL_NVIC_SetPriority (nvic_errq, 0, 0);   // this is mainly a F4 thing
    HAL_NVIC_EnableIRQ (nvic_errq);
}

/******************************************************************************/
