
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_L4_tables_adc.c
//
//
//                              STM32  L4 76 VG   (Discovery)        STM32L476xx / USE_STM32L476G_DISCO_REVB
//                              STM32  L4 76 RG   (Nucleo)           STM32L476xx / USE_STM32L4XX_NUCLEO
//
//
//                      MCU SPECIFIC    ADC    GPIO  PIN   DEFINITIONS
//
//  History:
//    08/25/15 - Verified tables. 16 physical channels on ADC1/2/3.  Duq
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


                     //---------------------------------------------------
                     //       Key portability constants / #defines
                     //---------------------------------------------------
#define  ADC_MAX_MODULES     1
#define  ADC_MAX_CHANNELS   16



//------------------------------------------------------------------------------
//                             ADC Pinout Definitions


//
//  ??? !!!   THE FOLLOWING NEED TO BE UPDATED/CHANGED FOR ADCs      ??? !!!


//                 TIMER_2            TIMER_5
//   A0     PA_0   TIMER_CHANNEL_1    TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5
//   A1     PA_1   TIMER_CHANNEL_2    TIMER_CHANNEL_2

//   A2     PA_4   - none -

//                 TIMER_3            TIMER_1
//   A3     PB_0   TIMER_CHANNEL_3    TIMER_CHANNEL_2_N

//   A4     PC_1   - none -

//   A5     PC_0   - none -

//
//          MCU
// Arduino  Pin    Primary          Alternate(s)
// -------  ---    -------          ------------
//                 TIMER_4            TIMER_10
//   D15    PB_8   TIMER_CHANNEL_3    TIMER_CHANNEL_1

//                 TIMER_4            TIMER_11
//   D14    PB_9   TIMER_CHANNEL_4    TIMER_CHANNEL_1

//                 TIMER_2
//   D13    PA_5   TIMER_CHANNEL_1   -                     -

//                 TIMER_3
//   D12    PA_6   TIMER_CHANNEL_1_ALT2

//                 TIMER_1            TIMER_3
//   D11    PA_7   TIMER_CHANNEL_1_N  TIMER_CHANNEL_2_ALT2

//                 TIMER_4
//   D10    PB_6   TIMER_CHANNEL_1

//                                    TIMER_3
//   D9     PC_7                      TIMER_CHANNEL_2_ALT2

//                 TIMER_1
//   D8     PA_9   TIMER_CHANNEL_2

//                 TIMER_1
//   D7     PA_8   TIMER_CHANNEL_1

//                 TIMER_2
//   D6     PB_10  TIMER_CHANNEL_3

//                 TIMER_3
//   D5     PB_4   TIMER_CHANNEL_1

//                 TIMER_3
//   D4     PB_5   TIMER_CHANNEL_2

//                 TIMER_2
//   D3     PB_3   TIMER_CHANNEL_2

//                 TIMER_1
//   D2     PA_10  TIMER_CHANNEL_3

//                 TIMER_2            TIMER_5                TIMER_9
//   D1     PA_2   TIMER_CHANNEL_3    TIMER_CHANNEL_3_ALT1   TIMER_CHANNEL_1

//                 TIMER_2            TIMER_5                TIMER_9
//   D0     PA_3   TIMER_CHANNEL_4    TIMER_CHANNEL_4_ALT1   TIMER_CHANNEL_2

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// Morpho   Pin    Primary           Alternate(s)
// -------  ---    -------           ------------
//                 TIMER_1
//   CNx    PA_11  TIMER_CHANNEL_4

//                 TIMER_3            TIMER_1
//   CNx    PB_1   TIMER_CHANNEL_4    TIMER_CHANNEL_3_N
//                 TIMER_4
//------------------------------------------------------------------------------

              //-----------------------------------------------
              // ADC Channel Definitions
              //    - Which channels supported and their GPIOs
              //------------------------------------------------
// ADC1 is defined as: #define ADC1   ((ADC_TypeDef *) ADC1_BASE) in stm32f401xe.h
// note: on STM32, user indexes start at 0, to handle channel 0 on STM's
//       Mask of 7 = supported on all 3 ADCs (1/2/3). Mask of 3 = support on ADC 1/2 only
//
const ADC_CHANNEL_BLK  _g_adc_channels [] =
       {
               // NOTE: the Physical Channel layout on L4 is _MUCH_ different than other STM32 MCUs
          {   0,       0, 0,ADC_CHANNEL_TEMPSENSOR,1,ADC1 }, // Internal TEMP -4    ADC_CHANNEL_17
          {   0,       0,   0,        0,           0,ADC1 }, // no Internal VCOMP -3
          {   0,       0,   0, ADC_CHANNEL_VBAT,   1,ADC1 }, // Internal VBAT -2    ADC_CHANNEL_18
          {   0,       0,   0, ADC_CHANNEL_VREFINT,1,ADC1 }, // Internal VREF -1    ADC_CHANNEL_0
          {   0,       0,      0,             0,  0,  0   }, // Channel 0 is reserved on L4 for VREFINT
                                                                 // Arduino Index ADC
          { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_1,  7, ADC1 }, // PC0    A5    10  1/2/3
          { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_2,  7, ADC1 }, // PC1    A4    11  1/2/3
          { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_3,  7, ADC1 }, // PC2          12  1/2/3
          { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_4,  7, ADC1 }, // PC3          13  1/2/3
          { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_5,  3, ADC1 }, // PA0    A0     0  1/2
          { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_6,  3, ADC1 }, // PA1    A1     1  1/2
          { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_7,  3, ADC1 }, // PA2    VCP    2    -
          { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_8,  3, ADC1 }, // PA3    VCP    3    -
          { GPIOA, GPIO_PIN_4, 0, ADC_CHANNEL_9,  3, ADC1 }, // PA4    A2     4  1/2
          { GPIOA, GPIO_PIN_5, 0, ADC_CHANNEL_10, 3, ADC1 }, // PA5           5  1/2
          { GPIOA, GPIO_PIN_6, 0, ADC_CHANNEL_11, 3, ADC1 }, // PA6           6  1/2
          { GPIOA, GPIO_PIN_7, 0, ADC_CHANNEL_12, 3, ADC1 }, // PA7           7  1/2
          { GPIOC, GPIO_PIN_4, 0, ADC_CHANNEL_13, 3, ADC1 }, // PC4          14  1/2
          { GPIOC, GPIO_PIN_5, 0, ADC_CHANNEL_14, 3, ADC1 }, // PC5          15  1/2
          { GPIOB, GPIO_PIN_0, 0, ADC_CHANNEL_15, 3, ADC1 }, // PB0    A3     8  1/2
          { GPIOB, GPIO_PIN_1, 0, ADC_CHANNEL_16, 3, ADC1 }, // PB1           9  1/2
       };


              //-----------------------------------------------
              // Triggerable Timers/Events available on L4 ADC
              //    - TIM1_TRGO      - TIM1_CC4
              //    - TIM2_TRGO      - TIM3_TRGO
              //    - TIM15_TRGO     -  etc
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
         { ADC_TRIGGER_TIMER_1_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIG_T1_CC1  },  // TImer1 CC1
         { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIG_T1_CC2  },  // TImer1 CC2
         { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIG_T1_CC3  },  // TImer1 CC3
         { ADC_TRIGGER_TIMER_1,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T1_TRGO },  // TImer1 Update/TRG0
         { ADC_TRIGGER_TIMER_1_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T1_TRGO2},  // TImer1 Update/TRG02  ??? TILT ???

         { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIG_T2_CC2  },  // TImer2 CC2
         { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T2_TRGO },  // TImer2 Update/TRG0

         { ADC_TRIGGER_TIMER_3_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIG_T3_CC4  },  // TImer3 CC4
         { ADC_TRIGGER_TIMER_3,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T3_TRGO },  // TImer3 Update/TRG0

         { ADC_TRIGGER_TIMER_4_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIG_T4_CC4  },  // TImer4 CC4
         { ADC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T4_TRGO },  // TImer4 Update/TRG0

         { ADC_TRIGGER_TIMER_6,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T6_TRGO },  // TImer6 Update/TRG0

         { ADC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T8_TRGO },  // TImer8 Update/TRG0
         { ADC_TRIGGER_TIMER_8_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T8_TRGO2},  // TImer8 Update/TRG02  ??? TILT ???

         { ADC_TRIGGER_TIMER_15,    TIM_TRGO_UPDATE, ADC_EXTERNALTRIG_T15_TRGO }, // TImer15 Update/TRG0

         { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIG_EXT_IT11 },  // External GPIO
         {     -1,                        0,                     0             }   // end of table
        };



//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_adc_get_channel_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the channels supported, and which GPIOs are used.
//******************************************************************************
ADC_CHANNEL_BLK  *board_adc_get_channel_block (int module_id, int channel_num)
{
    ADC_CHANNEL_BLK  *chan_blkp;

       // Note that addressing starts at -4, so we must add 4 to get 0 based
    chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num+INT_OFFSET];

    return (chan_blkp);
}


//******************************************************************************
//  board_adc_get_trigger_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the triggers supported, and which ExTSEL and MMS
//            Masks are used.
//******************************************************************************
ADC_TRIGGER_BLK  *board_adc_get_trigger_block (int module_id, int trigger_type)
{
    ADC_TRIGGER_BLK  *trigblkp;

                       // Point at the first entry in the table
    trigblkp = (ADC_TRIGGER_BLK*) &_g_adc_trigger_config_table[0];

                       // scan through the table to find a matching trigger type
    while (1)
      { if (trigblkp->trigger_user_api_id == -1)
           return (0L);             // end of table and no match
           if (trigblkp->trigger_user_api_id == trigger_type)
              {     // we got a successful math. Return it to caller
                 break;             // bail out of loop
              }
             else trigblkp++;       // otherwise, step to nexct entry in array
      }

    return (trigblkp);
}


//******************************************************************************
//  board_adc_enable_clock
//
//           Enable the clock for a specifc ADC module.
//           Requires the caller previously invoked board_adc_get_status_block()
//
//  Note that ADC_AUTO_MODULE = 0, so it is the first entry in the lookup table
//******************************************************************************
void  board_adc_enable_clocks (int module_id)
{
    __HAL_RCC_ADC_CLK_ENABLE();          // Enable ADC Periph clock

         // VVVV L4 only. extra step after __HAL_RCC_ADC_CLK_ENABLE() VVVV
    __HAL_RCC_ADC_CONFIG (RCC_ADCCLKSOURCE_SYSCLK);        // L4 only

         // Enable the ADC module's associated DMA module's clocks
    __HAL_RCC_DMA1_CLK_ENABLE();       // Enable DMA clock to be used with ADC
}



#if  TEST_APP
//*****************************************************************************
//*****************************************************************************
//                         TEST  APP
//*****************************************************************************
//*****************************************************************************

/**
  ******************************************************************************
  * @file    ADC/ADC_DMA_Transfer/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   This example describes how to use the DMA to convert continuously data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
*******************************************************************************/

//#include "main.h"

#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define DMAx_CHANNELx_CLK_ENABLE()      __HAL_RCC_DMA1_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOA

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_5
#define SAMPLINGTIME                    ADC_SAMPLETIME_6CYCLES_5


                               /* ADC handle declaration */
ADC_HandleTypeDef        AdcHandle;

                               /* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef   sConfig;

                               /* Converted value declaration */
uint32_t                 aResultDMA;


/***********************************************************************
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int  main (void)
{
       //------------------------------------------------------------
       // This sample code shows how to convert an analog input and
       // read the converted data using DMA transfer.
       // To proceed, 4 steps are required: */
       //------------------------------------------------------------

       /* STM32L4xx HAL library initialization:
       *  - Configure the Flash prefetch
       *  - Systick timer is configured by default as source of time base, but user
       *    can eventually implement his proper time base source (a general purpose
       *    timer for example or other time source), keeping in mind that Time base
       *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
       *    handled in milliseconds basis.
       *  - Set NVIC Group Priority to 4
       *  - Low Level Initialization
       */
  HAL_Init();

       /* Configure LED1 */
  BSP_LED_Init(LED1);

       /* Configure the system clock to 80 MHz */
  SystemClock_Config();


       /* Compute prescaler value to have TIM1 counter clock equal to 16,000,000 Hz */
  uhPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;


       /* ### - 1 - Initialize ADC peripheral ############################ */
  AdcHandle.Instance          = ADCx;
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
     {
         /* ADC de-initialization Error */
       Error_Handler();
     }

  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* DMA circular mode selected */
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */

      /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
     {
       Error_Handler();
     }

      /* ### - 2 - Start calibration ###################################### */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
     {
       Error_Handler();
     }

      /* ### - 3 - Channel configuration ################################## */
  sConfig.Channel      = ADCx_CHANNEL;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
     {
       Error_Handler();
     }

      /* ### - 4 - Start conversion in DMA mode ########################### */
  if (HAL_ADC_Start_DMA(&AdcHandle, &aResultDMA, 1) != HAL_OK)
     {
       Error_Handler();
     }

              /* Infinite Loop */
  while (1)
    {
    }
}


/*********************************************************************
* @brief  ADC MSP Init
* @param  hadc : ADC handle
* @retval None
*********************************************************************/
void  HAL_ADC_MspInit (ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef           GPIO_InitStruct;
  DMA_HandleTypeDef          DmaHandle;

      /*##-1- Enable peripherals and GPIO Clocks #########################*/
                          /* ADC Periph clock enable */
  ADCx_CLK_ENABLE();
                          /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
                          /* Enable DMA clock */
  DMAx_CHANNELx_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

      /*##- 2- Configure peripheral GPIO #############################*/
      /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin  = ADCx_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

      /*##- 3- Configure DMA #########################################*/
      /***************** Configure DMA parameters *********************/
  DmaHandle.Instance                 = DMA1_Channel1;
  DmaHandle.Init.Request             = DMA_REQUEST_0;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  DmaHandle.Init.Mode                = DMA_CIRCULAR;
  DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;

      /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit (&DmaHandle);
  HAL_DMA_Init (&DmaHandle);

      /* Associate the DMA handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

      /* NVIC configuration for DMA Input data interrupt */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


/*********************************************************************
*    This function handles DMA1_Channel1_IRQHandler interrupt request.
*
**********************************************************************/
void  DMA1_Channel1_IRQHandler (void)
{
    HAL_DMA_IRQHandler (AdcHandle.DMA_Handle);
}

#endif                        // if TEST_APP


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
