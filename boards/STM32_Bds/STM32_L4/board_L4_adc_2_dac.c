/**
  ******************************************************************************
  * @file    ADC/ADC_Sequencer/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   This example provides a short description of how to use the ADC
  *          peripheral with sequencer, to convert several channels.
  *          Channels converted are 1 channel on external pin and 2 internal 
  *          channels (VrefInt and temperature sensor).
  *          Moreover, voltage and temperature are then computed.
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
  ******************************************************************************
  */

//#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l476g_eval.h"

    /* Waveform voltage generation for test:                                      */
    /*  - If this literal is defined: For this example purpose, generates a       */
    /*    waveform voltage on a spare DAC channel, so user has just to connect    */
    /*    a wire between DAC channel output and ADC input to run this example.    */
    /*    (this avoid to user the need of an external signal generator).          */
    /*  - If this literal is not defined: User has to connect an external signal  */
    /*    generator on the selected ADC input to run this example.                */
#define WAVEFORM_VOLTAGE_GENERATION_FOR_TEST

#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

    /* Definition of ADCx channels */
#define ADCx_CHANNELa                   ADC_CHANNEL_5

    /* Definition of ADCx channels pins */
#define ADCx_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNELa_GPIO_PORT         GPIOA
#define ADCx_CHANNELa_PIN               GPIO_PIN_0

    /* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA                        DMA1_Channel1

#define ADCx_DMA_IRQn                   DMA1_Channel1_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel1_IRQHandler

    /* Definition of ADCx NVIC resources */
#define ADCx_IRQn                       ADC1_2_IRQn
#define ADCx_IRQHandler                 ADC1_2_IRQHandler

    /* Definition of DACx  resources */
#define DACx                            DAC1
#define DACx_CLK_ENABLE()               __HAL_RCC_DAC1_CLK_ENABLE()
#define DACx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define DACx_FORCE_RESET()              __HAL_RCC_DAC1_FORCE_RESET()
#define DACx_RELEASE_RESET()            __HAL_RCC_DAC1_RELEASE_RESET()

    /* Definition of DACx channels */
#define DACx_CHANNEL_TO_ADCx_CHANNELa    DAC_CHANNEL_1

    /* Definition of DACx channels pins */
#define DACx_CHANNEL_TO_ADCx_CHANNELa_PIN        GPIO_PIN_4
#define DACx_CHANNEL_TO_ADCx_CHANNELa_GPIO_PORT  GPIOA

    /* Definition of DACx NVIC resources */
#define DACx_IRQn                       TIM6_DAC_IRQn
#define DACx_IRQHandler                 TIM6_DAC_IRQHandler

#define VDD_APPLI                      ((uint32_t) 3300)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t) 4095)    /* Max value with a full range of 12 bits */
#define USERBUTTON_CLICK_COUNT_MAX     ((uint32_t)    4)    /* Maximum value of variable "UserButtonClickCount" */

    /* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    3)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

    /* Internal temperature sensor: constants data used for indicative values in  */
    /* this example. Refer to device datasheet for min/typ/max values.            */
    /* For more accurate values, device should be calibrated on offset and slope  */
    /* for application temperature range.                                         */
    /* At this time, temperature characterization values are not available, 
    *  approximate L4 figures are used. User is expected to resort to data sheet 
    *  to retrieve the final values */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)741)          /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
   


/******************************************************************************
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor standard parameters (refer
  *         to device datasheet).
  *         Computation formula:
  *         Temperature = (VTS - V25)/Avg_Slope + 25
  *         with VTS = temperature sensor voltage
  *              Avg_Slope = temperature sensor slope (unit: uV/DegCelsius)
  *              V25 = temperature sensor @25degC and Vdda 3.3V (unit: mV)
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
******************************************************************************/
#define COMPUTATION_TEMPERATURE_STD_PARAMS(TS_ADC_DATA)                        \
  ((((int32_t)(INTERNAL_TEMPSENSOR_V25 - (((TS_ADC_DATA) * VDD_APPLI) / RANGE_12BITS)   \
     ) * 1000                                                                  \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )


/*******************************************************************************
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)    \
                       ( ((ADC_DATA) * VDD_APPLI) / RANGE_12BITS)
  
  
  
                              /* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
                              /* DAC handler declaration */
DAC_HandleTypeDef    DacHandle;
#endif

                              /* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues [ADCCONVERTEDVALUES_BUFFER_SIZE];

                              /* Variables for ADC conversions results computation to physical values */
uint16_t   uhADCChannelToDAC_mVolt = 0;
uint16_t   uhVrefInt_mVolt         = 0;
 int32_t   wTemperature_DegreeCelsius = 0;
          
                              /* Variables to manage push button on board: 
                              ** interface between ExtLine interruption and main program */
uint8_t         ubUserButtonClickCount = 0;      /* Count number of clicks: Incremented after User Button interrupt */
__IO uint8_t    ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

                              /* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

void SystemClock_Config(void);
static void Error_Handler(void);
static void ADC_Config(void);
#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
static void DAC_Config(void);
#endif 


/******************************************************************************
*                             Main program.
* @param  None
* @retval None
*/
int  main (void)
{
      /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
      /* Configure the system clock to 80 MHz */
  SystemClock_Config();
  
      /*## Configure peripherals ######################################*/
  
      /* Initialize LEDs on board */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED1);
  
      /* Configure Wkup/Tamper push-button in Interrupt mode */
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);
  
      /* Configure the ADC peripheral */
  ADC_Config();
  
      /* Run the ADC calibration in single-ended mode */  
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
     {
          /* Calibration Error */
       Error_Handler();
     }


#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
      /* Configure the DAC peripheral */
  DAC_Config();
#endif
  

      /*## Enable peripherals #############################################*/
  
#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
      /* Set DAC Channel data register: channel corresponding to ADC channel CHANNELa */
      /* Set DAC output to 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V              */
  if (HAL_DAC_SetValue(&DacHandle, DACx_CHANNEL_TO_ADCx_CHANNELa, DAC_ALIGN_12B_R, RANGE_12BITS/2) != HAL_OK)
     {
           /* Setting value Error */
       Error_Handler();
     }
  
       /* Enable DAC Channel: channel corresponding to ADC channel CHANNELa */
  if (HAL_DAC_Start(&DacHandle, DACx_CHANNEL_TO_ADCx_CHANNELa) != HAL_OK)
     {
            /* Start Error */
       Error_Handler();
     }
#endif

      /*## Start ADC conversions #########################################*/
      /* Start ADC conversion on regular group with transfer by DMA */
  if (HAL_ADC_Start_DMA(&AdcHandle,
                        (uint32_t *)aADCxConvertedValues,
                        ADCCONVERTEDVALUES_BUFFER_SIZE
                       ) != HAL_OK)
     {
           /* Start Error */
       Error_Handler();
     }
  
      /* Infinite loop */
  while (1)
  {

        /* Wait for event on push button to perform following actions */
    while ((ubUserButtonClickEvent) == RESET)
      {
      }

        /* Reset variable for next loop iteration */
    ubUserButtonClickEvent = RESET;

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
        /* Set DAC voltage on channel corresponding to ADCx_CHANNELa  */
        /* in function of user button clicks count.                   */
        /* Set DAC output successively to:                            */
        /*  - minimum of full range (0 <=> ground 0V)                 */
        /*  - 1/4 of full range (4095 <=> Vdda=3.3V): 1023 <=> 0.825V */
        /*  - 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V  */
        /*  - 3/4 of full range (4095 <=> Vdda=3.3V): 3071 <=> 2.475V */
        /*  - maximum of full range (4095 <=> Vdda=3.3V)              */
    if (HAL_DAC_SetValue(&DacHandle,
                         DACx_CHANNEL_TO_ADCx_CHANNELa,
                         DAC_ALIGN_12B_R,
                         (RANGE_12BITS * ubUserButtonClickCount / USERBUTTON_CLICK_COUNT_MAX)
                        ) != HAL_OK)
       {
             /* Start Error */
         Error_Handler();
       }
#endif

        /* Wait for DAC settling time */
    HAL_Delay(1);
    
        /* Start ADC conversion */
        /* Since sequencer is enabled in discontinuous mode, this will perform  */
        /* the conversion of the next rank in sequencer.                        */
        /* Note: For this example, conversion is triggered by software start,   */
        /*       therefore "HAL_ADC_Start()" must be called for each conversion.*/
        /*       Since DMA transfer has been initiated previously by function   */
        /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer    */
        /*       active.                                                        */
    if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
       {
         Error_Handler(); 
       }
      
        /* After each intermediate conversion, 
        *   - EOS remains reset (EOS is set only every third conversion)
        *   - EOC is set then immediately reset by DMA reading of DR register.
        * Therefore, the only reliable flag to check the conversion end is EOSMP
        * (end of sampling flag).
        * Once EOSMP is set, the end of conversion will be reached when the successive
        * approximations are over. 
        *
        * RM indicates 12.5 ADC clock cycles for the successive approximations
        * duration with a 12-bit resolution, or 185.ns at 80 MHz.
        * Therefore, it is known that the conversion is over at
        * HAL_ADC_PollForConversion() API return */
    if (HAL_ADC_PollForEvent(&AdcHandle, ADC_EOSMP_EVENT, 10) != HAL_OK)
       {
         Error_Handler(); 
       }  
    
        /* Turn-on/off LED1 in function of ADC sequencer status */
        /*  - Turn-off if sequencer has not yet converted all ranks */    
        /*  - Turn-on if sequencer has converted all ranks */
    if (ubSequenceCompleted == RESET)
     {
      BSP_LED_Off(LED1);
     }
    else
     {
      BSP_LED_On(LED1);
      
          /* Computation of ADC conversions raw data to physical values */
          /* Note: ADC results are transferred into array "aADCxConvertedValues"  */
          /*       in the order of their rank in ADC sequencer.                   */
      uhADCChannelToDAC_mVolt    = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE (aADCxConvertedValues[0]);
      uhVrefInt_mVolt            = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE (aADCxConvertedValues[2]);
      wTemperature_DegreeCelsius = COMPUTATION_TEMPERATURE_STD_PARAMS (aADCxConvertedValues[1]);

          /* Reset variable for next loop iteration */
      ubSequenceCompleted = RESET;
     }
  }
}


/*****************************************************************
*                     ADC MSP initialization
*
*        This function configures the hardware resources used in this example:
*          - Enable clock of ADC peripheral
*          - Configure the GPIO associated to the peripheral channels
*          - Configure the DMA associated to the peripheral
*          - Configure the NVIC associated to the peripheral interruptions
* @param hadc: ADC handle pointer
* @retval None
*/
void  HAL_ADC_MspInit (ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef    GPIO_InitStruct;
  DMA_HandleTypeDef   DmaHandle;
  
      /*##-1- Enable peripherals and GPIO Clocks ###################*/
      /* Enable clock of GPIO associated to the peripheral channels */
  ADCx_CHANNELa_GPIO_CLK_ENABLE();
  
     /* Enable clock of ADCx peripheral */
  ADCx_CLK_ENABLE();

     /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

     /* Enable clock of DMA associated to the peripheral */
  ADCx_DMA_CLK_ENABLE();
  
     /*##-2- Configure peripheral GPIO ############################*/
     /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = ADCx_CHANNELa_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);
  
     /*##-3- Configure the DMA ####################################*/
     /* Configure DMA parameters */
  DmaHandle.Instance = ADCx_DMA;

  DmaHandle.Init.Request             = DMA_REQUEST_0;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
      /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

      /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);
  
      /*##-4- Configure the NVIC #########################################*/

      /* NVIC configuration for DMA interrupt (transfer completion or error) */
      /* Priority: high-priority */
  HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
  

      /* NVIC configuration for ADC interrupt */
      /* Priority: high-priority */
  HAL_NVIC_SetPriority(ADCx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADCx_IRQn);
}


/*****************************************************************
  * @brief  This function handles ADC interrupt request.
  * @param  None
  * @retval None
  */
void ADCx_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&AdcHandle);
}


/*****************************************************************
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void ADCx_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}


#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/*****************************************************************
* @brief  This function handles DAC interrupt request.
* @param  None
* @retval None
*/
void TIM6_DAC_IRQHandler(void)
{
  HAL_DAC_IRQHandler(&DacHandle);
}
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */



/****************************************************************
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 1
  *            PLL_N                          = 20
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source   */
  /* (Default MSI Oscillator enabled at system reset remains ON) */ 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}


/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  
  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;
  
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = ENABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.NbrOfConversion       = 3;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trigger the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because trigger of conversion by software start (no external event) */  
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* DMA circular mode selected */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints.                                   */
  sConfig.Channel      = ADCx_CHANNELa;               /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* Minimum sampling time */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank         = ADC_REGULAR_RANK_2;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel      = ADC_CHANNEL_VREFINT;
  sConfig.Rank         = ADC_REGULAR_RANK_3;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
}

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/**
  * @brief  DAC configuration
  * @note   For this example, generate a signal on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel 
  *         (pin PA.00) and ADC channel (pin PA.00) to run this example.
  *         (this prevents the user from resorting to an external signal generator)
  * @param  None
  * @retval None
  */
static void DAC_Config(void)
{
  static DAC_ChannelConfTypeDef sConfig;

  /* Configuration of DACx peripheral */
  DacHandle.Instance = DACx;

  if (HAL_DAC_DeInit(&DacHandle) != HAL_OK)
  {
    /* DAC deinitialization error */
    Error_Handler();
  }

  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {
    /* DAC initialization error */
    Error_Handler();
  }

  /* Configuration of DAC channel */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL_TO_ADCx_CHANNELa) != HAL_OK)
  {
    /* Channel configuration error */
    Error_Handler();
  }
}
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TAMPER_BUTTON_PIN)
  {
    /* Set variable to report push button event to main program */
    ubUserButtonClickEvent = SET;
  
    /* Manage ubUserButtonClickCount to increment it circularly from 0 to     */
    /* maximum value defined                                                  */
    if (ubUserButtonClickCount < USERBUTTON_CLICK_COUNT_MAX)
    {
      ubUserButtonClickCount++;
    }      
    else
    {
      ubUserButtonClickCount=0;
    }
    
  }
  
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with a potential error */
  
  /* In case of error, LED3 is toggling at a frequency of 1Hz */
  while(1)
  {
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);
    HAL_Delay(500);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
