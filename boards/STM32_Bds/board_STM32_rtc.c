
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           board_STM32_rtc.c
//
//
//  Common Logic for Real-Time_Clock support for STM32 MCUs.
//
//  RTC support is needed for timestamping of data etc (MEMS sensors, etc).
//  This is used for such things a Data Logging, etc.
//
//
//  History:
//    08/03/15 - Added for Industrial IoT OpenSource project data logging.  Duq
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
//*****************************************************************************

#include "user_api.h"

     //-------------------------------------------------------------------------
     // Global Defines needed by RTCC support
     //-------------------------------------------------------------------------

typedef struct adc_module_def          /* ADC Module definitions */
    {
        RTC_TypeDef  *adc_base;        /* ADC base address */
    } RTC_MODULE_BLK;

void  board_rtc_enable_clocks (int module_id);


     //------------------------------------------------------------------------
     //
     // Pull in        MCU dependent #defines
     //
     //------------------------------------------------------------------------

#if defined(__STM32F072__) || defined(STM32F072xB)
//                                         STM32 - F072  Nucleo
//#include "STM32_F0/board_F0_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(__STM32F091__) || defined(STM32F091xC)
//                                         STM32 - F091  Nucleo
//#include "STM32_F0/board_F0_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1                  // THIS MAY BE INCORRECT - DOES F0_91 support 2 ADCs ?
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(__STM32F103__) || defined(STM32F103xB)
//                                         STM32 - F103  Nucleo
//#include "STM32_F1/board_F1_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  DMA_HAS_FIFO_MODE     1
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(__STM32F303__) || defined(STM32F303xE)
//                                         STM32 - F303  Nucleo
//#include "STM32_F3/board_F3_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(__STM32F334__) || defined(STM32F334x8)
//                                         STM32 - F334  Nucleo
//#include "STM32_F3/board_F3_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
//                                         STM32 - F401  Nucleo
//#include "STM32_F4/board_F4_tables_rtc.c"

#define  ADC_1_MODULES           1
#define  ADC_SINGLE_MODULE       1
#define  ADC_RESOLUTION_DEFLT   ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT     ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT   ADC_SAMPLETIME_56CYCLES    // 56 because of Grove high impedance
#define  ADC_HAS_SQR_L          1
#define  DMA_INSTANCE           DMA2_Stream0
#define  DMA_CHANNEL_ADC1       DMA_CHANNEL_0
#define  DMA_ISR_IRQHandler     DMA2_Stream0_IRQHandler
#define  DMA_STREAM_IRQ         DMA2_Stream0_IRQn
#define  DMA_HAS_FIFO_MODE      1
#define  GPIO_SET_ANALOG_MODE   GPIO_MODE_ANALOG
#endif


#if defined(STM32F746xx) || defined(STM32F746NGHx)
//                                         STM32 - F746  Discovery
//#include "STM32_F7/board_F7_tables_rtc.c"

#define  ADC_3_MODULES          1
#define  ADC_MULTIPLE_MODULES   1
#define  ADC_RESOLUTION_DEFLT   ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT     ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_CHANNEL_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT   ADC_SAMPLETIME_28CYCLES
#define  ADC_HAS_SQR_L          1
#define  DMA_INSTANCE           DMA2_Stream0
#define  DMA_CHANNEL_DEFAULT    DMA_CHANNEL_3    // ADC 3 is F7 default ADC
#define  DMA_CHANNEL_ADC1       DMA_CHANNEL_0
#define  DMA_CHANNEL_ADC2       DMA_CHANNEL_1
#define  DMA_CHANNEL_ADC3       DMA_CHANNEL_2
#define  DMA_CHANNEL_ADC4       DMA_CHANNEL_3
#define  DMA_ISR_IRQHandler     DMA2_Stream0_IRQHandler
#define  DMA_STREAM_IRQ         DMA2_Stream0_IRQn
#define  DMA_HAS_FIFO_MODE      1
#define  GPIO_SET_ANALOG_MODE   GPIO_MODE_ANALOG
#endif


#if defined(STM32L053xx)
//                                         STM32 - L053  Nucleo
//#include "STM32_L0/board_L0_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_LOWPOWER_MODE 1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(STM32L152xE) || defined(STM32L152xC)
//                                         STM32 - L152  Nucleo
//#include "STM32_L1/board_L1_tables_rtc.c"

#define  ADC_1_MODULES         1
#define  ADC_SINGLE_MODULE     1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION12b
#define  ADC_PRESCALE_DEFLT    ADC_CLOCK_ASYNC_DIV2
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_24CYCLES
#define  ADC_HAS_SQR_L         1
#define  ADC_HAS_LOWPOWER_MODE 1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  DMA_HAS_FIFO_MODE     1
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG
#endif


#if defined(STM32L476xx)
//                                         STM32 - L476  Discovery/Nucleo
//#include "STM32_L4/board_L4_tables_rtc.c"

#define  ADC_3_MODULES         1
#define  ADC_MULTIPLE_MODULE   1
#define  ADC_RESOLUTION_DEFLT  ADC_RESOLUTION_12B
#define  ADC_PRESCALE_DEFLT    ADC_CLOCKPRESCALER_PCLK_DIV2
#define  ADC_MODULE_LEVEL_SAMPLING_TIME   1
#define  ADC_SAMPLETIME_DEFLT  ADC_SAMPLETIME_28CYCLES_5
#define  ADC_HAS_SQR_L         1
#define  ADC_HAS_LOWPOWER_MODE 1
#define  DMA_INSTANCE          DMA1_Channel1
#define  DMA_ISR_IRQHandler    DMA1_Channel1_IRQHandler
#define  DMA_STREAM_IRQ        DMA1_Channel1_IRQn
#define  DMA_HAS_FIFO_MODE     1
#define  DMA_REQUEST_NUM       DMA_REQUEST_0
#define  GPIO_SET_ANALOG_MODE  GPIO_MODE_ANALOG_ADC_CONTROL
#endif





//*****************************************************************************
//*****************************************************************************
//                      COMMON   TABLES  and  DEFINEs
//
//                               for
//
//                            RTC  Support
//*****************************************************************************
//*****************************************************************************
     RTC_HandleTypeDef    RtcHandle;


//*****************************************************************************
//*****************************************************************************
//                           COMMON   RTCC   Code
//
//                                    for
//
//                                STM32  MCUs
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_rtc_init
//
//         Initialize an RTC module.
//
//         Valid values for rtc_clock_type:  RTC_CLOCK_SOURCE_LSE
//                                           RTC_CLOCK_SOURCE_LSI
//*****************************************************************************

int  board_rtc_init (int rtc_clock_type)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
    RTC_HandleTypeDef         *hrtc;
    int                       rc;

//#if defined(EXCEEDS_32K_IAR_LIMIT)                   // WVD ADD

    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct
    memset (&PeriphClkInitStruct, 0, sizeof(PeriphClkInitStruct));
    memset (&RtcHandle, 0, sizeof(RtcHandle));

    if (rtc_clock_type == RTC_CLOCK_SOURCE_LSE)
       {         //---------------------------------------------------------
                 //     Setup RTC using LSE as clock source
                 //---------------------------------------------------------
         RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
         RCC_OscInitStruct.LSIState       = RCC_LSI_OFF;
         RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
         RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
         rc = HAL_RCC_OscConfig (&RCC_OscInitStruct);
         if (rc != HAL_OK)
            {
              Error_Handler();      /* Initialization Error */
            }

         PeriphClkInitStruct.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
         PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
         rc = HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);
         if (rc != HAL_OK)
            {
              Error_Handler();     /* Initialization Error */
            }
       }
      else if (rtc_clock_type == RTC_CLOCK_SOURCE_LSI)
       {         //---------------------------------------------------------
                 //     Setup RTC using LSI as clock source
                 //---------------------------------------------------------
         RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
         RCC_OscInitStruct.LSEState       = RCC_LSE_OFF;
         RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
         RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
         rc = HAL_RCC_OscConfig (&RCC_OscInitStruct);
         if (rc != HAL_OK)
            {
              Error_Handler();     /* Initialization Error */
            }

         PeriphClkInitStruct.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
         PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
         rc = HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);
         if (rc != HAL_OK)
            {
              Error_Handler();     /* Initialization Error */
            }
       }
      else return (-1);   // invalid clock source

                 //---------------------------------------------------------
                 // can now enable the RTC clocks
                 //---------------------------------------------------------
    __HAL_RCC_RTC_ENABLE();


                 //---------------------------------------------------------
                 // then configure the baseline RTC support as:
                 //
                 //    - Hour Format    = Format 12
                 //    - Asynch Prediv  = Value according to source clock
                 //    - Synch Prediv   = Value according to source clock
                 //    - OutPut         = Output Disable
                 //    - OutPutPolarity = High Polarity
                 //    - OutPutType     = Open Drain
                 //---------------------------------------------------------
    RtcHandle.Instance            = RTC;
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    rc = HAL_RTC_Init (&RtcHandle);
    if (rc != HAL_OK)
       {
          Error_Handler();     /* Initialization Error */
       }

                 //------------------------------------------------------------
                 // enable the interupt handler ISRs to catch RTC timestamps
                 //------------------------------------------------------------
    HAL_NVIC_SetPriority (TAMP_STAMP_IRQn, 0x0F, 0); // Configure NVIC for RTC TimeStamp
    HAL_NVIC_EnableIRQ (TAMP_STAMP_IRQn);            // and enable the IRQs

//#endif                  // defined(EXCEEDS_32K_IAR_LIMIT)       // WVD ADD

    return (0);           // denote success
}


/*************************************************************************
* @brief  Get the current date         (Month / Day / Year)
*
* @param  hh - the hour value to be set
* @param  mm - the minute value to be set
* @param  ss - the second value to be set
* @retval 0 if worked, else negative error code
*************************************************************************/

int  board_rtc_get_time (uint8_t *year, uint8_t *month, uint8_t *day,
                         uint8_t *day_of_week)
{
    RTC_DateTypeDef sdatestructure;

//#if defined(EXCEEDS_32K_IAR_LIMIT)                   // WVD ADD

    HAL_RTC_GetDate (&RtcHandle, &sdatestructure, FORMAT_BIN);

    *year  = sdatestructure.Year;
    *month = sdatestructure.Month;
    *day   = sdatestructure.Date;
    *day_of_week = sdatestructure.WeekDay;

//#endif                  // defined(EXCEEDS_32K_IAR_LIMIT)       // WVD ADD

    return (0);           // denote success
}


/*************************************************************************
* @brief  Get the current time        (Hour / Minute / Second)
*
* @param  hh - the hour value to be set
* @param  mm - the minute value to be set
* @param  ss - the second value to be set
* @retval 0 if worked, else negative error code
*************************************************************************/
int  board_rtc_get_time (uint8_t *hh, uint8_t *mm, uint8_t *ss,
                         uint8_t *subsec, uint8_t *time_format)
{
    RTC_TimeTypeDef   stimestructure;
    uint8_t           rndsubSec = 0;    // rounded copy of sub-seconds

//#if defined(EXCEEDS_32K_IAR_LIMIT)                   // WVD ADD

    rc = HAL_RTC_GetTime (&RtcHandle, &stimestructure, FORMAT_BIN);
    if (rc != HAL_OK)
       {
          Error_Handler();     /* Initialization Error */
       }

    *time_format = stimestructure.TimeFormat;
    *hh     = stimestructure.Hours;
    *mm     = stimestructure.Minutes;
    *ss     = stimestructure.Seconds;
    rndsubSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV+1))& 0xff);
    *subsec   = rndsubSec;

//#endif                  // defined(EXCEEDS_32K_IAR_LIMIT)       // WVD ADD

    return (0);           // denote success
}



/*************************************************************************
* @brief  Configure the current date         (Month / Day / Year)
*
* @param  hh - the hour value to be set
* @param  mm - the minute value to be set
* @param  ss - the second value to be set
* @retval 0 if worked, else negative error code
*************************************************************************/
int  board_rtc_set_time (uint8_t year, uint8_t month, uint8_t day,
                         uint8_t day_of_week)
{
    RTC_DateTypeDef sdatestructure;

//#if defined(EXCEEDS_32K_IAR_LIMIT)                   // WVD ADD

    memset (&sdatestructure, 0, sizeof(sdatestructure));  // clear struct

    sdatestructure.Year    = year;         // e.g. 0x14 = 2014 year
    sdatestructure.Month   = month;        // e.g. RTC_MONTH_FEBRUARY
    sdatestructure.Date    = day;          // e.g. 0x18 = 18th day of month
    sdatestructure.WeekDay = day_of_week;  // e.g. RTC_WEEKDAY_TUESDAY
    rc = HAL_RTC_SetDate (&RtcHandle, &sdatestructure, FORMAT_BCD);
/// rc = HAL_RTC_SetDate (&RtcHandle, &sdatestructure, FORMAT_BIN);   // ??? WVD Better ???

    if (rc != HAL_OK)
       {
          Error_Handler();     /* Initialization Error */
       }
//#endif                  // defined(EXCEEDS_32K_IAR_LIMIT)       // WVD ADD

    return (0);           // denote success
}


/*************************************************************************
* @brief  Configure the current time        (Hour / Minute / Second)
*
* @param  hh - the hour value to be set
* @param  mm - the minute value to be set
* @param  ss - the second value to be set
* @retval 0 if worked, else negative error code
*************************************************************************/
int  board_rtc_set_time (uint8_t hh, uint8_t mm, uint8_t ss)
{
    RTC_TimeTypeDef   stimestructure;

//#if defined(EXCEEDS_32K_IAR_LIMIT)                   // WVD ADD

    memset (&stimestructure, 0, sizeof(stimestructure));  // clear struct

    stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
    stimestructure.Hours          = hh;
    stimestructure.Minutes        = mm;
    stimestructure.Seconds        = ss;
    stimestructure.SubSeconds     = 0;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
    rc = HAL_RTC_SetTime (&RtcHandle, &stimestructure, FORMAT_BIN);

    if (rc != HAL_OK)
       {
          Error_Handler();     /* Initialization Error */
       }
//#endif                  // defined(EXCEEDS_32K_IAR_LIMIT)       // WVD ADD

    return (0);           // denote success
}

/******************************************************************************/
