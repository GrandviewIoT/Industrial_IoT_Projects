
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_F7_tables_adc.c
//
//
//                                STM32  F7  01
//
//
//                      MCU SPECIFIC    ADC    GPIO  PIN   DEFINITIONS
//******************************************************************************

//  int              _g_ADC_complete    = 0;  // 1 = all ADC conversions/DMAs are complete
//  char             _g_adc_clocks_on   = 0;
//  char             _g_active_channels = 0;
//  int              _g_sequencer       = 0;
//  int              _g_step_num        = 1;  // first sequencer step always = 1

//  unsigned char    _g_adc_step_map [16] = { 0 };  // indexed by channel #

//  unsigned short   _g_adc_conv_results[16];    // Internal buf to hold ADC DMAed results

ADC_CB_EVENT_HANDLER _g_adc_callback       = 0L;
    void             *_g_adc_callback_parm = 0;

//  int              _g_trigger_atmrpwm      = 0; // index to correct Timer/PWM
//  uint16_t         _g_trigger_auser_api_id = 0; // User API id for the trigger
//  uint16_t         _g_trigger_atmr_mmsmask = 0; // Associated Mask for TIM's  MMS
//  uint32_t         _g_trigger_adc_extmask  = 0; // Associated mask for ADC CR2 EXTSEL

    int              _g_DMA_complete  = 0;       // 1 = all ADC conversion DMAs are complete
    char             _g_DMA_overrun   = 0;       // 1 = DMA completed a new set before the previous one was processed
    unsigned long    dma_callback_seen = 0;      // DEBUG COUNTERs
    unsigned long    dma_rupt_seen     = 0;

//  ADC_HandleTypeDef       _g_AdcHandle_Mod_1;  // ADC 1 handle
//  ADC_HandleTypeDef       _g_AdcHandle_Mod_2;  // ADC 2 handle
//  ADC_HandleTypeDef       _g_AdcHandle_Mod_3;  // ADC 2 handle
    ADC_ChannelConfTypeDef  _g_ChanConfig;
//  DMA_HandleTypeDef       _g_DmaHandle;

    void  ADC1_DMA_IRQHandler (void);              // Function Prototypes


// note: on STM32, user indexes start at 0, to handle channel 0 on STM's                User  Physical
const ADC_CHANNEL_BLK  _g_adc_channels [] =                            // STM32 Arduino Index  ADC
        {
          {    0,     0,  0,         0,           0, ADC3},  // no Internal TEMP -4  ADC_CHANNEL_17   ??? !!! WVD VERIFY
          {    0,     0,  0,         0,           0, ADC3 }, // no Internal VCOMP -3
          {    0,     0,  0, ADC_CHANNEL_VBAT,    4, ADC3 }, // Internal VBAT -2  ADC_CHANNEL_18
          {    0,     0,  0, ADC_CHANNEL_VREFINT, 4, ADC3 }, // Internal VREF -1  ADC_CHANNEL_17
                                                            //    Arduino Index ADC
          { GPIOA, GPIO_PIN_0, 0, ADC_CHANNEL_0,  4, ADC3 }, // PA0   A0    0    ADC3_IN0
          { GPIOA, GPIO_PIN_1, 0, ADC_CHANNEL_1,  7, ADC3 }, // PA1         1    ADC123_IN1
          { GPIOA, GPIO_PIN_2, 0, ADC_CHANNEL_2,  7, ADC3 }, // PA2         2    ADC123_IN2
          { GPIOA, GPIO_PIN_3, 0, ADC_CHANNEL_3,  7, ADC3 }, // PA3         3    ADC123_IN3
          { GPIOF, GPIO_PIN_6, 0, ADC_CHANNEL_4,  4, ADC3 }, // PF6   A5    4    ADC3_IN4
          { GPIOF, GPIO_PIN_7, 0, ADC_CHANNEL_5,  4, ADC3 }, // PF7   A4    5    ADC3_IN5
          { GPIOF, GPIO_PIN_8, 0, ADC_CHANNEL_6,  4, ADC3 }, // PF8   A3    6    ADC3_IN6
          { GPIOF, GPIO_PIN_9, 0, ADC_CHANNEL_7,  4, ADC3 }, // PF9   A2    7    ADC3_IN7
          { GPIOF, GPIO_PIN_10,0, ADC_CHANNEL_8,  4, ADC3 }, // PF10  A1    8    ADC3_IN8
          { GPIOF, GPIO_PIN_3, 0, ADC_CHANNEL_9,  4, ADC3 }, // PF3         9    ADC3_IN9
          { GPIOC, GPIO_PIN_0, 0, ADC_CHANNEL_10, 4, ADC3 }, // PC0        10    ADC123_10
          { GPIOC, GPIO_PIN_1, 0, ADC_CHANNEL_11, 7, ADC3 }, // PC1        11    ADC123_11
          { GPIOC, GPIO_PIN_2, 0, ADC_CHANNEL_12, 7, ADC3 }, // PC2        12    ADC123_12
          { GPIOC, GPIO_PIN_3, 0, ADC_CHANNEL_13, 7, ADC3 }, // PC3        13    ADC123_13
          { GPIOF, GPIO_PIN_4, 0, ADC_CHANNEL_14, 4, ADC3 }, // PF4        14    ADC3_IN14
          { GPIOF, GPIO_PIN_5, 0, ADC_CHANNEL_15, 4, ADC3 }  // PF5        15    ADC3_IN15
        };

              //-----------------------------------------------
              // Triggerable Timers/Events available on F7 ADC
              //    - TIM1_CC1       - TIM3_CC4
              //    - TIM1_CC2       - TIM2_CC2
              //    - TIM1_CC3       - TIM4_CC4
              //    - TIM1_TRGO      - TIM1_TRGO_2
              //    - TIM2_TRGO      - TIM4_TRGO
              //    - TIM5_TRGO      - TIM6_TRG0
              //    - TIM8_TRGO      - TIM8_TRGO_2
              //------------------------------------------------
const ADC_TRIGGER_BLK _g_adc_trigger_config_table [] =
        {      // User API          TIM MMS         ADC EXTSEL
          { ADC_TRIGGER_TIMER_1_CC1, TIM_TRGO_OC1REF, ADC_EXTERNALTRIGCONV_T1_CC1 },  // TImer1 CC1
          { ADC_TRIGGER_TIMER_1_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T1_CC2 },  // TImer1 CC2
          { ADC_TRIGGER_TIMER_1_CC3, TIM_TRGO_OC3REF, ADC_EXTERNALTRIGCONV_T1_CC3 },  // TImer1 CC3
          { ADC_TRIGGER_TIMER_1,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO},  // TImer1 Update/TRG0
          { ADC_TRIGGER_TIMER_1_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T1_TRGO2}, // TImer1 Update/TRG0-2

          { ADC_TRIGGER_TIMER_2_CC2, TIM_TRGO_OC2REF, ADC_EXTERNALTRIGCONV_T2_CC2 },  // TImer2 CC2
          { ADC_TRIGGER_TIMER_2,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T2_TRGO},  // TImer2 Update/TRG0

          { ADC_TRIGGER_TIMER_3_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T3_CC4 },  // TImer3 CC4

          { ADC_TRIGGER_TIMER_4_CC4, TIM_TRGO_OC4REF, ADC_EXTERNALTRIGCONV_T4_CC4 },  // TImer4 CC4
          { ADC_TRIGGER_TIMER_4,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T4_TRGO},  // TImer4 Update/TRG0

          { ADC_TRIGGER_TIMER_5,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T5_TRGO},  // TImer5 Update/TRG0

          { ADC_TRIGGER_TIMER_6,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T6_TRGO},  // TImer6 Update/TRG0

          { ADC_TRIGGER_TIMER_8,     TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO},  // TImer8 Update/TRG0
          { ADC_TRIGGER_TIMER_8_P2,  TIM_TRGO_UPDATE, ADC_EXTERNALTRIGCONV_T8_TRGO2}, // TImer8 Update/TRG0-2

          { ADC_TRIGGER_GPIO_PIN,          0,         ADC_EXTERNALTRIGCONV_EXT_IT11 },// External GPIO
          {     -1,                        0,                     0               }   // end of table
        };


//*****************************************************************************
//*****************************************************************************
//                         MCU Specific  Utility  Routines
//*****************************************************************************
//*****************************************************************************

//******************************************************************************
//  board_adc_get_control_block
//
//            Locate and return the control/status block, containing key
//            operating fields, flags, and values, based on module id.
//******************************************************************************
ADC_CONTROL_BLK  *board_adc_get_control_block (int module_id)
{
    if (adc_module_id < ADC_CHANNEL_TEMPSENSOR  ||  adc_module_id > ADC_AUTO_MODULE)
       return (0L);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

       // ON L4 / F7, default module must be ADC 3 !!!
    if (module_id == ADC_AUTO_MODULE)
        return (&_g_adc_status_block_3);
       
    return (&_g_adc_status_block_1);        // there is only 1 ADC module on F4
}


//******************************************************************************
//  board_adc_get_channel_block
//
//            Locate and return the channel block for this ADC module.
//            It contains all the channels supported, and which GPIOs are used.
//******************************************************************************
ADC_CHANNEL_BLK  *board_adc_get_channel_block (int module_id, int channel_num)
{
    ADC_CHANNEL_BLK  *chan_blkp;

    chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num+INT_OFFSET];

#if NOT_NEEDED
    if (adc_module_id == 2)
       chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_2_channels [channel_num];       // F3/F7 
       else chan_blkp = (ADC_CHANNEL_BLK*) &_g_adc_1_channels [channel_num];  // F3/F7 
#endif

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
       // ON L4 / F7, default module must be ADC 3 !!!
    if (module_id == 1 || module_id == ADC_AUTO_MODULE)
       __HAL_RCC_ADC1_CLK_ENABLE();                   // Enable ADC Periph clock
       else if (module_id == 2)
               __HAL_RCC_ADC2_CLK_ENABLE();
       else if (module_id == 3)
               __HAL_RCC_ADC3_CLK_ENABLE();
       else if (module_id == 4)
               __HAL_RCC_ADC4_CLK_ENABLE();

         // VVVV L4 only. extra step after __HAL_RCC_ADC_CLK_ENABLE() VVVV
    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);   // L4 only

         // Enable the ADC module's associated DMA module's clocks
    __HAL_RCC_DMA2_CLK_ENABLE();       // Enable DMA2 clock        F4/F7
    __DMA1_CLK_ENABLE();               // Enable F3_03/L0/F0 DMA clock - F0 has only
                                       // 1 DMA unit, except F09x which has 2 and F3_34 has 3
}

//*****************************************************************************





//*****************************************************************************
//  board_adc_init
//
//         Initialize an ADC module, and configure the overall sampling
//         clock used for the ADC channels on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - PWM Module 0  Generator 0 / 1 / 2 / 3
//           - PWM Module 1  Generator 0 / 1 / 2 / 3
//           - Timer n
//           - Comparator n
//
// Note that on most platforms, the Triggering is _module_ wide, in other
//      words, all channels (ADC 1-16) will be triggered off the same timer/pwm.
//
// STM32-F7 has 3 ADC moodules, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC_SQR1 -> ADC_SQR3.
// ADC results are stored in a single 32-bit register: ADC_DR
// Results are stored in the lower order 16-bits.
// We use DMA to process multiple ADC conversion results, to avoid
// "interrupt per channel" overhead and complexity.
//
// The STM32-F7 Discovery 200-pin device physically supports up to 16 channels,
// of which 5 are wired out to the Nucleo/Arduino pins.
// Two internal sources (Temperature Sensor, Battery Monitor) are also available.
// The other ADCs are wired out to the "Morpho" connector.
// We allow support for all 16 channels.
//
// The STM32-F7 supports three 12-bit ADCs, with up to 16 Channels.
//*****************************************************************************

int  board_adc_init (int adc_module_id, uint32_t clock_rate,  int trigger_type,
                     int flags)
{
    ADC_TRIGGER_BLK  *trigblkp;
    int              rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (_g_adc_clocks_on == 0)
       {     // turn on the clocks for ADC module  (There are 3 on STM32 F7)
         __HAL_RCC_ADC3_CLK_ENABLE();            // Enable ADC 3 Periph clock
         __HAL_RCC_DMA2_CLK_ENABLE();            // Enable DMA2 clock
         _g_adc_clocks_on = 1;                   // denote clocks are now on

             //-----------------------------------------------------------------
             // This is the opportune time to do initial config of the ADC and
             // DMA modules. Note that we will later have to manuually override
             // the initial number of ADC channels (set SQR1 xxxx value).
             //
             // If calibration is required, call that out as a separate subrtn.
             //-----------------------------------------------------------------

// ??? !!!  WVD   in future, create a pointer to the handle from an array (ala SPI routines) with ADC3 as default  !!! ???

         _g_AdcHandle_Mod_3.Instance                   = ADC3; // single module: ADC1
         _g_AdcHandle_Mod_3.Init.NbrOfConversion       = 1;    // - this gets modified -
         _g_AdcHandle_Mod_3.Init.DMAContinuousRequests = ENABLE;             // Use DMA
         _g_AdcHandle_Mod_3.Init.Resolution            = ADC_RESOLUTION_12B; // 12 bit resol
         _g_AdcHandle_Mod_3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

         if (trigger_type == ADC_TRIGGER_USER_APP)
            {     //-----------------------------------------------------------
                  // no external triggers - use SW trigger calls from User App
                  //-----------------------------------------------------------
              _g_AdcHandle_Mod_3.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
              _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
            }
          else
            {      //-------------------------------------------------------------
                   // loop thru trigger table, find entry, and save needed flags.
                   //
                   // We do not turn on all the trigger related TIM flags
                   // until adc_enable() is requested.
                   // We also require the User App issue a timer_ADC_Trigger_Start()
                   // at some point, to cause the ADC to start sampling.
                   //-------------------------------------------------------------
              trigblkp = (ADC_TRIGGER_BLK*) &_g_adc_trigger_config_table[0];
              while (1)
                { if (trigblkp->trigger_user_api_id == -1)
                     return (ERR_ADC_UNSUPPORTED_TRIGGER_TYPE);  // end of table and no match
                  if (trigblkp->trigger_user_api_id == trigger_type)
                     {     // setup _g_AdcHandle.Init with proper Trigger flags
                       _g_AdcHandle_Mod_3.Init.ExternalTrigConv = trigblkp->trigger_adc_extmask;
                       if (flags & ADC_TRIGGER_FALLING)
                          _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
                          else if (flags & ADC_TRIGGER_RISEFALL)
                                  _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
                                  else _g_AdcHandle_Mod_3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
                           // save rest of parms to tb used at adc_enable() and timerpwm_enable()
                       _g_trigger_adc_extmask  = trigblkp->trigger_adc_extmask; // ADC EXTSEL
                       _g_trigger_atmr_mmsmask = trigblkp->trigger_tmr_mmsmask; // TIM MMS
                       _g_trigger_auser_api_id = trigger_type;
                           // the User API trigger type is an encoded field
                           // where the low order 4 bits = CCR, and the rest = index
                           // number of associated Timer/PWM
                       if (trigblkp->trigger_tmr_mmsmask != 0)      // a GEXTI GPIO pin = 0
                          _g_trigger_atmrpwm = (trigger_type >> 4); // save Timer/PWM index
                       break;             // bail out of loop
                     }
                  trigblkp++;             // step to nexct entry in array
                }
            }

         _g_AdcHandle_Mod_3.Init.ScanConvMode          = ENABLE;  // scan multiple channels
         _g_AdcHandle_Mod_3.Init.ContinuousConvMode    = DISABLE; // only when trigger
         _g_AdcHandle_Mod_3.Init.DiscontinuousConvMode = DISABLE; // not used
         _g_AdcHandle_Mod_3.Init.NbrOfDiscConversion   = 0;
         _g_AdcHandle_Mod_3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
         _g_AdcHandle_Mod_3.Init.EOCSelection          = DISABLE; // DMA will interrupt
                                                            //   not ADC EOC.
         rc = HAL_ADC_Init (&_g_AdcHandle_Mod_3);
         if (rc != HAL_OK)
            {
                 /* Initialization Error */
              board_error_handler();           // return error code instead  ???
            }

             //-----------------------------------------------------------------
             // Initialize the DMA associated with the ADC module.
             //   ?? Or should we hold off on this until we hit adc_enable() ?
             //-----------------------------------------------------------------
         _g_DmaHandle.Instance         = DMA2_Stream0;  // F7 ADC3 uses DMA2
         _g_DmaHandle.Init.Channel     = DMA_CHANNEL_2; // F7 ADC3 uses DMA Chan 2
         _g_DmaHandle.Init.Direction   = DMA_PERIPH_TO_MEMORY;  // ADC -> RAM

         _g_DmaHandle.Init.PeriphInc   = DMA_PINC_DISABLE;
         _g_DmaHandle.Init.MemInc      = DMA_MINC_ENABLE;       // step buf ptr
         _g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         _g_DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
         _g_DmaHandle.Init.Mode        = DMA_CIRCULAR;     // Treat as circular buf
         _g_DmaHandle.Init.Priority    = DMA_PRIORITY_HIGH;
         _g_DmaHandle.Init.FIFOMode    = DMA_FIFOMODE_DISABLE;
         _g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
         _g_DmaHandle.Init.MemBurst    = DMA_MBURST_SINGLE;
         _g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

         rc = HAL_DMA_Init (&_g_DmaHandle);

            // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&_g_AdcHandle_Mod_3, DMA_Handle, _g_DmaHandle);

            //----------------------------------------------------------
            //          Configure the NVIC for DMA2 interrupts
            // Configure NVIC for DMA transfer complete interrupt.
            //----------------------------------------------------------
         HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
         HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
       }

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate will be determined by the trigger source.
//
//         adc_module parm is really just an index number (0-3)
//         that denote which ADC module to use.
//         For the STM32 F7, there is only 3 ADC modules: ADC1, ADC2, ADC3
//         Nearly all of the 16 channels are connected to ADC3, and alternative
//         paths are defined to allow parallel operation with ADC1 or ADC2.
//
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference used        (FUTURE_UPGRAGDE)
//


//         STM32 F7 supports a max of 16 ADC channels and 3 ADC modules.
//         It has only 1 main sequencer.
//                    ^^^^^^^^^^^^^^^^^^^^
//                       sequencer per ADC ?  ==> array of sequncer/step values[module]id]


//*****************************************************************************

int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer,  int step_num,
                               int last,  int flags)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    ADC_CHANNEL_BLK  *adcblkp;
    int               orig_step_num;
    int               rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if (channel_num < 0  ||  channel_num > 15)
       return (ERR_ADC_CHANNEL_NUM_OUT_OF_RANGE);

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

          //-----------------------------------------------------
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of 16 entries
          // that contains pin # and GPIO base index, then use  _g_gpio_base[]
          //----------------------------------------------------
    adcblkp = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num];
         // first configure it as a straight GPIO Input, to clear everything out
    GPIO_InitStruct.Pin   = adcblkp->chan_gpio_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (adcblkp->chan_gpio_port, &GPIO_InitStruct);
         // then configure it over to an Analog Input
    GPIO_InitStruct.Pin  = adcblkp->chan_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (adcblkp->chan_gpio_port, &GPIO_InitStruct);

          //----------------------------------------------------------------
          // setup the step within the sequencer to use
          //----------------------------------------------------------------
    orig_step_num = step_num;          // save the step number passed on entry
    if (step_num == ADC_AUTO_STEP)
       step_num = _g_step_num;         // use our internally managed step #

       //-----------------------------------------------------------------
       // Configure the associated ADC channel     (as a regular channel)
       //
       // Note: Based on the intterupts (IT) that occur after each number
       //       "uhADCxConvertedValue"  ADC conversions (IT by DMA end
       //       of transfer), select sampling time and ADC clock with sufficient
       //       duration as to not create an overhead situation in IRQHandler.
       //
       // Note that "Rank" is really sequence id - the sequence in which
       // these will fire, starting at one, and going up.
       //-----------------------------------------------------------------
    _g_ChanConfig.Channel      = adcblkp->chan_adc_id;    // set ADC channel #
    _g_ChanConfig.Rank         = step_num;            // set step # in sequencer
    _g_ChanConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    _g_ChanConfig.Offset       = 0;

    rc = HAL_ADC_ConfigChannel (&_g_AdcHandle_Mod_3, &_g_ChanConfig);
    if (rc != HAL_OK)
       {
            /* Channel Configuration Error */
         board_error_handler();           // return error code instead  ???
       }

          // save the step number that was used for this channel
    _g_adc_step_map [channel_num] = (char) step_num;

    if (orig_step_num == ADC_AUTO_STEP)
       _g_step_num++;          // inc to next step slot in the sequencer

          // track how many channels have been configured for use
    _g_active_channels++;

    return (0);                // denote success
}


//*****************************************************************************
//  board_adc_check_conversions_done
//
//          Checks if the ALL the conversions for a sinple ADC are done.
//
//          For groups, the starting channel of the group, as specified as
//          the first channel listed in the board_adc_group_init() channels[]
//          array, is passed.
//*****************************************************************************
int  board_adc_check_conversions_done (int adc_module_id, int sequencer)
{
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    if ( ! _g_DMA_complete)
       return (0);                       // ADC and DMA are still busy

    return (1);                          // All conversions are completed
}


//*****************************************************************************
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    uint32_t   DataEntries;
    int        rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

//  if (sequencer == ADC_AUTO_SEQUENCE)
//     sequencer = _g_sequencer;  // use current auto-managed sequencer

    if ( !  _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    _g_DMA_complete = 0;  // clear I/O flags
    _g_DMA_overrun  = 0;

        //------------------------------------------------------------------
        // Update the "Number of Copnversions" in the ADC block
        // based upon the final number of channels configured.
        // To do this, we must update the ADC_SQR1_L field in SQR1 register
        //------------------------------------------------------------------
    _g_AdcHandle_Mod_3.Instance->SQR1 &= ~(ADC_SQR1_L);         // clear the bits
    _g_AdcHandle_Mod_3.Instance->SQR1 |= ADC_SQR1(_g_active_channels);

        //----------------------------------------------------------------------
        // the following both starts the ADC and apparently auto-initiates
        // the first Conversion IFF SW initiated, else lets trigger do its thing
        //----------------------------------------------------------------------
    DataEntries = _g_active_channels * 1;  // set # entries that DMA should xfer
    rc = HAL_ADC_Start_DMA (&_g_AdcHandle_Mod_3, (uint32_t*) &_g_adc_conv_results,
                            DataEntries);
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         board_error_handler();           // return error code instead  ???
       }
    return (0);                                    // denote success
}


//*****************************************************************************
//  board_adc_disable
//
//          Turn off one or all the sequencers that have been configured.
//
//          Reset everything in preparattioon for a re-configuratioon of
//          channels.
//*****************************************************************************
int  board_adc_disable (int adc_module_id, int sequencer)
{
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

    HAL_ADC_Stop_DMA (&_g_AdcHandle_Mod_3);

    return (0);                                     // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer,
                            uint16_t *channel_results)
{
    uint32_t  adc_module;
    int      i;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

// CAUTION: TI's code will automatically read an entire ARRAY (from FIFO)
//          if a sequence group is involved

//  rc = ADCSequenceDataGet (uint32_t ui32Base, uint32_t ui32SequenceNum,
//                           uint32_t *pui32Buffer)

//  channel_results[0] = HAL_ADC_GetValue (ADC1);   DMA will have already put it in channel_results

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    if (_g_DMA_complete == 0)
       return (0);     // denote 0 results because DMA rupt has not happened yet

    if (_g_DMA_overrun)
       {    // we had an overrun. Discard the results and tell user try again
         _g_DMA_overrun  = 0;  // clear error flag
         _g_DMA_complete = 0;  // reset for new pass
         return (-1);          // denote we had an overrun condition
       }
    for (i = 0;  i < _g_active_channels;  i++)
       {       // copy the internally DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_DMA_complete = 0;         // reset for new pass

    return (_g_active_channels); // pass back number of completed conversions
}


//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a group of ADCs, on a sequenced group
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
    uint32_t   adc_module;
    uint32_t   DataEntries;
    int        rc;

       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

       // Because there is only 1 sequencer, we ignore the sequencer check
//  if (sequencer < 0 || sequencer > ADC_AUTO_SEQUENCE)
//     return (ERR_ADC_SEQUENCER_ID_OUT_OF_RANGE);

//  if (sequencer == ADC_AUTO_SEQUENCE)
//     sequencer = _g_sequencer;      // use current auto-managed sequencer

    if ( ! _g_adc_clocks_on)
       return (ERR_ADC_MODULE_NOT_INITIALIZED);

    _g_DMA_complete = 0;              // clear I/O flags for new pass
    _g_DMA_overrun  = 0;

        // apparently, to start another ADC conversion, you must issue
        // another xxx_Start_IT()  or  xxx_Start_DMA() request !
// DMA was already initialized and setup for circular mode (effectively resets
// after every cycle). So all we want to do now is trigger the ADC Start again.
// Do not re-init everything, it can cause crashes/faults
//  DataEntries = _g_active_channels * 1;   // set # entries that DMA should xfer
//  rc = HAL_ADC_Start_DMA (&_g_AdcHandle, (uint32_t*) &_g_adc_conv_results,
//                          DataEntries);
    rc = HAL_ADC_Start_IT (&_g_AdcHandle_Mod_3); // Issue software start (SW) to ADC and DMA
    if (rc != HAL_OK)
       {
            /* Had a start Conversion Error */
         board_error_handler();            // return error code instead  ???
       }

    return (0);                                    // denote success
}


//*****************************************************************************
//  board_adc_set_callback
//
//          callback_function and callback_parm are used to provide callback
//          when the ADC conversions are complete.
//*****************************************************************************
int  board_adc_set_callback (int adc_module_id,
                             ADC_CB_EVENT_HANDLER callback_function,
                             void *callback_parm)
{
       // Because there is only 1 ADC module, we ignore the adc_module_id check
//  if (adc_module_id < 0  ||  adc_module_id > ADC_AUTO_MODULE)
//     return (ERR_ADC_MODULE_ID_OUT_OF_RANGE);

        //----------------------------------------------------------------
        // Save ADC completion callback parameters.
        //----------------------------------------------------------------
    _g_adc_callback      = callback_function;
    _g_adc_callback_parm = callback_parm;

    return (0);                           // denote success
}


/****************************************************************************
*                             ADC  ISR   Callback
*
*         ADC Conversion complete callback in non-blocking mode.
*         Called from DMA/ADC ISR Handler.
*
*         If running straight interrupts (xxx_IT), this gets called by
*         the HAL ADC library support when the ADC interrupt completes.
*
*         If running with DMA (xxx_DMA), this gets called
*         when the DMA interrupt completes, i.e. this gets
*         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
*
* @param  _g_AdcHandle : _g_AdcHandle handle
* @note   This example shows a simple way to report end of conversion,
*         and you can add your own implementation.
* @retval None
****************************************************************************/
void   HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *_g_AdcHandle)
{
    dma_callback_seen++;                        // DEBUG COUNTER

// ??? !!!  ADD REST OF SUPPORT FOR THIS on 06/26/15 ---- ??? !!!

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (_g_adc_callback != 0L)
       {
          (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                             _g_active_channels, 0);    // Call user handler
       }
}


/************************************************************************
*                              DMA  ADC  ISR
*
* @brief  This handles the DMA interrupt request for the ADC DMA channel.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

void  DMA2_Stream0_IRQHandler (void)
{
    dma_rupt_seen++;                               // DEBUG COUNTER

    _g_DMA_complete = 1;     // set status that ADCs and DMA completed.
                             // Used by adc_Check_All_Complete() logic.
                             // Alternative option = invoke user callback rtn.

// ??? is the following really needed - should I just clear it in here and be done with it ???

    HAL_DMA_IRQHandler (_g_AdcHandle_Mod_3.DMA_Handle);  // Call post rupt cleanup
                                                   //   and reset rupt flags

       //-------------------------------------------------------------
       // If a ADC completion callback has been configured, invoke it
       //-------------------------------------------------------------
    if (_g_adc_callback != 0L)
       {
          (_g_adc_callback) (_g_adc_callback_parm, _g_adc_conv_results,
                             _g_active_channels, 0);    // Call user handler
       }

//  HAL_ADC_Stop_DMA(hadc);  // ??? need - bit is blow up when re-enable ADC_IT
}

/******************************************************************************/
