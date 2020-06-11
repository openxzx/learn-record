/*
 * MCU for STM32L071xx
 */

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_OFF;
    //RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    //RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
    //RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

static void TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;   /* From Master Clock don't div again */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* Timer2 ClockSource select for ITR0 */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ITR0;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* Timer2 for Slave timer */
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter = 0x0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* Starting timer2 */
    HAL_TIM_Base_Start(&htim2);
}

static void TIM21_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    /* Enalbe Timer21 clock */
    __HAL_RCC_TIM21_CLK_ENABLE();

    /* Base parameters options */
    htim21.Instance = TIM21;
    htim21.Init.Prescaler = 16 - 1; /* Microsecond counter */
    htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim21.Init.Period = 0xFFFF;
    htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* ClockSource select Internal Clock */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* Timer21 for Master timer, Enable Master/Slave mode */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* Starting Timer21 */
    HAL_TIM_Base_Start(&htim21);
}

static void TIM22_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;

    __HAL_RCC_TIM22_CLK_ENABLE();

    htim22.Instance = TIM22;
    htim22.Init.Prescaler = 16 - 1; /* Using InterClock 16M, this will div 16 for 1MHz */
    htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim22.Init.Period = 0xFFFF;
    htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim22) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_NVIC_SetPriority(TIM22_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM22_IRQn);
}

void (* MicoseondTimerCallback)(void) = NULL;

int MicrosecondTimerStart(uint32_t pkt_count_us, void (*callback)(void))
{
    uint32_t timer_tick = 0;
    uint32_t timer_us = 0;
    uint32_t diff_us = 0;

    MicoseondTimerCallback = callback;

    timer_us = BoardGetMicroSecond();
    if (pkt_count_us < timer_us) {
        diff_us = 0xFFFFFFFF - timer_us + pkt_count_us;
        if (diff_us > 0x0000FFFF) {
            return FAIL;
        }
    } else {
        diff_us = pkt_count_us - timer_us;
		if (diff_us > 0x0000FFFF) {
            return FAIL;
        }
    }

    timer_tick = diff_us + 1;

    __HAL_TIM_SET_COUNTER(&htim22, 0x0000);

    /* Setting timer reload register, in order to change the period */
    htim22.Instance->ARR = timer_tick;

    /* Starting Timer interrupte */
    HAL_TIM_Base_Start_IT(&htim22);

    return SUCCESS;
}

void TIM22_IRQHandler( void )
{
    if (MicoseondTimerCallback) {
        MicoseondTimerCallback();
    }

    MicoseondTimerCallback = NULL;

    HAL_TIM_IRQHandler(&htim22);
    HAL_TIM_Base_Stop(&htim22);
}
