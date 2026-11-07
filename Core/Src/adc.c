#include "main.h"
#include "adc.h"

/* ADC采样结果存储数组（4个通道） */
volatile uint16_t adc_values[4] = {0};

void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;  /* ✅ 降低时钟分频，避免过快 */
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;  /* 使能扫描模式 */
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;  /* 序列转换结束标志 */
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;  /* 使能连续转换模式 */
    hadc1.Init.NbrOfConversion = 4;  /* 4个转换通道 */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;  /* DMA循环模式 */
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* ✅ STM32H7 Boost模式配置：通过寄存器直接设置 */
    /* ADC CR寄存器 bit 8-9: BOOST[1:0] = 11b (Boost mode on) */
    SET_BIT(hadc1.Instance->CR, ADC_CR_BOOST);

    /* 配置通道1: PA3 - ADC1_INP15 */
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置通道2: PA6 - ADC1_INP3 */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_2;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置通道3: PB0 - ADC1_INP9 */
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_3;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置通道4: PB1 - ADC1_INP5 */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_4;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /* 启动ADC DMA传输 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  获取指定通道的ADC原始值
 * @param  channel: ADC通道索引 (0-3)
 * @retval ADC原始值 (0-65535)
 */
uint16_t ADC_GetValue(uint8_t channel)
{
    if (channel < 4)
    {
        return adc_values[channel];
    }
    return 0;
}

/**
 * @brief  获取指定通道的电压值
 * @param  channel: ADC通道索引 (0-3)
 * @retval 电压值 (单位: V)
 */
float ADC_GetVoltage(uint8_t channel)
{
    if (channel < 4)
    {
        /* STM32H7的ADC是16位分辨率 */
        return (float)adc_values[channel] * ADC_VREF_MV / 65536.0f / 1000.0f;
    }
    return 0.0f;
}

/**
 * @brief  ADC MSP初始化函数 - 配置GPIO和DMA
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if(hadc->Instance == ADC1)
    {
        /* 配置ADC时钟源为PLL2P (Per_CK) */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /* 使能ADC1时钟 */
        __HAL_RCC_ADC12_CLK_ENABLE();

        /* 使能GPIO时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* 配置GPIO为模拟输入模式 */
        /* PA3 - ADC1_INP15 */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA6 - ADC1_INP3 */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PB0 - ADC1_INP9 */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PB1 - ADC1_INP5 */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* 配置DMA */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* DMA1_Stream0用于ADC1 */
        hdma_adc1.Instance = DMA1_Stream0;
        hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* 16位 */
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;     /* 16位 */
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        {
            Error_Handler();
        }

        /* 关联DMA到ADC */
        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

        /* V10.0: DMA中断优先级调整 - 降低优先级避免抢占控制循环 */
        /* 优先级(5, 0): 低于控制循环定时器(1, 0)，高于普通任务 */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
}