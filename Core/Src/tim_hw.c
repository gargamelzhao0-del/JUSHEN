#include "tim_hw.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "log_system.h"
#include <string.h>  /* For memset */

static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static uint32_t s_period = 0;

void TIM_HW_Init(uint32_t freqHz)
{
  /* Enable TIM3 and TIM4 clocks */
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  /* Ensure GPIOC / GPIOD clocks enabled for PWM pins PC6, PC7, PD12 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure PC6, PC7 -> TIM3_CH1 / TIM3_CH2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure PD12 -> TIM4_CH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t timer_clk = pclk * 2U; /* TIMxCLK = PCLKx * 2 when APB prescaler != 1 */

  /* Use a fixed resolution (period+1) of 1000 for 0.1% steps */
  uint32_t target_counts = 1000U;
  uint32_t prescaler = 0U;
  if (freqHz == 0) freqHz = 50;  /* Default 50Hz for oscilloscope trajectory observation */
  prescaler = (timer_clk / (freqHz * target_counts)) - 1U;
  if (prescaler > 0xFFFFU) prescaler = 0xFFFFU;

  /* Initialize TIM3 (CH1, CH2) */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = prescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = target_counts - 1U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    LOG_ERROR("HAL_TIM_PWM_Init failed for TIM3");
    Error_Handler();
  }

  /* Initialize TIM4 (CH1) */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = prescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = target_counts - 1U;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    LOG_ERROR("HAL_TIM_PWM_Init failed for TIM4");
    Error_Handler();
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  /* Configure TIM3 channels 1 and 2 */
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    LOG_ERROR("HAL_TIM3 PWM Config CH1 failed");
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    LOG_ERROR("HAL_TIM3 PWM Config CH2 failed");
    Error_Handler();
  }

  /* Configure TIM4 channel 1 */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    LOG_ERROR("HAL_TIM4 PWM Config CH1 failed");
    Error_Handler();
  }

  s_period = htim3.Init.Period; /* shared resolution */
  LOG_INFO("PWM init: TIM3/TIM4 @ %lu Hz, prescaler=%lu, period=%lu",
           freqHz, htim3.Init.Prescaler, htim3.Init.Period);

  /* Small initial duty to make output visible if started */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (s_period+1U) / 10U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (s_period+1U) / 10U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (s_period+1U) / 10U);
}

void TIM_HW_Start(void)
{
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) LOG_ERROR("HAL_TIM3 PWM_Start CH1 failed");
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) LOG_ERROR("HAL_TIM3 PWM_Start CH2 failed");
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) LOG_ERROR("HAL_TIM4 PWM_Start CH1 failed");

  LOG_INFO("PWM Output Started: TIM3(PC6/PC7) TIM4(PD12)");
}

void TIM_HW_SetDuty(uint8_t channel, uint8_t percent)
{
  if (percent > 100U) percent = 100U;
  uint32_t cmp = (s_period + 1U) * percent / 100U;
  switch (channel) {
    case 0: /* TIM3 CH1 -> PC6 */ __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, cmp); break;
    case 1: /* TIM3 CH2 -> PC7 */ __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmp); break;
    case 2: /* TIM4 CH1 -> PD12 */ __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, cmp); break;
    default: return;
  }
}

void TIM_HW_SetCompare(uint8_t channel, uint16_t compare)
{
  if (compare > 1000U) compare = 1000U;
  switch (channel) {
    case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare); break;
    case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare); break;
    case 2: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare); break;
    default: return;
  }
}

/* ============================================================================
 * PWM输入捕获功能实现 (PC6=TIM3_CH1, PC7=TIM3_CH2)
 * ============================================================================ */

/* 定时器句柄（用于PWM输入捕获） */
static TIM_HandleTypeDef htim3_capture;

/* PWM捕获数据缓存（单通道） */
typedef struct {
    uint32_t last_capture;   /* 上次捕获值（上升沿） */
    uint32_t period_ticks;   /* 周期（定时器ticks） */
    uint32_t pulse_ticks;    /* 高电平时间（定时器ticks） */
    uint8_t  edge_state;     /* 边沿状态：0=等待上升沿, 1=等待下降沿 */
    uint8_t  is_valid;       /* 数据有效标志 */
    uint32_t last_update_ms; /* 最后更新时间（用于超时检测） */
} PWM_Capture_Internal_t;

static PWM_Capture_Internal_t g_pwm_ch1;  /* PC6数据 */
static PWM_Capture_Internal_t g_pwm_ch2;  /* PC7数据 */
static uint32_t g_timer_clock_hz = 0;     /* 定时器时钟频率 */

/**
 * @brief  初始化PWM输入捕获 (PC6=TIM3_CH1, PC7=TIM3_CH2)
 * @note   使用独立输入捕获模式，分别测量两个PWM信号的周期和占空比
 */
void TIM_PWM_Capture_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 配置GPIO：PC6和PC7为TIM3复用功能（输入模式） */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;  /* 下拉，避免浮空 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 计算定时器时钟频率 */
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    g_timer_clock_hz = pclk * 2U;  /* TIM3在APB1，时钟 = PCLK1 * 2 */

    /* 配置TIM3基本参数（最大测量周期约65ms @ 200MHz时钟） */
    htim3_capture.Instance = TIM3;
    htim3_capture.Init.Prescaler = 199;  /* 200MHz / 200 = 1MHz (1us分辨率) */
    htim3_capture.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3_capture.Init.Period = 0xFFFF;  /* 16位定时器最大值 */
    htim3_capture.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3_capture.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&htim3_capture) != HAL_OK) {
        LOG_ERROR("TIM3 Input Capture Init failed");
        Error_Handler();
    }

    /* ===== 配置CH1 (PC6) - 独立输入捕获模式 ===== */
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* CH1配置：捕获上升沿和下降沿（通过极性切换） */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;  /* 初始为上升沿触发 */
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;        /* IC1映射到TI1 */
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 5;  /* 滤波，减少噪声 */

    if (HAL_TIM_IC_ConfigChannel(&htim3_capture, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        LOG_ERROR("TIM3 CH1 IC Config failed");
        Error_Handler();
    }

    /* ===== 配置CH2 (PC7) - 独立输入捕获模式 ===== */
    /* CH2配置：捕获上升沿和下降沿（通过极性切换） */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;  /* 初始为上升沿触发 */
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;        /* IC2映射到TI2 */
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 5;

    if (HAL_TIM_IC_ConfigChannel(&htim3_capture, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        LOG_ERROR("TIM3 CH2 IC Config failed");
        Error_Handler();
    }

    /* 配置中断优先级 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* 初始化捕获数据 */
    memset(&g_pwm_ch1, 0, sizeof(g_pwm_ch1));
    memset(&g_pwm_ch2, 0, sizeof(g_pwm_ch2));

    LOG_INFO("PWM Capture Init: TIM3_CH1(PC6) and TIM3_CH2(PC7) independent capture mode");
    LOG_INFO("PWM Capture: Timer clock=%lu Hz, Resolution=1us", g_timer_clock_hz);
}

/**
 * @brief  启动PWM输入捕获
 */
void TIM_PWM_Capture_Start(void)
{
    /* 启动CH1和CH2的输入捕获+中断 */
    if (HAL_TIM_IC_Start_IT(&htim3_capture, TIM_CHANNEL_1) != HAL_OK) {
        LOG_ERROR("TIM3 CH1 IC Start failed");
    }
    if (HAL_TIM_IC_Start_IT(&htim3_capture, TIM_CHANNEL_2) != HAL_OK) {
        LOG_ERROR("TIM3 CH2 IC Start failed");
    }

    LOG_INFO("PWM Capture Started - Monitoring PC6(CH1) and PC7(CH2)");
}

/**
 * @brief  读取PC6的PWM信号参数 (TIM3_CH1)
 * @param  data: 输出PWM数据结构
 */
void TIM_PWM_Capture_GetData_CH1(PWM_Capture_t *data)
{
    if (data == NULL) return;

    /* 检查数据超时（500ms无更新） */
    uint32_t now = HAL_GetTick();
    if ((now - g_pwm_ch1.last_update_ms) > 500) {
        g_pwm_ch1.is_valid = 0;
    }

    data->is_valid = g_pwm_ch1.is_valid;
    if (!data->is_valid) {
        data->frequency_hz = 0;
        data->duty_cycle = 0;
        data->period_us = 0;
        data->pulse_us = 0;
        return;
    }

    /* 计算频率和占空比 */
    uint32_t timer_freq = g_timer_clock_hz / (htim3_capture.Init.Prescaler + 1);
    data->period_us = (g_pwm_ch1.period_ticks * 1000000UL) / timer_freq;
    data->pulse_us  = (g_pwm_ch1.pulse_ticks * 1000000UL) / timer_freq;

    if (data->period_us > 0) {
        data->frequency_hz = 1000000UL / data->period_us;
        data->duty_cycle = (g_pwm_ch1.pulse_ticks * 10000UL) / g_pwm_ch1.period_ticks;
    } else {
        data->frequency_hz = 0;
        data->duty_cycle = 0;
    }
}

/**
 * @brief  读取PC7的PWM信号参数 (TIM3_CH2)
 * @param  data: 输出PWM数据结构
 */
void TIM_PWM_Capture_GetData_CH2(PWM_Capture_t *data)
{
    if (data == NULL) return;

    /* 检查数据超时（500ms无更新） */
    uint32_t now = HAL_GetTick();
    if ((now - g_pwm_ch2.last_update_ms) > 500) {
        g_pwm_ch2.is_valid = 0;
    }

    data->is_valid = g_pwm_ch2.is_valid;
    if (!data->is_valid) {
        data->frequency_hz = 0;
        data->duty_cycle = 0;
        data->period_us = 0;
        data->pulse_us = 0;
        return;
    }

    /* 计算频率和占空比 */
    uint32_t timer_freq = g_timer_clock_hz / (htim3_capture.Init.Prescaler + 1);
    data->period_us = (g_pwm_ch2.period_ticks * 1000000UL) / timer_freq;
    data->pulse_us  = (g_pwm_ch2.pulse_ticks * 1000000UL) / timer_freq;

    if (data->period_us > 0) {
        data->frequency_hz = 1000000UL / data->period_us;
        data->duty_cycle = (g_pwm_ch2.pulse_ticks * 10000UL) / g_pwm_ch2.period_ticks;
    } else {
        data->frequency_hz = 0;
        data->duty_cycle = 0;
    }
}

/**
 * @brief  TIM3输入捕获回调函数（处理PWM测量）
 * @param  htim: TIM句柄
 * @note   独立输入捕获模式：CH1测量PC6，CH2测量PC7
 *         每个通道通过极性切换分别捕获上升沿和下降沿
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    /* ===== CH1中断处理 (PC6) ===== */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (g_pwm_ch1.edge_state == 0) {
            /* 上升沿：记录时间戳，计算周期 */
            if (g_pwm_ch1.last_capture > 0) {
                /* 计算周期（处理定时器溢出） */
                if (capture_value >= g_pwm_ch1.last_capture) {
                    g_pwm_ch1.period_ticks = capture_value - g_pwm_ch1.last_capture;
                } else {
                    g_pwm_ch1.period_ticks = (0xFFFF - g_pwm_ch1.last_capture) + capture_value + 1;
                }
            }
            g_pwm_ch1.last_capture = capture_value;
            g_pwm_ch1.edge_state = 1;  /* 下次捕获下降沿 */

            /* 切换为下降沿触发 */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            /* 下降沿：计算脉宽 */
            if (capture_value >= g_pwm_ch1.last_capture) {
                g_pwm_ch1.pulse_ticks = capture_value - g_pwm_ch1.last_capture;
            } else {
                g_pwm_ch1.pulse_ticks = (0xFFFF - g_pwm_ch1.last_capture) + capture_value + 1;
            }

            /* 数据有效 */
            if (g_pwm_ch1.period_ticks > 0) {
                g_pwm_ch1.is_valid = 1;
                g_pwm_ch1.last_update_ms = HAL_GetTick();
            }

            g_pwm_ch1.edge_state = 0;  /* 下次捕获上升沿 */

            /* 切换为上升沿触发 */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    /* ===== CH2中断处理 (PC7) ===== */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if (g_pwm_ch2.edge_state == 0) {
            /* 上升沿：记录时间戳，计算周期 */
            if (g_pwm_ch2.last_capture > 0) {
                /* 计算周期（处理定时器溢出） */
                if (capture_value >= g_pwm_ch2.last_capture) {
                    g_pwm_ch2.period_ticks = capture_value - g_pwm_ch2.last_capture;
                } else {
                    g_pwm_ch2.period_ticks = (0xFFFF - g_pwm_ch2.last_capture) + capture_value + 1;
                }
            }
            g_pwm_ch2.last_capture = capture_value;
            g_pwm_ch2.edge_state = 1;  /* 下次捕获下降沿 */

            /* 切换为下降沿触发 */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            /* 下降沿：计算脉宽 */
            if (capture_value >= g_pwm_ch2.last_capture) {
                g_pwm_ch2.pulse_ticks = capture_value - g_pwm_ch2.last_capture;
            } else {
                g_pwm_ch2.pulse_ticks = (0xFFFF - g_pwm_ch2.last_capture) + capture_value + 1;
            }

            /* 数据有效 */
            if (g_pwm_ch2.period_ticks > 0) {
                g_pwm_ch2.is_valid = 1;
                g_pwm_ch2.last_update_ms = HAL_GetTick();
            }

            g_pwm_ch2.edge_state = 0;  /* 下次捕获上升沿 */

            /* 切换为上升沿触发 */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

/**
 * @brief  TIM3中断处理函数（需要在stm32h7xx_it.c中调用）
 * @note   将此函数添加到 stm32h7xx_it.c 的 TIM3_IRQHandler 中
 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3_capture);
}
