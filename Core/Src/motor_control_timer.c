/**
 ******************************************************************************
 * @file    motor_control_timer.c
 * @brief   电机控制硬件定时器模块实现
 ******************************************************************************
 */

#include "motor_control_timer.h"
#include "stm32h7xx_hal.h"
#include "log_system.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ============================================================================
 * 私有变量
 * ============================================================================ */

TIM_HandleTypeDef htim6;                  /* TIM6句柄 */
static osSemaphoreId_t motor_ctrl_semaphore;     /* 定时器信号量 */
static volatile bool timer_running = false;      /* 定时器运行标志 */

/* 统计信息 */
static volatile uint32_t stat_total_ticks = 0;   /* 总tick计数 */
static volatile uint32_t stat_missed_ticks = 0;  /* 丢失tick计数 */
static volatile uint32_t stat_max_response_us = 0; /* 最大响应时间 */
static volatile uint32_t last_tick_time_us = 0;  /* 上次tick时间 */

/* 调试信息（中断中更新，任务中读取）*/
static volatile uint32_t debug_irq_elapsed_ms = 0;    /* 100次中断耗时(ms) */
static volatile uint32_t debug_irq_arr = 0;           /* ARR寄存器值 */
static volatile uint32_t debug_irq_psc = 0;           /* PSC寄存器值 */
static volatile bool debug_irq_data_ready = false;    /* 数据就绪标志 */

/* ============================================================================
 * 私有函数声明
 * ============================================================================ */

static void tim6_clock_config(void);
static uint32_t get_current_us(void);

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief  初始化电机控制定时器（TIM6）
 */
int motor_ctrl_timer_init(void)
{
    RCC_ClkInitTypeDef clk_config;
    uint32_t flash_latency;
    uint32_t timer_clock_hz;
    uint32_t prescaler;
    uint32_t period;

    LOG_INFO("[MotorTimer] Initializing TIM6 for 200Hz control loop...");

    /* 1. 创建二值信号量 */
    motor_ctrl_semaphore = osSemaphoreNew(1, 0, NULL);
    if (motor_ctrl_semaphore == NULL) {
        LOG_ERROR("[MotorTimer] Failed to create semaphore!");
        return -1;
    }

    /* 2. 使能TIM6时钟 */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* 3. 获取定时器时钟频率 */
    HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
    timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    /* TIM6在APB1上，如果APB1预分频器不为1，定时器时钟自动x2 */
    if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
        timer_clock_hz *= 2;
    }

    LOG_INFO("[MotorTimer] Timer clock: %u Hz", (unsigned int)timer_clock_hz);
    LOG_INFO("[MotorTimer] APB1 Divider: 0x%X (HCLK_DIV1=0x%X)",
             clk_config.APB1CLKDivider, RCC_HCLK_DIV1);
    LOG_INFO("[MotorTimer] PCLK1 Freq: %u Hz (before x2)",
             (unsigned int)HAL_RCC_GetPCLK1Freq());

    /* 4. 计算预分频器和周期值
     * 目标：200Hz (5ms周期) - 与控制循环匹配
     * 方案：分频到10kHz计数器频率，周期设为50
     * 注意：TIM6是16位定时器，Period最大65535
     */
    prescaler = (timer_clock_hz / 10000) - 1;  /* 分频到10kHz */
    period = 50 - 1;                             /* 50个计数 = 5ms = 200Hz */

    /* 5. 配置TIM6 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = prescaler;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = period;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 改为DISABLE，使ARR立即生效 */

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        LOG_ERROR("[MotorTimer] TIM6 initialization failed!");
        return -1;
    }

    /* CRITICAL: 确保ARR和PSC值正确写入
     * 方案1: 直接写寄存器（ARPE=0时立即生效）
     */
    __HAL_TIM_SET_COUNTER(&htim6, 0);              /* 复位计数器 */
    htim6.Instance->ARR = period;                   /* 直接写ARR寄存器 */
    htim6.Instance->PSC = prescaler;                /* 直接写PSC寄存器 */

    /* 方案2: 生成更新事件以重载PSC（PSC需要更新事件才能生效）*/
    htim6.Instance->EGR = TIM_EGR_UG;              /* 生成更新事件，重载预分频器 */
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE); /* 清除UG产生的更新中断标志，避免误触发 */

    /* 6. 配置NVIC中断优先级 */
    /* 优先级5（较高，但低于FDCAN），子优先级0 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 7. 重置统计信息 */
    motor_ctrl_timer_reset_stats();

    /* 8. 验证配置是否正确写入 */
    uint32_t verify_psc = htim6.Instance->PSC;
    uint32_t verify_arr = htim6.Instance->ARR;

    LOG_INFO("[MotorTimer] TIM6 initialized successfully");
    LOG_INFO("[MotorTimer]   Prescaler: %u (verify: %lu)", (unsigned int)prescaler, verify_psc);
    LOG_INFO("[MotorTimer]   Period: %u (verify: %lu)", (unsigned int)period, verify_arr);
    LOG_INFO("[MotorTimer]   Frequency: 200 Hz (5ms) - 与控制循环匹配");
    LOG_INFO("[MotorTimer]   Timer Clock: %u Hz", (unsigned int)timer_clock_hz);
    LOG_INFO("[MotorTimer]   Counter Freq: %u Hz", (unsigned int)(timer_clock_hz / (prescaler + 1)));
    LOG_INFO("[MotorTimer]   Expected Period: %.3f ms", (float)(period + 1) * 1000.0f / (float)(timer_clock_hz / (prescaler + 1)));

    return 0;
}

/**
 * @brief  启动电机控制定时器
 */
int motor_ctrl_timer_start(void)
{
    if (timer_running) {
        LOG_WARN("[MotorTimer] Timer already running");
        return 0;
    }

    LOG_INFO("[MotorTimer] Starting TIM6...");

    /* 验证寄存器值 BEFORE starting */
    uint32_t psc_before = htim6.Instance->PSC;
    uint32_t arr_before = htim6.Instance->ARR;
    uint32_t cr1_before = htim6.Instance->CR1;
    LOG_INFO("[MotorTimer] Before start: PSC=%lu, ARR=%lu, CR1=0x%lX", psc_before, arr_before, cr1_before);

    /* 启动定时器中断 */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        LOG_ERROR("[MotorTimer] Failed to start TIM6!");
        return -1;
    }

    timer_running = true;
    last_tick_time_us = get_current_us();

    /* 验证寄存器值 AFTER starting */
    uint32_t psc_after = htim6.Instance->PSC;
    uint32_t arr_after = htim6.Instance->ARR;
    uint32_t cr1_after = htim6.Instance->CR1;
    uint32_t cnt_after = htim6.Instance->CNT;
    LOG_INFO("[MotorTimer] TIM6 started successfully");
    LOG_INFO("[MotorTimer] After start: PSC=%lu, ARR=%lu, CR1=0x%lX, CNT=%lu",
             psc_after, arr_after, cr1_after, cnt_after);

    /* 检查是否有变化 */
    if (psc_before != psc_after || arr_before != arr_after) {
        LOG_WARN("[MotorTimer] WARNING: Registers changed during start! PSC: %lu->%lu, ARR: %lu->%lu",
                 psc_before, psc_after, arr_before, arr_after);
    }

    return 0;
}

/**
 * @brief  停止电机控制定时器
 */
int motor_ctrl_timer_stop(void)
{
    if (!timer_running) {
        LOG_WARN("[MotorTimer] Timer not running");
        return 0;
    }

    LOG_INFO("[MotorTimer] Stopping TIM6...");

    /* 停止定时器中断 */
    if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
        LOG_ERROR("[MotorTimer] Failed to stop TIM6!");
        return -1;
    }

    timer_running = false;

    LOG_INFO("[MotorTimer] TIM6 stopped");
    return 0;
}

/**
 * @brief  获取定时器信号量句柄
 */
osSemaphoreId_t motor_ctrl_timer_get_semaphore(void)
{
    return motor_ctrl_semaphore;
}

/**
 * @brief  获取定时器统计信息
 */
void motor_ctrl_timer_get_stats(
    uint32_t *total_ticks,
    uint32_t *missed_ticks,
    uint32_t *max_response_us)
{
    if (total_ticks) {
        *total_ticks = stat_total_ticks;
    }
    if (missed_ticks) {
        *missed_ticks = stat_missed_ticks;
    }
    if (max_response_us) {
        *max_response_us = stat_max_response_us;
    }
}

/**
 * @brief  获取定时器调试信息（从中断更新）
 * @retval true=有新数据, false=无新数据
 */
bool motor_ctrl_timer_get_debug_info(
    uint32_t *elapsed_ms,
    uint32_t *arr_value,
    uint32_t *psc_value)
{
    if (!debug_irq_data_ready) {
        return false;
    }

    if (elapsed_ms) {
        *elapsed_ms = debug_irq_elapsed_ms;
    }
    if (arr_value) {
        *arr_value = debug_irq_arr;
    }
    if (psc_value) {
        *psc_value = debug_irq_psc;
    }

    debug_irq_data_ready = false;  /* 清除标志 */
    return true;
}

/**
 * @brief  重置统计信息
 */
void motor_ctrl_timer_reset_stats(void)
{
    stat_total_ticks = 0;
    stat_missed_ticks = 0;
    stat_max_response_us = 0;
    last_tick_time_us = 0;
}

/**
 * @brief  检查定时器是否正在运行
 */
bool motor_ctrl_timer_is_running(void)
{
    return timer_running;
}

/**
 * @brief  获取当前时间戳（微秒）
 */
uint32_t motor_ctrl_timer_get_us(void)
{
    return get_current_us();
}

/* ============================================================================
 * 中断回调函数
 * ============================================================================ */

/**
 * @brief  TIM6中断回调函数
 * @note   在TIM6_DAC_IRQHandler中调用
 */
void motor_ctrl_timer_irq_callback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 更新统计信息 */
    stat_total_ticks++;

    /* 每100次中断，记录硬件寄存器值（不打印，存到全局变量）*/
    static uint32_t debug_interval_counter = 0;
    static uint32_t last_debug_tick = 0;
    debug_interval_counter++;

    if (debug_interval_counter >= 100) {
        debug_interval_counter = 0;
        uint32_t current_tick = HAL_GetTick();

        /* 存储到全局变量，供任务读取并打印 */
        if (last_debug_tick > 0) {  /* 只检查last_debug_tick，第一次跳过 */
            uint32_t elapsed = current_tick - last_debug_tick;
            if (elapsed > 0) {
                debug_irq_elapsed_ms = elapsed;
                debug_irq_arr = htim6.Instance->ARR;
                debug_irq_psc = htim6.Instance->PSC;
                debug_irq_data_ready = true;  /* 标记数据就绪 */
            }
        }
        last_debug_tick = current_tick;
    }

    /* 检查信号量是否还有余量（检测任务是否未及时响应）*/
    if (osSemaphoreGetCount(motor_ctrl_semaphore) > 0) {
        stat_missed_ticks++;
    }

    /* 释放信号量，唤醒电机控制任务 */
    osSemaphoreRelease(motor_ctrl_semaphore);

    /* 计算响应时间（可选，用于性能监控）*/
    uint32_t current_us = get_current_us();
    if (last_tick_time_us > 0) {
        uint32_t response_us = current_us - last_tick_time_us;
        if (response_us > stat_max_response_us) {
            stat_max_response_us = response_us;
        }
    }
    last_tick_time_us = current_us;

    /* 通知FreeRTOS可能需要任务切换 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  获取当前时间戳（微秒）
 * @note   基于系统tick（1ms精度）
 */
static uint32_t get_current_us(void)
{
    /* 使用HAL_GetTick()获取毫秒数，转换为微秒 */
    /* 注意：这里精度是1ms，如果需要更高精度可以读取TIM6计数器 */
    return HAL_GetTick() * 1000;
}

/**
 * @brief  TIM6时钟配置（如需自定义）
 */
static void tim6_clock_config(void)
{
    /* 预留：如需要额外的时钟配置，在此添加 */
}
