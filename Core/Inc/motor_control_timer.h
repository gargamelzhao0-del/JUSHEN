/**
 ******************************************************************************
 * @file    motor_control_timer.h
 * @brief   电机控制硬件定时器模块 - 1kHz精确触发
 * @note    使用TIM6作为1ms周期定时器，通过信号量触发电机控制任务
 *          解决Realtime优先级任务饿死其他任务的问题
 ******************************************************************************
 */

#ifndef __MOTOR_CONTROL_TIMER_H
#define __MOTOR_CONTROL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* ============================================================================
 * 配置参数
 * ============================================================================ */

#define MOTOR_CTRL_TIMER_FREQ_HZ    200     /* 定时器频率: 200Hz (5ms周期) - 与控制循环匹配 */
#define MOTOR_CTRL_TIMER_PERIOD_US  5000    /* 定时器周期: 5000us (5ms) */

/* ============================================================================
 * 外部变量声明
 * ============================================================================ */

/**
 * @brief TIM6句柄（用于中断处理）
 * @note   在motor_control_timer.c中定义，在stm32h7xx_it.c中使用
 */
extern TIM_HandleTypeDef htim6;

/* ============================================================================
 * 公共API
 * ============================================================================ */

/**
 * @brief  初始化电机控制定时器（TIM6）
 * @retval 0=成功, -1=失败
 * @note   配置TIM6为1kHz周期中断
 */
int motor_ctrl_timer_init(void);

/**
 * @brief  启动电机控制定时器
 * @retval 0=成功, -1=失败
 */
int motor_ctrl_timer_start(void);

/**
 * @brief  停止电机控制定时器
 * @retval 0=成功, -1=失败
 */
int motor_ctrl_timer_stop(void);

/**
 * @brief  获取定时器信号量句柄
 * @retval 信号量句柄（用于任务等待）
 * @note   电机控制任务应使用此信号量等待定时器触发
 */
osSemaphoreId_t motor_ctrl_timer_get_semaphore(void);

/**
 * @brief  获取定时器统计信息
 * @param  total_ticks: 输出总tick计数
 * @param  missed_ticks: 输出丢失tick计数（任务未及时响应）
 * @param  max_response_us: 输出最大响应时间（微秒）
 */
void motor_ctrl_timer_get_stats(
    uint32_t *total_ticks,
    uint32_t *missed_ticks,
    uint32_t *max_response_us
);

/**
 * @brief  获取定时器调试信息（从中断更新）
 * @param  elapsed_ms: 输出100次中断耗时（毫秒）
 * @param  arr_value: 输出ARR寄存器值
 * @param  psc_value: 输出PSC寄存器值
 * @retval true=有新数据, false=无新数据
 * @note   中断每100次更新一次数据，任务调用此函数读取
 */
bool motor_ctrl_timer_get_debug_info(
    uint32_t *elapsed_ms,
    uint32_t *arr_value,
    uint32_t *psc_value
);

/**
 * @brief  重置统计信息
 */
void motor_ctrl_timer_reset_stats(void);

/**
 * @brief  检查定时器是否正在运行
 * @retval true=运行中, false=已停止
 */
bool motor_ctrl_timer_is_running(void);

/**
 * @brief  获取当前时间戳（微秒）
 * @retval 当前时间戳（32位，会溢出）
 * @note   基于TIM6计数器，用于精确时间测量
 */
uint32_t motor_ctrl_timer_get_us(void);

/* ============================================================================
 * 中断回调（内部使用，由stm32h7xx_it.c调用）
 * ============================================================================ */

/**
 * @brief  TIM6中断回调函数
 * @note   在TIM6_DAC_IRQHandler中调用
 */
void motor_ctrl_timer_irq_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_TIMER_H */
