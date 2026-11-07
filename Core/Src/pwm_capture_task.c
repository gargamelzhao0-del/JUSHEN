/**
 ******************************************************************************
 * @file    pwm_capture_task.c
 * @brief   PWM输入捕获监控任务（用于四足机器人遥控器输入等）
 ******************************************************************************
 */

#include "pwm_capture_task.h"
#include "tim_hw.h"
#include "cmsis_os.h"
#include "log_system.h"
#include <stdio.h>

/* 任务句柄 */
static osThreadId_t pwm_capture_task_handle = NULL;

/* 任务属性 */
static const osThreadAttr_t pwm_capture_task_attributes = {
    .name = "PWMCaptureTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief  PWM捕获监控任务函数
 * @param  argument: 未使用
 * @retval None
 */
static void PWMCaptureTask(void *argument)
{
    PWM_Capture_t ch1_data;  /* PC6数据 */
    PWM_Capture_t ch2_data;  /* PC7数据 */

    LOG_INFO("[PWM Capture Task] Started - monitoring PC6 and PC7");

    /* 等待系统稳定 */
    osDelay(500);

    for (;;)
    {
        /* 读取两个通道的PWM数据 */
        TIM_PWM_Capture_GetData_CH1(&ch1_data);
        TIM_PWM_Capture_GetData_CH2(&ch2_data);

        /* 打印CH1 (PC6) 数据 */
        if (ch1_data.is_valid) {
            LOG_RAW("[CH1/PC6] Freq=%u Hz, Duty=%.2f%%, Period=%u us, Pulse=%u us\r\n",
                   ch1_data.frequency_hz,
                   ch1_data.duty_cycle / 100.0f,
                   ch1_data.period_us,
                   ch1_data.pulse_us);
        } else {
            LOG_RAW("[CH1/PC6] No signal\r\n");
        }

        /* 打印CH2 (PC7) 数据 */
        if (ch2_data.is_valid) {
            LOG_RAW("[CH2/PC7] Freq=%u Hz, Duty=%.2f%%, Period=%u us, Pulse=%u us\r\n",
                   ch2_data.frequency_hz,
                   ch2_data.duty_cycle / 100.0f,
                   ch2_data.period_us,
                   ch2_data.pulse_us);
        } else {
            LOG_RAW("[CH2/PC7] No signal\r\n");
        }

        LOG_RAW("----------------------------------------\r\n");

        /* 每秒更新一次 */
        osDelay(1000);
    }
}

/**
 * @brief  启动PWM捕获监控任务
 * @retval 0=成功, -1=失败
 */
int PWM_Capture_Task_Start(void)
{
    if (pwm_capture_task_handle != NULL) {
        LOG_WARN("[PWM Capture Task] Already running");
        return 0;
    }

    pwm_capture_task_handle = osThreadNew(PWMCaptureTask, NULL, &pwm_capture_task_attributes);

    if (pwm_capture_task_handle == NULL) {
        LOG_ERROR("[PWM Capture Task] Failed to create task");
        return -1;
    }

    LOG_INFO("[PWM Capture Task] Created successfully");
    return 0;
}

/**
 * @brief  停止PWM捕获监控任务
 */
void PWM_Capture_Task_Stop(void)
{
    if (pwm_capture_task_handle != NULL) {
        osThreadTerminate(pwm_capture_task_handle);
        pwm_capture_task_handle = NULL;
        LOG_INFO("[PWM Capture Task] Stopped");
    }
}
