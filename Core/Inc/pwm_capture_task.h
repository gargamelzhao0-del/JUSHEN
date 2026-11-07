/**
 ******************************************************************************
 * @file    pwm_capture_task.h
 * @brief   PWM输入捕获监控任务头文件
 ******************************************************************************
 */

#ifndef __PWM_CAPTURE_TASK_H
#define __PWM_CAPTURE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  启动PWM捕获监控任务（可选）
 * @note   如果需要实时打印PWM捕获数据，可以调用此函数启动监控任务
 * @retval 0=成功, -1=失败
 */
int PWM_Capture_Task_Start(void);

/**
 * @brief  停止PWM捕获监控任务
 */
void PWM_Capture_Task_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CAPTURE_TASK_H */
