#ifndef __TIM_PWM_H__
#define __TIM_PWM_H__

#include "main.h"

/* 初始化 PWM 子系统（软件 PWM） */
void PWM_Init(void);

/* 启动 PWM 任务（在 FreeRTOS 环境） */
void PWM_StartTask(void);

/* 设置占空比（0..100） */
void PWM_SetDuty(uint8_t channel, uint8_t duty_percent);

#endif /* __TIM_PWM_H__ */
