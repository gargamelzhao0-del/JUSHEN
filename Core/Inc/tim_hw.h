/* Hardware PWM helper (TIM-based) */
#ifndef __TIM_HW_H
#define __TIM_HW_H

#include <stdint.h>

/* PWM输入捕获数据结构 */
typedef struct {
    uint32_t frequency_hz;  /* PWM频率 (Hz) */
    uint32_t duty_cycle;    /* 占空比 (0-10000, 表示0.00%-100.00%) */
    uint32_t period_us;     /* 周期 (微秒) */
    uint32_t pulse_us;      /* 高电平时间 (微秒) */
    uint8_t  is_valid;      /* 数据有效标志 (1=有效, 0=无效/超时) */
} PWM_Capture_t;

/* ===== PWM输出模式 ===== */
/* Initialize hardware PWM; freqHz = desired PWM frequency (e.g., 1000) */
void TIM_HW_Init(uint32_t freqHz);

/* Start hardware PWM outputs (channels configured in Init) */
void TIM_HW_Start(void);

/* Set duty percent [0..100] for channel index (0..2) */
void TIM_HW_SetDuty(uint8_t channel, uint8_t percent);

/* Set PWM compare value directly (0-1000 for 0%-100%) */
void TIM_HW_SetCompare(uint8_t channel, uint16_t compare);

/* ===== PWM输入捕获模式 ===== */
/* 初始化PWM输入捕获 (PC6=TIM3_CH1, PC7=TIM3_CH2) */
void TIM_PWM_Capture_Init(void);

/* 启动PWM输入捕获 */
void TIM_PWM_Capture_Start(void);

/* 读取PC6的PWM信号参数 (TIM3_CH1) */
void TIM_PWM_Capture_GetData_CH1(PWM_Capture_t *data);

/* 读取PC7的PWM信号参数 (TIM3_CH2) */
void TIM_PWM_Capture_GetData_CH2(PWM_Capture_t *data);

#endif /* __TIM_HW_H */
