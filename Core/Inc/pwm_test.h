/**
 * @file pwm_test.h
 * @brief PWM轨迹测试头文件
 * 
 * 功能：将STM32轨迹规划后的实际位置输出为PWM波形
 * 用途：验证轨迹规划算法是否正确实现了平滑控制
 * 
 * 示波器验证方法：
 * 1. 使用50Hz PWM频率（周期20ms），便于观察完整运动轨迹
 * 2. 示波器设置：DC耦合，时基1s/div或500ms/div
 * 3. 可选：RC低通滤波（R=10kΩ, C=10μF）将PWM转为模拟电压
 * 
 * 引脚映射：
 *    - PC6  (TIM3_CH1) -> 电机0 PWM输出
 *    - PC7  (TIM3_CH2) -> 电机1 PWM输出  
 *    - PD12 (TIM4_CH1) -> 电机2 PWM输出
 */

#ifndef __PWM_TEST_H
#define __PWM_TEST_H

#include <stdint.h>
#include <stdbool.h>

#define PWM_TEST_ENABLED  1

#define PWM_TEST_FREQ_50HZ      50      /* 50Hz - 推荐用于示波器观察轨迹包络 */
#define PWM_TEST_FREQ_1KHZ      1000    /* 1kHz - 默认频率 */
#define PWM_TEST_FREQ_25KHZ     25000   /* 25kHz - 高频PWM */

void PWM_Test_Init(void);
void PWM_Test_InitWithFreq(uint32_t freq_hz);
void PWM_Test_Enable(uint8_t enable);
uint8_t PWM_Test_IsEnabled(void);

void PWM_Test_Output(float* positions, uint8_t count);
void PWM_Test_SetDuty(uint8_t motor_id, float angle);

#endif
