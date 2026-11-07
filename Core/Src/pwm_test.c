/**
 * @file pwm_test.c
 * @brief PWM轨迹测试 - 用于示波器验证
 * 
 * 功能：将STM32轨迹规划后的实际位置输出为PWM波形
 * 用途：验证轨迹规划算法是否正确实现了平滑控制
 * 
 * 使用方法:
 * 1. 包含头文件: #include "pwm_test.h"
 * 2. 在初始化时调用: PWM_Test_Init() 或 PWM_Test_InitWithFreq(50) 推荐50Hz
 * 3. 在控制循环中调用: PWM_Test_Output(position_array, count)
 * 
 * 示波器接脚位:
 *    - PC6  (TIM3_CH1) -> 电机0 PWM输出
 *    - PC7  (TIM3_CH2) -> 电机1 PWM输出  
 *    - PD12 (TIM4_CH1) -> 电机2 PWM输出
 * 
 * PWM配置:
 * - 推荐使用50Hz PWM频率，便于示波器观察轨迹包络
 * - 占空比 = 角度/360 * 100%
 * - 示波器时基设置：1s/div 或 500ms/div
 */

#include "pwm_test.h"
#include "tim_hw.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "tim_hw.h"
#include <stdio.h>

#ifdef PWM_TEST_ENABLED

static uint8_t g_pwm_test_enabled = 0;
static uint32_t g_pwm_test_freq = PWM_TEST_FREQ_50HZ;

void PWM_Test_Init(void)
{
    PWM_Test_InitWithFreq(PWM_TEST_FREQ_50HZ);
}

void PWM_Test_InitWithFreq(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        freq_hz = PWM_TEST_FREQ_50HZ;
    }
    
    g_pwm_test_freq = freq_hz;
    g_pwm_test_enabled = 1;
    
    TIM_HW_Init(freq_hz);
    TIM_HW_Start();
    
    printf("[PWM Test] Initialized @ %u Hz - Trajectory will output to PWM pins\r\n", freq_hz);
    printf("[PWM Test] Pins: PC6(CH1), PC7(CH2), PD12(CH3)\r\n");
    
    if (freq_hz <= 100) {
        printf("[PWM Test] Low frequency mode - Suitable for oscilloscope trajectory observation\r\n");
        printf("[PWM Test] Oscilloscope settings: DC coupling, 1s/div or 500ms/div\r\n");
    }
}

void PWM_Test_Enable(uint8_t enable)
{
    g_pwm_test_enabled = enable;
    if (enable) {
        printf("[PWM Test] Enabled - PWM output active\r\n");
    } else {
        printf("[PWM Test] Disabled\r\n");
    }
}

uint8_t PWM_Test_IsEnabled(void)
{
    return g_pwm_test_enabled;
}

void PWM_Test_Output(float* positions, uint8_t count)
{
    if (!g_pwm_test_enabled) return;
    if (count == 0) return;

    for (uint8_t i = 0; i < count && i < 3; i++) {
        float angle = positions[i];

        if (angle < 0) angle = 0;
        if (angle > 360) angle = 360;

        uint32_t duty = (uint32_t)(angle * 1000.0f / 360.0f);
        if (duty > 1000) duty = 1000;

        TIM_HW_SetCompare(i, (uint16_t)duty);
    }
}

void PWM_Test_SetDuty(uint8_t motor_id, float angle)
{
    if (!g_pwm_test_enabled) return;

    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;

    uint32_t duty = (uint32_t)(angle * 1000.0f / 360.0f);
    if (duty > 1000) duty = 1000;

    TIM_HW_SetCompare(motor_id, (uint16_t)duty);
}

#else

void PWM_Test_Init(void) {}
void PWM_Test_Enable(uint8_t enable) {}
uint8_t PWM_Test_IsEnabled(void) { return 0; }
void PWM_Test_Output(float* positions, uint8_t count) {}
void PWM_Test_SetDuty(uint8_t motor_id, float angle) {}

#endif
