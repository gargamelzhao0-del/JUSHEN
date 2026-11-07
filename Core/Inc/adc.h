#ifndef ADC_H
#define ADC_H

#include "main.h"
#include "stm32h7xx_hal.h"

/* ADC通道索引定义 */
#define ADC_CH_PA3   0  /* PA3 - ADC1_INP15 */
#define ADC_CH_PA6   1  /* PA6 - ADC1_INP3 */
#define ADC_CH_PB0   2  /* PB0 - ADC1_INP9 */
#define ADC_CH_PB1   3  /* PB1 - ADC1_INP5 */

/* ADC参考电压 (mV) - STM32H7的ADC参考电压通常是3.3V */
#define ADC_VREF_MV  3300

/* ADC采样值数组（4个通道） */
extern volatile uint16_t adc_values[4];

/* 函数声明 */
void MX_ADC1_Init(void);
uint16_t ADC_GetValue(uint8_t channel);
float ADC_GetVoltage(uint8_t channel);

#endif