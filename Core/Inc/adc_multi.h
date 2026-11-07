#ifndef ADC_MULTI_H
#define ADC_MULTI_H

#include "stm32h7xx_hal.h"

#define ADC_CHANNELS 4
#define ADC_BUFFER_SIZE ADC_CHANNELS

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void);
void ADC_Start_DMA(uint32_t* pData, uint32_t Length);
void ADC_Stop(void);
uint16_t ADC_GetValue(uint8_t channel);

#endif
