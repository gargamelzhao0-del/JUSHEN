#include "dma.h"
#include "main.h"

/**
 * @brief  DMA初始化函数
 * @note   ADC的DMA配置已经移到adc.c中的HAL_ADC_MspInit函数
 *         此函数保留用于其他外设的DMA初始化
 */
void MX_DMA_Init(void)
{
    /* ADC的DMA配置已经在adc.c的HAL_ADC_MspInit中完成 */
    /* 如果有其他外设需要DMA，在这里配置 */
}