#include "spi.h"
#include "stm32f4xx_hal_spi.h"

SPI_RxBufferTypeDef SPI_RxBuffer;

extern SPI_HandleTypeDef hspi1;

void SPI_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI_RxBuffer.head = 0;
    SPI_RxBuffer.tail = 0;
    SPI_RxBuffer.count = 0;
}

void SPI_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
}

void SPI_CS_High(void)
{
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef SPI_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    SPI_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, pData, Size, Timeout);
    SPI_CS_High();
    return status;
}

HAL_StatusTypeDef SPI_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    SPI_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, pData, Size, Timeout);
    SPI_CS_High();
    return status;
}

HAL_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    SPI_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, Timeout);
    SPI_CS_High();
    return status;
}

uint8_t SPI_GetRxData(uint8_t *data)
{
    if(SPI_RxBuffer.count == 0)
    {
        return 0;
    }

    *data = SPI_RxBuffer.data[SPI_RxBuffer.tail];
    SPI_RxBuffer.tail = (SPI_RxBuffer.tail + 1) % SPI_RX_BUFFER_SIZE;
    SPI_RxBuffer.count--;

    return 1;
}

uint8_t SPI_IsRxBufferEmpty(void)
{
    return (SPI_RxBuffer.count == 0) ? 1 : 0;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

/* HAL_SPI_TxRxCpltCallback 和 HAL_SPI_ErrorCallback 在 power_comm_slave.c 中实现 */
