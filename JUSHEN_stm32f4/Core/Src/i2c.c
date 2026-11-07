#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

I2C_RxBufferTypeDef I2C_RxBuffer;

extern I2C_HandleTypeDef hi2c3;

void I2C_Init(void)
{
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }

    I2C_RxBuffer.head = 0;
    I2C_RxBuffer.tail = 0;
    I2C_RxBuffer.count = 0;
}

HAL_StatusTypeDef I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Transmit(&hi2c3, DevAddress, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Receive(&hi2c3, DevAddress, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Mem_Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Write(&hi2c3, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(&hi2c3, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

uint8_t I2C_GetRxData(uint8_t *data)
{
    if(I2C_RxBuffer.count == 0)
    {
        return 0;
    }

    *data = I2C_RxBuffer.data[I2C_RxBuffer.tail];
    I2C_RxBuffer.tail = (I2C_RxBuffer.tail + 1) % I2C_RX_BUFFER_SIZE;
    I2C_RxBuffer.count--;

    return 1;
}

uint8_t I2C_IsRxBufferEmpty(void)
{
    return (I2C_RxBuffer.count == 0) ? 1 : 0;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

/* HAL_I2C_ErrorCallback 在 power_comm_slave.c 中实现 */
