#ifndef __I2C_H
#define __I2C_H

#include "main.h"

#define I2C_RX_BUFFER_SIZE 64

typedef struct {
    uint8_t slave_addr;
    uint8_t reg_addr;
    uint8_t data[256];
    uint16_t length;
    uint8_t is_read;
} I2C_TransferTypeDef;

typedef struct {
    uint8_t data[I2C_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} I2C_RxBufferTypeDef;

extern I2C_RxBufferTypeDef I2C_RxBuffer;

void I2C_Init(void);
HAL_StatusTypeDef I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Mem_Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_GetRxData(uint8_t *data);
uint8_t I2C_IsRxBufferEmpty(void);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif
