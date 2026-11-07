#ifndef __SPI_H
#define __SPI_H

#include "main.h"

#define SPI_RX_BUFFER_SIZE 64
#define SPI_CS_PORT GPIOB
#define SPI_CS_PIN GPIO_PIN_1

typedef struct {
    uint8_t data[SPI_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} SPI_RxBufferTypeDef;

extern SPI_RxBufferTypeDef SPI_RxBuffer;

void SPI_Init(void);
void SPI_CS_Low(void);
void SPI_CS_High(void);
HAL_StatusTypeDef SPI_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
uint8_t SPI_GetRxData(uint8_t *data);
uint8_t SPI_IsRxBufferEmpty(void);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#endif
