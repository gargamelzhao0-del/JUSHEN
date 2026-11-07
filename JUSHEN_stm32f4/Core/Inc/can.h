#ifndef __CAN_H
#define __CAN_H

#include "main.h"

#define CAN_RX_BUFFER_SIZE 64
#define CAN_RESTART_TIMEOUT 100

typedef struct {
    uint32_t StdId;
    uint8_t Data[8];
    uint8_t DLC;
    uint8_t Flags;
} CAN_RxFrameTypeDef;

typedef struct {
    CAN_RxFrameTypeDef frame[CAN_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CAN_RxBufferTypeDef;

typedef struct {
    uint8_t error_warning;
    uint8_t error_passive;
    uint8_t bus_off;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
} CAN_ErrorStatusTypeDef;

extern CAN_RxBufferTypeDef CAN_RxBuffer;
extern CAN_ErrorStatusTypeDef CAN_ErrorStatus;

void CAN_Init(void);
HAL_StatusTypeDef CAN_Send_Polling(uint16_t StdId, uint8_t *Data, uint8_t Length);
uint8_t CAN_GetRxFrame(CAN_RxFrameTypeDef *frame);
uint8_t CAN_IsRxBufferEmpty(void);
uint8_t CAN_GetErrorStatus(CAN_ErrorStatusTypeDef *status);
uint8_t CAN_Recover(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#endif
