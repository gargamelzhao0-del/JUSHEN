#include "can.h"
#include "main.h"

CAN_RxBufferTypeDef CAN_RxBuffer;
CAN_ErrorStatusTypeDef CAN_ErrorStatus;

static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static volatile uint8_t CAN_Initialized = 0;

void CAN_Init(void)
{
    CAN_FilterTypeDef CAN_FilterConfig;

    CAN_ErrorStatus.error_warning = 0;
    CAN_ErrorStatus.error_passive = 0;
    CAN_ErrorStatus.bus_off = 0;
    CAN_ErrorStatus.tx_error_count = 0;
    CAN_ErrorStatus.rx_error_count = 0;

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 7;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    CAN_FilterConfig.FilterBank = 0;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING) != HAL_OK)
    {
        Error_Handler();
    }

    CAN_RxBuffer.head = 0;
    CAN_RxBuffer.tail = 0;
    CAN_RxBuffer.count = 0;

    TxHeader.ExtId = 0x00;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

    CAN_Initialized = 1;
}

HAL_StatusTypeDef CAN_Send_Polling(uint16_t StdId, uint8_t *Data, uint8_t Length)
{
    uint32_t TxMailbox;
    uint32_t timeout = 0;

    if(!CAN_Initialized)
    {
        return HAL_ERROR;
    }

    if(hcan1.State != HAL_CAN_STATE_READY)
    {
        if(CAN_Recover() != 0)
        {
            return HAL_ERROR;
        }
    }

    TxHeader.StdId = StdId;
    TxHeader.DLC = Length;

    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox) != HAL_OK)
    {
        CAN_ErrorStatus.tx_error_count++;
        return HAL_ERROR;
    }

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        timeout++;
        if(timeout > 0xFFFF)
        {
            CAN_ErrorStatus.tx_error_count++;
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

uint8_t CAN_GetRxFrame(CAN_RxFrameTypeDef *frame)
{
    if(CAN_RxBuffer.count == 0)
    {
        return 0;
    }

    frame->StdId = CAN_RxBuffer.frame[CAN_RxBuffer.tail].StdId;
    frame->DLC = CAN_RxBuffer.frame[CAN_RxBuffer.tail].DLC;
    frame->Flags = CAN_RxBuffer.frame[CAN_RxBuffer.tail].Flags;
    for(uint8_t i = 0; i < 8; i++)
    {
        frame->Data[i] = CAN_RxBuffer.frame[CAN_RxBuffer.tail].Data[i];
    }

    CAN_RxBuffer.tail = (CAN_RxBuffer.tail + 1) % CAN_RX_BUFFER_SIZE;
    CAN_RxBuffer.count--;

    return 1;
}

uint8_t CAN_IsRxBufferEmpty(void)
{
    return (CAN_RxBuffer.count == 0) ? 1 : 0;
}

uint8_t CAN_GetErrorStatus(CAN_ErrorStatusTypeDef *status)
{
    if(hcan1.Instance == NULL)
    {
        return 0;
    }

    uint32_t error_code = hcan1.ErrorCode;

    if(error_code & HAL_CAN_ERROR_EWG)
    {
        CAN_ErrorStatus.error_warning = 1;
    }
    else
    {
        CAN_ErrorStatus.error_warning = 0;
    }

    if(error_code & HAL_CAN_ERROR_EPV)
    {
        CAN_ErrorStatus.error_passive = 1;
    }
    else
    {
        CAN_ErrorStatus.error_passive = 0;
    }

    if(error_code & HAL_CAN_ERROR_BOF)
    {
        CAN_ErrorStatus.bus_off = 1;
    }
    else
    {
        CAN_ErrorStatus.bus_off = 0;
    }

    CAN_ErrorStatus.tx_error_count = ((hcan1.Instance->ESR) >> 24) & 0xFF;
    CAN_ErrorStatus.rx_error_count = ((hcan1.Instance->ESR) >> 16) & 0xFF;

    status->error_warning = CAN_ErrorStatus.error_warning;
    status->error_passive = CAN_ErrorStatus.error_passive;
    status->bus_off = CAN_ErrorStatus.bus_off;
    status->tx_error_count = CAN_ErrorStatus.tx_error_count;
    status->rx_error_count = CAN_ErrorStatus.rx_error_count;

    return 1;
}

uint8_t CAN_Recover(void)
{
    if(hcan1.Instance == NULL)
    {
        return 0;
    }

    HAL_CAN_Stop(&hcan1);

    HAL_CAN_DeInit(&hcan1);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 7;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return 0;
    }

    CAN_FilterTypeDef CAN_FilterConfig;
    CAN_FilterConfig.FilterBank = 0;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
    {
        return 0;
    }

    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return 0;
    }

    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING) != HAL_OK)
    {
        return 0;
    }

    CAN_ErrorStatus.error_warning = 0;
    CAN_ErrorStatus.error_passive = 0;
    CAN_ErrorStatus.bus_off = 0;
    CAN_ErrorStatus.tx_error_count = 0;
    CAN_ErrorStatus.rx_error_count = 0;

    CAN_Initialized = 1;

    return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rx_data[8];

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
    {
        if(CAN_RxBuffer.count < CAN_RX_BUFFER_SIZE)
        {
            CAN_RxBuffer.frame[CAN_RxBuffer.head].StdId = RxHeader.StdId;
            CAN_RxBuffer.frame[CAN_RxBuffer.head].DLC = RxHeader.DLC;
            CAN_RxBuffer.frame[CAN_RxBuffer.head].Flags = 0;

            for(uint8_t i = 0; i < 8; i++)
            {
                CAN_RxBuffer.frame[CAN_RxBuffer.head].Data[i] = rx_data[i];
            }

            CAN_RxBuffer.head = (CAN_RxBuffer.head + 1) % CAN_RX_BUFFER_SIZE;
            CAN_RxBuffer.count++;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error = hcan->ErrorCode;

    if(error & HAL_CAN_ERROR_EWG)
    {
        CAN_ErrorStatus.error_warning = 1;
    }

    if(error & HAL_CAN_ERROR_EPV)
    {
        CAN_ErrorStatus.error_passive = 1;
    }

    if(error & HAL_CAN_ERROR_BOF)
    {
        CAN_ErrorStatus.bus_off = 1;
    }

    if(error & HAL_CAN_ERROR_STF)
    {
        CAN_ErrorStatus.tx_error_count++;
    }

    if(error & HAL_CAN_ERROR_FOR)
    {
        CAN_ErrorStatus.rx_error_count++;
    }

    if(error & HAL_CAN_ERROR_ACK)
    {
        CAN_ErrorStatus.tx_error_count++;
    }

    if(error & HAL_CAN_ERROR_BR)
    {
        CAN_ErrorStatus.tx_error_count++;
    }

    if(error & HAL_CAN_ERROR_BD)
    {
        CAN_ErrorStatus.tx_error_count++;
    }

    if(error & HAL_CAN_ERROR_CRC)
    {
        CAN_ErrorStatus.rx_error_count++;
    }

    if((CAN_ErrorStatus.bus_off == 1) || (CAN_ErrorStatus.error_passive == 1))
    {
        CAN_Recover();
    }

    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);
}
