#include "power_comm_slave.h"
#include "i2c.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;

PowerCommSlave_t g_power_comm;

static PowerCommMode_t g_current_mode = POWER_COMM_I2C;
static PowerManager_t *g_pm_ref = NULL;

static void PrepareBatteryData(PowerManager_t *pm, uint8_t *buffer);
static void PrepareMotorData(PowerManager_t *pm, uint8_t *buffer);
static void PrepareFaultData(PowerManager_t *pm, uint8_t *buffer);
static void PrepareSystemData(PowerManager_t *pm, uint8_t *buffer);
static void PrepareEventData(PowerCommSlave_t *comm, PowerManager_t *pm, uint8_t *buffer);

/* ============== ALERT引脚控制 ============== */
static void AlertPin_SetActive(bool active)
{
    HAL_GPIO_WritePin(POWER_COMM_ALERT_PORT, POWER_COMM_ALERT_PIN,
                      active ? POWER_COMM_ALERT_ACTIVE : GPIO_PIN_RESET);
}

void PowerCommSlave_Init(PowerCommSlave_t *comm, PowerCommMode_t mode)
{
    memset(comm, 0, sizeof(PowerCommSlave_t));
    
    comm->reg_addr = 0;
    comm->transfer_active = false;
    comm->event_notify_pending = false;
    
    g_current_mode = mode;
    
    if (mode == POWER_COMM_I2C) {
        HAL_I2C_DisableListen_IT(&hi2c3);
        
        if (HAL_I2C_EnableListen_IT(&hi2c3) != HAL_OK) {
            printf("[PowerComm] I2C Listen enable failed\r\n");
        } else {
            printf("[PowerComm] I2C Slave mode enabled, address=0x%02X\r\n", 
                   POWER_COMM_I2C_ADDRESS);
        }
    } else {
        HAL_GPIO_WritePin(POWER_COMM_SPI_CS_PORT, POWER_COMM_SPI_CS_PIN, GPIO_PIN_SET);
        printf("[PowerComm] SPI Slave mode enabled\r\n");
    }
    
    comm->init_ok = true;
    printf("[PowerComm] Initialized, mode=%s\r\n", 
           mode == POWER_COMM_I2C ? "I2C" : "SPI");
}

void PowerCommSlave_Process(PowerCommSlave_t *comm, PowerManager_t *pm)
{
    if (!comm->init_ok || pm == NULL) {
        return;
    }
    
    g_pm_ref = pm;
    
    if (pm->event_pending && !comm->event_notify_pending) {
        comm->event_notify_pending = true;
        comm->event_notify_data[0] = (uint8_t)pm->pending_event;
        comm->event_notify_data[1] = pm->event_data;
        comm->event_notify_data[2] = pm->fault.fault_code;
        comm->event_notify_data[3] = pm->fault.fault_level;

        /* ============== 主动拉高ALERT引脚通知上位机 ============== */
        AlertPin_SetActive(true);

        printf("[PowerComm] Event pending: %s (data=%d)\r\n",
               PowerManager_GetEventString(pm->pending_event), pm->event_data);
    }

    if (pm->data_changed) {
        pm->comm_tx_count++;
    }

    /* ============== 异常状态下持续保持ALERT ============== */
    if (!pm->event_pending && !pm->bms_online) {
        AlertPin_SetActive(true);
    }
}

void PowerCommSlave_SetMode(PowerCommSlave_t *comm, PowerCommMode_t mode)
{
    if (g_current_mode == mode) {
        return;
    }
    
    if (g_current_mode == POWER_COMM_I2C) {
        HAL_I2C_DisableListen_IT(&hi2c3);
    }
    
    g_current_mode = mode;
    
    if (mode == POWER_COMM_I2C) {
        if (HAL_I2C_EnableListen_IT(&hi2c3) == HAL_OK) {
            printf("[PowerComm] Switched to I2C mode\r\n");
        }
    } else {
        HAL_GPIO_WritePin(POWER_COMM_SPI_CS_PORT, POWER_COMM_SPI_CS_PIN, GPIO_PIN_SET);
        printf("[PowerComm] Switched to SPI mode\r\n");
    }
}

PowerCommMode_t PowerCommSlave_GetMode(PowerCommSlave_t *comm)
{
    return g_current_mode;
}

void PowerCommSlave_NotifyEvent(PowerCommSlave_t *comm, EventType_t event, uint8_t data)
{
    comm->event_notify_pending = true;
    comm->event_notify_data[0] = (uint8_t)event;
    comm->event_notify_data[1] = data;
}

void PowerCommSlave_NotifyDataChange(PowerCommSlave_t *comm, uint8_t change_bitmap)
{
    comm->event_notify_data[2] = change_bitmap;
}

static void PrepareBatteryData(PowerManager_t *pm, uint8_t *buffer)
{
    memcpy(buffer, &pm->battery, sizeof(PowerBatteryStatus_t));
}

static void PrepareMotorData(PowerManager_t *pm, uint8_t *buffer)
{
    memcpy(buffer, &pm->motor, sizeof(PowerMotorStatus_t));
}

static void PrepareFaultData(PowerManager_t *pm, uint8_t *buffer)
{
    memcpy(buffer, &pm->fault, sizeof(PowerFaultStatus_t));
}

static void PrepareSystemData(PowerManager_t *pm, uint8_t *buffer)
{
    buffer[0] = pm->system.system_state;
    buffer[1] = pm->system.power_level;
    buffer[2] = pm->system.can_status;
    buffer[3] = pm->system.comm_status;
    
    if (pm->event_pending) {
        buffer[3] |= POWER_COMM_EVENT_FLAG;
    }
    if (pm->data_changed) {
        buffer[3] |= POWER_COMM_DATA_CHANGE_FLAG;
    }
}

static void PrepareEventData(PowerCommSlave_t *comm, PowerManager_t *pm, uint8_t *buffer)
{
    buffer[0] = (uint8_t)pm->pending_event;
    buffer[1] = pm->event_data;
    buffer[2] = pm->fault.fault_code;
    buffer[3] = pm->fault.fault_level;
    buffer[4] = pm->change_bitmap;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
}

void PowerCommSlave_HandleRegisterRead(PowerCommSlave_t *comm, PowerManager_t *pm,
                                        uint8_t reg_addr, uint8_t *data, uint16_t *len)
{
    if (pm == NULL) {
        *len = 0;
        return;
    }
    
    comm->rx_count++;
    
    switch (reg_addr) {
        case POWER_COMM_REG_BATTERY:
            PrepareBatteryData(pm, data);
            *len = sizeof(PowerBatteryStatus_t);
            break;
            
        case POWER_COMM_REG_MOTOR:
            PrepareMotorData(pm, data);
            *len = sizeof(PowerMotorStatus_t);
            break;
            
        case POWER_COMM_REG_FAULT:
            PrepareFaultData(pm, data);
            *len = sizeof(PowerFaultStatus_t);
            break;
            
        case POWER_COMM_REG_SYSTEM:
            PrepareSystemData(pm, data);
            *len = sizeof(PowerSystemStatus_t);
            break;
            
        case POWER_COMM_REG_VERSION:
            data[0] = (POWER_DATA_VERSION >> 8) & 0xFF;
            data[1] = POWER_DATA_VERSION & 0xFF;
            data[2] = g_current_mode;
            data[3] = 0;
            *len = 4;
            break;
            
        case POWER_COMM_REG_HEARTBEAT:
            data[0] = pm->bms_online ? 0x01 : 0x00;
            data[1] = pm->motor.motor_online;
            data[2] = pm->system.system_state;
            data[3] = pm->battery.soc;
            *len = 4;
            break;
            
        case POWER_COMM_REG_COMM_MODE:
            data[0] = g_current_mode;
            data[1] = comm->tx_count & 0xFF;
            data[2] = (comm->tx_count >> 8) & 0xFF;
            data[3] = comm->rx_count & 0xFF;
            data[4] = (comm->rx_count >> 8) & 0xFF;
            *len = 5;
            break;
            
        case POWER_COMM_REG_EVENT:
            PrepareEventData(comm, pm, data);
            *len = 8;
            if (pm->event_pending) {
                PowerManager_ClearEvent(pm);
                comm->event_notify_pending = false;
                /* 事件被上位机读取后，如果无更多事件则清除ALERT */
                if (!pm->event_pending) {
                    AlertPin_SetActive(false);
                }
            }
            break;
            
        case POWER_COMM_REG_ALL:
            {
                uint16_t offset = 0;
                PrepareBatteryData(pm, &data[offset]);
                offset += sizeof(PowerBatteryStatus_t);
                PrepareMotorData(pm, &data[offset]);
                offset += sizeof(PowerMotorStatus_t);
                PrepareFaultData(pm, &data[offset]);
                offset += sizeof(PowerFaultStatus_t);
                PrepareSystemData(pm, &data[offset]);
                offset += sizeof(PowerSystemStatus_t);
                *len = offset;
            }
            break;
            
        default:
            *len = 0;
            break;
    }
    
    comm->tx_count++;
}

void PowerCommSlave_HandleRegisterWrite(PowerCommSlave_t *comm, PowerManager_t *pm,
                                         uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    comm->rx_count++;
    
    switch (reg_addr) {
        case POWER_COMM_REG_COMM_MODE:
            if (len >= 1) {
                PowerCommMode_t new_mode = (PowerCommMode_t)data[0];
                if (new_mode != g_current_mode) {
                    PowerCommSlave_SetMode(comm, new_mode);
                    PowerManager_SetCommMode(pm, new_mode);
                }
            }
            break;
            
        case POWER_COMM_REG_HEARTBEAT:
            PowerManager_ClearDataChanged(pm);
            break;
            
        default:
            break;
    }
}

void PowerCommSlave_I2C_SlaveRxCallback(uint8_t *data, uint16_t len)
{
    if (len == 0) {
        return;
    }
    
    g_power_comm.reg_addr = data[0];
    
    if (len > 1) {
        PowerCommSlave_HandleRegisterWrite(&g_power_comm, g_pm_ref, 
                                           g_power_comm.reg_addr, 
                                           &data[1], len - 1);
    }
}

void PowerCommSlave_I2C_SlaveTxCallback(uint8_t *data, uint16_t max_len, uint16_t *actual_len)
{
    PowerCommSlave_HandleRegisterRead(&g_power_comm, g_pm_ref,
                                      g_power_comm.reg_addr, data, actual_len);
}

void PowerCommSlave_SPI_ProcessTransfer(uint8_t *rx_data, uint8_t *tx_data, uint16_t len)
{
    if (len == 0) {
        return;
    }
    
    uint8_t reg_addr = rx_data[0];
    uint16_t data_len = 0;
    
    if (len > 1) {
        PowerCommSlave_HandleRegisterWrite(&g_power_comm, g_pm_ref,
                                           reg_addr, &rx_data[1], len - 1);
    } else {
        PowerCommSlave_HandleRegisterRead(&g_power_comm, g_pm_ref,
                                          reg_addr, tx_data, &data_len);
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (hi2c == &hi2c3) {
        if (TransferDirection == I2C_DIRECTION_RECEIVE) {
            HAL_I2C_Slave_Seq_Transmit_IT(&hi2c3, g_power_comm.tx_buffer, 
                                          sizeof(g_power_comm.tx_buffer),
                                          I2C_FIRST_AND_LAST_FRAME);
        } else {
            HAL_I2C_Slave_Seq_Receive_IT(&hi2c3, g_power_comm.rx_buffer,
                                         sizeof(g_power_comm.rx_buffer),
                                         I2C_FIRST_AND_LAST_FRAME);
        }
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c3) {
        if (g_power_comm.rx_buffer[0] != 0) {
            g_power_comm.reg_addr = g_power_comm.rx_buffer[0];
            
            uint16_t len = 0;
            PowerCommSlave_HandleRegisterRead(&g_power_comm, g_pm_ref,
                                              g_power_comm.reg_addr,
                                              g_power_comm.tx_buffer, &len);
        }
        
        HAL_I2C_EnableListen_IT(&hi2c3);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c3) {
        g_power_comm.tx_count++;
        HAL_I2C_EnableListen_IT(&hi2c3);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c3) {
        g_power_comm.rx_count++;
        
        if (g_power_comm.rx_len > 0) {
            g_power_comm.reg_addr = g_power_comm.rx_buffer[0];
            
            if (g_power_comm.rx_len > 1) {
                PowerCommSlave_HandleRegisterWrite(&g_power_comm, g_pm_ref,
                                                   g_power_comm.reg_addr,
                                                   &g_power_comm.rx_buffer[1],
                                                   g_power_comm.rx_len - 1);
            }
        }
        
        HAL_I2C_EnableListen_IT(&hi2c3);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c3) {
        g_power_comm.error_count++;
        HAL_I2C_DeInit(&hi2c3);
        HAL_I2C_Init(&hi2c3);
        HAL_I2C_EnableListen_IT(&hi2c3);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        PowerCommSlave_SPI_ProcessTransfer(g_power_comm.rx_buffer,
                                           g_power_comm.tx_buffer,
                                           sizeof(g_power_comm.rx_buffer));
        
        HAL_SPI_TransmitReceive_IT(&hspi1, g_power_comm.tx_buffer,
                                   g_power_comm.rx_buffer,
                                   sizeof(g_power_comm.tx_buffer));
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        g_power_comm.error_count++;
        HAL_SPI_DeInit(&hspi1);
        HAL_SPI_Init(&hspi1);
        HAL_SPI_TransmitReceive_IT(&hspi1, g_power_comm.tx_buffer,
                                   g_power_comm.rx_buffer,
                                   sizeof(g_power_comm.tx_buffer));
    }
}
