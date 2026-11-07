#ifndef __POWER_COMM_SLAVE_H
#define __POWER_COMM_SLAVE_H

#include "main.h"
#include "power_types.h"
#include "power_manager.h"
#include <stdbool.h>

#define POWER_COMM_I2C_ADDRESS       0x40
#define POWER_COMM_I2C_ADDRESS_ALT   0x42

#define POWER_COMM_SPI_CS_PORT       GPIOB
#define POWER_COMM_SPI_CS_PIN        GPIO_PIN_1

/* ALERT引脚配置 - 用于主动通知上位机有异常事件 */
#define POWER_COMM_ALERT_PORT        GPIOB
#define POWER_COMM_ALERT_PIN         GPIO_PIN_2
#define POWER_COMM_ALERT_ACTIVE      GPIO_PIN_SET    /* 高电平表示有事件 */

#define POWER_COMM_REG_BATTERY       0x00
#define POWER_COMM_REG_MOTOR         0x10
#define POWER_COMM_REG_FAULT         0x20
#define POWER_COMM_REG_SYSTEM        0x30
#define POWER_COMM_REG_VERSION       0x40
#define POWER_COMM_REG_HEARTBEAT     0x50
#define POWER_COMM_REG_COMM_MODE     0x60
#define POWER_COMM_REG_THRESHOLD     0x70
#define POWER_COMM_REG_EVENT         0x80
#define POWER_COMM_REG_ALL           0xF0

#define POWER_COMM_EVENT_FLAG        0x01
#define POWER_COMM_DATA_CHANGE_FLAG  0x02

typedef struct {
    uint8_t rx_buffer[64];
    uint8_t tx_buffer[64];
    volatile uint8_t rx_len;
    volatile uint8_t tx_len;
    volatile uint8_t reg_addr;
    volatile bool transfer_active;
    
    uint16_t tx_count;
    uint16_t rx_count;
    uint16_t error_count;
    
    bool event_notify_pending;
    uint8_t event_notify_data[4];
    
    bool init_ok;
} PowerCommSlave_t;

void PowerCommSlave_Init(PowerCommSlave_t *comm, PowerCommMode_t mode);
void PowerCommSlave_Process(PowerCommSlave_t *comm, PowerManager_t *pm);

void PowerCommSlave_SetMode(PowerCommSlave_t *comm, PowerCommMode_t mode);
PowerCommMode_t PowerCommSlave_GetMode(PowerCommSlave_t *comm);

void PowerCommSlave_NotifyEvent(PowerCommSlave_t *comm, EventType_t event, uint8_t data);
void PowerCommSlave_NotifyDataChange(PowerCommSlave_t *comm, uint8_t change_bitmap);

void PowerCommSlave_I2C_SlaveRxCallback(uint8_t *data, uint16_t len);
void PowerCommSlave_I2C_SlaveTxCallback(uint8_t *data, uint16_t max_len, uint16_t *actual_len);

void PowerCommSlave_SPI_ProcessTransfer(uint8_t *rx_data, uint8_t *tx_data, uint16_t len);

void PowerCommSlave_HandleRegisterRead(PowerCommSlave_t *comm, PowerManager_t *pm, 
                                        uint8_t reg_addr, uint8_t *data, uint16_t *len);
void PowerCommSlave_HandleRegisterWrite(PowerCommSlave_t *comm, PowerManager_t *pm,
                                         uint8_t reg_addr, uint8_t *data, uint16_t len);

extern PowerCommSlave_t g_power_comm;

#endif
