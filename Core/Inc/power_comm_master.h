#ifndef __POWER_COMM_MASTER_H
#define __POWER_COMM_MASTER_H

#include "power_types.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"

#define POWER_SLAVE_I2C_ADDR        0x40
#define POWER_COMM_TIMEOUT_MS       100
#define POWER_COMM_RETRY_COUNT      3

#define POWER_REG_BATTERY           0x00
#define POWER_REG_MOTOR             0x10
#define POWER_REG_FAULT             0x20
#define POWER_REG_SYSTEM            0x30
#define POWER_REG_VERSION           0x40
#define POWER_REG_HEARTBEAT         0x50
#define POWER_REG_COMM_MODE         0x60
#define POWER_REG_EVENT             0x80
#define POWER_REG_ALL               0xF0

typedef struct {
    I2C_HandleTypeDef *hi2c;
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    
    PowerCommMode_t comm_mode;
    bool slave_online;
    uint8_t offline_count;
    
    uint16_t tx_count;
    uint16_t rx_count;
    uint16_t error_count;
    
    uint8_t rx_buffer[POWER_TOTAL_DATA_SIZE + 16];
    uint8_t tx_buffer[16];
    
    bool init_ok;
} PowerCommMaster_t;

void PowerCommMaster_Init(PowerCommMaster_t *comm, I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool PowerCommMaster_ReadBattery(PowerCommMaster_t *comm, PowerBatteryStatus_t *battery);

bool PowerCommMaster_ReadMotor(PowerCommMaster_t *comm, PowerMotorStatus_t *motor);

bool PowerCommMaster_ReadFault(PowerCommMaster_t *comm, PowerFaultStatus_t *fault);

bool PowerCommMaster_ReadSystem(PowerCommMaster_t *comm, PowerSystemStatus_t *system);

bool PowerCommMaster_ReadAll(PowerCommMaster_t *comm, PowerBatteryStatus_t *battery, PowerMotorStatus_t *motor, PowerFaultStatus_t *fault, PowerSystemStatus_t *system);

bool PowerCommMaster_ReadEvent(PowerCommMaster_t *comm, EventType_t *event_type, uint8_t *event_data);

bool PowerCommMaster_ReadHeartbeat(PowerCommMaster_t *comm, uint8_t *bms_online, uint8_t *motor_online, uint8_t *system_state, uint8_t *soc);

bool PowerCommMaster_SwitchCommMode(PowerCommMaster_t *comm, PowerCommMode_t new_mode);

bool PowerCommMaster_CheckSlaveOnline(PowerCommMaster_t *comm);

uint16_t PowerCommMaster_GetVersion(PowerCommMaster_t *comm);

#endif
