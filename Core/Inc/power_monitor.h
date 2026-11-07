#ifndef __POWER_MONITOR_H
#define __POWER_MONITOR_H

#include "power_types.h"
#include "power_comm_master.h"
#include "stm32h7xx_hal.h"

#define POWER_MONITOR_INTERVAL_MS    100
#define POWER_EVENT_QUEUE_SIZE       8

#define SOC_LOW_THRESHOLD           20
#define SOC_CRITICAL_THRESHOLD      10

typedef struct {
    PowerCommMaster_t comm;
    
    PowerBatteryStatus_t battery;
    PowerMotorStatus_t motor;
    PowerFaultStatus_t fault;
    PowerSystemStatus_t system;
    
    PowerSystemState_t system_state;
    PowerLevel_t power_level;
    
    bool f4_online;
    bool bms_online;
    uint8_t motor_online_bitmap;
    
    uint32_t last_update_tick;
    uint32_t last_event_tick;
    uint32_t f4_offline_count;
    
    PowerEventCallback_t event_callback;
    PowerEvent_t pending_events[POWER_EVENT_QUEUE_SIZE];
    uint8_t event_head;
    uint8_t event_tail;
    
    bool init_ok;
} PowerMonitor_t;

void PowerMonitor_Init(PowerMonitor_t *pm, I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

void PowerMonitor_SetEventCallback(PowerMonitor_t *pm, PowerEventCallback_t callback);

void PowerMonitor_Update(PowerMonitor_t *pm);

bool PowerMonitor_HasEvent(PowerMonitor_t *pm);

bool PowerMonitor_GetEvent(PowerMonitor_t *pm, PowerEvent_t *event);

uint8_t PowerMonitor_GetSOC(PowerMonitor_t *pm);

uint8_t PowerMonitor_GetSOH(PowerMonitor_t *pm);

uint16_t PowerMonitor_GetVoltage(PowerMonitor_t *pm);

int16_t PowerMonitor_GetCurrent(PowerMonitor_t *pm);

PowerLevel_t PowerMonitor_GetPowerLevel(PowerMonitor_t *pm);

PowerSystemState_t PowerMonitor_GetSystemState(PowerMonitor_t *pm);

bool PowerMonitor_IsBMSOnline(PowerMonitor_t *pm);

bool PowerMonitor_IsF4Online(PowerMonitor_t *pm);

bool PowerMonitor_IsMotorOnline(PowerMonitor_t *pm, uint8_t motor_id);

bool PowerMonitor_HasFault(PowerMonitor_t *pm);

FaultType_t PowerMonitor_GetFaultType(PowerMonitor_t *pm);

FaultLevel_t PowerMonitor_GetFaultLevel(PowerMonitor_t *pm);

const char* PowerMonitor_GetEventString(EventType_t event);

const char* PowerMonitor_GetStateString(PowerSystemState_t state);

const char* PowerMonitor_GetPowerLevelString(PowerLevel_t level);

#endif
