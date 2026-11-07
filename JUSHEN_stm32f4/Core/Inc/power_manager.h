#ifndef __POWER_MANAGER_H
#define __POWER_MANAGER_H

#include "main.h"
#include "power_types.h"
#include "bms.h"
#include <stdbool.h>

#define POWER_MANAGER_TASK_PERIOD_MS    10

void PowerManager_Init(PowerManager_t *pm);
void PowerManager_Process(PowerManager_t *pm, uint32_t current_tick);

void PowerManager_UpdateFromBMS(PowerManager_t *pm, BMS_HandleTypeDef *hbms);
void PowerManager_UpdateMotorStatus(PowerManager_t *pm, uint8_t motor_id, 
                                     uint16_t voltage, int16_t current, 
                                     bool online, bool overload, bool overtemp);

void PowerManager_SetCommMode(PowerManager_t *pm, PowerCommMode_t mode);
PowerCommMode_t PowerManager_GetCommMode(PowerManager_t *pm);
void PowerManager_SwitchCommMode(PowerManager_t *pm);

PowerSystemState_t PowerManager_GetSystemState(PowerManager_t *pm);
PowerLevel_t PowerManager_GetPowerLevel(PowerManager_t *pm);

bool PowerManager_IsChargeAllowed(PowerManager_t *pm);
bool PowerManager_IsDischargeAllowed(PowerManager_t *pm);
bool PowerManager_IsOperationSafe(PowerManager_t *pm);

bool PowerManager_HasPendingEvent(PowerManager_t *pm);
EventType_t PowerManager_GetPendingEvent(PowerManager_t *pm, uint8_t *data);
void PowerManager_ClearEvent(PowerManager_t *pm);

bool PowerManager_HasDataChanged(PowerManager_t *pm);
void PowerManager_ClearDataChanged(PowerManager_t *pm);

void PowerManager_GetDataPacket(PowerManager_t *pm, PowerDataPacket_t *packet);
void PowerManager_GetBatteryStatus(PowerManager_t *pm, PowerBatteryStatus_t *status);
void PowerManager_GetMotorStatus(PowerManager_t *pm, PowerMotorStatus_t *status);
void PowerManager_GetFaultStatus(PowerManager_t *pm, PowerFaultStatus_t *status);
void PowerManager_GetSystemStatus(PowerManager_t *pm, PowerSystemStatus_t *status);

const char* PowerManager_GetStateString(PowerSystemState_t state);
const char* PowerManager_GetPowerLevelString(PowerLevel_t level);
const char* PowerManager_GetFaultString(FaultType_t fault);
const char* PowerManager_GetEventString(EventType_t event);

void PowerManager_SetThreshold(PowerManager_t *pm, PowerThreshold_t *threshold);
void PowerManager_GetThreshold(PowerManager_t *pm, PowerThreshold_t *threshold);

extern PowerManager_t g_power_manager;

#endif
