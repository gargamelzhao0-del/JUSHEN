#include "power_monitor.h"
#include <string.h>
#include <stdio.h>

static const char* state_strings[] = {
    "初始化",
    "正常",
    "警告",
    "故障",
    "紧急"
};

static const char* level_strings[] = {
    "严重不足",
    "电量低",
    "电量中等",
    "电量充足",
    "电量充满"
};

static const char* event_strings[] = {
    "无事件",
    "电量低",
    "电量严重不足",
    "一级故障",
    "二级故障",
    "三级故障",
    "过流",
    "过温",
    "电机故障",
    "通信丢失",
    "充满电",
    "电量耗尽",
    "电量下降10%"
};

static void PushEvent(PowerMonitor_t *pm, EventType_t type, uint8_t data, uint8_t fault_code, uint8_t fault_level)
{
    uint8_t next_head = (pm->event_head + 1) % POWER_EVENT_QUEUE_SIZE;
    
    if (next_head == pm->event_tail) {
        return;
    }
    
    PowerEvent_t *evt = &pm->pending_events[pm->event_head];
    evt->event_type = type;
    evt->event_data = data;
    evt->fault_code = fault_code;
    evt->fault_level = fault_level;
    evt->timestamp = HAL_GetTick();
    
    pm->event_head = next_head;
    
    if (pm->event_callback != NULL) {
        pm->event_callback(evt);
    }
}

static void UpdatePowerLevel(PowerMonitor_t *pm)
{
    uint8_t soc = pm->battery.soc;
    
    if (soc <= SOC_CRITICAL_THRESHOLD) {
        pm->power_level = POWER_LEVEL_CRITICAL;
    } else if (soc <= SOC_LOW_THRESHOLD) {
        pm->power_level = POWER_LEVEL_LOW;
    } else if (soc <= 50) {
        pm->power_level = POWER_LEVEL_MEDIUM;
    } else if (soc <= 80) {
        pm->power_level = POWER_LEVEL_HIGH;
    } else {
        pm->power_level = POWER_LEVEL_FULL;
    }
}

static void UpdateSystemState(PowerMonitor_t *pm)
{
    if (!pm->f4_online) {
        pm->system_state = POWER_STATE_FAULT;
    } else if (!pm->bms_online) {
        pm->system_state = POWER_STATE_WARNING;
    } else if (pm->fault.fault_level >= FAULT_LEVEL_3) {
        pm->system_state = POWER_STATE_EMERGENCY;
    } else if (pm->fault.fault_level >= FAULT_LEVEL_2) {
        pm->system_state = POWER_STATE_FAULT;
    } else if (pm->fault.fault_level >= FAULT_LEVEL_1 || pm->power_level == POWER_LEVEL_CRITICAL) {
        pm->system_state = POWER_STATE_WARNING;
    } else {
        pm->system_state = POWER_STATE_NORMAL;
    }
}

static void CheckEvents(PowerMonitor_t *pm)
{
    static uint8_t last_soc = 100;
    static bool was_f4_online = false;
    static bool was_bms_online = false;
    static uint8_t last_fault_level = FAULT_LEVEL_NONE;
    
    if (was_f4_online && !pm->f4_online) {
        PushEvent(pm, EVENT_TYPE_COMM_LOST, 0, FAULT_TYPE_COMM_TIMEOUT, FAULT_LEVEL_2);
    }
    was_f4_online = pm->f4_online;
    
    if (was_bms_online && !pm->bms_online) {
        PushEvent(pm, EVENT_TYPE_COMM_LOST, 1, FAULT_TYPE_COMM_TIMEOUT, FAULT_LEVEL_1);
    }
    was_bms_online = pm->bms_online;
    
    if (pm->battery.soc != last_soc) {
        if (pm->battery.soc <= SOC_CRITICAL_THRESHOLD && last_soc > SOC_CRITICAL_THRESHOLD) {
            PushEvent(pm, EVENT_TYPE_SOC_CRITICAL, pm->battery.soc, 0, 0);
        } else if (pm->battery.soc <= SOC_LOW_THRESHOLD && last_soc > SOC_LOW_THRESHOLD) {
            PushEvent(pm, EVENT_TYPE_SOC_LOW, pm->battery.soc, 0, 0);
        }
        
        last_soc = pm->battery.soc;
    }
    
    if (pm->fault.fault_level != last_fault_level) {
        if (pm->fault.fault_level > FAULT_LEVEL_NONE) {
            PushEvent(pm, (EventType_t)(EVENT_TYPE_FAULT_LEVEL1 + pm->fault.fault_level - 1),
                      pm->fault.fault_code, pm->fault.fault_code, pm->fault.fault_level);
        }
        last_fault_level = pm->fault.fault_level;
    }
}

void PowerMonitor_Init(PowerMonitor_t *pm, I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    memset(pm, 0, sizeof(PowerMonitor_t));
    
    PowerCommMaster_Init(&pm->comm, hi2c, hspi, cs_port, cs_pin);
    
    pm->system_state = POWER_STATE_INIT;
    pm->power_level = POWER_LEVEL_MEDIUM;
    
    pm->battery.soc = 0;
    pm->battery.soh = 100;
    pm->battery.bms_online = 0;
    
    pm->f4_online = false;
    pm->bms_online = false;
    pm->motor_online_bitmap = 0;
    
    pm->event_head = 0;
    pm->event_tail = 0;
    pm->event_callback = NULL;
    
    pm->init_ok = true;
}

void PowerMonitor_SetEventCallback(PowerMonitor_t *pm, PowerEventCallback_t callback)
{
    pm->event_callback = callback;
}

void PowerMonitor_Update(PowerMonitor_t *pm)
{
    if (!pm->init_ok) {
        return;
    }
    
    uint32_t current_tick = HAL_GetTick();
    
    if (current_tick - pm->last_update_tick < POWER_MONITOR_INTERVAL_MS) {
        return;
    }
    pm->last_update_tick = current_tick;
    
    bool f4_was_online = pm->f4_online;
    pm->f4_online = PowerCommMaster_CheckSlaveOnline(&pm->comm);
    
    if (!pm->f4_online) {
        pm->f4_offline_count++;
        if (pm->f4_offline_count >= 3 && f4_was_online) {
            PushEvent(pm, EVENT_TYPE_COMM_LOST, 0, FAULT_TYPE_COMM_TIMEOUT, FAULT_LEVEL_2);
        }
        UpdateSystemState(pm);
        return;
    }
    
    pm->f4_offline_count = 0;
    
    EventType_t event_type;
    uint8_t event_data;
    
    if (PowerCommMaster_ReadEvent(&pm->comm, &event_type, &event_data)) {
        if (event_type != EVENT_TYPE_NONE) {
            PushEvent(pm, event_type, event_data, pm->fault.fault_code, pm->fault.fault_level);
        }
    }
    
    if (PowerCommMaster_ReadAll(&pm->comm, &pm->battery, &pm->motor, &pm->fault, &pm->system)) {
        pm->bms_online = (pm->battery.bms_online != 0);
        pm->motor_online_bitmap = pm->motor.motor_online;
    }
    
    UpdatePowerLevel(pm);
    UpdateSystemState(pm);
    CheckEvents(pm);
}

bool PowerMonitor_HasEvent(PowerMonitor_t *pm)
{
    return pm->event_head != pm->event_tail;
}

bool PowerMonitor_GetEvent(PowerMonitor_t *pm, PowerEvent_t *event)
{
    if (pm->event_head == pm->event_tail) {
        return false;
    }
    
    if (event != NULL) {
        *event = pm->pending_events[pm->event_tail];
    }
    
    pm->event_tail = (pm->event_tail + 1) % POWER_EVENT_QUEUE_SIZE;
    return true;
}

uint8_t PowerMonitor_GetSOC(PowerMonitor_t *pm)
{
    return pm->battery.soc;
}

uint8_t PowerMonitor_GetSOH(PowerMonitor_t *pm)
{
    return pm->battery.soh;
}

uint16_t PowerMonitor_GetVoltage(PowerMonitor_t *pm)
{
    return pm->battery.total_voltage;
}

int16_t PowerMonitor_GetCurrent(PowerMonitor_t *pm)
{
    return pm->battery.total_current;
}

PowerLevel_t PowerMonitor_GetPowerLevel(PowerMonitor_t *pm)
{
    return pm->power_level;
}

PowerSystemState_t PowerMonitor_GetSystemState(PowerMonitor_t *pm)
{
    return pm->system_state;
}

bool PowerMonitor_IsBMSOnline(PowerMonitor_t *pm)
{
    return pm->bms_online;
}

bool PowerMonitor_IsF4Online(PowerMonitor_t *pm)
{
    return pm->f4_online;
}

bool PowerMonitor_IsMotorOnline(PowerMonitor_t *pm, uint8_t motor_id)
{
    if (motor_id >= 4) {
        return false;
    }
    return (pm->motor_online_bitmap & (1 << motor_id)) != 0;
}

bool PowerMonitor_HasFault(PowerMonitor_t *pm)
{
    return pm->fault.fault_level > FAULT_LEVEL_NONE;
}

FaultType_t PowerMonitor_GetFaultType(PowerMonitor_t *pm)
{
    return (FaultType_t)pm->fault.fault_code;
}

FaultLevel_t PowerMonitor_GetFaultLevel(PowerMonitor_t *pm)
{
    return (FaultLevel_t)pm->fault.fault_level;
}

const char* PowerMonitor_GetEventString(EventType_t event)
{
    if (event <= EVENT_TYPE_SOC_DROP_10) {
        return event_strings[event];
    }
    return "未知";
}

const char* PowerMonitor_GetStateString(PowerSystemState_t state)
{
    if (state <= POWER_STATE_EMERGENCY) {
        return state_strings[state];
    }
    return "未知";
}

const char* PowerMonitor_GetPowerLevelString(PowerLevel_t level)
{
    if (level <= POWER_LEVEL_FULL) {
        return level_strings[level];
    }
    return "未知";
}
