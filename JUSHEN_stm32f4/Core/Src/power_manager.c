#include "power_manager.h"
#include <string.h>
#include <stdio.h>

PowerManager_t g_power_manager;

static PowerThreshold_t g_threshold = {
    .soc_low_threshold = 20,
    .soc_critical_threshold = 10,
    .soc_high_threshold = 80,
    .temp_high_threshold = 55,
    .temp_low_threshold = -10,
    .current_high_threshold = 200,
    .cell_diff_warn_threshold = 500,
    .cell_diff_critical_threshold = 800,
    .bms_timeout_ms = 5000,
    .motor_timeout_ms = 3000
};

#define MIN_STATE_DURATION_MS  1000  /* 最小状态持续时间，防止频繁切换 */

static const char* state_strings[] = {
    "初始化",
    "正常",
    "警告",
    "故障",
    "紧急"
};

static const char* level_strings[] = {
    "严重",
    "低电量",
    "中等",
    "充足",
    "满电"
};

static const char* fault_strings[] = {
    "无故障",
    "总压过压",
    "总压欠压",
    "单体过压",
    "单体欠压",
    "单体压差大",
    "电池过温",
    "电池低温",
    "电池温差大",
    "电池过流",
    "其他故障",
    "通信超时",
    "电机过载",
    "电机过温"
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

static uint16_t CalculateChecksum(uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* ============== 事件队列操作 ============== */
static void EventQueue_Push(PowerManager_t *pm, EventType_t event, uint8_t data)
{
    if (pm->event_queue_count >= 8) {
        /* 队列满，丢弃最旧的事件 */
        pm->event_queue_tail = (pm->event_queue_tail + 1) % 8;
        pm->event_queue_count--;
    }
    pm->event_queue[pm->event_queue_head] = event;
    pm->event_data_queue[pm->event_queue_head] = data;
    pm->event_queue_head = (pm->event_queue_head + 1) % 8;
    pm->event_queue_count++;
    pm->event_pending = true;

    /* 同时更新单槽位兼容接口 */
    pm->pending_event = event;
    pm->event_data = data;
}

static bool EventQueue_Pop(PowerManager_t *pm, EventType_t *event, uint8_t *data)
{
    if (pm->event_queue_count == 0) {
        pm->event_pending = false;
        return false;
    }
    if (event) *event = pm->event_queue[pm->event_queue_tail];
    if (data) *data = pm->event_data_queue[pm->event_queue_tail];
    pm->event_queue_tail = (pm->event_queue_tail + 1) % 8;
    pm->event_queue_count--;

    if (pm->event_queue_count > 0) {
        pm->pending_event = pm->event_queue[pm->event_queue_tail];
        pm->event_data = pm->event_data_queue[pm->event_queue_tail];
    } else {
        pm->event_pending = false;
        pm->pending_event = EVENT_TYPE_NONE;
        pm->event_data = 0;
    }
    return true;
}

void PowerManager_Init(PowerManager_t *pm)
{
    memset(pm, 0, sizeof(PowerManager_t));
    
    pm->comm_mode = POWER_COMM_I2C;
    pm->comm_mode_switch_pending = false;
    pm->system_state = POWER_STATE_INIT;
    pm->power_level = POWER_LEVEL_MEDIUM;
    
    pm->battery.soc = 0;
    pm->battery.soh = 100;
    pm->battery.charge_state = CHARGE_STATE_IDLE;
    pm->battery.safe_level = 0;
    pm->battery.bms_online = 0;
    
    pm->motor.motor_online = 0;
    pm->motor.motor_overload = 0;
    pm->motor.motor_overtemp = 0;
    
    pm->fault.fault_code = FAULT_TYPE_NONE;
    pm->fault.fault_level = POWER_FAULT_LEVEL_NONE;
    pm->fault.protect_action = PROTECT_ACTION_NONE;
    
    pm->system.system_state = POWER_STATE_INIT;
    pm->system.power_level = POWER_LEVEL_MEDIUM;
    pm->system.can_status = 0;
    pm->system.comm_status = 0;
    
    pm->last_update_tick = 0;
    pm->last_report_tick = 0;
    pm->last_bms_tick = 0;
    pm->last_motor_tick = 0;
    
    pm->bms_online = false;
    for (int i = 0; i < 4; i++) {
        pm->motor_online[i] = false;
        pm->motor_offline_count[i] = 0;
    }
    pm->bms_offline_count = 0;
    
    pm->event_pending = false;
    pm->pending_event = EVENT_TYPE_NONE;
    pm->event_data = 0;
    pm->event_queue_head = 0;
    pm->event_queue_tail = 0;
    pm->event_queue_count = 0;
    memset(pm->event_queue, 0, sizeof(pm->event_queue));
    memset(pm->event_data_queue, 0, sizeof(pm->event_data_queue));

    pm->prev_state = POWER_STATE_INIT;
    pm->state_entry_tick = 0;
    pm->state_transition_count = 0;

    pm->last_reported_soc = 100;
    pm->soc_milestone = 100;
    
    pm->data_changed = false;
    pm->change_bitmap = 0;
    
    pm->init_ok = true;
    
    printf("[PowerManager] Initialized, default comm mode: I2C\r\n");
}

void PowerManager_Process(PowerManager_t *pm, uint32_t current_tick)
{
    if (!pm->init_ok) {
        return;
    }
    
    bool bms_timeout = (current_tick - pm->last_bms_tick) > g_threshold.bms_timeout_ms;
    if (bms_timeout && pm->bms_online) {
        pm->bms_online = false;
        pm->bms_offline_count++;
        pm->battery.bms_online = 0;
        pm->data_changed = true;
        pm->change_bitmap |= 0x01;
        
        if (!pm->event_pending) {
            EventQueue_Push(pm, EVENT_TYPE_COMM_LOST, 0);
        }
        
        printf("[PowerManager] BMS offline, count=%d\r\n", pm->bms_offline_count);
    }
    
    for (int i = 0; i < 4; i++) {
        bool motor_timeout = (current_tick - pm->last_motor_tick) > g_threshold.motor_timeout_ms;
        if (motor_timeout && pm->motor_online[i]) {
            pm->motor_online[i] = false;
            pm->motor_offline_count[i]++;
            pm->motor.motor_online &= ~(1 << i);
            pm->data_changed = true;
            pm->change_bitmap |= 0x02;
        }
    }
    
    if (pm->bms_online) {
        if (pm->battery.soc <= g_threshold.soc_critical_threshold) {
            pm->power_level = POWER_LEVEL_CRITICAL;
            EventQueue_Push(pm, EVENT_TYPE_SOC_CRITICAL, pm->battery.soc);
        } else if (pm->battery.soc <= g_threshold.soc_low_threshold) {
            pm->power_level = POWER_LEVEL_LOW;
            EventQueue_Push(pm, EVENT_TYPE_SOC_LOW, pm->battery.soc);
        } else if (pm->battery.soc >= g_threshold.soc_high_threshold) {
            pm->power_level = POWER_LEVEL_HIGH;
        } else if (pm->battery.soc >= 95) {
            pm->power_level = POWER_LEVEL_FULL;
        } else {
            pm->power_level = POWER_LEVEL_MEDIUM;
        }
        
        if (pm->battery.max_temp > g_threshold.temp_high_threshold) {
            EventQueue_Push(pm, EVENT_TYPE_OVER_TEMP, pm->battery.max_temp);
        }
        
        if (pm->battery.cell_diff > g_threshold.cell_diff_critical_threshold) {
            pm->data_changed = true;
            pm->change_bitmap |= 0x01;
        }
    }
    
    /* ============== 修复: 完整的状态机转换逻辑 ============== */
    PowerSystemState_t new_state = pm->system_state;
    uint32_t time_in_state = current_tick - pm->state_entry_tick;

    /* 首次调用时初始化状态进入时间 */
    if (pm->state_entry_tick == 0) {
        pm->state_entry_tick = current_tick;
    }

    /* 确定新状态 */
    if (pm->fault.fault_level >= POWER_FAULT_LEVEL_3) {
        new_state = POWER_STATE_EMERGENCY;
    } else if (pm->fault.fault_level >= POWER_FAULT_LEVEL_2) {
        new_state = POWER_STATE_FAULT;
    } else if (pm->fault.fault_level >= POWER_FAULT_LEVEL_1 ||
               pm->power_level == POWER_LEVEL_CRITICAL ||
               !pm->bms_online) {
        new_state = POWER_STATE_WARNING;
    } else {
        new_state = POWER_STATE_NORMAL;
    }

    /* 检查是否需要状态转换 */
    if (new_state != pm->system_state) {
        bool is_recovery = (new_state < pm->system_state);

        if (is_recovery && time_in_state < MIN_STATE_DURATION_MS) {
            new_state = pm->system_state;
        } else {
            pm->prev_state = pm->system_state;
            pm->system_state = new_state;
            pm->state_entry_tick = current_tick;
            pm->state_transition_count++;
            pm->data_changed = true;
            pm->change_bitmap |= 0x04;

            switch (new_state) {
                case POWER_STATE_EMERGENCY:
                    printf("[PowerManager] !!! EMERGENCY state entered (from %s, trans #%u)\r\n",
                           state_strings[pm->prev_state], pm->state_transition_count);
                    break;
                case POWER_STATE_FAULT:
                    printf("[PowerManager] FAULT state entered (from %s, trans #%u)\r\n",
                           state_strings[pm->prev_state], pm->state_transition_count);
                    break;
                case POWER_STATE_WARNING:
                    printf("[PowerManager] WARNING state entered (from %s, trans #%u)\r\n",
                           state_strings[pm->prev_state], pm->state_transition_count);
                    break;
                case POWER_STATE_NORMAL:
                    printf("[PowerManager] NORMAL state recovered (from %s, trans #%u)\r\n",
                           state_strings[pm->prev_state], pm->state_transition_count);
                    break;
                default:
                    break;
            }
        }
    }
    
    pm->system.system_state = pm->system_state;
    pm->system.power_level = pm->power_level;
    pm->system.can_status = (pm->bms_online ? 0x01 : 0x00) | 
                            ((pm->motor.motor_online & 0x0F) << 1);
    pm->system.comm_status = pm->comm_mode;
    
    if (current_tick - pm->last_update_tick >= POWER_UPDATE_INTERVAL_MS) {
        pm->last_update_tick = current_tick;
        pm->data_changed = true;
        pm->change_bitmap |= 0x0F;
    }
    
    if (pm->comm_mode_switch_pending) {
        pm->comm_mode = pm->pending_mode;
        pm->comm_mode_switch_pending = false;
        printf("[PowerManager] Comm mode switched to: %s\r\n", 
               pm->comm_mode == POWER_COMM_I2C ? "I2C" : "SPI");
    }
}

void PowerManager_UpdateFromBMS(PowerManager_t *pm, BMS_HandleTypeDef *hbms)
{
    if (!pm->init_ok || hbms == NULL) {
        return;
    }
    
    pm->last_bms_tick = HAL_GetTick();
    
    bool was_offline = !pm->bms_online;
    pm->bms_online = hbms->bms_online;
    pm->battery.bms_online = hbms->bms_online ? 1 : 0;
    
    uint8_t old_soc = pm->battery.soc;
    uint8_t old_safe_level = pm->battery.safe_level;
    
    pm->battery.soc = hbms->basic_status.soc;
    pm->battery.soh = hbms->basic_status.soh;
    pm->battery.total_voltage = hbms->basic_status.total_voltage;
    pm->battery.total_current = hbms->basic_status.total_current;
    pm->battery.charge_state = hbms->cell_voltage.charge_state;
    
    pm->battery.max_temp = hbms->temp_version.max_temp;
    pm->battery.min_temp = hbms->temp_version.min_temp;
    
    if (hbms->cell_voltage.max_cell_voltage > hbms->cell_voltage.min_cell_voltage) {
        pm->battery.cell_diff = hbms->cell_voltage.max_cell_voltage - 
                                hbms->cell_voltage.min_cell_voltage;
    } else {
        pm->battery.cell_diff = 0;
    }
    
    BMS_SafeStateTypeDef safe_state = BMS_Safety_GetState(hbms);
    pm->battery.safe_level = (uint8_t)safe_state;
    
    if (hbms->safety.fault_over_voltage || hbms->safety.fault_cell_over_voltage) {
        pm->fault.fault_code = FAULT_TYPE_OVER_VOLTAGE;
    } else if (hbms->safety.fault_under_voltage || hbms->safety.fault_cell_under_voltage) {
        pm->fault.fault_code = FAULT_TYPE_UNDER_VOLTAGE;
    } else if (hbms->safety.fault_cell_voltage_diff) {
        pm->fault.fault_code = FAULT_TYPE_CELL_DIFF;
    } else if (hbms->safety.fault_over_temp) {
        pm->fault.fault_code = FAULT_TYPE_OVER_TEMP;
    } else if (hbms->safety.fault_over_current) {
        pm->fault.fault_code = FAULT_TYPE_OVER_CURRENT;
    } else {
        pm->fault.fault_code = FAULT_TYPE_NONE;
    }
    
    pm->fault.fault_level = hbms->cell_voltage.fault_level;
    if (pm->fault.fault_level == 0 && hbms->extended_status.fault_level > 0) {
        pm->fault.fault_level = hbms->extended_status.fault_level;
    }
    
    pm->fault.protect_action = (uint8_t)BMS_Safety_GetProtection(hbms);
    pm->fault.fault_bitmap = 
        (hbms->safety.fault_over_voltage ? 0x01 : 0) |
        (hbms->safety.fault_under_voltage ? 0x02 : 0) |
        (hbms->safety.fault_over_temp ? 0x04 : 0) |
        (hbms->safety.fault_over_current ? 0x08 : 0) |
        (hbms->safety.fault_comm_timeout ? 0x10 : 0);
    
    pm->fault.fault_timestamp = (uint16_t)(HAL_GetTick() / 1000);
    
    if (old_soc != pm->battery.soc) {
        pm->data_changed = true;
        pm->change_bitmap |= 0x01;
        
        if (pm->battery.soc < pm->soc_milestone) {
            uint8_t new_milestone = (pm->battery.soc / 10) * 10;
            
            if (new_milestone < pm->soc_milestone) {
                pm->soc_milestone = new_milestone;
                EventQueue_Push(pm, EVENT_TYPE_SOC_DROP_10, pm->battery.soc);
                printf("[PowerManager] SOC dropped to milestone: %d%%\r\n", pm->battery.soc);
            }
        } else if (pm->battery.soc > pm->soc_milestone + 10) {
            pm->soc_milestone = (pm->battery.soc / 10) * 10;
            printf("[PowerManager] SOC milestone updated to: %d%% (charging)\r\n", pm->soc_milestone);
        }
    }
    
    if (old_safe_level != pm->battery.safe_level) {
        pm->data_changed = true;
        pm->change_bitmap |= 0x04;
        
        if (pm->battery.safe_level >= 2) {
            EventQueue_Push(pm, (EventType_t)(EVENT_TYPE_FAULT_LEVEL1 + pm->battery.safe_level - 1),
                           pm->fault.fault_code);
        }
    }
    
    if (was_offline && pm->bms_online) {
        printf("[PowerManager] BMS online, SOC=%d%%, SOH=%d%%\r\n", 
               pm->battery.soc, pm->battery.soh);
    }
    
    pm->can_rx_count++;
}

void PowerManager_UpdateMotorStatus(PowerManager_t *pm, uint8_t motor_id,
                                     uint16_t voltage, int16_t current,
                                     bool online, bool overload, bool overtemp)
{
    if (!pm->init_ok || motor_id >= 4) {
        return;
    }
    
    pm->last_motor_tick = HAL_GetTick();
    
    bool was_offline = !pm->motor_online[motor_id];
    pm->motor_online[motor_id] = online;
    
    pm->motor.motor_voltage[motor_id] = voltage;
    pm->motor.motor_current[motor_id] = current;
    
    if (online) {
        pm->motor.motor_online |= (1 << motor_id);
    } else {
        pm->motor.motor_online &= ~(1 << motor_id);
    }
    
    if (overload) {
        pm->motor.motor_overload |= (1 << motor_id);
    } else {
        pm->motor.motor_overload &= ~(1 << motor_id);
    }
    
    if (overtemp) {
        pm->motor.motor_overtemp |= (1 << motor_id);
    } else {
        pm->motor.motor_overtemp &= ~(1 << motor_id);
    }
    
    pm->data_changed = true;
    pm->change_bitmap |= 0x02;
    
    if (overload || overtemp) {
        EventQueue_Push(pm, EVENT_TYPE_MOTOR_FAULT,
                       motor_id | (overload ? 0x10 : 0) | (overtemp ? 0x20 : 0));
    }
    
    if (was_offline && online) {
        printf("[PowerManager] Motor %d online, voltage=%dmV, current=%dmA\r\n",
               motor_id, voltage, current);
    }
}

void PowerManager_SetCommMode(PowerManager_t *pm, PowerCommMode_t mode)
{
    if (pm->comm_mode != mode) {
        pm->comm_mode_switch_pending = true;
        pm->pending_mode = mode;
        printf("[PowerManager] Comm mode switch requested: %s -> %s\r\n",
               pm->comm_mode == POWER_COMM_I2C ? "I2C" : "SPI",
               mode == POWER_COMM_I2C ? "I2C" : "SPI");
    }
}

PowerCommMode_t PowerManager_GetCommMode(PowerManager_t *pm)
{
    return pm->comm_mode;
}

void PowerManager_SwitchCommMode(PowerManager_t *pm)
{
    PowerCommMode_t new_mode = (pm->comm_mode == POWER_COMM_I2C) ? POWER_COMM_SPI : POWER_COMM_I2C;
    PowerManager_SetCommMode(pm, new_mode);
}

PowerSystemState_t PowerManager_GetSystemState(PowerManager_t *pm)
{
    return pm->system_state;
}

PowerLevel_t PowerManager_GetPowerLevel(PowerManager_t *pm)
{
    return pm->power_level;
}

bool PowerManager_IsChargeAllowed(PowerManager_t *pm)
{
    return pm->battery.soc < 95 && 
           pm->battery.safe_level < 3 &&
           pm->bms_online;
}

bool PowerManager_IsDischargeAllowed(PowerManager_t *pm)
{
    return pm->battery.soc > g_threshold.soc_critical_threshold &&
           pm->battery.safe_level < 3 &&
           pm->bms_online;
}

bool PowerManager_IsOperationSafe(PowerManager_t *pm)
{
    return pm->system_state == POWER_STATE_NORMAL ||
           pm->system_state == POWER_STATE_WARNING;
}

bool PowerManager_HasPendingEvent(PowerManager_t *pm)
{
    return pm->event_pending;
}

EventType_t PowerManager_GetPendingEvent(PowerManager_t *pm, uint8_t *data)
{
    if (data != NULL) {
        *data = pm->event_data;
    }
    return pm->pending_event;
}

void PowerManager_ClearEvent(PowerManager_t *pm)
{
    EventType_t event;
    uint8_t data;
    EventQueue_Pop(pm, &event, &data);
}

bool PowerManager_HasDataChanged(PowerManager_t *pm)
{
    return pm->data_changed;
}

void PowerManager_ClearDataChanged(PowerManager_t *pm)
{
    pm->data_changed = false;
    pm->change_bitmap = 0;
}

void PowerManager_GetDataPacket(PowerManager_t *pm, PowerDataPacket_t *packet)
{
    if (packet == NULL) {
        return;
    }
    
    packet->version = POWER_DATA_VERSION;
    packet->timestamp = (uint16_t)(HAL_GetTick() / 1000);
    
    memcpy(&packet->battery, &pm->battery, sizeof(PowerBatteryStatus_t));
    memcpy(&packet->motor, &pm->motor, sizeof(PowerMotorStatus_t));
    memcpy(&packet->fault, &pm->fault, sizeof(PowerFaultStatus_t));
    memcpy(&packet->system, &pm->system, sizeof(PowerSystemStatus_t));
    
    packet->checksum = CalculateChecksum((uint8_t*)packet, 
                                          sizeof(PowerDataPacket_t) - sizeof(uint16_t));
}

void PowerManager_GetBatteryStatus(PowerManager_t *pm, PowerBatteryStatus_t *status)
{
    if (status != NULL) {
        memcpy(status, &pm->battery, sizeof(PowerBatteryStatus_t));
    }
}

void PowerManager_GetMotorStatus(PowerManager_t *pm, PowerMotorStatus_t *status)
{
    if (status != NULL) {
        memcpy(status, &pm->motor, sizeof(PowerMotorStatus_t));
    }
}

void PowerManager_GetFaultStatus(PowerManager_t *pm, PowerFaultStatus_t *status)
{
    if (status != NULL) {
        memcpy(status, &pm->fault, sizeof(PowerFaultStatus_t));
    }
}

void PowerManager_GetSystemStatus(PowerManager_t *pm, PowerSystemStatus_t *status)
{
    if (status != NULL) {
        memcpy(status, &pm->system, sizeof(PowerSystemStatus_t));
    }
}

const char* PowerManager_GetStateString(PowerSystemState_t state)
{
    if (state <= POWER_STATE_EMERGENCY) {
        return state_strings[state];
    }
    return "未知";
}

const char* PowerManager_GetPowerLevelString(PowerLevel_t level)
{
    if (level <= POWER_LEVEL_FULL) {
        return level_strings[level];
    }
    return "未知";
}

const char* PowerManager_GetFaultString(FaultType_t fault)
{
    if (fault <= FAULT_TYPE_MOTOR_OVERTEMP) {
        return fault_strings[fault];
    }
    return "未知";
}

const char* PowerManager_GetEventString(EventType_t event)
{
    if (event <= EVENT_TYPE_SOC_DROP_10) {
        return event_strings[event];
    }
    return "未知";
}

void PowerManager_SetThreshold(PowerManager_t *pm, PowerThreshold_t *threshold)
{
    if (threshold != NULL) {
        memcpy(&g_threshold, threshold, sizeof(PowerThreshold_t));
    }
}

void PowerManager_GetThreshold(PowerManager_t *pm, PowerThreshold_t *threshold)
{
    if (threshold != NULL) {
        memcpy(threshold, &g_threshold, sizeof(PowerThreshold_t));
    }
}
