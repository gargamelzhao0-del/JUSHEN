#ifndef __POWER_TYPES_H
#define __POWER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_DATA_VERSION      0x0100

#define POWER_UPDATE_INTERVAL_MS    1000

#define POWER_BATTERY_DATA_SIZE     16
#define POWER_MOTOR_DATA_SIZE       24
#define POWER_FAULT_DATA_SIZE       8
#define POWER_SYSTEM_DATA_SIZE      4
#define POWER_TOTAL_DATA_SIZE       (POWER_BATTERY_DATA_SIZE + POWER_MOTOR_DATA_SIZE + POWER_FAULT_DATA_SIZE + POWER_SYSTEM_DATA_SIZE)

typedef enum {
    POWER_COMM_I2C = 0,
    POWER_COMM_SPI = 1
} PowerCommMode_t;

typedef enum {
    POWER_STATE_INIT = 0,
    POWER_STATE_NORMAL = 1,
    POWER_STATE_WARNING = 2,
    POWER_STATE_FAULT = 3,
    POWER_STATE_EMERGENCY = 4
} PowerSystemState_t;

typedef enum {
    POWER_LEVEL_CRITICAL = 0,
    POWER_LEVEL_LOW = 1,
    POWER_LEVEL_MEDIUM = 2,
    POWER_LEVEL_HIGH = 3,
    POWER_LEVEL_FULL = 4
} PowerLevel_t;

typedef enum {
    CHARGE_STATE_IDLE = 0,
    CHARGE_STATE_DISCHARGE = 1,
    CHARGE_STATE_CHARGE = 2
} ChargeState_t;

typedef enum {
    FAULT_TYPE_NONE = 0,
    FAULT_TYPE_OVER_VOLTAGE = 1,
    FAULT_TYPE_UNDER_VOLTAGE = 2,
    FAULT_TYPE_CELL_OVER_VOLTAGE = 3,
    FAULT_TYPE_CELL_UNDER_VOLTAGE = 4,
    FAULT_TYPE_CELL_DIFF = 5,
    FAULT_TYPE_OVER_TEMP = 6,
    FAULT_TYPE_UNDER_TEMP = 7,
    FAULT_TYPE_TEMP_DIFF = 8,
    FAULT_TYPE_OVER_CURRENT = 9,
    FAULT_TYPE_OTHER = 10,
    FAULT_TYPE_COMM_TIMEOUT = 11,
    FAULT_TYPE_MOTOR_OVERLOAD = 12,
    FAULT_TYPE_MOTOR_OVERTEMP = 13
} FaultType_t;

typedef enum {
    POWER_FAULT_LEVEL_NONE = 0,
    POWER_FAULT_LEVEL_1 = 1,
    POWER_FAULT_LEVEL_2 = 2,
    POWER_FAULT_LEVEL_3 = 3
} FaultLevel_t;

typedef enum {
    PROTECT_ACTION_NONE = 0,
    PROTECT_ACTION_STOP_CHARGE = 1,
    PROTECT_ACTION_STOP_DISCHARGE = 2,
    PROTECT_ACTION_STOP_ALL = 3,
    PROTECT_ACTION_EMERGENCY = 4
} ProtectAction_t;

typedef enum {
    EVENT_TYPE_NONE = 0,
    EVENT_TYPE_SOC_LOW = 1,
    EVENT_TYPE_SOC_CRITICAL = 2,
    EVENT_TYPE_FAULT_LEVEL1 = 3,
    EVENT_TYPE_FAULT_LEVEL2 = 4,
    EVENT_TYPE_FAULT_LEVEL3 = 5,
    EVENT_TYPE_OVER_CURRENT = 6,
    EVENT_TYPE_OVER_TEMP = 7,
    EVENT_TYPE_MOTOR_FAULT = 8,
    EVENT_TYPE_COMM_LOST = 9,
    EVENT_TYPE_CHARGE_FULL = 10,
    EVENT_TYPE_DISCHARGE_EMPTY = 11,
    EVENT_TYPE_SOC_DROP_10 = 12
} EventType_t;

typedef struct __attribute__((packed)) {
    uint8_t soc;
    uint8_t soh;
    uint16_t total_voltage;
    int16_t total_current;
    int8_t max_temp;
    int8_t min_temp;
    uint8_t charge_state;
    uint8_t safe_level;
    uint16_t cell_diff;
    uint8_t bms_online;
    uint8_t reserved[3];
} PowerBatteryStatus_t;

typedef struct __attribute__((packed)) {
    uint16_t motor_voltage[4];
    int16_t motor_current[4];
    uint8_t motor_online;
    uint8_t motor_overload;
    uint8_t motor_overtemp;
    uint8_t reserved[5];
} PowerMotorStatus_t;

typedef struct __attribute__((packed)) {
    uint8_t fault_code;
    uint8_t fault_level;
    uint8_t protect_action;
    uint8_t fault_bitmap;
    uint16_t fault_timestamp;
    uint8_t can_error_count;
    uint8_t reserved;
} PowerFaultStatus_t;

typedef struct __attribute__((packed)) {
    uint8_t system_state;
    uint8_t power_level;
    uint8_t can_status;
    uint8_t comm_status;
} PowerSystemStatus_t;

typedef struct __attribute__((packed)) {
    uint16_t version;
    uint16_t timestamp;
    PowerBatteryStatus_t battery;
    PowerMotorStatus_t motor;
    PowerFaultStatus_t fault;
    PowerSystemStatus_t system;
    uint16_t checksum;
} PowerDataPacket_t;

typedef struct {
    PowerCommMode_t comm_mode;
    bool comm_mode_switch_pending;
    PowerCommMode_t pending_mode;

    PowerSystemState_t system_state;
    PowerLevel_t power_level;

    PowerBatteryStatus_t battery;
    PowerMotorStatus_t motor;
    PowerFaultStatus_t fault;
    PowerSystemStatus_t system;

    uint32_t last_update_tick;
    uint32_t last_report_tick;
    uint32_t last_bms_tick;
    uint32_t last_motor_tick;

    bool bms_online;
    bool motor_online[4];
    uint8_t bms_offline_count;
    uint8_t motor_offline_count[4];

    bool event_pending;
    EventType_t pending_event;
    uint8_t event_data;

    /* ============== 事件队列（防止事件丢失） ============== */
    EventType_t event_queue[8];
    uint8_t event_data_queue[8];
    uint8_t event_queue_head;
    uint8_t event_queue_tail;
    uint8_t event_queue_count;

    uint8_t last_reported_soc;
    uint8_t soc_milestone;
    
    bool data_changed;
    uint8_t change_bitmap;

    /* ============== 状态机变量（从static改为实例变量） ============== */
    PowerSystemState_t prev_state;
    uint32_t state_entry_tick;
    uint8_t state_transition_count;

    uint16_t can_rx_count;
    uint16_t can_tx_count;
    uint16_t can_error_count;
    uint16_t comm_tx_count;
    uint16_t comm_rx_count;

    bool init_ok;
} PowerManager_t;

typedef struct {
    uint8_t soc_low_threshold;
    uint8_t soc_critical_threshold;
    uint8_t soc_high_threshold;
    int8_t temp_high_threshold;
    int8_t temp_low_threshold;
    int16_t current_high_threshold;
    uint16_t cell_diff_warn_threshold;
    uint16_t cell_diff_critical_threshold;
    uint16_t bms_timeout_ms;
    uint16_t motor_timeout_ms;
} PowerThreshold_t;

#endif
