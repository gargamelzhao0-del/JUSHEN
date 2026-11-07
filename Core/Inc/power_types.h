#ifndef __POWER_TYPES_H
#define __POWER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_DATA_VERSION      0x0100

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
    FAULT_LEVEL_NONE = 0,
    FAULT_LEVEL_1 = 1,
    FAULT_LEVEL_2 = 2,
    FAULT_LEVEL_3 = 3
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
    EventType_t event_type;
    uint8_t event_data;
    uint8_t fault_code;
    uint8_t fault_level;
    uint32_t timestamp;
} PowerEvent_t;

typedef void (*PowerEventCallback_t)(PowerEvent_t *event);

#endif
