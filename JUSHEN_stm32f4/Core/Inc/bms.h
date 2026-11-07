#ifndef __BMS_H
#define __BMS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define BMS_FRAME_TIMEOUT_MS    5000

typedef enum {
    BMS_MSG_IDLE = 0,
    BMS_MSG_DISCHARGE = 1,
    BMS_MSG_CHARGE = 2
} BMS_AGV_StateTypeDef;

typedef enum {
    BMS_QUERY_NONE = 0x00,
    BMS_QUERY_BASIC_STATUS = 0xF0,
    BMS_QUERY_CELL_VOLTAGE = 0xF1,
    BMS_QUERY_TEMP_VERSION = 0xF2,
    BMS_QUERY_LIMIT_PARAMS = 0xF3,
    BMS_QUERY_EXTENDED_STATUS = 0xF4
} BMS_QueryTypeDef;

typedef enum {
    BMS_CHARGER_CTRL_AUTO_OFF = 0xA1,
    BMS_CHARGER_CTRL_AUTO_ON = 0xA2,
    BMS_CHARGER_CTRL_MANUAL_ON = 0xA3
} BMS_ChargerCtrlTypeDef;

typedef enum {
    BMS_IDLE_STATE = 0,
    BMS_DISCHARGE_STATE = 1,
    BMS_CHARGE_STATE = 2
} BMS_ChargeStateTypeDef;

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVER_VOLTAGE = 1,
    FAULT_UNDER_VOLTAGE = 2,
    FAULT_CELL_OVER_VOLTAGE = 3,
    FAULT_CELL_UNDER_VOLTAGE = 4,
    FAULT_CELL_VOLTAGE_DIFF = 5,
    FAULT_OVER_TEMP = 6,
    FAULT_UNDER_TEMP = 7,
    FAULT_TEMP_DIFF = 8,
    FAULT_OVER_CURRENT = 9,
    FAULT_OTHER_DISCHARGE = 10
} BMS_FaultTypeDef;

typedef enum {
    FAULT_LEVEL_1 = 1,
    FAULT_LEVEL_2 = 2,
    FAULT_LEVEL_3 = 3
} BMS_FaultLevelTypeDef;

typedef enum {
    BMS_SAFE_STATE_NORMAL = 0,
    BMS_SAFE_STATE_WARNING = 1,
    BMS_SAFE_STATE_PROTECTION = 2,
    BMS_SAFE_STATE_FAULT = 3
} BMS_SafeStateTypeDef;

typedef enum {
    BMS_PROTECT_NONE = 0,
    BMS_PROTECT_STOP_CHARGE = 1,
    BMS_PROTECT_STOP_DISCHARGE = 2,
    BMS_PROTECT_STOP_ALL = 3,
    BMS_PROTECT_EMERGENCY = 4
} BMS_ProtectActionTypeDef;

typedef struct {
    uint8_t soc;
    uint8_t soh;
    uint16_t total_voltage;
    int16_t total_current;
    bool full_charge_flag;
    bool full_discharge_flag;
} BMS_BasicStatusTypeDef;

typedef struct {
    uint16_t max_cell_voltage;
    uint16_t min_cell_voltage;
    uint8_t max_voltage_pos;
    uint8_t min_voltage_pos;
    BMS_ChargeStateTypeDef charge_state;
    BMS_FaultTypeDef fault_type;
    uint8_t fault_level;
} BMS_CellVoltageTypeDef;

typedef struct {
    int8_t max_temp;
    int8_t min_temp;
    uint8_t max_temp_pos;
    uint8_t min_temp_pos;
    uint8_t version;
    uint8_t charge_cal_flag;
} BMS_TempVersionTypeDef;

typedef struct {
    uint16_t max_charge_voltage;
    int16_t max_charge_current;
    uint16_t max_discharge_current;
    int16_t max_feedback_current;
} BMS_LimitParamsTypeDef;

typedef struct {
    uint8_t status_flags;
    uint8_t soc;
    uint16_t cycle_count;
    int16_t total_current;
    int8_t max_temp;
    BMS_FaultTypeDef fault_type;
    uint8_t fault_level;
} BMS_ExtendedStatusTypeDef;

typedef struct {
    bool comm_flag;
    bool charge_temp_high;
    bool charge_temp_low;
    bool charge_current_high;
    bool insulation_fault;
    bool comm_status;
    bool fault_flag;
    bool charge_state;
    bool full_charge_flag;
    bool full_discharge_flag;
    bool cell_under_voltage_warn;
    bool cell_under_voltage_critical;
    bool over_current_protect;
    bool over_temp_protect;
} BMS_StatusFlagsTypeDef;

typedef struct {
    bool charge_temp_high;
    bool charge_temp_low;
    bool input_voltage_err;
    bool startup_status;
    bool comm_status;
    bool hardware_fault;
    bool charger_temp_high;
} CCS_StatusFlagsTypeDef;

typedef struct {
    uint16_t total_voltage_over;
    uint16_t total_voltage_under;
    uint16_t cell_voltage_over;
    uint16_t cell_voltage_under;
    uint16_t cell_voltage_diff_warn;
    uint16_t cell_voltage_diff_protect;
    int8_t temp_over;
    int8_t temp_under;
    int8_t temp_diff_warn;
    int16_t current_over_charge;
    int16_t current_over_discharge;
    int16_t current_protect_limit;
} BMS_SafeThresholdTypeDef;

typedef struct {
    BMS_SafeStateTypeDef safe_state;
    BMS_ProtectActionTypeDef protect_action;

    bool fault_over_voltage;
    bool fault_under_voltage;
    bool fault_cell_over_voltage;
    bool fault_cell_under_voltage;
    bool fault_cell_voltage_diff;
    bool fault_over_temp;
    bool fault_under_temp;
    bool fault_temp_diff;
    bool fault_over_current;
    bool fault_comm_timeout;
    bool fault_insulation;

    uint8_t warning_count_over_voltage;
    uint8_t warning_count_under_voltage;
    uint8_t warning_count_over_temp;
    uint8_t warning_count_over_current;
    uint8_t warning_count_cell_diff;

    uint32_t fault_start_tick;
    uint32_t last_warning_tick;
    bool protection_active;
} BMS_SafetyManageTypeDef;

typedef struct {
    bool init_ok;
    uint32_t last_recv_tick;

    BMS_BasicStatusTypeDef basic_status;
    BMS_CellVoltageTypeDef cell_voltage;
    BMS_TempVersionTypeDef temp_version;
    BMS_LimitParamsTypeDef limit_params;
    BMS_ExtendedStatusTypeDef extended_status;
    BMS_StatusFlagsTypeDef status_flags;
    BMS_SafetyManageTypeDef safety;

    BMS_QueryTypeDef query_pending;
    BMS_ChargerCtrlTypeDef charger_ctrl;

    bool bms_online;
    uint32_t bms_offline_count;
} BMS_HandleTypeDef;

extern BMS_HandleTypeDef g_bms_handle;

void BMS_Init(BMS_HandleTypeDef *hbms);
void BMS_Process(BMS_HandleTypeDef *hbms, uint32_t current_tick);

bool BMS_ParseFrame(BMS_HandleTypeDef *hbms, uint16_t can_id, uint8_t *data, uint8_t len);

void BMS_SendQueryFrame(BMS_HandleTypeDef *hbms, BMS_QueryTypeDef query_type);
void BMS_SendControlFrame(BMS_HandleTypeDef *hbms, BMS_AGV_StateTypeDef agv_state, BMS_QueryTypeDef query_type, BMS_ChargerCtrlTypeDef charger_ctrl);

BMS_FaultTypeDef BMS_GetFaultType(uint8_t fault_code, uint8_t *fault_level);
const char* BMS_GetFaultString(BMS_FaultTypeDef fault);

void BMS_Safety_Init(BMS_HandleTypeDef *hbms);
void BMS_Safety_Check(BMS_HandleTypeDef *hbms, uint32_t current_tick);
BMS_SafeStateTypeDef BMS_Safety_GetState(BMS_HandleTypeDef *hbms);
BMS_ProtectActionTypeDef BMS_Safety_GetProtection(BMS_HandleTypeDef *hbms);
const char* BMS_Safety_GetStateString(BMS_SafeStateTypeDef state);
const char* BMS_Safety_GetProtectString(BMS_ProtectActionTypeDef action);

bool BMS_IsChargeAllowed(BMS_HandleTypeDef *hbms);
bool BMS_IsDischargeAllowed(BMS_HandleTypeDef *hbms);

#endif
