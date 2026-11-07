#include "bms.h"
#include "can.h"
#include <string.h>
#include <stdio.h>

BMS_HandleTypeDef g_bms_handle;

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
    "其他放电故障"
};

static const char* safe_state_strings[] = {
    "正常",
    "警告",
    "保护",
    "故障"
};

static const char* protect_action_strings[] = {
    "无动作",
    "停止充电",
    "停止放电",
    "停止充放电",
    "紧急停止"
};

#define WARNING_THRESHOLD     3
#define CELL_VOLTAGE_OVER    4200
#define CELL_VOLTAGE_UNDER   3000
#define CELL_DIFF_WARN       500
#define CELL_DIFF_PROTECT    800
#define TEMP_OVER            55
#define TEMP_UNDER           -10
#define TEMP_DIFF_WARN       15
#define CURRENT_OVER_CHARGE  100
#define CURRENT_OVER_DISCHARGE 200

/* ============== 修复: BMS数据校验范围定义 ============== */
#define BMS_SOC_MIN          0
#define BMS_SOC_MAX          100
#define BMS_SOH_MIN          0
#define BMS_SOH_MAX          100
#define BMS_VOLTAGE_MIN      0
#define BMS_VOLTAGE_MAX      60000
#define BMS_CURRENT_MIN      -500
#define BMS_CURRENT_MAX      500
#define BMS_CELL_VOLTAGE_MIN 2500
#define BMS_CELL_VOLTAGE_MAX 4500
#define BMS_TEMP_MIN         -40
#define BMS_TEMP_MAX         85

static uint8_t bms_validation_error_count = 0;

static bool validate_soc(uint8_t soc)
{
    return (soc >= BMS_SOC_MIN && soc <= BMS_SOC_MAX);
}

static bool validate_soh(uint8_t soh)
{
    return (soh >= BMS_SOH_MIN && soh <= BMS_SOH_MAX);
}

static bool validate_voltage(uint16_t voltage)
{
    return (voltage >= BMS_VOLTAGE_MIN && voltage <= BMS_VOLTAGE_MAX);
}

static bool validate_current(int16_t current)
{
    return (current >= BMS_CURRENT_MIN && current <= BMS_CURRENT_MAX);
}

static bool validate_cell_voltage(uint16_t voltage)
{
    return (voltage >= BMS_CELL_VOLTAGE_MIN && voltage <= BMS_CELL_VOLTAGE_MAX);
}

static bool validate_temp(int8_t temp)
{
    return (temp >= BMS_TEMP_MIN && temp <= BMS_TEMP_MAX);
}

void BMS_Init(BMS_HandleTypeDef *hbms)
{
    memset(hbms, 0, sizeof(BMS_HandleTypeDef));
    hbms->init_ok = true;
    hbms->last_recv_tick = 0;
    hbms->bms_online = false;
    hbms->bms_offline_count = 0;
    hbms->query_pending = BMS_QUERY_NONE;
    hbms->charger_ctrl = BMS_CHARGER_CTRL_MANUAL_ON;
    BMS_Safety_Init(hbms);
}

void BMS_Process(BMS_HandleTypeDef *hbms, uint32_t current_tick)
{
    if (!hbms->init_ok) {
        return;
    }

    if (current_tick - hbms->last_recv_tick > BMS_FRAME_TIMEOUT_MS) {
        hbms->bms_online = false;
        hbms->bms_offline_count++;
    } else {
        hbms->bms_online = true;
    }

    BMS_Safety_Check(hbms, current_tick);
}

bool BMS_ParseFrame(BMS_HandleTypeDef *hbms, uint16_t can_id, uint8_t *data, uint8_t len)
{
    if (hbms == NULL || data == NULL || len != 8) {
        return false;
    }

    hbms->last_recv_tick = HAL_GetTick();
    hbms->bms_online = true;

    switch (can_id) {
        case 0x2F0:
            /* ============== 修复: 添加数据校验 ============== */
            if (validate_soc(data[0])) {
                hbms->basic_status.soc = data[0];
            } else {
                bms_validation_error_count++;
                printf("[BMS] Invalid SOC: %u (valid: 0-100)\r\n", data[0]);
            }
            if (validate_soh(data[1])) {
                hbms->basic_status.soh = data[1];
            } else {
                bms_validation_error_count++;
                printf("[BMS] Invalid SOH: %u (valid: 0-100)\r\n", data[1]);
            }
            {
                uint16_t voltage = (uint16_t)((data[2] << 8) | data[3]);
                if (validate_voltage(voltage)) {
                    hbms->basic_status.total_voltage = voltage;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid voltage: %u mV (valid: 0-60000)\r\n", voltage);
                }
            }
            {
                int16_t current = (int16_t)((data[4] << 8) | data[5]);
                if (validate_current(current)) {
                    hbms->basic_status.total_current = current;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid current: %d A (valid: -500~500)\r\n", current);
                }
            }
            hbms->basic_status.full_charge_flag = (data[6] == 1);
            hbms->basic_status.full_discharge_flag = (data[7] == 1);
            break;

        case 0x2F1:
            /* ============== 修复: 添加单体电压校验 ============== */
            {
                uint16_t max_v = (uint16_t)((data[0] << 8) | data[1]);
                uint16_t min_v = (uint16_t)((data[2] << 8) | data[3]);
                if (validate_cell_voltage(max_v)) {
                    hbms->cell_voltage.max_cell_voltage = max_v;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid max cell voltage: %u mV (valid: 2500-4500)\r\n", max_v);
                }
                if (validate_cell_voltage(min_v)) {
                    hbms->cell_voltage.min_cell_voltage = min_v;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid min cell voltage: %u mV (valid: 2500-4500)\r\n", min_v);
                }
            }
            hbms->cell_voltage.max_voltage_pos = data[4];
            hbms->cell_voltage.min_voltage_pos = data[5];
            hbms->cell_voltage.charge_state = (BMS_ChargeStateTypeDef)data[6];
            {
                uint8_t level = 0;
                hbms->cell_voltage.fault_type = BMS_GetFaultType(data[7], &level);
                hbms->cell_voltage.fault_level = level;
            }
            break;

        case 0x2F2:
            /* ============== 修复: 添加温度校验 ============== */
            {
                int8_t max_t = (int8_t)(data[0] - 40);
                int8_t min_t = (int8_t)(data[1] - 40);
                if (validate_temp(max_t)) {
                    hbms->temp_version.max_temp = max_t;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid max temp: %d C (valid: -40~85)\r\n", max_t);
                }
                if (validate_temp(min_t)) {
                    hbms->temp_version.min_temp = min_t;
                } else {
                    bms_validation_error_count++;
                    printf("[BMS] Invalid min temp: %d C (valid: -40~85)\r\n", min_t);
                }
            }
            hbms->temp_version.max_temp_pos = data[2];
            hbms->temp_version.min_temp_pos = data[3];
            hbms->temp_version.version = data[4];
            hbms->temp_version.charge_cal_flag = data[5];
            break;

        case 0x2F3:
            {
                uint16_t voltage = (uint16_t)((data[0] << 8) | data[1]);
                if (validate_voltage(voltage)) {
                    hbms->limit_params.max_charge_voltage = voltage;
                }
            }
            {
                int16_t current = (int16_t)((data[2] << 8) | data[3]);
                if (validate_current(current)) {
                    hbms->limit_params.max_charge_current = current;
                }
            }
            {
                uint16_t current = (uint16_t)((data[4] << 8) | data[5]);
                hbms->limit_params.max_discharge_current = current;
            }
            {
                int16_t current = (int16_t)((data[6] << 8) | data[7]);
                if (validate_current(current)) {
                    hbms->limit_params.max_feedback_current = current;
                }
            }
            break;

        case 0x2F4:
            hbms->extended_status.status_flags = data[0];
            if (validate_soc(data[1])) {
                hbms->extended_status.soc = data[1];
            }
            hbms->extended_status.cycle_count = (uint16_t)((data[2] << 8) | data[3]);
            {
                int16_t current = (int16_t)((data[4] << 8) | data[5]);
                if (validate_current(current)) {
                    hbms->extended_status.total_current = current;
                }
            }
            {
                int8_t temp = (int8_t)(data[6] - 40);
                if (validate_temp(temp)) {
                    hbms->extended_status.max_temp = temp;
                }
            }
            {
                uint8_t level = 0;
                hbms->extended_status.fault_type = BMS_GetFaultType(data[7], &level);
                hbms->extended_status.fault_level = level;
            }

            hbms->status_flags.comm_flag = (data[0] & 0x02) != 0;
            hbms->status_flags.full_charge_flag = (data[0] & 0x04) != 0;
            hbms->status_flags.full_discharge_flag = (data[0] & 0x08) != 0;
            hbms->status_flags.charge_temp_high = (data[0] & 0x80) != 0;
            hbms->status_flags.charge_state = (data[0] & 0x01) != 0;
            hbms->status_flags.cell_under_voltage_warn = (data[0] & 0x10) != 0;
            hbms->status_flags.cell_under_voltage_critical = (data[0] & 0x20) != 0;
            hbms->status_flags.over_current_protect = (data[0] & 0x40) != 0;
            break;

        default:
            return false;
    }

    return true;
}

void BMS_SendQueryFrame(BMS_HandleTypeDef *hbms, BMS_QueryTypeDef query_type)
{
    uint8_t tx_data[8] = {0};

    tx_data[1] = query_type;

    CAN_Send_Polling(0x6AC, tx_data, 8);
}

void BMS_SendControlFrame(BMS_HandleTypeDef *hbms, BMS_AGV_StateTypeDef agv_state, BMS_QueryTypeDef query_type, BMS_ChargerCtrlTypeDef charger_ctrl)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = agv_state;
    tx_data[1] = query_type;
    tx_data[2] = charger_ctrl;

    CAN_Send_Polling(0x6AC, tx_data, 8);
}

BMS_FaultTypeDef BMS_GetFaultType(uint8_t fault_code, uint8_t *fault_level)
{
    if (fault_code == 0) {
        if (fault_level) *fault_level = 0;
        return FAULT_NONE;
    }

    if (fault_code >= 1 && fault_code <= 10) {
        if (fault_level) *fault_level = FAULT_LEVEL_1;
        return (BMS_FaultTypeDef)fault_code;
    }
    else if (fault_code >= 51 && fault_code <= 60) {
        if (fault_level) *fault_level = FAULT_LEVEL_2;
        return (BMS_FaultTypeDef)(fault_code - 50);
    }
    else if (fault_code >= 101 && fault_code <= 110) {
        if (fault_level) *fault_level = FAULT_LEVEL_3;
        return (BMS_FaultTypeDef)(fault_code - 100);
    }

    if (fault_level) *fault_level = 0;
    return FAULT_NONE;
}

const char* BMS_GetFaultString(BMS_FaultTypeDef fault)
{
    if (fault >= FAULT_NONE && fault <= FAULT_OTHER_DISCHARGE) {
        return fault_strings[fault];
    }
    return "未知故障";
}

void BMS_Safety_Init(BMS_HandleTypeDef *hbms)
{
    if (hbms == NULL) return;

    hbms->safety.safe_state = BMS_SAFE_STATE_NORMAL;
    hbms->safety.protect_action = BMS_PROTECT_NONE;

    hbms->safety.fault_over_voltage = false;
    hbms->safety.fault_under_voltage = false;
    hbms->safety.fault_cell_over_voltage = false;
    hbms->safety.fault_cell_under_voltage = false;
    hbms->safety.fault_cell_voltage_diff = false;
    hbms->safety.fault_over_temp = false;
    hbms->safety.fault_under_temp = false;
    hbms->safety.fault_temp_diff = false;
    hbms->safety.fault_over_current = false;
    hbms->safety.fault_comm_timeout = false;
    hbms->safety.fault_insulation = false;

    hbms->safety.warning_count_over_voltage = 0;
    hbms->safety.warning_count_under_voltage = 0;
    hbms->safety.warning_count_over_temp = 0;
    hbms->safety.warning_count_over_current = 0;
    hbms->safety.warning_count_cell_diff = 0;

    hbms->safety.fault_start_tick = 0;
    hbms->safety.last_warning_tick = 0;
    hbms->safety.protection_active = false;
}

void BMS_Safety_Check(BMS_HandleTypeDef *hbms, uint32_t current_tick)
{
    if (hbms == NULL || !hbms->init_ok) return;

    bool has_warning = false;
    bool has_fault = false;
    BMS_ProtectActionTypeDef protect_action = BMS_PROTECT_NONE;

    if (!hbms->bms_online) {
        hbms->safety.fault_comm_timeout = true;
        has_fault = true;
        protect_action = BMS_PROTECT_STOP_ALL;
    } else {
        hbms->safety.fault_comm_timeout = false;
    }

    uint16_t cell_max = hbms->cell_voltage.max_cell_voltage;
    uint16_t cell_min = hbms->cell_voltage.min_cell_voltage;
    uint16_t cell_diff = (cell_max > cell_min) ? (cell_max - cell_min) : 0;

    if (cell_max >= CELL_VOLTAGE_OVER) {
        hbms->safety.warning_count_over_voltage++;
        if (hbms->safety.warning_count_over_voltage >= WARNING_THRESHOLD) {
            hbms->safety.fault_cell_over_voltage = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_CHARGE;
        } else {
            has_warning = true;
        }
    } else {
        hbms->safety.warning_count_over_voltage = 0;
        hbms->safety.fault_cell_over_voltage = false;
    }

    if (cell_min <= CELL_VOLTAGE_UNDER) {
        hbms->safety.warning_count_under_voltage++;
        if (hbms->safety.warning_count_under_voltage >= WARNING_THRESHOLD) {
            hbms->safety.fault_cell_under_voltage = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_DISCHARGE;
        } else {
            has_warning = true;
        }
    } else {
        hbms->safety.warning_count_under_voltage = 0;
        hbms->safety.fault_cell_under_voltage = false;
    }

    if (cell_diff >= CELL_DIFF_PROTECT) {
        hbms->safety.warning_count_cell_diff++;
        if (hbms->safety.warning_count_cell_diff >= WARNING_THRESHOLD) {
            hbms->safety.fault_cell_voltage_diff = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_ALL;
        } else {
            has_warning = true;
        }
    } else if (cell_diff >= CELL_DIFF_WARN) {
        has_warning = true;
        hbms->safety.warning_count_cell_diff++;
    } else {
        hbms->safety.warning_count_cell_diff = 0;
        hbms->safety.fault_cell_voltage_diff = false;
    }

    int8_t temp_max = hbms->temp_version.max_temp;
    int8_t temp_min = hbms->temp_version.min_temp;
    int8_t temp_diff = (temp_max > temp_min) ? (temp_max - temp_min) : 0;

    if (temp_max >= TEMP_OVER) {
        hbms->safety.warning_count_over_temp++;
        if (hbms->safety.warning_count_over_temp >= WARNING_THRESHOLD) {
            hbms->safety.fault_over_temp = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_ALL;
        } else {
            has_warning = true;
        }
    } else {
        hbms->safety.warning_count_over_temp = 0;
        hbms->safety.fault_over_temp = false;
    }

    if (temp_min <= TEMP_UNDER) {
        hbms->safety.fault_under_temp = true;
        has_fault = true;
        protect_action = BMS_PROTECT_STOP_ALL;
    } else {
        hbms->safety.fault_under_temp = false;
    }

    if (temp_diff >= TEMP_DIFF_WARN) {
        has_warning = true;
    }

    int16_t current = hbms->basic_status.total_current;

    if (current < 0 && (-current) >= CURRENT_OVER_CHARGE) {
        hbms->safety.warning_count_over_current++;
        if (hbms->safety.warning_count_over_current >= WARNING_THRESHOLD) {
            hbms->safety.fault_over_current = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_CHARGE;
        } else {
            has_warning = true;
        }
    } else if (current > 0 && current >= CURRENT_OVER_DISCHARGE) {
        hbms->safety.warning_count_over_current++;
        if (hbms->safety.warning_count_over_current >= WARNING_THRESHOLD) {
            hbms->safety.fault_over_current = true;
            has_fault = true;
            protect_action = BMS_PROTECT_STOP_DISCHARGE;
        } else {
            has_warning = true;
        }
    } else {
        hbms->safety.warning_count_over_current = 0;
        hbms->safety.fault_over_current = false;
    }

    if (hbms->basic_status.full_charge_flag) {
        protect_action = (protect_action > BMS_PROTECT_STOP_CHARGE) ?
                         protect_action : BMS_PROTECT_STOP_CHARGE;
    }

    if (hbms->basic_status.full_discharge_flag) {
        protect_action = (protect_action > BMS_PROTECT_STOP_DISCHARGE) ?
                         protect_action : BMS_PROTECT_STOP_DISCHARGE;
    }

    if (hbms->extended_status.fault_level == FAULT_LEVEL_3) {
        has_fault = true;
        protect_action = BMS_PROTECT_STOP_ALL;
    }

    if (has_fault) {
        hbms->safety.safe_state = BMS_SAFE_STATE_FAULT;
        if (hbms->safety.fault_start_tick == 0) {
            hbms->safety.fault_start_tick = current_tick;
        }
        hbms->safety.protection_active = true;
    } else if (has_warning) {
        hbms->safety.safe_state = BMS_SAFE_STATE_WARNING;
        hbms->safety.last_warning_tick = current_tick;
        hbms->safety.protection_active = false;
    } else {
        hbms->safety.safe_state = BMS_SAFE_STATE_NORMAL;
        hbms->safety.protection_active = false;
        hbms->safety.fault_start_tick = 0;
    }

    /* ============== 修复: 保护动作可恢复 ============== */
    /* 当所有故障清除时，保护动作降级恢复 */
    if (!has_fault && !has_warning) {
        /* 所有故障已清除，逐步恢复保护动作 */
        hbms->safety.protect_action = BMS_PROTECT_NONE;
    } else {
        /* 取当前计算的保护动作与历史动作中较严重者 */
        if (protect_action > hbms->safety.protect_action) {
            hbms->safety.protect_action = protect_action;
        } else if (!has_fault) {
            /* 故障已解除但仍有警告，允许从STOP_ALL降级 */
            hbms->safety.protect_action = protect_action;
        }
    }
}

BMS_SafeStateTypeDef BMS_Safety_GetState(BMS_HandleTypeDef *hbms)
{
    if (hbms == NULL) return BMS_SAFE_STATE_FAULT;
    return hbms->safety.safe_state;
}

BMS_ProtectActionTypeDef BMS_Safety_GetProtection(BMS_HandleTypeDef *hbms)
{
    if (hbms == NULL) return BMS_PROTECT_EMERGENCY;
    return hbms->safety.protect_action;
}

const char* BMS_Safety_GetStateString(BMS_SafeStateTypeDef state)
{
    if (state <= BMS_SAFE_STATE_FAULT) {
        return safe_state_strings[state];
    }
    return "未知状态";
}

const char* BMS_Safety_GetProtectString(BMS_ProtectActionTypeDef action)
{
    if (action <= BMS_PROTECT_EMERGENCY) {
        return protect_action_strings[action];
    }
    return "未知动作";
}

bool BMS_IsChargeAllowed(BMS_HandleTypeDef *hbms)
{
    if (hbms == NULL || !hbms->init_ok) return false;

    if (!hbms->bms_online) return false;

    if (hbms->safety.protect_action == BMS_PROTECT_STOP_ALL ||
        hbms->safety.protect_action == BMS_PROTECT_EMERGENCY) {
        return false;
    }

    if (hbms->safety.protect_action == BMS_PROTECT_STOP_CHARGE) {
        return false;
    }

    if (hbms->basic_status.full_charge_flag) {
        return false;
    }

    if (hbms->safety.fault_cell_over_voltage ||
        hbms->safety.fault_over_current) {
        return false;
    }

    return true;
}

bool BMS_IsDischargeAllowed(BMS_HandleTypeDef *hbms)
{
    if (hbms == NULL || !hbms->init_ok) return false;

    if (!hbms->bms_online) return false;

    if (hbms->safety.protect_action == BMS_PROTECT_STOP_ALL ||
        hbms->safety.protect_action == BMS_PROTECT_EMERGENCY) {
        return false;
    }

    if (hbms->safety.protect_action == BMS_PROTECT_STOP_DISCHARGE) {
        return false;
    }

    if (hbms->basic_status.full_discharge_flag) {
        return false;
    }

    if (hbms->safety.fault_cell_under_voltage ||
        hbms->safety.fault_over_current ||
        hbms->safety.fault_over_temp ||
        hbms->safety.fault_under_temp) {
        return false;
    }

    return true;
}
