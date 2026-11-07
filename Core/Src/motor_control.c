/**
 ****************************************************************************************************
 * @file        motor_control.c
 * @version     V2.0 - 工业级优化版本
 * @brief       Xynova旋转关节电机控制驱动实现
 * @note        基于CAN-FD通讯协议实现电机实时控制
 *              V2.0: 统一错误处理策略，使用system_config.h参数定义
 ****************************************************************************************************
 */

#include "motor_control.h"
#include "system_config.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "motor_control_enhanced.h"
#include "security_auth.h"
#include "log_system.h"
#include "error_handling.h"
#include "watchdog.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * V2.0: 统一错误处理宏
 * ============================================================================
 */

#define MOTOR_SET_ERROR(ctx, code, msg) do { \
    (ctx)->last_error = (code); \
    strncpy((ctx)->last_error_msg, (msg), sizeof((ctx)->last_error_msg) - 1); \
    ERROR_ERROR(MODULE_MOTOR_CONTROL, (code), (msg)); \
} while(0)

#define VALIDATE_PARAM(val) do { \
    if (isnan(val) || isinf(val)) { \
        ERROR_ERROR(MODULE_MOTOR_CONTROL, MOTOR_ERR_INVALID_PARAM, "Invalid parameter: NaN or Inf detected"); \
        return MOTOR_ERR_INVALID_PARAM; \
    } \
} while(0)

#define VALIDATE_PARAM_RANGE(val, min, max, name) do { \
    if (isnan(val) || isinf(val) || (val) < (min) || (val) > (max)) { \
        char error_msg[64]; \
        snprintf(error_msg, sizeof(error_msg), "%s out of range: %.2f (valid: %.2f~%.2f)", (name), (val), (min), (max)); \
        ERROR_ERROR(MODULE_MOTOR_CONTROL, MOTOR_ERR_INVALID_PARAM, error_msg); \
        return MOTOR_ERR_INVALID_PARAM; \
    } \
} while(0)

#define VALIDATE_POSITION(pos) VALIDATE_PARAM_RANGE(pos, MOTOR_POS_MIN_DEG, MOTOR_POS_MAX_DEG, "Position")
#define VALIDATE_VELOCITY(vel) VALIDATE_PARAM_RANGE(fabs(vel), 0.0f, MOTOR_VEL_MAX_DEG_S, "Velocity")
#define VALIDATE_ACCEL(accel) VALIDATE_PARAM_RANGE(fabs(accel), 0.0f, MOTOR_ACCEL_MAX_DEG_S2, "Acceleration")
#define VALIDATE_TORQUE(torque) VALIDATE_PARAM_RANGE(fabs(torque), 0.0f, MOTOR_TORQUE_MAX_NM, "Torque")

#define VALIDATE_DEVICE_ID(id) do { \
    if ((id) < MOTOR_ID_MIN || (id) > MOTOR_ID_MAX) { \
        char error_msg[64]; \
        snprintf(error_msg, sizeof(error_msg), "Invalid device ID: 0x%02X", (id)); \
        ERROR_ERROR(MODULE_MOTOR_CONTROL, MOTOR_ERR_INVALID_ID, error_msg); \
        return MOTOR_ERR_INVALID_ID; \
    } \
} while(0)

#define VALIDATE_CAN_CHANNEL(ch) do { \
    if ((ch) < 1 || (ch) > SYSTEM_MAX_CAN_CHANNELS) { \
        char error_msg[64]; \
        snprintf(error_msg, sizeof(error_msg), "Invalid CAN channel: %d", (ch)); \
        ERROR_ERROR(MODULE_MOTOR_CONTROL, MOTOR_ERR_INVALID_PARAM, error_msg); \
        return MOTOR_ERR_INVALID_PARAM; \
    } \
} while(0)

#define CHECK_EMERGENCY_STOP() do { \
    if (!Security_IsSafeToOperate()) { \
        ERROR_WARN(MODULE_MOTOR_CONTROL, MOTOR_ERR_NOT_ENABLED, "Emergency stop active, command rejected"); \
        return MOTOR_ERR_NOT_ENABLED; \
    } \
} while(0)

/* ============================================================================
 * V2.0: 错误上下文结构体
 * ============================================================================ */

typedef struct {
    MotorErrorCode_t last_error;
    char last_error_msg[64];
    uint32_t error_count;
    uint32_t last_error_tick;
} MotorErrorContext_t;

static MotorErrorContext_t g_motor_error_ctx = {0};

/* 外部FDCAN句柄声明 */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* 内部发送封装: 通过 can 索引选择 fdcan1/ fdcan2 的发送接口 */
static uint8_t motor_send_by_idx(uint8_t can, uint8_t *txdata, FDCAN_TxConfigTypeDef *config)
{
    if (can == 2) {
        return fdcan2_send_msg_ex(txdata, config);
    }
    return fdcan1_send_msg_ex(txdata, config);
}

/* 错误命令状态跟踪 (用于区分清除/查询) */
static volatile uint8_t g_error_cmd_pending[128] = {0};  /* 0=查询, 1=清除 */

/* 错误回调函数指针 */
static motor_error_callback_t g_motor_error_callback = NULL;

/* ============== 修复: 电机状态缓存超时机制 ============== */
/* 使用system_config.h中定义的超时时间 */
#ifndef MOTOR_STATUS_TIMEOUT_MS
#define MOTOR_STATUS_TIMEOUT_MS  500
#endif

typedef struct {
    MotorStatus_t status;
    uint32_t last_update_tick;
    bool valid;
} MotorStatusCache_t;

static MotorStatusCache_t g_motor_status_cache[128];

/* RTOS objects for RX processing - 提前声明以避免在使用处未声明的编译错误 */
static osMessageQueueId_t motor_rx_queue = NULL;
static osThreadId_t motor_rx_task_handle = NULL;
#define MOTOR_RX_QUEUE_SIZE 64  /* 增大队列以处理高频反馈 */

/* 消息处理统计 */
static volatile uint32_t g_msg_total_count = 0;         /* 总消息数 */
static volatile uint32_t g_msg_processed_count = 0;    /* 成功处理数 */
static volatile uint32_t g_msg_unknown_count = 0;       /* 未知消息数 */
static volatile uint32_t g_msg_queue_full_count = 0;   /* 队列满次数 */

/******************************************************************************************/
/* 辅助函数实现 */

/**
 * @brief       电机模式转字符串 (安全映射函数)
 * @param       mode        : 电机运行模式
 * @retval      模式字符串
 */
static const char* motor_mode_to_string(MotorMode_t mode)
{
    switch (mode) {
        case MOTOR_MODE_POSITION: return "POSITION";
        case MOTOR_MODE_SPEED:    return "SPEED";
        case MOTOR_MODE_TORQUE:   return "TORQUE";
        case MOTOR_MODE_PROFILE:  return "PROFILE";
        default:                  return "UNKNOWN";
    }
}

/**
 * @brief       构建29位CAN-FD扩展标识符
 * @param       device_id   : 设备ID (0x00~0x7F)
 * @param       message_id  : 消息ID (0x000~0x3FF)
 * @retval      29位扩展标识符
 */
uint32_t motor_build_ext_id(uint8_t device_id, uint16_t message_id)
{
    if (device_id > MOTOR_ID_MAX) {
        LOG_WARN("[MOTOR] device_id 0x%02X exceeds max 0x7F, clamped", device_id);
        device_id = MOTOR_ID_MAX;
    }
    if (message_id > 0x3FF) {
        LOG_WARN("[MOTOR] message_id 0x%03X exceeds max 0x3FF, clamped", message_id);
        message_id = 0x3FF;
    }

    return ((uint32_t)device_id << 12) | ((uint32_t)message_id);
}

/**
 * @brief       浮点数转字节数组 (小端序)
 * @param       value   : 浮点数值
 * @param       bytes   : 字节数组 (至少4字节)
 * @retval      无
 */
void motor_float_to_bytes(float value, uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } converter;

    converter.f = value;

    /* 小端序: 低字节在前 */
    bytes[0] = converter.b[0];
    bytes[1] = converter.b[1];
    bytes[2] = converter.b[2];
    bytes[3] = converter.b[3];
}

/**
 * @brief       字节数组转浮点数 (小端序)
 * @param       bytes   : 字节数组 (至少4字节)
 * @retval      浮点数值
 */
float motor_bytes_to_float(const uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } converter;

    /* 小端序: 低字节在前 */
    converter.b[0] = bytes[0];
    converter.b[1] = bytes[1];
    converter.b[2] = bytes[2];
    converter.b[3] = bytes[3];

    return converter.f;
}

/******************************************************************************************/
/* 基本控制命令实现 */

/**
 * @brief       使能/失能电机
 * @param       device_id   : 设备ID
 * @param       enable      : 0=失能, 1=使能
 * @retval      0=成功, 1=失败
 */
uint8_t motor_enable(uint8_t can, uint8_t device_id, uint8_t enable)
{
    uint8_t txdata[8] = {0};
    txdata[0] = enable ? 0x01 : 0x00;
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_ENABLE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_1;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: %s\r\n", device_id, enable ? "ENABLE" : "DISABLE");
    
    motor_cmd_track(device_id, MOTOR_MSG_ENABLE, (float)enable);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置运行模式
 * @param       device_id   : 设备ID
 * @param       mode        : 运行模式
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_mode(uint8_t can, uint8_t device_id, MotorMode_t mode)
{
    uint8_t txdata[8] = {0};
    txdata[0] = (uint8_t)mode;

    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_MODE_SET);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_1;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set mode to %s\r\n", device_id, motor_mode_to_string(mode));
    
    motor_cmd_track(device_id, MOTOR_MSG_MODE_SET, (float)mode);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       启动/停止电机
 * @param       device_id   : 设备ID
 * @param       start       : 1=停止, 0=启动
 * @retval      0=成功, 1=失败
 */
uint8_t motor_start_stop(uint8_t can, uint8_t device_id, uint8_t start)
{
    uint8_t txdata[8] = {0};
    /* 约定: start==1 -> 启动(0x00)，start==0 -> 停止(0x01) */
    txdata[0] = start ? 0x00 : 0x01;

    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_START_STOP);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_1;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: %s\r\n", device_id, start ? "START" : "STOP");
    
    motor_cmd_track(device_id, MOTOR_MSG_START_STOP, (float)start);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置零点
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_zero(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_ZERO);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set zero point\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       恢复出厂零点
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_restore_zero(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_RESTORE_ZERO);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Restore factory zero\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       保存控制参数至FLASH
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_save_ctrl_params(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SAVE_CTRL_PARAM);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Save control parameters to FLASH\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       保存限制参数至FLASH
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_save_limit_params(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SAVE_LIMIT_PARAM);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Save limit parameters to FLASH\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       保存软件零点至FLASH
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_save_soft_zero(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SAVE_SOFT_ZERO);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Save software zero point to FLASH\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取设备ID
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       dev_id      : 设备ID指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际设备ID会在motor_process_rx_message()中自动打印
 */
uint8_t motor_read_device_id(uint8_t can, uint8_t device_id, uint32_t *dev_id)
{
    uint8_t txdata[8] = {0};
    (void)dev_id;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_DEVICE_ID);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request device ID\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置设备ID
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 当前设备ID
 * @param       new_dev_id  : 新设备ID (0x00~0x7F)
 * @retval      0=成功, 1=失败
 * @note        设置后需要重新上电才能生效
 */
uint8_t motor_set_device_id(uint8_t can, uint8_t device_id, uint32_t new_dev_id)
{
    uint8_t txdata[8] = {0};
    /* 将32位设备ID转换为4字节小端序 */
    txdata[0] = (uint8_t)(new_dev_id & 0xFF);
    txdata[1] = (uint8_t)((new_dev_id >> 8) & 0xFF);
    txdata[2] = (uint8_t)((new_dev_id >> 16) & 0xFF);
    txdata[3] = (uint8_t)((new_dev_id >> 24) & 0xFF);

    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_DEVICE_ID);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set new device ID = 0x%08X\r\n", device_id, (unsigned int)new_dev_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       恢复默认参数
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 * @note        将所有参数恢复至出厂默认值
 */
uint8_t motor_restore_default(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_RESTORE_DEFAULT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Restore default parameters\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 目标值设置实现 */

/**
 * @brief       设置位置目标
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       position    : 位置目标 (°)
 * @retval      MotorErrorCode_t错误码
 */
uint8_t motor_set_position(uint8_t can, uint8_t device_id, float position)
{
    VALIDATE_CAN_CHANNEL(can);
    VALIDATE_DEVICE_ID(device_id);
    VALIDATE_PARAM(position);
    CHECK_EMERGENCY_STOP();
    
    ValidateResult_t validate = Security_ValidateMotorAngle(position);
    if (validate != VALIDATE_OK) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_INVALID_PARAM, "Position out of range");
        return MOTOR_ERR_INVALID_PARAM;
    }
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(position, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_POS_TARGET);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    motor_exec_record_target(device_id, position);

    uint8_t result = motor_send_by_idx(can, txdata, &config);
    if (result != 0) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_CAN_FAILURE, "CAN send failed");
    }
    return result;
}

/**
 * @brief       设置速度目标
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       speed       : 速度目标 (rpm)
 * @retval      MotorErrorCode_t错误码
 */
uint8_t motor_set_speed(uint8_t can, uint8_t device_id, float speed)
{
    VALIDATE_CAN_CHANNEL(can);
    VALIDATE_DEVICE_ID(device_id);
    VALIDATE_PARAM(speed);
    CHECK_EMERGENCY_STOP();
    
    ValidateResult_t validate = Security_ValidateMotorSpeed(speed);
    if (validate != VALIDATE_OK) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_INVALID_PARAM, "Speed out of range");
        return MOTOR_ERR_INVALID_PARAM;
    }
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(speed, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SPEED_TARGET);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    uint8_t result = motor_send_by_idx(can, txdata, &config);
    if (result != 0) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_CAN_FAILURE, "CAN send failed");
    }
    return result;
}

/**
 * @brief       设置转矩目标
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       torque      : 转矩目标 (Nm)
 * @retval      MotorErrorCode_t错误码
 */
uint8_t motor_set_torque(uint8_t can, uint8_t device_id, float torque)
{
    VALIDATE_CAN_CHANNEL(can);
    VALIDATE_DEVICE_ID(device_id);
    VALIDATE_PARAM(torque);
    CHECK_EMERGENCY_STOP();
    
    ValidateResult_t validate = Security_ValidateMotorTorque(torque);
    if (validate != VALIDATE_OK) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_INVALID_PARAM, "Torque out of range");
        return MOTOR_ERR_INVALID_PARAM;
    }
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(torque, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_TORQUE_TARGET);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    uint8_t result = motor_send_by_idx(can, txdata, &config);
    if (result != 0) {
        MOTOR_SET_ERROR(&g_motor_error_ctx, MOTOR_ERR_CAN_FAILURE, "CAN send failed");
    }
    return result;
}

/******************************************************************************************/
/* 控制环参数设置实现 */

/**
 * @brief       设置位置环Kp
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kp          : 位置环Kp参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_kp(uint8_t can, uint8_t device_id, float kp)
{
    VALIDATE_PARAM(kp);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kp, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_POS_KP);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Position Kp = %.3f\r\n", device_id, kp);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置位置环Ki
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       ki          : 位置环Ki参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_ki(uint8_t can, uint8_t device_id, float ki)
{
    VALIDATE_PARAM(ki);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(ki, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_POS_KI);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Position Ki = %.3f\r\n", device_id, ki);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置位置环Kd
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kd          : 位置环Kd参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_kd(uint8_t can, uint8_t device_id, float kd)
{
    VALIDATE_PARAM(kd);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kd, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_POS_KD);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Position Kd = %.3f\r\n", device_id, kd);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置速度环Kp
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kp          : 速度环Kp参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_speed_kp(uint8_t can, uint8_t device_id, float kp)
{
    VALIDATE_PARAM(kp);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kp, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_SPEED_KP);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Speed Kp = %.3f\r\n", device_id, kp);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置速度环Ki
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       ki          : 速度环Ki参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_speed_ki(uint8_t can, uint8_t device_id, float ki)
{
    VALIDATE_PARAM(ki);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(ki, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_SPEED_KI);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Speed Ki = %.3f\r\n", device_id, ki);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置速度环Kd
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kd          : 速度环Kd参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_speed_kd(uint8_t can, uint8_t device_id, float kd)
{
    VALIDATE_PARAM(kd);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kd, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_SPEED_KD);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Speed Kd = %.3f\r\n", device_id, kd);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置电流环Kp
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kp          : 电流环Kp参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_curr_kp(uint8_t can, uint8_t device_id, float kp)
{
    VALIDATE_PARAM(kp);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kp, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_CURR_KP);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    printf("[MOTOR] Device 0x%02X: Set Current Kp = %.3f\r\n", device_id, kp);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置电流环Ki
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       ki          : 电流环Ki参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_curr_ki(uint8_t can, uint8_t device_id, float ki)
{
    VALIDATE_PARAM(ki);
    
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(ki, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_CURR_KI);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Current Ki = %.3f\r\n", device_id, ki);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置电流环Kd
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       kd          : 电流环Kd参数
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_curr_kd(uint8_t can, uint8_t device_id, float kd)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(kd, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_CURR_KD);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Current Kd = %.3f\r\n", device_id, kd);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置位置环带宽
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       bandwidth   : 位置环带宽
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_bandwidth(uint8_t can, uint8_t device_id, float bandwidth)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(bandwidth, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_POS_BANDWIDTH);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Position Bandwidth = %.1f\r\n", device_id, bandwidth);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置速度环带宽
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       bandwidth   : 速度环带宽
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_speed_bandwidth(uint8_t can, uint8_t device_id, float bandwidth)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(bandwidth, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_SPEED_BANDWIDTH);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Speed Bandwidth = %.1f\r\n", device_id, bandwidth);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置电流环带宽
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       bandwidth   : 电流环带宽
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_curr_bandwidth(uint8_t can, uint8_t device_id, float bandwidth)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(bandwidth, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SET_CURR_BANDWIDTH);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set Current Bandwidth = %.1f\r\n", device_id, bandwidth);

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 限制参数设置实现 */

/**
 * @brief       设置限定速度
 * @param       device_id   : 设备ID
 * @param       speed       : 限定速度 (rpm)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_limit_speed(uint8_t can, uint8_t device_id, float speed)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(speed, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_LIMIT_SPEED);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置限定加速度
 * @param       device_id   : 设备ID
 * @param       accel       : 限定加速度 (rpm/s)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_limit_accel(uint8_t can, uint8_t device_id, float accel)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(accel, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_LIMIT_ACCEL);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置力矩斜率
 * @param       device_id   : 设备ID
 * @param       slope       : 力矩斜率 (Nm/s)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_torque_slope(uint8_t can, uint8_t device_id, float slope)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(slope, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_TORQUE_SLOPE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

#if 0
    LOG_RAW("[MOTOR] Device 0x%02X: Set torque slope = %.2f Nm/s\r\n", device_id, slope);
#endif

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 限幅参数设置实现 */

/**
 * @brief       设置位置上限
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       upper_limit : 位置上限 (°)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_upper_limit(uint8_t can, uint8_t device_id, float upper_limit)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(upper_limit, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_POS_UPPER_LIMIT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set position upper limit = %.2f deg\r\n", device_id, upper_limit);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置位置下限
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       lower_limit : 位置下限 (°)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_pos_lower_limit(uint8_t can, uint8_t device_id, float lower_limit)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(lower_limit, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_POS_LOWER_LIMIT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set position lower limit = %.2f deg\r\n", device_id, lower_limit);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置速度限幅
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       speed_limit : 速度限幅 (rpm)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_speed_limit(uint8_t can, uint8_t device_id, float speed_limit)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(speed_limit, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_SPEED_LIMIT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

#if 0
    LOG_RAW("[MOTOR] Device 0x%02X: Set speed limit = %.2f rpm\r\n", device_id, speed_limit);
#endif

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置扭矩限幅
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       torque_limit: 扭矩限幅 (Nm)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_torque_limit(uint8_t can, uint8_t device_id, float torque_limit)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(torque_limit, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_TORQUE_LIMIT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

#if 0
    LOG_RAW("[MOTOR] Device 0x%02X: Set torque limit = %.2f Nm\r\n", device_id, torque_limit);
#endif

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       设置电流限幅
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @param       current_limit: 电流限幅 (A)
 * @retval      0=成功, 1=失败
 */
uint8_t motor_set_current_limit(uint8_t can, uint8_t device_id, float current_limit)
{
    uint8_t txdata[8] = {0};
    motor_float_to_bytes(current_limit, txdata);
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_CURRENT_LIMIT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_4;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Set current limit = %.2f A\r\n", device_id, current_limit);

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 参数读取实现 */

/**
 * @brief       读取电机电阻
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作），返回值会在接收回调中处理
 */
uint8_t motor_read_resistance(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_RESISTANCE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request resistance\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取电机电感
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_inductance(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_INDUCTANCE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request inductance\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取峰值转速
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_peak_speed(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_PEAK_SPEED);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request peak speed\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取峰值扭矩
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_peak_torque(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_PEAK_TORQUE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request peak torque\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取额定转速
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_rated_speed(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_RATED_SPEED);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request rated speed\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取额定扭矩
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_rated_torque(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_RATED_TORQUE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request rated torque\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取额定电流
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_rated_current(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_RATED_CURRENT);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request rated current\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取额定电压
 * @param       can         : CAN通道 (1或2)
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_read_rated_voltage(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_RATED_VOLTAGE);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request rated voltage\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取监测数据 (9*4字节)
 * @param       device_id   : 设备ID
 * @param       data        : 监测数据结构体指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际数据需在CAN接收中断中调用motor_parse_monitor_data()解析
 *              或在motor_process_rx_message()中自动处理
 */
uint8_t motor_read_monitor_data(uint8_t can, uint8_t device_id, MotorMonitorData_t *data)
{
    uint8_t txdata[8] = {0};
    (void)data;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_MONITOR);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

#if 0
    LOG_RAW("[MOTOR] Device 0x%02X: Request monitor data\r\n", device_id);
#endif

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取电机固有参数 (11*4字节)
 * @param       device_id   : 设备ID
 * @param       params      : 参数结构体指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际数据需在CAN接收中断中调用motor_parse_motor_params()解析
 *              或在motor_process_rx_message()中自动处理
 */
uint8_t motor_read_motor_params(uint8_t can, uint8_t device_id, MotorParams_t *params)
{
    uint8_t txdata[8] = {0};
    (void)params;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_MOTOR_PARAMS);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request motor parameters\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取控制参数 (12*4字节)
 * @param       device_id   : 设备ID
 * @param       params      : 参数结构体指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际数据需在CAN接收中断中调用motor_parse_ctrl_params()解析
 *              或在motor_process_rx_message()中自动处理
 */
uint8_t motor_read_ctrl_params(uint8_t can, uint8_t device_id, MotorCtrlParams_t *params)
{
    uint8_t txdata[8] = {0};
    (void)params;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_CTRL_PARAMS);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request control parameters\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       读取限制参数 (5*4字节)
 * @param       device_id   : 设备ID
 * @param       params      : 参数结构体指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际数据需在CAN接收中断中调用motor_parse_limit_params()解析
 *              或在motor_process_rx_message()中自动处理
 */
uint8_t motor_read_limit_params(uint8_t can, uint8_t device_id, MotorLimitParams_t *params)
{
    uint8_t txdata[8] = {0};
    (void)params;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_READ_LIMIT_PARAMS);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request limit parameters\r\n", device_id);

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 错误处理实现 */

/**
 * @brief       清除错误码
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 */
uint8_t motor_clear_error(uint8_t can, uint8_t device_id)
{
    uint8_t txdata[8] = {0};
    txdata[0] = 0x00;  /* 写0清除故障 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_ERROR);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_1;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Clear error\r\n", device_id);

    /* 标记为清除命令 */
    if (device_id < 128) {
        g_error_cmd_pending[device_id] = 1;
    }

    return motor_send_by_idx(can, txdata, &config);
}

/**
 * @brief       获取错误码
 * @param       device_id   : 设备ID
 * @param       error       : 错误码指针 (预留参数，当前未使用)
 * @retval      0=成功, 1=失败
 * @note        此函数仅发送读取请求（异步操作）
 *              实际错误码会在motor_process_rx_message()中自动打印
 */
uint8_t motor_get_error(uint8_t can, uint8_t device_id, MotorError_t *error)
{
    uint8_t txdata[8] = {0};
    (void)error;  /* 参数预留用于未来扩展，避免编译器警告 */
    FDCAN_TxConfigTypeDef config = {0};
    config.identifier = motor_build_ext_id(device_id, MOTOR_MSG_ERROR);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.data_length = FDCAN_DLC_BYTES_0;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;

    LOG_RAW("[MOTOR] Device 0x%02X: Request error status\r\n", device_id);

    /* 标记为查询命令 */
    if (device_id < 128) {
        g_error_cmd_pending[device_id] = 0;
    }

    return motor_send_by_idx(can, txdata, &config);
}

/******************************************************************************************/
/* 初始化与控制流程实现 */

/**
 * @brief       电机初始化序列
 * @param       device_id   : 设备ID
 * @param       mode        : 运行模式
 * @retval      0=成功, 1=失败
 * @note        按照协议文档推荐的顺序: 使能 -> 设置模式 -> 启动
 */
uint8_t motor_init_sequence(uint8_t can, uint8_t device_id, MotorMode_t mode)
{
    LOG_RAW("\r\n[MOTOR] ===== Starting motor initialization sequence =====\r\n");

    if (motor_enable(can, device_id, 1) != 0) {
        LOG_RAW("[MOTOR] ERROR: Failed to enable motor\r\n");
        return 1;
    }
    HAL_Delay(100);

    if (motor_set_mode(can, device_id, mode) != 0) {
        LOG_RAW("[MOTOR] ERROR: Failed to set motor mode\r\n");
        return 1;
    }
    HAL_Delay(100);

    if (motor_start_stop(can, device_id, 1) != 0) {
        LOG_RAW("[MOTOR] ERROR: Failed to start motor\r\n");
        return 1;
    }
    HAL_Delay(100);

    LOG_RAW("[MOTOR] ===== Motor initialization completed successfully =====\r\n\r\n");
    return 0;
}

/**
 * @brief       电机关闭序列
 * @param       device_id   : 设备ID
 * @retval      0=成功, 1=失败
 * @note        按照安全顺序: 停止 -> 失能
 */
uint8_t motor_shutdown_sequence(uint8_t can, uint8_t device_id)
{
    LOG_RAW("\r\n[MOTOR] ===== Starting motor shutdown sequence =====\r\n");

    if (motor_start_stop(can, device_id, 0) != 0) {
        LOG_RAW("[MOTOR] ERROR: Failed to stop motor\r\n");
        return 1;
    }
    HAL_Delay(200);

    if (motor_enable(can, device_id, 0) != 0) {
        LOG_RAW("[MOTOR] ERROR: Failed to disable motor\r\n");
        return 1;
    }
    HAL_Delay(100);

    LOG_RAW("[MOTOR] ===== Motor shutdown completed successfully =====\r\n\r\n");
    return 0;
}

/******************************************************************************************/
/* 状态管理与打印实现 */

/**
 * @brief       打印监测数据
 * @param       data        : 监测数据结构体指针
 * @retval      无
 */
void motor_print_monitor_data(MotorMonitorData_t *data)
{
    LOG_RAW("\r\n[MOTOR] ===== Monitor Data =====\r\n");
    LOG_RAW("  Position:    Target=%.2f deg, Actual=%.2f deg\r\n",
           data->target_position, data->actual_position);
    LOG_RAW("  Speed:       Target=%.2f rpm, Actual=%.2f rpm\r\n",
           data->target_speed, data->actual_speed);
    LOG_RAW("  Torque:      Target=%.2f Nm,  Actual=%.2f Nm\r\n",
           data->target_torque, data->actual_torque);
    LOG_RAW("  Temperature: Motor=%.1f C,    Board=%.1f C\r\n",
           data->motor_temperature, data->board_temperature);
    LOG_RAW("  Bus Voltage: %.1f V\r\n", data->bus_voltage);
    LOG_RAW("================================\r\n\r\n");
}

/**
 * @brief       打印电机状态
 * @param       device_id   : 设备ID
 * @param       status      : 状态结构体指针
 * @retval      无
 */
void motor_print_status(uint8_t device_id, MotorStatus_t *status)
{
    const char* error_str[] = {
        "NONE", "OVERVOLTAGE", "OVERCURRENT",
        "MOTOR_OVERTEMP", "BOARD_OVERTEMP", "UNDERVOLTAGE"
    };

    LOG_RAW("\r\n[MOTOR] ===== Status of Device 0x%02X =====\r\n", device_id);
    LOG_RAW("  Enabled:     %s\r\n", status->enabled ? "YES" : "NO");
    LOG_RAW("  Running:     %s\r\n", status->running ? "YES" : "NO");
    LOG_RAW("  Mode:        %s\r\n", motor_mode_to_string(status->mode));
    LOG_RAW("  Error:       %s\r\n", status->error_code < 6 ? error_str[status->error_code] : "UNKNOWN");
    LOG_RAW("=========================================\r\n\r\n");
}

/**
 * @brief       更新并缓存设备状态
 * @param       device_id   : 设备ID
 * @param       status      : 状态结构体指针（来自设备或上层）
 * @retval      无
 * @note        会把状态复制到本地缓存并打印简要信息；若注册了错误回调且状态含错误，会回调。
 */
void motor_update_status(uint8_t device_id, MotorStatus_t *status)
{
    if (status == NULL) return;
    if (device_id >= 128) {
        LOG_RAW("[MOTOR] Warning: device_id 0x%02X out of range for status update\r\n", device_id);
        return;
    }

    /* ============== 修复: 更新缓存并记录时间戳 ============== */
    g_motor_status_cache[device_id].status = *status;
    g_motor_status_cache[device_id].last_update_tick = HAL_GetTick();
    g_motor_status_cache[device_id].valid = true;

    /* 打印简要状态 */
    LOG_RAW("[MOTOR] Status updated Device=0x%02X Enabled=%u Running=%u Mode=%s Error=%u\r\n",
           device_id, status->enabled, status->running, motor_mode_to_string(status->mode), status->error_code);

    /* 如有错误且回调已注册，调用回调 */
    if (status->error_code != MOTOR_ERROR_NONE && g_motor_error_callback != NULL) {
        g_motor_error_callback(device_id, status->error_code);
    }
}

/**
 * @brief       检查电机状态缓存是否有效（带超时检测）
 * @param       device_id   : 设备ID
 * @param       status      : 状态结构体指针（输出）
 * @retval      true=有效, false=无效/超时
 */
bool motor_get_status_with_timeout(uint8_t device_id, MotorStatus_t *status)
{
    if (device_id >= 128 || status == NULL) {
        return false;
    }
    
    MotorStatusCache_t *cache = &g_motor_status_cache[device_id];
    
    /* 检查是否有效 */
    if (!cache->valid) {
        return false;
    }
    
    /* 检查是否超时 */
    uint32_t elapsed = HAL_GetTick() - cache->last_update_tick;
    if (elapsed > MOTOR_STATUS_TIMEOUT_MS) {
        cache->valid = false;
        LOG_RAW("[MOTOR] Status timeout for device 0x%02X (%u ms)\r\n", device_id, elapsed);
        return false;
    }
    
    *status = cache->status;
    return true;
}

/**
 * @brief       解析监测数据 (从CAN接收数据中提取)
 * @param       rxdata      : 接收的CAN数据 (至少36字节)
 * @param       data        : 监测数据结构体指针
 * @retval      无
 */
void motor_parse_monitor_data(const uint8_t *rxdata, MotorMonitorData_t *data)
{
    if (rxdata == NULL || data == NULL) {
        LOG_RAW("[MOTOR] ERROR: NULL pointer in motor_parse_monitor_data\r\n");
        return;
    }

    data->target_position     = motor_bytes_to_float(&rxdata[0]);
    data->actual_position     = motor_bytes_to_float(&rxdata[4]);
    data->target_speed        = motor_bytes_to_float(&rxdata[8]);
    data->actual_speed        = motor_bytes_to_float(&rxdata[12]);
    data->target_torque       = motor_bytes_to_float(&rxdata[16]);
    data->actual_torque       = motor_bytes_to_float(&rxdata[20]);
    data->motor_temperature   = motor_bytes_to_float(&rxdata[24]);
    data->board_temperature   = motor_bytes_to_float(&rxdata[28]);
    data->bus_voltage         = motor_bytes_to_float(&rxdata[32]);
}

/**
 * @brief       解析电机固有参数 (11*4字节)
 * @param       rxdata      : 接收的CAN数据
 * @param       params      : 参数结构体指针
 * @retval      无
 */
void motor_parse_motor_params(const uint8_t *rxdata, MotorParams_t *params)
{
    if (rxdata == NULL || params == NULL) {
        LOG_RAW("[MOTOR] ERROR: NULL pointer in motor_parse_motor_params\r\n");
        return;
    }

    params->resistance     = motor_bytes_to_float(&rxdata[0]);
    params->inductance     = motor_bytes_to_float(&rxdata[4]);
    params->peak_speed     = motor_bytes_to_float(&rxdata[8]);
    params->peak_torque    = motor_bytes_to_float(&rxdata[12]);
    params->rated_speed    = motor_bytes_to_float(&rxdata[16]);
    params->rated_torque   = motor_bytes_to_float(&rxdata[20]);
    params->rated_current  = motor_bytes_to_float(&rxdata[24]);
    params->rated_voltage  = motor_bytes_to_float(&rxdata[28]);
    params->inertia        = motor_bytes_to_float(&rxdata[32]);
    params->peak_current   = motor_bytes_to_float(&rxdata[36]);
    params->gear_ratio     = motor_bytes_to_float(&rxdata[40]);

    LOG_RAW("[MOTOR] Motor Parameters Parsed:\r\n");
    LOG_RAW("  Resistance: %.3f Ohm\r\n", params->resistance);
    LOG_RAW("  Inductance: %.6f H\r\n", params->inductance);
    LOG_RAW("  Peak Speed: %.1f rpm\r\n", params->peak_speed);
    LOG_RAW("  Rated Speed: %.1f rpm\r\n", params->rated_speed);
    LOG_RAW("  Peak Torque: %.2f Nm\r\n", params->peak_torque);
    LOG_RAW("  Gear Ratio: %.2f\r\n", params->gear_ratio);
}

/**
 * @brief       解析控制参数 (12*4字节)
 * @param       rxdata      : 接收的CAN数据
 * @param       params      : 参数结构体指针
 * @retval      无
 */
void motor_parse_ctrl_params(const uint8_t *rxdata, MotorCtrlParams_t *params)
{
    if (rxdata == NULL || params == NULL) {
        LOG_RAW("[MOTOR] ERROR: NULL pointer in motor_parse_ctrl_params\r\n");
        return;
    }

    params->pos_kp            = motor_bytes_to_float(&rxdata[0]);
    params->pos_ki            = motor_bytes_to_float(&rxdata[4]);
    params->pos_kd            = motor_bytes_to_float(&rxdata[8]);
    params->speed_kp          = motor_bytes_to_float(&rxdata[12]);
    params->speed_ki          = motor_bytes_to_float(&rxdata[16]);
    params->speed_kd          = motor_bytes_to_float(&rxdata[20]);
    params->curr_kp           = motor_bytes_to_float(&rxdata[24]);
    params->curr_ki           = motor_bytes_to_float(&rxdata[28]);
    params->curr_kd           = motor_bytes_to_float(&rxdata[32]);
    params->pos_bandwidth     = motor_bytes_to_float(&rxdata[36]);
    params->speed_bandwidth   = motor_bytes_to_float(&rxdata[40]);
    params->curr_bandwidth    = motor_bytes_to_float(&rxdata[44]);

    LOG_RAW("[MOTOR] Control Parameters Parsed:\r\n");
    LOG_RAW("  Position Loop: Kp=%.3f, Ki=%.3f, Kd=%.3f, BW=%.1f\r\n",
           params->pos_kp, params->pos_ki, params->pos_kd, params->pos_bandwidth);
    LOG_RAW("  Speed Loop: Kp=%.3f, Ki=%.3f, Kd=%.3f, BW=%.1f\r\n",
           params->speed_kp, params->speed_ki, params->speed_kd, params->speed_bandwidth);
    LOG_RAW("  Current Loop: Kp=%.3f, Ki=%.3f, Kd=%.3f, BW=%.1f\r\n",
           params->curr_kp, params->curr_ki, params->curr_kd, params->curr_bandwidth);
}

/**
 * @brief       解析限制参数 (5*4字节)
 * @param       rxdata      : 接收的CAN数据
 * @param       params      : 参数结构体指针
 * @retval      无
 */
void motor_parse_limit_params(const uint8_t *rxdata, MotorLimitParams_t *params)
{
    if (rxdata == NULL || params == NULL) {
        LOG_RAW("[MOTOR] ERROR: NULL pointer in motor_parse_limit_params\r\n");
        return;
    }

    params->pos_upper_limit   = motor_bytes_to_float(&rxdata[0]);
    params->pos_lower_limit   = motor_bytes_to_float(&rxdata[4]);
    params->speed_limit       = motor_bytes_to_float(&rxdata[8]);
    params->torque_limit      = motor_bytes_to_float(&rxdata[12]);
    params->current_limit     = motor_bytes_to_float(&rxdata[16]);

    LOG_RAW("[MOTOR] Limit Parameters Parsed:\r\n");
    LOG_RAW("  Position: %.1f ~ %.1f deg\r\n", params->pos_lower_limit, params->pos_upper_limit);
    LOG_RAW("  Speed: %.1f rpm\r\n", params->speed_limit);
    LOG_RAW("  Torque: %.2f Nm\r\n", params->torque_limit);
    LOG_RAW("  Current: %.2f A\r\n", params->current_limit);
}

/**
 * @brief       处理接收的CAN消息 (在CAN接收回调中调用)
 * @param       rx_id       : 接收到的CAN ID (29位扩展ID)
 * @param       rxdata      : 接收的数据
 * @param       data_len    : 数据长度
 * @retval      无
 * @note        根据消息ID解析不同类型的返回数据
 */
void motor_process_rx_message(uint32_t rx_id, const uint8_t *rxdata, uint8_t data_len)
{
    if (rxdata == NULL) {
        return;
    }

    /* 将接收到的消息打包并入队（ISR/回调安全，非阻塞） */
    typedef struct {
        uint32_t rx_id;
        uint8_t len;
        uint8_t data[64];
    } MotorRxEvent_t;

    MotorRxEvent_t evt;
    evt.rx_id = rx_id;
    evt.len = (data_len > 64) ? 64 : data_len;
    memcpy(evt.data, rxdata, evt.len);

    if (motor_rx_queue != NULL) {
        osStatus_t st = osMessageQueuePut(motor_rx_queue, &evt, 0, 0); /* non-blocking */
        g_msg_total_count++;
        if (st != osOK) {
            g_msg_queue_full_count++;
        }
    }
}

/* Motor RX processing task - 在任务上下文中处理并打印 */
static void MotorRxTask(void *argument)
{
    (void)argument;
    typedef struct {
        uint32_t rx_id;
        uint8_t len;
        uint8_t data[64];
    } MotorRxEvent_t;

    MotorRxEvent_t evt;
    for (;;) {
        if (motor_rx_queue != NULL) {
            osStatus_t st = osMessageQueueGet(motor_rx_queue, &evt, NULL, osWaitForever);
            if (st == osOK) {
                uint8_t device_id = (evt.rx_id >> 12) & 0x7F;
                uint16_t message_id = evt.rx_id & 0x3FF;
                uint8_t can_channel = (device_id <= 6) ? 1 : 2;  /* CAN1: ID 1-6, CAN2: ID 7-12 */
                uint8_t *rxdata = evt.data;
                uint8_t data_len = evt.len;

                printf("[MOTOR-RX] Device=0x%02X, Msg=0x%03X, Len=%d\r\n", device_id, message_id, data_len);

                switch (message_id) {
                    case MOTOR_MSG_ENABLE:
                    case MOTOR_MSG_MODE_SET:
                        if (data_len >= 1) {
                            printf("[MOTOR-RX] Command result: %s\r\n", rxdata[0] ? "SUCCESS" : "FAILED");
                            motor_cmd_handle_response(device_id, message_id, rxdata[0] ? 0 : 1);
                        }
                        break;
                    case MOTOR_MSG_SET_ZERO:
                    case MOTOR_MSG_RESTORE_ZERO:
                    case MOTOR_MSG_SAVE_CTRL_PARAM:
                    case MOTOR_MSG_SAVE_LIMIT_PARAM:
                    case MOTOR_MSG_SAVE_SOFT_ZERO:
                    case MOTOR_MSG_RESTORE_DEFAULT:
                    /* PID控制参数设置返回 */
                    case MOTOR_MSG_SET_POS_KP:
                    case MOTOR_MSG_SET_POS_KI:
                    case MOTOR_MSG_SET_POS_KD:
                    case MOTOR_MSG_SET_SPEED_KP:
                    case MOTOR_MSG_SET_SPEED_KI:
                    case MOTOR_MSG_SET_SPEED_KD:
                    case MOTOR_MSG_SET_CURR_KP:
                    case MOTOR_MSG_SET_CURR_KI:
                    case MOTOR_MSG_SET_CURR_KD:
                    case MOTOR_MSG_SET_POS_BANDWIDTH:
                    case MOTOR_MSG_SET_SPEED_BANDWIDTH:
                    case MOTOR_MSG_SET_CURR_BANDWIDTH:
                    /* 限幅参数设置返回 */
                    case MOTOR_MSG_POS_UPPER_LIMIT:
                    case MOTOR_MSG_POS_LOWER_LIMIT:
                    case MOTOR_MSG_SPEED_LIMIT:
                    case MOTOR_MSG_TORQUE_LIMIT:
                    case MOTOR_MSG_CURRENT_LIMIT:
                    /* 目标值设置返回 (无返回值指令) */
                    case MOTOR_MSG_POS_TARGET:
                    case MOTOR_MSG_SPEED_TARGET:
                    case MOTOR_MSG_TORQUE_TARGET:
                    /* 限制参数设置返回 (无返回值指令) */
                    case MOTOR_MSG_LIMIT_SPEED:
                    case MOTOR_MSG_LIMIT_ACCEL:
                    case MOTOR_MSG_TORQUE_SLOPE:
                        if (data_len >= 1) printf("[MOTOR-RX] Command result: %s\r\n", rxdata[0] ? "SUCCESS" : "FAILED");
                        break;
                    case MOTOR_MSG_START_STOP:
                        if (data_len >= 1 && device_id >= 1 && device_id <= 128) {
                            uint8_t motor_idx = device_id - 1;
                            switch (rxdata[0]) {
                                case 0x00: 
                                    printf("[MOTOR-RX] START/STOP: FAILED\r\n");
                                    motor_cmd_handle_response(device_id, MOTOR_MSG_START_STOP, 1);
                                    motor_handle_update_running(motor_idx, 0, 0);
                                    break;
                                case 0x01: 
                                    printf("[MOTOR-RX] START/STOP: SUCCESS\r\n");
                                    motor_cmd_handle_response(device_id, MOTOR_MSG_START_STOP, 0);
                                    motor_handle_update_running(motor_idx, 1, 1);
                                    break;
                                case 0x02: 
                                    printf("[MOTOR-RX] START/STOP: Software zero point activated\r\n");
                                    motor_cmd_handle_response(device_id, MOTOR_MSG_START_STOP, 0);
                                    motor_handle_update_running(motor_idx, 1, 1);
                                    break;
                                default: printf("[MOTOR-RX] START/STOP: Unknown result 0x%02X\r\n", rxdata[0]); break;
                            }
                        } else {
                            printf("[MOTOR-RX] START/STOP: Invalid device_id=%u or data_len=%u\r\n", device_id, data_len);
                        }
                        break;
                    case MOTOR_MSG_ERROR:
                        if (data_len >= 1) {
                            if (device_id < 128 && g_error_cmd_pending[device_id] == 1) {
                                if (rxdata[0] == 0x00) printf("[MOTOR-RX] Clear error result: FAILED\r\n");
                                else if (rxdata[0] == 0x01) printf("[MOTOR-RX] Clear error result: SUCCESS\r\n");
                                else printf("[MOTOR-RX] Clear error result: Unknown 0x%02X\r\n", rxdata[0]);
                                g_error_cmd_pending[device_id] = 0;
                            } else if (device_id >= 1 && device_id <= 128) {
                                const char* error_str[] = {"NONE","OVERVOLTAGE","OVERCURRENT","MOTOR_OVERTEMP","BOARD_OVERTEMP","UNDERVOLTAGE"};
                                uint8_t error_code = rxdata[0];
                                printf("[MOTOR-RX] Error Code: 0x%02X (%s)\r\n", error_code, error_code < 6 ? error_str[error_code] : "UNKNOWN");
                                
                                /* 更新到motor_control_enhanced的句柄中 */
                                motor_handle_update_error(device_id - 1, (MotorError_t)error_code);
                                
                                if (error_code != MOTOR_ERROR_NONE && g_motor_error_callback != NULL) {
                                    g_motor_error_callback(device_id, (MotorError_t)error_code);
                                }
                            } else {
                                printf("[MOTOR-RX] Error: Invalid device_id=%u\r\n", device_id);
                            }
                        }
                        break;
                    case MOTOR_MSG_DEVICE_ID:
                        if (data_len >= 4) {
                            uint32_t dev_id = (rxdata[3] << 24) | (rxdata[2] << 16) | (rxdata[1] << 8) | rxdata[0];
                            printf("[MOTOR-RX] Device ID: 0x%08X\r\n", dev_id);
                        }
                        break;
                    case MOTOR_MSG_MONITOR:
                        if (data_len >= 36) {
                            MotorMonitorData_t monitor_data;
                            motor_parse_monitor_data(rxdata, &monitor_data);
                            motor_print_monitor_data(&monitor_data);
                            
                            /* 更新到motor_control_enhanced的句柄中 */
                            motor_process_monitor_feedback(can_channel, device_id, rxdata, data_len);
                        } else {
                            printf("[MOTOR-RX] Warning: Monitor data length insufficient (%d < 36)\r\n", data_len);
                        }
                        break;
                    case MOTOR_MSG_READ_MOTOR_PARAMS:
                        if (data_len >= 44) {
                            MotorParams_t motor_params;
                            motor_parse_motor_params(rxdata, &motor_params);
                        } else printf("[MOTOR-RX] Warning: Motor params length insufficient (%d < 44)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_CTRL_PARAMS:
                        if (data_len >= 48) {
                            MotorCtrlParams_t ctrl_params;
                            motor_parse_ctrl_params(rxdata, &ctrl_params);
                        } else printf("[MOTOR-RX] Warning: Control params length insufficient (%d < 48)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_LIMIT_PARAMS:
                        if (data_len >= 20) {
                            MotorLimitParams_t limit_params;
                            motor_parse_limit_params(rxdata, &limit_params);
                        } else printf("[MOTOR-RX] Warning: Limit params length insufficient (%d < 20)\r\n", data_len);
                        break;
                    /* 单个参数读取返回 (4字节float) */
                    case MOTOR_MSG_READ_RESISTANCE:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Resistance = %.3f Ohm\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Resistance data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_INDUCTANCE:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Inductance = %.6f H\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Inductance data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_PEAK_SPEED:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Peak Speed = %.1f rpm\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Peak speed data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_PEAK_TORQUE:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Peak Torque = %.2f Nm\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Peak torque data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_RATED_SPEED:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Rated Speed = %.1f rpm\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Rated speed data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_RATED_TORQUE:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Rated Torque = %.2f Nm\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Rated torque data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_RATED_CURRENT:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Rated Current = %.2f A\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Rated current data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    case MOTOR_MSG_READ_RATED_VOLTAGE:
                        if (data_len >= 4) {
                            float value = motor_bytes_to_float(rxdata);
                            printf("[MOTOR-RX] Rated Voltage = %.1f V\r\n", value);
                        } else printf("[MOTOR-RX] Warning: Rated voltage data length insufficient (%d < 4)\r\n", data_len);
                        break;
                    default:
                        printf("[MOTOR-RX] Unknown message ID: 0x%03X\r\n", message_id);
                        g_msg_unknown_count++;
                        break;
                }
                
                /* 成功处理计数 (仅在非默认case时增加) */
                g_msg_processed_count++;
            }
        }
    }
}

/* 初始化 motor 控制子系统 (创建队列/任务) */
/* 修复: 定义统一的消息结构体大小 */
#define MOTOR_RX_MSG_SIZE    (sizeof(uint32_t) + 1 + 64)  /* rx_id + len + data[64] */

void motor_control_init(void)
{
    motor_cmd_init();
    motor_exec_stats_init();
    
    if (motor_rx_queue == NULL) {
        motor_rx_queue = osMessageQueueNew(MOTOR_RX_QUEUE_SIZE, MOTOR_RX_MSG_SIZE, NULL);
    }
    if (motor_rx_task_handle == NULL) {
        const osThreadAttr_t motor_rx_task_attr = {
            .name = "MotorRx",
            .stack_size = 512 * 4,
            .priority = (osPriority_t) osPriorityAboveNormal,
        };
        motor_rx_task_handle = osThreadNew(MotorRxTask, NULL, &motor_rx_task_attr);
    }
}

void motor_control_deinit(void)
{
    if (motor_rx_task_handle != NULL) {
        osThreadTerminate(motor_rx_task_handle);
        motor_rx_task_handle = NULL;
    }
    if (motor_rx_queue != NULL) {
        osMessageQueueDelete(motor_rx_queue);
        motor_rx_queue = NULL;
    }
}

/******************************************************************************************/
/* 回调管理实现 */

/**
 * @brief       注册电机错误回调函数
 * @param       callback    : 回调函数指针 (NULL表示取消注册)
 * @retval      无
 * @note        当电机主动上报错误或查询到非零错误码时，会调用此回调
 */
void motor_register_error_callback(motor_error_callback_t callback)
{
    g_motor_error_callback = callback;
    printf("[MOTOR] Error callback %s\r\n", callback ? "registered" : "unregistered");
}

/******************************************************************************************/
/* 批量控制函数实现 */

/**
 * @brief       启用所有电机
 * @retval      无
 * @note        依次启用12个电机（ID: 0x01-0x0C）
 */
void motor_control_enable_all(void)
{
    printf("[MOTOR] Enabling all motors...\r\n");

    /* 电机分布：FDCAN1控制前6个，FDCAN2控制后6个 */
    for (uint8_t i = 1; i <= 6; i++) {
        motor_enable(1, i, 1);  /* FDCAN1: 0x01-0x06 */
    }

    osDelay(1);  /* 1ms总线恢复时间 */

    for (uint8_t i = 7; i <= 12; i++) {
        motor_enable(2, i, 1);  /* FDCAN2: 0x07-0x0C */
    }

    printf("[MOTOR] All motors enabled\r\n");
}

/**
 * @brief       禁用所有电机
 * @retval      无
 * @note        依次禁用12个电机（ID: 0x01-0x0C）
 */
void motor_control_disable_all(void)
{
    printf("[MOTOR] Disabling all motors...\r\n");

    for (uint8_t i = 1; i <= 6; i++) {
        motor_enable(1, i, 0);  /* FDCAN1: 0x01-0x06，0=禁用 */
    }

    osDelay(1);  /* 1ms总线恢复时间 */

    for (uint8_t i = 7; i <= 12; i++) {
        motor_enable(2, i, 0);  /* FDCAN2: 0x07-0x0C，0=禁用 */
    }

    printf("[MOTOR] All motors disabled\r\n");
}

/**
 * @brief       停止所有电机（保持使能）
 * @retval      无
 * @note        依次停止12个电机，电机保持使能状态
 */
void motor_control_stop_all(void)
{
    printf("[MOTOR] Stopping all motors...\r\n");

    for (uint8_t i = 1; i <= 6; i++) {
        motor_start_stop(1, i, 0);  /* FDCAN1: 0x01-0x06，0=停止 */
        osDelay(5);
    }

    for (uint8_t i = 7; i <= 12; i++) {
        motor_start_stop(2, i, 0);  /* FDCAN2: 0x07-0x0C，0=停止 */
        osDelay(5);
    }

    printf("[MOTOR] All motors stopped\r\n");
}

/**
 * @brief       紧急停止所有电机
 * @retval      无
 * @note        先停止所有电机，然后禁用所有电机
 */
void motor_control_emergency_stop_all(void)
{
    LOG_RAW("[MOTOR] *** EMERGENCY STOP ***\r\n");

    watchdog_feed_iwdg();

    /* P0-7修复: 去掉所有osDelay，并行发送所有停止命令
     * 原代码逐个发送+delay(2ms)，12电机总计62ms，机器人可能已摔倒
     * 现在先快速发送所有停止命令（无delay），再发送所有失能命令 */
    for (uint8_t i = 1; i <= 6; i++) {
        motor_start_stop(1, i, 0);
    }
    for (uint8_t i = 7; i <= 12; i++) {
        motor_start_stop(2, i, 0);
    }

    watchdog_feed_iwdg();

    /* 失能所有电机 */
    for (uint8_t i = 1; i <= 6; i++) {
        motor_enable(1, i, 0);
    }
    for (uint8_t i = 7; i <= 12; i++) {
        motor_enable(2, i, 0);
    }

    watchdog_feed_iwdg();

    LOG_RAW("[MOTOR] Emergency stop completed\r\n");
}

/**
 * @brief       切断电机电源（硬件层面）
 * @retval      无
 * @note        紧急情况下切断电机驱动电源，需要根据实际硬件配置GPIO
 */
void motor_control_power_cut(void)
{
    LOG_RAW("[MOTOR] *** POWER CUT ***\r\n");

#if defined(MOTOR_POWER_EN_GPIO_Port) && defined(MOTOR_POWER_EN_Pin)
    HAL_GPIO_WritePin(MOTOR_POWER_EN_GPIO_Port, MOTOR_POWER_EN_Pin, GPIO_PIN_RESET);
    LOG_RAW("[MOTOR] Power cut: GPIO pin set to LOW\r\n");
#else
    static bool power_cut_warning_shown = false;
    if (!power_cut_warning_shown) {
        LOG_RAW("[MOTOR] WARNING: MOTOR_POWER_EN not defined, using software stop instead\r\n");
        LOG_RAW("[MOTOR] Please define MOTOR_POWER_EN_GPIO_Port and MOTOR_POWER_EN_Pin in hardware config\r\n");
        power_cut_warning_shown = true;
    }
    motor_control_emergency_stop_all();
#endif
}

/**
 * @brief       打印消息处理统计信息
 * @retval      无
 */
void motor_print_rx_statistics(void)
{
    printf("\n========== Motor RX Statistics ==========\r\n");
    printf("  Total Messages:    %u\r\n", (unsigned int)g_msg_total_count);
    printf("  Processed:         %u\r\n", (unsigned int)g_msg_processed_count);
    printf("  Unknown Messages: %u\r\n", (unsigned int)g_msg_unknown_count);
    printf("  Queue Full:       %u\r\n", (unsigned int)g_msg_queue_full_count);
    if (g_msg_total_count > 0) {
        printf("  Success Rate:     %.1f%%\r\n", 
               100.0f * (float)g_msg_processed_count / (float)g_msg_total_count);
    }
    printf("=========================================\r\n");
}

/**
 * @brief       重置消息处理统计
 * @retval      无
 */
void motor_reset_rx_statistics(void)
{
    g_msg_total_count = 0;
    g_msg_processed_count = 0;
    g_msg_unknown_count = 0;
    g_msg_queue_full_count = 0;
    printf("[MOTOR] RX statistics reset\r\n");
}

/******************************************************************************************/
/* 命令确认与重试机制实现 */

static MotorCommand_t g_cmd_queue[MOTOR_CMD_QUEUE_SIZE];
static MotorCmdStats_t g_cmd_stats = {0};
static uint8_t g_cmd_head = 0;
static uint8_t g_cmd_tail = 0;

void motor_cmd_init(void)
{
    memset(g_cmd_queue, 0, sizeof(g_cmd_queue));
    memset(&g_cmd_stats, 0, sizeof(g_cmd_stats));
    g_cmd_head = 0;
    g_cmd_tail = 0;
    printf("[MOTOR-CMD] Command tracking initialized\r\n");
}

uint8_t motor_cmd_track(uint8_t device_id, uint16_t message_id, float target_value)
{
    uint8_t next_head = (g_cmd_head + 1) % MOTOR_CMD_QUEUE_SIZE;
    
    if (next_head == g_cmd_tail) {
        LOG_RAW("[MOTOR-CMD] Warning: Command queue full!\r\n");
        return 1;
    }
    
    g_cmd_queue[g_cmd_head].device_id = device_id;
    g_cmd_queue[g_cmd_head].message_id = message_id;
    g_cmd_queue[g_cmd_head].target_value = target_value;
    g_cmd_queue[g_cmd_head].send_tick = HAL_GetTick();
    g_cmd_queue[g_cmd_head].retry_count = 0;
    g_cmd_queue[g_cmd_head].acknowledged = 0;
    
    g_cmd_head = next_head;
    g_cmd_stats.total_commands++;
    
    return 0;
}

void motor_cmd_handle_response(uint8_t device_id, uint16_t message_id, uint8_t result)
{
    uint8_t idx = g_cmd_tail;
    
    while (idx != g_cmd_head) {
        if (g_cmd_queue[idx].device_id == device_id && 
            g_cmd_queue[idx].message_id == message_id &&
            g_cmd_queue[idx].acknowledged == 0) {
            
            g_cmd_queue[idx].acknowledged = 1;
            
            if (result == 0) {
                g_cmd_stats.success_count++;
                printf("[MOTOR-CMD] Device 0x%02X Cmd 0x%03X: SUCCESS\r\n", device_id, message_id);
            } else {
                g_cmd_stats.failed_count++;
                printf("[MOTOR-CMD] Device 0x%02X Cmd 0x%03X: FAILED\r\n", device_id, message_id);
            }
            
            while (g_cmd_tail != g_cmd_head && g_cmd_queue[g_cmd_tail].acknowledged) {
                g_cmd_tail = (g_cmd_tail + 1) % MOTOR_CMD_QUEUE_SIZE;
            }
            return;
        }
        idx = (idx + 1) % MOTOR_CMD_QUEUE_SIZE;
    }
}

void motor_cmd_process_timeout(void)
{
    uint32_t current_tick = HAL_GetTick();
    uint8_t processed = 0;
    
    for (uint8_t i = 0; i < MOTOR_CMD_QUEUE_SIZE; i++) {
        if (g_cmd_queue[i].acknowledged) continue;
        if (g_cmd_queue[i].device_id == 0) continue;
        
        uint32_t elapsed = current_tick - g_cmd_queue[i].send_tick;
        
        if (MOTOR_CMD_TIMEOUT_MS > 0 && elapsed > MOTOR_CMD_TIMEOUT_MS) {
            if (g_cmd_queue[i].retry_count < MOTOR_CMD_MAX_RETRY) {
                g_cmd_queue[i].retry_count++;
                g_cmd_stats.retry_count++;
                g_cmd_queue[i].send_tick = current_tick;
                
                printf("[MOTOR-CMD] Timeout! Device 0x%02X Cmd 0x%03X Retry %d/%d\r\n",
                       g_cmd_queue[i].device_id, 
                       g_cmd_queue[i].message_id,
                       g_cmd_queue[i].retry_count,
                       MOTOR_CMD_MAX_RETRY);
                
                processed = 1;
            } else {
                g_cmd_queue[i].acknowledged = 1;
                g_cmd_stats.timeout_count++;
                g_cmd_stats.failed_count++;
                
                printf("[MOTOR-CMD] Device 0x%02X Cmd 0x%03X: TIMEOUT (max retries)\r\n",
                       g_cmd_queue[i].device_id, 
                       g_cmd_queue[i].message_id);
            }
        }
    }
    
    while (g_cmd_tail != g_cmd_head && g_cmd_queue[g_cmd_tail].acknowledged) {
        g_cmd_tail = (g_cmd_tail + 1) % MOTOR_CMD_QUEUE_SIZE;
    }
}

void motor_cmd_get_stats(MotorCmdStats_t *stats)
{
    if (stats) {
        memcpy(stats, &g_cmd_stats, sizeof(MotorCmdStats_t));
    }
}

void motor_cmd_print_stats(void)
{
    printf("\n========== Command Tracking Stats ==========\r\n");
    printf("Total Commands:  %lu\r\n", (unsigned long)g_cmd_stats.total_commands);
    printf("Success:        %lu\r\n", (unsigned long)g_cmd_stats.success_count);
    printf("Retry:          %lu\r\n", (unsigned long)g_cmd_stats.retry_count);
    printf("Failed:         %lu\r\n", (unsigned long)g_cmd_stats.failed_count);
    printf("Timeout:        %lu\r\n", (unsigned long)g_cmd_stats.timeout_count);
    printf("===========================================\r\n");
}

uint8_t motor_cmd_get_failed(MotorCommand_t *cmd)
{
    if (cmd == NULL) return 1;
    
    for (uint8_t i = 0; i < MOTOR_CMD_QUEUE_SIZE; i++) {
        if (g_cmd_queue[i].device_id == 0) continue;
        if (g_cmd_queue[i].acknowledged) continue;
        
        uint32_t elapsed = HAL_GetTick() - g_cmd_queue[i].send_tick;
        if (MOTOR_CMD_TIMEOUT_MS > 0 && elapsed > MOTOR_CMD_TIMEOUT_MS) {
            memcpy(cmd, &g_cmd_queue[i], sizeof(MotorCommand_t));
            return 0;
        }
    }
    return 1;
}

uint8_t motor_cmd_retry(uint8_t device_id, uint16_t message_id, float target_value)
{
    uint8_t txdata[8] = {0};
    FDCAN_TxConfigTypeDef config = {0};
    
    config.identifier = motor_build_ext_id(device_id, message_id);
    config.id_type = FDCAN_EXTENDED_ID;
    config.frame_type = FDCAN_DATA_FRAME;
    config.fd_format = FDCAN_FD_CAN;
    config.bit_rate_switch = FDCAN_BRS_ON;
    
    switch (message_id) {
        case MOTOR_MSG_ENABLE:
            txdata[0] = (uint8_t)target_value;
            config.data_length = FDCAN_DLC_BYTES_1;
            printf("[MOTOR-CMD] Retry: Device 0x%02X ENABLE (%.0f)\r\n", device_id, target_value);
            break;
        case MOTOR_MSG_MODE_SET:
            txdata[0] = (uint8_t)target_value;
            config.data_length = FDCAN_DLC_BYTES_1;
            printf("[MOTOR-CMD] Retry: Device 0x%02X MODE (%.0f)\r\n", device_id, target_value);
            break;
        case MOTOR_MSG_START_STOP:
            txdata[0] = target_value ? 0x00 : 0x01;
            config.data_length = FDCAN_DLC_BYTES_1;
            printf("[MOTOR-CMD] Retry: Device 0x%02X START/STOP (%.0f)\r\n", device_id, target_value);
            break;
        default:
            printf("[MOTOR-CMD] Retry: Unknown message ID 0x%03X\r\n", message_id);
            return 1;
    }
    
    motor_cmd_track(device_id, message_id, target_value);
    return motor_send_by_idx(1, txdata, &config);
}

/******************************************************************************************/
/* 位置目标执行确认统计实现 */

#define MOTOR_MAX_DEVICES 128

static MotorExecStats_t g_exec_stats[MOTOR_MAX_DEVICES];
static bool g_exec_stats_initialized = false;

void motor_exec_stats_init(void)
{
    memset(g_exec_stats, 0, sizeof(g_exec_stats));
    g_exec_stats_initialized = true;
    printf("[MOTOR-EXEC] Execution statistics initialized\r\n");
}

void motor_exec_record_target(uint8_t device_id, float target)
{
    if (device_id >= MOTOR_MAX_DEVICES || !g_exec_stats_initialized) {
        return;
    }
    
    MotorExecStats_t *stats = &g_exec_stats[device_id];
    uint8_t idx = stats->head;
    
    stats->records[idx].target_position = target;
    stats->records[idx].actual_position = 0.0f;
    stats->records[idx].error = 0.0f;
    stats->records[idx].send_tick = HAL_GetTick();
    stats->records[idx].confirm_tick = 0;
    stats->records[idx].confirmed = 0;
    stats->records[idx].timeout = 0;
    stats->records[idx].error_exceeded = 0;
    
    stats->head = (stats->head + 1) % MOTOR_EXEC_STATS_WINDOW;
    if (stats->count < MOTOR_EXEC_STATS_WINDOW) {
        stats->count++;
    }
    
    stats->total_commands++;
    stats->last_update_tick = HAL_GetTick();
}

void motor_exec_confirm(uint8_t device_id, float actual)
{
    if (device_id >= MOTOR_MAX_DEVICES || !g_exec_stats_initialized) {
        return;
    }
    
    MotorExecStats_t *stats = &g_exec_stats[device_id];
    
    if (stats->count == 0) {
        return;
    }
    
    int8_t newest_unconfirmed = -1;
    for (int8_t i = (int8_t)stats->count - 1; i >= 0; i--) {
        uint8_t idx = (stats->head + MOTOR_EXEC_STATS_WINDOW - 1 - i) % MOTOR_EXEC_STATS_WINDOW;
        if (!stats->records[idx].confirmed && !stats->records[idx].timeout) {
            newest_unconfirmed = idx;
            break;
        }
    }
    
    if (newest_unconfirmed < 0) {
        return;
    }
    
    MotorExecRecord_t *rec = &stats->records[newest_unconfirmed];
    
    rec->actual_position = actual;
    rec->error = fabsf(rec->target_position - actual);
    rec->confirm_tick = HAL_GetTick();
    rec->confirmed = 1;
    rec->error_exceeded = (rec->error > MOTOR_EXEC_ERROR_THRESHOLD) ? 1 : 0;
    
    stats->confirmed_count++;
    if (rec->error_exceeded) {
        stats->error_exceeded_count++;
    }
    
    float total_error = 0.0f;
    uint32_t confirmed_in_window = 0;
    stats->max_error = 0.0f;
    float total_latency = 0.0f;
    
    for (uint8_t i = 0; i < stats->count; i++) {
        uint8_t idx = (stats->head + MOTOR_EXEC_STATS_WINDOW - stats->count + i) % MOTOR_EXEC_STATS_WINDOW;
        if (stats->records[idx].confirmed) {
            total_error += stats->records[idx].error;
            if (stats->records[idx].error > stats->max_error) {
                stats->max_error = stats->records[idx].error;
            }
            total_latency += (stats->records[idx].confirm_tick - stats->records[idx].send_tick);
            confirmed_in_window++;
        }
    }
    
    if (confirmed_in_window > 0) {
        stats->avg_error = total_error / confirmed_in_window;
        stats->avg_latency_ms = total_latency / confirmed_in_window;
    }
    
    stats->last_update_tick = HAL_GetTick();
}

void motor_exec_process_timeout(void)
{
    if (!g_exec_stats_initialized) {
        return;
    }
    
    uint32_t now = HAL_GetTick();
    
    for (uint16_t dev = 0; dev < MOTOR_MAX_DEVICES; dev++) {
        MotorExecStats_t *stats = &g_exec_stats[dev];
        
        if (stats->count == 0) {
            continue;
        }
        
        for (uint8_t i = 0; i < stats->count; i++) {
            uint8_t idx = (stats->head + MOTOR_EXEC_STATS_WINDOW - stats->count + i) % MOTOR_EXEC_STATS_WINDOW;
            MotorExecRecord_t *rec = &stats->records[idx];
            
            if (!rec->confirmed && !rec->timeout) {
                if ((now - rec->send_tick) > MOTOR_EXEC_TIMEOUT_MS) {
                    rec->timeout = 1;
                    stats->timeout_count++;
                }
            }
        }
    }
}

void motor_exec_get_stats(uint8_t device_id, MotorExecStats_t *stats)
{
    if (device_id >= MOTOR_MAX_DEVICES || stats == NULL || !g_exec_stats_initialized) {
        if (stats != NULL) {
            memset(stats, 0, sizeof(MotorExecStats_t));
        }
        return;
    }
    
    memcpy(stats, &g_exec_stats[device_id], sizeof(MotorExecStats_t));
}

void motor_exec_get_summary(MotorExecSummary_t *summary)
{
    if (summary == NULL || !g_exec_stats_initialized) {
        return;
    }
    
    memset(summary, 0, sizeof(MotorExecSummary_t));
    
    float total_error = 0.0f;
    uint32_t total_confirmed = 0;
    
    for (uint16_t dev = 0; dev < MOTOR_MAX_DEVICES; dev++) {
        MotorExecStats_t *stats = &g_exec_stats[dev];
        
        summary->total_commands += stats->total_commands;
        summary->total_confirmed += stats->confirmed_count;
        summary->total_timeout += stats->timeout_count;
        summary->total_error_exceeded += stats->error_exceeded_count;
        
        if (stats->max_error > summary->global_max_error) {
            summary->global_max_error = stats->max_error;
        }
        
        total_error += stats->avg_error * stats->confirmed_count;
        total_confirmed += stats->confirmed_count;
    }
    
    if (total_confirmed > 0) {
        summary->global_avg_error = total_error / total_confirmed;
    }
    
    if (summary->total_commands > 0) {
        summary->confirm_rate_x1000 = (summary->total_confirmed * 1000) / summary->total_commands;
    }
}

void motor_exec_print_stats(uint8_t device_id)
{
    if (device_id >= MOTOR_MAX_DEVICES || !g_exec_stats_initialized) {
        printf("[MOTOR-EXEC] Invalid device ID or not initialized\r\n");
        return;
    }
    
    MotorExecStats_t *stats = &g_exec_stats[device_id];
    
    printf("\r\n========== Motor %d Execution Stats ==========\r\n", device_id);
    printf("Total Commands:    %u\r\n", stats->total_commands);
    printf("Confirmed:         %u\r\n", stats->confirmed_count);
    printf("Timeout:           %u\r\n", stats->timeout_count);
    printf("Error Exceeded:    %u\r\n", stats->error_exceeded_count);
    
    if (stats->total_commands > 0) {
        uint32_t rate = (stats->confirmed_count * 1000) / stats->total_commands;
        printf("Confirm Rate:      %u.%u%%\r\n", rate / 10, rate % 10);
    }
    
    printf("Avg Error:         %.2f deg\r\n", stats->avg_error);
    printf("Max Error:         %.2f deg\r\n", stats->max_error);
    printf("Avg Latency:       %.1f ms\r\n", stats->avg_latency_ms);
    printf("=============================================\r\n\r\n");
}

void motor_exec_print_summary(void)
{
    MotorExecSummary_t summary;
    motor_exec_get_summary(&summary);
    
    printf("\r\n========== Motor Execution Summary ==========\r\n");
    printf("Total Commands:    %u\r\n", summary.total_commands);
    printf("Total Confirmed:   %u\r\n", summary.total_confirmed);
    printf("Total Timeout:     %u\r\n", summary.total_timeout);
    printf("Error Exceeded:    %u\r\n", summary.total_error_exceeded);
    
    if (summary.total_commands > 0) {
        printf("Confirm Rate:      %u.%u%%\r\n", 
               summary.confirm_rate_x1000 / 10, 
               summary.confirm_rate_x1000 % 10);
    }
    
    printf("Global Avg Error:  %.2f deg\r\n", summary.global_avg_error);
    printf("Global Max Error:  %.2f deg\r\n", summary.global_max_error);
    printf("=============================================\r\n\r\n");
}

void motor_exec_reset_stats(uint8_t device_id)
{
    if (device_id >= MOTOR_MAX_DEVICES || !g_exec_stats_initialized) {
        return;
    }
    
    memset(&g_exec_stats[device_id], 0, sizeof(MotorExecStats_t));
    printf("[MOTOR-EXEC] Stats reset for device %d\r\n", device_id);
}

void motor_exec_reset_all(void)
{
    if (!g_exec_stats_initialized) {
        return;
    }
    
    memset(g_exec_stats, 0, sizeof(g_exec_stats));
    printf("[MOTOR-EXEC] All stats reset\r\n");
}

