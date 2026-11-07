/**
 ****************************************************************************************************
 * @file        motor_control.h
 * @version     V1.0
 * @brief       Xynova旋转关节电机控制驱动头文件
 * @note        基于CAN-FD通讯协议实现电机实时控制
 ****************************************************************************************************
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************************/
/* 消息ID定义 - 基本控制命令 */
#define MOTOR_MSG_ENABLE                0x001   /* 使能/失能 */
#define MOTOR_MSG_MODE_SET              0x002   /* 运行模式设置 */
#define MOTOR_MSG_START_STOP            0x004   /* 启动/停止 */
#define MOTOR_MSG_SET_ZERO              0x006   /* 设置零点 */
#define MOTOR_MSG_RESTORE_ZERO          0x007   /* 恢复出厂零点 */
#define MOTOR_MSG_SAVE_CTRL_PARAM       0x00A   /* 保存控制参数至FLASH */
#define MOTOR_MSG_SAVE_LIMIT_PARAM      0x00B   /* 保存限制参数至FLASH */

/* 消息ID定义 - 错误与状态反馈 */
#define MOTOR_MSG_ERROR                 0x00F   /* 错误上报 */

/* 消息ID定义 - 设备识别与配置 */
#define MOTOR_MSG_DEVICE_ID             0x011   /* 读取/设置设备ID */
#define MOTOR_MSG_SAVE_SOFT_ZERO        0x012   /* 保存软件零点 */

/* 消息ID定义 - 目标值设置 */
#define MOTOR_MSG_POS_TARGET            0x131   /* 位置目标设置 (°) */
#define MOTOR_MSG_SPEED_TARGET          0x133   /* 速度目标设置 (rpm) */
#define MOTOR_MSG_TORQUE_TARGET         0x134   /* 转矩目标设置 (Nm) */

/* 消息ID定义 - 限制参数设置 */
#define MOTOR_MSG_LIMIT_SPEED           0x141   /* 限定速度 (rpm) */
#define MOTOR_MSG_LIMIT_ACCEL           0x142   /* 限定加速度 (rpm/s) */
#define MOTOR_MSG_TORQUE_SLOPE          0x151   /* 力矩斜率 (Nm/s) */

/* 消息ID定义 - 参数读取 */
#define MOTOR_MSG_READ_RESISTANCE       0x210   /* 读取电阻 (Ω) */
#define MOTOR_MSG_READ_INDUCTANCE       0x211   /* 读取电感 (H) */
#define MOTOR_MSG_READ_PEAK_SPEED       0x212   /* 读取峰值转速 (rpm) */
#define MOTOR_MSG_READ_PEAK_TORQUE      0x213   /* 读取峰值扭矩 (Nm) */
#define MOTOR_MSG_READ_RATED_SPEED      0x214   /* 读取额定转速 (rpm) */
#define MOTOR_MSG_READ_RATED_TORQUE     0x215   /* 读取额定扭矩 (Nm) */
#define MOTOR_MSG_READ_RATED_CURRENT    0x216   /* 读取额定电流 (A) */
#define MOTOR_MSG_READ_RATED_VOLTAGE    0x217   /* 读取额定电压 (V) */
#define MOTOR_MSG_READ_MOTOR_PARAMS     0x21F   /* 读取电机固有参数 (11*4字节) */

/* 消息ID定义 - 控制环参数设置 */
#define MOTOR_MSG_SET_POS_KP            0x221   /* 设置位置环Kp */
#define MOTOR_MSG_SET_POS_KI            0x222   /* 设置位置环Ki */
#define MOTOR_MSG_SET_POS_KD            0x223   /* 设置位置环Kd */
#define MOTOR_MSG_SET_SPEED_KP          0x224   /* 设置转速环Kp */
#define MOTOR_MSG_SET_SPEED_KI          0x225   /* 设置转速环Ki */
#define MOTOR_MSG_SET_SPEED_KD          0x226   /* 设置转速环Kd */
#define MOTOR_MSG_SET_CURR_KP           0x227   /* 设置电流环Kp */
#define MOTOR_MSG_SET_CURR_KI           0x228   /* 设置电流环Ki */
#define MOTOR_MSG_SET_CURR_KD           0x229   /* 设置电流环Kd */
#define MOTOR_MSG_SET_POS_BANDWIDTH     0x22A   /* 设置位置环带宽 */
#define MOTOR_MSG_SET_SPEED_BANDWIDTH   0x22B   /* 设置转速环带宽 */
#define MOTOR_MSG_SET_CURR_BANDWIDTH    0x22C   /* 设置电流环带宽 */
#define MOTOR_MSG_READ_CTRL_PARAMS      0x22F   /* 读取控制参数 (12*4字节) */

/* 消息ID定义 - 限幅参数 */
#define MOTOR_MSG_POS_UPPER_LIMIT       0x240   /* 位置上限 (°) */
#define MOTOR_MSG_POS_LOWER_LIMIT       0x241   /* 位置下限 (°) */
#define MOTOR_MSG_SPEED_LIMIT           0x242   /* 速度限幅 (rpm) */
#define MOTOR_MSG_TORQUE_LIMIT          0x243   /* 扭矩限幅 (Nm) */
#define MOTOR_MSG_CURRENT_LIMIT         0x244   /* 电流限幅 (A) */
#define MOTOR_MSG_READ_LIMIT_PARAMS     0x24F   /* 读取限制参数 (5*4字节) */

/* 消息ID定义 - 监测与恢复 */
#define MOTOR_MSG_RESTORE_DEFAULT       0x251   /* 恢复默认参数 */
#define MOTOR_MSG_MONITOR               0x31F   /* 监测命令 (9*4字节) */

/******************************************************************************************/
/* 电机运行模式枚举 */
typedef enum {
    MOTOR_MODE_POSITION = 0x00,     /* 位置模式 */
    MOTOR_MODE_SPEED    = 0x01,     /* 速度模式 */
    MOTOR_MODE_TORQUE   = 0x02,     /* 力矩模式 */
    MOTOR_MODE_PROFILE  = 0x06      /* 轮廓位置模式（暂不支持）*/
} MotorMode_t;

/* 电机错误代码枚举 */
typedef enum {
    MOTOR_ERROR_NONE            = 0x00,  /* 无故障 */
    MOTOR_ERROR_OVERVOLTAGE     = 0x01,  /* 过压 */
    MOTOR_ERROR_OVERCURRENT     = 0x02,  /* 过流 */
    MOTOR_ERROR_MOTOR_OVERTEMP  = 0x03,  /* 电机过温 */
    MOTOR_ERROR_BOARD_OVERTEMP  = 0x04,  /* 电路板过温 */
    MOTOR_ERROR_UNDERVOLTAGE    = 0x05,  /* 欠压 */
    MOTOR_ERROR_TIMEOUT         = 0x06,  /* 通信超时 */
    MOTOR_ERROR_STALL           = 0x07,  /* 堵转 */
    MOTOR_ERROR_POSITION       = 0x08   /* 位置超限 */
} MotorError_t;

/* 电机错误标志宏（用于位掩码操作）*/
#define MOTOR_ERROR_FLAG_NONE        0x0000
#define MOTOR_ERROR_FLAG_OVERVOLTAGE  0x0001
#define MOTOR_ERROR_FLAG_OVERCURRENT  0x0002
#define MOTOR_ERROR_FLAG_OVERTEMP     0x0004
#define MOTOR_ERROR_FLAG_UNDERVOLTAGE 0x0008
#define MOTOR_ERROR_FLAG_TIMEOUT      0x0010
#define MOTOR_ERROR_FLAG_STALL        0x0020
#define MOTOR_ERROR_FLAG_POSITION     0x0040

/* 通信超时阈值（ms）- 设为0可禁用超时检测(用于无电机测试) */
//#define MOTOR_STATUS_TIMEOUT_MS  0

/******************************************************************************************/
/* 电机状态结构体 */
typedef struct {
    uint8_t enabled;                /* 使能状态: 0=失能, 1=使能 */
    uint8_t running;                /* 运行状态: 0=停止, 1=运行 */
    MotorMode_t mode;               /* 运行模式 */
    MotorError_t error_code;        /* 错误代码 */
    uint8_t device_id;              /* 设备ID */
} MotorStatus_t;

/* 电机监控数据结构体 (9*4字节 = 36字节) */
typedef struct {
    float target_position;          /* 目标位置 (°) */
    float actual_position;          /* 实际位置 (°) */
    float target_speed;             /* 目标转速 (rpm) */
    float actual_speed;             /* 实际转速 (rpm) */
    float target_torque;            /* 目标转矩 (Nm) */
    float actual_torque;            /* 实际转矩 (Nm) */
    float motor_temperature;        /* 电机温度 (°C) */
    float board_temperature;        /* 电路板温度 (°C) */
    float bus_voltage;              /* 母线电压 (V) */
} MotorMonitorData_t;

/* 电机固有参数结构体 (11*4字节 = 44字节) */
typedef struct {
    float resistance;               /* 电阻 (Ω) */
    float inductance;               /* 电感 (H) */
    float peak_speed;               /* 峰值转速 (rpm) */
    float peak_torque;              /* 峰值转矩 (Nm) */
    float rated_speed;              /* 额定转速 (rpm) */
    float rated_torque;             /* 额定转矩 (Nm) */
    float rated_current;            /* 额定电流 (A) */
    float rated_voltage;            /* 额定电压 (V) */
    float inertia;                  /* 转动惯量 (kg·m²) */
    float peak_current;             /* 峰值电流 (A) */
    float gear_ratio;               /* 减速比 */
} MotorParams_t;

/* 电机控制PID参数结构体 (12*4字节 = 48字节) */
typedef struct {
    float pos_kp;                   /* 位置环Kp */
    float pos_ki;                   /* 位置环Ki */
    float pos_kd;                   /* 位置环Kd */
    float speed_kp;                 /* 转速环Kp */
    float speed_ki;                 /* 转速环Ki */
    float speed_kd;                 /* 转速环Kd */
    float curr_kp;                  /* 电流环Kp */
    float curr_ki;                  /* 电流环Ki */
    float curr_kd;                  /* 电流环Kd */
    float pos_bandwidth;            /* 位置环带宽 */
    float speed_bandwidth;          /* 转速环带宽 */
    float curr_bandwidth;           /* 电流环带宽 */
} MotorCtrlParams_t;

/* 电机限制参数结构体 (5*4字节 = 20字节) */
typedef struct {
    float pos_upper_limit;          /* 位置上限 (°) */
    float pos_lower_limit;          /* 位置下限 (°) */
    float speed_limit;              /* 速度限幅 (rpm) */
    float torque_limit;             /* 扭矩限幅 (Nm) */
    float current_limit;            /* 电流限幅 (A) */
} MotorLimitParams_t;

/******************************************************************************************/
/* 回调函数类型定义 */

/**
 * @brief       电机错误回调函数类型
 * @param       device_id   : 发生错误的设备ID
 * @param       error_code  : 错误码
 * @note        当电机主动上报错误或查询到错误时被调用
 */
typedef void (*motor_error_callback_t)(uint8_t device_id, MotorError_t error_code);

/******************************************************************************************/
/* 函数声明 - 基本控制命令 (新增第一个参数 `can` 选择 CAN 实例: 1 或 2) */
uint8_t motor_enable(uint8_t can, uint8_t device_id, uint8_t enable);               /* 使能/失能电机 */
uint8_t motor_set_mode(uint8_t can, uint8_t device_id, MotorMode_t mode);          /* 设置运行模式 */
uint8_t motor_start_stop(uint8_t can, uint8_t device_id, uint8_t start);           /* 启动/停止电机 */
uint8_t motor_set_zero(uint8_t can, uint8_t device_id);                            /* 设置零点 */
uint8_t motor_restore_zero(uint8_t can, uint8_t device_id);                        /* 恢复出厂零点 */
uint8_t motor_save_ctrl_params(uint8_t can, uint8_t device_id);                    /* 保存控制参数至FLASH */
uint8_t motor_save_limit_params(uint8_t can, uint8_t device_id);                   /* 保存限制参数至FLASH */
uint8_t motor_save_soft_zero(uint8_t can, uint8_t device_id);                      /* 保存软件零点至FLASH */
uint8_t motor_restore_default(uint8_t can, uint8_t device_id);                     /* 恢复默认参数 */

/* 函数声明 - 设备ID管理 */
uint8_t motor_read_device_id(uint8_t can, uint8_t device_id, uint32_t *dev_id);    /* 读取设备ID */
uint8_t motor_set_device_id(uint8_t can, uint8_t device_id, uint32_t new_dev_id);  /* 设置设备ID */

/* 函数声明 - 目标值设置 */
uint8_t motor_set_position(uint8_t can, uint8_t device_id, float position);        /* 设置位置目标 (°) */
uint8_t motor_set_speed(uint8_t can, uint8_t device_id, float speed);              /* 设置速度目标 (rpm) */
uint8_t motor_set_torque(uint8_t can, uint8_t device_id, float torque);            /* 设置转矩目标 (Nm) */

/* 函数声明 - 限制参数设置 */
uint8_t motor_set_limit_speed(uint8_t can, uint8_t device_id, float speed);        /* 设置限定速度 (rpm) */
uint8_t motor_set_limit_accel(uint8_t can, uint8_t device_id, float accel);        /* 设置限定加速度 (rpm/s) */
uint8_t motor_set_torque_slope(uint8_t can, uint8_t device_id, float slope);       /* 设置力矩斜率 (Nm/s) */

/* 函数声明 - PID控制参数设置 */
uint8_t motor_set_pos_kp(uint8_t can, uint8_t device_id, float kp);                /* 设置位置环Kp */
uint8_t motor_set_pos_ki(uint8_t can, uint8_t device_id, float ki);                /* 设置位置环Ki */
uint8_t motor_set_pos_kd(uint8_t can, uint8_t device_id, float kd);                /* 设置位置环Kd */
uint8_t motor_set_speed_kp(uint8_t can, uint8_t device_id, float kp);              /* 设置速度环Kp */
uint8_t motor_set_speed_ki(uint8_t can, uint8_t device_id, float ki);              /* 设置速度环Ki */
uint8_t motor_set_speed_kd(uint8_t can, uint8_t device_id, float kd);              /* 设置速度环Kd */
uint8_t motor_set_curr_kp(uint8_t can, uint8_t device_id, float kp);               /* 设置电流环Kp */
uint8_t motor_set_curr_ki(uint8_t can, uint8_t device_id, float ki);               /* 设置电流环Ki */
uint8_t motor_set_curr_kd(uint8_t can, uint8_t device_id, float kd);               /* 设置电流环Kd */
uint8_t motor_set_pos_bandwidth(uint8_t can, uint8_t device_id, float bandwidth);  /* 设置位置环带宽 */
uint8_t motor_set_speed_bandwidth(uint8_t can, uint8_t device_id, float bandwidth);/* 设置速度环带宽 */
uint8_t motor_set_curr_bandwidth(uint8_t can, uint8_t device_id, float bandwidth); /* 设置电流环带宽 */

/* 函数声明 - 限幅参数设置 */
uint8_t motor_set_pos_upper_limit(uint8_t can, uint8_t device_id, float upper_limit); /* 设置位置上限 (°) */
uint8_t motor_set_pos_lower_limit(uint8_t can, uint8_t device_id, float lower_limit); /* 设置位置下限 (°) */
uint8_t motor_set_speed_limit(uint8_t can, uint8_t device_id, float speed_limit);     /* 设置速度限幅 (rpm) */
uint8_t motor_set_torque_limit(uint8_t can, uint8_t device_id, float torque_limit);   /* 设置扭矩限幅 (Nm) */
uint8_t motor_set_current_limit(uint8_t can, uint8_t device_id, float current_limit); /* 设置电流限幅 (A) */

/* 函数声明 - 参数读取 */
uint8_t motor_read_resistance(uint8_t can, uint8_t device_id);                            /* 读取电机电阻 */
uint8_t motor_read_inductance(uint8_t can, uint8_t device_id);                            /* 读取电机电感 */
uint8_t motor_read_peak_speed(uint8_t can, uint8_t device_id);                            /* 读取峰值转速 */
uint8_t motor_read_peak_torque(uint8_t can, uint8_t device_id);                           /* 读取峰值扭矩 */
uint8_t motor_read_rated_speed(uint8_t can, uint8_t device_id);                           /* 读取额定转速 */
uint8_t motor_read_rated_torque(uint8_t can, uint8_t device_id);                          /* 读取额定扭矩 */
uint8_t motor_read_rated_current(uint8_t can, uint8_t device_id);                         /* 读取额定电流 */
uint8_t motor_read_rated_voltage(uint8_t can, uint8_t device_id);                         /* 读取额定电压 */
uint8_t motor_read_monitor_data(uint8_t can, uint8_t device_id, MotorMonitorData_t *data);   /* 读取监测数据 */
uint8_t motor_read_motor_params(uint8_t can, uint8_t device_id, MotorParams_t *params);      /* 读取电机固有参数 */
uint8_t motor_read_ctrl_params(uint8_t can, uint8_t device_id, MotorCtrlParams_t *params);   /* 读取控制参数 */
uint8_t motor_read_limit_params(uint8_t can, uint8_t device_id, MotorLimitParams_t *params); /* 读取限制参数 */

/* 函数声明 - 错误处理 */
uint8_t motor_clear_error(uint8_t can, uint8_t device_id);                         /* 清除错误码 */
uint8_t motor_get_error(uint8_t can, uint8_t device_id, MotorError_t *error);      /* 获取错误码 */

/* 函数声明 - 辅助函数 */
uint32_t motor_build_ext_id(uint8_t device_id, uint16_t message_id);  /* 构建扩展ID */
void motor_float_to_bytes(float value, uint8_t *bytes);                /* 浮点数转字节数组 */
float motor_bytes_to_float(const uint8_t *bytes);                      /* 字节数组转浮点数 */

/* 函数声明 - 初始化与控制流程 */
uint8_t motor_init_sequence(uint8_t can, uint8_t device_id, MotorMode_t mode);     /* 电机初始化序列 */
uint8_t motor_shutdown_sequence(uint8_t can, uint8_t device_id);                   /* 电机关闭序列 */

/* 函数声明 - 状态管理 */
void motor_update_status(uint8_t device_id, MotorStatus_t *status);   /* 更新电机状态 */
void motor_print_status(uint8_t device_id, MotorStatus_t *status);    /* 打印电机状态 */
void motor_print_monitor_data(MotorMonitorData_t *data);               /* 打印监测数据 */
bool motor_get_status_with_timeout(uint8_t device_id, MotorStatus_t *status);  /* 获取状态（带超时检测）*/

/* 函数声明 - 数据解析 */
void motor_parse_monitor_data(const uint8_t *rxdata, MotorMonitorData_t *data);  /* 解析监测数据 */
void motor_parse_motor_params(const uint8_t *rxdata, MotorParams_t *params);     /* 解析电机参数 */
void motor_parse_ctrl_params(const uint8_t *rxdata, MotorCtrlParams_t *params);  /* 解析控制参数 */
void motor_parse_limit_params(const uint8_t *rxdata, MotorLimitParams_t *params);/* 解析限制参数 */

/* 函数声明 - CAN接收回调处理 */
void motor_process_rx_message(uint32_t rx_id, const uint8_t *rxdata, uint8_t data_len); /* 处理接收的CAN消息 */

/* 函数声明 - 回调管理 */
void motor_register_error_callback(motor_error_callback_t callback);  /* 注册错误回调函数 */

/* 初始化/去初始化 */
void motor_control_init(void);   /* 在 RTOS 启动时调用，创建队列和处理任务 */
void motor_control_deinit(void);

/* 批量控制函数 */
void motor_control_enable_all(void);        /* 启用所有电机 */
void motor_control_disable_all(void);       /* 禁用所有电机 */
void motor_control_stop_all(void);          /* 停止所有电机（保持使能）*/
void motor_control_emergency_stop_all(void); /* 紧急停止所有电机 */
void motor_control_power_cut(void);        /* 切断电机电源（硬件层面）*/
void motor_print_rx_statistics(void);       /* 打印消息处理统计 */
void motor_reset_rx_statistics(void);        /* 重置消息处理统计 */

/******************************************************************************************/
/* 命令确认与重试机制 */

#define MOTOR_CMD_MAX_RETRY       3       /* 最大重试次数 */
#define MOTOR_CMD_TIMEOUT_MS      0       /* 命令超时时间(ms) - 设为0可禁用(用于无电机测试) */
#define MOTOR_CMD_QUEUE_SIZE      64      /* 命令队列大小 - 增加到64以支持多电机同时初始化 */

/* 命令追踪结构体 */
typedef struct {
    uint8_t device_id;            /* 电机ID */
    uint16_t message_id;          /* 消息ID */
    uint32_t send_tick;           /* 发送时间 */
    uint8_t retry_count;          /* 重试次数 */
    uint8_t acknowledged;          /* 是否已确认 */
    float target_value;           /* 目标值(用于位置/速度/扭矩) */
} MotorCommand_t;

/* 命令追踪统计 */
typedef struct {
    uint32_t total_commands;      /* 总命令数 */
    uint32_t success_count;       /* 成功次数 */
    uint32_t retry_count;         /* 重试次数 */
    uint32_t failed_count;        /* 失败次数 */
    uint32_t timeout_count;       /* 超时次数 */
} MotorCmdStats_t;

/* 函数声明 - 命令追踪 */
void motor_cmd_init(void);                                        /* 初始化命令追踪 */
uint8_t motor_cmd_track(uint8_t device_id, uint16_t message_id, float target_value);  /* 记录命令 */
void motor_cmd_handle_response(uint8_t device_id, uint16_t message_id, uint8_t result); /* 处理响应 */
void motor_cmd_process_timeout(void);                             /* 处理超时(在任务中调用) */
uint8_t motor_cmd_get_failed(MotorCommand_t *cmd);                /* 获取失败命令(供手动重试) */
void motor_cmd_get_stats(MotorCmdStats_t *stats);                 /* 获取统计 */
void motor_cmd_print_stats(void);                                 /* 打印统计 */
uint8_t motor_cmd_retry(uint8_t device_id, uint16_t message_id, float target_value);  /* 重试命令 */

/******************************************************************************************/
/* 位置目标执行确认统计 - 轻量级统计机制 */

#define MOTOR_EXEC_STATS_WINDOW     32      /* 滑动窗口大小 */
#define MOTOR_EXEC_ERROR_THRESHOLD  5.0f    /* 执行误差阈值 (°) */
#define MOTOR_EXEC_TIMEOUT_MS       200     /* 执行超时时间 (ms) */

/* 单次执行记录 */
typedef struct {
    float target_position;         /* 目标位置 (°) */
    float actual_position;         /* 实际位置 (°) */
    float error;                   /* 执行误差 (°) */
    uint32_t send_tick;            /* 发送时间戳 */
    uint32_t confirm_tick;         /* 确认时间戳 */
    uint8_t confirmed;             /* 是否已确认 */
    uint8_t timeout;               /* 是否超时 */
    uint8_t error_exceeded;        /* 误差是否超限 */
} MotorExecRecord_t;

/* 单个电机的执行统计 */
typedef struct {
    MotorExecRecord_t records[MOTOR_EXEC_STATS_WINDOW];  /* 滑动窗口记录 */
    uint8_t head;                  /* 窗口头指针 */
    uint8_t count;                 /* 当前记录数 */
    
    uint32_t total_commands;       /* 总发送次数 */
    uint32_t confirmed_count;      /* 已确认次数 */
    uint32_t timeout_count;        /* 超时次数 */
    uint32_t error_exceeded_count; /* 误差超限次数 */
    
    float avg_error;               /* 平均误差 (°) */
    float max_error;               /* 最大误差 (°) */
    float avg_latency_ms;          /* 平均确认延迟 (ms) */
    uint32_t last_update_tick;     /* 最后更新时间 */
} MotorExecStats_t;

/* 全局执行统计摘要 */
typedef struct {
    uint32_t total_commands;       /* 所有电机总发送次数 */
    uint32_t total_confirmed;      /* 总确认次数 */
    uint32_t total_timeout;        /* 总超时次数 */
    uint32_t total_error_exceeded; /* 总误差超限次数 */
    float global_avg_error;        /* 全局平均误差 */
    float global_max_error;        /* 全局最大误差 */
    uint32_t confirm_rate_x1000;   /* 确认率 * 1000 (如 950 = 95.0%) */
} MotorExecSummary_t;

/* 函数声明 - 位置执行确认统计 */
void motor_exec_stats_init(void);                              /* 初始化统计 */
void motor_exec_record_target(uint8_t device_id, float target); /* 记录发送的目标位置 */
void motor_exec_confirm(uint8_t device_id, float actual);      /* 确认执行结果 */
void motor_exec_process_timeout(void);                         /* 处理超时(周期调用) */
void motor_exec_get_stats(uint8_t device_id, MotorExecStats_t *stats); /* 获取单电机统计 */
void motor_exec_get_summary(MotorExecSummary_t *summary);      /* 获取全局摘要 */
void motor_exec_print_stats(uint8_t device_id);                /* 打印单电机统计 */
void motor_exec_print_summary(void);                           /* 打印全局摘要 */
void motor_exec_reset_stats(uint8_t device_id);                /* 重置单电机统计 */
void motor_exec_reset_all(void);                               /* 重置所有统计 */

#endif /* __MOTOR_CONTROL_H */
