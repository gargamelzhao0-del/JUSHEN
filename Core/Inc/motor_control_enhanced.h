/**
 ****************************************************************************************************
 * @file        motor_control_enhanced.h
 * @version     V1.0
 * @brief       电机控制增强模块 - 批量优化和高级功能
 * @details     为motor_control.c添加批量控制、反馈优化等高级功能
 ****************************************************************************************************
 */

#ifndef __MOTOR_CONTROL_ENHANCED_H
#define __MOTOR_CONTROL_ENHANCED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_control.h"
#include "quad_command_fusion.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 配置常量
 * ============================================================================ */

#define MOTOR_MAX_HANDLES           12      /* 最大电机句柄数量（12电机）*/

/* V9.0: 电机保护阈值参数 */
#define MOTOR_TEMP_WARNING          70.0f   /* 电机温度警告阈值 (℃) */
#define MOTOR_TEMP_CRITICAL         85.0f   /* 电机温度危险阈值 (℃) */
#define MOTOR_TEMP_SHUTDOWN         95.0f   /* 电机温度停机阈值 (℃) */

#define MOTOR_BOARD_TEMP_WARNING    60.0f   /* 电路板温度警告阈值 (℃) */
#define MOTOR_BOARD_TEMP_CRITICAL   75.0f   /* 电路板温度危险阈值 (℃) */

#define MOTOR_VOLTAGE_WARNING       20.0f   /* 母线电压过低警告 (V) */
#define MOTOR_VOLTAGE_CRITICAL      18.0f   /* 母线电压过低危险 (V) */
#define MOTOR_VOLTAGE_OVER          50.0f   /* 母线电压过高压 (V) */

#define MOTOR_TORQUE_WARNING        300.0f   /* 转矩警告阈值 (Nm) */
#define MOTOR_TORQUE_CRITICAL       400.0f   /* 转矩危险阈值 (Nm) */

/* 反馈超时时间定义在 quad_command_fusion.h 中 */
#define MOTOR_ERROR_FLAG_NONE          0x0000
#define MOTOR_ERROR_OVER_TEMP          0x0100  /* 电机过温 */
#define MOTOR_ERROR_UNDER_VOLTAGE     0x0200  /* 欠压 */
#define MOTOR_ERROR_OVER_VOLTAGE      0x0400  /* 过压 */
#define MOTOR_ERROR_TORQUE_OVER       0x0800  /* 转矩过载 */
#define MOTOR_BATCH_MAX_SIZE        12      /* 批量命令最大数量 */
/* 反馈超时时间定义在 quad_command_fusion.h 中 */

/* ============================================================================
 * 电机句柄结构体（扩展）
 * ============================================================================ */

/**
 * @brief 电机控制句柄（完整状态）
 * @note  整合了电机的所有状态信息，便于上层访问
 */
typedef struct {
    /* 基本信息 */
    uint8_t can_channel;            /* CAN通道（1或2）*/
    uint8_t device_id;              /* 设备ID */
    uint8_t motor_index;            /* 电机索引（0-11）*/
    bool initialized;               /* 是否已初始化 */

    /* 状态信息 */
    MotorStatus_t status;           /* 基本状态 */
    MotorMonitorData_t monitor;     /* 监控数据 */

    /* 目标值 */
    float target_position;          /* 目标位置（度）*/
    float target_speed;             /* 目标速度（rpm）*/
    float target_torque;            /* 目标力矩（Nm）*/

    /* 当前值（从monitor数据更新）*/
    float current_position;         /* 当前位置（度）*/
    float current_velocity;         /* 当前速度（rpm）*/
    float current_torque;           /* 当前力矩（Nm）*/
    float temperature;              /* 电机温度（℃）*/
    float board_temperature;       /* V9.0: 电路板温度（℃）*/
    float bus_voltage;              /* 母线电压（V）*/
    float current;                  /* 电流（A）- 从力矩估算 */

    /* 限制参数（缓存）*/
    float torque_limit;             /* 力矩限制（Nm）*/
    float speed_limit;              /* 速度限制（rpm）*/
    float accel_limit;              /* 加速度限制（rpm/s）*/
    float pos_kp;                   /* 位置环Kp */

    /* 错误信息 */
    uint16_t error_flags;           /* 错误标志位 */
    uint32_t last_error_time;       /* 最后错误时间（ms）*/

    /* 反馈统计 */
    uint32_t last_feedback_time;    /* 最后反馈时间（ms）*/
    uint32_t feedback_count;        /* 反馈计数 */
    uint32_t feedback_timeout_count;/* 反馈超时计数 */

} MotorHandle;

/* ============================================================================
 * 批量控制结构体
 * ============================================================================ */

/**
 * @brief 批量位置命令
 */
typedef struct {
    uint8_t motor_ids[MOTOR_BATCH_MAX_SIZE];    /* 电机索引数组 */
    float positions[MOTOR_BATCH_MAX_SIZE];       /* 位置数组 */
    uint8_t count;                               /* 数量 */
} BatchPositionCommand_t;

/**
 * @brief 批量参数命令（相同参数）
 */
typedef struct {
    uint8_t motor_ids[MOTOR_BATCH_MAX_SIZE];    /* 电机索引数组 */
    uint8_t count;                               /* 数量 */
    float value;                                 /* 参数值 */
} BatchParamCommand_t;

/* ============================================================================
 * 函数声明 - 电机句柄管理
 * ============================================================================ */

/**
 * @brief 初始化电机句柄系统
 * @return 0=成功, 非0=失败
 */
uint8_t motor_handles_init(void);

/**
 * @brief 创建电机句柄
 * @param motor_index 电机索引（0-11）
 * @param can_channel CAN通道（1或2）
 * @param device_id 设备ID
 * @return 0=成功, 非0=失败
 */
uint8_t motor_handle_create(uint8_t motor_index, uint8_t can_channel, uint8_t device_id);

/**
 * @brief 获取电机句柄
 * @param motor_index 电机索引（0-11）
 * @return 电机句柄指针，失败返回NULL
 */
MotorHandle* motor_handle_get(uint8_t motor_index);

/**
 * @brief 根据CAN通道和设备ID获取电机句柄
 * @param can_channel CAN通道（1或2）
 * @param device_id 设备ID
 * @return 电机句柄指针，失败返回NULL
 */
MotorHandle* get_motor_handle(uint8_t can_channel, uint8_t device_id);

/**
 * @brief 更新电机句柄的监控数据
 * @param motor_index 电机索引（0-11）
 * @param monitor 监控数据指针
 * @return 0=成功, 非0=失败
 */
uint8_t motor_handle_update_monitor(uint8_t motor_index, const MotorMonitorData_t* monitor);

/**
 * @brief 更新电机句柄的错误状态
 * @param motor_index 电机索引（0-11）
 * @param error_code 错误码
 * @return 0=成功, 非0=失败
 */
uint8_t motor_handle_update_error(uint8_t motor_index, MotorError_t error_code);

/**
 * @brief 更新电机句柄的运行状态（启动/停止反馈）
 * @param motor_index 电机索引（0-11）
 * @param running 运行状态（0=停止, 1=运行）
 * @param enabled 使能状态（0=失能, 1=使能）
 * @return 0=成功, 非0=失败
 */
uint8_t motor_handle_update_running(uint8_t motor_index, uint8_t running, uint8_t enabled);

/**
 * @brief 检查电机反馈是否超时
 * @param motor_index 电机索引（0-11）
 * @return true=超时, false=正常
 */
bool motor_handle_is_feedback_timeout(uint8_t motor_index);

/* ============================================================================
 * 函数声明 - 批量控制
 * ============================================================================ */

/**
 * @brief 批量设置位置目标
 * @param motor_ids 电机索引数组
 * @param positions 位置数组
 * @param count 数量
 * @return 成功发送的命令数
 */
uint8_t motor_batch_set_positions(uint8_t* motor_ids, float* positions, uint8_t count);

/**
 * @brief 批量设置相同力矩限制
 * @param motor_ids 电机索引数组
 * @param count 数量
 * @param torque_limit 力矩限制值（Nm）
 * @return 成功发送的命令数
 */
uint8_t motor_batch_set_torque_limit(uint8_t* motor_ids, uint8_t count, float torque_limit);

/**
 * @brief 批量设置相同速度限制
 * @param motor_ids 电机索引数组
 * @param count 数量
 * @param speed_limit 速度限制值（rpm）
 * @return 成功发送的命令数
 */
uint8_t motor_batch_set_speed_limit(uint8_t* motor_ids, uint8_t count, float speed_limit);

/**
 * @brief 批量设置相同加速度限制
 * @param motor_ids 电机索引数组
 * @param count 数量
 * @param accel_limit 加速度限制值（rpm/s）
 * @return 成功发送的命令数
 */
uint8_t motor_batch_set_accel_limit(uint8_t* motor_ids, uint8_t count, float accel_limit);

/**
 * @brief 批量设置相同位置环Kp
 * @param motor_ids 电机索引数组
 * @param count 数量
 * @param pos_kp 位置环Kp值
 * @return 成功发送的命令数
 */
uint8_t motor_batch_set_pos_kp(uint8_t* motor_ids, uint8_t count, float pos_kp);

/**
 * @brief 批量使能电机
 * @param motor_ids 电机索引数组
 * @param count 数量
 * @param enable true=使能, false=失能
 * @return 成功发送的命令数
 */
uint8_t motor_batch_enable(uint8_t* motor_ids, uint8_t count, bool enable);

/* ============================================================================
 * 函数声明 - 反馈优化
 * ============================================================================ */

/**
 * @brief 请求所有电机的监控数据
 * @return 成功发送的命令数
 */
uint8_t motor_request_all_monitor_data(void);

/**
 * @brief 处理电机监控数据反馈
 * @param can_channel CAN通道
 * @param device_id 设备ID
 * @param rxdata 接收数据
 * @param data_len 数据长度
 * @note 内部调用，由CAN接收中断调用
 */
void motor_process_monitor_feedback(uint8_t can_channel, uint8_t device_id,
                                     const uint8_t* rxdata, uint8_t data_len);

/**
 * @brief 打印所有电机状态
 */
void motor_print_all_status(void);

/* ============================================================================
 * 函数声明 - 紧急功能
 * ============================================================================ */

/**
 * @brief 紧急停止单个电机
 * @param motor_index 电机索引（0-11）
 */
void motor_emergency_stop(uint8_t can_channel, uint8_t device_id);

/**
 * @brief 禁用单个电机
 * @param can_channel CAN通道
 * @param device_id 设备ID
 */
void motor_disable(uint8_t can_channel, uint8_t device_id);

/**
 * @brief 清除单个电机错误
 * @param can_channel CAN通道
 * @param device_id 设备ID
 */
void motor_clear_errors(uint8_t can_channel, uint8_t device_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_ENHANCED_H */
