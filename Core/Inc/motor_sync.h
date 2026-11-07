/**
 ******************************************************************************
 * @file    motor_sync.h
 * @brief   电机同步控制模块 - 解决12个电机命令同步问题
 * @note    提供基于时间戳的同步发送机制，确保所有电机同时启动
 ******************************************************************************
 */

#ifndef __MOTOR_SYNC_H
#define __MOTOR_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "motor_control.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "cmsis_os.h"

/* ============================================================================
 * 电机同步配置
 * ============================================================================ */

#define MOTOR_SYNC_MAX_MOTORS   12      /* 最大电机数量 */
#define MOTOR_SYNC_TIMEOUT_MS   10      /* 命令发送超时（毫秒） */

/**
 * @brief 电机命令同步模式
 */
typedef enum {
    SYNC_MODE_IMMEDIATE,    /* 立即发送（无同步） */
    SYNC_MODE_BATCH,        /* 批量发送（所有命令一起发送） */
    SYNC_MODE_TIMESTAMP,    /* 基于时间戳同步（预留执行时间） */
    SYNC_MODE_PARALLEL,     /* 并行发送（CAN1和CAN2同时发送） */
    SYNC_MODE_PRIORITY      /* 优先级发送（按优先级排序后发送） */
} MotorSyncMode_t;

/**
 * @brief 电机命令类型
 */
typedef enum {
    MOTOR_CMD_ENABLE,       /* 使能命令 */
    MOTOR_CMD_DISABLE,      /* 失能命令 */
    MOTOR_CMD_POSITION,     /* 位置控制命令 */
    MOTOR_CMD_VELOCITY,     /* 速度控制命令 */
    MOTOR_CMD_TORQUE,       /* 力矩控制命令 */
    MOTOR_CMD_STOP          /* 停止命令 */
} MotorCommandType_t;

/**
 * @brief 电机命令结构（扩展）
 */
typedef struct {
    uint8_t  motor_id;          /* 电机ID (0x01-0x0C) */
    uint8_t  can_port;          /* CAN端口 (1=FDCAN1, 2=FDCAN2) */
    MotorCommandType_t type;    /* 命令类型 */

    /* 命令参数（根据类型使用） */
    union {
        struct {
            float position;     /* 位置模式：目标位置（rad） */
            float max_speed;    /* 位置模式：最大速度（rad/s） */
        } pos;

        struct {
            float speed;        /* 速度模式：目标速度（rad/s） */
        } vel;

        struct {
            float torque;       /* 力矩模式：目标力矩（N·m） */
        } trq;
    } params;

    uint32_t timestamp_us;      /* 期望执行时间戳（微秒，用于时间戳模式） */
    uint8_t  priority;          /* 命令优先级 (0=lowest, 255=highest) */
} MotorSyncCommand_t;

/**
 * @brief 同步发送结果
 */
typedef struct {
    uint8_t  total_motors;      /* 总电机数 */
    uint8_t  success_count;     /* 成功发送的电机数 */
    uint8_t  failed_count;      /* 失败的电机数 */
    uint32_t send_duration_us;  /* 发送耗时（微秒） */
    uint8_t  failed_ids[MOTOR_SYNC_MAX_MOTORS];  /* 失败的电机ID列表 */
} MotorSyncResult_t;

/* ============================================================================
 * 公共API
 * ============================================================================ */

/**
 * @brief  初始化电机同步控制模块
 * @param  mode: 同步模式
 * @retval 0=成功, -1=失败
 */
int motor_sync_init(MotorSyncMode_t mode);

/**
 * @brief  设置同步模式
 * @param  mode: 同步模式
 */
void motor_sync_set_mode(MotorSyncMode_t mode);

/**
 * @brief  获取当前同步模式
 * @retval 同步模式
 */
MotorSyncMode_t motor_sync_get_mode(void);

/**
 * @brief  同步发送多个电机命令（批量模式）
 * @param  commands: 命令数组
 * @param  count: 命令数量（最多12个）
 * @param  result: 输出发送结果（可选，传NULL跳过）
 * @retval 0=全部成功, -1=有失败
 * @note   所有命令会尽可能快地连续发送，实现准同步
 */
int motor_sync_send_batch(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);

/**
 * @brief  发送12个电机的位置命令（快捷函数）
 * @param  positions: 12个电机的目标位置数组（rad）
 * @param  max_speed: 最大速度（rad/s），所有电机统一
 * @param  result: 输出发送结果（可选）
 * @retval 0=全部成功, -1=有失败
 */
int motor_sync_send_all_position(const float positions[12], float max_speed, MotorSyncResult_t *result);

/**
 * @brief  发送12个电机的速度命令（快捷函数）
 * @param  velocities: 12个电机的目标速度数组（rad/s）
 * @param  result: 输出发送结果（可选）
 * @retval 0=全部成功, -1=有失败
 */
int motor_sync_send_all_velocity(const float velocities[12], MotorSyncResult_t *result);

/**
 * @brief  发送12个电机的力矩命令（快捷函数）
 * @param  torques: 12个电机的目标力矩数组（N·m）
 * @param  result: 输出发送结果（可选）
 * @retval 0=全部成功, -1=有失败
 */
int motor_sync_send_all_torque(const float torques[12], MotorSyncResult_t *result);

/**
 * @brief  同步启用所有电机
 * @param  result: 输出发送结果（可选）
 * @retval 0=全部成功, -1=有失败
 */
int motor_sync_enable_all(MotorSyncResult_t *result);

/**
 * @brief  同步禁用所有电机
 * @param  result: 输出发送结果（可选）
 * @retval 0=全部成功, -1=有失败
 */
int motor_sync_disable_all(MotorSyncResult_t *result);

/**
 * @brief  获取同步发送统计信息
 * @param  total_sent: 输出总发送次数（可选）
 * @param  total_failed: 输出总失败次数（可选）
 * @param  avg_duration_us: 输出平均发送耗时（微秒）（可选）
 */
void motor_sync_get_stats(uint32_t *total_sent, uint32_t *total_failed, uint32_t *avg_duration_us);

/**
 * @brief  获取扩展统计信息（包含抖动监控）
 * @param  total_sent: 输出总发送次数（可选）
 * @param  total_failed: 输出总失败次数（可选）
 * @param  avg_duration_us: 输出平均发送耗时（微秒）（可选）
 * @param  min_duration_us: 输出最小发送耗时（微秒）（可选）
 * @param  max_duration_us: 输出最大发送耗时（微秒）（可选）
 * @param  jitter_us: 输出抖动（最大-最小，微秒）（可选）
 */
void motor_sync_get_stats_extended(
    uint32_t *total_sent,
    uint32_t *total_failed,
    uint32_t *avg_duration_us,
    uint32_t *min_duration_us,
    uint32_t *max_duration_us,
    uint32_t *jitter_us
);

/**
 * @brief  重置统计信息
 */
void motor_sync_reset_stats(void);

/**
 * @brief  打印同步发送结果（调试用）
 * @param  result: 发送结果
 */
void motor_sync_print_result(const MotorSyncResult_t *result);

/**
 * @brief  设置电机发送使能状态
 * @param  enabled: true=启用发送, false=禁用发送
 * @note   禁用后，所有电机命令将被忽略（不发送到CAN总线）
 *         用于测试或调试其他功能时临时停止电机控制
 */
void motor_sync_set_send_enabled(bool enabled);

/**
 * @brief  获取电机发送使能状态
 * @retval true=已启用, false=已禁用
 */
bool motor_sync_get_send_enabled(void);

/* ============================================================================ 
 * 高级功能（可选）
 * ============================================================================ */

/**
 * @brief  设置电机ID映射表（用于非标准布局）
 * @param  motor_map: 电机映射表（12个元素）
 *                    motor_map[i] = {motor_id, can_port}
 * @note   默认映射：电机0-5在CAN1，电机6-11在CAN2
 *         ID从0x01开始
 */
void motor_sync_set_motor_map(const uint8_t motor_map[12][2]);

/**
 * @brief  获取当前电机映射表
 * @param  motor_map: 输出映射表（12个元素）
 */
void motor_sync_get_motor_map(uint8_t motor_map[12][2]);

/**
 * @brief  基于时间戳同步发送命令
 * @param  commands: 命令数组
 * @param  count: 命令数量
 * @param  timestamp_us: 执行时间戳（微秒）
 * @param  result: 输出发送结果
 * @retval 0=成功, -1=失败
 * @note   所有命令会在指定的时间戳同时执行
 */
int motor_sync_send_timestamp(const MotorSyncCommand_t *commands, uint8_t count, uint32_t timestamp_us, MotorSyncResult_t *result);

/**
 * @brief  并行发送命令（CAN1和CAN2同时发送）
 * @param  commands: 命令数组
 * @param  count: 命令数量
 * @param  result: 输出发送结果
 * @retval 0=成功, -1=失败
 * @note   利用双CAN总线并行发送，提高发送速度
 */

/**
 * @brief  发送带优先级的命令批处理
 * @param  commands: 命令数组
 * @param  count: 命令数量
 * @param  result: 输出发送结果
 * @retval 0=成功, -1=失败
 * @note   高优先级命令会先发送
 */
int motor_sync_send_priority(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);

/**
 * @brief  紧急停止所有电机（高优先级）
 * @param  result: 输出发送结果
 * @retval 0=成功, -1=失败
 * @note   最高优先级发送，确保所有电机尽快停止
 */
int motor_sync_emergency_stop_all(MotorSyncResult_t *result);

/**
 * @brief  获取电机发送状态
 * @param  motor_id: 电机ID
 * @retval true=发送成功, false=发送失败或未发送
 */
bool motor_sync_get_motor_status(uint8_t motor_id);

/**
 * @brief  设置命令重试次数
 * @param  retry_count: 重试次数（0-5）
 */
void motor_sync_set_retry_count(uint8_t retry_count);

/**
 * @brief  获取命令重试次数
 * @retval 当前重试次数
 */
uint8_t motor_sync_get_retry_count(void);

/* ============================================================================
 * 双CAN总线联动控制
 * ============================================================================ */

/**
 * @brief 双CAN总线状态结构
 */
typedef struct {
    bool can1_healthy;              /* CAN1健康状态 */
    bool can2_healthy;              /* CAN2健康状态 */
    uint32_t can1_last_success_tick; /* CAN1上次成功时间 */
    uint32_t can2_last_success_tick; /* CAN2上次成功时间 */
    uint32_t can1_error_count;      /* CAN1连续错误计数 */
    uint32_t can2_error_count;      /* CAN2连续错误计数 */
    uint32_t can1_total_sent;       /* CAN1总发送数 */
    uint32_t can2_total_sent;       /* CAN2总发送数 */
    uint32_t can1_total_failed;     /* CAN1总失败数 */
    uint32_t can2_total_failed;     /* CAN2总失败数 */
} DualCANStatus_t;

/**
 * @brief 初始化双CAN总线联动控制
 * @retval 0=成功, -1=失败
 */
int dual_can_init(void);

/**
 * @brief 检查双CAN总线健康状态
 * @retval true=两条总线都健康, false=至少一条总线故障
 */
bool dual_can_check_health(void);

/**
 * @brief 获取双CAN总线状态
 * @param status: 输出状态结构
 */
void dual_can_get_status(DualCANStatus_t *status);

/**
 * @brief CAN发送成功回调（由底层调用）
 * @param can_channel: CAN通道 (1或2)
 */
void dual_can_on_send_success(uint8_t can_channel);

/**
 * @brief CAN发送失败回调（由底层调用）
 * @param can_channel: CAN通道 (1或2)
 */
void dual_can_on_send_error(uint8_t can_channel);

/**
 * @brief 设置CAN总线故障阈值
 * @param timeout_ms: 超时时间（毫秒）
 * @param error_count: 连续错误次数阈值
 */
void dual_can_set_threshold(uint32_t timeout_ms, uint32_t error_count);

/**
 * @brief 重置CAN总线状态（用于故障恢复）
 * @param can_channel: CAN通道 (1或2, 0=全部)
 */
void dual_can_reset(uint8_t can_channel);

/**
 * @brief 检查是否允许发送命令
 * @retval true=允许发送, false=禁止发送（总线故障）
 */
bool dual_can_can_send(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SYNC_H */
