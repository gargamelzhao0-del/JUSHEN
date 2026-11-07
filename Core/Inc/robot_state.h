/**
 ******************************************************************************
 * @file    robot_state.h
 * @brief   四足机器狗系统状态管理模块
 * @note    提供统一的状态机和状态查询接口，协调各子系统
 ******************************************************************************
 */

#ifndef __ROBOT_STATE_H
#define __ROBOT_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 系统状态定义
 * ============================================================================ */

/**
 * @brief 机器狗系统状态枚举
 */
typedef enum {
    ROBOT_STATE_INIT,           /* 系统初始化中 */
    ROBOT_STATE_IDLE,           /* 空闲状态（电机未使能） */
    ROBOT_STATE_READY,          /* 就绪状态（电机使能，等待命令） */
    ROBOT_STATE_RUNNING,        /* 运行状态（正在执行运动命令） */
    ROBOT_STATE_ERROR,          /* 错误状态（需要处理故障） */
    ROBOT_STATE_EMERGENCY       /* 紧急停止状态 */
} RobotState_t;

/**
 * @brief 系统错误标志位（位掩码）
 */
typedef enum {
    ERROR_NONE              = 0x0000,
    ERROR_MOTOR_FAULT       = 0x0001,   /* 电机故障 */
    ERROR_CAN_BUS_OFF       = 0x0002,   /* CAN总线故障 */
    ERROR_TCP_DISCONNECT    = 0x0004,   /* TCP连接断开 */
    ERROR_COMMAND_TIMEOUT   = 0x0008,   /* 命令超时 */
    ERROR_OVERHEAT          = 0x0010,   /* 过热 */
    ERROR_VOLTAGE_FAULT     = 0x0020,   /* 电压异常 */
    ERROR_WATCHDOG          = 0x0040,   /* 看门狗超时 */
    ERROR_CRITICAL          = 0x8000    /* 严重错误（需要重启） */
} SystemError_t;

/**
 * @brief 紧急停止级别
 */
typedef enum {
    ESTOP_LEVEL_SOFT = 0,      /* 软停止：平滑减速停止 */
    ESTOP_LEVEL_HARD = 1,      /* 硬停止：立即停止 */
    ESTOP_LEVEL_POWER_CUT = 2  /* 断电停止：切断电机电源 */
} EmergencyStopLevel_t;

/**
 * @brief 系统运行统计
 */
typedef struct {
    uint32_t uptime_ms;             /* 系统运行时间（毫秒） */
    uint32_t command_count;         /* 接收到的命令总数 */
    uint32_t error_count;           /* 错误总数 */
    uint32_t motor_command_sent;    /* 发送的电机命令数 */
    uint32_t tcp_tx_bytes;          /* TCP发送字节数 */
    uint32_t tcp_rx_bytes;          /* TCP接收字节数 */
    uint16_t free_heap_kb;          /* 剩余堆空间（KB） */
} SystemStats_t;

/* ============================================================================
 * 公共API
 * ============================================================================ */

/**
 * @brief  初始化机器狗状态管理模块
 * @retval 0=成功, -1=失败
 */
int robot_state_init(void);

/**
 * @brief  启动机器狗状态管理任务（FreeRTOS任务）
 * @retval 0=成功, -1=失败
 */
int robot_state_start_task(void);

/**
 * @brief  获取当前系统状态
 * @retval 当前状态
 */
RobotState_t robot_state_get(void);

/**
 * @brief  请求状态转换
 * @param  new_state: 目标状态
 * @retval true=转换成功, false=转换被拒绝
 */
bool robot_state_request_transition(RobotState_t new_state);

/**
 * @brief  设置系统错误标志
 * @param  error: 错误标志（可以是多个标志的OR组合）
 */
void robot_state_set_error(SystemError_t error);

/**
 * @brief  清除系统错误标志
 * @param  error: 要清除的错误标志
 */
void robot_state_clear_error(SystemError_t error);

/**
 * @brief  获取当前系统错误标志
 * @retval 错误标志（位掩码）
 */
uint16_t robot_state_get_errors(void);

/**
 * @brief  检查系统是否有错误
 * @retval true=有错误, false=无错误
 */
bool robot_state_has_error(void);

/**
 * @brief  获取系统运行统计信息
 * @param  stats: 输出统计信息结构体指针
 */
void robot_state_get_stats(SystemStats_t *stats);

/**
 * @brief  更新命令时间戳（用于超时检测）
 * @note   每次收到上位机命令时调用
 */
void robot_state_update_command_timestamp(void);

/**
 * @brief  检查命令是否超时
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval true=超时, false=未超时
 */
bool robot_state_is_command_timeout(uint32_t timeout_ms);

/**
 * @brief  紧急停止（立即停止所有电机）
 * @note   可以从任何上下文调用（包括中断）
 */
void robot_state_emergency_stop(void);

/**
 * @brief  多级紧急停止
 * @param  level: 停止级别
 * @note   可以从任何上下文调用（包括中断）
 *         SOFT: 平滑减速停止（保持使能）
 *         HARD: 立即停止（失能电机）
 *         POWER_CUT: 切断电机电源（硬件信号）
 */
void robot_state_emergency_stop_ex(EmergencyStopLevel_t level);

/**
 * @brief  检查是否处于紧急停止状态
 * @retval true=紧急停止中, false=正常
 */
bool robot_state_is_emergency_stopped(void);

/**
 * @brief  从紧急停止状态恢复
 * @retval 0=成功, -1=失败（有未清除的错误）
 * @note   只有清除所有错误后才能恢复
 */
int robot_state_resume_from_emergency(void);

/**
 * @brief  获取状态名称字符串（用于调试）
 * @param  state: 状态枚举值
 * @retval 状态名称字符串
 */
const char* robot_state_get_name(RobotState_t state);

/**
 * @brief  打印系统状态信息（通过日志系统）
 */
void robot_state_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_STATE_H */
