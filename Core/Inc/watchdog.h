/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   系统看门狗模块 - 硬件看门狗(IWDG) + 软件任务监控
 * @note    包含独立看门狗和任务心跳监控功能
 ******************************************************************************
 */

#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "system_config.h"  /* TCP_ENABLED配置 */
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 配置参数
 * ============================================================================ */

/**
 * @brief  看门狗超时时间配置
 */
#define WDG_IWDG_TIMEOUT_MS         3000    /* V10.0: 硬件看门狗超时: 3秒 (从5秒缩短) */
#define WDG_TASK_TIMEOUT_MS         2000    /* V10.0: 任务心跳超时: 2秒 (从3秒缩短) */
#define WDG_CHECK_INTERVAL_MS       500     /* V10.0: 看门狗检查间隔: 500ms (从1秒缩短) */
#define WDG_EARLY_WARNING_MS        1500    /* V10.0: 早期警告阈值: 1.5秒 */

/**
 * @brief  监控的关键任务ID
 */
typedef enum {
    WDG_TASK_DEFAULT = 0,          /* DefaultTask (主任务) */
    WDG_TASK_MOTOR_CTRL,           /* MotorControlLoopTask (电机控制) */
#if TCP_ENABLED
    WDG_TASK_TCP_SEND,             /* TCPSendTask (TCP发送) */
#endif
    WDG_TASK_FDCAN_RX,             /* FDCANRxTask (CAN接收) */
    WDG_TASK_WATCHDOG,             /* WatchdogTask (看门狗自身) */
    WDG_TASK_POWER_MONITOR,        /* PowerMonitorTask (电源监控) */

    WDG_TASK_MAX                   /* 任务总数 */
} WDG_TaskID_t;

/**
 * @brief  任务健康状态
 */
typedef enum {
    WDG_HEALTH_OK = 0,             /* 任务健康 */
    WDG_HEALTH_WARNING,            /* 任务警告（心跳较慢）*/
    WDG_HEALTH_TIMEOUT,            /* 任务超时（可能卡死）*/
    WDG_HEALTH_NOT_STARTED         /* 任务未启动 */
} WDG_HealthStatus_t;

/**
 * @brief  任务监控信息
 */
typedef struct {
    const char *task_name;         /* 任务名称 */
    uint32_t last_heartbeat_ms;    /* 上次心跳时间戳 */
    uint32_t heartbeat_count;      /* 心跳计数 */
    uint32_t timeout_count;        /* 超时次数 */
    WDG_HealthStatus_t status;     /* 健康状态 */
    bool enabled;                  /* 是否启用监控 */
} WDG_TaskInfo_t;

/* ============================================================================
 * 公共API - 初始化和控制
 * ============================================================================ */

/**
 * @brief  初始化看门狗系统
 * @retval 0=成功, -1=失败
 * @note   初始化IWDG和任务监控系统
 */
int watchdog_init(void);

/**
 * @brief  启动硬件看门狗（IWDG）
 * @retval 0=成功, -1=失败
 * @note   一旦启动，IWDG将无法停止，必须定期喂狗
 */
int watchdog_start_iwdg(void);

/**
 * @brief  喂硬件看门狗（IWDG）
 * @note   必须在超时前调用，否则系统重启
 */
void watchdog_feed_iwdg(void);

/**
 * @brief  检查系统重启原因
 * @retval true=上次复位由看门狗触发, false=其他原因
 */
bool watchdog_check_reset_reason(void);

/* ============================================================================
 * 公共API - 任务心跳监控
 * ============================================================================ */

/**
 * @brief  注册任务心跳（任务定期调用此函数）
 * @param  task_id: 任务ID
 * @note   每个关键任务应定期调用此函数更新心跳
 */
void watchdog_feed_task(WDG_TaskID_t task_id);

/**
 * @brief  使能/禁用任务监控
 * @param  task_id: 任务ID
 * @param  enabled: true=启用监控, false=禁用监控
 */
void watchdog_set_task_enabled(WDG_TaskID_t task_id, bool enabled);

/**
 * @brief  获取任务健康状态
 * @param  task_id: 任务ID
 * @retval 任务健康状态
 */
WDG_HealthStatus_t watchdog_get_task_health(WDG_TaskID_t task_id);

/**
 * @brief  获取任务监控信息
 * @param  task_id: 任务ID
 * @param  info: 输出任务信息
 * @retval 0=成功, -1=失败
 */
int watchdog_get_task_info(WDG_TaskID_t task_id, WDG_TaskInfo_t *info);

/* ============================================================================
 * 公共API - 看门狗任务
 * ============================================================================ */

/**
 * @brief  看门狗任务入口函数
 * @param  argument: FreeRTOS任务参数（未使用）
 * @note   这是FreeRTOS任务函数，由系统调用
 */
void StartWatchdogTask(void *argument);

/**
 * @brief  检查所有任务健康状态
 * @note   内部函数，由看门狗任务调用
 */
void watchdog_check_all_tasks(void);

/**
 * @brief  打印看门狗统计信息
 */
void watchdog_print_stats(void);

/* ============================================================================
 * 工具函数
 * ============================================================================ */

/**
 * @brief  获取任务名称字符串
 * @param  task_id: 任务ID
 * @retval 任务名称
 */
const char* watchdog_get_task_name(WDG_TaskID_t task_id);

/**
 * @brief  获取健康状态字符串
 * @param  status: 健康状态
 * @retval 状态描述
 */
const char* watchdog_get_health_string(WDG_HealthStatus_t status);

/* ============================================================================
 * 测试辅助函数（仅调试使用）
 * ============================================================================ */

/**
 * @brief  模拟看门狗任务卡死（用于测试IWDG复位）
 * @param  duration_ms: 卡死持续时间（毫秒），0=永久卡死
 * @note   ⚠️ 警告：此函数会导致系统复位，仅用于测试！
 */
void watchdog_test_hang(uint32_t duration_ms);

/**
 * @brief  获取测试标志状态（用于其他任务模拟卡死）
 * @param  task_id: 任务ID
 * @retval true=该任务应模拟卡死, false=正常运行
 */
bool watchdog_test_should_hang(WDG_TaskID_t task_id);

/**
 * @brief  触发任务卡死测试
 * @param  task_id: 要测试的任务ID
 * @param  duration_ms: 卡死持续时间（毫秒）
 * @note   调用后，对应任务会在下一个循环中暂停指定时间
 */
void watchdog_test_trigger_hang(WDG_TaskID_t task_id, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
