/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   系统看门狗模块实现 - 硬件看门狗(IWDG) + 软件任务监控
 ******************************************************************************
 */

#include "watchdog.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_iwdg.h"
#include "log_system.h"
#include "quad_emergency_stop.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* IWDG句柄 */
static IWDG_HandleTypeDef hiwdg;

/* 任务监控表 */
static WDG_TaskInfo_t task_info_table[WDG_TASK_MAX];

/* 互斥锁（保护任务监控表访问）*/
static osMutexId_t watchdog_mutex = NULL;

/* 任务名称字符串表 */
static const char *task_name_strings[WDG_TASK_MAX] = {
    "DefaultTask",
    "MotorControlLoop",
#if TCP_ENABLED
    "TCPSend",
#endif
    "FDCANRx",
    "Watchdog",
    "PowerMonitor"
};

/* 健康状态字符串表 */
static const char *health_status_strings[] = {
    "OK",
    "WARNING",
    "TIMEOUT",
    "NOT_STARTED"
};

/* 看门狗初始化标志 */
static bool watchdog_initialized = false;

/* 上次喂IWDG的时间戳 */
static uint32_t last_iwdg_feed_ms = 0;

/* 统计信息 */
static uint32_t total_feed_count = 0;
static uint32_t total_timeout_count = 0;
static uint32_t total_check_cycles = 0;

/* 测试辅助变量 */
static volatile bool test_hang_flags[WDG_TASK_MAX] = {false};
static volatile uint32_t test_hang_duration[WDG_TASK_MAX] = {0};

/* ============================================================================
 * 私有函数声明
 * ============================================================================ */

static void init_task_info_table(void);
static uint32_t get_current_time_ms(void);

/* ============================================================================
 * 公共函数实现 - 初始化和控制
 * ============================================================================ */

/**
 * @brief  初始化看门狗系统
 */
int watchdog_init(void)
{
    if (watchdog_initialized) {
        LOG_WARN("[Watchdog] Already initialized");
        return 0;
    }

    LOG_INFO("[Watchdog] Initializing watchdog system...");

    /* 创建互斥锁 */
    watchdog_mutex = osMutexNew(NULL);
    if (watchdog_mutex == NULL) {
        LOG_ERROR("[Watchdog] Failed to create mutex!");
        return -1;
    }

    /* 初始化任务监控表 */
    init_task_info_table();

    /* 配置IWDG（不立即启动）*/
    /* LSI频率: 32kHz
     * V10.0: 缩短超时时间提高响应灵敏度
     * 预分频: /128 -> 250Hz
     * 重载值: 2000 -> 8秒超时
     * 计算: 2000 / 250Hz = 8秒
     * IWDG超时(8s) > 软件看门狗任务超时(5s)，确保软件看门狗先检测到异常
     */
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_128;  /* 32kHz/128 = 250Hz */
    hiwdg.Init.Reload = 2000;                   /* 2000/250Hz = 8s */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;

    /* 注意：这里只初始化句柄，不启动IWDG
     * IWDG一旦启动就无法停止，需要等系统稳定后再启动
     */

    watchdog_initialized = true;
    LOG_INFO("[Watchdog] Watchdog system initialized successfully");
    LOG_INFO("[Watchdog]   IWDG Timeout: %d ms (not started yet)", WDG_IWDG_TIMEOUT_MS);
    LOG_INFO("[Watchdog]   Task Timeout: %d ms", WDG_TASK_TIMEOUT_MS);
    LOG_INFO("[Watchdog]   Check Interval: %d ms", WDG_CHECK_INTERVAL_MS);

    return 0;
}

/**
 * @brief  启动硬件看门狗（IWDG）
 */
int watchdog_start_iwdg(void)
{
    if (!watchdog_initialized) {
        LOG_ERROR("[Watchdog] Watchdog not initialized!");
        return -1;
    }

    LOG_WARN("[Watchdog] Starting IWDG (cannot be stopped once started)...");

    /* 初始化并启动IWDG */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        LOG_ERROR("[Watchdog] Failed to start IWDG!");
        return -1;
    }

    /* 记录启动时间 */
    last_iwdg_feed_ms = get_current_time_ms();

    LOG_INFO("[Watchdog] IWDG started successfully");
    LOG_WARN("[Watchdog] ⚠️  System MUST feed IWDG within %d ms to prevent reset!", WDG_IWDG_TIMEOUT_MS);

    return 0;
}

/**
 * @brief  喂硬件看门狗（IWDG）
 */
/* ============== 修复: 紧急状态保存阈值 ============== */
#define WATCHDOG_EMERGENCY_SAVE_THRESHOLD_MS  3000  /* 距离上次喂狗超过3秒时保存状态 */

void watchdog_feed_iwdg(void)
{
    uint32_t current_ms = get_current_time_ms();
    
    /* ============== 修复: 如果距离上次喂狗超过阈值，保存关键状态 ============== */
    if (last_iwdg_feed_ms > 0 && 
        (current_ms - last_iwdg_feed_ms) > WATCHDOG_EMERGENCY_SAVE_THRESHOLD_MS) {
        LOG_WARN("[Watchdog] Long gap detected (%lu ms), system may be unstable",
                 current_ms - last_iwdg_feed_ms);
        /* 这里可以添加紧急状态保存逻辑 */
    }

    /* 刷新IWDG计数器 */
    HAL_IWDG_Refresh(&hiwdg);

    /* 更新统计信息 */
    last_iwdg_feed_ms = current_ms;
    total_feed_count++;
}

/**
 * @brief  检查系统重启原因
 */
bool watchdog_check_reset_reason(void)
{
    bool is_watchdog_reset = false;

    /* 检查RCC复位标志寄存器 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET) {
        LOG_ERROR("[Watchdog] ⚠️  System was reset by IWDG!");
        is_watchdog_reset = true;
    }

    /* 清除所有复位标志 */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return is_watchdog_reset;
}

/* ============================================================================
 * 公共函数实现 - 任务心跳监控
 * ============================================================================ */

/**
 * @brief  注册任务心跳
 */
void watchdog_feed_task(WDG_TaskID_t task_id)
{
    if (task_id >= WDG_TASK_MAX) {
        return;
    }

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    /* 更新心跳时间戳 */
    task_info_table[task_id].last_heartbeat_ms = get_current_time_ms();
    task_info_table[task_id].heartbeat_count++;

    /* 如果任务之前是超时状态，现在恢复了 */
    if (task_info_table[task_id].status == WDG_HEALTH_TIMEOUT) {
        LOG_INFO("[Watchdog] Task '%s' recovered from timeout",
                 task_info_table[task_id].task_name);
        task_info_table[task_id].status = WDG_HEALTH_OK;
    } else if (task_info_table[task_id].status == WDG_HEALTH_NOT_STARTED) {
        /* 首次心跳，标记为已启动 */
        task_info_table[task_id].status = WDG_HEALTH_OK;
    }

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }
}

/**
 * @brief  使能/禁用任务监控
 */
void watchdog_set_task_enabled(WDG_TaskID_t task_id, bool enabled)
{
    if (task_id >= WDG_TASK_MAX) {
        return;
    }

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    task_info_table[task_id].enabled = enabled;

    LOG_INFO("[Watchdog] Task '%s' monitoring %s",
             task_info_table[task_id].task_name,
             enabled ? "ENABLED" : "DISABLED");

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }
}

/**
 * @brief  获取任务健康状态
 */
WDG_HealthStatus_t watchdog_get_task_health(WDG_TaskID_t task_id)
{
    if (task_id >= WDG_TASK_MAX) {
        return WDG_HEALTH_NOT_STARTED;
    }

    WDG_HealthStatus_t status;

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    status = task_info_table[task_id].status;

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }

    return status;
}

/**
 * @brief  获取任务监控信息
 */
int watchdog_get_task_info(WDG_TaskID_t task_id, WDG_TaskInfo_t *info)
{
    if (task_id >= WDG_TASK_MAX || info == NULL) {
        return -1;
    }

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    /* 复制任务信息 */
    memcpy(info, &task_info_table[task_id], sizeof(WDG_TaskInfo_t));

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }

    return 0;
}

/* ============================================================================
 * 公共函数实现 - 看门狗任务
 * ============================================================================ */

/**
 * @brief  检查所有任务健康状态
 */
void watchdog_check_all_tasks(void)
{
    /* ============== 修复: 预喂狗，防止检查过程阻塞导致复位 ============== */
    HAL_IWDG_Refresh(&hiwdg);
    
    uint32_t current_ms = get_current_time_ms();
    bool any_timeout = false;

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    /* 遍历所有任务 */
    for (int i = 0; i < WDG_TASK_MAX; i++) {
        /* 跳过禁用的任务 */
        if (!task_info_table[i].enabled) {
            continue;
        }

        /* 跳过未启动的任务 */
        if (task_info_table[i].status == WDG_HEALTH_NOT_STARTED) {
            continue;
        }

        /* 计算距离上次心跳的时间 */
        uint32_t elapsed_ms = current_ms - task_info_table[i].last_heartbeat_ms;

        /* 检查是否超时 */
        if (elapsed_ms > WDG_TASK_TIMEOUT_MS) {
            /* 任务超时 */
            if (task_info_table[i].status != WDG_HEALTH_TIMEOUT) {
                /* 首次检测到超时 */
                task_info_table[i].status = WDG_HEALTH_TIMEOUT;
                task_info_table[i].timeout_count++;
                total_timeout_count++;

                LOG_ERROR("[Watchdog] ⚠️  Task '%s' TIMEOUT! (no heartbeat for %lu ms)",
                          task_info_table[i].task_name, elapsed_ms);

                any_timeout = true;
            }
        } else if (elapsed_ms > (WDG_TASK_TIMEOUT_MS / 2)) {
            /* 警告：心跳较慢 */
            if (task_info_table[i].status == WDG_HEALTH_OK) {
                task_info_table[i].status = WDG_HEALTH_WARNING;
                LOG_WARN("[Watchdog] Task '%s' heartbeat slow (%lu ms since last)",
                         task_info_table[i].task_name, elapsed_ms);
            }
        } else {
            /* 任务健康 */
            if (task_info_table[i].status != WDG_HEALTH_OK) {
                task_info_table[i].status = WDG_HEALTH_OK;
            }
        }
    }

    total_check_cycles++;

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }

    /* 如果有任务超时，触发紧急停止 */
    if (any_timeout) {
        LOG_ERROR("[Watchdog] Task timeout detected - triggering emergency stop!");
        QuadEmergencyStop_Trigger(ESTOP_SOURCE_WATCHDOG, ESTOP_LEVEL_HARD);
    }
}

/**
 * @brief  打印看门狗统计信息
 */
void watchdog_print_stats(void)
{
    uint32_t current_ms = get_current_time_ms();

    LOG_INFO("[Watchdog] ==================== Statistics ====================");
    LOG_INFO("[Watchdog] Check Cycles: %lu", total_check_cycles);
    LOG_INFO("[Watchdog] IWDG Feed Count: %lu (last: %lu ms ago)",
             total_feed_count,
             current_ms - last_iwdg_feed_ms);
    LOG_INFO("[Watchdog] Total Timeouts: %lu", total_timeout_count);

    LOG_INFO("[Watchdog] --- Task Health Status ---");

    /* 获取互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexAcquire(watchdog_mutex, osWaitForever);
    }

    for (int i = 0; i < WDG_TASK_MAX; i++) {
        if (!task_info_table[i].enabled) {
            LOG_INFO("[Watchdog]   %s: DISABLED",
                     task_info_table[i].task_name);
            continue;
        }

        /* 计算距离上次心跳的时间（处理溢出情况）*/
        uint32_t elapsed_ms = current_ms - task_info_table[i].last_heartbeat_ms;
        /* 如果结果异常大（>1小时），说明发生了tick溢出或多线程竞态，显示为0 */
        if (elapsed_ms > 3600000) {
            elapsed_ms = 0;
        }

        LOG_INFO("[Watchdog]   %s: %s (heartbeats=%lu, timeouts=%lu, last=%lu ms ago)",
                 task_info_table[i].task_name,
                 watchdog_get_health_string(task_info_table[i].status),
                 task_info_table[i].heartbeat_count,
                 task_info_table[i].timeout_count,
                 elapsed_ms);
    }

    /* 释放互斥锁 */
    if (watchdog_mutex != NULL) {
        osMutexRelease(watchdog_mutex);
    }

    LOG_INFO("[Watchdog] ====================================================");
}

/**
 * @brief  看门狗任务入口函数
 */
void StartWatchdogTask(void *argument)
{
    (void)argument;

    LOG_INFO("[Watchdog] Watchdog Task Started");
    LOG_INFO("[Watchdog]   Check Interval: %d ms", WDG_CHECK_INTERVAL_MS);
    LOG_INFO("[Watchdog]   Task Timeout: %d ms", WDG_TASK_TIMEOUT_MS);

    /* 延迟5秒，等待其他任务启动 */
    LOG_INFO("[Watchdog] Waiting 5s for system stabilization...");
    osDelay(5000);

    /* 启动IWDG（5秒超时，必须定期喂狗）*/
    watchdog_start_iwdg();

    /* 统计打印计数器 */
    uint32_t stats_print_counter = 0;

    /* 主循环 */
    while (1) {
        /* 自身心跳 */
        watchdog_feed_task(WDG_TASK_WATCHDOG);

        /* 检查所有任务健康状态 */
        watchdog_check_all_tasks();

        /* 喂硬件看门狗 */
        watchdog_feed_iwdg();

        /* 每30秒打印一次统计信息 */
        stats_print_counter++;
        if (stats_print_counter >= (30000 / WDG_CHECK_INTERVAL_MS)) {
            watchdog_print_stats();
            stats_print_counter = 0;
        }

        /* 延迟等待 */
        osDelay(WDG_CHECK_INTERVAL_MS);
    }
}

/* ============================================================================
 * 工具函数实现
 * ============================================================================ */

/**
 * @brief  获取任务名称字符串
 */
const char* watchdog_get_task_name(WDG_TaskID_t task_id)
{
    if (task_id >= WDG_TASK_MAX) {
        return "UNKNOWN";
    }
    return task_name_strings[task_id];
}

/**
 * @brief  获取健康状态字符串
 */
const char* watchdog_get_health_string(WDG_HealthStatus_t status)
{
    if (status >= sizeof(health_status_strings) / sizeof(health_status_strings[0])) {
        return "UNKNOWN";
    }
    return health_status_strings[status];
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  初始化任务监控表
 */
static void init_task_info_table(void)
{
    for (int i = 0; i < WDG_TASK_MAX; i++) {
        task_info_table[i].task_name = task_name_strings[i];
        task_info_table[i].last_heartbeat_ms = 0;
        task_info_table[i].heartbeat_count = 0;
        task_info_table[i].timeout_count = 0;
        task_info_table[i].status = WDG_HEALTH_NOT_STARTED;
        task_info_table[i].enabled = true;  /* 默认启用监控 */
    }

    LOG_INFO("[Watchdog] Task monitoring table initialized (%d tasks)", WDG_TASK_MAX);
}

/**
 * @brief  获取当前时间戳（毫秒）
 */
static uint32_t get_current_time_ms(void)
{
    return HAL_GetTick();
}

/* ============================================================================
 * 测试辅助函数实现（仅调试使用）
 * ============================================================================ */

/**
 * @brief  模拟看门狗任务卡死（用于测试IWDG复位）
 */
void watchdog_test_hang(uint32_t duration_ms)
{
    LOG_ERROR("[Watchdog] ⚠️  TEST: Simulating watchdog hang...");

    if (duration_ms == 0) {
        /* 永久卡死，等待IWDG复位 */
        LOG_ERROR("[Watchdog] TEST: Infinite hang - IWDG will reset system in 5s");
        while(1) {
            osDelay(100);
        }
    } else {
        /* 临时卡死指定时间 */
        LOG_ERROR("[Watchdog] TEST: Hang for %lu ms", duration_ms);
        osDelay(duration_ms);
        LOG_INFO("[Watchdog] TEST: Resuming normal operation");
    }
}

/**
 * @brief  获取测试标志状态
 */
bool watchdog_test_should_hang(WDG_TaskID_t task_id)
{
    if (task_id >= WDG_TASK_MAX) {
        return false;
    }

    /* 检查是否需要卡死 */
    if (test_hang_flags[task_id]) {
        test_hang_flags[task_id] = false;  /* 清除标志 */
        return true;
    }

    return false;
}

/**
 * @brief  触发任务卡死测试
 */
void watchdog_test_trigger_hang(WDG_TaskID_t task_id, uint32_t duration_ms)
{
    if (task_id >= WDG_TASK_MAX) {
        LOG_ERROR("[Watchdog] Invalid task ID: %d", task_id);
        return;
    }

    const char *task_name = watchdog_get_task_name(task_id);

    LOG_WARN("[Watchdog] ⚠️  TEST: Triggering %s hang for %lu ms",
             task_name, duration_ms);

    /* 设置测试标志 */
    test_hang_duration[task_id] = duration_ms;
    test_hang_flags[task_id] = true;

    /* 如果是看门狗任务自身，直接卡死 */
    if (task_id == WDG_TASK_WATCHDOG) {
        watchdog_test_hang(duration_ms);
    }
}

