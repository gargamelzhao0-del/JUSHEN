/**
 ******************************************************************************
 * @file    robot_state.c
 * @brief   四足机器狗系统状态管理模块实现
 ******************************************************************************
 */

#include "robot_state.h"
#include "motor_control.h"
#include "log_system.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 系统状态 */
static RobotState_t current_state = ROBOT_STATE_INIT;
static RobotState_t previous_state = ROBOT_STATE_INIT;

/* 错误标志 */
static uint16_t error_flags = ERROR_NONE;

/* 统计信息 */
static SystemStats_t system_stats = {0};

/* 命令时间戳（用于超时检测） */
static uint32_t last_command_timestamp = 0;

/* 任务句柄 */
static osThreadId_t state_task_handle = NULL;

/* 互斥锁（保护状态变量） */
static osMutexId_t state_mutex = NULL;

/* ============================================================================
 * 私有函数原型
 * ============================================================================ */

static void robot_state_task(void *argument);
static bool is_transition_valid(RobotState_t from, RobotState_t to);
static void on_state_enter(RobotState_t state);
static void on_state_exit(RobotState_t state);
static void check_system_health(void);

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief  初始化机器狗状态管理模块
 */
int robot_state_init(void)
{
    /* 创建互斥锁 */
    const osMutexAttr_t mutex_attr = {
        .name = "StateMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    state_mutex = osMutexNew(&mutex_attr);
    if (state_mutex == NULL) {
        LOG_ERROR("[RobotState] Failed to create mutex");
        return -1;
    }

    /* 初始化统计信息 */
    memset(&system_stats, 0, sizeof(system_stats));

    /* 初始化状态 */
    current_state = ROBOT_STATE_INIT;
    error_flags = ERROR_NONE;
    last_command_timestamp = HAL_GetTick();

    LOG_INFO("[RobotState] State manager initialized");
    return 0;
}

/**
 * @brief  启动机器狗状态管理任务
 */
int robot_state_start_task(void)
{
    const osThreadAttr_t task_attr = {
        .name = "RobotStateTask",
        .stack_size = 1024 * 4,
        .priority = osPriorityNormal,
    };

    state_task_handle = osThreadNew(robot_state_task, NULL, &task_attr);

    if (state_task_handle == NULL) {
        LOG_ERROR("[RobotState] Failed to create state task");
        return -1;
    }

    LOG_INFO("[RobotState] State task started");
    return 0;
}

/**
 * @brief  获取当前系统状态
 */
RobotState_t robot_state_get(void)
{
    RobotState_t state;

    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    state = current_state;

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }

    return state;
}

/**
 * @brief  请求状态转换
 */
bool robot_state_request_transition(RobotState_t new_state)
{
    bool success = false;

    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    /* 检查状态转换是否有效 */
    if (is_transition_valid(current_state, new_state)) {
        LOG_INFO("[RobotState] State transition: %s -> %s",
                 robot_state_get_name(current_state),
                 robot_state_get_name(new_state));

        /* 执行退出回调 */
        on_state_exit(current_state);

        /* 更新状态 */
        previous_state = current_state;
        current_state = new_state;

        /* 执行进入回调 */
        on_state_enter(new_state);

        success = true;
    } else {
        LOG_WARN("[RobotState] Invalid transition: %s -> %s",
                 robot_state_get_name(current_state),
                 robot_state_get_name(new_state));
    }

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }

    return success;
}

/**
 * @brief  设置系统错误标志
 */
void robot_state_set_error(SystemError_t error)
{
    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    error_flags |= error;
    system_stats.error_count++;

    /* 如果不在错误状态，则转换到错误状态 */
    if (current_state != ROBOT_STATE_ERROR && current_state != ROBOT_STATE_EMERGENCY) {
        if (error & ERROR_CRITICAL) {
            /* 严重错误 -> 紧急停止 */
            robot_state_emergency_stop();
        } else {
            /* 一般错误 -> 错误状态 */
            previous_state = current_state;
            current_state = ROBOT_STATE_ERROR;
            on_state_enter(ROBOT_STATE_ERROR);
        }
    }

    LOG_ERROR("[RobotState] Error set: 0x%04X (total flags: 0x%04X)", error, error_flags);

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }
}

/**
 * @brief  清除系统错误标志
 */
void robot_state_clear_error(SystemError_t error)
{
    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    error_flags &= ~error;

    LOG_INFO("[RobotState] Error cleared: 0x%04X (remaining: 0x%04X)", error, error_flags);

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }
}

/**
 * @brief  获取当前系统错误标志
 */
uint16_t robot_state_get_errors(void)
{
    uint16_t errors;

    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    errors = error_flags;

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }

    return errors;
}

/**
 * @brief  检查系统是否有错误
 */
bool robot_state_has_error(void)
{
    return (error_flags != ERROR_NONE);
}

/**
 * @brief  获取系统运行统计信息
 */
void robot_state_get_stats(SystemStats_t *stats)
{
    if (stats == NULL) return;

    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    memcpy(stats, &system_stats, sizeof(SystemStats_t));

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }
}

/**
 * @brief  更新命令时间戳
 */
void robot_state_update_command_timestamp(void)
{
    last_command_timestamp = HAL_GetTick();
    system_stats.command_count++;
}

/**
 * @brief  检查命令是否超时
 */
bool robot_state_is_command_timeout(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed;

    /* 处理uint32_t溢出 */
    if (now >= last_command_timestamp) {
        elapsed = now - last_command_timestamp;
    } else {
        elapsed = (0xFFFFFFFF - last_command_timestamp) + now + 1;
    }

    return (elapsed > timeout_ms);
}

/**
 * @brief  紧急停止（默认硬停止）
 */
void robot_state_emergency_stop(void)
{
    robot_state_emergency_stop_ex(ESTOP_LEVEL_HARD);
}

/**
 * @brief  多级紧急停止
 */
void robot_state_emergency_stop_ex(EmergencyStopLevel_t level)
{
    /* 可以从任何上下文调用，包括中断 */
    current_state = ROBOT_STATE_EMERGENCY;
    error_flags |= ERROR_CRITICAL;

    LOG_ERROR("[RobotState] EMERGENCY STOP ACTIVATED! Level: %d", level);

    switch (level) {
        case ESTOP_LEVEL_SOFT:
            /* 软停止：平滑减速（保持使能）*/
            LOG_WARN("[RobotState] Soft stop: smooth deceleration...");
            motor_control_stop_all();  /* 停止但保持使能 */
            break;

        case ESTOP_LEVEL_HARD:
            /* 硬停止：立即停止并失能 */
            LOG_ERROR("[RobotState] Hard stop: immediate halt!");
            motor_control_emergency_stop_all();  /* 失能所有电机 */
            break;

        case ESTOP_LEVEL_POWER_CUT:
            /* 断电停止：切断电机电源（硬件信号）*/
            LOG_ERROR("[RobotState] Power cut: cutting motor power!");
            motor_control_power_cut();
            break;

        default:
            LOG_ERROR("[RobotState] Unknown emergency stop level: %d", level);
            motor_control_emergency_stop_all();
            break;
    }
}

/**
 * @brief  检查是否处于紧急停止状态
 */
bool robot_state_is_emergency_stopped(void)
{
    return (current_state == ROBOT_STATE_EMERGENCY);
}

/**
 * @brief  从紧急停止状态恢复
 */
int robot_state_resume_from_emergency(void)
{
    if (current_state != ROBOT_STATE_EMERGENCY) {
        LOG_WARN("[RobotState] Not in emergency state, cannot resume");
        return -1;
    }

    /* 检查是否有未清除的错误 */
    if (error_flags != ERROR_NONE) {
        LOG_ERROR("[RobotState] Cannot resume: errors present (0x%04X)", error_flags);
        return -1;
    }

    /* 恢复到IDLE状态 */
    if (state_mutex != NULL) {
        osMutexAcquire(state_mutex, osWaitForever);
    }

    previous_state = current_state;
    current_state = ROBOT_STATE_IDLE;
    on_state_enter(ROBOT_STATE_IDLE);

    if (state_mutex != NULL) {
        osMutexRelease(state_mutex);
    }

    LOG_INFO("[RobotState] Resumed from emergency stop -> IDLE");
    return 0;
}

/**
 * @brief  获取状态名称字符串
 */
const char* robot_state_get_name(RobotState_t state)
{
    switch (state) {
        case ROBOT_STATE_INIT:      return "INIT";
        case ROBOT_STATE_IDLE:      return "IDLE";
        case ROBOT_STATE_READY:     return "READY";
        case ROBOT_STATE_RUNNING:   return "RUNNING";
        case ROBOT_STATE_ERROR:     return "ERROR";
        case ROBOT_STATE_EMERGENCY: return "EMERGENCY";
        default:                    return "UNKNOWN";
    }
}

/**
 * @brief  打印系统状态信息
 */
void robot_state_print_status(void)
{
    SystemStats_t stats;
    robot_state_get_stats(&stats);

    LOG_INFO("========== Robot State Status ==========");
    LOG_INFO("  State: %s", robot_state_get_name(current_state));
    LOG_INFO("  Errors: 0x%04X", error_flags);
    LOG_INFO("  Uptime: %lu s", stats.uptime_ms / 1000);
    LOG_INFO("  Commands: %lu", stats.command_count);
    LOG_INFO("  Errors: %lu", stats.error_count);
    LOG_INFO("  Free Heap: %u KB", stats.free_heap_kb);
    LOG_INFO("========================================");
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  机器狗状态管理任务
 */
static void robot_state_task(void *argument)
{
    (void)argument;
    uint32_t loop_count = 0;

    LOG_INFO("[RobotState] State task running");

    /* 等待系统初始化完成 */
    osDelay(1000);

    /* 初始化完成，转换到IDLE状态 */
    robot_state_request_transition(ROBOT_STATE_IDLE);

    for (;;)
    {
        /* 更新系统统计 */
        system_stats.uptime_ms = HAL_GetTick();
        system_stats.free_heap_kb = xPortGetFreeHeapSize() / 1024;

        /* 执行系统健康检查 */
        check_system_health();

        /* 每10秒打印一次状态 */
        if (loop_count % 100 == 0) {
            robot_state_print_status();
        }

        loop_count++;
        osDelay(100);  /* 100ms */
    }
}

/**
 * @brief  检查状态转换是否有效
 */
static bool is_transition_valid(RobotState_t from, RobotState_t to)
{
    /* 允许的状态转换表 */
    switch (from) {
        case ROBOT_STATE_INIT:
            return (to == ROBOT_STATE_IDLE);

        case ROBOT_STATE_IDLE:
            return (to == ROBOT_STATE_READY || to == ROBOT_STATE_ERROR || to == ROBOT_STATE_EMERGENCY);

        case ROBOT_STATE_READY:
            return (to == ROBOT_STATE_RUNNING || to == ROBOT_STATE_IDLE ||
                    to == ROBOT_STATE_ERROR || to == ROBOT_STATE_EMERGENCY);

        case ROBOT_STATE_RUNNING:
            return (to == ROBOT_STATE_READY || to == ROBOT_STATE_IDLE ||
                    to == ROBOT_STATE_ERROR || to == ROBOT_STATE_EMERGENCY);

        case ROBOT_STATE_ERROR:
            /* 只有清除所有错误后才能返回IDLE */
            return (to == ROBOT_STATE_IDLE || to == ROBOT_STATE_EMERGENCY);

        case ROBOT_STATE_EMERGENCY:
            /* 紧急停止后只能重启系统 */
            return false;

        default:
            return false;
    }
}

/**
 * @brief  状态进入回调
 */
static void on_state_enter(RobotState_t state)
{
    switch (state) {
        case ROBOT_STATE_IDLE:
            /* 禁用所有电机 */
            LOG_INFO("[RobotState] Entering IDLE - disabling motors");
            motor_control_disable_all();
            break;

        case ROBOT_STATE_READY:
            /* 使能所有电机 */
            LOG_INFO("[RobotState] Entering READY - enabling motors");
            motor_control_enable_all();
            break;

        case ROBOT_STATE_RUNNING:
            LOG_INFO("[RobotState] Entering RUNNING");
            break;

        case ROBOT_STATE_ERROR:
            LOG_WARN("[RobotState] Entering ERROR state");
            /* 停止电机但不禁用（允许恢复） */
            motor_control_stop_all();
            break;

        case ROBOT_STATE_EMERGENCY:
            LOG_ERROR("[RobotState] Entering EMERGENCY state");
            motor_control_emergency_stop_all();
            break;

        default:
            break;
    }
}

/**
 * @brief  状态退出回调
 */
static void on_state_exit(RobotState_t state)
{
    /* 如果需要在退出状态时执行清理操作，在这里添加 */
    (void)state;
}

/**
 * @brief  系统健康检查
 */
static void check_system_health(void)
{
    /* 检查命令超时（5秒无命令则报警） */
    if (current_state == ROBOT_STATE_RUNNING) {
        if (robot_state_is_command_timeout(5000)) {
            LOG_WARN("[RobotState] Command timeout detected");
            robot_state_set_error(ERROR_COMMAND_TIMEOUT);
        }
    }

    /* 检查堆空间 */
    if (system_stats.free_heap_kb < 2) {
        LOG_ERROR("[RobotState] Critical: Low heap memory!");
        robot_state_set_error(ERROR_CRITICAL);
    }

    /* 可以添加更多健康检查... */
}
