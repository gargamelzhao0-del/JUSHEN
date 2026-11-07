/**
 ******************************************************************************
 * @file    motor_sync.c
 * @brief   电机同步控制模块实现
 ******************************************************************************
 */

#include "motor_sync.h"
#include "motor_control.h"
#include "log_system.h"
#include "quad_emergency_stop.h"
#include "watchdog.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 当前同步模式 */
static MotorSyncMode_t sync_mode = SYNC_MODE_BATCH;

/* 电机映射表：[索引0-11] -> {motor_id, can_port} */
/* 默认：电机0-5在CAN1(ID 0x01-0x06), 电机6-11在CAN2(ID 0x07-0x0C) */
static uint8_t motor_map[MOTOR_SYNC_MAX_MOTORS][2] = {
    {0x01, 1}, {0x02, 1}, {0x03, 1}, {0x04, 1}, {0x05, 1}, {0x06, 1},  /* CAN1: 前左腿、前右腿 */
    {0x07, 2}, {0x08, 2}, {0x09, 2}, {0x0A, 2}, {0x0B, 2}, {0x0C, 2}   /* CAN2: 后左腿、后右腿 */
};

/* 统计信息 */
static uint32_t total_sent_count = 0;
static uint32_t total_failed_count = 0;
static uint32_t total_duration_us = 0;
static uint32_t min_duration_us = 0xFFFFFFFF;
static uint32_t max_duration_us = 0;

/* 互斥锁 */
static osMutexId_t sync_mutex = NULL;

/* DWT初始化标志 */
static bool dwt_initialized = false;

/* 电机发送使能标志 */
static bool motor_send_enabled = true;

/* 命令重试次数 */
static uint8_t retry_count = 2;

/* 电机状态跟踪 */
static bool motor_status[12] = {false};

/* 并行发送缓冲区 */
static MotorSyncCommand_t can1_commands[6];
static MotorSyncCommand_t can2_commands[6];

/* 并行发送结果结构（线程安全）*/
typedef struct {
    uint8_t success_count;
    uint8_t failed_count;
    uint8_t failed_ids[6];
    bool can1_done;
    bool can2_done;
} ParallelSendResult_t;

static ParallelSendResult_t g_parallel_result;
static osMutexId_t parallel_result_mutex = NULL;
static osThreadId_t can1_thread_id = NULL;
static osThreadId_t can2_thread_id = NULL;
static bool parallel_initialized = false;
static osSemaphoreId_t can1_sem = NULL;
static osSemaphoreId_t can2_sem = NULL;
static volatile uint8_t g_can1_count = 0;
static volatile uint8_t g_can2_count = 0;

/* ============================================================================ 
 * 私有函数原型
 * ============================================================================ */

static uint32_t get_micros(void);
static int send_single_command(const MotorSyncCommand_t *cmd);
static void init_dwt_counter(void);
static int send_command_with_retry(const MotorSyncCommand_t *cmd);
static int compare_command_priority(const void *a, const void *b);
static void split_commands_by_can(const MotorSyncCommand_t *commands, uint8_t count, uint8_t *can1_count, uint8_t *can2_count);
static int motor_sync_send_parallel(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);
static int motor_sync_send_parallel_sequential(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);

/* 公共API函数声明 */
int motor_sync_send_batch_legacy(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);
int motor_sync_send_timestamp(const MotorSyncCommand_t *commands, uint8_t count, uint32_t timestamp_us, MotorSyncResult_t *result);
int motor_sync_send_priority(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result);
int motor_sync_emergency_stop_all(MotorSyncResult_t *result);
bool motor_sync_get_motor_status(uint8_t motor_id);
void motor_sync_set_retry_count(uint8_t retry_count);
uint8_t motor_sync_get_retry_count(void);

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief  初始化电机同步控制模块
 */
int motor_sync_init(MotorSyncMode_t mode)
{
    /* 创建互斥锁 */
    const osMutexAttr_t mutex_attr = {
        .name = "MotorSyncMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    sync_mutex = osMutexNew(&mutex_attr);
    if (sync_mutex == NULL) {
        LOG_ERROR("[MotorSync] Failed to create mutex");
        return -1;
    }

    sync_mode = mode;

    /* 初始化DWT计数器（用于微秒级时间测量）*/
    init_dwt_counter();

    LOG_INFO("[MotorSync] Initialized with mode: %d", mode);
    return 0;
}

/**
 * @brief  基于时间戳同步发送命令
 */
int motor_sync_send_timestamp(const MotorSyncCommand_t *commands, uint8_t count, uint32_t timestamp_us, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid parameters");
        return -1;
    }

    /* 检查发送使能状态 */
    if (!motor_send_enabled) {
        /* 发送被禁用，返回成功但不实际发送 */
        if (result != NULL) {
            memset(result, 0, sizeof(MotorSyncResult_t));
            result->total_motors = count;
            result->success_count = count;  /* 假装成功，避免上层报错 */
        }
        return 0;
    }

    /* 初始化结果 */
    MotorSyncResult_t local_result = {0};
    local_result.total_motors = count;

    /* 记录开始时间 */
    uint32_t start_us = get_micros();

    /* 计算时间差 */
    int32_t time_diff_us = timestamp_us - start_us;

    /* 如果时间戳在过去，直接发送 */
    if (time_diff_us <= 0) {
        LOG_WARN("[MotorSync] Timestamp in past, sending immediately");
        return motor_sync_send_parallel(commands, count, result);
    }

    /* 等待到指定时间戳（仅在需要未来时间戳时才等待） */
    uint32_t now_us = get_micros();
    if (timestamp_us > now_us + 1000) {
        while (get_micros() < timestamp_us) {
            osDelay(1);
        }
    } else if (timestamp_us > now_us) {
        while (get_micros() < timestamp_us) { }
    }

    /* 在指定时间戳发送命令 */
    int ret = motor_sync_send_parallel(commands, count, &local_result);

    /* 记录结束时间 */
    uint32_t end_us = get_micros();
    local_result.send_duration_us = (end_us >= start_us) ? (end_us - start_us) : 0;

    /* 更新统计 */
    total_sent_count++;
    if (local_result.failed_count > 0) {
        total_failed_count++;
    }
    total_duration_us += local_result.send_duration_us;

    /* 更新最小/最大duration */
    if (local_result.send_duration_us < min_duration_us) {
        min_duration_us = local_result.send_duration_us;
    }
    if (local_result.send_duration_us > max_duration_us) {
        max_duration_us = local_result.send_duration_us;
    }

    /* 输出结果 */
    if (result != NULL) {
        memcpy(result, &local_result, sizeof(MotorSyncResult_t));
    }

    return ret;
}

/**
 * @brief  发送带优先级的命令批处理
 */
int motor_sync_send_priority(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid parameters");
        return -1;
    }

    /* 检查发送使能状态 */
    if (!motor_send_enabled) {
        /* 发送被禁用，返回成功但不实际发送 */
        if (result != NULL) {
            memset(result, 0, sizeof(MotorSyncResult_t));
            result->total_motors = count;
            result->success_count = count;  /* 假装成功，避免上层报错 */
        }
        return 0;
    }

    /* 优先级模式复用并行发送，因为并行发送已经包含了优先级排序 */
    return motor_sync_send_parallel(commands, count, result);
}

/**
 * @brief  紧急停止所有电机（高优先级）
 */
int motor_sync_emergency_stop_all(MotorSyncResult_t *result)
{
    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建紧急停止命令 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_STOP;
        commands[i].priority = 255;  /* 最高优先级 */
    }

    LOG_ERROR("[MotorSync] Emergency stop all motors!");
    return motor_sync_send_parallel(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  获取电机发送状态
 */
bool motor_sync_get_motor_status(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid motor ID: %d", motor_id);
        return false;
    }

    return motor_status[motor_id - 1];
}

/**
 * @brief  设置命令重试次数
 */
void motor_sync_set_retry_count(uint8_t count)
{
    if (count > 5) {
        LOG_WARN("[MotorSync] Retry count too high, setting to 5");
        retry_count = 5;
    } else {
        retry_count = count;
    }

    LOG_INFO("[MotorSync] Retry count set to: %d", retry_count);
}

/**
 * @brief  获取命令重试次数
 */
uint8_t motor_sync_get_retry_count(void)
{
    return retry_count;
}

/**
 * @brief  设置同步模式
 */
void motor_sync_set_mode(MotorSyncMode_t mode)
{
    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    sync_mode = mode;
    LOG_INFO("[MotorSync] Mode changed to: %d", mode);

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }
}

/**
 * @brief  获取当前同步模式
 */
MotorSyncMode_t motor_sync_get_mode(void)
{
    return sync_mode;
}

/**
 * @brief  同步发送多个电机命令
 */
int motor_sync_send_batch(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid parameters");
        return -1;
    }

    /* 根据同步模式选择发送方法 */
    switch (sync_mode) {
        case SYNC_MODE_BATCH:
            /* 传统批量发送模式 */
            return motor_sync_send_batch_legacy(commands, count, result);
        
        case SYNC_MODE_PARALLEL:
            /* 并行发送模式（CAN1和CAN2同时发送） */
            return motor_sync_send_parallel(commands, count, result);
        
        case SYNC_MODE_PRIORITY:
            /* 优先级发送模式（按优先级排序后发送） */
            return motor_sync_send_parallel(commands, count, result);  /* 优先级模式复用并行发送 */
        
        default:
            LOG_ERROR("[MotorSync] Unknown sync mode: %d", sync_mode);
            return -1;
    }
}

/**
 * @brief  传统批量发送模式（向后兼容）
 */
int motor_sync_send_batch_legacy(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid parameters");
        return -1;
    }

    /* 检查发送使能状态 */
    if (!motor_send_enabled) {
        /* 发送被禁用，返回成功但不实际发送 */
        if (result != NULL) {
            memset(result, 0, sizeof(MotorSyncResult_t));
            result->total_motors = count;
            result->success_count = count;  /* 假装成功，避免上层报错 */
        }
        return 0;
    }

    /* 初始化结果 */
    MotorSyncResult_t local_result = {0};
    local_result.total_motors = count;

    /* 记录开始时间 */
    uint32_t start_us = get_micros();

    /* 整个批次加锁一次，减少锁开销 */
    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    /* 批量发送所有命令 */
    for (uint8_t i = 0; i < count; i++) {
        int ret = send_command_with_retry(&commands[i]);

        if (ret == 0) {
            local_result.success_count++;
        } else {
            local_result.failed_ids[local_result.failed_count++] = commands[i].motor_id;
        }
    }

    /* 发送完成后释放锁 */
    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }

    /* 记录结束时间 */
    uint32_t end_us = get_micros();
    local_result.send_duration_us = (end_us >= start_us) ? (end_us - start_us) : 0;

    /* 更新统计 */
    total_sent_count++;
    if (local_result.failed_count > 0) {
        total_failed_count++;
    }
    total_duration_us += local_result.send_duration_us;

    /* 更新最小/最大duration */
    if (local_result.send_duration_us < min_duration_us) {
        min_duration_us = local_result.send_duration_us;
    }
    if (local_result.send_duration_us > max_duration_us) {
        max_duration_us = local_result.send_duration_us;
    }

    /* 输出结果 */
    if (result != NULL) {
        memcpy(result, &local_result, sizeof(MotorSyncResult_t));
    }

    /* 打印警告（如果有失败） */
    if (local_result.failed_count > 0) {
        LOG_WARN("[MotorSync] Batch send: %d/%d failed (duration: %lu us)",
                 local_result.failed_count, count, local_result.send_duration_us);
        return -1;
    }

    LOG_DEBUG("[MotorSync] Batch send: %d motors OK (duration: %lu us)",
              count, local_result.send_duration_us);
    return 0;
}

/**
 * @brief  发送12个电机的位置命令
 */
int motor_sync_send_all_position(const float positions[12], float max_speed, MotorSyncResult_t *result)
{
    if (positions == NULL) {
        LOG_ERROR("[MotorSync] NULL positions array");
        return -1;
    }

    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建命令数组 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_POSITION;
        commands[i].params.pos.position = positions[i];
        commands[i].params.pos.max_speed = max_speed;
        commands[i].priority = 128;  /* 默认优先级 */
    }

    return motor_sync_send_batch(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  发送12个电机的速度命令
 */
int motor_sync_send_all_velocity(const float velocities[12], MotorSyncResult_t *result)
{
    if (velocities == NULL) {
        LOG_ERROR("[MotorSync] NULL velocities array");
        return -1;
    }

    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建命令数组 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_VELOCITY;
        commands[i].params.vel.speed = velocities[i];
        commands[i].priority = 128;
    }

    return motor_sync_send_batch(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  发送12个电机的力矩命令
 */
int motor_sync_send_all_torque(const float torques[12], MotorSyncResult_t *result)
{
    if (torques == NULL) {
        LOG_ERROR("[MotorSync] NULL torques array");
        return -1;
    }

    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建命令数组 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_TORQUE;
        commands[i].params.trq.torque = torques[i];
        commands[i].priority = 128;
    }

    return motor_sync_send_batch(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  同步启用所有电机
 */
int motor_sync_enable_all(MotorSyncResult_t *result)
{
    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建启用命令 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_ENABLE;
        commands[i].priority = 255;  /* 高优先级 */
    }

    LOG_INFO("[MotorSync] Enabling all motors...");
    return motor_sync_send_batch(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  同步禁用所有电机
 */
int motor_sync_disable_all(MotorSyncResult_t *result)
{
    MotorSyncCommand_t commands[MOTOR_SYNC_MAX_MOTORS];

    /* 构建禁用命令 */
    for (uint8_t i = 0; i < MOTOR_SYNC_MAX_MOTORS; i++) {
        commands[i].motor_id = motor_map[i][0];
        commands[i].can_port = motor_map[i][1];
        commands[i].type = MOTOR_CMD_DISABLE;
        commands[i].priority = 255;
    }

    LOG_INFO("[MotorSync] Disabling all motors...");
    return motor_sync_send_batch(commands, MOTOR_SYNC_MAX_MOTORS, result);
}

/**
 * @brief  获取同步发送统计信息
 */
void motor_sync_get_stats(uint32_t *total_sent, uint32_t *total_failed, uint32_t *avg_duration_us)
{
    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    if (total_sent != NULL) {
        *total_sent = total_sent_count;
    }

    if (total_failed != NULL) {
        *total_failed = total_failed_count;
    }

    if (avg_duration_us != NULL) {
        if (total_sent_count > 0) {
            *avg_duration_us = total_duration_us / total_sent_count;
        } else {
            *avg_duration_us = 0;
        }
    }

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }
}

/**
 * @brief  获取扩展统计信息（包含抖动监控）
 */
void motor_sync_get_stats_extended(
    uint32_t *total_sent,
    uint32_t *total_failed,
    uint32_t *avg_duration_us,
    uint32_t *min_duration_us_out,
    uint32_t *max_duration_us_out,
    uint32_t *jitter_us)
{
    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    if (total_sent != NULL) {
        *total_sent = total_sent_count;
    }

    if (total_failed != NULL) {
        *total_failed = total_failed_count;
    }

    if (avg_duration_us != NULL) {
        if (total_sent_count > 0) {
            *avg_duration_us = total_duration_us / total_sent_count;
        } else {
            *avg_duration_us = 0;
        }
    }

    if (min_duration_us_out != NULL) {
        *min_duration_us_out = (min_duration_us == 0xFFFFFFFF) ? 0 : min_duration_us;
    }

    if (max_duration_us_out != NULL) {
        *max_duration_us_out = max_duration_us;
    }

    if (jitter_us != NULL) {
        if (min_duration_us != 0xFFFFFFFF && max_duration_us > min_duration_us) {
            *jitter_us = max_duration_us - min_duration_us;
        } else {
            *jitter_us = 0;
        }
    }

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }
}

/**
 * @brief  重置统计信息
 */
void motor_sync_reset_stats(void)
{
    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    total_sent_count = 0;
    total_failed_count = 0;
    total_duration_us = 0;
    min_duration_us = 0xFFFFFFFF;
    max_duration_us = 0;

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }

    LOG_INFO("[MotorSync] Statistics reset");
}

/**
 * @brief  打印同步发送结果
 */
void motor_sync_print_result(const MotorSyncResult_t *result)
{
    if (result == NULL) return;

    LOG_INFO("[MotorSync] === Batch Result ===");
    LOG_INFO("  Total: %d, Success: %d, Failed: %d",
             result->total_motors, result->success_count, result->failed_count);
    LOG_INFO("  Duration: %lu us", result->send_duration_us);

    if (result->failed_count > 0) {
        LOG_RAW("  Failed IDs: ");
        for (uint8_t i = 0; i < result->failed_count; i++) {
            LOG_RAW("0x%02X ", result->failed_ids[i]);
        }
        LOG_RAW("\r\n");
    }
}

/**
 * @brief  设置电机发送使能状态
 */
void motor_sync_set_send_enabled(bool enabled)
{
    motor_send_enabled = enabled;

    if (enabled) {
        LOG_INFO("[MotorSync] Motor sending ENABLED");
    } else {
        LOG_WARN("[MotorSync] Motor sending DISABLED (commands will be ignored)");
    }
}

/**
 * @brief  获取电机发送使能状态
 */
bool motor_sync_get_send_enabled(void)
{
    return motor_send_enabled;
}

/**
 * @brief  设置电机ID映射表
 */
void motor_sync_set_motor_map(const uint8_t motor_map_new[12][2])
{
    if (motor_map_new == NULL) return;

    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    memcpy(motor_map, motor_map_new, sizeof(motor_map));
    LOG_INFO("[MotorSync] Motor map updated");

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }
}

/**
 * @brief  获取当前电机映射表
 */
void motor_sync_get_motor_map(uint8_t motor_map_out[12][2])
{
    if (motor_map_out == NULL) return;

    if (sync_mutex != NULL) {
        osMutexAcquire(sync_mutex, osWaitForever);
    }

    memcpy(motor_map_out, motor_map, sizeof(motor_map));

    if (sync_mutex != NULL) {
        osMutexRelease(sync_mutex);
    }
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  获取微秒级时间戳
 */
static uint32_t get_micros(void)
{
    /* 使用DWT计数器获取微秒级时间戳 */
    if (!dwt_initialized) {
        return HAL_GetTick() * 1000;  /* 降级到毫秒级 */
    }

    uint32_t cycles = DWT->CYCCNT;
    uint32_t micros = cycles / (SystemCoreClock / 1000000);
    return micros;
}

/**
 * @brief  初始化DWT计数器
 * @note   DWT (Data Watchpoint and Trace) 用于精确时间测量
 */
static void init_dwt_counter(void)
{
    /* 使能TRC (Trace) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 重置DWT计数器 */
    DWT->CYCCNT = 0;

    /* 使能DWT计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dwt_initialized = true;

    LOG_DEBUG("[MotorSync] DWT counter initialized for microsecond timing");
}

/**
 * @brief  发送单个电机命令
 */
static int send_single_command(const MotorSyncCommand_t *cmd)
{
    if (cmd == NULL) return -1;

    int ret = -1;

    /* 根据命令类型调用对应的电机控制函数 */
    switch (cmd->type) {
        case MOTOR_CMD_ENABLE:
            ret = motor_enable(cmd->can_port, cmd->motor_id, 1);  /* 1=使能 */
            break;

        case MOTOR_CMD_DISABLE:
            ret = motor_enable(cmd->can_port, cmd->motor_id, 0);  /* 0=失能 */
            break;

        case MOTOR_CMD_POSITION:
            ret = motor_set_position(
                cmd->can_port,
                cmd->motor_id,
                cmd->params.pos.position
            );
            /* 注意：max_speed需要单独设置，如果需要的话 */
            if (ret == 0 && cmd->params.pos.max_speed > 0.0f) {
                motor_set_limit_speed(cmd->can_port, cmd->motor_id, cmd->params.pos.max_speed);
            }
            break;

        case MOTOR_CMD_VELOCITY:
            ret = motor_set_speed(
                cmd->can_port,
                cmd->motor_id,
                cmd->params.vel.speed
            );
            break;

        case MOTOR_CMD_TORQUE:
            ret = motor_set_torque(
                cmd->can_port,
                cmd->motor_id,
                cmd->params.trq.torque
            );
            break;

        case MOTOR_CMD_STOP:
            ret = motor_start_stop(cmd->can_port, cmd->motor_id, 0);  /* 0=停止 */
            break;

        default:
            LOG_ERROR("[MotorSync] Unknown command type: %d", cmd->type);
            ret = -1;
            break;
    }

    return ret;
}

/**
 * @brief  带重试机制的命令发送
 */
/* ============== 修复: 电机同步超时保护 ============== */
#define MOTOR_SYNC_MAX_RETRY_TIME_MS  100  /* 最大重试时间 */

static int send_command_with_retry(const MotorSyncCommand_t *cmd)
{
    if (cmd == NULL) return -1;

    int ret = -1;
    uint32_t start_tick = HAL_GetTick();

    /* 尝试发送命令，带重试和总超时保护 */
    for (uint8_t attempt = 0; attempt <= retry_count; attempt++) {
        /* 检查总超时 */
        if (HAL_GetTick() - start_tick > MOTOR_SYNC_MAX_RETRY_TIME_MS) {
            LOG_WARN("[MotorSync] Retry timeout for motor 0x%02X", cmd->motor_id);
            break;
        }
        
        ret = send_single_command(cmd);
        if (ret == 0) {
            /* 发送成功 */
            motor_status[cmd->motor_id - 1] = true;
            /* 通知双CAN总线模块 */
            dual_can_on_send_success(cmd->can_port);
            break;
        }

        /* 发送失败，短暂延时后重试 */
        if (attempt < retry_count) {
            osDelay(1);  /* 1ms延时 */
        }
    }

    if (ret != 0) {
        /* 所有尝试都失败 */
        motor_status[cmd->motor_id - 1] = false;
        /* 通知双CAN总线模块 */
        dual_can_on_send_error(cmd->can_port);
        LOG_WARN("[MotorSync] Command failed after %d attempts: motor 0x%02X",
                 retry_count + 1, cmd->motor_id);
    }

    return ret;
}

/**
 * @brief  命令优先级比较函数
 * @note   用于qsort排序，优先级高的命令排在前面
 */
static int compare_command_priority(const void *a, const void *b)
{
    const MotorSyncCommand_t *cmd1 = (const MotorSyncCommand_t *)a;
    const MotorSyncCommand_t *cmd2 = (const MotorSyncCommand_t *)b;

    /* 优先级值越大，优先级越高 */
    if (cmd1->priority > cmd2->priority) {
        return -1;  /* cmd1排在前面 */
    } else if (cmd1->priority < cmd2->priority) {
        return 1;  /* cmd2排在前面 */
    } else {
        return 0;  /* 优先级相同 */
    }
}

/**
 * @brief  按CAN端口拆分命令
 */
static void split_commands_by_can(const MotorSyncCommand_t *commands, uint8_t count, uint8_t *can1_count, uint8_t *can2_count)
{
    if (commands == NULL || can1_count == NULL || can2_count == NULL) return;

    *can1_count = 0;
    *can2_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (commands[i].can_port == 1) {
            if (*can1_count < 6) {
                can1_commands[*can1_count] = commands[i];
                (*can1_count)++;
            }
        } else if (commands[i].can_port == 2) {
            if (*can2_count < 6) {
                can2_commands[*can2_count] = commands[i];
                (*can2_count)++;
            }
        }
    }
}

/**
 * @brief CAN1并行发送线程函数
 * @note  使用信号量触发，每次Resume都会重新执行发送循环
 */
static void can1_send_thread(void *argument)
{
    (void)argument;

    for (;;) {
        osSemaphoreAcquire(can1_sem, osWaitForever);

        for (uint8_t i = 0; i < g_can1_count; i++) {
            int ret = send_command_with_retry(&can1_commands[i]);
            if (ret == 0) {
                osMutexWait(parallel_result_mutex, osWaitForever);
                g_parallel_result.success_count++;
                osMutexRelease(parallel_result_mutex);
            } else {
                osMutexWait(parallel_result_mutex, osWaitForever);
                if (g_parallel_result.failed_count < 6) {
                    g_parallel_result.failed_ids[g_parallel_result.failed_count++] = can1_commands[i].motor_id;
                }
                osMutexRelease(parallel_result_mutex);
            }
        }

        osMutexWait(parallel_result_mutex, osWaitForever);
        g_parallel_result.can1_done = true;
        osMutexRelease(parallel_result_mutex);
    }
}

/**
 * @brief CAN2并行发送线程函数
 * @note  使用信号量触发，每次Resume都会重新执行发送循环
 */
static void can2_send_thread(void *argument)
{
    (void)argument;

    for (;;) {
        osSemaphoreAcquire(can2_sem, osWaitForever);

        for (uint8_t i = 0; i < g_can2_count; i++) {
            int ret = send_command_with_retry(&can2_commands[i]);
            if (ret == 0) {
                osMutexWait(parallel_result_mutex, osWaitForever);
                g_parallel_result.success_count++;
                osMutexRelease(parallel_result_mutex);
            } else {
                osMutexWait(parallel_result_mutex, osWaitForever);
                if (g_parallel_result.failed_count < 6) {
                    g_parallel_result.failed_ids[g_parallel_result.failed_count++] = can2_commands[i].motor_id;
                }
                osMutexRelease(parallel_result_mutex);
            }
        }

        osMutexWait(parallel_result_mutex, osWaitForever);
        g_parallel_result.can2_done = true;
        osMutexRelease(parallel_result_mutex);
    }
}

/**
 * @brief 初始化并行发送线程
 */
static int init_parallel_threads(void)
{
    if (parallel_initialized) {
        return 0;
    }

    parallel_result_mutex = osMutexNew(NULL);
    if (parallel_result_mutex == NULL) {
        LOG_ERROR("[MotorSync] Failed to create parallel result mutex");
        return -1;
    }

    can1_sem = osSemaphoreNew(1, 0, NULL);
    if (can1_sem == NULL) {
        LOG_ERROR("[MotorSync] Failed to create CAN1 semaphore");
        return -1;
    }

    can2_sem = osSemaphoreNew(1, 0, NULL);
    if (can2_sem == NULL) {
        LOG_ERROR("[MotorSync] Failed to create CAN2 semaphore");
        return -1;
    }

    const osThreadAttr_t can1_thread_attr = {
        .name = "CAN1_Send",
        .stack_size = 512,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    can1_thread_id = osThreadNew(can1_send_thread, NULL, &can1_thread_attr);
    if (can1_thread_id == NULL) {
        LOG_ERROR("[MotorSync] Failed to create CAN1 thread");
        return -1;
    }

    const osThreadAttr_t can2_thread_attr = {
        .name = "CAN2_Send",
        .stack_size = 512,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    can2_thread_id = osThreadNew(can2_send_thread, NULL, &can2_thread_attr);
    if (can2_thread_id == NULL) {
        LOG_ERROR("[MotorSync] Failed to create CAN2 thread");
        return -1;
    }

    parallel_initialized = true;
    LOG_INFO("[MotorSync] Parallel threads initialized with semaphores");
    return 0;
}

/**
 * @brief  并行发送电机命令（真正的双线程并行）
 * @note   使用CMSIS-RTOS双线程+信号量实现CAN1和CAN2真正并行发送
 */
static int motor_sync_send_parallel(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        LOG_ERROR("[MotorSync] Invalid parameters");
        return -1;
    }

    if (!motor_send_enabled) {
        if (result != NULL) {
            memset(result, 0, sizeof(MotorSyncResult_t));
            result->total_motors = count;
            result->success_count = count;
        }
        return 0;
    }

    if (!parallel_initialized) {
        if (init_parallel_threads() != 0) {
            LOG_WARN("[MotorSync] Parallel init failed, falling back to sequential");
            return motor_sync_send_parallel_sequential(commands, count, result);
        }
    }

    MotorSyncResult_t local_result = {0};
    local_result.total_motors = count;
    uint32_t start_us = get_micros();

    MotorSyncCommand_t sorted_commands[MOTOR_SYNC_MAX_MOTORS];
    memcpy(sorted_commands, commands, count * sizeof(MotorSyncCommand_t));
    qsort(sorted_commands, count, sizeof(MotorSyncCommand_t), compare_command_priority);

    uint8_t can1_count = 0;
    uint8_t can2_count = 0;
    split_commands_by_can(sorted_commands, count, &can1_count, &can2_count);

    memcpy(can1_commands, sorted_commands, can1_count * sizeof(MotorSyncCommand_t));
    for (uint8_t i = 0; i < can1_count; i++) {
        can1_commands[i].can_port = 1;
    }

    const MotorSyncCommand_t* can2_start = sorted_commands + can1_count;
    memcpy(can2_commands, can2_start, can2_count * sizeof(MotorSyncCommand_t));
    for (uint8_t i = 0; i < can2_count; i++) {
        can2_commands[i].can_port = 2;
    }

    g_can1_count = can1_count;
    g_can2_count = can2_count;

    osMutexWait(parallel_result_mutex, osWaitForever);
    memset(&g_parallel_result, 0, sizeof(ParallelSendResult_t));
    g_parallel_result.can1_done = (can1_count == 0);
    g_parallel_result.can2_done = (can2_count == 0);
    osMutexRelease(parallel_result_mutex);

    if (can1_count > 0) {
        osSemaphoreRelease(can1_sem);
    }
    if (can2_count > 0) {
        osSemaphoreRelease(can2_sem);
    }

    uint32_t timeout = 50;
    while (timeout > 0) {
        osMutexWait(parallel_result_mutex, osWaitForever);
        bool all_done = g_parallel_result.can1_done && g_parallel_result.can2_done;
        osMutexRelease(parallel_result_mutex);

        if (all_done) {
            break;
        }
        watchdog_feed_iwdg();
        osDelay(1);
        timeout--;
    }

    osMutexWait(parallel_result_mutex, osWaitForever);
    local_result.success_count = g_parallel_result.success_count;
    local_result.failed_count = g_parallel_result.failed_count;
    memcpy(local_result.failed_ids, g_parallel_result.failed_ids, g_parallel_result.failed_count);
    osMutexRelease(parallel_result_mutex);

    uint32_t end_us = get_micros();
    local_result.send_duration_us = (end_us >= start_us) ? (end_us - start_us) : 0;

    total_sent_count++;
    if (local_result.failed_count > 0) {
        total_failed_count++;
    }
    total_duration_us += local_result.send_duration_us;

    if (local_result.send_duration_us < min_duration_us) {
        min_duration_us = local_result.send_duration_us;
    }
    if (local_result.send_duration_us > max_duration_us) {
        max_duration_us = local_result.send_duration_us;
    }

    if (result != NULL) {
        memcpy(result, &local_result, sizeof(MotorSyncResult_t));
    }

    if (local_result.failed_count > 0) {
        LOG_WARN("[MotorSync] Parallel send: %d/%d failed (duration: %lu us)",
                 local_result.failed_count, count, local_result.send_duration_us);
        return -1;
    }

    LOG_DEBUG("[MotorSync] Parallel send: %d motors OK (duration: %lu us)",
              count, local_result.send_duration_us);
    return 0;
}

/**
 * @brief 串行回退发送（当并行发送失败时使用）
 */
static int motor_sync_send_parallel_sequential(const MotorSyncCommand_t *commands, uint8_t count, MotorSyncResult_t *result)
{
    if (commands == NULL || count == 0 || count > MOTOR_SYNC_MAX_MOTORS) {
        return -1;
    }

    if (!motor_send_enabled) {
        if (result != NULL) {
            memset(result, 0, sizeof(MotorSyncResult_t));
            result->total_motors = count;
            result->success_count = count;
        }
        return 0;
    }

    MotorSyncResult_t local_result = {0};
    local_result.total_motors = count;
    uint32_t start_us = get_micros();

    MotorSyncCommand_t sorted_commands[MOTOR_SYNC_MAX_MOTORS];
    memcpy(sorted_commands, commands, count * sizeof(MotorSyncCommand_t));
    qsort(sorted_commands, count, sizeof(MotorSyncCommand_t), compare_command_priority);

    uint8_t can1_count = 0;
    uint8_t can2_count = 0;
    split_commands_by_can(sorted_commands, count, &can1_count, &can2_count);

    for (uint8_t i = 0; i < can1_count; i++) {
        int ret = send_command_with_retry(&can1_commands[i]);
        if (ret == 0) {
            local_result.success_count++;
        } else {
            local_result.failed_ids[local_result.failed_count++] = can1_commands[i].motor_id;
        }
    }

    for (uint8_t i = 0; i < can2_count; i++) {
        int ret = send_command_with_retry(&can2_commands[i]);
        if (ret == 0) {
            local_result.success_count++;
        } else {
            local_result.failed_ids[local_result.failed_count++] = can2_commands[i].motor_id;
        }
    }

    uint32_t end_us = get_micros();
    local_result.send_duration_us = (end_us >= start_us) ? (end_us - start_us) : 0;

    total_sent_count++;
    if (local_result.failed_count > 0) {
        total_failed_count++;
    }
    total_duration_us += local_result.send_duration_us;

    if (local_result.send_duration_us < min_duration_us) {
        min_duration_us = local_result.send_duration_us;
    }
    if (local_result.send_duration_us > max_duration_us) {
        max_duration_us = local_result.send_duration_us;
    }

    if (result != NULL) {
        memcpy(result, &local_result, sizeof(MotorSyncResult_t));
    }

    if (local_result.failed_count > 0) {
        return -1;
    }

    return 0;
}

/* ============================================================================
 * 双CAN总线联动控制实现
 * ============================================================================ */

#define DUAL_CAN_DEFAULT_TIMEOUT_MS     100
#define DUAL_CAN_DEFAULT_ERROR_THRESHOLD 5

static DualCANStatus_t g_dual_can_status = {
    .can1_healthy = true,
    .can2_healthy = true,
    .can1_last_success_tick = 0,
    .can2_last_success_tick = 0,
    .can1_error_count = 0,
    .can2_error_count = 0,
    .can1_total_sent = 0,
    .can2_total_sent = 0,
    .can1_total_failed = 0,
    .can2_total_failed = 0
};

static uint32_t g_can_timeout_ms = DUAL_CAN_DEFAULT_TIMEOUT_MS;
static uint32_t g_can_error_threshold = DUAL_CAN_DEFAULT_ERROR_THRESHOLD;
static osMutexId_t dual_can_mutex = NULL;
static bool dual_can_initialized = false;

int dual_can_init(void)
{
    if (dual_can_initialized) {
        return 0;
    }
    
    dual_can_mutex = osMutexNew(NULL);
    if (dual_can_mutex == NULL) {
        LOG_ERROR("[DualCAN] Failed to create mutex");
        return -1;
    }
    
    g_dual_can_status.can1_healthy = true;
    g_dual_can_status.can2_healthy = true;
    g_dual_can_status.can1_last_success_tick = HAL_GetTick();
    g_dual_can_status.can2_last_success_tick = HAL_GetTick();
    g_dual_can_status.can1_error_count = 0;
    g_dual_can_status.can2_error_count = 0;
    
    dual_can_initialized = true;
    LOG_INFO("[DualCAN] Initialized (timeout=%lums, threshold=%lu)",
             g_can_timeout_ms, g_can_error_threshold);
    return 0;
}

bool dual_can_check_health(void)
{
    if (!dual_can_initialized) {
        dual_can_init();
    }
    
    uint32_t current_tick = HAL_GetTick();
    bool need_emergency_stop = false;
    
    if (dual_can_mutex != NULL) {
        osMutexAcquire(dual_can_mutex, osWaitForever);
    }
    
    if (g_dual_can_status.can1_healthy) {
        uint32_t elapsed1 = current_tick - g_dual_can_status.can1_last_success_tick;
        if (elapsed1 > g_can_timeout_ms && g_dual_can_status.can1_error_count >= g_can_error_threshold) {
            g_dual_can_status.can1_healthy = false;
            LOG_ERROR("[DualCAN] CAN1 fault detected! (elapsed=%lums, errors=%lu)",
                      elapsed1, g_dual_can_status.can1_error_count);
            need_emergency_stop = true;
        }
    }
    
    if (g_dual_can_status.can2_healthy) {
        uint32_t elapsed2 = current_tick - g_dual_can_status.can2_last_success_tick;
        if (elapsed2 > g_can_timeout_ms && g_dual_can_status.can2_error_count >= g_can_error_threshold) {
            g_dual_can_status.can2_healthy = false;
            LOG_ERROR("[DualCAN] CAN2 fault detected! (elapsed=%lums, errors=%lu)",
                      elapsed2, g_dual_can_status.can2_error_count);
            need_emergency_stop = true;
        }
    }
    
    if (dual_can_mutex != NULL) {
        osMutexRelease(dual_can_mutex);
    }
    
    if (need_emergency_stop) {
        LOG_ERROR("[DualCAN] Bus failure! Triggering emergency stop!");
        /* 使用紧急停止模块函数 */
        extern void QuadEmergencyStop_Trigger(EmergencyStopSource_t source, uint32_t level);
        QuadEmergencyStop_Trigger(ESTOP_SOURCE_SOFTWARE, ESTOP_LEVEL_HARD);
        motor_sync_emergency_stop_all(NULL);
    }
    
    return g_dual_can_status.can1_healthy && g_dual_can_status.can2_healthy;
}

void dual_can_get_status(DualCANStatus_t *status)
{
    if (status == NULL) return;
    
    if (dual_can_mutex != NULL) {
        osMutexAcquire(dual_can_mutex, osWaitForever);
    }
    
    memcpy(status, &g_dual_can_status, sizeof(DualCANStatus_t));
    
    if (dual_can_mutex != NULL) {
        osMutexRelease(dual_can_mutex);
    }
}

void dual_can_on_send_success(uint8_t can_channel)
{
    if (!dual_can_initialized) return;
    
    if (dual_can_mutex != NULL) {
        osMutexAcquire(dual_can_mutex, osWaitForever);
    }
    
    if (can_channel == 1) {
        g_dual_can_status.can1_last_success_tick = HAL_GetTick();
        g_dual_can_status.can1_error_count = 0;
        g_dual_can_status.can1_healthy = true;
        g_dual_can_status.can1_total_sent++;
    } else if (can_channel == 2) {
        g_dual_can_status.can2_last_success_tick = HAL_GetTick();
        g_dual_can_status.can2_error_count = 0;
        g_dual_can_status.can2_healthy = true;
        g_dual_can_status.can2_total_sent++;
    }
    
    if (dual_can_mutex != NULL) {
        osMutexRelease(dual_can_mutex);
    }
}

void dual_can_on_send_error(uint8_t can_channel)
{
    if (!dual_can_initialized) return;
    
    if (dual_can_mutex != NULL) {
        osMutexAcquire(dual_can_mutex, osWaitForever);
    }
    
    if (can_channel == 1) {
        g_dual_can_status.can1_error_count++;
        g_dual_can_status.can1_total_failed++;
        LOG_WARN("[DualCAN] CAN1 send error (count=%lu)", g_dual_can_status.can1_error_count);
    } else if (can_channel == 2) {
        g_dual_can_status.can2_error_count++;
        g_dual_can_status.can2_total_failed++;
        LOG_WARN("[DualCAN] CAN2 send error (count=%lu)", g_dual_can_status.can2_error_count);
    }
    
    if (dual_can_mutex != NULL) {
        osMutexRelease(dual_can_mutex);
    }
    
    dual_can_check_health();
}

void dual_can_set_threshold(uint32_t timeout_ms, uint32_t error_count)
{
    g_can_timeout_ms = timeout_ms;
    g_can_error_threshold = error_count;
    LOG_INFO("[DualCAN] Threshold updated: timeout=%lums, errors=%lu",
             g_can_timeout_ms, g_can_error_threshold);
}

void dual_can_reset(uint8_t can_channel)
{
    if (dual_can_mutex != NULL) {
        osMutexAcquire(dual_can_mutex, osWaitForever);
    }
    
    if (can_channel == 0 || can_channel == 1) {
        g_dual_can_status.can1_healthy = true;
        g_dual_can_status.can1_error_count = 0;
        g_dual_can_status.can1_last_success_tick = HAL_GetTick();
        LOG_INFO("[DualCAN] CAN1 reset");
    }
    
    if (can_channel == 0 || can_channel == 2) {
        g_dual_can_status.can2_healthy = true;
        g_dual_can_status.can2_error_count = 0;
        g_dual_can_status.can2_last_success_tick = HAL_GetTick();
        LOG_INFO("[DualCAN] CAN2 reset");
    }
    
    if (dual_can_mutex != NULL) {
        osMutexRelease(dual_can_mutex);
    }
}

bool dual_can_can_send(void)
{
    if (!dual_can_initialized) {
        dual_can_init();
    }
    
    return g_dual_can_status.can1_healthy && g_dual_can_status.can2_healthy;
}
