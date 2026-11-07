/**
 ****************************************************************************************************
 * @file        motor_control_enhanced.c
 * @version     V1.0
 * @brief       电机控制增强模块实现
 ****************************************************************************************************
 */

#include "motor_control_enhanced.h"
#include "motor_control.h"
#include "cmsis_os.h"
#include "log_system.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * 私有全局变量
 * ============================================================================ */

/* 电机句柄数组（12电机）*/
static MotorHandle g_motor_handles[MOTOR_MAX_HANDLES];

/* 系统初始化标志 */
static bool g_handles_initialized = false;

/* 互斥锁 */
static osMutexId_t g_handles_mutex = NULL;

/* ============================================================================
 * 电机句柄管理实现
 * ============================================================================ */

/**
 * @brief 初始化电机句柄系统
 */
uint8_t motor_handles_init(void)
{
    if (g_handles_initialized) {
        return 0;  /* 已初始化 */
    }

    /* 创建互斥锁 */
    const osMutexAttr_t mutex_attr = {
        .name = "MotorHandlesMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    g_handles_mutex = osMutexNew(&mutex_attr);
    if (g_handles_mutex == NULL) {
        return 1;  /* 创建互斥锁失败 */
    }

    /* 清空句柄数组 */
    memset(g_motor_handles, 0, sizeof(g_motor_handles));

    /* 初始化每个句柄的基本信息 */
    for (uint8_t i = 0; i < MOTOR_MAX_HANDLES; i++) {
        g_motor_handles[i].motor_index = i;
        g_motor_handles[i].initialized = false;
    }

    g_handles_initialized = true;
    return 0;  /* 成功 */
}

/**
 * @brief 创建电机句柄
 */
uint8_t motor_handle_create(uint8_t motor_index, uint8_t can_channel, uint8_t device_id)
{
    if (motor_index >= MOTOR_MAX_HANDLES) {
        return 1;  /* 索引越界 */
    }

    if (!g_handles_initialized) {
        motor_handles_init();
    }

    osMutexAcquire(g_handles_mutex, osWaitForever);

    MotorHandle* handle = &g_motor_handles[motor_index];

    /* 设置基本信息 */
    handle->motor_index = motor_index;
    handle->can_channel = can_channel;
    handle->device_id = device_id;
    handle->initialized = true;

    /* 设置默认限制参数 */
    handle->torque_limit = 3.0f;    /* 默认3Nm */
    handle->speed_limit = 120.0f;   /* 默认120rpm */
    handle->accel_limit = 800.0f;   /* 默认800rpm/s */
    handle->pos_kp = 1.5f;          /* 默认Kp=1.5 */

    /* 清空统计信息 */
    handle->feedback_count = 0;
    handle->feedback_timeout_count = 0;
    handle->last_feedback_time = 0;
    handle->last_error_time = 0;

    osMutexRelease(g_handles_mutex);

    return 0;  /* 成功 */
}

/**
 * @brief 获取电机句柄
 */
MotorHandle* motor_handle_get(uint8_t motor_index)
{
    if (motor_index >= MOTOR_MAX_HANDLES) {
        return NULL;
    }

    if (!g_handles_initialized) {
        return NULL;
    }

    MotorHandle* handle = &g_motor_handles[motor_index];
    if (!handle->initialized) {
        return NULL;
    }

    return handle;
}

/**
 * @brief 根据CAN通道和设备ID获取电机句柄
 */
MotorHandle* get_motor_handle(uint8_t can_channel, uint8_t device_id)
{
    if (!g_handles_initialized) {
        return NULL;
    }

    /* 遍历查找匹配的句柄 */
    for (uint8_t i = 0; i < MOTOR_MAX_HANDLES; i++) {
        MotorHandle* handle = &g_motor_handles[i];
        if (handle->initialized &&
            handle->can_channel == can_channel &&
            handle->device_id == device_id) {
            return handle;
        }
    }

    return NULL;
}

/**
 * @brief 更新电机句柄的监控数据
 */
uint8_t motor_handle_update_monitor(uint8_t motor_index, const MotorMonitorData_t* monitor)
{
    if (motor_index >= MOTOR_MAX_HANDLES || monitor == NULL) {
        return 1;
    }

    MotorHandle* handle = motor_handle_get(motor_index);
    if (handle == NULL) {
        return 2;
    }

    osMutexAcquire(g_handles_mutex, osWaitForever);

    /* 复制监控数据 */
    memcpy(&handle->monitor, monitor, sizeof(MotorMonitorData_t));

    /* 更新快速访问字段 */
    handle->current_position = monitor->actual_position;
    handle->current_velocity = monitor->actual_speed;
    handle->current_torque = monitor->actual_torque;
    handle->temperature = monitor->motor_temperature;
    handle->board_temperature = monitor->board_temperature;  /* V9.0 */
    handle->bus_voltage = monitor->bus_voltage;

    /* V9.0: 电机保护检测 */
    if (monitor->motor_temperature >= MOTOR_TEMP_CRITICAL) {
        handle->error_flags |= MOTOR_ERROR_OVER_TEMP;
        LOG_RAW("[MOTOR_PROTECT] Motor %d: OVER TEMPERATURE! %.1f°C\r\n",
               motor_index, monitor->motor_temperature);
    }
    if (monitor->bus_voltage < MOTOR_VOLTAGE_CRITICAL) {
        handle->error_flags |= MOTOR_ERROR_UNDER_VOLTAGE;
        LOG_RAW("[MOTOR_PROTECT] Motor %d: UNDER VOLTAGE! %.1fV\r\n",
               motor_index, monitor->bus_voltage);
    } else if (monitor->bus_voltage > MOTOR_VOLTAGE_OVER) {
        handle->error_flags |= MOTOR_ERROR_OVER_VOLTAGE;
        LOG_RAW("[MOTOR_PROTECT] Motor %d: OVER VOLTAGE! %.1fV\r\n",
               motor_index, monitor->bus_voltage);
    }
    if (fabsf(handle->current_torque) > MOTOR_TORQUE_CRITICAL) {
        handle->error_flags |= MOTOR_ERROR_TORQUE_OVER;
        LOG_RAW("[MOTOR_PROTECT] Motor %d: TORQUE OVER! %.2fNm\r\n",
               motor_index, handle->current_torque);
    }

    /* 从力矩估算电流（简化模型）*/
    handle->current = handle->current_torque / 0.5f;  /* 假设力矩常数0.5 Nm/A */

    /* 更新反馈统计 */
    handle->last_feedback_time = HAL_GetTick();
    handle->feedback_count++;

    osMutexRelease(g_handles_mutex);

    return 0;
}

/**
 * @brief 更新电机句柄的错误状态
 */
uint8_t motor_handle_update_error(uint8_t motor_index, MotorError_t error_code)
{
    if (motor_index >= MOTOR_MAX_HANDLES) {
        return 1;
    }

    MotorHandle* handle = motor_handle_get(motor_index);
    if (handle == NULL) {
        return 2;
    }

    osMutexAcquire(g_handles_mutex, osWaitForever);

    /* 更新错误码 */
    handle->status.error_code = error_code;

    /* 设置错误标志 */
    if (error_code != MOTOR_ERROR_NONE) {
        handle->error_flags |= (1 << error_code);
        handle->last_error_time = HAL_GetTick();
    } else {
        handle->error_flags = 0;
    }

    osMutexRelease(g_handles_mutex);

    return 0;
}

/**
 * @brief 更新电机句柄的运行状态（启动/停止反馈）
 */
uint8_t motor_handle_update_running(uint8_t motor_index, uint8_t running, uint8_t enabled)
{
    if (motor_index >= MOTOR_MAX_HANDLES) {
        return 1;
    }

    MotorHandle* handle = motor_handle_get(motor_index);
    if (handle == NULL) {
        return 2;
    }

    osMutexAcquire(g_handles_mutex, osWaitForever);

    /* 更新运行状态 */
    handle->status.running = running ? 1 : 0;
    handle->status.enabled = enabled ? 1 : 0;

    osMutexRelease(g_handles_mutex);

    return 0;
}

/**
 * @brief 检查电机反馈是否超时
 */
bool motor_handle_is_feedback_timeout(uint8_t motor_index)
{
    MotorHandle* handle = motor_handle_get(motor_index);
    if (handle == NULL) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - handle->last_feedback_time;

    if (MOTOR_FEEDBACK_TIMEOUT_MS > 0 && elapsed > MOTOR_FEEDBACK_TIMEOUT_MS) {
        handle->feedback_timeout_count++;
        return true;
    }

    return false;
}

/* ============================================================================
 * 批量控制实现
 * ============================================================================ */

/**
 * @brief 批量设置位置目标
 */
uint8_t motor_batch_set_positions(uint8_t* motor_ids, float* positions, uint8_t count)
{
    if (motor_ids == NULL || positions == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 发送位置命令 */
        if (motor_set_position(handle->can_channel, handle->device_id, positions[i]) == 0) {
            handle->target_position = positions[i];
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 批量设置相同力矩限制
 */
uint8_t motor_batch_set_torque_limit(uint8_t* motor_ids, uint8_t count, float torque_limit)
{
    if (motor_ids == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 检查参数缓存，避免重复发送 */
        if (handle->torque_limit == torque_limit) {
            success_count++;
            continue;  /* 参数未变化，跳过 */
        }

        /* 发送力矩限制命令 */
        if (motor_set_torque_limit(handle->can_channel, handle->device_id, torque_limit) == 0) {
            handle->torque_limit = torque_limit;
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 批量设置相同速度限制
 */
uint8_t motor_batch_set_speed_limit(uint8_t* motor_ids, uint8_t count, float speed_limit)
{
    if (motor_ids == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 检查参数缓存 */
        if (handle->speed_limit == speed_limit) {
            success_count++;
            continue;
        }

        /* 发送速度限制命令 */
        if (motor_set_speed_limit(handle->can_channel, handle->device_id, speed_limit) == 0) {
            handle->speed_limit = speed_limit;
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 批量设置相同加速度限制
 */
uint8_t motor_batch_set_accel_limit(uint8_t* motor_ids, uint8_t count, float accel_limit)
{
    if (motor_ids == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 检查参数缓存 */
        if (handle->accel_limit == accel_limit) {
            success_count++;
            continue;
        }

        /* 发送加速度限制命令 */
        if (motor_set_limit_accel(handle->can_channel, handle->device_id, accel_limit) == 0) {
            handle->accel_limit = accel_limit;
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 批量设置相同位置环Kp
 */
uint8_t motor_batch_set_pos_kp(uint8_t* motor_ids, uint8_t count, float pos_kp)
{
    if (motor_ids == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 检查参数缓存 */
        if (handle->pos_kp == pos_kp) {
            success_count++;
            continue;
        }

        /* 发送Kp命令（已禁用，使用电机默认PID）*/
        // if (motor_set_pos_kp(handle->can_channel, handle->device_id, pos_kp) == 0) {
        //     handle->pos_kp = pos_kp;
        //     success_count++;
        // }
    }

    return success_count;
}

/**
 * @brief 批量使能电机
 */
uint8_t motor_batch_enable(uint8_t* motor_ids, uint8_t count, bool enable)
{
    if (motor_ids == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        MotorHandle* handle = motor_handle_get(motor_ids[i]);
        if (handle == NULL) {
            continue;
        }

        /* 发送使能命令 */
        if (motor_enable(handle->can_channel, handle->device_id, enable ? 1 : 0) == 0) {
            handle->status.enabled = enable ? 1 : 0;
            success_count++;
        }
    }

    return success_count;
}

/* ============================================================================
 * 反馈优化实现
 * ============================================================================ */

/**
 * @brief 请求所有电机的监控数据
 */
uint8_t motor_request_all_monitor_data(void)
{
    uint8_t success_count = 0;

    for (uint8_t i = 0; i < MOTOR_MAX_HANDLES; i++) {
        MotorHandle* handle = motor_handle_get(i);
        if (handle == NULL) {
            continue;
        }

        /* 发送监控数据请求 */
        if (motor_read_monitor_data(handle->can_channel, handle->device_id, NULL) == 0) {
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 处理电机监控数据反馈
 */
void motor_process_monitor_feedback(uint8_t can_channel, uint8_t device_id,
                                     const uint8_t* rxdata, uint8_t data_len)
{
    /* 查找对应的电机句柄 */
    MotorHandle* handle = get_motor_handle(can_channel, device_id);
    if (handle == NULL) {
        return;
    }

    /* 解析监控数据 */
    MotorMonitorData_t monitor;
    motor_parse_monitor_data(rxdata, &monitor);

    /* 更新句柄 */
    motor_handle_update_monitor(handle->motor_index, &monitor);

    motor_exec_confirm(device_id, monitor.actual_position);
}

/**
 * @brief 打印所有电机状态
 */
void motor_print_all_status(void)
{
    printf("\n========== Motor Status (All 12 Motors) ==========\n");

    for (uint8_t i = 0; i < MOTOR_MAX_HANDLES; i++) {
        MotorHandle* handle = motor_handle_get(i);
        if (handle == NULL || !handle->initialized) {
            printf("Motor[%2d]: NOT INITIALIZED\n", i);
            continue;
        }

        printf("Motor[%2d] CAN%d ID%d: ", i, handle->can_channel, handle->device_id);
        printf("Pos=%6.2f° Vel=%6.1frpm Torq=%5.2fNm Temp=%4.1f°C ",
               handle->current_position,
               handle->current_velocity,
               handle->current_torque,
               handle->temperature);

        if (handle->error_flags != 0) {
            printf("[ERROR:0x%04X]", handle->error_flags);
        } else {
            printf("[OK]");
        }

        printf("\n");
    }

    printf("==================================================\n\n");
}

/* ============================================================================
 * 紧急功能实现
 * ============================================================================ */

/**
 * @brief 紧急停止单个电机
 */
void motor_emergency_stop(uint8_t can_channel, uint8_t device_id)
{
    /* 失能电机 */
    motor_enable(can_channel, device_id, 0);

    /* 停止电机 */
    motor_start_stop(can_channel, device_id, 0);
}

/**
 * @brief 禁用单个电机
 */
void motor_disable(uint8_t can_channel, uint8_t device_id)
{
    motor_enable(can_channel, device_id, 0);
}

/**
 * @brief 清除单个电机错误
 */
void motor_clear_errors(uint8_t can_channel, uint8_t device_id)
{
    motor_clear_error(can_channel, device_id);

    /* 更新句柄 */
    MotorHandle* handle = get_motor_handle(can_channel, device_id);
    if (handle != NULL) {
        motor_handle_update_error(handle->motor_index, MOTOR_ERROR_NONE);
    }
}
