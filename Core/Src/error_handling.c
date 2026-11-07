/**
 ******************************************************************************
 * @file    error_handling.c
 * @brief   系统错误处理和异常管理 - 工业级错误处理框架实现
 * @version V1.0
 * @note    工业级代码优化：统一错误处理，增强系统可靠性
 ******************************************************************************
 */

#include "error_handling.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "log_system.h"
#include "security_auth.h"
#include <string.h>

/* 前向声明 */
void Security_TriggerEmergencyStop(void);

/* ============================================================================
 * 私有定义
 * ============================================================================
 */

#define MAX_ERROR_QUEUE_SIZE 64  /* V12.0: 最大错误队列大小（从32增加到64） */
#define ERROR_QUEUE_TIMEOUT_MS 100  /* 错误队列操作超时时间 */
#define ERROR_DEDUP_INTERVAL_MS 200  /* V12.0: 相同错误去重间隔（毫秒） */

/* ============================================================================
 * 私有变量
 * ============================================================================
 */

static osMutexId_t error_mutex = NULL;  /* 错误管理互斥锁 */
static osMessageQueueId_t error_queue = NULL;  /* 错误消息队列 */
static ErrorStats_t error_stats = {0};  /* 错误统计信息 */
static uint32_t error_id_counter = 1;  /* 错误ID计数器 */
static ErrorCallback_t error_callback = NULL;  /* 错误回调函数 */
static bool error_handler_initialized = false;  /* 错误处理系统初始化标志 */

/* V12.0: 错误去重机制 */
static uint32_t g_last_error_code = 0;        /* 上次错误码 */
static uint32_t g_last_error_module = 0;      /* 上次错误模块 */
static uint32_t g_last_error_tick = 0;        /* 上次错误时间 */
static uint32_t g_error_dedup_count = 0;      /* 去重计数器 */

/* ============================================================================
 * 私有函数原型
 * ============================================================================
 */

static void error_handler_process_error(const ErrorInfo_t* error);
static void error_handler_update_stats(const ErrorInfo_t* error);
static bool error_handler_execute_recovery_strategy(const ErrorInfo_t* error);
static const char* error_type_to_string(ErrorType_t type);
static const char* error_level_to_string(ErrorLevel_t level);
static const char* recovery_strategy_to_string(RecoveryStrategy_t strategy);

/* ============================================================================
 * 公共函数实现
 * ============================================================================
 */

/**
 * @brief 初始化错误处理系统
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_Init(void)
{
    /* 防止重复初始化 */
    if (error_handler_initialized) {
        return true;
    }

    /* 创建互斥锁 */
    error_mutex = osMutexNew(NULL);
    if (error_mutex == NULL) {
        return false;
    }

    /* 创建错误消息队列 */
    error_queue = osMessageQueueNew(MAX_ERROR_QUEUE_SIZE, sizeof(ErrorInfo_t), NULL);
    if (error_queue == NULL) {
        osMutexDelete(error_mutex);
        error_mutex = NULL;
        return false;
    }

    /* 重置错误统计 */
    ErrorHandler_ResetStats();

    error_handler_initialized = true;
    /* Use printf instead of LOG_INFO because scheduler is not yet started */
    printf("[INFO] Error handling system initialized\r\n");
    return true;
}

/**
 * @brief 记录错误信息
 * @param type 错误类型
 * @param level 错误严重程度
 * @param module_id 模块ID
 * @param error_code 具体错误码
 * @param message 错误描述
 * @param recovery 恢复策略
 * @return 错误ID
 */
uint32_t ErrorHandler_RecordError(ErrorType_t type, ErrorLevel_t level, 
                                 uint32_t module_id, uint32_t error_code, 
                                 const char* message, RecoveryStrategy_t recovery)
{
    ErrorInfo_t error;
    uint32_t error_id = 0;
    uint32_t current_tick = HAL_GetTick();

    /* 检查初始化状态 */
    if (!error_handler_initialized) {
        /* 如果未初始化，直接打印错误 */
        printf("[ERROR] Error handling system not initialized: %s\r\n", message);
        return 0;
    }

    /* V12.0: 错误去重检查 - 相同模块的相同错误在短时间内只记录一次 */
    if (error_code == g_last_error_code && module_id == g_last_error_module) {
        uint32_t elapsed = current_tick - g_last_error_tick;
        if (elapsed < ERROR_DEDUP_INTERVAL_MS) {
            g_error_dedup_count++;
            return 0;  /* 跳过重复错误 */
        }
    }

    /* 加锁保护 */
    if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        return 0;
    }

    /* V12.0: 更新去重状态 */
    g_last_error_code = error_code;
    g_last_error_module = module_id;
    g_last_error_tick = current_tick;
    if (g_error_dedup_count > 0) {
        LOG_INFO("Error dedup: %lu duplicate errors suppressed", (unsigned long)g_error_dedup_count);
        g_error_dedup_count = 0;
    }

    /* 生成错误ID */
    error_id = error_id_counter++;

    /* 填充错误信息 */
    error.error_id = error_id;
    error.error_type = type;
    error.error_level = level;
    error.timestamp = current_tick;
    error.module_id = module_id;
    error.error_code = error_code;
    error.error_message = message;
    error.recovery = recovery;
    error.retry_count = 0;
    error.resolved = false;

    /* 更新错误统计 */
    error_handler_update_stats(&error);

    /* 释放锁 */
    osMutexRelease(error_mutex);

    /* 发送错误到队列 */
    if (osMessageQueuePut(error_queue, &error, 0, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        LOG_WARN("Error queue full, dropping error: %s", message);
        return 0;
    }

    /* 处理错误 */
    error_handler_process_error(&error);

    return error_id;
}

/**
 * @brief 解决错误
 * @param error_id 错误ID
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_ResolveError(uint32_t error_id)
{
    ErrorInfo_t error;
    uint32_t i;

    if (!error_handler_initialized) {
        return false;
    }

    /* 加锁保护 */
    if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        return false;
    }

    /* 查找并解决错误 */
    for (i = 0; i < MAX_ERROR_QUEUE_SIZE; i++) {
        if (osMessageQueueGet(error_queue, &error, NULL, 0) == osOK) {
            if (error.error_id == error_id && !error.resolved) {
                error.resolved = true;
                error_stats.recovered_errors++;
                // 将更新后的错误放回队列
                osMessageQueuePut(error_queue, &error, 0, 0);
                break;
            } else {
                // 将非目标错误放回队列
                osMessageQueuePut(error_queue, &error, 0, 0);
            }
        } else {
            break;
        }
    }

    /* 释放锁 */
    osMutexRelease(error_mutex);

    if (i < MAX_ERROR_QUEUE_SIZE) {
        LOG_INFO("Error %u resolved", error_id);
        return true;
    }

    return false;
}

/**
 * @brief 获取错误信息
 * @param error_id 错误ID
 * @param error_info 错误信息指针
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_GetErrorInfo(uint32_t error_id, ErrorInfo_t* error_info)
{
    ErrorInfo_t error;
    uint32_t i;

    if (!error_handler_initialized || error_info == NULL) {
        return false;
    }

    /* 加锁保护 */
    if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        return false;
    }

    /* 查找错误 */
    for (i = 0; i < MAX_ERROR_QUEUE_SIZE; i++) {
        if (osMessageQueueGet(error_queue, &error, NULL, 0) == osOK) {
            if (error.error_id == error_id) {
                memcpy(error_info, &error, sizeof(ErrorInfo_t));
                // 将错误放回队列
                osMessageQueuePut(error_queue, &error, 0, 0);
                osMutexRelease(error_mutex);
                return true;
            } else {
                // 将非目标错误放回队列
                osMessageQueuePut(error_queue, &error, 0, 0);
            }
        } else {
            break;
        }
    }

    /* 释放锁 */
    osMutexRelease(error_mutex);
    return false;
}

/**
 * @brief 获取错误统计信息
 * @param stats 错误统计信息指针
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_GetStats(ErrorStats_t* stats)
{
    if (!error_handler_initialized || stats == NULL) {
        return false;
    }

    /* 加锁保护 */
    if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        return false;
    }

    /* 复制统计信息 */
    memcpy(stats, &error_stats, sizeof(ErrorStats_t));

    /* 释放锁 */
    osMutexRelease(error_mutex);
    return true;
}

/**
 * @brief 注册错误回调函数
 * @param callback 回调函数
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_RegisterCallback(ErrorCallback_t callback)
{
    if (!error_handler_initialized) {
        return false;
    }

    /* 加锁保护 */
    if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) != osOK) {
        return false;
    }

    /* 注册回调 */
    error_callback = callback;

    /* 释放锁 */
    osMutexRelease(error_mutex);
    return true;
}

/**
 * @brief 执行错误恢复策略
 * @param error_id 错误ID
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_ExecuteRecovery(uint32_t error_id)
{
    ErrorInfo_t error;

    if (!error_handler_initialized) {
        return false;
    }

    /* 获取错误信息 */
    if (!ErrorHandler_GetErrorInfo(error_id, &error)) {
        return false;
    }

    /* 执行恢复策略 */
    return error_handler_execute_recovery_strategy(&error);
}

/**
 * @brief 检查系统健康状态
 * @return 系统健康状态，true表示健康，false表示存在严重错误
 */
bool ErrorHandler_CheckSystemHealth(void)
{
    ErrorStats_t stats;

    if (!error_handler_initialized) {
        return false;
    }

    /* 获取错误统计 */
    if (!ErrorHandler_GetStats(&stats)) {
        return false;
    }

    /* 检查是否存在未解决的严重错误 */
    if (stats.critical_errors > 0 || stats.fatal_errors > 0) {
        return false;
    }

    return true;
}

/**
 * @brief 重置错误统计
 */
void ErrorHandler_ResetStats(void)
{
    /* 加锁保护 */
    if (error_mutex != NULL) {
        if (osMutexAcquire(error_mutex, pdMS_TO_TICKS(ERROR_QUEUE_TIMEOUT_MS)) == osOK) {
            memset(&error_stats, 0, sizeof(ErrorStats_t));
            osMutexRelease(error_mutex);
        }
    } else {
        memset(&error_stats, 0, sizeof(ErrorStats_t));
    }
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================
 */

/**
 * @brief 处理错误
 * @param error 错误信息
 */
static void error_handler_process_error(const ErrorInfo_t* error)
{
    /* 根据错误级别记录日志 */
    switch (error->error_level) {
        case ERROR_LEVEL_INFO:
            LOG_INFO("[ERROR] ID:%u %s: %s (Module:0x%04X, Code:0x%08X)",
                    error->error_id, error_type_to_string(error->error_type),
                    error->error_message, error->module_id, error->error_code);
            break;
        case ERROR_LEVEL_WARNING:
            LOG_WARN("[ERROR] ID:%u %s: %s (Module:0x%04X, Code:0x%08X)",
                    error->error_id, error_type_to_string(error->error_type),
                    error->error_message, error->module_id, error->error_code);
            break;
        case ERROR_LEVEL_ERROR:
            LOG_ERROR("[ERROR] ID:%u %s: %s (Module:0x%04X, Code:0x%08X)",
                     error->error_id, error_type_to_string(error->error_type),
                     error->error_message, error->module_id, error->error_code);
            break;
        case ERROR_LEVEL_CRITICAL:
            LOG_ERROR("[CRITICAL] ID:%u %s: %s (Module:0x%04X, Code:0x%08X, Recovery:%s)",
                     error->error_id, error_type_to_string(error->error_type),
                     error->error_message, error->module_id, error->error_code,
                     recovery_strategy_to_string(error->recovery));
            break;
        case ERROR_LEVEL_FATAL:
            LOG_ERROR("[FATAL] ID:%u %s: %s (Module:0x%04X, Code:0x%08X, Recovery:%s)",
                     error->error_id, error_type_to_string(error->error_type),
                     error->error_message, error->module_id, error->error_code,
                     recovery_strategy_to_string(error->recovery));
            break;
        default:
            break;
    }

    /* 执行恢复策略 */
    error_handler_execute_recovery_strategy(error);

    /* 调用回调函数 */
    if (error_callback != NULL) {
        error_callback(error);
    }
}

/**
 * @brief 更新错误统计
 * @param error 错误信息
 */
static void error_handler_update_stats(const ErrorInfo_t* error)
{
    error_stats.total_errors++;
    
    if (error->error_type < ERROR_TYPE_MAX) {
        error_stats.errors_by_type[error->error_type]++;
    }
    
    if (error->error_level < ERROR_LEVEL_MAX) {
        error_stats.errors_by_level[error->error_level]++;
    }
    
    if (error->error_level >= ERROR_LEVEL_CRITICAL) {
        error_stats.critical_errors++;
    }
    
    if (error->error_level == ERROR_LEVEL_FATAL) {
        error_stats.fatal_errors++;
    }
}

/**
 * @brief 执行错误恢复策略
 * @param error 错误信息
 * @return 成功返回true，失败返回false
 */
static bool error_handler_execute_recovery_strategy(const ErrorInfo_t* error)
{
    switch (error->recovery) {
        case RECOVERY_NONE:
            /* 无恢复策略 */
            break;
        
        case RECOVERY_RETRY:
            /* 重试操作 - 这里可以添加重试逻辑 */
            LOG_INFO("Executing recovery: RETRY for error %u", error->error_id);
            break;
        
        case RECOVERY_FALLBACK:
            /* 回退到备用方案 */
            LOG_INFO("Executing recovery: FALLBACK for error %u", error->error_id);
            break;
        
        case RECOVERY_RESET:
            /* 重置模块 */
            LOG_INFO("Executing recovery: RESET for error %u", error->error_id);
            break;
        
        case RECOVERY_RESTART:
            /* 重启系统 */
            LOG_ERROR("Executing recovery: SYSTEM RESTART for error %u", error->error_id);
            /* 这里可以添加系统重启逻辑 */
            break;
        
        case RECOVERY_EMERGENCY_STOP:
            /* 紧急停止 */
            LOG_ERROR("Executing recovery: EMERGENCY STOP for error %u", error->error_id);
            Security_TriggerEmergencyStop();
            break;
        
        default:
            break;
    }

    return true;
}

/**
 * @brief 错误类型转字符串
 * @param type 错误类型
 * @return 错误类型字符串
 */
static const char* error_type_to_string(ErrorType_t type)
{
    switch (type) {
        case ERROR_TYPE_NONE:
            return "NONE";
        case ERROR_TYPE_HARDWARE:
            return "HARDWARE";
        case ERROR_TYPE_SOFTWARE:
            return "SOFTWARE";
        case ERROR_TYPE_COMMUNICATION:
            return "COMMUNICATION";
        case ERROR_TYPE_SECURITY:
            return "SECURITY";
        case ERROR_TYPE_RESOURCE:
            return "RESOURCE";
        case ERROR_TYPE_TIMEOUT:
            return "TIMEOUT";
        case ERROR_TYPE_CONFIG:
            return "CONFIG";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 错误级别转字符串
 * @param level 错误级别
 * @return 错误级别字符串
 */
static const char* error_level_to_string(ErrorLevel_t level)
{
    switch (level) {
        case ERROR_LEVEL_INFO:
            return "INFO";
        case ERROR_LEVEL_WARNING:
            return "WARNING";
        case ERROR_LEVEL_ERROR:
            return "ERROR";
        case ERROR_LEVEL_CRITICAL:
            return "CRITICAL";
        case ERROR_LEVEL_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 恢复策略转字符串
 * @param strategy 恢复策略
 * @return 恢复策略字符串
 */
static const char* recovery_strategy_to_string(RecoveryStrategy_t strategy)
{
    switch (strategy) {
        case RECOVERY_NONE:
            return "NONE";
        case RECOVERY_RETRY:
            return "RETRY";
        case RECOVERY_FALLBACK:
            return "FALLBACK";
        case RECOVERY_RESET:
            return "RESET";
        case RECOVERY_RESTART:
            return "RESTART";
        case RECOVERY_EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        default:
            return "UNKNOWN";
    }
}