/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : log_system.h
  * @brief          : Thread-safe logging system with multiple log levels
  * @version        : 1.0.0
  * @date           : 2025-12-22
  ******************************************************************************
  * @attention
  *
  * Thread-safe logging system designed for FreeRTOS environment.
  * Supports multiple log levels: ERROR, WARN, INFO, DEBUG
  * Includes timestamp and task name in log output
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LOG_SYSTEM_H
#define __LOG_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Log level enumeration
 * @note Values must match system_config.h LogLevel_t definition
 */
typedef enum {
    LOG_LEVEL_NONE  = 0,  /* No logging */
    LOG_LEVEL_ERROR = 1,  /* Critical errors that may cause system failure */
    LOG_LEVEL_WARN  = 2,  /* Warnings that don't stop execution */
    LOG_LEVEL_INFO  = 3,  /* General information messages */
    LOG_LEVEL_DEBUG = 4,  /* Detailed debug information */
    LOG_LEVEL_TRACE = 5   /* Very detailed trace information */
} LogLevel_t;

/**
 * @brief Log statistics structure
 */
typedef struct {
    uint32_t error_count;    /* Total error messages logged */
    uint32_t warn_count;     /* Total warning messages logged */
    uint32_t info_count;     /* Total info messages logged */
    uint32_t debug_count;    /* Total debug messages logged */
    uint32_t total_count;    /* Total messages logged */
    uint32_t dropped_count;  /* Messages dropped due to mutex timeout */
} LogStats_t;

/* Exported constants --------------------------------------------------------*/

/**
 * @brief Current log level - adjust to filter log output
 * LOG_LEVEL_ERROR: Only errors
 * LOG_LEVEL_WARN:  Errors and warnings
 * LOG_LEVEL_INFO:  Errors, warnings, and info (recommended for normal operation)
 * LOG_LEVEL_DEBUG: All messages (use for debugging only)
 */
#ifndef LOG_LEVEL_CURRENT
#define LOG_LEVEL_CURRENT LOG_LEVEL_INFO
#endif

/**
 * @brief Enable/disable timestamp in log output
 */
#ifndef LOG_ENABLE_TIMESTAMP
#define LOG_ENABLE_TIMESTAMP 1
#endif

/**
 * @brief Enable/disable task name in log output
 */
#ifndef LOG_ENABLE_TASK_NAME
#define LOG_ENABLE_TASK_NAME 1
#endif

/**
 * @brief Enable/disable color codes in log output (for terminal support)
 */
#ifndef LOG_ENABLE_COLOR
#define LOG_ENABLE_COLOR 0
#endif

/* Exported macros -----------------------------------------------------------*/

/**
 * @brief Log an error message (always printed if LOG_LEVEL_CURRENT >= LOG_LEVEL_ERROR)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * Example: LOG_ERROR("Motor %d init failed, status=%d", motor_id, status);
 */
#define LOG_ERROR(fmt, ...) \
    do { \
        if (LOG_LEVEL_CURRENT >= LOG_LEVEL_ERROR) { \
            log_print(LOG_LEVEL_ERROR, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief Log a warning message (printed if LOG_LEVEL_CURRENT >= LOG_LEVEL_WARN)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * Example: LOG_WARN("Motor %d feedback timeout (%d ms)", motor_id, timeout_ms);
 */
#define LOG_WARN(fmt, ...) \
    do { \
        if (LOG_LEVEL_CURRENT >= LOG_LEVEL_WARN) { \
            log_print(LOG_LEVEL_WARN, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief Log an info message (printed if LOG_LEVEL_CURRENT >= LOG_LEVEL_INFO)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * Example: LOG_INFO("System initialized, uptime=%lu ms", HAL_GetTick());
 */
#define LOG_INFO(fmt, ...) \
    do { \
        if (LOG_LEVEL_CURRENT >= LOG_LEVEL_INFO) { \
            log_print(LOG_LEVEL_INFO, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief Log a debug message (printed if LOG_LEVEL_CURRENT >= LOG_LEVEL_DEBUG)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * Example: LOG_DEBUG("CAN TX: ID=0x%03X, data=%02X %02X", can_id, data[0], data[1]);
 */
#define LOG_DEBUG(fmt, ...) \
    do { \
        if (LOG_LEVEL_CURRENT >= LOG_LEVEL_DEBUG) { \
            log_print(LOG_LEVEL_DEBUG, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief Log a raw message without formatting (no timestamp, level, or function name)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * Useful for printing banners, tables, or continuation of previous log lines
 * Example: LOG_RAW("========================================\r\n");
 */
#define LOG_RAW(fmt, ...) \
    do { \
        log_print_raw(fmt, ##__VA_ARGS__); \
    } while(0)

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the logging system
 * @note Must be called before using any log functions
 * @note Safe to call multiple times (idempotent)
 * @retval true: success, false: failure
 */
bool log_system_init(void);

/**
 * @brief Deinitialize the logging system
 * @note Should be called during system shutdown
 */
void log_system_deinit(void);

/**
 * @brief Core logging function (thread-safe)
 * @param level: Log level
 * @param func: Function name (use __FUNCTION__)
 * @param line: Line number (use __LINE__)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * @note Normally you should use LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG macros instead
 */
void log_print(LogLevel_t level, const char* func, uint32_t line, const char* fmt, ...);

/**
 * @brief Print raw message without formatting (thread-safe)
 * @param fmt: printf-style format string
 * @param ...: variable arguments
 *
 * @note Normally you should use LOG_RAW macro instead
 */
void log_print_raw(const char* fmt, ...);

/**
 * @brief Get logging statistics
 * @param stats: Pointer to statistics structure to fill
 * @retval true: success, false: failure (stats is NULL)
 */
bool log_get_stats(LogStats_t* stats);

/**
 * @brief Reset logging statistics
 */
void log_reset_stats(void);

/**
 * @brief Print logging statistics to console
 */
void log_print_stats(void);

/**
 * @brief Print a formatted banner/separator line
 * @param title: Title text (can be NULL for simple separator)
 *
 * Example output with title:
 * ========== System Initialization ==========
 *
 * Example output without title (pass NULL):
 * ===========================================
 */
void log_print_banner(const char* title);

/**
 * @brief Print system information (uptime, heap, tasks, etc.)
 */
void log_print_system_info(void);

/* ============================================================================
 * V2.0: 运行时日志级别控制函数
 * ============================================================================ */

/**
 * @brief 设置运行时日志级别
 * @param level 新的日志级别
 */
void log_set_level(LogLevel_t level);

/**
 * @brief 获取当前日志级别
 * @return 当前日志级别
 */
LogLevel_t log_get_level(void);

/**
 * @brief 设置模块过滤器
 * @param module_mask 模块位掩码
 */
void log_set_module_filter(uint32_t module_mask);

/**
 * @brief 获取模块过滤器
 * @return 当前模块过滤器
 */
uint32_t log_get_module_filter(void);

/**
 * @brief 启用指定模块的日志
 * @param module_bit 模块位 (0-31)
 */
void log_enable_module(uint8_t module_bit);

/**
 * @brief 禁用指定模块的日志
 * @param module_bit 模块位 (0-31)
 */
void log_disable_module(uint8_t module_bit);

/**
 * @brief 打印当前日志配置
 */
void log_print_config(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_SYSTEM_H */
