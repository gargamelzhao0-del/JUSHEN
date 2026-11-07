/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : log_system.c
  * @brief          : Thread-safe logging system implementation
  * @version        : 2.0.0 - 工业级优化版本
  * @date           : 2025-12-22
  * @note           : V2.0: 添加运行时日志级别控制，过滤不需要的日志
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "log_system.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define LOG_BUFFER_SIZE 256
#define LOG_MUTEX_TIMEOUT_MS 100

/* ANSI color codes */
#define COLOR_RED     "\033[31m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RESET   "\033[0m"

/* V2.0: 运行时日志级别控制 */
static LogLevel_t g_log_level = LOG_LEVEL_INFO;  /* 默认日志级别 */
static uint32_t g_log_module_filter = 0xFFFFFFFF; /* 模块过滤器(位掩码) */

/* Private variables ---------------------------------------------------------*/
static osMutexId_t log_mutex = NULL;
static LogStats_t log_stats = {0};
static bool log_initialized = false;

/* Private function prototypes -----------------------------------------------*/
static const char* log_level_to_string(LogLevel_t level);
static const char* log_level_to_color(LogLevel_t level);
static void log_update_stats(LogLevel_t level);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize the logging system
 */
bool log_system_init(void)
{
    /* Prevent multiple initialization */
    if (log_initialized) {
        return true;
    }

    /* Create mutex for thread-safe logging */
    log_mutex = osMutexNew(NULL);
    if (log_mutex == NULL) {
        return false;
    }

    /* Reset statistics */
    memset(&log_stats, 0, sizeof(LogStats_t));

    log_initialized = true;
    return true;
}

/**
 * @brief Deinitialize the logging system
 */
void log_system_deinit(void)
{
    if (!log_initialized) {
        return;
    }

    if (log_mutex != NULL) {
        osMutexDelete(log_mutex);
        log_mutex = NULL;
    }

    log_initialized = false;
}

/**
 * @brief Core logging function with formatting
 */
void log_print(LogLevel_t level, const char* func, uint32_t line, const char* fmt, ...)
{
    char buffer[LOG_BUFFER_SIZE];
    va_list args;
    int offset = 0;

    /* V2.0: 运行时日志级别过滤 */
    if (level > g_log_level) {
        return;  /* 跳过低于当前日志级别的消息 */
    }

    /* Early exit if logging not initialized */
    if (!log_initialized) {
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        printf("\r\n");
        return;
    }

    /* Try to acquire mutex */
    if (osMutexAcquire(log_mutex, pdMS_TO_TICKS(LOG_MUTEX_TIMEOUT_MS)) != osOK) {
        log_stats.dropped_count++;
        return;
    }

    /* Build log message */
    #if LOG_ENABLE_COLOR
    offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "%s", log_level_to_color(level));
    #endif

    /* Add timestamp */
    #if LOG_ENABLE_TIMESTAMP
    uint32_t ticks = HAL_GetTick();
    uint32_t seconds = ticks / 1000;
    uint32_t milliseconds = ticks % 1000;
    offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "[%6u.%03u] ", (unsigned int)seconds, (unsigned int)milliseconds);
    #endif

    /* Add log level */
    offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "[%-5s] ", log_level_to_string(level));

    /* Add task name - only if scheduler is running */
    #if LOG_ENABLE_TASK_NAME
    /* Check if scheduler is running before calling pcTaskGetName */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        const char* task_name = pcTaskGetName(NULL);
        if (task_name != NULL) {
            offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "[%-16s] ", task_name);
        }
    } else {
        offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "[%-16s] ", "[PRE-OS]");
    }
    #endif

    /* Add function name and line number for errors and warnings */
    if (level <= LOG_LEVEL_WARN) {
        offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "%s:%u: ", func, (unsigned int)line);
    }

    /* Add user message */
    va_start(args, fmt);
    offset += vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, fmt, args);
    va_end(args);

    /* Add color reset */
    #if LOG_ENABLE_COLOR
    offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "%s", COLOR_RESET);
    #endif

    /* Add line termination */
    offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset, "\r\n");

    /* Print to console */
    printf("%s", buffer);

    /* Update statistics */
    log_update_stats(level);

    /* Release mutex */
    osMutexRelease(log_mutex);
}

/* ============================================================================
 * V2.0: 运行时日志级别控制函数
 * ============================================================================ */

/**
 * @brief 设置运行时日志级别
 * @param level 新的日志级别
 */
void log_set_level(LogLevel_t level)
{
    g_log_level = level;
    printf("[LOG] Log level set to: %s\r\n", log_level_to_string(level));
}

/**
 * @brief 获取当前日志级别
 * @return 当前日志级别
 */
LogLevel_t log_get_level(void)
{
    return g_log_level;
}

/**
 * @brief 设置模块过滤器
 * @param module_mask 模块位掩码
 */
void log_set_module_filter(uint32_t module_mask)
{
    g_log_module_filter = module_mask;
}

/**
 * @brief 获取模块过滤器
 * @return 当前模块过滤器
 */
uint32_t log_get_module_filter(void)
{
    return g_log_module_filter;
}

/**
 * @brief 启用指定模块的日志
 * @param module_bit 模块位 (0-31)
 */
void log_enable_module(uint8_t module_bit)
{
    if (module_bit < 32) {
        g_log_module_filter |= (1U << module_bit);
    }
}

/**
 * @brief 禁用指定模块的日志
 * @param module_bit 模块位 (0-31)
 */
void log_disable_module(uint8_t module_bit)
{
    if (module_bit < 32) {
        g_log_module_filter &= ~(1U << module_bit);
    }
}

/**
 * @brief 打印当前日志配置
 */
void log_print_config(void)
{
    printf("\r\n========== Log System Configuration ==========\r\n");
    printf("Log Level: %s\r\n", log_level_to_string(g_log_level));
    printf("Module Filter: 0x%08X\r\n", (unsigned int)g_log_module_filter);
    printf("Initialized: %s\r\n", log_initialized ? "Yes" : "No");
    printf("==============================================\r\n\r\n");
}

/**
 * @brief Print raw message without formatting
 */
void log_print_raw(const char* fmt, ...)
{
    va_list args;

    /* Early exit if logging not initialized */
    if (!log_initialized) {
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        return;
    }

    /* Acquire mutex */
    if (osMutexAcquire(log_mutex, pdMS_TO_TICKS(LOG_MUTEX_TIMEOUT_MS)) != osOK) {
        log_stats.dropped_count++;
        return;
    }

    /* Print message */
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    /* Release mutex */
    osMutexRelease(log_mutex);
}

/**
 * @brief Get logging statistics
 */
bool log_get_stats(LogStats_t* stats)
{
    if (stats == NULL || !log_initialized) {
        return false;
    }

    if (osMutexAcquire(log_mutex, pdMS_TO_TICKS(LOG_MUTEX_TIMEOUT_MS)) != osOK) {
        return false;
    }

    memcpy(stats, &log_stats, sizeof(LogStats_t));

    osMutexRelease(log_mutex);
    return true;
}

/**
 * @brief Reset logging statistics
 */
void log_reset_stats(void)
{
    if (!log_initialized) {
        return;
    }

    if (osMutexAcquire(log_mutex, pdMS_TO_TICKS(LOG_MUTEX_TIMEOUT_MS)) != osOK) {
        return;
    }

    memset(&log_stats, 0, sizeof(LogStats_t));

    osMutexRelease(log_mutex);
}

/**
 * @brief Print logging statistics
 */
void log_print_stats(void)
{
    LogStats_t stats;

    if (!log_get_stats(&stats)) {
        printf("ERROR: Failed to get log statistics\r\n");
        return;
    }

    log_print_banner("Logging Statistics");
    LOG_RAW("  Total Messages:   %u\r\n", (unsigned int)stats.total_count);
    LOG_RAW("  ERROR Messages:   %u\r\n", (unsigned int)stats.error_count);
    LOG_RAW("  WARN Messages:    %u\r\n", (unsigned int)stats.warn_count);
    LOG_RAW("  INFO Messages:    %u\r\n", (unsigned int)stats.info_count);
    LOG_RAW("  DEBUG Messages:   %u\r\n", (unsigned int)stats.debug_count);
    LOG_RAW("  Dropped Messages: %u\r\n", (unsigned int)stats.dropped_count);
    log_print_banner(NULL);
}

/**
 * @brief Print a formatted banner
 */
void log_print_banner(const char* title)
{
    if (title == NULL) {
        LOG_RAW("===========================================\r\n");
    } else {
        int title_len = strlen(title);
        int total_len = 50;
        int padding = (total_len - title_len - 2) / 2;

        LOG_RAW("\r\n");
        for (int i = 0; i < padding; i++) {
            LOG_RAW("=");
        }
        LOG_RAW(" %s ", title);
        for (int i = 0; i < padding; i++) {
            LOG_RAW("=");
        }
        LOG_RAW("\r\n");
    }
}

/**
 * @brief Print system information
 */
void log_print_system_info(void)
{
    uint32_t uptime_s = HAL_GetTick() / 1000;
    uint32_t uptime_m = uptime_s / 60;
    uint32_t uptime_h = uptime_m / 60;

    log_print_banner("System Information");

    LOG_RAW("  System Clock:     %u MHz\r\n", (unsigned int)(SystemCoreClock / 1000000));
    LOG_RAW("  Uptime:           %uh %um %us\r\n",
            (unsigned int)uptime_h, (unsigned int)(uptime_m % 60), (unsigned int)(uptime_s % 60));

    #if (configUSE_TRACE_FACILITY == 1)
    LOG_RAW("  Heap Free:        %u bytes\r\n", (unsigned int)xPortGetFreeHeapSize());
    LOG_RAW("  Heap Min Free:    %u bytes\r\n", (unsigned int)xPortGetMinimumEverFreeHeapSize());
    #endif

    LOG_RAW("  Tasks Running:    %u\r\n", (unsigned int)uxTaskGetNumberOfTasks());

    log_print_banner(NULL);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Convert log level to string
 */
static const char* log_level_to_string(LogLevel_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR: return "ERROR";
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_INFO:  return "INFO";
        case LOG_LEVEL_DEBUG: return "DEBUG";
        default:              return "UNKNOWN";
    }
}

/**
 * @brief Convert log level to ANSI color code
 */
static const char* log_level_to_color(LogLevel_t level)
{
    #if LOG_ENABLE_COLOR
    switch (level) {
        case LOG_LEVEL_ERROR: return COLOR_RED;
        case LOG_LEVEL_WARN:  return COLOR_YELLOW;
        case LOG_LEVEL_INFO:  return COLOR_GREEN;
        case LOG_LEVEL_DEBUG: return COLOR_CYAN;
        default:              return COLOR_RESET;
    }
    #else
    (void)level;
    return "";
    #endif
}

/**
 * @brief Update logging statistics
 */
static void log_update_stats(LogLevel_t level)
{
    log_stats.total_count++;

    switch (level) {
        case LOG_LEVEL_ERROR:
            log_stats.error_count++;
            break;
        case LOG_LEVEL_WARN:
            log_stats.warn_count++;
            break;
        case LOG_LEVEL_INFO:
            log_stats.info_count++;
            break;
        case LOG_LEVEL_DEBUG:
            log_stats.debug_count++;
            break;
        default:
            break;
    }
}
