/**
 ******************************************************************************
 * @file    error_handling.h
 * @brief   系统错误处理和异常管理 - 工业级错误处理框架
 * @version V1.0
 * @note    工业级代码优化：统一错误处理，增强系统可靠性
 ******************************************************************************
 */

#ifndef __ERROR_HANDLING_H
#define __ERROR_HANDLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"

/* ============================================================================
 * 错误类型定义
 * ============================================================================
 */

/**
 * @brief 系统错误类型枚举
 */
typedef enum {
    ERROR_TYPE_NONE = 0,          /* 无错误 */
    ERROR_TYPE_HARDWARE = 1,      /* 硬件错误 */
    ERROR_TYPE_SOFTWARE = 2,      /* 软件错误 */
    ERROR_TYPE_COMMUNICATION = 3, /* 通信错误 */
    ERROR_TYPE_SECURITY = 4,      /* 安全错误 */
    ERROR_TYPE_RESOURCE = 5,      /* 资源错误 */
    ERROR_TYPE_TIMEOUT = 6,       /* 超时错误 */
    ERROR_TYPE_CONFIG = 7,        /* 配置错误 */
    ERROR_TYPE_MAX
} ErrorType_t;

/**
 * @brief 错误严重程度
 */
typedef enum {
    ERROR_LEVEL_INFO = 0,     /* 信息级错误 */
    ERROR_LEVEL_WARNING = 1,  /* 警告级错误 */
    ERROR_LEVEL_ERROR = 2,    /* 错误级错误 */
    ERROR_LEVEL_CRITICAL = 3, /* 严重错误 */
    ERROR_LEVEL_FATAL = 4,    /* 致命错误 */
    ERROR_LEVEL_MAX
} ErrorLevel_t;

/**
 * @brief 错误恢复策略
 */
typedef enum {
    RECOVERY_NONE = 0,          /* 无恢复策略 */
    RECOVERY_RETRY = 1,         /* 重试操作 */
    RECOVERY_FALLBACK = 2,      /* 回退到备用方案 */
    RECOVERY_RESET = 3,         /* 重置模块 */
    RECOVERY_RESTART = 4,       /* 重启系统 */
    RECOVERY_EMERGENCY_STOP = 5, /* 紧急停止 */
    RECOVERY_MAX
} RecoveryStrategy_t;

/* ============================================================================
 * 错误管理结构体
 * ============================================================================
 */

/**
 * @brief 错误信息结构体
 */
typedef struct {
    uint32_t error_id;           /* 错误ID */
    ErrorType_t error_type;      /* 错误类型 */
    ErrorLevel_t error_level;    /* 错误严重程度 */
    uint32_t timestamp;          /* 错误发生时间戳 */
    uint32_t module_id;          /* 模块ID */
    uint32_t error_code;         /* 具体错误码 */
    const char* error_message;   /* 错误描述 */
    RecoveryStrategy_t recovery; /* 恢复策略 */
    uint32_t retry_count;        /* 重试次数 */
    bool resolved;               /* 是否已解决 */
} ErrorInfo_t;

/**
 * @brief 错误统计信息
 */
typedef struct {
    uint32_t total_errors;       /* 总错误数 */
    uint32_t errors_by_type[ERROR_TYPE_MAX]; /* 按类型统计 */
    uint32_t errors_by_level[ERROR_LEVEL_MAX]; /* 按级别统计 */
    uint32_t recovered_errors;   /* 已恢复的错误数 */
    uint32_t critical_errors;    /* 严重错误数 */
    uint32_t fatal_errors;       /* 致命错误数 */
} ErrorStats_t;

/**
 * @brief 错误回调函数类型
 */
typedef void (*ErrorCallback_t)(const ErrorInfo_t* error);

/* ============================================================================
 * 错误管理API
 * ============================================================================
 */

/**
 * @brief 初始化错误处理系统
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_Init(void);

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
                                 const char* message, RecoveryStrategy_t recovery);

/**
 * @brief 解决错误
 * @param error_id 错误ID
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_ResolveError(uint32_t error_id);

/**
 * @brief 获取错误信息
 * @param error_id 错误ID
 * @param error_info 错误信息指针
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_GetErrorInfo(uint32_t error_id, ErrorInfo_t* error_info);

/**
 * @brief 获取错误统计信息
 * @param stats 错误统计信息指针
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_GetStats(ErrorStats_t* stats);

/**
 * @brief 注册错误回调函数
 * @param callback 回调函数
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_RegisterCallback(ErrorCallback_t callback);

/**
 * @brief 执行错误恢复策略
 * @param error_id 错误ID
 * @return 成功返回true，失败返回false
 */
bool ErrorHandler_ExecuteRecovery(uint32_t error_id);

/**
 * @brief 检查系统健康状态
 * @return 系统健康状态，true表示健康，false表示存在严重错误
 */
bool ErrorHandler_CheckSystemHealth(void);

/**
 * @brief 重置错误统计
 */
void ErrorHandler_ResetStats(void);

/* ============================================================================
 * 错误处理宏
 * ============================================================================
 */

/**
 * @brief 记录错误的宏
 */
#define ERROR_RECORD(type, level, module, code, msg, recovery) \
    ErrorHandler_RecordError((type), (level), (module), (code), (msg), (recovery))

/**
 * @brief 记录信息级错误
 */
#define ERROR_INFO(module, code, msg) \
    ERROR_RECORD(ERROR_TYPE_SOFTWARE, ERROR_LEVEL_INFO, (module), (code), (msg), RECOVERY_NONE)

/**
 * @brief 记录警告级错误
 */
#define ERROR_WARN(module, code, msg) \
    ERROR_RECORD(ERROR_TYPE_SOFTWARE, ERROR_LEVEL_WARNING, (module), (code), (msg), RECOVERY_RETRY)

/**
 * @brief 记录错误级错误
 */
#define ERROR_ERROR(module, code, msg) \
    ERROR_RECORD(ERROR_TYPE_SOFTWARE, ERROR_LEVEL_ERROR, (module), (code), (msg), RECOVERY_FALLBACK)

/**
 * @brief 记录严重错误
 */
#define ERROR_CRITICAL(module, code, msg) \
    ERROR_RECORD(ERROR_TYPE_SOFTWARE, ERROR_LEVEL_CRITICAL, (module), (code), (msg), RECOVERY_RESET)

/**
 * @brief 记录致命错误
 */
#define ERROR_FATAL(module, code, msg) \
    ERROR_RECORD(ERROR_TYPE_SOFTWARE, ERROR_LEVEL_FATAL, (module), (code), (msg), RECOVERY_EMERGENCY_STOP)

/* ============================================================================
 * 模块ID定义
 * ============================================================================
 */

#define MODULE_MAIN           0x0001  /* 主模块 */
#define MODULE_MOTOR_CONTROL  0x0002  /* 电机控制模块 */
#define MODULE_CAN_COMM       0x0003  /* CAN通信模块 */
#define MODULE_TRAJECTORY     0x0004  /* 轨迹规划模块 */
#define MODULE_SECURITY       0x0005  /* 安全模块 */
#define MODULE_LOGGING        0x0006  /* 日志模块 */
#define MODULE_TCP_COMM       0x0007  /* TCP通信模块 */
#define MODULE_SENSOR         0x0008  /* 传感器模块 */
#define MODULE_POWER          0x0009  /* 电源管理模块 */
#define MODULE_WATCHDOG       0x000A  /* 看门狗模块 */

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_HANDLING_H */