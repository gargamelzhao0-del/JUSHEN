/**
 ******************************************************************************
 * @file    security_auth.h
 * @brief   安全认证模块 - TCP通信认证与数据验证
 * @version V2.0 - 工业级优化版本
 * @note    实现密钥认证、数据完整性校验、输入验证
 *          使用硬件TRNG增强安全性
 ******************************************************************************
 */

#ifndef __SECURITY_AUTH_H
#define __SECURITY_AUTH_H

#include "main.h"
#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================================
 * 认证配置常量
 * ============================================================================ */

#define AUTH_KEY_LENGTH         16
#define AUTH_CHALLENGE_LENGTH   8
#define AUTH_TIMEOUT_MS         5000
#define AUTH_MAX_FAILED_ATTEMPTS    5
#define AUTH_LOCKOUT_TIME_MS        30000
#define AUTH_SESSION_TIMEOUT_MS     3600000

/* ============================================================================
 * 认证状态枚举
 * ============================================================================ */

typedef enum {
    AUTH_STATE_DISCONNECTED = 0,
    AUTH_STATE_CHALLENGE_SENT,
    AUTH_STATE_AUTHENTICATED,
    AUTH_STATE_FAILED,
    AUTH_STATE_BLOCKED
} AuthState_t;

/* ============================================================================
 * 认证上下文结构体
 * ============================================================================ */

typedef struct {
    AuthState_t state;
    uint32_t auth_time;
    uint8_t failed_attempts;
    uint8_t challenge[AUTH_CHALLENGE_LENGTH];
    uint32_t last_activity;
    uint32_t session_id;
} AuthContext_t;

/* ============================================================================
 * 数据验证结果枚举
 * ============================================================================ */

typedef enum {
    VALIDATE_OK = 0,
    VALIDATE_ERR_NAN,
    VALIDATE_ERR_INF,
    VALIDATE_ERR_RANGE,
    VALIDATE_ERR_NULL,
    VALIDATE_ERR_CHECKSUM
} ValidateResult_t;

/* ============================================================================
 * 初始化和基本操作
 * ============================================================================ */

void Security_Init(void);
void Security_GenerateChallenge(uint8_t* challenge, uint8_t len);
bool Security_VerifyResponse(const uint8_t* response, uint8_t len);
bool Security_Authenticate(const uint8_t* key, uint8_t len);
void Security_Reset(AuthContext_t* ctx);
uint32_t Security_GenerateSessionId(void);
bool Security_IsAuthenticated(AuthContext_t* ctx);
void Security_UpdateActivity(AuthContext_t* ctx);

/* ============================================================================
 * 数据验证函数
 * ============================================================================ */

ValidateResult_t Security_ValidateFloat(float value, float min_val, float max_val);
ValidateResult_t Security_ValidateFloatArray(const float* data, uint8_t count, float min_val, float max_val);
ValidateResult_t Security_ValidateMotorAngle(float angle);
ValidateResult_t Security_ValidateMotorSpeed(float speed);
ValidateResult_t Security_ValidateMotorTorque(float torque);
ValidateResult_t Security_ValidateMotorAccel(float accel);

/* ============================================================================
 * 校验和计算函数
 * ============================================================================ */

uint16_t Security_CRC16(const uint8_t* data, uint16_t len);
uint8_t Security_Checksum8(const uint8_t* data, uint16_t len);
uint32_t Security_CRC32(const uint8_t* data, uint16_t len);

/* ============================================================================
 * 安全状态管理
 * ============================================================================ */

bool Security_IsSafeToOperate(void);
void Security_SetEmergencyStop(bool active);
bool Security_GetEmergencyStop(void);
void Security_TriggerEmergencyStop(void);

/* ============================================================================
 * 密钥存储
 * ============================================================================ */

bool Security_LoadKeyFromStorage(uint8_t* key, uint8_t len);
bool Security_SaveKeyToStorage(const uint8_t* key, uint8_t len);

/* ============================================================================
 * V2.0新增：安全状态查询
 * ============================================================================ */

uint8_t Security_GetFailedAttempts(void);
bool Security_IsLockedOut(void);
uint32_t Security_GetLockoutRemaining(void);
bool Security_IsRNGReady(void);

#endif /* __SECURITY_AUTH_H */
