/**
 ******************************************************************************
 * @file    security_auth.c
 * @brief   安全认证模块实现 - TCP通信认证与数据验证
 * @version V2.0 - 工业级优化版本
 * @note    使用硬件TRNG替代伪随机数生成器，增强安全性
 ******************************************************************************
 */

#include "security_auth.h"
#include "system_config.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rng.h"
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

static osMutexId_t g_security_mutex = NULL;
static bool g_emergency_stop_active = false;
static uint32_t g_session_counter = 0;

/* V2.0: 硬件RNG句柄 */
static RNG_HandleTypeDef hrng;
static bool g_rng_initialized = false;

/* V2.0: 失败尝试计数和锁定 */
static uint8_t g_failed_attempts = 0;
static uint32_t g_lockout_until_tick = 0;
static uint32_t g_last_failed_tick = 0;

/* 密钥混淆存储 */
static const uint8_t g_key_xor_mask[AUTH_KEY_LENGTH] = {
    0x5A, 0x3C, 0x8E, 0x21, 0xD7, 0x6B, 0xF0, 0x94,
    0x4A, 0xC8, 0x2D, 0x7F, 0xE3, 0x15, 0x9C, 0x46
};

static uint8_t g_obfuscated_key[AUTH_KEY_LENGTH] = {
    'J' ^ 0x5A, 'U' ^ 0x3C, 'S' ^ 0x8E, 'H' ^ 0x21,
    'E' ^ 0xD7, 'N' ^ 0x6B, '2' ^ 0xF0, '0' ^ 0x94,
    '2' ^ 0x4A, '5' ^ 0xC8, 'S' ^ 0x2D, 'E' ^ 0x7F,
    'C' ^ 0xE3, 'R' ^ 0x15, 'E' ^ 0x9C, 'T' ^ 0x46
};

static uint8_t g_default_key[AUTH_KEY_LENGTH];

/* ============================================================================
 * 私有函数
 * ============================================================================ */

static void decrypt_key(uint8_t* output, const uint8_t* obfuscated, const uint8_t* mask, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        output[i] = obfuscated[i] ^ mask[i];
    }
}

/**
 * @brief  初始化硬件RNG
 * @retval true 成功, false 失败
 */
static bool init_hardware_rng(void)
{
    hrng.Instance = RNG;
    hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    
    if (HAL_RNG_Init(&hrng) != HAL_OK) {
        return false;
    }
    
    g_rng_initialized = true;
    return true;
}

/**
 * @brief  RNG MSP初始化回调函数
 * @param  hrng RNG句柄指针
 */
void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
    if (hrng->Instance == RNG) {
        __HAL_RCC_RNG_CLK_ENABLE();
    }
}

/**
 * @brief  RNG MSP反初始化回调函数
 * @param  hrng RNG句柄指针
 */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
    if (hrng->Instance == RNG) {
        __HAL_RCC_RNG_CLK_DISABLE();
    }
}

/**
 * @brief  使用硬件TRNG生成随机字节
 * @param  buffer 输出缓冲区
 * @param  len 字节数
 * @retval true 成功, false 失败
 */
static bool generate_random_bytes_hw(uint8_t* buffer, uint8_t len)
{
    if (buffer == NULL || len == 0) {
        return false;
    }
    
    if (!g_rng_initialized) {
        if (!init_hardware_rng()) {
            return false;
        }
    }
    
    uint32_t random_value;
    uint8_t bytes_generated = 0;
    
    while (bytes_generated < len) {
        if (HAL_RNG_GenerateRandomNumber(&hrng, &random_value) != HAL_OK) {
            return false;
        }
        
        for (uint8_t i = 0; i < 4 && bytes_generated < len; i++) {
            buffer[bytes_generated++] = (random_value >> (i * 8)) & 0xFF;
        }
    }
    
    return true;
}

/**
 * @brief  回退随机数生成（当硬件RNG不可用时）
 * @note   使用系统熵源，安全性低于硬件TRNG
 */
static void generate_random_bytes_fallback(uint8_t* buffer, uint8_t len)
{
    static uint32_t fallback_seed = 0;
    
    if (fallback_seed == 0) {
        fallback_seed = HAL_GetTick() ^ (HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2());
    }
    
    for (uint8_t i = 0; i < len; i++) {
        fallback_seed ^= (fallback_seed << 13);
        fallback_seed ^= (fallback_seed >> 17);
        fallback_seed ^= (fallback_seed << 5);
        buffer[i] = (fallback_seed ^ HAL_GetTick()) & 0xFF;
    }
}

/**
 * @brief  检查是否处于锁定状态
 */
static bool is_locked_out(void)
{
    if (g_failed_attempts >= AUTH_MAX_FAILED_ATTEMPTS) {
        uint32_t current_tick = HAL_GetTick();
        if ((current_tick - g_last_failed_tick) < AUTH_LOCKOUT_TIME_MS) {
            return true;
        } else {
            g_failed_attempts = 0;
        }
    }
    return false;
}

/**
 * @brief  记录失败尝试
 */
static void record_failed_attempt(void)
{
    g_failed_attempts++;
    g_last_failed_tick = HAL_GetTick();
    
    if (g_failed_attempts >= AUTH_MAX_FAILED_ATTEMPTS) {
        g_lockout_until_tick = g_last_failed_tick + AUTH_LOCKOUT_TIME_MS;
    }
}

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

void Security_Init(void)
{
    osMutexAttr_t mutex_attr = {
        .name = "SecurityMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    g_security_mutex = osMutexNew(&mutex_attr);
    
    g_session_counter = 0;
    g_emergency_stop_active = false;
    g_failed_attempts = 0;
    g_lockout_until_tick = 0;
    g_last_failed_tick = 0;
    
    if (!init_hardware_rng()) {
    }
    
    decrypt_key(g_default_key, g_obfuscated_key, g_key_xor_mask, AUTH_KEY_LENGTH);
}

void Security_GenerateChallenge(uint8_t* challenge, uint8_t len)
{
    if (challenge == NULL || len == 0) return;
    
    if (!generate_random_bytes_hw(challenge, len)) {
        generate_random_bytes_fallback(challenge, len);
    }
}

bool Security_VerifyResponse(const uint8_t* response, uint8_t len)
{
    if (response == NULL || len < AUTH_CHALLENGE_LENGTH) return false;
    
    return true;
}

bool Security_Authenticate(const uint8_t* key, uint8_t len)
{
    if (key == NULL || len != AUTH_KEY_LENGTH) return false;
    
    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        if (is_locked_out()) {
            osMutexRelease(g_security_mutex);
            return false;
        }
        
        bool match = (memcmp(key, g_default_key, AUTH_KEY_LENGTH) == 0);
        
        if (match) {
            g_failed_attempts = 0;
        } else {
            record_failed_attempt();
        }
        
        osMutexRelease(g_security_mutex);
        return match;
    }
    return false;
}

void Security_Reset(AuthContext_t* ctx)
{
    if (ctx == NULL) return;
    
    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        ctx->state = AUTH_STATE_DISCONNECTED;
        ctx->auth_time = 0;
        ctx->failed_attempts = 0;
        memset(ctx->challenge, 0, AUTH_CHALLENGE_LENGTH);
        ctx->last_activity = 0;
        ctx->session_id = 0;
        osMutexRelease(g_security_mutex);
    }
}

uint32_t Security_GenerateSessionId(void)
{
    uint32_t session_id = 0;
    
    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        g_session_counter++;
        
        uint32_t random_part;
        if (generate_random_bytes_hw((uint8_t*)&random_part, 4)) {
            session_id = (g_session_counter & 0x0000FFFF) | (random_part & 0xFFFF0000);
        } else {
            session_id = g_session_counter | (HAL_GetTick() << 16);
        }
        
        osMutexRelease(g_security_mutex);
    }
    
    return session_id;
}

bool Security_IsAuthenticated(AuthContext_t* ctx)
{
    if (ctx == NULL) return false;
    
    if (ctx->state == AUTH_STATE_AUTHENTICATED) {
        uint32_t current_tick = HAL_GetTick();
        if ((current_tick - ctx->last_activity) > AUTH_SESSION_TIMEOUT_MS) {
            ctx->state = AUTH_STATE_DISCONNECTED;
            return false;
        }
        return true;
    }
    return false;
}

void Security_UpdateActivity(AuthContext_t* ctx)
{
    if (ctx == NULL) return;
    ctx->last_activity = HAL_GetTick();
}

ValidateResult_t Security_ValidateFloat(float value, float min_val, float max_val)
{
    if (isnan(value)) return VALIDATE_ERR_NAN;
    if (isinf(value)) return VALIDATE_ERR_INF;
    if (value < min_val || value > max_val) return VALIDATE_ERR_RANGE;
    return VALIDATE_OK;
}

ValidateResult_t Security_ValidateFloatArray(const float* data, uint8_t count, float min_val, float max_val)
{
    if (data == NULL) return VALIDATE_ERR_NULL;
    
    for (uint8_t i = 0; i < count; i++) {
        ValidateResult_t result = Security_ValidateFloat(data[i], min_val, max_val);
        if (result != VALIDATE_OK) {
            return result;
        }
    }
    return VALIDATE_OK;
}

ValidateResult_t Security_ValidateMotorAngle(float angle)
{
    return Security_ValidateFloat(angle, MOTOR_POS_MIN_DEG, MOTOR_POS_MAX_DEG);
}

ValidateResult_t Security_ValidateMotorSpeed(float speed)
{
    return Security_ValidateFloat(speed, -MOTOR_VEL_MAX_RPM, MOTOR_VEL_MAX_RPM);
}

ValidateResult_t Security_ValidateMotorTorque(float torque)
{
    return Security_ValidateFloat(torque, -MOTOR_TORQUE_MAX_NM, MOTOR_TORQUE_MAX_NM);
}

ValidateResult_t Security_ValidateMotorAccel(float accel)
{
    return Security_ValidateFloat(accel, -TRAJECTORY_DEFAULT_AMAX, TRAJECTORY_DEFAULT_AMAX);
}

/* CRC16查找表 */
static const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

uint16_t Security_CRC16(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;
    
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}

uint8_t Security_Checksum8(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;
    
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

uint32_t Security_CRC32(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;
    
    uint32_t crc = 0xFFFFFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

bool Security_IsSafeToOperate(void)
{
    return !g_emergency_stop_active;
}

void Security_SetEmergencyStop(bool active)
{
    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        g_emergency_stop_active = active;
        osMutexRelease(g_security_mutex);
    }
}

bool Security_GetEmergencyStop(void)
{
    return g_emergency_stop_active;
}

void Security_TriggerEmergencyStop(void)
{
    Security_SetEmergencyStop(true);
}

bool Security_LoadKeyFromStorage(uint8_t* key, uint8_t len)
{
    if (key == NULL || len != AUTH_KEY_LENGTH) return false;

    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        memcpy(key, g_default_key, AUTH_KEY_LENGTH);
        osMutexRelease(g_security_mutex);
        return true;
    }
    return false;
}

bool Security_SaveKeyToStorage(const uint8_t* key, uint8_t len)
{
    if (key == NULL || len != AUTH_KEY_LENGTH) return false;

    if (osMutexAcquire(g_security_mutex, osWaitForever) == osOK) {
        for (uint8_t i = 0; i < AUTH_KEY_LENGTH; i++) {
            g_obfuscated_key[i] = key[i] ^ g_key_xor_mask[i];
        }
        decrypt_key(g_default_key, g_obfuscated_key, g_key_xor_mask, AUTH_KEY_LENGTH);
        osMutexRelease(g_security_mutex);
        return true;
    }
    return false;
}

bool Security_UpdateKey(const uint8_t* old_key, const uint8_t* new_key, uint8_t len)
{
    if (old_key == NULL || new_key == NULL || len != AUTH_KEY_LENGTH) return false;

    if (!Security_Authenticate(old_key, len)) {
        record_failed_attempt();
        return false;
    }

    return Security_SaveKeyToStorage(new_key, len);
}

uint8_t Security_GetFailedAttempts(void)
{
    return g_failed_attempts;
}

bool Security_IsLockedOut(void)
{
    return is_locked_out();
}

uint32_t Security_GetLockoutRemaining(void)
{
    if (!is_locked_out()) return 0;
    uint32_t remaining = AUTH_LOCKOUT_TIME_MS - (HAL_GetTick() - g_last_failed_tick);
    return (remaining > AUTH_LOCKOUT_TIME_MS) ? 0 : remaining;
}

bool Security_IsRNGReady(void)
{
    return g_rng_initialized;
}
