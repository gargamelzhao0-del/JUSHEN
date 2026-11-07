/**
 ******************************************************************************
 * @file    param_config.c
 * @brief   系统参数配置管理实现 - FLASH持久化存储
 ******************************************************************************
 */

#include "param_config.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash.h"
#include "log_system.h"
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 当前参数配置（内存中的副本）*/
static ParamConfig_t current_params;

/* 参数系统初始化标志 */
static bool param_initialized = false;

/* V10.0: Bank轮换写入计数器 - 减少FLASH磨损 */
static uint8_t g_current_write_bank = 1;  /* 当前写入Bank (1或2) */
static uint32_t g_write_count = 0;        /* 写入计数器 */
#define PARAM_MIN_SAVE_INTERVAL_MS  5000  /* 最小保存间隔5秒 */
static uint32_t g_last_save_time_ms = 0;  /* 上次保存时间 */

/* CRC32查找表（用于快速计算CRC32）*/
static uint32_t crc32_table[256];

/* ============================================================================
 * 私有函数声明
 * ============================================================================ */

static void init_crc32_table(void);
static uint32_t calculate_crc32(const uint8_t *data, uint32_t length);
static void load_default_params(void);
static bool validate_params(const ParamConfig_t *params);
static int flash_unlock_and_erase_sector(uint32_t sector_addr);
static int flash_write_data(uint32_t dest_addr, const uint8_t *data, uint32_t length);

/* ============================================================================
 * 公共函数实现 - 初始化和控制
 * ============================================================================ */

/**
 * @brief  初始化参数系统
 */
int param_init(void)
{
    if (param_initialized) {
        LOG_WARN("[Param] Already initialized");
        return 0;
    }

    LOG_INFO("[Param] Initializing parameter system...");

    /* 初始化CRC32查找表 */
    init_crc32_table();

    /* 尝试从FLASH加载参数 */
    if (param_load() == 0) {
        LOG_INFO("[Param] Parameters loaded from FLASH successfully");
    } else {
        LOG_WARN("[Param] Failed to load parameters, using defaults");
        load_default_params();
    }

    param_initialized = true;
    LOG_INFO("[Param] Parameter system initialized");

    return 0;
}

/**
 * @brief  从FLASH加载参数
 */
int param_load(void)
{
    ParamConfig_t temp_params;

    LOG_INFO("[Param] Loading parameters from FLASH...");

    /* 尝试从Bank1加载 */
    if (param_flash_read(1, &temp_params) == 0) {
        if (validate_params(&temp_params)) {
            memcpy(&current_params, &temp_params, sizeof(ParamConfig_t));
            LOG_INFO("[Param] Loaded from Bank1 (primary)");
            return 0;
        } else {
            LOG_WARN("[Param] Bank1 validation failed");
        }
    }

    /* Bank1失败，尝试从Bank2加载 */
    if (param_flash_read(2, &temp_params) == 0) {
        if (validate_params(&temp_params)) {
            memcpy(&current_params, &temp_params, sizeof(ParamConfig_t));
            LOG_WARN("[Param] Loaded from Bank2 (backup)");
            return 0;
        } else {
            LOG_WARN("[Param] Bank2 validation failed");
        }
    }

    /* V12.0: 两个Bank都失败，加载默认参数并保存到FLASH */
    LOG_ERROR("[Param] Both banks failed, loading defaults");
    load_default_params();
    
    /* V12.0: 首次运行时自动保存默认参数到FLASH */
    LOG_INFO("[Param] Saving default parameters to FLASH...");
    param_initialized = true;  /* 临时设置标志以允许保存 */
    int save_result = param_save();
    if (save_result == 0) {
        LOG_INFO("[Param] Default parameters saved successfully");
    } else {
        LOG_WARN("[Param] Failed to save default parameters (will retry next boot)");
    }
    
    return -1;
}

/**
 * @brief  保存参数到FLASH
 * @note   V10.0: 采用Bank轮换写入策略，减少FLASH磨损
 */
int param_save(void)
{
    if (!param_initialized) {
        LOG_ERROR("[Param] Not initialized");
        return -1;
    }

    uint32_t current_time = HAL_GetTick();
    if (g_last_save_time_ms > 0 && 
        (current_time - g_last_save_time_ms) < PARAM_MIN_SAVE_INTERVAL_MS) {
        LOG_WARN("[Param] Save too frequent, skipped (min interval: %dms)", PARAM_MIN_SAVE_INTERVAL_MS);
        return 0;
    }

    LOG_INFO("[Param] Saving parameters to FLASH (Bank %d)...", g_current_write_bank);

    /* 更新头部信息 */
    current_params.magic = PARAM_MAGIC;
    current_params.version = PARAM_VERSION;
    current_params.size = sizeof(ParamConfig_t);

    /* 计算CRC32（跳过CRC字段本身）*/
    uint8_t *data_ptr = (uint8_t *)&current_params;
    uint32_t data_length = sizeof(ParamConfig_t) - offsetof(ParamConfig_t, system);
    current_params.crc32 = calculate_crc32(data_ptr + offsetof(ParamConfig_t, system), data_length);

    /* V10.0: 只写入当前Bank，轮换策略减少磨损 */
    int result = param_flash_write(g_current_write_bank, &current_params);
    if (result != 0) {
        LOG_ERROR("[Param] Failed to write Bank%d", g_current_write_bank);
        /* 尝试写入另一个Bank */
        uint8_t backup_bank = (g_current_write_bank == 1) ? 2 : 1;
        result = param_flash_write(backup_bank, &current_params);
        if (result != 0) {
            LOG_ERROR("[Param] Failed to write backup Bank%d", backup_bank);
            return -1;
        }
    }

    /* 轮换到下一个Bank */
    g_current_write_bank = (g_current_write_bank == 1) ? 2 : 1;
    g_write_count++;
    g_last_save_time_ms = current_time;

    LOG_INFO("[Param] Parameters saved successfully (total saves: %lu)", g_write_count);
    return 0;
}

/**
 * @brief  恢复出厂设置
 */
int param_reset_to_default(void)
{
    LOG_WARN("[Param] Resetting to factory defaults...");

    load_default_params();

    /* 保存到FLASH */
    if (param_save() == 0) {
        LOG_INFO("[Param] Factory defaults saved to FLASH");
        return 0;
    } else {
        LOG_ERROR("[Param] Failed to save factory defaults");
        return -1;
    }
}

/* ============================================================================
 * 公共函数实现 - 参数读取
 * ============================================================================ */

/**
 * @brief  获取系统参数
 */
int param_get_system(SystemParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(param, &current_params.system, sizeof(SystemParam_t));
    return 0;
}

/**
 * @brief  获取网络参数
 */
int param_get_network(NetworkParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(param, &current_params.network, sizeof(NetworkParam_t));
    return 0;
}

/**
 * @brief  获取电机参数
 */
int param_get_motor(MotorParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(param, &current_params.motor, sizeof(MotorParam_t));
    return 0;
}

/**
 * @brief  获取传感器参数
 */
int param_get_sensor(SensorParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(param, &current_params.sensor, sizeof(SensorParam_t));
    return 0;
}

/**
 * @brief  获取完整参数配置
 */
int param_get_all(ParamConfig_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(param, &current_params, sizeof(ParamConfig_t));
    return 0;
}

/* ============================================================================
 * 公共函数实现 - 参数设置
 * ============================================================================ */

/**
 * @brief  设置系统参数
 */
int param_set_system(const SystemParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(&current_params.system, param, sizeof(SystemParam_t));
    LOG_INFO("[Param] System parameters updated (not saved yet)");
    return 0;
}

/**
 * @brief  设置网络参数
 */
int param_set_network(const NetworkParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(&current_params.network, param, sizeof(NetworkParam_t));
    LOG_INFO("[Param] Network parameters updated (not saved yet)");
    return 0;
}

/**
 * @brief  设置电机参数
 */
int param_set_motor(const MotorParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(&current_params.motor, param, sizeof(MotorParam_t));
    LOG_INFO("[Param] Motor parameters updated (not saved yet)");
    return 0;
}

/**
 * @brief  设置传感器参数
 */
int param_set_sensor(const SensorParam_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    memcpy(&current_params.sensor, param, sizeof(SensorParam_t));
    LOG_INFO("[Param] Sensor parameters updated (not saved yet)");
    return 0;
}

/**
 * @brief  设置完整参数配置
 */
int param_set_all(const ParamConfig_t *param)
{
    if (param == NULL || !param_initialized) {
        return -1;
    }

    /* 只复制数据部分，不包括头部 */
    memcpy(&current_params.system, &param->system,
           sizeof(ParamConfig_t) - offsetof(ParamConfig_t, system));

    LOG_INFO("[Param] All parameters updated (not saved yet)");
    return 0;
}

/* ============================================================================
 * 公共函数实现 - 工具函数
 * ============================================================================ */

/**
 * @brief  打印当前参数配置
 */
void param_print(void)
{
    if (!param_initialized) {
        LOG_ERROR("[Param] Not initialized");
        return;
    }

    LOG_INFO("[Param] ==================== Current Configuration ====================");

    /* 系统参数 */
    LOG_INFO("[Param] --- System Parameters ---");
    LOG_INFO("[Param]   Watchdog Timeout: %lu ms", current_params.system.watchdog_timeout_ms);
    LOG_INFO("[Param]   Log Level: %u", current_params.system.log_level);
    LOG_INFO("[Param]   Enable IWDG: %u", current_params.system.enable_iwdg);

    /* 网络参数 */
    LOG_INFO("[Param] --- Network Parameters ---");
    LOG_INFO("[Param]   IP Address: %u.%u.%u.%u",
             current_params.network.ip_addr[0], current_params.network.ip_addr[1],
             current_params.network.ip_addr[2], current_params.network.ip_addr[3]);
    LOG_INFO("[Param]   Netmask: %u.%u.%u.%u",
             current_params.network.netmask[0], current_params.network.netmask[1],
             current_params.network.netmask[2], current_params.network.netmask[3]);
    LOG_INFO("[Param]   Gateway: %u.%u.%u.%u",
             current_params.network.gateway[0], current_params.network.gateway[1],
             current_params.network.gateway[2], current_params.network.gateway[3]);
    LOG_INFO("[Param]   TCP Port: %u", current_params.network.tcp_port);
    LOG_INFO("[Param]   UDP Port: %u", current_params.network.udp_port);
    LOG_INFO("[Param]   Use DHCP: %u", current_params.network.use_dhcp);

    /* 电机参数 */
    LOG_INFO("[Param] --- Motor Parameters ---");
    LOG_INFO("[Param]   Control Frequency: %u Hz", current_params.motor.control_freq_hz);
    LOG_INFO("[Param]   Enable Motor Send: %u", current_params.motor.enable_motor_send);

    LOG_INFO("[Param] ==============================================================");
}

/**
 * @brief  检查参数是否有效
 */
bool param_is_valid(void)
{
    return validate_params(&current_params);
}

/**
 * @brief  获取参数版本号
 */
uint32_t param_get_version(void)
{
    return current_params.version;
}

/**
 * @brief  获取参数大小
 */
uint32_t param_get_size(void)
{
    return sizeof(ParamConfig_t);
}

/**
 * @brief  计算参数CRC32
 */
uint32_t param_calculate_crc(void)
{
    uint8_t *data_ptr = (uint8_t *)&current_params;
    uint32_t data_length = sizeof(ParamConfig_t) - offsetof(ParamConfig_t, system);
    return calculate_crc32(data_ptr + offsetof(ParamConfig_t, system), data_length);
}

/**
 * @brief  验证参数CRC32
 */
bool param_verify_crc(void)
{
    uint32_t calculated_crc = param_calculate_crc();
    return (calculated_crc == current_params.crc32);
}

/* ============================================================================
 * 公共函数实现 - FLASH操作
 * ============================================================================ */

/**
 * @brief  擦除参数FLASH扇区
 */
int param_flash_erase(uint8_t bank)
{
    uint32_t sector_addr;

    if (bank == 1) {
        sector_addr = PARAM_FLASH_BANK1_ADDR;
    } else if (bank == 2) {
        sector_addr = PARAM_FLASH_BANK2_ADDR;
    } else {
        LOG_ERROR("[Param] Invalid bank: %u", bank);
        return -1;
    }

    LOG_INFO("[Param] Erasing Bank%u sector (0x%08lX)...", bank, sector_addr);

    if (flash_unlock_and_erase_sector(sector_addr) != 0) {
        LOG_ERROR("[Param] Failed to erase Bank%u", bank);
        return -1;
    }

    LOG_INFO("[Param] Bank%u erased successfully", bank);
    return 0;
}

/**
 * @brief  从FLASH读取参数
 */
int param_flash_read(uint8_t bank, ParamConfig_t *param)
{
    uint32_t flash_addr;

    if (param == NULL) {
        return -1;
    }

    if (bank == 1) {
        flash_addr = PARAM_FLASH_BANK1_ADDR;
    } else if (bank == 2) {
        flash_addr = PARAM_FLASH_BANK2_ADDR;
    } else {
        LOG_ERROR("[Param] Invalid bank: %u", bank);
        return -1;
    }

    /* 直接从FLASH读取到结构体 */
    memcpy(param, (void *)flash_addr, sizeof(ParamConfig_t));

    return 0;
}

/**
 * @brief  写入参数到FLASH
 */
int param_flash_write(uint8_t bank, const ParamConfig_t *param)
{
    uint32_t flash_addr;

    if (param == NULL) {
        return -1;
    }

    if (bank == 1) {
        flash_addr = PARAM_FLASH_BANK1_ADDR;
    } else if (bank == 2) {
        flash_addr = PARAM_FLASH_BANK2_ADDR;
    } else {
        LOG_ERROR("[Param] Invalid bank: %u", bank);
        return -1;
    }

    LOG_INFO("[Param] Writing parameters to Bank%u (0x%08lX)...", bank, flash_addr);

    /* 先擦除扇区 */
    if (flash_unlock_and_erase_sector(flash_addr) != 0) {
        LOG_ERROR("[Param] Failed to erase Bank%u", bank);
        return -1;
    }

    /* 写入数据 */
    if (flash_write_data(flash_addr, (const uint8_t *)param, sizeof(ParamConfig_t)) != 0) {
        LOG_ERROR("[Param] Failed to write Bank%u", bank);
        return -1;
    }

    LOG_INFO("[Param] Bank%u written successfully", bank);
    return 0;
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  初始化CRC32查找表
 */
static void init_crc32_table(void)
{
    uint32_t polynomial = 0xEDB88320;

    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
        crc32_table[i] = crc;
    }
}

/**
 * @brief  计算CRC32
 */
static uint32_t calculate_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crc32_table[index];
    }

    return ~crc;
}

/**
 * @brief  加载默认参数
 */
static void load_default_params(void)
{
    LOG_INFO("[Param] Loading default parameters...");

    /* 清零整个结构体 */
    memset(&current_params, 0, sizeof(ParamConfig_t));

    /* 系统参数 */
    current_params.system.watchdog_timeout_ms = PARAM_DEFAULT_WATCHDOG_TIMEOUT_MS;
    current_params.system.log_level = PARAM_DEFAULT_LOG_LEVEL;
    current_params.system.enable_iwdg = PARAM_DEFAULT_ENABLE_IWDG;

    /* 网络参数 */
    uint8_t default_ip[] = PARAM_DEFAULT_IP_ADDR;
    uint8_t default_netmask[] = PARAM_DEFAULT_NETMASK;
    uint8_t default_gateway[] = PARAM_DEFAULT_GATEWAY;
    memcpy(current_params.network.ip_addr, default_ip, 4);
    memcpy(current_params.network.netmask, default_netmask, 4);
    memcpy(current_params.network.gateway, default_gateway, 4);
    current_params.network.tcp_port = PARAM_DEFAULT_TCP_PORT;
    current_params.network.udp_port = PARAM_DEFAULT_UDP_PORT;
    current_params.network.use_dhcp = PARAM_DEFAULT_USE_DHCP;

    /* 电机参数 */
    current_params.motor.control_freq_hz = PARAM_DEFAULT_CONTROL_FREQ_HZ;
    current_params.motor.enable_motor_send = PARAM_DEFAULT_ENABLE_MOTOR_SEND;

    /* 初始化电机ID映射表（默认1:1映射）*/
    for (int i = 0; i < 12; i++) {
        current_params.motor.motor_id_map[i] = i;
        current_params.motor.max_velocity[i] = PARAM_DEFAULT_MAX_VELOCITY;
        current_params.motor.max_torque[i] = PARAM_DEFAULT_MAX_TORQUE;
        current_params.motor.position_kp[i] = PARAM_DEFAULT_POSITION_KP;
        current_params.motor.position_kd[i] = PARAM_DEFAULT_POSITION_KD;
    }

    /* 传感器参数 */
    for (int i = 0; i < 4; i++) {
        current_params.sensor.force_zero[i] = PARAM_DEFAULT_FORCE_ZERO;
        current_params.sensor.force_scale[i] = PARAM_DEFAULT_FORCE_SCALE;
    }
    for (int i = 0; i < 3; i++) {
        current_params.sensor.imu_accel_offset[i] = PARAM_DEFAULT_IMU_OFFSET;
        current_params.sensor.imu_gyro_offset[i] = PARAM_DEFAULT_IMU_OFFSET;
        current_params.sensor.imu_mag_offset[i] = PARAM_DEFAULT_IMU_OFFSET;
    }

    LOG_INFO("[Param] Default parameters loaded");
}

/**
 * @brief  验证参数有效性
 */
static bool validate_params(const ParamConfig_t *params)
{
    if (params == NULL) {
        return false;
    }

    /* 检查魔数 */
    if (params->magic != PARAM_MAGIC) {
        LOG_WARN("[Param] Invalid magic: 0x%08lX (expected 0x%08lX)",
                 params->magic, PARAM_MAGIC);
        return false;
    }

    /* 检查版本（暂时只支持v1.0.0）*/
    if (params->version != PARAM_VERSION) {
        LOG_WARN("[Param] Unsupported version: 0x%08lX", params->version);
        return false;
    }

    /* 检查大小 */
    if (params->size != sizeof(ParamConfig_t)) {
        LOG_WARN("[Param] Size mismatch: %lu (expected %u)",
                 params->size, sizeof(ParamConfig_t));
        return false;
    }

    /* 验证CRC32 */
    uint8_t *data_ptr = (uint8_t *)params;
    uint32_t data_length = sizeof(ParamConfig_t) - offsetof(ParamConfig_t, system);
    uint32_t calculated_crc = calculate_crc32(data_ptr + offsetof(ParamConfig_t, system), data_length);

    if (calculated_crc != params->crc32) {
        LOG_WARN("[Param] CRC32 mismatch: 0x%08lX (expected 0x%08lX)",
                 calculated_crc, params->crc32);
        return false;
    }

    return true;
}

/**
 * @brief  解锁FLASH并擦除扇区
 */
static int flash_unlock_and_erase_sector(uint32_t sector_addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    HAL_StatusTypeDef status;

    /* 解锁FLASH */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        LOG_ERROR("[Param] Failed to unlock FLASH: %d", status);
        return -1;
    }

    /* 清除FLASH错误标志 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);

    /* 配置擦除参数（STM32H7的FLASH按Bank组织）*/
    if (sector_addr >= 0x08100000) {
        /* Bank2 */
        erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase_init.Banks = FLASH_BANK_2;
        erase_init.Sector = ((sector_addr - 0x08100000) / PARAM_FLASH_SECTOR_SIZE);
        erase_init.NbSectors = 1;
        erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V-3.6V */
    } else {
        /* Bank1 */
        erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase_init.Banks = FLASH_BANK_1;
        erase_init.Sector = ((sector_addr - 0x08000000) / PARAM_FLASH_SECTOR_SIZE);
        erase_init.NbSectors = 1;
        erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    }

    /* 执行擦除 */
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status != HAL_OK || sector_error != 0xFFFFFFFF) {
        LOG_ERROR("[Param] Erase failed: status=%d, error=0x%08lX", status, sector_error);
        HAL_FLASH_Lock();
        return -1;
    }

    /* 锁定FLASH */
    HAL_FLASH_Lock();

    return 0;
}

/**
 * @brief  写入数据到FLASH
 */
static int flash_write_data(uint32_t dest_addr, const uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status;
    uint32_t write_addr = dest_addr;
    const uint32_t *data_ptr = (const uint32_t *)data;
    uint32_t words_to_write = (length + 31) / 32;  /* 向上取整到256位（32字节）*/

    /* 解锁FLASH */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        LOG_ERROR("[Param] Failed to unlock FLASH for write: %d", status);
        return -1;
    }

    /* 清除FLASH错误标志 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);

    /* STM32H7 FLASH按256位（32字节）写入 */
    for (uint32_t i = 0; i < words_to_write; i++) {
        /* 准备32字节数据（8个字） */
        uint32_t data_to_write[8] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                       0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

        /* 复制实际数据（避免越界）*/
        uint32_t remaining_bytes = length - (i * 32);
        uint32_t bytes_to_copy = (remaining_bytes > 32) ? 32 : remaining_bytes;
        memcpy(data_to_write, data_ptr + (i * 8), bytes_to_copy);

        /* 写入256位 */
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, write_addr, (uint32_t)data_to_write);
        if (status != HAL_OK) {
            LOG_ERROR("[Param] Write failed at 0x%08lX: %d", write_addr, status);
            HAL_FLASH_Lock();
            return -1;
        }

        write_addr += 32;
    }

    /* 锁定FLASH */
    HAL_FLASH_Lock();

    return 0;
}
