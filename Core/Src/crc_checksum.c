/**
 ******************************************************************************
 * @file    crc_checksum.c
 * @brief   CRC16/CRC32校验模块实现
 ******************************************************************************
 */

#include "crc_checksum.h"
#include "log_system.h"
#include <string.h>

/* CRC硬件支持（如果可用）*/
#ifdef HAL_CRC_MODULE_ENABLED
#include "stm32h7xx_hal_crc.h"
#endif

/* ============================================================================
 * CRC16查找表（优化性能）
 * ============================================================================ */

/* CRC-16/MODBUS查找表 (多项式: 0x8005) */
static const uint16_t crc16_modbus_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* ============================================================================
 * CRC16实现
 * ============================================================================ */

/**
 * @brief  计算CRC-16/MODBUS
 */
uint16_t crc16_modbus(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0xFFFF;
    }

    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++) {
        uint8_t index = (uint8_t)(crc ^ data[i]);
        crc = (crc >> 8) ^ crc16_modbus_table[index];
    }

    return crc;
}

/**
 * @brief  计算CRC-16/CCITT (简化实现)
 */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0xFFFF;
    }

    uint16_t crc = 0xFFFF;
    const uint16_t polynomial = 0x1021;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

/**
 * @brief  验证CRC16数据包
 */
bool crc16_verify(const uint8_t *data, uint32_t length, CRC_Type_t crc_type)
{
    if (data == NULL || length < 3) {  /* 至少1字节数据 + 2字节CRC */
        return false;
    }

    /* 提取数据部分和CRC部分 */
    uint32_t data_length = length - 2;
    uint16_t received_crc = ((uint16_t)data[length - 2] << 8) | data[length - 1];

    /* 计算CRC */
    uint16_t calculated_crc = 0;
    if (crc_type == CRC_TYPE_CRC16_MODBUS) {
        calculated_crc = crc16_modbus(data, data_length);
    } else if (crc_type == CRC_TYPE_CRC16_CCITT) {
        calculated_crc = crc16_ccitt(data, data_length);
    } else {
        return false;
    }

    /* 比较 */
    if (received_crc == calculated_crc) {
        return true;
    } else {
        LOG_WARN("[CRC16] Verification failed: rx=0x%04X, calc=0x%04X",
                 received_crc, calculated_crc);
        return false;
    }
}

/* ============================================================================
 * CRC32实现（硬件加速）
 * ============================================================================ */

#ifdef HAL_CRC_MODULE_ENABLED
/* CRC硬件句柄（需要在main.c中初始化）*/
extern CRC_HandleTypeDef hcrc;
#endif

/**
 * @brief  初始化CRC硬件
 */
int crc32_hw_init(void)
{
    /* CRC硬件通常在main.c的MX_CRC_Init()中初始化 */
    /* 这里只是检查是否已初始化 */

    /* 注意：如果main.c中没有定义hcrc，此函数会返回错误 */
    /* 这不影响CRC16软件校验的正常使用 */
#ifdef HAL_CRC_MODULE_ENABLED
    if (hcrc.Instance == NULL) {
        LOG_WARN("[CRC32] CRC hardware not initialized, using CRC16 instead");
        return -1;
    }

    LOG_INFO("[CRC32] CRC hardware ready");
    return 0;
#else
    LOG_WARN("[CRC32] CRC hardware not available, using CRC16 instead");
    return -1;
#endif
}

/**
 * @brief  计算CRC32（硬件加速）
 */
uint32_t crc32_hw_calculate(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0xFFFFFFFF;
    }

#ifdef HAL_CRC_MODULE_ENABLED
    /* 使用HAL库的CRC硬件加速 */
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, length / 4);

    /* 处理剩余字节（非4字节对齐） */
    uint32_t remaining = length % 4;
    if (remaining > 0) {
        uint32_t last_word = 0;
        memcpy(&last_word, data + (length - remaining), remaining);
        crc = HAL_CRC_Accumulate(&hcrc, &last_word, 1);
    }

    return crc;
#else
    /* CRC硬件不可用，返回错误值 */
    LOG_WARN("[CRC32] CRC hardware not available");
    return 0xFFFFFFFF;
#endif
}

/**
 * @brief  验证CRC32数据包
 */
bool crc32_hw_verify(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length < 5) {  /* 至少1字节数据 + 4字节CRC */
        return false;
    }

    /* 提取数据部分和CRC部分（小端序）*/
    uint32_t data_length = length - 4;
    uint32_t received_crc = ((uint32_t)data[length - 4] <<  0) |
                           ((uint32_t)data[length - 3] <<  8) |
                           ((uint32_t)data[length - 2] << 16) |
                           ((uint32_t)data[length - 1] << 24);

    /* 计算CRC */
    uint32_t calculated_crc = crc32_hw_calculate(data, data_length);

    /* 比较 */
    if (received_crc == calculated_crc) {
        return true;
    } else {
        LOG_WARN("[CRC32] Verification failed: rx=0x%08lX, calc=0x%08lX",
                 received_crc, calculated_crc);
        return false;
    }
}

/* ============================================================================
 * 便捷函数
 * ============================================================================ */

/**
 * @brief  添加CRC16到数据包末尾
 */
uint32_t crc16_append(uint8_t *data, uint32_t data_length, CRC_Type_t crc_type)
{
    if (data == NULL || data_length == 0) {
        return 0;
    }

    /* 计算CRC */
    uint16_t crc = 0;
    if (crc_type == CRC_TYPE_CRC16_MODBUS) {
        crc = crc16_modbus(data, data_length);
    } else if (crc_type == CRC_TYPE_CRC16_CCITT) {
        crc = crc16_ccitt(data, data_length);
    } else {
        return data_length;
    }

    /* 添加CRC到末尾（大端序）*/
    data[data_length]     = (crc >> 8) & 0xFF;  /* 高字节 */
    data[data_length + 1] = (crc >> 0) & 0xFF;  /* 低字节 */

    return data_length + 2;
}

/**
 * @brief  添加CRC32到数据包末尾
 */
uint32_t crc32_hw_append(uint8_t *data, uint32_t data_length)
{
    if (data == NULL || data_length == 0) {
        return 0;
    }

    /* 计算CRC */
    uint32_t crc = crc32_hw_calculate(data, data_length);

    /* 添加CRC到末尾（小端序）*/
    data[data_length]     = (crc >>  0) & 0xFF;
    data[data_length + 1] = (crc >>  8) & 0xFF;
    data[data_length + 2] = (crc >> 16) & 0xFF;
    data[data_length + 3] = (crc >> 24) & 0xFF;

    return data_length + 4;
}

/**
 * @brief  打印CRC值
 */
void crc_print(uint32_t crc_value, CRC_Type_t crc_type)
{
    if (crc_type == CRC_TYPE_CRC32) {
        LOG_INFO("[CRC] CRC32 = 0x%08lX", crc_value);
    } else {
        LOG_INFO("[CRC] CRC16 = 0x%04X", (uint16_t)crc_value);
    }
}
