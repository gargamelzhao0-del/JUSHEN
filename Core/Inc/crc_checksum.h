/**
 ******************************************************************************
 * @file    crc_checksum.h
 * @brief   CRC16/CRC32校验模块 - 利用STM32H7硬件加速
 * @note    用于TCP通信数据完整性校验
 ******************************************************************************
 */

#ifndef __CRC_CHECKSUM_H
#define __CRC_CHECKSUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * CRC配置
 * ============================================================================ */

/**
 * @brief CRC算法类型
 */
typedef enum {
    CRC_TYPE_CRC16_MODBUS,   /* CRC-16/MODBUS (最常用) */
    CRC_TYPE_CRC16_CCITT,    /* CRC-16/CCITT */
    CRC_TYPE_CRC32,          /* CRC-32 (硬件加速) */
} CRC_Type_t;

/* ============================================================================
 * 公共API - CRC16
 * ============================================================================ */

/**
 * @brief  计算CRC-16/MODBUS校验和（软件实现）
 * @param  data: 数据缓冲区
 * @param  length: 数据长度（字节）
 * @retval CRC16值
 * @note   多项式: 0x8005, 初值: 0xFFFF, 结果异或: 0x0000
 *         应用: Modbus协议、工业控制
 */
uint16_t crc16_modbus(const uint8_t *data, uint32_t length);

/**
 * @brief  计算CRC16校验和（通用接口，默认使用MODBUS算法）
 * @param  data: 数据缓冲区
 * @param  length: 数据长度（字节）
 * @retval CRC16值
 * @note   这是crc16_modbus的别名，用于向后兼容
 */
static inline uint16_t crc16_calculate(const uint8_t *data, uint32_t length) {
    return crc16_modbus(data, length);
}

/**
 * @brief  计算CRC-16/CCITT校验和（软件实现）
 * @param  data: 数据缓冲区
 * @param  length: 数据长度（字节）
 * @retval CRC16值
 * @note   多项式: 0x1021, 初值: 0xFFFF, 结果异或: 0x0000
 *         应用: X.25、HDLC、PPP
 */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t length);

/**
 * @brief  验证CRC-16数据包完整性
 * @param  data: 完整数据包（包含2字节CRC）
 * @param  length: 数据包总长度
 * @param  crc_type: CRC类型
 * @retval true=校验通过, false=校验失败
 * @note   CRC16假设为数据包最后2个字节（大端序）
 */
bool crc16_verify(const uint8_t *data, uint32_t length, CRC_Type_t crc_type);

/* ============================================================================
 * 公共API - CRC32（硬件加速）
 * ============================================================================ */

/**
 * @brief  初始化CRC硬件模块
 * @retval 0=成功, -1=失败
 * @note   使用STM32H7硬件CRC加速器
 */
int crc32_hw_init(void);

/**
 * @brief  计算CRC-32校验和（硬件加速）
 * @param  data: 数据缓冲区（必须4字节对齐）
 * @param  length: 数据长度（字节）
 * @retval CRC32值
 * @note   使用STM32硬件CRC单元，比软件快10-100倍
 *         多项式: 0x04C11DB7 (CRC-32/IEEE 802.3)
 */
uint32_t crc32_hw_calculate(const uint8_t *data, uint32_t length);

/**
 * @brief  验证CRC-32数据包完整性（硬件加速）
 * @param  data: 完整数据包（包含4字节CRC）
 * @param  length: 数据包总长度
 * @retval true=校验通过, false=校验失败
 * @note   CRC32假设为数据包最后4个字节（小端序）
 */
bool crc32_hw_verify(const uint8_t *data, uint32_t length);

/* ============================================================================
 * 便捷函数
 * ============================================================================ */

/**
 * @brief  添加CRC16到数据包末尾
 * @param  data: 数据缓冲区（需要预留2字节空间）
 * @param  data_length: 数据长度（不含CRC）
 * @param  crc_type: CRC类型
 * @retval 总长度（data_length + 2）
 */
uint32_t crc16_append(uint8_t *data, uint32_t data_length, CRC_Type_t crc_type);

/**
 * @brief  添加CRC32到数据包末尾（硬件加速）
 * @param  data: 数据缓冲区（需要预留4字节空间）
 * @param  data_length: 数据长度（不含CRC）
 * @retval 总长度（data_length + 4）
 */
uint32_t crc32_hw_append(uint8_t *data, uint32_t data_length);

/**
 * @brief  打印CRC值（调试用）
 * @param  crc_value: CRC值
 * @param  crc_type: CRC类型
 */
void crc_print(uint32_t crc_value, CRC_Type_t crc_type);

#ifdef __cplusplus
}
#endif

#endif /* __CRC_CHECKSUM_H */
