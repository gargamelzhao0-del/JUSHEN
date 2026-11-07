/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* I2C设备地址 */
#define F407_I2C_ADDRESS          0x68  /* F407的I2C地址（7位）*/

/* F407寄存器定义 */
#define REG_POWER_STATUS          0x00  /* 电源状态 */
#define REG_BATTERY_VOLTAGE       0x01  /* 电池电压（2字节）*/
#define REG_BATTERY_CURRENT       0x03  /* 电池电流（2字节）*/
#define REG_BATTERY_PERCENTAGE    0x05  /* 电池百分比（1字节）*/
#define REG_TEMPERATURE           0x06  /* 温度（2字节）*/
#define REG_F407_STATUS           0x08  /* F407状态 */

/* 电源状态位定义 */
#define POWER_BIT_OK           0x01
#define POWER_BIT_LOW_VOLTAGE  0x02
#define POWER_BIT_OVER_TEMP    0x04
#define POWER_BIT_OVER_CURRENT 0x08

#define I2C_TIMEOUT_SHORT_MS   50    /* 短超时: 50ms (正常通信) */
#define I2C_TIMEOUT_LONG_MS    200   /* 长超时: 200ms (首次通信/慢设备) */

/* USER CODE END Private defines */

/* ============================================================================ 
 * 电源管理数据结构
 * ============================================================================ */

/**
 * @brief F407电源管理数据
 */
typedef struct {
    uint8_t power_status;        /* 电源状态 */
    float battery_voltage;       /* 电池电压（V）*/
    float battery_current;       /* 电池电流（A）*/
    uint8_t battery_percentage;  /* 电池百分比（0-100）*/
    float temperature;           /* 温度（℃）*/
    uint8_t f407_status;         /* F407状态 */
    uint32_t last_update_ms;     /* 最后更新时间 */
    bool is_connected;           /* 是否连接 */
} F407PowerData_t;

/* USER CODE BEGIN Prototypes */

/* ============================================================================
 * I2C初始化函数
 * ============================================================================ */

/**
 * @brief  I2C3硬件初始化函数
 * @note   配置I2C3: PA8(SCL), PC9(SDA), 100kHz
 * @retval None
 */
void MX_I2C3_Init(void);

/* ============================================================================
 * I2C通信API
 * ============================================================================ */

/**
 * @brief  初始化I2C通信
 * @retval 0=成功, -1=失败
 */
int i2c_init(void);

/**
 * @brief  读取F407电源管理数据
 * @param  data: 输出数据结构体
 * @retval 0=成功, -1=失败
 */
int i2c_read_f407_power(F407PowerData_t *data);

/**
 * @brief  写入数据到F407
 * @param  reg_addr: 寄存器地址
 * @param  data: 数据
 * @param  len: 数据长度
 * @retval 0=成功, -1=失败
 */
int i2c_write_f407(uint8_t reg_addr, uint8_t *data, uint8_t len);

/**
 * @brief  读取F407寄存器
 * @param  reg_addr: 寄存器地址
 * @param  data: 输出数据
 * @param  len: 数据长度
 * @retval 0=成功, -1=失败
 */
int i2c_read_f407_register(uint8_t reg_addr, uint8_t *data, uint8_t len);

/**
 * @brief  检查F407连接状态
 * @retval true=连接, false=断开
 */
bool i2c_is_f407_connected(void);

/**
 * @brief  获取F407电源数据（带缓存）
 * @param  data: 输出数据结构体
 * @retval 0=成功, -1=失败
 */
int i2c_get_f407_power_data(F407PowerData_t *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ I2C_H__ */