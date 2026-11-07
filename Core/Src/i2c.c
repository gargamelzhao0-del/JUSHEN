/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "log_system.h"

/* 全局变量 */
I2C_HandleTypeDef hi2c3;

static F407PowerData_t g_f407_power_data = {0};
static bool g_f407_connected = false;

/* 函数原型 */
static float convert_voltage(uint16_t raw);
static float convert_current(uint16_t raw);
static float convert_temperature(uint16_t raw);

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief I2C3初始化函数 - 配置I2C3, PA8(SCL), PC9(SDA)
  * @retval None
  */
void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C0EAFF;  /* 100kHz I2C时钟，根据时钟配置调整 */
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

  LOG_INFO("[I2C3] Initialized - PA8(SCL), PC9(SDA), 100kHz\r\n");
}

/* USER CODE END 1 */

/* USER CODE BEGIN I2C3_MspInit 0 */

/* USER CODE END I2C3_MspInit 0 */
/**
  * @brief I2C MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PC9     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }

}

/**
  * @brief I2C MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PC9     ------> I2C3_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 2 */

/* ============================================================================ 
 * I2C通信API实现
 * ============================================================================ */

/**
 * @brief  初始化I2C通信
 */
int i2c_init(void)
{
    /* 初始化电源数据 */
    g_f407_power_data.power_status = 0;
    g_f407_power_data.battery_voltage = 0.0f;
    g_f407_power_data.battery_current = 0.0f;
    g_f407_power_data.battery_percentage = 0;
    g_f407_power_data.temperature = 0.0f;
    g_f407_power_data.f407_status = 0;
    g_f407_power_data.last_update_ms = 0;
    g_f407_power_data.is_connected = false;
    g_f407_connected = false;

    LOG_INFO("[I2C] Initialized for F407 communication\r\n");

    return 0;
}

/**
 * @brief  读取F407寄存器
 */
int i2c_read_f407_register(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }

    /* 发送寄存器地址 */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, F407_I2C_ADDRESS << 1, &reg_addr, 1, I2C_TIMEOUT_SHORT_MS);
    if (status != HAL_OK) {
        LOG_ERROR("[I2C] Failed to transmit register address: %d\r\n", status);
        g_f407_connected = false;
        return -1;
    }

    /* 读取数据 */
    status = HAL_I2C_Master_Receive(&hi2c3, F407_I2C_ADDRESS << 1, data, len, I2C_TIMEOUT_SHORT_MS);
    if (status != HAL_OK) {
        LOG_ERROR("[I2C] Failed to receive data: %d\r\n", status);
        g_f407_connected = false;
        return -1;
    }

    g_f407_connected = true;
    return 0;
}

/**
 * @brief  写入数据到F407
 */
int i2c_write_f407(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }

    /* 准备发送数据（寄存器地址 + 数据）*/
    uint8_t tx_data[len + 1];
    tx_data[0] = reg_addr;
    memcpy(&tx_data[1], data, len);

    /* 发送数据 */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, F407_I2C_ADDRESS << 1, tx_data, len + 1, I2C_TIMEOUT_LONG_MS);
    if (status != HAL_OK) {
        LOG_ERROR("[I2C] Failed to write data: %d\r\n", status);
        g_f407_connected = false;
        return -1;
    }

    g_f407_connected = true;
    return 0;
}

/**
 * @brief  读取F407电源管理数据
 */
int i2c_read_f407_power(F407PowerData_t *data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t buffer[9];

    /* 读取所有电源数据 */
    if (i2c_read_f407_register(REG_POWER_STATUS, buffer, 9) != 0) {
        data->is_connected = false;
        return -1;
    }

    /* 解析数据 */
    data->power_status = buffer[0];
    uint16_t raw_voltage = (buffer[1] << 8) | buffer[2];
    uint16_t raw_current = (buffer[3] << 8) | buffer[4];
    data->battery_percentage = buffer[5];
    uint16_t raw_temp = (buffer[6] << 8) | buffer[7];
    data->f407_status = buffer[8];

    /* 转换为实际值 */
    data->battery_voltage = convert_voltage(raw_voltage);
    data->battery_current = convert_current(raw_current);
    data->temperature = convert_temperature(raw_temp);

    /* 更新时间戳 */
    data->last_update_ms = HAL_GetTick();
    data->is_connected = true;

    /* 更新缓存 */
    memcpy(&g_f407_power_data, data, sizeof(F407PowerData_t));
    g_f407_connected = true;

    return 0;
}

/**
 * @brief  检查F407连接状态
 */
bool i2c_is_f407_connected(void)
{
    return g_f407_connected;
}

/**
 * @brief  获取F407电源数据（带缓存）
 */
int i2c_get_f407_power_data(F407PowerData_t *data)
{
    if (data == NULL) {
        return -1;
    }

    /* 检查是否需要更新数据（超过100ms）*/
    uint32_t current_time = HAL_GetTick();
    if (current_time - g_f407_power_data.last_update_ms > 100) {
        /* 更新数据 */
        if (i2c_read_f407_power(data) != 0) {
            return -1;
        }
    } else {
        /* 使用缓存数据 */
        memcpy(data, &g_f407_power_data, sizeof(F407PowerData_t));
    }

    return 0;
}

/* ============================================================================ 
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  转换电压值
 */
static float convert_voltage(uint16_t raw)
{
    /* 假设12位ADC，3.3V参考电压，分压电路1:10 */
    return (float)raw * 3.3f * 10.0f / 4095.0f;
}

/**
 * @brief  转换电流值
 */
static float convert_current(uint16_t raw)
{
    /* 假设使用ACS712电流传感器，灵敏度185mV/A */
    float voltage = (float)raw * 3.3f / 4095.0f;
    return (voltage - 1.65f) / 0.185f;
}

/**
 * @brief  转换温度值
 */
static float convert_temperature(uint16_t raw)
{
    /* 假设使用LM35温度传感器，10mV/℃ */
    float voltage = (float)raw * 3.3f / 4095.0f;
    return voltage * 100.0f;
}

/* USER CODE END 2 */
