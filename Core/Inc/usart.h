/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USART1初始化函数 */
void MX_USART1_UART_Init(void);

/* 初始化printf互斥锁(必须在FreeRTOS启动后调用) */
void UART1_InitMutex(void);

/* UART1接收相关函数 */
uint8_t UART1_ReadByte(uint8_t *data);
uint16_t UART1_Available(void);
void UART1_FlushRxBuffer(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ USART_H__ */