/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "cmsis_os.h"

/* USART1接收缓冲区 */
#define UART1_RX_BUF_SIZE 256
uint8_t uart1_rx_buffer[UART1_RX_BUF_SIZE];
uint8_t uart1_rx_data;
volatile uint16_t uart1_rx_write_pos = 0;
volatile uint16_t uart1_rx_read_pos = 0;

/* ====== V5.0: 非阻塞TX环形缓冲区 ====== */
#define UART1_TX_BUF_SIZE   2048    /* TX环形缓冲区大小 */
static uint8_t uart1_tx_buffer[UART1_TX_BUF_SIZE];
static volatile uint16_t uart1_tx_head = 0;    /* 写入位置 (fputc) */
static volatile uint16_t uart1_tx_tail = 0;    /* 发送位置 (ISR) */
static volatile uint8_t  uart1_tx_busy = 0;    /* UART TX正在发送 */
static volatile uint32_t uart1_tx_dropped = 0; /* 因缓冲区满丢弃的字节数 */
static uint8_t uart1_tx_current_byte;          /* 当前正在发送的字节(必须是全局变量) */

/* printf互斥锁 - 保护printf在多线程环境下的线程安全 */
static osMutexId_t uart_print_mutex = NULL;

/* USER CODE END 0 */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN 1 */

/**
 * @brief USART1初始化函数 - 配置波特率115200, PA9(TX), PA10(RX)
 */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 开启UART1中断接收 */
  HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
}

/**
 * @brief 初始化printf互斥锁(在FreeRTOS启动后调用)
 * @note 必须在osKernelStart()之后调用
 */
void UART1_InitMutex(void)
{
  if (uart_print_mutex == NULL) {
    uart_print_mutex = osMutexNew(NULL);
  }
}

/**
 * @brief printf重定向到USART1 (非阻塞环形缓冲区版本)
 * @note V5.0: 写入环形缓冲区后立即返回，不等待发送完成
 *       缓冲区满时丢弃数据，不会阻塞调用任务
 */
int fputc(int ch, FILE *f)
{
  /* 计算下一个写入位置 */
  uint16_t next_head = (uart1_tx_head + 1) % UART1_TX_BUF_SIZE;

  if (next_head != uart1_tx_tail) {
    /* 缓冲区有空间，写入数据 */
    uart1_tx_buffer[uart1_tx_head] = (uint8_t)ch;
    uart1_tx_head = next_head;

    /* 如果UART空闲，启动发送 */
    if (!uart1_tx_busy) {
      uart1_tx_busy = 1;
      uart1_tx_current_byte = uart1_tx_buffer[uart1_tx_tail];
      uart1_tx_tail = (uart1_tx_tail + 1) % UART1_TX_BUF_SIZE;
      HAL_UART_Transmit_IT(&huart1, &uart1_tx_current_byte, 1);
    }
  } else {
    /* 缓冲区满，丢弃数据（不阻塞） */
    uart1_tx_dropped++;
  }

  return ch;
}

/**
 * @brief UART TX完成回调 - 触发发送下一个字节
 */
static void uart1_tx_send_next(void)
{
  if (uart1_tx_tail != uart1_tx_head) {
    uart1_tx_current_byte = uart1_tx_buffer[uart1_tx_tail];
    uart1_tx_tail = (uart1_tx_tail + 1) % UART1_TX_BUF_SIZE;
    HAL_UART_Transmit_IT(&huart1, &uart1_tx_current_byte, 1);
  } else {
    uart1_tx_busy = 0;  /* 缓冲区已空 */
  }
}

/**
 * @brief scanf重定向到USART1
 */
int fgetc(FILE *f)
{
  uint8_t ch;
  HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
  return ch;
}

/**
 * @brief UART1接收中断回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 将接收到的数据存入环形缓冲区 */
    uart1_rx_buffer[uart1_rx_write_pos] = uart1_rx_data;
    uart1_rx_write_pos = (uart1_rx_write_pos + 1) % UART1_RX_BUF_SIZE;

    /* 如果缓冲区满，覆盖最旧的数据 */
    if (uart1_rx_write_pos == uart1_rx_read_pos)
    {
      uart1_rx_read_pos = (uart1_rx_read_pos + 1) % UART1_RX_BUF_SIZE;
    }

    /* 继续接收下一个字节 */
    HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
  }
}

/**
 * @brief UART TX完成回调 - 触发发送下一个字节
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_tx_send_next();
  }
}

/**
 * @brief 读取UART1接收缓冲区中的一个字节
 * @param data: 指向存储读取数据的指针
 * @return 1:成功读取, 0:缓冲区为空
 */
uint8_t UART1_ReadByte(uint8_t *data)
{
  if (uart1_rx_read_pos != uart1_rx_write_pos)
  {
    *data = uart1_rx_buffer[uart1_rx_read_pos];
    uart1_rx_read_pos = (uart1_rx_read_pos + 1) % UART1_RX_BUF_SIZE;
    return 1;
  }
  return 0;
}

/**
 * @brief 获取UART1接收缓冲区中可读字节数
 * @return 可读字节数
 */
uint16_t UART1_Available(void)
{
  if (uart1_rx_write_pos >= uart1_rx_read_pos)
  {
    return uart1_rx_write_pos - uart1_rx_read_pos;
  }
  else
  {
    return UART1_RX_BUF_SIZE - uart1_rx_read_pos + uart1_rx_write_pos;
  }
}

/**
 * @brief 清空UART1接收缓冲区
 */
void UART1_FlushRxBuffer(void)
{
  uart1_rx_read_pos = uart1_rx_write_pos;
}

/* USER CODE END 1 */

/* USER CODE BEGIN USART1_MspInit 0 */

/* USER CODE END USART1_MspInit 0 */
/**
  * @brief USART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: USART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN USART1_MspInit 1 */

    /* USART1中断配置 */
    /* 注意: FreeRTOS要求调用API的中断优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) */
    /* 设置为6确保USART1中断可以安全调用FreeRTOS API */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspInit 1 */
  }

}

/**
  * @brief USART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: USART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
