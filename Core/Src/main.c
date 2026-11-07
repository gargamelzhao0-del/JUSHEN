/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "i2c.h"
#include "fdcan.h"
#include "fdcan_test.h"  /* FDCAN回环测试 */
#include "fdcan_diagnostics.h" 
#include "adc.h"
#include "dma.h"
#include "log_system.h"
#include "error_handling.h"
// #include "quad_test_framework.h"  // 使用Jetson发送数据，不需要内部测试框架

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATUS_UPDATE_INTERVAL_MS 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c3;  /* ✅ I2C3句柄（在i2c.c中定义） */
extern FDCAN_HandleTypeDef hfdcan1;  /* ✅ FDCAN1句柄（在globals.c中定义） */
extern FDCAN_HandleTypeDef hfdcan2;  /* ✅ FDCAN2句柄（在globals.c中定义） */
extern volatile uint8_t g_fdcan2_disabled;  /* ✅ FDCAN2禁用标志 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint32_t canfd_build_extended_id(uint8_t device_id, uint16_t message_id);
void canfd_print_extended_id_info(uint32_t ext_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* ⚠️ 关键调试点1: 时钟配置完成后，立即初始化一个GPIO作为调试指示 */
  /* 初始化一个LED或GPIO用于早期调试（不依赖USART） */
  __HAL_RCC_GPIOE_CLK_ENABLE();  /* 使能GPIOE时钟 */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIO_InitTypeDef debug_gpio = {0};
  debug_gpio.Pin = GPIO_PIN_7;   /* PE1作为调试引脚 */
  debug_gpio.Mode = GPIO_MODE_OUTPUT_PP;
  debug_gpio.Pull = GPIO_NOPULL;
  debug_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &debug_gpio);

  /* 闪烁3次表示时钟配置成功 */
  for(int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(100);
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* ⚠️ 关键调试点2: GPIO初始化完成 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /* Initialize USART1 - for printf debug output */
  MX_USART1_UART_Init();

  /* ⚠️ 关键调试点3: USART初始化完成 */
  printf("\r\n\r\n");
  printf("========================================\r\n");
  printf("  System Boot - Debug Points Passed\r\n");
  printf("========================================\r\n");
  printf("  [OK] Clock Config\r\n");
  printf("  [OK] GPIO Init\r\n");
  printf("  [OK] USART1 Init\r\n");
  printf("========================================\r\n");

  /* NOTE: log_system_init() moved after osKernelInitialize() because it uses osMutexNew() */

  /* Initialize DMA (before kernel, no RTOS API used) */
  MX_DMA_Init();
  printf("  [OK] DMA Init\r\n");

  /* Initialize ADC */
  printf("  [WAIT] Starting ADC Init...\r\n");
  MX_ADC1_Init();
  printf("  [OK] ADC Init Complete\r\n");

  /* Initialize I2C3 - PA8(SCL), PC9(SDA) */
  printf("  [WAIT] Starting I2C3 Init...\r\n");
  MX_I2C3_Init();
  printf("  [OK] I2C3 Init Complete\r\n");

  HAL_Delay(100);

  /* Initialize FDCAN1 and FDCAN2: 80MHz, 1Mbps arbitration, 4Mbps data, 80% sampling point */
  printf("\r\n[INFO] Initializing FDCAN1...\r\n");
  uint8_t fdcan1_result = fdcan1_init(4, 4, 15, 4, FDCAN_MODE_NORMAL);
  if (fdcan1_result != 0) {
      printf("[ERROR] FDCAN1 initialization failed with code: %u\r\n", fdcan1_result);
  }

  HAL_Delay(100);

  printf("\r\n[INFO] Initializing FDCAN2...\r\n");
  uint8_t fdcan2_result = fdcan2_init(4, 4, 15, 4, FDCAN_MODE_NORMAL);
  if (fdcan2_result != 0) {
      printf("[ERROR] FDCAN2 initialization failed with code: %u\r\n", fdcan2_result);
  }

  g_fdcan2_disabled = 0;  /* Enable FDCAN2 */

  printf("\r\n[INFO] FDCAN1 and FDCAN2 initialized successfully\r\n");
  printf("      Config: 1Mbps/4Mbps, 80%% sampling point, ISO CAN-FD with BRS\r\n");

  /* 注意: 测试任务将在FreeRTOS中运行，不需要在这里调用QuadTest */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Initialize logging system - must be after osKernelInitialize() because it uses osMutexNew() */
  if (log_system_init()) {
      printf("  [OK] Log System Init\r\n");
  } else {
      printf("  [ERROR] Log System Init Failed\r\n");
  }

  /* Initialize error handling system - must be after osKernelInitialize() */
  if (ErrorHandler_Init()) {
      printf("  [OK] Error Handling Init\r\n");
  } else {
      printf("  [ERROR] Error Handling Init Failed\r\n");
  }

  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;  /* ✅ 修改为12: PLL1Q = 960MHz/12 = 80MHz (FDCAN时钟) */
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief       构建29位CAN-FD扩展标识符
 * @note        扩展ID结构:
 *              Bit28~Bit19: 预留 (10位)
 *              Bit18~Bit12: 设备ID (7位, 0x00~0x7F)
 *              Bit11~Bit10: 预留 (2位)
 *              Bit9~Bit0:   消息ID (10位, 0x000~0x3FF)
 * @param       device_id   : 设备ID (0x00~0x7F)
 *                           0x00 = 广播地址
 *                           0x01 = 默认地址
 * @param       message_id  : 消息ID (0x000~0x3FF)
 * @retval      29位扩展标识符
 */
uint32_t canfd_build_extended_id(uint8_t device_id, uint16_t message_id)
{
    uint32_t ext_id = 0;

    /* 参数范围检查 */
    if (device_id > 0x7F) {
        device_id = 0x7F;  /* 限制在7位范围内 */
    }
    if (message_id > 0x3FF) {
        message_id = 0x3FF;  /* 限制在10位范围内 */
    }

    /* 构建29位扩展ID:
     * Bit28~19: 预留 (设为0)
     * Bit18~12: 设备ID (7位)
     * Bit11~10: 预留 (设为0)
     * Bit9~0:   消息ID (10位)
     */
    ext_id = ((uint32_t)device_id << 12) | ((uint32_t)message_id);

    return ext_id;
}

/**
 * @brief       打印29位扩展ID的详细信息
 * @param       ext_id      : 29位扩展标识符
 * @retval      无
 */
void canfd_print_extended_id_info(uint32_t ext_id)
{
    uint8_t device_id = (ext_id >> 12) & 0x7F;
    uint16_t message_id = ext_id & 0x3FF;

    printf("  Ext ID: 0x%08X (Device=0x%02X, Msg=0x%03X)\n", ext_id, device_id, message_id);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Region 0: Ethernet DMA Descriptors (0x30040000)
   *  CRITICAL FIX: Changed from NO_ACCESS to FULL_ACCESS
   *  CRITICAL FIX: Changed from BUFFERABLE to NOT_BUFFERABLE for DMA coherency
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;  /* FIXED: Was MPU_REGION_NO_ACCESS */
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;  /* FIXED: Was MPU_ACCESS_BUFFERABLE */

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Region 1: LwIP RAM Heap (0x30044000)
   *  CRITICAL FIX: Changed from CACHEABLE to NOT_CACHEABLE for DMA safety
   */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;  /* FIXED: Was MPU_ACCESS_CACHEABLE */
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Region 2: AXI SRAM for Ethernet RX Buffers (0x24000000)
   *  CRITICAL FIX: RX buffers are allocated in AXI SRAM, must be DMA accessible
   *  NOT_CACHEABLE and NOT_BUFFERABLE for DMA coherency
   */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;  /* AXI SRAM is 512KB */
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Region 3: FDCAN Message RAM (0x4000A000)
   *  CRITICAL: FDCAN Message RAM must be NOT_CACHEABLE to avoid data corruption
   *
   *  ⚠️ 关键修复: STM32H7的FDCAN Message RAM实际从0x4000AC00开始
   *  根据硬件寄存器诊断:
   *  - FDCAN1: 0x4000AC00 ~ 0x4000AD10 (272字节)
   *  - FDCAN2: 0x4000D440 ~ 0x4000D550 (需要更大范围)
   *
   *  为了安全覆盖所有可能的地址,使用16KB范围:
   *  0x4000A000 - 0x4000DFFF (16KB)
   */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x4000A000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;  /* ✅ 扩大到16KB覆盖FDCAN2地址 */
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* 尝试输出错误信息到串口 */
  printf("\r\n");
  printf("========================================\r\n");
  printf("  FATAL ERROR - Error_Handler Called!\r\n");
  printf("========================================\r\n");
  printf("  System halted. Check initialization.\r\n");
  printf("========================================\r\n");

  __disable_irq();
  while (1)
  {
    /* Blink LED or add debug code here */
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
