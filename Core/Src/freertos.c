/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS应用程序代码
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_config.h"  /* V4.0: TCP配置参数 */
#if TCP_ENABLED
#include "tcp_motor_server.h"  /* TCP communication server */
#endif
#include "semphr.h"
#include "event_groups.h"
#include "log_system.h"  /* Thread-safe logging system */
#include "usart.h"       /* For UART1_InitMutex */
#include "fdcan.h"       /* FDCAN functions */
#include "adc.h"         /* ADC functions */
#include <stdio.h>
#include <string.h>
/* motor control init */
#include "motor_control.h"
#include "motor_control_enhanced.h"
/* Hardware PWM module */
#include "tim_hw.h"
/* PWM Capture Task */
#include "pwm_capture_task.h"
#include "quad_motor_api.h" 

/* ===== 新增模块 ===== */
/* 系统状态管理 */
#include "robot_state.h"
/* 电机同步控制 */
#include "motor_sync.h"
/* 足端力传感器 */
#include "foot_force_sensor.h"
/* 电源管理 */
#include "power_management.h"
/* 电机控制定时器 */
#include "motor_control_timer.h"
/* 看门狗保护系统 */
#include "watchdog.h"
/* 参数配置系统 */
#include "param_config.h"
#include "quad_emergency_stop.h"
// #include "quad_test_framework.h"  // 使用Jetson发送数据，不需要内部测试框架
#include "quad_command_fusion.h"
#include "quad_control_loop.h"
#include "power_monitor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WATCHDOG_TASK_PRIORITY         (osPriorityNormal)       /* 看门狗任务优先级 (1s cycle) */
#define TCP_SEND_TASK_PRIORITY         (osPriorityNormal)       /* TCP发送任务优先级 (10ms cycle) */
#define FDCAN_TX_TASK_PRIORITY         (osPriorityNormal)       /* FDCAN发送任务优先级 */
#define FDCAN_RX_TASK_PRIORITY         (osPriorityAboveNormal)  /* FDCAN接收任务优先级(较高) */
#define ADC_TASK_PRIORITY              (osPriorityNormal)       /* ADC采集任务优先级 */

#define WATCHDOG_INTERVAL_MS           1000                 /* 看门狗周期(毫秒)*/
#define TCP_SEND_INTERVAL_MS           10                   /* 100Hz更新频率 */
#define FDCAN_TX_INTERVAL_MS           1                    /* FDCAN发送间隔: 1ms (1000Hz) */
#define ADC_SAMPLE_INTERVAL_MS         100                  /* ADC采样间隔: 100ms (10Hz) */

/* 足端力数据发送周期: 50Hz = 20ms */
#define FOOT_FORCE_SEND_INTERVAL_MS    20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 任务句柄 */
osThreadId_t watchdogTaskHandle;          /* 看门狗任务句柄 */
osThreadId_t tcpSendTaskHandle;           /* TCP发送任务句柄 */
osThreadId_t fdcanTxTaskHandle;           /* FDCAN发送任务句柄 */
osThreadId_t fdcanRxTaskHandle;           /* FDCAN接收任务句柄 */
osThreadId_t adcTaskHandle;               /* ADC采集任务句柄 */
osThreadId_t motorControlLoopHandle;      /* 电机实时控制循环任务句柄（500Hz）*/
osThreadId_t powerMonitorTaskHandle;      /* 电源监控任务句柄 */

/* 同步原语 */
#if TCP_ENABLED
osMutexId_t tcp_mutex;                    /* TCP通信互斥锁 */
#endif
osSemaphoreId_t fdcan_rx_semaphore;       /* FDCAN接收信号量 */
EventGroupHandle_t system_events;         /* 系统事件组 */

/* 事件标志 */
#define EVENT_SYSTEM_ERROR          (1 << 3)  /* 系统错误 */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,  /* Increased from 128*4 to 512*4 for LwIP init */
  .priority = (osPriority_t) osPriorityHigh,  /* High priority for initialization */
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void WatchdogTask(void *argument);         /* 看门狗任务函数 */
#if TCP_ENABLED
void TCPSendTask(void *argument);          /* TCP发送任务函数 */
#endif
void FDCANTxTask(void *argument);          /* FDCAN发送任务函数 */
void FDCANRxTask(void *argument);          /* FDCAN接收任务函数 */
void ADCTask(void *argument);              /* ADC采集任务函数 */
void PowerMonitorTask(void *argument);     /* 电源监控任务函数 */
/* ⭐ 替换为智能控制任务 */
/* void MotorControlLoopTask(void *argument); */ /* 旧版任务已废弃 */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);     /* 默认任务函数 */

#if TCP_ENABLED
extern void MX_LWIP_Init(void);            /* 外部LwIP初始化函数声明 */
#endif
void MX_FREERTOS_Init(void);               /* FreeRTOS初始化函数 */

/* Hook prototypes */  /* 钩子函数原型 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);  /* 堆栈溢出钩子函数 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* 运行时刻栈溢出检查在configCHECK_FOR_STACK_OVERFLOW定义为1或2时执行。
      如果检测到栈溢出，将调用此钩子函数。 */
   
   /* 禁用中断，防止其他任务干扰 */
   __disable_irq();
   
   /* 尝试通过串口输出错误信息 */
   printf("\r\n");
   printf("========================================\r\n");
   printf("  [FATAL] STACK OVERFLOW DETECTED!\r\n");
   printf("========================================\r\n");
   printf("  Task: %s\r\n", pcTaskName ? (char *)pcTaskName : "UNKNOWN");
   printf("  System halted.\r\n");
   printf("========================================\r\n");
   
   /* 闪烁LED指示错误 */
   while (1) {
       HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
       for (volatile uint32_t i = 0; i < 1000000; i++) { }
   }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS初始化函数
  * @param  无参数
  * @retval 无返回值
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* Initialize UART mutex for thread-safe printf */
  UART1_InitMutex();

  /* Initialize logging system */
  if (!log_system_init()) {
    printf("[ERROR] Log system init failed!\r\n");
    Error_Handler();
  }

  printf("[INFO] FreeRTOS Initialization Started\r\n");

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 创建互斥锁 */
#if TCP_ENABLED
  const osMutexAttr_t mutex_attr = {
    .name = NULL,
    .attr_bits = osMutexRecursive | osMutexPrioInherit,
    .cb_mem = NULL,
    .cb_size = 0U
  };

  tcp_mutex = osMutexNew(&mutex_attr);

  if (tcp_mutex == NULL) {
    printf("[ERROR] Failed to create TCP mutex!\r\n");
    Error_Handler();
  }
#else
  printf("[INFO] TCP disabled - skipping TCP mutex creation\r\n");
#endif
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Create event groups */
  system_events = xEventGroupCreate();
  if (system_events == NULL) {
    printf("[ERROR] Failed to create event group!\r\n");
    Error_Handler();
  }

  /* Create FDCAN RX semaphore */
  fdcan_rx_semaphore = osSemaphoreNew(1, 0, NULL);
  if (fdcan_rx_semaphore == NULL) {
    printf("[ERROR] Failed to create FDCAN RX semaphore!\r\n");
    Error_Handler();
  }

  printf("[INFO] Synchronization primitives created\r\n");
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 启动定时器，添加新的定时器，等等 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 添加队列，等等 */
  /* USER CODE END RTOS_QUEUES */

  /* 创建线程 */
  /* 创建默认任务 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 正常模式：创建所有任务 */

  /* 创建看门狗任务 */
  const osThreadAttr_t watchdogTask_attributes = {
    .name = "Watchdog",
    .stack_size = 256 * 4,
    .priority = WATCHDOG_TASK_PRIORITY,
  };
  watchdogTaskHandle = osThreadNew(WatchdogTask, NULL, &watchdogTask_attributes);

#if TCP_ENABLED
  /* 创建TCP发送任务 */
  /* V12.1: 增大堆栈以适应LwIP协议栈需求 (1024*4→1536*4=6144字节) */
  const osThreadAttr_t tcpSendTask_attributes = {
    .name = "TCPSend",
    .stack_size = 1536 * 4,
    .priority = TCP_SEND_TASK_PRIORITY,
  };
  tcpSendTaskHandle = osThreadNew(TCPSendTask, NULL, &tcpSendTask_attributes);
#else
  printf("[INFO] TCP disabled - skipping TCP Send task creation\r\n");
#endif

  /* 创建FDCAN发送任务 */
  const osThreadAttr_t fdcanTxTask_attributes = {
    .name = "FDCANTx",
    .stack_size = 512 * 4,
    .priority = FDCAN_TX_TASK_PRIORITY,
  };
  fdcanTxTaskHandle = osThreadNew(FDCANTxTask, NULL, &fdcanTxTask_attributes);

  /* 创建FDCAN接收任务 */
  const osThreadAttr_t fdcanRxTask_attributes = {
    .name = "FDCANRx",
    .stack_size = 512 * 4,
    .priority = FDCAN_RX_TASK_PRIORITY,
  };
  fdcanRxTaskHandle = osThreadNew(FDCANRxTask, NULL, &fdcanRxTask_attributes);

  /* 创建ADC采集任务 */
  const osThreadAttr_t adcTask_attributes = {
    .name = "ADC",
    .stack_size = 512 * 4,
    .priority = ADC_TASK_PRIORITY,
  };
  adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);

  if (watchdogTaskHandle == NULL
#if TCP_ENABLED
      || tcpSendTaskHandle == NULL
#endif
      || fdcanTxTaskHandle == NULL || fdcanRxTaskHandle == NULL ||
      adcTaskHandle == NULL) {
    printf("[ERROR] Failed to create one or more tasks!\r\n");
    Error_Handler();
  }
  printf("[INFO] All tasks created successfully\r\n");

  /* 初始化 motor 子系统（队列/任务） */
  motor_control_init();

    /* ===== PWM输出模式 - PD12输出50%占空比 ===== */
    TIM_HW_Init(25000);  /* 25kHz PWM频率 */
    TIM_HW_Start();
    TIM_HW_SetDuty(2, 50);  /* PD12 - 50%占空比 */
    printf("[PWM] PD12 output enabled @ 25kHz, 50%% duty cycle\r\n");

    /* ===== PWM输入捕获模式已禁用 ===== */
    /* 如果需要PWM输入捕获，取消注释下面的代码并注释掉PWM输出模式 */
    /*
    TIM_PWM_Capture_Init();
    TIM_PWM_Capture_Start();
    LOG_INFO("[PWM Capture] PC6(TIM3_CH1) and PC7(TIM3_CH2) ready for input");
    if (PWM_Capture_Task_Start() != 0) {
        LOG_ERROR("[PWM Capture] Failed to start capture task");
    }
    */

    /* ========== 新增模块初始化 ========== */

    /* 0. 初始化参数配置系统（最先初始化，因为其他模块可能需要读取参数）*/
    if (param_init() != 0) {
        LOG_ERROR("[Main] Parameter system init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Parameter system initialized");

    /* 打印当前参数配置 */
    param_print();

    /* 1. 初始化系统状态管理器 */
    if (robot_state_init() != 0) {
        LOG_ERROR("[Main] Robot state init failed!");
        Error_Handler();
    }
    if (robot_state_start_task() != 0) {
        LOG_ERROR("[Main] Robot state task failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Robot state manager initialized");

    /* 2. 初始化电机同步控制器 */
    if (motor_sync_init(SYNC_MODE_BATCH) != 0) {
        LOG_ERROR("[Main] Motor sync init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Motor sync controller initialized (BATCH mode)");

    /* ========== 初始化智能电机控制系统 ========== */
    
    /* 1. 初始化指令融合系统 (参数缓存，减少CAN帧数) */
    extern uint8_t Fusion_Init(void);
    Fusion_Init();
    LOG_INFO("[Main] ✓ Command fusion system initialized");

    /* 2. 初始化智能控制器 (场景识别 + 策略选择) */
    extern uint8_t SmartCtrl_Init(void);
    if (SmartCtrl_Init() != 0) {
        LOG_ERROR("[Main] Smart controller init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Smart controller initialized");

    /* 3. 初始化QuadMotor API (状态管理 + 使能控制) */
    extern uint8_t QuadMotor_Init(void);
    if (QuadMotor_Init() != 0) {
        LOG_ERROR("[Main] QuadMotor init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ QuadMotor API initialized");

    /* 4. 初始化并启动控制循环 */
    extern void QuadControlLoop_Init(void);
    QuadControlLoop_Init();
    LOG_INFO("[Main] ✓ Quad control loop initialized");

    extern void QuadControlLoop_Start(void);
    QuadControlLoop_Start();
    LOG_INFO("[Main] ✓ Quad control loop started");

    /* 5. 启动电机控制系统 (使能电机 + 设置RUNNING状态) */
    extern uint8_t QuadMotor_Start(void);
    printf("[Main] Calling QuadMotor_Start...\n");
    if (QuadMotor_Start() != 0) {
        LOG_ERROR("[Main] QuadMotor start failed!");
        Error_Handler();
    }
    printf("[Main] QuadMotor_Start done!\n");
    LOG_INFO("[Main] ✓ QuadMotor started (RUNNING)");
    printf("[Main] After QuadMotor started log\n");

    /* ========== 测试任务: 模拟上位机指令 ========== */
    /* 注意: 现在使用Jetson通过TCP发送数据，不需要内部测试框架 */
    /* 
    extern void QuadTest_Init(void);
    extern void QuadTest_RunDemo(uint8_t mode);
    printf("[Main] Calling QuadTest_Init...\n");
    QuadTest_Init();
    printf("[Main] QuadTest_Init returned!\n");
    LOG_INFO("[Main] ✓ Quad test framework initialized");

    const osThreadAttr_t testTask_attributes = {
        .name = "QuadTest",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    extern void QuadTest_RTOS_Task(void *argument);
    osThreadId_t testTaskHandle = osThreadNew(QuadTest_RTOS_Task, NULL, &testTask_attributes);
    if (testTaskHandle == NULL) {
        LOG_ERROR("[Main] Quad test task creation failed!");
    } else {
        LOG_INFO("[Main] ✓ Quad test task created");
    }


    printf("[Main] QuadTest_RunDemo returned!\n");
    LOG_INFO("[Main] ✓ Quad test mode started (HYBRID)");
    */

    /* 3. 初始化电机控制硬件定时器（TIM6, 1kHz）*/
    if (motor_ctrl_timer_init() != 0) {
        LOG_ERROR("[Main] Motor control timer init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Motor control timer (TIM6) initialized");

    /* 禁用电机发送 */
    // motor_sync_set_send_enabled(false);  
    // 现在CANFD不会发送任何电机命令

    /* 启用电机发送 */
    motor_sync_set_send_enabled(true);
    // 恢复正常发送


    /* 4. 智能电机控制任务已废弃 */
    /* 注释：MotorSmartControlTask已删除，新系统使用quad_motor_api.h */
    /* 如需使用新系统，请参考 quad_integration_example.c 中的示例 */
    /*
    const osThreadAttr_t smartMotorControl_attributes = {
        .name = "SmartMotorCtrl",
        .stack_size = 2048 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    motorControlLoopHandle = osThreadNew(MotorSmartControlTask, NULL, &smartMotorControl_attributes);
    if (motorControlLoopHandle == NULL) {
        LOG_ERROR("[Main] Smart motor control task creation failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Smart motor control task created (100Hz, Command fusion)");
    */

    /* 5. 启动电机控制定时器 */
    if (motor_ctrl_timer_start() != 0) {
        LOG_ERROR("[Main] Failed to start motor control timer!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Motor control timer started");

    /* 7. 初始化看门狗保护系统 */
    if (watchdog_init() != 0) {
        LOG_ERROR("[Main] Watchdog init failed!");
        Error_Handler();
    }
    LOG_INFO("[Main] ✓ Watchdog protection system initialized");

    /* 检查是否因看门狗重启 */
    if (watchdog_check_reset_reason()) {
        LOG_ERROR("[Main] ⚠️  Last reset was caused by watchdog!");
    }

    /* 注意: IWDG 已移至 StartWatchdogTask 中启动，确保任务就绪后才开始看门狗 */
    LOG_INFO("[Main] Hardware watchdog will start in WatchdogTask after 5s delay");

    LOG_INFO("[Main] ========================================");
    LOG_INFO("[Main] 🚀 Quadruped Robot System Ready!");
    LOG_INFO("[Main] ========================================");

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 添加事件，等等 */

  /* ===== 足端力传感器初始化 ===== */
  if (foot_force_init() == 0) {
    for (int i = 0; i < 4; i++) {
      foot_force_set_scale((FootID_t)i, 100.0f);  /* 根据实际传感器修改 */
    }
    LOG_INFO("[Main] ✓ Foot force sensors initialized");
  } else {
    LOG_WARN("[Main] Foot force sensors initialization failed");
  }

  /* ===== 电源管理初始化（可选）===== */
  /* 架构：H743(I2C主机) ←→ F407(I2C从机) ←→ BMS芯片 */
  extern I2C_HandleTypeDef hi2c3;
  if (power_init(&hi2c3, 0x55) == 0) {  /* F407 I2C从机地址，需根据F407固件确认 */
    power_start_monitor_task(1000);
    LOG_INFO("[Main] ✓ Power management initialized (via F407)");
  } else {
    LOG_WARN("[Main] Power management not available");
  }

  /* ===== 新电源监控系统（I2C/SPI双模式）===== */
  static PowerMonitor_t g_power_monitor;
  extern I2C_HandleTypeDef hi2c3;
  extern SPI_HandleTypeDef hspi1;
  
  PowerMonitor_Init(&g_power_monitor, &hi2c3, NULL, NULL, 0);
  
  const osThreadAttr_t powerMonitorTask_attributes = {
    .name = "PowerMonitor",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
  };
  powerMonitorTaskHandle = osThreadNew(PowerMonitorTask, &g_power_monitor, &powerMonitorTask_attributes);
  
  if (powerMonitorTaskHandle == NULL) {
    LOG_WARN("[Main] Power monitor task creation failed");
  } else {
    LOG_INFO("[Main] ✓ Power monitor task created (100ms cycle)");
  }

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  实现默认任务的函数
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
#if TCP_ENABLED
  /* 初始化LWIP的代码 */
  printf("[INIT] Starting LwIP...\n");
  MX_LWIP_Init();
  printf("[INIT] LwIP initialized\n");

  /* TCP客户端模式：STM32作为客户端主动连接上位机服务器 */
  /* 上位机作为TCP服务器监听端口9999 */
#else
  printf("[INIT] TCP disabled - skipping LwIP initialization\n");
#endif

  /* USER CODE BEGIN StartDefaultTask */

  printf("[INIT] System ready, entering main loop\n");

  /* 无限循环 - 处理LwIP网络数据包和串口命令 */
  for(;;)
  {
    /* LwIP使用tcpip线程模式，不需要手动调用MX_LWIP_Process() */
    /* 只需保持线程运行，让LwIP的内部线程处理网络事件 */

    /* 处理串口输入命令 (用于交互式测试控制) */
    uint8_t ch;
    if (UART1_ReadByte(&ch))
    {
      switch (ch)
      {
        /* ===== 看门狗测试命令 ===== */
        case 'w':  /* 打印看门狗统计信息 */
        case 'W':
          watchdog_print_stats();
          break;

        case '1':  /* 测试：电机任务卡死10秒 */
          printf("\r\n[TEST] Triggering MotorCtrl hang (10s)...\r\n");
          watchdog_test_trigger_hang(WDG_TASK_MOTOR_CTRL, 10000);
          break;

#if TCP_ENABLED
        case '2':  /* 测试：TCP任务卡死5秒 */
          printf("\r\n[TEST] Triggering TCPSend hang (5s)...\r\n");
          watchdog_test_trigger_hang(WDG_TASK_TCP_SEND, 5000);
          break;
#endif

        case '3':  /* 测试：看门狗任务永久卡死（IWDG复位） */
          printf("\r\n[TEST] Triggering Watchdog hang (IWDG will reset in 5s)...\r\n");
          watchdog_test_trigger_hang(WDG_TASK_WATCHDOG, 0);
          break;

        case '4':  /* 测试：DefaultTask卡死5秒 */
          printf("\r\n[TEST] Triggering DefaultTask hang (5s)...\r\n");
          watchdog_test_trigger_hang(WDG_TASK_DEFAULT, 5000);
          break;

        case '5':  /* 测试：FDCANRx卡死5秒 */
          printf("\r\n[TEST] Triggering FDCANRx hang (5s)...\r\n");
          watchdog_test_trigger_hang(WDG_TASK_FDCAN_RX, 5000);
          break;

        case '0':  /* 显示测试帮助 */
          printf("\r\n========== Watchdog Test Commands ==========\r\n");
          printf("  W/w - Print watchdog statistics\r\n");
          printf("  1   - Test: Motor task hang (10s)\r\n");
#if TCP_ENABLED
          printf("  2   - Test: TCP task hang (5s)\r\n");
#endif
          printf("  3   - Test: Watchdog hang (IWDG reset)\r\n");
          printf("  4   - Test: Default task hang (5s)\r\n");
          printf("  5   - Test: FDCANRx task hang (5s)\r\n");
          printf("  0   - Show this help\r\n");
          printf("============================================\r\n");
          break;

        /* ===== 参数配置命令 ===== */
        case 'c':  /* 显示参数配置帮助 */
          printf("\r\n========== Parameter Config Commands ==========\r\n");
          printf("  c   - Show parameter config help\r\n");
          printf("  C   - Print current parameters\r\n");
          printf("  S   - Save parameters to FLASH\r\n");
          printf("  X   - Reset to factory defaults\r\n");
          printf("===============================================\r\n");
          break;

        case 'C':  /* 打印当前参数配置 */
          printf("\r\n[Param] Printing current configuration...\r\n");
          param_print();
          break;

        case 'S':  /* 保存参数到FLASH */
          printf("\r\n[Param] Saving parameters to FLASH...\r\n");
          if (param_save() == 0) {
            printf("[Param] ✓ Parameters saved successfully\r\n");
          } else {
            printf("[Param] ✗ Failed to save parameters\r\n");
          }
          break;

        case 'X':  /* 恢复出厂设置 */
          printf("\r\n[Param] ⚠️  WARNING: This will reset all parameters to factory defaults!\r\n");
          printf("[Param] Resetting to factory defaults...\r\n");
          if (param_reset_to_default() == 0) {
            printf("[Param] ✓ Factory defaults loaded and saved\r\n");
            printf("[Param] TIP: You may need to restart the system for changes to take effect\r\n");
          } else {
            printf("[Param] ✗ Failed to reset to factory defaults\r\n");
          }
          break;

        /* ===== 足端力传感器校准命令 ===== */
        case 'f':  /* 显示足端力校准帮助 */
          printf("\r\n========== Foot Force Calibration Commands ==========\r\n");
          printf("  f   - Show calibration help\r\n");
          printf("  F   - Calibrate all sensors (zero point)\r\n");
          printf("  v   - View current calibration data\r\n");
          printf("  V   - Save calibration to FLASH\r\n");
          printf("  z   - Print current force readings\r\n");
          printf("=====================================================\r\n");
          break;

        case 'F':  /* 一键校准所有传感器 */
          printf("\r\n[FootForce] Starting calibration...\r\n");
          printf("[FootForce] ⚠️  IMPORTANT: Lift all feet off the ground!\r\n");
          foot_force_calibrate_all_zero();
          break;

        case 'v':  /* 查看校准数据 */
          printf("\r\n[FootForce] Current calibration data:\r\n");
          foot_force_print_calibration();
          break;

        case 'V':  /* 保存校准数据到FLASH */
          printf("\r\n[FootForce] Saving calibration to FLASH...\r\n");
          if (foot_force_save_calibration() == 0) {
            printf("[FootForce] ✓ Calibration saved successfully\r\n");
          } else {
            printf("[FootForce] ✗ Failed to save calibration\r\n");
          }
          break;

        case 'z':  /* 打印当前力值读数 */
          printf("\r\n[FootForce] Current force readings:\r\n");
          foot_force_print_all();
          break;

        default:
          /* 忽略其他字符 */
          break;
      }
    }

    /* 看门狗心跳 */
    watchdog_feed_task(WDG_TASK_POWER_MONITOR);

    /* 看门狗测试：检查是否需要模拟卡死 */
    if (watchdog_test_should_hang(WDG_TASK_DEFAULT)) {
      uint32_t hang_duration = 5000;  /* 默认5秒 */
      printf("\r\n[DefaultTask] ⚠️  TEST: Simulating task hang for %u ms\r\n", hang_duration);
      osDelay(hang_duration);
      printf("[DefaultTask] TEST: Task resumed\r\n");
    }

    osDelay(5);  /* 5ms轮询周期，TCP发送200Hz */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

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
static uint32_t canfd_build_extended_id(uint8_t device_id, uint16_t message_id)
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
  * @brief  Simplified Watchdog Task (Motor tasks removed)
  */
/**
  * @brief  看门狗任务 - 硬件看门狗(IWDG) + 软件任务监控
  * @param  argument: 未使用
  * @retval None
  */
void WatchdogTask(void *argument)
{
  /* 调用看门狗模块的任务入口函数 */
  StartWatchdogTask(argument);
}

#if TCP_ENABLED
/**
  * @brief  TCP发送任务 - Simplified (Motor data removed)
  * @param  argument: 未使用
  * @retval None
  */
void TCPSendTask(void *argument)
{
  printf("[INFO] TCP Send Task Started (Simplified - Motor data removed)\r\n");

  /* V4.0: 初始化无外设模式 */
#if TCP_NO_PERIPHERAL_MODE
  QuadEmergencyStop_SetNoPeripheralMode(true);
  printf("[TCP] No-peripheral mode ENABLED via config\r\n");
#endif

  /* Wait for network initialization */
  printf("[TCP] Waiting 5 seconds for network stack to initialize...\r\n");

  /* 倒计时显示 */
  for (int i = 5; i > 0; i--) {
    printf("[TCP] Network init countdown: %d seconds...\r\n", i);
    osDelay(1000);
  }

  printf("[TCP] Network initialization complete!\r\n");

  /* TCP客户端模式：STM32作为客户端主动连接上位机服务器 */
  const char* remote_ip = "192.168.1.20";  /* 上位机IP地址 */
  const uint16_t remote_port = 9999;

  printf("[TCP] ========================================\r\n");
  printf("[TCP] Attempting to connect to upper computer:\r\n");
  printf("[TCP]   Target IP:   %s\r\n", remote_ip);
  printf("[TCP]   Target Port: %d\r\n", remote_port);
  printf("[TCP] ========================================\r\n");

  if (!tcp_client_connect(remote_ip, remote_port)) {
    printf("[ERROR] [TCP] Failed to initiate connection to %s:%d\r\n", remote_ip, remote_port);
    printf("[ERROR] [TCP] Please check:\r\n");
    printf("[ERROR] [TCP]   1. Ethernet cable connected?\r\n");
    printf("[ERROR] [TCP]   2. Upper computer IP correct?\r\n");
    printf("[ERROR] [TCP]   3. Upper computer TCP server running?\r\n");
  } else {
    printf("[INFO] [TCP] Connection initiated to %s:%d\r\n", remote_ip, remote_port);
  }

  uint32_t tick_count = 0;
  uint32_t foot_force_send_count = 0;
  uint32_t heartbeat_count = 0;
  uint32_t reconnect_attempts = 0;
  uint32_t last_reconnect_tick = 0;

  for(;;)
  {
    /* 检查TCP客户端连接状态 */
    bool is_connected = tcp_client_is_connected();

    /* 同步TCP连接状态到控制循环 */
    static bool last_connection_state = false;
    if (is_connected != last_connection_state) {
        last_connection_state = is_connected;
        QuadControlLoop_SetTCPConnected(is_connected);
        if (is_connected) {
            printf("[TCP] Client connected - control loop active\r\n");
        } else {
            printf("[TCP] Client disconnected\r\n");
        }
    }

    /* ============== 修复: 日志输出限频 ============== */
    static bool last_debug_printed = false;
    static uint32_t last_debug_tick = 0;
    if (!is_connected && !last_debug_printed && tick_count > 10) {
        /* 限制最多每30秒输出一次 */
        if (tick_count - last_debug_tick > 6000) {  /* 30000ms / 5ms = 6000 */
            printf("[DEBUG] [TCP] Not connected, tick=%u\r\n", tick_count);
            last_debug_tick = tick_count;
        }
        last_debug_printed = true;
    } else if (is_connected) {
        last_debug_printed = false;
        last_debug_tick = 0;  /* 连接恢复时重置 */
    }

    /* TCP客户端模式：尝试重连 */
    if (!is_connected) {
      /* 每7秒尝试重连一次 */
      if ((tick_count - last_reconnect_tick) >= 700) {  /* 7000ms / 10ms = 700 */
        reconnect_attempts++;
        last_reconnect_tick = tick_count;
        printf("[WARN] [TCP] Not connected. Reconnecting... (attempt %u)\r\n", reconnect_attempts);

        tcp_client_connect(remote_ip, remote_port);
      }
    } else {
      /* 连接成功后重置重连计数 */
      if (reconnect_attempts > 0) {
        reconnect_attempts = 0;
        printf("[INFO] [TCP] Connection established!\r\n");
      }

      /* 发送心跳消息: 每2秒一次 */
      if (tick_count % 200 == 0) {  /* 200 * 10ms = 2000ms */
        char heartbeat_msg[64];

        /* V4.0: 心跳检测优化 - 使用配置参数 */
        if (!tcp_is_heartbeat_healthy(TCP_HEARTBEAT_TIMEOUT_MS)) {
          uint32_t missed = 0;
          tcp_get_pong_status(NULL, &missed);
          printf("[WARN] [TCP] PONG heartbeat timeout! missed_count=%u\r\n", missed);

          if (missed >= TCP_HEARTBEAT_MAX_MISSED) {
            /* V4.0: 心跳丢失不触发急停，仅进入安全保持模式 */
            printf("[WARN] [TCP] Max PONG misses reached, entering safe-hold mode\r\n");

            /* 检查无外设模式 */
            if (!QuadEmergencyStop_IsNoPeripheralMode()) {
              /* 正常模式：进入安全保持，不触发急停 */
              QuadEmergencyStop_SetTCPHold(true);
            } else {
              printf("[INFO] [TCP] No-peripheral mode: keeping connection alive\r\n");
            }
            /* V4.0: 不再主动断开连接，保持TCP连接等待恢复 */
          }
        } else {
          /* 心跳正常，清除安全保持状态 */
          if (QuadEmergencyStop_IsTCPHold()) {
            QuadEmergencyStop_SetTCPHold(false);
            printf("[INFO] [TCP] Heartbeat recovered, exiting safe-hold mode\r\n");
          }
        }

        /* ============== 修复: 心跳计数器溢出保护 ============== */
        #define HEARTBEAT_MAX_COUNT 1000000  /* 最大心跳计数 */
        if (heartbeat_count >= HEARTBEAT_MAX_COUNT) {
            heartbeat_count = 0;
            printf("[INFO] [TCP] Heartbeat counter reset\r\n");
        }

        snprintf(heartbeat_msg, sizeof(heartbeat_msg),
                 "HEARTBEAT %u\n", heartbeat_count++);

        /* V3.0: 发送心跳前重置PONG状态 */
        tcp_reset_pong_state();

        if (tcp_client_send_text(heartbeat_msg)) {
          if (heartbeat_count % 10 == 0) {  /* 每20秒打印一次 */
            printf("[INFO] [TCP] Sent heartbeat #%u\r\n", heartbeat_count);
          }
        } else {
          printf("[ERROR] [TCP] Failed to send heartbeat\r\n");
        }

        /* V5.0修复: 心跳发送后等待50ms，确保tcpip线程处理完回调后再发送数据帧
         * 原因: tcp_send_buffer是共享缓冲区，同一tick内心跳和数据帧会竞争覆盖 */
        osDelay(50);
      }

      /* 发送足端力数据: 每4 tick = 20ms (50Hz) */
      if (tick_count % 4 == 0) {
        uint8_t foot_force_packet[32];
        uint16_t idx = 0;

        foot_force_packet[idx++] = 0xAA;
        foot_force_packet[idx++] = 0x55;
        foot_force_packet[idx++] = 0x02;

        float fl = foot_force_get_filtered(FOOT_FRONT_LEFT);
        float fr = foot_force_get_filtered(FOOT_FRONT_RIGHT);
        float rl = foot_force_get_filtered(FOOT_REAR_LEFT);
        float rr = foot_force_get_filtered(FOOT_REAR_RIGHT);

        memcpy(&foot_force_packet[idx], &fl, 4); idx += 4;
        memcpy(&foot_force_packet[idx], &fr, 4); idx += 4;
        memcpy(&foot_force_packet[idx], &rl, 4); idx += 4;
        memcpy(&foot_force_packet[idx], &rr, 4); idx += 4;

        uint32_t timestamp = tick_count;
        memcpy(&foot_force_packet[idx], &timestamp, 4); idx += 4;

        uint8_t checksum = 0;
        for (uint16_t i = 0; i < idx; i++) {
          checksum += foot_force_packet[i];
        }
        foot_force_packet[idx++] = checksum;

        tcp_client_send_data(foot_force_packet, idx);

        foot_force_send_count++;
        if (foot_force_send_count % 100 == 0) {
          LOG_RAW("[INFO] Foot force sent: %u frames\r\n", foot_force_send_count);
        }
        if (foot_force_send_count == 1) {
          LOG_RAW("[INFO] [TCP] First foot force packet sent!\r\n");
        }
      }

      /* 发送电机反馈数据: 每4 tick = 20ms (50Hz)，与足端力错开2 tick */
      if ((tick_count + 2) % 4 == 0) {
#if CLOSED_LOOP_ENABLED
        /* 记录发送时间 */
        uint32_t send_time = HAL_GetTick();
        uint32_t feedback_received = 0;

        /* 闭环模式：发送0x31F监测指令到所有电机，获取实时反馈数据 */
        /* 注意：控制循环(200Hz)已以更高频率发送监测请求，此处为冗余补充 */
        motor_request_all_monitor_data();

        /* 等待CAN响应，最多等待5ms或所有电机都返回数据 */
        while ((HAL_GetTick() - send_time) < 5) {
          feedback_received = 0;
          for (uint8_t i = 0; i < 12; i++) {
            MotorHandle* handle = motor_handle_get(i);
            if (handle != NULL && handle->initialized) {
              if ((HAL_GetTick() - handle->last_feedback_time) < 10) {
                feedback_received++;
              }
            }
          }
          if (feedback_received >= 12) {
            break;
          }
          osDelay(1);
        }
#endif /* CLOSED_LOOP_ENABLED */

        /* 读取电机反馈数据并打包发送 */
        /* V3.0: 增加每个电机的独立时间戳，数据包大小计算:
         * - 帧头: 3字节 (0xAA, 0x55, 0x03)
         * - 每个电机: 4 + 4 + 4 + 4 = 16字节 (pos, vel, torque, motor_ts)
         * - 12个电机: 16 * 12 = 192字节
         * - 全局时间戳: 4字节
         * - 校验和: 1字节
         * - 总计: 3 + 192 + 4 + 1 = 200字节
         */
        uint8_t motor_feedback_packet[200];
        uint16_t idx = 0;

        motor_feedback_packet[idx++] = 0xAA;
        motor_feedback_packet[idx++] = 0x55;
        motor_feedback_packet[idx++] = 0x03;

        for (uint8_t i = 0; i < 12; i++) {
          MotorHandle* handle = motor_handle_get(i);
          if (handle != NULL && handle->initialized) {
            memcpy(&motor_feedback_packet[idx], &handle->monitor.actual_position, 4); idx += 4;
            memcpy(&motor_feedback_packet[idx], &handle->monitor.actual_speed, 4); idx += 4;
            memcpy(&motor_feedback_packet[idx], &handle->monitor.actual_torque, 4); idx += 4;
            /* V3.0: 添加每个电机的独立时间戳 */
            uint32_t motor_ts = handle->last_feedback_time;
            memcpy(&motor_feedback_packet[idx], &motor_ts, 4); idx += 4;
          } else {
            float zero = 0.0f;
            uint32_t zero_ts = 0;
            memcpy(&motor_feedback_packet[idx], &zero, 4); idx += 4;
            memcpy(&motor_feedback_packet[idx], &zero, 4); idx += 4;
            memcpy(&motor_feedback_packet[idx], &zero, 4); idx += 4;
            memcpy(&motor_feedback_packet[idx], &zero_ts, 4); idx += 4;
          }
        }

        uint32_t timestamp = tick_count;
        memcpy(&motor_feedback_packet[idx], &timestamp, 4); idx += 4;

        uint8_t checksum = 0;
        for (uint16_t i = 0; i < idx; i++) {
          checksum += motor_feedback_packet[i];
        }
        motor_feedback_packet[idx++] = checksum;

        if (tcp_client_send_data(motor_feedback_packet, idx)) {
          static uint32_t motor_feedback_count = 0;
          motor_feedback_count++;
          if (motor_feedback_count == 1) {
            LOG_RAW("[INFO] [TCP] First motor feedback packet sent! (len=%u)\r\n", idx);
          }
          if (motor_feedback_count % 100 == 0) {
            LOG_RAW("[INFO] Motor feedback sent: %u frames\r\n", motor_feedback_count);
          }
        }

        /* V9.1: 发送增强的电机状态，降低频率到25Hz避免缓冲区溢出 */
        if (tick_count % 8 == 0) {
          tcp_motor_server_send_all_motor_status();
        }
      }  /* 闭合 if ((tick_count + 2) % 4 == 0) */
    }  /* 闭合 if (is_connected) */

    /* V3.0: LED闪烁指示上位机数据接收状态 */
    /* 闪烁周期: 0.5秒 (250ms亮/250ms灭) */
    /* 超时时间: 1秒内收到数据则认为正在发送 */
    #define LED_BLINK_INTERVAL_MS    250    /* LED切换间隔 (ms) */
    #define UPPER_COMPUTER_TIMEOUT_MS 1000  /* 上位机数据超时时间 (ms) */
    
    static uint32_t last_led_toggle_tick = 0;
    static bool led_state = false;
    
    if (tcp_is_upper_computer_sending(UPPER_COMPUTER_TIMEOUT_MS)) {
      /* 上位机正在发送数据，LED闪烁 */
      uint32_t current_tick = HAL_GetTick();
      if (current_tick - last_led_toggle_tick >= LED_BLINK_INTERVAL_MS) {
        led_state = !led_state;
        if (led_state) {
          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        last_led_toggle_tick = current_tick;
      }
    } else {
      /* 上位机停止发送数据，LED熄灭 */
      if (led_state) {
        led_state = false;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
      }
    }

    /* 看门狗心跳 */
    watchdog_feed_task(WDG_TASK_TCP_SEND);

    /* 看门狗测试：检查是否需要模拟卡死 */
    if (watchdog_test_should_hang(WDG_TASK_TCP_SEND)) {
      uint32_t hang_duration = 5000;  /* 默认5秒 */
      printf("\r\n[TCPSend] ⚠️  TEST: Simulating task hang for %u ms\r\n", hang_duration);
      osDelay(hang_duration);
      printf("[TCPSend] TEST: Task resumed\r\n");
    }

    tick_count++;
    osDelay(TCP_SEND_INTERVAL_MS);
  }
}
#endif /* TCP_ENABLED */

/**
  * @brief  FDCAN发送测试任务
  * @note   使用29位扩展ID，1kHz高频发送测试
  * @param  argument: 未使用
  * @retval None
  */
void FDCANTxTask(void *argument)
{
  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  uint32_t cnt = 0;

  /* 缓冲区声明为静态全局变量，32字节对齐 */
  __attribute__((aligned(32))) static uint8_t fdcan1_txdata[64];
  __attribute__((aligned(32))) static uint8_t fdcan2_txdata[64];

  LOG_INFO("[FDCAN-TX] Task Started - Sending test data every 1 second");

  /* 配置FDCAN发送参数 */
  FDCAN_TxConfigTypeDef fdcan1_config;
  FDCAN_TxConfigTypeDef fdcan2_config;

  /* FDCAN1配置: Device ID=0x01 (默认地址), Message ID=0x100 */
  uint32_t fdcan1_ext_id = canfd_build_extended_id(0x01, 0x100);
  fdcan1_config.identifier = fdcan1_ext_id;
  fdcan1_config.id_type = FDCAN_EXTENDED_ID;
  fdcan1_config.frame_type = FDCAN_DATA_FRAME;
  fdcan1_config.fd_format = FDCAN_FD_CAN;
  fdcan1_config.bit_rate_switch = FDCAN_BRS_ON;
  fdcan1_config.data_length = FDCAN_DLC_BYTES_8;

  /* FDCAN2配置: Device ID=0x02 (自定义地址), Message ID=0x200 */
  uint32_t fdcan2_ext_id = canfd_build_extended_id(0x02, 0x200);
  fdcan2_config.identifier = fdcan2_ext_id;
  fdcan2_config.id_type = FDCAN_EXTENDED_ID;
  fdcan2_config.frame_type = FDCAN_DATA_FRAME;
  fdcan2_config.fd_format = FDCAN_FD_CAN;
  fdcan2_config.bit_rate_switch = FDCAN_BRS_ON;
  fdcan2_config.data_length = FDCAN_DLC_BYTES_8;

  LOG_INFO("[FDCAN1] Ext ID: 0x%08X (Device=0x%02X, Msg=0x%03X)",
           fdcan1_ext_id, (uint8_t)((fdcan1_ext_id >> 12) & 0x7F), (uint16_t)(fdcan1_ext_id & 0x3FF));
  LOG_INFO("[FDCAN2] Ext ID: 0x%08X (Device=0x%02X, Msg=0x%03X)",
           fdcan2_ext_id, (uint8_t)((fdcan2_ext_id >> 12) & 0x7F), (uint16_t)(fdcan2_ext_id & 0x3FF));

  /* 等待系统初始化 */
  osDelay(2000);

  for(;;)
  {
    /* 准备发送数据 - 8字节测试数据 */
    for (uint8_t i = 0; i < 8; i++) {
      fdcan1_txdata[i] = 0xA0 + (cnt % 256) + i;
      fdcan2_txdata[i] = 0xB0 + (cnt % 256) + i;
    }

    /* FDCAN1 发送 */
    // LOG_INFO("[FDCAN1-TX] #%u Data: %02X %02X %02X %02X %02X %02X %02X %02X",
    //          cnt, fdcan1_txdata[0], fdcan1_txdata[1], fdcan1_txdata[2], fdcan1_txdata[3],
    //          fdcan1_txdata[4], fdcan1_txdata[5], fdcan1_txdata[6], fdcan1_txdata[7]);

    // if (fdcan1_send_msg_ex(fdcan1_txdata, &fdcan1_config) != 0) {
    //   LOG_ERROR("[FDCAN1-TX] Send failed!");
    // }

    // /* FDCAN2 发送 */
    // LOG_INFO("[FDCAN2-TX] #%u Data: %02X %02X %02X %02X %02X %02X %02X %02X",
    //          cnt, fdcan2_txdata[0], fdcan2_txdata[1], fdcan2_txdata[2], fdcan2_txdata[3],
    //          fdcan2_txdata[4], fdcan2_txdata[5], fdcan2_txdata[6], fdcan2_txdata[7]);

    // if (fdcan2_send_msg_ex(fdcan2_txdata, &fdcan2_config) != 0) {
    //   LOG_ERROR("[FDCAN2-TX] Send failed!");
    // }

    cnt++;
    osDelay(1000);  /* 每秒发送一次 */
  }
}

/**
  * @brief  FDCAN接收任务 - 处理CANFD分析仪发送的数据
  * @note   通过中断信号量触发，接收并打印来自分析仪的数据
  */
void FDCANRxTask(void *argument)
{
  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern void motor_process_rx_message(uint32_t rx_id, const uint8_t *rxdata, uint8_t data_len);  /* 电机消息处理函数 */
  uint8_t rxdata[64];
  uint8_t rxlen;
  FDCAN_RxHeaderTypeDef RxHeader;
  uint32_t last_diagnostic_tick = 0;  /* 上次诊断时间 */
  uint32_t rx_count = 0;               /* 接收计数 */

  /* DLC到字节数转换表（统一定义，避免重复） */
  const uint8_t dlc_to_bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

  LOG_INFO("[FDCAN-RX] Receive Task Started - Waiting for motor responses");
  LOG_INFO("[FDCAN-RX] Both FDCAN1 and FDCAN2 are listening");

  for(;;)
  {
    /* 等待接收中断信号量（带超时，避免无限阻塞） */
    if (osSemaphoreAcquire(fdcan_rx_semaphore, pdMS_TO_TICKS(100)) == osOK)
    {
      uint32_t current_tick = HAL_GetTick();

      /* V4.0: 监测请求已移至控制循环中以200Hz统一发送，此处不再重复发送 */

      /* ==================== 检查FDCAN1 ==================== */
      uint32_t fifo1_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
      if (fifo1_level > 0)
      {
        rx_count++;  /* 接收计数 */

        /* 失效D-Cache确保读取最新数据 */
        uint32_t msg_ram_start = (uint32_t)hfdcan1.msgRam.StandardFilterSA & ~0x1FU;
        uint32_t msg_ram_size = ((hfdcan1.msgRam.EndAddress - hfdcan1.msgRam.StandardFilterSA) + 31) & ~31;
        SCB_InvalidateDCache_by_Addr((uint32_t *)msg_ram_start, msg_ram_size);
        __DSB();
        __ISB();

        /* 读取接收数据 */
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxdata) == HAL_OK)
        {
          /* 转换DLC到字节数 */
          uint32_t dlc_code = RxHeader.DataLength;
          rxlen = (dlc_code <= 15) ? dlc_to_bytes[dlc_code] : 0;

          /* 打印接收数据 */
          LOG_INFO("[FDCAN1-RX] ID:0x%08X Len:%d Data:", RxHeader.Identifier, rxlen);
          printf("  ");
          for (uint8_t i = 0; i < rxlen; i++) {
            printf("%02X ", rxdata[i]);
          }
          printf("\r\n");

          /* ✅ 调用电机消息处理函数，解析电机返回的执行结果 */
          motor_process_rx_message(RxHeader.Identifier, rxdata, rxlen);

        }
      }

      /* ==================== 检查FDCAN2 ==================== */
      uint32_t fifo2_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
      if (fifo2_level > 0)
      {
        /* 失效D-Cache确保读取最新数据 */
        uint32_t msg_ram_start = (uint32_t)hfdcan2.msgRam.StandardFilterSA & ~0x1FU;
        uint32_t msg_ram_size = ((hfdcan2.msgRam.EndAddress - hfdcan2.msgRam.StandardFilterSA) + 31) & ~31;
        SCB_InvalidateDCache_by_Addr((uint32_t *)msg_ram_start, msg_ram_size);
        __DSB();
        __ISB();

        /* 读取接收数据 */
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, rxdata) == HAL_OK)
        {
          /* 转换DLC到字节数 */
          uint32_t dlc_code = RxHeader.DataLength;
          rxlen = (dlc_code <= 15) ? dlc_to_bytes[dlc_code] : 0;

          /* 打印接收数据 */
          LOG_INFO("[FDCAN2-RX] ID:0x%08X Len:%d Data:", RxHeader.Identifier, rxlen);
          printf("  ");
          for (uint8_t i = 0; i < rxlen; i++) {
            printf("%02X ", rxdata[i]);
          }
          printf("\r\n");

          /* ✅ 调用电机消息处理函数，解析电机返回的执行结果 */
          motor_process_rx_message(RxHeader.Identifier, rxdata, rxlen);

        }
      }
    }

    /* 每10秒打印一次诊断信息（无论是否收到数据） */
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_diagnostic_tick >= 10000) {  /* 10秒 */
      last_diagnostic_tick = current_tick;

      /* 读取FDCAN1状态 */
      uint32_t psr1 = hfdcan1.Instance->PSR;
      uint32_t ecr1 = hfdcan1.Instance->ECR;
      uint32_t rxf0s1 = hfdcan1.Instance->RXF0S;
      uint32_t ir1 = hfdcan1.Instance->IR;

      printf("\r\n[FDCAN1 Diagnostic] (RX Count: %u)\r\n", rx_count);
      printf("  PSR: 0x%08X (LEC=%u, EP=%u, BO=%u)\r\n",
             psr1, (psr1 >> 0) & 0x7, (psr1 >> 5) & 0x1, (psr1 >> 7) & 0x1);
      printf("  ECR: 0x%08X (TEC=%u, REC=%u)\r\n",
             ecr1, (ecr1 >> 0) & 0xFF, (ecr1 >> 8) & 0x7F);
      printf("  RXF0S: 0x%08X (FillLevel=%u, GetIndex=%u)\r\n",
             rxf0s1, rxf0s1 & 0x7F, (rxf0s1 >> 8) & 0x3F);
      printf("  IR: 0x%08X\r\n", ir1);

      /* 读取FDCAN2状态 */
      uint32_t psr2 = hfdcan2.Instance->PSR;
      uint32_t ecr2 = hfdcan2.Instance->ECR;
      uint32_t rxf0s2 = hfdcan2.Instance->RXF0S;

      printf("[FDCAN2 Diagnostic]\r\n");
      printf("  PSR: 0x%08X (LEC=%u, EP=%u, BO=%u)\r\n",
             psr2, (psr2 >> 0) & 0x7, (psr2 >> 5) & 0x1, (psr2 >> 7) & 0x1);
      printf("  ECR: 0x%08X (TEC=%u, REC=%u)\r\n",
             ecr2, (ecr2 >> 0) & 0xFF, (ecr2 >> 8) & 0x7F);
      printf("  RXF0S: 0x%08X (FillLevel=%u)\r\n",
             rxf0s2, rxf0s2 & 0x7F);

      if (rx_count > 0) {
        printf("  Status: Receiving data normally\r\n\r\n");
      } else {
        printf("  Status: Waiting for CAN frames from analyzer...\r\n\r\n");
      }

      rx_count = 0;  /* 重置接收计数 */
    }

    /* 看门狗心跳 */
    watchdog_feed_task(WDG_TASK_FDCAN_RX);

    /* 看门狗测试：检查是否需要模拟卡死 */
    if (watchdog_test_should_hang(WDG_TASK_FDCAN_RX)) {
      uint32_t hang_duration = 5000;  /* 默认5秒 */
      LOG_WARN("[FDCANRx] ⚠️  TEST: Simulating task hang for %lu ms", hang_duration);
      osDelay(hang_duration);
      LOG_INFO("[FDCANRx] TEST: Task resumed");
    }
  }
}

/**
  * @brief  ADC采集任务 - 周期性读取4路ADC的原始值和电压值
  * @param  argument: 未使用
  * @retval None
  * @note   ADC配置：PA3, PA6, PB0, PB1 (4个通道)
  *         采样周期：100ms (10Hz)
  *         使用DMA自动采样，任务只负责读取和显示
  */
void ADCTask(void *argument)
{
  uint32_t loop_count = 0;

  printf("\r\n");
  printf("========================================\r\n");
  printf("  ADC Sampling Task Started\r\n");
  printf("========================================\r\n");
  printf("  Channels: 4 (PA3, PA6, PB0, PB1)\r\n");
  printf("  Resolution: 16-bit (0-65535)\r\n");
  printf("  Sample Rate: 10Hz (100ms interval)\r\n");
  printf("  Mode: Continuous DMA Circular\r\n");
  printf("  Waiting 2 seconds before sampling...\r\n");
  printf("========================================\r\n\r\n");

  /* 等待系统初始化 */
  osDelay(2000);

  /* ⚠️ 检查ADC是否初始化 */
  extern ADC_HandleTypeDef hadc1;
  printf("[ADC] Checking hadc1.Instance... (value=%p)\r\n", hadc1.Instance);
  if (hadc1.Instance == NULL) {
    printf("[ADC] ERROR: ADC not initialized! Task suspended.\r\n");
    for(;;) {
      osDelay(10000);  /* 永久休眠 */
    }
  }

  printf("[ADC] ADC sampling task ready, starting loop...\r\n");

  for(;;)
  {
    loop_count++;

    /* 读取4路ADC原始值 */
    uint16_t raw_pa3 = ADC_GetValue(ADC_CH_PA3);
    uint16_t raw_pa6 = ADC_GetValue(ADC_CH_PA6);
    uint16_t raw_pb0 = ADC_GetValue(ADC_CH_PB0);
    uint16_t raw_pb1 = ADC_GetValue(ADC_CH_PB1);

    /* ===== 更新足端力传感器 ===== */
    foot_force_update(FOOT_FRONT_LEFT,  raw_pa3);
    foot_force_update(FOOT_FRONT_RIGHT, raw_pa6);
    foot_force_update(FOOT_REAR_LEFT,   raw_pb0);
    foot_force_update(FOOT_REAR_RIGHT,  raw_pb1);

    /* 读取4路ADC电压值 */
    float volt_pa3 = ADC_GetVoltage(ADC_CH_PA3);
    float volt_pa6 = ADC_GetVoltage(ADC_CH_PA6);
    float volt_pb0 = ADC_GetVoltage(ADC_CH_PB0);
    float volt_pb1 = ADC_GetVoltage(ADC_CH_PB1);

    /* 每10次循环（约1秒）打印一次，便于调试 */
    if (loop_count % 10 == 0) {
      printf("[ADC] #%lu: PA3=%u(%.3fV) PA6=%u(%.3fV) PB0=%u(%.3fV) PB1=%u(%.3fV)\r\n",
             loop_count, raw_pa3, volt_pa3, raw_pa6, volt_pa6,
             raw_pb0, volt_pb0, raw_pb1, volt_pb1);
    }

    /* 每1000次（100秒）打印一次统计信息 */
    if (loop_count % 1000 == 0) {
      LOG_INFO("[ADC] Statistics: count=%u, heap=%u bytes",
               loop_count, (unsigned int)xPortGetFreeHeapSize());
    }

    /* 检测电压异常（示例：电压超出正常范围） */
    if (volt_pa3 > 3.35f || volt_pa6 > 3.35f ||
        volt_pb0 > 3.35f || volt_pb1 > 3.35f) {
      LOG_WARN("[ADC] Voltage out of range detected!");
    }

    /* 等待下一个采样周期 */
    osDelay(ADC_SAMPLE_INTERVAL_MS);
  }
}

/**
  * @brief  电机实时控制循环任务（改进版 - 使用硬件定时器触发）
  * @param  argument: 未使用
  * @retval 无返回值
  * @note   任务功能：
  *         - 500Hz更新频率（2ms周期）- 与控制循环匹配
  *         - 由TIM6硬件定时器触发（信号量）
  *         - 从轨迹插值器获取平滑位置
  *         - 同步发送12个电机命令
  *         - 实现稳定控制，不影响其他任务
  *         优先级：Normal（与TCP相同，保证实时性，不饿死其他任务）
  */
/**
 * @brief  旧版电机控制循环任务（已废弃，被MotorSmartControlTask替代）
 * @note   ⭐ 此任务已被 motor_smart_task.c 中的智能控制任务替换
 *         新任务使用指令融合策略，无需高频发送位置指令
 */
#if 0  /* 禁用旧版任务 */
void MotorControlLoopTask(void *argument)
{
  uint32_t loop_count = 0;
  osSemaphoreId_t timer_sem;        /* 定时器信号量 */

  LOG_INFO("[MotorCtrl] ========================================");
  LOG_INFO("[MotorCtrl] Motor Control Loop Task Started");
  LOG_INFO("[MotorCtrl] ========================================");
  LOG_INFO("[MotorCtrl] Update Rate: 100 Hz (10ms monitoring)");
  LOG_INFO("[MotorCtrl] Priority: Normal (hardware timer triggered)");
  LOG_INFO("[MotorCtrl] Test: Method B - Driver-level smooth control");
  LOG_INFO("[MotorCtrl] ========================================");

  /* 等待系统初始化完成 */
  osDelay(3000);

  /* ===== 方案B：电机驱动器级别的平滑控制测试 ===== */
  LOG_INFO("[MotorCtrl] ========================================");
  LOG_INFO("[MotorCtrl] Method B: Driver-Level Smooth Control Test");
  LOG_INFO("[MotorCtrl] Target Motor: ID=1 (CANFD1, Device 0x01)");
  LOG_INFO("[MotorCtrl] Motion: 0° -> 90° with driver smoothing");
  LOG_INFO("[MotorCtrl] Strategy: Set limits + Send once");
  LOG_INFO("[MotorCtrl] ========================================");

  /* 步骤1: 使能电机 */
  LOG_INFO("[MotorCtrl] Step 1: Enable motor...");
  if (motor_enable(1, 0x01, 1) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to enable motor!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Motor ID=1 enabled");
  }
  osDelay(100);

  /* 步骤2: 确保电机处于 POSITION 模式 */
  LOG_INFO("[MotorCtrl] Step 2: Set motor to POSITION mode...");
  if (motor_set_mode(1, 0x01, MOTOR_MODE_POSITION) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to set POSITION mode!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Motor ID=1 set to POSITION mode");
  }
  osDelay(50);  /* 等待模式切换完成 */

  /* 步骤3: 设置速度限制 */
  /* 计算：90度转动需要2秒
   * 平均角速度 = 90° / 2s = 45°/s
   * 转换为 rpm: 45°/s × (60s/min) / (360°/rev) = 7.5 rpm
   * 设置为 10 rpm 留一些余量
   */
  float limit_speed_rpm = 10.0f;  /* 10 rpm 限速 */
  LOG_INFO("[MotorCtrl] Step 3: Set speed limit to %.2f rpm...", limit_speed_rpm);
  if (motor_set_limit_speed(1, 0x01, limit_speed_rpm) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to set speed limit!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Speed limit set: %.2f rpm (≈ 45°/s)", limit_speed_rpm);
  }
  osDelay(50);

  /* 步骤4: 设置加速度限制 */
  /* 计算：梯形速度曲线，假设加速时间0.5秒
   * 加速度 = 45°/s / 0.5s = 90°/s²
   * 转换为 rpm/s: 90°/s² × (60s/min) / (360°/rev) = 15 rpm/s
   * 设置为 20 rpm/s
   */
  float limit_accel_rpm_s = 20.0f;  /* 20 rpm/s 加速度限制 */
  LOG_INFO("[MotorCtrl] Step 4: Set acceleration limit to %.2f rpm/s...", limit_accel_rpm_s);
  if (motor_set_limit_accel(1, 0x01, limit_accel_rpm_s) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to set acceleration limit!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Acceleration limit set: %.2f rpm/s (≈ 90°/s²)", limit_accel_rpm_s);
  }
  osDelay(50);

  /* 步骤5: 启动电机 */
  LOG_INFO("[MotorCtrl] Step 5: Start motor...");
  if (motor_start_stop(1, 0x01, 1) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to start motor!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Motor ID=1 started");
  }
  osDelay(100);

  /* 步骤6: 发送目标位置（只发送一次！）*/
  LOG_INFO("[MotorCtrl] Step 6: Send target position (90°) - ONCE ONLY!");
  if (motor_set_position(1, 0x01, 90.0f) != 0) {
    LOG_ERROR("[MotorCtrl] Failed to send target position!");
  } else {
    LOG_INFO("[MotorCtrl] ✓ Target position sent: 90.0° (driver will smooth the motion)");
  }

  LOG_INFO("[MotorCtrl] ========================================");
  LOG_INFO("[MotorCtrl] ⭐ Position command sent!");
  LOG_INFO("[MotorCtrl] Expected: Motor smoothly moves from 0° to 90°");
  LOG_INFO("[MotorCtrl] Duration: ~2 seconds (controlled by driver)");
  LOG_INFO("[MotorCtrl] CANFD Analyzer: Should see ONLY ONE position command!");
  LOG_INFO("[MotorCtrl] ========================================");

  /* 获取定时器信号量句柄（用于后续监控）*/
  timer_sem = motor_ctrl_timer_get_semaphore();
  if (timer_sem == NULL) {
    LOG_ERROR("[MotorCtrl] Failed to get timer semaphore!");
    Error_Handler();
  }

  LOG_INFO("[MotorCtrl] Entering monitoring loop (100Hz)...");

  /* 频率监控变量 */
  uint32_t freq_check_start_tick = HAL_GetTick();
  uint32_t freq_check_loop_count = 0;

  /* 实时监控循环（降低到100Hz以减少日志）*/
  for(;;)
  {
    /* 等待定时器信号量（1ms周期触发）*/
    if (osSemaphoreAcquire(timer_sem, osWaitForever) != osOK) {
      LOG_ERROR("[MotorCtrl] Failed to acquire timer semaphore!");
      continue;
    }

    loop_count++;
    freq_check_loop_count++;

    /* 每10ms采样一次（100Hz监控）*/
    if (loop_count % 10 != 0) {
      continue;
    }

    /* 频率监控：每10秒计算实际频率 */
    uint32_t elapsed_ms = HAL_GetTick() - freq_check_start_tick;
    if (elapsed_ms >= 10000) {  /* 10秒 */
      float actual_freq = (float)freq_check_loop_count * 1000.0f / (float)elapsed_ms;
      LOG_INFO("[MotorCtrl] ⚡ Loop frequency: %.2f Hz (expected: 1000 Hz)", actual_freq);
      freq_check_start_tick = HAL_GetTick();
      freq_check_loop_count = 0;
    }

    /* 简单监控 - 方案B不需要持续发送位置命令 */
    /* 电机驱动器会自动处理平滑运动 */

    /* 每1秒打印一次状态（仅用于确认循环正常运行）*/
    if (loop_count % 1000 == 0) {  /* 100Hz × 10 = 每1秒 */
      LOG_INFO("[Monitor] Loop running (count=%lu, motor is moving autonomously)", loop_count / 10);
    }

    /* 系统状态保持为READY */
    if (robot_state_get() != ROBOT_STATE_READY) {
      robot_state_request_transition(ROBOT_STATE_READY);
    }

    /* 主动让步，确保低优先级任务有机会运行 */
    osThreadYield();

    /* 看门狗心跳 */
    watchdog_feed_task(WDG_TASK_MOTOR_CTRL);

    /* 看门狗测试：检查是否需要模拟卡死 */
    if (watchdog_test_should_hang(WDG_TASK_MOTOR_CTRL)) {
      uint32_t hang_duration = 10000;  /* 默认10秒 */
      LOG_WARN("[MotorCtrl] ⚠️  TEST: Simulating task hang for %lu ms", hang_duration);
      osDelay(hang_duration);
      LOG_INFO("[MotorCtrl] TEST: Task resumed");
    }
  }
}
#endif  /* 禁用旧版任务结束 */

/**
  * @brief  电源监控任务 - 通过I2C/SPI与STM32F4通信获取电源状态
  * @param  argument: PowerMonitor_t指针
  * @retval None
  */
void PowerMonitorTask(void *argument)
{
  PowerMonitor_t *pm = (PowerMonitor_t *)argument;
  
  if (pm == NULL) {
    LOG_ERROR("[PowerMonitor] Invalid argument!");
    return;
  }
  
  LOG_INFO("[PowerMonitor] Task started, waiting for F4 slave...");
  osDelay(1000);
  
  static uint32_t last_soc_print_tick = 0;
  
  for (;;) {
    PowerMonitor_Update(pm);
    
    uint32_t current_tick = HAL_GetTick();
    
    if (PowerMonitor_HasEvent(pm)) {
      PowerEvent_t event;
      if (PowerMonitor_GetEvent(pm, &event)) {
        LOG_INFO("[PowerMonitor] Event: %s, data=%d", 
                 PowerMonitor_GetEventString(event.event_type), 
                 event.event_data);
        
        if (event.event_type == EVENT_TYPE_SOC_DROP_10) {
          printf("\r\n[PowerMonitor] ⚠️  SOC dropped to milestone: %d%%\r\n", event.event_data);
        } else if (event.event_type == EVENT_TYPE_SOC_CRITICAL) {
          printf("\r\n[PowerMonitor] 🔴 CRITICAL: Battery SOC = %d%%\r\n", event.event_data);
        } else if (event.event_type == EVENT_TYPE_SOC_LOW) {
          printf("\r\n[PowerMonitor] ⚠️  LOW BATTERY: SOC = %d%%\r\n", event.event_data);
        } else if (event.event_type >= EVENT_TYPE_FAULT_LEVEL1 && event.event_type <= EVENT_TYPE_FAULT_LEVEL3) {
          printf("\r\n[PowerMonitor] ⚠️  FAULT Level %d: code=%d\r\n", 
                 event.fault_level, event.fault_code);
        }
      }
    }
    
    if (current_tick - last_soc_print_tick >= 5000) {
      last_soc_print_tick = current_tick;
      
      if (PowerMonitor_IsF4Online(pm)) {
        printf("[PowerMonitor] SOC=%d%%, SOH=%d%%, V=%dmV, I=%dmA, State=%s\r\n",
               PowerMonitor_GetSOC(pm),
               PowerMonitor_GetSOH(pm),
               PowerMonitor_GetVoltage(pm),
               PowerMonitor_GetCurrent(pm),
               PowerMonitor_GetStateString(PowerMonitor_GetSystemState(pm)));
      } else {
        printf("[PowerMonitor] F4 slave offline, retrying...\r\n");
      }
    }
    
    watchdog_feed_task(WDG_TASK_POWER_MONITOR);
    
    osDelay(POWER_MONITOR_INTERVAL_MS);
  }
}

/* USER CODE END Application */
