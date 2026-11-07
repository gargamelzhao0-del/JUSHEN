/**
 ******************************************************************************
 * @file    quad_emergency_stop.c
 * @brief   四足机器狗紧急停止系统实现
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "quad_emergency_stop.h"
#include "quad_motor_api.h"
#include "motor_sync.h"
#include "motor_control.h"
#include "robot_state.h"
#include "log_system.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

static volatile EmergencyStopSource_t g_estop_source = ESTOP_SOURCE_NONE;
static volatile EmergencyStopLevel_t g_estop_level = ESTOP_LEVEL_HARD;
static volatile bool g_estop_active = false;
static volatile uint32_t g_estop_timestamp = 0;
static volatile uint32_t g_estop_trigger_tick = 0;  /* V11.0: 急停触发时间戳 */
static volatile uint8_t g_stopped_motors[QUAD_MAX_MOTORS] = {0};

static GPIO_TypeDef* g_estop_gpio_port = NULL;
static uint16_t g_estop_gpio_pin = 0;

static osMutexId_t g_estop_mutex = NULL;

void QuadEmergencyStop_Init(void)
{
    LOG_INFO("[ESTOP] Initializing emergency stop system...");

    osMutexAttr_t mutex_attr = {
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    g_estop_mutex = osMutexNew(&mutex_attr);

    g_estop_active = false;
    g_estop_source = ESTOP_SOURCE_NONE;
    g_estop_level = ESTOP_LEVEL_HARD;
    g_estop_timestamp = 0;

    for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
        g_stopped_motors[i] = 0;
    }

    LOG_INFO("[ESTOP] Emergency stop system initialized");
}

void QuadEmergencyStop_Trigger(EmergencyStopSource_t source, uint32_t level)
{
    osMutexAcquire(g_estop_mutex, osWaitForever);

    if (!g_estop_active) {
        g_estop_active = true;
        g_estop_source = source;
        g_estop_level = level;
        g_estop_timestamp = osKernelGetTickCount() * portTICK_PERIOD_MS;

        const char* source_name = "UNKNOWN";
        switch (source) {
            case ESTOP_SOURCE_GPIO:      source_name = "GPIO"; break;
            case ESTOP_SOURCE_SOFTWARE:   source_name = "SOFTWARE"; break;
            case ESTOP_SOURCE_WATCHDOG:   source_name = "WATCHDOG"; break;
            case ESTOP_SOURCE_TCP:        source_name = "TCP"; break;
            case ESTOP_SOURCE_OVERCURRENT:source_name = "OVERCURRENT"; break;
            case ESTOP_SOURCE_OVERTEMP:   source_name = "OVERTEMP"; break;
            default: break;
        }

        const char* level_name = "UNKNOWN";
        switch (level) {
            case ESTOP_LEVEL_SOFT:      level_name = "SOFT"; break;
            case ESTOP_LEVEL_HARD:      level_name = "HARD"; break;
            case ESTOP_LEVEL_POWER_CUT: level_name = "POWER_CUT"; break;
            default: break;
        }

        LOG_ERROR("[ESTOP] EMERGENCY STOP TRIGGERED! Source=%s, Level=%s",
                  source_name, level_name);

        switch (level) {
            case ESTOP_LEVEL_SOFT:
                for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
                    g_stopped_motors[i] = 1;
                }
                break;

            case ESTOP_LEVEL_HARD:
                motor_sync_emergency_stop_all(NULL);
                for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
                    g_stopped_motors[i] = 1;
                }
                break;

            case ESTOP_LEVEL_POWER_CUT:
                for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
                    motor_enable(i / 6 + 1, i + 1, 0);
                    g_stopped_motors[i] = 1;
                }
                break;
        }

        robot_state_emergency_stop_ex(level);
    }

    osMutexRelease(g_estop_mutex);
}

void QuadEmergencyStop_Clear(void)
{
    osMutexAcquire(g_estop_mutex, osWaitForever);

    if (g_estop_active) {
        g_estop_active = false;
        g_estop_source = ESTOP_SOURCE_NONE;

        LOG_INFO("[ESTOP] Emergency stop cleared, system resuming...");

        robot_state_resume_from_emergency();
    }

    osMutexRelease(g_estop_mutex);
}

bool QuadEmergencyStop_IsActive(void)
{
    return g_estop_active;
}

EmergencyStopSource_t QuadEmergencyStop_GetSource(void)
{
    return g_estop_source;
}

void QuadEmergencyStop_GetStatus(EmergencyStopStatus_t *status)
{
    if (status == NULL) return;

    osMutexAcquire(g_estop_mutex, osWaitForever);
    status->source = g_estop_source;
    status->level = g_estop_level;
    status->timestamp_ms = g_estop_timestamp;
    status->is_active = g_estop_active;
    for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
        status->stopped_motors[i] = g_stopped_motors[i];
    }
    osMutexRelease(g_estop_mutex);
}

void QuadEmergencyStop_PrintStatus(void)
{
    EmergencyStopStatus_t status;
    QuadEmergencyStop_GetStatus(&status);

    printf("\n========== Emergency Stop Status ==========\n");
    printf("Active:    %s\n", status.is_active ? "YES" : "NO");

    if (status.is_active) {
        const char* source_name = "UNKNOWN";
        switch (status.source) {
            case ESTOP_SOURCE_GPIO:      source_name = "GPIO Button"; break;
            case ESTOP_SOURCE_SOFTWARE:   source_name = "Software"; break;
            case ESTOP_SOURCE_WATCHDOG:   source_name = "Watchdog"; break;
            case ESTOP_SOURCE_TCP:        source_name = "TCP上位机"; break;
            case ESTOP_SOURCE_OVERCURRENT:source_name = "Overcurrent"; break;
            case ESTOP_SOURCE_OVERTEMP:   source_name = "Overtemperature"; break;
            default: break;
        }

        const char* level_name = "UNKNOWN";
        switch (status.level) {
            case ESTOP_LEVEL_SOFT:      level_name = "Soft Stop (平滑停止)"; break;
            case ESTOP_LEVEL_HARD:      level_name = "Hard Stop (立即停止)"; break;
            case ESTOP_LEVEL_POWER_CUT: level_name = "Power Cut (断电)"; break;
            default: break;
        }

        printf("Source:    %s\n", source_name);
        printf("Level:     %s\n", level_name);
        printf("Timestamp: %u ms\n", (unsigned int)status.timestamp_ms);
        printf("Stopped Motors: ");
        for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
            printf("%d", status.stopped_motors[i]);
            if (i < QUAD_MAX_MOTORS - 1) printf(", ");
        }
        printf("\n");
    }

    printf("============================================\n\n");
}

void QuadEmergencyStop_SetGPIOPin(GPIO_TypeDef* port, uint16_t pin)
{
    g_estop_gpio_port = port;
    g_estop_gpio_pin = pin;
}

void QuadEmergencyStop_EnableGPIOInterrupt(bool enable)
{
    if (g_estop_gpio_port == NULL || g_estop_gpio_pin == 0) {
        LOG_WARN("[ESTOP] GPIO pin not configured");
        return;
    }

    if (enable) {
        LOG_INFO("[ESTOP] GPIO emergency stop enabled");
    } else {
        LOG_INFO("[ESTOP] GPIO emergency stop disabled");
    }
}

/* ISR安全的急停触发标志 — 由ISR设置，由周期性任务检查并执行实际急停 */
static volatile bool g_estop_gpio_pending = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == g_estop_gpio_pin) {
        /* ISR上下文：绝对不能调用mutex/RTOS阻塞API
         * 仅设置标志位，由QuadEmergencyStop_Poll()在任务上下文中处理 */
        g_estop_gpio_pending = true;
    }
}

void QuadEmergencyStop_Poll(void)
{
    if (g_estop_gpio_pending) {
        g_estop_gpio_pending = false;
        QuadEmergencyStop_Trigger(ESTOP_SOURCE_GPIO, 1);
    }
}

/* ============== V4.0: 无外设模式支持 ============== */
static volatile bool g_no_peripheral_mode = false;

void QuadEmergencyStop_SetNoPeripheralMode(bool enabled)
{
    g_no_peripheral_mode = enabled;
    if (enabled) {
        LOG_INFO("[ESTOP] No-peripheral mode ENABLED");
        LOG_INFO("[ESTOP] TCP timeout will NOT trigger emergency stop");
    } else {
        LOG_INFO("[ESTOP] No-peripheral mode DISABLED");
    }
}

bool QuadEmergencyStop_IsNoPeripheralMode(void)
{
    return g_no_peripheral_mode;
}

/* ============== 修复: TCP超时状态管理函数 ============== */
static volatile bool g_tcp_timeout_active = false;
static volatile bool g_tcp_hold_active = false;
static volatile bool g_tcp_safe_stop_active = false;

void QuadEmergencyStop_SetTCPTimeout(bool active)
{
    g_tcp_timeout_active = active;
    if (active && !g_estop_active) {
        if (g_no_peripheral_mode) {
            LOG_DEBUG("[ESTOP] TCP timeout warning (ignored in no-peripheral mode)");
        } else {
            LOG_WARN("[ESTOP] TCP timeout warning activated");
        }
    }
}

void QuadEmergencyStop_SetTCPHold(bool active)
{
    g_tcp_hold_active = active;
    if (active) {
        if (g_no_peripheral_mode) {
            LOG_DEBUG("[ESTOP] TCP hold (ignored in no-peripheral mode)");
        } else {
            LOG_WARN("[ESTOP] TCP hold position mode activated");
        }
    }
}

void QuadEmergencyStop_SetTCPSafeStop(bool active)
{
    bool was_active = g_tcp_safe_stop_active;
    g_tcp_safe_stop_active = active;
    
    if (active && !was_active) {
        if (g_no_peripheral_mode) {
            LOG_WARN("[ESTOP] TCP safe stop IGNORED in no-peripheral mode");
            LOG_INFO("[ESTOP] Connection maintained, waiting for recovery...");
        } else {
            LOG_ERROR("[ESTOP] TCP safe stop triggered!");
            QuadEmergencyStop_Trigger(ESTOP_SOURCE_TCP_TIMEOUT, ESTOP_LEVEL_TCP_SAFE_STOP);
        }
    } else if (!active && was_active) {
        LOG_INFO("[ESTOP] TCP safe stop cleared");
        if (!g_no_peripheral_mode) {
            QuadEmergencyStop_Clear();
        }
    }
}

bool QuadEmergencyStop_IsTCPTimeout(void)
{
    return g_tcp_timeout_active;
}

bool QuadEmergencyStop_IsTCPHold(void)
{
    return g_tcp_hold_active;
}

bool QuadEmergencyStop_IsTCPSafeStop(void)
{
    return g_tcp_safe_stop_active;
}

void QuadEmergencyStop_ClearAllTCPFlags(void)
{
    g_tcp_timeout_active = false;
    g_tcp_hold_active = false;
    g_tcp_safe_stop_active = false;
}

/* ============================================================================
 * 高优先级紧急停止命令实现
 * ============================================================================ */

static EmergencyStopCallback_t g_emergency_stop_callback = NULL;

void QuadEmergencyStop_SetCallback(EmergencyStopCallback_t callback)
{
    g_emergency_stop_callback = callback;
}

void QuadEmergencyStop_Immediate(void)
{
    /* 1. 立即设置紧急停止标志 */
    g_estop_active = true;
    g_estop_source = ESTOP_SOURCE_SOFTWARE;
    g_estop_level = ESTOP_LEVEL_HARD;  /* 使用ESTOP_LEVEL_HARD */
    
    LOG_ERROR("[ESTOP] IMMEDIATE emergency stop triggered!");
    
    /* 2. 直接发送停止命令到CAN总线（绕过队列） */
    QuadEmergencyStop_SendHighPriority(0);  /* 0 = 全部CAN通道 */
    
    /* 3. 调用用户回调 */
    if (g_emergency_stop_callback != NULL) {
        g_emergency_stop_callback();
    }
    
    /* 4. 记录停止时间 */
    g_estop_trigger_tick = HAL_GetTick();
}

void QuadEmergencyStop_SendHighPriority(uint8_t can_channel)
{
    /* 直接调用底层电机停止函数，绕过命令队列 */
    extern uint8_t motor_start_stop(uint8_t can, uint8_t device_id, uint8_t start);
    
    if (can_channel == 0 || can_channel == 1) {
        /* CAN1: 电机1-6 */
        for (uint8_t id = 1; id <= 6; id++) {
            motor_start_stop(1, id, 0);  /* 0 = 停止 */
        }
    }
    
    if (can_channel == 0 || can_channel == 2) {
        /* CAN2: 电机7-12 */
        for (uint8_t id = 7; id <= 12; id++) {
            motor_start_stop(2, id, 0);  /* 0 = 停止 */
        }
    }
    
    LOG_ERROR("[ESTOP] High-priority stop commands sent to CAN%d", can_channel);
}
