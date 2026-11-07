/**
 ******************************************************************************
 * @file    quad_emergency_stop.h
 * @brief   四足机器狗紧急停止系统 (硬件 + 软件)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __QUAD_EMERGENCY_STOP_H__
#define __QUAD_EMERGENCY_STOP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "quad_types.h"
#include "robot_state.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef EMERGENCY_STOP_SOURCE_DEFINED
typedef enum {
    ESTOP_SOURCE_NONE = 0,
    ESTOP_SOURCE_GPIO = 1,       /* 硬件紧急停止按钮 */
    ESTOP_SOURCE_SOFTWARE = 2,    /* 软件触发 */
    ESTOP_SOURCE_WATCHDOG = 4,   /* 看门狗超时 */
    ESTOP_SOURCE_TCP = 8,        /* 上位机触发 */
    ESTOP_SOURCE_OVERCURRENT = 16,/* 过流保护 */
    ESTOP_SOURCE_OVERTEMP = 32,  /* 过温保护 */
    ESTOP_SOURCE_TCP_TIMEOUT = 64 /* TCP超时 - 修复: 添加TCP超时源 */
} EmergencyStopSource_t;
#define EMERGENCY_STOP_SOURCE_DEFINED
#endif

/* 修复: 添加TCP超时级别定义 */
#define ESTOP_LEVEL_TCP_TIMEOUT    0x10    /* TCP超时警告 */
#define ESTOP_LEVEL_TCP_HOLD       0x11    /* TCP超时保持 */
#define ESTOP_LEVEL_TCP_SAFE_STOP  0x12    /* TCP超时安全停止 */

typedef struct {
    EmergencyStopSource_t source;
    uint32_t level;
    uint32_t timestamp_ms;
    bool is_active;
    uint8_t stopped_motors[QUAD_MAX_MOTORS];
} EmergencyStopStatus_t;

void QuadEmergencyStop_Init(void);

void QuadEmergencyStop_Trigger(EmergencyStopSource_t source, uint32_t level);
void QuadEmergencyStop_Clear(void);

bool QuadEmergencyStop_IsActive(void);
EmergencyStopSource_t QuadEmergencyStop_GetSource(void);

void QuadEmergencyStop_GetStatus(EmergencyStopStatus_t *status);
void QuadEmergencyStop_PrintStatus(void);

void QuadEmergencyStop_SetGPIOPin(GPIO_TypeDef* port, uint16_t pin);
void QuadEmergencyStop_EnableGPIOInterrupt(bool enable);

/* 修复: TCP超时状态管理函数 */
void QuadEmergencyStop_SetTCPTimeout(bool active);
void QuadEmergencyStop_SetTCPHold(bool active);
void QuadEmergencyStop_SetTCPSafeStop(bool active);
bool QuadEmergencyStop_IsTCPTimeout(void);
bool QuadEmergencyStop_IsTCPHold(void);
bool QuadEmergencyStop_IsTCPSafeStop(void);
void QuadEmergencyStop_ClearAllTCPFlags(void);

/* V4.0: 无外设模式支持 */
void QuadEmergencyStop_SetNoPeripheralMode(bool enabled);
bool QuadEmergencyStop_IsNoPeripheralMode(void);

/* ============================================================================
 * 高优先级紧急停止命令 (绕过普通命令队列)
 * ============================================================================ */

/**
 * @brief 立即紧急停止（最高优先级，不经过队列）
 * @note  直接发送停止指令到CAN总线，确保最快响应
 */
void QuadEmergencyStop_Immediate(void);

/**
 * @brief 设置高优先级紧急停止回调
 * @param callback: 回调函数指针
 */
typedef void (*EmergencyStopCallback_t)(void);
void QuadEmergencyStop_SetCallback(EmergencyStopCallback_t callback);

/**
 * @brief 紧急停止命令结构（用于高优先级队列）
 */
typedef struct {
    uint8_t motor_id;          /* 电机ID */
    uint8_t can_port;          /* CAN端口 */
    uint8_t command;           /* 命令类型: 0=停止, 1=失能 */
    uint8_t priority;          /* 优先级 (固定为255) */
} EmergencyCommand_t;

/**
 * @brief 发送高优先级紧急停止命令
 * @param can_channel: CAN通道 (1或2, 0=全部)
 * @note  绕过普通命令队列，直接发送
 */
void QuadEmergencyStop_SendHighPriority(uint8_t can_channel);

/**
 * @brief 轮询检查ISR触发的急停请求
 * @note  必须在FreeRTOS任务上下文中周期性调用（如控制循环任务），
 *        用于将GPIO EXTI中断中延迟的急停请求转发到实际处理函数
 */
void QuadEmergencyStop_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_EMERGENCY_STOP_H__ */
