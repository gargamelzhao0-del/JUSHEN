/**
 ******************************************************************************
 * @file    motor_state_machine.h
 * @brief   电机状态机模块 - 确保正确的电机控制流程
 * @note    实现电机状态转换、安全保护、错误恢复
 ******************************************************************************
 */

#ifndef __MOTOR_STATE_MACHINE_H
#define __MOTOR_STATE_MACHINE_H

#include "main.h"
#include "motor_control.h"
#include "security_auth.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_SM_MAX_DEVICES    12
#define MOTOR_SM_TIMEOUT_MS     1000
#define MOTOR_SM_HEARTBEAT_MS   100

typedef enum {
    MOTOR_SM_STATE_OFF = 0,
    MOTOR_SM_STATE_DISABLED,
    MOTOR_SM_STATE_ENABLED,
    MOTOR_SM_STATE_MODE_SET,
    MOTOR_SM_STATE_RUNNING,
    MOTOR_SM_STATE_ERROR,
    MOTOR_SM_STATE_EMERGENCY_STOP
} MotorSmState_t;

typedef enum {
    MOTOR_SM_EVENT_NONE = 0,
    MOTOR_SM_EVENT_ENABLE,
    MOTOR_SM_EVENT_DISABLE,
    MOTOR_SM_EVENT_SET_MODE,
    MOTOR_SM_EVENT_START,
    MOTOR_SM_EVENT_STOP,
    MOTOR_SM_EVENT_ERROR,
    MOTOR_SM_EVENT_CLEAR_ERROR,
    MOTOR_SM_EVENT_EMERGENCY_STOP,
    MOTOR_SM_EVENT_HEARTBEAT_TIMEOUT
} MotorSmEvent_t;

typedef enum {
    MOTOR_SM_ERR_NONE = 0,
    MOTOR_SM_ERR_INVALID_TRANSITION,
    MOTOR_SM_ERR_TIMEOUT,
    MOTOR_SM_ERR_COMMUNICATION,
    MOTOR_SM_ERR_OVERTEMP,
    MOTOR_SM_ERR_OVERCURRENT,
    MOTOR_SM_ERR_POSITION_LIMIT,
    MOTOR_SM_ERR_VELOCITY_LIMIT,
    MOTOR_SM_ERR_TORQUE_LIMIT
} MotorSmError_t;

typedef struct {
    uint8_t can_channel;
    uint8_t device_id;
    MotorSmState_t state;
    MotorSmError_t last_error;
    MotorMode_t current_mode;
    uint32_t last_heartbeat;
    uint32_t last_command;
    uint32_t error_count;
    uint32_t transition_count;
    float last_position;
    float last_velocity;
    float last_torque;
    bool position_valid;
    bool velocity_valid;
    bool torque_valid;
} MotorSmContext_t;

typedef struct {
    float position_min;
    float position_max;
    float velocity_max;
    float torque_max;
    float temperature_max;
    float current_max;
    bool limits_enabled;
} MotorSmLimits_t;

void MotorSm_Init(void);
void MotorSm_Deinit(void);

MotorSmContext_t* MotorSm_GetContext(uint8_t device_id);

bool MotorSm_Enable(uint8_t device_id);
bool MotorSm_Disable(uint8_t device_id);
bool MotorSm_SetMode(uint8_t device_id, MotorMode_t mode);
bool MotorSm_Start(uint8_t device_id);
bool MotorSm_Stop(uint8_t device_id);
bool MotorSm_EmergencyStop(uint8_t device_id);
bool MotorSm_ClearError(uint8_t device_id);

bool MotorSm_SetPosition(uint8_t device_id, float position);
bool MotorSm_SetVelocity(uint8_t device_id, float velocity);
bool MotorSm_SetTorque(uint8_t device_id, float torque);

void MotorSm_SetLimits(uint8_t device_id, const MotorSmLimits_t* limits);
void MotorSm_EnableLimits(uint8_t device_id, bool enable);

MotorSmState_t MotorSm_GetState(uint8_t device_id);
bool MotorSm_IsRunning(uint8_t device_id);
bool MotorSm_HasError(uint8_t device_id);
MotorSmError_t MotorSm_GetLastError(uint8_t device_id);

void MotorSm_UpdateFeedback(uint8_t device_id, float position, float velocity, float torque);
void MotorSm_ProcessHeartbeat(uint8_t device_id);

void MotorSm_GlobalEmergencyStop(void);
void MotorSm_GlobalClearEmergency(void);
bool MotorSm_IsGlobalEmergency(void);

void MotorSm_Process(void);

const char* MotorSm_StateToString(MotorSmState_t state);
const char* MotorSm_ErrorToString(MotorSmError_t error);

#endif
