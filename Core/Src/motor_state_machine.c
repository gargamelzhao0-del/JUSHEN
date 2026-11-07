/**
 ******************************************************************************
 * @file    motor_state_machine.c
 * @brief   电机状态机模块实现 - 确保正确的电机控制流程
 ******************************************************************************
 */

#include "motor_state_machine.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

static MotorSmContext_t g_motor_ctx[MOTOR_SM_MAX_DEVICES];
static MotorSmLimits_t g_motor_limits[MOTOR_SM_MAX_DEVICES];
static osMutexId_t g_sm_mutex = NULL;
static bool g_global_emergency = false;

static const MotorSmLimits_t g_default_limits = {
    .position_min = 0.0f,
    .position_max = 360.0f,
    .velocity_max = 500.0f,
    .torque_max = 50.0f,
    .temperature_max = 80.0f,
    .current_max = 20.0f,
    .limits_enabled = true
};

static uint8_t get_device_index(uint8_t device_id)
{
    if (device_id >= 1 && device_id <= 6) {
        return device_id - 1;
    } else if (device_id >= 7 && device_id <= 12) {
        return device_id - 1;
    }
    return 0xFF;
}

void MotorSm_Init(void)
{
    osMutexAttr_t mutex_attr = {
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    g_sm_mutex = osMutexNew(&mutex_attr);
    
    memset(g_motor_ctx, 0, sizeof(g_motor_ctx));
    
    for (int i = 0; i < MOTOR_SM_MAX_DEVICES; i++) {
        g_motor_ctx[i].device_id = (uint8_t)(i + 1);
        g_motor_ctx[i].can_channel = (i < 6) ? 1 : 2;
        g_motor_ctx[i].state = MOTOR_SM_STATE_OFF;
        g_motor_ctx[i].current_mode = MOTOR_MODE_POSITION;
        
        memcpy(&g_motor_limits[i], &g_default_limits, sizeof(MotorSmLimits_t));
    }
    
    g_global_emergency = false;
    
    printf("[MotorSM] State machine initialized for %d motors\r\n", MOTOR_SM_MAX_DEVICES);
}

void MotorSm_Deinit(void)
{
    if (g_sm_mutex != NULL) {
        osMutexDelete(g_sm_mutex);
        g_sm_mutex = NULL;
    }
}

MotorSmContext_t* MotorSm_GetContext(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return NULL;
    return &g_motor_ctx[idx];
}

static bool check_transition(MotorSmContext_t* ctx, MotorSmEvent_t event)
{
    switch (ctx->state) {
        case MOTOR_SM_STATE_OFF:
            return (event == MOTOR_SM_EVENT_ENABLE);
            
        case MOTOR_SM_STATE_DISABLED:
            return (event == MOTOR_SM_EVENT_ENABLE || 
                    event == MOTOR_SM_EVENT_SET_MODE);
            
        case MOTOR_SM_STATE_ENABLED:
            return (event == MOTOR_SM_EVENT_DISABLE ||
                    event == MOTOR_SM_EVENT_SET_MODE ||
                    event == MOTOR_SM_EVENT_START ||
                    event == MOTOR_SM_EVENT_EMERGENCY_STOP);
            
        case MOTOR_SM_STATE_MODE_SET:
            return (event == MOTOR_SM_EVENT_DISABLE ||
                    event == MOTOR_SM_EVENT_SET_MODE ||
                    event == MOTOR_SM_EVENT_START ||
                    event == MOTOR_SM_EVENT_EMERGENCY_STOP);
            
        case MOTOR_SM_STATE_RUNNING:
            return (event == MOTOR_SM_EVENT_STOP ||
                    event == MOTOR_SM_EVENT_DISABLE ||
                    event == MOTOR_SM_EVENT_EMERGENCY_STOP ||
                    event == MOTOR_SM_EVENT_ERROR);
            
        case MOTOR_SM_STATE_ERROR:
            return (event == MOTOR_SM_EVENT_CLEAR_ERROR ||
                    event == MOTOR_SM_EVENT_EMERGENCY_STOP);
            
        case MOTOR_SM_STATE_EMERGENCY_STOP:
            return (event == MOTOR_SM_EVENT_CLEAR_ERROR);
            
        default:
            return false;
    }
}

static void set_state(MotorSmContext_t* ctx, MotorSmState_t new_state)
{
    if (ctx->state != new_state) {
        printf("[MotorSM] Device %d: %s -> %s\r\n", 
               ctx->device_id,
               MotorSm_StateToString(ctx->state),
               MotorSm_StateToString(new_state));
        ctx->state = new_state;
        ctx->transition_count++;
    }
}

bool MotorSm_Enable(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (g_global_emergency) {
        printf("[MotorSM] Device %d: Cannot enable - global emergency active\r\n", device_id);
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (check_transition(ctx, MOTOR_SM_EVENT_ENABLE)) {
        uint8_t result = motor_enable(ctx->can_channel, device_id, 1);
        if (result == 0) {
            set_state(ctx, MOTOR_SM_STATE_ENABLED);
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    } else {
        printf("[MotorSM] Device %d: Invalid transition for ENABLE in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        ctx->last_error = MOTOR_SM_ERR_INVALID_TRANSITION;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_Disable(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (check_transition(ctx, MOTOR_SM_EVENT_DISABLE)) {
        uint8_t result = motor_enable(ctx->can_channel, device_id, 0);
        if (result == 0) {
            set_state(ctx, MOTOR_SM_STATE_DISABLED);
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    } else {
        printf("[MotorSM] Device %d: Invalid transition for DISABLE in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        ctx->last_error = MOTOR_SM_ERR_INVALID_TRANSITION;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_SetMode(uint8_t device_id, MotorMode_t mode)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (check_transition(ctx, MOTOR_SM_EVENT_SET_MODE)) {
        uint8_t result = motor_set_mode(ctx->can_channel, device_id, mode);
        if (result == 0) {
            ctx->current_mode = mode;
            set_state(ctx, MOTOR_SM_STATE_MODE_SET);
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    } else {
        printf("[MotorSM] Device %d: Invalid transition for SET_MODE in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        ctx->last_error = MOTOR_SM_ERR_INVALID_TRANSITION;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_Start(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (g_global_emergency) {
        printf("[MotorSM] Device %d: Cannot start - global emergency active\r\n", device_id);
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (check_transition(ctx, MOTOR_SM_EVENT_START)) {
        uint8_t result = motor_start_stop(ctx->can_channel, device_id, 1);
        if (result == 0) {
            set_state(ctx, MOTOR_SM_STATE_RUNNING);
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    } else {
        printf("[MotorSM] Device %d: Invalid transition for START in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        ctx->last_error = MOTOR_SM_ERR_INVALID_TRANSITION;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_Stop(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (check_transition(ctx, MOTOR_SM_EVENT_STOP)) {
        uint8_t result = motor_start_stop(ctx->can_channel, device_id, 0);
        if (result == 0) {
            set_state(ctx, MOTOR_SM_STATE_MODE_SET);
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    } else {
        printf("[MotorSM] Device %d: Invalid transition for STOP in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        ctx->last_error = MOTOR_SM_ERR_INVALID_TRANSITION;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_EmergencyStop(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    
    motor_start_stop(ctx->can_channel, device_id, 0);
    motor_enable(ctx->can_channel, device_id, 0);
    
    set_state(ctx, MOTOR_SM_STATE_EMERGENCY_STOP);
    ctx->last_command = HAL_GetTick();
    
    osMutexRelease(g_sm_mutex);
    return true;
}

bool MotorSm_ClearError(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (ctx->state == MOTOR_SM_STATE_ERROR || 
        ctx->state == MOTOR_SM_STATE_EMERGENCY_STOP) {
        ctx->last_error = MOTOR_SM_ERR_NONE;
        ctx->error_count = 0;
        set_state(ctx, MOTOR_SM_STATE_DISABLED);
        success = true;
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

static bool check_limits(uint8_t idx, float position, float velocity, float torque)
{
    MotorSmLimits_t* limits = &g_motor_limits[idx];
    
    if (!limits->limits_enabled) return true;
    
    if (position < limits->position_min || position > limits->position_max) {
        printf("[MotorSM] Position %.2f out of range [%.2f, %.2f]\r\n",
               position, limits->position_min, limits->position_max);
        return false;
    }
    
    if (fabsf(velocity) > limits->velocity_max) {
        printf("[MotorSM] Velocity %.2f exceeds max %.2f\r\n",
               velocity, limits->velocity_max);
        return false;
    }
    
    if (fabsf(torque) > limits->torque_max) {
        printf("[MotorSM] Torque %.2f exceeds max %.2f\r\n",
               torque, limits->torque_max);
        return false;
    }
    
    return true;
}

bool MotorSm_SetPosition(uint8_t device_id, float position)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (g_global_emergency) {
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->state != MOTOR_SM_STATE_RUNNING) {
        printf("[MotorSM] Device %d: Cannot set position in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->current_mode != MOTOR_MODE_POSITION) {
        printf("[MotorSM] Device %d: Not in position mode (current: %d)\r\n",
               device_id, ctx->current_mode);
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (check_limits(idx, position, 0, 0)) {
        uint8_t result = motor_set_position(ctx->can_channel, device_id, position);
        if (result == 0) {
            ctx->last_position = position;
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_SetVelocity(uint8_t device_id, float velocity)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (g_global_emergency) {
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->state != MOTOR_SM_STATE_RUNNING) {
        printf("[MotorSM] Device %d: Cannot set velocity in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->current_mode != MOTOR_MODE_SPEED) {
        printf("[MotorSM] Device %d: Not in speed mode (current: %d)\r\n",
               device_id, ctx->current_mode);
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (check_limits(idx, 0, velocity, 0)) {
        uint8_t result = motor_set_speed(ctx->can_channel, device_id, velocity);
        if (result == 0) {
            ctx->last_velocity = velocity;
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

bool MotorSm_SetTorque(uint8_t device_id, float torque)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) != osOK) return false;
    
    MotorSmContext_t* ctx = &g_motor_ctx[idx];
    bool success = false;
    
    if (g_global_emergency) {
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->state != MOTOR_SM_STATE_RUNNING) {
        printf("[MotorSM] Device %d: Cannot set torque in state %s\r\n",
               device_id, MotorSm_StateToString(ctx->state));
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (ctx->current_mode != MOTOR_MODE_TORQUE) {
        printf("[MotorSM] Device %d: Not in torque mode (current: %d)\r\n",
               device_id, ctx->current_mode);
        osMutexRelease(g_sm_mutex);
        return false;
    }
    
    if (check_limits(idx, 0, 0, torque)) {
        uint8_t result = motor_set_torque(ctx->can_channel, device_id, torque);
        if (result == 0) {
            ctx->last_torque = torque;
            ctx->last_command = HAL_GetTick();
            success = true;
        }
    }
    
    osMutexRelease(g_sm_mutex);
    return success;
}

void MotorSm_SetLimits(uint8_t device_id, const MotorSmLimits_t* limits)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF || limits == NULL) return;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) == osOK) {
        memcpy(&g_motor_limits[idx], limits, sizeof(MotorSmLimits_t));
        osMutexRelease(g_sm_mutex);
    }
}

void MotorSm_EnableLimits(uint8_t device_id, bool enable)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) == osOK) {
        g_motor_limits[idx].limits_enabled = enable;
        osMutexRelease(g_sm_mutex);
    }
}

MotorSmState_t MotorSm_GetState(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return MOTOR_SM_STATE_ERROR;
    return g_motor_ctx[idx].state;
}

bool MotorSm_IsRunning(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return false;
    return g_motor_ctx[idx].state == MOTOR_SM_STATE_RUNNING;
}

bool MotorSm_HasError(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return true;
    return g_motor_ctx[idx].state == MOTOR_SM_STATE_ERROR ||
           g_motor_ctx[idx].state == MOTOR_SM_STATE_EMERGENCY_STOP;
}

MotorSmError_t MotorSm_GetLastError(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return MOTOR_SM_ERR_COMMUNICATION;
    return g_motor_ctx[idx].last_error;
}

void MotorSm_UpdateFeedback(uint8_t device_id, float position, float velocity, float torque)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return;
    
    if (osMutexAcquire(g_sm_mutex, osWaitForever) == osOK) {
        g_motor_ctx[idx].last_position = position;
        g_motor_ctx[idx].last_velocity = velocity;
        g_motor_ctx[idx].last_torque = torque;
        g_motor_ctx[idx].position_valid = true;
        g_motor_ctx[idx].velocity_valid = true;
        g_motor_ctx[idx].torque_valid = true;
        g_motor_ctx[idx].last_heartbeat = HAL_GetTick();
        osMutexRelease(g_sm_mutex);
    }
}

void MotorSm_ProcessHeartbeat(uint8_t device_id)
{
    uint8_t idx = get_device_index(device_id);
    if (idx == 0xFF) return;
    
    g_motor_ctx[idx].last_heartbeat = HAL_GetTick();
}

void MotorSm_GlobalEmergencyStop(void)
{
    if (osMutexAcquire(g_sm_mutex, osWaitForever) == osOK) {
        g_global_emergency = true;
        Security_SetEmergencyStop(true);
        
        for (int i = 0; i < MOTOR_SM_MAX_DEVICES; i++) {
            MotorSmContext_t* ctx = &g_motor_ctx[i];
            if (ctx->state == MOTOR_SM_STATE_RUNNING) {
                motor_start_stop(ctx->can_channel, ctx->device_id, 0);
            }
            set_state(ctx, MOTOR_SM_STATE_EMERGENCY_STOP);
        }
        
        osMutexRelease(g_sm_mutex);
    }
    
    printf("[MotorSM] Global emergency stop activated!\r\n");
}

void MotorSm_GlobalClearEmergency(void)
{
    if (osMutexAcquire(g_sm_mutex, osWaitForever) == osOK) {
        g_global_emergency = false;
        Security_SetEmergencyStop(false);
        
        for (int i = 0; i < MOTOR_SM_MAX_DEVICES; i++) {
            MotorSmContext_t* ctx = &g_motor_ctx[i];
            if (ctx->state == MOTOR_SM_STATE_EMERGENCY_STOP) {
                ctx->last_error = MOTOR_SM_ERR_NONE;
                set_state(ctx, MOTOR_SM_STATE_DISABLED);
            }
        }
        
        osMutexRelease(g_sm_mutex);
    }
    
    printf("[MotorSM] Global emergency cleared\r\n");
}

bool MotorSm_IsGlobalEmergency(void)
{
    return g_global_emergency;
}

void MotorSm_Process(void)
{
    uint32_t now = HAL_GetTick();
    
    if (osMutexAcquire(g_sm_mutex, 10) != osOK) return;
    
    for (int i = 0; i < MOTOR_SM_MAX_DEVICES; i++) {
        MotorSmContext_t* ctx = &g_motor_ctx[i];
        
        if (ctx->state == MOTOR_SM_STATE_RUNNING) {
            if ((now - ctx->last_heartbeat) > MOTOR_SM_HEARTBEAT_MS * 10) {
                printf("[MotorSM] Device %d: Heartbeat timeout\r\n", ctx->device_id);
                ctx->last_error = MOTOR_SM_ERR_TIMEOUT;
                set_state(ctx, MOTOR_SM_STATE_ERROR);
                ctx->error_count++;
            }
        }
    }
    
    osMutexRelease(g_sm_mutex);
}

const char* MotorSm_StateToString(MotorSmState_t state)
{
    switch (state) {
        case MOTOR_SM_STATE_OFF:            return "OFF";
        case MOTOR_SM_STATE_DISABLED:       return "DISABLED";
        case MOTOR_SM_STATE_ENABLED:        return "ENABLED";
        case MOTOR_SM_STATE_MODE_SET:       return "MODE_SET";
        case MOTOR_SM_STATE_RUNNING:        return "RUNNING";
        case MOTOR_SM_STATE_ERROR:          return "ERROR";
        case MOTOR_SM_STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                            return "UNKNOWN";
    }
}

const char* MotorSm_ErrorToString(MotorSmError_t error)
{
    switch (error) {
        case MOTOR_SM_ERR_NONE:              return "NONE";
        case MOTOR_SM_ERR_INVALID_TRANSITION: return "INVALID_TRANSITION";
        case MOTOR_SM_ERR_TIMEOUT:           return "TIMEOUT";
        case MOTOR_SM_ERR_COMMUNICATION:     return "COMMUNICATION";
        case MOTOR_SM_ERR_OVERTEMP:          return "OVERTEMP";
        case MOTOR_SM_ERR_OVERCURRENT:       return "OVERCURRENT";
        case MOTOR_SM_ERR_POSITION_LIMIT:    return "POSITION_LIMIT";
        case MOTOR_SM_ERR_VELOCITY_LIMIT:    return "VELOCITY_LIMIT";
        case MOTOR_SM_ERR_TORQUE_LIMIT:      return "TORQUE_LIMIT";
        default:                             return "UNKNOWN";
    }
}
