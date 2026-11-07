/**
 ******************************************************************************
 * @file    quad_hybrid_control.h
 * @brief   四足机器狗力位混合控制算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * 力位混合控制 (Force-Position Hybrid Control):
 * 在支撑相同时控制位置和力矩，实现对地面的柔顺接触
 *
 * 控制方程:
 * τ = Kp_pos * (θ_target - θ_actual) + Kp_force * (F_target - F_actual)
 *
 ******************************************************************************
 */

#ifndef __QUAD_HYBRID_CONTROL_H__
#define __QUAD_HYBRID_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "quad_types.h"
#include <stdint.h>
#include <stdbool.h>

#define HYBRID_MAX_MOTORS     12

typedef enum {
    HYBRID_MODE_DISABLED = 0,
    HYBRID_MODE_POSITION_ONLY,
    HYBRID_MODE_FORCE_ONLY,
    HYBRID_MODE_HYBRID
} HybridMode_t;

/* 使用 quad_types.h 中的 QuadHybridState_t 枚举 */
/* 移除重复的 HybridPhase_t 定义，使用 QuadHybridState_t 代替 */
typedef QuadHybridState_t HybridPhase_t;  /* 兼容旧代码 */

typedef struct {
    float position_kp;
    float position_ki;
    float position_kd;

    float force_kp;
    float force_ki;
    float force_kd;

    float max_torque;
    float max_velocity;

    float target_position;
    float target_force;

    float transition_time_ms;
} HybridControlParams_t;

typedef struct {
    HybridMode_t mode;
    HybridPhase_t phase;

    float current_position;
    float current_velocity;
    float current_force;

    float position_error;
    float position_error_prev;
    float position_integral;

    float force_error;
    float force_error_prev;
    float force_integral;

    float pos_derivative;
    float force_derivative;

    float output_torque;
    float output_position;

    uint32_t update_count;
    uint32_t last_update_tick;
} HybridMotorState_t;

typedef struct {
    uint8_t motor_id;
    bool enabled;
    HybridMode_t mode;

    HybridControlParams_t params;
    HybridMotorState_t state;

    float actual_position;
    float actual_velocity;
    float actual_torque;

    float foot_force;
    uint8_t contact_detected;
} HybridControl_t;

void HybridControl_Init(void);
void HybridControl_Deinit(void);

void HybridControl_Enable(uint8_t motor_id, bool enable);
bool HybridControl_IsEnabled(uint8_t motor_id);

void HybridControl_SetMode(uint8_t motor_id, HybridMode_t mode);
HybridMode_t HybridControl_GetMode(uint8_t motor_id);

void HybridControl_SetParams(uint8_t motor_id, HybridControlParams_t *params);
void HybridControl_GetParams(uint8_t motor_id, HybridControlParams_t *params);

void HybridControl_SetTarget(uint8_t motor_id, float target_position, float target_force);
void HybridControl_GetTarget(uint8_t motor_id, float *target_position, float *target_force);

void HybridControl_UpdateMotor(uint8_t motor_id,
                              float actual_position,
                              float actual_velocity,
                              float actual_force,
                              float *output_torque,
                              float *output_position);

void HybridControl_UpdateAll(float target_positions[HYBRID_MAX_MOTORS],
                           float actual_positions[HYBRID_MAX_MOTORS],
                           float actual_velocities[HYBRID_MAX_MOTORS],
                           float foot_forces[4],
                           float torque_outputs[HYBRID_MAX_MOTORS],
                           float position_outputs[HYBRID_MAX_MOTORS]);

void HybridControl_GetState(uint8_t motor_id, HybridMotorState_t *state);
void HybridControl_PrintStatus(uint8_t motor_id);

void HybridControl_SetForceGains(uint8_t motor_id, float kp, float ki, float kd);
void HybridControl_SetPositionGains(uint8_t motor_id, float kp, float ki, float kd);

void HybridControl_Reset(uint8_t motor_id);
void HybridControl_ResetAll(void);

extern bool g_hybrid_initialized;

uint8_t HybridControl_GetPhase(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_HYBRID_CONTROL_H__ */
