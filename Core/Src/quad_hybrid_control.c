/**
 ******************************************************************************
 * @file    quad_hybrid_control.c
 * @brief   四足机器狗力位混合控制算法实现
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * 力位混合控制核心算法:
 * ====================
 *
 * 1. 位置环 PID控制器:
 *    τ_pos = Kp_p * e_pos + Ki_p * ∫e_pos + Kd_p * de_pos/dt
 *    其中 e_pos = θ_target - θ_actual
 *
 * 2. 力环 PID控制器:
 *    τ_force = Kp_f * e_force + Ki_f * ∫e_force + Kd_f * de_force/dt
 *    其中 e_force = F_target - F_actual
 *
 * 3. 混合控制输出:
 *    τ_output = α * τ_pos + (1-α) * τ_force
 *    其中 α 是位置权重，根据相位动态调整
 *
 * 4. 相位控制:
 *    - SWING (摆动相): α=1.0 (纯位置控制)
 *    - TOUCHDOWN (触地相): α 线性过渡
 *    - SUPPORT (支撑相): α 动态调整
 *    - LIFT (抬腿相): α 线性过渡到1.0
 *
 ******************************************************************************
 */

#include "quad_hybrid_control.h"
#include "quad_types.h"
#include "foot_force_sensor.h"
#include "log_system.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

#define HYBRID_CONTROL_FREQUENCY    100.0f   /* 控制频率 100Hz */
#define HYBRID_DT                   0.01f    /* 控制周期 10ms */
#define FORCE_SENSOR_TIMEOUT_MS     500      /* V10.0: 力传感器超时时间 */

static HybridControl_t g_hybrid_controls[HYBRID_MAX_MOTORS];
bool g_hybrid_initialized = false;
static osMutexId_t g_hybrid_mutex = NULL;
static bool g_force_sensor_degraded = false;  /* V10.0: 力传感器降级标志 */

static const HybridControlParams_t DEFAULT_PARAMS = {
    .position_kp = 20.0f,
    .position_ki = 0.5f,
    .position_kd = 1.0f,

    .force_kp = 2.0f,
    .force_ki = 0.1f,
    .force_kd = 0.5f,

    .max_torque = 2.0f,
    .max_velocity = 100.0f,

    .target_position = 0.0f,
    .target_force = 5.0f,

    .transition_time_ms = 50.0f
};

void HybridControl_Init(void)
{
    if (g_hybrid_initialized) {
        LOG_WARN("[Hybrid] Already initialized");
        return;
    }

    LOG_INFO("[Hybrid] Initializing force-position hybrid control...");

    osMutexAttr_t mutex_attr = {
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    g_hybrid_mutex = osMutexNew(&mutex_attr);

    for (int i = 0; i < HYBRID_MAX_MOTORS; i++) {
        memset(&g_hybrid_controls[i], 0, sizeof(HybridControl_t));

        g_hybrid_controls[i].motor_id = i;
        g_hybrid_controls[i].enabled = false;
        g_hybrid_controls[i].mode = HYBRID_MODE_DISABLED;
        g_hybrid_controls[i].state.phase = HYBRID_STATE_IDLE;

        memcpy(&g_hybrid_controls[i].params, &DEFAULT_PARAMS, sizeof(HybridControlParams_t));
    }

    g_hybrid_initialized = true;
    LOG_INFO("[Hybrid] Force-position hybrid control initialized");
}

void HybridControl_Deinit(void)
{
    if (!g_hybrid_initialized) return;

    for (int i = 0; i < HYBRID_MAX_MOTORS; i++) {
        g_hybrid_controls[i].enabled = false;
        g_hybrid_controls[i].mode = HYBRID_MODE_DISABLED;
    }

    if (g_hybrid_mutex != NULL) {
        osMutexDelete(g_hybrid_mutex);
        g_hybrid_mutex = NULL;
    }

    g_hybrid_initialized = false;
    LOG_INFO("[Hybrid] Hybrid control deinitialized");
}

void HybridControl_Enable(uint8_t motor_id, bool enable)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;
    if (!g_hybrid_initialized) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);

    g_hybrid_controls[motor_id].enabled = enable;

    if (enable) {
        g_hybrid_controls[motor_id].mode = HYBRID_MODE_HYBRID;
        g_hybrid_controls[motor_id].state.phase = HYBRID_STATE_SWING;
        g_hybrid_controls[motor_id].state.position_integral = 0.0f;
        g_hybrid_controls[motor_id].state.force_integral = 0.0f;
        g_hybrid_controls[motor_id].state.position_error = 0.0f;
        g_hybrid_controls[motor_id].state.position_error_prev = 0.0f;
        g_hybrid_controls[motor_id].state.force_error = 0.0f;
        g_hybrid_controls[motor_id].state.force_error_prev = 0.0f;
        g_hybrid_controls[motor_id].state.pos_derivative = 0.0f;
        g_hybrid_controls[motor_id].state.force_derivative = 0.0f;
        g_hybrid_controls[motor_id].state.output_torque = 0.0f;
        LOG_INFO("[Hybrid] Motor %d hybrid control enabled", motor_id);
    } else {
        g_hybrid_controls[motor_id].mode = HYBRID_MODE_DISABLED;
        g_hybrid_controls[motor_id].state.output_torque = 0.0f;
        LOG_INFO("[Hybrid] Motor %d hybrid control disabled", motor_id);
    }

    osMutexRelease(g_hybrid_mutex);
}

bool HybridControl_IsEnabled(uint8_t motor_id)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return false;
    return g_hybrid_controls[motor_id].enabled;
}

void HybridControl_SetMode(uint8_t motor_id, HybridMode_t mode)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;
    g_hybrid_controls[motor_id].mode = mode;
}

HybridMode_t HybridControl_GetMode(uint8_t motor_id)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return HYBRID_MODE_DISABLED;
    return g_hybrid_controls[motor_id].mode;
}

void HybridControl_SetParams(uint8_t motor_id, HybridControlParams_t *params)
{
    if (motor_id >= HYBRID_MAX_MOTORS || params == NULL) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    memcpy(&g_hybrid_controls[motor_id].params, params, sizeof(HybridControlParams_t));
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_GetParams(uint8_t motor_id, HybridControlParams_t *params)
{
    if (motor_id >= HYBRID_MAX_MOTORS || params == NULL) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    memcpy(params, &g_hybrid_controls[motor_id].params, sizeof(HybridControlParams_t));
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_SetTarget(uint8_t motor_id, float target_position, float target_force)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    g_hybrid_controls[motor_id].params.target_position = target_position;
    g_hybrid_controls[motor_id].params.target_force = target_force;
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_GetTarget(uint8_t motor_id, float *target_position, float *target_force)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    if (target_position != NULL) {
        *target_position = g_hybrid_controls[motor_id].params.target_position;
    }
    if (target_force != NULL) {
        *target_force = g_hybrid_controls[motor_id].params.target_force;
    }
    osMutexRelease(g_hybrid_mutex);
}

static HybridPhase_t detect_phase(uint8_t motor_id, float foot_force, float force_threshold)
{
    static HybridPhase_t last_phase[HYBRID_MAX_MOTORS] = {HYBRID_STATE_SWING};
    static uint32_t touch_down_time[HYBRID_MAX_MOTORS] = {0};
    static uint32_t support_start_time[HYBRID_MAX_MOTORS] = {0};
    static uint32_t last_update_time[HYBRID_MAX_MOTORS] = {0};

    if (motor_id >= HYBRID_MAX_MOTORS) return HYBRID_STATE_SWING;

    uint32_t current_time = osKernelGetTickCount();
    uint32_t dt = 0;
    if (last_update_time[motor_id] > 0) {
        dt = current_time - last_update_time[motor_id];
    }
    last_update_time[motor_id] = current_time;

    bool contact_detected = (foot_force > force_threshold);
    HybridPhase_t current_phase = last_phase[motor_id];

    switch (current_phase) {
        case HYBRID_STATE_SWING:
        case HYBRID_STATE_LIFT:
            if (contact_detected) {
                touch_down_time[motor_id] = current_time;
                current_phase = HYBRID_STATE_TOUCHDOWN;
            }
            break;

        case HYBRID_STATE_TOUCHDOWN:
            if (!contact_detected) {
                current_phase = HYBRID_STATE_LIFT;
            } else if ((current_time - touch_down_time[motor_id]) > 20) {
                support_start_time[motor_id] = current_time;
                current_phase = HYBRID_STATE_SUPPORT;
            }
            break;

        case HYBRID_STATE_SUPPORT:
            if (!contact_detected) {
                current_phase = HYBRID_STATE_LIFT;
                touch_down_time[motor_id] = current_time;
            }
            break;

        default:
            current_phase = HYBRID_STATE_SWING;
            break;
    }

    last_phase[motor_id] = current_phase;
    return current_phase;
}

void HybridControl_UpdateMotor(uint8_t motor_id,
                              float actual_position,
                              float actual_velocity,
                              float actual_force,
                              float *output_torque,
                              float *output_position)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;
    if (output_torque == NULL || output_position == NULL) return;

    HybridControl_t *ctrl = &g_hybrid_controls[motor_id];

    if (!ctrl->enabled) {
        *output_torque = 0.0f;
        *output_position = ctrl->params.target_position;
        return;
    }

    ctrl->state.current_position = actual_position;
    ctrl->state.current_velocity = actual_velocity;
    ctrl->state.current_force = actual_force;

    HybridPhase_t prev_phase = ctrl->state.phase;
    ctrl->state.phase = detect_phase(motor_id, actual_force, 1.0f);

    if (prev_phase != ctrl->state.phase) {
        ctrl->state.position_integral *= 0.5f;
        ctrl->state.force_integral *= 0.5f;
    }

    float pos_error = ctrl->params.target_position - actual_position;
    float force_error = ctrl->params.target_force - actual_force;

    ctrl->state.position_error_prev = ctrl->state.position_error;
    ctrl->state.position_error = pos_error;

    float pos_integral_limit = (ctrl->params.position_ki > 0.001f) ?
                                (ctrl->params.max_torque / ctrl->params.position_ki) : 0.0f;
    ctrl->state.position_integral += pos_error * HYBRID_DT;
    if (ctrl->state.position_integral > pos_integral_limit) {
        ctrl->state.position_integral = pos_integral_limit;
    } else if (ctrl->state.position_integral < -pos_integral_limit) {
        ctrl->state.position_integral = -pos_integral_limit;
    }

    ctrl->state.force_error_prev = ctrl->state.force_error;
    ctrl->state.force_error = force_error;

    float force_integral_limit = (ctrl->params.force_ki > 0.001f) ?
                                  (ctrl->params.max_torque / ctrl->params.force_ki) : 0.0f;
    ctrl->state.force_integral += force_error * HYBRID_DT;
    if (ctrl->state.force_integral > force_integral_limit) {
        ctrl->state.force_integral = force_integral_limit;
    } else if (ctrl->state.force_integral < -force_integral_limit) {
        ctrl->state.force_integral = -force_integral_limit;
    }

    float pos_derivative_raw = (pos_error - ctrl->state.position_error_prev) / HYBRID_DT;
    float force_derivative_raw = (force_error - ctrl->state.force_error_prev) / HYBRID_DT;

    float derivative_filter = 0.3f;
    ctrl->state.pos_derivative = derivative_filter * pos_derivative_raw + (1.0f - derivative_filter) * ctrl->state.pos_derivative;
    ctrl->state.force_derivative = derivative_filter * force_derivative_raw + (1.0f - derivative_filter) * ctrl->state.force_derivative;

    float pos_control = ctrl->params.position_kp * pos_error +
                        ctrl->params.position_ki * ctrl->state.position_integral +
                        ctrl->params.position_kd * ctrl->state.pos_derivative;

    float force_control = ctrl->params.force_kp * force_error +
                         ctrl->params.force_ki * ctrl->state.force_integral +
                         ctrl->params.force_kd * ctrl->state.force_derivative;

    float pos_feedforward = 0.0f;
    if (ctrl->params.position_ki < 0.001f && fabsf(actual_velocity) > 0.1f) {
        pos_feedforward = 0.1f * actual_velocity;
    }

    float alpha = 0.5f;

    switch (ctrl->mode) {
        case HYBRID_MODE_POSITION_ONLY:
            alpha = 1.0f;
            break;

        case HYBRID_MODE_FORCE_ONLY:
            alpha = 0.0f;
            break;

        case HYBRID_MODE_HYBRID:
        default:
            switch (ctrl->state.phase) {
                case HYBRID_STATE_SWING:
                case HYBRID_STATE_LIFT:
                    alpha = 1.0f;
                    break;

                case HYBRID_STATE_TOUCHDOWN:
                    alpha = 0.7f;
                    break;

                case HYBRID_STATE_SUPPORT:
                    alpha = 0.3f;
                    break;

                default:
                    alpha = 0.5f;
                    break;
            }
            break;
    }

    float torque_output = alpha * (pos_control + pos_feedforward) + (1.0f - alpha) * force_control;

    if (torque_output > ctrl->params.max_torque) {
        torque_output = ctrl->params.max_torque;
    } else if (torque_output < -ctrl->params.max_torque) {
        torque_output = -ctrl->params.max_torque;
    }

    ctrl->state.output_torque = torque_output;
    ctrl->state.output_position = ctrl->params.target_position;

    ctrl->state.update_count++;
    ctrl->state.last_update_tick = osKernelGetTickCount();

    *output_torque = torque_output;
    *output_position = ctrl->params.target_position;
}

void HybridControl_UpdateAll(float target_positions[HYBRID_MAX_MOTORS],
                           float actual_positions[HYBRID_MAX_MOTORS],
                           float actual_velocities[HYBRID_MAX_MOTORS],
                           float foot_forces[4],
                           float torque_outputs[HYBRID_MAX_MOTORS],
                           float position_outputs[HYBRID_MAX_MOTORS])
{
    if (!g_hybrid_initialized) return;
    if (target_positions == NULL || actual_positions == NULL ||
        torque_outputs == NULL || position_outputs == NULL) return;

    /* V10.0: 力传感器超时检测与自动降级 */
    bool force_sensor_valid = foot_force_all_valid(FORCE_SENSOR_TIMEOUT_MS);
    if (!force_sensor_valid && !g_force_sensor_degraded) {
        g_force_sensor_degraded = true;
        LOG_WARN("[Hybrid] Force sensor timeout detected, degrading to position-only mode");
    } else if (force_sensor_valid && g_force_sensor_degraded) {
        g_force_sensor_degraded = false;
        LOG_INFO("[Hybrid] Force sensor recovered, resuming hybrid control");
    }

    for (int i = 0; i < HYBRID_MAX_MOTORS; i++) {
        osMutexAcquire(g_hybrid_mutex, osWaitForever);

        g_hybrid_controls[i].params.target_position = target_positions[i];

        int leg_id = i / 3;
        if (leg_id >= 0 && leg_id < 4 && foot_forces != NULL) {
            g_hybrid_controls[i].foot_force = foot_forces[leg_id];
            g_hybrid_controls[i].contact_detected = (foot_forces[leg_id] > 1.0f) ? 1 : 0;
        }

        /* V10.0: 力传感器降级时自动切换到纯位置控制 */
        HybridMode_t original_mode = g_hybrid_controls[i].mode;
        if (g_force_sensor_degraded && g_hybrid_controls[i].mode == HYBRID_MODE_HYBRID) {
            g_hybrid_controls[i].mode = HYBRID_MODE_POSITION_ONLY;
        }

        float torque = 0.0f;
        float position = 0.0f;

        HybridControl_UpdateMotor(i,
                                  actual_positions[i],
                                  actual_velocities ? actual_velocities[i] : 0.0f,
                                  g_hybrid_controls[i].foot_force,
                                  &torque,
                                  &position);

        /* V10.0: 恢复原始模式 */
        if (g_force_sensor_degraded) {
            g_hybrid_controls[i].mode = original_mode;
        }

        torque_outputs[i] = torque;
        position_outputs[i] = position;

        osMutexRelease(g_hybrid_mutex);
    }
}

void HybridControl_GetState(uint8_t motor_id, HybridMotorState_t *state)
{
    if (motor_id >= HYBRID_MAX_MOTORS || state == NULL) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    memcpy(state, &g_hybrid_controls[motor_id].state, sizeof(HybridMotorState_t));
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_PrintStatus(uint8_t motor_id)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    HybridControl_t *ctrl = &g_hybrid_controls[motor_id];

    printf("\n========== Hybrid Control Status (Motor %d) ==========\n", motor_id);
    printf("Enabled:      %s\n", ctrl->enabled ? "YES" : "NO");
    printf("Mode:         %s\n",
           ctrl->mode == HYBRID_MODE_DISABLED ? "DISABLED" :
           ctrl->mode == HYBRID_MODE_POSITION_ONLY ? "POSITION" :
           ctrl->mode == HYBRID_MODE_FORCE_ONLY ? "FORCE" : "HYBRID");
    printf("Phase:        %s\n",
           ctrl->state.phase == HYBRID_STATE_IDLE ? "IDLE" :
           ctrl->state.phase == HYBRID_STATE_SWING ? "SWING" :
           ctrl->state.phase == HYBRID_STATE_TOUCHDOWN ? "TOUCHDOWN" :
           ctrl->state.phase == HYBRID_STATE_SUPPORT ? "SUPPORT" : "LIFT");
    printf("Target Pos:   %.2f deg\n", ctrl->params.target_position);
    printf("Target Force: %.2f N\n", ctrl->params.target_force);
    printf("Actual Pos:   %.2f deg\n", ctrl->state.current_position);
    printf("Actual Force: %.2f N\n", ctrl->state.current_force);
    printf("Output Torque: %.3f Nm\n", ctrl->state.output_torque);
    printf("Update Count: %u\n", ctrl->state.update_count);
    printf("=====================================================\n\n");
}

void HybridControl_SetPositionGains(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    g_hybrid_controls[motor_id].params.position_kp = kp;
    g_hybrid_controls[motor_id].params.position_ki = ki;
    g_hybrid_controls[motor_id].params.position_kd = kd;
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_SetForceGains(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);
    g_hybrid_controls[motor_id].params.force_kp = kp;
    g_hybrid_controls[motor_id].params.force_ki = ki;
    g_hybrid_controls[motor_id].params.force_kd = kd;
    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_Reset(uint8_t motor_id)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return;

    osMutexAcquire(g_hybrid_mutex, osWaitForever);

    HybridControl_t *ctrl = &g_hybrid_controls[motor_id];
    ctrl->state.position_integral = 0.0f;
    ctrl->state.force_integral = 0.0f;
    ctrl->state.position_error = 0.0f;
    ctrl->state.position_error_prev = 0.0f;
    ctrl->state.force_error = 0.0f;
    ctrl->state.force_error_prev = 0.0f;
    ctrl->state.output_torque = 0.0f;
    ctrl->state.update_count = 0;

    osMutexRelease(g_hybrid_mutex);
}

void HybridControl_ResetAll(void)
{
    for (int i = 0; i < HYBRID_MAX_MOTORS; i++) {
        HybridControl_Reset(i);
    }
}

uint8_t HybridControl_GetPhase(uint8_t motor_id)
{
    if (motor_id >= HYBRID_MAX_MOTORS) return 0;
    return (uint8_t)g_hybrid_controls[motor_id].state.phase;
}
