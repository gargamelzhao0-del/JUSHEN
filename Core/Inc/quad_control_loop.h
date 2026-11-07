/**
 ******************************************************************************
 * @file    quad_control_loop.h
 * @brief   四足机器狗高精度控制循环 (200Hz + 硬件紧急停止)
 * @version V2.0 - 工业级优化版本
 * @note    全局变量封装到结构体，提高模块化程度
 ******************************************************************************
 */

#ifndef __QUAD_CONTROL_LOOP_H__
#define __QUAD_CONTROL_LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "quad_types.h"
#include "trajectory_planner.h"

/* ============================================================================
 * 控制状态枚举
 * ============================================================================ */

typedef enum {
    CONTROL_STATE_IDLE = 0,
    CONTROL_STATE_RUNNING,
    CONTROL_STATE_PAUSED,
    CONTROL_STATE_EMERGENCY_STOP,
    CONTROL_STATE_ERROR
} ControlState_t;

/* ============================================================================
 * TCP恢复状态枚举
 * ============================================================================ */

typedef enum {
    TCP_RECOVERY_PHASE_NORMAL,      /* 正常运行 */
    TCP_RECOVERY_PHASE_DISCONNECTED,/* 断开状态 */
    TCP_RECOVERY_PHASE_RECONNECTING,/* 重连中 */
    TCP_RECOVERY_PHASE_HOLD,        /* 保持位置 */
    TCP_RECOVERY_PHASE_SOFT_START   /* 软启动恢复 */
} TCPRecoveryPhase_t;

/* ============================================================================
 * 控制质量等级枚举
 * ============================================================================ */

typedef enum {
    CONTROL_QUALITY_HIGH,           /* 高质量: 200Hz, 全功能 */
    CONTROL_QUALITY_MEDIUM,         /* 中等: 100Hz, 简化功能 */
    CONTROL_QUALITY_LOW,            /* 低质量: 50Hz, 基本功能 */
    CONTROL_QUALITY_CRITICAL        /* 关键: 保持位置 */
} ControlQuality_t;

/* ============================================================================
 * 增强型电机反馈结构体
 * ============================================================================ */

typedef struct {
    float actual_position;
    float actual_velocity;
    float actual_torque;
    float position_error;
    float velocity_error;
    uint32_t last_update_tick;
    uint32_t feedback_timestamp;
    uint8_t feedback_valid;
} EnhancedMotorFeedback_t;

/* ============================================================================
 * CAN发送状态机枚举 (V3.0)
 * ============================================================================ */

typedef enum {
    CAN_SEND_STATE_NORMAL,
    CAN_SEND_STATE_ACCELERATING,
    CAN_SEND_STATE_DECELERATING,
    CAN_SEND_STATE_DIRECTION_CHANGE,
    CAN_SEND_STATE_NEAR_EXTREME,
    CAN_SEND_STATE_STABLE
} CANSendStateEnum_t;

/* ============================================================================
 * CAN发送状态结构体
 * ============================================================================ */

typedef struct {
    float last_sent_position;
    float last_sent_velocity;
    uint32_t last_send_tick;
    uint8_t skip_count;
    CANSendStateEnum_t state;
    uint32_t state_enter_tick;
} CANSendState_t;

/* ============================================================================
 * TCP丢包检测结构体
 * ============================================================================ */

typedef struct {
    uint32_t expected_seq;
    uint32_t last_received_seq;
    uint32_t lost_count;
    uint32_t total_lost;
    uint32_t total_received;
    uint32_t last_warn_tick;
    bool loss_detected;
} TCPPacketLoss_t;

/* ============================================================================
 * 控制降级结构体
 * ============================================================================ */

typedef struct {
    ControlQuality_t quality;
    uint32_t downgrade_tick;
    uint32_t upgrade_tick;
    uint8_t downgrade_count;
    uint8_t upgrade_stable_count;
    uint32_t effective_period_ms;
} ControlDegradation_t;

/* ============================================================================
 * 目标缓冲区结构体
 * ============================================================================ */

typedef struct {
    float buffer[QUAD_MAX_MOTORS][TARGET_BUFFER_SIZE];
    uint8_t write_idx;
    uint8_t read_idx;
    int8_t count;
    float last_interpolated[QUAD_MAX_MOTORS];
    uint8_t interpolation_step;
    float interp_error_accum[QUAD_MAX_MOTORS];
    float last_raw_target[QUAD_MAX_MOTORS];
    uint32_t last_sync_tick;
    bool first_data_received[QUAD_MAX_MOTORS];
    uint32_t underflow_count;
    uint32_t last_underflow_tick;
    bool underflow_detected;
} TargetBuffer_t;

/* ============================================================================
 * 闭环控制状态结构体
 * ============================================================================ */

typedef struct {
    float compensation[SYSTEM_MAX_MOTORS];
    float last_pos_error[SYSTEM_MAX_MOTORS];
    float last_vel_error[SYSTEM_MAX_MOTORS];
    uint32_t last_update_tick[SYSTEM_MAX_MOTORS];
    float integral[SYSTEM_MAX_MOTORS];
    float integral_limit;
    bool integral_enabled;
} ClosedLoopState_t;

/* ============================================================================
 * 前馈滤波状态结构体
 * ============================================================================ */

typedef struct {
    float filtered_vel[SYSTEM_MAX_MOTORS];
    float filtered_accel[SYSTEM_MAX_MOTORS];
} FeedforwardFilter_t;

/* ============================================================================
 * 控制循环统计结构体
 * ============================================================================ */

typedef struct {
    uint32_t loop_count;
    uint32_t last_execution_us;
    uint32_t min_execution_us;
    uint32_t max_execution_us;
    uint64_t total_exec_time;
} ControlLoopStats_t;

/* ============================================================================
 * TCP状态结构体
 * ============================================================================ */

typedef struct {
    bool connected;
    uint32_t last_receive_tick;
    TCPRecoveryPhase_t recovery_phase;
    uint32_t recovery_start_tick;
    uint32_t reconnect_count;
    uint32_t recovery_reconnect_count;
} TCPState_t;

/* ============================================================================
 * 控制循环主状态结构体 (封装所有全局状态)
 * ============================================================================ */

typedef struct {
    ControlState_t state;
    bool emergency_stop_flag;
    ControlLoopStats_t stats;
    TargetBuffer_t target_buffer;
    ClosedLoopState_t closed_loop;
    FeedforwardFilter_t ff_filter;
    TCPState_t tcp;
    TCPPacketLoss_t packet_loss;
    ControlDegradation_t degradation;
    EnhancedMotorFeedback_t motor_feedback[SYSTEM_MAX_MOTORS];
    CANSendState_t can_send_state[SYSTEM_MAX_MOTORS];
    /* V4.0: 已删除 last_speed_limit/last_accel_limit - 不再发送速度/加速度限制指令 */
    bool use_interpolation;
    uint32_t log_counter;
    uint32_t last_monitor_request_tick[SYSTEM_MAX_MOTORS];
    uint32_t traj_debug_counter;
    bool startup_phase;  /* V5.0: 启动阶段标志，抑制CAN发送直到回零完成 */
} QuadControlLoopContext_t;

/* ============================================================================
 * 控制循环状态查询结构体
 * ============================================================================ */

typedef struct {
    uint32_t loop_count;
    uint32_t last_execution_us;
    uint32_t min_execution_us;
    uint32_t max_execution_us;
    uint32_t avg_execution_us;
    float actual_frequency_hz;
    ControlState_t state;
    bool emergency_stopped;
} ControlLoopStatus_t;

/* ============================================================================
 * TCP超时状态结构体
 * ============================================================================ */

typedef struct {
    bool timeout_warning;
    bool hold_position;
    bool safe_stop;
    uint32_t elapsed_ms;
    bool tcp_connected;
} TCPTimeoutStatus_t;

/* ============================================================================
 * 公共函数接口
 * ============================================================================ */

void QuadControlLoop_Init(void);
void QuadControlLoop_Start(void);
void QuadControlLoop_Stop(void);
void QuadControlLoop_Pause(void);
void QuadControlLoop_Resume(void);

void QuadControlLoop_SetEmergencyStop(bool stopped);
bool QuadControlLoop_IsEmergencyStopped(void);

uint8_t QuadControlLoop_SetTargetAngles(float target_angles[QUAD_MAX_MOTORS]);

void QuadControlLoop_GetStatus(ControlLoopStatus_t *status);
void QuadControlLoop_PrintStatus(void);

uint32_t QuadControlLoop_GetTick(void);

void QuadControlLoop_GetTCPStatus(TCPTimeoutStatus_t *status);
void QuadControlLoop_ResetTCPTimeout(void);
void QuadControlLoop_SetTCPConnected(bool connected);

void QuadControlLoop_GetMotorFeedback(uint8_t motor_id, EnhancedMotorFeedback_t *feedback);

QuadControlLoopContext_t* QuadControlLoop_GetContext(void);

/* V5.0: 启动阶段控制 */
void QuadControlLoop_SetStartupPhase(bool active);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_CONTROL_LOOP_H__ */
