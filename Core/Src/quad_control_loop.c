/**
 ******************************************************************************
 * @file    quad_control_loop.c
 * @brief   四足机器狗高精度控制循环实现 (200Hz + 硬件紧急停止)
 * @version V2.0 - 工业级优化版本
 * @note    全局变量封装到结构体，提高模块化程度和可维护性
 ******************************************************************************
 */

#include "quad_control_loop.h"
#include "system_config.h"
#include "quad_motor_api.h"
#include "quad_smart_controller.h"
#include "quad_command_fusion.h"
#include "quad_emergency_stop.h"
#include "foot_force_sensor.h"
#include "motor_sync.h"
#include "motor_control.h"
#include "robot_state.h"
#include "log_system.h"
#include "motor_control_timer.h"
#include "trajectory_planner.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "adc.h"
#include <string.h>

/* ============================================================================
 * 私有变量 - 使用封装结构体
 * ============================================================================ */

static QuadControlLoopContext_t g_ctx = {0};
static osThreadId_t g_control_task_handle = NULL;
static osMutexId_t g_status_mutex = NULL;
static osMutexId_t g_angle_mutex = NULL;
static volatile uint32_t g_target_angles[QUAD_MAX_MOTORS] = {0};
static volatile bool g_new_target_received = false;

/* V12.0: 启动保护期 - 避免系统启动时误触发TCP超时 */
static uint32_t g_system_start_tick = 0;
#define TCP_STARTUP_GRACE_PERIOD_MS  8000  /* 启动后8秒内不触发TCP超时检测 */

/* ============================================================================
 * 私有函数前向声明
 * ============================================================================ */

static void control_loop_task(void *argument);
static void sync_control_state_to_robot_state(void);
static void target_buffer_write(const float* target_angles);
static bool target_buffer_read_interpolated(float* output);
static void target_buffer_reset(void);

/* ============================================================================
 * 初始化函数
 * ============================================================================ */

void QuadControlLoop_Init(void)
{
    LOG_INFO("[ControlLoop] Initializing 200Hz control loop (V2.0)...");
    
    memset(&g_ctx, 0, sizeof(g_ctx));
    
    g_ctx.state = CONTROL_STATE_IDLE;
    g_ctx.emergency_stop_flag = false;
    g_ctx.use_interpolation = true;
    g_ctx.startup_phase = true;  /* 启动阶段抑制CAN发送，直到QuadMotor_Start完成回零 */
    
    g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_NORMAL;
    g_ctx.tcp.last_receive_tick = HAL_GetTick();
    
    g_ctx.degradation.quality = CONTROL_QUALITY_HIGH;
    g_ctx.degradation.effective_period_ms = SYSTEM_CONTROL_PERIOD_MS;
    
    /* V3.0: 初始化闭环积分器参数 */
    g_ctx.closed_loop.integral_limit = 5.0f;
    g_ctx.closed_loop.integral_enabled = true;
    
    osMutexAttr_t mutex_attr = {
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    g_status_mutex = osMutexNew(&mutex_attr);
    g_angle_mutex = osMutexNew(&mutex_attr);

    Trajectory_EnableContinuousMode(true);

    /* V4.0: 不设置电机内部PID参数 - 电机内部已有完善的PID系统 */
    /* 本系统只进行指令控制：位置+速度前馈 */

    LOG_INFO("[ControlLoop] Control loop initialized (V2.0)");
}

/* ============================================================================
 * 双缓冲区操作函数
 * ============================================================================ */

static void target_buffer_write(const float* target_angles)
{
    if (target_angles == NULL) return;

    uint8_t write_idx = g_ctx.target_buffer.write_idx;
    uint32_t current_tick = HAL_GetTick();

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        float new_target = target_angles[i];

#if INTERP_ERROR_CORRECTION_ENABLE
        if (g_ctx.target_buffer.first_data_received[i]) {
            float expected_target = g_ctx.target_buffer.last_raw_target[i];
            float error = new_target - expected_target;
            float abs_error = fabsf(error);

            if (abs_error > INTERP_ERROR_THRESHOLD_DEG) {
                float correction = CLAMP(error, -INTERP_ERROR_MAX_CORRECTION_DEG, INTERP_ERROR_MAX_CORRECTION_DEG);
                g_ctx.target_buffer.interp_error_accum[i] += correction * 0.5f;
                g_ctx.target_buffer.interp_error_accum[i] = CLAMP(g_ctx.target_buffer.interp_error_accum[i],
                                                              -INTERP_ERROR_MAX_CORRECTION_DEG * 2,
                                                              INTERP_ERROR_MAX_CORRECTION_DEG * 2);
            }
        } else {
            g_ctx.target_buffer.first_data_received[i] = true;
        }

        g_ctx.target_buffer.last_raw_target[i] = new_target;
        new_target += g_ctx.target_buffer.interp_error_accum[i] * 0.1f;
        g_ctx.target_buffer.interp_error_accum[i] *= INTERP_ERROR_DECAY;
#endif

        g_ctx.target_buffer.buffer[i][write_idx] = new_target;
    }

    g_ctx.target_buffer.write_idx = (write_idx + 1) % TARGET_BUFFER_SIZE;

    if (g_ctx.target_buffer.count < TARGET_BUFFER_SIZE) {
        g_ctx.target_buffer.count++;
    }

#if INTERP_ERROR_CORRECTION_ENABLE
    if ((current_tick - g_ctx.target_buffer.last_sync_tick) > INTERP_SYNC_INTERVAL_MS) {
        for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
            g_ctx.target_buffer.interp_error_accum[i] *= 0.5f;
        }
        g_ctx.target_buffer.last_sync_tick = current_tick;
    }
#endif
}

static bool target_buffer_read_interpolated(float* output)
{
    if (output == NULL) return false;

    if (g_ctx.target_buffer.count < 2) {
        uint32_t current_tick = HAL_GetTick();
        
        g_ctx.target_buffer.underflow_detected = true;
        g_ctx.target_buffer.underflow_count++;
        
        if (current_tick - g_ctx.target_buffer.last_underflow_tick > 1000) {
            LOG_WARN("[ControlLoop] Buffer underflow! count=%d, underflow_count=%u", 
                     g_ctx.target_buffer.count, g_ctx.target_buffer.underflow_count);
            g_ctx.target_buffer.last_underflow_tick = current_tick;
        }
        
        /* V3.1: 长时间欠载安全措施 */
        #define UNDERFLOW_DEGRADE_THRESHOLD  100   /* 100次欠载(500ms)降级 */
        #define UNDERFLOW_CRITICAL_THRESHOLD 400   /* 400次欠载(2秒)急停 */
        
        if (g_ctx.target_buffer.underflow_count >= UNDERFLOW_CRITICAL_THRESHOLD) {
            LOG_ERROR("[ControlLoop] Critical underflow (%u), emergency stop!", 
                      g_ctx.target_buffer.underflow_count);
            QuadControlLoop_SetEmergencyStop(true);
            g_ctx.degradation.quality = CONTROL_QUALITY_CRITICAL;
            g_ctx.target_buffer.underflow_count = 0;  /* V3.2: 急停后重置计数器，避免重复触发 */
        } else if (g_ctx.target_buffer.underflow_count >= UNDERFLOW_DEGRADE_THRESHOLD) {
            if (g_ctx.degradation.quality != CONTROL_QUALITY_LOW) {
                LOG_WARN("[ControlLoop] Prolonged underflow, degrading control quality");
                g_ctx.degradation.quality = CONTROL_QUALITY_LOW;
            }
        }
        
        for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
            output[i] = g_ctx.target_buffer.last_interpolated[i];
        }
        return g_ctx.target_buffer.count > 0;
    }
    
    if (g_ctx.target_buffer.underflow_detected) {
        g_ctx.target_buffer.underflow_detected = false;
        g_ctx.target_buffer.underflow_count = 0;  /* V3.1: 恢复时重置计数 */
        if (g_ctx.degradation.quality != CONTROL_QUALITY_HIGH) {
            LOG_INFO("[ControlLoop] Buffer recovered, restoring control quality");
            g_ctx.degradation.quality = CONTROL_QUALITY_HIGH;
        } else {
            LOG_INFO("[ControlLoop] Buffer recovered");
        }
    }

    uint8_t read_idx = g_ctx.target_buffer.read_idx;
    uint8_t next_idx = (read_idx + 1) % TARGET_BUFFER_SIZE;
    float t = (float)g_ctx.target_buffer.interpolation_step / INTERPOLATION_STEPS;

    static float g_vel_estimate[QUAD_MAX_MOTORS] = {0};
    static float g_accel_estimate[QUAD_MAX_MOTORS] = {0};
    static float g_last_p1[QUAD_MAX_MOTORS] = {0};
    static float g_last_p0[QUAD_MAX_MOTORS] = {0};

    /* 预计算常量 */
    const float PREDICTION_BASE = 0.005f;
    const float PREDICTION_SCALE = 0.010f;
    const float PREDICTION_GAIN = 0.5f;
    const float ACCEL_GAIN = 0.15f;
    const float VEL_FILTER_ALPHA = 0.6f;
    const float VEL_FILTER_BETA = 0.4f;
    const float ACCEL_FILTER_ALPHA = 0.4f;
    const float ACCEL_FILTER_BETA = 0.6f;
    const float VEL_THRESHOLD = 100.0f;
    const float MAX_GAIN = 1.5f;
    const float GAIN_SCALE = 0.003f;

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        float p0 = g_ctx.target_buffer.buffer[i][read_idx];
        float p1 = g_ctx.target_buffer.buffer[i][next_idx];

        if (g_ctx.target_buffer.interpolation_step == 0) {
            float instant_vel = (p1 - p0) * 50.0f;
            g_vel_estimate[i] = VEL_FILTER_ALPHA * instant_vel + VEL_FILTER_BETA * g_vel_estimate[i];
            
            float instant_accel = (p1 - 2.0f * g_last_p1[i] + g_last_p0[i]) * 2500.0f;
            g_accel_estimate[i] = ACCEL_FILTER_ALPHA * instant_accel + ACCEL_FILTER_BETA * g_accel_estimate[i];
            
            g_last_p0[i] = g_last_p1[i];
            g_last_p1[i] = p1;
        }

        float linear_interp = p0 + (p1 - p0) * t;
        float prediction_time = PREDICTION_BASE + t * PREDICTION_SCALE;
        float predicted_offset = g_vel_estimate[i] * prediction_time;
        float accel_compensation = 0.5f * g_accel_estimate[i] * prediction_time * prediction_time;

        float adaptive_gain = 1.0f;
        float vel_magnitude = fabsf(g_vel_estimate[i]);
        if (vel_magnitude > VEL_THRESHOLD) {
            adaptive_gain = MIN(MAX_GAIN, 1.0f + GAIN_SCALE * (vel_magnitude - VEL_THRESHOLD));
        }

        output[i] = linear_interp + predicted_offset * adaptive_gain * PREDICTION_GAIN + accel_compensation * ACCEL_GAIN;
        output[i] = CLAMP(output[i], 0.0f, 359.99f);

        g_ctx.target_buffer.last_interpolated[i] = output[i];
    }

    g_ctx.target_buffer.interpolation_step++;

    if (g_ctx.target_buffer.interpolation_step >= INTERPOLATION_STEPS) {
        g_ctx.target_buffer.interpolation_step = 0;
        g_ctx.target_buffer.read_idx = next_idx;
        if (g_ctx.target_buffer.count > 0) {
            g_ctx.target_buffer.count--;
        }
    }

    return true;
}

static void target_buffer_reset(void)
{
    g_ctx.target_buffer.write_idx = 0;
    g_ctx.target_buffer.read_idx = 0;
    g_ctx.target_buffer.count = 0;
    g_ctx.target_buffer.interpolation_step = 0;
    g_ctx.target_buffer.last_sync_tick = 0;

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        g_ctx.target_buffer.last_interpolated[i] = 0.0f;
        g_ctx.target_buffer.interp_error_accum[i] = 0.0f;
        g_ctx.target_buffer.last_raw_target[i] = 0.0f;
        g_ctx.target_buffer.first_data_received[i] = false;
    }
}

/* ============================================================================
 * 控制循环主任务
 * ============================================================================ */

static void control_loop_task(void *argument)
{
    (void)argument;

    static bool last_estop_state = false;

    osSemaphoreId_t timer_sem = motor_ctrl_timer_get_semaphore();
    if (timer_sem == NULL) {
        LOG_ERROR("[ControlLoop] Failed to get timer semaphore!");
        return;
    }

    uint32_t tick_start, tick_end, exec_time;
    uint32_t frequency_calc_counter = 0;
    uint32_t last_frequency_calc_tick = 0;

    /* V12.0: 记录系统启动时间，用于启动保护期 */
    g_system_start_tick = HAL_GetTick();
    LOG_INFO("[ControlLoop] 200Hz control task started (TIM6 triggered)");

    uint32_t fdcan_check_counter = 0;

    while (1) {
        if (osSemaphoreAcquire(timer_sem, osWaitForever) != osOK) {
            LOG_WARN("[ControlLoop] Semaphore acquire failed, retrying...");
            continue;
        }

        motor_exec_process_timeout();

        fdcan_check_counter++;
        if (fdcan_check_counter >= 200) {
            fdcan_check_and_recover();
            fdcan_check_counter = 0;
        }

        tick_start = motor_ctrl_timer_get_us();

        if (g_ctx.emergency_stop_flag) {
            g_ctx.state = CONTROL_STATE_EMERGENCY_STOP;
            motor_sync_emergency_stop_all(NULL);
            robot_state_emergency_stop();
            sync_control_state_to_robot_state();
            
            for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
                g_ctx.closed_loop.compensation[i] = 0.0f;
            }
            
            LOG_ERROR("[ControlLoop] EMERGENCY STOP ACTIVATED!");
            continue;
        }

        bool current_estop_state = QuadEmergencyStop_IsActive();

        if (current_estop_state && !last_estop_state) {
            QuadEmergencyStop_SetTCPSafeStop(true);
            LOG_INFO("[ControlLoop] Emergency stop activated, TCP status synced");
        } else if (!current_estop_state && last_estop_state) {
            QuadEmergencyStop_ClearAllTCPFlags();
            LOG_INFO("[ControlLoop] Emergency stop cleared, TCP status synced");
        }
        last_estop_state = current_estop_state;

        if (g_ctx.state != CONTROL_STATE_RUNNING) {
            continue;
        }

        uint32_t current_tick = HAL_GetTick();
        uint32_t tcp_elapsed = (current_tick >= g_ctx.tcp.last_receive_tick) ?
                               (current_tick - g_ctx.tcp.last_receive_tick) :
                               (0xFFFFFFFF - g_ctx.tcp.last_receive_tick + current_tick + 1);

        /* V12.0: 启动保护期检查 - 避免系统启动时误触发TCP超时 */
        uint32_t system_uptime = current_tick - g_system_start_tick;
        bool in_startup_grace_period = (system_uptime < TCP_STARTUP_GRACE_PERIOD_MS);

        if (!g_ctx.tcp.connected && tcp_elapsed > TCP_RX_TIMEOUT_MS && !in_startup_grace_period) {
            if (g_ctx.tcp.recovery_phase == TCP_RECOVERY_PHASE_NORMAL) {
                g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_DISCONNECTED;
                LOG_INFO("[ControlLoop] TCP state machine: entering DISCONNECTED phase");
            }
            
            if (!QuadEmergencyStop_IsTCPTimeout()) {
                QuadEmergencyStop_SetTCPTimeout(true);
                g_ctx.tcp.reconnect_count = 0;
                LOG_WARN("[ControlLoop] TCP DISCONNECTED warning! %lu ms", (unsigned long)tcp_elapsed);
            }

            /* V4.0: 无外设模式下跳过急停触发 */
            if (!QuadEmergencyStop_IsNoPeripheralMode()) {
                if (tcp_elapsed > TCP_HOLD_TIMEOUT_MS && !QuadEmergencyStop_IsTCPHold()) {
                    QuadEmergencyStop_SetTCPHold(true);
                    LOG_WARN("[ControlLoop] TCP disconnected: HOLD position!");
                }

                if (tcp_elapsed > TCP_EMERGENCY_TIMEOUT_MS && !QuadEmergencyStop_IsTCPSafeStop()) {
                    QuadEmergencyStop_SetTCPSafeStop(true);
                    LOG_ERROR("[ControlLoop] TCP disconnected: SAFE STOP!");
                }
            } else {
                /* 无外设模式：仅记录日志，不触发急停 */
                if (tcp_elapsed > TCP_EMERGENCY_TIMEOUT_MS) {
                    static uint32_t last_no_periph_log = 0;
                    if ((current_tick - last_no_periph_log) > 5000) {
                        LOG_INFO("[ControlLoop] No-peripheral mode: TCP timeout ignored (%lu ms)", 
                               (unsigned long)tcp_elapsed);
                        last_no_periph_log = current_tick;
                    }
                }
            }
        }

        /* V12.0: 启动保护期内输出提示 */
        if (in_startup_grace_period && !g_ctx.tcp.connected) {
            static uint32_t startup_log_tick = 0;
            if ((current_tick - startup_log_tick) > 5000) {
                LOG_INFO("[ControlLoop] TCP startup grace period: %lu ms remaining", 
                       (unsigned long)(TCP_STARTUP_GRACE_PERIOD_MS - system_uptime));
                startup_log_tick = current_tick;
            }
        }

        if (QuadEmergencyStop_IsTCPTimeout() && (g_ctx.tcp.connected || tcp_elapsed <= TCP_RX_TIMEOUT_MS)) {
            if (QuadEmergencyStop_IsTCPSafeStop()) {
                if (g_ctx.tcp.connected) {
                    QuadEmergencyStop_ClearAllTCPFlags();
                    LOG_INFO("[ControlLoop] TCP reconnected, SAFE STOP cleared");
                }
            } else {
                QuadEmergencyStop_ClearAllTCPFlags();
                LOG_INFO("[ControlLoop] TCP connection restored");
            }
        }

        osMutexAcquire(g_angle_mutex, osWaitForever);
        float target_angles[QUAD_MAX_MOTORS];
        for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
            target_angles[i] = (float)g_target_angles[i] / 100.0f;
        }
        bool new_data = g_new_target_received;
        g_new_target_received = false;
        osMutexRelease(g_angle_mutex);

        if (new_data) {
            if (QuadEmergencyStop_IsTCPSafeStop() && !g_ctx.tcp.connected) {
            } else {
                if (QuadEmergencyStop_IsTCPTimeout() || QuadEmergencyStop_IsTCPHold() || QuadEmergencyStop_IsTCPSafeStop()) {
                    g_ctx.tcp.reconnect_count++;
                    g_ctx.tcp.recovery_reconnect_count++;
                    g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_RECONNECTING;
                    g_ctx.tcp.recovery_start_tick = current_tick;
                    LOG_INFO("[ControlLoop] TCP restored! Starting recovery (count=%u)", g_ctx.tcp.reconnect_count);
                }
                QuadEmergencyStop_ClearAllTCPFlags();
                g_ctx.tcp.last_receive_tick = current_tick;
                g_ctx.log_counter++;

                g_ctx.packet_loss.total_received++;
                g_ctx.packet_loss.lost_count = 0;
                g_ctx.packet_loss.loss_detected = false;
                
                g_ctx.degradation.upgrade_stable_count++;
                if (g_ctx.degradation.upgrade_stable_count >= CONTROL_UPGRADE_STABLE_COUNT &&
                    g_ctx.degradation.quality > CONTROL_QUALITY_HIGH) {
                    g_ctx.degradation.quality--;
                    g_ctx.degradation.upgrade_stable_count = 0;
                    LOG_INFO("[ControlLoop] Control quality upgraded to %d", g_ctx.degradation.quality);
                }

                for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
                    Trajectory_SetContinuousTarget(i, target_angles[i]);
                }

                if (g_ctx.log_counter >= 200) {
                    LOG_DEBUG("[ControlLoop] Target updated");
                    g_ctx.log_counter = 0;
                }
            }
        }

        float speed_limit_factor = 1.0f;
        
        switch (g_ctx.tcp.recovery_phase) {
            case TCP_RECOVERY_PHASE_RECONNECTING:
                speed_limit_factor = 0.0f;
                if (current_tick - g_ctx.tcp.recovery_start_tick > TCP_HOLD_DURATION_MS) {
                    g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_SOFT_START;
                    g_ctx.tcp.recovery_start_tick = current_tick;
                    LOG_INFO("[ControlLoop] Recovery: entering soft-start phase");
                }
                break;
                
            case TCP_RECOVERY_PHASE_SOFT_START:
                {
                    uint32_t elapsed = current_tick - g_ctx.tcp.recovery_start_tick;
                    if (elapsed < TCP_SOFT_START_DURATION_MS) {
                        speed_limit_factor = (float)elapsed / TCP_SOFT_START_DURATION_MS;
                        speed_limit_factor = speed_limit_factor * speed_limit_factor;
                    } else {
                        speed_limit_factor = 1.0f;
                        g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_NORMAL;
                        LOG_INFO("[ControlLoop] Recovery: normal operation restored");
                    }
                }
                break;
                
            default:
                speed_limit_factor = 1.0f;
                break;
        }

        {
            TrajectoryOutput_t traj_outputs[TRAJECTORY_MAX_MOTORS];
            Trajectory_StepAll(traj_outputs);

            g_ctx.traj_debug_counter++;
            
            {
#if CLOSED_LOOP_ENABLED
                /* 闭环模式：发送0x31F监测请求，获取电机实时反馈 */
                for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
                    /* ✅ V2.1: 每条CAN总线内部独立错峰，避免CAN2监测频率低于CAN1 */
                    uint8_t local_idx = (i < 6) ? i : (i - 6);
                    uint32_t stagger_tick = local_idx * CAN_MONITOR_STAGGER_OFFSET_MS;
                    uint32_t interval = CAN_MONITOR_REQUEST_INTERVAL_MS + stagger_tick;

                    if ((current_tick - g_ctx.last_monitor_request_tick[i]) >= interval) {
                        uint8_t can_channel = (i < 6) ? 1 : 2;
                        uint8_t device_id = i + 1;
                        motor_read_monitor_data(can_channel, device_id, NULL);
                        g_ctx.last_monitor_request_tick[i] = current_tick;
                    }
                }
#endif /* CLOSED_LOOP_ENABLED */

                /* 电机反馈处理与闭环修正 */
#if CLOSED_LOOP_ENABLED
                for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
                    QuadMotorStatus_t motor_status;
                    if (QuadMotor_GetStatus(i, &motor_status) == 0) {
                        /* 批量更新反馈数据 */
                        g_ctx.motor_feedback[i].actual_position = motor_status.position;
                        g_ctx.motor_feedback[i].actual_velocity = motor_status.velocity;
                        g_ctx.motor_feedback[i].actual_torque = motor_status.torque;
                        g_ctx.motor_feedback[i].last_update_tick = HAL_GetTick();

                        TrajectoryPlanner_t planner;
                        Trajectory_GetState(i, &planner);

                        float pos_error = motor_status.position - planner.current_position;
                        float abs_error = fabsf(pos_error);

                        /* 计算速度误差 */
                        float actual_vel_deg_s = motor_status.velocity * 6.0f;
                        float planned_vel_deg_s = planner.current_velocity;
                        float vel_error = actual_vel_deg_s - planned_vel_deg_s;
                        g_ctx.motor_feedback[i].position_error = pos_error;
                        g_ctx.motor_feedback[i].velocity_error = vel_error;

                        /* 死区判断 */
                        if (abs_error < CLOSED_LOOP_ERROR_DEADZONE_DEG) {
                            continue;
                        }

                        /* V2.1改进：渐进式闭环修正，避免位置跳变 */
                        /* 根据运动状态动态调整修正因子 */
                        float base_correction_factor;
                        uint8_t joint_type = (i % 3);
                        bool is_thigh = (joint_type == 0);

                        if (is_thigh) {
                            if (abs_error > 25.0f) {
                                base_correction_factor = CORRECTION_FACTOR_THIGH_LARGE;
                            } else if (abs_error > 15.0f) {
                                base_correction_factor = CORRECTION_FACTOR_THIGH_MEDIUM;
                            } else {
                                base_correction_factor = CORRECTION_FACTOR_THIGH_SMALL;
                            }
                        } else {
                            if (abs_error > 25.0f) {
                                base_correction_factor = CORRECTION_FACTOR_LARGE;
                            } else if (abs_error > 15.0f) {
                                base_correction_factor = CORRECTION_FACTOR_MEDIUM;
                            } else {
                                base_correction_factor = CORRECTION_FACTOR_SMALL;
                            }
                        }

                        /* 运动状态下降低修正幅度，保持积分器一致性 */
                        float motion_factor = 1.0f;
                        if (planner.is_moving && planned_vel_deg_s > 10.0f) {
                            /* 高速运动时降低修正幅度到30% */
                            motion_factor = 0.3f;
                        } else if (planner.is_moving) {
                            /* 低速运动时降低修正幅度到60% */
                            motion_factor = 0.6f;
                        }

                        float correction = pos_error * base_correction_factor * motion_factor;

                        /* V3.1: 优化积分实现 - 积分误差而非修正值 */
                        #define INTEGRAL_GAIN 0.005f
                        #define INTEGRAL_LIMIT 3.0f
                        #define INTEGRAL_DECAY 0.90f

                        if (g_ctx.closed_loop.integral_enabled) {
                            /* 积分位置误差，符号与pos_error一致 */
                            float signed_error = planner.target_position - planner.current_position;
                            g_ctx.closed_loop.integral[i] += signed_error * INTEGRAL_GAIN;
                            g_ctx.closed_loop.integral[i] = CLAMP(g_ctx.closed_loop.integral[i],
                                                                   -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

                            /* 到达目标时衰减积分 */
                            if (abs_error < 0.5f && fabsf(actual_vel_deg_s) < 5.0f) {
                                g_ctx.closed_loop.integral[i] *= INTEGRAL_DECAY;
                            }

                            /* 积分项加到修正值上 */
                            correction += g_ctx.closed_loop.integral[i];
                        }

                        /* 限制单次修正最大幅度，避免位置跳变 */
                        const float MAX_SINGLE_CORRECTION = 2.0f;  /* 单次最大修正2度 */
                        if (fabsf(correction) > MAX_SINGLE_CORRECTION) {
                            correction = (correction > 0) ? MAX_SINGLE_CORRECTION : -MAX_SINGLE_CORRECTION;
                        }

                        /* 应用渐进式修正 */
                        float corrected_pos = planner.current_position + correction;
                        Trajectory_SetCurrentPosition(i, corrected_pos);
                    }
                }
#endif /* CLOSED_LOOP_ENABLED */
            }
            
            /* 优化：电机控制和CAN发送 */
            for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
                TrajectoryPlanner_t planner;
                Trajectory_GetState(i, &planner);
                
                float pos_error = fabsf(planner.target_position - planner.current_position);
                
                uint8_t can_channel = (i < 6) ? 1 : 2;
                uint8_t device_id = i + 1;
                CANSendState_t* can_state = &g_ctx.can_send_state[i];
                
                /* V3.0: 使用状态机重构CAN发送决策 */
                bool is_accelerating = (planner.state == TRAJ_STATE_ACCELERATING);
                bool is_decelerating = (planner.state == TRAJ_STATE_DECELERATING);
                
                static bool last_direction[QUAD_MAX_MOTORS] = {true};
                bool direction_changing = (last_direction[i] != traj_outputs[i].direction && 
                                           traj_outputs[i].velocity < 30.0f);
                last_direction[i] = traj_outputs[i].direction;
                
                bool near_extreme_point = (traj_outputs[i].velocity < 20.0f && 
                                           traj_outputs[i].acceleration > 100.0f);
                bool is_stable = (pos_error < 0.3f && fabsf(traj_outputs[i].velocity) < 1.0f);
                
                CANSendStateEnum_t new_state = can_state->state;
                
                /* V3.2: 极值点检测优先级提高，确保极值点附近高频率发送 */
                if (near_extreme_point) {
                    new_state = CAN_SEND_STATE_NEAR_EXTREME;
                } else if (is_accelerating) {
                    new_state = CAN_SEND_STATE_ACCELERATING;
                } else if (is_decelerating) {
                    new_state = CAN_SEND_STATE_DECELERATING;
                } else if (direction_changing) {
                    new_state = CAN_SEND_STATE_DIRECTION_CHANGE;
                } else if (is_stable) {
                    new_state = CAN_SEND_STATE_STABLE;
                } else {
                    new_state = CAN_SEND_STATE_NORMAL;
                }
                
                if (new_state != can_state->state) {
                    /* V3.1: 状态切换日志 */
                    static const char* state_names[] = {
                        "NORMAL", "ACCEL", "DECEL", "DIR_CHG", "EXTREME", "STABLE"
                    };
                    LOG_DEBUG("[CAN] Motor %d state: %s -> %s", i, 
                              state_names[can_state->state], state_names[new_state]);
                    can_state->state = new_state;
                    can_state->state_enter_tick = current_tick;
                }
                
                uint32_t send_interval;
                switch (can_state->state) {
                    case CAN_SEND_STATE_ACCELERATING:
                    case CAN_SEND_STATE_DECELERATING:
                        send_interval = CAN_SEND_ACCEL_INTERVAL_MS;
                        break;
                    case CAN_SEND_STATE_DIRECTION_CHANGE:
                    case CAN_SEND_STATE_NEAR_EXTREME:
                        send_interval = 2;
                        break;
                    case CAN_SEND_STATE_STABLE:
                        send_interval = CAN_SEND_MAX_INTERVAL_MS * 2;
                        break;
                    default:
                        send_interval = CAN_SEND_MAX_INTERVAL_MS;
                }
                
                float pos_delta = fabsf(traj_outputs[i].position - can_state->last_sent_position);
                float vel_delta = fabsf(traj_outputs[i].velocity - can_state->last_sent_velocity);
                uint32_t time_delta = current_tick - can_state->last_send_tick;
                
                bool should_send = (pos_delta > CAN_SEND_POS_THRESHOLD_DEG ||
                                   vel_delta > CAN_SEND_VEL_THRESHOLD_RPM ||
                                   time_delta >= send_interval);

                /* V5.0: 启动阶段抑制CAN发送，等待QuadMotor_Start完成回零 */
                if (g_ctx.startup_phase) {
                    should_send = false;
                }

                if (should_send) {
                    float compensation = 0.0f;
#if CLOSED_LOOP_ENABLED
                    EnhancedMotorFeedback_t *fb = &g_ctx.motor_feedback[i];

                    /* 预测修正 */
                    #if PREDICTIVE_CORRECTION
                    {
                        float current_vel_deg_s = fb->actual_velocity * 6.0f;
                        float future_displacement = current_vel_deg_s * (PREDICTION_HORIZON_MS / 1000.0f);
                        compensation += future_displacement * PREDICTION_FACTOR;
                    }
                    #endif

                    /* 摩擦补偿 */
                    #if FRICTION_COMPENSATION
                    {
                        float abs_velocity = fabsf(fb->actual_velocity);
                        if (abs_velocity > 1.0f && abs_velocity < 20.0f) {
                            float friction_comp = FRICTION_COMPENSATION_GAIN * (20.0f - abs_velocity) * 0.05f;
                            float direction = (fb->actual_velocity > 0) ? 1.0f : -1.0f;
                            compensation += direction * friction_comp;
                        }
                    }
                    #endif

                    /* 转矩补偿 */
                    #if TORQUE_COMPENSATION
                    {
                        float abs_torque = fabsf(fb->actual_torque);
                        if (abs_torque > 0.3f) {
                            float torque_comp = fb->actual_torque * TORQUE_COMPENSATION_GAIN;
                            compensation += torque_comp;
                        }
                    }
                    #endif
#endif /* CLOSED_LOOP_ENABLED */
                    
                    /* 前馈补偿 */
                    float ff_vel = traj_outputs[i].feedforward_vel;
                    float ff_accel = traj_outputs[i].feedforward_accel;
                    
                    float distance_to_target = fabsf(planner.target_position - planner.current_position);
                    if (distance_to_target < ADAPTIVE_FF_DISTANCE_THRESHOLD) {
                        ff_vel *= ADAPTIVE_FF_REDUCTION_FACTOR;
                        ff_accel *= ADAPTIVE_FF_REDUCTION_FACTOR;
                    }
                    
                    /* 低通滤波 */
                    g_ctx.ff_filter.filtered_vel[i] = FF_FILTER_ALPHA * g_ctx.ff_filter.filtered_vel[i] + 
                                                     (1.0f - FF_FILTER_ALPHA) * ff_vel;
                    g_ctx.ff_filter.filtered_accel[i] = FF_FILTER_ALPHA * g_ctx.ff_filter.filtered_accel[i] + 
                                                       (1.0f - FF_FILTER_ALPHA) * ff_accel;
                    float ff_comp = g_ctx.ff_filter.filtered_vel[i] + g_ctx.ff_filter.filtered_accel[i];
                    
                    float direction_sign = traj_outputs[i].direction ? 1.0f : -1.0f;
                    ff_comp *= direction_sign;
                    compensation += ff_comp;

#if CLOSED_LOOP_ENABLED
                    /* 闭环补偿 */
                    compensation += g_ctx.closed_loop.compensation[i];
#endif

                    /* 计算补偿后位置 */
                    float compensated_position = traj_outputs[i].position + compensation;

#if CLOSED_LOOP_ENABLED
                    /* 限制补偿范围 */
                    float max_compensation = CLOSED_LOOP_COMPENSATION_MAX_DEG + 3.0f;
                    compensated_position = CLAMP(compensated_position,
                                                 planner.current_position - max_compensation,
                                                 planner.current_position + max_compensation);
#endif
                    
                    /* 发送位置命令 + 速度前馈 */
                    motor_set_position(can_channel, device_id, compensated_position);

                    /* V4.0: 速度前馈 - 复用上方direction_sign，速度为位移对时间的导数，用于辅助轨迹跟踪 */
                    float velocity_feedforward_rpm = traj_outputs[i].velocity * direction_sign / 6.0f;
                    velocity_feedforward_rpm = CLAMP(velocity_feedforward_rpm, -200.0f, 200.0f);
                    motor_set_speed(can_channel, device_id, velocity_feedforward_rpm);

                    /* 更新发送状态 */
                    can_state->last_sent_position = compensated_position;
                    can_state->last_sent_velocity = traj_outputs[i].velocity;
                    can_state->last_send_tick = current_tick;
                }
            }
        }

        foot_force_update_all((uint16_t*)adc_values);

        tick_end = motor_ctrl_timer_get_us();
        exec_time = tick_end - tick_start;

        osMutexAcquire(g_status_mutex, osWaitForever);
        g_ctx.stats.loop_count++;
        if (exec_time < g_ctx.stats.min_execution_us) g_ctx.stats.min_execution_us = exec_time;
        if (exec_time > g_ctx.stats.max_execution_us) g_ctx.stats.max_execution_us = exec_time;
        g_ctx.stats.total_exec_time += exec_time;
        g_ctx.stats.last_execution_us = exec_time;

        frequency_calc_counter++;
        if (last_frequency_calc_tick == 0) {
            last_frequency_calc_tick = tick_start;
        }
        uint32_t tick_since_last_calc = (tick_start - last_frequency_calc_tick) / 1000;
        if (tick_since_last_calc >= 1000) {
            uint32_t actual_frequency = frequency_calc_counter * 1000 / tick_since_last_calc;
            g_ctx.stats.total_exec_time = g_ctx.stats.total_exec_time / frequency_calc_counter;
            
            /* V12.0: 优化日志输出 - 只在频率异常或每5秒输出一次 */
            static uint32_t log_interval_counter = 0;
            bool freq_abnormal = (actual_frequency < 180 || actual_frequency > 220);
            
            log_interval_counter++;
            if (freq_abnormal || log_interval_counter >= 5) {
                LOG_RAW("[ControlLoop] Freq:%luHz Min:%luus Max:%luus Avg:%luus%s\r\n",
                       (unsigned long)actual_frequency,
                       (unsigned long)g_ctx.stats.min_execution_us,
                       (unsigned long)g_ctx.stats.max_execution_us,
                       (unsigned long)g_ctx.stats.total_exec_time,
                       freq_abnormal ? " [!]" : "");
                log_interval_counter = 0;
            }
            
            frequency_calc_counter = 0;
            last_frequency_calc_tick = tick_start;
            g_ctx.stats.min_execution_us = UINT32_MAX;
            g_ctx.stats.max_execution_us = 0;
        }
        osMutexRelease(g_status_mutex);
    }
}

/* ============================================================================
 * 公共接口函数
 * ============================================================================ */

void QuadControlLoop_Start(void)
{
    if (g_control_task_handle != NULL) {
        LOG_WARN("[ControlLoop] Task already running");
        return;
    }

    const osThreadAttr_t task_attr = {
        .name = "QuadControl",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t)osPriorityHigh,
    };

    g_control_task_handle = osThreadNew(control_loop_task, NULL, &task_attr);
    if (g_control_task_handle == NULL) {
        LOG_ERROR("[ControlLoop] Failed to create control task");
        g_ctx.state = CONTROL_STATE_ERROR;
        sync_control_state_to_robot_state();
        return;
    }

    g_ctx.state = CONTROL_STATE_RUNNING;
    sync_control_state_to_robot_state();
    LOG_INFO("[ControlLoop] Control loop started (200Hz)");
}

void QuadControlLoop_Stop(void)
{
    if (g_control_task_handle != NULL) {
        osThreadTerminate(g_control_task_handle);
        g_control_task_handle = NULL;
    }
    g_ctx.state = CONTROL_STATE_IDLE;
    sync_control_state_to_robot_state();
    LOG_INFO("[ControlLoop] Control loop stopped");
}

void QuadControlLoop_Pause(void)
{
    if (g_ctx.state == CONTROL_STATE_RUNNING) {
        g_ctx.state = CONTROL_STATE_PAUSED;
        sync_control_state_to_robot_state();
        LOG_INFO("[ControlLoop] Control loop paused");
    }
}

void QuadControlLoop_Resume(void)
{
    if (g_ctx.state == CONTROL_STATE_PAUSED) {
        g_ctx.state = CONTROL_STATE_RUNNING;
        sync_control_state_to_robot_state();
        LOG_INFO("[ControlLoop] Control loop resumed");
    }
}

void QuadControlLoop_SetEmergencyStop(bool stopped)
{
    g_ctx.emergency_stop_flag = stopped;
    if (stopped) {
        g_ctx.state = CONTROL_STATE_EMERGENCY_STOP;
        sync_control_state_to_robot_state();
    }
}

bool QuadControlLoop_IsEmergencyStopped(void)
{
    return g_ctx.emergency_stop_flag;
}

uint8_t QuadControlLoop_SetTargetAngles(float target_angles[QUAD_MAX_MOTORS])
{
    if (target_angles == NULL) return 1;

    osMutexAcquire(g_angle_mutex, osWaitForever);
    for (int i = 0; i < QUAD_MAX_MOTORS; i++) {
        g_target_angles[i] = (uint32_t)(target_angles[i] * 100.0f);
    }
    g_new_target_received = true;
    osMutexRelease(g_angle_mutex);

    return 0;
}

void QuadControlLoop_GetStatus(ControlLoopStatus_t *status)
{
    if (status == NULL) return;

    osMutexAcquire(g_status_mutex, osWaitForever);
    status->loop_count = g_ctx.stats.loop_count;
    status->last_execution_us = g_ctx.stats.last_execution_us;
    status->min_execution_us = g_ctx.stats.min_execution_us;
    status->max_execution_us = g_ctx.stats.max_execution_us;
    status->avg_execution_us = (g_ctx.stats.loop_count > 0) ? 
                                (uint32_t)(g_ctx.stats.total_exec_time / g_ctx.stats.loop_count) : 0;
    status->state = g_ctx.state;
    status->emergency_stopped = g_ctx.emergency_stop_flag;
    osMutexRelease(g_status_mutex);
}

void QuadControlLoop_PrintStatus(void)
{
    ControlLoopStatus_t status;
    QuadControlLoop_GetStatus(&status);

    printf("\n======== Control Loop Status =========\n");
    printf("State:           %s\n",
           status.state == CONTROL_STATE_RUNNING ? "RUNNING" :
           status.state == CONTROL_STATE_PAUSED ? "PAUSED" :
           status.state == CONTROL_STATE_EMERGENCY_STOP ? "EMERGENCY STOP" :
           status.state == CONTROL_STATE_ERROR ? "ERROR" : "IDLE");
    printf("Loop Count:      %u\n", status.loop_count);
    printf("Execution Time:  min=%u us, max=%u us, avg=%u us\n",
           status.min_execution_us, status.max_execution_us, status.avg_execution_us);
    printf("Emergency Stop:  %s\n", status.emergency_stopped ? "ACTIVE" : "CLEAR");
    printf("========================================\n\n");
}

uint32_t QuadControlLoop_GetTick(void)
{
    return g_ctx.stats.loop_count;
}

void QuadControlLoop_GetTCPStatus(TCPTimeoutStatus_t *status)
{
    if (status == NULL) return;

    uint32_t current_tick = HAL_GetTick();
    uint32_t tcp_elapsed = (current_tick >= g_ctx.tcp.last_receive_tick) ?
                           (current_tick - g_ctx.tcp.last_receive_tick) :
                           (0xFFFFFFFF - g_ctx.tcp.last_receive_tick + current_tick + 1);

    status->timeout_warning = QuadEmergencyStop_IsTCPTimeout();
    status->hold_position = QuadEmergencyStop_IsTCPHold();
    status->safe_stop = QuadEmergencyStop_IsTCPSafeStop();
    status->elapsed_ms = tcp_elapsed;
    status->tcp_connected = g_ctx.tcp.connected;
}

void QuadControlLoop_ResetTCPTimeout(void)
{
    QuadEmergencyStop_ClearAllTCPFlags();
    g_ctx.tcp.last_receive_tick = HAL_GetTick();
    printf("[ControlLoop] TCP timeout status reset\r\n");
}

void QuadControlLoop_SetTCPConnected(bool connected)
{
    bool old_state = g_ctx.tcp.connected;
    g_ctx.tcp.connected = connected;

    if (!old_state && connected) {
        printf("[ControlLoop] TCP connected!\r\n");
    } else if (old_state && !connected) {
        printf("[ControlLoop] TCP disconnected!\r\n");
    }
}

void QuadControlLoop_GetMotorFeedback(uint8_t motor_id, EnhancedMotorFeedback_t *feedback)
{
    if (motor_id >= QUAD_MAX_MOTORS || feedback == NULL) {
        return;
    }
    
    *feedback = g_ctx.motor_feedback[motor_id];
}

QuadControlLoopContext_t* QuadControlLoop_GetContext(void)
{
    return &g_ctx;
}

void QuadControlLoop_SetStartupPhase(bool active)
{
    g_ctx.startup_phase = active;
    if (!active) {
        LOG_INFO("[ControlLoop] Startup phase ended, CAN sending enabled");
    }
}

static void sync_control_state_to_robot_state(void)
{
}
