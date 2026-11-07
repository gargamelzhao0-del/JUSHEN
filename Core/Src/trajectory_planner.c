/**
 ****************************************************************************************************
 * @file        trajectory_planner.c
 * @version     V5.0 - 工业级优化版本
 * @brief       四足机器狗 - 轨迹规划器实现
 * @details     实现平滑的梯形速度曲线规划，确保电机平稳运动
 *              V4.0: 添加预测性轨迹规划，实现连续平滑的轨迹输出
 *              V5.0: 数值稳定性改进，添加NaN/Inf检测，安全除法
 ****************************************************************************************************
 */

#include "trajectory_planner.h"
#include "system_config.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "log_system.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * V5.0: 数值稳定性宏定义（使用system_config.h中的定义）
 * ============================================================================ */

/* 数值范围限制 */
#define TRAJECTORY_MIN_POSITION   0.0f
#define TRAJECTORY_MAX_POSITION   359.99f
#define TRAJECTORY_MAX_VELOCITY   1000.0f    /* deg/s */
#define TRAJECTORY_MAX_ACCEL      5000.0f    /* deg/s^2 */
#define TRAJECTORY_MIN_DT         0.0001f    /* 最小时间步长 */

/* ============================================================================
 * 私有函数前向声明
 * ============================================================================ */
static inline float clamp(float value, float min_val, float max_val);

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 轨迹规划器全局数据 */
typedef struct {
    TrajectoryPlanner_t planners[TRAJECTORY_MAX_MOTORS];
    bool initialized;
    osMutexId_t mutex;
    FeedforwardConfig_t ff_config;
    float vel_integral[TRAJECTORY_MAX_MOTORS];
    float last_pos_error[TRAJECTORY_MAX_MOTORS];
    float filtered_deriv[TRAJECTORY_MAX_MOTORS];
    bool use_backlash_compensation;
    bool use_spline_interpolation;
    bool use_predictive_planning;
    bool continuous_mode_enabled;
    float last_vel_reduction[TRAJECTORY_MAX_MOTORS];
    float last_accel_reduction[TRAJECTORY_MAX_MOTORS];
    uint8_t interpolation_counter[TRAJECTORY_MAX_MOTORS];
    ExtremePointConfig_t extreme_config;
    /* V3.0: 极值点确认机制 */
    struct {
        bool candidate_detected[TRAJECTORY_MAX_MOTORS];
        uint32_t candidate_tick[TRAJECTORY_MAX_MOTORS];
        float candidate_velocity[TRAJECTORY_MAX_MOTORS];
        bool confirmed[TRAJECTORY_MAX_MOTORS];
        uint8_t confirm_count[TRAJECTORY_MAX_MOTORS];
    } extreme_detector;
} TrajectoryGlobal_t;

static TrajectoryGlobal_t g_trajectory_global = {
    .initialized = false,
    .mutex = NULL,
    .ff_config = {
        .vel_ff_gain = 0.85f,
        .accel_ff_gain = 0.25f,
        .pos_kp = 70.0f,
        .vel_kp = 8.0f,
        .vel_ki = 0.18f,
        .vel_kd = 0.9f,
        .integral_limit = 180.0f
    },
    .use_backlash_compensation = true,
    .use_spline_interpolation = true,
    .use_predictive_planning = true,
    .continuous_mode_enabled = false,
    .last_vel_reduction = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .last_accel_reduction = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .interpolation_counter = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .extreme_config = {
        .extreme_distance_near = 2.0f,
        .extreme_distance_mid = 6.0f,
        .extreme_distance_far = 12.0f,
        .vel_reduction_near = 0.4f,
        .vel_reduction_mid = 0.8f,
        .accel_reduction_near = 0.3f,
        .accel_reduction_mid = 0.8f,
        .speed_ratio_threshold = 0.8f
    }
};

/* V11.0: 50Hz优化 - 三次样条插值缓冲区 */
#define SPLINE_BUFFER_SIZE  4   /* 保存最近4个数据点用于样条插值 */

typedef struct {
    float positions[SPLINE_BUFFER_SIZE];    /* 位置历史 */
    uint32_t timestamps[SPLINE_BUFFER_SIZE]; /* 时间戳 */
    uint8_t count;                          /* 有效数据数量 */
    uint8_t index;                          /* 当前写入索引 */
    /* 预计算的样条系数，减少插值时的计算 */
    float spline_coeffs[4];                 /* 样条系数 [a, b, c, d] */
    bool coeffs_valid;                      /* 系数是否有效 */
} SplineBuffer_t;

static SplineBuffer_t g_spline_buffer[TRAJECTORY_MAX_MOTORS] = {0};

/* ============================================================================
 * 私有辅助函数
 * ============================================================================ */

/**
 * @brief 符号函数
 */
static inline float sign(float x)
{
    if (!IS_VALID_FLOAT(x)) return 0.0f;
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * @brief 计算期望速度
 * @param p 规划器状态
 * @param distance 距离目标的距离
 * @param abs_distance 距离的绝对值
 * @param direction 运动方向
 * @param effective_v_max 有效最大速度
 * @param effective_a_max 有效最大加速度
 * @param effective_d_max 有效最大减速度
 * @param extreme_info 极值点信息
 * @param near_direction_change 是否接近方向切换点
 * @param pred_vel 预测速度
 * @param dt 时间步长
 * @return 期望速度
 */
static float calculate_desired_velocity(TrajectoryPlanner_t* p, float distance, float abs_distance, bool direction, float effective_v_max, float effective_a_max, float effective_d_max, ExtremePointInfo_t extreme_info, bool near_direction_change, float pred_vel, float dt)
{
    float desired_velocity = 0.0f;
    
    /* V6.0: 边界保护 - 确保输入参数有效 */
    if (!IS_VALID_FLOAT(abs_distance) || abs_distance < 0.0f) {
        abs_distance = 0.0f;
    }
    if (!IS_VALID_FLOAT(effective_v_max) || effective_v_max <= 0.0f) {
        effective_v_max = TRAJECTORY_DEFAULT_VMAX;
    }
    if (!IS_VALID_FLOAT(effective_a_max) || effective_a_max <= 0.0f) {
        effective_a_max = TRAJECTORY_DEFAULT_AMAX;
    }
    if (!IS_VALID_FLOAT(effective_d_max) || effective_d_max <= 0.0f) {
        effective_d_max = TRAJECTORY_DEFAULT_DMAX;
    }
    if (!IS_VALID_FLOAT(p->current_velocity)) {
        p->current_velocity = 0.0f;
    }
    
    if (abs_distance > p->config.pos_threshold) {
        /* 计算减速距离 */
        float braking_dist = Trajectory_CalcBrakingDistance(fabsf(p->current_velocity), effective_d_max);
        
        /* V6.0: 边界保护 - 确保减速距离有效 */
        if (!IS_VALID_FLOAT(braking_dist) || braking_dist < 0.0f) {
            braking_dist = 0.0f;
        }
        
        /* 正弦波优化：在极值点附近使用更保守的速度策略 */
        if (extreme_info.is_extreme_point || near_direction_change) {
            /* 极值点附近：限制最大速度 */
            effective_v_max = fminf(effective_v_max, TRAJECTORY_EXTREME_POINT_VMAX);
            braking_dist *= TRAJECTORY_BRAKING_SAFETY_FACTOR;  /* 增加安全裕度 */
        }
        
        if (abs_distance > braking_dist + p->config.pos_threshold) {
            /* 还需要继续运动 */
            if (p->current_velocity < effective_v_max) {
                /* 加速阶段 */
                desired_velocity = p->current_velocity + effective_a_max * dt;
                desired_velocity = fminf(desired_velocity, effective_v_max);
            } else {
                /* 匀速阶段 */
                desired_velocity = effective_v_max;
            }
        } else {
            /* 需要减速 */
            desired_velocity = p->current_velocity - effective_d_max * dt;
            desired_velocity = fmaxf(desired_velocity, 0.0f);
        }
        
        /* 预测速度融合 */
        if (p->prediction_valid && fabsf(pred_vel) > 1.0f) {
            /* V6.0: 边界保护 - 确保预测速度有效 */
            if (!IS_VALID_FLOAT(pred_vel)) {
                pred_vel = 0.0f;
            }
            
            float pred_dir = sign(pred_vel);
            float cur_dir = direction ? 1.0f : -1.0f;
            
            if (pred_dir == cur_dir) {
                /* 同向：可以稍微加速 */
                float pred_vel_factor = fminf(fabsf(pred_vel) * 0.15f, effective_v_max * 0.2f);
                desired_velocity = fminf(desired_velocity + pred_vel_factor, effective_v_max);
            } else {
                /* 反向：即将换向，提前减速 */
                desired_velocity *= 0.8f;
            }
        }
    } else {
        desired_velocity = 0.0f;
    }
    
    /* V6.0: 边界保护 - 最终速度范围检查（V13.0: 速度始终为非负）*/
    desired_velocity = clamp(desired_velocity, 0.0f, TRAJECTORY_MAX_VELOCITY);
    
    return desired_velocity;
}

/**
 * @brief 限制值在范围内
 */
static inline float clamp(float value, float min_val, float max_val)
{
    if (!IS_VALID_FLOAT(value)) return (min_val + max_val) / 2.0f;
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief V5.0: 安全的位置限制
 */
static inline float safe_clamp_position(float value)
{
    return CLAMP_VALID(value, TRAJECTORY_MIN_POSITION, TRAJECTORY_MAX_POSITION, 180.0f);
}

/**
 * @brief V5.0: 安全的速度限制
 */
static inline float safe_clamp_velocity(float value)
{
    return CLAMP_VALID(value, -TRAJECTORY_MAX_VELOCITY, TRAJECTORY_MAX_VELOCITY, 0.0f);
}

/**
 * @brief V5.0: 安全的加速度限制
 */
static inline float safe_clamp_accel(float value)
{
    return CLAMP_VALID(value, -TRAJECTORY_MAX_ACCEL, TRAJECTORY_MAX_ACCEL, 0.0f);
}

/**
 * @brief 计算期望加速度并应用Jerk限制
 * @param p 规划器状态
 * @param desired_velocity 期望速度
 * @param effective_a_max 有效最大加速度
 * @param effective_d_max 有效最大减速度
 * @param dt 时间步长
 * @return 期望加速度
 */
static float calculate_desired_acceleration(TrajectoryPlanner_t* p, float desired_velocity, float effective_a_max, float effective_d_max, float dt)
{
    /* V6.0: 边界保护 - 确保dt有效 */
    if (!IS_VALID_FLOAT(dt) || dt < TRAJECTORY_MIN_DT) {
        dt = TRAJECTORY_MIN_DT;
    }
    
    /* V6.0: 边界保护 - 确保速度值有效 */
    if (!IS_VALID_FLOAT(desired_velocity)) {
        desired_velocity = 0.0f;
    }
    if (!IS_VALID_FLOAT(p->current_velocity)) {
        p->current_velocity = 0.0f;
    }
    
    float desired_accel = (desired_velocity - p->current_velocity) / dt;
    
    /* V6.0: 边界保护 - 确保加速度值有效 */
    if (!IS_VALID_FLOAT(desired_accel)) {
        desired_accel = 0.0f;
    }
    
    desired_accel = clamp(desired_accel, -effective_d_max, effective_a_max);
    
    float accel_diff = desired_accel - p->current_acceleration;
    
    /* V6.0: 边界保护 - 确保加速度差值有效 */
    if (!IS_VALID_FLOAT(accel_diff)) {
        accel_diff = 0.0f;
    }
    
    float base_jerk_limit = TRAJECTORY_JERK_LIMIT;
    
    float current_vel_magnitude = fabsf(p->current_velocity);
    if (current_vel_magnitude < TRAJECTORY_LOW_VEL_THRESHOLD) {
        base_jerk_limit *= 0.5f;
    } else if (current_vel_magnitude > TRAJECTORY_HIGH_VEL_THRESHOLD) {
        base_jerk_limit *= 1.2f;
    }
    
    float accel_magnitude = fabsf(p->current_acceleration);
    if (accel_magnitude > effective_a_max * 0.7f) {
        base_jerk_limit *= 0.6f;
    }
    
    float max_accel_change = base_jerk_limit * dt;
    
    /* V6.0: 边界保护 - 确保最大加速度变化值有效 */
    if (!IS_VALID_FLOAT(max_accel_change) || max_accel_change < 0.0f) {
        max_accel_change = 0.0f;
    }
    
    if (fabsf(accel_diff) > max_accel_change) {
        float sign_accel = sign(accel_diff);
        float transition_factor = 1.0f - fabsf(accel_diff) / (2.0f * max_accel_change);
        if (transition_factor < 0.0f) transition_factor = 0.0f;
        float smooth_factor = 0.5f + 0.5f * transition_factor;
        accel_diff = sign_accel * max_accel_change * smooth_factor;
    }
    
    p->current_acceleration += accel_diff;
    
    /* V6.0: 边界保护 - 最终加速度范围检查 */
    p->current_acceleration = clamp(p->current_acceleration, -TRAJECTORY_MAX_ACCEL, TRAJECTORY_MAX_ACCEL);
    
    return desired_accel;
}

/**
 * @brief 初始化默认配置
 */
static void init_default_config(TrajectoryConfig_t* config)
{
    config->v_max = TRAJECTORY_DEFAULT_VMAX;
    config->a_max = TRAJECTORY_DEFAULT_AMAX;
    config->d_max = TRAJECTORY_DEFAULT_DMAX;
    config->jerk_limit = TRAJECTORY_JERK_LIMIT;
    config->smooth_factor = TRAJECTORY_SMOOTH_FACTOR;
    config->pos_threshold = TRAJECTORY_POS_THRESHOLD;
    config->vel_threshold = TRAJECTORY_VEL_THRESHOLD;
}

/* ============================================================================
 * V4.0 预测性轨迹规划函数
 * ============================================================================ */

/**
 * @brief 添加目标位置到历史缓冲区
 * @param motor_id 电机索引
 * @param position 目标位置
 * @param velocity 目标速度（可选，0表示未知）
 */
static void history_push(uint8_t motor_id, float position, float velocity)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    
    p->history_positions[p->history_index] = position;
    p->history_velocities[p->history_index] = velocity;
    p->history_timestamps[p->history_index] = HAL_GetTick();
    
    p->history_index = (p->history_index + 1) % TRAJECTORY_HISTORY_SIZE;
    if (p->history_count < TRAJECTORY_HISTORY_SIZE) {
        p->history_count++;
    }
}

/**
 * @brief V12.0: 极值点检测 - 优化方案B1
 * @param motor_id 电机索引
 * @param output 输出极值点信息
 * 
 * @note 检测历史数据中的极值点（峰值/谷值），用于调整速度和加速度
 *       在极值点附近降低速度和加速度，减少峰值误差
 *       V6.0: 优化检测灵敏度和响应速度
 */
static void detect_extreme_point(uint8_t motor_id, ExtremePointInfo_t* output)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || output == NULL) return;
    
    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    
    output->is_extreme_point = false;
    output->velocity_reduction = 1.0f;
    output->accel_reduction = 1.0f;
    
    if (p->history_count < 3) return;
    
    uint8_t idx0 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 3) % TRAJECTORY_HISTORY_SIZE;
    uint8_t idx1 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 2) % TRAJECTORY_HISTORY_SIZE;
    uint8_t idx2 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 1) % TRAJECTORY_HISTORY_SIZE;
    
    float p0 = p->history_positions[idx0];
    float p1 = p->history_positions[idx1];
    float p2 = p->history_positions[idx2];
    
    ExtremePointConfig_t* cfg = &g_trajectory_global.extreme_config;
    uint32_t current_tick = HAL_GetTick();
    
    /* V3.0: 计算当前速度和加速度 */
    float vel_recent = p2 - p1;
    float vel_prev = p1 - p0;
    float current_velocity = fabsf(vel_recent) * 200.0f; /* 转换为deg/s */
    float current_accel = fabsf(vel_recent - vel_prev) * 40000.0f; /* 转换为deg/s^2 */
    
    /* V3.0: 极值点候选检测 - 速度接近零且加速度大 */
    bool is_candidate = (current_velocity < 30.0f && current_accel > 100.0f) ||
                        ((p1 > p0 && p1 > p2) || (p1 < p0 && p1 < p2));
    
    if (is_candidate) {
        if (!g_trajectory_global.extreme_detector.candidate_detected[motor_id]) {
            g_trajectory_global.extreme_detector.candidate_detected[motor_id] = true;
            g_trajectory_global.extreme_detector.candidate_tick[motor_id] = current_tick;
            g_trajectory_global.extreme_detector.candidate_velocity[motor_id] = current_velocity;
            g_trajectory_global.extreme_detector.confirm_count[motor_id] = 1;
        } else {
            g_trajectory_global.extreme_detector.confirm_count[motor_id]++;
            
            if (g_trajectory_global.extreme_detector.confirm_count[motor_id] >= 2) {
                g_trajectory_global.extreme_detector.confirmed[motor_id] = true;
            }
        }
    } else {
        g_trajectory_global.extreme_detector.candidate_detected[motor_id] = false;
        g_trajectory_global.extreme_detector.confirm_count[motor_id] = 0;
        
        if (current_tick - g_trajectory_global.extreme_detector.candidate_tick[motor_id] > 50) {
            g_trajectory_global.extreme_detector.confirmed[motor_id] = false;
        }
    }
    
    if (g_trajectory_global.extreme_detector.confirmed[motor_id]) {
        output->is_extreme_point = true;
        output->is_peak = (p1 > p0);
        
        output->extreme_distance = fabsf(p->target_position - p1);
        
        if (output->extreme_distance < cfg->extreme_distance_near) {
            output->velocity_reduction = cfg->vel_reduction_near;
            output->accel_reduction = cfg->accel_reduction_near;
        } else if (output->extreme_distance < cfg->extreme_distance_mid) {
            float distance_factor = (output->extreme_distance - cfg->extreme_distance_near) / 
                                    (cfg->extreme_distance_mid - cfg->extreme_distance_near);
            output->velocity_reduction = cfg->vel_reduction_near + 
                                        (cfg->vel_reduction_mid - cfg->vel_reduction_near) * distance_factor;
            output->accel_reduction = cfg->accel_reduction_near + 
                                     (cfg->accel_reduction_mid - cfg->accel_reduction_near) * distance_factor;
        } else if (output->extreme_distance < cfg->extreme_distance_far) {
            float distance_factor = (output->extreme_distance - cfg->extreme_distance_mid) / 
                                    (cfg->extreme_distance_far - cfg->extreme_distance_mid);
            output->velocity_reduction = cfg->vel_reduction_mid + (1.0f - cfg->vel_reduction_mid) * distance_factor;
            output->accel_reduction = cfg->accel_reduction_mid + (1.0f - cfg->accel_reduction_mid) * distance_factor;
        }
        
        output->detection_tick = HAL_GetTick();
    }
    
    if (p->history_count >= 4) {
        uint8_t idx3 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 4) % TRAJECTORY_HISTORY_SIZE;
        float p3 = p->history_positions[idx3];
        
        float vel_recent = p2 - p1;
        float vel_prev = p1 - p0;
        float vel_older = p0 - p3;
        
        if (fabsf(vel_recent) < fabsf(vel_prev) && fabsf(vel_prev) < fabsf(vel_older)) {
            float speed_ratio = fabsf(vel_recent) / (fabsf(vel_prev) + 0.001f);
            if (speed_ratio < cfg->speed_ratio_threshold) {
                output->velocity_reduction = fminf(output->velocity_reduction, 0.6f + 0.4f * speed_ratio);
                output->accel_reduction = fminf(output->accel_reduction, 0.5f + 0.5f * speed_ratio);
            }
        }
        
        if (p->history_count >= 5) {
            uint8_t idx4 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 5) % TRAJECTORY_HISTORY_SIZE;
            float p4 = p->history_positions[idx4];
            float vel_oldest = p3 - p4;
            
            /* 检测速度方向连续变化 */
            float vel_dir_recent = sign(vel_recent);
            float vel_dir_prev = sign(vel_prev);
            float vel_dir_older = sign(vel_older);
            float vel_dir_oldest = sign(vel_oldest);
            
            /* 如果速度方向开始反转，提前减速 */
            if (vel_dir_recent != vel_dir_prev && fabsf(vel_recent) < fabsf(vel_prev)) {
                output->velocity_reduction = fminf(output->velocity_reduction, 0.6f);
                output->accel_reduction = fminf(output->accel_reduction, 0.5f);
            }
        }
    }
    
    /* V6.0: 使用全局结构中的字段替代静态变量，确保线程安全 */
    /* 动态调整平滑因子：在极值点附近减少平滑效果，提高响应速度 */
    float smooth_alpha = 0.6f;
    if (output->is_extreme_point || output->velocity_reduction < 0.8f) {
        smooth_alpha = 0.3f;  /* 减少平滑，提高响应速度 */
    }
    
    /* 加锁保护共享资源 */
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    output->velocity_reduction = smooth_alpha * g_trajectory_global.last_vel_reduction[motor_id] + (1.0f - smooth_alpha) * output->velocity_reduction;
    output->accel_reduction = smooth_alpha * g_trajectory_global.last_accel_reduction[motor_id] + (1.0f - smooth_alpha) * output->accel_reduction;
    
    g_trajectory_global.last_vel_reduction[motor_id] = output->velocity_reduction;
    g_trajectory_global.last_accel_reduction[motor_id] = output->accel_reduction;
    
    /* 释放锁 */
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
}

/**
 * @brief 使用二次多项式拟合预测未来目标位置
 * @param motor_id 电机索引
 * @param predict_time 预测时间 (s)
 * @return 预测的目标位置
 * 
 * @note 使用最小二乘法拟合二次多项式: p(t) = a*t^2 + b*t + c
 *       可以同时预测位置和速度
 *       V6.0: 改进数值稳定性，使用更稳定的最小二乘法实现
 */
static float predict_future_target(uint8_t motor_id, float predict_time, float* predicted_velocity)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) {
        if (predicted_velocity) *predicted_velocity = 0.0f;
        return 180.0f;
    }
    
    /* 验证预测时间 */
    if (!IS_VALID_FLOAT(predict_time) || predict_time < 0.0f || predict_time > 1.0f) {
        LOG_WARN("[Trajectory] Invalid predict_time %.6f, using default", predict_time);
        predict_time = 0.02f;  /* 使用默认值 20ms */
    }
    
    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    
    /* 需要至少3个数据点才能进行二次拟合 */
    if (p->history_count < 3) {
        if (predicted_velocity) *predicted_velocity = 0.0f;
        p->prediction_valid = false;
        return p->target_position;
    }
    
    /* 获取最近的数据点 */
    uint8_t n = (p->history_count < TRAJECTORY_HISTORY_SIZE) ? p->history_count : TRAJECTORY_HISTORY_SIZE;
    
    /* 计算时间基准（相对于最新数据点的时间）*/
    uint32_t latest_tick = p->history_timestamps[(p->history_index + TRAJECTORY_HISTORY_SIZE - 1) % TRAJECTORY_HISTORY_SIZE];
    
    /* 检查时间戳有效性 */
    if (latest_tick == 0) {
        if (predicted_velocity) *predicted_velocity = 0.0f;
        p->prediction_valid = false;
        return p->target_position;
    }
    
    /* 准备拟合数据 */
    float sum_t = 0.0f, sum_t2 = 0.0f, sum_t3 = 0.0f, sum_t4 = 0.0f;
    float sum_p = 0.0f, sum_tp = 0.0f, sum_t2p = 0.0f;
    
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (p->history_index + TRAJECTORY_HISTORY_SIZE - 1 - i) % TRAJECTORY_HISTORY_SIZE;
        
        /* 时间转换为秒，相对于最新点 */
        float dt_ms = (int32_t)(latest_tick - p->history_timestamps[idx]);
        float t = -dt_ms / 1000.0f;  /* 负值表示过去 */
        
        float pos = p->history_positions[idx];
        
        /* 数值稳定性检查 */
        if (!IS_VALID_FLOAT(t) || !IS_VALID_FLOAT(pos) || p->history_timestamps[idx] == 0) {
            continue;  /* 跳过无效数据点 */
        }
        
        sum_t += t;
        sum_t2 += t * t;
        sum_t3 += t * t * t;
        sum_t4 += t * t * t * t;
        sum_p += pos;
        sum_tp += t * pos;
        sum_t2p += t * t * pos;
        valid_count++;
    }
    
    /* 确保有足够的有效数据点 */
    if (valid_count < 3) {
        if (predicted_velocity) *predicted_velocity = 0.0f;
        p->prediction_valid = false;
        return p->target_position;
    }
    
    /* 求解二次多项式系数: p(t) = a*t^2 + b*t + c
     * 使用更稳定的最小二乘法实现
     */
    float denom = valid_count * sum_t2 * sum_t4 + 2.0f * sum_t * sum_t2 * sum_t3 
                - sum_t2 * sum_t2 * sum_t2 - valid_count * sum_t3 * sum_t3 - sum_t * sum_t * sum_t4;
    
    /* 增强的数值稳定性检查 */
    if (fabsf(denom) < 1e-6f) {
        /* 无法求解，使用线性外推 */
        uint8_t idx1 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 1) % TRAJECTORY_HISTORY_SIZE;
        uint8_t idx2 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 2) % TRAJECTORY_HISTORY_SIZE;
        
        float pos1 = p->history_positions[idx1];
        float pos2 = p->history_positions[idx2];
        
        if (!IS_VALID_FLOAT(pos1) || !IS_VALID_FLOAT(pos2)) {
            if (predicted_velocity) *predicted_velocity = 0.0f;
            p->prediction_valid = false;
            return p->target_position;
        }
        
        float vel = SAFE_DIVIDE(pos1 - pos2, 0.02f, 0.0f);
        if (predicted_velocity) *predicted_velocity = vel;
        p->prediction_valid = true;
        
        float predicted_pos = p->target_position + vel * predict_time;
        predicted_pos = clamp(predicted_pos, 0.0f, 359.99f);
        return predicted_pos;
    }
    
    /* 计算系数，使用SAFE_DIVIDE确保数值稳定性 */
    float a = SAFE_DIVIDE(valid_count * sum_t2p + sum_t * sum_tp * sum_t2 - sum_t2 * sum_t2 * sum_p - sum_t * sum_t * sum_t2p - valid_count * sum_t3 * sum_tp, denom, 0.0f);
    float b = SAFE_DIVIDE(sum_p * sum_t2 * sum_t4 + sum_t * sum_t2p * sum_t3 + sum_t2 * sum_tp * sum_t4 
             - sum_t2p * sum_t2 * sum_t2 - sum_p * sum_t3 * sum_t3 - sum_t * sum_tp * sum_t4, denom, 0.0f);
    float c = SAFE_DIVIDE(sum_t2p * sum_t2 * sum_t2 + sum_tp * sum_t * sum_t4 + sum_p * sum_t3 * sum_t3 
             - sum_t2p * sum_t3 * sum_t2 - sum_tp * sum_t2 * sum_t4 - sum_p * sum_t2 * sum_t2, denom, 0.0f);
    
    /* 数值有效性检查 */
    if (!IS_VALID_FLOAT(a) || !IS_VALID_FLOAT(b) || !IS_VALID_FLOAT(c)) {
        if (predicted_velocity) *predicted_velocity = 0.0f;
        p->prediction_valid = false;
        return p->target_position;
    }
    
    /* 预测未来位置: p(predict_time) = a*predict_time^2 + b*predict_time + c */
    float predicted_pos = a * predict_time * predict_time + b * predict_time + c;
    
    /* 预测速度: v(predict_time) = 2*a*predict_time + b */
    if (predicted_velocity) {
        *predicted_velocity = 2.0f * a * predict_time + b;
        /* 速度有效性检查 */
        if (!IS_VALID_FLOAT(*predicted_velocity)) {
            *predicted_velocity = 0.0f;
        }
        /* 速度限制 */
        *predicted_velocity = safe_clamp_velocity(*predicted_velocity);
    }
    
    p->prediction_valid = true;
    
    /* 限制预测位置在合理范围内 */
    predicted_pos = clamp(predicted_pos, 0.0f, 359.99f);
    
    return predicted_pos;
}

/* ============================================================================
 * V11.0: 50Hz优化 - 三次样条插值
 * ============================================================================ */

/**
 * @brief 更新样条插值缓冲区
 * @param motor_id 电机索引
 * @param position 新位置数据
 * @note V6.0: 添加线程安全保护
 */
static void spline_buffer_push(uint8_t motor_id, float position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    /* V6.0: 加锁保护共享资源 */
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    SplineBuffer_t* buf = &g_spline_buffer[motor_id];
    
    buf->positions[buf->index] = position;
    buf->timestamps[buf->index] = HAL_GetTick();
    buf->index = (buf->index + 1) % SPLINE_BUFFER_SIZE;
    
    if (buf->count < SPLINE_BUFFER_SIZE) {
        buf->count++;
    }
    
    /* 预计算样条系数，当缓冲区满时 */
    if (buf->count >= 4) {
        /* 获取最近的4个点: P0, P1, P2, P3 */
        uint8_t idx[4];
        for (int i = 0; i < 4; i++) {
            idx[i] = (buf->index + SPLINE_BUFFER_SIZE - 4 + i) % SPLINE_BUFFER_SIZE;
        }
        
        float p0 = buf->positions[idx[0]];
        float p1 = buf->positions[idx[1]];
        float p2 = buf->positions[idx[2]];
        float p3 = buf->positions[idx[3]];
        
        /* 预计算Catmull-Rom样条系数 */
        buf->spline_coeffs[0] = -0.5f * p0 + 1.5f * p1 - 1.5f * p2 + 0.5f * p3;      /* a */
        buf->spline_coeffs[1] = p0 - 2.5f * p1 + 2.0f * p2 - 0.5f * p3;               /* b */
        buf->spline_coeffs[2] = -0.5f * p0 + 0.5f * p2;                                /* c */
        buf->spline_coeffs[3] = p1;                                                    /* d */
        
        buf->coeffs_valid = true;
    } else {
        buf->coeffs_valid = false;
    }
    
    /* V6.0: 释放锁 */
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
}

/**
 * @brief 三次样条插值 - 在50Hz数据点之间生成平滑轨迹
 * @param motor_id 电机索引
 * @param t 插值时间参数 [0, 1]，表示在两个数据点之间的位置
 * @return 插值后的目标位置
 * 
 * @note V11.0: 使用Catmull-Rom样条实现平滑插值
 *       在200Hz控制周期中，每4个周期使用一次新数据
 *       中间3个周期使用样条插值生成平滑过渡
 *       V6.0: 添加线程安全保护
 */
static float spline_interpolate(uint8_t motor_id, float t)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 0.0f;
    
    /* V6.0: 加锁保护共享资源 */
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    SplineBuffer_t* buf = &g_spline_buffer[motor_id];
    
    float result = 0.0f;
    
    /* 需要至少4个点才能进行三次样条插值 */
    if (buf->count < 4) {
        /* 点数不足，使用线性插值 */
        if (buf->count >= 2) {
            uint8_t idx0 = (buf->index + SPLINE_BUFFER_SIZE - 2) % SPLINE_BUFFER_SIZE;
            uint8_t idx1 = (buf->index + SPLINE_BUFFER_SIZE - 1) % SPLINE_BUFFER_SIZE;
            float p0 = buf->positions[idx0];
            float p1 = buf->positions[idx1];
            result = p0 + (p1 - p0) * t;
        } else if (buf->count > 0) {
            result = buf->positions[(buf->index + SPLINE_BUFFER_SIZE - 1) % SPLINE_BUFFER_SIZE];
        }
    } else if (buf->coeffs_valid) {
        /* 使用预计算的样条系数，减少计算复杂度 */
        float t2 = t * t;
        float t3 = t2 * t;
        
        /* P(t) = a*t^3 + b*t^2 + c*t + d */
        result = buf->spline_coeffs[0] * t3 + 
                 buf->spline_coeffs[1] * t2 + 
                 buf->spline_coeffs[2] * t + 
                 buf->spline_coeffs[3];
    } else {
        /* 系数无效，回退到传统计算方法 */
        uint8_t idx[4];
        for (int i = 0; i < 4; i++) {
            idx[i] = (buf->index + SPLINE_BUFFER_SIZE - 4 + i) % SPLINE_BUFFER_SIZE;
        }
        
        float p0 = buf->positions[idx[0]];
        float p1 = buf->positions[idx[1]];
        float p2 = buf->positions[idx[2]];
        float p3 = buf->positions[idx[3]];
        
        float t2 = t * t;
        float t3 = t2 * t;
        
        result = 0.5f * (2.0f * p1 +
                       (-p0 + p2) * t +
                       (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                       (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
    }
    
    /* 释放锁 */
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    /* 限制结果在合理范围内 */
    return clamp(result, 0.0f, 359.99f);
}

/**
 * @brief 计算平滑过渡的目标位置
 * @param motor_id 电机索引
 * @param new_target 新目标位置
 * @param dt 时间步长
 * @return 平滑后的目标位置
 * 
 * @note 使用一阶低通滤波 + 速度限制实现平滑过渡
 *       V5.0: 添加数值稳定性检查
 */
static float smooth_target_transition(uint8_t motor_id, float new_target, float dt)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return SAFE_FLOAT(new_target, 180.0f);
    
    /* V5.0: 验证输入参数 */
    if (!IS_VALID_FLOAT(new_target)) {
        LOG_WARN("[Trajectory] Invalid new_target for motor %d", motor_id);
        new_target = 180.0f;
    }
    
    /* V11.0: 防止除零错误 */
    if (dt < TRAJECTORY_MIN_DT) {
        return new_target;
    }
    
    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    
    /* V5.0: 验证当前状态 */
    if (!IS_VALID_FLOAT(p->smooth_target)) {
        p->smooth_target = new_target;
    }
    if (!IS_VALID_FLOAT(p->dynamic_v_max)) {
        p->dynamic_v_max = TRAJECTORY_DEFAULT_VMAX;
    }
    
    /* 计算目标变化量 */
    float target_diff = new_target - p->smooth_target;
    
    /* 计算最大允许变化量（基于最大速度）*/
    float max_change = p->dynamic_v_max * dt * 2.0f;
    
    /* 限制变化量 */
    if (fabsf(target_diff) > max_change) {
        target_diff = sign(target_diff) * max_change;
    }
    
    /* 更新平滑目标 */
    p->smooth_target += target_diff;
    
    /* V5.0: 确保平滑目标在有效范围内 */
    p->smooth_target = safe_clamp_position(p->smooth_target);
    
    /* 计算平滑速度 */
    p->smooth_velocity = SAFE_DIVIDE(target_diff, dt, 0.0f);
    p->smooth_velocity = safe_clamp_velocity(p->smooth_velocity);
    
    return p->smooth_target;
}

/**
 * @brief 生成连续平滑的轨迹输出
 * @param motor_id 电机索引
 * @param target_position 目标位置
 * @param target_velocity 目标速度（可选）
 * @param dt 时间步长
 * @param output 输出结构体
 * 
 * @note V4.0 核心算法：确保轨迹的连续性
 *       1. 使用预测目标而非原始目标
 *       2. 平滑过渡到新目标
 *       3. 保证位置、速度、加速度的连续性
 */
static void generate_smooth_trajectory(uint8_t motor_id, float target_position, float target_velocity, float dt, TrajectoryOutput_t* output)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || output == NULL) return;

    /* 验证输入参数 */
    if (!IS_VALID_FLOAT(target_position) || !IS_VALID_FLOAT(target_velocity) || !IS_VALID_FLOAT(dt)) {
        LOG_ERROR("[Trajectory] Invalid input parameters for motor %d", motor_id);
        /* 初始化输出为安全值 */
        memset(output, 0, sizeof(TrajectoryOutput_t));
        output->position = 180.0f;
        output->state = TRAJ_STATE_ERROR;
        return;
    }

    /* 验证时间步长 */
    if (dt < TRAJECTORY_MIN_DT || dt > 0.1f) {
        LOG_WARN("[Trajectory] Invalid dt %.6f for motor %d, using default", dt, motor_id);
        dt = 0.005f;  /* 使用默认值 5ms */
    }

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    /* 验证规划器状态 */
    if (!IS_VALID_FLOAT(p->current_position) || !IS_VALID_FLOAT(p->current_velocity) || !IS_VALID_FLOAT(p->current_acceleration)) {
        LOG_WARN("[Trajectory] Invalid planner state for motor %d, resetting", motor_id);
        /* 重置规划器状态 */
        p->current_position = 180.0f;
        p->current_velocity = 0.0f;
        p->current_acceleration = 0.0f;
        p->smooth_target = 180.0f;
        p->smooth_velocity = 0.0f;
    }

    /* V11.0: 更新样条插值缓冲区 */
    spline_buffer_push(motor_id, target_position);

    /* 1. 添加到历史缓冲区 */
    history_push(motor_id, target_position, target_velocity);

    /* V12.0: 极值点检测 - 优化方案B1 */
    ExtremePointInfo_t extreme_info;
    detect_extreme_point(motor_id, &extreme_info);

    /* V11.0: 50Hz优化 - 使用样条插值生成更平滑的中间目标
     * 在200Hz控制周期中，每4个周期收到一次新数据
     * 使用样条插值填补中间轨迹
     */
    /* V6.0: 使用全局结构中的字段替代静态变量，确保线程安全 */
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    g_trajectory_global.interpolation_counter[motor_id] = (g_trajectory_global.interpolation_counter[motor_id] + 1) % 4;
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    float interpolated_target;
    /* V6.0: 使用全局结构中的字段 */
    uint8_t counter;
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    counter = g_trajectory_global.interpolation_counter[motor_id];
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    if (g_trajectory_global.use_spline_interpolation) {
        /* 始终使用样条插值，确保轨迹连续性 */
        float t = counter / 4.0f;
        interpolated_target = spline_interpolate(motor_id, t);
        
        /* 当新数据到达时，确保平滑过渡到新目标 */
        if (counter == 0) {
            /* 新数据到达，使用平滑过渡 */
            float prev_target = p->smooth_target;
            float target_diff = target_position - prev_target;
            float smooth_factor = 0.2f;  /* 平滑因子，控制过渡速度 */
            interpolated_target = prev_target + target_diff * smooth_factor;
        }
    } else {
        /* 不使用样条插值，直接使用目标位置 */
        interpolated_target = target_position;
    }

    /* 2. 预测未来目标 */
    float pred_vel = 0.0f;
    float predicted_target = predict_future_target(motor_id, TRAJECTORY_PREDICTION_HORIZON, &pred_vel);
    p->predicted_position = predicted_target;
    p->predicted_velocity = pred_vel;

    /* V8.0: 添加反向间隙补偿（与仿真脚本同步） */
    float compensated_target = interpolated_target;  /* V11.0: 使用插值后的目标 */
    if (g_trajectory_global.use_backlash_compensation) {
        compensated_target = Trajectory_ApplyBacklashCompensation(motor_id, interpolated_target, p->current_position);
    }

    /* 3. 使用补偿后的目标 */
    float smooth_target = smooth_target_transition(motor_id, compensated_target, dt);
    
    /* V12.0: 应用极值点速度/加速度折扣 */
    float effective_v_max = p->dynamic_v_max * extreme_info.velocity_reduction;
    float effective_a_max = p->dynamic_a_max * extreme_info.accel_reduction;
    float effective_d_max = p->dynamic_d_max * extreme_info.accel_reduction;
    
    /* 4. 计算到平滑目标的距离 */
    float distance = smooth_target - p->current_position;
    float abs_distance = fabsf(distance);
    bool direction = (distance >= 0.0f);
    
    /* 正弦波优化：检测是否接近方向切换点 */
    bool near_direction_change = false;
    if (p->history_count >= 3) {
        uint8_t idx1 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 2) % TRAJECTORY_HISTORY_SIZE;
        uint8_t idx2 = (p->history_index + TRAJECTORY_HISTORY_SIZE - 1) % TRAJECTORY_HISTORY_SIZE;
        float recent_vel = p->history_positions[idx2] - p->history_positions[idx1];
        float current_dir = direction ? 1.0f : -1.0f;
        /* 如果历史速度方向与当前方向相反，说明即将换向 */
        if (sign(recent_vel) != current_dir && fabsf(recent_vel) > 0.5f) {
            near_direction_change = true;
        }
    }
    
    /* 5. 计算期望速度（基于距离和预测速度）*/
    float desired_velocity = calculate_desired_velocity(p, distance, abs_distance, direction, effective_v_max, effective_a_max, effective_d_max, extreme_info, near_direction_change, pred_vel, dt);
    
    /* 6. 计算期望加速度并应用Jerk限制 */
    float desired_accel = calculate_desired_acceleration(p, desired_velocity, effective_a_max, effective_d_max, dt);
    
    /* 8. 积分更新速度 */
    p->current_velocity += p->current_acceleration * dt;
    p->current_velocity = clamp(p->current_velocity, 0.0f, effective_v_max);
    
    /* 9. 积分更新位置 */
    float direction_sign = direction ? 1.0f : -1.0f;
    p->current_position += direction_sign * p->current_velocity * dt;
    
    /* 10. 位置范围检查 */
    p->current_position = clamp(p->current_position, 0.0f, 359.99f);
    
    /* 11. 检查是否到达目标 */
    float remaining = smooth_target - p->current_position;
    if (fabsf(remaining) < p->config.pos_threshold && p->current_velocity < p->config.vel_threshold) {
        p->current_position = smooth_target;
        p->current_velocity = 0.0f;
        p->current_acceleration = 0.0f;
        p->state = TRAJ_STATE_COMPLETED;
        p->is_moving = false;
    }
    
    /* 12. 更新方向 */
    p->direction = direction;
    p->remaining_distance = remaining;
    
    /* V5.5: 加速度平滑滤波 - 消除状态切换时的阶跃 */
    #define ACCEL_FILTER_ALPHA 0.80f
    p->filtered_acceleration = ACCEL_FILTER_ALPHA * p->filtered_acceleration + 
                               (1.0f - ACCEL_FILTER_ALPHA) * p->current_acceleration;
    
    /* 13. 填充输出 - V13.0: 速度和加速度始终为正（绝对值），方向由direction变量控制 */
    output->position = p->current_position;
    output->velocity = p->current_velocity;  /* V13.0: 速度始终为正（绝对值） */
    output->acceleration = p->filtered_acceleration;  /* V13.0: 加速度始终为正（绝对值） */
    output->state = p->state;
    output->is_completed = (p->state == TRAJ_STATE_COMPLETED);
    output->direction = direction;  /* V13.0: 输出方向标志 */
    
    /* 14. 计算前馈输出 - V13.0: 前馈始终为正 */
    output->feedforward_vel = g_trajectory_global.ff_config.vel_ff_gain * p->current_velocity;
    output->feedforward_accel = g_trajectory_global.ff_config.accel_ff_gain * p->filtered_acceleration;
    
    /* 15. PID反馈补偿 */
    float pos_error = smooth_target - p->current_position;
    g_trajectory_global.vel_integral[motor_id] += pos_error * dt;
    g_trajectory_global.vel_integral[motor_id] = clamp(g_trajectory_global.vel_integral[motor_id], -g_trajectory_global.ff_config.integral_limit, g_trajectory_global.ff_config.integral_limit);
    
    float alpha = 0.8f;
    float raw_deriv = (dt > 0.0001f) ? ((pos_error - g_trajectory_global.last_pos_error[motor_id]) / dt) : 0.0f;
    g_trajectory_global.filtered_deriv[motor_id] = alpha * g_trajectory_global.filtered_deriv[motor_id] + (1.0f - alpha) * raw_deriv;
    g_trajectory_global.last_pos_error[motor_id] = pos_error;
    
    float pid_output = g_trajectory_global.ff_config.pos_kp * pos_error 
                     + g_trajectory_global.ff_config.vel_ki * g_trajectory_global.vel_integral[motor_id]
                     + g_trajectory_global.ff_config.vel_kd * g_trajectory_global.filtered_deriv[motor_id];
    
    output->feedforward_vel += pid_output * 0.01f;
    
    /* 16. 保存上次规划输出（用于连续性检查）*/
    p->last_planned_position = output->position;
    p->last_planned_velocity = output->velocity;
    p->last_planned_accel = output->acceleration;
}

/**
 * @brief 计算梯形速度曲线
 * @param distance 总距离
 * @param v_max 最大速度
 * @param a_max 最大加速度
 * @param d_max 最大减速度
 * @param peak_vel 输出峰值速度
 * @param accel_dist 输出加速距离
 * @param cruise_dist 输出匀速距离
 * @param decel_dist 输出减速距离
 */
static void calculate_trapezoid_profile(float distance, float v_max, float a_max, float d_max,
                                         float* peak_vel, float* accel_dist, float* cruise_dist, float* decel_dist)
{
    float abs_dist = fabsf(distance);
    
    if (abs_dist < 0.001f) {
        *peak_vel = 0.0f;
        *accel_dist = 0.0f;
        *cruise_dist = 0.0f;
        *decel_dist = 0.0f;
        return;
    }
    
    /* 计算三角形速度曲线能达到的峰值速度 */
    float v_triangle = sqrtf(2.0f * a_max * abs_dist * d_max / (a_max + d_max));
    
    /* 确定实际峰值速度 */
    if (v_triangle <= v_max) {
        /* 三角形曲线 */
        *peak_vel = v_triangle;
        *accel_dist = (v_triangle * v_triangle) / (2.0f * a_max);
        *decel_dist = (v_triangle * v_triangle) / (2.0f * d_max);
        *cruise_dist = 0.0f;
    } else {
        /* 梯形曲线 */
        *peak_vel = v_max;
        *accel_dist = (v_max * v_max) / (2.0f * a_max);
        *decel_dist = (v_max * v_max) / (2.0f * d_max);
        *cruise_dist = abs_dist - *accel_dist - *decel_dist;
        
        /* 防止负数（距离不够） */
        if (*cruise_dist < 0.0f) {
            *cruise_dist = 0.0f;
            /* 重新计算峰值速度 */
            *peak_vel = sqrtf(2.0f * a_max * abs_dist * d_max / (a_max + d_max));
            *accel_dist = (*peak_vel * *peak_vel) / (2.0f * a_max);
            *decel_dist = (*peak_vel * *peak_vel) / (2.0f * d_max);
        }
    }
}

/**
 * @brief 计算S曲线规划参数（七段式）
 * @param distance 总距离 (deg)
 * @param v_max 最大速度 (deg/s)
 * @param a_max 最大加速度 (deg/s²)
 * @param jerk 加加速度限制 (deg/s³)
 * @param s_curve 输出S曲线状态
 * @return 是否成功规划
 *
 * @note V2.0: 优化计算精度，确保运动学约束满足
 */
static bool calculate_s_curve_profile(float distance, float v_max, float a_max, float jerk,
                                       SCurveState_t* s_curve)
{
    float abs_dist = fabsf(distance);
    
    if (abs_dist < 0.1f) {
        memset(s_curve, 0, sizeof(SCurveState_t));
        return false;
    }
    
    /* 参数有效性检查 */
    if (jerk < 1.0f) jerk = TRAJECTORY_JERK_LIMIT;
    if (a_max < 1.0f) a_max = TRAJECTORY_DEFAULT_AMAX;
    if (v_max < 1.0f) v_max = TRAJECTORY_DEFAULT_VMAX;
    
    /* 计算达到最大加速度所需的时间 */
    float t_j = a_max / jerk;
    
    /* 在jerk阶段达到的速度增量 */
    float v_j = 0.5f * jerk * t_j * t_j;
    
    /* 在jerk阶段移动的距离 */
    float d_j = 0.5f * v_j * t_j;
    
    /* 计算恒定加速度阶段的速度和距离 */
    float v_accel = v_max - 2.0f * v_j;
    float t_a = (v_accel > 0.0f) ? (v_accel / a_max) : 0.0f;
    
    /* 加速阶段总距离 */
    float d_accel = 2.0f * d_j + v_accel * t_a + v_j * t_a;
    
    /* 加速+减速总距离 */
    float d_accel_decel = 2.0f * d_accel;
    
    s_curve->t_jerk1 = t_j;
    s_curve->t_accel = t_a;
    s_curve->t_jerk2 = t_j;
    s_curve->t_jerk3 = t_j;
    s_curve->t_decel = t_a;
    s_curve->t_jerk4 = t_j;
    
    if (d_accel_decel >= abs_dist) {
        /* 三角形曲线：无法达到最大速度
         * 使用精确公式计算峰值速度
         * d = 2 * (v_peak^2 / (2*a_max) + v_peak*t_j/2)
         * 简化为: v_peak = sqrt(a_max * abs_dist / 2) 近似
         */
        float v_peak = sqrtf(abs_dist * a_max * 0.5f);
        
        /* 限制峰值速度不超过最大速度 */
        if (v_peak > v_max) v_peak = v_max;
        
        /* 重新计算jerk时间 */
        t_j = (jerk > 0.001f) ? (v_peak / jerk) : 0.0f;
        if (t_j > abs_dist / v_peak) t_j = abs_dist / v_peak * 0.25f;
        
        s_curve->t_jerk1 = t_j;
        s_curve->t_accel = 0.0f;
        s_curve->t_jerk2 = t_j;
        s_curve->t_cruise = 0.0f;
        s_curve->t_jerk3 = t_j;
        s_curve->t_decel = 0.0f;
        s_curve->t_jerk4 = t_j;
        s_curve->peak_velocity = v_peak;
    } else {
        /* 梯形曲线：可以达到最大速度 */
        float d_cruise = abs_dist - d_accel_decel;
        s_curve->t_cruise = (v_max > 0.001f) ? (d_cruise / v_max) : 0.0f;
        s_curve->peak_velocity = v_max;
    }
    
    s_curve->phase = S_CURVE_PHASE_JERK_POS;
    s_curve->phase_time = 0.0f;
    s_curve->jerk = jerk;
    s_curve->use_s_curve = true;
    
    return true;
}

/**
 * @brief S 曲线一步执行 - 改进版本，使用连续变化的 jerk
 * @param p 规划器状态
 * @param dt 时间步长
 * @param direction 运动方向
 * @return 规划的位置增量
 * 
 * @note V2.0 改进:
 * - 使用正弦 jerk 曲线替代阶跃变化
 * - 确保加加速度连续
 * - 减少机械冲击
 */
static float s_curve_step(TrajectoryPlanner_t* p, float dt, bool direction)
{
    SCurveState_t* s = &p->s_curve;
    float dir_sign = direction ? 1.0f : -1.0f;  /* 用于位置变化方向 */
    float jerk = 0.0f;
    float accel = p->current_acceleration;
    float vel = p->current_velocity;
    
    s->phase_time += dt;
    
    /* V3.0 改进：速度始终为正，加速度正负表示加速/减速 */
    switch (s->phase) {
        case S_CURVE_PHASE_JERK_POS:
            /* 加速阶段：jerk为正 */
            if (s->t_jerk1 > 0.001f) {
                float jerk_phase = s->phase_time / s->t_jerk1;
                jerk = s->jerk * sinf(M_PI * jerk_phase);
            } else {
                jerk = s->jerk;
            }
            if (s->phase_time >= s->t_jerk1) {
                s->phase = S_CURVE_PHASE_ACCEL;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_ACCEL:
            jerk = 0.0f;
            /* V5.0: 不再强制设置加速度，保持连续性 */
            if (s->phase_time >= s->t_accel) {
                s->phase = S_CURVE_PHASE_JERK_NEG;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_JERK_NEG:
            /* 加速结束：jerk为负 */
            if (s->t_jerk2 > 0.001f) {
                float jerk_phase = s->phase_time / s->t_jerk2;
                jerk = -s->jerk * sinf(M_PI * jerk_phase);
            } else {
                jerk = -s->jerk;
            }
            if (s->phase_time >= s->t_jerk2) {
                s->phase = S_CURVE_PHASE_CRUISE;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_CRUISE:
            jerk = 0.0f;
            accel = 0.0f;
            vel = s->peak_velocity;  /* V3.0: 速度始终为正 */
            if (s->phase_time >= s->t_cruise) {
                s->phase = S_CURVE_PHASE_JERK_NEG2;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_JERK_NEG2:
            /* 减速开始：jerk为负 */
            if (s->t_jerk3 > 0.001f) {
                float jerk_phase = s->phase_time / s->t_jerk3;
                jerk = -s->jerk * sinf(M_PI * jerk_phase);
            } else {
                jerk = -s->jerk;
            }
            if (s->phase_time >= s->t_jerk3) {
                s->phase = S_CURVE_PHASE_DECEL;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_DECEL:
            jerk = 0.0f;
            /* V5.0: 不再强制设置加速度，保持连续性 */
            if (s->phase_time >= s->t_decel) {
                s->phase = S_CURVE_PHASE_JERK_POS2;
                s->phase_time = 0.0f;
            }
            break;
            
        case S_CURVE_PHASE_JERK_POS2:
            /* 减速结束：jerk为正 */
            if (s->t_jerk4 > 0.001f) {
                float jerk_phase = s->phase_time / s->t_jerk4;
                jerk = s->jerk * sinf(M_PI * jerk_phase);
            } else {
                jerk = s->jerk;
            }
            if (s->phase_time >= s->t_jerk4) {
                s->use_s_curve = false;
            }
            break;
    }
    
    if (s->use_s_curve) {
        /* 积分得到加速度和速度 */
        accel += jerk * dt;
        vel += accel * dt;
        
        /* V3.0: 速度限制为非负值 [0, peak_velocity] */
        vel = clamp(vel, 0.0f, s->peak_velocity * 1.1f);
        
        p->current_acceleration = accel;
        p->current_velocity = vel;
    }
    
    /* V3.0: 返回位置变化量，方向由dir_sign控制 */
    return dir_sign * vel * dt;
}

/* ============================================================================
 * 公共函数实现 - 初始化
 * ============================================================================ */

uint8_t Trajectory_Init(void)
{
    if (g_trajectory_global.initialized) {
        printf("[Trajectory] Already initialized\r\n");
        return 0;
    }

    /* 创建互斥锁 */
    if (g_trajectory_global.mutex == NULL) {
        osMutexAttr_t mutex_attr = {
            .name = "TrajectoryMutex",
            .attr_bits = osMutexRecursive | osMutexPrioInherit,
        };
        g_trajectory_global.mutex = osMutexNew(&mutex_attr);
    }

    /* 初始化所有规划器 */
    for (uint8_t i = 0; i < TRAJECTORY_MAX_MOTORS; i++) {
        Trajectory_InitSingle(i);
    }

    g_trajectory_global.initialized = true;
    printf("[Trajectory] Trajectory planner initialized (12 motors)\r\n");
    return 0;
}

void Trajectory_Deinit(void)
{
    if (!g_trajectory_global.initialized) return;

    /* 删除互斥锁 */
    if (g_trajectory_global.mutex != NULL) {
        osMutexDelete(g_trajectory_global.mutex);
        g_trajectory_global.mutex = NULL;
    }

    g_trajectory_global.initialized = false;
    printf("[Trajectory] Trajectory planner deinitialized\r\n");
}

uint8_t Trajectory_InitSingle(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    /* 清空状态 */
    memset(p, 0, sizeof(TrajectoryPlanner_t));

    /* 初始化配置 */
    init_default_config(&p->config);

    /* 初始化动态速度/加速度（默认使用配置值）*/
    p->dynamic_v_max = p->config.v_max;
    p->dynamic_a_max = p->config.a_max;
    p->dynamic_d_max = p->config.d_max;

    /* 初始化状态 - 默认站立位置为180° */
    p->state = TRAJ_STATE_IDLE;
    p->target_position = 180.0f;
    p->current_position = 180.0f;
    p->current_velocity = 0.0f;
    p->current_acceleration = 0.0f;
    p->is_moving = false;
    p->target_updated = false;

    /* 初始化前瞻缓冲 */
    p->lookahead_count = 0;
    p->lookahead_index = 0;
    p->lookahead_direction_change = false;
    for (uint8_t i = 0; i < TRAJECTORY_LOOKAHEAD_COUNT; i++) {
        p->lookahead_positions[i] = 180.0f;
    }

    /* V4.0: 初始化预测性轨迹规划数据结构 */
    for (uint8_t i = 0; i < TRAJECTORY_HISTORY_SIZE; i++) {
        p->history_positions[i] = 180.0f;
        p->history_velocities[i] = 0.0f;
        p->history_timestamps[i] = 0;
    }
    p->history_index = 0;
    p->history_count = 0;
    p->predicted_position = 180.0f;
    p->predicted_velocity = 0.0f;
    p->smooth_target = 180.0f;
    p->smooth_velocity = 0.0f;
    p->prediction_valid = false;
    p->last_planned_position = 180.0f;
    p->last_planned_velocity = 0.0f;
    p->last_planned_accel = 0.0f;
    p->blend_factor = 0.0f;

    return 0;
}

/* ============================================================================
 * 公共函数实现 - 配置
 * ============================================================================ */

uint8_t Trajectory_SetConfig(uint8_t motor_id, const TrajectoryConfig_t* config)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || config == NULL) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    memcpy(&g_trajectory_global.planners[motor_id].config, config, sizeof(TrajectoryConfig_t));

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_GetConfig(uint8_t motor_id, TrajectoryConfig_t* config)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || config == NULL) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    memcpy(config, &g_trajectory_global.planners[motor_id].config, sizeof(TrajectoryConfig_t));

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_SetMotionParams(uint8_t motor_id, float v_max, float a_max)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    g_trajectory_global.planners[motor_id].config.v_max = v_max;
    g_trajectory_global.planners[motor_id].config.a_max = a_max;
    g_trajectory_global.planners[motor_id].config.d_max = a_max; /* 减速度默认等于加速度 */

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_SetDynamicParams(uint8_t motor_id, float v_max, float a_max, float d_max)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    g_trajectory_global.planners[motor_id].dynamic_v_max = v_max;
    g_trajectory_global.planners[motor_id].dynamic_a_max = a_max;
    g_trajectory_global.planners[motor_id].dynamic_d_max = d_max;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

/* ============================================================================
 * 公共函数实现 - 控制
 * ============================================================================ */

uint8_t Trajectory_SetTarget(uint8_t motor_id, float target_position)
{
    return Trajectory_SetTargetWithVelocity(motor_id, target_position, 0.0f);
}

uint8_t Trajectory_SetTargetWithVelocity(uint8_t motor_id, float target_position, float initial_velocity)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    /* 角度限制检查 (0° ~ 359.99°) */
    if (target_position < 0.0f) {
        target_position = 0.0f;
    }
    if (target_position > 359.99f) {
        target_position = 359.99f;
    }

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    /* 优化1: 检查目标位置是否与当前位置相同（减少不必要的处理）*/
    float pos_diff_from_current = fabsf(target_position - p->current_position);
    if (pos_diff_from_current < TRAJECTORY_POS_THRESHOLD) {
        /* 目标位置已接近当前位置，无需移动 */
        if (p->is_moving) {
            p->is_moving = false;
            p->state = TRAJ_STATE_COMPLETED;
            p->current_velocity = 0.0f;
            p->current_acceleration = 0.0f;
        }
        // 清除前瞻缓冲
        p->lookahead_count = 0;
        p->lookahead_index = 0;
        p->lookahead_direction_change = false;

        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 0;
    }

    /* 检查目标是否变化（相对于当前目标）*/
    float target_diff = fabsf(target_position - p->target_position);
    if (target_diff < TRAJECTORY_BUFFER_MIN_DIFF && p->is_moving) {
        /* 目标基本没变，不重新规划 */
        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 0;
    }

    /* 检查是否需要从当前位置开始规划，还是从当前速度继续 */
    // 如果方向改变，需要先减速停止再反向
    float new_direction = (target_position >= p->current_position);

    /* 计算剩余距离 */
    float remaining_dist = target_position - p->current_position;
    /* 关键修复：如果remaining_distance为负（超调），则反转方向并取绝对值 */
    if (remaining_dist < 0.0f) {
        new_direction = false;
        remaining_dist = fabsf(remaining_dist);
    }
    
    /* BUG修复: 先保存旧方向，再设置新方向 */
    bool old_direction = p->direction;
    p->direction = new_direction;
    p->remaining_distance = remaining_dist;

    /* 设置新目标 */
    p->target_position = target_position;
    p->target_updated = true;

    /* 使用动态速度/加速度参数（如果前瞻检测到方向改变，会自动调整）*/
    float v_max = p->dynamic_v_max;
    float a_max = p->dynamic_a_max;
    float d_max = p->dynamic_d_max;

    /* 优化2: 检查方向是否改变 - 使用旧方向比较 */
    if (p->is_moving && (new_direction != old_direction)) {
        /* V3.0 改进：方向改变时，速度始终为正 */
        float current_speed = p->current_velocity;  /* V3.0: 速度始终为正 */

        // 计算减速到0的距离
        float braking_dist = Trajectory_CalcBrakingDistance(current_speed, d_max);

        // 如果当前距离足够减速到停止，就减速
        // 否则直接反向加速
        if (braking_dist >= fabsf(p->remaining_distance)) {
            // 需要减速 - 进入减速状态
            p->state = TRAJ_STATE_DECELERATING;
            p->current_acceleration = -d_max;  /* V3.0: 减速时加速度为负 */
        } else {
            // 距离足够，直接反向
            // 重新计算新方向
            p->direction = new_direction;
            p->remaining_distance = target_position - p->current_position;

            // 使用较小的初始速度（避免冲击）
            p->current_velocity = p->current_velocity * 0.3f;  // 保留30%速度

            // 进入加速状态
            p->state = TRAJ_STATE_ACCELERATING;
            p->current_acceleration = a_max;  /* V3.0: 加速时加速度为正 */
        }
    } else {
        /* 方向相同或静止：正常计算轨迹 */
        /* 计算最优轨迹（使用动态参数）*/
        float peak_vel, accel_dist, cruise_dist, decel_dist;
        calculate_trapezoid_profile(
            p->remaining_distance,
            v_max,
            a_max,
            d_max,
            &peak_vel, &accel_dist, &cruise_dist, &decel_dist
        );
        p->peak_velocity = peak_vel;  /* V3.0: 峰值速度始终为正 */

        /* 初始化S曲线规划 */
        if (p->config.jerk_limit > 0.0f) {
            calculate_s_curve_profile(
                p->remaining_distance,
                v_max,
                a_max,
                p->config.jerk_limit,
                &p->s_curve
            );
        }

        /* 如果有初始速度，使用传入值；否则保持当前速度继续运动（优化3）*/
        /* V3.0: 初始速度取绝对值 */
        if (fabsf(initial_velocity) > 0.1f) {
            p->current_velocity = fabsf(initial_velocity);
        }
        /* 注意：不强制设为0，保留当前速度以实现平滑过渡 */

        /* 开始运动 */
        p->state = TRAJ_STATE_ACCELERATING;
        p->current_acceleration = a_max;  /* V3.0: 加速时加速度为正 */
    }

    p->is_moving = true;
    p->tick_count = 0;

    /* 减少日志输出频率：只在目标变化较大时输出 */
    static uint32_t log_counter[TRAJECTORY_MAX_MOTORS] = {0};
    log_counter[motor_id]++;
    if (log_counter[motor_id] >= 50 || target_diff > 5.0f) {
        printf("[Traj] Motor %u: SetTarget %.1f -> %.1f, dist=%.1f, state=%d, v_max=%.1f\r\n",
               motor_id, p->current_position, target_position, p->remaining_distance, p->state, v_max);
        log_counter[motor_id] = 0;
    }

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_UpdateCurrentPosition(uint8_t motor_id, float current_position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    g_trajectory_global.planners[motor_id].current_position = current_position;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_Stop(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    /* 设置当前位置为目标（平滑停止）*/
    p->target_position = p->current_position;
    p->state = TRAJ_STATE_DECELERATING;
    p->is_moving = true;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_EmergencyStop(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    /* 立即停止 */
    p->current_velocity = 0.0f;
    p->current_acceleration = 0.0f;
    p->target_position = p->current_position;
    p->is_moving = false;
    p->state = TRAJ_STATE_IDLE;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

/* ============================================================================
 * 公共函数实现 - 执行
 * ============================================================================ */

/**
 * @brief 启用/禁用预测性轨迹规划
 */
void Trajectory_EnablePredictivePlanning(bool enable)
{
    g_trajectory_global.use_predictive_planning = enable;
    printf("[Trajectory] Predictive planning %s\r\n", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief 启用/禁用反向间隙补偿（V8.0新增）
 */
void Trajectory_EnableBacklashCompensation(bool enable)
{
    g_trajectory_global.use_backlash_compensation = enable;
    printf("[Trajectory] Backlash compensation %s\r\n", enable ? "ENABLED" : "DISABLED");
}

void Trajectory_EnableContinuousMode(bool enable)
{
    g_trajectory_global.continuous_mode_enabled = enable;
    printf("[Trajectory] Continuous mode %s\r\n", enable ? "ENABLED" : "DISABLED");
}

bool Trajectory_IsContinuousMode(void)
{
    return g_trajectory_global.continuous_mode_enabled;
}

uint8_t Trajectory_SetContinuousTarget(uint8_t motor_id, float target_position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    if (target_position < 0.0f) target_position = 0.0f;
    if (target_position > 359.99f) target_position = 359.99f;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    float target_diff = target_position - p->target_position;

    if (fabsf(target_diff) < TRAJECTORY_CONTINUOUS_MIN_DIFF && p->is_moving) {
        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 0;
    }

    float pos_diff = target_position - p->current_position;

    bool new_direction = (pos_diff >= 0.0f);

    p->target_position = target_position;
    p->target_updated = true;

    if (p->is_moving && (new_direction != p->direction)) {
        /* V13.0: 正弦波优化 - 改进换向处理，确保平滑过渡 */
        float current_speed = p->current_velocity;
        float d_max = p->dynamic_d_max;
        float braking_dist = Trajectory_CalcBrakingDistance(current_speed, d_max);

        /* 检查是否接近极值点 */
        ExtremePointInfo_t extreme_info;
        detect_extreme_point(motor_id, &extreme_info);
        
        if (extreme_info.is_extreme_point || braking_dist >= fabsf(p->remaining_distance)) {
            /* V13.0: 在极值点或需要完全停止：减速到接近0再换向 */
            p->state = TRAJ_STATE_DECELERATING;
            /* 使用更激进的减速 */
            p->current_acceleration = -d_max * 0.9f;
            p->direction = new_direction;
            /* V13.0: 极值点附近速度降到更低，确保平滑过渡 */
            p->current_velocity = 5.0f;  /* 降到5度/秒，接近静止再换向 */
        } else {
            /* V13.0: 距离足够，平滑过渡 */
            p->direction = new_direction;
            p->remaining_distance = target_position - p->current_position;
            /* V13.0: 方向切换时速度降到更低，避免冲击 */
            p->current_velocity = 15.0f;  /* 固定较低的初始速度 */
            p->state = TRAJ_STATE_ACCELERATING;
            /* V13.0: 换向后使用较低的初始加速度 */
            p->current_acceleration = p->dynamic_a_max * 0.5f;
        }
    } else if (p->is_moving) {
        /* V11.0: 同方向运动，动态调整速度 */
        p->remaining_distance = pos_diff;
        p->direction = new_direction;
        /* 根据剩余距离动态调整速度限制 */
        float dist_abs = fabsf(pos_diff);
        if (dist_abs < 10.0f) {
            /* 接近目标，降低速度 */
            p->dynamic_v_max = fminf(p->config.v_max, dist_abs * 50.0f);
        } else {
            p->dynamic_v_max = p->config.v_max;
        }
    } else {
        p->remaining_distance = fabsf(target_position - p->current_position);
        p->direction = new_direction;
        /* V13.0: 启动时使用较低的初始速度 */
        p->current_velocity = 20.0f;
        p->state = TRAJ_STATE_ACCELERATING;
        p->current_acceleration = p->dynamic_a_max * 0.8f;
    }

    p->is_moving = true;
    p->tick_count = 0;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_Step(uint8_t motor_id, TrajectoryOutput_t* output)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || output == NULL) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    float dt = TRAJECTORY_CONTROL_PERIOD;
    
    /* V4.0: 使用预测性轨迹规划算法 */
    if (g_trajectory_global.use_predictive_planning && p->is_moving) {
        generate_smooth_trajectory(motor_id, p->target_position, 0.0f, dt, output);
        
        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 0;
    }

    /* V3.0: 原有的状态机算法（作为备用）*/
    
    /* 使用动态速度/加速度参数 */
    float v_max = p->dynamic_v_max;
    float a_max = p->dynamic_a_max;
    float d_max = p->dynamic_d_max;

    /* 更新剩余距离 */
    p->remaining_distance = p->target_position - p->current_position;
    float abs_remaining = fabsf(p->remaining_distance);

    /* 计算减速距离（当前速度下需要的最小停止距离）*/
    /* V3.0 改进：速度始终为正，不需要fabsf */
    float braking_dist = Trajectory_CalcBrakingDistance(
        p->current_velocity,
        d_max
    );

    /* 状态机 */
    bool use_s_curve_this_step = false;
    
    switch (p->state) {
        case TRAJ_STATE_IDLE:
            /* 空闲状态，检查是否有新目标 */
            if (p->is_moving && abs_remaining > p->config.pos_threshold) {
                p->state = TRAJ_STATE_ACCELERATING;
            }
            break;

        case TRAJ_STATE_ACCELERATING:
            {
                /* 优先使用S曲线规划（如果启用）*/
                if (p->s_curve.use_s_curve && p->config.jerk_limit > 0.0f) {
                    float pos_delta = s_curve_step(p, dt, p->direction);
                    p->current_position += pos_delta;
                    use_s_curve_this_step = true;
                    
                    /* 检查S曲线是否完成 */
                    if (!p->s_curve.use_s_curve) {
                        p->state = TRAJ_STATE_STOPPING;
                    }
                } else {
                    /* 检查是否需要开始减速（使用动态减速度）*/
                    if (abs_remaining <= braking_dist + p->config.pos_threshold) {
                        p->state = TRAJ_STATE_DECELERATING;
                        p->current_acceleration = -d_max;  /* V3.0: 速度始终为正，减速时加速度为负 */
                    }
                    /* 检查是否达到最大速度（使用动态速度）*/
                    else if (p->current_velocity >= v_max) {
                        p->state = TRAJ_STATE_CRUISING;
                        p->current_velocity = v_max;  /* V3.0: 速度始终为正 */
                        p->current_acceleration = 0.0f;
                    }
                    else {
                        /* 继续加速（使用动态加速度）*/
                        p->current_acceleration = a_max;  /* V3.0: 加速时加速度为正 */
                    }
                }
            }
            break;

        case TRAJ_STATE_CRUISING:
            {
                /* 匀速运动，检查是否需要开始减速（使用动态减速度）*/
                if (abs_remaining <= braking_dist + p->config.pos_threshold) {
                    p->state = TRAJ_STATE_DECELERATING;
                    p->current_acceleration = -d_max;  /* V3.0: 速度始终为正，减速时加速度为负 */
                } else {
                    p->current_acceleration = 0.0f;
                }
            }
            break;

        case TRAJ_STATE_DECELERATING:
            {
                /* 减速阶段（使用动态减速度）*/
                p->current_acceleration = -d_max;  /* V3.0: 速度始终为正，减速时加速度为负 */

                /* 检查是否接近停止 - 使用"且"条件确保同时满足速度和位置要求 */
                bool velocity_low = p->current_velocity < p->config.vel_threshold;  /* V3.0: 速度始终为正 */
                bool position_reached = abs_remaining < p->config.pos_threshold;
                bool overshoot_detected = (p->direction && p->remaining_distance < 0.0f) ||
                                          (!p->direction && p->remaining_distance > 0.0f);
                
                if ((velocity_low && position_reached) || overshoot_detected) {
                    p->state = TRAJ_STATE_STOPPING;

                    /* 到达目标后，检查前瞻缓冲是否有下一个目标 */
                    if (p->lookahead_count > 0) {
                        /* 取出下一个目标 */
                        float next_target = p->lookahead_positions[p->lookahead_index];
                        p->lookahead_index++;
                        if (p->lookahead_index >= TRAJECTORY_LOOKAHEAD_COUNT) {
                            p->lookahead_index = 0;
                        }
                        p->lookahead_count--;

                        /* 设置为新目标 */
                        p->target_position = next_target;
                        p->remaining_distance = next_target - p->current_position;
                        p->direction = (p->remaining_distance >= 0.0f);
                        p->is_moving = true;
                        p->state = TRAJ_STATE_ACCELERATING;

                        /* 恢复默认参数（等待新的前瞻检测）*/
                        p->dynamic_v_max = p->config.v_max;
                        p->dynamic_a_max = p->config.a_max;
                        p->dynamic_d_max = p->config.d_max;
                    }
                }
            }
            break;

        case TRAJ_STATE_STOPPING:
            {
                /* V3.0 改进：速度始终为正，停止阶段只需减速到0 */
                float stop_decel = p->config.d_max * 2.0f;
                
                /* 预测下一周期速度 */
                float next_velocity = p->current_velocity - stop_decel * dt;
                
                /* 检查速度是否会过零或已经很小 */
                if (next_velocity <= 0.0f || p->current_velocity < 0.5f) {
                    /* 速度即将过零或已经很小，直接停止 */
                    p->current_velocity = 0.0f;
                    p->current_acceleration = 0.0f;
                    /* 关键修复：不直接跳到目标位置，而是保持在当前位置 */
                    /* 位置范围检查 */
                    if (p->current_position < 0.0f) p->current_position = 0.0f;
                    if (p->current_position > 359.99f) p->current_position = 359.99f;
                    p->is_moving = false;
                    p->state = TRAJ_STATE_COMPLETED;
                    
                    /* 清除S曲线状态 */
                    p->s_curve.use_s_curve = false;
                } else {
                    /* 正常减速 */
                    p->current_acceleration = -stop_decel;  /* V3.0: 减速时加速度为负 */
                }
            }
            break;

        case TRAJ_STATE_COMPLETED:
            {
                /* 已完成 - 优化：保留当前速度供下次目标使用 */
                /* 只有在真正需要停止时才清零 */
                p->current_acceleration = 0.0f;
                p->is_moving = false;

                /* 检查是否有新目标（目标与当前位置差距较大）*/
                if (abs_remaining > p->config.pos_threshold * 2.0f) {
                    /* 新目标存在，状态设为IDLE等待处理
                     * 不在这里清零速度，让Fusion层处理时决定是否保留 */
                    p->state = TRAJ_STATE_IDLE;
                    p->is_moving = true;  // 标记为需要运动
                } else {
                    /* 确实到达目标且无新目标，可以停止 */
                    p->current_velocity = 0.0f;
                }
            }
            break;

        default:
            p->state = TRAJ_STATE_IDLE;
            break;
    }

    /* 积分更新位置和速度（S曲线模式跳过，因为已在s_curve_step中更新）*/
    if (!use_s_curve_this_step) {
        /* V3.0 改进：速度始终为非负值，方向由 direction 变量控制 */
        p->current_velocity += p->current_acceleration * dt;

        /* 速度限幅 - 始终为非负值 [0, v_max] */
        p->current_velocity = clamp(p->current_velocity, 0.0f, p->dynamic_v_max);

        /* 位置更新 - 根据方向决定位置变化 */
        float direction_sign = p->direction ? 1.0f : -1.0f;
        p->current_position += direction_sign * p->current_velocity * dt;
    }

    /* 填充输出 */
    /* 关键修复：添加位置范围检查和超调保护 */
    if (p->current_position < 0.0f) p->current_position = 0.0f;
    if (p->current_position > 359.99f) p->current_position = 359.99f;
    
    /* 超调检测：如果运动方向与剩余距离方向相反，说明已经超调 */
    float remaining = p->target_position - p->current_position;
    if ((p->direction && remaining < 0.0f) || (!p->direction && remaining > 0.0f)) {
        /* 已经超调，强制设置到目标位置 */
        p->current_position = p->target_position;
        p->current_velocity = 0.0f;
        p->current_acceleration = 0.0f;
        p->filtered_acceleration = 0.0f;  /* V5.5: 重置滤波加速度 */
        p->state = TRAJ_STATE_COMPLETED;
    }
    
    /* V5.5: 加速度平滑滤波 */
    p->filtered_acceleration = ACCEL_FILTER_ALPHA * p->filtered_acceleration + 
                               (1.0f - ACCEL_FILTER_ALPHA) * p->current_acceleration;
    
    /* V13.0: 速度和加速度始终为正（绝对值），方向由direction变量控制 */
    output->position = p->current_position;
    output->velocity = p->current_velocity;  /* V13.0: 始终为正 */
    output->acceleration = p->filtered_acceleration;  /* V13.0: 始终为正 */
    output->is_completed = (p->state == TRAJ_STATE_COMPLETED);
    output->state = p->state;
    output->direction = p->direction;  /* V13.0: 输出方向标志 */
    
    /* 计算前馈控制输出 - V13.0: 前馈始终为正 */
    {
        /* 速度前馈 */
        output->feedforward_vel = g_trajectory_global.ff_config.vel_ff_gain * p->current_velocity;
        
        /* 加速度前馈 */
        output->feedforward_accel = g_trajectory_global.ff_config.accel_ff_gain * p->current_acceleration;
        
        /* PID反馈补偿（用于位置误差校正）*/
        float pos_error = p->target_position - p->current_position;
        
        /* 积分项（带抗饱和）*/
        g_trajectory_global.vel_integral[motor_id] += pos_error * dt;
        g_trajectory_global.vel_integral[motor_id] = clamp(g_trajectory_global.vel_integral[motor_id], 
                                          -g_trajectory_global.ff_config.integral_limit, 
                                          g_trajectory_global.ff_config.integral_limit);
        
        /* 微分项（带低通滤波减少噪声）*/
        float alpha = 0.8f;
        float raw_deriv = (dt > 0.0001f) ? ((pos_error - g_trajectory_global.last_pos_error[motor_id]) / dt) : 0.0f;
        g_trajectory_global.filtered_deriv[motor_id] = alpha * g_trajectory_global.filtered_deriv[motor_id] + (1.0f - alpha) * raw_deriv;
        g_trajectory_global.last_pos_error[motor_id] = pos_error;

        /* PID输出 */
        float pid_output = g_trajectory_global.ff_config.pos_kp * pos_error
                         + g_trajectory_global.ff_config.vel_ki * g_trajectory_global.vel_integral[motor_id]
                         + g_trajectory_global.ff_config.vel_kd * g_trajectory_global.filtered_deriv[motor_id];
        
        /* 将PID输出叠加到速度前馈 */
        output->feedforward_vel += pid_output * 0.01f;
    }

    p->tick_count++;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_StepAll(TrajectoryOutput_t outputs[TRAJECTORY_MAX_MOTORS])
{
    if (outputs == NULL) return 0;

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < TRAJECTORY_MAX_MOTORS; i++) {
        if (Trajectory_Step(i, &outputs[i]) == 0) {
            success_count++;
        }
    }

    return success_count;
}

/* ============================================================================
 * 公共函数实现 - 状态查询
 * ============================================================================ */

uint8_t Trajectory_GetState(uint8_t motor_id, TrajectoryPlanner_t* planner)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || planner == NULL) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    memcpy(planner, &g_trajectory_global.planners[motor_id], sizeof(TrajectoryPlanner_t));

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

bool Trajectory_IsCompleted(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return true;

    return (g_trajectory_global.planners[motor_id].state == TRAJ_STATE_COMPLETED ||
            g_trajectory_global.planners[motor_id].state == TRAJ_STATE_IDLE);
}

float Trajectory_GetRemainingDistance(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 0.0f;

    return g_trajectory_global.planners[motor_id].remaining_distance;
}

float Trajectory_GetEstimatedTime(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return -1.0f;

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    if (!p->is_moving) return 0.0f;

    float abs_dist = fabsf(p->remaining_distance);
    float v_max = p->config.v_max;
    float a_max = p->config.a_max;
    float d_max = p->config.d_max;

    /* 计算三角形曲线时间 */
    float v_peak = Trajectory_CalcOptimalVelocity(abs_dist, v_max, a_max);
    
    float t_accel = (a_max > 0.001f) ? (v_peak / a_max) : 0.0f;
    float t_decel = (d_max > 0.001f) ? (v_peak / d_max) : 0.0f;
    float t_cruise = 0.0f;

    if (v_peak >= v_max) {
        float d_accel = (a_max > 0.001f) ? (v_max * v_max / (2.0f * a_max)) : 0.0f;
        float d_decel = (d_max > 0.001f) ? (v_max * v_max / (2.0f * d_max)) : 0.0f;
        float d_cruise = abs_dist - d_accel - d_decel;
        t_cruise = (v_max > 0.001f) ? (d_cruise / v_max) : 0.0f;
    }

    return t_accel + t_cruise + t_decel;
}

/* ============================================================================
 * 公共函数实现 - 高级功能
 * ============================================================================ */

uint8_t Trajectory_Precalculate(uint8_t motor_id, float target_position,
                                 float* peak_velocity, float* total_time)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 1;

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    float distance = target_position - p->current_position;

    float accel_dist, cruise_dist, decel_dist;
    calculate_trapezoid_profile(
        distance,
        p->config.v_max,
        p->config.a_max,
        p->config.d_max,
        peak_velocity,
        &accel_dist,
        &cruise_dist,
        &decel_dist
    );

    float t_accel = (*peak_velocity > 0.001f && p->config.a_max > 0.001f) ? 
                    (*peak_velocity / p->config.a_max) : 0.0f;
    float t_decel = (*peak_velocity > 0.001f && p->config.d_max > 0.001f) ? 
                    (*peak_velocity / p->config.d_max) : 0.0f;
    float t_cruise = (*peak_velocity > 0.001f) ? (cruise_dist / *peak_velocity) : 0.0f;
    *total_time = t_accel + t_cruise + t_decel;

    return 0;
}

void Trajectory_PrintStatus(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    const char* state_str[] = {"IDLE", "ACCEL", "CRUISE", "DECEL", "STOPPING", "COMPLETED"};

    printf("\r\n========== Trajectory Planner [%d] ==========\r\n", motor_id);
    printf("State:          %s\r\n", state_str[p->state]);
    printf("Target:         %.2f deg\r\n", p->target_position);
    printf("Current Pos:    %.2f deg\r\n", p->current_position);
    printf("Current Vel:    %.2f deg/s\r\n", p->current_velocity);
    printf("Current Accel:  %.2f deg/s²\r\n", p->current_acceleration);
    printf("Remaining:      %.2f deg\r\n", p->remaining_distance);
    printf("Peak Vel:       %.2f deg/s\r\n", p->peak_velocity);
    printf("Is Moving:      %s\r\n", p->is_moving ? "YES" : "NO");
    printf("Config: Vmax=%.1f, Amax=%.1f, Dmax=%.1f\r\n",
           p->config.v_max, p->config.a_max, p->config.d_max);
    printf("=============================================\r\n");
}

void Trajectory_PrintAllStatus(void)
{
    printf("\r\n========== All Trajectory Planners ==========\r\n");
    for (uint8_t i = 0; i < TRAJECTORY_MAX_MOTORS; i++) {
        TrajectoryPlanner_t* p = &g_trajectory_global.planners[i];
        printf("[%2d] %s  Pos:%7.1f  Vel:%6.1f  Tgt:%7.1f  Rem:%6.1f\r\n",
               i,
               p->is_moving ? "MOV" : "IDLE",
               p->current_position,
               p->current_velocity,
               p->target_position,
               p->remaining_distance);
    }
    printf("=============================================\r\n");
}

/* ============================================================================
 * 前瞻缓冲功能实现
 * ============================================================================ */

uint8_t Trajectory_PushLookahead(uint8_t motor_id, float target_position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 2;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    if (p->lookahead_count >= TRAJECTORY_LOOKAHEAD_COUNT) {
        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 1;  // 缓冲已满
    }

    // 添加到缓冲队列末尾
    uint8_t insert_index = p->lookahead_index + p->lookahead_count;
    if (insert_index >= TRAJECTORY_LOOKAHEAD_COUNT) {
        insert_index -= TRAJECTORY_LOOKAHEAD_COUNT;
    }

    p->lookahead_positions[insert_index] = target_position;
    p->lookahead_count++;

    float current_dir = p->target_position - p->current_position;
    float lookahead_dir = target_position - p->target_position;

    bool direction_will_change = false;
    if (fabsf(current_dir) > TRAJECTORY_POS_THRESHOLD) {
        if ((current_dir > 0.0f && lookahead_dir < 0.0f) || 
            (current_dir < 0.0f && lookahead_dir > 0.0f)) {
            direction_will_change = true;
        }
    }

    if (direction_will_change) {
        p->lookahead_direction_change = true;

        p->dynamic_v_max = p->config.v_max * 0.5f;
        p->dynamic_a_max = p->config.a_max * 0.7f;
        p->dynamic_d_max = p->config.d_max * 1.5f;
        
        printf("[Traj] Motor %u: Direction change detected! CurTgt=%.1f NextTgt=%.1f - Reducing params\r\n",
               motor_id, p->target_position, target_position);
    } else {
        p->lookahead_direction_change = false;
        p->dynamic_v_max = p->config.v_max;
        p->dynamic_a_max = p->config.a_max;
        p->dynamic_d_max = p->config.d_max;
    }

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

uint8_t Trajectory_GetLookahead(uint8_t motor_id, uint8_t index, float* target_position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS || target_position == NULL) return 2;
    if (index >= TRAJECTORY_LOOKAHEAD_COUNT) return 1;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];

    if (index >= p->lookahead_count) {
        if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
        return 1;
    }

    uint8_t read_index = p->lookahead_index + index;
    if (read_index >= TRAJECTORY_LOOKAHEAD_COUNT) {
        read_index -= TRAJECTORY_LOOKAHEAD_COUNT;
    }

    *target_position = p->lookahead_positions[read_index];

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return 0;
}

bool Trajectory_CheckDirectionChange(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return false;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    bool result = g_trajectory_global.planners[motor_id].lookahead_direction_change;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return result;
}

void Trajectory_ClearLookahead(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    TrajectoryPlanner_t* p = &g_trajectory_global.planners[motor_id];
    p->lookahead_count = 0;
    p->lookahead_index = 0;
    p->lookahead_direction_change = false;

    // 恢复默认速度/加速度
    p->dynamic_v_max = p->config.v_max;
    p->dynamic_a_max = p->config.a_max;
    p->dynamic_d_max = p->config.d_max;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
}

void Trajectory_ClearAllLookahead(void)
{
    for (uint8_t i = 0; i < TRAJECTORY_MAX_MOTORS; i++) {
        Trajectory_ClearLookahead(i);
    }
}

/* ============================================================================
 * 高级补偿功能
 * ============================================================================ */

/* 反向间隙补偿配置 - V8.0 添加极值点因子 */
static BacklashConfig_t g_backlash_config[TRAJECTORY_MAX_MOTORS] = {
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 0 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 1 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 2 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 3 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 4 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 5 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 6 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 7 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 8 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 9 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 10 - V8.0
    {.backlash_compensation = 1.5f, .last_direction = 0.0f, .extreme_point_factor = 0.3f},  // Motor 11 - V8.0
};

/* 摩擦力补偿配置 */
static FrictionConfig_t g_friction_config[TRAJECTORY_MAX_MOTORS] = {
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 0
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 1
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 2
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 3
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 4
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 5
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 6
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 7
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 8
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 9
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 10
    {.coulomb_friction = 0.5f, .viscous_friction = 0.01f},  // Motor 11
};

/**
 * @brief V14.0: 设置规划器当前位置（用于闭环修正）
 * @param motor_id 电机 ID
 * @param position 当前位置 (deg)
 */
void Trajectory_SetCurrentPosition(uint8_t motor_id, float position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    g_trajectory_global.planners[motor_id].current_position = clamp(position, 0.0f, 359.99f);
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
}

/**
 * @brief 设置反向间隙补偿参数
 * @param motor_id 电机 ID
 * @param backlash 反向间隙补偿量 (deg)
 */
void Trajectory_SetBacklashCompensation(uint8_t motor_id, float backlash)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    g_backlash_config[motor_id].backlash_compensation = backlash;
    g_backlash_config[motor_id].last_direction = 0.0f;  // 重置方向状态
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    printf("[Backlash] Motor %u: backlash compensation set to %.2f deg\r\n", 
           motor_id, backlash);
}

/**
 * @brief 设置摩擦力补偿参数
 * @param motor_id 电机 ID
 * @param coulomb 库伦摩擦力矩 (Nm)
 * @param viscous 粘性摩擦系数 (Nm/(rad/s))
 */
void Trajectory_SetFrictionCompensation(uint8_t motor_id, float coulomb, float viscous)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    g_friction_config[motor_id].coulomb_friction = coulomb;
    g_friction_config[motor_id].viscous_friction = viscous;
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    printf("[Friction] Motor %u: friction compensation set (coulomb=%.3f, viscous=%.4f)\r\n", 
           motor_id, coulomb, viscous);
}

/**
 * @brief 应用反向间隙补偿
 * @param motor_id 电机 ID
 * @param target_position 目标位置
 * @param current_position 当前位置
 * @return 补偿后的目标位置
 */
float Trajectory_ApplyBacklashCompensation(uint8_t motor_id, float target_position, float current_position)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return target_position;
    
    float current_direction = (target_position > current_position) ? 1.0f : -1.0f;
    float compensated_target = target_position;
    
    // V8.0: 检测极值点 - 距离目标很近时减少补偿
    float distance_to_target = fabsf(target_position - current_position);
    float effective_factor = 1.0f;
    if (distance_to_target < 25.0f) {
        effective_factor = g_backlash_config[motor_id].extreme_point_factor;  // 极值点折扣
    }
    
    // 检测方向切换
    if (current_direction != g_backlash_config[motor_id].last_direction && 
        g_backlash_config[motor_id].last_direction != 0.0f) {
        
        // V8.0: 添加反向间隙补偿（带极值点折扣）
        float compensation = g_backlash_config[motor_id].backlash_compensation * effective_factor;
        compensated_target += current_direction * compensation;
        
        // V12.0: 限频输出日志，避免阻塞控制循环
        static uint32_t g_backlash_log_counter = 0;
        static uint32_t g_backlash_log_tick = 0;
        uint32_t current_tick = HAL_GetTick();
        
        g_backlash_log_counter++;
        if ((current_tick - g_backlash_log_tick) > 5000) {  // 每5秒输出一次汇总
            if (g_backlash_log_counter > 0) {
                printf("[Backlash] %lu extreme point compensations in last 5s\r\n", 
                       (unsigned long)g_backlash_log_counter);
            }
            g_backlash_log_counter = 0;
            g_backlash_log_tick = current_tick;
        }
    }
    
    // 更新方向状态
    g_backlash_config[motor_id].last_direction = current_direction;
    
    return compensated_target;
}

/**
 * @brief 计算摩擦力补偿力矩
 * @param motor_id 电机 ID
 * @param velocity 当前速度 (deg/s)
 * @param torque_command 原始力矩指令 (Nm)
 * @return 补偿后的力矩指令 (Nm)
 */
float Trajectory_ApplyFrictionCompensation(uint8_t motor_id, float velocity, float torque_command)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return torque_command;
    
    float friction_torque = 0.0f;
    
    // 库伦摩擦 (方向相关)
    if (velocity > 0.1f) {
        friction_torque += g_friction_config[motor_id].coulomb_friction;
    } else if (velocity < -0.1f) {
        friction_torque -= g_friction_config[motor_id].coulomb_friction;
    }
    
    // 粘性摩擦
    // 注意：速度单位转换 deg/s -> rad/s
    float velocity_rad = velocity * M_PI / 180.0f;
    friction_torque += g_friction_config[motor_id].viscous_friction * velocity_rad;
    
    return torque_command + friction_torque;
}

/**
 * @brief 获取补偿配置信息
 * @param motor_id 电机 ID
 * @param backlash 输出反向间隙补偿量
 * @param coulomb 出库伦摩擦力矩
 * @param viscous 输出粘性摩擦系数
 */
void Trajectory_GetCompensationConfig(uint8_t motor_id, float* backlash, float* coulomb, float* viscous)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    if (backlash) *backlash = g_backlash_config[motor_id].backlash_compensation;
    if (coulomb) *coulomb = g_friction_config[motor_id].coulomb_friction;
    if (viscous) *viscous = g_friction_config[motor_id].viscous_friction;
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
}

uint8_t Trajectory_GetLookaheadCount(uint8_t motor_id)
{
    if (motor_id >= TRAJECTORY_MAX_MOTORS) return 0;

    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);

    uint8_t count = g_trajectory_global.planners[motor_id].lookahead_count;

    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);

    return count;
}

uint8_t Trajectory_SetExtremePointConfig(uint8_t motor_id, const ExtremePointConfig_t* config)
{
    if (config == NULL) return 1;
    
    if (!IS_VALID_FLOAT(config->extreme_distance_near) || config->extreme_distance_near < 0) return 2;
    if (!IS_VALID_FLOAT(config->extreme_distance_mid) || config->extreme_distance_mid < config->extreme_distance_near) return 3;
    if (!IS_VALID_FLOAT(config->extreme_distance_far) || config->extreme_distance_far < config->extreme_distance_mid) return 4;
    if (!IS_VALID_FLOAT(config->vel_reduction_near) || config->vel_reduction_near < 0 || config->vel_reduction_near > 1) return 5;
    if (!IS_VALID_FLOAT(config->vel_reduction_mid) || config->vel_reduction_mid < config->vel_reduction_near || config->vel_reduction_mid > 1) return 6;
    if (!IS_VALID_FLOAT(config->accel_reduction_near) || config->accel_reduction_near < 0 || config->accel_reduction_near > 1) return 7;
    if (!IS_VALID_FLOAT(config->accel_reduction_mid) || config->accel_reduction_mid < config->accel_reduction_near || config->accel_reduction_mid > 1) return 8;
    if (!IS_VALID_FLOAT(config->speed_ratio_threshold) || config->speed_ratio_threshold < 0 || config->speed_ratio_threshold > 1) return 9;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    g_trajectory_global.extreme_config = *config;
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    LOG_INFO("[Trajectory] Extreme point config updated: near=%.1f, mid=%.1f, far=%.1f deg",
             config->extreme_distance_near, config->extreme_distance_mid, config->extreme_distance_far);
    
    return 0;
}

uint8_t Trajectory_GetExtremePointConfig(uint8_t motor_id, ExtremePointConfig_t* config)
{
    if (config == NULL) return 1;
    
    if (g_trajectory_global.mutex) osMutexAcquire(g_trajectory_global.mutex, osWaitForever);
    
    *config = g_trajectory_global.extreme_config;
    
    if (g_trajectory_global.mutex) osMutexRelease(g_trajectory_global.mutex);
    
    return 0;
}
