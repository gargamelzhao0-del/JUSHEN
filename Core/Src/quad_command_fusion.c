/**
 ****************************************************************************************************
 * @file        quad_command_fusion.c
 * @version     V2.0
 * @brief       四足机器狗 - 指令融合层实现（集成轨迹规划）
 * @details     融合底层CANFD指令，优化总线负载，提供高层控制接口
 *              V2.0: 集成轨迹规划器，实现平滑运动控制
 ****************************************************************************************************
 */

#include "quad_command_fusion.h"
#include "motor_control.h"
#include "trajectory_planner.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * 私有类型定义
 * ============================================================================ */

/**
 * @brief 参数缓存结构（避免重复发送相同参数）
 */
typedef struct {
    float torque_limit;     /* 缓存的力矩限制 */
    float speed_limit;      /* 缓存的速度限制 */
    float accel_limit;      /* 缓存的加速度限制 */
    float pos_kp;           /* 缓存的位置Kp */
    uint8_t control_mode;   /* 缓存的控制模式 */
    float pos_limit_plus;   /* 缓存的位置上限 */
    float pos_limit_minus;  /* 缓存的位置下限 */
    float target_force;     /* 缓存的目标力矩 */
    float torque_slope;     /* 缓存的力矩斜率 */
    bool valid;             /* 缓存是否有效 */
    uint32_t last_update_tick;  /* 最后更新时间戳 */
    uint8_t error_count;    /* 连续错误计数 */
} ParamCache_t;

/**
 * @brief 统计信息结构
 */
typedef struct {
    uint32_t total_commands;    /* 总指令调用数 */
    uint32_t total_frames;      /* 实际发送的CANFD帧数 */
    uint32_t cache_hits;        /* 缓存命中次数 */
} FusionStats_t;

/* ============================================================================
 * 私有全局变量
 * ============================================================================ */

/* 参数缓存数组（12个电机）*/
static ParamCache_t g_param_cache[QUAD_MAX_MOTORS] = {0};

/**
 * @brief 前瞻缓冲历史数据（用于趋势预测）
 */
typedef struct {
    float target_positions[3];  /* 最近3帧目标位置 [最新, 前1帧, 前2帧] */
    uint32_t timestamps[3];    /* 对应的时间戳 */
    uint8_t valid_count;        /* 有效数据数量 */
    uint8_t write_index;        /* 写入索引 */
} LookaheadHistory_t;

/**
 * @brief 大历史缓冲区（用于曲线拟合和曲率计算）
 */
#define HISTORY_BUFFER_SIZE  32

typedef struct {
    float positions[HISTORY_BUFFER_SIZE];  /* 历史位置数据 */
    uint32_t timestamps[HISTORY_BUFFER_SIZE]; /* 时间戳 */
    uint8_t write_idx;       /* 写入位置 */
    uint8_t count;           /* 有效数据数量 */
    float curvature;         /* 计算得到的曲率 */
    float velocity_avg;      /* 平均速度 */
    float acceleration_avg;  /* 平均加速度 */
} HistoryBuffer_t;

/* 前瞻缓冲历史数据（12个电机）*/
static LookaheadHistory_t g_lookahead_history[QUAD_MAX_MOTORS] = {0};

/* 大历史缓冲区（12个电机）*/
static HistoryBuffer_t g_history_buffer[QUAD_MAX_MOTORS] = {0};

/* 统计信息 */
static FusionStats_t g_stats = {0};

/* 初始化标志 */
static bool g_fusion_initialized = false;

/* 控制模式（运行时可切换）*/
static uint8_t g_control_mode = QUAD_CONTROL_MODE;

/* ============================================================================
 * 私有辅助函数
 * ============================================================================ */

/**
 * @brief 比较浮点数是否相等（容差0.01）
 */
static inline bool float_equal(float a, float b)
{
    return (a > b - 0.01f) && (a < b + 0.01f);
}

/**
 * @brief 检查参数是否需要更新
 * @param motor_id 电机索引
 * @param param_type 参数类型
 * @param value 新值
 * @return true=需要更新, false=缓存命中
 */
static bool need_update(uint8_t motor_id, const char* param_type, float value)
{
    if (motor_id >= QUAD_MAX_MOTORS) return false;

    ParamCache_t* cache = &g_param_cache[motor_id];

    if (!cache->valid) {
        return true;
    }

    /* 缓存超时检测：超过5秒自动失效 */
    uint32_t current_tick = osKernelGetTickCount();
    if (current_tick - cache->last_update_tick > 5000) {
        cache->valid = false;
        return true;
    }

    /* 连续错误检测：超过3次错误自动失效 */
    if (cache->error_count > 3) {
        cache->valid = false;
        cache->error_count = 0;
        return true;
    }

    /* 根据参数类型检查缓存 */
    if (strcmp(param_type, "torque_limit") == 0) {
        return !float_equal(cache->torque_limit, value);
    } else if (strcmp(param_type, "speed_limit") == 0) {
        return !float_equal(cache->speed_limit, value);
    } else if (strcmp(param_type, "accel_limit") == 0) {
        return !float_equal(cache->accel_limit, value);
    } else if (strcmp(param_type, "pos_kp") == 0) {
        return !float_equal(cache->pos_kp, value);
    }

    return true;
}

/**
 * @brief 更新参数缓存
 */
static void update_cache(uint8_t motor_id, const char* param_type, float value)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    ParamCache_t* cache = &g_param_cache[motor_id];
    cache->last_update_tick = osKernelGetTickCount();

    if (strcmp(param_type, "torque_limit") == 0) {
        cache->torque_limit = value;
    } else if (strcmp(param_type, "speed_limit") == 0) {
        cache->speed_limit = value;
    } else if (strcmp(param_type, "accel_limit") == 0) {
        cache->accel_limit = value;
    } else if (strcmp(param_type, "pos_kp") == 0) {
        cache->pos_kp = value;
    }

    cache->valid = true;
}

/**
 * @brief 趋势分析结果
 */
typedef enum {
    TREND_ACCELERATING,    /* 加速趋势 */
    TREND_DECELERATING,    /* 减速趋势 */
    TREND_REVERSING,       /* 反向趋势 */
    TREND_STABLE,          /* 稳定趋势 */
    TREND_UNKNOWN          /* 数据不足 */
} TrendDirection_t;

/**
 * @brief 添加目标位置到历史缓冲
 */
static void Lookahead_PushHistory(uint8_t motor_id, float target_pos)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    LookaheadHistory_t* hist = &g_lookahead_history[motor_id];

    /* 移动历史数据 */
    hist->target_positions[2] = hist->target_positions[1];
    hist->target_positions[1] = hist->target_positions[0];
    hist->target_positions[0] = target_pos;

    hist->timestamps[2] = hist->timestamps[1];
    hist->timestamps[1] = hist->timestamps[0];
    hist->timestamps[0] = osKernelGetTickCount();

    /* 更新有效计数 */
    if (hist->valid_count < 3) {
        hist->valid_count++;
    }
}

/**
 * @brief 分析运动趋势（基于历史数据）
 * @return 趋势方向
 */
static TrendDirection_t Lookahead_AnalyzeTrend(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return TREND_UNKNOWN;

    LookaheadHistory_t* hist = &g_lookahead_history[motor_id];

    /* 需要至少2帧数据才能分析趋势 */
    if (hist->valid_count < 2) {
        return TREND_UNKNOWN;
    }

    float latest = hist->target_positions[0];     /* 最新 */
    float prev1 = hist->target_positions[1];      /* 前1帧 */
    float prev2 = hist->target_positions[2];     /* 前2帧 */

    /* 计算变化趋势 */
    float delta1 = latest - prev1;    /* 最新-前1 */
    float delta2 = prev1 - prev2;     /* 前1-前2 */

    /* V3.1 优化：增大趋势判断阈值，避免噪声误判 */
    /* 50Hz下20ms间隔，0.5°太小，改为2.0° */
    const float TREND_THRESHOLD = 2.0f;

    /* 判断趋势 */
    if (delta1 > TREND_THRESHOLD && delta2 > TREND_THRESHOLD) {
        /* 连续正向增大 → 加速 */
        return TREND_ACCELERATING;
    } else if (delta1 < -TREND_THRESHOLD && delta2 < -TREND_THRESHOLD) {
        /* 连续反向减小 → 加速 */
        return TREND_ACCELERATING;
    } else if ((delta1 > TREND_THRESHOLD && delta2 < -TREND_THRESHOLD) || 
               (delta1 < -TREND_THRESHOLD && delta2 > TREND_THRESHOLD)) {
        /* 方向改变 → 反向 */
        return TREND_REVERSING;
    } else if ((delta1 > 0 && delta2 > 0) || (delta1 < 0 && delta2 < 0)) {
        /* 同向但减小 → 减速 */
        if (fabsf(delta1) < fabsf(delta2)) {
            return TREND_DECELERATING;
        }
    }

    return TREND_STABLE;
}

/**
 * @brief 根据趋势预调整速度/加速度参数
 * @param motor_id 电机ID
 * @param current_speed 当前速度限制
 * @param current_accel 当前加速度限制
 * @param p_new_speed 输出：新的速度限制
 * @param p_new_accel 输出：新的加速度限制
 */
static void Lookahead_AdjustParams(uint8_t motor_id,
                                   float current_speed, float current_accel,
                                   float* p_new_speed, float* p_new_accel)
{
    TrendDirection_t trend = Lookahead_AnalyzeTrend(motor_id);

    switch (trend) {
        case TREND_ACCELERATING:
            /* 加速趋势：提高速度限制，允许快速加速 */
            *p_new_speed = current_speed * 1.2f;
            *p_new_accel = current_accel * 1.3f;
            break;

        case TREND_DECELERATING:
            /* 减速趋势：降低速度限制，提前减速 */
            *p_new_speed = current_speed * 0.6f;
            *p_new_accel = current_accel * 0.7f;
            break;

        case TREND_REVERSING:
            /* 反向趋势：大幅降低速度，提高减速度 */
            *p_new_speed = current_speed * 0.3f;
            *p_new_accel = current_accel * 0.5f;
            break;

        case TREND_STABLE:
            /* 稳定趋势：保持当前参数 */
            *p_new_speed = current_speed;
            *p_new_accel = current_accel;
            break;

        case TREND_UNKNOWN:
        default:
            /* 数据不足，使用默认参数 */
            *p_new_speed = current_speed;
            *p_new_accel = current_accel;
            break;
    }
}

/**
 * @brief 添加数据到大历史缓冲区（FIFO：丢弃最旧数据）
 */
static void HistoryBuffer_Push(uint8_t motor_id, float position)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    HistoryBuffer_t* buf = &g_history_buffer[motor_id];

    /* 写入新数据到当前位置 */
    buf->positions[buf->write_idx] = position;
    buf->timestamps[buf->write_idx] = osKernelGetTickCount();

    /* 更新写入索引（环形缓冲区）*/
    buf->write_idx = (buf->write_idx + 1) % HISTORY_BUFFER_SIZE;

    /* 更新有效数据数量 */
    if (buf->count < HISTORY_BUFFER_SIZE) {
        buf->count++;
    }
}

/**
 * @brief 计算曲率（基于二阶差分）
 * @return 曲率值（正值=正向加速，负值=反向加速，绝对值越大=变化越剧烈）
 */
static float HistoryBuffer_CalculateCurvature(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return 0.0f;

    HistoryBuffer_t* buf = &g_history_buffer[motor_id];

    /* 需要至少3帧数据才能计算曲率 */
    if (buf->count < 3) {
        return 0.0f;
    }

    /* 获取最近的数据点 */
    uint8_t idx_latest = (buf->write_idx + HISTORY_BUFFER_SIZE - 1) % HISTORY_BUFFER_SIZE;
    uint8_t idx_prev1 = (buf->write_idx + HISTORY_BUFFER_SIZE - 2) % HISTORY_BUFFER_SIZE;
    uint8_t idx_prev2 = (buf->write_idx + HISTORY_BUFFER_SIZE - 3) % HISTORY_BUFFER_SIZE;

    float p0 = buf->positions[idx_latest];  /* 最新 */
    float p1 = buf->positions[idx_prev1];  /* 前1 */
    float p2 = buf->positions[idx_prev2];  /* 前2 */

    /* 计算二阶差分（曲率近似）*/
    float dt = 20.0f;  /* 50Hz = 20ms */
    float v1 = (p0 - p1) / dt;   /* v(t) */
    float v2 = (p1 - p2) / dt;   /* v(t-1) */
    float curvature = (v1 - v2) / dt;  /* a(t) = dv/dt */

    return curvature;
}

/**
 * @brief 计算平均速度和加速度
 */
static void HistoryBuffer_CalculateMotion(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    HistoryBuffer_t* buf = &g_history_buffer[motor_id];

    if (buf->count < 2) {
        buf->velocity_avg = 0.0f;
        buf->acceleration_avg = 0.0f;
        return;
    }

    /* 使用最近N帧计算平均速度 */
    uint8_t n = (buf->count < 8) ? buf->count : 8;
    float total_velocity = 0.0f;

    for (uint8_t i = 0; i < n - 1; i++) {
        uint8_t idx1 = (buf->write_idx + HISTORY_BUFFER_SIZE - 1 - i) % HISTORY_BUFFER_SIZE;
        uint8_t idx2 = (buf->write_idx + HISTORY_BUFFER_SIZE - 2 - i) % HISTORY_BUFFER_SIZE;

        float delta_pos = buf->positions[idx1] - buf->positions[idx2];
        float delta_time = 20.0f;  /* 50Hz */

        total_velocity += delta_pos / delta_time;
    }

    buf->velocity_avg = total_velocity / (n - 1);

    /* 计算平均加速度 */
    buf->acceleration_avg = HistoryBuffer_CalculateCurvature(motor_id);
}

/**
 * @brief 根据曲率调整前瞻预测参数
 * @param motor_id 电机ID
 * @param base_v_max 基础最大速度
 * @param base_a_max 基础最大加速度
 * @param p_out_v_max 输出：调整后的最大速度
 * @param p_out_a_max 输出：调整后的最大加速度
 * @param p_out_d_max 输出：调整后的最大减速度
 */
static void HistoryBuffer_AdjustByCurvature(uint8_t motor_id,
                                            float base_v_max, float base_a_max,
                                            float* p_out_v_max,
                                            float* p_out_a_max,
                                            float* p_out_d_max)
{
    /* 先更新曲率计算 */
    float curvature = HistoryBuffer_CalculateCurvature(motor_id);
    g_history_buffer[motor_id].curvature = curvature;

    HistoryBuffer_CalculateMotion(motor_id);

    /* 根据曲率调整参数 */
    float curvature_abs = fabsf(curvature);

    /* 曲率阈值判断 */
    if (curvature_abs < 1.0f) {
        /* 曲率很小：曲线平缓，可以提高响应速度 */
        *p_out_v_max = base_v_max * 1.2f;
        *p_out_a_max = base_a_max * 1.3f;
        *p_out_d_max = base_a_max * 1.0f;
    } else if (curvature_abs < 5.0f) {
        /* 中等曲率：保持正常 */
        *p_out_v_max = base_v_max * 1.0f;
        *p_out_a_max = base_a_max * 1.0f;
        *p_out_d_max = base_a_max * 1.2f;
    } else if (curvature_abs < 10.0f) {
        /* 较大曲率：变化较剧烈，需要提前减速 */
        *p_out_v_max = base_v_max * 0.7f;
        *p_out_a_max = base_a_max * 0.8f;
        *p_out_d_max = base_a_max * 1.5f;
    } else {
        /* 很大曲率：变化非常剧烈，大幅降低速度 */
        *p_out_v_max = base_v_max * 0.4f;
        *p_out_a_max = base_a_max * 0.5f;
        *p_out_d_max = base_a_max * 2.0f;
    }

    /* 方向判断：如果是反向（曲率为负），进一步降低参数 */
    if (curvature < -1.0f) {
        *p_out_v_max *= 0.7f;
        *p_out_a_max *= 0.7f;
        *p_out_d_max *= 1.5f;
    }
}

/* ============================================================================
 * 公共函数实现 - 初始化
 * ============================================================================ */

/**
 * @brief 初始化指令融合系统
 */
uint8_t Fusion_Init(void)
{
    if (g_fusion_initialized) {
        printf("[Fusion] Already initialized\r\n");
        return 0;
    }

    /* 清空缓存 */
    memset(g_param_cache, 0, sizeof(g_param_cache));

    /* 清空统计 */
    memset(&g_stats, 0, sizeof(g_stats));

    /* 初始化轨迹规划器 */
    Trajectory_Init();

    g_fusion_initialized = true;

    printf("[Fusion] Command fusion system initialized (with trajectory planner)\r\n");
    return 0;
}

/**
 * @brief 反初始化指令融合系统
 */
void Fusion_Deinit(void)
{
    /* 反初始化轨迹规划器 */
    Trajectory_Deinit();
    
    g_fusion_initialized = false;
    printf("[Fusion] Command fusion system deinitialized\r\n");
}

/* ============================================================================
 * 公共函数实现 - 摆动相控制
 * ============================================================================ */

/**
 * @brief 发送摆动相融合指令（带轨迹规划）
 * @note V2.1: 只设置目标位置，轨迹规划在控制循环中执行
 *       这样确保轨迹规划器以500Hz频率运行
 */
uint8_t Fusion_SendSwingCommand(uint8_t motor_id, float target_pos)
{
    if (!g_fusion_initialized) {
        printf("[Fusion] ERROR: Not initialized\r\n");
        return 1;
    }

    if (motor_id >= QUAD_MAX_MOTORS) {
        return 1;
    }

    uint8_t frames_sent = 0;
    g_stats.total_commands++;

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;

    ParamCache_t* cache = &g_param_cache[motor_id];

    /* 1. 切换到位置控制模式 */
    if (!cache->valid || cache->control_mode != MOTOR_MODE_POSITION) {
        motor_set_mode(can_channel, device_id, MOTOR_MODE_POSITION);
        cache->control_mode = MOTOR_MODE_POSITION;
        g_stats.total_frames++;
        frames_sent++;
    }

    /* 2. 设置力矩限制 */
    if (need_update(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT)) {
        motor_set_torque_limit(can_channel, device_id, FUSION_SWING_TORQUE_LIMIT);
        update_cache(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    /* 3. 设置目标位置到轨迹规划器
     * 优化核心：大历史缓冲(32帧) + 曲率计算 + 自适应参数调整
     * 这样50Hz的离散目标能被平滑处理
     */
    /* 3.1 先获取当前轨迹规划器状态 */
    TrajectoryPlanner_t planner;
    Trajectory_GetState(motor_id, &planner);

    /* 3.2 添加到大历史缓冲（用于曲线拟合和曲率计算）*/
    HistoryBuffer_Push(motor_id, target_pos);

    /* 3.3 添加到前瞻缓冲（用于短期趋势分析）*/
    Lookahead_PushHistory(motor_id, target_pos);
    
    /* 3.3.1 推送前瞻目标到轨迹规划器（用于预测未来轨迹点）*/
    Trajectory_PushLookahead(motor_id, target_pos);

    /* 3.3.2 设置主目标 - 确保轨迹规划器立即响应 */
    Trajectory_SetTarget(motor_id, target_pos);

    /* 3.4 根据曲率计算自适应参数 */
    float curv_v_max, curv_a_max, curv_d_max;
    HistoryBuffer_AdjustByCurvature(motor_id,
                                     planner.config.v_max,
                                     planner.config.a_max,
                                     &curv_v_max, &curv_a_max, &curv_d_max);

    /* 3.5 获取短期趋势作为补充调整 */
    TrendDirection_t trend = Lookahead_AnalyzeTrend(motor_id);

    /* 3.6 综合调整轨迹规划器的动态参数 */
    float final_v_max = curv_v_max;
    float final_a_max = curv_a_max;
    float final_d_max = curv_d_max;

    /* 短期趋势作为微调 */
    switch (trend) {
        case TREND_ACCELERATING:
            final_v_max *= 1.1f;
            final_a_max *= 1.1f;
            break;
        case TREND_DECELERATING:
            final_v_max *= 0.9f;
            final_a_max *= 0.9f;
            final_d_max *= 1.2f;
            break;
        case TREND_REVERSING:
            final_v_max *= 0.8f;
            final_a_max *= 0.8f;
            final_d_max *= 1.3f;
            break;
        case TREND_STABLE:
        case TREND_UNKNOWN:
        default:
            break;
    }

    /* 更新轨迹规划器的动态参数 */
    Trajectory_SetDynamicParams(motor_id, final_v_max, final_a_max, final_d_max);

    /* 3.7 设置目标位置到轨迹规划器 */
    float current_to_target = target_pos - planner.current_position;
    float current_direction = (current_to_target >= 0.0f) ? 1.0f : -1.0f;

    /* 获取当前速度作为初始速度（实现平滑过渡的关键！）*/
    float initial_velocity = planner.current_velocity;

    /* 检测方向切换：如果当前有速度，且新方向与速度方向相反 */
    float speed_direction = (planner.current_velocity >= 0.0f) ? 1.0f : -1.0f;
    bool direction_change = (fabsf(planner.current_velocity) > 5.0f) &&
                            (current_direction != speed_direction);

    /* 只有当目标变化较大时才更新 */
    float target_diff = fabsf(target_pos - planner.target_position);
    if (target_diff > TRAJECTORY_BUFFER_MIN_DIFF || direction_change) {

        /* 方向切换处理：需要提前减速 */
        if (direction_change) {
            float braking_dist = Trajectory_CalcBrakingDistance(
                fabsf(planner.current_velocity),
                planner.dynamic_d_max
            );

            float brake_target = planner.current_position +
                                (braking_dist * 0.5f) * speed_direction;

            if (brake_target > 0.0f && brake_target < 360.0f) {
                Trajectory_SetTargetWithVelocity(motor_id, brake_target, initial_velocity);
            } else {
                Trajectory_SetTargetWithVelocity(motor_id, target_pos, initial_velocity * 0.3f);
            }
        } else {
            /* 正常方向：保留当前速度继续运动 */
            Trajectory_SetTargetWithVelocity(motor_id, target_pos, initial_velocity);
        }
    } else {
        /* 目标变化很小，不重置轨迹规划器 */
    }

    cache->valid = true;
    g_stats.total_frames += frames_sent;

    return 0;
}

/**
 * @brief 发送摆动相融合指令（原始版本，不使用轨迹规划）
 * @note 用于需要直接控制位置的场合
 */
uint8_t Fusion_SendSwingCommandDirect(uint8_t motor_id, float target_pos)
{
    if (!g_fusion_initialized) {
        return 1;
    }

    if (motor_id >= QUAD_MAX_MOTORS) {
        return 1;
    }

    uint8_t frames_sent = 0;
    g_stats.total_commands++;

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;

    ParamCache_t* cache = &g_param_cache[motor_id];

    if (!cache->valid || cache->control_mode != MOTOR_MODE_POSITION) {
        motor_set_mode(can_channel, device_id, MOTOR_MODE_POSITION);
        cache->control_mode = MOTOR_MODE_POSITION;
        g_stats.total_frames++;
        frames_sent++;
    }

    if (need_update(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT)) {
        motor_set_torque_limit(can_channel, device_id, FUSION_SWING_TORQUE_LIMIT);
        update_cache(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    if (need_update(motor_id, "speed_limit", FUSION_SWING_SPEED_LIMIT)) {
        motor_set_limit_speed(can_channel, device_id, FUSION_SWING_SPEED_LIMIT);
        update_cache(motor_id, "speed_limit", FUSION_SWING_SPEED_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    if (need_update(motor_id, "accel_limit", FUSION_SWING_ACCEL_LIMIT)) {
        motor_set_limit_accel(can_channel, device_id, FUSION_SWING_ACCEL_LIMIT);
        update_cache(motor_id, "accel_limit", FUSION_SWING_ACCEL_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    motor_set_position(can_channel, device_id, target_pos);
    frames_sent++;

    cache->valid = true;
    g_stats.total_frames += frames_sent;

    return 0;
}

/**
 * @brief 批量发送摆动相指令（优化版）
 */
uint8_t Fusion_SendBatchSwingCommands(uint8_t* motor_ids,
                                       float* target_positions,
                                       uint8_t count)
{
    if (!g_fusion_initialized || motor_ids == NULL || target_positions == NULL) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (Fusion_SendSwingCommand(motor_ids[i], target_positions[i]) == 0) {
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 发送摆动相融合指令（带自适应PID）
 */
uint8_t Fusion_SendSwingCommandEx(uint8_t motor_id, float target_pos,
                                   float kp, float ki, float kd)
{
    if (!g_fusion_initialized) {
        return 1;
    }

    if (motor_id >= QUAD_MAX_MOTORS) {
        return 1;
    }

    uint8_t frames_sent = 0;
    g_stats.total_commands++;

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;

    ParamCache_t* cache = &g_param_cache[motor_id];

    /* 1. 切换到位置控制模式（每次都发送，确保模式正确）*/
    if (!cache->valid || cache->control_mode != MOTOR_MODE_POSITION) {
        motor_set_mode(can_channel, device_id, MOTOR_MODE_POSITION);
        cache->control_mode = MOTOR_MODE_POSITION;
        g_stats.total_frames++;
        frames_sent++;
        printf("[Fusion] Motor %u: Set POSITION mode (Ex)\r\n", motor_id);
    }

    /* PID参数已禁用，这些变量保留但不使用 */
    // float use_kp = (kp > 0) ? kp : FUSION_SWING_POS_KP;
    // float use_ki = (ki > 0) ? ki : FUSION_SWING_POS_KI;
    // float use_kd = (kd > 0) ? kd : FUSION_SWING_POS_KD;
    (void)kp;
    (void)ki;
    (void)kd;

    /* 2. 设置力矩限制（如果需要更新）*/
    if (need_update(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT)) {
        motor_set_torque_limit(can_channel, device_id, FUSION_SWING_TORQUE_LIMIT);
        update_cache(motor_id, "torque_limit", FUSION_SWING_TORQUE_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    /* 3. 设置速度限制（如果需要更新）*/
    if (need_update(motor_id, "speed_limit", FUSION_SWING_SPEED_LIMIT)) {
        motor_set_limit_speed(can_channel, device_id, FUSION_SWING_SPEED_LIMIT);
        update_cache(motor_id, "speed_limit", FUSION_SWING_SPEED_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    /* 4. 设置加速度限制（如果需要更新）*/
    if (need_update(motor_id, "accel_limit", FUSION_SWING_ACCEL_LIMIT)) {
        motor_set_limit_accel(can_channel, device_id, FUSION_SWING_ACCEL_LIMIT);
        update_cache(motor_id, "accel_limit", FUSION_SWING_ACCEL_LIMIT);
        frames_sent++;
    } else {
        g_stats.cache_hits++;
    }

    /* 5. 设置位置Kp（已禁用，使用电机默认PID）*/
    // if (need_update(motor_id, "pos_kp", use_kp)) {
    //     motor_set_pos_kp(can_channel, device_id, use_kp);
    //     update_cache(motor_id, "pos_kp", use_kp);
    //     frames_sent++;
    // } else {
    //     g_stats.cache_hits++;
    // }

    /* 6. 设置位置Ki（已禁用，使用电机默认PID）*/
    // if (use_ki > 0 && need_update(motor_id, "pos_ki", use_ki)) {
    //     motor_set_pos_ki(can_channel, device_id, use_ki);
    //     update_cache(motor_id, "pos_ki", use_ki);
    //     frames_sent++;
    // }

    /* 7. 设置位置Kd（已禁用，使用电机默认PID）*/
    // if (use_kd > 0 && need_update(motor_id, "pos_kd", use_kd)) {
    //     motor_set_pos_kd(can_channel, device_id, use_kd);
    //     update_cache(motor_id, "pos_kd", use_kd);
    //     frames_sent++;
    // }

    /* 8. 发送位置目标（总是发送）*/
    motor_set_position(can_channel, device_id, target_pos);
    frames_sent++;

    /* 标记缓存有效 */
    cache->valid = true;

    g_stats.total_frames += frames_sent;

    return 0;
}

/* ============================================================================
 * 公共函数实现 - 支撑相控制
 * ============================================================================ */

/**
 * @brief 发送支撑相融合指令（力位混合）
 * @note 添加参数缓存机制，避免重复发送相同参数
 */
uint8_t Fusion_SendStanceCommand(uint8_t motor_id,
                                  float target_force,
                                  float pos_limit)
{
    if (!g_fusion_initialized) {
        return 1;
    }

    if (motor_id >= QUAD_MAX_MOTORS) {
        return 1;
    }

    g_stats.total_commands++;

    /* 根据电机ID计算CAN通道和设备ID */
    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;

    ParamCache_t* cache = &g_param_cache[motor_id];

    /* 1. 切换到力矩控制模式（使用缓存）*/
    if (!cache->valid || cache->control_mode != MOTOR_MODE_TORQUE) {
        motor_set_mode(can_channel, device_id, MOTOR_MODE_TORQUE);
        cache->control_mode = MOTOR_MODE_TORQUE;
        g_stats.total_frames++;
        printf("[Fusion] Motor %u: Set TORQUE mode (force=%.1fNm)\r\n", 
               motor_id, target_force);
    } else {
        g_stats.cache_hits++;
    }

    /* 2. 设置目标力矩（每次都需要发送）*/
    motor_set_torque(can_channel, device_id, target_force);
    g_stats.total_frames++;

    /* 3. 设置力矩斜率（斜坡响应，使用缓存）*/
    if (!cache->valid || !float_equal(cache->torque_slope, FUSION_STANCE_TORQUE_SLOPE)) {
        motor_set_torque_slope(can_channel, device_id, FUSION_STANCE_TORQUE_SLOPE);
        cache->torque_slope = FUSION_STANCE_TORQUE_SLOPE;
        g_stats.total_frames++;
    } else {
        g_stats.cache_hits++;
    }

    /* 4. 设置位置限幅（使用缓存）*/
    if (!cache->valid || !float_equal(cache->pos_limit_plus, pos_limit)) {
        motor_set_pos_upper_limit(can_channel, device_id, pos_limit);
        cache->pos_limit_plus = pos_limit;
        g_stats.total_frames++;
    } else {
        g_stats.cache_hits++;
    }

    if (!cache->valid || !float_equal(cache->pos_limit_minus, -pos_limit)) {
        motor_set_pos_lower_limit(can_channel, device_id, -pos_limit);
        cache->pos_limit_minus = -pos_limit;
        g_stats.total_frames++;
    } else {
        g_stats.cache_hits++;
    }

    /* 标记缓存有效 */
    cache->valid = true;

    return 0;
}

/**
 * @brief 批量发送支撑相指令
 */
uint8_t Fusion_SendBatchStanceCommands(uint8_t* motor_ids,
                                        float* target_forces,
                                        float* pos_limits,
                                        uint8_t count)
{
    if (!g_fusion_initialized || motor_ids == NULL ||
        target_forces == NULL || pos_limits == NULL) {
        return 0;
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (Fusion_SendStanceCommand(motor_ids[i],
                                      target_forces[i],
                                      pos_limits[i]) == 0) {
            success_count++;
        }
    }

    return success_count;
}

/* ============================================================================
 * 公共函数实现 - 参数缓存管理
 * ============================================================================ */

/**
 * @brief 清除指定电机的参数缓存
 */
void Fusion_ClearCache(uint8_t motor_id)
{
    if (motor_id < QUAD_MAX_MOTORS) {
        memset(&g_param_cache[motor_id], 0, sizeof(ParamCache_t));
    }
}

/**
 * @brief 清除所有电机的参数缓存
 */
void Fusion_ClearAllCache(void)
{
    memset(g_param_cache, 0, sizeof(g_param_cache));
    printf("[Fusion] All parameter cache cleared\r\n");
}

/* ============================================================================
 * 公共函数实现 - 预设模板
 * ============================================================================ */

/**
 * @brief 获取摆动相预设参数模板
 */
void Fusion_GetSwingTemplate(FusedCommand_t* cmd, float target_pos)
{
    if (cmd == NULL) return;

    memset(cmd, 0, sizeof(FusedCommand_t));

    cmd->target_position = target_pos;
    cmd->speed_limit = FUSION_SWING_SPEED_LIMIT;
    cmd->accel_limit = FUSION_SWING_ACCEL_LIMIT;
    cmd->torque_limit = FUSION_SWING_TORQUE_LIMIT;
    cmd->pos_kp = FUSION_SWING_POS_KP;
    cmd->control_mode = MOTOR_MODE_POSITION;
    cmd->need_send = true;
    cmd->force_update = false;
}

/**
 * @brief 获取支撑相预设参数模板
 */
void Fusion_GetStanceTemplate(FusedCommand_t* cmd,
                               float target_force,
                               float pos_limit)
{
    if (cmd == NULL) return;

    memset(cmd, 0, sizeof(FusedCommand_t));

    cmd->torque_target = target_force;
    cmd->speed_limit = pos_limit;  /* 复用字段存储位置限幅 */
    cmd->control_mode = MOTOR_MODE_TORQUE;
    cmd->need_send = true;
    cmd->force_update = false;
}

/**
 * @brief 获取停止预设参数模板
 */
void Fusion_GetStopTemplate(FusedCommand_t* cmd, float current_pos)
{
    if (cmd == NULL) return;

    memset(cmd, 0, sizeof(FusedCommand_t));

    cmd->target_position = current_pos;
    cmd->speed_limit = 0.0f;
    cmd->torque_limit = 1.0f;  /* 小力矩保持位置 */
    cmd->control_mode = MOTOR_MODE_POSITION;
    cmd->need_send = true;
    cmd->force_update = false;
}

/* ============================================================================
 * 公共函数实现 - 统计和调试
 * ============================================================================ */

/**
 * @brief 获取发送统计信息
 */
void Fusion_GetStats(uint32_t* total_commands,
                     uint32_t* total_frames,
                     float* optimization_rate)
{
    if (total_commands) *total_commands = g_stats.total_commands;
    if (total_frames) *total_frames = g_stats.total_frames;

    if (optimization_rate) {
        if (g_stats.total_commands > 0) {
            /* 优化率 = (缓存命中数 / 总命令数) × 100% */
            *optimization_rate = (float)g_stats.cache_hits /
                               (float)(g_stats.total_commands * 4) * 100.0f;
        } else {
            *optimization_rate = 0.0f;
        }
    }
}

/**
 * @brief 重置统计信息
 */
void Fusion_ResetStats(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    printf("[Fusion] Statistics reset\r\n");
}

/**
 * @brief 打印融合层状态（调试用）
 */
void Fusion_PrintStatus(void)
{
    printf("\r\n========== Fusion Layer Status ==========\r\n");
    printf("Initialized:     %s\r\n", g_fusion_initialized ? "YES" : "NO");
    printf("Control Mode:     %s\r\n", 
           g_control_mode == CONTROL_MODE_PURE_POSITION ? "Pure Position" : "Hybrid");
    printf("Total Commands:  %u\r\n", g_stats.total_commands);
    printf("Total Frames:    %u\r\n", g_stats.total_frames);
    printf("Cache Hits:      %u\r\n", g_stats.cache_hits);

    if (g_stats.total_commands > 0) {
        float avg_frames = (float)g_stats.total_frames /
                          (float)g_stats.total_commands;
        float opt_rate = (float)g_stats.cache_hits /
                        (float)(g_stats.total_commands * 4) * 100.0f;
        printf("Avg Frames/Cmd:  %.2f\r\n", avg_frames);
        printf("Optimization:    %.1f%%\r\n", opt_rate);
    }

    printf("=========================================\r\n\r\n");
}

/**
 * @brief 设置控制模式（运行时切换）
 * @param mode: 0=纯位置控制, 1=力位混合控制
 * @return 0=成功, 1=失败
 */
uint8_t Fusion_SetControlMode(uint8_t mode)
{
    if (mode > CONTROL_MODE_HYBRID) {
        printf("[Fusion] Error: Invalid control mode %d\r\n", mode);
        return 1;
    }
    
    if (mode != g_control_mode) {
        printf("[Fusion] Control mode changed: %s -> %s\r\n",
               g_control_mode == CONTROL_MODE_PURE_POSITION ? "Pure Position" : "Hybrid",
               mode == CONTROL_MODE_PURE_POSITION ? "Pure Position" : "Hybrid");
        
        g_control_mode = mode;
        Fusion_ClearAllCache();
    }
    
    return 0;
}

/**
 * @brief 获取当前控制模式
 * @return 当前控制模式
 */
uint8_t Fusion_GetControlMode(void)
{
    return g_control_mode;
}

/* ============================================================================
 * 高级融合函数实现
 * ============================================================================ */

/**
 * @brief 发送通用融合指令（根据FusedCommand_t智能融合）
 * @param motor_id 电机索引（0-11）
 * @param cmd 融合指令结构体指针
 * @return 实际发送的CANFD帧数
 *
 * @note 根据FusedCommand_t中的参数自动选择要发送的指令
 *       利用参数缓存机制避免重复发送
 */
uint8_t Fusion_SendCommand(uint8_t motor_id, FusedCommand_t* cmd)
{
    if (motor_id >= QUAD_MAX_MOTORS || cmd == NULL) {
        return 0;
    }

    if (!cmd->need_send) {
        return 0;  /* 不需要发送 */
    }

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;
    uint8_t frames_sent = 0;

    g_stats.total_commands++;

    /* 根据control_mode发送对应指令 */
    switch (cmd->control_mode) {
        case 0:  /* 位置控制模式 */
            /* 发送限制参数（如果需要）*/
            if (cmd->force_update || need_update(motor_id, "torque_limit", cmd->torque_limit)) {
                motor_set_torque_limit(can_channel, device_id, cmd->torque_limit);
                update_cache(motor_id, "torque_limit", cmd->torque_limit);
                frames_sent++;
            } else {
                g_stats.cache_hits++;
            }

            if (cmd->force_update || need_update(motor_id, "speed_limit", cmd->speed_limit)) {
                motor_set_speed_limit(can_channel, device_id, cmd->speed_limit);
                update_cache(motor_id, "speed_limit", cmd->speed_limit);
                frames_sent++;
            } else {
                g_stats.cache_hits++;
            }

            if (cmd->force_update || need_update(motor_id, "accel_limit", cmd->accel_limit)) {
                motor_set_limit_accel(can_channel, device_id, cmd->accel_limit);
                update_cache(motor_id, "accel_limit", cmd->accel_limit);
                frames_sent++;
            } else {
                g_stats.cache_hits++;
            }

            /* PID已禁用，使用电机默认PID */
            // if (cmd->force_update || need_update(motor_id, "pos_kp", cmd->pos_kp)) {
            //     motor_set_pos_kp(can_channel, device_id, cmd->pos_kp);
            //     update_cache(motor_id, "pos_kp", cmd->pos_kp);
            //     frames_sent++;
            // } else {
            //     g_stats.cache_hits++;
            // }

            /* 发送位置目标（总是发送）*/
            motor_set_position(can_channel, device_id, cmd->target_position);
            frames_sent++;
            break;

        case 1:  /* 力矩控制模式 */
            /* 发送力矩目标 */
            motor_set_torque(can_channel, device_id, cmd->torque_target);
            frames_sent++;
            break;

        case 2:  /* 力位混合模式 */
            /* 先设置力矩限制 */
            if (cmd->force_update || need_update(motor_id, "torque_limit", cmd->torque_limit)) {
                motor_set_torque_limit(can_channel, device_id, cmd->torque_limit);
                update_cache(motor_id, "torque_limit", cmd->torque_limit);
                frames_sent++;
            } else {
                g_stats.cache_hits++;
            }

            /* 设置位置目标（作为位置限幅）*/
            motor_set_position(can_channel, device_id, cmd->target_position);
            frames_sent++;
            break;

        default:
            return 0;  /* 未知模式 */
    }

    g_stats.total_frames += frames_sent;
    return frames_sent;
}

/**
 * @brief 批量发送融合指令（优化版本）
 * @param motor_ids 电机索引数组
 * @param commands 融合指令数组
 * @param count 数量
 * @return 成功发送的电机数量
 *
 * @note 自动检测相同参数，进行批量优化
 */
uint8_t Fusion_SendBatchCommands(uint8_t* motor_ids,
                                  FusedCommand_t* commands,
                                  uint8_t count)
{
    if (motor_ids == NULL || commands == NULL || count == 0) {
        return 0;
    }

    uint8_t success_count = 0;

    if (count == 0 || motor_ids == NULL || commands == NULL) {
        return 0;
    }

    /* 批量优化：统计需要发送的相同参数 */
    float common_torque_limit = commands[0].torque_limit;
    float common_speed_limit = commands[0].speed_limit;
    float common_accel_limit = commands[0].accel_limit;
    bool all_torque_same = true;
    bool all_speed_same = true;
    bool all_accel_same = true;

    for (uint8_t i = 1; i < count; i++) {
        if (commands[i].torque_limit != common_torque_limit) all_torque_same = false;
        if (commands[i].speed_limit != common_speed_limit) all_speed_same = false;
        if (commands[i].accel_limit != common_accel_limit) all_accel_same = false;
    }

    /* 预处理：先发送公共参数（仅发送一次到多个电机）*/
    if (count >= 2) {
        uint8_t can1_motors[6], can2_motors[6];
        uint8_t can1_count = 0, can2_count = 0;

        for (uint8_t i = 0; i < count; i++) {
            uint8_t can_ch = (motor_ids[i] < 6) ? 1 : 2;
            if (can_ch == 1) can1_motors[can1_count++] = motor_ids[i] + 1;
            else can2_motors[can2_count++] = motor_ids[i] + 1;
        }

        if (all_torque_same && can1_count > 0) {
            for (uint8_t i = 0; i < can1_count; i++) {
                motor_set_torque_limit(1, can1_motors[i], common_torque_limit);
            }
        }
        if (all_torque_same && can2_count > 0) {
            for (uint8_t i = 0; i < can2_count; i++) {
                motor_set_torque_limit(2, can2_motors[i], common_torque_limit);
            }
        }

        if (all_speed_same && can1_count > 0) {
            for (uint8_t i = 0; i < can1_count; i++) {
                motor_set_speed_limit(1, can1_motors[i], common_speed_limit);
            }
        }
        if (all_speed_same && can2_count > 0) {
            for (uint8_t i = 0; i < can2_count; i++) {
                motor_set_speed_limit(2, can2_motors[i], common_speed_limit);
            }
        }

        if (all_accel_same && can1_count > 0) {
            for (uint8_t i = 0; i < can1_count; i++) {
                motor_set_limit_accel(1, can1_motors[i], common_accel_limit);
            }
        }
        if (all_accel_same && can2_count > 0) {
            for (uint8_t i = 0; i < can2_count; i++) {
                motor_set_limit_accel(2, can2_motors[i], common_accel_limit);
            }
        }

        for (uint8_t i = 0; i < count; i++) {
            update_cache(motor_ids[i], "torque_limit", commands[i].torque_limit);
            update_cache(motor_ids[i], "speed_limit", commands[i].speed_limit);
            update_cache(motor_ids[i], "accel_limit", commands[i].accel_limit);
        }
    }

    for (uint8_t i = 0; i < count; i++) {
        if (Fusion_SendCommand(motor_ids[i], &commands[i]) > 0) {
            success_count++;
        }
    }

    return success_count;
}

/**
 * @brief 强制更新（忽略缓存，强制发送所有参数）
 * @param motor_id 电机索引（0-11）
 * @param cmd 融合指令结构体指针
 * @return 实际发送的CANFD帧数
 *
 * @note 用于确保参数同步，例如：
 *       - 电机重启后
 *       - 参数丢失后
 *       - 需要重新初始化时
 */
uint8_t Fusion_ForceUpdate(uint8_t motor_id, FusedCommand_t* cmd)
{
    if (motor_id >= QUAD_MAX_MOTORS || cmd == NULL) {
        return 0;
    }

    /* 设置强制更新标志 */
    cmd->force_update = true;
    cmd->need_send = true;

    /* 调用通用融合指令 */
    uint8_t frames = Fusion_SendCommand(motor_id, cmd);

    /* 重置强制更新标志 */
    cmd->force_update = false;

    return frames;
}
