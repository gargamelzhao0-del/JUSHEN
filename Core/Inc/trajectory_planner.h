/**
 ****************************************************************************************************
 * @file        trajectory_planner.h
 * @version     V1.0
 * @brief       四足机器狗 - 轨迹规划器
 * @details     实现平滑的梯形/S曲线速度规划，确保电机平稳运动
 ****************************************************************************************************
 * @attention
 *
 * 功能说明：
 * 1. 梯形速度曲线规划（加速-匀速-减速）
 * 2. 支持动态目标更新（实时响应）
 * 3. 自动计算最优速度和加速度
 * 4. 防止到达目标时速度过快
 *
 * 解决问题：
 * - 电机到达目标位置时速度依然很快
 * - 反向运动时冲击大
 * - 运动不平稳
 *
 ****************************************************************************************************
 */

#ifndef __TRAJECTORY_PLANNER_H
#define __TRAJECTORY_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "system_config.h"

/* ============================================================================
 * 配置参数
 * ============================================================================ */

#ifndef TRAJECTORY_MAX_MOTORS
#define TRAJECTORY_MAX_MOTORS       12      /* 最大电机数量 */
#endif

/* 控制周期（200Hz控制循环）*/
#define TRAJECTORY_CONTROL_PERIOD   0.005f  /* 控制周期 (s) = 5ms */

/* 前瞻缓冲配置 */
#define TRAJECTORY_LOOKAHEAD_COUNT  5       /* 前瞻目标数量 - 增加到5 */
#define TRAJECTORY_BUFFER_MIN_DIFF  0.5f    /* 缓冲更新最小角度差 (deg) */

/* V4.0 预测性轨迹规划配置 */
#ifndef TRAJECTORY_HISTORY_SIZE
#define TRAJECTORY_HISTORY_SIZE     8       /* 历史位置缓冲区大小 */
#endif
#define TRAJECTORY_PREDICTION_HORIZON  0.030f /* 预测时间范围 (s) */
#define TRAJECTORY_SMOOTH_BLEND_TIME   0.015f /* 平滑过渡时间 (s) */
#define TRAJECTORY_VEL_SMOOTH_BLEND_TIME   0.015f /* 平滑过渡时间 (s) */
#define TRAJECTORY_VEL_SMOOTH_ALPHA    0.35f  /* 速度平滑因子 */

/* V6.0 连续轨迹跟踪模式配置 */
#define TRAJECTORY_CONTINUOUS_MIN_DIFF   0.1f   /* 连续模式最小更新差值 (deg) */
#define TRAJECTORY_CONTINUOUS_VEL_PRESERVE 0.9f /* 连续模式速度保持系数 */
#define TRAJECTORY_CONTINUOUS_MAX_VEL_JUMP 500.0f /* 最大速度跳变限制 (deg/s) */

/* V14.0: 动态增益参数 */
#define GAIN_BOOST_THRESHOLD       10.0f   /* 增益增强阈值 (deg) */
#define GAIN_BOOST_FACTOR          1.5f    /* 增益增强因子 */
#define GAIN_RAMP_FACTOR           0.6f    /* 增益斜坡因子 */

/* V14.0: 自适应前馈参数 - 同步copy版本 */
#define ADAPTIVE_FF_VEL_GAIN_BASE    0.85f  /* 基础速度前馈增益 */
#define ADAPTIVE_FF_ACCEL_GAIN_BASE  0.25f  /* 基础加速度前馈增益 */
#define ADAPTIVE_FF_ERROR_SCALE      0.01f  /* 误差对前馈的缩放因子 */
#define ADAPTIVE_FF_DISTANCE_TH      55.0f  /* 极值点距离阈值 (deg) */
#define ADAPTIVE_FF_REDUCTION        0.30f  /* 极值点前馈降低因子 */

/* ============================================================================
 * 类型定义
 * ============================================================================ */

/**
 * @brief 轨迹规划器状态枚举
 */
typedef enum {
    TRAJ_STATE_IDLE = 0,        /* 空闲状态 */
    TRAJ_STATE_ACCELERATING,    /* 加速阶段 */
    TRAJ_STATE_CRUISING,        /* 匀速阶段 */
    TRAJ_STATE_DECELERATING,    /* 减速阶段 */
    TRAJ_STATE_STOPPING,        /* 停止阶段 */
    TRAJ_STATE_COMPLETED,       /* 已完成 */
    TRAJ_STATE_ERROR            /* 错误状态 */
} TrajectoryState_t;

/**
 * @brief S曲线阶段枚举（七段式S曲线）
 */
typedef enum {
    S_CURVE_PHASE_JERK_POS = 0, /* 正加加速度阶段 */
    S_CURVE_PHASE_ACCEL,        /* 恒定加速度阶段 */
    S_CURVE_PHASE_JERK_NEG,     /* 负加加速度阶段（加速结束）*/
    S_CURVE_PHASE_CRUISE,       /* 匀速阶段 */
    S_CURVE_PHASE_JERK_NEG2,    /* 负加加速度阶段（减速开始）*/
    S_CURVE_PHASE_DECEL,        /* 恒定减速度阶段 */
    S_CURVE_PHASE_JERK_POS2     /* 正加加速度阶段（减速结束）*/
} SCurvePhase_t;

/**
 * @brief S曲线规划状态
 */
typedef struct {
    SCurvePhase_t phase;        /* 当前阶段 */
    float jerk;                 /* 当前加加速度 (deg/s³) */
    float t_jerk1;              /* 第一jerk阶段时间 (s) */
    float t_accel;              /* 恒定加速度时间 (s) */
    float t_jerk2;              /* 第二jerk阶段时间 (s) */
    float t_cruise;             /* 匀速时间 (s) */
    float t_jerk3;              /* 第三jerk阶段时间 (s) */
    float t_decel;              /* 恒定减速度时间 (s) */
    float t_jerk4;              /* 第四jerk阶段时间 (s) */
    float phase_time;           /* 当前阶段已用时间 (s) */
    float peak_velocity;        /* 峰值速度 (deg/s) */
    bool use_s_curve;           /* 是否使用S曲线 */
} SCurveState_t;

/**
 * @brief 轨迹规划器配置
 */
typedef struct {
    float v_max;                /* 最大速度 (deg/s) */
    float a_max;                /* 最大加速度 (deg/s²) */
    float d_max;                /* 最大减速度 (deg/s²) */
    float jerk_limit;           /* 加加速度限制 (deg/s³) */
    float smooth_factor;        /* 平滑因子 */
    float pos_threshold;        /* 位置到达阈值 */
    float vel_threshold;        /* 速度到达阈值 */
} TrajectoryConfig_t;

/**
 * @brief 轨迹规划器状态
 */
typedef struct {
    /* 目标状态 */
    float target_position;      /* 当前目标位置 (deg) */
    float target_velocity;      /* 目标速度 (deg/s) - 通常为0 */

    /* 当前状态 */
    float current_position;     /* 当前位置 (deg) */
    float current_velocity;      /* 当前速度 (deg/s) */
    float current_acceleration;  /* 当前加速度 (deg/s²) */
    float filtered_acceleration; /* V5.5: 滤波后的加速度 (消除状态切换阶跃) */

    /* 规划状态 */
    TrajectoryState_t state;    /* 当前状态 */
    uint32_t tick_count;        /* 滴答计数 */
    float remaining_distance;   /* 剩余距离 (deg) */
    float peak_velocity;        /* 峰值速度 (deg/s) */

    /* 标志位 */
    bool target_updated;        /* 目标是否更新 */
    bool is_moving;             /* 是否正在运动 */
    bool direction;             /* 运动方向 (true=正向) */

    /* 前瞻缓冲 - 用于预读未来目标 */
    float lookahead_positions[TRAJECTORY_LOOKAHEAD_COUNT];  /* 前瞻目标位置 */
    uint8_t lookahead_count;    /* 已缓冲的目标数量 */
    uint8_t lookahead_index;    /* 当前处理到的前瞻索引 */
    bool lookahead_direction_change;  /* 检测到方向改变 */

    /* 速度/加速度动态调整 */
    float dynamic_v_max;        /* 动态最大速度 (deg/s) */
    float dynamic_a_max;        /* 动态最大加速度 (deg/s²) */
    float dynamic_d_max;        /* 动态最大减速度 (deg/s²) */

    /* S曲线规划状态 */
    SCurveState_t s_curve;      /* S曲线状态 */

    /* V4.0 预测性轨迹规划 */
    float history_positions[TRAJECTORY_HISTORY_SIZE];  /* 历史位置缓冲 */
    float history_velocities[TRAJECTORY_HISTORY_SIZE]; /* 历史速度缓冲 */
    uint32_t history_timestamps[TRAJECTORY_HISTORY_SIZE]; /* 时间戳 */
    uint8_t history_index;      /* 写入索引 */
    uint8_t history_count;      /* 有效数据数量 */
    float predicted_position;   /* 预测的下一目标位置 */
    float predicted_velocity;   /* 预测的下一目标速度 */
    float smooth_target;        /* 平滑后的目标位置 */
    float smooth_velocity;      /* 平滑后的目标速度 */
    bool prediction_valid;      /* 预测是否有效 */
    
    /* V4.0 轨迹连续性保障 */
    float last_planned_position;  /* 上次规划输出位置 */
    float last_planned_velocity;  /* 上次规划输出速度 */
    float last_planned_accel;     /* 上次规划输出加速度 */
    float blend_factor;           /* 平滑混合因子 */
    
    /* 配置 */
    TrajectoryConfig_t config;  /* 配置参数 */
} TrajectoryPlanner_t;

/**
 * @brief 轨迹规划器输出
 */
typedef struct {
    float position;             /* 规划位置 (deg) */
    float velocity;             /* 规划速度 (deg/s) - V13.0: 始终为正（绝对值） */
    float acceleration;         /* 规划加速度 (deg/s²) - V13.0: 始终为正（绝对值） */
    bool is_completed;          /* 是否完成 */
    TrajectoryState_t state;    /* 当前状态 */
    bool direction;             /* V13.0: 运动方向 (true=正向增加, false=负向减少) */
    
    /* 前馈控制输出 */
    float feedforward_vel;      /* 速度前馈 (deg/s) - V13.0: 始终为正 */
    float feedforward_accel;    /* 加速度前馈 (deg/s²) - V13.0: 始终为正 */
} TrajectoryOutput_t;

/**
 * @brief 前馈控制参数
 */
typedef struct {
    float vel_ff_gain;          /* 速度前馈增益 */
    float accel_ff_gain;        /* 加速度前馈增益 */
    float pos_kp;               /* 位置比例增益 */
    float vel_kp;               /* 速度比例增益 */
    float vel_ki;               /* 速度积分增益 */
    float vel_kd;               /* 速度微分增益 */
    float integral_limit;       /* 积分限幅 */
} FeedforwardConfig_t;

/**
 * @brief 反向间隙补偿配置 - V8.0 添加极值点因子
 */
typedef struct {
    float backlash_compensation;  /* 反向间隙补偿量 (deg) */
    float last_direction;         /* 上次运动方向 */
    float extreme_point_factor;   /* V8.0: 极值点补偿折扣 (0-1) */
} BacklashConfig_t;

/**
 * @brief 摩擦力补偿配置
 */
typedef struct {
    float coulomb_friction;       /* 库伦摩擦力矩 (Nm) */
    float viscous_friction;       /* 粘性摩擦系数 (Nm/(rad/s)) */
} FrictionConfig_t;

/**
 * @brief V12.0: 极值点检测信息 - 优化方案B1
 */
typedef struct {
    bool is_extreme_point;        /* 是否为极值点 */
    bool is_peak;                 /* true=峰值, false=谷值 */
    float extreme_distance;       /* 距离极值点的距离 (deg) */
    float velocity_reduction;     /* 速度折扣因子 (0-1) */
    float accel_reduction;        /* 加速度折扣因子 (0-1) */
    uint32_t detection_tick;      /* 检测时间戳 */
} ExtremePointInfo_t;

/* ============================================================================
 * 初始化函数
 * ============================================================================ */

/**
 * @brief 初始化所有轨迹规划器
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_Init(void);

/**
 * @brief 反初始化轨迹规划器
 */
void Trajectory_Deinit(void);

/**
 * @brief 初始化单个规划器
 * @param motor_id 电机索引 (0-11)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_InitSingle(uint8_t motor_id);

/* ============================================================================
 * 配置函数
 * ============================================================================ */

/**
 * @brief 设置规划器配置
 * @param motor_id 电机索引 (0-11)
 * @param config 配置参数指针
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_SetConfig(uint8_t motor_id, const TrajectoryConfig_t* config);

/**
 * @brief 获取规划器配置
 * @param motor_id 电机索引 (0-11)
 * @param config 配置参数输出指针
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_GetConfig(uint8_t motor_id, TrajectoryConfig_t* config);

/**
 * @brief 设置运动参数（简化接口）
 * @param motor_id 电机索引 (0-11)
 * @param v_max 最大速度 (deg/s)
 * @param a_max 最大加速度 (deg/s²)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_SetMotionParams(uint8_t motor_id, float v_max, float a_max);

/**
 * @brief 设置动态参数（用于实时调整速度/加速度限制）
 * @param motor_id 电机索引 (0-11)
 * @param v_max 动态最大速度 (deg/s)
 * @param a_max 动态最大加速度 (deg/s²)
 * @param d_max 动态最大减速度 (deg/s²)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_SetDynamicParams(uint8_t motor_id, float v_max, float a_max, float d_max);

/* ============================================================================
 * 控制函数
 * ============================================================================ */

/**
 * @brief 设置目标位置
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置 (deg)
 * @return 0=成功, 非0=失败
 * @note 内部会自动计算最优轨迹
 */
uint8_t Trajectory_SetTarget(uint8_t motor_id, float target_position);

/**
 * @brief 设置目标位置（带初始速度）
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置 (deg)
 * @param initial_velocity 初始速度 (deg/s)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_SetTargetWithVelocity(uint8_t motor_id, float target_position, float initial_velocity);

/**
 * @brief 更新当前位置（反馈更新）
 * @param motor_id 电机索引 (0-11)
 * @param current_position 当前位置 (deg)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_UpdateCurrentPosition(uint8_t motor_id, float current_position);

/**
 * @brief V14.0: 设置规划器当前位置（用于闭环修正）
 * @param motor_id 电机索引 (0-11)
 * @param position 当前位置 (deg)
 */
void Trajectory_SetCurrentPosition(uint8_t motor_id, float position);

/**
 * @brief 立即停止（平滑减速）
 * @param motor_id 电机索引 (0-11)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_Stop(uint8_t motor_id);

/**
 * @brief 紧急停止（立即停止）
 * @param motor_id 电机索引 (0-11)
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_EmergencyStop(uint8_t motor_id);

/* ============================================================================
 * 执行函数
 * ============================================================================ */

/**
 * @brief 执行一步轨迹规划（每个控制周期调用）
 * @param motor_id 电机索引 (0-11)
 * @param output 输出结构体指针
 * @return 0=成功, 非0=失败
 * @note 在控制循环中以固定频率调用（如500Hz）
 */
uint8_t Trajectory_Step(uint8_t motor_id, TrajectoryOutput_t* output);

/**
 * @brief 批量执行所有规划器
 * @param outputs 输出数组指针（12个元素）
 * @return 成功执行的规划器数量
 */
uint8_t Trajectory_StepAll(TrajectoryOutput_t outputs[TRAJECTORY_MAX_MOTORS]);

/* ============================================================================
 * 状态查询函数
 * ============================================================================ */

/**
 * @brief 获取规划器状态
 * @param motor_id 电机索引 (0-11)
 * @param planner 状态结构体输出指针
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_GetState(uint8_t motor_id, TrajectoryPlanner_t* planner);

/**
 * @brief 检查是否到达目标
 * @param motor_id 电机索引 (0-11)
 * @return true=已到达, false=未到达
 */
bool Trajectory_IsCompleted(uint8_t motor_id);

/**
 * @brief 获取剩余距离
 * @param motor_id 电机索引 (0-11)
 * @return 剩余距离 (deg)
 */
float Trajectory_GetRemainingDistance(uint8_t motor_id);

/**
 * @brief 获取预计到达时间
 * @param motor_id 电机索引 (0-11)
 * @return 预计到达时间 (s)，-1表示错误
 */
float Trajectory_GetEstimatedTime(uint8_t motor_id);

/* ============================================================================
 * 高级功能
 * ============================================================================ */

/**
 * @brief V2.0: 极值点检测参数配置结构体
 */
typedef struct {
    float extreme_distance_near;    /* 极值点近距离阈值 (deg), 默认2.0 */
    float extreme_distance_mid;     /* 极值点中距离阈值 (deg), 默认6.0 */
    float extreme_distance_far;     /* 极值点远距离阈值 (deg), 默认12.0 */
    float vel_reduction_near;       /* 近距离速度折扣因子, 默认0.4 */
    float vel_reduction_mid;        /* 中距离速度折扣因子, 默认0.8 */
    float accel_reduction_near;     /* 近距离加速度折扣因子, 默认0.3 */
    float accel_reduction_mid;      /* 中距离加速度折扣因子, 默认0.8 */
    float speed_ratio_threshold;    /* 速度比率阈值, 默认0.8 */
} ExtremePointConfig_t;

/**
 * @brief V2.0: 设置极值点检测参数
 * @param motor_id 电机索引 (0-11), 255表示全局设置
 * @param config 参数配置指针
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_SetExtremePointConfig(uint8_t motor_id, const ExtremePointConfig_t* config);

/**
 * @brief V2.0: 获取极值点检测参数
 * @param motor_id 电机索引 (0-11)
 * @param config 参数配置输出指针
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_GetExtremePointConfig(uint8_t motor_id, ExtremePointConfig_t* config);

/**
 * @brief 预计算轨迹（用于调试）
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置 (deg)
 * @param peak_velocity 输出峰值速度
 * @param total_time 输出总时间
 * @return 0=成功, 非0=失败
 */
uint8_t Trajectory_Precalculate(uint8_t motor_id, float target_position,
                                 float* peak_velocity, float* total_time);

/**
 * @brief 打印规划器状态（调试用）
 * @param motor_id 电机索引 (0-11)
 */
void Trajectory_PrintStatus(uint8_t motor_id);

/**
 * @brief 打印所有规划器状态
 */
void Trajectory_PrintAllStatus(void);

/* ============================================================================
 * 前瞻缓冲功能（用于预读未来目标）
 * ============================================================================ */

/**
 * @brief 添加前瞻目标到缓冲队列
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置 (deg)
 * @return 0=成功, 1=缓冲已满, 2=参数错误
 * @note 当检测到方向改变时，会自动调整速度/加速度实现提前减速
 */
uint8_t Trajectory_PushLookahead(uint8_t motor_id, float target_position);

/**
 * @brief 获取前瞻目标（不弹出）
 * @param motor_id 电机索引 (0-11)
 * @param index 前瞻索引 (0 = 下一个目标, 1 = 下下个目标)
 * @param target_position 输出目标位置
 * @return 0=成功, 1=索引无效
 */
uint8_t Trajectory_GetLookahead(uint8_t motor_id, uint8_t index, float* target_position);

/**
 * @brief 获取前瞻缓冲状态
 * @param motor_id 电机索引 (0-11)
 * @return 缓冲的目标数量
 */
uint8_t Trajectory_GetLookaheadCount(uint8_t motor_id);

/**
 * @brief 设置连续轨迹目标（V6.0 新增）
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置 (deg)
 * @return 0=成功, 非0=失败
 * @note 用于连续轨迹跟踪模式，支持平滑速度保持
 */
uint8_t Trajectory_SetContinuousTarget(uint8_t motor_id, float target_position);

/**
 * @brief 启用/禁用连续轨迹模式
 * @param enable true=启用, false=禁用
 */
void Trajectory_EnableContinuousMode(bool enable);

/**
 * @brief 检查是否处于连续轨迹模式
 * @return true=连续模式启用, false=普通模式
 */
bool Trajectory_IsContinuousMode(void);

/**
 * @brief 检查前瞻缓冲是否检测到方向改变
 * @param motor_id 电机索引 (0-11)
 * @return true=检测到方向改变，需要提前减速
 */
bool Trajectory_CheckDirectionChange(uint8_t motor_id);

/**
 * @brief 清除前瞻缓冲
 * @param motor_id 电机索引 (0-11)
 */
void Trajectory_ClearLookahead(uint8_t motor_id);

/**
 * @brief 清除所有电机的前瞻缓冲
 */
void Trajectory_ClearAllLookahead(void);

/**
 * @brief 获取前瞻缓冲状态
 * @param motor_id 电机索引 (0-11)
 * @return 缓冲的目标数量
 */
uint8_t Trajectory_GetLookaheadCount(uint8_t motor_id);

/* ============================================================================
 * 高级补偿功能
 * ============================================================================ */

/**
 * @brief 设置反向间隙补偿参数
 * @param motor_id 电机 ID
 * @param backlash 反向间隙补偿量 (deg)
 */
void Trajectory_SetBacklashCompensation(uint8_t motor_id, float backlash);

/**
 * @brief 设置摩擦力补偿参数
 * @param motor_id 电机 ID
 * @param coulomb 库伦摩擦力矩 (Nm)
 * @param viscous 粘性摩擦系数 (Nm/(rad/s))
 */
void Trajectory_SetFrictionCompensation(uint8_t motor_id, float coulomb, float viscous);

/**
 * @brief 应用反向间隙补偿
 * @param motor_id 电机 ID
 * @param target_position 目标位置
 * @param current_position 当前位置
 * @return 补偿后的目标位置
 */
float Trajectory_ApplyBacklashCompensation(uint8_t motor_id, float target_position, float current_position);

/**
 * @brief 计算摩擦力补偿力矩
 * @param motor_id 电机 ID
 * @param velocity 当前速度 (deg/s)
 * @param torque_command 原始力矩指令 (Nm)
 * @return 补偿后的力矩指令 (Nm)
 */
float Trajectory_ApplyFrictionCompensation(uint8_t motor_id, float velocity, float torque_command);

/**
 * @brief 获取补偿配置信息
 * @param motor_id 电机 ID
 * @param backlash 输出反向间隙补偿量
 * @param coulomb 出库伦摩擦力矩
 * @param viscous 输出粘性摩擦系数
 */
void Trajectory_GetCompensationConfig(uint8_t motor_id, float* backlash, float* coulomb, float* viscous);

/**
 * @brief 启用/禁用反向间隙补偿（V8.0新增）
 * @param enable true=启用, false=禁用
 */
void Trajectory_EnableBacklashCompensation(bool enable);

/* ============================================================================
 * 辅助函数
 * ============================================================================ */

/**
 * @brief 计算减速距离
 * @param velocity 当前速度 (deg/s)
 * @param deceleration 减速度 (deg/s²)
 * @return 减速距离 (deg)
 */
static inline float Trajectory_CalcBrakingDistance(float velocity, float deceleration)
{
    if (deceleration <= 0.0f) return 0.0f;
    return (velocity * velocity) / (2.0f * deceleration);
}

/**
 * @brief 计算加速到目标速度所需距离
 * @param v0 初始速度 (deg/s)
 * @param v1 目标速度 (deg/s)
 * @param acceleration 加速度 (deg/s²)
 * @return 所需距离 (deg)
 */
static inline float Trajectory_CalcAccelDistance(float v0, float v1, float acceleration)
{
    if (acceleration <= 0.0f) return 0.0f;
    return (v1 * v1 - v0 * v0) / (2.0f * acceleration);
}

/**
 * @brief 计算最优峰值速度
 * @param distance 总距离 (deg)
 * @param v_max 最大速度限制 (deg/s)
 * @param a_max 最大加速度 (deg/s²)
 * @return 最优峰值速度 (deg/s)
 */
static inline float Trajectory_CalcOptimalVelocity(float distance, float v_max, float a_max)
{
    if (distance <= 0.0f || a_max <= 0.0f) return 0.0f;

    /* 三角形速度曲线的峰值速度 */
    float v_triangle = sqrtf(fabsf(distance) * a_max);

    /* 返回较小值（三角形或梯形）*/
    return (v_triangle < v_max) ? v_triangle : v_max;
}

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_PLANNER_H */
