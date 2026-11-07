/**
 ****************************************************************************************************
 * @file        quad_types.h
 * @version     V1.0
 * @brief       四足机器狗电机控制系统 - 核心类型定义
 * @details     定义所有模块共用的枚举、结构体和常量
 ****************************************************************************************************
 * @attention
 *
 * 版权所有 (c) 2026
 * 保留所有权利
 *
 ****************************************************************************************************
 */

#ifndef __QUAD_TYPES_H
#define __QUAD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_control.h"

/* ============================================================================
 * 常量定义
 * ============================================================================ */

#define QUAD_MAX_MOTORS         12      /* 最大电机数量 */
#define QUAD_NUM_LEGS           4       /* 腿部数量 */
#define QUAD_MOTORS_PER_LEG     3       /* 每条腿的电机数量 */

/* 电机角度安全限位（度），中位为180°，范围0°~360° */
#define QUAD_MOTOR_ANGLE_MAX    360.0f  /* 最大角度 */
#define QUAD_MOTOR_ANGLE_MIN    0.0f    /* 最小角度 */

/* 推荐的工作范围（根据具体机械设计调整）*/
#define QUAD_MOTOR_ANGLE_LIMIT_DEFAULT  90.0f  /* 默认单侧限位 */

/* ============================================================================
 * 腿部和关节枚举
 * ============================================================================ */

/**
 * @brief 腿部ID枚举
 */
typedef enum {
    LEG_FRONT_LEFT  = 0,    /* 前左腿 */
    LEG_FRONT_RIGHT = 1,    /* 前右腿 */
    LEG_REAR_LEFT   = 2,    /* 后左腿 */
    LEG_REAR_RIGHT  = 3,    /* 后右腿 */
} QuadLegID_t;

/**
 * @brief 关节ID枚举（每条腿3个关节）
 */
typedef enum {
    JOINT_HIP   = 0,        /* 髋关节（Abduction/Adduction）*/
    JOINT_THIGH = 1,        /* 大腿关节（Hip Flexion/Extension）*/
    JOINT_CALF  = 2,        /* 小腿关节（Knee Flexion/Extension）*/
} QuadJointID_t;

/**
 * @brief 足端ID枚举（与腿部ID对应）
 */
typedef enum {
    FOOT_FRONT_LEFT  = 0,   /* 前左足端 */
    FOOT_FRONT_RIGHT = 1,   /* 前右足端 */
    FOOT_REAR_LEFT   = 2,   /* 后左足端 */
    FOOT_REAR_RIGHT  = 3,   /* 后右足端 */
} QuadFootID_t;

/* ============================================================================
 * 运动场景和控制策略枚举
 * ============================================================================ */

/**
 * @brief 运动场景类型
 */
typedef enum {
    SCENE_SWING      = 0,   /* 摆动相（腿在空中）*/
    SCENE_STANCE     = 1,   /* 支撑相（足端接触地面）*/
    SCENE_TRANSITION = 2,   /* 过渡阶段（接触/离地瞬间）*/
    SCENE_STOP       = 3,   /* 停止状态 */
    SCENE_UNKNOWN    = 4,   /* 未知状态 */
} QuadScene_t;

/**
 * @brief 控制策略类型
 */
typedef enum {
    STRATEGY_POSITION  = 0, /* 纯位置控制 */
    STRATEGY_HYBRID    = 1, /* 力位混合控制 */
    STRATEGY_IMPEDANCE = 2, /* 阻抗控制 */
    STRATEGY_TORQUE    = 3, /* 纯力矩控制 */
} QuadStrategy_t;

/**
 * @brief 力位混合控制状态（用于SmartController）
 */
typedef enum {
    HYBRID_STATE_SWING     = 0,    /* 摆动相 - 位置控制 */
    HYBRID_STATE_STANCE    = 1,    /* 支撑相 - 力矩控制 */
    HYBRID_STATE_TRANSITION = 2,   /* 过渡阶段 */
    HYBRID_STATE_FORCE      = 3,   /* 力控制阶段 */
    HYBRID_STATE_LIFT       = 4,   /* 抬腿阶段 */
    HYBRID_STATE_TOUCHDOWN  = 5,   /* 触地阶段 */
    HYBRID_STATE_SUPPORT    = 6,   /* 支撑阶段 */
    HYBRID_STATE_IDLE       = 7,   /* 空闲 */
} QuadHybridState_t;

/**
 * @brief 腿部控制模式（用户API）
 */
typedef enum {
    LEG_MODE_POSITION   = 0,    /* 位置控制模式 */
    LEG_MODE_HYBRID     = 1,    /* 力位混合模式（自动切换）*/
    LEG_MODE_TORQUE     = 2,    /* 力矩控制模式 */
    LEG_MODE_DISABLED   = 3,    /* 禁用 */
} QuadLegMode_t;

/* ============================================================================
 * 电机状态结构体
 * ============================================================================ */

/**
 * @brief 单个电机实时状态
 */
typedef struct {
    /* 基础状态 */
    uint8_t motor_id;           /* 电机ID (0-11) */
    bool enabled;               /* 使能状态 */

    /* 位置和速度 */
    float position;             /* 当前位置（度）*/
    float velocity;             /* 当前速度（deg/s）*/
    float target_position;      /* 目标位置（度）*/

    /* 力矩 */
    float torque;               /* 当前力矩（Nm）*/
    float target_torque;        /* 目标力矩（Nm）*/

    /* 温度和电压 */
    float temperature;          /* 电机温度（℃）*/
    float voltage;              /* 母线电压（V）*/
    float current;              /* 电流（A）*/

    /* 控制状态 */
    QuadScene_t scene;          /* 当前场景 */
    QuadStrategy_t strategy;    /* 控制策略 */
    QuadHybridState_t hybrid_state; /* 力位混合状态 */

    /* 错误状态 */
    uint16_t error_flags;       /* 错误标志 */

    /* 时间戳 */
    uint32_t last_update_ms;    /* 最后更新时间（ms）*/
} QuadMotorStatus_t;

/* ============================================================================
 * 腿部状态
 * ============================================================================ */

/**
 * @brief 腿部状态（3个关节）
 */
typedef struct {
    QuadLegID_t leg_id;         /* 腿部ID */
    QuadLegMode_t mode;         /* 控制模式 */

    /* 关节状态 */
    QuadMotorStatus_t joints[QUAD_MOTORS_PER_LEG];  /* 3个关节状态 */

    /* 足端力传感器 */
    float foot_force;           /* 足端力（N）*/
    bool is_contact;            /* 是否接触地面 */

    /* 腿部整体状态 */
    QuadScene_t scene;          /* 当前场景（摆动/支撑）*/

} QuadLegStatus_t;

/* ============================================================================
 * 控制参数结构体
 * ============================================================================ */

/**
 * @brief 摆动相控制参数
 */
typedef struct {
    float torque_limit;         /* 力矩限制（Nm）*/
    float speed_limit;          /* 速度限制（rpm）*/
    float accel_limit;          /* 加速度限制（rpm/s）*/
    float pos_kp;               /* 位置环Kp */
} SwingParams_t;

/**
 * @brief 支撑相控制参数（力位混合）
 */
typedef struct {
    float force_threshold;      /* 接触力阈值（N）*/
    float torque_target;        /* 目标支撑力矩（Nm）*/
    float pos_limit;            /* 位置限幅（度）*/
    float transition_time;      /* 模式切换过渡时间（s）*/
} StanceParams_t;

/**
 * @brief 场景识别参数
 */
typedef struct {
    float force_contact_threshold;  /* 接触检测力阈值（N）*/
    float force_filter_window;      /* 力传感器滤波窗口大小 */
    float angle_change_threshold;   /* 角度变化阈值（度）*/
    uint16_t debounce_count;        /* 防抖动计数 */
} SceneRecognitionParams_t;

/**
 * @brief 融合指令集
 */
typedef struct {
    /* 位置控制参数 */
    float target_position;      /* 目标位置（度）*/

    /* 速度和加速度限制 */
    float speed_limit;          /* 速度限制（rpm）*/
    float accel_limit;          /* 加速度限制（rpm/s）*/

    /* 力矩参数 */
    float torque_limit;         /* 力矩限制（Nm）*/
    float torque_target;        /* 目标力矩（Nm）*/

    /* PID参数 */
    float pos_kp;               /* 位置环Kp */

    /* 控制模式 */
    uint8_t control_mode;       /* 控制模式（位置/力矩/混合）*/

    /* 发送标志 */
    bool need_send;             /* 是否需要发送 */
    bool force_update;          /* 强制更新（忽略缓存）*/

} FusedCommand_t;

/* ============================================================================
 * 辅助宏定义
 * ============================================================================ */

/**
 * @brief 根据腿部ID和关节ID计算电机索引
 * @param leg_id 腿部ID (0-3)
 * @param joint_id 关节ID (0-2)
 * @return 电机索引 (0-11)
 */
#define QUAD_MOTOR_INDEX(leg_id, joint_id) \
    ((leg_id) * QUAD_MOTORS_PER_LEG + (joint_id))

/**
 * @brief 根据电机索引获取腿部ID
 * @param motor_id 电机索引 (0-11)
 * @return 腿部ID (0-3)
 */
#define QUAD_GET_LEG_ID(motor_id) \
    ((motor_id) / QUAD_MOTORS_PER_LEG)

/**
 * @brief 根据电机索引获取关节ID
 * @param motor_id 电机索引 (0-11)
 * @return 关节ID (0-2)
 */
#define QUAD_GET_JOINT_ID(motor_id) \
    ((motor_id) % QUAD_MOTORS_PER_LEG)

/* ============================================================================
 * 函数声明（类型转换和辅助函数）
 * ============================================================================ */

/**
 * @brief 获取场景名称字符串
 * @param scene 场景类型
 * @return 场景名称字符串
 */
const char* quad_scene_name(QuadScene_t scene);

/**
 * @brief 获取策略名称字符串
 * @param strategy 策略类型
 * @return 策略名称字符串
 */
const char* quad_strategy_name(QuadStrategy_t strategy);

/**
 * @brief 获取腿部名称字符串
 * @param leg_id 腿部ID
 * @return 腿部名称字符串
 */
const char* quad_leg_name(QuadLegID_t leg_id);

/**
 * @brief 获取关节名称字符串
 * @param joint_id 关节ID
 * @return 关节名称字符串
 */
const char* quad_joint_name(QuadJointID_t joint_id);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_TYPES_H */
