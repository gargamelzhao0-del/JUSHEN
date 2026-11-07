/**
 ****************************************************************************************************
 * @file        quad_command_fusion.h
 * @version     V1.0
 * @brief       四足机器狗 - 指令融合层
 * @details     将复杂的底层CANFD指令融合为高层控制指令，优化总线负载
 ****************************************************************************************************
 * @attention
 *
 * 功能说明：
 * 1. 预定义摆动相/支撑相参数集
 * 2. 融合多条底层指令为一次发送
 * 3. 参数缓存机制（避免重复发送）
 * 4. 批量优化（减少CANFD帧数）
 *
 ****************************************************************************************************
 */

#ifndef __QUAD_COMMAND_FUSION_H
#define __QUAD_COMMAND_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "quad_types.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 控制模式选择（编译时选择）
 * ============================================================================ */
#define CONTROL_MODE_PURE_POSITION   0   /* 纯位置控制模式 */
#define CONTROL_MODE_HYBRID         1   /* 力位混合控制模式 */

/**
 * @brief 选择控制模式
 * @note 通过修改此宏选择控制模式：0=纯位置控制，1=力位混合控制
 *       上位机命令优先级更高，可通过TCP动态切换
 */
#ifndef QUAD_CONTROL_MODE
#define QUAD_CONTROL_MODE   CONTROL_MODE_PURE_POSITION
#endif

/* ============================================================================
 * 测试模式配置（用于无电机/无力传感器测试）
 * ============================================================================ */

/**
 * @brief 测试模式开关
 * @note 设置为1时，禁用电机反馈超时检测，用于CANFD分析仪测试
 *       P0-6修复: 正常运行时必须设为0，启用电机反馈超时保护
 */
#ifndef TEST_MODE_ENABLED
#define TEST_MODE_ENABLED           0       /* 正常运行模式 */
#endif

/**
 * @brief 无电机测试模式
 * @note 设置为1时，禁用电机反馈超时和错误检测
 */
#ifndef TEST_MODE_NO_MOTOR
#define TEST_MODE_NO_MOTOR          1       /* 无电机测试 */
#endif

/**
 * @brief 无足端力传感器测试模式
 * @note 设置为1时，禁用足端力依赖的功能
 */
#ifndef TEST_MODE_NO_FORCE_SENSOR
#define TEST_MODE_NO_FORCE_SENSOR   1       /* 无力传感器测试 */
#endif

/* 电机反馈超时（ms）- 测试模式下设为0禁用 */
#if TEST_MODE_ENABLED && TEST_MODE_NO_MOTOR
#define FUSION_FEEDBACK_TIMEOUT_MS      0   /* 测试模式：禁用超时 */
#define MOTOR_FEEDBACK_TIMEOUT_MS       0   /* 测试模式：禁用超时 */
#else
#define FUSION_FEEDBACK_TIMEOUT_MS      100 /* 正常运行：100ms超时 */
#define MOTOR_FEEDBACK_TIMEOUT_MS       100 /* 正常运行：100ms超时 */
#endif

/* ============================================================================
 * 融合参数集定义（预定义模板）
 * ============================================================================ */

/**
 * @brief 摆动相默认参数
 * @note 针对80kg负载/300Nm峰值扭矩电机优化，使用电机默认PID
 * 速度/加速度限制单位: rpm, rpm/s (电机驱动器单位)
 */
#define FUSION_SWING_TORQUE_LIMIT       300.0f   /* 力矩限制（Nm）- 匹配峰值扭矩 */
#define FUSION_SWING_SPEED_LIMIT        60.0f    /* 速度限制（rpm）- ≈360deg/s */
#define FUSION_SWING_ACCEL_LIMIT        100.0f   /* 加速度限制（rpm/s）- ≈600deg/s² */
/* PID参数已禁用，使用电机默认PID，此处定义为0 */
#define FUSION_SWING_POS_KP             0.0f    /* 位置环Kp - 已禁用 */
#define FUSION_SWING_POS_KI             0.0f    /* 位置环Ki - 已禁用 */
#define FUSION_SWING_POS_KD             0.0f    /* 位置环Kd - 已禁用 */

/* V14.0: 关节差异化PID参数 - 完全同步copy版本 */
#define JOINT_PID_JOINT1_KP             80.0f   /* Hip关节 - Kp */
#define JOINT_PID_JOINT1_KI             0.12f   /* Hip关节 - Ki */
#define JOINT_PID_JOINT1_KD             1.0f    /* Hip关节 - Kd */
#define JOINT_PID_JOINT1_FF_VEL         0.8f    /* Hip关节 - 速度前馈 - V14.0 同步copy版本 */
#define JOINT_PID_JOINT1_FF_ACCEL       0.20f   /* Hip关节 - 加速度前馈 - V14.0 同步copy版本 */

#define JOINT_PID_JOINT2_KP             300.0f  /* Thigh关节 - Kp - V8.1 同步仿真 */
#define JOINT_PID_JOINT2_KI             0.25f   /* Thigh关节 - Ki - V8.1 同步仿真 */
#define JOINT_PID_JOINT2_KD             18.0f   /* Thigh关节 - Kd - V8.1 同步仿真 */
#define JOINT_PID_JOINT2_FF_VEL         0.80f   /* Thigh关节 - 速度前馈 - V14.0 同步copy版本 */
#define JOINT_PID_JOINT2_FF_ACCEL       0.25f   /* Thigh关节 - 加速度前馈 - V14.0 同步copy版本 */

#define JOINT_PID_JOINT3_KP             34.0f   /* Knee关节 - Kp */
#define JOINT_PID_JOINT3_KI             0.02f   /* Knee关节 - Ki */
#define JOINT_PID_JOINT3_KD             1.9f    /* Knee关节 - Kd */
#define JOINT_PID_JOINT3_FF_VEL         0.20f   /* Knee关节 - 速度前馈 - V14.0 同步copy版本 */
#define JOINT_PID_JOINT3_FF_ACCEL       0.05f   /* Knee关节 - 加速度前馈 - V14.0 同步copy版本 */

/* V8.1: 电机模型差异化参数 - 阻尼和摩擦补偿 */
#define JOINT_MOTOR_DAMPING_JOINT1      0.025f  /* Hip关节 - 阻尼 */
#define JOINT_MOTOR_FRICTION_JOINT1     0.012f  /* Hip关节 - 摩擦 */

#define JOINT_MOTOR_DAMPING_JOINT2      0.090f  /* Thigh关节 - 阻尼 - V8.1 同步仿真 */
#define JOINT_MOTOR_FRICTION_JOINT2     0.026f  /* Thigh关节 - 摩擦 */

#define JOINT_MOTOR_DAMPING_JOINT3      0.145f  /* Knee关节 - 阻尼 */
#define JOINT_MOTOR_FRICTION_JOINT3     0.065f  /* Knee关节 - 摩擦 */

/* V8.0: 自适应前馈控制阈值 - 极值点检测 */
#define ADAPTIVE_FF_DISTANCE_THRESHOLD   55.0f   /* 极值点距离阈值 (deg) */
#define ADAPTIVE_FF_REDUCTION_FACTOR     0.30f   /* 极值点前馈降低因子 (70%降低) */

/**
 * @brief 支撑相默认参数（力位混合）
 * @note 高负载支撑需要更大的力矩输出
 */
#define FUSION_STANCE_FORCE_THRESHOLD   50.0f    /* 进入支撑相力阈值（N）*/
#define FUSION_SWING_FORCE_THRESHOLD    30.0f    /* 释放支撑相力阈值（N）- 滞后处理 */
#define FUSION_STANCE_TORQUE_TARGET     80.0f    /* 目标支撑力矩（Nm）- 支撑80kg */
#define FUSION_STANCE_POS_LIMIT         45.0f    /* 位置限幅（度）- 更大运动范围 */
#define FUSION_STANCE_TRANSITION_TIME   0.05f    /* 过渡时间（s）- 快速切换 */
#define FUSION_STANCE_TORQUE_SLOPE      50.0f    /* 力矩斜率（Nm/s）- 斜坡响应 */

/**
 * @brief 支撑相力矩环PID参数（已禁用，使用电机默认PID）
 */
//#define FUSION_STANCE_FORCE_KP          5.0f     /* 力矩环Kp - 已禁用 */
//#define FUSION_STANCE_FORCE_KI         0.5f     /* 力矩环Ki - 已禁用 */
//#define FUSION_STANCE_FORCE_KD         0.2f     /* 力矩环Kd - 已禁用 */

/**
 * @brief 速度环PID参数（已禁用，使用电机默认PID）
 */
//#define FUSION_SPEED_KP                15.0f    /* 速度环Kp - 已禁用 */
//#define FUSION_SPEED_KI                0.2f     /* 速度环Ki - 已禁用 */
//#define FUSION_SPEED_KD                0.5f     /* 速度环Kd - 已禁用 */

/**
 * @brief 电流环PID参数
 */
#define FUSION_CURRENT_KP              10.0f    /* 电流环Kp */
#define FUSION_CURRENT_KI              1.0f     /* 电流环Ki */
#define FUSION_CURRENT_KD              0.1f     /* 电流环Kd */

/**
 * @brief 自适应控制参数
 */
#define FUSION_ADAPTIVE_ENABLE         1         /* 自适应控制使能 */
#define FUSION_LOAD_ESTIMATE_WINDOW    10        /* 负载估计窗口大小 */
#define FUSION_ERROR_THRESHOLD_HIGH    10.0f     /* 位置误差阈值（度）- 高 */
#define FUSION_ERROR_THRESHOLD_LOW     2.0f      /* 位置误差阈值（度）- 低 */
#define FUSION_TORQUE_SAFETY_LIMIT     250.0f    /* 安全扭矩限制（Nm）*/

/**
 * @brief 场景识别默认参数
 */
#define FUSION_SCENE_FORCE_THRESHOLD    50.0f    /* 场景切换力阈值（N）*/
#define FUSION_SCENE_ANGLE_THRESHOLD    20.0f    /* 角度变化阈值（度）*/
#define FUSION_SCENE_DEBOUNCE_COUNT     3        /* 防抖动计数 */

/**
 * @brief 电机反馈监控参数
 */
#define FUSION_TEMP_WARNING             80.0f     /* 温度警告阈值（℃）*/
#define FUSION_TEMP_CRITICAL           100.0f    /* 温度危险阈值（℃）*/
#define FUSION_CURRENT_WARNING         30.0f     /* 电流警告阈值（A）*/

/* ============================================================================
 * 初始化函数
 * ============================================================================ */

/**
 * @brief 初始化指令融合系统
 * @return 0=成功, 非0=失败
 */
uint8_t Fusion_Init(void);

/**
 * @brief 反初始化指令融合系统
 */
void Fusion_Deinit(void);

/* ============================================================================
 * 摆动相控制（位置优先）
 * ============================================================================ */

/**
 * @brief 发送摆动相融合指令（带轨迹规划）
 * @param motor_id 电机索引 (0-11)
 * @param target_pos 目标位置（度）
 * @return 0=成功, 非0=失败
 * @note V2.1: 只设置目标位置，轨迹规划在控制循环中执行
 *       控制循环以200Hz频率调用Trajectory_StepAll()实现平滑运动
 *       内部操作：
 *       1. 切换到位置控制模式
 *       2. 设置力矩限制
 *       3. Trajectory_SetTarget() 设置目标到规划器
 *       （轨迹规划执行和CANFD发送在quad_control_loop.c中完成）
 */
uint8_t Fusion_SendSwingCommand(uint8_t motor_id, float target_pos);

/**
 * @brief 发送摆动相融合指令（直接控制，不使用轨迹规划）
 * @param motor_id 电机索引 (0-11)
 * @param target_pos 目标位置（度）
 * @return 0=成功, 非0=失败
 * @note 用于需要直接控制位置的场合，不经过轨迹规划
 */
uint8_t Fusion_SendSwingCommandDirect(uint8_t motor_id, float target_pos);

/**
 * @brief 发送摆动相融合指令（带自适应PID）
 * @param motor_id 电机索引 (0-11)
 * @param target_pos 目标位置（度）
 * @param kp 位置环Kp（可选，0表示使用默认值）
 * @param ki 位置环Ki（可选，0表示使用默认值）
 * @param kd 位置环Kd（可选，0表示使用默认值）
 * @return 0=成功, 非0=失败
 */
uint8_t Fusion_SendSwingCommandEx(uint8_t motor_id, float target_pos,
                                   float kp, float ki, float kd);

/**
 * @brief 批量发送摆动相指令（优化版）
 * @param motor_ids 电机索引数组
 * @param target_positions 目标位置数组
 * @param count 电机数量
 * @return 成功发送的电机数量
 * @note 优化：相同参数只发送一次，减少CANFD帧数
 */
uint8_t Fusion_SendBatchSwingCommands(uint8_t* motor_ids,
                                       float* target_positions,
                                       uint8_t count);

/* ============================================================================
 * 支撑相控制（力位混合）
 * ============================================================================ */

/**
 * @brief 发送支撑相融合指令（力位混合）
 * @param motor_id 电机索引 (0-11)
 * @param target_force 目标力矩（Nm）
 * @param pos_limit 位置限幅（度）
 * @return 0=成功, 非0=失败
 * @note 内部调用多条底层指令：
 *       1. motor_set_mode(TORQUE_MODE)
 *       2. motor_set_torque()
 *       3. motor_set_pos_upper_limit()
 *       4. motor_set_pos_lower_limit()
 */
uint8_t Fusion_SendStanceCommand(uint8_t motor_id,
                                  float target_force,
                                  float pos_limit);

/**
 * @brief 批量发送支撑相指令
 * @param motor_ids 电机索引数组
 * @param target_forces 目标力矩数组
 * @param pos_limits 位置限幅数组
 * @param count 电机数量
 * @return 成功发送的电机数量
 */
uint8_t Fusion_SendBatchStanceCommands(uint8_t* motor_ids,
                                        float* target_forces,
                                        float* pos_limits,
                                        uint8_t count);

/* ============================================================================
 * 通用融合指令
 * ============================================================================ */

/**
 * @brief 发送通用融合指令
 * @param motor_id 电机索引 (0-11)
 * @param cmd 融合指令结构体指针
 * @return 0=成功, 非0=失败
 * @note 根据cmd中的参数自动选择需要发送的底层指令
 */
uint8_t Fusion_SendCommand(uint8_t motor_id, FusedCommand_t* cmd);

/**
 * @brief 批量发送融合指令（最灵活）
 * @param motor_ids 电机索引数组
 * @param commands 融合指令数组
 * @param count 电机数量
 * @return 成功发送的电机数量
 * @note 自动优化批量发送，合并相同参数
 */
uint8_t Fusion_SendBatchCommands(uint8_t* motor_ids,
                                  FusedCommand_t* commands,
                                  uint8_t count);

/* ============================================================================
 * 参数缓存管理
 * ============================================================================ */

/**
 * @brief 清除指定电机的参数缓存
 * @param motor_id 电机索引 (0-11)
 * @note 下次发送时将强制更新所有参数
 */
void Fusion_ClearCache(uint8_t motor_id);

/**
 * @brief 清除所有电机的参数缓存
 */
void Fusion_ClearAllCache(void);

/**
 * @brief 强制刷新所有参数（忽略缓存）
 * @param motor_id 电机索引 (0-11)
 * @param cmd 融合指令结构体指针
 * @return 0=成功, 非0=失败
 */
uint8_t Fusion_ForceUpdate(uint8_t motor_id, FusedCommand_t* cmd);

/* ============================================================================
 * 预设模板（快速配置）
 * ============================================================================ */

/**
 * @brief 获取摆动相预设参数模板
 * @param cmd 融合指令结构体指针（输出）
 * @param target_pos 目标位置（度）
 */
void Fusion_GetSwingTemplate(FusedCommand_t* cmd, float target_pos);

/**
 * @brief 获取支撑相预设参数模板
 * @param cmd 融合指令结构体指针（输出）
 * @param target_force 目标力矩（Nm）
 * @param pos_limit 位置限幅（度）
 */
void Fusion_GetStanceTemplate(FusedCommand_t* cmd,
                               float target_force,
                               float pos_limit);

/**
 * @brief 获取停止预设参数模板
 * @param cmd 融合指令结构体指针（输出）
 * @param current_pos 当前位置（度）
 */
void Fusion_GetStopTemplate(FusedCommand_t* cmd, float current_pos);

/* ============================================================================
 * 统计和调试
 * ============================================================================ */

/**
 * @brief 获取发送统计信息
 * @param total_commands 总指令数（输出）
 * @param total_frames 总CANFD帧数（输出）
 * @param optimization_rate 优化率（输出，百分比）
 */
void Fusion_GetStats(uint32_t* total_commands,
                     uint32_t* total_frames,
                     float* optimization_rate);

/**
 * @brief 重置统计信息
 */
void Fusion_ResetStats(void);

/**
 * @brief 设置控制模式（运行时切换）
 * @param mode: 0=纯位置控制, 1=力位混合控制
 * @return 0=成功, 1=失败
 */
uint8_t Fusion_SetControlMode(uint8_t mode);

/**
 * @brief 获取当前控制模式
 * @return 当前控制模式
 */
uint8_t Fusion_GetControlMode(void);

/**
 * @brief 打印融合层状态（调试用）
 */
void Fusion_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_COMMAND_FUSION_H */
