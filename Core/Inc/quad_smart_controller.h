/**
 ****************************************************************************************************
 * @file        quad_smart_controller.h
 * @version     V1.0
 * @brief       四足机器狗 - 智能控制核心
 * @details     场景识别、策略选择、力位混合状态机、反馈处理
 ****************************************************************************************************
 * @attention
 *
 * 核心功能：
 * 1. 场景识别器 - 自动识别摆动相/支撑相/过渡/停止
 * 2. 策略选择器 - 根据场景选择控制策略
 * 3. 力位混合状态机 - 3状态自动切换
 * 4. 反馈处理器 - 电机反馈+力传感器融合
 *
 ****************************************************************************************************
 */

#ifndef __QUAD_SMART_CONTROLLER_H
#define __QUAD_SMART_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "quad_types.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 初始化和配置
 * ============================================================================ */

/**
 * @brief 初始化智能控制器
 * @return 0=成功, 非0=失败
 */
uint8_t SmartCtrl_Init(void);

/**
 * @brief 反初始化智能控制器
 */
void SmartCtrl_Deinit(void);

/**
 * @brief 使能/失能智能控制
 * @param enable true=使能, false=失能
 */
void SmartCtrl_Enable(bool enable);

/* ============================================================================
 * 场景识别器
 * ============================================================================ */

/**
 * @brief 识别单个电机的运动场景
 * @param motor_id 电机索引 (0-11)
 * @param current_angle 当前角度（度）
 * @param target_angle 目标角度（度）
 * @param current_force 当前足端力（N）
 * @return 识别的场景类型
 *
 * @note 识别逻辑：
 *       - 摆动相: 足端力 < 阈值 且 角度变化大
 *       - 支撑相: 足端力 > 阈值 且 角度变化小
 *       - 过渡: 力快速变化
 *       - 停止: 目标==当前 且 速度≈0
 */
QuadScene_t Scene_Identify(uint8_t motor_id,
                           float current_angle,
                           float target_angle,
                           float current_force);

/**
 * @brief 识别腿部场景（基于3个关节的综合判断）
 * @param leg_id 腿部ID (0-3)
 * @return 腿部场景类型
 * @note 综合3个关节的场景，采用多数投票机制
 */
QuadScene_t Scene_IdentifyLeg(QuadLegID_t leg_id);

/**
 * @brief 配置场景识别参数
 * @param params 场景识别参数结构体指针
 */
void Scene_ConfigParams(SceneRecognitionParams_t* params);

/**
 * @brief 获取当前场景识别参数
 * @param params 场景识别参数结构体指针（输出）
 */
void Scene_GetParams(SceneRecognitionParams_t* params);

/* ============================================================================
 * 策略选择器
 * ============================================================================ */

/**
 * @brief 根据场景选择控制策略
 * @param scene 运动场景
 * @return 控制策略
 *
 * @note 默认映射：
 *       - SCENE_SWING    → STRATEGY_POSITION
 *       - SCENE_STANCE   → STRATEGY_HYBRID
 *       - SCENE_TRANSITION → STRATEGY_IMPEDANCE
 *       - SCENE_STOP     → STRATEGY_POSITION
 */
QuadStrategy_t Strategy_Select(QuadScene_t scene);

/**
 * @brief 手动设置策略（覆盖自动选择）
 * @param motor_id 电机索引 (0-11)
 * @param strategy 控制策略
 * @note 用于调试或特殊控制需求
 */
void Strategy_SetManual(uint8_t motor_id, QuadStrategy_t strategy);

/**
 * @brief 恢复自动策略选择
 * @param motor_id 电机索引 (0-11)
 */
void Strategy_RestoreAuto(uint8_t motor_id);

/* ============================================================================
 * 力位混合控制状态机
 * ============================================================================ */

/**
 * @brief 更新力位混合控制状态机
 * @param motor_id 电机索引 (0-11)
 * @return 0=成功, 非0=失败
 *
 * @note 状态机逻辑：
 *       [位置控制] --力矩超阈值--> [过渡] --时间到--> [力控制]
 *            ↑                                          ↓
 *            └─────────────位置偏离大───────────────────┘
 *
 *       状态1: 位置控制
 *         - 发送位置目标
 *         - 监测实际力矩
 *         - 力矩 > 阈值 → 状态2
 *
 *       状态2: 过渡阶段
 *         - 平滑切换控制模式
 *         - 延迟0.05-0.1s
 *         - 时间到 → 状态3
 *
 *       状态3: 力控制
 *         - 发送力矩目标
 *         - 监测实际位置
 *         - 位置偏离 > 阈值 → 状态1
 */
uint8_t HybridCtrl_Update(uint8_t motor_id);

/**
 * @brief 获取力位混合控制状态
 * @param motor_id 电机索引 (0-11)
 * @return 当前状态
 */
QuadHybridState_t HybridCtrl_GetState(uint8_t motor_id);

/**
 * @brief 配置力位混合参数
 * @param motor_id 电机索引 (0-11)
 * @param params 支撑相参数结构体指针
 */
void HybridCtrl_ConfigParams(uint8_t motor_id, StanceParams_t* params);

/**
 * @brief 强制切换到指定状态（调试用）
 * @param motor_id 电机索引 (0-11)
 * @param state 目标状态
 */
void HybridCtrl_ForceState(uint8_t motor_id, QuadHybridState_t state);

/* ============================================================================
 * 反馈处理器
 * ============================================================================ */

/**
 * @brief 更新电机反馈数据
 * @param motor_id 电机索引 (0-11)
 * @param position 实际位置（度）
 * @param velocity 实际速度（deg/s）
 * @param torque 实际力矩（Nm）
 * @param temperature 电机温度（℃）
 * @param error_flags 错误标志
 */
void Feedback_UpdateMotor(uint8_t motor_id,
                         float position,
                         float velocity,
                         float torque,
                         float temperature,
                         uint16_t error_flags);

/**
 * @brief 更新足端力传感器数据
 * @param foot_id 足端ID (0-3)
 * @param force 足端力（N）
 */
void Feedback_UpdateFootForce(QuadFootID_t foot_id, float force);

/**
 * @brief 获取电机状态
 * @param motor_id 电机索引 (0-11)
 * @param status 电机状态结构体指针（输出）
 * @return 0=成功, 非0=失败
 */
uint8_t Feedback_GetMotorStatus(uint8_t motor_id, QuadMotorStatus_t* status);

/**
 * @brief 获取腿部状态
 * @param leg_id 腿部ID (0-3)
 * @param status 腿部状态结构体指针（输出）
 * @return 0=成功, 非0=失败
 */
uint8_t Feedback_GetLegStatus(QuadLegID_t leg_id, QuadLegStatus_t* status);

/**
 * @brief 检查反馈超时
 * @param motor_id 电机索引 (0-11)
 * @param timeout_ms 超时时间（毫秒）
 * @return true=超时, false=正常
 */
bool Feedback_IsTimeout(uint8_t motor_id, uint32_t timeout_ms);

/* ============================================================================
 * 主控制更新函数
 * ============================================================================ */

/**
 * @brief 智能控制器主更新函数（单电机）
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置（度）
 * @return 0=成功, 非0=失败
 *
 * @note 完整流程：
 *       1. 获取当前状态（位置、力传感器）
 *       2. 场景识别
 *       3. 策略选择
 *       4. 根据策略发送指令
 *       5. 更新状态机（如果是混合模式）
 */
uint8_t SmartCtrl_UpdateMotor(uint8_t motor_id, float target_position);

/**
 * @brief 智能控制器主更新函数（12电机批量）
 * @param target_positions 12个目标位置数组（度）
 * @return 成功更新的电机数量
 *
 * @note 这是高层API调用的核心函数
 */
uint8_t SmartCtrl_UpdateAll(float target_positions[QUAD_MAX_MOTORS]);

/* ============================================================================
 * 高级配置和查询函数
 * ============================================================================ */

/**
 * @brief 使能/失能力位混合控制
 * @param enable true=使能, false=失能
 */
void SmartCtrl_EnableHybrid(bool enable);

/**
 * @brief 设置单个电机的控制策略
 * @param motor_id 电机索引 (0-11)
 * @param strategy 控制策略
 */
void SmartCtrl_SetMotorStrategy(uint8_t motor_id, QuadStrategy_t strategy);

/**
 * @brief 配置摆动相参数（全局）
 * @param params 摆动相参数结构体指针
 */
void SmartCtrl_ConfigSwingParams(SwingParams_t* params);

/**
 * @brief 配置支撑相参数（单电机）
 * @param motor_id 电机索引 (0-11)
 * @param params 支撑相参数结构体指针
 */
void SmartCtrl_ConfigStanceParams(uint8_t motor_id, StanceParams_t* params);

/**
 * @brief 配置场景识别参数
 * @param params 场景识别参数结构体指针
 */
void SmartCtrl_ConfigSceneParams(SceneRecognitionParams_t* params);

/**
 * @brief 获取电机当前场景
 * @param motor_id 电机索引 (0-11)
 * @return 当前场景类型
 */
QuadScene_t SmartCtrl_GetMotorScene(uint8_t motor_id);

/**
 * @brief 获取电机当前控制策略
 * @param motor_id 电机索引 (0-11)
 * @return 当前控制策略
 */
QuadStrategy_t SmartCtrl_GetMotorStrategy(uint8_t motor_id);

/**
 * @brief 获取电机力位混合状态
 * @param motor_id 电机索引 (0-11)
 * @return 当前混合状态
 */
QuadHybridState_t SmartCtrl_GetHybridState(uint8_t motor_id);

/* ============================================================================
 * 统计和调试
 * ============================================================================ */

/**
 * @brief 获取场景识别统计
 * @param scene_counts 各场景出现次数数组（输出）[5]
 * @param total_updates 总更新次数（输出）
 */
void SmartCtrl_GetSceneStats(uint32_t scene_counts[5], uint32_t* total_updates);

/**
 * @brief 获取策略使用统计
 * @param strategy_counts 各策略使用次数数组（输出）[4]
 */
void SmartCtrl_GetStrategyStats(uint32_t strategy_counts[4]);

/**
 * @brief 获取状态切换统计
 * @param motor_id 电机索引 (0-11)
 * @param switch_count 状态切换次数（输出）
 * @param current_state 当前状态（输出）
 */
void SmartCtrl_GetSwitchStats(uint8_t motor_id,
                              uint32_t* switch_count,
                              QuadHybridState_t* current_state);

/**
 * @brief 重置所有统计信息
 */
void SmartCtrl_ResetStats(void);

/**
 * @brief 打印智能控制器状态（调试用）
 * @param motor_id 电机索引 (0-11), 0xFF=打印全部
 */
void SmartCtrl_PrintStatus(uint8_t motor_id);

/**
 * @brief 获取电机位置误差（用于诊断）
 * @param motor_id 电机索引 (0-11)
 * @return 位置误差（目标-实际）（度）
 */
float SmartCtrl_GetPositionError(uint8_t motor_id);

/**
 * @brief 获取自适应PID参数
 * @param motor_id 电机索引 (0-11)
 * @param kp 输出：比例系数
 * @param ki 输出：积分系数
 * @param kd 输出：微分系数
 */
void SmartCtrl_GetAdaptivePID(uint8_t motor_id, float* kp, float* ki, float* kd);

/**
 * @brief 检查电机是否需要反馈数据
 * @param motor_id 电机索引 (0-11)
 * @return true=需要, false=不需要
 */
bool SmartCtrl_NeedsFeedback(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_SMART_CONTROLLER_H */
