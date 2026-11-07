/**
 ****************************************************************************************************
 * @file        quad_motor_api.h
 * @version     V1.0
 * @brief       四足机器狗 - 高层电机控制API（用户接口）
 * @details     简化的用户API，一行代码实现复杂的电机控制
 ****************************************************************************************************
 * @attention
 *
 * 使用示例：
 * ```c
 * // 初始化
 * QuadMotor_Init();
 * QuadMotor_EnableHybridControl(true);
 *
 * // 主循环（50Hz）
 * while(1) {
 *     // 接收上位机RL控制器数据
 *     float target_angles[12];
 *     tcp_receive_rl_data(target_angles);
 *
 *     // ⭐一行代码搞定！
 *     QuadMotor_UpdateFromRL(target_angles);
 *
 *     osDelay(20);  // 50Hz
 * }
 * ```
 *
 ****************************************************************************************************
 */

#ifndef __QUAD_MOTOR_API_H
#define __QUAD_MOTOR_API_H

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
 * @brief 初始化四足电机控制系统
 * @return 0=成功, 非0=失败
 * @note  必须在使用其他API前调用
 *        内部会初始化：底层驱动、指令融合层、智能控制器
 */
uint8_t QuadMotor_Init(void);

/**
 * @brief 反初始化四足电机控制系统
 */
void QuadMotor_Deinit(void);

/**
 * @brief 启动智能控制系统
 * @return 0=成功, 非0=失败
 * @note  调用此函数后，系统进入运行状态
 */
uint8_t QuadMotor_Start(void);

/**
 * @brief 停止智能控制系统
 */
void QuadMotor_Stop(void);

/* ============================================================================
 * 核心控制API（⭐最常用）
 * ============================================================================ */

/**
 * @brief 从RL控制器更新12电机目标角度（⭐核心API）
 * @param target_angles 12个关节目标角度数组（度）
 * @return 0=成功, 非0=失败
 *
 * @note  ⭐这是最核心的API⭐
 *        自动完成：
 *        1. 场景识别（摆动/支撑/过渡/停止）
 *        2. 策略选择（位置/力位混合/阻抗）
 *        3. 指令融合（优化CANFD帧数）
 *        4. 力位混合自动切换
 *        5. 足端力反馈处理
 *
 * @example
 *        float angles[12] = {10, 20, 30, ...};  // 从上位机接收
 *        QuadMotor_UpdateFromRL(angles);         // 一行搞定！
 */
uint8_t QuadMotor_UpdateFromRL(float target_angles[QUAD_MAX_MOTORS]);

/* ============================================================================
 * 高级控制API
 * ============================================================================ */

/**
 * @brief 使能/失能力位混合控制
 * @param enable true=使能, false=失能（纯位置控制）
 * @note  默认使能
 *        失能后所有电机使用纯位置控制（摆动相参数）
 */
void QuadMotor_EnableHybridControl(bool enable);

/**
 * @brief 设置单条腿的控制模式
 * @param leg_id 腿部ID (0=前左, 1=前右, 2=后左, 3=后右)
 * @param mode 控制模式
 * @return 0=成功, 非0=失败
 *
 * @note  可以为不同的腿设置不同的模式
 *        例如：前腿摆动时，后腿支撑
 */
uint8_t QuadMotor_SetLegMode(QuadLegID_t leg_id, QuadLegMode_t mode);

/**
 * @brief 设置单个电机的目标位置（手动控制）
 * @param motor_id 电机索引 (0-11)
 * @param target_position 目标位置（度）
 * @return 0=成功, 非0=失败
 *
 * @note  用于单电机调试或特殊控制
 *        仍会应用智能控制策略
 */
uint8_t QuadMotor_SetPosition(uint8_t motor_id, float target_position);

/**
 * @brief 批量设置电机位置（可指定任意数量）
 * @param motor_ids 电机索引数组
 * @param target_positions 目标位置数组
 * @param count 电机数量
 * @return 成功设置的电机数量
 */
uint8_t QuadMotor_SetBatchPositions(uint8_t* motor_ids,
                                     float* target_positions,
                                     uint8_t count);

/* ============================================================================
 * 状态查询API
 * ============================================================================ */

/**
 * @brief 获取所有电机状态
 * @param status 12电机状态数组（输出）
 * @return 0=成功, 非0=失败
 */
uint8_t QuadMotor_GetAllStatus(QuadMotorStatus_t status[QUAD_MAX_MOTORS]);

/**
 * @brief 获取单个电机状态
 * @param motor_id 电机索引 (0-11)
 * @param status 电机状态结构体指针（输出）
 * @return 0=成功, 非0=失败
 */
uint8_t QuadMotor_GetStatus(uint8_t motor_id, QuadMotorStatus_t* status);

/**
 * @brief 获取4条腿的状态
 * @param leg_status 4条腿状态数组（输出）
 * @return 0=成功, 非0=失败
 */
uint8_t QuadMotor_GetLegStatus(QuadLegStatus_t leg_status[QUAD_NUM_LEGS]);

/**
 * @brief 检查是否所有电机都到达目标位置
 * @param tolerance 容差（度）
 * @return true=全部到达, false=未到达
 */
bool QuadMotor_IsAllMotorsReached(float tolerance);

/**
 * @brief 检查指定腿是否到达目标位置
 * @param leg_id 腿部ID (0-3)
 * @param tolerance 容差（度）
 * @return true=到达, false=未到达
 */
bool QuadMotor_IsLegReached(QuadLegID_t leg_id, float tolerance);

/* ============================================================================
 * 安全控制API
 * ============================================================================ */

/**
 * @brief 紧急停止所有电机
 * @note  立即停止所有电机运动，保持当前位置
 */
void QuadMotor_EmergencyStop(void);

/**
 * @brief 使能/失能所有电机
 * @param enable true=使能, false=失能
 */
void QuadMotor_EnableAll(bool enable);

/**
 * @brief 使能/失能单个电机
 * @param motor_id 电机索引 (0-11)
 * @param enable true=使能, false=失能
 * @return 0=成功, 非0=失败
 */
uint8_t QuadMotor_Enable(uint8_t motor_id, bool enable);

/**
 * @brief 检查系统是否有错误
 * @return true=有错误, false=正常
 */
bool QuadMotor_HasError(void);

/**
 * @brief 获取错误信息
 * @param error_motors 错误电机ID数组（输出）
 * @param max_count 数组最大容量
 * @return 错误电机数量
 */
uint8_t QuadMotor_GetErrors(uint8_t* error_motors, uint8_t max_count);

/**
 * @brief 清除所有错误
 */
void QuadMotor_ClearErrors(void);

/* ============================================================================
 * 参数配置API
 * ============================================================================ */

/**
 * @brief 配置摆动相参数（全局）
 * @param params 摆动相参数结构体指针
 */
void QuadMotor_ConfigSwingParams(SwingParams_t* params);

/**
 * @brief 配置支撑相参数（单电机）
 * @param motor_id 电机索引 (0-11)
 * @param params 支撑相参数结构体指针
 */
void QuadMotor_ConfigStanceParams(uint8_t motor_id, StanceParams_t* params);

/**
 * @brief 配置场景识别参数
 * @param params 场景识别参数结构体指针
 */
void QuadMotor_ConfigSceneParams(SceneRecognitionParams_t* params);

/* ============================================================================
 * 统计和调试API
 * ============================================================================ */

/**
 * @brief 获取系统运行统计
 * @param total_updates 总更新次数（输出）
 * @param total_canfd_frames 总CANFD帧数（输出）
 * @param optimization_rate 优化率（输出，百分比）
 */
void QuadMotor_GetStats(uint32_t* total_updates,
                        uint32_t* total_canfd_frames,
                        float* optimization_rate);

/**
 * @brief 打印系统状态（调试用）
 * @param detailed true=详细信息, false=简要信息
 */
void QuadMotor_PrintStatus(bool detailed);

/**
 * @brief 打印单个电机状态
 * @param motor_id 电机索引 (0-11)
 */
void QuadMotor_PrintMotorStatus(uint8_t motor_id);

/**
 * @brief 重置所有统计信息
 */
void QuadMotor_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_MOTOR_API_H */
