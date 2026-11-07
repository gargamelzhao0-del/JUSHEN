/**
 ****************************************************************************************************
 * @file        quad_motor_api.c
 * @version     V1.0
 * @brief       四足机器狗 - 高层电机控制API实现
 * @details     实现简化的用户API，一行代码实现复杂的电机控制
 ****************************************************************************************************
 * @attention
 *
 * 架构说明：
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  quad_motor_api.c (本层)                                        │
 * │  - QuadMotor_UpdateFromRL() ⭐核心API                           │
 * │  - QuadMotor_EnableHybridControl()                              │
 * │  - QuadMotor_SetLegMode()                                       │
 * └────────────────────┬────────────────────────────────────────────┘
 *                      ↓
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  quad_smart_controller.c (智能控制层)                           │
 * │  - SmartCtrl_UpdateAll()      场景识别 + 策略选择               │
 * │  - HybridCtrl_Update()        力位混合自动切换                  │
 * │  - Feedback_UpdateMotor()     反馈处理                          │
 * └────────────────────┬────────────────────────────────────────────┘
 *                      ↓
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  quad_command_fusion.c (指令融合层)                             │
 * │  - Fusion_SendSwingCommand()  参数缓存 + 指令融合               │
 * │  - Fusion_SendStanceCommand() 力位混合参数配置                  │
 * └────────────────────┬────────────────────────────────────────────┘
 *                      ↓
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  motor_control.c (底层驱动)                                     │
 * │  - motor_set_position()       CANFD命令发送                     │
 * │  - motor_set_torque_limit()   底层硬件接口                      │
 * └─────────────────────────────────────────────────────────────────┘
 *
 ****************************************************************************************************
 */

#include "quad_motor_api.h"
#include "quad_smart_controller.h"
#include "quad_command_fusion.h"
#include "quad_control_loop.h"
#include "motor_control.h"
#include "motor_control_enhanced.h"  /* 电机句柄管理 */
#include "trajectory_planner.h"      /* V5.0: 轨迹规划器，用于启动回零 */
#include "system_config.h"           /* V5.0: MOTOR_DEFAULT_ANGLE_DEG等配置 */
#include "foot_force_sensor.h"
#include "cmsis_os.h"               /* V5.0: osDelay */
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * 私有类型定义
 * ============================================================================ */

/**
 * @brief 系统状态枚举
 */
typedef enum {
    SYS_STATE_UNINIT    = 0,    /* 未初始化 */
    SYS_STATE_IDLE      = 1,    /* 空闲状态 */
    SYS_STATE_RUNNING   = 2,    /* 运行中 */
    SYS_STATE_ERROR     = 3,    /* 错误状态 */
} SystemState_t;

/* ============================================================================
 * 私有全局变量
 * ============================================================================ */

/* 系统状态 */
static SystemState_t g_system_state = SYS_STATE_UNINIT;

/* 力位混合控制使能标志 */
static bool g_hybrid_control_enabled = true;

/* 电机状态缓存（12电机）*/
static QuadMotorStatus_t g_motor_status[QUAD_MAX_MOTORS];

/* 腿部状态缓存（4条腿）*/
static QuadLegStatus_t g_leg_status[QUAD_NUM_LEGS];

/* 统计信息 */
static struct {
    uint32_t total_updates;         /* 总更新次数 */
    uint32_t update_errors;         /* 更新错误次数 */
    uint32_t emergency_stops;       /* 紧急停止次数 */
} g_api_stats = {0};

/* ============================================================================
 * 私有函数声明
 * ============================================================================ */

static uint8_t validate_motor_id(uint8_t motor_id);
static uint8_t validate_leg_id(QuadLegID_t leg_id);
static void update_motor_status_from_driver(uint8_t motor_id);
static void update_all_motor_status(void);

/* ============================================================================
 * 初始化和配置
 * ============================================================================ */

/**
 * @brief 初始化四足电机控制系统
 */
uint8_t QuadMotor_Init(void)
{
    if (g_system_state != SYS_STATE_UNINIT) {
        return 1;  /* 已经初始化过 */
    }

    /* 1. 初始化底层电机驱动（假设已在外部初始化）*/
    /* motor_control_init() 应该已在 main.c 中调用 */

    /* 2. 初始化指令融合层 */
    Fusion_Init();

    /* 3. 初始化智能控制器 */
    SmartCtrl_Init();

    /* 4. 清空状态缓存 */
    memset(g_motor_status, 0, sizeof(g_motor_status));
    memset(g_leg_status, 0, sizeof(g_leg_status));

    /* 5. 初始化电机ID */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        g_motor_status[i].motor_id = i;
        g_motor_status[i].enabled = false;
    }

    /* 6. 初始化腿部ID */
    for (uint8_t i = 0; i < QUAD_NUM_LEGS; i++) {
        g_leg_status[i].leg_id = (QuadLegID_t)i;
        g_leg_status[i].mode = LEG_MODE_HYBRID;  /* 默认混合模式 */
    }

    /* 7. 设置系统状态 */
    g_system_state = SYS_STATE_IDLE;

    return 0;  /* 成功 */
}

/**
 * @brief 反初始化四足电机控制系统
 */
void QuadMotor_Deinit(void)
{
    /* 1. 紧急停止所有电机 */
    QuadMotor_EmergencyStop();

    /* 2. 失能所有电机 */
    QuadMotor_EnableAll(false);

    /* 3. 重置系统状态 */
    g_system_state = SYS_STATE_UNINIT;
}

/**
 * @brief 启动智能控制系统
 * @note V2.0: 优化初始化流程，减少阻塞等待时间
 */
uint8_t QuadMotor_Start(void)
{
    if (g_system_state != SYS_STATE_IDLE) {
        return 1;
    }

    printf("[QuadMotor] Starting motor initialization...\r\n");

    /* === Step 0: 同步轨迹规划器到0°，匹配即将发送的初始位置 === */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        Trajectory_SetCurrentPosition(i, 0.0f);
    }

    /* === Step 1: 使能所有电机 + 设置参数 + 初始位置0° === */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        uint8_t can = (i < 6) ? 1 : 2;
        uint8_t device_id = i + 1;

        /* 1. 使能电机 */
        motor_enable(can, device_id, 1);

        /* 2. 设置为位置控制模式 */
        motor_set_mode(can, device_id, MOTOR_MODE_POSITION);

        /* 3. 设置限制参数 */
        motor_set_torque_limit(can, device_id, 300.0f);
        motor_set_limit_speed(can, device_id, 120.0f);
        motor_set_limit_accel(can, device_id, 200.0f);

        /* 4. 设置初始位置为0度 */
        motor_set_position(can, device_id, 0.0f);

        /* 5. 启动电机运行 */
        motor_start_stop(can, device_id, 1);

        printf("[QuadMotor] Motor %u (CAN%u, ID=%u) init sent\r\n", i, can, device_id);
    }

#if CLOSED_LOOP_ENABLED
    /* === 真实电机模式：读取当前位置 → 3秒运动到默认180° === */

    /* Step 2: 发送监测请求，获取电机当前位置 */
    printf("[QuadMotor] Reading motor positions...\r\n");
    for (uint8_t retry = 0; retry < MOTOR_HOMING_READ_TIMEOUT_MS / 10; retry++) {
        for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
            uint8_t can = (i < 6) ? 1 : 2;
            uint8_t device_id = i + 1;
            motor_read_monitor_data(can, device_id, NULL);
        }
        osDelay(10);

        /* 检查是否所有电机都已反馈 */
        uint8_t received = 0;
        for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
            MotorHandle* handle = motor_handle_get(i);
            if (handle != NULL && handle->initialized) {
                received++;
            }
        }
        if (received >= QUAD_MAX_MOTORS) break;
    }

    /* Step 3: 用实际位置更新轨迹规划器 */
    float actual_positions[QUAD_MAX_MOTORS];
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        MotorHandle* handle = motor_handle_get(i);
        if (handle != NULL && handle->initialized) {
            actual_positions[i] = handle->monitor.actual_position;
        } else {
            actual_positions[i] = 0.0f;
        }

        /* 更新轨迹规划器初始位置为实际位置 */
        Trajectory_SetCurrentPosition(i, actual_positions[i]);

        /* 电机保持在当前位置，不跳变 */
        uint8_t can = (i < 6) ? 1 : 2;
        uint8_t device_id = i + 1;
        motor_set_position(can, device_id, actual_positions[i]);

        printf("[QuadMotor] Motor %u actual position: %.2f°\r\n", i, actual_positions[i]);
    }

    /* Step 4: 解除启动抑制，允许控制循环发送CAN命令 */
    QuadControlLoop_SetStartupPhase(false);

    /* Step 5: 设置目标为180°，轨迹规划器自动平滑过渡 */
    printf("[QuadMotor] Homing to %.1f° over %d ms...\r\n",
           MOTOR_DEFAULT_ANGLE_DEG, MOTOR_HOMING_DURATION_MS);
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        Trajectory_SetContinuousTarget(i, MOTOR_DEFAULT_ANGLE_DEG);
    }

    /* Step 6: 等待回零完成 */
    osDelay(MOTOR_HOMING_DURATION_MS);
    printf("[QuadMotor] Homing complete, all motors at %.1f°\r\n", MOTOR_DEFAULT_ANGLE_DEG);
#else
    /* === 测试模式（CAN-FD分析仪）：不执行回零，电机保持0° === */
    printf("[QuadMotor] Test mode: homing disabled, motors at 0°\r\n");

    /* 解除启动抑制 */
    QuadControlLoop_SetStartupPhase(false);
#endif

    /* 设置系统状态 */
    g_system_state = SYS_STATE_RUNNING;

    printf("[QuadMotor] All 12 motors initialization complete\r\n");
    return 0;
}

/**
 * @brief 停止智能控制系统
 */
void QuadMotor_Stop(void)
{
    /* 失能所有电机 */
    QuadMotor_EnableAll(false);

    /* 设置系统状态 */
    g_system_state = SYS_STATE_IDLE;
}

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
 */
uint8_t QuadMotor_UpdateFromRL(float target_angles[QUAD_MAX_MOTORS])
{
    if (g_system_state != SYS_STATE_RUNNING) {
        static uint32_t last_log_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_log_tick > 2000) {
            printf("[QuadMotor] UpdateFromRL skipped: state=%d (expected RUNNING=%d)\r\n", g_system_state, SYS_STATE_RUNNING);
            last_log_tick = now;
        }
        return 1;
    }

    if (target_angles == NULL) {
        return 2;
    }

    /* 安全检查：验证所有目标角度是否在允许范围内 */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        if (target_angles[i] > QUAD_MOTOR_ANGLE_MAX || target_angles[i] < QUAD_MOTOR_ANGLE_MIN) {
            g_api_stats.update_errors++;
            printf("[QuadMotor] UpdateFromRL: angle out of range motor %d = %.1f\r\n", i, target_angles[i]);
            return 4;
        }
    }

    /* 统计更新次数 */
    g_api_stats.total_updates++;

    /*
     * ⭐核心调用：智能控制器统一处理⭐
     * SmartCtrl_UpdateAll 会自动完成：
     * - 场景识别（基于角度变化 + 足端力）
     * - 策略选择（SWING→位置控制, STANCE→力位混合）
     * - 指令融合（参数缓存 + 批量优化）
     * - 力位混合自动切换（3状态机）
     */
    uint8_t result = SmartCtrl_UpdateAll(target_angles);

    if (result != 0) {
        g_api_stats.update_errors++;
        return 3;
    }

    /* 更新电机状态缓存（从底层读取）*/
    update_all_motor_status();

    return 0;
}

/* ============================================================================
 * 高级控制API
 * ============================================================================ */

/**
 * @brief 使能/失能力位混合控制
 */
void QuadMotor_EnableHybridControl(bool enable)
{
    g_hybrid_control_enabled = enable;
    SmartCtrl_EnableHybrid(enable);
}

/**
 * @brief 设置单条腿的控制模式
 */
uint8_t QuadMotor_SetLegMode(QuadLegID_t leg_id, QuadLegMode_t mode)
{
    if (validate_leg_id(leg_id) != 0) {
        return 1;  /* 腿部ID无效 */
    }

    /* 更新腿部状态缓存 */
    g_leg_status[leg_id].mode = mode;

    /* 为该腿的3个关节设置控制模式 */
    for (uint8_t joint = 0; joint < QUAD_MOTORS_PER_LEG; joint++) {
        uint8_t motor_id = QUAD_MOTOR_INDEX(leg_id, joint);

        /* 根据模式设置智能控制器 */
        switch (mode) {
            case LEG_MODE_POSITION:
                /* 纯位置控制 - 通过智能控制器设置 */
                SmartCtrl_SetMotorStrategy(motor_id, STRATEGY_POSITION);
                break;

            case LEG_MODE_HYBRID:
                /* 力位混合 - 使能自动切换 */
                SmartCtrl_SetMotorStrategy(motor_id, STRATEGY_HYBRID);
                break;

            case LEG_MODE_TORQUE:
                /* 纯力矩控制 */
                SmartCtrl_SetMotorStrategy(motor_id, STRATEGY_TORQUE);
                break;

            case LEG_MODE_DISABLED:
                /* 失能该电机 */
                QuadMotor_Enable(motor_id, false);
                break;

            default:
                return 2;  /* 模式无效 */
        }
    }

    return 0;  /* 成功 */
}

/**
 * @brief 设置单个电机的目标位置（手动控制）
 */
uint8_t QuadMotor_SetPosition(uint8_t motor_id, float target_position)
{
    if (validate_motor_id(motor_id) != 0) {
        return 1;  /* 电机ID无效 */
    }

    /* 调用智能控制器的单电机更新 */
    return SmartCtrl_UpdateMotor(motor_id, target_position);
}

/**
 * @brief 批量设置电机位置
 */
uint8_t QuadMotor_SetBatchPositions(uint8_t* motor_ids,
                                     float* target_positions,
                                     uint8_t count)
{
    if (motor_ids == NULL || target_positions == NULL || count == 0) {
        return 0;  /* 参数错误 */
    }

    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (QuadMotor_SetPosition(motor_ids[i], target_positions[i]) == 0) {
            success_count++;
        }
    }

    return success_count;
}

/* ============================================================================
 * 状态查询API
 * ============================================================================ */

/**
 * @brief 获取所有电机状态
 */
uint8_t QuadMotor_GetAllStatus(QuadMotorStatus_t status[QUAD_MAX_MOTORS])
{
    if (status == NULL) {
        return 1;  /* 参数错误 */
    }

    /* 先更新状态缓存 */
    update_all_motor_status();

    /* 复制到输出数组 */
    memcpy(status, g_motor_status, sizeof(g_motor_status));

    return 0;  /* 成功 */
}

/**
 * @brief 获取单个电机状态
 */
uint8_t QuadMotor_GetStatus(uint8_t motor_id, QuadMotorStatus_t* status)
{
    if (validate_motor_id(motor_id) != 0 || status == NULL) {
        return 1;  /* 参数错误 */
    }

    /* 更新该电机状态 */
    update_motor_status_from_driver(motor_id);

    /* 复制到输出 */
    memcpy(status, &g_motor_status[motor_id], sizeof(QuadMotorStatus_t));

    return 0;  /* 成功 */
}

/**
 * @brief 获取4条腿的状态
 */
uint8_t QuadMotor_GetLegStatus(QuadLegStatus_t leg_status[QUAD_NUM_LEGS])
{
    if (leg_status == NULL) {
        return 1;  /* 参数错误 */
    }

    /* 更新所有电机状态 */
    update_all_motor_status();

    /* 构建腿部状态 */
    for (uint8_t leg = 0; leg < QUAD_NUM_LEGS; leg++) {
        g_leg_status[leg].leg_id = (QuadLegID_t)leg;

        /* 复制该腿的3个关节状态 */
        for (uint8_t joint = 0; joint < QUAD_MOTORS_PER_LEG; joint++) {
            uint8_t motor_id = QUAD_MOTOR_INDEX(leg, joint);
            memcpy(&g_leg_status[leg].joints[joint],
                   &g_motor_status[motor_id],
                   sizeof(QuadMotorStatus_t));
        }

        /* 获取足端力（从传感器）*/
        g_leg_status[leg].foot_force = foot_force_get_filtered((QuadFootID_t)leg);
        g_leg_status[leg].is_contact = foot_force_is_contact((QuadFootID_t)leg);

        /* 获取腿部整体场景（以髋关节为准）*/
        uint8_t hip_motor_id = QUAD_MOTOR_INDEX(leg, JOINT_HIP);
        g_leg_status[leg].scene = g_motor_status[hip_motor_id].scene;
    }

    /* 复制到输出数组 */
    memcpy(leg_status, g_leg_status, sizeof(g_leg_status));

    return 0;  /* 成功 */
}

/**
 * @brief 检查是否所有电机都到达目标位置
 */
bool QuadMotor_IsAllMotorsReached(float tolerance)
{
    update_all_motor_status();

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        float error = g_motor_status[i].target_position - g_motor_status[i].position;
        if (fabsf(error) > tolerance) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 检查指定腿是否到达目标位置
 */
bool QuadMotor_IsLegReached(QuadLegID_t leg_id, float tolerance)
{
    if (validate_leg_id(leg_id) != 0) {
        return false;
    }

    for (uint8_t joint = 0; joint < QUAD_MOTORS_PER_LEG; joint++) {
        uint8_t motor_id = QUAD_MOTOR_INDEX(leg_id, joint);
        update_motor_status_from_driver(motor_id);

        float error = g_motor_status[motor_id].target_position - g_motor_status[motor_id].position;
        if (fabsf(error) > tolerance) {
            return false;
        }
    }

    return true;
}

/* ============================================================================
 * 安全控制API
 * ============================================================================ */

/**
 * @brief 紧急停止所有电机
 */
void QuadMotor_EmergencyStop(void)
{
    /* 调用底层驱动的紧急停止 */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        uint8_t can_channel = (i < 6) ? 1 : 2;
        uint8_t device_id = i + 1;

        /* 失能电机 */
        motor_enable(can_channel, device_id, 0);
        /* 停止电机 */
        motor_start_stop(can_channel, device_id, 0);
    }

    /* 更新统计 */
    g_api_stats.emergency_stops++;

    /* 设置系统状态为错误 */
    g_system_state = SYS_STATE_ERROR;
}

/**
 * @brief 使能/失能所有电机
 */
void QuadMotor_EnableAll(bool enable)
{
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        QuadMotor_Enable(i, enable);
    }
}

/**
 * @brief 使能/失能单个电机
 */
uint8_t QuadMotor_Enable(uint8_t motor_id, bool enable)
{
    if (validate_motor_id(motor_id) != 0) {
        return 1;  /* 电机ID无效 */
    }

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;

    /* 调用底层驱动 */
    if (enable) {
        motor_enable(can_channel, device_id, 1);
    } else {
        motor_enable(can_channel, device_id, 0);
    }

    /* 更新状态缓存 */
    g_motor_status[motor_id].enabled = enable;

    return 0;  /* 成功 */
}

/**
 * @brief 检查系统是否有错误
 */
bool QuadMotor_HasError(void)
{
    update_all_motor_status();

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        if (g_motor_status[i].error_flags != 0) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 获取错误信息
 */
uint8_t QuadMotor_GetErrors(uint8_t* error_motors, uint8_t max_count)
{
    if (error_motors == NULL || max_count == 0) {
        return 0;
    }

    update_all_motor_status();

    uint8_t error_count = 0;

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS && error_count < max_count; i++) {
        if (g_motor_status[i].error_flags != 0) {
            error_motors[error_count++] = i;
        }
    }

    return error_count;
}

/**
 * @brief 清除所有错误
 */
void QuadMotor_ClearErrors(void)
{
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        uint8_t can_channel = (i < 6) ? 1 : 2;
        uint8_t device_id = i + 1;

        /* 调用底层清除错误 */
        motor_clear_error(can_channel, device_id);

        /* 清除状态缓存 */
        g_motor_status[i].error_flags = 0;
    }

    /* 如果系统处于错误状态，恢复到空闲状态 */
    if (g_system_state == SYS_STATE_ERROR) {
        g_system_state = SYS_STATE_IDLE;
    }
}

/* ============================================================================
 * 参数配置API
 * ============================================================================ */

/**
 * @brief 配置摆动相参数（全局）
 */
void QuadMotor_ConfigSwingParams(SwingParams_t* params)
{
    if (params != NULL) {
        SmartCtrl_ConfigSwingParams(params);
    }
}

/**
 * @brief 配置支撑相参数（单电机）
 */
void QuadMotor_ConfigStanceParams(uint8_t motor_id, StanceParams_t* params)
{
    if (validate_motor_id(motor_id) == 0 && params != NULL) {
        SmartCtrl_ConfigStanceParams(motor_id, params);
    }
}

/**
 * @brief 配置场景识别参数
 */
void QuadMotor_ConfigSceneParams(SceneRecognitionParams_t* params)
{
    if (params != NULL) {
        SmartCtrl_ConfigSceneParams(params);
    }
}

/* ============================================================================
 * 统计和调试API
 * ============================================================================ */

/**
 * @brief 获取系统运行统计
 */
void QuadMotor_GetStats(uint32_t* total_updates,
                        uint32_t* total_canfd_frames,
                        float* optimization_rate)
{
    /* API层统计 */
    if (total_updates != NULL) {
        *total_updates = g_api_stats.total_updates;
    }

    /* 融合层统计 */
    uint32_t total_commands = 0;
    uint32_t total_frames = 0;
    float opt_rate = 0.0f;

    Fusion_GetStats(&total_commands, &total_frames, &opt_rate);

    if (total_canfd_frames != NULL) {
        *total_canfd_frames = total_frames;
    }

    if (optimization_rate != NULL) {
        *optimization_rate = opt_rate;
    }
}

/**
 * @brief 打印系统状态（调试用）
 */
void QuadMotor_PrintStatus(bool detailed)
{
    printf("\n========== Quadruped Motor System Status ==========\n");
    printf("System State: %d\n", g_system_state);
    printf("Hybrid Control: %s\n", g_hybrid_control_enabled ? "Enabled" : "Disabled");
    printf("Total Updates: %u\n", (unsigned int)g_api_stats.total_updates);
    printf("Update Errors: %u\n", (unsigned int)g_api_stats.update_errors);
    printf("Emergency Stops: %u\n", (unsigned int)g_api_stats.emergency_stops);

    /* 获取融合层统计 */
    uint32_t total_commands = 0;
    uint32_t total_frames = 0;
    float opt_rate = 0.0f;
    Fusion_GetStats(&total_commands, &total_frames, &opt_rate);

    printf("Total Commands: %u\n", (unsigned int)total_commands);
    printf("Total CANFD Frames: %u\n", (unsigned int)total_frames);
    printf("Optimization Rate: %.1f%%\n", opt_rate);

    if (detailed) {
        printf("\n--- Motor Status (12 Motors) ---\n");
        update_all_motor_status();

        for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
            QuadMotor_PrintMotorStatus(i);
        }
    }

    printf("==================================================\n\n");
}

/**
 * @brief 打印单个电机状态
 */
void QuadMotor_PrintMotorStatus(uint8_t motor_id)
{
    if (validate_motor_id(motor_id) != 0) {
        return;
    }

    QuadMotorStatus_t* st = &g_motor_status[motor_id];

    printf("Motor[%d]: Pos=%.2f Vel=%.2f Torque=%.2f Temp=%.1f Scene=%s Strategy=%s Error=0x%04X\n",
           st->motor_id,
           st->position,
           st->velocity,
           st->torque,
           st->temperature,
           quad_scene_name(st->scene),
           quad_strategy_name(st->strategy),
           st->error_flags);
}

/**
 * @brief 重置所有统计信息
 */
void QuadMotor_ResetStats(void)
{
    g_api_stats.total_updates = 0;
    g_api_stats.update_errors = 0;
    g_api_stats.emergency_stops = 0;

    Fusion_ResetStats();
    SmartCtrl_ResetStats();
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief 验证电机ID有效性
 */
static uint8_t validate_motor_id(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) {
        return 1;  /* 无效 */
    }
    return 0;  /* 有效 */
}

/**
 * @brief 验证腿部ID有效性
 */
static uint8_t validate_leg_id(QuadLegID_t leg_id)
{
    if (leg_id >= QUAD_NUM_LEGS) {
        return 1;  /* 无效 */
    }
    return 0;  /* 有效 */
}

/**
 * @brief 从底层驱动更新单个电机状态（带超时检测）
 */
static void update_motor_status_from_driver(uint8_t motor_id)
{
    if (validate_motor_id(motor_id) != 0) {
        return;
    }

    uint8_t can_channel = (motor_id < 6) ? 1 : 2;
    uint8_t device_id = motor_id + 1;
    uint32_t current_time = HAL_GetTick();

    MotorHandle* handle = get_motor_handle(can_channel, device_id);
    if (handle != NULL) {
        g_motor_status[motor_id].position = handle->current_position;
        g_motor_status[motor_id].velocity = handle->current_velocity;
        g_motor_status[motor_id].torque = handle->current_torque;
        g_motor_status[motor_id].temperature = handle->temperature;
        g_motor_status[motor_id].voltage = handle->bus_voltage;
        g_motor_status[motor_id].current = handle->current;
        g_motor_status[motor_id].target_position = handle->target_position;
        g_motor_status[motor_id].target_torque = handle->torque_limit;

        /* 错误检测 */
        g_motor_status[motor_id].error_flags = handle->error_flags;

        /* 超时检测：如果状态超过阈值未更新，标记超时错误 */
        uint32_t elapsed;
        if (current_time >= g_motor_status[motor_id].last_update_ms) {
            elapsed = current_time - g_motor_status[motor_id].last_update_ms;
        } else {
            elapsed = (0xFFFFFFFF - g_motor_status[motor_id].last_update_ms) + current_time + 1;
        }

        if (MOTOR_STATUS_TIMEOUT_MS > 0 && elapsed > MOTOR_STATUS_TIMEOUT_MS) {
            g_motor_status[motor_id].error_flags |= MOTOR_ERROR_FLAG_TIMEOUT;
        }
    }

    /* 从智能控制器获取控制状态 */
    g_motor_status[motor_id].scene = SmartCtrl_GetMotorScene(motor_id);
    g_motor_status[motor_id].strategy = SmartCtrl_GetMotorStrategy(motor_id);
    g_motor_status[motor_id].hybrid_state = SmartCtrl_GetHybridState(motor_id);

    /* 更新时间戳 */
    g_motor_status[motor_id].last_update_ms = current_time;
}

/**
 * @brief 更新所有电机状态
 */
static void update_all_motor_status(void)
{
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        update_motor_status_from_driver(i);
    }
}
