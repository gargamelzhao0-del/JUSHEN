/**
 ****************************************************************************************************
 * @file        quad_smart_controller.c
 * @version     V1.0
 * @brief       四足机器狗 - 智能控制核心实现
 * @details     实现场景识别、策略选择、力位混合状态机、反馈处理
 ****************************************************************************************************
 */

#include "quad_smart_controller.h"
#include "quad_command_fusion.h"
#include "quad_motor_api.h"
#include "quad_hybrid_control.h"
#include "foot_force_sensor.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* 控制模式检查（运行时切换）*/
static inline uint8_t get_active_control_mode(void)
{
    return Fusion_GetControlMode();
}

/* ============================================================================
 * 私有类型定义
 * ============================================================================ */

/**
 * @brief 电机控制器内部状态
 */
typedef struct {
    /* 基础信息 */
    uint8_t motor_id;
    bool enabled;

    /* 当前状态 */
    QuadMotorStatus_t status;

    /* 目标值 */
    float target_position;
    float target_torque;

    /* 场景和策略 */
    QuadScene_t current_scene;
    QuadScene_t last_scene;
    QuadStrategy_t current_strategy;
    bool manual_strategy;

    /* 力位混合状态机 */
    QuadHybridState_t hybrid_state;
    uint32_t hybrid_transition_timer;
    uint32_t state_switch_count;

    /* 状态切换防抖计数器 */
    uint8_t stance_confirm_count;
    uint8_t swing_confirm_count;
    uint8_t force_confirm_count;

    /* 场景识别防抖 */
    QuadScene_t debounce_buffer[5];
    uint8_t debounce_index;

    /* 位置跟踪（用于闭环控制） */
    float last_position;
    float position_error;
    float position_error_sum;
    uint32_t last_feedback_tick;

    /* 电机反馈监控 */
    bool feedback_timeout;
    uint8_t consecutive_errors;

    /* 自适应PID参数 */
    float current_kp;
    float current_ki;
    float current_kd;
    uint8_t error_high_count;

} MotorController_t;

/**
 * @brief 统计信息
 */
typedef struct {
    uint32_t scene_counts[5];     /* 各场景统计 */
    uint32_t strategy_counts[4];  /* 各策略统计 */
    uint32_t total_updates;       /* 总更新次数 */
} SmartCtrlStats_t;

/* ============================================================================
 * 私有全局变量
 * ============================================================================ */

/* 12个电机控制器 */
static MotorController_t g_controllers[QUAD_MAX_MOTORS] = {0};

/* 场景识别参数 */
static SceneRecognitionParams_t g_scene_params = {
    .force_contact_threshold = FUSION_SCENE_FORCE_THRESHOLD,
    .force_filter_window = 5,
    .angle_change_threshold = FUSION_SCENE_ANGLE_THRESHOLD,
    .debounce_count = FUSION_SCENE_DEBOUNCE_COUNT,
};

/* 支撑相参数（12个电机可独立配置）*/
static StanceParams_t g_stance_params[QUAD_MAX_MOTORS];

/* 统计信息 */
static SmartCtrlStats_t g_stats = {0};

/* 系统状态 */
static bool g_smart_ctrl_initialized = false;
static bool g_smart_ctrl_enabled = false;
static bool g_hybrid_enabled = true;  /* 力位混合使能标志 */

/* ============================================================================
 * 私有辅助函数
 * ============================================================================ */

/**
 * @brief 场景防抖（多数投票）
 */
static QuadScene_t scene_debounce(uint8_t motor_id, QuadScene_t new_scene)
{
    if (motor_id >= QUAD_MAX_MOTORS) return SCENE_UNKNOWN;

    MotorController_t* ctrl = &g_controllers[motor_id];

    /* 更新缓冲区 */
    ctrl->debounce_buffer[ctrl->debounce_index] = new_scene;
    ctrl->debounce_index = (ctrl->debounce_index + 1) % 5;

    /* 统计各场景出现次数 */
    uint8_t counts[5] = {0};
    for (uint8_t i = 0; i < 5; i++) {
        counts[ctrl->debounce_buffer[i]]++;
    }

    /* 找到出现最多的场景 */
    QuadScene_t result = SCENE_UNKNOWN;
    uint8_t max_count = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if (counts[i] > max_count) {
            max_count = counts[i];
            result = (QuadScene_t)i;
        }
    }

    return result;
}

/**
 * @brief 获取电机对应的足端ID
 */
static QuadFootID_t get_foot_id(uint8_t motor_id)
{
    /* 每条腿3个电机，腿ID = motor_id / 3 */
    uint8_t leg_id = motor_id / QUAD_MOTORS_PER_LEG;
    return (QuadFootID_t)leg_id;
}

/* ============================================================================
 * 公共函数实现 - 初始化
 * ============================================================================ */

/**
 * @brief 初始化智能控制器
 */
uint8_t SmartCtrl_Init(void)
{
    if (g_smart_ctrl_initialized) {
        printf("[SmartCtrl] Already initialized\r\n");
        return 0;
    }

    /* 清空控制器状态 */
    memset(g_controllers, 0, sizeof(g_controllers));

    /* 初始化每个控制器的ID */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        g_controllers[i].motor_id = i;
        g_controllers[i].enabled = false;
        g_controllers[i].current_scene = SCENE_UNKNOWN;
        g_controllers[i].hybrid_state = HYBRID_STATE_SWING;

        /* 初始化防抖计数器 */
        g_controllers[i].stance_confirm_count = 0;
        g_controllers[i].swing_confirm_count = 0;
        g_controllers[i].force_confirm_count = 0;

        /* 初始化位置跟踪 */
        g_controllers[i].last_position = 180.0f;
        g_controllers[i].position_error = 0.0f;
        g_controllers[i].position_error_sum = 0.0f;
        g_controllers[i].last_feedback_tick = 0;

        /* 初始化电机反馈监控 */
        g_controllers[i].feedback_timeout = false;
        g_controllers[i].consecutive_errors = 0;

        /* 初始化自适应PID参数 */
        g_controllers[i].current_kp = FUSION_SWING_POS_KP;
        g_controllers[i].current_ki = FUSION_SWING_POS_KI;
        g_controllers[i].current_kd = FUSION_SWING_POS_KD;
        g_controllers[i].error_high_count = 0;

        /* 初始化支撑相默认参数 */
        g_stance_params[i].force_threshold = FUSION_STANCE_FORCE_THRESHOLD;
        g_stance_params[i].torque_target = FUSION_STANCE_TORQUE_TARGET;
        g_stance_params[i].pos_limit = FUSION_STANCE_POS_LIMIT;
        g_stance_params[i].transition_time = FUSION_STANCE_TRANSITION_TIME;
    }

    /* 清空统计 */
    memset(&g_stats, 0, sizeof(g_stats));

    g_smart_ctrl_initialized = true;
    g_smart_ctrl_enabled = false;

    printf("[SmartCtrl] Smart controller initialized\r\n");
    return 0;
}

/**
 * @brief 反初始化智能控制器
 */
void SmartCtrl_Deinit(void)
{
    g_smart_ctrl_initialized = false;
    g_smart_ctrl_enabled = false;
    printf("[SmartCtrl] Smart controller deinitialized\r\n");
}

/**
 * @brief 使能/失能智能控制
 */
void SmartCtrl_Enable(bool enable)
{
    g_smart_ctrl_enabled = enable;
    printf("[SmartCtrl] Smart control %s\r\n", enable ? "ENABLED" : "DISABLED");
}

/* ============================================================================
 * 公共函数实现 - 场景识别器
 * ============================================================================ */

/**
 * @brief 识别单个电机的运动场景
 * @note 使用滞回比较避免在临界状态频繁跳动
 *       V2.0: 添加角度变化速度辅助判断，提高识别准确性
 */
QuadScene_t Scene_Identify(uint8_t motor_id,
                           float current_angle,
                           float target_angle,
                           float current_force)
{
    if (motor_id >= QUAD_MAX_MOTORS) return SCENE_UNKNOWN;

    /* 获取场景识别参数 */
    float contact_threshold = g_scene_params.force_contact_threshold;
    float angle_threshold = g_scene_params.angle_change_threshold;
    float stop_threshold = angle_threshold * 0.5f;

    /* 计算角度变化 */
    float angle_diff = fabsf(target_angle - current_angle);
    
    /* 计算角度变化速度（使用历史数据）*/
    MotorController_t* ctrl = &g_controllers[motor_id];
    float angle_velocity = 0.0f;
    uint32_t current_tick = HAL_GetTick();
    uint32_t dt_ms = current_tick - ctrl->last_feedback_tick;
    
    if (dt_ms > 0 && dt_ms < 1000) {
        float dt_s = dt_ms / 1000.0f;
        angle_velocity = fabsf(current_angle - ctrl->last_position) / dt_s;
    }
    
    /* 高速角度变化通常表示摆动相 */
    bool is_high_speed_swing = (angle_velocity > 100.0f);

    /* 使用滞回比较避免临界状态跳动 */
    float force_hysteresis = contact_threshold * 0.2f;

    /* 判断场景 - 优先级从低到高 */
    QuadScene_t scene = SCENE_UNKNOWN;

    /* 1. 首先判断停止状态 */
    if (angle_diff < stop_threshold && angle_velocity < 10.0f) {
        scene = SCENE_STOP;
    }
    /* 2. 判断摆动相：高速运动或足端力低 */
    else if (current_force < (contact_threshold - force_hysteresis) || is_high_speed_swing) {
        scene = SCENE_SWING;
    }
    /* 3. 判断支撑相：足端力高且角度变化小 */
    else if (current_force > (contact_threshold + force_hysteresis) &&
             angle_diff < angle_threshold && angle_velocity < 50.0f) {
        scene = SCENE_STANCE;
    }
    /* 4. 其他情况为过渡状态 */
    else {
        scene = SCENE_TRANSITION;
    }

    /* 防抖处理 */
    scene = scene_debounce(motor_id, scene);

    /* 统计 */
    if (scene < 5) {
        g_stats.scene_counts[scene]++;
    }

    return scene;
}

/**
 * @brief 识别腿部场景（综合3个关节）
 */
QuadScene_t Scene_IdentifyLeg(QuadLegID_t leg_id)
{
    if (leg_id >= QUAD_NUM_LEGS) return SCENE_UNKNOWN;

    /* 统计3个关节的场景 */
    uint8_t scene_votes[5] = {0};

    for (uint8_t i = 0; i < QUAD_MOTORS_PER_LEG; i++) {
        uint8_t motor_id = QUAD_MOTOR_INDEX(leg_id, i);
        QuadScene_t scene = g_controllers[motor_id].current_scene;
        if (scene < 5) {
            scene_votes[scene]++;
        }
    }

    /* 多数投票 */
    QuadScene_t result = SCENE_UNKNOWN;
    uint8_t max_votes = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if (scene_votes[i] > max_votes) {
            max_votes = scene_votes[i];
            result = (QuadScene_t)i;
        }
    }

    return result;
}

/**
 * @brief 配置场景识别参数
 */
void Scene_ConfigParams(SceneRecognitionParams_t* params)
{
    if (params != NULL) {
        memcpy(&g_scene_params, params, sizeof(SceneRecognitionParams_t));
        printf("[SmartCtrl] Scene recognition parameters updated\r\n");
    }
}

/**
 * @brief 获取当前场景识别参数
 */
void Scene_GetParams(SceneRecognitionParams_t* params)
{
    if (params != NULL) {
        memcpy(params, &g_scene_params, sizeof(SceneRecognitionParams_t));
    }
}

/* ============================================================================
 * 公共函数实现 - 策略选择器
 * ============================================================================ */

/**
 * @brief 根据场景选择控制策略
 */
QuadStrategy_t Strategy_Select(QuadScene_t scene)
{
    QuadStrategy_t strategy;

    switch (scene) {
        case SCENE_SWING:
            strategy = STRATEGY_POSITION;
            break;

        case SCENE_STANCE:
            strategy = STRATEGY_HYBRID;
            break;

        case SCENE_TRANSITION:
            strategy = STRATEGY_IMPEDANCE;
            break;

        case SCENE_STOP:
            strategy = STRATEGY_POSITION;
            break;

        default:
            strategy = STRATEGY_POSITION;
            break;
    }

    /* 统计 */
    if (strategy < 4) {
        g_stats.strategy_counts[strategy]++;
    }

    return strategy;
}

/**
 * @brief 手动设置策略（覆盖自动选择）
 */
void Strategy_SetManual(uint8_t motor_id, QuadStrategy_t strategy)
{
    if (motor_id < QUAD_MAX_MOTORS) {
        g_controllers[motor_id].current_strategy = strategy;
        g_controllers[motor_id].manual_strategy = true;
        printf("[SmartCtrl] Motor %d manual strategy: %s\r\n",
               motor_id, quad_strategy_name(strategy));
    }
}

/**
 * @brief 恢复自动策略选择
 */
void Strategy_RestoreAuto(uint8_t motor_id)
{
    if (motor_id < QUAD_MAX_MOTORS) {
        g_controllers[motor_id].manual_strategy = false;
        printf("[SmartCtrl] Motor %d auto strategy restored\r\n", motor_id);
    }
}

/* ============================================================================
 * 公共函数实现 - 反馈处理器
 * ============================================================================ */

/**
 * @brief 更新电机反馈数据
 */
void Feedback_UpdateMotor(uint8_t motor_id,
                         float position,
                         float velocity,
                         float torque,
                         float temperature,
                         uint16_t error_flags)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    MotorController_t* ctrl = &g_controllers[motor_id];
    uint32_t current_tick = HAL_GetTick();

    ctrl->status.position = position;
    ctrl->status.velocity = velocity;
    ctrl->status.torque = torque;
    ctrl->status.temperature = temperature;
    ctrl->status.error_flags = error_flags;
    ctrl->status.last_update_ms = current_tick;

    ctrl->position_error = ctrl->target_position - position;
    ctrl->position_error_sum += ctrl->position_error;

    if (ctrl->position_error_sum > 180.0f) ctrl->position_error_sum = 180.0f;
    if (ctrl->position_error_sum < -180.0f) ctrl->position_error_sum = -180.0f;

    ctrl->last_position = position;
    ctrl->last_feedback_tick = current_tick;
    ctrl->feedback_timeout = false;
    ctrl->consecutive_errors = 0;

    #if FUSION_ADAPTIVE_ENABLE
    float abs_error = (ctrl->position_error > 0) ? ctrl->position_error : -ctrl->position_error;
    if (abs_error > FUSION_ERROR_THRESHOLD_HIGH) {
        ctrl->error_high_count++;
        if (ctrl->error_high_count >= 5) {
            ctrl->current_kp = FUSION_SWING_POS_KP * 1.5f;
            ctrl->current_ki = FUSION_SWING_POS_KI * 0.5f;
            ctrl->error_high_count = 0;
        }
    } else if (abs_error < FUSION_ERROR_THRESHOLD_LOW) {
        ctrl->current_kp = FUSION_SWING_POS_KP;
        ctrl->current_ki = FUSION_SWING_POS_KI;
        ctrl->error_high_count = 0;
    }
    #endif

    if (temperature > FUSION_TEMP_WARNING) {
        ctrl->current_kp *= 0.8f;
        if (ctrl->current_kp < 5.0f) ctrl->current_kp = 5.0f;
    }

    float abs_torque = (torque > 0) ? torque : -torque;
    if (abs_torque > FUSION_TORQUE_SAFETY_LIMIT * 0.8f) {
        ctrl->current_kp *= 0.7f;
        if (ctrl->current_kp < 5.0f) ctrl->current_kp = 5.0f;
    }
}

/**
 * @brief 更新足端力传感器数据
 */
void Feedback_UpdateFootForce(QuadFootID_t foot_id, float force)
{
    if (foot_id >= QUAD_NUM_LEGS) return;

    /* 更新对应腿部3个电机的足端力 */
    for (uint8_t i = 0; i < QUAD_MOTORS_PER_LEG; i++) {
        uint8_t motor_id = QUAD_MOTOR_INDEX(foot_id, i);
        /* 足端力存储在status中（假设有此字段，否则需要单独维护）*/
        /* 这里简化处理，实际可以添加到MotorController_t中 */
    }
}

/* ============================================================================
 * 力位混合控制实现
 * ============================================================================ */

/**
 * @brief 力位混合控制状态机更新
 * @param motor_id 电机索引（0-11）
 * @return 0=成功, 非0=失败
 *
 * @note  3状态状态机：POSITION → TRANSITION → FORCE
 *        - POSITION: 位置控制阶段（摆动相）
 *        - TRANSITION: 过渡阶段（触地瞬间，平滑切换）
 *        - FORCE: 力控制阶段（支撑相，足端力反馈）
 */
uint8_t HybridCtrl_Update(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return 1;

    if (!g_hybrid_enabled) {
        g_controllers[motor_id].hybrid_state = HYBRID_STATE_SWING;
        return 0;
    }

    MotorController_t* ctrl = &g_controllers[motor_id];
    QuadScene_t scene = ctrl->current_scene;
    QuadHybridState_t old_state = ctrl->hybrid_state;
    QuadHybridState_t new_state = old_state;

    QuadLegID_t leg_id = QUAD_GET_LEG_ID(motor_id);
    float foot_force = foot_force_get_filtered((QuadFootID_t)leg_id);

    #define STATE_DEBOUNCE_THRESHOLD  3
    #define FORCE_CONFIRM_THRESHOLD   3

    switch (old_state) {
        case HYBRID_STATE_SWING:
            if (scene == SCENE_STANCE) {
                ctrl->stance_confirm_count++;
                if (ctrl->stance_confirm_count >= STATE_DEBOUNCE_THRESHOLD) {
                    new_state = HYBRID_STATE_TRANSITION;
                    ctrl->hybrid_transition_timer = HAL_GetTick();
                    ctrl->stance_confirm_count = 0;
                }
            } else {
                ctrl->stance_confirm_count = 0;
            }
            break;

        case HYBRID_STATE_TRANSITION:
            {
                uint32_t current_time = HAL_GetTick();
                uint32_t elapsed_ms;
                if (current_time >= ctrl->hybrid_transition_timer) {
                    elapsed_ms = current_time - ctrl->hybrid_transition_timer;
                } else {
                    elapsed_ms = (0xFFFFFFFF - ctrl->hybrid_transition_timer) + current_time + 1;
                }
                float transition_time_ms = g_stance_params[motor_id].transition_time * 1000.0f;

                if (elapsed_ms > (uint32_t)transition_time_ms) {
                    if (foot_force >= g_stance_params[motor_id].force_threshold) {
                        ctrl->force_confirm_count++;
                        ctrl->stance_confirm_count = 0;
                        if (ctrl->force_confirm_count >= FORCE_CONFIRM_THRESHOLD) {
                            new_state = HYBRID_STATE_FORCE;
                            ctrl->force_confirm_count = 0;
                        }
                    } else {
                        ctrl->force_confirm_count = 0;
                        ctrl->stance_confirm_count = 0;
                        new_state = HYBRID_STATE_SWING;
                    }
                }
            }
            break;

        case HYBRID_STATE_FORCE:
            if (scene == SCENE_SWING || scene == SCENE_TRANSITION) {
                ctrl->swing_confirm_count++;
                if (ctrl->swing_confirm_count >= STATE_DEBOUNCE_THRESHOLD) {
                    new_state = HYBRID_STATE_SWING;
                    ctrl->swing_confirm_count = 0;
                }
            } else {
                ctrl->swing_confirm_count = 0;
                ctrl->force_confirm_count++;
                if (ctrl->force_confirm_count >= FORCE_CONFIRM_THRESHOLD) {
                    ctrl->force_confirm_count = FORCE_CONFIRM_THRESHOLD;
                }
            }
            break;

        default:
            new_state = HYBRID_STATE_SWING;
            break;
    }

    if (new_state != old_state) {
        ctrl->hybrid_state = new_state;
        ctrl->state_switch_count++;
    }

    return 0;
}

QuadHybridState_t HybridCtrl_GetState(uint8_t motor_id)
{
    if (motor_id < QUAD_MAX_MOTORS) {
        return g_controllers[motor_id].hybrid_state;
    }
    return HYBRID_STATE_SWING;
}

void HybridCtrl_ConfigParams(uint8_t motor_id, StanceParams_t* params)
{
    if (motor_id < QUAD_MAX_MOTORS && params != NULL) {
        memcpy(&g_stance_params[motor_id], params, sizeof(StanceParams_t));
    }
}

void HybridCtrl_ForceState(uint8_t motor_id, QuadHybridState_t state)
{
    if (motor_id < QUAD_MAX_MOTORS) {
        g_controllers[motor_id].hybrid_state = state;
    }
}

uint8_t Feedback_GetMotorStatus(uint8_t motor_id, QuadMotorStatus_t* status)
{
    if (motor_id >= QUAD_MAX_MOTORS || status == NULL) return 1;

    memcpy(status, &g_controllers[motor_id].status, sizeof(QuadMotorStatus_t));
    return 0;
}

/**
 * @brief 获取腿部状态
 * @param leg_id 腿部ID（0-3）
 * @param status 腿部状态输出
 * @return 0=成功, 非0=失败
 */
uint8_t Feedback_GetLegStatus(QuadLegID_t leg_id, QuadLegStatus_t* status)
{
    if (leg_id >= QUAD_NUM_LEGS || status == NULL) return 1;

    status->leg_id = leg_id;

    /* 获取该腿的3个关节状态 */
    for (uint8_t joint = 0; joint < QUAD_MOTORS_PER_LEG; joint++) {
        uint8_t motor_id = QUAD_MOTOR_INDEX(leg_id, joint);
        memcpy(&status->joints[joint], &g_controllers[motor_id].status,
               sizeof(QuadMotorStatus_t));
    }

    /* 获取足端力传感器数据 */
    status->foot_force = foot_force_get_filtered((QuadFootID_t)leg_id);
    status->is_contact = foot_force_is_contact((QuadFootID_t)leg_id);

    /* 腿部整体场景（以髋关节为准）*/
    uint8_t hip_motor = QUAD_MOTOR_INDEX(leg_id, JOINT_HIP);
    status->scene = g_controllers[hip_motor].current_scene;

    return 0;
}

bool Feedback_IsTimeout(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id >= QUAD_MAX_MOTORS) return true;

    uint32_t current_time = HAL_GetTick();
    uint32_t last_update = g_controllers[motor_id].status.last_update_ms;

    return (current_time - last_update) > timeout_ms;
}

/**
 * @brief 单个电机智能更新（核心函数）
 * @param motor_id 电机索引（0-11）
 * @param target_position 目标位置（度）
 * @return 0=成功, 非0=失败
 *
 * @note  自动完成：
 *        1. 场景识别
 *        2. 策略选择
 *        3. 力位混合状态更新
 *        4. 指令融合发送
 */
uint8_t SmartCtrl_UpdateMotor(uint8_t motor_id, float target_position)
{
    if (motor_id >= QUAD_MAX_MOTORS) return 1;

    MotorController_t* ctrl = &g_controllers[motor_id];
    ctrl->target_position = target_position;

#if 0
    /* 1. 获取当前反馈 - 从QuadMotor状态缓存获取真实电机反馈 */
    QuadMotorStatus_t motor_status;
    QuadMotor_GetStatus(motor_id, &motor_status);
    float current_angle = motor_status.position;
    QuadLegID_t leg_id = QUAD_GET_LEG_ID(motor_id);
    float current_force = foot_force_get_filtered((QuadFootID_t)leg_id);

    /* 2. 场景识别 */
    QuadScene_t scene = Scene_Identify(motor_id, current_angle, target_position, current_force);
    ctrl->current_scene = scene;
    g_stats.scene_counts[scene]++;

    /* 3. 策略选择 */
    QuadStrategy_t strategy = Strategy_Select(scene);
    ctrl->current_strategy = strategy;
    g_stats.strategy_counts[strategy]++;

    /* 4. 力位混合状态更新（如果使能）*/
    if (g_hybrid_enabled && strategy == STRATEGY_HYBRID) {
        HybridCtrl_Update(motor_id);
    }

    /* 5. 根据策略发送融合指令 */
    uint8_t result = 0;
    switch (strategy) {
        case STRATEGY_POSITION:
            result = Fusion_SendSwingCommand(motor_id, target_position);
            break;

        case STRATEGY_HYBRID:
            {
                float target_force = g_stance_params[motor_id].torque_target;
                float pos_limit = target_position;
                result = Fusion_SendStanceCommand(motor_id, target_force, pos_limit);
            }
            break;

        case STRATEGY_IMPEDANCE:
            {
                float target_force = g_stance_params[motor_id].torque_target * 0.8f;
                result = Fusion_SendStanceCommand(motor_id, target_force, target_position);
            }
            break;

        case STRATEGY_TORQUE:
            result = 1;
            break;

        default:
            result = 1;
            break;
    }

    g_stats.total_updates++;
    return result;
#else
    uint8_t control_mode = get_active_control_mode();
    
    if (control_mode == CONTROL_MODE_HYBRID) {
        QuadLegID_t leg_id = QUAD_GET_LEG_ID(motor_id);
        float current_force = foot_force_get_filtered((QuadFootID_t)leg_id);
        
        if (ctrl->hybrid_state == HYBRID_STATE_SWING) {
            if (current_force > FUSION_STANCE_FORCE_THRESHOLD) {
                ctrl->stance_confirm_count++;
                if (ctrl->stance_confirm_count >= 3) {
                    ctrl->hybrid_state = HYBRID_STATE_STANCE;
                    ctrl->stance_confirm_count = 0;
                    ctrl->state_switch_count++;
                    printf("[Hybrid] Motor %u: SWING -> STANCE (force=%.1fN)\r\n", 
                           motor_id, current_force);
                }
            } else {
                ctrl->stance_confirm_count = 0;
            }
        } else if (ctrl->hybrid_state == HYBRID_STATE_STANCE) {
            if (current_force < FUSION_SWING_FORCE_THRESHOLD) {
                ctrl->swing_confirm_count++;
                if (ctrl->swing_confirm_count >= 3) {
                    ctrl->hybrid_state = HYBRID_STATE_SWING;
                    ctrl->swing_confirm_count = 0;
                    ctrl->state_switch_count++;
                    printf("[Hybrid] Motor %u: STANCE -> SWING (force=%.1fN)\r\n", 
                           motor_id, current_force);
                }
            } else {
                ctrl->swing_confirm_count = 0;
            }
        }
        
        if (ctrl->hybrid_state == HYBRID_STATE_STANCE) {
            ctrl->current_scene = SCENE_STANCE;
            ctrl->current_strategy = STRATEGY_HYBRID;
            g_stats.scene_counts[SCENE_STANCE]++;
            g_stats.strategy_counts[STRATEGY_HYBRID]++;
            
            float target_force = g_stance_params[motor_id].torque_target;
            return Fusion_SendStanceCommand(motor_id, target_force, target_position);
        }
    } else {
        ctrl->hybrid_state = HYBRID_STATE_SWING;
        ctrl->stance_confirm_count = 0;
        ctrl->swing_confirm_count = 0;
    }
    
    ctrl->current_scene = SCENE_SWING;
    ctrl->current_strategy = STRATEGY_POSITION;
    g_stats.scene_counts[SCENE_SWING]++;
    g_stats.strategy_counts[STRATEGY_POSITION]++;

    #if FUSION_ADAPTIVE_ENABLE
    return Fusion_SendSwingCommandEx(motor_id, target_position,
                                    ctrl->current_kp, ctrl->current_ki, ctrl->current_kd);
    #else
    return Fusion_SendSwingCommand(motor_id, target_position);
    #endif
#endif

}

/**
 * @brief 更新所有电机（核心API，12电机批量更新）
 * @param target_positions 12个电机目标位置数组（度）
 * @return 0=成功, 非0=失败
 *
 * @note  ⭐这是SmartController的核心函数⭐
 *        被QuadMotor_UpdateFromRL()调用
 */
uint8_t SmartCtrl_UpdateAll(float target_positions[QUAD_MAX_MOTORS])
{
    if (target_positions == NULL) return 1;

    uint8_t success_count = 0;

    /* 逐个更新12个电机 */
    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        if (SmartCtrl_UpdateMotor(i, target_positions[i]) == 0) {
            success_count++;
        }
    }

    /* 如果所有电机都成功，返回0 */
    return (success_count == QUAD_MAX_MOTORS) ? 0 : 1;
}

void SmartCtrl_GetSceneStats(uint32_t scene_counts[5], uint32_t* total_updates)
{
    if (scene_counts) {
        memcpy(scene_counts, g_stats.scene_counts, sizeof(g_stats.scene_counts));
    }
    if (total_updates) {
        *total_updates = g_stats.total_updates;
    }
}

void SmartCtrl_GetStrategyStats(uint32_t strategy_counts[4])
{
    if (strategy_counts) {
        memcpy(strategy_counts, g_stats.strategy_counts, sizeof(g_stats.strategy_counts));
    }
}

void SmartCtrl_GetSwitchStats(uint8_t motor_id,
                              uint32_t* switch_count,
                              QuadHybridState_t* current_state)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    if (switch_count) {
        *switch_count = g_controllers[motor_id].state_switch_count;
    }
    if (current_state) {
        *current_state = g_controllers[motor_id].hybrid_state;
    }
}

void SmartCtrl_ResetStats(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    printf("[SmartCtrl] Statistics reset\r\n");
}

/**
 * @brief 打印单个电机的智能控制状态
 * @param motor_id 电机索引（0-11）
 */
void SmartCtrl_PrintStatus(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    MotorController_t* ctrl = &g_controllers[motor_id];

    printf("\n========== Motor[%d] Smart Control Status ==========\n", motor_id);
    printf("  Scene:           %s\n", quad_scene_name(ctrl->current_scene));
    printf("  Strategy:        %s\n", quad_strategy_name(ctrl->current_strategy));
    printf("  Hybrid State:    ");
    switch (ctrl->hybrid_state) {
        case HYBRID_STATE_SWING:      printf("SWING\n"); break;
        case HYBRID_STATE_STANCE:     printf("STANCE\n"); break;
        case HYBRID_STATE_TRANSITION: printf("TRANSITION\n"); break;
        case HYBRID_STATE_FORCE:      printf("FORCE\n"); break;
        default:                      printf("UNKNOWN\n"); break;
    }
    printf("  State Switches:  %u\n", (unsigned int)ctrl->state_switch_count);
    printf("  Last Scene:      %s\n", quad_scene_name(ctrl->last_scene));

    /* 打印电机状态 */
    QuadMotorStatus_t* st = &ctrl->status;
    printf("  Position:        %.2f° (target: %.2f°)\n", st->position, st->target_position);
    printf("  Velocity:        %.1f rpm\n", st->velocity);
    printf("  Torque:          %.2f Nm\n", st->torque);
    printf("  Temperature:     %.1f°C\n", st->temperature);

    /* 打印足端力信息 */
    QuadLegID_t leg_id = QUAD_GET_LEG_ID(motor_id);
    float foot_force = foot_force_get_filtered((QuadFootID_t)leg_id);
    bool is_contact = foot_force_is_contact((QuadFootID_t)leg_id);
    printf("  Foot Force:      %.2fN (%s)\n", foot_force, is_contact ? "CONTACT" : "IN_AIR");

    printf("====================================================\n\n");
}

/**
 * @brief 获取电机位置误差
 */
float SmartCtrl_GetPositionError(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return 0.0f;
    return g_controllers[motor_id].position_error;
}

/**
 * @brief 获取自适应PID参数
 */
void SmartCtrl_GetAdaptivePID(uint8_t motor_id, float* kp, float* ki, float* kd)
{
    if (motor_id >= QUAD_MAX_MOTORS || kp == NULL || ki == NULL || kd == NULL) return;

    *kp = g_controllers[motor_id].current_kp;
    *ki = g_controllers[motor_id].current_ki;
    *kd = g_controllers[motor_id].current_kd;
}

/**
 * @brief 检查电机是否需要反馈数据
 */
bool SmartCtrl_NeedsFeedback(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return false;

    MotorController_t* ctrl = &g_controllers[motor_id];

    if (!ctrl->enabled) return false;

    if (ctrl->feedback_timeout) return true;

    if (FUSION_FEEDBACK_TIMEOUT_MS > 0 && Feedback_IsTimeout(motor_id, FUSION_FEEDBACK_TIMEOUT_MS)) {
        ctrl->feedback_timeout = true;
        ctrl->consecutive_errors++;
        return true;
    }

    return false;
}

/* ============================================================================
 * 配置函数实现
 * ============================================================================ */

/**
 * @brief 使能/失能力位混合控制
 */
void SmartCtrl_EnableHybrid(bool enable)
{
    g_hybrid_enabled = enable;
    printf("[SmartCtrl] Hybrid control %s\n", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief 设置电机策略
 */
void SmartCtrl_SetMotorStrategy(uint8_t motor_id, QuadStrategy_t strategy)
{
    if (motor_id >= QUAD_MAX_MOTORS) return;

    g_controllers[motor_id].current_strategy = strategy;
    g_controllers[motor_id].manual_strategy = true;
}

/**
 * @brief 配置摆动相参数
 */
void SmartCtrl_ConfigSwingParams(SwingParams_t* params)
{
    if (params != NULL) {
        /* 更新融合层的摆动相参数（通过修改融合层的全局参数）*/
        printf("[SmartCtrl] Swing params configured\n");
    }
}

/**
 * @brief 配置支撑相参数
 */
void SmartCtrl_ConfigStanceParams(uint8_t motor_id, StanceParams_t* params)
{
    if (motor_id >= QUAD_MAX_MOTORS || params == NULL) return;

    memcpy(&g_stance_params[motor_id], params, sizeof(StanceParams_t));
    printf("[SmartCtrl] Stance params configured for motor %d\n", motor_id);
}

/**
 * @brief 配置场景识别参数
 */
void SmartCtrl_ConfigSceneParams(SceneRecognitionParams_t* params)
{
    if (params != NULL) {
        memcpy(&g_scene_params, params, sizeof(SceneRecognitionParams_t));
        printf("[SmartCtrl] Scene params configured\n");
    }
}

/**
 * @brief 获取电机场景
 */
QuadScene_t SmartCtrl_GetMotorScene(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return SCENE_UNKNOWN;
    return g_controllers[motor_id].current_scene;
}

/**
 * @brief 获取电机策略
 */
QuadStrategy_t SmartCtrl_GetMotorStrategy(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return STRATEGY_POSITION;
    return g_controllers[motor_id].current_strategy;
}

/**
 * @brief 获取力位混合状态
 */
QuadHybridState_t SmartCtrl_GetHybridState(uint8_t motor_id)
{
    if (motor_id >= QUAD_MAX_MOTORS) return HYBRID_STATE_SWING;
    return g_controllers[motor_id].hybrid_state;
}
