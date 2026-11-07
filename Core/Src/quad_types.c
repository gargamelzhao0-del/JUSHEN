/**
 ****************************************************************************************************
 * @file        quad_types.c
 * @version     V1.0
 * @brief       四足机器狗电机控制系统 - 类型辅助函数实现
 * @details     实现类型转换和字符串输出等辅助函数
 ****************************************************************************************************
 */

#include "quad_types.h"
#include <stddef.h>

/* ============================================================================
 * 私有常量
 * ============================================================================ */

/* 场景名称字符串数组 */
static const char* const scene_names[] = {
    "SWING",        /* SCENE_SWING */
    "STANCE",       /* SCENE_STANCE */
    "TRANSITION",   /* SCENE_TRANSITION */
    "STOP",         /* SCENE_STOP */
    "UNKNOWN",      /* SCENE_UNKNOWN */
};

/* 策略名称字符串数组 */
static const char* const strategy_names[] = {
    "POSITION",     /* STRATEGY_POSITION */
    "HYBRID",       /* STRATEGY_HYBRID */
    "IMPEDANCE",    /* STRATEGY_IMPEDANCE */
    "TORQUE",       /* STRATEGY_TORQUE */
};

/* 腿部名称字符串数组 */
static const char* const leg_names[] = {
    "FRONT_LEFT",   /* LEG_FRONT_LEFT */
    "FRONT_RIGHT",  /* LEG_FRONT_RIGHT */
    "REAR_LEFT",    /* LEG_REAR_LEFT */
    "REAR_RIGHT",   /* LEG_REAR_RIGHT */
};

/* 关节名称字符串数组 */
static const char* const joint_names[] = {
    "HIP",          /* JOINT_HIP */
    "THIGH",        /* JOINT_THIGH */
    "CALF",         /* JOINT_CALF */
};

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief 获取场景名称字符串
 */
const char* quad_scene_name(QuadScene_t scene)
{
    if (scene < sizeof(scene_names) / sizeof(scene_names[0])) {
        return scene_names[scene];
    }
    return "INVALID";
}

/**
 * @brief 获取策略名称字符串
 */
const char* quad_strategy_name(QuadStrategy_t strategy)
{
    if (strategy < sizeof(strategy_names) / sizeof(strategy_names[0])) {
        return strategy_names[strategy];
    }
    return "INVALID";
}

/**
 * @brief 获取腿部名称字符串
 */
const char* quad_leg_name(QuadLegID_t leg_id)
{
    if (leg_id < sizeof(leg_names) / sizeof(leg_names[0])) {
        return leg_names[leg_id];
    }
    return "INVALID";
}

/**
 * @brief 获取关节名称字符串
 */
const char* quad_joint_name(QuadJointID_t joint_id)
{
    if (joint_id < sizeof(joint_names) / sizeof(joint_names[0])) {
        return joint_names[joint_id];
    }
    return "INVALID";
}
