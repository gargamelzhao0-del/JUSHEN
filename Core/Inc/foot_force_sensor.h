/**
 ******************************************************************************
 * @file    foot_force_sensor.h
 * @brief   足端力传感器模块（基于ADC采集）
 * @note    用于四足机器狗的足端力检测和地面接触判断
 ******************************************************************************
 */

#ifndef __FOOT_FORCE_SENSOR_H
#define __FOOT_FORCE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "quad_types.h"
#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

/* 类型别名 - 统一使用quad_types中的定义 */
typedef QuadFootID_t FootID_t;

/* ============================================================================
 * 配置参数
 * ============================================================================ */

#define FOOT_FORCE_SENSOR_COUNT     4       /* 足端传感器数量（4足）*/
#define FOOT_FORCE_SAMPLE_RATE_HZ   100     /* 采样率（Hz）*/
#ifndef FOOT_FORCE_FILTER_SIZE
#define FOOT_FORCE_FILTER_SIZE      10      /* 滤波器窗口大小 */
#endif
#define FOOT_FORCE_MAX_FILTER_SIZE  32      /* 最大滤波器窗口大小 */

/* 滤波器类型 */
typedef enum {
    FILTER_TYPE_MOVING_AVERAGE,  /* 移动平均滤波器 */
    FILTER_TYPE_LOW_PASS,        /* 低通滤波器 */
    FILTER_TYPE_MEDIAN,          /* 中值滤波器 */
    FILTER_TYPE_COMBINED         /* 组合滤波器 */
} FilterType_t;

/**
 * @brief 足端状态
 */
typedef enum {
    FOOT_STATE_IN_AIR,      /* 腾空（无接触）*/
    FOOT_STATE_TOUCHING,    /* 轻触地面 */
    FOOT_STATE_SUPPORTING,  /* 支撑（承重）*/
    FOOT_STATE_OVERLOAD     /* 过载 */
} FootState_t;

/**
 * @brief 足端力传感器数据
 */
typedef struct {
    float    force;             /* 当前力值（N）*/
    float    force_filtered;    /* 滤波后的力值（N）*/
    float    force_max;         /* 峰值力（N）*/
    float    force_min;         /* 最小力（N）*/
    FootState_t state;          /* 足端状态 */
    bool     is_contact;        /* 是否接触地面 */
    uint32_t contact_time_ms;   /* 接触时间（毫秒）*/
    uint32_t last_update_ms;    /* 最后更新时间 */
} FootForceData_t;

/**
 * @brief 传感器校准数据
 */
typedef struct {
    float offset;               /* 零点偏移（N）*/
    float scale;                /* 增益系数 */
    uint16_t adc_zero;          /* ADC零点值 */
} FootSensorCalib_t;

/* ============================================================================
 * 公共API - 初始化与配置
 * ============================================================================ */

/**
 * @brief  初始化足端力传感器系统
 * @retval 0=成功, -1=失败
 */
int foot_force_init(void);

/**
 * @brief  设置传感器与ADC通道的映射
 * @param  foot_id: 足端ID
 * @param  adc_channel: ADC通道号
 * @retval 0=成功, -1=失败
 */
int foot_force_set_channel(FootID_t foot_id, uint8_t adc_channel);

/**
 * @brief  校准传感器（零点标定）
 * @param  foot_id: 足端ID
 * @note   在无负载状态下调用，记录零点偏移
 */
void foot_force_calibrate_zero(FootID_t foot_id);

/**
 * @brief  校准所有传感器（一键零点标定）
 * @note   在无负载状态下调用，同时标定所有4个传感器
 */
void foot_force_calibrate_all_zero(void);

/**
 * @brief  保存校准数据到参数系统
 * @retval 0=成功, -1=失败
 * @note   将当前校准数据保存到param_config并写入FLASH
 */
int foot_force_save_calibration(void);

/**
 * @brief  从参数系统加载校准数据
 * @retval 0=成功, -1=失败
 * @note   从param_config读取校准数据并应用
 */
int foot_force_load_calibration(void);

/**
 * @brief  打印校准数据
 * @note   显示当前所有传感器的校准参数
 */
void foot_force_print_calibration(void);

/**
 * @brief  设置传感器增益
 * @param  foot_id: 足端ID
 * @param  scale: 增益系数（N/V）
 */
void foot_force_set_scale(FootID_t foot_id, float scale);

/* ============================================================================
 * 公共API - 数据读取
 * ============================================================================ */

/**
 * @brief  更新传感器数据（在ADC采样回调中调用）
 * @param  foot_id: 足端ID
 * @param  adc_value: ADC原始值
 * @retval 0=成功, -1=失败
 */
int foot_force_update(FootID_t foot_id, uint16_t adc_value);

/**
 * @brief  获取足端力数据
 * @param  foot_id: 足端ID
 * @param  data: 输出数据结构体
 * @retval 0=成功, -1=失败
 */
int foot_force_get_data(FootID_t foot_id, FootForceData_t *data);

/**
 * @brief  获取足端力值
 * @param  foot_id: 足端ID
 * @retval 力值（N），失败返回-1.0f
 */
float foot_force_get_value(FootID_t foot_id);

/**
 * @brief  获取滤波后的力值
 * @param  foot_id: 足端ID
 * @retval 滤波后的力值（N）
 */
float foot_force_get_filtered(FootID_t foot_id);

/**
 * @brief  获取足端状态
 * @param  foot_id: 足端ID
 * @retval 足端状态枚举值
 */
FootState_t foot_force_get_state(FootID_t foot_id);

/**
 * @brief  检查是否接触地面
 * @param  foot_id: 足端ID
 * @retval true=接触, false=腾空
 */
bool foot_force_is_contact(FootID_t foot_id);

/**
 * @brief  获取所有足端的接触状态（位掩码）
 * @retval 接触状态位掩码（bit0=前左, bit1=前右, bit2=后左, bit3=后右）
 */
uint8_t foot_force_get_contact_mask(void);

/**
 * @brief  获取支撑足数量
 * @retval 支撑足数量（0-4）
 */
uint8_t foot_force_get_support_count(void);

/* ============================================================================
 * 公共API - 阈值设置
 * ============================================================================ */

/**
 * @brief  设置接触阈值
 * @param  contact_threshold: 接触判定阈值（N）
 */
void foot_force_set_contact_threshold(float contact_threshold);

/**
 * @brief  设置支撑阈值
 * @param  support_threshold: 支撑判定阈值（N）
 */
void foot_force_set_support_threshold(float support_threshold);

/**
 * @brief  设置过载阈值
 * @param  overload_threshold: 过载判定阈值（N）
 */
void foot_force_set_overload_threshold(float overload_threshold);

/* ============================================================================ 
 * 公共API - 高级功能
 * ============================================================================ */

/**
 * @brief  计算质心（基于足端力分布）
 * @param  com_x: 输出质心X坐标（m）
 * @param  com_y: 输出质心Y坐标（m）
 * @note   需要预先设置足端位置
 */
void foot_force_calc_center_of_mass(float *com_x, float *com_y);

/**
 * @brief  检测步态事件（触地/离地）
 * @param  foot_id: 足端ID
 * @retval 1=触地事件, -1=离地事件, 0=无事件
 */
int foot_force_detect_gait_event(FootID_t foot_id);

/**
 * @brief  重置峰值统计
 * @param  foot_id: 足端ID
 */
void foot_force_reset_stats(FootID_t foot_id);

/**
 * @brief  打印所有足端力数据（调试用）
 */
void foot_force_print_all(void);

/**
 * @brief  设置滤波器类型
 * @param  foot_id: 足端ID
 * @param  filter_type: 滤波器类型
 */
void foot_force_set_filter_type(FootID_t foot_id, FilterType_t filter_type);

/**
 * @brief  设置低通滤波器参数
 * @param  foot_id: 足端ID
 * @param  cutoff_freq: 截止频率（Hz）
 */
void foot_force_set_lowpass_params(FootID_t foot_id, float cutoff_freq);

/**
 * @brief  获取力数据变化率
 * @param  foot_id: 足端ID
 * @retval 力变化率（N/s）
 */
float foot_force_get_rate_of_change(FootID_t foot_id);

/**
 * @brief  批量更新所有足端力数据
 * @param  adc_values: 4个足端的ADC值数组
 */
void foot_force_update_all(uint16_t adc_values[4]);

/**
 * @brief  获取地面粗糙度估计
 * @param  foot_id: 足端ID
 * @retval 粗糙度值（0-1，值越大越粗糙）
 */
float foot_force_estimate_roughness(FootID_t foot_id);

/**
 * @brief  获取足端名称字符串
 * @param  foot_id: 足端ID
 * @retval 足端名称字符串
 */
const char* foot_force_get_name(FootID_t foot_id);

/**
 * @brief  获取足端状态名称字符串
 * @param  state: 足端状态
 * @retval 状态名称字符串
 */
const char* foot_force_get_state_name(FootState_t state);

/* ============================================================================
 * 公共API - 故障检测
 * ============================================================================ */

/**
 * @brief  检查传感器是否有故障
 * @param  foot_id: 足端ID
 * @retval true=有故障, false=正常
 */
bool foot_force_has_fault(FootID_t foot_id);

/**
 * @brief  获取传感器故障信息
 * @param  foot_id: 足端ID
 * @param  fault_count: 故障计数输出（可选）
 * @param  is_stuck: 信号卡死状态输出（可选）
 * @param  out_of_range: 信号超范围状态输出（可选）
 * @retval 0=成功, -1=失败
 */
int foot_force_get_fault_info(FootID_t foot_id, uint32_t *fault_count, bool *is_stuck, bool *out_of_range);

/**
 * @brief  检查传感器是否有效（V10.0新增）
 * @param  foot_id: 足端ID
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval true=有效, false=超时/无效
 */
bool foot_force_is_valid(FootID_t foot_id, uint32_t timeout_ms);

/**
 * @brief  获取传感器有效状态掩码（V10.0新增）
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval 有效状态掩码（bit0=前左, bit1=前右, bit2=后左, bit3=后右）
 */
uint8_t foot_force_get_valid_mask(uint32_t timeout_ms);

/**
 * @brief  检查是否所有传感器都有效（V10.0新增）
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval true=全部有效, false=有传感器超时
 */
bool foot_force_all_valid(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __FOOT_FORCE_SENSOR_H */
