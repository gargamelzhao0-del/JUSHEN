/**
 ******************************************************************************
 * @file    power_management.h
 * @brief   电源管理模块 - I2C通信接口
 * @note    与外部电源管理芯片（BMS）通信，监控电池状态
 ******************************************************************************
 */

#ifndef __POWER_MANAGEMENT_H
#define __POWER_MANAGEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 电源管理配置
 * ============================================================================ */

#define POWER_I2C_TIMEOUT_MS        100     /* I2C通信超时（毫秒）*/
#define POWER_VOLTAGE_LOW_THR       22.0f   /* 低电压阈值（V）*/
#define POWER_VOLTAGE_CRITICAL_THR  20.0f   /* 严重低电压阈值（V）*/
#define POWER_TEMP_HIGH_THR         55.0f   /* 高温阈值（°C）*/

/**
 * @brief 电源状态
 */
typedef enum {
    POWER_STATUS_NORMAL,        /* 正常 */
    POWER_STATUS_LOW_VOLTAGE,   /* 低电压 */
    POWER_STATUS_CRITICAL,      /* 严重低电压 */
    POWER_STATUS_OVERHEAT,      /* 过热 */
    POWER_STATUS_CHARGING,      /* 充电中 */
    POWER_STATUS_ERROR          /* 错误 */
} PowerStatus_t;

/**
 * @brief 电源数据结构
 */
typedef struct {
    float    voltage;           /* 电池电压（V）*/
    float    current;           /* 电池电流（A）*/
    float    temperature;       /* 电池温度（°C）*/
    float    capacity;          /* 剩余容量（%）*/
    uint32_t energy_consumed;   /* 已消耗能量（mWh）*/
    PowerStatus_t status;       /* 电源状态 */
    bool     is_charging;       /* 是否在充电 */
    bool     is_connected;      /* BMS是否连接 */
    uint32_t last_update_ms;    /* 最后更新时间 */
} PowerData_t;

/* ============================================================================
 * 公共API
 * ============================================================================ */

/**
 * @brief  初始化电源管理模块
 * @param  i2c_handle: I2C句柄指针（如 &hi2c3）
 * @param  device_addr: BMS设备I2C地址（7位地址）
 * @retval 0=成功, -1=失败
 */
int power_init(void *i2c_handle, uint8_t device_addr);

/**
 * @brief  读取电源数据
 * @param  data: 输出电源数据结构体
 * @retval 0=成功, -1=失败
 */
int power_read_data(PowerData_t *data);

/**
 * @brief  获取电池电压
 * @retval 电压值（V），失败返回-1.0f
 */
float power_get_voltage(void);

/**
 * @brief  获取电池电流
 * @retval 电流值（A），失败返回0.0f
 */
float power_get_current(void);

/**
 * @brief  获取电池温度
 * @retval 温度值（°C），失败返回-999.0f
 */
float power_get_temperature(void);

/**
 * @brief  获取剩余容量百分比
 * @retval 容量百分比（0-100%），失败返回0
 */
float power_get_capacity(void);

/**
 * @brief  获取电源状态
 * @retval 电源状态枚举值
 */
PowerStatus_t power_get_status(void);

/**
 * @brief  检查是否在充电
 * @retval true=充电中, false=未充电
 */
bool power_is_charging(void);

/**
 * @brief  检查BMS是否连接
 * @retval true=已连接, false=未连接
 */
bool power_is_connected(void);

/**
 * @brief  设置低电压告警回调函数
 * @param  callback: 回调函数指针
 * @note   当电压低于阈值时调用
 */
void power_set_low_voltage_callback(void (*callback)(float voltage));

/**
 * @brief  设置过热告警回调函数
 * @param  callback: 回调函数指针
 * @note   当温度高于阈值时调用
 */
void power_set_overheat_callback(void (*callback)(float temperature));

/**
 * @brief  启动电源监控任务（FreeRTOS）
 * @param  update_interval_ms: 更新间隔（毫秒）
 * @retval 0=成功, -1=失败
 */
int power_start_monitor_task(uint32_t update_interval_ms);

/**
 * @brief  停止电源监控任务
 */
void power_stop_monitor_task(void);

/**
 * @brief  打印电源状态（调试用）
 */
void power_print_status(void);

/**
 * @brief  获取电源状态名称字符串
 * @param  status: 电源状态
 * @retval 状态名称字符串
 */
const char* power_get_status_name(PowerStatus_t status);

/**
 * @brief V2.0: BMS寄存器配置结构体
 */
typedef struct {
    uint8_t reg_voltage_h;      /* 电压高字节寄存器地址 */
    uint8_t reg_voltage_l;      /* 电压低字节寄存器地址 */
    uint8_t reg_current_h;      /* 电流高字节寄存器地址 */
    uint8_t reg_current_l;      /* 电流低字节寄存器地址 */
    uint8_t reg_temp;           /* 温度寄存器地址 */
    uint8_t reg_capacity;       /* 容量寄存器地址 */
    uint8_t reg_status;         /* 状态寄存器地址 */
    uint8_t reg_soc;            /* SOC寄存器地址 */
    uint8_t reg_soh;            /* SOH寄存器地址 */
} BMSRegisterConfig_t;

/**
 * @brief V2.0: 配置BMS寄存器地址
 * @param config 寄存器配置结构体指针
 * @retval 0=成功, -1=失败
 * @note 用于适配不同BMS固件的寄存器映射
 */
int power_configure_bms_registers(const BMSRegisterConfig_t* config);

/**
 * @brief V2.0: 获取当前BMS寄存器配置
 * @param config 输出配置结构体指针
 * @retval 0=成功, -1=失败
 */
int power_get_bms_config(BMSRegisterConfig_t* config);

/* ============================================================================
 * 高级功能（可选）
 * ============================================================================ */

/**
 * @brief  估算剩余运行时间
 * @param  average_power_w: 平均功率（W）
 * @retval 剩余运行时间（分钟），失败返回0
 */
uint32_t power_estimate_runtime(float average_power_w);

/**
 * @brief  计算能量消耗
 * @retval 累计消耗能量（Wh）
 */
float power_get_energy_consumed(void);

/**
 * @brief  复位能量计数器
 */
void power_reset_energy_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MANAGEMENT_H */
