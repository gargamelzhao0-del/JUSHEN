#ifndef SIGNAL_ACQUISITION_H
#define SIGNAL_ACQUISITION_H
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_dma.h"

/* 信号类型枚举 */
typedef enum {
    SIGNAL_TYPE_VOLTAGE = 0,      // 电压信号
    SIGNAL_TYPE_CURRENT = 1,      // 电流信号
    SIGNAL_TYPE_TEMPERATURE = 2,  // 温度信号
    SIGNAL_TYPE_PRESSURE = 3,     // 压力信号
    SIGNAL_TYPE_POSITION = 4,     // 位置信号
    SIGNAL_TYPE_VELOCITY = 5,     // 速度信号
    SIGNAL_TYPE_ACCELERATION = 6, // 加速度信号
    SIGNAL_TYPE_CUSTOM = 7        // 自定义信号
} SignalType;

/* 信号处理算法 */
typedef enum {
    FILTER_NONE = 0,              // 无滤波
    FILTER_MOVING_AVERAGE = 1,    // 移动平均滤波
    FILTER_MEDIAN = 2,            // 中值滤波
    FILTER_KALMAN = 3,            // 卡尔曼滤波
    FILTER_LOWPASS = 4            // 低通滤波
} FilterAlgorithm;

/* 信号通道配置 */
typedef struct {
    uint8_t channel_id;           // 通道ID (0-15)
    SignalType signal_type;       // 信号类型
    ADC_TypeDef* adc_instance;    // ADC实例
    uint32_t adc_channel;         // ADC通道
    float scale_factor;           // 缩放因子 (ADC值->物理值)
    float offset;                 // 偏移量
    float min_value;             // 最小值
    float max_value;             // 最大值
    uint32_t sample_rate_hz;     // 采样率 (Hz)
    FilterAlgorithm filter_type;  // 滤波算法
    uint16_t filter_window;      // 滤波窗口大小
    bool enabled;                 // 通道使能
} SignalChannelConfig;

/* 信号数据点 */
typedef struct {
    uint32_t timestamp;           // 时间戳 (ms)
    uint16_t raw_value;          // 原始ADC值
    float processed_value;       // 处理后的物理值
    bool valid;                  // 数据有效性
} SignalDataPoint;

/* 信号缓冲区 */
typedef struct {
    SignalDataPoint* buffer;     // 数据缓冲区
    uint16_t buffer_size;        // 缓冲区大小
    uint16_t write_index;        // 写索引
    uint16_t read_index;         // 读索引
    uint16_t count;              // 当前数据点数
    bool overflow;               // 缓冲区溢出标志
} SignalBuffer;

/* 信号统计信息 */
typedef struct {
    float min_value;             // 最小值
    float max_value;             // 最大值
    float average_value;         // 平均值
    float rms_value;             // 有效值
    float variance;              // 方差
    uint32_t sample_count;      // 采样计数
    uint32_t error_count;        // 错误计数
} SignalStatistics;

/* 信号采集通道 */
typedef struct {
    SignalChannelConfig config;  // 通道配置
    SignalBuffer buffer;         // 数据缓冲区
    SignalStatistics stats;      // 统计信息
    uint32_t last_sample_time;   // 最后采样时间
    float filter_state;          // 滤波器状态
    uint16_t filter_buffer[32];  // 滤波缓冲区
    uint8_t filter_index;        // 滤波索引
    bool calibration_mode;       // 校准模式
    /* V10.0: 卡尔曼滤波器独立状态 */
    float kalman_P;              // 卡尔曼滤波器误差协方差
    bool kalman_initialized;     // 卡尔曼滤波器初始化标志
} SignalChannel;

/* 信号采集系统 */
typedef struct {
    SignalChannel* channels;     // 通道数组
    uint8_t channel_count;       // 通道数量
    ADC_HandleTypeDef* hadc;     // ADC句柄
    DMA_HandleTypeDef* hdma;     // DMA句柄
    uint32_t system_time;        // 系统时间
    bool dma_enabled;            // DMA使能
    bool continuous_mode;        // 连续采集模式
    uint32_t total_samples;      // 总采样数
    uint32_t error_count;        // 总错误数
} SignalAcquisitionSystem;

/* ============ API函数声明 ============ */

/* 系统初始化 */
bool signal_acquisition_init(SignalAcquisitionSystem* sys, ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma);

/* 通道配置 */
bool signal_channel_configure(SignalAcquisitionSystem* sys, uint8_t channel_id, const SignalChannelConfig* config);

/* 启动采集 */
bool signal_acquisition_start(SignalAcquisitionSystem* sys);

/* 停止采集 */
bool signal_acquisition_stop(SignalAcquisitionSystem* sys);

/* 单次采集 */
bool signal_acquisition_single_sample(SignalAcquisitionSystem* sys, uint8_t channel_id, float* result);

/* 连续采集 */
bool signal_acquisition_continuous_start(SignalAcquisitionSystem* sys, uint8_t channel_id);

/* 获取最新数据 */
bool signal_get_latest_data(SignalAcquisitionSystem* sys, uint8_t channel_id, SignalDataPoint* data);

/* 获取历史数据 */
bool signal_get_historical_data(SignalAcquisitionSystem* sys, uint8_t channel_id, 
                                SignalDataPoint* data, uint16_t count);

/* 获取系统总采样数 */
uint32_t signal_get_total_samples(SignalAcquisitionSystem* sys);

/* 获取通道统计信息 */
bool signal_get_channel_statistics(SignalAcquisitionSystem* sys, uint8_t channel_id, SignalStatistics* stats);

/* 信号滤波处理 */
float signal_apply_filter(SignalChannel* channel, uint16_t raw_value);

/* 统计信息更新 */
void signal_update_statistics(SignalChannel* channel, float value);

/* 校准功能 */
bool signal_calibration_start(SignalAcquisitionSystem* sys, uint8_t channel_id);
bool signal_calibration_stop(SignalAcquisitionSystem* sys, uint8_t channel_id, float known_value);

/* 数据导出 */
bool signal_export_data(SignalAcquisitionSystem* sys, uint8_t channel_id, 
                       uint16_t start_index, uint16_t count, char* filename);

/* 系统状态检查 */
bool signal_acquisition_is_running(SignalAcquisitionSystem* sys);
uint32_t signal_get_total_samples(SignalAcquisitionSystem* sys);

/* 错误处理 */
void signal_clear_errors(SignalAcquisitionSystem* sys);

/* 回调函数 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);


#endif // SIGNAL_ACQUISITION_H