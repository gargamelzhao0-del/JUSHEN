#include "FreeRTOS.h"      
#include "task.h" 
#include "signal_acquisition.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

/* 全局变量 */
static SignalAcquisitionSystem* g_signal_sys = NULL;
static uint16_t g_adc_buffer[16] = {0};  // DMA缓冲区

/* 私有函数声明 */
static void signal_process_sample(SignalAcquisitionSystem* sys, uint8_t channel_id, uint16_t raw_value);
static float signal_moving_average_filter(SignalChannel* channel, uint16_t raw_value);
static float signal_median_filter(SignalChannel* channel, uint16_t raw_value);
static float signal_kalman_filter(SignalChannel* channel, uint16_t raw_value);
static float signal_lowpass_filter(SignalChannel* channel, uint16_t raw_value);
static void signal_buffer_write(SignalBuffer* buffer, const SignalDataPoint* data);
static bool signal_buffer_read(SignalBuffer* buffer, SignalDataPoint* data);

/* ============ 系统初始化 ============ */

bool signal_acquisition_init(SignalAcquisitionSystem* sys, ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma)
{
    if (!sys || !hadc) {
        return false;
    }
    
    /* 初始化系统结构 */
    memset(sys, 0, sizeof(SignalAcquisitionSystem));
    sys->hadc = hadc;
    sys->hdma = hdma;
    sys->system_time = HAL_GetTick();
    sys->dma_enabled = (hdma != NULL);
    
    /* 配置ADC */
    if (sys->dma_enabled) {
        /* DMA模式配置 - STM32H7使用HAL库函数 */
        /* 无需手动配置寄存器，HAL库会自动处理 */
    }
    
    g_signal_sys = sys;
    return true;
}

/* ============ 通道配置 ============ */

bool signal_channel_configure(SignalAcquisitionSystem* sys, uint8_t channel_id, const SignalChannelConfig* config)
{
    if (!sys || !config || channel_id >= sys->channel_count) {
        return false;
    }

    SignalChannel* channel = &sys->channels[channel_id];

    /* 复制配置 */
    memcpy(&channel->config, config, sizeof(SignalChannelConfig));

    /* 释放旧缓冲区，防止内存泄漏 */
    if (channel->buffer.buffer != NULL) {
        free(channel->buffer.buffer);
        channel->buffer.buffer = NULL;
    }

    /* 初始化缓冲区 */
    if (channel->buffer.buffer_size > 0) {
        channel->buffer.buffer = (SignalDataPoint*)malloc(
            channel->buffer.buffer_size * sizeof(SignalDataPoint));
        if (!channel->buffer.buffer) {
            return false;
        }
        memset(channel->buffer.buffer, 0,
               channel->buffer.buffer_size * sizeof(SignalDataPoint));
    }

    /* 初始化统计信息 */
    memset(&channel->stats, 0, sizeof(SignalStatistics));
    channel->stats.min_value = FLT_MAX;
    channel->stats.max_value = FLT_MIN;

    /* 初始化滤波器 */
    memset(channel->filter_buffer, 0, sizeof(channel->filter_buffer));
    channel->filter_index = 0;
    channel->filter_state = 0.0f;

    return true;
}

/* ============ 采集控制 ============ */

bool signal_acquisition_start(SignalAcquisitionSystem* sys)
{
    if (!sys || !sys->hadc) {
        return false;
    }
    
    /* 启动ADC */
    if (sys->dma_enabled) {
        /* DMA连续采集模式 */
        if (HAL_ADC_Start_DMA(sys->hadc, (uint32_t*)g_adc_buffer, 
                             sizeof(g_adc_buffer)/sizeof(g_adc_buffer[0])) != HAL_OK) {
            return false;
        }
    } else {
        /* 轮询模式 */
        if (HAL_ADC_Start(sys->hadc) != HAL_OK) {
            return false;
        }
    }
    
    sys->continuous_mode = true;
    return true;
}

bool signal_acquisition_stop(SignalAcquisitionSystem* sys)
{
    if (!sys || !sys->hadc) {
        return false;
    }
    
    if (sys->dma_enabled) {
        HAL_ADC_Stop_DMA(sys->hadc);
    } else {
        HAL_ADC_Stop(sys->hadc);
    }
    
    sys->continuous_mode = false;
    return true;
}

bool signal_acquisition_single_sample(SignalAcquisitionSystem* sys, uint8_t channel_id, float* result)
{
    if (!sys || !result || channel_id >= sys->channel_count) {
        return false;
    }
    
    SignalChannel* channel = &sys->channels[channel_id];
    
    /* 配置ADC通道 */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel->config.adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(sys->hadc, &sConfig) != HAL_OK) {
        return false;
    }
    
    /* 启动转换 */
    if (HAL_ADC_Start(sys->hadc) != HAL_OK) {
        return false;
    }
    
    /* 等待转换完成 */
    if (HAL_ADC_PollForConversion(sys->hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(sys->hadc);
        return false;
    }
    
    /* 读取结果 */
    uint16_t raw_value = HAL_ADC_GetValue(sys->hadc);
    HAL_ADC_Stop(sys->hadc);
    
    /* 处理数据 */
    *result = signal_apply_filter(channel, raw_value);
    
    /* 更新统计信息 */
    signal_process_sample(sys, channel_id, raw_value);
    
    return true;
}

/* ============ 数据处理 ============ */

static void signal_process_sample(SignalAcquisitionSystem* sys, uint8_t channel_id, uint16_t raw_value)
{
    if (!sys || channel_id >= sys->channel_count) {
        return;
    }
    
    SignalChannel* channel = &sys->channels[channel_id];
    
    /* 应用滤波 */
    float processed_value = signal_apply_filter(channel, raw_value);
    
    /* 创建数据点 */
    SignalDataPoint data_point;
    data_point.timestamp = HAL_GetTick();
    data_point.raw_value = raw_value;
    data_point.processed_value = processed_value;
    data_point.valid = true;
    
    /* 写入缓冲区 */
    signal_buffer_write(&channel->buffer, &data_point);
    
    /* 更新统计信息 */
    signal_update_statistics(channel, processed_value);
    
    sys->total_samples++;
    channel->last_sample_time = data_point.timestamp;
}

float signal_apply_filter(SignalChannel* channel, uint16_t raw_value)
{
    if (!channel) {
        return 0.0f;
    }
    
    /* 转换为物理值 */
    float physical_value = (raw_value * channel->config.scale_factor) + channel->config.offset;
    
    /* 应用滤波算法 */
    switch (channel->config.filter_type) {
        case FILTER_MOVING_AVERAGE:
            return signal_moving_average_filter(channel, raw_value);
        case FILTER_MEDIAN:
            return signal_median_filter(channel, raw_value);
        case FILTER_KALMAN:
            return signal_kalman_filter(channel, raw_value);
        case FILTER_LOWPASS:
            return signal_lowpass_filter(channel, raw_value);
        case FILTER_NONE:
        default:
            return physical_value;
    }
}

/* ============ 滤波算法实现 ============ */

static float signal_moving_average_filter(SignalChannel* channel, uint16_t raw_value)
{
    /* 更新滤波缓冲区 */
    channel->filter_buffer[channel->filter_index] = raw_value;
    channel->filter_index = (channel->filter_index + 1) % channel->config.filter_window;
    
    /* 计算平均值 */
    uint32_t sum = 0;
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < channel->config.filter_window; i++) {
        if (channel->filter_buffer[i] != 0) {
            sum += channel->filter_buffer[i];
            valid_count++;
        }
    }
    
    if (valid_count == 0) {
        return (raw_value * channel->config.scale_factor) + channel->config.offset;
    }
    
    float avg_raw = (valid_count > 0) ? (float)sum / valid_count : (float)raw_value;
    return (avg_raw * channel->config.scale_factor) + channel->config.offset;
}

static float signal_median_filter(SignalChannel* channel, uint16_t raw_value)
{
    /* 更新滤波缓冲区 */
    channel->filter_buffer[channel->filter_index] = raw_value;
    channel->filter_index = (channel->filter_index + 1) % channel->config.filter_window;
    
    /* 复制并排序 */
    uint16_t temp_buffer[32];
    memcpy(temp_buffer, channel->filter_buffer, sizeof(temp_buffer));
    
    /* 简单冒泡排序 */
    for (uint8_t i = 0; i < channel->config.filter_window - 1; i++) {
        for (uint8_t j = 0; j < channel->config.filter_window - i - 1; j++) {
            if (temp_buffer[j] > temp_buffer[j + 1]) {
                uint16_t temp = temp_buffer[j];
                temp_buffer[j] = temp_buffer[j + 1];
                temp_buffer[j + 1] = temp;
            }
        }
    }
    
    /* 取中值 */
    uint16_t median_raw = temp_buffer[channel->config.filter_window / 2];
    return (median_raw * channel->config.scale_factor) + channel->config.offset;
}

static float signal_kalman_filter(SignalChannel* channel, uint16_t raw_value)
{
    /* 简化的卡尔曼滤波 */
    float Q = 0.1f;  // 过程噪声
    float R = 0.1f;  // 测量噪声
    
    float measured_value = (raw_value * channel->config.scale_factor) + channel->config.offset;
    
    /* V10.0: 使用结构体中的独立状态，避免多通道冲突 */
    if (!channel->kalman_initialized) {
        /* 第一次调用：初始化滤波器状态 */
        channel->filter_state = measured_value;
        channel->kalman_P = 1.0f;
        channel->kalman_initialized = true;
        
        return channel->filter_state;
    }
    
    /* 预测步骤 */
    channel->kalman_P = channel->kalman_P + Q;
    
    /* 更新步骤 */
    float K = channel->kalman_P / (channel->kalman_P + R);
    channel->filter_state = channel->filter_state + K * (measured_value - channel->filter_state);
    channel->kalman_P = (1 - K) * channel->kalman_P;
    
    return channel->filter_state;
}

static float signal_lowpass_filter(SignalChannel* channel, uint16_t raw_value)
{
    float alpha = 0.1f;  // 滤波系数
    float measured_value = (raw_value * channel->config.scale_factor) + channel->config.offset;
    
    channel->filter_state = alpha * measured_value + (1 - alpha) * channel->filter_state;
    return channel->filter_state;
}

/* ============ 缓冲区管理 ============ */

static void signal_buffer_write(SignalBuffer* buffer, const SignalDataPoint* data)
{
    if (!buffer || !buffer->buffer || !data) {
        return;
    }
    
    buffer->buffer[buffer->write_index] = *data;
    buffer->write_index = (buffer->write_index + 1) % buffer->buffer_size;
    
    if (buffer->count < buffer->buffer_size) {
        buffer->count++;
    } else {
        buffer->overflow = true;
        buffer->read_index = (buffer->read_index + 1) % buffer->buffer_size;
    }
}

static bool signal_buffer_read(SignalBuffer* buffer, SignalDataPoint* data)
{
    if (!buffer || !buffer->buffer || !data || buffer->count == 0) {
        return false;
    }
    
    *data = buffer->buffer[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % buffer->buffer_size;
    buffer->count--;
    
    return true;
}

/* ============ 数据获取 ============ */

bool signal_get_latest_data(SignalAcquisitionSystem* sys, uint8_t channel_id, SignalDataPoint* data)
{
    if (!sys || !data || channel_id >= sys->channel_count) {
        return false;
    }
    
    SignalChannel* channel = &sys->channels[channel_id];
    SignalBuffer* buffer = &channel->buffer;
    
    if (buffer->count == 0) {
        return false;
    }
    
    uint16_t latest_index = (buffer->write_index == 0) ? 
                           buffer->buffer_size - 1 : buffer->write_index - 1;
    *data = buffer->buffer[latest_index];
    
    return true;
}

bool signal_get_historical_data(SignalAcquisitionSystem* sys, uint8_t channel_id, 
                               SignalDataPoint* data, uint16_t count)
{
    if (!sys || !data || channel_id >= sys->channel_count) {
        return false;
    }
    
    SignalChannel* channel = &sys->channels[channel_id];
    SignalBuffer* buffer = &channel->buffer;
    
    if (count > buffer->count) {
        count = buffer->count;
    }
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t index = (buffer->read_index + i) % buffer->buffer_size;
        data[i] = buffer->buffer[index];
    }
    
    return true;
}

/* ============ 统计信息 ============ */

void signal_update_statistics(SignalChannel* channel, float value)
{
    if (!channel) {
        return;
    }
    
    SignalStatistics* stats = &channel->stats;
    
    /* 更新最小最大值 */
    if (value < stats->min_value) {
        stats->min_value = value;
    }
    if (value > stats->max_value) {
        stats->max_value = value;
    }
    
    /* 更新平均值和方差 */
    stats->sample_count++;
    float delta = value - stats->average_value;
    stats->average_value += delta / stats->sample_count;
    
    if (stats->sample_count > 1) {
        float delta2 = value - stats->average_value;
        stats->variance = ((stats->sample_count - 2) * stats->variance + delta * delta2) / (stats->sample_count - 1);
    }
    
    /* 更新RMS值 */
    stats->rms_value = sqrtf((stats->rms_value * stats->rms_value * (stats->sample_count - 1) + value * value) / stats->sample_count);
}

/* ============ 校准功能 ============ */

bool signal_calibration_start(SignalAcquisitionSystem* sys, uint8_t channel_id)
{
    if (!sys || channel_id >= sys->channel_count) {
        return false;
    }
    
    sys->channels[channel_id].calibration_mode = true;
    memset(&sys->channels[channel_id].stats, 0, sizeof(SignalStatistics));
    sys->channels[channel_id].stats.min_value = FLT_MAX;
    sys->channels[channel_id].stats.max_value = FLT_MIN;
    
    return true;
}

bool signal_calibration_stop(SignalAcquisitionSystem* sys, uint8_t channel_id, float known_value)
{
    if (!sys || channel_id >= sys->channel_count) {
        return false;
    }
    
    SignalChannel* channel = &sys->channels[channel_id];
    
    if (!channel->calibration_mode || channel->stats.sample_count == 0) {
        return false;
    }
    
    /* 计算新的缩放因子和偏移量 */
    float measured_avg = channel->stats.average_value;
    channel->config.scale_factor = known_value / measured_avg;
    channel->config.offset = 0.0f;  // 假设已知值为0点
    
    channel->calibration_mode = false;
    return true;
}

/* ============ ADC回调函数 ============ */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (!g_signal_sys || hadc != g_signal_sys->hadc) {
        return;
    }
    
    /* 处理DMA转换完成的数据 */
    for (uint8_t i = 0; i < g_signal_sys->channel_count; i++) {
        if (g_signal_sys->channels[i].config.enabled) {
            signal_process_sample(g_signal_sys, i, g_adc_buffer[i]);
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* 半缓冲区转换完成处理 */
    HAL_ADC_ConvCpltCallback(hadc);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if (g_signal_sys) {
        g_signal_sys->error_count++;
    }
}

/* ============ 工具函数 ============ */

bool signal_acquisition_is_running(SignalAcquisitionSystem* sys)
{
    return sys && sys->continuous_mode;
}

uint32_t signal_get_total_samples(SignalAcquisitionSystem* sys)
{
    return sys ? sys->total_samples : 0;
}

void signal_clear_errors(SignalAcquisitionSystem* sys)
{
    if (sys) {
        sys->error_count = 0;
        for (uint8_t i = 0; i < sys->channel_count; i++) {
            sys->channels[i].stats.error_count = 0;
        }
    }
}

/* ============ 连续采样功能 ============ */

bool signal_acquisition_continuous_start(SignalAcquisitionSystem* sys, uint8_t channel_id)
{
    if (!sys || channel_id >= sys->channel_count || !sys->channels[channel_id].config.enabled) {
        return false;
    }
    
    /* 启动连续采样模式 */
    sys->continuous_mode = true;
    
    /* 配置ADC为连续转换模式 */
    if (sys->dma_enabled) {
        /* 启动DMA连续转换 */
        if (HAL_ADC_Start_DMA(sys->hadc, (uint32_t*)g_adc_buffer, sys->channel_count) != HAL_OK) {
            return false;
        }
    } else {
        /* 启动轮询模式连续转换 */
        if (HAL_ADC_Start(sys->hadc) != HAL_OK) {
            return false;
        }
    }
    
    return true;
}

bool signal_get_channel_statistics(SignalAcquisitionSystem* sys, uint8_t channel_id, SignalStatistics* stats)
{
    if (!sys || channel_id >= sys->channel_count || !stats) {
        return false;
    }
    
    /* 复制统计信息 */
    memcpy(stats, &sys->channels[channel_id].stats, sizeof(SignalStatistics));
    return true;
}

