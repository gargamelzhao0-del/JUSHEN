/**
 ******************************************************************************
 * @file    foot_force_sensor.c
 * @brief   足端力传感器模块实现
 ******************************************************************************
 */

#include "foot_force_sensor.h"
#include "adc.h"
#include "signal_acquisition.h"
#include "log_system.h"
#include "param_config.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 传感器数据 */
static FootForceData_t g_foot_data[FOOT_FORCE_SENSOR_COUNT] = {0};

/* 校准数据
 * ============================================================================
 * ⚠️ 重要提示：以下校准参数需要根据实际传感器进行标定！
 * 
 * 校准参数说明：
 * ┌─────────────┬──────────────────────────────────────────────────────────┐
 * │ 参数        │ 说明                                                      │
 * ├─────────────┼──────────────────────────────────────────────────────────┤
 * │ offset      │ 零点偏移（N），无负载时的输出值                           │
 * │ gain        │ 增益（N/V），电压到力的转换系数                           │
 * │ adc_zero    │ ADC零点值，无负载时的ADC读数（通常为ADC中点，如2048）     │
 * └─────────────┴──────────────────────────────────────────────────────────┘
 * 
 * 标定方法：
 * 1. 零点标定：传感器无负载时，记录ADC读数作为adc_zero
 * 2. 增益标定：施加已知重量（如1kg=9.8N），测量ADC变化量
 *    gain = 力值变化 / 电压变化 = 力值 / ((ADC变化/4096) * 3.3V)
 * 3. 偏移标定：根据实际安装位置调整offset
 * 
 * 标定步骤：
 * 1. 将机器狗抬起，使四脚悬空
 * 2. 调用foot_force_calibrate_zero()进行零点标定
 * 3. 逐个脚放置标准砝码，记录ADC变化
 * 4. 计算gain并更新下方参数
 * 
 * 注意：每个传感器的参数可能不同，建议逐个标定！
 */
static FootSensorCalib_t g_calib[FOOT_FORCE_SENSOR_COUNT] = {
    {0.0f, 100.0f, 2048},  /* 前左：默认增益100N/V，零点2048 */
    {0.0f, 100.0f, 2048},  /* 前右 */
    {0.0f, 100.0f, 2048},  /* 后左 */
    {0.0f, 100.0f, 2048}   /* 后右 */
};

/* ADC通道映射 */
static uint8_t g_adc_channel[FOOT_FORCE_SENSOR_COUNT] = {
    ADC_CH_PA3,  /* 前左 → PA3 */
    ADC_CH_PA6,  /* 前右 → PA6 */
    ADC_CH_PB0,  /* 后左 → PB0 */
    ADC_CH_PB1   /* 后右 → PB1 */
};

/* 阈值配置 */
static float g_contact_threshold  = 5.0f;   /* 接触阈值（N）*/
static float g_support_threshold  = 20.0f;  /* 支撑阈值（N）*/
static float g_overload_threshold = 200.0f; /* 过载阈值（N）*/

/* 滤波器缓冲区 */
static float g_filter_buffer[FOOT_FORCE_SENSOR_COUNT][FOOT_FORCE_FILTER_SIZE] = {0};
static uint8_t g_filter_index[FOOT_FORCE_SENSOR_COUNT] = {0};

/* 互斥锁 */
static osMutexId_t foot_force_mutex = NULL;

/* 上一次接触状态（用于边沿检测）*/
static bool g_last_contact[FOOT_FORCE_SENSOR_COUNT] = {false};

/* 滤波器配置 */
static FilterType_t g_filter_type[FOOT_FORCE_SENSOR_COUNT] = {FILTER_TYPE_MOVING_AVERAGE};
static float g_lowpass_alpha[FOOT_FORCE_SENSOR_COUNT] = {0.1f};

/* 力变化率计算 */
static float g_last_force[FOOT_FORCE_SENSOR_COUNT] = {0.0f};
static float g_force_rate[FOOT_FORCE_SENSOR_COUNT] = {0.0f};

/* 地面粗糙度估计 */
static float g_roughness_buffer[FOOT_FORCE_SENSOR_COUNT][20] = {0};
static uint8_t g_roughness_index[FOOT_FORCE_SENSOR_COUNT] = {0};

/* ============== 故障检测相关变量 ============== */
typedef struct {
    bool sensor_fault;              /* 传感器故障标志 */
    bool signal_out_of_range;       /* 信号超出范围 */
    bool signal_stuck;              /* 信号卡死 */
    uint32_t fault_count;           /* 故障计数 */
    uint32_t last_fault_time;       /* 最后故障时间 */
    uint16_t last_adc_value;        /* 上次ADC值 */
    uint32_t stuck_counter;         /* 卡死计数器 */
    float last_valid_force;         /* 最后有效力值 */
    uint32_t current_recovery_ms;   /* V2.0: 当前恢复等待时间 */
    uint8_t normal_count;           /* V2.0: 连续正常计数 */
} SensorFaultContext_t;

static SensorFaultContext_t g_fault_ctx[FOOT_FORCE_SENSOR_COUNT] = {0};

/* 故障检测阈值 */
#define FOOT_FORCE_ADC_MIN_VALID      100     /* ADC最小有效值 */
#define FOOT_FORCE_ADC_MAX_VALID      65000   /* ADC最大有效值 */
#define FOOT_FORCE_STUCK_THRESHOLD    50      /* ADC值变化小于此值视为卡死 */
#define FOOT_FORCE_STUCK_COUNT_MAX    100     /* 连续卡死次数阈值 */

/* V2.0: 渐进式故障恢复配置 */
#define FOOT_FORCE_FAULT_RECOVERY_BASE_MS    1000    /* 基础故障恢复时间 (ms) */
#define FOOT_FORCE_FAULT_RECOVERY_MAX_MS     5000    /* 最大故障恢复时间 (ms) */
#define FOOT_FORCE_FAULT_RECOVERY_INCR_MS    500     /* 每次失败增加时间 (ms) */
#define FOOT_FORCE_FAULT_CLEAR_COUNT         3       /* 连续正常次数清除故障 */

/* ============================================================================ 
 * 私有函数原型
 * ============================================================================ */

static float moving_average_filter(FootID_t foot_id, float new_value);
static float low_pass_filter(FootID_t foot_id, float new_value);
static float median_filter(FootID_t foot_id, float new_value);
static float combined_filter(FootID_t foot_id, float new_value);
static void update_foot_state(FootID_t foot_id);
static bool check_sensor_fault(FootID_t foot_id, uint16_t adc_value);

/* ============================================================================
 * 私有函数实现 - 故障检测
 * ============================================================================ */

/**
 * @brief  检查传感器故障
 * @param  foot_id: 足端ID
 * @param  adc_value: ADC采样值
 * @retval true=故障, false=正常
 */
static bool check_sensor_fault(FootID_t foot_id, uint16_t adc_value)
{
    SensorFaultContext_t *ctx = &g_fault_ctx[foot_id];
    bool fault_detected = false;
    uint32_t current_time = HAL_GetTick();
    
    /* 1. 检查信号是否超出范围 */
    if (adc_value < FOOT_FORCE_ADC_MIN_VALID || adc_value > FOOT_FORCE_ADC_MAX_VALID) {
        ctx->signal_out_of_range = true;
        fault_detected = true;
        LOG_WARN("[FootForce] %s: ADC out of range (%u)", 
                 foot_force_get_name(foot_id), adc_value);
    } else {
        ctx->signal_out_of_range = false;
    }
    
    /* 2. 检查信号是否卡死（连续多次ADC值几乎不变）*/
    int16_t adc_delta = (int16_t)adc_value - (int16_t)ctx->last_adc_value;
    if (abs(adc_delta) < FOOT_FORCE_STUCK_THRESHOLD) {
        ctx->stuck_counter++;
        if (ctx->stuck_counter >= FOOT_FORCE_STUCK_COUNT_MAX) {
            ctx->signal_stuck = true;
            fault_detected = true;
            LOG_WARN("[FootForce] %s: Signal stuck detected (ADC=%u, count=%u)",
                     foot_force_get_name(foot_id), adc_value, ctx->stuck_counter);
        }
    } else {
        ctx->stuck_counter = 0;
        ctx->signal_stuck = false;
    }
    
    ctx->last_adc_value = adc_value;
    
    if (fault_detected) {
        ctx->sensor_fault = true;
        ctx->fault_count++;
        ctx->last_fault_time = current_time;
        ctx->normal_count = 0;
        if (ctx->current_recovery_ms < FOOT_FORCE_FAULT_RECOVERY_MAX_MS) {
            ctx->current_recovery_ms += FOOT_FORCE_FAULT_RECOVERY_INCR_MS;
            if (ctx->current_recovery_ms > FOOT_FORCE_FAULT_RECOVERY_MAX_MS) {
                ctx->current_recovery_ms = FOOT_FORCE_FAULT_RECOVERY_MAX_MS;
            }
        }
    } else {
        if (ctx->sensor_fault) {
            ctx->normal_count++;
            if ((current_time - ctx->last_fault_time) > ctx->current_recovery_ms &&
                ctx->normal_count >= FOOT_FORCE_FAULT_CLEAR_COUNT) {
                uint32_t actual_recovery_ms = ctx->current_recovery_ms;
                ctx->sensor_fault = false;
                ctx->signal_out_of_range = false;
                ctx->signal_stuck = false;
                ctx->stuck_counter = 0;
                ctx->normal_count = 0;
                ctx->current_recovery_ms = FOOT_FORCE_FAULT_RECOVERY_BASE_MS;
                LOG_INFO("[FootForce] %s: Fault cleared (recovery time: %lums)",
                         foot_force_get_name(foot_id), actual_recovery_ms);
            }
        }
    }
    
    return ctx->sensor_fault;
}

/**
 * @brief  检查传感器是否有故障
 * @param  foot_id: 足端ID
 * @retval true=有故障, false=正常
 */
bool foot_force_has_fault(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return true;
    }
    return g_fault_ctx[foot_id].sensor_fault;
}

/**
 * @brief  获取传感器故障信息
 * @param  foot_id: 足端ID
 * @param  fault_info: 故障信息输出
 * @retval 0=成功, -1=失败
 */
int foot_force_get_fault_info(FootID_t foot_id, uint32_t *fault_count, bool *is_stuck, bool *out_of_range)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return -1;
    }
    
    if (fault_count) *fault_count = g_fault_ctx[foot_id].fault_count;
    if (is_stuck) *is_stuck = g_fault_ctx[foot_id].signal_stuck;
    if (out_of_range) *out_of_range = g_fault_ctx[foot_id].signal_out_of_range;
    
    return 0;
}

/* ============================================================================
 * 公共函数实现 - 初始化与配置
 * ============================================================================ */

/**
 * @brief  初始化足端力传感器系统
 */
int foot_force_init(void)
{
    /* 创建互斥锁 */
    const osMutexAttr_t mutex_attr = {
        .name = "FootForceMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    foot_force_mutex = osMutexNew(&mutex_attr);
    if (foot_force_mutex == NULL) {
        LOG_ERROR("[FootForce] Failed to create mutex");
        return -1;
    }

    /* 初始化所有传感器数据 */
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        memset(&g_foot_data[i], 0, sizeof(FootForceData_t));
        g_foot_data[i].state = FOOT_STATE_IN_AIR;
        g_foot_data[i].is_contact = false;
    }

    /* 尝试从参数系统加载校准数据 */
    if (foot_force_load_calibration() != 0) {
        LOG_WARN("[FootForce] No saved calibration found, using defaults");
    }

    LOG_INFO("[FootForce] Foot force sensor system initialized");
    LOG_INFO("[FootForce] Thresholds: contact=%.1fN, support=%.1fN, overload=%.1fN",
             g_contact_threshold, g_support_threshold, g_overload_threshold);

    return 0;
}

/**
 * @brief  设置传感器与ADC通道的映射
 */
int foot_force_set_channel(FootID_t foot_id, uint8_t adc_channel)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return -1;
    }

    g_adc_channel[foot_id] = adc_channel;
    LOG_INFO("[FootForce] %s mapped to ADC channel %d",
             foot_force_get_name(foot_id), adc_channel);
    return 0;
}

/**
 * @brief  校准传感器（零点标定）
 */
void foot_force_calibrate_zero(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return;
    }

    /* 读取当前ADC值作为零点 */
    uint16_t adc_value = ADC_GetValue(g_adc_channel[foot_id]);
    g_calib[foot_id].adc_zero = adc_value;
    g_calib[foot_id].offset = 0.0f;

    LOG_INFO("[FootForce] %s calibrated: ADC_zero=%u",
             foot_force_get_name(foot_id), adc_value);
}

/**
 * @brief  设置传感器增益
 */
void foot_force_set_scale(FootID_t foot_id, float scale)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return;
    }

    g_calib[foot_id].scale = scale;
    LOG_INFO("[FootForce] %s scale set to %.2f N/V",
             foot_force_get_name(foot_id), scale);
}

/**
 * @brief  校准所有传感器（一键零点标定）
 */
void foot_force_calibrate_all_zero(void)
{
    LOG_INFO("[FootForce] Starting calibration for all sensors...");
    LOG_WARN("[FootForce] ⚠️  Please ensure all feet are in the air (no load)");

    /* 给用户2秒准备时间 */
    osDelay(2000);

    LOG_INFO("[FootForce] Calibrating...");

    /* 校准所有4个传感器 */
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        foot_force_calibrate_zero((FootID_t)i);
        osDelay(100);  /* 延迟100ms让传感器稳定 */
    }

    LOG_INFO("[FootForce] ✓ All sensors calibrated successfully!");
    LOG_INFO("[FootForce] TIP: Use 'F' to save calibration to FLASH");
}

/**
 * @brief  保存校准数据到参数系统
 */
int foot_force_save_calibration(void)
{
    /* 包含param_config.h */
    extern int param_get_sensor(SensorParam_t *param);
    extern int param_set_sensor(const SensorParam_t *param);
    extern int param_save(void);

    SensorParam_t sensor_param;

    /* 读取当前传感器参数 */
    if (param_get_sensor(&sensor_param) != 0) {
        LOG_ERROR("[FootForce] Failed to get sensor parameters");
        return -1;
    }

    /* 更新校准数据 */
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        /* 保存零点值（使用force_zero数组存储ADC零点，需要归一化到0-1范围）*/
        sensor_param.force_zero[i] = (float)g_calib[i].adc_zero;

        /* 保存增益（使用force_scale数组）*/
        sensor_param.force_scale[i] = g_calib[i].scale;
    }

    /* 写入参数系统 */
    if (param_set_sensor(&sensor_param) != 0) {
        LOG_ERROR("[FootForce] Failed to set sensor parameters");
        return -1;
    }

    /* 保存到FLASH */
    if (param_save() != 0) {
        LOG_ERROR("[FootForce] Failed to save parameters to FLASH");
        return -1;
    }

    LOG_INFO("[FootForce] ✓ Calibration data saved to FLASH");
    return 0;
}

/**
 * @brief  从参数系统加载校准数据
 */
int foot_force_load_calibration(void)
{
    /* 包含param_config.h */
    extern int param_get_sensor(SensorParam_t *param);

    SensorParam_t sensor_param;

    /* 读取传感器参数 */
    if (param_get_sensor(&sensor_param) != 0) {
        LOG_WARN("[FootForce] Failed to load calibration, using defaults");
        return -1;
    }

    /* 应用校准数据 */
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        /* 加载零点值 */
        g_calib[i].adc_zero = (uint16_t)sensor_param.force_zero[i];

        /* 加载增益 */
        g_calib[i].scale = sensor_param.force_scale[i];

        LOG_INFO("[FootForce] %s: ADC_zero=%u, scale=%.2f N/V",
                 foot_force_get_name((FootID_t)i),
                 g_calib[i].adc_zero,
                 g_calib[i].scale);
    }

    LOG_INFO("[FootForce] ✓ Calibration data loaded from FLASH");
    return 0;
}

/**
 * @brief  打印校准数据
 */
void foot_force_print_calibration(void)
{
    LOG_INFO("[FootForce] ==================== Calibration Data ====================");

    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        LOG_INFO("[FootForce]   %s:",
                 foot_force_get_name((FootID_t)i));
        LOG_INFO("[FootForce]     ADC Zero:  %u (%.3f V)",
                 g_calib[i].adc_zero,
                 (float)g_calib[i].adc_zero * 3.3f / 65535.0f);
        LOG_INFO("[FootForce]     Scale:     %.2f N/V",
                 g_calib[i].scale);
        LOG_INFO("[FootForce]     Offset:    %.3f N",
                 g_calib[i].offset);
    }

    LOG_INFO("[FootForce] ============================================================");
}

/* ============================================================================
 * 公共函数实现 - 数据读取
 * ============================================================================ */

/**
 * @brief  更新传感器数据
 */
int foot_force_update(FootID_t foot_id, uint16_t adc_value)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return -1;
    }

    osMutexAcquire(foot_force_mutex, osWaitForever);

    /* 0. 故障检测 */
    if (check_sensor_fault(foot_id, adc_value)) {
        /* 传感器故障时使用最后有效值 */
        g_foot_data[foot_id].force = g_fault_ctx[foot_id].last_valid_force;
        g_foot_data[foot_id].force_filtered = g_fault_ctx[foot_id].last_valid_force;
        g_foot_data[foot_id].state = FOOT_STATE_IN_AIR;  /* 故障时默认为腾空 */
        g_foot_data[foot_id].is_contact = false;
        osMutexRelease(foot_force_mutex);
        return -2;  /* 返回故障状态 */
    }

    /* 1. 计算电压（假设3.3V参考电压，16位ADC）*/
    float voltage = (float)adc_value * 3.3f / 65535.0f;

    /* 2. 计算力值（考虑零点和增益）*/
    float zero_voltage = (float)g_calib[foot_id].adc_zero * 3.3f / 65535.0f;
    float delta_voltage = voltage - zero_voltage;
    float force = delta_voltage * g_calib[foot_id].scale + g_calib[foot_id].offset;

    /* 力值不能为负（传感器不测拉力）*/
    if (force < 0.0f) {
        force = 0.0f;
    }

    /* 3. 滤波 */
    float force_filtered;
    switch (g_filter_type[foot_id]) {
        case FILTER_TYPE_LOW_PASS:
            force_filtered = low_pass_filter(foot_id, force);
            break;
        case FILTER_TYPE_MEDIAN:
            force_filtered = median_filter(foot_id, force);
            break;
        case FILTER_TYPE_COMBINED:
            force_filtered = combined_filter(foot_id, force);
            break;
        default:
            force_filtered = moving_average_filter(foot_id, force);
            break;
    }

    /* 4. 更新数据 */
    g_foot_data[foot_id].force = force;
    g_foot_data[foot_id].force_filtered = force_filtered;
    
    /* 5. 计算力变化率 */
    float delta_force = force_filtered - g_last_force[foot_id];
    float delta_time = 0.01f;  /* 假设100Hz采样率 */
    g_force_rate[foot_id] = delta_force / delta_time;
    g_last_force[foot_id] = force_filtered;

    /* 更新峰值 */
    if (force_filtered > g_foot_data[foot_id].force_max) {
        g_foot_data[foot_id].force_max = force_filtered;
    }
    if (force_filtered < g_foot_data[foot_id].force_min || g_foot_data[foot_id].force_min == 0.0f) {
        g_foot_data[foot_id].force_min = force_filtered;
    }

    /* 5. 更新状态 */
    update_foot_state(foot_id);

    /* 6. 更新时间戳 */
    g_foot_data[foot_id].last_update_ms = HAL_GetTick();

    /* 7. 更新接触时间 */
    if (g_foot_data[foot_id].is_contact) {
        g_foot_data[foot_id].contact_time_ms++;
    } else {
        g_foot_data[foot_id].contact_time_ms = 0;
    }

    /* 8. 保存最后有效力值（用于故障恢复）*/
    g_fault_ctx[foot_id].last_valid_force = force_filtered;

    osMutexRelease(foot_force_mutex);

    return 0;
}

/**
 * @brief  获取足端力数据
 */
int foot_force_get_data(FootID_t foot_id, FootForceData_t *data)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT || data == NULL) {
        return -1;
    }

    osMutexAcquire(foot_force_mutex, osWaitForever);
    memcpy(data, &g_foot_data[foot_id], sizeof(FootForceData_t));
    osMutexRelease(foot_force_mutex);

    return 0;
}

/**
 * @brief  获取足端力值
 */
float foot_force_get_value(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return -1.0f;
    }
    return g_foot_data[foot_id].force;
}

/**
 * @brief  获取滤波后的力值
 */
float foot_force_get_filtered(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return -1.0f;
    }
    return g_foot_data[foot_id].force_filtered;
}

/**
 * @brief  获取足端状态
 */
FootState_t foot_force_get_state(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return FOOT_STATE_IN_AIR;
    }
    return g_foot_data[foot_id].state;
}

/**
 * @brief  检查是否接触地面
 */
bool foot_force_is_contact(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return false;
    }
    return g_foot_data[foot_id].is_contact;
}

/**
 * @brief  获取所有足端的接触状态（位掩码）
 */
uint8_t foot_force_get_contact_mask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        if (g_foot_data[i].is_contact) {
            mask |= (1 << i);
        }
    }
    return mask;
}

/**
 * @brief  获取支撑足数量
 */
uint8_t foot_force_get_support_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        if (g_foot_data[i].state == FOOT_STATE_SUPPORTING) {
            count++;
        }
    }
    return count;
}

/* ============================================================================
 * 公共函数实现 - 阈值设置
 * ============================================================================ */

void foot_force_set_contact_threshold(float contact_threshold)
{
    g_contact_threshold = contact_threshold;
}

void foot_force_set_support_threshold(float support_threshold)
{
    g_support_threshold = support_threshold;
}

void foot_force_set_overload_threshold(float overload_threshold)
{
    g_overload_threshold = overload_threshold;
}

/* ============================================================================
 * 公共函数实现 - 高级功能
 * ============================================================================ */

/**
 * @brief  检测步态事件
 */
int foot_force_detect_gait_event(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return 0;
    }

    bool current_contact = g_foot_data[foot_id].is_contact;
    bool last_contact = g_last_contact[foot_id];

    /* 触地事件：从腾空到接触 */
    if (current_contact && !last_contact) {
        g_last_contact[foot_id] = current_contact;
        return 1;  /* 触地 */
    }

    /* 离地事件：从接触到腾空 */
    if (!current_contact && last_contact) {
        g_last_contact[foot_id] = current_contact;
        return -1;  /* 离地 */
    }

    g_last_contact[foot_id] = current_contact;
    return 0;  /* 无事件 */
}

/**
 * @brief  重置峰值统计
 */
void foot_force_reset_stats(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return;
    }

    osMutexAcquire(foot_force_mutex, osWaitForever);
    g_foot_data[foot_id].force_max = 0.0f;
    g_foot_data[foot_id].force_min = 0.0f;
    osMutexRelease(foot_force_mutex);
}

/**
 * @brief  打印所有足端力数据
 */
void foot_force_print_all(void)
{
    LOG_INFO("========== Foot Force Sensors ==========");
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        LOG_INFO("  %s: %.2fN [%s] %s",
                 foot_force_get_name((FootID_t)i),
                 g_foot_data[i].force_filtered,
                 foot_force_get_state_name(g_foot_data[i].state),
                 g_foot_data[i].is_contact ? "CONTACT" : "IN_AIR");
    }
    LOG_INFO("  Support count: %d/4", foot_force_get_support_count());
    LOG_INFO("========================================");
}

/**
 * @brief  获取足端名称字符串
 */
const char* foot_force_get_name(FootID_t foot_id)
{
    switch (foot_id) {
        case FOOT_FRONT_LEFT:  return "FL";
        case FOOT_FRONT_RIGHT: return "FR";
        case FOOT_REAR_LEFT:   return "RL";
        case FOOT_REAR_RIGHT:  return "RR";
        default:               return "??";
    }
}

/**
 * @brief  获取足端状态名称字符串
 */
const char* foot_force_get_state_name(FootState_t state)
{
    switch (state) {
        case FOOT_STATE_IN_AIR:     return "In Air";
        case FOOT_STATE_TOUCHING:   return "Touching";
        case FOOT_STATE_SUPPORTING: return "Supporting";
        case FOOT_STATE_OVERLOAD:   return "OVERLOAD!";
        default:                    return "Unknown";
    }
}

/**
 * @brief  检查传感器是否有效（V10.0新增）
 * @note   用于力传感器超时检测，支持自动降级
 */
bool foot_force_is_valid(FootID_t foot_id, uint32_t timeout_ms)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return false;
    }
    
    uint32_t current_tick = HAL_GetTick();
    uint32_t last_update = g_foot_data[foot_id].last_update_ms;
    
    if (last_update == 0) {
        return false;
    }
    
    uint32_t elapsed = (current_tick >= last_update) ? 
                       (current_tick - last_update) : 
                       (0xFFFFFFFF - last_update + current_tick + 1);
    
    return (elapsed <= timeout_ms);
}

/**
 * @brief  获取传感器有效状态掩码（V10.0新增）
 */
uint8_t foot_force_get_valid_mask(uint32_t timeout_ms)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        if (foot_force_is_valid((FootID_t)i, timeout_ms)) {
            mask |= (1 << i);
        }
    }
    return mask;
}

/**
 * @brief  检查是否所有传感器都有效（V10.0新增）
 */
bool foot_force_all_valid(uint32_t timeout_ms)
{
    return (foot_force_get_valid_mask(timeout_ms) == 0x0F);
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  移动平均滤波器
 */
static float moving_average_filter(FootID_t foot_id, float new_value)
{
    /* 更新缓冲区 */
    g_filter_buffer[foot_id][g_filter_index[foot_id]] = new_value;
    g_filter_index[foot_id] = (g_filter_index[foot_id] + 1) % FOOT_FORCE_FILTER_SIZE;

    /* 计算平均值 */
    float sum = 0.0f;
    for (uint8_t i = 0; i < FOOT_FORCE_FILTER_SIZE; i++) {
        sum += g_filter_buffer[foot_id][i];
    }

    return sum / (float)FOOT_FORCE_FILTER_SIZE;
}

/**
 * @brief  更新足端状态
 */
static void update_foot_state(FootID_t foot_id)
{
    float force = g_foot_data[foot_id].force_filtered;

    /* 判断状态 */
    if (force >= g_overload_threshold) {
        g_foot_data[foot_id].state = FOOT_STATE_OVERLOAD;
        g_foot_data[foot_id].is_contact = true;
    } else if (force >= g_support_threshold) {
        g_foot_data[foot_id].state = FOOT_STATE_SUPPORTING;
        g_foot_data[foot_id].is_contact = true;
    } else if (force >= g_contact_threshold) {
        g_foot_data[foot_id].state = FOOT_STATE_TOUCHING;
        g_foot_data[foot_id].is_contact = true;
    } else {
        g_foot_data[foot_id].state = FOOT_STATE_IN_AIR;
        g_foot_data[foot_id].is_contact = false;
    }
}

/* ============================================================================ 
 * 私有函数实现 - 滤波器
 * ============================================================================ */

/**
 * @brief  低通滤波器
 */
static float low_pass_filter(FootID_t foot_id, float new_value)
{
    static float filtered_value[FOOT_FORCE_SENSOR_COUNT] = {0.0f};
    
    filtered_value[foot_id] = g_lowpass_alpha[foot_id] * new_value + 
                             (1.0f - g_lowpass_alpha[foot_id]) * filtered_value[foot_id];
    
    return filtered_value[foot_id];
}

/**
 * @brief  中值滤波器
 */
static float median_filter(FootID_t foot_id, float new_value)
{
    /* 更新缓冲区 */
    g_filter_buffer[foot_id][g_filter_index[foot_id]] = new_value;
    g_filter_index[foot_id] = (g_filter_index[foot_id] + 1) % FOOT_FORCE_FILTER_SIZE;
    
    /* 复制到临时数组 */
    float temp[FOOT_FORCE_FILTER_SIZE];
    memcpy(temp, g_filter_buffer[foot_id], sizeof(temp));
    
    /* 冒泡排序 */
    for (uint8_t i = 0; i < FOOT_FORCE_FILTER_SIZE - 1; i++) {
        for (uint8_t j = 0; j < FOOT_FORCE_FILTER_SIZE - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    /* 返回中值 */
    return temp[FOOT_FORCE_FILTER_SIZE / 2];
}

/**
 * @brief  组合滤波器（中值+低通）
 */
static float combined_filter(FootID_t foot_id, float new_value)
{
    /* 先中值滤波去除尖峰 */
    float median_filtered = median_filter(foot_id, new_value);
    /* 再低通滤波平滑 */
    float lowpass_filtered = low_pass_filter(foot_id, median_filtered);
    
    return lowpass_filtered;
}

/* ============================================================================ 
 * 公共函数实现 - 高级功能扩展
 * ============================================================================ */

/**
 * @brief  设置滤波器类型
 */
void foot_force_set_filter_type(FootID_t foot_id, FilterType_t filter_type)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return;
    }
    
    g_filter_type[foot_id] = filter_type;
    LOG_INFO("[FootForce] %s filter set to %d\r\n", 
             foot_force_get_name(foot_id), filter_type);
}

/**
 * @brief  设置低通滤波器参数
 */
void foot_force_set_lowpass_params(FootID_t foot_id, float cutoff_freq)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return;
    }
    
    /* 计算alpha参数（基于采样率）*/
    float dt = 1.0f / FOOT_FORCE_SAMPLE_RATE_HZ;
    float tau = 1.0f / (2.0f * 3.14159f * cutoff_freq);
    g_lowpass_alpha[foot_id] = dt / (tau + dt);
    
    LOG_INFO("[FootForce] %s lowpass cutoff set to %.1f Hz (alpha=%.3f)\r\n", 
             foot_force_get_name(foot_id), cutoff_freq, g_lowpass_alpha[foot_id]);
}

/**
 * @brief  获取力数据变化率
 */
float foot_force_get_rate_of_change(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return 0.0f;
    }
    
    return g_force_rate[foot_id];
}

/**
 * @brief  批量更新所有足端力数据
 */
void foot_force_update_all(uint16_t adc_values[4])
{
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        foot_force_update((FootID_t)i, adc_values[i]);
    }
}

/**
 * @brief  获取地面粗糙度估计
 */
float foot_force_estimate_roughness(FootID_t foot_id)
{
    if (foot_id >= FOOT_FORCE_SENSOR_COUNT) {
        return 0.0f;
    }
    
    /* 只有在接触地面时才计算粗糙度 */
    if (!g_foot_data[foot_id].is_contact) {
        return 0.0f;
    }
    
    /* 更新粗糙度缓冲区 */
    g_roughness_buffer[foot_id][g_roughness_index[foot_id]] = fabsf(g_force_rate[foot_id]);
    g_roughness_index[foot_id] = (g_roughness_index[foot_id] + 1) % 20;
    
    /* 计算平均变化率作为粗糙度指标 */
    float sum = 0.0f;
    for (uint8_t i = 0; i < 20; i++) {
        sum += g_roughness_buffer[foot_id][i];
    }
    float avg_rate = sum / 20.0f;
    
    /* 归一化到0-1范围 */
    float roughness = fminf(avg_rate / 1000.0f, 1.0f);
    
    return roughness;
}

/**
 * @brief  计算质心（基于足端力分布）
 */
void foot_force_calc_center_of_mass(float *com_x, float *com_y)
{
    /* 足端位置（假设机器人尺寸）*/
    float foot_positions[4][2] = {
        {-0.2f,  0.15f},  /* 前左 */
        { 0.2f,  0.15f},  /* 前右 */
        {-0.2f, -0.15f},  /* 后左 */
        { 0.2f, -0.15f}   /* 后右 */
    };
    
    float total_force = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    
    for (uint8_t i = 0; i < FOOT_FORCE_SENSOR_COUNT; i++) {
        float force = g_foot_data[i].force_filtered;
        total_force += force;
        sum_x += force * foot_positions[i][0];
        sum_y += force * foot_positions[i][1];
    }
    
    if (total_force > 0.0f) {
        *com_x = sum_x / total_force;
        *com_y = sum_y / total_force;
    } else {
        *com_x = 0.0f;
        *com_y = 0.0f;
    }
}
