/**
 ******************************************************************************
 * @file    power_management.c
 * @brief   电源管理模块实现（通用框架）
 * @note    需根据具体BMS芯片适配寄存器地址
 ******************************************************************************
 */

#include "power_management.h"
#include "i2c.h"
#include "log_system.h"
#include "robot_state.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* I2C配置 */
static I2C_HandleTypeDef *g_hi2c = NULL;
static uint8_t g_device_addr = 0x00;

/* 电源数据缓存 */
static PowerData_t g_power_data = {0};

/* 监控任务 */
static osThreadId_t power_monitor_task_handle = NULL;
static uint32_t g_update_interval_ms = 1000;

/* 回调函数 */
static void (*g_low_voltage_callback)(float) = NULL;
static void (*g_overheat_callback)(float) = NULL;

/* 互斥锁 */
static osMutexId_t power_mutex = NULL;

/* ============================================================================
 * 私有函数原型
 * ============================================================================ */

static int power_i2c_read_register(uint8_t reg_addr, uint8_t *data, uint16_t length);
static int power_i2c_write_register(uint8_t reg_addr, uint8_t *data, uint16_t length);
static void power_monitor_task(void *argument);
static void power_check_alarms(const PowerData_t *data);

/* ============================================================================
 * BMS寄存器定义 - V2.0: 可配置寄存器地址
 * ============================================================================
 * ⚠️ 重要提示：以下寄存器地址需要根据实际使用的BMS芯片进行配置！
 * 
 * 常见BMS芯片寄存器地址参考：
 * ┌─────────────────┬────────────┬────────────┬────────────┐
 * │ 参数            │ TI BQ40Z50 │ TI BQ34Z100│ 默认值     │
 * ├─────────────────┼────────────┼────────────┼────────────┤
 * │ 电压高位        │ 0x08       │ 0x08       │ 0x08       │
 * │ 电压低位        │ 0x09       │ 0x09       │ 0x09       │
 * │ 电流高位        │ 0x0A       │ 0x0A       │ 0x0A       │
 * │ 电流低位        │ 0x0B       │ 0x0B       │ 0x0B       │
 * │ 温度            │ 0x0C       │ 0x0C       │ 0x0C       │
 * │ 容量            │ 0x0D       │ 0x0D       │ 0x0D       │
 * │ 状态            │ 0x0E       │ 0x0E       │ 0x0E       │
 * │ SOC             │ 0x0F       │ 0x0F       │ 0x0F       │
 * │ SOH             │ 0x10       │ 0x10       │ 0x10       │
 * └─────────────────┴────────────┴────────────┴────────────┘
 * 
 * 使用方法：
 * 1. 确认实际使用的BMS芯片型号
 * 2. 查阅芯片数据手册获取寄存器地址
 * 3. 在power_init()之前调用power_set_bms_config()配置
 * 
 * 注意：如果BMS未连接或地址错误，系统会输出警告但不会阻止启动
 */

static BMSRegisterConfig_t g_bms_config = {
    .reg_voltage_h = 0x08,
    .reg_voltage_l = 0x09,
    .reg_current_h = 0x0A,
    .reg_current_l = 0x0B,
    .reg_temp = 0x0C,
    .reg_capacity = 0x0D,
    .reg_status = 0x0E,
    .reg_soc = 0x0F,
    .reg_soh = 0x10
};

#define BMS_REG_VOLTAGE_H       g_bms_config.reg_voltage_h
#define BMS_REG_VOLTAGE_L       g_bms_config.reg_voltage_l
#define BMS_REG_CURRENT_H       g_bms_config.reg_current_h
#define BMS_REG_CURRENT_L       g_bms_config.reg_current_l
#define BMS_REG_TEMP            g_bms_config.reg_temp
#define BMS_REG_CAPACITY        g_bms_config.reg_capacity
#define BMS_REG_STATUS          g_bms_config.reg_status
#define BMS_REG_SOC             g_bms_config.reg_soc
#define BMS_REG_SOH             g_bms_config.reg_soh

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief  初始化电源管理模块
 */
int power_init(void *i2c_handle, uint8_t device_addr)
{
    if (i2c_handle == NULL) {
        LOG_ERROR("[Power] Invalid I2C handle");
        return -1;
    }

    g_hi2c = (I2C_HandleTypeDef *)i2c_handle;
    g_device_addr = device_addr << 1;  /* 转换为8位地址 */

    /* 创建互斥锁 */
    const osMutexAttr_t mutex_attr = {
        .name = "PowerMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    power_mutex = osMutexNew(&mutex_attr);
    if (power_mutex == NULL) {
        LOG_ERROR("[Power] Failed to create mutex");
        return -1;
    }

    /* 初始化电源数据 */
    memset(&g_power_data, 0, sizeof(PowerData_t));
    g_power_data.status = POWER_STATUS_NORMAL;
    g_power_data.is_connected = false;

    /* 测试I2C通信 */
    if (HAL_I2C_IsDeviceReady(g_hi2c, g_device_addr, 3, POWER_I2C_TIMEOUT_MS) == HAL_OK) {
        g_power_data.is_connected = true;
        LOG_INFO("[Power] BMS connected at I2C addr 0x%02X", device_addr);
    } else {
        LOG_WARN("[Power] BMS not responding (addr 0x%02X)", device_addr);
        /* 不报错，允许系统继续运行 */
    }

    LOG_INFO("[Power] Power management initialized");
    return 0;
}

/**
 * @brief  读取电源数据
 */
int power_read_data(PowerData_t *data)
{
    if (g_hi2c == NULL) {
        LOG_ERROR("[Power] Power management not initialized");
        return -1;
    }

    osMutexAcquire(power_mutex, osWaitForever);

    uint8_t buffer[2];
    int ret = 0;

    /* 读取电压（16位，单位：10mV）*/
    if (power_i2c_read_register(BMS_REG_VOLTAGE_H, buffer, 2) == 0) {
        uint16_t voltage_raw = ((uint16_t)buffer[0] << 8) | buffer[1];
        g_power_data.voltage = voltage_raw * 0.01f;  /* 转换为V */
    } else {
        ret = -1;
    }

    /* 读取电流（16位，单位：10mA，有符号）*/
    if (power_i2c_read_register(BMS_REG_CURRENT_H, buffer, 2) == 0) {
        int16_t current_raw = ((int16_t)buffer[0] << 8) | buffer[1];
        g_power_data.current = current_raw * 0.01f;  /* 转换为A */
    } else {
        ret = -1;
    }

    /* 读取温度（8位，单位：°C）*/
    if (power_i2c_read_register(BMS_REG_TEMP, buffer, 1) == 0) {
        g_power_data.temperature = (float)buffer[0];
    } else {
        ret = -1;
    }

    /* 读取剩余容量（8位，单位：%）*/
    if (power_i2c_read_register(BMS_REG_CAPACITY, buffer, 1) == 0) {
        g_power_data.capacity = (float)buffer[0];
    } else {
        ret = -1;
    }

    /* 读取状态寄存器 */
    if (power_i2c_read_register(BMS_REG_STATUS, buffer, 1) == 0) {
        /* 解析状态位（根据实际BMS芯片定义）*/
        g_power_data.is_charging = (buffer[0] & 0x01) ? true : false;
        g_power_data.is_connected = true;
    } else {
        g_power_data.is_connected = false;
        ret = -1;
    }

    /* 更新状态 */
    if (ret == 0) {
        g_power_data.last_update_ms = HAL_GetTick();

        /* 判断电源状态 */
        if (g_power_data.voltage < POWER_VOLTAGE_CRITICAL_THR) {
            g_power_data.status = POWER_STATUS_CRITICAL;
        } else if (g_power_data.voltage < POWER_VOLTAGE_LOW_THR) {
            g_power_data.status = POWER_STATUS_LOW_VOLTAGE;
        } else if (g_power_data.temperature > POWER_TEMP_HIGH_THR) {
            g_power_data.status = POWER_STATUS_OVERHEAT;
        } else if (g_power_data.is_charging) {
            g_power_data.status = POWER_STATUS_CHARGING;
        } else {
            g_power_data.status = POWER_STATUS_NORMAL;
        }

        /* 检查告警 */
        power_check_alarms(&g_power_data);
    } else {
        g_power_data.status = POWER_STATUS_ERROR;
    }

    /* 输出数据 */
    if (data != NULL) {
        memcpy(data, &g_power_data, sizeof(PowerData_t));
    }

    osMutexRelease(power_mutex);

    return ret;
}

/**
 * @brief  获取电池电压
 */
float power_get_voltage(void)
{
    if (power_mutex != NULL) {
        osMutexAcquire(power_mutex, osWaitForever);
    }
    float voltage = g_power_data.voltage;
    if (power_mutex != NULL) {
        osMutexRelease(power_mutex);
    }
    return voltage;
}

/**
 * @brief  获取电池电流
 */
float power_get_current(void)
{
    if (power_mutex != NULL) {
        osMutexAcquire(power_mutex, osWaitForever);
    }
    float current = g_power_data.current;
    if (power_mutex != NULL) {
        osMutexRelease(power_mutex);
    }
    return current;
}

/**
 * @brief  获取电池温度
 */
float power_get_temperature(void)
{
    if (power_mutex != NULL) {
        osMutexAcquire(power_mutex, osWaitForever);
    }
    float temperature = g_power_data.temperature;
    if (power_mutex != NULL) {
        osMutexRelease(power_mutex);
    }
    return temperature;
}

/**
 * @brief  获取剩余容量百分比
 */
float power_get_capacity(void)
{
    if (power_mutex != NULL) {
        osMutexAcquire(power_mutex, osWaitForever);
    }
    float capacity = g_power_data.capacity;
    if (power_mutex != NULL) {
        osMutexRelease(power_mutex);
    }
    return capacity;
}

/**
 * @brief  获取电源状态
 */
PowerStatus_t power_get_status(void)
{
    return g_power_data.status;
}

/**
 * @brief  检查是否在充电
 */
bool power_is_charging(void)
{
    return g_power_data.is_charging;
}

/**
 * @brief  检查BMS是否连接
 */
bool power_is_connected(void)
{
    return g_power_data.is_connected;
}

/**
 * @brief  设置低电压告警回调
 */
void power_set_low_voltage_callback(void (*callback)(float voltage))
{
    g_low_voltage_callback = callback;
}

/**
 * @brief  设置过热告警回调
 */
void power_set_overheat_callback(void (*callback)(float temperature))
{
    g_overheat_callback = callback;
}

/**
 * @brief  启动电源监控任务
 */
int power_start_monitor_task(uint32_t update_interval_ms)
{
    if (power_monitor_task_handle != NULL) {
        LOG_WARN("[Power] Monitor task already running");
        return 0;
    }

    g_update_interval_ms = update_interval_ms;

    const osThreadAttr_t task_attr = {
        .name = "PowerMonitor",
        .stack_size = 1024 * 2,
        .priority = osPriorityNormal,
    };

    power_monitor_task_handle = osThreadNew(power_monitor_task, NULL, &task_attr);

    if (power_monitor_task_handle == NULL) {
        LOG_ERROR("[Power] Failed to create monitor task");
        return -1;
    }

    LOG_INFO("[Power] Monitor task started (interval=%lu ms)", update_interval_ms);
    return 0;
}

/**
 * @brief  停止电源监控任务
 */
void power_stop_monitor_task(void)
{
    if (power_monitor_task_handle != NULL) {
        osThreadTerminate(power_monitor_task_handle);
        power_monitor_task_handle = NULL;
        LOG_INFO("[Power] Monitor task stopped");
    }
}

/**
 * @brief  打印电源状态
 */
void power_print_status(void)
{
    osMutexAcquire(power_mutex, osWaitForever);

    LOG_INFO("========== Power Status ==========");
    LOG_INFO("  Voltage:     %.2f V", g_power_data.voltage);
    LOG_INFO("  Current:     %.2f A", g_power_data.current);
    LOG_INFO("  Temperature: %.1f °C", g_power_data.temperature);
    LOG_INFO("  Capacity:    %.0f %%", g_power_data.capacity);
    LOG_INFO("  Status:      %s", power_get_status_name(g_power_data.status));
    LOG_INFO("  Charging:    %s", g_power_data.is_charging ? "Yes" : "No");
    LOG_INFO("  Connected:   %s", g_power_data.is_connected ? "Yes" : "No");
    LOG_INFO("==================================");

    osMutexRelease(power_mutex);
}

/**
 * @brief  获取电源状态名称字符串
 */
const char* power_get_status_name(PowerStatus_t status)
{
    switch (status) {
        case POWER_STATUS_NORMAL:       return "Normal";
        case POWER_STATUS_LOW_VOLTAGE:  return "Low Voltage";
        case POWER_STATUS_CRITICAL:     return "Critical!";
        case POWER_STATUS_OVERHEAT:     return "Overheat";
        case POWER_STATUS_CHARGING:     return "Charging";
        case POWER_STATUS_ERROR:        return "Error";
        default:                        return "Unknown";
    }
}

int power_configure_bms_registers(const BMSRegisterConfig_t* config)
{
    if (config == NULL) {
        LOG_ERROR("[Power] Invalid BMS config pointer");
        return -1;
    }
    
    if (power_mutex) osMutexAcquire(power_mutex, osWaitForever);
    
    g_bms_config.reg_voltage_h = config->reg_voltage_h;
    g_bms_config.reg_voltage_l = config->reg_voltage_l;
    g_bms_config.reg_current_h = config->reg_current_h;
    g_bms_config.reg_current_l = config->reg_current_l;
    g_bms_config.reg_temp = config->reg_temp;
    g_bms_config.reg_capacity = config->reg_capacity;
    g_bms_config.reg_status = config->reg_status;
    g_bms_config.reg_soc = config->reg_soc;
    g_bms_config.reg_soh = config->reg_soh;
    
    if (power_mutex) osMutexRelease(power_mutex);
    
    LOG_INFO("[Power] BMS registers configured: V=0x%02X%02X, I=0x%02X%02X, T=0x%02X",
             config->reg_voltage_h, config->reg_voltage_l,
             config->reg_current_h, config->reg_current_l,
             config->reg_temp);
    
    return 0;
}

int power_get_bms_config(BMSRegisterConfig_t* config)
{
    if (config == NULL) {
        return -1;
    }
    
    if (power_mutex) osMutexAcquire(power_mutex, osWaitForever);
    
    config->reg_voltage_h = g_bms_config.reg_voltage_h;
    config->reg_voltage_l = g_bms_config.reg_voltage_l;
    config->reg_current_h = g_bms_config.reg_current_h;
    config->reg_current_l = g_bms_config.reg_current_l;
    config->reg_temp = g_bms_config.reg_temp;
    config->reg_capacity = g_bms_config.reg_capacity;
    config->reg_status = g_bms_config.reg_status;
    config->reg_soc = g_bms_config.reg_soc;
    config->reg_soh = g_bms_config.reg_soh;
    
    if (power_mutex) osMutexRelease(power_mutex);
    
    return 0;
}

/**
 * @brief  估算剩余运行时间
 */
uint32_t power_estimate_runtime(float average_power_w)
{
    if (average_power_w <= 0.0f || g_power_data.capacity <= 0.0f) {
        return 0;
    }

    /* 假设电池总容量为5000mAh @ 24V（可根据实际电池修改）*/
    float battery_capacity_wh = 5.0f * 24.0f;  /* 120Wh */
    float remaining_energy_wh = battery_capacity_wh * (g_power_data.capacity / 100.0f);

    /* 剩余时间（分钟）= 剩余能量 / 平均功率 × 60 */
    uint32_t runtime_min = (uint32_t)((remaining_energy_wh / average_power_w) * 60.0f);

    return runtime_min;
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  I2C读取寄存器
 */
static int power_i2c_read_register(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    if (g_hi2c == NULL || data == NULL) {
        return -1;
    }

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        g_hi2c,
        g_device_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        POWER_I2C_TIMEOUT_MS
    );

    return (ret == HAL_OK) ? 0 : -1;
}

/**
 * @brief  I2C写入寄存器
 */
static int power_i2c_write_register(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    if (g_hi2c == NULL || data == NULL) {
        return -1;
    }

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        g_hi2c,
        g_device_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        POWER_I2C_TIMEOUT_MS
    );

    return (ret == HAL_OK) ? 0 : -1;
}

/**
 * @brief  电源监控任务
 */
static void power_monitor_task(void *argument)
{
    (void)argument;

    LOG_INFO("[Power] Monitor task running");

    osDelay(2000);  /* 等待系统初始化 */

    for (;;)
    {
        /* 读取电源数据 */
        power_read_data(NULL);

        /* 每10秒打印一次状态 */
        static uint32_t print_counter = 0;
        if (++print_counter >= (10000 / g_update_interval_ms)) {
            power_print_status();
            print_counter = 0;
        }

        osDelay(g_update_interval_ms);
    }
}

/**
 * @brief  检查告警
 */
static void power_check_alarms(const PowerData_t *data)
{
    /* 低电压告警 */
    if (data->voltage < POWER_VOLTAGE_LOW_THR) {
        if (g_low_voltage_callback != NULL) {
            g_low_voltage_callback(data->voltage);
        }

        /* 设置系统错误标志 */
        if (data->voltage < POWER_VOLTAGE_CRITICAL_THR) {
            robot_state_set_error(ERROR_VOLTAGE_FAULT);
            LOG_ERROR("[Power] Critical low voltage: %.2f V", data->voltage);
        } else {
            LOG_WARN("[Power] Low voltage: %.2f V", data->voltage);
        }
    }

    /* 过热告警 */
    if (data->temperature > POWER_TEMP_HIGH_THR) {
        if (g_overheat_callback != NULL) {
            g_overheat_callback(data->temperature);
        }

        robot_state_set_error(ERROR_OVERHEAT);
        LOG_ERROR("[Power] Battery overheat: %.1f °C", data->temperature);
    }
}
