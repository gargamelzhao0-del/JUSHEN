/**
 ******************************************************************************
 * @file    param_config.h
 * @brief   系统参数配置管理 - FLASH持久化存储
 * @note    支持参数保存、加载、备份、恢复出厂设置
 ******************************************************************************
 */

#ifndef __PARAM_CONFIG_H
#define __PARAM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 配置参数
 * ============================================================================ */

/* FLASH地址配置 */
#define PARAM_FLASH_BANK1_ADDR      0x081E0000  /* 主参数区：Sector 14 (128KB) */
#define PARAM_FLASH_BANK2_ADDR      0x081F0000  /* 备份参数区：Sector 15 (128KB) */
#define PARAM_FLASH_SECTOR_SIZE     (128 * 1024)  /* 128KB */

/* 参数魔数和版本 */
#define PARAM_MAGIC                 0x50415241  /* "PARA" */
#ifndef PARAM_VERSION
#define PARAM_VERSION               0x00010000  /* v1.0.0 */
#endif

/* 日志级别枚举 */
typedef enum {
    PARAM_LOG_LEVEL_ERROR = 0,
    PARAM_LOG_LEVEL_WARN,
    PARAM_LOG_LEVEL_INFO,
    PARAM_LOG_LEVEL_DEBUG,
} ParamLogLevel_t;

/* ============================================================================
 * 参数结构定义
 * ============================================================================ */

/**
 * @brief  系统参数
 */
typedef struct {
    uint32_t watchdog_timeout_ms;   /* 看门狗超时时间（毫秒）*/
    uint8_t log_level;              /* 日志级别 */
    uint8_t enable_iwdg;            /* 是否启用IWDG */
    uint8_t reserved[2];            /* 保留字节（对齐）*/
} SystemParam_t;

/**
 * @brief  网络参数
 */
typedef struct {
    uint8_t ip_addr[4];             /* IP地址 */
    uint8_t netmask[4];             /* 子网掩码 */
    uint8_t gateway[4];             /* 网关 */
    uint16_t tcp_port;              /* TCP端口 */
    uint16_t udp_port;              /* UDP端口 */
    uint8_t mac_addr[6];            /* MAC地址 */
    uint8_t use_dhcp;               /* 是否使用DHCP */
    uint8_t reserved[1];            /* 保留字节（对齐）*/
} NetworkParam_t;

/**
 * @brief  电机参数
 */
typedef struct {
    uint16_t control_freq_hz;       /* 控制频率（Hz）*/
    uint8_t motor_id_map[12];       /* 电机ID映射表 */
    float max_velocity[12];         /* 最大速度限制（rad/s）*/
    float max_torque[12];           /* 最大扭矩限制（N·m）*/
    float position_kp[12];          /* 位置控制P增益 */
    float position_kd[12];          /* 位置控制D增益 */
    uint8_t enable_motor_send;      /* 是否启用电机发送 */
    uint8_t reserved[3];            /* 保留字节（对齐）*/
} MotorParam_t;

/**
 * @brief  传感器校准参数
 */
typedef struct {
    float force_zero[4];            /* 足端力传感器零点（4个足端）*/
    float force_scale[4];           /* 足端力传感器比例 */
    float imu_accel_offset[3];      /* IMU加速度零点偏移（x,y,z）*/
    float imu_gyro_offset[3];       /* IMU陀螺仪零点偏移（x,y,z）*/
    float imu_mag_offset[3];        /* IMU磁力计零点偏移（x,y,z）*/
    uint8_t reserved[4];            /* 保留字节（对齐）*/
} SensorParam_t;

/**
 * @brief  完整参数配置结构
 */
typedef struct __attribute__((packed)) {
    /* 头部信息（16字节）*/
    uint32_t magic;                 /* 魔数：0x50415241 ("PARA") */
    uint32_t version;               /* 版本号 */
    uint32_t size;                  /* 参数区大小（字节）*/
    uint32_t crc32;                 /* CRC32校验和 */

    /* 参数数据 */
    SystemParam_t system;           /* 系统参数 */
    NetworkParam_t network;         /* 网络参数 */
    MotorParam_t motor;             /* 电机参数 */
    SensorParam_t sensor;           /* 传感器参数 */

    /* 保留扩展空间（256字节）*/
    uint8_t reserved[256];
} ParamConfig_t;

/* ============================================================================
 * 公共API - 初始化和控制
 * ============================================================================ */

/**
 * @brief  初始化参数系统
 * @retval 0=成功, -1=失败
 * @note   加载FLASH中的参数，如果无效则使用默认值
 */
int param_init(void);

/**
 * @brief  从FLASH加载参数
 * @retval 0=成功, -1=失败
 * @note   优先从Bank1加载，如果失败则从Bank2加载
 */
int param_load(void);

/**
 * @brief  保存参数到FLASH
 * @retval 0=成功, -1=失败
 * @note   同时写入Bank1和Bank2（双备份）
 */
int param_save(void);

/**
 * @brief  恢复出厂设置
 * @retval 0=成功, -1=失败
 * @note   加载默认参数并保存到FLASH
 */
int param_reset_to_default(void);

/* ============================================================================
 * 公共API - 参数读取
 * ============================================================================ */

/**
 * @brief  获取系统参数
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_get_system(SystemParam_t *param);

/**
 * @brief  获取网络参数
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_get_network(NetworkParam_t *param);

/**
 * @brief  获取电机参数
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_get_motor(MotorParam_t *param);

/**
 * @brief  获取传感器参数
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_get_sensor(SensorParam_t *param);

/**
 * @brief  获取完整参数配置
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_get_all(ParamConfig_t *param);

/* ============================================================================
 * 公共API - 参数设置
 * ============================================================================ */

/**
 * @brief  设置系统参数
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 * @note   仅修改内存中的参数，需要调用param_save()保存
 */
int param_set_system(const SystemParam_t *param);

/**
 * @brief  设置网络参数
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 */
int param_set_network(const NetworkParam_t *param);

/**
 * @brief  设置电机参数
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 */
int param_set_motor(const MotorParam_t *param);

/**
 * @brief  设置传感器参数
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 */
int param_set_sensor(const SensorParam_t *param);

/**
 * @brief  设置完整参数配置
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 */
int param_set_all(const ParamConfig_t *param);

/* ============================================================================
 * 公共API - 工具函数
 * ============================================================================ */

/**
 * @brief  打印当前参数配置
 */
void param_print(void);

/**
 * @brief  检查参数是否有效
 * @retval true=有效, false=无效
 */
bool param_is_valid(void);

/**
 * @brief  获取参数版本号
 * @retval 版本号（格式：0xMMmmpppp，MM=主版本，mm=次版本，pppp=补丁）
 */
uint32_t param_get_version(void);

/**
 * @brief  获取参数大小
 * @retval 参数结构大小（字节）
 */
uint32_t param_get_size(void);

/**
 * @brief  计算参数CRC32
 * @retval CRC32校验值
 */
uint32_t param_calculate_crc(void);

/**
 * @brief  验证参数CRC32
 * @retval true=校验通过, false=校验失败
 */
bool param_verify_crc(void);

/* ============================================================================
 * 公共API - FLASH操作（底层接口）
 * ============================================================================ */

/**
 * @brief  擦除参数FLASH扇区
 * @param  bank: 1=Bank1, 2=Bank2
 * @retval 0=成功, -1=失败
 */
int param_flash_erase(uint8_t bank);

/**
 * @brief  从FLASH读取参数
 * @param  bank: 1=Bank1, 2=Bank2
 * @param  param: 输出参数结构
 * @retval 0=成功, -1=失败
 */
int param_flash_read(uint8_t bank, ParamConfig_t *param);

/**
 * @brief  写入参数到FLASH
 * @param  bank: 1=Bank1, 2=Bank2
 * @param  param: 输入参数结构
 * @retval 0=成功, -1=失败
 */
int param_flash_write(uint8_t bank, const ParamConfig_t *param);

/* ============================================================================
 * 默认参数值定义
 * ============================================================================ */

/* 系统默认参数 */
#define PARAM_DEFAULT_WATCHDOG_TIMEOUT_MS   3000
#define PARAM_DEFAULT_LOG_LEVEL             PARAM_LOG_LEVEL_INFO
#define PARAM_DEFAULT_ENABLE_IWDG           1

/* 网络默认参数 */
#define PARAM_DEFAULT_IP_ADDR               {192, 168, 1, 100}
#define PARAM_DEFAULT_NETMASK               {255, 255, 255, 0}
#define PARAM_DEFAULT_GATEWAY               {192, 168, 1, 1}
#define PARAM_DEFAULT_TCP_PORT              9999
#define PARAM_DEFAULT_UDP_PORT              8888
#define PARAM_DEFAULT_USE_DHCP              0

/* 电机默认参数 */
#define PARAM_DEFAULT_CONTROL_FREQ_HZ       10
#define PARAM_DEFAULT_MAX_VELOCITY          5.0f    /* rad/s */
#define PARAM_DEFAULT_MAX_TORQUE            10.0f   /* N·m */
#define PARAM_DEFAULT_POSITION_KP           1.0f
#define PARAM_DEFAULT_POSITION_KD           0.1f
#define PARAM_DEFAULT_ENABLE_MOTOR_SEND     1

/* 传感器默认参数 */
#define PARAM_DEFAULT_FORCE_ZERO            0.0f
#define PARAM_DEFAULT_FORCE_SCALE           1.0f
#define PARAM_DEFAULT_IMU_OFFSET            0.0f

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_CONFIG_H */
