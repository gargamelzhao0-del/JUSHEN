/**
 ******************************************************************************
 * @file    system_config.h
 * @brief   系统统一配置头文件 - 集中管理所有魔数和参数
 * @version V1.0
 * @note    工业级代码优化：消除魔数，提高可维护性
 ******************************************************************************
 */

#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <float.h>

/* ============================================================================
 * 系统基础配置
 * ============================================================================ */

#define SYSTEM_CONTROL_FREQUENCY_HZ      200     /* 控制循环频率 */
#define SYSTEM_CONTROL_PERIOD_MS         5       /* 控制周期 */
#define SYSTEM_MAX_MOTORS                12      /* 最大电机数量 */
#define SYSTEM_MAX_CAN_CHANNELS          2       /* CAN通道数量 */

/* ============================================================================
 * 电机控制配置
 * ============================================================================ */

/* 电机ID范围 */
#define MOTOR_ID_MIN                     0x01
#define MOTOR_ID_MAX                     0x7F
#define MOTOR_ID_COUNT                   128

/* 电机状态超时 */
#ifndef MOTOR_STATUS_TIMEOUT_MS
#define MOTOR_STATUS_TIMEOUT_MS          500     /* 电机状态超时时间 (从200ms调整为500ms) */
#endif
#define MOTOR_RX_QUEUE_SIZE              64      /* 电机接收队列大小 */

/* 电机错误恢复配置 */
#define MOTOR_ERROR_RECOVERY_DELAY_MS    100     /* 电机错误恢复延迟 */
#define MOTOR_MAX_RETRY_COUNT            3       /* 最大重试次数 */

/* 电机错误码定义 */
typedef enum {
    MOTOR_ERR_OK                = 0,
    MOTOR_ERR_INVALID_PARAM     = 1,
    MOTOR_ERR_INVALID_ID        = 2,
    MOTOR_ERR_TIMEOUT           = 3,
    MOTOR_ERR_CAN_FAILURE       = 4,
    MOTOR_ERR_NOT_ENABLED       = 5,
    MOTOR_ERR_NOT_INITIALIZED   = 6,
    MOTOR_ERR_QUEUE_FULL        = 7,
    MOTOR_ERR_INVALID_MODE      = 8
} MotorErrorCode_t;

/* ============================================================================
 * CAN-FD通信配置
 * ============================================================================ */

/* CAN波特率配置 */
#define CAN_ARB_BAUDRATE_BPS             1000000 /* 仲裁段波特率 1Mbps */
#define CAN_DATA_BAUDRATE_BPS            4000000 /* 数据段波特率 4Mbps */
#define CAN_CLOCK_HZ                     80000000/* CAN时钟 80MHz */

/* CAN发送配置 */
#define CAN_SEND_TIMEOUT_MS              10      /* CAN发送超时 */
#define CAN_SEND_RETRY_COUNT             3       /* CAN发送重试次数 */
#define CAN_TX_FIFO_SIZE                 8       /* 发送FIFO大小 */

/* CAN发送优化阈值 */
#define CAN_SEND_POS_THRESHOLD_DEG       0.12f   /* 位置变化阈值 (度) */
#define CAN_SEND_VEL_THRESHOLD_RPM       1.8f    /* 速度变化阈值 */
#define CAN_SEND_MAX_INTERVAL_MS         8       /* 最大发送间隔 */
#define CAN_SEND_ACCEL_INTERVAL_MS       3       /* 加速阶段发送间隔 */
#define CAN_SEND_DECEL_INTERVAL_MS       3       /* 减速阶段发送间隔 */
#define CAN_SEND_SKIP_MAX                1       /* 允许最多跳过帧数 */

/* CAN监测请求配置 */
#define CAN_MONITOR_REQUEST_INTERVAL_MS  8       /* 监测请求间隔 */
#define CAN_MONITOR_STAGGER_OFFSET_MS    1       /* 电机间错开发送间隔 */

/* CAN Message RAM配置 */
#define CAN1_MESSAGE_RAM_OFFSET          0
#define CAN2_MESSAGE_RAM_OFFSET          1280    /* FDCAN2偏移量，避免与FDCAN1冲突 */
#define CAN_STD_FILTERS_NBR              28
#define CAN_EXT_FILTERS_NBR              8
#define CAN_RX_FIFO0_ELMTS_NBR           20
#define CAN_TX_FIFO_ELMTS_NBR            10

/* ============================================================================
 * 闭环控制配置
 * ============================================================================ */

/* 闭环控制总开关
 * 0 = 开环模式 (CAN-FD分析仪测试时使用，不发0x31F监测指令，不做位置修正)
 * 1 = 闭环模式 (正常电机运行，完整反馈闭环)
 */
#ifndef CLOSED_LOOP_ENABLED
#define CLOSED_LOOP_ENABLED              0
#endif

/* 闭环控制参数 */
#define CLOSED_LOOP_UPDATE_INTERVAL      1       /* 闭环修正间隔 */
#define CLOSED_LOOP_ERROR_THRESHOLD_DEG  2.0f    /* 闭环误差阈值 (度) */
#define CLOSED_LOOP_ERROR_DEADZONE_DEG   0.3f    /* 闭环误差死区 (度) */
#define CLOSED_LOOP_COMPENSATION_MAX_DEG 4.0f    /* 闭环补偿量最大值 (度) */
#define CLOSED_LOOP_COMPENSATION_DECAY   0.98f   /* 闭环补偿量衰减因子 */

/* 电机启动回零配置 */
#define MOTOR_DEFAULT_ANGLE_DEG        180.0f  /* 默认站立角度 (度) */
#define MOTOR_HOMING_DURATION_MS       3000    /* 回零运动时间 (ms) */
#define MOTOR_HOMING_READ_TIMEOUT_MS   100     /* 读取当前位置超时 (ms) */

/* 关节差异化修正因子 */
#define CORRECTION_FACTOR_THIGH_LARGE    0.50f   /* THIGH误差>25度 */
#define CORRECTION_FACTOR_THIGH_MEDIUM   0.35f   /* THIGH误差15-25度 */
#define CORRECTION_FACTOR_THIGH_SMALL    0.25f   /* THIGH误差<15度 */
#define CORRECTION_FACTOR_LARGE          0.40f   /* 其他关节误差>25度 */
#define CORRECTION_FACTOR_MEDIUM         0.28f   /* 其他关节误差15-25度 */
#define CORRECTION_FACTOR_SMALL          0.20f   /* 其他关节误差<15度 */

/* 前馈控制参数 */
#define VELOCITY_FF_WEIGHT               0.5f    /* 速度前馈权重 */
#define TORQUE_COMPENSATION_GAIN         0.05f   /* 转矩补偿增益 */
#define FRICTION_COMPENSATION_GAIN       0.03f   /* 摩擦补偿增益 */
#define PREDICTION_FACTOR                0.15f   /* 预测修正权重 */

/* ============================================================================
 * 轨迹规划器配置
 * ============================================================================ */

/* 轨迹规划速度限制 */
#define TRAJECTORY_EXTREME_POINT_VMAX    150.0f  /* 极值点附近最大速度 (deg/s) */
#define TRAJECTORY_BRAKING_SAFETY_FACTOR 1.5f    /* 减速距离安全系数 */
#define TRAJECTORY_LOW_VEL_THRESHOLD     50.0f   /* 低速阈值 (deg/s) */
#define TRAJECTORY_HIGH_VEL_THRESHOLD    200.0f  /* 高速阈值 (deg/s) */
#define VELOCITY_ERROR_THRESHOLD_RPM     50.0f   /* 速度误差阈值 */

/* ============================================================================
 * 轨迹规划配置
 * ============================================================================ */

/* 默认运动限制 */
#define TRAJECTORY_DEFAULT_VMAX          300.0f  /* 默认最大速度 */
#define TRAJECTORY_DEFAULT_AMAX          1000.0f /* 默认最大加速度 (deg/s^2) */
#define TRAJECTORY_DEFAULT_DMAX          1000.0f /* 默认最大减速度 (deg/s^2) */
#define TRAJECTORY_JERK_LIMIT            5000.0f /* 加加速度限制 (deg/s^3) */
#define TRAJECTORY_SMOOTH_FACTOR         0.8f    /* 平滑因子 */
#define TRAJECTORY_POS_THRESHOLD         0.1f    /* 位置阈值 (度) */
#define TRAJECTORY_VEL_THRESHOLD         1.0f    /* 速度阈值 */

/* 轨迹规划器配置 */
#define TRAJECTORY_MAX_MOTORS            12      /* 最大电机数量 */
#define TRAJECTORY_HISTORY_SIZE          8       /* 历史数据缓冲区大小 */
#define TRAJECTORY_SPLINE_BUFFER_SIZE    4       /* 样条插值缓冲区大小 */

/* 数值稳定性阈值 */
#define TRAJECTORY_DENOM_EPSILON         1e-6f   /* 分母最小值阈值 */
#define TRAJECTORY_POS_EPSILON           1e-4f   /* 位置最小变化阈值 */

/* 前馈滤波器配置 */
#define FF_FILTER_ALPHA                  0.7f    /* 前馈低通滤波系数 */
#define ADAPTIVE_FF_DISTANCE_THRESHOLD   55.0f   /* 极值点距离阈值 (deg) */
#define ADAPTIVE_FF_REDUCTION_FACTOR     0.30f   /* 极值点前馈降低因子 (70%降低) */

/* 控制质量升降级配置 */
#define CONTROL_UPGRADE_STABLE_COUNT     10      /* 升级所需稳定计数 */

/* ============================================================================
 * TCP通信配置
 * ============================================================================ */

/* ⚠️ TCP功能总开关 - 禁用时不初始化LwIP和TCP相关任务 */
/* 设置为0时禁用所有TCP/以太网功能，用于硬件测试 */
#ifndef TCP_ENABLED
#define TCP_ENABLED                      0       /* 0=禁用TCP, 1=启用TCP */
#endif

/* TCP连接配置 - 统一使用9999端口 */
#define TCP_SERVER_PORT                  9999    /* TCP服务器端口 (与上位机config.py一致) */
#define TCP_MAX_CLIENTS                  2       /* 最大客户端数量 */
#define TCP_RX_BUFFER_SIZE               1024    /* 接收缓冲区大小 */
#define TCP_TX_BUFFER_SIZE               1024    /* 发送缓冲区大小 */

/* TCP超时配置 - V4.0: 工业级优化，延长超时时间提高容错性 */
#define TCP_CONNECT_TIMEOUT_MS           10000   /* 连接超时 (10秒) */
#define TCP_RX_TIMEOUT_MS                3000    /* 接收超时警告 (3秒) */
#define TCP_HOLD_TIMEOUT_MS              5000    /* 超时保持阶段 (5秒) */
#define TCP_EMERGENCY_TIMEOUT_MS         10000   /* 超时紧急停止阶段 (10秒) */
#define TCP_STARTUP_GRACE_PERIOD_MS      8000    /* 启动保护期 (5秒内不触发急停) */

/* TCP心跳配置 - V4.0: 更宽容的心跳策略 */
#define TCP_HEARTBEAT_INTERVAL_MS        1000    /* 心跳发送间隔 (1秒) */
#define TCP_HEARTBEAT_TIMEOUT_MS         5000    /* 心跳超时时间 (5秒) */
#define TCP_HEARTBEAT_MAX_MISSED         5       /* 最大丢失心跳数 (5次) */
#define TCP_HEARTBEAT_CHECK_MIN_MS       500     /* 心跳检测最小间隔 */

/* TCP无外设模式配置 - V4.0: 支持无外设测试 */
#define TCP_NO_PERIPHERAL_MODE           1       /* 启用无外设模式 (0=禁用, 1=启用) */
#define TCP_NO_PERIPHERAL_IGNORE_ESTOP   1       /* 无外设模式下忽略TCP超时急停 */

/* TCP丢包检测 */
#define TCP_PACKET_LOSS_THRESHOLD        5       /* 连续丢包阈值 */
#define TCP_PACKET_LOSS_WINDOW_MS        200     /* 丢包检测窗口 */
#define TCP_PACKET_LOSS_WARN_INTERVAL_MS 5000    /* 丢包警告间隔 */

/* TCP软启动配置 */
#define TCP_SOFT_START_DURATION_MS       1000    /* 软启动持续时间 */
#define TCP_HOLD_DURATION_MS             500     /* 保持阶段持续时间 */

/* ============================================================================
 * 数据缓冲配置
 * ============================================================================ */

/* 双缓冲配置 */
#define TARGET_BUFFER_SIZE               2       /* 双缓冲大小 */
#define INTERPOLATION_STEPS              4       /* 每帧数据插值步数 (200Hz/50Hz) */

/* 插值误差补偿 */
#define INTERP_ERROR_THRESHOLD_DEG       0.5f    /* 误差检测阈值 (度) */
#define INTERP_ERROR_MAX_CORRECTION_DEG  2.0f    /* 最大单次修正量 (度) */
#define INTERP_ERROR_DECAY               0.98f   /* 误差衰减因子 */
#define INTERP_SYNC_INTERVAL_MS          100     /* 强制同步间隔 */

/* ============================================================================
 * 安全监控配置
 * ============================================================================ */

/* 看门狗配置
 * 超时层级关系必须满足: IWDG > TaskTimeout > CheckInterval
 * 原配置 IWDG(3s) < TaskTimeout(5s) 导致硬件看门狗先于软件看门狗复位
 * 修复: IWDG=8s > TaskTimeout=5s > CheckInterval=1s
 */
#define WATCHDOG_IWDG_TIMEOUT_MS         8000    /* 硬件看门狗超时 (必须 > TASK_TIMEOUT) */
#define WATCHDOG_TASK_TIMEOUT_MS         5000    /* 任务超时 (软件看门狗检测) */
#define WATCHDOG_CHECK_INTERVAL_MS       1000    /* 检查间隔 */
#define WATCHDOG_EMERGENCY_SAVE_MS       4000    /* 紧急状态保存阈值 (< IWDG超时) */

/* 急停配置 */
#define ESTOP_DEBOUNCE_TIME_MS           50      /* 急停消抖时间 */
#define ESTOP_RECOVERY_DELAY_MS          1000    /* 急停恢复延迟 */

/* ============================================================================
 * 电源管理配置
 * ============================================================================ */

/* 电源阈值 */
#define POWER_VOLTAGE_MIN_V              20.0f   /* 最低电压 */
#define POWER_VOLTAGE_MAX_V              30.0f   /* 最高电压 */
#define POWER_CURRENT_MAX_A              50.0f   /* 最大电流 */
#define POWER_TEMP_MAX_C                 60.0f   /* 最高温度 */

/* BMS配置 */
#define BMS_I2C_ADDRESS                  0x0A    /* BMS I2C地址 */
#define BMS_TIMEOUT_MS                   1000    /* BMS通信超时 */
#define BMS_SOC_LOW_THRESHOLD            20      /* 低电量阈值 (%) */
#define BMS_SOC_CRITICAL_THRESHOLD       10      /* 电量严重不足阈值 (%) */

/* ============================================================================
 * 传感器配置
 * ============================================================================ */

/* 足端力传感器 */
#define FOOT_FORCE_SENSOR_COUNT          4       /* 传感器数量 */
#define FOOT_FORCE_FILTER_SIZE           8       /* 滤波器缓冲区大小 */
#define FOOT_FORCE_CONTACT_THRESHOLD_N   5.0f    /* 接触阈值 (N) */
#define FOOT_FORCE_SUPPORT_THRESHOLD_N   20.0f   /* 支撑阈值 (N) */
#define FOOT_FORCE_OVERLOAD_THRESHOLD_N  200.0f  /* 过载阈值 (N) */

/* ADC配置 */
#define ADC_SAMPLE_RATE_HZ               1000    /* ADC采样率 */
#define ADC_RESOLUTION_BITS              12      /* ADC分辨率 */
#define ADC_VREF_MV                      3300    /* ADC参考电压 */

/* ============================================================================
 * 参数存储配置
 * ============================================================================ */

/* FLASH配置 */
#define PARAM_MAGIC_NUMBER               0x4A53484D  /* "JSHM" */
#ifndef PARAM_VERSION
#define PARAM_VERSION                    0x0100      /* 版本号 1.0 */
#endif
#define PARAM_MIN_SAVE_INTERVAL_MS       5000        /* 最小保存间隔 */
#define PARAM_FLASH_SECTOR_SIZE          (128 * 1024) /* FLASH扇区大小 */

/* ============================================================================
 * 日志系统配置
 * ============================================================================ */

/* 日志配置 */
#define LOG_BUFFER_SIZE                  256     /* 日志缓冲区大小 */
#define LOG_MUTEX_TIMEOUT_MS             100     /* 日志互斥锁超时 */
#define LOG_DEFAULT_LEVEL                3       /* 默认日志级别 (INFO=3) */

/* ============================================================================
 * 安全认证配置
 * ============================================================================ */

/* 认证配置 */
#define AUTH_KEY_LENGTH                  16      /* 密钥长度 */
#define AUTH_CHALLENGE_LENGTH            8       /* 挑战码长度 */
#define AUTH_SESSION_TIMEOUT_MS          3600000 /* 会话超时 (1小时) */
#define AUTH_MAX_FAILED_ATTEMPTS         5       /* 最大失败尝试次数 */
#define AUTH_LOCKOUT_TIME_MS             30000   /* 锁定时间 */

/* 数据验证范围 - V15.0: 更新电机参数 */
#define MOTOR_POS_MIN_DEG                0.0f    /* 电机位置最小值 (deg) */
#define MOTOR_POS_MAX_DEG                359.9f  /* 电机位置最大值 (deg) */
#define MOTOR_VEL_MAX_DEG_S              1000.0f /* 电机最大速度 (deg/s) - 绝对值 */
#define MOTOR_VEL_MAX_RPM                167.0f  /* 电机最大速度 (RPM) - 绝对值 (1000/6 ≈ 167) */
#define MOTOR_ACCEL_MAX_DEG_S2           5000.0f /* 电机最大加速度 (deg/s²) - 绝对值 */
#define MOTOR_TORQUE_MAX_NM              300.0f  /* 电机最大扭矩 (Nm) - 峰值300Nm */

/* ============================================================================
 * 数值稳定性宏
 * ============================================================================ */

#ifndef MIN
#define MIN(a, b)    ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)    ((a) > (b) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

#ifndef CLAMP_VALID
#define CLAMP_VALID(value, min_val, max_val, default_val) \
    ((isnan(value) || isinf(value)) ? (default_val) : \
     ((value) < (min_val) ? (min_val) : \
      ((value) > (max_val) ? (max_val) : (value))))
#endif

#ifndef SAFE_FLOAT
#define SAFE_FLOAT(value, default_val) \
    ((isnan(value) || isinf(value)) ? (default_val) : (value))
#endif

#ifndef ABS
#define ABS(x)       ((x) < 0 ? -(x) : (x))
#endif

/* 数值稳定性检查 */
#define IS_VALID_FLOAT(x)     (!isnan(x) && !isinf(x))
#define SAFE_DIVIDE(a, b, default)  ((ABS(b) > FLT_EPSILON) ? ((a) / (b)) : (default))
#define SAFE_SQRT(x)          ((x) > 0 ? sqrtf(x) : 0.0f)

/* ============================================================================
 * 断言和错误处理宏
 * ============================================================================ */

#ifdef DEBUG
#define SYSTEM_ASSERT(expr)   do { if (!(expr)) { system_assert_failed(#expr, __FILE__, __LINE__); } } while(0)
#else
#define SYSTEM_ASSERT(expr)   ((void)0)
#endif

#define RETURN_IF_NULL(ptr)   do { if ((ptr) == NULL) return MOTOR_ERR_INVALID_PARAM; } while(0)
#define RETURN_IF_ERROR(expr) do { int __err = (expr); if (__err != MOTOR_ERR_OK) return __err; } while(0)

/* ============================================================================
 * 函数声明
 * ============================================================================ */

void system_assert_failed(const char* expr, const char* file, int line);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CONFIG_H */
