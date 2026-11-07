/**
 ******************************************************************************
 * @file    protocol_codec.h
 * @brief   二进制通信协议编解码模块
 * @note    用于STM32下位机与Jetson上位机高效通信
 ******************************************************************************
 */

#ifndef __PROTOCOL_CODEC_H
#define __PROTOCOL_CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 协议配置
 * ============================================================================ */

#define PROTO_HEADER_0           0xAA    /* 帧头字节1 */
#define PROTO_HEADER_1           0x55    /* 帧头字节2 */
#define PROTO_FOOTER_0           0x0D    /* 帧尾字节1 */
#define PROTO_FOOTER_1           0x0A    /* 帧尾字节2 */

#define PROTO_MAX_PAYLOAD_SIZE   512     /* 最大负载长度（字节）*/
#define PROTO_MAX_FRAME_SIZE     (PROTO_MAX_PAYLOAD_SIZE + 16)  /* 最大帧长度 */

/**
 * @brief 命令类型（上位机 → 下位机）
 */
typedef enum {
    /* 系统控制命令 (0x00-0x0F) */
    CMD_SYSTEM_RESET        = 0x00,  /* 系统复位 */
    CMD_SYSTEM_HEARTBEAT    = 0x01,  /* 心跳包 */
    CMD_SYSTEM_STATUS_REQ   = 0x02,  /* 查询系统状态 */
    CMD_SYSTEM_ENTER_IDLE   = 0x03,  /* 进入IDLE状态 */
    CMD_SYSTEM_ENTER_READY  = 0x04,  /* 进入READY状态 */
    CMD_SYSTEM_EMERGENCY    = 0x0F,  /* 紧急停止 */

    /* 电机控制命令 (0x10-0x2F) */
    CMD_MOTOR_ENABLE_ALL    = 0x10,  /* 启用所有电机 */
    CMD_MOTOR_DISABLE_ALL   = 0x11,  /* 禁用所有电机 */
    CMD_MOTOR_POS_CONTROL   = 0x12,  /* 位置控制（12关节）*/
    CMD_MOTOR_VEL_CONTROL   = 0x13,  /* 速度控制（12关节）*/
    CMD_MOTOR_TRQ_CONTROL   = 0x14,  /* 力矩控制（12关节）*/
    CMD_MOTOR_STATUS_REQ    = 0x15,  /* 查询电机状态 */
    CMD_MOTOR_SET_PARAMS    = 0x16,  /* 设置电机参数 */
    CMD_MOTOR_SEND_ENABLE   = 0x17,  /* 启用/禁用电机发送（1字节: 0=禁用, 1=启用）*/

    /* 传感器命令 (0x30-0x3F) */
    CMD_SENSOR_FORCE_REQ    = 0x30,  /* 查询足端力传感器 */
    CMD_SENSOR_POWER_REQ    = 0x31,  /* 查询电源状态 */

    /* 配置命令 (0x40-0x4F) */
    CMD_CONFIG_SAVE         = 0x40,  /* 保存配置到FLASH */
    CMD_CONFIG_LOAD         = 0x41,  /* 加载配置 */
} ProtocolCommand_t;

/**
 * @brief 响应类型（下位机 → 上位机）
 */
typedef enum {
    /* 系统响应 (0x80-0x8F) */
    RESP_SYSTEM_ACK          = 0x80,  /* 命令确认 */
    RESP_SYSTEM_NACK         = 0x81,  /* 命令拒绝 */
    RESP_SYSTEM_STATUS       = 0x82,  /* 系统状态 */
    RESP_SYSTEM_ERROR        = 0x8F,  /* 系统错误 */

    /* 电机响应 (0x90-0x9F) */
    RESP_MOTOR_STATUS        = 0x90,  /* 电机状态（12关节）*/
    RESP_MOTOR_FEEDBACK      = 0x91,  /* 电机实时反馈 */

    /* 传感器响应 (0xA0-0xAF) */
    RESP_SENSOR_FORCE        = 0xA0,  /* 足端力传感器数据 */
    RESP_SENSOR_POWER        = 0xA1,  /* 电源状态数据 */
} ProtocolResponse_t;

/**
 * @brief 协议帧结构
 */
typedef struct {
    uint8_t  header[2];         /* 帧头 [0xAA, 0x55] */
    uint8_t  cmd_type;          /* 命令/响应类型 */
    uint16_t payload_length;    /* 负载长度（小端序）*/
    uint8_t  payload[PROTO_MAX_PAYLOAD_SIZE]; /* 负载数据 */
    uint16_t crc16;             /* CRC16校验（大端序）*/
    uint8_t  footer[2];         /* 帧尾 [0x0D, 0x0A] */
} __attribute__((packed)) ProtocolFrame_t;

/**
 * @brief 电机位置控制数据（CMD_MOTOR_POS_CONTROL）
 */
typedef struct {
    float positions[12];        /* 12个关节目标位置（rad）*/
    float max_velocity;         /* 最大速度（rad/s）*/
    uint32_t timestamp_ms;      /* 时间戳（毫秒）*/
} __attribute__((packed)) MotorPositionCmd_t;

/**
 * @brief 电机状态反馈数据（RESP_MOTOR_STATUS）
 */
typedef struct {
    float positions[12];        /* 12个关节当前位置（rad）*/
    float velocities[12];       /* 12个关节当前速度（rad/s）*/
    float currents[12];         /* 12个关节当前电流（A）*/
    int16_t temperatures[12];   /* 12个关节温度（°C × 10）*/
    uint16_t error_flags[12];   /* 12个关节错误标志 */
} __attribute__((packed)) MotorStatusResp_t;

/**
 * @brief 系统状态响应数据（RESP_SYSTEM_STATUS）
 */
typedef struct {
    uint8_t  robot_state;       /* 系统状态（RobotState_t）*/
    uint16_t error_flags;       /* 错误标志（SystemError_t）*/
    uint32_t uptime_ms;         /* 运行时间（毫秒）*/
    uint32_t command_count;     /* 接收命令总数 */
    uint16_t free_heap_kb;      /* 剩余堆空间（KB）*/
    uint8_t  motor_online;      /* 在线电机数量 */
} __attribute__((packed)) SystemStatusResp_t;

/**
 * @brief 足端力传感器数据（RESP_SENSOR_FORCE）
 */
typedef struct {
    float force_fl;             /* 前左足端力（N）*/
    float force_fr;             /* 前右足端力（N）*/
    float force_rl;             /* 后左足端力（N）*/
    float force_rr;             /* 后右足端力（N）*/
    uint32_t timestamp_ms;      /* 时间戳 */
} __attribute__((packed)) FootForceResp_t;

/* ============================================================================
 * 公共API - 编码（打包）
 * ============================================================================ */

/**
 * @brief  编码命令帧（下位机发送）
 * @param  cmd_type: 响应类型
 * @param  payload: 负载数据
 * @param  payload_length: 负载长度
 * @param  output: 输出缓冲区
 * @param  output_size: 输出缓冲区大小
 * @retval 编码后的帧长度，失败返回0
 */
uint32_t protocol_encode(
    uint8_t cmd_type,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output,
    uint32_t output_size
);

/**
 * @brief  快速编码ACK/NACK（无负载）
 * @param  is_ack: true=ACK, false=NACK
 * @param  output: 输出缓冲区
 * @retval 编码后的帧长度
 */
uint32_t protocol_encode_ack(bool is_ack, uint8_t *output);

/**
 * @brief  编码电机状态响应
 * @param  status: 电机状态数据
 * @param  output: 输出缓冲区
 * @retval 编码后的帧长度
 */
uint32_t protocol_encode_motor_status(const MotorStatusResp_t *status, uint8_t *output);

/**
 * @brief  编码系统状态响应
 * @param  status: 系统状态数据
 * @param  output: 输出缓冲区
 * @retval 编码后的帧长度
 */
uint32_t protocol_encode_system_status(const SystemStatusResp_t *status, uint8_t *output);

/**
 * @brief  编码足端力传感器响应
 * @param  force: 足端力数据
 * @param  output: 输出缓冲区
 * @retval 编码后的帧长度
 */
uint32_t protocol_encode_foot_force(const FootForceResp_t *force, uint8_t *output);

/* ============================================================================
 * 公共API - 解码（解包）
 * ============================================================================ */

/**
 * @brief  解码接收到的帧
 * @param  input: 输入缓冲区
 * @param  input_length: 输入长度
 * @param  cmd_type: 输出命令类型
 * @param  payload: 输出负载数据
 * @param  payload_length: 输出负载长度
 * @retval 0=成功, -1=帧头错误, -2=CRC错误, -3=其他错误
 */
int protocol_decode(
    const uint8_t *input,
    uint32_t input_length,
    uint8_t *cmd_type,
    uint8_t *payload,
    uint16_t *payload_length
);

/**
 * @brief  解码电机位置控制命令
 * @param  payload: 负载数据
 * @param  length: 负载长度
 * @param  cmd: 输出命令结构体
 * @retval 0=成功, -1=失败
 */
int protocol_decode_motor_pos_cmd(
    const uint8_t *payload,
    uint16_t length,
    MotorPositionCmd_t *cmd
);

/* ============================================================================
 * 公共API - 工具函数
 * ============================================================================ */

/**
 * @brief  查找帧头位置
 * @param  buffer: 数据缓冲区
 * @param  length: 缓冲区长度
 * @retval 帧头位置，未找到返回-1
 */
int protocol_find_header(const uint8_t *buffer, uint32_t length);

/**
 * @brief  计算完整帧的长度
 * @param  payload_length: 负载长度
 * @retval 完整帧长度（包含帧头、CRC、帧尾）
 */
uint32_t protocol_calc_frame_length(uint16_t payload_length);

/**
 * @brief  获取命令名称字符串（调试用）
 * @param  cmd_type: 命令类型
 * @retval 命令名称字符串
 */
const char* protocol_get_cmd_name(uint8_t cmd_type);

/**
 * @brief  打印帧内容（调试用）
 * @param  data: 帧数据
 * @param  length: 帧长度
 */
void protocol_print_frame(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_CODEC_H */
