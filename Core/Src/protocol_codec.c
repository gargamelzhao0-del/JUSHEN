/**
 ******************************************************************************
 * @file    protocol_codec.c
 * @brief   二进制通信协议编解码实现
 ******************************************************************************
 */

#include "protocol_codec.h"
#include "crc_checksum.h"
#include "log_system.h"
#include <string.h>

/* ============================================================================
 * 编码（打包）函数实现
 * ============================================================================ */

/**
 * @brief  编码通用帧
 */
uint32_t protocol_encode(
    uint8_t cmd_type,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output,
    uint32_t output_size)
{
    if (output == NULL) {
        return 0;
    }

    /* 检查缓冲区大小 */
    uint32_t frame_length = protocol_calc_frame_length(payload_length);
    if (frame_length > output_size || payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        LOG_ERROR("[Protocol] Buffer too small or payload too large");
        return 0;
    }

    uint32_t offset = 0;

    /* 1. 帧头 */
    output[offset++] = PROTO_HEADER_0;
    output[offset++] = PROTO_HEADER_1;

    /* 2. 命令类型 */
    output[offset++] = cmd_type;

    /* 3. 负载长度（小端序）*/
    output[offset++] = (payload_length >> 0) & 0xFF;
    output[offset++] = (payload_length >> 8) & 0xFF;

    /* 4. 负载数据 */
    if (payload != NULL && payload_length > 0) {
        memcpy(&output[offset], payload, payload_length);
        offset += payload_length;
    }

    /* 5. CRC16校验（对 cmd_type + length + payload 计算）*/
    uint16_t crc = crc16_modbus(&output[2], 1 + 2 + payload_length);
    output[offset++] = (crc >> 8) & 0xFF;  /* CRC高字节 */
    output[offset++] = (crc >> 0) & 0xFF;  /* CRC低字节 */

    /* 6. 帧尾 */
    output[offset++] = PROTO_FOOTER_0;
    output[offset++] = PROTO_FOOTER_1;

    return offset;
}

/**
 * @brief  快速编码ACK/NACK
 */
uint32_t protocol_encode_ack(bool is_ack, uint8_t *output)
{
    uint8_t cmd_type = is_ack ? RESP_SYSTEM_ACK : RESP_SYSTEM_NACK;
    return protocol_encode(cmd_type, NULL, 0, output, PROTO_MAX_FRAME_SIZE);
}

/**
 * @brief  编码电机状态响应
 */
uint32_t protocol_encode_motor_status(const MotorStatusResp_t *status, uint8_t *output)
{
    if (status == NULL || output == NULL) {
        return 0;
    }

    return protocol_encode(
        RESP_MOTOR_STATUS,
        (const uint8_t*)status,
        sizeof(MotorStatusResp_t),
        output,
        PROTO_MAX_FRAME_SIZE
    );
}

/**
 * @brief  编码系统状态响应
 */
uint32_t protocol_encode_system_status(const SystemStatusResp_t *status, uint8_t *output)
{
    if (status == NULL || output == NULL) {
        return 0;
    }

    return protocol_encode(
        RESP_SYSTEM_STATUS,
        (const uint8_t*)status,
        sizeof(SystemStatusResp_t),
        output,
        PROTO_MAX_FRAME_SIZE
    );
}

/**
 * @brief  编码足端力传感器响应
 */
uint32_t protocol_encode_foot_force(const FootForceResp_t *force, uint8_t *output)
{
    if (force == NULL || output == NULL) {
        return 0;
    }

    return protocol_encode(
        RESP_SENSOR_FORCE,
        (const uint8_t*)force,
        sizeof(FootForceResp_t),
        output,
        PROTO_MAX_FRAME_SIZE
    );
}

/* ============================================================================
 * 解码（解包）函数实现
 * ============================================================================ */

/**
 * @brief  解码接收到的帧
 */
int protocol_decode(
    const uint8_t *input,
    uint32_t input_length,
    uint8_t *cmd_type,
    uint8_t *payload,
    uint16_t *payload_length)
{
    if (input == NULL || cmd_type == NULL || payload_length == NULL) {
        return -3;
    }

    /* 最小帧长度检查 */
    uint32_t min_frame_length = 2 + 1 + 2 + 2 + 2;  /* header + cmd + len + crc + footer */
    if (input_length < min_frame_length) {
        LOG_ERROR("[Protocol] Frame too short: %lu bytes", input_length);
        return -3;
    }

    /* 1. 检查帧头 */
    if (input[0] != PROTO_HEADER_0 || input[1] != PROTO_HEADER_1) {
        LOG_ERROR("[Protocol] Invalid header: 0x%02X 0x%02X", input[0], input[1]);
        return -1;
    }

    /* 2. 提取命令类型 */
    *cmd_type = input[2];

    /* 3. 提取负载长度（小端序）*/
    *payload_length = ((uint16_t)input[3] << 0) | ((uint16_t)input[4] << 8);

    /* 检查负载长度合法性 */
    if (*payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        LOG_ERROR("[Protocol] Payload too large: %u bytes", *payload_length);
        return -3;
    }

    /* 检查总长度 */
    uint32_t expected_length = protocol_calc_frame_length(*payload_length);
    if (input_length < expected_length) {
        LOG_ERROR("[Protocol] Incomplete frame: %lu < %lu", input_length, expected_length);
        return -3;
    }

    /* 4. 提取CRC */
    uint32_t crc_offset = 2 + 1 + 2 + (*payload_length);
    uint16_t received_crc = ((uint16_t)input[crc_offset] << 8) |
                           ((uint16_t)input[crc_offset + 1] << 0);

    /* 5. 计算CRC（对 cmd + length + payload）*/
    uint16_t calculated_crc = crc16_modbus(&input[2], 1 + 2 + (*payload_length));

    /* 6. 验证CRC */
    if (received_crc != calculated_crc) {
        LOG_ERROR("[Protocol] CRC mismatch: rx=0x%04X, calc=0x%04X",
                  received_crc, calculated_crc);
        return -2;
    }

    /* 7. 检查帧尾 */
    uint32_t footer_offset = crc_offset + 2;
    if (input[footer_offset] != PROTO_FOOTER_0 || input[footer_offset + 1] != PROTO_FOOTER_1) {
        LOG_WARN("[Protocol] Invalid footer: 0x%02X 0x%02X",
                 input[footer_offset], input[footer_offset + 1]);
        /* 帧尾错误不影响数据完整性，只发出警告 */
    }

    /* 8. 提取负载数据 */
    if (payload != NULL && (*payload_length) > 0) {
        memcpy(payload, &input[5], *payload_length);
    }

    return 0;  /* 成功 */
}

/**
 * @brief  解码电机位置控制命令
 */
int protocol_decode_motor_pos_cmd(
    const uint8_t *payload,
    uint16_t length,
    MotorPositionCmd_t *cmd)
{
    if (payload == NULL || cmd == NULL) {
        return -1;
    }

    /* 检查长度 */
    if (length != sizeof(MotorPositionCmd_t)) {
        LOG_ERROR("[Protocol] Invalid motor position cmd size: %u != %u",
                  length, sizeof(MotorPositionCmd_t));
        return -1;
    }

    /* 直接复制 */
    memcpy(cmd, payload, sizeof(MotorPositionCmd_t));

    return 0;
}

/* ============================================================================
 * 工具函数实现
 * ============================================================================ */

/**
 * @brief  查找帧头位置
 */
int protocol_find_header(const uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length < 2) {
        return -1;
    }

    for (uint32_t i = 0; i < length - 1; i++) {
        if (buffer[i] == PROTO_HEADER_0 && buffer[i + 1] == PROTO_HEADER_1) {
            return (int)i;
        }
    }

    return -1;  /* 未找到 */
}

/**
 * @brief  计算完整帧的长度
 */
uint32_t protocol_calc_frame_length(uint16_t payload_length)
{
    /* header(2) + cmd(1) + length(2) + payload(N) + crc(2) + footer(2) */
    return 2 + 1 + 2 + payload_length + 2 + 2;
}

/**
 * @brief  获取命令名称字符串
 */
const char* protocol_get_cmd_name(uint8_t cmd_type)
{
    switch (cmd_type) {
        /* 系统命令 */
        case CMD_SYSTEM_RESET:       return "SYS_RESET";
        case CMD_SYSTEM_HEARTBEAT:   return "SYS_HEARTBEAT";
        case CMD_SYSTEM_STATUS_REQ:  return "SYS_STATUS_REQ";
        case CMD_SYSTEM_ENTER_IDLE:  return "SYS_ENTER_IDLE";
        case CMD_SYSTEM_ENTER_READY: return "SYS_ENTER_READY";
        case CMD_SYSTEM_EMERGENCY:   return "SYS_EMERGENCY";

        /* 电机命令 */
        case CMD_MOTOR_ENABLE_ALL:   return "MOTOR_ENABLE_ALL";
        case CMD_MOTOR_DISABLE_ALL:  return "MOTOR_DISABLE_ALL";
        case CMD_MOTOR_POS_CONTROL:  return "MOTOR_POS_CTRL";
        case CMD_MOTOR_VEL_CONTROL:  return "MOTOR_VEL_CTRL";
        case CMD_MOTOR_TRQ_CONTROL:  return "MOTOR_TRQ_CTRL";
        case CMD_MOTOR_STATUS_REQ:   return "MOTOR_STATUS_REQ";
        case CMD_MOTOR_SEND_ENABLE:  return "MOTOR_SEND_ENABLE";

        /* 传感器命令 */
        case CMD_SENSOR_FORCE_REQ:   return "SENSOR_FORCE_REQ";
        case CMD_SENSOR_POWER_REQ:   return "SENSOR_POWER_REQ";

        /* 系统响应 */
        case RESP_SYSTEM_ACK:        return "RESP_ACK";
        case RESP_SYSTEM_NACK:       return "RESP_NACK";
        case RESP_SYSTEM_STATUS:     return "RESP_SYS_STATUS";
        case RESP_SYSTEM_ERROR:      return "RESP_SYS_ERROR";

        /* 电机响应 */
        case RESP_MOTOR_STATUS:      return "RESP_MOTOR_STATUS";
        case RESP_MOTOR_FEEDBACK:    return "RESP_MOTOR_FEEDBACK";

        /* 传感器响应 */
        case RESP_SENSOR_FORCE:      return "RESP_SENSOR_FORCE";
        case RESP_SENSOR_POWER:      return "RESP_SENSOR_POWER";

        default:                     return "UNKNOWN";
    }
}

/**
 * @brief  打印帧内容（调试用）
 */
void protocol_print_frame(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return;
    }

    LOG_INFO("[Protocol] Frame dump (%lu bytes):", length);

    /* 打印前64字节（避免太长）*/
    uint32_t print_len = (length > 64) ? 64 : length;

    printf("  ");
    for (uint32_t i = 0; i < print_len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\r\n  ");
        }
    }
    if (length > 64) {
        printf("... (%u more bytes)\r\n", length - 64);
    } else {
        printf("\r\n");
    }
}
