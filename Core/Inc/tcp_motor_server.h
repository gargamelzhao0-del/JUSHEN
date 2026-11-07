/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tcp_motor_server.h
  * @brief   TCP电机数据服务器 - 用于与Jetson Orin NX进行通信
  * @author  工业控制系统
  * @date    2025-12-16
  ******************************************************************************
  * @attention
  *
  * 该模块提供高速、低延迟的TCP通信，用于通过LAN8720以太网
  * 与NVIDIA Jetson Orin NX进行电机控制数据交换。
  *
  * V3.4: 当前系统使用客户端模式（STM32主动连接上位机）
  * 
  * 主要功能：
  * - 客户端模式：STM32主动连接上位机（当前使用）
  * - 服务器模式：STM32监听端口等待连接（保留备用）
  * - 发送电机状态数据（位置、速度、电流、温度等）
  * - 接收电机控制命令（模式设置、目标值、紧急停止等）
  * - 提供数据校验和错误处理机制
  *
  * 通信协议：
  * - STM32 -> Jetson: 32字节电机状态数据包，周期发送
  * - Jetson -> STM32: 8字节电机控制命令包，事件触发
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef TCP_MOTOR_SERVER_H
#define TCP_MOTOR_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/tcp.h"
#include <stdint.h>
#include <stdbool.h>

/* ============== 运行模式配置 ============== */
#define TCP_MODE_CLIENT  1
#define TCP_MODE_SERVER  2
#define TCP_OPERATING_MODE TCP_MODE_CLIENT  /* V3.4: 当前使用客户端模式 */

/* ============== 配置参数 ============== */
#define MOTOR_SERVER_PORT 9999          /* TCP服务器监听端口 (与上位机config.py一致) */
#define MAX_TCP_CLIENTS 2               /* 最大客户端连接数 */
#define MOTOR_DATA_PACKET_SIZE 32       /* 电机数据包包大小（字节） */
#define MOTOR_CMD_PACKET_SIZE 8         /* 电机命令包大小（字节） */

/* ============== 数据结构定义 ============== */

/**
 * @brief 电机数据打包结构（精简版，16字节）
 * V10.0: 精简为位置/速度/扭矩/电流，减少网络带宽占用
 * 网络传输优化：使用packed属性避免字节对齐
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;      /* 系统时间戳（毫秒）*/
    uint8_t motor_id;           /* 电机ID (0-11) */
    uint8_t checksum;           /* 数据校验和 */
    float position;             /* 电机位置（角度，度）*/
    float velocity;             /* 电机速度（角度/秒）*/
    float torque;               /* 电机扭矩（Nm）*/
    float current;              /* 电机电流（安培）*/
} MotorDataPacket_t;

#define MOTOR_DATA_PACKET_SIZE_NEW  16  /* 精简后数据包大小（字节） */

/**
 * @brief 电机命令包结构（8字节）
 * 用于接收来自Jetson的控制命令
 */
typedef struct __attribute__((packed)) {
    uint8_t command;            /* 命令类型：1=设置模式，2=设置电流，3=设置速度，4=设置位置 */
    uint8_t reserved;           /* 保留字段，用于未来扩展 */
    float value;                /* 命令数值 */
    uint16_t checksum;          /* 数据校验和 */
} MotorCommandPacket_t;

/* 命令类型定义 - DEPRECATED (旧协议命令，已被新CAN FD协议替代) */
#if 0
#define CMD_SET_MODE        1   /* 设置电机工作模式 */
#define CMD_SET_CURRENT     2   /* 设置目标电流值 */
#define CMD_SET_VELOCITY    3   /* 设置目标速度值 */
#define CMD_SET_POSITION    4   /* 设置目标位置值 */
#define CMD_EMERGENCY_STOP  5   /* 紧急停止命令 */
#define CMD_CLEAR_ERRORS    6   /* 清除错误标志 */
#define CMD_SET_CONTROL_MODE 7  /* 设置控制模式: 0=纯位置, 1=力位混合 */
#endif
/* NOTE: 新的命令定义在 canfd_motor_driver.h 中 (CMD_SET_POSITION=0x41, CMD_SET_VELOCITY=0x42, etc.) */

/* ============== 公共API函数接口 ============== */

/**
 * @brief 初始化TCP电机数据服务器
 * @note 必须在LwIP网络协议栈初始化之后调用
 * @return true 初始化成功，false 初始化失败
 */
bool tcp_motor_server_init(void);

/**
 * @brief 向所有已连接的客户端发送电机数据 (单电机, 保留兼容性)
 * @param data 指向电机数据包的指针
 * @return 成功发送数据的客户端数量
 */
uint8_t tcp_motor_server_send_data(MotorDataPacket_t* data);

/* DEPRECATED: Old multi-motor function - replaced by Quadruped_PackStatusForTCP + tcp_client_send_data */
#if 0
/**
 * @brief 向所有已连接的客户端发送12电机状态数据 (已废弃)
 * @param multi_ctrl 多电机控制器句柄
 * @return 成功发送数据的客户端数量
 */
uint8_t tcp_motor_server_send_multi_motor_status(MultiMotorController* multi_ctrl);

/**
 * @brief 将电机状态数据打包成网络传输格式 (单电机, 保留兼容性, 已废弃)
 * @param motor 电机控制句柄
 * @param packet 输出数据包指针
 */
void tcp_motor_pack_data(MotorHandle* motor, MotorDataPacket_t* packet);
#endif

/**
 * @brief 获取当前活跃的TCP连接数量
 * @return 活跃客户端数量
 */
uint8_t tcp_motor_server_get_client_count(void);

/**
 * @brief 获取服务器运行统计信息
 * @param tx_packets 发送数据包计数指针
 * @param rx_packets 接收数据包计数指针
 * @param tx_errors 发送错误计数指针
 */
void tcp_motor_server_get_stats(uint32_t* tx_packets, uint32_t* rx_packets, uint32_t* tx_errors);

/**
 * @brief 向所有已连接的客户端发送简单文本消息
 * @param message 要发送的文本消息字符串
 * @return 成功发送消息的客户端数量
 */
uint8_t tcp_motor_server_send_text(const char* message);

/* V9.0: 发送所有12个电机的状态上报给上位机 */
uint8_t tcp_motor_server_send_all_motor_status(void);

/**
 * @brief 初始化TCP客户端，连接到指定的上位机
 * @param remote_ip 上位机IP地址字符串 (例如: "192.168.1.20")
 * @param remote_port 上位机端口号
 * @return true 连接成功，false 连接失败
 */
bool tcp_client_connect(const char* remote_ip, uint16_t remote_port);

/**
 * @brief 通过TCP客户端发送文本消息到上位机
 * @param message 要发送的文本消息字符串
 * @return true 发送成功，false 发送失败
 */
bool tcp_client_send_text(const char* message);

/**
 * @brief 通过TCP客户端发送二进制数据到上位机
 * @param data 要发送的数据指针
 * @param len 数据长度（字节）
 * @return true 发送成功，false 发送失败
 */
bool tcp_client_send_data(uint8_t* data, uint16_t len);

/**
 * @brief 获取TCP客户端连接状态
 * @return true 已连接，false 未连接
 */
bool tcp_client_is_connected(void);

/**
 * @brief 获取TCP服务器客户端连接状态（用于服务器模式）
 * @return true 有客户端连接，false 无客户端连接
 */
bool tcp_server_is_client_connected(void);

/* ============== V2.0: 心跳机制相关函数 ============== */

/**
 * @brief 检查TCP客户端连接健康状态
 * @return true 健康, false 不健康（心跳超时）
 */
bool tcp_client_is_healthy(void);

/**
 * @brief 获取心跳统计信息
 * @param missed_count 丢失心跳次数输出
 * @param last_rx_time 上次接收心跳时间输出
 */
void tcp_get_heartbeat_stats(uint8_t* missed_count, uint32_t* last_rx_time);

/**
 * @brief 获取上位机数据接收状态
 * @param last_rx_tick 输出上次接收数据的时间戳（毫秒）
 * @param rx_count 输出接收数据计数
 * @note V3.0: 用于LED闪烁指示上位机数据接收状态
 */
void tcp_get_upper_computer_rx_status(uint32_t* last_rx_tick, uint32_t* rx_count);

/**
 * @brief 检查上位机是否正在发送数据
 * @param timeout_ms 超时时间（毫秒），超过此时间未收到数据则认为停止发送
 * @return true 上位机正在发送数据，false 上位机已停止发送
 * @note V3.0: 用于LED闪烁指示
 */
bool tcp_is_upper_computer_sending(uint32_t timeout_ms);

/**
 * @brief 获取PONG心跳响应状态
 * @param last_pong_tick 输出上次收到PONG的时间戳（毫秒）
 * @param missed_count 输出连续丢失PONG的次数
 * @note V3.0: 用于双向心跳确认
 */
void tcp_get_pong_status(uint32_t* last_pong_tick, uint32_t* missed_count);

/**
 * @brief 重置PONG心跳状态
 * @note V3.0: 在发送心跳前调用
 */
void tcp_reset_pong_state(void);

/**
 * @brief 检查PONG心跳是否超时
 * @param timeout_ms 超时时间（毫秒）
 * @return true 心跳正常，false 心跳超时
 * @note V3.0: 用于检测上位机是否存活
 */
bool tcp_is_heartbeat_healthy(uint32_t timeout_ms);

/**
 * @brief 断开TCP客户端连接
 * @note V3.1: 用于心跳超时后主动断开连接
 */
void tcp_client_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_MOTOR_SERVER_H */
