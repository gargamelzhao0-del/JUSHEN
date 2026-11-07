/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tcp_motor_server.c
  * @brief   TCP电机数据服务器实现源码
  * @details 该文件实现了与Jetson Orin NX的TCP通信功能
  *          包含TCP服务器初始化、数据发送、命令处理等核心功能
  * @version V2.0 - 工业级优化版本
  *          V2.0: 添加心跳机制，改进连接健康检测
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tcp_motor_server.h"
#include "system_config.h"
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/etharp.h"
#include "cmsis_os.h"
#include "quad_control_loop.h"
#include "quad_command_fusion.h"
#include "quad_types.h"
#include "security_auth.h"
#include "motor_control_enhanced.h"
#include "motor_sync.h"
#include "log_system.h"
#include <string.h>
#include <stdio.h>
#include <math.h>  /* V12.0: 添加math.h用于fmodf */

/* 心跳机制配置使用system_config.h中的定义 */

/* 心跳消息定义 */
static const char* HEARTBEAT_PING = "PING";
static const char* HEARTBEAT_PONG = "PONG";

/* ============== 私有类型定义 ============== */

/* 心跳状态结构体 */
typedef struct {
    uint32_t last_heartbeat_tx;     /* 上次发送心跳时间 */
    uint32_t last_heartbeat_rx;     /* 上次接收心跳时间 */
    uint8_t missed_count;           /* 连续丢失心跳次数 */
    bool waiting_pong;              /* 是否等待PONG响应 */
    bool enabled;                   /* 心跳是否启用 */
} HeartbeatState_t;

typedef struct {
    struct tcp_pcb* pcb;
    bool active;
    uint32_t tx_packets;
    uint32_t rx_packets;
    AuthContext_t auth;
    HeartbeatState_t heartbeat;     /* V2.0: 心跳状态 */
} TcpClient_t;

/* TCP客户端连接参数结构（用于tcpip线程回调） */
typedef struct {
    ip_addr_t remote_addr;      /* 远程IP地址 */
    uint16_t remote_port;       /* 远程端口 */
    err_t result;               /* 操作结果 */
    osSemaphoreId_t done_sem;   /* 完成信号量 */
    volatile bool in_use;       /* V3.4: 是否正在使用，防止竞态 */
} TcpClientConnectParams_t;

/* V3.4: 连接请求池，避免静态变量竞态 */
#define MAX_CONNECT_REQUESTS 2
static TcpClientConnectParams_t g_connect_requests[MAX_CONNECT_REQUESTS];
static osMutexId_t g_connect_pool_mutex = NULL;

/* ============== 私有函数前向声明 ============== */
static void tcp_heartbeat_init(HeartbeatState_t* hb);

/* ============== 私有全局变量 ============== */
static struct tcp_pcb* server_pcb = NULL;    /* TCP服务器控制块 */
static TcpClient_t clients[MAX_TCP_CLIENTS] = {0};  /* 客户端连接数组 */
static osMutexId_t tcp_mutex = NULL;         /* TCP操作互斥锁 */
static uint32_t total_tx_packets = 0;        /* 总发送包数 */
static uint32_t total_rx_packets = 0;        /* 总接收包数 */
static uint32_t total_tx_errors = 0;         /* 总发送错误数 */

/* TCP客户端相关变量 */
static struct tcp_pcb* client_pcb = NULL;    /* TCP客户端控制块 */
static volatile bool client_connected = false; /* 客户端连接状态 (volatile for thread safety) */
static osMutexId_t tcp_client_mutex = NULL;  /* TCP客户端互斥锁 */

/* V2.2: TCP发送缓冲区和信号量 - 用于线程安全发送 */
/* V12.1: 增大缓冲区以容纳更大的数据包 (256→512字节) */
/* P0-3修复: 使用双缓冲消除生产者-消费者竞态 */
#define TCP_SEND_BUFFER_SIZE 512
static uint8_t tcp_send_buffers[2][TCP_SEND_BUFFER_SIZE];  /* 双缓冲 */
static uint16_t tcp_send_lens[2] = {0, 0};                 /* 每个buffer对应的长度 */
static volatile uint8_t tcp_send_write_idx = 0;             /* 当前写入的buffer索引 */
static volatile uint8_t tcp_send_result = false;
static osSemaphoreId_t tcp_send_sem = NULL;

/* V2.4: 异步发送统计 */
static volatile uint32_t tcp_async_send_count = 0;
static volatile uint32_t tcp_async_fail_count = 0;

/* V3.0: 上位机数据接收时间戳 - 用于LED闪烁指示 */
static volatile uint32_t g_upper_computer_last_rx_tick = 0;
static volatile uint32_t g_upper_computer_rx_count = 0;

/* V3.0: PONG心跳响应状态 - 用于双向心跳确认 */
static volatile bool g_heartbeat_pong_received = false;
static volatile uint32_t g_heartbeat_last_pong_tick = 0;
static volatile uint32_t g_heartbeat_missed_count = 0;

/* V3.4: 增强版接收状态机 - 修复缓冲区溢出问题 */
typedef enum {
    RX_STATE_WAIT_HEADER1,      /* 等待帧头第1字节 0xAA */
    RX_STATE_WAIT_HEADER2,      /* 等待帧头第2字节 0x55 */
    RX_STATE_WAIT_TYPE,         /* 等待帧类型 */
    RX_STATE_WAIT_DATA,         /* 等待数据 */
    RX_STATE_COMPLETE,          /* 帧接收完成 */
    RX_STATE_ERROR              /* V3.4: 错误状态 */
} RxState_t;

typedef struct {
    RxState_t state;
    uint8_t buffer[512];        /* V3.4: 增大缓冲区到512字节 */
    uint16_t buffer_len;        /* 当前缓冲区长度 */
    uint8_t frame_type;         /* 帧类型 */
    uint16_t expected_len;      /* 期望帧长度 */
    uint32_t error_count;       /* V3.4: 错误计数 */
    uint32_t overflow_count;    /* V3.4: 溢出计数 */
} RxContext_t;

static RxContext_t g_rx_ctx = {RX_STATE_WAIT_HEADER1, {0}, 0, 0, 0, 0, 0};

/* ============== 问题修复: TCP连接状态竞态条件 ============== */
/**
 * @brief 统一的TCP连接状态设置函数（线程安全）
 * @param connected 连接状态
 */
static void tcp_client_set_connected(bool connected)
{
    if (tcp_client_mutex != NULL) {
        osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
    }
    client_connected = connected;
    if (tcp_client_mutex != NULL) {
        osMutexRelease(tcp_client_mutex);
    }
}

/* ============== 外部变量声明 ============== */
/* OLD: extern MotorHandle g_motor; - No longer used with new quadruped system */
/* OLD: extern osMutexId_t motor_mutex; - No longer used */
/* OLD: extern MultiMotorController g_multi_motor; - Replaced by QuadrupedController_t */

/* ============== 私有函数原型声明 ============== */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);           /* 处理客户端连接请求 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); /* 处理接收到的数据 */
static void tcp_server_error(void *arg, err_t err);                                    /* 处理连接错误 */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);                         /* TCP轮询回调函数 */
static uint8_t calculate_checksum(uint8_t* data, uint16_t len);                        /* 计算数据校验和 */

/* TCP客户端回调函数 */
static err_t tcp_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err);     /* 客户端连接成功回调 */
static err_t tcp_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); /* 客户端接收回调 */
static void tcp_client_error_cb(void *arg, err_t err);                                 /* 客户端错误回调 */
static err_t tcp_client_poll_cb(void *arg, struct tcp_pcb *tpcb);                     /* 客户端轮询回调 */
static void tcp_client_connect_tcpip_thread(void *arg);                                /* 在tcpip线程中执行连接 */

/* ============== 公共函数实现 ============== */

/**
 * @brief 初始化TCP电机数据服务器
 * @details 创建TCP服务器，绑定到指定端口，开始监听客户端连接
 *          包括互斥锁创建、TCP控制块初始化、端口绑定和监听设置
 * @return true 初始化成功，false 初始化失败
 */
bool tcp_motor_server_init(void)
{
    printf("[TCP] V10.0: H7作为TCP客户端模式初始化...\n");
    
    tcp_mutex = osMutexNew(NULL);
    if (tcp_mutex == NULL) {
        printf("[TCP] 互斥锁创建失败\n");
        return false;
    }

    tcp_client_mutex = osMutexNew(NULL);
    if (tcp_client_mutex == NULL) {
        printf("[TCP] 客户端互斥锁创建失败\n");
        return false;
    }

    tcp_send_sem = osSemaphoreNew(1, 0, NULL);
    if (tcp_send_sem == NULL) {
        printf("[TCP] 发送信号量创建失败\n");
        return false;
    }

    printf("[TCP] TCP客户端模式初始化完成，等待连接上位机服务器...\n");
    printf("[TCP] 请在主循环中调用 tcp_client_connect() 连接上位机\n");
    return true;
}

/**
 * @brief 向所有已连接的客户端发送电机数据
 * @details 遍历所有活跃的客户端连接，发送电机数据报文
 *          包括数据校验和计算、TCP数据发送、错误处理和统计更新
 * @param data 指向电机数据包的指针
 * @return 成功发送数据的客户端数量
 */
uint8_t tcp_motor_server_send_data(MotorDataPacket_t* data)
{
    if (tcp_mutex == NULL || data == NULL) return 0;

    uint8_t clients_sent = 0;

    osMutexAcquire(tcp_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active && clients[i].pcb != NULL) {
            /* 计算数据包校验和 */
            data->checksum = calculate_checksum((uint8_t*)data, sizeof(MotorDataPacket_t) - 1);

            /* 尝试发送数据 */
            err_t err = tcp_write(clients[i].pcb, data, sizeof(MotorDataPacket_t), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(clients[i].pcb);  /* 强制立即发送 */
                clients[i].tx_packets++;
                total_tx_packets++;
                clients_sent++;
            } else if (err == ERR_MEM) {
                /* 发送缓冲区满 - 跳过本次传输 */
                total_tx_errors++;
            } else {
                /* 其他错误 - 标记客户端为非活跃状态 */
                clients[i].active = false;
                total_tx_errors++;
            }
        }
    }

    osMutexRelease(tcp_mutex);
    return clients_sent;
}

/* DEPRECATED: This function is for the old multi-motor system and is no longer used */
/* The new quadruped system uses Quadruped_PackStatusForTCP() and tcp_client_send_data() instead */
#if 0
/**
 * @brief 向所有已连接的客户端发送12电机状态数据 (已废弃)
 */
uint8_t tcp_motor_server_send_multi_motor_status(MultiMotorController* multi_ctrl)
{
    if (tcp_mutex == NULL || multi_ctrl == NULL) return 0;

    uint8_t clients_sent = 0;
    MultiMotorStatusPacket status_packet;

    /* 打包12电机状态数据 */
    multi_motor_pack_status(multi_ctrl, &status_packet);

    osMutexAcquire(tcp_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active && clients[i].pcb != NULL) {
            /* 发送状态包 */
            err_t err = tcp_write(clients[i].pcb, &status_packet,
                                  sizeof(MultiMotorStatusPacket), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(clients[i].pcb);
                clients[i].tx_packets++;
                total_tx_packets++;
                clients_sent++;
            } else if (err == ERR_MEM) {
                total_tx_errors++;
            } else {
                clients[i].active = false;
                total_tx_errors++;
            }
        }
    }

    osMutexRelease(tcp_mutex);
    return clients_sent;
}
#endif  /* Deprecated function */

/* DEPRECATED: This function is for the old single motor system and is no longer used */
#if 0
/**
 * @brief 将电机状态数据打包成网络传输格式 (单电机,保留兼容性)
 * @details 从电机控制句柄中读取当前状态信息，填入数据包结构
 *          时间戳使用HAL库函数获取，确保与系统时间同步
 * @param motor 电机控制句柄
 * @param packet 输出数据包指针
 */
void tcp_motor_pack_data(MotorHandle* motor, MotorDataPacket_t* packet)
{
    if (!motor || !packet) return;

    MotorHandle* handle = motor_handle_get(motor->motor_index);

    packet->timestamp_ms = HAL_GetTick();  /* 获取系统时间戳 */
    packet->motor_id = motor->motor_index;
    packet->enabled = motor->enabled;
    packet->position = motor->status.position;
    packet->velocity = motor->status.velocity;
    packet->torque = (handle != NULL) ? handle->current_torque : 0.0f;  /* V9.0: 扭矩 */
    packet->current = motor->status.current;
    packet->motor_temp = motor->status.motor_temp;
    packet->board_temp = (handle != NULL) ? handle->board_temperature : 0.0f;  /* V9.0: 电路板温度 */
    packet->bus_voltage = motor->status.bus_voltage;
    packet->error_flags = *(uint16_t*)&motor->status.error;
    packet->mode = motor->status.mode;
    packet->checksum = 0;  /* 校验和在发送函数中计算 */
}
#endif  /* Deprecated function */

/**
 * @brief 获取当前活跃的TCP连接数量
 * @details 遍历客户端连接数组，统计标记为活跃状态的连接数
 * @return 活跃客户端数量
 */
uint8_t tcp_motor_server_get_client_count(void)
{
    if (tcp_mutex == NULL) return 0;

    uint8_t count = 0;
    osMutexAcquire(tcp_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active) count++;
    }

    osMutexRelease(tcp_mutex);
    return count;
}

/**
 * @brief 获取服务器运行统计信息
 * @details 返回TCP服务器的运行统计数据，用于监控和调试
 * @param tx_packets 发送数据包计数指针
 * @param rx_packets 接收数据包计数指针
 * @param tx_errors 发送错误计数指针
 */
void tcp_motor_server_get_stats(uint32_t* tx_packets, uint32_t* rx_packets, uint32_t* tx_errors)
{
    if (tx_packets) *tx_packets = total_tx_packets;
    if (rx_packets) *rx_packets = total_rx_packets;
    if (tx_errors) *tx_errors = total_tx_errors;
}

/**
 * @brief 向所有已连接的客户端发送简单文本消息
 * @details 用于发送调试消息或简单的字符串通信
 * @param message 要发送的文本消息字符串
 * @return 成功发送消息的客户端数量
 */
uint8_t tcp_motor_server_send_text(const char* message)
{
    if (tcp_mutex == NULL || message == NULL) return 0;

    uint8_t clients_sent = 0;
    size_t msg_len = strlen(message);

    osMutexAcquire(tcp_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active && clients[i].pcb != NULL) {
            /* 尝试发送文本消息 */
            err_t err = tcp_write(clients[i].pcb, message, msg_len, TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(clients[i].pcb);  /* 强制立即发送 */
                clients[i].tx_packets++;
                total_tx_packets++;
                clients_sent++;
            } else if (err == ERR_MEM) {
                /* 发送缓冲区满 - 跳过本次传输 */
                total_tx_errors++;
            } else {
                /* 其他错误 - 标记客户端为非活跃状态 */
                clients[i].active = false;
                total_tx_errors++;
            }
        }
    }

    osMutexRelease(tcp_mutex);
    return clients_sent;
}

/* ============== 私有函数实现 ============== */

/**
 * @brief TCP服务器接受连接回调函数
 * @details 处理新的客户端连接请求，分配客户端槽位，设置回调函数
 * @param arg 用户自定义参数（指向客户端结构体）
 * @param newpcb 新建立的TCP连接控制块
 * @param err 连接错误状态
 * @return ERR_OK 接受成功，ERR_ABRT 拒绝连接，ERR_VAL 参数错误
 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (err != ERR_OK || newpcb == NULL) return ERR_VAL;

    osMutexAcquire(tcp_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (!clients[i].active) {
            clients[i].pcb = newpcb;
            clients[i].active = true;
            clients[i].tx_packets = 0;
            clients[i].rx_packets = 0;
            
            Security_Reset(&clients[i].auth);
            Security_GenerateChallenge(clients[i].auth.challenge, AUTH_CHALLENGE_LENGTH);
            clients[i].auth.state = AUTH_STATE_CHALLENGE_SENT;
            clients[i].auth.auth_time = HAL_GetTick();
            clients[i].auth.session_id = Security_GenerateSessionId();
            
            tcp_heartbeat_init(&clients[i].heartbeat);

            tcp_setprio(newpcb, TCP_PRIO_MAX);
            tcp_arg(newpcb, &clients[i]);
            tcp_recv(newpcb, tcp_server_recv);
            tcp_err(newpcb, tcp_server_error);
            tcp_poll(newpcb, tcp_server_poll, 4);

            uint8_t auth_packet[2 + AUTH_CHALLENGE_LENGTH];
            auth_packet[0] = 0xAA;
            auth_packet[1] = 0x55;
            memcpy(&auth_packet[2], clients[i].auth.challenge, AUTH_CHALLENGE_LENGTH);
            
            err_t write_err = tcp_write(newpcb, auth_packet, sizeof(auth_packet), TCP_WRITE_FLAG_COPY);
            if (write_err == ERR_OK) {
                tcp_output(newpcb);
                LOG_INFO("[TCP] Client %d connected, auth challenge sent (session=%lu)", 
                       i, (unsigned long)clients[i].auth.session_id);
            } else {
                LOG_ERROR("[TCP] Failed to send auth challenge: %d", write_err);
            }
            
            osMutexRelease(tcp_mutex);
            return ERR_OK;
        }
    }

    osMutexRelease(tcp_mutex);

    tcp_close(newpcb);
    LOG_WARN("[TCP] Connection rejected (max clients reached)");
    return ERR_ABRT;
}

/**
 * @brief TCP数据接收回调函数
 * @details 处理从客户端接收到的数据，解析电机命令并执行相应操作
 * @param arg 用户自定义参数（指向客户端结构体）
 * @param tpcb TCP连接控制块
 * @param p 接收到的数据缓冲区
 * @param err 接收错误状态
 * @return ERR_OK 处理成功，其他值表示错误
 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TcpClient_t* client = (TcpClient_t*)arg;

    if (err != ERR_OK) {
        if (p != NULL) pbuf_free(p);
        return err;
    }

    if (p == NULL) {
        client->active = false;
        Security_Reset(&client->auth);
        tcp_close(tpcb);
        printf("[TCP] 客户端已断开连接\n");
        return ERR_OK;
    }

    if (client->auth.state == AUTH_STATE_CHALLENGE_SENT) {
        if (p->tot_len >= AUTH_KEY_LENGTH) {
            uint8_t response[AUTH_KEY_LENGTH];
            pbuf_copy_partial(p, response, AUTH_KEY_LENGTH, 0);
            
            if (Security_Authenticate(response, AUTH_KEY_LENGTH)) {
                client->auth.state = AUTH_STATE_AUTHENTICATED;
                client->auth.failed_attempts = 0;
                Security_UpdateActivity(&client->auth);
                
                const char* auth_ok = "AUTH_OK";
                if (tcp_mutex != NULL) osMutexAcquire(tcp_mutex, portMAX_DELAY);
                tcp_write(tpcb, auth_ok, strlen(auth_ok), TCP_WRITE_FLAG_COPY);
                tcp_output(tpcb);
                if (tcp_mutex != NULL) osMutexRelease(tcp_mutex);
                
                printf("[TCP] 客户端认证成功 (session=%lu)\n", 
                       (unsigned long)client->auth.session_id);
            } else {
                client->auth.failed_attempts++;
                printf("[TCP] 认证失败，尝试次数: %d\n", client->auth.failed_attempts);
                
                if (client->auth.failed_attempts >= AUTH_MAX_FAILED_ATTEMPTS) {
                    client->auth.state = AUTH_STATE_BLOCKED;
                    const char* auth_block = "AUTH_BLOCKED";
                    if (tcp_mutex != NULL) osMutexAcquire(tcp_mutex, portMAX_DELAY);
                    tcp_write(tpcb, auth_block, strlen(auth_block), TCP_WRITE_FLAG_COPY);
                    tcp_output(tpcb);
                    if (tcp_mutex != NULL) osMutexRelease(tcp_mutex);
                    
                    printf("[TCP] 客户端被阻止 (session=%lu)\n", 
                           (unsigned long)client->auth.session_id);
                } else {
                    const char* auth_fail = "AUTH_FAIL";
                    if (tcp_mutex != NULL) osMutexAcquire(tcp_mutex, portMAX_DELAY);
                    tcp_write(tpcb, auth_fail, strlen(auth_fail), TCP_WRITE_FLAG_COPY);
                    tcp_output(tpcb);
                    if (tcp_mutex != NULL) osMutexRelease(tcp_mutex);
                }
            }
            
            tcp_recved(tpcb, p->tot_len);
            pbuf_free(p);
            return ERR_OK;
        }
        
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    if (client->auth.state == AUTH_STATE_BLOCKED) {
        printf("[TCP] 拒绝被阻止客户端的数据\n");
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    if (!Security_IsAuthenticated(&client->auth)) {
        printf("[TCP] 拒绝未认证客户端的数据\n");
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    Security_UpdateActivity(&client->auth);

    if (p->tot_len >= 2 && p->tot_len <= 64) {
        char text_buffer[65];
        pbuf_copy_partial(p, text_buffer, MIN(p->tot_len, 64), 0);
        text_buffer[MIN(p->tot_len, 64)] = '\0';

        if (strncmp(text_buffer, "ok", 2) == 0) {
            printf("[TCP] 收到上位机回复: %s\n", text_buffer);
            client->rx_packets++;
            total_rx_packets++;
            tcp_recved(tpcb, p->tot_len);
            pbuf_free(p);
            return ERR_OK;
        }
    }

    uint16_t data_offset = 0;
    char text_buffer[65];
    pbuf_copy_partial(p, text_buffer, MIN(p->tot_len, 64), 0);
    if (p->tot_len >= 50) {
        if (text_buffer[0] == 'o' && text_buffer[1] == 'k') {
            data_offset = 2;
            printf("[TCP Client] 检测到'ok'前缀，跳过2字节\r\n");
        }
    }
    
    printf("[TCP Client] RECV: checking 48-byte handler, p->tot_len=%d, offset=%d\r\n", p->tot_len, data_offset);
    if (p->tot_len >= 48 + data_offset) {
        float target_angles[12];
        pbuf_copy_partial(p, target_angles, sizeof(target_angles), data_offset);

        /* 验证角度范围 - 按关节类型限制 (V11.0: 统一为0-359度) */
        bool valid = true;
        for (uint8_t i = 0; i < 12; i++) {
            QuadJointID_t joint_id = QUAD_GET_JOINT_ID(i);
            float min_angle = 0.0f;
            float max_angle = 359.0f;

            switch (joint_id) {
                case JOINT_HIP:
                    min_angle = 0.0f;
                    max_angle = 359.0f;
                    break;
                case JOINT_THIGH:
                    min_angle = 0.0f;
                    max_angle = 359.0f;
                    break;
                case JOINT_CALF:
                    min_angle = 0.0f;
                    max_angle = 359.0f;
                    break;
                default:
                    min_angle = 0.0f;
                    max_angle = 359.0f;
                    break;
            }

            if (target_angles[i] > max_angle || target_angles[i] < min_angle ||
                isnan(target_angles[i]) || isinf(target_angles[i])) {
                valid = false;
                printf("[TCP] 角度超出限制: motor[%d] joint=%d value=%.2f range=[%.1f, %.1f]\n",
                       i, joint_id, target_angles[i], min_angle, max_angle);
                break;
            }
        }

        if (valid) {
            uint8_t result = QuadControlLoop_SetTargetAngles(target_angles);
            if (result == 0) {
                printf("[TCP] 收到目标角度更新 (48 bytes)\n");
                client->rx_packets++;
                total_rx_packets++;
            } else {
                printf("[TCP] 设置目标角度失败: %d\n", result);
            }
        } else {
            const char* error_msg = "DATA_INVALID";
            uint8_t validate_result = 0;
            
            for (int j = 0; j < QUAD_MAX_MOTORS; j++) {
                if (isnan(target_angles[j])) {
                    validate_result = VALIDATE_ERR_NAN;
                    break;
                }
                if (isinf(target_angles[j])) {
                    validate_result = VALIDATE_ERR_INF;
                    break;
                }
                validate_result = VALIDATE_ERR_RANGE;
            }
            
            switch (validate_result) {
                case VALIDATE_ERR_NAN:
                    error_msg = "DATA_NAN";
                    printf("[TCP] 角度数据包含NaN\n");
                    break;
                case VALIDATE_ERR_INF:
                    error_msg = "DATA_INF";
                    printf("[TCP] 角度数据包含Inf\n");
                    break;
                case VALIDATE_ERR_RANGE:
                    error_msg = "DATA_RANGE";
                    printf("[TCP] 角度数据超出范围\n");
                    break;
                default:
                    printf("[TCP] 角度数据验证失败: %d\n", validate_result);
                    break;
            }
            if (tcp_mutex != NULL) osMutexAcquire(tcp_mutex, portMAX_DELAY);
            tcp_write(tpcb, error_msg, strlen(error_msg), TCP_WRITE_FLAG_COPY);
            tcp_output(tpcb);
            if (tcp_mutex != NULL) osMutexRelease(tcp_mutex);
        }
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

/**
 * @brief TCP连接错误处理回调函数
 * @details 处理TCP连接中的各种错误情况，清理客户端资源
 * @param arg 用户自定义参数（指向客户端结构体）
 * @param err 错误代码
 */
static void tcp_server_error(void *arg, err_t err)
{
    osMutexAcquire(tcp_mutex, portMAX_DELAY);
    
    TcpClient_t* client = (TcpClient_t*)arg;
    if (client) {
        /* 验证客户端指针是否指向有效的clients数组元素 */
        bool valid_client = false;
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (&clients[i] == client) {
                valid_client = true;
                client->active = false;
                printf("[TCP] 客户端 %d 连接错误: %d\n", i, err);
                break;
            }
        }
        
        if (!valid_client) {
            printf("[TCP] 接收到无效客户端指针\n");
        }
    }
    
    osMutexRelease(tcp_mutex);
}

/**
 * @brief TCP轮询回调函数
 * @details 定期检查TCP连接状态，执行心跳检测
 * @param arg 用户自定义参数（指向客户端结构体）
 * @param tpcb TCP连接控制块
 * @return ERR_OK 正常
 * @note V2.1: 添加互斥锁保护，防止LwIP断言失败
 */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    TcpClient_t* client = (TcpClient_t*)arg;
    if (client == NULL || !client->active) {
        return ERR_OK;
    }
    
    uint32_t now = HAL_GetTick();
    HeartbeatState_t* hb = &client->heartbeat;
    
    if (!hb->enabled) {
        return ERR_OK;
    }
    
    if (hb->waiting_pong) {
        if ((now - hb->last_heartbeat_tx) > TCP_HEARTBEAT_TIMEOUT_MS) {
            hb->missed_count++;
            hb->waiting_pong = false;
            LOG_WARN("[TCP] Heartbeat timeout, missed_count=%d", hb->missed_count);

            if (hb->missed_count >= TCP_HEARTBEAT_MAX_MISSED) {
                LOG_ERROR("[TCP] Max heartbeat misses reached, closing connection");
                client->active = false;
                Security_Reset(&client->auth);
                tcp_abort(tpcb);
                return ERR_ABRT;
            }
        }
    } else {
        if ((now - hb->last_heartbeat_tx) > TCP_HEARTBEAT_INTERVAL_MS) {
            if (tcp_mutex != NULL) {
                osMutexAcquire(tcp_mutex, portMAX_DELAY);
            }
            err_t err = tcp_write(tpcb, HEARTBEAT_PING, strlen(HEARTBEAT_PING), TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(tpcb);
                hb->last_heartbeat_tx = now;
                hb->waiting_pong = true;
                LOG_DEBUG("[TCP] Heartbeat PING sent");
            }
            if (tcp_mutex != NULL) {
                osMutexRelease(tcp_mutex);
            }
        }
    }
    
    return ERR_OK;
}

/* ============== 心跳处理函数 ============== */

/**
 * @brief 处理接收到的心跳响应
 * @param client 客户端结构体指针
 * @param is_pong 是否为PONG响应
 */
static void tcp_handle_heartbeat_response(TcpClient_t* client, bool is_pong)
{
    if (client == NULL) return;
    
    HeartbeatState_t* hb = &client->heartbeat;
    
    if (is_pong && hb->waiting_pong) {
        hb->last_heartbeat_rx = HAL_GetTick();
        hb->missed_count = 0;
        hb->waiting_pong = false;
        LOG_DEBUG("[TCP] Heartbeat PONG received");
    }
}

/**
 * @brief 初始化心跳状态
 * @param hb 心跳状态结构体指针
 */
static void tcp_heartbeat_init(HeartbeatState_t* hb)
{
    if (hb == NULL) return;
    
    hb->last_heartbeat_tx = HAL_GetTick();
    hb->last_heartbeat_rx = HAL_GetTick();
    hb->missed_count = 0;
    hb->waiting_pong = false;
    hb->enabled = true;
}

/**
 * @brief 检查连接健康状态（公共API）
 * @return true 健康, false 不健康
 */
bool tcp_client_is_healthy(void)
{
    return tcp_client_is_connected();
}

/**
 * @brief 获取心跳统计信息
 * @param missed_count 丢失心跳次数输出
 * @param last_rx_time 上次接收心跳时间输出
 */
void tcp_get_heartbeat_stats(uint8_t* missed_count, uint32_t* last_rx_time)
{
    if (missed_count != NULL) {
        *missed_count = 0;
    }
    if (last_rx_time != NULL) {
        *last_rx_time = 0;
    }
    
    osMutexAcquire(tcp_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active) {
            if (missed_count != NULL) {
                *missed_count = clients[i].heartbeat.missed_count;
            }
            if (last_rx_time != NULL) {
                *last_rx_time = clients[i].heartbeat.last_heartbeat_rx;
            }
            break;
        }
    }
    
    osMutexRelease(tcp_mutex);
}

/**
 * @brief 计算数据校验和
 * @details 使用简单的异或运算计算数据包的校验和，用于数据完整性验证
 * @param data 指向待校验数据的指针
 * @param len 数据长度（字节数）
 * @return 计算得到的校验和（8位）
 */
static uint8_t calculate_checksum(uint8_t* data, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

/* V9.0: 发送所有12个电机的状态上报给上位机 */
uint8_t tcp_motor_server_send_all_motor_status(void)
{
    uint8_t packet_buffer[32];
    EnhancedMotorFeedback_t feedback;
    MotorHandle* handle;
    uint8_t sent_count = 0;

    for (uint8_t i = 0; i < QUAD_MAX_MOTORS; i++) {
        QuadControlLoop_GetMotorFeedback(i, &feedback);
        handle = motor_handle_get(i);

        uint16_t idx = 0;
        packet_buffer[idx++] = 0xAA;
        packet_buffer[idx++] = 0x55;
        packet_buffer[idx++] = 0x04;

        uint32_t timestamp = HAL_GetTick();
        memcpy(&packet_buffer[idx], &timestamp, 4); idx += 4;
        packet_buffer[idx++] = i;
        
        float position = feedback.actual_position;
        float velocity = feedback.actual_velocity;
        float torque = feedback.actual_torque;
        float current = (handle != NULL) ? handle->current : 0.0f;
        
        memcpy(&packet_buffer[idx], &position, 4); idx += 4;
        memcpy(&packet_buffer[idx], &velocity, 4); idx += 4;
        memcpy(&packet_buffer[idx], &torque, 4); idx += 4;
        memcpy(&packet_buffer[idx], &current, 4); idx += 4;

        uint8_t checksum = 0;
        for (uint16_t j = 0; j < idx; j++) {
            checksum += packet_buffer[j];
        }
        packet_buffer[idx++] = checksum;

        if (tcp_client_send_data(packet_buffer, idx)) {
            sent_count++;
        }
    }

    return sent_count;
}

/* ============== TCP客户端功能实现 ============== */

/**
 * @brief TCP客户端连接成功回调函数
 */
static err_t tcp_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    printf("[TCP Client] ========== Connected callback triggered ==========\n");
    printf("[TCP Client] Callback: err=%d, tpcb=0x%p, tpcb->state=%d, rcv_wnd=%d\n", 
           err, tpcb, tpcb ? tpcb->state : -1, tpcb ? tpcb->rcv_wnd : 0);
    
    printf("[TCP Client] client_pcb at connect callback: %p\n", client_pcb);
    printf("[TCP Client] client_pcb->recv: %p\n", client_pcb ? client_pcb->recv : 0);

    if (err != ERR_OK) {
        printf("[TCP Client] Connection failed: %d\n", err);
        tcp_client_set_connected(false);
        return err;
    }

    tcp_client_set_connected(true);
    printf("[TCP Client] Connected successfully to remote host\n");
    printf("[TCP Client] client_connected flag set to TRUE (with mutex protection)\n");

    /* 回调函数已在tcp_client_connect_tcpip_thread中设置 */
    return ERR_OK;
}

/**
 * @brief TCP客户端接收回调函数
 * @note V3.3: 增加帧分片处理状态机，支持不完整帧的累积
 */
static err_t tcp_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    LOG_RAW("[TCP Client] RECV callback triggered: p=%p, err=%d\r\n", p, err);
    LOG_RAW("[TCP Client] RECV: p->tot_len = %d, tcp_wnd = %d\r\n", p ? p->tot_len : 0, tpcb ? tpcb->rcv_wnd : 0);

    if (err != ERR_OK) {
        LOG_RAW("[TCP Client] RECV callback: err != ERR_OK, err=%d\r\n", err);
        if (p != NULL) pbuf_free(p);
        return err;
    }

    if (p == NULL) {
        tcp_client_set_connected(false);
        /* V3.5: 互斥锁保护client_pcb写操作，消除竞态条件 */
        if (tcp_client_mutex != NULL) {
            osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
        }
        tcp_close(tpcb);
        client_pcb = NULL;
        if (tcp_client_mutex != NULL) {
            osMutexRelease(tcp_client_mutex);
        }
        LOG_RAW("[TCP Client] Connection closed by remote host\n");
        return ERR_OK;
    }

    LOG_RAW("\r\n========================================\r\n");
    LOG_RAW("[TCP Client] Received %d bytes from Jetson\r\n", p->tot_len);

    /* V3.3: 使用状态机处理帧分片 */
    static uint8_t rx_buffer[512];
    uint16_t copy_len = (p->tot_len < 511) ? p->tot_len : 511;
    pbuf_copy_partial(p, rx_buffer, copy_len, 0);

    LOG_RAW("[TCP Client] HEX: ");
    for (int i = 0; i < copy_len && i < 64; i++) {
        LOG_RAW("%02X ", rx_buffer[i]);
        if ((i + 1) % 16 == 0) LOG_RAW("\r\n              ");
    }
    if (copy_len > 64) {
        LOG_RAW("\r\n              ... (%d more bytes)", copy_len - 64);
    }
    LOG_RAW("\r\n");

    float target_angles[12];
    memset(target_angles, 0, sizeof(target_angles));
    bool valid = false;
    uint16_t payload_offset = 0;

    /* V3.0: 处理PONG心跳响应 */
    if (copy_len >= 4 && strncmp((char*)rx_buffer, "PONG", 4) == 0) {
        LOG_RAW("[TCP Client] 收到PONG心跳响应\r\n");
        g_heartbeat_pong_received = true;
        g_heartbeat_last_pong_tick = HAL_GetTick();
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    /* V3.3: 帧分片处理 - 将数据追加到接收上下文缓冲区 */
    for (uint16_t i = 0; i < copy_len; i++) {
        uint8_t byte = rx_buffer[i];

        switch (g_rx_ctx.state) {
            case RX_STATE_WAIT_HEADER1:
                if (byte == 0xAA) {
                    g_rx_ctx.buffer[0] = byte;
                    g_rx_ctx.buffer_len = 1;
                    g_rx_ctx.state = RX_STATE_WAIT_HEADER2;
                }
                break;

            case RX_STATE_WAIT_HEADER2:
                if (byte == 0x55) {
                    g_rx_ctx.buffer[1] = byte;
                    g_rx_ctx.buffer_len = 2;
                    g_rx_ctx.state = RX_STATE_WAIT_TYPE;
                } else {
                    g_rx_ctx.state = RX_STATE_WAIT_HEADER1;
                }
                break;

            case RX_STATE_WAIT_TYPE:
                g_rx_ctx.buffer[2] = byte;
                g_rx_ctx.buffer_len = 3;
                g_rx_ctx.frame_type = byte;
                g_rx_ctx.state = RX_STATE_WAIT_DATA;

                /* 根据帧类型确定期望长度 */
                if (byte == 0x12) {
                    /* 位置控制命令: 帧头(3) + 长度(2) + 负载(48) + CRC(2) + 帧尾(2) = 57字节 */
                    g_rx_ctx.expected_len = 57;
                } else if (byte == 0x01 || byte == 0x0F || byte == 0x10 || byte == 0x11) {
                    /* 心跳/紧急停止/使能/禁用命令: 最小帧 9字节 */
                    g_rx_ctx.expected_len = 9;
                } else if (byte == 0x17) {
                    /* 数据发送使能命令 */
                    g_rx_ctx.expected_len = 10;
                } else {
                    /* 未知帧类型，重置 */
                    LOG_WARN("[TCP] Unknown frame type: 0x%02X", byte);
                    g_rx_ctx.error_count++;
                    g_rx_ctx.state = RX_STATE_WAIT_HEADER1;
                }
                break;

            case RX_STATE_WAIT_DATA:
                /* V3.4: 检查缓冲区溢出 */
                if (g_rx_ctx.buffer_len >= sizeof(g_rx_ctx.buffer)) {
                    LOG_ERROR("[TCP] RX buffer overflow! expected=%d, max=%d",
                              g_rx_ctx.expected_len, sizeof(g_rx_ctx.buffer));
                    g_rx_ctx.overflow_count++;
                    g_rx_ctx.state = RX_STATE_ERROR;
                    break;
                }
                
                g_rx_ctx.buffer[g_rx_ctx.buffer_len++] = byte;

                /* 检查是否收到完整帧 */
                if (g_rx_ctx.buffer_len >= g_rx_ctx.expected_len) {
                    g_rx_ctx.state = RX_STATE_COMPLETE;
                }
                break;

            case RX_STATE_ERROR:
                /* V3.4: 错误恢复 - 重置状态机 */
                LOG_WARN("[TCP] RX state machine in error state, resetting...");
                g_rx_ctx.state = RX_STATE_WAIT_HEADER1;
                g_rx_ctx.buffer_len = 0;
                g_rx_ctx.error_count++;
                break;

            default:
                g_rx_ctx.state = RX_STATE_WAIT_HEADER1;
                break;
        }

        /* 处理完整帧 */
        if (g_rx_ctx.state == RX_STATE_COMPLETE) {
            LOG_RAW("[TCP Client] 完整帧接收完成: type=0x%02X, len=%d\r\n",
                   g_rx_ctx.frame_type, g_rx_ctx.buffer_len);

            /* 处理帧数据（复制到rx_buffer供后续处理） */
            memcpy(rx_buffer, g_rx_ctx.buffer, g_rx_ctx.buffer_len);
            copy_len = g_rx_ctx.buffer_len;

            /* 重置状态机 */
            g_rx_ctx.state = RX_STATE_WAIT_HEADER1;
            g_rx_ctx.buffer_len = 0;

            /* 跳出循环，进入帧处理逻辑 */
            break;
        }
    }

    /* 如果状态机还在等待更多数据，继续等待 */
    if (g_rx_ctx.state != RX_STATE_WAIT_HEADER1) {
        LOG_RAW("[TCP Client] 帧分片: 等待更多数据 (current=%d, expected=%d)\r\n",
               g_rx_ctx.buffer_len, g_rx_ctx.expected_len);
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    /* 以下为原有的帧处理逻辑 */
    if (copy_len >= 9 && rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55) {
        uint8_t cmd_type = rx_buffer[2];
        uint16_t payload_len = (uint16_t)rx_buffer[3] | ((uint16_t)rx_buffer[4] << 8);
        
        LOG_RAW("[TCP Client] 协议帧: cmd=0x%02X, payload_len=%d\r\n", cmd_type, payload_len);
        
        uint16_t frame_len = 5 + payload_len + 2 + 2;
        if (copy_len >= frame_len) {
            uint16_t crc_offset = 5 + payload_len;
            uint16_t recv_crc = (uint16_t)rx_buffer[crc_offset] | ((uint16_t)rx_buffer[crc_offset + 1] << 8);
            
            uint16_t calc_crc = 0xFFFF;
            for (int i = 0; i < crc_offset; i++) {
                calc_crc ^= rx_buffer[i];
                for (int j = 0; j < 8; j++) {
                    if (calc_crc & 0x0001) {
                        calc_crc = (calc_crc >> 1) ^ 0xA001;
                    } else {
                        calc_crc >>= 1;
                    }
                }
            }
            
            if (calc_crc == recv_crc) {
                LOG_RAW("[TCP Client] CRC16校验通过\r\n");
                
                switch (cmd_type) {
                    case 0x01:
                        LOG_RAW("[TCP Client] 收到心跳命令\r\n");
                        break;
                        
                    case 0x0F:
                        LOG_RAW("[TCP Client] 收到紧急停止命令!\r\n");
                        QuadControlLoop_SetEmergencyStop(true);
                        break;
                        
                    case 0x10:
                        LOG_RAW("[TCP Client] 收到电机使能命令\r\n");
                        {
                            MotorSyncResult_t enable_result;
                            int ret = motor_sync_enable_all(&enable_result);
                            if (ret == 0) {
                                LOG_RAW("[TCP Client] 电机使能成功: %d/%d\r\n", 
                                       enable_result.success_count, enable_result.total_motors);
                            } else {
                                LOG_RAW("[TCP Client] 电机使能失败\r\n");
                            }
                        }
                        break;
                        
                    case 0x11:
                        LOG_RAW("[TCP Client] 收到电机禁用命令\r\n");
                        {
                            MotorSyncResult_t disable_result;
                            int ret = motor_sync_disable_all(&disable_result);
                            if (ret == 0) {
                                LOG_RAW("[TCP Client] 电机禁用成功: %d/%d\r\n", 
                                       disable_result.success_count, disable_result.total_motors);
                            } else {
                                LOG_RAW("[TCP Client] 电机禁用失败\r\n");
                            }
                        }
                        break;
                        
                    case 0x12:
                        if (payload_len >= 48) {
                            payload_offset = 5;
                            valid = true;
                            LOG_RAW("[TCP Client] 位置控制命令，负载有效\r\n");
                        } else {
                            LOG_RAW("[TCP Client] 位置控制命令负载长度不足: %d < 48\r\n", payload_len);
                        }
                        break;
                        
                    case 0x17:
                        if (payload_len >= 1) {
                            bool enable = (rx_buffer[5] != 0);
                            LOG_RAW("[TCP Client] 收到数据发送使能命令: %s\r\n", enable ? "启用" : "禁用");
                            motor_sync_set_send_enabled(enable);
                        } else {
                            LOG_RAW("[TCP Client] 数据发送使能命令负载长度不足\r\n");
                        }
                        break;
                        
                    default:
                        LOG_RAW("[TCP Client] 未知命令类型: 0x%02X\r\n", cmd_type);
                        break;
                }
            } else {
                LOG_RAW("[TCP Client] CRC16校验失败: calc=0x%04X, recv=0x%04X\r\n", calc_crc, recv_crc);
            }
        } else {
            LOG_RAW("[TCP Client] 帧不完整: %d < %d\r\n", copy_len, frame_len);
        }
    }
    /* P0-5修复: 移除无CRC校验的裸数据回退路径
     * 原代码在数据不匹配协议帧格式时直接当48字节float数组处理，
     * 无任何完整性校验，对安全关键的四足机器人极其危险 */
    else {
        LOG_WARN("[TCP Client] Unrecognized data format: len=%d, first bytes=%02X %02X",
                 copy_len, copy_len > 0 ? rx_buffer[0] : 0, copy_len > 1 ? rx_buffer[1] : 0);
    }

    if (valid && (copy_len - payload_offset) >= 48) {
        memcpy(target_angles, &rx_buffer[payload_offset], sizeof(target_angles));

        for (uint8_t i = 0; i < 12; i++) {
            if (isnan(target_angles[i]) || isinf(target_angles[i])) {
                valid = false;
                LOG_RAW("[TCP Client] 角度数据无效: motor[%d] NaN或Inf\n", i);
                break;
            }

            if (target_angles[i] >= 360.0f || target_angles[i] < 0.0f) {
                target_angles[i] = fmodf(target_angles[i], 360.0f);
                if (target_angles[i] < 0.0f) {
                    target_angles[i] += 360.0f;
                }
            }

            if (target_angles[i] >= 360.0f) target_angles[i] = 0.0f;
            if (target_angles[i] < 0.0f) target_angles[i] = 0.0f;
        }

        if (valid) {
            LOG_RAW("[TCP Client] 目标角度: ");
            for (int i = 0; i < 12; i++) {
                LOG_RAW("%.1f ", target_angles[i]);
            }
            LOG_RAW("\r\n");
            
            uint8_t result = QuadControlLoop_SetTargetAngles(target_angles);
            if (result == 0) {
                LOG_RAW("[TCP Client] 目标角度设置成功\r\n");
                g_upper_computer_last_rx_tick = HAL_GetTick();
                g_upper_computer_rx_count++;
            } else {
                LOG_RAW("[TCP Client] 设置目标角度失败: %d\n", result);
            }
        }
    }
    LOG_RAW("========================================\r\n");

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

/**
 * @brief TCP客户端错误回调函数
 * @note V3.3: 增强PCB竞态条件处理
 */
static void tcp_client_error_cb(void *arg, err_t err)
{
    printf("[TCP Client] Connection error: %d ", err);
    /* Decode error for debugging */
    switch(err) {
        case ERR_ABRT: printf("(ERR_ABRT - Connection aborted)\n"); break;
        case ERR_RST: printf("(ERR_RST - Connection reset)\n"); break;
        case ERR_CLSD: printf("(ERR_CLSD - Connection closed)\n"); break;
        case ERR_CONN: printf("(ERR_CONN - Not connected)\n"); break;
        case ERR_TIMEOUT: printf("(ERR_TIMEOUT - Timeout)\n"); break;
        default: printf("(Unknown error)\n"); break;
    }

    /* V3.3: 原子操作顺序 - 先设置连接状态，再清空PCB指针 */
    /* 1. 首先设置连接状态为false（原子操作） */
    tcp_client_set_connected(false);

    /* 2. V3.5: 互斥锁保护client_pcb写操作，消除竞态条件 */
    /* IMPORTANT: PCB is already freed by LwIP when this callback is called
     * Do NOT call tcp_abort() or tcp_close() here! */
    if (tcp_client_mutex != NULL) {
        osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
    }
    client_pcb = NULL;
    if (tcp_client_mutex != NULL) {
        osMutexRelease(tcp_client_mutex);
    }

    /* 3. 重置心跳状态 */
    g_heartbeat_pong_received = false;
    g_heartbeat_missed_count = 0;
}

/**
 * @brief TCP客户端轮询回调函数
 * @details 用于检测连接超时，每2秒调用一次
 */
static err_t tcp_client_poll_cb(void *arg, struct tcp_pcb *tpcb)
{
    bool connected = tcp_client_is_connected();
    printf("[TCP Client] Poll callback: state=%d, connected=%d, rcv_wnd=%d\n", 
           tpcb ? tpcb->state : -1, connected, tpcb ? tpcb->rcv_wnd : 0);
    
    if (tpcb == NULL || !connected) {
        printf("[TCP Client] Poll: connection not established, will retry...\n");
    }
    
    return ERR_OK;
}

/**
 * @brief V3.4: 分配连接请求结构（从连接池）
 * @return 可用的连接请求结构指针，或NULL（池已满）
 */
static TcpClientConnectParams_t* alloc_connect_request(void)
{
    if (g_connect_pool_mutex == NULL) {
        g_connect_pool_mutex = osMutexNew(NULL);
        if (g_connect_pool_mutex == NULL) {
            printf("[TCP Client] Failed to create connect pool mutex\n");
            return NULL;
        }
    }
    
    osMutexAcquire(g_connect_pool_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_CONNECT_REQUESTS; i++) {
        if (!g_connect_requests[i].in_use) {
            g_connect_requests[i].in_use = true;
            memset(&g_connect_requests[i], 0, sizeof(TcpClientConnectParams_t));
            g_connect_requests[i].in_use = true;  /* 重新设置，因为memset清除了 */
            osMutexRelease(g_connect_pool_mutex);
            printf("[TCP Client] Allocated connect request slot %d\n", i);
            return &g_connect_requests[i];
        }
    }
    
    osMutexRelease(g_connect_pool_mutex);
    printf("[TCP Client] No available connect request slot\n");
    return NULL;
}

/**
 * @brief V3.4: 释放连接请求结构（归还到连接池）
 * @param req 要释放的连接请求结构指针
 */
static void free_connect_request(TcpClientConnectParams_t* req)
{
    if (req == NULL) return;
    
    osMutexAcquire(g_connect_pool_mutex, portMAX_DELAY);
    
    if (req->done_sem != NULL) {
        osSemaphoreDelete(req->done_sem);
        req->done_sem = NULL;
    }
    req->in_use = false;
    
    osMutexRelease(g_connect_pool_mutex);
    printf("[TCP Client] Freed connect request slot\n");
}

/**
 * @brief 在tcpip线程中执行TCP连接操作（线程安全）
 * @details 这个函数会被tcpip_callback_with_block()调用，确保TCP操作在正确的线程中执行
 * @param arg 指向TcpClientConnectParams_t结构的指针
 */
static void tcp_client_connect_tcpip_thread(void *arg)
{
    TcpClientConnectParams_t *params = (TcpClientConnectParams_t *)arg;

    printf("[TCP Client] ========== tcpip thread starting ==========\n");
    printf("[TCP Client] tcpip thread: remote_addr=0x%08X, port=%u\n",
           (unsigned int)params->remote_addr.addr, params->remote_port);

    /* 验证网络字节序 */
    uint8_t *ip_bytes = (uint8_t *)&params->remote_addr.addr;
    printf("[TCP Client] tcpip thread: IP bytes = %u.%u.%u.%u\n",
           ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);

    /* CRITICAL: Send ARP request FIRST to populate ARP table */
    extern struct netif gnetif;
    printf("[TCP Client] tcpip thread: Sending ARP request...\n");
    etharp_request(&gnetif, &params->remote_addr);

    /* CRITICAL: Check if client_pcb is already set */
    printf("[TCP Client] tcpip thread: client_pcb before cleanup = 0x%p\n", (void*)client_pcb);

    /* 如果已经连接，先断开 */
    /* P0-4修复: tcp_abort会触发error_cb同步调用，error_cb也要获取tcp_client_mutex
     * 如果在此处持有mutex则死锁（mutex非递归）。解决方案：先在mutex内保存旧PCB指针，
     * 释放mutex后再调用tcp_abort */
    struct tcp_pcb* old_pcb = NULL;
    if (tcp_client_mutex != NULL) {
        osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
    }
    if (client_pcb != NULL) {
        printf("[TCP Client] tcpip thread: Aborting existing connection (PCB=0x%p)\n", client_pcb);
        old_pcb = client_pcb;
        client_pcb = NULL;
        tcp_client_set_connected(false);
    }
    if (tcp_client_mutex != NULL) {
        osMutexRelease(tcp_client_mutex);
    }
    /* 在mutex之外安全地abort旧PCB */
    if (old_pcb != NULL) {
        tcp_abort(old_pcb);
        printf("[TCP Client] tcpip thread: Old connection aborted\n");
    }

    /* 创建新的TCP控制块 */
    client_pcb = tcp_new();
    if (client_pcb == NULL) {
        printf("[TCP Client] Failed to create PCB in tcpip thread\n");
        params->result = ERR_MEM;
        /* Signal completion */
        if (params->done_sem != NULL) {
            osSemaphoreRelease(params->done_sem);
        }
        return;
    }
    printf("[TCP Client] tcpip thread: PCB created successfully\n");

    /* 绑定到本地任意端口 */
    err_t bind_err = tcp_bind(client_pcb, IP_ADDR_ANY, 0);
    if (bind_err != ERR_OK) {
        printf("[TCP Client] Bind failed: %d\n", bind_err);
        tcp_abort(client_pcb);
        /* V3.5: 互斥锁保护client_pcb写操作 */
        if (tcp_client_mutex != NULL) {
            osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
        }
        client_pcb = NULL;
        if (tcp_client_mutex != NULL) {
            osMutexRelease(tcp_client_mutex);
        }
        params->result = bind_err;
        /* Signal completion */
        if (params->done_sem != NULL) {
            osSemaphoreRelease(params->done_sem);
        }
        return;
    }
    printf("[TCP Client] tcpip thread: Bind successful (local_port=%u)\n", client_pcb->local_port);

    /* CRITICAL: Set callbacks BEFORE tcp_connect() to ensure they are ready */
    tcp_recv(client_pcb, tcp_client_recv_cb);
    printf("[TCP Client] recv callback set: %p\r\n", tcp_client_recv_cb);
    tcp_err(client_pcb, tcp_client_error_cb);
    
    /* 添加轮询回调用于检测连接超时 */
    tcp_poll(client_pcb, tcp_client_poll_cb, 4);  /* 每4*500ms = 2秒检查一次 */

    /* Set argument for callbacks */
    tcp_arg(client_pcb, NULL);

    printf("[TCP Client] Connecting to %s:%d (in tcpip thread)...\n",
           ip4addr_ntoa(&params->remote_addr), params->remote_port);

    /* 连接到远程主机 */
    err_t err = tcp_connect(client_pcb, &params->remote_addr, params->remote_port, tcp_client_connected_cb);
    if (err != ERR_OK) {
        printf("[TCP Client] tcp_connect() failed: %d\n", err);
        if (err == ERR_ISCONN) {
            printf("[TCP Client] Already connected (ERR_ISCONN)\n");
        } else if (err == ERR_VAL) {
            printf("[TCP Client] Invalid PCB state (ERR_VAL)\n");
        } else if (err == ERR_RTE) {
            printf("[TCP Client] No route to host (ERR_RTE)\n");
        }
        tcp_abort(client_pcb);
        /* V3.5: 互斥锁保护client_pcb写操作 */
        if (tcp_client_mutex != NULL) {
            osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
        }
        client_pcb = NULL;
        if (tcp_client_mutex != NULL) {
            osMutexRelease(tcp_client_mutex);
        }
        params->result = err;
        /* Signal completion */
        if (params->done_sem != NULL) {
            osSemaphoreRelease(params->done_sem);
        }
        return;
    }
    printf("[TCP Client] tcpip thread: tcp_connect() returned ERR_OK\n");

    /* Print PCB state for debugging */
    printf("[TCP Client] tcpip thread: PCB state = %d\n", client_pcb->state);

    /* CRITICAL: Force TCP output to send SYN packet immediately */
    err_t output_err = tcp_output(client_pcb);
    if (output_err == ERR_OK) {
        printf("[TCP Client] tcpip thread: tcp_output() successful (SYN should be sent)\n");
    } else {
        printf("[TCP Client] tcpip thread: tcp_output() failed: %d\n", output_err);
    }

    /* Check PCB state again after output */
    printf("[TCP Client] tcpip thread: PCB state after output = %d\n", client_pcb->state);

    params->result = ERR_OK;
    printf("[TCP Client] ========== tcpip thread completed ==========\n");

    /* Signal completion to main thread */
    if (params->done_sem != NULL) {
        osSemaphoreRelease(params->done_sem);
    }
}

/**
 * @brief 初始化TCP客户端，连接到指定的上位机
 * @param remote_ip 上位机IP地址字符串 (例如: "192.168.1.20")
 * @param remote_port 上位机端口号
 * @return true 连接成功，false 连接失败
 * @note V3.4: 使用连接请求池，避免静态变量竞态条件
 */
bool tcp_client_connect(const char* remote_ip, uint16_t remote_port)
{
    static osMutexId_t connect_mutex = NULL;
    static bool connect_in_progress = false;

    if (connect_mutex == NULL) {
        connect_mutex = osMutexNew(NULL);
        if (connect_mutex == NULL) {
            printf("[TCP Client] Failed to create connect mutex\n");
            return false;
        }
    }

    if (osMutexAcquire(connect_mutex, 100) != osOK) {
        printf("[TCP Client] Connection already in progress, skipping...\n");
        return false;
    }

    if (connect_in_progress) {
        printf("[TCP Client] Previous connection still in progress\n");
        osMutexRelease(connect_mutex);
        return false;
    }

    connect_in_progress = true;

    if (tcp_client_mutex == NULL) {
        tcp_client_mutex = osMutexNew(NULL);
        if (tcp_client_mutex == NULL) {
            printf("[TCP Client] Failed to create mutex\n");
            connect_in_progress = false;
            osMutexRelease(connect_mutex);
            return false;
        }
    }

    bool need_cleanup = (client_pcb != NULL);
    osMutexRelease(connect_mutex);

    if (need_cleanup) {
        printf("[TCP Client] Waiting for old connection to cleanup...\n");
        osDelay(1000);
    }

    /* V3.4: 使用连接请求池分配参数结构 */
    TcpClientConnectParams_t* params = alloc_connect_request();
    if (params == NULL) {
        printf("[TCP Client] No available connect request slot\n");
        connect_in_progress = false;
        return false;
    }

    params->done_sem = osSemaphoreNew(1, 0, NULL);
    if (params->done_sem == NULL) {
        printf("[TCP Client] Failed to create done semaphore\n");
        free_connect_request(params);
        connect_in_progress = false;
        return false;
    }

    if (!ip4addr_aton(remote_ip, &params->remote_addr)) {
        printf("[TCP Client] Invalid IP address: %s\n", remote_ip);
        free_connect_request(params);
        connect_in_progress = false;
        return false;
    }

    params->remote_port = remote_port;
    params->result = ERR_VAL;

    printf("[TCP Client] Initiating connection to %s:%d\n", remote_ip, remote_port);
    printf("[TCP Client]   IP addr (network byte order): 0x%08X\n", (unsigned int)params->remote_addr.addr);

    uint8_t *ip_bytes = (uint8_t *)&params->remote_addr.addr;
    printf("[TCP Client]   Byte representation (LE): %u.%u.%u.%u\n",
           ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);

    if (params->remote_addr.addr == 0x1401A8C0 || params->remote_addr.addr == 0xC0A80114) {
        printf("[TCP Client]   ✓ IP address looks correct\n");
    } else {
        printf("[TCP Client]   ✗ WARNING: Unexpected IP value!\n");
    }

    printf("[TCP Client] Calling tcpip_callback_with_block...\n");
    err_t err = tcpip_callback_with_block(tcp_client_connect_tcpip_thread, params, 1);
    printf("[TCP Client] tcpip_callback_with_block returned: %d\n", err);

    if (err != ERR_OK) {
        printf("[TCP Client] tcpip_callback_with_block failed: %d\n", err);
        free_connect_request(params);
        connect_in_progress = false;
        return false;
    }

    printf("[TCP Client] Waiting for tcpip thread to complete...\n");
    osStatus_t sem_status = osSemaphoreAcquire(params->done_sem, 3000);
    if (sem_status != osOK) {
        printf("[TCP Client] Timeout waiting for tcpip thread! status=%d\n", sem_status);
        free_connect_request(params);
        connect_in_progress = false;
        return false;
    }
    printf("[TCP Client] tcpip thread completed\n");

    err_t result = params->result;
    free_connect_request(params);
    connect_in_progress = false;

    printf("[TCP Client] Checking result: %d\n", result);
    if (result != ERR_OK) {
        printf("[TCP Client] Connection initiation failed: %d\n", result);
        return false;
    }

    printf("[TCP Client] Connection initiated successfully (waiting for callback)\n");
    return true;
}

/**
 * @brief 通过TCP客户端发送文本消息到上位机
 * @param message 要发送的文本消息字符串
 * @return true 发送成功，false 发送失败
 * @note V2.2: 使用tcpip_callback确保在正确的线程中执行TCP操作
 */
bool tcp_client_send_text(const char* message)
{
    if (message == NULL) return false;
    size_t msg_len = strlen(message);
    if (msg_len == 0 || msg_len > TCP_SEND_BUFFER_SIZE) return false;
    
    return tcp_client_send_data((uint8_t*)message, (uint16_t)msg_len);
}

/**
 * @brief 在tcpip线程中执行TCP发送（内部回调函数）
 * @param arg PCB指针（通过tcpip_try_callback传递）
 * @note V3.4: 使用传递的PCB指针而非全局变量，避免TOCTOU竞态
 */
static void tcp_client_send_tcpip_thread(void *arg)
{
    struct tcp_pcb* pcb = (struct tcp_pcb*)arg;
    /* P0-3修复: 使用读取索引(与写入索引互补)，确保回调读取的是稳定数据 */
    uint8_t read_idx = tcp_send_write_idx ^ 1;

    /* V3.5: 使用传入的pcb指针而非全局client_pcb，避免竞态 */
    if (pcb == NULL) {
        tcp_async_fail_count++;
        return;
    }

    /* V3.5: 先检查PCB状态（无锁），此函数在tcpip线程中运行 */
    if (pcb->state != ESTABLISHED) {
        tcp_async_fail_count++;
        return;
    }

    osMutexAcquire(tcp_client_mutex, portMAX_DELAY);

    /* V3.5: 验证连接状态（不再比较client_pcb指针，因为error_cb可能已置NULL） */
    if (!client_connected) {
        tcp_async_fail_count++;
        osMutexRelease(tcp_client_mutex);
        return;
    }

    err_t err = tcp_write(pcb, tcp_send_buffers[read_idx], tcp_send_lens[read_idx], TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(pcb);
        total_tx_packets++;
        tcp_async_send_count++;
    } else {
        total_tx_errors++;
        tcp_async_fail_count++;
        if (err != ERR_MEM) {
            tcp_client_set_connected(false);
        }
    }

    osMutexRelease(tcp_client_mutex);
}

/**
 * @brief 通过TCP客户端发送二进制数据到上位机（异步）
 * @param data 要发送的数据指针
 * @param len 数据长度（字节）
 * @return true 发送请求已提交，false 发送失败
 * @note V3.4: 使用局部变量保存PCB指针并传递给回调，避免TOCTOU竞态
 */
bool tcp_client_send_data(uint8_t* data, uint16_t len)
{
    static uint32_t last_warn_tick = 0;
    static uint32_t wouldblock_count = 0;
    struct tcp_pcb* pcb_snapshot = NULL;

    if (tcp_client_mutex == NULL || data == NULL || len == 0) return false;
    if (len > TCP_SEND_BUFFER_SIZE) {
        printf("[TCP] ERROR: Data too large: %u > %u\n", len, TCP_SEND_BUFFER_SIZE);
        return false;
    }

    osMutexAcquire(tcp_client_mutex, portMAX_DELAY);

    /* V3.4: 使用局部变量保存PCB指针快照，避免后续访问全局变量 */
    if (!client_connected || client_pcb == NULL) {
        osMutexRelease(tcp_client_mutex);
        return false;
    }

    pcb_snapshot = client_pcb;

    /* V3.4: 检查PCB状态是否有效（避免访问已释放的PCB） */
    if (pcb_snapshot->state != ESTABLISHED) {
        printf("[TCP] PCB state not ESTABLISHED: %d, skipping send\n", pcb_snapshot->state);
        osMutexRelease(tcp_client_mutex);
        return false;
    }

    u16_t snd_buf = tcp_sndbuf(pcb_snapshot);
    if (snd_buf < len) {
        if (HAL_GetTick() - last_warn_tick > 1000) {
            printf("[TCP] WARN: Send buffer low: %u bytes available, need %u\n", snd_buf, len);
            last_warn_tick = HAL_GetTick();
        }
    }

    memcpy(tcp_send_buffers[tcp_send_write_idx], data, len);
    tcp_send_lens[tcp_send_write_idx] = len;
    /* P0-3修复: 切换写入索引，使tcpip回调读取刚写入的buffer（互补索引） */
    uint8_t send_idx = tcp_send_write_idx;
    tcp_send_write_idx ^= 1;

    /* V3.4: 将PCB指针快照传递给回调函数，而非NULL */
    err_t err = tcpip_try_callback(tcp_client_send_tcpip_thread, pcb_snapshot);
    if (err == ERR_WOULDBLOCK) {
        tcp_async_fail_count++;
        wouldblock_count++;
        if (HAL_GetTick() - last_warn_tick > 1000) {
            printf("[TCP] WARN: tcpip_try_callback busy (ERR_WOULDBLOCK), total drops: %u\n", wouldblock_count);
            last_warn_tick = HAL_GetTick();
        }
        osMutexRelease(tcp_client_mutex);
        return false;
    } else if (err != ERR_OK) {
        tcp_async_fail_count++;
        printf("[TCP] ERROR: tcpip_try_callback failed: %d\n", err);
        osMutexRelease(tcp_client_mutex);
        return false;
    }

    osMutexRelease(tcp_client_mutex);
    return true;
}

/**
 * @brief 获取TCP客户端连接状态
 * @return true 已连接，false 未连接
 */
bool tcp_client_is_connected(void)
{
    bool connected;
    if (tcp_client_mutex != NULL) {
        osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
    }
    connected = client_connected;
    if (tcp_client_mutex != NULL) {
        osMutexRelease(tcp_client_mutex);
    }
    return connected;
}

bool tcp_server_is_client_connected(void)
{
    if (tcp_mutex == NULL) return false;
    
    osMutexAcquire(tcp_mutex, portMAX_DELAY);
    bool has_client = false;
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i].active) {
            has_client = true;
            break;
        }
    }
    osMutexRelease(tcp_mutex);
    return has_client;
}

/**
 * @brief 获取上位机数据接收状态
 * @param last_rx_tick 输出上次接收数据的时间戳（毫秒）
 * @param rx_count 输出接收数据计数
 * @note V3.0: 用于LED闪烁指示上位机数据接收状态
 */
void tcp_get_upper_computer_rx_status(uint32_t* last_rx_tick, uint32_t* rx_count)
{
    if (last_rx_tick != NULL) {
        *last_rx_tick = g_upper_computer_last_rx_tick;
    }
    if (rx_count != NULL) {
        *rx_count = g_upper_computer_rx_count;
    }
}

/**
 * @brief 检查上位机是否正在发送数据
 * @param timeout_ms 超时时间（毫秒），超过此时间未收到数据则认为停止发送
 * @return true 上位机正在发送数据，false 上位机已停止发送
 * @note V3.0: 用于LED闪烁指示
 */
bool tcp_is_upper_computer_sending(uint32_t timeout_ms)
{
    uint32_t current_tick = HAL_GetTick();
    if (g_upper_computer_last_rx_tick == 0) {
        return false;
    }
    return (current_tick - g_upper_computer_last_rx_tick) < timeout_ms;
}

/**
 * @brief 获取PONG心跳响应状态
 * @param last_pong_tick 输出上次收到PONG的时间戳（毫秒）
 * @param missed_count 输出连续丢失PONG的次数
 * @note V3.0: 用于双向心跳确认
 */
void tcp_get_pong_status(uint32_t* last_pong_tick, uint32_t* missed_count)
{
    if (last_pong_tick != NULL) {
        *last_pong_tick = g_heartbeat_last_pong_tick;
    }
    if (missed_count != NULL) {
        *missed_count = g_heartbeat_missed_count;
    }
}

/**
 * @brief 重置PONG心跳状态
 * @note V3.0: 在发送心跳前调用
 */
void tcp_reset_pong_state(void)
{
    g_heartbeat_pong_received = false;
}

/**
 * @brief 检查PONG心跳是否超时
 * @param timeout_ms 超时时间（毫秒）
 * @return true 心跳正常，false 心跳超时
 * @note V4.0: 优化心跳检测，使用配置参数，增加启动保护期
 */
bool tcp_is_heartbeat_healthy(uint32_t timeout_ms)
{
    uint32_t current_tick = HAL_GetTick();

    static uint32_t last_check_tick = 0;
    static uint32_t first_heartbeat_tick = 0;
    static bool startup_grace_period = true;

    if (first_heartbeat_tick == 0) {
        first_heartbeat_tick = current_tick;
        last_check_tick = current_tick;
    }

    if (startup_grace_period) {
        if ((current_tick - first_heartbeat_tick) < TCP_STARTUP_GRACE_PERIOD_MS) {
            return true;
        }
        startup_grace_period = false;
    }

    if ((current_tick - last_check_tick) < TCP_HEARTBEAT_CHECK_MIN_MS) {
        return (g_heartbeat_missed_count < TCP_HEARTBEAT_MAX_MISSED);
    }
    last_check_tick = current_tick;

    if (g_heartbeat_last_pong_tick == 0) {
        g_heartbeat_missed_count++;
        return false;
    }

    if ((current_tick - g_heartbeat_last_pong_tick) > timeout_ms) {
        g_heartbeat_missed_count++;
        return false;
    }

    if (g_heartbeat_pong_received) {
        g_heartbeat_missed_count = 0;
    }

    return true;
}

/**
 * @brief 断开TCP客户端连接
 * @note V4.0: 优化断开处理，保留心跳状态以便重连后快速恢复
 */
void tcp_client_disconnect(void)
{
    printf("[TCP Client] Disconnecting...\r\n");

    /* V3.5: 互斥锁保护client_pcb读写，防止与tcpip线程回调竞态 */
    if (tcp_client_mutex != NULL) {
        osMutexAcquire(tcp_client_mutex, portMAX_DELAY);
    }
    if (client_pcb != NULL) {
        tcp_abort(client_pcb);
        client_pcb = NULL;
    }
    if (tcp_client_mutex != NULL) {
        osMutexRelease(tcp_client_mutex);
    }

    client_connected = false;
    
    /* V4.0: 不重置心跳计数，保留状态以便调试 */
    /* g_heartbeat_pong_received = false; */
    /* g_heartbeat_last_pong_tick = 0; */
    /* g_heartbeat_missed_count = 0; */
    
    printf("[TCP Client] Disconnected, heartbeat_state_preserved\r\n");
}
