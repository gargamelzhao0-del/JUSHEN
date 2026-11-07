# 四足机器人上下位机TCP通信协议文档

**版本**: V2.0  
**日期**: 2025-03-24  
**适用项目**: JUSHEN2 四足机器人控制系统

---

## 目录

1. [网络架构](#1-网络架构)
2. [通信协议](#2-通信协议)
3. [数据包格式](#3-数据包格式)
4. [心跳机制](#4-心跳机制)
5. [错误处理机制](#5-错误处理机制)
6. [超时恢复机制](#6-超时恢复机制)
7. [上位机实现指南](#7-上位机实现指南)
8. [常见问题排查](#8-常见问题排查)

---

## 1. 网络架构

### 1.1 网络拓扑

```
┌─────────────────────────────────────────────────────────────────────┐
│                         以太网直连                                   │
│                                                                      │
│  ┌──────────────────┐                    ┌──────────────────┐       │
│  │     上位机        │                    │    下位机         │       │
│  │   (Jetson/PC)    │◄───── TCP ────────►│   (STM32H743)    │       │
│  │                  │                    │                  │       │
│  │  TCP Server      │    主动连接        │  TCP Client      │       │
│  │  192.168.1.100   │◄──────────────────│  192.168.1.101   │       │
│  │  Port: 9999      │                    │                  │       │
│  └──────────────────┘                    └──────────────────┘       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 角色定义

| 角色 | 设备 | IP地址 | 端口 | 说明 |
|------|------|--------|------|------|
| **TCP Server** | 上位机 (PC/Jetson) | 192.168.1.100 (示例) | 9999 | 被动监听，等待连接 |
| **TCP Client** | 下位机 (STM32H7) | 192.168.1.101 | 随机 | 主动连接上位机 |

### 1.3 网络配置参数

```c
// 位于 system_config.h

// TCP连接配置
#define TCP_SERVER_PORT          9999    // TCP服务器端口
#define TCP_MAX_CLIENTS          2       // 最大客户端数量
#define TCP_RX_BUFFER_SIZE       1024    // 接收缓冲区大小
#define TCP_TX_BUFFER_SIZE       1024    // 发送缓冲区大小

// TCP超时配置
#define TCP_CONNECT_TIMEOUT_MS   5000    // 连接超时
#define TCP_RX_TIMEOUT_MS        1000    // 接收超时警告阈值
#define TCP_HOLD_TIMEOUT_MS      2000    // 超时保持阶段阈值
#define TCP_EMERGENCY_TIMEOUT_MS 3000    // 超时紧急停止阈值

// TCP心跳配置 - V3.3: 统一心跳间隔为1000ms
#define TCP_HEARTBEAT_INTERVAL_MS   1000  // 心跳发送间隔 (1秒)
#define TCP_HEARTBEAT_TIMEOUT_MS    3000  // 心跳超时时间 (3个心跳周期)
#define TCP_HEARTBEAT_MAX_MISSED    3     // 最大丢失心跳数
```

---

## 2. 通信协议

### 2.1 连接建立流程

```
┌─────────┐                              ┌─────────┐
│ 上位机   │                              │ 下位机   │
│(Server) │                              │(Client) │
└────┬────┘                              └────┬────┘
     │                                        │
     │  1. 创建Socket, 监听端口9999           │
     │◄───────────────────────────────────────│
     │                                        │
     │  2. 等待连接...                         │
     │                                        │
     │         3. 主动发起TCP连接请求 (SYN)    │
     │◄───────────────────────────────────────│
     │                                        │
     │  4. 接受连接 (SYN-ACK)                 │
     │───────────────────────────────────────►│
     │                                        │
     │         5. 连接确认 (ACK)              │
     │◄───────────────────────────────────────│
     │                                        │
     │  ★ 连接建立成功 ★                      │
     │                                        │
     │         6. 开始发送心跳/数据            │
     │◄───────────────────────────────────────│
     │                                        │
```

### 2.2 数据流向

```
┌─────────────────────────────────────────────────────────────┐
│                      数据流向图                              │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  上位机 ───────────────────────────────────► 下位机          │
│         发送目标角度 (48字节)                  接收并执行      │
│                                                              │
│  上位机 ◄─────────────────────────────────── 下位机          │
│         接收反馈数据                          发送状态反馈      │
│         - 足端力数据 (32字节)                                 │
│         - 电机状态 (136字节)                                  │
│         - 心跳消息 (文本)                                     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 通信频率

| 数据类型 | 发送方向 | 频率 | 说明 |
|----------|----------|------|------|
| 目标角度 | 上位机 → 下位机 | 50Hz (20ms) | 12个电机目标位置 |
| 足端力数据 | 下位机 → 上位机 | 50Hz (20ms) | 4个足端力传感器 |
| 电机状态 | 下位机 → 上位机 | 50Hz (20ms) | 12个电机位置/速度/扭矩 |
| 心跳消息 | 下位机 → 上位机 | 1Hz (1000ms) | 连接状态检测 |

---

## 3. 数据包格式

### 3.1 目标角度数据包 (上位机 → 下位机)

**用途**: 上位机发送12个电机的目标角度

```
┌─────────────────────────────────────────────────────────────────┐
│  字节偏移  │  字段名        │  类型   │  说明                    │
├─────────────────────────────────────────────────────────────────┤
│  0-3       │  motor[0]      │  float  │  电机1目标角度 (度)       │
│  4-7       │  motor[1]      │  float  │  电机2目标角度 (度)       │
│  8-11      │  motor[2]      │  float  │  电机3目标角度 (度)       │
│  ...       │  ...           │  ...    │  ...                     │
│  44-47     │  motor[11]     │  float  │  电机12目标角度 (度)      │
└─────────────────────────────────────────────────────────────────┘

总长度: 48字节
字节序: 小端序 (Little-Endian)
角度范围: 0.0 ~ 359.9 度
```

**电机编号对应关系**:

```
电机索引    关节类型    腿部位置
────────────────────────────────
  0         HIP        前左腿髋关节
  1         THIGH      前左腿大腿
  2         CALF       前左腿小腿
  3         HIP        前右腿髋关节
  4         THIGH      前右腿大腿
  5         CALF       前右腿小腿
  6         HIP        后左腿髋关节
  7         THIGH      后左腿大腿
  8         CALF       后左腿小腿
  9         HIP        后右腿髋关节
  10        THIGH      后右腿大腿
  11        CALF       后右腿小腿
```

### 3.2 足端力数据包 (下位机 → 上位机)

**用途**: 下位机上报4个足端力传感器数据

```
┌─────────────────────────────────────────────────────────────────┐
│  字节偏移  │  字段名        │  类型   │  说明                    │
├─────────────────────────────────────────────────────────────────┤
│  0         │  header[0]     │  uint8  │  帧头 0xAA               │
│  1         │  header[1]     │  uint8  │  帧头 0x55               │
│  2         │  type          │  uint8  │  数据类型 0x02           │
│  3-6       │  front_left    │  float  │  前左足端力 (N)           │
│  7-10      │  front_right   │  float  │  前右足端力 (N)           │
│  11-14     │  rear_left     │  float  │  后左足端力 (N)           │
│  15-18     │  rear_right    │  float  │  后右足端力 (N)           │
│  19-22     │  timestamp     │  uint32 │  时间戳 (tick)           │
│  23        │  checksum      │  uint8  │  校验和 (所有字节累加)    │
└─────────────────────────────────────────────────────────────────┘

总长度: 24字节
```

### 3.3 电机状态数据包 (下位机 → 上位机)

**用途**: 下位机上报12个电机的实时状态

```
┌─────────────────────────────────────────────────────────────────┐
│  字节偏移  │  字段名        │  类型   │  说明                    │
├─────────────────────────────────────────────────────────────────┤
│  0         │  header[0]     │  uint8  │  帧头 0xAA               │
│  1         │  header[1]     │  uint8  │  帧头 0x55               │
│  2         │  type          │  uint8  │  数据类型 0x03           │
│  3-6       │  motor[0].pos  │  float  │  电机1位置 (度)           │
│  7-10      │  motor[0].vel  │  float  │  电机1速度 (RPM)          │
│  11-14     │  motor[0].trq  │  float  │  电机1扭矩 (Nm)           │
│  ...       │  ...           │  ...    │  (每个电机12字节)         │
│ 135-138    │  timestamp     │  uint32 │  时间戳 (tick)           │
│ 139        │  checksum      │  uint8  │  校验和                   │
└─────────────────────────────────────────────────────────────────┘

总长度: 140字节
每个电机数据: 12字节 (位置4 + 速度4 + 扭矩4)
```

### 3.4 心跳消息 (下位机 → 上位机)

**用途**: 连接状态检测

```
格式: 文本字符串
内容: "HEARTBEAT <count>\n"
示例: "HEARTBEAT 123\n"

说明:
- count: 心跳计数器，每次发送递增
- 用于检测连接是否存活
- 上位机可选择响应或仅接收
```

---

## 4. 心跳机制

### 4.1 心跳发送策略

下位机每 **1秒** 发送一次心跳消息：

```c
// 位于 freertos.c - TCPSendTask

// 发送心跳消息: 每1秒一次
if (tick_count % 200 == 0) {  // 1000ms / 5ms = 200
    char heartbeat_msg[64];
    snprintf(heartbeat_msg, sizeof(heartbeat_msg),
             "HEARTBEAT %u\n", heartbeat_count++);
    
    if (tcp_client_send_text(heartbeat_msg)) {
        // 发送成功
    } else {
        // 发送失败，可能连接断开
    }
}
```

### 4.2 心跳计数器溢出保护

```c
// 心跳计数器溢出保护
#define HEARTBEAT_MAX_COUNT 1000000

if (heartbeat_count >= HEARTBEAT_MAX_COUNT) {
    heartbeat_count = 0;
    printf("[INFO] [TCP] Heartbeat counter reset\n");
}
```

### 4.3 上位机心跳处理建议

```python
def handle_heartbeat(message: str):
    """
    处理心跳消息
    
    Args:
        message: 心跳消息，格式 "HEARTBEAT <count>"
    """
    try:
        parts = message.strip().split()
        if len(parts) >= 2 and parts[0] == "HEARTBEAT":
            count = int(parts[1])
            # 更新最后心跳时间
            self.last_heartbeat_time = time.time()
            # 可选：验证计数器连续性
            if hasattr(self, 'last_heartbeat_count'):
                expected = self.last_heartbeat_count + 1
                if count != expected:
                    print(f"[WARN] Heartbeat skip: expected {expected}, got {count}")
            self.last_heartbeat_count = count
    except Exception as e:
        print(f"[ERROR] Failed to parse heartbeat: {e}")
```

---

## 5. 错误处理机制

### 5.1 错误触发条件

下位机会在以下情况下触发错误处理机制：

| 错误类型 | 触发条件 | 错误源 |
|----------|----------|--------|
| **TCP断开** | TCP连接断开 | `ESTOP_SOURCE_TCP` |
| **TCP超时** | 超过1000ms未收到数据 | `ESTOP_SOURCE_TCP_TIMEOUT` |
| **看门狗超时** | 任务卡死超过阈值 | `ESTOP_SOURCE_WATCHDOG` |
| **硬件急停** | GPIO急停按钮按下 | `ESTOP_SOURCE_GPIO` |
| **过流保护** | 电机电流超限 | `ESTOP_SOURCE_OVERCURRENT` |
| **过温保护** | 电机温度超限 | `ESTOP_SOURCE_OVERTEMP` |

### 5.2 TCP超时分级处理

下位机实现了三级超时处理机制：

```
┌─────────────────────────────────────────────────────────────────┐
│                    TCP超时分级处理流程                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  正常运行                                                        │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ 超时 1000ms (TCP_RX_TIMEOUT_MS)         │                    │
│  │ → 进入 DISCONNECTED 状态                 │                    │
│  │ → 设置 TCP_TIMEOUT 标志                  │                    │
│  │ → 输出警告日志                           │                    │
│  └─────────────────────────────────────────┘                    │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ 超时 2000ms (TCP_HOLD_TIMEOUT_MS)       │                    │
│  │ → 设置 TCP_HOLD 标志                     │                    │
│  │ → 电机保持当前位置                        │                    │
│  │ → 停止响应新的目标角度                    │                    │
│  └─────────────────────────────────────────┘                    │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ 超时 3000ms (TCP_EMERGENCY_TIMEOUT_MS)  │                    │
│  │ → 设置 TCP_SAFE_STOP 标志                │                    │
│  │ → 触发紧急停止                           │                    │
│  │ → 所有电机立即停止                        │                    │
│  └─────────────────────────────────────────┘                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.3 紧急停止级别

```c
// 位于 quad_emergency_stop.h

typedef enum {
    ESTOP_LEVEL_SOFT,       // 软停止：平滑减速停止
    ESTOP_LEVEL_HARD,       // 硬停止：立即停止
    ESTOP_LEVEL_POWER_CUT   // 断电：关闭电机使能
} EmergencyStopLevel_t;
```

### 5.4 上位机如何触发紧急停止

上位机可以通过以下方式触发紧急停止：

#### 方式1: 断开TCP连接
```python
# 直接关闭连接，下位机会在超时后自动触发紧急停止
client_socket.close()
```

#### 方式2: 发送特殊命令 (需实现)
```python
# 发送紧急停止命令
EMERGENCY_STOP_CMD = b"\xFF\xFF\xFF\xFF"  # 特殊命令
client_socket.send(EMERGENCY_STOP_CMD)
```

#### 方式3: 停止发送数据
```python
# 停止发送目标角度，下位机会在超时后进入保持状态
# 超过3000ms后触发紧急停止
time.sleep(4)  # 等待超时
```

---

## 6. 超时恢复机制

### 6.1 恢复流程

当TCP连接恢复后，下位机会执行渐进式恢复：

```
┌─────────────────────────────────────────────────────────────────┐
│                    TCP恢复流程                                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  TCP连接恢复                                                     │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ Phase 1: RECONNECTING (重连阶段)         │                    │
│  │ 持续时间: 500ms (TCP_HOLD_DURATION_MS)  │                    │
│  │ 动作: 清除所有TCP标志                     │                    │
│  │       速度限制因子: 0%                    │                    │
│  └─────────────────────────────────────────┘                    │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ Phase 2: SOFT_START (软启动阶段)         │                    │
│  │ 持续时间: 1000ms (TCP_SOFT_START_MS)    │                    │
│  │ 动作: 速度逐渐增加 0% → 100%             │                    │
│  │       使用二次曲线平滑过渡                │                    │
│  └─────────────────────────────────────────┘                    │
│     │                                                            │
│     ▼                                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │ Phase 3: NORMAL (正常运行)               │                    │
│  │ 动作: 恢复正常控制                        │                    │
│  │       速度限制因子: 100%                  │                    │
│  └─────────────────────────────────────────┘                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 软启动算法

```c
// 位于 quad_control_loop.c

case TCP_RECOVERY_PHASE_SOFT_START:
    {
        uint32_t elapsed = current_tick - g_ctx.tcp.recovery_start_tick;
        if (elapsed < TCP_SOFT_START_DURATION_MS) {
            // 使用二次曲线平滑过渡
            speed_limit_factor = (float)elapsed / TCP_SOFT_START_DURATION_MS;
            speed_limit_factor = speed_limit_factor * speed_limit_factor;  // 平方
        } else {
            speed_limit_factor = 1.0f;
            g_ctx.tcp.recovery_phase = TCP_RECOVERY_PHASE_NORMAL;
        }
    }
    break;
```

### 6.3 上位机恢复配合

上位机在连接恢复后应该：

```python
def on_connection_restored():
    """连接恢复后的处理"""
    # 1. 等待下位机完成恢复流程 (约1.5秒)
    time.sleep(1.5)
    
    # 2. 发送当前安全位置作为起始点
    safe_angles = get_safe_standing_angles()
    send_motor_angles(safe_angles)
    
    # 3. 逐渐增加运动幅度
    for i in range(10):
        factor = (i + 1) / 10.0
        scaled_angles = interpolate_angles(last_angles, target_angles, factor)
        send_motor_angles(scaled_angles)
        time.sleep(0.1)
    
    # 4. 恢复正常控制
    print("[INFO] Connection recovery complete")
```

---

## 7. 上位机实现指南

### 7.1 完整Python示例

```python
#!/usr/bin/env python3
"""
四足机器人上位机TCP通信完整示例
"""

import socket
import struct
import time
import threading
from typing import Optional, Callable
from dataclasses import dataclass

# ==================== 配置 ====================
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 9999
MOTOR_COUNT = 12
ANGLE_PACKET_SIZE = 48

# ==================== 数据类 ====================
@dataclass
class MotorAngles:
    angles: list  # 12个float
    
    def validate(self) -> bool:
        for i, angle in enumerate(self.angles):
            if not (0.0 <= angle <= 359.9):
                print(f"[WARN] Motor {i} angle {angle:.2f} out of range")
                return False
        return True
    
    def to_bytes(self) -> bytes:
        return struct.pack('<12f', *self.angles)
    
    @staticmethod
    def from_bytes(data: bytes) -> 'MotorAngles':
        angles = list(struct.unpack('<12f', data[:ANGLE_PACKET_SIZE]))
        return MotorAngles(angles=angles)

@dataclass
class FootForceData:
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float
    timestamp: int

@dataclass
class MotorStatus:
    position: float
    velocity: float
    torque: float

# ==================== TCP服务器 ====================
class QuadrupedTCPServer:
    def __init__(self, host: str = SERVER_HOST, port: int = SERVER_PORT):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False
        self.connected = False
        
        # 回调函数
        self.on_foot_force: Optional[Callable[[FootForceData], None]] = None
        self.on_motor_status: Optional[Callable[[list], None]] = None
        self.on_heartbeat: Optional[Callable[[int], None]] = None
        self.on_connected: Optional[Callable[[], None]] = None
        self.on_disconnected: Optional[Callable[[], None]] = None
        
        # 心跳监控
        self.last_heartbeat_time = 0
        self.heartbeat_timeout = 3.0  # 秒
        
        # 统计
        self.tx_count = 0
        self.rx_count = 0
    
    def start(self):
        """启动服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        self.running = True
        print(f"[SERVER] Listening on {self.host}:{self.port}")
        
        # 启动接受线程
        threading.Thread(target=self._accept_loop, daemon=True).start()
    
    def _accept_loop(self):
        """接受连接循环"""
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                client_sock, client_addr = self.server_socket.accept()
                
                print(f"\n[SERVER] Client connected: {client_addr}")
                
                self.client_socket = client_sock
                self.client_socket.settimeout(5.0)
                self.connected = True
                self.last_heartbeat_time = time.time()
                
                if self.on_connected:
                    self.on_connected()
                
                # 启动接收线程
                threading.Thread(target=self._receive_loop, daemon=True).start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[SERVER] Accept error: {e}")
    
    def _receive_loop(self):
        """接收数据循环"""
        buffer = bytearray()
        
        while self.running and self.connected:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self._handle_disconnect()
                    break
                
                buffer.extend(data)
                
                # 处理数据包
                self._process_buffer(buffer)
                
                # 心跳超时检测
                if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
                    print("[WARN] Heartbeat timeout!")
                    self._handle_disconnect()
                    break
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running and self.connected:
                    print(f"[SERVER] Receive error: {e}")
                    self._handle_disconnect()
                break
    
    def _process_buffer(self, buffer: bytearray):
        """处理接收缓冲区"""
        while len(buffer) > 0:
            # 检查文本心跳消息
            text_end = buffer.find(b'\n')
            if text_end >= 0:
                text = bytes(buffer[:text_end]).decode('utf-8', errors='ignore')
                del buffer[:text_end + 1]
                
                if text.startswith("HEARTBEAT"):
                    self.last_heartbeat_time = time.time()
                    parts = text.split()
                    if len(parts) >= 2:
                        count = int(parts[1])
                        if self.on_heartbeat:
                            self.on_heartbeat(count)
                continue
            
            # 检查二进制数据包
            if len(buffer) >= 3:
                header = buffer[0:2]
                data_type = buffer[2]
                
                if header == b'\xAA\x55':
                    if data_type == 0x02 and len(buffer) >= 24:
                        # 足端力数据包
                        self._parse_foot_force(buffer[:24])
                        del buffer[:24]
                        continue
                    elif data_type == 0x03 and len(buffer) >= 140:
                        # 电机状态数据包
                        self._parse_motor_status(buffer[:140])
                        del buffer[:140]
                        continue
                
                # 未知数据包，丢弃一个字节
                del buffer[0]
            else:
                break
    
    def _parse_foot_force(self, data: bytes):
        """解析足端力数据"""
        try:
            values = struct.unpack('<ffffI', data[3:23])
            foot_force = FootForceData(
                front_left=values[0],
                front_right=values[1],
                rear_left=values[2],
                rear_right=values[3],
                timestamp=values[4]
            )
            self.rx_count += 1
            if self.on_foot_force:
                self.on_foot_force(foot_force)
        except Exception as e:
            print(f"[ERROR] Failed to parse foot force: {e}")
    
    def _parse_motor_status(self, data: bytes):
        """解析电机状态"""
        try:
            statuses = []
            for i in range(12):
                offset = 3 + i * 12
                pos, vel, trq = struct.unpack('<fff', data[offset:offset+12])
                statuses.append(MotorStatus(position=pos, velocity=vel, torque=trq))
            
            self.rx_count += 1
            if self.on_motor_status:
                self.on_motor_status(statuses)
        except Exception as e:
            print(f"[ERROR] Failed to parse motor status: {e}")
    
    def _handle_disconnect(self):
        """处理断开连接"""
        self.connected = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        self.client_socket = None
        print("[SERVER] Client disconnected")
        
        if self.on_disconnected:
            self.on_disconnected()
    
    def send_angles(self, angles: MotorAngles) -> bool:
        """发送目标角度"""
        if not self.connected or not self.client_socket:
            return False
        
        if not angles.validate():
            return False
        
        try:
            data = angles.to_bytes()
            self.client_socket.send(data)
            self.tx_count += 1
            return True
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
            self._handle_disconnect()
            return False
    
    def stop(self):
        """停止服务器"""
        self.running = False
        self.connected = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        print(f"[SERVER] Stopped. TX: {self.tx_count}, RX: {self.rx_count}")


# ==================== 使用示例 ====================
def main():
    server = QuadrupedTCPServer()
    
    # 设置回调
    def on_connected():
        print("[CALLBACK] Connected!")
    
    def on_disconnected():
        print("[CALLBACK] Disconnected!")
    
    def on_heartbeat(count):
        print(f"[CALLBACK] Heartbeat #{count}")
    
    def on_foot_force(data: FootForceData):
        print(f"[CALLBACK] Foot force: FL={data.front_left:.1f}N, "
              f"FR={data.front_right:.1f}N, RL={data.rear_left:.1f}N, "
              f"RR={data.rear_right:.1f}N")
    
    def on_motor_status(statuses: list):
        print(f"[CALLBACK] Motor status: {len(statuses)} motors")
    
    server.on_connected = on_connected
    server.on_disconnected = on_disconnected
    server.on_heartbeat = on_heartbeat
    server.on_foot_force = on_foot_force
    server.on_motor_status = on_motor_status
    
    # 启动服务器
    server.start()
    
    try:
        # 主循环 - 发送目标角度
        while True:
            if server.connected:
                # 示例：发送站立姿态
                angles = MotorAngles(angles=[180.0] * 12)
                if server.send_angles(angles):
                    print(f"[TX] Sent angles")
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
```

### 7.2 上位机注意事项

1. **必须作为TCP服务器运行**，等待下位机主动连接
2. **建议发送频率为50Hz**，与下位机控制循环匹配
3. **角度范围必须为0~359.9度**，超出范围会被拒绝
4. **监控心跳超时**，超过3秒未收到心跳应视为连接断开
5. **实现重连机制**，下位机会自动尝试重连

---

## 8. 常见问题排查

### 8.1 连接问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 无法连接 | IP不在同一网段 | 检查IP配置，确保在同一网段 |
| 无法连接 | 防火墙阻止 | 开放端口9999或关闭防火墙 |
| 连接后立即断开 | 端口被占用 | 更换端口或关闭占用进程 |
| 连接不稳定 | 网线松动 | 检查网线连接 |

### 8.2 数据问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 角度被拒绝 | 超出范围 | 确保角度在0~359.9度内 |
| 数据解析错误 | 字节序错误 | 使用小端序解析 |
| 数据丢失 | 发送频率过高 | 降低发送频率到50Hz |
| 延迟大 | Nagle算法 | 启用TCP_NODELAY |

### 8.3 调试命令

```bash
# 检查网络连接
ping 192.168.1.101

# 检查端口监听
netstat -an | grep 9999

# 抓包分析
tcpdump -i eth0 port 9999 -w capture.pcap
```

---

## 附录A: 配置参数速查表

| 参数 | 值 | 文件位置 |
|------|-----|----------|
| 服务器端口 | 9999 | system_config.h |
| 接收超时警告 | 1000ms | system_config.h |
| 保持阶段超时 | 2000ms | system_config.h |
| 紧急停止超时 | 3000ms | system_config.h |
| 心跳间隔 | 1000ms | freertos.c |
| 控制频率 | 200Hz | system_config.h |
| 数据发送频率 | 50Hz | freertos.c |
| 电机数量 | 12 | system_config.h |

---

**文档结束**
