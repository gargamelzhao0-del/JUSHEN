# 四足机器狗上位机控制框架

## 项目简介

这是一个完整的四足机器狗上位机控制框架，运行在Jetson Orin NX上，与STM32下位机通过TCP通信。

### 主要功能

- **TCP通信**: 与STM32下位机通信，发送电机位置命令，接收传感器数据
- **状态估计**: 融合IMU、足端力、关节反馈数据，估计机器人状态
- **RL策略推理**: 支持PyTorch、ONNX模型推理，输出12关节位置
- **安全管理**: 关节限位、温度、电流、电压、倾倒检测等安全保护
- **50Hz控制循环**: 实时控制循环，协调各模块运行

## 项目结构

```
quadruped_control/
├── __init__.py              # 包初始化
├── config.py                # 配置文件
├── data_types.py            # 数据类型定义
├── tcp_client.py            # TCP通信模块
├── state_estimator.py       # 状态估计模块
├── observation_processor.py # 观测处理模块
├── policy_runner.py         # RL策略推理模块
├── safety_manager.py        # 安全管理模块
├── main_controller.py       # 主控制器
└── run.py                   # 启动脚本
```

## 快速开始

### 1. 安装依赖

```bash
pip install numpy
pip install torch  # 如果使用PyTorch模型
pip install onnxruntime  # 如果使用ONNX模型
```

### 2. 测试模式（无RL模型）

```bash
cd /home/jetson/JUSHEN/quadruped_control
python run.py --mock
```

### 3. 使用RL模型

```bash
python run.py --model /path/to/your/policy.pt
```

### 4. 使用ONNX模型

```bash
python run.py --model /path/to/your/policy.onnx --type onnx
```

## 配置说明

### 网络配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| jetson_ip | 0.0.0.0 | Jetson监听地址 |
| tcp_port | 9999 | TCP端口 |
| stm32_ip | 192.168.1.101 | STM32 IP地址 |

### 控制配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| control_frequency | 50.0 | 控制频率 (Hz) |
| smoothing_enabled | True | 动作平滑 |
| smoothing_factor | 0.3 | 平滑因子 |

### 安全配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| position_limits | (30, 330) | 关节位置限位 (度) |
| temperature_limit | 65.0 | 温度上限 (°C) |
| current_limit | 8.0 | 电流上限 (A) |
| voltage_min | 20.0 | 电压下限 (V) |
| tilt_angle_max | 45.0 | 倾斜角度上限 (度) |

## 通信协议

### 帧格式

```
┌─────────┬─────────┬───────────┬───────────────┬───────────┬─────────┐
│ 帧头(2B) │ 类型(1B) │ 长度(2B)  │   负载数据    │ CRC16(2B) │ 帧尾(2B) │
│ 0xAA 55 │ cmd_type │ len(小端) │   payload     │  校验和   │ 0x0D 0A │
└─────────┴─────────┴───────────┴───────────────┴───────────┴─────────┘
```

### 主要命令

| 命令码 | 名称 | 说明 |
|--------|------|------|
| 0x12 | 位置控制 | 发送12个关节位置 |
| 0x0F | 紧急停止 | 立即停止所有电机 |
| 0x10 | 使能电机 | 使能所有电机 |
| 0x11 | 禁用电机 | 禁用所有电机 |

### 主要响应

| 响应码 | 名称 | 说明 |
|--------|------|------|
| 0x90 | 电机状态 | 位置/速度/电流/温度/错误 |
| 0xA0 | 足端力 | 4个足端力 + 接触掩码 |
| 0xA1 | 电源状态 | 电压/电流/温度/容量 |

## 电机索引

```
索引  腿部    关节
0     前左FL  髋关节
1     前左FL  大腿关节
2     前左FL  小腿关节
3     前右FR  髋关节
4     前右FR  大腿关节
5     前右FR  小腿关节
6     后左RL  髋关节
7     后左RL  大腿关节
8     后左RL  小腿关节
9     后右RR  髋关节
10    后右RR  大腿关节
11    后右RR  小腿关节
```

## 使用示例

### Python API

```python
from quadruped_control import MainController, Config

config = Config()
config.network.tcp_port = 9999
config.control.control_frequency = 50.0

controller = MainController(config)

controller.load_policy("/path/to/policy.pt", "pytorch")

controller.start(wait_for_connection=True, timeout=30.0)

controller.set_command(vx=0.5, vy=0.0, vyaw=0.0)

controller.stop()
```

### 自定义回调

```python
def on_state_change(state):
    print(f"控制器状态: {state}")

def on_cycle_complete(robot_state):
    print(f"电机位置: {robot_state.motor_status.positions}")

controller.set_state_change_callback(on_state_change)
controller.set_cycle_complete_callback(on_cycle_complete)
```

## 注意事项

1. **启动顺序**: 先启动上位机程序，等待STM32连接
2. **网络配置**: 确保Jetson和STM32在同一网段
3. **安全检查**: 系统会自动检查关节限位、温度、电压等
4. **紧急停止**: 发生危险时系统会自动触发紧急停止

## 版本历史

- v1.0.0 (2026-03-24): 初始版本
