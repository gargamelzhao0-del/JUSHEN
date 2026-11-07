#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四足机器狗Trot步态模拟器 - 上位机运行
Quadruped Robot Trot Gait Simulator (Jetson)

功能:
- 模拟强化学习输出的50Hz 12电机位置数据
- 生成Trot步态（对角小跑）让机器狗前进
- 通过TCP发送给STM32 (符合工程协议格式)
- 实时显示3D步态动画预览

使用方法:
    python trot_gait_simulator.py
    python trot_gait_simulator.py --mode server  # 仅TCP服务器
    python trot_gait_simulator.py --mode preview  # 仅动画预览
    python trot_gait_simulator.py --mode both  # 两者都运行
"""

import numpy as np
import struct
import socket
import time
import argparse
import threading
import os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

NUM_MOTORS = 12
MOTOR_NEUTRAL = 180.0

PROTO_HEADER_0 = 0xAA
PROTO_HEADER_1 = 0x55
PROTO_FOOTER_0 = 0x0D
PROTO_FOOTER_1 = 0x0A

AUTH_KEY_LENGTH = 16
AUTH_CHALLENGE_LENGTH = 8
AUTH_KEY = b'JUSHEN2025SECRET'

CMD_SYSTEM_RESET        = 0x00
CMD_SYSTEM_HEARTBEAT    = 0x01
CMD_SYSTEM_STATUS_REQ   = 0x02
CMD_SYSTEM_ENTER_IDLE   = 0x03
CMD_SYSTEM_ENTER_READY  = 0x04
CMD_SYSTEM_EMERGENCY    = 0x0F

CMD_MOTOR_ENABLE_ALL    = 0x10
CMD_MOTOR_DISABLE_ALL   = 0x11
CMD_MOTOR_POS_CONTROL   = 0x12
CMD_MOTOR_VEL_CONTROL   = 0x13
CMD_MOTOR_TRQ_CONTROL   = 0x14
CMD_MOTOR_STATUS_REQ    = 0x15
CMD_MOTOR_SET_PARAMS    = 0x16
CMD_MOTOR_SEND_ENABLE   = 0x17

CMD_SENSOR_FORCE_REQ    = 0x30
CMD_SENSOR_POWER_REQ    = 0x31

CMD_CONFIG_SAVE         = 0x40
CMD_CONFIG_LOAD         = 0x41

RESP_SYSTEM_ACK         = 0x80
RESP_SYSTEM_NACK        = 0x81
RESP_SYSTEM_STATUS      = 0x82
RESP_SYSTEM_ERROR       = 0x8F

RESP_MOTOR_STATUS       = 0x90
RESP_MOTOR_FEEDBACK     = 0x91

RESP_SENSOR_FORCE       = 0xA0
RESP_SENSOR_POWER       = 0xA1

AUTH_STATE_DISCONNECTED = 0
AUTH_STATE_CHALLENGE_SENT = 1
AUTH_STATE_AUTHENTICATED = 2
AUTH_STATE_FAILED = 3
AUTH_STATE_BLOCKED = 4

CRC16_MODBUS_TABLE = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
]

def crc16_modbus(data: bytes) -> int:
    """计算CRC-16/MODBUS校验"""
    crc = 0xFFFF
    for byte in data:
        index = (crc ^ byte) & 0xFF
        crc = (crc >> 8) ^ CRC16_MODBUS_TABLE[index]
    return crc

class ProtocolCodec:
    """协议编解码器 - 符合工程protocol_codec.c"""
    
    @staticmethod
    def encode_frame(cmd_type: int, payload: bytes = b'') -> bytes:
        """编码完整帧"""
        frame = bytearray()
        frame.append(PROTO_HEADER_0)
        frame.append(PROTO_HEADER_1)
        frame.append(cmd_type)
        frame.append(len(payload) & 0xFF)
        frame.append((len(payload) >> 8) & 0xFF)
        frame.extend(payload)
        crc = crc16_modbus(bytes(frame[2:]))
        frame.append((crc >> 8) & 0xFF)
        frame.append(crc & 0xFF)
        frame.append(PROTO_FOOTER_0)
        frame.append(PROTO_FOOTER_1)
        return bytes(frame)
    
    @staticmethod
    def encode_motor_pos_cmd(positions: list, max_velocity: float = 5.0, 
                             timestamp_ms: int = 0) -> bytes:
        """编码电机位置控制命令 (CMD_MOTOR_POS_CONTROL)"""
        payload = bytearray()
        for pos in positions:
            pos_rad = (pos - MOTOR_NEUTRAL) * np.pi / 180.0
            payload.extend(struct.pack('<f', pos_rad))
        payload.extend(struct.pack('<f', max_velocity))
        payload.extend(struct.pack('<I', timestamp_ms))
        return ProtocolCodec.encode_frame(CMD_MOTOR_POS_CONTROL, bytes(payload))
    
    @staticmethod
    def encode_heartbeat() -> bytes:
        """编码心跳包"""
        return ProtocolCodec.encode_frame(CMD_SYSTEM_HEARTBEAT)
    
    @staticmethod
    def encode_motor_enable_all() -> bytes:
        """编码启用所有电机命令"""
        return ProtocolCodec.encode_frame(CMD_MOTOR_ENABLE_ALL)
    
    @staticmethod
    def encode_motor_disable_all() -> bytes:
        """编码禁用所有电机命令"""
        return ProtocolCodec.encode_frame(CMD_MOTOR_DISABLE_ALL)
    
    @staticmethod
    def encode_system_status_req() -> bytes:
        """编码查询系统状态命令"""
        return ProtocolCodec.encode_frame(CMD_SYSTEM_STATUS_REQ)
    
    @staticmethod
    def encode_motor_status_req() -> bytes:
        """编码查询电机状态命令"""
        return ProtocolCodec.encode_frame(CMD_MOTOR_STATUS_REQ)
    
    @staticmethod
    def encode_emergency_stop() -> bytes:
        """编码紧急停止命令"""
        return ProtocolCodec.encode_frame(CMD_SYSTEM_EMERGENCY)
    
    @staticmethod
    def decode_frame(data: bytes) -> dict:
        """解码接收到的帧"""
        if len(data) < 9:
            return None
        
        if data[0] != PROTO_HEADER_0 or data[1] != PROTO_HEADER_1:
            return None
        
        cmd_type = data[2]
        payload_len = data[3] | (data[4] << 8)
        
        if len(data) < 5 + payload_len + 4:
            return None
        
        payload = data[5:5 + payload_len]
        
        crc_offset = 5 + payload_len
        received_crc = (data[crc_offset] << 8) | data[crc_offset + 1]
        calculated_crc = crc16_modbus(data[2:crc_offset])
        
        if received_crc != calculated_crc:
            return {'cmd_type': cmd_type, 'payload': payload, 'crc_error': True}
        
        return {'cmd_type': cmd_type, 'payload': payload, 'crc_error': False}
    
    @staticmethod
    def decode_motor_status(payload: bytes) -> dict:
        """解码电机状态响应"""
        if len(payload) < 108:
            return None
        
        positions = struct.unpack('<12f', payload[0:48])
        velocities = struct.unpack('<12f', payload[48:96])
        currents = struct.unpack('<12f', payload[96:144])
        temperatures = struct.unpack('<12h', payload[144:168])
        error_flags = struct.unpack('<12H', payload[168:192])
        
        return {
            'positions': [p * 180.0 / np.pi + MOTOR_NEUTRAL for p in positions],
            'velocities': velocities,
            'currents': currents,
            'temperatures': temperatures,
            'error_flags': error_flags
        }
    
    @staticmethod
    def decode_system_status(payload: bytes) -> dict:
        """解码系统状态响应"""
        if len(payload) < 14:
            return None
        
        return {
            'robot_state': payload[0],
            'error_flags': payload[1] | (payload[2] << 8),
            'uptime_ms': struct.unpack('<I', payload[3:7])[0],
            'command_count': struct.unpack('<I', payload[7:11])[0],
            'free_heap_kb': struct.unpack('<H', payload[11:13])[0],
            'motor_online': payload[13]
        }
    
    @staticmethod
    def decode_foot_force(payload: bytes) -> dict:
        """解码足端力传感器响应"""
        if len(payload) < 20:
            return None
        
        return {
            'force_fl': struct.unpack('<f', payload[0:4])[0],
            'force_fr': struct.unpack('<f', payload[4:8])[0],
            'force_rl': struct.unpack('<f', payload[8:12])[0],
            'force_rr': struct.unpack('<f', payload[12:16])[0],
            'timestamp_ms': struct.unpack('<I', payload[16:20])[0]
        }

class GaitGenerator:
    STATE_INIT = 0       # 初始状态
    STATE_WARMUP = 1     # 预热阶段：电机移动到工作位置
    STATE_GAIT = 2       # 步态阶段

    def __init__(self):
        self.amplitude = 25
        self.frequency = 1.0
        self.forward_speed = 1.0
        self.body_sway = 5
        self.smooth_mode = True

        self.phase_fl = 0
        self.phase_fr = np.pi
        self.phase_rl = np.pi
        self.phase_rr = 0

        self.prev_angles = None
        self.velocity_history = np.zeros((12, 5))

        self.prev_forward_offset = None

        self.state = self.STATE_INIT
        self.start_time = None
        self.warmup_duration = 2.0  # 预热时间2秒
        self.work_positions = self._calc_work_positions()

    def _calc_work_positions(self):
        work_pos = {}
        for leg_id in range(4):
            offset = self.forward_speed * 15 * (1 if leg_id in [0, 3] else -1)
            work_pos[leg_id] = {
                'hip': MOTOR_NEUTRAL + offset,
                'thigh': MOTOR_NEUTRAL,
                'knee': MOTOR_NEUTRAL
            }
        return work_pos

    def set_state(self, new_state):
        self.state = new_state

    def is_warmup_done(self):
        return self.state == self.STATE_GAIT

    def smooth_forward_offset(self, leg_id, time_sec):
        """
        平滑前进偏移 - 避免启动时的跳变
        使用正弦波使 offset 在正负之间平滑变化
        
        关键改进:
        - 替代固定 offset 值
        - 与步态相位同步
        - 确保连续性
        """
        period = 1.0 / self.frequency
        phase = (time_sec / period) % 1.0

        # FL 和 RR 使用相同相位，FR 和 RL 使用相反相位
        if leg_id in [0, 3]:  # FL, RR
            offset_phase = 0
        else:  # FR, RL
            offset_phase = 0.5

        current_phase = (phase + offset_phase) % 1.0

        # 使用平滑的正弦变化，替代固定 offset
        # 这样 offset 会在 +max 和 -max 之间连续变化
        offset_value = self.forward_speed * 15 * np.sin(2 * np.pi * current_phase)

        return offset_value

    def smooth_step(self, t, period=1.0, smooth_factor=0.3):
        """
        平滑步态函数：模拟RL学习的加减速曲线
        使用S型曲线(sigmoid)代替纯正弦波

        @param t: 当前时间(秒)
        @param period: 步态周期(秒)
        @param smooth_factor: 平滑因子(0-1)，越大越平滑
        """
        phase = (t / period) % 1.0

        # 将相位转换为平滑的S型曲线
        # 使用余弦函数的一半周期: 0 -> 1 -> 0
        cos_val = np.cos(2 * np.pi * phase)

        # 平滑处理：在峰值附近减速
        if smooth_factor > 0:
            # S型曲线：使用erf函数近似
            x = (phase - 0.5) * 4  # 转换到 -2 到 2
            smooth = (1 + np.tanh(x * smooth_factor * 2)) / 2

            # 结合余弦和S型曲线
            result = cos_val * (0.5 + 0.5 * smooth)
        else:
            result = cos_val

        return result

    def generate_smooth_trot(self, time_sec):
        """
        生成平滑的Trot步态 - 模拟真实RL输出
        特点：在方向切换时自动减速，更真实的生物步态
        """
        period = 1.0 / self.frequency
        angles = []

        for leg_id in range(4):
            if leg_id == 0:
                phase = self.phase_fl
            elif leg_id == 1:
                phase = self.phase_fr
            elif leg_id == 2:
                phase = self.phase_rl
            else:
                phase = self.phase_rr

            # 计算平滑的基础摆动
            # 使用改进的S型曲线
            effective_phase = (time_sec * self.frequency + phase / (2 * np.pi)) % 1.0

            # 摆动相(stance phase)的S型曲线
            swing = self._smooth_swing_phase(effective_phase)

            # 各关节角度计算
            hip_amp = self.amplitude * 0.4
            thigh_amp = self.amplitude * 1.2
            knee_amp = self.amplitude * 0.8

            # 前进偏移 - 固定基准，模拟身体倾斜
            forward_offset = self.forward_speed * 20 * (1 if leg_id in [0, 3] else -1)

            # 髋关节：主要前后摆动
            hip_angle = MOTOR_NEUTRAL + forward_offset + hip_amp * swing

            # 大腿关节：摆动 + 抬腿
            lift = max(0, swing) * self.amplitude * 0.3
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * swing + lift

            # 膝关节：摆动 + 弯曲
            knee_bend = max(0, swing) * knee_amp * 0.5
            knee_angle = MOTOR_NEUTRAL + knee_amp * swing + knee_bend

            angles.extend([hip_angle, thigh_angle, knee_angle])

        return angles

    def _smooth_swing_phase(self, phase):
        """
        计算平滑的摆动相 - 使用改进的速度曲线
        在峰值附近自动减速，模拟真实RL学习的步态
        """
        # 将phase (0-1) 转换为 -1 到 1
        x = phase * 2 - 1

        # 使用余弦函数生成基本波形
        base = np.cos(np.pi * x)

        # 添加平滑因子的S型曲线修正
        # 在峰值处(x接近0或2附近)减速
        smooth_factor = 0.4

        # 修正因子：在峰值附近降低速度
        correction = 1 - smooth_factor * (1 - np.abs(base))
        result = base * correction

        return result

    def generate_rl_like_trot(self, time_sec):
        """
        生成更像RL学习的步态 - 带有加减速曲线和随机性
        支持预热阶段：电机慢慢移动到工作位置
        """
        if self.state == self.STATE_WARMUP:
            return self._generate_warmup(time_sec)
        elif self.state == self.STATE_GAIT:
            return self._generate_gait(time_sec)
        else:
            return [MOTOR_NEUTRAL] * 12

    def _generate_warmup(self, time_sec):
        """
        预热阶段：电机从默认位置慢慢移动到工作位置
        使用改进的 S 型曲线实现超平滑过渡
        避免启动时的冲击
        """
        if self.start_time is None:
            self.start_time = time_sec

        elapsed = time_sec - self.start_time
        progress = min(elapsed / self.warmup_duration, 1.0)

        # 使用 5 次多项式 S 型曲线 (比 3 次更平滑)
        # f(t) = 10t³ - 15t⁴ + 6t⁵
        # 特点：起点和终点的速度、加速度都为 0
        if progress < 1.0:
            smooth_progress = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
        else:
            smooth_progress = 1.0

        angles = []
        for leg_id in range(4):
            work = self.work_positions[leg_id]

            # 使用平滑进度渐进式移动到工作位置
            hip = MOTOR_NEUTRAL + (work['hip'] - MOTOR_NEUTRAL) * smooth_progress
            thigh = MOTOR_NEUTRAL + (work['thigh'] - MOTOR_NEUTRAL) * smooth_progress
            knee = MOTOR_NEUTRAL + (work['knee'] - MOTOR_NEUTRAL) * smooth_progress

            angles.extend([hip, thigh, knee])

        if progress >= 1.0:
            self.state = self.STATE_GAIT
            print(f'[GAIT] Warmup done, entering gait mode')

        return angles

    def _generate_gait(self, time_sec):
        """
        步态阶段：生成正常步态
        使用平滑的正弦曲线，消除所有方向跳变
        """
        period = 1.0 / self.frequency
        t = time_sec % period
        phase = t / period

        angles = []

        for leg_id in range(4):
            if leg_id == 0:
                phase_offset = 0
            elif leg_id == 1:
                phase_offset = 0.5  # 对角同步
            elif leg_id == 2:
                phase_offset = 0.5  # 对角同步
            else:
                phase_offset = 0

            # 计算当前相位
            current_phase = (phase + phase_offset) % 1.0

            # 使用完全平滑的正弦曲线 (替代梯形曲线)
            swing = self._smooth_sine_swing(current_phase)

            # 添加微小的随机扰动模拟 RL 的不确定性
            noise = np.random.randn() * 0.3

            hip_amp = self.amplitude * 0.4 + noise
            thigh_amp = self.amplitude * 1.2 + noise * 1.5
            knee_amp = self.amplitude * 0.8 + noise

            # 前进偏移 - 使用平滑变化而非固定值
            forward_offset = self.smooth_forward_offset(leg_id, time_sec)

            hip_angle = MOTOR_NEUTRAL + forward_offset + hip_amp * swing
            lift = max(0, swing) * self.amplitude * 0.25
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * swing + lift
            knee_bend = max(0, swing) * knee_amp * 0.4
            knee_angle = MOTOR_NEUTRAL + knee_amp * swing + knee_bend

            angles.extend([hip_angle, thigh_angle, knee_angle])

        return angles

    def _smooth_sine_swing(self, phase):
        """
        完全平滑的正弦摆动曲线 - 消除所有不连续点
        使用修正的正弦波，确保位置、速度、加速度都连续
        
        传统梯形曲线问题:
        - 在 0.15, 0.5, 0.85 处速度/加速度不连续
        
        改进方案:
        - 使用基波 + 三次谐波合成
        - 在峰值附近自动平滑减速
        """
        # 基础正弦波 (周期为 2 倍 phase)
        # 这样 phase: 0->0.5 为正半周，0.5->1.0 为负半周
        base_sine = np.sin(2 * np.pi * phase)
        
        # 添加三次谐波修正，使波形在峰值附近更平滑
        # 三次谐波使速度曲线在峰值处更圆润
        third_harmonic = 0.1 * np.sin(6 * np.pi * phase)
        
        # 合成最终波形
        swing = base_sine + third_harmonic
        
        # 应用平滑因子 (可选)
        smooth_factor = 0.85
        swing = smooth_factor * swing
        
        return swing

    def _trapezoid_swing(self, phase):
        """
        梯形速度曲线 - 模拟真实RL的加减速
        特点：在摆动开始和结束时减速，中间匀速
        期望：0 -> 1 -> 0 -> -1 -> 0
        """
        if phase < 0.15:
            # 启动阶段：0 -> 1 (平滑加速)
            t = phase / 0.15
            return np.sin(np.pi * t / 2)
        elif phase < 0.5:
            # 正向返回阶段：1 -> 0 (线性)
            t = (phase - 0.15) / 0.35
            return 1 - t
        elif phase < 0.85:
            # 反向阶段：0 -> -1 (线性)
            t = (phase - 0.5) / 0.35
            return -t
        else:
            # 减速停止阶段：-1 -> 0 (平滑减速)
            t = (phase - 0.85) / 0.15
            return -1 + np.sin(np.pi * t / 2)

    def generate_trot_gait(self, time_sec):
        phase_fl = 0
        phase_fr = np.pi
        phase_rl = np.pi
        phase_rr = 0
        
        hip_amp = self.amplitude * 0.3
        thigh_amp = self.amplitude
        knee_amp = self.amplitude * 0.8
        
        angles = []
        
        for leg_id in range(4):
            if leg_id == 0:
                phase = phase_fl
            elif leg_id == 1:
                phase = phase_fr
            elif leg_id == 2:
                phase = phase_rl
            else:
                phase = phase_rr
            
            hip_angle = MOTOR_NEUTRAL + hip_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            knee_angle = MOTOR_NEUTRAL + knee_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            
            angles.extend([hip_angle, thigh_angle, knee_angle])
        
        return angles
    
    def generate_trot_forward(self, time_sec, forward_speed=1.0):
        phase_fl = 0
        phase_fr = np.pi
        phase_rl = np.pi
        phase_rr = 0
        
        hip_amp = self.amplitude * 0.4
        thigh_amp = self.amplitude * 1.2
        knee_amp = self.amplitude * 0.8
        
        step_phase = 2 * np.pi * self.frequency * time_sec
        
        lift_amp = self.amplitude * 0.3
        
        angles = []
        
        for leg_id in range(4):
            if leg_id == 0:
                phase = phase_fl
                forward_offset = forward_speed * 0.2
            elif leg_id == 1:
                phase = phase_fr
                forward_offset = -forward_speed * 0.2
            elif leg_id == 2:
                phase = phase_rl
                forward_offset = -forward_speed * 0.2
            else:
                phase = phase_rr
                forward_offset = forward_speed * 0.2
            
            base_swing = np.sin(step_phase + phase)
            
            hip_angle = MOTOR_NEUTRAL + forward_offset + hip_amp * base_swing
            
            lift = np.where(base_swing > 0, base_swing * lift_amp, 0)
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * base_swing + lift
            
            knee_bend = np.where(base_swing > 0, base_swing * knee_amp * 0.5, 0)
            knee_angle = MOTOR_NEUTRAL + knee_amp * base_swing + knee_bend
            
            angles.extend([hip_angle, thigh_angle, knee_angle])
        
        return angles


class QuadTCPServer:
    """TCP服务器 - 符合工程协议格式"""
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.gait_generator = GaitGenerator()
        self.send_count = 0
        self.recv_count = 0
        self.lock = threading.Lock()
        self.forward_speed = 1.0
        self.amplitude = 25
        self.frequency = 1.0
        self.tcp_receive_enabled = True
        self.motor_enabled = False
        self.last_heartbeat_time = 0
        self.heartbeat_interval = 1.0
        self.recv_buffer = bytearray()
    
    def set_params(self, amplitude, frequency, forward_speed):
        self.gait_generator.amplitude = amplitude
        self.gait_generator.frequency = frequency
        self.forward_speed = forward_speed
    
    def parse_response(self, data: bytes) -> list:
        """解析下位机响应数据 - 返回解析后的帧列表"""
        self.recv_buffer.extend(data)
        frames = []
        
        while len(self.recv_buffer) >= 9:
            header_pos = -1
            for i in range(len(self.recv_buffer) - 1):
                if self.recv_buffer[i] == PROTO_HEADER_0 and self.recv_buffer[i + 1] == PROTO_HEADER_1:
                    header_pos = i
                    break
            
            if header_pos == -1:
                self.recv_buffer.clear()
                break
            
            if header_pos > 0:
                del self.recv_buffer[:header_pos]
            
            if len(self.recv_buffer) < 9:
                break
            
            payload_len = self.recv_buffer[3] | (self.recv_buffer[4] << 8)
            frame_len = 5 + payload_len + 4
            
            if len(self.recv_buffer) < frame_len:
                break
            
            frame_data = bytes(self.recv_buffer[:frame_len])
            del self.recv_buffer[:frame_len]
            
            frame = ProtocolCodec.decode_frame(frame_data)
            if frame:
                frames.append(frame)
        
        return frames
    
    def handle_response(self, frame: dict):
        """处理下位机响应"""
        cmd_type = frame['cmd_type']
        payload = frame.get('payload', b'')
        
        if cmd_type == RESP_SYSTEM_ACK:
            print('[TCP] 收到ACK确认')
        elif cmd_type == RESP_SYSTEM_NACK:
            print('[TCP] 收到NACK拒绝')
        elif cmd_type == RESP_SYSTEM_STATUS:
            status = ProtocolCodec.decode_system_status(payload)
            if status:
                print(f'[TCP] 系统状态: state={status["robot_state"]}, '
                      f'uptime={status["uptime_ms"]/1000:.1f}s, '
                      f'motors={status["motor_online"]}')
        elif cmd_type == RESP_MOTOR_STATUS:
            motor_status = ProtocolCodec.decode_motor_status(payload)
            if motor_status:
                print(f'[TCP] 电机状态: pos=[{motor_status["positions"][0]:.1f}, '
                      f'{motor_status["positions"][1]:.1f}, {motor_status["positions"][2]:.1f}...]')
        elif cmd_type == RESP_SENSOR_FORCE:
            force = ProtocolCodec.decode_foot_force(payload)
            if force:
                print(f'[TCP] 足端力: FL={force["force_fl"]:.1f}N FR={force["force_fr"]:.1f}N '
                      f'RL={force["force_rl"]:.1f}N RR={force["force_rr"]:.1f}N')
        else:
            print(f'[TCP] 收到未知响应: cmd=0x{cmd_type:02X}')
    
    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        
        print(f'[TCP] 服务器监听: {self.host}:{self.port}')
        print(f'[TCP] 等待STM32客户端连接...')
        print(f'[TCP] 数据格式: 48字节float数组 (12个角度值, 单位:度)')
        
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    print(f'[TCP] STM32客户端连接: {addr}')
                    self.handle_client()
                except socket.timeout:
                    continue
            except Exception as e:
                if self.running:
                    print(f'[TCP] 错误: {e}')
                break
    
    def handle_client(self):
        if not self.client_socket:
            return

        self.gait_generator.state = self.gait_generator.STATE_WARMUP
        self.gait_generator.start_time = None
        self.motor_enabled = False
        self.recv_buffer.clear()
        print(f'[TCP] Starting warmup phase ({self.gait_generator.warmup_duration}s)...')

        start_time = time.time()
        self.client_socket.settimeout(0.5)
        
        try:
            while self.running:
                if self.tcp_receive_enabled:
                    try:
                        data = self.client_socket.recv(512)
                        if not data:
                            print('[TCP] STM32客户端断开连接')
                            break
                        
                        self.recv_count += 1
                        text = data.decode('utf-8', errors='ignore')
                        print(f'[TCP] 收到STM32数据: {text[:30] if len(text) > 30 else text}')
                            
                    except socket.timeout:
                        pass
                    except Exception as e:
                        print(f'[TCP] 接收错误: {e}')
                        break

                elapsed = time.time() - start_time
                
                if elapsed - self.last_heartbeat_time > self.heartbeat_interval:
                    if elapsed - self.last_heartbeat_time > self.heartbeat_interval:
                        self.last_heartbeat_time = elapsed
                
                if not self.motor_enabled and self.gait_generator.state == self.gait_generator.STATE_GAIT:
                    self.motor_enabled = True
                    print('[TCP] 进入步态模式，开始发送位置数据')

                angles = self.gait_generator.generate_rl_like_trot(elapsed)
                data = struct.pack('12f', *angles)
                
                try:
                    self.client_socket.sendall(data)
                    self.send_count += 1
                    
                    if self.send_count % 50 == 0:
                        print(f'[TCP] 已发送 {self.send_count} 帧, 收到 {self.recv_count} 帧')
                    
                    time.sleep(0.02)
                    
                except Exception as e:
                    print(f'[TCP] 发送失败: {e}')
                    break
                    
        except Exception as e:
            print(f'[TCP] 客户端错误: {e}')
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            print('[TCP] STM32客户端断开')
    
    def stop(self):
        self.running = False
        if self.client_socket:
            try:
                estop_cmd = ProtocolCodec.encode_emergency_stop()
                self.client_socket.sendall(estop_cmd)
            except:
                pass
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print('[TCP] 服务器已停止')


class GaitPreview:
    def __init__(self, gait_generator, preview_freq=30):
        self.gait_generator = gait_generator
        self.time = 0
        self.running = True
        self.preview_freq = preview_freq
        self.preview_time_step = 1.0 / preview_freq

        self.gait_generator.state = self.gait_generator.STATE_WARMUP
        self.gait_generator.start_time = None
        print(f'[Animation] Warmup phase ({self.gait_generator.warmup_duration}s) - motors moving to work position')

        self.fig = plt.figure(figsize=(16, 8))
        self.fig.suptitle('Quadruped Robot - Trot Gait Preview (50Hz)', 
                          fontsize=16, fontweight='bold')
        
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_joints = self.fig.add_subplot(122)
        
        self.setup_3d_view()
        self.setup_joint_view()
        
        self.leg_lines = {}
        self.leg_scatters = {}
        self.body_patch = None
        
        plt.tight_layout()
        
        plt.ion()
        plt.show()
        plt.pause(0.1)
    
    def setup_3d_view(self):
        self.ax_3d.set_xlim(-250, 250)
        self.ax_3d.set_ylim(-250, 250)
        self.ax_3d.set_zlim(-50, 500)
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D View')
        self.ax_3d.view_init(elev=25, azim=45)
    
    def setup_joint_view(self):
        self.ax_joints.set_xlim(0, 10)
        self.ax_joints.set_ylim(0, 12)
        self.ax_joints.axis('off')
        self.ax_joints.set_title('Joint Angles', fontsize=12)
        
        self.joint_labels = []
        leg_names = ['FL', 'FR', 'RL', 'RR']
        joint_names = ['Hip', 'Thigh', 'Knee']
        
        for i in range(12):
            leg = i // 3
            joint = i % 3
            y_pos = 11 - i * 0.9
            label = self.ax_joints.text(0.5, y_pos, 
                                        f'{leg_names[leg]}_{joint_names[joint]}: 180.0°',
                                        fontsize=9, fontfamily='monospace')
            self.joint_labels.append(label)
        
        self.info_label = self.ax_joints.text(5, 0.5, 
                                             'Time: 0.00s | Freq: 1.0Hz',
                                             fontsize=10, ha='center')
        
        self.ax_joints.text(5, -0.5,
                           'Trot Gait: FL+RR同步, FR+RL同步\n红色箭头=前进方向',
                           fontsize=8, ha='center', style='italic')
    
    def draw_body_simple(self):
        body_width = 180
        body_length = 220
        body_height = 30
        body_z = 200

        x = [-body_length/2, body_length/2]
        y = [-body_width/2, body_width/2]
        z = [body_z - body_height/2, body_z + body_height/2]

        xx, yy = np.meshgrid(x, y)

        self.ax_3d.plot_wireframe(xx, yy, z[0] * np.ones_like(xx), color='#3498DB', alpha=0.5)
        self.ax_3d.plot_wireframe(xx, yy, z[1] * np.ones_like(xx), color='#3498DB', alpha=0.5)

        for i in [0, 1]:
            for j in [0, 1]:
                self.ax_3d.plot([x[i], x[i]], [y[j], y[j]], z, color='#3498DB', alpha=0.5)

        self.draw_coordinate_axes()

    def draw_coordinate_axes(self):
        origin = (0, 0, 0)
        axis_length = 150

        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                          axis_length, 0, 0,
                          color='red', arrow_length_ratio=0.1, linewidth=2, alpha=0.8)

        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                          0, axis_length, 0,
                          color='green', arrow_length_ratio=0.1, linewidth=2, alpha=0.8)

        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                          0, 0, axis_length,
                          color='blue', arrow_length_ratio=0.1, linewidth=2, alpha=0.8)

        self.ax_3d.text(axis_length + 10, 0, 0, 'X (Right)', color='red', fontsize=10, fontweight='bold')
        self.ax_3d.text(0, axis_length + 10, 0, 'Y (Front)', color='green', fontsize=10, fontweight='bold')
        self.ax_3d.text(0, 0, axis_length + 10, 'Z (Up)', color='blue', fontsize=10, fontweight='bold')
    
    def calculate_leg_points(self, angles):
        leg_config = {
            0: {'name': 'FL', 'base_x': -110, 'base_y': 110},
            1: {'name': 'FR', 'base_x': 110, 'base_y': 110},
            2: {'name': 'RL', 'base_x': -110, 'base_y': -110},
            3: {'name': 'RR', 'base_x': 110, 'base_y': -110}
        }
        
        leg_points = {}
        
        motor_width = 10
        segment_thigh = 70
        segment_knee = 70
        
        for leg in range(4):
            hip_id = leg * 3
            thigh_id = leg * 3 + 1
            knee_id = leg * 3 + 2

            hip_angle = angles[hip_id] - MOTOR_NEUTRAL
            thigh_angle = angles[thigh_id] - MOTOR_NEUTRAL
            knee_angle = angles[knee_id] - MOTOR_NEUTRAL

            config = leg_config[leg]

            base_x = config['base_x']
            base_y = config['base_y']
            base_z = 200

            hip_x = base_x
            hip_y = base_y - motor_width if base_y > 0 else base_y + motor_width
            hip_z = base_z

            motor1_x = base_x
            motor1_y = hip_y - motor_width if base_y > 0 else hip_y + motor_width
            motor1_z = base_z

            thigh_end_x = motor1_x + segment_thigh * np.sin(np.radians(thigh_angle))
            thigh_end_y = motor1_y
            thigh_end_z = motor1_z - segment_thigh * np.cos(np.radians(thigh_angle))

            motor2_x = thigh_end_x
            motor2_y = thigh_end_y
            motor2_z = thigh_end_z

            knee_x = motor2_x + segment_knee * np.sin(np.radians(thigh_angle + knee_angle))
            knee_y = motor2_y
            knee_z = motor2_z - segment_knee * np.cos(np.radians(thigh_angle + knee_angle))

            foot_x = knee_x
            foot_y = knee_y
            foot_z = max(knee_z - 10, 5)

            leg_points[leg] = {
                'base': (base_x, base_y, base_z),
                'motor0': (hip_x, hip_y, hip_z),
                'motor1': (motor1_x, motor1_y, motor1_z),
                'motor2': (motor2_x, motor2_y, motor2_z),
                'foot': (foot_x, foot_y, foot_z)
            }

        return leg_points
    
    def draw_legs(self, leg_points):
        colors = ['#E74C3C', '#3498DB', '#2ECC71', '#F39C12']

        for leg in range(4):
            points = [
                leg_points[leg]['base'],
                leg_points[leg]['motor0'],
                leg_points[leg]['motor1'],
                leg_points[leg]['motor2'],
                leg_points[leg]['foot']
            ]

            color = colors[leg]

            for i in range(len(points) - 1):
                width = 3 - i * 0.5
                self.ax_3d.plot([points[i][0], points[i+1][0]],
                               [points[i][1], points[i+1][1]],
                               [points[i][2], points[i+1][2]],
                               color=color, linewidth=max(width, 1), alpha=0.8)

            for idx, point in enumerate(points[1:4]):
                self.ax_3d.scatter([point[0]], [point[1]], [point[2]],
                                  s=120, c='yellow', alpha=0.9, edgecolors='black', linewidth=2, marker='o')
                label_idx = leg * 3 + idx
                self.ax_3d.text(point[0] + 15, point[1], point[2] + 15,
                              f'M{label_idx}', color='black', fontsize=8, fontweight='bold')

        self.ax_3d.scatter([], [], [], s=120, c='yellow', edgecolors='black',
                          label='Motor')
        self.ax_3d.legend(loc='upper right', fontsize=8)
    
    def update_frame(self, time_step=None):
        if time_step is None:
            time_step = self.preview_time_step

        self.time += time_step

        self.gait_generator.frequency = 2.5

        angles = self.gait_generator.generate_rl_like_trot(self.time)
        
        for i, label in enumerate(self.joint_labels):
            leg_names = ['FL', 'FR', 'RL', 'RR']
            joint_names = ['Hip', 'Thigh', 'Knee']
            leg = i // 3
            joint = i % 3
            label.set_text(f'{leg_names[leg]}_{joint_names[joint]}: {angles[i]:.1f}°')
        
        state_name = ['INIT', 'WARMUP', 'GAIT'][self.gait_generator.state]
        self.info_label.set_text(f'Time: {self.time:.2f}s | State: {state_name} | Freq: {self.gait_generator.frequency}Hz')
        
        self.ax_3d.clear()
        
        self.ax_3d.set_xlim(-250, 250)
        self.ax_3d.set_ylim(-250, 250)
        self.ax_3d.set_zlim(-50, 500)
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D View')
        self.ax_3d.view_init(elev=25, azim=45)
        
        self.draw_body_simple()
        
        leg_points = self.calculate_leg_points(angles)
        self.draw_legs(leg_points)
        
        self.fig.suptitle(f'Quadruped Robot - Trot Gait Preview (Time: {self.time:.2f}s)', 
                         fontsize=16, fontweight='bold')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def animate(self):
        import time
        print("[Animation] Starting animation, press Ctrl+C to stop")
        frame_time = 1.0 / self.preview_freq
        time_scale = 2.0
        try:
            while self.running:
                start = time.time()
                self.update_frame(self.preview_time_step * time_scale)
                elapsed = time.time() - start
                sleep_time = max(0, frame_time - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\n[Animation] Stopped")


def main():
    parser = argparse.ArgumentParser(description='Quadruped Robot Trot Gait Simulator')
    parser.add_argument('--mode', choices=['server', 'preview', 'both'], default='both',
                       help='运行模式')
    parser.add_argument('--amp', type=float, default=25, help='步幅幅度 (度)')
    parser.add_argument('--freq', type=float, default=1.0, help='步态频率 (Hz)')
    parser.add_argument('--speed', type=float, default=1.0, help='前进速度')
    parser.add_argument('--port', type=int, default=9999, help='TCP端口')
    parser.add_argument('--gait', choices=['original', 'smooth', 'rl'], default='rl',
                       help='步态模式: original=原始正弦, smooth=S型曲线, rl=模拟RL(推荐)')
    parser.add_argument('--preview-freq', type=int, default=30,
                       help='动画刷新频率 (Hz)，默认30Hz')

    args = parser.parse_args()

    gait_generator = GaitGenerator()
    gait_generator.amplitude = args.amp
    gait_generator.frequency = args.freq
    
    print("=" * 60)
    print("  四足机器狗 Trot 步态模拟器")
    print("=" * 60)
    print(f"  步幅幅度: {args.amp}°")
    print(f"  步态频率: {args.freq} Hz")
    print(f"  前进速度: {args.speed}")
    print(f"  TCP端口:  {args.port}")
    print("=" * 60)
    print("  步态说明:")
    print("    FL (左前) + RR (右后) 同步")
    print("    FR (右前) + RL (左后) 同步")
    print("    这就是Trot对角小跑步态")
    print("=" * 60)
    
    if args.mode in ['server', 'both']:
        print('[启动] TCP服务器...')
        server = QuadTCPServer(port=args.port)
        server.set_params(args.amp, args.freq, args.speed)
        
        server_thread = threading.Thread(target=server.start)
        server_thread.daemon = True
        server_thread.start()
    
    if args.mode in ['preview', 'both']:
        if os.environ.get('DISPLAY') is None:
            print('[警告] 未检测到DISPLAY环境变量，跳过3D动画预览')
            print('[提示] 如需预览动画，请:')
            print('  - 在有显示器的电脑上运行: python trot_gait_simulator.py --mode preview')
            print('  - 或设置虚拟显示器: export DISPLAY=:0')
        else:
            print('[启动] 3D动画预览...')
            preview = GaitPreview(gait_generator, args.preview_freq)
            preview.gait_generator = gait_generator
            preview.animate()
    
    if args.mode == 'server':
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print('\n[停止] 服务器...')
            server.stop()


if __name__ == '__main__':
    main()
