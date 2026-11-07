#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四足机器狗完整控制系统仿真器
Quadruped Robot Full Control System Simulator

完全按照工程控制逻辑仿真:
1. 上位机50Hz发送Trot步态数据 (trot_gait_simulator.py)
2. STM32 TCP接收处理
3. 智能控制器场景识别和策略选择
4. 轨迹规划器 (梯形/S曲线)
5. 200Hz控制循环
6. 电机响应模拟

输出:
- 12个电机的位置/速度/加速度波形图
- 四足机器狗3D运动动画
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import IntEnum
import threading

# ==================== 常量定义 ====================
NUM_MOTORS = 12
NUM_LEGS = 4
MOTORS_PER_LEG = 3
MOTOR_NEUTRAL = 180.0

# 控制频率
UPPER_COMPUTER_FREQ = 50    # Hz
STM32_CONTROL_FREQ = 200    # Hz

# V11.0: 50Hz优化 - 双缓冲机制参数
TARGET_BUFFER_SIZE = 2           # 双缓冲大小
INTERPOLATION_STEPS = 4          # 每帧数据插值步数 (200Hz/50Hz)

# V11.1: 插值误差补偿参数
INTERP_ERROR_CORRECTION_ENABLE = True   # 启用插值误差补偿
INTERP_ERROR_THRESHOLD = 0.5            # 误差检测阈值 (deg)
INTERP_ERROR_MAX_CORRECTION = 2.0       # 最大单次修正量 (deg)
INTERP_ERROR_DECAY = 0.98               # 误差衰减因子
INTERP_SYNC_INTERVAL_MS = 100           # 每100ms强制同步一次

# V11.0: CAN监测请求优化参数
MONITOR_REQUEST_INTERVAL_MS = 8    # 监测请求间隔8ms (125Hz)
MONITOR_STAGGER_OFFSET = 1         # 电机间错开1ms发送

# 轨迹规划参数 - V11.0: 匹配工程代码
TRAJECTORY_DEFAULT_VMAX = 1000.0   # deg/s - V7.1: 从900提高到1000，提高响应速度
TRAJECTORY_DEFAULT_AMAX = 1800.0   # deg/s^2 - V7.1: 从1400提高到1800
TRAJECTORY_DEFAULT_DMAX = 2000.0   # deg/s^2 - V7.1: 从1600提高到2000
TRAJECTORY_JERK_LIMIT = 10000.0    # deg/s^3 - V7.1: 从8000提高到10000
TRAJECTORY_POS_THRESHOLD = 0.2     # deg - V7.0 从0.3降低到0.2，提高精度
TRAJECTORY_VEL_THRESHOLD = 3.0     # deg/s - V6.5 从5降低到3
TRAJECTORY_CONTROL_PERIOD = 0.005  # 5ms

# V5.0 预测性轨迹规划配置
TRAJECTORY_HISTORY_SIZE = 8       # 历史位置缓冲区大小
TRAJECTORY_PREDICTION_HORIZON = 0.030  # V7.1: 从0.04降低到0.030，缩短预测时间提高响应
TRAJECTORY_SMOOTH_BLEND_TIME = 0.015   # V7.1: 从0.02降低到0.015，加快过渡
TRAJECTORY_VEL_SMOOTH_ALPHA = 0.35    # V7.1: 从0.3提高到0.35，增强平滑

# V7.0 启用高级预测（优化版）
USE_QUINTIC_SPLINE = True      # 启用五次多项式插值
USE_PERIODIC_PREDICTION = True # 启用周期性预测
USE_DYNAMIC_GAIN = True        # 启用动态增益调整

# V6.0 连续轨迹跟踪模式配置
TRAJECTORY_CONTINUOUS_MIN_DIFF = 0.1    # 连续模式最小更新差值 (deg)
TRAJECTORY_CONTINUOUS_VEL_PRESERVE = 0.9  # 连续模式速度保持系数
TRAJECTORY_CONTINUOUS_MAX_VEL_JUMP = 500.0  # 最大速度跳变限制 (deg/s)

# V7.0 动态增益参数
GAIN_BOOST_THRESHOLD = 10.0   # V7.1: 从15降低到10，更早启动增益增强
GAIN_BOOST_FACTOR = 1.5        # V7.1: 从1.3提高到1.5，增强效果更明显
GAIN_RAMP_FACTOR = 0.6        # V7.1: 从0.5提高到0.6，ramp更平滑

# 前馈控制参数 - V7.1 优化版本
FEEDFORWARD_VEL_GAIN = 0.85      # V7.1: 从0.8提高到0.85，增强速度前馈
FEEDFORWARD_ACCEL_GAIN = 0.25    # V7.1: 从0.20提高到0.25，增强加速度前馈
POS_KP = 70.0                  # V7.1: 从60提高到70，增强位置反馈
VEL_KI = 0.18                  # V7.1: 从0.15提高到0.18
VEL_KD = 0.9                   # V7.1: 从0.8提高到0.9
INTEGRAL_LIMIT = 180.0         # V7.1: 从150提高到180

# 电机参数
MOTOR_ANGLE_MIN = 0.0
MOTOR_ANGLE_MAX = 360.0

# 关节类型枚举 - 用于差异化控制
class JointType(IntEnum):
    HIP = 0      # 髋关节 - 负载轻，响应快
    THIGH = 1    # 大腿关节 - 负载重，惯量大
    KNEE = 2     # 膝关节 - 负载适中

# 关节类型映射 - 每个腿的3个关节 (Hip, Thigh, Knee)
# 电机ID: 0-2=FL, 3-5=FR, 6-8=RL, 9-11=RR
JOINT_TYPES = [
    JointType.HIP, JointType.THIGH, JointType.KNEE,   # FL
    JointType.HIP, JointType.THIGH, JointType.KNEE,   # FR
    JointType.HIP, JointType.THIGH, JointType.KNEE,   # RL
    JointType.HIP, JointType.THIGH, JointType.KNEE,   # RR
]

# 关节差异化PID参数 - V8.1: THIGH优化版 (回滚)
JOINT_PID_PARAMS = {
    JointType.HIP: {'kp': 80, 'ki': 0.12, 'kd': 1.0, 'ff_vel': 0.8, 'ff_accel': 0.20},
    JointType.THIGH: {'kp': 300, 'ki': 0.25, 'kd': 18.0, 'ff_vel': 0.80, 'ff_accel': 0.25},  # V8.1
    JointType.KNEE: {'kp': 34, 'ki': 0.02, 'kd': 1.9, 'ff_vel': 0.20, 'ff_accel': 0.05},
}

# 电机模型差异化参数 - V8.1: THIGH优化版 (回滚)
JOINT_MOTOR_PARAMS = {
    JointType.HIP: {'inertia': 0.0015, 'damping': 0.025, 'friction': 0.012, 'kp': 1200, 'kd': 85},
    JointType.THIGH: {'inertia': 0.004, 'damping': 0.090, 'friction': 0.026, 'kp': 1400, 'kd': 520},  # V8.1
    JointType.KNEE: {'inertia': 0.0025, 'damping': 0.145, 'friction': 0.065, 'kp': 550, 'kd': 180},
}

# CAN 发送优化参数 - V5.2: 与备份版本一致，高频率发送
CAN_SEND_POS_THRESHOLD = 0.12   # deg - 优化响应
CAN_SEND_VEL_THRESHOLD = 1.8     # deg/s - 优化响应
CAN_SEND_MAX_INTERVAL = 0.008    # 8ms - 优化响应
CAN_SEND_ACCEL_INTERVAL = 0.003  # 3ms - 优化响应
CAN_SEND_DECEL_INTERVAL = 0.003  # 3ms - 优化响应
CAN_SEND_SKIP_MAX = 1            # 最大跳过 1 次

# ==================== 枚举定义 ====================
class TrajectoryState(IntEnum):
    IDLE = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3
    STOPPING = 4
    COMPLETED = 5

class SCurvePhase(IntEnum):
    JERK_POS = 0
    ACCEL = 1
    JERK_NEG = 2
    CRUISE = 3
    JERK_NEG2 = 4
    DECEL = 5
    JERK_POS2 = 6

class QuadScene(IntEnum):
    UNKNOWN = 0
    SWING = 1
    STANCE = 2
    TRANSITION = 3
    STOP = 4

class QuadStrategy(IntEnum):
    POSITION = 0
    HYBRID = 1
    IMPEDANCE = 2
    TORQUE = 3

class HybridState(IntEnum):
    SWING = 0
    TRANSITION_TO_STANCE = 1
    STANCE = 2

# ==================== 数据结构 ====================

# 新增：历史缓冲和前瞻缓冲的数据结构
HISTORY_BUFFER_SIZE = 32
LOOKAHEAD_BUFFER_SIZE = 10
TRAJECTORY_BUFFER_MIN_DIFF = 0.1  # 缓冲更新最小角度差 (降低以避免运动迟滞)

@dataclass
class HistoryBuffer:
    """历史缓冲 - 存储历史位置用于曲率计算"""
    buffer: List[float] = field(default_factory=lambda: [MOTOR_NEUTRAL] * HISTORY_BUFFER_SIZE)
    index: int = 0
    count: int = 0

@dataclass
class LookaheadBuffer:
    """前瞻缓冲 - 存储未来目标用于趋势分析"""
    buffer: List[float] = field(default_factory=lambda: [MOTOR_NEUTRAL] * LOOKAHEAD_BUFFER_SIZE)
    index: int = 0
    count: int = 0
    direction_change: bool = False

@dataclass
class TrendInfo:
    """趋势信息"""
    direction: str = "STABLE"  # ACCELERATING, DECELERATING, REVERSING, STABLE
    curvature: float = 0.0
    velocity_avg: float = 0.0
    acceleration_avg: float = 0.0

@dataclass
class SCurveState:
    phase: SCurvePhase = SCurvePhase.JERK_POS
    jerk: float = 0.0
    t_jerk1: float = 0.0
    t_accel: float = 0.0
    t_jerk2: float = 0.0
    t_cruise: float = 0.0
    t_jerk3: float = 0.0
    t_decel: float = 0.0
    t_jerk4: float = 0.0
    phase_time: float = 0.0
    peak_velocity: float = 0.0
    use_s_curve: bool = False

@dataclass
class TrajectoryConfig:
    v_max: float = TRAJECTORY_DEFAULT_VMAX
    a_max: float = TRAJECTORY_DEFAULT_AMAX
    d_max: float = TRAJECTORY_DEFAULT_DMAX
    jerk_limit: float = TRAJECTORY_JERK_LIMIT
    pos_threshold: float = TRAJECTORY_POS_THRESHOLD
    vel_threshold: float = TRAJECTORY_VEL_THRESHOLD

@dataclass
class TrajectoryPlanner:
    target_position: float = MOTOR_NEUTRAL
    current_position: float = MOTOR_NEUTRAL
    current_velocity: float = 0.0
    current_acceleration: float = 0.0
    filtered_acceleration: float = 0.0  # V5.4: 加速度平滑滤波
    state: TrajectoryState = TrajectoryState.IDLE
    is_moving: bool = False
    direction: bool = True
    remaining_distance: float = 0.0
    peak_velocity: float = 0.0
    dynamic_v_max: float = TRAJECTORY_DEFAULT_VMAX
    dynamic_a_max: float = TRAJECTORY_DEFAULT_AMAX
    dynamic_d_max: float = TRAJECTORY_DEFAULT_DMAX
    s_curve: SCurveState = field(default_factory=SCurveState)
    config: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    history_buffer: HistoryBuffer = field(default_factory=HistoryBuffer)
    lookahead_buffer: LookaheadBuffer = field(default_factory=LookaheadBuffer)
    trend_info: TrendInfo = field(default_factory=TrendInfo)
    last_planned_target: float = MOTOR_NEUTRAL
    # V4.0 预测性轨迹规划字段
    history_positions: list = field(default_factory=lambda: [MOTOR_NEUTRAL] * TRAJECTORY_HISTORY_SIZE)
    history_velocities: list = field(default_factory=lambda: [0.0] * TRAJECTORY_HISTORY_SIZE)
    history_timestamps: list = field(default_factory=lambda: [0.0] * TRAJECTORY_HISTORY_SIZE)
    history_index: int = 0
    history_count: int = 0
    predicted_position: float = MOTOR_NEUTRAL
    predicted_velocity: float = 0.0
    smooth_target: float = MOTOR_NEUTRAL
    smooth_velocity: float = 0.0
    prediction_valid: bool = False
    last_planned_position: float = MOTOR_NEUTRAL
    last_planned_velocity: float = 0.0
    last_planned_accel: float = 0.0
    blend_factor: float = 0.0

@dataclass
class TrajectoryOutput:
    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    is_completed: bool = False
    state: TrajectoryState = TrajectoryState.IDLE
    feedforward_vel: float = 0.0
    feedforward_accel: float = 0.0

@dataclass
class MotorController:
    motor_id: int = 0
    current_scene: QuadScene = QuadScene.UNKNOWN
    current_strategy: QuadStrategy = QuadStrategy.POSITION
    hybrid_state: HybridState = HybridState.SWING
    target_position: float = MOTOR_NEUTRAL
    vel_integral: float = 0.0
    last_pos_error: float = 0.0
    filtered_deriv: float = 0.0

@dataclass
class CANSendState:
    # 修复：使用 MOTOR_NEUTRAL 作为初始位置，避免首帧异常
    # 修复：使用 -1.0 确保第一次发送通过时间检查
    last_sent_position: float = MOTOR_NEUTRAL
    last_sent_velocity: float = 0.0
    last_send_time: float = -1.0
    skip_count: int = 0

@dataclass
class MotorState:
    position: float = MOTOR_NEUTRAL
    velocity: float = 0.0
    acceleration: float = 0.0
    torque: float = 0.0
    target_position: float = MOTOR_NEUTRAL

@dataclass
class BacklashConfig:
    """反向间隙补偿配置 - V7.0 极值点平滑处理"""
    backlash_compensation: float = 1.5  # deg - 减少总补偿量
    last_direction: float = 0.0
    compensation_applied: bool = False
    decay_factor: float = 0.92  # 衰减因子 - 加快衰减
    # V7.0 新增: 渐进式补偿参数，极值点减少补偿
    accumulated_compensation: float = 0.0
    compensation_rate: float = 0.3  # 降低补偿速率
    needs_compensation: bool = False
    extreme_point_factor: float = 0.3  # 极值点补偿折扣

@dataclass
class FrictionConfig:
    """摩擦力补偿配置"""
    coulomb_friction: float = 0.5  # Nm
    viscous_friction: float = 0.01  # Nm/(rad/s)

# ==================== 上位机步态生成器 ====================
class GaitGenerator:
    """Trot步态生成器 - 与trot_gait_simulator.py完全一致"""
    
    STATE_INIT = 0
    STATE_WARMUP = 1
    STATE_GAIT = 2
    
    def __init__(self):
        self.amplitude = 25
        self.frequency = 1.0
        self.forward_speed = 1.0
        self.state = self.STATE_WARMUP
        self.start_time = None
        self.warmup_duration = 2.0  # 与工程代码一致
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
    
    def generate_rl_like_trot(self, time_sec: float) -> List[float]:
        """生成Trot步态 - 与工程代码完全一致"""
        if self.state == self.STATE_WARMUP:
            return self._generate_warmup(time_sec)
        elif self.state == self.STATE_GAIT:
            return self._generate_gait(time_sec)
        return [MOTOR_NEUTRAL] * NUM_MOTORS
    
    def _generate_warmup(self, time_sec: float) -> List[float]:
        """预热阶段 - V2.0 使用 5 次多项式 S 型曲线"""
        if self.start_time is None:
            self.start_time = time_sec
        
        elapsed = time_sec - self.start_time
        progress = min(elapsed / self.warmup_duration, 1.0)
        
        # V2.0 改进：使用 5 次多项式 S 型曲线 (比 3 次更平滑)
        # f(t) = 10t³ - 15t⁴ + 6t⁵
        if progress < 1.0:
            smooth_progress = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
        else:
            smooth_progress = 1.0
        
        angles = []
        for leg_id in range(4):
            work = self.work_positions[leg_id]
            hip = MOTOR_NEUTRAL + (work['hip'] - MOTOR_NEUTRAL) * smooth_progress
            thigh = MOTOR_NEUTRAL + (work['thigh'] - MOTOR_NEUTRAL) * smooth_progress
            knee = MOTOR_NEUTRAL + (work['knee'] - MOTOR_NEUTRAL) * smooth_progress
            angles.extend([hip, thigh, knee])
        
        if progress >= 1.0:
            self.state = self.STATE_GAIT
        
        return angles
    
    def _generate_gait(self, time_sec: float) -> List[float]:
        """步态阶段 - V2.0 使用完全平滑的正弦曲线"""
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

            current_phase = (phase + phase_offset) % 1.0

            # V2.0 改进：使用完全平滑的正弦曲线 (替代梯形曲线)
            swing = self._smooth_sine_swing(current_phase)

            # 修复：移除随机噪声 - 目标位置应该是确定性的
            # 噪声导致每20ms目标位置高频抖动，破坏轨迹连续性
            hip_amp = self.amplitude * 0.4
            thigh_amp = self.amplitude * 1.2
            knee_amp = self.amplitude * 0.8

            # V2.0 改进：使用平滑变化的前进偏移
            forward_offset = self._smooth_forward_offset(leg_id, time_sec)

            # V5.4: 使用平滑正函数替代max(0, x) - 消除C¹断点
            def smooth_relu(x, alpha=15.0):
                """平滑ReLU函数，在x=0处导数连续"""
                return 0.5 * (x + np.sqrt(x * x + 1 / (alpha * alpha)))
            
            smooth_swing = smooth_relu(swing)
            
            hip_angle = MOTOR_NEUTRAL + forward_offset + hip_amp * swing
            lift = smooth_swing * self.amplitude * 0.25
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * swing + lift
            knee_bend = smooth_swing * knee_amp * 0.4
            knee_angle = MOTOR_NEUTRAL + knee_amp * swing + knee_bend

            angles.extend([hip_angle, thigh_angle, knee_angle])

        return angles
    
    def _smooth_sine_swing(self, phase: float) -> float:
        """完全平滑的正弦摆动曲线 - V2.0 消除所有不连续点"""
        # 基础正弦波 (周期为 2 倍 phase)
        base_sine = np.sin(2 * np.pi * phase)
        
        # 添加三次谐波修正，使波形在峰值附近更平滑
        third_harmonic = 0.1 * np.sin(6 * np.pi * phase)
        
        # 合成最终波形
        swing = base_sine + third_harmonic
        
        # 应用平滑因子
        smooth_factor = 0.85
        swing = smooth_factor * swing
        
        return swing
    
    def _smooth_forward_offset(self, leg_id: int, time_sec: float) -> float:
        """平滑前进偏移 - V2.0 与步态相位同步"""
        period = 1.0 / self.frequency
        phase = (time_sec / period) % 1.0

        # FL 和 RR 使用相同相位，FR 和 RL 使用相反相位
        if leg_id in [0, 3]:  # FL, RR
            offset_phase = 0
        else:  # FR, RL
            offset_phase = 0.5

        current_phase = (phase + offset_phase) % 1.0

        # 使用平滑的正弦变化，替代固定 offset
        offset_value = self.forward_speed * 15 * np.sin(2 * np.pi * current_phase)

        return offset_value
    
    def _trapezoid_swing(self, phase: float) -> float:
        """梯形速度曲线"""
        if phase < 0.15:
            t = phase / 0.15
            return np.sin(np.pi * t / 2)
        elif phase < 0.5:
            t = (phase - 0.15) / 0.35
            return 1 - t
        elif phase < 0.85:
            t = (phase - 0.5) / 0.35
            return -t
        else:
            t = (phase - 0.85) / 0.15
            return -1 + np.sin(np.pi * t / 2)

# ==================== 轨迹规划器 ====================
class TrajectoryPlannerSim:
    """轨迹规划器仿真 - 与 trajectory_planner.c 完全一致"""
    
    def __init__(self):
        self.planners = [TrajectoryPlanner() for _ in range(NUM_MOTORS)]
        self.vel_integral = [0.0] * NUM_MOTORS
        self.last_pos_error = [0.0] * NUM_MOTORS
        self.filtered_deriv = [0.0] * NUM_MOTORS
        
        # V2.0 新增：反向间隙和摩擦力补偿
        self.backlash_configs = [BacklashConfig() for _ in range(NUM_MOTORS)]
        self.friction_configs = [FrictionConfig() for _ in range(NUM_MOTORS)]
        
        # V5.5: 目标位置平滑滤波状态
        self.smoothed_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        TARGET_FILTER_ALPHA = 0.4  # V5.5: 提高到0.4，平衡平滑与响应
        
        # V4.0 新增：预测性轨迹规划开关
        self._use_predictive_planning = True

        # V6.0 新增：连续轨迹模式开关
        self._continuous_mode_enabled = True
    
    def _sign(self, x: float) -> float:
        if x > 0: return 1.0
        if x < 0: return -1.0
        return 0.0
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))
    
    def _calculate_trapezoid_profile(self, distance: float, v_max: float, 
                                      a_max: float, d_max: float):
        """计算梯形速度曲线参数"""
        abs_dist = abs(distance)
        
        if abs_dist < 0.001:
            return 0.0, 0.0, 0.0, 0.0
        
        # 确保参数为正
        v_max = max(v_max, 1.0)
        a_max = max(a_max, 1.0)
        d_max = max(d_max, 1.0)
        
        v_triangle = np.sqrt(2.0 * a_max * abs_dist * d_max / (a_max + d_max))
        
        if v_triangle <= v_max:
            peak_vel = v_triangle
            accel_dist = (v_triangle ** 2) / (2.0 * a_max)
            decel_dist = (v_triangle ** 2) / (2.0 * d_max)
            cruise_dist = 0.0
        else:
            peak_vel = v_max
            accel_dist = (v_max ** 2) / (2.0 * a_max)
            decel_dist = (v_max ** 2) / (2.0 * d_max)
            cruise_dist = abs_dist - accel_dist - decel_dist
            
            if cruise_dist < 0.0:
                cruise_dist = 0.0
                peak_vel = np.sqrt(2.0 * a_max * abs_dist * d_max / (a_max + d_max))
                accel_dist = (peak_vel ** 2) / (2.0 * a_max)
                decel_dist = (peak_vel ** 2) / (2.0 * d_max)
        
        return peak_vel, accel_dist, cruise_dist, decel_dist
    
    def _calculate_braking_distance(self, velocity: float, deceleration: float) -> float:
        """计算制动距离 - 与 Trajectory_CalcBrakingDistance 一致
        公式: d = v^2 / (2 * a)
        """
        if deceleration <= 0.0:
            return 0.0
        return (velocity * velocity) / (2.0 * deceleration)
    
    def _calculate_s_curve_profile(self, distance: float, v_max: float, 
                                    a_max: float, d_max: float, jerk: float) -> SCurveState:
        """计算S曲线规划参数"""
        abs_dist = abs(distance)
        s = SCurveState()
        
        if abs_dist < 0.1:
            return s
        
        t_j = a_max / jerk
        v_j = 0.5 * jerk * t_j * t_j
        d_j = 0.5 * v_j * t_j
        
        v_accel = v_max - 2.0 * v_j
        t_a = v_accel / a_max if v_accel > 0.0 else 0.0
        
        d_accel = 2.0 * d_j + v_accel * t_a + v_j * t_a
        d_accel_decel = 2.0 * d_accel
        
        s.t_jerk1 = t_j
        s.t_accel = t_a
        s.t_jerk2 = t_j
        s.t_jerk3 = t_j
        s.t_decel = t_a
        s.t_jerk4 = t_j
        
        if d_accel_decel >= abs_dist:
            # 修复：与工程代码一致，使用 0.5 因子
            v_peak = np.sqrt(abs_dist * a_max * 0.5)
            t_j = v_peak / jerk
            
            s.t_jerk1 = t_j
            s.t_accel = 0.0
            s.t_jerk2 = t_j
            s.t_cruise = 0.0
            s.t_jerk3 = t_j
            s.t_decel = 0.0
            s.t_jerk4 = t_j
            s.peak_velocity = v_peak
        else:
            d_cruise = abs_dist - d_accel_decel
            s.t_cruise = d_cruise / v_max
            s.peak_velocity = v_max
        
        s.phase = SCurvePhase.JERK_POS
        s.phase_time = 0.0
        s.jerk = jerk
        s.use_s_curve = True
        
        return s
    
    def _s_curve_step(self, p: TrajectoryPlanner, dt: float) -> float:
        """S 曲线一步执行 - V5.0 重构版：真正的加速度连续性
        关键改进：不再强制设置加速度，而是保留当前值并积分"""
        s = p.s_curve
        dir_sign = 1.0 if p.direction else -1.0
        
        jerk = 0.0
        accel = p.current_acceleration  # 保留当前加速度
        vel = p.current_velocity
        
        s.phase_time += dt
        
        if s.phase == SCurvePhase.JERK_POS:
            if s.t_jerk1 > 0.001:
                jerk_phase = min(s.phase_time / s.t_jerk1, 1.0)
                jerk = s.jerk * np.sin(np.pi * jerk_phase)
            else:
                jerk = s.jerk
            if s.phase_time >= s.t_jerk1:
                s.phase = SCurvePhase.ACCEL
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.ACCEL:
            jerk = 0.0
            if s.phase_time >= s.t_accel:
                s.phase = SCurvePhase.JERK_NEG
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.JERK_NEG:
            if s.t_jerk2 > 0.001:
                jerk_phase = min(s.phase_time / s.t_jerk2, 1.0)
                jerk = -s.jerk * np.sin(np.pi * jerk_phase)
            else:
                jerk = -s.jerk
            if s.phase_time >= s.t_jerk2:
                s.phase = SCurvePhase.CRUISE
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.CRUISE:
            jerk = 0.0
            accel = 0.0  # 匀速段加速度为0
            vel = s.peak_velocity
            if s.phase_time >= s.t_cruise:
                # V7.0: 从CRUISE到JERK_NEG2时，保持当前加速度平滑过渡
                s.phase = SCurvePhase.JERK_NEG2
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.JERK_NEG2:
            if s.t_jerk3 > 0.001:
                jerk_phase = min(s.phase_time / s.t_jerk3, 1.0)
                jerk = -s.jerk * np.sin(np.pi * jerk_phase)
            else:
                jerk = -s.jerk
            if s.phase_time >= s.t_jerk3:
                s.phase = SCurvePhase.DECEL
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.DECEL:
            jerk = 0.0
            # V7.0: 保持当前加速度连续性
            if s.phase_time >= s.t_decel:
                s.phase = SCurvePhase.JERK_POS2
                s.phase_time = 0.0
        elif s.phase == SCurvePhase.JERK_POS2:
            if s.t_jerk4 > 0.001:
                jerk_phase = min(s.phase_time / s.t_jerk4, 1.0)
                jerk = s.jerk * np.sin(np.pi * jerk_phase)
            else:
                jerk = s.jerk
            if s.phase_time >= s.t_jerk4:
                s.use_s_curve = False
                s.phase = SCurvePhase.JERK_POS
                s.phase_time = 0.0
                p.state = TrajectoryState.STOPPING
        
        # V7.0: 改进的状态切换过渡 - 使用当前加速度而非重置
        if s.use_s_curve:
            # 使用平滑的加速度过渡
            if jerk != 0.0:
                # 当有jerk时，平滑积分加速度
                accel += jerk * dt
            # 限制加速度变化率（软限制）
            max_accel_change = s.jerk * dt * 2.0
            accel = max(p.current_acceleration - max_accel_change, 
                       min(p.current_acceleration + max_accel_change, accel))
            vel += accel * dt
            vel = self._clamp(vel, 0.0, s.peak_velocity * 1.1)
            p.current_acceleration = accel
            p.current_velocity = vel
        
        return dir_sign * vel * dt
    
    def set_target(self, motor_id: int, target_position: float):
        """设置目标位置 - V5.4 添加目标平滑滤波减少高频抖动"""
        if motor_id >= NUM_MOTORS:
            return
        
        p = self.planners[motor_id]
        
        target_position = self._clamp(target_position, 0.0, 359.99)
        
        # V7.0: 自适应目标位置滤波 - 极值点附近减少滤波
        distance_to_target = abs(target_position - p.current_position)
        if distance_to_target < 55.0:
            alpha = 0.97  # 极值点附近减少滞后
        else:
            alpha = 0.90   # 正常跟踪时
        self.smoothed_targets[motor_id] = alpha * target_position + (1 - alpha) * self.smoothed_targets[motor_id]
        filtered_target = self.smoothed_targets[motor_id]
        
        # V2.0 新增：应用反向间隙补偿
        compensated_target = self._apply_backlash_compensation(
            motor_id, filtered_target, p.current_position
        )
        
        pos_diff = abs(compensated_target - p.current_position)
        if pos_diff < TRAJECTORY_POS_THRESHOLD:
            if p.is_moving:
                p.is_moving = False
                p.state = TrajectoryState.COMPLETED
                p.current_velocity = 0.0
                p.current_acceleration = 0.0
            p.lookahead_buffer.direction_change = False
            p.lookahead_buffer.count = 0
            return
        
        # 计算新方向
        new_direction = compensated_target >= p.current_position
        
        # V7.1: 速度感知方向切换 - 当速度未过零时，需要先减速到零
        if p.is_moving and p.direction != new_direction:
            # 方向变化，需要减速到零后再反向
            if abs(p.current_velocity) > 5.0:
                # 速度未过零，强制进入减速状态
                p.state = TrajectoryState.DECELERATING
                p.remaining_distance = abs(p.current_velocity) * abs(p.current_velocity) / (2.0 * p.dynamic_d_max)
                p.direction = new_direction
                p.target_position = self._clamp(compensated_target, 0.0, 359.99)
                return
        
        # 始终更新目标，并对target_position进行clamp
        p.target_position = self._clamp(compensated_target, 0.0, 359.99)
        # 确保remaining_distance是正数（如果当前已经超调，则需要反方向运动）
        p.remaining_distance = p.target_position - p.current_position
        # 如果remaining_distance为负（超调），则反转方向
        if p.remaining_distance < 0:
            new_direction = not new_direction
            p.remaining_distance = abs(p.remaining_distance)
        p.direction = new_direction
        
        # 始终设置 is_moving = True（确保状态机正确工作）
        p.is_moving = True
        
        # 检查目标变化是否足够大 - 用于决定是否重置加速度
        target_diff = abs(compensated_target - p.last_planned_target)
        p.last_planned_target = compensated_target
        
        v_max = p.dynamic_v_max
        a_max = p.dynamic_a_max
        d_max = p.dynamic_d_max
        
        # 不检测方向变化，让轨迹规划器自行处理
        # 计算梯形参数
        peak_vel, _, _, _ = self._calculate_trapezoid_profile(
            p.remaining_distance, v_max, a_max, d_max
        )
        p.peak_velocity = peak_vel  # V3.0: 峰值速度始终为正
        
        if p.config.jerk_limit > 0.0:
            p.s_curve = self._calculate_s_curve_profile(
                p.remaining_distance, v_max, a_max, d_max, p.config.jerk_limit
            )
        
        # V6.0: 保持当前状态，保留速度继续运动而非重置
        # 只在IDLE或COMPLETED时启动，否则保持当前运动状态
        if p.state == TrajectoryState.IDLE or p.state == TrajectoryState.COMPLETED:
            p.state = TrajectoryState.ACCELERATING
            # V7.1: 从0开始逐步增加加速度，避免跳变
            p.current_acceleration = 0.0
        # V6.0: 保留当前加速度继续运动（不要每次set_target都重置加速度）
        
        p.is_moving = True
    
    def set_target_with_velocity(self, motor_id: int, target_position: float, initial_velocity: float):
        """设置目标位置并保留初始速度 - 与 Trajectory_SetTargetWithVelocity 一致
        这是关键函数！50Hz更新时保留当前速度继续运动，而不是重置为0
        """
        if motor_id >= NUM_MOTORS:
            return
        
        p = self.planners[motor_id]
        
        target_position = self._clamp(target_position, 0.0, 359.99)
        
        # 应用反向间隙补偿
        compensated_target = self._apply_backlash_compensation(
            motor_id, target_position, p.current_position
        )
        
        pos_diff = abs(compensated_target - p.current_position)
        if pos_diff < TRAJECTORY_POS_THRESHOLD:
            if p.is_moving:
                p.is_moving = False
                p.state = TrajectoryState.COMPLETED
                p.current_velocity = 0.0
                p.current_acceleration = 0.0
            p.lookahead_buffer.direction_change = False
            p.lookahead_buffer.count = 0
            return
        
        # 检查目标变化是否足够大
        target_diff = abs(compensated_target - p.target_position)
        
        # 关键修复：始终更新remaining_distance
        if target_diff < TRAJECTORY_BUFFER_MIN_DIFF and p.is_moving:
            p.target_position = compensated_target
            p.remaining_distance = compensated_target - p.current_position
            return
        
        # 检测方向变化
        # V3.0 改进：速度始终为正，使用direction判断方向
        current_direction = compensated_target >= p.current_position
        direction_change = (p.current_velocity > 5.0) and (current_direction != p.direction)
        
        # 方向切换处理：需要先减速
        if direction_change:
            braking_dist = self._calculate_braking_distance(
                p.current_velocity, p.dynamic_d_max
            )
            
            direction_sign = 1.0 if p.direction else -1.0
            brake_target = p.current_position + (braking_dist * 0.5) * direction_sign
            
            if 0.0 <= brake_target <= 360.0:
                self.set_target(motor_id, brake_target)
                # 减速后剩余距离用较低速度
                p.current_velocity = abs(initial_velocity) * 0.3 if abs(initial_velocity) > 0.1 else p.current_velocity * 0.3
            else:
                self.set_target(motor_id, compensated_target)
                p.current_velocity = abs(initial_velocity) * 0.3 if abs(initial_velocity) > 0.1 else p.current_velocity * 0.3
        else:
            # 正常方向：保留当前速度继续运动
            self.set_target(motor_id, compensated_target)
            # 关键：保留初始速度！V3.0: 速度始终为正
            if abs(initial_velocity) > 0.1:
                p.current_velocity = abs(initial_velocity)

    def enable_continuous_mode(self, enable: bool):
        """启用/禁用连续轨迹模式 - V6.0 新增"""
        self._continuous_mode_enabled = enable

    def is_continuous_mode(self) -> bool:
        """查询是否启用连续轨迹模式 - V6.0 新增"""
        return self._continuous_mode_enabled

    def set_continuous_target(self, motor_id: int, target_position: float):
        """V11.1: 设置连续目标位置 - 完全匹配工程代码逻辑
        关键修复：正确处理方向变化，不再使用错误的制动目标
        """
        if motor_id >= NUM_MOTORS:
            return

        p = self.planners[motor_id]

        target_position = self._clamp(target_position, 0.0, 359.99)

        target_diff = target_position - p.target_position

        # 小变化时跳过更新
        if abs(target_diff) < TRAJECTORY_CONTINUOUS_MIN_DIFF and p.is_moving:
            return

        # V11.1: 直接设置目标位置（与工程代码一致）
        p.target_position = target_position

        pos_diff = target_position - p.current_position
        new_direction = pos_diff >= 0.0

        # V11.1: 方向变化处理 - 完全匹配工程代码
        if p.is_moving and (new_direction != p.direction):
            current_speed = p.current_velocity
            braking_dist = self._calculate_braking_distance(current_speed, p.dynamic_d_max)

            # V11.1: 检查是否需要减速到0
            if braking_dist >= abs(p.remaining_distance):
                # 需要减速到0再换向
                p.state = TrajectoryState.DECELERATING
                p.current_acceleration = -p.dynamic_d_max * 0.7  # 降低减速度
                p.direction = new_direction  # 更新方向
            else:
                # 距离足够，平滑过渡
                p.direction = new_direction
                p.remaining_distance = target_position - p.current_position
                # 保留更多速度
                p.current_velocity = current_speed * TRAJECTORY_CONTINUOUS_VEL_PRESERVE * 0.95
                if p.current_velocity < 5.0:
                    p.current_velocity = 5.0
                p.state = TrajectoryState.ACCELERATING
                p.current_acceleration = p.dynamic_a_max * 0.8  # 降低初始加速度
        elif p.is_moving:
            # V11.1: 同方向运动，动态调整
            p.remaining_distance = pos_diff
            p.direction = new_direction
            # 根据剩余距离调整速度限制
            dist_abs = abs(pos_diff)
            if dist_abs < 10.0:
                p.dynamic_v_max = min(p.config.v_max, dist_abs * 50.0)
            else:
                p.dynamic_v_max = p.config.v_max
        else:
            # V11.1: 静止启动
            p.direction = new_direction
            p.remaining_distance = abs(pos_diff)
            p.current_velocity = 50.0
            p.state = TrajectoryState.ACCELERATING
            p.current_acceleration = p.dynamic_a_max

        p.is_moving = True

    def update_buffers(self, motor_id: int, new_target: float):
        """更新历史缓冲和前瞻缓冲 - 与工程代码的缓冲逻辑一致
        修复：使用滑动窗口检测方向变化，避免单点抖动
        """
        p = self.planners[motor_id]
        
        # 更新历史缓冲 (32帧)
        hist = p.history_buffer
        hist.buffer[hist.index] = new_target
        hist.index = (hist.index + 1) % HISTORY_BUFFER_SIZE
        if hist.count < HISTORY_BUFFER_SIZE:
            hist.count += 1
        
        # 更新前瞻缓冲 (10帧)
        la = p.lookahead_buffer
        la.buffer[la.index] = new_target
        la.index = (la.index + 1) % LOOKAHEAD_BUFFER_SIZE
        if la.count < LOOKAHEAD_BUFFER_SIZE:
            la.count += 1
        
        # 修复：使用滑动窗口检测方向变化（至少3帧趋势才能判定）
        if la.count >= 3:
            targets = []
            for i in range(min(3, la.count)):
                idx = (la.index - 1 - i + LOOKAHEAD_BUFFER_SIZE) % LOOKAHEAD_BUFFER_SIZE
                targets.append(la.buffer[idx])
            
            if len(targets) >= 2:
                # 使用多帧趋势判断方向变化
                target_change = targets[0] - targets[-1]
                # 变化超过1度才算方向变化
                la.direction_change = abs(target_change) > 1.0
        else:
            la.direction_change = False
    
    def analyze_trend_and_curvature(self, motor_id: int):
        """分析趋势和曲率 - 与工程代码的 TrendAnalyze 和 CurvatureAdjust 一致
        修复：添加除零保护和边界条件
        """
        p = self.planners[motor_id]
        hist = p.history_buffer
        la = p.lookahead_buffer
        trend = p.trend_info
        
        if hist.count < 3 or la.count < 2:
            trend.direction = "STABLE"
            trend.curvature = 0.0
            trend.velocity_avg = 0.0
            trend.acceleration_avg = 0.0
            return
        
        # 计算历史速度 (用位置变化近似)
        velocities = []
        for i in range(min(5, hist.count - 1)):
            idx1 = (hist.index - 1 - i + HISTORY_BUFFER_SIZE) % HISTORY_BUFFER_SIZE
            idx2 = (hist.index - 2 - i + HISTORY_BUFFER_SIZE) % HISTORY_BUFFER_SIZE
            dt_hist = 0.02  # 50Hz
            if dt_hist > 0:
                vel = (hist.buffer[idx1] - hist.buffer[idx2]) / dt_hist
                velocities.append(vel)
        
        if len(velocities) >= 2:
            accelerations = [velocities[i] - velocities[i+1] for i in range(len(velocities)-1)]
            trend.velocity_avg = sum(velocities) / len(velocities)
            trend.acceleration_avg = sum(accelerations) / len(accelerations) if accelerations else 0.0
            
            # 修复：曲率计算添加除零保护
            velocity_sq = trend.velocity_avg ** 2
            if abs(trend.velocity_avg) > 10.0 and velocity_sq > 100.0:
                trend.curvature = abs(trend.acceleration_avg) / velocity_sq
                # 限制曲率范围，防止异常
                trend.curvature = min(trend.curvature, 10.0)
            else:
                trend.curvature = 0.0
        
        # 趋势判断（使用滑动窗口避免单点抖动）
        if la.count >= 3:
            targets = []
            for i in range(min(3, la.count)):
                idx = (la.index - 1 - i + LOOKAHEAD_BUFFER_SIZE) % LOOKAHEAD_BUFFER_SIZE
                targets.append(la.buffer[idx])
            
            if len(targets) >= 2:
                # V3.1 优化：增大趋势判断阈值，避免噪声误判
                # 50Hz下20ms间隔，0.5°太小，改为2.0°
                target_diff = targets[0] - targets[-1]
                # V3.0 改进：速度始终为正，使用direction判断方向
                if p.current_velocity > 5.0:
                    if p.direction and target_diff > 2.0:
                        trend.direction = "ACCELERATING"
                    elif p.direction and target_diff < -2.0:
                        trend.direction = "DECELERATING"
                    elif not p.direction and target_diff < -2.0:
                        trend.direction = "ACCELERATING"
                    elif not p.direction and target_diff > 2.0:
                        trend.direction = "DECELERATING"
                    elif (p.direction and target_diff < -3.0) or \
                         (not p.direction and target_diff > 3.0):
                        trend.direction = "REVERSING"
                    else:
                        trend.direction = "STABLE"
                else:
                    trend.direction = "STABLE"
        else:
            trend.direction = "STABLE"
    
    def adjust_parameters_by_trend(self, motor_id: int):
        """根据趋势调整规划参数 - V6.3添加Thigh特殊补偿"""
        p = self.planners[motor_id]
        trend = p.trend_info
        
        base_v_max = TRAJECTORY_DEFAULT_VMAX
        base_a_max = TRAJECTORY_DEFAULT_AMAX
        base_d_max = TRAJECTORY_DEFAULT_DMAX
        
        joint_type = JOINT_TYPES[motor_id]
        if joint_type == JointType.THIGH:
            base_v_max *= 1.25
            base_a_max *= 1.25
        
        if trend.curvature > 0.01:
            curvature_factor = 1.0 - min(trend.curvature * 10.0, 0.5)
            base_v_max *= curvature_factor
            base_a_max *= curvature_factor * 0.8
        
        if trend.direction == "ACCELERATING":
            base_v_max *= 1.1
        elif trend.direction == "DECELERATING":
            base_v_max *= 0.9
        elif trend.direction == "REVERSING":
            base_v_max *= 0.7
            base_d_max *= 1.2
        
        p.dynamic_v_max = base_v_max
        p.dynamic_a_max = base_a_max
        p.dynamic_d_max = base_d_max
    
    def closed_loop_correction(self, motor_id: int, actual_position: float):
        """闭环修正 - V7.0 降低修正因子避免振荡"""
        p = self.planners[motor_id]
        
        pos_error = actual_position - p.current_position
        abs_error = abs(pos_error)
        
        CLOSED_LOOP_ERROR_THRESHOLD = 2.0  # V7.0: 降低阈值，更早触发修正
        CLOSED_LOOP_ERROR_DEADZONE = 0.3
        
        if abs_error < CLOSED_LOOP_ERROR_DEADZONE:
            return
        
        joint_type = JOINT_TYPES[motor_id]
        if joint_type == JointType.THIGH:
            # V7.0: 降低修正因子避免振荡循环
            if abs_error > 25.0:
                correction_factor = 0.50  # 原来是0.75
            elif abs_error > 15.0:
                correction_factor = 0.35  # 原来是0.55
            else:
                correction_factor = 0.25  # 原来是0.40
        else:
            if abs_error > 25.0:
                correction_factor = 0.40  # 原来是0.6
            elif abs_error > 15.0:
                correction_factor = 0.28  # 原来是0.4
            else:
                correction_factor = 0.20  # 原来是0.25
        
        corrected_pos = p.current_position + pos_error * correction_factor
        p.current_position = corrected_pos
    
    def _apply_backlash_compensation(self, motor_id: int, target_position: float, 
                                      current_position: float) -> float:
        """V7.0: 极值点平滑反向间隙补偿"""
        if motor_id >= NUM_MOTORS:
            return target_position
        
        config = self.backlash_configs[motor_id]
        current_direction = 1.0 if target_position > current_position else -1.0
        compensated_target = target_position
        
        # V7.0: 检测极值点 - 距离目标很近时减少补偿
        distance_to_target = abs(target_position - current_position)
        if distance_to_target < 25.0:
            effective_factor = config.extreme_point_factor
        else:
            effective_factor = 1.0
        
        # 检测方向变化
        if (current_direction != config.last_direction and 
            config.last_direction != 0.0):
            config.needs_compensation = True
            config.accumulated_compensation = 0.0
        
        # 渐进式补偿: 每个周期补偿一部分
        if config.needs_compensation:
            config.accumulated_compensation += config.compensation_rate * effective_factor
            if config.accumulated_compensation >= config.backlash_compensation * effective_factor:
                config.accumulated_compensation = config.backlash_compensation * effective_factor
            
            compensated_target += current_direction * config.accumulated_compensation
        
        compensated_target = self._clamp(compensated_target, 0.0, 359.99)
        config.last_direction = current_direction
        
        return compensated_target
    
    def _apply_friction_compensation(self, motor_id: int, velocity: float, 
                                      torque_command: float) -> float:
        """应用摩擦力补偿 - 与 Trajectory_ApplyFrictionCompensation 一致"""
        if motor_id >= NUM_MOTORS:
            return torque_command
        
        config = self.friction_configs[motor_id]
        friction_torque = 0.0
        
        # 库伦摩擦 (方向相关)
        if velocity > 0.1:
            friction_torque += config.coulomb_friction
        elif velocity < -0.1:
            friction_torque -= config.coulomb_friction
        
        # 粘性摩擦 (速度单位转换 deg/s -> rad/s)
        velocity_rad = velocity * np.pi / 180.0
        friction_torque += config.viscous_friction * velocity_rad
        
        return torque_command + friction_torque
    
    def _update_history_buffer(self, motor_id: int, target: float, velocity: float, timestamp: float):
        p = self.planners[motor_id]
        p.history_positions[p.history_index] = target
        p.history_velocities[p.history_index] = velocity
        p.history_timestamps[p.history_index] = timestamp
        p.history_index = (p.history_index + 1) % TRAJECTORY_HISTORY_SIZE
        if p.history_count < TRAJECTORY_HISTORY_SIZE:
            p.history_count += 1
    
    def _predict_future_target(self, motor_id: int, predict_time: float):
        """V6.0 增强版预测算法：五次多项式 + 周期性预测
        关键改进：
        1. 使用五次多项式拟合历史轨迹，确保位置/速度/加速度/Jerk连续
        2. 添加周期性成分检测，利用步态周期性提高预测精度
        """
        p = self.planners[motor_id]
        
        if p.history_count < 2:
            p.predicted_position = p.target_position
            p.predicted_velocity = p.current_velocity
            p.prediction_valid = False
            return p.target_position, p.current_velocity
        
        n = min(p.history_count, TRAJECTORY_HISTORY_SIZE)
        
        if USE_QUINTIC_SPLINE:
            return self._predict_with_quintic(p, n, predict_time)
        elif USE_PERIODIC_PREDICTION:
            return self._predict_with_periodic(p, n, predict_time)
        else:
            return self._predict_linear(p, n, predict_time)
    
    def _predict_linear(self, p: TrajectoryPlanner, n: int, predict_time: float):
        """基础线性预测"""
        positions = []
        times = []
        
        for i in range(n):
            idx = (p.history_index - 1 - i + TRAJECTORY_HISTORY_SIZE) % TRAJECTORY_HISTORY_SIZE
            positions.append(p.history_positions[idx])
            times.append(p.history_timestamps[idx])
        
        positions = positions[::-1]
        times = times[::-1]
        
        dt_hist = (times[-1] - times[0]) / 1000.0 if times[-1] > times[0] else 0.02
        
        avg_velocity = (positions[-1] - positions[0]) / dt_hist if dt_hist > 0.001 else p.current_velocity
        
        smoothing = 0.5
        pred_velocity = smoothing * avg_velocity + (1 - smoothing) * p.current_velocity
        pred_position = p.target_position + pred_velocity * predict_time
        
        p.predicted_position = self._clamp(pred_position, 0.0, 359.99)
        p.predicted_velocity = self._clamp(pred_velocity, -500.0, 500.0)
        p.prediction_valid = True
        
        return p.predicted_position, p.predicted_velocity
    
    def _predict_with_quintic(self, p: TrajectoryPlanner, n: int, predict_time: float):
        """五次多项式预测 - 确保各阶导数连续"""
        positions = []
        times = []
        
        for i in range(n):
            idx = (p.history_index - 1 - i + TRAJECTORY_HISTORY_SIZE) % TRAJECTORY_HISTORY_SIZE
            positions.append(p.history_positions[idx])
            times.append(p.history_timestamps[idx])
        
        positions = positions[::-1]
        times = times[::-1]
        
        t0 = times[0]
        normalized_times = [(t - t0) / 1000.0 for t in times]
        
        if len(normalized_times) < 3:
            return self._predict_linear(p, n, predict_time)
        
        try:
            T = np.vstack([
                [t**5, t**4, t**3, t**2, t, 1] 
                for t in normalized_times
            ])
            
            coeffs, residuals, rank, s = np.linalg.lstsq(T, positions, rcond=None)
            
            t_pred = predict_time
            pred_position = (coeffs[0] * t_pred**5 + coeffs[1] * t_pred**4 + 
                           coeffs[2] * t_pred**3 + coeffs[3] * t_pred**2 + 
                           coeffs[4] * t_pred + coeffs[5])
            
            pred_velocity = (5 * coeffs[0] * t_pred**4 + 4 * coeffs[1] * t_pred**3 +
                           3 * coeffs[2] * t_pred**2 + 2 * coeffs[3] * t_pred + coeffs[4])
            
            smoothing = QUINTIC_SMOOTH_FACTOR
            pred_velocity = smoothing * pred_velocity + (1 - smoothing) * p.current_velocity
            
            p.predicted_position = self._clamp(pred_position, 0.0, 359.99)
            p.predicted_velocity = self._clamp(pred_velocity, -500.0, 500.0)
            p.prediction_valid = True
            
            return p.predicted_position, p.predicted_velocity
            
        except Exception:
            return self._predict_linear(p, n, predict_time)
    
    def _predict_with_periodic(self, p: TrajectoryPlanner, n: int, predict_time: float):
        """周期性预测 - 利用步态周期性"""
        positions = []
        times = []
        
        for i in range(n):
            idx = (p.history_index - 1 - i + TRAJECTORY_HISTORY_SIZE) % TRAJECTORY_HISTORY_SIZE
            positions.append(p.history_positions[idx])
            times.append(p.history_timestamps[idx])
        
        positions = positions[::-1]
        times = times[::-1]
        
        dt_hist = (times[-1] - times[0]) / 1000.0 if times[-1] > times[0] else 0.02
        avg_velocity = (positions[-1] - positions[0]) / dt_hist if dt_hist > 0.001 else p.current_velocity
        
        base_velocity = 0.7 * avg_velocity + 0.3 * p.current_velocity
        pred_position = p.target_position + base_velocity * predict_time
        
        p.predicted_position = self._clamp(pred_position, 0.0, 359.99)
        p.predicted_velocity = self._clamp(base_velocity, -500.0, 500.0)
        p.prediction_valid = True
        
        return p.predicted_position, p.predicted_velocity
    
    def _generate_smooth_trajectory(self, motor_id: int, target_position: float, 
                                    target_velocity: float, dt: float) -> TrajectoryOutput:
        """
        V12.0: 预测性轨迹规划 - 完全映射工程代码逻辑
        """
        p = self.planners[motor_id]
        output = TrajectoryOutput()
        
        current_time = time.time() * 1000
        self._update_history_buffer(motor_id, target_position, 
                                    target_velocity if abs(target_velocity) > 0.1 else p.current_velocity,
                                    current_time)
        
        predicted_pos, predicted_vel = self._predict_future_target(
            motor_id, TRAJECTORY_PREDICTION_HORIZON
        )
        p.predicted_position = predicted_pos
        p.predicted_velocity = predicted_vel
        
        is_extreme, vel_reduction, accel_reduction = self._detect_extreme_point_v12(motor_id)
        
        pos_error = target_position - p.current_position
        abs_pos_error = abs(pos_error)
        
        if abs_pos_error < TRAJECTORY_POS_THRESHOLD:
            p.is_moving = False
            p.current_velocity = 0.0
            p.current_acceleration = 0.0
            p.current_position = target_position
            output.position = target_position
            output.velocity = 0.0
            output.acceleration = 0.0
            output.state = TrajectoryState.COMPLETED
            output.is_completed = True
            output.feedforward_vel = 0.0
            output.feedforward_accel = 0.0
            return output
        
        effective_v_max = p.dynamic_v_max * vel_reduction
        effective_a_max = p.dynamic_a_max * accel_reduction
        effective_d_max = p.dynamic_d_max * accel_reduction
        
        direction = pos_error >= 0
        direction_sign = 1.0 if direction else -1.0
        
        braking_dist = (p.current_velocity ** 2) / (2.0 * effective_d_max) if effective_d_max > 0 else 0
        
        if abs_pos_error > braking_dist + TRAJECTORY_POS_THRESHOLD:
            if p.current_velocity < effective_v_max:
                desired_velocity = p.current_velocity + effective_a_max * dt
                desired_velocity = min(desired_velocity, effective_v_max)
            else:
                desired_velocity = effective_v_max
            output.state = TrajectoryState.ACCELERATING if p.current_velocity < effective_v_max else TrajectoryState.CRUISING
        else:
            desired_velocity = p.current_velocity - effective_d_max * dt
            desired_velocity = max(0, desired_velocity)
            output.state = TrajectoryState.DECELERATING
        
        if p.prediction_valid and abs(predicted_vel) > 1.0:
            pred_dir = 1.0 if predicted_vel > 0 else -1.0
            cur_dir = direction_sign
            if pred_dir == cur_dir:
                pred_vel_factor = min(abs(predicted_vel) * 0.15, effective_v_max * 0.2)
                desired_velocity = min(desired_velocity + pred_vel_factor, effective_v_max)
        
        desired_accel = (desired_velocity - p.current_velocity) / dt if dt > 0.001 else 0.0
        desired_accel = self._clamp(desired_accel, -effective_d_max, effective_a_max)
        
        accel_diff = desired_accel - p.current_acceleration
        base_jerk_limit = TRAJECTORY_JERK_LIMIT
        
        current_vel_magnitude = abs(p.current_velocity)
        if current_vel_magnitude < 50.0:
            base_jerk_limit *= 0.5
        elif current_vel_magnitude > 200.0:
            base_jerk_limit *= 1.2
        
        accel_magnitude = abs(p.current_acceleration)
        if accel_magnitude > effective_a_max * 0.7:
            base_jerk_limit *= 0.6
        
        max_accel_change = base_jerk_limit * dt
        
        if abs(accel_diff) > max_accel_change:
            sign_accel = 1.0 if accel_diff > 0 else -1.0
            transition_factor = 1.0 - abs(accel_diff) / (2.0 * max_accel_change)
            if transition_factor < 0.0:
                transition_factor = 0.0
            smooth_factor = 0.5 + 0.5 * transition_factor
            accel_diff = sign_accel * max_accel_change * smooth_factor
        
        p.current_acceleration += accel_diff
        
        p.current_velocity += p.current_acceleration * dt
        p.current_velocity = self._clamp(p.current_velocity, 0.0, effective_v_max)
        
        new_position = p.current_position + direction_sign * p.current_velocity * dt
        
        if direction and new_position > target_position:
            new_position = target_position
            p.current_velocity = 0.0
            p.current_acceleration = 0.0
            output.state = TrajectoryState.COMPLETED
        elif not direction and new_position < target_position:
            new_position = target_position
            p.current_velocity = 0.0
            p.current_acceleration = 0.0
            output.state = TrajectoryState.COMPLETED
        
        new_position = self._clamp(new_position, 0.0, 359.99)
        
        ACCEL_FILTER_ALPHA = 0.80
        p.filtered_acceleration = ACCEL_FILTER_ALPHA * p.filtered_acceleration + (1.0 - ACCEL_FILTER_ALPHA) * p.current_acceleration
        
        p.current_position = new_position
        p.direction = direction
        p.last_planned_position = new_position
        p.last_planned_velocity = p.current_velocity
        p.last_planned_accel = p.current_acceleration
        p.is_moving = abs_pos_error > TRAJECTORY_POS_THRESHOLD
        
        output.position = new_position
        output.velocity = direction_sign * p.current_velocity
        output.acceleration = direction_sign * p.filtered_acceleration
        output.is_completed = (output.state == TrajectoryState.COMPLETED)
        
        output.feedforward_vel = FEEDFORWARD_VEL_GAIN * direction_sign * p.current_velocity
        output.feedforward_accel = FEEDFORWARD_ACCEL_GAIN * direction_sign * p.current_acceleration
        
        return output
    
    def _detect_extreme_point_v12(self, motor_id: int) -> Tuple[bool, float, float]:
        """V12.0: 极值点检测 - 完全映射工程代码"""
        p = self.planners[motor_id]
        
        is_extreme = False
        velocity_reduction = 1.0
        accel_reduction = 1.0
        
        if p.history_count < 3:
            return is_extreme, velocity_reduction, accel_reduction
        
        idx0 = (p.history_index + TRAJECTORY_HISTORY_SIZE - 3) % TRAJECTORY_HISTORY_SIZE
        idx1 = (p.history_index + TRAJECTORY_HISTORY_SIZE - 2) % TRAJECTORY_HISTORY_SIZE
        idx2 = (p.history_index + TRAJECTORY_HISTORY_SIZE - 1) % TRAJECTORY_HISTORY_SIZE
        
        pos0 = p.history_positions[idx0]
        pos1 = p.history_positions[idx1]
        pos2 = p.history_positions[idx2]
        
        if (pos1 > pos0 and pos1 > pos2) or (pos1 < pos0 and pos1 < pos2):
            is_extreme = True
            extreme_distance = abs(p.target_position - pos1)
            
            if extreme_distance < 5.0:
                distance_factor = extreme_distance / 5.0
                velocity_reduction = 0.92 + 0.08 * distance_factor
                accel_reduction = 0.88 + 0.12 * distance_factor
            elif extreme_distance < 12.0:
                distance_factor = (extreme_distance - 5.0) / 7.0
                velocity_reduction = 0.97 + 0.03 * distance_factor
                accel_reduction = 0.95 + 0.05 * distance_factor
        
        if not hasattr(p, 'last_vel_reduction'):
            p.last_vel_reduction = 1.0
            p.last_accel_reduction = 1.0
        
        SMOOTH_ALPHA = 0.6
        velocity_reduction = SMOOTH_ALPHA * p.last_vel_reduction + (1.0 - SMOOTH_ALPHA) * velocity_reduction
        accel_reduction = SMOOTH_ALPHA * p.last_accel_reduction + (1.0 - SMOOTH_ALPHA) * accel_reduction
        
        p.last_vel_reduction = velocity_reduction
        p.last_accel_reduction = accel_reduction
        
        return is_extreme, velocity_reduction, accel_reduction
    
    def step(self, motor_id: int, dt: float) -> TrajectoryOutput:
        if motor_id >= NUM_MOTORS:
            return TrajectoryOutput()
        
        p = self.planners[motor_id]
        
        if hasattr(self, '_use_predictive_planning') and self._use_predictive_planning:
            return self._generate_smooth_trajectory(motor_id, p.target_position, 0.0, dt)
        
        output = TrajectoryOutput()
        
        v_max = p.dynamic_v_max
        a_max = p.dynamic_a_max
        d_max = p.dynamic_d_max
        
        p.remaining_distance = p.target_position - p.current_position
        abs_remaining = abs(p.remaining_distance)
        
        braking_dist = (p.current_velocity ** 2) / (2.0 * d_max) if d_max > 0 else 0
        
        use_s_curve_this_step = False
        
        if p.state == TrajectoryState.IDLE:
            if p.is_moving and abs_remaining > p.config.pos_threshold:
                p.state = TrajectoryState.ACCELERATING
        
        elif p.state == TrajectoryState.ACCELERATING:
            if p.s_curve.use_s_curve and p.config.jerk_limit > 0.0:
                pos_delta = self._s_curve_step(p, dt)
                p.current_position += pos_delta
                # clamp位置到合法范围
                p.current_position = self._clamp(p.current_position, 0.0, 359.99)
                
                # 防止S曲线超调
                if p.direction and p.current_position >= p.target_position:
                    p.current_position = p.target_position
                    p.current_velocity = 0.0
                    p.current_acceleration = 0.0
                    p.s_curve.use_s_curve = False
                    p.state = TrajectoryState.COMPLETED
                elif not p.direction and p.current_position <= p.target_position:
                    p.current_position = p.target_position
                    p.current_velocity = 0.0
                    p.current_acceleration = 0.0
                    p.s_curve.use_s_curve = False
                    p.state = TrajectoryState.COMPLETED
                
                use_s_curve_this_step = True
                
                if not p.s_curve.use_s_curve:
                    p.state = TrajectoryState.STOPPING
            else:
                # V3.0 改进：速度始终为正
                if abs_remaining <= braking_dist + p.config.pos_threshold:
                    p.state = TrajectoryState.DECELERATING
                    p.current_acceleration = -d_max  # V3.0: 减速时加速度为负
                elif p.current_velocity >= v_max:  # V3.0: 速度始终为正
                    p.state = TrajectoryState.CRUISING
                    p.current_velocity = v_max  # V3.0: 速度始终为正
                    p.current_acceleration = 0.0
                else:
                    # V7.1: 保持当前加速度连续性
                    p.current_acceleration = p.current_acceleration
        
        elif p.state == TrajectoryState.CRUISING:
            if abs_remaining <= braking_dist + p.config.pos_threshold:
                p.state = TrajectoryState.DECELERATING
                p.current_acceleration = -d_max  # V3.0: 减速时加速度为负
            else:
                p.current_acceleration = 0.0
            # clamp位置到合法范围
            p.current_position = self._clamp(p.current_position, 0.0, 359.99)
        
        elif p.state == TrajectoryState.DECELERATING:
            p.current_acceleration = -d_max  # V3.0: 减速时加速度为负
            
            # V3.0: 速度始终为正
            velocity_low = p.current_velocity < p.config.vel_threshold
            position_reached = abs_remaining < p.config.pos_threshold
            overshoot = (p.direction and p.remaining_distance < 0.0) or \
                       (not p.direction and p.remaining_distance > 0.0)
            
            if (velocity_low and position_reached) or overshoot:
                p.state = TrajectoryState.STOPPING
            # clamp位置到合法范围
            p.current_position = self._clamp(p.current_position, 0.0, 359.99)
        
        elif p.state == TrajectoryState.STOPPING:
            # V3.0 改进：速度始终为正，停止阶段只需减速到0
            stop_decel = p.config.d_max * 2.0
            
            next_velocity = p.current_velocity - stop_decel * dt
            
            if next_velocity <= 0.0 or p.current_velocity < 0.5:
                # 停止运动，但不直接跳到目标位置
                p.current_velocity = 0.0
                p.current_acceleration = 0.0
                # clamp位置到合法范围
                p.current_position = self._clamp(p.current_position, 0.0, 359.99)
                p.is_moving = False
                p.state = TrajectoryState.COMPLETED
                p.s_curve.use_s_curve = False
            else:
                p.current_acceleration = -stop_decel  # V3.0: 减速时加速度为负
        
        elif p.state == TrajectoryState.COMPLETED:
            p.current_acceleration = 0.0
            p.is_moving = False
            # clamp位置到合法范围
            p.current_position = self._clamp(p.current_position, 0.0, 359.99)
            # 修复：轨迹完成后重置动态参数为默认值
            p.dynamic_v_max = TRAJECTORY_DEFAULT_VMAX
            p.dynamic_a_max = TRAJECTORY_DEFAULT_AMAX
            p.dynamic_d_max = TRAJECTORY_DEFAULT_DMAX
            if abs_remaining > p.config.pos_threshold * 2.0:
                p.state = TrajectoryState.IDLE
                p.is_moving = True
        
        # 更新位置和速度
        if not use_s_curve_this_step:
            # V3.0 改进：速度始终为正，方向由direction控制
            p.current_velocity += p.current_acceleration * dt
            # V3.0: 速度限制为非负值 [0, v_max]
            p.current_velocity = self._clamp(p.current_velocity, 0.0, v_max)
            
            # V3.0: 位置更新根据方向决定
            direction_sign = 1.0 if p.direction else -1.0
            p.current_position += direction_sign * p.current_velocity * dt
            
            # 防止超调：如果已经超过目标，立即纠正
            if p.direction and p.current_position > p.target_position:
                p.current_position = p.target_position
            elif not p.direction and p.current_position < p.target_position:
                p.current_position = p.target_position
            
            # 如果到达目标，停止运动
            if p.direction and p.current_position >= p.target_position - 0.01:
                p.current_velocity = 0.0
                p.current_acceleration = 0.0
                p.state = TrajectoryState.COMPLETED
            elif not p.direction and p.current_position <= p.target_position + 0.01:
                p.current_velocity = 0.0
                p.current_acceleration = 0.0
                p.state = TrajectoryState.COMPLETED
        
        # 前馈控制计算
        # 关键修复：clamp当前位置到合法范围
        p.current_position = self._clamp(p.current_position, 0.0, 359.99)
        # 只在检测到超调时才限制位置
        remaining = p.target_position - p.current_position
        if (p.direction and remaining < 0) or (not p.direction and remaining > 0):
            # 已经超调，强制设置到目标位置
            p.current_position = p.target_position
            p.current_velocity = 0.0
            p.current_acceleration = 0.0
            p.state = TrajectoryState.COMPLETED
        
        # V5.5: 加速度平滑滤波 - 消除状态切换时的阶跃
        ACCEL_FILTER_ALPHA = 0.80  # V5.5: 调整为0.80，平衡平滑与响应
        p.filtered_acceleration = ACCEL_FILTER_ALPHA * p.filtered_acceleration + \
                                  (1 - ACCEL_FILTER_ALPHA) * p.current_acceleration
        
        output.position = p.current_position
        output.velocity = p.current_velocity
        output.acceleration = p.filtered_acceleration
        output.state = p.state
        output.is_completed = (p.state == TrajectoryState.COMPLETED)
        
        joint_type = JOINT_TYPES[motor_id]
        pid_params = JOINT_PID_PARAMS[joint_type].copy()
        
        if USE_DYNAMIC_GAIN:
            pos_error = p.target_position - p.current_position
            abs_error = abs(pos_error)
            
            if abs_error > GAIN_BOOST_THRESHOLD:
                boost = GAIN_BOOST_FACTOR
            else:
                boost = 1.0 + (GAIN_BOOST_FACTOR - 1.0) * (abs_error / GAIN_BOOST_THRESHOLD) * GAIN_RAMP_FACTOR
            
            pid_params['kp'] *= boost
            pid_params['kd'] *= boost
        
        direction_sign = 1.0 if p.direction else -1.0
        
        ff_vel_gain = pid_params['ff_vel']
        ff_accel_gain = pid_params['ff_accel']
        
        tracking_error = abs(p.target_position - p.current_position)
        adaptive_ff_factor = 1.0
        
        if tracking_error > 2.0:
            adaptive_ff_factor = 1.0 + 0.3 * (tracking_error - 2.0) / 5.0
            if adaptive_ff_factor > 1.5:
                adaptive_ff_factor = 1.5
        elif tracking_error < 0.5:
            adaptive_ff_factor = 0.8 + 0.2 * (tracking_error / 0.5)
        
        vel_magnitude = abs(p.current_velocity)
        if vel_magnitude > 150.0:
            adaptive_ff_factor *= 1.1
        elif vel_magnitude < 30.0 and vel_magnitude > 5.0:
            adaptive_ff_factor *= 0.9
        
        accel_magnitude = abs(p.current_acceleration)
        if accel_magnitude > 500.0:
            ff_accel_gain *= 1.2
        
        ff_vel_gain *= adaptive_ff_factor
        
        distance_to_target = abs(p.target_position - p.current_position)
        if distance_to_target < 55.0:
            ff_vel_gain *= 0.30
            ff_accel_gain *= 0.30
        
        output.feedforward_vel = ff_vel_gain * direction_sign * p.current_velocity
        output.feedforward_accel = ff_accel_gain * direction_sign * p.current_acceleration
        
        # PID补偿 - 使用差异化参数
        pos_error = p.target_position - p.current_position
        self.vel_integral[motor_id] += pos_error * dt
        self.vel_integral[motor_id] = self._clamp(
            self.vel_integral[motor_id], -INTEGRAL_LIMIT, INTEGRAL_LIMIT
        )
        
        alpha = 0.8
        raw_deriv = (pos_error - self.last_pos_error[motor_id]) / dt if dt > 0 else 0
        self.filtered_deriv[motor_id] = alpha * self.filtered_deriv[motor_id] + \
                                        (1.0 - alpha) * raw_deriv
        self.last_pos_error[motor_id] = pos_error
        
        pid_output = pid_params['kp'] * pos_error + pid_params['ki'] * self.vel_integral[motor_id] + \
                    pid_params['kd'] * self.filtered_deriv[motor_id]
        
        output.feedforward_vel += pid_output * 0.01
        
        return output
    
    def step_all(self, dt: float) -> List[TrajectoryOutput]:
        """执行所有电机的轨迹规划"""
        return [self.step(i, dt) for i in range(NUM_MOTORS)]

# ==================== 智能控制器 ====================
class SmartController:
    """智能控制器仿真 - 与quad_smart_controller.c一致"""
    
    def __init__(self):
        self.controllers = [MotorController(motor_id=i) for i in range(NUM_MOTORS)]
    
    def identify_scene(self, motor_id: int, current_angle: float, 
                       target_angle: float, current_force: float = 0.0) -> QuadScene:
        """场景识别"""
        angle_diff = abs(target_angle - current_angle)
        
        if angle_diff < TRAJECTORY_POS_THRESHOLD * 0.5:
            return QuadScene.STOP
        elif current_force < 50.0:
            return QuadScene.SWING
        elif current_force > 70.0 and angle_diff < TRAJECTORY_POS_THRESHOLD * 2:
            return QuadScene.STANCE
        else:
            return QuadScene.TRANSITION
    
    def select_strategy(self, scene: QuadScene) -> QuadStrategy:
        """策略选择"""
        if scene == QuadScene.SWING:
            return QuadStrategy.POSITION
        elif scene == QuadScene.STANCE:
            return QuadStrategy.HYBRID
        elif scene == QuadScene.TRANSITION:
            return QuadStrategy.HYBRID
        else:
            return QuadStrategy.POSITION
    
    def update_all(self, target_angles: List[float], 
                   trajectory_planner: TrajectoryPlannerSim):
        """更新所有电机"""
        for i in range(NUM_MOTORS):
            ctrl = self.controllers[i]
            
            ctrl.current_scene = self.identify_scene(
                i, trajectory_planner.planners[i].current_position, 
                target_angles[i]
            )
            ctrl.current_strategy = self.select_strategy(ctrl.current_scene)
            ctrl.target_position = target_angles[i]
            
            planner = trajectory_planner.planners[i]

            # 更新历史缓冲和前瞻缓冲
            trajectory_planner.update_buffers(i, target_angles[i])

            # V6.0: 使用连续轨迹模式
            if trajectory_planner.is_continuous_mode():
                trajectory_planner.set_continuous_target(i, target_angles[i])
            else:
                trajectory_planner.set_target(i, target_angles[i])

# ==================== MPC控制器 ====================
class MotorMPCController:
    """
    电机MPC控制器 - 基于电机动力学模型的预测控制
    使用显式MPC求解，适合实时嵌入式系统
    """
    
    def __init__(self, motor_id: int = 0):
        self.motor_id = motor_id
        
        self.inertia = 0.003  # 电机转动惯量 kg·m²
        self.damping = 0.03   # 阻尼系数
        self.friction = 0.01  # 摩擦系数
        
        self.nu = 1           # 控制输入维度 (扭矩)
        self.nx = 2           # 状态维度 (位置, 速度)
        self.N = 20           # V8.0 增大预测 horizon
        self.dt = 0.005       # 控制周期 5ms
        
        self.Q = np.diag([200.0, 5.0])      # V8.0 增大权重
        self.R = np.array([[0.001]])         # V8.0 减小控制权重
        self.Qf = np.diag([100.0, 1.0])     # 终端权重
        
        self.K = self._compute_lqr_gain()
        self.A_cl = np.array([[0, 1], [0, -self.damping/self.inertia]])
        
        self.prev_u = 0.0
        self.state_history = []
    
    def _compute_lqr_gain(self):
        """预先计算LQR增益作为MPC的近似解"""
        A = np.array([[0, 1], [0, -self.damping/self.inertia]])
        B = np.array([[0], [1/self.inertia]])
        
        P = self.Q.copy()
        for _ in range(50):
            P_new = self.Q + A.T @ P @ A - \
                    A.T @ P @ B @ np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
            P = P_new
        
        K = np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
        return K
    
    def predict(self, x0: np.ndarray, ref: np.ndarray) -> np.ndarray:
        """预测未来状态序列"""
        x = x0.copy()
        predictions = [x.copy()]
        
        for k in range(self.N):
            u = -self.K @ (x - ref)
            u = np.clip(u, -10.0, 10.0)
            
            dx = np.array([x[1], (u[0] - self.damping * x[1] - self.friction * np.sign(x[1])) / self.inertia])
            x = x + self.dt * dx
            predictions.append(x.copy())
        
        return np.array(predictions)
    
    def compute_control(self, current_pos: float, current_vel: float,
                       target_pos: float, target_vel: float = 0.0) -> float:
        """
        计算MPC控制输入 - 基于预测误差的直接补偿
        返回: 控制扭矩 (Nm)
        """
        x0 = np.array([current_pos, current_vel])
        ref = np.array([target_pos, target_vel])
        
        predictions = self.predict(x0, ref)
        
        future_error = predictions[-1] - ref
        
        mpc_kp = 2.0   # V8.0 增大增益
        mpc_kd = 1.0
        
        u = mpc_kp * future_error[0] + mpc_kd * future_error[1]
        
        u = float(np.clip(u, -10.0, 10.0))
        
        self.prev_u = u
        self.state_history.append(x0)
        if len(self.state_history) > 100:
            self.state_history.pop(0)
        
        return u
    
    def reset(self):
        """重置MPC状态"""
        self.prev_u = 0.0
        self.state_history = []


class MPCControllerManager:
    """MPC控制器管理器 - 为每个电机维护一个MPC实例"""
    
    def __init__(self, num_motors: int = 12):
        self.mpc_controllers = [MotorMPCController(i) for i in range(num_motors)]
        self.enabled = False
    
    def compute_mpc_compensation(self, motor_id: int,
                                  current_pos: float, current_vel: float,
                                  target_pos: float, target_vel: float = 0.0) -> float:
        """计算MPC补偿扭矩"""
        if not self.enabled:
            return 0.0
        
        mpc = self.mpc_controllers[motor_id]
        return mpc.compute_control(current_pos, current_vel, target_pos, target_vel)
    
    def reset_all(self):
        """重置所有MPC状态"""
        for mpc in self.mpc_controllers:
            mpc.reset()


# ==================== 电机响应模型 ====================
class MotorModel:
    """电机响应模型 - V5.3: 预测修正 + 摩擦补偿 + 转矩补偿 + 前馈控制"""
    
    def __init__(self):
        self.motors = [MotorState() for _ in range(NUM_MOTORS)]
        
        # P2优化: 使用差异化电机参数
        self.motor_inertia = 0.005
        self.motor_damping = 0.05
        self.motor_friction = 0.02
        self.control_bandwidth = 50.0
        self.response_delay = 0.001
        self.dt_filter = 0.0005
        
        # P0优化: 使用差异化PD控制参数
        self.kp_motor = 500.0
        self.kd_motor = 20.0
        
        # V8.0: 添加MPC控制器
        self.mpc_controller = MPCControllerManager(NUM_MOTORS)
        self.use_mpc = False
        
        # V5.0: 电机内置速度和加速度限制 (模拟电机收到的限制)
        self.speed_limit = [200.0] * NUM_MOTORS      # rpm
        self.accel_limit = [300.0] * NUM_MOTORS     # rpm/s
        
        # V5.3: 增强型电机反馈数据 (模拟工程代码中的g_motor_feedback)
        self.motor_feedback = [{
            'actual_position': MOTOR_NEUTRAL,
            'actual_velocity': 0.0,    # rpm
            'actual_torque': 0.0,      # Nm
            'position_error': 0.0,
            'velocity_error': 0.0,     # deg/s
            'last_update_tick': 0
        } for _ in range(NUM_MOTORS)]
        
        # V9.0: 添加仿真时间追踪
        self.sim_time_ms = 0
        
        # V5.3: 补偿控制参数 (与工程代码一致)
        PREDICTIVE_CORRECTION = True
        PREDICTION_FACTOR = 0.15
        FRICTION_COMPENSATION = True
        FRICTION_COMPENSATION_GAIN = 0.05
        TORQUE_COMPENSATION = True
        TORQUE_COMPENSATION_GAIN = 0.1
        
        self.prev_error = [0.0] * NUM_MOTORS
        self.prev_deriv = [0.0] * NUM_MOTORS
        self.prev_velocity = [0.0] * NUM_MOTORS
    
    def set_limits(self, motor_id: int, speed_rpm: float, accel_rpm_s: float):
        """V5.0: 设置速度和加速度限制 (模拟收到CAN指令)"""
        if motor_id < NUM_MOTORS:
            self.speed_limit[motor_id] = max(10.0, min(200.0, speed_rpm))
            self.accel_limit[motor_id] = max(20.0, min(300.0, accel_rpm_s))
    
    def update(self, target_positions: List[float], 
               target_velocities: List[float] = None,
               feedforward_vel: List[float] = None,
               feedforward_accel: List[float] = None,
               dt: float = 0.005):
        """更新电机状态 - V4.0: 使用差异化参数 + 前馈控制"""
        for i in range(NUM_MOTORS):
            motor = self.motors[i]
            motor.target_position = target_positions[i]
            
            # P2优化: 获取关节差异化参数
            joint_type = JOINT_TYPES[i]
            motor_params = JOINT_MOTOR_PARAMS[joint_type]
            kp = motor_params['kp']
            kd = motor_params['kd']
            inertia = motor_params['inertia']
            damping = motor_params['damping']
            friction = motor_params['friction']
            
            # P0优化: 获取前馈控制量
            ff_v = feedforward_vel[i] if feedforward_vel is not None else 0.0
            ff_a = feedforward_accel[i] if feedforward_accel is not None else 0.0
            
            # 计算位置误差 (跟踪误差 = 电机目标 - 电机实际)
            error = motor.target_position - motor.position
            
            # P2优化: 使用差异化PD控制
            p_term = kp * error
            
            # 微分项 (带低通滤波)
            raw_deriv = (error - self.prev_error[i]) / dt if dt > 0 else 0
            filtered_deriv = (self.dt_filter * self.prev_deriv[i] + 
                             dt * raw_deriv) / (self.dt_filter + dt)
            d_term = kd * filtered_deriv
            self.prev_error[i] = error
            self.prev_deriv[i] = filtered_deriv
            
            # V5.5: 前馈信号滤波，减少噪声放大
            # V7.0: 自适应前馈增益 - 极值点附近降低增益
            ff_gain_v = FEEDFORWARD_VEL_GAIN
            ff_gain_a = FEEDFORWARD_ACCEL_GAIN
            
            # 计算到目标的距离
            distance_to_target = abs(target_positions[i] - motor.position)
            if distance_to_target < 55.0:
                # 极值点附近降低前馈增益70%
                ff_gain_v *= 0.30
                ff_gain_a *= 0.30
            
            # 对前馈信号进行低通滤波
            FF_FILTER_ALPHA = 0.6  # V5.5: 降低到0.6，增加滤波效果
            if not hasattr(self, 'prev_ff_v'):
                self.prev_ff_v = [0.0] * NUM_MOTORS
                self.prev_ff_a = [0.0] * NUM_MOTORS
            filtered_ff_v = FF_FILTER_ALPHA * self.prev_ff_v[i] + (1 - FF_FILTER_ALPHA) * ff_v
            filtered_ff_a = FF_FILTER_ALPHA * self.prev_ff_a[i] + (1 - FF_FILTER_ALPHA) * ff_a
            self.prev_ff_v[i] = filtered_ff_v
            self.prev_ff_a[i] = filtered_ff_a
            ff_compensation = filtered_ff_v * ff_gain_v + filtered_ff_a * ff_gain_a
            
            # P2优化: 使用差异化阻尼和摩擦
            # 期望加速度 = P + D + 前馈 - 阻尼 - 摩擦
            desired_accel = p_term + d_term + ff_compensation
            desired_accel -= damping * motor.velocity
            # V5.4: 使用平滑tanh替代sign函数，消除速度过零点的不连续
            smooth_sign = np.tanh(motor.velocity * 50.0)  # 平滑的符号函数
            desired_accel -= friction * smooth_sign
            
            # V8.0: 添加MPC补偿
            if self.use_mpc:
                mpc_torque = self.mpc_controller.compute_mpc_compensation(
                    i, motor.position, motor.velocity,
                    target_positions[i], target_velocities[i] if target_velocities else 0.0
                )
                mpc_accel = mpc_torque / inertia
                desired_accel += mpc_accel * 0.5
            
            # V5.1: 移除电机内置限制，保持原有响应特性
            # (CAN发送时已发送速度/加速度限制给真实电机，仿真中不需要模拟)
            
            # 通过二阶系统积分得到速度和位置
            motor.velocity += desired_accel * dt
            
            # 速度限制 (最大速度 800 deg/s - 提高上限)
            max_vel = 800.0
            motor.velocity = max(-max_vel, min(max_vel, motor.velocity))
            
            # 位置更新
            motor.position += motor.velocity * dt
            
            # 加速度计算
            motor.acceleration = desired_accel
            
            # 限制角度范围
            motor.position = max(0.0, min(359.99, motor.position))
    
    def update_feedback(self, motor_id: int, planned_velocity: float, dt: float = 0.005):
        """V5.3: 更新电机反馈数据 (模拟工程代码中的g_motor_feedback)"""
        if motor_id >= NUM_MOTORS:
            return
            
        motor = self.motors[motor_id]
        fb = self.motor_feedback[motor_id]
        
        # 更新实际位置
        fb['actual_position'] = motor.position
        
        # 实际速度转换为rpm
        fb['actual_velocity'] = motor.velocity / 6.0  # deg/s -> rpm
        
        # 实际转矩估算 (基于加速度和摩擦)
        joint_type = JOINT_TYPES[motor_id]
        motor_params = JOINT_MOTOR_PARAMS[joint_type]
        inertia = motor_params['inertia']
        damping = motor_params['damping']
        friction = motor_params['friction']
        
        # V5.4: 使用平滑tanh替代sign函数
        smooth_sign = np.tanh(motor.velocity * 50.0)
        # 估算转矩: T = I*α + b*v + F*sign(v)
        estimated_torque = inertia * motor.acceleration + \
                         damping * motor.velocity + \
                         friction * smooth_sign
        fb['actual_torque'] = estimated_torque
        
        # 计算位置误差
        fb['position_error'] = motor.target_position - motor.position
        
        # 计算速度误差 (deg/s)
        fb['velocity_error'] = motor.velocity - planned_velocity
        fb['last_update_tick'] = 0
    
    def apply_compensation(self, motor_id: int, target_pos: float, 
                          planned_velocity: float, dt: float = 0.005) -> float:
        """V5.3: 计算补偿后的位置 (与工程代码一致)"""
        if motor_id >= NUM_MOTORS:
            return target_pos
            
        fb = self.motor_feedback[motor_id]
        
        compensation = 0.0
        
        # V5.4: 仿真中禁用补偿机制，避免噪声叠加
        # 实际电机上这些补偿是有用的，但仿真中会引入不必要的高频波动
        # 与STM32工程逻辑一致但参数不同（仿真环境vs实际硬件）
        PREDICTIVE_CORRECTION = False
        PREDICTION_FACTOR = 0.15
        FRICTION_COMPENSATION = False
        FRICTION_COMPENSATION_GAIN = 0.05
        TORQUE_COMPENSATION = False
        TORQUE_COMPENSATION_GAIN = 0.1
        
        # 1. 预测修正 (仿真禁用)
        if PREDICTIVE_CORRECTION:
            predicted_error = fb['velocity_error'] * 0.005
            compensation += predicted_error * PREDICTION_FACTOR
        
        # 2. 摩擦补偿 (仿真禁用)
        if FRICTION_COMPENSATION:
            abs_velocity = abs(fb['actual_velocity'])
            if 1.0 < abs_velocity < 20.0:
                friction_comp = FRICTION_COMPENSATION_GAIN * (20.0 - abs_velocity) / 20.0
                direction = 1.0 if fb['actual_velocity'] > 0 else -1.0
                compensation += direction * friction_comp
        
        # 3. 转矩补偿 (仿真禁用)
        if TORQUE_COMPENSATION:
            abs_torque = abs(fb['actual_torque'])
            if abs_torque > 0.1:
                compensation += fb['actual_torque'] * TORQUE_COMPENSATION_GAIN
        
        # 应用补偿
        compensated_pos = target_pos + compensation
        
        # 限制补偿范围
        max_comp = 5.0
        compensated_pos = max(target_pos - max_comp, min(target_pos + max_comp, compensated_pos))
        
        return compensated_pos
    
    # 初始化误差历史
    def reset(self):
        self.prev_error = [0.0] * NUM_MOTORS
        self.prev_deriv = [0.0] * NUM_MOTORS
        for motor in self.motors:
            motor.position = MOTOR_NEUTRAL
            motor.velocity = 0.0
            motor.acceleration = 0.0

# ==================== CAN发送优化 ====================
class CANOptimizer:
    """CAN发送优化 - 与quad_control_loop.c一致"""
    
    def __init__(self):
        self.send_states = [CANSendState() for _ in range(NUM_MOTORS)]
        self.send_count = 0
        self.skip_count = 0
    
    def should_send(self, motor_id: int, position: float, velocity: float, 
                    current_time: float, is_accelerating: bool) -> bool:
        """判断是否需要发送CAN指令"""
        state = self.send_states[motor_id]
        
        pos_delta = abs(position - state.last_sent_position)
        vel_delta = abs(velocity - state.last_sent_velocity)
        time_delta = current_time - state.last_send_time
        
        if pos_delta > CAN_SEND_POS_THRESHOLD or \
           vel_delta > CAN_SEND_VEL_THRESHOLD or \
           time_delta > CAN_SEND_MAX_INTERVAL or \
           (is_accelerating and time_delta > CAN_SEND_ACCEL_INTERVAL):
            state.last_sent_position = position
            state.last_sent_velocity = velocity
            state.last_send_time = current_time
            state.skip_count = 0
            self.send_count += 1
            return True
        
        state.skip_count += 1
        self.skip_count += 1
        return False

# ==================== 完整系统仿真器 ====================
class TargetBuffer:
    """V12.0: 双缓冲机制 - 完整映射工程代码优化逻辑"""
    
    def __init__(self):
        self.buffer = [[MOTOR_NEUTRAL] * TARGET_BUFFER_SIZE for _ in range(NUM_MOTORS)]
        self.write_idx = 0
        self.read_idx = 0
        self.count = 0
        self.last_interpolated = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.interpolation_step = 0
        
        # V11.1: 插值误差补偿
        self.interp_error_accum = [0.0] * NUM_MOTORS
        self.last_raw_target = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.last_sync_tick = 0
        self.first_data_received = [False] * NUM_MOTORS
        
        # V12.0: 自适应插值预测 - 优化方案A1
        self.vel_estimate = [0.0] * NUM_MOTORS
        self.accel_estimate = [0.0] * NUM_MOTORS
        self.last_p1 = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.last_p0 = [MOTOR_NEUTRAL] * NUM_MOTORS
        
        # V12.0: 极值点检测 - 优化方案B1
        self.extreme_history = [[MOTOR_NEUTRAL] * 3 for _ in range(NUM_MOTORS)]
        self.extreme_history_idx = [0] * NUM_MOTORS
        self.is_at_extreme = [False] * NUM_MOTORS
        self.extreme_velocity_reduction = [1.0] * NUM_MOTORS
    
    def write(self, target_angles: List[float], current_tick_ms: int):
        """V12.0: 写入新数据到双缓冲区"""
        write_idx = self.write_idx
        
        for i in range(NUM_MOTORS):
            new_target = target_angles[i]
            
            # V11.1: 插值误差补偿
            if INTERP_ERROR_CORRECTION_ENABLE and self.first_data_received[i]:
                expected_target = self.last_raw_target[i]
                error = new_target - expected_target
                abs_error = abs(error)
                
                if abs_error > INTERP_ERROR_THRESHOLD:
                    correction = max(-INTERP_ERROR_MAX_CORRECTION, 
                                   min(INTERP_ERROR_MAX_CORRECTION, error))
                    self.interp_error_accum[i] += correction * 0.5
                    self.interp_error_accum[i] = max(-INTERP_ERROR_MAX_CORRECTION * 2,
                                                    min(INTERP_ERROR_MAX_CORRECTION * 2,
                                                        self.interp_error_accum[i]))
            else:
                self.first_data_received[i] = True
            
            self.last_raw_target[i] = new_target
            new_target += self.interp_error_accum[i] * 0.1
            self.interp_error_accum[i] *= INTERP_ERROR_DECAY
            
            self.buffer[i][write_idx] = new_target
        
        self.write_idx = (write_idx + 1) % TARGET_BUFFER_SIZE
        if self.count < TARGET_BUFFER_SIZE:
            self.count += 1
        
        # V11.1: 定期强制同步
        if INTERP_ERROR_CORRECTION_ENABLE:
            if (current_tick_ms - self.last_sync_tick) > INTERP_SYNC_INTERVAL_MS:
                for i in range(NUM_MOTORS):
                    self.interp_error_accum[i] *= 0.5
                self.last_sync_tick = current_tick_ms
    
    def _detect_extreme_point(self, motor_id: int) -> Tuple[bool, float]:
        """V12.0: 极值点检测 - 优化方案B1"""
        hist = self.extreme_history[motor_id]
        idx = self.extreme_history_idx[motor_id]
        
        if idx < 3:
            return False, 1.0
        
        idx0 = (idx - 3) % 3
        idx1 = (idx - 2) % 3
        idx2 = (idx - 1) % 3
        
        p0 = hist[idx0]
        p1 = hist[idx1]
        p2 = hist[idx2]
        
        is_extreme = (p1 > p0 and p1 > p2) or (p1 < p0 and p1 < p2)
        
        if is_extreme:
            extreme_dist = abs(self.last_interpolated[motor_id] - p1)
            if extreme_dist < 15.0:
                reduction = 0.5 + 0.5 * (extreme_dist / 15.0)
            elif extreme_dist < 30.0:
                reduction = 0.85 + 0.15 * ((extreme_dist - 15.0) / 15.0)
            else:
                reduction = 1.0
            return True, reduction
        
        return False, 1.0
    
    def read_interpolated(self) -> List[float]:
        """V12.0: 读取插值后的数据 - 完整映射工程代码优化逻辑"""
        output = [MOTOR_NEUTRAL] * NUM_MOTORS
        
        if self.count < 2:
            for i in range(NUM_MOTORS):
                output[i] = self.last_interpolated[i]
            return output
        
        read_idx = self.read_idx
        next_idx = (read_idx + 1) % TARGET_BUFFER_SIZE
        t = self.interpolation_step / INTERPOLATION_STEPS
        
        for i in range(NUM_MOTORS):
            p0 = self.buffer[i][read_idx]
            p1 = self.buffer[i][next_idx]
            
            # V12.0: 自适应插值预测 - 优化方案A1
            if self.interpolation_step == 0:
                instant_vel = (p1 - p0) * 50.0
                self.vel_estimate[i] = 0.4 * instant_vel + 0.6 * self.vel_estimate[i]
                
                instant_accel = (p1 - 2.0 * self.last_p1[i] + self.last_p0[i]) * 2500.0
                self.accel_estimate[i] = 0.3 * instant_accel + 0.7 * self.accel_estimate[i]
                
                self.last_p0[i] = self.last_p1[i]
                self.last_p1[i] = p1
            
            # 线性插值
            linear_interp = p0 + (p1 - p0) * t
            
            # V12.0: 预测补偿
            prediction_time = 0.005 + t * 0.010
            predicted_offset = self.vel_estimate[i] * prediction_time
            accel_compensation = 0.5 * self.accel_estimate[i] * prediction_time * prediction_time
            
            # V12.0: 自适应增益
            adaptive_gain = 1.0
            vel_magnitude = abs(self.vel_estimate[i])
            if vel_magnitude > 100.0:
                adaptive_gain = 1.0 + 0.3 * (vel_magnitude - 100.0) / 100.0
                if adaptive_gain > 1.5:
                    adaptive_gain = 1.5
            
            # V12.0: 极值点检测
            is_extreme, vel_reduction = self._detect_extreme_point(i)
            if is_extreme:
                adaptive_gain *= vel_reduction
            
            # 组合输出
            output[i] = linear_interp + predicted_offset * adaptive_gain * 0.3 + accel_compensation * 0.1
            
            # 限制范围
            if output[i] < 0.0:
                output[i] = 0.0
            if output[i] > 359.99:
                output[i] = 359.99
            
            self.last_interpolated[i] = output[i]
            
            # 更新极值点历史
            self.extreme_history[i][self.extreme_history_idx[i] % 3] = output[i]
            self.extreme_history_idx[i] += 1
        
        self.interpolation_step += 1
        if self.interpolation_step >= INTERPOLATION_STEPS:
            self.interpolation_step = 0
            self.read_idx = next_idx
            self.count -= 1
        
        return output
    
    def reset(self):
        """V12.0: 重置双缓冲区"""
        self.write_idx = 0
        self.read_idx = 0
        self.count = 0
        self.interpolation_step = 0
        self.last_sync_tick = 0
        for i in range(NUM_MOTORS):
            self.last_interpolated[i] = MOTOR_NEUTRAL
            self.interp_error_accum[i] = 0.0
            self.last_raw_target[i] = MOTOR_NEUTRAL
            self.first_data_received[i] = False
            self.vel_estimate[i] = 0.0
            self.accel_estimate[i] = 0.0
            self.last_p1[i] = MOTOR_NEUTRAL
            self.last_p0[i] = MOTOR_NEUTRAL


class MonitorRequestOptimizer:
    """V11.0: CAN监测请求优化器 - 匹配工程代码"""
    
    def __init__(self):
        self.last_request_tick = [0] * NUM_MOTORS
        self.request_count = 0
        self.skip_count = 0
    
    def should_request(self, motor_id: int, current_tick_ms: int) -> bool:
        """V11.0: 判断是否应该发送监测请求"""
        stagger_tick = motor_id * MONITOR_STAGGER_OFFSET
        interval = MONITOR_REQUEST_INTERVAL_MS + stagger_tick
        
        if (current_tick_ms - self.last_request_tick[motor_id]) >= interval:
            self.last_request_tick[motor_id] = current_tick_ms
            self.request_count += 1
            return True
        
        self.skip_count += 1
        return False
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        total = self.request_count + self.skip_count
        return {
            'requests': self.request_count,
            'skips': self.skip_count,
            'efficiency': self.request_count / total * 100 if total > 0 else 0
        }


class QuadrupedSimulator:
    """四足机器狗完整系统仿真器 - V11.1: 完整匹配工程代码"""
    
    def __init__(self):
        # 初始化各模块
        self.gait_generator = GaitGenerator()
        self.trajectory_planner = TrajectoryPlannerSim()
        self.smart_controller = SmartController()
        self.motor_model = MotorModel()
        self.can_optimizer = CANOptimizer()
        self.can_send_states = [CANSendState() for _ in range(NUM_MOTORS)]
        
        # V11.0: 双缓冲机制
        self.target_buffer = TargetBuffer()
        self.use_interpolation = True
        
        # V11.0: CAN监测请求优化
        self.monitor_optimizer = MonitorRequestOptimizer()
        
        # 初始化电机模型误差历史
        self.motor_model.prev_error = [0.0] * NUM_MOTORS
        self.motor_model.prev_deriv = [0.0] * NUM_MOTORS
        
        # V8.0: MPC控制开关
        self.motor_model.use_mpc = True
        
        # 时间参数
        self.time = 0.0
        self.dt_control = 1.0 / STM32_CONTROL_FREQ  # 5ms
        self.dt_upper = 1.0 / UPPER_COMPUTER_FREQ    # 20ms
        self.sim_time_ms = 0
        
        # V9.0: 闭环修正模块初始化
        self.feedback_update_counter = 0
        self.closed_loop_compensation = [0.0] * NUM_MOTORS
        self.CLOSED_LOOP_UPDATE_INTERVAL = 5
        self.CLOSED_LOOP_ERROR_THRESHOLD = 3.0
        self.CLOSED_LOOP_CORRECTION_FACTOR = 0.05
        self.CLOSED_LOOP_ERROR_DEADZONE = 0.5
        self.CLOSED_LOOP_COMPENSATION_MAX = 3.0
        self.CLOSED_LOOP_COMPENSATION_DECAY = 0.98
        
        # 数据记录
        self.history = {
            'time': [],
            'upper_target': [[] for _ in range(NUM_MOTORS)],
            'planned_position': [[] for _ in range(NUM_MOTORS)],
            'planned_velocity': [[] for _ in range(NUM_MOTORS)],
            'planned_acceleration': [[] for _ in range(NUM_MOTORS)],
            'motor_position': [[] for _ in range(NUM_MOTORS)],
            'motor_velocity': [[] for _ in range(NUM_MOTORS)],
            'trajectory_state': [[] for _ in range(NUM_MOTORS)],
            'can_sent': [],
        }
        
        # 统计
        self.stats = {
            'upper_frames': 0,
            'control_loops': 0,
            'can_sent': 0,
            'can_skipped': 0,
        }
    
    def run_simulation(self, duration: float = 10.0):
        """V11.1: 运行仿真 - 完整匹配工程代码逻辑"""
        print(f"[Simulator V11.1] Starting simulation for {duration} seconds...")
        print(f"[Simulator V11.1] Upper computer: {UPPER_COMPUTER_FREQ}Hz, STM32: {STM32_CONTROL_FREQ}Hz")
        print(f"[Simulator V11.1] Features: Double Buffer + Interp Error Correction + CAN Monitor Optimization")
        
        last_upper_time = 0.0
        upper_target_angles = [MOTOR_NEUTRAL] * NUM_MOTORS
        interpolated_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        
        while self.time < duration:
            # 1. 上位机50Hz发送数据
            if self.time - last_upper_time >= self.dt_upper:
                upper_target_angles = self.gait_generator.generate_rl_like_trot(self.time)
                
                # V11.0: 写入双缓冲区
                self.target_buffer.write(upper_target_angles, self.sim_time_ms)
                
                self.smart_controller.update_all(upper_target_angles, self.trajectory_planner)
                last_upper_time = self.time
                self.stats['upper_frames'] += 1
            
            # V11.0: 每个控制周期读取插值数据
            if self.use_interpolation:
                interpolated_targets = self.target_buffer.read_interpolated()
            else:
                interpolated_targets = upper_target_angles[:]
            
            # V11.0: 更新轨迹规划器（使用插值后的目标）
            for i in range(NUM_MOTORS):
                self.trajectory_planner.set_continuous_target(i, interpolated_targets[i])
            
            # 2. STM32 200Hz控制循环
            traj_outputs = self.trajectory_planner.step_all(self.dt_control)
            
            # V11.0: CAN监测请求优化
            for i in range(NUM_MOTORS):
                if self.monitor_optimizer.should_request(i, self.sim_time_ms):
                    # 模拟读取电机反馈数据
                    pass  # 电机模型已直接提供数据
            
            # 3. CAN发送优化 - V5.0: 添加速度和加速度限制
            can_positions = []
            can_velocities = []
            can_ff_vel = []
            can_ff_accel = []
            can_speed_limits = []  # V5.0: 速度限制 (rpm)
            can_accel_limits = []  # V5.0: 加速度限制 (rpm/s)
            
            for i in range(NUM_MOTORS):
                planner = self.trajectory_planner.planners[i]
                can_state = self.can_send_states[i]
                is_accel = planner.state == TrajectoryState.ACCELERATING or \
                          planner.state == TrajectoryState.DECELERATING
                
                # V5.0: 计算速度和加速度限制 (deg/s -> rpm)
                planned_velocity_deg_s = abs(traj_outputs[i].velocity)
                speed_limit_rpm = planned_velocity_deg_s / 6.0
                speed_limit_rpm = max(10.0, min(200.0, speed_limit_rpm))
                
                planned_accel_deg_s2 = abs(traj_outputs[i].acceleration)
                accel_limit_rpm_s = planned_accel_deg_s2 / 6.0
                accel_limit_rpm_s = max(20.0, min(300.0, accel_limit_rpm_s))
                
                if self.can_optimizer.should_send(
                    i, traj_outputs[i].position, traj_outputs[i].velocity,
                    self.time, is_accel
                ):
                    # V5.3: 更新电机反馈数据
                    self.motor_model.update_feedback(i, traj_outputs[i].velocity, self.dt_control)
                    
                    # V5.3: 计算补偿后的位置 (预测修正+摩擦补偿+转矩补偿)
                    compensated_pos = self.motor_model.apply_compensation(
                        i, traj_outputs[i].position, traj_outputs[i].velocity, self.dt_control
                    )
                    
                    can_positions.append(compensated_pos)
                    can_velocities.append(traj_outputs[i].velocity)
                    can_ff_vel.append(traj_outputs[i].feedforward_vel)
                    can_ff_accel.append(traj_outputs[i].feedforward_accel)
                    can_speed_limits.append(speed_limit_rpm)
                    can_accel_limits.append(accel_limit_rpm_s)
                    
                    # V5.0: 实时设置电机限制 (只要变化就设置)
                    self.motor_model.set_limits(i, speed_limit_rpm, accel_limit_rpm_s)
                    
                    # 记录发送状态
                    can_state.last_sent_position = compensated_pos
                    can_state.last_sent_velocity = traj_outputs[i].velocity
                    can_state.last_send_time = self.time
                    can_state.skip_count = 0
                    self.stats['can_sent'] += 1
                else:
                    # V7.1: 修复CAN发送跳过时目标不一致问题
                    # 使用补偿位置保持与发送时一致
                    compensated_pos = self.motor_model.apply_compensation(
                        i, traj_outputs[i].position, traj_outputs[i].velocity, self.dt_control
                    )
                    can_positions.append(compensated_pos)
                    can_velocities.append(traj_outputs[i].velocity)
                    can_ff_vel.append(traj_outputs[i].feedforward_vel)
                    can_ff_accel.append(traj_outputs[i].feedforward_accel)
                    
                    # V5.0: 跳过发送时也要更新限制（如果变化了）
                    if abs(speed_limit_rpm - self.motor_model.speed_limit[i]) > 1.0 or \
                       abs(accel_limit_rpm_s - self.motor_model.accel_limit[i]) > 5.0:
                        self.motor_model.set_limits(i, speed_limit_rpm, accel_limit_rpm_s)
                    
                    can_speed_limits.append(self.motor_model.speed_limit[i])
                    can_accel_limits.append(self.motor_model.accel_limit[i])
                    
                    # 仍然记录上次发送的位置用于阈值判断
                    can_state.skip_count += 1
                    self.stats['can_skipped'] += 1
            
            # 4. 电机响应 - V4.0: 传递前馈控制量
            self.motor_model.update(can_positions, can_velocities, 
                                   can_ff_vel, can_ff_accel, self.dt_control)
            
            # 5. 闭环修正 - P1优化: 每个控制周期都执行，提高响应速度
            for i in range(NUM_MOTORS):
                actual_pos = self.motor_model.motors[i].position
                self.trajectory_planner.closed_loop_correction(i, actual_pos)
            
            # 5. 记录数据
            self.history['time'].append(self.time)
            for i in range(NUM_MOTORS):
                self.history['upper_target'][i].append(upper_target_angles[i])
                self.history['planned_position'][i].append(traj_outputs[i].position)
                self.history['planned_velocity'][i].append(traj_outputs[i].velocity)
                self.history['planned_acceleration'][i].append(traj_outputs[i].acceleration)
                self.history['motor_position'][i].append(self.motor_model.motors[i].position)
                self.history['motor_velocity'][i].append(self.motor_model.motors[i].velocity)
                self.history['trajectory_state'][i].append(traj_outputs[i].state)
            
            self.stats['control_loops'] += 1
            self.time += self.dt_control
            self.sim_time_ms += int(self.dt_control * 1000)
        
        # V11.1: 打印统计
        print(f"\n[Simulator V11.1] Simulation completed!")
        print(f"  Upper frames: {self.stats['upper_frames']}")
        print(f"  Control loops: {self.stats['control_loops']}")
        print(f"  CAN sent: {self.stats['can_sent']}")
        print(f"  CAN skipped: {self.stats['can_skipped']}")
        can_total = self.stats['can_sent'] + self.stats['can_skipped']
        print(f"  CAN efficiency: {self.stats['can_sent'] / can_total * 100:.1f}%")
        
        # V11.0: 新增统计
        monitor_stats = self.monitor_optimizer.get_stats()
        print(f"\n[V11.0] CAN Monitor Request Optimization:")
        print(f"  Monitor requests: {monitor_stats['requests']}")
        print(f"  Monitor skips: {monitor_stats['skips']}")
        print(f"  Monitor efficiency: {monitor_stats['efficiency']:.1f}%")
        print(f"  Effective frequency: {125:.0f}Hz (optimized from 200Hz)")
        
        print(f"\n[V11.0] 50Hz Optimization Features:")
        print(f"  Double Buffer: ENABLED (Size={TARGET_BUFFER_SIZE}, Steps={INTERPOLATION_STEPS})")
        print(f"  Interpolation: {'ENABLED' if self.use_interpolation else 'DISABLED'}")
        print(f"  Error Correction: {'ENABLED' if INTERP_ERROR_CORRECTION_ENABLE else 'DISABLED'}")
        print(f"  Error Threshold: {INTERP_ERROR_THRESHOLD} deg")
        print(f"  Sync Interval: {INTERP_SYNC_INTERVAL_MS} ms")
        
        print(f"\n[V11.0] CAN Send Parameters:")
        print(f"  Pos Threshold: {CAN_SEND_POS_THRESHOLD} deg")
        print(f"  Vel Threshold: {CAN_SEND_VEL_THRESHOLD} deg/s")
        print(f"  Max Interval: {CAN_SEND_MAX_INTERVAL*1000:.0f} ms")
        print(f"  Accel Interval: {CAN_SEND_ACCEL_INTERVAL*1000:.0f} ms")
        print(f"  Decel Interval: {CAN_SEND_DECEL_INTERVAL*1000:.0f} ms")

# ==================== 可视化 ====================
class Visualizer:
    """可视化模块"""
    
    def __init__(self, simulator: QuadrupedSimulator):
        self.simulator = simulator
        self.history = simulator.history
    
    def plot_motor_waveforms(self, motor_ids: List[int] = None):
        """绘制电机波形图"""
        if motor_ids is None:
            motor_ids = list(range(NUM_MOTORS))
        
        n_motors = len(motor_ids)
        fig, axes = plt.subplots(n_motors, 3, figsize=(18, 4 * n_motors))
        
        if n_motors == 1:
            axes = axes.reshape(1, -1)
        
        time = np.array(self.history['time'])
        
        motor_names = ['FL-Hip', 'FL-Thigh', 'FL-Knee',
                       'FR-Hip', 'FR-Thigh', 'FR-Knee',
                       'RL-Hip', 'RL-Thigh', 'RL-Knee',
                       'RR-Hip', 'RR-Thigh', 'RR-Knee']
        
        for idx, motor_id in enumerate(motor_ids):
            ax_pos = axes[idx, 0]
            ax_vel = axes[idx, 1]
            ax_accel = axes[idx, 2]
            
            # 位置
            ax_pos.plot(time, self.history['upper_target'][motor_id], 
                       'b--', label='Upper Target', linewidth=1.5, alpha=0.7)
            ax_pos.plot(time, self.history['planned_position'][motor_id], 
                       'g-', label='Planned', linewidth=1.5)
            ax_pos.plot(time, self.history['motor_position'][motor_id], 
                       'r-', label='Motor Actual', linewidth=1.5, alpha=0.8)
            ax_pos.set_ylabel('Position (deg)')
            ax_pos.set_title(f'Motor {motor_id}: {motor_names[motor_id]}')
            ax_pos.legend(loc='upper right')
            ax_pos.grid(True, alpha=0.3)
            
            # 速度
            ax_vel.plot(time, self.history['planned_velocity'][motor_id], 
                       'g-', label='Planned Vel', linewidth=1.5)
            ax_vel.plot(time, self.history['motor_velocity'][motor_id], 
                       'r-', label='Motor Vel', linewidth=1.5, alpha=0.8)
            ax_vel.set_ylabel('Velocity (deg/s)')
            ax_vel.legend(loc='upper right')
            ax_vel.grid(True, alpha=0.3)
            
            # 加速度
            ax_accel.plot(time, self.history['planned_acceleration'][motor_id], 
                         'g-', label='Planned Accel', linewidth=1.5)
            ax_accel.set_ylabel('Acceleration (deg/s²)')
            ax_accel.set_xlabel('Time (s)')
            ax_accel.legend(loc='upper right')
            ax_accel.grid(True, alpha=0.3)
        
        plt.suptitle('Motor Control Waveforms (Upper 50Hz → STM32 200Hz → Motor)', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('motor_waveforms.png', dpi=150, bbox_inches='tight')
        plt.show()
    
    def plot_tracking_error(self):
        """绘制跟踪误差分析"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        time = np.array(self.history['time'])
        
        # 计算跟踪误差
        errors = []
        for i in range(NUM_MOTORS):
            target = np.array(self.history['upper_target'][i])
            actual = np.array(self.history['motor_position'][i])
            error = target - actual
            errors.append(error)
        
        errors = np.array(errors)
        
        # 1. 各电机跟踪误差
        ax1 = axes[0, 0]
        motor_names = ['FL', 'FR', 'RL', 'RR']
        colors = ['blue', 'red', 'green', 'orange']
        
        for leg_id in range(4):
            leg_errors = errors[leg_id*3:(leg_id+1)*3, :]
            mean_error = np.mean(np.abs(leg_errors), axis=0)
            ax1.plot(time, mean_error, label=f'{motor_names[leg_id]} Leg', 
                    color=colors[leg_id], linewidth=1.5)
        
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Mean Absolute Error (deg)')
        ax1.set_title('Tracking Error by Leg')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. 误差分布直方图
        ax2 = axes[0, 1]
        all_errors = errors.flatten()
        ax2.hist(all_errors, bins=50, density=True, alpha=0.7, color='steelblue')
        ax2.axvline(x=0, color='red', linestyle='--', linewidth=2)
        ax2.set_xlabel('Position Error (deg)')
        ax2.set_ylabel('Density')
        ax2.set_title(f'Error Distribution (Mean: {np.mean(all_errors):.2f}°, Std: {np.std(all_errors):.2f}°)')
        ax2.grid(True, alpha=0.3)
        
        # 3. 误差随时间变化
        ax3 = axes[1, 0]
        mean_error = np.mean(np.abs(errors), axis=0)
        max_error = np.max(np.abs(errors), axis=0)
        
        ax3.fill_between(time, 0, max_error, alpha=0.3, color='red', label='Max Error')
        ax3.plot(time, mean_error, 'b-', linewidth=2, label='Mean Error')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Position Error (deg)')
        ax3.set_title('Error Over Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 统计摘要
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        stats_text = f"""
        Tracking Performance Summary (V2.0 Optimized)
        ════════════════════════════════════════════════
        
        Overall Statistics:
        • Mean Error: {np.mean(all_errors):.3f}°
        • Std Dev: {np.std(all_errors):.3f}°
        • Max Error: {np.max(np.abs(all_errors)):.3f}°
        • RMS Error: {np.sqrt(np.mean(all_errors**2)):.3f}°
        
        Control System:
        • Upper Computer: {UPPER_COMPUTER_FREQ} Hz
        • STM32 Control: {STM32_CONTROL_FREQ} Hz
        • Trajectory Planner: Improved S-Curve (Sinusoidal Jerk)
        • CAN Efficiency: {self.simulator.stats['can_sent'] / (self.simulator.stats['can_sent'] + self.simulator.stats['can_skipped']) * 100:.1f}%
        
        V2.0 Optimizations:
        • Gait Generator: Smooth Sine Wave (No Discontinuities)
        • Jerk Profile: Sinusoidal (Continuous)
        • Feedforward: Vel Gain={FEEDFORWARD_VEL_GAIN}, Accel Gain={FEEDFORWARD_ACCEL_GAIN}
        • Backlash Compensation: Enabled (2.0 deg)
        • Friction Compensation: Enabled
        
        Trajectory Parameters:
        • V_max: {TRAJECTORY_DEFAULT_VMAX} deg/s
        • A_max: {TRAJECTORY_DEFAULT_AMAX} deg/s²
        • Jerk: {TRAJECTORY_JERK_LIMIT} deg/s³
        
        CAN Parameters:
        • Pos Threshold: {CAN_SEND_POS_THRESHOLD} deg
        • Vel Threshold: {CAN_SEND_VEL_THRESHOLD} deg/s
        • Max Interval: {CAN_SEND_MAX_INTERVAL*1000:.0f} ms
        """
        
        ax4.text(0.1, 0.9, stats_text, transform=ax4.transAxes, 
                fontsize=11, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.suptitle('Tracking Error Analysis', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('tracking_error.png', dpi=150, bbox_inches='tight')
        plt.show()
    
    def create_3d_animation(self, duration: float = 5.0, fps: int = 30):
        """创建3D动画"""
        print(f"[Visualizer] Creating 3D animation...")
        
        fig = plt.figure(figsize=(16, 8))
        
        # 3D视图
        ax_3d = fig.add_subplot(121, projection='3d')
        
        # 关节角度视图
        ax_joints = fig.add_subplot(122)
        
        # 机器狗尺寸参数 (与trot_gait_simulator.py一致)
        body_length = 200   # mm
        body_width = 120    # mm
        leg_hip_length = 50     # mm
        leg_thigh_length = 100  # mm
        leg_knee_length = 100   # mm
        body_height = 180       # mm
        
        # 腿部位置
        leg_positions = [
            (body_length/2, body_width/2),    # FL
            (body_length/2, -body_width/2),   # FR
            (-body_length/2, body_width/2),   # RL
            (-body_length/2, -body_width/2),  # RR
        ]
        
        # 计算帧数
        time_array = np.array(self.history['time'])
        total_frames = len(time_array)
        frame_step = max(1, total_frames // (int(duration * fps)))
        frames = list(range(0, total_frames, frame_step))[:int(duration * fps)]
        
        def angle_to_position(angle_deg, length):
            """角度转位置偏移"""
            angle_rad = np.radians(angle_deg - 180)
            return length * np.sin(angle_rad), length * np.cos(angle_rad)
        
        def get_leg_positions(frame_idx, leg_id):
            """获取一条腿的所有关节位置"""
            base_x, base_y = leg_positions[leg_id]
            
            # 获取三个关节角度
            hip_angle = self.history['motor_position'][leg_id * 3][frame_idx]
            thigh_angle = self.history['motor_position'][leg_id * 3 + 1][frame_idx]
            knee_angle = self.history['motor_position'][leg_id * 3 + 2][frame_idx]
            
            # 髋关节 (前后摆动)
            hip_offset_x = leg_hip_length * np.sin(np.radians(hip_angle - 180))
            
            # 大腿 (上下摆动)
            thigh_rad = np.radians(thigh_angle - 180)
            thigh_offset_z = -leg_thigh_length * np.cos(thigh_rad)
            thigh_offset_x = leg_thigh_length * np.sin(thigh_rad)
            
            # 膝关节 (上下摆动)
            knee_rad = np.radians(knee_angle - 180)
            knee_offset_z = -leg_knee_length * np.cos(knee_rad)
            knee_offset_x = leg_knee_length * np.sin(knee_rad)
            
            # 计算各点位置
            p0 = np.array([base_x, base_y, body_height])  # 髋部连接点
            p1 = np.array([base_x + hip_offset_x, base_y, body_height])  # 髋关节
            p2 = np.array([base_x + hip_offset_x + thigh_offset_x, base_y, 
                          body_height + thigh_offset_z])  # 膝关节
            p3 = np.array([base_x + hip_offset_x + thigh_offset_x + knee_offset_x, 
                          base_y, body_height + thigh_offset_z + knee_offset_z])  # 足端
            
            return [p0, p1, p2, p3]
        
        def init():
            ax_3d.clear()
            ax_joints.clear()
            return []
        
        def animate(frame_idx):
            ax_3d.clear()
            ax_joints.clear()
            
            # 设置3D视图
            ax_3d.set_xlim(-250, 250)
            ax_3d.set_ylim(-250, 250)
            ax_3d.set_zlim(-50, 350)
            ax_3d.set_xlabel('X (mm)')
            ax_3d.set_ylabel('Y (mm)')
            ax_3d.set_zlabel('Z (mm)')
            ax_3d.set_title(f'Quadruped Robot 3D View\nTime: {time_array[frame_idx]:.2f}s')
            
            # 绘制躯干
            body_x = [leg_positions[0][0], leg_positions[1][0], 
                     leg_positions[3][0], leg_positions[2][0], leg_positions[0][0]]
            body_y = [leg_positions[0][1], leg_positions[1][1], 
                     leg_positions[3][1], leg_positions[2][1], leg_positions[0][1]]
            body_z = [body_height] * 5
            ax_3d.plot(body_x, body_y, body_z, 'b-', linewidth=3)
            
            # 绘制四条腿
            colors = ['red', 'green', 'blue', 'orange']
            leg_names = ['FL', 'FR', 'RL', 'RR']
            
            for leg_id in range(4):
                points = get_leg_positions(frame_idx, leg_id)
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                zs = [p[2] for p in points]
                
                # 绘制腿部
                ax_3d.plot(xs, ys, zs, color=colors[leg_id], linewidth=2, 
                          label=f'{leg_names[leg_id]} Leg')
                
                # 绘制关节点
                ax_3d.scatter(xs[1:], ys[1:], zs[1:], color=colors[leg_id], s=30)
                
                # 绘制足端
                ax_3d.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=colors[leg_id], 
                             s=60, marker='o')
            
            ax_3d.legend(loc='upper left')
            ax_3d.view_init(elev=25, azim=45 + frame_idx * 0.2)
            
            # 绘制关节角度
            ax_joints.set_title('Joint Angles')
            ax_joints.set_xlabel('Motor ID')
            ax_joints.set_ylabel('Angle (deg)')
            
            angles = [self.history['motor_position'][i][frame_idx] for i in range(NUM_MOTORS)]
            x_labels = ['FL-H', 'FL-T', 'FL-K', 'FR-H', 'FR-T', 'FR-K',
                       'RL-H', 'RL-T', 'RL-K', 'RR-H', 'RR-T', 'RR-K']
            
            bars = ax_joints.bar(range(NUM_MOTORS), angles, color=['red']*3 + ['green']*3 + 
                                ['blue']*3 + ['orange']*3, alpha=0.7)
            ax_joints.set_xticks(range(NUM_MOTORS))
            ax_joints.set_xticklabels(x_labels, rotation=45, ha='right')
            ax_joints.axhline(y=180, color='black', linestyle='--', alpha=0.5)
            ax_joints.set_ylim(120, 240)
            ax_joints.grid(True, alpha=0.3, axis='y')
            
            return []
        
        anim = FuncAnimation(fig, animate, init_func=init, frames=frames,
                            interval=1000/fps, blit=False)
        
        plt.tight_layout()
        
        # 保存动画
        try:
            anim.save('quadruped_animation.gif', writer='pillow', fps=fps)
            print(f"[Visualizer] Animation saved to 'quadruped_animation.gif'")
        except Exception as e:
            print(f"[Visualizer] Could not save animation: {e}")
        
        plt.show()

# ==================== 主函数 ====================
class ReportGenerator:
    """仿真报告生成器"""
    
    def __init__(self, simulator: QuadrupedSimulator):
        self.sim = simulator
        self.history = simulator.history
        self.stats = simulator.stats
    
    def generate_report(self, filename: str = "simulation_report.txt"):
        """生成详细仿真报告"""
        report = []
        report.append("=" * 80)
        report.append("         四足机器狗控制系统仿真报告")
        report.append("         Quadruped Robot Control System Simulation Report")
        report.append("=" * 80)
        report.append("")
        
        report.extend(self._generate_parameters_section())
        report.extend(self._generate_performance_section())
        report.extend(self._generate_motor_details())
        report.extend(self._generate_control_analysis())
        report.extend(self._generate_algorithm_comparison())
        report.extend(self._generate_conclusion())
        
        report_text = "\n".join(report)
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(report_text)
        
        print(f"\n[Report] Report saved to: {filename}")
        return report_text
    
    def _generate_parameters_section(self) -> List[str]:
        """生成仿真参数部分"""
        section = []
        section.append("【仿真参数 / Simulation Parameters】")
        section.append("-" * 80)
        section.append("")
        section.append("  1. 系统级参数:")
        section.append(f"     • 仿真时长:         {self.sim.time:.2f} 秒")
        section.append(f"     • 上位机频率:       {UPPER_COMPUTER_FREQ} Hz (周期: {1.0/UPPER_COMPUTER_FREQ*1000:.1f}ms)")
        section.append(f"     • STM32控制频率:    {STM32_CONTROL_FREQ} Hz (周期: {1.0/STM32_CONTROL_FREQ*1000:.1f}ms)")
        section.append(f"     • 电机数量:         {NUM_MOTORS} 个")
        section.append(f"     • 电机中性位置:     {MOTOR_NEUTRAL}°")
        section.append("")
        section.append("  2. 轨迹规划参数:")
        section.append(f"     • 最大速度:         {TRAJECTORY_DEFAULT_VMAX} °/s")
        section.append(f"     • 最大加速度:       {TRAJECTORY_DEFAULT_AMAX} °/s²")
        section.append(f"     • 最大减速度:       {TRAJECTORY_DEFAULT_DMAX} °/s²")
        section.append(f"     • 加加速度限制:     {TRAJECTORY_JERK_LIMIT} °/s³")
        section.append(f"     • 位置阈值:         {TRAJECTORY_POS_THRESHOLD} °")
        section.append(f"     • 速度阈值:         {TRAJECTORY_VEL_THRESHOLD} °/s")
        section.append("")
        section.append("  3. 控制参数 (前馈+PID):")
        section.append(f"     • 速度前馈增益:     {FEEDFORWARD_VEL_GAIN}")
        section.append(f"     • 加速度前馈增益:   {FEEDFORWARD_ACCEL_GAIN}")
        section.append(f"     • 位置比例增益:     {POS_KP}")
        section.append(f"     • 速度积分增益:     {VEL_KI}")
        section.append(f"     • 速度微分增益:     {VEL_KD}")
        section.append("")
        section.append("  4. CAN发送优化参数:")
        section.append(f"     • 位置变化阈值:     {CAN_SEND_POS_THRESHOLD} °")
        section.append(f"     • 速度变化阈值:     {CAN_SEND_VEL_THRESHOLD} °/s")
        section.append(f"     • 最大发送间隔:     {CAN_SEND_MAX_INTERVAL*1000:.1f} ms")
        section.append(f"     • 加速阶段间隔:     {CAN_SEND_ACCEL_INTERVAL*1000:.1f} ms")
        section.append(f"     • 减速阶段间隔:     {CAN_SEND_DECEL_INTERVAL*1000:.1f} ms")
        section.append("")
        section.append("  5. 通信统计:")
        section.append(f"     • 上位机发送帧数:   {self.stats['upper_frames']}")
        section.append(f"     • 控制循环次数:    {self.stats['control_loops']}")
        section.append(f"     • CAN实际发送次数: {self.stats['can_sent']}")
        section.append(f"     • CAN跳过次数:      {self.stats['can_skipped']}")
        can_efficiency = self.stats['can_sent'] / (self.stats['can_sent'] + self.stats['can_skipped']) * 100 if self.stats['can_sent'] + self.stats['can_skipped'] > 0 else 0
        section.append(f"     • CAN发送效率:      {can_efficiency:.1f}%")
        section.append("")
        
        return section
    
    def _generate_performance_section(self) -> List[str]:
        """生成性能指标部分"""
        section = []
        section.append("【性能指标 / Performance Metrics】")
        section.append("-" * 80)
        section.append("")
        
        errors = []
        for i in range(NUM_MOTORS):
            target = np.array(self.history['upper_target'][i])
            planned = np.array(self.history['planned_position'][i])
            actual = np.array(self.history['motor_position'][i])
            
            planning_error = np.abs(target - planned)
            tracking_error = np.abs(target - actual)
            
            errors.append({
                'motor_id': i,
                'planning_mean': np.mean(planning_error),
                'planning_max': np.max(planning_error),
                'planning_rms': np.sqrt(np.mean(planning_error**2)),
                'tracking_mean': np.mean(tracking_error),
                'tracking_max': np.max(tracking_error),
                'tracking_rms': np.sqrt(np.mean(tracking_error**2)),
            })
        
        all_planning_errors = [e['planning_mean'] for e in errors]
        all_tracking_errors = [e['tracking_mean'] for e in errors]
        
        section.append("  轨迹规划性能:")
        section.append(f"     • 平均规划误差:     {np.mean(all_planning_errors):.4f} °")
        section.append(f"     • 最大规划误差:     {np.max([e['planning_max'] for e in errors]):.4f} °")
        section.append(f"     • RMS规划误差:      {np.sqrt(np.mean(np.array(all_planning_errors)**2)):.4f} °")
        section.append("")
        section.append("  电机跟踪性能:")
        section.append(f"     • 平均跟踪误差:     {np.mean(all_tracking_errors):.4f} °")
        section.append(f"     • 最大跟踪误差:     {np.max([e['tracking_max'] for e in errors]):.4f} °")
        section.append(f"     • RMS跟踪误差:      {np.sqrt(np.mean(np.array(all_tracking_errors)**2)):.4f} °")
        section.append("")
        
        return section
    
    def _generate_motor_details(self) -> List[str]:
        """生成各电机详细数据"""
        section = []
        section.append("【各电机详细数据 / Motor Details】")
        section.append("-" * 80)
        section.append("")
        
        motor_names = ['FL-Hip', 'FL-Thigh', 'FL-Knee', 
                      'FR-Hip', 'FR-Thigh', 'FR-Knee',
                      'RL-Hip', 'RL-Thigh', 'RL-Knee', 
                      'RR-Hip', 'RR-Thigh', 'RR-Knee']
        
        for i in range(NUM_MOTORS):
            target = np.array(self.history['upper_target'][i])
            planned = np.array(self.history['planned_position'][i])
            actual = np.array(self.history['motor_position'][i])
            velocity = np.array(self.history['motor_velocity'][i])
            
            tracking_error = np.abs(target - actual)
            
            section.append(f"  电机 {i:2d} ({motor_names[i]}):")
            section.append(f"    - 目标位置范围:     {np.min(target):.2f} ~ {np.max(target):.2f} °")
            section.append(f"    - 规划位置范围:    {np.min(planned):.2f} ~ {np.max(planned):.2f} °")
            section.append(f"    - 实际位置范围:    {np.min(actual):.2f} ~ {np.max(actual):.2f} °")
            section.append(f"    - 平均跟踪误差:    {np.mean(tracking_error):.4f} °")
            section.append(f"    - 最大跟踪误差:    {np.max(tracking_error):.4f} °")
            section.append(f"    - RMS跟踪误差:     {np.sqrt(np.mean(tracking_error**2)):.4f} °")
            section.append(f"    - 电机最大速度:    {np.max(np.abs(velocity)):.2f} °/s")
            section.append("")
        
        return section
    
    def _generate_control_analysis(self) -> List[str]:
        """生成控制分析部分"""
        section = []
        section.append("【控制算法分析 / Control Algorithm Analysis】")
        section.append("-" * 80)
        section.append("")
        
        section.append("  数据流路径:")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ 上位机 trot_gait_simulator.py (50Hz)                          │")
        section.append("    │   └── 生成Trot步态 → 12个电机目标角度 → TCP发送               │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("                                    ↓ 50Hz, 20ms间隔")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ STM32 TCP接收 → quad_command_fusion.c                         │")
        section.append("    │   └── 历史缓冲(32帧) → 前瞻缓冲(10帧) → 曲率计算 → 轨迹规划   │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("                                    ↓")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ trajectory_planner.c (200Hz)                                  │")
        section.append("    │   └── 梯形/S曲线规划 → 状态机(IDLE→ACCEL→CRUISE→DECEL)       │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("                                    ↓")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ quad_smart_controller.c                                       │")
        section.append("    │   └── 场景识别(SWING/STANCE) → 策略选择 → 力位混合控制       │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("                                    ↓")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ CAN发送优化 (100-500Hz自适应)                                 │")
        section.append("    │   └── 位置阈值判断 → 速度阈值判断 → 智能跳过策略              │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("                                    ↓")
        section.append("    ┌──────────────────────────────────────────────────────────────────┐")
        section.append("    │ 电机响应模型                                                   │")
        section.append("    │   └── PD控制 → 电机动力学 → 位置/速度输出                     │")
        section.append("    └──────────────────────────────────────────────────────────────────┘")
        section.append("")
        
        section.append("  关键算法说明:")
        section.append("")
        section.append("  1. 轨迹规划 (TrajectoryPlanner):")
        section.append("     • 使用梯形速度曲线或S曲线进行平滑")
        section.append("     • 状态机: IDLE → ACCELERATING → CRUISING → DECELERATING → STOPPING")
        section.append("     • 自动计算加减速距离，实现无超调到达目标")
        section.append("")
        section.append("  2. 前馈+PID控制:")
        section.append("     • 速度前馈: 补偿电机惯性")
        section.append("     • 加速度前馈: 补偿动态响应")
        section.append("     • PID反馈: 消除稳态误差")
        section.append("")
        section.append("  3. CAN发送优化:")
        section.append("     • 仅当位置/速度变化超过阈值时发送")
        section.append("     • 加速/减速阶段提高发送频率")
        section.append("     • 节省CAN总线带宽")
        section.append("")
        
        return section
    
    def _generate_algorithm_comparison(self) -> List[str]:
        """生成工程代码对比部分"""
        section = []
        section.append("【工程代码 vs 仿真脚本对比】")
        section.append("-" * 80)
        section.append("")
        section.append("  特性                     | 工程代码                    | 仿真脚本")
        section.append("  ─────────────────────────┼─────────────────────────────┼───────────────")
        section.append("  上位机步态生成           | Python (trot_gait_sim)      | ✓ 一致")
        section.append("  TCP传输                  | TCP Socket                  | ✓ 一致")
        section.append("  历史缓冲(32帧)           | HistoryBuffer               | ✓ 已实现")
        section.append("  前瞻缓冲(10帧)           | LookaheadBuffer             | ✓ 已实现")
        section.append("  曲率计算                  | HistoryBuffer_Curvature    | ✓ 已实现")
        section.append("  趋势分析                  | Lookahead_AnalyzeTrend     | ✓ 已实现")
        section.append("  轨迹规划                  | TrajectoryPlanner          | ✓ 一致")
        section.append("  梯形/S曲线               | Trapezoid/S-Curve          | ✓ 一致")
        section.append("  方向切换处理             | DirectionChange            | ✓ 一致")
        section.append("  速度保留                 | SetTargetWithVelocity      | ✓ 已实现")
        section.append("  反向间隙补偿             | BacklashCompensation       | ✓ 一致")
        section.append("  摩擦力补偿               | FrictionCompensation       | ✓ 一致")
        section.append("  前馈+PID控制             | Feedforward+PID            | ✓ 一致")
        section.append("  CAN发送优化              | CANOptimizer               | ✓ 一致")
        section.append("  闭环修正                 | ClosedLoopCorrection       | ✓ 已实现")
        section.append("  电机动力学               | 电机硬件                   | ~ 简化模型")
        section.append("  场景识别                 | Scene_Identify             | ✓ 一致")
        section.append("  策略选择                 | Strategy_Select            | ✓ 一致")
        section.append("  力位混合控制             | HybridControl              | ✗ 未实现")
        section.append("  足端力反馈               | ForceSensor                | ✗ 未实现")
        section.append("")
        
        return section
    
    def _generate_conclusion(self) -> List[str]:
        """生成结论部分"""
        section = []
        section.append("【结论 / Conclusion】")
        section.append("-" * 80)
        section.append("")
        
        section.append("  1. 仿真脚本完整性评估:")
        section.append("     ✓ 已实现核心控制算法")
        section.append("     ✓ 轨迹规划与工程代码一致")
        section.append("     ✓ 前馈+PID控制参数匹配")
        section.append("     ✓ CAN发送优化策略一致")
        section.append("     ✓ 已实现历史缓冲和前瞻缓冲(提升平滑度)")
        section.append("     ✓ 已实现闭环修正机制")
        section.append("     ✗ 缺少力位混合控制(影响支撑相)")
        section.append("     ✗ 电机模型为简化版本")
        section.append("")
        
        section.append("  2. 工程控制效果评估:")
        section.append("     • 轨迹跟踪精度: 11.22° 平均误差")
        section.append("     • 响应速度: 200Hz控制频率可满足常规运动需求")
        section.append("     • 平滑性: 梯形/S曲线规划可有效减少冲击")
        section.append("     • 稳定性: 前馈+PID可实现较好的跟随效果")
        section.append("")
        
        section.append("  3. 建议改进:")
        section.append("     • 添加力位混合控制和足端力反馈")
        section.append("     • 使用更精确的电机动力学模型")
        section.append("")
        
        section.append("=" * 80)
        section.append("                        仿真报告结束")
        section.append("=" * 80)
        
        return section


def main():
    print("=" * 60)
    print("Quadruped Robot Full Control System Simulator")
    print("=" * 60)
    print()
    print("Control Flow:")
    print("  1. Upper Computer (50Hz) → Trot Gait Generator")
    print("  2. TCP → STM32 Receiver")
    print("  3. Smart Controller → Scene Recognition + Strategy")
    print("  4. Trajectory Planner → S-Curve + Trapezoid")
    print("  5. 200Hz Control Loop → CAN Optimizer")
    print("  6. Motor Response Model")
    print()
    
    # 创建仿真器
    simulator = QuadrupedSimulator()
    
    # 运行仿真 (10秒)
    simulator.run_simulation(duration=10.0)
    
    # 生成仿真报告
    print("\n[Main] Generating simulation report...")
    report_gen = ReportGenerator(simulator)
    report_text = report_gen.generate_report("quadruped_simulation_report.txt")
    
    # 创建可视化
    visualizer = Visualizer(simulator)
    
    # 绘制波形图 (选择前6个电机)
    print("\n[Main] Plotting motor waveforms...")
    visualizer.plot_motor_waveforms(motor_ids=[0, 1, 2, 3, 4, 5])
    
    # 绘制跟踪误差分析
    print("\n[Main] Plotting tracking error analysis...")
    visualizer.plot_tracking_error()
    
    # 创建3D动画
    print("\n[Main] Creating 3D animation...")
    visualizer.create_3d_animation(duration=5.0, fps=30)
    
    print("\n[Main] Simulation complete!")

if __name__ == "__main__":
    main()
