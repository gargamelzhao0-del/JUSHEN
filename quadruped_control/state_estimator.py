"""
状态估计模块 - 四足机器狗上位机控制
State Estimator for Quadruped Robot Control
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass, field
from collections import deque
import time

from data_types import (
    MotorStatus, FootForceData, IMUData, RobotState, 
    ContactState, RobotMode
)
from config import StateEstimatorConfig


@dataclass
class StateEstimate:
    """状态估计结果"""
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    foot_positions: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    foot_velocities: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    contact_states: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.int32))
    
    is_valid: bool = False
    timestamp: float = 0.0


class LowPassFilter:
    """低通滤波器"""
    
    def __init__(self, alpha: float = 0.5):
        self.alpha = alpha
        self.value: Optional[np.ndarray] = None
    
    def filter(self, new_value: np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = new_value.copy()
        else:
            self.value = self.alpha * self.value + (1 - self.alpha) * new_value
        return self.value.copy()
    
    def reset(self):
        self.value = None


class StateEstimator:
    """状态估计器"""
    
    LEG_INDICES = {
        'FL': 0, 'FR': 1, 'RL': 2, 'RR': 3
    }
    
    def __init__(self, config: Optional[StateEstimatorConfig] = None):
        self.config = config or StateEstimatorConfig()
        
        self._estimate = StateEstimate()
        self._robot_state = RobotState()
        
        self._attitude_filter = LowPassFilter(self.config.attitude_filter_alpha)
        self._velocity_filter = LowPassFilter(self.config.velocity_filter_alpha)
        
        self._position_history = deque(maxlen=10)
        self._velocity_history = deque(maxlen=5)
        
        self._last_update_time = time.time()
        self._last_position = None
        
        self._gravity = np.array([0.0, 0.0, -9.81])
        
        self._hip_offsets = np.array([
            [0.2, 0.1, 0.0],
            [0.2, -0.1, 0.0],
            [-0.2, 0.1, 0.0],
            [-0.2, -0.1, 0.0],
        ])
        
        self._leg_lengths = np.array([0.1, 0.15])
    
    def update(self, motor_status: MotorStatus, 
               foot_force: Optional[FootForceData] = None,
               imu_data: Optional[IMUData] = None) -> StateEstimate:
        """
        更新状态估计
        
        Args:
            motor_status: 电机状态
            foot_force: 足端力数据（可选）
            imu_data: IMU数据（可选）
        
        Returns:
            状态估计结果
        """
        current_time = time.time()
        dt = current_time - self._last_update_time
        self._last_update_time = current_time
        
        if imu_data is not None:
            self._estimate_attitude_from_imu(imu_data)
        
        if foot_force is not None:
            self._estimate_contact_states(foot_force)
        
        self._estimate_foot_positions(motor_status)
        
        self._estimate_base_velocity(motor_status, dt)
        
        self._estimate.is_valid = True
        self._estimate.timestamp = current_time
        
        if foot_force is not None:
            self._update_robot_state(motor_status, foot_force)
        
        return self._estimate
    
    def _estimate_attitude_from_imu(self, imu_data: IMUData):
        """从IMU估计姿态"""
        euler = imu_data.euler_angles
        
        filtered_euler = self._attitude_filter.filter(euler)
        
        self._estimate.base_orientation = filtered_euler
        self._estimate.base_angular_velocity = imu_data.angular_velocity.copy()
        
        self._robot_state.imu_data = imu_data
        self._robot_state.base_orientation = filtered_euler
        self._robot_state.base_angular_velocity = imu_data.angular_velocity
    
    def _estimate_contact_states(self, foot_force: FootForceData):
        """估计足端接触状态"""
        for i in range(4):
            force = foot_force.forces[i]
            
            if force >= self.config.support_threshold:
                self._estimate.contact_states[i] = ContactState.SUPPORTING
            elif force >= self.config.contact_threshold:
                self._estimate.contact_states[i] = ContactState.TOUCHING
            else:
                self._estimate.contact_states[i] = ContactState.NONE
        
        foot_force.contact_states = self._estimate.contact_states.copy()
    
    def _estimate_foot_positions(self, motor_status: MotorStatus):
        """估计足端位置（正运动学）"""
        for leg_id in range(4):
            joint_angles = motor_status.get_leg_positions(leg_id)
            foot_pos = self._forward_kinematics(leg_id, joint_angles)
            self._estimate.foot_positions[leg_id] = foot_pos
    
    def _forward_kinematics(self, leg_id: int, joint_angles: np.ndarray) -> np.ndarray:
        """
        正运动学：关节角度 -> 足端位置
        
        Args:
            leg_id: 腿ID (0-3)
            joint_angles: [hip, thigh, knee] 关节角度（度）
        
        Returns:
            足端位置 [x, y, z]
        """
        hip_offset = self._hip_offsets[leg_id]
        
        hip_angle = np.deg2rad(joint_angles[0] - 180.0)
        thigh_angle = np.deg2rad(joint_angles[1] - 180.0)
        knee_angle = np.deg2rad(joint_angles[2] - 180.0)
        
        L1 = self._leg_lengths[0]
        L2 = self._leg_lengths[1]
        
        direction = 1.0 if leg_id in [0, 2] else -1.0
        
        x = hip_offset[0] + L1 * np.sin(thigh_angle) + L2 * np.sin(thigh_angle + knee_angle)
        y = hip_offset[1] + direction * (L1 * np.cos(thigh_angle) + L2 * np.cos(thigh_angle + knee_angle)) * np.sin(hip_angle)
        z = -L1 * np.cos(thigh_angle) - L2 * np.cos(thigh_angle + knee_angle)
        
        return np.array([x, y, z])
    
    def _estimate_base_velocity(self, motor_status: MotorStatus, dt: float):
        """估计机体速度"""
        if dt <= 0 or dt > 1.0:
            return
        
        current_positions = motor_status.positions.copy()
        
        if self._last_position is not None:
            joint_velocities = (current_positions - self._last_position) / dt
            
            base_vel = np.zeros(3)
            num_contact = 0
            
            for leg_id in range(4):
                if self._estimate.contact_states[leg_id] == ContactState.SUPPORTING:
                    num_contact += 1
            
            if num_contact > 0:
                base_vel = self._velocity_filter.filter(base_vel)
                self._estimate.base_velocity = base_vel
        
        self._last_position = current_positions
    
    def _update_robot_state(self, motor_status: MotorStatus, foot_force: FootForceData):
        """更新机器人状态"""
        self._robot_state.motor_status = motor_status
        self._robot_state.foot_force = foot_force
        self._robot_state.base_position = self._estimate.base_position.copy()
        self._robot_state.base_velocity = self._estimate.base_velocity.copy()
        self._robot_state.base_orientation = self._estimate.base_orientation.copy()
        self._robot_state.base_angular_velocity = self._estimate.base_angular_velocity.copy()
        self._robot_state.update_timestamp()
    
    def get_robot_state(self) -> RobotState:
        """获取机器人完整状态"""
        return self._robot_state
    
    def get_estimate(self) -> StateEstimate:
        """获取状态估计结果"""
        return self._estimate
    
    def get_contact_states(self) -> np.ndarray:
        """获取接触状态"""
        return self._estimate.contact_states.copy()
    
    def is_stable(self) -> bool:
        """检查机器人是否稳定"""
        roll, pitch, yaw = self._estimate.base_orientation
        
        if abs(roll) > np.deg2rad(45) or abs(pitch) > np.deg2rad(45):
            return False
        
        return True
    
    def get_support_polygon(self) -> np.ndarray:
        """获取支撑多边形"""
        support_feet = []
        for i in range(4):
            if self._estimate.contact_states[i] == ContactState.SUPPORTING:
                support_feet.append(self._estimate.foot_positions[i][:2])
        
        if len(support_feet) >= 3:
            return np.array(support_feet)
        return np.array([])
    
    def reset(self):
        """重置状态估计器"""
        self._estimate = StateEstimate()
        self._robot_state = RobotState()
        self._attitude_filter.reset()
        self._velocity_filter.reset()
        self._position_history.clear()
        self._velocity_history.clear()
        self._last_position = None
        self._last_update_time = time.time()
        print("[StateEstimator] 已重置")
