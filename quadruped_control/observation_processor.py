"""
观测处理模块 - 四足机器狗上位机控制
Observation Processor for Quadruped Robot Control

将传感器数据转换为RL策略输入格式
"""

import numpy as np
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field
from collections import deque

from data_types import RobotState, Command, RobotMode
from config import PolicyConfig


@dataclass
class ObservationConfig:
    """观测配置"""
    history_length: int = 1
    
    normalize_joint_pos: bool = True
    joint_pos_scale: float = 180.0
    joint_pos_offset: float = 180.0
    
    normalize_velocity: bool = True
    velocity_scale: float = 100.0
    
    normalize_force: bool = True
    force_scale: float = 100.0
    
    normalize_command: bool = True
    command_scale: float = 1.0
    
    include_gravity: bool = True
    include_clock: bool = False
    clock_frequency: float = 10.0


class ObservationProcessor:
    """观测处理器"""
    
    def __init__(self, config: Optional[ObservationConfig] = None,
                 policy_config: Optional[PolicyConfig] = None):
        self.config = config or ObservationConfig()
        self.policy_config = policy_config
        
        self._obs_dim = self._calculate_obs_dim()
        
        self._obs_history: deque = deque(maxlen=self.config.history_length)
        
        self._last_action = np.zeros(12, dtype=np.float32)
        
        self._clock_phase = 0.0
        self._step_count = 0
    
    def _calculate_obs_dim(self) -> int:
        """计算观测维度"""
        dim = 0
        
        dim += 3
        
        dim += 3
        
        dim += 3
        
        dim += 12
        
        dim += 12
        
        dim += 4
        
        dim += 3
        
        dim += 12
        
        if self.config.include_gravity:
            dim += 3
        
        if self.config.include_clock:
            dim += 2
        
        return dim
    
    def process(self, robot_state: RobotState, 
                command: Optional[Command] = None) -> np.ndarray:
        """
        处理状态数据，生成观测向量
        
        Args:
            robot_state: 机器人状态
            command: 控制命令（可选）
        
        Returns:
            观测向量
        """
        obs_components = []
        
        base_orientation = robot_state.base_orientation.copy()
        obs_components.append(base_orientation)
        
        base_angular_velocity = robot_state.base_angular_velocity.copy()
        obs_components.append(base_angular_velocity)
        
        base_velocity = robot_state.base_velocity.copy()
        obs_components.append(base_velocity)
        
        joint_positions = robot_state.motor_status.positions.copy()
        if self.config.normalize_joint_pos:
            joint_positions = (joint_positions - self.config.joint_pos_offset) / self.config.joint_pos_scale
        obs_components.append(joint_positions)
        
        joint_velocities = robot_state.motor_status.velocities.copy()
        if self.config.normalize_velocity:
            joint_velocities = joint_velocities / self.config.velocity_scale
        obs_components.append(joint_velocities)
        
        foot_forces = robot_state.foot_force.forces.copy() if robot_state.foot_force else np.zeros(4)
        if self.config.normalize_force:
            foot_forces = foot_forces / self.config.force_scale
        obs_components.append(foot_forces)
        
        if command is not None:
            cmd = np.array([command.vx, command.vy, command.vyaw], dtype=np.float32)
            if self.config.normalize_command:
                cmd = cmd / self.config.command_scale
        else:
            cmd = np.zeros(3, dtype=np.float32)
        obs_components.append(cmd)
        
        obs_components.append(self._last_action.copy())
        
        if self.config.include_gravity:
            gravity = np.array([0.0, 0.0, -1.0])
            roll, pitch, yaw = base_orientation
            cr, cp = np.cos(roll), np.cos(pitch)
            sr, sp = np.sin(roll), np.sin(pitch)
            gravity_world = np.array([
                sp,
                -sr * cp,
                cr * cp
            ])
            obs_components.append(gravity_world)
        
        if self.config.include_clock:
            phase = np.array([
                np.sin(2 * np.pi * self._clock_phase),
                np.cos(2 * np.pi * self._clock_phase)
            ])
            obs_components.append(phase)
        
        obs = np.concatenate(obs_components).astype(np.float32)
        
        self._obs_history.append(obs)
        self._step_count += 1
        self._clock_phase = (self._step_count / self.config.clock_frequency) % 1.0
        
        return obs
    
    def process_with_history(self, robot_state: RobotState,
                            command: Optional[Command] = None) -> np.ndarray:
        """
        处理状态数据，生成带历史的观测向量
        
        Args:
            robot_state: 机器人状态
            command: 控制命令
        
        Returns:
            带历史的观测向量
        """
        current_obs = self.process(robot_state, command)
        
        if len(self._obs_history) < self.config.history_length:
            history_obs = np.tile(current_obs, self.config.history_length)
        else:
            history_obs = np.concatenate(list(self._obs_history))
        
        return history_obs
    
    def update_last_action(self, action: np.ndarray):
        """更新上一步动作"""
        self._last_action = action.copy()
    
    def get_observation_dim(self) -> int:
        """获取观测维度"""
        return self._obs_dim
    
    def get_history_observation_dim(self) -> int:
        """获取带历史的观测维度"""
        return self._obs_dim * self.config.history_length
    
    def reset(self):
        """重置观测处理器"""
        self._obs_history.clear()
        self._last_action = np.zeros(12, dtype=np.float32)
        self._clock_phase = 0.0
        self._step_count = 0
    
    @staticmethod
    def create_default_observation(obs_dim: int) -> np.ndarray:
        """创建默认观测向量"""
        return np.zeros(obs_dim, dtype=np.float32)


class ActionProcessor:
    """动作处理器"""
    
    def __init__(self, 
                 action_scale: np.ndarray = None,
                 action_offset: np.ndarray = None,
                 clip_actions: bool = True,
                 clip_range: tuple = (-1.0, 1.0)):
        self.action_scale = action_scale if action_scale is not None else np.ones(12) * 0.25
        self.action_offset = action_offset if action_offset is not None else np.ones(12) * 180.0
        self.clip_actions = clip_actions
        self.clip_range = clip_range
        
        self._default_stance = np.ones(12) * 180.0
    
    def process(self, action: np.ndarray, 
                blend_with_default: float = 0.0) -> np.ndarray:
        """
        处理RL输出的动作
        
        Args:
            action: RL策略输出的动作
            blend_with_default: 与默认站姿的混合比例 (0-1)
        
        Returns:
            处理后的关节位置
        """
        if action.shape != (12,):
            raise ValueError(f"动作维度错误: {action.shape}, 期望 (12,)")
        
        if self.clip_actions:
            action = np.clip(action, self.clip_range[0], self.clip_range[1])
        
        joint_positions = action * self.action_scale + self.action_offset
        
        if blend_with_default > 0:
            joint_positions = (1 - blend_with_default) * joint_positions + \
                            blend_with_default * self._default_stance
        
        return joint_positions.astype(np.float32)
    
    def set_default_stance(self, stance: np.ndarray):
        """设置默认站姿"""
        if stance.shape == (12,):
            self._default_stance = stance.copy()
    
    def set_action_scale(self, scale: np.ndarray):
        """设置动作缩放"""
        if scale.shape == (12,):
            self.action_scale = scale.copy()
    
    def set_action_offset(self, offset: np.ndarray):
        """设置动作偏移"""
        if offset.shape == (12,):
            self.action_offset = offset.copy()
