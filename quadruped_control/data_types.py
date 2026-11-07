"""
数据类型定义 - 四足机器狗上位机控制
Data Types for Quadruped Robot Control
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from enum import IntEnum
import numpy as np
import time


class CommandType(IntEnum):
    """命令类型枚举"""
    HEARTBEAT = 0x01
    EMERGENCY_STOP = 0x0F
    MOTOR_ENABLE_ALL = 0x10
    MOTOR_DISABLE_ALL = 0x11
    MOTOR_POS_CONTROL = 0x12
    MOTOR_VEL_CONTROL = 0x13
    MOTOR_TRQ_CONTROL = 0x14
    MOTOR_SEND_ENABLE = 0x17


class ResponseType(IntEnum):
    """响应类型枚举"""
    ACK = 0x80
    NACK = 0x81
    MOTOR_STATUS = 0x90
    FOOT_FORCE = 0xA0
    POWER_STATUS = 0xA1


class RobotMode(IntEnum):
    """机器人模式枚举"""
    DISABLED = 0
    STANDING = 1
    WALKING = 2
    TROTTING = 3
    FALLING = 4
    RECOVERY = 5


class ContactState(IntEnum):
    """足端接触状态"""
    NONE = 0
    TOUCHING = 1
    SUPPORTING = 2


@dataclass
class MotorStatus:
    """电机状态数据"""
    positions: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.float32))
    velocities: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.float32))
    currents: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.float32))
    temperatures: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.float32))
    error_flags: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.uint16))
    timestamp: float = 0.0
    
    def is_valid(self) -> bool:
        """检查数据是否有效"""
        return (self.positions.shape == (12,) and 
                self.velocities.shape == (12,) and
                self.currents.shape == (12,) and
                self.timestamp > 0)
    
    def get_leg_positions(self, leg_id: int) -> np.ndarray:
        """获取指定腿的关节位置 [hip, thigh, knee]"""
        if 0 <= leg_id < 4:
            return self.positions[leg_id*3 : leg_id*3+3]
        return np.zeros(3)
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'positions': self.positions.tolist(),
            'velocities': self.velocities.tolist(),
            'currents': self.currents.tolist(),
            'temperatures': self.temperatures.tolist(),
            'error_flags': self.error_flags.tolist(),
            'timestamp': self.timestamp,
        }


@dataclass
class FootForceData:
    """足端力数据"""
    forces: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
    contact_mask: int = 0
    contact_states: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.int32))
    timestamp: float = 0.0
    
    FL: int = 0
    FR: int = 1
    RL: int = 2
    RR: int = 3
    
    def is_contact(self, leg_id: int) -> bool:
        """检查指定腿是否接触地面"""
        return bool(self.contact_mask & (1 << leg_id))
    
    def get_force(self, leg_id: int) -> float:
        """获取指定腿的足端力"""
        if 0 <= leg_id < 4:
            return self.forces[leg_id]
        return 0.0
    
    def update_contact_states(self, touch_threshold: float = 5.0, 
                              support_threshold: float = 20.0):
        """根据力值更新接触状态"""
        for i in range(4):
            if self.forces[i] >= support_threshold:
                self.contact_states[i] = ContactState.SUPPORTING
            elif self.forces[i] >= touch_threshold:
                self.contact_states[i] = ContactState.TOUCHING
            else:
                self.contact_states[i] = ContactState.NONE
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'forces': self.forces.tolist(),
            'contact_mask': self.contact_mask,
            'contact_states': self.contact_states.tolist(),
            'timestamp': self.timestamp,
        }


@dataclass
class PowerStatus:
    """电源状态数据"""
    voltage: float = 0.0
    current: float = 0.0
    temperature: float = 0.0
    capacity: float = 0.0
    status: int = 0
    is_charging: bool = False
    timestamp: float = 0.0
    
    def is_low_voltage(self, threshold: float = 22.0) -> bool:
        """检查是否低电压"""
        return self.voltage < threshold
    
    def is_critical(self, threshold: float = 20.0) -> bool:
        """检查是否严重低电压"""
        return self.voltage < threshold
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'voltage': self.voltage,
            'current': self.current,
            'temperature': self.temperature,
            'capacity': self.capacity,
            'status': self.status,
            'is_charging': self.is_charging,
            'timestamp': self.timestamp,
        }


@dataclass
class IMUData:
    """IMU数据"""
    orientation: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    timestamp: float = 0.0
    
    @property
    def roll(self) -> float:
        """获取Roll角度（弧度）"""
        w, x, y, z = self.orientation
        return np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    
    @property
    def pitch(self) -> float:
        """获取Pitch角度（弧度）"""
        w, x, y, z = self.orientation
        return np.arcsin(2.0 * (w * y - z * x))
    
    @property
    def yaw(self) -> float:
        """获取Yaw角度（弧度）"""
        w, x, y, z = self.orientation
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    @property
    def euler_angles(self) -> np.ndarray:
        """获取欧拉角 [roll, pitch, yaw]（弧度）"""
        return np.array([self.roll, self.pitch, self.yaw])
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'orientation': self.orientation.tolist(),
            'angular_velocity': self.angular_velocity.tolist(),
            'linear_acceleration': self.linear_acceleration.tolist(),
            'euler_angles': self.euler_angles.tolist(),
            'timestamp': self.timestamp,
        }


@dataclass
class RobotState:
    """机器人完整状态"""
    motor_status: MotorStatus = field(default_factory=MotorStatus)
    foot_force: FootForceData = field(default_factory=FootForceData)
    power_status: PowerStatus = field(default_factory=PowerStatus)
    imu_data: IMUData = field(default_factory=IMUData)
    
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_orientation: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    
    mode: RobotMode = RobotMode.DISABLED
    timestamp: float = 0.0
    
    def update_timestamp(self):
        """更新时间戳"""
        self.timestamp = time.time()
    
    def is_standing(self) -> bool:
        """检查是否站立"""
        return self.mode in [RobotMode.STANDING, RobotMode.WALKING, RobotMode.TROTTING]
    
    def get_observation_vector(self) -> np.ndarray:
        """生成观测向量（用于RL策略）"""
        obs = np.concatenate([
            self.base_orientation,
            self.base_angular_velocity,
            self.base_velocity,
            self.motor_status.positions / 180.0 - 1.0,
            self.motor_status.velocities / 100.0,
            self.foot_force.forces / 100.0,
        ])
        return obs.astype(np.float32)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'motor_status': self.motor_status.to_dict(),
            'foot_force': self.foot_force.to_dict(),
            'power_status': self.power_status.to_dict(),
            'imu_data': self.imu_data.to_dict(),
            'base_position': self.base_position.tolist(),
            'base_velocity': self.base_velocity.tolist(),
            'base_orientation': self.base_orientation.tolist(),
            'base_angular_velocity': self.base_angular_velocity.tolist(),
            'mode': self.mode.name,
            'timestamp': self.timestamp,
        }


@dataclass
class Command:
    """控制命令"""
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    mode: RobotMode = RobotMode.STANDING
    timestamp: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """转换为数组"""
        return np.array([self.vx, self.vy, self.vyaw], dtype=np.float32)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'vx': self.vx,
            'vy': self.vy,
            'vyaw': self.vyaw,
            'mode': self.mode.name,
            'timestamp': self.timestamp,
        }
