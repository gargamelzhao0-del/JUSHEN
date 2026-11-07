"""
配置文件 - 四足机器狗上位机控制参数
Configuration for Quadruped Robot High-Level Control
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


@dataclass
class NetworkConfig:
    """网络配置"""
    stm32_ip: str = "192.168.1.100"
    jetson_ip: str = "0.0.0.0"
    tcp_port: int = 9999
    connection_timeout: float = 30.0
    reconnect_interval: float = 2.0
    heartbeat_interval: float = 1.0


@dataclass
class MotorConfig:
    """电机配置"""
    num_motors: int = 12
    position_min: float = 0.0
    position_max: float = 358.0
    velocity_max: float = 500.0
    current_max: float = 10.0
    temperature_max: float = 70.0
    
    default_positions: np.ndarray = field(default_factory=lambda: np.array([
        180.0, 180.0, 180.0,
        180.0, 180.0, 180.0,
        180.0, 180.0, 180.0,
        180.0, 180.0, 180.0,
    ]))
    
    joint_names: List[str] = field(default_factory=lambda: [
        "FL_hip", "FL_thigh", "FL_knee",
        "FR_hip", "FR_thigh", "FR_knee",
        "RL_hip", "RL_thigh", "RL_knee",
        "RR_hip", "RR_thigh", "RR_knee",
    ])


@dataclass
class SafetyConfig:
    """安全配置"""
    position_limits: Tuple[float, float] = (30.0, 330.0)
    velocity_limit: float = 400.0
    current_limit: float = 8.0
    temperature_limit: float = 65.0
    voltage_min: float = 20.0
    voltage_max: float = 29.4
    tilt_angle_max: float = 45.0
    communication_timeout: float = 0.5
    emergency_stop_enabled: bool = True


@dataclass
class ControlConfig:
    """控制配置"""
    control_frequency: float = 50.0
    control_period: float = 0.02
    
    action_scale: float = 1.0
    action_clip: float = 50.0
    
    smoothing_enabled: bool = True
    smoothing_factor: float = 0.3
    
    default_posture_blend: float = 0.1


@dataclass
class StateEstimatorConfig:
    """状态估计配置"""
    imu_topic: str = "/imu/data"
    foot_force_topic: str = "/foot_force"
    motor_status_topic: str = "/motor_status"
    
    contact_threshold: float = 5.0
    support_threshold: float = 20.0
    
    velocity_filter_alpha: float = 0.8
    attitude_filter_alpha: float = 0.9


@dataclass
class IMUConfig:
    """IMU配置"""
    enabled: bool = True
    use_ros2: bool = True
    topic: str = "/imu/data_raw"
    frame_id: str = "gyro_link"
    serial_port: str = "/dev/ttyACM0"
    baud_rate: int = 921600


@dataclass
class PolicyConfig:
    """策略配置"""
    model_path: str = ""
    model_type: str = "pytorch"
    
    observation_dim: int = 48
    action_dim: int = 12
    
    history_length: int = 1
    
    action_scale: np.ndarray = field(default_factory=lambda: np.ones(12) * 0.25)
    action_offset: np.ndarray = field(default_factory=lambda: np.ones(12) * 180.0)
    
    clip_actions: bool = True
    action_noise: float = 0.0


@dataclass
class Config:
    """主配置类"""
    network: NetworkConfig = field(default_factory=NetworkConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    state_estimator: StateEstimatorConfig = field(default_factory=StateEstimatorConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    policy: PolicyConfig = field(default_factory=PolicyConfig)
    
    debug_mode: bool = False
    log_level: str = "INFO"
    
    no_peripheral_mode: bool = False  # V3.2: 无外设模式（仅TCP通信测试）
    
    @classmethod
    def from_yaml(cls, yaml_path: str) -> "Config":
        """从YAML文件加载配置"""
        import yaml
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        config = cls()
        if 'network' in data:
            for k, v in data['network'].items():
                if hasattr(config.network, k):
                    setattr(config.network, k, v)
        if 'motor' in data:
            for k, v in data['motor'].items():
                if hasattr(config.motor, k):
                    setattr(config.motor, k, v)
        if 'safety' in data:
            for k, v in data['safety'].items():
                if hasattr(config.safety, k):
                    setattr(config.safety, k, v)
        if 'control' in data:
            for k, v in data['control'].items():
                if hasattr(config.control, k):
                    setattr(config.control, k, v)
        if 'imu' in data:
            for k, v in data['imu'].items():
                if hasattr(config.imu, k):
                    setattr(config.imu, k, v)
        if 'policy' in data:
            for k, v in data['policy'].items():
                if hasattr(config.policy, k):
                    setattr(config.policy, k, v)
        
        return config
    
    def to_yaml(self, yaml_path: str):
        """保存配置到YAML文件"""
        import yaml
        data = {
            'network': self.network.__dict__,
            'motor': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                     for k, v in self.motor.__dict__.items()},
            'safety': self.safety.__dict__,
            'control': self.control.__dict__,
            'policy': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                      for k, v in self.policy.__dict__.items()},
            'debug_mode': self.debug_mode,
            'log_level': self.log_level,
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
