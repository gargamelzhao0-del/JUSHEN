"""
IMU驱动模块 - 订阅ROS2话题获取IMU数据
IMU Driver - Subscribe to ROS2 topic for IMU data

使用超核电子IMU SDK (ROS2版本)
"""

import threading
import time
import numpy as np
from typing import Optional, Callable
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[IMU] 警告: ROS2未安装，IMU功能将不可用")

from data_types import IMUData


@dataclass
class IMUConfig:
    """IMU配置"""
    enabled: bool = True
    topic: str = "/imu/data_raw"
    frame_id: str = "gyro_link"
    use_ros2: bool = True


class IMUSubscriber(Node if ROS2_AVAILABLE else object):
    """ROS2 IMU订阅节点"""
    
    def __init__(self, topic: str = "/imu/data_raw", callback: Callable = None):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2不可用")
        
        super().__init__('imu_subscriber')
        
        self._callback = callback
        self._latest_imu: Optional[IMUData] = None
        self._lock = threading.Lock()
        
        self.subscription = self.create_subscription(
            Imu,
            topic,
            self._imu_callback,
            10
        )
        
        self.get_logger().info(f"[IMU] 订阅话题: {topic}")
    
    def _imu_callback(self, msg: Imu):
        """ROS2 IMU消息回调"""
        imu_data = IMUData()
        
        imu_data.orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ], dtype=np.float32)
        
        imu_data.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        
        imu_data.linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float32)
        
        imu_data.timestamp = time.time()
        
        with self._lock:
            self._latest_imu = imu_data
        
        if self._callback:
            self._callback(imu_data)
    
    def get_latest_imu(self) -> Optional[IMUData]:
        """获取最新IMU数据"""
        with self._lock:
            return self._latest_imu


class IMUDriver:
    """IMU驱动 - 封装ROS2订阅"""
    
    def __init__(self, config: IMUConfig = None):
        self.config = config or IMUConfig()
        self._imu_data: Optional[IMUData] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._node: Optional[IMUSubscriber] = None
        self._on_data_callback: Optional[Callable] = None
    
    @property
    def imu_data(self) -> Optional[IMUData]:
        """获取最新IMU数据"""
        with self._lock:
            return self._imu_data
    
    @property
    def is_running(self) -> bool:
        """是否正在运行"""
        return self._running
    
    def set_callback(self, callback: Callable[[IMUData], None]):
        """设置数据回调"""
        self._on_data_callback = callback
    
    def start(self) -> bool:
        """启动IMU驱动"""
        if not self.config.enabled:
            print("[IMU] IMU功能已禁用")
            return False
        
        if not ROS2_AVAILABLE:
            print("[IMU] ROS2不可用，无法启动IMU驱动")
            return False
        
        if self._running:
            print("[IMU] IMU驱动已在运行")
            return True
        
        try:
            rclpy.init()
            
            self._node = IMUSubscriber(
                topic=self.config.topic,
                callback=self._on_imu_data
            )
            
            self._running = True
            self._thread = threading.Thread(target=self._spin_thread, daemon=True)
            self._thread.start()
            
            print(f"[IMU] 驱动已启动，订阅话题: {self.config.topic}")
            return True
            
        except Exception as e:
            print(f"[IMU] 启动失败: {e}")
            return False
    
    def stop(self):
        """停止IMU驱动"""
        if not self._running:
            return
        
        self._running = False
        
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        
        if self._node:
            try:
                self._node.destroy_node()
            except:
                pass
            self._node = None
        
        print("[IMU] 驱动已停止")
    
    def _spin_thread(self):
        """ROS2 spin线程"""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
    
    def _on_imu_data(self, imu_data: IMUData):
        """IMU数据回调"""
        with self._lock:
            self._imu_data = imu_data
        
        if self._on_data_callback:
            self._on_data_callback(imu_data)
    
    def get_orientation(self) -> Optional[np.ndarray]:
        """获取姿态四元数 [w, x, y, z]"""
        with self._lock:
            if self._imu_data:
                return self._imu_data.orientation.copy()
        return None
    
    def get_angular_velocity(self) -> Optional[np.ndarray]:
        """获取角速度 [x, y, z] rad/s"""
        with self._lock:
            if self._imu_data:
                return self._imu_data.angular_velocity.copy()
        return None
    
    def get_linear_acceleration(self) -> Optional[np.ndarray]:
        """获取线加速度 [x, y, z] m/s²"""
        with self._lock:
            if self._imu_data:
                return self._imu_data.linear_acceleration.copy()
        return None
    
    def get_euler_angles(self) -> Optional[np.ndarray]:
        """获取欧拉角 [roll, pitch, yaw] 弧度"""
        with self._lock:
            if self._imu_data:
                return self._imu_data.euler_angles.copy()
        return None


class MockIMUDriver:
    """模拟IMU驱动 - 用于测试"""
    
    def __init__(self, config: IMUConfig = None):
        self.config = config or IMUConfig()
        self._imu_data: Optional[IMUData] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._on_data_callback: Optional[Callable] = None
    
    @property
    def imu_data(self) -> Optional[IMUData]:
        return self._imu_data
    
    @property
    def is_running(self) -> bool:
        return self._running
    
    def set_callback(self, callback: Callable[[IMUData], None]):
        self._on_data_callback = callback
    
    def start(self) -> bool:
        if self._running:
            return True
        
        self._running = True
        self._thread = threading.Thread(target=self._mock_thread, daemon=True)
        self._thread.start()
        
        print("[IMU] 模拟驱动已启动")
        return True
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        print("[IMU] 模拟驱动已停止")
    
    def _mock_thread(self):
        """模拟数据生成线程"""
        t = 0.0
        while self._running:
            imu_data = IMUData()
            
            imu_data.orientation = np.array([
                np.cos(t * 0.1),
                0.0,
                0.0,
                np.sin(t * 0.1)
            ], dtype=np.float32)
            
            imu_data.angular_velocity = np.array([
                0.01 * np.sin(t),
                0.01 * np.cos(t),
                0.0
            ], dtype=np.float32)
            
            imu_data.linear_acceleration = np.array([
                0.0,
                0.0,
                9.8
            ], dtype=np.float32)
            
            imu_data.timestamp = time.time()
            
            self._imu_data = imu_data
            
            if self._on_data_callback:
                self._on_data_callback(imu_data)
            
            time.sleep(0.01)
            t += 0.01
    
    def get_orientation(self) -> Optional[np.ndarray]:
        return self._imu_data.orientation.copy() if self._imu_data else None
    
    def get_angular_velocity(self) -> Optional[np.ndarray]:
        return self._imu_data.angular_velocity.copy() if self._imu_data else None
    
    def get_linear_acceleration(self) -> Optional[np.ndarray]:
        return self._imu_data.linear_acceleration.copy() if self._imu_data else None
    
    def get_euler_angles(self) -> Optional[np.ndarray]:
        return self._imu_data.euler_angles.copy() if self._imu_data else None


def create_imu_driver(config: IMUConfig = None, mock: bool = False) -> IMUDriver:
    """创建IMU驱动实例"""
    config = config or IMUConfig()
    
    if mock or not ROS2_AVAILABLE:
        return MockIMUDriver(config)
    else:
        return IMUDriver(config)
