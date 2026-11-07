"""
安全管理模块 - 四足机器狗上位机控制
Safety Manager for Quadruped Robot Control
"""

import numpy as np
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import time

from data_types import MotorStatus, FootForceData, PowerStatus, RobotState, RobotMode
from config import SafetyConfig


class SafetyStatus(Enum):
    """安全状态"""
    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


@dataclass
class SafetyCheckResult:
    """安全检查结果"""
    is_safe: bool
    status: SafetyStatus
    message: str
    details: Dict[str, Any] = field(default_factory=dict)


class SafetyManager:
    """安全管理器"""
    
    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        
        self._last_communication_time = time.time()
        self._last_check_time = time.time()
        self._check_count = 0
        
        self._error_history: List[str] = []
        self._max_history_size = 100
        
        self._emergency_triggered = False
        self._emergency_callback = None
        
        # V3.0: 急停恢复机制
        self._emergency_reason: Optional[str] = None
        self._emergency_time: Optional[float] = None
        self._recovery_cooldown = 5.0  # 恢复冷却时间(秒)
        self._recovery_attempts = 0
        self._max_recovery_attempts = 3
    
    def set_emergency_callback(self, callback):
        """设置紧急停止回调函数"""
        self._emergency_callback = callback
    
    def check_all(self, robot_state: RobotState) -> SafetyCheckResult:
        """
        执行所有安全检查
        
        Args:
            robot_state: 机器人状态
        
        Returns:
            安全检查结果
        """
        self._check_count += 1
        self._last_check_time = time.time()
        
        results = []
        
        results.append(self.check_joint_limits(robot_state.motor_status))
        results.append(self.check_temperature(robot_state.motor_status))
        results.append(self.check_current(robot_state.motor_status))
        results.append(self.check_voltage(robot_state.power_status))
        results.append(self.check_tilt_angle(robot_state))
        results.append(self.check_communication(robot_state))
        results.append(self.check_error_flags(robot_state.motor_status))
        
        for result in results:
            if result.status == SafetyStatus.EMERGENCY:
                self._trigger_emergency(result.message)
                return result
        
        for result in results:
            if result.status == SafetyStatus.CRITICAL:
                return result
        
        for result in results:
            if result.status == SafetyStatus.WARNING:
                return result
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="所有检查通过"
        )
    
    def check_joint_limits(self, motor_status: MotorStatus) -> SafetyCheckResult:
        """检查关节限位"""
        if motor_status is None or not motor_status.is_valid():
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.WARNING,
                message="电机状态数据无效"
            )
        
        positions = motor_status.positions
        min_limit, max_limit = self.config.position_limits
        
        violations = []
        for i, pos in enumerate(positions):
            if pos < min_limit or pos > max_limit:
                violations.append(f"电机{i}: {pos:.1f}°")
        
        if violations:
            msg = f"关节限位违规: {', '.join(violations[:3])}"
            self._add_error(msg)
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.CRITICAL,
                message=msg,
                details={"violations": violations}
            )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="关节限位正常"
        )
    
    def check_temperature(self, motor_status: MotorStatus) -> SafetyCheckResult:
        """检查电机温度"""
        if motor_status is None:
            return SafetyCheckResult(
                is_safe=True,
                status=SafetyStatus.SAFE,
                message="无温度数据"
            )
        
        temperatures = motor_status.temperatures
        max_temp = self.config.temperature_limit
        
        overheated = []
        for i, temp in enumerate(temperatures):
            if temp > max_temp:
                overheated.append(f"电机{i}: {temp:.1f}°C")
        
        if overheated:
            msg = f"电机过温: {', '.join(overheated[:3])}"
            self._add_error(msg)
            
            if any(t > max_temp + 10 for t in temperatures):
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.EMERGENCY,
                    message=msg,
                    details={"overheated": overheated}
                )
            else:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.CRITICAL,
                    message=msg,
                    details={"overheated": overheated}
                )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="温度正常"
        )
    
    def check_current(self, motor_status: MotorStatus) -> SafetyCheckResult:
        """检查电机电流"""
        if motor_status is None:
            return SafetyCheckResult(
                is_safe=True,
                status=SafetyStatus.SAFE,
                message="无电流数据"
            )
        
        currents = motor_status.currents
        max_current = self.config.current_limit
        
        overcurrent = []
        for i, current in enumerate(currents):
            if current > max_current:
                overcurrent.append(f"电机{i}: {current:.1f}A")
        
        if overcurrent:
            msg = f"电机过流: {', '.join(overcurrent[:3])}"
            self._add_error(msg)
            
            if any(c > max_current * 1.5 for c in currents):
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.EMERGENCY,
                    message=msg,
                    details={"overcurrent": overcurrent}
                )
            else:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.WARNING,
                    message=msg,
                    details={"overcurrent": overcurrent}
                )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="电流正常"
        )
    
    def check_voltage(self, power_status: PowerStatus) -> SafetyCheckResult:
        """检查电池电压"""
        if power_status is None:
            return SafetyCheckResult(
                is_safe=True,
                status=SafetyStatus.SAFE,
                message="无电压数据"
            )
        
        voltage = power_status.voltage
        
        if voltage < self.config.voltage_min:
            msg = f"电压过低: {voltage:.1f}V"
            self._add_error(msg)
            
            if voltage < self.config.voltage_min - 2:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.EMERGENCY,
                    message=msg
                )
            else:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.CRITICAL,
                    message=msg
                )
        
        if voltage > self.config.voltage_max:
            msg = f"电压过高: {voltage:.1f}V"
            self._add_error(msg)
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.WARNING,
                message=msg
            )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message=f"电压正常: {voltage:.1f}V"
        )
    
    def check_tilt_angle(self, robot_state: RobotState) -> SafetyCheckResult:
        """检查机体倾斜角度"""
        orientation = robot_state.base_orientation
        roll, pitch, yaw = orientation
        
        roll_deg = np.rad2deg(abs(roll))
        pitch_deg = np.rad2deg(abs(pitch))
        max_tilt = self.config.tilt_angle_max
        
        if roll_deg > max_tilt or pitch_deg > max_tilt:
            msg = f"机体倾斜过大: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°"
            self._add_error(msg)
            
            if roll_deg > max_tilt + 15 or pitch_deg > max_tilt + 15:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.EMERGENCY,
                    message=msg,
                    details={"roll": roll_deg, "pitch": pitch_deg}
                )
            else:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.CRITICAL,
                    message=msg,
                    details={"roll": roll_deg, "pitch": pitch_deg}
                )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message=f"姿态正常: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°"
        )
    
    def check_communication(self, robot_state: RobotState) -> SafetyCheckResult:
        """检查通信状态"""
        current_time = time.time()
        
        if robot_state.motor_status is None:
            motor_age = float('inf')
        else:
            motor_age = current_time - robot_state.motor_status.timestamp
        
        if robot_state.foot_force is None:
            force_age = float('inf')
        else:
            force_age = current_time - robot_state.foot_force.timestamp
        
        timeout = self.config.communication_timeout
        
        if motor_age > timeout or force_age > timeout:
            msg = f"通信超时: 电机数据{motor_age:.2f}s, 足端力数据{force_age:.2f}s"
            self._add_error(msg)
            
            if motor_age > timeout * 3 or force_age > timeout * 3:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.EMERGENCY,
                    message=msg
                )
            else:
                return SafetyCheckResult(
                    is_safe=False,
                    status=SafetyStatus.WARNING,
                    message=msg
                )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="通信正常"
        )
    
    def check_error_flags(self, motor_status: MotorStatus) -> SafetyCheckResult:
        """检查电机错误标志"""
        if motor_status is None:
            return SafetyCheckResult(
                is_safe=True,
                status=SafetyStatus.SAFE,
                message="无错误标志数据"
            )
        
        errors = []
        for i, flag in enumerate(motor_status.error_flags):
            if flag != 0:
                errors.append(f"电机{i}: 0x{flag:04X}")
        
        if errors:
            msg = f"电机错误: {', '.join(errors[:3])}"
            self._add_error(msg)
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.CRITICAL,
                message=msg,
                details={"errors": errors}
            )
        
        return SafetyCheckResult(
            is_safe=True,
            status=SafetyStatus.SAFE,
            message="无电机错误"
        )
    
    def safe_positions(self, positions: np.ndarray) -> np.ndarray:
        """
        安全位置限制
        
        Args:
            positions: 目标位置
        
        Returns:
            限制后的位置
        """
        return np.clip(positions, self.config.position_limits[0], self.config.position_limits[1])
    
    def smooth_action(self, new_action: np.ndarray, 
                      prev_action: np.ndarray,
                      smoothing_factor: float = 0.3) -> np.ndarray:
        """
        动作平滑
        
        Args:
            new_action: 新动作
            prev_action: 上一步动作
            smoothing_factor: 平滑因子 (0-1, 越大越平滑)
        
        Returns:
            平滑后的动作
        """
        return smoothing_factor * prev_action + (1 - smoothing_factor) * new_action
    
    def limit_velocity(self, positions: np.ndarray,
                       prev_positions: np.ndarray,
                       dt: float) -> np.ndarray:
        """
        限制速度
        
        Args:
            positions: 目标位置
            prev_positions: 上一步位置
            dt: 时间间隔
        
        Returns:
            限制速度后的位置
        """
        if dt <= 0:
            return positions
        
        max_delta = self.config.velocity_limit * dt
        delta = positions - prev_positions
        delta = np.clip(delta, -max_delta, max_delta)
        
        return prev_positions + delta
    
    def _trigger_emergency(self, reason: str):
        """触发紧急停止"""
        if not self._emergency_triggered:
            self._emergency_triggered = True
            self._emergency_reason = reason
            self._emergency_time = time.time()
            print(f"[SafetyManager] 紧急停止触发: {reason}")
            
            if self._emergency_callback:
                self._emergency_callback(reason)
    
    def can_recover(self) -> Tuple[bool, str]:
        """
        检查是否可以恢复
        
        Returns:
            (是否可以恢复, 原因说明)
        """
        if not self._emergency_triggered:
            return False, "未处于急停状态"
        
        if self._emergency_time is None:
            return False, "急停时间未知"
        
        elapsed = time.time() - self._emergency_time
        if elapsed < self._recovery_cooldown:
            return False, f"冷却中，还需等待 {self._recovery_cooldown - elapsed:.1f} 秒"
        
        if self._recovery_attempts >= self._max_recovery_attempts:
            return False, f"已达到最大恢复尝试次数 ({self._max_recovery_attempts})"
        
        return True, "可以尝试恢复"
    
    def request_recovery(self, robot_state: RobotState) -> SafetyCheckResult:
        """
        请求从急停状态恢复
        
        Args:
            robot_state: 当前机器人状态
        
        Returns:
            恢复检查结果
        """
        can_recover, msg = self.can_recover()
        if not can_recover:
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.EMERGENCY,
                message=msg
            )
        
        # 重新检查所有安全项
        result = self.check_all(robot_state)
        if result.is_safe:
            self._emergency_triggered = False
            self._emergency_reason = None
            self._emergency_time = None
            self._recovery_attempts = 0
            return SafetyCheckResult(
                is_safe=True,
                status=SafetyStatus.SAFE,
                message="急停已清除，系统恢复正常"
            )
        else:
            self._recovery_attempts += 1
            return SafetyCheckResult(
                is_safe=False,
                status=SafetyStatus.EMERGENCY,
                message=f"恢复失败 ({self._recovery_attempts}/{self._max_recovery_attempts}): {result.message}"
            )
    
    def get_emergency_info(self) -> Dict[str, Any]:
        """获取急停信息"""
        return {
            "triggered": self._emergency_triggered,
            "reason": self._emergency_reason,
            "time": self._emergency_time,
            "elapsed": time.time() - self._emergency_time if self._emergency_time else 0,
            "recovery_attempts": self._recovery_attempts,
            "can_recover": self.can_recover()[0]
        }
    
    def _add_error(self, error_msg: str):
        """添加错误到历史记录"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self._error_history.append(f"[{timestamp}] {error_msg}")
        
        if len(self._error_history) > self._max_history_size:
            self._error_history.pop(0)
    
    def get_error_history(self) -> List[str]:
        """获取错误历史"""
        return self._error_history.copy()
    
    def clear_errors(self):
        """清除错误历史"""
        self._error_history.clear()
    
    def is_emergency(self) -> bool:
        """检查是否处于紧急状态"""
        return self._emergency_triggered
    
    def reset_emergency(self):
        """重置紧急状态（手动强制重置）"""
        self._emergency_triggered = False
        self._emergency_reason = None
        self._emergency_time = None
        self._recovery_attempts = 0
        print("[SafetyManager] 紧急状态已手动重置")
    
    def get_check_count(self) -> int:
        """获取检查次数"""
        return self._check_count
