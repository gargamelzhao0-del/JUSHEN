"""
主控制循环 - 四足机器狗上位机控制
Main Controller for Quadruped Robot Control

整合所有模块，实现50Hz控制循环
"""

import numpy as np
import time
import threading
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import signal
import sys

from config import Config
from data_types import (
    MotorStatus, FootForceData, PowerStatus, IMUData,
    RobotState, Command, RobotMode
)
from tcp_client import QuadrupedTCPClient
from state_estimator import StateEstimator
from observation_processor import ObservationProcessor, ActionProcessor
from policy_runner import PolicyRunner, MockPolicyRunner
from safety_manager import SafetyManager, SafetyStatus
from imu_driver import create_imu_driver, IMUConfig


class ControllerState(Enum):
    """控制器状态"""
    IDLE = "idle"
    CONNECTING = "connecting"
    CONNECTED = "connected"  # V3.2: 新增已连接状态
    STANDING_UP = "standing_up"
    RUNNING = "running"
    EMERGENCY = "emergency"
    ERROR = "error"  # V3.2: 新增错误状态
    STOPPED = "stopped"


@dataclass
class ControllerStats:
    """控制器统计"""
    control_cycles: int = 0
    total_runtime: float = 0.0
    avg_cycle_time: float = 0.0
    max_cycle_time: float = 0.0
    min_cycle_time: float = float('inf')
    inference_time_avg: float = 0.0
    communication_time_avg: float = 0.0


class MainController:
    """主控制器"""
    
    def __init__(self, config: Optional[Config] = None):
        """
        初始化主控制器
        
        Args:
            config: 配置对象
        """
        self.config = config or Config()
        
        # V3.2: 无外设模式标志
        self._no_peripheral_mode = self.config.no_peripheral_mode
        self._no_peripheral_detected = False  # 自动检测到的无外设状态
        
        self.tcp_client = QuadrupedTCPClient(
            host=self.config.network.jetson_ip,
            port=self.config.network.tcp_port
        )
        
        self.state_estimator = StateEstimator(self.config.state_estimator)
        
        self.obs_processor = ObservationProcessor()
        self.action_processor = ActionProcessor(
            action_scale=self.config.policy.action_scale,
            action_offset=self.config.policy.action_offset,
            clip_actions=self.config.policy.clip_actions
        )
        
        self.policy_runner: Optional[PolicyRunner] = None
        
        self.safety_manager = SafetyManager(self.config.safety)
        
        self.imu_driver = create_imu_driver(
            config=IMUConfig(
                enabled=self.config.imu.enabled,
                use_ros2=self.config.imu.use_ros2,
                topic=self.config.imu.topic,
                frame_id=self.config.imu.frame_id
            ),
            mock=False
        )
        
        self._robot_state = RobotState()
        self._command = Command()
        
        self._state = ControllerState.IDLE
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        
        self._stats = ControllerStats()
        self._last_cycle_time = 0.0
        
        self._default_stance = np.ones(12) * 180.0
        
        self._on_state_change: Optional[Callable[[ControllerState], None]] = None
        self._on_cycle_complete: Optional[Callable[[RobotState], None]] = None
        
        self._setup_callbacks()
    
    def _setup_callbacks(self):
        """设置回调函数"""
        self.tcp_client.on_motor_status = self._on_motor_status
        self.tcp_client.on_foot_force = self._on_foot_force
        self.tcp_client.on_power_status = self._on_power_status
        self.tcp_client.on_connected = self._on_tcp_connected
        self.tcp_client.on_disconnected = self._on_tcp_disconnected
        
        self.safety_manager.set_emergency_callback(self._on_emergency)
    
    def load_policy(self, model_path: str, model_type: str = "pytorch") -> bool:
        """
        加载RL策略模型
        
        Args:
            model_path: 模型路径
            model_type: 模型类型
        
        Returns:
            是否加载成功
        """
        self.policy_runner = PolicyRunner(
            model_path=model_path,
            model_type=model_type,
            action_dim=self.config.policy.action_dim,
            observation_dim=self.config.policy.observation_dim,
            action_scale=self.config.policy.action_scale,
            action_offset=self.config.policy.action_offset,
        )
        
        if self.policy_runner.is_ready():
            print(f"[MainController] 策略模型加载成功: {model_path}")
            return True
        else:
            print(f"[MainController] 策略模型加载失败")
            return False
    
    def use_mock_policy(self):
        """使用模拟策略（用于测试）"""
        self.policy_runner = MockPolicyRunner(
            action_dim=self.config.policy.action_dim,
            observation_dim=self.config.policy.observation_dim,
            gait_frequency=1.0,
            amplitude=0.3
        )
        print("[MainController] 使用模拟策略")
    
    def start(self, wait_for_connection: bool = True, timeout: float = 30.0) -> bool:
        """
        启动控制器
        
        Args:
            wait_for_connection: 是否等待连接
            timeout: 连接超时时间
        
        Returns:
            是否启动成功
        """
        if self._running:
            print("[MainController] 控制器已在运行")
            return True
        
        self._set_state(ControllerState.CONNECTING)
        
        self.imu_driver.start()
        
        if wait_for_connection:
            if not self.tcp_client.start_server(timeout):
                print("[MainController] 连接失败，使用独立模式")
        else:
            print("[MainController] 测试模式，不等待STM32连接")
        
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        print(f"[MainController] 控制器已启动 (频率: {self.config.control.control_frequency}Hz)")
        return True
    
    def stop(self):
        """停止控制器"""
        if not self._running:
            return
        
        self._running = False
        
        self.imu_driver.stop()
        
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        
        self.tcp_client.stop()
        self._set_state(ControllerState.STOPPED)
        
        print("[MainController] 控制器已停止")
    
    def _control_loop(self):
        """控制循环"""
        control_period = 1.0 / self.config.control.control_frequency
        
        print("[MainController] 控制循环开始")
        
        startup_count = 0
        startup_cycles = int(2.0 / control_period)
        startup_start_time = time.time()  # V3.2: 添加启动超时检测
        STARTUP_TIMEOUT = 30.0  # 30秒超时
        last_state = self._state  # V3.2: 追踪状态变化
        last_connected = False  # V3.2: 追踪连接状态变化
        consecutive_errors = 0  # V3.2: 连续错误计数
        MAX_CONSECUTIVE_ERRORS = 10  # V3.2: 最大连续错误数
        
        while self._running:
            cycle_start = time.perf_counter()
            
            # V3.2: 检测连接状态变化
            current_connected = self.tcp_client.connected
            if current_connected != last_connected:
                if current_connected:
                    print("[MainController] 连接已恢复，重置启动序列")
                    startup_count = 0
                    startup_start_time = time.time()
                    # 重置无外设检测状态
                    self._no_peripheral_detected = False
                    # V3.2: 如果当前是ERROR状态，重置为IDLE以允许重新启动
                    if self._state == ControllerState.ERROR:
                        print("[MainController] 从ERROR状态恢复，重置为IDLE")
                        self._set_state(ControllerState.IDLE)
                else:
                    print("[MainController] 连接已断开，等待重连...")
                    self._set_state(ControllerState.IDLE)
                last_connected = current_connected
            
            if not current_connected:
                time.sleep(0.1)
                continue
            
            try:
                # V3.2: ERROR状态下等待用户干预
                if self._state == ControllerState.ERROR:
                    time.sleep(0.1)
                    last_state = self._state
                    continue
                
                # V3.2: 检测从ERROR恢复，重置启动序列
                if last_state == ControllerState.ERROR and self._state == ControllerState.IDLE:
                    print("[MainController] 检测到状态恢复，重新开始启动序列")
                    startup_count = 0
                    startup_start_time = time.time()
                last_state = self._state
                
                if startup_count < startup_cycles:
                    # V3.2: 检查启动超时
                    elapsed = time.time() - startup_start_time
                    if elapsed > STARTUP_TIMEOUT:
                        print(f"[MainController] 启动超时 ({elapsed:.1f}s)，进入错误状态")
                        self._set_state(ControllerState.ERROR)
                        # 重置启动时间，允许用户重试
                        startup_start_time = time.time()
                        time.sleep(1.0)  # 等待1秒后重试
                        continue
                    
                    result = self._startup_sequence(startup_count, startup_cycles)
                    # V3.1: 只有当启动序列返回True时才推进进度
                    if result:
                        startup_count += 1
                    else:
                        # 误差过大时暂停，但不重置计数器
                        pass
                else:
                    self._set_state(ControllerState.RUNNING)
                    self._execute_control_cycle()
                
            except Exception as e:
                import traceback
                print(f"[MainController] 控制周期错误: {e}")
                print(f"[MainController] 错误堆栈:\n{traceback.format_exc()}")
                
                # V3.2: 连续错误计数和处理
                consecutive_errors += 1
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    print(f"[MainController] 连续错误次数达到 {consecutive_errors}，进入ERROR状态")
                    self._set_state(ControllerState.ERROR)
                    consecutive_errors = 0
            else:
                # V3.2: 成功执行后重置错误计数
                consecutive_errors = 0
            
            self._stats.control_cycles += 1
            
            cycle_time = time.perf_counter() - cycle_start
            self._update_cycle_stats(cycle_time)
            
            sleep_time = control_period - cycle_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("[MainController] 控制循环结束")
    
    def _startup_sequence(self, current: int, total: int):
        """启动序列"""
        self._set_state(ControllerState.STANDING_UP)
        
        progress = current / total
        target_positions = self._default_stance.copy()
        
        blend = self._smooth_blend(progress)
        
        positions = self._default_stance.copy()
        
        if self.tcp_client.send_positions(positions):
            if current % 50 == 0:
                print(f"[MainController] 启动中... {progress*100:.0f}%")
                
                motor_status = self.tcp_client.get_motor_status()
                
                # V3.2: 检测无外设模式，跳过电机位置误差检查
                if self._check_no_peripheral_mode(motor_status):
                    print("[MainController] 无外设模式：跳过电机位置验证")
                    return True
                
                if motor_status and motor_status.is_valid():
                    errors = np.abs(np.array(motor_status.positions) - target_positions)
                    max_error = np.max(errors)
                    avg_error = np.mean(errors)
                    
                    # V3.1: 误差过大时的处理
                    if max_error > 30.0:
                        print(f"[MainController] 严重: 电机位置误差过大! 最大误差: {max_error:.1f}°")
                        print(f"[MainController] 暂停启动，等待电机调整...")
                        # 返回False表示暂停启动进度
                        return False
                    elif max_error > 15.0:
                        print(f"[MainController] 警告: 电机位置误差过大! 最大误差: {max_error:.1f}°, 平均误差: {avg_error:.1f}°")
                        print(f"[MainController] 减慢启动速度...")
                        # 返回False表示减慢启动进度
                        return False
                    elif max_error > 5.0:
                        print(f"[MainController] 电机位置调整中... 最大误差: {max_error:.1f}°")
                    else:
                        print(f"[MainController] 电机位置正常, 最大误差: {max_error:.1f}°")
                else:
                    print("[MainController] 等待电机反馈...")
        
        return True
    
    def _smooth_blend(self, t: float) -> float:
        """平滑过渡函数"""
        return t * t * (3 - 2 * t)
    
    def _execute_control_cycle(self):
        """执行控制周期"""
        motor_status = self.tcp_client.get_motor_status()
        foot_force = self.tcp_client.get_foot_force()
        power_status = self.tcp_client.get_power_status()
        imu_data = self.imu_driver.imu_data
        
        if motor_status:
            self._robot_state.motor_status = motor_status
        if foot_force:
            self._robot_state.foot_force = foot_force
        if power_status:
            self._robot_state.power_status = power_status
        if imu_data:
            self._robot_state.imu_data = imu_data
        
        self._robot_state.update_timestamp()
        
        if motor_status is None:
            return
        
        # V3.2: 检测无外设模式（电机反馈全为0）
        no_peripheral = self._check_no_peripheral_mode(motor_status)
        if no_peripheral:
            # 无外设模式：跳过安全检查和策略推理，发送默认位置
            if self._stats.control_cycles % 50 == 0:  # 每1秒打印一次
                print("[MainController] 无外设模式：跳过安全检查")
            self.tcp_client.send_positions(self._default_stance)
            return
        
        estimate = self.state_estimator.update(
            motor_status, 
            foot_force if foot_force else self._robot_state.foot_force,
            self._robot_state.imu_data
        )
        self._robot_state.base_orientation = estimate.base_orientation
        self._robot_state.base_velocity = estimate.base_velocity
        
        safety_result = self.safety_manager.check_all(self._robot_state)
        if not safety_result.is_safe:
            print(f"[MainController] 安全检查: {safety_result.message}")
            if safety_result.status == SafetyStatus.EMERGENCY:
                # V3.1: 自动尝试恢复
                can_recover, msg = self.safety_manager.can_recover()
                if can_recover:
                    print(f"[MainController] 尝试自动恢复...")
                    recovery_result = self.safety_manager.request_recovery(self._robot_state)
                    if recovery_result.is_safe:
                        print(f"[MainController] 自动恢复成功")
                        self._set_state(ControllerState.RUNNING)
                    else:
                        print(f"[MainController] 自动恢复失败: {recovery_result.message}")
                return
        
        if self.policy_runner is None or not self.policy_runner.is_ready():
            return
        
        observation = self.obs_processor.process(self._robot_state, self._command)
        
        action = self.policy_runner.infer(observation)
        
        self.obs_processor.update_last_action(action)
        
        positions = self.policy_runner.action_to_positions(action)
        
        positions = self.safety_manager.safe_positions(positions)
        
        if self.config.control.smoothing_enabled and self._robot_state.motor_status is not None:
            prev_positions = self._robot_state.motor_status.positions
            positions = self.safety_manager.smooth_action(
                positions, prev_positions, self.config.control.smoothing_factor
            )
        
        self.tcp_client.send_positions(positions)
        
        if self._on_cycle_complete:
            self._on_cycle_complete(self._robot_state)
    
    def _update_cycle_stats(self, cycle_time: float):
        """更新周期统计"""
        self._stats.total_runtime += cycle_time
        n = self._stats.control_cycles
        self._stats.avg_cycle_time = self._stats.total_runtime / n
        self._stats.max_cycle_time = max(self._stats.max_cycle_time, cycle_time)
        self._stats.min_cycle_time = min(self._stats.min_cycle_time, cycle_time)
    
    def _on_motor_status(self, status: MotorStatus):
        """电机状态回调"""
        pass
    
    def _on_foot_force(self, force: FootForceData):
        """足端力回调"""
        pass
    
    def _on_power_status(self, status: PowerStatus):
        """电源状态回调"""
        if status.is_low_voltage():
            print(f"[MainController] 低电压警告: {status.voltage:.1f}V")
    
    def _on_tcp_connected(self):
        """TCP连接回调"""
        print("[MainController] TCP已连接")
        self._set_state(ControllerState.CONNECTED)
        
        # V3.2: 重置无外设检测状态，允许重新检测
        self._no_peripheral_detected = False
        
        # 发送电机使能命令
        self.tcp_client.enable_motors()
        self.tcp_client.enable_data_send(True)
        
        print("[MainController] 已发送电机使能命令")
    
    def _on_tcp_disconnected(self):
        """TCP断开回调"""
        print("[MainController] TCP已断开，等待自动重连...")
        self._set_state(ControllerState.IDLE)
        
        # V3.2: 清理状态
        self._robot_state.motor_status = None
        self._robot_state.foot_force = None
    
    def _on_emergency(self, reason: str):
        """紧急停止回调"""
        print(f"[MainController] 紧急停止: {reason}")
        self._set_state(ControllerState.EMERGENCY)
        self.tcp_client.emergency_stop()
    
    def _set_state(self, state: ControllerState):
        """设置控制器状态"""
        if self._state != state:
            self._state = state
            if self._on_state_change:
                self._on_state_change(state)
    
    def set_command(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0):
        """
        设置速度命令
        
        Args:
            vx: 前进速度
            vy: 横向速度
            vyaw: 旋转速度
        """
        self._command.vx = vx
        self._command.vy = vy
        self._command.vyaw = vyaw
        self._command.timestamp = time.time()
    
    def set_mode(self, mode: RobotMode):
        """设置机器人模式"""
        self._robot_state.mode = mode
    
    def emergency_stop(self):
        """手动触发紧急停止"""
        self.safety_manager._trigger_emergency("用户触发")
    
    def reset_emergency(self):
        """重置紧急状态"""
        self.safety_manager.reset_emergency()
        self._set_state(ControllerState.RUNNING)
    
    def reset_error_and_retry(self):
        """V3.2: 重置ERROR状态并重新尝试启动"""
        if self._state == ControllerState.ERROR:
            print("[MainController] 重置ERROR状态，准备重新启动")
            self._set_state(ControllerState.IDLE)
            return True
        return False
    
    def _check_no_peripheral_mode(self, motor_status) -> bool:
        """
        V3.2: 检测无外设模式
        
        当电机反馈全为0时，说明CAN总线上没有电机响应
        此时应该跳过安全检查和策略推理，避免触发错误
        """
        # 如果配置中已启用无外设模式，直接返回True
        if self._no_peripheral_mode:
            return True
        
        if motor_status is None:
            return True
        
        # 检查所有电机位置是否都为0
        positions = motor_status.positions
        if positions is not None and len(positions) == 12:
            all_zero = np.allclose(positions, 0.0, atol=1e-3)
            if all_zero:
                # 进一步检查速度和电流是否也为0
                velocities = motor_status.velocities
                currents = motor_status.currents
                if velocities is not None and currents is not None:
                    velocities_zero = np.allclose(velocities, 0.0, atol=1e-3)
                    currents_zero = np.allclose(currents, 0.0, atol=1e-3)
                    if velocities_zero and currents_zero:
                        # 自动检测到无外设，打印提示（仅首次）
                        if not self._no_peripheral_detected:
                            print("[MainController] 自动检测到无外设模式（电机反馈全为0）")
                            self._no_peripheral_detected = True
                        return True
        
        return False
    
    def get_robot_state(self) -> RobotState:
        """获取机器人状态"""
        return self._robot_state
    
    def get_stats(self) -> ControllerStats:
        """获取统计信息"""
        return self._stats
    
    def get_state(self) -> ControllerState:
        """获取控制器状态"""
        return self._state
    
    def is_running(self) -> bool:
        """检查是否运行中"""
        return self._running and self._state == ControllerState.RUNNING
    
    def set_state_change_callback(self, callback: Callable[[ControllerState], None]):
        """设置状态变化回调"""
        self._on_state_change = callback
    
    def set_cycle_complete_callback(self, callback: Callable[[RobotState], None]):
        """设置周期完成回调"""
        self._on_cycle_complete = callback


def run_controller(config: Optional[Config] = None,
                   model_path: str = "",
                   use_mock: bool = False,
                   wait_for_connection: bool = True):
    """
    运行控制器（便捷函数）
    
    Args:
        config: 配置对象
        model_path: 模型路径
        use_mock: 是否使用模拟策略
        wait_for_connection: 是否等待STM32连接
    """
    controller = MainController(config)
    
    def signal_handler(sig, frame):
        print("\n[MainController] 接收到退出信号")
        controller.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if use_mock or not model_path:
        controller.use_mock_policy()
    else:
        if not controller.load_policy(model_path):
            print("[MainController] 模型加载失败，使用模拟策略")
            controller.use_mock_policy()
    
    if controller.start(wait_for_connection=wait_for_connection, timeout=30.0):
        print("[MainController] 控制器运行中，按Ctrl+C退出")
        
        while controller._running:
            time.sleep(1.0)
            
            stats = controller.get_stats()
            state = controller.get_state()
            print(f"[MainController] 状态: {state.value} | "
                  f"周期: {stats.control_cycles} | "
                  f"平均时间: {stats.avg_cycle_time*1000:.2f}ms")
    else:
        print("[MainController] 启动失败")
