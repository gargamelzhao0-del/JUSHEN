"""
RL策略推理模块 - 四足机器狗上位机控制
Policy Runner for Quadruped Robot Control

支持PyTorch、ONNX、TensorRT模型推理
"""

import numpy as np
from typing import Optional, Union, Dict, Any
from dataclasses import dataclass
from enum import Enum
import time
import os


class ModelType(Enum):
    """模型类型"""
    PYTORCH = "pytorch"
    ONNX = "onnx"
    TENSORRT = "tensorrt"
    MOCK = "mock"


@dataclass
class InferenceStats:
    """推理统计"""
    total_inferences: int = 0
    total_time_ms: float = 0.0
    avg_time_ms: float = 0.0
    max_time_ms: float = 0.0
    min_time_ms: float = float('inf')


class PolicyRunner:
    """RL策略推理器"""
    
    def __init__(self, 
                 model_path: str = "",
                 model_type: str = "pytorch",
                 action_dim: int = 12,
                 observation_dim: int = 48,
                 action_scale: np.ndarray = None,
                 action_offset: np.ndarray = None,
                 clip_actions: bool = True):
        """
        初始化策略推理器
        
        Args:
            model_path: 模型文件路径
            model_type: 模型类型 (pytorch/onnx/tensorrt/mock)
            action_dim: 动作维度
            observation_dim: 观测维度
            action_scale: 动作缩放因子
            action_offset: 动作偏移
            clip_actions: 是否裁剪动作
        """
        self.model_path = model_path
        self.model_type = ModelType(model_type.lower())
        self.action_dim = action_dim
        self.observation_dim = observation_dim
        
        self.action_scale = action_scale if action_scale is not None else np.ones(action_dim) * 0.25
        self.action_offset = action_offset if action_offset is not None else np.ones(action_dim) * 180.0
        self.clip_actions = clip_actions
        
        self.model = None
        self.session = None
        self.is_loaded = False
        
        self._last_action = np.zeros(action_dim, dtype=np.float32)
        self._stats = InferenceStats()
        
        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
    
    def load_model(self, model_path: str) -> bool:
        """
        加载模型
        
        Args:
            model_path: 模型文件路径
        
        Returns:
            是否加载成功
        """
        try:
            if self.model_type == ModelType.PYTORCH:
                return self._load_pytorch(model_path)
            elif self.model_type == ModelType.ONNX:
                return self._load_onnx(model_path)
            elif self.model_type == ModelType.TENSORRT:
                return self._load_tensorrt(model_path)
            elif self.model_type == ModelType.MOCK:
                return self._load_mock()
            else:
                print(f"[PolicyRunner] 不支持的模型类型: {self.model_type}")
                return False
        except Exception as e:
            print(f"[PolicyRunner] 加载模型失败: {e}")
            return False
    
    def _load_pytorch(self, model_path: str) -> bool:
        """加载PyTorch模型"""
        try:
            import torch
            
            if model_path.endswith('.pt') or model_path.endswith('.pth'):
                self.model = torch.jit.load(model_path)
            else:
                self.model = torch.load(model_path)
            
            self.model.eval()
            self.is_loaded = True
            print(f"[PolicyRunner] PyTorch模型加载成功: {model_path}")
            return True
        except ImportError:
            print("[PolicyRunner] PyTorch未安装，尝试使用ONNX")
            return False
        except Exception as e:
            print(f"[PolicyRunner] 加载PyTorch模型失败: {e}")
            return False
    
    def _load_onnx(self, model_path: str) -> bool:
        """加载ONNX模型"""
        try:
            import onnxruntime as ort
            
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, providers=providers)
            
            input_name = self.session.get_inputs()[0].name
            output_name = self.session.get_outputs()[0].name
            
            self.is_loaded = True
            print(f"[PolicyRunner] ONNX模型加载成功: {model_path}")
            return True
        except ImportError:
            print("[PolicyRunner] onnxruntime未安装")
            return False
        except Exception as e:
            print(f"[PolicyRunner] 加载ONNX模型失败: {e}")
            return False
    
    def _load_tensorrt(self, model_path: str) -> bool:
        """加载TensorRT模型"""
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit
            
            print("[PolicyRunner] TensorRT支持待实现")
            return False
        except ImportError:
            print("[PolicyRunner] TensorRT未安装")
            return False
    
    def _load_mock(self) -> bool:
        """加载模拟模型（用于测试）"""
        self.is_loaded = True
        print("[PolicyRunner] Mock模型已加载（测试模式）")
        return True
    
    def infer(self, observation: np.ndarray) -> np.ndarray:
        """
        执行推理
        
        Args:
            observation: 观测向量
        
        Returns:
            动作向量
        """
        if not self.is_loaded:
            return self._default_action()
        
        start_time = time.perf_counter()
        
        try:
            if self.model_type == ModelType.PYTORCH:
                action = self._infer_pytorch(observation)
            elif self.model_type == ModelType.ONNX:
                action = self._infer_onnx(observation)
            elif self.model_type == ModelType.TENSORRT:
                action = self._infer_tensorrt(observation)
            elif self.model_type == ModelType.MOCK:
                action = self._infer_mock(observation)
            else:
                action = self._default_action()
        except Exception as e:
            print(f"[PolicyRunner] 推理失败: {e}")
            action = self._last_action.copy()
        
        inference_time = (time.perf_counter() - start_time) * 1000
        self._update_stats(inference_time)
        
        if self.clip_actions:
            action = np.clip(action, -1.0, 1.0)
        
        self._last_action = action.copy()
        
        return action
    
    def _infer_pytorch(self, observation: np.ndarray) -> np.ndarray:
        """PyTorch推理"""
        import torch
        
        obs_tensor = torch.from_numpy(observation).float().unsqueeze(0)
        
        with torch.no_grad():
            action_tensor = self.model(obs_tensor)
        
        action = action_tensor.squeeze(0).numpy()
        return action.astype(np.float32)
    
    def _infer_onnx(self, observation: np.ndarray) -> np.ndarray:
        """ONNX推理"""
        input_name = self.session.get_inputs()[0].name
        output_name = self.session.get_outputs()[0].name
        
        obs_batch = observation.reshape(1, -1).astype(np.float32)
        
        outputs = self.session.run([output_name], {input_name: obs_batch})
        
        action = outputs[0].squeeze(0)
        return action.astype(np.float32)
    
    def _infer_tensorrt(self, observation: np.ndarray) -> np.ndarray:
        """TensorRT推理"""
        return self._default_action()
    
    def _infer_mock(self, observation: np.ndarray) -> np.ndarray:
        """模拟推理（用于测试）"""
        action = np.sin(time.time() * 2 * np.pi * 1.0) * 0.1 * np.ones(self.action_dim)
        return action.astype(np.float32)
    
    def _default_action(self) -> np.ndarray:
        """默认动作"""
        return np.zeros(self.action_dim, dtype=np.float32)
    
    def _update_stats(self, inference_time_ms: float):
        """更新统计信息"""
        self._stats.total_inferences += 1
        self._stats.total_time_ms += inference_time_ms
        self._stats.avg_time_ms = self._stats.total_time_ms / self._stats.total_inferences
        self._stats.max_time_ms = max(self._stats.max_time_ms, inference_time_ms)
        self._stats.min_time_ms = min(self._stats.min_time_ms, inference_time_ms)
    
    def get_stats(self) -> InferenceStats:
        """获取推理统计"""
        return self._stats
    
    def get_last_action(self) -> np.ndarray:
        """获取上一步动作"""
        return self._last_action.copy()
    
    def action_to_positions(self, action: np.ndarray) -> np.ndarray:
        """
        将动作转换为关节位置
        
        Args:
            action: 动作向量 (-1 到 1)
        
        Returns:
            关节位置（度）
        """
        positions = action * self.action_scale + self.action_offset
        return positions.astype(np.float32)
    
    def reset(self):
        """重置策略"""
        self._last_action = np.zeros(self.action_dim, dtype=np.float32)
        self._stats = InferenceStats()
        print("[PolicyRunner] 已重置")
    
    def is_ready(self) -> bool:
        """检查是否就绪"""
        return self.is_loaded


class MockPolicyRunner(PolicyRunner):
    """模拟策略推理器（用于测试）"""
    
    def __init__(self, 
                 action_dim: int = 12,
                 observation_dim: int = 48,
                 gait_frequency: float = 1.0,
                 amplitude: float = 0.3):
        super().__init__(
            model_path="",
            model_type="mock",
            action_dim=action_dim,
            observation_dim=observation_dim
        )
        self.gait_frequency = gait_frequency
        self.amplitude = amplitude
        self._phase = 0.0
        
        self._phase_offsets = np.array([
            0.0, 0.0, 0.0,
            np.pi, np.pi, np.pi,
            np.pi, np.pi, np.pi,
            0.0, 0.0, 0.0,
        ])
    
    def _infer_mock(self, observation: np.ndarray) -> np.ndarray:
        """生成Trot步态"""
        action = np.zeros(self.action_dim, dtype=np.float32)
        
        for i in range(self.action_dim):
            phase = self._phase + self._phase_offsets[i]
            action[i] = self.amplitude * np.sin(2 * np.pi * phase)
        
        self._phase += self.gait_frequency / 50.0
        if self._phase >= 1.0:
            self._phase -= 1.0
        
        return action
