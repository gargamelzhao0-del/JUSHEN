"""
TCP通信模块 - 四足机器狗上位机控制
TCP Communication Module for Quadruped Robot Control

基于STM32下位机通信协议实现
V5.0: 永久运行 + 日志文件输出
"""

import socket
import struct
import threading
import time
import os
import numpy as np
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from datetime import datetime

from data_types import (
    CommandType, ResponseType,
    MotorStatus, FootForceData, PowerStatus,
)


MAX_BUFFER_SIZE = 65536


class FileLogger:
    """V5.0: 日志系统 - 同时输出到控制台和.md文件"""

    def __init__(self, log_dir: str = "logs"):
        self._log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_dir)
        os.makedirs(self._log_dir, exist_ok=True)
        self._log_path = os.path.join(self._log_dir, "tcp_log.md")
        self._lock = threading.Lock()
        # 每次启动覆盖旧日志
        with open(self._log_path, 'w', encoding='utf-8') as f:
            f.write(f"# TCP通信日志\n\n")
            f.write(f"**启动时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n---\n\n")

    def log(self, msg: str):
        """输出日志到控制台和文件"""
        ts = datetime.now().strftime('%H:%M:%S')
        console_msg = f"[{ts}] {msg}"
        file_msg = f"**{ts}** {msg}  "
        print(console_msg)
        with self._lock:
            try:
                with open(self._log_path, 'a', encoding='utf-8') as f:
                    f.write(file_msg + "\n")
            except Exception:
                pass

    def log_block(self, lines: list):
        """输出多行日志块到文件和控制台"""
        ts = datetime.now().strftime('%H:%M:%S')
        with self._lock:
            try:
                with open(self._log_path, 'a', encoding='utf-8') as f:
                    f.write(f"\n**{ts}**\n\n")
                    for line in lines:
                        f.write(f"{line}  \n")
                    f.write("\n")
            except Exception:
                pass
        for line in lines:
            print(line)


# 全局日志实例
_logger: Optional[FileLogger] = None


def get_logger() -> FileLogger:
    """获取全局日志实例"""
    global _logger
    if _logger is None:
        _logger = FileLogger()
    return _logger


@dataclass
class TCPStats:
    """TCP统计信息"""
    packets_sent: int = 0
    packets_received: int = 0
    bytes_sent: int = 0
    bytes_received: int = 0
    errors: int = 0
    last_receive_time: float = 0.0


class ProtocolCodec:
    """协议编解码器"""

    FRAME_HEADER = bytes([0xAA, 0x55])
    FRAME_TAIL = bytes([0x0D, 0x0A])

    @staticmethod
    def calculate_crc16(data: bytes) -> int:
        """计算CRC16校验和"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def encode_command(cmd_type: int, payload: bytes = b'') -> bytes:
        """编码命令帧"""
        length = len(payload)
        frame = ProtocolCodec.FRAME_HEADER
        frame += bytes([cmd_type])
        frame += struct.pack('<H', length)
        frame += payload
        crc = ProtocolCodec.calculate_crc16(frame)
        frame += struct.pack('<H', crc)
        frame += ProtocolCodec.FRAME_TAIL
        return frame

    @staticmethod
    def decode_response(data: bytes) -> Optional[Dict[str, Any]]:
        """解码响应帧"""
        if len(data) < 9:
            return None

        if data[0:2] != ProtocolCodec.FRAME_HEADER:
            return None

        if data[-2:] != ProtocolCodec.FRAME_TAIL:
            return None

        resp_type = data[2]
        length = struct.unpack('<H', data[3:5])[0]
        payload = data[5:5+length]
        crc_received = struct.unpack('<H', data[5+length:7+length])[0]

        crc_calc = ProtocolCodec.calculate_crc16(data[:5+length])
        if crc_calc != crc_received:
            return None

        return {
            'type': resp_type,
            'length': length,
            'payload': payload,
        }

    @staticmethod
    def parse_motor_status(payload: bytes) -> Optional[MotorStatus]:
        """
        解析电机状态数据包 (V3.0格式)

        payload = 192字节(电机数据) + 4字节(时间戳) = 196字节
        """
        EXPECTED_PAYLOAD_SIZE = 196

        if len(payload) < EXPECTED_PAYLOAD_SIZE:
            get_logger().log(f"[Protocol] 电机状态数据长度不足: {len(payload)} < {EXPECTED_PAYLOAD_SIZE}")
            return None

        try:
            idx = 0
            positions = np.zeros(12, dtype=np.float32)
            velocities = np.zeros(12, dtype=np.float32)
            torques = np.zeros(12, dtype=np.float32)
            motor_timestamps = np.zeros(12, dtype=np.float32)

            for i in range(12):
                positions[i] = struct.unpack('<f', payload[idx:idx+4])[0]
                idx += 4
                velocities[i] = struct.unpack('<f', payload[idx:idx+4])[0]
                idx += 4
                torques[i] = struct.unpack('<f', payload[idx:idx+4])[0]
                idx += 4
                motor_ts = struct.unpack('<I', payload[idx:idx+4])[0]
                motor_timestamps[i] = motor_ts / 1000.0
                idx += 4

            timestamp = struct.unpack('<I', payload[idx:idx+4])[0]

            return MotorStatus(
                positions=positions,
                velocities=velocities,
                currents=torques,
                temperatures=np.zeros(12, dtype=np.float32),
                error_flags=np.zeros(12, dtype=np.uint16),
                timestamp=timestamp / 1000.0,
            )
        except Exception as e:
            get_logger().log(f"[Protocol] 解析电机状态失败: {e}")
            return None

    @staticmethod
    def parse_motor_status_enhanced(payload: bytes) -> Optional[Dict[str, Any]]:
        """解析增强电机状态数据包"""
        if len(payload) < 22:
            return None

        try:
            timestamp_ms = struct.unpack('<I', payload[0:4])[0]
            motor_id = payload[4]
            position = struct.unpack('<f', payload[6:10])[0]
            velocity = struct.unpack('<f', payload[10:14])[0]
            torque = struct.unpack('<f', payload[14:18])[0]
            current = struct.unpack('<f', payload[18:22])[0]

            return {
                'timestamp': timestamp_ms / 1000.0,
                'motor_id': motor_id,
                'position': position,
                'velocity': velocity,
                'torque': torque,
                'current': current,
            }
        except Exception as e:
            get_logger().log(f"[Protocol] 解析增强电机状态失败: {e}")
            return None

    @staticmethod
    def parse_foot_force(payload: bytes) -> Optional[FootForceData]:
        """解析足端力数据包"""
        if len(payload) < 20:
            return None

        try:
            forces = np.array(struct.unpack('<4f', payload[0:16]))
            timestamp = struct.unpack('<I', payload[16:20])[0]

            contact_mask = 0
            for i, force in enumerate(forces):
                if abs(force) > 5.0:
                    contact_mask |= (1 << i)

            return FootForceData(
                forces=forces,
                contact_mask=contact_mask,
                timestamp=timestamp / 1000.0,
            )
        except Exception as e:
            get_logger().log(f"[Protocol] 解析足端力失败: {e}")
            return None

    @staticmethod
    def parse_power_status(payload: bytes) -> Optional[PowerStatus]:
        """解析电源状态数据包"""
        if len(payload) < 22:
            return None

        try:
            voltage = struct.unpack('<f', payload[0:4])[0]
            current = struct.unpack('<f', payload[4:8])[0]
            temperature = struct.unpack('<f', payload[8:12])[0]
            capacity = struct.unpack('<f', payload[12:16])[0]
            status = payload[16]
            is_charging = bool(payload[17])
            timestamp = struct.unpack('<I', payload[18:22])[0]

            return PowerStatus(
                voltage=voltage,
                current=current,
                temperature=temperature,
                capacity=capacity,
                status=status,
                is_charging=is_charging,
                timestamp=timestamp / 1000.0,
            )
        except Exception as e:
            get_logger().log(f"[Protocol] 解析电源状态失败: {e}")
            return None


class QuadrupedTCPClient:
    """四足机器狗TCP服务端 - 永久运行，等待STM32连接"""

    def __init__(self, host: str = '0.0.0.0', port: int = 9999):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self._connected = False
        self._running = False

        self.rx_thread: Optional[threading.Thread] = None
        self.rx_buffer = bytearray()

        self.motor_status: Optional[MotorStatus] = None
        self.foot_force: Optional[FootForceData] = None
        self.power_status: Optional[PowerStatus] = None

        self.on_motor_status: Optional[Callable[[MotorStatus], None]] = None
        self.on_foot_force: Optional[Callable[[FootForceData], None]] = None
        self.on_power_status: Optional[Callable[[PowerStatus], None]] = None
        self.on_connected: Optional[Callable[[], None]] = None
        self.on_disconnected: Optional[Callable[[], None]] = None

        self.lock = threading.Lock()
        self._state_lock = threading.Lock()
        self.stats = TCPStats()

        # 心跳
        self._last_heartbeat_time = time.time()
        self._heartbeat_count = 0
        self._last_heartbeat_print_time = 0.0
        self._heartbeat_print_interval = 3.0

        # 数据汇总
        self._motor_frame_count = 0
        self._foot_force_frame_count = 0
        self._last_summary_time = time.time()
        self._summary_interval = 5.0

        self._log = get_logger().log
        self._log_block = get_logger().log_block

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @connected.setter
    def connected(self, value: bool):
        with self._state_lock:
            self._connected = value

    @property
    def running(self) -> bool:
        with self._state_lock:
            return self._running

    @running.setter
    def running(self, value: bool):
        with self._state_lock:
            self._running = value

    def start_server_permanent(self) -> bool:
        """
        V5.0: 永久启动TCP服务器，断开后自动等待重连
        服务器一直运行，直到调用 stop()
        """
        self.running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        self._log(f"[TCP] 服务器永久启动于 {self.host}:{self.port}，等待STM32连接...")
        return True

    def _accept_loop(self):
        """V5.0: 永久接受连接循环 — 断开后重新等待"""
        while self.running:
            server_socket = None
            try:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except Exception:
                    pass
                server_socket.bind((self.host, self.port))
                server_socket.listen(1)
                server_socket.settimeout(1.0)
                self.server_socket = server_socket

                # 等待连接
                while self.running:
                    try:
                        client_socket, addr = server_socket.accept()
                        break
                    except socket.timeout:
                        continue

                if not self.running:
                    break

                self._log(f"[TCP] STM32已连接: {addr}")

                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                client_socket.settimeout(0.1)

                with self._state_lock:
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except Exception:
                            pass
                    self.client_socket = client_socket
                    self._connected = True

                self.rx_buffer = bytearray()
                self._heartbeat_count = 0
                self._last_heartbeat_time = time.time()
                self._last_heartbeat_print_time = time.time()
                self._last_summary_time = time.time()
                self._motor_frame_count = 0
                self._foot_force_frame_count = 0

                # 启动接收线程
                self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
                self.rx_thread.start()

                if self.on_connected:
                    self.on_connected()

                # 等待接收线程结束（即连接断开）
                if self.rx_thread:
                    self.rx_thread.join()

                self._log("[TCP] STM32断开连接，重新等待连接...")

            except OSError as e:
                if self.running:
                    # Windows: errno 10048 = WSAEADDRINUSE
                    if hasattr(e, 'winerror') and e.winerror == 10048:
                        self._log(f"[TCP] 端口 {self.port} 被占用，等待释放...")
                    elif hasattr(e, 'errno') and e.errno == 98:
                        self._log(f"[TCP] 端口 {self.port} 被占用，等待释放...")
                    else:
                        self._log(f"[TCP] 连接错误: {e}")
                    time.sleep(1.0)
            except Exception as e:
                if self.running:
                    self._log(f"[TCP] 连接错误: {e}")
                    time.sleep(1.0)
            finally:
                if server_socket:
                    try:
                        server_socket.close()
                    except Exception:
                        pass

    def _receive_loop(self):
        """接收数据循环"""
        _debug_recv_count = 0
        while self.running and self.connected:
            try:
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    break

                self.rx_buffer.extend(chunk)
                with self._state_lock:
                    self.stats.bytes_received += len(chunk)

                # 调试: 检查收到的原始数据中是否有心跳文本
                _debug_recv_count += 1
                if _debug_recv_count <= 100 or b'HEARTBEAT' in chunk or b'PING' in chunk:
                    has_hb = b'HEARTBEAT' in chunk
                    has_ping = b'PING' in chunk
                    if has_hb or has_ping or _debug_recv_count <= 5:
                        self._log(f"[TCP-DEBUG] recv #{_debug_recv_count}: {len(chunk)}B, "
                                  f"heartbeat={has_hb}, ping={has_ping}, "
                                  f"first8={chunk[:8].hex()}")

                self._process_buffer()

                self._check_periodic_output()

            except socket.timeout:
                self._check_periodic_output()
                continue
            except UnicodeDecodeError:
                continue
            except Exception as e:
                if self.running:
                    self._log(f"[TCP] 接收错误: {e}")
                break

        # 连接断开，清理
        self.connected = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass
            self.client_socket = None
        if self.on_disconnected:
            self.on_disconnected()

    def _check_periodic_output(self):
        """检查是否需要输出周期性汇总和心跳状态"""
        now = time.time()

        if now - self._last_heartbeat_print_time >= self._heartbeat_print_interval:
            self._print_heartbeat_status()
            self._last_heartbeat_print_time = now

        if now - self._last_summary_time >= self._summary_interval:
            self._print_data_summary()
            self._last_summary_time = now

    def _print_heartbeat_status(self):
        """打印心跳状态（每3秒一次）"""
        now = time.time()
        elapsed = now - self._last_heartbeat_time

        if self._heartbeat_count > 0 and elapsed < 5.0:
            self._log(f"[心跳] 正常 | 累计: {self._heartbeat_count}次 | 距上次: {elapsed:.1f}s")
        elif elapsed >= 5.0:
            self._log(f"[心跳] 超过 {elapsed:.1f}s 未收到心跳! | 累计: {self._heartbeat_count}次")

    def _print_data_summary(self):
        """打印数据汇总（每5秒一次）"""
        motor_rate = self._motor_frame_count / self._summary_interval
        force_rate = self._foot_force_frame_count / self._summary_interval

        lines = []
        lines.append("=" * 60)
        lines.append(f"[数据汇总] 过去 {self._summary_interval:.0f}s 统计")
        lines.append(f"  帧速率: 电机 {motor_rate:.1f}fps, 足端力 {force_rate:.1f}fps | "
                     f"总接收: {self.stats.bytes_received/1024:.1f}KB")

        with self.lock:
            ms = self.motor_status
            ff = self.foot_force

        if ms and ms.is_valid():
            leg_names = ['FL', 'FR', 'RL', 'RR']
            lines.append("  电机位置 (度):")
            for leg in range(4):
                pos = ms.positions[leg*3:(leg+1)*3]
                lines.append(f"    {leg_names[leg]}: {pos[0]:7.2f}  {pos[1]:7.2f}  {pos[2]:7.2f}")

            vel_max = np.max(np.abs(ms.velocities))
            vel_avg = np.mean(np.abs(ms.velocities))
            lines.append(f"  速度: 平均 {vel_avg:.1f} deg/s, 最大 {vel_max:.1f} deg/s")

            cur_max = np.max(np.abs(ms.currents))
            cur_avg = np.mean(np.abs(ms.currents))
            lines.append(f"  扭矩: 平均 {cur_avg:.2f}, 最大 {cur_max:.2f}")
        else:
            lines.append("  电机状态: 暂无数据")

        if ff:
            leg_labels = ['FL', 'FR', 'RL', 'RR']
            forces_str = "  ".join(f"{leg_labels[i]}={ff.forces[i]:6.1f}N" for i in range(4))
            contacts = "  ".join(f"{leg_labels[i]}={'接触' if ff.is_contact(i) else '--'}" for i in range(4))
            lines.append(f"  足端力: {forces_str}")
            lines.append(f"  接触:   {contacts}")
        else:
            lines.append("  足端力: 暂无数据")

        lines.append("=" * 60)
        self._log_block(lines)

        self._motor_frame_count = 0
        self._foot_force_frame_count = 0

    def _process_buffer(self):
        """处理接收缓冲区"""
        FOOT_FORCE_FRAME_TYPE = 0x02
        MOTOR_FEEDBACK_FRAME_TYPE = 0x03
        ENHANCED_MOTOR_STATUS_TYPE = 0x04

        FRAME_SIZES = {
            0x02: 24,
            0x03: 200,
            0x04: 23,
        }

        # 心跳文本，用于检测部分匹配
        HEARTBEAT_TEXT = b'HEARTBEAT'

        processed_count = 0
        MAX_PROCESSED_PER_CALL = 50

        while len(self.rx_buffer) >= 4 and processed_count < MAX_PROCESSED_PER_CALL:
            processed_count += 1

            heartbeat_pos = bytes(self.rx_buffer).find(HEARTBEAT_TEXT)
            if heartbeat_pos != -1:
                newline_idx = self.rx_buffer.find(b'\n', heartbeat_pos)

                if newline_idx != -1:
                    heartbeat_msg = bytes(self.rx_buffer[heartbeat_pos:newline_idx+1])
                    del self.rx_buffer[:newline_idx+1]
                    self._handle_heartbeat(heartbeat_msg)
                    continue
                elif len(self.rx_buffer) - heartbeat_pos > 50:
                    # 心跳文本后太久没有换行，可能是假阳性，跳过
                    del self.rx_buffer[:heartbeat_pos + 9]
                else:
                    # 等待心跳完整到达
                    break

            if len(self.rx_buffer) < 2:
                break

            # 检查是否为二进制帧头
            if self.rx_buffer[0] == 0xAA and self.rx_buffer[1] == 0x55:
                # 二进制帧处理
                if len(self.rx_buffer) < 4:
                    break

                frame_type = self.rx_buffer[2]
                frame_size = FRAME_SIZES.get(frame_type)
                if frame_size is None:
                    del self.rx_buffer[:2]
                    continue

                if len(self.rx_buffer) < frame_size:
                    break

                frame = bytes(self.rx_buffer[:frame_size])
                del self.rx_buffer[:frame_size]

                if frame_type == FOOT_FORCE_FRAME_TYPE:
                    self._handle_foot_force_frame(frame)
                elif frame_type == MOTOR_FEEDBACK_FRAME_TYPE:
                    self._handle_motor_feedback_frame(frame)
                elif frame_type == ENHANCED_MOTOR_STATUS_TYPE:
                    self._handle_enhanced_motor_status_frame(frame)
                continue

            # 非帧头字节：检查是否可能是部分心跳文本
            # 检查缓冲区尾部是否包含 "HEARTBEAT" 的前缀
            buffer_tail = bytes(self.rx_buffer[-min(9, len(self.rx_buffer)):])
            is_partial_heartbeat = False
            for i in range(1, len(HEARTBEAT_TEXT)):
                if buffer_tail.endswith(HEARTBEAT_TEXT[:i]):
                    is_partial_heartbeat = True
                    break

            if is_partial_heartbeat:
                # 保留部分心跳文本，等待下次recv
                break

            # 既不是帧头也不是部分心跳，删除这个无效字节
            del self.rx_buffer[:1]

            if len(self.rx_buffer) > MAX_BUFFER_SIZE:
                skip_count = 0
                while len(self.rx_buffer) >= 2 and self.rx_buffer[0:2] != ProtocolCodec.FRAME_HEADER:
                    # 同样检查部分心跳
                    tail = bytes(self.rx_buffer[-min(9, len(self.rx_buffer)):])
                    partial = False
                    for i in range(1, len(HEARTBEAT_TEXT)):
                        if tail.endswith(HEARTBEAT_TEXT[:i]):
                            partial = True
                            break
                    if partial:
                        break
                    del self.rx_buffer[:1]
                    skip_count += 1
                if skip_count > 0:
                    self._log(f"[TCP] 缓冲区恢复: 跳过{skip_count}字节")

    def _handle_foot_force_frame(self, frame: bytes):
        """处理足端力数据帧"""
        if len(frame) < 24:
            return

        checksum_calc = sum(frame[:-1]) & 0xFF
        checksum_recv = frame[-1]
        if checksum_calc != checksum_recv:
            return

        payload = frame[3:-1]
        foot_force = ProtocolCodec.parse_foot_force(payload)
        if foot_force:
            with self.lock:
                self.foot_force = foot_force

            self._foot_force_frame_count += 1

            if self.on_foot_force:
                self.on_foot_force(foot_force)
            self.stats.packets_received += 1
            self.stats.last_receive_time = time.time()

    def _handle_motor_feedback_frame(self, frame: bytes):
        """处理电机反馈数据帧"""
        if len(frame) < 152:
            return

        checksum_calc = sum(frame[:-1]) & 0xFF
        checksum_recv = frame[-1]
        if checksum_calc != checksum_recv:
            return

        payload = frame[3:-1]
        motor_status = ProtocolCodec.parse_motor_status(payload)
        if motor_status:
            with self.lock:
                self.motor_status = motor_status

            self._motor_frame_count += 1

            if self.on_motor_status:
                self.on_motor_status(motor_status)
            self.stats.packets_received += 1
            self.stats.last_receive_time = time.time()

    def _handle_enhanced_motor_status_frame(self, frame: bytes):
        """处理增强电机状态帧"""
        if len(frame) < 23:
            return

        checksum_calc = sum(frame[:-1]) & 0xFF
        checksum_recv = frame[-1]
        if checksum_calc != checksum_recv:
            return

        try:
            idx = 3
            timestamp = struct.unpack('<I', frame[idx:idx+4])[0]
            idx += 4
            motor_id = frame[idx]
            idx += 1
            position = struct.unpack('<f', frame[idx:idx+4])[0]
            idx += 4
            velocity = struct.unpack('<f', frame[idx:idx+4])[0]
            idx += 4
            torque = struct.unpack('<f', frame[idx:idx+4])[0]
            idx += 4
            current = struct.unpack('<f', frame[idx:idx+4])[0]

            self._motor_frame_count += 1

            if motor_id < 12:
                with self.lock:
                    if self.motor_status is None:
                        self.motor_status = MotorStatus()
                    self.motor_status.positions[motor_id] = position
                    self.motor_status.velocities[motor_id] = velocity
                    self.motor_status.currents[motor_id] = current
                    self.motor_status.timestamp = timestamp / 1000.0

                if self.on_motor_status and motor_id == 11:
                    self.on_motor_status(self.motor_status)

        except Exception:
            pass

    def _handle_heartbeat(self, heartbeat_msg: bytes):
        """处理心跳消息（回复PONG确认）"""
        self._heartbeat_count += 1
        self._last_heartbeat_time = time.time()

        try:
            self.send_raw(b"PONG\n")
        except Exception:
            pass

    def _handle_response(self, response: Dict[str, Any]):
        """处理响应"""
        self.stats.packets_received += 1
        self.stats.last_receive_time = time.time()

        resp_type = response['type']
        payload = response['payload']

        if resp_type == ResponseType.ACK:
            pass
        elif resp_type == ResponseType.NACK:
            error_code = payload[0] if payload else 0
            self._log(f"[TCP] NACK, error: 0x{error_code:02X}")
            self.stats.errors += 1

        elif resp_type == ResponseType.MOTOR_STATUS:
            motor_status = ProtocolCodec.parse_motor_status(payload)
            if motor_status:
                with self.lock:
                    self.motor_status = motor_status
                if self.on_motor_status:
                    self.on_motor_status(motor_status)

        elif resp_type == ResponseType.FOOT_FORCE:
            foot_force = ProtocolCodec.parse_foot_force(payload)
            if foot_force:
                with self.lock:
                    self.foot_force = foot_force
                if self.on_foot_force:
                    self.on_foot_force(foot_force)

        elif resp_type == ResponseType.POWER_STATUS:
            power_status = ProtocolCodec.parse_power_status(payload)
            if power_status:
                with self.lock:
                    self.power_status = power_status
                if self.on_power_status:
                    self.on_power_status(power_status)

    def send_raw(self, data: bytes) -> bool:
        """直接发送原始数据（线程安全）"""
        with self._state_lock:
            if not self._connected or self.client_socket is None:
                return False
            sock = self.client_socket

        try:
            sock.sendall(data)
            with self._state_lock:
                self.stats.packets_sent += 1
                self.stats.bytes_sent += len(data)
            return True
        except Exception as e:
            self._log(f"[TCP] 发送失败: {e}")
            return False

    def send_command(self, cmd_type: int, payload: bytes = b'') -> bool:
        """发送命令（带协议封装，线程安全）"""
        with self._state_lock:
            if not self._connected or self.client_socket is None:
                return False
            sock = self.client_socket

        try:
            frame = ProtocolCodec.encode_command(cmd_type, payload)
            sock.sendall(frame)
            with self._state_lock:
                self.stats.packets_sent += 1
                self.stats.bytes_sent += len(frame)
            return True
        except Exception as e:
            self._log(f"[TCP] 发送失败: {e}")
            return False

    def send_positions(self, positions: np.ndarray, use_protocol: bool = True) -> bool:
        """发送12个关节位置命令"""
        if positions.shape != (12,):
            self._log(f"[TCP] 位置数据维度错误: {positions.shape}")
            return False

        clipped_positions = np.clip(positions, 0.0, 359.999)
        clipped_positions = np.where(np.abs(clipped_positions) < 0.001, 0.0, clipped_positions)

        float_array = np.zeros(12, dtype=np.float32)
        for i in range(12):
            val = float(clipped_positions[i])
            if val < 0.001:
                val = 0.0
            float_array[i] = val

        payload = struct.pack('<12f', *float_array)

        if use_protocol:
            frame = ProtocolCodec.encode_command(CommandType.MOTOR_POS_CONTROL, payload)
            return self.send_raw(frame)
        else:
            return self.send_raw(payload)

    def send_velocities(self, velocities: np.ndarray) -> bool:
        """发送12个关节速度命令"""
        if velocities.shape != (12,):
            return False
        payload = struct.pack('<12f', *velocities.astype(np.float32))
        return self.send_command(CommandType.MOTOR_VEL_CONTROL, payload)

    def send_torques(self, torques: np.ndarray) -> bool:
        """发送12个关节力矩命令"""
        if torques.shape != (12,):
            return False
        payload = struct.pack('<12f', *torques.astype(np.float32))
        return self.send_command(CommandType.MOTOR_TRQ_CONTROL, payload)

    def enable_motors(self) -> bool:
        """使能所有电机"""
        return self.send_command(CommandType.MOTOR_ENABLE_ALL)

    def disable_motors(self) -> bool:
        """禁用所有电机"""
        return self.send_command(CommandType.MOTOR_DISABLE_ALL)

    def emergency_stop(self) -> bool:
        """紧急停止"""
        self._log("[TCP] 发送紧急停止命令!")
        return self.send_command(CommandType.EMERGENCY_STOP)

    def send_heartbeat(self) -> bool:
        """发送心跳"""
        return self.send_command(CommandType.HEARTBEAT)

    def enable_data_send(self, enable: bool = True) -> bool:
        """启用/禁用电机数据发送"""
        payload = bytes([1 if enable else 0])
        return self.send_command(CommandType.MOTOR_SEND_ENABLE, payload)

    def get_motor_status(self) -> Optional[MotorStatus]:
        """获取最新电机状态"""
        with self.lock:
            return self.motor_status

    def get_foot_force(self) -> Optional[FootForceData]:
        """获取最新足端力数据"""
        with self.lock:
            return self.foot_force

    def get_power_status(self) -> Optional[PowerStatus]:
        """获取最新电源状态"""
        with self.lock:
            return self.power_status

    def get_stats(self) -> TCPStats:
        """获取统计信息"""
        return self.stats

    def stop(self):
        """停止服务器"""
        self.running = False
        self.connected = False

        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass

        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)

        if hasattr(self, '_accept_thread') and self._accept_thread and self._accept_thread.is_alive():
            self._accept_thread.join(timeout=2.0)

        self._log("[TCP] 服务器已停止")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False
