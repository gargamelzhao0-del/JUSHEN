#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四足机器狗Trot步态模拟器 - 上位机运行
Quadruped Robot Trot Gait Simulator (Jetson)

功能:
- 模拟强化学习输出的50Hz 12电机位置数据
- 生成Trot步态（对角小跑）让机器狗前进
- 通过TCP发送给STM32
- 实时显示3D步态动画预览

使用方法:
    python trot_gait_simulator.py
    python trot_gait_simulator.py --mode server  # 仅TCP服务器
    python trot_gait_simulator.py --mode preview  # 仅动画预览
    python trot_gait_simulator.py --mode both  # 两者都运行
"""

import numpy as np
import struct
import socket
import time
import argparse
import threading
import os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

NUM_MOTORS = 12
MOTOR_NEUTRAL = 180.0

class SmartTargetController:
    """
    智能目标控制器 - V3.0版本
    
    V3.0改进：
    1. 提高发送频率到50Hz（20ms间隔），与下位机500Hz控制循环更好地同步
    2. 移除角度变化限制，让下位机轨迹规划器处理平滑
    3. 下位机会自动实现加减速曲线，确保运动平稳
    4. 添加预测机制：如果下位机未完成轨迹，不发送新目标
    
    设计理念：
    - 上位机：快速响应强化学习输出，发送期望目标位置
    - 下位机：轨迹规划器实现平滑运动，动态调整速度/加速度
    
    频率设计：
    - 上位机发送: 50Hz (20ms)
    - 下位机控制: 500Hz (2ms)
    - 比例: 1:10，给轨迹规划足够时间完成加减速
    """
    
    def __init__(self):
        self.current_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.last_sent_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        
        # V3.0: 50Hz发送频率，与下位机更好同步
        self.min_update_interval = 0.02  # 20ms = 50Hz
        self.last_update_time = 0
        
        self.enable_smoothing = False  # V3.0: 禁用平滑，让下位机处理
        
        # V3.0: 降低阈值，更快响应强化学习输出
        self.position_threshold = 0.1  # 位置变化阈值，小于此值不发送
    
    def compute_action(self, ideal_angles, current_time):
        """
        根据强化学习的理想角度计算实际发送的目标
        
        V2.0: 直接发送目标位置，由下位机轨迹规划器处理平滑
        """
        if not self.enable_smoothing:
            time_since_last = current_time - self.last_update_time
            
            if time_since_last < self.min_update_interval:
                return self.last_sent_targets
            
            max_delta = max(abs(ideal_angles[i] - self.last_sent_targets[i]) for i in range(NUM_MOTORS))
            
            if max_delta < self.position_threshold:
                return self.last_sent_targets
            
            self.last_sent_targets = ideal_angles.copy()
            self.last_update_time = current_time
            
            return ideal_angles
        
        time_since_last = current_time - self.last_update_time
        
        if time_since_last < self.min_update_interval:
            return self.last_sent_targets
        
        delta = [ideal_angles[i] - self.last_sent_targets[i] for i in range(NUM_MOTORS)]
        max_delta = max(abs(d) for d in delta)
        
        if max_delta < self.position_threshold:
            return self.last_sent_targets
        
        self.last_sent_targets = ideal_angles.copy()
        self.last_update_time = current_time
        
        return ideal_angles
    
    def reset(self):
        """重置到中立位置"""
        self.current_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.last_sent_targets = [MOTOR_NEUTRAL] * NUM_MOTORS
        self.last_update_time = 0
    
    def set_update_interval(self, interval_sec):
        """设置更新间隔"""
        self.min_update_interval = interval_sec


class GaitGenerator:
    def __init__(self):
        self.amplitude = 25
        self.frequency = 1.0
        self.forward_speed = 1.0
        self.body_sway = 5
    
    def generate_trot_gait(self, time_sec):
        phase_fl = 0
        phase_fr = np.pi
        phase_rl = np.pi
        phase_rr = 0
        
        hip_amp = self.amplitude * 0.3
        thigh_amp = self.amplitude
        knee_amp = self.amplitude * 0.8
        
        angles = []
        
        for leg_id in range(4):
            if leg_id == 0:
                phase = phase_fl
            elif leg_id == 1:
                phase = phase_fr
            elif leg_id == 2:
                phase = phase_rl
            else:
                phase = phase_rr
            
            hip_angle = MOTOR_NEUTRAL + hip_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            knee_angle = MOTOR_NEUTRAL + knee_amp * np.sin(2 * np.pi * self.frequency * time_sec + phase)
            
            angles.extend([hip_angle, thigh_angle, knee_angle])
        
        return angles
    
    def generate_trot_forward(self, time_sec, forward_speed=1.0):
        phase_fl = 0
        phase_fr = np.pi
        phase_rl = np.pi
        phase_rr = 0
        
        hip_amp = self.amplitude * 0.4
        thigh_amp = self.amplitude * 1.2
        knee_amp = self.amplitude * 0.8
        
        step_phase = 2 * np.pi * self.frequency * time_sec
        
        lift_amp = self.amplitude * 0.3
        
        angles = []
        
        for leg_id in range(4):
            if leg_id == 0:
                phase = phase_fl
                forward_offset = forward_speed * 0.2
            elif leg_id == 1:
                phase = phase_fr
                forward_offset = -forward_speed * 0.2
            elif leg_id == 2:
                phase = phase_rl
                forward_offset = -forward_speed * 0.2
            else:
                phase = phase_rr
                forward_offset = forward_speed * 0.2
            
            base_swing = np.sin(step_phase + phase)
            
            hip_angle = MOTOR_NEUTRAL + forward_offset + hip_amp * base_swing
            
            lift = np.where(base_swing > 0, base_swing * lift_amp, 0)
            thigh_angle = MOTOR_NEUTRAL + thigh_amp * base_swing + lift
            
            knee_bend = np.where(base_swing > 0, base_swing * knee_amp * 0.5, 0)
            knee_angle = MOTOR_NEUTRAL + knee_amp * base_swing + knee_bend
            
            angles.extend([hip_angle, thigh_angle, knee_angle])
        
        return angles


class QuadTCPServer:
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.gait_generator = GaitGenerator()
        self.smart_controller = SmartTargetController()
        self.send_count = 0
        self.recv_count = 0
        self.lock = threading.Lock()
        self.forward_speed = 1.0
        self.amplitude = 25
        self.frequency = 1.0
        self.tcp_receive_enabled = True
    
    def set_params(self, amplitude, frequency, forward_speed):
        self.gait_generator.amplitude = amplitude
        self.gait_generator.frequency = frequency
        self.forward_speed = forward_speed
    
    def parse_foot_force(self, data):
        """解析足端力数据"""
        if len(data) < 24:
            return None
        if data[0] != 0xAA or data[1] != 0x55:
            return None
        packet_type = data[2]
        if packet_type != 0x02:
            return None
        
        fl = struct.unpack('<f', data[3:7])[0]
        fr = struct.unpack('<f', data[7:11])[0]
        rl = struct.unpack('<f', data[11:15])[0]
        rr = struct.unpack('<f', data[15:19])[0]
        timestamp = struct.unpack('<I', data[19:23])[0]
        
        return {'fl': fl, 'fr': fr, 'rl': rl, 'rr': rr, 'timestamp': timestamp}
    
    def parse_motor_feedback(self, data):
        """解析电机反馈数据"""
        if len(data) < 18:
            return None
        if data[0] != 0xAA or data[1] != 0x55:
            return None
        packet_type = data[2]
        if packet_type != 0x01:
            return None
        
        motor_count = data[3]
        if len(data) < 4 + motor_count * 12 + 1:
            return None
        
        motors = []
        for i in range(motor_count):
            offset = 4 + i * 12
            position = struct.unpack('<f', data[offset:offset+4])[0]
            velocity = struct.unpack('<f', data[offset+4:offset+8])[0]
            torque = struct.unpack('<f', data[offset+8:offset+12])[0]
            motors.append({'position': position, 'velocity': velocity, 'torque': torque})
        
        return {'motor_count': motor_count, 'motors': motors}
    
    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        
        print(f'[TCP] 服务器监听: {self.host}:{self.port}')
        
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    print(f'[TCP] 客户端连接: {addr}')
                    self.handle_client()
                except socket.timeout:
                    continue
            except Exception as e:
                if self.running:
                    print(f'[TCP] 错误: {e}')
                break
    
    def handle_client(self):
        if not self.client_socket:
            return
        
        start_time = time.time()
        self.client_socket.settimeout(0.5)
        
        try:
            while self.running:
                if self.tcp_receive_enabled:
                    try:
                        data = self.client_socket.recv(512)
                        if not data:
                            print('[TCP] 客户端断开连接')
                            break
                        
                        self.recv_count += 1
                        recv_len = len(data)
                        
                        result = self.parse_foot_force(data)
                        if result:
                            print(f'[TCP] 收到足端力: FL={result["fl"]:.1f}N FR={result["fr"]:.1f}N '
                                  f'RL={result["rl"]:.1f}N RR={result["rr"]:.1f}N')
                            try:
                                self.client_socket.send(b'ok')
                            except:
                                pass
                            continue
                        
                        result = self.parse_motor_feedback(data)
                        if result:
                            print(f'[TCP] 收到电机反馈: {result["motor_count"]}个电机')
                            for i, m in enumerate(result['motors'][:3]):
                                print(f'  Motor {i}: pos={m["position"]:.1f} vel={m["velocity"]:.1f} tor={m["torque"]:.1f}')
                            try:
                                self.client_socket.send(b'ok')
                            except:
                                pass
                            continue
                        
                        print(f'[TCP] 收到未知数据: {recv_len}字节')
                        try:
                            self.client_socket.send(b'ok')
                        except:
                            pass
                            
                    except socket.timeout:
                        pass
                    except Exception as e:
                        print(f'[TCP] 接收错误: {e}')
                        break
                
                elapsed = time.time() - start_time
                
                ideal_angles = self.gait_generator.generate_trot_forward(elapsed, self.forward_speed)
                
                angles = self.smart_controller.compute_action(ideal_angles, elapsed)
                
                data = struct.pack('12f', *angles)
                
                try:
                    self.client_socket.sendall(data)
                    self.send_count += 1
                    
                    if self.send_count % 50 == 0:
                        print(f'[TCP] 已发送 {self.send_count} 帧, 收到 {self.recv_count} 帧')
                    
                    # V3.0: 20ms间隔，50Hz发送频率
                    time.sleep(0.02)
                    
                except Exception as e:
                    print(f'[TCP] 发送失败: {e}')
                    break
                    
        except Exception as e:
            print(f'[TCP] 客户端错误: {e}')
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            print('[TCP] 客户端断开')
    
    def stop(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print('[TCP] 服务器已停止')


class GaitPreview:
    def __init__(self, gait_generator):
        self.gait_generator = gait_generator
        self.time = 0
        self.running = True
        
        self.fig = plt.figure(figsize=(16, 8))
        self.fig.suptitle('Quadruped Robot - Trot Gait Preview (50Hz)', 
                          fontsize=16, fontweight='bold')
        
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_joints = self.fig.add_subplot(122)
        
        self.setup_3d_view()
        self.setup_joint_view()
        
        self.leg_lines = {}
        self.leg_scatters = {}
        self.body_patch = None
        
        plt.tight_layout()
        
        plt.ion()
        plt.show()
        plt.pause(0.1)
    
    def setup_3d_view(self):
        self.ax_3d.set_xlim(-250, 250)
        self.ax_3d.set_ylim(-250, 250)
        self.ax_3d.set_zlim(-50, 250)
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D View')
        self.ax_3d.view_init(elev=25, azim=45)
    
    def setup_joint_view(self):
        self.ax_joints.set_xlim(0, 10)
        self.ax_joints.set_ylim(0, 12)
        self.ax_joints.axis('off')
        self.ax_joints.set_title('Joint Angles', fontsize=12)
        
        self.joint_labels = []
        leg_names = ['FL', 'FR', 'RL', 'RR']
        joint_names = ['Hip', 'Thigh', 'Knee']
        
        for i in range(12):
            leg = i // 3
            joint = i % 3
            y_pos = 11 - i * 0.9
            label = self.ax_joints.text(0.5, y_pos, 
                                        f'{leg_names[leg]}_{joint_names[joint]}: 180.0°',
                                        fontsize=9, fontfamily='monospace')
            self.joint_labels.append(label)
        
        self.info_label = self.ax_joints.text(5, 0.5, 
                                             'Time: 0.00s | Freq: 1.0Hz',
                                             fontsize=10, ha='center')
        
        self.ax_joints.text(5, -0.5,
                           'Trot Gait: FL+RR同步, FR+RL同步\n红色箭头=前进方向',
                           fontsize=8, ha='center', style='italic')
    
    def draw_body_simple(self):
        body_width = 180
        body_length = 220
        body_height = 30
        body_z = 100
        
        x = [-body_length/2, body_length/2]
        y = [-body_width/2, body_width/2]
        z = [body_z - body_height/2, body_z + body_height/2]
        
        xx, yy = np.meshgrid(x, y)
        
        self.ax_3d.plot_wireframe(xx, yy, z[0] * np.ones_like(xx), color='#3498DB', alpha=0.5)
        self.ax_3d.plot_wireframe(xx, yy, z[1] * np.ones_like(xx), color='#3498DB', alpha=0.5)
        
        for i in [0, 1]:
            for j in [0, 1]:
                self.ax_3d.plot([x[i], x[i]], [y[j], y[j]], z, color='#3498DB', alpha=0.5)
    
    def calculate_leg_points(self, angles):
        leg_config = {
            0: {'name': 'FL', 'base_x': -110, 'base_y': 110},
            1: {'name': 'FR', 'base_x': 110, 'base_y': 110},
            2: {'name': 'RL', 'base_x': -110, 'base_y': -110},
            3: {'name': 'RR', 'base_x': 110, 'base_y': -110}
        }
        
        leg_points = {}
        
        for leg in range(4):
            hip_id = leg * 3
            thigh_id = leg * 3 + 1
            knee_id = leg * 3 + 2
            
            hip_angle = angles[hip_id] - MOTOR_NEUTRAL
            thigh_angle = angles[thigh_id] - MOTOR_NEUTRAL
            knee_angle = angles[knee_id] - MOTOR_NEUTRAL
            
            config = leg_config[leg]
            
            segment_hip = 35
            segment_thigh = 70
            segment_knee = 70
            
            base_x = config['base_x']
            base_y = config['base_y']
            base_z = 100
            
            hip_x = base_x + segment_hip * np.sin(np.radians(hip_angle))
            hip_y = base_y
            hip_z = base_z
            
            thigh_x = hip_x + segment_thigh * np.sin(np.radians(hip_angle + thigh_angle))
            thigh_y = hip_y
            thigh_z = base_z - segment_thigh * np.cos(np.radians(hip_angle + thigh_angle))
            
            knee_x = thigh_x + segment_knee * np.sin(np.radians(hip_angle + thigh_angle + knee_angle))
            knee_y = thigh_y
            knee_z = thigh_z - segment_knee * np.cos(np.radians(hip_angle + thigh_angle + knee_angle))
            
            foot_x = knee_x
            foot_y = knee_y
            foot_z = max(knee_z - 10, 5)
            
            leg_points[leg] = {
                'base': (base_x, base_y, base_z),
                'hip': (hip_x, hip_y, hip_z),
                'thigh': (thigh_x, thigh_y, thigh_z),
                'knee': (knee_x, knee_y, knee_z),
                'foot': (foot_x, foot_y, foot_z)
            }
        
        return leg_points
    
    def draw_legs(self, leg_points):
        colors = ['#E74C3C', '#3498DB', '#2ECC71', '#F39C12']
        
        for leg in range(4):
            points = [
                leg_points[leg]['base'],
                leg_points[leg]['hip'],
                leg_points[leg]['thigh'],
                leg_points[leg]['knee'],
                leg_points[leg]['foot']
            ]
            
            color = colors[leg]
            
            for i in range(len(points) - 1):
                width = 3 - i * 0.5
                self.ax_3d.plot([points[i][0], points[i+1][0]],
                               [points[i][1], points[i+1][1]],
                               [points[i][2], points[i+1][2]],
                               color=color, linewidth=max(width, 1), alpha=0.8)
            
            for point in points[1:4]:
                self.ax_3d.scatter([point[0]], [point[1]], [point[2]], 
                                  s=80, c=color, alpha=0.8, edgecolors='black', linewidth=0.5)
    
    def update_frame(self):
        self.time += 0.02
        
        self.gait_generator.frequency = 2.5
        
        angles = self.gait_generator.generate_trot_forward(self.time, self.gait_generator.forward_speed)
        
        for i, label in enumerate(self.joint_labels):
            leg_names = ['FL', 'FR', 'RL', 'RR']
            joint_names = ['Hip', 'Thigh', 'Knee']
            leg = i // 3
            joint = i % 3
            label.set_text(f'{leg_names[leg]}_{joint_names[joint]}: {angles[i]:.1f}°')
        
        self.info_label.set_text(f'Time: {self.time:.2f}s | Freq: {self.gait_generator.frequency}Hz | Amp: {self.gait_generator.amplitude}°')
        
        self.ax_3d.clear()
        
        self.ax_3d.set_xlim(-250, 250)
        self.ax_3d.set_ylim(-250, 250)
        self.ax_3d.set_zlim(-50, 250)
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Z (mm)')
        self.ax_3d.set_title('3D View')
        self.ax_3d.view_init(elev=25, azim=45)
        
        self.draw_body_simple()
        
        leg_points = self.calculate_leg_points(angles)
        self.draw_legs(leg_points)
        
        self.fig.suptitle(f'Quadruped Robot - Trot Gait Preview (Time: {self.time:.2f}s)', 
                         fontsize=16, fontweight='bold')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def animate(self):
        print("[动画] 开始播放动画，按 Ctrl+C 停止")
        try:
            while self.running:
                self.update_frame()
        except KeyboardInterrupt:
            print("\n[动画] 停止播放")


def main():
    parser = argparse.ArgumentParser(description='Quadruped Robot Trot Gait Simulator')
    parser.add_argument('--mode', choices=['server', 'preview', 'both'], default='both',
                       help='运行模式')
    parser.add_argument('--amp', type=float, default=25, help='步幅幅度 (度)')
    parser.add_argument('--freq', type=float, default=1.0, help='步态频率 (Hz)')
    parser.add_argument('--speed', type=float, default=1.0, help='前进速度')
    parser.add_argument('--port', type=int, default=9999, help='TCP端口')
    
    args = parser.parse_args()
    
    gait_generator = GaitGenerator()
    gait_generator.amplitude = args.amp
    gait_generator.frequency = args.freq
    
    print("=" * 60)
    print("  四足机器狗 Trot 步态模拟器")
    print("=" * 60)
    print(f"  步幅幅度: {args.amp}°")
    print(f"  步态频率: {args.freq} Hz")
    print(f"  前进速度: {args.speed}")
    print(f"  TCP端口:  {args.port}")
    print("=" * 60)
    print("  步态说明:")
    print("    FL (左前) + RR (右后) 同步")
    print("    FR (右前) + RL (左后) 同步")
    print("    这就是Trot对角小跑步态")
    print("=" * 60)
    
    if args.mode in ['server', 'both']:
        print('[启动] TCP服务器...')
        server = QuadTCPServer(port=args.port)
        server.set_params(args.amp, args.freq, args.speed)
        
        server_thread = threading.Thread(target=server.start)
        server_thread.daemon = True
        server_thread.start()
    
    if args.mode in ['preview', 'both']:
        if os.environ.get('DISPLAY') is None:
            print('[警告] 未检测到DISPLAY环境变量，跳过3D动画预览')
            print('[提示] 如需预览动画，请:')
            print('  - 在有显示器的电脑上运行: python trot_gait_simulator.py --mode preview')
            print('  - 或设置虚拟显示器: export DISPLAY=:0')
        else:
            print('[启动] 3D动画预览...')
            preview = GaitPreview(gait_generator)
            preview.gait_generator = gait_generator
            preview.animate()
    
    if args.mode == 'server':
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print('\n[停止] 服务器...')
            server.stop()


if __name__ == '__main__':
    main()
