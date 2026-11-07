#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四足机器狗3D步态仿真工具 - 完整版
Quadruped Robot 3D Gait Simulation - Complete

功能:
- 读取CAN CSV数据文件 (1-6电机和7-12电机)
- 解析所有指令: 位置目标(0x131)、扭矩目标(0x134)、位置限幅(0x240/0x241)
- 建立完整的3D四足机器人模型
- 支持实时播放和GIF导出
"""

import csv
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import argparse
import os

MOTOR_NEUTRAL = 180.0

def get_motor_info(can_id):
    """解析CAN ID，返回(电机编号, 消息类型)
    CAN ID格式: [DeviceID(7bit) << 12] | [MessageID(10bit)]
    DeviceID: 1-12 对应电机1-12
    """
    device_id = (can_id >> 12) & 0x7F
    message_id = can_id & 0x3FF
    
    if 1 <= device_id <= 12:
        motor_idx = device_id - 1
        
        if message_id == 0x131:
            return (motor_idx, 'position_target')
        elif message_id == 0x134:
            return (motor_idx, 'torque_target')
        elif message_id == 0x221:
            return (motor_idx, 'speed_limit')
        elif message_id == 0x222:
            return (motor_idx, 'accel_limit')
        elif message_id == 0x223:
            return (motor_idx, 'pos_kp')
        elif message_id == 0x240:
            return (motor_idx, 'pos_upper_limit')
        elif message_id == 0x241:
            return (motor_idx, 'pos_lower_limit')
        elif message_id == 0x243:
            return (motor_idx, 'torque_limit')
        elif message_id == 0x141:
            return (motor_idx, 'position_feedback')
    
    return (None, None)

def bytes_to_float(hex_str):
    try:
        hex_str = hex_str.replace(' ', '')
        if len(hex_str) >= 8:
            byte_data = bytes.fromhex(hex_str[:8])
            return struct.unpack('f', byte_data)[0]
    except:
        pass
    return 0.0

def parse_csv(filepath):
    """解析CSV文件，提取所有指令数据"""
    motor_data = {i: {
        'time': [], 
        'pos_target': [],      # 0x131 位置目标
        'torque_target': [],   # 0x134 扭矩目标
        'pos_upper': [],       # 0x240 位置上限
        'pos_lower': [],       # 0x241 位置下限
        'pos_feedback': []     # 0x40  位置反馈
    } for i in range(12)}
    
    if not os.path.exists(filepath):
        print(f'[WARN] File not found: {filepath}')
        return motor_data
    
    print(f'[DEBUG] Parsing: {filepath}')
    
    with open(filepath, 'r', encoding='gbk', errors='ignore') as f:
        reader = csv.reader(f)
        
        for row_num, row in enumerate(reader):
            if row_num == 0:
                continue
            
            if len(row) < 7:
                continue
            try:
                can_id_str = row[4].strip()
                if not can_id_str.startswith('0x'):
                    continue
                can_id = int(can_id_str, 16)
            except:
                continue
            
            motor_idx, msg_type = get_motor_info(can_id)
            if motor_idx is None:
                continue
            
            data = row[6].strip() if len(row) > 6 else ''
            value = bytes_to_float(data)
            
            if msg_type == 'position_target':
                motor_data[motor_idx]['pos_target'].append(value)
            elif msg_type == 'torque_target':
                motor_data[motor_idx]['torque_target'].append(value)
            elif msg_type == 'pos_upper_limit':
                motor_data[motor_idx]['pos_upper'].append(value)
            elif msg_type == 'pos_lower_limit':
                motor_data[motor_idx]['pos_lower'].append(value)
            elif msg_type == 'position_feedback':
                motor_data[motor_idx]['pos_feedback'].append(value)
    
    return motor_data

def merge_motor_data(data_1_6, data_7_12):
    """合并两个CSV的数据"""
    merged = {}
    for i in range(6):
        merged[i] = {
            'time': list(data_1_6[i]['time']),
            'pos_target': list(data_1_6[i]['pos_target']),
            'torque_target': list(data_1_6[i]['torque_target']),
            'pos_upper': list(data_1_6[i]['pos_upper']),
            'pos_lower': list(data_1_6[i]['pos_lower']),
            'pos_feedback': list(data_1_6[i]['pos_feedback']),
        }
    for i in range(6, 12):
        merged[i] = {
            'time': list(data_7_12[i]['time']),
            'pos_target': list(data_7_12[i]['pos_target']),
            'torque_target': list(data_7_12[i]['torque_target']),
            'pos_upper': list(data_7_12[i]['pos_upper']),
            'pos_lower': list(data_7_12[i]['pos_lower']),
            'pos_feedback': list(data_7_12[i]['pos_feedback']),
        }
    return merged

def interpolate_data(data_list, target_length):
    """对数据进行线性插值，使其扩展到目标长度"""
    if len(data_list) == 0:
        return [MOTOR_NEUTRAL] * target_length
    
    if len(data_list) >= target_length:
        return data_list[:target_length]
    
    result = []
    for i in range(target_length):
        pos = i * (len(data_list) - 1) / (target_length - 1) if target_length > 1 else 0
        idx = int(pos)
        frac = pos - idx
        
        if idx >= len(data_list) - 1:
            result.append(data_list[-1])
        else:
            val = data_list[idx] * (1 - frac) + data_list[idx + 1] * frac
            result.append(val)
    
    return result

def align_data_lengths(motor_data, target_fps=30, duration_sec=10):
    """对齐所有电机数据长度，使用插值填充稀疏数据"""
    max_frames = 0
    for i in range(12):
        max_frames = max(max_frames, 
                         len(motor_data[i]['pos_target']),
                         len(motor_data[i]['torque_target']),
                         len(motor_data[i]['pos_upper']),
                         len(motor_data[i]['pos_lower']),
                         len(motor_data[i]['pos_feedback']))
    
    if max_frames == 0:
        max_frames = target_fps * duration_sec
    
    min_frames = max_frames
    for i in range(12):
        for key in ['pos_target', 'torque_target', 'pos_upper', 'pos_lower', 'pos_feedback']:
            min_frames = min(min_frames, len(motor_data[i][key]))
    
    use_interpolation = min_frames < 100 and min_frames > 0
    
    if use_interpolation:
        print(f"[INFO] 数据稀疏 ({min_frames}帧), 使用插值扩展到 {max_frames} 帧")
    
    for i in range(12):
        for key in ['pos_target', 'torque_target', 'pos_upper', 'pos_lower', 'pos_feedback']:
            data_len = len(motor_data[i][key])
            
            if data_len == 0:
                motor_data[i][key] = [MOTOR_NEUTRAL] * max_frames
            elif data_len < max_frames:
                if use_interpolation:
                    motor_data[i][key] = interpolate_data(motor_data[i][key], max_frames)
                else:
                    last_val = motor_data[i][key][-1]
                    motor_data[i][key].extend([last_val] * (max_frames - data_len))
    
    return motor_data, max_frames

def map_motor_to_leg(motor_idx):
    """将电机映射到腿和关节"""
    leg_map = {
        0: {'leg': 0, 'joint': 'hip'},
        1: {'leg': 0, 'joint': 'thigh'},
        2: {'leg': 0, 'joint': 'knee'},
        3: {'leg': 1, 'joint': 'hip'},
        4: {'leg': 1, 'joint': 'thigh'},
        5: {'leg': 1, 'joint': 'knee'},
        6: {'leg': 2, 'joint': 'hip'},
        7: {'leg': 2, 'joint': 'thigh'},
        8: {'leg': 2, 'joint': 'knee'},
        9: {'leg': 3, 'joint': 'hip'},
        10: {'leg': 3, 'joint': 'thigh'},
        11: {'leg': 3, 'joint': 'knee'},
    }
    return leg_map.get(motor_idx, None)

def get_joint_angle(motor_idx, pos_value):
    """将电机位置转换为关节角度"""
    return pos_value - MOTOR_NEUTRAL

class QuadrupedRobot3D:
    def __init__(self):
        self.fig = plt.figure(figsize=(16, 12))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.leg_names = ['FL (前左)', 'FR (前右)', 'RL (后左)', 'RR (后右)']
        
        self._setup_view()
    
    def _setup_view(self):
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(-250, 50)
        self.ax.set_xlabel('X (mm)', fontsize=10)
        self.ax.set_ylabel('Y (mm)', fontsize=10)
        self.ax.set_zlabel('Z (mm)', fontsize=10)
        self.ax.set_title('四足机器狗3D步态仿真 - 完整指令数据', fontsize=14, fontweight='bold')
        
        self.ax.view_init(elev=25, azim=45)
    
    def calculate_leg_points(self, leg, joint_angles):
        leg_config = {
            0: {'name': 'FL', 'base_x': -120, 'base_y': 120},
            1: {'name': 'FR', 'base_x': 120, 'base_y': 120},
            2: {'name': 'RL', 'base_x': -120, 'base_y': -120},
            3: {'name': 'RR', 'base_x': 120, 'base_y': -120}
        }
        
        config = leg_config[leg]
        
        hip_angle = joint_angles['hip']
        thigh_angle = joint_angles['thigh']
        knee_angle = joint_angles['knee']
        
        segment_hip = 40
        segment_thigh = 80
        segment_knee = 80
        
        hip_x = config['base_x'] + segment_hip * np.sin(np.radians(hip_angle))
        hip_y = config['base_y']
        hip_z = -100
        
        thigh_x = hip_x + segment_thigh * np.sin(np.radians(hip_angle + thigh_angle))
        thigh_y = hip_y
        thigh_z = hip_z - segment_thigh * np.cos(np.radians(hip_angle + thigh_angle))
        
        knee_x = thigh_x + segment_knee * np.sin(np.radians(hip_angle + thigh_angle + knee_angle))
        knee_y = hip_y
        knee_z = thigh_z - segment_knee * np.cos(np.radians(hip_angle + thigh_angle + knee_angle))
        
        foot_x = knee_x
        foot_y = knee_y
        foot_z = min(knee_z + 10, 0)
        
        return {
            'base': (config['base_x'], config['base_y'], -100),
            'hip': (hip_x, hip_y, hip_z),
            'thigh': (thigh_x, thigh_y, thigh_z),
            'knee': (knee_x, knee_y, knee_z),
            'foot': (foot_x, foot_y, foot_z),
            'name': config['name']
        }
    
    def draw_body(self):
        body_width = 200
        body_length = 240
        body_height = 40
        body_z = -100
        
        vertices = [
            [-body_length/2, -body_width/2, body_z - body_height/2],
            [body_length/2, -body_width/2, body_z - body_height/2],
            [body_length/2, body_width/2, body_z - body_height/2],
            [-body_length/2, body_width/2, body_z - body_height/2],
            [-body_length/2, -body_width/2, body_z + body_height/2],
            [body_length/2, -body_width/2, body_z + body_height/2],
            [body_length/2, body_width/2, body_z + body_height/2],
            [-body_length/2, body_width/2, body_z + body_height/2]
        ]
        
        faces = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5]
        ]
        
        body = Poly3DCollection([[vertices[i] for i in face] for face in faces],
                                alpha=0.6, facecolor='#3498DB', edgecolor='#2980B9', linewidth=0.5)
        self.ax.add_collection3d(body)
        
        self.ax.text(0, 0, -130, 'Body', fontsize=10, color='#2C3E50', fontweight='bold')
    
    def draw_leg(self, leg_points, leg_num):
        colors = ['#E74C3C', '#3498DB', '#2ECC71', '#F39C12']
        color = colors[leg_num]
        
        segments = [
            ('base', 'hip', color),
            ('hip', 'thigh', '#E67E22'),
            ('thigh', 'knee', '#9B59B6'),
            ('knee', 'foot', '#1ABC9C')
        ]
        
        for start, end, c in segments:
            xs = [leg_points[start][0], leg_points[end][0]]
            ys = [leg_points[start][1], leg_points[end][1]]
            zs = [leg_points[start][2], leg_points[end][2]]
            self.ax.plot(xs, ys, zs, c=c, linewidth=3)
        
        for point in ['hip', 'thigh', 'knee', 'foot']:
            self.ax.scatter(*leg_points[point], c=colors[leg_num], s=30, zorder=5)
        
        self.ax.text(leg_points['foot'][0], leg_points['foot'][1], leg_points['foot'][2],
                    leg_points['name'], fontsize=8, color=colors[leg_num])
    
    def update_frame(self, frame, motor_data, frame_skip=5):
        self.ax.clear()
        self._setup_view()
        
        self.draw_body()
        
        for leg in range(4):
            joint_angles = {}
            for joint_name in ['hip', 'thigh', 'knee']:
                motor_idx = leg * 3 + ['hip', 'thigh', 'knee'].index(joint_name)
                
                pos_targets = motor_data[motor_idx]['pos_target']
                pos_uppers = motor_data[motor_idx]['pos_upper']
                pos_feedback = motor_data[motor_idx]['pos_feedback']
                
                if frame < len(pos_targets) and pos_targets[frame] != 0:
                    angle = get_joint_angle(motor_idx, pos_targets[frame])
                elif frame < len(pos_feedback) and pos_feedback[frame] != 0:
                    angle = get_joint_angle(motor_idx, pos_feedback[frame])
                else:
                    angle = 0
                
                joint_angles[joint_name] = angle
            
            leg_points = self.calculate_leg_points(leg, joint_angles)
            self.draw_leg(leg_points, leg)
        
        torque_values = []
        for motor_idx in range(12):
            torques = motor_data[motor_idx]['torque_target']
            if frame < len(torques):
                torque_values.append(torques[frame])
            else:
                torque_values.append(0)
        
        title = f'帧: {frame}/{len(motor_data[0]["pos_target"])}\n'
        title += f'扭矩: ' + " ".join([f"M{i+1}={t:.1f}" for i, t in enumerate(torque_values[:6])])
        self.ax.set_title(title, fontsize=10)
        
        return []
    
    def animate(self, motor_data, total_frames, output_path=None, frame_skip=3):
        print(f'[INFO] Creating animation ({total_frames} frames, skip={frame_skip})...')
        
        frames_to_show = list(range(0, total_frames, frame_skip))
        
        ani = animation.FuncAnimation(
            self.fig,
            self.update_frame,
            frames=frames_to_show,
            fargs=(motor_data, frame_skip),
            interval=50,
            blit=False
        )
        
        if output_path:
            print(f'[INFO] Saving animation to: {output_path}')
            ani.save(output_path, writer='pillow', fps=20)
            print(f'[INFO] Animation saved!')
        
        plt.show()
    
    def plot_data_summary(self, motor_data):
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        time_points = range(min(len(motor_data[i]['pos_target']) for i in range(12)))
        
        for motor_idx in range(12):
            label = f'M{motor_idx+1}'
            axes[0].plot(time_points, motor_data[motor_idx]['pos_target'][:len(time_points)], label=label)
        
        axes[0].set_title('位置目标 (0x131)')
        axes[0].set_xlabel('帧')
        axes[0].set_ylabel('角度 (°)')
        axes[0].legend(ncol=4, fontsize=8)
        axes[0].grid(True)
        
        for motor_idx in range(12):
            label = f'M{motor_idx+1}'
            axes[1].plot(time_points, motor_data[motor_idx]['torque_target'][:len(time_points)], label=label)
        
        axes[1].set_title('扭矩目标 (0x134)')
        axes[1].set_xlabel('帧')
        axes[1].set_ylabel('扭矩 (Nm)')
        axes[1].legend(ncol=4, fontsize=8)
        axes[1].grid(True)
        
        for motor_idx in range(12):
            label = f'M{motor_idx+1}'
            axes[2].plot(time_points, motor_data[motor_idx]['pos_upper'][:len(time_points)], label=label)
        
        axes[2].set_title('位置上限 (0x240)')
        axes[2].set_xlabel('帧')
        axes[2].set_ylabel('角度 (°)')
        axes[2].legend(ncol=4, fontsize=8)
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig('data_summary.png', dpi=150)
        print('[INFO] Data summary saved to data_summary.png')
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='四足机器狗3D步态仿真 - 完整版')
    parser.add_argument('file1', help='1-6电机CSV文件')
    parser.add_argument('file2', help='7-12电机CSV文件')
    parser.add_argument('-m', '--mode', default='both', choices=['3d', 'plot', 'both'], help='显示模式')
    parser.add_argument('-o', '--output', help='输出GIF文件')
    parser.add_argument('-s', '--skip', type=int, default=3, help='帧跳过数')
    
    args = parser.parse_args()
    
    print('='*60)
    print('四足机器狗3D步态仿真 - 完整指令数据版')
    print('='*60)
    
    print(f'\n[INFO] Parsing CSV files...')
    data1 = parse_csv(args.file1)
    data2 = parse_csv(args.file2)
    
    print('\n[INFO] Merging motor data...')
    merged_data = merge_motor_data(data1, data2)
    
    print('[INFO] Aligning data lengths...')
    aligned_data, total_frames = align_data_lengths(merged_data)
    
    print(f'\n[INFO] Total frames: {total_frames}')
    print('[INFO] Frame counts:')
    for i in range(12):
        print(f'  Motor {i+1}: pos_target={len(aligned_data[i]["pos_target"])}, '
              f'torque={len(aligned_data[i]["torque_target"])}, '
              f'pos_upper={len(aligned_data[i]["pos_upper"])}, '
              f'pos_feedback={len(aligned_data[i]["pos_feedback"])}')
    
    if args.mode in ['plot', 'both']:
        robot = QuadrupedRobot3D()
        robot.plot_data_summary(aligned_data)
    
    if args.mode in ['3d', 'both']:
        robot = QuadrupedRobot3D()
        
        output_3d = args.output + '_3d.gif' if args.output else 'simulation_3d.gif'
        robot.animate(aligned_data, total_frames, output_path=output_3d, frame_skip=args.skip)
    
    print('\n[INFO] Done!')

if __name__ == '__main__':
    main()
