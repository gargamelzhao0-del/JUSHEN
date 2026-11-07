#!/usr/bin/env python3
import csv
import struct

def get_motor_idx(can_id):
    low16 = can_id & 0xFFFF
    device_id = (low16 >> 8) & 0xFF
    msg_type = low16 & 0xFF
    
    if msg_type == 0x40:
        motor_num = device_id >> 4
        if 1 <= motor_num <= 12:
            return motor_num - 1
    return None

def bytes_to_float(hex_str):
    try:
        hex_str = hex_str.replace(' ', '')
        if len(hex_str) >= 8:
            byte_data = bytes.fromhex(hex_str[:8])
            return struct.unpack('f', byte_data)[0]
    except:
        pass
    return 0.0

print("=== 验证解析 msg=0x40 (实际位置反馈) ===\n")

print("--- 1-6电机CSV ---")
counts = [0]*12
positions = [[] for _ in range(12)]
with open('1-6电机测试分析仪实际.csv', 'r', encoding='utf-8', errors='ignore') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) < 7:
            continue
        try:
            can_id = int(row[4].strip(), 16)
            idx = get_motor_idx(can_id)
            if idx is not None:
                pos = bytes_to_float(row[6].strip())
                positions[idx].append(pos)
                counts[idx] += 1
        except:
            pass

for i in range(12):
    if counts[i] > 0:
        print(f"Motor {i+1}: {counts[i]} frames, pos范围={min(positions[i]):.2f}~{max(positions[i]):.2f}°")

print("\n--- 7-12电机CSV ---")
counts = [0]*12
positions = [[] for _ in range(12)]
with open('7-12电机测试分析仪实际.csv', 'r', encoding='utf-8', errors='ignore') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) < 7:
            continue
        try:
            can_id = int(row[4].strip(), 16)
            idx = get_motor_idx(can_id)
            if idx is not None:
                pos = bytes_to_float(row[6].strip())
                positions[idx].append(pos)
                counts[idx] += 1
        except:
            pass

for i in range(12):
    if counts[i] > 0:
        print(f"Motor {i+7}: {counts[i]} frames, pos范围={min(positions[i]):.2f}~{max(positions[i]):.2f}°")

print("\n\n=== 数据样例 (前10帧) ===")
print("--- 1-6电机CSV ---")
with open('1-6电机测试分析仪实际.csv', 'r', encoding='utf-8', errors='ignore') as f:
    reader = csv.reader(f)
    frame_count = 0
    for row in reader:
        if len(row) < 7:
            continue
        try:
            can_id = int(row[4].strip(), 16)
            idx = get_motor_idx(can_id)
            if idx is not None and frame_count < 10:
                pos = bytes_to_float(row[6].strip())
                print(f"  Motor {idx+1}: pos={pos:.2f}°")
                frame_count += 1
        except:
            pass
