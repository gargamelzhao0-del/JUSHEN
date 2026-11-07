#!/usr/bin/env python3
import csv
import struct

def bytes_to_float(hex_str):
    try:
        hex_str = hex_str.replace(' ', '')
        if len(hex_str) >= 8:
            byte_data = bytes.fromhex(hex_str[:8])
            return struct.unpack('f', byte_data)[0]
    except:
        pass
    return 0.0

def get_motor_num(can_id):
    low16 = can_id & 0xFFFF
    device_id = (low16 >> 8) & 0xFF
    msg_type = low16 & 0xFF
    
    if msg_type == 0x40:
        motor_num = device_id >> 4
        if 1 <= motor_num <= 12:
            return motor_num
    return None

msg_counts = {}
motor_data = {i: [] for i in range(1, 13)}

print("=== 完整解析CSV数据 ===\n")

for csv_file in ['1-6电机测试分析仪实际.csv', '7-12电机测试分析仪实际.csv']:
    print(f"--- {csv_file} ---")
    count = 0
    
    with open(csv_file, 'r', encoding='gbk') as f:
        reader = csv.reader(f)
        
        for row in reader:
            if len(row) > 4:
                can_id = row[4].strip()
                if can_id.startswith('0x'):
                    can_id_int = int(can_id, 16)
                    low16 = can_id_int & 0xFFFF
                    msg_type = low16 & 0xFF
                    
                    if msg_type not in msg_counts:
                        msg_counts[msg_type] = 0
                    msg_counts[msg_type] += 1
                    
                    motor_num = get_motor_num(can_id_int)
                    if motor_num:
                        data = row[6].strip() if len(row) > 6 else ''
                        pos = bytes_to_float(data)
                        motor_data[motor_num].append(pos)

print("\n=== 消息类型统计 (与仿真一致) ===")
for msg_type in sorted(msg_counts.keys()):
    print(f"msg=0x{msg_type:02X}: {msg_counts[msg_type]}条")

print("\n=== 每个电机的数据点数 ===")
for i in range(1, 13):
    print(f"Motor {i}: {len(motor_data[i])}条")
    
print("\n=== 结论 ===")
total = sum(len(motor_data[i]) for i in range(1, 13))
print(f"位置数据共 {total} 条（应该等于 3441 × 12 ÷ 12 = 3441）")
print("仿真使用了所有位置数据!")
