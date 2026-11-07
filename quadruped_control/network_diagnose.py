#!/usr/bin/env python3
"""
网络诊断脚本 - 检查TCP连接问题
Network Diagnostic Script - Check TCP Connection Issues
"""

import socket
import subprocess
import platform
import sys

def get_local_ips():
    """获取本机所有IP地址"""
    ips = []
    try:
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        ips.append(('hostname', local_ip))
    except:
        pass
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.append(('primary', s.getsockname()[0]))
        s.close()
    except:
        pass
    
    return ips

def check_port_available(port=9999):
    """检查端口是否可用"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('0.0.0.0', port))
        s.close()
        return True
    except Exception as e:
        return False, str(e)

def start_tcp_server(port=9999, timeout=10):
    """启动TCP服务器等待连接"""
    print(f"\n[TCP Server] 启动TCP服务器于 0.0.0.0:{port}")
    print(f"[TCP Server] 等待STM32连接... (超时: {timeout}秒)")
    print(f"[TCP Server] 嵌入式端应该连接到本机的IP地址")
    
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.settimeout(timeout)
        
        client, addr = server.accept()
        print(f"\n[TCP Server] ✓ 连接成功! 来自: {addr}")
        
        data = client.recv(1024)
        if data:
            print(f"[TCP Server] 收到数据: {len(data)} 字节")
            print(f"[TCP Server] HEX: {data.hex()}")
        
        client.close()
        server.close()
        return True
        
    except socket.timeout:
        print(f"\n[TCP Server] ✗ 等待连接超时")
        print("[TCP Server] 请检查:")
        print("  1. STM32是否已启动")
        print("  2. STM32配置的目标IP是否正确")
        print("  3. 网线是否连接正常")
        return False
    except Exception as e:
        print(f"\n[TCP Server] ✗ 错误: {e}")
        return False

def main():
    print("=" * 60)
    print("  网络诊断 - TCP连接问题排查")
    print("=" * 60)
    
    print("\n[1] 本机IP地址:")
    ips = get_local_ips()
    if ips:
        for name, ip in ips:
            print(f"    {name}: {ip}")
    else:
        print("    无法获取IP地址")
    
    print("\n[2] 端口检查 (9999):")
    result = check_port_available(9999)
    if result is True:
        print("    ✓ 端口9999可用")
    else:
        print(f"    ✗ 端口9999不可用: {result[1]}")
    
    print("\n[3] 嵌入式端配置:")
    print("    目标IP: 192.168.1.20")
    print("    目标端口: 9999")
    
    print("\n[4] 请确认:")
    print("    - Jetson Orin NX 的IP地址是否为 192.168.1.20?")
    print("    - 如果不是，需要修改嵌入式端代码中的 remote_ip")
    
    local_ips = [ip for name, ip in ips]
    if '192.168.1.20' not in local_ips:
        print("\n    ⚠️  警告: 本机IP不是 192.168.1.20!")
        print("    本机IP列表:", local_ips)
        print("\n    解决方案:")
        print("    1. 修改嵌入式端 freertos.c 中的 remote_ip 为正确的IP")
        print("    2. 或者设置Jetson的静态IP为 192.168.1.20")
    
    print("\n" + "=" * 60)
    print("  启动TCP服务器测试...")
    print("=" * 60)
    
    start_tcp_server(9999, timeout=15)

if __name__ == "__main__":
    main()
