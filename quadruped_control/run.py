#!/usr/bin/env python3
"""
四足机器狗上位机 - 数据监控模式
V5.0: 永久运行，接收数据，日志输出到文件

使用方法:
    python run.py                          # 数据监控模式（无模型）
    python run.py --model path/to/model.pt # 加载RL模型
    python run.py --port 9999              # 指定端口
    python run.py --ip 192.168.1.20        # 指定IP
"""

import sys
import os
import signal
import time
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tcp_client import QuadrupedTCPClient, get_logger


def main():
    parser = argparse.ArgumentParser(description='四足机器狗上位机 - 数据监控')
    parser.add_argument('--ip', type=str, default='0.0.0.0', help='监听IP (default: 0.0.0.0)')
    parser.add_argument('--port', '-p', type=int, default=9999, help='TCP端口 (default: 9999)')
    args = parser.parse_args()

    logger = get_logger()

    print("=" * 60)
    print("  四足机器狗上位机 - 数据监控模式 (V5.0)")
    print("  永久运行 | 断线自动等待重连 | 日志输出到 logs/tcp_log.md")
    print("=" * 60)

    tcp = QuadrupedTCPClient(host=args.ip, port=args.port)

    def on_connected():
        logger.log("[主] STM32连接成功，开始接收数据")
        tcp.enable_motors()
        tcp.enable_data_send(True)

    def on_disconnected():
        logger.log("[主] STM32断开，自动等待重连...")

    tcp.on_connected = on_connected
    tcp.on_disconnected = on_disconnected

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        print("\n[主] 接收到退出信号，停止服务器...")
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    tcp.start_server_permanent()
    logger.log("[主] 服务器已启动，按Ctrl+C退出")

    stop_event.wait()  # 阻塞直到Ctrl+C
    tcp.stop()


if __name__ == "__main__":
    main()
