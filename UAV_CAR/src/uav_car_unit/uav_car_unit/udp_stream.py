#!/usr/bin/env python3
"""
UDP图像传输发送端
高效的图像数据传输，专为无人机视觉系统优化
"""
import cv2
import socket
import struct
import time
import threading
import queue
import zlib
from typing import Optional, Tuple


class UDPImageSender:
    """UDP图像发送器 - 高性能低延迟"""
    def __init__(self, target_ip: str, target_port: int = 5001,
                 jpeg_quality: int = 80, max_packet_size: int = 1400):
        """
        初始化UDP图像发送器
        Args:
            target_ip: 目标IP地址（地面站IP）
            target_port: 目标端口
            jpeg_quality: JPEG压缩质量 (1-100)
            max_packet_size: 最大UDP包大小 (建议1400字节以避免分片)
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.jpeg_quality = jpeg_quality
        self.max_packet_size = max_packet_size
        self.header_size = 20  # 包头大小
        self.max_payload = max_packet_size - self.header_size
        # 网络相关
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 设置发送缓冲区大小
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        # 帧队列和发送线程
        self.frame_queue = queue.Queue(maxsize=3)  # 只保留最新3帧
        self.running = threading.Event()
        self.sender_thread = None
        # 帧计数器
        self.frame_id = 0
        # 启动发送线程
        self.start_sender()
        print(f"[INFO] UDP图像发送器已启动")
        print(f"[INFO] 目标地址: {target_ip}:{target_port}")
        print(f"[INFO] JPEG质量: {jpeg_quality}%, 最大包大小: {max_packet_size}字节")

    def pack_header(self, frame_id: int, packet_id: int, total_packets: int,
                   stream_id: int, payload_size: int, timestamp: float) -> bytes:
        """
        打包UDP包头
        格式: frame_id(4) + packet_id(2) + total_packets(2) + stream_id(1) +
              payload_size(2) + timestamp(8) + reserved(1) = 20字节
        """
        return struct.pack(
            '>I H H B H d B',
            frame_id,           # 帧ID (4字节)
            packet_id,          # 包ID (2字节)
            total_packets,      # 总包数 (2字节)
            stream_id,          # 流ID (1字节)
            payload_size,       # 负载大小 (2字节)
            timestamp,          # 时间戳 (8字节)
            0                   # 保留字段 (1字节)
        )

    def send_frame(self, frame, stream_id: int = 1) -> bool:
        """
        发送图像帧（非阻塞）
        Args:
            frame: OpenCV图像
            stream_id: 流ID，用于区分不同视频流
        Returns:
            bool: 成功加入发送队列返回True
        """
        if not self.running.is_set():
            return False
        try:
            # 如果队列满了，丢弃最旧的帧
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            # 加入发送队列
            self.frame_queue.put_nowait((frame.copy(), stream_id, time.time()))
            return True
        except queue.Full:
            return False

    def _send_frame_immediate(self, frame, stream_id: int, timestamp: float):
        """立即发送帧数据"""
        try:
            # 1. 压缩图像
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_data = cv2.imencode('.jpg', frame, encode_param)
            if not success:
                print("[WARNING] 图像编码失败")
                return False
            # 2. 可选：使用zlib进一步压缩（针对低质量JPEG效果不明显，可禁用）
            # compressed_data = zlib.compress(encoded_data.tobytes(), level=1)
            compressed_data = encoded_data.tobytes()
            # 3. 分包发送
            data_size = len(compressed_data)
            total_packets = (data_size + self.max_payload - 1) // self.max_payload
            self.frame_id = (self.frame_id + 1) % 0xFFFFFFFF
            for packet_id in range(total_packets):
                start_idx = packet_id * self.max_payload
                end_idx = min(start_idx + self.max_payload, data_size)
                payload = compressed_data[start_idx:end_idx]
                # 打包数据
                header = self.pack_header(
                    self.frame_id, packet_id, total_packets,
                    stream_id, len(payload), timestamp
                )
                packet = header + payload
                # 发送UDP包
                self.socket.sendto(packet, (self.target_ip, self.target_port))

            return True
        except Exception as e:
            print(f"[ERROR] 发送帧失败: {e}")
            return False

    def start_sender(self):
        """启动发送线程"""
        if self.sender_thread and self.sender_thread.is_alive():
            return
        self.running.set()
        self.sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.sender_thread.start()

    def _sender_loop(self):
        """发送线程主循环"""
        print("[INFO] UDP发送线程已启动")
        while self.running.is_set():
            try:
                # 从队列获取帧数据
                try:
                    frame, stream_id, timestamp = self.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                # 发送帧
                self._send_frame_immediate(frame, stream_id, timestamp)
            except Exception as e:
                print(f"[ERROR] 发送线程错误: {e}")
                time.sleep(0.1)
        print("[INFO] UDP发送线程已退出")

    def stop(self):
        """停止发送器"""
        self.running.clear()
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join(timeout=2.0)
        if self.socket:
            self.socket.close()
        print("[INFO] UDP图像发送器已停止")

    def __del__(self):
        """析构函数"""
        self.stop()