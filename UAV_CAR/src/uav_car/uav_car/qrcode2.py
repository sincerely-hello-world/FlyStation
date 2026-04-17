# import cv2
# from pyzbar.pyzbar import decode

# # 打开USB摄像头（0表示默认摄像头，根据实际情况可能需要调整索引）
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("无法打开摄像头")
#     exit()

# print("按 'q' 键退出程序")

# while True:
#     # 读取一帧画面
#     ret, frame = cap.read()
#     if not ret:
#         print("无法读取帧")
#         break

#     # 对当前帧进行QR码识别
#     decoded_objects = decode(frame)
#     for obj in decoded_objects:
#         # 打印识别到的QR码数据
#         print("QR码类型:", obj.type)
#         print("QR码数据:", obj.data.decode('utf-8'))
        
#         # 在帧上绘制矩形框（可选，用于可视化）
#         pts = obj.polygon
#         if len(pts) > 4:
#             hull = cv2.convexHull(np.array([point for point in pts], dtype=np.float32))
#             hull = list(map(tuple, np.squeeze(hull)))
#         else:
#             hull = pts
#         n = len(hull)
#         for j in range(0, n):
#             cv2.line(frame, hull[j], hull[(j + 1) % n], (0, 255, 0), 3)

#     # 显示当前帧
#     cv2.imshow('USB Camera with QR Detection', frame)

#     # 按 'q' 键退出循环
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # 释放资源
# cap.release()
# cv2.destroyAllWindows()


#----------------------------
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np
from pyzbar.pyzbar import decode,ZBarSymbol

from std_srvs.srv import Trigger

class QRCodeDetectorNode(Node):
    def __init__(self,name='qrcode2'):
        super().__init__(name)
        self.client_led2_trigger = self.create_client(Trigger,"led2_trigger")
        # 创建图像QRimage 发布者（话题名可自定义，例如 /camera/image_annotated）
        # self.topic_publisher_qrimage = self.create_publisher(
        #     Image,
        #     '/image/image_qrcode2',   # ← 推荐命名：带处理结果的图像
        #     10
        # )
 
        self.topic_publisher_qrcompressedimage2 = self.create_publisher(
            CompressedImage,
            '/image/image_qrcode2/compressed',   # ← 推荐命名：带处理结果的图像
            10
        )
        # 创建发布者 QRcode 发布者（保持不变）
        self.topic_publisher_qrcode2 = self.create_publisher(String, 'qrcode2_data_topic', 10)
    

        # 创建 cv_bridge 实例（只需创建一次）
        self.bridge = CvBridge()

        # 打开USB摄像头（索引0，根据实际情况调整）
        self.cap = self.open_camera_with_resolution(2)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            raise RuntimeError("摄像头打开失败")
        # ─────────────── 关键：在初始化时读取一次分辨率 ───────────────
        # 获取摄像头属性（注意：width 是 CAP_PROP_FRAME_WIDTH，height 是 CAP_PROP_FRAME_HEIGHT）
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 打印到 ROS 日志（推荐）
        self.get_logger().info(f"摄像头初始化分辨率：{width} × {height}")
        self.get_logger().info("QR码检测节点已启动，按 Ctrl+C 退出")

        self.crop_size = 480 # 600 # 480
        self.get_logger().info(f"real摄像头裁剪后大小 {self.crop_size}*{self.crop_size} 分辨率")

        # 创建定时器（相当于ROS的循环频率）
        # 这里每秒检测30帧（约33ms一次）
        self.qr_timer = self.create_timer(0.033, self.qr_timer_callback)

        

    def open_camera_with_resolution(self,index=2, width=1280, height=720):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            raise RuntimeError("无法打开摄像头")

        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 读取实际值（很多摄像头不会严格按照你设置的来）
        real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"real摄像头 {index} 分辨率：请求 {width}x{height} → 实际 {real_w}x{real_h}")
        return cap 

    def qr_timer_callback(self):
        ret, frame = self.cap.read()
        
        frame = crop_center_with_offset(frame, self.crop_size,0,0)
        if not ret:
            self.get_logger().error("无法读取帧")
            return

        # QR码检测
        decoded_objects = decode(frame,symbols=[ZBarSymbol.QRCODE])
        for obj in decoded_objects:
            try:
                qrdata = obj.data.decode('utf-8')
                self.get_logger().info(f"QRCODE2 NODE 识别到QR码: {qrdata}")

                # 发布消息
                msg = String()
                msg.data = qrdata
                self.topic_publisher_qrcode2.publish(msg)
                self.client_led2_trigger.call_async(Trigger.Request())
                # # 在帧上绘制矩形框（可选，用于可视化）
                # pts = obj.polygon
                # if len(pts) > 4:
                #     hull = cv2.convexHull(np.array([point for point in pts], dtype=np.float32))
                #     hull = list(map(tuple, np.squeeze(hull)))
                # else:
                #     hull = pts
                # n = len(hull)
                # for j in range(0, n):
                #     cv2.line(frame, hull[j], hull[(j + 1) % n], (0, 255, 0), 3)
            except Exception as e:
                self.get_logger().warn(f"QR码解码失败: {e}")

        decode_img = None
        # try:
        #     # # 最常用编码："bgr8"（因为 cv2 默认是 BGR 通道顺序）
        #     # ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        #     # # 加上时间戳和 frame_id（强烈推荐，rviz 需要）
        #     # ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        #     # ros_image_msg.header.frame_id = "image_qrcode"   # 可改成你的坐标系名
        #     # self.topic_publisher_qrimage.publish(ros_image_msg)

        #     ros_compressed_image_msg = CompressedImage()
        #     success,encoded_image = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY, 43])
        #     ros_compressed_image_msg.data= np.array(encoded_image).tobytes()
        #     # if success: # cv2 m
        #     #     # 编码成功才执行解码逻辑
        #     #     nparr = np.frombuffer(ros_compressed_image_msg.data, np.uint8)
        #     #     decode_img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        #     # bridge m
        #     # ros_compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame, 'jpg') 

        #     ros_compressed_image_msg.header.stamp=self.get_clock().now().to_msg()
        #     ros_compressed_image_msg.header.frame_id = "compressedimage_qrcode2"   # 可改成你的坐标系名
        #     ros_compressed_image_msg.format='jpeg'
            
        #     self.topic_publisher_qrcompressedimage.publish(ros_compressed_image_msg)

        # except Exception as e:
        #     self.get_logger().error(f"image 转换失败: {e}")

        # # ─────────────── 本地调试显示（可选） ───────────────
        # # decode_img=self.bridge.compressed_imgmsg_to_cv2(ros_compressed_image_msg)
        # # cv2.imshow('USB Camera with QR Detection', decode_img)

        # # decode_img=self.bridge.imgmsg_to_cv2(ros_image_msg)
        # # cv2.imshow('USB Camera with QR Detection', decode_img)
        # # cv2.imshow('USB Camera vedio2', frame)
        # # cv2.waitKey(1) # ← 关键！1ms 就够了，不会明显卡顿  # 必须有！否则窗口不刷新

    def destroy_node(self):
        # 释放资源
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = QRCodeDetectorNode()
        # 推荐使用spin方式（阻塞式，自动处理回调）
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

def crop_center_with_offset(frame, crop_size, offset_x=0, offset_y=0):
    """
    从 frame 中裁剪一个 crop_size x crop_size 的区域，
    裁剪中心默认为图像中心，可通过 offset_x / offset_y 偏移（像素）。
    
    参数:
        frame (np.ndarray): 输入图像 (H, W, C)
        crop_size (int): 裁剪区域的宽高（正方形）
        offset_x (int): 相对于中心点的水平偏移（正：向右，负：向左）
        offset_y (int): 相对于中心点的垂直偏移（正：向下，负：向上）
    
    返回:
        cropped (np.ndarray): 裁剪后的图像，若原图小于 crop_size，则自动缩小裁剪区域（不放大）
                            若无法裁剪（如 crop_size <= 0），返回原图或空数组。
    """
    if crop_size <= 0:
        raise ValueError("crop_size 必须为正整数")
    
    h, w = frame.shape[:2]
    
    # 实际可用的最大裁剪尺寸（不能超过原图）
    actual_crop = min(crop_size, w, h)
    
    # 原始中心点
    center_x = w // 2
    center_y = h // 2

    # 应用偏移后的目标中心
    target_cx = center_x + offset_x
    target_cy = center_y + offset_y

    # 计算裁剪边界（以目标中心为中心）
    half = actual_crop // 2
    left = target_cx - half
    right = left + actual_crop
    top = target_cy - half
    bottom = top + actual_crop

    # 边界校正：确保 [left, right) 和 [top, bottom) 在 [0, w) 和 [0, h) 范围内
    if left < 0:
        left = 0
        right = actual_crop
    if right > w:
        right = w
        left = w - actual_crop
    if top < 0:
        top = 0
        bottom = actual_crop
    if bottom > h:
        bottom = h
        top = h - actual_crop
    cropped = frame[top:bottom, left:right]
    return cropped
