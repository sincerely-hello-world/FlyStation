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
from geometry_msgs.msg import Point

def open_camera_with_resolution(index:int, width:int, height:int):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头")

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap 

def rotate_frame(frame, angleClockwise:int = 180):
    """顺时针旋转图像"""
    if angleClockwise == 0 or angleClockwise == 360:
        return frame
    elif angleClockwise == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angleClockwise == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angleClockwise == 270 or angleClockwise == -90:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        # 非90度倍数的旋转，使用仿射变换
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        matrix = cv2.getRotationMatrix2D(center, angleClockwise, 1.0)
        rotated = cv2.warpAffine(frame, matrix, (w, h))
        return rotated

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
    cropped_frame = frame[top:bottom, left:right]
    return cropped_frame







# 1. 定义数据结构
# from dataclasses import dataclass, field
from typing import Optional, Tuple

 
class DetectedObject:
    def __init__(self, offset_x=0, offset_y=0,color_name="orange"):
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.color_name=color_name

import cv2
import numpy as np

def detect_yellow_block(frame):
    """
    ROS2 坐标系定制版：锁定LED光斑，并输出符合ROS2（X前, Y左）定义的相对画面中心偏移量
    画面中心: (0, 0)
    左上角: X+, Y+ (向前且向左)
    左下角: X-, Y+ (向后且向左)
    """
    # 1. 获取画面尺寸并计算中心点
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    # 2. 转换为 HSV 空间并提取橙黄色掩膜
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([10, 90, 90])
    upper_orange = np.array([25, 240, 240])
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # 补充：寻找最大色块
    # contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if not contours:  return frame, None  # 未检测到任何色块
    # largest_contour = max(contours, key=cv2.contourArea)
    # x, y, w, h = cv2.boundingRect(largest_contour)
    # roi_mask = orange_mask[y:y+h, x:x+w]
    
    # 3. 【严谨去噪组合拳】
    kernel_small = np.ones((1, 1), np.uint8)
    kernel_large = np.ones((15, 15), np.uint8) 
    kernel_restore = np.ones((2, 2), np.uint8)

    eroded_1 = cv2.erode(orange_mask, kernel_small, iterations=1)
    dilated = cv2.dilate(eroded_1, kernel_large, iterations=1)
    eroded_2 = cv2.erode(dilated, kernel_restore, iterations=1)
    processed_mask = eroded_2

    # 4. 【Canny 边缘检测】
    canny_edges = cv2.Canny(processed_mask, 50, 150)

    # 5. 寻找轮廓
    contours, _ = cv2.findContours(canny_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected_object = None # []

    # 6. 过滤并锁定目标
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area > 30 and area < 500 :
            # 使用几何矩计算精确重心 (图像像素坐标系)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                blob_cx = int(M["m10"] / M["m00"])
                blob_cy = int(M["m01"] / M["m00"])
            else:
                x, y, w, h = cv2.boundingRect(largest_contour)
                blob_cx, blob_cy = x + w // 2, y + h // 2
            
            # ========================================================
            # 【核心修改：转换到 ROS2 2D 坐标系 (X前, Y左)】
            # 图像向上 (blob_cy 变小) -> ROS2 X变大 (Forward)
            offset_x = center_y - blob_cy 
            
            # 图像向左 (blob_cx 变小) -> ROS2 Y变大 (Left)
            offset_y = center_x - blob_cx
            # ========================================================
            detected_object = DetectedObject(offset_x,offset_y)
            # --- 调试可视化 ---
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            ctr = (int(x), int(y))
            r = int(radius)
            
            # 画出包裹圆和中心
            cv2.circle(frame, ctr, r, (0, 255, 0), 2)                  # 绿色圆圈
            cv2.circle(frame, (blob_cx, blob_cy), 5, (0, 0, 255), -1)    # 红色点：光斑质心
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)   # 蓝色点：画面中心
            
            # 绘制自定义的 ROS2 坐标轴示意 (视觉Debug用)
            # 蓝色线指向上方（X+ 前），绿色线指向左方（Y+ 左）
            cv2.line(frame, (center_x, center_y), (center_x, center_y - 40), (255, 0, 0), 2) # X轴
            cv2.line(frame, (center_x, center_y), (center_x - 40, center_y), (0, 255, 0), 2) # Y轴
            
            # 实时打印符合 ROS2 定义的偏移数据
            info_text = f"ROS2 X(Fwd): {offset_x}, Y(Lft): {offset_y}"
            cv2.putText(frame, info_text, 
                        (blob_cx + r + 10, blob_cy), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return frame, detected_object

import cv2
import numpy as np

def detect_black_disk(frame):
    """
    ROS2 坐标系定制版：锁定白色背景上的黑色圆盘，并输出相对画面中心偏移量
    画面中心: (0, 0)
    左上角: X+, Y+ (向前且向左)
    左下角: X-, Y+ (向后且向左)
    """
    # 1. 获取画面尺寸并计算中心点
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    # 2. 【灰度化】
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 3. 【腐蚀与膨胀】去噪
    # 注：对灰度图而言，黑色是低亮度值。腐蚀会使暗色区域扩大，膨胀会使亮色区域扩大。
    kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(gray, kernel, iterations=1)
    dilated = cv2.dilate(eroded, kernel, iterations=1)

    # 4. 【高斯去噪】平滑边缘，提升霍夫圆检测的准确率
    blurred = cv2.GaussianBlur(dilated, (9, 9), 2)

    # 5. 【霍夫圆变换】检测图像中的圆形
    # param1: Canny边缘检测的高阈值
    # param2: 累加器阈值，值越小检测到的圆越多（但也越容易误报），50 比较适合强对比度场景
    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1, 
        minDist=250,          # 两个圆心之间的最小距离
        param1=100, 
        param2=60, 
        minRadius=35,        # 限制最小圆半径
        maxRadius=min(width, height) // 2 # 限制最大圆半径
    )

    detected_object = None

    # 6. 过滤并锁定目标（按最大面积/半径筛选）
    if circles is not None:
        # 将结果转换为整数 [cx, cy, r]
        circles = np.around(circles)[0, :]
        
        # 【按最大面积/半径筛选】找出其中半径 r 最大的圆盘
        largest_circle = max(circles, key=lambda c: c[2])
        blob_cx, blob_cy, r = int(largest_circle[0]), int(largest_circle[1]), int(largest_circle[2])

        # ========================================================
        # 【ROS2 2D 坐标系转换 (X前, Y左)】
        # 图像向上 (blob_cy 变小) -> ROS2 X变大 (Forward)
        offset_x = center_y - blob_cy 
        
        # 图像向左 (blob_cx 变小) -> ROS2 Y变大 (Left)
        offset_y = center_x - blob_cx
        # ========================================================
        
        # 实例化你的 ROS2 目标对象
        detected_object = DetectedObject(offset_x, offset_y)

        # --- 调试可视化 ---
        # 画出包裹圆和中心
        cv2.circle(frame, (blob_cx, blob_cy), r, (0, 255, 0), 2)                  # 绿色圆圈：检测到的圆盘
        cv2.circle(frame, (blob_cx, blob_cy), 5, (0, 0, 255), -1)                # 红色点：圆盘质心
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)               # 蓝色点：画面中心
        
        # 绘制自定义的 ROS2 坐标轴示意 (视觉Debug用)
        cv2.line(frame, (center_x, center_y), (center_x, center_y - 40), (255, 0, 0), 2) # X轴（蓝线指前）
        cv2.line(frame, (center_x, center_y), (center_x - 40, center_y), (0, 255, 0), 2) # Y轴（绿线指左）
        
        # 实时打印符合 ROS2 定义的偏移数据
        info_text = f"ROS2 X(Fwd): {offset_x}, Y(Lft): {offset_y}"
        cv2.putText(frame, info_text, 
                    (blob_cx + r + 10, blob_cy), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return frame, detected_object

def myself_offset_process(x, y):
    # 计算 x 和 y 绝对值中的最大值（即切比雪夫距离）
    if max(abs(x), abs(y)) < 66:
        return float(0),float(0)
    # 如果超出范围，返回原坐标（或你需要的其他处理结果）
    return float(x), float(y)
    
class camNode(Node):
    def __init__(self,name='camNode'):
        super().__init__(name)
        self.camIndex = 0    # 摄像头编号
        self.camWidth = 1280 # 请求的分辨率
        self.camHeight = 720
        self.crop_size = 520 # 截取摄像头中心区域
        self.crop_offsetx = 0
        self.crop_offsety = -120
        self.angleClockwise = 180 # 镜头顺时针旋转角度

        self.client_led1_trigger = self.create_client(Trigger,"led1_trigger")
        self.client_led2_trigger = self.create_client(Trigger,"led2_trigger")
 
        # 发布
        self.topic_pub_camera_data = self.create_publisher(Point, 'fly/camera/data', 10)

        # 创建 cv_bridge 实例 
        self.bridge = CvBridge()

        # 打开USB摄像头（索引0，根据实际情况调整）
        self.cap = open_camera_with_resolution(self.camIndex, self.camWidth, self.camHeight)

        # 获取摄像头属性（注意：width 是 CAP_PROP_FRAME_WIDTH，height 是 CAP_PROP_FRAME_HEIGHT）
        real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # 打印到 ROS 日志（推荐）
        self.get_logger().info(f"real摄像头 {self.camIndex} 分辨率：请求 {self.camWidth}x{self.camHeight} → 实际 {real_w}x{real_h}")
        self.get_logger().info(f"摄像头初始化分辨率：{self.camWidth}x{self.camHeight}")
        self.get_logger().info(f"real摄像头裁剪中心后大小 {self.crop_size}*{self.crop_size}")
        self.get_logger().info("摄像头节点已启动，按 Ctrl+C 退出")

        # 这里每秒检测30帧（约33ms一次）
        self.cam_timer = self.create_timer(0.033, self.cam_timer_callback)

    def cam_timer_callback(self):
        ret, frame = self.cap.read()
        
        frame = crop_center_with_offset(frame, self.crop_size,self.crop_offsetx,self.crop_offsety)
        frame = rotate_frame(frame, self.angleClockwise)
        if not ret:
            self.get_logger().error("无法读取帧")
            return

        frame,objects = detect_black_disk(frame)
        if objects is not None:
            msg = Point()
            msg.x,msg.y = myself_offset_process(objects.offset_x , objects.offset_y)
            self.topic_pub_camera_data.publish(msg)
            self.get_logger().info(f'cam detected -> X: {msg.x:.1f}, Y: {msg.y:.1f}')

            self.client_led1_trigger.call_async(Trigger.Request())
            self.client_led2_trigger.call_async(Trigger.Request())
            
        # cv2.imshow('USB Camera vedio', frame)
        # cv2.waitKey(1) # ← 关键！1ms 就够了，不会明显卡顿  # 必须有！否则窗口不刷新

    def destroy_node(self):
        # 释放资源
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = camNode()
        # 推荐使用spin方式（阻塞式，自动处理回调）
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback # 在文件顶部导入
        print(f"发生异常: {e}")
        traceback.print_exc()  # 这会打印出完整的报错堆栈信息

    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

