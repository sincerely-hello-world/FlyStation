import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty

pipeline = (
    "v4l2src device=/dev/video0 ! "  # 指定视频设备
    "image/jpeg, width=640, height=480, framerate=30/1 ! "  # 设置分辨率和帧率
    "jpegdec ! "  # 新增解码器
    "videoconvert ! "  # 转换为 OpenCV 支持的格式
    "appsink"  # 输出到 OpenCV
)


class Camera(Node):

    def __init__(self, name, cap_index, frame_vel=0.03, topic_stack=10):
        super().__init__(name)

        self.name = name
        self.cap_index = cap_index
        self.frame_vel = frame_vel
        self.topic_stack = topic_stack

        self.server_open = self.create_service(Empty, name + "_open_service", self.service_camera_open_callback)
        self.server_close = self.create_service(Empty, name + "_close_service", self.service_camera_close_callback)

        self.pub_camera = self.create_publisher(Image, name + "_data_topic", topic_stack)

        self.cap = None
        self.height = 480   
        self.width = 640
        self.timer_pub_camera = None
        self.cv_bridge = CvBridge()

        self.get_logger().warning(
            f"camera initialize success!cap_index:{cap_index},frame_vel:{frame_vel},topic_stack:{topic_stack}")

    def service_camera_open_callback(self, request, response):
        if self.cap is not None or self.timer_pub_camera is not None:
            self.get_logger().warning(
                f"camera has already been opened!Please make sure to choose the right camera to open!")
            return response

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return response

        ret, frame = self.cap.read()
        if ret:
            frame = frame[0:480, 80:560]
            self.get_logger().warning("[camera test] read camera success")
            h, w, _ = frame.shape
            self.get_logger().warning(f"height:{h}px,width:{w}")
            self.timer_pub_camera = self.create_timer(self.frame_vel, self.timer_pub_camera_callback)
            self.get_logger().warning("start pub camera success")
        else:
            self.get_logger().error('[camera test] read camera fail')
        return response

    def service_camera_close_callback(self, request, response):
        if self.timer_pub_camera is not None:
            self.timer_pub_camera.cancel()
            self.timer_pub_camera = None
            self.get_logger().warning("stop pub image success")
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            self.get_logger().warning("close camera success")
            
        self.get_logger().warning("respond to close success")
        
        
        return response

    def timer_pub_camera_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = frame[0:480, 80:560]
            
            self.pub_camera.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
            # self.get_logger().info('pub image success')
        else:
            self.get_logger().error('pub image fail')
    
