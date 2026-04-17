#import uav_car_unit.camera as ca
#
#import rclpy
#
#
#def main(args=None):
#    rclpy.init(args=args)
#    camera = ca.Camera("camera", 0)
#    rclpy.spin(camera)
#    camera.destroy_node()
#    rclpy.shutdown()




import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty


width = 1280
height = 720

pipeline = (
    "v4l2src device=/dev/video0 ! "   
    f"image/jpeg, width={width}, height={height}, framerate=30/1 ! "  
    "jpegdec ! "   
    "videoconvert ! "  
    # "videoflip method=rotate-180 ! "
    "appsink"  
)


class Camera(Node):

    def __init__(self, name, cap_index, frame_vel=0.03, topic_stack=10):
        super().__init__(name)

        self.name = name
        self.ok = 0
        
        self.cap_index = cap_index
        self.frame_vel = frame_vel
        self.topic_stack = topic_stack

        self.server_open = self.create_service(Empty, name + "_open_service", self.service_camera_open_callback)
        self.server_close = self.create_service(Empty, name + "_close_service", self.service_camera_close_callback)

        self.pub_camera = self.create_publisher(Image, name + "_data_topic", topic_stack)  

        self.cap = None
        self.height = height   
        self.width = width
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
            self.get_logger().error("can't open carmera failed!")
            return response

        ret, frame = self.cap.read()
        if ret:
            #frame = frame[0:480, 80:560]
            h, w = frame.shape[:2]
            # 假设 h >= 480 且 w >= 480（你已经确认过）
            start_y = (h - 480) // 2
            start_x = (w - 480) // 2
            frame = frame[start_y : start_y + 480, start_x : start_x + 480]
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
            #frame = frame[0:480, 80:560]
            h, w = frame.shape[:2]
            # 假设 h >= 480 且 w >= 480（你已经确认过）
            start_y = (h - 480) // 2
            start_x = (w - 480) // 2
            frame = frame[start_y : start_y + 480, start_x : start_x + 480]
            self.pub_camera.publish(  self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8') )
            if self.ok < 1:
              self.ok = 1
              self.get_logger().info('pub image success')
        else:
            self.get_logger().error('pub image fail')
    
            
def main(args=None):
    rclpy.init(args=args)
    camera = Camera("camera", 0)
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
