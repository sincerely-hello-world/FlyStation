


import numpy as np

import serial,platform

import pyrealsense2 as rs 
from uav_car.tofsense import TOFSense_P_F
 
import rclpy
from rclpy.node import Node 
from std_srvs.srv import Empty
from std_msgs.msg import String
from uav_car_interfaces.msg import T265Data

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup# 读串口操作，禁用ReentrantCallbackGroup，避免同一回调创建多个串口实例，导致崩溃

from uav_car.my_format import TGformat


class T265(Node):
    def __init__(self, name='t265', pub_cycle = 0.02,   topic_stack = 10):
        super().__init__(name)
        
        self.ok = 0
        self.name = name
        self.pub_cycle = pub_cycle
        self.topic_stack = topic_stack
 
        self.t265_callback_group = MutuallyExclusiveCallbackGroup()
        self.ToFsensor_callback_group = MutuallyExclusiveCallbackGroup() # 读串口千万别用ReentrantCallbackGroup， 同一回调会创建多个读串口实例，直接崩溃

        self.t265_pubMsg = T265Data()
        self.uart3_pubMsg = String()

        # t265 变量成员
        self.t265_pipe = None
        self.t265_cfg = None 
        self.timer_pub_t265 = None
        
        self.topic_uart3_pub = self.create_publisher(String, 'uart_sender3_data_topic', topic_stack) 
        self.topic_uart4_pub = self.create_publisher(String, 'uart_sender4_data_topic', topic_stack) 
        self.topic_t265_pub = self.create_publisher(T265Data, 't265_data_topic', topic_stack) 

        self.server_t265_open = self.create_service(Empty, 't265_open_service', self.service_t265_open_callback) # 1
        self.server_t265_close = self.create_service(Empty, 't265_close_service', self.service_t265_close_callback)  # 2
        
        # TOFSense 激光测距
        self.uart1 = serial.Serial("/dev/ttyS1",921600)
        self.TOF = TOFSense_P_F(self.uart1)
        self.ToFsensor_z = 0.0
        self.ToFsensor_init_z = 0.0
        self.ToFsensor_z_valid = False  # 初始无效
        self.timer_ToFsensor = self.create_timer(0.025, self.ToFsensor_read, callback_group = self.ToFsensor_callback_group)  
        self.get_logger().info(f'ToFsensor start ok')
        
        self.get_logger().info(f't265 node start  ok new')
        self.timer_log = self.create_timer(1, self.mylog)  

    def service_t265_open_callback(self, request, response):
        self.get_logger().warning("enter pub t265 success===============new")   
        if self.timer_pub_t265 is not None:
            self.get_logger().warning("t265 has already been started!Check for safety use!")   
            return response
        self.t265_pipe = rs.pipeline()
        self.t265_cfg = rs.config()
        self.t265_cfg.enable_stream(rs.stream.pose)
        self.t265_pipe.start(self.t265_cfg)
        self.timer_pub_t265 = self.create_timer(0.02, callback = self.T265_read,callback_group = self.t265_callback_group)
        self.get_logger().warning("start pub t265 success---------------")   
        return response

    def ToFsensor_read(self):
      data = self.TOF.get_data()
      if isinstance(data, dict) and (dis := data.get("dis", 0)) > 0:
          if self.ToFsensor_init_z == 0.0:
            self.ToFsensor_init_z = dis
            self.get_logger().info(f'ToFsensor laser-init h[{self.ToFsensor_init_z:+6.3f}m success]')
          self.ToFsensor_z = dis - self.ToFsensor_init_z
          self.ToFsensor_z_valid = True
          self.t265_pubMsg.tof_z = self.ToFsensor_z
      else: 
          self.get_logger().warning("ToFsensor 掉电/错误连接/读取数据错误, failed")
        
        
    def T265_read(self):
        frames = self.t265_pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
            data = pose.get_pose_data()      
            self.t265_pubMsg.pos_x = data.translation.x
            self.t265_pubMsg.pos_y = data.translation.y
            self.t265_pubMsg.pos_z = data.translation.z
            
            # self.t265_pubMsg.pos_orient_x = data.rotation.x
            # self.t265_pubMsg.pos_orient_y = data.rotation.y
            # self.t265_pubMsg.pos_orient_z = data.rotation.z
            # self.t265_pubMsg.pos_orient_w = data.rotation.w

            # self.t265_pubMsg.vel_linear_x = data.velocity.x
            # self.t265_pubMsg.vel_linear_y = data.velocity.y
            # self.t265_pubMsg.vel_linear_z = data.velocity.z
            # self.t265_pubMsg.vel_angular_x = data.angular_velocity.x
            # self.t265_pubMsg.vel_angular_y = data.    
            # self.t265_pubMsg.vel_angular_y = data.angular_velocity.y
            # self.t265_pubMsg.vel_angular_z = data.angular_velocity.z

            # self.t265_pubMsg.acc_linear_x = data.acceleration.x
            # self.t265_pubMsg.acc_linear_y = data.acceleration.y
            # self.t265_pubMsg.acc_linear_z = data.acceleration.z
            # self.t265_pubMsg.acc_angular_x = data.angular_acceleration.x
            # self.t265_pubMsg.acc_angular_y = data.angular_acceleration.y
            # self.t265_pubMsg.acc_angular_z = data.angular_acceleration.z

            # self.t265_pubMsg.frame_id = "t265_camera"
            # self.t265_pubMsg.stamp = self.get_clock().now().to_msg()
            
            self.t265_pubMsg.confidence = data.tracker_confidence
            self.t265_pubMsg = self.data_process(self.t265_pubMsg)
            self.topic_t265_pub.publish(self.t265_pubMsg)

            if self.ok < 1:
                self.ok = 1
                self.get_logger().info(f't265 init ok')
                self.get_logger().info(f't265 初始化成功， 读到了数据')
                self.get_logger().info(
                    f't265 confidence:[{self.t265_pubMsg.confidence:1d}], '
                    f'pos(x:{self.t265_pubMsg.pos_x:+6.3f}m, y:{self.t265_pubMsg.pos_y:+6.3f}m), '
                    f'z[{self.t265_pubMsg.pos_z:+6.3f}m]')
            # self.uart3_pubMsg.data = t265_data_to_uart_std("54", self.t265_pubMsg) # hex古法发送
            if self.t265_pubMsg.confidence >= 0:
                # self.uart3_pubMsg.data = TGformat('T',self.t265_pubMsg.pos_x, self.t265_pubMsg.pos_y, self.t265_pubMsg.tof_z, '') # 使用激光定高数据！
                self.uart3_pubMsg.data = TGformat('T',self.t265_pubMsg.pos_x, self.t265_pubMsg.pos_y, self.t265_pubMsg.pos_z, '') # 使用T265定高
                self.topic_uart3_pub.publish(self.uart3_pubMsg) # send t265 data to uart3

    def mylog(self):
            self.get_logger().info(
                    f't265 confidence:[{self.t265_pubMsg.confidence:1d}], '
                    f'pos(x:{self.t265_pubMsg.pos_x:+6.3f}m, y:{self.t265_pubMsg.pos_y:+6.3f}m, z:{self.t265_pubMsg.pos_z:+6.3f}m), '
                    f'tof_z[{self.t265_pubMsg.tof_z:+6.3f}m]')
                    
                    
    def service_t265_close_callback(self, request, response):
        if self.timer_pub_t265 is not None:
            self.timer_pub_t265.cancel()
            self.timer_pub_t265 = None
        self.t265_pipe.stop()
        self.t265_pipe = None
        self.t265_cfg = None
        self.get_logger().warning(f't265 close success!')
        return response

    def data_process(self, msg):
        msg.pos_x = msg.pos_x * (-1)
        # swap y and z
        tmp = msg.pos_y
        msg.pos_y = msg.pos_z
        msg.pos_z = tmp 
        
        ### V1 25年赛题坐标 
        # msg.pos_x = -msg.pos_x * 0.975 + 0.25
        # msg.pos_y = -msg.pos_y * 0.963 + 0.25
        # msg.pos_z = msg.pos_z * 1.0

        ### V2 T265朝上
        msg.pos_x = -msg.pos_x 
        msg.pos_y = -msg.pos_y

        # ### V3  T265朝前朝右侧
        # tmp = msg.pos_x
        # msg.pos_x  = -msg.pos_y
        # msg.pos_y = tmp

        ### debug 
        # self.get_logger().info(
        #    f'position confidence:[{msg.confidence:1d}],'
        #    f'pos(x:{msg.pos_x:+10.6f}m, y:{msg.pos_y:+10.6f}m), '
        #    f'h[{msg.pos_z:+10.6f}m]')
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = T265("t265")
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    
if __name__ == '__main__':
    main()


def t265_data_to_uart_std(send_header, data):
    send_content = send_header + " "
    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_x)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_y)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_z)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5 + " "
    return send_content

def extract_digits(num):
    sign = "2B"
    if num < 0:
        sign = "2D"
    single_dig = int(abs(num) % 10) + 30
    reverse_single_dig = int(abs(num) * 10 % 10) + 30
    reverse_ten_dig = int(abs(num) * 100 % 10) + 30
    reverse_hundred_dig = int(abs(num) * 1000 % 10) + 30
    reverse_thousand_dig = int(abs(num) * 10000 % 10) + 30
    return sign + " ", str(single_dig)+ " ", str(reverse_single_dig)+ " ", str(reverse_ten_dig)+ " ", str(reverse_hundred_dig)+ " ", str(reverse_thousand_dig) + " "


# ros2 回调组说明
# https://blog.csdn.net/m0_73800387/article/details/138439256
# ros2 回调组示例
# https://github.com/liugehaizaixue/python_ros2_note/blob/main/ros2_sub_MutuallyExclusiveCallbackGroup.py
