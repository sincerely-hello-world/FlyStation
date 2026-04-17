from uav_car_interfaces.msg import T265Data
from uav_car_interfaces.msg import ImageProcessorData
import rclpy

import time
import os

from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


def block_for_response(self):
    while rclpy.ok():
        rclpy.spin_once(self)
        if self.future.done():
            try:
                response = self.future.result()
                self.get_logger().warning(f'get response success')
            except Exception as e:
                self.get_logger().error(
                    f'get response fail')
                self.get_logger().error(str(e))
                block_for_error(self)
            return response
        
def block_for_time_no_spin(self, last_time):
    self.get_logger().warning(f"start block for {last_time}s")
    dur = 0.0
    start = time.time()
    while dur < last_time:
        now = time.time()
        dur = now - start
    self.get_logger().warning(f"end block for {last_time}s")

def block_for_time(self, last_time):
    self.get_logger().warning(f"start block for {last_time}s")
    dur = 0.0
    start = time.time()
    while dur < last_time:
        rclpy.spin_once(self)
        now = time.time()
        dur = now - start
    self.get_logger().warning(f"end block for {last_time}s")


def block_for_error(self):
    self.get_logger().error("Due to exception,node will be blocked permanently!!!")
    rclpy.spin(self)


def block_for_file(self, path):
    while rclpy.ok():
        if os.path.exists(path):
            self.get_logger().warning(f'get file success')
            break
        self.get_logger().info(f'in while for waiting file')


# t265
def t265_init(self):
    self.client_t265_open = self.create_client(Empty, "t265_open_service")
    


def t265_open(self):
    while not self.client_t265_open.wait_for_service(timeout_sec=3.0):
        self.get_logger().warning('正在等待t265_open服务的server启动')
    self.get_logger().info('t265_open服务的server启动成功')
    self.t265OpenRequest = Empty.Request()
    self.t265service_future = self.client_t265_open.call_async(self.t265OpenRequest)  # 异步方式发送服务请求

    
 

# t265-uart
def t265_uart_init(self):
    self.client_t265_uart_open = self.create_client(Empty, "t265_uart_open_service")
    while not self.client_t265_uart_open.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待t265_uart_open服务的server启动')
    self.t265UartOpenRequest = Empty.Request()


def t265_uart_open(self):
    self.future = self.client_t265_uart_open.call_async(self.t265UartOpenRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending start t265_uart request success')
    block_for_response(self)
    self.get_logger().warning(f'start t265_uart success')


# camera
def camera_init(self):
    self.client_camera_open = self.create_client(Empty, "camera_open_service")
    while not self.client_camera_open.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待camera_open服务的server启动')
    self.cameraOpenRequest = Empty.Request()

    self.client_camera_close = self.create_client(Empty, "camera_close_service")
    while not self.client_camera_close.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待camera_close服务的server启动')
    self.cameraCloseRequest = Empty.Request()


def camera_open(self):
    self.future = self.client_camera_open.call_async(self.cameraOpenRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending start camera request success')
    block_for_response(self)
    self.get_logger().warning(f'start camera success')


def camera_close(self):
    self.future = self.client_camera_close.call_async(self.cameraCloseRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending close camera request success')
    block_for_response(self)
    self.get_logger().warning(f'close camera success')


# camera2
def camera2_init(self):
    self.client_camera2_open = self.create_client(Empty, "camera2_open_service")
    while not self.client_camera2_open.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待camera2_open服务的server启动')
    self.camera2OpenRequest = Empty.Request()

    self.client_camera2_close = self.create_client(Empty, "camera2_close_service")
    while not self.client_camera2_close.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待camera2_close服务的server启动')
    self.camera2CloseRequest = Empty.Request()


def camera2_open(self):
    self.future = self.client_camera2_open.call_async(self.camera2OpenRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending start camera2 request success')
    block_for_response(self)
    self.get_logger().warning(f'start camera2 success')


def camera2_close(self):
    self.future = self.client_camera2_close.call_async(self.camera2CloseRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending close camera2 request success')
    block_for_response(self)
    self.get_logger().warning(f'close camera2 success')


# laser
def laser_init(self):
    self.client_laser_open = self.create_client(LaserOpenService, "laser_open_service")
    while not self.client_laser_open.wait_for_service(timeout_sec=1.0):
        self.get_logger().warning('正在等待laser_open服务的server启动')
    self.laserOpenServiceRequest = LaserOpenService.Request()


def laser_open(self, mode, count):
    self.laserOpenServiceRequest.mode = mode
    self.laserOpenServiceRequest.count = count
    self.future = self.client_laser_open.call_async(self.laserOpenServiceRequest)  # 异步方式发送服务请求
    self.get_logger().warning(f'sending laser open request success')
    block_for_response(self)
    self.get_logger().warning(f'laser run success')

    
# using topic
# uart_sender -pub
def uart_sender0_init(self, topic_stack=10):
    self.pub_uart_sender0 = self.create_publisher(String, 'uart_sender0_data_topic', topic_stack)


def uart_sender0_send(self, data):
    msg = String()
    msg.data = data
    self.pub_uart_sender0.publish(msg)
    self.get_logger().info(f'uart0 send success')   
    
def uart_sender1_init(self, topic_stack=10):
    self.pub_uart_sender1 = self.create_publisher(String, 'uart_sender1_data_topic', topic_stack)


def uart_sender1_send(self, data):
    msg = String()
    msg.data = data
    self.pub_uart_sender1.publish(msg)
    self.get_logger().info(f'uart1 send success')


def uart_sender3_init(self, topic_stack=10):
    self.pub_uart_sender3 = self.create_publisher(String, 'uart_sender3_data_topic', topic_stack)


def uart_sender3_send(self, data):
    msg = String()
    msg.data = data
    self.pub_uart_sender3.publish(msg)
    self.get_logger().info(f'uart3 send {msg} success')

def uart_sender3_send_slient(self, data):
    msg = String()
    msg.data = data
    self.pub_uart_sender3.publish(msg)
#    self.get_logger().info(f'uart3 send {msg} success')



def uart_sender4_init(self, topic_stack=10):
    self.pub_uart_sender4 = self.create_publisher(String, 'uart_sender4_data_topic', topic_stack)


def uart_sender4_send(self, data):
    msg = String()
    msg.data = data
    self.pub_uart_sender4.publish(msg)
    self.get_logger().info(f'uart4 send {msg} success')

# ridar_receiver -sub 
def poles_receiver_init(self, topic_stack=10):
    self.sub_poles_date = self.create_subscription(Float32MultiArray, "poles_array", self.poles_callback,topic_stack)

# t265_receiver -sub mention: there is no class to present for t265_receiver,just to receive t265 data -sub
def t265_receiver_init(self, topic_stack=5):
    self.client_t265_data = self.create_subscription(T265Data, "t265_data_topic", self.t265_receiver_callback,
                                                     topic_stack)
def uart_receiver0_init(self, topic_stack=10):
    self.sub_uart_receiver0 = self.create_subscription(String, "uart_receiver0_data_topic",
                                                       self.uart_receiver1_callback, topic_stack)
# uart_receiver -sub
def uart_receiver1_init(self, topic_stack=10):
    self.sub_uart_receiver1 = self.create_subscription(String, "uart_receiver1_data_topic",
                                                       self.uart_receiver1_callback, topic_stack)


def uart_receiver3_init(self, topic_stack=10):
    self.sub_uart_receiver3 = self.create_subscription(String, "uart_receiver3_data_topic",
                                                       self.uart_receiver3_callback, topic_stack)


def uart_receiver4_init(self, topic_stack=10):
    self.sub_uart_receiver4 = self.create_subscription(String, "uart_receiver4_data_topic",
                                                       self.uart_receiver4_callback, topic_stack)

# image_processor -sub
def image_processor_init(self, topic_stack=5):
    self.sub_image_processor = self.create_subscription(ImageProcessorData, "image_processor_topic", 
                                                        self.image_processor_callback, topic_stack)

def remove_file(self, path):
    if os.path.exists(path):
        os.remove(path)
        self.get_logger().warning(f'remove file:{path}')


def remove_folder(self, path):
    if os.path.exists(path):
        if os.path.isfile(path) or os.path.islink(path):
            os.remove(path)
            self.get_logger().warning(f"remove file:{path}")
        else:
            for filename in os.listdir(path):
                remove_folder(self, os.path.join(path, filename))
            os.rmdir(path)
            self.get_logger().warning(f"remove dir:{path}")


def count_files_in_folder(folder_path):
    count = 0
    for _, _, files in os.walk(folder_path):
        count += len(files)
        return count


def count_folders(folder_path):
    count = 0
    for item in os.listdir(folder_path):
        if os.path.isdir(os.path.join(folder_path, item)):
            count += 1
    return count


def oak_data_to_uart_std(self, send_header, send_eof, data, status):
    send_content = send_header + " "
    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_x)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_y)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_z)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5 + status + " " + send_eof + " "
    return send_content

# uart policy
def t265_data_to_uart_std(self, send_header, send_eof, data):
    send_content = send_header + " "
    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_x)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_y)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5

    xbs,xb1,xb2,xb3,xb4,xb5 = extract_digits(data.pos_z)
    send_content = send_content + xbs + xb1 + xb2 + xb3 + xb4 + xb5 + send_eof + " "
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

 
