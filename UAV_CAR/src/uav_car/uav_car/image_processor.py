import rclpy
from rclpy.node import Node
import cv2
import math
import time
from periphery import PWM
from periphery import GPIO # 
from cv_bridge import CvBridge
import numpy as np
from uav_car_unit.yolo_detector import YOLODetector
from uav_car_unit.udp_stream import UDPImageSender
from uav_car_interfaces.msg import ImageProcessorData, T265Data
from sensor_msgs.msg import Image
from std_msgs.msg import String,Float64MultiArray
import uav_car_unit.utils_ros as ur

class ImageProcessor(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.sub_camera = self.create_subscription(
            Image, "camera_data_topic", self.sub_camera_callback,5
        )       
        self.sub_t265_loc = self.create_subscription(
            T265Data, "t265_data_topic", self.sub_t265_loc_callback,5
        )
        self.sub_find_last_point = self.create_subscription(
            Float64MultiArray, "fly_path_turns_last_topic", self.sub_find_last_point_callback,5
        )
        self.pub_image_processor = self.create_publisher(ImageProcessorData, "image_processor_topic", 10)
        # self.pub_follow = self.create_publisher(FollowData, "follow_topic", 10)
        # self.pub_animal_picture = self.create_publisher(AnimalPictureData, "animal_picture_topic", 10)
        self.sub_udp_info = self.create_subscription(
            String, "udp_info_topic", self.udp_info_callback, 10)
        self.udp_sender = None
        self.init_yolo_detector()
        self.cv_bridge = CvBridge()
        self.captured_raw_image = None
        self.captured_image = None
        
        self.find_state = True
        self.find_last_point = None
        self.find_position = None
        # 当前位置
        self.cx = 0.0
        self.cy = 0.0 
        self.cz = 0.0
        
        # pixel 偏移
        self.px_offset = 0.0
        self.py_offset = 0.0
        
        # 偏移位置
        self.dx = 0.0
        self.dy = 0.0

        # 上舵机 0-90右 90-180左
        self.pwm_servo_chip_up = 1
        self.pwm_servo_channel_up = 0
        self.pwm_servo_frequency_up = 50
        self.pwm_servo_init_up = 89
        
        # 下舵机 0-90下 90-180上
        self.pwm_servo_chip_down = 3    
        self.pwm_servo_channel_down = 0
        self.pwm_servo_frequency_down = 50
        self.pwm_servo_init_down = 92

        # 停止坐标及标志位
        self.stop_x = 0.0
        self.stop_y = 0.0
        self.stop_flag = False
        self.find_same_flag = False
        self.now_position = None
        
        self.laser_count = 0
        self.time_out_count = 0
        
        self.animals = []
        self.animals_laser = []
        self.animals_len = 0
        self.count_for_len = 0
        self.state_laser = 0
        self.state_laser_animals = 0
        

        self.angles_index = 0
        self.servo_angles = [
            [[66, 55], [66, 58], [66, 61], [66, 64], [67, 66], [67, 69], [67, 72], [67, 75], [67, 78], [67, 81], [67, 84], [67, 87], [68, 89], [68, 92], [68, 95], [68, 98]] ,
			[[69, 55], [69, 58], [69, 61], [69, 64], [69, 66], [69, 69], [70, 72], [70, 75], [70, 78], [70, 81], [70, 84], [70, 86], [70, 89], [70, 92], [71, 95], [71, 98]] ,
			[[72, 55], [72, 58], [72, 61], [72, 64], [72, 66], [72, 69], [72, 72], [72, 75], [73, 78], [73, 81], [73, 83], [73, 86], [73, 89], [73, 92], [73, 95], [73, 98]] ,
			[[74, 55], [75, 58], [75, 61], [75, 64], [75, 66], [75, 69], [75, 72], [75, 75], [75, 78], [75, 81], [75, 83], [76, 86], [76, 89], [76, 92], [76, 95], [76, 98]] ,
			[[77, 55], [77, 58], [77, 61], [77, 63], [78, 66], [78, 69], [78, 72], [78, 75], [78, 78], [78, 80], [78, 83], [78, 86], [78, 89], [78, 92], [79, 95], [79, 97]] ,
			[[80, 55], [80, 58], [80, 61], [80, 63], [80, 66], [80, 69], [81, 72], [81, 75], [81, 78], [81, 80], [81, 83], [81, 86], [81, 89], [81, 92], [81, 95], [81, 97]] ,
			[[83, 55], [83, 58], [83, 61], [83, 63], [83, 66], [83, 69], [83, 72], [83, 75], [83, 78], [84, 80], [84, 83], [84, 86], [84, 89], [84, 92], [84, 94], [84, 97]] ,
			[[86, 55], [86, 58], [86, 61], [86, 63], [86, 66], [86, 69], [86, 72], [86, 75], [86, 77], [86, 80], [86, 83], [86, 86], [86, 89], [87, 91], [87, 94], [87, 97]] ,
			[[88, 55], [88, 58], [89, 61], [89, 63], [89, 66], [89, 69], [89, 72], [89, 75], [89, 77], [89, 80], [89, 83], [89, 86], [89, 89], [89, 91], [89, 94], [89, 97]] ,
			[[91, 55], [91, 58], [91, 61], [91, 63], [91, 66], [91, 69], [92, 72], [92, 75], [92, 77], [92, 80], [92, 83], [92, 86], [92, 88], [92, 91], [92, 94], [92, 97]] ,
			[[94, 55], [94, 58], [94, 61], [94, 63], [94, 66], [94, 69], [94, 72], [94, 74], [94, 77], [94, 80], [94, 83], [94, 86], [95, 88], [95, 91], [95, 94], [95, 97]] ,
			[[97, 55], [97, 58], [97, 61], [97, 63], [97, 66], [97, 69], [97, 72], [97, 74], [97, 77], [97, 80], [97, 83], [97, 85], [97, 88], [97, 91], [97, 94], [97, 97]] ,
			[[100, 55], [100, 58], [100, 61], [100, 63], [100, 66], [100, 69], [100, 72], [100, 74], [100, 77], [100, 80], [100, 83], [100, 85], [100, 88], [100, 91], [100, 94], [100, 96]] ,   
			[[102, 55], [102, 58], [102, 61], [102, 63], [102, 66], [102, 69], [103, 72], [103, 74], [103, 77], [103, 80], [103, 83], [103, 85], [103, 88], [103, 91], [103, 94], [103, 96]] ,   
			[[105, 55], [105, 58], [105, 60], [105, 63], [105, 66], [105, 69], [105, 71], [105, 74], [105, 77], [105, 80], [105, 82], [105, 85], [105, 88], [105, 91], [105, 93], [105, 96]] ,   
			[[108, 55], [108, 58], [108, 60], [108, 63], [108, 66], [108, 69], [108, 71], [108, 74], [108, 77], [108, 80], [108, 82], [108, 85], [108, 88], [108, 91], [108, 93], [108, 96]] ,
        ]
        
        # 激光笔红
        self.laser1_gpio_id = 35
        self.laser1_gpio_mode = "out"
        # 激光笔绿
        self.laser2_gpio_id = 54
        self.laser2_gpio_mode = "out"
        
        # 初始化激光笔
        self.laser1_gpio = GPIO(self.laser1_gpio_id, self.laser1_gpio_mode)
        self.laser1_gpio.write(True)  # 激光笔红灯开启
        
        self.laser2_gpio = GPIO(self.laser2_gpio_id, self.laser2_gpio_mode)
        self.laser2_gpio.write(False) 

        ur.uart_sender4_init(self)
        
        # 初始化舵机
        self.pwm_servo_up = PWM(self.pwm_servo_chip_up, self.pwm_servo_channel_up)
        self.pwm_servo_up.frequency = self.pwm_servo_frequency_up
        self.pwm_servo_up.duty_cycle = self.angle_to_pwm(self.pwm_servo_init_up)
        self.pwm_servo_up.enable()
        
        self.pwm_servo_down = PWM(self.pwm_servo_chip_down, self.pwm_servo_channel_down)
        self.pwm_servo_down.frequency = self.pwm_servo_frequency_down
        self.pwm_servo_down.duty_cycle = self.angle_to_pwm(self.pwm_servo_init_down)
        self.pwm_servo_down.enable()



    """初始化YOLO检测器"""
    def init_yolo_detector(self):
        try:
            # YOLO模型路径 - 请根据实际路径修改
            from pathlib import Path
            script_dir = Path(__file__).parent.resolve()
            model_path = str(script_dir / "animaldetectv5_noquant.rknn")
            # model_path = "/home/orangepi/Desktop/UAV_CAR__Test/UAV_CAR/src/uav_car/uav_car/animaldetectv5_noquant.rknn"
            # 动物类别
            animal_classes = ["monkey", "tiger", "elephant", "wolf", "peacock"]
            # 创建YOLO检测器
            self.yolo_detector = YOLODetector(
                model_path=model_path,
                classes=animal_classes,
                obj_thresh=0.63,
                nms_thresh=0.45,
                img_size=(480, 480)
            )
            self.get_logger().info("YOLO检测器初始化成功")
            self.get_logger().info(f"模型路径: {model_path}")
            self.get_logger().info(f"支持的动物类别: {animal_classes}")
        except Exception as e:
            self.get_logger().error(f"YOLO检测器初始化失败: {str(e)}")
            self.yolo_detector = None
            
    def sub_camera_callback(self, data):
#        self.get_logger().info('Captured an image')
        self.captured_raw_image = data

        tmp_image = self.captured_raw_image
        if tmp_image is not None:
            self.captured_image = self.cv_bridge.imgmsg_to_cv2(tmp_image, 'bgr8')
            draw_image = self.captured_image.copy()
            if self.find_state:
                self.animals, draw_image = self.animals_recognition(self.captured_image)

                if self.animals_len > 0:
                    self.count_for_len += 1
                self.animals_len = len(self.animals)
                
                if self.animals and self.find_same_flag is False and self.cz > 0.8:
                    for animal in self.animals:
                        
                        self.dx, self.dy = self.calculate_distance(animal[0], animal[1])
                        self.get_logger().info(f"Animal offset: dx={self.dx}, dy={self.dy}, type={animal[2]}")
                        animal_position = self.get_grid_position(self.dx + self.cx, self.dy + self.cy)
                        self.get_logger().info(f"Animal position in grid: {animal_position}")
                        self.now_position = self.get_grid_position(self.cx, self.cy)
                        self.get_logger().info(f"Current position in grid: {self.now_position}")                   
                        if animal_position is not None and self.now_position is not None:
                            if animal_position[0] == self.now_position[0] and animal_position[1] == self.now_position[1]:
                                self.stop_flag = True
                                self.find_same_flag = True
                                self.stop_x, self.stop_y = self.get_grid_center(self.now_position[0], self.now_position[1])
                                msg = T265Data()
                                msg.pos_x = self.stop_x
                                msg.pos_y = self.stop_y
                                msg.pos_z = 1.0
                                self.send_location(msg)
                                self.get_logger().info(f"Animal found at position: {animal_position}, stopping at ({self.stop_x}, {self.stop_y})")
                                break
            if self.udp_sender is not None:
                success = self.udp_sender.send_frame(draw_image, stream_id=1)
                if not success:
                    self.get_logger().warning("UDP frame send failed (queue full)")         
                    
    def animals_recognition(self, image):
        # 动物列表
        animals = []
        draw_image = image.copy()
        if self.yolo_detector is not None:
            # 使用YOLO检测器进行动物识别
            detection , draw_image= self.yolo_detector.detect_and_draw(image,draw_conf=True)
            boxes = detection["boxes"]
            class_names = detection["class_names"]
            for i,(box, class_name) in enumerate(zip(boxes, class_names)):
                x1, y1, x2, y2 = [int(coord) for coord in box]
                off_x = 240 - int((y1 + y2) / 2)
                off_y = 240 - int((x1 + x2) / 2)
                if class_name == "monkey":
                    animal_type = 0
                elif class_name == "tiger":
                    animal_type = 1
                elif class_name == "elephant":
                    animal_type = 2
                elif class_name == "wolf":
                    animal_type = 3
                elif class_name == "peacock":
                    animal_type = 4
                if abs(off_x) < 171 and abs(off_y) < 171:
                    animals.append((off_x, off_y, animal_type)) 
                self.get_logger().info(f"Detected animal {i+1}: {class_name} at ({off_x}, {off_y})")
#            self.get_logger().info(f"Detected {len(animals)} animals")
            
        return animals, draw_image

    def udp_info_callback(self, msg):
        try:
            # 若msg.data为"open:"+"ip"，则打开UDP发送
            if msg.data.startswith("open:"):
                # 先关闭已存在的UDP发送器
                if self.udp_sender is not None:
                    self.udp_sender.stop()
                    self.udp_sender = None
                
                # 解析IP地址
                ip = msg.data.split(":")[1]
                
                # 创建新的UDP发送器
                self.udp_sender = UDPImageSender(
                    target_ip=ip, 
                    target_port=5000,  # 使用你原来的端口
                    jpeg_quality=80,   # 可调整质量
                    max_packet_size=1400
                )
                self.get_logger().info(f"UDP sender opened to {ip}:5000")
                
            elif msg.data == "close":
                if self.udp_sender is not None:
                    self.udp_sender.stop()
                    self.udp_sender = None
                    self.get_logger().info("UDP sender closed")
            else:
                self.get_logger().warning(f"Unknown UDP command: {msg.data}")
                
        except Exception as e:
            self.get_logger().error(f"UDP info callback error: {e}")
            if self.udp_sender is not None:
                self.udp_sender.stop()
                self.udp_sender = None
                
    def sub_find_last_point_callback(self, data):
        self.find_last_point = data.data
        # 除以一百统一单位
        self.find_last_point = [point / 100.0 for point in self.find_last_point]
        self.get_logger().info(f"Received find last point: {self.find_last_point}")

        
    def sub_t265_loc_callback(self, data):
        self.cx = data.pos_x
        self.cy = data.pos_y
        self.cz = data.pos_z
        
        if self.find_last_point is not None:
            if abs(self.cx - self.find_last_point[0]) < 0.05 and abs(self.cy - self.find_last_point[1]) < 0.05:
                self.find_position = self.get_grid_position(self.cx, self.cy)
            if self.find_position is not None:
                if self.get_grid_position(self.cx, self.cy) != self.find_position:
                    self.find_state = False
                    self.get_logger().info(f"开始返航")
                    
        if self.stop_flag:
            if self.state_laser == 0 and abs(self.cx - self.stop_x) < 0.05 and abs(self.cy - self.stop_y) < 0.05:
                self.get_logger().info(f"到达停止位置: ({self.stop_x}, {self.stop_y})")
                self.count_for_len = 0
                self.state_laser = 1
            if self.state_laser == 1:
                # self.animals_len(动物的数量)大于零，选取大于零的五次结果中动物总数最多的一个
                if self.animals_len > 0:
                    if len(self.animals_laser) == 0:
                        self.animals_laser = self.animals.copy()
                    else:
                        if len(self.animals) > len(self.animals_laser):
                            self.animals_laser = self.animals.copy()
                else:
                    self.time_out_count += 1
                    if self.time_out_count > 100:
                        self.get_logger().info("No animals detected, resetting state")
                        self.state_laser = 0
                        self.laser2_gpio.write(False)
                        msg = "99 "
                        ur.uart_sender4_send(self, msg)
                        self.time_out_count = 0
                        
                if  self.count_for_len >= 5:
                    #确认动物存在
                    self.state_laser = 2
                    self.count_for_len = 0
                    self.laser2_gpio.write(True)
                    # animal_picture = AnimalPictureData()
                    # animal_picture.fx = 8 - self.now_position[1]
                    # animal_picture.fy = self.now_position[0]
                    # animal_picture.animalpicture = self.captured_raw_image
                    # self.pub_animal_picture.publish(animal_picture)
                
                    
            if self.state_laser == 2:
                if self.state_laser_animals <= len(self.animals_laser) - 1:
                    
                    angles = self.get_servo_target_angles(self.animals_laser[self.state_laser_animals][0], self.animals_laser[self.state_laser_animals][1])
                    if self.angles_index < len(angles):
                        down_angle, up_angle = angles[self.angles_index]
                        self.pwm_servo_down.duty_cycle = self.angle_to_pwm(down_angle)
                        self.pwm_servo_up.duty_cycle = self.angle_to_pwm(up_angle)
                        if self.laser_count > 4:
                            self.angles_index += 1
                            self.laser_count = 0        
                        else:
                            self.laser_count += 1
                        self.get_logger().info(f" servSettingo angles: down={down_angle}, up={up_angle}, index={self.angles_index}")        
                    else:
                        animal_msg = ImageProcessorData()
                        animal_msg.res_x = 8 - self.now_position[0]
                        animal_msg.res_y = self.now_position[1]
                        animal_msg.res_flag = self.animals_laser[self.state_laser_animals][2]  # 假设第三个元素是动物类型
                        self.pub_image_processor.publish(animal_msg)
                        self.get_logger().info(f"Processed animal {self.state_laser_animals + 1}/{len(self.animals_laser)}: {self.animals_laser[self.state_laser_animals][2]}")
                        self.angles_index = 0
                        self.state_laser_animals += 1
                else:
                    # 重置状态
                    self.state_laser_animals = 0
                    self.state_laser = 0
                    self.time_out_count = 0
                    self.animals_laser = []
                    self.laser2_gpio.write(False)
                    msg = "99 "
                    ur.uart_sender4_send(self, msg)
                    self.stop_flag = False
                    self.get_logger().info("Animals laser processing completed, reset state")
                        
        if self.get_grid_position(self.cx, self.cy) != self.now_position:
            self.find_same_flag = False
            #self.get_logger().info(f"Current position changed: {self.now_position} -> ({self.cx}, {self.cy})")


    def calculate_distance(self, offset_x, offset_y):

        w_offset_x = offset_x *  0.7 / 480
        w_offset_y = offset_y *  0.7 / 480
        return w_offset_x, w_offset_y
    
    def angle_to_pwm(self, angle):
        # 将角度转换为占空比
        duty_cycle = (angle/180.0) * 0.1 + 0.875 
        return duty_cycle
    
        
    def get_servo_target_angles(self, offset_x, offset_y):
        grid_size = 30
        rows = 16  # 16x16网格
        cols = 16
        
        # 计算目标舵机的行列索引（中心偏移8列/行）
        row_index = 8 - int(offset_x // grid_size) 
        col_index = 8 - int(offset_y // grid_size)

        # 存储结果：目标角度 + 周围有效角度
        angles_list = []

        # 检查目标位置是否在有效范围内
        if 0 <= row_index < rows and 0 <= col_index < cols:
            # 获取目标舵机角度
            target_down, target_up = self.servo_angles[row_index][col_index]
            angles_list.append((target_down, target_up))

            # 定义周围8个方向的偏移量（上下左右）
            surrounding_offsets = [
                (-1, 0),  # 上一行
                (0, -1), (0, 1),  # 当前行左右
                (1, 0),   # 下一行
                (-1, -1), (-1, 1),  # 上斜左，上斜右
                (1, -1), (1, 1),   # 下斜左，下斜右
                (2, 0), (-2, 0),  # 上下两行
                (0, 2), (0, -2)   # 左右两列
            ]

            # 遍历周围位置，筛选有效角度
            for dr, dc in surrounding_offsets:
                adj_row = row_index + dr  # 相邻行索引
                adj_col = col_index + dc  # 相邻列索引
               
                if 0 <= adj_row < rows and 0 <= adj_col < cols:
                    adj_down, adj_up = self.servo_angles[adj_row][adj_col]
                    angles_list.append((adj_down, adj_up))
        else:
            # 目标位置无效时，仅返回默认角度
            angles_list.append((92, 89))

        return angles_list

    def get_grid_position(self, x, y):
        grid_size = 0.5  # 方格大小
        rows = 9  # 总行数
        cols = 7  # 总列数
        
        # 计算在哪一行和哪一列
        col_index = int(x // grid_size)
        row_index = int(y // grid_size)
        
        # 检查是否在有效范围内
        if 0 <= row_index < rows and 0 <= col_index < cols:
            return (row_index, col_index)
        else:
            return None
        
    def get_grid_center(self, row_index, col_index):
        """
        获取指定方格的中心坐标
        
        参数:
            row_index: 行索引 (0-6)
            col_index: 列索引 (0-8)
            
        返回:
            tuple: (center_x, center_y) 方格中心坐标
        """
        grid_size = 0.5
        center_x = col_index * grid_size + grid_size / 2
        center_y = row_index * grid_size + grid_size / 2
        return center_x, center_y
            
    def send_location(self, data):
        # config header to send to shufly!
        # data.header = "99"
        # self.pub_t265_uart_send_pos.publish(data)
        msg = ur.t265_data_to_uart_std(self, "98", "35", data)
        ur.uart_sender4_send(self, msg)
        self.get_logger().warning(f'send location data success')
        
def main(args=None):
    rclpy.init(args=args)
    imageProcessor = ImageProcessor("image_processor")
    rclpy.spin(imageProcessor)
    imageProcessor.destroy_node()
    rclpy.shutdown()
    cv2.destroy_all_windows()

if __name__ == '__main__':
    main()
