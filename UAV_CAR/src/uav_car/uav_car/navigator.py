import math,time
import uav_car_unit.utils_ros as ur

import rclpy                                     
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String

# 自定义消息和服务的数据类型
from uav_car_interfaces.msg import T265Data
from uav_car_interfaces.srv import ControlService

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup# 读串口操作，禁用ReentrantCallbackGroup，避免同一回调创建多个串口实例，导致崩溃

class Navigator(Node):
    def __init__(self, name):
        super().__init__(name) 
        self.topic_stack = 10

        self.hand_active = False # 手动控制模式标志！
        
        self.get_logger().warning('navigator initialize') 
        self.pos_callback_group = MutuallyExclusiveCallbackGroup()  
        
        # t265 open service client
        self.client_t265_open = self.create_client(Empty, "t265_open_service")
        
        # uart topic
        # self.mcu_callback_group = MutuallyExclusiveCallbackGroup()  
        # self.topic_uart4_sub = self.create_subscription(String, 'uart_reader4_data_topic', self.mcu_callback, callback_group=self.mcu_callback_group)
        # def mcu_callback(self):
        
        self.topic_uart4_pub = self.create_publisher(String, 'uart_sender4_data_topic', self.topic_stack)
        self.topic_uart3_pub = self.create_publisher(String, 'uart_sender3_data_topic', self.topic_stack)

        # t265  topic: 获取postion
        self.pos = T265Data() # 存储最新的t265数据，供其他函数使用
        
        self.topic_t265_sub = self.create_subscription(T265Data, 't265_data_topic', self.sync_position_data, self.topic_stack,callback_group=self.pos_callback_group) # 订阅t265数据，回调函数处理数据
        

        # 自定义服务，接收来自外部的起飞降落指令
        self.server_command = self.create_service(ControlService, "command_service", self.commander_callback) 

        time.sleep(3)  # 延迟 3 秒
        self.t265_open()
        self.get_logger().info(f'navigetor节点启动完成')

    def t265_open(self):
        while not self.client_t265_open.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('正在等待t265_open服务的server启动')
        self.get_logger().info('t265_open服务的server启动成功,已发送启动请求')
        self.t265OpenRequest = Empty.Request()
        self.t265service_future = self.client_t265_open.call_async(self.t265OpenRequest) # 异步方式发送服务请求

    def sync_position_data(self, msg: T265Data):
        # 处理接收到的t265数据，发布到uart3主题
        self.pos = msg  # 更新最新的t265数据到成员变量
        if abs(self.pos.pos_z ) > 2.1:
            msg_land_cmd = String()
            msg_land_cmd.data = 'S' # 0x53 Land
            self.topic_uart4_pub.publish(msg_land_cmd) 
            self.get_logger().warning(f"飞机高度失效！！！")
 
    def commander_callback(self, request, response):  # 接收地面站指令！
        # linux-uart3接mcu uart7 实时的t265数据  
        # linux-uart4接mcu uart2 不定时的消息指令

        # 前往定点 G
        # 起飞    R
        # 降落    S
        # 旋转朝向 T
        msg = String()
        
        if request.req.startswith('H') and len(request.req) == 19:
            # 手动控制：H开头，长度19
            modified_msg = 'G' + request.req[1:]  # 替换首字母为 G
            msg.data = modified_msg
            self.topic_uart4_pub.publish(msg)
            # 激活手动优先级：屏蔽后续 G 消息
            self.hand_active = True
            self.get_logger().warning(f"进入手动控制模式，屏蔽编程位置控制")
            
        elif request.req == 'takeoff':
        # linux uart3 - MCU uart7 t265实时数据  t265就绪起飞
        # linux uart4 - MCU uart2 其他指令：降落  飞向定点 
            msg.data = 'R'
            self.topic_uart4_pub.publish(msg) # 0x52  
        elif request.req == 'land': # 降落指令 land
            msg.data = 'S' # 0x53
            self.topic_uart4_pub.publish(msg)   
        elif request.req == 'normalLand': # 降落指令 land
            msg.data = 'U' # 0x55
            self.topic_uart4_pub.publish(msg)   
        elif request.req.startswith('G') and len(request.req) == 19 and self.hand_active == False :  # 首字母是 'G' 且总长度为 19, 并且无手动干预时，编程控制飞往目标点
            msg.data = request.req
            self.topic_uart4_pub.publish(msg)
        else:
            msg.data = request.req # 如果是其他的杂乱消息，不理会，但是打印出来
            self.get_logger().warning(f"navigator杂乱消息: {request.req}  ")
        
        self.get_logger().warning(f"飞行站已调用：{request.req} service called")
        response.echo = f"飞行站已调用：{request.req}服务" # 打印一下服务调用
        return response
                
    

def main(args=None):
    rclpy.init(args=args)
    node = Navigator("navigator")
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
                                
