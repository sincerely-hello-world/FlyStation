import math,time
# import uav_car_unit.utils_ros as ur

import rclpy                                     
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String,Bool

# 自定义消息和服务的数据类型
from uav_car_interfaces.msg import T265Data
from uav_car_interfaces.srv import ControlService

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup# 读串口操作，禁用ReentrantCallbackGroup，避免同一回调创建多个串口实例，导致崩溃

from uav_car.my_format import TGformat

class Navigator(Node):
    def __init__(self, name):
        super().__init__(name) 
        self.get_logger().warning('navigator节点初始化中....') 

        self.is_flying = False #  飞机目前状态
        self.heart_active = False # 地面站心跳是否存活？
        self.heightMax = 2.1 # 米
        self.hand_active = False # 手动控制模式标志！
        
        
        self.pos_callback_group = MutuallyExclusiveCallbackGroup()  
 
        # uart topic
        # self.mcu_callback_group = MutuallyExclusiveCallbackGroup()  
        # self.topic_uart4_sub = self.create_subscription(String, 'uart_reader4_data_topic', self.mcu_callback, callback_group=self.mcu_callback_group)
        # def mcu_callback(self):
        
        self.topic_uart4_pub = self.create_publisher(String, 'uart_sender4_data_topic', 10)
        self.topic_uart3_pub = self.create_publisher(String, 'uart_sender3_data_topic', 10)

        # t265  topic: 获取postion
        self.pos = T265Data() # 存储最新的t265数据，供其他函数使用
        self.topic_t265_sub = self.create_subscription(T265Data, 't265_data_topic', self.sync_position_data, 10,callback_group=self.pos_callback_group) # 订阅t265数据，回调函数处理数据

        
        # t265 open service client, 让T265节点发布飞机位置
        self.client_t265_open = self.create_client(Empty, "t265_open_service")
        # 自定义服务，接收来自外部的起飞降落指令
        self.server_command = self.create_service(ControlService, "command_service", self.commander_callback) 

        # 心跳服务订阅：避免地面站失联
        self.ground_heart_beat_sub = self.create_subscription(Bool, '/FlyStation/heartbeat/status', self.heart_beat_callback,10)

        time.sleep(3)  # 延迟 3 秒
        self.t265_open()
        self.get_logger().warning(f'navigetor节点启动完成')

    def heart_beat_callback(self,msg):
        heartbeat_ok = msg.data
        self.heart_active = heartbeat_ok

        if self.is_flying: # 飞机起飞了
            if not heartbeat_ok:
                self.get_logger().error('【安全警报】飞机在空中，但地面站心跳断开！执行紧急降落！')
                self.topic_uart4_pub.publish(String(data='S'))   # 0x53 发布降落指令！ 紧急降落！ 地面站失去联系了！
            else:# 心跳正常，且在空中，可以持续打印或保持静默
                self.get_logger().info('地面站心跳正常', throttle_duration_sec=2.0)
                pass
        else: # 飞机还在地面上
            if not heartbeat_ok:
                self.get_logger().warn('地面站心跳未连接，此时禁止起飞。', throttle_duration_sec=0.5)
                self.heart_active = False
            else:# 心跳正常，可以持续打印或保持静默
                self.get_logger().info('地面站心跳正常', throttle_duration_sec=2.0)
                pass


    def t265_open(self):
        while not self.client_t265_open.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('正在等待t265_open服务的server启动')
        self.get_logger().info('t265_open服务的server启动成功,已发送启动请求')
        self.t265OpenRequest = Empty.Request()
        self.t265service_future = self.client_t265_open.call_async(self.t265OpenRequest) # 异步方式发送服务请求

    def sync_position_data(self, msg: T265Data):
        # 处理接收到的t265数据，发布到uart3主题
        self.pos = msg  # 更新最新的t265数据到成员变量
        if abs(self.pos.pos_z ) > self.heightMax:
            msg_land_cmd = String()
            msg_land_cmd.data = 'S' # 0x53 Land
            self.topic_uart4_pub.publish(msg_land_cmd) 
            self.get_logger().warning(f"飞机高度失效！！！")
 
    def commander_callback(self, request, response):  # 接收地面站指令！
        # linux-uart3接mcu uart7 实时的t265数据  
        # linux-uart4接mcu uart2 不定时的消息指令

        # 前往定点 G 写了
        # 起飞    R 写了
        # 降落锁死 S 写了
        # 降落正常 U 写了
        # 旋转朝向 T 没写
        msg = String()
        
        if request.req.startswith('H') and len(request.req) == 19:
            # 手动控制：H开头，长度19
            modified_msg = 'G' + request.req[1:]  # 替换首字母为 G
            msg.data = modified_msg
            self.topic_uart4_pub.publish(msg)
            # 激活手动优先级：屏蔽后续 编程控制 消息
            self.hand_active = True
            self.get_logger().warning(f"进入手动控制模式，屏蔽编程位置控制")
        
        # linux uart3 - MCU uart7 t265实时数据  t265就绪起飞
        # linux uart4 - MCU uart2 其他指令：降落  飞向定点 
        elif request.req == 'takeoff' and self.heart_active == True: # 起飞指令 takeoff
            msg.data = 'R'
            self.topic_uart4_pub.publish(msg) # 0x52   
            self.is_flying = True
            self.get_logger().warning(f"飞行站已调用：{request.req}")
            response.echo = f"飞行站已调用：{request.req}服务,心跳正常,允许起飞，正在起飞"  
            return response
        
        elif request.req == 'land': # 降落指令 land
            msg.data = 'S' # 0x53
            self.topic_uart4_pub.publish(msg)   
        elif request.req == 'normalLand': # 降落指令 land
            msg.data = 'U' # 0x55
            self.topic_uart4_pub.publish(msg)
        elif request.req == 'hover':
            msg.data = TGformat('G',self.pos.pos_x, self.pos.pos_y, self.pos.pos_z,'')
            self.topic_uart4_pub.publish(msg)
            if msg.data.startswith('G') and len(msg.data) == 19:
                self.get_logger().warning(f"悬停呼唤成功")

        elif request.req.startswith('G') and len(request.req) == 19 and self.hand_active == False :  # 首字母是 'G' 且总长度为 19, 并且无手动干预时，编程控制飞往目标点
            msg.data = request.req
            self.topic_uart4_pub.publish(msg)
        else:
            msg.data = request.req # 如果是其他的杂乱消息，不理会，但是打印出来
            self.get_logger().warning(f"navigator杂乱消息: {request.req}  ")
       
        self.get_logger().warning(f"飞行站已调用：{request.req} service called")
        response.echo = f"飞行站已调用：{request.req}服务" # 打印一下服务调用
        return response
        # elif request.req == 'down40':
        #     pos_z_down = self.pos.pos_z-0.4
        #     if pos_z_down <  0 : # 最低高度
        #         pos_z_down = self.heightMax
        #     msg.data = TGformat('G',self.pos.pos_x, self.pos.pos_y, pos_z_down,'')
        #     self.topic_uart4_pub.publish(msg)
        #     if msg.data.startswith('G') and len(msg.data) == 19:
        #         self.get_logger().warning(f"下降呼唤成功")
        # elif request.req == 'up40':
        #     pos_z_up = self.pos.pos_z+0.4
        #     if pos_z_up > self.heightMax : # 限制最大高度
        #         pos_z_up = self.heightMax
        #     msg.data = TGformat('G',self.pos.pos_x, self.pos.pos_y, pos_z_up,'')
        #     self.topic_uart4_pub.publish(msg)
        #     if msg.data.startswith('G') and len(msg.data) == 19:
        #         self.get_logger().warning(f"上升呼唤成功")
                
    

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
                                
