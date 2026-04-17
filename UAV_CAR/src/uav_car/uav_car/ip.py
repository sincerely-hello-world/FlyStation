import socket
def get_local_ip():
   s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   try:
       # 连接到公共 DNS 服务器（如 Google 的 8.8.8.8）
       s.connect(("8.8.8.8", 80))
       ip = s.getsockname()[0]
   finally:
       s.close()
   return ip
# print(f"当前网络接口的局域网 IP 是: {get_local_ip()}")

 

#----------------------------
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
import socket

class IP_Pub_Node(Node):
    def __init__(self,name='fly_ip'):
        super().__init__(name)
 
        self.topic_publisher_fly_ip = self.create_publisher(
            String,
            '/fly_ip',   
            10
        )
         
        self.topic_publisher_ip = self.create_publisher(String, 'fly_ip_topic', 10)
        # 打印到 ROS 日志（推荐）
        self.get_logger().info(f"ip地址发布节点已启动") 

        # 创建定时器（相当于ROS的循环频率）
        # 这里每秒检测30帧（约33ms一次）
        self.qr_timer = self.create_timer(0.033, self.ip_pub_timer_callback)

    def ip_pub_timer_callback(self):

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = IP_Pub_Node()
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

