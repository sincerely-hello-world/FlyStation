import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool

class FlyHeartBeatNode(Node):
    def __init__(self):
        super().__init__('fly_heartbeat_client')
        
        # 配置参数
        self.timeout_duration = 2.0  # 超时阈值：2秒
        
        # 使用节点时钟记录上一次收到心跳的时间
        # 初始化为当前时间，防止启动时瞬间误判超时
        self.last_heartbeat_time = self.get_clock().now()
        self.is_connected = False # 初始状态为未连接

        # 订阅地面站的心跳话题
        self.heartbeat_sub = self.create_subscription(
            Empty,
            '/GroundStation/heartbeat', 
            self.heartbeat_callback,
            10
        )

        # 发布飞行站的接收心跳的状态（给控制节点看）
        self.status_pub = self.create_publisher(Bool, '/FlyStation/heartbeat/status', 10)
        
        # 创建高频检查定时器（每 0.3 秒检查一次是否超时，保证误差不超过 0.3 秒）
        self.check_timer = self.create_timer(0.3, self.check_heartbeat_timeout)
        
        self.get_logger().info('飞行站心跳检测节点已启动，等待地面站首次心跳...')

    def heartbeat_callback(self, msg):
        # 收到心跳，更新时间戳
        self.last_heartbeat_time = self.get_clock().now()
        
        if not self.is_connected:
            self.is_connected = True
            self.get_logger().info('与地面站建立连接！')
            self.publish_status(True)

    def check_heartbeat_timeout(self):
        # 计算当前时间与上一次收到心跳时间的差值
        now = self.get_clock().now()
        elapsed_time = (now - self.last_heartbeat_time).nanoseconds / 1e9 # 转换为秒

        if elapsed_time > self.timeout_duration:
            # 如果之前是连接状态，说明刚刚发生超时
            if self.is_connected:
                self.is_connected = False
                self.get_logger().error(f'心跳超时！已持续 {elapsed_time:.2f} 秒未收到心跳！')
            
            # 持续发布断开状态，并触发安全降落
            self.publish_status(False)
            self.fly_land_call()
        else:
            # 心跳正常，持续发布正常状态
            if self.is_connected:
                self.publish_status(True)
                self.get_logger().info('飞行站： 与地面站心跳链接正常', throttle_duration_sec=1.0)

    def publish_status(self, status: bool):
        msg = Bool()
        msg.data = status
        self.status_pub.publish(msg)

    def fly_land_call(self):
        # 这里可以留空，或者在这里直接调用降落服务/发布降落指令
        # 既然你打算让控制节点接收 /FlyStation/heartbeat/status，控制节点负责降落即可
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FlyHeartBeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()