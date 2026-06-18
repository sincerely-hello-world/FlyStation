import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class GroundStationHeartbeatNode(Node):
    def __init__(self):
        super().__init__('ground_station_heartbeat')
        
        # 1. 核心参数配置
        # 建议发送周期小于飞行站的超时阈值（例如：飞行站3秒超时，地面站1秒发一次非常安全）
        self.publish_period = 0.53  # 单位：秒
        
        # 2. 创建发布者（专职发布心跳话题）
        # 话题名称必须与飞行站订阅的名称完全一致：'/GroundStation/heartbeat'
        self.heartbeat_pub = self.create_publisher(
            Empty, 
            '/GroundStation/heartbeat', 
            10
        )
        
        # 3. 创建定时器，按照固定周期执行发布
        self.timer = self.create_timer(self.publish_period, self.publish_heartbeat)
        
        self.get_logger().info(f'地面站心跳节点已启动，正在以 {self.publish_period}s 的周期持续发送心跳...')

    def publish_heartbeat(self):
        # 实例化一个空的 Empty 消息
        msg = Empty()
        
        # 发布消息
        self.heartbeat_pub.publish(msg)
        
        # 为了不让终端被日志刷屏，可以使用 debug 级别日志，或者每10次打印一次
        # 这里用 info 打印，方便你初期调试观察
        # self.get_logger().debug('已发送地面站心跳 [Empty]', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = GroundStationHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        for i in range(5):
            node.get_logger().warn('地面站心跳节点被手动关闭！无人机可能会触发心跳断开保护！')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()