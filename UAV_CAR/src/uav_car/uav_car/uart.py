import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String  #  ROS2 uart串口 接收发送的话题，均使用 msg/String
from rclpy.executors import MultiThreadedExecutor

class UartClass(Node):
    def __init__(self,
                 node_name='uartx_node', # 替换为实际节点名称，如 'uart3_node' 或 'uart4_node'
                 serial_port='/dev/ttySx',  # 替换为实际串口设备，如 /dev/ttyS3 或 /dev/ttyS4
                 baudrate=57600,
                 frame_length=9,  # 接收端 定长数据帧 长度为 frame_length
                 frame_header='&',

                 send_topic_name='uart_senderx_data_topic',            # 发送端订阅的 topic 名
                 recv_topic_name='uart_readerx_data_topic',            # 接收端发布的 topic 名
                 debug_send=True,
                 debug_recv=True,
                 ):
        super().__init__(node_name)

        self.serial_port = serial_port
        self.baudrate = baudrate
        self.frame_length = frame_length
        self.frame_header = frame_header.encode()  # 转为 bytes
        self.debug_send = debug_send
        self.debug_recv = debug_recv
        self.stop_thread_flag = False
        
        # 共用一个串口对象（关键！）
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.02)
            self.get_logger().info(f'{self.get_name()}串口 {serial_port} 打开成功，波特率 {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口:{self.get_name()}:{e}')
            raise
        # === 发送部分 ===
        self.send_topic_name =   send_topic_name
        self.sub_send = self.create_subscription(
            String, topic=self.send_topic_name, callback=self.send_uart_data, qos_profile=10
        )
        self.get_logger().info(f'{self.get_name()}串口 {serial_port},已订阅发送主题: {self.send_topic_name}')

        # === 接收部分 ===
        self.recv_topic_name = recv_topic_name
        self.pub_recv = self.create_publisher(String, self.recv_topic_name, 10)
        self.get_logger().info(f'{self.get_name()}串口 {serial_port},将发布接收数据到主题: {self.recv_topic_name}')

        # 启动接收线程
        
        self.recv_thread = threading.Thread(target=self.read_serial)
        self.recv_thread.start()
        

    def send_uart_data(self, msg: String):
        """回调：当有数据要通过串口发送时"""
        try:
            send_content = msg.data #.strip()
            # hex_with_0x = ''.join(f'0x{b:02x}' for b in send_content.encode())
            if not send_content:
                self.get_logger().warn(f'空字符串，未发送任何数据')
                return
 
            if self.debug_send: # 打印发送的十六进制字符串和对应的 ASCII 字符 要发送数据hex为{hex_with_0x}
                self.get_logger().info(f"要发送数据为{send_content} success")
 
            if self.ser.is_open: 
                sent = self.ser.write(send_content.encode('utf-8'))
                if sent != len(send_content):
                    self.get_logger().warn(f'发送错误：仅发送了 {sent}/{len(send_content)} 字节 error')
        except ValueError as e:
            self.get_logger().error(f'无效的消息内容: "{msg.data}" - {e}')
        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')
    # def send_uart_data(self, msg: String): #  可以发crc校验码版本
    #     """回调：当有数据要通过串口发送时"""
    #     try:
    #         hex_str = msg.data #.strip()
    #         if not hex_str:
    #             return
    #         # 将十六进制字符串转为字节（如 "4142" -> b'AB'）
    #         data_bytes = bytes.fromhex(hex_str)
    #         if self.debug_send: # 打印发送的十六进制字符串和对应的 ASCII 字符
    #             readable = data_bytes.decode('ascii')
    #             self.get_logger().info(f"要发送数据 hex_str:{msg.data} 即 data_bytes:{readable}; success")
 
    #         if self.ser.is_open: # 实际写给串口的是 data_bytes 
    #             sent = self.ser.write(data_bytes)
    #             if sent != len(data_bytes):
    #                 self.get_logger().warn(f'发送错误：仅发送了 {sent}/{len(data_bytes)} 字节 error')

    #     except ValueError as e:
    #         self.get_logger().error(f'无效的十六进制字符串: "{msg.data}" - {e}')
    #     except Exception as e:
    #         self.get_logger().error(f'发送失败: {e}')

    def read_serial(self):
        """后台线程：持续读取串口数据并解析帧"""
        # buffer = b''
        # max_buffer_size = self.frame_length * 3

        while rclpy.ok() and not self.stop_thread_flag:
            data = self.ser.readline()
            #data = self.ser.readline()
            if data:
                # 先尝试解码，遇到非法 UTF-8 用 * 替代（但 \x00 不会触发这里）
                decoded = data.decode('utf-8', errors='replace')
                # 再把 decode 后的字符串中所有不可见字符（含 \x00）替换为 *
                rec_line = sanitize_string(decoded, '*')
                # rec_line = data.decode('utf-8')
                msg = String()
                msg.data = rec_line
                self.pub_recv.publish(msg)
                if self.debug_recv:
                    self.get_logger().info(f"接收到帧: {rec_line}")
            # if self.ser.in_waiting > 0:
            #     raw = self.ser.read(self.ser.in_waiting)
            #     buffer += raw
            #     if len(buffer) > max_buffer_size:
            #         buffer = buffer[-max_buffer_size:]
            #     # 尝试提取完整帧
            #     while len(buffer) >= self.frame_length:
            #         if buffer.startswith(self.frame_header):
            #             frame = buffer[:self.frame_length]
            #             buffer = buffer[self.frame_length:]
            #             try:
            #                 line = frame.decode('utf-8', errors='replace').strip()
            #                 if self.debug_recv:
            #                     self.get_logger().info(f"接收到帧: {line}")
            #                 msg = String()
            #                 msg.data = line
            #                 self.pub_recv.publish(msg)
            #             except Exception as e:
            #                 self.get_logger().warn(f'帧解码失败: {e}')
            #         else:
            #             # 丢弃第一个字节，继续找帧头
            #             buffer = buffer[1:]
            # else:
            #     time.sleep(0.005)  # 减少 CPU 占用

    def destroy_node(self):
        self.stop_thread_flag = True
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)

    # 创建两个串口读取节点
    uart3_node = UartClass( # uart3 发送 t265数据给 MCU的Uart7
        node_name='UART3',
        serial_port='/dev/ttyS3',
        baudrate=57600,
        send_topic_name='uart_sender3_data_topic',
        recv_topic_name='uart_reader3_data_topic',
        # frame_length=9,
        # frame_header='&',
        debug_send=False,# 
        debug_recv=True,
    )

    uart4_node = UartClass( # uart4 发送  控制指令 MCU的Uart2
        node_name='UART4',
        serial_port='/dev/ttyS4',
        baudrate=57600,
        send_topic_name='uart_sender4_data_topic',
        recv_topic_name='uart_reader4_data_topic',
        # frame_length=9,
        # frame_header='&',
        debug_send=False,# 
        debug_recv=True,
    )
        # 使用多线程执行器，让两个节点并发运行
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(uart3_node)
    executor.add_node(uart4_node)

    try:
        # 阻塞运行，直到 Ctrl+C
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        uart3_node.destroy_node()
        uart4_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


def sanitize_string(s: str, replacement: str = '*') -> str:
    """
    将字符串中所有控制字符（包括 \x00）、非打印字符替换为 replacement。
    """
    # 方法1：只保留可打印 ASCII（32~126），其余替换为 *
    cleaned = ''.join(
        c if 32 <= ord(c) <= 126 else replacement
        for c in s
    )
    return cleaned
