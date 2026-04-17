import rclpy                                      
from rclpy.node import Node 

import serial         

from std_msgs.msg import String                  


class UartSender(Node):
    
    def __init__(self, name, serial_port, baudrate, topic_stack=50):
        super().__init__(name)

        self.name = name # uart_sender0~4
        self.topic_stack = topic_stack

        self.sub = self.create_subscription(String, name + "_data_topic", self.send_uart_data, topic_stack) 

        self.serial_port = serial_port
        self.baudrate = baudrate
        self.sink_serial = serial.Serial(self.serial_port, self.baudrate)
        
        self.get_logger().warning(f"init success")

    def send_uart_data(self, msg):
        # self.get_logger().info(f"request to send:{msg.data}")
        if self.sink_serial.isOpen():
            hex_msg = bytes.fromhex(msg.data)
            send_count = self.sink_serial.write(hex_msg)
            if send_count == len(hex_msg):
                return True         
            else:
                return False

  