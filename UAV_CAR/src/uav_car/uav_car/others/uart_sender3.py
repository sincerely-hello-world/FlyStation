import uav_car_unit.uart_sender as us

import rclpy 


def main(args=None):
    rclpy.init(args=args)                                  
    uartSender = us.UartSender("uart_sender3", "/dev/ttyS3", 57600, 3)                                
    rclpy.spin(uartSender)                                     
    uartSender.destroy_node()                                   
    rclpy.shutdown()  
