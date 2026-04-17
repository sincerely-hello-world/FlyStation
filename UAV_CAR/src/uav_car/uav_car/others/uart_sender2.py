import uav_car_unit.uart_sender as us

import rclpy 


def main(args=None):
    rclpy.init(args=args)                                  
    uartSender = us.UartSender("uart_sender2", "/dev/ttyS2", 57600)                                
    rclpy.spin(uartSender)                                     
    uartSender.destroy_node()                                   
    rclpy.shutdown()  
