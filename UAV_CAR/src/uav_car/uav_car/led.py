#!/usr/bin/env python3

import rclpy
from rclpy.node import Node,Timer
from std_srvs.srv import Trigger
from periphery import GPIO

 

class LEDNode(Node):
    LED_ON  = True
    LED_OFF = False
    def __init__(self):
        super().__init__('LED')

        # 配置 GPIO 引脚编号（根据你的硬件平台修改）
        self.LED1_PIN_RED = 35
        self.LED2_PIN_GREEN = 54

        # 初始化 GPIO（输出模式）
        try:
            self.led1 = GPIO(self.LED1_PIN_RED, "out")
            self.led1.write(self.LED_OFF)
            self.led2 = GPIO(self.LED2_PIN_GREEN, "out")
            self.led2.write(self.LED_OFF)
            self.get_logger().info(f'GPIO {self.LED1_PIN_RED} and {self.LED2_PIN_GREEN} initialized as outputs.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            raise

        # 在 __init__ 中：
        self.create_service(Trigger, 'led1_on',  lambda r, s: self._set_led(self.led1, self.LED_ON, 'pin35(Red)-ON', r, s))
        self.create_service(Trigger, 'led1_off', lambda r, s: self._set_led(self.led1, self.LED_OFF,'pin35(Red)-OFF', r, s))
        self.create_service(Trigger, 'led2_on',  lambda r, s: self._set_led(self.led2, self.LED_ON, 'pin54(GREEN)-ON', r, s))
        self.create_service(Trigger, 'led2_off', lambda r, s: self._set_led(self.led2, self.LED_OFF,'pin54(GREEN)-OFF', r, s))

        self.create_service(Trigger, 'led1_trigger', self.timer_to_shutdown_led1)
        self.create_service(Trigger, 'led2_trigger', self.timer_to_shutdown_led2)
        
        self.led1_timer = None
        self.led2_timer = None

        self.get_logger().info('LED Node 服务初始化完成')

    def led1_turn_off(self):
        self.led1.write(self.LED_OFF)
        self.led1_timer.cancel()
        self.led1_timer = None
        self.get_logger().info('LED1 trigger off')

    def timer_to_shutdown_led1(self,request, response):
        self.get_logger().info('LED1 trigger on')
        self.led1.write(self.LED_ON)
        if self.led1_timer is not None:
            self.led1_timer.cancel()
            self.led1_timer = None
        self.led1_timer = self.create_timer(0.79, self.led1_turn_off)
        response.message = "LED1: trigger ok "
        return response

    def led2_turn_off(self):
        self.led2.write(self.LED_OFF)
        self.led2_timer.cancel()
        self.led2_timer = None
        self.get_logger().info('LED2 trigger off')

    def timer_to_shutdown_led2(self,request, response):
        self.get_logger().info('LED2 trigger on')
        self.led2.write(self.LED_ON)
        if self.led2_timer is not None:
            self.led2_timer.cancel()
            self.led2_timer = None
        self.led2_timer = self.create_timer(1.23, self.led2_turn_off)
        response.message = "LED2: trigger ok "
        return response

    # 统一处理函数
    def _set_led(self, gpio, state, tip, request=None, response=None):
        gpio.write(state)
        response.success = True
        response.message = f"LED: {tip}"
        self.get_logger().info(response.message)
        return response

    def cleanup(self):
        for name in ['led1', 'led2']:
            led = getattr(self, name, None)
            if led is not None:
                led.write(self.LED_OFF)
                led.close()
        self.get_logger().info('GPIO resources cleaned up.')

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()