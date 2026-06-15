from periphery import PWM
import time

# 实例化 PWM 对象，参数分别为 chip（芯片编号） 和 channel（通道编号）
# 例如使用 pwmchip0 的 channel 0
# 3-0 内测
# 1-0 外侧

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from periphery import PWM
import traceback
from functools import partial



class PwmServoNode(Node):
    def __init__(self):
        super().__init__('Servo')

        self.pwmPolarity = 'inversed'
        self.lockAngle = 64
        self.unLockAngle = 110
        # 1. 初始化 periphery PWM (使用你定义的 chip=1, channel=0)
        try:
            # 定义硬件通道
            self.Servo = [PWM(1, 0), PWM(3, 0)]

            # 使用 for 循环批量初始化
            for i, servo in enumerate(self.Servo):
                # 1. 打印初始极性

                current_polarity = servo.polarity
                self.get_logger().info(f"舵机 [{i}] 初始 PWM 极性: {current_polarity}")
                
                # 2. 尝试设置为 normal，如果失败则保持原样
                if current_polarity != "normal":
                    try:
                        servo.polarity = "normal"
                        self.get_logger().info(f"舵机 [{i}] 极性已成功设置为 normal")
                    except OSError as e:
                        # 捕获异常：说明硬件不支持修改，或当前状态无法更改
                        self.get_logger().warn(f"舵机 [{i}] 无法设置为 normal (原因: {e})，将保持当前极性: {servo.polarity}")
                else:
                    self.get_logger().info(f"舵机 [{i}] 已经是 normal 极性，无需修改")
                
                servo.frequency = 50
                servo.duty_cycle = self.angle_to_pwnDuty(self.lockAngle, servo.polarity)  # 初始位置
                servo.enable()

                
                self.get_logger().info(f"舵机通道 [{i}] (PWM {servo.chip}, {servo.channel}) 初始化成功！")
            self.get_logger().info("PWM 节点初始化成功！当前频率: 50Hz")
        except Exception as e:
            self.get_logger().error(f"PWM 初始化失败: {e}")
            traceback.print_exc()
            raise e

        # 2. 创建订阅者，话题名为 'servo_angle'，类型为 String 
        self.subscription0 = self.create_subscription(String,'fly/servo10', partial(self.angle_callback,index=0), 10)
        self.subscription1 = self.create_subscription(String,'fly/servo30', partial(self.angle_callback,index=1), 10)
        
        self.get_logger().info("已订阅话题: 'fly/servo**'，等待接收角度字符串...")
 
    def angle_callback(self, msg, index):
        try:
            angle_str = msg.data.strip()
            # 特殊动作： lock  unlock
            if angle_str == 'lock': # 俩通道执行相同的操作，接不同的接口没问题
                self.Servo[0].duty_cycle = self.angle_to_pwnDuty(self.lockAngle, self.Servo[0].polarity)
                self.Servo[1].duty_cycle = self.angle_to_pwnDuty(self.lockAngle, self.Servo[1].polarity)
                self.get_logger().info(f"舵机 [{index}] 成功接收角度: {angle_str}°, 锁定状态")
                return
            elif angle_str == 'unlock':
                self.Servo[0].duty_cycle = self.angle_to_pwnDuty(self.unLockAngle, self.Servo[0].polarity) 
                self.Servo[1].duty_cycle = self.angle_to_pwnDuty(self.unLockAngle, self.Servo[1].polarity)
                self.get_logger().info(f"舵机 [{index}] 成功接收角度: {angle_str}°, 投放状态")
                return
            
            # 解析接收到的字符串为浮点数
            angle = float(angle_str)
            # 调用你的角度转占空比函数
            duty_cycle = self.angle_to_pwnDuty(angle,self.Servo[0].polarity)
            # 【核心修改】通过 index 动态控制 self.Servo 列表中对应的舵机
            self.Servo[index].duty_cycle = duty_cycle
            self.get_logger().info(
                f"舵机 [{index}] 成功接收角度: {angle_str}° -> 转换占空比: {duty_cycle:.4f}"
            )

        except ValueError:
            self.get_logger().warn(f"舵机 [{index}] 收到无效输入: '{msg.data}'，请输入数字。")
        except Exception as e:
            self.get_logger().error(f"设置舵机 [{index}] PWM 失败: {e}")



    def angle_to_pwnDuty(self, angle:float, polarity:str = 'normal'):
            # 限制角度在 0 到 180 度之间
            # print(f"极性是{polarity}")
            if angle < 0.0: angle = 0.0
            if angle > 180.0: angle = 180.0

            # --- 角度转占空比计算逻辑 (以50Hz频率，0.5ms~2.5ms脉宽的标准舵机为例) ---
            # 0度   -> 0.5ms 脉宽 -> 占空比 = 0.5ms / 20ms = 0.025 (2.5%)
            # 180度 -> 2.5ms 脉宽 -> 占空比 = 2.5ms / 20ms = 0.125 (12.5%)
            # 公式: duty = 0.025 + (angle / 180.0) * (0.125 - 0.025)
            if polarity == 'normal':
                duty =  0.025 + (angle / 180.0) * 0.10 # servo.polarity = "normal" # 强制将极性纠正为高电平有效 (normal)
            elif polarity == 'inversed':
                duty = (180-angle)/180.0 * 0.1 + 0.875  # servo.polarity = "inversed" 但是貌似rk3855 pwm输出是是默认为"inversed" 占空比表示低电平，额
            return duty
    
    

def destroy_node(self):
    # 使用 for 循环批量安全关闭
    if hasattr(self, 'Servo'):
        for i, servo in enumerate(self.Servo):
            try:
                servo.disable()
                servo.close()
                self.get_logger().info(f"舵机通道 [{i}] 硬件已安全关闭。")
            except Exception as e:
                self.get_logger().error(f"关闭舵机通道 [{i}] 失败: {e}")
    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PwmServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("接收到退出信号。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# if __name__ == '__main__':

#     from periphery import PWM
#     import time

#     try:
#         angle = 64
#         servo = PWM(1, 0)
#         servo.polarity = "normal"
#         servo.frequency = 50
#         servo.duty_cycle= 0.025 + (angle / 180.0) * 0.10
#         servo.enable()

#         print("舵机控制模式已启动！")
#         print("请输入 0.0 ~ 180.0 之间的浮点数来控制角度。")
#         print("输入 'q' 或 'quit' 退出程序。\n")

#         while True:
#             # 获取用户输入
#             user_input = input("请输入目标角度: ").strip()
            
#             # 支持输入 q 或 quit 优雅退出
#             if user_input.lower() in ('q', 'quit'):
#                 print("收到退出指令，正在关闭...")
#                 break
            
#             try:
#                 # 将输入转换为浮点数
#                 angle = float(user_input)
                
#                 # 检查角度是否在有效范围内
#                 if not (0 <= angle <= 180):
#                     print("⚠️ 警告: 角度必须在 0 ~ 180 之间，请重新输入！\n")
#                     continue
                
#                 # 将角度转换为占空比 (0.025 ~ 0.125)
#                 duty_cycle = 0.025 + (angle / 180.0) * 0.10
#                 servo.duty_cycle = duty_cycle
                
#                 print(f"✅ 当前角度: {angle}° | 占空比: {duty_cycle:.4f}\n")
                
#                 # 稍微延时，让舵机有时间响应并稳定
#                 time.sleep(0.1) 
                
#             except ValueError:
#                 print("⚠️ 错误: 无效的输入！请输入有效的数字或 'q' 退出。\n")

#     except KeyboardInterrupt:
#         print("\n测试被手动中断 (Ctrl+C)。")
#     except Exception as e:
#         print(f"发生系统错误: {e}")
#     finally:
#         # 确保无论发生什么，都能安全关闭硬件资源
#         if 'servo' in locals():
#             servo.disable()
#             servo.close()
#             print("PWM 通道已安全关闭。")