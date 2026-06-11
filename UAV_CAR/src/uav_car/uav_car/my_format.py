#要发送数据 hex_str:54 2B 30 30 30 30 30 2B 30 30 30 30 31 2D 30 30 30 30 38   
# 即 data_bytes:T+00000+00001-00008; success
#要发送数据 hex_str:54 2D 30 30 30 30 30 2B 30 30 30 30 30 2D 30 30 30 30 35   
# 即 data_bytes:T-00000+00000-00005; success
# position confidence:[2],pos(x:-0.033m, y:-0.068m), h[-0.000m]
# 要发送数据 hex_str:54 2D 30 30 33 32 37 2D 30 30 36 37 35 2D 30 30 30 30 34   
# 即 data_bytes:T-00327-00675-00004; success

# pos(x:+0.062m, y:+0.153m), h[-0.002m]

# self.get_logger().info(
#         #     f'position confidence:[{self.position.confidence:1d}],'
#         #     f'pos(x:{self.position.pos_x:+5.3f}m, y:{self.position.pos_y:+5.3f}m), '
#         #     f'h[{self.position.pos_z:+5.3f}m]')

def Meter_to_mm(value): 
    
    '''
     将单位为米的值转换为毫米,并格式化为5位整数,保留符号
     例如：-123.45678m -> -23456mm -> "-23456"
     例如：+123.45678m -> +23456mm -> "+23456"
     例如：0.12m ->  120mm -> "+00120"
     例如：-0.1m -> -100mm -> "-00100"
    '''
    return ('+' if value >= 0 else '-') + f"{int(abs(value) * 1000) % 100000:05d}"
    
def Meter_to_mmm(value): 
    return ('+' if value >= 0 else '-') + f"{int(abs(value) * 10000) % 100000:05d}"
    
def TGformat(head: str, x: float, y: float, z: float, end: str, input_tip="输入的xyz数字单位均为米，输出的字符串格式见函数过程", maxHeightZ = 2.1):

    
#    原版 = 1
#    if 原版 :
#        head='T' # original
#        end =''
#        x_mm = Meter_to_mmm(x)
#        y_mm = Meter_to_mmm(y)
#        z_mm = Meter_to_mmm(z)
#    else:
#        head='#' 
#        end =''
#        x_mm = Meter_to_mm(x)
#        y_mm = Meter_to_mm(y)
#        z_mm = Meter_to_mm(z)

        # x=123.45678, y=123.45678, z=123.45678 
        # return=  '#+23456+23456+23456?'

    if z > maxHeightZ:
        z = maxHeightZ
    head = head
    x_mm = Meter_to_mmm(x)
    y_mm = Meter_to_mmm(y)
    z_mm = Meter_to_mmm(z)
    end  = ''
    
    formatted_str = f"{head}{x_mm}{y_mm}{z_mm}{end}" # 1 + (1+5)* 3 + 0  = 19  19byte定长度字符串，方便串口解析
    return formatted_str
 
if __name__ == "__main__":
    
    result = format(head='T', x=123.45678, y=123.45678, z=123.45678, end='')
    print(result)  # 输出: +23456
 
