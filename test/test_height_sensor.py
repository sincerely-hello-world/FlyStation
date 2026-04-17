import serial
from tofsense import TOFSense_P_F

ser = serial.Serial("/dev/ttyS1", 921600)
tof = TOFSense_P_F(ser)


last_valid_data = None

def get_distance():
    global last_valid_data
    data_all = tof.get_data()
    
    if data_all and "error" not in data_all:
        current_dis = data_all.get("dis")
        if current_dis is not None:
            last_valid_data = data_all
            return current_dis
    
    # 如果当前读取失败，返回上一次的有效数据
    return last_valid_data.get("dis") if last_valid_data else None

# 使用示例
data_z = get_distance()
