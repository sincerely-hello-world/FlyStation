TOFSENSE_P_MARK = "00"
TOFSENSE_M_MARK = "01"
TOFSENSE_DATA_LEN = 16
TOFSENSE_DATA_LEN_M_88 = 400
TOFSENSE_DATA_LEN_M_44 = 112


class TOFSense:
    def __init__(self, Frame_Mark, ser=None):
        self.Frame_Header = "57"  # 固定帧头
        self.Frame_Mark = Frame_Mark  # 固定帧标记
        self.ser = ser

        if Frame_Mark == TOFSENSE_P_MARK:
            self.datalen = TOFSENSE_DATA_LEN
        elif Frame_Mark == TOFSENSE_M_MARK:
            self.datalen = TOFSENSE_DATA_LEN_M_88  # 8*8模式
            # self.datalen = TOFSENSE_DATA_LEN_M_44  # 4*4模式

    def get_datafromser(self):
        if self.ser is None:
            return None

        temp = self.ser.read(1).hex()
        if temp == self.Frame_Header:
            temp = self.ser.read(1).hex()
            if temp == self.Frame_Mark:
                raw_data = (
                    self.Frame_Header
                    + self.Frame_Mark
                    + self.ser.read(self.datalen - 2).hex()
                )
                return raw_data
        return None

    def check_data(self, data):
        if data is None:
            return False

        # 以ser.read()读取数据为bytes类型
        if isinstance(data, bytes):
            str_data = data.hex()
        elif isinstance(data, str):
            str_data = data
        else:
            return False

        # 校验和
        data_temp, original_checksum = str_data[:-2], str_data[-2:]
        checksum = sum(
            int(data_temp[i : i + 2], 16) for i in range(0, len(data_temp), 2)
        )
        calculated_checksum = hex(checksum)[-2:]

        if calculated_checksum != original_checksum:
            return False
        # 校验和通过

        # 数据切割
        cleaned_str = str_data.strip()
        hex_values = [cleaned_str[i : i + 2] for i in range(0, len(cleaned_str), 2)]

        # 帧头判断
        return (
            hex_values
            if hex_values[0] == self.Frame_Header and hex_values[1] == self.Frame_Mark
            else False
        )

    def send_read_frame(self, id):
        if self.ser is None:
            return None
        try:
            # 协议数据
            protocol_data = [0x57, 0x10, 0xFF, 0xFF, id, 0xFF, 0xFF]

            # 更新校验和
            checksum = sum(protocol_data) & 0xFF

            # 拼接更新后的校验和
            protocol_data.append(checksum)

            # 转化为bytes
            protocol_bytes = bytes(protocol_data)

            # 发送
            self.ser.write(protocol_bytes)

        except Exception as e:
            print(f"发生异常: {e}")
            return None
        return True


# 该类支持TOFSense-P/PS/F/FP
class TOFSense_P_F(TOFSense):
    def __init__(self, ser):
        super().__init__(TOFSENSE_P_MARK, ser)
        self.data_dict = {
            "id": 0,
            "system_time": 0,
            "dis": 0,
            "dis_status": 0,
            "signal_strength": 0,
            "range_precision": 0,
        }

    def get_data(self):
        """
        获取所有解析数据
        return: {0}: 解析错误
                >0: 解析数据
        """
        data = self.get_datafromser()
        return self.__unpack_data_str(data) if data else {0}

    def get_data_inquire(self, id):
        """
        获取所有解析数据
        id : 传感器ID 0~255
        return: {0}: 解析错误
                >0: 解析数据
        """
        self.send_read_frame(id)
        data = self.get_datafromser()
        return self.__unpack_data_str(data) if data else {0}

    def __unpack_data_str(self, data):
        temp = self.check_data(data)
        # 解析数据
        if temp == False:
            return False

        self.data_dict["id"] = int(temp[3], 16)  # 传感器ID
        self.data_dict["system_time"] = int(
            temp[7] + temp[6] + temp[5] + temp[4], 16
        )  # 传感器上电时间
        self.data_dict["dis"] = (
            int(temp[10] + temp[9] + temp[8], 16)
        ) / 1000  # 测距距离 单位m
        self.data_dict["dis_status"] = int(temp[11], 16)  # 距离状态指示
        self.data_dict["signal_strength"] = int(temp[13] + temp[12], 16)  # 信号强度
        self.data_dict["range_precision"] = int(temp[14], 16)  # 距离状态指示
        return self.data_dict


# 该类支持TOFSense-M/MS
class TOFSense_M(TOFSense):
    def __init__(self, ser):
        super().__init__(TOFSENSE_M_MARK, ser)
        self.data_dict = {
            "id": 0,
            "system_time": 0,
            "zone_map": 0,
            "dis": [0.0] * 64,
            "dis_status": [0] * 64,
            "signal_strength": [0] * 64,
        }

    def get_data(self):
        """
        获取所有解析数据
        return: {0}: 解析错误
                >0: 解析数据
        """
        data = self.get_datafromser()
        return self.__unpack_data_str(data) if data else {0}

    def get_data_inquire(self, id):
        """
        获取所有解析数据
        id : 传感器ID 0~255
        return: {0}: 解析错误
                >0: 解析数据
        """
        self.send_read_frame(id)
        data = self.get_datafromser()
        return self.__unpack_data_str(data) if data else {0}

    def __unpack_data_str(self, data):
        temp = self.check_data(data)
        # 解析数据
        if temp == False:
            return False

        self.data_dict["id"] = int(temp[3], 16)  # 传感器ID
        self.data_dict["system_time"] = int(
            temp[7] + temp[6] + temp[5] + temp[4], 16
        )  # 传感器上电时间
        self.data_dict["zone_map"] = int(temp[8], 16)
        for i in range(0, 64):
            self.data_dict["dis"][i] = round(
                (
                    (int(temp[11 + i * 6] + temp[10 + i * 6] + temp[9 + i * 6], 16))
                    / 1000
                    / 1000
                ),
                2,
            )  # 测距距离 单位m
            self.data_dict["dis_status"][i] = int(temp[12 + i * 6], 16)
            self.data_dict["signal_strength"][i] = int(
                temp[14 + i * 6] + temp[13 + i * 6], 16
            )
        return self.data_dict