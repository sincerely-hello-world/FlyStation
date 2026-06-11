#识别多个色块扩展
import cv2 as cv  # OpenCV库，用于计算机视觉任务
import numpy as np  # 数值计算库，用于数组操作
 
# 空函数用于滑动条回调（OpenCV滑动条必须绑定回调函数）
def nothing(x):
    pass
 
# 初始化摄像头
cap = cv.VideoCapture(0)  # 参数1表示使用第二个摄像头设备（0为默认摄像头）
if not cap.isOpened():
    raise IOError("无法打开摄像头")  # 摄像头初始化失败时报错
 
# 创建控制窗口和滑动条
cv.namedWindow('controls', cv.WINDOW_NORMAL)  # 创建可调整大小的控制窗口
 
# 红色范围1的滑动条（H值在0-10区间）
cv.createTrackbar('R_H_min1', 'controls', 0, 179, nothing)
cv.createTrackbar('R_H_max1', 'controls', 10, 179, nothing)
 
# 红色范围2的滑动条（H值在160-179区间，解决红色在HSV色相环首尾的问题）[7](@ref)
cv.createTrackbar('R_H_min2', 'controls', 160, 179, nothing)
cv.createTrackbar('R_H_max2', 'controls', 179, 179, nothing)
 
# 蓝色范围的滑动条（H值在100-130区间）[6](@ref)
cv.createTrackbar('B_H_min', 'controls', 100, 179, nothing)
cv.createTrackbar('B_H_max', 'controls', 130, 179, nothing)
 
# 绿色范围的滑动条（H值在35-85区间）[6](@ref)
cv.createTrackbar('G_H_min', 'controls', 35, 179, nothing)
cv.createTrackbar('G_H_max', 'controls', 85, 179, nothing)

# --- 新增：黄色范围的滑动条 (H值通常在20-35左右) ---
cv.createTrackbar('Y_H_min', 'controls', 15, 179, nothing)   # 黄色H最小值 (初始设为15)
cv.createTrackbar('Y_H_max', 'controls', 35, 179, nothing)   # 黄色H最大值 (初始设为35)
 
# 共用饱和度(S)和亮度(V)阈值的滑动条
cv.createTrackbar('S_min', 'controls', 50, 255, nothing)  # 饱和度最小值
cv.createTrackbar('V_min', 'controls', 50, 255, nothing)  # 亮度最小值
 
# 最小面积阈值滑动条（过滤小噪点）[4](@ref)
cv.createTrackbar('Min Area', 'controls', 500, 5000, nothing)
 
# 颜色标签和对应的绘图颜色（BGR格式）
color_labels = {
    'red': (0, 0, 255),    # BGR: 红色
    'blue': (255, 0, 0),   # BGR: 蓝色
    'green': (0, 255, 0),   # BGR: 绿色
    # --- 新增：黄色 ---
    'yellow': (0, 255, 255) 
}
 
# 主循环：持续处理视频帧
while True:
    # 读取当前帧
    ret, frame = cap.read()
    if not ret:  # 检查帧是否成功读取
        break
        
    # 获取所有滑动条的当前位置值
    r_min1 = cv.getTrackbarPos('R_H_min1', 'controls')  # 红色范围1 H最小值
    r_max1 = cv.getTrackbarPos('R_H_max1', 'controls')  # 红色范围1 H最大值
    r_min2 = cv.getTrackbarPos('R_H_min2', 'controls')  # 红色范围2 H最小值
    r_max2 = cv.getTrackbarPos('R_H_max2', 'controls')  # 红色范围2 H最大值
    b_min = cv.getTrackbarPos('B_H_min', 'controls')    # 蓝色范围 H最小值
    b_max = cv.getTrackbarPos('B_H_max', 'controls')    # 蓝色范围 H最大值
    g_min = cv.getTrackbarPos('G_H_min', 'controls')    # 绿色范围 H最小值
    g_max = cv.getTrackbarPos('G_H_max', 'controls')    # 绿色范围 H最大值
        # --- 新增：获取黄色滑动条数值 ---
    y_min = cv.getTrackbarPos('Y_H_min', 'controls') 
    y_max = cv.getTrackbarPos('Y_H_max', 'controls') 

    s_min = cv.getTrackbarPos('S_min', 'controls')      # 饱和度最小值
    v_min = cv.getTrackbarPos('V_min', 'controls')      # 亮度最小值
    min_area = cv.getTrackbarPos('Min Area', 'controls')  # 最小面积阈值（过滤小噪点）
 
    # 将BGR图像转换为HSV颜色空间（更适合颜色检测）[4,6](@ref)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # 为每种颜色创建掩码（二值图像，白色表示目标颜色区域）
    mask_red1 = cv.inRange(hsv, np.array([r_min1, s_min, v_min]), np.array([r_max1, 255, 255]))
    mask_red2 = cv.inRange(hsv, np.array([r_min2, s_min, v_min]), np.array([r_max2, 255, 255]))
    mask_blue = cv.inRange(hsv, np.array([b_min, s_min, v_min]), np.array([b_max, 255, 255]))
    mask_green = cv.inRange(hsv, np.array([g_min, s_min, v_min]), np.array([g_max, 255, 255]))
     # --- 新增：黄色掩码 ---
    mask_yellow = cv.inRange(hsv, np.array([y_min, s_min, v_min]), np.array([y_max, 255, 255]))

    # 合并红色掩码（处理红色在HSV色相环首尾的问题）[7](@ref)
    mask_red = cv.bitwise_or(mask_red1, mask_red2)
    
    # 形态学处理：开运算（先腐蚀后膨胀）减少噪声[3,7](@ref)
    kernel = np.ones((5, 5), np.uint8)  # 5x5矩形核
    mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
    mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)
    mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)
    # --- 新增：黄色形态学处理 ---
    mask_yellow = cv.morphologyEx(mask_yellow, cv.MORPH_OPEN, kernel)
    
    # 创建结果图像副本（用于绘制检测结果）
    result_img = frame.copy()
    
    # 处理每种颜色的轮廓
    for color_name, mask in [('blue', mask_blue), ('yellow', mask_yellow)]: # ('green', mask_green),('red', mask_red),  
        # 查找轮廓（只检测外部轮廓，使用简单近似）[4,7](@ref)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # 遍历所有轮廓
        for contour in contours:
            # 计算轮廓面积并过滤小面积轮廓（减少噪声）[4](@ref)
            if cv.contourArea(contour) < min_area:
                continue
                
            # 获取边界矩形参数（x,y为左上角坐标，w,h为宽高）
            x, y, w, h = cv.boundingRect(contour)
            # 计算中心点坐标
            center_x = int(x + w/2)
            center_y = int(y + h/2)
            
            # 绘制边界框（使用颜色对应的线条）[2,7](@ref)
            cv.rectangle(result_img, (x, y), (x+w, y+h), color_labels[color_name], 2)
            # 绘制中心点（实心圆）
            cv.circle(result_img, (center_x, center_y), 5, color_labels[color_name], -1)
            
            # 添加颜色标签（在边界框上方显示颜色名称）[7](@ref)
            cv.putText(result_img, color_name.upper(), (x, y-10), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, color_labels[color_name], 2)
 
    # 创建合并所有颜色的掩码（用于显示整体检测区域）
    mask_combined = cv.bitwise_or(mask_red, cv.bitwise_or(mask_blue, mask_green))
    # 应用掩码提取目标区域（黑色背景）
    res = cv.bitwise_and(frame, frame, mask=mask_combined)
    
    # 水平拼接三幅图像：原始帧、合并掩码（转换为BGR）、结果帧[7](@ref)
    mask_bgr = cv.cvtColor(mask_combined, cv.COLOR_GRAY2BGR)  # 单通道转三通道
    display_img = np.concatenate((frame, mask_bgr, result_img), axis=1)
    
    # 显示拼接后的图像
    cv.imshow('Color Blob Detection', display_img)
    
    # 检测ESC键按下（ASCII 27）退出循环
    if cv.waitKey(5) & 0xFF == 27: 
        break
 
# 释放摄像头资源并关闭所有窗口
cap.release()
cv.destroyAllWindows()